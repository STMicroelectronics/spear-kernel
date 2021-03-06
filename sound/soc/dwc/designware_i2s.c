/*
 * ALSA SoC Synopsys I2S Audio Layer
 *
 * sound/soc/spear/designware_i2s.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar <rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <sound/designware_i2s.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* common register for all channel */
#define IER		0x000
#define IRER		0x004
#define ITER		0x008
#define CER		0x00C
#define CCR		0x010
#define RXFFR		0x014
#define TXFFR		0x018

/* I2STxRxRegisters for all channels */
#define LRBR_LTHR(x)	(0x40 * x + 0x020)
#define RRBR_RTHR(x)	(0x40 * x + 0x024)
#define RER(x)		(0x40 * x + 0x028)
#define TER(x)		(0x40 * x + 0x02C)
#define RCR(x)		(0x40 * x + 0x030)
#define TCR(x)		(0x40 * x + 0x034)
#define ISR(x)		(0x40 * x + 0x038)
#define IMR(x)		(0x40 * x + 0x03C)
#define ROR(x)		(0x40 * x + 0x040)
#define TOR(x)		(0x40 * x + 0x044)
#define RFCR(x)		(0x40 * x + 0x048)
#define TFCR(x)		(0x40 * x + 0x04C)
#define RFF(x)		(0x40 * x + 0x050)
#define TFF(x)		(0x40 * x + 0x054)

/* I2SCOMPRegisters */
#define I2S_COMP_PARAM_2	0x01F0
#define I2S_COMP_PARAM_1	0x01F4
#define I2S_COMP_VERSION	0x01F8
#define I2S_COMP_TYPE		0x01FC

/* Tx and Rx Configuration Register (TCR and RCR) */
#define WLEN_16BIT		(0x2 << 0)
#define WLEN_24BIT		(0x4 << 0)
#define WLEN_32BIT		(0x5 << 0)

/* Tx FIFO Configuration Register (TFCR) */
#define TFCR_EMPTY_TRIGGER	0x2

/* Rx FIFO Configuration Register (RFCR) */
#define RFCR_FULL_TRIGGER	0x7

/* Interrupt Register (ISR and IMR) */
#define TX_FIFO_OVERRUN		(0x1 << 5)
#define TX_FIFO_EMPTY		(0x1 << 4)
#define RX_FIFO_OVERRUN		(0x1 << 1)
#define RX_DATA_AVAILABLE	(0x1 << 0)

/* Clock Configuration Register (CCR) */
#define CCR_WS_MASK		(0x3 << 3)
#define CCR_WS_16SCLK		(0x0 << 3)
#define CCR_WS_24SCLK		(0x1 << 3)
#define CCR_WS_32SCLK		(0x2 << 3)

#define MAX_CHANNEL_NUM		8
#define MIN_CHANNEL_NUM		2
#define DMA_TX_BURST_LEN	16
#define DMA_RX_BURST_LEN	16

struct dw_i2s_dev {
	void __iomem *i2s_base;
	struct clk *clk;
	int active;
	unsigned int capability;
	struct device *dev;

	/* data related to DMA transfers b/w i2s and DMAC */
	struct i2s_dma_data play_dma_data;
	struct i2s_dma_data capture_dma_data;
	struct i2s_hw_config_data config;
	int (*i2s_clk_cfg)(struct i2s_hw_config_data *config);
};

static inline void i2s_write_reg(void *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void *io_base, int reg)
{
	return readl(io_base + reg);
}

static inline void i2s_enable_channels(struct dw_i2s_dev *dev, u32 stream)
{
	int i, channels;

	channels = dev->config.chan_nr / 2;
	for (i = 0; i < channels; i++) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s_write_reg(dev->i2s_base, TER(i), 1);
		else
			i2s_write_reg(dev->i2s_base, RER(i), 1);
	}
}

static inline void i2s_disable_all_channels(struct dw_i2s_dev *dev, u32 stream)
{
	int i, channels;

	channels = MAX_CHANNEL_NUM / 2;
	for (i = 0; i < channels; i++) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s_write_reg(dev->i2s_base, TER(i), 0);
		else
			i2s_write_reg(dev->i2s_base, RER(i), 0);
	}
}

static inline void i2s_clear_irqs(struct dw_i2s_dev *dev, u32 stream)
{
	int i, channels;

	channels = dev->config.chan_nr / 2;
	for (i = 0; i < channels; i++) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s_read_reg(dev->i2s_base, TOR(i));
		else
			i2s_read_reg(dev->i2s_base, ROR(i));
	}
}

static inline void i2s_enable_irqs(struct dw_i2s_dev *dev, u32 mask)
{
	int i, channels;
	u32 val;

	channels = dev->config.chan_nr / 2;
	for (i = 0; i < channels; i++) {
		val = i2s_read_reg(dev->i2s_base, IMR(i));
		i2s_write_reg(dev->i2s_base, IMR(i), val & ~mask);
	}
}

static inline void i2s_disable_irqs(struct dw_i2s_dev *dev, u32 mask)
{
	int i, channels;
	u32 val;

	channels = dev->config.chan_nr / 2;
	for (i = 0; i < channels; i++) {
		val = i2s_read_reg(dev->i2s_base, IMR(i));
		i2s_write_reg(dev->i2s_base, IMR(i), val | mask);
	}
}

static int dw_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	struct i2s_dma_data *dma_data = NULL;

	if (!(dev->capability & DWC_I2S_RECORD) &&
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE))
		return -EINVAL;

	if (!(dev->capability & DWC_I2S_PLAY) &&
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK))
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &dev->play_dma_data;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &dev->capture_dma_data;

	snd_soc_dai_set_dma_data(cpu_dai, substream, (void *)dma_data);

	return 0;
}

static int dw_i2s_hw_config(struct dw_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	struct i2s_hw_config_data *config = &dev->config;
	u32 clock_config, xfer_resolution;
	int i, ret, channels;

	switch (config->data_width) {
	case 16:
		clock_config = CCR_WS_16SCLK;
		xfer_resolution = WLEN_16BIT;
		break;
	case 24:
		clock_config = CCR_WS_24SCLK;
		xfer_resolution = WLEN_24BIT;
		break;
	case 32:
		clock_config = CCR_WS_32SCLK;
		xfer_resolution = WLEN_32BIT;
		break;
	default:
		dev_err(dev->dev, "unsupported format");
		return -EINVAL;

	}

	i2s_disable_all_channels(dev, substream->stream);

	channels = config->chan_nr / 2;
	for (i = 0; i < channels; i++) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			i2s_write_reg(dev->i2s_base, TCR(i), xfer_resolution);
			i2s_write_reg(dev->i2s_base, TFCR(i), TFCR_EMPTY_TRIGGER);
		} else {
			i2s_write_reg(dev->i2s_base, RCR(i), xfer_resolution);
			i2s_write_reg(dev->i2s_base, RFCR(i),
					RFCR_FULL_TRIGGER);
		}
	}

	i2s_enable_channels(dev, substream->stream);

	i2s_write_reg(dev->i2s_base, CCR, clock_config);

	if (dev->i2s_clk_cfg) {
		ret = dev->i2s_clk_cfg(config);
		if (ret < 0) {
			dev_err(dev->dev, "runtime audio clk config fail\n");
			return ret;
		}
	}

	return 0;
}
static int dw_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	struct i2s_hw_config_data *config = &dev->config;

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			config->data_width = 16;
			dev->play_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_2_BYTES;
			dev->capture_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_2_BYTES;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			config->data_width = 24;
			dev->play_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_2_BYTES;
			dev->capture_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_2_BYTES;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			config->data_width = 32;
			dev->play_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_4_BYTES;
			dev->capture_dma_data.addr_width =
				DMA_SLAVE_BUSWIDTH_4_BYTES;
			break;
		default:
			dev_err(dev->dev, "unsupported PCM fmt");
			return -EINVAL;
	}

	config->chan_nr = params_channels(params);
	config->sample_rate = params_rate(params);

	return 0;
}

void i2s_start(struct dw_i2s_dev *dev, struct snd_pcm_substream *substream)
{
	/* Disable I2S */
	i2s_write_reg(dev->i2s_base, IER, 0);

	/* Program I2S */
	dw_i2s_hw_config(dev, substream);

	/* Enable I2S */
	i2s_write_reg(dev->i2s_base, IER, 1);

	/* Enable Tx or Rx blocks */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		i2s_enable_irqs(dev, TX_FIFO_OVERRUN | TX_FIFO_EMPTY);
		i2s_write_reg(dev->i2s_base, ITER, 1);
	} else {
		i2s_enable_irqs(dev, RX_FIFO_OVERRUN | RX_DATA_AVAILABLE);
		i2s_write_reg(dev->i2s_base, IRER, 1);
	}

	/* Enable clock generation */
	i2s_write_reg(dev->i2s_base, CER, 1);
}

static void i2s_stop(struct dw_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		i2s_write_reg(dev->i2s_base, ITER, 0);
		i2s_disable_irqs(dev, TX_FIFO_OVERRUN | TX_FIFO_EMPTY);
	} else {
		i2s_write_reg(dev->i2s_base, IRER, 0);
		i2s_disable_irqs(dev, RX_FIFO_OVERRUN | RX_DATA_AVAILABLE);
	}

	i2s_clear_irqs(dev, substream->stream);

	if (!dev->active) {
		i2s_write_reg(dev->i2s_base, CER, 0);
		i2s_write_reg(dev->i2s_base, IER, 0);
	}
}

static void dw_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	snd_soc_dai_set_dma_data(dai, substream, NULL);
}

static int dw_i2s_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev->active++;
		i2s_start(dev, substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev->active--;
		i2s_stop(dev, substream);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct snd_soc_dai_ops dw_i2s_dai_ops = {
	.startup	= dw_i2s_startup,
	.shutdown	= dw_i2s_shutdown,
	.hw_params	= dw_i2s_hw_params,
	.trigger	= dw_i2s_trigger,
};

#ifdef CONFIG_PM

static int dw_i2s_suspend(struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	clk_disable_unprepare(dev->clk);
	return 0;
}

static int dw_i2s_resume(struct snd_soc_dai *dai)
{
	struct dw_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	clk_prepare_enable(dev->clk);
	return 0;
}

#else
#define dw_i2s_suspend	NULL
#define dw_i2s_resume	NULL
#endif

static int dw_i2s_probe(struct platform_device *pdev)
{
	const struct i2s_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	struct dw_i2s_dev *dev;
	struct resource *res;
	int ret, channel;
	unsigned int cap = 0;
	struct snd_soc_dai_driver *dw_i2s_dai;

	if (!pdata) {
		dev_err(&pdev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no i2s resource defined\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
				resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "i2s region already claimed\n");
		return -EBUSY;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_warn(&pdev->dev, "kzalloc fail\n");
		return -ENOMEM;
	}

	dev->i2s_base = devm_ioremap(&pdev->dev, res->start,
			resource_size(res));
	if (!dev->i2s_base) {
		dev_err(&pdev->dev, "ioremap fail for i2s_region\n");
		return -ENOMEM;
	}

	if (of_get_property(np, "play", NULL))
		cap |= DWC_I2S_PLAY;
	if (of_get_property(np, "record", NULL))
		cap |= DWC_I2S_RECORD;

	dev->capability = cap;
	dev->i2s_clk_cfg = pdata->i2s_clk_cfg;

	/* Set DMA slaves info */

	dev->play_dma_data.data = pdata->play_dma_data;
	dev->capture_dma_data.data = pdata->capture_dma_data;
	dev->play_dma_data.addr = res->start + I2S_TXDMA;
	dev->capture_dma_data.addr = res->start + I2S_RXDMA;
	dev->play_dma_data.max_burst = DMA_TX_BURST_LEN;
	dev->capture_dma_data.max_burst = DMA_RX_BURST_LEN;
	dev->play_dma_data.filter = pdata->filter;
	dev->capture_dma_data.filter = pdata->filter;

	dev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dev->clk))
		return PTR_ERR(dev->clk);

	ret = clk_prepare_enable(dev->clk);
	if (ret < 0)
		goto err_clk_put;

	dw_i2s_dai = devm_kzalloc(&pdev->dev, sizeof(*dw_i2s_dai), GFP_KERNEL);
	if (!dw_i2s_dai) {
		dev_err(&pdev->dev, "mem allocation failed for dai driver\n");
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	if (cap & DWC_I2S_PLAY) {
		dev_dbg(&pdev->dev, " SPEAr: play supported\n");
		dw_i2s_dai->playback.channels_min = MIN_CHANNEL_NUM;
		of_property_read_u32(np, "channel", &channel);
		dw_i2s_dai->playback.channels_max = channel;
		dw_i2s_dai->playback.formats = pdata->snd_fmts;
		dw_i2s_dai->playback.rates = pdata->snd_rates;
	}

	if (cap & DWC_I2S_RECORD) {
		dev_dbg(&pdev->dev, "SPEAr: record supported\n");
		dw_i2s_dai->capture.channels_min = MIN_CHANNEL_NUM;
		of_property_read_u32(np, "channel", &channel);
		dw_i2s_dai->capture.channels_max = channel;
		dw_i2s_dai->capture.formats = pdata->snd_fmts;
		dw_i2s_dai->capture.rates = pdata->snd_rates;
	}

	dw_i2s_dai->ops = &dw_i2s_dai_ops;
	dw_i2s_dai->suspend = dw_i2s_suspend;
	dw_i2s_dai->resume = dw_i2s_resume;

	dev->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, dev);
	ret = snd_soc_register_dai(&pdev->dev, dw_i2s_dai);
	if (ret != 0) {
		dev_err(&pdev->dev, "not able to register dai\n");
		goto err_set_drvdata;
	}

	if (pdata && pdata->clk_init)
		pdata->clk_init();

	return 0;

err_set_drvdata:
	dev_set_drvdata(&pdev->dev, NULL);
err_clk_disable:
	clk_disable_unprepare(dev->clk);
err_clk_put:
	clk_put(dev->clk);
	return ret;
}

static int dw_i2s_remove(struct platform_device *pdev)
{
	struct dw_i2s_dev *dev = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);

	clk_put(dev->clk);

	return 0;
}

static const struct of_device_id dwc_i2s_of_match[]  = {
	{ .compatible = "snps,designware-i2s", },
	{},
};
MODULE_DEVICE_TABLE(of, dwc_i2s_of_match);

static struct platform_driver dw_i2s_driver = {
	.probe		= dw_i2s_probe,
	.remove		= dw_i2s_remove,
	.driver		= {
		.name	= "designware-i2s",
		.owner	= THIS_MODULE,
		.of_match_table = dwc_i2s_of_match,
	},
};

module_platform_driver(dw_i2s_driver);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("DESIGNWARE I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:designware_i2s");
