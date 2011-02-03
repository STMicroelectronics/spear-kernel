/*
 * ALSA SoC I2S Audio Layer for ST spear13xx processor
 *
 * sound/soc/spear/spear13xx-i2s.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <mach/misc_regs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "spear13xx-pcm.h"
#include "spear13xx-i2s.h"

/* common register for all channel */
#define IER		0x000
#define IRER		0x004
#define ITER		0x008
#define CER		0x00C
#define CCR		0x010
#define RXFFR		0x014
#define TXFFR		0x018

/* I2STxRxRegisters for channel 0 */
#define LRBR0_LTHR0	0x020
#define RRBR0_RTHR0	0x024
#define RER0		0x028
#define TER0		0x02C
#define RCR0		0x030
#define TCR0		0x034
#define ISR0		0x038
#define IMR0		0x03C
#define ROR0		0x040
#define TOR0		0x044
#define RFCR0		0x048
#define TFCR0		0x04C
#define RFF0		0x050
#define TFF0		0x054

/* I2STxRxRegisters for channel 1 */
#define LRBR1_LTHR1	0x060
#define RRBR1_RTHR1	0x064
#define RER1		0x068
#define TER1		0x06C
#define RCR1		0x070
#define TCR1		0x074
#define ISR1		0x078
#define IMR1		0x07C
#define ROR1		0x080
#define TOR1		0x084
#define RFCR1		0x088
#define TFCR1		0x08C
#define RFF1		0x090
#define TFF1		0x094

/* I2STxRxRegisters for channel 2 */
#define LRBR2_LTHR2	0x0A0
#define RRBR2_RTHR2	0x0A4
#define RER2		0x0A8
#define TER2		0x0AC
#define RCR2		0x0B0
#define TCR2		0x0B4
#define ISR2		0x0B8
#define IMR2		0x0BC
#define ROR2		0x0C0
#define TOR2		0x0C4
#define RFCR2		0x0C8
#define TFCR2		0x0CC
#define RFF2		0x0D0
#define TFF2		0x0D4

/* I2STxRxRegisters for channel 3*/
#define LRBR3_LTHR3	0x0E0
#define RRBR3_RTHR3	0x0E4
#define RER3		0x0E8
#define TER3		0x0EC
#define RCR3		0x0F0
#define TCR3		0x0F4
#define ISR3		0x0F8
#define IMR3		0x0FC
#define ROR3		0x100
#define TOR3		0x104
#define RFCR3		0x108
#define TFCR3		0x10C
#define RFF3		0x110
#define TFF3		0x114

/* I2SDMARegisters */
#define RXDMA		0x01C0
#define RRXDMA		0x01C4
#define TXDMA		0x01C8
#define RTXDMA		0x01CC

/* I2SCOMPRegisters */
#define I2S_COMP_PARAM_2	0x01F0
#define I2S_COMP_PARAM_1	0x01F4
#define I2S_COMP_VERSION	0x01F8
#define I2S_COMP_TYPE		0x01FC

#define SPEAR13XX_I2S_RATES	SNDRV_PCM_RATE_48000
#define SPEAR13XX_I2S_FORMAT	SNDRV_PCM_FMTBIT_S16_LE

struct spear13xx_i2s_dev {
	void __iomem *i2s_base;
	struct resource *res;
	struct clk *clk;
	int play_irq;
	int mode;
	int capture_irq;
	struct device *dev;
	struct spear13xx_pcm_dma_params *dma_params[2];
};

void get_dma_start_addr(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct spear13xx_runtime_data *prtd = substream->runtime->private_data;
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	prtd->txdma = dev->res->start + TXDMA;
	prtd->rxdma = dev->res->start + RXDMA;

	substream->runtime->private_data = prtd;
}

static inline void i2s_write_reg(void *io_base, int reg, u32 val)
{
	__raw_writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void *io_base, int reg)
{
	return __raw_readl(io_base + reg);
}

static int
spear13xx_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
		unsigned int freq, int dir)
{
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	struct clk *sclk_clk, *src_clk;
	int ret = -EINVAL;

	sclk_clk = clk_get_sys(NULL, "i2s_sclk_clk");
	if (IS_ERR(sclk_clk)) {
		dev_err(dev->dev, "couldn't get i2s_sclk\n");
		return PTR_ERR(sclk_clk);
	}

	src_clk = clk_get_sys(NULL, "i2s_src_clk");
	if (IS_ERR(src_clk)) {
		ret = PTR_ERR(src_clk);
		dev_err(dev->dev, "couldn't get i2s_src_sclk\n");
		goto put_sclk_clk;
	}

	if (clk_set_parent(sclk_clk, src_clk))
		goto put_src_clk;

	ret = clk_enable(sclk_clk);
	if (ret < 0) {
		dev_err(dev->dev, "enable i2s_sclk fail\n");
		goto put_src_clk;
	}
	return 0;

put_src_clk:
	clk_put(src_clk);
put_sclk_clk:
	clk_put(sclk_clk);

	return ret;
}

void
i2s_start_play(struct spear13xx_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	u32 val; /*dma mode slection*/

	val = readl(PERIP_CFG);
	val &= ~0xFFFFFFFC;
	i2s_write_reg(dev->i2s_base, IER, 1);
	i2s_write_reg(dev->i2s_base, TER0, 0);
	i2s_write_reg(dev->i2s_base, TER1, 0);

	/* for 2.0 audio*/
	if (dev->mode <= 2) {
		i2s_write_reg(dev->i2s_base, CCR, 0x00);
		if (!val) {
			i2s_write_reg(dev->i2s_base, TCR0, 0x2);
			i2s_write_reg(dev->i2s_base, TFCR0, 0x07);
			i2s_write_reg(dev->i2s_base, IMR0, 0x00);
			i2s_write_reg(dev->i2s_base, TER0, 1);
		} else {
			i2s_write_reg(dev->i2s_base, TCR1, 0x2);
			i2s_write_reg(dev->i2s_base, TFCR1, 0x07);
			i2s_write_reg(dev->i2s_base, IMR1, 0x00);
			i2s_write_reg(dev->i2s_base, TER1, 1);
		}
	} else { /*audio 2.0 onwards */
		i2s_write_reg(dev->i2s_base, CCR, 0x10);
		i2s_write_reg(dev->i2s_base, TCR0, 0x5);
		i2s_write_reg(dev->i2s_base, TCR1, 0x5);

		i2s_write_reg(dev->i2s_base, TFCR0, 0x07);
		i2s_write_reg(dev->i2s_base, TFCR1, 0x07);
		i2s_write_reg(dev->i2s_base, IMR0, 0x00);
		i2s_write_reg(dev->i2s_base, IMR1, 0x00);
		i2s_write_reg(dev->i2s_base, TER0, 1);
		i2s_write_reg(dev->i2s_base, TER1, 1);
	}

	i2s_write_reg(dev->i2s_base, ITER, 1);
	i2s_write_reg(dev->i2s_base, CER, 1);
}

void
i2s_start_rec(struct spear13xx_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	u32 val; /*dma mode slection*/

	val = readl(PERIP_CFG);
	val &= ~0xFFFFFFFC;
	i2s_write_reg(dev->i2s_base, IER, 1);
	i2s_write_reg(dev->i2s_base, RER0, 0);
	i2s_write_reg(dev->i2s_base, RER1, 0);

	/* for 2.0 audio*/
	if (dev->mode <= 2) {
		i2s_write_reg(dev->i2s_base, CCR, 0x00);
		if (!val) {
			i2s_write_reg(dev->i2s_base, RCR0, 0x2);
			i2s_write_reg(dev->i2s_base, RFCR0, 0x07);
			i2s_write_reg(dev->i2s_base, IMR0, 0x00);
			i2s_write_reg(dev->i2s_base, RER0, 1);
		} else {
			i2s_write_reg(dev->i2s_base, RCR1, 0x2);
			i2s_write_reg(dev->i2s_base, RFCR1, 0x07);
			i2s_write_reg(dev->i2s_base, IMR1, 0x00);
			i2s_write_reg(dev->i2s_base, TER1, 1);
		}
	} else { /*audio 2.0 onwards */
		i2s_write_reg(dev->i2s_base, CCR, 0x10);
		i2s_write_reg(dev->i2s_base, RCR0, 0x5);
		i2s_write_reg(dev->i2s_base, RCR1, 0x5);

		i2s_write_reg(dev->i2s_base, RFCR0, 0x07);
		i2s_write_reg(dev->i2s_base, RFCR1, 0x07);
		i2s_write_reg(dev->i2s_base, IMR0, 0x00);
		i2s_write_reg(dev->i2s_base, IMR1, 0x00);
		i2s_write_reg(dev->i2s_base, RER0, 1);
		i2s_write_reg(dev->i2s_base, RER1, 1);
	}

	i2s_write_reg(dev->i2s_base, IRER, 1);
	i2s_write_reg(dev->i2s_base, CER, 1);
}

void
i2s_stop(struct spear13xx_i2s_dev *dev, struct snd_pcm_substream *substream)
{
	i2s_write_reg(dev->i2s_base, IER, 0);
	i2s_write_reg(dev->i2s_base, IMR0, 0x33);
	i2s_write_reg(dev->i2s_base, IMR1, 0x33);
	i2s_write_reg(dev->i2s_base, ITER, 0);
	i2s_write_reg(dev->i2s_base, IRER, 0);
	i2s_write_reg(dev->i2s_base, CER, 0);
}

static irqreturn_t i2s_play_irq(int irq, void *_dev)
{
	struct spear13xx_i2s_dev *dev = (struct spear13xx_i2s_dev *)_dev;
	u32 ch0, ch1;

	/* check for the tx data overrun condition */
	ch0 = i2s_read_reg(dev->i2s_base, ISR0) & 0x20;
	ch1 = i2s_read_reg(dev->i2s_base, ISR1) & 0x20;
	if (ch0 || ch1) {

		/* disable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 0);

		/* disable tx block */
		i2s_write_reg(dev->i2s_base, ITER, 0);

		/* flush all the tx fifo */
		i2s_write_reg(dev->i2s_base, TXFFR, 1);

		/* clear tx data overrun interrupt: channel 0 */
		i2s_read_reg(dev->i2s_base, TOR0);

		/* clear tx data overrun interrupt: channel 1 */
		i2s_read_reg(dev->i2s_base, TOR1);
	}

	return IRQ_HANDLED;
}

static irqreturn_t i2s_capture_irq(int irq, void *_dev)
{
	struct spear13xx_i2s_dev *dev = (struct spear13xx_i2s_dev *)_dev;
	u32 ch0, ch1;

	/* check for the rx data overrun condition */
	ch0 = i2s_read_reg(dev->i2s_base, ISR0) & 0x02;
	ch1 = i2s_read_reg(dev->i2s_base, ISR1) & 0x02;
	if (ch0 || ch1) {

		/* disable i2s block */
		i2s_write_reg(dev->i2s_base, IER, 0);

		/* disable rx block */
		i2s_write_reg(dev->i2s_base, IRER, 0);

		/* flush all the rx fifo */
		i2s_write_reg(dev->i2s_base, RXFFR, 1);

		/* clear rx data overrun interrupt: channel 0 */
		i2s_read_reg(dev->i2s_base, ROR0);

		/* clear rx data overrun interrupt: channel 1 */
		i2s_read_reg(dev->i2s_base, ROR1);
	}

	return IRQ_HANDLED;
}

static int
spear13xx_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(cpu_dai);
	u32 ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = request_irq(dev->play_irq, i2s_play_irq, 0,
				"spear13xx-i2s", dev);
	} else {
		ret = request_irq(dev->capture_irq, i2s_capture_irq,
				0, "spear13xx-i2s", dev);
	}
	if (ret) {
		dev_err(dev->dev, "irq registration failure\n");
		iounmap(dev->i2s_base);
		clk_disable(dev->clk);
		clk_put(dev->clk);
		kfree(dev);
		release_mem_region(dev->res->start, resource_size(dev->res));
		return ret;
	}
	/* unmask i2s interrupt for channel 0 */
	i2s_write_reg(dev->i2s_base, IMR0, 0x00);

	/* unmask i2s interrupt for channel 1 */
	i2s_write_reg(dev->i2s_base, IMR1, 0x00);
	snd_soc_dai_set_dma_data(cpu_dai, substream, dev->dma_params);

	return 0;
}

static int spear13xx_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	u32 channel;

	channel = params_channels(params);

	dev->mode = channel;

	return 0;
}

static void
spear13xx_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (dev->play_irq)
			free_irq(dev->play_irq, dev);
	} else {
		if (dev->capture_irq)
			free_irq(dev->capture_irq, dev);
	}

	/* mask i2s interrupt for channel 0 */
	i2s_write_reg(dev->i2s_base, IMR0, 0x33);

	/* mask i2s interrupt for channel 1 */
	i2s_write_reg(dev->i2s_base, IMR1, 0x33);

	i2s_stop(dev, substream);

}

static int
spear13xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct spear13xx_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s_start_play(dev, substream);
		else
			i2s_start_rec(dev, substream);
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		i2s_stop(dev, substream);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct snd_soc_dai_ops spear13xx_i2s_dai_ops = {
	.startup	= spear13xx_i2s_startup,
	.shutdown	= spear13xx_i2s_shutdown,
	.hw_params	= spear13xx_i2s_hw_params,
	.trigger	= spear13xx_i2s_trigger,
	.set_sysclk	= spear13xx_i2s_set_dai_sysclk,
};

struct snd_soc_dai_driver spear13xx_i2s_dai = {
	.playback = {
		.channels_min = MAX_CHANNEL_NUM,
		.channels_max = MIN_CHANNEL_NUM,
		.rates = SPEAR13XX_I2S_RATES,
		.formats = SPEAR13XX_I2S_FORMAT,
	},
	.capture = {
		.channels_min = MAX_CHANNEL_NUM,
		.channels_max = MIN_CHANNEL_NUM,
		.rates = SPEAR13XX_I2S_RATES,
		.formats = SPEAR13XX_I2S_FORMAT,
	},
	.ops = &spear13xx_i2s_dai_ops,
};

static int
spear13xx_i2s_probe(struct platform_device *pdev)
{
	struct spear13xx_i2s_dev *dev;
	struct resource *res;
	int ret;

	if (!pdev) {
		dev_err(&pdev->dev, "Invalid platform device\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no i2s resource defined\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "i2s region already claimed\n");
		return -EBUSY;
	}
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

	dev->res = res;

	dev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(dev->clk)) {
		ret = PTR_ERR(dev->clk);
		goto err_kfree;
	}

	ret = clk_enable(dev->clk);
	if (ret < 0)
		goto err_clk_put;

	dev->i2s_base = ioremap(res->start, resource_size(res));
	if (!dev->i2s_base) {
		dev_err(&pdev->dev, "ioremap fail for i2s_region\n");
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	dev->play_irq = platform_get_irq_byname(pdev, "play_irq");
	if (!dev->play_irq) {
		dev_err(&pdev->dev, "play irq not defined\n");
		ret = -EBUSY;
		goto err_iounmap_i2s;
	}

	dev->capture_irq = platform_get_irq_byname(pdev, "record_irq");
	if (!dev->capture_irq) {
		dev_err(&pdev->dev, "record irq not defined\n");
		ret = -EBUSY;
		goto err_free_play_irq;
	}

	dev->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, dev);

	ret = snd_soc_register_dai(&pdev->dev, &spear13xx_i2s_dai);
	if (ret != 0)
		goto err_free_capture_irq;

	return 0;

err_free_capture_irq:
	dev_set_drvdata(&pdev->dev, NULL);
	free_irq(dev->capture_irq, pdev);
err_free_play_irq:
	free_irq(dev->play_irq, pdev);
err_iounmap_i2s:
	iounmap(dev->i2s_base);
err_clk_disable:
	clk_disable(dev->clk);
err_clk_put:
	clk_put(dev->clk);
err_kfree:
	kfree(dev);
err_release_mem_region:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int
spear13xx_i2s_remove(struct platform_device *pdev)
{
	struct spear13xx_i2s_dev *dev = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	free_irq(dev->capture_irq, pdev);
	free_irq(dev->play_irq, pdev);
	iounmap(dev->i2s_base);
	clk_disable(dev->clk);
	clk_put(dev->clk);
	kfree(dev);
	release_mem_region(dev->res->start, resource_size(dev->res));

	return 0;
}

static struct platform_driver spear13xx_i2s_driver = {
	.probe		= spear13xx_i2s_probe,
	.remove		= spear13xx_i2s_remove,
	.driver		= {
		.name	= "spear13xx-i2s",
		.owner	= THIS_MODULE,
	},
};

static int __init spear13xx_i2s_init(void)
{
	return platform_driver_register(&spear13xx_i2s_driver);
}
module_init(spear13xx_i2s_init);

static void __exit spear13xx_i2s_exit(void)
{
	platform_driver_unregister(&spear13xx_i2s_driver);
}
module_exit(spear13xx_i2s_exit);

MODULE_AUTHOR("Rajeev Kumar");
MODULE_DESCRIPTION("SPEAr I2S SoC Interface");
MODULE_LICENSE("GPL");
