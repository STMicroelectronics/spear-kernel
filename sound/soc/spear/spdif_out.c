/*
 * ALSA SoC SPDIF Out Audio Layer for spear processors
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <mach/spdif_out.h>
#include "spdif_out_regs.h"

struct spdif_out_dev {
	struct device *dev;
	struct clk *clk;
	void *dma_params;
	void *io_base;
};

static int spdif_out_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct spdif_out_dev *host = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	snd_soc_dai_set_dma_data(cpu_dai, substream, host->dma_params);

	ret = clk_enable(host->clk);
	if (ret)
		return ret;

	writel(SPDIF_OUT_RESET, host->io_base + SPDIF_OUT_SOFT_RST);
	msleep(1);
	writel(readl(host->io_base + SPDIF_OUT_SOFT_RST) & ~SPDIF_OUT_RESET,
			host->io_base + SPDIF_OUT_SOFT_RST);

	writel(SPDIF_OUT_FDMA_TRIG_16 | SPDIF_OUT_MEMFMT_16_16 |
			SPDIF_OUT_VALID_HW | SPDIF_OUT_USER_HW |
			SPDIF_OUT_CHNLSTA_HW | SPDIF_OUT_PARITY_HW,
			host->io_base + SPDIF_OUT_CFG);

	writel(0x7F, host->io_base + SPDIF_OUT_INT_STA_CLR);
	writel(0x7F, host->io_base + SPDIF_OUT_INT_EN_CLR);

	return 0;
}

static void spdif_out_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct spdif_out_dev *host = snd_soc_dai_get_drvdata(dai);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	clk_disable(host->clk);
	snd_soc_dai_set_dma_data(dai, substream, NULL);
}

static int spdif_out_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct spdif_out_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 rate, ctrl, divider, spdif_core_freq;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	rate = params_rate(params);

	switch (rate) {
	case 8000:
	case 16000:
	case 32000:
	case 64000:
		/*
		 * The clock is multiplied by 10 to bring it to feasible range
		 * of frequencies for sscg
		 */
		spdif_core_freq = 64000 * 128 * 10;	/* 81.92 MHz */
		break;
	case 5512:
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		spdif_core_freq = 176400 * 128;	/* 22.5792 MHz */
		break;
	case 48000:
	case 96000:
	case 192000:
	default:
		spdif_core_freq = 192000 * 128;	/* 24.576 MHz */
		break;
	}

	clk_set_rate(host->clk, spdif_core_freq);
	divider = DIV_ROUND_CLOSEST(clk_get_rate(host->clk), (rate * 128));

	ctrl = readl(host->io_base + SPDIF_OUT_CTRL);
	ctrl &= ~SPDIF_DIVIDER_MASK;
	ctrl |= (divider << SPDIF_DIVIDER_SHIFT) & SPDIF_DIVIDER_MASK;
	writel(ctrl, host->io_base + SPDIF_OUT_CTRL);

	return 0;
}

static int spdif_out_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct spdif_out_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 ctrl;
	int ret = 0;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl = readl(host->io_base + SPDIF_OUT_CTRL);
		ctrl &= ~SPDIF_OPMODE_MASK;
		ctrl |= SPDIF_OPMODE_AUD_DATA | SPDIF_STATE_NORMAL;
		writel(ctrl, host->io_base + SPDIF_OUT_CTRL);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl = readl(host->io_base + SPDIF_OUT_CTRL);
		ctrl &= ~SPDIF_OPMODE_MASK;
		ctrl |= SPDIF_OPMODE_OFF;
		writel(ctrl, host->io_base + SPDIF_OUT_CTRL);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct snd_soc_dai_ops spdif_out_dai_ops = {
	.startup	= spdif_out_startup,
	.shutdown	= spdif_out_shutdown,
	.trigger	= spdif_out_trigger,
	.hw_params	= spdif_out_hw_params,
};

struct snd_soc_dai_driver spdif_out_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
				 SNDRV_PCM_RATE_192000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &spdif_out_dai_ops,
};

static int spdif_out_probe(struct platform_device *pdev)
{
	struct spdif_out_dev *host;
	struct spdif_out_platform_data *pdata;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	if (!devm_request_mem_region(&pdev->dev, res->start,
				resource_size(res), pdev->name)) {
		dev_warn(&pdev->dev, "Failed to get memory resourse\n");
		return -ENOENT;
	}

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host) {
		dev_warn(&pdev->dev, "kzalloc fail\n");
		return -ENOMEM;
	}

	host->io_base = devm_ioremap(&pdev->dev, res->start,
				resource_size(res));
	if (!host->io_base) {
		dev_warn(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	host->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk))
		return PTR_ERR(host->clk);

	pdata = dev_get_platdata(&pdev->dev);

	host->dev = &pdev->dev;
	host->dma_params = pdata->dma_params;

	dev_set_drvdata(&pdev->dev, host);

	ret = snd_soc_register_dai(&pdev->dev, &spdif_out_dai);
	if (ret != 0) {
		clk_put(host->clk);
		return ret;
	}

	return 0;
}

static int spdif_out_remove(struct platform_device *pdev)
{
	struct spdif_out_dev *host = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);

	clk_put(host->clk);

	return 0;
}

static struct platform_driver spdif_out_driver = {
	.probe		= spdif_out_probe,
	.remove		= spdif_out_remove,
	.driver		= {
		.name	= "spdif-out",
		.owner	= THIS_MODULE,
	},
};

static int __init spdif_out_init(void)
{
	return platform_driver_register(&spdif_out_driver);
}
module_init(spdif_out_init);

static void __exit spdif_out_exit(void)
{
	platform_driver_unregister(&spdif_out_driver);
}
module_exit(spdif_out_exit);

MODULE_AUTHOR("Vipin Kumar <vipin.kumar@st.com>");
MODULE_DESCRIPTION("SPEAr SPDIF OUT SoC Interface");
MODULE_LICENSE("GPL");
