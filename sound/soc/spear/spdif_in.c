/*
 * ALSA SoC SPDIF In Audio Layer for spear processors
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
#include <linux/spear_dma.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/spdif.h>
#include "spdif_in_regs.h"

struct spdif_in_dev {
	struct clk *clk;
	struct dma_data dma_params;
	void *io_base;
};

static int spdif_in_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct spdif_in_dev *host = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;
	u32 ctrl;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	snd_soc_dai_set_dma_data(cpu_dai, substream, (void *)&host->dma_params);

	ret = clk_enable(host->clk);
	if (ret)
		return ret;

	ctrl = SPDIF_IN_PRTYEN | SPDIF_IN_STATEN | SPDIF_IN_USREN |
		SPDIF_IN_VALEN;
	ctrl |= SPDIF_MODE_16BIT | SPDIF_IN_SAMPLE | SPDIF_FIFO_THRES_16;

	writel(ctrl, host->io_base + SPDIF_IN_CTRL);
	writel(0xF, host->io_base + SPDIF_IN_IRQ_MASK);

	return 0;
}

static void spdif_in_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct spdif_in_dev *host = snd_soc_dai_get_drvdata(dai);

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return;

	clk_disable(host->clk);
	snd_soc_dai_set_dma_data(dai, substream, NULL);
}

static int spdif_in_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct spdif_in_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 ctrl, format;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	format = params_format(params);

	ctrl = readl(host->io_base + SPDIF_IN_CTRL);

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ctrl |= SPDIF_XTRACT_16BIT;
		break;

	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
		ctrl &= ~SPDIF_XTRACT_16BIT;
		break;		
	}

	writel(ctrl, host->io_base + SPDIF_IN_CTRL);

	return 0;
}

static int spdif_in_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct spdif_in_dev *host = snd_soc_dai_get_drvdata(dai);
	u32 ctrl;
	int ret = 0;

	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl = readl(host->io_base + SPDIF_IN_CTRL);
		ctrl |= SPDIF_IN_ENB;
		writel(ctrl, host->io_base + SPDIF_IN_CTRL);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl = readl(host->io_base + SPDIF_IN_CTRL);
		ctrl &= ~SPDIF_IN_ENB;
		writel(ctrl, host->io_base + SPDIF_IN_CTRL);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct snd_soc_dai_ops spdif_in_dai_ops = {
	.startup	= spdif_in_startup,
	.shutdown	= spdif_in_shutdown,
	.trigger	= spdif_in_trigger,
	.hw_params	= spdif_in_hw_params,
};

struct snd_soc_dai_driver spdif_in_dai = {
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
				 SNDRV_PCM_RATE_192000),
		.formats = SNDRV_PCM_FMTBIT_S16_LE | \
			   SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},
	.ops = &spdif_in_dai_ops,
};

static int spdif_in_probe(struct platform_device *pdev)
{
	struct spdif_in_dev *host;
	struct spdif_platform_data *pdata;
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

	host->dma_params.data = pdata->dma_params;
	host->dma_params.filter = pdata->filter;

	dev_set_drvdata(&pdev->dev, host);

	ret = snd_soc_register_dai(&pdev->dev, &spdif_in_dai);
	if (ret != 0) {
		clk_put(host->clk);
		return ret;
	}

	return 0;
}

static int spdif_in_remove(struct platform_device *pdev)
{
	struct spdif_in_dev *host = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);

	clk_put(host->clk);

	return 0;
}

static struct platform_driver spdif_in_driver = {
	.probe		= spdif_in_probe,
	.remove		= spdif_in_remove,
	.driver		= {
		.name	= "spdif-in",
		.owner	= THIS_MODULE,
	},
};

static int __init spdif_in_init(void)
{
	return platform_driver_register(&spdif_in_driver);
}
module_init(spdif_in_init);

static void __exit spdif_in_exit(void)
{
	platform_driver_unregister(&spdif_in_driver);
}
module_exit(spdif_in_exit);

MODULE_AUTHOR("Vipin Kumar <vipin.kumar@st.com>");
MODULE_DESCRIPTION("SPEAr SPDIF IN SoC Interface");
MODULE_LICENSE("GPL");
