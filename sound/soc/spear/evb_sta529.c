/*
 * ASoC machine driver for spear platform
 *
 * sound/soc/spear/evb_sta529.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

#include <mach/generic.h>
#include <mach/misc_regs.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/sta529.h"
#include "spear13xx-i2s.h"
#include "spear13xx-pcm.h"

static int
sta529_evb_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	u32 freq, format, rate, channel;
	u32 ref_clock, val;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_CBS_CFM);
	if (ret < 0)
		return ret;

	format = params_format(params);
	rate = params_rate(params);
	channel = params_channels(params);
	freq = format * rate * channel * 8;
	ref_clock = freq * 8;

	/* set the codec system clock for DAC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0 , ref_clock,
			SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, freq, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/*setting ref clock in 278 offset*/
	val = readl(PERIP2_CLK_ENB);
	val |= 0x80;
	writel(val, PERIP2_CLK_ENB);

	/*setting mode 0 in conf regiter: 32c offset*/
	val = readl(PERIP_CFG);
	val |= 0x0;
	writel(val, PERIP_CFG);

	return 0;
}

static struct snd_soc_ops sta529_evb_ops = {
	.hw_params	= sta529_evb_hw_params,
};

/* spear digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link evb_dai = {
	.name		= "SPEARSTA529",
	.stream_name	= "STA529",
	.cpu_dai	= &spear13xx_i2s_dai,
	.codec_dai	= &sta529_dai,
	.ops		= &sta529_evb_ops,
};

/* spear audio machine driver */
static struct snd_soc_card snd_soc_sta529 = {
	.name		= "spearevb",
	.platform	= &spear13xx_soc_platform,
	.dai_link	= &evb_dai,
	.num_links	= 1,
};

/* evb audio private data */
static struct sta529_setup_data evb_sta529_setup = {
	.i2c_bus	= 0,
	.i2c_address	= 0x1a,
};

/* spear audio subsystem */
static struct snd_soc_device evb_sta529_snd_devdata = {
	.card		= &snd_soc_sta529,
	.codec_dev	= &soc_codec_dev_sta529,
	.codec_data	= &evb_sta529_setup,
};

static struct platform_device *evb_snd_device;

static int __init spear_init(void)
{
	int ret;

	evb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!evb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(evb_snd_device, &evb_sta529_snd_devdata);
	evb_sta529_snd_devdata.dev = &evb_snd_device->dev;
	ret = platform_device_add(evb_snd_device);
	if (ret)
		platform_device_put(evb_snd_device);

	return ret;
}
module_init(spear_init);

static void __exit spear_exit(void)
{
	platform_device_unregister(evb_snd_device);
}
module_exit(spear_exit);

MODULE_AUTHOR("Rajeev Kumar");
MODULE_DESCRIPTION("ST SPEAR EVB ASoC driver");
MODULE_LICENSE("GPL");
