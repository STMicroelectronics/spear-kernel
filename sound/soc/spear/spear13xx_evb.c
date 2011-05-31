/*
 * ASoC machine driver for spear platform
 *
 * sound/soc/spear/spear13xx_evb.c
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
#include <mach/hardware.h>
#include <mach/misc_regs.h>
#include "mach/spear1340_misc_regs.h"

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/sta529.h"
#include "spear13xx_pcm.h"

#define CHANNEL_MASK_M	0x00000030
#define CHANNEL_MASK_S	0x000000C0

static int
sta529_evb_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
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

	if (cpu_is_spear1340()) {
		val = readl(SPEAR1340_PERIP_CFG);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val = (val & ~CHANNEL_MASK_M) | (channel << 4);
			writel(val, SPEAR1340_PERIP_CFG);
		} else {
			val = (val & ~CHANNEL_MASK_S) | (channel << 6);
			writel(val, SPEAR1340_PERIP_CFG);
		}

	}

	else if (cpu_is_spear1300() || cpu_is_spear1310_reva() ||
			cpu_is_spear900() || cpu_is_spear1310()) {
		/*setting mode 0 in conf regiter: 32c offset*/
		val = readl(PERIP_CFG);
		val &= ~0x7;
		writel(val, PERIP_CFG);

	}

	return 0;
}

static struct snd_soc_ops sta529_evb_ops = {
	.hw_params	= sta529_evb_hw_params,
};

/* synopsys digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link evb_dai = {
	.name		= "STA529",
	.stream_name	= "STA529",
	.cpu_dai_name	= "designware-i2s.0",
	.platform_name	= "designware-pcm-audio",
	.codec_dai_name	= "sta529-audio",
	.codec_name	= "sta529-codec.0-001a",
	.ops		= &sta529_evb_ops,
};

/* synopsys audio machine driver */
static struct snd_soc_card spear_sta529 = {
	.name		= "spear_sta529",
	.dai_link	= &evb_dai,
	.num_links	= 1,
};

static struct platform_device *evb_snd_device;

static int __init dw_init(void)
{
	int ret;

	/* Create and register platform device */
	evb_snd_device = platform_device_alloc("soc-audio", 0);
	if (!evb_snd_device) {
		printk(KERN_ERR "platform_device_alloc fails\n");
		return -ENOMEM;
	}

	platform_set_drvdata(evb_snd_device, &spear_sta529);
	ret = platform_device_add(evb_snd_device);
	if (ret) {
		printk(KERN_ERR "Unable to add platform device\n");
		platform_device_put(evb_snd_device);
	}

	return ret;
}
module_init(dw_init);

static void __exit dw_exit(void)
{
	platform_device_unregister(evb_snd_device);
}
module_exit(dw_exit);

MODULE_AUTHOR("Rajeev Kumar");
MODULE_DESCRIPTION("ST SYNOPSYS EVB ASoC driver");
MODULE_LICENSE("GPL");
