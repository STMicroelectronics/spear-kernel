/*
 * ASoC machine driver for SPEAr evaluation boards
 *
 * sound/soc/spear/spear_evb.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/hardware.h>


/* SPEAr audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear_evb_dai[] = {
	{
		.name		= "sta529-pcm",
		.stream_name	= "pcm",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	},
};

/* SPEAr audio machine driver */
static struct snd_soc_card spear_snd_card = {
	.name		= "spear-evb",
	.dai_link	= spear_evb_dai,
	.num_links	= ARRAY_SIZE(spear_evb_dai),
};

/* SPEAr320s audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear320s_evb_dai[] = {
	{
		.name		= "sta529-pcm",
		.stream_name	= "pcm",
		.cpu_dai_name	= "designware-i2s",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	},
};

/* SPEAr320s audio machine driver */
static struct snd_soc_card spear320s_snd_card = {
	.name		= "spear320s-evb",
	.dai_link	= spear320s_evb_dai,
	.num_links	= ARRAY_SIZE(spear320s_evb_dai),
};

/* LCAD audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link lcad_evb_dai[] = {
	{
		.name		= "sta529-pcm0",
		.stream_name	= "I2S Playback",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	}, {
		.name		= "sta529-pcm1",
		.stream_name	= "I2S Capture",
		.cpu_dai_name	= "designware-i2s.1",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	},
};

static struct snd_soc_card lcad_snd_card = {
	.name		= "lcad-evb",
	.dai_link	= lcad_evb_dai,
	.num_links	= ARRAY_SIZE(lcad_evb_dai),
};

/* Audio machine driver for SPEAr1340 evb */

/* SPEAr1340 audio interface glue - connects codec <--> CPU <--> platform */
static struct snd_soc_dai_link spear1340_evb_dai[] = {
	{
		.name		= "spdif-pcm0",
		.stream_name	= "SPDIF Playback",
		.cpu_dai_name	= "spdif-out",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "dit-hifi",
		.codec_name	= "spdif-dit",
		.ops		= NULL,
	}, {
		.name		= "spdif-pcm1",
		.stream_name	= "SPDIF Capture",
		.cpu_dai_name	= "spdif-in",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "dir-hifi",
		.codec_name	= "spdif-dir",
		.ops		= NULL,
	}, {
		.name		= "sta529-pcm0",
		.stream_name	= "I2S Playback",
		.cpu_dai_name	= "designware-i2s.0",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	}, {
		.name		= "sta529-pcm1",
		.stream_name	= "I2S Capture",
		.cpu_dai_name	= "designware-i2s.1",
		.platform_name	= "spear-pcm-audio",
		.codec_dai_name	= "sta529-audio",
		.codec_name	= "sta529-codec.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFM,
		.ops		= NULL,
	},
};

static struct snd_soc_card spear1340_snd_card = {
	.name		= "spear1340-evb",
	.dai_link	= spear1340_evb_dai,
	.num_links	= ARRAY_SIZE(spear1340_evb_dai),
};

static __devinit int spear_evb_probe(struct platform_device *pdev)
{
	struct snd_soc_card *spear_soc_card;
	int ret;

	if (of_machine_is_compatible("st,spear_lcad"))
		spear_soc_card = &lcad_snd_card;
	else if (of_machine_is_compatible("st,spear1340"))
		spear_soc_card = &spear1340_snd_card;
	else if (of_machine_is_compatible("st,spear320"))
		spear_soc_card = &spear320s_snd_card;
	else
		spear_soc_card = &spear_snd_card;

	spear_soc_card->dev = &pdev->dev;

	ret = snd_soc_register_card(spear_soc_card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int __devexit spear_evb_remove(struct platform_device *pdev)
{
	struct snd_soc_card *spear_soc_card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(spear_soc_card);

	return 0;
}

static struct platform_driver spear_evb_driver = {
	.driver = {
		.name = "spear-evb",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = spear_evb_probe,
	.remove = __devexit_p(spear_evb_remove),
};

module_platform_driver(spear_evb_driver);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("ST SPEAr EVB ASoC driver");
MODULE_LICENSE("GPL");
