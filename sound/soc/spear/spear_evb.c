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
#include <sound/spear_dma.h>
#include <mach/hardware.h>


struct snd_soc_card spear_soc_card;

static __devinit int spear_evb_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_link *evb_dai;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *spear_np, *codec_np;
	int i, nr_controllers, ret = 0;

	if (!np)
		return -EINVAL; /* no device tree */

	ret = spear_pcm_platform_register(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err;
	}

	of_property_read_u32(np, "nr_controllers", &nr_controllers);

	evb_dai = devm_kzalloc(&pdev->dev,
			(nr_controllers * sizeof(*evb_dai)), GFP_KERNEL);
	if (!evb_dai) {
		dev_warn(&pdev->dev, "kzalloc fail for evb_dai\n");
		return -ENOMEM;
	}

	for (i = 0; i < nr_controllers ; i++) {
		spear_np = of_parse_phandle(np, "audio-controllers", i);
		codec_np = of_parse_phandle(np, "audio-codecs", i);
		if (!spear_np || !codec_np) {
			dev_err(&pdev->dev, "phandle missing or invalid\n");
			return -EINVAL;
		}

		ret = of_property_read_string_index(np, "dai_name", i,
				&evb_dai[i].name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Cannot parse names: %d\n", ret);
			goto err;
		}

		ret = of_property_read_string_index(np, "codec_dai_name", i,
				&evb_dai[i].codec_dai_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Cannot parse codec-dai-name: %d\n",
					ret);
			goto err;
		}
		ret = of_property_read_string_index(np, "stream_name", i,
				&evb_dai[i].stream_name);
		if (ret < 0) {
			dev_err(&pdev->dev, "Cannot parse stream names: %d\n",
					ret);
			goto err;
		}

		evb_dai[i].cpu_dai_name = NULL;
		evb_dai[i].cpu_dai_of_node = spear_np;
		evb_dai[i].platform_name = "sound.6";
		evb_dai[i].codec_name = NULL;
		evb_dai[i].codec_of_node = codec_np;
		if (!(strcmp(evb_dai[i].codec_dai_name, "sta529-audio")))
			evb_dai[i].dai_fmt = SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_CBS_CFM;
		of_node_put(spear_np);
		of_node_put(codec_np);
	}

	spear_soc_card.name = "spear-evb";
	spear_soc_card.dai_link = evb_dai;
	spear_soc_card.num_links = nr_controllers;

	spear_soc_card.dev = &pdev->dev;
	platform_set_drvdata(pdev, &spear_soc_card);

	ret = snd_soc_register_card(&spear_soc_card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);
		goto err;
	}

	return 0;
err:
	return ret;
}

static int __devexit spear_evb_remove(struct platform_device *pdev)
{
	struct snd_soc_card *spear_soc_card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(spear_soc_card);
	spear_pcm_platform_unregister(pdev);
	return 0;
}

static const struct of_device_id spear_evb_dt_ids[] = {
	{ .compatible = "spear,spear-evb", },
	{ }
};
MODULE_DEVICE_TABLE(of, spear_evb_dt_ids);

static struct platform_driver spear_evb_driver = {
	.driver = {
		.name = "spear-evb",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = spear_evb_dt_ids,
	},
	.probe = spear_evb_probe,
	.remove = __devexit_p(spear_evb_remove),
};

module_platform_driver(spear_evb_driver);

MODULE_AUTHOR("Rajeev Kumar <rajeev-dlh.kumar@st.com>");
MODULE_DESCRIPTION("ST SPEAr EVB ASoC driver");
MODULE_LICENSE("GPL");
