/*
 * ASoC codec driver for spear platform
 *
 * sound/soc/codecs/sta529.c -- spear ALSA Soc codec driver
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
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/misc_regs.h>
#include "sta529.h"

#define STA529_VERSION "0.1"
static const u8 sta529_reg[STA529_CACHEREGNUM] = {
	0x35, 0xc8, 0x50, 0x00,
	0x00, 0x00, 0x02, 0x00,
	0x02, 0x05, 0x12, 0x41,
	0x13, 0x41, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x80, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x52, 0x40,
	0x21, 0xef, 0x04, 0x06,
	0x41, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,
};

static const struct snd_kcontrol_new sta529_snd_controls[] = {
	SOC_SINGLE("Master Playback Volume", STA529_MVOL, 0, 127, 1),
	SOC_SINGLE("Left Playback Volume", STA529_LVOL, 0, 127, 1),
	SOC_SINGLE("Right Playback Volume", STA529_RVOL, 0, 127, 1),
};

static struct snd_soc_device *sta529_socdev;

/* reading from register cache: sta529 register value */
u8 sta529_read_reg_cache(struct snd_soc_codec *codec, u32 reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= STA529_CACHEREGNUM)
		return -EINVAL;

	return cache[reg];
}

/* write register cache : sta529 register value. */
static inline void
sta529_write_reg_cache(struct snd_soc_codec *codec, u8 reg, int value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= STA529_CACHEREGNUM)
		return;

	cache[reg] = value;
}

/* write to the sta529 register space */
static int
sta529_write(struct snd_soc_codec *codec, u32 reg, u32 value)
{
	u8 data[2];

	data[0] = reg & 0xff;
	data[1] = value & 0xff;
	sta529_write_reg_cache(codec, data[0], data[1]);
	if (codec->hw_write(codec->control_data, data, NUM_OF_MSG)
			== NUM_OF_MSG)
		return 0;
	else
		return -EIO;
}

static int
spear_sta529_set_dai_fmt(struct snd_soc_dai *codec_dai, u32 fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 mode = 0;
	int val;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		mode = LEFT_J_DATA_FORMAT;
		break;
	case SND_SOC_DAIFMT_I2S:
		mode = I2S_DATA_FORMAT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		mode = RIGHT_J_DATA_FORMAT;
		break;
	default:
		return -EINVAL;
	}

	val = sta529_read_reg_cache(codec, STA529_S2PCFG0);
	val |= mode;
	/*this setting will be used with actual h/w */
	sta529_write(codec, STA529_S2PCFG0, val);

	return 0;
}

static int
spear_sta_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
		unsigned int freq, int dir)
{
	int ret = -EINVAL;
	struct clk *clk;

	clk = clk_get_sys(NULL, "i2s_ref_clk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_clk;
	}
	if (clk_set_rate(clk, freq))
		goto err_put_clk;

	ret = clk_enable(clk);
	if (ret < 0)
		goto err_put_clk;

	return 0;

err_put_clk:
	clk_put(clk);
err_clk:
	return ret;
}

static int spear_sta529_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	u8 mute_reg = sta529_read_reg_cache(codec, STA529_FFXCFG0) &
		~CODEC_MUTE_VAL;

	if (mute)
		mute_reg |= CODEC_MUTE_VAL;

	sta529_write(codec, STA529_FFXCFG0, mute_reg);

	return 0;
}

static int
sta529_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	u16 sts;

	switch (level) {
	case SND_SOC_BIAS_ON:
		spear_sta529_mute(codec->dai, 0);
		break;
	case SND_SOC_BIAS_PREPARE:
		spear_sta529_mute(codec->dai, 1);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		sts = sta529_read_reg_cache(codec, STA529_FFXCFG0);
		sta529_write(codec, STA529_FFXCFG0, sts & POWER_STBY);

		break;
	}

	/*store the label for powers down audio subsystem for suspend.This is
	 ** used by soc core layer*/
	codec->bias_level = level;
	return 0;

}

static struct snd_soc_dai_ops sta529_dai_ops = {
	.set_fmt	= spear_sta529_set_dai_fmt,
	.digital_mute	= spear_sta529_mute,
	.set_sysclk	= spear_sta_set_dai_sysclk,
};

struct snd_soc_dai sta529_dai = {
	.name = "STA529",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SPEAR_PCM_RATES,
		.formats = SPEAR_PCM_FORMAT,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SPEAR_PCM_RATES,
		.formats = SPEAR_PCM_FORMAT,
	},
	.ops	= &sta529_dai_ops,
};
EXPORT_SYMBOL_GPL(sta529_dai);

typedef unsigned int (*codec_read)(struct snd_soc_codec *, unsigned int);
/*
 * initialise the sta529 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int sta529_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret , i ;
	u8 *cache ;
	u8 data[2];

	codec->name = "STA529";
	codec->owner = THIS_MODULE;
	codec->read = (codec_read)sta529_read_reg_cache;
	codec->write = sta529_write;
	codec->set_bias_level = sta529_set_bias_level;
	codec->dai = &sta529_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(sta529_reg);
	codec->reg_cache = kmemdup(sta529_reg, sizeof(sta529_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	cache = codec->reg_cache;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(socdev->dev, "sta529: failed to create pcms\n");
		goto pcm_err;
	}

	for (i = 0; i < ARRAY_SIZE(sta529_reg); i++) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, NUM_OF_MSG);
	}

	/* power on device */
	sta529_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	snd_soc_add_controls(codec, sta529_snd_controls,
			ARRAY_SIZE(sta529_snd_controls));
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(socdev->dev, "sta529: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);

	return ret;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int
sta529_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = sta529_socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	i2c_set_clientdata(i2c, codec);
	codec->dev = &i2c->dev;
	codec->control_data = i2c;

	ret = sta529_init(socdev);
	if (ret < 0)
		dev_err(socdev->dev, "failed to initialise sta529\n");

	return ret;
}

static int sta529_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);

	kfree(codec->reg_cache);

	return 0;
}

static const struct i2c_device_id sta529_i2c_id[] = {
	{ "sta529", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sta529_i2c_id);

static struct i2c_driver sta529_i2c_driver = {
	.driver = {
		.name = "STA529_I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe		= sta529_i2c_probe,
	.remove		= sta529_i2c_remove,
	.id_table	= sta529_i2c_id,
};

static int
sta529_add_i2c_device(struct platform_device *pdev,
		const struct sta529_setup_data *setup)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;

	ret = i2c_add_driver(&sta529_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "sta529", I2C_NAME_SIZE);

	adapter = i2c_get_adapter(setup->i2c_bus);
	if (!adapter) {
		dev_err(&pdev->dev, "can't get i2c adapter %d\n",
				setup->i2c_bus);
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		dev_err(&pdev->dev, "can't add i2c device at 0x%x\n",
				(unsigned int)info.addr);
		goto err_driver;
	}
	i2c_put_adapter(adapter);

	return 0;

err_driver:
	i2c_del_driver(&sta529_i2c_driver);
	return -ENODEV;
}
#endif

static int spear_sta529_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct sta529_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	dev_info(&pdev->dev, "spear Audio Codec %s", STA529_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	sta529_socdev = socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		codec->hw_write = (hw_write_t) i2c_master_send;
		ret = sta529_add_i2c_device(pdev, setup);
	}
	if (ret != 0)
		kfree(codec);
#endif
	return ret;
}

/* power down chip */
static int spear_sta529_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		sta529_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (codec->control_data)
		i2c_unregister_device(codec->control_data);
	i2c_del_driver(&sta529_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

static int spear_sta529_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	sta529_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int spear_sta529_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	for (i = 0; i < ARRAY_SIZE(sta529_reg); i++) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, NUM_OF_MSG);
	}

	sta529_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	sta529_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sta529 = {
	.probe = spear_sta529_probe,
	.remove = spear_sta529_remove,
	.suspend = spear_sta529_suspend,
	.resume = spear_sta529_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_sta529);

static int __init sta529_modinit(void)
{
	return snd_soc_register_dai(&sta529_dai);
}
module_init(sta529_modinit);

static void __exit sta529_exit(void)
{
	snd_soc_unregister_dai(&sta529_dai);
}
module_exit(sta529_exit);

MODULE_DESCRIPTION("Soc STA529 driver");
MODULE_AUTHOR("Rajeev Kumar");
MODULE_LICENSE("GPL");
