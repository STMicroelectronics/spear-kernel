/*
 * V4L2 SoC Camera driver for Hynix HI704 Camera Sensor
 *
 * Copyright (C) 2011 ST Microelectronics
 * Bhavna Yadav <bhavna.yadav@st.com>
 *
 * Based on ST VS6725 sensor
 * Copyright (C) 2011 ST Microelectronics 
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Based on MT9M001 CMOS Image Sensor from Micron
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-subdev.h>

/* supported resolutions */
#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QVGATYPE1_WIDTH		320
#define QVGATYPE1_HEIGHT	240
#define QVGATYPE2_WIDTH		QVGATYPE1_WIDTH
#define QVGATYPE2_HEIGHT	QVGATYPE1_HEIGHT
#define QQVGATYPE1_WIDTH	160
#define QQVGATYPE1_HEIGHT	120
#define QQVGATYPE2_WIDTH	QQVGATYPE1_WIDTH
#define QQVGATYPE2_HEIGHT	QQVGATYPE1_HEIGHT

/* max supported frame rate */
#define MAX_FRAME_RATE	30

/* hi704 cropping windows params */
#define HI704_MAX_WIDTH		VGA_WIDTH
#define HI704_MAX_HEIGHT	VGA_HEIGHT
#define HI704_MIN_WIDTH		0
#define HI704_MIN_HEIGHT	0
#define HI704_COLUMN_SKIP	3
#define HI704_ROW_SKIP		3

/* Default Values for Various Controls */
#define DEF_BRIGHTNESS		3
#define DEF_ISO			0
#define DEF_AWB			0
#define DEF_EFFECT		0
#define DEF_ZOOM		0
#define DEF_FLIP		0
#define DEF_SCENE		0
#define DEF_METERING_EXPOSURE	0
#define DEF_FLASH		0

/*
 * user-interface registers (firmware)
 */

/* Hi704 device parameter registers */
#define DEVICE_ID			0x0004
	#define HI704_DEVICE_ID		0x96

/* HI704 Page Mode Register */
#define PAGE_MODE			0x03

/* Image output format and Image Effect Registers */
#define PAGE_10				0x10
#define ISPCTL1				0x10
	#define DATA_FORMAT_YUV422		(0 << 4)
	#define DATA_FORMAT_RGB565		(4 << 4)
	#define DATA_FORMAT_RGB444		(7 << 4)
	#define DATA_FORMAT_MASK		(0xf << 4)
	#define DATA_FORMAT_UYPHASE_MASK	(3 << 0)
	#define DATA_FORMAT_UPHASE		(1 << 0)
	#define DATA_FORMAT_YPHASE		(1 << 1)

#define ISPCTL2				0x11
#define ISPCTL3				0x12
#define ISPCTL4				0x13
#define ISPCTL5				0x14

/* DeviceID and IMG size and Windowing and Sync (Page Mode = 0) */
#define PAGE_0				0x00
#define PWRCTL				0x01
	#define POWER_SLEEP			BIT(0)
#define VDOCTL1				0x10
#define VDOCTL2				0x11
	#define	HFLIP				BIT(0)
	#define	VFLIP				BIT(1)
#define SYNCCTL				0x12
	#define CLOCK_INV			BIT(2)
	#define HSYNC_POLARITY			BIT(4)
	#define VSYNC_POLARITY			BIT(5)
#define	WIN_ROW_HIGH			0x20
#define WIN_ROW_LOW			0x21
#define WIN_COL_HIGH			0x22
#define WIN_COL_LOW			0x23
#define WIN_HEIGHT_HIGH			0x24
#define WIN_HEIGHT_LOW			0x25
#define WIN_WIDTH_HIGH			0x26
#define	WIN_WIDTH_LOW			0x27
#define HORI_BLANK_HIGH			0x40
#define HORI_BLANK_LOW			0x41
#define VSYNC_HIGH			0X42
#define VSYNC_LOW			0X43
#define	VSYNC_CLIP			0x44
#define VSYNC_CTRL1			0x45
#define VSYNC_CTRL2			0x46
#define	VSYNC_CTRL3			0x47

struct hi704 {
	struct v4l2_subdev subdev;
	struct v4l2_rect rect;		/* sensor cropping window */
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
	struct soc_camera_link *icl;

	int contrast;
	int brightness;
	int gain;
	int awb;			/* automatic white balance */
	int aec;			/* automatic exposure control */
	int color_effect;		/* special color effect */
	int gamma;
	bool vflip;
	bool hflip;
	int model;			/* ident codes from v4l2-chip-ident.h */
};

/* useful for an array of registers */
struct hi704_reg {
	u8 reg;
#define REG_TERM	0x00
	u8 val;
#define VAL_TERM	0x00
};

struct capture_size {
	unsigned long width;
	unsigned long height;
};

#define HI704_DEF_WINDOW	0	/* VGA is the default image size */
struct capture_size hi704_windows[] = {
	{VGA_WIDTH, VGA_HEIGHT},		/* VGA */
	{QVGATYPE1_WIDTH, QVGATYPE1_HEIGHT},	/* QVGA with FR 30 */
	{QVGATYPE2_WIDTH, QVGATYPE2_HEIGHT},	/* QVGA with FR 60 */
	{QQVGATYPE1_WIDTH, QQVGATYPE1_HEIGHT},	/* QQVGA with FR 30 */
	{QQVGATYPE2_WIDTH, QQVGATYPE2_HEIGHT},	/* QQVGA with FR 60 */
};

static struct hi704_reg hi704_window_vga[] = {
	{0x03, 0x00}, {0x10, 0x00}, {0x20, 0x00}, {0x21, 0x02}, {0x22, 0x00},
	{0x23, 0x02}, {0x24, 0x01}, {0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_window_qvgatype1[] = {
	{0x03, 0x00}, {0x10, 0x10}, {0x20, 0x00}, {0x21, 0x02}, {0x22, 0x00},
	{0x23, 0x02}, {0x24, 0x01}, {0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_window_qvgatype2[] = {
	{0x03, 0x00}, {0x10, 0x01}, {0x20, 0x00}, {0x21, 0x02}, {0x22, 0x00},
	{0x23, 0x02}, {0x24, 0x01}, {0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_window_qqvgatype1[] = {
	{0x03, 0x00}, {0x10, 0x20}, {0x20, 0x00}, {0x21, 0x02}, {0x22, 0x00},
	{0x23, 0x02}, {0x24, 0x01}, {0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_window_qqvgatype2[] = {
	{0x03, 0x00}, {0x10, 0x21}, {0x20, 0x00}, {0x21, 0x02}, {0x22, 0x00},
	{0x23, 0x02}, {0x24, 0x01}, {0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80},
	{REG_TERM, VAL_TERM}
};

struct hi704_reg *hi704_reg_window[] = {
	hi704_window_vga,
	hi704_window_qvgatype1,
	hi704_window_qvgatype2,
	hi704_window_qqvgatype1,
	hi704_window_qqvgatype2,
};

static struct hi704_reg hi704_brightness_0[] = {
	{0x03, 0x10},
	{0x12, 0x30},
	{0x40, 0xa0},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_brightness_1[] = {
	{0x03, 0x10},
	{0x12, 0x30},
	{0x40, 0x90},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_brightness_2[] = {
	{0x03, 0x10},
	{0x12, 0x30},
	{0x40, 0x80},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_brightness_3[] = {
	{0x03, 0x10},
	{0x12, 0x30},
	{0x40, 0x10},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_brightness_4[] = {
	{0x03, 0x10},
	{0x12, 0x30},
	{0x40, 0x20},
	{REG_TERM, VAL_TERM}
};

/* Array of Sensor Brightness */
struct hi704_reg *hi704_reg_brightness[] = {
	hi704_brightness_0,
	hi704_brightness_1,
	hi704_brightness_2,
	hi704_brightness_3,
	hi704_brightness_4
};

/* Auto White Balance : Auto */
static struct hi704_reg hi704_awb_auto[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x83, 0x52},
	{0x84, 0x1B},
	{0x85, 0x50},
	{0x86, 0x25},
	{0x03, 0x22},
	{0x10, 0xea},
	{REG_TERM, VAL_TERM}
};

/* Auto White Balance : DayLight */
static struct hi704_reg hi704_awb_daylight[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x80, 0x40},
	{0x81, 0x20},
	{0x82, 0x28},
	{0x83, 0x45},
	{0x84, 0x35},
	{0x85, 0x2d},
	{0x86, 0x1c},
	{REG_TERM, VAL_TERM}
};

/* Auto White Balance: Incandescent */
static struct hi704_reg hi704_awb_incandescent[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x80, 0x26},
	{0x81, 0x20},
	{0x82, 0x55},
	{0x83, 0x24},
	{0x84, 0x1e},
	{0x85, 0x58},
	{0x86, 0x4a},
	{REG_TERM, VAL_TERM}
};

/* Auto White Balance: Fluorescent */
static struct hi704_reg hi704_awb_fluorescent[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x80, 0x35},
	{0x81, 0x20},
	{0x82, 0x32},
	{0x83, 0x3c},
	{0x84, 0x2c},
	{0x85, 0x45},
	{0x86, 0x35},
	{REG_TERM, VAL_TERM}
};

/* Auto White Balance: Cloudy */
static struct hi704_reg hi704_awb_cloudy[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x80, 0x50},
	{0x81, 0x20},
	{0x82, 0x24},
	{0x83, 0x6d},
	{0x84, 0x65},
	{0x85, 0x24},
	{0x86, 0x1c},
	{REG_TERM, VAL_TERM}
};

/* Auto White Balance: Sunset */
static struct hi704_reg hi704_awb_sunset[] = {
	{0x03, 0x22},
	{0x10, 0x6a},
	{0x80, 0x33},
	{0x81, 0x20},
	{0x82, 0x3d},
	{0x83, 0x2e},
	{0x84, 0x24},
	{0x85, 0x43},
	{0x86, 0x3d},
	{REG_TERM, VAL_TERM}
};

/* Array of Auto White Balance type*/
struct hi704_reg *hi704_reg_awb[6] = {
	hi704_awb_auto,
	hi704_awb_daylight,
	hi704_awb_incandescent,
	hi704_awb_fluorescent,
	hi704_awb_cloudy,
	hi704_awb_sunset
};

static struct hi704_reg hi704_iso_auto[] = {
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_iso_100[] = {
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_iso_200[] = {
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_iso_400[] = {
	{REG_TERM, VAL_TERM}
};

struct hi704_reg *hi704_reg_iso[4] = {
	hi704_iso_auto,
	hi704_iso_100,
	hi704_iso_200,
	hi704_iso_400
};

/* Hi704 Effect: Normal */
static struct hi704_reg hi704_effect_normal[] = {
	{0x03, 0x10},
	{0x11, 0x43},
	{0x12, 0x30},
	{0x13, 0x00},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Effect: gray */
static struct hi704_reg hi704_effect_gray[] = {
	{0x03, 0x10},
	{0x11, 0x43},
	{0x12, 0x33},
	{0x13, 0x00},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Effect: negative */
static struct hi704_reg hi704_effect_negative[] = {
	{0x03, 0x10},
	{0x11, 0x43},
	{0x12, 0x38},
	{0x13, 0x00},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Effect: speia */
static struct hi704_reg hi704_effect_sepia[] = {
	{0x03, 0x10},
	{0x11, 0x43},
	{0x12, 0x33},
	{0x13, 0x00},
	{0x44, 0x25},
	{0x45, 0xa6},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Effect: sharpness */
static struct hi704_reg hi704_effect_sharpness[] = {
	{0x03, 0x10},
	{0x11, 0x63},
	{0x12, 0x33},
	{0x13, 0x02},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Effect: sketch */
static struct hi704_reg hi704_effect_sketch[] = {
	{0x03, 0x10},
	{0x11, 0x53},
	{0x12, 0x38},
	{0x13, 0x02},
	{0x44, 0x80},
	{0x45, 0x80},
	{0x47, 0x7f},
	{REG_TERM, VAL_TERM}
};

/* Hi704 Array Effect */
struct hi704_reg *hi704_reg_effect[6] = {
	hi704_effect_normal,
	hi704_effect_gray,
	hi704_effect_negative,
	hi704_effect_sepia,
	hi704_effect_sharpness,
	hi704_effect_sketch,
};

/* Hi704 Scene: Auto */
static struct hi704_reg hi704_scene_auto[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Scene: Night */
static struct hi704_reg hi704_scene_night[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Scene: Landscape */
static struct hi704_reg hi704_scene_landscape[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Scene: Potrait */
static struct hi704_reg hi704_scene_portrait[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Scene: Sport */
static struct hi704_reg hi704_scene_sport[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Array of Scene */
struct hi704_reg *hi704_reg_scene[5] = {
	hi704_scene_auto,
	hi704_scene_night,
	hi704_scene_landscape,
	hi704_scene_portrait,
	hi704_scene_sport
};

/* Hi704 Metering Exposure: Mtrix */
static struct hi704_reg hi704_me_mtrix[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Metering Exposure : Weighted */
static struct hi704_reg hi704_me_center_weighted[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Metering Exposure: Spot */
static struct hi704_reg hi704_me_spot[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Array Metering Exposure */
struct hi704_reg *hi704_reg_metering_exposure[3] = {
	hi704_me_mtrix,
	hi704_me_center_weighted,
	hi704_me_spot,
};

/* Hi704 Alpha: Single */
static struct hi704_reg hi704_af_single[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Alpha: Manual */
static struct hi704_reg hi704_af_manual[] = {
	{REG_TERM, VAL_TERM}
};

/* Hi704 Array of Alpha */
struct hi704_reg *hi704_reg_af[2] = {
	hi704_af_single,
	hi704_af_manual,
};

/*
 * formats supported by hi704
 */
struct hi704_format {
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;
};

/* YUYV8 is the default fallback format */
#define HI704_DEF_FORMAT	0
static struct hi704_format hi704_formats[] = {
	{
		.mbus_code = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code = V4L2_MBUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code = V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code = V4L2_MBUS_FMT_BGR565_2X8_BE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	}, {
		.mbus_code = V4L2_MBUS_FMT_RGB565_2X8_BE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	}, {
		.mbus_code = V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	}
};

/* default HI704 format */
static struct v4l2_mbus_framefmt hi704_default_fmt = {
	.width = HI704_MAX_WIDTH - HI704_MIN_WIDTH,
	.height = HI704_MAX_HEIGHT - HI704_MIN_HEIGHT,
	.code = V4L2_MBUS_FMT_YUYV8_2X8,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_JPEG,
};

/* controls supported by HI704 */
static const struct v4l2_queryctrl hi704_controls[] = {
	{
		.id		= V4L2_CID_CONTRAST,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Contrast",
		.minimum	= 0,
		.maximum	= 0xc8,
		.step		= 1,
		.default_value	= 0x73,
	},
	{
		.id		= V4L2_CID_SATURATION,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Saturation",
		.minimum	= 0,
		.maximum	= 0xc8,
		.step		= 1,
		.default_value	= 0x76,
	},
	{
		.id		= V4L2_CID_BRIGHTNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Brightness",
		.minimum	= 0,
		.maximum	= 0xc8,
		.step		= 1,
		.default_value	= 0x64,
	},
	{
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain", /* Peaking Gain */
		.minimum	= 0,
		.maximum	= 0x1f,
		.step		= 1,
		.default_value	= 0x0f,
	},
	{
		.id		= V4L2_CID_AUTO_WHITE_BALANCE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "AWB",
		.minimum	= 0, /* off */
		.maximum	= 1, /* on */
		.step		= 1,
		.default_value	= 1,
	},
	{
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "AEC",
		.minimum	= 0, /* automatic mode */
		.maximum	= 3, /* flashgun mode */
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_GAMMA,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gamma",
		.minimum	= 0,
		.maximum	= 0x1f,
		.step		= 1,
		.default_value	= 0x14,
	},
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_COLORFX,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Color Effect",
		.minimum	= 0, /* none */
		.maximum	= 9, /* vivid */
		.step		= 1,
		.default_value	= 0,
	},
};

static int hi704_prog_default(struct i2c_client *client);

/* read a register */
static int hi704_reg_read(struct i2c_client *client, int reg, u8 *val)
{
	int ret, retry = 0;
	u8 data = reg;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &data,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};
	do {
		ret = i2c_transfer(client->adapter, msg, 2);
	} while (retry++ < 3 && ret != 2);

	if (ret != 2) {
		dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a register */
static int hi704_reg_write(struct i2c_client *client, int reg, u8 val)
{
	int ret, retry = 0;
	u8 data[2] = {reg, val};
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = &data[0],
	};
	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		udelay(100);
	} while (retry++ < 3 && ret != 1);
	if (ret != 1) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a set of sequential registers */
static int hi704_reg_write_multiple(struct i2c_client *client,
					struct hi704_reg *reg_list)
{
	int ret;

	while (reg_list->reg != REG_TERM) {
		ret = hi704_reg_write(client, reg_list->reg,
						reg_list->val);
		if (ret < 0)
			return ret;

		reg_list++;
	}

	/* wait for the changes to actually happen */
	mdelay(20);

	return 0;
}

static struct hi704 *to_hi704(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct hi704, subdev);
}

/* Alter bus settings on camera side */
static int hi704_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	int ret;
	u8 temp;

	flags = soc_camera_apply_sensor_flags(icl, flags);

	ret = hi704_reg_write(client, PAGE_MODE, PAGE_0);
	if (!ret)
		ret = hi704_reg_read(client, SYNCCTL, &temp);
	if (ret)
		return ret;

	if (flags & SOCAM_PCLK_SAMPLE_RISING)
		temp |= CLOCK_INV;
	else
		temp &= ~CLOCK_INV;

	if (flags & SOCAM_HSYNC_ACTIVE_HIGH)
		temp &= ~HSYNC_POLARITY;
	else
		temp |= HSYNC_POLARITY;

	if (flags & SOCAM_VSYNC_ACTIVE_HIGH)
		temp |= VSYNC_POLARITY;
	else
		temp &= ~VSYNC_POLARITY;

	ret = hi704_reg_write(client, SYNCCTL, temp);

	return ret;
}

/* Request bus settings on camera side */
static unsigned long hi704_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_MASTER |
		SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

/*
 * HI704 is powered OFF through this sequence
 * 1. toggle PWRCTL register bit1 (low->high)
 *
 * HI704 is powered ON through this sequence
 * 1. toggle PWRCTL register bit1 (high->low)
 * 2. set registers for NORMAL opertion
 */
static int hi704_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv;
	int ret;

	if (!client)
		/*
		 * HI704 probe has not been invoked as of now,
		 * so fake a correct reply
		 */
		return 0;

	priv = to_hi704(client);

	if (!priv || !priv->icl)
		/*
		 * HI704 probe has not been invoked as of now,
		 * so fake a correct reply
		 */
		return 0;

	ret = hi704_reg_write(client, PWRCTL, 0x81);
	if (!ret)
		ret = hi704_reg_write(client, PWRCTL, 0x83);
	if (!ret)
		ret = hi704_reg_write(client, PWRCTL, 0x81);
	if (ret) {
		dev_err(&client->dev, "HI704 power ON failed\n");
		return ret;
	}

	if (on) {
		/* program sensor specific patches/quirks */
		ret = hi704_prog_default(client);
		if (ret) {
			dev_err(&client->dev,
				"HI704 default register program failed\n");
			return ret;
		}
	}
	return 0;

}

/* get the current settings of a control on hi704 */
static int hi704_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = priv->brightness;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = priv->awb;
		break;

	case V4L2_CID_COLORFX:
		ctrl->value = priv->color_effect;
		break;
	case V4L2_CID_VFLIP:
		ctrl->value = priv->vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->value = priv->hflip;
		break;

	default:
		/* we don't support this ctrl id */
		return -EINVAL;
	}
	return 0;
}

/* set some particular settings of a control on HI704 */
static int hi704_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	u8 temp;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value >= ARRAY_SIZE(hi704_reg_brightness))
			ctrl->value = ARRAY_SIZE(hi704_reg_brightness) - 1;

		ret = hi704_reg_write_multiple(client,
				hi704_reg_brightness[ctrl->value]);
		if (!ret)
			priv->brightness = ctrl->value;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		if (ctrl->value >= ARRAY_SIZE(hi704_reg_awb))
			ctrl->value = ARRAY_SIZE(hi704_reg_awb) - 1;

		ret = hi704_reg_write_multiple(client,
				hi704_reg_awb[ctrl->value]);
		if (!ret)
			priv->awb = ctrl->value;
		break;

	case V4L2_CID_COLORFX:
		if (ctrl->value >= ARRAY_SIZE(hi704_reg_effect))
			ctrl->value = ARRAY_SIZE(hi704_reg_effect) - 1;

		ret = hi704_reg_write_multiple(client,
				hi704_reg_effect[ctrl->value]);
		if (!ret)
			priv->color_effect = ctrl->value;
		break;
	case V4L2_CID_VFLIP:
		ret = hi704_reg_write(client, PAGE_MODE, PAGE_0);
		if (!ret)
			ret = hi704_reg_read(client, VDOCTL2, &temp);
		if (ret)
			return ret;
		if (ctrl->value)
			ret = hi704_reg_write(client, VDOCTL2, temp | VFLIP);
		if (!ret)
			priv->vflip = ctrl->value;
		break;
	case V4L2_CID_HFLIP:
		ret = hi704_reg_write(client, PAGE_MODE, PAGE_0);
		if (!ret)
			ret = hi704_reg_read(client, VDOCTL2, &temp);
		if (ret)
			return ret;
		if (ctrl->value)
			ret = hi704_reg_write(client, VDOCTL2, temp | HFLIP);
		if (!ret)
			priv->hflip = ctrl->value;
		break;

	default:
		/* we don't support this ctrl id */
		return -EINVAL;
	}

	return ret;
}

static int hi704_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident = priv->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int hi704_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	u8 val;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*
	 * if the register number is greater than the max supported
	 * report error
	 */
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR ||
			reg->reg > LAST_REGISTER_ADDR)
		return -EINVAL;
	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 1;

	ret = hi704_reg_read(client, reg->reg, &val);
	if (!ret)
		reg->val = (u64)val;

	return ret;
}

static int hi704_set_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*
	 * if the register number is greater than the max supported
	 * report error
	 */
	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR ||
			reg->reg > LAST_REGISTER_ADDR)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	return hi704_reg_write(client, reg->reg, reg->val);
}
#endif

/* start/stop streaming from the device */
static int hi704_s_stream(struct v4l2_subdev *sd, int enable)
{
	u8 pwrctl;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = hi704_reg_read(client, PWRCTL, &pwrctl);
	if (ret)
		return ret;

	if (enable)
		ret = hi704_reg_write(client, PWRCTL, pwrctl & ~POWER_SLEEP);
	else
		ret = hi704_reg_write(client, PWRCTL, pwrctl | POWER_SLEEP);

	if (ret)
		return ret;

	return 0;
}

/* Get streaming parameters */
static int hi704_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct  hi704 *priv = to_hi704(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = priv->frame_rate.numerator;
	cp->timeperframe.denominator = priv->frame_rate.denominator;

	return 0;
}

/* Set streaming parameters */
static int hi704_s_parm(struct v4l2_subdev *sd,
				struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct  hi704 *priv = to_hi704(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0
			|| (tpf->denominator > tpf->numerator * MAX_FRAME_RATE)) {
		/* reset to max frame rate */
		tpf->numerator = 1;
		tpf->denominator = MAX_FRAME_RATE;
	}
	priv->frame_rate.numerator = tpf->numerator;
	priv->frame_rate.denominator = tpf->denominator;

	return 0;
}

/* get the format we are currently capturing in */
static int hi704_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);

	*mf = priv->fmt;

	return 0;
}

/*
 * Try the format provided by application, without really making any
 * hardware settings.
 */
static int hi704_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(hi704_formats); index++)
		if (hi704_formats[index].mbus_code == mf->code)
			break;

	if (index >= ARRAY_SIZE(hi704_formats))
		/* fallback to the default format */
		index = 0;

	mf->code = hi704_formats[index].mbus_code;
	mf->colorspace = hi704_formats[index].colorspace;
	mf->field = V4L2_FIELD_NONE;

	/* check for upper resolution boundary */
	if (mf->width > HI704_MAX_WIDTH)
		mf->width = HI704_MAX_WIDTH;

	if (mf->height > HI704_MAX_HEIGHT)
		mf->height = HI704_MAX_HEIGHT;

	return 0;
}

/* set sensor image format */
static int hi704_set_image_format(struct i2c_client *client,
					struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	u8 ispctl1;

	ret = hi704_reg_write(client, PAGE_MODE, PAGE_10);
	if (ret)
		return ret;

	ret = hi704_reg_read(client, ISPCTL1, &ispctl1);
	if (ret)
		return ret;

	ispctl1 &= ~DATA_FORMAT_MASK;
	ispctl1 &= ~DATA_FORMAT_UYPHASE_MASK;

	switch (mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		ispctl1 |= DATA_FORMAT_YUV422;

		switch (mf->code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			ispctl1 |= DATA_FORMAT_UPHASE;
			ispctl1 |= DATA_FORMAT_YPHASE;
			break;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			ispctl1 |= DATA_FORMAT_YPHASE;
			break;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			ispctl1 |= DATA_FORMAT_UPHASE;
			break;
		case V4L2_MBUS_FMT_VYUY8_2X8:
		default:
			break;
		}
		break;

	case V4L2_MBUS_FMT_BGR565_2X8_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		ispctl1 |= DATA_FORMAT_RGB565;

		switch (mf->code) {
		case V4L2_MBUS_FMT_BGR565_2X8_BE:
			ispctl1 |= DATA_FORMAT_UPHASE;
			break;
		case V4L2_MBUS_FMT_RGB565_2X8_BE:
		default:
			break;
		}
		break;

	case V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE:
		ispctl1 |= DATA_FORMAT_RGB444;
		break;

	default:
		break;
	}

	ret = hi704_reg_write(client, ISPCTL1, ispctl1);

	return ret;
}

/* set sensor image size */
static int hi704_set_window(struct i2c_client *client,
					struct v4l2_mbus_framefmt *mf)
{
	struct capture_size *cs = hi704_windows;
	int i;

	for (i = 0; i < ARRAY_SIZE(hi704_windows); i++)
		if ((cs[i].width == mf->width) && (cs[i].width == mf->width))
			break;

	if (i >= ARRAY_SIZE(hi704_windows))
		i = HI704_DEF_WINDOW;

	return hi704_reg_write_multiple(client, hi704_reg_window[i]);
}

/* set the format we will capture in */
static int hi704_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int ret;

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);

	ret = hi704_try_fmt(sd, mf);

	/* set image format */
	if (!ret)
		ret = hi704_set_image_format(client, mf);

	/* set image size */
	if (!ret)
		ret = hi704_set_window(client, mf);

	if (!ret)
		priv->fmt = *mf;

	return ret;
}

static int hi704_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(hi704_formats))
		return -EINVAL;

	*code = hi704_formats[index].mbus_code;
	return 0;
}

static int hi704_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret;

	/* turn off CE pin of camera sensor */
	ret = hi704_s_power(sd, 0);

	return ret;
}

static int hi704_resume(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi704 *priv = to_hi704(client);
	int ret;
	u8 pwrctl;

	/* turn on CE pin of camera sensor */
	ret = hi704_s_power(sd, 1);

	/*
	 * As per the last format set before we went for suspend,
	 * reprogram sensor image size and image format
	 */
	if (!ret)
		ret = hi704_set_image_format(client, &priv->fmt);

	if (!ret)
		ret = hi704_set_window(client, &priv->fmt);

	if (!ret)
		ret = hi704_reg_read(client, PWRCTL, &pwrctl);

	if (!ret)
		ret = hi704_reg_write(client, PWRCTL, pwrctl & ~POWER_SLEEP);

	return ret;
}

static struct soc_camera_ops hi704_ops = {
	.suspend = hi704_suspend,
	.resume	= hi704_resume,
	.set_bus_param = hi704_set_bus_param,
	.query_bus_param = hi704_query_bus_param,
	.controls = hi704_controls,
	.num_controls = ARRAY_SIZE(hi704_controls),
};

static struct v4l2_subdev_core_ops hi704_subdev_core_ops = {
	.s_power = hi704_s_power,
	.g_ctrl = hi704_g_ctrl,
	.s_ctrl = hi704_s_ctrl,
	.g_chip_ident = hi704_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = hi704_get_register,
	.s_register = hi704_set_register,
#endif
};

static struct v4l2_subdev_video_ops hi704_video_ops = {
	.s_stream = hi704_s_stream,
	.s_parm = hi704_s_parm,
	.g_parm = hi704_g_parm,
	.g_mbus_fmt = hi704_g_fmt,
	.s_mbus_fmt = hi704_s_fmt,
	.try_mbus_fmt = hi704_try_fmt,
	.enum_mbus_fmt = hi704_enum_fmt,
};

static struct v4l2_subdev_ops hi704_subdev_ops = {
	.core = &hi704_subdev_core_ops,
	.video = &hi704_video_ops,
};

static struct hi704_reg hi704_initialize[] = {
	/* Page Mode 0 */
	{0x03, 0x00}, {0x10, 0x00}, {0x11, 0x90}, {0x12, 0x20},
	{0x20, 0x00}, {0x21, 0x04}, {0x22, 0x00}, {0x23, 0x04},
	{0x40, 0x01}, {0x41, 0x58}, {0x42, 0x00}, {0x43, 0x14},
	/* Black Level Calibration */
	{0x80, 0x2e}, {0x81, 0x7e}, {0x82, 0x90}, {0x83, 0x30},
	{0x84, 0x2c}, {0x85, 0x4b}, {0x89, 0x48}, {0x90, 0x0b},
	{0x91, 0x0b}, {0x92, 0x48}, {0x93, 0x48}, {0x98, 0x38},
	{0x99, 0x40}, {0xa0, 0x00}, {0xa8, 0x40},

	/* Page Mode 2 */
	{0x03, 0x02}, {0x13, 0x40}, {0x14, 0x04}, {0x1a, 0x00},
	{0x1b, 0x08}, {0x20, 0x33}, {0x21, 0xaa}, {0x22, 0xa7},
	{0x23, 0x32}, {0x3b, 0x48}, {0x50, 0x21}, {0x52, 0xa2},
	{0x53, 0x0a}, {0x54, 0x30}, {0x55, 0x10}, {0x56, 0x0c},
	{0x59, 0x0F}, {0x60, 0xca}, {0x61, 0xdb}, {0x62, 0xca},
	{0x63, 0xda}, {0x64, 0xca}, {0x65, 0xda}, {0x72, 0xcb},
	{0x73, 0xd8}, {0x74, 0xcb}, {0x75, 0xd8}, {0x80, 0x02},
	{0x81, 0xbd}, {0x82, 0x24}, {0x83, 0x3e}, {0x84, 0x24},
	{0x85, 0x3e}, {0x92, 0x72}, {0x93, 0x8c}, {0x94, 0x72},
	{0x95, 0x8c}, {0xa0, 0x03}, {0xa1, 0xbb}, {0xa4, 0xbb},
	{0xa5, 0x03}, {0xa8, 0x44}, {0xa9, 0x6a}, {0xaa, 0x92},
	{0xab, 0xb7}, {0xb8, 0xc9}, {0xb9, 0xd0}, {0xbc, 0x20},
	{0xbd, 0x28}, {0xc0, 0xDE}, {0xc1, 0xEC}, {0xc2, 0xDE},
	{0xc3, 0xEC}, {0xc4, 0xE0}, {0xc5, 0xEA}, {0xc6, 0xE0},
	{0xc7, 0xEa}, {0xc8, 0xe1}, {0xc9, 0xe8}, {0xca, 0xe1},
	{0xcb, 0xe8}, {0xcc, 0xe2}, {0xcd, 0xe7}, {0xce, 0xe2},
	{0xcf, 0xe7}, {0xd0, 0xc8}, {0xd1, 0xef},

	/* Image Output Format and Image Effect (Page Mode 0x10) */
	{0x03, 0x10}, {0x10, 0x01}, {0x11, 0x43}, {0x12, 0x30},
	{0x40, 0x80}, {0x41, 0x00}, {0x48, 0x90}, {0x50, 0x48},
	{0x60, 0x7f}, {0x61, 0x00}, {0x62, 0x9a}, {0x63, 0x9a},
	{0x64, 0x48}, {0x66, 0x90}, {0x67, 0x9e},

	/* Z-LPF (Page Mode 0x11) */
	{0x03, 0x11}, {0x10, 0x25}, {0x11, 0x1f}, {0x20, 0x00},
	{0x21, 0x38}, {0x23, 0x0a}, {0x60, 0x10}, {0x61, 0x82},
	{0x62, 0x00}, {0x63, 0x83}, {0x64, 0x83}, {0x67, 0xF0},
	{0x68, 0x30}, {0x69, 0x10},

	/* YC-LPF, B-LPF, Dead Pixel Cancellation (Page Mode 0x12) */
	{0x03, 0x12}, {0x40, 0xe9}, {0x41, 0x09}, {0x50, 0x18},
	{0x51, 0x24}, {0x70, 0x1f}, {0x71, 0x00}, {0x72, 0x00},
	{0x73, 0x00}, {0x74, 0x10}, {0x75, 0x10}, {0x76, 0x20},
	{0x77, 0x80}, {0x78, 0x88}, {0x79, 0x18}, {0xb0, 0x7d},

	/* Edge Enhancement (Page Mode 0x13) */
	{0x03, 0x13}, {0x10, 0x01}, {0x11, 0x89}, {0x12, 0x14},
	{0x13, 0x19}, {0x14, 0x08}, {0x20, 0x07}, {0x21, 0x07},
	{0x23, 0x30}, {0x24, 0x33}, {0x25, 0x08}, {0x26, 0x18},
	{0x27, 0x00}, {0x28, 0x08}, {0x29, 0x90}, {0x2a, 0xe0},
	{0x2b, 0x10}, {0x2c, 0x28}, {0x2d, 0x40}, {0x2e, 0x00},
	{0x2f, 0x00}, {0x30, 0x11}, {0x80, 0x03}, {0x81, 0x07},
	{0x90, 0x07}, {0x91, 0x07}, {0x92, 0x00}, {0x93, 0x20},
	{0x94, 0x42}, {0x95, 0x60},

	/* Lens Shading (Page Mode 0x14) */
	{0x03, 0x14}, {0x10, 0x01}, {0x20, 0x60}, {0x21, 0x80},
	{0x22, 0x66}, {0x23, 0x50}, {0x24, 0x44},

	/* Color Correction (Page Mode 0x15) */
	{0x03, 0x15}, {0x10, 0x03}, {0x14, 0x3c}, {0x16, 0x2c},
	{0x17, 0x2f}, {0x30, 0xcb}, {0x31, 0x61}, {0x32, 0x16},
	{0x33, 0x23}, {0x34, 0xce}, {0x35, 0x2b}, {0x36, 0x01},
	{0x37, 0x34}, {0x38, 0x75}, {0x40, 0x87}, {0x41, 0x18},
	{0x42, 0x91}, {0x43, 0x94}, {0x44, 0x9f}, {0x45, 0x33},
	{0x46, 0x00}, {0x47, 0x94}, {0x48, 0x14},

	/* Gamma Correction (Page Mode 0x16) */
	{0x03, 0x16}, {0x30, 0x00}, {0x31, 0x0a}, {0x32, 0x1b},
	{0x33, 0x2e}, {0x34, 0x5c}, {0x35, 0x79}, {0x36, 0x97},
	{0x37, 0xa7}, {0x38, 0xb5}, {0x39, 0xc3}, {0x3a, 0xcf},
	{0x3b, 0xe1}, {0x3c, 0xf2}, {0x3d, 0xff}, {0x3e, 0xff},

	/* Auto Flicker Cancellation (Page Mode 0x17) */
	{0x03, 0x17}, {0xc4, 0x3c}, {0xc5, 0x32},

	/* Auto Exposure (Page Mode 0x20) */
	{0x03, 0x20}, {0x10, 0x0c}, {0x11, 0x04}, {0x20, 0x01},
	{0x28, 0x27}, {0x29, 0xa1}, {0x2a, 0xf0}, {0x2b, 0x34},
	{0x2c, 0x2b}, {0x30, 0xf8}, {0x39, 0x22}, {0x3a, 0xde},
	{0x3b, 0x22}, {0x3c, 0xde}, {0x60, 0x95}, {0x68, 0x3c},
	{0x69, 0x64}, {0x6A, 0x28}, {0x6B, 0xc8}, {0x70, 0x48},
	{0x76, 0x22}, {0x77, 0x02}, {0x78, 0x12}, {0x79, 0x27},
	{0x7a, 0x23}, {0x7c, 0x1d}, {0x7d, 0x22}, {0x83, 0x00},
	{0x84, 0xaf}, {0x85, 0xc8}, {0x86, 0x00}, {0x87, 0xfa},
	{0x88, 0x02}, {0x89, 0x49}, {0x8a, 0xf0}, {0x8b, 0x3a},
	{0x8c, 0x98}, {0x8d, 0x30}, {0x8e, 0xd4}, {0x91, 0x04},
	{0x92, 0x91}, {0x93, 0xec}, {0x94, 0x01}, {0x95, 0xb7},
	{0x96, 0x74}, {0x98, 0x8C}, {0x99, 0x23}, {0x9c, 0x0b},
	{0x9d, 0xb8}, {0x9e, 0x00}, {0x9f, 0xfa}, {0xb1, 0x14},
	{0xb2, 0x50}, {0xb4, 0x14}, {0xb5, 0x38}, {0xb6, 0x26},
	{0xb7, 0x20}, {0xb8, 0x1d}, {0xb9, 0x1b}, {0xba, 0x1a},
	{0xbb, 0x19}, {0xbc, 0x19}, {0xbd, 0x18}, {0xc0, 0x1a},
	{0xc3, 0xd8}, {0xc4, 0xd8},

	/* Auto White Balance (Page Mode 0x22) */
	{0x03, 0x22}, {0x10, 0xe2}, {0x11, 0x26}, {0x20, 0x34},
	{0x21, 0x40}, {0x30, 0x80}, {0x31, 0x80}, {0x38, 0x12},
	{0x39, 0x33}, {0x40, 0xf0}, {0x41, 0x55}, {0x42, 0x33},
	{0x43, 0xf3}, {0x44, 0x88}, {0x45, 0x66}, {0x46, 0x02},
	{0x80, 0x3a}, {0x81, 0x20}, {0x82, 0x40}, {0x83, 0x52},
	{0x84, 0x1b}, {0x85, 0x50}, {0x86, 0x25}, {0x87, 0x4d},
	{0x88, 0x38}, {0x89, 0x3e}, {0x8a, 0x29}, {0x8b, 0x02},
	{0x8d, 0x22}, {0x8e, 0x71}, {0x8f, 0x63}, {0x90, 0x60},
	{0x91, 0x5c}, {0x92, 0x56}, {0x93, 0x52}, {0x94, 0x4c},
	{0x95, 0x36}, {0x96, 0x31}, {0x97, 0x2e}, {0x98, 0x2a},
	{0x99, 0x29}, {0x9a, 0x26}, {0x9b, 0x09}, {0x03, 0x22},
	{0x10, 0xfb},

	/* Page Mode 0x20 */
	{0x03, 0x20}, {0x10, 0x9c}, {0x01, 0x80},

	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_set_preview[] = {

	{0x01, 0x81}, {0x03, 0x20}, {0x10, 0x1c}, {0x03, 0x22},
	{0x10, 0x69},

	/* Page Mode = 0 */
	{0x03, 0x00}, {0x11, 0x90}, {0x12, 0x20}, {0x20, 0x00},
	{0x21, 0x04}, {0x22, 0x00}, {0x23, 0x04}, {0x24, 0x01},
	{0x25, 0xe0}, {0x26, 0x02}, {0x27, 0x80}, {0x40, 0x01},
	{0x41, 0x58}, {0x42, 0x00}, {0x43, 0x14}, {0x03, 0x20},
	{0x18, 0x38},

	/*Auto Exposure (Page Mode = 20) */
	{0x83, 0x02}, {0x84, 0xbf}, {0x85, 0x20}, {0x86, 0x00},
	{0x87, 0xfa}, {0x88, 0x02}, {0x89, 0xbf}, {0x8a, 0x20},
	{0x8B, 0x3a}, {0x8C, 0x98}, {0x8D, 0x30}, {0x8E, 0xd4},
	{0x9c, 0x06}, {0x9d, 0xd6}, {0x9e, 0x00}, {0x9f, 0xfa},
	{0x18, 0x30}, {0x03, 0x20}, {0x2b, 0x34}, {0x30, 0x78},

	/* Black Level Calibration (Page Mode = 0) */
	{0x03, 0x00}, {0x90, 0x0c}, {0x91, 0x0c}, {0x92, 0x78},
	{0x93, 0x70}, {0x01, 0x80}, {0x03, 0x20}, {0x10, 0x9c},
	{0x03, 0x22}, {0x10, 0xe9},
	{REG_TERM, VAL_TERM}
};

static struct hi704_reg hi704_set_capture[] = {
	{0x01, 0x81}, {0x03, 0x20}, {0x10, 0x1c}, {0x03, 0x22},
	{0x10, 0x69}, {0x03, 0x00},

	/* Image Size and windowing and sync (Page Mode = 0)*/
	{0x11, 0x90}, {0x12, 0x20}, {0x20, 0x00}, {0x21, 0x04},
	{0x22, 0x00}, {0x23, 0x04}, {0x24, 0x01}, {0x25, 0xe0},
	{0x26, 0x02}, {0x27, 0x80}, {0x40, 0x01},
	/* Hblank 344 Vblank 20 */
	{0x41, 0x58}, {0x42, 0x00}, {0x43, 0x14}, {0x03, 0x20},

	/* Auto Exposure (Page Mode = 20)*/
	{0x18, 0x38}, {0x83, 0x02}, /* EXP Normal 33.33 fps */
	{0x84, 0xbf}, {0x85, 0x20}, {0x86, 0x00}, /*EXPMin 6000.00 fps*/
	{0x87, 0xfa}, {0x88, 0x02}, /*EXP Max 8.33 fps */
	{0x89, 0xbf}, {0x8a, 0x20}, {0x8B, 0x3a}, /* EXP100 */
	{0x8C, 0x98}, {0x8D, 0x30}, /* EXP120 */
	{0x8E, 0xd4}, {0x9c, 0x06}, /*EXP Limit 857.14 fps */
	{0x9d, 0xd6}, {0x9e, 0x00}, /* EXP Unit */
	{0x9f, 0xfa}, {0x18, 0x30},
	/*AntiBand Unlock*/
	{0x03, 0x20}, {0x2b, 0x34}, {0x30, 0x78},

	/* Black Level Black Level Calibration (Page Mode = 0)*/
	{0x03, 0x00},
	{0x90, 0x0c}, /* BLC_TIME_TH_ON*/
	{0x91, 0x0c}, /* BLC_TIME_TH_OFF */
	{0x92, 0x78}, /* BLC_AG_TH_ON */
	{0x93, 0x70}, /* BLC_AG_TH_OFF*/

	{0x01, 0x80}, {0x03, 0x20}, {0x10, 0x9c}, {0x03, 0x22},
	{0x10, 0xe9},

	{REG_TERM, VAL_TERM}
};

/*
 * program default register values:
 */
static int hi704_prog_default(struct i2c_client *client)
{
	int ret = 0;

	ret = hi704_reg_write_multiple(client, hi704_initialize);
	if (!ret)
		ret = hi704_reg_write_multiple(client, hi704_set_capture);
	if (!ret)
		ret = hi704_reg_write_multiple(client, hi704_set_preview);
	return ret;
}

static int hi704_camera_init(struct soc_camera_device *icd,
				struct i2c_client *client)
{
	int ret = 0;
	unsigned char dev_id;
	struct hi704 *priv = to_hi704(client);

	/*
	 * check and show device id, firmware version and patch version
	 */
	ret = hi704_reg_read(client, DEVICE_ID, &dev_id);
	if (ret)
		return ret;

	if (dev_id != HI704_DEVICE_ID) {
		dev_err(&client->dev, "Device ID error 0x%02x\n", dev_id);
		return -ENODEV;
	}

	priv->model = dev_id;
	dev_info(&client->dev,
		"HI704 Device-ID=0x%02x\n", dev_id);

	return 0;
}

static int hi704_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct hi704 *priv;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &hi704_subdev_ops);

	icd->ops = &hi704_ops;
	priv->icl = icl;

	/* set the default format */
	priv->fmt = hi704_default_fmt;
	priv->rect.left = 0;
	priv->rect.top = 0;

	ret = hi704_camera_init(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(priv);
	}

	return ret;
}

static int hi704_remove(struct i2c_client *client)
{
	struct hi704 *priv = to_hi704(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	if (icd)
		icd->ops = NULL;
	kfree(priv);
	return 0;
}

static const struct i2c_device_id hi704_id[] = {
	{ "hi704", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hi704_id);

static struct i2c_driver hi704_i2c_driver = {
	.driver = {
		.name = "hi704",
	},
	.probe = hi704_probe,
	.remove = hi704_remove,
	.id_table = hi704_id,
};

static int __init hi704_mod_init(void)
{
	return i2c_add_driver(&hi704_i2c_driver);
}
module_init(hi704_mod_init);

static void __exit hi704_mod_exit(void)
{
	i2c_del_driver(&hi704_i2c_driver);
}
module_exit(hi704_mod_exit);

MODULE_DESCRIPTION("Hynix HI704 Camera Sensor Driver");
MODULE_AUTHOR("Bhavna Yadav <bhavna.yadav@st.com");
MODULE_LICENSE("GPL v2");
