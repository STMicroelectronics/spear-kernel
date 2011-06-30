/*
 * V4L2 SoC Camera driver for ST VS6725 Camera Sensor
 *
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

/* vs6725 cropping windows params */
#define VS6725_MAX_WIDTH		1600
#define VS6725_MAX_HEIGHT		1200
#define VS6725_MIN_WIDTH		0
#define VS6725_MIN_HEIGHT		0
#define VS6725_COLUMN_SKIP		8
#define VS6725_ROW_SKIP			8

/* device parameters */
#define DEVICE_ID_HI			0x02
#define DEVICE_ID_LO			0xD5
#define FIRMWARE_VERSION		0x1
#define PATCH_VERSION			0

/* hsync, vsync related */
#define HSYNC_ENABLE			BIT(0)
#define HSYNC_POLARITY_ACTIVE_LO	(0 << 1)
#define HSYNC_POLARITY_ACTIVE_HI	BIT(1)
#define VSYNC_ENABLE			BIT(0)
#define VSYNC_POLARITY_ACTIVE_LO	(0 << 1)
#define VSYNC_POLARITY_ACTIVE_HI	BIT(1)

/* pclk related */
#define PCLK_PROG_POL_LO_INIT_LO	(0 << 0)
#define PCLK_PROG_POL_HI_INIT_LO	BIT(0)
#define PCLK_PROG_POL_LO_INIT_HI	BIT(1)
#define PCLK_PROG_POL_HI_INIT_HI	(BIT(1) | BIT(0))
#define PCLK_SYNC_EN			BIT(2)
#define PCLK_HSYNC_N			BIT(3)
#define PCLK_HSYNC_N_INTERNAL		BIT(4)
#define PCLK_VSYNC_N			BIT(5)
#define PCLK_VSYNC_N_INTERNAL		BIT(6)
#define PCLK_SYNC_INTERFRAME_EN		BIT(7)

/* view-live settings */
#define DISABLE_VIEW_LIVE		0
#define ENABLE_VIEW_LIVE		1

/* pipe info */
#define PIPE_0				0
#define PIPE_1				1

/* default zoom step sizes */
#define DEFAULT_ZOOM_STEP_SIZE_LO	0x01
#define DEFAULT_ZOOM_STEP_SIZE_HI	0

/* sensor mode */
#define UXGA_MODE			0
#define ANALOG_BINNING_MODE		1

/* frame rate mode */
#define MANUAL_FRAME_RATE		0
#define AUTO_FRAME_RATE			1

/* register write masks for HI and LO bytes */
#define WRITE_HI_BYTE(x)		((x) & 0xff)
#define WRITE_LO_BYTE(x)		(((x) & 0xff00) >> 8)

/* rgb flip shift */
#define RGB_FLIP_SHIFT(x)		((x) <<	1)

/* register address space */
#define USER_INTERFACE_REG_BASE		0x0
#define HARDWARE_REG_BASE		0xD900
#define LAST_REGISTER_ADDR		0xDA30

/* FIXME: find a better way of doing this */
#define SET_TO_ONE			1

/* vs6725 i2c write address is 0x20 */

/*
 * user-interface registers (firmware)
 */

/* vs6725 device parameter registers */
struct device_param {
	u8 device_id_hi;
	u8 device_id_lo;
	u8 fm_version;
	u8 patch_version;
};

/* vs6725 mode manager registers */
struct mode_manager {
	u8 user_cmd;
	u8 state;
	u8 active_pipe_bank;
	u8 view_live_en;
	u8 frames_streamed;
	u8 stream_length;
	u8 csi_en;
};

/* vs6725 zoom control registers */
struct zoom_control {
	u8 zoom_ctrl;
	u8 zoom_size_hi;
	u8 zoom_size_lo;
	u8 _reserved_zoom[13];
	u8 max_derating;
};

/* vs6725 pipe setup registers */
struct pipe_setup {
	u8 sensor_mode;
	u8 image_size;
	u8 manual_hs_hi;
	u8 manual_hs_lo;
	u8 manual_vs_hi;
	u8 manual_vs_lo;
	u8 data_format;
	u8 bayer_out_align;
	u8 channel_id;
	u8 gamma_r;
	u8 gamma_g;
	u8 gamma_b;
	u8 peaking_gain;
	u8 _reserved_pipe[3];
};

/* vs6725 pipe setup common registers */
struct pipe_setup_common {
	u8 contrast;
	u8 color_saturation;
	u8 brightness;
	u8 hori_mirror;
	u8 vert_flip;
};

/* vs6725 clock chain param FP inputs */
struct clock_chain_param {
	u8 ext_clock_freq_hi;
	u8 ext_clock_freq_lo;
	u8 pll_out_freq_hi;
	u8 pll_out_freq_lo;
	u8 e_div;
};

/* vs6725 static frame rate control registers */
struct static_frame_rate_ctrl {
	u8 des_frame_rate_num_hi;
	u8 des_frame_rate_num_lo;
	u8 des_frame_rate_den;
};

/* vs6725 static frame rate status registers */
struct static_frame_rate_sts {
	u8 req_frame_rate_hi;
	u8 req_frame_rate_lo;
	u8 max_frame_rate;
	u8 min_frame_rate;
};

/* vs6725 auto frame rate control registers */
struct auto_frame_rate_ctrl {
	u8 mode;
	u8 gain_thresh_lo_num;
	u8 gain_thresh_lo_den;
	u8 gain_thresh_hi_num;
	u8 gain_thresh_hi_den;
	u8 usr_min_frame_rate;
	u8 usr_max_frame_rate;
	u8 relative_change_num;
	u8 relative_change_den;
	u8 divorce_min_frame_rate;
};

/* vs6725 auto frame rate status registers */
struct auto_frame_rate_sts {
	u8 implied_gain_hi;
	u8 implied_gain_lo;
	u8 max_frame_len_lines_hi;
	u8 max_frame_len_lines_lo;
	u8 min_frame_len_lines_hi;
	u8 min_frame_len_lines_lo;
	u8 frame_len_change_lines_hi;
	u8 frame_len_change_lines_lo;
	u8 desired_auto_frame_rate_hi;
	u8 desired_auto_frame_rate_lo;
	u8 curr_frame_len_lines_hi;
	u8 curr_frame_len_lines_lo;
	u8 desired_frame_len_lines_hi;
	u8 desired_frame_len_lines_lo;
	u8 auto_frame_rate_stable;
	u8 auto_frame_rate_clip;
};

/* vs6725 exposure control registers */
struct exposure_ctrl {
	u8 mode;
	u8 metering;
	u8 manual_exp_time_num;
	u8 manual_exp_time_den;
	u8 manual_des_exp_time_hi;
	u8 manual_des_exp_time_lo;
	u8 cold_start_time_hi;
	u8 cold_start_time_lo;
	u8 exp_compensation;
	u8 misc_settings;
	u8 direct_mode_int_lines_hi;
	u8 direct_mode_int_lines_lo;
	u8 direct_mode_int_pixels_hi;
	u8 direct_mode_int_pixels_lo;
	u8 direct_mode_analog_gain_hi;
	u8 direct_mode_analog_gain_lo;
	u8 direct_mode_digital_gain_hi;
	u8 direct_mode_digital_gain_lo;
	u8 fg_mode_int_lines_hi;
	u8 fg_mode_int_lines_lo;
	u8 fg_mode_int_pixels_hi;
	u8 fg_mode_int_pixels_lo;
	u8 fg_mode_analog_gain_hi;
	u8 fg_mode_analog_gain_lo;
	u8 fg_mode_digital_gain_hi;
	u8 fg_mode_digital_gain_lo;
	u8 freeze_auto_exp;
	u8 usr_max_int_time_hi;
	u8 usr_max_int_time_lo;
	u8 rec_fg_thresh_hi;
	u8 rec_fg_thresh_lo;
	u8 _reserved_exp_ctrl;
	u8 anti_flicker_mode;
};

/* vs6725 exposure algo controls registers */
struct exposure_algo_ctrl {
	u8 digital_gain_ceiling_hi;
	u8 digital_gain_ceiling_lo;
};

/* vs6725 exposure status registers */
struct exposure_sts {
	u8 int_pending_lines_hi;
	u8 int_pending_lines_lo;
	u8 int_pending_pixels_hi;
	u8 int_pending_pixels_lo;
	u8 analog_gain_pending_hi;
	u8 analog_gain_pending_lo;
	u8 digital_gain_pending_hi;
	u8 digital_gain_pending_lo;
	u8 desired_exp_time_hi;
	u8 desired_exp_time_lo;
	u8 compiled_exp_time_hi;
	u8 compiled_exp_time_lo;
	u8 _reserved_exp_sts;
	u8 max_int_lines_hi;
	u8 max_int_lines_lo;
	u8 total_int_time_hi;
	u8 total_int_time_lo;
	u8 a_gain_pending_hi;
	u8 a_gain_pending_lo;
};

/* vs6725 flicker detect registers */
struct flicker_detect {
	u8 enb_detect;
	u8 detect_start;
	u8 flicker_id_thresh_hi;
	u8 flicker_id_thresh_lo;
	u8 win_times;
	u8 frame_rate_shift_no;
	u8 manual_fref_enb;
	u8 manual_fref100_hi;
	u8 manual_fref100_lo;
	u8 manual_fref120_hi;
	u8 manual_fref120_lo;
	u8 flicker_freq_hi;
	u8 flicker_freq_lo;
};

/* vs6725 white balance control registers */
struct white_bal_ctrl {
	u8 mode;
	u8 manu_red_gain;
	u8 manu_green_gain;
	u8 manu_blue_gain;
	u8 misc_settings;
	u8 hue_r_bias_hi;
	u8 hue_r_bias_lo;
	u8 hue_b_bias_hi;
	u8 hue_b_bias_lo;
	u8 flash_red_gain_hi;
	u8 flash_red_gain_lo;
	u8 flash_green_gain_hi;
	u8 flash_green_gain_lo;
	u8 flash_blue_gain_hi;
	u8 flash_blue_gain_lo;
};

/* vs6725 image stability registers */
struct image_stability {
	u8 white_bal_stable;
	u8 exp_stable;
	u8 _reserved_image_stb;
	u8 stable;
};

/* vs6725 exposure sensor constant registers */
struct exp_sensor_const {
	u8 sensor_a_gain_ceiling_hi;
	u8 sensor_a_gain_ceiling_lo;
};

/* vs6725 flash control registers */
struct flash_control {
	u8 flash_mode;
	u8 _reserved_flash_ctr[7];
	u8 flash_recommended;
};

/* vs6725 anti-vignettte registers */
struct anti_vignette {
	u8 disable;
};

/* vs6725 special effect control registers */
struct spl_effect_ctrl {
	u8 negative;
	u8 solarising;
	u8 sketch;
	u8 _reserved_spl_eff;
	u8 colour_effect;
};

/* vs6725 test pattern registers */
struct test_pattern {
	u8 enb_test_pattern;
	u8 test_pattern;
};

/*
 * low-level hardware registers
 */

/* vs6725 output format registers */
struct output_format {
	u8 opf_dctrl;
	u8 _reserved_out_fmt1[3];
	u8 opf_ycbcr_setup;
	u8 _reserved_out_fmt2[3];
	u8 opf_rgb_setup;
};

/* vs6725 output interface registers */
struct output_interface {
	u8 oif_hsync_setup;
	u8 _reserved_oi1[3];
	u8 oif_vsync_setup;
	u8 _reserved_oi2[31];
	u8 oif_pclk_setup;
};

struct vs6725_user_intf_regs {
	struct device_param dev_param;		/* 0x0000 to 0x0003 */
	u8 _reserved0[12];			/* 0x0004 to 0x000F */
	struct mode_manager mode_mgr;		/* 0x0010 to 0x0016 */
	u8 _reserved1[12];			/* 0x0017 to 0x001F */
	struct zoom_control zoom_ctrl;		/* 0x0020 to 0x0030 */
	u8 _reserved2[15];			/* 0x0031 to 0x003F */
	struct pipe_setup pipe[2];		/* [0]=pipe 0, [1]=pipe 1 */
	struct pipe_setup_common pipe_common;	/* 0x0060 to 0x0064 */
	u8 _reserved3[11];			/* 0x0065 to 0x006F */
	struct clock_chain_param clk_chn_param;	/* 0x0070 to 0x0074 */
	u8 _reserved4[27];			/* 0x0075 to 0x008F */
	struct static_frame_rate_ctrl stat_ctr;	/* 0x0090 to 0x0092 */
	u8 _reserved5[13];			/* 0x0093 to 0x009F */
	struct static_frame_rate_sts stat_sts;	/* 0x00A0 to 0x00A5 */
	u8 _reserved6[10];			/* 0x00A6 to 0x00AF */
	struct auto_frame_rate_ctrl auto_ctr;	/* 0x00B0 to 0x00B9 */
	u8 _reserved7[6];			/* 0x00BA to 0x00BF */
	struct auto_frame_rate_sts auto_sts;	/* 0x00C0 to 0x00CF */
	u8 _reserved8[32];			/* 0x00D0 to 0x00EF */
	struct exposure_ctrl exp_ctrl;		/* 0x00F0 to 0x0110 */
	u8 _reserved9[29];			/* 0x0111 to 0x012D */
	struct exposure_algo_ctrl exp_a_ctrl;	/* 0x012E to 0x012F */
	u8 _reserved10[36];			/* 0x0130 to 0x0153 */
	struct exposure_sts exp_sts;		/* 0x0154 to 0x0166 */
	u8 _reserved11[9];			/* 0x0167 to 0x016F */
	struct flicker_detect flick_detect;	/* 0x0170 to 0x017D */
	u8 _reserved12[2];			/* 0x017E to 0x017F */
	struct white_bal_ctrl wb_ctrl;		/* 0x0180 to 0x018E */
	u8 _reserved13[130];			/* 0x018F to 0x0210 */
	struct image_stability img_stability;	/* 0x0211 to 0x0214 */
	u8 _reserved14[29];			/* 0x0215 to 0x0231 */
	struct exp_sensor_const exp_sensor;	/* 0x0232 to 0x0233 */
	u8 _reserved15[12];			/* 0x0234 to 0x023F */
	struct flash_control flash_ctr;		/* 0x0240 to 0x0248 */
	u8 _reserved16[23];			/* 0x0249 to 0x025F */
	struct anti_vignette vignette;		/* 0x0260 */
	u8 _reserved17[95];			/* 0x0261 to 0x02BF */
	struct spl_effect_ctrl effect_ctr;	/* 0x02C0 to 0x02C4 */
	u8 _reserved18[95];			/* 0x02C5 to 0x049F */
	struct test_pattern test;		/* 0x04A0 to 0x04A1 */
};

struct vs6725_hardware_regs {
	struct output_format out_format;	/* 0xD900 to 0xD908 */
	u8 _reserved19[259];			/* 0xD909 to 0xDA0B */
	struct output_interface out_intf;	/* 0xDA0C to 0xDA30 */
};

struct vs6725 {
	struct v4l2_subdev subdev;
	int active_pipe;		/* whether pipe 0 or pipe 1 is active */
	int saturation;
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
	struct v4l2_rect rect;		/* sensor cropping window */
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	struct vs6725_user_intf_regs *intf_regs;
	struct vs6725_hardware_regs *hw_regs;
};

enum vs6725_exposure_ctrl_mode {
	AUTOMATIC_MODE = 0,
	COMPILED_MANUAL_MODE,
	DIRECT_MANUAL_MODE,
	FLASHGUN_MODE,
};

enum vs6725_color_effects {
	COLOR_EFFECT_NORMAL = 0,
	COLOR_EFFECT_RED_ONLY,
	COLOR_EFFECT_YELLOW_ONLY,
	COLOR_EFFECT_GREEN_ONLY,
	COLOR_EFFECT_BLUE_ONLY,
	COLOR_EFFECT_BLACKNWHITE,
	COLOR_EFFECT_SEPIA,
	COLOR_EFFECT_ANTIQUE,
	COLOR_EFFECT_AQUA,
	COLOR_EFFECT_MANUAL_MATRIX,
};

enum vs6725_data_formats {
	DATA_FORMAT_YCBCR_JFIF = 0,
	DATA_FORMAT_YCBCR_REC601,
	DATA_FORMAT_YCBCR_CUSTOM,
	DATA_FORMAT_RGB_565,
	DATA_FORMAT_RGB_565_CUSTOM,
	DATA_FORMAT_RGB_444,
	DATA_FORMAT_RGB_444_CUSTOM,
	DATA_FORMAT_BAYER_BYPASS,
};

enum vs6725_ycbcr_sequence_setup {
	CBYCRY_DATA_SEQUENCE = 0,
	CRYCBY_DATA_SEQUENCE,
	YCBYCR_DATA_SEQUENCE,
	YCRYCB_DATA_SEQUENCE,
};

enum vs6725_rgb_sequence_setup {
	GBR_DATA_SEQUENCE = 0,
	RBG_DATA_SEQUENCE,
	BRG_DATA_SEQUENCE,
	GRB_DATA_SEQUENCE,
	RGB_DATA_SEQUENCE,
	BGR_DATA_SEQUENCE,
};

enum vs6725_rgb444_format {
	RGB444_ZERO_PADDING_ON = 0,
	RGB444_ZERO_PADDING_OFF,
};

enum vs6725_user_command {
	CMD_BOOT = 1,
	CMD_RUN,
	CMD_STOP,
};

enum vs6725_state {
	STATE_RAW = 0x10,
	STATE_IDLE = 0x20,
	STATE_RUNNING = 0x30,
};

enum vs6725_zoom_control {
	ZOOM_STOP = 0,
	ZOOM_START_IN,
	ZOOM_START_OUT,
	ZOOM_STEP_IN,
	ZOOM_STEP_OUT,
};

enum vs6725_image_size {
	IMAGE_SIZE_UXGA = 0,
	IMAGE_SIZE_SXGA,
	IMAGE_SIZE_SVGA,
	IMAGE_SIZE_VGA,
	IMAGE_SIZE_CIF,
	IMAGE_SIZE_QVGA,
	IMAGE_SIZE_QCIF,
	IMAGE_SIZE_QQVGA,
	IMAGE_SIZE_QQCIF,
	IMAGE_SIZE_MANUAL,
};

/*
 * pixel codes supported by VS6725:
 * Note that by default the endianess is big-endian as defined by
 * OPF_DCTRL register
 */
static enum v4l2_mbus_pixelcode vs6725_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_YVYU8_2X8,
	V4L2_MBUS_FMT_VYUY8_2X8,
	V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE,
	V4L2_MBUS_FMT_RGB565_2X8_BE,
};

/* controls supported by VS6725 */
static const struct v4l2_queryctrl vs6725_controls[] = {
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
		/* write only control */
		/*
		 * VS6725 provides a zoom relative feature in which the
		 * zoom step can be programmed. This allows a single
		 * step of zoom.
		 */
		.id		= V4L2_CID_ZOOM_RELATIVE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Zoom Relative",
		.minimum	= 0,
		.maximum	= 0xffff,
		.step		= 1, /* 1 step = 5% */
		.default_value	= 0x0001,
	},
	{	/*
		 * VS6725 provides zoom continuous feature using which
		 * it is possible to achieve a continuous zoom by simply
		 * selecting the commands zoom_in, zoom_out and zoom_stop
		 */
		.id		= V4L2_CID_ZOOM_CONTINUOUS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Zoom Continuous",
		/* other fields cannot be defined */
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

static bool is_unscaled_image_ok(int width, int height, struct v4l2_rect *rect)
{
	return width > rect->width || height > rect->height;
}

/* read a register */
static int vs6725_reg_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	u8 data = reg;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	*val = data;
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/* write a register */
static int vs6725_reg_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = {reg, val};
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	udelay(100);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

static struct vs6725 *to_vs6725(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct vs6725, subdev);
}

/* Alter bus settings on camera side */
static int vs6725_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct vs6725 *priv = to_vs6725(client);
	int ret;

	flags = soc_camera_apply_sensor_flags(icl, flags);

	if (flags & SOCAM_PCLK_SAMPLE_RISING)
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_pclk_setup,
				PCLK_PROG_POL_HI_INIT_LO);
	else
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_pclk_setup,
				PCLK_PROG_POL_LO_INIT_LO);
	if (ret)
		return ret;

	if (flags & SOCAM_HSYNC_ACTIVE_LOW)
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_hsync_setup,
				HSYNC_POLARITY_ACTIVE_LO);
	else
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_hsync_setup,
				VSYNC_POLARITY_ACTIVE_HI);
	if (ret)
		return ret;

	if (flags & SOCAM_VSYNC_ACTIVE_LOW)
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_vsync_setup,
				VSYNC_POLARITY_ACTIVE_LO);
	else
		ret = vs6725_reg_write(client,
				priv->hw_regs->out_intf.oif_hsync_setup,
				VSYNC_POLARITY_ACTIVE_HI);

	return ret;
}

/* Request bus settings on camera side */
static unsigned long vs6725_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	/*
	 * FIXME: Revisit this later
	 * these settings must be passed from platform data somehow
	 */
	unsigned long flags = SOCAM_MASTER |
		SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

/* get the current settings of a control on Vs6725 */
static int vs6725_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	unsigned char val;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	switch (ctrl->id) {
	case V4L2_CID_CONTRAST:
		ret = vs6725_reg_read(client,
				priv->intf_regs->pipe_common.contrast,
				&val);
		ctrl->value = val;
		break;
	case V4L2_CID_SATURATION:
		ret = vs6725_reg_read(client,
				priv->intf_regs->pipe_common.color_saturation,
				&val);
		ctrl->value = val;
		break;
	case V4L2_CID_GAIN:
		ret = vs6725_reg_read(client,
			priv->intf_regs->pipe[priv->active_pipe].peaking_gain,
			&val);
		ctrl->value = val;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = priv->awb;
		break;
	/* FIXME: Exposure related stuff to be formalized */
	case V4L2_CID_EXPOSURE_AUTO:
		ctrl->value = priv->aec;
		break;
	/*
	 * FIXME: VS6725 supports dynamic frame rate control so
	 * should we also implement
	 * V4L2_CID_EXPOSURE_AUTO_PRIORITY support here?
	 */
	case V4L2_CID_GAMMA:
		ctrl->value = priv->gamma;
		break;
	case V4L2_CID_VFLIP:
		ret = vs6725_reg_read(client,
				priv->intf_regs->pipe_common.vert_flip,
				&val);
		ctrl->value = val;
		break;
	case V4L2_CID_HFLIP:
		ret = vs6725_reg_read(client,
				priv->intf_regs->pipe_common.hori_mirror,
				&val);
		ctrl->value = val;
		break;
	case V4L2_CID_COLORFX:
		ctrl->value = priv->color_effect;
		break;
	}

	return 0;
}

/* set some particular settings of a control on Vs6725 */
static int vs6725_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	switch (ctrl->id) {
	case V4L2_CID_CONTRAST:
		ret = vs6725_reg_write(client,
				priv->intf_regs->pipe_common.contrast,
				ctrl->value);
		if (!ret)
			priv->contrast = ctrl->value;
		break;
	case V4L2_CID_SATURATION:
		ret = vs6725_reg_write(client,
				priv->intf_regs->pipe_common.color_saturation,
				ctrl->value);
		if (!ret)
			priv->saturation = ctrl->value;
		break;
	case V4L2_CID_GAIN:
		ret = vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].peaking_gain,
			ctrl->value);
		if (!ret)
			priv->gain = ctrl->value;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = vs6725_reg_write(client, priv->intf_regs->wb_ctrl.mode,
					ctrl->value);
		if (!ret)
			priv->awb = ctrl->value;
		break;
	/* FIXME: Exposure related stuff to be formalized */
	case V4L2_CID_EXPOSURE_AUTO:
		switch (ctrl->value) {
		case V4L2_EXPOSURE_AUTO:
			ret = vs6725_reg_write(client,
					priv->intf_regs->exp_ctrl.mode,
					AUTOMATIC_MODE);
			break;
		default:
			/*
			 * FIXME: implement in a better way as
			 * DIRECT_MANUAL_MODE may be different from the
			 * COMPILED_MANUAL_MODE and we need to set the
			 * correct manual mode here. Also set the
			 * related manual exposure related registers
			 * here, for e.g. exposure time etc. after
			 * clarification from the sensor designers
			 */
			ret = vs6725_reg_write(client,
					priv->intf_regs->exp_ctrl.mode,
					DIRECT_MANUAL_MODE);
			break;
		}
		if (!ret)
			priv->aec = ctrl->value;
		break;
		/*
		 * FIXME: VS6725 supports dynamic frame rate control so
		 * should we also implement
		 * V4L2_CID_EXPOSURE_AUTO_PRIORITY support here?
		 */

	case V4L2_CID_GAMMA:
		/*
		 * FIXME: not clear if we need to set the same GAMMA gain of
		 * the RED, BLUE and GREEN channels here. Verify with
		 * sensor designers.
		 */
		ret = vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].gamma_r,
			ctrl->value);
		if (!ret)
			ret = vs6725_reg_write(client,
				priv->intf_regs->pipe[priv->active_pipe].
				gamma_g, ctrl->value);
		if (!ret)
			ret = vs6725_reg_write(client,
				priv->intf_regs->pipe[priv->active_pipe].
				gamma_b, ctrl->value);
		if (!ret)
			priv->gamma = ctrl->value;
		break;
	case V4L2_CID_VFLIP:
		ret = vs6725_reg_write(client,
				priv->intf_regs->pipe_common.vert_flip,
				ctrl->value);
		if (!ret)
			priv->vflip = ctrl->value;
		break;
	case V4L2_CID_HFLIP:
		ret = vs6725_reg_write(client,
				priv->intf_regs->pipe_common.hori_mirror,
				ctrl->value);
		if (!ret)
			priv->hflip = ctrl->value;
		break;
	case V4L2_CID_ZOOM_RELATIVE:
		/*
		 * use the default step size of 5% for every relative zoom
		 * operation
		 */
		ret = vs6725_reg_write(client,
				priv->intf_regs->zoom_ctrl.zoom_size_hi,
				DEFAULT_ZOOM_STEP_SIZE_HI);
		if (!ret)
			ret = vs6725_reg_write(client,
				priv->intf_regs->zoom_ctrl.zoom_size_lo,
				DEFAULT_ZOOM_STEP_SIZE_LO);
		if (!ret) {
			/*
			 * negative values move the zoom lens towards the
			 * wide-angle direction, whereas positive values move
			 * the zoom lens group towards the telephoto direction
			 */
			if (ctrl->value < 0) {
				ret = vs6725_reg_write(client,
					priv->intf_regs->zoom_ctrl.zoom_ctrl,
					ZOOM_STEP_IN);
			} else {
				ret = vs6725_reg_write(client,
					priv->intf_regs->zoom_ctrl.zoom_ctrl,
					ZOOM_STEP_OUT);
			}
		}

		if (ret != 0)
			dev_err(&client->dev,
				"zoom-relative operation failed\n");
		break;
	case V4L2_CID_ZOOM_CONTINUOUS:
		/*
		 * 1. positive values move the zoom lens group towards the
		 * telephoto direction.
		 * 2. a value of zero stops the zoom lens group movement.
		 * 3. negative values move the zoom lens group towards the
		 * wide-angle direction
		 */
		if (ctrl->value < 0)
			ret = vs6725_reg_write(client,
				priv->intf_regs->zoom_ctrl.zoom_ctrl,
				ZOOM_START_OUT);
		else if (ctrl->value > 0)
			ret = vs6725_reg_write(client,
				priv->intf_regs->zoom_ctrl.zoom_ctrl,
				ZOOM_START_IN);
		else
			ret = vs6725_reg_write(client,
				priv->intf_regs->zoom_ctrl.zoom_ctrl,
				ZOOM_STOP);

		if (ret != 0)
			dev_err(&client->dev,
				"zoom-continuous operation failed\n");
		break;
	case V4L2_CID_COLORFX:
		switch (ctrl->value) {
		case V4L2_COLORFX_NONE:
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.colour_effect,
				COLOR_EFFECT_NORMAL);
			break;
		case V4L2_COLORFX_BW:
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.colour_effect,
				COLOR_EFFECT_BLACKNWHITE);
			break;
		case V4L2_COLORFX_SEPIA:
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.colour_effect,
				COLOR_EFFECT_SEPIA);
			break;
		case V4L2_COLORFX_NEGATIVE:
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.negative,
				SET_TO_ONE);
			break;
		case V4L2_COLORFX_SKETCH:
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.sketch,
				SET_TO_ONE);
			break;
		default:
			/*
			 * FIXME: V4L2_COLORFX_EMBOSS, V4L2_COLORFX_SKY_BLUE,
			 * V4L2_COLORFX_GRASS_GREEN, V4L2_COLORFX_SKIN_WHITEN
			 * and V4L2_COLORFX_VIVID color effects are not
			 * supported by VS6725, so setting coler effect
			 * as NONE in such cases.
			 */
			dev_info(&client->dev, "color format not supported"
					"setting color effect as none\n");
			ret = vs6725_reg_write(client,
				priv->intf_regs->effect_ctr.colour_effect,
				COLOR_EFFECT_NORMAL);
			break;
		}
		if (!ret)
			priv->color_effect = ctrl->value;
		break;
	}

	return ret;
}

static int vs6725_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident = priv->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vs6725_get_register(struct v4l2_subdev *sd,
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

	ret = vs6725_reg_read(client, reg->reg, &val);
	if (!ret)
		reg->val = (u64)val;

	return ret;
}

static int vs6725_set_register(struct v4l2_subdev *sd,
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

	return vs6725_reg_write(client, reg->reg, reg->val);
}
#endif

/* start/stop streaming from the device */
static int vs6725_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	if (enable)
		ret = vs6725_reg_write(client,
				priv->intf_regs->mode_mgr.user_cmd,
				CMD_RUN);
	else
		ret = vs6725_reg_write(client,
				priv->intf_regs->mode_mgr.user_cmd,
				CMD_STOP);

	if (ret < 0)
		return ret;

	return 0;
}

static int vs6725_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left = VS6725_COLUMN_SKIP;
	a->bounds.top = VS6725_ROW_SKIP;
	a->bounds.width = VS6725_MAX_WIDTH;
	a->bounds.height = VS6725_MAX_HEIGHT;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator = 1;
	a->pixelaspect.denominator = 1;

	return 0;
}

/* get the current cropping rectangle */
static int vs6725_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->c = priv->rect;

	return 0;
}

/* change the current cropping rectangle */
static int vs6725_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);
	struct v4l2_rect *rect = &a->c;
	int ret;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/*
	 * FIXME: I think height should be aligned by 2 because the
	 * V4L2 specs state that "the driver must round the vertical
	 * offset of the cropping rectangle to frame lines modulo two,
	 * such that the field order cannot be confused" but not sure if
	 * the width field should also be aligned here.
	 */
	rect->height = ALIGN(rect->height, 2);
	rect->width = ALIGN(rect->width, 2);

	/* FIXME: the datasheet doesn't specify minimum sizes */
	soc_camera_limit_side(&rect->left, &rect->width,
			VS6725_COLUMN_SKIP, VS6725_MIN_WIDTH, VS6725_MAX_WIDTH);
	soc_camera_limit_side(&rect->top, &rect->height,
			VS6725_ROW_SKIP, VS6725_MIN_HEIGHT, VS6725_MAX_HEIGHT);

	/* cropping means we set the image size manually */
	ret = vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].image_size,
				IMAGE_SIZE_MANUAL);
	if (!ret) {
		priv->rect.left = rect->left;
		priv->rect.width = rect->width;
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].manual_hs_lo,
			WRITE_LO_BYTE(rect->left + rect->width));
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].manual_hs_hi,
			WRITE_HI_BYTE(rect->left + rect->width));
	}

	if (!ret) {
		priv->rect.height = rect->height;
		priv->rect.top = rect->top;
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].manual_vs_lo,
			WRITE_LO_BYTE(rect->top + rect->height));
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].manual_vs_hi,
			WRITE_HI_BYTE(rect->top + rect->height));
	}

	return ret;
}

/* get the format we will capture in */
static int vs6725_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = priv->code;
	mf->colorspace = priv->colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

/* set the format we will capture in */
static int vs6725_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);
	struct v4l2_crop a = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.c = {
			.left	= priv->rect.left + priv->rect.width -
					mf->width,
			.top	= priv->rect.top + priv->rect.height -
					mf->height,
			.width	= mf->width,
			.height	= mf->height,
		},
	};
	enum v4l2_mbus_pixelcode code = mf->code;

	/*
	 * FIXME: VS6725 specs state that both YCbCr-JFIF and
	 * YCbCr-Rec601 data formats are supported and in addtion some
	 * custom modes are also supported for RGB444, RGB565 and YCbCr.
	 * There is also support for a Bayer data format in Bypass mode.
	 * Need to clarify details of these formats from the sensor
	 * designer before adding the support for the same.
	 *
	 * Note that later we may need to add colorspace Rec601 in
	 * addition to JPEG for YCbCr data formats.
	 *
	 * Also note that section 3.8.3 of the user manual mentions a
	 * bYCbCrSetup register which can be used to change the order of
	 * YCbcr components. However, no such details are present in the
	 * interface/Hardware register section. Instead there is a
	 * OPF_YCBCR_SETUP register in the Output Format section which
	 * mentions the capability to swap Y and Cb, Cr components.
	 * Treating the details mentioned in section 3.8.3 as correct as
	 * of now.
	 */
	switch (code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		dev_dbg(&client->dev, "pixel format YUYV8_2X8\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_YCBCR_JFIF);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_ycbcr_setup,
			YCBYCR_DATA_SEQUENCE);
		priv->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		dev_dbg(&client->dev, "pixel format UYVY8_2X8\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_YCBCR_JFIF);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_ycbcr_setup,
			CBYCRY_DATA_SEQUENCE);
		priv->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		dev_dbg(&client->dev, "pixel format YVYU8_2X8\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_YCBCR_JFIF);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_ycbcr_setup,
			YCRYCB_DATA_SEQUENCE);
		priv->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		dev_dbg(&client->dev, "pixel format VYUY8_2X8\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_YCBCR_JFIF);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_ycbcr_setup,
			CRYCBY_DATA_SEQUENCE);
		priv->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE:
		dev_dbg(&client->dev, "pixel format RBG444_2X8_PADHI_BE\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_RGB_444);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_rgb_setup,
			RGB_FLIP_SHIFT(RGB_DATA_SEQUENCE) |
			RGB444_ZERO_PADDING_ON);
		priv->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		dev_dbg(&client->dev, "pixel format RGB565_2X8_BE\n");
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_RGB_565);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_rgb_setup,
			RGB_FLIP_SHIFT(RGB_DATA_SEQUENCE));
		priv->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	default:
		/*
		 * V4L2 specs state that we should _not_ return
		 * error in case we do not support a particular format
		 * and instead should apply the default settings and
		 * return.
		 */
		dev_err(&client->dev, "Pixel format not handled: 0x%x\n"
			"Reverting to default YCbCr-JFIF format\n", code);
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].data_format,
			DATA_FORMAT_YCBCR_JFIF);
		ret |= vs6725_reg_write(client,
			priv->hw_regs->out_format.opf_ycbcr_setup,
			CBYCRY_DATA_SEQUENCE);
		priv->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	}

	if (!ret) {
		priv->code = code;
		ret = vs6725_s_crop(sd, &a);
	}

	if (!ret) {
		mf->colorspace = priv->colorspace;
		mf->width = priv->rect.width;
		mf->height = priv->rect.height;
	}

	return ret;
}

static int vs6725_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	if (is_unscaled_image_ok(mf->width, mf->height, &priv->rect))
		v4l_bound_align_image(&mf->width, VS6725_MIN_WIDTH,
			VS6725_MAX_WIDTH, 1,
			&mf->height, VS6725_MIN_HEIGHT,
			VS6725_MAX_HEIGHT, 1, 0);

	mf->field = V4L2_FIELD_NONE;

	switch (mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
	default:
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		mf->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}

	return 0;
}

static int vs6725_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(vs6725_codes))
		return -EINVAL;

	*code = vs6725_codes[index];
	return 0;
}

static struct soc_camera_ops vs6725_ops = {
	.set_bus_param = vs6725_set_bus_param,
	.query_bus_param = vs6725_query_bus_param,
	.controls = vs6725_controls,
	.num_controls = ARRAY_SIZE(vs6725_controls),
};

static struct v4l2_subdev_core_ops vs6725_subdev_core_ops = {
	.g_ctrl = vs6725_g_ctrl,
	.s_ctrl = vs6725_s_ctrl,
	.g_chip_ident = vs6725_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = vs6725_get_register,
	.s_register = vs6725_set_register,
#endif
};

static struct v4l2_subdev_video_ops vs6725_video_ops = {
	.s_stream = vs6725_s_stream,
	.g_mbus_fmt = vs6725_g_fmt,
	.s_mbus_fmt = vs6725_s_fmt,
	.try_mbus_fmt = vs6725_try_fmt,
	.enum_mbus_fmt = vs6725_enum_fmt,
	.cropcap = vs6725_cropcap,
	.g_crop = vs6725_g_crop,
	.s_crop	= vs6725_s_crop,
};

static struct v4l2_subdev_ops vs6725_subdev_ops = {
	.core = &vs6725_subdev_core_ops,
	.video = &vs6725_video_ops,
};

/*
 * program default register values:
 * REVISIT: there may be other registers that need to be
 * programmed here. Confirm from sensor designers
 */
static int vs6725_prog_default(struct i2c_client *client)
{
	int ret;
	struct vs6725 *priv = to_vs6725(client);

	/*
	 * we do not support ViewLive mode, so there will
	 * be no switching between pipe 0 and 1 for alternative frames
	 */
	ret = vs6725_reg_write(client,
				priv->intf_regs->mode_mgr.view_live_en,
				DISABLE_VIEW_LIVE);

	if (!ret)
		ret = vs6725_reg_write(client,
				priv->intf_regs->mode_mgr.active_pipe_bank,
				PIPE_0);

	/* pipe 0 related settings */
	if (!ret) {
		priv->active_pipe = PIPE_0;
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].
			sensor_mode, UXGA_MODE);
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].
			image_size, IMAGE_SIZE_UXGA);
		ret |= vs6725_reg_write(client,
			priv->intf_regs->pipe[priv->active_pipe].
			data_format, DATA_FORMAT_YCBCR_JFIF);
	}

	/* we set automatic frame rate control as default ... */
	if (!ret)
		ret = vs6725_reg_write(client,
				priv->intf_regs->auto_ctr.mode,
				AUTO_FRAME_RATE);

	/* ... and automatic expsoure control */
	if (!ret)
		ret = vs6725_reg_write(client,
				priv->intf_regs->exp_ctrl.mode,
				AUTOMATIC_MODE);

	/*
	 * REVISIT: the default values of the hardware registers after
	 * reset, set the output format as big-endian, data sequence as
	 * CbYCrY/RGB and HSYNC/VSYNC as active LOW which are the desired
	 * settings, so there should be no need to explicitly program the same
	 */

	return 0;
}

static int vs6725_camera_init(struct soc_camera_device *icd,
				struct i2c_client *client)
{
	int ret = 0;
	unsigned char dev_id_hi, dev_id_lo;
	unsigned char fm_ver, patch_ver;
	struct vs6725 *priv = to_vs6725(client);

	/*
	 * check and show device id, firmware version and patch version
	 */
	ret = vs6725_reg_read(client, priv->intf_regs->dev_param.device_id_hi,
				&dev_id_hi);
	if (!ret)
		ret = vs6725_reg_read(client,
				priv->intf_regs->dev_param.device_id_lo,
				&dev_id_lo);
	if (!ret)
		ret = vs6725_reg_read(client,
				priv->intf_regs->dev_param.fm_version,
				&fm_ver);
	if (!ret)
		ret = vs6725_reg_read(client,
				priv->intf_regs->dev_param.patch_version,
				&patch_ver);

	if (ret)
		return ret;

	if ((dev_id_hi != DEVICE_ID_HI) ||
			(dev_id_lo != DEVICE_ID_LO)) {
		dev_err(&client->dev, "Device ID error 0x%02x:0x%02x\n",
				dev_id_hi, dev_id_lo);
		return -ENODEV;
	}

	priv->model = V4L2_IDENT_VS6725;

	dev_info(&client->dev,
		"vs6725 Device ID 0x%02x::0x%02x Firmware Ver 0x%02x"
		"Patch Ver 0x%02x\n", dev_id_hi, dev_id_lo, fm_ver, patch_ver);

	ret = vs6725_prog_default(client);

	return ret;
}

static int vs6725_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct vs6725 *priv;
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

	v4l2_i2c_subdev_init(&priv->subdev, client, &vs6725_subdev_ops);

	icd->ops = &vs6725_ops;

	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = VS6725_MAX_WIDTH - VS6725_MIN_WIDTH;
	priv->rect.height = VS6725_MAX_HEIGHT - VS6725_MIN_HEIGHT;
	priv->code = V4L2_MBUS_FMT_YUYV8_2X8;
	priv->colorspace = V4L2_COLORSPACE_JPEG;

	/* assign starting addresses */
	priv->intf_regs = (struct vs6725_user_intf_regs *)
				USER_INTERFACE_REG_BASE;
	priv->hw_regs = (struct vs6725_hardware_regs *)
				HARDWARE_REG_BASE;
	ret = vs6725_camera_init(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(priv);
	}

	return ret;
}

static int vs6725_remove(struct i2c_client *client)
{
	struct vs6725 *priv = to_vs6725(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	if (icd)
		icd->ops = NULL;
	kfree(priv);
	return 0;
}

static const struct i2c_device_id vs6725_id[] = {
	{ "vs6725", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vs6725_id);

static struct i2c_driver vs6725_i2c_driver = {
	.driver = {
		.name = "vs6725",
	},
	.probe = vs6725_probe,
	.remove = vs6725_remove,
	.id_table = vs6725_id,
};

static int __init vs6725_mod_init(void)
{
	return i2c_add_driver(&vs6725_i2c_driver);
}
module_init(vs6725_mod_init);

static void __exit vs6725_mod_exit(void)
{
	i2c_del_driver(&vs6725_i2c_driver);
}
module_exit(vs6725_mod_exit);

MODULE_DESCRIPTION("ST VS6725 Camera Sensor Driver");
MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com");
MODULE_LICENSE("GPL v2");
