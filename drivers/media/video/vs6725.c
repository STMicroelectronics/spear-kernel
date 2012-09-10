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
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>

/* supported resolutions */
#define UXGA_WIDTH	1600
#define UXGA_HEIGHT	1200
#define SXGA_WIDTH	1280
#define SXGA_HEIGHT	1024
#define SVGA_WIDTH	800
#define SVGA_HEIGHT	600
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144
#define QQVGA_WIDTH	160
#define QQVGA_HEIGHT	120
#define QQCIF_WIDTH	88
#define QQCIF_HEIGHT	72

/* max supported frame rate */
#define MAX_FRAME_RATE	30

/* vs6725 cropping windows params */
#define VS6725_MAX_WIDTH		UXGA_WIDTH
#define VS6725_MAX_HEIGHT		UXGA_HEIGHT
#define VS6725_MIN_WIDTH		0
#define VS6725_MIN_HEIGHT		0
#define VS6725_COLUMN_SKIP		8
#define VS6725_ROW_SKIP			8

/* device parameters */
#define VS6725_DEVICE_ID_HI		0x02
#define VS6725_DEVICE_ID_LO		0xD5
#define VS6725_FIRMWARE_VERSION		0x1
#define VS6725_PATCH_VERSION		0

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
#define WRITE_HI_BYTE(x)		(((x) & 0xff00) >> 8)
#define WRITE_LO_BYTE(x)		((x) & 0xff)

/* rgb flip shift */
#define RGB_FLIP_SHIFT(x)		((x) <<	1)

/* register address space */
#define USER_INTERFACE_REG_BASE		0x0
#define HARDWARE_REG_BASE		0xD900
#define LAST_REGISTER_ADDR		0xDA30

/* vs6725 i2c write address is 0x20 */

/*
 * user-interface registers (firmware)
 */

/* vs6725 device parameter registers */
#define DEVICE_ID_HI			0x0000
#define DEVICE_ID_LO			0x0001
#define FM_VERSION			0x0002
#define PATCH_VERSION			0x0003
/* vs6725 mode manager registers */
#define USER_CMD			0x0010
#define	STATE				0x0011
#define ACTIVE_PIPE_BANK		0x0012
#define VIEW_LIVE_EN			0x0013
#define FRAMES_STREAMED			0x0014
#define STREAM_LENGTH			0x0015
#define CSI_EN				0x0016
/* vs6725 zoom control registers */
#define ZOOM_CTRL			0x0020
#define ZOOM_SIZE_HI			0x0021
#define ZOOM_SIZE_LO			0x0022
#define MAX_DERATING			0x0030
/* vs6725 pipe0 setup registers */
#define PIPE0_SENSOR_MODE		0x0040
#define PIPE0_IMAGE_SIZE		0x0041
#define PIPE0_MANUAL_HS_HI		0x0042
#define PIPE0_MANUAL_HS_LO		0x0043
#define PIPE0_MANUAL_VS_HI		0x0044
#define PIPE0_MANUAL_VS_LO		0x0045
#define PIPE0_DATA_FORMAT		0x0046
#define PIPE0_BAYER_OUT_ALIGN		0x0047
#define PIPE0_CHANNEL_ID		0x0048
#define PIPE0_GAMMA_R			0x0049
#define PIPE0_GAMMA_G			0x004A
#define PIPE0_GAMMA_B			0x004B
#define PIPE0_PEAKING_GAIN		0x004C
/* vs6725 pipe1 setup registers */
#define PIPE1_SENSOR_MODE		0x0050
#define PIPE1_IMAGE_SIZE		0x0051
#define PIPE1_MANUAL_HS_HI		0x0052
#define PIPE1_MANUAL_HS_LO		0x0053
#define PIPE1_MANUAL_VS_HI		0x0054
#define PIPE1_MANUAL_VS_LO		0x0055
#define PIPE1_DATA_FORMAT		0x0056
#define PIPE1_BAYER_OUT_ALIGN		0x0057
#define PIPE1_CHANNEL_ID		0x0058
#define PIPE1_GAMMA_R			0x0059
#define PIPE1_GAMMA_G			0x005A
#define PIPE1_GAMMA_B			0x005B
#define PIPE1_PEAKING_GAIN		0x005C
/* vs6725 pipe setup common registers */
#define CONTRAST			0x0060
#define COLOR_SATURATION		0x0061
#define BRIGHTNESS			0x0062
#define HORI_MIRROR			0x0063
#define VERT_FLIP			0x0064
/* vs6725 clock chain param FP inputs */
#define EXT_CLOCK_FREQ_HI		0x0070
#define EXT_CLOCK_FREQ_LO		0x0071
#define PLL_OUT_FREQ_HI			0x0072
#define PLL_OUT_FREQ_LO			0x0073
#define E_DIV				0x0074
/* vs6725 static frame rate control registers */
#define DES_FRAME_RATE_NUM_HI		0x0090
#define DES_FRAME_RATE_NUM_LO		0x0091
#define DES_FRAME_RATE_DEN		0x0092
/* vs6725 static frame rate status registers */
#define REQ_FRAME_RATE_HI		0x00A0
#define REQ_FRAME_RATE_LO		0x00A1
#define MAX_FRAME_RATE_HI		0x00A2
#define MAX_FRAME_RATE_LO		0x00A3
#define MIN_FRAME_RATE_HI		0x00A4
#define MIN_FRAME_RATE_LO		0x00A5
/* vs6725 auto frame rate control registers */
#define AUTO_FRAME_MODE			0x00B0
#define GAIN_THRESH_LO_NUM		0x00B1
#define GAIN_THRESH_LO_DEN		0x00B2
#define GAIN_THRESH_HI_NUM		0x00B3
#define GAIN_THRESH_HI_DEN		0x00B4
#define USR_MIN_FRAME_RATE		0x00B5
#define USR_MAX_FRAME_RATE		0x00B6
#define RELATIVE_CHANGE_NUM		0x00B7
#define RELATIVE_CHANGE_DEN		0x00B8
#define DIVORCE_MIN_FRAME_RATE		0x00B9
/* vs6725 auto frame rate status registers */
#define IMPLIED_GAIN_HI			0x00C0
#define IMPLIED_GAIN_LO			0x00C1
#define MAX_FRAME_LEN_LINES_HI		0x00C2
#define MAX_FRAME_LEN_LINES_LO		0x00C3
#define MIN_FRAME_LEN_LINES_HI		0x00C4
#define MIN_FRAME_LEN_LINES_LO		0x00C5
#define FRAME_LEN_CHANGE_LINES_HI	0x00C6
#define FRAME_LEN_CHANGE_LINES_LO	0x00C7
#define DESIRED_AUTO_FRAME_RATE_HI	0x00C8
#define DESIRED_AUTO_FRAME_RATE_LO	0x00C9
#define CURR_FRAME_LEN_LINES_HI		0x00CA
#define CURR_FRAME_LEN_LINES_LO		0x00CB
#define DESIRED_FRAME_LEN_LINES_HI	0x00CC
#define DESIRED_FRAME_LEN_LINES_LO	0x00CD
#define AUTO_FRAME_RATE_STABLE		0x00CE
#define AUTO_FRAME_RATE_CLIP		0x00CF
/* vs6725 exposure control registers */
#define EXP_CTRL_MODE			0x00F0
#define METERING			0x00F1
#define MANUAL_EXP_TIME_NUM		0x00F2
#define MANUAL_EXP_TIME_DEN		0x00F3
#define MANUAL_DES_EXP_TIME_HI		0x00F4
#define MANUAL_DES_EXP_TIME_LO		0x00F5
#define COLD_START_TIME_HI		0x00F6
#define COLD_START_TIME_LO		0x00F7
#define EXP_COMPENSATION		0x00F8
#define EXP_MISC_SETTINGS		0x00F9
#define DIRECT_MODE_INT_LINES_HI	0x00FA
#define DIRECT_MODE_INT_LINES_LO	0x00FB
#define DIRECT_MODE_INT_PIXELS_HI	0x00FC
#define DIRECT_MODE_INT_PIXELS_LO	0x00FD
#define DIRECT_MODE_ANALOG_GAIN_HI	0x00FE
#define DIRECT_MODE_ANALOG_GAIN_LO	0x00FF
#define DIRECT_MODE_DIGITAL_GAIN_HI	0x0100
#define DIRECT_MODE_DIGITAL_GAIN_LO	0x0101
#define FG_MODE_INT_LINES_HI		0x0102
#define FG_MODE_INT_LINES_LO		0x0103
#define FG_MODE_INT_PIXELS_HI		0x0104
#define FG_MODE_INT_PIXELS_LO		0x0105
#define FG_MODE_ANALOG_GAIN_HI		0x0106
#define FG_MODE_ANALOG_GAIN_LO		0x0107
#define FG_MODE_DIGITAL_GAIN_HI		0x0108
#define FG_MODE_DIGITAL_GAIN_LO		0x0109
#define FREEZE_AUTO_EXP			0x010A
#define USR_MAX_INT_TIME_HI		0x010B
#define USR_MAX_INT_TIME_LO		0x010C
#define REC_FG_THRESH_HI		0x010D
#define REC_FG_THRESH_LO		0x010E
#define ANTI_FLICKER_MODE		0x0110
/* vs6725 exposure algo controls registers */
#define DIGITAL_GAIN_CEILING_HI		0x012E
#define DIGITAL_GAIN_CEILING_LO		0x012F
/* vs6725 exposure status registers */
#define INT_PENDING_LINES_HI		0x0154
#define INT_PENDING_LINES_LO		0x0155
#define INT_PENDING_PIXELS_HI		0x0156
#define INT_PENDING_PIXELS_LO		0x0157
#define ANALOG_GAIN_PENDING_HI		0x0158
#define ANALOG_GAIN_PENDING_LO		0x0159
#define DIGITAL_GAIN_PENDING_HI		0x015A
#define DIGITAL_GAIN_PENDING_LO		0x015B
#define DESIRED_EXP_TIME_HI		0x015C
#define DESIRED_EXP_TIME_LO		0x015D
#define COMPILED_EXP_TIME_HI		0x015E
#define COMPILED_EXP_TIME_LO		0x015F
#define MAX_INT_LINES_HI		0x0161
#define MAX_INT_LINES_LO		0x0162
#define TOTAL_INT_TIME_HI		0x0163
#define TOTAL_INT_TIME_LO		0x0164
#define A_GAIN_PENDING_HI		0x0165
#define A_GAIN_PENDING_LO		0x0166
/* vs6725 flicker detect registers */
#define ENB_DETECT			0x0170
#define DETECT_START			0x0171
#define MAX_NO_ATTEMPT			0x0172
#define FLICKER_ID_THRESH_HI		0x0173
#define FLICKER_ID_THRESH_LO		0x0174
#define WIN_TIMES			0x0175
#define FRAME_RATE_SHIFT_NO		0x0176
#define MANUAL_FREF_ENB			0x0177
#define MANUAL_FREF100_HI		0x0178
#define MANUAL_FREF100_LO		0x0179
#define MANUAL_FREF120_HI		0x017A
#define MANUAL_FREF120_LO		0x017B
#define FLICKER_FREQ_HI			0x017C
#define FLICKER_FREQ_LO			0x017D
/* vs6725 white balance control registers */
#define WB_CTL_MODE			0x0180
#define MANU_RED_GAIN			0x0181
#define MANU_GREEN_GAIN			0x0182
#define MANU_BLUE_GAIN			0x0183
#define WB_MISC_SETTINGS		0x0184
#define HUE_R_BIAS_HI			0x0185
#define HUE_R_BIAS_LO			0x0186
#define HUE_B_BIAS_HI			0x0187
#define HUE_B_BIAS_LO			0x0188
#define FLASH_RED_GAIN_HI		0x0189
#define FLASH_RED_GAIN_LO		0x018A
#define FLASH_GREEN_GAIN_HI		0x018B
#define FLASH_GREEN_GAIN_LO		0x018C
#define FLASH_BLUE_GAIN_HI		0x018D
#define FLASH_BLUE_GAIN_LO		0x018E
/* vs6725 image stability registers */
#define WHITE_BAL_STABLE		0x0211
#define EXP_STABLE			0x0212
#define STABLE				0x0214
/* vs6725 exposure sensor constant registers */
#define SENSOR_A_GAIN_CEILING_HI	0x0232
#define SENSOR_A_GAIN_CEILING_LO	0x0233
/* vs6725 flash control registers */
#define FLASH_MODE			0x0240
#define FLASH_RECOMMENDED		0x0248
/* vs6725 anti-vignettte registers */
#define AV_DISABLE			0x0260
/* vs6725 special effect control registers */
#define NEGATIVE			0x02C0
#define SOLARISING			0x02C1
#define SKETCH				0x02C2
#define COLOUR_EFFECT			0x02C4
/* vs6725 test pattern registers */
#define ENB_TEST_PATTERN		0x04A0
#define TEST_PATTERN			0x04A1

/*
 * low-level hardware registers
 */

/* vs6725 output format registers */
#define OPF_DCTRL			0xD900
#define OPF_YCBCR_SETUP			0xD904
#define OPF_RGB_SETUP			0xD908
/* vs6725 output interface registers */
#define OIF_HSYNC_SETUP			0xDA0C
#define OIF_VSYNC_SETUP			0xDA10
#define OIF_PCLK_SETUP			0xDA30

/* default register values */
#define VS6725_DEF_GAIN			0x0f
#define VS6725_DEF_SAT			0x76
#define VS6725_DEF_BRIGHT		0x64
#define VS6725_DEF_CONTRAST		0x73
#define VS6725_DEF_AECH			0
#define VS6725_DEF_GAMMA		0x14
#define VS6725_DEF_BLUE			0
#define VS6725_DEF_RED			0
#define VS6725_DEF_ZOOM_CONT		0x1
#define VS6725_DEF_ZOOM_REL		0x1

struct vs6725 {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct soc_camera_link *icl;
	int active_pipe;		/* whether pipe 0 or pipe 1 is active */
	struct {
		/* exposure/autoexposure cluster */
		struct v4l2_ctrl *autoexposure;
		struct v4l2_ctrl *exposure;
	};
	struct {
		/* blue/red/autowhitebalance cluster */
		struct v4l2_ctrl *autowb;
		struct v4l2_ctrl *blue;
		struct v4l2_ctrl *red;
	};
	int model;			/* ident codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;		/* sensor cropping window */
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
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
 * formats supported by VS6725
 */
struct vs6725_format {
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;
};

static struct vs6725_format vs6725_formats[] = {
	{
		.mbus_code = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code = V4L2_MBUS_FMT_YVYU8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code = V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code = V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
	{
		.mbus_code = V4L2_MBUS_FMT_RGB565_2X8_BE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	},
};

/* default VS6725 format */
static struct v4l2_mbus_framefmt vs6725_default_fmt = {
	.width = VS6725_MAX_WIDTH - VS6725_MIN_WIDTH,
	.height = VS6725_MAX_HEIGHT - VS6725_MIN_HEIGHT,
	.code = V4L2_MBUS_FMT_YUYV8_2X8,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_JPEG,
};

/* useful in-case you need to write an array of registers */
struct regval_list {
	int reg;
	u8 val;
};

/*
 * Two patches and lots of other shadow register writes are
 * required for correct functioning of VS6725 camera sensor.
 * These values have been obtained from ST. There
 * is really no making sense of most of these.
 *
 * The details of these shadow registers are not available in the
 * VS6725 user manual.
 *
 * Do not change this unless you confirm the effects of new values
 * from ST as the same may cause the sensor to stop working.
 *
 * These settings give a 512*512 VGA YUYV 4:2:2 output at 15fps
 * (note that only Pipe0 is used by default and view-live is
 * turned OFF)
 */
static struct regval_list vs6725_patch1[] = {
	/* hang the mcu */
	{0xffff, 0x01},
	/* setup write to patch ram */
	{0xe000, 0x03},
	/* write patches to patch RAM at address 0x9000 : 33 bytes */
	{0x9000, 0x90}, {0x9001, 0x01}, {0x9002, 0xCA}, {0x9003, 0xE0},
	{0x9004, 0x60}, {0x9005, 0x04}, {0x9006, 0x7F}, {0x9007, 0x00},
	{0x9008, 0x80}, {0x9009, 0x02}, {0x900A, 0x7F}, {0x900B, 0x03},
	{0x900C, 0xEF}, {0x900D, 0x54}, {0x900E, 0x03}, {0x900F, 0xFF},
	{0x9010, 0xC4}, {0x9011, 0x33}, {0x9012, 0x33}, {0x9013, 0x54},
	{0x9014, 0xC0}, {0x9015, 0xFF}, {0x9016, 0x90}, {0x9017, 0xD7},
	{0x9018, 0x0C}, {0x9019, 0xE0}, {0x901A, 0x54}, {0x901B, 0x3F},
	{0x901C, 0x4F}, {0x901D, 0xF0}, {0x901E, 0x02}, {0x901F, 0x4A},
	{0x9020, 0x35},
	/* write patches to patch RAM at address 0x9028 : 19 bytes */
	{0x9028, 0x24}, {0x9029, 0x01}, {0x902A, 0xFE}, {0x902B, 0xE4},
	{0x902C, 0x02}, {0x902D, 0x33}, {0x902E, 0x36}, {0x902F, 0xCF},
	{0x9030, 0x24}, {0x9031, 0xFF}, {0x9032, 0xCF}, {0x9033, 0x34},
	{0x9034, 0xFF}, {0x9035, 0x90}, {0x9036, 0xD9}, {0x9037, 0x2C},
	{0x9038, 0x02}, {0x9039, 0x33}, {0x903A, 0x5A},
	/* write patches to patch RAM at address 0x9048 : 62 bytes */
	{0x9048, 0xA3}, {0x9049, 0xEF}, {0x904A, 0xF0}, {0x904B, 0xEB},
	{0x904C, 0xAE}, {0x904D, 0x02}, {0x904E, 0x78}, {0x904F, 0x02},
	{0x9050, 0xCE}, {0x9051, 0xC3}, {0x9052, 0x13}, {0x9053, 0xCE},
	{0x9054, 0x13}, {0x9055, 0xD8}, {0x9056, 0xF9}, {0x9057, 0xFF},
	{0x9058, 0x90}, {0x9059, 0x00}, {0x905A, 0x36}, {0x905B, 0xEE},
	{0x905C, 0xF0}, {0x905D, 0xA3}, {0x905E, 0xEF}, {0x905F, 0xF0},
	{0x9060, 0xC3}, {0x9061, 0x74}, {0x9062, 0x91}, {0x9063, 0x9F},
	{0x9064, 0xFF}, {0x9065, 0x74}, {0x9066, 0x01}, {0x9067, 0x9E},
	{0x9068, 0xFE}, {0x9069, 0xEF}, {0x906a, 0x25}, {0x906b, 0xE0},
	{0x906c, 0xFF}, {0x906d, 0xEE}, {0x906e, 0x33}, {0x906f, 0x90},
	{0x9070, 0x00}, {0x9071, 0x32}, {0x9072, 0xF0}, {0x9073, 0xA3},
	{0x9074, 0xEF}, {0x9075, 0xF0}, {0x9076, 0x90}, {0x9077, 0x04},
	{0x9078, 0xF8}, {0x9079, 0xE0}, {0x907a, 0xFC}, {0x907b, 0xA3},
	{0x907c, 0xE0}, {0x907d, 0xFD}, {0x907e, 0xAE}, {0x907f, 0x04},
	{0x9080, 0x78}, {0x9081, 0x02}, {0x9082, 0xCE}, {0x9083, 0xC3},
	{0x9084, 0x13}, {0x9085, 0xCE},
	/* write patches to patch RAM at address 0x9086 : 37 bytes */
	{0x9086, 0x13}, {0x9087, 0xD8}, {0x9088, 0xF9}, {0x9089, 0xFF},
	{0x908A, 0x90}, {0x908B, 0x00}, {0x908C, 0x38}, {0x908D, 0xEE},
	{0x908E, 0xF0}, {0x908F, 0xA3}, {0x9090, 0xEF}, {0x9091, 0xF0},
	{0x9092, 0xC3}, {0x9093, 0x74}, {0x9094, 0x2D}, {0x9095, 0x9F},
	{0x9096, 0xFF}, {0x9097, 0x74}, {0x9098, 0x01}, {0x9099, 0x9E},
	{0x909A, 0xFE}, {0x909B, 0xEF}, {0x909C, 0x25}, {0x909D, 0xE0},
	{0x909E, 0xFF}, {0x909F, 0xEE}, {0x90A0, 0x33}, {0x90A1, 0x90},
	{0x90A2, 0x00}, {0x90A3, 0x34}, {0x90A4, 0xF0}, {0x90A5, 0xA3},
	{0x90A6, 0xEF}, {0x90A7, 0xF0}, {0x90A8, 0x02}, {0x90A9, 0x22},
	{0x90AA, 0x0A},
	/* write patches to patch RAM at address 0x90ac : 62 bytes */
	{0x90AC, 0x74}, {0x90AD, 0x3D}, {0x90AE, 0xF0}, {0x90AF, 0xA3},
	{0x90B0, 0x74}, {0x90B1, 0x53}, {0x90B2, 0xF0}, {0x90B3, 0xA3},
	{0x90B4, 0x74}, {0x90B5, 0x3E}, {0x90B6, 0xF0}, {0x90B7, 0xA3},
	{0x90B8, 0x74}, {0x90B9, 0x00}, {0x90BA, 0xF0}, {0x90BB, 0xA3},
	{0x90BC, 0x74}, {0x90BD, 0x3C}, {0x90BE, 0xF0}, {0x90BF, 0xA3},
	{0x90C0, 0x74}, {0x90C1, 0xC9}, {0x90C2, 0x02}, {0x90C3, 0x60},
	{0x90C4, 0x03}, {0x90C5, 0x90}, {0x90C6, 0x01}, {0x90C7, 0xCA},
	{0x90C8, 0xE0}, {0x90C9, 0x60}, {0x90CA, 0x23}, {0x90CB, 0x90},
	{0x90CC, 0x03}, {0x90CD, 0xF0}, {0x90CE, 0xE0}, {0x90CF, 0xFE},
	{0x90D0, 0xA3}, {0x90D1, 0xE0}, {0x90D2, 0xFF}, {0x90D3, 0x12},
	{0x90D4, 0x7C}, {0x90D5, 0x79}, {0x90D6, 0x7B}, {0x90D7, 0x03},
	{0x90D8, 0x90}, {0x90D9, 0x02}, {0x90DA, 0x78}, {0x90DB, 0xE0},
	{0x90DC, 0xFC}, {0x90DD, 0xA3}, {0x90DE, 0xE0}, {0x90DF, 0xFD},
	{0x90e0, 0x12}, {0x90e1, 0x7A}, {0x90e2, 0xBA}, {0x90e3, 0x12},
	{0x90e4, 0x79}, {0x90e5, 0xA5}, {0x90e6, 0xA3}, {0x90e7, 0xEE},
	{0x90e8, 0xF0}, {0x90e9, 0xA3},
	/* write patches to patch RAM at address 0x90ea : 22 bytes */
	{0x90EA, 0xEF}, {0x90EB, 0xF0}, {0x90EC, 0x80}, {0x90ED, 0x0F},
	{0x90EE, 0x90}, {0x90EF, 0x03}, {0x90F0, 0xF0}, {0x90F1, 0xE0},
	{0x90F2, 0xFF}, {0x90F3, 0xA3}, {0x90F4, 0xE0}, {0x90F5, 0x90},
	{0x90F6, 0x02}, {0x90F7, 0x7A}, {0x90F8, 0xCF}, {0x90F9, 0xF0},
	{0x90FA, 0xA3}, {0x90FB, 0xEF}, {0x90FC, 0xF0}, {0x90FD, 0x90},
	{0x90FE, 0x02}, {0x90ff, 0x7A},
	/* write patches to patch RAM at address 0x9100 : 62 bytes */
	{0x9100, 0x02}, {0x9101, 0x4A}, {0x9102, 0x38}, {0x9103, 0x90},
	{0x9104, 0x05}, {0x9105, 0x31}, {0x9106, 0xE0}, {0x9107, 0xFE},
	{0x9108, 0xA3}, {0x9109, 0xE0}, {0x910A, 0xFF}, {0x910B, 0xE4},
	{0x910C, 0xFB}, {0x910D, 0x12}, {0x910E, 0x7A}, {0x910F, 0xBA},
	{0x9110, 0x7B}, {0x9111, 0x05}, {0x9112, 0x7D}, {0x9113, 0x00},
	{0x9114, 0x7C}, {0x9115, 0x02}, {0x9116, 0x12}, {0x9117, 0x7A},
	{0x9118, 0xBA}, {0x9119, 0x90}, {0x911A, 0x05}, {0x911B, 0x31},
	{0x911C, 0xEE}, {0x911D, 0xF0}, {0x911E, 0xA3}, {0x911F, 0xEF},
	{0x9120, 0x00}, {0x9121, 0x02}, {0x9122, 0x03}, {0x9123, 0x2A},
	{0x9124, 0x90}, {0x9125, 0x02}, {0x9126, 0x8E}, {0x9127, 0xEF},
	{0x9128, 0xF0}, {0x9129, 0x02}, {0x912A, 0x55}, {0x912B, 0x68},
	{0x912C, 0x90}, {0x912D, 0x02}, {0x912E, 0x8E}, {0x912F, 0x02},
	{0x9130, 0x73}, {0x9131, 0xBA}, {0x9132, 0x12}, {0x9133, 0x74},
	{0x9134, 0xCB}, {0x9135, 0x90}, {0x9136, 0xC2}, {0x9137, 0xF4},
	{0x9138, 0x74}, {0x9139, 0x01}, {0x913A, 0xF0}, {0x913B, 0x02},
	{0x913C, 0x61}, {0x913D, 0xAF},
	/* write patches to patch RAM at address 0x9140 : 3 bytes */
	{0x9140, 0x02}, {0x9141, 0x4C}, {0x9142, 0xE9},
	/* write patches to patch RAM at address 0x915c : 14 bytes */
	{0x915C, 0xC0}, {0X915D, 0x82}, {0X915E, 0x63}, {0X915F, 0xA2},
	{0X9160, 0x01}, {0X9161, 0xC0}, {0X9162, 0x83}, {0X9163, 0xC0},
	{0X9164, 0x82}, {0X9165, 0xC0}, {0X9166, 0xD0}, {0X9167, 0x02},
	{0X9168, 0x69}, {0X9169, 0xD9},
	/* write patches to patch RAM at address 0x9182 : 30 bytes */
	{0x9182, 0x90}, {0X9183, 0xC3}, {0X9184, 0xD8}, {0X9185, 0x74},
	{0X9186, 0x64}, {0X9187, 0xF0}, {0X9188, 0x90}, {0X9189, 0xC3},
	{0X918A, 0xDC}, {0X918B, 0x74}, {0X918C, 0x13}, {0X918D, 0xF0},
	{0X918E, 0x02}, {0X918F, 0x71}, {0X9190, 0x60}, {0X9191, 0x8E},
	{0X9192, 0x2C}, {0X9193, 0x8F}, {0X9194, 0x2D}, {0X9195, 0x90},
	{0X9196, 0x02}, {0X9197, 0x78}, {0X9198, 0xEE}, {0X9199, 0xF0},
	{0X919A, 0xA3}, {0X919B, 0xEF}, {0X919C, 0xF0}, {0X919D, 0x02},
	{0X919E, 0x73}, {0X919F, 0xE7},
	/* write patches to patch RAM at address 0x92ac : 62 bytes */
	{0x92AC, 0x90}, {0x92AD, 0x03}, {0x92AE, 0xA0}, {0x92AF, 0xE0},
	{0x92B0, 0x70}, {0x92B1, 0x03}, {0x92B2, 0x02}, {0x92B3, 0xC3},
	{0x92B4, 0x7F}, {0x92B5, 0x90}, {0x92B6, 0x03}, {0x92B7, 0xA3},
	{0x92B8, 0xE0}, {0x92B9, 0xFE}, {0x92BA, 0xA3}, {0x92BB, 0xE0},
	{0x92BC, 0xFF}, {0x92BD, 0xC3}, {0x92BE, 0x90}, {0x92BF, 0x04},
	{0x92C0, 0xFE}, {0x92C1, 0xE0}, {0x92C2, 0x9F}, {0x92C3, 0x90},
	{0x92C4, 0x04}, {0x92C5, 0xFD}, {0x92C6, 0xE0}, {0x92C7, 0x9E},
	{0x92C8, 0x50}, {0x92C9, 0x36}, {0x92CA, 0x90}, {0x92CB, 0x03},
	{0x92CC, 0x48}, {0x92CD, 0xE0}, {0x92CE, 0xFF}, {0x92CF, 0x90},
	{0x92D0, 0x03}, {0x92D1, 0x4A}, {0x92D2, 0xE0}, {0x92D3, 0xFD},
	{0x92D4, 0x90}, {0x92D5, 0x04}, {0x92D6, 0xFF}, {0x92D7, 0xE0},
	{0x92D8, 0xFA}, {0x92D9, 0xA3}, {0x92DA, 0xE0}, {0x92DB, 0xFB},
	{0x92DC, 0x12}, {0x92DD, 0xC6}, {0x92DE, 0x80}, {0x92DF, 0x90},
	{0x92E0, 0xCA}, {0x92E1, 0x64}, {0x92E2, 0xEF}, {0x92E3, 0xF0},
	{0x92E4, 0x90}, {0x92E5, 0x03}, {0x92E6, 0x49}, {0x92E7, 0xE0},
	{0x92E8, 0xFF}, {0x92E9, 0x90},
	/* write patches to patch RAM at address 0x92ea : 22 bytes */
	{0x92EA, 0x03}, {0x92EB, 0x4B}, {0x92EC, 0xE0}, {0x92ED, 0xFD},
	{0x92EE, 0x90}, {0x92EF, 0x04}, {0x92F0, 0xFF}, {0x92F1, 0xE0},
	{0x92F2, 0xFA}, {0x92F3, 0xA3}, {0x92F4, 0xE0}, {0x92F5, 0xFB},
	{0x92F6, 0x12}, {0x92F7, 0xC6}, {0x92F8, 0x80}, {0x92F9, 0x90},
	{0x92FA, 0xCA}, {0x92FB, 0x68}, {0x92FC, 0xEF}, {0x92FD, 0xF0},
	{0x92FE, 0x80}, {0x92ff, 0x7F},
	/* write patches to patch RAM at address 0x9300 : 62 bytes */
	{0x9300, 0x90}, {0x9301, 0x03}, {0x9302, 0xA5}, {0x9303, 0xE0},
	{0x9304, 0xFE}, {0x9305, 0xA3}, {0x9306, 0xE0}, {0x9307, 0xFF},
	{0x9308, 0xC3}, {0x9309, 0x90}, {0x930A, 0x04}, {0x930B, 0xFE},
	{0x930C, 0xE0}, {0x930D, 0x9F}, {0x930E, 0x90}, {0x930F, 0x04},
	{0x9310, 0xFD}, {0x9311, 0xE0}, {0x9312, 0x9E}, {0x9313, 0x50},
	{0x9314, 0x36}, {0x9315, 0x90}, {0x9316, 0x03}, {0x9317, 0x4A},
	{0x9318, 0xE0}, {0x9319, 0xFF}, {0x931A, 0x90}, {0x931B, 0x03},
	{0x931C, 0x4C}, {0x931D, 0xE0}, {0x931E, 0xFD}, {0x931F, 0x90},
	{0x9320, 0x04}, {0x9321, 0xFF}, {0x9322, 0xE0}, {0x9323, 0xFA},
	{0x9324, 0xA3}, {0x9325, 0xE0}, {0x9326, 0xFB}, {0x9327, 0x12},
	{0x9328, 0xC6}, {0x9329, 0x80}, {0x932A, 0x90}, {0x932B, 0xCA},
	{0x932C, 0x64}, {0x932D, 0xEF}, {0x932E, 0xF0}, {0x932F, 0x90},
	{0x9330, 0x03}, {0x9331, 0x4B}, {0x9332, 0xE0}, {0x9333, 0xFF},
	{0x9334, 0x90}, {0x9335, 0x03}, {0x9336, 0x4D}, {0x9337, 0xE0},
	{0x9338, 0xFD}, {0x9339, 0x90}, {0x933A, 0x04}, {0x933B, 0xFF},
	{0x933C, 0xE0}, {0x933D, 0xFA},
	/* write patches to patch RAM at address 0x933e : 62 bytes */
	{0x933E, 0xA3}, {0x933F, 0xE0}, {0x9340, 0xFB}, {0x9341, 0x12},
	{0x9342, 0xC6}, {0x9343, 0x80}, {0x9344, 0x90}, {0x9345, 0xCA},
	{0x9346, 0x68}, {0x9347, 0xEF}, {0x9348, 0xF0}, {0x9349, 0x80},
	{0x934A, 0x34}, {0x934B, 0x90}, {0x934C, 0x03}, {0x934D, 0x4C},
	{0x934E, 0xE0}, {0x934F, 0xFF}, {0x9350, 0x90}, {0x9351, 0x03},
	{0x9352, 0x4E}, {0x9353, 0xE0}, {0x9354, 0xFD}, {0x9355, 0x90},
	{0x9356, 0x04}, {0x9357, 0xFF}, {0x9358, 0xE0}, {0x9359, 0xFA},
	{0x935A, 0xA3}, {0x935B, 0xE0}, {0x935C, 0xFB}, {0x935D, 0x12},
	{0x935E, 0xC6}, {0x935F, 0x80}, {0x9360, 0x90}, {0x9361, 0xCA},
	{0x9362, 0x64}, {0x9363, 0xEF}, {0x9364, 0xF0}, {0x9365, 0x90},
	{0x9366, 0x03}, {0x9367, 0x4D}, {0x9368, 0xE0}, {0x9369, 0xFF},
	{0x936A, 0x90}, {0x936B, 0x03}, {0x936C, 0x4F}, {0x936D, 0xE0},
	{0x936E, 0xFD}, {0x936F, 0x90}, {0x9370, 0x04}, {0x9371, 0xFF},
	{0x9372, 0xE0}, {0x9373, 0xFA}, {0x9374, 0xA3}, {0x9375, 0xE0},
	{0x9376, 0xFB}, {0x9377, 0x12}, {0x9378, 0xC6}, {0x9379, 0x80},
	{0x937a, 0x90}, {0x937b, 0xCA},
	/* write patches to patch RAM at address 0x937c : 7 bytes */
	{0x937c, 0x68}, {0X937D, 0xEF}, {0X937E, 0xF0}, {0X937F, 0x12},
	{0X9380, 0x23}, {0X9381, 0x48}, {0X9382, 0x22},
	/* write patches to patch RAM at address 0x938a : 3 bytes */
	{0x938a, 0x02}, {0x938B, 0x61}, {0x938C, 0xEF},
	/* write patches to patch RAM at address 0x9390 : 62 bytes */
	{0x9390, 0x90}, {0x9391, 0x04}, {0x9392, 0x70}, {0x9393, 0x74},
	{0x9394, 0x02}, {0x9395, 0xF0}, {0x9396, 0x90}, {0x9397, 0x04},
	{0x9398, 0x72}, {0x9399, 0x14}, {0x939A, 0xF0}, {0x939B, 0x12},
	{0x939C, 0x64}, {0x939D, 0x4D}, {0x939E, 0x90}, {0x939F, 0x04},
	{0x93A0, 0x73}, {0x93A1, 0x12}, {0x93A2, 0xC4}, {0x93A3, 0xE0},
	{0x93A4, 0x78}, {0x93A5, 0x18}, {0x93A6, 0x12}, {0x93A7, 0xC4},
	{0x93A8, 0xEC}, {0x93A9, 0x90}, {0x93AA, 0x02}, {0x93AB, 0x04},
	{0x93AC, 0xEF}, {0x93AD, 0xF0}, {0x93AE, 0xFB}, {0x93AF, 0x90},
	{0x93B0, 0x04}, {0x93B1, 0x73}, {0x93B2, 0x12}, {0x93B3, 0xC4},
	{0x93B4, 0xE0}, {0x93B5, 0x78}, {0x93B6, 0x10}, {0x93B7, 0x12},
	{0x93B8, 0xC4}, {0x93B9, 0xEC}, {0x93BA, 0x90}, {0x93BB, 0x02},
	{0x93BC, 0x05}, {0x93BD, 0xEF}, {0x93BE, 0xF0}, {0x93BF, 0xFA},
	{0x93C0, 0x90}, {0x93C1, 0x04}, {0x93C2, 0x73}, {0x93C3, 0x12},
	{0x93C4, 0xC4}, {0x93C5, 0xE0}, {0x93C6, 0x78}, {0x93C7, 0x08},
	{0x93C8, 0x12}, {0x93C9, 0xC4}, {0x93CA, 0xEC}, {0x93CB, 0xA9},
	{0x93CC, 0x07}, {0x93CD, 0x90},
	/* write patches to patch RAM at address 0x93ce : 50 bytes */
	{0x93ce, 0x02}, {0x93CF, 0x06}, {0x93D0, 0xE9}, {0x93D1, 0xF0},
	{0x93D2, 0x90}, {0x93D3, 0x04}, {0x93D4, 0x76}, {0x93D5, 0xE0},
	{0x93D6, 0x90}, {0x93D7, 0x02}, {0x93D8, 0x07}, {0x93D9, 0xF0},
	{0x93DA, 0x90}, {0x93DB, 0x04}, {0x93DC, 0x77}, {0x93DD, 0x12},
	{0x93DE, 0xC4}, {0x93DF, 0xE0}, {0x93E0, 0x78}, {0x93E1, 0x18},
	{0x93E2, 0x12}, {0x93E3, 0xC4}, {0x93E4, 0xEC}, {0x93E5, 0x90},
	{0x93E6, 0x02}, {0x93E7, 0x19}, {0x93E8, 0xEF}, {0x93E9, 0xF0},
	{0x93EA, 0x90}, {0x93EB, 0x04}, {0x93EC, 0x77}, {0x93ED, 0x12},
	{0x93EE, 0xC4}, {0x93EF, 0xE0}, {0x93F0, 0x78}, {0x93F1, 0x10},
	{0x93F2, 0x12}, {0x93F3, 0xC4}, {0x93F4, 0xEC}, {0x93F5, 0x90},
	{0x93F6, 0x02}, {0x93F7, 0x1A}, {0x93F8, 0xEF}, {0x93F9, 0xF0},
	{0x93FA, 0x90}, {0x93FB, 0x04}, {0x93FC, 0x77}, {0x93FD, 0x12},
	{0x93FE, 0xC4}, {0x93ff, 0xE0},
	/* write patches to patch RAM at address 0x9400 : 62 bytes */
	{0x9400, 0x78}, {0x9401, 0x08}, {0x9402, 0x12}, {0x9403, 0xC4},
	{0x9404, 0xEC}, {0x9405, 0x90}, {0x9406, 0x02}, {0x9407, 0x1B},
	{0x9408, 0xEF}, {0x9409, 0xF0}, {0x940A, 0x90}, {0x940B, 0x04},
	{0x940C, 0x7A}, {0x940D, 0xE0}, {0x940E, 0xFE}, {0x940F, 0x90},
	{0x9410, 0x02}, {0x9411, 0x1C}, {0x9412, 0xF0}, {0x9413, 0xEB},
	{0x9414, 0x24}, {0x9415, 0xFC}, {0x9416, 0x90}, {0x9417, 0x02},
	{0x9418, 0x50}, {0x9419, 0xF0}, {0x941A, 0xEA}, {0x941B, 0x24},
	{0x941C, 0x10}, {0x941D, 0xA3}, {0x941E, 0xF0}, {0x941F, 0xE9},
	{0x9420, 0x24}, {0X9421, 0xF8}, {0X9422, 0xA3}, {0X9423, 0xF0},
	{0X9424, 0x90}, {0X9425, 0x02}, {0X9426, 0x07}, {0X9427, 0xE0},
	{0X9428, 0x24}, {0X9429, 0x08}, {0X942A, 0x90}, {0X942B, 0x02},
	{0X942C, 0x53}, {0X942D, 0xF0}, {0X942E, 0x90}, {0X942F, 0x02},
	{0X9430, 0x19}, {0X9431, 0xE0}, {0X9432, 0x24}, {0X9433, 0xD8},
	{0X9434, 0x90}, {0X9435, 0x02}, {0X9436, 0x54}, {0X9437, 0xF0},
	{0X9438, 0x90}, {0X9439, 0x02}, {0X943A, 0x1A}, {0X943B, 0xE0},
	{0X943C, 0x24}, {0X943D, 0xF2},
	/* write patches to patch RAM at address 0x943e : 47 bytes */
	{0x943e, 0x90}, {0X943F, 0x02}, {0X9440, 0x55}, {0X9441, 0xF0},
	{0X9442, 0xEF}, {0X9443, 0x24}, {0X9444, 0xEC}, {0X9445, 0xA3},
	{0X9446, 0xF0}, {0X9447, 0xEE}, {0X9448, 0x24}, {0X9449, 0xD2},
	{0X944A, 0xA3}, {0X944B, 0xF0}, {0X944C, 0x02}, {0X944D, 0x6E},
	{0X944E, 0x42}, {0X944F, 0x90}, {0X9450, 0xC0}, {0X9451, 0x24},
	{0X9452, 0xE0}, {0X9453, 0x30}, {0X9454, 0xE0}, {0X9455, 0xF9},
	{0X9456, 0xE0}, {0X9457, 0x44}, {0X9458, 0x04}, {0X9459, 0xF0},
	{0X945A, 0x90}, {0X945B, 0x00}, {0X945C, 0x67}, {0X945D, 0xE0},
	{0X945E, 0x02}, {0X945F, 0x45}, {0X9460, 0xF0}, {0X9461, 0x90},
	{0X9462, 0x02}, {0X9463, 0x04}, {0X9464, 0xE0}, {0X9465, 0x2F},
	{0X9466, 0x90}, {0X9467, 0x02}, {0X9468, 0x50}, {0X9469, 0xF0},
	{0X946A, 0x02}, {0X946B, 0x06}, {0X946C, 0x25},
	/* write patches to patch RAM at address 0x9470 : 12 bytes */
	{0x9470, 0x90}, {0x9471, 0x02}, {0x9472, 0x05}, {0x9473, 0xE0},
	{0x9474, 0x2F}, {0x9475, 0x90}, {0x9476, 0x02}, {0x9477, 0x51},
	{0x9478, 0xF0}, {0x9479, 0x02}, {0x947A, 0x06}, {0x947B, 0x58},
	/* write patches to patch RAM at address 0x9480 : 12 bytes */
	{0x9480, 0x90}, {0x9481, 0x02}, {0x9482, 0x06}, {0x9483, 0xE0},
	{0x9484, 0x2F}, {0x9485, 0x90}, {0x9486, 0x02}, {0x9487, 0x52},
	{0x9488, 0xF0}, {0x9489, 0x02}, {0x948A, 0x06}, {0x948B, 0x8B},
	/* write patches to patch RAM at address 0x9490 : 12 bytes */
	{0x9490, 0x90}, {0x9491, 0x02}, {0x9492, 0x07}, {0x9493, 0xE0},
	{0x9494, 0x2F}, {0x9495, 0x90}, {0x9496, 0x02}, {0x9497, 0x53},
	{0x9498, 0xF0}, {0x9499, 0x02}, {0x949A, 0x06}, {0x949B, 0xBE},
	/* write patches to patch RAM at address 0x94a0 : 12 bytes */
	{0x94a0, 0x90}, {0x94a1, 0x02}, {0x94a2, 0x19}, {0x94a3, 0xE0},
	{0x94a4, 0x2F}, {0x94a5, 0x90}, {0x94a6, 0x02}, {0x94a7, 0x54},
	{0x94a8, 0xF0}, {0x94a9, 0x02}, {0x94aA, 0x06}, {0x94aB, 0xEE},
	/* write patches to patch RAM at address 0x94b0 : 12 bytes */
	{0x94b0, 0x90}, {0x94B1, 0x02}, {0x94B2, 0x1A}, {0x94B3, 0xE0},
	{0x94B4, 0x2F}, {0x94B5, 0x90}, {0x94B6, 0x02}, {0x94B7, 0x55},
	{0x94B8, 0xF0}, {0x94B9, 0x02}, {0x94BA, 0x07}, {0x94BB, 0x21},
	/* write patches to patch RAM at address 0x94c0 : 12 bytes */
	{0x94c0, 0x90}, {0x94c1, 0x02}, {0x94c2, 0x1B}, {0x94c3, 0xE0},
	{0x94c4, 0x2F}, {0x94c5, 0x90}, {0x94c6, 0x02}, {0x94c7, 0x56},
	{0x94c8, 0xF0}, {0x94c9, 0x02}, {0x94cA, 0x07}, {0x94cB, 0x54},
	/* write patches to patch RAM at address 0x94d0 : 12 bytes */
	{0x94d0, 0x90}, {0x94d1, 0x02}, {0x94d2, 0x1C}, {0x94d3, 0xE0},
	{0x94d4, 0x2F}, {0x94d5, 0x90}, {0x94d6, 0x02}, {0x94d7, 0x57},
	{0x94d8, 0xF0}, {0x94d9, 0x02}, {0x94dA, 0x07}, {0x94dB, 0x87},
	/* write patches to patch RAM at address 0x94e0 : 31 bytes */
	{0x94e0, 0xE0}, {0x94e1, 0xFC}, {0x94e2, 0xA3}, {0x94e3, 0xE0},
	{0x94e4, 0xFD}, {0x94e5, 0xA3}, {0x94e6, 0xE0}, {0x94e7, 0xFE},
	{0x94e8, 0xA3}, {0x94e9, 0xE0}, {0x94eA, 0xFF}, {0x94eB, 0x22},
	{0x94eC, 0xE8}, {0x94eD, 0x60}, {0x94eE, 0x0F}, {0x94eF, 0xEC},
	{0x94F0, 0xC3}, {0x94F1, 0x13}, {0x94F2, 0xFC}, {0x94F3, 0xED},
	{0x94F4, 0x13}, {0x94F5, 0xFD}, {0x94F6, 0xEE}, {0x94F7, 0x13},
	{0x94F8, 0xFE}, {0x94F9, 0xEF}, {0x94FA, 0x13}, {0x94FB, 0xFF},
	{0x94FC, 0xD8}, {0x94FD, 0xF1}, {0x94FE, 0x22},
	/* write patches to patch RAM at address 0x9502 : 62 bytes */
	{0x9502, 0x12}, {0x9503, 0x73}, {0x9504, 0xE8}, {0x9505, 0x02},
	{0x9506, 0x62}, {0x9507, 0x2D}, {0x9508, 0x74}, {0x9509, 0x01},
	{0x950A, 0xF0}, {0x950B, 0x90}, {0x950C, 0xC0}, {0x950D, 0x24},
	{0x950E, 0xE0}, {0x950F, 0x44}, {0x9510, 0x02}, {0x9511, 0xF0},
	{0x9512, 0x90}, {0x9513, 0xC0}, {0x9514, 0x24}, {0x9515, 0xE0},
	{0x9516, 0x30}, {0x9517, 0xE0}, {0x9518, 0xF9}, {0x9519, 0xE0},
	{0x951A, 0x44}, {0x951B, 0x04}, {0x951C, 0xF0}, {0x951D, 0x22},
	{0x951E, 0xF0}, {0x951F, 0x90}, {0x9520, 0x04}, {0x9521, 0x70},
	{0x9522, 0x74}, {0x9523, 0x04}, {0x9524, 0xF0}, {0x9525, 0x90},
	{0x9526, 0x04}, {0x9527, 0x72}, {0x9528, 0x74}, {0x9529, 0x01},
	{0x952A, 0xF0}, {0x952B, 0x12}, {0x952C, 0x64}, {0x952D, 0x4D},
	{0x952E, 0x12}, {0x952F, 0xC7}, {0x9530, 0xC4}, {0x9531, 0x90},
	{0x9532, 0x04}, {0x9533, 0x7B}, {0x9534, 0x12}, {0x9535, 0xC4},
	{0x9536, 0xE0}, {0x9537, 0x78}, {0x9538, 0x10}, {0x9539, 0x12},
	{0x953A, 0xC4}, {0x953B, 0xEC}, {0x953C, 0x90}, {0x953D, 0x02},
	{0x953E, 0xB9}, {0x953F, 0xEE},
	/* write patches to patch RAM at address 0x9540 : 62 bytes */
	{0x9540, 0xF0}, {0x9541, 0xFA}, {0x9542, 0xA3}, {0x9543, 0xEF},
	{0x9544, 0xF0}, {0x9545, 0xFB}, {0x9546, 0x90}, {0x9547, 0x04},
	{0x9548, 0x7B}, {0x9549, 0x12}, {0x954A, 0xC4}, {0x954B, 0xE0},
	{0x954C, 0x90}, {0x954D, 0x02}, {0x954E, 0xBB}, {0x954F, 0xEE},
	{0x9550, 0xF0}, {0x9551, 0xA3}, {0x9552, 0xEF}, {0x9553, 0xF0},
	{0x9554, 0x90}, {0x9555, 0x04}, {0x9556, 0x7F}, {0x9557, 0x12},
	{0x9558, 0xC4}, {0x9559, 0xE0}, {0x955A, 0x00}, {0x955B, 0x00},
	{0x955C, 0x00}, {0x955D, 0x00}, {0x955E, 0x00}, {0x955F, 0x90},
	{0x9560, 0x02}, {0x9561, 0xBD}, {0x9562, 0xEE}, {0x9563, 0xF0},
	{0x9564, 0xA3}, {0x9565, 0xEF}, {0x9566, 0xF0}, {0x9567, 0x90},
	{0x9568, 0x02}, {0x9569, 0x22}, {0x956A, 0xE0}, {0x956B, 0xFE},
	{0x956C, 0xA3}, {0x956D, 0xE0}, {0x956E, 0xFF}, {0x956F, 0xAD},
	{0x9570, 0x03}, {0x9571, 0xAC}, {0x9572, 0x02}, {0x9573, 0x7B},
	{0x9574, 0x02}, {0x9575, 0x12}, {0x9576, 0x7A}, {0x9577, 0xBA},
	{0x9578, 0x90}, {0x9579, 0x02}, {0x957A, 0x22}, {0x957B, 0xEE},
	{0x957C, 0xF0}, {0x957D, 0xA3},
	/* write patches to patch RAM at address 0x957e : 62 bytes */
	{0x957e, 0xEF}, {0x957F, 0xF0}, {0x9580, 0xA3}, {0x9581, 0xE0},
	{0x9582, 0xFE}, {0x9583, 0xA3}, {0x9584, 0xE0}, {0x9585, 0xFF},
	{0x9586, 0x90}, {0x9587, 0x02}, {0x9588, 0xBB}, {0x9589, 0xE0},
	{0x958A, 0xFC}, {0x958B, 0xA3}, {0x958C, 0xE0}, {0x958D, 0xFD},
	{0x958E, 0x12}, {0x958F, 0x7A}, {0x9590, 0xBA}, {0x9591, 0x90},
	{0x9592, 0x02}, {0x9593, 0x24}, {0x9594, 0xEE}, {0x9595, 0xF0},
	{0x9596, 0xA3}, {0x9597, 0xEF}, {0x9598, 0xF0}, {0x9599, 0xA3},
	{0x959A, 0xE0}, {0x959B, 0xFE}, {0x959C, 0xA3}, {0x959D, 0xE0},
	{0x959E, 0xFF}, {0x959F, 0x90}, {0x95A0, 0x02}, {0x95A1, 0xBD},
	{0x95A2, 0xE0}, {0x95A3, 0xFC}, {0x95A4, 0xA3}, {0x95A5, 0xE0},
	{0x95A6, 0xFD}, {0x95A7, 0x12}, {0x95A8, 0x7A}, {0x95A9, 0xBA},
	{0x95AA, 0x90}, {0x95AB, 0x02}, {0x95AC, 0x26}, {0x95AD, 0xEE},
	{0x95AE, 0xF0}, {0x95AF, 0xA3}, {0x95B0, 0xEF}, {0x95B1, 0xF0},
	{0x95B2, 0x02}, {0x95B3, 0xC3}, {0x95B4, 0x9E}, {0x95B5, 0x02},
	{0x95B6, 0x62}, {0x95B7, 0x22}, {0x95B8, 0x90}, {0x95B9, 0x05},
	{0x95BA, 0xE0}, {0x95BB, 0xE0},
	/* write patches to patch RAM at address 0x95bc : 62 bytes */
	{0x95bc, 0xFE}, {0x95bD, 0xA3}, {0x95bE, 0xE0}, {0x95bF, 0xFF},
	{0x95C0, 0xA3}, {0X95C1, 0xE0}, {0X95C2, 0xFC}, {0X95C3, 0xA3},
	{0X95C4, 0xE0}, {0X95C5, 0xFD}, {0X95C6, 0xA3}, {0X95C7, 0xE0},
	{0X95C8, 0xFA}, {0X95C9, 0xA3}, {0X95CA, 0xE0}, {0X95CB, 0xFB},
	{0X95CC, 0x90}, {0X95CD, 0x05}, {0X95CE, 0x47}, {0X95CF, 0x74},
	{0X95D0, 0x3E}, {0X95D1, 0xF0}, {0X95D2, 0xA3}, {0X95D3, 0xE4},
	{0X95D4, 0xF0}, {0X95D5, 0x90}, {0X95D6, 0x05}, {0X95D7, 0xE6},
	{0X95D8, 0xE0}, {0X95D9, 0x90}, {0X95DA, 0x05}, {0X95DB, 0x49},
	{0X95DC, 0xF0}, {0X95DD, 0x12}, {0X95DE, 0x2C}, {0X95DF, 0xAB},
	{0X95E0, 0x90}, {0X95E1, 0x05}, {0X95E2, 0x31}, {0X95E3, 0xEE},
	{0X95E4, 0xF0}, {0X95E5, 0xFC}, {0X95E6, 0xA3}, {0X95E7, 0xEF},
	{0X95E8, 0xF0}, {0X95E9, 0xFD}, {0X95EA, 0x90}, {0X95EB, 0x05},
	{0X95EC, 0xE7}, {0X95ED, 0xE0}, {0X95EE, 0xFE}, {0X95EF, 0xA3},
	{0X95F0, 0xE0}, {0X95F1, 0xFF}, {0X95F2, 0x7B}, {0X95F3, 0x02},
	{0X95F4, 0x12}, {0X95F5, 0x7A}, {0X95F6, 0xBA}, {0X95F7, 0x90},
	{0X95F8, 0x01}, {0X95F9, 0xC8},
	/* write patches to patch RAM at address 0x95fa : 6 bytes */
	{0x95fa, 0xEE}, {0x95fB, 0xF0}, {0x95fC, 0xFC}, {0x95fD, 0xA3},
	{0x95fE, 0xEF}, {0x95ff, 0xF0},
	/* write patches to patch RAM at address 0x9600 : 12 bytes */
	{0x9600, 0xFD}, {0x9601, 0x90}, {0x9602, 0x01}, {0x9603, 0xF2},
	{0x9604, 0xE0}, {0x9605, 0xFE}, {0x9606, 0xA3}, {0x9607, 0xE0},
	{0x9608, 0xFF}, {0x9609, 0x02}, {0x960A, 0x03}, {0x960B, 0x3D},
	/* write patches to patch RAM at address 0x9610 : 62 bytes */
	{0x9610, 0xA9}, {0x9611, 0x07}, {0x9612, 0xEC}, {0x9613, 0x29},
	{0x9614, 0xF9}, {0x9615, 0xFF}, {0x9616, 0x12}, {0x9617, 0x77},
	{0x9618, 0x6D}, {0x9619, 0x90}, {0x961A, 0x05}, {0x961B, 0xFD},
	{0x961C, 0xEE}, {0x961D, 0xF0}, {0x961E, 0xA3}, {0x961F, 0xEF},
	{0x9620, 0xF0}, {0x9621, 0x90}, {0x9622, 0x05}, {0x9623, 0xF0},
	{0x9624, 0xE0}, {0x9625, 0xFE}, {0x9626, 0xA3}, {0x9627, 0xE0},
	{0x9628, 0xFF}, {0x9629, 0xA3}, {0x962A, 0xE0}, {0x962B, 0xFA},
	{0x962C, 0xA3}, {0x962D, 0xE0}, {0x962E, 0xFB}, {0x962F, 0xA3},
	{0x9630, 0xE0}, {0x9631, 0xFD}, {0x9632, 0xA3}, {0x9633, 0xE0},
	{0x9634, 0x90}, {0x9635, 0x05}, {0x9636, 0x47}, {0x9637, 0xCD},
	{0x9638, 0xF0}, {0x9639, 0xA3}, {0x963A, 0xED}, {0x963B, 0xF0},
	{0x963C, 0x90}, {0x963D, 0x05}, {0x963E, 0xF6}, {0x963F, 0xE0},
	{0x9640, 0x90}, {0x9641, 0x05}, {0x9642, 0x49}, {0x9643, 0xF0},
	{0x9644, 0x7D}, {0x9645, 0x00}, {0x9646, 0x7C}, {0x9647, 0x3E},
	{0x9648, 0x12}, {0x9649, 0x2C}, {0x964A, 0xAB}, {0x964B, 0x90},
	{0x964C, 0x05}, {0x964D, 0xFA},
	/* write patches to patch RAM at address 0x964e : 35 bytes */
	{0x964e, 0xEE}, {0x964F, 0xF0}, {0x9650, 0xA3}, {0x9651, 0xEF},
	{0x9652, 0xF0}, {0x9653, 0x90}, {0x9654, 0x05}, {0x9655, 0xFD},
	{0x9656, 0xE0}, {0x9657, 0xFC}, {0x9658, 0xA3}, {0x9659, 0xE0},
	{0x965A, 0xFD}, {0x965B, 0x7B}, {0x965C, 0x02}, {0x965D, 0x12},
	{0x965E, 0x7A}, {0x965F, 0xBA}, {0x9660, 0x90}, {0x9661, 0x05},
	{0x9662, 0xFD}, {0x9663, 0xEE}, {0x9664, 0xF0}, {0x9665, 0xA3},
	{0x9666, 0xEF}, {0x9667, 0xF0}, {0x9668, 0x12}, {0x9669, 0x77},
	{0x966A, 0x23}, {0x966B, 0xA9}, {0x966C, 0x07}, {0x966D, 0x00},
	{0x966E, 0x02}, {0x966F, 0x7A}, {0x9670, 0xEB},
	/* write patches to patch RAM at address 0x9680 : 25 bytes */
	{0x9680, 0xAC}, {0x9681, 0x07}, {0x9682, 0xC3}, {0x9683, 0xED},
	{0x9684, 0x9C}, {0x9685, 0xFF}, {0x9686, 0x12}, {0x9687, 0x77},
	{0x9688, 0x6D}, {0x9689, 0xAE}, {0x968A, 0x02}, {0x968B, 0xAF},
	{0x968C, 0x03}, {0x968D, 0x7D}, {0x968E, 0x02}, {0x968F, 0x12},
	{0x9690, 0x7C}, {0x9691, 0x38}, {0x9692, 0x12}, {0x9693, 0x77},
	{0x9694, 0x23}, {0x9695, 0xEC}, {0x9696, 0x2F}, {0x9697, 0xFF},
	{0x9698, 0x22},
	/* write patches to patch RAM at address 0x96a0 : 14 bytes */
	{0x96a0, 0xD0}, {0x96a1, 0xD0}, {0x96a2, 0xD0}, {0x96a3, 0x82},
	{0x96a4, 0xD0}, {0x96a5, 0x83}, {0x96a6, 0x63}, {0x96a7, 0xA2},
	{0x96a8, 0x01}, {0x96a9, 0xD0}, {0x96aA, 0x82}, {0x96aB, 0x02},
	{0x96aC, 0x6A}, {0x96aD, 0x35},
	/* write patches to patch RAM at address 0x97bf : 3 bytes */
	{0x97bf, 0x02}, {0x97C0, 0x5F}, {0x97C1, 0x48},
	/* write patches to patch RAM at address 0x97c4 : 60 bytes */
	{0x97c4, 0x90}, {0x97c5, 0x04}, {0x97c6, 0x7F}, {0x97c7, 0x12},
	{0x97c8, 0xC4}, {0x97c9, 0xE0}, {0x97cA, 0xEF}, {0x97cB, 0x4E},
	{0x97cC, 0x00}, {0x97cD, 0x00}, {0x97cE, 0x70}, {0x97cF, 0x2F},
	{0x97D0, 0x90}, {0x97D1, 0x04}, {0x97D2, 0x72}, {0x97D3, 0x74},
	{0x97D4, 0x1B}, {0x97D5, 0xF0}, {0x97D6, 0x90}, {0x97D7, 0x04},
	{0x97D8, 0x70}, {0x97D9, 0x74}, {0x97DA, 0x04}, {0x97DB, 0xF0},
	{0x97DC, 0x12}, {0x97DD, 0x64}, {0x97DE, 0x4D}, {0x97DF, 0x90},
	{0x97E0, 0x04}, {0x97E1, 0x7B}, {0x97E2, 0x12}, {0x97E3, 0xC4},
	{0x97E4, 0xE0}, {0x97E5, 0xEC}, {0x97E6, 0x4D}, {0x97E7, 0x4E},
	{0x97E8, 0x4F}, {0x97E9, 0x70}, {0x97EA, 0x14}, {0x97EB, 0x90},
	{0x97EC, 0x04}, {0x97ED, 0x7B}, {0x97EE, 0x12}, {0x97EF, 0x12},
	{0x97F0, 0xBA}, {0x97F1, 0x3E}, {0x97F2, 0x68}, {0x97F3, 0x3E},
	{0x97F4, 0x00}, {0x97F5, 0x90}, {0x97F6, 0x04}, {0x97F7, 0x7F},
	{0x97F8, 0x12}, {0x97F9, 0x12}, {0x97FA, 0xBA}, {0x97FB, 0x00},
	{0x97FC, 0x00}, {0x97FD, 0x3E}, {0x97FE, 0xDF}, {0x97ff, 0x22},
	/* write patches to patch RAM at address 0xe004 : 5 bytes */
	{0xe004, 0x4A}, {0xe005, 0x17}, {0xe006, 0x00}, {0xe007, 0x00},
	{0xe008, 0x02},
	/* write patches to patch RAM at address 0xe00A : 5 bytes */
	{0xe00a, 0x03}, {0xe00B, 0x35}, {0xe00C, 0x05}, {0xe00D, 0xB8},
	{0xe00E, 0x02},
	/* write patches to patch RAM at address 0xe010 : 5 bytes */
	{0xe010, 0x45}, {0xe011, 0xEF}, {0xe012, 0x04}, {0xe013, 0x4F},
	{0xe014, 0x02},
	/* write patches to patch RAM at address 0xe058 : 5 bytes */
	{0xe058, 0x4A}, {0xe059, 0x35}, {0xe05A, 0x00}, {0xe05B, 0xC5},
	{0xe05C, 0x02},
	/* write patches to patch RAM at address 0xe05e : 5 bytes */
	{0xe05e, 0x03}, {0xe05F, 0x15}, {0xe060, 0x01}, {0xe061, 0x03},
	{0xe062, 0x02},
	/* write patches to patch RAM at address 0xe064 : 5 bytes */
	{0xe064, 0x55}, {0xe065, 0x67}, {0xe066, 0x01}, {0xe067, 0x24},
	{0xe068, 0x02},
	/* write patches to patch RAM at address 0xe06a : 5 bytes */
	{0xe06a, 0x73}, {0xe06B, 0xB7}, {0xe06C, 0x01}, {0xe06D, 0x2C},
	{0xe06E, 0x02},
	/* write patches to patch RAM at address 0xe070 : 5 bytes */
	{0xe070, 0x4C}, {0xe071, 0xA4}, {0xe072, 0x01}, {0xe073, 0x40},
	{0xe074, 0x02},
	/* write patches to patch RAM at address 0xe076 : 5 bytes */
	{0xe076, 0x22}, {0xe077, 0x07}, {0xe078, 0x00}, {0xe079, 0x48},
	{0xe07A, 0x02},
	/* write patches to patch RAM at address 0xe07c : 5 bytes */
	{0xe07c, 0x69}, {0xe07D, 0xD7}, {0xe07E, 0x01}, {0xe07F, 0x5C},
	{0xe080, 0x02},
	/* write patches to patch RAM at address 0xe082 : 5 bytes */
	{0xe082, 0x33}, {0xe083, 0x32}, {0xe084, 0x00}, {0xe085, 0x28},
	{0xe086, 0x02},
	/* write patches to patch RAM at address 0xe088 : 5 bytes */
	{0xe088, 0x33}, {0xe089, 0x57}, {0xe08A, 0x00}, {0xe08B, 0x2F},
	{0xe08C, 0x02},
	/* write patches to patch RAM at address 0xe08E : 5 bytes */
	{0xe08e, 0x71}, {0xe08F, 0x54}, {0xe090, 0x01}, {0xe091, 0x82},
	{0xe092, 0x02},
	/* write patches to patch RAM at address 0xe094 : 5 bytes */
	{0xe094, 0x73}, {0xe095, 0xE3}, {0xe096, 0x01}, {0xe097, 0x91},
	{0xe098, 0x02},
	/* write patches to patch RAM at address 0xe09a : 5 bytes */
	{0xe09a, 0x6A}, {0xe09B, 0x33}, {0xe09C, 0x06}, {0xe09D, 0xA0},
	{0xe09E, 0x02},
	/* write patches to patch RAM at address 0xe100 : 5 bytes */
	{0xe100, 0x09}, {0xe101, 0x1C}, {0xe102, 0x02}, {0xe103, 0xAC},
	{0xe104, 0x02},
	/* write patches to patch RAM at address 0xe016 : 5 bytes */
	{0xe016, 0x62}, {0xe017, 0x2A}, {0xe018, 0x07}, {0xe019, 0xBF},
	{0xe01A, 0x02},
	/* write patches to patch RAM at address 0xe01C : 5 bytes */
	{0xe01c, 0x06}, {0xe01D, 0x20}, {0xe01E, 0x04}, {0xe01f, 0x61},
	{0xe020, 0x02},
	/* write patches to patch RAM at address 0xe022 : 5 bytes */
	{0xe022, 0x06}, {0xe023, 0x53}, {0xe024, 0x04}, {0xe025, 0x70},
	{0xe026, 0x02},
	/* write patches to patch RAM at address 0xe028 : 5 bytes */
	{0xe028, 0x06}, {0xe029, 0x86}, {0xe02A, 0x04}, {0xe02B, 0x80},
	{0xe02C, 0x02},
	/* write patches to patch RAM at address 0xe02E : 5 bytes */
	{0xe02e, 0x06}, {0xe02F, 0xB9}, {0xe030, 0x04}, {0xe031, 0x90},
	{0xe032, 0x02},
	/* write patches to patch RAM at address 0xe034 : 5 bytes */
	{0xe034, 0x06}, {0xe035, 0xE9}, {0xe036, 0x04}, {0xe037, 0xA0},
	{0xe038, 0x02},
	/* write patches to patch RAM at address 0xe03A : 5 bytes */
	{0xe03a, 0x07}, {0xe03B, 0x1C}, {0xe03C, 0x04}, {0xe03D, 0xB0},
	{0xe03E, 0x02},
	/* write patches to patch RAM at address 0xe040 : 5 bytes */
	{0xe040, 0x07}, {0xe041, 0x4F}, {0xe042, 0x04}, {0xe043, 0xC0},
	{0xe044, 0x02},
	/* write patches to patch RAM at address 0xe046 : 5 bytes */
	{0xe046, 0x07}, {0xe047, 0x82}, {0xe048, 0x04}, {0xe049, 0xD0},
	{0xe04A, 0x02},
	/* write patches to patch RAM at address 0xe04C : 5 bytes */
	{0xe04c, 0x61}, {0xe04D, 0xEC}, {0xe04E, 0x03}, {0xe04F, 0x8A},
	{0xe050, 0x02},
	/* write patches to patch RAM at address 0xe052 : 5 bytes */
	{0xe052, 0x6E}, {0xe053, 0x73}, {0xe054, 0x05}, {0xe055, 0x02},
	{0xe056, 0x02},
	/* write patches to patch RAM at address 0xe106 : 5 bytes */
	{0xe106, 0x62}, {0xe107, 0x1F}, {0xe108, 0x05}, {0xe109, 0xB5},
	{0xe10A, 0x02},
	/* write patches to patch RAM at address 0xe10C : 5 bytes */
	{0xe10c, 0x5F}, {0xe10D, 0xE2}, {0xe10E, 0x05}, {0xe10F, 0x1E},
	{0xe110, 0x02},
	/* write patches to patch RAM at address 0xe112 : 5 bytes */
	{0xe112, 0x61}, {0xe113, 0xAC}, {0xe114, 0x01}, {0xe115, 0x32},
	{0xe116, 0x02},
	/* write patches to patch RAM at address 0xe118 : 5 bytes */
	{0xe118, 0x5F}, {0xe119, 0xEF}, {0xe11A, 0x00}, {0xe11B, 0xAC},
	{0xe11C, 0x02},
	/* write patches to patch RAM at address 0xe11E : 5 bytes */
	{0xe11e, 0x45}, {0xe11F, 0xE8}, {0xe120, 0x05}, {0xe121, 0x08},
	{0xe122, 0x02},
	/* setup execute mode for patch ram */
	{0xe000, 0x01},
	/* reset the MCU */
	{0xffff, 0x00},

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list vs6725_patch2[] = {
	{0xc234, 0x01}, /* Core_Reg enable */
	{OPF_DCTRL, 0x04}, /* DCTRL */
	{E_DIV, 0x01}, /* Set Divider to 1 */
	{MAX_DERATING, 0x10}, /* Set Output Clock DeRating Factor to 16 */

	/* ExposureAlgorithmControls: fpMinimumDesiredExposureTime_us */
	{0x0124, 0x47},
	{0x0125, 0x80},

	/*
	 * SENSOR-MODE	LINE-LENGTH	FRAME-RATE
	 *
	 * UXGA		2010(default)	14.99fps
	 * UXGA		2005		15.03fps
	 * UXGA		2000		15.06fps
	 * ABIN		1980(default})	29.84fps
	 * SVGA		1970		29.99fps
	 * SVGA		1965		30.07fps
	 * SVGA		1960		30.14fps
	 */

	/* VTIMING related */
	{0xC344, 0x07},	/* line_length_req {MSB} */
	{0xC345, 0xda}, /* line_length_req {LSB} */

	/* ExposureAlgorithmControls */
	{DIGITAL_GAIN_CEILING_HI, 0x40}, /* (2.5-> 0x4080) MSB */
	{DIGITAL_GAIN_CEILING_LO, 0x80}, /* (2.5-> 0x4080} LSB */

	/* ArcticControl */
	{0x02d0, 0x00},	/* fInhibitAutomaticMode {VPIP_FALSE} */
	{0x02d3, 0x00},	/* fTightGreenRequest {VPIP_FALSE} */

	/* Set Ring Weight */
	{0x0341, 0x01},	/* ArcticRingWeightControl bMode {DamperOff} */
	{0x0340, 0x40},	/* ArcticRingWeightControl bMaxValue */

	/* SetArcticCenterWeight: 16 */
	{0x0351, 0x01},	/* ArcticCenterWeightControl bMode {DamperOff} */
	{0x0350, 0x10},	/* ArcticCenterWeightControl bMaxValue */

	{0x02D4, 0x00},	/* ArcticControl uwUserFrameSigma {MSB} */
	{0x02D5, 0x0f},	/* ArcticControl uwUserFrameSigma {LSB} */

	/*
	 * Set Arctic
	 * Mode: Digital*Analogue*WB_Average
	 * Day Setting: SigmaWeight of 1 @ Gain: 2
	 * Night Setting: SigmaWeight of 16 @ Gain: 20
	 */
	{0x0321, 0x04},	/* ArcticSigmaWeightControl bMode */
	{0x0320, 0x10},	/* ArcticSigmaWeightControl bMaxValue */
	{0x0322, 0x40},	/* ArcticSigmaWeightControl fpLowThreshold (2) {MSB} */
	{0x0323, 0x00},	/* ArcticSigmaWeightControl fpLowThreshold (2) {LSB} */
	{0x0324, 0x46},	/* ArcticSigmaWeightControl fpHighThreshold (20){MSB} */
	{0x0325, 0x80},	/* ArcticSigmaWeightControl fpHighThreshold (20){LSB} */
	{0x0326, 0x36},	/* ArcticSigmaWeightControl fpFactor (0.0625) {MSB} */
	{0x0327, 0x00},	/* ArcticSigmaWeightControl fpFactor (0.0625) {LSB} */

	/*
	 * Page: Still and VF PeakingGainControl
	 * Mode: Digital*Analogue*GreenGain
	 * Day Setting: 28 @ Digital*Analogue*GreenGain of 1
	 * Night Setting: 0 @ Digital*Analogue*GreenGain of 20
	 */
	{0x0290, 0x03},	/* Still_PeakingGainControl bMode */
	{0x0291, 0x3e},	/* Still_PeakingGainControl fpLowThreshold (1) {MSB} */
	{0x0292, 0x00},	/* Still_PeakingGainControl fpLowThreshold (1) {LSB} */
	{0x0293, 0x46},	/* Still_PeakingGainControl fpHighThreshold (20){MSB} */
	{0x0294, 0x80},	/* Still_PeakingGainControl fpHighThreshold (20){LSB} */
	{0x0295, 0x00},	/* Still_PeakingGainControl fpFactor (0) {MSB} */
	{0x0296, 0x00},	/* Still_PeakingGainControl fpFactor (0) {LSB} */

	/* WhiteBalanceControls fpGainClip (3.5-> 0x4180} */
	{0x018F, 0x41},
	{0x0190, 0x80},

	/*
	 * Set Minimum Weighted White Balance Tilts
	 * Tilt: [2,2,2]
	 */
	{0x03fa, 0x02},	/* MinWeightedWBControls GreenChannelToAccumulate */
	{0x03F2, 0x40},	/* MinWeightedWBControls fpRedTiltGain (2) {MSB} */
	{0x03F3, 0x00},	/* MinWeightedWBControls fpRedTiltGain (2) {LSB} */
	{0x03F4, 0x40},	/* MinWeightedWBControls fpGreen1TiltGain (2) {MSB} */
	{0x03F5, 0x00},	/* MinWeightedWBControls fpGreen1TiltGain (2) {LSB} */
	{0x03F6, 0x40},	/* MinWeightedWBControls fpGreen2TiltGain (2) {MSB} */
	{0x03F7, 0x00},	/* MinWeightedWBControls fpGreen2TiltGain (2) {LSB} */
	{0x03F8, 0x40},	/* MinWeightedWBControls fpBlueTiltGain (2) {MSB} */
	{0x03F9, 0x00},	/* MinWeightedWBControls fpBlueTiltGain (2) {LSB} */

	/*
	 * Set Minimum Weighted White Balance
	 * Disable: VPIP_FALSE
	 * SaturationThreshold: 1000
	 */
	{0x03F0, 0x03},	/* ConvergeTime bNumberOfFramesBefore_StatStable{MSB} */
	{0x03F1, 0xe8},	/* ConvergeTime bNumberOfFramesBefore_StatStable{LSB} */

	/*
	 * Page: ColourMatrixDamper
	 * Mode: Digital*Analogue*WB_Average
	 * Gain: 2 to 20
	 * Factor: 0
	 */
	{0x0270, 0x04},	/* ColourMatrixDamper bMode */
	{0x0271, 0x40},	/* ColourMatrixDamper fpLowThreshold (2) {MSB} */
	{0x0272, 0x00},	/* ColourMatrixDamper fpLowThreshold (2) {LSB} */
	{0x0273, 0x46},	/* ColourMatrixDamper fpHighThreshold (20) {MSB} */
	{0x0274, 0x80},	/* ColourMatrixDamper fpHighThreshold (20) {LSB} */
	{0x0275, 0x00},	/* ColourMatrixDamper fpFactor (0) {MSB} */
	{0x0276, 0x00},	/* ColourMatrixDamper fpFactor (0) {LSB} */

	{0x0229, 0x01},	/* SensorSetup fDisableOffsetDamper {DamperOff} */
	{0x0228, 0x00},	/* SensorSetup bBlackCorrectionOffset */

	/*
	 * Set Scythe
	 * Mode: Digital*Analogue*WB_Average
	 * Day Setting: Effective Scythe of 33 @ 40
	 * Night Setting: Effective Scythe of 24 @ 2
	 */
	{0x0311, 0x04},	/* ArcticScytheWeightControl bMode */
	{0x0310, 0x21},	/* ArcticScytheWeightControl bRegMaxValue */
	{0x0316, 0x3C},	/* ArcticScytheWeightControl fpFactor (0.7273) MSB */
	{0x0317, 0xE9},	/* ArcticScytheWeightControl fpFactor (0.7273) LSB */
	{0x0312, 0x40},	/* ArcticScytheWeightControl fpLowThreshold (2) MSB */
	{0x0313, 0x00},	/* ArcticScytheWeightControl fpLowThreshold (2) LSB */
	{0x0314, 0x48},	/* ArcticScytheWeightControl fpHighThreshold (40) MSB */
	{0x0315, 0x80},	/* ArcticScytheWeightControl fpHighThreshold (40) LSB */

	/*
	 * Set Luma Offset
	 * Offset: 16
	 * Excursion: 271
	 * MidPoint*2: 239
	 */
	{PIPE0_DATA_FORMAT, 0x02},	/* CE0_TransformType_YCbCr_Custom */
	{0x0420, 0x01},	/* CE0_CoderOutputSignalRange uwLumaExcursion MSB */
	{0x0421, 0x0f},	/* CE0_CoderOutputSignalRange uwLumaExcursion LSB */
	{0x0422, 0x00},	/* CE0_CoderOutputSignalRange uwLumaMidpointTimes MSB */
	{0x0423, 0xef},	/* CE0_CoderOutputSignalRange uwLumaMidpointTimes LSB */
	{0x0424, 0x01},	/* CE0_CoderOutputSignalRange uwChromaExcursion MSB */
	{0x0425, 0x00},	/* CE0_CoderOutputSignalRange uwChromaExcursion LSB */
	{0x0426, 0x00},	/* CE0_CoderOutputSignalRange uwChromaMidpoint MSB */
	{0x0427, 0xff},	/* CE0_CoderOutputSignalRange uwChromaMidpoint LSB */

	/*
	 * Set Luma Offset
	 * Offset: 16
	 * Excursion: 271
	 * MidPoint*2: 239
	 */
	{PIPE1_DATA_FORMAT, 0x02},	/* CE1_TransformType_YCbCr_Custom */
	{0x0430, 0x01},	/* CE1_CoderOutputSignalRange uwLumaExcursion MSB */
	{0x0431, 0x0f},	/* CE1_CoderOutputSignalRange uwLumaExcursion LSB */
	{0x0432, 0x00},	/* CE1_CoderOutputSignalRange uwLumaMidpointTimes MSB */
	{0x0433, 0xef},	/* CE1_CoderOutputSignalRange uwLumaMidpointTimes LSB */
	{0x0434, 0x01},	/* CE1_CoderOutputSignalRange uwChromaExcursion MSB */
	{0x0435, 0x00},	/* CE1_CoderOutputSignalRange uwChromaExcursion LSB */
	{0x0436, 0x00},	/* CE1_CoderOutputSignalRange uwChromaMidpoint MSB */
	{0x0437, 0xff},	/* CE1_CoderOutputSignalRange uwChromaMidpoint LSB */

	/* Set Personality Settings for Low Noise */
	{0xc298, 0xA5},	/* PRW_MAN_SPARE pwr_spare0 */
	{0xc212, 0x05},	/* PWR_MAN SETUP_CLK_CP4V_CPNEG_EN */
	{0xc213, 0x00},	/* PWR_MAN SETUP_CLK_CP4V_CPNEG_DIV */
	{0xc3dc, 0x13},	/* VTIMING ANALOG_TIMING_2 */
	{0xc3d8, 0x64},	/* VTIMING ANALOG_TIMING_1 */
	{0xc3b8, 0x0D},	/* VTIMING ANALOG_BTL_LEVEL_SETUP */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_non_gui[] = {
	/*
	 * Page: PeakingCoringControl
	 * Mode: Digital*Analogue*GreenGain
	 * Day Setting: 40 @ Digital*Analogue*GreenGain of 20
	 * Night Setting: 6 @ Digital*Analogue*GreenGain of 2
	 */
	{0x0298, 0x03},	/* PeakingCoringControl bMode */
	{0x0297, 0x26},	/* PeakingCoringControl bMaxValue */
	{0x029A, 0x40},	/* PeakingCoringControl fpLowThreshold (2) {MSB} */
	{0x029B, 0x00},	/* PeakingCoringControl fpLowThreshold (2) {LSB} */
	{0x029C, 0x46},	/* PeakingCoringControl fpHighThreshold (20) {MSB} */
	{0x029D, 0x80},	/* PeakingCoringControl fpHighThreshold (20) {LSB} */
	{0x029E, 0x38},	/* PeakingCoringControl fpFactor (0.150) {MSB} */
	{0x029F, 0x66},	/* PeakingCoringControl fpFactor (0.150) {LSB} */

	/*
	 * Set Gaussian
	 * Mode: Digital*Analogue*WB_Average
	 * Day Setting: Effective Gaussian of 64 @ Combined Gain: 2
	 * Night Setting: Effective Gaussian of 64 @ Combined Gain: 20
	 */
	{0x0331, 0x04},	/* ArcticGaussianWeightControl bMode */
	{0x0330, 0x40},	/* ArcticGaussianWeightControl bMaxValue */
	{0x0336, 0x3E},	/* ArcticGaussianWeightControl fpFactor (1) {MSB} */
	{0x0337, 0x00},	/* ArcticGaussianWeightControl fpFactor (1) {LSB} */
	{0x0332, 0x40},	/* ArcticGaussianWeightControl fpLowThreshold(2){MSB} */
	{0x0333, 0x00},	/* ArcticGaussianWeightControl fpLowThreshold(2){LSB} */
	{0x0334, 0x46},	/* ArcticGaussianWeightControl fpHighThresh(20) {MSB} */
	{0x0335, 0x80},	/* ArcticGaussianWeightControl fpHighThresh(20) {LSB} */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_streaming[] = {
	{USR_MIN_FRAME_RATE, 7},	/* bUserMinimumFrameRate_Hz */
	{USR_MAX_FRAME_RATE, 30},	/* bUserMaximumFrameRate_Hz */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_color[] = {
	/* Pipe0 - VF - GammaR, G, B - 16 */
	{PIPE0_GAMMA_R, 16},
	{PIPE0_GAMMA_G, 16},
	{PIPE0_GAMMA_B, 16},
	/* Pipe1 - VF - GammaR, G, B - 16 */
	{PIPE1_GAMMA_R, 16},
	{PIPE1_GAMMA_G, 16},
	{PIPE1_GAMMA_B, 16},
	/* Set Contrast */
	{CONTRAST, 115},
	/* Set Colour Saturation */
	{COLOR_SATURATION, 100},
	/* Set Brightness */
	{BRIGHTNESS, 105},

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_exposure_ctrl[] = {
	/* Set Anti-Flicker */
	{FLICKER_FREQ_HI, 0x4b},	/* fpFlickerFrequency (100) {MSB} */
	{FLICKER_FREQ_LO, 0x20},	/* fpFlickerFrequency (100) {LSB} */
	/* Set Exposure */
	{METERING, 0x00},		/* ExposureMetering_flat */
	{EXP_COMPENSATION, 0xFF},	/* iExposureCompensation */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_sharpness[] = {
	{0x0297, 40},	/* bUserPeakLoThresh */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_orientation[] = {
	{HORI_MIRROR, 0},	/* fHorizontalMirror */
	{VERT_FLIP, 0},		/* fVerticalFlip */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_before_analog_binnining_off[] = {
	/* Red gain */
	{MANU_RED_GAIN, 0x80},
	/* Green gain */
	{MANU_GREEN_GAIN, 0x80},
	/* Blue gain */
	{MANU_BLUE_GAIN, 0x80},
	/* White balance mode */
	{WB_CTL_MODE, 0x1},
	/* Applying sharpness default for stream 1 */
	{PIPE0_PEAKING_GAIN, 0x5},
	/* Applying sharpness default for stream 2 */
	{PIPE1_PEAKING_GAIN, 0xf},
	/* Magnification factor */
	{ZOOM_SIZE_HI, 0x0},
	{ZOOM_SIZE_LO, 0x1},

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_analog_binnining_off[] = {
	{0xc207, 0x00}, /* Reset HW write required for analogue binning mode */
	{0xc344, 0x07}, /* line length 2010 for normal mode */
	{0xc345, 0xda},
	{0x0201, 0x00}, /* DarkCalMode_LeakyOffset */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_before_auto_frame_rate_on[] = {
	/* Set sensor mode (HiRes or LoRes) */
	{PIPE0_SENSOR_MODE, 0x0},
	{PIPE1_SENSOR_MODE, 0x0},
	/* View/Live setup - Disabled */
	{VIEW_LIVE_EN, 0x0},
	/* Active pipe setup */
	{ACTIVE_PIPE_BANK, 0x0},	/* PIPE_0 is active pipe */
	/* Manual frame rate */
	{DES_FRAME_RATE_NUM_HI, 0x0},	/* uwDesiredFrameRate_Num_hi */
	{DES_FRAME_RATE_NUM_LO, 0xF},	/* uwDesiredFrameRate_Num_lo, 15 fps */
	{DES_FRAME_RATE_DEN, 0x1},	/* bDesiredFrameRate_Den */
	{AUTO_FRAME_MODE, 0x0},	/* bMode {1=Frame Rate Mode Manual} */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_bayer_off[] = {
	/* White Balance Disabled */
	{WB_CTL_MODE, 1},	/* Manual WB */
	/* AV Enable */
	{AV_DISABLE, 0x00},	/* AntiVignetteControl fDisable {VPIP_FALSE} */
	/* Enable Arctic */
	{0x02d1, 0x00},		/* AntiVignetteControl fDisable {VPIP_FALSE} */

	/* end of array */
	{0x0000, 0x00},
};

static struct regval_list default_pre_run_setup[] = {
	/* Analogue Gain Ceiling */
	{0x0234, 0x00},	/* ExpSensorConstants uwSensorAnalogGainCeiling MSB */
	{0x0235, 0xb0},	/* ExpSensorConstants uwSensorAnalogGainCeiling LSB */

	/*
	 * Programmed to page:
	 * sRGBColourMatrix0
	 * 1.581	-0.212	-0.369
	 * -0.643	1.849	-0.206
	 * 0.013	-1.635	2.622
	 */
	{0x0360, 0xb9},	/* sRGBColourMatrix0 fpGInR (-0.212-> 0xB964) {MSB} */
	{0x0361, 0x64},	/* sRGBColourMatrix0 fpGInR (-0.212-> 0xB964) {LSB} */
	{0x0362, 0xba},	/* sRGBColourMatrix0 fpBInR (-0.369-> 0xBAF4) {MSB} */
	{0x0363, 0xf4},	/* sRGBColourMatrix0 fpBInR (-0.369-> 0xBAF4) {LSB} */

	{0x0364, 0xbc},	/* sRGBColourMatrix0 fpRInG (-0.643-> 0xBC92) {MSB} */
	{0x0365, 0x92},	/* sRGBColourMatrix0 fpRInG (-0.643-> 0xBC92) {LSB} */
	{0x0366, 0xb9},	/* sRGBColourMatrix0 fpBInG (-0.206-> 0xB94C) {MSB} */
	{0x0367, 0x4c},	/* sRGBColourMatrix0 fpBInG (-0.206-> 0xB94C) {LSB} */

	{0x0368, 0x31},	/* sRGBColourMatrix0 fpRInB (0.013-> 0x3154) {MSB} */
	{0x0369, 0x54},	/* sRGBColourMatrix0 fpRInB (0.013-> 0x3154) {LSB} */
	{0x036A, 0xbf},	/* sRGBColourMatrix0 fpGInB (-1.64-> 0xBF45) {MSB} */
	{0x036B, 0x45},	/* sRGBColourMatrix0 fpGInB (-1.64-> 0xBF45) {LSB} */

	/*
	 * Programmed to page:
	 * sRGBColourMatrix1
	 * 1.738	-0.491	-0.247
	 * -0.508	1.818	-0.31
	 * 0.047	-1.114	2.067
	 */
	{0x0370, 0xbb},	/* sRGBColourMatrix1 fpGInR (-0.491-> 0xBBEE) {MSB} */
	{0x0371, 0xee},	/* sRGBColourMatrix1 fpGInR (-0.491-> 0xBBEE) {LSB} */
	{0x0372, 0xb9},	/* sRGBColourMatrix1 fpBInR (-0.247-> 0xB9F4) {MSB} */
	{0x0373, 0xf4},	/* sRGBColourMatrix1 fpBInR (-0.247-> 0xB9F4) {LSB} */

	{0x0374, 0xbc},	/* sRGBColourMatrix1 fpRInG (-0.508-> 0xBC08) {MSB} */
	{0x0375, 0x08},	/* sRGBColourMatrix1 fpRInG (-0.508-> 0xBC08) {LSB} */
	{0x0376, 0xba},	/* sRGBColourMatrix1 fpBInG (-0.31-> 0xBA7B) {MSB} */
	{0x0377, 0x7b},	/* sRGBColourMatrix1 fpBInG (-0.31-> 0xBA7B) {LSB} */

	{0x0378, 0x35},	/* sRGBColourMatrix1 fpRInB (0.047-> 0x3502) {MSB} */
	{0x0379, 0x02},	/* sRGBColourMatrix1 fpRInB (0.047-> 0x3502) {LSB} */
	{0x037A, 0xbe},	/* sRGBColourMatrix1 fpGInB (-1.11-> 0xBE3A) {MSB} */
	{0x037B, 0x3a},	/* sRGBColourMatrix1 fpGInB (-1.11-> 0xBE3A) {LSB} */

	/*
	 * Programmed to page:
	 * sRGBColourMatrix2
	 * 2.562	-1.473	-0.089
	 * -0.509	1.712	-0.203
	 * -0.003	-0.721	1.724
	 */
	{0x0380, 0xbe},	/* sRGBColourMatrix2 fpGInR (-1.47-> 0xBEF2) {MSB} */
	{0x0381, 0xf2},	/* sRGBColourMatrix2 fpGInR (-1.47-> 0xBEF2) {LSB} */
	{0x0382, 0xb6},	/* sRGBColourMatrix2 fpBInR (-0.089-> 0xB6D9) {MSB} */
	{0x0383, 0xd9},	/* sRGBColourMatrix2 fpBInR (-0.089-> 0xB6D9) {LSB} */

	{0x0384, 0xbc},	/* sRGBColourMatrix2 fpRInG (-0.509-> 0xBC09) {MSB} */
	{0x0385, 0x09},	/* sRGBColourMatrix2 fpRInG (-0.509-> 0xBC09) {LSB} */
	{0x0386, 0xb9},	/* sRGBColourMatrix2 fpBInG (-0.203-> 0xB93F) {MSB} */
	{0x0387, 0x3f},	/* sRGBColourMatrix2 fpBInG (-0.203-> 0xB93F) {LSB} */

	{0x0388, 0xad},	/* sRGBColourMatrix2 fpRInB (-0.003-> 0xAD12) {MSB} */
	{0x0389, 0x12},	/* sRGBColourMatrix2 fpRInB (-0.003-> 0xAD12) {LSB} */
	{0x038A, 0xbc},	/* sRGBColourMatrix2 fpGInB (-0.721-> 0xBCE2) {MSB} */
	{0x038B, 0xe2},	/* sRGBColourMatrix2 fpGInB (-0.721-> 0xBCE2) {LSB} */

	/*
	 * Programmed to page:
	 * sRGBColourMatrix3
	 * 2.248	-1.048	-0.2
	 * -0.319	1.849	-0.53
	 * 0.002	-0.595	1.593
	 */
	{0x0390, 0xbe},	/* sRGBColourMatrix3 fpGInR (-1.05-> 0xBE19) {MSB} */
	{0x0391, 0x19},	/* sRGBColourMatrix3 fpGInR (-1.05-> 0xBE19) {LSB} */
	{0x0392, 0xb9},	/* sRGBColourMatrix3 fpBInR (-0.2-> 0xB933) {MSB} */
	{0x0393, 0x33},	/* sRGBColourMatrix3 fpBInR (-0.2-> 0xB933) {LSB} */

	{0x0394, 0xba},	/* sRGBColourMatrix3 fpRInG (-0.319-> 0xBA8D) {MSB} */
	{0x0395, 0x8d},	/* sRGBColourMatrix3 fpRInG (-0.319-> 0xBA8D) {LSB} */
	{0x0396, 0xbc},	/* sRGBColourMatrix3 fpBInG (-0.53-> 0xBC1F) {MSB} */
	{0x0397, 0x1f},	/* sRGBColourMatrix3 fpBInG (-0.53-> 0xBC1F) {LSB} */

	{0x0398, 0x2c},	/* sRGBColourMatrix3 fpRInB (0.002-> 0x2C0C) {MSB} */
	{0x0399, 0x0c},	/* sRGBColourMatrix3 fpRInB (0.002-> 0x2C0C) {LSB} */
	{0x039A, 0xbc},	/* sRGBColourMatrix3 fpGInB (-0.595-> 0xBC61) {MSB} */
	{0x039B, 0x61},	/* sRGBColourMatrix3 fpGInB (-0.595-> 0xBC61) {LSB} */

	/*
	 * Adaptive settings
	 * RedNorm's For adaptive colour matrix
	 */
	{0x0277, 0x01},	/* AdaptiveMatrixControl fEnable {VPIP_TRUE} */
	/* MeanHorizonDisribution: 0.174316 Hor */
	{0x036C, 0x38},	/* AdaptiveMatrixControl fpRedRef0 {MSB} */
	{0x036D, 0xca},	/* AdaptiveMatrixControl fpRedRef0 {LSB} */
	/* MeanIncADisribution: 0.217773 IncA */
	{0x037C, 0x39},	/* AdaptiveMatrixControl fpRedRef1 {MSB} */
	{0x037D, 0x7c},	/* AdaptiveMatrixControl fpRedRef1 {LSB} */
	/* MeanCoolWhiteDisribution: 0.305667 Cwf */
	{0x038C, 0x3a},	/* AdaptiveMatrixControl fpRedRef2 {MSB} */
	{0x038D, 0x72},	/* AdaptiveMatrixControl fpRedRef2 {LSB} */
	/* MeanDayLightDisribution: 0.398926 Day */
	{0x039C, 0x3b},	/* AdaptiveMatrixControl fpRedRef3 {MSB} */
	{0x039D, 0x31},	/* AdaptiveMatrixControl fpRedRef3 {LSB} */

	/*
	 * Adaptive settings
	 * RedNorm's for adaptive AV
	 */
	{0x0260, 0x0},	/* AntiVignetteControl fDisable 0 */
	{0x0262, 0x14},	/* AntiVignetteControl r2_shift 20 */
	{0x03a0, 0x1},	/* AdaptiveAntiVignetteControl fEnable 1 */

	{0x03a1, 0x39},	/* fpRedRef0 MSB 0.217773 IncA */
	{0x03a2, 0x7c},	/* fpRedRef0 LSB */
	{0x03a3, 0x3a},	/* fpRedRef1 MSB 0.280273 Desk */
	{0x03a4, 0x3e},	/* fpRedRef1 LSB */
	{0x03a5, 0x3a},	/* fpRedRef2 MSB 0.305667 cwf */
	{0x03a6, 0x72},	/* fpRedRef2 LSB */
	{0x03a7, 0x3b},	/* fpRedRef3 MSB 0.380859 Bright daylight */
	{0x03a8, 0x0c},	/* fpRedRef3 LSB */

	/*
	 * Anti-Vignetting settings
	 * Page: AdaptiveAntiVignetteParameters0
	 */
	{0x03b0, 0xb6},	/* iHorizontalOffsetR	-74 */
	{0x03b1, 0xf2},	/* iVerticalOffsetR	-14 */
	{0x03b2, 0x3c},	/* iR2RCoefficient	 60 */
	{0x03b3, 0xc4},	/* iR4RCoefficient	-60 */
	{0x03b4, 0x0},	/* iHorizontalOffsetGR	0 */
	{0x03b5, 0xdc},	/* iVerticalOffsetGR	-36 */
	{0x03b6, 0x29},	/* iR2GRCoefficient	41 */
	{0x03b7, 0xb8},	/* iR4GRCoefficient	-72 */
	{0x03b8, 0xc2},	/* iHorizontalOffsetGB	-62 */
	{0x03b9, 0xe0},	/* iVerticalOffsetGB	-32 */
	{0x03ba, 0x26},	/* iR2GBCoefficient	38 */
	{0x03bb, 0xc4},	/* iR4GBCoefficient	-60 */
	{0x03bc, 0xf8},	/* iHorizontalOffsetB	-8 */
	{0x03bd, 0xd2},	/* iVerticalOffsetB	-46 */
	{0x03be, 0x22},	/* iR2BCoefficient	34 */
	{0x03bf, 0xc2},	/* iR4BCoefficient	-62 */
	{0x0348, 0x43},	/* bUnityOffset_GR	67 */
	{0x0349, 0x40},	/* bUnityOffset_GB	64 */

	/*
	 * Anti-Vignetting settings
	 * Page: AdaptiveAntiVignetteParameters1
	 */
	{0x03c0, 0xb0},	/* iHorizontalOffsetR	-80 */
	{0x03c1, 0xf0},	/* iVerticalOffsetR	-16 */
	{0x03c2, 0x2e},	/* iR2RCoefficient	 46 */
	{0x03c3, 0xd0},	/* iR4RCoefficient	-48 */
	{0x03c4, 0x6},	/* iHorizontalOffsetGR	 6 */
	{0x03c5, 0xdc},	/* iVerticalOffsetGR	-36 */
	{0x03c6, 0x24},	/* iR2GRCoefficient	 36 */
	{0x03c7, 0xc3},	/* iR4GRCoefficient	-61 */
	{0x03c8, 0xbc},	/* iHorizontalOffsetGB	-68 */
	{0x03c9, 0xe2},	/* iVerticalOffsetGB	-30 */
	{0x03ca, 0x21},	/* iR2GBCoefficient	 33 */
	{0x03cb, 0xcd},	/* iR4GBCoefficient	-51 */
	{0x03cc, 0x2},	/* iHorizontalOffsetB	 2 */
	{0x03cd, 0xda},	/* iVerticalOffsetB	-38 */
	{0x03ce, 0x1e},	/* iR2BCoefficient	 30 */
	{0x03cf, 0xca},	/* iR4BCoefficient	-54 */
	{0x034a, 0x43},	/* bUnityOffset_GR	 67 */
	{0x034b, 0x40},	/* bUnityOffset_GB	 64 */

	/*
	 * Anti-Vignetting settings
	 * Page: AdaptiveAntiVignetteParameters2
	 */
	{0x03d0, 0xb0},	/* iHorizontalOffsetR	-80 */
	{0x03d1, 0xea},	/* iVerticalOffsetR	-22 */
	{0x03d2, 0x2d},	/* iR2RCoefficient	45 */
	{0x03d3, 0xc7},	/* iR4RCoefficient	-57 */
	{0x03d4, 0xa},	/* iHorizontalOffsetGR	10 */
	{0x03d5, 0xd6},	/* iVerticalOffsetGR	-42 */
	{0x03d6, 0x23},	/* iR2GRCoefficient	35 */
	{0x03d7, 0xc4},	/* iR4GRCoefficient	-60 */
	{0x03d8, 0xc0},	/* iHorizontalOffsetGB	-64 */
	{0x03d9, 0xda},	/* iVerticalOffsetGB	-38 */
	{0x03da, 0x20},	/* iR2GBCoefficient	32 */
	{0x03db, 0xcc},	/* iR4GBCoefficient	-52 */
	{0x03dc, 0x12},	/* iHorizontalOffsetB	18 */
	{0x03dd, 0xd6},	/* iVerticalOffsetB	-42 */
	{0x03de, 0x1d},	/* iR2BCoefficient	29 */
	{0x03df, 0xcd},	/* iR4BCoefficient	-51 */
	{0x034c, 0x43},	/* bUnityOffset_GR	67 */
	{0x034d, 0x40},	/* bUnityOffset_GB	64 */

	/*
	 * Anti-Vignetting settings
	 * Page: AdaptiveAntiVignetteParameters3
	 */
	{0x03e0, 0xb2},	/* iHorizontalOffsetR	-78 */
	{0x03e1, 0xec},	/* iVerticalOffsetR	-20 */
	{0x03e2, 0x31},	/* iR2RCoefficient	49 */
	{0x03e3, 0xbd},	/* iR4RCoefficient	-67 */
	{0x03e4, 0x12},	/* iHorizontalOffsetGR	18 */
	{0x03e5, 0xd6},	/* iVerticalOffsetGR	-42 */
	{0x03e6, 0x21},	/* iR2GRCoefficient	33 */
	{0x03e7, 0xc6},	/* iR4GRCoefficient	-58 */
	{0x03e8, 0xc0},	/* iHorizontalOffsetGB	-64 */
	{0x03e9, 0xda},	/* iVerticalOffsetGB	-38 */
	{0x03ea, 0x20},	/* iR2GBCoefficient	32 */
	{0x03eb, 0xcc},	/* iR4GBCoefficient	-52 */
	{0x03ec, 0x20},	/* iHorizontalOffsetB	32 */
	{0x03ed, 0xdc},	/* iVerticalOffsetB	-36 */
	{0x03ee, 0x1c},	/* iR2BCoefficient	28 */
	{0x03ef, 0xd3},	/* iR4BCoefficient	-45 */
	{0x034e, 0x43},	/* bUnityOffset_GR	67 */
	{0x034f, 0x40},	/* bUnityOffset_GB	64 */

	/*
	 * Set Contrained WhiteBalance
	 * LocusA = [0.25454, 0.50352]
	 * LocusB = [0.41075, 0.32418]
	 * CT Range: [3000, 7400]
	 */
	{0x01ca, 0x01},	/* WhiteBalanceConstrainerControls fEnable VPIP_TRUE */
	{0x01C4, 0x3b},	/* WhiteBalanceConstrainerControls fpRedB (0.411) MSB */
	{0x01C5, 0x49},	/* WhiteBalanceConstrainerControls fpRedB (0.411) LSB */
	{0x01C6, 0x3a},	/* WhiteBalanceConstrainerControls fpBlueB (0.324)MSB */
	{0x01C7, 0x98},	/* WhiteBalanceConstrainerControls fpBlueB (0.324)LSB */
	{0x01C0, 0x3a},	/* WhiteBalanceConstrainerControls fpRedA (0.255) MSB */
	{0x01C1, 0x09},	/* WhiteBalanceConstrainerControls fpRedA (0.255) LSB */
	{0x01C2, 0x3c},	/* WhiteBalanceConstrainerControls fpBlueA (0.504)MSB */
	{0x01C3, 0x04},	/* WhiteBalanceConstrainerControls fpBlueA (0.504)LSB */

	/* Cold Start Gains: [1.2451, 1, 1.7019] - 4000k */
	{0x05C3, 0x3e},	/* WhiteBalanceColdStart fpRedGain (1.25) MSB */
	{0x05C4, 0x7e},	/* WhiteBalanceColdStart fpRedGain (1.25) LSB */
	{0x05C5, 0x3e},	/* WhiteBalanceColdStart fpGreenGain (1) MSB */
	{0x05C6, 0x00},	/* WhiteBalanceColdStart fpGreenGain (1) LSB */
	{0x05C7, 0x3f},	/* WhiteBalanceColdStart fpBlueGain (1.7) MSB */
	{0x05C8, 0x67},	/* WhiteBalanceColdStart fpBlueGain (1.7) LSB */

	/* Dynamic Distance From Locus */
	{0x05E0, 0x56},	/* DynamicDistanceFromLocus fpLowThreshold(6e+003)MSB */
	{0x05E1, 0xee},	/* DynamicDistanceFromLocus fpLowThreshold(6e+003)LSB */
	{0x05E2, 0x3a},	/* DynamicDistanceFromLocus fpMinimumDampOP(0.35)MSB */
	{0x05E3, 0xcd},	/* DynamicDistanceFromLocus fpMinimumDampOP(0.35)LSB */
	{0x05E4, 0x58},	/* DynamicDistanceFromLocus fpHighThresh(1.1e+004)MSB */
	{0x05E5, 0xb0},	/* DynamicDistanceFromLocus fpHighThresh(1.1e+004)LSB */
	{0x05e6, 0x05},	/* DynamicDistanceFromLocus fDisable CompiledExposure */
	{0x05E7, 0x31},	/* DynamicDistanceFromLocus fpMaxValue (0.015) MSB */
	{0x05E8, 0xd7},	/* DynamicDistanceFromLocus fpMaxValue (0.015) LSB */

	/* Black Offset Damper */
	{0x0228, 0xFb},	/* SensorSetup bBlackCorrectionOffset */
	{0x0229, 0x04},	/* SensorSetup fDisableOffsetDamper */
	{0x022A, 0x3e},	/* SensorSetup fpOffsetDamperLowThreshold (1) MSB */
	{0x022B, 0x00},	/* SensorSetup fpOffsetDamperLowThreshold (1) LSB */
	{0x022C, 0x48},	/* SensorSetup fpOffsetDamperHighThreshold (40) MSB */
	{0x022D, 0x80},	/* SensorSetup fpOffsetDamperHighThreshold (40) LSB */
	{0x022E, 0x00},	/* SensorSetup fpMinimumOffsetDamperOutput (0) MSB */
	{0x022F, 0x00},	/* SensorSetup fpMinimumOffsetDamperOutput (0) LSB */
	{0x0230, 0x3e},	/* SensorSetup fpMaximumOffsetDamperOutput (1) MSB */
	{0x0231, 0x00},	/* SensorSetup fpMaximumOffsetDamperOutput (1) LSB */

	/*
	 * Fade to Black setup
	 * Fades pixels to black in very low light
	 */
	{0x02b0, 0x05},	/* Enable Fade to Black */
	{0x02b3, 0x6b},	/* Damper low threshold (7 lux at 7 fps) */
	{0x02b4, 0xd1},	/* LSB */
	{0x02b5, 0x6c},	/* Damper high threshold (4 lux at 7 fps) */
	{0x02b6, 0x6f},	/* LSB */

	/*
	 * Adjust NR settings
	 * Increase scythe to 75 % of max in good light
	 */
	{0x0316, 0x3d},	/* ArcticScytheWeightControl fpFactor (0.75) MSB */
	{0x0317, 0x00},	/* ArcticScytheWeightControl fpFactor (0.75) LSB */

	/* end of array */
	{0x0000, 0x00},
};

static int vs6725_prog_default(struct i2c_client *client);

/* read a register */
static int vs6725_reg_read(struct i2c_client *client, int reg, u8 *val)
{
	int ret;
	u8 data[2] = {reg >> 8, reg};
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = &data[0],
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a register */
static int vs6725_reg_write(struct i2c_client *client, int reg, u8 val)
{
	int ret;
	u8 data[3] = {reg >> 8, reg, val};
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = &data[0],
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	udelay(100);

	if (ret != 1) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/* write a set of sequential registers */
static int vs6725_reg_write_multiple(struct i2c_client *client,
					struct regval_list *reg_list)
{
	int ret;

	while (reg_list->reg != 0x0000) {
		ret = vs6725_reg_write(client, reg_list->reg,
						reg_list->val);
		if (ret < 0)
			return ret;

		reg_list++;
	}

	/* wait for the changes to actually happen */
	mdelay(50);

	return 0;
}

static struct vs6725 *to_vs6725(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct vs6725, subdev);
}

/* Alter bus settings on camera side */
static int vs6725_s_mbus_config(struct v4l2_subdev *sd,
					const struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_link *icl = soc_camera_i2c_to_link(client);
	unsigned long flags = soc_camera_apply_board_flags(icl, cfg);
	int ret;

	ret = vs6725_reg_write(client,
			OIF_PCLK_SETUP,
			0xFE | PCLK_PROG_POL_HI_INIT_LO);
	if (ret)
		return ret;
	ret = vs6725_reg_write(client,
				OIF_HSYNC_SETUP,
				0x0C | HSYNC_ENABLE | HSYNC_POLARITY_ACTIVE_LO);
	if (ret)
		return ret;
	ret = vs6725_reg_write(client,
				OIF_VSYNC_SETUP,
				0x0C | VSYNC_ENABLE | VSYNC_POLARITY_ACTIVE_LO);

	return ret;
}

/* Request bus settings on camera side */
static int vs6725_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_link *icl = soc_camera_i2c_to_link(client);

	/*
	 * FIXME: Limit mbus configs as per what is working now.
	 * Note that the sensor supports a wider set of configs.
	 * Add support for the same later.
	 */
	cfg->flags = V4L2_MBUS_MASTER | V4L2_MBUS_PCLK_SAMPLE_FALLING |
		V4L2_MBUS_HSYNC_ACTIVE_HIGH | V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = soc_camera_apply_board_flags(icl, cfg);

	return 0;
}

/*
 * After VS6725 is enabled by pulling CE pin HI,
 * we need to write the patches and default registers to
 * ensure that the sensor is in a correct default state.
 */
static int vs6725_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv;
	int ret;

	if (!client)
		/*
		 * VS6725 probe has not been invoked as of now,
		 * so fake a correct reply
		 */
		return 0;

	priv = to_vs6725(client);

	if (!priv || !priv->icl)
		/*
		 * VS6725 probe has not been invoked as of now,
		 * so fake a correct reply
		 */
		return 0;

	if (!on) {
		/* power-off the sensor */
		if (priv->icl->power) {
			ret = priv->icl->power(&client->dev, 0);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed to power-off the camera.\n");
				goto out;
			}
		}
	} else {
		/* power-on the sensor */
		if (priv->icl->power) {
			ret = priv->icl->power(&client->dev, 1);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed to power-on the camera.\n");
				goto out;
			}
		}

		/* add delay as defined in specs */
		mdelay(100);

		/* program sensor specific patches/quirks */
		ret = vs6725_prog_default(client);
		if (ret) {
			dev_err(&client->dev,
				"VS6725 default register program failed\n");
			goto out;
		}
	}

	return 0;
out:
	return ret;
}

/* Get status of additional camera capabilities */
static int vs6725_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vs6725 *priv = container_of(ctrl->handler, struct vs6725, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = vs6725_reg_read(client, MANU_RED_GAIN, &val);
		if (!ret) {
			priv->red->val = val;
			ret = vs6725_reg_read(client, MANU_BLUE_GAIN, &val);
		}
		if (!ret)
			priv->blue->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = vs6725_reg_read(client, EXP_CTRL_MODE, &val);
		if (!ret)
			priv->exposure->val = val;
		break;
	}

	return ret;
}

/* set some particular settings of a control on Vs6725 */
static int vs6725_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vs6725 *priv = container_of(ctrl->handler, struct vs6725, hdl);
	struct v4l2_subdev *sd = &priv->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = vs6725_reg_write(client, BRIGHTNESS, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = vs6725_reg_write(client, CONTRAST, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = vs6725_reg_write(client, COLOR_SATURATION, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		ret = vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_PEAKING_GAIN :
			PIPE1_PEAKING_GAIN,
			ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = vs6725_reg_write(client, WB_CTL_MODE, ctrl->val);
		break;
	/* FIXME: Exposure related stuff to be formalized */
	case V4L2_CID_EXPOSURE_AUTO:
		switch (ctrl->val) {
		case V4L2_EXPOSURE_AUTO:
			ret = vs6725_reg_write(client, EXP_CTRL_MODE,
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
					EXP_CTRL_MODE,
					DIRECT_MANUAL_MODE);
			break;
		}
		break;
	case V4L2_CID_GAMMA:
		/*
		 * FIXME: not clear if we need to set the same GAMMA gain of
		 * the RED, BLUE and GREEN channels here. Verify with
		 * sensor designers.
		 */
		ret = vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_GAMMA_R :
				PIPE1_GAMMA_R,
			ctrl->val);
		if (!ret)
			ret = vs6725_reg_write(client,
				priv->active_pipe == PIPE_0 ? PIPE0_GAMMA_G :
					PIPE1_GAMMA_G,
				ctrl->val);
		if (!ret)
			ret = vs6725_reg_write(client,
				priv->active_pipe == PIPE_0 ? PIPE0_GAMMA_B :
					PIPE1_GAMMA_B,
				ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = vs6725_reg_write(client, VERT_FLIP, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = vs6725_reg_write(client, HORI_MIRROR, ctrl->val);
		break;
	case V4L2_CID_ZOOM_RELATIVE:
		/*
		 * use the default step size of 5% for every relative zoom
		 * operation
		 */
		ret = vs6725_reg_write(client,
				ZOOM_SIZE_HI,
				DEFAULT_ZOOM_STEP_SIZE_HI);
		if (!ret)
			ret = vs6725_reg_write(client,
				ZOOM_SIZE_LO,
				DEFAULT_ZOOM_STEP_SIZE_LO);
		if (!ret) {
			/*
			 * negative values move the zoom lens towards the
			 * wide-angle direction, whereas positive values move
			 * the zoom lens group towards the telephoto direction
			 */
			if (ctrl->val < 0) {
				ret = vs6725_reg_write(client,
					ZOOM_CTRL,
					ZOOM_STEP_IN);
			} else {
				ret = vs6725_reg_write(client,
					ZOOM_CTRL,
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
		if (ctrl->val < 0)
			ret = vs6725_reg_write(client,
				ZOOM_CTRL,
				ZOOM_START_OUT);
		else if (ctrl->val > 0)
			ret = vs6725_reg_write(client,
				ZOOM_CTRL,
				ZOOM_START_IN);
		else
			ret = vs6725_reg_write(client,
				ZOOM_CTRL,
				ZOOM_STOP);

		if (ret != 0)
			dev_err(&client->dev,
				"zoom-continuous operation failed\n");
		break;
	case V4L2_CID_COLORFX:
		switch (ctrl->val) {
		case V4L2_COLORFX_NONE:
			ret = vs6725_reg_write(client,
				COLOUR_EFFECT,
				COLOR_EFFECT_NORMAL);
			break;
		case V4L2_COLORFX_BW:
			ret = vs6725_reg_write(client,
				COLOUR_EFFECT,
				COLOR_EFFECT_BLACKNWHITE);
			break;
		case V4L2_COLORFX_SEPIA:
			ret = vs6725_reg_write(client,
				COLOUR_EFFECT,
				COLOR_EFFECT_SEPIA);
			break;
		case V4L2_COLORFX_NEGATIVE:
			ret = vs6725_reg_write(client,
				NEGATIVE, 1);
			break;
		case V4L2_COLORFX_SKETCH:
			ret = vs6725_reg_write(client,
				SKETCH, 1);
			break;
		default:
			/*
			 * Note that VS6725 sensor does not
			 * support other V4L2_COLORFX_* effects.
			 */
			dev_warn(&client->dev, "color format not supported\n");
			ret = -EINVAL;
			break;
		}
		break;
	default:
		/* we don't support this ctrl id */
		ret = -EINVAL;
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

	reg->size = 2;

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

	if (enable)
		ret = vs6725_reg_write(client, USER_CMD, CMD_RUN);
	else
		ret = vs6725_reg_write(client, USER_CMD, CMD_STOP);

	if (ret != 0)
		return -EIO;

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

	/* FIXME: the datasheet doesn't specify minimum sizes */
	soc_camera_limit_side(&rect->left, &rect->width,
			VS6725_COLUMN_SKIP, VS6725_MIN_WIDTH, VS6725_MAX_WIDTH);
	soc_camera_limit_side(&rect->top, &rect->height,
			VS6725_ROW_SKIP, VS6725_MIN_HEIGHT, VS6725_MAX_HEIGHT);

	/* cropping means we set the image size manually */
	ret = vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_IMAGE_SIZE :
				PIPE1_IMAGE_SIZE,
			IMAGE_SIZE_MANUAL);
	if (!ret) {
		priv->rect.left = rect->left;
		priv->rect.width = rect->width;
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_MANUAL_HS_LO :
				PIPE1_MANUAL_HS_LO,
			WRITE_LO_BYTE(rect->left + rect->width));
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_MANUAL_HS_HI :
				PIPE1_MANUAL_HS_HI,
			WRITE_HI_BYTE(rect->left + rect->width));
	}

	if (!ret) {
		priv->rect.height = rect->height;
		priv->rect.top = rect->top;
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_MANUAL_VS_LO :
				PIPE1_MANUAL_VS_LO,
			WRITE_LO_BYTE(rect->top + rect->height));
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_MANUAL_VS_HI :
				PIPE1_MANUAL_VS_HI,
			WRITE_HI_BYTE(rect->top + rect->height));
	}

	return ret;
}

static int vs6725_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = priv->frame_rate.denominator;
	cp->timeperframe.denominator = priv->frame_rate.numerator;

	return 0;
}

static int vs6725_s_parm(struct v4l2_subdev *sd,
				struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);
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

	/*
	 * numerator and denominator conventions used by VS6725 and v4l2
	 * are complimentary to each other.
	 */
	priv->frame_rate.numerator = tpf->denominator;
	priv->frame_rate.denominator = tpf->numerator;

	/* manual frame rate settings */
	vs6725_reg_write(client, AUTO_FRAME_MODE, MANUAL_FRAME_RATE);
	vs6725_reg_write(client, DES_FRAME_RATE_NUM_HI,
			WRITE_HI_BYTE(priv->frame_rate.numerator));
	vs6725_reg_write(client, DES_FRAME_RATE_NUM_LO,
			WRITE_LO_BYTE(priv->frame_rate.numerator));
	vs6725_reg_write(client, DES_FRAME_RATE_DEN,
			WRITE_LO_BYTE(priv->frame_rate.denominator));

	return 0;
}

/* get the format we are currently capturing in */
static int vs6725_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	*mf = priv->fmt;

	return 0;
}

/*
 * Try the format provided by application, without really making any
 * hardware settings.
 */
static int vs6725_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(vs6725_formats); index++)
		if (vs6725_formats[index].mbus_code == mf->code)
			break;

	if (index >= ARRAY_SIZE(vs6725_formats))
		/* fallback to the default format */
		index = 0;

	mf->code = vs6725_formats[index].mbus_code;
	mf->colorspace = vs6725_formats[index].colorspace;
	mf->field = V4L2_FIELD_NONE;

	/* check for upper resolution boundary */
	if (mf->width > UXGA_WIDTH)
		mf->width = UXGA_WIDTH;

	if (mf->height > UXGA_HEIGHT)
		mf->height = UXGA_HEIGHT;

	return 0;
}

/* set sensor image format */
static int vs6725_set_image_format(struct i2c_client *client,
					struct v4l2_mbus_framefmt *mf)
{
	struct vs6725 *priv = to_vs6725(client);
	int ret = 0;

	switch (mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_YCBCR_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_YCBCR_SETUP,
				YCBYCR_DATA_SEQUENCE);
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_YCBCR_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_YCBCR_SETUP,
				CBYCRY_DATA_SEQUENCE);
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_YCBCR_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_YCBCR_SETUP,
				YCRYCB_DATA_SEQUENCE);
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_YCBCR_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_YCBCR_SETUP,
				CRYCBY_DATA_SEQUENCE);
		break;
	case V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_RGB_444_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_RGB_SETUP,
				RGB_FLIP_SHIFT(RGB_DATA_SEQUENCE) |
				RGB444_ZERO_PADDING_ON);
		break;
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_RGB_565_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_RGB_SETUP,
				RGB_FLIP_SHIFT(RGB_DATA_SEQUENCE));
		break;
	default:
		/*
		 * V4L2 specs state that we should _not_ return
		 * error in case we do not support a particular format
		 * and instead should apply the default settings and
		 * return.
		 */
		dev_err(&client->dev, "Pixel format not handled: 0x%x\n"
			"Reverting to default YVYU8 format\n", mf->code);
		ret |= vs6725_reg_write(client,
			priv->active_pipe == PIPE_0 ? PIPE0_DATA_FORMAT :
				PIPE1_DATA_FORMAT,
				DATA_FORMAT_YCBCR_CUSTOM);
		ret |= vs6725_reg_write(client,
				OPF_YCBCR_SETUP,
				YCBYCR_DATA_SEQUENCE);
		break;
	}

	return ret;
}

/* set sensor image size */
static int vs6725_set_image_size(struct i2c_client *client,
					struct v4l2_mbus_framefmt *mf)
{
	struct vs6725 *priv = to_vs6725(client);
	int ret = 0;

	if ((mf->width == UXGA_WIDTH) && (mf->height == UXGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_UXGA);
	else if ((mf->width == SXGA_WIDTH) &&
			(mf->height == SXGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_SXGA);
	else if ((mf->width == SVGA_WIDTH) &&
			(mf->height == SVGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_SVGA);
	else if ((mf->width == VGA_WIDTH) &&
			(mf->height == VGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_VGA);
	else if ((mf->width == CIF_WIDTH) &&
			(mf->height == CIF_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_CIF);
	else if ((mf->width == QVGA_WIDTH) &&
			(mf->height == QVGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_QVGA);
	else if ((mf->width == QCIF_WIDTH) &&
			(mf->height == QCIF_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_QCIF);
	else if ((mf->width == QQVGA_WIDTH) &&
			(mf->height == QQVGA_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_QQVGA);
	else if ((mf->width == QQCIF_WIDTH) &&
			(mf->height == QQCIF_HEIGHT))
		ret = vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_QQCIF);
	else {
		/* manual size */
		ret |= vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_IMAGE_SIZE : PIPE1_IMAGE_SIZE,
				IMAGE_SIZE_MANUAL);
		ret |= vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_MANUAL_HS_HI : PIPE1_MANUAL_HS_HI,
				WRITE_HI_BYTE(mf->width));
		ret |= vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_MANUAL_HS_LO : PIPE1_MANUAL_HS_LO,
				WRITE_LO_BYTE(mf->width));
		ret |= vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_MANUAL_VS_HI : PIPE1_MANUAL_VS_HI,
				WRITE_HI_BYTE(mf->height));
		ret |= vs6725_reg_write(client,
				(priv->active_pipe == PIPE_0) ?
				PIPE0_MANUAL_VS_LO : PIPE1_MANUAL_VS_LO,
				WRITE_LO_BYTE(mf->height));
	}

	return ret;
}

/* set the format we will capture in */
static int vs6725_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vs6725 *priv = to_vs6725(client);

	ret = vs6725_try_fmt(sd, mf);
	if (ret)
		return ret;

	/* set image format */
	if (!ret)
		ret = vs6725_set_image_format(client, mf);

	/* set image size */
	if (!ret)
		ret = vs6725_set_image_size(client, mf);

	if (!ret)
		priv->fmt = *mf;

	return ret;
}

static int vs6725_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(vs6725_formats))
		return -EINVAL;

	*code = vs6725_formats[index].mbus_code;
	return 0;
}

static const struct v4l2_ctrl_ops vs6725_ctrl_ops = {
	.g_volatile_ctrl = vs6725_g_volatile_ctrl,
	.s_ctrl = vs6725_s_ctrl,
};

static struct v4l2_subdev_core_ops vs6725_subdev_core_ops = {
	.s_power = vs6725_s_power,
	.g_chip_ident = vs6725_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = vs6725_get_register,
	.s_register = vs6725_set_register,
#endif
};

static struct v4l2_subdev_video_ops vs6725_video_ops = {
	.s_stream = vs6725_s_stream,
	.s_parm = vs6725_s_parm,
	.g_parm = vs6725_g_parm,
	.g_mbus_fmt = vs6725_g_fmt,
	.s_mbus_fmt = vs6725_s_fmt,
	.try_mbus_fmt = vs6725_try_fmt,
	.enum_mbus_fmt = vs6725_enum_fmt,
	.cropcap = vs6725_cropcap,
	.g_crop = vs6725_g_crop,
	.s_crop	= vs6725_s_crop,
	.g_mbus_config = vs6725_g_mbus_config,
	.s_mbus_config = vs6725_s_mbus_config,
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
	int ret = 0;

	ret |= vs6725_reg_write_multiple(client, vs6725_patch1);
	ret |= vs6725_reg_write_multiple(client, vs6725_patch2);
	ret |= vs6725_reg_write_multiple(client, default_non_gui);
	ret |= vs6725_reg_write_multiple(client, default_streaming);
	ret |= vs6725_reg_write_multiple(client, default_color);
	ret |= vs6725_reg_write_multiple(client, default_exposure_ctrl);
	ret |= vs6725_reg_write_multiple(client, default_sharpness);
	ret |= vs6725_reg_write_multiple(client, default_orientation);
	ret |= vs6725_reg_write_multiple(client,
					default_before_analog_binnining_off);
	ret |= vs6725_reg_write_multiple(client, default_analog_binnining_off);
	ret |= vs6725_reg_write_multiple(client,
			default_before_auto_frame_rate_on);
	ret |= vs6725_reg_write_multiple(client, default_bayer_off);
	ret |= vs6725_reg_write_multiple(client, default_pre_run_setup);

	return ret;
}

static int vs6725_camera_init(struct i2c_client *client)
{
	int ret = 0;
	unsigned char dev_id_hi, dev_id_lo;
	unsigned char fm_ver, patch_ver;
	struct vs6725 *priv = to_vs6725(client);

	/*
	 * check and show device id, firmware version and patch version
	 */
	ret = vs6725_reg_read(client, DEVICE_ID_HI, &dev_id_hi);
	if (!ret)
		ret = vs6725_reg_read(client, DEVICE_ID_LO, &dev_id_lo);
	if (!ret)
		ret = vs6725_reg_read(client, FM_VERSION, &fm_ver);
	if (!ret)
		ret = vs6725_reg_read(client, PATCH_VERSION, &patch_ver);

	if (ret)
		return ret;

	if ((dev_id_hi != VS6725_DEVICE_ID_HI) ||
			(dev_id_lo != VS6725_DEVICE_ID_LO)) {
		dev_err(&client->dev, "Device ID error 0x%02x:0x%02x\n",
				dev_id_hi, dev_id_lo);
		return -ENODEV;
	}

	priv->model = V4L2_IDENT_VS6725;

	dev_info(&client->dev,
		"vs6725 Device-ID=0x%02x::0x%02x, Firmware-Ver=0x%02x"
		" Patch-Ver=0x%02x\n", dev_id_hi, dev_id_lo, fm_ver, patch_ver);

	priv->active_pipe = PIPE_0;

	return 0;
}

static int vs6725_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct vs6725 *priv;
	struct soc_camera_link *icl = soc_camera_i2c_to_link(client);
	int ret;

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

	v4l2_ctrl_handler_init(&priv->hdl, 13);

	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_GAIN, 0, 0x1f, 1, VS6725_DEF_GAIN);
	priv->autowb = v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	priv->blue = v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_BLUE_BALANCE, 0, 0xff, 1, VS6725_DEF_BLUE);
	priv->red = v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_RED_BALANCE, 0, 0xff, 1, VS6725_DEF_RED);

	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_SATURATION, 0, 0xc8, 1, VS6725_DEF_SAT);
	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 0xc8, 1, VS6725_DEF_BRIGHT);
	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 0xc8, 1, VS6725_DEF_CONTRAST);

	priv->autoexposure = v4l2_ctrl_new_std_menu(&priv->hdl,
			&vs6725_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
			V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	priv->exposure = v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_EXPOSURE, 0, 0x3, 1, VS6725_DEF_AECH);

	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_GAMMA, 0, 0x1f, 1, VS6725_DEF_GAMMA);
	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_ZOOM_RELATIVE, 0, 0xffff, 1,
			VS6725_DEF_ZOOM_REL);
	v4l2_ctrl_new_std(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_ZOOM_CONTINUOUS, 0, 0xffff, 1,
			VS6725_DEF_ZOOM_CONT);
	v4l2_ctrl_new_std_menu(&priv->hdl, &vs6725_ctrl_ops,
			V4L2_CID_COLORFX, V4L2_COLORFX_SKETCH,
			V4L2_COLORFX_EMBOSS, COLOR_EFFECT_NORMAL);

	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		kfree(priv);
		return priv->hdl.error;
	}

	v4l2_ctrl_auto_cluster(3, &priv->autowb, 0, true);
	v4l2_ctrl_auto_cluster(2, &priv->autoexposure,
				V4L2_EXPOSURE_MANUAL, true);

	priv->icl = icl;

	/* set the default format */
	priv->fmt = vs6725_default_fmt;
	priv->rect.left = 0;
	priv->rect.top = 0;

	mdelay(10);
	ret = vs6725_camera_init(client);
	if (!ret)
		ret = v4l2_ctrl_handler_setup(&priv->hdl);

	if (ret) {
		v4l2_ctrl_handler_free(&priv->hdl);
		kfree(priv);
	}

	return ret;
}

static int vs6725_remove(struct i2c_client *client)
{
	struct vs6725 *priv = to_vs6725(client);

	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id vs6725_id[] = {
	{ "vs6725", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vs6725_id);

#ifdef CONFIG_OF
static const struct of_device_id vs6725_of_match[] = {
	{.compatible = "st,vs6725",},
	{}
};

MODULE_DEVICE_TABLE(of, vs6725_of_match);
#endif

static struct i2c_driver vs6725_i2c_driver = {
	.driver = {
		.name = "vs6725",
#ifdef CONFIG_OF
		.of_match_table = vs6725_of_match,
#endif

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
