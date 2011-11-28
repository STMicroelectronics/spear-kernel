/*
 * SPEAr VIP platform specific code
 *
 * Copyright (C) 2011 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __VIP_H
#define __VIP_H

#include <linux/platform_device.h>

/* vip polarity settings : common for hsync, vsync and pixclk */
enum vip_polarity {
	POL_ACTIVE_LOW = 0,
	POL_ACTIVE_HIGH,
};

/* vip rgb width settings */
enum vip_rgb_width {
	SIXTEEN_BIT = 16,
	TWENTYFOUR_BIT = 24,
	THIRTYTWO_BIT = 32,
};

/* vip video modes */
enum vip_video_modes {
	SINGLE_PORT = 0,
	DUAL_PORT,
};

/**
 * struct vip_config - configuration data for vip instance.
 * @vsync_pol: polarity of vsync signal
 * @hsync_pol: polarity of hsync signal
 * @pix_clk_pol: polarity of pixel clock signal
 * @rgb_width: selects the rgb width
 * @vdo_mode: selects the video mode
 */
struct vip_config {
	enum vip_polarity vsync_pol;
	enum vip_polarity hsync_pol;
	enum vip_polarity pix_clk_pol;
	enum vip_rgb_width rgb_width;
	enum vip_video_modes vdo_mode;
};

/**
 * struct vip_subdev_route - routing info of subdevs connected to vip
 * @input: input source to be selected out of the possible ones
 * @output: output to be selected out of the possible ones
 * @subdev_count: number of subdevs connected to this bridge driver
 */
struct vip_subdev_route {
	u32 input;
	u32 output;
};

/**
 * struct vip_subdev_info - info of subdevs connected to vip
 * @name: sub-device name
 * @grp_id: sub-device group id
 * @num_inputs: number of inputs supported
 * @inputs: inputs available at the sub device
 * @routes: sub-dev routing information for each input
 * @can_route: check if sub-dev supports routing
 * @board_info: i2c subdev board info
 */
struct vip_subdev_info {
	char name[32];
	int grp_id;
	int num_inputs;
	struct v4l2_input *inputs;
	struct vip_subdev_route *routes;
	int can_route;
	struct i2c_board_info board_info;
};

/**
 * struct vip_plat_data - platform structure for configuring vip
 * @card_name: evm card info
 * @config: configuration specific to this vip instance
 * @subdev_info: info of attached subdevs
 * @subdev_count: number of subdevs connected to this bridge driver
 * @i2c_adapter_id: i2c bus adapter no
 * @is_field_end_gpio_based: tells whether some gpio will be used
 *      as a source of field-end interrupt to the CPU.
 * @gpio_for_frame_end_intr: gpio available for asserting frame end
 */
struct vip_plat_data {
	char *card_name;
	struct vip_config *config;
	struct vip_subdev_info *subdev_info;
	int subdev_count;
	int i2c_adapter_id;
	bool is_field_end_gpio_based;
	int gpio_for_frame_end_intr;
};

static inline void vip_set_plat_data(struct platform_device *pdev,
		struct vip_plat_data *pdata)
{
	pdev->dev.platform_data = pdata;
}

#endif /* __VIP_H */
