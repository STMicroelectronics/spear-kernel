/*
* arch/arm/mach-spear13xx/db9000_lcd.c
* Copyright (C) 2010 Digital Blocks
*
* Based on arch/arm/mach-pxa/devices.c
*
* This file is licensed under the terms of the GNU General Public
* License version 2. This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <mach/spear.h>
#include <mach/db9000-regs.h>
#include <mach/db9000fb_info.h>

/* This is enough for the size of 800x480 (1.5 MB) */
#define FRAMESIZE 0x00180000

static struct db9000fb_mode_info sharp_LQ043T3DX0A_mode = {
	.mode = {
		.name = "Sharp LQ043T3DA0A",
		.refresh = 0,
		.xres = 480,
		.yres = 272,
		.pixclock = 111000,
		.left_margin = 2,
		.right_margin = 2,
		.upper_margin = 2,
		.lower_margin = 2,
		.hsync_len = 40,
		.vsync_len = 10,
		.sync = 0,/* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
	},
	.bpp = 24,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FBP,
	.pctr = 0,
	.dear = 0,

};
struct db9000fb_mach_info sharp_lcd_info = {
	.modes		= &sharp_LQ043T3DX0A_mode,
	.num_modes	= 1,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

/* 10.4 inch lcd pannel information */
static struct db9000fb_mode_info chimei_b101aw02_mode = {
	.mode = {
		.name = "Auo B101AW02",
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = 18181,
		.left_margin = 30,
		.right_margin = 30,
		.upper_margin = 20,
		.lower_margin = 20,
		.hsync_len = 30,
		.vsync_len = 20,
		.sync = 0, /* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
	},
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FDW(2),
	.pctr = 0,
	.dear = 0,
};
struct db9000fb_mach_info chimei_b101aw02_info = {
	.modes          = &chimei_b101aw02_mode,
	.num_modes      = 1,
	.lcd_conn       = LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

void clcd_set_plat_data(struct platform_device *pdev,
		struct db9000fb_mach_info *data)
{
	pdev->dev.platform_data = data;
}
