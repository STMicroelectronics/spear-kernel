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

#ifdef CONFIG_FB_DB9000_LCD_SHARP_LQ043T3DX0A
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
static struct db9000fb_mach_info spear1300_evb_sharp_lcd_info = {
	.modes		= &sharp_LQ043T3DX0A_mode,
	.num_modes	= 1,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};
#endif

void __init db9000_register_device(struct platform_device *dev, void *data)
{
	int ret;
	dev->dev.platform_data = data;
	ret = platform_device_register(dev);
	if (ret)
		dev_err(&dev->dev, "unable to register device: %d\n", ret);
}


static struct resource db9000fb_resources[] = {
	[0] = {
		.start	= SPEAR13XX_DB9000_LCD_BASE,
		.end	= SPEAR13XX_DB9000_LCD_BASE +
				SPEAR13XX_DB9000_LCD_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_CLCD,
		.end	= IRQ_CLCD,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

struct platform_device db9000_device_fb = {
	.name		= "db9000-fb",
	.id		= 0,
	.dev		= {
	.parent = NULL,
	.dma_mask = &fb_dma_mask,
	.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(db9000fb_resources),
	.resource	= db9000fb_resources,
};

void __init set_db9000_fb_info(struct db9000fb_mach_info *info)
{
	db9000_register_device(&db9000_device_fb, info);
}

void __init set_db9000_fb_parent(struct device *parent_dev)
{
	db9000_device_fb.dev.parent = parent_dev;
}


#if defined(CONFIG_FB_DB9000) || defined(CONFIG_FB_DB9000_MODULE)

#if 0
static struct platform_pwm_backlight_data spear1300_evb_backlight_data = {
	.pwm_id		= 3,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 10000,
};

static struct platform_device spear1300_evb_backlight_device = {
	.name = "pwm-backlight",
	.dev  = {
/*	.parent = &pxa27x_device_pwm1.dev, */
	.platform_data = &spear1300_evb_backlight_data,
	},
};
#endif

void __init spear1300_evb_init_lcd(void)
{
/*	platform_device_register(&spear1300_evb_backlight_device); */
	set_db9000_fb_info(&spear1300_evb_sharp_lcd_info);
}
#else
static inline void spear1300_evb_init_lcd(void) {}
#endif
