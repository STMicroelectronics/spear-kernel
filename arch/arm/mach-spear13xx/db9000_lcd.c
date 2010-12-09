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
#include <linux/clk.h>
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
		.name = "Chemei B101AW02",
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
	.pctr = DB9000_PCTR_PCI(1) | DB9000_PCTR_PCB(0),
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

unsigned long clcd_get_fb_size(struct db9000fb_mach_info *data, int buffer_cnt)
{
	unsigned long size;
	struct db9000fb_mode_info *info = data->modes;

	size = (buffer_cnt * info->mode.xres * info->mode.yres * info->bpp / 8);

	return size;
}

void clcd_set_plat_data(struct platform_device *pdev,
		struct db9000fb_mach_info *data)
{
	unsigned int status = 0;
	struct db9000fb_mode_info *inf = data->modes;
	struct clk *pclk, *vco_clk, *clcd_pclk, *fb_clk, *ah_clk;

	pdev->dev.platform_data = data;

	if (!strcmp("Chemei B101AW02", inf->mode.name)) {
		vco_clk = clk_get(NULL, "vco1div4_clk");
		if (IS_ERR(vco_clk)) {
			status = PTR_ERR(vco_clk);
			pr_err("%s:vco1div 4 clock get fail\n", __func__);
			return ;
		}

		pclk = clk_get(NULL, "clcd_synth_clk");
		if (IS_ERR(pclk)) {
			status = PTR_ERR(pclk);
			pr_err("%s:clcd synth clock get fail\n", __func__);
			goto free_vco_clk;
		}
		clk_set_parent(pclk, vco_clk);

		clcd_pclk = clk_get_sys("clcd_pixel_clk", NULL);
		if (IS_ERR(clcd_pclk)) {
			status = PTR_ERR(clcd_pclk);
			pr_err("%s:clcd-pixel clock get fail\n", __func__);
			goto free_pclk;
		}
		clk_set_parent(clcd_pclk, pclk);

		fb_clk = clk_get_sys("clcd-db9000", NULL);
		if (IS_ERR(fb_clk)) {
			status = PTR_ERR(fb_clk);
			pr_err("%s:clcd clock get fail\n", __func__);
			goto free_clcd_pclk;
		}
		clk_set_parent(fb_clk, clcd_pclk);
		clk_set_rate(fb_clk, 58000000);

free_clcd_pclk:
		clk_put(clcd_pclk);
free_pclk:
		clk_put(pclk);
free_vco_clk:
		clk_put(vco_clk);

	} else if (!strcmp("Sharp LQ043T3DA0A", inf->mode.name)) {

		ah_clk = clk_get(NULL, "ahb_clk");
		if (IS_ERR(ah_clk)) {
			status = PTR_ERR(ah_clk);
			pr_err("%s:enabling ahb_clk fail\n", __func__);
			return ;
		}
		fb_clk = clk_get_sys("clcd-db9000", NULL);
		if (IS_ERR(fb_clk)) {
			status = PTR_ERR(fb_clk);
			pr_err("%s:enabling fb_clk fail\n", __func__);
			clk_put(ah_clk);
			return;
		}
		clk_set_parent(fb_clk, ah_clk);
	}
	return ;
}
