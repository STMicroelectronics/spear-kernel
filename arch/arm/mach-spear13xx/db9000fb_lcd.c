/*
* arch/arm/mach-spear13xx/db9000_lcd.c
* Copyright (C) 2010 Digital Blocks
*
* Based on arch/arm/mach-pxa/devices.c
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any kind,
* whether express or implied.
*/

#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/db9000fb.h>
#include <asm/setup.h>
#include <mach/generic.h>

static struct fb_videomode def_modelist[] = {
	{
		"480x272-32@0", 0, 480, 272, 111000, 2, 2, 2, 2, 40, 10, 0,
		FB_VMODE_NONINTERLACED
	}, {
		"800x480-32@0", 0, 800, 480, 33333, 40, 40, 29, 13, 48, 3, 0,
		FB_VMODE_NONINTERLACED
	}, {
		"1024x768-32@60", 60, 1024, 768, 15384, 160, 24, 29, 3, 136, 6,
		0, FB_VMODE_NONINTERLACED
	}, {
		"1280x720-32@60", 60, 1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	}, {
		"1920x540-32@60", 60, 1920, 540, 13468, 148, 88, 15, 2, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	}, {
		"1920x1080-32@60", 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED
	},
};

static struct db9000fb_ctrl_info ctrl_info = {
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FDW(2),
	.pwmfr = DB9000_PWMFR_PWM_FCD(24),
	.pctr = DB9000_PCTR_PCI,
	.dear = 0,
};

static unsigned long frame_buf_base;
#define FRAMEBUFFER_SIZE	(NUM_OF_FRAMEBUFFERS * PANEL_MAX_XRES * \
		PANEL_MAX_YRES * PANEL_MAX_BPP / 8)

/* This will reserve memory for clcd framebuffer */
void spear13xx_reserve_mem(void)
{
	frame_buf_base = memblock_alloc(FRAMEBUFFER_SIZE, SZ_1M);

	if (!frame_buf_base)
		pr_err("Unable to allocate videobufs in fixup\n");
	memblock_free(frame_buf_base, FRAMEBUFFER_SIZE);
	memblock_remove(frame_buf_base, FRAMEBUFFER_SIZE);
}

static void set_fb_address(struct db9000fb_mach_info *pdata)
{
	pdata->mem_size = FRAMEBUFFER_SIZE;
	pdata->frame_buf_base = frame_buf_base;
}
static int clcd_set_plat_data(struct db9000fb_mach_info *data)
{
	struct clk *syn_clk, *pixel_clk, *ah_clk;
	int ret = 0;

	syn_clk = clk_get(NULL, "clcd_syn_clk");
	if (IS_ERR(syn_clk)) {
		pr_err("%s:enabling clcd_syn_clk fail\n", __func__);
		return ret;
	}

	pixel_clk = clk_get(NULL, "clcd_pixel_mclk");
	if (IS_ERR(syn_clk)) {
		pr_err("%s:enabling clcd_pixel_clk fail\n", __func__);
		goto free_syn_clk;
	}

	ah_clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(ah_clk)) {
		pr_err("%s:enabling ahb_clk fail\n", __func__);
		goto free_pixel_clk;
	}

	ret = clk_set_parent(pixel_clk, syn_clk);
	if (ret < 0) {
		pr_err("Failed to set parent syn_clk to clcd_pixel_clk\n");
		goto free_ah_clk;
	}

	data->bus_clk = ah_clk;
	data->pixel_clk = pixel_clk;
	set_fb_address(data);

free_ah_clk:
	clk_put(ah_clk);
free_pixel_clk:
	clk_put(pixel_clk);
free_syn_clk:
	clk_put(syn_clk);

	return ret;
}

struct db9000fb_mach_info clcd_plat_info = {
	.modes		= def_modelist,
	.num_modes	= ARRAY_SIZE(def_modelist),
	.ctrl_info	= &ctrl_info,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.cmap_static	= 0,
	.cmap_inverse	= 0,
	.clcd_plat_conf = clcd_set_plat_data,
};
