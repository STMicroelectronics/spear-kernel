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
#include <asm/setup.h>
#include <mach/db9000-regs.h>
#include <mach/db9000fb_info.h>
#include <mach/generic.h>
#include <mach/hardware.h>

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
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1),
	.pwmfr = ~DB9000_PWMFR_PWM_FCI | DB9000_PWMFR_PWM_FCE |
		DB9000_PWMFR_PWM_FCD(0x18),
	.pctr = 0,
	.dear = 0,
};
static struct db9000fb_mach_info sharp_lcd_info = {
	.modes		= &sharp_LQ043T3DX0A_mode,
	.num_modes	= 1,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

static struct db9000fb_mode_info hannstar_hsd07_mode = {
	.mode = {
		.name = "Hannstar HSD07",
		.refresh = 0,
		.xres = 800,
		.yres = 480,
		.pixclock = 33333,
		.left_margin = 40,
		.right_margin = 40,
		.upper_margin = 29,
		.lower_margin = 13,
		.hsync_len = 48,
		.vsync_len = 3,
		.sync = 0, /* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
	},
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1),
	.pwmfr = ~DB9000_PWMFR_PWM_FCI | DB9000_PWMFR_PWM_FCE |
		DB9000_PWMFR_PWM_FCD(0x18),
	.pctr = 0,
	.dear = 0,
};
static struct db9000fb_mach_info hannstar_hsd07_info = {
	.modes		= &hannstar_hsd07_mode,
	.num_modes	= 1,
	.lcd_conn	= LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

/* Max possible resolution for HDMI TX */
static struct db9000fb_mode_info hdmi_1080p_mode = {
	.mode = { 	/* 1080p */
		.name = "HDMI 1080p",
		.refresh = 60,
		.xres = 1920,
		.yres = 1080,
		.pixclock = 6734,
		.left_margin = 148,
		.right_margin = 88,
		.upper_margin = 36,
		.lower_margin = 4,
		.hsync_len = 44,
		.vsync_len = 5,
		.sync = 0, /* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
	},
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FDW(2),
	.pwmfr = DB9000_PWMFR_PWM_FCI | DB9000_PWMFR_PWM_FCE |
		DB9000_PWMFR_PWM_FCD(0x8),
	.pctr = DB9000_PCTR_PCI,
	.dear = 0,
};

static struct db9000fb_mach_info hdmi_1080p_info = {
	.modes          = &hdmi_1080p_mode,
	.num_modes      = 1,
	.lcd_conn       = LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

static struct db9000fb_mode_info chimei_b101aw02_mode = {
	.mode = {
		.name = "Chemei B101AW02",
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = 15384,
		.left_margin = 160,
		.right_margin = 24,
		.upper_margin = 29,
		.lower_margin = 3,
		.hsync_len = 136,
		.vsync_len = 6,
		.sync = 0, /* FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT */
	},
	.bpp = 32,
	.cr1 = DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_HSP |
		DB9000_CR1_VSP | DB9000_CR1_OPS(1) | DB9000_CR1_FDW(2),
	.pwmfr = DB9000_PWMFR_PWM_FCI | DB9000_PWMFR_PWM_FCE |
		DB9000_PWMFR_PWM_FCD(0x8),
	.pctr = DB9000_PCTR_PCI,
	.dear = 0,
};
static struct db9000fb_mach_info chimei_b101aw02_info = {
	.modes          = &chimei_b101aw02_mode,
	.num_modes      = 1,
	.lcd_conn       = LCD_PCLK_EDGE_FALL,
	.video_mem_size = 0,
	.cmap_static    = 0,
	.cmap_inverse   = 0,
};

static void clcd_set_plat_data(struct platform_device *pdev,
		struct db9000fb_mach_info *data)
{
	struct db9000fb_mode_info *inf = data->modes;
	struct clk *pclk, *vco_clk, *clcd_pclk, *fb_clk, *ah_clk;

	pdev->dev.platform_data = data;

#ifdef CONFIG_CPU_SPEAR1340
	if (cpu_is_spear1340())
		data->clcd_mux_selection = &config_clcd_gpio_pads;
#endif

	if (!strcmp("Chemei B101AW02", inf->mode.name)
			|| (!strcmp("HDMI 1080p", inf->mode.name))) {
		vco_clk = clk_get(NULL, "vco1div4_clk");
		if (IS_ERR(vco_clk)) {
			pr_err("%s:vco1div 4 clock get fail\n", __func__);
			return ;
		}

		pclk = clk_get(NULL, "clcd_synth_clk");
		if (IS_ERR(pclk)) {
			pr_err("%s:clcd synth clock get fail\n", __func__);
			goto free_vco_clk;
		}
		clk_set_parent(pclk, vco_clk);

		clcd_pclk = clk_get_sys("clcd_pixel_clk", NULL);
		if (IS_ERR(clcd_pclk)) {
			pr_err("%s:clcd-pixel clock get fail\n", __func__);
			goto free_pclk;
		}
		clk_set_parent(clcd_pclk, pclk);

		fb_clk = clk_get_sys("clcd-db9000", NULL);
		if (IS_ERR(fb_clk)) {
			pr_err("%s:clcd clock get fail\n", __func__);
			goto free_clcd_pclk;
		}
		clk_set_parent(fb_clk, clcd_pclk);

free_clcd_pclk:
		clk_put(clcd_pclk);
free_pclk:
		clk_put(pclk);
free_vco_clk:
		clk_put(vco_clk);

	} else if (!strcmp("Sharp LQ043T3DA0A", inf->mode.name)) {

		ah_clk = clk_get(NULL, "ahb_clk");
		if (IS_ERR(ah_clk)) {
			pr_err("%s:enabling ahb_clk fail\n", __func__);
			return ;
		}
		fb_clk = clk_get_sys("clcd-db9000", NULL);
		if (IS_ERR(fb_clk)) {
			pr_err("%s:enabling fb_clk fail\n", __func__);
			clk_put(ah_clk);
			return;
		}
		clk_set_parent(fb_clk, ah_clk);
	}
	return ;
}

/* string specifying which clcd boards are requested */
static char spear13xx_panel[20] = {'\0', };
static int __init spear13xx_panel_select(char *panel)
{
	if (strlen(panel) <= sizeof(spear13xx_panel))
		strcpy(spear13xx_panel, panel);
	else
		return -ENOMEM;

	return 0;
}
__setup("panel=", spear13xx_panel_select);

static struct db9000fb_mach_info *panel_to_mach_info(char *panel)
{
	struct db9000fb_mach_info *mach_info;

	if (!strcmp(spear13xx_panel, "chimei"))
		mach_info = &chimei_b101aw02_info;
	else if (!strcmp(spear13xx_panel, "sharp"))
		mach_info = &sharp_lcd_info;
	else if (!strcmp(spear13xx_panel, "hannstar"))
		mach_info = &hannstar_hsd07_info;
	else if (!strcmp(spear13xx_panel, "1080p"))
		mach_info = &hdmi_1080p_info;
	else {
		/* choose a default panel based upon board */
		if (machine_is_spear1340_evb() || machine_is_spear900_evb())
			mach_info = &chimei_b101aw02_info;
		else
			mach_info = &sharp_lcd_info;
	}

	return mach_info;
}

static unsigned long frame_buf_base;
void spear13xx_panel_fixup(struct meminfo *mi)
{
	int size;

	size = (NUM_OF_FRAMEBUFFERS * PANEL_MAX_XRES * PANEL_MAX_YRES *
			PANEL_MAX_BPP / 8);
	frame_buf_base = reserve_mem(mi, ALIGN(size, SZ_1M));
	if (frame_buf_base == ~0)
		pr_err("Unable to allocate fb buffer\n");
}

void spear13xx_panel_init(struct platform_device *pdev)
{
	struct db9000fb_mach_info *mach_info;

	mach_info = panel_to_mach_info(spear13xx_panel);

	if (!mach_info) {
		pr_err("Invalid panel requested: %s\n", spear13xx_panel);
		return;
	}

	mach_info->frame_buf_base = frame_buf_base;
	clcd_set_plat_data(&spear13xx_db9000_clcd_device, mach_info);
}
