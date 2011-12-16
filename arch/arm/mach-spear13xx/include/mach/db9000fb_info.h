#ifndef __DB9000FB_INFO_H__
#define __DB9000FB_INFO_H__
/*
 *  arch/arm/mach-spear13xx/include/mach/db9000fb_info.h
 *
 *  Support for the db9000 frame buffer.
 *  Author: Digital Blocks, LLC
 *
 *  Based on pxafb.h by
 *  Author:     Jean-Frederic Clere
 *  Created:    Sep 22, 2003
 *  Copyright:  jfclere@sinix.net
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <asm/setup.h>
#include <linux/fb.h>
#include <mach/irqs.h>

/*
 * Supported LCD connections
 * bit     9 : for pixel clock edge
 */

#define LCD_PCLK_EDGE_RISE	(0 << 9)
#define LCD_PCLK_EDGE_FALL	(1 << 9)

#define NUM_OF_FRAMEBUFFERS 2
#define PALETTE_SIZE	(128 * 4)
#define PANEL_MAX_XRES 1920
#define PANEL_MAX_YRES 1080
#define PANEL_MAX_BPP 32
/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-spear13xx and used in the probe routine
 * of linux/drivers/video/db9000fb.c
 */
struct db9000fb_mode_info {
	struct fb_videomode	mode;
	u8	bpp;
	u32	cr1;
	u32	pctr;
	u32	dear;
	u32	pwmfr;
	u_int		cmap_greyscale : 1,
			depth:8,
			unused:23;
};

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-spear13xx/machine_name.c
 * and used in the probe routine of linux/drivers/video/db9000fb.c
 */
struct db9000fb_mach_info {
	struct db9000fb_mode_info *modes;
	unsigned int num_modes;

	unsigned int	lcd_conn;
	unsigned long	video_mem_size;
	unsigned long frame_buf_base;

	u_int		fixed_modes : 1,
			cmap_inverse:1,
			cmap_static:1,
			acceleration_enabled:1,
			unused:28;
	void (*clcd_mux_selection) (bool);

};
unsigned long db9000fb_get_hsync_time(struct device *dev);

void spear13xx_panel_fixup(struct meminfo *mi);
void spear13xx_panel_init(struct platform_device *pdev);
int db9000fb_config_screen(struct fb_var_screeninfo *screen);
#endif
