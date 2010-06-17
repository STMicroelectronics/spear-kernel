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

#include <linux/fb.h>
#include <mach/db9000-regs.h>
#include <mach/irqs.h>

/*
 * Supported LCD connections
 *
 *
 * bit     9 : for pixel clock edge
 */

#define LCD_PCLK_EDGE_RISE	(0 << 9)
#define LCD_PCLK_EDGE_FALL	(1 << 9)

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-pxa/machine_name.c and used in the probe routine
 * of linux/drivers/video/pxafb.c
 */
struct db9000fb_mode_info {
	struct fb_videomode     mode;
   u8                      bpp;
   u32                     cr1;
   u_int		cmap_greyscale:1,
			depth:8,
			unused:23;
};

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-spear13xx/machine_name.c and used in the probe routine
 * of linux/drivers/video/pxafb.c
 */
#if 0
struct db9000fb_mode_info {
	u_long		pixclock;

	u_short		xres;
	u_short		yres;

	u_char		bpp;
	u_int		cmap_greyscale:1,
			depth:8,
			unused:23;

	/* Parallel Mode Timing */
	u_char		hsync_len;
	u_char		left_margin;
	u_char		right_margin;

	u_char		vsync_len;
	u_char		upper_margin;
	u_char		lower_margin;
	u_char		sync;

};
#endif
//struct pxafb_mach_info {
struct db9000fb_mach_info {
	struct db9000fb_mode_info *modes;
	unsigned int num_modes;

	unsigned int	lcd_conn;
	unsigned long	video_mem_size;

	u_int		fixed_modes:1,
			cmap_inverse:1,
			cmap_static:1,
			acceleration_enabled:1,
			unused:28;

//	u_int		cr1;
//	u_int		lccr3;
//	u_int		lccr4;
//	void (*db9000fb_backlight_power)(int);
//	void (*db9000fb_lcd_power)(int, struct fb_var_screeninfo *);
};
void set_pxa_fb_info(struct db9000fb_mach_info *hard_pxa_fb_info);
void set_pxa_fb_parent(struct device *parent_dev);
unsigned long db9000fb_get_hsync_time(struct device *dev);

