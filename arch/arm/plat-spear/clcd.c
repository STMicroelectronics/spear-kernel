/*
* arch/arm/plat-spear/clcd.c
*
* Copyright (C) 2012 ST Microelectronics
* Vipul Kumar Samar <vipulkumar.samar@st.com>
* Shiraz Hashim<shiraz.hashim@st.com>
*
* This file is licensed under the terms of the GNU General Public
* License version 2. This program is licensed "as is" without any
* warranty of any kind, whether express or implied.
*/

#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>

/* This is enough for the size of 800x480 (1.5 MB) */
#define FRAMESIZE 0x00180000

static struct clcd_panel *panel_info;
int clcd_panel_setup(struct clcd_panel *panel)
{
	if (!panel)
		return -EINVAL;
	else
		panel_info = panel;

	return 0;
}

static int clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->panel = panel_info;
	/* Detect which LCD panel is connected */
	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, FRAMESIZE,
			&dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}
	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = FRAMESIZE;

	return 0;
}

static int clcd_check(struct clcd_fb *fb, struct fb_var_screeninfo *var)
{
	var->xres_virtual = var->xres = (var->xres + 15) & ~15;
	var->yres_virtual = var->yres = (var->yres + 1) & ~1;

#define CHECK(e, l, h) (var->e < l || var->e > h)
	if (CHECK(right_margin, (1), 256) ||	/* back porch */
			CHECK(left_margin, (1), 256) ||	/* front porch */
			CHECK(hsync_len, (1), 256) ||
			var->xres > 4096 ||
			var->lower_margin > 255 ||	/* back porch */
			var->upper_margin > 255 ||	/* front porch */
			var->vsync_len > 32 ||
			var->yres > 1024)
		return -EINVAL;
#undef CHECK

	/* single panel mode: PCD = max(PCD, 1) */
	/* dual panel mode: PCD = max(PCD, 5) */

	/*
	 * You can't change the grayscale setting, and
	 * we can only do non-interlaced video.
	 */
	if (var->grayscale != fb->fb.var.grayscale ||
			(var->vmode & FB_VMODE_MASK) != FB_VMODE_NONINTERLACED)
		return -EINVAL;

#define CHECK(e) (var->e != fb->fb.var.e)
	if (fb->panel->fixedtimings &&
			(CHECK(xres)		||
			 CHECK(yres)		||
			 CHECK(bits_per_pixel)	||
			 CHECK(pixclock)	||
			 CHECK(left_margin)	||
			 CHECK(right_margin)	||
			 CHECK(upper_margin)	||
			 CHECK(lower_margin)	||
			 CHECK(hsync_len)	||
			 CHECK(vsync_len)	||
			 CHECK(sync)))
		return -EINVAL;
#undef CHECK

	var->nonstd = 0;
	var->accel_flags = 0;

	return 0;
}

static void clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			fb->fb.screen_base, fb->fb.fix.smem_start);
}

static int clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
			fb->fb.screen_base,
			fb->fb.fix.smem_start,
			fb->fb.fix.smem_len);
}

struct clcd_board pl110_plat_data = {
	.name = "spear-clcd",
	.caps= CLCD_CAP_5551 | CLCD_CAP_565 | CLCD_CAP_888,
	.check = clcd_check,
	.decode = clcdfb_decode,
	.setup = clcd_setup,
	.mmap = clcd_mmap,
	.remove = clcd_remove,
};
