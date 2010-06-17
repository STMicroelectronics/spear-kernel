#ifndef __DB9000FB_H__
#define __DB9000FB_H__

/*
 * linux/drivers/video/db9000fb.h
 *    -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 *  Copyright (C) 2010 Digital Blocks, Inc.
 *  Based on pxafb.h
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *  2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 *  - ported pxafb code to DB9000
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

/* bitfields endian games */
/* copied from fore200e.h */
#if defined(__LITTLE_ENDIAN_BITFIELD)
#define BITFIELD2(b1, b2)                    b1; b2;
#define BITFIELD3(b1, b2, b3)                b1; b2; b3;
#define BITFIELD4(b1, b2, b3, b4)            b1; b2; b3; b4;
#define BITFIELD5(b1, b2, b3, b4, b5)        b1; b2; b3; b4; b5;
#define BITFIELD6(b1, b2, b3, b4, b5, b6)    b1; b2; b3; b4; b5; b6;
#elif defined(__BIG_ENDIAN_BITFIELD)
#define BITFIELD2(b1, b2)                                    b2; b1;
#define BITFIELD3(b1, b2, b3)                            b3; b2; b1;
#define BITFIELD4(b1, b2, b3, b4)                    b4; b3; b2; b1;
#define BITFIELD5(b1, b2, b3, b4, b5)            b5; b4; b3; b2; b1;
#define BITFIELD6(b1, b2, b3, b4, b5, b6)    b6; b5; b4; b3; b2; b1;
#else
#error unknown bitfield endianess
#endif

#define to_db9000fb(info)	container_of(info, struct db9000fb_info, fb)

/* DB9000 LCD DMA Frame descriptor */
typedef struct db9000fb_dma_descriptor {
	struct {
		BITFIELD2(
        u32 fdnav :  1,    /*          */
        u32 fdna  :  30    /*          */
		)
	};
	struct {
		BITFIELD6(
		u32 dbar_dear_ld_en		: 1,
		u32 overlay_win_load	: 16,
		u32 fnc					: 1,
		u32 flc					: 1,
		u32 fd_ower				: 12,
		u32 fd_ower_ld_en		: 1
		)
	};
	u32 fd_dbar;
	u32 fd_dear;
	u32 fd_ow_dbar[DB9000_NUM_OW];

}db9000fb_dma_descriptor_t;

enum {
	PAL_NONE	= -1,
	PAL_STATIC	= 0,
	PAL_IN_FB	= 1,
//	PAL_OV2		= 2,
	PAL_MAX,
};

enum {
	DMA_BASE	= 0,
	DMA_DESCRIPTOR	= 1,
	DMA_MAX,
};

#define PALETTE_SIZE	(128 * 4)

struct db9000fb_frame_buff {
	unsigned char *frame_buff;
	unsigned char *pixel_data_start;
};

struct db9000fb_info {
	struct fb_info		            fb;
    struct device                   *dev;
	struct clk			            *clk;

	void __iomem		            *mmio_base;
    void __iomem		            *misc_io_base;

   size_t                           dma_buff_size;
   dma_addr_t		                dma_buff_phys;
	db9000fb_dma_descriptor_t	    *f_descriptor;

	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */
	/* raw memory addresses */
	dma_addr_t		                 map_dma;	/* physical */
	u_char *		                 map_cpu;	/* virtual */
	u_int			                 map_size;
	unsigned long	                hsync_time;


   void __iomem		                *video_mem;	/* virtual address of frame buffer */
	unsigned long		            video_mem_phys;	/* physical address of frame buffer */
	size_t			video_mem_size;	/* size of the frame buffer */
   size_t         video_mem_size_used;
	u16 *			   palette_cpu;	/* virtual address of palette memory */
	u_int			   palette_size;
   int            palette_mode;
   u_int		cmap_inverse:1,
				cmap_static:1,
				unused:30;


   /* Local images/copies of device registers */
	u32				                  reg_cr1;
	u32				                  reg_htr;
	u32				                  reg_vtr1;
	u32				                  reg_vtr2;
	u32				                  reg_pctr;
	u32				                  reg_isr;
	u32				                  reg_imr;
	u32				                  reg_ivr;
	u32				                  reg_iscr;
	u32				                  reg_dbar;
	u32				                  reg_dcar;
	u32				                  reg_dear;
	u32				                  reg_pwmfr;
	u32				                  reg_pwmdcr;
	u32				                  reg_dfbar;
	u32				                  reg_dflar;
	u32				                  reg_cir;

   u32                              palette[PALETTE_SIZE/4];

	//	unsigned long	hsync_time;

	volatile u_char		            state;
	volatile u_char		            task_state;
	struct mutex		               ctrlr_lock;
	wait_queue_head_t	               ctrlr_wait;
	struct work_struct	            task;

	struct completion	               disable_done;

};

#define TO_INF(ptr,member) container_of(ptr,struct db9000fb_info,member)

/*
 * These are the lcd controller states & actions for set_ctrlr_state
 */
#define C_DISABLE		(0)
#define C_ENABLE		(1)
#define C_DISABLE_CLKCHANGE	(2)
#define C_ENABLE_CLKCHANGE	(3)
#define C_REENABLE		(4)
#define C_DISABLE_PM		(5)
#define C_ENABLE_PM		(6)
#define C_STARTUP		(7)

#define DB9000FB_NAME	"DB9000FB"

/*
 * Minimum X and Y resolutions
 */
#define MIN_XRES	16
#define MIN_YRES	64

#endif /* __DB9000FB_H__ */
