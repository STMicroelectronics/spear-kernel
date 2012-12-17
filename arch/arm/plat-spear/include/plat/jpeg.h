/*
 * arch/arm/plat-spear/include/plat/jpeg.h
 *
 * JPEG definitions for SPEAr platform
 *
 * Copyright (C) 2012 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_JPEG_H
#define __PLAT_JPEG_H

#include <linux/types.h>

#ifdef CONFIG_ARCH_SPEAR13XX
#define JPEG_DMA_MAX_COUNT	2048
#else
#include <asm/hardware/pl080.h>
#define JPEG_DMA_MAX_COUNT	PL080_CONTROL_TRANSFER_SIZE_MASK
#endif

struct dma_chan;
struct jpeg_plat_data {
	bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
	void *mem2jpeg_slave;
	void *jpeg2mem_slave;
	void (*plat_reset)(void);
};

static inline void jpeg_ip_reset(void __iomem *jpeg_addr, u32 reset_bit)
{
	u32 tmp = 0;
	void __iomem *addr = jpeg_addr;

	tmp = readl(addr);
	writel((reset_bit | tmp), addr);
	writel(tmp, addr);
}

#endif /* __PLAT_JPEG_H */
