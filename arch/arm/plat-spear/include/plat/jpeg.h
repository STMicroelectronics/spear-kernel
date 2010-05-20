/*
 * arch/arm/plat-spear/include/plat/jpeg.h
 *
 * JPEG definitions for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_JPEG_H
#define __PLAT_JPEG_H

#include <linux/platform_device.h>

#ifdef CONFIG_ARCH_SPEAR13XX
#include <linux/dw_dmac.h>

#define JPEG_BURST	DW_DMA_MSIZE_8
#define JPEG_WIDTH	DW_DMA_SLAVE_WIDTH_32BIT
#define DMA_MAX_COUNT	2048
#define MEM_MASTER	DW_DMA_MASTER1
#define JPEG_MASTER	DW_DMA_MASTER1

struct jpeg_plat_data {
	struct dw_dma_slave mem2jpeg_slave;
	struct dw_dma_slave jpeg2mem_slave;
};
#else
#include <linux/pl080_dmac.h>

#define JPEG_BURST	BURST_8
#define JPEG_WIDTH	WIDTH_WORD
#define DMA_MAX_COUNT	PL080_CHAN_MAX_COUNT
#define MEM_MASTER	DMA_MASTER_MEMORY
#define JPEG_MASTER	DMA_MASTER_JPEG

struct jpeg_plat_data {
	struct pl080_dma_slave mem2jpeg_slave;
	struct pl080_dma_slave jpeg2mem_slave;
};
#endif

void set_jpeg_dma_configuration(struct platform_device *jpeg_pdev,
		struct device *dma_dev);
void set_jpeg_tx_rx_reg(struct jpeg_plat_data *data, dma_addr_t tx_reg,
		dma_addr_t rx_reg);
#endif /* __PLAT_JPEG_H */
