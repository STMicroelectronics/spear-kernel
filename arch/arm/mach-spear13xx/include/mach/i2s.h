/*
 * arch/arm/mach-spear13xx/include/mach/i2s.h
 *
 * I2S helper macros for spear13xx machine family
 *
 * Copyright (C) 2011 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_I2S_H
#define __MACH_I2S_H

#include <linux/designware_i2s.h>
#include <linux/dw_dmac.h>

/*
 * I2S_DS: used to declare DMA data for I2S
 * @ddev: Pointer to DMA dev to be used
 * @tx_req: DMA request line for tx
 * @rx_req: DMA request line for rx
 *
 * Default values:
 * - DMA Width is kept as 16 bits
 * - DMA Burst size is kept as 16 transfers, as we have two FIFO's width depth
 *   of 8. And both of them will be read in a single DMA trnasfer.
 * - Master for Mem is selected as 0 and for i2s as 1
 */
#define I2S_DS(ddev, tx_req, rx_req)		{		\
	.mem2i2s_slave = {					\
		/* Play */					\
		.dma_dev = ddev,				\
		.reg_width = DW_DMA_SLAVE_WIDTH_16BIT,		\
		.cfg_hi = DWC_CFGH_DST_PER(tx_req),		\
		.cfg_lo = 0,					\
		.src_master = 0,				\
		.dst_master = 1,				\
		.src_msize = DW_DMA_MSIZE_16,			\
		.dst_msize = DW_DMA_MSIZE_16,			\
		.fc = DW_DMA_FC_D_M2P,				\
	},							\
	.i2s2mem_slave = {					\
		/* Record */					\
		.dma_dev = ddev,				\
		.reg_width = DW_DMA_SLAVE_WIDTH_16BIT,		\
		.cfg_hi = DWC_CFGH_SRC_PER(rx_req),		\
		.cfg_lo = 0,					\
		.src_master = 1,				\
		.dst_master = 0,				\
		.src_msize = DW_DMA_MSIZE_16,			\
		.dst_msize = DW_DMA_MSIZE_16,			\
		.fc = DW_DMA_FC_D_P2M,				\
	},							\
}

#endif /* __MACH_I2S_H */
