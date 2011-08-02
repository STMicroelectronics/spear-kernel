/*
 * arch/arm/mach-spear6xx/include/mach/dma.h
 *
 * DMA information for SPEAr6xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_DMA_H
#define __MACH_DMA_H

#include <linux/amba/pl08x.h>

#define MAX_DMA_ADDRESS		0xffffffff

enum dma_master_info {
	DMA_MASTER_MEMORY = PL08X_AHB1,
	DMA_MASTER_NAND = PL08X_AHB1,
	DMA_MASTER_UART_0 = PL08X_AHB1,
	DMA_MASTER_UART_1 = PL08X_AHB1,
	DMA_MASTER_SPI_0 = PL08X_AHB1,
	DMA_MASTER_SPI_1 = PL08X_AHB1,
	DMA_MASTER_SPI_2 = PL08X_AHB2,
	DMA_MASTER_I2C = PL08X_AHB1,
	DMA_MASTER_IRDA = PL08X_AHB1,
	DMA_MASTER_ADC = PL08X_AHB2,
	DMA_MASTER_JPEG = PL08X_AHB1,
	DMA_MASTER_RAS = PL08X_AHB1,
	DMA_MASTER_EXT = PL08X_AHB2,
};

#endif /* __MACH_DMA_H */
