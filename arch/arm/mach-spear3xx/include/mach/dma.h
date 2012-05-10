/*
 * arch/arm/mach-spear3xx/include/mach/dma.h
 *
 * DMA information for SPEAr3xx machine family
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

#define MAX_DMA_ADDRESS		0xffffffff

/* Platform-configurable bits in src & dest master */
enum dma_master {
	MASTER_AHB0,
	MASTER_AHB1,
	MASTER_NONE
};

/* Burst Size */
enum dma_burst {
	BURST_1,
	BURST_4,
	BURST_8,
	BURST_16,
	BURST_32,
	BURST_64,
	BURST_128,
	BURST_256,
	BURST_NONE
};

enum dma_master_info {
	DMA_MASTER_MEMORY = MASTER_AHB0,
	DMA_MASTER_NAND = MASTER_AHB0,
	DMA_MASTER_UART_0 = MASTER_AHB0,
	DMA_MASTER_SPI_0 = MASTER_AHB0,
	DMA_MASTER_I2C = MASTER_AHB0,
	DMA_MASTER_IRDA = MASTER_AHB0,
	DMA_MASTER_ADC = MASTER_AHB0,
	DMA_MASTER_JPEG = MASTER_AHB0,
	DMA_MASTER_RAS = MASTER_AHB0,
};

/*request id of all the peripherals.*/
enum request_id {
	/* Scheme 00 */
	DMA_REQ_UART_RX_0 = 2,
	DMA_REQ_UART_TX_0 = 3,
	DMA_REQ_SPI_RX_0 = 8,
	DMA_REQ_SPI_TX_0 = 9,
	DMA_REQ_I2C_RX = 10,
	DMA_REQ_I2C_TX = 11,
	DMA_REQ_IRDA = 12,
	DMA_REQ_ADC = 13,
	DMA_REQ_TO_JPEG = 14,
	DMA_REQ_FROM_JPEG = 15,

	/* Scheme 01 */
	DMA_REQ_RAS_0_RX = 0,
	DMA_REQ_RAS_0_TX = 1,
	DMA_REQ_RAS_1_RX = 2,
	DMA_REQ_RAS_1_TX = 3,
	DMA_REQ_RAS_2_RX = 4,
	DMA_REQ_RAS_2_TX = 5,
	DMA_REQ_RAS_3_RX = 6,
	DMA_REQ_RAS_3_TX = 7,
	DMA_REQ_RAS_4_RX = 8,
	DMA_REQ_RAS_4_TX = 9,
	DMA_REQ_RAS_5_RX = 10,
	DMA_REQ_RAS_5_TX = 11,
	DMA_REQ_RAS_6_RX = 12,
	DMA_REQ_RAS_6_TX = 13,
	DMA_REQ_RAS_7_RX = 14,
	DMA_REQ_RAS_7_TX = 15,

	/* RAS request lines for SPEAr320s */
	DMA_REQ_RAS_SSP1_RX = 0,
	DMA_REQ_RAS_SSP1_TX = 1,
	DMA_REQ_RAS_SSP2_RX = 2,
	DMA_REQ_RAS_SSP2_TX = 3,
	DMA_REQ_RAS_UART1_RX = 4,
	DMA_REQ_RAS_UART1_TX = 5,
	DMA_REQ_RAS_UART2_RX = 6,
	DMA_REQ_RAS_UART2_TX = 7,
	DMA_REQ_RAS_I2C1_RX = 8,
	DMA_REQ_RAS_I2C1_TX = 9,
	DMA_REQ_RAS_I2C2_RX = 10,
	DMA_REQ_RAS_I2C2_TX = 11,
	DMA_REQ_RAS_I2S_RX = 12,
	DMA_REQ_RAS_I2S_TX = 13,
	DMA_REQ_RAS_RS484_RX = 14,
	DMA_REQ_RAS_RS484_TX = 15,
};

#endif /* __MACH_DMA_H */
