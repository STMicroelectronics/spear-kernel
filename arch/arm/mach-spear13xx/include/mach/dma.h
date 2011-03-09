/*
 * arch/arm/mach-spear13xx/include/mach/dma.h
 *
 * DMA information for SPEAr13xx machine family
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

/* request id of all the peripherals */
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR900) || \
			defined(CONFIG_CPU_SPEAR1310)
enum dma_master_info {
	SPEAR13XX_DMA_MASTER_MCIF = 0,
	SPEAR13XX_DMA_MASTER_FSMC = 1,
	SPEAR13XX_DMA_MASTER_JPEG = 1,

	/* Accessible from both 0 & 1 */
	SPEAR13XX_DMA_MASTER_MEMORY = 0,
	SPEAR13XX_DMA_MASTER_ADC = 0,
	SPEAR13XX_DMA_MASTER_I2S = 0,
	SPEAR13XX_DMA_MASTER_UART0 = 0,
	SPEAR13XX_DMA_MASTER_SPI0 = 0,
	SPEAR13XX_DMA_MASTER_I2C = 0,
};

enum request_id {
	/* Scheme 00 */
	SPEAR13XX_DMA_REQ_ADC = 0,
	SPEAR13XX_DMA_REQ_FROM_JPEG = 2,
	SPEAR13XX_DMA_REQ_TO_JPEG = 3,
	SPEAR13XX_DMA_REQ_SPI0_TX = 4,
	SPEAR13XX_DMA_REQ_SPI0_RX = 5,
	SPEAR13XX_DMA_REQ_UART0_TX = 6,
	SPEAR13XX_DMA_REQ_UART0_RX = 7,
	SPEAR13XX_DMA_REQ_I2C_TX = 8,
	SPEAR13XX_DMA_REQ_I2C_RX = 9,
	SPEAR13XX_DMA_REQ_I2S_TX = 10,
	SPEAR13XX_DMA_REQ_I2S_RX = 11,
};
#endif

#endif /* __MACH_DMA_H */
