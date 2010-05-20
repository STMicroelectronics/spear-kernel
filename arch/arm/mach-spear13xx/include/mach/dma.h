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

/*request id of all the peripherals.*/
enum request_id {
	/* Scheme 00 */
	DMA_REQ_ADC = 0,
	DMA_REQ_FROM_JPEG = 2,
	DMA_REQ_TO_JPEG = 3,
	DMA_REQ_SPI_TX_0 = 4,
	DMA_REQ_SPI_RX_0 = 5,
	DMA_REQ_UART_TX_0 = 6,
	DMA_REQ_UART_RX_0 = 7,
	DMA_REQ_I2C_TX = 8,
	DMA_REQ_I2C_RX = 9,
	DMA_REQ_I2S_TX = 10,
	DMA_REQ_I2S_RX = 11,
};

#endif /* __MACH_DMA_H */
