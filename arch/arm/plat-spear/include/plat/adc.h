/*
 * arch/arm/plat-spear/include/plat/adc.h
 *
 * ADC definitions for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_ADC_H
#define __PLAT_ADC_H

#include <linux/platform_device.h>
#include <linux/spear_adc_usr.h>

#ifdef CONFIG_ARCH_SPEAR13XX
#include <linux/dw_dmac.h>

#define ADC_BURST	DW_DMA_MSIZE_1
#define ADC_WIDTH	DW_DMA_SLAVE_WIDTH_32BIT
#define DMA_MAX_COUNT	2048

struct adc_plat_data {
	struct adc_config config;
	struct dw_dma_slave slave;
};
#else
#include <linux/pl080_dmac.h>

#define ADC_BURST	BURST_1
#define ADC_WIDTH	WIDTH_WORD
#define DMA_MAX_COUNT	PL080_CHAN_MAX_COUNT

struct adc_plat_data {
	struct adc_config config;
	struct pl080_dma_slave slave;
};
#endif

void set_adc_plat_data(struct platform_device *adc_pdev,
		struct device *dma_dev);
#ifdef CONFIG_SPEAR_ADC_DMA_IF
void set_adc_rx_reg(struct adc_plat_data *data, dma_addr_t rx_reg);
#endif /* CONFIG_SPEAR_ADC_DMA_IF */
#endif /* __PLAT_ADC_H */
