/*
 * arch/arm/plat-spear/adc.c
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <plat/adc.h>
#include <mach/dma.h>

#ifndef CONFIG_ARCH_SPEAR13XX
#include <linux/amba/pl08x.h>
#endif /* !CONFIG_ARCH_SPEAR13XX */

void set_adc_plat_data(struct platform_device *adc_pdev,
		struct device *dma_dev)
{
	struct adc_plat_data data = {
		/* default configuration */
		.slave = {0, },
		.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500,
			INTERNAL_SCAN,
#ifndef CONFIG_ARCH_SPEAR6XX
			NORMAL_RESOLUTION,
#endif
			14000000, 0},
	};

#ifdef CONFIG_SPEAR_ADC_DMA_IF
#ifdef CONFIG_ARCH_SPEAR13XX
	data.slave.dma_dev = dma_dev;
	data.slave.reg_width = ADC_WIDTH;
	data.slave.cfg_hi = DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_ADC);
	data.slave.cfg_lo = 0;
	data.slave.src_master = 1;
	data.slave.dst_master = 1;
	data.slave.src_msize = ADC_BURST;
	data.slave.dst_msize = ADC_BURST;
	data.slave.fc = DW_DMA_FC_D_P2M;
#else
	data.runtime_config = true;
	data.dma_filter = pl08x_filter_id;
	data.slave.direction = DMA_FROM_DEVICE;
	data.slave.src_addr_width = ADC_WIDTH;
	data.slave.src_maxburst = ADC_BURST;
#endif /* !CONFIG_ARCH_SPEAR13XX */
#endif /* CONFIG_SPEAR_ADC_DMA_IF */

	/* set adc plat data */
	if (platform_device_add_data(adc_pdev, &data, sizeof(data)))
		pr_err("adc: Error setting plat data");
}

#ifdef CONFIG_SPEAR_ADC_DMA_IF
void set_adc_rx_reg(struct adc_plat_data *data, dma_addr_t rx_reg)
{
#ifdef CONFIG_ARCH_SPEAR13XX
	data->slave.rx_reg = rx_reg;
#else
	data->slave.src_addr = rx_reg;
#endif
}
#endif /* CONFIG_SPEAR_ADC_DMA_IF */
