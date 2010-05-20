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
#include <mach/dma.h>
#include <plat/adc.h>

#ifndef CONFIG_ARCH_SPEAR13XX
/* macros for configuring dma */
#define DMA_CTL	(PL080_CHAN_CTL_USER_MODE | PL080_CHAN_CTL_NON_BUFFERABLE |\
		PL080_CHAN_CTL_NON_CACHEABLE | PL080_CHAN_CTL_DEST_ADDR_INC |\
		PL080_CHAN_CTL_DEST_BURST(ADC_BURST) | \
		PL080_CHAN_CTL_SRC_BURST(ADC_BURST) | \
		PL080_CHAN_CTL_DEST_WIDTH(ADC_WIDTH))

#define DMA_CFG	((PL080_CHAN_CFG_FLOW_CTRL(DMA_PERIPHERAL_TO_MEMORY) |\
			PL080_CHAN_CFG_SRC_RQID(DMA_REQ_ADC)))
#endif /* !CONFIG_ARCH_SPEAR13XX */

void set_adc_plat_data(struct platform_device *adc_pdev,
		struct device *dma_dev)
{
	struct adc_plat_data data = {
		/* default configuration */
		.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500,
			INTERNAL_SCAN,
#ifndef CONFIG_ARCH_SPEAR6XX
			NORMAL_RESOLUTION,
#endif
			14000000, 0},
	};

#ifdef CONFIG_SPEAR_ADC_DMA_IF

	data.slave.dma_dev = dma_dev;
	data.slave.reg_width = ADC_WIDTH;
#ifdef CONFIG_ARCH_SPEAR13XX
	data.slave.cfg_hi = DWC_CFGH_SRC_PER(DMA_REQ_ADC);
	data.slave.cfg_lo = 0;
	data.slave.sms = DW_DMA_MASTER1;
	data.slave.dms = DW_DMA_MASTER1;
	data.slave.smsize = ADC_BURST;
	data.slave.dmsize = ADC_BURST;
	data.slave.fc = DW_DMA_FC_D_P2M;
#else
	data.slave.ctl = DMA_CTL |
			PL080_CHAN_CTL_SRC_WIDTH(ADC_WIDTH);
	data.slave.cfg = DMA_CFG;
	data.slave.src_master = DMA_MASTER_ADC;
	data.slave.dest_master = DMA_MASTER_MEMORY;
#endif /* !CONFIG_ARCH_SPEAR13XX */
#endif /* CONFIG_SPEAR_ADC_DMA_IF */

	/* set adc plat data */
	if (platform_device_add_data(adc_pdev, &data, sizeof(data)))
		pr_err("adc: Error setting plat data");
}

#ifdef CONFIG_SPEAR_ADC_DMA_IF
void set_adc_rx_reg(struct adc_plat_data *data, dma_addr_t rx_reg)
{
	data->slave.rx_reg = rx_reg;
}
#endif /* CONFIG_SPEAR_ADC_DMA_IF */
