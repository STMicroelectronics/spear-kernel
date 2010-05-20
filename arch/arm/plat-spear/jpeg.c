/*
 * arch/arm/plat-spear/jpeg.c
 *
 * jpeg platform specific information file
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
#include <plat/jpeg.h>

#ifndef CONFIG_ARCH_SPEAR13XX
/* macros for configuring dma */
#define READ_DMA_CTL	(PL080_CHAN_CTL_USER_MODE | \
		PL080_CHAN_CTL_NON_BUFFERABLE | PL080_CHAN_CTL_NON_CACHEABLE | \
		PL080_CHAN_CTL_DEST_ADDR_INC | \
		PL080_CHAN_CTL_DEST_BURST(JPEG_BURST) | \
		PL080_CHAN_CTL_SRC_BURST(JPEG_BURST))

#define WRITE_DMA_CTL	(PL080_CHAN_CTL_USER_MODE | \
		PL080_CHAN_CTL_NON_BUFFERABLE | PL080_CHAN_CTL_NON_CACHEABLE | \
		PL080_CHAN_CTL_SRC_ADDR_INC | \
		PL080_CHAN_CTL_DEST_BURST(JPEG_BURST) | \
		PL080_CHAN_CTL_SRC_BURST(JPEG_BURST))

#define READ_DMA_CFG	((PL080_CHAN_CFG_FLOW_CTRL(SRC_PERIPHERAL_TO_MEMORY) |\
			PL080_CHAN_CFG_SRC_RQID(DMA_REQ_FROM_JPEG)))
#define WRITE_DMA_CFG	((PL080_CHAN_CFG_FLOW_CTRL(DMA_MEMORY_TO_PERIPHERAL) |\
			PL080_CHAN_CFG_DEST_RQID(DMA_REQ_TO_JPEG)))
#endif /* !CONFIG_ARCH_SPEAR13XX */

void set_jpeg_dma_configuration(struct platform_device *jpeg_pdev,
		struct device *dma_dev)
{
	struct jpeg_plat_data data;

	data.mem2jpeg_slave.dma_dev = dma_dev;
	data.mem2jpeg_slave.reg_width = JPEG_WIDTH;
	data.jpeg2mem_slave.dma_dev = dma_dev;
	data.jpeg2mem_slave.reg_width = JPEG_WIDTH;
#ifdef CONFIG_ARCH_SPEAR13XX
	data.mem2jpeg_slave.cfg_hi = DWC_CFGH_DST_PER(DMA_REQ_TO_JPEG);
	data.mem2jpeg_slave.cfg_lo = 0;
	data.mem2jpeg_slave.sms = MEM_MASTER;
	data.mem2jpeg_slave.dms = JPEG_MASTER;
	data.mem2jpeg_slave.smsize = JPEG_BURST;
	data.mem2jpeg_slave.dmsize = JPEG_BURST;
	data.mem2jpeg_slave.fc = DW_DMA_FC_D_M2P;

	data.jpeg2mem_slave.cfg_hi = DWC_CFGH_SRC_PER(DMA_REQ_FROM_JPEG);
	data.jpeg2mem_slave.cfg_lo = 0;
	data.jpeg2mem_slave.sms = JPEG_MASTER;
	data.jpeg2mem_slave.dms = MEM_MASTER;
	data.jpeg2mem_slave.smsize = JPEG_BURST;
	data.jpeg2mem_slave.dmsize = JPEG_BURST;
	data.jpeg2mem_slave.fc = DW_DMA_FC_P_P2M;
#else
	data.mem2jpeg_slave.ctl = WRITE_DMA_CTL |
			PL080_CHAN_CTL_SRC_WIDTH(JPEG_WIDTH);
	data.mem2jpeg_slave.cfg = WRITE_DMA_CFG;
	data.mem2jpeg_slave.src_master = MEM_MASTER;
	data.mem2jpeg_slave.dest_master = JPEG_MASTER;

	data.jpeg2mem_slave.ctl = READ_DMA_CTL +
		PL080_CHAN_CTL_DEST_WIDTH(JPEG_WIDTH);
	data.jpeg2mem_slave.cfg = READ_DMA_CFG;
	data.jpeg2mem_slave.src_master = JPEG_MASTER;
	data.jpeg2mem_slave.dest_master = MEM_MASTER;
#endif /* !CONFIG_ARCH_SPEAR13XX */

	/* set jpeg plat data */
	if (platform_device_add_data(jpeg_pdev, &data, sizeof(data)))
		pr_err("JPEG: Error setting plat data");
}

void set_jpeg_tx_rx_reg(struct jpeg_plat_data *data, dma_addr_t tx_reg,
		dma_addr_t rx_reg)
{
	data->mem2jpeg_slave.tx_reg = tx_reg;
	data->mem2jpeg_slave.rx_reg = 0;
	data->jpeg2mem_slave.tx_reg = 0;
	data->jpeg2mem_slave.rx_reg = rx_reg;
}
