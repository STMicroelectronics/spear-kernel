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

#ifndef CONFIG_ARCH_SPEAR13XX
#include <linux/amba/pl08x.h>
#endif

#include <linux/kernel.h>
#include <plat/jpeg.h>
#include <mach/dma.h>

void set_jpeg_dma_configuration(struct platform_device *jpeg_pdev,
		struct device *dma_dev)
{
	struct jpeg_plat_data data = {0, };

#ifdef CONFIG_ARCH_SPEAR13XX
	data.mem2jpeg_slave.dma_dev = dma_dev;
	data.mem2jpeg_slave.reg_width = JPEG_WIDTH;
	data.jpeg2mem_slave.dma_dev = dma_dev;
	data.jpeg2mem_slave.reg_width = JPEG_WIDTH;
	data.mem2jpeg_slave.cfg_hi =
		DWC_CFGH_DST_PER(SPEAR13XX_DMA_REQ_TO_JPEG);
	data.mem2jpeg_slave.cfg_lo = 0;
	data.mem2jpeg_slave.src_master = MEM_MASTER;
	data.mem2jpeg_slave.dst_master = JPEG_MASTER;
	data.mem2jpeg_slave.src_msize = JPEG_BURST;
	data.mem2jpeg_slave.dst_msize = JPEG_BURST;
	data.mem2jpeg_slave.fc = DW_DMA_FC_D_M2P;

	data.jpeg2mem_slave.cfg_hi =
		DWC_CFGH_SRC_PER(SPEAR13XX_DMA_REQ_FROM_JPEG);
	data.jpeg2mem_slave.cfg_lo = 0;
	data.jpeg2mem_slave.src_master = JPEG_MASTER;
	data.jpeg2mem_slave.dst_master = MEM_MASTER;
	data.jpeg2mem_slave.src_msize = JPEG_BURST;
	data.jpeg2mem_slave.dst_msize = JPEG_BURST;
	data.jpeg2mem_slave.fc = DW_DMA_FC_P_P2M;
#else
	data.runtime_config = true;
	data.dma_filter = pl08x_filter_id;
	data.mem2jpeg_slave.direction = DMA_TO_DEVICE;
	data.mem2jpeg_slave.dst_addr_width = JPEG_WIDTH;
	data.mem2jpeg_slave.dst_maxburst = JPEG_BURST;

	data.jpeg2mem_slave.direction = DMA_FROM_DEVICE;
	data.jpeg2mem_slave.src_addr_width = JPEG_WIDTH;
	data.jpeg2mem_slave.src_maxburst = JPEG_BURST;
#endif /* !CONFIG_ARCH_SPEAR13XX */

	/* set jpeg plat data */
	if (platform_device_add_data(jpeg_pdev, &data, sizeof(data)))
		pr_err("JPEG: Error setting plat data");
}

void set_jpeg_tx_rx_reg(struct jpeg_plat_data *data, dma_addr_t tx_reg,
		dma_addr_t rx_reg)
{
#ifdef CONFIG_ARCH_SPEAR13XX
	data->mem2jpeg_slave.tx_reg = tx_reg;
	data->mem2jpeg_slave.rx_reg = 0;
	data->jpeg2mem_slave.tx_reg = 0;
	data->jpeg2mem_slave.rx_reg = rx_reg;
#else
	data->mem2jpeg_slave.dst_addr = tx_reg;
	data->jpeg2mem_slave.src_addr = rx_reg;
#endif
}
