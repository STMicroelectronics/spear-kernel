/*
* include/linux/pl080_dmac.h
* ARM PL080 DMA Controller driver - header file
*
* Copyright (ST) 2010 Viresh Kumar (viresh.kumar@st.com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/

#ifndef PL080_DMAC_H
#define PL080_DMAC_H

#include <linux/dmaengine.h>
#include <mach/dma.h>

/*
* This is maximum number of transfers done by dma per lli. Total transfer size
* will be this number multiplied by src peripheral access width specified by
* enum dma_width.
*/
#define PL080_CHAN_MAX_COUNT	4095U

/**
 * struct pl080_dma_platform_data - Controller configuration parameters
 * @nr_channels: Number of channels supported by hardware (max 8)
 */
struct pl080_dma_platform_data {
	unsigned int	nr_channels;
};

/* Flow controller modes */
enum flow_ctrl {
	DMA_MEMORY_TO_MEMORY,
	DMA_MEMORY_TO_PERIPHERAL,
	DMA_PERIPHERAL_TO_MEMORY,
	DMA_PERIPHERAL_TO_PERIPHERAL,
	DEST_PERIPHERAL_TO_PERIPHERAL,
	DEST_MEMORY_TO_PERIPHERAL,
	SRC_PERIPHERAL_TO_MEMORY,
	SRC_PERIPHERAL_TO_PERIPHERAL
};

/* Access Width */
enum dma_width {
	WIDTH_BYTE,
	WIDTH_HALFWORD,
	WIDTH_WORD
};

/**
 * struct pl080_dma_slave - Controller-specific information about a slave
 * @slave: Generic information about the slave
 * @ctl: Platform-specific initializer for the CTL register
 * @cfg: Platform-specific initializer for the CFG register
 * @src_master: DMA master selection for src. Give the master info from
 * enum dma_master_info present in mach/dma.h or by yourself if you want to use
 * other master.
 * @dest_master: DMA master selection for dest. Give the master info from
 * enum dma_master_info present in mach/dma.h or by yourself if you want to use
 * other master.
 */
struct pl080_dma_slave {
	struct device		*dma_dev;
	dma_addr_t		tx_reg;
	dma_addr_t		rx_reg;
	enum dma_width		reg_width;
	u32			ctl;
	u32			cfg;
	enum dma_master		src_master;
	enum dma_master		dest_master;
};

/* Platform-configurable bits in CTL : following macros are used to fill ctl
 * field of struct pl080_dma_slave. Some of below macros can be ORRED and
 * copied to ctl
 */
/*select this if you want interrupt at end of each packet*/
#define PL080_CHAN_CTL_INT_LLI		(1 << 31)
/*select one of the two modes*/
#define PL080_CHAN_CTL_PRIV_MODE	(1 << 28)
#define PL080_CHAN_CTL_USER_MODE	(0 << 28)
/*select one of the two modes*/
#define PL080_CHAN_CTL_BUFFERABLE	(1 << 29)
#define PL080_CHAN_CTL_NON_BUFFERABLE	(0 << 29)
/*select one of the two modes*/
#define PL080_CHAN_CTL_CACHEABLE	(1 << 30)
#define PL080_CHAN_CTL_NON_CACHEABLE	(0 << 30)
/*select this if you want to increment src or destination address after each
 ** transfer*/
#define PL080_CHAN_CTL_DEST_ADDR_INC	(1 << 27)
#define PL080_CHAN_CTL_SRC_ADDR_INC	(1 << 26)
/* For mem-2-mem transfer both src and dest width must be selected if slave
 ** structure is made. */
/* For peripheral oriented xfers select EITHER src OR dest access width.
 ** reg_width field of peripheral is used for passing peripheral width. If
 ** periph is src then fill dest width else vice versa */
/* Here x can be one of type enum dma_width */
#define PL080_CHAN_CTL_DEST_WIDTH(x)	((x) << 21)
#define PL080_CHAN_CTL_SRC_WIDTH(x)	((x) << 18)
#define PL080_CHAN_CTL_SRC_WIDTH_MASK	(7 << 18)
#define PL080_CHAN_CTL_GET_SRC_WIDTH(x)	(((x) >> 18) & 7)
#define PL080_CHAN_CTL_DEST_WIDTH_MASK	(7 << 21)
#define PL080_CHAN_CTL_GET_DEST_WIDTH(x)	(((x) >> 21) & 7)
/*select src AND dest burst size. here x can be one of type enum dma_burst*/
#define PL080_CHAN_CTL_DEST_BURST(x)	((x) << 15)
#define PL080_CHAN_CTL_SRC_BURST(x)	((x) << 12)

/* Platform-configurable bits in CFG : following macros are used to fill cfg
 * field of struct pl080_dma_slave. Some of below macros can be ORRED and
 * copied to cfg
 */
/*select flow controller for transfer. here x is of type enum flow_ctrl*/
#define PL080_CHAN_CFG_FLOW_CTRL(x)	((x) << 11)
/* select request id of src AND dest periph in peripheral oriented transfers.
 ** here x is of type enum request_id */
#define PL080_CHAN_CFG_DEST_RQID(x)	((x) << 6)
#define PL080_CHAN_CFG_SRC_RQID(x)	((x) << 1)
/* Channel enable */
#define PL080_CHAN_CFG_ENABLE	(1 << 0)

#endif /* PL080_DMAC_H */
