/*
 * This is the driver for the GMAC on-chip Ethernet controller for ST SoCs.
 * DWC Ether MAC 10/100/1000 Universal version 3.41a has been used for
 * developing this code.
 *
 * Copyright (C) 2007-2009 STMicroelectronics Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 */

#include <linux/netdevice.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/stmmac.h>

#include "stmmac.h"
#include "gmac.h"

#undef GMAC_DEBUG
/*#define GMAC_DEBUG*/
#undef FRAME_FILTER_DEBUG
/*#define FRAME_FILTER_DEBUG*/
#ifdef GMAC_DEBUG
#define DBG(fmt, args...) printk(fmt, ## args)
#else
#define DBG(fmt, args...) do { } while (0)
#endif

static void gmac_dump_regs(unsigned long ioaddr)
{
	int i;
	pr_info("\t----------------------------------------------\n"
			"\t GMAC registers (base addr = 0x%8x)\n"
			"\t----------------------------------------------\n",
			(unsigned int)ioaddr);

	for (i = 0; i < 55; i++) {
		int offset = i * 4;
		pr_info("\tReg No. %d (offset 0x%x): 0x%08x\n", i,
				offset, readl(ioaddr + offset));
	}
	return;
}

static int gmac_dma_init(unsigned long ioaddr, int pbl, u32 dma_tx, u32 dma_rx)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);
	do {} while ((readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET));

	value = DMA_BUS_MODE_FB | ((pbl << DMA_BUS_MODE_PBL_SHIFT) |
			(pbl << DMA_BUS_MODE_RPBL_SHIFT));

#ifdef CONFIG_STMMAC_DA
	value |= DMA_BUS_MODE_DA;	/* Rx has priority over tx */
#endif
	writel(value, ioaddr + DMA_BUS_MODE);

	/* Mask interrupts by writing to CSR7 */
	writel(DMA_INTR_DEFAULT_MASK, ioaddr + DMA_INTR_ENA);

	/* The base address of the RX/TX descriptor lists must be written into
	 * DMA CSR3 and CSR4, respectively. */
	writel(dma_tx, ioaddr + DMA_TX_BASE_ADDR);
	writel(dma_rx, ioaddr + DMA_RCV_BASE_ADDR);

	return 0;
}

static void gmac_dma_operation_mode(unsigned long ioaddr, int txmode,
					int rxmode)
{
	u32 csr6 = readl(ioaddr + DMA_CONTROL);

	if (txmode == SF_DMA_MODE) {
		DBG(KERN_DEBUG "GMAC: enabling TX store and forward mode\n");
		/* Transmit COE type 2 cannot be done in cut-through mode. */
		csr6 |= DMA_CONTROL_TSF;
		/* Operating on second frame increase the performance
		 * especially when transmit store-and-forward is used.*/
		csr6 |= DMA_CONTROL_OSF;
	} else {
		DBG(KERN_DEBUG "GMAC: disabling TX store and forward mode"
				" (threshold = %d)\n", txmode);
		csr6 &= ~DMA_CONTROL_TSF;
		csr6 &= DMA_CONTROL_TC_TX_MASK;
		/* Set the transmit threashold */
		if (txmode <= 32)
			csr6 |= DMA_CONTROL_TTC_32;
		else if (txmode <= 64)
			csr6 |= DMA_CONTROL_TTC_64;
		else if (txmode <= 128)
			csr6 |= DMA_CONTROL_TTC_128;
		else if (txmode <= 192)
			csr6 |= DMA_CONTROL_TTC_192;
		else
			csr6 |= DMA_CONTROL_TTC_256;
	}

	if (rxmode == SF_DMA_MODE) {
		DBG(KERN_DEBUG "GMAC: enabling RX store and forward mode\n");
		csr6 |= DMA_CONTROL_RSF;
	} else {
		DBG(KERN_DEBUG "GMAC: disabling RX store and forward mode"
				" (threshold = %d)\n", rxmode);
		csr6 &= ~DMA_CONTROL_RSF;
		csr6 &= DMA_CONTROL_TC_RX_MASK;
		if (rxmode <= 32)
			csr6 |= DMA_CONTROL_RTC_32;
		else if (rxmode <= 64)
			csr6 |= DMA_CONTROL_RTC_64;
		else if (rxmode <= 96)
			csr6 |= DMA_CONTROL_RTC_96;
		else
			csr6 |= DMA_CONTROL_RTC_128;
	}

	writel(csr6, ioaddr + DMA_CONTROL);
	return;
}

/* Not yet implemented --- no RMON module */
static void gmac_dma_diagnostic_fr(void *data, struct stmmac_extra_stats *x,
				unsigned long ioaddr)
{
	return;
}

static void gmac_dump_dma_regs(unsigned long ioaddr)
{
	int i;
	pr_info(" DMA registers\n");
	for (i = 0; i < 22; i++) {
		if ((i < 9) || (i > 17)) {
			int offset = i * 4;
			pr_err("\t Reg No. %d (offset 0x%x): 0x%08x\n", i,
				(DMA_BUS_MODE + offset),
				readl(ioaddr + DMA_BUS_MODE + offset));
		}
	}
	return;
}

static void gmac_irq_status(unsigned long ioaddr)
{
	u32 intr_status = readl(ioaddr + GMAC_INT_STATUS);

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & mmc_tx_irq))
		DBG(KERN_DEBUG "GMAC: MMC tx interrupt: 0x%08x\n",
			readl(ioaddr + GMAC_MMC_TX_INTR));
	if (unlikely(intr_status & mmc_rx_irq))
		DBG(KERN_DEBUG "GMAC: MMC rx interrupt: 0x%08x\n",
			readl(ioaddr + GMAC_MMC_RX_INTR));
	if (unlikely(intr_status & mmc_rx_csum_offload_irq))
		DBG(KERN_DEBUG "GMAC: MMC rx csum offload: 0x%08x\n",
			readl(ioaddr + GMAC_MMC_RX_CSUM_OFFLOAD));
	if (unlikely(intr_status & pmt_irq)) {
		DBG(KERN_DEBUG "GMAC: received Magic frame\n");
		/* clear the PMT bits 5 and 6 by reading the PMT
		 * status register. */
		readl(ioaddr + GMAC_PMT);
	}

	return;
}

static void gmac_core_init(unsigned long ioaddr, int disable_readahead,
		int csum_engine)
{
	u32 value = readl(ioaddr + GMAC_CONTROL);

	/*
	 * Enable check sum offloading for
	 * type-1/type-2 engines only
	 */
	value |= GMAC_CORE_INIT;
	if (csum_engine != STMAC_TYPE_0)
		value |= GMAC_CONTROL_ACS | GMAC_CONTROL_IPC;
	writel(value, ioaddr + GMAC_CONTROL);

	/* STBus Bridge Configuration */
	if (disable_readahead) {
		value = readl(ioaddr + GMAC_AHB_CONFIG);
		value &= GMAC_AHB_CONFIG_READ_AHEAD_MASK;
		writel(value, ioaddr + GMAC_AHB_CONFIG);
	}

	/* Freeze MMC counters */
	writel(0x8, ioaddr + GMAC_MMC_CTRL);
	/* Mask GMAC interrupts */
	writel(0x207, ioaddr + GMAC_INT_MASK);

#ifdef STMMAC_VLAN_TAG_USED
	/* Tag detection without filtering */
	writel(0x0, ioaddr + GMAC_VLAN_TAG);
#endif
	return;
}

static void gmac_set_umac_addr(unsigned long ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	stmmac_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void gmac_get_umac_addr(unsigned long ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	stmmac_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void gmac_set_filter(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;
	unsigned int value = 0;

	DBG(KERN_INFO "%s: # mcasts %d, # unicast %d\n",
		__func__, dev->mc_count, dev->uc_count);

	if (dev->flags & IFF_PROMISC)
		value = GMAC_FRAME_FILTER_PR;
	else if ((dev->mc_count > HASH_TABLE_SIZE)
			|| (dev->flags & IFF_ALLMULTI)) {
		value = GMAC_FRAME_FILTER_PM;	/* pass all multi */
		writel(0xffffffff, ioaddr + GMAC_HASH_HIGH);
		writel(0xffffffff, ioaddr + GMAC_HASH_LOW);
	} else if (dev->mc_count > 0) {
		int i;
		u32 mc_filter[2];
		struct dev_mc_list *mclist;

		/* Hash filter for multicast */
		value = GMAC_FRAME_FILTER_HMC;

		memset(mc_filter, 0, sizeof(mc_filter));
		for (i = 0, mclist = dev->mc_list;
				mclist && i < dev->mc_count;
				i++, mclist = mclist->next) {
			/* The upper 6 bits of the calculated CRC are used to
				index the contens of the hash table */
			int bit_nr =
			bitrev32(~crc32_le(~0, mclist->dmi_addr, 6)) >> 26;
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
		writel(mc_filter[0], ioaddr + GMAC_HASH_LOW);
		writel(mc_filter[1], ioaddr + GMAC_HASH_HIGH);
	}

	/* Handle multiple unicast addresses (perfect filtering)*/
	if (dev->uc.count > GMAC_MAX_UNICAST_ADDRESSES)
		/* Switch to promiscuous mode is more than 16 addrs
			are required */
		value |= GMAC_FRAME_FILTER_PR;
	else {
		int i;
		struct netdev_hw_addr *ha;

		i = 1;
		list_for_each_entry(ha, &dev->uc.list, list) {
			unsigned char *addr = ha->addr;

			gmac_set_umac_addr(ioaddr, addr, i);
			DBG(KERN_INFO "\t%d - Unicast addr %pM\n", i, addr);
			i++;
		}
	}

#ifdef FRAME_FILTER_DEBUG
	/* Enable Receive all mode (to debug filtering_fail errors) */
	value |= GMAC_FRAME_FILTER_RA;
#endif
	writel(value, ioaddr + GMAC_FRAME_FILTER);

	DBG(KERN_INFO "\tFrame Filter reg: 0x%08x\n\tHash regs: "
		"HI 0x%08x, LO 0x%08x\n", readl(ioaddr + GMAC_FRAME_FILTER),
		readl(ioaddr + GMAC_HASH_HIGH), readl(ioaddr + GMAC_HASH_LOW));

	return;
}

static void gmac_flow_ctrl(unsigned long ioaddr, unsigned int duplex,
				unsigned int fc, unsigned int pause_time)
{
	unsigned int flow = 0;

	DBG(KERN_DEBUG "GMAC Flow-Control:\n");
	if (fc & FLOW_RX) {
		DBG(KERN_DEBUG "\tReceive Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_RFE;
	}
	if (fc & FLOW_TX) {
		DBG(KERN_DEBUG "\tTransmit Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_TFE;
	}

	if (duplex) {
		DBG(KERN_DEBUG "\tduplex mode: pause time: %d\n", pause_time);
		flow |= (pause_time << GMAC_FLOW_CTRL_PT_SHIFT);
	}

	writel(flow, ioaddr + GMAC_FLOW_CTRL);
	return;
}

static void gmac_pmt(unsigned long ioaddr, unsigned long mode)
{
	unsigned int pmt = 0;

	if (mode == WAKE_MAGIC) {
		DBG(KERN_DEBUG "GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	} else if (mode == WAKE_UCAST) {
		DBG(KERN_DEBUG "GMAC: WOL on global unicast\n");
		pmt |= global_unicast;
	}

	writel(pmt, ioaddr + GMAC_PMT);
	return;
}

struct stmmac_ops gmac_driver = {
	.core_init = gmac_core_init,
	.dump_mac_regs = gmac_dump_regs,
	.dma_init = gmac_dma_init,
	.dump_dma_regs = gmac_dump_dma_regs,
	.dma_mode = gmac_dma_operation_mode,
	.dma_diagnostic_fr = gmac_dma_diagnostic_fr,
	.set_filter = gmac_set_filter,
	.flow_ctrl = gmac_flow_ctrl,
	.pmt = gmac_pmt,
	.host_irq_status = gmac_irq_status,
	.set_umac_addr = gmac_set_umac_addr,
	.get_umac_addr = gmac_get_umac_addr,
};

/* Transmit FIFO flush operation */
void gmac_flush_tx_fifo(unsigned long ioaddr)
{
	u32 csr6 = readl(ioaddr + DMA_CONTROL);
	writel((csr6 | DMA_CONTROL_FTF), ioaddr + DMA_CONTROL);

	do {} while ((readl(ioaddr + DMA_CONTROL) & DMA_CONTROL_FTF));
}

struct mac_device_info *gmac_setup(unsigned long ioaddr)
{
	struct mac_device_info *mac;
	u32 uid = readl(ioaddr + GMAC_VERSION);

	pr_info("\tGMAC - user ID: 0x%x, Synopsys ID: 0x%x\n",
		((uid & 0x0000ff00) >> 8), (uid & 0x000000ff));

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);

	mac->ops = &gmac_driver;
	mac->pmt = PMT_SUPPORTED;
	mac->link.port = GMAC_CONTROL_PS;
	mac->link.duplex = GMAC_CONTROL_DM;
	mac->link.speed = GMAC_CONTROL_FES;
	mac->mii.addr = GMAC_MII_ADDR;
	mac->mii.data = GMAC_MII_DATA;

	return mac;
}
