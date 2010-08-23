/*
 * This is the driver for the MAC 10/100 on-chip Ethernet controller
 * currently tested on all the ST boards based on STb7109 and stx7200 SoCs.
 *
 * DWC Ether MAC 10/100 Universal version 4.0 has been used for developing
 * this code.
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

#include "common.h"
#include "mac100.h"

#undef MAC100_DEBUG
/*#define MAC100_DEBUG*/
#ifdef MAC100_DEBUG
#define DBG(fmt, args...) printk(fmt, ## args)
#else
#define DBG(fmt, args...) do { } while (0)
#endif

static void mac100_core_init(unsigned long ioaddr, int disable_readahead,
		int csum_engine)
{
	u32 value = readl(ioaddr + MAC_CONTROL);

	writel((value | MAC_CORE_INIT), ioaddr + MAC_CONTROL);

#ifdef STMMAC_VLAN_TAG_USED
	writel(ETH_P_8021Q, ioaddr + MAC_VLAN1);
#endif
	return;
}

static void mac100_dump_mac_regs(unsigned long ioaddr)
{
	pr_info("\t----------------------------------------------\n"
		"\t MAC100 CSR (base addr = 0x%8x)\n"
		"\t----------------------------------------------\n",
		(unsigned int)ioaddr);
	pr_info("\tcontrol reg (offset 0x%x): 0x%08x\n", MAC_CONTROL,
			readl(ioaddr + MAC_CONTROL));
	pr_info("\taddr HI (offset 0x%x): 0x%08x\n ", MAC_ADDR_HIGH,
			readl(ioaddr + MAC_ADDR_HIGH));
	pr_info("\taddr LO (offset 0x%x): 0x%08x\n", MAC_ADDR_LOW,
			readl(ioaddr + MAC_ADDR_LOW));
	pr_info("\tmulticast hash HI (offset 0x%x): 0x%08x\n",
			MAC_HASH_HIGH, readl(ioaddr + MAC_HASH_HIGH));
	pr_info("\tmulticast hash LO (offset 0x%x): 0x%08x\n",
			MAC_HASH_LOW, readl(ioaddr + MAC_HASH_LOW));
	pr_info("\tflow control (offset 0x%x): 0x%08x\n",
			MAC_FLOW_CTRL, readl(ioaddr + MAC_FLOW_CTRL));
	pr_info("\tVLAN1 tag (offset 0x%x): 0x%08x\n", MAC_VLAN1,
			readl(ioaddr + MAC_VLAN1));
	pr_info("\tVLAN2 tag (offset 0x%x): 0x%08x\n", MAC_VLAN2,
			readl(ioaddr + MAC_VLAN2));
	pr_info("\n\tMAC management counter registers\n");
	pr_info("\t MMC crtl (offset 0x%x): 0x%08x\n",
			MMC_CONTROL, readl(ioaddr + MMC_CONTROL));
	pr_info("\t MMC High Interrupt (offset 0x%x): 0x%08x\n",
			MMC_HIGH_INTR, readl(ioaddr + MMC_HIGH_INTR));
	pr_info("\t MMC Low Interrupt (offset 0x%x): 0x%08x\n",
			MMC_LOW_INTR, readl(ioaddr + MMC_LOW_INTR));
	pr_info("\t MMC High Interrupt Mask (offset 0x%x): 0x%08x\n",
			MMC_HIGH_INTR_MASK, readl(ioaddr + MMC_HIGH_INTR_MASK));
	pr_info("\t MMC Low Interrupt Mask (offset 0x%x): 0x%08x\n",
			MMC_LOW_INTR_MASK, readl(ioaddr + MMC_LOW_INTR_MASK));
	return;
}

static int mac100_dma_init(unsigned long ioaddr, int pbl, u32 dma_tx,
		u32 dma_rx)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);
	do {} while ((readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET));

	/* Enable Application Access by writing to DMA CSR0 */
	writel(DMA_BUS_MODE_DEFAULT | (pbl << DMA_BUS_MODE_PBL_SHIFT),
			ioaddr + DMA_BUS_MODE);

	/* Mask interrupts by writing to CSR7 */
	writel(DMA_INTR_DEFAULT_MASK, ioaddr + DMA_INTR_ENA);

	/* The base address of the RX/TX descriptor lists must be written into
	 * DMA CSR3 and CSR4, respectively. */
	writel(dma_tx, ioaddr + DMA_TX_BASE_ADDR);
	writel(dma_rx, ioaddr + DMA_RCV_BASE_ADDR);

	return 0;
}

/* Store and Forward capability is not used at all..
 * The transmit threshold can be programmed by
 * setting the TTC bits in the DMA control register.*/
static void mac100_dma_operation_mode(unsigned long ioaddr, int txmode,
		int rxmode)
{
	u32 csr6 = readl(ioaddr + DMA_CONTROL);

	if (txmode <= 32)
		csr6 |= DMA_CONTROL_TTC_32;
	else if (txmode <= 64)
		csr6 |= DMA_CONTROL_TTC_64;
	else
		csr6 |= DMA_CONTROL_TTC_128;

	writel(csr6, ioaddr + DMA_CONTROL);

	return;
}

static void mac100_dump_dma_regs(unsigned long ioaddr)
{
	int i;

	DBG(KERN_DEBUG "MAC100 DMA CSR\n");
	for (i = 0; i < 9; i++)
		pr_debug("\t CSR%d (offset 0x%x): 0x%08x\n", i,
				(DMA_BUS_MODE + i * 4),
				readl(ioaddr + DMA_BUS_MODE + i * 4));
	DBG(KERN_DEBUG "\t CSR20 (offset 0x%x): 0x%08x\n",
		DMA_CUR_TX_BUF_ADDR, readl(ioaddr + DMA_CUR_TX_BUF_ADDR));
	DBG(KERN_DEBUG "\t CSR21 (offset 0x%x): 0x%08x\n",
		DMA_CUR_RX_BUF_ADDR, readl(ioaddr + DMA_CUR_RX_BUF_ADDR));
	return;
}

/* DMA controller has two counters to track the number of
 * the receive missed frames.
 */
static void mac100_dma_diagnostic_fr(void *data, struct stmmac_extra_stats *x,
		unsigned long ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	u32 csr8 = readl(ioaddr + DMA_MISSED_FRAME_CTR);

	if (unlikely(csr8)) {
		if (csr8 & DMA_MISSED_FRAME_OVE) {
			stats->rx_over_errors += 0x800;
			x->rx_overflow_cntr += 0x800;
		} else {
			unsigned int ove_cntr;
			ove_cntr = ((csr8 & DMA_MISSED_FRAME_OVE_CNTR) >> 17);
			stats->rx_over_errors += ove_cntr;
			x->rx_overflow_cntr += ove_cntr;
		}

		if (csr8 & DMA_MISSED_FRAME_OVE_M) {
			stats->rx_missed_errors += 0xffff;
			x->rx_missed_cntr += 0xffff;
		} else {
			unsigned int miss_f = (csr8 & DMA_MISSED_FRAME_M_CNTR);
			stats->rx_missed_errors += miss_f;
			x->rx_missed_cntr += miss_f;
		}
	}
	return;
}

static void mac100_irq_status(unsigned long ioaddr)
{
	return;
}

static void mac100_set_umac_addr(unsigned long ioaddr, unsigned char *addr,
		unsigned int reg_n)
{
	stmmac_set_mac_addr(ioaddr, addr, MAC_ADDR_HIGH, MAC_ADDR_LOW);
}

static void mac100_get_umac_addr(unsigned long ioaddr, unsigned char *addr,
		unsigned int reg_n)
{
	stmmac_get_mac_addr(ioaddr, addr, MAC_ADDR_HIGH, MAC_ADDR_LOW);
}

static void mac100_set_filter(struct net_device *dev)
{
	unsigned long ioaddr = dev->base_addr;
	u32 value = readl(ioaddr + MAC_CONTROL);

	if (dev->flags & IFF_PROMISC) {
		value |= MAC_CONTROL_PR;
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_IF | MAC_CONTROL_HO |
				MAC_CONTROL_HP);
	} else if ((dev->mc_count > HASH_TABLE_SIZE)
			|| (dev->flags & IFF_ALLMULTI)) {
		value |= MAC_CONTROL_PM;
		value &= ~(MAC_CONTROL_PR | MAC_CONTROL_IF | MAC_CONTROL_HO);
		writel(0xffffffff, ioaddr + MAC_HASH_HIGH);
		writel(0xffffffff, ioaddr + MAC_HASH_LOW);
	} else if (dev->mc_count == 0) {	/* no multicast */
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_PR | MAC_CONTROL_IF |
				MAC_CONTROL_HO | MAC_CONTROL_HP);
	} else {
		int i;
		u32 mc_filter[2];
		struct dev_mc_list *mclist;

		/* Perfect filter mode for physical address and Hash
		 * filter for multicast */
		value |= MAC_CONTROL_HP;
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_PR | MAC_CONTROL_IF
				| MAC_CONTROL_HO);

		memset(mc_filter, 0, sizeof(mc_filter));
		for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
				i++, mclist = mclist->next) {
			/* The upper 6 bits of the calculated CRC are used to
			 * index the contens of the hash table */
			int bit_nr =
				ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
		writel(mc_filter[0], ioaddr + MAC_HASH_LOW);
		writel(mc_filter[1], ioaddr + MAC_HASH_HIGH);
	}

	writel(value, ioaddr + MAC_CONTROL);

	DBG(KERN_INFO "%s: CTRL reg: 0x%08x Hash regs: "
			"HI 0x%08x, LO 0x%08x\n",
			__func__, readl(ioaddr + MAC_CONTROL),
			readl(ioaddr + MAC_HASH_HIGH),
			readl(ioaddr + MAC_HASH_LOW));
	return;
}

static void mac100_flow_ctrl(unsigned long ioaddr, unsigned int duplex,
		unsigned int fc, unsigned int pause_time)
{
	unsigned int flow = MAC_FLOW_CTRL_ENABLE;

	if (duplex)
		flow |= (pause_time << MAC_FLOW_CTRL_PT_SHIFT);
	writel(flow, ioaddr + MAC_FLOW_CTRL);

	return;
}

/* No PMT module supported in our SoC for the Ethernet Controller. */
static void mac100_pmt(unsigned long ioaddr, unsigned long mode)
{
	return;
}

struct stmmac_ops mac100_driver = {
	.core_init = mac100_core_init,
	.dump_mac_regs = mac100_dump_mac_regs,
	.dma_init = mac100_dma_init,
	.dump_dma_regs = mac100_dump_dma_regs,
	.dma_mode = mac100_dma_operation_mode,
	.dma_diagnostic_fr = mac100_dma_diagnostic_fr,
	.set_filter = mac100_set_filter,
	.flow_ctrl = mac100_flow_ctrl,
	.pmt = mac100_pmt,
	.host_irq_status = mac100_irq_status,
	.set_umac_addr = mac100_set_umac_addr,
	.get_umac_addr = mac100_get_umac_addr,
};

struct mac_device_info *mac100_setup(unsigned long ioaddr)
{
	struct mac_device_info *mac;

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);

	pr_info("\tMAC 10/100\n");

	mac->ops = &mac100_driver;
	mac->pmt = PMT_NOT_SUPPORTED;
	mac->link.port = MAC_CONTROL_PS;
	mac->link.duplex = MAC_CONTROL_F;
	mac->link.speed = 0;
	mac->mii.addr = MAC_MII_ADDR;
	mac->mii.data = MAC_MII_DATA;

	return mac;
}
