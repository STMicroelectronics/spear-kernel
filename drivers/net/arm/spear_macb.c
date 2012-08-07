/*
 * drivers/net/arm/spear_macb.c
 *
 * SPEAr MACB Ethernet Controller driver
 *
 * Copyright (C) 2004-2006 Atmel Corporation
 * Copyright (C) 2008 STMicroelectronics Corporation
 *
 * This driver bases on atmel macb controller driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <mach/irqs.h>
#include <mach/macb_eth.h>
#include <mach/spear.h>

/* #define DEBUG */
/* #define TEST_LOOPBACK */

#include "spear_macb.h"

#define HW_COUNTER_20MS 0x0EA6
#define HW_COUNTER_1MS 0x00BC
#define HW_COUNTER_CUR HW_COUNTER_1MS

#define TMR_IN_USE	1
#define TMR_FREE	0

#define RX_BUFFER_SIZE		128
#define RX_RING_SIZE		512
#define RX_RING_BYTES		(sizeof(struct dma_desc) * RX_RING_SIZE)

/* Make the IP header word-aligned (the ethernet header is 14 bytes) */
#define RX_OFFSET		2

#define TX_RING_SIZE		(128 + 1)
#define DEF_TX_RING_PENDING	(TX_RING_SIZE - 2)
#define TX_RING_BYTES		(sizeof(struct dma_desc) * TX_RING_SIZE)

#define TX_RING_GAP(bp)						\
	(TX_RING_SIZE - 1 - (bp)->tx_pending)
#define TX_BUFFS_AVAIL(bp)					\
	(((bp)->tx_tail <= (bp)->tx_head) ?			\
	 (bp)->tx_tail + (bp)->tx_pending - (bp)->tx_head :	\
	 (bp)->tx_tail - (bp)->tx_head - TX_RING_GAP(bp))

/* keep the last entry, never fill it */
#define NEXT_TX(n)		(((n) + 1) & (TX_RING_SIZE - 2))
#define NEXT_RX(n)		(((n) + 1) & (RX_RING_SIZE - 1))

/* minimum number of free TX descriptors before waking up TX process */
#define MACB_TX_WAKEUP_THRESH	(TX_RING_SIZE / 4)

#define MACB_RX_INT_FLAGS	(MACB_BIT(RCOMP) | MACB_BIT(RXUBR)	\
				 | MACB_BIT(ISR_ROVR))

/*HW Timer Handlers */

static void spear_chk_tmr_intr(struct macb *bp)
{
	u16 val;

	val = bp->timer_ops->timer_read_status(bp->hw_tx_timeout);
	if (!(val & GPT_CTRL_MATCH_INT)) {
		bp->timer_ops->timer_stop(bp->hw_tx_timeout);
		bp->timer_ops->timer_set_match(bp->hw_tx_timeout,
				GPT_STATUS_MATCH);
		bp->timer_ops->timer_set_load_start(bp->hw_tx_timeout, 0,
					 HW_COUNTER_CUR);
	}
}

static irqreturn_t spear_hw_timer_handler(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct macb *bp = netdev_priv(dev);
	struct spear_timer *hw_timer = bp->hw_tx_timeout;
	unsigned int cur_tx, pre_tx, pos, config;
	unsigned long flags;
	struct mii_bus *bus;

	bp->timer_ops->timer_write_status(hw_timer, GPT_STATUS_MATCH);
	bus = bp->phy_dev->bus;
	spin_lock_irqsave(&bp->lock, flags);

	if ((bp->time_active == 0) || (bp->counter == TMR_FREE)) {
		bp->timer_ops->timer_set_load_start(bp->hw_tx_timeout, 0,
					 HW_COUNTER_CUR);
		bp->counter = TMR_FREE;
		spin_unlock_irqrestore(&bp->lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * get the current tx pointer & the previous tx pointer since sometimes
	 * the tx pointer stops at the prev position. Wrap case don't need to
	 * be considered because hardware wrap never happen.
	 */
	cur_tx = macb_readl(bp, TBQP);
	pos = ((cur_tx - bp->tx_ring_dma) / sizeof(struct dma_desc));
	bp->tx_ring[pos].ctrl &= (~MACB_BIT(TX_ERROR));

	/* if it is 0, just 0, don't wrap */
	if (pos != 0)
		pos = pos - 1;
	pre_tx = bp->tx_ring_dma + pos * sizeof(struct dma_desc);

	/* disable TX */
	config = macb_readl(bp, NCR);
	config &= (~MACB_BIT(TE));
	macb_writel(bp, NCR, config);

	/* rewrite the TX base address */
	if (!(bp->tx_ring[pos].ctrl & MACB_BIT(TX_USED)))
		macb_writel(bp, TBQP, pre_tx);
	else
		macb_writel(bp, TBQP, cur_tx);

	bp->tx_ring[pos].ctrl &= (~MACB_BIT(TX_ERROR));

	/* clear the USED BIT in TSR */
	macb_writel(bp, TSR, MACB_BIT(UBR));
	/* enable TX and kick it start */
	macb_writel(bp, NCR, config | MACB_BIT(TE));
	macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(TSTART));

	spin_unlock_irqrestore(&bp->lock, flags);

	bp->counter = TMR_FREE;
	bp->time_active = 1;
	bp->timer_ops->timer_set_load_start(bp->hw_tx_timeout, 0,
			HW_COUNTER_CUR);
	return IRQ_HANDLED;
}

static int spear_init_hw_timer(struct macb *bp)
{
	struct spear_timer *hw_timer;
	struct net_device *dev = bp->dev;
	int ret;
	u32 period;

	bp->timer_ops = &timer_ops;
	hw_timer = bp->timer_ops->timer_request();
	if (hw_timer == NULL) {
		dev_info(&bp->pdev->dev, "Failed to get the HW timer.\n");
		return -EBUSY;
	}

	bp->timer_ops->timer_set_source(hw_timer, GPT_SRC_PLL3_CLK);
	bp->timer_ops->timer_set_prescalar(hw_timer, GPT_CTRL_PRESCALER256);

	period = clk_get_rate(hw_timer->clk) / HZ;
	period >>= GPT_CTRL_PRESCALER256;
	bp->timer_ops->timer_set_match(hw_timer, GPT_STATUS_MATCH);

	ret = request_irq(hw_timer->irq, spear_hw_timer_handler,
			 0 , "SPEAr_eth_tmr", dev);
	if (ret) {
		dev_info(&bp->pdev->dev, "Unable to request irq for timer.\n");
		bp->timer_ops->timer_free(hw_timer);
		return ret;
	}

	bp->hw_tx_timeout = hw_timer;

	return 0;
}

static void __macb_set_hwaddr(struct macb *bp)
{
	u32 bottom;
	u16 top;

	bottom = cpu_to_le32(*((u32 *) bp->dev->dev_addr));
	macb_writel(bp, SA1B, bottom);
	top = cpu_to_le16(*((u16 *) (bp->dev->dev_addr + 4)));
	macb_writel(bp, SA1T, top);
}

static void __init macb_get_hwaddr(struct macb *bp)
{
	u32 bottom;
	u16 top;
	u8 addr[6];

	bottom = macb_readl(bp, SA1B);
	top = macb_readl(bp, SA1T);

	addr[0] = bottom & 0xff;
	addr[1] = (bottom >> 8) & 0xff;
	addr[2] = (bottom >> 16) & 0xff;
	addr[3] = (bottom >> 24) & 0xff;
	addr[4] = top & 0xff;
	addr[5] = (top >> 8) & 0xff;

	if (is_valid_ether_addr(addr))
		memcpy(bp->dev->dev_addr, addr, sizeof(addr));
}

static int macb_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	int value;
	struct macb *bp;
	struct macb_base_data *pdata;

	bp = bus->priv;
	pdata = dev_get_platdata(&bp->pdev->dev);

	if (pdata->plat_mdio_control)
		pdata->plat_mdio_control(bp->pdev);

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_READ)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)));

	/* wait for end of transfer */
	while (!MACB_BFEXT(IDLE, macb_readl(bp, NSR)))
		cpu_relax();

	value = MACB_BFEXT(DATA, macb_readl(bp, MAN));

	return value;
}

static int macb_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
		u16 value)
{
	struct macb *bp;
	struct macb_base_data *pdata;

	bp = bus->priv;
	pdata = dev_get_platdata(&bp->pdev->dev);

	if (pdata->plat_mdio_control)
		pdata->plat_mdio_control(bp->pdev);

	macb_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF)
			      | MACB_BF(RW, MACB_MAN_WRITE)
			      | MACB_BF(PHYA, mii_id)
			      | MACB_BF(REGA, regnum)
			      | MACB_BF(CODE, MACB_MAN_CODE)
			      | MACB_BF(DATA, value)));

	/* wait for end of transfer */
	while (!MACB_BFEXT(IDLE, macb_readl(bp, NSR)))
		cpu_relax();

	return 0;
}

static int macb_mdio_reset(struct mii_bus *bus)
{
	return 0;
}

static void macb_handle_link_change(struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;
	unsigned long flags;
	struct mii_bus *bus;
	int status_change = 0;

	bus = bp->phy_dev->bus;
	spin_lock_irqsave(&bp->lock, flags);

	if (phydev->link) {
		if ((bp->speed != phydev->speed) ||
		 (bp->duplex != phydev->duplex)) {
			u32 reg;

			reg = macb_readl(bp, NCFGR);
			reg &= ~(MACB_BIT(SPD) | MACB_BIT(FD));
			if (phydev->duplex)
				reg |= MACB_BIT(FD);

			if (phydev->speed == SPEED_100)
				reg |= MACB_BIT(SPD);

			macb_writel(bp, NCFGR, reg);
			bp->speed = phydev->speed;
			bp->duplex = phydev->duplex;
			status_change = 1;
		}
	}

	if (phydev->link != bp->link) {
		if (!phydev->link) {
			bp->speed = 0;
			bp->duplex = -1;
		}
		bp->link = phydev->link;
		status_change = 1;
	}

	spin_unlock_irqrestore(&bp->lock, flags);

	if (status_change) {
		if (phydev->link)
			dev_info(&bp->pdev->dev, "%s: link up (%d/%s)\n",
				 dev->name, phydev->speed,
				 DUPLEX_FULL ==
				 phydev->duplex ? "Full" : "Half");
		else
			dev_info(&bp->pdev->dev, "%s: link down\n", dev->name);
	}
}

/* based on au1000_eth. c*/
static int macb_mii_probe(struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	struct macb_base_data *pdata;
	int phy_addr;

	pdata = bp->pdev->dev.platform_data;
	/* find the first phy */
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (bp->mii_bus->phy_map[phy_addr]) {
			if (pdata->phy_addr == phy_addr)
				phydev = bp->mii_bus->phy_map[phy_addr];
		}
	}

	if (!phydev) {
		printk(KERN_ERR "%s: no PHY found\n", dev->name);
		return -1;
	}

	/* attach the mac to the phy */
	if (pdata && pdata->is_rmii) {
		phydev = phy_connect(dev, dev_name(&phydev->dev),
			&macb_handle_link_change, 0, PHY_INTERFACE_MODE_RMII);
	} else {
		phydev = phy_connect(dev, dev_name(&phydev->dev),
			&macb_handle_link_change, 0, PHY_INTERFACE_MODE_MII);
	}

	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	/* mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = phydev->supported;
	bp->link = 0;
	bp->speed = 0;
	bp->duplex = -1;
	bp->phy_dev = phydev;

	return 0;
}

static int macb_mii_init(struct macb *bp)
{
	struct macb_base_data *pdata;
	int err = -ENXIO, i;

	/* Enable management port */
	macb_writel(bp, NCR, MACB_BIT(MPE));

	bp->mii_bus = mdiobus_alloc();
	if (bp->mii_bus == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	bp->mii_bus->name = "MACB_mii_bus";
	bp->mii_bus->read = &macb_mdio_read;
	bp->mii_bus->write = &macb_mdio_write;
	bp->mii_bus->reset = &macb_mdio_reset;
	bp->mii_bus->priv = bp;
	bp->mii_bus->parent = &bp->dev->dev;
	pdata = bp->pdev->dev.platform_data;

	if (pdata) {
		bp->mii_bus->phy_mask = pdata->phy_mask;
		snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%x",
				pdata->bus_id);
	} else {
		snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%x",
				bp->pdev->id);
	}

	bp->mii_bus->irq = kmalloc(sizeof(int)*PHY_MAX_ADDR, GFP_KERNEL);
	if (!bp->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		bp->mii_bus->irq[i] = PHY_POLL;

	platform_set_drvdata(bp->dev, bp->mii_bus);

	if (mdiobus_register(bp->mii_bus))
		goto err_out_free_mdio_irq;

	if (macb_mii_probe(bp->dev) != 0)
		goto err_out_unregister_bus;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(bp->mii_bus);
err_out_free_mdio_irq:
	kfree(bp->mii_bus->irq);
err_out_free_mdiobus:
	mdiobus_free(bp->mii_bus);
err_out:
	return err;
}

static void macb_update_stats(struct macb *bp)
{
	u32 __iomem *reg = bp->regs + MACB_PFR;
	u32 *p = &bp->hw_stats.rx_pause_frames;
	u32 *end = &bp->hw_stats.tx_pause_frames + 1;

	WARN_ON((unsigned long)(end - p - 1) != (MACB_TPF - MACB_PFR) / 4);

	for (; p < end; p++, reg++)
		*p += __raw_readl(reg);
}

static void macb_tx(struct macb *bp)
{
	unsigned int tail;
	unsigned int head;
	u32 status, config, cur_tx, tsr;
	u32 tmp, pos;
	status = macb_readl(bp, TSR);
	macb_writel(bp, TSR, (status & (~MACB_BIT(UBR))));

#ifdef DEBUG
	dev_info(&bp->pdev->dev, "in macb_tx.\n");
#endif
	spear_chk_tmr_intr(bp);
	bp->counter = TMR_FREE;
	bp->time_active = 0;

	config = macb_readl(bp, NCFGR);
	dev_dbg(&bp->pdev->dev, "macb_tx status = %02lx\n",
		(unsigned long)status);

	if (status & MACB_BIT(TSR_RLE)) {
		/* disable Tx DMA Engine */
		config = macb_readl(bp, NCR);
		config &= (~MACB_BIT(TE));
		macb_writel(bp, NCR, config);

		cur_tx = macb_readl(bp, TBQP);
		pos = ((cur_tx - bp->tx_ring_dma) / sizeof(struct dma_desc));
		bp->tx_ring[pos].ctrl &= (~(MACB_BIT(TX_USED) |
					MACB_BIT(TX_ERROR)));
		bp->counter = TMR_IN_USE;
		spear_chk_tmr_intr(bp);
		bp->time_active = 1;
		/* clear the USED bit */
		macb_writel(bp, TSR, MACB_BIT(UBR));
		/* reset the tx base address */
		macb_writel(bp, TBQP, cur_tx);
		/* start transmit */
		macb_writel(bp, NCR, config | MACB_BIT(TE));
		macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(TSTART));
	}

	if (status & MACB_BIT(UND)) {
		int i;

		dev_info(&bp->pdev->dev, "%s: TX underrun, resetting buffers\n",
			 bp->dev->name);
		head = bp->tx_head;
		/*Mark all the buffer as used to avoid sending a lost buffer */
		for (i = 0; i < TX_RING_SIZE; i++)
			bp->tx_ring[i].ctrl = MACB_BIT(TX_USED);

		/* free transmit buffer in upper layer */
		for (tail = bp->tx_tail; tail != head; tail = NEXT_TX(tail)) {
			struct ring_info *rp = &bp->tx_skb[tail];
			struct sk_buff *skb = rp->skb;

			BUG_ON(skb == NULL);
			rmb();
			dma_unmap_single(&bp->pdev->dev, rp->mapping, skb->len,
					 DMA_TO_DEVICE);
			rp->skb = NULL;
			dev_kfree_skb_irq(skb);
		}

		bp->tx_head = bp->tx_tail = 0;
		bp->first = 1;
		/* clear the USED bit */
		macb_writel(bp, TSR, MACB_BIT(UBR));
		config = macb_readl(bp, NCR);
		config &= (~MACB_BIT(TE));
		macb_writel(bp, NCR, config);
		macb_writel(bp, TBQP, bp->tx_ring_dma);
		macb_writel(bp, NCR, config | MACB_BIT(TE));

		return;
	}

	tsr = macb_readl(bp, TSR);
	if (tsr & MACB_BIT(UBR)) {
		cur_tx = macb_readl(bp, TBQP);
		tmp = (u32) (bp->tx_ring_dma + (TX_RING_SIZE - 1)
			* sizeof(struct dma_desc));
		if (tmp == cur_tx) {
			if (!(bp->tx_ring[0].ctrl & MACB_BIT(TX_USED))) {
				bp->counter = TMR_IN_USE;
				spear_chk_tmr_intr(bp);
				bp->time_active = 1;
			}

			config = macb_readl(bp, NCR);
			config &= (~MACB_BIT(TE));
			macb_writel(bp, NCR, config);

			/* clear the USED bit */
			macb_writel(bp, TSR, MACB_BIT(UBR));
			/* reset the tx base address */
			macb_writel(bp, TBQP, bp->tx_ring_dma);
			/* start transmit */
			macb_writel(bp, NCR, config | MACB_BIT(TE));
			macb_writel(bp, NCR,
				macb_readl(bp, NCR) | MACB_BIT(TSTART));
		} else {
			/* handle hardware race condition here
			first we must grantee it is not the
			case that retry times limit happens
			 */
			if (!(tsr & MACB_BIT(TSR_RLE))) {
				pos = ((cur_tx - bp->tx_ring_dma) /
						sizeof(struct dma_desc));
				if (!(bp->tx_ring[pos].ctrl &
						MACB_BIT(TX_USED))) {
					bp->counter = TMR_IN_USE;
					spear_chk_tmr_intr(bp);
					bp->time_active = 1;
				}

				config = macb_readl(bp, NCR);
				config &= (~MACB_BIT(TE));
				macb_writel(bp, NCR, config);

				/* clear the USED bit */
				macb_writel(bp, TSR, MACB_BIT(UBR));
				/* reset the tx base address */
				macb_writel(bp, TBQP, cur_tx);
				/* start transmit */
				macb_writel(bp, NCR, config | MACB_BIT(TE));
				macb_writel(bp, NCR,
					macb_readl(bp, NCR) | MACB_BIT(TSTART));
			}
		}
	} else {
		bp->counter = TMR_IN_USE;
		spear_chk_tmr_intr(bp);
		bp->time_active = 1;
	}

	if (!(status & MACB_BIT(COMP)))
		/*
		 * This may happen when a buffer becomes complete
		 * between reading the ISR and scanning the
		 * descriptors. Nothing to worry about.
		 */
		return;

	head = bp->tx_head;
	for (tail = bp->tx_tail; tail != head; tail = NEXT_TX(tail)) {
		struct ring_info *rp = &bp->tx_skb[tail];
		struct sk_buff *skb = rp->skb;
		u32 bufstat;

		BUG_ON(skb == NULL);

		rmb();
		bufstat = bp->tx_ring[tail].ctrl;
		if (!(bufstat & MACB_BIT(TX_USED))
			|| (bufstat & MACB_BIT(TX_ERROR)))
			break;

		dev_dbg(&bp->pdev->dev, "skb %u (data %p) TX complete\n",
			tail, skb->data);
		dma_unmap_single(&bp->pdev->dev, rp->mapping, skb->len,
				 DMA_TO_DEVICE);
		bp->stats.tx_packets++;
		bp->stats.tx_bytes += skb->len;
		rp->skb = NULL;
		dev_kfree_skb_irq(skb);

	}

	bp->tx_tail = tail;
	if (netif_queue_stopped(bp->dev) &&
		TX_BUFFS_AVAIL(bp) > MACB_TX_WAKEUP_THRESH)
		netif_wake_queue(bp->dev);
}

static int macb_rx_frame(struct macb *bp, unsigned int first_frag,
			 unsigned int last_frag)
{
	unsigned int len;
	unsigned int frag;
	unsigned int offset = 0;
	struct sk_buff *skb;

	len = MACB_BFEXT(RX_FRMLEN, bp->rx_ring[last_frag].ctrl);
#ifdef DEBUG
	dev_info(&bp->pdev->dev, "macb_rx_frame frags %u - %u (len %u)\n",
		 first_frag, last_frag, len);
#endif
	skb = netdev_alloc_skb(bp->dev, len + RX_OFFSET);
	if (!skb) {
		bp->stats.rx_dropped++;
		for (frag = first_frag;; frag = NEXT_RX(frag)) {
			bp->rx_ring[frag].addr &= ~MACB_BIT(RX_USED);
			if (frag == last_frag)
				break;
		}
		wmb();
		return 1;
	}

	skb_reserve(skb, RX_OFFSET);
	skb->ip_summed = CHECKSUM_NONE;
	skb_put(skb, len);

	for (frag = first_frag;; frag = NEXT_RX(frag)) {
		unsigned int frag_len = RX_BUFFER_SIZE;

		if (offset + frag_len > len) {
			BUG_ON(frag != last_frag);
			frag_len = len - offset;
		}
		memcpy(skb->data + offset,
			(bp->rx_buffers + (RX_BUFFER_SIZE * frag)), frag_len);
		offset += RX_BUFFER_SIZE;
		bp->rx_ring[frag].addr &= ~MACB_BIT(RX_USED);
		wmb();
		if (frag == last_frag)
			break;
	}

	skb->protocol = eth_type_trans(skb, bp->dev);
#ifdef TEST_LOOPBACK
	dev_info(&bp->pdev->dev, "receive data is :\n", skb->len);
	for (i = 0; i < skb->len; i++) {
		dev_info(&bp->pdev->dev, "%02x ", skb->data[i]);
		if (((i + 1) % 32) == 0)
			printk(KERN_INFO "\n");
	}
	dev_info(&bp->pdev->dev, "\n");
#endif
	bp->stats.rx_packets++;
	bp->stats.rx_bytes += len;
	bp->dev->last_rx = jiffies;
	dev_dbg(&bp->pdev->dev, "received skb of length %u, csum: %08x\n",
		skb->len, skb->csum);
	netif_receive_skb(skb);

	return 0;
}

/* Mark DMA descriptors from begin up to and not including end as unused */
static void discard_partial_frame(struct macb *bp, unsigned int begin,
				 unsigned int end)
{
	unsigned int frag;

#ifdef DEBUG
	dev_info(&bp->pdev->dev, "in discard_partial_frame.\n");
#endif

	for (frag = begin; frag != end; frag = NEXT_RX(frag))
		bp->rx_ring[frag].addr &= ~MACB_BIT(RX_USED);
	wmb();

	/*
	 * When this happens, the hardware stats registers for
	 * whatever caused this is updated, so we don't have to record
	 * anything.
	 */
}

static int macb_rx(struct macb *bp, int budget)
{
	int received = 0;
	unsigned int tail = bp->rx_tail;
	int first_frag = -1;

#ifdef DEBUG
	dev_info(&bp->pdev->dev, "in macb_rx.\n");
#endif

	for (; budget > 0; tail = NEXT_RX(tail)) {
		u32 addr, ctrl;

		rmb();
		addr = bp->rx_ring[tail].addr;
		ctrl = bp->rx_ring[tail].ctrl;

#ifdef DEBUG
		dev_info(&bp->pdev->dev, "addr = 0x%08x.\n", addr);
		dev_info(&bp->pdev->dev, "ctrl = 0x%08x.\n", ctrl);
#endif
		if (!(addr & MACB_BIT(RX_USED)))
			break;

		if (ctrl & MACB_BIT(RX_SOF)) {
			if (first_frag != -1)
				discard_partial_frame(bp, first_frag, tail);
			first_frag = tail;
		}

		if (ctrl & MACB_BIT(RX_EOF)) {
			int dropped;
			BUG_ON(first_frag == -1);

			dropped = macb_rx_frame(bp, first_frag, tail);
			first_frag = -1;
			if (!dropped) {
				received++;
				budget--;
			}
		}
	}

	if (first_frag != -1)
		bp->rx_tail = first_frag;
	else
		bp->rx_tail = tail;

	return received;
}

static int macb_poll(struct napi_struct *napi, int budget)
{
	struct macb *bp = container_of(napi, struct macb, napi);
	int work_done;
	u32 status;

	status = macb_readl(bp, RSR);
	macb_writel(bp, RSR, status);

#ifdef DEBUG
	dev_info(&bp->pdev->dev, "in macb_poll.\n");
	dev_info(&bp->pdev->dev, "status = 0x%08x.\n", status);
#endif

	work_done = 0;
	if (!status) {
		/*
		 * This may happen if an interrupt was pending before
		 * this function was called last time, and no packets
		 * have been received since.
		 */
		napi_complete(napi);
		goto out;
	}
	dev_dbg(&bp->pdev->dev, "poll: status = %08lx, budget = %d\n",
		(unsigned long)status, budget);
	if (!(status & MACB_BIT(REC))) {
		dev_warn(&bp->pdev->dev,
			 "No RX buffers complete, status = %02lx\n",
			 (unsigned long)status);
		napi_complete(napi);
		goto out;
	}

	work_done = macb_rx(bp, budget);
	if (work_done < budget)
		napi_complete(napi);

	/*
	 * We've done what we can to clean the buffers. Make sure we
	 * get notified when new packets arrive.
	 */
out:
	macb_writel(bp, IER, MACB_RX_INT_FLAGS);

	/* TODO: Handle errors */

	return work_done;
}
static irqreturn_t macb_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct macb *bp = netdev_priv(dev);
	u32 status;
	unsigned long flags;

	status = macb_readl(bp, ISR);

	if (unlikely(!status))
		return IRQ_NONE;

#ifdef DEBUG
	dev_info(&bp->pdev->dev, "macb_interrupt.\n");
	dev_info(&bp->pdev->dev, "status = 0x%08x.\n", status);
#endif
	spin_lock_irqsave(&bp->lock, flags);

	while (status) {
		/* close possible race with dev_close */
		if (unlikely(!netif_running(dev))) {
			macb_writel(bp, IDR, ~0UL);
			break;
		}

		if (status & MACB_RX_INT_FLAGS) {
			if (napi_schedule_prep(&bp->napi)) {
				/*
				 * There's no point taking any more interrupts
				 * until we have processed the buffers
				 */
				macb_writel(bp, IDR, MACB_RX_INT_FLAGS);
				dev_dbg(&bp->pdev->dev,
					"scheduling RX softirq\n");
				__napi_schedule(&bp->napi);
			}
		}

		if (status & (MACB_BIT(TCOMP) | MACB_BIT(ISR_TUND) |
				MACB_BIT(ISR_RLE)))
			macb_tx(bp);

		/*
		 * Link change detection isn't possible with RMII, so we'll
		 * add that if/when we get our hands on a full-blown MII PHY.
		 */

		if (status & MACB_BIT(HRESP)) {
			/*
			 * TODO: Reset the hardware, and maybe move the printk
			 * to a lower-priority context as well (work queue?)
			 */
			dev_dbg(&bp->pdev->dev,
				"%s: DMA bus error: HRESP not OK\n", dev->name);
		}

		status = macb_readl(bp, ISR);
	}
	spin_unlock_irqrestore(&bp->lock, flags);

	return IRQ_HANDLED;
}
static int macb_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	dma_addr_t mapping;
	unsigned int len, entry, pos;
	u32 ctrl, tsr, config, cur_tx;
	unsigned long flags;
	u32 tmp;

#ifdef DEBUG
	int i;

	dev_info(&bp->pdev->dev,
		 "start_xmit: len %u head %p data %p tail %p end %p\n",
		 skb->len, skb->head, skb->data, skb_tail_pointer(skb),
		 skb_end_pointer(skb));
	dev_info(&bp->pdev->dev, "data:");
	for (i = 0; i < skb->len; i++)
		printk(KERN_INFO " %02x", (unsigned int)skb->data[i]);
	printk("\n");

#endif

	len = skb->len;
	spin_lock_irqsave(&bp->lock, flags);
	/* This is a hard error, log it. */
	if (TX_BUFFS_AVAIL(bp) < 1) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&bp->lock, flags);
		dev_err(&bp->pdev->dev,
			"BUG! Tx Ring full when queue awake!\n");
		dev_dbg(&bp->pdev->dev, "tx_head = %u, tx_tail = %u\n",
			bp->tx_head, bp->tx_tail);
		return NETDEV_TX_BUSY;
	}

	entry = bp->tx_head;
	dev_dbg(&bp->pdev->dev, "Allocated ring entry %u\n", entry);
	mapping = dma_map_single(&bp->pdev->dev, skb->data, len,
			DMA_TO_DEVICE);
	bp->tx_skb[entry].skb = skb;
	bp->tx_skb[entry].mapping = mapping;
	dev_dbg(&bp->pdev->dev, "Mapped skb data %p to DMA addr %08lx\n",
		skb->data, (unsigned long)mapping);

	ctrl = MACB_BF(TX_FRMLEN, len);
	ctrl |= MACB_BIT(TX_LAST);
	if (entry == (TX_RING_SIZE - 1))
		ctrl |= MACB_BIT(TX_WRAP);

	bp->tx_ring[entry].addr = mapping;
	wmb();
	bp->tx_ring[entry].ctrl = ctrl;
	wmb();

	bp->tx_head = NEXT_TX(entry);

	tsr = macb_readl(bp, TSR);
	if (tsr & MACB_BIT(UBR)) {
		cur_tx = macb_readl(bp, TBQP);
		config = macb_readl(bp, NCR);
		config &= (~MACB_BIT(TE));
		macb_writel(bp, NCR, config);
		/* if it stops at the last entry we never fill
		wrap it manually */
		tmp = (u32) (bp->tx_ring_dma + (TX_RING_SIZE - 1)
				* sizeof(struct dma_desc));
		if (cur_tx == tmp) {
			if (!(bp->tx_ring[0].ctrl & MACB_BIT(TX_USED))) {
				if (bp->time_active == 0)
					spear_chk_tmr_intr(bp);
				bp->time_active = 1;
			}
			/* clear the USED bit */
			macb_writel(bp, TSR, MACB_BIT(UBR));
			/* fix hardware issue here */
			macb_writel(bp, TBQP, bp->tx_ring_dma);
			macb_writel(bp, NCR, config | MACB_BIT(TE));
			/* start transmit */
			macb_writel(bp, NCR,
				macb_readl(bp, NCR) | MACB_BIT(TSTART));
		} else
			/* double check: in case it has sent
			the current packet */
		if (cur_tx == (u32) (bp->tx_ring_dma + bp->tx_head
					* sizeof(struct dma_desc))) {
			/* but this case will enter interrupt later,
			so maybe be ommited */
			if (bp->time_active)
				bp->time_active = 0;
			/* clear the USED bit */
			macb_writel(bp, TSR, MACB_BIT(UBR));
			/* fix hardware issue here */
			macb_writel(bp, TBQP, cur_tx);
			macb_writel(bp, NCR, config | MACB_BIT(TE));
			/* start transmit */
			macb_writel(bp, NCR,
				macb_readl(bp, NCR) | MACB_BIT(TSTART));
		} else {
			/*
			 * There's one case that, interrupt has generated but
			 * not enterred because of spin_lock_irq. Later, the
			 * interrupt will clear the timer created now, because
			 * of the last sent buffer. It's OK because interrupt
			 * will create another timer if there's data left to
			 * tranmit in TX FIFO
			 */
			if (bp->time_active == 0)
				spear_chk_tmr_intr(bp);
			bp->time_active = 1;
			tmp = cur_tx - bp->tx_ring_dma;
			pos = tmp / sizeof(struct dma_desc);

			if (cur_tx != (u32) (bp->tx_ring_dma +
					entry * sizeof(struct dma_desc))) {
				dev_info(&bp->pdev->dev, "the position is not \
						right: cur = %d, entry = %d.\n"
						, pos, entry);
			}

			/* clear the USED bit */
			macb_writel(bp, TSR, MACB_BIT(UBR));
			/* fix hardware issue here */
			macb_writel(bp, TBQP, cur_tx);
			macb_writel(bp, NCR, config | MACB_BIT(TE));
			/* start transmit */
			macb_writel(bp, NCR,
				macb_readl(bp, NCR) | MACB_BIT(TSTART));
		}
	}

	if (bp->first) {
		spear_chk_tmr_intr(bp);
		bp->time_active = 1;

		macb_writel(bp, NCR, macb_readl(bp, NCR) | MACB_BIT(TSTART));
		bp->first = 0;
	}

	if (TX_BUFFS_AVAIL(bp) < 1)
		netif_stop_queue(dev);

	bp->counter = TMR_IN_USE;
	if (bp->time_active == 0)
		spear_chk_tmr_intr(bp);
	bp->time_active = 1;
	spin_unlock_irqrestore(&bp->lock, flags);
	dev->trans_start = jiffies;
#ifdef DEBUG
	/* dump the register */
	reg = macb_readl(bp, TSR);
	dev_info(&bp->pdev->dev, "Transmit status register is 0x%08x.\n", reg);
	reg = macb_readl(bp, NCR);
	dev_info(&bp->pdev->dev, "The network control register is 0x%08x.\n",
		 reg);
	reg = macb_readl(bp, NCFGR);
	dev_info(&bp->pdev->dev,
		 "The network configurtion register is 0x%08x.\n", reg);

#endif
	return NETDEV_TX_OK;
}

static void macb_free_consistent(struct macb *bp)
{
	kfree(bp->tx_skb);
	bp->tx_skb = NULL;

	if (bp->rx_ring) {
		dma_free_coherent(&bp->pdev->dev, RX_RING_BYTES,
				 bp->rx_ring, bp->rx_ring_dma);
		bp->rx_ring = NULL;
	}
	if (bp->tx_ring) {
		dma_free_coherent(&bp->pdev->dev, TX_RING_BYTES,
				 bp->tx_ring, bp->tx_ring_dma);
		bp->tx_ring = NULL;
	}
	if (bp->rx_buffers) {
		dma_free_coherent(&bp->pdev->dev,
				 RX_RING_SIZE * RX_BUFFER_SIZE,
				 bp->rx_buffers, bp->rx_buffers_dma);
		bp->rx_buffers = NULL;
	}
}

static int macb_alloc_consistent(struct macb *bp)
{
	int size;

	size = TX_RING_SIZE * sizeof(struct ring_info);
	bp->tx_skb = kmalloc(size, GFP_KERNEL);
	if (!bp->tx_skb)
		goto out_err;

	size = RX_RING_BYTES;
	bp->rx_ring = dma_alloc_coherent(&bp->pdev->dev, size,
					 &bp->rx_ring_dma, GFP_KERNEL);
	if (!bp->rx_ring)
		goto out_err;
	dev_dbg(&bp->pdev->dev,
		"Allocated RX ring of %d bytes at %08lx (mapped %p)\n",
		size, (unsigned long)bp->rx_ring_dma, bp->rx_ring);

	size = TX_RING_BYTES;
	bp->tx_ring = dma_alloc_coherent(&bp->pdev->dev, size,
					 &bp->tx_ring_dma, GFP_KERNEL);
	if (!bp->tx_ring)
		goto out_err;
	dev_dbg(&bp->pdev->dev,
		"Allocated TX ring of %d bytes at %08lx (mapped %p)\n",
		size, (unsigned long)bp->tx_ring_dma, bp->tx_ring);

	size = RX_RING_SIZE * RX_BUFFER_SIZE;
	bp->rx_buffers = dma_alloc_coherent(&bp->pdev->dev, size,
					&bp->rx_buffers_dma, GFP_KERNEL);
	if (!bp->rx_buffers)
		goto out_err;
	dev_dbg(&bp->pdev->dev,
		"Allocated RX buffers of %d bytes at %08lx (mapped %p)\n",
		size, (unsigned long)bp->rx_buffers_dma, bp->rx_buffers);

	return 0;
out_err:
	macb_free_consistent(bp);
	return -ENOMEM;
}

static void macb_init_rings(struct macb *bp)
{
	int i;
	dma_addr_t addr;

	addr = bp->rx_buffers_dma;
	for (i = 0; i < RX_RING_SIZE; i++) {
		bp->rx_ring[i].addr = addr;
		bp->rx_ring[i].ctrl = 0;
		addr += RX_BUFFER_SIZE;
	}
	bp->rx_ring[RX_RING_SIZE - 1].addr |= MACB_BIT(RX_WRAP);

	for (i = 0; i < TX_RING_SIZE; i++) {
		bp->tx_ring[i].addr = 0;
		bp->tx_ring[i].ctrl = MACB_BIT(TX_USED);
	}
	bp->tx_ring[TX_RING_SIZE - 1].ctrl |= MACB_BIT(TX_WRAP);

	bp->rx_tail = bp->tx_head = bp->tx_tail = 0;
	bp->first = 1;
}

static void macb_reset_hw(struct macb *bp)
{
	/* Make sure we have the write buffer for ourselves */
	wmb();

	/*
	 * Disable RX and TX (XXX: Should we halt the transmission
	 * more gracefully?) and we should not close the mdio port
	 */
	macb_writel(bp, NCR, 0);

	/* Clear the stats registers (XXX: Update stats first?) */
	macb_writel(bp, NCR, MACB_BIT(CLRSTAT));
	/* keep the mdio port , otherwise other eth will not work */
	macb_writel(bp, NCR, MACB_BIT(MPE));

	/* Clear all status flags */
	macb_writel(bp, TSR, ~0UL);
	macb_writel(bp, RSR, ~0UL);

	/* Disable all interrupts */
	macb_writel(bp, IDR, ~0UL);
	macb_readl(bp, ISR);
}

static void macb_init_hw(struct macb *bp)
{
	u32 config;

	macb_reset_hw(bp);
	__macb_set_hwaddr(bp);

	config = macb_readl(bp, NCFGR) | MACB_BF(CLK, 3);
	config |= MACB_BIT(FD);			/* full duplex */
	config |= MACB_BIT(CLK1);		/* MCK equ pclk div by 128*/
	config |= MACB_BIT(SPD);

	dev_info(&bp->pdev->dev, "config = 0x%08x.\n", config);

	if (bp->dev->flags & IFF_PROMISC) {
		config |= MACB_BIT(CAF);	/* Copy All Frames */
		dev_info(&bp->pdev->dev,
			 "this interface receive all the frames.\n");
	}

	if (!(bp->dev->flags & IFF_BROADCAST))
		config |= MACB_BIT(NBC);	/* No BroadCast */
	macb_writel(bp, NCFGR, config);

	/* Initialize TX and RX FIFO pointer */
	macb_writel(bp, RBQP, bp->rx_ring_dma);
	macb_writel(bp, TBQP, bp->tx_ring_dma);

	/* Enable TX and RX */
#ifndef TEST_LOOPBACK
	macb_writel(bp, NCR, MACB_BIT(RE) | MACB_BIT(TE) | MACB_BIT(MPE));
#else
	macb_writel(bp, NCR,
		MACB_BIT(RE) | MACB_BIT(TE) | MACB_BIT(MPE) |
		MACB_BIT(LLB));
#endif

	/* Enable interrupts */
	macb_writel(bp, IER, (MACB_BIT(RCOMP)
			| MACB_BIT(RXUBR)
			| MACB_BIT(ISR_TUND)
			| MACB_BIT(ISR_RLE)
			| MACB_BIT(TXERR)
			| MACB_BIT(TCOMP)
			| MACB_BIT(ISR_ROVR)
			| MACB_BIT(HRESP)));

}

/*
 * The hash address register is 64 bits long and takes up two
 * locations in the memory map. The least significant bits are stored
 * in EMAC_HSL and the most significant bits in EMAC_HSH.
 *
 * The unicast hash enable and the multicast hash enable bits in the
 * network configuration register enable the reception of hash matched
 * frames. The destination address is reduced to a 6 bit index into
 * the 64 bit hash register using the following hash function. The
 * hash function is an exclusive or of every sixth bit of the
 * destination address.
 *
 * hi[5] = da[5] ^ da[11] ^ da[17] ^ da[23] ^ da[29] ^ da[35] ^ da[41] ^ da[47]
 * hi[4] = da[4] ^ da[10] ^ da[16] ^ da[22] ^ da[28] ^ da[34] ^ da[40] ^ da[46]
 * hi[3] = da[3] ^ da[09] ^ da[15] ^ da[21] ^ da[27] ^ da[33] ^ da[39] ^ da[45]
 * hi[2] = da[2] ^ da[08] ^ da[14] ^ da[20] ^ da[26] ^ da[32] ^ da[38] ^ da[44]
 * hi[1] = da[1] ^ da[07] ^ da[13] ^ da[19] ^ da[25] ^ da[31] ^ da[37] ^ da[43]
 * hi[0] = da[0] ^ da[06] ^ da[12] ^ da[18] ^ da[24] ^ da[30] ^ da[36] ^ da[42]
 *
 * da[0] represents the least significant bit of the first byte
 * received, that is, the multicast/unicast indicator, and da[47]
 * represents the most significant bit of the last byte received.  If
 * the hash index, hi[n], points to a bit that is set in the hash
 * register then the frame will be matched according to whether the
 * frame is multicast or unicast.  A multicast match will be signalled
 * if the multicast hash enable bit is set, da[0] is 1 and the hash
 * index points to a bit set in the hash register.  A unicast match
 * will be signalled if the unicast hash enable bit is set, da[0] is 0
 * and the hash index points to a bit set in the hash register.  To
 * receive all multicast frames, the hash register should be set with
 * all ones and the multicast hash enable bit should be set in the
 * network configuration register.
 */

static inline int hash_bit_value(int bitnr, __u8 *addr)
{
	if (addr[bitnr / 8] & (1 << (bitnr % 8)))
		return 1;
	return 0;
}

/*
 * Return the hash index value for the specified address.
 */
static int hash_get_index(__u8 *addr)
{
	int i, j, bitval;
	int hash_index = 0;

	for (j = 0; j < 6; j++) {
		for (i = 0, bitval = 0; i < 8; i++)
			bitval ^= hash_bit_value(i * 6 + j, addr);

		hash_index |= (bitval << j);
	}

	return hash_index;
}

/*
 * Add multicast addresses to the internal multicast-hash table.
 */
static void macb_sethashtable(struct net_device *dev)
{
	struct netdev_hw_addr *ha;
	unsigned long mc_filter[2];
	unsigned int bitnr;
	struct macb *bp = netdev_priv(dev);

	mc_filter[0] = mc_filter[1] = 0;

	netdev_for_each_mc_addr(ha, dev) {
		bitnr = hash_get_index(ha->addr);
		mc_filter[bitnr >> 5] |= 1 << (bitnr & 31);
	}

	macb_writel(bp, HRB, mc_filter[0]);
	macb_writel(bp, HRT, mc_filter[1]);
}

/*
 * Enable/Disable promiscuous and multicast modes.
 */
static void macb_set_rx_mode(struct net_device *dev)
{
	unsigned long cfg;
	struct macb *bp = netdev_priv(dev);

	dev_info(&bp->pdev->dev, "in macb_set_rx_mode.\n");

	cfg = macb_readl(bp, NCFGR);

	if (dev->flags & IFF_PROMISC)
		/* Enable promiscuous mode */
		cfg |= MACB_BIT(CAF);
	else if (dev->flags & (~IFF_PROMISC))
		/* Disable promiscuous mode */
		cfg &= ~MACB_BIT(CAF);

	if (dev->flags & IFF_ALLMULTI) {
		/* Enable all multicast mode */
		macb_writel(bp, HRB, -1);
		macb_writel(bp, HRT, -1);
		cfg |= MACB_BIT(NCFGR_MTI);
		dev_info(&bp->pdev->dev, "enable all multicast mode.\n");
	} else if (!netdev_mc_empty(dev)) {
		/* Enable specific multicasts */
		macb_sethashtable(dev);
		cfg |= MACB_BIT(NCFGR_MTI);
	} else if (dev->flags & (~IFF_ALLMULTI)) {
		/* Disable all multicast mode */
		macb_writel(bp, HRB, 0);
		macb_writel(bp, HRT, 0);
		cfg &= ~MACB_BIT(NCFGR_MTI);
		dev_info(&bp->pdev->dev, "Disable all multicast mode.\n");
	}
	macb_writel(bp, NCFGR, cfg);
}

static int macb_open(struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	int err;
	u32 reg;

	/* if the phy is not yet register, retry later */
#ifndef TEST_LOOPBACK
	if (!bp->phy_dev)
		return -EAGAIN;
#endif

	if (!is_valid_ether_addr(dev->dev_addr)) {
		dev_info(&bp->pdev->dev, "invalid hw address, using random\n");
		random_ether_addr(bp->dev->dev_addr);
	}

	err = macb_alloc_consistent(bp);
	if (err) {
		dev_info(&bp->pdev->dev,
			 "%s: Unable to allocate DMA memory (error %d)\n",
			 dev->name, err);
		return err;
	}

	napi_enable(&bp->napi);
	macb_init_rings(bp);
	macb_init_hw(bp);

#ifndef TEST_LOOPBACK
	/* reset phy - this also wakes it from PDOWN */
	phy_write(bp->phy_dev, MII_BMCR, BMCR_RESET);
	/* schedule a link state check */
	phy_start(bp->phy_dev);
#endif

	netif_start_queue(dev);
	/* dump the register */
	reg = macb_readl(bp, NCR);
	dev_info(&bp->pdev->dev, "The network control register is 0x%08x.\n",
		 reg);
	reg = macb_readl(bp, NCFGR);
	dev_info(&bp->pdev->dev,
		 "The network configurtion register is 0x%08x.\n", reg);

	return 0;
}

static int macb_close(struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	unsigned long flags;

	netif_stop_queue(dev);
	napi_disable(&bp->napi);
	if (bp->phy_dev) {
		phy_stop(bp->phy_dev);
		phy_write(bp->phy_dev, MII_BMCR, BMCR_PDOWN);
	}

	spin_lock_irqsave(&bp->lock, flags);
	bp->timer_ops->timer_stop(bp->hw_tx_timeout);
	bp->time_active = 0;
	macb_reset_hw(bp);
	netif_carrier_off(dev);
	spin_unlock_irqrestore(&bp->lock, flags);
	macb_free_consistent(bp);

	return 0;
}

static struct net_device_stats *macb_get_stats(struct net_device *dev)
{
	struct macb *bp = netdev_priv(dev);
	struct net_device_stats *nstat = &bp->stats;
	struct macb_stats *hwstat = &bp->hw_stats;

	/* read stats from hardware */
	macb_update_stats(bp);

	/* Convert HW stats into netdevice stats */
	nstat->rx_errors = (hwstat->rx_fcs_errors +
			hwstat->rx_align_errors +
			hwstat->rx_resource_errors +
			hwstat->rx_overruns +
			hwstat->rx_oversize_pkts +
			hwstat->rx_jabbers +
			hwstat->rx_undersize_pkts +
			hwstat->sqe_test_errors +
			hwstat->rx_length_mismatch);
	nstat->tx_errors = (hwstat->tx_late_cols +
			hwstat->tx_excessive_cols +
			hwstat->tx_underruns + hwstat->tx_carrier_errors);
	nstat->collisions = (hwstat->tx_single_cols +
			hwstat->tx_multiple_cols +
			hwstat->tx_excessive_cols);
	nstat->rx_length_errors = (hwstat->rx_oversize_pkts +
				hwstat->rx_jabbers +
				hwstat->rx_undersize_pkts +
				hwstat->rx_length_mismatch);
	nstat->rx_over_errors = hwstat->rx_resource_errors;
	nstat->rx_crc_errors = hwstat->rx_fcs_errors;
	nstat->rx_frame_errors = hwstat->rx_align_errors;
	nstat->rx_fifo_errors = hwstat->rx_overruns;
	/* XXX: What does "missed" mean? */
	nstat->tx_aborted_errors = hwstat->tx_excessive_cols;
	nstat->tx_carrier_errors = hwstat->tx_carrier_errors;
	nstat->tx_fifo_errors = hwstat->tx_underruns;
	/* Don't know about heartbeat or window errors... */

	return nstat;
}

static int macb_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct macb *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int macb_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct macb *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static void macb_get_drvinfo(struct net_device *dev,
			struct ethtool_drvinfo *info)
{
	struct macb *bp = netdev_priv(dev);

	strcpy(info->driver, bp->pdev->dev.driver->name);
}

static struct ethtool_ops macb_ethtool_ops = {
	.get_settings = macb_get_settings,
	.set_settings = macb_set_settings,
	.get_drvinfo = macb_get_drvinfo,
	.get_link = ethtool_op_get_link,
};

static int macb_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct macb *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!netif_running(dev)) {
		dev_info(&bp->pdev->dev,
			 "The network interface is not running.\n");
		return -EINVAL;
	}
#ifdef TEST_LOOPBACK
	return generic_mii_ioctl(&bp->mii, if_mii(rq), cmd, NULL);
#else
	if (!phydev) {
		dev_info(&bp->pdev->dev, "The phydev dev is NULL.\n");
		return -ENODEV;
	}
	return phy_mii_ioctl(phydev, rq, cmd);
#endif

}

static const struct net_device_ops macb_netdev_ops = {
	.ndo_open		= macb_open,
	.ndo_stop		= macb_close,
	.ndo_start_xmit		= macb_start_xmit,
	.ndo_set_multicast_list	= macb_set_rx_mode,
	.ndo_get_stats		= macb_get_stats,
	.ndo_do_ioctl		= macb_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= macb_poll_controller,
#endif
};

static int __init macb_probe(struct platform_device *pdev)
{
	struct macb_base_data *pdata;
	struct resource *regs;
	struct net_device *dev;
	struct macb *bp;
	struct phy_device *phydev;
	int err = -ENXIO;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "no mmio resource defined\n");
		goto err_out;
	}

	err = -ENOMEM;
	dev = alloc_etherdev(sizeof(*bp));
	if (!dev) {
		dev_err(&pdev->dev, "etherdev alloc failed, aborting.\n");
		goto err_out;
	}
	/* set the parent dev */
	SET_NETDEV_DEV(dev, &pdev->dev);

	/* TODO: Actually, we have some interesting features... */
	dev->features |= 0;
	bp = netdev_priv(dev);
	bp->pdev = pdev;
	bp->dev = dev;
	pdata = pdev->dev.platform_data;
	spin_lock_init(&bp->lock);
	bp->regs = ioremap(regs->start, regs->end - regs->start + 1);
	if (!bp->regs) {
		dev_err(&pdev->dev, "failed to map registers, aborting.\n");
		err = -ENOMEM;
		goto err_out_free_dev;
	}

	/* Disable all interrupts */
	macb_writel(bp, IDR, ~0UL);
	macb_readl(bp, ISR);
	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "no irq resource defined.\n");
		goto err_out_iounmap;
	}
	err = request_irq(dev->irq, macb_interrupt, IRQF_SHARED,
			dev->name, dev);
	if (err) {
		printk(KERN_ERR "%s: Unable to request IRQ %d (error %d)\n",
				dev->name, dev->irq, err);
		goto err_out_iounmap;
	}

	dev->netdev_ops = &macb_netdev_ops;
	netif_napi_add(dev, &bp->napi, macb_poll, 64);
	dev->ethtool_ops = &macb_ethtool_ops;
	dev->base_addr = regs->start;
	macb_get_hwaddr(bp);

	if (pdata && pdata->is_rmii)
		macb_writel(bp, USRIO, 0);
	else
		macb_writel(bp, USRIO, MACB_BIT(MII));

	if (pdata->gpio_num >= 0) {
		err = gpio_request(pdata->gpio_num, "spr_phy_rst_gpio");
		if (err < 0)
			goto err_out_free_irq;

		err = gpio_direction_output(pdata->gpio_num , 0);
		if (err < 0)
			goto err_out_free_gpio;

		gpio_set_value(pdata->gpio_num, 1);
	}

	bp->tx_pending = DEF_TX_RING_PENDING;
	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Cannot register net device, aborting.\n");
		goto err_out_free_gpio;
	}

	err = spear_init_hw_timer(bp);
	if (err) {
		dev_dbg(&bp->pdev->dev, "Failed to initialize the hw timer.\n");
		goto err_out_unregister_netdev;
	}

	bp->time_active = 0;
	if (macb_mii_init(bp) != 0)
		goto err_out_unregister_timer;

	platform_set_drvdata(pdev, dev);
	printk(KERN_INFO "%s: Atmel MACB at 0x%08lx irq %d (%pM)\n",
			dev->name, dev->base_addr, dev->irq, dev->dev_addr);
	phydev = bp->phy_dev;
	printk(KERN_INFO "%s: attached PHY driver [%s] "
		"(mii_bus:phy_addr=%s, irq=%d)\n", dev->name,
		phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	return 0;
err_out_unregister_timer:
	free_irq(bp->hw_tx_timeout->irq, dev);
	bp->timer_ops->timer_free(bp->hw_tx_timeout);
err_out_unregister_netdev:
	unregister_netdev(dev);
err_out_free_gpio:
	gpio_free(pdata->gpio_num);
err_out_free_irq:
	free_irq(dev->irq, dev);
err_out_iounmap:
	iounmap(bp->regs);
err_out_free_dev:
	free_netdev(dev);
err_out:
	platform_set_drvdata(pdev, NULL);
	return err;
}

static int __exit macb_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct macb *bp;
	struct macb_base_data *pdata;

	dev = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	if (dev) {
		bp = netdev_priv(dev);
		if (pdata->gpio_num >= 0)
			gpio_free(pdata->gpio_num);

		free_irq(bp->hw_tx_timeout->irq, dev);
		bp->timer_ops->timer_free(bp->hw_tx_timeout);
		if (bp->phy_dev)
			phy_disconnect(bp->phy_dev);

		mdiobus_unregister(bp->mii_bus);
		kfree(bp->mii_bus->irq);
		mdiobus_free(bp->mii_bus);
		unregister_netdev(dev);
		free_irq(dev->irq, dev);
		iounmap(bp->regs);
		free_netdev(dev);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM
static int macb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *netdev = platform_get_drvdata(pdev);

	netif_device_detach(netdev);
	return 0;
}

static int macb_resume(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);

	netif_device_attach(netdev);

	return 0;
}
#else
#define macb_suspend	NULL
#define macb_resume	NULL
#endif

static struct platform_driver macb_driver = {
	.remove		= __exit_p(macb_remove),
	.suspend	= macb_suspend,
	.resume		= macb_resume,
	.driver		= {
		.name		= "macb",
		.owner	= THIS_MODULE,
	},
};

static int __init macb_init(void)
{
	return platform_driver_probe(&macb_driver, macb_probe);
}

static void __exit macb_exit(void)
{
	platform_driver_unregister(&macb_driver);
}

module_init(macb_init);
module_exit(macb_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPEAr3XX MACB Ethernet driver");
MODULE_AUTHOR("Haavard Skinnemoen <hskinnemoen@atmel.com>,\
		Deepak Sikri <deepak.sikri@st.com>");
MODULE_ALIAS("platform:macb");
