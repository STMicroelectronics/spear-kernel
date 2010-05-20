/*
 * drivers/dma/pl080_dmac.c
 *
 * ARM PL080 DMA Controller driver - source file
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include "pl080_dmac_regs.h"

/*
 * This supports the "ARM PL080 Central DMA Controller", (PL080_ahb_dmac) which
 * is used with various AMBA 2.0 systems
 *
 * The driver has currently been tested only with the SPEAr3XX and SPEAr6XX
 */

#define PL080_CHAN_DEFAULT_CFG	(PL080_CHAN_CFG_FLOW_CTRL(DMA_MEMORY_TO_MEMORY))

#define PL080_CHAN_DEFAULT_CTL	(PL080_CHAN_CTL_USER_MODE	\
		| PL080_CHAN_CTL_NON_BUFFERABLE	\
		| PL080_CHAN_CTL_NON_CACHEABLE	\
		| PL080_CHAN_CTL_DEST_BURST(BURST_16)	\
		| PL080_CHAN_CTL_SRC_BURST(BURST_16))

/*
 * Number of descriptors to allocate for each channel. This should be
 * made configurable somehow; preferably, the clients (at least the
 * ones using slave transfers) should be able to give us a hint.
 */
#define NR_DESCS_PER_CHANNEL	1024
#define NR_CHANNEL		8
#define DRIVER_NAME	"pl080_dmac"

/*----------------------------------------------------------------------*/

/*
 * Because we're not relying on writeback from the controller (it may not
 * even be configured into the core!) we don't need to use dma_pool.  These
 * descriptors -- and associated data -- are cacheable.  We do need to make
 * sure their dcache entries are written back before handing them off to
 * the controller, though.
 */

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}
static struct device *chan2parent(struct dma_chan *chan)
{
	return chan->dev->device.parent;
}

static struct pl080_desc *pl080c_first_active(struct pl080_dma_chan *pl080c)
{
	return list_entry(pl080c->active_list.next, struct pl080_desc,
			desc_node);
}

static struct pl080_desc *pl080c_first_queued(struct pl080_dma_chan *pl080c)
{
	return list_entry(pl080c->queue.next, struct pl080_desc, desc_node);
}

/* getting dma descriptor from channel free pool */
static struct pl080_desc *pl080c_desc_get(struct pl080_dma_chan *pl080c)
{
	struct pl080_desc *desc, *_desc;
	struct pl080_desc *ret = NULL;
	unsigned int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&pl080c->lock, flags);
	list_for_each_entry_safe(desc, _desc, &pl080c->free_list, desc_node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del(&desc->desc_node);
			ret = desc;
			break;
		}
		dev_dbg(chan2dev(&pl080c->chan), "desc %p not ACKed\n", desc);
		i++;
	}
	spin_unlock_irqrestore(&pl080c->lock, flags);

	dev_vdbg(chan2dev(&pl080c->chan),
			"scanned %u descriptors on freelist\n", i);

	return ret;
}
#if 0
static void pl080c_sync_desc_for_cpu(struct pl080_dma_chan *pl080c, struct
		pl080_desc * desc)
{
	struct pl080_desc *child;

	list_for_each_entry(child, &desc->tx_list, desc_node)
		dma_sync_single_for_cpu(chan2parent(&pl080c->chan),
				child->txd.phys, sizeof(child->lli),
				DMA_TO_DEVICE);
	dma_sync_single_for_cpu(chan2parent(&pl080c->chan),
			desc->txd.phys, sizeof(desc->lli),
			DMA_TO_DEVICE);
}
#endif

/*
 * move a descriptor, including any children, to the free list.
 * `desc' must not be on any lists.
 */
static void
pl080c_desc_put(struct pl080_dma_chan *pl080c, struct pl080_desc *desc)
{
	unsigned long flags;
	if (desc) {
		struct pl080_desc *child;

#if 0
		/* this may not be required here */
		pl080c_sync_desc_for_cpu(pl080c, desc);
#endif

		spin_lock_irqsave(&pl080c->lock, flags);
		list_for_each_entry(child, &desc->tx_list, desc_node)
			dev_vdbg(chan2dev(&pl080c->chan),
					"moving child desc %p to freelist\n",
					child);
		list_splice_init(&desc->tx_list, &pl080c->free_list);
		dev_vdbg(chan2dev(&pl080c->chan),
				"moving desc %p to freelist\n", desc);
		list_add(&desc->desc_node, &pl080c->free_list);
		spin_unlock_irqrestore(&pl080c->lock, flags);
	}
}

/* called with pl080c->lock held and irqs disabled */
static dma_cookie_t
pl080c_assign_cookie(struct pl080_dma_chan *pl080c, struct pl080_desc *desc)
{
	dma_cookie_t cookie = pl080c->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	pl080c->chan.cookie = cookie;
	desc->txd.cookie = cookie;

	return cookie;
}

/*----------------------------------------------------------------------*/

/* starts xfer, called with pl080c->lock held and irqs disabled */
static void
pl080c_dostart(struct pl080_dma_chan *pl080c, struct pl080_desc *first)
{
	u32 cfg = 0;

	/* assert: channel is idle */
	cfg = channel_readl(pl080c, CFG);
	if (cfg & PL080_CHAN_CFG_ENABLE) {
		dev_err(chan2dev(&pl080c->chan),
				"BUG: Attempted to start non-idle channel\n");
		dev_err(chan2dev(&pl080c->chan), "SAR: 0x%x DAR: 0x%x LLP: 0x%x"
				" CTL: 0x%x CFG:0x%x\n",
				channel_readl(pl080c, SAR),
				channel_readl(pl080c, DAR),
				channel_readl(pl080c, LLP),
				channel_readl(pl080c, CTL),
				channel_readl(pl080c, CFG));

		/* the irq handler will hopefully advance the queue... */
		return;
	}

	/* start actual dma transfers */
	channel_writel(pl080c, SAR, first->lli.sar);
	channel_writel(pl080c, DAR, first->lli.dar);
	channel_writel(pl080c, CTL, first->lli.ctl);
	channel_writel(pl080c, LLP, first->lli.llp);
	channel_writel(pl080c, CFG, cfg | PL080_CHAN_CFG_ENABLE);
}

/*----------------------------------------------------------------------*/

/* called after xfer requested for descriptor is complete */
static void
pl080c_descriptor_complete(struct pl080_dma_chan *pl080c, struct pl080_desc
		*desc)
{
	dma_async_tx_callback		callback;
	void				*param;
	struct dma_async_tx_descriptor	*txd = &desc->txd;

	dev_vdbg(chan2dev(&pl080c->chan), "descriptor %u complete\n",
			txd->cookie);

	pl080c->completed = txd->cookie;
	callback = txd->callback;
	param = txd->callback_param;

#if 0
	/* this may not be required here */
	pl080c_sync_desc_for_cpu(pl080c, desc);
#endif
	list_splice_init(&desc->tx_list, &pl080c->free_list);
	list_move(&desc->desc_node, &pl080c->free_list);

	/*
	 * we use dma_unmap_page() regardless of how the buffers were
	 * mapped before they were submitted...
	 */
	if (!(txd->flags & DMA_COMPL_SKIP_DEST_UNMAP))
		dma_unmap_page(chan2parent(&pl080c->chan), desc->lli.dar,
				desc->len, DMA_FROM_DEVICE);
	if (!(txd->flags & DMA_COMPL_SKIP_SRC_UNMAP))
		dma_unmap_page(chan2parent(&pl080c->chan), desc->lli.sar,
				desc->len, DMA_TO_DEVICE);

	/*
	 * the api requires that no submissions are done from a
	 * callback, so we don't need to drop the lock here
	 */
	if (callback)
		callback(param);
}

/* complete all descriptors for a desired channel */
static void
pl080c_complete_all(struct pl080_dma *pl080, struct pl080_dma_chan *pl080c)
{
	struct pl080_desc *desc, *_desc;
	u32 cfg = 0;
	LIST_HEAD(list);

	cfg = channel_readl(pl080c, CFG);
	if (cfg & PL080_CHAN_CFG_ENABLE) {
		dev_err(chan2dev(&pl080c->chan),
				"BUG: XFER bit set, but channel not idle!\n");

		/* try to continue after resetting the channel... */
		channel_writel(pl080c, CFG, cfg & ~PL080_CHAN_CFG_ENABLE);
	}

	/*
	 * submit queued descriptors asap, i.e. before we go through
	 * the completed ones.
	 */
	if (!list_empty(&pl080c->queue))
		pl080c_dostart(pl080c, pl080c_first_queued(pl080c));
	list_splice_init(&pl080c->active_list, &list);
	list_splice_init(&pl080c->queue, &pl080c->active_list);

	list_for_each_entry_safe(desc, _desc, &list, desc_node)
		pl080c_descriptor_complete(pl080c, desc);
}

/*
 * check channel and descriptors status. if channel is free then schedule next
 * xfer
 */
static void
pl080c_scan_descriptors(struct pl080_dma *pl080, struct pl080_dma_chan *pl080c)
{
	dma_addr_t llp;
	struct pl080_desc *desc, *_desc;
	struct pl080_desc *child;
	u32 status_xfer;
	u32 cfg = 0;

	/*
	 * clear block interrupt flag before scanning so that we don't
	 * miss any, and read llp before raw_xfer to ensure it is
	 * valid if we decide to scan the list.
	 */
	llp = channel_readl(pl080c, LLP);
	status_xfer = dma_readl(pl080, INT_STATUS);

	if (status_xfer & pl080c->mask) {
		if (dma_readl(pl080, INT_ERR_STATUS) & pl080c->mask)
			dev_err(chan2dev(&pl080c->chan),
					"BUG: Transfer failed for channel id %d"
					"\n", pl080c->chan.chan_id);

		/* everything we've submitted is done */
		dma_writel(pl080, INT_TC_CLEAR, pl080c->mask);
		dma_writel(pl080, INT_ERR_CLEAR, pl080c->mask);
		pl080c_complete_all(pl080, pl080c);
		return;
	}

	dev_vdbg(chan2dev(&pl080c->chan), "scan_descriptors: llp=0x%x\n", llp);

	list_for_each_entry_safe(desc, _desc, &pl080c->active_list, desc_node) {
		if (desc->lli.llp == llp)
			/* this one is currently in progress */
			return;

		list_for_each_entry(child, &desc->tx_list, desc_node)
			if (child->lli.llp == llp)
				/* currently in progress */
				return;

		/*
		 * no descriptors so far seem to be in progress, i.e.
		 * this one must be done.
		 */
		pl080c_descriptor_complete(pl080c, desc);
	}

	dev_err(chan2dev(&pl080c->chan),
			"BUG: All descriptors done, but channel not idle!\n");

	/* try to continue after resetting the channel... */
	cfg = channel_readl(pl080c, CFG);
	channel_writel(pl080c, CFG, cfg & ~PL080_CHAN_CFG_ENABLE);

	if (!list_empty(&pl080c->queue)) {
		pl080c_dostart(pl080c, pl080c_first_queued(pl080c));
		list_splice_init(&pl080c->queue, &pl080c->active_list);
	}
}

static void
pl080c_dump_lli(struct pl080_dma_chan *pl080c, struct pl080_lli *lli)
{
	dev_printk(KERN_CRIT, chan2dev(&pl080c->chan),
			" desc: s0x%x d0x%x l0x%x c0x%x\n",
			lli->sar, lli->dar, lli->llp,
			lli->ctl);
}

static void
pl080c_handle_error(struct pl080_dma *pl080, struct pl080_dma_chan *pl080c)
{
	struct pl080_desc *bad_desc;
	struct pl080_desc *child;

	/*
	 * the descriptor currently at the head of the active list is
	 * borked. since we don't have any way to report errors, we'll
	 * just have to scream loudly and try to carry on.
	 */
	bad_desc = pl080c_first_active(pl080c);
	list_del_init(&bad_desc->desc_node);
	list_splice_init(&pl080c->queue, pl080c->active_list.prev);

	/* clear the error flag and try to restart the controller */
	dma_writel(pl080, INT_ERR_CLEAR, pl080c->mask);
	if (!list_empty(&pl080c->active_list))
		pl080c_dostart(pl080c, pl080c_first_active(pl080c));

	/*
	 * kern_critical may seem harsh, but since this only happens
	 * when someone submits a bad physical address in a
	 * descriptor, we should consider ourselves lucky that the
	 * controller flagged an error instead of scribbling over
	 * random memory locations.
	 */
	dev_printk(KERN_CRIT, chan2dev(&pl080c->chan),
			"Bad descriptor submitted for DMA!\n");
	dev_printk(KERN_CRIT, chan2dev(&pl080c->chan),
			" cookie: %d\n", bad_desc->txd.cookie);
	pl080c_dump_lli(pl080c, &bad_desc->lli);
	list_for_each_entry(child, &bad_desc->tx_list, desc_node)
		pl080c_dump_lli(pl080c, &child->lli);

	/* pretend the descriptor completed successfully */
	pl080c_descriptor_complete(pl080c, bad_desc);
}

/* dma interrupt handler */
static irqreturn_t pl080_dma_interrupt(int irq, void *dev_id)
{
	struct pl080_dma *pl080 = (struct pl080_dma *)dev_id;
	struct pl080_dma_chan *pl080c;
	u32 status_xfer;
	u32 status_err;
	int i;

	/* read interrupt status */
	status_xfer = dma_readl(pl080, INT_TC_STATUS);
	status_err = dma_readl(pl080, INT_ERR_STATUS);

	dev_vdbg(pl080->dma.dev, "irq handler: status_xfer=%x status_err=%x\n",
			status_xfer, status_err);

	/* call appropriate fn for error or success */
	/* iterate this loop for chancnt channels, till a xfer or err interrupt
	 ** processing is pending */
	for (i = 0; (i < pl080->dma.chancnt) && (status_err || status_xfer);
			i++) {
		pl080c = &pl080->chan[i];
		if (status_err & (1 << i)) {
			status_err &= ~(1 << i);
			pl080c_handle_error(pl080, pl080c);
		} else if (status_xfer & (1 << i)) {
			status_xfer &= ~(1 << i);
			pl080c_scan_descriptors(pl080, pl080c);
		}
	}

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------*/

/*
 * add descriptor prepared by prep_* fns to the channel queue or start xfer if
 * queue is empty
 */
static dma_cookie_t pl080_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct pl080_desc		*desc = txd_to_pl080_desc(tx);
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(tx->chan);
	dma_cookie_t		cookie;
	unsigned long flags;

	spin_lock_irqsave(&pl080c->lock, flags);
	cookie = pl080c_assign_cookie(pl080c, desc);

	/*
	 * revisit: we should attempt to chain as many descriptors as
	 * possible, perhaps even appending to those already submitted
	 * for dma. but this is hard to do in a race-free manner.
	 */
	if (list_empty(&pl080c->active_list)) {
		dev_vdbg(chan2dev(tx->chan), "tx_submit: started %u\n",
				desc->txd.cookie);
		pl080c_dostart(pl080c, desc);
		list_add_tail(&desc->desc_node, &pl080c->active_list);
	} else {
		dev_vdbg(chan2dev(tx->chan), "tx_submit: queued %u\n",
				desc->txd.cookie);

		list_add_tail(&desc->desc_node, &pl080c->queue);
	}

	spin_unlock_irqrestore(&pl080c->lock, flags);

	return cookie;
}
/**
 * pl080_prep_dma_memset - prepares a memset operation
 * @chan: structure representing dma channel to be used for this operation
 * @dest: address specifying memory to be initialized
 * @value: value to be set at dest address
 * @len: number of bytes to be set
 * @flags: flags indicating certain dma behaviour
 *
 * This function initializes a range of addresses specified by dest field with a
 * value specified by value field(converted to a unsigned char). It can be used
 * instead of software memset function for better performance
 * This function returns pointer to async transaction descriptor
 */
static struct dma_async_tx_descriptor *
pl080_prep_dma_memset(struct dma_chan *chan, dma_addr_t dest, int value, size_t
		len, unsigned long flags)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_dma_slave	*pl080s = chan->private;
	struct pl080_desc		*desc, *first, *prev;
	size_t			xfer_count, offset;
	unsigned int		width;
	u32			ctl, x;
	int src	= __pa(&pl080c->memset_value);

	/* copying lower byte to all four bytes of integer */
	value = value & 0xFF;
	value |= value << 8;
	value |= value << 16;

	pl080c->memset_value = value;
	dma_map_single(chan2parent(chan), &pl080c->memset_value,
			sizeof(pl080c->memset_value), DMA_TO_DEVICE);

	dev_vdbg(chan2dev(chan), "prep_dma_memset d0x%x s0x%d l0x%zx f0x%lx\n",
			dest, value, len, flags);

	if (unlikely(!len)) {
		dev_dbg(chan2dev(chan), "prep_dma_memset: length is zero!\n");
		return NULL;
	}

	/*
	 * we can be a lot more clever here, but this should take care
	 * of the most common optimization.
	 */
	x = (src | dest | len);
	width = (x & 0x1) ? WIDTH_BYTE :
		(x & 0x2) ? WIDTH_HALFWORD : WIDTH_WORD;

	/* check if user wants to configure with his own configurations. for
	 ** configuring with user configuration pl080s should not be null. */
	if (!pl080s) {
		ctl = PL080_CHAN_DEFAULT_CTL | PL080_CHAN_CTL_DEST_WIDTH(width)
			| PL080_CHAN_CTL_SRC_WIDTH(width) |
			PL080_CHAN_CTL_DEST_ADDR_INC;

	} else {
		ctl = pl080s->ctl;

		/* if master passed from user is none then configure ahb0 */
		ctl |= PL080_CHAN_CTL_SRC_MASTER(pl080s->src_master !=
				MASTER_NONE ? pl080s->src_master : MASTER_AHB0);
		ctl |= PL080_CHAN_CTL_DEST_MASTER(pl080s->dest_master !=
				MASTER_NONE ? pl080s->dest_master :
				MASTER_AHB0);

		/* if width configured is not possible due to non word
		 ** allignment of src or dest address */
		if (PL080_CHAN_CTL_SRC_WIDTH(width) <
				(ctl & PL080_CHAN_CTL_SRC_WIDTH_MASK)) {
			ctl &= ~PL080_CHAN_CTL_SRC_WIDTH_MASK;
			ctl |= PL080_CHAN_CTL_SRC_WIDTH(width);
		}

		if (PL080_CHAN_CTL_DEST_WIDTH(width) <
				(ctl & PL080_CHAN_CTL_DEST_WIDTH_MASK)) {
			ctl &= ~PL080_CHAN_CTL_DEST_WIDTH_MASK;
			ctl |= PL080_CHAN_CTL_DEST_WIDTH(width);
		}

		width = PL080_CHAN_CTL_GET_SRC_WIDTH(ctl);
	}

	prev = first = NULL;

	/* make lli list for xfer */
	for (offset = 0; offset < len; offset += xfer_count << width) {
		xfer_count = min_t(size_t, (len - offset) >> width,
				PL080_CHAN_MAX_COUNT);

		desc = pl080c_desc_get(pl080c);
		if (!desc)
			goto err_desc_get;

		desc->lli.sar = src;
		desc->lli.dar = dest + offset;
		desc->lli.ctl = ctl | xfer_count;

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->txd.phys;
			dma_sync_single_for_device(chan2parent(chan),
					prev->txd.phys, sizeof(prev->lli),
					DMA_TO_DEVICE);
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
	}

	/* trigger interrupt after last block */
	prev->lli.ctl |= PL080_CHAN_CTL_INT_LLI;

	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan),
			prev->txd.phys, sizeof(prev->lli),
			DMA_TO_DEVICE);

	first->txd.flags = flags | DMA_CTRL_ACK;
	first->len = len;

	return &first->txd;

err_desc_get:
	pl080c_desc_put(pl080c, first);
	return NULL;
}

/**
 * pl080_prep_dma_memcpy - prepares a memcpy operation
 * @chan: structure representing dma channel to be used for this operation
 * @dest: address specifying destination memory
 * @src: address specifying source memory
 * @len: number of bytes to be copied
 * @flags: flags indicating certain dma behaviour
 *
 * This function copies len amount of bytes from src to dest address
 * It can be used instead of software memcpy function for better performance
 * This function returns pointer to async transaction descriptor
 */
static struct dma_async_tx_descriptor *
pl080_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_dma_slave	*pl080s = chan->private;
	struct pl080_desc		*desc, *first, *prev;
	size_t			xfer_count, offset;
	unsigned int		width;
	u32			ctl, cfg, x;

	dev_vdbg(chan2dev(chan), "prep_dma_memcpy d0x%x s0x%x l0x%zx f0x%lx\n",
			dest, src, len, flags);

	if (unlikely(!len)) {
		dev_dbg(chan2dev(chan), "prep_dma_memcpy: length is zero!\n");
		return NULL;
	}

	/*
	 * we can be a lot more clever here, but this should take care
	 * of the most common optimization.
	 */
	x = (src | dest | len);
	width = (x & 0x1) ? WIDTH_BYTE :
		(x & 0x2) ? WIDTH_HALFWORD : WIDTH_WORD;

	/* check if user wants to configure with his own configurations. for
	 ** configuring with user configuration pl080s should not be null. */
	if (!pl080s) {
		ctl = PL080_CHAN_DEFAULT_CTL | PL080_CHAN_CTL_DEST_WIDTH(width)
			| PL080_CHAN_CTL_SRC_WIDTH(width) |
			PL080_CHAN_CTL_DEST_ADDR_INC |
			PL080_CHAN_CTL_SRC_ADDR_INC;
	} else {
		ctl = pl080s->ctl;

		/* if master passed from user is none then configure ahb0 */
		ctl |= PL080_CHAN_CTL_SRC_MASTER(pl080s->src_master !=
				MASTER_NONE ? pl080s->src_master : MASTER_AHB0);
		ctl |= PL080_CHAN_CTL_DEST_MASTER(pl080s->dest_master !=
				MASTER_NONE ? pl080s->dest_master :
				MASTER_AHB0);

		/* if width configured is not possible due to non word
		 ** allignment of src or dest address. */
		if (PL080_CHAN_CTL_SRC_WIDTH(width) <
				(ctl & PL080_CHAN_CTL_SRC_WIDTH_MASK)) {
			ctl &= ~PL080_CHAN_CTL_SRC_WIDTH_MASK;
			ctl |= PL080_CHAN_CTL_SRC_WIDTH(width);
		}

		if (PL080_CHAN_CTL_DEST_WIDTH(width) <
				(ctl & PL080_CHAN_CTL_DEST_WIDTH_MASK)) {
			ctl &= ~PL080_CHAN_CTL_DEST_WIDTH_MASK;
			ctl |= PL080_CHAN_CTL_DEST_WIDTH(width);
		}

		width = PL080_CHAN_CTL_GET_SRC_WIDTH(ctl);

		/*
		 * we need controller-specific data to set up slave
		 * transfers.
		 */
		cfg = pl080s->cfg | PL080_CHAN_CFG_INT_TC_ENABLE |
			PL080_CHAN_CFG_INT_ERR_ENABLE;
		channel_writel(pl080c, CFG, cfg);
	}

	prev = first = NULL;

	/* make lli list for xfer */
	for (offset = 0; offset < len; offset += xfer_count << width) {
		xfer_count = min_t(size_t, (len - offset) >> width,
				PL080_CHAN_MAX_COUNT);

		desc = pl080c_desc_get(pl080c);
		if (!desc)
			goto err_desc_get;

		desc->lli.sar = src + offset;
		desc->lli.dar = dest + offset;
		desc->lli.ctl = ctl | xfer_count;

		if (!first) {
			first = desc;
		} else {
			prev->lli.llp = desc->txd.phys;
			dma_sync_single_for_device(chan2parent(chan),
					prev->txd.phys, sizeof(prev->lli),
					DMA_TO_DEVICE);
			list_add_tail(&desc->desc_node,
					&first->tx_list);
		}
		prev = desc;
	}

	/* trigger interrupt after last block */
	prev->lli.ctl |= PL080_CHAN_CTL_INT_LLI;

	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan),
			prev->txd.phys, sizeof(prev->lli),
			DMA_TO_DEVICE);

	first->txd.flags = flags | DMA_CTRL_ACK;
	first->len = len;

	return &first->txd;

err_desc_get:
	pl080c_desc_put(pl080c, first);
	return NULL;
}

/**
 * pl080_prep_slave_sg - prepares a slave dma operation
 * @chan: structure representing dma channel to be used for this operation
 * @sgl: pointer to scatter list describing dma transfer
 * @sg_len: number of scatterlist nodes in sgl
 * @direction: direction of transfer, DMA_TO_DEVICE or DMA_FROM_DEVICE
 * @flags: flags indicating certain dma behaviour
 *
 * This function xfers data between address specified by scatter list and rx or
 * tx register of slave peripheral acquiring the dma channel specified.
 * This function returns pointer to async transaction descriptor
 */
static struct dma_async_tx_descriptor *
pl080_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_data_direction direction,
		unsigned long flags)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_dma_slave	*pl080s = chan->private;
	struct pl080_desc		*prev, *first;
	unsigned int		reg_width, i;
	struct scatterlist	*sg;
	dma_addr_t		reg;
	size_t			total_len = 0;
	u32			ctl = 0, cfg = 0;

	dev_vdbg(chan2dev(chan), "prep_dma_slave\n");

	if (unlikely(!pl080s || !sg_len))
		return NULL;

	reg_width = pl080s->reg_width;
	prev = first = NULL;

	/*
	 * we need controller-specific data to set up slave
	 * transfers.
	 */
	cfg = pl080s->cfg | PL080_CHAN_CFG_INT_TC_ENABLE |
		PL080_CHAN_CFG_INT_ERR_ENABLE;
	channel_writel(pl080c, CFG, cfg);

	sg_len = dma_map_sg(chan2parent(chan), sgl, sg_len, direction);

	/* if master passed from user is none then configure ahb0 */
	ctl |= PL080_CHAN_CTL_SRC_MASTER(pl080s->src_master != MASTER_NONE ?
			pl080s->src_master : MASTER_AHB0);
	ctl |= PL080_CHAN_CTL_DEST_MASTER(pl080s->dest_master !=
			MASTER_NONE ? pl080s->dest_master :
			MASTER_AHB0);
	switch (direction) {
	case DMA_TO_DEVICE:
		ctl |= pl080s->ctl | PL080_CHAN_CTL_DEST_WIDTH(reg_width);

		reg = pl080s->tx_reg;
		for_each_sg(sgl, sg, sg_len, i) {
			struct pl080_desc	*desc;
			u32 len, mem, mask;

			desc = pl080c_desc_get(pl080c);
			if (!desc) {
				dev_err(chan2dev(chan), "not enough descriptors"
						" available\n");
				goto err_desc_get;
			}

			mem = sg_phys(sg);
			len = sg_dma_len(sg);
			mask = PL080_CHAN_CTL_GET_SRC_WIDTH(ctl);

			desc->lli.sar = mem;
			desc->lli.dar = reg;
			if (len & 3) {
				ctl &= ~PL080_CHAN_CTL_DEST_WIDTH_MASK;
				ctl |= PL080_CHAN_CTL_DEST_WIDTH(WIDTH_BYTE);
				ctl &= ~PL080_CHAN_CTL_SRC_WIDTH_MASK;
				ctl |= PL080_CHAN_CTL_SRC_WIDTH(WIDTH_BYTE);
				desc->lli.ctl = ctl |
					(len & PL080_CHAN_MAX_COUNT);
			} else
				desc->lli.ctl = ctl | ((len >> mask) &
						PL080_CHAN_MAX_COUNT);

			if (!first) {
				first = desc;
			} else {
				prev->lli.llp = desc->txd.phys;
				dma_sync_single_for_device(chan2parent(chan),
						prev->txd.phys,
						sizeof(prev->lli),
						DMA_TO_DEVICE);
				list_add_tail(&desc->desc_node,
						&first->tx_list);
			}
			prev = desc;
			total_len += len;
		}
		break;
	case DMA_FROM_DEVICE:
		ctl |= pl080s->ctl | PL080_CHAN_CTL_SRC_WIDTH(reg_width);

		reg = pl080s->rx_reg;
		for_each_sg(sgl, sg, sg_len, i) {
			struct pl080_desc	*desc;
			u32 len, mem, mask;

			desc = pl080c_desc_get(pl080c);
			if (!desc) {
				dev_err(chan2dev(chan), "not enough descriptors"
						" available\n");
				goto err_desc_get;
			}

			mem = sg_phys(sg);
			len = sg_dma_len(sg);
			mask = PL080_CHAN_CTL_GET_DEST_WIDTH(ctl);

			desc->lli.sar = reg;
			desc->lli.dar = mem;
			if (len & 3) {
				ctl &= ~PL080_CHAN_CTL_DEST_WIDTH_MASK;
				ctl |= PL080_CHAN_CTL_DEST_WIDTH(WIDTH_BYTE);
				desc->lli.ctl = ctl |
					(len & PL080_CHAN_MAX_COUNT);
			} else
				desc->lli.ctl = ctl | ((len >> mask) &
						PL080_CHAN_MAX_COUNT);

			if (!first) {
				first = desc;
			} else {
				prev->lli.llp = desc->txd.phys;
				dma_sync_single_for_device(chan2parent(chan),
						prev->txd.phys,
						sizeof(prev->lli),
						DMA_TO_DEVICE);
				list_add_tail(&desc->desc_node,
						&first->tx_list);
			}
			prev = desc;
			total_len += len;
		}
		break;
	default:
		return NULL;
	}

	/* trigger interrupt after last block */
	prev->lli.ctl |= PL080_CHAN_CTL_INT_LLI;

	prev->lli.llp = 0;
	dma_sync_single_for_device(chan2parent(chan), prev->txd.phys,
			sizeof(prev->lli), DMA_TO_DEVICE);

	first->len = total_len;

	return &first->txd;

err_desc_get:
	pl080c_desc_put(pl080c, first);
	return NULL;
}

/**
 * pl080_control - control pending operations
 * @chan: structure representing dma channel to be used for this operation
 * @cmd: cmd, eg: DMA_TERMINATE_ALL
 * @arg: arguments
 *
 * This function controls existing xfer on a channel.
 * Currently we only support xfer terminat. This will stop all
 * ongoing xfers and data consistency will not be guaranteed for any ongoing
 * xfer
 */
static int pl080_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_desc		*desc, *_desc;
	LIST_HEAD(list);
	unsigned long flags;

	/* Only supports DMA_TERMINATE_ALL */
	if (cmd != DMA_TERMINATE_ALL)
		return -ENXIO;

	/*
	 * this is only called when something went wrong elsewhere, so
	 * we don't really care about the data. just disable the
	 * channel. we still have to poll the channel enable bit due
	 * to ahb/hsb limitations.
	 */
	spin_lock_irqsave(&pl080c->lock, flags);

	channel_writel(pl080c, CFG, 0);

	/* active_list entries will end up before queued entries */
	list_splice_init(&pl080c->queue, &list);
	list_splice_init(&pl080c->active_list, &list);

	spin_unlock_irqrestore(&pl080c->lock, flags);

	/* flush all pending and queued descriptors */
	list_for_each_entry_safe(desc, _desc, &list, desc_node)
		pl080c_descriptor_complete(pl080c, desc);

	return 0;
}

/**
 * pl080_tx_status - returns DMA transaction status
 * @chan: structure representing dma channel to be used for this operation
 * @cookie: transaction identifier to test status of
 * @txstate: dma tx_state pointer
 *
 * This function returns the DMA transaction status on a dma channel. It will
 * return one of the values from enum dma_status.
 */
static enum dma_status pl080_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	dma_cookie_t		last_used;
	dma_cookie_t		last_complete;
	enum dma_status		ret;

	last_complete = pl080c->completed;
	last_used = chan->cookie;

	ret = dma_async_is_complete(cookie, last_complete, last_used);
	if (ret != DMA_SUCCESS) {
		pl080c_scan_descriptors(to_pl080_dma(chan->device), pl080c);

		last_complete = pl080c->completed;
		last_used = chan->cookie;

		ret = dma_async_is_complete(cookie, last_complete, last_used);
	}

	dma_set_tx_state(txstate, last_complete, last_used, 0);

	return ret;
}

/**
 * pl080_issue_pending - push pending transactions to hardware
 * @chan: structure representing dma channel to be used for this operation
 *
 * This function start any pending transaction on a dma channel
 */
static void pl080_issue_pending(struct dma_chan *chan)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&pl080c->lock, flags);
	if (!list_empty(&pl080c->queue))
		pl080c_scan_descriptors(to_pl080_dma(chan->device), pl080c);
	spin_unlock_irqrestore(&pl080c->lock, flags);
}

/**
 * pl080_alloc_chan_resources - allocate resources and return the
 *	number of allocated descriptors
 * @chan: structure representing dma channel to be used for this operation
 * @client: pointer to clien data structure
 *
 * This function is called by the dma kernel framework whenever a client asks
 * for some channels.
 * It will return number of allocated descriptors
 */
static int pl080_alloc_chan_resources(struct dma_chan *chan)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_desc		*desc;
	struct pl080_dma_slave	*pl080s = chan->private;
	int			i;
	u32			cfg;
	unsigned long flags;

	dev_vdbg(chan2dev(chan), "alloc_chan_resources\n");

	/* assert: channel is idle */
	if (channel_readl(pl080c, CFG) & PL080_CHAN_CFG_ENABLE) {
		dev_dbg(chan2dev(chan), "DMA channel not idle?\n");
		return -EIO;
	}

	pl080c->completed = chan->cookie = 1;

	if (!pl080s) {
		cfg = PL080_CHAN_DEFAULT_CFG;
		cfg = cfg | PL080_CHAN_CFG_INT_TC_ENABLE |
			PL080_CHAN_CFG_INT_ERR_ENABLE;
		channel_writel(pl080c, CFG, cfg);
	}

	/*
	 * note: some controllers may have additional features that we
	 * need to initialize here, like "scatter-gather" (which
	 * doesn't mean what you think it means), and status writeback.
	 */

	spin_lock_irqsave(&pl080c->lock, flags);
	i = pl080c->descs_allocated;
	while (pl080c->descs_allocated < NR_DESCS_PER_CHANNEL) {
		spin_unlock_irqrestore(&pl080c->lock, flags);

		desc = kzalloc(sizeof(struct pl080_desc), GFP_KERNEL);
		if (!desc) {
			dev_info(chan2dev(chan),
					"only allocated %d descriptors\n", i);
			spin_lock_irqsave(&pl080c->lock, flags);
			break;
		}

		INIT_LIST_HEAD(&desc->tx_list);
		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.tx_submit = pl080_tx_submit;
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.phys = dma_map_single(chan2parent(chan), &desc->lli,
				sizeof(desc->lli), DMA_TO_DEVICE);
		pl080c_desc_put(pl080c, desc);

		spin_lock_irqsave(&pl080c->lock, flags);
		i = ++pl080c->descs_allocated;
	}

	spin_unlock_irqrestore(&pl080c->lock, flags);

	dev_dbg(chan2dev(chan),
			"alloc_chan_resources allocated %d descriptors\n", i);

	return i;
}

/**
 * pl080_free_chan_resources - release DMA channel's resources
 * @chan: structure representing dma channel to be used for this operation
 *
 * This function releases previously acquired DMA channels. It is also called
 * from DMA kernel framework
 */
static void pl080_free_chan_resources(struct dma_chan *chan)
{
	struct pl080_dma_chan	*pl080c = to_pl080_dma_chan(chan);
	struct pl080_desc		*desc, *_desc;
	LIST_HEAD(list);
	unsigned long flags;

	dev_dbg(chan2dev(chan), "free_chan_resources (descs allocated=%u)\n",
			pl080c->descs_allocated);

	channel_writel(pl080c, CFG, 0);

	/* assert: channel is idle */
	BUG_ON(!list_empty(&pl080c->active_list));
	BUG_ON(!list_empty(&pl080c->queue));
	BUG_ON(channel_readl(pl080c, CFG) & PL080_CHAN_CFG_ENABLE);
	spin_lock_irqsave(&pl080c->lock, flags);
	list_splice_init(&pl080c->free_list, &list);
	pl080c->descs_allocated = 0;

	spin_unlock_irqrestore(&pl080c->lock, flags);

	list_for_each_entry_safe(desc, _desc, &list, desc_node) {
		dev_vdbg(chan2dev(chan), "freeing descriptor %p\n", desc);
		dma_unmap_single(chan2parent(chan), desc->txd.phys,
				sizeof(desc->lli), DMA_TO_DEVICE);
		kfree(desc);
	}

	dev_vdbg(chan2dev(chan), "free_chan_resources done\n");
}

#ifdef CONFIG_CPU_FREQ
/*----------------------------------------------------------------------*/

/* halts and resumes ongoing dma xfer - used in cpu freq */
/* halt == TRUE -> halt xfer and halt == FALSE -> resume xfer */
void dma_halt(struct pl080_dma *pl080, int halt)
{
	u32 i = 0, val, enbld = dma_readl(pl080, CH_ENBLD);

	while (enbld) {
		if (enbld & (1 << i)) {
			val = channel_readl(&pl080->chan[i], CFG);
			if (halt)
				val |= PL080_CHAN_CFG_HALT;
			else
				val &= ~PL080_CHAN_CFG_HALT;

			channel_writel(&pl080->chan[i], CFG, val);
			enbld &= ~(1 << i);
		}
		i++;
	}
}

static int
dmac_notifier(struct notifier_block *self, unsigned long phase, void *p)
{
	struct pl080_dma *pl080 = container_of(self, struct pl080_dma,
			notifier_blk);

	switch (phase) {
	case CPUFREQ_PRECHANGE:
		dma_halt(pl080, 1);
		return NOTIFY_OK;

	case CPUFREQ_RESUMECHANGE:
	case CPUFREQ_POSTCHANGE:
		dma_halt(pl080, 0);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}
#endif

static int __init pl080_dma_probe(struct platform_device *pdev)
{
	struct resource		*io;
	struct pl080_dma	*pl080;
	size_t			size;
	int			irq;
	int			err;
	int			i;

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	size = sizeof(struct pl080_dma);
	size += NR_CHANNEL * sizeof(struct pl080_dma_chan);
	pl080 = kzalloc(size, GFP_KERNEL);
	if (!pl080)
		return -ENOMEM;

	if (!request_mem_region(io->start, io->end - io->start + 1,
				pdev->dev.driver->name)) {
		err = -EBUSY;
		goto err_kfree;
	}

	pl080->regs = ioremap(io->start, io->end - io->start + 1);
	if (!pl080->regs) {
		err = -ENOMEM;
		goto err_release_r;
	}

	pl080->clk = clk_get(&pdev->dev, "dma_fclk");
	if (IS_ERR(pl080->clk)) {
		err = PTR_ERR(pl080->clk);
		goto err_clk;
	}
	clk_enable(pl080->clk);

	/* force dma off, just in case */
	dma_writel(pl080, CFG, 0);

	err = request_irq(irq, pl080_dma_interrupt, 0, DRIVER_NAME, pl080);
	if (err)
		goto err_irq;

	platform_set_drvdata(pdev, pl080);

	pl080->all_chan_mask = (1 << NR_CHANNEL) - 1;

	INIT_LIST_HEAD(&pl080->dma.channels);
	for (i = 0; i < NR_CHANNEL; i++, pl080->dma.chancnt++) {
		struct pl080_dma_chan	*pl080c = &pl080->chan[i];

		pl080c->chan.device = &pl080->dma;
		pl080c->chan.cookie = pl080c->completed = 1;
		pl080c->chan.chan_id = i;
		list_add_tail(&pl080c->chan.device_node, &pl080->dma.channels);

		pl080c->ch_regs = &__pl080_regs(pl080)->CHAN[i];
		spin_lock_init(&pl080c->lock);
		pl080c->mask = 1 << i;

		INIT_LIST_HEAD(&pl080c->active_list);
		INIT_LIST_HEAD(&pl080c->queue);
		INIT_LIST_HEAD(&pl080c->free_list);
	}

	/* clear/disable all interrupts on all channels. */
	dma_writel(pl080, INT_TC_CLEAR, pl080->all_chan_mask);
	dma_writel(pl080, INT_ERR_CLEAR, pl080->all_chan_mask);

	dma_cap_set(DMA_MEMCPY, pl080->dma.cap_mask);
	dma_cap_set(DMA_MEMSET, pl080->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, pl080->dma.cap_mask);
	pl080->dma.dev = &pdev->dev;

	pl080->dma.device_alloc_chan_resources = pl080_alloc_chan_resources;
	pl080->dma.device_free_chan_resources = pl080_free_chan_resources;
	pl080->dma.device_prep_dma_memcpy = pl080_prep_dma_memcpy;
	pl080->dma.device_prep_dma_memset = pl080_prep_dma_memset;
	pl080->dma.device_prep_slave_sg = pl080_prep_slave_sg;
	pl080->dma.device_control = pl080_control;
	pl080->dma.device_tx_status = pl080_tx_status;
	pl080->dma.device_issue_pending = pl080_issue_pending;

	dma_writel(pl080, CFG, PL080_CFG_DMA_EN | PL080_CHAN_CFG_ENDIAN_M1(0) |
			PL080_CHAN_CFG_ENDIAN_M2(0));

	dev_info(&pdev->dev, "%s: %d channels\n",
			dev_name(&pdev->dev), pl080->dma.chancnt);

	dma_async_device_register(&pl080->dma);

#ifdef CONFIG_CPU_FREQ
	pl080->notifier_blk.notifier_call = &dmac_notifier;
	pl080->notifier_blk.next = NULL;
	pl080->notifier_blk.priority = 0;
	cpufreq_register_notifier(&pl080->notifier_blk,
			CPUFREQ_TRANSITION_NOTIFIER);
#endif
	return 0;

err_irq:
	clk_disable(pl080->clk);
	clk_put(pl080->clk);
err_clk:
	iounmap(pl080->regs);
	pl080->regs = NULL;
err_release_r:
	release_resource(io);
err_kfree:
	kfree(pl080);
	return err;
}

static int __exit pl080_dma_remove(struct platform_device *pdev)
{
	struct pl080_dma		*pl080 = platform_get_drvdata(pdev);
	struct pl080_dma_chan	*pl080c, *_pl080c;
	struct resource		*io;

	dma_writel(pl080, CFG, 0);
	dma_async_device_unregister(&pl080->dma);

	free_irq(platform_get_irq(pdev, 0), pl080);

	list_for_each_entry_safe(pl080c, _pl080c, &pl080->dma.channels,
			chan.device_node) {
		list_del(&pl080c->chan.device_node);
		channel_clear_bit(pl080, CFG, 0);
	}

	clk_disable(pl080->clk);
	clk_put(pl080->clk);

	iounmap(pl080->regs);
	pl080->regs = NULL;

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(io->start, PL080_REGLEN);

#ifdef CONFIG_CPU_FREQ
	cpufreq_unregister_notifier(&pl080->notifier_blk,
			CPUFREQ_TRANSITION_NOTIFIER);
#endif

	kfree(pl080);
	return 0;
}

static void pl080_dma_shutdown(struct platform_device *pdev)
{
	struct pl080_dma *pl080 = platform_get_drvdata(pdev);

	dma_writel(pl080, CFG, 0);
	clk_disable(pl080->clk);
}

static int pl080_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pl080_dma *pl080 = platform_get_drvdata(pdev);

	dma_writel(pl080, CFG, 0);
	clk_disable(pl080->clk);
	return 0;
}

static int pl080_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pl080_dma *pl080 = platform_get_drvdata(pdev);

	clk_enable(pl080->clk);
	dma_writel(pl080, CFG, PL080_CFG_DMA_EN | PL080_CHAN_CFG_ENDIAN_M1(0) |
			PL080_CHAN_CFG_ENDIAN_M2(0));
	return 0;
}

static const struct dev_pm_ops pl080_dev_pm_ops = {
	.suspend_noirq = pl080_suspend_noirq,
	.resume_noirq = pl080_resume_noirq,
};

static struct platform_driver pl080_dma_driver = {
	.remove		= __exit_p(pl080_dma_remove),
	.shutdown	= pl080_dma_shutdown,
	.driver = {
		.name	= DRIVER_NAME,
		.pm	= &pl080_dev_pm_ops,
	},
};

static int __init pl080_init(void)
{
	return platform_driver_probe(&pl080_dma_driver, pl080_dma_probe);
}
subsys_initcall(pl080_init);

static void __exit pl080_exit(void)
{
	platform_driver_unregister(&pl080_dma_driver);
}
module_exit(pl080_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PL080 DMA Controller driver");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@st.com>");
