/*
* drivers/dma/pl080_dmac_regs.h
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

#ifndef PL080_DMAC_REGS_H
#define PL080_DMAC_REGS_H

#include <linux/pl080_dmac.h>

#define PL080_DMA_MAX_NR_CHANNELS	8

/* size of dma registers */
#define PL080_REGLEN		0x200

/* hardware register definitions. */
struct pl080_dma_chan_regs {
	u32	SAR;		/* Source Address Register */
	u32	DAR;		/* destination address register */
	u32	LLP;		/* linked list pointer */
	u32	CTL;		/* control register */
	u32	CFG;		/* configuration register */
	u32	PAD[3];		/* padding of 12 bytes is required after every
				   channel registers */
};

struct pl080_dma_regs {
	/* irq handling */
	u32	INT_STATUS;	/* interrupt status - tc | error */
	u32	INT_TC_STATUS;	/* interrupt status - tc */
	u32	INT_TC_CLEAR;	/* interrupt clear - tc */
	u32	INT_ERR_STATUS;	/* interrupt status - error */
	u32	INT_ERR_CLEAR;	/* interrupt clear - error */
	u32	INT_TC_RAW;	/* raw interrupt - tc*/
	u32	INT_ERR_RAW;	/* raw interrupt - error */

	u32	CH_ENBLD;	/* enabled channels */

	/* software handshaking */
	u32	BREQ;		/* burst request */
	u32	SREQ;		/* single request */
	u32	LAST_BREQ;	/* last burst request */
	u32	LAST_SREQ;	/* single burst request */

	u32	CFG;		/* dmac configuration */
	u32	SYNC;		/* sync */

	u32	PAD[50];	/* padding of 200 bytes is required between sync
				   register and channel registers */

	/* per-channel registers */
	struct pl080_dma_chan_regs	CHAN[PL080_DMA_MAX_NR_CHANNELS];
};

/* bitfields in dma CFG */
/* here x is endianness, little - 0, big - 1 */
#define	PL080_CHAN_CFG_ENDIAN_M1(x)	((x) << 1)
#define	PL080_CHAN_CFG_ENDIAN_M2(x)	((x) << 2)
#define PL080_CFG_DMA_EN		(1 << 0)

/* bitfields in CHN LLP */
#define	PL080_CHAN_LLP_MASTER2		(1 << 0)
#define	PL080_CHAN_LLP_MASTER1		(0 << 0)

/* bitfields in CHN CTL */
#define PL080_CHAN_CTL_DEST_MASTER(x)		((x) << 25)
#define PL080_CHAN_CTL_SRC_MASTER(x)		((x) << 24)

/* bitfields in CHN CFG */
#define	PL080_CHAN_CFG_ENABLE		(1 << 0)
/* interrupt enable */
#define PL080_CHAN_CFG_INT_TC_ENABLE	(1 << 15)
#define PL080_CHAN_CFG_INT_ERR_ENABLE	(1 << 14)
#define PL080_CHAN_CFG_HALT		(1 << 18)

struct pl080_dma_chan {
	struct dma_chan		chan;
	void __iomem		*ch_regs;
	u8			mask;

	spinlock_t		lock;

	/* these other elements are all protected by lock */
	dma_cookie_t		completed;
	struct list_head	active_list;
	struct list_head	queue;
	struct list_head	free_list;

	unsigned int		descs_allocated;
	unsigned int		memset_value;
};

static inline struct pl080_dma_chan_regs __iomem *
__pl080c_regs(struct pl080_dma_chan *pl080c)
{
	return pl080c->ch_regs;
}

#define channel_readl(pl080c, name) \
	__raw_readl(&(__pl080c_regs(pl080c)->name))
#define channel_writel(pl080c, name, val) \
	__raw_writel((val), &(__pl080c_regs(pl080c)->name))

static inline struct pl080_dma_chan *to_pl080_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct pl080_dma_chan, chan);
}

struct pl080_dma {
	struct dma_device	dma;
	void __iomem		*regs;
	struct clk		*clk;
#ifdef CONFIG_CPU_FREQ
	struct notifier_block notifier_blk;
#endif

	u8			all_chan_mask;

	struct pl080_dma_chan	chan[0];
};

static inline
struct pl080_dma_regs __iomem *__pl080_regs(struct pl080_dma *pl080)
{
	return pl080->regs;
}

#define dma_readl(pl080, name) \
	__raw_readl(&(__pl080_regs(pl080)->name))
#define dma_writel(pl080, name, val) \
	__raw_writel((val), &(__pl080_regs(pl080)->name))

#define channel_set_bit(pl080, reg, mask) \
	dma_writel(pl080, reg, ((mask) << 8) | (mask))
#define channel_clear_bit(pl080, reg, mask) \
	dma_writel(pl080, reg, ((mask) << 8) | 0)

static inline struct pl080_dma *to_pl080_dma(struct dma_device *ddev)
{
	return container_of(ddev, struct pl080_dma, dma);
}

/* lli == linked list item; a.k.a. dma block descriptor */
struct pl080_lli {
	/* values that are not changed by hardware */
	dma_addr_t	sar;
	dma_addr_t	dar;
	dma_addr_t	llp;		/* chain to next lli */
	u32		ctl;
};

struct pl080_desc {
	/* first values the hardware uses */
	struct pl080_lli			lli;

	/* then values for driver housekeeping */
	struct list_head		desc_node;
	struct list_head		tx_list;
	struct dma_async_tx_descriptor	txd;
	size_t				len;
};

static inline struct pl080_desc *
txd_to_pl080_desc(struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct pl080_desc, txd);
}

#endif /* PL080_DMAC_REGS_H */
