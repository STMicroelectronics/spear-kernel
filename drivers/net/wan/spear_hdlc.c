/*
 * drivers/net/wan/spear_hdlc.c
 *
 * HDLC driver for TDM/E1 and RS485 HDLC
 * controllers in SPEAr SoC
 *
 * Copyright (C) 2010 ST Microelectronics
 * Frank Shi <frank.shi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/hdlc.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_data/spear_hdlc.h>
#include <linux/pm.h>
#include "spear_hdlc.h"

/* net_device to our structure */
#define dev_to_channel(dev)	((struct channel_t *)dev_to_hdlc(dev)->priv)
#define dev_to_port(dev)	(dev_to_channel(dev)->port)

/* Descriptor number operations */
#define next_desc_num(i)	(((i) + 1) % DESC_PER_RING)
#define prev_desc_num(i)	(((i) + DESC_PER_RING - 1) % DESC_PER_RING)
#define tx_desc_pending(ch)	(((ch)->tx_eoq_pos + DESC_PER_RING - \
					(ch)->tx_cur_pos) % DESC_PER_RING)
#define tx_desc_free(ch)	(((ch)->tx_cur_pos + DESC_PER_RING - \
					(ch)->tx_eoq_pos - 1) % DESC_PER_RING)

#define virt_to_phys(addr)	((u32)(addr) - port->mem_virt + port->mem_phys)

#define MAX_CHANNEL			512

struct port_t;

/* Logic hdlc channel */
struct channel_t {
	struct net_device	*dev;	/* pointer to embedded net_device */
	struct port_t		*port;	/* pointer to physical interface */
	int			id;		/* channel id */

	struct tx_desc		*tx_desc;	/* tx descriptor ring */
	struct rx_desc		*rx_desc;	/* rx descirptor ring */

	int			tx_cur_pos;	/* start of tx queue */
	int			tx_eoq_pos;	/* end of tx queue */
	int			rx_cur_pos;	/* start of rx queue */
	int			rx_frm_pos;	/* start of hdlc frame */

	int			tx_bytes;	/* tx bytes for one frame */
	int			rx_bytes;	/* rx bytes for one frame */

	int			tx_running;	/* tx status */
	int			rx_running;	/* rx status */
};

/* Physcial interface */
struct port_t {
	u32			mem_virt;/* memory pool virtual address */
	dma_addr_t		mem_phys;/* memory pool physical address */

	void __iomem		*reg_base;	/* registers base address */
	/* current position in the interrupt queue */
	int			intq_pos;

	struct ibe		*iba;	/* base address of initial block */
	unsigned short		*intq;	/* base address of interrupt queue */

	/* logic channels on physical interface */
	struct channel_t	*ch[MAX_CHANNEL];

	spinlock_t		lock;	/* spin lock for atomic operation */
	/* tasklet for interrupt processing */
	struct tasklet_struct	int_tasklet;
	struct clk		*clk;

	/* function to build HTCR & HRCR value */
	u32			(*htcr) (struct port_t *);
	u32			(*hrcr) (struct port_t *);

	struct kobject	**tsa_kobjs;	/* kobjects to represent timeslot */
	struct platform_device	*pdev;		/* platform_device reference */

	/* common port parameters */
	int		has_tsa;	/* support timeslot or not */
	int		nr_channel;/* number of logic channels on this port */
	int		nr_timeslot;	/* number of timeslot actually using */
	int		tx_falling_edge;/* tx edge option */
	int		rx_rising_edge;	/* rx edge option */

	/* TDM HDLC port paramters */
	int		ts0_delay;/* delay between SYNC and beginning of TS0 */

	/* RS485 HDLC port paramters */
	int			cts_enable;	/* CSMA enable or not */
	int			cts_delay;	/* CTS delay */
	int			penalty;	/* CSMA penalty significant */

	/* HDLC controller bit field parameters */
	int			max_timeslot;
	int			hdlc_int_type_bit;
	int			hdlc_int_chan_shift;
	int			edge_cfg_bit;
	int			edge_cfg_en_bit;
	int			tsa_init_bit;
	int			tsa_busy_bit;
	int			tsa_read_bit;
	int			tsa_val_shift;
	int			intq_size;
};

/* Bit filed alias */
#define TSA_READ			(port->tsa_read_bit)
#define TSA_BUSY			(port->tsa_busy_bit)
#define TSA_INIT			(port->tsa_init_bit)
#define TSA_TS_MASK			(port->max_timeslot - 1)
#define TSA_VAL_SHIFT			(port->tsa_val_shift)
#define TSA_VAL_MASK			(0xff << TSA_VAL_SHIFT)
#define TSA_CHAN_MASK			(port->max_timeslot - 1)

#define HDLC_INT_CHAN_MASK		((port->max_timeslot - 1) << \
						port->hdlc_int_chan_shift)
#define HDLC_INT_CHAN_SHIFT		(port->hdlc_int_chan_shift)

/* Reg type */
#define RX_TYPE				0
#define TX_TYPE				1

static inline unsigned long
reg_readl(struct port_t *port, unsigned int off)
{
	return readl(port->reg_base + off);
}

static inline void
reg_writel(struct port_t *port, unsigned int off, unsigned long val)
{
	writel(val, port->reg_base + off);
}

/* Check and wait TSA busy signal */
static int wait_tsa_ready(struct port_t *port, int type)
{
	int timeout = 500;

	while (reg_readl(port, TAAR_RX_OFFSET + 0x10 * type) &
			TSA_BUSY) {
		if (timeout-- <= 0) {
			dev_err(&port->pdev->dev, "tsa busy timeout\n");
			return -EBUSY;
		}
	}

	return 0;
}

/* Read TSA memory */
static u32 tsa_read(struct port_t *port, int slot, int type)
{
	u32 offset, tmp;
	int ret;

	offset = 0x10 * type;
	if (slot == 0)
		slot = port->nr_timeslot & (port->max_timeslot - 1);

	ret = wait_tsa_ready(port, type);
	if (ret)
		return ret;

	tmp = reg_readl(port, TAAR_RX_OFFSET + offset);
	tmp &= ~TSA_TS_MASK;
	tmp |= TSA_READ;
	tmp |= (slot & TSA_TS_MASK);

	ret = wait_tsa_ready(port, type);
	if (ret)
		return ret;

	reg_writel(port, TAAR_RX_OFFSET + offset, tmp);

	ret = wait_tsa_ready(port, type);
	if (ret)
		return ret;

	return reg_readl(port, TADR_RX_OFFSET + offset);
}

/* Write TSA memory */
static void tsa_write(struct port_t *port, int slot, int type, u32 val)
{
	u32 offset, tmp;

	offset = 0x10 * type;
	if (slot == 0)
		slot = port->nr_timeslot & (port->max_timeslot - 1);

	wait_tsa_ready(port, type);

	reg_writel(port, TADR_RX_OFFSET + offset, val);

	wait_tsa_ready(port, type);

	tmp = reg_readl(port, TAAR_RX_OFFSET + offset);
	tmp &= ~TSA_TS_MASK;
	tmp &= ~TSA_READ;
	tmp |= (slot & TSA_TS_MASK);

	wait_tsa_ready(port, type);

	reg_writel(port, TAAR_RX_OFFSET + offset, val);

	wait_tsa_ready(port, type);
}

/* Read timeslot bit valid setting */
static ssize_t tsa_bitval_get(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	struct device *dev;
	struct platform_device *pdev;
	struct port_t *port;
	int i, type;
	u32 tsa_val = -1;

	type = (!strcmp(attr->attr.name, "rx_bitval")) ? RX_TYPE : TX_TYPE;

	dev = container_of(kobj->parent, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	port = (struct port_t *)platform_get_drvdata(pdev);

	for (i = 0; i < port->nr_timeslot; i++) {
		if (port->tsa_kobjs[i] == kobj) {
			tsa_val = tsa_read(port, i, type);
			break;
		}
	}

	return sprintf(buf, "0x%x\n", (tsa_val & TSA_VAL_MASK) >>
				TSA_VAL_SHIFT);
}

/* Write timeslot bit valid setting */
static ssize_t tsa_bitval_set(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct device *dev;
	struct platform_device *pdev;
	struct port_t *port;
	int ret, i, type;
	u32 tsa_val;
	ulong val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret)
		return ret;

	type = (!strcmp(attr->attr.name, "rx_bitval")) ? RX_TYPE : TX_TYPE;

	dev = container_of(kobj->parent, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	port = (struct port_t *)platform_get_drvdata(pdev);

	for (i = 0; i < port->nr_timeslot; i++) {
		if (port->tsa_kobjs[i] == kobj) {
			tsa_val = tsa_read(port, i, type);
			tsa_val &= ~TSA_VAL_MASK;
			tsa_val |= (val << TSA_VAL_SHIFT) & TSA_VAL_MASK;
			tsa_write(port, i, type, tsa_val);
			break;
		}
	}

	return count;
}

/* Bit valid attributes */
static struct kobj_attribute tsa_tx_bitval_attr =
	__ATTR(tx_bitval, 0644, tsa_bitval_get, tsa_bitval_set);
static struct kobj_attribute tsa_rx_bitval_attr =
	__ATTR(rx_bitval, 0644, tsa_bitval_get, tsa_bitval_set);

/* Read timeslot assignment */
static ssize_t tsa_chan_get(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	struct device *dev;
	struct platform_device *pdev;
	struct port_t *port;
	int i, type;
	u32 tsa_val = -1;

	type = (!strcmp(attr->attr.name, "rx_chan")) ? RX_TYPE : TX_TYPE;

	dev = container_of(kobj->parent, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	port = (struct port_t *)platform_get_drvdata(pdev);

	for (i = 0; i < port->nr_timeslot; i++) {
		if (port->tsa_kobjs[i] == kobj) {
			tsa_val = tsa_read(port, i, type);
			break;
		}
	}

	return sprintf(buf, "%d\n", tsa_val & TSA_CHAN_MASK);
}

/* Write timeslot assignment */
static ssize_t tsa_chan_set(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	struct device *dev;
	struct platform_device *pdev;
	struct port_t *port;
	int ret, i, type;
	u32 tsa_val;
	ulong val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret)
		return ret;

	type = (!strcmp(attr->attr.name, "rx_chan")) ? RX_TYPE : TX_TYPE;

	dev = container_of(kobj->parent, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	port = (struct port_t *)platform_get_drvdata(pdev);

	for (i = 0; i < port->nr_timeslot; i++) {
		if (port->tsa_kobjs[i] == kobj) {
			tsa_val = tsa_read(port, i, type);
			tsa_val &= ~TSA_CHAN_MASK;
			tsa_val |= val & TSA_CHAN_MASK;
			tsa_write(port, i, type, tsa_val);
			break;
		}
	}

	return count;
}

/* Timeslot channel assignment attribute */
static struct kobj_attribute tsa_tx_chan_attr =
	__ATTR(tx_chan, 0644, tsa_chan_get, tsa_chan_set);
static struct kobj_attribute tsa_rx_chan_attr =
	__ATTR(rx_chan, 0644, tsa_chan_get, tsa_chan_set);

/* TSA kobject attributes */
static struct attribute *tsa_attrs[] = {
	&tsa_tx_bitval_attr.attr,
	&tsa_rx_bitval_attr.attr,
	&tsa_tx_chan_attr.attr,
	&tsa_rx_chan_attr.attr,
	NULL
};

static struct attribute_group tsa_attr_group = {.attrs = tsa_attrs};

/* tdm_hdlc_htcr */
static u32 tdm_hdlc_htcr(struct port_t *port)
{
	u32 cmd = 0;

	if (port->tx_falling_edge)
		cmd |= port->edge_cfg_bit;
	cmd |= port->edge_cfg_en_bit;

	return cmd;
}

/* tdm_hdlc_hrcr */
static u32 tdm_hdlc_hrcr(struct port_t *port)
{
	u32 cmd = 0;

	if (port->rx_rising_edge)
		cmd |= port->edge_cfg_bit;
	cmd |= port->edge_cfg_en_bit;

	return cmd;
}

/* rs485_hdlc_htcr */
static u32 rs485_hdlc_htcr(struct port_t *port)
{
	u32 cmd = 0;

	if (port->tx_falling_edge)
		cmd |= port->edge_cfg_bit;

	if (port->cts_enable) {
		cmd |= CSMA_ENABLE;
		cmd |= port->penalty << PENALTY_SHIFT;
		cmd |= port->cts_delay << CTS_DELAY_SHIFT;
	}

	return cmd;
}

/* rs485_hdlc_hrcr */
static u32 rs485_hdlc_hrcr(struct port_t *port)
{
	u32 cmd = 0;

	if (port->rx_rising_edge)
		cmd |= port->edge_cfg_bit;

	return cmd;
}

/* Write Tx DMA cmd */
inline void write_tx_cmd(struct port_t *port, int chan, int cmd)
{
	reg_writel(port, HTCR_OFFSET, chan << HXCR_CHAN_SHIFT |
			(*port->htcr)(port) | cmd);
}

/* Write Rx DMA cmd */
inline void write_rx_cmd(struct port_t *port, int chan, int cmd)
{
	reg_writel(port, HRCR_OFFSET, chan << HXCR_CHAN_SHIFT |
			(*port->htcr)(port) | cmd);
}

/* Dump descriptors of one channel */
static void dump_desc(struct channel_t *ch)
{
	int i;

	dev_dbg(&ch->dev->dev, "Dump descriptor\n");
	for (i = 0; i < DESC_PER_RING; i++) {
		dev_dbg(&ch->dev->dev, "tx_desc[%d]: %08x %08x %08x %08x\n", i,
				ch->tx_desc[i].bint_bof_eof_eoq_nbt,
				ch->tx_desc[i].crcc_pri_tba,
				ch->tx_desc[i].ntda,
				ch->tx_desc[i].cft_abt_und);
	}
	for (i = 0; i < DESC_PER_RING; i++) {
		dev_dbg(&ch->dev->dev, "rx_desc[%d]: %08x %08x %08x %08x\n", i,
				ch->rx_desc[i].sim_ibc_eoq_sob,
				ch->rx_desc[i].rba, ch->rx_desc[i].nrda,
				ch->rx_desc[i].fr_abt_ovf_fcrc_nbr);
	}
}

/**
 * reset_ring() : Make Tx/Rx ring ready for data transfer
 *	e.g.
 *	Tx: every descriptor is EOQ
 *	Rx: only EOQ at the last descriptor
 */
static void reset_ring(struct channel_t *ch)
{
	int i;

	ch->tx_cur_pos = 0;
	ch->tx_eoq_pos = 0;
	ch->rx_cur_pos = 0;
	ch->rx_frm_pos = 0;

	ch->tx_bytes = 0;
	ch->rx_bytes = 0;

	for (i = 0; i < DESC_PER_RING; i++) {

		ch->tx_desc[i].bint_bof_eof_eoq_nbt = TX_DESC_EOQ;
		ch->tx_desc[i].cft_abt_und = 0;

		ch->rx_desc[i].sim_ibc_eoq_sob = RX_DESC_IBC | BUFFER_SIZE;
		ch->rx_desc[i].fr_abt_ovf_fcrc_nbr = -1;

	}

	ch->rx_desc[DESC_PER_RING - 1].sim_ibc_eoq_sob |= RX_DESC_EOQ;
}

/**
 * channel_open() : This function is called when executing "ifconfig xxx up"
 */
static int channel_open(struct net_device *dev)
{
	struct channel_t *ch = dev_to_channel(dev);
	struct port_t *port = ch->port;
	int timeout = 500;

	/* Reset TX/RX ring */
	reset_ring(ch);

	/* Set IBA */
	port->iba[ch->id].tda = virt_to_phys(ch->tx_desc);
	port->iba[ch->id].rda = virt_to_phys(ch->rx_desc);

	/* Kick DMA */
	write_tx_cmd(port, ch->id, DMA_CMD_START);
	write_rx_cmd(port, ch->id, DMA_CMD_START);

	/* Tx/Rx is running now */
	ch->tx_running = 1;
	ch->rx_running = 1;

	/* Since Tx queue is empty, wait TX to halt */
	while (ch->tx_running) {
		if (timeout-- <= 0) {
			dev_err(&dev->dev, "Wait Tx halt timeout\n");
			return -EBUSY;
		}
		msleep(1);
	}

	/* Start network queue */
	netif_start_queue(dev);

	/* Call hdlc layer open */
	return hdlc_open(dev);
}

/**
 * channel_close() : This function is called when executing "ifconfig xxx down"
 */
static int channel_close(struct net_device *dev)
{
	struct channel_t *ch = dev_to_channel(dev);
	struct port_t *port = ch->port;
	int timeout = 500;

	/* Stop network queue */
	netif_stop_queue(dev);

	/* Stop DMA */
	write_tx_cmd(port, ch->id, DMA_CMD_HALT);
	write_rx_cmd(port, ch->id, DMA_CMD_HALT);

	/* Wait Tx & Rx to halt */
	while (ch->tx_running || ch->rx_running) {
		if (timeout-- <= 0) {
			dev_err(&dev->dev, "Wait Tx & Rx halt timeout\n");
			break;
		}
		msleep(1);
	}

	/* Call hdlc layer close */
	hdlc_close(dev);

	return 0;
}

/**
 * channel_attach() : This function will be called to change HDLC encoding and
 * etc.
 */
static int channel_attach(struct net_device *dev, unsigned short encoding,
			  unsigned short parity)
{
	/* Currently not support */
	return 0;
}

/**
 * channel_xmit() : This function will be called to transmit one frame
 */
static int channel_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct channel_t *ch = dev_to_channel(dev);
	struct port_t *port = ch->port;
	struct net_device_stats *stats = &ch->dev->stats;

	/* check skb length */
	if (skb->len > BUFFER_SIZE || skb->len < 2) {
		dev_warn(&dev->dev, "Invalid packet length\n");
		stats->tx_dropped++;
		return 0;
	}

	/* check any free descriptor */
	if (tx_desc_free(ch) < 1) {
		dev_err(&dev->dev, "No available descriptor\n");
		stats->tx_dropped++;
		return -EBUSY;
	}

	/* check buffer alignment */
	if ((u32) skb->data & 0x3) {
		/* alloc new skb for alignment */
		struct sk_buff *skb1 = dev_alloc_skb(skb->len);
		memcpy(skb_put(skb1, skb->len), skb->data, skb->len);
		dev_kfree_skb(skb);
		skb = skb1;
	}

	/* map skb address in tx descriptor */
	ch->tx_desc[ch->tx_eoq_pos].crcc_pri_tba = dma_map_single(NULL,
			skb->data, skb->len, DMA_TO_DEVICE);
	ch->tx_desc[ch->tx_eoq_pos].skb = skb;

#ifdef DEBUG_DUMP_TX
	{
		int i;
		dev_debug(&dev->dev, "Tx %d bytes:\n", skb->len);
		for (i = 0; i < skb->len; i++)
			dev_debug(&dev->dev, "%02x\n", skb->data[i]);
	}
#endif
	spin_lock(&port->lock);

	/* set descriptor flag */
	ch->tx_desc[ch->tx_eoq_pos].bint_bof_eof_eoq_nbt = TX_DESC_BINT
		| TX_DESC_BOF | TX_DESC_EOF | skb->len;
	ch->tx_eoq_pos = next_desc_num(ch->tx_eoq_pos);

	/* stop xmit queue if no descriptor available for more frame */
	if (tx_desc_free(ch) < 1)
		netif_stop_queue(dev);

	/* start xmit */
	if (!ch->tx_running) {
		/* reset TDA */
		port->iba[ch->id].tda =
			virt_to_phys(&ch->tx_desc[ch->tx_cur_pos]);

		/* write START cmd */
		write_tx_cmd(port, ch->id, DMA_CMD_START);
		ch->tx_running = 1;
	}

	spin_unlock(&port->lock);

	return 0;
}

/**
 * do_tx_intr() : Sub-routine to handle tx interrupt
 */
static void do_tx_intr(struct port_t *port, u16 status)
{
	int id = (status & HDLC_INT_CHAN_MASK) >> HDLC_INT_CHAN_SHIFT;
	struct channel_t *ch = port->ch[id];
	struct net_device_stats *stats = &ch->dev->stats;
	u16 desc_status;

	if (status & HDLC_INT_HALT)
		ch->tx_running = 0;

	if (status & HDLC_INT_EOQ) {
		if (tx_desc_pending(ch) > 0) {
			/* reset TDA */
			port->iba[ch->id].tda =
				virt_to_phys(&ch->tx_desc[ch->tx_cur_pos]);

			/* write START cmd */
			write_tx_cmd(port, ch->id, DMA_CMD_START);
			ch->tx_running = 1;
		}
	}

	if (status & HDLC_INT_BE) {
		desc_status = ch->tx_desc[ch->tx_cur_pos].cft_abt_und;
		if (desc_status & TX_DESC_ERR_MASK) {
			dev_err(&ch->dev->dev, "Tx Desc[%d] err=%x\n",
					ch->tx_cur_pos, desc_status);
		} else {
			ch->tx_bytes += ch->tx_desc[ch->tx_cur_pos].
				bint_bof_eof_eoq_nbt & TX_DESC_NBT_MASK;
		}
		ch->tx_desc[ch->tx_cur_pos].bint_bof_eof_eoq_nbt |= TX_DESC_EOQ;

		/* free skb */
		dev_kfree_skb(ch->tx_desc[ch->tx_cur_pos].skb);
		ch->tx_desc[ch->tx_cur_pos].skb = NULL;

		ch->tx_cur_pos = next_desc_num(ch->tx_cur_pos);

		if (tx_desc_free(ch) > 2)
			netif_wake_queue(ch->dev);
	}

	if (status & HDLC_INT_CFT) {
		stats->tx_packets++;
		stats->tx_bytes += ch->tx_bytes;
		ch->tx_bytes = 0;
	}

	if (status & HDLC_INT_RRLF) {
		desc_status = ch->tx_desc[ch->tx_cur_pos].cft_abt_und;
		dev_err(&ch->dev->dev, "RRLF: Tx Desc[%d]=%x\n",
				ch->tx_cur_pos, desc_status);
		stats->tx_errors++;

		/* restart tx */
		write_tx_cmd(port, ch->id, DMA_CMD_CONTINUE);
	}
}

/* rx_ring_sync */
static void rx_ring_sync(struct channel_t *ch)
{
	/* increase rx_frm_pos pointer to sync with rx_cur_pos */
	while (ch->rx_frm_pos != ch->rx_cur_pos) {
		ch->rx_desc[prev_desc_num(ch->rx_frm_pos)].sim_ibc_eoq_sob
			&= ~RX_DESC_EOQ;
		ch->rx_desc[ch->rx_frm_pos].sim_ibc_eoq_sob |= RX_DESC_EOQ;
		ch->rx_desc[ch->rx_frm_pos].fr_abt_ovf_fcrc_nbr = -1;
		ch->rx_frm_pos = next_desc_num(ch->rx_frm_pos);
	}
	ch->rx_bytes = 0;
}

/**
 * do_rx_intr(): Sub-routine to handle rx interrupt
 */
static void do_rx_intr(struct port_t *port, u16 status)
{
	int id = (status & HDLC_INT_CHAN_MASK) >> HDLC_INT_CHAN_SHIFT;
	struct channel_t *ch = port->ch[id];
	struct net_device_stats *stats = &ch->dev->stats;
	struct sk_buff *skb;
	u16 desc_status;
	int len;

	if (status & HDLC_INT_HALT) {
		dev_dbg(&ch->dev->dev, "rx halt\n");
		ch->rx_running = 0;
	}

	if (status & HDLC_INT_EOQ) {
		dev_warn(&ch->dev->dev, "rx eoq\n");
		rx_ring_sync(ch);
		write_rx_cmd(port, ch->id, DMA_CMD_CONTINUE);
	} else if (status & HDLC_INT_ERF) {
		desc_status = ch->rx_desc[ch->rx_cur_pos].fr_abt_ovf_fcrc_nbr;
		dev_err(&ch->dev->dev, "ERF: Rx Desc[%d]=%x Ints=%x\n",
				ch->rx_cur_pos, desc_status, status);
		stats->rx_errors++;
		ch->rx_cur_pos = next_desc_num(ch->rx_cur_pos);
		rx_ring_sync(ch);
	} else if (status & (HDLC_INT_BF | HDLC_INT_CFR)) {
		desc_status = ch->rx_desc[ch->rx_cur_pos].fr_abt_ovf_fcrc_nbr;
		if (desc_status & RX_DESC_ERR_MASK) {
			dev_err(&ch->dev->dev, "Rx Desc[%d] err=%x Ints=%x\n",
					ch->rx_cur_pos, desc_status, status);
		} else {
			if (status & HDLC_INT_BF) {
				if ((desc_status & RX_DESC_NBR_MASK) !=
						BUFFER_SIZE)
					dev_err(&ch->dev->dev, "size in \
						descriptor != BUFFER_SIZE\n");
			}

			ch->rx_bytes += desc_status & RX_DESC_NBR_MASK;
		}
		ch->rx_cur_pos = next_desc_num(ch->rx_cur_pos);
	}

	if (status & HDLC_INT_CFR) {
		struct sk_buff *skbi;

		stats->rx_packets++;
		stats->rx_bytes += ch->rx_bytes;

		if (ch->rx_bytes > BUFFER_SIZE) {
			int pos = ch->rx_frm_pos;

			/* alloc new skb for long frame */
			skb = dev_alloc_skb(ch->rx_bytes);

			/* copy data from multiple-descriptor */
			while (ch->rx_bytes > 0) {
				skbi = ch->rx_desc[pos].skb;
				len = ch->rx_bytes > BUFFER_SIZE ?
					BUFFER_SIZE : ch->rx_bytes;
				dma_map_single(NULL, skbi->data, len,
						DMA_FROM_DEVICE);
				memcpy(skb_put(skb, len), skbi->data, len);
				ch->rx_bytes -= len;
				pos = next_desc_num(pos);
			}
		} else {
			/* get finished skb */
			skb = ch->rx_desc[ch->rx_frm_pos].skb;
			skb_put(skb, ch->rx_bytes);
			dma_map_single(NULL, skb->data, skb->len,
					DMA_FROM_DEVICE);

			/* prepare new skb for this rx descriptor */
			skbi = dev_alloc_skb(BUFFER_SIZE);
			ch->rx_desc[ch->rx_frm_pos].rba =
				dma_map_single(NULL, skbi->data,
						BUFFER_SIZE, DMA_FROM_DEVICE);
			ch->rx_desc[ch->rx_frm_pos].skb = skbi;
		}

#ifdef DEBUG_DUMP_RX
		{
			int i;
			dev_debug(&ch->dev->dev, "Rx %d bytes:\n", skb->len);
			for (i = 0; i < skb->len; i++)
				dev_debug(&ch->dev->dev, "%02x\n",
						skb->data[i]);
		}
#endif

		rx_ring_sync(ch);

		skb->dev = ch->dev;
		skb->protocol = hdlc_type_trans(skb, ch->dev);

		netif_rx(skb);
	}

}

/**
 * spear_hdlc_int_tasklet() : interrupt processing tasklet
 */
static void spear_hdlc_int_tasklet(unsigned long data)
{
	struct port_t *port = (struct port_t *)data;
	u16 status;

	spin_lock(&port->lock);

	while ((status = port->intq[port->intq_pos]) & NEW_STATUS) {

		dev_dbg(&port->pdev->dev, "int[%d] = %04x\n",
				port->intq_pos, status);

		if (status & port->hdlc_int_type_bit) {
			/* Tx interrupt */
			do_tx_intr(port, status);
		} else {
			/* Rx interrupt */
			do_rx_intr(port, status);
		}

		/* finish handling the status */
		port->intq[port->intq_pos] = 0;
		port->intq_pos = (port->intq_pos + 1) & (port->intq_size - 1);
	}

	spin_unlock(&port->lock);
}

/**
 * port_interrupt_handler() : Read interrupt status from int queue,
 * callsubroutine according to the Tx/Rx flag in status word
 */
static irqreturn_t port_interrupt_handler(int irq, void *dev_id)
{
	struct port_t *port = (struct port_t *)dev_id;
	u32 ir;

	ir = reg_readl(port, IR_OFFSET);

	if (ir & ~IR_HDLC)
		dev_warn(&port->pdev->dev, "ir=%x\n", ir);

	if (ir & IR_HDLC)
		tasklet_schedule(&port->int_tasklet);

	return IRQ_HANDLED;
}

/**
 * channel_init() : Initialize one channel: alloc descirptors, prepare skb for
 * rx, make descirptor ring etc.
 */
static void channel_init(struct channel_t *ch, u32 *desc_addr)
{
	int i;
	struct sk_buff *skb;
	struct port_t *port = ch->port;

	/* assign memory for descriptors */
	ch->tx_desc = (struct tx_desc *)*desc_addr;
	*desc_addr += sizeof(struct tx_desc) *DESC_PER_RING;
	ch->rx_desc = (struct rx_desc *)*desc_addr;
	*desc_addr += sizeof(struct rx_desc) *DESC_PER_RING;

	/* prepare buffer for every descriptor
	 * build descriptor ring
	 */
	for (i = 0; i < DESC_PER_RING; i++) {
		/* fill tx descriptor */
		ch->tx_desc[i].ntda = virt_to_phys(ch->tx_desc +
				next_desc_num(i));

		/* fill rx descriptor */
		skb = dev_alloc_skb(BUFFER_SIZE);
		ch->rx_desc[i].rba = dma_map_single(NULL, skb->data,
				BUFFER_SIZE, DMA_FROM_DEVICE);
		ch->rx_desc[i].skb = skb;

		ch->rx_desc[i].nrda = virt_to_phys(ch->rx_desc +
				next_desc_num(i));
	}

	dump_desc(ch);
}

/**
 * spear_tsa_init() : TSA init for TDM HDLC controller
 */
static int spear_tsa_init(struct port_t *port)
{
	int timeout = 500;

	if (!port->has_tsa)
		return -ENODEV;

	reg_writel(port, TAAR_RX_OFFSET, TSA_INIT);
	reg_writel(port, TAAR_TX_OFFSET, TSA_INIT);
	while ((reg_readl(port, TAAR_RX_OFFSET) & TSA_INIT) ||
			(reg_readl(port, TAAR_TX_OFFSET) & TSA_INIT)) {
		if (timeout-- <= 0) {
			dev_err(&port->pdev->dev, "TSA init timeout\n");
			break;
		}
		msleep(1);
	}

	return 0;
}

/**
 * spear_create_tsa_sysfs() : Create timeslot objects in sysfs
 */
static int spear_create_tsa_sysfs(struct port_t *port)
{
	struct platform_device *pdev = port->pdev;
	struct kobject *kobj;
	char name[16];
	int i;

	port->tsa_kobjs = kzalloc(sizeof(void *) *port->nr_timeslot,
			GFP_KERNEL);
	if (!port->tsa_kobjs) {
		dev_err(&pdev->dev, "alloc tsa_kobjs failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < port->nr_timeslot; i++) {
		sprintf(name, "ts%d", i);
		kobj = kobject_create_and_add(name, &pdev->dev.kobj);
		port->tsa_kobjs[i] = kobj;

		if (sysfs_create_group(kobj, &tsa_attr_group))
			dev_err(&pdev->dev, "create timeslot kobj to sysfs \
					failed\n");
	}

	return 0;
}

/**
 * spear_remove_tsa_sysfs() : Remove timeslot objects in sysfs
 */
static void spear_remove_tsa_sysfs(struct port_t *port)
{
	struct kobject *kobj;
	int i;

	for (i = 0; i < port->nr_timeslot; i++) {
		kobj = port->tsa_kobjs[i];
		sysfs_remove_group(kobj, &tsa_attr_group);
		kobject_put(kobj);
	}

	kfree(port->tsa_kobjs);
}

/* net_device interface */
static const struct net_device_ops spear_hdlc_ops = {
	.ndo_open	= channel_open,
	.ndo_stop	= channel_close,
	.ndo_change_mtu = hdlc_change_mtu,
	.ndo_start_xmit = hdlc_start_xmit,
	.ndo_do_ioctl	= hdlc_ioctl,
};

/**
 * spear_hdlc_drv_probe() : Common driver probe function
 */
static int spear_hdlc_drv_probe(struct platform_device *pdev)
{
	struct port_t *port = platform_get_drvdata(pdev);
	struct resource *mem, *irq;
	dma_addr_t desc_addr;
	int i, err;

	/* init spinlock */
	spin_lock_init(&port->lock);

	/* get platform resource */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_dbg(&pdev->dev, "memory resource not defined\n");
		return -ENOMEM;
	}

	if (!devm_request_mem_region(&pdev->dev, mem->start,
				resource_size(mem), pdev->name)) {
		dev_dbg(&pdev->dev, "cannot request region\n");
		return -EBUSY;
	}

	/* map controller registers */
	port->reg_base = devm_ioremap(&pdev->dev, mem->start,
			resource_size(mem));
	if (!port->reg_base) {
		dev_dbg(&pdev->dev, "failed to remap registers\n");
		return -ENOMEM;
	}

	/* alloc memory pool */
	port->mem_virt = (u32) dma_alloc_coherent(&pdev->dev,
			MEM_POOL_SIZE, &port->mem_phys, GFP_KERNEL);
	if (!port->mem_virt)
		return -ENOMEM;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_dbg(&pdev->dev, "irq resource not defined\n");
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "virt=%08x, phys=%08x\n",
		port->mem_virt, port->mem_phys);

	/* assign memory for initial block and descriptors */
	port->iba = (struct ibe *) port->mem_virt;
	port->intq = (u16 *) (port->mem_virt + port->max_timeslot *
			sizeof(struct ibe));
	desc_addr = (u32) port->intq + port->intq_size *
		sizeof(unsigned short);

	dev_dbg(&pdev->dev, "iba=%p, intq=%p, desc=%08x\n",
		port->iba, port->intq, desc_addr);

	/* set IBA */
	reg_writel(port, IBAR_OFFSET, virt_to_phys(port->iba));

	if (port->has_tsa) {
		spear_tsa_init(port);
		spear_create_tsa_sysfs(port);
	}

	/* create hdlc channel */
	for (i = 0; i < port->nr_channel; i++) {
		struct channel_t *ch;
		struct net_device *dev;
		struct hdlc_device *hdlc;

		/* alloc channel structure */
		ch = kzalloc(sizeof(struct channel_t), GFP_KERNEL);
		if (!ch)
			return -ENOMEM;

		ch->id = i;
		ch->port = port;
		port->ch[i] = ch;

		/* alloc hdlc device */
		dev = alloc_hdlcdev(ch);
		if (!dev)
			return -ENODEV;

		ch->dev = dev;
		hdlc = dev_to_hdlc(dev);

		/* register Channel Ops. */
		hdlc->attach = channel_attach;
		hdlc->xmit = channel_xmit;
		dev->netdev_ops = &spear_hdlc_ops;

		dev->tx_queue_len = DESC_PER_RING - 1;

		/* register hdlc channel */
		err = register_hdlc_device(dev);
		if (err)
			return err;

		/* initialize channel */
		channel_init(ch, &desc_addr);
	}

	/* init interrupt tasklet */
	tasklet_init(&port->int_tasklet, spear_hdlc_int_tasklet,
			(unsigned long)port);

	/* register irq handler */
	if (request_irq(irq->start, port_interrupt_handler, 0,
				pdev->name, (void *)port))
		return -ENODEV;

	/* unmask all interrupt */
	reg_writel(port, IMR_OFFSET, 0);

	dev_info(&pdev->dev, "HDLC probe successful");
	return 0;
}

/**
 * tdm_hdlc_drv_probe() : Driver probe function for TDM HDLC controller
 */
static int tdm_hdlc_drv_probe(struct platform_device *pdev)
{
	struct port_t *port;
	struct tdm_hdlc_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct clk *clk;
	int ret = 0;

	if (!pdata && !np)
		return -EINVAL;

	if (np) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			pr_warn("tdm_hdlc kzalloc fail\n");
			return -ENOMEM;
		}

		if (of_property_read_u32(np, "ip_type", &pdata->ip_type))
			pr_debug("unable to get ip_type\n");
		if (of_property_read_u32(np, "nr_channel", &pdata->nr_channel))
			pr_debug("unable to get nr_channel\n");
		if (of_property_read_u32(np, "nr_timeslot",
					&pdata->nr_timeslot))
			pr_debug("unable to get nr_timeslot\n");
		if (of_property_read_u32(np, "tx_falling_edge",
					&pdata->tx_falling_edge))
			pr_debug("unable to get tx_falling_edge\n");
		if (of_property_read_u32(np, "rx_rising_edge",
					&pdata->rx_rising_edge))
			pr_debug("unable to get rx_rising_edge\n");
		if (of_property_read_u32(np, "ts0_delay", &pdata->ts0_delay))
			pr_debug("unable to get ts0_delay\n");
	}

	port = devm_kzalloc(&pdev->dev, sizeof(struct port_t), GFP_KERNEL);
	if (!port) {
		pr_warn("tdm_hdlc port kzalloc fail\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, port);

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto free_mem;
	}

	port->clk		= clk;
	port->pdev		= pdev;
	port->has_tsa		= 1;
	port->nr_channel	= pdata->nr_channel;
	port->nr_timeslot	= pdata->nr_timeslot;
	port->ts0_delay		= pdata->ts0_delay;
	port->tx_falling_edge	= pdata->tx_falling_edge;
	port->rx_rising_edge	= pdata->rx_rising_edge;
	port->htcr		= tdm_hdlc_htcr;
	port->hrcr		= tdm_hdlc_hrcr;

	switch (pdata->ip_type) {
	case SPEAR1310_TDM_HDLC:
		ret = clk_set_rate(clk, 250000000);	/* for 250 MHz */
		if (ret) {
			pr_err("Failed to set proper clk rate\n");
			goto free_clk;
		}

		port->max_timeslot	= 512;
		port->hdlc_int_type_bit = SPEAR1310_HDLC_INT_TYPE_BIT;
		port->hdlc_int_chan_shift = SPEAR1310_HDLC_INT_CHAN_SHIFT;
		port->edge_cfg_bit	= SPEAR1310_HDLC_EDGE_CFG_BIT;
		port->edge_cfg_en_bit	= SPEAR1310_HDLC_CFG_EN_BIT;
		port->intq_size		= SPEAR1310_HDLC_INTQ_SIZE;
		port->tsa_init_bit	= SPEAR1310_HDLC_TSA_INIT_BIT;
		port->tsa_busy_bit	= SPEAR1310_HDLC_TSA_BUSY_BIT;
		port->tsa_read_bit	= SPEAR1310_HDLC_TSA_READ_BIT;
		port->tsa_val_shift	= SPEAR1310_HDLC_TSA_VAL_SHIFT;
		break;
	case SPEAR310_TDM_HDLC:
		port->max_timeslot	= 128;
		port->hdlc_int_type_bit = SPEAR310_HDLC_INT_TYPE_BIT;
		port->hdlc_int_chan_shift = SPEAR310_HDLC_INT_CHAN_SHIFT;
		port->edge_cfg_bit	= SPEAR310_HDLC_EDGE_CFG_BIT;
		port->edge_cfg_en_bit	= SPEAR310_HDLC_CFG_EN_BIT;
		port->intq_size		= SPEAR310_HDLC_INTQ_SIZE;
		port->tsa_init_bit	= SPEAR310_HDLC_TSA_INIT_BIT;
		port->tsa_busy_bit	= SPEAR310_HDLC_TSA_BUSY_BIT;
		port->tsa_read_bit	= SPEAR310_HDLC_TSA_READ_BIT;
		port->tsa_val_shift	= SPEAR310_HDLC_TSA_VAL_SHIFT;
		break;
	default:
		goto free_clk;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("Failed to enable TDM clk\n");
		goto free_clk;
	}

	/* call command probe */
	ret = spear_hdlc_drv_probe(pdev);
	if (ret) {
		pr_err("SPEAr HDLC driver probe failed\n");
		goto free_disable_clk;
	}

	return ret;

free_disable_clk:
	clk_disable_unprepare(clk);
free_clk:
	clk_put(clk);
free_mem:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

/**
 * rs485_hdlc_drv_probe() : Driver probe function for RS485 HDLC controller
 */
static int rs485_hdlc_drv_probe(struct platform_device *pdev)
{
	struct port_t *port;
	struct clk *clk;
	struct rs485_hdlc_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (!pdata && !np)
		return -EINVAL;

	if (np) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			pr_warn("td_hdlc kzalloc fail\n");
			return -ENOMEM;
		}

		if (of_property_read_u32(np, "tx_falling_edge",
					&pdata->tx_falling_edge))
			pr_debug("unable to get tx_falling_edge\n");
		if (of_property_read_u32(np, "rx_rising_edge",
					&pdata->rx_rising_edge))
			pr_debug("unable to get rx_rising_edge\n");
		if (of_property_read_u32(np, "cts_enable", &pdata->cts_enable))
			pr_debug("unable to get cts_enable\n");
		if (of_property_read_u32(np, "cts_delay", &pdata->cts_delay))
			pr_debug("unable to get cts_delay\n");
	}

	port = devm_kzalloc(&pdev->dev, sizeof(struct port_t), GFP_KERNEL);
	if (!port) {
		pr_warn("tdm_hdlc port kzalloc fail\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, port);

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto free_mem;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("Failed to enable TDM clk\n");
		goto free_clk;
	}

	port->clk		= clk;
	port->pdev		= pdev;
	port->has_tsa		= 0;
	port->nr_channel	= 1;
	port->cts_enable	= pdata->cts_enable;
	port->cts_delay		= pdata->cts_delay;
	port->tx_falling_edge	= pdata->tx_falling_edge;
	port->rx_rising_edge	= pdata->rx_rising_edge;
	port->htcr		= rs485_hdlc_htcr;
	port->hrcr		= rs485_hdlc_hrcr;

	port->max_timeslot	= 128;
	port->nr_timeslot	= 1;
	port->hdlc_int_type_bit = RS485_INT_TYPE_BIT;
	port->hdlc_int_chan_shift = RS485_INT_CHAN_SHIFT;
	port->edge_cfg_bit	= RS485_EDGE_CFG_BIT;
	port->edge_cfg_en_bit	= RS485_CFG_EN_BIT;
	port->intq_size		= RS485_INTQ_SIZE;

	/* call common probe */
	ret = spear_hdlc_drv_probe(pdev);
	if (ret) {
		pr_err("SPEAr HDLC driver probe failed\n");
		goto free_disable_clk;
	}

	return ret;

free_disable_clk:
	clk_disable_unprepare(clk);
free_clk:
	clk_put(clk);
free_mem:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

/* spear_hdlc_drv_remove */
static int spear_hdlc_drv_remove(struct platform_device *pdev)
{
	struct port_t *port = platform_get_drvdata(pdev);
	struct resource *irq;
	int i;

	tasklet_kill(&port->int_tasklet);

	/* free irq */
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	free_irq(irq->start, port);

	/* free all channels */
	for (i = 0; i < port->nr_channel; i++) {
		unregister_hdlc_device(port->ch[i]->dev);
		free_netdev(port->ch[i]->dev);
		kfree(port->ch[i]);
	}

	/* remove sysfs */
	if (port->has_tsa)
		spear_remove_tsa_sysfs(port);

	/* free dma memory pool */
	dma_free_coherent(&pdev->dev, MEM_POOL_SIZE, (void *)
			port->mem_virt, port->mem_phys);

	if (port->clk) {
		clk_disable_unprepare(port->clk);
		clk_put(port->clk);
	}

	/* free port */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int spear_hdlc_drv_suspend(struct device *dev)
{
	return 0;
}

static int spear_hdlc_drv_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(spear_hdlc_pm_ops, spear_hdlc_drv_suspend,
		spear_hdlc_drv_resume);

#ifdef CONFIG_OF
static const struct of_device_id tdm_hdlc_id_table[] = {
	{ .compatible = "st,tdm_hdlc" },
	{}
};
MODULE_DEVICE_TABLE(of, tdm_hdlc_id_table);

static const struct of_device_id rs485_hdlc_id_table[] = {
	{ .compatible = "st,rs485_hdlc" },
	{}
};
MODULE_DEVICE_TABLE(of, rs485_hdlc_id_table);
#endif

static struct platform_driver tdm_hdlc_driver = {
	.driver		= {
		.name	= "tdm_hdlc",
		.pm	= &spear_hdlc_pm_ops,
		.of_match_table = of_match_ptr(tdm_hdlc_id_table),
	},
	.probe		= tdm_hdlc_drv_probe,
	.remove		= spear_hdlc_drv_remove,
};

static struct platform_driver rs485_hdlc_driver = {
	.driver		= {
		.name	= "rs485_hdlc",
		.pm	= &spear_hdlc_pm_ops,
		.of_match_table = of_match_ptr(rs485_hdlc_id_table),
	},
	.probe		= rs485_hdlc_drv_probe,
	.remove		= spear_hdlc_drv_remove,
};

static int __init spear_hdlc_init(void)
{
	int ret;

	/* register tdm hdlc driver */
	ret = platform_driver_register(&tdm_hdlc_driver);
	if (ret)
		return ret;

	/* register rs485 hdlc driver */
	ret = platform_driver_register(&rs485_hdlc_driver);
	if (ret) {
		platform_driver_unregister(&tdm_hdlc_driver);
		return ret;
	}

	return 0;
}
module_init(spear_hdlc_init);

static void __exit spear_hdlc_cleanup(void)
{
	platform_driver_unregister(&tdm_hdlc_driver);
	platform_driver_unregister(&rs485_hdlc_driver);
}
module_exit(spear_hdlc_cleanup);

MODULE_AUTHOR("Frank Shi <frank.shi@st.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HDLC controller driver for SPEAr SoC");
