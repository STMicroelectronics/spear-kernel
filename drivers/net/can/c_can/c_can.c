/*
 * CAN bus driver for Bosch C_CAN controller
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the C_CAN driver originally written by:
 * Copyright (C) 2007
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 *
 * TX and RX NAPI implementation has been borrowed from at91 CAN driver
 * written by:
 * Copyright
 * (C) 2007 by Hans J. Koch <hjk@linutronix.de>
 * (C) 2008, 2009 by Marc Kleine-Budde <kernel@pengutronix.de>
 *
 * Bosch C_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch C_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/pdf/Users_Manual_C_CAN.pdf
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#include "c_can.h"

static struct can_bittiming_const c_can_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,	/* 6-bit BRP field + 4-bit BRPE field*/
	.brp_inc = 1,
};

static inline int get_tx_next_msg_obj(const struct c_can_priv *priv)
{
	return (priv->tx_next & C_CAN_NEXT_MSG_OBJ_MASK) +
			C_CAN_MSG_OBJ_TX_FIRST;
}

static inline int get_tx_echo_msg_obj(const struct c_can_priv *priv)
{
	return (priv->tx_echo & C_CAN_NEXT_MSG_OBJ_MASK) +
			C_CAN_MSG_OBJ_TX_FIRST;
}

static u32 c_can_read_reg32(struct c_can_priv *priv, void *reg)
{
	u32 val = priv->read_reg(priv, reg);
	val |= ((u32) priv->read_reg(priv, reg + 2)) << 16;
	return val;
}

void c_can_enable_all_interrupts(struct c_can_priv *priv,
						int enable)
{
	unsigned int cntrl_save = priv->read_reg(priv,
						&priv->reg_base->control);

	if (enable)
		cntrl_save |= (CONTROL_SIE | CONTROL_EIE | CONTROL_IE);
	else
		cntrl_save &= ~(CONTROL_EIE | CONTROL_IE | CONTROL_SIE);

	priv->write_reg(priv, &priv->reg_base->control, cntrl_save);
}
EXPORT_SYMBOL_GPL(c_can_enable_all_interrupts);

static inline void c_can_object_get(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct c_can_priv *priv = netdev_priv(dev);
	int count = MIN_TIMEOUT_VALUE;

	/*
	 * As per specs, after writting the message object number in the
	 * IF command request register the transfer b/w interface
	 * register and message RAM must be complete in 6 CAN-CLK
	 * period.
	 */

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].com_mask,
			IFX_WRITE_LOW_16BIT(mask));
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].com_reg,
			IFX_WRITE_LOW_16BIT(objno + 1));

	while (count) {
		if (!(priv->read_reg(priv,
					&priv->reg_base->ifreg[iface].com_reg) &
					IF_COMR_BUSY))
			break;
		count--;
		udelay(1);
	}

	if (!count)
		dev_err(dev->dev.parent, "timed out in object get\n");
}

static inline void c_can_object_put(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct c_can_priv *priv = netdev_priv(dev);
	int count = MIN_TIMEOUT_VALUE;

	/*
	 * As per specs, after writting the message object number in the
	 * IF command request register the transfer b/w interface
	 * register and message RAM must be complete in 6 CAN-CLK
	 * period.
	 */

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].com_mask,
			(IF_COMM_WR | IFX_WRITE_LOW_16BIT(mask)));
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].com_reg,
			IFX_WRITE_LOW_16BIT(objno + 1));

	while (count) {
		if (!(priv->read_reg(priv,
				&priv->reg_base->ifreg[iface].com_reg) &
				IF_COMR_BUSY))
			break;

		count--;
		udelay(1);
	}

	if (!count)
		dev_err(dev->dev.parent, "timed out in object put\n");
}

int c_can_write_msg_object(struct net_device *dev,
			int iface, struct can_frame *frame, int objno)
{
	int i;
	u16 flags = 0;
	unsigned int id;
	struct c_can_priv *priv = netdev_priv(dev);

	if (frame->can_id & CAN_EFF_FLAG) {
		id = frame->can_id & CAN_EFF_MASK;
		flags |= IF_ARB_MSGXTD;
	} else
		id = ((frame->can_id & CAN_SFF_MASK) << 18);

	if (!(frame->can_id & CAN_RTR_FLAG))
		flags |= IF_ARB_TRANSMIT;

	flags |= IF_ARB_MSGVAL;

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb1,
				IFX_WRITE_LOW_16BIT(id));
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb2, flags |
				IFX_WRITE_HIGH_16BIT(id));

	for (i = 0; i < frame->can_dlc; i += 2) {
		priv->write_reg(priv, &priv->reg_base->ifreg[iface].data[i / 2],
				frame->data[i] | (frame->data[i + 1] << 8));
	}

	return frame->can_dlc;
}

static inline void c_can_mark_rx_msg_obj(struct net_device *dev,
						int iface, int ctrl_mask,
						int obj)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl,
			ctrl_mask & ~(IF_MCONT_MSGLST | IF_MCONT_INTPND));

	c_can_object_put(dev, iface, obj, IF_COMM_CONTROL);

}

static inline void c_can_activate_all_lower_rx_msg_obj(struct net_device *dev,
						int iface,
						int ctrl_mask)
{
	int i;
	struct c_can_priv *priv = netdev_priv(dev);

	for (i = 0; i < C_CAN_MSG_RX_LOW_LAST; i++) {
		priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl,
				ctrl_mask & ~(IF_MCONT_MSGLST |
					IF_MCONT_INTPND | IF_MCONT_NEWDAT));
		c_can_object_put(dev, iface, i + 1, IF_COMM_CONTROL);
	}
}

static inline void c_can_activate_rx_msg_obj(struct net_device *dev,
						int iface, int ctrl_mask,
						int obj)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl,
			ctrl_mask & ~(IF_MCONT_MSGLST |
				IF_MCONT_INTPND | IF_MCONT_NEWDAT));

	c_can_object_put(dev, iface, obj, IF_COMM_CONTROL);

}

static void c_can_handle_lost_msg_obj(struct net_device *dev,
					int iface, int objno)
{
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	dev_err(dev->dev.parent, "msg lost in buffer %d\n", objno);

	c_can_object_get(dev, iface, objno, IF_COMM_ALL &
						~IF_COMM_TXRQST);

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl,
			IF_MCONT_CLR_MSGLST);

	c_can_object_put(dev, 0, objno, IF_COMM_CONTROL);

	/* create an error msg */
	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
	stats->rx_errors++;
	stats->rx_over_errors++;

	netif_receive_skb(skb);
}

static int c_can_read_msg_object(struct net_device *dev, int iface, int ctrl,
				int objno)
{
	u16 flags, data;
	int i;
	unsigned int val;
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	skb = alloc_can_skb(dev, &frame);
	if (!skb) {
		stats->rx_dropped++;
		return -ENOMEM;
	}

	frame->can_dlc = get_can_dlc(ctrl & 0x0F);

	for (i = 0; i < frame->can_dlc; i += 2) {
		data = priv->read_reg(priv,
				&priv->reg_base->ifreg[iface].data[i / 2]);
		frame->data[i] = data;
		frame->data[i + 1] = data >> 8;
	}

	flags =	priv->read_reg(priv, &priv->reg_base->ifreg[iface].arb2);
	val = priv->read_reg(priv, &priv->reg_base->ifreg[iface].arb1) |
		(flags << 16);

	if (flags & IF_ARB_MSGXTD)
		frame->can_id = (val & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		frame->can_id = (val >> 18) & CAN_SFF_MASK;

	if (flags & IF_ARB_TRANSMIT)
		frame->can_id |= CAN_RTR_FLAG;

	netif_receive_skb(skb);

	stats->rx_packets++;
	stats->rx_bytes += frame->can_dlc;

	return 0;
}

static void c_can_setup_receive_object(struct net_device *dev, int iface,
					int objno, unsigned int mask,
					unsigned int id, unsigned int mcont)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].mask1,
			IFX_WRITE_LOW_16BIT(mask));
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].mask2,
			IFX_WRITE_HIGH_16BIT(mask));

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb1,
			IFX_WRITE_LOW_16BIT(id));
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb2,
			(IF_ARB_MSGVAL | IFX_WRITE_HIGH_16BIT(id)));

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl, mcont);
	c_can_object_put(dev, iface, objno, IF_COMM_ALL &
						~IF_COMM_TXRQST);

	dev_dbg(dev->dev.parent, "obj no:%d, msgval:0x%08x\n", objno,
			c_can_read_reg32(priv, &priv->reg_base->msgval1));

}

static void c_can_inval_msg_object(struct net_device *dev, int iface, int objno)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb1, 0);
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].arb2, 0);
	priv->write_reg(priv, &priv->reg_base->ifreg[iface].msg_cntrl, 0);

	c_can_object_put(dev, iface, objno,
				IF_COMM_ARB | IF_COMM_CONTROL);

	dev_dbg(dev->dev.parent, "obj no:%d, msgval:0x%08x\n", objno,
			c_can_read_reg32(priv, &priv->reg_base->msgval1));

}

static netdev_tx_t c_can_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	u32 val;
	u32 msg_obj_no;
	struct c_can_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	msg_obj_no = get_tx_next_msg_obj(priv);

	/* prepare message object for transmission */
	val = c_can_write_msg_object(dev, 0, frame, msg_obj_no);

	/* enable interrupt for this message object */
	priv->write_reg(priv, &priv->reg_base->ifreg[0].msg_cntrl,
			IF_MCONT_TXIE | IF_MCONT_TXRQST | IF_MCONT_EOB |
			(val & 0xf));
	c_can_object_put(dev, 0, msg_obj_no, IF_COMM_ALL);

	can_put_echo_skb(skb, dev, msg_obj_no - C_CAN_MSG_OBJ_TX_FIRST);

	priv->tx_next++;
	if ((priv->tx_next & C_CAN_NEXT_MSG_OBJ_MASK) == 0)
		netif_stop_queue(dev);

	return NETDEV_TX_OK;
}

static int c_can_set_bittiming(struct net_device *dev)
{
	unsigned int reg_btr, reg_brpe, ctrl_save;
	u8 brp, brpe, sjw, tseg1, tseg2;
	u32 ten_bit_brp;
	struct c_can_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;

	/* c_can provides a 6-bit brp and 4-bit brpe fields */
	ten_bit_brp = bt->brp - 1;
	brp = ten_bit_brp & BTR_BRP_MASK;
	brpe = ten_bit_brp >> 6;

	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;

	reg_btr = ((brp) | (sjw << BTR_SJW_SHIFT) | (tseg1 << BTR_TSEG1_SHIFT) |
			(tseg2 << BTR_TSEG2_SHIFT));

	reg_brpe = brpe & BRP_EXT_BRPE_MASK;

	dev_info(dev->dev.parent,
		"setting BTR=%04x BRPE=%04x\n", reg_btr, reg_brpe);

	ctrl_save = priv->read_reg(priv, &priv->reg_base->control);
	priv->write_reg(priv, &priv->reg_base->control,
			ctrl_save | CONTROL_CCE | CONTROL_INIT);
	priv->write_reg(priv, &priv->reg_base->btr, reg_btr);
	priv->write_reg(priv, &priv->reg_base->brp_ext, reg_brpe);
	priv->write_reg(priv, &priv->reg_base->control, ctrl_save);

	return 0;
}

/*
 * Configure C_CAN message objects for Tx and Rx purposes:
 * C_CAN provides a total of 32 message objects that can be configured
 * either for Tx or Rx purposes. Here the first 16 message objects are used as
 * a reception FIFO. The end of reception FIFO is signified by the EoB bit
 * being SET. The remaining 16 message objects are kept aside for Tx purposes.
 * See user guide document for further details on configuring message
 * objects.
 */
static void c_can_configure_msg_objects(struct net_device *dev)
{
	int i;

	/* first invalidate all message objects */
	for (i = 0; i <= C_CAN_NO_OF_OBJECTS; i++)
		c_can_inval_msg_object(dev, 0, i);

	/* setup receive message objects */
	for (i = C_CAN_MSG_OBJ_RX_FIRST + 1 ; i < C_CAN_MSG_OBJ_RX_LAST; i++)
		c_can_setup_receive_object(dev, 0, i, 0, 0,
			((IF_MCONT_RXIE | IF_MCONT_UMASK) & ~IF_MCONT_EOB));

	c_can_setup_receive_object(dev, 0, C_CAN_MSG_OBJ_RX_LAST, 0, 0,
			IF_MCONT_EOB | IF_MCONT_RXIE | IF_MCONT_UMASK);
}

/*
 * Configure C_CAN chip:
 * - enable/disable auto-retransmission
 * - set operating mode
 * - configure message objects
 */
static void c_can_chip_config(struct net_device *dev)
{
	struct c_can_priv *priv = netdev_priv(dev);

	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		/* disable automatic retransmission */
		priv->write_reg(priv, &priv->reg_base->control,
				CONTROL_DISABLE_AR);
	else
		/* enable automatic retransmission */
		priv->write_reg(priv, &priv->reg_base->control,
				CONTROL_ENABLE_AR);

	if (priv->can.ctrlmode & (CAN_CTRLMODE_LISTENONLY &
					CAN_CTRLMODE_LOOPBACK)) {
		/* loopback + silent mode : useful for hot self-test */
		priv->write_reg(priv, &priv->reg_base->control, (CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST));
		priv->write_reg(priv, &priv->reg_base->test,
				(TEST_LBACK | TEST_SILENT));
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* loopback mode : useful for self-test function */
		priv->write_reg(priv, &priv->reg_base->control, (CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST));
		priv->write_reg(priv, &priv->reg_base->test, TEST_LBACK);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* silent mode : bus-monitoring mode */
		priv->write_reg(priv, &priv->reg_base->control, (CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST));
		priv->write_reg(priv, &priv->reg_base->test, TEST_SILENT);
	} else
		/* normal mode*/
		priv->write_reg(priv, &priv->reg_base->control,
				(CONTROL_EIE | CONTROL_SIE | CONTROL_IE));

	/* configure message objects */
	c_can_configure_msg_objects(dev);
}

static void c_can_start(struct net_device *dev)
{
	struct c_can_priv *priv = netdev_priv(dev);

	/* enable status change, error and module interrupts */
	c_can_enable_all_interrupts(priv, ENABLE_ALL_INTERRUPTS);

	/* basic c_can configuration */
	c_can_chip_config(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* reset tx helper pointers */
	priv->tx_next = priv->tx_echo = 0;
}

static void c_can_stop(struct net_device *dev)
{
	struct c_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts */
	c_can_enable_all_interrupts(priv, DISABLE_ALL_INTERRUPTS);

	/* set the state as STOPPED */
	priv->can.state = CAN_STATE_STOPPED;
}

static int c_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		c_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int c_can_get_berr_counter(const struct net_device *dev,
					struct can_berr_counter *bec)
{
	unsigned int reg_err_counter;
	struct c_can_priv *priv = netdev_priv(dev);

	reg_err_counter = priv->read_reg(priv, &priv->reg_base->error_counter);
	bec->rxerr = ((reg_err_counter & ERR_COUNTER_REC_MASK) >>
				ERR_COUNTER_REC_SHIFT);
	bec->txerr = (reg_err_counter & ERR_COUNTER_TEC_MASK);

	return 0;
}

/*
 * theory of operation:
 *
 * priv->tx_echo holds the number of the oldest can_frame put for
 * transmission into the hardware, but not yet ACKed by the CAN tx
 * complete IRQ.
 *
 * We iterate from priv->tx_echo to priv->tx_next and check if the
 * packet has been transmitted, echo it back to the CAN framework.
 * If we discover a not yet transmitted package, stop looking for more.
 */
static void c_can_do_tx(struct net_device *dev)
{
	u32 val;
	u32 msg_obj_no;
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	for (/* nix */; (priv->tx_next - priv->tx_echo) > 0; priv->tx_echo++) {
		msg_obj_no = get_tx_echo_msg_obj(priv);
		c_can_inval_msg_object(dev, 0, msg_obj_no);
		val = c_can_read_reg32(priv, &priv->reg_base->txrqst1);
		if (!(val & (1 << msg_obj_no))) {
			can_get_echo_skb(dev,
					msg_obj_no - C_CAN_MSG_OBJ_TX_FIRST);
			stats->tx_bytes += priv->read_reg(priv,
					&priv->reg_base->ifreg[0].msg_cntrl)
					& IF_MCONT_DLC_MASK;
			stats->tx_packets++;
		}
	}

	/* restart queue if wrap-up or if queue stalled on last pkt */
	if (((priv->tx_next & C_CAN_NEXT_MSG_OBJ_MASK) != 0) ||
			((priv->tx_echo & C_CAN_NEXT_MSG_OBJ_MASK) == 0))
		netif_wake_queue(dev);
}

/*
 * theory of operation:
 *
 * c_can core saves a received CAN message into the first free message
 * object it finds free (starting with the lowest). Bits NEWDAT and
 * INTPND are set for this message object indicating that a new message
 * has arrived. To work-around this issue, we keep two groups of message
 * objects whose partitioning is defined by C_CAN_MSG_OBJ_RX_SPLIT.
 *
 * To ensure in-order frame reception we use the following
 * approach while re-activating a message object to receive further
 * frames:
 * - if the current message object number is lower than
 *   C_CAN_MSG_RX_LOW_LAST, do not clear the NEWDAT bit while clearing
 *   the INTPND bit.
 * - if the current message object number is equal to
 *   C_CAN_MSG_RX_LOW_LAST then clear the NEWDAT bit of all lower
 *   receive message objects.
 * - if the current message object number is greater than
 *   C_CAN_MSG_RX_LOW_LAST then clear the NEWDAT bit of
 *   only this message object.
 */
static int c_can_do_rx_poll(struct net_device *dev, int quota)
{
	u32 num_rx_pkts = 0;
	unsigned int msg_obj, msg_ctrl_save;
	struct c_can_priv *priv = netdev_priv(dev);
	u32 val = c_can_read_reg32(priv, &priv->reg_base->intpnd1);

	for (msg_obj = C_CAN_MSG_OBJ_RX_FIRST;
			msg_obj <= C_CAN_MSG_OBJ_RX_LAST && quota > 0;
			msg_obj++) {
		if (val & (1 << msg_obj)) {
			c_can_object_get(dev, 0, msg_obj, IF_COMM_ALL &
					~IF_COMM_TXRQST);
			msg_ctrl_save = priv->read_reg(priv,
					&priv->reg_base->ifreg[0].msg_cntrl);

			if (msg_ctrl_save & IF_MCONT_EOB)
				return num_rx_pkts;

			if (msg_ctrl_save & IF_MCONT_MSGLST) {
				c_can_handle_lost_msg_obj(dev, 0, msg_obj);
				num_rx_pkts++;
				quota--;
				continue;
			}

			if (!(msg_ctrl_save & IF_MCONT_NEWDAT))
				continue;

			/* read the data from the message object */
			c_can_read_msg_object(dev, 0, msg_ctrl_save, msg_obj);

			if (msg_obj < C_CAN_MSG_RX_LOW_LAST)
				c_can_mark_rx_msg_obj(dev, 0,
						msg_ctrl_save, msg_obj);
			else if (msg_obj > C_CAN_MSG_RX_LOW_LAST)
				/* activate this msg obj */
				c_can_activate_rx_msg_obj(dev, 0,
						msg_ctrl_save, msg_obj);
			else if (msg_obj == C_CAN_MSG_RX_LOW_LAST)
				/* activate all lower message objects */
				c_can_activate_all_lower_rx_msg_obj(dev,
						0, msg_ctrl_save);

			num_rx_pkts++;
			quota--;
		}
		val = c_can_read_reg32(priv, &priv->reg_base->intpnd1);
	}

	return num_rx_pkts;
}

static inline int c_can_has_and_handle_berr(struct c_can_priv *priv)
{
	return (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
		(priv->current_status & STATUS_LEC_MASK);
}

static int c_can_err(struct net_device *dev,
				enum c_can_bus_error_types error_type,
				enum c_can_lec_type lec_type)
{
	unsigned int reg_err_counter;
	unsigned int rx_err_passive;
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;

	/* propogate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	c_can_get_berr_counter(dev, &bec);
	reg_err_counter = priv->read_reg(priv, &priv->reg_base->error_counter);
	rx_err_passive = ((reg_err_counter & ERR_COUNTER_RP_MASK) >>
				ERR_COUNTER_RP_SHIFT);

	if (error_type & C_CAN_ERROR_WARNING) {
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		if (bec.rxerr > 96)
			cf->data[1] |= CAN_ERR_CRTL_RX_WARNING;
		if (bec.txerr > 96)
			cf->data[1] |= CAN_ERR_CRTL_TX_WARNING;
	}
	if (error_type & C_CAN_ERROR_PASSIVE) {
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		if (rx_err_passive)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
	}
	if (error_type & C_CAN_BUS_OFF) {
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		/* disable all interrupts in bus-off mode to ensure that
		 * the CPU is not hogged down
		 */
		c_can_enable_all_interrupts(priv, DISABLE_ALL_INTERRUPTS);
		can_bus_off(dev);
	}

	/*
	 * check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */
	switch (lec_type) {
		/* common for all type of bus errors */
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;
		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
		cf->data[2] |= CAN_ERR_PROT_UNSPEC;

	case LEC_STUFF_ERROR:
		dev_dbg(dev->dev.parent, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;

	case LEC_FORM_ERROR:
		dev_dbg(dev->dev.parent, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;

	case LEC_ACK_ERROR:
		dev_dbg(dev->dev.parent, "ack error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_ACK |
				CAN_ERR_PROT_LOC_ACK_DEL);
		break;

	case LEC_BIT1_ERROR:
		dev_dbg(dev->dev.parent, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;

	case LEC_BIT0_ERROR:
		dev_dbg(dev->dev.parent, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;

	case LEC_CRC_ERROR:
		dev_dbg(dev->dev.parent, "CRC error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_CRC_SEQ |
				CAN_ERR_PROT_LOC_CRC_DEL);
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int c_can_poll(struct napi_struct *napi, int quota)
{
	u16 irqstatus;
	int lec_type = 0;
	int work_done = 0;
	struct net_device *dev = napi->dev;
	struct c_can_priv *priv = netdev_priv(dev);
	enum c_can_bus_error_types error_type = C_CAN_NO_ERROR;

	irqstatus = priv->read_reg(priv, &priv->reg_base->ir);

	/* status events have the highest priority */
	if (irqstatus == STATUS_INTERRUPT) {
		priv->current_status = priv->read_reg(priv,
					&priv->reg_base->status);

		/* handle Tx/Rx events */
		if (priv->current_status & STATUS_TXOK)
			priv->write_reg(priv, &priv->reg_base->status,
					(priv->current_status & ~STATUS_TXOK));

		if (priv->current_status & STATUS_RXOK)
			priv->write_reg(priv, &priv->reg_base->status,
					(priv->current_status & ~STATUS_RXOK));

		/* handle bus error events */
		if (priv->current_status & STATUS_EWARN) {
			dev_dbg(dev->dev.parent,
					"entered error warning state\n");
			error_type = C_CAN_ERROR_WARNING;
		}
		if ((priv->current_status & STATUS_EPASS) &&
				(!(priv->last_status & STATUS_EPASS))) {
			dev_dbg(dev->dev.parent,
					"entered error passive state\n");
			error_type = C_CAN_ERROR_PASSIVE;
		}
		if ((priv->current_status & STATUS_BOFF) &&
				(!(priv->last_status & STATUS_BOFF))) {
			dev_dbg(dev->dev.parent,
					"entered bus off state\n");
			error_type = C_CAN_BUS_OFF;
		}

		/* handle bus recovery events */
		if ((!(priv->current_status & STATUS_EPASS)) &&
				(priv->last_status & STATUS_EPASS)) {
			dev_dbg(dev->dev.parent,
					"left error passive state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}
		if ((!(priv->current_status & STATUS_BOFF)) &&
				(priv->last_status & STATUS_BOFF)) {
			dev_dbg(dev->dev.parent,
					"left bus off state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}

		priv->last_status = priv->current_status;

		/* handle error on the bus */
		lec_type = c_can_has_and_handle_berr(priv);
		if (lec_type && (error_type != C_CAN_NO_ERROR))
			work_done += c_can_err(dev, error_type, lec_type);
	} else if ((irqstatus > C_CAN_MSG_OBJ_RX_FIRST) &&
			(irqstatus <= C_CAN_MSG_OBJ_RX_LAST)) {
		/* handle events corresponding to receive message objects */
		work_done += c_can_do_rx_poll(dev, (quota - work_done));
	} else if ((irqstatus > C_CAN_MSG_OBJ_TX_FIRST) &&
			(irqstatus <= C_CAN_MSG_OBJ_TX_LAST)) {
		/* handle events corresponding to transmit message objects */
		c_can_do_tx(dev);
	}

	if (work_done < quota) {
		napi_complete(napi);
		/* enable all IRQs */
		c_can_enable_all_interrupts(priv, ENABLE_ALL_INTERRUPTS);
	}

	return work_done;
}

static irqreturn_t c_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct c_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts and schedule the NAPI */
	c_can_enable_all_interrupts(priv, DISABLE_ALL_INTERRUPTS);
	napi_schedule(&priv->napi);

	return IRQ_HANDLED;
}

static int c_can_open(struct net_device *dev)
{
	int err;
	struct c_can_priv *priv = netdev_priv(dev);

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		dev_err(dev->dev.parent, "failed to open can device\n");
		return err;
	}

	/* register interrupt handler */
	err = request_irq(dev->irq, &c_can_isr, priv->irq_flags, dev->name,
				dev);
	if (err < 0) {
		dev_err(dev->dev.parent, "failed to attach interrupt\n");
		goto exit_irq_fail;
	}

	/* start the c_can controller */
	c_can_start(dev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

exit_irq_fail:
	close_candev(dev);
	return err;
}

static int c_can_close(struct net_device *dev)
{
	struct c_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	c_can_stop(dev);
	free_irq(dev->irq, dev);
	close_candev(dev);

	return 0;
}

struct net_device *alloc_c_can_dev(void)
{
	struct net_device *dev;
	struct c_can_priv *priv;

	dev = alloc_candev(sizeof(struct c_can_priv), C_CAN_MSG_OBJ_TX_NUM);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	netif_napi_add(dev, &priv->napi, c_can_poll, C_CAN_NAPI_WEIGHT);

	priv->dev = dev;
	priv->can.bittiming_const = &c_can_bittiming_const;
	priv->can.do_set_bittiming = c_can_set_bittiming;
	priv->can.do_set_mode = c_can_set_mode;
	priv->can.do_get_berr_counter = c_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_ONE_SHOT |
					CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_BERR_REPORTING;

	return dev;
}
EXPORT_SYMBOL_GPL(alloc_c_can_dev);

void free_c_can_dev(struct net_device *dev)
{
	free_candev(dev);
}
EXPORT_SYMBOL_GPL(free_c_can_dev);

static const struct net_device_ops c_can_netdev_ops = {
	.ndo_open = c_can_open,
	.ndo_stop = c_can_close,
	.ndo_start_xmit = c_can_start_xmit,
};

int register_c_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &c_can_netdev_ops;

	return register_candev(dev);
}
EXPORT_SYMBOL_GPL(register_c_can_dev);

void unregister_c_can_dev(struct net_device *dev)
{
	unregister_candev(dev);
}
EXPORT_SYMBOL_GPL(unregister_c_can_dev);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch C_CAN controller");
