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
 * (C) 2007 by Hans J. Koch <hjk@hansjkoch.de>
 * (C) 2008, 2009 by Marc Kleine-Budde <kernel@pengutronix.de>
 *
 * Bosch C_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch C_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/media/en/pdf/ipmodules_1/c_can/
 * users_manual_c_can.pdf
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * Note about RX message object configuration:
 * -------------------------------------------
 * An important aspect for receive message object configuration is that
 * the UMASK bit is set to 1, for all these objects, because we want to
 * receive messages arriving from any node in the CAN network. Otherwise
 * we would be limited to receiving message objects of only those
 * messages which have been sent by CAN nodes for which we have provided
 * the appropriate CAN identifier in the receive message object.
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
#include <linux/io.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#include "c_can.h"

/* control register */
#define CONTROL_TEST		BIT(7)
#define CONTROL_CCE		BIT(6)
#define CONTROL_DISABLE_AR	BIT(5)
#define CONTROL_ENABLE_AR	(0 << 5)
#define CONTROL_EIE		BIT(3)
#define CONTROL_SIE		BIT(2)
#define CONTROL_IE		BIT(1)
#define CONTROL_INIT		BIT(0)

/* test register */
#define TEST_RX			BIT(7)
#define TEST_TX1		BIT(6)
#define TEST_TX2		BIT(5)
#define TEST_LBACK		BIT(4)
#define TEST_SILENT		BIT(3)
#define TEST_BASIC		BIT(2)

/* status register */
#define STATUS_BOFF		BIT(7)
#define STATUS_EWARN		BIT(6)
#define STATUS_EPASS		BIT(5)
#define STATUS_RXOK		BIT(4)
#define STATUS_TXOK		BIT(3)

/* error counter register */
#define ERR_CNT_TEC_MASK	0xff
#define ERR_CNT_TEC_SHIFT	0
#define ERR_CNT_REC_SHIFT	8
#define ERR_CNT_REC_MASK	(0x7f << ERR_CNT_REC_SHIFT)
#define ERR_CNT_RP_SHIFT	15
#define ERR_CNT_RP_MASK		(0x1 << ERR_CNT_RP_SHIFT)

/* bit-timing register */
#define BTR_BRP_MASK		0x3f
#define BTR_BRP_SHIFT		0
#define BTR_SJW_SHIFT		6
#define BTR_SJW_MASK		(0x3 << BTR_SJW_SHIFT)
#define BTR_TSEG1_SHIFT		8
#define BTR_TSEG1_MASK		(0xf << BTR_TSEG1_SHIFT)
#define BTR_TSEG2_SHIFT		12
#define BTR_TSEG2_MASK		(0x7 << BTR_TSEG2_SHIFT)

/* brp extension register */
#define BRP_EXT_BRPE_MASK	0x0f
#define BRP_EXT_BRPE_SHIFT	0

/* IFx command request */
#define IF_COMR_BUSY		BIT(15)

/* IFx command mask */
#define IF_COMM_WR		BIT(7)
#define IF_COMM_MASK		BIT(6)
#define IF_COMM_ARB		BIT(5)
#define IF_COMM_CONTROL		BIT(4)
#define IF_COMM_CLR_INT_PND	BIT(3)
#define IF_COMM_TXRQST		BIT(2)
#define IF_COMM_DATAA		BIT(1)
#define IF_COMM_DATAB		BIT(0)
#define IF_COMM_ALL		(IF_COMM_MASK | IF_COMM_ARB | \
				IF_COMM_CONTROL | IF_COMM_TXRQST | \
				IF_COMM_DATAA | IF_COMM_DATAB)

/* IFx arbitration */
#define IF_ARB_MSGVAL		BIT(15)
#define IF_ARB_MSGXTD		BIT(14)
#define IF_ARB_TRANSMIT		BIT(13)

/* IFx message control */
#define IF_MCONT_NEWDAT		BIT(15)
#define IF_MCONT_MSGLST		BIT(14)
#define IF_MCONT_CLR_MSGLST	(0 << 14)
#define IF_MCONT_INTPND		BIT(13)
#define IF_MCONT_UMASK		BIT(12)
#define IF_MCONT_TXIE		BIT(11)
#define IF_MCONT_RXIE		BIT(10)
#define IF_MCONT_RMTEN		BIT(9)
#define IF_MCONT_TXRQST		BIT(8)
#define IF_MCONT_EOB		BIT(7)
#define IF_MCONT_DLC_MASK	0xf

/*
 * IFx register masks:
 * allow easy operation on 16-bit registers when the
 * argument is 32-bit instead
 */
#define IFX_WRITE_LOW_16BIT(x)	((x) & 0xFFFF)
#define IFX_WRITE_HIGH_16BIT(x)	(((x) & 0xFFFF0000) >> 16)

/* status interrupt */
#define STATUS_INTERRUPT	0x8000

/* global interrupt masks */
#define ENABLE_ALL_INTERRUPTS	1
#define DISABLE_ALL_INTERRUPTS	0

/* minimum timeout for checking BUSY status */
#define MIN_TIMEOUT_VALUE	6

/* c_can lec values */
enum c_can_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
	LEC_UNUSED,
};

/*
 * c_can error types:
 * Bus errors (BUS_OFF, ERROR_WARNING, ERROR_PASSIVE) are supported
 */
enum c_can_bus_error_types {
	C_CAN_NO_ERROR = 0,
	C_CAN_BUS_OFF,
	C_CAN_ERROR_WARNING,
	C_CAN_ERROR_PASSIVE,
};

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

static inline unsigned int get_msg_obj_rx_first(const struct c_can_priv *priv)
{
	return priv->devtype_data.rx_first;
}

static inline unsigned int get_msg_obj_rx_last(const struct c_can_priv *priv)
{
	return priv->devtype_data.rx_last;
}

static inline unsigned int get_msg_obj_rx_split(const struct c_can_priv *priv)
{
	return priv->devtype_data.rx_split;
}

static inline unsigned int get_msg_obj_rx_num(const struct c_can_priv *priv)
{
	return get_msg_obj_rx_last(priv) - get_msg_obj_rx_first(priv) + 1;
}

static inline unsigned int get_msg_obj_rx_low_last(
			const struct c_can_priv *priv)
{
	return get_msg_obj_rx_split(priv) - 1;
}

static inline unsigned int get_msg_obj_tx_num(const struct c_can_priv *priv)
{
	return priv->devtype_data.tx_num;
}

static inline unsigned int get_msg_obj_tx_first(const struct c_can_priv *priv)
{
	return get_msg_obj_rx_last(priv) + 1;
}

static inline unsigned int get_msg_obj_tx_last(const struct c_can_priv *priv)
{
	return get_msg_obj_tx_first(priv) + get_msg_obj_tx_num(priv) - 1;
}

static inline int get_total_msg_objs(const struct c_can_priv *priv)
{
	return priv->devtype_data.rx_last + priv->devtype_data.tx_num;
}

static inline int get_txrqst_mask(const struct c_can_priv *priv)
{
	return ((1 << get_msg_obj_tx_num(priv)) - 1) <<
					get_msg_obj_rx_num(priv);
}

static u32 c_can_read_reg32(struct c_can_priv *priv, void *reg)
{
	u32 val = priv->read_reg(priv, reg);
	val |= ((u32) priv->read_reg(priv, reg + 2)) << 16;
	return val;
}

static void c_can_enable_all_interrupts(struct c_can_priv *priv,
						int enable)
{
	unsigned int cntrl_save = priv->read_reg(priv,
						&priv->regs->control);

	if (enable)
		cntrl_save |= (CONTROL_SIE | CONTROL_EIE | CONTROL_IE);
	else
		cntrl_save &= ~(CONTROL_EIE | CONTROL_IE | CONTROL_SIE);

	priv->write_reg(priv, &priv->regs->control, cntrl_save);
}

static inline int c_can_msg_obj_is_busy(struct c_can_priv *priv, int iface)
{
	int count = MIN_TIMEOUT_VALUE;

	while (count && priv->read_reg(priv,
				&priv->regs->ifregs[iface].com_req) &
				IF_COMR_BUSY) {
		count--;
		udelay(1);
	}

	if (!count)
		return 1;

	return 0;
}

static inline void c_can_object_get(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct c_can_priv *priv = netdev_priv(dev);

	/*
	 * As per specs, after writting the message object number in the
	 * IF command request register the transfer b/w interface
	 * register and message RAM must be complete in 6 CAN-CLK
	 * period.
	 */
	priv->write_reg(priv, &priv->regs->ifregs[iface].com_mask,
			IFX_WRITE_LOW_16BIT(mask));
	priv->write_reg(priv, &priv->regs->ifregs[iface].com_req,
			IFX_WRITE_LOW_16BIT(objno));

	if (c_can_msg_obj_is_busy(priv, iface))
		netdev_err(dev, "timed out in object get\n");
}

static inline void c_can_object_put(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct c_can_priv *priv = netdev_priv(dev);

	/*
	 * As per specs, after writting the message object number in the
	 * IF command request register the transfer b/w interface
	 * register and message RAM must be complete in 6 CAN-CLK
	 * period.
	 */
	priv->write_reg(priv, &priv->regs->ifregs[iface].com_mask,
			(IF_COMM_WR | IFX_WRITE_LOW_16BIT(mask)));
	priv->write_reg(priv, &priv->regs->ifregs[iface].com_req,
			IFX_WRITE_LOW_16BIT(objno));

	if (c_can_msg_obj_is_busy(priv, iface))
		netdev_err(dev, "timed out in object put\n");
}

static void c_can_write_msg_object(struct net_device *dev,
			int iface, struct can_frame *frame, int objno)
{
	int i;
	u16 flags = 0;
	unsigned int id;
	struct c_can_priv *priv = netdev_priv(dev);

	if (!(frame->can_id & CAN_RTR_FLAG))
		flags |= IF_ARB_TRANSMIT;

	if (frame->can_id & CAN_EFF_FLAG) {
		id = frame->can_id & CAN_EFF_MASK;
		flags |= IF_ARB_MSGXTD;
	} else
		id = ((frame->can_id & CAN_SFF_MASK) << 18);

	flags |= IF_ARB_MSGVAL;

	priv->write_reg(priv, &priv->regs->ifregs[iface].arb1,
				IFX_WRITE_LOW_16BIT(id));
	priv->write_reg(priv, &priv->regs->ifregs[iface].arb2, flags |
				IFX_WRITE_HIGH_16BIT(id));

	for (i = 0; i < frame->can_dlc; i += 2) {
		priv->write_reg(priv, &priv->regs->ifregs[iface].data[i / 2],
				frame->data[i] | (frame->data[i + 1] << 8));
	}

	if (priv->is_quirk_required) {
		/*
		 * SPEAr HW bug fix:
		 * As all message objects must be programmed
		 * with UMASK bit set to 1, even TX objects must
		 * be programmed in the same fashion.
		 */

		/* enable interrupt for this message object */
		priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
				IF_MCONT_TXIE | IF_MCONT_TXRQST | IF_MCONT_EOB |
				IF_MCONT_UMASK | frame->can_dlc);
	} else {
		/* enable interrupt for this message object */
		priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
				IF_MCONT_TXIE | IF_MCONT_TXRQST | IF_MCONT_EOB |
				frame->can_dlc);
	}

	c_can_object_put(dev, iface, objno, IF_COMM_ALL);
}

static inline void c_can_mark_rx_msg_obj(struct net_device *dev,
						int iface, int ctrl_mask,
						int obj)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
			ctrl_mask & ~(IF_MCONT_MSGLST | IF_MCONT_INTPND));
	c_can_object_put(dev, iface, obj, IF_COMM_CONTROL);

}

static inline void c_can_activate_all_lower_rx_msg_obj(struct net_device *dev,
						int iface,
						int ctrl_mask)
{
	int i;
	struct c_can_priv *priv = netdev_priv(dev);

	for (i = get_msg_obj_rx_first(priv);
			i <= get_msg_obj_rx_low_last(priv); i++) {
		priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
				ctrl_mask & ~(IF_MCONT_MSGLST |
					IF_MCONT_INTPND | IF_MCONT_NEWDAT));
		c_can_object_put(dev, iface, i, IF_COMM_CONTROL);
	}
}

static inline void c_can_activate_rx_msg_obj(struct net_device *dev,
						int iface, int ctrl_mask,
						int obj)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
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

	netdev_err(dev, "msg lost in buffer %d\n", objno);

	c_can_object_get(dev, iface, objno, IF_COMM_ALL & ~IF_COMM_TXRQST);

	priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl,
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

static void dummy_read_msgobj_one(struct net_device *dev)
{
	int msg_ctrl_save, arb_save, can_id;
	int data_a1, data_a2, data_b1, data_b2;

	struct c_can_priv *priv = netdev_priv(dev);

	c_can_object_get(dev, 0, 1, IF_COMM_ALL &
			~IF_COMM_TXRQST);
	msg_ctrl_save = priv->read_reg(priv,
			&priv->regs->ifregs[0].msg_cntrl);
	arb_save = priv->read_reg(priv, &priv->regs->ifregs[0].arb1) |
		(priv->read_reg(priv, &priv->regs->ifregs[0].arb2)
		 << 16);
	can_id = (arb_save >> 18) & CAN_SFF_MASK;

	data_a1 = priv->read_reg(priv,
			&priv->regs->ifregs[0].data[0]);
	data_a2 = priv->read_reg(priv,
			&priv->regs->ifregs[0].data[1]);
	data_b1 = priv->read_reg(priv,
			&priv->regs->ifregs[0].data[2]);
	data_b2 = priv->read_reg(priv,
			&priv->regs->ifregs[0].data[3]);
}

static int c_can_read_msg_object(struct net_device *dev, int iface,
		int ctrl, int msg_no)
{
	u16 flags, data;
	int i;
	unsigned int val;
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	u8 data_first_read[8];
	int can_id_first_read;

	u8 data_second_read[8], data_third_read[8], data_fourth_read[8];
	int can_id_second_read, can_id_third_read, can_id_fourth_read;

	skb = alloc_can_skb(dev, &frame);
	if (!skb) {
		stats->rx_dropped++;
		return -ENOMEM;
	}

	frame->can_dlc = get_can_dlc(ctrl & 0x0F);

	/* 1st try for ID */
	flags =	priv->read_reg(priv, &priv->regs->ifregs[iface].arb2);
	val = priv->read_reg(priv, &priv->regs->ifregs[iface].arb1) |
		(flags << 16);

	if (flags & IF_ARB_MSGXTD)
		/* extended identifier */
		can_id_first_read = (val & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		/* standard identifier */
		can_id_first_read = (val >> 18) & CAN_SFF_MASK;

	/* 1st try for data */
	if (flags & IF_ARB_TRANSMIT)
		frame->can_id |= CAN_RTR_FLAG;
	else {
		for (i = 0; i < frame->can_dlc; i += 2) {
			data = priv->read_reg(priv,
					&priv->regs->ifregs[iface].data[i / 2]);
			data_first_read[i] = data;
			data_first_read[i + 1] = data >> 8;
		}
	}

	if (priv->is_quirk_required) {
		/*
		 * SPEAr HW bugfix:
		 * read message multiple times to ensure it is correct.
		 */

		/* 2nd try for ID */
		flags = 0;
		val = 0;
		c_can_object_get(dev, 0, msg_no, IF_COMM_ALL &
				~IF_COMM_TXRQST);
		flags =	priv->read_reg(priv, &priv->regs->ifregs[iface].arb2);
		val = priv->read_reg(priv, &priv->regs->ifregs[iface].arb1) |
			(flags << 16);

		if (flags & IF_ARB_MSGXTD)
			/* extended identifier */
			can_id_second_read = (val & CAN_EFF_MASK) |
						CAN_EFF_FLAG;
		else
			/* standard identifier */
			can_id_second_read = (val >> 18) & CAN_SFF_MASK;

		/* 2nd try for data */
		if (flags & IF_ARB_TRANSMIT)
			frame->can_id |= CAN_RTR_FLAG;
		else {
			for (i = 0; i < frame->can_dlc; i += 2) {
				data = priv->read_reg(priv,
					&priv->regs->ifregs[iface].data[i / 2]);
				data_second_read[i] = data;
				data_second_read[i + 1] = data >> 8;
			}
		}

		/* 3rd try for ID */
		flags = 0;
		val = 0;
		c_can_object_get(dev, 0, msg_no, IF_COMM_ALL &
				~IF_COMM_TXRQST);
		flags =	priv->read_reg(priv, &priv->regs->ifregs[iface].arb2);
		val = priv->read_reg(priv, &priv->regs->ifregs[iface].arb1) |
			(flags << 16);

		if (flags & IF_ARB_MSGXTD)
			/* extended identifier */
			can_id_third_read = (val & CAN_EFF_MASK) | CAN_EFF_FLAG;
		else
			/* standard identifier */
			can_id_third_read = (val >> 18) & CAN_SFF_MASK;

		/* 3rd try for data */
		if (flags & IF_ARB_TRANSMIT)
			frame->can_id |= CAN_RTR_FLAG;
		else {
			for (i = 0; i < frame->can_dlc; i += 2) {
				data = priv->read_reg(priv,
					&priv->regs->ifregs[iface].data[i / 2]);
				data_third_read[i] = data;
				data_third_read[i + 1] = data >> 8;
			}
		}

		if ((can_id_first_read != can_id_second_read) ||
				(can_id_second_read != can_id_third_read) ||
				(can_id_first_read != can_id_third_read) ||
				memcmp(data_first_read, data_second_read, 8) ||
				memcmp(data_second_read, data_third_read, 8) ||
				memcmp(data_first_read, data_third_read, 8)) {
			/* first handle the case of CAN-id */

			/*
			 * CAN ids read already don't match. In such a case
			 * try to read one more time (i.e. 4th time) and try to
			 * compare with the ID read 3rd time. If it matches,
			 * this is the best possible CAN id and if not, the
			 * best CAN id is any value which is same any of the
			 * 2 reads out of 4. If all such checks fail, fallback
			 * to the last read CAN id.
			 *
			 * Note: Always give priority to ID read later
			 */

			/* 4th try for ID */
			flags = 0;
			val = 0;
			c_can_object_get(dev, 0, msg_no, IF_COMM_ALL &
					~IF_COMM_TXRQST);
			flags =	priv->read_reg(priv,
					&priv->regs->ifregs[iface].arb2);
			val = priv->read_reg(priv,
					&priv->regs->ifregs[iface].arb1) |
				(flags << 16);

			if (flags & IF_ARB_MSGXTD)
				/* extended identifier */
				can_id_fourth_read = (val & CAN_EFF_MASK) |
					CAN_EFF_FLAG;
			else
				/* standard identifier */
				can_id_fourth_read = (val >> 18) & CAN_SFF_MASK;

			if (can_id_third_read == can_id_fourth_read)
				frame->can_id = can_id_fourth_read;
			else if (can_id_second_read == can_id_third_read)
				frame->can_id = can_id_third_read;
			else if (can_id_first_read == can_id_second_read)
				frame->can_id = can_id_second_read;
			else
				frame->can_id = can_id_fourth_read;

			/* now handle CAN data */

			/*
			 * CAN data read already don't match. In such a case
			 * try to read one more time (i.e. 4th time) and try to
			 * compare with the data read 3rd time. If it matches,
			 * this is the best possible CAN data and if not,
			 * the best CAN data is any value which is same any of
			 * the 2 reads out of 4. If all such checks fail,
			 * fallback to the last read CAN data.
			 *
			 * Note: Always give priority to data read later
			 */

			if (flags & IF_ARB_TRANSMIT)
				frame->can_id |= CAN_RTR_FLAG;
			else {
				/* 4th try for data */
				for (i = 0; i < frame->can_dlc; i += 2) {
					data = priv->read_reg(priv,
						&priv->regs->
						ifregs[iface].data[i / 2]);
					data_fourth_read[i] = data;
					data_fourth_read[i + 1] = data >> 8;
				}
			}

			if (!memcmp(data_third_read, data_fourth_read, 8))
				memcpy(frame->data, data_fourth_read, 8);
			else if (!memcmp(data_second_read, data_third_read, 8))
				memcpy(frame->data, data_third_read, 8);
			else if (!memcmp(data_first_read, data_second_read, 8))
				memcpy(frame->data, data_second_read, 8);
			else
				memcpy(frame->data, data_fourth_read, 8);
		} else {
			frame->can_id = can_id_third_read;
			memcpy(frame->data, data_third_read, 8);
		}
	} else {
		frame->can_id = can_id_first_read;
		memcpy(frame->data, data_first_read, 8);
	}

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

	priv->write_reg(priv, &priv->regs->ifregs[iface].mask1,
			IFX_WRITE_LOW_16BIT(mask));
	priv->write_reg(priv, &priv->regs->ifregs[iface].mask2,
			IFX_WRITE_HIGH_16BIT(mask));

	priv->write_reg(priv, &priv->regs->ifregs[iface].arb1,
			IFX_WRITE_LOW_16BIT(id));
	priv->write_reg(priv, &priv->regs->ifregs[iface].arb2,
			(IF_ARB_MSGVAL | IFX_WRITE_HIGH_16BIT(id)));

	priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl, mcont);
	c_can_object_put(dev, iface, objno, IF_COMM_ALL & ~IF_COMM_TXRQST);

	netdev_dbg(dev, "obj no:%d, msgval:0x%08x\n", objno,
			c_can_read_reg32(priv, &priv->regs->msgval1));
}

static void c_can_inval_msg_object(struct net_device *dev, int iface, int objno)
{
	struct c_can_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, &priv->regs->ifregs[iface].arb1, 0);
	priv->write_reg(priv, &priv->regs->ifregs[iface].arb2, 0);
	priv->write_reg(priv, &priv->regs->ifregs[iface].msg_cntrl, 0);

	c_can_object_put(dev, iface, objno, IF_COMM_ARB | IF_COMM_CONTROL);

	netdev_dbg(dev, "obj no:%d, msgval:0x%08x\n", objno,
			c_can_read_reg32(priv, &priv->regs->msgval1));
}

static inline int c_can_is_tx_obj_busy(struct c_can_priv *priv, int objno)
{
	int val = c_can_read_reg32(priv, &priv->regs->txrqst1);

	/*
	 * as transmission request register's bit n-1 corresponds to
	 * message object n, we need to handle the same properly.
	 */
	if (val & (1 << (objno - 1)))
		return 1;

	return 0;
}

static netdev_tx_t c_can_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	u32 msg_obj_no;
	struct c_can_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	msg_obj_no = priv->tx_next;

	/* check for wrap */
	if (priv->tx_next == get_msg_obj_tx_last(priv)) {
		if (c_can_read_reg32(priv, &priv->regs->txrqst1) &
				get_txrqst_mask(priv)) {
			if (priv->is_quirk_required) {
				netif_stop_queue(dev);
				priv->tx_next = get_msg_obj_tx_first(priv);
				return NETDEV_TX_BUSY;
			} else {
				netif_stop_queue(dev);
			}
		}
		priv->tx_next = get_msg_obj_tx_first(priv);
	} else {
		priv->tx_next++;
	}

	/* prepare message object for transmission */
	can_put_echo_skb(skb, dev, msg_obj_no - get_msg_obj_tx_first(priv));
	c_can_write_msg_object(dev, 0, frame, msg_obj_no);

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
	reg_btr = brp | (sjw << BTR_SJW_SHIFT) | (tseg1 << BTR_TSEG1_SHIFT) |
			(tseg2 << BTR_TSEG2_SHIFT);
	reg_brpe = brpe & BRP_EXT_BRPE_MASK;

	netdev_info(dev,
		"setting BTR=%04x BRPE=%04x\n", reg_btr, reg_brpe);

	ctrl_save = priv->read_reg(priv, &priv->regs->control);
	priv->write_reg(priv, &priv->regs->control,
			ctrl_save | CONTROL_CCE | CONTROL_INIT);
	priv->write_reg(priv, &priv->regs->btr, reg_btr);
	priv->write_reg(priv, &priv->regs->brp_ext, reg_brpe);
	priv->write_reg(priv, &priv->regs->control, ctrl_save);

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
	struct c_can_priv *priv = netdev_priv(dev);

	/* first invalidate all message objects */
	for (i = get_msg_obj_rx_first(priv); i <= get_total_msg_objs(priv); i++)
		c_can_inval_msg_object(dev, 0, i);

	/* setup receive message objects */
	for (i = get_msg_obj_rx_first(priv); i < get_msg_obj_rx_last(priv); i++)
		c_can_setup_receive_object(dev, 0, i, 0, 0,
			(IF_MCONT_RXIE | IF_MCONT_UMASK) & ~IF_MCONT_EOB);

	if (priv->is_quirk_required) {
		/*
		 * SPEAr HW bugfix:
		 * set the EoB bit for the 31st message object and mark it as
		 * busy
		 */
		c_can_setup_receive_object(dev, 0, get_msg_obj_rx_last(priv),
				0, 0, IF_MCONT_NEWDAT | IF_MCONT_EOB |
				IF_MCONT_RXIE | IF_MCONT_UMASK);

		/*
		 * SPEAr HW bugfix:
		 * perform dummy read operation on message object 1, as the last
		 * data on the bus is that of message object 31.
		 */
		dummy_read_msgobj_one(dev);
	} else {
		c_can_setup_receive_object(dev, 0, get_msg_obj_rx_last(priv),
				0, 0, IF_MCONT_EOB | IF_MCONT_RXIE |
				IF_MCONT_UMASK);
	}
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
		priv->write_reg(priv, &priv->regs->control,
				CONTROL_DISABLE_AR);
	else
		/* enable automatic retransmission */
		priv->write_reg(priv, &priv->regs->control,
				CONTROL_ENABLE_AR);

	if (priv->can.ctrlmode & (CAN_CTRLMODE_LISTENONLY &
					CAN_CTRLMODE_LOOPBACK)) {
		/* loopback + silent mode : useful for hot self-test */
		priv->write_reg(priv, &priv->regs->control, CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST);
		priv->write_reg(priv, &priv->regs->test,
				TEST_LBACK | TEST_SILENT);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* loopback mode : useful for self-test function */
		priv->write_reg(priv, &priv->regs->control, CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST);
		priv->write_reg(priv, &priv->regs->test, TEST_LBACK);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* silent mode : bus-monitoring mode */
		priv->write_reg(priv, &priv->regs->control, CONTROL_EIE |
				CONTROL_SIE | CONTROL_IE | CONTROL_TEST);
		priv->write_reg(priv, &priv->regs->test, TEST_SILENT);
	} else
		/* normal mode*/
		priv->write_reg(priv, &priv->regs->control,
				CONTROL_EIE | CONTROL_SIE | CONTROL_IE);

	/* configure message objects */
	c_can_configure_msg_objects(dev);

	/* set a `lec` value so that we can check for updates later */
	priv->write_reg(priv, &priv->regs->status, LEC_UNUSED);

	/* set bittiming params */
	c_can_set_bittiming(dev);
}

static void c_can_start(struct net_device *dev)
{
	struct c_can_priv *priv = netdev_priv(dev);

	/* basic c_can configuration */
	c_can_chip_config(dev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* reset status flags */
	priv->last_status = priv->current_status = 0;

	/* reset tx helper pointers */
	priv->tx_next = get_msg_obj_tx_first(priv);

	/* reset rx helper pointers */
	priv->rx_next = 0;
	priv->rx_flag = 0;

	/* enable status change, error and module interrupts */
	c_can_enable_all_interrupts(priv, ENABLE_ALL_INTERRUPTS);
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

	reg_err_counter = priv->read_reg(priv, &priv->regs->err_cnt);
	bec->rxerr = (reg_err_counter & ERR_CNT_REC_MASK) >>
				ERR_CNT_REC_SHIFT;
	bec->txerr = reg_err_counter & ERR_CNT_TEC_MASK;

	return 0;
}

static void c_can_do_tx(struct net_device *dev, u16 irqstatus)
{
	u32 val;
	u32 msg_obj_no;
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	msg_obj_no = irqstatus;
	val = c_can_read_reg32(priv, &priv->regs->txrqst1);
	if (!(val & (1 << (msg_obj_no - 1)))) {
		can_get_echo_skb(dev,
			msg_obj_no - get_msg_obj_tx_first(priv));
		stats->tx_bytes += priv->read_reg(priv,
				&priv->regs->ifregs[0].msg_cntrl)
				& IF_MCONT_DLC_MASK;
		stats->tx_packets++;
		c_can_inval_msg_object(dev, 0, msg_obj_no);
	}

	/* restart queue if wrap-up */
	if (irqstatus == get_msg_obj_tx_last(priv))
		netif_wake_queue(dev);
}

/*
 * theory of operation:
 *
 * c_can core saves a received CAN message into the first free message
 * object it finds free (starting with the lowest). Bits NEWDAT and
 * INTPND are set for this message object indicating that a new message
 * has arrived. To work-around this issue, we keep two groups of message
 * objects whose partitioning is defined by get_msg_obj_rx_split(priv).
 *
 * To ensure in-order frame reception we use the following
 * approach while re-activating a message object to receive further
 * frames:
 * - if the current message object number is lower than
 *   get_msg_obj_rx_low_last(priv), do not clear the NEWDAT bit while clearing
 *   the INTPND bit.
 * - if the current message object number is equal to
 *   get_msg_obj_rx_low_last(priv) then clear the NEWDAT bit of all lower
 *   receive message objects.
 * - if the current message object number is greater than
 *   get_msg_obj_rx_low_last(priv) then clear the NEWDAT bit of
 *   only this message object.
 */
static int c_can_do_rx_poll(struct net_device *dev, int quota)
{
	u32 num_rx_pkts = 0;
	int obj;
	unsigned int msg_ctrl_save;
	struct c_can_priv *priv = netdev_priv(dev);
	u32 val = c_can_read_reg32(priv, &priv->regs->intpnd1);
	u32 newdat = c_can_read_reg32(priv, &priv->regs->newdat1);
	const unsigned long *addr = (unsigned long *)&val;
	priv->rx_flag = 0;

again:
	for (obj = find_next_bit(addr, get_msg_obj_rx_last(priv),
				priv->rx_next);
			obj < get_msg_obj_rx_last(priv) && quota > 0;
			val = c_can_read_reg32(priv, &priv->regs->intpnd1),
			newdat = c_can_read_reg32(priv, &priv->regs->newdat1),
			obj = find_next_bit(addr, get_msg_obj_rx_last(priv),
				++priv->rx_next)) {
		priv->rx_flag = 1;

		c_can_object_get(dev, 0, obj + 1, IF_COMM_ALL &
				~IF_COMM_TXRQST);

		msg_ctrl_save = priv->read_reg(priv,
				&priv->regs->ifregs[0].msg_cntrl);

		if (msg_ctrl_save & IF_MCONT_EOB) {
			netdev_err(dev, "EoB object reached, "
					"msg obj=%d, intpnd=%x, newdat=%x\n",
					obj + 1, val, newdat);
			if (priv->is_quirk_required) {
				/*
				 * SPEAr HW bugfix:
				 * perform dummy read operation on message
				 * object 1, as the last data on the bus is
				 * that of message object 31. If message
				 * object 31 is not set for EoB though,
				 * this operation should not be required (TBD??)
				 */
				dummy_read_msgobj_one(dev);
			}

			return num_rx_pkts;
		}

		if (msg_ctrl_save & IF_MCONT_MSGLST) {
			c_can_handle_lost_msg_obj(dev, 0, obj + 1);
			num_rx_pkts++;
			quota--;
			continue;
		}

		if ((!(msg_ctrl_save & IF_MCONT_NEWDAT)) ||
				(!(msg_ctrl_save & IF_MCONT_INTPND))) {
			continue;
		}

		/* read the data from the message object */
		c_can_read_msg_object(dev, 0, msg_ctrl_save, obj + 1);

		if ((obj + 1) == get_msg_obj_rx_low_last(priv))
			/* activate all lower message objects */
			c_can_activate_all_lower_rx_msg_obj(dev,
					0, msg_ctrl_save);
		else if ((obj + 1) < get_msg_obj_rx_low_last(priv))
			c_can_mark_rx_msg_obj(dev, 0,
					msg_ctrl_save, obj + 1);
		else if ((obj + 1) > get_msg_obj_rx_low_last(priv))
			/* activate this msg obj */
			c_can_activate_rx_msg_obj(dev, 0,
					msg_ctrl_save, obj + 1);

		num_rx_pkts++;
		quota--;

	}

	/* upper group completed, look again in lower */
	if (priv->rx_next > get_msg_obj_rx_low_last(priv) &&
		quota > 0 && obj >= get_msg_obj_rx_last(priv) &&
		!(priv->rx_flag)) {
		priv->rx_next = 0;
		goto again;
	}

	/*
	 * special case of wrapping:
	 * no further packets are expected, as we have to wrap back to
	 * message object 1
	 */
	if (priv->rx_next == get_msg_obj_rx_low_last(priv) &&
		quota > 0 &&
		obj >= get_msg_obj_rx_last(priv) &&
		!(priv->rx_flag)) {
		priv->rx_next = 0;
		goto again;
	}

	/*
	 * corner case checking:
	 * insure we never go out of bounds
	 */
	if (priv->rx_next >= get_msg_obj_rx_last(priv) && quota > 0) {
		priv->rx_next = 0;
		goto again;
	}

	return num_rx_pkts;
}

static inline int c_can_has_and_handle_berr(struct c_can_priv *priv)
{
	return (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
		(priv->current_status & LEC_UNUSED);
}

static int c_can_handle_state_change(struct net_device *dev,
				enum c_can_bus_error_types error_type)
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
	reg_err_counter = priv->read_reg(priv, &priv->regs->err_cnt);
	rx_err_passive = (reg_err_counter & ERR_CNT_RP_MASK) >>
				ERR_CNT_RP_SHIFT;

	switch (error_type) {
	case C_CAN_ERROR_WARNING:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			CAN_ERR_CRTL_TX_WARNING :
			CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;

		break;
	case C_CAN_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		if (rx_err_passive)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case C_CAN_BUS_OFF:
		/* bus-off state */
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		/*
		 * disable all interrupts in bus-off mode to ensure that
		 * the CPU is not hogged down
		 */
		c_can_enable_all_interrupts(priv, DISABLE_ALL_INTERRUPTS);
		can_bus_off(dev);
		break;
	default:
		break;
	}

	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static int c_can_handle_bus_err(struct net_device *dev,
				enum c_can_lec_type lec_type)
{
	struct c_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	/*
	 * early exit if no lec update or no error.
	 * no lec update means that no CAN bus event has been detected
	 * since CPU wrote 0x7 value to status reg.
	 */
	if (lec_type == LEC_UNUSED || lec_type == LEC_NO_ERROR)
		return 0;

	/* propogate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/*
	 * check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */

	/* common for all type of bus errors */
	priv->can.can_stats.bus_error++;
	stats->rx_errors++;
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	cf->data[2] |= CAN_ERR_PROT_UNSPEC;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_dbg(dev, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_dbg(dev, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_dbg(dev, "ack error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_ACK |
				CAN_ERR_PROT_LOC_ACK_DEL);
		break;
	case LEC_BIT1_ERROR:
		netdev_dbg(dev, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_dbg(dev, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_dbg(dev, "CRC error\n");
		cf->data[2] |= (CAN_ERR_PROT_LOC_CRC_SEQ |
				CAN_ERR_PROT_LOC_CRC_DEL);
		break;
	default:
		break;
	}

	/* set a `lec` value so that we can check for updates later */
	priv->write_reg(priv, &priv->regs->status, LEC_UNUSED);

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

	irqstatus = priv->read_reg(priv, &priv->regs->interrupt);
	if (!irqstatus)
		goto end;

	/* status events have the highest priority */
	if (irqstatus == STATUS_INTERRUPT) {
		priv->current_status = priv->read_reg(priv,
					&priv->regs->status);

		/* handle Tx/Rx events */
		if (priv->current_status & STATUS_TXOK)
			priv->write_reg(priv, &priv->regs->status,
					priv->current_status & ~STATUS_TXOK);

		if (priv->current_status & STATUS_RXOK)
			priv->write_reg(priv, &priv->regs->status,
					priv->current_status & ~STATUS_RXOK);

		/* handle state changes */
		if ((priv->current_status & STATUS_EWARN) &&
				(!(priv->last_status & STATUS_EWARN))) {
			netdev_dbg(dev, "entered error warning state\n");
			work_done += c_can_handle_state_change(dev,
						C_CAN_ERROR_WARNING);
		}
		if ((priv->current_status & STATUS_EPASS) &&
				(!(priv->last_status & STATUS_EPASS))) {
			netdev_dbg(dev, "entered error passive state\n");
			work_done += c_can_handle_state_change(dev,
						C_CAN_ERROR_PASSIVE);
		}
		if ((priv->current_status & STATUS_BOFF) &&
				(!(priv->last_status & STATUS_BOFF))) {
			netdev_dbg(dev, "entered bus off state\n");
			work_done += c_can_handle_state_change(dev,
						C_CAN_BUS_OFF);
		}

		/* handle bus recovery events */
		if ((!(priv->current_status & STATUS_BOFF)) &&
				(priv->last_status & STATUS_BOFF)) {
			netdev_dbg(dev, "left bus off state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}
		if ((!(priv->current_status & STATUS_EPASS)) &&
				(priv->last_status & STATUS_EPASS)) {
			netdev_dbg(dev, "left error passive state\n");
			priv->can.state = CAN_STATE_ERROR_ACTIVE;
		}

		priv->last_status = priv->current_status;

		/* handle lec errors on the bus */
		lec_type = c_can_has_and_handle_berr(priv);
		if (lec_type)
			work_done += c_can_handle_bus_err(dev, lec_type);
	} else if ((irqstatus >= get_msg_obj_rx_first(priv)) &&
			(irqstatus <= get_msg_obj_rx_last(priv))) {
		/* handle events corresponding to receive message objects */
		work_done += c_can_do_rx_poll(dev, (quota - work_done));
	} else if ((irqstatus >= get_msg_obj_tx_first(priv)) &&
			(irqstatus <= get_msg_obj_tx_last(priv))) {
		/* handle events corresponding to transmit message objects */
		c_can_do_tx(dev, irqstatus);

		if (priv->is_quirk_required) {
			/*
			 * SPEAr HW bug fix:
			 * better to do a dummy read since we just finished a
			 * TX. The TX object has EoB bit set to 1 and if we
			 * immediately do a RX followed by a TX, the last data
			 * on the message read bus will be with EoB set.
			 * So, do a dummy read first.
			 */
			dummy_read_msgobj_one(dev);
		}
	}

end:
	if (work_done < quota) {
		napi_complete(napi);
		/* enable all IRQs */
		c_can_enable_all_interrupts(priv, ENABLE_ALL_INTERRUPTS);
	}

	return work_done;
}

static irqreturn_t c_can_isr(int irq, void *dev_id)
{
	u16 irqstatus;
	struct net_device *dev = (struct net_device *)dev_id;
	struct c_can_priv *priv = netdev_priv(dev);

	irqstatus = priv->read_reg(priv, &priv->regs->interrupt);
	if (!irqstatus)
		return IRQ_NONE;

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
		netdev_err(dev, "failed to open can device\n");
		return err;
	}

	/* register interrupt handler */
	err = request_irq(dev->irq, &c_can_isr, IRQF_SHARED, dev->name,
				dev);
	if (err < 0) {
		netdev_err(dev, "failed to request interrupt\n");
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

struct net_device *alloc_c_can_dev(unsigned int echo_count)
{
	struct net_device *dev;
	struct c_can_priv *priv;

	dev = alloc_candev(sizeof(struct c_can_priv), echo_count);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	netif_napi_add(dev, &priv->napi, c_can_poll, get_msg_obj_rx_num(priv));

	priv->dev = dev;
	priv->can.bittiming_const = &c_can_bittiming_const;
	priv->can.do_set_mode = c_can_set_mode;
	priv->can.do_get_berr_counter = c_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_ONE_SHOT |
					CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_BERR_REPORTING;
	priv->can_start = c_can_start;
	priv->can_stop = c_can_stop;

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
	struct c_can_priv *priv = netdev_priv(dev);

	/* disable all interrupts */
	c_can_enable_all_interrupts(priv, DISABLE_ALL_INTERRUPTS);

	unregister_candev(dev);
}
EXPORT_SYMBOL_GPL(unregister_c_can_dev);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch C_CAN controller");
