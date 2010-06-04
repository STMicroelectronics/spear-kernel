/*
 * drivers/net/can/bosch_ccan.c
 *
 * CAN bus driver for Bosch CCAN controller
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the CCAN driver originally written by:
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 * which can be viewed here:
 * http://svn.berlios.de/svnroot/repos/socketcan/trunk/kernel/2.6/
 * drivers/net/can/old/ccan/ccan.c
 *
 * Bosch CCAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch CCAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/pdf/Users_Manual_C_CAN.pdf
 *
 * TODO:
 * - BRP extension support to be added.
 * - Add support for IF2 in Basic mode.
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

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#include "bosch_ccan.h"

#define DRV_NAME "bosch_ccan"

static struct can_bittiming_const bosch_ccan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,		/* 6-bit BRP field */
	.brp_inc = 1,
};

static u32 bosch_ccan_read_reg32(struct net_device *dev, enum ccan_regs reg)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	u32 val = priv->read_reg(priv, reg);
	val |= ((u32) priv->read_reg(priv, reg + 2)) << 16;

	return val;
}

static void bosch_ccan_write_reg32(struct net_device *dev, enum ccan_regs reg,
					u32 val)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, reg, val & 0xffff);
	priv->write_reg(priv, reg + 2, val >> 16);
}

static inline void bosch_ccan_object_get(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, CAN_IF_COMM(iface), mask);
	priv->write_reg(priv, CAN_IF_COMR(iface), objno + 1);
	while (priv->read_reg(priv, CAN_IF_COMR(iface)) & IF_COMR_BUSY)
		dev_info(dev->dev.parent, "busy in object get\n");
}

static inline void bosch_ccan_object_put(struct net_device *dev,
					int iface, int objno, int mask)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	priv->write_reg(priv, CAN_IF_COMM(iface), IF_COMM_WR | mask);
	priv->write_reg(priv, CAN_IF_COMR(iface), objno + 1);
	while (priv->read_reg(priv, CAN_IF_COMR(iface)) & IF_COMR_BUSY)
		dev_info(dev->dev.parent, "busy in object put\n");
}

int bosch_ccan_write_object(struct net_device *dev,
			int iface, struct can_frame *frame, int objno)
{
	unsigned int val;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	if (frame->can_id & CAN_EFF_FLAG)
		val = IF_ARB_MSGXTD | (frame->can_id & CAN_EFF_MASK);
	else
		val = ((frame->can_id & CAN_SFF_MASK) << 18);

	if (!(frame->can_id & CAN_RTR_FLAG))
		val |= IF_ARB_TRANSMIT;

	val |= IF_ARB_MSGVAL;
	bosch_ccan_write_reg32(dev, CAN_IF_ARB(iface), val);

	memcpy(&val, &frame->data[0], 4);
	bosch_ccan_write_reg32(dev, CAN_IF_DATAA(iface), val);
	memcpy(&val, &frame->data[4], 4);
	bosch_ccan_write_reg32(dev, CAN_IF_DATAB(iface), val);
	priv->write_reg(priv, CAN_IF_MCONT(iface),
			IF_MCONT_TXIE | IF_MCONT_TXRQST | IF_MCONT_EOB |
			(frame->can_dlc & 0xf));

	bosch_ccan_object_put(dev, 0, objno, IF_COMM_ALL);

	return 0;
}

int bosch_ccan_read_object(struct net_device *dev, int iface, int objno)
{
	unsigned int val, ctrl, data;
	struct bosch_ccan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (skb == NULL) {
		dev_err(dev->dev.parent,
				"failed to allocate skb for read object\n");
		return -ENOMEM;
	}

	skb->dev = dev;
	bosch_ccan_object_get(dev, 0, objno, IF_COMM_ALL & ~IF_COMM_TXRQST);
	frame = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

	ctrl = priv->read_reg(priv, CAN_IF_MCONT(iface));
	if (ctrl & IF_MCONT_MSGLST) {
		stats->rx_errors++;
		dev_info(dev->dev.parent, "msg lost in buffer %d\n", objno);
	}

	frame->can_dlc = ctrl & 0xf;
	val = bosch_ccan_read_reg32(dev, CAN_IF_ARB(iface));
	data = bosch_ccan_read_reg32(dev, CAN_IF_DATAA(iface));
	memcpy(&frame->data[0], &data, 4);
	data = bosch_ccan_read_reg32(dev, CAN_IF_DATAB(iface));
	memcpy(&frame->data[4], &data, 4);

	if (val & IF_ARB_MSGXTD)
		frame->can_id = (val & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		frame->can_id = (val >> 18) & CAN_SFF_MASK;

	if (val & IF_ARB_TRANSMIT)
		frame->can_id |= CAN_RTR_FLAG;

	priv->write_reg(priv, CAN_IF_MCONT(iface), ctrl &
			~(IF_MCONT_MSGLST | IF_MCONT_INTPND | IF_MCONT_NEWDAT));

	bosch_ccan_object_put(dev, 0, objno, IF_COMM_CONTROL);

	skb->protocol = __constant_htons(ETH_P_CAN);
	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += frame->can_dlc;

	return 0;
}

static int bosch_ccan_setup_receive_object(struct net_device *dev, int iface,
					int objno, unsigned int mask,
					unsigned int id, unsigned int mcont)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	bosch_ccan_write_reg32(dev, CAN_IF_MASK(iface), mask);
	bosch_ccan_write_reg32(dev, CAN_IF_ARB(iface), IF_ARB_MSGVAL | id);

	priv->write_reg(priv, CAN_IF_MCONT(iface), mcont);
	bosch_ccan_object_put(dev, 0, objno, IF_COMM_ALL & ~IF_COMM_TXRQST);

	dev_dbg(dev->dev.parent, "obj no:%d, msgval:0x%08x\n", objno,
			bosch_ccan_read_reg32(dev, CAN_MSGVAL));

	return 0;
}

static int bosch_ccan_inval_object(struct net_device *dev, int iface, int objno)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	bosch_ccan_write_reg32(dev, CAN_IF_ARB(iface), 0);
	priv->write_reg(priv, CAN_IF_MCONT(iface), 0);
	bosch_ccan_object_put(dev, 0, objno, IF_COMM_ARB | IF_COMM_CONTROL);

	dev_dbg(dev->dev.parent, "obj no:%d, msgval:0x%08x\n", objno,
			bosch_ccan_read_reg32(dev, CAN_MSGVAL));

	return 0;
}

static netdev_tx_t bosch_ccan_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct net_device_stats *stats = &dev->stats;

	netif_stop_queue(dev);

	bosch_ccan_write_object(dev, 0, frame, priv->tx_object);
	priv->tx_object++;

	stats->tx_bytes += frame->can_dlc;
	dev->trans_start = jiffies;
	can_put_echo_skb(skb, dev, 0);

	return NETDEV_TX_OK;
}

static int bosch_ccan_set_bittiming(struct net_device *dev)
{
	unsigned int reg_timing, ctrl_save;
	u8 brp, sjw, tseg1, tseg2;
	struct bosch_ccan_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;

	reg_timing = (brp & BTR_BRP_MASK) |
		((sjw << BTR_SJW_SHIFT) & BTR_SJW_MASK) |
		((tseg1 << BTR_TSEG1_SHIFT) & BTR_TSEG1_MASK) |
		((tseg2 << BTR_TSEG2_SHIFT) & BTR_TSEG2_MASK);

	dev_dbg(dev->dev.parent, "brp = %d, sjw = %d, seg1 = %d, seg2 = %d\n",
			brp, sjw, tseg1, tseg2);
	dev_dbg(dev->dev.parent, "setting BTR to %04x\n", reg_timing);

	ctrl_save = priv->read_reg(priv, CAN_CONTROL);
	priv->write_reg(priv, CAN_CONTROL,
			ctrl_save | CONTROL_CCE | CONTROL_INIT);
	priv->write_reg(priv, CAN_BTR, reg_timing);
	priv->write_reg(priv, CAN_CONTROL, ctrl_save);

	return 0;
}

/*
 * Configure CCAN auto-retransmission:
 * CCAN provides means to disable automatic retransmission of
 * frames to allow CCAN to work within a TTCAN environment.
 * One must be careful about the different bevaior of TxRqst and
 * NewDat in the case automatic retransmssion is disabled.
 * See user guide document for details.
 */
static int bosch_ccan_auto_retransmission_config(struct net_device *dev,
					enum bosch_ccan_auto_tx_config mode)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	switch (mode) {
	case CCAN_ENABLE_AUTO_RE_TRANSMIT:
		priv->write_reg(priv, CAN_CONTROL, CONTROL_ENABLE_AR);
		break;
	case CCAN_DISABLE_AUTO_RE_TRANSMIT:
		priv->write_reg(priv, CAN_CONTROL, CONTROL_DISABLE_AR);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*
 * Configure CCAN operating mode
 */
static int bosch_ccan_set_operating_mode(struct net_device *dev,
				enum bosch_ccan_operating_mode mode)
{
	unsigned int cntrl_reg = 0;
	unsigned int test_reg = 0;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	switch (mode) {
	case CCAN_NORMAL_MODE:
		cntrl_reg = (CONTROL_EIE | CONTROL_SIE | CONTROL_IE);
		break;
	case CCAN_BASIC_MODE:
		/* basic mode : CCAN runs without the message RAM */
		cntrl_reg = (CONTROL_EIE | CONTROL_SIE |
				CONTROL_IE | CONTROL_TEST);
		test_reg = TEST_BASIC;
		break;
	case CCAN_LOOPBACK_MODE:
		/* loopback mode : useful for self-test function */
		cntrl_reg = (CONTROL_EIE | CONTROL_SIE |
				CONTROL_IE | CONTROL_TEST);
		test_reg = TEST_LBACK;
		break;
	case CCAN_LOOPBACK_WITH_SILENT_MODE:
		/* loopback + silent mode : useful for hot self-test */
		cntrl_reg = (CONTROL_EIE | CONTROL_SIE |
				CONTROL_IE | CONTROL_TEST);
		test_reg = (TEST_LBACK | TEST_SILENT);
		break;
	case CCAN_SILENT_MODE:
		/* silent mode : bus-monitoring mode */
		cntrl_reg = (CONTROL_EIE | CONTROL_SIE |
				CONTROL_IE | CONTROL_TEST);
		test_reg = TEST_SILENT;
		break;
	default:
		return -EOPNOTSUPP;
	}

	priv->write_reg(priv, CAN_CONTROL, cntrl_reg);

	/* set test mode only when we do not want NORMAL mode */
	if (test_reg)
		priv->write_reg(priv, CAN_TEST, test_reg);

	return 0;
}

/*
 * Configure CCAN message objects for Tx and Rx purposes:
 * CCAN provides a total of 32 message objects that can be configured
 * either for Tx or Rx purposes. This configuration may vary as per the
 * system design. Here by default 16 message objects are kept aside for
 * Tx purposes and 16 for Rx purposes. See user guide document for details.
 */
static int bosch_ccan_configure_msg_objects(struct net_device *dev)
{
	int i;

	/* setup message objects */
	for (i = 0; i <= MAX_OBJECT; i++)
		bosch_ccan_inval_object(dev, 0, i);

	for (i = MAX_TRANSMIT_OBJECT + 1; i < MAX_OBJECT; i++)
		bosch_ccan_setup_receive_object(dev, 0, i, 0, 0,
						IF_MCONT_RXIE | IF_MCONT_UMASK);

	bosch_ccan_setup_receive_object(dev, 0, MAX_OBJECT, 0, 0, IF_MCONT_EOB |
						IF_MCONT_RXIE | IF_MCONT_UMASK);

	return 0;
}

/*
 * Configure CCAN chip:
 * - enable/disable auto-retransmission
 * - set operating mode
 */
static int bosch_ccan_chip_config(struct net_device *dev)
{
	int err;

	/* enable automatic retransmission */
	err = bosch_ccan_auto_retransmission_config(dev,
					CCAN_ENABLE_AUTO_RE_TRANSMIT);
	if (err)
		return err;

	/* enable normal operating mode */
	err = bosch_ccan_set_operating_mode(dev, CCAN_NORMAL_MODE);
	if (err)
		return err;

	return 0;
}

static int bosch_ccan_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		bosch_ccan_configure_msg_objects(dev);
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
		dev_info(dev->dev.parent, "CAN_MODE_START requested\n");
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int bosch_ccan_get_state(const struct net_device *dev,
				enum can_state *state)
{
	u32 reg_status;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	reg_status = priv->read_reg(priv, CAN_STATUS);

	if (reg_status & STATUS_EPASS)
		*state = CAN_STATE_ERROR_PASSIVE;
	else if (reg_status & STATUS_EWARN)
		*state = CAN_STATE_ERROR_WARNING;
	else if (reg_status & STATUS_BOFF)
		*state = CAN_STATE_BUS_OFF;
	else
		*state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static int bosch_ccan_do_status_irq(struct net_device *dev)
{
	int status, diff;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	status = priv->read_reg(priv, CAN_STATUS);
	status &= ~(STATUS_TXOK | STATUS_RXOK);
	diff = status ^ priv->last_status;

	if (diff & STATUS_EPASS) {
		if (status & STATUS_EPASS)
			dev_info(dev->dev.parent,
					"entered error passive state\n");
		else
			dev_info(dev->dev.parent,
					"left error passive state\n");
	}
	if (diff & STATUS_EWARN) {
		if (status & STATUS_EWARN)
			dev_info(dev->dev.parent,
					"entered error warning state\n");
		else
			dev_info(dev->dev.parent,
					"left error warning state\n");
	}
	if (diff & STATUS_BOFF) {
		if (status & STATUS_BOFF)
			dev_info(dev->dev.parent, "entered busoff state\n");
		else
			dev_info(dev->dev.parent, "left busoff state\n");
	}

	if (diff & STATUS_LEC_MASK) {
		switch (status & STATUS_LEC_MASK) {
		case LEC_STUFF_ERROR:
			dev_info(dev->dev.parent, "suffing error\n");
			break;
		case LEC_FORM_ERROR:
			dev_info(dev->dev.parent, "form error\n");
			break;
		case LEC_ACK_ERROR:
			dev_info(dev->dev.parent, "ack error\n");
			break;
		case LEC_BIT1_ERROR:
			dev_info(dev->dev.parent, "bit1 error\n");
			break;
		case LEC_BIT0_ERROR:
			dev_info(dev->dev.parent, "bit0 error\n");
			break;
		case LEC_CRC_ERROR:
			dev_info(dev->dev.parent, "CRC error\n");
			break;
		}
	}

	priv->write_reg(priv, CAN_STATUS, 0);
	priv->last_status = status;

	return diff ? 1 : 0;
}

static void bosch_ccan_do_object_irq(struct net_device *dev, u16 irqstatus)
{
	int i;
	u32 val;
	struct net_device_stats *stats = &dev->stats;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	if (irqstatus > MAX_TRANSMIT_OBJECT) {
		val = bosch_ccan_read_reg32(dev, CAN_NEWDAT);
		while (val & RECEIVE_OBJECT_BITS) {
			for (i = MAX_TRANSMIT_OBJECT + 1; i <= MAX_OBJECT; i++)
				if (val & (1 << i))
					bosch_ccan_read_object(dev, 0, i);

			val = bosch_ccan_read_reg32(dev, CAN_NEWDAT);
		}
	} else {
		bosch_ccan_inval_object(dev, 0, irqstatus - 1);
		val = bosch_ccan_read_reg32(dev, CAN_TXRQST);
		if (!val) {
			can_get_echo_skb(dev, 0);
			priv->tx_object--;
			stats->tx_packets++;
			netif_wake_queue(dev);
		}
	}
}

static void do_statuspoll(struct work_struct *work)
{
	struct bosch_ccan_priv *priv = container_of(
						((struct delayed_work *)work),
						struct bosch_ccan_priv, work);

	priv->write_reg(priv, CAN_CONTROL,
			CONTROL_SIE | CONTROL_EIE | CONTROL_IE);
}

static irqreturn_t bosch_ccan_isr(int irq, void *dev_id)
{
	u16 irqstatus;
	unsigned int cntrl_save;
	struct net_device *dev = (struct net_device *)dev_id;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	irqstatus = priv->read_reg(priv, CAN_IR);

	if (!irqstatus)
		return IRQ_NONE;
	while (irqstatus) {
		if (irqstatus == 0x8000) {
			if (bosch_ccan_do_status_irq(dev)) {
				/*
				 * CCAN core tends to flood us with
				 * interrupts when certain error states don't
				 * disappear. Disable interrupts and see if it's
				 * getting better later.
				 */

				cntrl_save = priv->read_reg(priv, CAN_CONTROL);
				cntrl_save = ((cntrl_save & ~CONTROL_EIE) &
						~CONTROL_IE);
				priv->write_reg(priv, CAN_CONTROL, cntrl_save);

				schedule_delayed_work(&priv->work, HZ / 10);
				goto exit;
			}
		} else
			bosch_ccan_do_object_irq(dev, irqstatus);

		irqstatus = priv->read_reg(priv, CAN_IR);
	}

exit:
	return IRQ_HANDLED;
}

static int bosch_ccan_open(struct net_device *dev)
{
	int err;
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		dev_err(dev->dev.parent, "failed to open can device\n");
		return err;
	}

	/* register interrupt handler */
	err = request_irq(dev->irq, &bosch_ccan_isr, priv->irq_flags, dev->name,
				(void *)dev);
	if (err < 0) {
		dev_err(dev->dev.parent, "failed to attach interrupt\n");
		goto exit_irq_fail;
	}

	/* set the desired chip configuration */
	err = bosch_ccan_chip_config(dev);
	if (err)
		goto exit_config_fail;

	netif_start_queue(dev);

	return 0;

exit_config_fail:
	free_irq(dev->irq, dev);
exit_irq_fail:
	close_candev(dev);
	return err;
}

static int bosch_ccan_close(struct net_device *dev)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);

	cancel_delayed_work(&priv->work);
	flush_scheduled_work();

	/* mask all IRQs */
	priv->write_reg(priv, CAN_CONTROL, 0);

	free_irq(dev->irq, dev);
	close_candev(dev);

	return 0;
}

struct net_device *alloc_bosch_ccandev(int sizeof_priv)
{
	struct net_device *dev;
	struct bosch_ccan_priv *priv;

	dev = alloc_candev(sizeof(struct bosch_ccan_priv) + sizeof_priv);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &bosch_ccan_bittiming_const;
	priv->can.do_set_bittiming = bosch_ccan_set_bittiming;
	priv->can.do_get_state = bosch_ccan_get_state;
	priv->can.do_set_mode = bosch_ccan_set_mode;

	priv->tx_object = 0;

	return dev;
}
EXPORT_SYMBOL(alloc_bosch_ccandev);

void free_bosch_ccandev(struct net_device *dev)
{
	free_candev(dev);
}
EXPORT_SYMBOL(free_bosch_ccandev);

static const struct net_device_ops bosch_ccan_netdev_ops = {
	.ndo_open = bosch_ccan_open,
	.ndo_stop = bosch_ccan_close,
	.ndo_start_xmit = bosch_ccan_start_xmit,
};

int register_bosch_ccandev(struct net_device *dev)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	/* set the desired chip configuration */
	bosch_ccan_set_mode(dev, CAN_MODE_START);

	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &bosch_ccan_netdev_ops;

	INIT_DELAYED_WORK(&priv->work, do_statuspoll);

	return register_candev(dev);
}
EXPORT_SYMBOL(register_bosch_ccandev);

void unregister_bosch_ccandev(struct net_device *dev)
{
	struct bosch_ccan_priv *priv = netdev_priv(dev);

	bosch_ccan_set_mode(dev, CAN_MODE_START);

	cancel_delayed_work(&priv->work);
	flush_scheduled_work();

	unregister_candev(dev);
}
EXPORT_SYMBOL(unregister_bosch_ccandev);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN bus driver for Bosch CCAN controller");
