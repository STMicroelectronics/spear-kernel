/*
 * drivers/net/irda/dice_ir.c
 *
 * DICE Fast IrDA Controller Driver
 *
 * Copyright (C) 2011 ST Microelectronics
 * Amit Virdi <amit.virdi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/*
 * DICE Fast IrDA controller supports three basic modes of operations -
 * Serial InfraRed (SIR), Medium InfraRed (MIR) and Fast InfraRed (FIR).
 * This driver currently supports only SIR mode.
 *
 * Tested on arch/arm/mach-spear3xx
 */

#include <asm/byteorder.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <net/irda/irda.h>
#include <net/irda/irda_device.h>
#include <net/irda/irlap.h>
#include <net/irda/irlap_frame.h>

#define DRIVER_NAME "dice_ir"
/* #define DICE_IR_DEBUG */

#ifdef DICE_IR_DEBUG
unsigned int dbg_cntr;
unsigned int dbg_int[1024];
unsigned int dbg_byt_cntr[1024];
unsigned int dbg_rfs[1024];
unsigned int dbg_state[1024];
#endif

/* Various states in which the Fast IrDA Controller can be */
enum {
	IRDA_INACTIVE,
	IRDA_LISTENING,
	IRDA_RECEIVING,
	IRDA_TRANSMITTING,
};

/* Register Offsets */
/* Control and Status Registers */
#define IRDA_CTRL	0x10
	#define IRDA_ENABLE	0x1
	#define IRDA_DISABLE	0x0

#define IRDA_CONF	0x14
	#define RATV_40MHZ	0x00000C35
	#define RATV_48MHZ	0x00000EA6
	#define RATV_56MHZ	0x00001117
	#define BURST_SIZE_1W	0x00000000
	#define BURST_SIZE_2W	0x00010000
	#define BURST_SIZE_4W	0x00020000
	#define POLL_RX_HIGH	(1 << 19)

#define IRDA_PARAM	0x18
	#define FIRDA_MODE_SIR	0x0
	#define FIRDA_MODE_MIR	0x1
	#define FIRDA_MODE_FIR	0x2
	#define IRDA_MNRB_70	(0x046 << 16)
	#define IRDA_MNRB_134	(0x0A6 << 16)
	#define IRDA_MNRB_262	(0x106 << 16)
	#define IRDA_MNRB_518	(0x206 << 16)
	#define IRDA_MNRB_1030	(0x406 << 16)
	#define IRDA_MNRB_2054	(0x806 << 16)

#define IRDA_DIVD	0x1C
	#define FIRDA_DIV_VAL	0x00F70304

#define IRDA_STAT	0x20
	#define IRDA_STATE_TRANSMITTING	0x02
	#define IRDA_STATE_RECEIVING	0x01

#define IRDA_TFS	0x24
#define IRDA_RFS	0x28

/* Data Registers */
#define IRDA_TXB	0x2C
#define IRDA_RXB	0x30

/* Interrupt and DMA Registers */
#define IRDA_IMSC	0xE8
#define IRDA_RIS	0xEC
#define IRDA_MIS	0xF0
#define IRDA_ICR	0xF4
#define IRDA_ISR	0xF8
#define IRDA_DMA	0xFC

/* Register configuration for IRDA_DIVD
 *
 * FIRDA is to operate in SIR mode
 * So the K, L and N values from table 5 in the device data sheet are
 * K = 3, L = 250. Hence, DEC = L-K = 247, INC = K = 3
 */

#define K_SIR		3
#define L_SIR		250

#define	INC_VAL		K_SIR			/* INC = K */
#define DEC_VAL		(L_SIR - K_SIR)		/* DEC = L-K */

#define N_9600		59
#define N_19200		29
#define N_38400		14
#define N_57600		9
#define	N_115200	4

#define IRDA_DIV_9600	((DEC_VAL<<16) | (INC_VAL<<8) | N_9600)
#define IRDA_DIV_19200	((DEC_VAL<<16) | (INC_VAL<<8) | N_19200)
#define IRDA_DIV_38400	((DEC_VAL<<16) | (INC_VAL<<8) | N_38400)
#define IRDA_DIV_57600	((DEC_VAL<<16) | (INC_VAL<<8) | N_57600)
#define IRDA_DIV_115200	((DEC_VAL<<16) | (INC_VAL<<8) | N_115200)

/* DICE Fast IrDA Interrupts */
#define LSREQ_INTERRUPT		1	/* Last single request */
#define SREQ_INTERRUPT		(1<<1)	/* Single request */
#define LBREQ_INTERRUPT		(1<<2)	/* Last burst request */
#define BREQ_INTERRUPT		(1<<3)	/* Burst Request */
#define FT_INTERRUPT		(1<<4)	/* Frame Transmitted */
#define SD_INTERRUPT		(1<<5)	/* Signal Detected */
#define FI_INTERRUPT		(1<<6)	/* Frame Invalid */
#define FD_INTERRUPT		(1<<7)	/* Frame Detected */

#define SINGLE_WORD		1
#define BURST_SIZE		4
#define WORD_SIZE		4
#define MAX_RECEIVE_BUFF	2056

#define GET_WORD(buff, word) do { \
	int i; \
	for (i = 0; i < sizeof(word); i++) \
		word |= ((*(buff + i)) << (i * 8));\
} while (0)

#define PUT_WORD(buff, word) do { \
	int i; \
	for (i = 0; i < sizeof(word); i++) {\
		*(buff + i) = (word & 0xFF) ; \
		word >>= 8; \
	} \
} while (0)

struct divisor_reg_val {
	int baudrate;
	u32 div_reg_val;
};

static const struct divisor_reg_val irda_div_cfg[] = {
	{9600, IRDA_DIV_9600},
	{19200, IRDA_DIV_19200},
	{38400, IRDA_DIV_38400},
	{57600, IRDA_DIV_57600},
	{115200, IRDA_DIV_115200},
};

/*
 * DICE FIRDA dev structure
 * @qos: The QOS parameters
 * @slock: spinlock for the device
 * @vbase: virtual base of the device
 * @ndev: pointer to the net device structure
 * @clkl: pointer to the clock structure
 * @txskb: Transmit skb pointer
 * @rxskb: Receive skb pointer
 * @irlap: Pointer to main IrLAP structure
 * @speed: Current transmission speed (baudrate)
 * @newspeed: Transmission speed for the next skb
 * @suspended: Is the device in suspend mode?
 * @irq: irq number
 * @byte_counter: used to signify how many bytes have been received/transmitted
 * for the current frame
 * */
struct dice_irdev {
	struct qos_info		qos;
	spinlock_t		slock;
	void __iomem		*vbase;
	struct net_device	*ndev;
#ifdef CONFIG_HAVE_CLK
	struct clk		*clk;
#endif
	struct sk_buff		*txskb;
	struct sk_buff		*rxskb;
	struct irlap_cb		*irlap;
	int			speed;
	int			newspeed;
	int			suspended;
	u32			irq;
	u32			byte_counter;
};

#ifdef DICE_IR_DEBUG
static void dice_ir_dump_bytes(struct net_device *ndev, u8 *buffer, int length)
{
	int count, i;
	for (i = 0, count = 0; count < (length/4); count++, i += 4)
		dev_dbg(&ndev->dev , "%02x %02x %02x %02x ", \
			buffer[i], buffer[i+1], buffer[i+2], buffer[i+3]);

	if (length % 4) {
		for (; i < length; i++)
			dev_dbg(&ndev->dev , "%02x ", buffer[i]);
	}
}
#endif

static int dice_ir_set_baudrate(struct net_device *ndev, int speed)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	int index, ret = 0;
	u32 reg_val = IRDA_DIV_9600;
	u32 irda_state = readl(irdev->vbase + IRDA_STAT);

	/* The baudrate can be set only by disabling the device */
	if (!((IRDA_STATE_TRANSMITTING == irda_state) ||
				(IRDA_STATE_RECEIVING == irda_state))) {
		writel(IRDA_DISABLE, irdev->vbase + IRDA_CTRL);

		for (index = ARRAY_SIZE(irda_div_cfg)-1; index >= 0; index--) {
			if (irda_div_cfg[index].baudrate == speed)
				reg_val = irda_div_cfg[index].div_reg_val;
		}

#ifdef DICE_IR_DEBUG
		dev_dbg(&ndev->dev, "The new divider register value is 0x%x\n",
				reg_val);
#endif

		writel(reg_val, irdev->vbase + IRDA_DIVD);
		writel(IRDA_ENABLE, irdev->vbase + IRDA_CTRL);
	} else {
		dev_err(&ndev->dev, "%s: The device is currently in use, "\
				"try again later...\n", __func__);
		ret = -EAGAIN;
	}

	return ret;
}

static void dice_ir_send_irlap_frame(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	struct sk_buff *rxskb = irdev->rxskb;

	/* Feed it to IrLAP layer */
	rxskb->dev = ndev;
	skb_reset_mac_header(rxskb);
	rxskb->protocol = htons(ETH_P_IRDA);

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += irdev->rxskb->len;

#ifdef DICE_IR_DEBUG
	dev_dbg(&ndev->dev, "%s skb->len = %i\n", __func__, rxskb->len);
#endif
	netif_rx(rxskb);

	/* allocate a new rx skb */
	irdev->rxskb = dev_alloc_skb(MAX_RECEIVE_BUFF);
	if (!irdev->rxskb)
		BUG();

	irdev->byte_counter = 0;
}

static int dice_handle_frame_invalid(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	u32 irda_state;

	irda_state = readl(irdev->vbase + IRDA_STAT);

#ifdef DICE_IR_DEBUG
	dev_dbg(&ndev->dev, "%s state = %i (1->Rx 2->Tx)\n",
			__func__, irda_state);
#endif

	if (!(readl(irdev->vbase + IRDA_CTRL))) {
		/* Device is disabled - do nothing*/
		return 1;
	}

	if (IRDA_STATE_TRANSMITTING != irda_state ||
			IRDA_STATE_RECEIVING != irda_state) {
		irdev->byte_counter = 0;

		/* Free the skb */
		if (irdev->txskb) {
			dev_kfree_skb(irdev->txskb);
			irdev->txskb = NULL;
		}

		ndev->stats.rx_dropped++;
		ndev->stats.rx_errors++;
#ifdef DICE_IR_DEBUG
		dev_dbg(&ndev->dev, "%s Invalid Frame\n", __func__);
#endif
		return 1;
	} else {
		/*
		 * The device goes back to listening/transmitting state.
		 * Do nothing
		 */
		irdev->byte_counter = 0;
		return 0;
	}
}

static void dice_handle_data_req(struct net_device *ndev, int num_words)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	struct sk_buff *skb;
	int count = 0;
	u32 irda_state = readl(irdev->vbase + IRDA_STAT);

	if (IRDA_STATE_TRANSMITTING == irda_state) {
		skb = irdev->txskb;

		for (count = 0; count < num_words; count++) {
			u32 word = 0;
			GET_WORD(skb->data + irdev->byte_counter, word);
			writel(word, irdev->vbase + IRDA_TXB);
			irdev->byte_counter += WORD_SIZE;
		}

	} else if (IRDA_STATE_RECEIVING == irda_state) {
		skb = irdev->rxskb;

		for (count = 0; count < num_words; count++) {
			u32 word = readl(irdev->vbase + IRDA_RXB);
			PUT_WORD(skb->data + irdev->byte_counter, word);
			irdev->byte_counter += WORD_SIZE;
		}
	} else
		dev_err(&ndev->dev, "%s:irda_state = %i\n", \
				__func__, irda_state);
}

static void dice_handle_last_burst_req(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	struct sk_buff *skb;
	int count = 0;
	u32 word, rfs;
	u32 irda_state = readl(irdev->vbase + IRDA_STAT);

	if (IRDA_STATE_TRANSMITTING == irda_state) {
		skb = irdev->txskb;

		if (!skb) {
			dev_dbg(&ndev->dev, "%s: SKB NULL\n", __func__);
			return;
		}

		while (skb->len > irdev->byte_counter) {
			/* Write a burst in the FIFO */
			for (count = 0; count < BURST_SIZE; count++) {
				word = 0;
				GET_WORD(skb->data + irdev->byte_counter, word);
				writel(word, irdev->vbase + IRDA_TXB);
				irdev->byte_counter += WORD_SIZE;
			}
		}
	} else if (IRDA_STATE_RECEIVING == irda_state) {
		rfs = readl(irdev->vbase + IRDA_RFS);
		skb = irdev->rxskb;

		while (rfs > irdev->byte_counter) {
			/* Read a BURST */
			for (count = 0; count < BURST_SIZE; count++) {
				word = readl(irdev->vbase + IRDA_RXB);
				PUT_WORD(skb->data + irdev->byte_counter, word);
				irdev->byte_counter += WORD_SIZE;
			}
		}

		/* Correct the byte_counter offset */
		irdev->byte_counter -= WORD_SIZE;
		irdev->byte_counter += rfs - irdev->byte_counter;

		irdev->byte_counter -= 2; /*CRC Bytes*/
		skb_put(skb, irdev->byte_counter);

#ifdef DICE_IR_DEBUG
		dev_dbg(&ndev->dev, "%s: Read %i bytes\n", \
				__func__, irdev->byte_counter);
		dice_ir_dump_bytes(ndev, skb->data, irdev->byte_counter);
#endif
		dice_ir_send_irlap_frame(ndev);

		/* Enable the queue again! */
		netif_wake_queue(ndev);
	}
}

static void dice_handle_last_single_req(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	struct sk_buff *skb;
	int num_words = 1;
	u32 rfs = 0;
	u32 irda_state = readl(irdev->vbase + IRDA_STAT);

	if (IRDA_STATE_TRANSMITTING == irda_state) {
		u32 word;
		skb = irdev->txskb;

		if (!skb) {
			dev_dbg(&ndev->dev, "%s: SKB NULL\n", __func__);
			return;
		}

		if (skb->len - irdev->byte_counter > WORD_SIZE) {
			if ((skb->len - irdev->byte_counter) % WORD_SIZE)
				num_words = (skb->len - irdev->byte_counter) /
					WORD_SIZE;
			else
				num_words = ((skb->len - irdev->byte_counter) /
					WORD_SIZE) + 1;
		}

		while (num_words) {
			word = 0;
			GET_WORD(skb->data + irdev->byte_counter, word);
			writel(word, irdev->vbase + IRDA_TXB);
			irdev->byte_counter += WORD_SIZE;
			num_words--;
		}

		/* Correct the byte counter offset */
		irdev->byte_counter = skb->len;

	} else {
		u32 word = 0;
		skb = irdev->rxskb;
		rfs = readl(irdev->vbase + IRDA_RFS);

		/* Check how many bytes actually are to be read */
		if (rfs - irdev->byte_counter > WORD_SIZE) {
			if ((rfs - irdev->byte_counter) % WORD_SIZE)
				num_words = (rfs - irdev->byte_counter) /
					WORD_SIZE;
			else
				num_words = ((rfs - irdev->byte_counter) /
					WORD_SIZE) + 1;
		}

		while (num_words) {
			word = readl(irdev->vbase + IRDA_RXB);
			PUT_WORD(skb->data + irdev->byte_counter, word);
			irdev->byte_counter += WORD_SIZE;
			num_words--;
		}

		dev_dbg(&ndev->dev, "byte_counter %u rfs = %u\n", \
				irdev->byte_counter, rfs);
		/* Correct the byte_counter offset */
		irdev->byte_counter -= WORD_SIZE;
		irdev->byte_counter += rfs - irdev->byte_counter;

		irdev->byte_counter -= 2; /*CRC Bytes*/
		skb_put(skb, irdev->byte_counter);

#ifdef DICE_IR_DEBUG
		dev_dbg(&ndev->dev, "%s: Read %i bytes\n", \
				__func__, irdev->byte_counter);
		dice_ir_dump_bytes(ndev, skb->data, irdev->byte_counter);
#endif
		dice_ir_send_irlap_frame(ndev);

		/* Enable the queue again! */
		netif_wake_queue(ndev);
	}
}

static void dice_handle_frame_transmitted(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	u32 status_reg;

	/* Update the statistics */
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += irdev->txskb->len;

	status_reg = readl(irdev->vbase + IRDA_STAT);
	if (status_reg)
		dev_err(&ndev->dev, "%s Status Register should have been 0x0."\
				" Current value %u\n", \
				__func__, status_reg);

	if (irdev->newspeed) {
		dice_ir_set_baudrate(ndev, irdev->newspeed);
		irdev->speed = irdev->newspeed;
		irdev->newspeed = 0;
	}
}

static void dice_handle_signal_detect(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	u32 status_reg;

	status_reg = readl(irdev->vbase + IRDA_STAT);
	if (!(status_reg & IRDA_STATE_RECEIVING)) {
		/* The device returned from receiving state */
		return;
	}

	/* Stop the queue since the device is in reception state */
	netif_stop_queue(ndev);
}

static irqreturn_t dice_irda_isr(int irq, void *dev_id)
{
	u16 irqstatus;
	unsigned long flags;
	struct net_device *ndev = (struct net_device *)dev_id;
	struct dice_irdev *irdev = netdev_priv(ndev);

	spin_lock_irqsave(&irdev->slock, flags);

	while ((irqstatus = readl(irdev->vbase + IRDA_RIS))) {
#ifdef DICE_IR_DEBUG
		if (dbg_cntr <= 1023) {
			dbg_int[dbg_cntr] = irqstatus;
			dbg_byt_cntr[dbg_cntr] = irdev->byte_counter;
			dbg_state[dbg_cntr] = readl(irdev->vbase + IRDA_STAT);
			dbg_rfs[dbg_cntr] = readl(irdev->vbase + IRDA_RFS);
			dbg_cntr++;
		}
#endif
		if (irqstatus & FI_INTERRUPT) {
			if (dice_handle_frame_invalid(ndev)) {
				writel(irqstatus, irdev->vbase + IRDA_ICR);
				spin_unlock_irqrestore(&irdev->slock, flags);
				return IRQ_HANDLED;
			}
		}

		if (irqstatus & FT_INTERRUPT) {
			dice_handle_frame_transmitted(ndev);
#ifdef DICE_IR_DEBUG
			dev_dbg(&ndev->dev, "Frame Transmitted\n");
#endif

			/* Free the skb */
			if (irdev->txskb) {
				dev_kfree_skb(irdev->txskb);
				irdev->txskb = NULL;
			}

			/* Enable the queue again! */
			netif_wake_queue(ndev);
		}

		if (irqstatus & SD_INTERRUPT)
			dice_handle_signal_detect(ndev);

		if (irqstatus & FD_INTERRUPT)
			irdev->byte_counter = 0;

		if (irqstatus & BREQ_INTERRUPT)
			dice_handle_data_req(ndev, BURST_SIZE);

		if (irqstatus & SREQ_INTERRUPT)
			dice_handle_data_req(ndev, SINGLE_WORD);

		if (irqstatus & LBREQ_INTERRUPT)
			dice_handle_last_burst_req(ndev);

		if (irqstatus & LSREQ_INTERRUPT)
			dice_handle_last_single_req(ndev);

		/* Clear Interrupts */
		writel(irqstatus, irdev->vbase + IRDA_ICR);

	}
	spin_unlock_irqrestore(&irdev->slock, flags);
	return IRQ_HANDLED;
}

static int dice_ir_open(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	int err = 0;
	struct sk_buff *rxskb = NULL;

#ifdef DICE_IR_DEBUG
	dev_info(&ndev->dev, "%s\n.", __func__);
#endif
	irdev->newspeed = 0;
	irdev->speed = IR_9600;

	irdev->irlap = irlap_open(ndev, &irdev->qos, DRIVER_NAME);
	if (!irdev->irlap)
		return -ENOMEM;

#ifdef DICE_IR_DEBUG
	memset(dbg_int, 0xff, sizeof(dbg_int));
	memset(dbg_byt_cntr, 0xff, sizeof(dbg_byt_cntr));
	memset(dbg_rfs, 0xff, sizeof(dbg_rfs));
	memset(dbg_state, 0xff, sizeof(dbg_state));
#endif

	netif_start_queue(ndev);

	/* Allocate receive skb */
	rxskb = dev_alloc_skb(MAX_RECEIVE_BUFF);
	if (!rxskb)
		goto err_stop_queue;

	rxskb->len = 0;

	irdev->byte_counter = 0;
	irdev->rxskb = rxskb;
	irdev->txskb = NULL;

	/* Enable the device */
	writel(IRDA_ENABLE, irdev->vbase + IRDA_CTRL);

	return err;

err_stop_queue:
	netif_stop_queue(ndev);
	return err;
}

static int dice_ir_stop(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);

#ifdef DICE_IR_DEBUG
	dev_info(&ndev->dev, "%s\n.", __func__);
#endif
	writel(IRDA_DISABLE, irdev->vbase + IRDA_CTRL);

	/* Stop IrLAP */
	if (irdev->irlap) {
		irlap_close(irdev->irlap);
		irdev->irlap = NULL;
	}

	netif_stop_queue(ndev);

	if (irdev->txskb) {
		dev_kfree_skb(irdev->txskb);
		irdev->txskb = NULL;
	}

	if (irdev->rxskb) {
		dev_kfree_skb(irdev->rxskb);
		irdev->rxskb = NULL;
	}

	return 0;
}

static int dice_ir_hard_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	int speed = irda_get_next_speed(skb);
	unsigned long flags;
	u32 irda_state;

	if (0 == skb->len) {
		dev_dbg(&ndev->dev, "%s: SKB len = %i\n", __func__, skb->len);

		/* Bug on the stack side.
		 * Workaround: Free the skb and return TX_OK so that this skb
		 * is not queued again. */
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	netif_stop_queue(ndev);
	irdev->txskb = skb;

#ifdef DICE_IR_DEBUG
	dev_info(&ndev->dev, "%s TFS = %i\n", __func__, skb->len);
	dice_ir_dump_bytes(ndev, skb->data, skb->len);
#endif

	irda_state = readl(irdev->vbase + IRDA_STAT);
	if (irda_state) {
#ifdef DICE_IR_DEBUG
		dev_info(&ndev->dev, "Cannot start transmission. "\
				"Device state %u\n", irda_state);
		dev_info(&ndev->dev, "Try after sometime.\n");
#endif
		irdev->txskb = NULL;

		/* Enable the queue ! */
		netif_wake_queue(ndev);

		/* Return TX_BUSY so that the packet is queued again for
		 * transmission */
		return NETDEV_TX_BUSY;
	}

	spin_lock_irqsave(&irdev->slock, flags);
	writel((skb->len - 1) , irdev->vbase + IRDA_TFS);
	irdev->byte_counter = 0;
	ndev->trans_start = jiffies;
	spin_unlock_irqrestore(&irdev->slock, flags);

	if (speed != irdev->speed && speed != -1)
		irdev->newspeed = speed;

	return NETDEV_TX_OK;
}

static int dice_ir_ioctl(struct net_device *ndev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct dice_irdev *irdev = netdev_priv(ndev);
	int ret = -EPERM;
	unsigned long flags;
#ifdef DICE_IR_DEBUG
	u32 irqstatus;
#endif

	spin_lock_irqsave(&irdev->slock, flags);

	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN))
			ret = dice_ir_set_baudrate(ndev, rq->ifr_baudrate);
		break;

	case SIOCSMEDIABUSY:
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(ndev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		rq->ifr_receiving = (IRDA_STATE_RECEIVING & readl(irdev->vbase
					+ IRDA_STAT));
#ifdef DICE_IR_DEBUG
		irqstatus = readl(irdev->vbase + IRDA_MIS);
		dev_dbg(&ndev->dev, "rq->ifr_receiving = %i, irqstatus = %u\n",
				rq->ifr_receiving, irqstatus);
#endif
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	spin_unlock_irqrestore(&irdev->slock, flags);
	return ret;
}

static struct net_device_stats *dice_ir_stats(struct net_device *dev)
{
	struct dice_irdev *irdev = netdev_priv(dev);

	return &irdev->ndev->stats;
}

static const struct net_device_ops dice_ir_ndo = {
	.ndo_open		= dice_ir_open,
	.ndo_stop		= dice_ir_stop,
	.ndo_start_xmit		= dice_ir_hard_xmit,
	.ndo_do_ioctl		= dice_ir_ioctl,
	.ndo_get_stats		= dice_ir_stats,
};

static int dice_irda_init(struct net_device *ndev)
{
	struct dice_irdev *irdev = netdev_priv(ndev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&irdev->slock, flags);
#ifdef CONFIG_HAVE_CLK
	ret = clk_enable(irdev->clk);
	if (ret) {
		dev_err(&ndev->dev, "clock enable failed");
		spin_unlock_irqrestore(&irdev->slock, flags);
		return ret;
	}
#endif
	/* Configuration register */
	writel(BURST_SIZE_4W | RATV_48MHZ | POLL_RX_HIGH ,
			irdev->vbase + IRDA_CONF);

	/* Parameter register */
	writel(FIRDA_MODE_SIR | IRDA_MNRB_2054, irdev->vbase + IRDA_PARAM);

	/* Divider register */
	/* Default initialized - Discovery Protocol is initiated at 9600bps */
	writel(IRDA_DIV_9600, irdev->vbase + IRDA_DIVD);

	/* Enable the relevant interrupts on IRDA_IMSC */
	writel(LSREQ_INTERRUPT | LBREQ_INTERRUPT | BREQ_INTERRUPT |
			SREQ_INTERRUPT | FT_INTERRUPT | SD_INTERRUPT |
			FI_INTERRUPT | FD_INTERRUPT,
			irdev->vbase + IRDA_IMSC);

	spin_unlock_irqrestore(&irdev->slock, flags);
	return ret;
}

static int __devinit dice_ir_probe(struct platform_device *pdev)
{
	struct dice_irdev *irdev;
	struct net_device *ndev;
	struct resource *mem;
	int irq, ret, err = -ENOMEM;
	u32 baudrate_mask;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq < 0) {
		dev_err(&pdev->dev, "Not enough platform resources.\n");
		ret = -ENXIO;
		goto exit;
	}

	ndev = alloc_irdadev(sizeof(*irdev));
	if (!ndev) {
		ret = -ENOMEM;
		goto exit;
	}

	irdev = netdev_priv(ndev);

	if (!devm_request_mem_region(&pdev->dev, mem->start, resource_size(mem),
				DRIVER_NAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -ENXIO;
		goto err_free_dev;
	}

	irdev->vbase = devm_ioremap_nocache(&pdev->dev, mem->start,
			resource_size(mem));
	if (!irdev->vbase) {
		dev_err(&pdev->dev, "Unable to map irda device\n");
		ret = -ENXIO;
		goto err_free_dev;
	}

	ndev->irq = irq;
	err = request_irq(ndev->irq, &dice_irda_isr, IRQF_TRIGGER_NONE,
			ndev->name, ndev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		ret = err;
		goto err_free_dev;
	}

#ifdef CONFIG_HAVE_CLK
	irdev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(irdev->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		ret = PTR_ERR(irdev->clk);
		goto err_free_irq;
	}
#endif

	irda_init_max_qos_capabilies(&irdev->qos);

	ndev->netdev_ops = &dice_ir_ndo;
	irdev->ndev = ndev;

	baudrate_mask = IR_9600;
	irdev->qos.baud_rate.bits &= baudrate_mask;
	irdev->qos.min_turn_time.bits = 1; /* 10 ms or more */
	irda_qos_bits_to_value(&irdev->qos);

	spin_lock_init(&irdev->slock);

	ret = dice_irda_init(irdev->ndev);
	if (ret)
		goto err_free_clk;

	ret = register_netdev(ndev);
	if (ret)
		goto err_free_clk;

	platform_set_drvdata(pdev, ndev);

	dev_info(&pdev->dev, "DICE Fast IrDA probed successfully\n");

	goto exit;

err_free_clk:
#ifdef CONFIG_HAVE_CLK
	clk_put(irdev->clk);
#endif
err_free_irq:
	free_irq(ndev->irq, ndev);
err_free_dev:
	free_netdev(ndev);
exit:
	return ret;
}

static int __devexit dice_ir_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct dice_irdev *irdev = netdev_priv(ndev);
	unsigned long flags;

	if (!irdev)
		return 0;

	/* Disable the device */
	spin_lock_irqsave(&irdev->slock, flags);
	writel(IRDA_DISABLE, irdev->vbase + IRDA_CTRL);
	spin_unlock_irqrestore(&irdev->slock, flags);

	free_irq(ndev->irq, ndev);
	unregister_netdev(ndev);

#ifdef CONFIG_HAVE_CLK
	/* Disable and free the clock */
	clk_disable(irdev->clk);
	clk_put(irdev->clk);
#endif

	if (irdev->txskb)
		dev_kfree_skb(irdev->txskb);
	if (irdev->rxskb)
		dev_kfree_skb(irdev->rxskb);

	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int dice_ir_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct dice_irdev *irdev = netdev_priv(ndev);

	dev_info(&pdev->dev, "%s, Waking\n", DRIVER_NAME);

	if (!irdev->suspended)
		return 0;

	dice_ir_open(ndev);
	irdev->suspended = 0;
	return 0;
}

static int dice_ir_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct dice_irdev *irdev = netdev_priv(ndev);

	dev_info(&pdev->dev, "%s, Suspending\n", DRIVER_NAME);

	if (irdev->suspended)
		return 0;

	dice_ir_stop(ndev);
	irdev->suspended = 1;
	return 0;
}
#endif

static struct platform_driver dice_ir_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.bus	= &platform_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= dice_ir_probe,
	.remove		= __devexit_p(dice_ir_remove),
#ifdef CONFIG_PM
	.resume		= dice_ir_resume,
	.suspend	= dice_ir_suspend,
#endif
};

static int __init dice_ir_init(void)
{
	return platform_driver_register(&dice_ir_driver);
}
module_init(dice_ir_init);

static void __exit dice_ir_exit(void)
{
	platform_driver_unregister(&dice_ir_driver);
}
module_exit(dice_ir_exit);

MODULE_AUTHOR("Amit Virdi <amit.virdi@st.com>");
MODULE_DESCRIPTION("Fast IrDA Controller Driver")
MODULE_LICENSE("GPL");
