/*
 * drivers/net/can/bosch_ccan.h
 *
 * CAN bus driver definitions for Bosch CCAN controller
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
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __CCAN_H__
#define __CCAN_H__

#include <linux/can.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

/* ccan register offsets */
enum ccan_regs {
	CAN_CONTROL = 0x00,
	CAN_STATUS = 0x02,
	CAN_ERROR = 0x04,
	CAN_BTR = 0x06,
	CAN_IR = 0x08,
	CAN_TEST = 0x0a,
	CAN_BRP_EXT = 0x0c,
	CAN_IF1 = 0x10,
	CAN_IF2 = 0x40,
	CAN_TXRQST = 0x80,	/* 32bit */
	CAN_NEWDAT = 0x90,	/* 32bit */
	CAN_INTPND = 0xa0,	/* 32bit */
	CAN_MSGVAL = 0xb0,	/* 32bit */
};

#define CAN_IF_COMR(x)		(CAN_IF1 + (x) * 0x30 + 0x00)
#define CAN_IF_COMM(x)		(CAN_IF1 + (x) * 0x30 + 0x02)
#define CAN_IF_MASK(x)		(CAN_IF1 + (x) * 0x30 + 0x04)	/* 32bit */
#define CAN_IF_ARB(x)		(CAN_IF1 + (x) * 0x30 + 0x08)	/* 32bit */
#define CAN_IF_MCONT(x)		(CAN_IF1 + (x) * 0x30 + 0x0c)
#define CAN_IF_DATAA(x)		(CAN_IF1 + (x) * 0x30 + 0x0e)	/* 32bit */
#define CAN_IF_DATAB(x)		(CAN_IF1 + (x) * 0x30 + 0x12)	/* 32bit */

/* control register */
#define CONTROL_TEST		(1<<7)
#define CONTROL_CCE		(1<<6)
#define CONTROL_DISABLE_AR	(1<<5)
#define CONTROL_ENABLE_AR	(0<<5)
#define CONTROL_EIE		(1<<3)
#define CONTROL_SIE		(1<<2)
#define CONTROL_IE		(1<<1)
#define CONTROL_INIT		(1<<0)

/* test register */
#define TEST_RX			(1<<7)
#define TEST_TX1		(1<<6)
#define TEST_TX2		(1<<5)
#define TEST_LBACK		(1<<4)
#define TEST_SILENT		(1<<3)
#define TEST_BASIC		(1<<2)

/* status register */
#define STATUS_BOFF		(1<<7)
#define STATUS_EWARN		(1<<6)
#define STATUS_EPASS		(1<<5)
#define STATUS_RXOK		(1<<4)
#define STATUS_TXOK		(1<<3)
#define STATUS_LEC_MASK		(1<<2)
#define LEC_STUFF_ERROR		1
#define LEC_FORM_ERROR		2
#define LEC_ACK_ERROR		3
#define LEC_BIT1_ERROR		4
#define LEC_BIT0_ERROR		5
#define LEC_CRC_ERROR		6

/* bit-timing register */
#define BTR_BRP_MASK		0x3f
#define BTR_BRP_SHIFT		0
#define BTR_SJW_SHIFT		6
#define BTR_SJW_MASK		(0x3<<BTR_SJW_SHIFT)
#define BTR_TSEG1_SHIFT		8
#define BTR_TSEG1_MASK		(0xf<<BTR_TSEG1_SHIFT)
#define BTR_TSEG2_SHIFT		12
#define BTR_TSEG2_MASK		(0x7<<BTR_TSEG2_SHIFT)

/* IFx command request */
#define IF_COMR_BUSY		(1<<15)

/* IFx command mask */
#define IF_COMM_WR		(1<<7)
#define IF_COMM_MASK		(1<<6)
#define IF_COMM_ARB		(1<<5)
#define IF_COMM_CONTROL		(1<<4)
#define IF_COMM_CLR_INT_PND	(1<<3)
#define IF_COMM_TXRQST		(1<<2)
#define IF_COMM_DATAA		(1<<1)
#define IF_COMM_DATAB		(1<<0)
#define IF_COMM_ALL		(IF_COMM_MASK | IF_COMM_ARB | \
				IF_COMM_CONTROL | IF_COMM_TXRQST | \
				IF_COMM_DATAA | IF_COMM_DATAB)

/* IFx arbitration */
#define IF_ARB_MSGVAL		(1<<31)
#define IF_ARB_MSGXTD		(1<<30)
#define IF_ARB_TRANSMIT		(1<<29)

/* IFx message control */
#define IF_MCONT_NEWDAT		(1<<15)
#define IF_MCONT_MSGLST		(1<<14)
#define IF_MCONT_INTPND		(1<<13)
#define IF_MCONT_UMASK		(1<<12)
#define IF_MCONT_TXIE		(1<<11)
#define IF_MCONT_RXIE		(1<<10)
#define IF_MCONT_RMTEN		(1<<9)
#define IF_MCONT_TXRQST		(1<<8)
#define IF_MCONT_EOB		(1<<7)

/* message object */
#define MAX_OBJECT		31
#define MAX_TRANSMIT_OBJECT	15
#define RECEIVE_OBJECT_BITS	0xffff0000

/*
 * CCAN operating modes:
 * Support is available for default as well as test operating modes.
 * Normal mode will generally be used as a default mode in most cases,
 * however, various test modes may be useful in specific use-cases.
 */
enum bosch_ccan_operating_mode {
	CCAN_NORMAL_MODE = 0,
	CCAN_BASIC_MODE,
	CCAN_LOOPBACK_MODE,
	CCAN_LOOPBACK_WITH_SILENT_MODE,
	CCAN_SILENT_MODE
};

/*
 * Automatic Retransmssion compliance with ISO11898, 6.3.3 Recovery Management:
 * Support is available for enabling automatic retransmission of frames
 * (default behavior) as well as disabling the same for TTCAN
 * environments.
 */
enum bosch_ccan_auto_tx_config {
	CCAN_ENABLE_AUTO_RE_TRANSMIT = 0,
	CCAN_DISABLE_AUTO_RE_TRANSMIT
};

/* CCAN private data structure */
struct bosch_ccan_priv {
	struct can_priv can;	/* must be the first member */
	struct net_device *dev;
	int tx_object;
	int last_status;
	struct delayed_work work;
	u16 (*read_reg) (const struct bosch_ccan_priv *priv,
				enum ccan_regs reg);
	void (*write_reg) (const struct bosch_ccan_priv *priv,
				enum ccan_regs reg, u16 val);
	void __iomem *reg_base;	 /* ioremap'ed address to registers */
	unsigned long irq_flags; /* for request_irq() */
	struct clk *clk;
};

struct net_device *alloc_bosch_ccandev(int sizeof_priv);
void free_bosch_ccandev(struct net_device *dev);
int register_bosch_ccandev(struct net_device *dev);
void unregister_bosch_ccandev(struct net_device *dev);

#endif /* __CCAN_H__ */
