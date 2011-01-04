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

#ifndef C_CAN_H
#define C_CAN_H

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
#define STATUS_LEC_MASK		0x07

/* error counter register */
#define ERR_COUNTER_TEC_MASK	0xff
#define ERR_COUNTER_TEC_SHIFT	0
#define ERR_COUNTER_REC_SHIFT	8
#define ERR_COUNTER_REC_MASK	(0x7f << ERR_COUNTER_REC_SHIFT)
#define ERR_COUNTER_RP_SHIFT	15
#define ERR_COUNTER_RP_MASK	(0x1 << ERR_COUNTER_RP_SHIFT)

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

/* message object split */
#define C_CAN_NO_OF_OBJECTS	31
#define C_CAN_MSG_OBJ_RX_NUM	16
#define C_CAN_MSG_OBJ_TX_NUM	16

#define C_CAN_MSG_OBJ_RX_FIRST	0
#define C_CAN_MSG_OBJ_RX_LAST	(C_CAN_MSG_OBJ_RX_FIRST + \
				C_CAN_MSG_OBJ_RX_NUM - 1)

#define C_CAN_MSG_OBJ_TX_FIRST	(C_CAN_MSG_OBJ_RX_LAST + 1)
#define C_CAN_MSG_OBJ_TX_LAST	(C_CAN_MSG_OBJ_TX_FIRST + \
				C_CAN_MSG_OBJ_TX_NUM - 1)

#define C_CAN_MSG_OBJ_RX_SPLIT	8
#define C_CAN_MSG_RX_LOW_LAST	(C_CAN_MSG_OBJ_RX_SPLIT - 1)

#define C_CAN_NEXT_MSG_OBJ_MASK	(C_CAN_MSG_OBJ_TX_NUM - 1)
#define RECEIVE_OBJECT_BITS	0x0000ffff

/* status interrupt */
#define STATUS_INTERRUPT	0x8000

/* global interrupt masks */
#define ENABLE_ALL_INTERRUPTS	1
#define DISABLE_ALL_INTERRUPTS	0

/* minimum timeout for checking BUSY status */
#define MIN_TIMEOUT_VALUE	6

/* napi related */
#define C_CAN_NAPI_WEIGHT	C_CAN_MSG_OBJ_RX_NUM

/* c_can IF registers */
struct c_can_if_regs {
	u16 com_reg;
	u16 com_mask;
	u16 mask1;
	u16 mask2;
	u16 arb1;
	u16 arb2;
	u16 msg_cntrl;
	u16 data[4];
	u16 _reserved[13];
};

/* c_can hardware registers */
struct c_can_regs {
	u16 control;
	u16 status;
	u16 error_counter;
	u16 btr;
	u16 ir;
	u16 test;
	u16 brp_ext;
	u16 _reserved1;
	struct c_can_if_regs ifreg[2]; /* [0] = IF1 and [1] = IF2 */
	u16 _reserved2[8];
	u16 txrqst1;
	u16 txrqst2;
	u16 _reserved3[6];
	u16 newdat1;
	u16 newdat2;
	u16 _reserved4[6];
	u16 intpnd1;
	u16 intpnd2;
	u16 _reserved5[6];
	u16 msgval1;
	u16 msgval2;
	u16 _reserved6[6];
};

/* c_can lec values */
enum c_can_lec_type {
	LEC_STUFF_ERROR = 1,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
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

/* c_can private data structure */
struct c_can_priv {
	struct can_priv can;	/* must be the first member */
	struct napi_struct napi;
	struct net_device *dev;
	int tx_object;
	int current_status;
	int last_status;
	u16 (*read_reg) (struct c_can_priv *priv, void *reg);
	void (*write_reg) (struct c_can_priv *priv, void *reg, u16 val);
	struct c_can_regs __iomem *reg_base;
	unsigned long irq_flags; /* for request_irq() */
	unsigned int tx_next;
	unsigned int tx_echo;
	struct clk *clk;
};

void c_can_enable_all_interrupts(struct c_can_priv *priv, int enable);
struct net_device *alloc_c_can_dev(void);
void free_c_can_dev(struct net_device *dev);
int register_c_can_dev(struct net_device *dev);
void unregister_c_can_dev(struct net_device *dev);

#endif /* C_CAN_H */
