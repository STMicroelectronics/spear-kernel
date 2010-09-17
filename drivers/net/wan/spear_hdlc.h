/*
 * drivers/net/wan/spear_hdlc.h
 *
 * Header file of TDM/E1 and RS485 HDLC driver
 * for SPEAr SoC
 *
 * Copyright (C) 2010 ST Microelectronics
 * Frank Shi <frank.shi@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __SPEAR_HDLC_H
#define __SPEAR_HDLC_H

/* Descriptor per tx/rx ring */
#define DESC_PER_RING			8

/* Buffer size per descriptor */
#define BUFFER_SIZE			2048

/* MEM_POOL_SIZE defines how much memory allocated
 * for tx/rx descriptors and interrupt queue
 */
#define MEM_POOL_SIZE			65536

/* Register offset */
#define TAAR_RX_OFFSET			0x00
#define TADR_RX_OFFSET			0x04
#define FCR_RX_OFFSET			0x08
#define TAAR_TX_OFFSET			0x10
#define TADR_TX_OFFSET			0x14
#define FCR_TX_OFFSET			0x18
#define HTCR_OFFSET			0x20
#define HRCR_OFFSET			0x30
#define AFRAR_OFFSET			0x34
#define AFRDR_OFFSET			0x38
#define IBAR_OFFSET			0x40
#define IMR_OFFSET			0x44
#define IR_OFFSET			0x48
#define IQSR_OFFSET			0x4C


/* The following data structures are used by controller
 */

/* Initial block entry */
struct ibe {
	unsigned int TDA;
	unsigned int RDA;
};

/* Rx descriptor */
struct rx_desc {
	unsigned int SIM_IBC_EOQ_SOB;
	unsigned int RBA;
	unsigned int NRDA;
	unsigned int FR_ABT_OVF_FCRC_NBR;
	struct sk_buff *skb;
	unsigned int rsvd[3];
};

/* Tx descriptor */
struct tx_desc {
	unsigned int BINT_BOF_EOF_EOQ_NBT;
	unsigned int CRCC_PRI_TBA;
	unsigned int NTDA;
	unsigned int CFT_ABT_UND;
	struct sk_buff *skb;
	unsigned int rsvd[3];
};

/* Bit field defines */
#define TS0_DELAY_SHIFT				(12)
#define TS0_DELAY_MASK				(0x3 << TS0_DELAY_SHIFT)

#define HXCR_CHAN_SHIFT				(12)
#define HXCR_BUSY				(1 << 11)
#define HXCR_READ				(1 << 10)
#define DMA_CMD_ABORT				(0x0)
#define DMA_CMD_START				(0x1)
#define DMA_CMD_CONTINUE			(0x2)
#define DMA_CMD_HALT				(0x3)

#define CSMA_ENABLE				(1 << 6)
#define CTS_DELAY_SHIFT				(16)
#define PENALTY_SHIFT				(7)

#define TX_DESC_BINT				(1 << 15)
#define TX_DESC_BOF				(1 << 14)
#define TX_DESC_EOF				(1 << 13)
#define TX_DESC_EOQ				(1 << 12)
#define TX_DESC_NBT_MASK			(0xfff)
#define TX_DESC_STATUS_MASK			(0xe000)
#define TX_DESC_ERR_MASK			(0x6000)
#define TX_DESC_CFT				(0x8000)

#define RX_DESC_SIM				(1 << 14)
#define RX_DESC_IBC				(1 << 13)
#define RX_DESC_EOQ				(1 << 12)
#define RX_DESC_SOB_MASK			(0xffe)
#define RX_DESC_NBR_MASK			(0xfff)
#define RX_DESC_STATUS_MASK			(0xf000)
#define RX_DESC_STATUS_FR			(0x8000)
#define RX_DESC_ERR_MASK			(0x7000)

#define IR_HDLC					(1 << 0)
#define IR_ICOV					(1 << 1)

#define NEW_STATUS				(1 << 15)
#define HDLC_INT_CFT				(1 << 4)
#define HDLC_INT_CFR				(1 << 4)
#define HDLC_INT_BE				(1 << 3)
#define HDLC_INT_BF				(1 << 3)
#define HDLC_INT_HALT				(1 << 2)
#define HDLC_INT_EOQ				(1 << 1)
#define HDLC_INT_RRLF				(1 << 0)
#define HDLC_INT_ERF				(1 << 0)


#define SPEAR1310_HDLC_INT_TYPE_BIT		(1 << 14)
#define SPEAR1310_HDLC_INT_CHAN_SHIFT		(5)
#define SPEAR1310_HDLC_EDGE_CFG_BIT		(1 << 22)
#define SPEAR1310_HDLC_CFG_EN_BIT		(1 << 21)
#define SPEAR1310_HDLC_INTQ_SIZE		(512)
#define SPEAR1310_HDLC_TSA_READ_BIT		(1 << 11)
#define SPEAR1310_HDLC_TSA_BUSY_BIT		(1 << 10)
#define SPEAR1310_HDLC_TSA_INIT_BIT		(1 << 9)
#define SPEAR1310_HDLC_TSA_VAL_SHIFT		(9)

#define SPEAR310_HDLC_INT_TYPE_BIT		(1 << 13)
#define SPEAR310_HDLC_INT_CHAN_SHIFT		(6)
#define SPEAR310_HDLC_EDGE_CFG_BIT		(1 << 20)
#define SPEAR310_HDLC_CFG_EN_BIT		(1 << 19)
#define SPEAR310_HDLC_INTQ_SIZE			(512)
#define SPEAR310_HDLC_TSA_READ_BIT		(1 << 10)
#define SPEAR310_HDLC_TSA_BUSY_BIT		(1 << 9)
#define SPEAR310_HDLC_TSA_INIT_BIT		(1 << 8)
#define SPEAR310_HDLC_TSA_VAL_SHIFT		(7)

#define RS485_INT_TYPE_BIT			(1 << 13)
#define RS485_INT_CHAN_SHIFT			(6)
#define RS485_EDGE_CFG_BIT			(1 << 11)
#define RS485_CFG_EN_BIT			(0)
#define RS485_INTQ_SIZE				(512)


#endif	/* __SPEAR_HDLC_H */
