/*
 * drivers/usb/gadget/designware_udc.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar <rajeev-dlh.kumar@st.com>
 * Shiraz HAshim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _DESIGNWARE_UDC_H
#define _DESIGNWARE_UDC_H

#include <linux/clk.h>
#include <asm-generic/ioctl.h>

#undef DEBUG
#define DRIVER_DESC		"Designware USB Device Controller driver"

#define dw_udc_debug_flags	(DBG_INTS | DBG_REQUESTS)

#ifdef DEBUG
#define DW_UDC_DBG(type, fmt, args...)					\
	do {							\
		if (dw_udc_debug_flags & (type)) {		\
			pr_info("dw_udc: " fmt , ## args);	\
		}						\
	} while (0)
#else
#define DW_UDC_DBG(type, fmt, args...) do { } while (0)
#endif /* DEBUG */

/* Modify this variable in order to trace different parts of the driver */
#define DBG_GENERIC		(0x1 << 0)
#define DBG_FUNC_ENTRY		(0x1 << 1)
#define DBG_FUNC_EXIT		(0x1 << 2)
#define DBG_INTS		(0x1 << 3)
#define DBG_PACKETS		(0x1 << 4)
#define DBG_ADDRESS		(0x1 << 5)
#define DBG_ENDPOINT		(0x1 << 6)
#define DBG_REQUESTS		(0x1 << 7)
#define DBG_FLOW		(0x1 << 8)
#define DBG_QUEUES		(0x1 << 9)
#define DBG_REGISTERS		(0x1 << 10)
#define DBG_EP0STATE		(0x1 << 11)

#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

/* usb device register offsets */
#define UDC_EP_IN_REG_OFF	0x000
#define UDC_EP_OUT_REG_OFF	0x200
#define UDC_GLOB_REG_OFF	0x400
#define UDC20_REG_OFF		0x500

/* synopsys udc memory mapped registers */
struct dw_udc_glob_regs {
	u32 dev_conf;
	u32 dev_control;
	u32 dev_status;
	u32 dev_int;
	u32 dev_int_mask;
	u32 endp_int;
	u32 endp_int_mask;
} __attribute__ ((aligned(4), packed));

/* Device Configuration Register */
#define DEV_CONF_HS_SPEED	(0x0 << 0)
#define DEV_CONF_LS_SPEED	(0x2 << 0)
#define DEV_CONF_FS_SPEED	(0x3 << 0)
#define DEV_CONF_REMWAKEUP	(0x1 << 2)
#define DEV_CONF_SELFPOW	(0x1 << 3)
#define DEV_CONF_SYNCFRAME	(0x1 << 4)
#define DEV_CONF_PHYINT_8	(0x1 << 5)
#define DEV_CONF_PHYINT_16	(0x0 << 5)
#define DEV_CONF_UTMI_BIDIR	(0x1 << 6)
#define DEV_CONF_STATUS_STALL	(0x1 << 7)
#define DEV_CONF_CSR_PRG	(0x1 << 17)

/* Device Control Register */
#define DEV_CNTL_RESUME		(0x1 << 0)
#define DEV_CNTL_TFFLUSH	(0x1 << 1)
#define DEV_CNTL_RXDMAEN	(0x1 << 2)
#define DEV_CNTL_TXDMAEN	(0x1 << 3)
#define DEV_CNTL_DESCRUPD	(0x1 << 4)
#define DEV_CNTL_BIGEND		(0x1 << 5)
#define DEV_CNTL_BUFFILL	(0x1 << 6)
#define DEV_CNTL_TSHLDEN	(0x1 << 7)
#define DEV_CNTL_BURSTEN	(0x1 << 8)
#define DEV_CNTL_DMAMODE	(0x1 << 9)
#define DEV_CNTL_SD		(0x1 << 10)
#define DEV_CNTL_SCALEDOWN	(0x1 << 11)
#define DEV_CNTL_CSR_DONE	(0x1 << 13)
#define DEV_CNTL_BURSTLENU	(0x1 << 16)
#define DEV_CNTL_BURSTLENMSK	(0xff << 16)
#define DEV_CNTL_TSHLDLENU	(0x1 << 24)
#define DEV_CNTL_TSHLDLENMSK	(0xff << 24)

/* Device Status register */
#define DEV_STAT_CFG		(0xf << 0)
#define DEV_STAT_INTF		(0xf << 4)
#define DEV_STAT_ALT		(0xf << 8)
#define DEV_STAT_SUSP		(0x1 << 12)
#define DEV_STAT_ENUM		(0x3 << 13)
#define DEV_STAT_ENUM_HS	(0x0 << 13)
#define DEV_STAT_ENUM_FS	(0x1 << 13)
#define DEV_STAT_ENUM_LS	(0x2 << 13)
#define DEV_STAT_RXFIFO_EMPTY	(0x1 << 15)
#define DEV_STAT_PHY_ERR	(0x1 << 16)
#define DEV_STAT_TS		(0xf << 28)

/* Device Interrupt Register */
#define DEV_INT_MSK		(0x7f << 0)

#define DEV_INT_SETCFG		(0x1 << 0)
#define DEV_INT_SETINTF		(0x1 << 1)
#define DEV_INT_INACTIVE	(0x1 << 2)
#define DEV_INT_USBRESET	(0x1 << 3)
#define DEV_INT_SUSPUSB		(0x1 << 4)
#define DEV_INT_SOF		(0x1 << 5)
#define DEV_INT_ENUM		(0x1 << 6)

/* Endpoint Interrupt Mask Register */
#define ENDP_INT_INEPMSK	(0xffff << 0)
#define ENDP_INT_OUTEPMSK	(0xffff << 16)

#define ENDP0_IN_INT		(0x1 << 0)
#define ENDP0_OUT_INT		(0x1 << 16)

struct dw_udc_epin_regs {
	u32 control;
	u32 status;
	u32 bufsize;
	u32 max_pack_size;
	u32 reserved1;
	u32 desc_ptr;
	u32 reserved2;
	u32 write_confirm;
} __attribute__ ((aligned(4), packed));

struct dw_udc_epout_regs {
	u32 control;
	u32 status;
	u32 frame_num;
	u32 bufsize;
	u32 setup_ptr;
	u32 desc_ptr;
	u32 reserved1;
	u32 read_confirm;
} __attribute__ ((aligned(4), packed));

/* Endpoint Control Register */
#define	ENDP_CNTL_STALL		(0x1 << 0)
#define	ENDP_CNTL_FLUSH		(0x1 << 1)
#define	ENDP_CNTL_SNOOP		(0x1 << 2)
#define	ENDP_CNTL_POLL		(0x1 << 3)
#define	ENDP_CNTL_ISO		(0x1 << 4)
#define	ENDP_CNTL_BULK		(0x2 << 4)
#define	ENDP_CNTL_INT		(0x3 << 4)
#define	ENDP_CNTL_NAK		(0x1 << 6)
#define	ENDP_CNTL_SNAK		(0x1 << 7)
#define	ENDP_CNTL_CNAK		(0x1 << 8)
#define	ENDP_CNTL_RRDY		(0x1 << 9)
#define	ENDP_CNTL_CLOSEDESC	(0x1 << 10)

/* Endpoint Status Register */
#define	ENDP_STATUS_OUTMSK	(0x3 << 4)
#define	ENDP_STATUS_OUT_NONE	(0x0 << 4)
#define	ENDP_STATUS_OUT_DATA	(0x1 << 4)
#define	ENDP_STATUS_OUT_SETUP	(0x2 << 4)
#define	ENDP_STATUS_IN		(0x1 << 6)
#define	ENDP_STATUS_BUFFNAV	(0x1 << 7)
#define	ENDP_STATUS_FATERR	(0x1 << 8)
#define	ENDP_STATUS_HOSTBUSERR	(0x1 << 9)
#define	ENDP_STATUS_TDC		(0x1 << 10)

struct dw_udc_plug_regs {
	u32 status;
	u32 pending;
} __attribute__ ((aligned(4), packed));

#define PLUG_STATUS_EN			(0x1 << 0)
#define PLUG_STATUS_ATTACHED		(0x1 << 1)
#define PLUG_STATUS_PHY_RESET		(0x1 << 2)
#define PLUG_STATUS_PHY_MODE		(0x1 << 3)

#define PLUG_INTPEND			(0x1 << 0)

/* DMA Descriptors: in/out */
struct dw_udc_bulkd {
	__le32 status;
	__le32 reserved;
	__le32 bufp;
	__le32 nextd;
	/* The following fields are used by the driver only */
	struct list_head desc_list;
	dma_addr_t dma_addr;
} __attribute__ ((aligned(16), packed));

/* Bulk Descriptor Status Field */
#define	DMAUSB_BS_MASK		(0x3 << 30)
#define	DMAUSB_HOSTRDY		(0x0 << 30)
#define	DMAUSB_DMABSY		(0x1 << 30)
#define	DMAUSB_DMADONE		(0x2 << 30)
#define	DMAUSB_HOSTBSY		(0x3 << 30)

#define	DMAUSB_RXSTS_MASK	(0x3 << 28)
#define	DMAUSB_SUCCESS		(0x0 << 28)
#define	DMAUSB_DESCRERR		(0x1 << 28)
#define	DMAUSB_BUFERR		(0x3 << 28)
#define	DMAUSB_LASTDESCR	(0x1 << 27)
#define	DMAUSB_LEN_MASK		(0xFFFF << 0)

/* setup data structure */
struct dw_udc_setupd {
	__le32 status;
	__le32 reserved;
	__le32 data1;
	__le32 data2;
} __attribute__ ((aligned(16), packed));

/* endpoint specific structure */
struct dw_udc_ep {
	struct usb_ep ep;
	struct dw_udc_dev *udev;
	struct dw_udc_epin_regs __iomem *in_regs;
	struct dw_udc_epout_regs __iomem *out_regs;
	const struct usb_endpoint_descriptor *desc;
	struct dw_udc_bulkd *curr_chain_in;
	struct dw_udc_bulkd *curr_chain_out;
	struct list_head queue;

	/*
	 * These pointer are the virtual equivalents of those in
	 * the UDC registers.
	 */
	struct dw_udc_bulkd *desc_in_ptr;
	struct dw_udc_bulkd *desc_out_ptr;
	struct dw_udc_bulkd *setup_ptr;
	u16 fifo_size;
	u8 addr;
	u8 attrib;
	u8 intf;	/* the interface to which it belongs */
	u8 alt_intf;	/* the alt interface to which it belongs */
	u8 stopped;
	u8 config_req;
	unsigned long last_updated; /* when did last data arrived */
};

enum ep0_state {
	EP0_CTRL_IDLE = 0,
	EP0_DATA_IN_STAGE = 1,
	EP0_DATA_OUT_STAGE = 2,
	EP0_STATUS_IN_STAGE = 3,
	EP0_STATUS_OUT_STAGE = 4,
	EP0_ACK_SETCONF_INTER_DELAYED = 5,
};

struct dw_udc_request {
	struct usb_request req;
	struct list_head queue;
	u8 mapped; /* whether dma mapped or not */
};

struct dw_udc_dev {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct device *dev;
	spinlock_t lock;
	enum ep0_state ep0state;
	struct dma_pool *desc_pool;
	struct timer_list out_ep_timer;
	struct clk *clk;
	int irq;
	u8 int_cmd;
	u8 active_suspend;
	u8 irq_wake;

	void __iomem *csr_base;
	struct dw_udc_epin_regs __iomem *epin_base;
	struct dw_udc_epout_regs __iomem *epout_base;
	struct dw_udc_glob_regs __iomem *glob_base;
	struct dw_udc_plug_regs __iomem *plug_base;
	struct dw_udc_fifo_regs __iomem *fifo_base;

	int num_ep;
	struct dw_udc_ep *ep;
};

#define is_ep_in(ep)	(((ep)->addr) & USB_DIR_IN)

/* IOCTL calls */
/* Program UDC20 Regs */
#define UDC_SET_CONFIG	_IOW('u', 0x01, struct usb_descriptor_header)

static inline int usb_gadget_ioctl(struct usb_gadget *gadget, unsigned code,
		unsigned long param)
{
	if (!gadget->ops->ioctl)
		return -EOPNOTSUPP;
	return gadget->ops->ioctl(gadget, code, param);
}

#endif /* _DESIGNWARE_UDC_H */
