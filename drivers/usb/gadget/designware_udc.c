/*
 * drivers/usb/gadget/designware_udc.c
 *
 * Copyright (C) 2010 ST Microelectronics
 *
 * Rajeev Kumar <rajeev-dlh.kumar@st.com>
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/version.h>
#include <asm/unaligned.h>
#include <plat/udc.h>
#ifdef CONFIG_ARCH_SPEAR13XX
#include <mach/misc_regs.h>
#endif
#include "designware_udc.h"

static const char driver_name[] = "designware-udc";

#ifdef DEBUG
static void show_queues(struct dw_udc_ep *ep)
{
	struct dw_udc_request *req;
	unsigned size;
	int t = ep->addr;

	size = PAGE_SIZE;

	DW_UDC_DBG(DBG_QUEUES, "%s (ep%d %s)\n",
			ep->ep.name, t, (t & USB_DIR_IN) ? "in" : "out");

	if (list_empty(&ep->queue)) {
		DW_UDC_DBG(DBG_QUEUES, "\t(nothing queued)\n");
		return;
	}

	list_for_each_entry(req, &ep->queue, queue) {
		DW_UDC_DBG(DBG_QUEUES, "\treq %p len %d/%d buf %p\n",
				&req->req, req->req.actual, req->req.length,
				req->req.buf);
	}
	return;
}

static void show_chain(struct dw_udc_ep *ep)
{
	struct dw_udc_bulkd *tmp_desc;

	DW_UDC_DBG(DBG_REGISTERS, "Chain for ep %s:\n", ep->ep.name);

	if (is_ep_in(ep))
		tmp_desc = ep->desc_in_ptr;
	else
		tmp_desc = ep->desc_out_ptr;

	while (tmp_desc != 0x0) {
		DW_UDC_DBG(DBG_REGISTERS, "%08x %08x %08x\n",
				le32_to_cpu(tmp_desc->status),
				le32_to_cpu(tmp_desc->bufp),
				le32_to_cpu(tmp_desc->nextd));

		if (le32_to_cpu(tmp_desc->status) & DMAUSB_LASTDESCR)
			break;
		tmp_desc =
			list_entry(tmp_desc->desc_list.next,
					struct dw_udc_bulkd, desc_list);
	}
}

static void show_global(struct dw_udc_dev *udev)
{
	struct dw_udc_glob_regs *glob = udev->glob_base;

	DW_UDC_DBG(DBG_REGISTERS, "Global registers:\n");
	DW_UDC_DBG(DBG_REGISTERS, "Config:%08x Control:%08x Status:%08x\n",
			readl(&glob->dev_conf),
			readl(&glob->dev_control), readl(&glob->dev_status));
	DW_UDC_DBG(DBG_REGISTERS, "D.Int:%08x mask=%08x E.Int:%08x mask=%08x\n",
			readl(&glob->dev_int), readl(&glob->dev_int_mask),
			readl(&glob->endp_int), readl(&glob->endp_int_mask));
}

static void show_registers(struct dw_udc_ep *ep)
{
	struct dw_udc_epin_regs *in_regs = ep->in_regs;
	struct dw_udc_epout_regs *out_regs = ep->out_regs;

	DW_UDC_DBG(DBG_REGISTERS, "Endp:%s, IN REGS\n", ep->ep.name);
	DW_UDC_DBG(DBG_REGISTERS, "Control:%08x Status:%08x Bufsize:%08x\n",
			readl(&in_regs->control), readl(&in_regs->status),
			readl(&in_regs->bufsize));
	DW_UDC_DBG(DBG_REGISTERS, "Max packet:%08x desc_ptr:%08x\n",
			readl(&in_regs->max_pack_size),
			readl(&in_regs->desc_ptr));
	DW_UDC_DBG(DBG_REGISTERS, "OUT REGS\n");
	DW_UDC_DBG(DBG_REGISTERS, "Control:%08x Status:%08x Frame num:%08x\n",
			readl(&out_regs->control), readl(&out_regs->status),
			readl(&out_regs->frame_num));
	DW_UDC_DBG(DBG_REGISTERS, "Bufsize:%08x setup_ptr %08x desc_ptr:%08x\n",
			readl(&out_regs->bufsize), readl(&out_regs->setup_ptr),
			readl(&out_regs->desc_ptr));
}

#else
static void show_queues(struct dw_udc_ep *ep)
{
}
static void show_chain(struct dw_udc_ep *ep)
{
}
static void show_global(struct dw_udc_dev *udev)
{
}
static void show_registers(struct dw_udc_ep *ep)
{
}
#endif /* DEBUG */

static int desc_pool_init(struct dw_udc_dev *udev)
{

	udev->desc_pool = dma_pool_create("udc desc pool", udev->dev,
			sizeof(struct dw_udc_bulkd), 16, 0);
	if (!udev->desc_pool)
		return -ENOMEM;

	return 0;
}

static void desc_pool_remove(struct dw_udc_dev *udev)
{
	if (udev->desc_pool)
		dma_pool_destroy(udev->desc_pool);
	udev->desc_pool = NULL;
}

static inline void desc_init(struct dw_udc_bulkd *desc, dma_addr_t dma)
{
	memset(desc, 0, sizeof *desc);
	desc->dma_addr = dma;
	desc->status = cpu_to_le32(DMAUSB_HOSTRDY);
	desc->bufp = 0x0;
	wmb();
	INIT_LIST_HEAD(&desc->desc_list);
}

static struct dw_udc_bulkd *desc_alloc(struct dw_udc_dev *udev, int flags)
{
	struct dw_udc_bulkd *desc;
	dma_addr_t dma;

	desc = dma_pool_alloc(udev->desc_pool, flags, &dma);
	if (desc != NULL)
		desc_init(desc, dma);
	else
		dev_err(udev->dev, "desc pool_alloc fail %s\n", __func__);

	return desc;
}

static void desc_free(struct dw_udc_dev *udev, struct dw_udc_bulkd *desc)
{
	dma_pool_free(udev->desc_pool, desc, desc->dma_addr);
}

static void udc_put_descrs(struct dw_udc_dev *udev, struct dw_udc_bulkd *desc)
{
	struct list_head *entry, *temp;
	struct dw_udc_bulkd *tmp;

	if (desc == NULL)
		return;

	list_for_each_safe(entry, temp, &desc->desc_list) {
		tmp = list_entry(entry, struct dw_udc_bulkd, desc_list);
		list_del(entry);
		desc_free(udev, tmp);
	}

	list_del(&desc->desc_list);
	desc_free(udev, desc);
}

static void *udc_get_descrs(struct dw_udc_dev *udev, unsigned short num)
{
	struct dw_udc_bulkd *desc, *desc_prev, *head;

	head = desc_alloc(udev, GFP_ATOMIC);
	if (unlikely(!head)) {
		dev_err(udev->dev, "desc_alloc fail %s\n", __func__);
		return NULL;
	}

	desc = head;
	while (--num) {
		desc_prev = desc;
		desc = desc_alloc(udev, GFP_ATOMIC);
		if (unlikely(!desc)) {
			dev_err(udev->dev, "Not enough memory for descrs\n");
			goto cleanup;
		}
		desc_prev->nextd = cpu_to_le32(desc->dma_addr);
		wmb();
		list_add_tail(&desc->desc_list, &head->desc_list);
	}

	return head;

cleanup:
	udc_put_descrs(udev, head);
	return NULL;
}

/*
 * Routine to program UDC NE registers
 *
 *	EndPtBuf[3:0]	= EndPoint number
 *	EndPtBuf[4]	= EndPoint direction
 *	EndPtBuf[6:5]	= EndPoint Type
 *	EndPtBuf[10:7]	= Config number
 *	EndPtBuf[14:11]	= Interface number
 *	EndPtBuf[18:15]	= Alternate number
 *	EndPtBuf[29:19]	= EndPoint MaxPktSize
 *	EndPtBuf[31:30]	= Unused bits
 */

static void udc_prog_udc20_regs(struct dw_udc_dev *udev,
		struct dw_udc_ep *ep)
{
	struct dw_udc_glob_regs *glob = udev->glob_base;
	void __iomem *udc20_regs = udev->csr_base + UDC20_REG_OFF;
	u32 config, intf, alt_intf;
	u32 epn, dir, maxpacket, attrib;

	epn = ep->addr & ~USB_DIR_IN;
	dir = is_ep_in(ep) ? 1 : 0;
	dir <<= 4;

	attrib = ep->attrib << 5;

	config = readl(&glob->dev_status) & DEV_STAT_CFG;
	config <<= 7;

	intf = ep->intf;
	intf <<= 11;

	alt_intf = ep->alt_intf;
	alt_intf <<= 15;

	maxpacket = ep->ep.maxpacket;
	maxpacket <<= 19;

	if (epn == 0) {
		intf = 0;
		alt_intf = 0;
		config = 0;
	}

	/* program at the right offset */
	writel((maxpacket | alt_intf | intf | config | attrib | dir | epn),
			(udc20_regs + ((epn + 1) << 2)));
}

void udc_set_config(struct dw_udc_dev *udev,
		struct usb_descriptor_header **src)
{
	struct dw_udc_ep *ep;
	unsigned desc_type;
	u8 epn;
	u8 intf = 0;
	u8 alt_intf = 0;

	for (; 0 != *src; src++) {
		desc_type = (*src)->bDescriptorType;

		switch (desc_type) {
		case USB_DT_INTERFACE:
			intf = ((struct usb_interface_descriptor *)(*src))->
				bInterfaceNumber;
			alt_intf = ((struct usb_interface_descriptor *)(*src))->
				bAlternateSetting;
			break;
		case USB_DT_ENDPOINT:
			epn = ((struct usb_endpoint_descriptor *)(*src))->
				bEndpointAddress & ~USB_DIR_IN;
			ep = &udev->ep[epn];
			ep->alt_intf = alt_intf;
			ep->intf = intf;
			ep->attrib = ((struct usb_endpoint_descriptor *)(*src))
				->bmAttributes & 0x3;

			udc_prog_udc20_regs(udev, ep);
#ifdef CONFIG_ARCH_SPEAR13XX
			udelay(1);
#endif
			break;
		default:
			;
		}
	}
}

static void udc_config_ep(struct dw_udc_dev *udev, struct dw_udc_ep *ep)
{
	struct dw_udc_epin_regs *in_regs;
	struct dw_udc_epout_regs *out_regs;
	struct dw_udc_glob_regs *glob;
	u32 tmp;

	in_regs = ep->in_regs;
	out_regs = ep->out_regs;
	glob = ep->udev->glob_base;

	switch (ep->attrib) {
	case USB_ENDPOINT_XFER_CONTROL:
		tmp = (ep->attrib << 4) | ENDP_CNTL_FLUSH;
		writel(tmp, &in_regs->control);

		writel(ep->fifo_size, &in_regs->bufsize);
		writel(ep->ep.maxpacket, &in_regs->max_pack_size);

		writel(ep->attrib, &out_regs->control);
		tmp = ep->ep.maxpacket | (ep->fifo_size << 16);
		writel(tmp, &out_regs->bufsize);
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
	case USB_ENDPOINT_XFER_ISOC:
		if (is_ep_in(ep)) {
			tmp = ((ep->attrib << 4) | ENDP_CNTL_CNAK |
					ENDP_CNTL_FLUSH) & ~ENDP_CNTL_SNAK;
			writel(tmp, &in_regs->control);

			writel(ep->fifo_size, &in_regs->bufsize);
			writel(ep->ep.maxpacket, &in_regs->max_pack_size);
		} else {
			tmp = (ep->attrib << 4) | ENDP_CNTL_CNAK;
			writel(tmp, &out_regs->control);

			tmp = ep->ep.maxpacket | (ep->fifo_size << 16);
			writel(tmp, &out_regs->bufsize);
		}
		break;

	default:
		break;
	}
}

/* Routine to program DMA wrapper registers */
static void udc_config_dma(struct dw_udc_ep *ep)
{
	unsigned int epn = ep->addr & ~USB_DIR_IN;
	struct dw_udc_dev *udev;
	u32 tmp, descp;
	struct dw_udc_epin_regs *in_regs;
	struct dw_udc_epout_regs *out_regs;
	struct dw_udc_glob_regs *glob;
	struct dw_udc_bulkd *desc;

	in_regs = ep->in_regs;
	out_regs = ep->out_regs;
	glob = ep->udev->glob_base;
	udev = ep->udev;

	if (ep->attrib == USB_ENDPOINT_XFER_CONTROL) {
		/* program descriptor for setup packet */
		desc = udc_get_descrs(udev, 1);
		if (desc == NULL)
			return;

		descp = desc->dma_addr;
		writel(descp, &(out_regs->setup_ptr));
		ep->setup_ptr = desc;

		/* program descriptor for out stage packet */
		desc = udc_get_descrs(udev, 1);
		if (desc == NULL)
			return;

		descp = desc->dma_addr;
		writel(descp, &(out_regs->desc_ptr));
		ep->desc_out_ptr = desc;

		/*
		 * set NULL descriptor for in stage packet
		 * will be set when required
		 */
		writel(0, &in_regs->desc_ptr);
		ep->desc_in_ptr = NULL;

		/* set RRDY bit to receive any setup packets */
		tmp = readl(&out_regs->control);
		tmp |= ENDP_CNTL_RRDY;
		writel(tmp, &out_regs->control);

		/* prepare intr mask */
		tmp = readl(&glob->endp_int_mask);
		tmp &= ~((ENDP0_IN_INT << epn) | (ENDP0_OUT_INT << epn));
	} else {
		if (is_ep_in(ep)) {
			writel(0, &in_regs->desc_ptr);
			ep->desc_in_ptr = NULL;

			tmp = readl(&glob->endp_int_mask);
			tmp &= ~(ENDP0_IN_INT << epn);
		} else {
			writel(0, &out_regs->desc_ptr);
			ep->desc_out_ptr = NULL;

			tmp = readl(&glob->endp_int_mask);
			tmp &= ~(ENDP0_OUT_INT << epn);
		}
	}

	/* enable endpoint interrupts */
	writel(tmp, &glob->endp_int_mask);
}

static void udc_clear_nak(struct dw_udc_ep *ep)
{
	u32 tmp;

	tmp = readl(&ep->in_regs->control);
	tmp |= ENDP_CNTL_CNAK;
	tmp &= ~ENDP_CNTL_SNAK;
	writel(tmp, &ep->in_regs->control);

	tmp = readl(&ep->out_regs->control);
	tmp |= ENDP_CNTL_CNAK;
	tmp &= ~ENDP_CNTL_SNAK;
	writel(tmp, &ep->out_regs->control);
}

static void udc_stall_ep(struct dw_udc_ep *ep)
{
	u32 tmp;

	if (is_ep_in(ep)) {
		tmp = readl(&ep->out_regs->control);
		tmp |= ENDP_CNTL_STALL;
		writel(tmp, &ep->out_regs->control);
	} else {
		tmp = readl(&ep->in_regs->control);
		tmp |= ENDP_CNTL_STALL;
		writel(tmp, &ep->in_regs->control);
	}
}

static void udc_reinit(struct dw_udc_dev *udev)
{
	u32 i;

	INIT_LIST_HEAD(&udev->gadget.ep_list);
	INIT_LIST_HEAD(&udev->gadget.ep0->ep_list);
	udev->ep0state = EP0_CTRL_IDLE;
	udev->int_cmd = 0;

	for (i = 0; i < udev->num_ep; i++) {
		struct dw_udc_ep *ep = &udev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udev->gadget.ep_list);

		ep->curr_chain_in = NULL;
		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
	}
}

static int udc_ep0_disable(struct dw_udc_ep *ep)
{
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	struct dw_udc_epin_regs *epregs;
	unsigned int tmp;

	udev = ep->udev;
	glob = udev->glob_base;
	epregs = ep->in_regs;

	udc_put_descrs(udev, ep->setup_ptr);
	writel(0, &ep->out_regs->setup_ptr);
	ep->setup_ptr = NULL;

	udc_put_descrs(udev, ep->curr_chain_in);
	writel(0, &ep->in_regs->desc_ptr);
	ep->desc_in_ptr = NULL;
	ep->curr_chain_in = 0;

	tmp = readl(&glob->endp_int_mask);
	tmp |= (ENDP0_IN_INT | ENDP0_OUT_INT);
	writel(tmp, &glob->endp_int_mask);

	writel(ENDP0_IN_INT | ENDP0_OUT_INT, &glob->endp_int);
	writel(ENDP_STATUS_IN | ENDP_STATUS_TDC, &ep->in_regs->status);

	tmp = readl(&ep->in_regs->control);
	tmp |= ENDP_CNTL_FLUSH;
	writel(tmp, &ep->in_regs->control);

	udc_put_descrs(udev, ep->desc_out_ptr);
	writel(0, &ep->out_regs->desc_ptr);
	ep->desc_out_ptr = NULL;
	ep->curr_chain_out = 0;

	writel(ENDP_STATUS_OUT_DATA, &ep->out_regs->status);

	return 0;
}

static void udc_disconnect(struct dw_udc_dev *udev)
{
	struct dw_udc_plug_regs *plug = udev->plug_base;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	u32 status;

	udev->gadget.speed = USB_SPEED_UNKNOWN;

	status = readl(&glob->dev_control);
	status |= DEV_CNTL_SD;
	writel(status, &glob->dev_control);

	/* put phy in reset */
	status = readl(&plug->status);
	status |= (PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
	status &= ~PLUG_STATUS_EN;
	writel(status, &plug->status);
}

static void udc_connect(struct dw_udc_dev *udev)
{
	struct dw_udc_plug_regs *plug = udev->plug_base;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	u32 status;

	status = readl(&plug->status);
	if (status & PLUG_STATUS_ATTACHED)
		status &= ~(PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
	else {
		/*
		 * USB cable detached
		 * Reset the PHY and switch the mode.
		 */
		status |= (PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
	}

	writel(status, &plug->status);

	status = readl(&plug->status);
	status |= PLUG_STATUS_EN;
	writel(status, &plug->status);

	status = readl(&glob->dev_control);
	status &= ~DEV_CNTL_SD;
	writel(status, &glob->dev_control);
}

/*
 * until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct dw_udc_dev *udev)
{
	u32 tmp, status;
#ifdef CONFIG_ARCH_SPEAR13XX
	u32 val;
#endif
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct usb_gadget_driver *driver;

	udev->gadget.speed = USB_SPEED_UNKNOWN;
	driver = (struct usb_gadget_driver *)udev->driver;

	writel(~0, &glob->dev_int_mask);
	writel(~0, &glob->endp_int_mask);

#ifdef CONFIG_ARCH_SPEAR13XX
	/* UDC RESET */
	val = readl(VA_PERIP1_SW_RST);
	val |= (1 << 11);
	writel(val, VA_PERIP1_SW_RST);

	udelay(10);

	val = readl(VA_PERIP1_SW_RST);
	val &= ~(1 << 11);
	writel(val, VA_PERIP1_SW_RST);
#endif
	tmp = DEV_CONF_HS_SPEED | DEV_CONF_REMWAKEUP |
		DEV_CONF_PHYINT_16 | DEV_CONF_CSR_PRG;
	writel(tmp, &glob->dev_conf);

	tmp = DEV_CNTL_BURSTEN | DEV_CNTL_SD |
		((7 * DEV_CNTL_BURSTLENU) & DEV_CNTL_BURSTLENMSK) |
		DEV_CNTL_DMAMODE | DEV_CNTL_TXDMAEN;

	writel(tmp, &glob->dev_control);

	udc_config_ep(udev, &udev->ep[0]);

	writel(DEV_INT_MSK, &glob->dev_int);
	writel(DEV_INT_SOF, &glob->dev_int_mask);

	udc_connect(udev);

	status = readl(&glob->dev_status);

	if ((status & DEV_STAT_ENUM) == DEV_STAT_ENUM_HS)
		udev->gadget.speed = USB_SPEED_HIGH;
	else
		udev->gadget.speed = USB_SPEED_FULL;
}

static void
req_done(struct dw_udc_ep *ep, struct dw_udc_request *req, int status)
{
	unsigned stopped;
	unsigned long flags;
	struct dw_udc_dev *udev;

	if (req == NULL)
		return;

	udev = ep->udev;

	if (req->mapped) {
		dma_unmap_single(udev->dev, req->req.dma, req->req.length,
				is_ep_in(ep) ? DMA_TO_DEVICE :
				DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}

	spin_lock_irqsave(&udev->lock, flags);
	list_del_init(&req->queue);

	req->req.status = status;

	/*
	 * Invoke the completion callback
	 * (don't modify queue heads during completion callback)
	 */
	stopped = ep->stopped;
	ep->stopped = 1;
	spin_unlock_irqrestore(&udev->lock, flags);

	req->req.complete(&ep->ep, &req->req);

	spin_lock_irqsave(&udev->lock, flags);
	ep->stopped = stopped;
	spin_unlock_irqrestore(&udev->lock, flags);

	DW_UDC_DBG(DBG_REQUESTS, "compt %s req %p stat %d buf %p len %u/%u\n",
			ep->ep.name, req, status, req->req.buf,
			req->req.actual, req->req.length);
}

static void cancel_all_req(struct dw_udc_ep *ep, int status)
{
	struct dw_udc_request *req;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next,
				struct dw_udc_request, queue);
		req_done(ep, req, status);
	}
}

static int
kick_dma(struct dw_udc_ep *ep, struct dw_udc_request *req,
		unsigned char is_in)
{
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_epin_regs *inregs = ep->in_regs;
	struct dw_udc_epout_regs *outregs = ep->out_regs;
	struct dw_udc_bulkd *head_descr, *desc_ptr;
	unsigned int dma_descr_num = 0;
	int epn = ep->addr & ~USB_DIR_IN;
	unsigned int rest = 0, tmp;
	unsigned char *bufp = NULL;
	struct list_head *ptr;
	u32 descp;

	if (req->req.length >= SZ_64K) {
		/* Lengths from 0 to 65535 (inclusive) are supported */
		dev_err(udev->dev, "invalid req length %u\n", req->req.length);
		return -EINVAL;
	}

	if (req->req.dma == DMA_ADDR_INVALID) {
		req->req.dma = dma_map_single(udev->dev, req->req.buf,
				req->req.length, is_in ? DMA_TO_DEVICE :
				DMA_FROM_DEVICE);
		req->mapped = 1;
		bufp = (unsigned char *)req->req.dma;
	}

	DW_UDC_DBG(DBG_GENERIC, "%s: req l/%u dma/%08x buf/%p %c%c%c\n",
			ep->ep.name, req->req.length, req->req.dma,
			req->req.buf,
			req->req.zero ? 'Z' : 'z',
			req->req.short_not_ok ? 'S' : 's',
			req->req.no_interrupt ? 'I' : 'i');

	dma_descr_num = DIV_ROUND_UP(req->req.length, ep->ep.maxpacket);
	if (dma_descr_num == 0)
		dma_descr_num = 1;

	rest = req->req.length;

	head_descr = desc_ptr =
		(struct dw_udc_bulkd *)udc_get_descrs(udev, dma_descr_num);
	if (!desc_ptr) {
		cancel_all_req(ep, -EOVERFLOW);
		return -ENOMEM;
	}

	if (is_in) {
		list_for_each(ptr, &head_descr->desc_list) {
			desc_ptr->status = cpu_to_le32(DMAUSB_HOSTRDY |
					ep->ep.maxpacket);
			desc_ptr->bufp = cpu_to_le32(bufp);
			bufp += ep->ep.maxpacket;
			rest -= ep->ep.maxpacket;
			desc_ptr = list_entry(ptr, struct dw_udc_bulkd,
					desc_list);
		}

		/* Last Descriptors */
		desc_ptr->bufp = cpu_to_le32(bufp);
		desc_ptr->status = cpu_to_le32(DMAUSB_LASTDESCR |
				(rest & DMAUSB_LEN_MASK));

		ep->curr_chain_in = head_descr;
		ep->desc_in_ptr = head_descr;
		writel(head_descr->dma_addr,
				&inregs->desc_ptr);

		/*
		 * unmask in endpoint interrupt
		 * which was masked when we received IN token
		 */
		tmp = readl(&glob->endp_int_mask);
		tmp &= ~(1 << epn);
		writel(tmp, &glob->endp_int_mask);

	} else {
		list_for_each(ptr, &head_descr->desc_list) {
			desc_ptr->status = cpu_to_le32(DMAUSB_HOSTRDY);
			desc_ptr->bufp = cpu_to_le32(bufp);
			bufp += ep->ep.maxpacket;
			desc_ptr = list_entry(ptr, struct dw_udc_bulkd,
					desc_list);
		}

		desc_ptr->bufp = cpu_to_le32(bufp);
		desc_ptr->status = cpu_to_le32(DMAUSB_LASTDESCR |
				DMAUSB_HOSTRDY);

		descp = head_descr->dma_addr;
		writel(descp, &outregs->desc_ptr);
		ep->desc_out_ptr = head_descr;
	}

	wmb();
	return 0;
}

/*
 * Cancel all USB request still pending for every endpoint and
 * disconnect the gadget driver.
 */
static void
stop_activity(struct dw_udc_dev *udev, struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (udev->gadget.speed == USB_SPEED_UNKNOWN)
		return;

	udev->gadget.speed = USB_SPEED_UNKNOWN;

	del_timer_sync(&udev->out_ep_timer);

	if (driver)
		driver->disconnect(&udev->gadget);

	/* drain out if any buffer remains pending on any ep */
	for (i = 0; i < udev->num_ep; i++) {
		struct dw_udc_ep *ep = &udev->ep[i];
		cancel_all_req(ep, -ESHUTDOWN);
	}

	udc_ep0_disable(&udev->ep[0]);
	/* re-init driver-visible data structures */
	udc_reinit(udev);
}

static void udc_handle_epn_out_int(struct dw_udc_ep *ep)
{
	int epn = ep->addr & ~USB_DIR_IN;
	unsigned int epn_bit = (1 << (epn + 16));
	struct dw_udc_bulkd *tmp_desc;
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_epout_regs *epregs = ep->out_regs;
	struct dw_udc_request *req;
	u32 tmp;

	writel(epn_bit, &glob->endp_int);

	tmp = readl(&epregs->status);

	if (!(tmp & ENDP_STATUS_OUT_DATA))
		return;

	show_chain(ep);

	writel(ENDP_STATUS_OUT_DATA, &epregs->status);

	/* search in the current chain the endpoint marked as LAST */
	tmp_desc = ep->desc_out_ptr;
	ep->last_updated = jiffies;

	while (tmp_desc != 0x0) {
		if (le32_to_cpu(tmp_desc->status) & DMAUSB_LASTDESCR)
			break;
		tmp_desc = list_entry(tmp_desc->desc_list.next,
				struct dw_udc_bulkd,
				desc_list);
	}

	if (tmp_desc && (le32_to_cpu(tmp_desc->status) & DMAUSB_DMADONE)) {
		ep->curr_chain_out = ep->desc_out_ptr;
		writel(0, &epregs->desc_ptr);
		ep->desc_out_ptr = NULL;

		/*
		 * For every non-control endpoints we must invoke
		 * req_done().
		 */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct
					dw_udc_request, queue);
			req->req.actual = tmp_desc->status & DMAUSB_LEN_MASK;

			udc_put_descrs(udev, ep->curr_chain_out);
			ep->curr_chain_out = 0;

			req_done(ep, req, 0);
		}
	} else {
		/* Move Descriptors into FREE list */
		udc_put_descrs(udev, ep->desc_out_ptr);
		writel(0, &epregs->desc_ptr);
		ep->desc_out_ptr = NULL;
	}

	/* restart i/o */
	if (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct dw_udc_request, queue);
		if (kick_dma(ep, req, 0) == 0) {
			tmp = readl(&epregs->control);
			tmp |= ENDP_CNTL_RRDY;
			writel(tmp, &epregs->control);
		}
	}
}

static void udc_rescan_isoc_desc(struct dw_udc_ep *ep)
{
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	struct dw_udc_bulkd *head_desc, *desc;
	u32 sof, frame, tmp;

	udev = ep->udev;
	glob = udev->glob_base;

	head_desc = ep->curr_chain_in;
	if (!head_desc)
		return;

	tmp = readl(&glob->dev_status);
	sof = (tmp & 0xffe00000) >> 21;
	frame = (tmp & 0x1c0000) >> 18;

	list_for_each_entry(desc, &head_desc->desc_list, desc_list)
		desc->status |= cpu_to_le32((sof << 19) | (frame << 16));
	wmb();
}

static void udc_handle_epn_in_int(struct dw_udc_ep *ep)
{
	int epn = ep->addr & ~USB_DIR_IN;
	unsigned int epn_bit = (1 << epn);
	struct dw_udc_bulkd *desc_ptr, *head_desc;
	struct dw_udc_request *req;
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_epin_regs *epregs = ep->in_regs;
	u32 tmp, status;

	writel(epn_bit, &glob->endp_int);

	status = readl(&epregs->status);
	/* If DMA completed transfer to TxFIFO */
	if (status & ENDP_STATUS_TDC) {
		show_chain(ep);
		/* Interrupt ack */
		writel(ENDP_STATUS_TDC, &epregs->status);

		head_desc = ep->desc_in_ptr;
		/* the DMA completed the current descriptor; free it !! */
		udc_put_descrs(udev, head_desc);
		writel(0, &epregs->desc_ptr);
		ep->desc_in_ptr = 0;
		head_desc = NULL;
		ep->curr_chain_in = 0;

		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					struct dw_udc_request, queue);
			req_done(ep, req, 0);
		}

		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					struct dw_udc_request, queue);
			kick_dma(ep, req, 1);
		}
	}

	/* The current IN endpoint received a USB IN request. */
	if (status & ENDP_STATUS_IN) {
		writel(ENDP_STATUS_IN, &epregs->status);

		desc_ptr = ep->curr_chain_in;

		if (desc_ptr != 0x0) {
			ep->curr_chain_in = 0;
			if (ep->attrib == USB_ENDPOINT_XFER_ISOC) {
				/*
				 * update sof and frame number in isoch
				 * descriptors
				 */
				udc_rescan_isoc_desc(ep);
			}

			tmp = readl(&(epregs->control));
			tmp |= ENDP_CNTL_POLL;
			writel(tmp, &(epregs->control));

			DW_UDC_DBG(DBG_ADDRESS, "desc_ptr->bufp = %08x\n",
					le32_to_cpu(desc_ptr->bufp));
		} else {
			/*
			 * This check is important to see if TDC is
			 * pending
			 * mask interrupt in case TDC is not pending
			 * as we don't want more IN Tokens to
			 * bother us for this endpoint
			 */

			if (!ep->desc_in_ptr) {
				tmp = readl(&glob->endp_int_mask);
				tmp |= (1 << epn);
				writel(tmp, &glob->endp_int_mask);
			}
		}
	}
}

static int dw_ep_disable(struct usb_ep *_ep)
{
	struct dw_udc_ep *ep;
	unsigned long flags;
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	struct dw_udc_epin_regs *epregs;
	unsigned int epn, tmp, stopped;

	ep = container_of(_ep, struct dw_udc_ep, ep);
	udev = ep->udev;
	glob = udev->glob_base;
	epregs = ep->in_regs;
	epn = ep->addr & ~USB_DIR_IN;
	stopped = ep->stopped;
	ep->stopped = 1;

	if (!_ep || !ep->desc) {
		dev_err(udev->dev, "%s, %s not enabled\n", __func__,
				_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	if (!is_ep_in(ep)) {
		tmp = readl(&ep->out_regs->control);
		tmp |= ENDP_CNTL_CLOSEDESC;
		writel(tmp, &ep->out_regs->control);

		/*
		 * Verify, will this raise an interrupt
		 * so that we should call intr handler
		 */
		udc_handle_epn_out_int(ep);
	} else {
		/*
		 * When ep_disable is called with partially served IN descriptor
		 * chain, the TxFIFO must be flushed till DMA completely serves
		 * the chain, otherwise DMA would be in in-consistent state.
		 */

		while ((tmp = readl(&(epregs->control))) & ENDP_CNTL_POLL) {
			/* flush TX FIFO */
			tmp |= ENDP_CNTL_FLUSH;
			writel(tmp, &epregs->control);
			wmb();
		}
		tmp |= ENDP_CNTL_FLUSH;
		writel(tmp, &epregs->control);
		wmb();
		/*
		 * Verify, will this raise an interrupt
		 * so that we should call intr handler
		 */
		udc_handle_epn_in_int(ep);
	}

	cancel_all_req(ep, -ESHUTDOWN);

	spin_lock_irqsave(&ep->udev->lock, flags);
	ep->desc = 0;
	ep->stopped = stopped;
	ep->config_req = 0;

	if (is_ep_in(ep)) {
		udc_put_descrs(udev, ep->desc_in_ptr);

		writel(0, &ep->in_regs->desc_ptr);
		ep->desc_in_ptr = NULL;
		ep->curr_chain_in = 0;

		/* clear and mask endpoint intrs when disabled */
		tmp = readl(&glob->endp_int_mask);
		tmp |= (ENDP0_IN_INT << epn);
		writel(tmp, &glob->endp_int_mask);

		/* clear intrs */
		writel((ENDP0_IN_INT << epn), &glob->endp_int);
		writel((ENDP_STATUS_IN | ENDP_STATUS_TDC),
				&ep->in_regs->status);

	} else {
		udc_put_descrs(udev, ep->desc_out_ptr);

		writel(0, &ep->out_regs->desc_ptr);
		ep->desc_out_ptr = NULL;
		ep->curr_chain_out = 0;

		/* disable RRDY when endpoint disabled */
		tmp = readl(&ep->out_regs->control);
		tmp &= ~ENDP_CNTL_RRDY;
		writel(tmp, &ep->out_regs->control);

		tmp = readl(&glob->endp_int_mask);
		tmp |= (ENDP0_OUT_INT << epn);
		writel(tmp, &glob->endp_int_mask);

		writel((ENDP0_OUT_INT << epn), &glob->endp_int);
		writel(ENDP_STATUS_OUT_DATA, &ep->out_regs->status);

	}

	spin_unlock_irqrestore(&ep->udev->lock, flags);

	return 0;
}

static struct usb_request *dw_ep_alloc_request(struct usb_ep *_ep,
		gfp_t gfp_flags)
{
	struct dw_udc_request *req;

	if (!_ep)
		return NULL;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;
	INIT_LIST_HEAD(&req->queue);
	req->req.dma = DMA_ADDR_INVALID;

	return &req->req;
}

static void dw_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct dw_udc_request *req;

	if (!_ep)
		return;

	req = container_of(_req, struct dw_udc_request, req);
	WARN_ON(!list_empty(&req->queue));
	list_del_init(&req->queue);
	kfree(req);
}

static int dw_ep_queue(struct usb_ep *_ep,
		struct usb_request *_req, gfp_t gfp_flags)
{
	struct dw_udc_epout_regs *out_regs;
	struct dw_udc_epin_regs *in_regs;
	struct dw_udc_glob_regs *glob;
	struct dw_udc_dev *udev;
	struct dw_udc_request *req;
	struct dw_udc_ep *ep;
	unsigned char is_in;
	unsigned long flags;
	int rcv_rdy_flag = 0;	/* flag to chk whether kick dma called or not */
	u32 tmp;

	req = container_of(_req, struct dw_udc_request, req);
	ep = container_of(_ep, struct dw_udc_ep, ep);

	in_regs = ep->in_regs;
	out_regs = ep->out_regs;
	is_in = is_ep_in(ep);
	udev = ep->udev;
	glob = udev->glob_base;

	if (_req->status == -EINPROGRESS)
		return -EINPROGRESS;

	spin_lock_irqsave(&udev->lock, flags);

	DW_UDC_DBG(DBG_REQUESTS, "%s:%s req %p: length %d buf %p\n", __func__,
			ep->ep.name, _req, _req->length, _req->buf);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/*
	 * We can keep only 1 request at a given time in control endpoint.
	 * Free if something already queued
	 */

	if ((ep->attrib == USB_ENDPOINT_XFER_CONTROL) &&
			!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct dw_udc_request, queue);
		spin_unlock_irqrestore(&udev->lock, flags);
		req_done(ep, req, 0);
		spin_lock_irqsave(&udev->lock, flags);
	}

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->attrib == USB_ENDPOINT_XFER_CONTROL) {
			switch (udev->ep0state) {
			case EP0_DATA_IN_STAGE:
				if (kick_dma(ep, req, 1) == 0)
					udc_clear_nak(ep);
				break;
			case EP0_DATA_OUT_STAGE:
				/* why is the desc released */
				udc_put_descrs(udev, ep->desc_out_ptr);
				ep->desc_out_ptr = 0x00;

				if (kick_dma(ep, req, 0) == 0) {
					udc_clear_nak(ep);
					rcv_rdy_flag = 1;
				}
				break;
			case EP0_CTRL_IDLE:
				if (req->req.length == 0) {
					spin_unlock_irqrestore(&udev->lock,
							flags);
					req_done(ep, req, 0);
					spin_lock_irqsave(&udev->lock, flags);
					req = 0;
				}
				break;
			case EP0_ACK_SETCONF_INTER_DELAYED:
				if (req->req.length == 0) {
					spin_unlock_irqrestore(&udev->lock,
							flags);
					req_done(ep, req, 0);
					spin_lock_irqsave(&udev->lock, flags);
					/* send ack to the host */
					tmp = readl(&glob->dev_control);
					tmp |= DEV_CNTL_CSR_DONE;
					writel(tmp, &glob->dev_control);
					udev->ep0state = EP0_CTRL_IDLE;
					udev->int_cmd = 0;

					spin_unlock_irqrestore(&udev->lock,
							flags);

					return 0;
				} else {
					dev_err(udev->dev, "got a request while\
					in state EP0_ACK_SETCONF_INTER_DELAYED \
					of %d\n", req->req.length);
					list_del_init(&req->queue);
					return -EINVAL;
				}
				break;

			default:
				dev_err(udev->dev, "Queued req while in state \
						%d\n", udev->ep0state);
				break;
			}
		} else {
			if (kick_dma(ep, req, is_in) == 0) {
				if (!is_in)
					rcv_rdy_flag = 1;
				udc_clear_nak(ep);
			}
		}
	}

	if (ep->stopped) {
		DW_UDC_DBG(DBG_REQUESTS, "Req %p queued while ep %d stopped\n",
				req, ep - &udev->ep[0]);

	}

	/* dma irq handler advances the queue. */
	if (likely(req != 0))
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&udev->lock, flags);

	show_queues(ep);

	/* finally enable DMA reception */
	if (rcv_rdy_flag) {
		tmp = readl(&(out_regs->control));
		tmp |= ENDP_CNTL_RRDY;
		writel(tmp, &(out_regs->control));
	}

	return 0;
}

static int dw_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct dw_udc_ep *ep;
	unsigned long flags;
	struct dw_udc_request *req;
	struct dw_udc_request *request;
	struct dw_udc_dev *udev;
	u32 tmp;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct dw_udc_ep, ep);
	request = container_of(_req, struct dw_udc_request, req);

	if (!_ep || ep->ep.name == driver_name)
		return -EINVAL;
	udev = ep->udev;

	spin_lock_irqsave(&udev->lock, flags);

	/* make sure it's actually queued on this endpoint */

	list_for_each_entry(req, &ep->queue, queue) {
		if (&(req->req) == _req)
			break;
	}
	if (&(req->req) != _req) {
		spin_unlock_irqrestore(&udev->lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&udev->lock, flags);
	req_done(ep, req, -ECONNRESET);
	spin_lock_irqsave(&udev->lock, flags);

	if (is_ep_in(ep)) {
		udc_put_descrs(udev, ep->curr_chain_in);
		writel(0, &ep->in_regs->desc_ptr);
		ep->desc_in_ptr = NULL;
		ep->curr_chain_in = 0;

		/* Flush transmit FIFO */
		tmp = readl(&ep->in_regs->control);
		tmp |= ENDP_CNTL_FLUSH;
		writel(tmp, &ep->in_regs->control);

	} else {
		udc_put_descrs(udev, ep->curr_chain_out);
		writel(0, &ep->out_regs->desc_ptr);
		ep->desc_out_ptr = NULL;
		ep->curr_chain_out = 0;
	}

	spin_unlock_irqrestore(&udev->lock, flags);
	return 0;
}

static int dw_ep_set_halt(struct usb_ep *_ep, int halt)
{
	struct dw_udc_epin_regs *inregs;
	struct dw_udc_epout_regs *outregs;
	struct dw_udc_dev *udev;
	struct dw_udc_ep *ep;
	unsigned long flags;
	int status = 0;
	u32 tmp;

	ep = container_of(_ep, struct dw_udc_ep, ep);
	udev = ep->udev;
	inregs = ep->in_regs;
	outregs = ep->out_regs;

	spin_lock_irqsave(&udev->lock, flags);

	if (halt) {
		if (!list_empty(&ep->queue)) {
			dev_err(udev->dev, "DONOT STAL IF LIST IS NOT EMPTY\n");
			status = -EAGAIN;
			goto done;
		}
		/* STALL the endpoint */
		if (is_ep_in(ep)) {
			tmp = readl(&inregs->control);
			tmp |= ENDP_CNTL_STALL;
			writel(tmp, &inregs->control);
		} else {
			tmp = readl(&outregs->control);
			tmp |= ENDP_CNTL_STALL;
			writel(tmp, &outregs->control);
		}
	} else {
		/*
		 * on reception of clear feature the udc subsystem clears
		 * the stall bit and set the NAK bit.
		 * when clear_halt is issued by gadget the NAK bit gets cleared
		 */
		if (is_ep_in(ep)) {
			tmp = readl(&inregs->control);
			tmp |= ENDP_CNTL_CNAK;
			tmp &= ~ENDP_CNTL_SNAK;
			writel(tmp, &inregs->control);
		} else {
			tmp = readl(&outregs->control);
			tmp |= ENDP_CNTL_CNAK;
			tmp &= ~ENDP_CNTL_SNAK;
			writel(tmp, &outregs->control);
		}
	}
done:
	spin_unlock_irqrestore(&ep->udev->lock, flags);
	return status;
}

static int dw_ep_fifo_status(struct usb_ep *_ep)
{
	return -EOPNOTSUPP;
}

static void dw_ep_fifo_flush(struct usb_ep *_ep)
{
	struct dw_udc_epin_regs *regs;
	struct dw_udc_dev *udev;
	struct dw_udc_ep *ep;
	u32 tmp;

	ep = container_of(_ep, struct dw_udc_ep, ep);

	udev = ep->udev;
	regs = ep->in_regs;

	if (is_ep_in(ep)) {
		tmp = readl(&regs->control);
		tmp |= ENDP_CNTL_FLUSH;
		writel(tmp, &regs->control);
	}
}

static int
dw_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct dw_udc_ep *ep;
	struct dw_udc_dev *udev;
	u16 maxpacket;
	unsigned long flags;

	ep = container_of(_ep, struct dw_udc_ep, ep);
	udev = ep->udev;

	if (!_ep || !desc || ep->desc
			|| desc->bDescriptorType != USB_DT_ENDPOINT) {
		dev_err(udev->dev, "descriptor\n");
		return -EINVAL;
	}

	if (ep == &udev->ep[0])
		return -EINVAL;

	if (!udev->driver || udev->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(udev->dev, "unknown error\n");
		return -ESHUTDOWN;
	}

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_BULK:
		ep->attrib = USB_ENDPOINT_XFER_BULK;
		break;
	case USB_ENDPOINT_XFER_INT:
		ep->attrib = USB_ENDPOINT_XFER_INT;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		ep->attrib = USB_ENDPOINT_XFER_ISOC;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&udev->lock, flags);

	maxpacket = le16_to_cpu(get_unaligned(&desc->wMaxPacketSize)) & 0x7ff;

	ep->ep.maxpacket = maxpacket;
	ep->stopped = 0;
	ep->config_req = 0;
	ep->desc = desc;

	udc_config_ep(udev, ep);
	udc_config_dma(ep);

	DW_UDC_DBG(DBG_ENDPOINT, "enable endpoint num.%d %s %s maxpacket %u\n",
			ep - udev->ep, ep->ep.name, is_ep_in(ep) ? "IN" : "OUT",
			maxpacket);

	spin_unlock_irqrestore(&udev->lock, flags);
	return 0;
}

/* Out Endpoint Timeout handler */
static void udc_ep_timeout(unsigned long data)
{
	struct dw_udc_dev *udev = (struct dw_udc_dev *)data;
	struct dw_udc_ep *ep;
	u32 tmp = 0, i;

	for (i = 0; i < udev->num_ep; i++) {
		ep = &udev->ep[i];

		if (is_ep_in(ep))
			continue;

		if (time_after(jiffies, ep->last_updated + HZ)) {
			/*
			 * find out whether some thing already has
			 * been received by this endpoint
			 */
			if (ep->desc_out_ptr)
				tmp = le32_to_cpu(ep->desc_out_ptr->status);

			if (tmp & DMAUSB_DMABSY) {
				/* ask DMA to close this descriptor */
				tmp = readl(&ep->out_regs->control);
				tmp |= ENDP_CNTL_CLOSEDESC;
				writel(tmp, &ep->out_regs->control);
			}
		}
	}

	/* schedule again */
	mod_timer(&udev->out_ep_timer, jiffies + HZ);
}

static void udc_handle_ep0_in(struct dw_udc_ep *ep)
{
	struct dw_udc_bulkd *desc_ptr, *head_desc;

	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_epin_regs *epregs = ep->in_regs;
	u32 tmp, status, descp;

	status = readl(&epregs->status);

	/* If DMA completed transfer to TxFIFO */
	if (status & ENDP_STATUS_TDC) {
		writel(ENDP_STATUS_TDC, &epregs->status);

		head_desc = ep->desc_in_ptr;
		/* the DMA completed the current descriptor; free it !! */
		udc_put_descrs(udev, head_desc);

		writel(0, &epregs->desc_ptr);
		ep->desc_in_ptr = 0;
		head_desc = NULL;
		ep->curr_chain_in = 0;

		switch (udev->ep0state) {
		case EP0_DATA_IN_STAGE:
			udev->ep0state = EP0_STATUS_OUT_STAGE;
			DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n",
					udev->ep0state);
			break;
		case EP0_STATUS_IN_STAGE:
			udev->ep0state = EP0_CTRL_IDLE;
			DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n",
					udev->ep0state);
			break;
		default:
			dev_err(udev->dev, "Data tx from ep0 in state %d\n",
					udev->ep0state);
			udc_stall_ep(ep);
			break;
		}
	}
	/* The current IN endpoint received a USB IN request. */
	status = readl(&epregs->status);
	if (status & ENDP_STATUS_IN) {
		/* clear status IN bit */
		writel(ENDP_STATUS_IN, &epregs->status);

		desc_ptr = ep->curr_chain_in;
		ep->curr_chain_in = 0;

		if (desc_ptr != 0x0) {
			descp = desc_ptr->dma_addr;
			DW_UDC_DBG(DBG_ADDRESS, "descriptor pointer = %08x\n",
					descp);
			writel(descp, &(epregs->desc_ptr));
			ep->desc_in_ptr = desc_ptr;

			tmp = readl(&(epregs->control));
			tmp |= ENDP_CNTL_POLL;
			writel(tmp, &(epregs->control));

			DW_UDC_DBG(DBG_ADDRESS, "desc_ptr->bufp = %08x\n",
					le32_to_cpu(desc_ptr->bufp));

		} else {
			/* This check is important */
			if (!ep->desc_in_ptr) {
				/* mask interrupt */
				tmp = readl(&glob->endp_int_mask);
				tmp |= 1;
				writel(tmp, &glob->endp_int_mask);
			}
		}
	}
}

static void udc_handle_ep0_out_data(struct dw_udc_ep *ep)
{
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_epout_regs *ep0_oregs = ep->out_regs;
	struct dw_udc_ep *ep0 = ep;
	struct dw_udc_request *req;
	struct dw_udc_bulkd *descp;
	u32 tmp;

again:
	/*
	 * Not a setup packet. Remember that here we are only managing the
	 * ep0 OUT interrupt case.
	 */

	ep->last_updated = jiffies;	/* update when last touched */

	if (list_empty(&ep->queue)) {
		/* following only for recovery */
		descp = ep0->desc_out_ptr;
		udc_put_descrs(udev, descp);
		ep0->desc_out_ptr = 0;
		writel(0, &(ep0_oregs->desc_ptr));

		dev_err(udev->dev, "OUT req rcvd while list empty for ep %d, \
				data discarded\n", ep - &udev->ep[0]);
		goto req_err;
	}
	req = list_entry(ep0->queue.next, struct dw_udc_request, queue);

	if (udev->ep0state == EP0_STATUS_OUT_STAGE) {
		/* status out stage */
		req_done(ep0, req, 0);

		descp = ep0->desc_out_ptr;
		ep0->desc_out_ptr = 0x00;
		udc_put_descrs(udev, descp);
		udev->ep0state = EP0_CTRL_IDLE;
		writel(0, &(ep0_oregs->desc_ptr));
		DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n", udev->ep0state);

	} else if (udev->ep0state == EP0_DATA_OUT_STAGE) {
		/* handle data received in data out stage */
		udev->ep0state = EP0_CTRL_IDLE;

		descp = ep0->desc_out_ptr;
		while (descp != 0x0) {
			if (le32_to_cpu(descp->status) & DMAUSB_LASTDESCR)
				break;
			descp = list_entry(descp->desc_list.next,
					struct dw_udc_bulkd, desc_list);
		}

		if (descp && (le32_to_cpu(descp->status) & DMAUSB_DMADONE)) {
			req->req.actual = descp->status & DMAUSB_LEN_MASK;
			req_done(ep0, req, 0);
		}

		descp = ep0->desc_out_ptr;
		udc_put_descrs(udev, descp);
		ep0->desc_out_ptr = 0;
		writel(0, &(ep0_oregs->desc_ptr));

		DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n", udev->ep0state);

	} else if (udev->ep0state == EP0_DATA_IN_STAGE) {
		/*
		 * This should not happen in normal cases Following is a
		 * workaround and not a clean approach it was done because
		 * we are reaching here in ep0 state as IN DATA which means
		 * the we are handling OUT STS intr before IN TDC intr and
		 * this leads to ep0 state as IN DATA where as it should be
		 * STS OUT
		 */
		if (readl(&ep->in_regs->status) & ENDP_STATUS_TDC) {
			udc_handle_ep0_in(ep);
			goto again;
		} else
			dev_err(udev->dev, "NO TDC Intr present, \
					it should have been\n");
	} else {
		dev_err(udev->dev, "Should never reach here\n");
		req_done(ep0, req, 0);
	}

req_err:
	/*
	 * Set new descriptor chain (and re-enable the DMA if the case)
	 */
	descp = udc_get_descrs(udev, 1);
	if (descp != NULL) {
		DW_UDC_DBG(DBG_ADDRESS, "descriptor pointer = %p\n", descp);
		writel(descp->dma_addr, &(ep0_oregs->desc_ptr));
		ep0->desc_out_ptr = descp;

		tmp = readl(&ep0_oregs->control);
		tmp |= ENDP_CNTL_RRDY;
		writel(tmp, &ep0_oregs->control);

		tmp = readl(&glob->dev_control);
		tmp |= DEV_CNTL_RXDMAEN;
		writel(tmp, &glob->dev_control);

	} else {
		dev_err(udev->dev, "Could not allocate memory\n");
		writel(0, &ep0_oregs->desc_ptr);
		ep0->desc_out_ptr = NULL;
	}
}

static void udc_handle_ep0_setup(struct dw_udc_ep *ep)
{
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_setupd *setup_desc;
	struct usb_ctrlrequest u_ctrl_req;
	struct dw_udc_ep *ep0 = ep;
	struct dw_udc_epout_regs *ep0_oregs = ep0->out_regs;
	unsigned char *setup_data;
	u32 tmp, status;
	int ret = 0;

	setup_desc = (struct dw_udc_setupd *)(ep0->setup_ptr);
	setup_data = (unsigned char *)(&(setup_desc->data1));

	status = le32_to_cpu(setup_desc->status);
	if ((status & DMAUSB_BS_MASK) == DMAUSB_DMADONE) {
		/* get setup data */
		memcpy(&u_ctrl_req, setup_data, 8);
		/* swap some fields in a big_endian architecture */
		le16_to_cpus(&u_ctrl_req.wValue);
		le16_to_cpus(&u_ctrl_req.wIndex);
		le16_to_cpus(&u_ctrl_req.wLength);

		DW_UDC_DBG(DBG_REQUESTS, "SETUP %02x.%02x v%04x i%04x l%04x\n",
				u_ctrl_req.bRequestType, u_ctrl_req.bRequest,
				u_ctrl_req.wValue, u_ctrl_req.wIndex,
				u_ctrl_req.wLength);

		/* If ep0 was not idle, stall it */
		if (udev->ep0state != EP0_CTRL_IDLE) {
			/* STALL the endpoint */
			dev_err(udev->dev, "Setup packet recvd while not idle:"
					"stall ep0\n");
			udc_stall_ep(ep0);
			return;
		}

		if (u_ctrl_req.bRequestType & USB_DIR_IN)
			udev->ep0state = EP0_DATA_IN_STAGE;
		else
			udev->ep0state = EP0_DATA_OUT_STAGE;

		DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n", udev->ep0state);

		if (u_ctrl_req.wLength == 0) {
			/*
			 * This is because Status IN is handled internally
			 */
			udev->ep0state = EP0_CTRL_IDLE;
		}

		/* Delegate request fulfillment to gadget driver */
		if (udev->driver != NULL && udev->driver->setup != NULL) {
			/* call the gadget driver setup() routine */
			ret = udev->driver->setup(&udev->gadget, &u_ctrl_req);
			if (ret < 0) {
				udc_stall_ep(ep0);
				setup_desc->status =
					cpu_to_le32(DMAUSB_HOSTRDY);
				tmp = readl(&ep0_oregs->control);
				tmp |= ENDP_CNTL_RRDY;
				writel(tmp, &ep0_oregs->control);
				udev->ep0state = EP0_CTRL_IDLE;

				return;
			}
		} else {
			dev_err(udev->dev, "Failed to call gadget setup()\n");
			udev->ep0state = EP0_CTRL_IDLE;
		}

		setup_desc->status = cpu_to_le32(DMAUSB_HOSTRDY);

		if ((udev->ep0state == EP0_CTRL_IDLE) || (udev->ep0state ==
					EP0_DATA_IN_STAGE)) {
			tmp = readl(&ep0_oregs->control);
			tmp |= ENDP_CNTL_RRDY;
			writel(tmp, &ep0_oregs->control);
		}

		udc_clear_nak(ep0);
		/*
		 * The DMA was automatically disabled
		 * at the end of the transfer
		 */
		tmp = readl(&glob->dev_control);
		tmp |= DEV_CNTL_RXDMAEN;
		writel(tmp, &glob->dev_control);
	}
}

static void udc_handle_ep0_out(struct dw_udc_ep *ep)
{
	struct dw_udc_dev *udev = ep->udev;
	struct dw_udc_epout_regs *regs = ep->out_regs;
	u32 status, condition;

	/* Check if a setup packet was received */
	status = readl(&regs->status);
	condition = status & ENDP_STATUS_OUTMSK;

	/*
	 * Setup packets can still be received after an internally
	 * decoded command, so we manage them later.
	 */
	if (udev->int_cmd && condition == ENDP_STATUS_OUT_SETUP)
		return;

	if (!condition || (condition & ENDP_STATUS_OUT_DATA)) {
		writel(ENDP_STATUS_OUT_DATA, &regs->status);
		udc_handle_ep0_out_data(ep);

	} else if (condition & ENDP_STATUS_OUT_SETUP) {
		writel(ENDP_STATUS_OUT_SETUP, &regs->status);
		if (udev->ep0state == EP0_STATUS_OUT_STAGE) {
			/* First handle status data stage */
			udc_handle_ep0_out_data(ep);
		}
		udc_handle_ep0_setup(ep);

	} else {
		dev_err(udev->dev, "Out Int received, but OUT field of status\n"
				"register was neither data nor setup, but %d\n",
				condition);
	}
}

static void udc_handle_ep0_irq(struct dw_udc_dev *udev)
{
	struct dw_udc_ep *ep0 = &udev->ep[0];
	struct dw_udc_glob_regs *glob = udev->glob_base;
	u32 tmp;

	/*
	 * It is necessary to serve in or out ep0 interrupt first,
	 * depending on ep0state, in order to make the ep0 state
	 * machine evolve the right way.
	 */

	show_registers(ep0);
	tmp = readl(&glob->endp_int);

	switch (udev->ep0state) {
	case EP0_DATA_IN_STAGE:
	case EP0_STATUS_IN_STAGE:
		if (tmp & ENDP0_IN_INT) {
			writel(ENDP0_IN_INT, &glob->endp_int);
			udc_handle_ep0_in(ep0);
		}

		tmp = readl(&glob->endp_int);
		if (tmp & ENDP0_OUT_INT) {
			writel(ENDP0_OUT_INT, &glob->endp_int);
			udc_handle_ep0_out(ep0);
		}
		break;
	case EP0_DATA_OUT_STAGE:
	case EP0_STATUS_OUT_STAGE:
	case EP0_CTRL_IDLE:
	default:
		if (tmp & ENDP0_OUT_INT) {
			writel(ENDP0_OUT_INT, &glob->endp_int);
			udc_handle_ep0_out(ep0);
		}

		tmp = readl(&glob->endp_int);
		if (tmp & ENDP0_IN_INT) {
			writel(ENDP0_IN_INT, &glob->endp_int);
			udc_handle_ep0_in(ep0);
		}
		break;
	}
}

/*
 * USB Interrupt Handler
 */
static void
udc_handle_internal_cmds(struct dw_udc_dev *udev,
		struct usb_ctrlrequest u_ctrl_req)
{
	struct dw_udc_ep *ep0 = &(udev->ep[0]);
	/*
	 * Manage pending ints. We are sure that no more requests are issued
	 * by the host, because the hw NAKs them
	 */
	udev->int_cmd = 1;

	udc_handle_ep0_irq(udev);

	cancel_all_req(ep0, -EPROTO);

	/* The STATUS_IN transaction will be discarded. */
	DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n", udev->ep0state);

	/*
	 * change the state of ep0 as EP0_ACK_SETCONF_INTER_DELAYED when
	 * set interface or set configuration command received.
	 * DEV_CNTL_CSR_DONE bit is not set here so that udc will not
	 * sent ack.we would complete the request from ep_queue
	 */

	udev->ep0state = EP0_ACK_SETCONF_INTER_DELAYED;
	if (udev->driver != NULL && udev->driver->setup != NULL)
		udev->driver->setup(&udev->gadget, &u_ctrl_req);
}

static void udc_handle_ep_irq(struct dw_udc_dev *udev)
{
	u32 i;
	unsigned int endp_int;
	int epn;
	unsigned int epn_bit;

	endp_int = readl(&udev->glob_base->endp_int);

	/* ep0 is handled by seperate function */
	/* Let's handle all the other endpoints */
	for (i = 1; i < udev->num_ep; i++) {
		struct dw_udc_ep *ep = &udev->ep[i];
		epn = ep->addr & ~USB_DIR_IN;
		epn_bit = (1 << epn);

		if (is_ep_in(ep)) {
			if (endp_int & epn_bit) {
				show_registers(ep);
				udc_handle_epn_in_int(ep);
			}
		} else {
			if (endp_int & (epn_bit << 16)) {
				show_registers(ep);
				udc_handle_epn_out_int(ep);
			}
		}
	}
}

static void udc_handle_plug_irq(struct dw_udc_dev *udev)
{
	struct dw_udc_plug_regs *plug = udev->plug_base;
	unsigned int tmp;

	tmp = readl(&plug->status);
	if (tmp & PLUG_STATUS_ATTACHED) {
		/* USB cable attached
		 * Turn off PHY reset bit (PLUG detect).
		 * Switch PHY opmode to normal operation (PLUG detect).
		 */
		tmp &= ~(PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
	} else {
		/* USB cable detached
		 * Reset the PHY and switch the mode.
		 */
		tmp |= (PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
	}
	writel(tmp, &plug->status);
	return;
}

static void udc_handle_device_irq(struct dw_udc_dev *udev)
{
	struct dw_udc_glob_regs *glob = udev->glob_base;
	struct dw_udc_plug_regs *plug = udev->plug_base;
	struct dw_udc_epin_regs *ep0regs = udev->epin_base;
	u32 tmp, dev_int, status;

	dev_int = readl(&glob->dev_int);

	if (!dev_int)
		return;

	/* USB Reset detected on cable */
	if (dev_int & DEV_INT_USBRESET) {
		/* Clear interrupt */
		writel(DEV_INT_USBRESET, &glob->dev_int);

		DW_UDC_DBG(DBG_INTS, "USB RESET\n");

		/* all endpoint irq disabled */
		writel(~0x0, &glob->endp_int);
		writel(~0x0, &glob->endp_int_mask);

		/* Disable RX DMA */
		tmp = readl(&glob->dev_control);
		tmp &= ~DEV_CNTL_RXDMAEN;
		writel(tmp, &glob->dev_control);

		stop_activity(udev, udev->driver);

		/*
		 * Turn off PHY reset bit (PLUG detect).
		 * Switch PHY opmode to normal operation (PLUG detect).
		 */
		tmp = readl(&plug->status);
		if (tmp & PLUG_STATUS_ATTACHED)
			tmp &= ~(PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
		else {
			/* USB cable detached
			 * Reset the PHY and switch the mode.
			 */
			tmp |= (PLUG_STATUS_PHY_RESET | PLUG_STATUS_PHY_MODE);
		}

		writel(tmp, &plug->status);

		tmp = readl(&ep0regs->control);
		tmp |= ENDP_CNTL_FLUSH;
		writel(tmp, &ep0regs->control);

	}
	/* Device Enumeration completed */
	if (dev_int & DEV_INT_ENUM) {
		/* Clear interrupt */
		writel(DEV_INT_ENUM, &glob->dev_int);

		DW_UDC_DBG(DBG_INTS, "USB Enumeration\n");

		status = readl(&glob->dev_status);
		/* Check Device speed */
		if ((status & DEV_STAT_ENUM) == DEV_STAT_ENUM_HS)
			udev->gadget.speed = USB_SPEED_HIGH;
		else
			udev->gadget.speed = USB_SPEED_FULL;

		udc_config_ep(udev, &udev->ep[0]);

		/* Need to program UDC20_Regs specific for the controller */
		udc_prog_udc20_regs(udev, &udev->ep[0]);

		udev->ep0state = EP0_CTRL_IDLE;
		DW_UDC_DBG(DBG_EP0STATE, "ep0state = %d\n", udev->ep0state);

		/* program DMA buffer and interrupt registers */
		udc_config_dma(&udev->ep[0]);

		/* enable RX_DMA */
		tmp = readl(&glob->dev_control);
		tmp |= DEV_CNTL_RXDMAEN;
		writel(tmp, &glob->dev_control);

	}

	if (dev_int & DEV_INT_INACTIVE) {
		/* The USB will be in SUSPEND in 3 ms */
		/* Clear interrupt */
		writel(DEV_INT_INACTIVE, &glob->dev_int);
		if (udev->gadget.speed != USB_SPEED_UNKNOWN && udev->driver
				&& udev->driver->suspend)
			udev->driver->suspend(&udev->gadget);

		/* also enable SOF interrupt to detect RESUME */
		tmp = readl(&glob->dev_int_mask);
		tmp &= ~DEV_INT_SOF;
		writel(tmp, &glob->dev_int_mask);
	}

	if (dev_int & DEV_INT_SETCFG) {
		struct usb_ctrlrequest u_ctrl_req;

		writel(DEV_INT_SETCFG, &glob->dev_int);
		DW_UDC_DBG(DBG_INTS, "Set Configuration\n");

		/*
		 * Since the UDC_AHB controller filters SET_CONFIGURATION,
		 * we need to send a fake one to the gadget driver above.
		 * We also make sure the ep0 state is EP0_CTRL_IDLE, so that
		 * the STATUS_IN transaction coming from above is discarded.
		 */

		u_ctrl_req.bRequestType =
			USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
		u_ctrl_req.bRequest = USB_REQ_SET_CONFIGURATION;
		u_ctrl_req.wValue = readl(&glob->dev_status) & DEV_STAT_CFG;
		u_ctrl_req.wIndex = 0;
		u_ctrl_req.wLength = 0;

		DW_UDC_DBG(DBG_REQUESTS, "SETUP %02x.%02x v%04x i%04x l%04x\n",
				u_ctrl_req.bRequestType, u_ctrl_req.bRequest,
				u_ctrl_req.wValue, u_ctrl_req.wIndex,
				u_ctrl_req.wLength);

		udc_handle_internal_cmds(udev, u_ctrl_req);

		/*
		 * start the timer which keeps watch on out endpoints
		 * The pupose of this timer is to see if dma list is
		 * partially filled on an endpoint for a long time. If
		 * so it completes the request with whatever data
		 * (partial) is there in the list
		 */

		mod_timer(&udev->out_ep_timer, jiffies + HZ);
	}

	if (dev_int & DEV_INT_SETINTF) {
		struct usb_ctrlrequest u_ctrl_req;

		writel(DEV_INT_SETINTF, &glob->dev_int);
		DW_UDC_DBG(DBG_INTS, "Set Interface\n");

		/*
		 * Similar to SET CONFIG Request we need to prepare a fake
		 * request
		 */
		u_ctrl_req.bRequestType =
			USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;
		u_ctrl_req.bRequest = USB_REQ_SET_INTERFACE;
		u_ctrl_req.wValue =
			(readl(&glob->dev_status) & DEV_STAT_ALT) >> 8;
		u_ctrl_req.wIndex =
			(readl(&glob->dev_status) & DEV_STAT_INTF) >> 4;;

		u_ctrl_req.wLength = 0;

		DW_UDC_DBG(DBG_REQUESTS, "SETUP %02x.%02x v%04x i%04x l%04x\n",
				u_ctrl_req.bRequestType, u_ctrl_req.bRequest,
				u_ctrl_req.wValue, u_ctrl_req.wIndex,
				u_ctrl_req.wLength);

		udc_handle_internal_cmds(udev, u_ctrl_req);
	}

	if (dev_int & DEV_INT_SUSPUSB)
		writel(DEV_INT_SUSPUSB, &glob->dev_int);

	if (dev_int & DEV_INT_SOF) {
		/*
		 * There is no direct interrupt for RESUME
		 * So SOF is enabled only after suspend
		 * On receiving SOF interrupt RESUME is assumed
		 */
		writel(DEV_INT_SOF, &glob->dev_int);
		/* Now disable SOF interrupt */
		tmp = readl(&glob->dev_int_mask);
		tmp |= DEV_INT_SOF;
		writel(tmp, &glob->dev_int_mask);

		if (udev->gadget.speed != USB_SPEED_UNKNOWN && udev->driver
				&& udev->driver->resume)
			udev->driver->resume(&udev->gadget);
	}
}

static irqreturn_t dw_udc_irq(int irq, void *_dev)
{
	struct dw_udc_dev *udev = (struct dw_udc_dev *)_dev;
	u32 dev_int, endp_int, plug_int;

	dev_int = readl(&udev->glob_base->dev_int);
	endp_int = readl(&udev->glob_base->endp_int);
	plug_int = readl(&udev->plug_base->pending);

	DW_UDC_DBG(DBG_INTS, "devint=%08x, endpint=%08x\n plugint=%08x",
			dev_int, endp_int, plug_int);
	if (plug_int || dev_int || endp_int) {
		/* Plug detect interrupt (high priority) */
		if (plug_int)
			udc_handle_plug_irq(udev);

		if (dev_int) {
			show_global(udev);
			udc_handle_device_irq(udev);
		}

		/* Endpoint interrupts */
		if (endp_int) {
			/* handle ep0 interrupts first */
			if ((endp_int & ENDP0_OUT_INT) ||
					(endp_int & ENDP0_IN_INT))
				udc_handle_ep0_irq(udev);
			else
				udc_handle_ep_irq(udev);
		}
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static struct usb_ep_ops dw_udc_ep_ops = {
	.enable = dw_ep_enable,
	.disable = dw_ep_disable,
	.alloc_request = dw_ep_alloc_request,
	.free_request = dw_ep_free_request,
	.queue = dw_ep_queue,
	.dequeue = dw_ep_dequeue,
	.set_halt = dw_ep_set_halt,
	.fifo_status = dw_ep_fifo_status,
	.fifo_flush = dw_ep_fifo_flush,
};

static int dw_udc_get_frame(struct usb_gadget *_gadget)
{
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	unsigned long flags;
	unsigned int tmp;

	udev = container_of(_gadget, struct dw_udc_dev, gadget);

	glob = udev->glob_base;
	spin_lock_irqsave(&udev->lock, flags);
	tmp = readl(&(glob->dev_status));
	spin_unlock_irqrestore(&udev->lock, flags);

	return (tmp >> 18) & 0x3fff;
}

static int dw_udc_wakeup(struct usb_gadget *_gadget)
{
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	unsigned long flags;
	unsigned int tmp;

	udev = container_of(_gadget, struct dw_udc_dev, gadget);
	glob = udev->glob_base;

	spin_lock_irqsave(&udev->lock, flags);

	tmp = readl(&glob->dev_control);
	writel(tmp | DEV_CNTL_RESUME, &glob->dev_control);
	mdelay(1);
	writel(tmp, &glob->dev_control);

	spin_unlock_irqrestore(&udev->lock, flags);

	return 0;
}

static int dw_udc_set_selfpowered(struct usb_gadget *gadget, int selfpowered)
{
	struct dw_udc_dev *udev;
	struct dw_udc_glob_regs *glob;
	unsigned long flags;
	unsigned int tmp;

	udev = container_of(gadget, struct dw_udc_dev, gadget);
	glob = udev->glob_base;

	spin_lock_irqsave(&udev->lock, flags);

	tmp = readl(&glob->dev_conf);
	if (selfpowered)
		tmp |= DEV_CONF_SELFPOW;
	else
		tmp &= ~DEV_CONF_SELFPOW;

	writel(tmp, &glob->dev_conf);

	spin_unlock_irqrestore(&udev->lock, flags);

	return 0;
}

static int
dw_udc_ioctl(struct usb_gadget *gadget, unsigned code, unsigned long param)
{
	struct usb_descriptor_header **function;
	struct dw_udc_dev *udev;

	if (param == 0)
		return -EINVAL;

	udev = container_of(gadget, struct dw_udc_dev, gadget);
	function = (struct usb_descriptor_header **)param;

	/* Only 1 ioctl as of now */
	switch (code) {
	case UDC_SET_CONFIG:
		udc_set_config(udev, function);
		break;
	default:
		dev_err(udev->dev, "Wrong code in ioctl call\n");
		return -EINVAL;
	}

	return 0;
}

static void dw_udc_release(struct device *dev)
{
	pr_debug("%s %s\n", __func__, dev_name(dev));
}

static struct dw_udc_dev the_controller = {
	.gadget = {
		.ep_list = LIST_HEAD_INIT(the_controller.gadget.ep_list),
		.max_speed = USB_SPEED_HIGH,
		.name = driver_name,
		.dev = {
			.init_name = "gadget",
			.release = dw_udc_release,
		},
	},
};

static int dw_udc_pullup(struct usb_gadget *_gadget, int is_on)
{
	struct dw_udc_dev *udev = &the_controller;
	unsigned long		flags;

	if (!_gadget)
		return -ENODEV;
	if (!udev)
		return -ENODEV;

	spin_lock_irqsave(&udev->lock, flags);

	if (is_on)
		udc_connect(udev);
	else
		udc_disconnect(udev);

	spin_unlock_irqrestore(&udev->lock, flags);

	return 0;
}

/*
 * when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests. then usb traffic follows until a
 * disconnect is reported. then a host may connect again, or
 * the driver might get unbound.
 */
int dw_udc_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct dw_udc_dev *udev = &the_controller;
	int retval;
	unsigned long flags;

	/* Paranoid */
	if (!driver || driver->max_speed < USB_SPEED_FULL || !bind ||
			!driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!udev || !udev->dev)
		return -ENODEV;
	if (udev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	udev->driver = driver;
	udev->gadget.dev.driver = &driver->driver;

	retval = device_add(&udev->gadget.dev);
	if (retval) {
		dev_err(udev->dev, "Could not add gadget: %d\n", retval);
		return retval;
	}

	retval = bind(&udev->gadget);
	if (retval) {
		dev_err(udev->dev, "bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		device_del(&udev->gadget.dev);

		udev->driver = 0;
		udev->gadget.dev.driver = 0;
		return retval;
	}

	/*
	 * following timer called when ep out contains multiple of maxpacket
	 * data pending in ep out dma chain, this timer closes the current dma
	 * descriptor passing on to application the stale data
	 */
	init_timer(&udev->out_ep_timer);
	udev->out_ep_timer.function = &udc_ep_timeout;
	udev->out_ep_timer.expires = jiffies + HZ;
	udev->out_ep_timer.data = (unsigned long)udev;

	DW_UDC_DBG(DBG_ADDRESS, "udev = %p\n", udev);

	spin_lock_irqsave(&udev->lock, flags);
	udc_enable(udev);
	spin_unlock_irqrestore(&udev->lock, flags);

	dev_info(udev->dev, "reg gadget driver '%s'\n", driver->driver.name);

	return 0;
}

int dw_udc_stop(struct usb_gadget_driver *driver)
{
	struct dw_udc_dev *udev = &the_controller;

	if (!udev)
		return -ENODEV;
	if (!driver || driver != udev->driver)
		return -EINVAL;

	stop_activity(udev, driver);
	udc_disconnect(udev);

	driver->unbind(&udev->gadget);
	udev->driver = 0;

	device_del(&udev->gadget.dev);

	return 0;
}
static const struct usb_gadget_ops dw_udc_ops = {
	.get_frame = dw_udc_get_frame,
	.wakeup = dw_udc_wakeup,
	.set_selfpowered = dw_udc_set_selfpowered,
	.ioctl = dw_udc_ioctl,
	.pullup = dw_udc_pullup,
	.start = dw_udc_start,
	.stop = dw_udc_stop,
};

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
static const char proc_node_name[] = "driver/synudc";

static int
dw_udc_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	struct dw_udc_dev *udev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
			"%s\nGadget driver: %s\n\n", driver_name,
			udev->driver ? udev->driver->driver.name : "(none)");
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static const char proc_node_regs_name[] = "driver/dw_udc_regs";

static int
dw_udc_proc_regs_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	struct dw_udc_dev *udev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;
	u32 *p;
	u8 *startp;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	startp = (udev->csr_base + UDC20_REG_OFF);
	p = (u32 *) startp;
	/* UDC regs show */
	while ((u8 *) p < startp + 0x40 && size > 0) {
		t = scnprintf(next, size, "%p: %08x %08x %08x %08x\n",
				p, p[0], p[1], p[2], p[3]);
		p += 4;
		size -= t;
		next += t;
	}

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static const char proc_node_endp_name[] = "driver/dw_udc_endp";

static int
dw_udc_proc_endp_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	struct dw_udc_dev *udev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int i, t;
	struct dw_udc_glob_regs *glob = udev->glob_base;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	t = scnprintf(next, size, "Config: %08x Control: %08x Status: %08x\n",
			readl(&glob->dev_conf),
			readl(&glob->dev_control), readl(&glob->dev_status));

	size -= t;
	next += t;

	t = scnprintf(next, size, "Dev.int = %08x Endp.int. = %08x\n",
			readl(&(udev->glob_base->dev_int)),
			readl(&(udev->glob_base->endp_int)));
	size -= t;
	next += t;

	for (i = 0; i < 6; i++) {

		struct dw_udc_ep *ep = &udev->ep[i];
		struct dw_udc_epin_regs *in_regs;
		struct dw_udc_epout_regs *out_regs;

		in_regs = ep->in_regs;
		out_regs = ep->out_regs;

		t = scnprintf(next, size, "Endp: %s\n\tIN REGS\n", ep->ep.name);
		size -= t;
		next += t;
		t = scnprintf(next, size,
				"\tControl: %08x Status: %08x Bufsize: %08x\n",
				readl(&in_regs->control),
				readl(&in_regs->status),
				readl(&in_regs->bufsize));
		size -= t;
		next += t;
		t = scnprintf(next, size, "\tMax packet: %08x desc_ptr: %08x\n",
				readl(&in_regs->max_pack_size),
				readl(&in_regs->desc_ptr));
		size -= t;
		next += t;
		t = scnprintf(next, size, "\tOUT REGS\n");
		size -= t;
		next += t;
		t = scnprintf(next, size,
				"\tControl:%08x Status:%08x Frame num:%08x\n",
				readl(&out_regs->control),
				readl(&out_regs->status),
				readl(&out_regs->frame_num));
		size -= t;
		next += t;
		t = scnprintf(next, size,
				"\tBufsize:%08x setup_ptr:%08x desc_ptr:%08x\n",
				readl(&out_regs->bufsize),
				readl(&out_regs->setup_ptr),
				readl(&out_regs->desc_ptr));
		size -= t;
		next += t;
	}

	local_irq_restore(flags);
	*eof = 1;

	return count - size;
}

static const char proc_node_out_chain_name[] = "driver/dw_udc_out_chain";

static int
dw_udc_proc_out_chain(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	struct dw_udc_bulkd *tmp_desc;
	char *buf = page;
	struct dw_udc_dev *udev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;
	struct dw_udc_ep *ep = &udev->ep[2];

	if (off != 0)
		return 0;

	local_irq_save(flags);

	t = scnprintf(next, size, "Out Chain for ep %s:\n", ep->ep.name);

	size -= t;
	next += t;

	tmp_desc = ep->desc_out_ptr;
	while (tmp_desc != 0x0) {

		t = scnprintf(next, size, "%08x %08x %08x\n",
				le32_to_cpu(tmp_desc->status),
				le32_to_cpu(tmp_desc->bufp),
				le32_to_cpu(tmp_desc->nextd));

		size -= t;
		next += t;

		if (le32_to_cpu(tmp_desc->status) & DMAUSB_LASTDESCR)
			break;
		tmp_desc = list_entry(tmp_desc->desc_list.next,
				struct dw_udc_bulkd, desc_list);
	}

	local_irq_restore(flags);
	*eof = 1;

	return count - size;
}

static const char proc_node_in_chain_name[] = "driver/dw_udc_in_chain";

static int
dw_udc_proc_in_chain(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	struct dw_udc_bulkd *tmp_desc;
	char *buf = page;
	struct dw_udc_dev *udev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;
	struct dw_udc_ep *ep = &udev->ep[1];

	if (off != 0)
		return 0;

	local_irq_save(flags);

	t = scnprintf(next, size, "IN Chain for ep %s:\n", ep->ep.name);

	size -= t;
	next += t;

	tmp_desc = ep->curr_chain_in;
	while (tmp_desc != 0x0) {

		t = scnprintf(next, size, "%08x %08x %08x %08x\n",
				tmp_desc->dma_addr,
				le32_to_cpu(tmp_desc->status),
				le32_to_cpu(tmp_desc->bufp),
				le32_to_cpu(tmp_desc->nextd));

		size -= t;
		next += t;

		if (le32_to_cpu(tmp_desc->status) & DMAUSB_LASTDESCR)
			break;
		tmp_desc = list_entry(tmp_desc->desc_list.next,
				struct dw_udc_bulkd, desc_list);
	}

	local_irq_restore(flags);
	*eof = 1;

	return count - size;
}

void dw_udc_create_proc_files(void)
{
	struct dw_udc_dev *udev = &the_controller;
	create_proc_read_entry(proc_node_name, 0, NULL, dw_udc_proc_read, udev);
	create_proc_read_entry(proc_node_regs_name, 0, NULL,
			dw_udc_proc_regs_read, udev);
	create_proc_read_entry(proc_node_endp_name, 0, NULL,
			dw_udc_proc_endp_read, udev);
	create_proc_read_entry(proc_node_out_chain_name, 0, NULL,
			dw_udc_proc_out_chain, udev);
	create_proc_read_entry(proc_node_in_chain_name, 0, NULL,
			dw_udc_proc_in_chain, udev);
}

void dw_udc_remove_proc_files(void)
{
	remove_proc_entry(proc_node_name, NULL);
	remove_proc_entry(proc_node_regs_name, NULL);
	remove_proc_entry(proc_node_endp_name, NULL);
	remove_proc_entry(proc_node_out_chain_name, NULL);
	remove_proc_entry(proc_node_in_chain_name, NULL);

}
#endif /* CONFIG_USB_GADGET_DEBUG_FILES */

static int __devinit dw_udc_probe(struct platform_device *pdev)
{
	struct udc_platform_data *pdata;
	struct dw_udc_dev *udev = &the_controller;
	struct resource *csr, *plug;
	struct dw_udc_ep *ep;
	int i, retval = 0;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata)
		return -ENXIO;

	csr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!csr) {
		dev_err(&pdev->dev, "no csr resource defined\n");
		return -EBUSY;
	}

	plug = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!plug) {
		dev_err(&pdev->dev, "no plug resource defined\n");
		return -EBUSY;
	}

	if (!request_mem_region(csr->start, resource_size(csr), pdev->name)) {
		dev_err(&pdev->dev, "udc csr region already claimed\n");
		return -EBUSY;
	}

	if (!request_mem_region(plug->start, resource_size(plug), pdev->name)) {
		dev_err(&pdev->dev, "udc plug region already claimed\n");
		retval = -EBUSY;
		goto err_release_csr_region;
	}

	udev->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(udev->clk)) {
		retval = PTR_ERR(udev->clk);
		goto err_release_plug_region;
	}

	retval = clk_prepare_enable(udev->clk);
	if (retval < 0)
		goto err_clk_put;

	udev->csr_base = ioremap(csr->start, resource_size(csr));
	if (!udev->csr_base) {
		dev_err(&pdev->dev, "Unable to map I/O memory, aborting\n");
		retval = -ENOMEM;
		goto err_disable_clock;
	}

	udev->plug_base = ioremap(plug->start, resource_size(plug));
	if (!udev->plug_base) {
		dev_err(&pdev->dev, "Unable to map I/O memory, aborting.\n");
		retval = -ENOMEM;
		goto err_iounmap_csr;
	}

	spin_lock_init(&udev->lock);

	udev->epin_base = (struct dw_udc_epin_regs __iomem *)(udev->csr_base);
	udev->epout_base = (struct dw_udc_epout_regs __iomem *)(udev->csr_base +
			UDC_EP_OUT_REG_OFF);
	udev->glob_base = (struct dw_udc_glob_regs __iomem *)(udev->csr_base +
			UDC_GLOB_REG_OFF);

	udev->num_ep = pdata->num_ep;

	udev->ep = kzalloc(pdata->num_ep * sizeof(struct dw_udc_ep),
			GFP_ATOMIC);
	if (!udev->ep) {
		dev_err(&pdev->dev, "couldn't allocate memory for endpoints\n");
		goto err_iounmap_plug;
	}

	udev->gadget.ops = &dw_udc_ops,
	udev->gadget.ep0 = &udev->ep[0].ep;
	for (i = 0; i < pdata->num_ep; i++) {
		ep = &udev->ep[i];
		ep->udev = udev;
		ep->ep.ops = &dw_udc_ep_ops;
		ep->ep.name = pdata->ep[i].name;
		ep->addr = pdata->ep[i].addr;
		ep->attrib = pdata->ep[i].attrib;
		ep->fifo_size = pdata->ep[i].fifo_size;
		ep->ep.maxpacket = pdata->ep[i].maxpacket;

		INIT_LIST_HEAD(&ep->queue);
		ep->in_regs = &udev->epin_base[i];
		ep->out_regs = &udev->epout_base[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udev->gadget.ep_list);
	}

	udev->dev = &pdev->dev;
	device_initialize(&udev->gadget.dev);
	dev_set_name(&udev->gadget.dev, "gadget");
	udev->gadget.dev.parent = &pdev->dev;
	udev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	udev->gadget.max_speed = USB_SPEED_HIGH;
	udev->int_cmd = 0;

	platform_set_drvdata(pdev, udev);

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	dw_udc_create_proc_files();
#endif

	/* Restore a known hardware state */

	udc_disconnect(udev);
	udc_reinit(udev);

	retval = desc_pool_init(udev);
	if (retval != 0) {
		retval = -ENOMEM;
		goto err_mem_ep;
	}

	/* other non-static parts of init */
	udev->irq = platform_get_irq(pdev, 0);
	if (udev->irq < 0) {
		dev_err(&pdev->dev, "No irq, aborting.\n");
		goto err_mem_pool_remove;
	}

	retval = request_irq(udev->irq, dw_udc_irq, 0, driver_name, udev);
	if (retval != 0) {
		dev_err(&pdev->dev, "%s: can't get irq %d, err %d\n",
				driver_name, udev->irq, retval);
		retval = -EBUSY;
		goto err_mem_pool_remove;
	}

	retval = usb_add_gadget_udc(&pdev->dev, &udev->gadget);
	if (retval)
		goto err_mem_pool_remove;

	dev_info(&pdev->dev, "Device Synopsys UDC probed csr %p: plug %p\n",
			udev->csr_base, udev->plug_base);
	return 0;

err_mem_pool_remove:
	desc_pool_remove(udev);
err_mem_ep:
	kfree(udev->ep);
	platform_set_drvdata(pdev, NULL);
err_iounmap_plug:
	iounmap(udev->plug_base);
err_iounmap_csr:
	iounmap(udev->csr_base);
err_disable_clock:
	clk_disable_unprepare(udev->clk);
err_clk_put:
	clk_put(udev->clk);
err_release_plug_region:
	release_mem_region(plug->start, resource_size(plug));
err_release_csr_region:
	release_mem_region(csr->start, resource_size(csr));

	return retval;
}

static int __devexit dw_udc_remove(struct platform_device *pdev)
{
	struct udc_platform_data *pdata;
	struct dw_udc_dev *udev = platform_get_drvdata(pdev);
	int irq;

	pdata = dev_get_platdata(&pdev->dev);
	irq = platform_get_irq(pdev, 0);

	/* Donot forget to disable endpoint 0 */
	udc_disconnect(udev);
	desc_pool_remove(udev);

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	dw_udc_remove_proc_files();
#endif

	usb_gadget_unregister_driver(udev->driver);

	free_irq(irq, udev);
	clk_disable_unprepare(udev->clk);
	platform_set_drvdata(pdev, NULL);

	/* release memory */
	iounmap(udev->csr_base);
	iounmap(udev->plug_base);

	kfree(udev->ep);

	return 0;
}

#ifdef CONFIG_PM
static int dw_udc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_udc_dev *udev = platform_get_drvdata(pdev);
	struct dw_udc_glob_regs *glob = udev->glob_base;

	/* check if gadget is registered and bus is not suspended */
	if (!(readl(&glob->dev_status) & DEV_STAT_SUSP) && udev->driver) {
		/* Device is connected, disconnect before going to suspend */
		stop_activity(udev, udev->driver);
		udc_disconnect(udev);
	} else {
		if (!enable_irq_wake(udev->irq))
			udev->irq_wake = 1;
		udev->active_suspend = 1;
	}

	return 0;
}

static int dw_udc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_udc_dev *udev = platform_get_drvdata(pdev);

	if (udev->active_suspend) {
		/* bus resumed */
		if (udev->irq_wake) {
			disable_irq_wake(udev->irq);
			udev->irq_wake = 0;
		}
		udev->active_suspend = 0;
	} else
		udc_connect(udev);

	/* wakeup host if required */
	dw_udc_wakeup(&udev->gadget);
	/* Connect the USB */
	/* maybe the host would enumerate us if we nudged it */
	msleep(100);

	return 0;
}

static const struct dev_pm_ops dw_udc_pm_ops = {
	.suspend = dw_udc_suspend,
	.resume = dw_udc_resume,
	.freeze = dw_udc_suspend,
	.restore = dw_udc_resume,
};

#endif

#if defined(CONFIG_OF)
static const struct of_device_id dwc_usbd_match[] = {
	{.compatible = "snps,designware-udc",},
	{}
};
MODULE_DEVICE_TABLE(of, dwc_usbd_match);
#endif

static struct platform_driver dw_udc_driver = {
	.probe = dw_udc_probe,
	.remove = __devexit_p(dw_udc_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = (char *)driver_name,
#if defined(CONFIG_OF)
		   .of_match_table = dwc_usbd_match,
#endif

#ifdef CONFIG_PM
	.pm = &dw_udc_pm_ops,
#endif

	},
};

static int __init dw_udc_init(void)
{
	return platform_driver_register(&dw_udc_driver);
}
module_init(dw_udc_init);

static void __exit dw_udc_exit(void)
{
	platform_driver_unregister(&dw_udc_driver);
}
module_exit(dw_udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Rajeev Kumar<rajeev-dlh.kumar@st.com>, \
		Shiraz Hashim<shiraz.hashim@st.com>");
MODULE_LICENSE("GPL");
