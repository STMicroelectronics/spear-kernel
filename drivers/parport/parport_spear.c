/*
 * linux/drivers/parport/parport_spear.c
 * Parallel Port device driver
 *
 * Copyright (C) 2009 STmicroelectronics. (Ashish Priyadarshi)
 * Ashish Priyadarshi
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/parport.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>

/* Status Register */
#define PARPORT_STAT_DATA_AVAIL		(1 << 7)
#define PARPORT_STAT_SELIN		(1 << 6)
#define PARPORT_STAT_INIT		(1 << 5)
#define PARPORT_STAT_AUTO_FEED		(1 << 4)
#define PARPORT_STAT_ERROR		(1 << 3)
#define PARPORT_STAT_IDLE		(1 << 2)
#define PARPORT_STAT_FORCE_BUSY		(1 << 1)
#define PARPORT_STAT_INTERFACE		(1 << 0)

/* Control Register */
#define PARPORT_CTRL_DATA_RCVD		(1 << 7)
#define PARPORT_CTRL_FAULT		(1 << 6)
#define PARPORT_CTRL_PERROR		(1 << 5)
#define PARPORT_CTRL_SW_ERR_CLR		(1 << 4)
#define PARPORT_CTRL_FORCE_BUSY		(1 << 1)
#define PARPORT_CTRL_SELECT		(1 << 0)

/* Interrupt Status Register */
#define PARPORT_RAW_INTR_SELIN		(1 << 7)
#define PARPORT_RAW_INTR_INIT		(1 << 6)
#define PARPORT_RAW_INTR_AUTOFD		(1 << 5)
#define PARPORT_RAW_INTR_DATA_AVAIL	(1 << 4)
#define PARPORT_MASKED_INTR_SELIN	(1 << 3)
#define PARPORT_MASKED_INTR_INIT	(1 << 2)
#define PARPORT_MASKED_INTR_AUTOFD	(1 << 1)
#define PARPORT_MASKED_INTR_DATA_AVAIL	(1 << 0)

/* Interrupt Enable/Clear Register */
#define PARPORT_INTR_SELIN		(1 << 3)
#define PARPORT_INTR_INIT		(1 << 2)
#define PARPORT_INTR_AUTOFD		(1 << 1)
#define PARPORT_INTR_DATA_AVAIL		(1 << 0)

/**
 * struct spear_parport_dev - structure to define parallel port device
 *
 * @parport: Parallel-port device structure
 * @suspend: Parallel-port state
 * @dev: Platform device structure
 * @irq_enabled: Flag to show whether Intr is enabled
 * @base: Parallel-port Base offset of registers
 * @spp_data: Parallel-port Data register
 * @spp_stat: Parallel-port Status register
 * @spp_ctrl: Parallel-port Control register
 * @spp_intr_stat: Parallel-port Intr status register
 * @spp_intr_enbl: Parallel-port Intr enable register
 * @spp_intr_clr: Parallel-port Intr clear register
 */
struct spear_parport_dev {
	struct parport		*parport;
	struct parport_state	 suspend;
	struct device		*dev;
	spinlock_t		lock;
	u8		 irq_enabled;
	void __iomem		*base;
	void __iomem		*spp_data;
	void __iomem		*spp_stat;
	void __iomem		*spp_ctrl;
	void __iomem		*spp_intr_stat;
	void __iomem		*spp_intr_enbl;
	void __iomem		*spp_intr_clr;
};

static inline struct spear_parport_dev *pp_to_drv(struct parport *dev)
{
	return dev->private_data;
}

/**
 * spear_parport_read_data - read 8-bit data from parallel port
 * @dev: parallel-port structure
 */
static u8 spear_parport_read_data(struct parport *dev)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	u8 ctrl = readb(priv->spp_ctrl);
	u8 enbl = readb(priv->spp_intr_enbl);
	char data;

	data = readb(priv->spp_data);

	/* clear data-received interrupt */
	ctrl |= PARPORT_CTRL_DATA_RCVD;
	writeb(ctrl, priv->spp_ctrl);

	/* enable data-received interrupt */
	enbl |= PARPORT_INTR_DATA_AVAIL;
	writeb(enbl, priv->spp_intr_enbl);

	return data;
}

/**
 * spear_parport_write_data - write 8-bit data to parallel port
 * @dev: parallel-port structure
 * @data: data to be written
 *
 * Empty function, as write is not supported by Hw
 */
static void spear_parport_write_data(struct parport *dev, unsigned char data)
{
	return;
}

/**
 * spear_parport_read_control - read control register of parallel port
 * @dev: parallel-port structure
 */
static unsigned char spear_parport_read_control(struct parport *dev)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	unsigned char ret = 0;
	u8 ctrl;

	/*
	 * No need to handle PARPORT_CONTROL_STROBE and PARPORT_CONTROL_AUTOFD
	 * as it will be taken care by Hw
	 */

	ctrl = readb(priv->spp_ctrl);

	if (!(ctrl & PARPORT_CTRL_FORCE_BUSY))
		ret |= PARPORT_CONTROL_INIT;

	if (ctrl & PARPORT_CTRL_SELECT)
		ret |= PARPORT_CONTROL_SELECT;

	return ret;
}

/**
 * spear_parport_write_control - write control register of parallel port
 * @dev: parallel-port structure
 * @control: Value to write into the Control register
 */
static void spear_parport_write_control(struct parport *dev,
			unsigned char control)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	unsigned int ctrl = readb(priv->spp_ctrl);
	/*
	 * No need to handle PARPORT_CONTROL_STROBE and PARPORT_CONTROL_AUTOFD
	 * as it will be taken care by Hw
	 */

	if (control & PARPORT_CONTROL_INIT)
		ctrl &= ~(PARPORT_CTRL_FORCE_BUSY);

	if (control & PARPORT_CONTROL_SELECT)
		ctrl |= PARPORT_CTRL_SELECT;

	writeb(ctrl, priv->spp_ctrl);
}

/**
 * spear_parport_read_status - read status register of parallel port
 * @dev: parallel-port structure
 */
static unsigned char spear_parport_read_status(struct parport *dev)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	unsigned char ret = 0;
	u8 status;

	status = readb(priv->spp_stat);

	if (status & PARPORT_STAT_FORCE_BUSY)
		ret |= PARPORT_STATUS_BUSY;

	/* data-available means nACK will be active low otherwise high */
	if (status & PARPORT_STAT_DATA_AVAIL)
		ret &= ~PARPORT_STATUS_ACK;
	else
		ret |= PARPORT_STATUS_ACK;

	if (status & PARPORT_STAT_ERROR)
		ret |= PARPORT_STATUS_ERROR;

	if (status & PARPORT_STAT_INTERFACE)
		ret |= PARPORT_STATUS_SELECT;

	if (status & PARPORT_STAT_ERROR)
		ret |= PARPORT_STATUS_PAPEROUT;

	return ret;
}

/**
 * spear_parport_frob_control - write some selected bits of
 * Control-register only
 * @dev: parallel-port structure
 * @mask: bits to be masked
 * @val: value to be set in masked bits
 */
static unsigned char spear_parport_frob_control(struct parport *dev,
		unsigned char mask, unsigned char val)
{
	u8 old;

	old = spear_parport_read_control(dev);
	spear_parport_write_control(dev, (old & ~mask) | val);

	return old;
}

/**
 * spear_parport_enable_irq - enable intr
 * @dev: parallel-port structure
 */
static void spear_parport_enable_irq(struct parport *dev)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->irq_enabled) {
		enable_irq(dev->irq);
		priv->irq_enabled = 1;
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

/**
 * spear_parport_disable_irq - disable intr
 * @dev: parallel-port structure
 */
static void spear_parport_disable_irq(struct parport *dev)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->irq_enabled) {
		disable_irq(dev->irq);
		priv->irq_enabled = 0;
	}

	spin_unlock_irqrestore(&priv->lock, flags);
}

/**
 * spear_parport_data_forward - Change direction of data flow
 * @dev: parallel-port structure
 *
 * Not implemented as not supported by Hw
 */
static void spear_parport_data_forward(struct parport *dev)
{
	return;
}

/**
 * spear_parport_data_reverse - Change direction of data flow
 * @dev: parallel-port structure
 *
 * Not implemented as not supported by Hw
 */
static void spear_parport_data_reverse(struct parport *dev)
{
	return;
}

/**
 * spear_parport_init_state - Clear parallel-port data structure
 * @dev: parallel-port structure
 * @s: parallel-port state
 */
static void spear_parport_init_state(struct pardevice *d,
		struct parport_state *s)
{
	struct spear_parport_dev *priv = pp_to_drv(d->port);

	memset(s, 0, sizeof(*s));

	s->u.pc.ctr = readb(priv->spp_ctrl);
}

/**
 * spear_parport_save_state - Save Control register during suspend
 * @dev: parallel-port structure
 * @s: parallel-port state
 */
static void spear_parport_save_state(struct parport *dev,
		struct parport_state *s)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);

	s->u.pc.ctr = readb(priv->spp_ctrl);
}

/**
 * spear_parport_restore_state - Restore Control register during resume
 * @dev: parallel-port structure
 * @s: parallel-port state
 */
static void spear_parport_restore_state(struct parport *dev,
					struct parport_state *s)
{
	struct spear_parport_dev *priv = pp_to_drv(dev);

	writeb(s->u.pc.ctr, priv->spp_ctrl);
}

static struct parport_operations parport_spear_ops = {
	.write_data = spear_parport_write_data,
	.read_data = spear_parport_read_data,

	.write_control = spear_parport_write_control,
	.read_control = spear_parport_read_control,
	.frob_control = spear_parport_frob_control,

	.read_status = spear_parport_read_status,

	.enable_irq = spear_parport_enable_irq,
	.disable_irq = spear_parport_disable_irq,

	.data_forward = spear_parport_data_forward,
	.data_reverse = spear_parport_data_reverse,

	.init_state = spear_parport_init_state,
	.save_state = spear_parport_save_state,
	.restore_state = spear_parport_restore_state,

	.epp_write_data	 = parport_ieee1284_epp_write_data,
	.epp_read_data	 = parport_ieee1284_epp_read_data,
	.epp_write_addr	 = parport_ieee1284_epp_write_addr,
	.epp_read_addr	 = parport_ieee1284_epp_read_addr,

	.ecp_write_data	 = parport_ieee1284_ecp_write_data,
	.ecp_read_data	 = parport_ieee1284_ecp_read_data,
	.ecp_write_addr	 = parport_ieee1284_ecp_write_addr,

	.compat_write_data = parport_ieee1284_write_compat,
	.nibble_read_data = parport_ieee1284_read_nibble,
	.byte_read_data	= parport_ieee1284_read_byte,

	.owner = THIS_MODULE,
};

/**
 * spear_parport_irq_handler - Interrupt handler for parallel port
 * @irq: interrupt number
 * @dev_id: device info
 */
static irqreturn_t spear_parport_irq_handler(int irq, void *dev_id)
{
	struct spear_parport_dev *priv = dev_id;
	u8 stat;
	u8 enbl;

	stat = readb(priv->spp_intr_stat);
	enbl = readb(priv->spp_intr_enbl);

	if (stat & PARPORT_MASKED_INTR_DATA_AVAIL) {
		/*
		 * disable data received interrupt
		 * read data and clear interrupt will be done in
		 * read_data routine
		 */

		enbl &= ~PARPORT_INTR_DATA_AVAIL;
		writeb(enbl, priv->spp_intr_enbl);

		parport_irq_handler(irq, priv->parport);

	}

	if (stat & PARPORT_MASKED_INTR_SELIN) {
		/*
		 * clear device Select interrupt
		 * and initialize the port controls
		 */
		writeb(PARPORT_INTR_SELIN, priv->spp_intr_clr);
		writeb(PARPORT_CTRL_SELECT, priv->spp_ctrl);
	}

	if (stat & PARPORT_MASKED_INTR_INIT)
		writeb(PARPORT_INTR_INIT, priv->spp_intr_clr);

	if (stat & PARPORT_MASKED_INTR_AUTOFD)
		writeb(PARPORT_INTR_AUTOFD, priv->spp_intr_clr);

	return IRQ_HANDLED;
}

/**
 * spear_parport_probe - Startup routine for parallel port
 * @pdev: platform device structure
 */
static int spear_parport_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct parport *p = NULL;
	struct spear_parport_dev *priv;
	struct resource *res;
	int irq, ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(dev, "no memory for private data\n");
		return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		irq = PARPORT_IRQ_NONE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no MEM specified\n");
		ret = -ENXIO;
		goto exit_mem;
	}

	priv->base = ioremap(res->start, resource_size(res));
	if (priv->base == NULL) {
		dev_err(dev, "cannot ioremap region\n");
		ret = -ENXIO;
		goto exit_mem;
	}

	p = parport_register_port((unsigned long)priv->base, irq,
				   PARPORT_DMA_NONE, &parport_spear_ops);
	if (p == NULL) {
		dev_err(dev, "failed to register parallel port\n");
		ret = -ENOMEM;
		goto exit_unmap;
	}

	spin_lock_init(&priv->lock);
	p->private_data = priv;
	priv->parport = p;
	priv->dev = dev;
	p->dev = dev;
	p->irq = irq;
	p->modes = PARPORT_MODE_COMPAT;

	priv->spp_data = priv->base;
	priv->spp_stat = priv->base + 0x04;
	priv->spp_ctrl = priv->base + 0x08;
	priv->spp_intr_stat = priv->base + 0x0C;
	priv->spp_intr_enbl = priv->base + 0x10;
	priv->spp_intr_clr = priv->base + 0x14;

	writeb(PARPORT_CTRL_SELECT, priv->spp_ctrl);

	if (irq >= 0) {
		ret = request_irq(irq, spear_parport_irq_handler,
				0, pdev->name, priv);
		if (ret < 0)
			goto exit_port;
		priv->irq_enabled = 1;
	}

	platform_set_drvdata(pdev, p);

	parport_announce_port(p);

	/* enable all (not only the Data-available) Interrupt */
	writeb(PARPORT_INTR_DATA_AVAIL | PARPORT_INTR_AUTOFD |
			PARPORT_INTR_SELIN | PARPORT_INTR_INIT,
			priv->spp_intr_enbl);
	dev_info(dev, "attached parallel port driver\n");
	return 0;

exit_port:
	parport_remove_port(p);
exit_unmap:
	iounmap(priv->base);
exit_mem:
	kfree(priv);
	return ret;
}

/**
 * spear_parport_remove - Exit routine of parallel port
 * @pdev: platform device structure
 */
static int spear_parport_remove(struct platform_device *pdev)
{
	struct parport *p = platform_get_drvdata(pdev);
	struct spear_parport_dev *priv = pp_to_drv(p);

	free_irq(p->irq, priv);
	parport_remove_port(p);
	iounmap(priv->base);
	kfree(priv);
	return 0;
}

#ifdef CONFIG_PM
/**
 * spear_parport_suspend - Suspend Parallel port during Power save
 * @pdev: platform device structure
 * @state: power state
 */
static int spear_parport_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct parport *p = platform_get_drvdata(pdev);
	struct spear_parport_dev *priv = pp_to_drv(p);

	spear_parport_save_state(p, &priv->suspend);

	writeb(~(PARPORT_CTRL_SELECT), priv->spp_ctrl);
	return 0;
}

/**
 * spear_parport_resume - Resumes Parallel port from power save mode
 * @pdev: platform device structure
 */
static int spear_parport_resume(struct platform_device *pdev)
{
	struct parport *p = platform_get_drvdata(pdev);
	struct spear_parport_dev *priv = pp_to_drv(p);

	spear_parport_restore_state(p, &priv->suspend);
	return 0;
}
#else
#define spear_parport_suspend	NULL
#define spear_parport_resume	NULL
#endif

static struct platform_driver spear_pp_drv = {
	.driver = {
		.name = "spear-spp",
		.owner = THIS_MODULE,
	},
	.probe = spear_parport_probe,
	.remove = spear_parport_remove,
	.suspend = spear_parport_suspend,
	.resume = spear_parport_resume,
};

static int __init spear_parport_init(void)
{
	return platform_driver_register(&spear_pp_drv);
}
module_init(spear_parport_init)

static void __exit spear_parport_exit(void)
{
	platform_driver_unregister(&spear_pp_drv);
}
module_exit(spear_parport_exit)

MODULE_ALIAS("platform:spear-spp");
MODULE_AUTHOR("Shiraz Hashim <shiraz.hashim@st.com>");
MODULE_DESCRIPTION("SPEAr parallel port driver");
MODULE_LICENSE("GPL");
