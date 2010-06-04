/*
 * drivers/net/can/spear_can.c
 *
 * CAN bus driver for SPEAr SoC that houses two independent
 * Bosch CCAN controllers.
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * Borrowed heavily from the original code written for Hynix7202 by:
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix <s.hauer@pengutronix.de>
 * - Simon Kallweit, intefo AG <simon.kallweit@intefo.ch>
 * which can be viewed here:
 * http://svn.berlios.de/svnroot/repos/socketcan/trunk/kernel/2.6/
 * drivers/net/can/old/ccan/
 *
 * TODO:
 * - Power Management support to be added
 * - Special handling of objects to be added to fix 'remote request' issue
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <mach/hardware.h>

#include "bosch_ccan.h"

#define DRV_NAME	"spear_can"
#define CAN_ENABLE	0x0e

static u16 spear_can_read_reg(const struct bosch_ccan_priv *priv,
				enum ccan_regs reg)
{
	u16 val;

	/* shifting 1 place because 16 bit registers are word aligned */
	val = readw(priv->reg_base + (reg << 1));
	return val;
}

static void spear_can_write_reg(const struct bosch_ccan_priv *priv,
				enum ccan_regs reg, u16 val)
{
	/* shifting 1 place because 16 bit registers are word aligned */
	writew(val, priv->reg_base + (reg << 1));
}

static int spear_can_drv_probe(struct platform_device *pdev)
{
	int ret;
	void __iomem *addr;
	struct net_device *dev;
	struct bosch_ccan_priv *priv;
	struct resource *mem, *irq;
	struct clk *clk;

	/* get the appropriate clk */
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		ret = -ENODEV;
		goto exit;
	}

	/* get the platform data */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!mem || !irq) {
		ret = -ENODEV;
		goto exit_free_clk;
	}

	if (!request_mem_region(mem->start, resource_size(mem), DRV_NAME)) {
		dev_err(&pdev->dev, "resource unavailable\n");
		ret = -ENODEV;
		goto exit_free_clk;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		dev_err(&pdev->dev, "failed to map can port\n");
		ret = -ENOMEM;
		goto exit_release_mem;
	}

	/* allocate the ccan device */
	dev = alloc_bosch_ccandev(0);
	if (!dev) {
		clk_put(clk);
		ret = -ENOMEM;
		goto exit_iounmap;
	}

	priv = netdev_priv(dev);
	dev->irq = irq->start;
	priv->irq_flags = irq->flags;
	priv->reg_base = addr;
	priv->can.clock.freq = clk_get_rate(clk);
	priv->read_reg = spear_can_read_reg;
	priv->write_reg = spear_can_write_reg;
	priv->clk = clk;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	/* register ccan */
	spear_can_write_reg(priv, CAN_ENABLE, 1);
	ret = register_bosch_ccandev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			DRV_NAME, ret);
		goto exit_free_device;
	}

	dev_info(&pdev->dev, "%s device registered (reg_base=%p, irq=%d)\n",
		 DRV_NAME, priv->reg_base, dev->irq);
	return 0;

exit_free_device:
	platform_set_drvdata(pdev, NULL);
	free_bosch_ccandev(dev);
exit_iounmap:
	iounmap(addr);
exit_release_mem:
	release_mem_region(mem->start, resource_size(mem));
exit_free_clk:
	clk_put(clk);
exit:
	dev_err(&pdev->dev, "probe failed\n");

	return ret;
}

static int spear_can_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct bosch_ccan_priv *priv = netdev_priv(dev);
	struct resource *mem;

	unregister_bosch_ccandev(dev);
	platform_set_drvdata(pdev, NULL);

	if (priv->reg_base)
		iounmap(priv->reg_base);

	clk_put(priv->clk);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	free_bosch_ccandev(dev);

	return 0;
}

#ifdef CONFIG_PM
static int spear_can_drv_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	return 0;
}

static int spear_can_drv_resume(struct platform_device *pdev)
{
	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver spear_can_driver = {
	.driver		= {
		.name	= DRV_NAME,
	},
	.probe		= spear_can_drv_probe,
	.remove		= spear_can_drv_remove,
#ifdef CONFIG_PM
	.suspend	= spear_can_drv_suspend,
	.resume		= spear_can_drv_resume,
#endif	/* CONFIG_PM */
};

static int __init spear_can_init(void)
{
	return platform_driver_register(&spear_can_driver);
}
module_init(spear_can_init);

static void __exit spear_can_cleanup(void)
{
	platform_driver_unregister(&spear_can_driver);
}
module_exit(spear_can_cleanup);

MODULE_AUTHOR("Bhupesh Sharma <bhupesh.sharma@st.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN bus driver for SPEAr SoC which has 2 CCAN controllers");
