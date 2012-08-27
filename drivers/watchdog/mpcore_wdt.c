/*
 * Watchdog driver for the mpcore watchdog timer
 *
 * (c) Copyright 2004 ARM Limited
 *
 * Based on the SoftDog driver:
 * (c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * Neither Alan Cox nor CymruNet Ltd. admit liability nor provide
 * warranty for any of this software. This material is provided
 * "AS-IS" and at no charge.
 *
 * (c) Copyright 1995    Alan Cox <alan@lxorguk.ukuu.org.uk>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#include <asm/smp_twd.h>

struct mpcore_wdt {
	struct watchdog_device wdd;
	struct device	*dev;
	void __iomem	*base;
	spinlock_t	lock;
	int		irq;
	unsigned int	perturb;
};

#define MIN_TIME	0x0001
#define MAX_TIME	0xFFFF
#define TIMER_MARGIN	60
static int mpcore_margin = TIMER_MARGIN;
module_param(mpcore_margin, int, 0);
MODULE_PARM_DESC(mpcore_margin,
	"MPcore timer margin in seconds. (0 < mpcore_margin < 65536, default="
				__MODULE_STRING(TIMER_MARGIN) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define ONLY_TESTING	0
static int mpcore_noboot = ONLY_TESTING;
module_param(mpcore_noboot, int, 0);
MODULE_PARM_DESC(mpcore_noboot, "MPcore watchdog action, "
	"set to 1 to ignore reboots, 0 to reboot (default="
					__MODULE_STRING(ONLY_TESTING) ")");

/*
 * This is the interrupt handler.  Note that we only use this
 * in testing mode, so don't actually do a reboot here.
 */
static irqreturn_t mpcore_wdt_fire(int irq, void *arg)
{
	struct mpcore_wdt *wdt = arg;

	/* Check it really was our interrupt */
	if (readl(wdt->base + TWD_WDOG_INTSTAT)) {
		dev_crit(wdt->dev, "Triggered - Reboot ignored.\n");
		/* Clear the interrupt on the watchdog */
		writel(1, wdt->base + TWD_WDOG_INTSTAT);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 * mpcore_wdt_ping - reload the timer
 *
 * Note that the spec says a DIFFERENT value must be written to the reload
 * register each time.  The "perturb" variable deals with this by adding 1 to
 * the count every other time the function is called.
 */
static int mpcore_wdt_ping(struct watchdog_device *wdd)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long count;

	spin_lock(&wdt->lock);
	/* Assume prescale is set to 256 */
	count =  __raw_readl(wdt->base + TWD_WDOG_COUNTER);
	count = (0xFFFFFFFFU - count) * (HZ / 5);
	count = (count / 256) * mpcore_margin;

	/* Reload the counter */
	writel(count + wdt->perturb, wdt->base + TWD_WDOG_LOAD);
	wdt->perturb = wdt->perturb ? 0 : 1;
	spin_unlock(&wdt->lock);

	return 0;
}

static int mpcore_wdt_stop(struct watchdog_device *wdd)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	writel(0x12345678, wdt->base + TWD_WDOG_DISABLE);
	writel(0x87654321, wdt->base + TWD_WDOG_DISABLE);
	writel(0x0, wdt->base + TWD_WDOG_CONTROL);
	spin_unlock(&wdt->lock);

	return 0;
}

static int mpcore_wdt_start(struct watchdog_device *wdd)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);

	dev_info(wdt->dev, "enabling watchdog.\n");

	/* This loads the count register but does NOT start the count yet */
	mpcore_wdt_ping(wdd);

	if (mpcore_noboot) {
		/* Enable watchdog - prescale=256, watchdog mode=0, enable=1 */
		writel(0x0000FF01, wdt->base + TWD_WDOG_CONTROL);
	} else {
		/* Enable watchdog - prescale=256, watchdog mode=1, enable=1 */
		writel(0x0000FF09, wdt->base + TWD_WDOG_CONTROL);
	}

	return 0;
}

static int mpcore_wdt_set_heartbeat(struct watchdog_device *wdd, unsigned int t)
{
	mpcore_margin = t;
	return 0;
}

static const struct watchdog_info mpcore_wdt_info = {
	.options		= WDIOF_SETTIMEOUT |
				  WDIOF_KEEPALIVEPING |
				  WDIOF_MAGICCLOSE,
	.identity		= "MPcore Watchdog",
};

static const struct watchdog_ops mpcore_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= mpcore_wdt_start,
	.stop		= mpcore_wdt_stop,
	.ping		= mpcore_wdt_ping,
	.set_timeout	= mpcore_wdt_set_heartbeat,
};

/*
 * System shutdown handler.  Turn off the watchdog if we're restarting or
 * halting the system.
 */
static void mpcore_wdt_shutdown(struct platform_device *pdev)
{
	struct mpcore_wdt *wdt = platform_get_drvdata(pdev);

	if (system_state == SYSTEM_RESTART || system_state == SYSTEM_HALT)
		mpcore_wdt_stop(&wdt->wdd);
}

static int __devinit mpcore_wdt_probe(struct platform_device *pdev)
{
	struct mpcore_wdt *wdt;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	wdt = devm_kzalloc(&pdev->dev, sizeof(struct mpcore_wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = &pdev->dev;
	wdt->irq = platform_get_irq(pdev, 0);
	if (wdt->irq >= 0) {
		ret = devm_request_irq(wdt->dev, wdt->irq, mpcore_wdt_fire, 0,
				"mpcore_wdt", wdt);
		if (ret) {
			dev_err(wdt->dev,
					"cannot register IRQ%d for watchdog\n",
					wdt->irq);
			return ret;
		}
	}

	wdt->base = devm_ioremap(wdt->dev, res->start, resource_size(res));
	if (!wdt->base)
		return -ENOMEM;

	wdt->wdd.info = &mpcore_wdt_info;
	wdt->wdd.ops = &mpcore_wdt_ops;
	wdt->wdd.min_timeout = MIN_TIME;
	wdt->wdd.max_timeout = MAX_TIME;
	spin_lock_init(&wdt->lock);

	watchdog_set_nowayout(&wdt->wdd, nowayout);
	platform_set_drvdata(pdev, wdt);
	watchdog_set_drvdata(&wdt->wdd, wdt);

	mpcore_wdt_stop(&wdt->wdd);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(wdt->dev, "watchdog_register_device() failed: %d\n",
				ret);
		goto err_register;
	}

	/*
	 * Check that the mpcore_margin value is within it's range; if not reset
	 * to the default
	 */
	if (mpcore_margin < MIN_TIME || mpcore_margin > MAX_TIME) {
		mpcore_margin = TIMER_MARGIN;
		dev_info(wdt->dev, "mpcore_margin value must be 0 < mpcore_margin < 65536, using %d\n",
			TIMER_MARGIN);
	}

	mpcore_wdt_set_heartbeat(NULL, mpcore_margin);
	dev_info(wdt->dev, "MPcore Watchdog Timer: 0.1. mpcore_noboot=%d "
			"mpcore_margin=%d sec (nowayout= %d)\n", mpcore_noboot,
			mpcore_margin, nowayout);

	return 0;

err_register:
	platform_set_drvdata(pdev, NULL);
	watchdog_set_drvdata(&wdt->wdd, NULL);

	return ret;
}

static int __devexit mpcore_wdt_remove(struct platform_device *pdev)
{
	struct mpcore_wdt *wdt = platform_get_drvdata(pdev);

	watchdog_unregister_device(&wdt->wdd);
	platform_set_drvdata(pdev, NULL);
	watchdog_set_drvdata(&wdt->wdd, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int mpcore_wdt_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct mpcore_wdt *wdt = platform_get_drvdata(pdev);
	mpcore_wdt_stop(&wdt->wdd);	/* Turn the WDT off */

	return 0;
}

static int mpcore_wdt_resume(struct platform_device *pdev)
{
	struct mpcore_wdt *wdt = platform_get_drvdata(pdev);

	if (watchdog_active(&wdt->wdd))
		mpcore_wdt_start(&wdt->wdd);

	return 0;
}
#else
#define mpcore_wdt_suspend	NULL
#define mpcore_wdt_resume	NULL
#endif

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:mpcore_wdt");

static struct platform_driver mpcore_wdt_driver = {
	.probe		= mpcore_wdt_probe,
	.remove		= __devexit_p(mpcore_wdt_remove),
	.suspend	= mpcore_wdt_suspend,
	.resume		= mpcore_wdt_resume,
	.shutdown	= mpcore_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "mpcore_wdt",
	},
};

module_platform_driver(mpcore_wdt_driver);

MODULE_AUTHOR("ARM Limited");
MODULE_DESCRIPTION("MPcore Watchdog Device Driver");
MODULE_LICENSE("GPL");
