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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/watchdog.h>

/*
 * TWD_WDOG is at offset 0x20 from TWD base address. Following register offsets
 * doesn't contain this extra 0x20 offset, i.e. users of TWD_WDOG must pass base
 * address of WDOG to WDOG driver instead of TWD module.
 */
#define TWD_WDOG_LOAD				0x00
#define TWD_WDOG_COUNTER			0x04
#define TWD_WDOG_CONTROL			0x08
#define TWD_WDOG_INTSTAT			0x0C
#define TWD_WDOG_RESETSTAT			0x10
#define TWD_WDOG_DISABLE			0x14

#define TWD_WDOG_LOAD_MIN			0x00000000
#define TWD_WDOG_LOAD_MAX			0xFFFFFFFF

#define TWD_WDOG_CONTROL_ENABLE			(1 << 0)
#define TWD_WDOG_CONTROL_IRQ_ENABLE		(1 << 2)
#define TWD_WDOG_CONTROL_WDT_MODE		(1 << 3)
#define TWD_WDOG_CONTROL_WDT_PRESCALE(x)	((x) << 8)
#define TWD_WDOG_CONTROL_PRESCALE_MIN		0x00
#define TWD_WDOG_CONTROL_PRESCALE_MAX		0xFF

#define TWD_WDOG_RESETSTAT_MASK			0x1

struct mpcore_wdt {
	struct watchdog_device wdd;
	struct device	*dev;
	void __iomem	*base;
	spinlock_t	lock;
	struct clk	*clk;
	unsigned long	clk_rate;	/* In Hz */
	int		irq;
	unsigned int	prescale;
	unsigned int	load_val;
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

static int clk_rate;
module_param(clk_rate, int, 0);
MODULE_PARM_DESC(clk_rate,
	"Watchdog clock rate in Hz, required if clk_get() fails");

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
	if (readl_relaxed(wdt->base + TWD_WDOG_INTSTAT)) {
		dev_crit(wdt->dev, "Triggered - Reboot ignored.\n");
		/* Clear the interrupt on the watchdog */
		writel_relaxed(1, wdt->base + TWD_WDOG_INTSTAT);
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

	spin_lock(&wdt->lock);
	writel_relaxed(wdt->load_val + wdt->perturb, wdt->base + TWD_WDOG_LOAD);
	wdt->perturb = !wdt->perturb;
	spin_unlock(&wdt->lock);

	return 0;
}

static void __mpcore_wdt_stop(struct mpcore_wdt *wdt)
{
	spin_lock(&wdt->lock);
	writel_relaxed(0x12345678, wdt->base + TWD_WDOG_DISABLE);
	writel_relaxed(0x87654321, wdt->base + TWD_WDOG_DISABLE);
	writel_relaxed(0x0, wdt->base + TWD_WDOG_CONTROL);
	spin_unlock(&wdt->lock);
}

static int mpcore_wdt_stop(struct watchdog_device *wdd)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);

	__mpcore_wdt_stop(wdt);

	if (!IS_ERR(wdt->clk)) {
		clk_disable(wdt->clk);
		clk_unprepare(wdt->clk);
	}

	return 0;
}

static int mpcore_wdt_start(struct watchdog_device *wdd)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 val, mode;
	int ret;

	dev_info(wdt->dev, "enabling watchdog.\n");

	if (!IS_ERR(wdt->clk)) {
		ret = clk_prepare(wdt->clk);
		if (ret) {
			dev_err(wdt->dev, "clock prepare fail");
			return ret;
		}

		ret = clk_enable(wdt->clk);
		if (ret) {
			dev_err(wdt->dev, "Clock enable failed\n");
			clk_unprepare(wdt->clk);
			return ret;
		}
	}

	/* This loads the count register but does NOT start the count yet */
	mpcore_wdt_ping(wdd);

	if (mpcore_noboot)
		mode = 0;
	else
		mode = TWD_WDOG_CONTROL_WDT_MODE;

	spin_lock(&wdt->lock);

	val = TWD_WDOG_CONTROL_WDT_PRESCALE(wdt->prescale) |
		TWD_WDOG_CONTROL_ENABLE | mode;
	writel_relaxed(val, wdt->base + TWD_WDOG_CONTROL);

	spin_unlock(&wdt->lock);

	return 0;
}

/* binary search */
static inline void bsearch(u32 *var, u32 var_start, u32 var_end,
		const u64 param, const u32 timeout, u32 rate)
{
	u64 tmp = 0;

	/* get the lowest var value that can satisfy our requirement */
	while (var_start < var_end) {
		tmp = var_start;
		tmp += var_end;
		tmp = div_u64(tmp, 2);
		if (timeout > div_u64((param + 1) * (tmp + 1), rate)) {
			if (var_start == tmp)
				break;
			else
				var_start = tmp;
		} else {
			if (var_end == tmp)
				break;
			else
				var_end = tmp;
		}
	}
	*var = tmp;
}

static int
mpcore_wdt_set_heartbeat(struct watchdog_device *wdd, unsigned int timeout)
{
	struct mpcore_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int psc, rate = wdt->clk_rate;
	u64 load = 0;

	/* get appropriate value of psc and load */
	bsearch(&psc, TWD_WDOG_CONTROL_PRESCALE_MIN,
			TWD_WDOG_CONTROL_PRESCALE_MAX, TWD_WDOG_LOAD_MAX,
			timeout, rate);
	bsearch((u32 *)&load, TWD_WDOG_LOAD_MIN, TWD_WDOG_LOAD_MAX, psc,
			timeout, rate);

	spin_lock(&wdt->lock);
	wdt->prescale = psc;
	wdt->load_val = load;

	/* roundup timeout to closest positive integer value */
	mpcore_margin = div_u64((psc + 1) * (load + 1) + (rate / 2), rate);
	spin_unlock(&wdt->lock);

	return 0;
}

static const struct watchdog_info mpcore_wdt_info = {
	.options		= WDIOF_SETTIMEOUT |
				  WDIOF_KEEPALIVEPING |
				  WDIOF_CARDRESET |
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
		__mpcore_wdt_stop(wdt);
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

	wdt->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(wdt->clk)) {
		dev_warn(&pdev->dev, "Clock not found\n");
		wdt->clk_rate = clk_rate;
	} else {
		wdt->clk_rate = clk_get_rate(wdt->clk);

		ret = clk_prepare(wdt->clk);
		if (ret) {
			dev_err(wdt->dev, "clock prepare fail");
			goto err_put_clk;
		}

		ret = clk_enable(wdt->clk);
		if (ret) {
			dev_err(&pdev->dev, "Clock enable failed\n");
			clk_unprepare(wdt->clk);
			goto err_put_clk;
		}
	}

	wdt->wdd.bootstatus = (readl_relaxed(wdt->base + TWD_WDOG_RESETSTAT) &
			TWD_WDOG_RESETSTAT_MASK) ? WDIOF_CARDRESET : 0;

	mpcore_wdt_stop(&wdt->wdd);

	if (!wdt->clk_rate) {
		dev_err(&pdev->dev, "Clock rate can't be zero\n");
		ret = -EINVAL;
		goto err_put_clk;
	}

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(wdt->dev, "watchdog_register_device() failed: %d\n",
				ret);
		goto err_put_clk;
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

	mpcore_wdt_set_heartbeat(&wdt->wdd, mpcore_margin);
	dev_info(wdt->dev, "MPcore Watchdog Timer: 0.1. mpcore_noboot=%d "
			"mpcore_margin=%d sec (nowayout= %d)\n", mpcore_noboot,
			mpcore_margin, nowayout);

	return 0;

err_put_clk:
	if (!IS_ERR(wdt->clk))
		clk_put(wdt->clk);

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
static int mpcore_wdt_suspend(struct device *dev)
{
	struct mpcore_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return mpcore_wdt_stop(&wdt->wdd);	/* Turn the WDT off */

	return 0;
}

static int mpcore_wdt_resume(struct device *dev)
{
	struct mpcore_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return mpcore_wdt_start(&wdt->wdd);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mpcore_wdt_dev_pm_ops, mpcore_wdt_suspend,
		mpcore_wdt_resume);

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:mpcore_wdt");

static struct platform_driver mpcore_wdt_driver = {
	.probe		= mpcore_wdt_probe,
	.remove		= __devexit_p(mpcore_wdt_remove),
	.shutdown	= mpcore_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "mpcore_wdt",
		.pm	= &mpcore_wdt_dev_pm_ops,
	},
};

module_platform_driver(mpcore_wdt_driver);

MODULE_AUTHOR("ARM Limited");
MODULE_DESCRIPTION("MPcore Watchdog Device Driver");
MODULE_LICENSE("GPL");
