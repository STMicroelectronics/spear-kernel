/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/i2c/i2c-designware.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "i2c-designware-core.h"

static struct i2c_algorithm i2c_dw_algo = {
	.master_xfer	= i2c_dw_xfer,
	.functionality	= i2c_dw_func,
};
static u32 i2c_dw_get_clk_rate_khz(struct dw_i2c_dev *dev)
{
	return clk_get_rate(dev->clk)/1000;
}

static int __devinit dw_i2c_parse_dt(struct platform_device *pdev,
		struct dw_i2c_dev *dev)
{
	struct device_node *np = pdev->dev.of_node;
	struct i2c_adapter *adap = &dev->adapter;
	struct i2c_bus_recovery_info *dw_recovery_info;
	enum of_gpio_flags flags;

	if (!np) {
		dev_err(&pdev->dev, "Missing DT data\n");
		return -EINVAL;
	}

	if (of_property_read_bool(np, "stop-control"))
		dev->stop_control = true;

	if (of_property_read_bool(np, "write-16bit"))
		dev->accessor_flags |= ACCESS_WRITE_16BIT;

	dw_recovery_info = adap->bus_recovery_info;

	if (of_property_read_bool(np, "recovery,gpio"))
		dw_recovery_info->is_gpio_recovery = true;
	else
		return -EINVAL;

	dw_recovery_info->scl_gpio = of_get_named_gpio_flags(np,
			"recovery,scl-gpio", 0, &flags);
	dw_recovery_info->scl_gpio_flags = flags;

	if (of_property_read_bool(np, "recovery,skip-sda-poll"))
		dw_recovery_info->skip_sda_polling = true;

	if (!dw_recovery_info->skip_sda_polling) {
		dw_recovery_info->sda_gpio = of_get_named_gpio_flags(np,
				"recovery,sda-gpio", 0, &flags);
		dw_recovery_info->sda_gpio_flags = flags;
	}

	dw_recovery_info->clock_rate_khz =
		clk_get_rate(dev->clk) / 1000;

	return 0;
}

static int __devinit dw_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dw_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct resource *mem, *ioarea;
	struct i2c_dw_pdata *pdata;
	struct i2c_bus_recovery_info *dw_recovery_info = NULL;
	int irq, r;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq; /* -ENXIO */
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct dw_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}

	init_completion(&dev->cmd_complete);
	mutex_init(&dev->lock);
	dev->dev = get_device(&pdev->dev);
	dev->irq = irq;
	platform_set_drvdata(pdev, dev);

	dev->clk = clk_get(&pdev->dev, NULL);
	dev->get_clk_rate_khz = i2c_dw_get_clk_rate_khz;

	if (IS_ERR(dev->clk)) {
		r = -ENODEV;
		goto err_free_mem;
	}
	clk_prepare_enable(dev->clk);

	dev->functionality =
		I2C_FUNC_I2C |
		I2C_FUNC_10BIT_ADDR |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
	dev->master_cfg =  DW_IC_CON_MASTER | DW_IC_CON_SLAVE_DISABLE |
		DW_IC_CON_RESTART_EN | DW_IC_CON_SPEED_FAST;

	dev->base = ioremap(mem->start, resource_size(mem));
	if (dev->base == NULL) {
		dev_err(&pdev->dev, "failure mapping io resources\n");
		r = -EBUSY;
		goto err_unuse_clocks;
	}

	pdata = dev_get_platdata(&pdev->dev);

	{
		u32 param1 = i2c_dw_read_comp_param(dev);

		dev->tx_fifo_depth = ((param1 >> 16) & 0xff) + 1;
		dev->rx_fifo_depth = ((param1 >> 8)  & 0xff) + 1;
	}
	r = i2c_dw_init(dev);
	if (r)
		goto err_iounmap;

	i2c_dw_disable_int(dev);
	r = request_irq(dev->irq, i2c_dw_isr, IRQF_DISABLED, pdev->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		goto err_iounmap;
	}

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "Synopsys DesignWare I2C adapter",
			sizeof(adap->name));
	adap->algo = &i2c_dw_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = np;

	adap->nr = pdev->id;

	dw_recovery_info =
		kzalloc(sizeof(*dw_recovery_info), GFP_KERNEL);
	if (!dw_recovery_info) {
		r = -ENOMEM;
		goto err_free_irq;
	}

	adap->bus_recovery_info = dw_recovery_info;
	if (pdata) {
		dw_recovery_info->get_gpio = pdata->get_gpio;
		dw_recovery_info->put_gpio = pdata->put_gpio;
		dw_recovery_info->scl_gpio = pdata->scl_gpio;
		dw_recovery_info->scl_gpio_flags = pdata->scl_gpio_flags;
		dw_recovery_info->recover_bus = pdata->recover_bus;
		dw_recovery_info->is_gpio_recovery = pdata->is_gpio_recovery;
		dw_recovery_info->clock_rate_khz =
			clk_get_rate(dev->clk) / 1000;

		if (!pdata->skip_sda_polling) {
			dw_recovery_info->sda_gpio = pdata->sda_gpio;
			dw_recovery_info->sda_gpio_flags =
				pdata->sda_gpio_flags;
		}
	} else {
		r = dw_i2c_parse_dt(pdev, dev);
		if (r) {
			kfree(dw_recovery_info);
			adap->bus_recovery_info = NULL;
		}
	}

	dev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dev->pinctrl)) {
		dev_dbg(&pdev->dev, "could not get pinctrl instance\n");
	} else {
		dev->pins_default = pinctrl_lookup_state(dev->pinctrl,
						 PINCTRL_STATE_DEFAULT);
		if (IS_ERR(dev->pins_default))
			dev_dbg(&pdev->dev, "could not get default pinstate\n");

		dev->pins_sleep = pinctrl_lookup_state(dev->pinctrl,
						PINCTRL_STATE_SLEEP);
		if (IS_ERR(dev->pins_sleep))
			dev_dbg(&pdev->dev, "could not get sleep pinstate\n");
	}

	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		goto err_free_recovery;
	}
	of_i2c_register_devices(adap);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	return 0;

err_free_recovery:
	kfree(dw_recovery_info);
err_free_irq:
	free_irq(dev->irq, dev);
err_iounmap:
	iounmap(dev->base);
err_unuse_clocks:
	clk_disable_unprepare(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	put_device(&pdev->dev);
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return r;
}

static int __devexit dw_i2c_remove(struct platform_device *pdev)
{
	struct dw_i2c_dev *dev = platform_get_drvdata(pdev);
	struct i2c_dw_pdata *pdata;
	struct resource *mem;

	pdata = dev_get_platdata(&pdev->dev);
	if (pdata)
		kfree(dev->adapter.bus_recovery_info);
	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&dev->adapter);
	put_device(&pdev->dev);

	clk_disable_unprepare(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;

	i2c_dw_disable(dev);
	pm_runtime_disable(&pdev->dev);
	free_irq(dev->irq, dev);
	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw_i2c_of_match[] = {
	{ .compatible = "snps,designware-i2c", },
	{},
};
MODULE_DEVICE_TABLE(of, dw_i2c_of_match);
#endif

#ifdef CONFIG_PM
static int dw_i2c_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2c_dev *i_dev = platform_get_drvdata(pdev);

	pm_runtime_get_sync(dev);
	clk_disable_unprepare(i_dev->clk);

	return 0;
}

static int dw_i2c_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2c_dev *i_dev = platform_get_drvdata(pdev);

	clk_prepare_enable(i_dev->clk);
	i2c_dw_init(i_dev);
	pm_runtime_put(dev);

	return 0;
}

static int dw_i2c_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2c_dev *i_dev = platform_get_drvdata(pdev);

	clk_disable_unprepare(i_dev->clk);
	return 0;
}

static int dw_i2c_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_i2c_dev *i_dev = platform_get_drvdata(pdev);

	return clk_prepare_enable(i_dev->clk);
}

static int dw_i2c_runtime_idle(struct device *dev)
{
	return pm_schedule_suspend(dev, 500);
}

#endif

const struct dev_pm_ops dw_i2c_dev_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(dw_i2c_suspend, dw_i2c_resume)
        SET_RUNTIME_PM_OPS(dw_i2c_runtime_suspend, dw_i2c_runtime_resume, dw_i2c_runtime_idle)
};


/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_designware");

static struct platform_driver dw_i2c_driver = {
	.remove		= __devexit_p(dw_i2c_remove),
	.driver		= {
		.name	= "i2c_designware",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dw_i2c_of_match),
		.pm	= &dw_i2c_dev_pm_ops,
	},
};

static int __init dw_i2c_init_driver(void)
{
	return platform_driver_probe(&dw_i2c_driver, dw_i2c_probe);
}
subsys_initcall(dw_i2c_init_driver);

static void __exit dw_i2c_exit_driver(void)
{
	platform_driver_unregister(&dw_i2c_driver);
}
module_exit(dw_i2c_exit_driver);

MODULE_AUTHOR("Baruch Siach <baruch@tkos.co.il>");
MODULE_DESCRIPTION("Synopsys DesignWare I2C bus adapter");
MODULE_LICENSE("GPL");
