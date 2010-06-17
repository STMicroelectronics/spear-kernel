/*
 * drivers/input/keyboard/keyboard-spear.c
 *
 * SPEAr Keyboard Driver
 * Based on omap-keypad driver
 *
 * Copyright (C) 2010 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/types.h>
#include <plat/keyboard.h>

/* Keyboard Regsiters */
#define MODE_REG	0x00	/* 16 bit reg */
#define STATUS_REG	0x0C	/* 2 bit reg */
#define DATA_REG	0x10	/* 8 bit reg */
#define INTR_MASK	0x54

/* Register Values */
/*
 * pclk freq mask = (APB FEQ -1)= 82 MHZ.Programme bit 15-9 in mode
 * control register as 1010010(82MHZ)
 */
#define PCLK_FREQ_MSK	0xA400	/* 82 MHz */
#define START_SCAN	0x0100
#define SCAN_RATE_10	0x0000
#define SCAN_RATE_20	0x0004
#define SCAN_RATE_40	0x0008
#define SCAN_RATE_80	0x000C
#define MODE_KEYBOARD	0x0002
#define DATA_AVAIL	0x2

#define KEY_MASK	0xFF000000
#define KEY_VALUE	0x00FFFFFF
#define ROW_MASK	0xF0
#define COLUMN_MASK	0x0F
#define ROW_SHIFT	4

struct spear_kbd {
	struct input_dev *input;
	void __iomem *io_base;		/* Keyboard Base Address */
	struct clk *clk;
	int *keymap;
};
/* TODO: Need to optimize this function */
static inline int get_key_value(struct spear_kbd *dev, int row, int col)
{
	int i, key;
	int *keymap = dev->keymap;

	key = KEY(row, col, 0);
	for (i = 0; keymap[i] != 0; i++)
		if ((keymap[i] & KEY_MASK) == key)
			return keymap[i] & KEY_VALUE;
	return -ENOKEY;
}

static irqreturn_t spear_kbd_interrupt(int irq, void *dev_id)
{
	struct spear_kbd *dev = dev_id;
	static u8 last_key ;
	static u8 last_event;
	int key;
	u8 sts, val = 0;

	if (dev == NULL) {
		pr_err("Keyboard: Invalid dev_id in irq handler\n");
		return IRQ_NONE;
	}

	sts = readb(dev->io_base + STATUS_REG);
	if (sts & DATA_AVAIL) {
		/* following reads active (row, col) pair */
		val = readb(dev->io_base + DATA_REG);
		key = get_key_value(dev, (val & ROW_MASK)>>ROW_SHIFT, (val
					& COLUMN_MASK));

		/* valid key press event */
		if (key >= 0) {
			if (last_event == 1) {
				/* check if we missed a release event */
				input_report_key(dev->input, last_key,
						!last_event);
			}
			/* notify key press */
			last_event = 1;
			last_key = key;
			input_report_key(dev->input, key, last_event);
		} else {
			/* notify key release */
			last_event = 0;
			input_report_key(dev->input, last_key, last_event);
		}
	} else
		return IRQ_NONE;

	/* clear interrupt */
	writeb(0, dev->io_base + STATUS_REG);

	return IRQ_HANDLED;
}

static int __init spear_kbd_probe(struct platform_device *pdev)
{
	struct spear_kbd *kbd;
	struct kbd_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res;
	int i, ret, irq;
	u16 val = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "Invalid platform data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no keyboard resource defined\n");
		return -EBUSY;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "keyboard region already claimed\n");
		return -EBUSY;
	}

	kbd = kzalloc(sizeof(*kbd), GFP_KERNEL);
	if (!kbd) {
		dev_err(&pdev->dev, "out of memory\n");
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

	kbd->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(kbd->clk)) {
		ret = PTR_ERR(kbd->clk);
		goto err_kfree;
	}

	ret = clk_enable(kbd->clk);
	if (ret < 0)
		goto err_clk_put;

	platform_set_drvdata(pdev, kbd);
	kbd->keymap = pdata->keymap; /* key mappings */

	kbd->io_base = ioremap(res->start, resource_size(res));
	if (!kbd->io_base) {
		dev_err(&pdev->dev, "ioremap fail for kbd_region\n");
		ret = -ENOMEM;
		goto err_clear_plat_data;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "not able to get irq for the device\n");
		ret = irq;
		goto err_iounmap;
	}

	ret = request_irq(irq, spear_kbd_interrupt, 0, "keyboard",
			kbd);
	if (ret) {
		dev_err(&pdev->dev, "request_irq fail\n");
		goto err_iounmap;
	}

	kbd->input = input_allocate_device();
	if (!kbd->input) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "input device allocation fail\n");
		goto err_free_irq;
	}

	if (pdata->rep)
		__set_bit(EV_REP, kbd->input->evbit);

	/* setup input device */
	__set_bit(EV_KEY, kbd->input->evbit);

	for (i = 0; kbd->keymap[i] != 0; i++)
		__set_bit(kbd->keymap[i] & KEY_MAX, kbd->input->keybit);

	kbd->input->name = "keyboard";
	kbd->input->phys = "keyboard/input0";
	kbd->input->dev.parent = &pdev->dev;
	kbd->input->id.bustype = BUS_HOST;
	kbd->input->id.vendor = 0x0001;
	kbd->input->id.product = 0x0001;
	kbd->input->id.version = 0x0100;

	ret = input_register_device(kbd->input);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register keyboard device\n");
		goto err_free_dev;
	}

	/* program keyboard */
	val |= SCAN_RATE_80 | MODE_KEYBOARD | PCLK_FREQ_MSK;
	writew(val, kbd->io_base + MODE_REG);

	writeb(1, kbd->io_base + STATUS_REG);

	/* start key scan */
	val |= START_SCAN;
	writew(val, kbd->io_base + MODE_REG);

	device_init_wakeup(&pdev->dev, 1);

	return 0;

err_free_dev:
	input_free_device(kbd->input);
err_free_irq:
	free_irq(irq, pdev);
err_iounmap:
	iounmap(kbd->io_base);
err_clear_plat_data:
	platform_set_drvdata(pdev, NULL);
	clk_disable(kbd->clk);
err_clk_put:
	clk_put(kbd->clk);
err_kfree:
	kfree(kbd);
err_release_mem_region:
	release_mem_region(res->start, resource_size(res));

	return ret;
}

static int spear_kbd_remove(struct platform_device *pdev)
{
	struct spear_kbd *kbd = platform_get_drvdata(pdev);
	struct resource *res;
	u16 val;
	int irq;

	val = readw(kbd->io_base + MODE_REG);
	val &= ~START_SCAN;
	writew(val, kbd->io_base + MODE_REG);

	/* unregister input device */
	input_unregister_device(kbd->input);
	input_free_device(kbd->input);

	irq = platform_get_irq(pdev, 0);
	if (irq)
		free_irq(irq, pdev);

	iounmap(kbd->io_base);
	platform_set_drvdata(pdev, NULL);
	clk_disable(kbd->clk);
	clk_put(kbd->clk);
	kfree(kbd);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));

	return 0;
}

#ifdef CONFIG_PM
static int spear_kbd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spear_kbd *kbd = platform_get_drvdata(pdev);
	int irq;

	irq = platform_get_irq(pdev, 0);
	clk_disable(kbd->clk);
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(irq);

	return 0;
}

static int spear_kbd_resume(struct platform_device *pdev)
{
	struct spear_kbd *kbd = platform_get_drvdata(pdev);
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(irq);
	clk_enable(kbd->clk);

	return 0;
}
#else
#define spear_kbd_suspend	NULL
#define spear_kbd_resume	NULL
#endif

static struct platform_driver spear_kbd_driver = {
	.probe		= spear_kbd_probe,
	.remove		= spear_kbd_remove,
	.suspend	= spear_kbd_suspend,
	.resume		= spear_kbd_resume,
	.driver		= {
		.name	= "keyboard",
		.owner	= THIS_MODULE,
	},
};

static int __devinit spear_kbd_init(void)
{
	return platform_driver_register(&spear_kbd_driver);
}
module_init(spear_kbd_init);

static void __exit spear_kbd_exit(void)
{
	platform_driver_unregister(&spear_kbd_driver);
}
module_exit(spear_kbd_exit);

MODULE_AUTHOR("Rajeev Kumar");
MODULE_DESCRIPTION("SPEAr Keyboard Driver");
MODULE_LICENSE("GPL");
