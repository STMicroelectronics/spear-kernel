/*
 * arch/arm/plat-spear/plgpio.c
 *
 * SPEAr platform PLGPIO driver source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>

#define MAX_GPIO_PER_REG		32
#define PIN_OFFSET(pin)			(pin % MAX_GPIO_PER_REG)
#define REG_OFFSET(base, reg, pin)	(base + reg + (pin / MAX_GPIO_PER_REG)\
		* sizeof(int *))

/*
 * struct plgpio: plgpio driver specific structure
 *
 * lock: lock for guarding gpio registers
 * base: base address of plgpio block
 * irq_base: irq number of plgpio0
 * chip: gpio framework specific chip information structure
 * grp_size: number of gpio's in a group for interrupt registers
 * p2o: function ptr for pin to offset conversion. This is required only for
 * machines where mapping b/w pin and offset is not 1-to-1.
 * o2p: function ptr for offset to pin conversion. This is required only for
 * machines where mapping b/w pin and offset is not 1-to-1.
 * p2o_regs: mask of registers for which p2o and o2p are applicable
 * regs: register offsets
 */
struct plgpio {
	spinlock_t		lock;
	void __iomem		*base;
	unsigned		irq_base;
	struct gpio_chip	chip;
	u32			grp_size;
	int			(*p2o)(int pin);	/* pin_to_offset */
	int			(*o2p)(int offset);	/* offset_to_pin */
	u32			p2o_regs;
	struct plgpio_regs	regs;
};

/* register manipulation inline functions */
static inline u32 is_plgpio_set(void __iomem *base, u32 pin, u32 reg)
{
	u32 offset = PIN_OFFSET(pin);
	void __iomem *reg_off = REG_OFFSET(base, reg, pin);
	u32 val = readl(reg_off);

	return val & (1 << offset);
}

static inline void plgpio_reg_set(void __iomem *base, u32 pin, u32 reg)
{
	u32 offset = PIN_OFFSET(pin);
	void __iomem *reg_off = REG_OFFSET(base, reg, pin);
	u32 val = readl(reg_off);

	writel(val | (1 << offset), reg_off);
}

static inline void plgpio_reg_reset(void __iomem *base, u32 pin, u32 reg)
{
	u32 offset = PIN_OFFSET(pin);
	void __iomem *reg_off = REG_OFFSET(base, reg, pin);
	u32 val = readl(reg_off);

	writel(val & ~(1 << offset), reg_off);
}

/* gpio framework specific routines */
static int plgpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);
	unsigned long flags;

	if (offset >= chip->ngpio)
		return -EINVAL;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_DIR_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return -EINVAL;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_set(plgpio->base, offset, plgpio->regs.dir);
	spin_unlock_irqrestore(&plgpio->lock, flags);

	return 0;
}

static int plgpio_direction_output(struct gpio_chip *chip, unsigned offset,
		int value)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);
	unsigned long flags;
	unsigned dir_offset = offset, wdata_offset = offset, tmp;

	if (offset >= chip->ngpio)
		return -EINVAL;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & (PTO_DIR_REG | PTO_WDATA_REG))) {
		tmp = plgpio->p2o(offset);
		if (tmp == -1)
			return -EINVAL;

		if (plgpio->p2o_regs & PTO_DIR_REG)
			dir_offset = tmp;
		if (plgpio->p2o_regs & PTO_WDATA_REG)
			wdata_offset = tmp;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_reset(plgpio->base, dir_offset, plgpio->regs.dir);
	if (value)
		plgpio_reg_set(plgpio->base, wdata_offset,
				plgpio->regs.wdata);
	else
		plgpio_reg_reset(plgpio->base, wdata_offset,
				plgpio->regs.wdata);
	spin_unlock_irqrestore(&plgpio->lock, flags);

	return 0;
}

static int plgpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);

	if (offset >= chip->ngpio)
		return -EINVAL;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_RDATA_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return -EINVAL;
	}

	return is_plgpio_set(plgpio->base, offset, plgpio->regs.rdata);
}

static void plgpio_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);

	if (offset >= chip->ngpio)
		return;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_WDATA_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return;
	}

	if (value)
		plgpio_reg_set(plgpio->base, offset, plgpio->regs.wdata);
	else
		plgpio_reg_reset(plgpio->base, offset, plgpio->regs.wdata);
}

static int plgpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);
	unsigned long flags;
	int ret = 0;

	if (offset >= chip->ngpio)
		return -EINVAL;

	/*
	 * put gpio in IN mode before enabling it. This make enabling gpio safe
	 */
	ret = plgpio_direction_input(chip, offset);
	if (ret)
		return ret;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_ENB_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return -EINVAL;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_set(plgpio->base, offset, plgpio->regs.enb);
	spin_unlock_irqrestore(&plgpio->lock, flags);
	return 0;
}

static void plgpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);
	unsigned long flags;

	if (offset >= chip->ngpio)
		return;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_ENB_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_reset(plgpio->base, offset, plgpio->regs.enb);
	spin_unlock_irqrestore(&plgpio->lock, flags);
}

static int plgpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct plgpio *plgpio = container_of(chip, struct plgpio, chip);
	int size = plgpio->grp_size ? plgpio->grp_size : 1;

	if (plgpio->irq_base == (unsigned) -1)
		return -EINVAL;

	return plgpio->irq_base + offset / size;
}

/* PLGPIO IRQ */
static void plgpio_irq_mask(unsigned irq)
{
	struct plgpio *plgpio = get_irq_chip_data(irq);
	int offset = irq - plgpio->irq_base;
	unsigned long flags;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_IE_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_set(plgpio->base, offset, plgpio->regs.ie);
	spin_unlock_irqrestore(&plgpio->lock, flags);
}

static void plgpio_irq_unmask(unsigned irq)
{
	struct plgpio *plgpio = get_irq_chip_data(irq);
	int offset = irq - plgpio->irq_base;
	unsigned long flags;

	/* get correct offset for "offset" pin */
	if (plgpio->p2o && (plgpio->p2o_regs & PTO_IE_REG)) {
		offset = plgpio->p2o(offset);
		if (offset == -1)
			return;
	}

	spin_lock_irqsave(&plgpio->lock, flags);
	plgpio_reg_reset(plgpio->base, offset, plgpio->regs.ie);
	spin_unlock_irqrestore(&plgpio->lock, flags);
}

static int plgpio_irq_type(unsigned irq, unsigned trigger)
{
	struct plgpio *plgpio = get_irq_chip_data(irq);
	int size = plgpio->grp_size ? plgpio->grp_size : 1;
	int offset = irq - plgpio->irq_base;

	if (offset >= DIV_ROUND_UP(plgpio->chip.ngpio, size))
		return -EINVAL;

	if (trigger != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;
	return 0;
}

static struct irq_chip plgpio_irqchip = {
	.name		= "PLGPIO",
	.mask		= plgpio_irq_mask,
	.unmask		= plgpio_irq_unmask,
	.set_type	= plgpio_irq_type,
};

static void plgpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct plgpio *plgpio = get_irq_data(irq);
	unsigned long pending;
	int regs_count, size, count, pin, offset, i = 0;

	size = plgpio->grp_size ? plgpio->grp_size : 1;
	count = DIV_ROUND_UP(plgpio->chip.ngpio, size);
	regs_count = DIV_ROUND_UP(count, MAX_GPIO_PER_REG);

	/* check all plgpio MIS registers for a possible interrupt */
	for (; i < regs_count; i++) {
		pending = readl(plgpio->base + plgpio->regs.mis +
				i * sizeof(int *));
		if (!pending)
			continue;

		/*
		 * clear extra bits in last register having gpios < MAX/REG
		 * ex: Suppose there are max 102 plgpios. then last register
		 * must have only (102 - MAX_GPIO_PER_REG * 3) = 6 relevant bits
		 * so, we must not take other 28 bits into consideration for
		 * checking interrupt. so clear those bits.
		 */
		count = count - i * MAX_GPIO_PER_REG;
		if (count < MAX_GPIO_PER_REG)
			pending &= (1 << count) - 1;

		for_each_set_bit(offset, &pending, MAX_GPIO_PER_REG) {
			/* get correct pin for "offset" */
			if (plgpio->o2p && (plgpio->p2o_regs & PTO_MIS_REG)) {
				pin = plgpio->o2p(offset);
				if (pin == -1)
					continue;
			} else
				pin = offset;

			/* get correct irq line number */
			pin = i * MAX_GPIO_PER_REG + pin;
			/* get correct gpio pin number */
			pin *= size;
			generic_handle_irq(plgpio_to_irq(&plgpio->chip, pin));
		}
	}
}

static int __devinit plgpio_probe(struct platform_device *pdev)
{
	struct plgpio_platform_data *pdata;
	struct plgpio *plgpio;
	int ret, irq, i, count;
	struct resource *res;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "invalid platform data\n");
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -EBUSY;
		dev_dbg(&pdev->dev, "invalid IORESOURCE_MEM\n");
		goto fail;
	}

	if (!request_mem_region(res->start, resource_size(res), "plgpio")) {
		ret = -EBUSY;
		dev_dbg(&pdev->dev, "request mem region fail\n");
		goto fail;
	}

	plgpio = kzalloc(sizeof(*plgpio), GFP_KERNEL);
	if (!plgpio) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "memory allocation fail\n");
		goto release_region;
	}

	plgpio->base = ioremap(res->start, resource_size(res));
	if (!plgpio->base) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "ioremap fail\n");
		goto kfree;
	}

	spin_lock_init(&plgpio->lock);

	plgpio->chip.request = plgpio_request;
	plgpio->chip.free = plgpio_free;
	plgpio->chip.direction_input = plgpio_direction_input;
	plgpio->chip.direction_output = plgpio_direction_output;
	plgpio->chip.get = plgpio_get_value;
	plgpio->chip.set = plgpio_set_value;
	plgpio->chip.to_irq = plgpio_to_irq;
	plgpio->chip.base = pdata->gpio_base;
	plgpio->chip.ngpio = pdata->gpio_count;
	plgpio->chip.label = dev_name(&pdev->dev);
	plgpio->chip.dev = &pdev->dev;
	plgpio->chip.owner = THIS_MODULE;
	plgpio->irq_base = pdata->irq_base;
	plgpio->grp_size = pdata->grp_size;
	plgpio->p2o = pdata->p2o;
	plgpio->o2p = pdata->o2p;
	plgpio->p2o_regs = pdata->p2o_regs;
	plgpio->regs.enb = pdata->regs.enb;
	plgpio->regs.wdata = pdata->regs.wdata;
	plgpio->regs.dir = pdata->regs.dir;
	plgpio->regs.ie = pdata->regs.ie;
	plgpio->regs.rdata = pdata->regs.rdata;
	plgpio->regs.mis = pdata->regs.mis;

	ret = gpiochip_add(&plgpio->chip);
	if (ret) {
		dev_dbg(&pdev->dev, "unable to add gpio chip\n");
		goto iounmap;
	}

	/* irq_chip support */
	if (pdata->irq_base == (unsigned) -1) {
		dev_info(&pdev->dev, "Initialization successful\n");
		return 0;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "invalid irq number\n");
		goto remove_gpiochip;
	}

	count = pdata->grp_size ? pdata->grp_size : 1;
	count = DIV_ROUND_UP(pdata->gpio_count, count);
	set_irq_chained_handler(irq, plgpio_irq_handler);
	for (i = 0; i < count; i++) {
		set_irq_chip(i+plgpio->irq_base, &plgpio_irqchip);
		set_irq_handler(i+plgpio->irq_base, handle_simple_irq);
		set_irq_flags(i+plgpio->irq_base, IRQF_VALID);
		set_irq_chip_data(i+plgpio->irq_base, plgpio);
	}
	set_irq_data(irq, plgpio);
	dev_info(&pdev->dev, "Initialization successful\n");

	return 0;

remove_gpiochip:
	if (gpiochip_remove(&plgpio->chip))
		dev_dbg(&pdev->dev, "unable to remove gpiochip\n");
iounmap:
	iounmap(plgpio->base);
kfree:
	kfree(plgpio);
release_region:
	release_mem_region(res->start, resource_size(res));
fail:
	dev_err(&pdev->dev, "probe fail: %d\n", ret);
	return ret;
}

static struct platform_driver plgpio_driver = {
	.probe		= plgpio_probe,
	.driver		= {
		.name	= "plgpio",
		.owner	= THIS_MODULE,
	},
};

static int __init plgpio_init(void)
{
	return platform_driver_register(&plgpio_driver);
}
subsys_initcall(plgpio_init);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@st.com>");
MODULE_DESCRIPTION("SPEAr PLGPIO driver");
MODULE_LICENSE("GPL");
