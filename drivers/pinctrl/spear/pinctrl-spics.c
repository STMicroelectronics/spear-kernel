/*
 * SPEAr platform SPI chipselect abstraction over gpiolib
 *
 * Copyright (C) 2012 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* maximum chipselects */
#define NUM_OF_GPIO	4

/*
 * Provision is available on some SPEAr SoCs to control ARM PL022 spi cs
 * through system registers. This register lies outside spi (pl022)
 * address space into system registers.
 *
 * It provides control for spi chip select lines so that any chipselect
 * (out of 4 possible chipselects in pl022) can be made low to select
 * the particular slave.
 */

/** struct spear_spics - represents spi chip select control
  * base:		base address
  * perip_cfg:		configuration register
  * sw_enable_bit: 	bit to enable s/w control over chipselects
  * cs_value_bit: 	bit to program high or low chipselect
  * cs_enable_mask: 	mask to select bits required to select chipselect
  * cs_enable_shift: 	bit pos of cs_enable_mask
  * chip:		gpio_chip abstraction
  */
struct spear_spics {
	void __iomem		*base;
	u32			perip_cfg;
	u32			sw_enable_bit;
	u32			cs_value_bit;
	u32			cs_enable_mask;
	u32			cs_enable_shift;
	unsigned long		use_count;
	struct gpio_chip	chip;
};

/* gpio framework specific routines */
static int spics_get_value(struct gpio_chip *chip, unsigned offset)
{
	return -ENXIO;
}

static void spics_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	struct spear_spics *spics =
		container_of(chip, struct spear_spics, chip);
	u32 tmp;

	tmp = readl_relaxed(spics->base + spics->perip_cfg);
	tmp &= ~(spics->cs_enable_mask << spics->cs_enable_shift);
	tmp |= (offset << spics->cs_enable_shift);

	tmp &= ~(0x1 << spics->cs_value_bit);
	tmp |= (value << spics->cs_value_bit);
	writel_relaxed(tmp, spics->base + spics->perip_cfg);
}

static int spics_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return -ENXIO;
}

static int spics_direction_output(struct gpio_chip *chip, unsigned offset,
		int value)
{
	spics_set_value(chip, offset, value);
	return 0;
}

static int spics_request(struct gpio_chip *chip, unsigned offset)
{
	struct spear_spics *spics =
		container_of(chip, struct spear_spics, chip);
	u32 tmp;

	if (offset >= NUM_OF_GPIO)
		return -EINVAL;

	tmp = readl_relaxed(spics->base + spics->perip_cfg);
	tmp |= 0x1 << spics->sw_enable_bit;
	tmp |= (0x1 << spics->cs_value_bit);
	writel_relaxed(tmp, spics->base + spics->perip_cfg);

	spics->use_count++;

	return 0;
}

static void spics_free(struct gpio_chip *chip, unsigned offset)
{
	struct spear_spics *spics =
		container_of(chip, struct spear_spics, chip);
	u32 tmp;

	if (--spics->use_count == 0) {
		tmp = readl_relaxed(spics->base + spics->perip_cfg);
		tmp &= ~(0x1 << spics->sw_enable_bit);
		writel_relaxed(tmp, spics->base + spics->perip_cfg);
	}
}

static int __devinit spics_probe_dt(struct platform_device *pdev,
		struct spear_spics *spics)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(np, "st-spics,peripcfg-reg",
			&spics->perip_cfg);
	ret |= of_property_read_u32(np, "st-spics,sw-enable-bit",
			&spics->sw_enable_bit);
	ret |= of_property_read_u32(np, "st-spics,cs-value-bit",
			&spics->cs_value_bit);
	ret |= of_property_read_u32(np, "st-spics,cs-enable-mask",
			&spics->cs_enable_mask);
	ret |= of_property_read_u32(np, "st-spics,cs-enable-shift",
			&spics->cs_enable_shift);
	return ret;
}

static int __devinit spics_gpio_probe(struct platform_device *pdev)
{
	struct spear_spics *spics;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "invalid IORESOURCE_MEM\n");
		return -EBUSY;
	}

	spics = devm_kzalloc(&pdev->dev, sizeof(*spics), GFP_KERNEL);
	if (!spics) {
		dev_err(&pdev->dev, "memory allocation fail\n");
		return -ENOMEM;
	}

	spics->base = devm_request_and_ioremap(&pdev->dev, res);
	if (!spics->base) {
		dev_err(&pdev->dev, "request and ioremap fail\n");
		return -ENOMEM;
	}

	ret = spics_probe_dt(pdev, spics);
	if (ret) {
		dev_err(&pdev->dev, "DT probe failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, spics);

	spics->chip.ngpio = NUM_OF_GPIO;
	spics->chip.base = -1;
	spics->chip.request = spics_request;
	spics->chip.free = spics_free;
	spics->chip.direction_input = spics_direction_input;
	spics->chip.direction_output = spics_direction_output;
	spics->chip.get = spics_get_value;
	spics->chip.set = spics_set_value;
	spics->chip.label = dev_name(&pdev->dev);
	spics->chip.dev = &pdev->dev;
	spics->chip.owner = THIS_MODULE;

	ret = gpiochip_add(&spics->chip);
	if (ret) {
		dev_err(&pdev->dev, "unable to add gpio chip\n");
		return ret;
	}

	dev_info(&pdev->dev, "spear spics registered\n");
	return 0;
}

static const struct of_device_id spics_gpio_of_match[] = {
	{ .compatible = "st,spear-spics-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, spics_gpio_of_match);

static struct platform_driver spics_gpio_driver = {
	.probe = spics_gpio_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "spear-spics-gpio",
		.of_match_table = of_match_ptr(spics_gpio_of_match),
	},
};

static int __init spics_gpio_init(void)
{
	return platform_driver_register(&spics_gpio_driver);
}
subsys_initcall(spics_gpio_init);

MODULE_AUTHOR("Shiraz Hashim <shiraz.hashim@st.com>");
MODULE_DESCRIPTION("ST Microlectronics SPEAr SPI Chip Select Abstraction");
MODULE_LICENSE("GPL");
