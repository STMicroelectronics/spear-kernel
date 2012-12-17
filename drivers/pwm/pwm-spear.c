/*
 * ST Microelectronics SPEAr Pulse Width Modulator driver
 *
 * Copyright (C) 2012 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

/* PWM registers and bits definitions */
#define PWMCR			0x00	/* Control Register */
#define PWMDCR			0x04	/* Duty Cycle Register */
#define PWMPCR			0x08	/* Period Register */
/* Following only available on 13xx SoCs */
#define PWMMCR			0x3C	/* Master Control Register */

#define PWM_ENABLE		0x1

#define MIN_PRESCALE		0x00
#define MAX_PRESCALE		0x3FFF
#define PRESCALE_SHIFT		2

#define MIN_DUTY		0x0001
#define MAX_DUTY		0xFFFF

#define MIN_PERIOD		0x0001
#define MAX_PERIOD		0xFFFF

#define NUM_PWM			4

/**
 * struct pwm: struct representing pwm ip
 *
 * mmio_base: base address of pwm
 * clk: pointer to clk structure of pwm ip
 * chip: linux pwm chip representation
 * dev: pointer to device structure of pwm
 */
struct spear_pwm_chip {
	void __iomem *mmio_base;
	struct clk *clk;
	struct pwm_chip	chip;
	struct device *dev;
};

static inline struct spear_pwm_chip *to_spear_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct spear_pwm_chip, chip);
}

static inline u32 spear_pwm_readl(struct spear_pwm_chip *chip, unsigned int num,
		unsigned long offset)
{
	return readl_relaxed(chip->mmio_base + (num << 4) + offset);
}

static inline void spear_pwm_writel(struct spear_pwm_chip *chip,
		unsigned int num, unsigned long offset, unsigned long val)
{
	writel_relaxed(val, chip->mmio_base + (num << 4) + offset);
}

/*
 * period_ns = 10^9 * (PRESCALE + 1) * PV / PWM_CLK_RATE
 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
 *
 * PV = (PWM_CLK_RATE * period_ns)/ (10^9 * (PRESCALE + 1))
 * DC = (PWM_CLK_RATE * duty_ns)/ (10^9 * (PRESCALE + 1))
 */
int spear_pwm_config(struct pwm_chip *pwm, struct pwm_device *pwmd, int duty_ns,
		int period_ns)
{
	struct spear_pwm_chip *pc = to_spear_pwm_chip(pwm);
	u64 val, div, clk_rate;
	unsigned long prescale = MIN_PRESCALE, pv, dc;
	int ret = -EINVAL;

	if (period_ns == 0 || duty_ns > period_ns)
		goto err;

	/*
	 * Find pv, dc and prescale to suit duty_ns and period_ns. This is done
	 * according to formulas provided above this routine.
	 */
	clk_rate = clk_get_rate(pc->clk);
	while (1) {
		div = 1000000000;
		div *= 1 + prescale;
		val = clk_rate * period_ns;
		pv = div64_u64(val, div);
		val = clk_rate * duty_ns;
		dc = div64_u64(val, div);

		/* if duty_ns and period_ns are not acheivable then return */
		if (!pv || !dc || pv < MIN_PERIOD || dc < MIN_DUTY)
			goto err;

		/*
		 * if pv and dc have crossed their upper limit, then increase
		 * prescale and recalculate pv and dc.
		 */
		if ((pv > MAX_PERIOD) || (dc > MAX_DUTY)) {
			prescale++;
			if (prescale > MAX_PRESCALE)
				goto err;
			continue;
		}
		break;
	}

	/*
	 * NOTE: the clock to PWM has to be enabled first before writing to the
	 * registers.
	 */
	ret = clk_prepare_enable(pc->clk);
	if (ret)
		goto err;

	spear_pwm_writel(pc, pwmd->hwpwm, PWMCR, prescale << PRESCALE_SHIFT);
	spear_pwm_writel(pc, pwmd->hwpwm, PWMDCR, dc);
	spear_pwm_writel(pc, pwmd->hwpwm, PWMPCR, pv);
	clk_disable_unprepare(pc->clk);

	return 0;
err:
	dev_err(pc->dev, "pwm config fail\n");
	return ret;
}

static int spear_pwm_enable(struct pwm_chip *pwm, struct pwm_device *pwmd)
{
	struct spear_pwm_chip *pc = to_spear_pwm_chip(pwm);
	int rc = 0;
	u32 val;

	rc = clk_prepare_enable(pc->clk);
	if (rc < 0)
		return rc;

	val = spear_pwm_readl(pc, pwmd->hwpwm, PWMCR);
	val |= PWM_ENABLE;
	spear_pwm_writel(pc, pwmd->hwpwm, PWMCR, val);

	return 0;
}

static void spear_pwm_disable(struct pwm_chip *pwm, struct pwm_device *pwmd)
{
	struct spear_pwm_chip *pc = to_spear_pwm_chip(pwm);
	u32 val;

	val = spear_pwm_readl(pc, pwmd->hwpwm, PWMCR);
	val &= ~PWM_ENABLE;
	spear_pwm_writel(pc, pwmd->hwpwm, PWMCR, val);

	clk_disable_unprepare(pc->clk);
}

static const struct pwm_ops spear_pwm_ops = {
	.config = spear_pwm_config,
	.enable = spear_pwm_enable,
	.disable = spear_pwm_disable,
	.owner = THIS_MODULE,
};

static int __devinit spear_pwm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct spear_pwm_chip *pc;
	struct resource *r;
	int ret;
	u32 val;

	pc = devm_kzalloc(&pdev->dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pc->dev = &pdev->dev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "no memory resources defined\n");
		return -ENODEV;
	}

	pc->mmio_base = devm_request_and_ioremap(&pdev->dev, r);
	if (!pc->mmio_base)
		return -EADDRNOTAVAIL;

	platform_set_drvdata(pdev, pc);

	pc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pc->clk))
		return PTR_ERR(pc->clk);

	pc->chip.dev = &pdev->dev;
	pc->chip.ops = &spear_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = NUM_PWM;

	ret = pwmchip_add(&pc->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		return ret;
	}

	if (np && of_device_is_compatible(np, "st,spear13xx-pwm")) {
		ret = clk_prepare_enable(pc->clk);
		if (ret < 0)
			return pwmchip_remove(&pc->chip);

		/* Following enables PWM device, channels would still be
		 * enabled individually through their control register
		 **/
		val = readl(pc->mmio_base + PWMMCR);
		val |= PWM_ENABLE;
		writel(val, pc->mmio_base + PWMMCR);

		clk_disable_unprepare(pc->clk);
	}

	return 0;
}

static int __devexit spear_pwm_remove(struct platform_device *pdev)
{
	struct spear_pwm_chip *pc = platform_get_drvdata(pdev);
	int i;

	if (WARN_ON(!pc))
		return -ENODEV;

	for (i = 0; i < NUM_PWM; i++) {
		struct pwm_device *pwmd = &pc->chip.pwms[i];

		if (!test_bit(PWMF_ENABLED, &pwmd->flags))
			if (clk_prepare_enable(pc->clk) < 0)
				continue;

		spear_pwm_writel(pc, i, PWMCR, 0);
		clk_disable_unprepare(pc->clk);
	}

	return pwmchip_remove(&pc->chip);
}

#ifdef CONFIG_OF
static struct of_device_id spear_pwm_of_match[] = {
	{ .compatible = "st,spear-pwm" },
	{ .compatible = "st,spear13xx-pwm" },
	{ }
};

MODULE_DEVICE_TABLE(of, spear_pwm_of_match);
#endif

static struct platform_driver spear_pwm_driver = {
	.driver = {
		.name = "spear-pwm",
		.of_match_table = of_match_ptr(spear_pwm_of_match),
	},
	.probe = spear_pwm_probe,
	.remove = __devexit_p(spear_pwm_remove),
};

module_platform_driver(spear_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shiraz Hashim <shiraz.hashim@st.com>");
MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.com>");
MODULE_ALIAS("platform:st-pwm");
