/*
 * arch/arm/mach-spear13xx/spear900.c
 *
 * SPEAr900 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/dw_pcie.h>
#include <mach/generic.h>
#include <mach/misc_regs.h>

/* pmx driver structure */
static struct pmx_driver pmx_driver;

/* Add spear900 specific devices here */

void __init spear900_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* pmx initialization */
	pmx_driver.mode = pmx_mode;
	pmx_driver.devs = pmx_devs;
	pmx_driver.devs_count = pmx_dev_count;

	ret = pmx_register(&pmx_driver);
	if (ret)
		pr_err("padmux: registeration failed. err no: %d\n", ret);
}

int spear900_pcie_clk_init(struct pcie_port *pp)
{
	/*
	 * Enable all CLK in CFG registers here only. Idealy only PCIE0
	 * should have been enabled. But Controler does not work
	 * properly if PCIE1 and PCIE2's CFG CLK is enabled in stages.
	 */
	writel(PCIE0_CFG_VAL | PCIE1_CFG_VAL | PCIE2_CFG_VAL, VA_PCIE_CFG);

	if (pp->clk == NULL) {
		pp->clk = clk_get_sys("dw_pcie.0", NULL);

		if (IS_ERR(pp->clk)) {
			pr_err("%s:couldn't get clk for pcie0\n", __func__);
			return -ENODEV;
		}
	}

	if (clk_enable(pp->clk)) {
		pr_err("%s:couldn't enable clk for pcie0\n", __func__);
		return -ENODEV;
	}

	return 0;
}

int spear900_pcie_clk_exit(struct pcie_port *pp)
{
	writel(0, VA_PCIE_CFG);
	if (pp->clk)
		clk_disable(pp->clk);

	return 0;
}
