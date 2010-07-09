/*
 * arch/arm/mach-spear13xx/spear1300.c
 *
 * SPEAr1300 machine source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/generic.h>
#include <mach/spear.h>

/* pmx driver structure */
struct pmx_driver pmx_driver;

/* Add spear1300 specific devices here */

void __init spear1300_init(void)
{
	int ret;

	/* call spear13xx family common init function */
	spear13xx_init();

	/* pmx initialization */
	pmx_driver.base = ioremap(SPEAR13XX_FUNC_ENB_BASE, SZ_4K);
	if (pmx_driver.base) {
		ret = pmx_register(&pmx_driver);
		if (ret)
			pr_err("padmux: registeration failed. err no: %d\n",
					ret);
		iounmap(pmx_driver.base);
	}
}
