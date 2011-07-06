/*
 * arch/arm/mach-spear13xx/pm_common.c
 *
 * SPEAr13xx Power Management common features
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/types.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/pm_common.h>

static void spear900_lcad_poweroff(void) {
	
	/* Verify if SPEAR_POWEROFF_GPIO is correctly set */
	if (gpio_is_valid(SPEAR900_LCAD_POWEROFF_GPIO)) {
		/* PowerOFF the SPEAr 900 LCAD */
		gpio_set_value(SPEAR900_LCAD_POWEROFF_GPIO, SPEAR900_LCAD_POWEROFF_GPIO_VALUE);
	}else
		printk(KERN_WARNING "SPEAr Power Management: PM Invalid SPEAR900_LCAD_POWEROFF_GPIO");
}


/* Generic function used to manage all the spear13xx PM states */
void spear13xx_pm_functions(int function) {
	
	if (function == SPEAR_POWEROFF) {
		
		if(machine_is_spear900_lcad())
			spear900_lcad_poweroff();
		if(machine_is_spear900_evb())
			printk(KERN_WARNING "SPEAr Power Management: PM function not supported\n");
		if(machine_is_spear1300_evb())
			printk(KERN_WARNING "SPEAr Power Management: PM function not supported\n");
		
	} else
	if (function == SPEAR_RESTART)
		printk(KERN_WARNING "SPEAr Power Management: PM function not supported\n");
	else
		printk(KERN_WARNING "SPEAr Power Management: PM function not supported\n");
	
}