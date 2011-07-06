/*
 * arch/arm/mach-spear13xx/include/mach/spear_gpio.h
 *
 * SPEAr13xx Power Management GPIO management header
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef SPEAR_GPIO_H
#define SPEAR_GPIO_H
#include <mach/pm_common.h>


/* SPEAr Power Management GPIOs */
static struct gpio spear_gpio[] = {
	{ 
		.gpio	= 	POWEROFF_GPIO, 
		.flags	=	GPIOF_OUT_INIT_HIGH, 
		.label	=	"PowerOFF GPIO", 
	},
};

#endif /* SPEAR_GPIO_H */