/*
 * arch/arm/mach-spear13xx/include/mach/pm_common.h
 *
 * SPEAr13xx Power Management common header
 *
 * Copyright (C) 2011 ST Microelectronics
 * Vincenzo Frascino <vincenzo.frascino@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef PM_COMMON_H
#define PM_COMMON_H

#define SPEAR900_LCAD_POWEROFF_GPIO		9
#define SPEAR900_LCAD_PM_UP_GPIO			10
#define SPEAR900_LCAD_POWEROFF_GPIO_VALUE	0

#define SPEAR900_LCAD_BATTERY_SWC_GPIO		11
#define SPEAR900_LCAD_BATTERY_ST2_GPIO		12
#define SPEAR900_LCAD_BATTERY_ST1_GPIO		13

#define POWEROFF_GPIO		SPEAR900_LCAD_POWEROFF_GPIO
#define PM_UP_GPIO			SPEAR900_LCAD_PM_UP_GPIO
#define BATTERY_SWC_GPIO	SPEAR900_LCAD_BATTERY_SWC_GPIO
#define BATTERY_ST2_GPIO	SPEAR900_LCAD_BATTERY_ST2_GPIO
#define BATTERY_ST1_GPIO	SPEAR900_LCAD_BATTERY_ST1_GPIO

#define SPEAR_POWEROFF	0
#define SPEAR_RESTART		1

void spear13xx_pm_functions(int function);

#endif /* PM_COMMON_H */