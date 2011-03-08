/*
 * arch/arm/plat-spear/include/plat/hardware.h
 *
 * Hardware definitions for SPEAr
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_HARDWARE_H
#define __PLAT_HARDWARE_H

#include <linux/types.h>
#include <asm/mach-types.h>

#ifndef __ASSEMBLY__
#define IOMEM(x)	((void __iomem __force *)(x))
#else
#define IOMEM(x)	(x)
#endif

#ifndef __ASSEMBLY__
static inline bool cpu_is_spear300(void)
{
	return machine_is_spear300_evb();
}

static inline bool cpu_is_spear310(void)
{
	return machine_is_spear310_evb();
}

static inline bool cpu_is_spear320(void)
{
	return machine_is_spear320_evb();
}

static inline bool cpu_is_spear600(void)
{
	return machine_is_spear600_evb();
}

static inline bool cpu_is_spear1300(void)
{
	return machine_is_spear1300_evb();
}

static inline bool cpu_is_spear1310(void)
{
	return machine_is_spear1310_evb();
}

static inline bool cpu_is_spear1340(void)
{
	return machine_is_spear1340_evb();
}

static inline bool cpu_is_spear900(void)
{
	return machine_is_spear900_evb();
}
#endif /* __ASSEMBLY__ */

#endif /* __PLAT_HARDWARE_H */
