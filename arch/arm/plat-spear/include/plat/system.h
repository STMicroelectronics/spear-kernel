/*
 * arch/arm/plat-spear/include/plat/system.h
 *
 * SPEAr platform specific architecture functions
 *
 * Copyright (C) 2012 ST Microelectronics
 * Deepak Sikri<deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_SYSTEM_H
#define __PLAT_SYSTEM_H

#include <linux/io.h>
#include <asm/hardware/sp810.h>
#include <mach/spear.h>

static inline int arch_change_mode(int mode)
{
	return sysctl_change_mode((void __iomem *)VA_SPEAR_SYS_CTRL_BASE, mode);
}

#endif /* __PLAT_SYSTEM_H */
