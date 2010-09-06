/*
 * arch/arm/mach-spear13xx/include/mach/spear1310.h
 *
 * SPEAr1310 Machine specific definition
 *
 * Copyright (C) 2010 ST Microelectronics
 * Bhupesh Sharma <bhupesh.sharma@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifdef CONFIG_MACH_SPEAR1310

#ifndef __MACH_SPEAR1310_H
#define __MACH_SPEAR1310_H

#define SPEAR1310_CAN0_BASE		UL(0x6DA00000)
#define SPEAR1310_CAN1_BASE		UL(0x6DB00000)
#define SPEAR1310_RAS_BASE		UL(0x6C800000)

/* RAS Area Control Register */
#define SPEAR1310_RAS_CTRL_REG0		(SPEAR1310_RAS_BASE + 0x0)
#define SPEAR1310_RAS_CTRL_REG1		(SPEAR1310_RAS_BASE + 0x4)
#define SPEAR1310_PHY_CLK_MASK		0xF
#define SPEAR1310_PHY_CLK_SHIFT		0

#endif /* __MACH_SPEAR1310_H */

#endif /* CONFIG_MACH_SPEAR1310 */
