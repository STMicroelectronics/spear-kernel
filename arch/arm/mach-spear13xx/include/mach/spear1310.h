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
#define SPEAR1310_GETH1_BASE		UL(0x6D000000)
#define SPEAR1310_GETH2_BASE		UL(0x6D100000)
#define SPEAR1310_GETH3_BASE		UL(0x6D200000)
#define SPEAR1310_GETH4_BASE		UL(0x6D300000)

/* RAS Area Control Register */
#define RAS_CTRL_REG1		(SPEAR1310_RAS_BASE + 0x4)
#define PHY_CLK_MASK		0xF
#define PHY_CLK_SHIFT		0

#endif /* __MACH_SPEAR1310_H */

#endif /* CONFIG_MACH_SPEAR1310 */
