/*
 * arch/arm/mach-spear13xx/include/mach/io.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_IO_H
#define __MACH_IO_H

#define IO_SPACE_LIMIT	0xFFFF

#ifdef CONFIG_SPEAR13XX_PCI
extern void __iomem *spear13xx_pcie_io_base(unsigned long addr);

static inline void __iomem *__io(unsigned long addr)
{
	return spear13xx_pcie_io_base(addr) + (addr & IO_SPACE_LIMIT);
}
#else
static inline void __iomem *__io(unsigned long addr)
{
	return (void __iomem *)addr;
}
#endif

#define __io(a)		__io(a)

#endif /* __MACH_IO_H */
