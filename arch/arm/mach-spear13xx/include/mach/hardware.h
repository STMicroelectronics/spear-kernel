/*
 * arch/arm/mach-spear13xx/include/mach/hardware.h
 *
 * Hardware definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

#include <mach/spear.h>

/* Vitual to physical translation of statically mapped space */
#define IO_ADDRESS(x)		(x | 0xF0000000)

/* typesafe io address */
#define __io_address(n)		__io(IO_ADDRESS(n))

#if defined(CONFIG_PCI)
#define PCIBIOS_MIN_IO		0
#define PCIBIOS_MIN_MEM		0
#define pcibios_assign_all_busses()	0
#endif


#endif /* __MACH_HARDWARE_H */
