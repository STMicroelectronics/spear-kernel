/*
 * arch/arm/mach-spear13xx/include/mach/vmalloc.h
 *
 * Defining Vmalloc area for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_VMALLOC_H
#define __MACH_VMALLOC_H

#include <mach/memory.h>

#define VMALLOC_SIZE		(0x30000000)
#define VMALLOC_END		(PAGE_OFFSET + VMALLOC_SIZE)

#endif /* __MACH_VMALLOC_H */
