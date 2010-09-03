/*
 * arch/arm/mach-spear3xx/include/mach/emi.h
 *
 * EMI macros for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_EMI_H
#define __MACH_EMI_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>

#define EMI_FLASH_WIDTH8	1
#define EMI_FLASH_WIDTH16	2
#define EMI_FLASH_WIDTH32	4

#define EMI_REG_SIZE		0x100
#define EMI_BANK_REG_SIZE	0x18

#define TAP_REG			(0x0)
#define TSDP_REG		(0x4)
#define TDPW_REG		(0x8)
#define TDPR_REG		(0xC)
#define TDCS_REG		(0x10)
#define CTRL_REG		(0x14)

#if defined(CONFIG_MACH_SPEAR310)
#define TIMEOUT_REG		(0x90)
#define ACK_REG			(0x94)
#define IRQ_REG			(0x98)

#define EMI_MAX_BANKS		6

#elif defined(CONFIG_MACH_SPEAR320)
#define TIMEOUT_REG		(0x60)
#define ACK_REG			(0x64)
#define IRQ_REG			(0x68)

#define EMI_MAX_BANKS		4

#endif

/* Control register definitions */
#define EMI_CNTL_WIDTH8		(0 << 0)
#define EMI_CNTL_WIDTH16	(1 << 0)
#define EMI_CNTL_WIDTH32	(2 << 0)
#define EMI_CNTL_ENBBYTEW	(1 << 2)
#define EMI_CNTL_ENBBYTER	(1 << 3)
#define EMI_CNTL_ENBBYTERW	(EMI_CNTL_ENBBYTER | EMI_CNTL_ENBBYTEW)

extern int __init emi_init(struct platform_device *pdev, unsigned long base,
		u32 bank, u32 width);
extern void __init emi_init_board_info(struct platform_device *pdev,
		struct resource *resources, int res_num, struct mtd_partition
		*partitions, unsigned int nr_partitions, unsigned int width);
#endif
