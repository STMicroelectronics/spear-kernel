/*
 * EMI macros for spear3xx platform
 *
 * Copyright (C) 2012 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SPEAR3XX_EMI_H
#define __MACH_SPEAR3XX_EMI_H

#define EMI_BANK_REG_SZ		0x18

#define TAP			(0x0)
#define TSDP			(0x4)
#define TDPW			(0x8)
#define TDPR			(0xC)
#define TDCS			(0x10)
#define CTRL			(0x14)

#define SPEAR310_TIMEOUT_REG	(0x90)
#define SPEAR310_ACK_REG	(0x94)
#define SPEAR310_IRQ_REG	(0x98)

#define SPEAR320_TIMEOUT_REG	(0x60)
#define SPEAR320_ACK_REG	(0x64)
#define SPEAR320_IRQ_REG	(0x68)

/* Control register definitions */
#define EMI_CNTL_WIDTH8		(0 << 0)
#define EMI_CNTL_WIDTH16	(1 << 0)
#define EMI_CNTL_WIDTH32	(2 << 0)
#define EMI_CNTL_ENBBYTEW	(1 << 2)
#define EMI_CNTL_ENBBYTER	(1 << 3)
#define EMI_CNTL_ENBBYTERW	(EMI_CNTL_ENBBYTER | EMI_CNTL_ENBBYTEW)

#define EMI_REG(base, bank, reg)		(base + \
						EMI_BANK_REG_SZ * (bank) + \
						reg)

#endif
