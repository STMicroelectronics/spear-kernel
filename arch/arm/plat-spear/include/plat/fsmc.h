/*
 * arch/arm/plat-spear/include/plat/fsmc.h
 *
 * SPEAr platform nand interface header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_FSMC_H
#define __PLAT_FSMC_H

#include <linux/delay.h>
#include <linux/types.h>
#include <asm/param.h>

#define FSMC_MAX_NAND_BANKS	4

struct nand_bank_regs {
	u32 pc;
	u32 sts;
	u32 comm;
	u32 attrib;
	u32 ioata;
	u32 ecc1;
	u32 ecc2;
	u32 ecc3;
};

struct fsmc_regs {
	u8 reserved_1[0x40];
	struct nand_bank_regs bank_regs[FSMC_MAX_NAND_BANKS];
	u8 reserved_2[0xfe0 - 0xc0];
	u32 peripid0;			/* 0xfe0 */
	u32 peripid1;			/* 0xfe4 */
	u32 peripid2;			/* 0xfe8 */
	u32 peripid3;			/* 0xfec */
	u32 pcellid0;			/* 0xff0 */
	u32 pcellid1;			/* 0xff4 */
	u32 pcellid2;			/* 0xff8 */
	u32 pcellid3;			/* 0xffc */
};

#define FSMC_BUSY_WAIT_TIMEOUT	(1 * HZ)

/* pc register definitions */
#define FSMC_RESET		(1 << 0)
#define FSMC_WAITON		(1 << 1)
#define FSMC_ENABLE		(1 << 2)
#define FSMC_DEVTYPE_NAND	(1 << 3)
#define FSMC_DEVWID_8		(0 << 4)
#define FSMC_DEVWID_16		(1 << 4)
#define FSMC_ECCEN		(1 << 6)
#define FSMC_ECCPLEN_512	(0 << 7)
#define FSMC_ECCPLEN_256	(1 << 7)
#define FSMC_TCLR_1		(1 << 9)
#define FSMC_TAR_1		(1 << 13)

/* sts register definitions */
#define FSMC_CODE_RDY		(1 << 15)

/* comm register definitions */
#define FSMC_TSET_0		(0 << 0)
#define FSMC_TWAIT_6		(6 << 8)
#define FSMC_THOLD_4		(4 << 16)
#define FSMC_THIZ_1		(1 << 24)

/* peripid2 register definitions */
#define FSMC_REVISION_MSK	(0xf)
#define FSMC_REVISION_SHFT	(0x4)

#define FSMC_VER1		1
#define FSMC_VER2		2
#define FSMC_VER3		3
#define FSMC_VER4		4
#define FSMC_VER5		5
#define FSMC_VER6		6
#define FSMC_VER7		7
#define FSMC_VER8		8

static inline u32 get_fsmc_version(struct fsmc_regs *regs)
{
	return (readl(&regs->peripid2) >> FSMC_REVISION_SHFT) &
				FSMC_REVISION_MSK;
}

/*
 * There are 13 bytes of ecc for every 512 byte block in FSMC version 8
 * and it has to be read consecutively and immediately after the 512
 * byte data block for hardware to generate the error bit offsets
 * Managing the ecc bytes in the following way is easier. This way is
 * similar to oobfree structure maintained already in u-boot nand driver
 */
#define MAX_ECCPLACE_ENTRIES	32

struct fsmc_nand_eccplace {
	u8 offset;
	u8 length;
};

struct fsmc_eccplace {
	struct fsmc_nand_eccplace eccplace[MAX_ECCPLACE_ENTRIES];
};

#endif /* __PLAT_FSMC_H */
