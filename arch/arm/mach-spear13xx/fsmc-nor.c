/*
 * arch/arm/mach-spear13xx/fsmc-nor.c
 *
 * FSMC (Flexible Static Memory Controller) interface for NOR
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipin Kumar<vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/io.h>
#include <plat/fsmc.h>

int __init fsmc_nor_init(struct platform_device *pdev, unsigned long base,
		u32 bank, u32 width)
{
	void __iomem *fsmc_nor_base;
	struct fsmc_regs *regs;
	struct clk *clk;
	int ret;
	u32 ctrl;

	if (bank > (FSMC_MAX_NOR_BANKS - 1))
		return -EINVAL;

	fsmc_nor_base = ioremap(base, FSMC_NOR_REG_SIZE);
	if (!fsmc_nor_base)
		return -ENOMEM;

	clk = clk_get(NULL, "fsmc");
	if (IS_ERR(clk)) {
		iounmap(fsmc_nor_base);
		return PTR_ERR(clk);
	}

	ret = clk_enable(clk);
	if (ret) {
		iounmap(fsmc_nor_base);
		return ret;
	}

	regs = (struct fsmc_regs *)fsmc_nor_base;

	ctrl = WAIT_ENB | WRT_ENABLE | WPROT | NOR_DEV | BANK_ENABLE;

	switch (width) {
	case FSMC_FLASH_WIDTH8:
		ctrl |= WIDTH_8;
		break;

	case FSMC_FLASH_WIDTH16:
		ctrl |= WIDTH_16;
		break;

	case FSMC_FLASH_WIDTH32:
		ctrl |= WIDTH_32;
		break;

	default:
		ctrl |= WIDTH_8;
		break;
	}

	writel(ctrl, &regs->nor_bank_regs[bank].ctrl);
	writel(0x0FFFFFFF, &regs->nor_bank_regs[bank].ctrl_tim);
	writel(ctrl | RSTPWRDWN, &regs->nor_bank_regs[bank].ctrl);

	iounmap(fsmc_nor_base);

	return 0;
}

void __init fsmc_init_board_info(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int width)
{
	fsmc_nor_set_plat_data(pdev, partitions, nr_partitions, width);
}
