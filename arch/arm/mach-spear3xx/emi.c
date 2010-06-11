/*
 * arch/arm/mach-spear3xx/emi.c
 *
 * EMI (External Memory Interface) file
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
#include <linux/io.h>
#include <mach/emi.h>

int __init emi_init(struct platform_device *pdev, unsigned long base,
		u32 bank, u32 width)
{
	void __iomem *emi_reg_base;
	struct clk *clk;
	int ret;

	if (bank > (EMI_MAX_BANKS - 1))
		return -EINVAL;

	emi_reg_base = ioremap(base, EMI_REG_SIZE);
	if (!emi_reg_base)
		return -ENOMEM;

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		iounmap(emi_reg_base);
		return PTR_ERR(clk);
	}

	ret = clk_enable(clk);
	if (ret) {
		iounmap(emi_reg_base);
		return ret;
	}

	/* set the timing */
	writel(0x10, emi_reg_base + (EMI_BANK_REG_SIZE * bank) + TAP_REG);
	writel(0x05, emi_reg_base + (EMI_BANK_REG_SIZE * bank) + TSDP_REG);
	writel(0x0a, emi_reg_base + (EMI_BANK_REG_SIZE * bank) + TDPW_REG);
	writel(0x0a, emi_reg_base + (EMI_BANK_REG_SIZE * bank) + TDPR_REG);
	writel(0x05, emi_reg_base + (EMI_BANK_REG_SIZE * bank) + TDCS_REG);

	switch (width) {
	case EMI_FLASH_WIDTH8:
		width = EMI_CNTL_WIDTH8;
		break;

	case EMI_FLASH_WIDTH16:
		width = EMI_CNTL_WIDTH16;
		break;

	case EMI_FLASH_WIDTH32:
		width = EMI_CNTL_WIDTH32;
		break;
	default:
		width = EMI_CNTL_WIDTH8;
		break;
	}
	/* set the data width */
	writel(width | EMI_CNTL_ENBBYTERW,
		emi_reg_base + (EMI_BANK_REG_SIZE * bank) + CTRL_REG);

	/* disable all the acks */
	writel(0x3f << bank, emi_reg_base + ACK_REG);

	iounmap(emi_reg_base);

	return 0;
}

void __init emi_init_board_info(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int width)
{
	emi_init_plat_data(pdev, partitions, nr_partitions, width);
}
