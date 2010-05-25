/*
 * arch/arm/plat-spear/include/plat/nand.h
 *
 * NAND macros for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Vipin Kumar <vipin.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_NAND_H
#define __PLAT_NAND_H

#include <linux/mtd/partitions.h>

#define SPEAR_NAND_BW8		1
#define SPEAR_NAND_BW16		2

#if defined(CONFIG_MACH_SPEAR310)
#define PLAT_NAND_CLE		(1 << 17)
#define PLAT_NAND_ALE		(1 << 16)
#else
#define PLAT_NAND_CLE		(1 << 16)
#define PLAT_NAND_ALE		(1 << 17)
#endif

struct nand_platform_data {
	/*
	 * Board specific information
	 * Set from arch/arm/mach-spear<mach>/spear<mach>_evb.c
	 */

	/*
	 * Use the default partition table present in the NAND driver if
	 * partitions is set to NULL.
	 */
	struct mtd_partition	*partitions;
	unsigned int		nr_partitions;
	unsigned int		options;
	unsigned int		width;

	/*
	 * Machine specific information
	 * Set from arch/arm/mach-spear<mach>/spear<mach>.c
	 */

	unsigned int		bank;
	/*
	 * Set to NULL if bank selection is not supported by machine
	 * architecture
	 * -> eg. when controller supports only one bank
	 */
	void			(*select_bank)(u32 bank, u32 busw);
};

/* This function is used to set platform data field of pdev->dev */
static inline void nand_set_plat_data(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int options, unsigned int width)
{
	struct nand_platform_data *nand_plat_data;
	nand_plat_data = dev_get_platdata(&pdev->dev);

	if (partitions) {
		nand_plat_data->partitions = partitions;
		nand_plat_data->nr_partitions = nr_partitions;
	}

	nand_plat_data->options = options;
	nand_plat_data->width = width;
}

#endif /* __PLAT_NAND_H */
