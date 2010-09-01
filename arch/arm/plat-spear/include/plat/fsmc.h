/*
 * arch/arm/plat-spear/include/plat/fsmc.h
 *
 * FSMC definitions for SPEAr platform
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

#include <linux/mtd/fsmc.h>

/* This function is used to set platform data field of pdev->dev */
static inline void fsmc_nand_set_plat_data(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int options, unsigned int width)
{
	struct fsmc_nand_platform_data *plat_data;
	plat_data = dev_get_platdata(&pdev->dev);

	if (partitions) {
		plat_data->partitions = partitions;
		plat_data->nr_partitions = nr_partitions;
	}

	plat_data->options = options;
	plat_data->width = width;
}

#endif /* __PLAT_FSMC_H */
