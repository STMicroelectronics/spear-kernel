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

#include <linux/mtd/physmap.h>
#include <linux/mtd/fsmc.h>
#include <plat/hardware.h>

/*
 * The placement of the Command Latch Enable (CLE) and
 * Address Latch Enable (ALE) is twised around in the
 * SPEAR310 implementation.
 */
#if defined(CONFIG_CPU_SPEAR310)
#define SPEAR310_PLAT_NAND_CLE	(1 << 17)
#define SPEAR310_PLAT_NAND_ALE	(1 << 16)
#endif

#define PLAT_NAND_CLE		(1 << 16)
#define PLAT_NAND_ALE		(1 << 17)

/* This function is used to set platform data field of pdev->dev */
static inline void fsmc_nand_set_plat_data(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int options, unsigned int width,
		struct fsmc_nand_timings *timings, unsigned int max_banks)
{
	struct fsmc_nand_platform_data *plat_data;
	plat_data = dev_get_platdata(&pdev->dev);

	if (partitions) {
		plat_data->partitions = partitions;
		plat_data->nr_partitions = nr_partitions;
	}

	if (cpu_is_spear310()) {
#ifdef CONFIG_CPU_SPEAR310
		plat_data->ale_off = SPEAR310_PLAT_NAND_ALE;
		plat_data->cle_off = SPEAR310_PLAT_NAND_CLE;
#endif
	} else {
		plat_data->ale_off = PLAT_NAND_ALE;
		plat_data->cle_off = PLAT_NAND_CLE;
	}

	plat_data->options = options;
	plat_data->width = width;
	plat_data->nand_timings = timings;
	plat_data->max_banks = max_banks;
}

static inline void fsmc_nor_set_plat_data(struct platform_device *pdev,
		struct mtd_partition *partitions, unsigned int nr_partitions,
		unsigned int width)
{
	struct physmap_flash_data *plat_data;
	plat_data = dev_get_platdata(&pdev->dev);

	if (partitions) {
		plat_data->parts = partitions;
		plat_data->nr_parts = nr_partitions;
	}

	plat_data->width = width;
}
#endif /* __PLAT_FSMC_H */
