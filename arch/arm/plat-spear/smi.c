/*
 * arch/arm/plat-spear/smi.c
 *
 * spear smi platform intialization
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <asm/mach-types.h>
#include <plat/smi.h>
#include <mach/hardware.h>

/*
 * physical base address of flash/bank mem map base associated with smi
 * depends on SoC
 */

#if defined(CONFIG_ARCH_SPEAR13XX)
#define FLASH_MEM_BASE	SPEAR13XX_SMI_MEM0_BASE

#elif defined(CONFIG_ARCH_SPEAR3XX)
#define FLASH_MEM_BASE	SPEAR3XX_ICM3_SMEM_BASE

#elif defined(CONFIG_ARCH_SPEAR6XX)
#define FLASH_MEM_BASE	SPEAR6XX_ICM3_SMEM_BASE

#endif

/* serial nor flash specific board data */
static struct mtd_partition m25p64_partition_info[] = {
	DEFINE_PARTS("Xloader", 0x00, 0x10000),
	DEFINE_PARTS("UBoot", 0x10000, 0x40000),
	DEFINE_PARTS("Kernel", 0x50000, 0x2C0000),
	DEFINE_PARTS("Root File System", 0x310000, 0x4F0000),
};

static struct spear_smi_flash_info nor_flash_info[] = {
	{
		.name = "m25p64",
		.fast_mode = 1,
		.mem_base = FLASH_MEM_BASE,
		.size = 8 * 1024 * 1024,
		.num_parts = ARRAY_SIZE(m25p64_partition_info),
		.parts = m25p64_partition_info,
	},
};

#ifdef CONFIG_CPU_SPEAR1340
static struct mtd_partition m25p40_partition_info[] = {
	DEFINE_PARTS("Root File System", 0x0000, 0x80000),
};

static struct spear_smi_flash_info spear1340_nor_flash_info[] = {
	{
		.name = "m25p64",
		.fast_mode = 1,
		.mem_base = FLASH_MEM_BASE,
		.size = 8 * 1024 * 1024,
		.num_parts = ARRAY_SIZE(m25p64_partition_info),
		.parts = m25p64_partition_info,
	}, {
		.name = "m25p40",
		.fast_mode = 0,
		.mem_base = SPEAR13XX_SMI_MEM1_BASE,
		.size = 512 * 1024,
		.num_parts = ARRAY_SIZE(m25p40_partition_info),
		.parts = m25p40_partition_info,
	},
};
#endif

/* smi specific board data */
static struct spear_smi_plat_data smi_plat_data = {
	.clk_rate = 50000000,	/* 50MHz */
	.num_flashes = ARRAY_SIZE(nor_flash_info),
	.board_flash_info = nor_flash_info,
};

void smi_init_board_info(struct platform_device *pdev)
{
#ifdef CONFIG_CPU_SPEAR1340
	if (cpu_is_spear1340()) {
		smi_plat_data.board_flash_info = spear1340_nor_flash_info;
		smi_plat_data.num_flashes =
			ARRAY_SIZE(spear1340_nor_flash_info);
	}
#endif
	smi_set_plat_data(pdev, &smi_plat_data);
}
