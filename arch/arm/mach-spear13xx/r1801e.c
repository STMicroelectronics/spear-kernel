/*
 * arch/arm/mach-spear13xx/r1801e.c
 *
 * ZT SOM board source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mtd/fsmc.h>
#include <linux/mtd/nand.h>
#include <linux/phy.h>
#include <linux/stmmac.h>
#include <asm/mach-types.h>
#include <plat/fsmc.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>
#include <mach/spear_pcie.h>

#define PARTITION(n, off, sz)	{.name = n, .offset = off, .size = sz}

#ifdef CONFIG_USE_PLGPIO
/* Pad muxing for PLGPIO_11 "System Alive" LED indicator */
static struct pmx_mux_reg pmx_plgpio_10_11_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT0_TMR1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_10_11_modes[] = {
	{
		.mux_regs = pmx_plgpio_10_11_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_10_11_mux),
	},
};

struct pmx_dev pmx_plgpio_10_11 = {
	.name = "plgpio_10_11",
	.modes = pmx_plgpio_10_11_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_10_11_modes),
};

/* Pad muxing for PLGPIO_9 "PWR_OFF" output */
static struct pmx_mux_reg pmx_plgpio_8_9_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT0_TMR2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_8_9_modes[] = {
	{
		.mux_regs = pmx_plgpio_8_9_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_8_9_mux),
	},
};

static struct pmx_dev pmx_plgpio_8_9 = {
	.name = "plgpio_8_9",
	.modes = pmx_plgpio_8_9_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_8_9_modes),
};

/* Pad muxing for PLGPIO_7 "SHTDWN_RQST" input */
static struct pmx_mux_reg pmx_plgpio_6_7_mux[] = {
	{
		.address = PAD_FUNCTION_EN_2,
		.mask = SPEAR13XX_PMX_GPT1_TMR1_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_plgpio_6_7_modes[] = {
	{
		.mux_regs = pmx_plgpio_6_7_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_plgpio_6_7_mux),
	},
};

static struct pmx_dev pmx_plgpio_6_7 = {
	.name = "plgpio_6_7",
	.modes = pmx_plgpio_6_7_modes,
	.mode_count = ARRAY_SIZE(pmx_plgpio_6_7_modes),
};
#endif /* CONFIG_USE_PLGPIO */

/* NAND partition table */
static struct mtd_partition partition_info[] = {
	PARTITION("X-loader", 0, 4 * 0x20000),
	PARTITION("U-Boot", 0x80000, 3 * 0x20000),
	PARTITION("Kernel", 0xe0000, 24 * 0x20000),
	PARTITION("Root File System", 0x3e0000, 8161 * 0x20000),
};

static struct plat_stmmacphy_data phy0_private_data;

static int phy_reset(void *mii)
{
	u16 phyid1, phyid2;
	struct mii_bus *bus;

	bus = (struct mii_bus *)mii;
	phyid1 = bus->read(bus, phy0_private_data.phy_addr, 0x2);
	phyid2 = bus->read(bus, phy0_private_data.phy_addr, 0x3);

	if (phyid1 == 0x0022 && (phyid2 & 0xfff0) == 0x1610) {
		/*
		 * Note: Adjust timing within Micrel
		 * PHY, otherwise link doesn't work
		 */
		bus->write(bus, phy0_private_data.phy_addr, 0xb, 0x8104);
		bus->write(bus, phy0_private_data.phy_addr, 0xc, 0x7700);
	}

	return 0;
}

/* Ethernet phy-0 device registeration */
static struct plat_stmmacphy_data phy0_private_data = {
	.bus_id = 0,
	.phy_addr = 0,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_GMII,
	.phy_clk_cfg = spear13xx_eth_phy_clk_cfg,
	.phy_reset = phy_reset,
};

static struct resource phy0_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device r1801e_phy0_device = {
	.name		= "stmmacphy",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &phy0_resources,
	.dev.platform_data = &phy0_private_data,
};

/* padmux devices to enable */
static struct pmx_dev *pmx_devs[] = {
	/* spear13xx specific devices */
	&spear13xx_pmx_i2c,
	&spear13xx_pmx_egpio_grp,
	&spear13xx_pmx_gmii,
	&spear13xx_pmx_mcif,
	&spear13xx_pmx_uart0,
	&spear13xx_pmx_nand_16bit,

	/* spear1310 reva specific devices */
	&spear1310_reva_pmx_i2c1,
	&spear1310_reva_pmx_uart_1,
	&spear1310_reva_pmx_uart_2,
	&spear1310_reva_pmx_uart_3_4_5,
#ifdef CONFIG_USE_PLGPIO
	&pmx_plgpio_10_11,
	&pmx_plgpio_8_9,
	&pmx_plgpio_6_7,
#endif
};

static struct amba_device *amba_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_gpio_device[0],
	&spear13xx_gpio_device[1],
	&spear13xx_ssp_device,
	&spear13xx_uart_device,

	/* spear1310 reva specific devices */
	&spear1310_reva_uart1_device,
	&spear1310_reva_uart2_device,
	&spear1310_reva_uart3_device,
	&spear1310_reva_uart4_device,
	&spear1310_reva_uart5_device,
};

static struct platform_device *plat_devs[] __initdata = {
	/* spear13xx specific devices */
	&spear13xx_adc_device,
	&spear13xx_cpufreq_device,
	&spear13xx_dmac_device[0],
	&spear13xx_dmac_device[1],
	&spear13xx_ehci0_device,
	&spear13xx_ehci1_device,
	&spear13xx_eth_device,
	&spear13xx_i2c_device,
	&spear13xx_jpeg_device,
	&spear13xx_ohci0_device,
	&spear13xx_ohci1_device,
	&spear13xx_pcie_host0_device,
	&spear13xx_pcie_host1_device,
	&spear13xx_pcie_host2_device,
	&spear13xx_rtc_device,
	&spear13xx_sdhci_device,
	&spear13xx_wdt_device,
	&spear13xx_nand_device,

	/* spear1310 reva specific devices */
	&spear1310_reva_i2c1_device,
	&spear1310_reva_plgpio_device,
	&r1801e_phy0_device,
};

#ifdef CONFIG_SPEAR_PCIE_REV341
 /* This function is needed for board specific PCIe initilization */
static void __init r1801e_pcie_board_init(void)
{
	void *plat_data;

	plat_data = dev_get_platdata(&spear13xx_pcie_host0_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
	((struct pcie_port_info *)plat_data)->is_gen1 = PCIE_IS_GEN1;

	plat_data = dev_get_platdata(&spear13xx_pcie_host1_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
	((struct pcie_port_info *)plat_data)->is_gen1 = PCIE_IS_GEN1;

	plat_data = dev_get_platdata(&spear13xx_pcie_host2_device.dev);
	PCIE_PORT_INIT((struct pcie_port_info *)plat_data, SPEAR_PCIE_REV_3_41);
	((struct pcie_port_info *)plat_data)->is_gen1 = PCIE_IS_GEN1;
}
#endif

static void __init ras_fsmc_config(u32 mode, u32 width)
{
	u32 val, *address;

	address = ioremap(SPEAR1310_REVA_RAS_CTRL_REG0, SZ_16);

	val = readl(address);
	val &= ~(RAS_FSMC_MODE_MASK | RAS_FSMC_WIDTH_MASK);
	val |= mode;
	val |= width;
	val |= RAS_FSMC_CS_SPLIT;

	writel(val, address);

	iounmap(address);
}

static void __init r1801e_init(void)
{
	int i;

	/*
	 * SPEAr1310 reva FSMC cannot used as NOR and NAND at the same time
	 * For the moment, disable NAND and use NOR only
	 * If NAND is needed, enable the following code and disable all
	 * code for NOR. Also enable nand in padmux configuration to
	 * use it.
	 */
	/* set nand device's plat data */
	fsmc_nand_set_plat_data(&spear13xx_nand_device, partition_info,
			ARRAY_SIZE(partition_info), NAND_SKIP_BBTSCAN,
			FSMC_NAND_BW16, NULL, 1);
	nand_mach_init(FSMC_NAND_BW16);

	/* call spear1310 reva machine init function */
	spear1310_reva_init(NULL, pmx_devs, ARRAY_SIZE(pmx_devs));

#ifdef CONFIG_SPEAR_PCIE_REV341
	r1801e_pcie_board_init();
#endif

	/* Add Platform Devices */
	platform_add_devices(plat_devs, ARRAY_SIZE(plat_devs));

	/* Add Amba Devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++)
		amba_device_register(amba_devs[i], &iomem_resource);

	/* Initialize fsmc regiters */
	ras_fsmc_config(RAS_FSMC_MODE_NAND, RAS_FSMC_WIDTH_16);

}

MACHINE_START(R1801E, "ST-SPEAR1310-REVA-R1801e")
	.boot_params	=	0x00000100,
	.map_io		=	spear1310_reva_map_io,
	.init_irq	=	spear13xx_init_irq,
	.timer		=	&spear13xx_timer,
	.init_machine	=	r1801e_init,
MACHINE_END
