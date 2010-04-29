/*
 * arch/arm/mach-spear6xx/spear6xx.c
 *
 * SPEAr6XX machines common source file
 *
 * Copyright (C) 2009 ST Microelectronics
 * Rajeev Kumar<rajeev-dlh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/amba/pl061.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <asm/hardware/vic.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <mach/irqs.h>
#include <mach/generic.h>
#include <mach/spear.h>

/* Add spear6xx machines common devices here */

/* CLCD device registration */
struct amba_device clcd_device = {
	.dev = {
		.init_name = "clcd",
		.coherent_dma_mask = ~0,
		.platform_data = &clcd_plat_data,
	},
	.res = {
		.start = SPEAR6XX_ICM3_CLCD_BASE,
		.end = SPEAR6XX_ICM3_CLCD_BASE + SPEAR6XX_ICM3_CLCD_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.irq = {IRQ_BASIC_CLCD, NO_IRQ},
};

static struct plat_stmmacphy_data phy_private_data = {
	.bus_id = 0,
	.phy_addr = -1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_MII,
};

static struct resource phy_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy_device = {
	.name = "stmmacphy",
	.id = 0,
	.num_resources = 1,
	.resource = &phy_resources,
	.dev.platform_data = &phy_private_data,
};

static struct plat_stmmacenet_data eth_platform_data = {
	.has_gmac = 0,
};

static struct resource eth_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_GMAC_BASE,
		.end = SPEAR6XX_ICM4_GMAC_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GMAC_2,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GMAC_1,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 eth_dma_mask = ~(u32) 0;

struct platform_device eth_device = {
	.name = "stmmaceth",
	.id = -1,
	.num_resources = ARRAY_SIZE(eth_resources),
	.resource = eth_resources,
	.dev = {
		.platform_data = &eth_platform_data,
		.dma_mask = &eth_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* uart device registeration */
struct amba_device uart_device[] = {
	{
		.dev = {
			.init_name = "uart0",
		},
		.res = {
			.start = SPEAR6XX_ICM1_UART0_BASE,
			.end = SPEAR6XX_ICM1_UART0_BASE +
				SPEAR6XX_ICM1_UART0_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_UART_0, NO_IRQ},
	}, {
		.dev = {
			.init_name = "uart1",
		},
		.res = {
			.start = SPEAR6XX_ICM1_UART1_BASE,
			.end = SPEAR6XX_ICM1_UART1_BASE +
				SPEAR6XX_ICM1_UART1_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_UART_1, NO_IRQ},
	}
};

/* gpio device registeration */
static struct pl061_platform_data gpio_plat_data[] = {
	{
		.gpio_base	= 0,
		.irq_base	= SPEAR_GPIO0_INT_BASE,
	}, {
		.gpio_base	= 8,
		.irq_base	= SPEAR_GPIO1_INT_BASE,
	}, {
		.gpio_base	= 16,
		.irq_base	= SPEAR_GPIO2_INT_BASE,
	},
};

struct amba_device gpio_device[] = {
	{
		.dev = {
			.init_name = "gpio0",
			.platform_data = &gpio_plat_data[0],
		},
		.res = {
			.start = SPEAR6XX_CPU_GPIO_BASE,
			.end = SPEAR6XX_CPU_GPIO_BASE +
				SPEAR6XX_CPU_GPIO_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_LOCAL_GPIO, NO_IRQ},
	}, {
		.dev = {
			.init_name = "gpio1",
			.platform_data = &gpio_plat_data[1],
		},
		.res = {
			.start = SPEAR6XX_ICM3_GPIO_BASE,
			.end = SPEAR6XX_ICM3_GPIO_BASE +
				SPEAR6XX_ICM3_GPIO_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_BASIC_GPIO, NO_IRQ},
	}, {
		.dev = {
			.init_name = "gpio2",
			.platform_data = &gpio_plat_data[2],
		},
		.res = {
			.start = SPEAR6XX_ICM2_GPIO_BASE,
			.end = SPEAR6XX_ICM2_GPIO_BASE +
				SPEAR6XX_ICM2_GPIO_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_APPL_GPIO, NO_IRQ},
	}
};

/* rtc device registration */
static struct resource rtc_resources[] = {
	{
		.start = SPEAR6XX_ICM3_RTC_BASE,
		.end = SPEAR6XX_ICM3_RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_BASIC_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

/* This will add devices, and do machine specific tasks */
void __init spear6xx_init(void)
{
	/* nothing to do for now */
}

/* This will initialize vic */
void __init spear6xx_init_irq(void)
{
	vic_init((void __iomem *)VA_SPEAR6XX_CPU_VIC_PRI_BASE, 0, ~0, 0);
	vic_init((void __iomem *)VA_SPEAR6XX_CPU_VIC_SEC_BASE, 32, ~0, 0);
}

/* Following will create static virtual/physical mappings */
static struct map_desc spear6xx_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR6XX_ICM1_UART0_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM1_UART0_BASE),
		.length		= SPEAR6XX_ICM1_UART0_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_CPU_VIC_PRI_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_CPU_VIC_PRI_BASE),
		.length		= SPEAR6XX_CPU_VIC_PRI_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_CPU_VIC_SEC_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_CPU_VIC_SEC_BASE),
		.length		= SPEAR6XX_CPU_VIC_SEC_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_ICM3_SYS_CTRL_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM3_SYS_CTRL_BASE),
		.length		= SPEAR6XX_ICM3_MISC_REG_BASE,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_ICM3_MISC_REG_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM3_MISC_REG_BASE),
		.length		= SPEAR6XX_ICM3_MISC_REG_SIZE,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear6xx_map_io(void)
{
	iotable_init(spear6xx_io_desc, ARRAY_SIZE(spear6xx_io_desc));

	/* This will initialize clock framework */
	clk_init();
}

static void __init spear6xx_timer_init(void)
{
	spear_setup_timer();
}

struct sys_timer spear6xx_timer = {
	.init = spear6xx_timer_init,
};
