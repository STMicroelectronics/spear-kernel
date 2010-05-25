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
			.end = SPEAR6XX_CPU_GPIO_BASE + SZ_4K - 1,
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
			.end = SPEAR6XX_ICM3_GPIO_BASE + SZ_4K - 1,
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
			.end = SPEAR6XX_ICM2_GPIO_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_APPL_GPIO, NO_IRQ},
	}
};

/* adc device registeration */
static struct resource adc_resources[] = {
	{
		.start = SPEAR6XX_ICM2_ADC_BASE,
		.end = SPEAR6XX_ICM2_ADC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_APPL_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device adc_device = {
	.name = "adc",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(adc_resources),
	.resource = adc_resources,
};

/* dmac device registeration */
static struct resource dmac_resources[] = {
	{
		.start = SPEAR6XX_ICM3_DMA_BASE,
		.end = SPEAR6XX_ICM3_DMA_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_BASIC_DMA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device dmac_device = {
	.name = "pl080_dmac",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(dmac_resources),
	.resource = dmac_resources,
};

/* stmmac device registeration */
static struct plat_stmmacphy_data phy_private_data = {
	.bus_id = 0,
	.phy_addr = 1,
	.phy_mask = 0,
	.interface = PHY_INTERFACE_MODE_GMII,
};

static struct resource phy_resources = {
	.name = "phyirq",
	.start = -1,
	.end = -1,
	.flags = IORESOURCE_IRQ,
};

struct platform_device phy_device = {
	.name = "stmmacphy",
	.id = -1,
	.num_resources = 1,
	.resource = &phy_resources,
	.dev.platform_data = &phy_private_data,
};

static struct plat_stmmacenet_data eth_platform_data = {
	.has_gmac = 1,
	.enh_desc = 0,
	.tx_csum = 0,
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

/* i2c device registeration */
static struct resource i2c_resources[] = {
	{
		.start = SPEAR6XX_ICM1_I2C_BASE,
		.end = SPEAR6XX_ICM1_I2C_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device i2c_device = {
	.name = "i2c_designware",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c_resources),
	.resource = i2c_resources,
};

/* usb host device registeration */
static struct resource ehci0_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_USB_EHCI0_BASE,
		.end = SPEAR6XX_ICM4_USB_EHCI0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_EHCI_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ehci1_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_USB_EHCI1_BASE,
		.end = SPEAR6XX_ICM4_USB_EHCI1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_EHCI_1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci0_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_USB_OHCI0_BASE,
		.end = SPEAR6XX_ICM4_USB_OHCI0_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_OHCI_0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci1_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_USB_OHCI1_BASE,
		.end = SPEAR6XX_ICM4_USB_OHCI1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB_H_OHCI_1,
		.flags = IORESOURCE_IRQ,
	},
};

/* usbh0_id defaults to 0, being static variable */
static int usbh0_id;
static int usbh1_id = 1;
static u64 ehci0_dmamask = ~0;

struct platform_device ehci0_device = {
	.name = "spear-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ehci0_dmamask,
		.platform_data = &usbh0_id,
	},
	.num_resources = ARRAY_SIZE(ehci0_resources),
	.resource = ehci0_resources,
};

static u64 ehci1_dmamask = ~0;

struct platform_device ehci1_device = {
	.name = "spear-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ehci1_dmamask,
		.platform_data = &usbh1_id,
	},
	.num_resources = ARRAY_SIZE(ehci1_resources),
	.resource = ehci1_resources,
};

static u64 ohci0_dmamask = ~0;

struct platform_device ohci0_device = {
	.name = "spear-ohci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci0_dmamask,
		.platform_data = &usbh0_id,
	},
	.num_resources = ARRAY_SIZE(ohci0_resources),
	.resource = ohci0_resources,
};

static u64 ohci1_dmamask = ~0;

struct platform_device ohci1_device = {
	.name = "spear-ohci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &ohci1_dmamask,
		.platform_data = &usbh1_id,
	},
	.num_resources = ARRAY_SIZE(ohci1_resources),
	.resource = ohci1_resources,
};

/* jpeg device registeration */
static struct resource jpeg_resources[] = {
	{
		.start = SPEAR6XX_ICM1_JPEG_BASE,
		.end = SPEAR6XX_ICM1_JPEG_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_JPEG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device jpeg_device = {
	.name = "jpeg-designware",
	.id = -1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(jpeg_resources),
	.resource = jpeg_resources,
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

/* smi device registration */
static struct resource smi_resources[] = {
	{
		.start = SPEAR6XX_ICM3_SMI_CTRL_BASE,
		.end = SPEAR6XX_ICM3_SMI_CTRL_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_BASIC_SMI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device smi_device = {
	.name = "smi",
	.id = -1,
	.num_resources = ARRAY_SIZE(smi_resources),
	.resource = smi_resources,
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
