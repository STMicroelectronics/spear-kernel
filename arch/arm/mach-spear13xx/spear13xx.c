/*
 * arch/arm/mach-spear13xx/spear13xx.c
 *
 * SPEAr13XX machines common source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/amba/pl061.h>
#include <linux/dw_dmac.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <asm/hardware/gic.h>
#include <asm/irq.h>
#include <asm/localtimer.h>
#include <asm/mach/arch.h>
#include <asm/smp_twd.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

/* Add spear13xx machines common devices here */
/* gpio device registeration */
static struct pl061_platform_data gpio_plat_data[] = {
	{
		.gpio_base	= 0,
		.irq_base	= SPEAR_GPIO0_INT_BASE,
	}, {
		.gpio_base	= 8,
		.irq_base	= SPEAR_GPIO1_INT_BASE,
	},
};

struct amba_device gpio_device[] = {
	{
		.dev = {
			.init_name = "gpio0",
			.platform_data = &gpio_plat_data[0],
		},
		.res = {
			.start = SPEAR13XX_GPIO0_BASE,
			.end = SPEAR13XX_GPIO0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_GPIO0, NO_IRQ},
	}, {
		.dev = {
			.init_name = "gpio1",
			.platform_data = &gpio_plat_data[1],
		},
		.res = {
			.start = SPEAR13XX_GPIO1_BASE,
			.end = SPEAR13XX_GPIO1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_GPIO1, NO_IRQ},
	}
};

/* uart device registeration */
struct amba_device uart_device = {
	.dev = {
		.init_name = "uart",
	},
	.res = {
		.start = SPEAR13XX_UART_BASE,
		.end = SPEAR13XX_UART_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_UART, NO_IRQ},
};

/* adc device registeration */
static struct resource adc_resources[] = {
	{
		.start = SPEAR13XX_ADC_BASE,
		.end = SPEAR13XX_ADC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_ADC,
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
struct dw_dma_platform_data dmac_plat_data = {8, };
static struct resource dmac_resources[][2] = {
	[0] = {
		{
			.start = SPEAR13XX_DMAC0_BASE,
			.end = SPEAR13XX_DMAC0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = IRQ_DMAC0_COMBINED,
			.flags = IORESOURCE_IRQ,
		},
	},
	[1] = {
		{
			.start = SPEAR13XX_DMAC1_BASE,
			.end = SPEAR13XX_DMAC1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = IRQ_DMAC1_COMBINED,
			.flags = IORESOURCE_IRQ,
		},
	},
};

struct platform_device dmac_device[] = {
	[0] = {
		.name = "dw_dmac",
		.id = 0,
		.dev = {
			.coherent_dma_mask = ~0,
			.platform_data = &dmac_plat_data,
		},
		.num_resources = ARRAY_SIZE(dmac_resources[0]),
		.resource = dmac_resources[0],
	},
	[1] = {
		.name = "dw_dmac",
		.id = 1,
		.dev = {
			.coherent_dma_mask = ~0,
			.platform_data = &dmac_plat_data,
		},
		.num_resources = ARRAY_SIZE(dmac_resources[1]),
		.resource = dmac_resources[1],
	},
};

/* i2c device registeration */
static struct resource i2c_resources[] = {
	{
		.start = SPEAR13XX_I2C_BASE,
		.end = SPEAR13XX_I2C_BASE + SZ_4K - 1,
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

/* Ethernet device registeration */
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
	.name		= "stmmacphy",
	.id		= -1,
	.num_resources	= 1,
	.resource	= &phy_resources,
	.dev.platform_data = &phy_private_data,
};

static struct plat_stmmacenet_data ether_platform_data = {
	.has_gmac = 1,
	.enh_desc = 1,
	.tx_csum = 1,
};

static struct resource eth_resources[] = {
	[0] = {
		.start = SPEAR13XX_GETH_BASE,
		.end = SPEAR13XX_GETH_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GMAC_1,
		.flags = IORESOURCE_IRQ,
		.name = "macirq",
	},
	[2] = {
		.start = IRQ_GMAC_2,
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
		.platform_data = &ether_platform_data,
		.dma_mask = &eth_dma_mask,
		.coherent_dma_mask = ~0,
	},
};

/* usb host device registeration */
static struct resource ehci0_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC0_EHCI_BASE,
		.end = SPEAR13XX_UHC0_EHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBH_EHCI0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ehci1_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC1_EHCI_BASE,
		.end = SPEAR13XX_UHC1_EHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBH_EHCI1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ohci0_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC0_OHCI_BASE,
		.end = SPEAR13XX_UHC0_OHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBH_OHCI0,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource ohci1_resources[] = {
	[0] = {
		.start = SPEAR13XX_UHC1_OHCI_BASE,
		.end = SPEAR13XX_UHC1_OHCI_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBH_OHCI1,
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
		.start = SPEAR13XX_JPEG_BASE,
		.end = SPEAR13XX_JPEG_BASE + SZ_8K - 1,
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

/* keyboard device registration */
static struct resource kbd_resources[] = {
	{
		.start = SPEAR13XX_KBD_BASE,
		.end = SPEAR13XX_KBD_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_KBD,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device kbd_device = {
	.name = "keyboard",
	.id = -1,
	.num_resources = ARRAY_SIZE(kbd_resources),
	.resource = kbd_resources,
};

/* rtc device registration */
static struct resource rtc_resources[] = {
	{
		.start = SPEAR13XX_RTC_BASE,
		.end = SPEAR13XX_RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

static void dmac_setup(void)
{
	/*
	 * This function does static initialization of few misc regs for dmac
	 * operations.
	 */
	/* setting Peripheral flow controller for jpeg */
	writel(1 << DMA_REQ_FROM_JPEG, DMAC_FLOW_SEL);
}

/* Do spear13xx familiy common initialization part here */
void __init spear13xx_init(void)
{
	dmac_setup();
}

/* This will initialize vic */
void __init spear13xx_init_irq(void)
{
	gic_dist_init(0, __io_address(SPEAR13XX_GIC_DIST_BASE), 29);
	gic_cpu_init(0, __io_address(SPEAR13XX_GIC_CPU_BASE));
}

/* Following will create static virtual/physical mappings */
struct map_desc spear13xx_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SPEAR13XX_UART_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_UART_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_A9SM_PERIP_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_A9SM_PERIP_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_MISC_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_MISC_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_SYSRAM0_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_SYSRAM0_BASE),
		.length		= SZ_32K,
		.type		= MT_DEVICE
	},
};

/* This will create static memory mapping for selected devices */
void __init spear13xx_map_io(void)
{
	iotable_init(spear13xx_io_desc, ARRAY_SIZE(spear13xx_io_desc));

	/* This will initialize clock framework */
	clk_init();
}

static void __init spear13xx_timer_init(void)
{
#ifdef CONFIG_LOCAL_TIMERS
	/* Setup the local timer base */
	twd_base = __io_address(SPEAR13XX_LOCAL_TMR_BASE);
#endif
	spear_setup_timer();
}

struct sys_timer spear13xx_timer = {
	.init = spear13xx_timer_init,
};
