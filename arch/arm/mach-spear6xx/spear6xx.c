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
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/amba/pl08x.h>
#include <linux/amba/serial.h>
#include <linux/netdevice.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <linux/mtd/fsmc.h>
#include <linux/spear_adc_usr.h>
#include <linux/stmmac.h>
#include <asm/hardware/pl080.h>
#include <asm/hardware/vic.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <plat/adc.h>
#include <plat/cpufreq.h>
#include <plat/jpeg.h>
#include <plat/touchscreen.h>
#include <plat/udc.h>
#include <mach/irqs.h>
#include <mach/generic.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

/* The wake sources are routed through vic-2 */
#define SPEAR6XX_WKUP_SRCS_VIC2		(1 << (IRQ_GMAC_1 - 32) | \
					1 << (IRQ_USB_DEV - 32) | \
					1 << (IRQ_BASIC_RTC - 32) |\
					1 << (IRQ_BASIC_GPIO - 32))

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
		.end = SPEAR6XX_ICM3_CLCD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.irq = {IRQ_BASIC_CLCD, NO_IRQ},
};

static struct pl08x_platform_data pl080_plat_data = {
	.memcpy_channel = {
		.bus_id = "memcpy",
		.cctl = (PL080_BSIZE_16 << PL080_CONTROL_SB_SIZE_SHIFT | \
			PL080_BSIZE_16 << PL080_CONTROL_DB_SIZE_SHIFT | \
			PL080_WIDTH_32BIT << PL080_CONTROL_SWIDTH_SHIFT | \
			PL080_WIDTH_32BIT << PL080_CONTROL_DWIDTH_SHIFT | \
			PL080_CONTROL_PROT_BUFF | PL080_CONTROL_PROT_CACHE | \
			PL080_CONTROL_PROT_SYS),
	},
	.lli_buses = PL08X_AHB1,
	.mem_buses = PL08X_AHB1,
};

struct amba_device dma_device = {
	.dev = {
		.init_name = "pl080_dmac",
		.coherent_dma_mask = ~0,
		.platform_data = &pl080_plat_data,
	},
	.res = {
		.start = SPEAR6XX_ICM3_DMA_BASE,
		.end = SPEAR6XX_ICM3_DMA_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_BASIC_DMA, NO_IRQ},
};

/* ssp device registration */
static struct pl022_ssp_controller ssp_platform_data[] = {
	{
		.bus_id = 0,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp0_tx",
		.dma_rx_param = "ssp0_rx",

		/*
		 * This is number of spi devices that can be connected to spi.
		 * There are two type of chipselects on which slave devices can
		 * work. One is chip select provided by spi masters other is
		 * controlled through external gpio's. We can't use chipselect
		 * provided from spi master (because as soon as FIFO becomes
		 * empty, CS is disabled and transfer ends). So this number now
		 * depends on number of gpios available for spi. each slave on
		 * each master requires a separate gpio pin.
		 */
		.num_chipselect = 2,
	}, {
		.bus_id = 1,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp1_tx",
		.dma_rx_param = "ssp1_rx",
		.num_chipselect = 2,
	}, {
		.bus_id = 2,
		.enable_dma = 1,
		.dma_filter = pl08x_filter_id,
		.dma_tx_param = "ssp2_tx",
		.dma_rx_param = "ssp2_rx",
		.num_chipselect = 2,
	}
};

struct amba_device ssp_device[] = {
	{
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.0",
			.platform_data = &ssp_platform_data[0],
		},
		.res = {
			.start = SPEAR6XX_ICM1_SSP0_BASE,
			.end = SPEAR6XX_ICM1_SSP0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_SSP_1, NO_IRQ},
	}, {
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.1",
			.platform_data = &ssp_platform_data[1],
		},
		.res = {
			.start = SPEAR6XX_ICM1_SSP1_BASE,
			.end = SPEAR6XX_ICM1_SSP1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_SSP_2, NO_IRQ},
	}, {
		.dev = {
			.coherent_dma_mask = ~0,
			.init_name = "ssp-pl022.2",
			.platform_data = &ssp_platform_data[2],
		},
		.res = {
			.start = SPEAR6XX_ICM2_SSP2_BASE,
			.end = SPEAR6XX_ICM2_SSP2_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_APPL_SSP, NO_IRQ},
	}
};

/* As uart0 is used for console, so disable DMA here */
/* uart devices plat data */
#if 0
static struct amba_pl011_data uart0_data = {
	.dma_filter = pl08x_filter_id,
	.dma_tx_param = "uart0_tx",
	.dma_rx_param = "uart0_rx",
}
#endif

static struct amba_pl011_data uart1_data = {
	.dma_filter = pl08x_filter_id,
	.dma_tx_param = "uart1_tx",
	.dma_rx_param = "uart1_rx",
};

/* uart device registration */
struct amba_device uart_device[] = {
	{
		.dev = {
			.init_name = "uart0",
/*			.platform_data = &uart0_data, */
		},
		.res = {
			.start = SPEAR6XX_ICM1_UART0_BASE,
			.end = SPEAR6XX_ICM1_UART0_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_UART_0, NO_IRQ},
	}, {
		.dev = {
			.init_name = "uart1",
			.platform_data = &uart1_data,
		},
		.res = {
			.start = SPEAR6XX_ICM1_UART1_BASE,
			.end = SPEAR6XX_ICM1_UART1_BASE + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		.irq = {IRQ_UART_1, NO_IRQ},
	}
};

/* gpio device registration */
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

/* watchdog device registeration */
struct amba_device wdt_device = {
	.dev = {
		.init_name = "wdt",
	},
	.res = {
		.start = SPEAR6XX_ICM3_WDT_BASE,
		.end = SPEAR6XX_ICM3_WDT_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

/* adc device registeration */
static struct adc_plat_data adc_pdata = {
	.dma_filter = pl08x_filter_id,
	.dma_data = "adc",
	.config = {CONTINUOUS_CONVERSION, EXTERNAL_VOLT, 2500, INTERNAL_SCAN,
		14000000, 0},
};

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
		.platform_data = &adc_pdata,
	},
	.num_resources = ARRAY_SIZE(adc_resources),
	.resource = adc_resources,
};

/* cpufreq platform device */
static u32 cpu_freq_tbl[] = {
	166000, /* 166 MHZ */
	266000, /* 266 MHZ */
	332000, /* 332 MHZ */
};

static struct spear_cpufreq_pdata cpufreq_pdata = {
	.cpu_freq_table = cpu_freq_tbl,
	.tbl_len = ARRAY_SIZE(cpu_freq_tbl),
};

struct platform_device cpufreq_device = {
	.name = "cpufreq-spear",
	.id = -1,
	.dev = {
		.platform_data = &cpufreq_pdata,
	},
};

/* stmmac device registeration */
static struct plat_stmmacenet_data eth_platform_data = {
	.bus_id = 0,
	.has_gmac = 1,
	.enh_desc = 0,
	.tx_coe = 0,
	.pbl = 8,
	.csum_off_engine = STMAC_TYPE_1,
	.bugged_jumbo = 0,
	.features = NETIF_F_HW_CSUM,
	.pmt = 1,
};

static struct resource eth_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_GMAC_BASE,
		.end = SPEAR6XX_ICM4_GMAC_BASE + SZ_32K - 1,
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
		.name = "eth_wake_irq",
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

/* Fast Irda Controller registration */
static struct resource irda_resources[] = {
	{
		.start = SPEAR6XX_ICM1_IRDA_BASE,
		.end = SPEAR6XX_ICM1_IRDA_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_IRDA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device irda_device = {
	.name = "dice_ir",
	.id = -1,
	.num_resources = ARRAY_SIZE(irda_resources),
	.resource = irda_resources,
};

/* nand device registeration */
static struct fsmc_nand_platform_data nand_platform_data = {
	.mode = USE_WORD_ACCESS,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR6XX_ICM1_NAND_BASE,
		.end = SPEAR6XX_ICM1_NAND_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR6XX_ICM1_FSMC_BASE,
		.end = SPEAR6XX_ICM1_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
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
static struct jpeg_plat_data jpeg_pdata = {
	.dma_filter = pl08x_filter_id,
	.mem2jpeg_slave = "to_jpeg",
	.jpeg2mem_slave = "from_jpeg",
};

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
		.platform_data = &jpeg_pdata,
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

/* touchscreen Device registration */
static struct spear_touchscreen touchscreen_info = {
	.adc_channel_x = ADC_CHANNEL5,
	.adc_channel_y = ADC_CHANNEL6,
	.gpio_pin = APPL_GPIO_7,
};

struct platform_device touchscreen_device = {
	.name = "spear-ts",
	.id = -1,
	.dev = {
		.platform_data = &touchscreen_info,
		.coherent_dma_mask = ~0,
	},
	.num_resources = 0,
	.resource = NULL,
};

/* usb device registeration */
static struct resource udc_resources[] = {
	[0] = {
		.start = SPEAR6XX_ICM4_USBD_CSR_BASE,
		.end = SPEAR6XX_ICM4_USBD_CSR_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR6XX_ICM4_USBD_PLDT_BASE,
		.end = SPEAR6XX_ICM4_USBD_PLDT_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.start = IRQ_USB_DEV,
		.end = IRQ_USB_DEV,
		.flags = IORESOURCE_IRQ,
	},
};
struct platform_device udc_device = {
	.name = "designware_udc",
	.id = -1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = ARRAY_SIZE(udc_resources),
	.resource = udc_resources,
};

/* This will add devices, and do machine specific tasks */
void __init spear6xx_init(void)
{
	set_udc_plat_data(&udc_device);
}

/* This will initialize vic */
void __init spear6xx_init_irq(void)
{
	vic_init((void __iomem *)VA_SPEAR6XX_CPU_VIC_PRI_BASE, 0, ~0, 0);
	vic_init((void __iomem *)VA_SPEAR6XX_CPU_VIC_SEC_BASE, 32, ~0,
			SPEAR6XX_WKUP_SRCS_VIC2);
}

/* Following will create static virtual/physical mappings */
static struct map_desc spear6xx_io_desc[] __initdata = {
	{
		.virtual	= VA_SPEAR6XX_ICM1_UART0_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM1_UART0_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_CPU_VIC_PRI_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_CPU_VIC_PRI_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_CPU_VIC_SEC_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_CPU_VIC_SEC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_ICM3_SYS_CTRL_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM3_SYS_CTRL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= VA_SPEAR6XX_ICM3_MISC_REG_BASE,
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM3_MISC_REG_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR6XX_ICM3_SDRAM_CTRL_BASE),
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM3_SDRAM_CTRL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	}, {
		.virtual	= IO_ADDRESS(SPEAR6XX_ICM1_SRAM_BASE),
		.pfn		= __phys_to_pfn(SPEAR6XX_ICM1_SRAM_BASE),
		.length		= SZ_4K,
		.type		= MT_MEMORY_NONCACHED
	},
};

/* This will create static memory mapping for selected devices */
void __init spear6xx_map_io(void)
{
	iotable_init(spear6xx_io_desc, ARRAY_SIZE(spear6xx_io_desc));

	/* This will initialize clock framework */
	spear6xx_clk_init();
}

static void __init spear6xx_timer_init(void)
{
	char pclk_name[] = "pll3_48m_clk";
	struct clk *gpt_clk, *pclk;

	/* get the system timer clock */
	gpt_clk = clk_get_sys("gpt0", NULL);
	if (IS_ERR(gpt_clk)) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		BUG();
	}

	/* get the suitable parent clock for timer*/
	pclk = clk_get(NULL, pclk_name);
	if (IS_ERR(pclk)) {
		pr_err("%s:couldn't get %s as parent for gpt\n",
				__func__, pclk_name);
		BUG();
	}

	clk_set_parent(gpt_clk, pclk);
	clk_put(gpt_clk);
	clk_put(pclk);

	spear_setup_timer();
}

struct sys_timer spear6xx_timer = {
	.init = spear6xx_timer_init,
};
