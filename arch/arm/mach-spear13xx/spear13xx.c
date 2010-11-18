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
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/dw_dmac.h>
#include <linux/mtd/physmap.h>
#include <linux/ptrace.h>
#include <linux/io.h>
#include <linux/mtd/fsmc.h>
#include <asm/hardware/gic.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/localtimer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_twd.h>
#include <plat/udc.h>
#include <mach/dma.h>
#include <mach/generic.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/misc_regs.h>

unsigned long db900fb_buffer_phys;
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

struct amba_device spear13xx_gpio_device[] = {
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

/* ssp device registeration */
static struct pl022_ssp_controller ssp_platform_data = {
	.bus_id = 0,
	.enable_dma = 0,
	/*
	 * This is number of spi devices that can be connected to spi. There are
	 * two type of chipselects on which slave devices can work. One is chip
	 * select provided by spi masters other is controlled through external
	 * gpio's. We can't use chipselect provided from spi master (because as
	 * soon as FIFO becomes empty, CS is disabled and transfer ends). So
	 * this number now depends on number of gpios available for spi. each
	 * slave on each master requires a separate gpio pin.
	 */
	.num_chipselect = 2,
};

struct amba_device spear13xx_ssp_device = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp-pl022",
		.platform_data = &ssp_platform_data,
	},
	.res = {
		.start = SPEAR13XX_SSP_BASE,
		.end = SPEAR13XX_SSP_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	.irq = {IRQ_SSP, NO_IRQ},
};

/* uart device registeration */
struct amba_device spear13xx_uart_device = {
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

struct platform_device spear13xx_adc_device = {
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

struct platform_device spear13xx_dmac_device[] = {
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

/* i2s0 device registeration */
static struct resource i2s0_resources[] = {
	{
		.start	= SPEAR13XX_I2S0_BASE,
		.end	= SPEAR13XX_I2S0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {

		.name	= "play_irq",
		.start	= IRQ_PLAY_I2S0,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "record_irq",
		.start	= IRQ_REC_I2S0,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_i2s0_device = {
	.name = "spear13xx-i2s",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2s0_resources),
	.resource = i2s0_resources,
};

/* i2s1 device registeration */
static struct resource i2s1_resources[] = {
	{
		.start	= SPEAR13XX_I2S1_BASE,
		.end	= SPEAR13XX_I2S1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	}, {

		.name	= "play_irq",
		.start	= IRQ_PLAY_I2S1,
		.flags	= IORESOURCE_IRQ,
	}, {
		.name	= "record_irq",
		.start	= IRQ_REC_I2S1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_i2s1_device = {
	.name = "spear13xx-i2s",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2s1_resources),
	.resource = i2s1_resources,
};

struct platform_device spear13xx_pcm_device = {
	.name		= "spear-pcm-audio",
	.id		= -1,
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

struct platform_device spear13xx_i2c_device = {
	.name = "i2c_designware",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(i2c_resources),
	.resource = i2c_resources,
};

/* fsmc nor flash device registeration */
static struct physmap_flash_data fsmc_norflash_data;
static struct resource fsmc_nor_resources[] = {
	{
		.start	= SPEAR13XX_FSMC_MEM_BASE,
		.end	= SPEAR13XX_FSMC_MEM_BASE + SZ_16M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_fsmc_nor_device = {
	.name	= "physmap-flash",
	.id	= -1,
	.resource = fsmc_nor_resources,
	.num_resources = ARRAY_SIZE(fsmc_nor_resources),
	.dev.platform_data = &fsmc_norflash_data,
};

/* nand device registeration */
void __init nand_mach_init(u32 busw)
{
	u32 fsmc_cfg = readl(FSMC_CFG);
	fsmc_cfg &= ~(FSMC_MEMSEL_MASK << FSMC_MEMSEL_SHIFT);
	fsmc_cfg |= (FSMC_MEM_NAND << FSMC_MEMSEL_SHIFT);

	if (busw == FSMC_NAND_BW16)
		fsmc_cfg |= 1 << NAND_DEV_WIDTH16;
	else
		fsmc_cfg &= ~(1 << NAND_DEV_WIDTH16);

	writel(fsmc_cfg, FSMC_CFG);
}

static void nand_select_bank(u32 bank, u32 busw)
{
	u32 fsmc_cfg = readl(FSMC_CFG);

	fsmc_cfg &= ~(NAND_BANK_MASK << NAND_BANK_SHIFT);
	fsmc_cfg |= (bank << NAND_BANK_SHIFT);

	if (busw)
		fsmc_cfg |= 1 << NAND_DEV_WIDTH16;
	else
		fsmc_cfg &= ~(1 << NAND_DEV_WIDTH16);

	writel(fsmc_cfg, FSMC_CFG);
}

static struct fsmc_nand_platform_data nand_platform_data = {
	.select_bank = nand_select_bank,
};

static struct resource nand_resources[] = {
	{
		.name = "nand_data",
		.start = SPEAR13XX_FSMC_MEM_BASE,
		.end = SPEAR13XX_FSMC_MEM_BASE + SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.name = "fsmc_regs",
		.start = SPEAR13XX_FSMC_BASE,
		.end = SPEAR13XX_FSMC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_nand_device = {
	.name = "fsmc-nand",
	.id = -1,
	.resource = nand_resources,
	.num_resources = ARRAY_SIZE(nand_resources),
	.dev.platform_data = &nand_platform_data,
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

struct platform_device spear13xx_ehci0_device = {
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

struct platform_device spear13xx_ehci1_device = {
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

struct platform_device spear13xx_ohci0_device = {
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
struct platform_device spear13xx_ohci1_device = {
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

struct platform_device spear13xx_jpeg_device = {
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

struct platform_device spear13xx_kbd_device = {
	.name = "keyboard",
	.id = -1,
	.num_resources = ARRAY_SIZE(kbd_resources),
	.resource = kbd_resources,
};

/* clcd db9000 devide registration */
static struct resource db9000fb_resources[] = {
	[0] = {
		.start = SPEAR13XX_DB9000_LCD_BASE,
		.end = SPEAR13XX_DB9000_LCD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CLCD,
		.end = IRQ_CLCD,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

struct platform_device spear13xx_db9000_clcd_device = {
	.name = "clcd-db9000",
	.id = -1,
	.dev = {
		.parent = NULL,
		.dma_mask = &fb_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(db9000fb_resources),
	.resource = db9000fb_resources,
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

struct platform_device spear13xx_rtc_device = {
	.name = "rtc-spear",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

#ifdef CONFIG_PCIEPORTBUS
/* PCIE0 clock always needs to be enabled if any of the three PCIE port
 * have to be used. So call this function from the board initilization
 * file. Ideally , all controller should have been independent from
 * others with respect to clock.
 */
int enable_pcie0_clk(void)
{
	struct clk *clk;
	/*Enable all CLK in CFG registers here only. Idealy only PCIE0
	 * should have been enabled. But Controler does not work
	 * properly if PCIE1 and PCIE2's CFG CLK is enabled in stages.
	 */
	writel(PCIE0_CFG_VAL | PCIE1_CFG_VAL | PCIE2_CFG_VAL, PCIE_CFG);
	clk = clk_get_sys("pcie0", NULL);
	if (IS_ERR(clk)) {
		pr_err("%s:couldn't get clk for pcie0\n", __func__);
		return -ENODEV;
	}
	if (clk_enable(clk)) {
		pr_err("%s:couldn't enable clk for pcie0\n", __func__);
		return -ENODEV;
	}

	return 0;
}
#endif

/* pcie gadget registration */
static int pcie_gadget0_id;
static u64 pcie_gadget0_dmamask = ~0;
static struct resource pcie_gadget0_resources[] = {
	{
		.start = SPEAR13XX_PCIE0_APP_BASE,
		.end = SPEAR13XX_PCIE0_APP_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE0_BASE,
		.end = SPEAR13XX_PCIE0_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_PCIE0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pcie_gadget0_device = {
	.name = "pcie-gadget-spear",
	.id = 0,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget0_dmamask,
		.platform_data = &pcie_gadget0_id,
	},
	.num_resources = ARRAY_SIZE(pcie_gadget0_resources),
	.resource = pcie_gadget0_resources,
};

static int pcie_gadget1_id;
static u64 pcie_gadget1_dmamask = ~0;
static struct resource pcie_gadget1_resources[] = {
	{
		.start = SPEAR13XX_PCIE1_APP_BASE,
		.end = SPEAR13XX_PCIE1_APP_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE1_BASE,
		.end = SPEAR13XX_PCIE1_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_PCIE1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pcie_gadget1_device = {
	.name = "pcie-gadget-spear",
	.id = 1,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget1_dmamask,
		.platform_data = &pcie_gadget1_id,
	},
	.num_resources = ARRAY_SIZE(pcie_gadget1_resources),
	.resource = pcie_gadget1_resources,
};

static int pcie_gadget2_id;
static u64 pcie_gadget2_dmamask = ~0;
static struct resource pcie_gadget2_resources[] = {
	{
		.start = SPEAR13XX_PCIE2_APP_BASE,
		.end = SPEAR13XX_PCIE2_APP_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = SPEAR13XX_PCIE2_BASE,
		.end = SPEAR13XX_PCIE2_BASE + SZ_8K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_PCIE2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_pcie_gadget2_device = {
	.name = "pcie-gadget-spear",
	.id = 2,
	.dev = {
		.coherent_dma_mask = ~0,
		.dma_mask = &pcie_gadget2_dmamask,
		.platform_data = &pcie_gadget2_id,
	},
	.num_resources = ARRAY_SIZE(pcie_gadget2_resources),
	.resource = pcie_gadget2_resources,
};

/* sdhci (sdio) device declaration */
static struct resource sdhci_resources[] = {
	{
		.start	= SPEAR13XX_MCIF_SDHCI_BASE,
		.end	= SPEAR13XX_MCIF_SDHCI_BASE + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_SDHCI,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device spear13xx_sdhci_device = {
	.dev = {
		.coherent_dma_mask = ~0,
	},
	.name = "sdhci",
	.id = -1,
	.num_resources = ARRAY_SIZE(sdhci_resources),
	.resource = sdhci_resources,
};

/* smi device registration */
static struct resource smi_resources[] = {
	{
		.start = SPEAR13XX_SMI_CTRL_BASE,
		.end = SPEAR13XX_SMI_CTRL_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_SMI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_smi_device = {
	.name = "smi",
	.id = -1,
	.num_resources = ARRAY_SIZE(smi_resources),
	.resource = smi_resources,
};

/* usb device registeration */
static struct resource udc_resources[] = {
	[0] = {
		.start = SPEAR13XX_UDC_BASE,
		.end = SPEAR13XX_UDC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SPEAR13XX_UPD_BASE,
		.end = SPEAR13XX_UPD_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.start = IRQ_UDC,
		.end = IRQ_UDC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device spear13xx_udc_device = {
	.name = "designware_udc",
	.id = -1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources = ARRAY_SIZE(udc_resources),
	.resource = udc_resources,
};

/* wdt device registration */
static struct resource wdt_resources[] = {
	{
		.start = SPEAR13XX_WDT_BASE,
		.end = SPEAR13XX_WDT_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device spear13xx_wdt_device = {
	.name = "cortexa9-wdt",
	.id = -1,
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
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

#ifdef CONFIG_SND_SOC_STA529
static void i2s_clk_init(void)
{
	struct clk *i2s_src_clk, *pll3_clk, *i2s_ref_clk;

	i2s_src_clk = clk_get_sys(NULL, "i2s_src_clk");
	if (IS_ERR(i2s_src_clk)) {
		pr_err("%s:couldn't get clk for i2s_src_clk\n", __func__);
		return;
	}

	pll3_clk = clk_get_sys(NULL, "pll3_clk");
	if (IS_ERR(pll3_clk)) {
		pr_err("%s:couldn't get clk for pll3_clck\n", __func__);
		goto put_src_clk;
	}

	i2s_ref_clk = clk_get_sys(NULL, "i2s_ref_clk");
	if (IS_ERR(i2s_ref_clk)) {
		pr_err("%s:couldn't get clk for i2s_ref_clk\n", __func__);
		goto put_pll3_clk;
	}

	if (clk_set_parent(i2s_src_clk, pll3_clk))
		goto put_ref_clk;

	if (clk_set_rate(pll3_clk, 49152000)) /* 49.15 Mhz */
		goto put_ref_clk;

	if (clk_set_rate(i2s_ref_clk, 12288000)) /*12.288 Mhz */
		goto put_ref_clk;

	if (clk_enable(i2s_ref_clk)) {
		pr_err("%s:enabling i2s_ref_clk_fail\n", __func__);
		goto put_ref_clk;
	}
	goto put_pll3_clk;

put_ref_clk:
	clk_put(i2s_ref_clk);
put_pll3_clk:
	clk_put(pll3_clk);
put_src_clk:
	clk_put(i2s_src_clk);

	return;
}
#endif

/* Do spear13xx familiy common initialization part here */
void __init spear13xx_init(void)
{
#ifdef CONFIG_CACHE_L2X0
	/*
	 * 512KB (64KB/way), 8-way associativity, parity supported
	 *
	 * TODO: 0x249, picked from nomadik, to be analyzed
	 * Comment from nomadik:
	 * At full speed latency must be >=2, so 0x249 in low bits
	 */
	l2x0_init(__io_address(SPEAR13XX_L2CC_BASE), 0x00260249, 0xfe00ffff);
#endif

#ifdef CONFIG_SND_SOC_STA529
	i2s_clk_init();
#endif

	dmac_setup();
	set_udc_plat_data(&spear13xx_udc_device);
}

/* This will initialize vic */
void __init spear13xx_init_irq(void)
{
	gic_dist_init(0, __io_address(SPEAR13XX_GIC_DIST_BASE), 29);
	gic_cpu_init(0, __io_address(SPEAR13XX_GIC_CPU_BASE));
}
static unsigned long reserve_mem(struct meminfo *mi, unsigned long size)
{
	unsigned long addr = ~0;
	int i;
	for (i = mi->nr_banks - 1; i >= 0; i--)
		if (mi->bank[i].size >= size) {
			mi->bank[i].size -= size;
			addr = mi->bank[i].start + mi->bank[i].size;
			break;
		}

	return addr;
}

void __init spear13xx_fixup(struct machine_desc *desc, struct tag *tags,
		char **cmdline, struct meminfo *mi)
{
	db900fb_buffer_phys = reserve_mem(mi, 1024*768*8);
	if (db900fb_buffer_phys == ~0)
		printk(KERN_ERR"Unable to allocate fb buffer\n");
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
#ifdef CONFIG_CACHE_L2X0
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_L2CC_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_L2CC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
#endif
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
	}, {
		.virtual	= IO_ADDRESS(SPEAR13XX_SYSRAM1_BASE),
		.pfn		= __phys_to_pfn(SPEAR13XX_SYSRAM1_BASE),
		.length		= SZ_1M,
		.type		= MT_MEMORY_NONCACHED
	},
};

/* This will create static memory mapping for selected devices */
void __init spear13xx_map_io(void)
{
	iotable_init(spear13xx_io_desc, ARRAY_SIZE(spear13xx_io_desc));

	/* This will initialize clock framework */
	spear13xx_clk_init();
}

static void __init spear13xx_timer_init(void)
{
	char pclk_name[] = "osc1_24m_clk";
	struct clk *gpt_clk, *pclk;

#ifdef CONFIG_LOCAL_TIMERS
	/* Setup the local timer base */
	twd_base = __io_address(SPEAR13XX_LOCAL_TMR_BASE);
#endif

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

struct sys_timer spear13xx_timer = {
	.init = spear13xx_timer_init,
};

/* pad multiplexing support */
/* devices */

/* Pad multiplexing for i2c device */
static struct pmx_mux_reg pmx_i2c_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_I2C_MASK,
		.value = PMX_I2C_MASK,
	},
};

static struct pmx_dev_mode pmx_i2c_modes[] = {
	{
		.mux_regs = pmx_i2c_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2c_mux),
	},
};

struct pmx_dev pmx_i2c = {
	.name = "i2c",
	.modes = pmx_i2c_modes,
	.mode_count = ARRAY_SIZE(pmx_i2c_modes),
};

/* Pad multiplexing for ssp device */
static struct pmx_mux_reg pmx_ssp_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_SSP_MASK,
		.value = PMX_SSP_MASK,
	},
};

static struct pmx_dev_mode pmx_ssp_modes[] = {
	{
		.mux_regs = pmx_ssp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_ssp_mux),
	},
};

struct pmx_dev pmx_ssp = {
	.name = "ssp",
	.modes = pmx_ssp_modes,
	.mode_count = ARRAY_SIZE(pmx_ssp_modes),
};

/* Pad multiplexing for i2s1 device */
static struct pmx_mux_reg pmx_i2s1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_I2S1_MASK,
		.value = PMX_I2S1_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s1_modes[] = {
	{
		.mux_regs = pmx_i2s1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s1_mux),
	},
};

struct pmx_dev pmx_i2s1 = {
	.name = "i2s1",
	.modes = pmx_i2s1_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s1_modes),
};

/* Pad multiplexing for i2s2 device */
static struct pmx_mux_reg pmx_i2s2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_I2S2_MASK,
		.value = PMX_I2S2_MASK,
	},
};

static struct pmx_dev_mode pmx_i2s2_modes[] = {
	{
		.mux_regs = pmx_i2s2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_i2s2_mux),
	},
};

struct pmx_dev pmx_i2s2 = {
	.name = "i2s2",
	.modes = pmx_i2s2_modes,
	.mode_count = ARRAY_SIZE(pmx_i2s2_modes),
};

/* Pad multiplexing for clcd device */
static struct pmx_mux_reg pmx_clcd_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_CLCD1_MASK,
		.value = PMX_CLCD1_MASK,
	},
};

static struct pmx_dev_mode pmx_clcd_modes[] = {
	{
		.mux_regs = pmx_clcd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_mux),
	},
};

struct pmx_dev pmx_clcd = {
	.name = "clcd",
	.modes = pmx_clcd_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_modes),
};

/* Pad multiplexing for clcd_hires device */
static struct pmx_mux_reg pmx_clcd_hires_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_CLCD1_MASK,
		.value = PMX_CLCD1_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_CLCD2_MASK,
		.value = PMX_CLCD2_MASK,
	},
};

static struct pmx_dev_mode pmx_clcd_hires_modes[] = {
	{
		.mux_regs = pmx_clcd_hires_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_clcd_hires_mux),
	},
};

struct pmx_dev pmx_clcd_hires = {
	.name = "clcd_high_res",
	.modes = pmx_clcd_hires_modes,
	.mode_count = ARRAY_SIZE(pmx_clcd_hires_modes),
};

/*
 * By default, all EGPIOs are enabled.
 * TBD : Board specific enabling of specific GPIOs only
 */
static struct pmx_mux_reg pmx_egpio_grp_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_EGPIO_0_GRP_MASK,
		.value = PMX_EGPIO_0_GRP_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_EGPIO_1_GRP_MASK,
		.value = PMX_EGPIO_1_GRP_MASK,
	},
};

static struct pmx_dev_mode pmx_egpio_grp_modes[] = {
	{
		.mux_regs = pmx_egpio_grp_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_egpio_grp_mux),
	},
};

struct pmx_dev pmx_egpio_grp = {
	.name = "egpios",
	.modes = pmx_egpio_grp_modes,
	.mode_count = ARRAY_SIZE(pmx_egpio_grp_modes),
};

/* Pad multiplexing for smi 2 chips device */
static struct pmx_mux_reg pmx_smi_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_SMI_MASK,
		.value = PMX_SMI_MASK,
	},
};

static struct pmx_dev_mode pmx_smi_2_modes[] = {
	{
		.mux_regs = pmx_smi_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smi_2_mux),
	},
};

struct pmx_dev pmx_smi_2_chips = {
	.name = "smi_2_chips",
	.modes = pmx_smi_2_modes,
	.mode_count = ARRAY_SIZE(pmx_smi_2_modes),
};

/* Pad multiplexing for smi 4 chips device */
static struct pmx_mux_reg pmx_smi_4_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_SMI_MASK,
		.value = PMX_SMI_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_SMINCS2_MASK | PMX_SMINCS3_MASK,
		.value = PMX_SMINCS2_MASK | PMX_SMINCS3_MASK,
	},
};

static struct pmx_dev_mode pmx_smi_4_modes[] = {
	{
		.mux_regs = pmx_smi_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_smi_4_mux),
	},
};

struct pmx_dev pmx_smi_4_chips = {
	.name = "smi_4_chips",
	.modes = pmx_smi_4_modes,
	.mode_count = ARRAY_SIZE(pmx_smi_4_modes),
};

/* Pad multiplexing for gmii device */
static struct pmx_mux_reg pmx_gmii_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_GMII_MASK,
		.value = PMX_GMII_MASK,
	},
};

static struct pmx_dev_mode pmx_gmii_modes[] = {
	{
		.mux_regs = pmx_gmii_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gmii_mux),
	},
};

struct pmx_dev pmx_gmii = {
	.name = "gmii",
	.modes = pmx_gmii_modes,
	.mode_count = ARRAY_SIZE(pmx_gmii_modes),
};

/* Pad multiplexing for nand 8bit (4 chips) */
static struct pmx_mux_reg pmx_nand8_4_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_NAND8BIT4DEV_0_MASK,
		.value = PMX_NAND8BIT4DEV_0_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NAND8BIT4DEV_1_MASK | PMX_KEYBOARD_6X6_MASK,
		.value = PMX_NAND8BIT4DEV_1_MASK | PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand8_4_modes[] = {
	{
		.mux_regs = pmx_nand8_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand8_4_mux),
	},
};

struct pmx_dev pmx_nand_8bit_4_chips = {
	.name = "nand-8bit_4_chips",
	.modes = pmx_nand8_4_modes,
	.mode_count = ARRAY_SIZE(pmx_nand8_4_modes),
};

/* Pad multiplexing for nand 8bit device (cs0 only) */
static struct pmx_mux_reg pmx_nand8_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_NAND8BIT_0_MASK,
		.value = PMX_NAND8BIT_0_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NAND8BIT_1_MASK | PMX_KEYBOARD_6X6_MASK,
		.value = PMX_NAND8BIT_1_MASK | PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand8_modes[] = {
	{
		.mux_regs = pmx_nand8_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand8_mux),
	},
};

struct pmx_dev pmx_nand_8bit = {
	.name = "nand-8bit",
	.modes = pmx_nand8_modes,
	.mode_count = ARRAY_SIZE(pmx_nand8_modes),
};

/*
 * Pad multiplexing for nand 16bit device
 * Note : Enabling pmx_nand_16bit means that all the required pads for
 *   16bit nand device operations are enabled. These also include pads
 *   for 8bit devices
 */
static struct pmx_mux_reg pmx_nand16_4_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_NAND16BIT4DEV_0_MASK,
		.value = PMX_NAND16BIT4DEV_0_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NAND16BIT4DEV_1_MASK | PMX_KEYBOARD_6X6_MASK,
		.value = PMX_NAND16BIT4DEV_1_MASK | PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand16_4_modes[] = {
	{
		.mux_regs = pmx_nand16_4_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand16_4_mux),
	},
};

struct pmx_dev pmx_nand_16bit_4_chips = {
	.name = "nand-16bit_4_chips",
	.modes = pmx_nand16_4_modes,
	.mode_count = ARRAY_SIZE(pmx_nand16_4_modes),
};

/* Pad multiplexing for nand 16bit device (cs0 only) */
static struct pmx_mux_reg pmx_nand16_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_NAND16BIT_0_MASK,
		.value = PMX_NAND16BIT_0_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NAND16BIT_1_MASK | PMX_KEYBOARD_6X6_MASK,
		.value = PMX_NAND16BIT_1_MASK | PMX_KEYBOARD_6X6_MASK,
	},
};

static struct pmx_dev_mode pmx_nand16_modes[] = {
	{
		.mux_regs = pmx_nand16_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_nand16_mux),
	},
};

struct pmx_dev pmx_nand_16bit = {
	.name = "nand-16bit",
	.modes = pmx_nand16_modes,
	.mode_count = ARRAY_SIZE(pmx_nand16_modes),
};

/* Pad multiplexing for keyboard_6x6 device */
static struct pmx_mux_reg pmx_keyboard_6x6_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_KEYBOARD_6X6_MASK,
		.value = PMX_KEYBOARD_6X6_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NFIO815_MASK | PMX_NFCE1_MASK | \
			PMX_NFCE2_MASK | PMX_NFWPRT1_MASK | PMX_NFWPRT2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_keyboard_6x6_modes[] = {
	{
		.mux_regs = pmx_keyboard_6x6_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_6x6_mux),
	},
};

struct pmx_dev pmx_keyboard_6x6 = {
	.name = "keyboard_6x6",
	.modes = pmx_keyboard_6x6_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_6x6_modes),
};

/* Pad multiplexing for keyboard_9x9 device */
static struct pmx_mux_reg pmx_keyboard_9x9_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_KEYBOARD_6X6_MASK | PMX_KBD_ROWCOL68_MASK,
		.value = PMX_KEYBOARD_6X6_MASK | PMX_KBD_ROWCOL68_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_NFIO815_MASK | PMX_NFCE1_MASK | \
			PMX_NFCE2_MASK | PMX_NFWPRT1_MASK | PMX_NFWPRT2_MASK,
		.value = 0,
	},
};

static struct pmx_dev_mode pmx_keyboard_9x9_modes[] = {
	{
		.mux_regs = pmx_keyboard_9x9_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_keyboard_9x9_mux),
	},
};

struct pmx_dev pmx_keyboard_9x9 = {
	.name = "keyboard_9x9",
	.modes = pmx_keyboard_9x9_modes,
	.mode_count = ARRAY_SIZE(pmx_keyboard_9x9_modes),
};

/* Pad multiplexing for uart0 device */
static struct pmx_mux_reg pmx_uart0_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_0,
		.mask = PMX_UART0_MASK,
		.value = PMX_UART0_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modes[] = {
	{
		.mux_regs = pmx_uart0_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_mux),
	},
};

struct pmx_dev pmx_uart0 = {
	.name = "uart0",
	.modes = pmx_uart0_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modes),
};

/* Pad multiplexing for uart0_modem device */
static struct pmx_mux_reg pmx_uart0_modem_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_UART0_MODEM_MASK,
		.value = PMX_UART0_MODEM_MASK,
	},
};

static struct pmx_dev_mode pmx_uart0_modem_modes[] = {
	{
		.mux_regs = pmx_uart0_modem_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_uart0_modem_mux),
	},
};

struct pmx_dev pmx_uart0_modem = {
	.name = "uart0_modem",
	.modes = pmx_uart0_modem_modes,
	.mode_count = ARRAY_SIZE(pmx_uart0_modem_modes),
};

/* Pad multiplexing for gpt_0_1 device */
static struct pmx_mux_reg pmx_gpt_0_1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_GPT0_TMR1_MASK,
		.value = PMX_GPT0_TMR1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_1_modes[] = {
	{
		.mux_regs = pmx_gpt_0_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_1_mux),
	},
};

struct pmx_dev pmx_gpt_0_1 = {
	.name = "gpt_0_1",
	.modes = pmx_gpt_0_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_1_modes),
};

/* Pad multiplexing for gpt_0_2 device */
static struct pmx_mux_reg pmx_gpt_0_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_GPT0_TMR2_MASK,
		.value = PMX_GPT0_TMR2_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_0_2_modes[] = {
	{
		.mux_regs = pmx_gpt_0_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_0_2_mux),
	},
};

struct pmx_dev pmx_gpt_0_2 = {
	.name = "gpt_0_2",
	.modes = pmx_gpt_0_2_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_0_2_modes),
};

/* Pad multiplexing for gpt_1_1 device */
static struct pmx_mux_reg pmx_gpt_1_1_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_GPT1_TMR1_MASK,
		.value = PMX_GPT1_TMR1_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_1_1_modes[] = {
	{
		.mux_regs = pmx_gpt_1_1_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_1_1_mux),
	},
};

struct pmx_dev pmx_gpt_1_1 = {
	.name = "gpt_1_1",
	.modes = pmx_gpt_1_1_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_1_1_modes),
};

/* Pad multiplexing for gpt_1_2 device */
static struct pmx_mux_reg pmx_gpt_1_2_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_GPT1_TMR2_MASK,
		.value = PMX_GPT1_TMR2_MASK,
	},
};

static struct pmx_dev_mode pmx_gpt_1_2_modes[] = {
	{
		.mux_regs = pmx_gpt_1_2_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_gpt_1_2_mux),
	},
};

struct pmx_dev pmx_gpt_1_2 = {
	.name = "gpt_1_2",
	.modes = pmx_gpt_1_2_modes,
	.mode_count = ARRAY_SIZE(pmx_gpt_1_2_modes),
};

/* Pad multiplexing for mcif device */
static struct pmx_mux_reg pmx_mcif_mux[] = {
	{
		.address = PAD_MUX_CONFIG_REG_1,
		.mask = PMX_MCIFALL_1_MASK,
		.value = PMX_MCIFALL_1_MASK,
	}, {
		.address = PAD_MUX_CONFIG_REG_2,
		.mask = PMX_MCIFALL_2_MASK,
		.value = PMX_MCIFALL_2_MASK,
	},
};

static struct pmx_dev_mode pmx_mcif_modes[] = {
	{
		.mux_regs = pmx_mcif_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_mcif_mux),
	},
};

struct pmx_dev pmx_mcif = {
	.name = "mcif",
	.modes = pmx_mcif_modes,
	.mode_count = ARRAY_SIZE(pmx_mcif_modes),
};

/* Pad multiplexing for sdhci device */
static struct pmx_mux_reg pmx_sdhci_mux[] = {
	{
		.address = SDHCI_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_SD,
	},
};

static struct pmx_dev_mode pmx_sdhci_modes[] = {
	{
		.mux_regs = pmx_sdhci_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_sdhci_mux),
	},
};

struct pmx_dev pmx_sdhci = {
	.name = "sdhci",
	.modes = pmx_sdhci_modes,
	.mode_count = ARRAY_SIZE(pmx_sdhci_modes),
};

/* Pad multiplexing for cf device */
static struct pmx_mux_reg pmx_cf_mux[] = {
	{
		.address = SDHCI_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_CF,
	},
};

static struct pmx_dev_mode pmx_cf_modes[] = {
	{
		.mux_regs = pmx_cf_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_cf_mux),
	},
};

struct pmx_dev pmx_cf = {
	.name = "cf",
	.modes = pmx_cf_modes,
	.mode_count = ARRAY_SIZE(pmx_cf_modes),
};

/* Pad multiplexing for xd device */
static struct pmx_mux_reg pmx_xd_mux[] = {
	{
		.address = SDHCI_CFG,
		.mask = MCIF_SEL_MASK,
		.value = MCIF_SEL_XD,
	},
};

static struct pmx_dev_mode pmx_xd_modes[] = {
	{
		.mux_regs = pmx_xd_mux,
		.mux_reg_cnt = ARRAY_SIZE(pmx_xd_mux),
	},
};

struct pmx_dev pmx_xd = {
	.name = "xd",
	.modes = pmx_xd_modes,
	.mode_count = ARRAY_SIZE(pmx_xd_modes),
};
