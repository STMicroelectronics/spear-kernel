/*
 * arch/arm/mach-spear13xx/include/mach/generic.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <plat/padmux.h>

/*
 * Function enable (Pad multiplexing register) offsets
 */
/* Pad multiplexing base */
#define SPEAR13XX_FUNC_ENB_BASE		UL(0xE0700650)
#define SPEAR13XX_PCM_CFG_BASE		UL(0xE0700100)

#define PAD_MUX_CONFIG_REG_0	UL(0xE0700650)
#define PAD_MUX_CONFIG_REG_1	UL(0xE0700654)
#define PAD_MUX_CONFIG_REG_2	UL(0xE0700658)
#define PAD_MUX_CONFIG_REG_3	UL(0xE070065C)

#if defined(CONFIG_MACH_SPEAR1310)
#define SPEAR1310_FUNC_CNTL_0	UL(0x6C800000)

#define PMX_SMII_MASK		(1 << 24)	/* Func cntl reg0 */
#define PMX_EGPIO7_MASK		(1 << 2)	/* Pcm cfg reg */
#endif

/* pad mux declarations */
#define PMX_I2S1_MASK		(1 << 3)
#define PMX_I2S2_MASK		(1 << 16)	/* Offset 4 */
#define PMX_CLCD1_MASK		(1 << 5)
#define PMX_CLCD2_MASK		(1 << 3)	/* Offset 4 */
#define PMX_EGPIO00_MASK	(1 << 6)
#define PMX_EGPIO01_MASK	(1 << 7)
#define PMX_EGPIO02_MASK	(1 << 8)
#define PMX_EGPIO03_MASK	(1 << 9)
#define PMX_EGPIO04_MASK	(1 << 10)
#define PMX_EGPIO05_MASK	(1 << 11)
#define PMX_EGPIO06_MASK	(1 << 12)
#define PMX_EGPIO07_MASK	(1 << 13)
#define PMX_EGPIO08_MASK	(1 << 14)
#define PMX_EGPIO09_MASK	(1 << 15)
#define PMX_EGPIO10_MASK	(1 << 5)	/* Offset 4 */
#define PMX_EGPIO11_MASK	(1 << 6)	/* Offset 4 */
#define PMX_EGPIO12_MASK	(1 << 7)	/* Offset 4 */
#define PMX_EGPIO13_MASK	(1 << 8)	/* Offset 4 */
#define PMX_EGPIO14_MASK	(1 << 9)	/* Offset 4 */
#define PMX_EGPIO15_MASK	(1 << 10)	/* Offset 4 */
#define PMX_EGPIO_0_GRP_MASK	(PMX_EGPIO00_MASK | PMX_EGPIO01_MASK | \
		PMX_EGPIO02_MASK | PMX_EGPIO03_MASK | PMX_EGPIO04_MASK | \
		PMX_EGPIO05_MASK | PMX_EGPIO06_MASK | PMX_EGPIO07_MASK | \
		PMX_EGPIO08_MASK | PMX_EGPIO09_MASK)
#define PMX_EGPIO_1_GRP_MASK	(PMX_EGPIO10_MASK | PMX_EGPIO11_MASK | \
		PMX_EGPIO12_MASK | PMX_EGPIO13_MASK | PMX_EGPIO14_MASK | \
		PMX_EGPIO15_MASK)

#define PMX_SMI_MASK		(1 << 16)
#define PMX_SMINCS2_MASK	(1 << 1)	/* Offset 4 */
#define PMX_SMINCS3_MASK	(1 << 2)	/* Offset 4 */

#define PMX_GMIICLK_MASK			(1 << 18)
#define PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK	(1 << 19)
#define PMX_RXCLK_RDV_TXEN_D03_MASK		(1 << 20)
#define PMX_GMIID47_MASK			(1 << 21)
#define PMX_MDC_MDIO_MASK			(1 << 22)

#define PMX_GMII_MASK		(PMX_GMIICLK_MASK | \
		PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK | \
		PMX_RXCLK_RDV_TXEN_D03_MASK | PMX_GMIID47_MASK | \
		PMX_MDC_MDIO_MASK)

#define PMX_NAND8_MASK		(1 << 17)
#define PMX_NFAD023_MASK	(1 << 24)
#define PMX_NFAD24_MASK		(1 << 25)
#define PMX_NFAD25_MASK		(1 << 26)
#define PMX_NFWPRT1_MASK	(1 << 24)	/* Offset 4 */
#define PMX_NFWPRT2_MASK	(1 << 26)	/* Offset 4 */
#define PMX_NFWPRT3_MASK	(1 << 28)
#define PMX_NFRSTPWDWN0_MASK	(1 << 29)
#define PMX_NFRSTPWDWN1_MASK	(1 << 30)
#define PMX_NFRSTPWDWN2_MASK	(1 << 31)
#define PMX_NFRSTPWDWN3_MASK	(1 << 0)	/* Offset 4 */
#define PMX_NFCE1_MASK		(1 << 20)	/* Offset 4 */
#define PMX_NFCE2_MASK		(1 << 22)	/* Offset 4 */
#define PMX_NFCE3_MASK		(1 << 27)
#define PMX_NFIO815_MASK	(1 << 18)	/* Offset 4 */

#define PMX_NAND8BIT_0_MASK	(PMX_NAND8_MASK | PMX_NFAD023_MASK | \
		PMX_NFAD24_MASK | PMX_NFAD25_MASK | PMX_NFWPRT3_MASK | \
		PMX_NFRSTPWDWN0_MASK | PMX_NFRSTPWDWN1_MASK | \
		PMX_NFRSTPWDWN2_MASK | PMX_NFCE3_MASK)
#define PMX_NAND8BIT_1_MASK	(PMX_NFRSTPWDWN3_MASK)

#define PMX_NAND8BIT4DEV_0_MASK	(PMX_NAND8BIT_0_MASK)
#define PMX_NAND8BIT4DEV_1_MASK	(PMX_NAND8BIT_1_MASK | PMX_NFCE1_MASK | \
		PMX_NFCE2_MASK | PMX_NFWPRT1_MASK | PMX_NFWPRT2_MASK)

#define PMX_NAND16BIT_0_MASK	(PMX_NAND8BIT_0_MASK)
#define PMX_NAND16BIT_1_MASK	(PMX_NAND8BIT_1_MASK | PMX_NFIO815_MASK)
#define PMX_NAND16BIT4DEV_0_MASK	(PMX_NAND8BIT4DEV_0_MASK)
#define PMX_NAND16BIT4DEV_1_MASK	(PMX_NAND8BIT4DEV_1_MASK | \
					PMX_NFIO815_MASK)

#define PMX_KBD_ROW0_MASK	(1 << 25)	/* Offset 4 */
#define PMX_KBD_ROW1_MASK	(1 << 23)	/* Offset 4 */
#define PMX_KBD_ROWCOL25_MASK	(1 << 17)	/* Offset 4 */
#define PMX_KBD_ROWCOL68_MASK	(1 << 4)	/* Offset 4 */
#define PMX_KBD_COL0_MASK	(1 << 21)	/* Offset 4 */
#define PMX_KBD_COL1_MASK	(1 << 19)	/* Offset 4 */
#define PMX_KEYBOARD_6X6_MASK	(PMX_KBD_ROW0_MASK | PMX_KBD_ROW1_MASK | \
		PMX_KBD_ROWCOL25_MASK | PMX_KBD_COL0_MASK | PMX_KBD_COL1_MASK)

#define PMX_UART0_MASK		(1 << 1)
#define PMX_I2C_MASK		(1 << 2)
#define PMX_SSP_MASK		(1 << 4)
#define PMX_UART0_MODEM_MASK	(1 << 11)	/* Offset 4 */
#define PMX_GPT0_TMR1_MASK	(1 << 12)	/* Offset 4 */
#define PMX_GPT0_TMR2_MASK	(1 << 13)	/* Offset 4 */
#define PMX_GPT1_TMR1_MASK	(1 << 14)	/* Offset 4 */
#define PMX_GPT1_TMR2_MASK	(1 << 15)	/* Offset 4 */

#define PMX_MCIDATA0_MASK	(1 << 27)	/* Offset 4 */
#define PMX_MCIDATA1_MASK	(1 << 28)	/* Offset 4 */
#define PMX_MCIDATA2_MASK	(1 << 29)	/* Offset 4 */
#define PMX_MCIDATA3_MASK	(1 << 30)	/* Offset 4 */
#define PMX_MCIDATA4_MASK	(1 << 31)	/* Offset 4 */
#define PMX_MCIDATA5_MASK	(1 << 0)	/* Offset 8 */
#define PMX_MCIDATA6_MASK	(1 << 1)	/* Offset 8 */
#define PMX_MCIDATA7_MASK	(1 << 2)	/* Offset 8 */
#define PMX_MCIDATA1SD_MASK	(1 << 3)	/* Offset 8 */
#define PMX_MCIDATA2SD_MASK	(1 << 4)	/* Offset 8 */
#define PMX_MCIDATA3SD_MASK	(1 << 5)	/* Offset 8 */
#define PMX_MCIADDR0ALE_MASK	(1 << 6)	/* Offset 8 */
#define PMX_MCIADDR1CLECLK_MASK	(1 << 7)	/* Offset 8 */
#define PMX_MCIADDR2_MASK	(1 << 8)	/* Offset 8 */
#define PMX_MCICECF_MASK	(1 << 9)	/* Offset 8 */
#define PMX_MCICEXD_MASK	(1 << 10)	/* Offset 8 */
#define PMX_MCICESDMMC_MASK	(1 << 11)	/* Offset 8 */
#define PMX_MCICDCF1_MASK	(1 << 12)	/* Offset 8 */
#define PMX_MCICDCF2_MASK	(1 << 13)	/* Offset 8 */
#define PMX_MCICDXD_MASK	(1 << 14)	/* Offset 8 */
#define PMX_MCICDSDMMC_MASK	(1 << 15)	/* Offset 8 */
#define PMX_MCIDATADIR_MASK	(1 << 16)	/* Offset 8 */
#define PMX_MCIDMARQWP_MASK	(1 << 17)	/* Offset 8 */
#define PMX_MCIIORDRE_MASK	(1 << 18)	/* Offset 8 */
#define PMX_MCIIOWRWE_MASK	(1 << 19)	/* Offset 8 */
#define PMX_MCIRESETCF_MASK	(1 << 20)	/* Offset 8 */
#define PMX_MCICS0CE_MASK	(1 << 21)	/* Offset 8 */
#define PMX_MCICFINTR_MASK	(1 << 22)	/* Offset 8 */
#define PMX_MCIIORDY_MASK	(1 << 23)	/* Offset 8 */
#define PMX_MCICS1_MASK		(1 << 24)	/* Offset 8 */
#define PMX_MCIDMAACK_MASK	(1 << 25)	/* Offset 8 */
#define PMX_MCISDCMD_MASK	(1 << 26)	/* Offset 8 */
#define PMX_MCILEDS_MASK	(1 << 27)	/* Offset 8 */

#define PMX_MCIFALL_1_MASK	(0xF8000000)
#define PMX_MCIFALL_2_MASK	(0x0FFFFFFF)

/* pad mux devices */
extern struct pmx_dev pmx_i2c;
extern struct pmx_dev pmx_ssp;
extern struct pmx_dev pmx_i2s1;
extern struct pmx_dev pmx_i2s2;
extern struct pmx_dev pmx_clcd;
extern struct pmx_dev pmx_clcd_hires;
extern struct pmx_dev pmx_egpio_grp;
extern struct pmx_dev pmx_smi_2_chips;
extern struct pmx_dev pmx_smi_4_chips;
extern struct pmx_dev pmx_gmii;
extern struct pmx_dev pmx_nand_8bit;
extern struct pmx_dev pmx_nand_16bit;
extern struct pmx_dev pmx_keyboard_6x6;
extern struct pmx_dev pmx_keyboard_9x9;
extern struct pmx_dev pmx_uart0;
extern struct pmx_dev pmx_uart0_modem;
extern struct pmx_dev pmx_gpt_0_1;
extern struct pmx_dev pmx_gpt_0_2;
extern struct pmx_dev pmx_gpt_1_1;
extern struct pmx_dev pmx_gpt_1_2;
extern struct pmx_dev pmx_mcif;
extern struct pmx_dev pmx_sdhci;
extern struct pmx_dev pmx_cf;
extern struct pmx_dev pmx_xd;

#if defined(CONFIG_MACH_SPEAR1310)
extern struct pmx_dev pmx_uart1_modem;
extern struct pmx_dev pmx_uart_1;
extern struct pmx_dev pmx_uart_2;
extern struct pmx_dev pmx_uart_3_4_5;
extern struct pmx_dev pmx_rs485_hdlc_1_2;
extern struct pmx_dev pmx_tdm_hdlc_1_2;
extern struct pmx_dev pmx_nand32bit;
extern struct pmx_dev pmx_fsmc16bit_4_chips;
extern struct pmx_dev pmx_fsmc32bit_4_chips;
extern struct pmx_dev pmx_gmii1;
extern struct pmx_dev pmx_rgmii;
extern struct pmx_dev pmx_i2c1;
extern struct pmx_dev pmx_smii_0_1_2;
extern struct pmx_dev pmx_can;
extern struct pmx_dev pmx_uart1_modem;
#endif

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	IRQ_GPT0_TMR1

/* Add spear13xx family device structure declarations here */
extern struct amba_device spear13xx_gpio_device[];
extern struct amba_device spear13xx_ssp_device;
extern struct amba_device spear13xx_uart_device;
extern struct platform_device spear13xx_adc_device;
extern struct platform_device spear13xx_db9000_clcd_device;
extern struct platform_device spear13xx_dmac_device[];
extern struct platform_device spear13xx_ehci0_device;
extern struct platform_device spear13xx_ehci1_device;
extern struct platform_device spear13xx_fsmc_nor_device;
extern struct platform_device spear13xx_i2c_device;
extern struct platform_device spear13xx_i2s0_device;
extern struct platform_device spear13xx_i2s1_device;
extern struct platform_device spear13xx_jpeg_device;
extern struct platform_device spear13xx_kbd_device;
extern struct platform_device spear13xx_nand_device;
extern struct platform_device spear13xx_ohci0_device;
extern struct platform_device spear13xx_ohci1_device;
extern struct platform_device spear13xx_pcie_gadget0_device;
extern struct platform_device spear13xx_pcie_gadget1_device;
extern struct platform_device spear13xx_pcie_gadget2_device;
extern struct platform_device spear13xx_pcm_device;
extern struct platform_device spear13xx_rtc_device;
extern struct platform_device spear13xx_sdhci_device;
extern struct platform_device spear13xx_smi_device;
extern struct platform_device spear13xx_udc_device;
extern struct platform_device spear13xx_wdt_device;
extern struct sys_timer spear13xx_timer;

/* Add spear13xx structure declarations here */
extern struct db9000fb_mach_info sharp_lcd_info;
extern struct db9000fb_mach_info chimei_b101aw02_info;

/* Add spear13xx family function declarations here */
void __init spear13xx_clk_init(void);
void __init i2c_register_default_devices(void);
void __init spear_setup_timer(void);
void __init spear13xx_map_io(void);
void __init spear13xx_init_irq(void);
void __init spear13xx_init(void);
void __init nand_mach_init(u32 busw);
void spear13xx_secondary_startup(void);
void pcm_init(struct device *dma_dev);
unsigned long reserve_mem(struct meminfo *mi, unsigned long size);

/* spear1300 declarations */
#ifdef CONFIG_MACH_SPEAR1300
/* Add spear1300 machine function declarations here */
void __init spear1300_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_MACH_SPEAR1300 */

/* spear1310 declarations */
#ifdef CONFIG_MACH_SPEAR1310
/* Add spear1310 machine device structure declarations here */
extern struct amba_device spear1310_uart1_device;
extern struct amba_device spear1310_uart2_device;
extern struct amba_device spear1310_uart3_device;
extern struct amba_device spear1310_uart4_device;
extern struct amba_device spear1310_uart5_device;
extern struct platform_device spear1310_can0_device;
extern struct platform_device spear1310_can1_device;
extern struct platform_device spear1310_i2c1_device;
extern struct platform_device spear1310_ras_fsmc_nor_device;
extern struct platform_device spear1310_plgpio_device;
extern struct platform_device spear1310_rs485_0_device;
extern struct platform_device spear1310_rs485_1_device;
extern struct platform_device spear1310_tdm_hdlc_0_device;
extern struct platform_device spear1310_tdm_hdlc_1_device;

/* Add spear1310 machine function declarations here */
void __init spear1310_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);
void __init spear1310_map_io(void);

#endif /* CONFIG_MACH_SPEAR1310 */

/* spear900 declarations */
#ifdef CONFIG_MACH_SPEAR900
/* Add spear900 machine function declarations here */
void __init spear900_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_MACH_SPEAR900 */

#endif /* __MACH_GENERIC_H */
