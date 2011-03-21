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
#define SPEAR13XX_PCM_CFG_BASE		UL(0xE0700100)

#define SPEAR13XX_PMX_CFG0		UL(0xE0700650)
#define SPEAR13XX_PMX_CFG1		UL(0xE0700654)
#define SPEAR13XX_PMX_CFG2		UL(0xE0700658)
#define SPEAR13XX_PMX_CFG3		UL(0xE070065C)

#if defined(CONFIG_CPU_SPEAR1310)
#define SPEAR1310_FUNC_CNTL_0		UL(0x6C800000)

#define SPEAR1310_PMX_SMII_MASK		(1 << 24)	/* Func cntl reg0 */
#define SPEAR1310_PMX_EGPIO7_MASK	(1 << 2)	/* Pcm cfg reg */
#endif

/* pad mux declarations */
#define SPEAR13XX_PMX_I2S1_MASK		(1 << 3)
#define SPEAR13XX_PMX_I2S2_MASK		(1 << 16)	/* Offset 4 */
#define SPEAR13XX_PMX_CLCD1_MASK	(1 << 5)
#define SPEAR13XX_PMX_CLCD2_MASK	(1 << 3)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO00_MASK	(1 << 6)
#define SPEAR13XX_PMX_EGPIO01_MASK	(1 << 7)
#define SPEAR13XX_PMX_EGPIO02_MASK	(1 << 8)
#define SPEAR13XX_PMX_EGPIO03_MASK	(1 << 9)
#define SPEAR13XX_PMX_EGPIO04_MASK	(1 << 10)
#define SPEAR13XX_PMX_EGPIO05_MASK	(1 << 11)
#define SPEAR13XX_PMX_EGPIO06_MASK	(1 << 12)
#define SPEAR13XX_PMX_EGPIO07_MASK	(1 << 13)
#define SPEAR13XX_PMX_EGPIO08_MASK	(1 << 14)
#define SPEAR13XX_PMX_EGPIO09_MASK	(1 << 15)
#define SPEAR13XX_PMX_EGPIO10_MASK	(1 << 5)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO11_MASK	(1 << 6)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO12_MASK	(1 << 7)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO13_MASK	(1 << 8)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO14_MASK	(1 << 9)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO15_MASK	(1 << 10)	/* Offset 4 */
#define SPEAR13XX_PMX_EGPIO_0_GRP_MASK (SPEAR13XX_PMX_EGPIO00_MASK | \
		SPEAR13XX_PMX_EGPIO01_MASK | SPEAR13XX_PMX_EGPIO02_MASK | \
		SPEAR13XX_PMX_EGPIO03_MASK | SPEAR13XX_PMX_EGPIO04_MASK | \
		SPEAR13XX_PMX_EGPIO05_MASK | SPEAR13XX_PMX_EGPIO06_MASK | \
		SPEAR13XX_PMX_EGPIO07_MASK | SPEAR13XX_PMX_EGPIO08_MASK | \
		SPEAR13XX_PMX_EGPIO09_MASK)
#define SPEAR13XX_PMX_EGPIO_1_GRP_MASK (SPEAR13XX_PMX_EGPIO10_MASK | \
		SPEAR13XX_PMX_EGPIO11_MASK | SPEAR13XX_PMX_EGPIO12_MASK | \
		SPEAR13XX_PMX_EGPIO13_MASK | SPEAR13XX_PMX_EGPIO14_MASK | \
		SPEAR13XX_PMX_EGPIO15_MASK)

#define SPEAR13XX_PMX_SMI_MASK		(1 << 16)
#define SPEAR13XX_PMX_SMINCS2_MASK	(1 << 1)	/* Offset 4 */
#define SPEAR13XX_PMX_SMINCS3_MASK	(1 << 2)	/* Offset 4 */

#define SPEAR13XX_PMX_GMIICLK_MASK			(1 << 18)
#define SPEAR13XX_PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK	(1 << 19)
#define SPEAR13XX_PMX_RXCLK_RDV_TXEN_D03_MASK		(1 << 20)
#define SPEAR13XX_PMX_GMIID47_MASK			(1 << 21)
#define SPEAR13XX_PMX_MDC_MDIO_MASK			(1 << 22)

#define SPEAR13XX_PMX_GMII_MASK (SPEAR13XX_PMX_GMIICLK_MASK | \
		SPEAR13XX_PMX_GMIICOL_CRS_XFERER_MIITXCLK_MASK | \
		SPEAR13XX_PMX_RXCLK_RDV_TXEN_D03_MASK | \
		SPEAR13XX_PMX_GMIID47_MASK | SPEAR13XX_PMX_MDC_MDIO_MASK)

#define SPEAR13XX_PMX_NAND8_MASK	(1 << 17)
#define SPEAR13XX_PMX_NFAD023_MASK	(1 << 24)
#define SPEAR13XX_PMX_NFAD24_MASK	(1 << 25)
#define SPEAR13XX_PMX_NFAD25_MASK	(1 << 26)
#define SPEAR13XX_PMX_NFWPRT1_MASK	(1 << 24)	/* Offset 4 */
#define SPEAR13XX_PMX_NFWPRT2_MASK	(1 << 26)	/* Offset 4 */
#define SPEAR13XX_PMX_NFWPRT3_MASK	(1 << 28)
#define SPEAR13XX_PMX_NFRSTPWDWN0_MASK	(1 << 29)
#define SPEAR13XX_PMX_NFRSTPWDWN1_MASK	(1 << 30)
#define SPEAR13XX_PMX_NFRSTPWDWN2_MASK	(1 << 31)
#define SPEAR13XX_PMX_NFRSTPWDWN3_MASK	(1 << 0)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE1_MASK	(1 << 20)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE2_MASK	(1 << 22)	/* Offset 4 */
#define SPEAR13XX_PMX_NFCE3_MASK	(1 << 27)
#define SPEAR13XX_PMX_NFIO815_MASK	(1 << 18)	/* Offset 4 */

#define SPEAR13XX_PMX_NAND8BIT_0_MASK (SPEAR13XX_PMX_NAND8_MASK | \
		SPEAR13XX_PMX_NFAD023_MASK | SPEAR13XX_PMX_NFAD24_MASK | \
		SPEAR13XX_PMX_NFAD25_MASK | SPEAR13XX_PMX_NFWPRT3_MASK | \
		SPEAR13XX_PMX_NFRSTPWDWN0_MASK | SPEAR13XX_PMX_NFRSTPWDWN1_MASK\
		| SPEAR13XX_PMX_NFRSTPWDWN2_MASK | SPEAR13XX_PMX_NFCE3_MASK)
#define SPEAR13XX_PMX_NAND8BIT_1_MASK	(SPEAR13XX_PMX_NFRSTPWDWN3_MASK)

#define SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK	SPEAR13XX_PMX_NAND8BIT_0_MASK
#define SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK	(SPEAR13XX_PMX_NAND8BIT_1_MASK\
		| SPEAR13XX_PMX_NFCE1_MASK | SPEAR13XX_PMX_NFCE2_MASK | \
		SPEAR13XX_PMX_NFWPRT1_MASK | SPEAR13XX_PMX_NFWPRT2_MASK)

#define SPEAR13XX_PMX_NAND16BIT_0_MASK	(SPEAR13XX_PMX_NAND8BIT_0_MASK)
#define SPEAR13XX_PMX_NAND16BIT_1_MASK	(SPEAR13XX_PMX_NAND8BIT_1_MASK | \
		SPEAR13XX_PMX_NFIO815_MASK)
#define SPEAR13XX_PMX_NAND16BIT4DEV_0_MASK SPEAR13XX_PMX_NAND8BIT4DEV_0_MASK
#define SPEAR13XX_PMX_NAND16BIT4DEV_1_MASK	\
	(SPEAR13XX_PMX_NAND8BIT4DEV_1_MASK | SPEAR13XX_PMX_NFIO815_MASK)

#define SPEAR13XX_PMX_KBD_ROW0_MASK	(1 << 25)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROW1_MASK	(1 << 23)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROWCOL25_MASK	(1 << 17)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_ROWCOL68_MASK	(1 << 4)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_COL0_MASK	(1 << 21)	/* Offset 4 */
#define SPEAR13XX_PMX_KBD_COL1_MASK	(1 << 19)	/* Offset 4 */
#define SPEAR13XX_PMX_KEYBOARD_6X6_MASK	(SPEAR13XX_PMX_KBD_ROW0_MASK | \
		SPEAR13XX_PMX_KBD_ROW1_MASK | SPEAR13XX_PMX_KBD_ROWCOL25_MASK \
		| SPEAR13XX_PMX_KBD_COL0_MASK | SPEAR13XX_PMX_KBD_COL1_MASK)

#define SPEAR13XX_PMX_UART0_MASK	(1 << 1)
#define SPEAR13XX_PMX_I2C_MASK		(1 << 2)
#define SPEAR13XX_PMX_SSP_MASK		(1 << 4)
#define SPEAR13XX_PMX_UART0_MODEM_MASK	(1 << 11)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT0_TMR1_MASK	(1 << 12)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT0_TMR2_MASK	(1 << 13)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT1_TMR1_MASK	(1 << 14)	/* Offset 4 */
#define SPEAR13XX_PMX_GPT1_TMR2_MASK	(1 << 15)	/* Offset 4 */

#define SPEAR13XX_PMX_MCIDATA0_MASK	(1 << 27)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA1_MASK	(1 << 28)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA2_MASK	(1 << 29)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA3_MASK	(1 << 30)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA4_MASK	(1 << 31)	/* Offset 4 */
#define SPEAR13XX_PMX_MCIDATA5_MASK	(1 << 0)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA6_MASK	(1 << 1)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA7_MASK	(1 << 2)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA1SD_MASK	(1 << 3)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA2SD_MASK	(1 << 4)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATA3SD_MASK	(1 << 5)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR0ALE_MASK	(1 << 6)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR1CLECLK_MASK (1 << 7)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIADDR2_MASK	(1 << 8)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICECF_MASK	(1 << 9)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICEXD_MASK	(1 << 10)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICESDMMC_MASK	(1 << 11)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDCF1_MASK	(1 << 12)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDCF2_MASK	(1 << 13)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDXD_MASK	(1 << 14)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICDSDMMC_MASK	(1 << 15)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDATADIR_MASK	(1 << 16)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDMARQWP_MASK	(1 << 17)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIORDRE_MASK	(1 << 18)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIOWRWE_MASK	(1 << 19)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIRESETCF_MASK	(1 << 20)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICS0CE_MASK	(1 << 21)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICFINTR_MASK	(1 << 22)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIIORDY_MASK	(1 << 23)	/* Offset 8 */
#define SPEAR13XX_PMX_MCICS1_MASK	(1 << 24)	/* Offset 8 */
#define SPEAR13XX_PMX_MCIDMAACK_MASK	(1 << 25)	/* Offset 8 */
#define SPEAR13XX_PMX_MCISDCMD_MASK	(1 << 26)	/* Offset 8 */
#define SPEAR13XX_PMX_MCILEDS_MASK	(1 << 27)	/* Offset 8 */

#define SPEAR13XX_PMX_MCIFALL_1_MASK	(0xF8000000)
#define SPEAR13XX_PMX_MCIFALL_2_MASK	(0x0FFFFFFF)

/* pad mux devices */
extern struct pmx_dev spear13xx_pmx_i2c;
extern struct pmx_dev spear13xx_pmx_ssp;
extern struct pmx_dev spear13xx_pmx_i2s1;
extern struct pmx_dev spear13xx_pmx_i2s2;
extern struct pmx_dev spear13xx_pmx_clcd;
extern struct pmx_dev spear13xx_pmx_clcd_hires;
extern struct pmx_dev spear13xx_pmx_egpio_grp;
extern struct pmx_dev spear13xx_pmx_smi_2_chips;
extern struct pmx_dev spear13xx_pmx_smi_4_chips;
extern struct pmx_dev spear13xx_pmx_gmii;
extern struct pmx_dev spear13xx_pmx_nand_8bit;
extern struct pmx_dev spear13xx_pmx_nand_16bit;
extern struct pmx_dev spear13xx_pmx_keyboard_6x6;
extern struct pmx_dev spear13xx_pmx_keyboard_9x9;
extern struct pmx_dev spear13xx_pmx_uart0;
extern struct pmx_dev spear13xx_pmx_uart0_modem;
extern struct pmx_dev spear13xx_pmx_gpt_0_1;
extern struct pmx_dev spear13xx_pmx_gpt_0_2;
extern struct pmx_dev spear13xx_pmx_gpt_1_1;
extern struct pmx_dev spear13xx_pmx_gpt_1_2;
extern struct pmx_dev spear13xx_pmx_mcif;
extern struct pmx_dev spear13xx_pmx_sdhci;
extern struct pmx_dev spear13xx_pmx_cf;
extern struct pmx_dev spear13xx_pmx_xd;

#if defined(CONFIG_CPU_SPEAR1310)
extern struct pmx_dev spear1310_pmx_uart1_modem;
extern struct pmx_dev spear1310_pmx_uart_1;
extern struct pmx_dev spear1310_pmx_uart_2;
extern struct pmx_dev spear1310_pmx_uart_3_4_5;
extern struct pmx_dev spear1310_pmx_rs485_hdlc_1_2;
extern struct pmx_dev spear1310_pmx_tdm_hdlc_1_2;
extern struct pmx_dev spear1310_pmx_nand32bit;
extern struct pmx_dev spear1310_pmx_fsmc16bit_4_chips;
extern struct pmx_dev spear1310_pmx_fsmc32bit_4_chips;
extern struct pmx_dev spear1310_pmx_gmii1;
extern struct pmx_dev spear1310_pmx_rgmii;
extern struct pmx_dev spear1310_pmx_i2c1;
extern struct pmx_dev spear1310_pmx_smii_0_1_2;
extern struct pmx_dev spear1310_pmx_can;
extern struct pmx_dev spear1310_pmx_uart1_modem;
#endif

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	SPEAR13XX_IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	SPEAR13XX_IRQ_GPT0_TMR1

/* Add spear13xx family device structure declarations here */
extern struct amba_device spear13xx_gpio_device[];
extern struct amba_device spear13xx_ssp_device;
extern struct amba_device spear13xx_uart_device;
extern struct platform_device spear13xx_adc_device;
extern struct platform_device spear13xx_cf_device;
extern struct platform_device spear13xx_db9000_clcd_device;
extern struct platform_device spear13xx_dmac_device[];
extern struct platform_device spear13xx_ehci0_device;
extern struct platform_device spear13xx_ehci1_device;
extern struct platform_device spear13xx_eth_device;
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
extern struct dw_dma_slave cf_dma_priv;

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
#ifdef CONFIG_CPU_SPEAR1300
/* Add spear1300 machine function declarations here */
void __init spear1300_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_CPU_SPEAR1300 */

/* spear1310 declarations */
#ifdef CONFIG_CPU_SPEAR1310
/* Add spear1310 machine device structure declarations here */
extern struct amba_device spear1310_uart1_device;
extern struct amba_device spear1310_uart2_device;
extern struct amba_device spear1310_uart3_device;
extern struct amba_device spear1310_uart4_device;
extern struct amba_device spear1310_uart5_device;
extern struct platform_device spear1310_can0_device;
extern struct platform_device spear1310_can1_device;
extern struct platform_device spear1310_eth1_device;
extern struct platform_device spear1310_eth2_device;
extern struct platform_device spear1310_eth3_device;
extern struct platform_device spear1310_eth4_device;
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

#endif /* CONFIG_CPU_SPEAR1310 */

/* spear900 declarations */
#ifdef CONFIG_CPU_SPEAR900
/* Add spear900 machine function declarations here */
void __init spear900_init(struct pmx_mode *pmx_mode, struct pmx_dev **pmx_devs,
		u8 pmx_dev_count);

#endif /* CONFIG_CPU_SPEAR900 */

#endif /* __MACH_GENERIC_H */
