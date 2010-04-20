/*
 * arch/arm/mach-spear13xx/include/mach/irqs.h
 *
 * IRQ helper macros for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_IRQS_H
#define __MACH_IRQS_H

/* IRQ definitions */
/*
 * SGI : ID0 - ID15
 * PPI : ID16 - ID31
 * SHPI : ID32 - ID224
 */

#define IRQ_LOCALTIMER		29
#define IRQ_LOCALWDOG		30

/* Shared Peripheral Interrupt (SHPI) */
#define IRQ_SHPI_START		32

#define IRQ_ADC			(IRQ_SHPI_START + 12)
#define IRQ_FSMC0		(IRQ_SHPI_START + 20)
#define IRQ_FSMC1		(IRQ_SHPI_START + 21)
#define IRQ_FSMC2		(IRQ_SHPI_START + 22)
#define IRQ_FSMC3		(IRQ_SHPI_START + 23)
#define IRQ_GPIOA		(IRQ_SHPI_START + 24)
#define IRQ_GPIOB		(IRQ_SHPI_START + 25)
#define IRQ_MMC			(IRQ_SHPI_START + 28)
#define IRQ_CF			(IRQ_SHPI_START + 29)
#define IRQ_SMI			(IRQ_SHPI_START + 30)
#define IRQ_SSP			(IRQ_SHPI_START + 31)
#define IRQ_C3			(IRQ_SHPI_START + 32)
#define IRQ_UART		(IRQ_SHPI_START + 35)
#define IRQ_RTC			(IRQ_SHPI_START + 36)
#define IRQ_GPT0_TMR0		(IRQ_SHPI_START + 37)
#define IRQ_GPT0_TMR1		(IRQ_SHPI_START + 38)
#define IRQ_GPT1_TMR0		(IRQ_SHPI_START + 39)
#define IRQ_GPT1_TMR1		(IRQ_SHPI_START + 40)
#define IRQ_I2C			(IRQ_SHPI_START + 41)
#define IRQ_GPT2_TMR0		(IRQ_SHPI_START + 42)
#define IRQ_GPT2_TMR1		(IRQ_SHPI_START + 43)
#define IRQ_GPT3_TMR0		(IRQ_SHPI_START + 44)
#define IRQ_GPT3_TMR1		(IRQ_SHPI_START + 45)
#define IRQ_JPEG		(IRQ_SHPI_START + 50)
#define IRQ_KBD			(IRQ_SHPI_START + 52)

#define IRQ_PLAY_I2S1		(IRQ_SHPI_START + 26)
#define IRQ_REC_I2S1		(IRQ_SHPI_START + 53)
#define IRQ_PLAY_I2S2		(IRQ_SHPI_START + 10)
#define IRQ_REC_I2S2		(IRQ_SHPI_START + 11)

#define IRQ_DMAC		(IRQ_SHPI_START + 59)
#define IRQ_UDC			(IRQ_SHPI_START + 62)
#define IRQ_UPD			(IRQ_SHPI_START + 63)
#define IRQ_USBH_EHCI0		(IRQ_SHPI_START + 64)
#define IRQ_USBH_OHCI0		(IRQ_SHPI_START + 65)
#define IRQ_USBH_EHCI1		(IRQ_SHPI_START + 66)
#define IRQ_USBH_OHCI1		(IRQ_SHPI_START + 67)
#define IRQ_PCIE1		(IRQ_SHPI_START + 68)
#define IRQ_PCIE2		(IRQ_SHPI_START + 69)
#define IRQ_PCIE3		(IRQ_SHPI_START + 70)

#define IRQ_GIC_END		(IRQ_SHPI_START + 128)

#define NR_IRQS 		IRQ_GIC_END

#endif /* __MACH_IRQS_H */
