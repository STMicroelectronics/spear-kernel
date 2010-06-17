#ifndef __DB9000_REGS_H
#define __DB9000_REGS_H
/*
 * linux/drivers/video/db9000fb.h
 *    -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 *  Copyright (C) 2010 Digital Blocks, Inc.
 *  2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/byteorder/little_endian.h>
#include <mach/bitfield.h>

/*
 * LCD Controller Registers and Bits Definitions
 */
#define DB9000_CR1		(0x000)	/* LCD Controller Control Register 1 */
#define DB9000_HTR		(0x008)	/* Horizontal Timing Register */
#define DB9000_VTR1		(0x00C)	/* Vertical Timing Register 1 */
#define DB9000_VTR2		(0x010)	/* Vertical Timing Register 2 */
#define DB9000_PCTR		(0x014)	/* Pixel Clock Timing Register */
#define DB9000_ISR		(0x018)	/* Interrupt Status Register */
#define DB9000_IMR		(0x01C)	/* Interrupt Mask Register */
#define DB9000_IVR		(0x020)	/* Interrupt Vector Register */
#define DB9000_ISCR		(0x024)	/* Interrupt Scan Compare Register */
#define DB9000_DBAR		(0x028)	/* DMA Base Address Register */
#define DB9000_DCAR		(0x02C)	/* DMA Current Address Register */
#define DB9000_DEAR		(0x030)	/* DMA End Address Register */
#define DB9000_PWMFR	(0x034)	/* PWM Frequency Register */
#define DB9000_PWMDCR	(0x038)	/* PWM Duty Cycle Register */
#define DB9000_DFBAR	(0x03C)	/* DMA Frame Descriptor Branch Address Register */
#define DB9000_DFLAR	(0x040)	/* DMA Frame Descriptor Last Address Register */
#define DB9000_CIR		(0x1FC)	/* Core Identification Register */
#define DB9000_PALT		(0x200)	/* Palette Data Words */
#define DB9000_PALT_2P	(0x400)	/* Palette Data Words - 2nd Port/2 Port TFT */

/* Overlay Window Registers  */
#define DB9000_NUM_OW	(16)
#define DB9000_OWER		(0x600)	/* Overlay Window Enable Register */

/* Overlay Window 0 Registers  */
#define DB9000_OWXSER0		(0x604)	/* Overlay Window X Start/End Register 0 */
#define DB9000_OWYSER0		(0x608)	/* Overlay Window Y Start/End Register 0 */
#define DB9000_OWDBAR0		(0x60C)	/* Overlay Window DMA BAR 0 */
#define DB9000_OWDCAR0		(0x610)	/* Overlay Window DMA Current Addr Register 0 */

/* Overlay Window 1 Registers  */
#define DB9000_OWXSER1		(0x614)	/* Overlay Window X Start/End Register 1 */
#define DB9000_OWYSER1		(0x618)	/* Overlay Window Y Start/End Register 1 */
#define DB9000_OWDBAR1		(0x61C)	/* Overlay Window DMA BAR 1 */
#define DB9000_OWDCAR1		(0x620)	/* Overlay Window DMA Current Addr Register 1 */

/* Overlay Window 2 Registers  */
#define DB9000_OWXSER2		(0x624)	/* Overlay Window X Start/End Register 2 */
#define DB9000_OWYSER2		(0x628)	/* Overlay Window Y Start/End Register 2 */
#define DB9000_OWDBAR2		(0x62C)	/* Overlay Window DMA BAR 2 */
#define DB9000_OWDCAR2		(0x630)	/* Overlay Window DMA Current Addr Register 2 */

/* Overlay Window 3 Registers  */
#define DB9000_OWXSER3		(0x634)	/* Overlay Window X Start/End Register 3 */
#define DB9000_OWYSER3		(0x638)	/* Overlay Window Y Start/End Register 3 */
#define DB9000_OWDBAR3		(0x63C)	/* Overlay Window DMA BAR 3 */
#define DB9000_OWDCAR3		(0x640)	/* Overlay Window DMA Current Addr Register 3 */

/* Overlay Window 4 Registers  */
#define DB9000_OWXSER4		(0x644)	/* Overlay Window X Start/End Register 4 */
#define DB9000_OWYSER4		(0x648)	/* Overlay Window Y Start/End Register 4 */
#define DB9000_OWDBAR4		(0x64C)	/* Overlay Window DMA BAR 4 */
#define DB9000_OWDCAR4		(0x650)	/* Overlay Window DMA Current Addr Register 4 */

/* Overlay Window 5 Registers  */
#define DB9000_OWXSER5		(0x654)	/* Overlay Window X Start/End Register 5 */
#define DB9000_OWYSER5		(0x658)	/* Overlay Window Y Start/End Register 5 */
#define DB9000_OWDBAR5		(0x65C)	/* Overlay Window DMA BAR 5 */
#define DB9000_OWDCAR5		(0x660)	/* Overlay Window DMA Current Addr Register 5 */

/* Overlay Window 6 Registers  */
#define DB9000_OWXSER6		(0x664)	/* Overlay Window X Start/End Register 6 */
#define DB9000_OWYSER6		(0x668)	/* Overlay Window Y Start/End Register 6 */
#define DB9000_OWDBAR6		(0x66C)	/* Overlay Window DMA BAR 6 */
#define DB9000_OWDCAR6		(0x670)	/* Overlay Window DMA Current Addr Register 6 */

/* Overlay Window 7 Registers  */
#define DB9000_OWXSER7		(0x674)	/* Overlay Window X Start/End Register 7 */
#define DB9000_OWYSER7		(0x678)	/* Overlay Window Y Start/End Register 7 */
#define DB9000_OWDBAR7		(0x67C)	/* Overlay Window DMA BAR 7 */
#define DB9000_OWDCAR7		(0x680)	/* Overlay Window DMA Current Addr Register 7 */

/* Overlay Window 8 Registers  */
#define DB9000_OWXSER8		(0x684)	/* Overlay Window X Start/End Register 8 */
#define DB9000_OWYSER8		(0x688)	/* Overlay Window Y Start/End Register 8 */
#define DB9000_OWDBAR8		(0x68C)	/* Overlay Window DMA BAR 8 */
#define DB9000_OWDCAR8		(0x690)	/* Overlay Window DMA Current Addr Register 8 */

/* Overlay Window 9 Registers  */
#define DB9000_OWXSER9		(0x694)	/* Overlay Window X Start/End Register 9 */
#define DB9000_OWYSER9		(0x698)	/* Overlay Window Y Start/End Register 9 */
#define DB9000_OWDBAR9		(0x69C)	/* Overlay Window DMA BAR 9 */
#define DB9000_OWDCAR9		(0x6A0)	/* Overlay Window DMA Current Addr Register 9 */

/* Overlay Window 10 Registers  */
#define DB9000_OWXSER10		(0x6A4)	/* Overlay Window X Start/End Register 10 */
#define DB9000_OWYSER10		(0x6A8)	/* Overlay Window Y Start/End Register 10 */
#define DB9000_OWDBAR10		(0x6AC)	/* Overlay Window DMA BAR 10 */
#define DB9000_OWDCAR10		(0x6B0)	/* Overlay Window DMA Current Addr Register 10 */

/* Overlay Window 11 Registers  */
#define DB9000_OWXSER11		(0x6B4)	/* Overlay Window X Start/End Register 11 */
#define DB9000_OWYSER11		(0x6B8)	/* Overlay Window Y Start/End Register 11 */
#define DB9000_OWDBAR11		(0x6BC)	/* Overlay Window DMA BAR 11 */
#define DB9000_OWDCAR11		(0x6C0)	/* Overlay Window DMA Current Addr Register 11 */

/* Overlay Window 12 Registers  */
#define DB9000_OWXSER12		(0x6C4)	/* Overlay Window X Start/End Register 12 */
#define DB9000_OWYSER12		(0x6C8)	/* Overlay Window Y Start/End Register 12 */
#define DB9000_OWDBAR12		(0x6CC)	/* Overlay Window DMA BAR 12 */
#define DB9000_OWDCAR12		(0x6D0)	/* Overlay Window DMA Current Addr Register 12 */

/* Overlay Window 13 Registers  */
#define DB9000_OWXSER13		(0x6D4)	/* Overlay Window X Start/End Register 13 */
#define DB9000_OWYSER13		(0x6D8)	/* Overlay Window Y Start/End Register 13 */
#define DB9000_OWDBAR13		(0x6DC)	/* Overlay Window DMA BAR 13 */
#define DB9000_OWDCAR13		(0x6E0)	/* Overlay Window DMA Current Addr Register 13 */

/* Overlay Window 14 Registers  */
#define DB9000_OWXSER14		(0x6E4)	/* Overlay Window X Start/End Register 14 */
#define DB9000_OWYSER14		(0x6E8)	/* Overlay Window Y Start/End Register 14 */
#define DB9000_OWDBAR14		(0x6EC)	/* Overlay Window DMA BAR 14 */
#define DB9000_OWDCAR14		(0x6F0)	/* Overlay Window DMA Current Addr Register 14 */

/* Overlay Window 15 Registers  */
#define DB9000_OWXSER15		(0x6F4)	/* Overlay Window X Start/End Register 15 */
#define DB9000_OWYSER15		(0x6F8)	/* Overlay Window Y Start/End Register 15 */
#define DB9000_OWDBAR15		(0x6FC)	/* Overlay Window DMA BAR 15 */
#define DB9000_OWDCAR15		(0x700)	/* Overlay Window DMA Current Addr Register 15 */

/* Control Register 1, Offset 0x000 */
#define DB9000_CR1_ENB		(1 << 0)	/* LCD Controller Enable */
#define DB9000_CR1_LPE		(1 << 1)	/* LCD Power Enable */
#define DB9000_CR1_BPP(x)	(((x) & 0x7) << 2) /* LCD Bits per Pixel */
#define DB9000_CR1_RGB		(1 << 5)	/* RGB or BGR Format */
#define DB9000_CR1_EPO		(1 << 6)	/* Big or Little Endian Pixel Ordering */
#define DB9000_CR1_EBO		(1 << 7)	/* Big or Little Endian Byte Ordering  */
#define DB9000_CR1_DEP		(1 << 8)	/* Data Enable Polarity */
#define DB9000_CR1_PCP		(1 << 9)	/* Pixel Clock Polarity */
#define DB9000_CR1_HSP		(1 << 10)	/* Horizontal Sync Polarity */
#define DB9000_CR1_VSP		(1 << 11)	/* Vertical Sync Polarity */
#define DB9000_CR1_OPS(x)	(((x) & 0x7) << 12)	/* Output Pixel Select */
#define DB9000_CR1_PSS		(1 << 15)	/* Palette Load Source */
#define DB9000_CR1_FDW(x)	(((x) & 0x3) << 16)	/* FIFO DMA Request Words */
#define DB9000_CR1_LPS		(1 << 18)	/* LCD 1 or Port Select */
#define DB9000_CR1_FBP		(1 << 19)	/* Frame Buffer 24bpp Packed Word */
#define DB9000_CR1_DEE		(1 << 20)	/* DMA End Address Enable */
#define DB9000_CR1_DFR		(1 << 21)	/* DMA Frame Descriptor Re-start */
#define DB9000_CR1_DFB		(1 << 22)	/* DMA Frame Descriptor Branch Address Enable */
#define DB9000_CR1_DFE		(1 << 23)	/* DMA Frame Descriptor Enable */

#define DB9000_CR1_BPP_1bpp		(0)	/* 1 bit per pixel */
#define DB9000_CR1_BPP_2bpp		(1)	/* 2 bits per pixel */
#define DB9000_CR1_BPP_4bpp		(2)	/* 4 bits per pixel */
#define DB9000_CR1_BPP_8bpp		(3)	/* 8 bits per pixel */
#define DB9000_CR1_BPP_16bpp	(4)	/* 16 bits per pixel */
#define DB9000_CR1_BPP_18bpp	(5)	/* 18 bits per pixel */
#define DB9000_CR1_BPP_24bpp	(6)	/* 14 bits per pixel */

#define DB9000_CR1_PixRsEdg	(DB9000_CR1_PCP*0)	/*  Pixel clock Rising-Edge */
#define DB9000_CR1_PixFlEdg	(DB9000_CR1_PCP*1)	/*  Pixel clock Falling-Edge */

/* Horizontal Timing Register, Offset 0x008 */
#define DB9000_HTR_HFP(x)		(((x) & 0xff) << 0)  /* Horizontal Front Porch */
#define DB9000_HTR_PPL(x)		(((x) & 0xff) << 8)  /* Pixels per Line */
#define DB9000_HTR_HBP(x)		(((x) & 0xff) << 16) /* Horizontal Back Porch */
#define DB9000_HTR_HSW(x)		(((x) & 0xff) << 24) /* Horizontal Sync Width */

/* Vertical Timing Register 1, Offset 0x00C */
#define DB9000_VTR1_VSW(x)		(((x) & 0xff) << 0)  /* Vertical Sync Width */
#define DB9000_VTR1_VFP(x)		(((x) & 0xff) << 8)  /* Vertical Front Porch */
#define DB9000_VTR1_VBP(x)		(((x) & 0xff) << 16) /* Vertical Back Porch */

/* Vertical Timing Register 2, Offset 0x010 */
#define DB9000_VTR2_LPP(x)		(((x) & 0xfff) << 0)  /* Lines Per Panel */

/* Pixel Clock Timing Register, Offset 0x014 */
#define DB9000_PCTR_PCD(x)		(((x) & 0xff) << 0) /* Pixel Clock Divider */
#define DB9000_PCTR_PCB(x)		(((x) & 0x01) << 8)			/* Pixel Clock Divider Bypass */
#define DB9000_PCTR_PCI(x)		(((x) & 0x01) << 9)			/* Pixel Clock Input Select */

/* Interrupt Status Register, Offset 0x018 */
#define DB9000_ISR_OFU		(1 << 0)  /* Output FIFO Underrun */
#define DB9000_ISR_OFO		(1 << 1)  /* Output FIFO Overrun */
#define DB9000_ISR_IFU		(1 << 2)  /* Input FIFO Underrun */
#define DB9000_ISR_IFO		(1 << 3)  /* Input FIFO Overrun */
#define DB9000_ISR_FER		(1 << 4)  /* OR of OFU, OFO, IFU, IFO */
#define DB9000_ISR_MBE		(1 << 5)  /* Master Bus Error */
#define DB9000_ISR_VCT		(1 << 6)  /* Vertical Compare Triggered */
#define DB9000_ISR_BAU		(1 << 7)  /* DMA Base Address Register Update to CAR */
#define DB9000_ISR_LDD		(1 << 8)  /* LCD Controller Disable Done */
// #ifdef CONFIG_AXI_BUS
#define DB9000_ISR_ABL		(1 << 9)  /* AXI Master - Read Burst Length Error */
#define DB9000_ISR_ARI		(1 << 10) /* AXI Master - Return ID Error */
#define DB9000_ISR_ARS		(1 << 11) /* AXI Master - Response Signal Error */
// #endif
#define DB9000_ISR_FBE		(1 << 12) /* Frame Descriptor - Bus Error */
#define DB9000_ISR_FNC		(1 << 13) /* Frame Descriptor - Node Complete */
#define DB9000_ISR_FLC		(1 << 14) /* Frame Descriptor - List Complete */

/* Interrupt Mask Register, Offset 0x01C */
#define DB9000_ISR_OFUM		(1 << 0)  /* Output FIFO Underrun - Mask */
#define DB9000_ISR_OFOM		(1 << 1)  /* Output FIFO Overrun - Mask */
#define DB9000_ISR_IFUM		(1 << 2)  /* Input FIFO Underrun - Mask */
#define DB9000_ISR_IFOM		(1 << 3)  /* Input FIFO Overrun - Mask */
#define DB9000_ISR_FERM		(1 << 4)  /* OR of OFU, OFO, IFU, IFO - Mask */
#define DB9000_ISR_MBEM		(1 << 5)  /* Master Bus Error - Mask */
#define DB9000_ISR_VCTM		(1 << 6)  /* Vertical Compare Triggered - Mask */
#define DB9000_ISR_BAUM		(1 << 7)  /* DMA Base Address Register Update to CAR - Mask */
#define DB9000_ISR_LDDM		(1 << 8)  /* LCD Controller Disable Done - Mask */
// #ifdef CONFIG_AXI_BUS
#define DB9000_ISR_ABLM		(1 << 9)  /* AXI Master - Read Burst Length Error - Mask */
#define DB9000_ISR_ARIM		(1 << 10) /* AXI Master - Return ID Error - Mask */
#define DB9000_ISR_ARSM		(1 << 11) /* AXI Master - Response Signal Error - Mask */
// #endif
#define DB9000_ISR_FBEM		(1 << 12) /* Frame Descriptor - Bus Error - Mask */
#define DB9000_ISR_FNCM		(1 << 13) /* Frame Descriptor - Node Complete - Mask */
#define DB9000_ISR_FLCM		(1 << 14) /* Frame Descriptor - List Complete - Mask */

/* Interrupt Vector Register, Offset 0x020 */
#define DB9000_ISR_OFUV		(1 << 0)  /* OFU & OFUM */
#define DB9000_ISR_OFOV		(1 << 1)  /* OFO & OFOM */
#define DB9000_ISR_IFUV		(1 << 2)  /* IFU & IFUM */
#define DB9000_ISR_IFOV		(1 << 3)  /* IFO & IFOM */
#define DB9000_ISR_FERV		(1 << 4)  /* FER & FERM */
#define DB9000_ISR_MBEV		(1 << 5)  /* MBE & MBEM */
#define DB9000_ISR_VCTV		(1 << 6)  /* VCT & VCTM */
#define DB9000_ISR_BAUV		(1 << 7)  /* BAU & BAUM */
#define DB9000_ISR_LDDV		(1 << 8)  /* LDD & LDDM */
// #ifdef CONFIG_AXI_BUS
#define DB9000_ISR_ABLV		(1 << 9)  /* ABL & ABLM */
#define DB9000_ISR_ARIV		(1 << 10) /* ARI & ARIM */
#define DB9000_ISR_ARSV		(1 << 11) /* ARS & ARSM */
// #endif
#define DB9000_ISR_FBEV		(1 << 12) /* FBE & FBEM */
#define DB9000_ISR_FNCV		(1 << 13) /* FNC & FNCM */
#define DB9000_ISR_FLCV		(1 << 14) /* FLC & FLCM */

/* Interrupt Scan Compare Register, Offset 0x024 */
#define DB9000_ISCR_VSC(x)		((x) & 0x7)

/* PWM Frequency Register, Offset 0x034 */
#define DB9000_PWMFR_PWM_FCD(x)	(((x) & 0xfffff) << 0)
#define DB9000_PWMFR_PWM_FCE	(1 << 20)
#define DB9000_PWMFR_PWM_FCI	(1 << 21)

/* PWM Duty Cycle Register, Offset 0x038 */
#define DB9000_PWMDCR_DCR(x)	((x) & 0xff)

#endif /* __DB9000_REGS_H */
