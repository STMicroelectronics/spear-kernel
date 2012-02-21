/*
 * sil9135a - Silicon Image SIL9135A HDMI Receiver
 *
 * Copyright 2011 ST Microelectronics. All rights reserved.
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _SIL9135A_
#define _SIL9135A_
/* Platform dependent definition */
struct sil9135a_platform_data {
	u8 ci2ca;
};

/* I2C Reg definitions for address 0x60 */
/* IDs and Revision */
#define SIL_9135A_VND_IDL	0
#define SIL_9135A_VND_IDH	1
#define SIL_9135A_DEV_IDL	2
#define SIL_9135A_DEV_IDH	3
#define SIL_9135A_DEV_REV	4
/* Software Reset */
#define SIL_9135A_SRST		5
/* System Status */
#define SIL_9135A_STATE		6
/* Software Reset */
#define SIL_9135A_SRST2		7
/* System Control */
#define SIL_9135A_SYS_CTRL1	8
	#define SIL_9135A_24_BIT_MODE(reg, x)	\
		(((reg) & (~((u8)0x1) << 2)) | (x << 2))
	#define SIL_9135A_OCLKINV(reg, x)	\
		(((reg) & (~((u8)0x1) << 1)) | (x << 1))
/* ID Port Switch */
#define SIL_9135A_SYS_SWTCHC	9
	#define SIL_9135A_SELECT_HDMI(reg, x)	\
		(((reg) & (~((u8)0x33) << 0)) | (0x11 << x))
/* PCLK Auto Stop Register */
#define SIL_9135A_PCLK_CNT_MAX	0xA
/* HDCP Shadow BKSV */
#define SIL_9135A_HDCP_BKSV1	0x1A
#define SIL_9135A_HDCP_BKSV2	0x1B
#define SIL_9135A_HDCP_BKSV3	0x1C
#define SIL_9135A_HDCP_BKSV4	0x1D
#define SIL_9135A_HDCP_BKSV5	0x1E
/* HDCP Shadow Ri */
#define SIL_9135A_HDCP_RI1	0x1F
#define SIL_9135A_HDCP_RI2	0x20
/* HDCP Shadow AKSV */
#define SIL_9135A_HDCP_AKSV1	0x21
#define SIL_9135A_HDCP_AKSV2	0x22
#define SIL_9135A_HDCP_AKSV3	0x23
#define SIL_9135A_HDCP_AKSV4	0x24
#define SIL_9135A_HDCP_AKSV5	0x25
/* HDCP Shadow AN */
#define SIL_9135A_HDCP_AN1	0x26
#define SIL_9135A_HDCP_AN2	0x27
#define SIL_9135A_HDCP_AN3	0x28
#define SIL_9135A_HDCP_AN4	0x29
#define SIL_9135A_HDCP_AN5	0x2A
#define SIL_9135A_HDCP_AN6	0x2B
#define SIL_9135A_HDCP_AN7	0x2C
#define SIL_9135A_HDCP_AN8	0x2D
/* HDCP Debug */
#define SIL_9135A_HDCPCTRL	0x31
/* HDCP Status */
#define SIL_9135A_HDCP_STAT	0x32
/* Video In H Resolution */
#define SIL_9135A_H_RESL	0x3A
#define SIL_9135A_H_RESH	0x3B
/* Video In V Refresh */
#define SIL_9135A_V_RESL	0x3C
#define SIL_9135A_V_RESH	0x3D
/* Video Output Control */
#define SIL_9135A_VID_CTRL	0x48
/* Video Output Mode */
#define SIL_9135A_VID_MODE2	0x49
/* Video Output Mode */
#define SIL_9135A_VID_MODE1	0x4A
/* Video Output Digital Blank Value */
#define SIL_9135A_VID_BLANK1	0x4B
#define SIL_9135A_VID_BLANK2	0x4C
#define SIL_9135A_VID_BLANK3	0x4D
/* Video In DE Pixel Width */
#define SIL_9135A_DE_PIXL	0x4E
#define SIL_9135A_DE_PIXH	0x4F
/* Video In DE Line Height */
#define SIL_9135A_DE_LINL	0x50
#define SIL_9135A_DE_LINH	0x51
/* Video In VSYNC to Active Lines */
#define SIL_9135A_VID_VTAVL	0x52
/* Video In Vertical FP */
#define SIL_9135A_VID_VFP	0x53
/* Video Output Field2 BP */
#define SIL_9135A_VID_F2BPM	0x54
/* Video In Status */
#define SIL_9135A_VID_STAT	0x55
/* Video Channel Mapper */
#define SIL_9135A_VID_CH_MAP	0x56
/* Video Output Control */
#define SIL_9135A_VID_CTRL2	0x57
/* Video Output Horizontal FP */
#define SIL_9135A_VID_HFP1	0x59
#define SIL_9135A_VID_HFP2	0x5A
/* Video Output HSYNC Width */
#define SIL_9135A_VID_HSWIDL	0x5B
#define SIL_9135A_VID_HSWIDH	0x5C
/* Video Auto Format */
#define SIL_9135A_VID_AOF	0x5F
/* Video Output Deep Color Status */
#define SIL_9135A_DC_STAT	0x61
/* Video Pixel Clock Timing */
#define SIL_9135A_VID_XPCNT1	0x6E
#define SIL_9135A_VID_XPCNT2	0x6F
/* Interrupts State Bit */
#define SIL_9135A_INTR_STATE	0x70
#define SIL_9135A_INTR1		0x71
#define SIL_9135A_INTR2		0x72
#define SIL_9135A_INTR3		0x73
#define SIL_9135A_INTR4		0x74
/* Interrupts Unmask */
#define SIL_9135A_INTR1_UNMASK	0x75
#define SIL_9135A_INTR2_UNMASK	0x76
#define SIL_9135A_INTR3_UNMASK	0x77
#define SIL_9135A_INTR4_UNMASK	0x78
/* Interrupt Control */
#define SIL_9135A_INT_CTRL	0x79
	#define SIL_9135A_INT_SET(reg, x) \
		(((reg) & (~((u8)0x1) << 2)) | (x << 2))
	#define SIL_9135A_INT_OPEN_DRAIN(reg, x) \
		(((reg) & (~((u8)0x1) << 1)) | (x << 1))
/* Interrupts InfoFrame Control */
#define SIL_9135A_IP_CTRL	0x7A
/* Interrupts Status bit */
#define SIL_9135A_INTR5		0x7B
#define SIL_9135A_INTR6		0x7C
/* Interrupts Unmask */
#define SIL_9135A_INTR5_UNMASK	0x7D
#define SIL_9135A_INTR6_UNMASK	0x7E
/* TMDS Analog Control #2 */
#define SIL_9135A_TMDS_CCTRL2	0x81
/* TMDS Termination Control */
#define SIL_9135A_TMDS_TERMCTRL	0x82
/* ACR Config */
#define SIL_9135A_ACR_CFG1	0x88
#define SIL_9135A_ACR_CFG2	0x89
/* Interrupts Status Bit */
#define SIL_9135A_INTR7		0x90
/* Interrupts Unmask */
#define SIL_9135A_INTR7_UNMASK	0x92
/* AEC Control */
#define SIL_9135A_AEC_CTRL	0xB5
	#define SIL_9135A_ENABLE_AEC(reg, x) \
		(((reg) & (~((u8)0x1) << 2)) | (x << 2))
/* AEC Exception */
#define SIL_9135A_AEC_EN1	0xB6
	#define SIL_9135A_AEC_CABLE_UNPLUG(reg, x) \
		(((reg) & (~((u8)0x1) << 0)) | (x << 0))
	#define SIL_9135A_AEC_SYNC_DETECT(reg, x) \
		(((reg) & (~((u8)0x1) << 6)) | (x << 6))
	#define SIL_9135A_AEC_CS_DETECT(reg, x) \
		(((reg) & (~((u8)0x1) << 7)) | ((u8)x << 7))
#define SIL_9135A_AEC_EN2	0xB7
	#define SIL_9135A_AEC_FIFO_UNDERRUN(reg, x) \
		(((reg) & (~((u8)0x1) << 1)) | (x << 1))
	#define SIL_9135A_AEC_FIFO_OVERRUN(reg, x) \
		(((reg) & (~((u8)0x1) << 2)) | (x << 2))
	#define SIL_9135A_AEC_HR_CHANGE(reg, x)	 \
		(((reg) & (~((u8)0x1) << 7)) | ((u8)x << 7))
#define SIL_9135A_AEC_EN3	0xB8
	#define SIL_9135A_AEC_VR_CHANGE(reg, x)	 \
		(((reg) & (~((u8)0x1) << 0)) | (x << 0))
/* AEC Video Configuration Mask */
#define SIL_9135A_AVC_EN1	0xB9
#define SIL_9135A_AVC_EN2	0xBA
/* ECC Control */
#define SIL_9135A_ECC_CTRL	0xBB
/* BCH Threshold */
#define SIL_9135A_BCH_THRES	0xBC
/* ECC T4 Corrected Threshold */
#define SIL_9135A_T4_THRES	0xBD
/* ECC T4 Uncorrected Threshold */
#define SIL_9135A_T4_UNTHRES	0xBE
/* ECC Error Threshold and Count */
#define SIL_9135A_PKT_THRESH	0xBF
#define SIL_9135A_PKT_THRESH2	0xC0
#define SIL_9135A_T4_PKT_THRES	0xC1
#define SIL_9135A_T4_PKT_THRES2	0xC2
#define SIL_9135A_BCH_PKT_THRES	0xC3
#define SIL_9135A_BCH_PKT_THRES2	0xC4
#define SIL_9135A_HDCP_THRES	0xC5
#define SIL_9135A_HDCP_THRES2	0xC6
#define SIL_9135A_PKT_CNT	0xC7
#define SIL_9135A_PKT_CNT2	0xC8
#define SIL_9135A_T4_ERR	0xC9
#define SIL_9135A_T4_ERR2	0xCA
#define SIL_9135A_BCH_ERR	0xCB
#define SIL_9135A_BCH_ERR2	0xCC
#define SIL_9135A_HDCP_ERR	0xCD
#define SIL_9135A_HDCP_ERR2	0xCE

/* I2C Reg definitions for address 0x68 */
/* ACR Control #1 */
#define SIL_9135A_ACR_CTRL1	0x0
/* ACR Audio Frequency */
#define SIL_9135A_FREQ_SVAL	0x2
/* ACR N Value */
#define SIL_9135A_N_SVAL1	0x3
#define SIL_9135A_N_SVAL2	0x4
#define SIL_9135A_N_SVAL3	0x5
#define SIL_9135A_N_HVAL1	0x6
#define SIL_9135A_N_HVAL2	0x7
#define SIL_9135A_N_HVAL3	0x8
/* ACR CTS Value */
#define SIL_9135A_CTS_SVAL1	0x9
#define SIL_9135A_CTS_SVAL2	0xA
#define SIL_9135A_CTS_SVAL3	0xB
#define SIL_9135A_CTS_HVAL1	0xC
#define SIL_9135A_CTS_HVAL2	0xD
#define SIL_9135A_CTS_HVAL3	0xE
/* ACR PLL Lock */
#define SIL_9135A_LKWIN_SVAL	0x13
	#define SIL_9135A_LKWIN_WINDIV(x)	((x & 0xE0) | 1)
#define SIL_9135A_LKTHRESH1	0x14
	#define SIL_9135A_SET_LKTHRESH1(x)	(x)
#define SIL_9135A_LKTHRESH2	0x15
	#define SIL_9135A_SET_LKTHRESH2(x)	(x)
#define SIL_9135A_LKTHRESH3	0x16
	#define SIL_9135A_SET_LKTHRESH3(x)	(x & 0xF0)
/* ACR HW Fs */
#define SIL_9135A_TCLK_FS	0x17
/* ACR Control #3 */
#define SIL_9135A_ACR_CTRL3	0x18
	#define SIL_9135A_CTS_THRESH(reg, x)	\
		(((reg) & ((u8)0xF << 3)) | ((x) << 3))
/* Audio Out I2S Control #1 */
#define SIL_9135A_I2S_CTRL1	0x26
	#define SIL_9135A_I2S_LSB(reg, x)		\
		(((reg) & (~((u8)0x1 << 1))) | (x << 1))
/* Audio Out Control */
#define SIL_9135A_AUD_CTRL	0x29
/* Audio Out S/PDIF CHST1 */
#define SIL_9135A_CHST1		0x2A
/* Audio Out S/PDIF CHST2 */
#define SIL_9135A_CHST2		0x2B
/* Audio Out S/PDIF CHST3 */
#define SIL_9135A_CHST3		0x2C
/* Audio CHST5 Overwrite */
#define SIL_9135A_OW_CHST5	0x2F
/* Audio Out S/PDIF CHST4 */
#define SIL_9135A_CHST4		0x30
/* Audio Out S/PDIF CHST5 */
#define SIL_9135A_CHST5		0x31
/* HDMI Audio Status */
#define SIL_9135A_AUDP_STAT	0x34
/* HDMI Mute Path */
#define SIL_9135A_HDMI_MUTE	0x37
	#define SIL_9135A_RPT_LAST_GOOD(reg, x)		\
		(((reg) & (~((u8)0x1 << 1))) | (x << 1))
	#define SIL_9135A_SEND_BLANKING(reg, x)		\
		(((reg) & (~((u8)0x1 << 2))) | (x << 2))
/* HDMI HDCP Enable Criteria */
#define SIL_9135A_AUDP_CRIT2	0x38
/* HDMI FIFO Read/Write Pointer Difference */
#define SIL_9135A_FIFO_PTRDIFF	0x39
/* Power Down Total */
#define SIL_9135A_PD_TOT	0x3C
/* System Power Down #3 */
#define SIL_9135A_PD_SYS3	0x3D
/* System Power Down #2 */
#define SIL_9135A_PD_SYS2	0x3E
/* System Power Down */
#define SIL_9135A_PD_SYS	0x3F
/* CEA-861B AVI InfoFrame */
#define SIL_9135A_AVI_TYPE	0x40
#define SIL_9135A_AVI_VERS	0x41
#define SIL_9135A_AVI_LEN	0x42
#define SIL_9135A_AVI_CHSUM	0x43
#define SIL_9135A_AVI_DBYTE1	0x44
#define SIL_9135A_AVI_DBYTE2	0x45
#define SIL_9135A_AVI_DBYTE3	0x47
#define SIL_9135A_AVI_DBYTE4	0x48
#define SIL_9135A_AVI_DBYTE5	0x49
#define SIL_9135A_AVI_DBYTE7	0x4A
#define SIL_9135A_AVI_DBYTE8	0x4B
#define SIL_9135A_AVI_DBYTE9	0x4C
#define SIL_9135A_AVI_DBYTE10	0x4D
#define SIL_9135A_AVI_DBYTE11	0x4E
#define SIL_9135A_AVI_DBYTE12	0x4F
#define SIL_9135A_AVI_DBYTE13	0x50
#define SIL_9135A_AVI_DBYTE14	0x51
#define SIL_9135A_AVI_DBYTE15	0x52
/* CEA-861B SPD InfoFrame */
#define SIL_9135A_SPD_TYPE	0x60
#define SIL_9135A_SPD_VERS	0x61
#define SIL_9135A_SPD_LEN	0x62
#define SIL_9135A_SPD_CHSUM	0x63
#define SIL_9135A_SPD_DBYTE1	0x64
#define SIL_9135A_SPD_DBYTE2	0x65
#define SIL_9135A_SPD_DBYTE3	0x66
#define SIL_9135A_SPD_DBYTE4	0x67
#define SIL_9135A_SPD_DBYTE5	0x68
#define SIL_9135A_SPD_DBYTE6	0x69
#define SIL_9135A_SPD_DBYTE7	0x6A
#define SIL_9135A_SPD_DBYTE8	0x6B
#define SIL_9135A_SPD_DBYTE9	0x6C
#define SIL_9135A_SPD_DBYTE10	0x6D
#define SIL_9135A_SPD_DBYTE11	0x6E
#define SIL_9135A_SPD_DBYTE12	0x6F
#define SIL_9135A_SPD_DBYTE13	0x70
#define SIL_9135A_SPD_DBYTE14	0x71
#define SIL_9135A_SPD_DBYTE15	0x72
#define SIL_9135A_SPD_DBYTE16	0x73
#define SIL_9135A_SPD_DBYTE17	0x74
#define SIL_9135A_SPD_DBYTE18	0x75
#define SIL_9135A_SPD_DBYTE19	0x76
#define SIL_9135A_SPD_DBYTE20	0x77
#define SIL_9135A_SPD_DBYTE21	0x78
#define SIL_9135A_SPD_DBYTE22	0x79
#define SIL_9135A_SPD_DBYTE23	0x7A
#define SIL_9135A_SPD_DBYTE24	0x7B
#define SIL_9135A_SPD_DBYTE25	0x7C
#define SIL_9135A_SPD_DBYTE26	0x7D
#define SIL_9135A_SPD_DBYTE27	0x7E
/* Packet Type Decode */
#define SIL_9135A_SPD_DEC	0x7F
/* Audio InfoFrame */
#define SIL_9135A_AUDIO_TYPE	0x80
#define SIL_9135A_AUDIO_VERS	0x81
#define SIL_9135A_AUDIO_LEN	0x82
#define SIL_9135A_AUDIO_CHSUM	0x83
#define SIL_9135A_AUDIO_DBYTE1	0x84
#define SIL_9135A_AUDIO_DBYTE2	0x85
#define SIL_9135A_AUDIO_DBYTE3	0x86
#define SIL_9135A_AUDIO_DBYTE4	0x87
#define SIL_9135A_AUDIO_DBYTE5	0x88
#define SIL_9135A_AUDIO_DBYTE6	0x89
#define SIL_9135A_AUDIO_DBYTE7	0x8a
#define SIL_9135A_AUDIO_DBYTE8	0x8b
#define SIL_9135A_AUDIO_DBYTE9	0x8c
#define SIL_9135A_AUDIO_DBYTE10	0x8d
/* MPEG Infoframe */
#define SIL_9135A_MPEG_TYPE	0xA0
#define SIL_9135A_MPEG_VERS	0xA1
#define SIL_9135A_MPEG_LEN	0xA2
#define SIL_9135A_MPEG_CHSUM	0XA3
#define SIL_9135A_MPEG_DBYTE1	0xA4
#define SIL_9135A_MPEG_DBYTE2	0xA5
#define SIL_9135A_MPEG_DBYTE3	0xA6
#define SIL_9135A_MPEG_DBYTE4	0xA7
#define SIL_9135A_MPEG_DBYTE5	0xA8
#define SIL_9135A_MPEG_DBYTE6	0xA9
#define SIL_9135A_MPEG_DBYTE7	0xAA
#define SIL_9135A_MPEG_DBYTE8	0xAB
#define SIL_9135A_MPEG_DBYTE9	0xAC
#define SIL_9135A_MPEG_DBYTE10	0xAD
#define SIL_9135A_MPEG_DBYTE11	0xAE
#define SIL_9135A_MPEG_DBYTE12	0xAF
#define SIL_9135A_MPEG_DBYTE13	0xB0
#define SIL_9135A_MPEG_DBYTE14	0xB1
#define SIL_9135A_MPEG_DBYTE15	0xB2
#define SIL_9135A_MPEG_DBYTE16	0xB3
#define SIL_9135A_MPEG_DBYTE17	0xB4
#define SIL_9135A_MPEG_DBYTE18	0xB5
#define SIL_9135A_MPEG_DBYTE19	0xB6
#define SIL_9135A_MPEG_DBYTE20	0xB7
#define SIL_9135A_MPEG_DBYTE21	0xB8
#define SIL_9135A_MPEG_DBYTE22	0xB9
#define SIL_9135A_MPEG_DBYTE23	0xBA
#define SIL_9135A_MPEG_DBYTE24	0xBB
#define SIL_9135A_MPEG_DBYTE25	0xBC
#define SIL_9135A_MPEG_DBYTE26	0xBD
#define SIL_9135A_MPEG_DBYTE27	0xBE
/* Packet Type Decode */
#define SIL_9135A_MPEG_DEC	0xBF
/* Unrecognized Packet */
#define SIL_9135A_UNR_DBYTE1	0xC0
#define SIL_9135A_UNR_DBYTE2	0xC1
#define SIL_9135A_UNR_DBYTE3	0xC2
#define SIL_9135A_UNR_DBYTE4	0xC3
#define SIL_9135A_UNR_DBYTE5	0xC4
#define SIL_9135A_UNR_DBYTE6	0xC5
#define SIL_9135A_UNR_DBYTE7	0xC6
#define SIL_9135A_UNR_DBYTE8	0xC7
#define SIL_9135A_UNR_DBYTE9	0xC8
#define SIL_9135A_UNR_DBYTE10	0xC9
#define SIL_9135A_UNR_DBYTE11	0xCA
#define SIL_9135A_UNR_DBYTE12	0xCB
#define SIL_9135A_UNR_DBYTE13	0xCC
#define SIL_9135A_UNR_DBYTE14	0xCD
#define SIL_9135A_UNR_DBYTE15	0xCE
#define SIL_9135A_UNR_DBYTE16	0xCF
#define SIL_9135A_UNR_DBYTE17	0xD0
#define SIL_9135A_UNR_DBYTE18	0xD1
#define SIL_9135A_UNR_DBYTE19	0xD2
#define SIL_9135A_UNR_DBYTE20	0xD3
#define SIL_9135A_UNR_DBYTE21	0xD4
#define SIL_9135A_UNR_DBYTE22	0xD5
#define SIL_9135A_UNR_DBYTE23	0xD6
#define SIL_9135A_UNR_DBYTE24	0xD7
#define SIL_9135A_UNR_DBYTE25	0xD8
#define SIL_9135A_UNR_DBYTE26	0xD9
#define SIL_9135A_UNR_DBYTE27	0xDA
#define SIL_9135A_UNR_DBYTE28	0xDB
#define SIL_9135A_UNR_DBYTE29	0xDC
#define SIL_9135A_UNR_DBYTE30	0xDD
#define SIL_9135A_UNR_DBYTE31	0xDE
/* General Control Packet */
#define SIL_9135A_GCP_DBYTE1	0xDF
/* Audio Control Packet */
#define SIL_9135A_ACP_HB0	0xE0
#define SIL_9135A_ACP_HB1	0xE1
#define SIL_9135A_ACP_HB2	0xE2
#define SIL_9135A_ACP_DBYTE0	0xE3
#define SIL_9135A_ACP_DBYTE1	0xE4
#define SIL_9135A_ACP_DBYTE2	0xE5
#define SIL_9135A_ACP_DBYTE3	0xE6
#define SIL_9135A_ACP_DBYTE4	0xE7
#define SIL_9135A_ACP_DBYTE5	0xE8
#define SIL_9135A_ACP_DBYTE6	0xE9
#define SIL_9135A_ACP_DBYTE7	0xEA
#define SIL_9135A_ACP_DBYTE8	0xEB
#define SIL_9135A_ACP_DBYTE9	0xEC
#define SIL_9135A_ACP_DBYTE10	0xED
#define SIL_9135A_ACP_DBYTE11	0xEE
#define SIL_9135A_ACP_DBYTE12	0xEF
#define SIL_9135A_ACP_DBYTE13	0xF0
#define SIL_9135A_ACP_DBYTE14	0xF1
#define SIL_9135A_ACP_DBYTE15	0xF2
#define SIL_9135A_ACP_DBYTE16	0xF3
#define SIL_9135A_ACP_DBYTE17	0xF4
#define SIL_9135A_ACP_DBYTE18	0xF5
#define SIL_9135A_ACP_DBYTE19	0xF6
#define SIL_9135A_ACP_DBYTE20	0xF7
#define SIL_9135A_ACP_DBYTE21	0xF8
#define SIL_9135A_ACP_DBYTE22	0xF9
#define SIL_9135A_ACP_DBYTE23	0xFA
#define SIL_9135A_ACP_DBYTE24	0xFB
#define SIL_9135A_ACP_DBYTE25	0xFC
#define SIL_9135A_ACP_DBYTE26	0xFD
#define SIL_9135A_ACP_DBYTE27	0xFE
/* Packet Type Decode Reg*/
#define SIL_9135A_ACP_DEC	0xFF
#endif
