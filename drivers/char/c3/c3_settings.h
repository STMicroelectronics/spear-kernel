/*
 * Copyright (C) 2007-2010 STMicroelectronics Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

/* --------------------------------------------------------------------
 * C3 driver platform-spefic settings
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2010-02-11 MGR Added support for SPEAr basic rev. BA.
 * - 2010-09-16 SK: Added Spear13xx support.
 * - 2010-09-16 SK: Removed support for older archs.
 * -		Only Spear3xx and Spear13xx supported now.
 * - 2011-02021 SK: Removed SP, GP, MPR support.
 * ----------------------------------------------------------------- */

#ifndef __C3_SETTINGS_H__
#define __C3_SETTINGS_H__


/* -------------------------------------------------------------------
 * TYPES
 * ---------------------------------------------------------------- */

typedef struct {
	int ids_idx;
	int channel_idx;

} c3_alg_info_t;

/* -------------------------------------------------------------------
 * CONSTANTS
 * ---------------------------------------------------------------- */

#define C3_SYS_HWID_offset		((unsigned int)0x3FC)
#define C3_SYS_SCR_offset		((unsigned int)0x000)
#define C3_SYS_ISDn_shift		((unsigned int)20)
#define C3_SYS_ISDn_mask		((unsigned int)0x00F00000)
#define C3_SYS_ARST_mask		((unsigned int)0x00010000)
#define C3_ID_IS_mask			((unsigned int)0x00800000)
#define C3_ID_BUSY_mask			((unsigned int)0xC0000000)
#define C3_HIF_offset			((unsigned int)0x0400)
#define C3_HIF_MSIZE_offset		((unsigned int)0x300)
#define C3_HIF_MBAR_offset		((unsigned int)0x304)
#define C3_HIF_MCR_offset		((unsigned int)0x308)
#define C3_ID_size			((unsigned int)0x400)
#define C3_IDS0_offset			((unsigned int)0x1000)
#define C3_ID_IP_offset			((unsigned int)0x010)
#define C3_ID_IR0_offset		((unsigned int)0x020)
#define C3_ID_IR1_offset		((unsigned int)0x024)
#define C3_ID_IR2_offset		((unsigned int)0x028)
#define C3_ID_IR3_offset		((unsigned int)0x02C)
#define C3_ID_SCR_offset		((unsigned int)0x000)
#define C3_HIF_ENABLE_MEM_mask		((unsigned int)0x00000001)
#define C3_CONTROL_ERROR_INT_mask	((unsigned int)0x00200000)
#define C3_CONTROL_STOP_INT_mask	((unsigned int)0x00400000)
#define C3_CONTROL_SSE_mask		((unsigned int)0x00080000)
#define C3_IDS_IDLE_mask		((unsigned int)0x80000000)
#define C3_IDS_RUN_mask			((unsigned int)0xc0000000)
#define C3_IDS_ERROR_mask		((unsigned int)0x40000000)
#define C3_IDS_BUS_ERROR_mask		((unsigned int)0x20000000)
#define C3_IDS_CHANNEL_ERROR_mask	((unsigned int)0x04000000)
#define C3_SCATTER_GATHER_THRESOLD	((unsigned int)4096)

#if defined(C3_SPEARBASIC_REVBA_FFFF10xx)

/* =================================================================
 *
 * C3_SPEARBASIC_REVBA_FFFF10xx
 *
 * ================================================================= */

/* GENERAL */
#define C3_HID_MASK			((unsigned int)0xFFFFFF00)
#define C3_HID				((unsigned int)0xFFFF1000)
#define C3_IDS_COUPLE_PATH		((unsigned int)0)
#define C3_IDS_NUMBER			((unsigned int)1)
#define C3_MAJOR			((unsigned int)0) /* Dynamic */
#undef C3_PKA7				/*Not available for SPEArBasic */

/* CHANNELS */
#undef MOVE_CHANNEL_INFO		/* {Not Available} */

/* {Available, IDS 0 - Channel 1 - No s/g} */
#define DES_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x1 })
/* {Available, IDS 0 - Channel 2 - No s/g} */
#define AES_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x2 })
/* {Available, IDS 0 - Channel 3 - No s/g}, NO SHA-256 */
#define UH_CHANNEL_INFO			((c3_alg_info_t) { 0x0, 0x3 })
#undef  UH2_CHANNEL_INFO		/* {Not Available} */
#undef  PKA_CHANNEL_INFO		/* {Not Available} */
#undef  MPCM_CHANNEL_INFO		/* {Not Available} */
#undef  RNG_CHANNEL_INFO		/* {Not Available} */

/* SCATTER & GATHER SETTINGS */
#undef  USE_MOVE_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_DES_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_AES_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_PKA_SCATTER_GATHER		/* No scatter/gather */
#undef  SG_ENABLED

/* CHANNELS SETTINGS */
/* --- MPCM --- */
#undef USE_MPCM_AES_ECB
#undef USE_MPCM_AES_CBC
#undef USE_MPCM_AES_CTR
#undef USE_MPCM_AES_XCBC_MAC_96

/* BUS/ARCHITECTURE */
#define C3_SPEARBASIC_REVBA

/* EXTRA FEATURES: CCM */
#define C3_COUPLING_ON

/* EXTRA FEATURES: LOCAL MEMORY */
#undef C3_LOCAL_MEMORY_ON

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#elif defined(C3_SPEARBASIC_REVA_FFFF10xx)

/* =================================================================
 *
 * C3_SPEARBASIC_REVA_FFFF10xx
 *
 * ================================================================= */


/* GENERAL */
#define C3_HID_MASK			((unsigned int)0xFFFFFF00)
#define C3_HID				((unsigned int)0xFFFF1000)
#define C3_IDS_COUPLE_PATH		((unsigned int)0)
#define C3_IDS_NUMBER			((unsigned int)1)
#define C3_MAJOR			((unsigned int)0) /* Dynamic */
#undef C3_PKA7				/*Not available for SPEArBasic */

/* CHANNELS */
#undef MOVE_CHANNEL_INFO		/* {Not Available} */
/* {Available, IDS 0 - Channel 1 - No s/g} */
#define DES_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x1 })
/* {Available, IDS 0 - Channel 2 - No s/g} */
#define AES_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x2 })
/* {Available, IDS 0 - Channel 3 - No s/g} NO SHA-256 */
#define UH_CHANNEL_INFO			((c3_alg_info_t) { 0x0, 0x3 })
#undef  UH2_CHANNEL_INFO		/* {Not Available} */
#undef  PKA_CHANNEL_INFO		/* {Not Available} */
#undef  MPCM_CHANNEL_INFO		/* {Not Available} */
#undef  RNG_CHANNEL_INFO		/* {Not Available} */

/* SCATTER & GATHER SETTINGS */
#undef  USE_MOVE_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_DES_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_AES_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_PKA_SCATTER_GATHER		/* No scatter/gather */
#undef  SG_ENABLED

/* CHANNELS SETTINGS */
/* --- MPCM --- */
#undef USE_MPCM_AES_ECB
#undef USE_MPCM_AES_CBC
#undef USE_MPCM_AES_CTR
#undef USE_MPCM_AES_XCBC_MAC_96

/* BUS/ARCHITECTURE */
#define C3_SPEARBASIC_REVA

/* EXTRA FEATURES: CCM */
#define C3_COUPLING_ON

/* EXTRA FEATURES: LOCAL MEMORY */
#undef C3_LOCAL_MEMORY_ON

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#elif defined C3_SPEAR13xx

/* =================================================================
 *
 * C3_SPEAR13xx
 *
 * ================================================================= */

/* GENERAL */
#define C3_HID_MASK			((unsigned int)0xFFFFFF00)
#define C3_HID				((unsigned int)0xFFFF8000)
#define C3_IDS_COUPLE_PATH		((unsigned int)2)
#define C3_IDS_NUMBER			((unsigned int)2)
#define C3_MAJOR			((unsigned int)0) /* Dynamic */

#define C3_PKA7				/* Available */

/* CHANNELS */

/* {Available, IDS 0 - Channel 0 } */
#define MOVE_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x0 })

/* {Available, IDS 0 - Channel 1 } */
#define DES_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x1 })

/* {Not Available} */
#undef  AES_CHANNEL_INFO

/* {Available, IDS 0 - Channel 3 } */
#define UH_CHANNEL_INFO			((c3_alg_info_t) { 0x0, 0x3 })

/* {Available, IDS 1 - Channel 4 } */
#define UH2_CHANNEL_INFO		((c3_alg_info_t) { 0x1, 0x4 })

/* {Available, IDS 0 - Channel 5 */
#define PKA_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x5 })

/* {Available, IDS 0 - Channel 2, MPCM for AES, s/g available } */
#define MPCM_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x2 })

/* {Available, IDS 0 - Channel 6 */
#define RNG_CHANNEL_INFO		((c3_alg_info_t) { 0x0, 0x6 })

/* SCATTER & GATHER SETTINGS */
#define USE_AES_SCATTER_GATHER		/* MPCM for AES and it supports S&G */
#undef  USE_MOVE_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_DES_SCATTER_GATHER		/* No scatter/gather */
#undef  USE_PKA_SCATTER_GATHER		/* No scatter/gather */
#define  SG_ENABLED			/* MPCM supports SG */

/* CHANNELS SETTINGS */
/* --- MPCM --- */
#define USE_MPCM_AES_ECB
#define USE_MPCM_AES_CBC
#define USE_MPCM_AES_CTR
#undef USE_MPCM_AES_XCBC_MAC_96

/* BUS/ARCHITECTURE */
#define C3_SPEAR13XX

/* EXTRA FEATURES: CCM */
#define C3_COUPLING_ON

/* EXTRA FEATURES: LOCAL MEMORY */
/* Local memory address MUST be multiple of 64KB */
#define C3_LOCAL_MEMORY_ON
#define LOCAL_MEMORY_ADDRESS		((unsigned long)0xF0000000)
#define LOCAL_MEMORY_SIZE		((unsigned int)16384)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


#else
#error "No C3 platform specified"
#endif

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef C3_SPEARBASIC_REVA
#define C3_UH_PATCH()			WAIT(100, prgmem);
#else
#define C3_UH_PATCH()
#endif

#ifndef C3_HID_MASK
#define C3_HID_MASK			((unsigned int)0xFFFFFFFF)
#endif

#if defined(C3_LOCAL_MEMORY_ON) && !defined(LOCAL_MEMORY_ADDRESS)
#error "Local memory is set to ON, but memory address is not set!"
#endif

#if defined(C3_LOCAL_MEMORY_ON) && !defined(LOCAL_MEMORY_SIZE)
#error "Local memory is set to ON, but memory size is not set!"
#endif


#endif /* __C3_SETTINGS_H__ */
