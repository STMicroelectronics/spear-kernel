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
 * C3 driver MPCM
 *
 * ST - 2007-08-01
 * Alessandro Miglietti
 * ----------------------------------------------------------------- */

#ifndef __C3_MPCM_H__
#define __C3_MPCM_H__

/*-------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------- */

/* AES CBC, CTR and ECB key load */
#define MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX     ((unsigned int)0x1)

/* AES CBC and CBC key schedule */
#define MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX     ((unsigned int)0x2)

/* AES CTR and CBC init Ivec */
#define MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX        ((unsigned int)0x3)

/* AES ECB append */
#define MPCM_AES_ECB_ENC_APPEND_PROGRAM_INDEX           ((unsigned int)0x4)
#define MPCM_AES_ECB_DEC_APPEND_PROGRAM_INDEX           ((unsigned int)0x5)

/* AES CBC append */
#define MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX           ((unsigned int)0x6)
#define MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX           ((unsigned int)0x7)

/* AES CTR append */
#define MPCM_AES_CTR_APPEND_PROGRAM_INDEX               ((unsigned int)0x8)

/* AES-XCBC-MAX-96 */
#define MPCM_AES_XCBC_MAC_96_KEY_LOAD_PROGRAM_INDEX     ((unsigned int)0x9)
#define MPCM_AES_XCBC_MAC_96_APPEND_PROGRAM_INDEX       ((unsigned int)0xA)
#define MPCM_AES_XCBC_MAC_96_END_PROGRAM_INDEX          ((unsigned int)0xB)

/*-------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

unsigned int c3_mpcm_init(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#endif /* __C3_MPCM_H__ */
