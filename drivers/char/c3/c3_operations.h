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
 * C3 driver operations
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2009-09-08 MGR Added PKA v.7 support.
 * ----------------------------------------------------------------- */

#ifndef __C3_OPERATIONS_H__
#define __C3_OPERATIONS_H__

/* -------------------------------------------------------------------
 * MACROS
 * ---------------------------------------------------------------- */

/* Based on the Instruction Set 2.1 - C3v3 */

/* Module Number (31-28) */

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        CU                               |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define NEXT(address, prgmem) \
	do { \
		*(prgmem++) = \
			((0 << 28) +    /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (2 << 23));    /* Operation: NEXT (25-23) */ \
		*(prgmem++) = address; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define NOP(prgmem) \
	do { \
		*(prgmem++) = \
			((0 << 28) +    /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (3 << 23));    /* Operation: NOP (25-23) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define WAIT(cycles, prgmem) \
	do { \
		*(prgmem++) = \
			((0 << 28) +    /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (1 << 23) +    /* Operation: NOP (25-23) */ \
			 cycles); \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MOVE(len, src, dest, prgmem) \
	do { \
		*(prgmem++) = \
			((MOVE_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (5 << 23) +    /* Operation: MOVE_data (25-23) */ \
			 (0 << 21) +    /* No logical operation: 0 (22-21) */ \
			 (len));        /* len: (15-0) */ \
		*(prgmem++) = src; \
		*(prgmem++) = dest; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define CHAIN(master, slave, prgmem) \
	do { \
		*(prgmem++) = \
			(0 +    /* Mod Num */ \
			 0 +    /* Add Instr Words: 0 (27-26) */ \
			 (6 << 23) +    /* Operation: COUPLE (25-23) */	\
			 (master << 19) +       /* Master device (22-19) */ \
			 (1 << 18) +    /* Chaining (18) */ \
			 (slave << 14) +        /* Slave device (17-14) */ \
			 (C3_IDS_COUPLE_PATH << 11));   /* Path num (13-11) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define COUPLE(master, slave, prgmem) \
	do { \
		*(prgmem++) = \
			(0 +    /* Mod Num */ \
			 0 +    /* Add Instr Words: 0 (27-26) */ \
			 (6 << 23) +    /* Operation: COUPLE (25-23) */	\
			 (master << 19) +       /* Master device (22-19) */ \
			 (0 << 18) +    /* Coupling (18) */ \
			 (slave << 14) +        /* Slave device (17-14) */ \
			 (C3_IDS_COUPLE_PATH << 11));   /* Path num (13-11) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define STOP(prgmem) \
	do { \
		*(prgmem++) = \
			0 +     /* Mod Num */ \
			0 +     /* Add Instr Words: 0 (27-26) */ \
			0;	/* Operation: STOP (25-23) */	\
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       AES                               |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define AES_RANDOMIZE(seed, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* Randomize (25) */ \
			 16),   /* Len */ \
		*(prgmem++) = seed; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_CBC_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_CBC_FVE_START(key_len, flags, key, ivec, lba_size, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 (1 << 20) +    /* FVE (20) */ \
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
		*(prgmem++) = lba_size;	\
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_CBC_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_CBC_FVE_START(key_len, flags, key, ivec, lba_size, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 (1 << 20) +    /* FVE (20) */ \
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
		*(prgmem++) = lba_size;	\
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_ECB_START(key_len, flags, key, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: CBC (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_ECB_START(key_len, flasg, key, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: CBC (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_CTR_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (2 << 21) +    /* Mode: CTR (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_CTR_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (2 << 21) +    /* Mode: CTR (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_CTR_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (2 << 21) +    /* Mode: CTR (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_CTR_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (2 << 21) +    /* Mode: CTR (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_XTS_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_XTS_FVE_START(key_len, flags, key, ivec, lba_size, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 (1 << 20) +    /* FVE (20) */ \
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
		*(prgmem++) = lba_size;	\
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_DECRYPT_XTS_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_XTS_START(key_len, flags, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_XTS_FVE_START(key_len, flags, key, ivec, lba_size, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 (1 << 20) +    /* FVE (20) */ \
			 ((flags) << 18) +      /*flags: DPA | DUALCORE */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
		*(prgmem++) = lba_size;	\
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define AES_ENCRYPT_XTS_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((AES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* AES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (3 << 21) +    /* Mode: XTS (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       DES                               |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define DES_DECRYPT_CBC_START(key_len, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_DECRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_ENCRYPT_CBC_START(key_len, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_ENCRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_DECRYPT_ECB_START(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_DECRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_ENCRYPT_ECB_START(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define DES_ENCRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (0 << 25) +    /* DES (25) */ \
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       3DES                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define TRIPLE_DES_DECRYPT_CBC_START(key_len, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_DECRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_ENCRYPT_CBC_START(key_len, key, ivec, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = ivec; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_ENCRYPT_CBC_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (1 << 21) +    /* Mode: CBC (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_DECRYPT_ECB_START(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (1 << 24) +    /* Decrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_DECRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (1 << 24) +    /* Decrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_ENCRYPT_ECB_START(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (0 << 24) +    /* Encrypt (24) */ \
			 (0 << 23) +    /* Operation: START (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define TRIPLE_DES_ENCRYPT_ECB_APPEND(data_len, data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((DES_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* 3DES (25) */	\
			 (0 << 24) +    /* Encrypt (24) */ \
			 (1 << 23) +    /* Operation: APPEND (23) */ \
			 (0 << 21) +    /* Mode: ECB (22-21) */	\
			 data_len),     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA1                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define SHA1_INIT(prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (0 << 21));    /* Operation: INIT (22-21) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA1_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA1_END(result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20));    /* Save full digest (20) */ \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA1_HMAC_INIT(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (0 << 21) +    /* Operation: INIT (22-21) */ \
			 key_len);      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA1_HMAC_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA1_HMAC_END(key_len, key, result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA1 (24-23) */ \
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20) +    /* Save full digest (20) */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA256                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define SHA256_INIT(prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (0 << 21));    /* Operation: INIT (22-21) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA256_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA256_END(result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20));    /* Save full digest (20) */ \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA256_HMAC_INIT(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (0 << 21) +    /* Operation: INIT (22-21) */ \
			 key_len);      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA256_HMAC_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA256_HMAC_END(key_len, key, result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (2 << 23) +    /* Algorithm: SHA256 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20) +    /* Save full digest (20) */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA384                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define SHA384_INIT(prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (0 << 21));    /* Operation: INIT (22-21) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA384_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA384_END(result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20));    /* Save full digest (20) */ \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA384_HMAC_INIT(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (0 << 21) +    /* Operation: INIT (22-21) */ \
			 key_len);      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA384_HMAC_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA384_HMAC_END(key_len, key, result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: SHA384 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20) +    /* Save full digest (20) */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA512                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define SHA512_INIT(prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (0 << 21));    /* Operation: INIT (22-21) */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA512_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA512_END(result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20));    /* Save full digest (20) */ \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA512_HMAC_INIT(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (0 << 21) +    /* Operation: INIT (22-21) */ \
			 key_len);      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA512_HMAC_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SHA512_HMAC_END(key_len, key, result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH2_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (1 << 23) +    /* Algorithm: SHA512 (24-23) */	\
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20) +    /* Save full digest (20) */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                         MD5                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define MD5_INIT(prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (0 << 21));	/* Operation: INIT (22-21) */	\
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MD5_APPEND(data_len, data, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (1 << 21) +    /* Operation:APPEND (22-21) */ \
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MD5_END(result, prgmem)	\
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20));    /* Save full digest (20) */ \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MD5_HMAC_INIT(key_len, key, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (0 << 21) +    /* Operation: INIT (22-21) */ \
			 key_len);      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MD5_HMAC_APPEND(data_len, data, prgmem)	\
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (1 << 21) +    /* Operation: APPEND (22-21) */	\
			 data_len);     /* data_len: (15-0) */ \
		*(prgmem++) = data; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MD5_HMAC_END(key_len, key, result, prgmem) \
	do { \
		*(prgmem++) = \
			((UH_CHANNEL_INFO.channel_idx << 28) +  /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (0 << 23) +    /* Algorithm: MD5 (24-23) */ \
			 (2 << 21) +    /* Operation: END (22-21) */ \
			 (0 << 20) +    /* Save full digest (20) */ \
			 key_len),      /* key_len: (15-0) */ \
		*(prgmem++) = key; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                     DIGEST COMMON                       |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define SAVE_CONTEXT(ctx, channel, prgmem) \
	do { \
		*(prgmem++) = \
			((channel << 28) +      /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (3 << 23) +    /* Algorithm: CONTEXT (24-23) */ \
			 (0 << 22));    /* Operation: SAVE (22-21) */ \
		*(prgmem++) = ctx; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define RESTORE_CONTEXT(ctx, channel, prgmem) \
	do { \
		*(prgmem++) = \
			((channel << 28) +      /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 (0 << 25) +    /* HASH (25) */	\
			 (3 << 23) +    /* Algorithm: CONTEXT (24-23) */ \
			 (1 << 22));    /* Operation: RESTORE (22-21) */ \
		*(prgmem++) = ctx; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define SAVE_HMAC_CONTEXT(ctx, channel, prgmem)	\
	do { \
		*(prgmem++) = \
			((channel << 28) +      /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (3 << 23) +    /* Algorithm: CONTEXT (24-23) */ \
			 (0 << 22) +    /* Operation: SAVE (22-21) */ \
			 *(prgmem++) = ctx; \
			 } while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define RESTORE_HMAC_CONTEXT(ctx, channel, prgmem) \
	do { \
		*(prgmem++) = \
			((channel << 28) +      /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 25) +    /* HMAC (25) */	\
			 (3 << 23) +    /* Algorithm: CONTEXT (24-23) */ \
			 (1 << 22) +    /* Operation: RESTORE (22-21) */ \
			 *(prgmem++) = ctx; \
			 } while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                         PKA                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define PKA_MONTY_EXP(sec_data, sec_data_len, pub_data, pub_data_len, sca_cm, \
		      result, \
		      prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (3 << 23) +    /* Operation: MONTY_EXP (25-23) */ \
			 ((sca_cm & 0x3) << 21) + /*Side chan countermeasures*/\
			 pub_data_len + sec_data_len);  /* Data len */ \
		*(prgmem++) = sec_data;	\
		*(prgmem++) = pub_data;	\
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_MONTY_PAR(data, data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (1 << 23) +    /* Operation: MONTY_PAR (25-23) */ \
			 data_len);     /* Data len */ \
		*(prgmem++) = data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_MOD_EXP(sec_data, sec_data_len, pub_data, pub_data_len, sca_cm, \
		    result, \
		    prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (3 << 26) +    /* Add Instr Words: 3 (27-26) */ \
			 (2 << 23) +    /* Operation: MOD_EXP (25-23) */ \
			 ((sca_cm & 0x3) << 21) +/*SideChannel countermeasure*/\
			 pub_data_len + sec_data_len);  /* Data len */ \
		*(prgmem++) = sec_data;	\
		*(prgmem++) = pub_data;	\
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_ECC_MUL(in_data, in_data_len, sca_cm, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (6 << 23) +    /* Operation: ECC_MUL (25-23) */ \
			 ((sca_cm & 0x3) << 21) +/*SideChannel countermeasure*/\
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_ECC_MONTY_MUL(in_data, in_data_len, sca_cm, result, prgmem)	\
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (7 << 23) +    /* Operation: ECC_MONTY_MUL (25-23) */ \
			 ((sca_cm & 0x3) << 21) +/*SideChannel countermeasure*/\
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                         PKA7                            |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define PKA_ECC_DSA_SIGN(in_data, in_data_len, sca_cm, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ECC_DSA_SIGN (25-23) */ \
			 ((sca_cm & 0x3) << 21) +/*SideChannel countermeasure*/\
			 (8 << 17) +    /* Operation: ECC_DSA_SIGN (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_ECC_DSA_VERIFY(in_data, in_data_len, sca_cm, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation:ECC_DSA_VERIFY (25-23) */ \
			 ((sca_cm & 0x3) << 21) +/*SideChannel countermeasure*/\
			 (12 << 17) +   /* Operation: ECC_DSA_VERIFY (20-17) */\
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MUL(in_data, in_data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_MUL (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (13 << 17) +   /* Operation: ARITH_MUL (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_ADD(in_data, in_data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_ADD (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (14 << 17) +   /* Operation: ARITH_ADD (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_SUB(in_data, in_data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_SUB (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (15 << 17) +   /* Operation: ARITH_SUB (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_CMP(in_data, in_data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_CMP (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (7 << 17) +    /* Operation: ARITH_CMP (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MOD_RED(in_data, in_data_len, result, prgmem)	\
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_MOD_RED (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (0 << 17) +    /* Operation: ARITH_MOD_RED (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MOD_INV(in_data, in_data_len, result, prgmem)	\
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_MOD_INV (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (1 << 17) +    /* Operation: ARITH_MOD_INV (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MOD_ADD(in_data, in_data_len, result, prgmem)	\
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_MOD_ADD (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (2 << 17) +    /* Operation: ARITH_MOD_ADD (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MOD_SUB(in_data, in_data_len, result, prgmem)	\
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: ARITH_MOD_SUB (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (3 << 17) +    /* Operation: ARITH_MOD_SUB (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_ARITH_MONTY_MUL(in_data, in_data_len, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: MONTY_MUL (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures*/ \
			 (4 << 17) +    /* Operation: MONTY_MUL (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define PKA_RSA_CRT(in_data, in_data_len, sca_cm, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: RSA_CRT (25-23) */ \
			/* Side channel countermeasures. */	\
			 ((sca_cm & 0x3) << 21) + \
			 (5 << 17) +    /* Operation: RSA_CRT (20-17) */ \
			 in_data_len);  /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_MEM_RESET(prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (0 << 26) +    /* Add Instr Words: 0 (27-26) */ \
			 (4 << 23) +    /* Operation: MEM_RESET (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (6 << 17) +    /* Operation: MEM_RESET (20-17) */ \
			 0);    /* Data len */ \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Side channel: no countermeasures (TODO: add other modes...) */
#define PKA_MEM_READ(in_data, result, prgmem) \
	do { \
		*(prgmem++) = \
			((PKA_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (2 << 26) +    /* Add Instr Words: 2 (27-26) */ \
			 (4 << 23) +    /* Operation: MEM_READ (25-23) */ \
			 (0 << 21) +    /* Side channel: no countermeasures */ \
			 (9 << 17) +    /* Operation: MEM_READ (20-17) */ \
			 8);    /* Data len */ \
		*(prgmem++) = in_data; \
		*(prgmem++) = result; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        MPCM                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define MPCM_DOWNLOAD(prgno, addr, ptr, n, prgmem) \
	do { \
		*(prgmem++) = \
			((MPCM_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (36 << 22) +   /*Instruction code: DOWNLOAD (27-22)*/\
			 (prgno << 16) +        /* Program number (21-16) */ \
			 (n));  /* Micro-sequence length */ \
		*(prgmem++) = addr; \
		*(prgmem++) = ptr; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MPCM_EXECUTE_S(prgno, srcptr, n, prgmem) \
	do { \
		*(prgmem++) = \
			((MPCM_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (30 << 22) +   /* Instruction code: EXEC_S (27-22) */ \
			 (prgno << 16) +        /* Program number (21-16) */ \
			 (n));  /* Byte count */ \
		*(prgmem++) = srcptr; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MPCM_EXECUTE_SD(prgno, srcptr, destptr, n, prgmem) \
	do { \
		*(prgmem++) = \
			((MPCM_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (47 << 22) +   /* Instruction code: EXEC_SD (27-22) */\
			 (prgno << 16) +        /* Program number (21-16) */ \
			 (n));  /* Byte count */ \
		*(prgmem++) = srcptr; \
		*(prgmem++) = destptr; \
	} while (0)


/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                          RNG                            |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

#define RNG(buffer, len, prgmem) \
	do { \
		*(prgmem++) = \
			((RNG_CHANNEL_INFO.channel_idx << 28) + /* Mod Num */ \
			 (1 << 26) +    /* Add Instr Words: 1 (27-26) */ \
			 len); \
		*(prgmem++) = buffer; \
	} while (0)


#endif  /* __C3_OPERATIONS_H__ */
