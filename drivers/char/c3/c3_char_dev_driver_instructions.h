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
 * C3 character device driver instructions mapping types & constants
 *
 * ST - 2007-10-19
 * Alessandro Miglietti
 *
 * - 2009-09-08 MGR Added PKA v.7 support.
 * - 2010-02-09 MGR Bug fix: split AES and DES prototypes.
 * - 2011-02-21 SK: Removed SP, GP, MPR functions.
 * ----------------------------------------------------------------- */

#ifndef __C3_CHAR_DEV_DRIVER_INSTRUCTIONS_H__
#define __C3_CHAR_DEV_DRIVER_INSTRUCTIONS_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_driver_core.h"
#include "c3u_types.h"

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define MAX_INSTR_PARAMS_NUMBER                     ((unsigned int)32)

/* --------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------  */

typedef unsigned int (*c3_cipher_randomize_function_t)(
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_cipher_aes_cbc_ctr_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_cipher_aes_ecb_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_cipher_des_cbc_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_cipher_des_ecb_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hash_init_function_t)(
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hash_append_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hash_end_function_t)(
	unsigned char *,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hash_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hmac_init_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hmac_append_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hmac_end_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_hmac_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_exp_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned char *,
	unsigned int,
	sca_t,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_monty_par_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_ecc_mul_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	sca_t,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_ecc_dsa_sign_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	sca_t,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_ecc_dsa_verify_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	sca_t,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_mul_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_add_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_sub_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_cmp_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_mod_red_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_mod_inv_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_mod_add_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_mod_sub_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_arith_monty_mul_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_rsa_crt_function_t)(
	unsigned char *,
	unsigned int,
	unsigned char *,
	unsigned int,
	sca_t,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_mem_reset_function_t)(
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_pka_mem_read_function_t)(
	unsigned char *,
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned int (*c3_rng_function_t)(
	unsigned char *,
	unsigned int,
	c3_callback_ptr_t,
	void *);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


typedef enum {
	C3_CIPHER_AES_CBC_CTR_FUNCTION = 0,
	C3_CIPHER_AES_ECB_FUNCTION,
	C3_CIPHER_DES_CBC_FUNCTION,
	C3_CIPHER_DES_ECB_FUNCTION,
	C3_HASH_INIT_FUNCTION,
	C3_HASH_APPEND_FUNCTION,
	C3_HASH_END_FUNCTION,
	C3_HASH_FUNCTION,
	C3_HMAC_INIT_FUNCTION,
	C3_HMAC_APPEND_FUNCTION,
	C3_HMAC_END_FUNCTION,
	C3_HMAC_FUNCTION,
	C3_PKA_EXP_FUNCTION,
	C3_PKA_MONTY_PAR_FUNCTION,
	C3_PKA_ECC_MUL_FUNCTION,
	C3_RNG_FUNCTION,
	C3_PKA_ECC_DSA_SIGN_FUNCTION,
	C3_PKA_ECC_DSA_VERIFY_FUNCTION,
	C3_PKA_ARITH_MUL_FUNCTION,
	C3_PKA_ARITH_ADD_FUNCTION,
	C3_PKA_ARITH_SUB_FUNCTION,
	C3_PKA_ARITH_CMP_FUNCTION,
	C3_PKA_ARITH_MOD_RED_FUNCTION,
	C3_PKA_ARITH_MOD_INV_FUNCTION,
	C3_PKA_ARITH_MOD_ADD_FUNCTION,
	C3_PKA_ARITH_MOD_SUB_FUNCTION,
	C3_PKA_ARITH_MONTY_MUL_FUNCTION,
	C3_PKA_RSA_CRT_FUNCTION,
	C3_PKA_MEM_RESET_FUNCTION,
	C3_PKA_MEM_READ_FUNCTION,
	C3_CIPHER_RANDOMIZE_FUNCTION,
} c3_function_type_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	c3_instr_code_t instruction;
	void *fun;
	c3_function_type_t function_type;
	unsigned int params_number;
	unsigned int scatter_gather_enable; /* 1 means "on" */
	struct {
		c3_buffer_type_t type;
		unsigned int ptr_index;
		/* If the size of the buffer is a constant,
		 * "size_index" should be placed to -1
		 * and the value stored in "size" */
		unsigned int size_index;
		unsigned int size;

	} params_indexes[MAX_INSTR_PARAMS_NUMBER];

} c3_instr_params_t;

#endif /* __C3_CHAR_DEV_DRIVER_INSTRUCTIONS_H__ */
