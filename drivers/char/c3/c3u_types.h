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
 * C3 types (user-space library)
 *
 * ST - 2007-05-21
 * Alessandro Miglietti
 * ----------------------------------------------------------------- */

#ifndef __C3U_TYPES_H__
#define __C3U_TYPES_H__

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define C3_CDD_MAX_PARAMS                               ((unsigned int)64)
#define C3_CDD_MAX_BUFFERS                              ((unsigned int)64)

/* --------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------  */

typedef enum {
	/* AES */
	C3_CDD_AES_CBC_decrypt = 0,
	C3_CDD_AES_CBC_encrypt,
	C3_CDD_AES_ECB_decrypt,
	C3_CDD_AES_ECB_encrypt,
	C3_CDD_AES_CTR_decrypt,
	C3_CDD_AES_CTR_encrypt,

	/* DES */
	C3_CDD_DES_CBC_decrypt,
	C3_CDD_DES_CBC_encrypt,
	C3_CDD_DES_ECB_decrypt,
	C3_CDD_DES_ECB_encrypt,

	/* 3DES */
	C3_CDD_3DES_CBC_decrypt,
	C3_CDD_3DES_CBC_encrypt,
	C3_CDD_3DES_ECB_decrypt,
	C3_CDD_3DES_ECB_encrypt,

	/* SHA1 */
	C3_CDD_HASH_SHA1_init,
	C3_CDD_HASH_SHA1_append,
	C3_CDD_HASH_SHA1_end,
	C3_CDD_HASH_SHA1,

	C3_CDD_HMAC_SHA1_init,
	C3_CDD_HMAC_SHA1_append,
	C3_CDD_HMAC_SHA1_end,
	C3_CDD_HMAC_SHA1,

	/* SHA256 */
	C3_CDD_HASH_SHA256_init,
	C3_CDD_HASH_SHA256_append,
	C3_CDD_HASH_SHA256_end,
	C3_CDD_HASH_SHA256,

	C3_CDD_HMAC_SHA256_init,
	C3_CDD_HMAC_SHA256_append,
	C3_CDD_HMAC_SHA256_end,
	C3_CDD_HMAC_SHA256,

	/* SHA384 */
	C3_CDD_HASH_SHA384_init,
	C3_CDD_HASH_SHA384_append,
	C3_CDD_HASH_SHA384_end,
	C3_CDD_HASH_SHA384,

	C3_CDD_HMAC_SHA384_init,
	C3_CDD_HMAC_SHA384_append,
	C3_CDD_HMAC_SHA384_end,
	C3_CDD_HMAC_SHA384,

	/* SHA512 */
	C3_CDD_HASH_SHA512_init,
	C3_CDD_HASH_SHA512_append,
	C3_CDD_HASH_SHA512_end,
	C3_CDD_HASH_SHA512,

	C3_CDD_HMAC_SHA512_init,
	C3_CDD_HMAC_SHA512_append,
	C3_CDD_HMAC_SHA512_end,
	C3_CDD_HMAC_SHA512,

	/* MD5 */
	C3_CDD_HASH_MD5_init,
	C3_CDD_HASH_MD5_append,
	C3_CDD_HASH_MD5_end,
	C3_CDD_HASH_MD5,

	C3_CDD_HMAC_MD5_init,
	C3_CDD_HMAC_MD5_append,
	C3_CDD_HMAC_MD5_end,
	C3_CDD_HMAC_MD5,

	/* PKA */
	C3_CDD_PKA_monty_exp,
	C3_CDD_PKA_monty_par,
	C3_CDD_PKA_mod_exp,
	C3_CDD_PKA_ecc_mul,
	C3_CDD_PKA_ecc_monty_mul,

	/* RNG */
	C3_CDD_RNG,

	/* PKA7 */
	C3_CDD_PKA_ecc_dsa_sign,
	C3_CDD_PKA_ecc_dsa_verify,
	C3_CDD_PKA_arith_mul,
	C3_CDD_PKA_arith_add,
	C3_CDD_PKA_arith_sub,
	C3_CDD_PKA_arith_cmp,
	C3_CDD_PKA_arith_mod_red,
	C3_CDD_PKA_arith_mod_inv,
	C3_CDD_PKA_arith_mod_add,
	C3_CDD_PKA_arith_mod_sub,
	C3_CDD_PKA_arith_monty_mul,
	C3_CDD_PKA_rsa_crt,
	C3_CDD_PKA_mem_reset,
	C3_CDD_PKA_mem_read,

	/*TO DO: I put new func here to be sure not to mess up things, temp!*/
	C3_CDD_AES_randomize,
	C3_CDD_AES_XTS_decrypt,
	C3_CDD_AES_XTS_encrypt,

} c3_instr_code_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef unsigned long c3_cdd_param_t;


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef enum {
	C3_BUFFER_INPUT = 0,
	C3_BUFFER_OUTPUT = 1

} c3_buffer_type_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	c3_buffer_type_t type;
	unsigned char *user_ptr;
	unsigned char *kernel_ptr;
	unsigned int size;

} c3_buffer_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	unsigned int buffers_number;
	c3_buffer_t buffers[C3_CDD_MAX_BUFFERS];

} c3_buffer_list_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	c3_instr_code_t instruction;
	unsigned int params_number;
	c3_cdd_param_t params[C3_CDD_MAX_PARAMS];
	/* Private field: user-space caller shouldn't use it */
	c3_buffer_list_t buffer_list;

} c3_cdd_param_list_t;

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef enum {
	C3_IOCTL_CMD_RECEIVE_MESSAGE,
	C3_IOCTL_CMD_SEND_MESSAGE

} c3_ioctl_cmd_t;

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	unsigned char *msg;
	unsigned int msg_size;

} c3_ioctl_arg_t;

#endif /* #define __C3_TYPES_H__ */
