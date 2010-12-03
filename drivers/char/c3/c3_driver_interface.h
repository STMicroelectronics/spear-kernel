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
 * C3 driver interface
 *
 * ST - 2007-04-12
 * Alessandro Miglietti
 *
 * - 2009-09-07 MGR Added PKA v.7 support.
 * - 2010-09-16 SK: Removed IPSec requests. Use kernel CryptoAPI.
 * ----------------------------------------------------------------- */

#ifndef __C3_DRIVER_INTERFACE_H__
#define __C3_DRIVER_INTERFACE_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------- */

#ifndef CRYPTOAPI
#define CRYPTOAPI
#endif

#include "c3_types.h"
#include "c3_cryptoapi.h"


/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------- */

#define C3_DRIVER_DIGEST_CTX_MAX_SIZE\
	512

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_DRIVER_SHA512_OUT_LEN                                    64
#define C3_DRIVER_SHA384_OUT_LEN                                    48
#define C3_DRIVER_SHA256_OUT_LEN                                    32
#define C3_DRIVER_SHA1_OUT_LEN\
	20
#define C3_DRIVER_MD5_OUT_LEN\
	16

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_DRIVER_AES_IVEC_SIZE\
	16
#define C3_DRIVER_DES_IVEC_SIZE\
	8
#define C3_DRIVER_3DES_IVEC_SIZE\
	8


#define C3_FVE\
	0x4

/* --------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       AES                               |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_AES_randomize
(
	unsigned char *seed,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CBC_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CBC_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


unsigned int c3_AES_CBC_encrypt_sg
(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


unsigned int c3_AES_CBC_decrypt_sg
(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_ECB_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_ECB_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_encrypt_sg
(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_decrypt_sg
(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_XTS_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_XTS_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       DES                               |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_DES_CBC_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_CBC_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_ECB_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_ECB_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                       3DES                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_3DES_CBC_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_CBC_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_ECB_encrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_ECB_decrypt
(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA1                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA1_init
(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_append_sg
(
	c3_callback_ptr_t callback,
	void *callback_param
);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_end
(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_sg
(
	c3_callback_ptr_t callback,
	void *callback_param
);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_init
(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_append_sg
(
	c3_callback_ptr_t callback,
	void *callback_param
);


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_end
(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_sg
(
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);



/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA256                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA256_init
(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_end
(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_init
(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_end
(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA384                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA384_init
(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_end
(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_init
(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_end
(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SHA512                           |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA512_init
(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_end
(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_init
(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_end
(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        MD5                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_MD5_init
(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_end
(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_init
(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_append
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_end
(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        PKA                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_PKA_monty_exp
(
	unsigned char *sec_data,
	unsigned int sec_data_len,
	unsigned char *pub_data,
	unsigned char *result,
	unsigned int pub_data_len, /* same as result len */
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_monty_par
(
	unsigned char *data,
	unsigned int data_len,
	unsigned char *monty_parameter,
	unsigned int monty_parameter_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mod_exp
(
	unsigned char *sec_data,
	unsigned int sec_data_len,
	unsigned char *pub_data,
	unsigned char *result,
	unsigned int pub_data_len, /* same as result len */
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_mul
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_monty_mul
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        PKA7                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_PKA_ecc_dsa_sign
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_dsa_verify
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mul
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_add
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_sub
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_cmp
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_red
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_inv
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_add
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_sub
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_monty_mul
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_rsa_crt
(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mem_reset
(
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mem_read
(
	unsigned char *in_data,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        MPR                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_MPR_load_and_run
(
	unsigned char *SN_memory_pool,
	unsigned int SN_memory_pool_len,
	unsigned int first_SN_offset,
	unsigned int SNs_number,
	unsigned char *parameters,
	unsigned int parameters_len,
	unsigned char *input,
	unsigned int input_len,
	unsigned char *status,
	unsigned int status_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MPR_run
(
	unsigned char *parameters,
	unsigned int parameters_len,
	unsigned char *input,
	unsigned int input_len,
	unsigned char *status,
	unsigned int status_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                         RNG                             |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_RNG
(
	unsigned char *buffer,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                        SP                            |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SP_send_receive
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned int output_len, /* Make * int pointner? */
	unsigned int standby,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SP_timeout
(
	unsigned char *timeout_value,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SP_read
(
	unsigned int address,
	unsigned char *value,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SP_standby
(
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SP_write
(
	unsigned int address,
	unsigned int value,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||                         GP                              |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_GP_load_firmware
(
	unsigned char *address,
	unsigned int size,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_run
(
	unsigned int code,
	unsigned char *params_address,
	unsigned int params_size,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_setup
(
	unsigned int debug_input_phys_area,
	unsigned int debug_output_phys_area,
	unsigned int debug_area_size,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_dump_memory
(
	unsigned int gp_memory_address,
	unsigned char *buffer_address,
	unsigned int buffer_size,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||            IPSEC crypto functions (no HW channel)       |||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */
/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_ESP4_AES_SHA_out
(
	unsigned char *input,
	unsigned int input_len,
	unsigned int input_cipher_offset,
	unsigned char *cipher_output,
	unsigned int output_cipher_offset,
	unsigned char *cipher_key,
	unsigned int cipher_key_len,
	unsigned char *cipher_ivec,
	unsigned char *digest_output,
	unsigned char *digest_key,
	unsigned int digest_key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_ESP4_AES_SHA_in
(
	unsigned char *input,
	unsigned int input_len,
	unsigned int input_cipher_offset,
	unsigned char *cipher_output,
	unsigned int output_cipher_offset,
	unsigned char *cipher_key,
	unsigned int cipher_key_len,
	unsigned char *cipher_ivec,
	unsigned char *digest_output,
	unsigned char *digest_key,
	unsigned int digest_key_len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef C3_LOCAL_MEMORY_ON

unsigned int c3_write_local_memory
(
	unsigned char *input,
	unsigned int output,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_read_local_memory
(
	unsigned int input,
	unsigned char *output,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
);

#endif  /* #ifdef C3_LOCAL_MEMORY_ON */

#endif  /* __C3_DRIVER_INTERFACE_H__ */
