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
 * C3 Crypto API kernel module.
 *
 * ST - 2010-02-11
 * Shikhar Khattar
 * ----------------------------------------------------------------- */

#ifndef __C3_CYPTOAPI_H__
#define __C3_CYPTOAPI_H__

#include <crypto/algapi.h>
#include <crypto/ctr.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/moduleparam.h>

#define C3_ERROR(S...)							\
	do {								\
		printk(KERN_ERR "[C3 Error]:" S);			\
	} while (0)

#define C3_INF(S...)							\
	do {								\
		printk(KERN_INFO "[C3 Info]:" S);			\
	} while (0)

#ifdef CRYPTOAPI_DEBUG

#define C3_DBG(S...)							\
	do {								\
		printk(KERN_DEBUG "[C3 Debug]:" S);			\
	} while (0)

#define C3_DUMP_HEX(buf, len)						\
	do {								\
		if (!(buf))						\
			break;						\
		print_hex_dump(KERN_DEBUG, "Raw Buffer: ",		\
		DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 0);		\
		printk(KERN_DEBUG "\n");				\
	} while (0)

#else
#define C3_DBG(S...)
#define C3_DUMP_HEX(buf, len)
#endif


#define MAX_KEYLEN 32
#define AES_IV_LENGTH 16

#define OP_ENC 0
#define OP_DEC 1

#define TYPE_AES 0

#define MODE_CBC 0
#define MODE_CTR 1
#define MODE_RFC3686_CTR 2

#define MODE_SHA1 0
#define MODE_HMAC_SHA1 1

#define C3_DRIVER_SHA1_OUT_LEN 20
#define C3_DRIVER_DIGEST_CTX_MAX_SIZE 512

#define C3_PRIORITY 300

#define SG_WALK(sglist, sg, nr, __i)	\
	for (__i = 0, sg = (sglist); __i < (nr);	\
		__i++, sg = scatterwalk_sg_next(sg))

/* One per one mapped entry in SG list */
struct c3_crypto_dma {
	dma_addr_t dma_input;
	dma_addr_t dma_output;          /* Not used for Hash */
	unsigned int dma_len;
	int copy;                       /* Single SG entry copy */

	unsigned char *temp_buffer;
	dma_addr_t dma_temp_buffer;
	/* Number of padding bytes, if needed for AES-CTR. [Padded with 0's] */
	unsigned int pad;
};

struct c3_aes_template {
	char name[CRYPTO_MAX_ALG_NAME];
	char drv_name[CRYPTO_MAX_ALG_NAME];
	unsigned int bsize;         /* Block size */
	struct ablkcipher_alg ablkcipher;
	struct crypto_alg alg;
};

struct c3_hash_template {
	char name[CRYPTO_MAX_ALG_NAME];
	char drv_name[CRYPTO_MAX_ALG_NAME];
	struct ahash_alg ahash;
};

struct c3_aes_context {
	unsigned char *c3_key;
	unsigned int c3_key_len;
	unsigned char nonce[CTR_RFC3686_NONCE_SIZE];
};

/* TODO: Combine into a single req ctx ? */

struct c3_aes_request_context {
	unsigned char *c3_ivec;

	unsigned int num_src_sg;
	unsigned int num_dst_sg;

	unsigned int nbytes;

	struct scatterlist *sg_src;
	struct scatterlist *sg_dst;

	unsigned char mode;
	unsigned int copy; /* If we are making a copy of the whole input */

	struct c3_crypto_dma *dma_buf;
	unsigned int num_dma_buf;

	int src_chained;
	int dst_chained;
};

struct c3_hash_request_context {
	unsigned char *c3_ctx;

	struct scatterlist *sg_src;
	unsigned int num_src_sg;

	unsigned int nbytes;

	unsigned char *result;

	unsigned char *temp_result; /* NULL if output is already aligned */
	dma_addr_t dma_temp_result; /* Result goes here. */

	unsigned int type; /* Type of operation:Init,Update,Final,Digest */
	unsigned int copy; /* If we are making a copy of the whole input */

	struct c3_crypto_dma *dma_buf;
	unsigned int num_dma_buf;

	int src_chained;
};

struct c3_hash_context {
	/* HMAC SHA 1 */
	unsigned char *c3_key;
	unsigned int c3_key_len;
};

#endif /*__C3_CYPTOAPI_H__*/

