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

/* ---------------------------------------------------------------------------
 * C3 CryptoAPI kernel module.
 *
 * ST - 2010-02-11
 * Shikhar Khattar

 * Registration code taken from HIFN_795x.c
 * AES-CTR (RFC 3686) code taken from ctr.c
 * SG Chaining code taken from talitos.c
 *
 * Tested with Linux Kernel 2.6.32 + crypto/authenc.c, patches(from 2.6.34) on
 * Spear300 Rev BA and Spear1310
 * ---------------------------------------------------------------------------

TODO:
 * FIXME:Un-Aligned I/O Scatterlists
 * IPSEC with single pass / single request to C3 HW.
 * Memory leak: Data(ctx) is not freed if user calls INIT,APPEND but not final.
 * Software Fallback in case of failure (especially for
 * hash init/update/final ... )
 * Error checking
 * ...
 */

#include "c3_common.h"
#include "c3_cryptoapi.h"
#include "c3_driver_core.h"
#include "c3_driver_interface.h"

/* c3_driver_interface.c */
extern void c3_unmap_sg(
	struct scatterlist *sg, unsigned int num_sg,
	enum dma_data_direction dir, int chained);
extern void c3_unmap_sg_chain(
	struct scatterlist *sg, enum dma_data_direction dir);
extern int count_sg(
	struct scatterlist *sg_list, int nbytes, int *chained);
extern unsigned int count_sg_total(
	struct scatterlist *sg_list, int nbytes);
extern int c3_map_sg(
	struct scatterlist *sg, unsigned int nents,
	enum dma_data_direction dir, int chained);

/* ===========================================================================
 * ===========================================================================
 */


/* AES CALLBACK FUNCTION called by C3
 *  Status != 0 in case of error
 */

/* TODO: Not tested for sg_src != sg_dst. Also, num_src_sg = num_dst_sg. */
static void callback_aes(void *param, unsigned int status)
{
	unsigned int i = 0, j = 0;
#ifdef CRYPTOAPI_DEBUG
	unsigned char *op;
#endif
	struct ablkcipher_request *req = (struct ablkcipher_request *) param;
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(
		req);

	if (status) {
		C3_ERROR(
			"C3 Internal Error");
		status = -EINVAL;
	}

	/* Temporary: In case of misaligned Input Scatterlist, the input data
	 * is copied into a linear buffer allocated using kmalloc
	 * (guaranteed to be 4 byte aligned for 32 bit CPUs).
	 * In case linear memory cannot be allocated atomically,
	 * the request will fail. The output is placed in the same location
	 * and copied back to the output scatterlist. */

	/* Padding is only in the case of AES-CTR and AES-RFC3686-CTR */

	if (!req_ctx->copy) {
		/* We mapped the SG's, normal DMA */
		C3_DBG(
			"We mapped the SG's, normal DMA");

		if (req->src == req->dst)
			c3_unmap_sg(
				req_ctx->sg_src, req_ctx->num_src_sg,
				DMA_BIDIRECTIONAL, req_ctx->src_chained);
		else {
			c3_unmap_sg(
				req->src, req_ctx->num_src_sg, DMA_TO_DEVICE,
				req_ctx->src_chained);
			c3_unmap_sg(
				req->dst, req_ctx->num_dst_sg, DMA_FROM_DEVICE,
				req_ctx->dst_chained);
		}
		for (j = 0; j < req_ctx->num_dma_buf; j++) {
			/* If we made a copy of the SG entry
			 * (for AES-CTR/RFC3686 padding) */

			if ((req_ctx->dma_buf[j]).copy) {
				/* SG entry copied, copy back to dest without
				 * the padding bytes */
				C3_DBG(
					"we made a copy of the SG entry "
					"(for AES-CTR/RFC3686 padding)");
				dma_unmap_single(
					NULL,
					(req_ctx->dma_buf[j]).dma_temp_buffer,
					(req_ctx->dma_buf[j]).dma_len
						+ (req_ctx->dma_buf[j]).pad,
					DMA_BIDIRECTIONAL);
				i = sg_copy_from_buffer(
					&req_ctx->sg_dst[j], 1,
					(req_ctx->dma_buf[j]).temp_buffer,
					(req_ctx->dma_buf[j]).dma_len
						- (req_ctx->dma_buf[j]).pad);
				if (i != (req_ctx->dma_buf[j]).dma_len
					- (req_ctx->dma_buf[j]).pad) {
					C3_ERROR(
						"copying from temp buffer");
					status = -EAGAIN;
				}

				kfree((req_ctx->dma_buf[j]).temp_buffer);
				(req_ctx->dma_buf[j]).temp_buffer = NULL;
			}
		}
	}
	/* FIXME:Temp We copied the entire input to a linear buffer */
	else {
		C3_DBG(
			"We copied the entire input to a linear buffer");
		dma_unmap_single(
			NULL, req_ctx->dma_buf->dma_temp_buffer,
			req_ctx->nbytes + req_ctx->dma_buf->pad,
			DMA_BIDIRECTIONAL);
		/* Copy back just the request */
		i = sg_copy_from_buffer(
			req->dst, count_sg_total(
				req->dst, req_ctx->nbytes),
			req_ctx->dma_buf->temp_buffer, req_ctx->nbytes);
		if (i != req_ctx->nbytes) {
			C3_ERROR(
				"copying from temp buffer");
			status = -EAGAIN;
		}

		kfree(req_ctx->dma_buf->temp_buffer);
		req_ctx->dma_buf->temp_buffer = NULL;
	}

	kfree(req_ctx->dma_buf);
	req_ctx->dma_buf = NULL;
	kfree(req_ctx->c3_ivec);

#ifdef CRYPTOAPI_DEBUG
	op = kmalloc(req->nbytes, GFP_ATOMIC);
	i = sg_copy_to_buffer(req->dst, count_sg_total(req->dst,
			req->nbytes), op,
		req->nbytes);
	if (i != req->nbytes)
		C3_ERROR("Error : copying to temp buffer,"
			" copied = %d, total = %d", i, req->nbytes);
	C3_DBG("Output:");
	C3_DUMP_HEX(op, req->nbytes);
	kfree(op);
#endif

	/* Call application callback */
	if (req->base.complete)
		req->base.complete(&req->base, status);
}

/* ===========================================================================
 * ===========================================================================
 */

/* HASH CALLBACK FUNCTION called by C3
 *  Status != 0 in case of error
 *  Callback for any one request (init, append, final,digest)
 */

/* TODO:Temporary: In case of misaligned Input Scatterlist, the input data is
 * copied into a linear buffer allocated using kmalloc (guaranteed to be 4 byte
 * aligned for 32 bit CPUs). In case linear memory cannot be allocated
 * atomically, the request will fail. */

static void callback_hash(void *param, unsigned int status)
{
	struct ahash_request *req = (struct ahash_request *) param;
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(req);

	if (status) {
		C3_ERROR("C3 Internal Error");
		status = -EINVAL;
	}

	/* TODO: Data(ctx) is not freed if user calls
	 * INIT,APPEND but not FINAL. */
	switch (req_ctx->type) {
	case 2: /* UPDATE/APPEND */
		if (!req_ctx->copy) {
			/* All SG entries are aligned to 4 bytes, Normal DMA */
			C3_DBG(
				"Update/Append : All SG entries are aligned to "
				"4 bytes, Normal DMA");
			c3_unmap_sg(
				req_ctx->sg_src, req_ctx->num_src_sg,
				DMA_TO_DEVICE, req_ctx->src_chained);
		} else {
			/* We made a copy of the whole request */
			C3_DBG(
				"Update/Append : We made a copy of the "
				"whole request");
			dma_unmap_single(
				NULL, req_ctx->dma_buf->dma_temp_buffer,
				req_ctx->dma_buf->dma_len, DMA_TO_DEVICE);

			kfree(req_ctx->dma_buf->temp_buffer);
		}
		kfree(req_ctx->dma_buf);
		break;

	case 4: /* One shot Digest */
		dma_unmap_single(
			NULL, req_ctx->dma_temp_result, C3_DRIVER_SHA1_OUT_LEN,
			DMA_FROM_DEVICE);

		if (req_ctx->temp_result) {
			/* Output was not aligned, so we did a memcpy */
			C3_DBG(
				"Output was not aligned, so we did a memcpy");
			memcpy(
				req->result, req_ctx->temp_result,
				C3_DRIVER_SHA1_OUT_LEN);
			kfree(req_ctx->temp_result);
		}

		if (!req_ctx->copy) {
			/* All SG entries were aligned to 4 bytes, Normal DMA */
			C3_DBG(
				"End/Final: All SG entries were aligned to 4 bytes, "
				"Normal DMA");
			c3_unmap_sg(
				req_ctx->sg_src, req_ctx->num_src_sg,
				DMA_TO_DEVICE, req_ctx->src_chained);
		} else {
			/* We made a copy of the whole request */
			C3_DBG(
				"We made a copy of the whole request");
			dma_unmap_single(
				NULL, req_ctx->dma_buf->dma_temp_buffer,
				req_ctx->dma_buf->dma_len, DMA_TO_DEVICE);

			kfree(req_ctx->dma_buf->temp_buffer);
		}

		kfree(req_ctx->c3_ctx);
		kfree(req_ctx->dma_buf);

#ifdef CRYPTOAPI_DEBUG
		C3_DBG("Output");
		C3_DUMP_HEX(req->result, C3_DRIVER_SHA1_OUT_LEN);
#endif
		break;

	/* Init and End */
	default:
		break;
	}

	/* Call application callback */
	if (req->base.complete)
		req->base.complete(&req->base, status);
}

/* ===========================================================================
 * ===========================================================================
 */

/* AES RFC3686 CTR SETKEY */
static inline int c3_aes_ctr_setkey(
	struct crypto_ablkcipher *cipher, const unsigned char *key,
	unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(
		cipher);
	struct c3_aes_context *ctx = crypto_tfm_ctx(
		tfm);

	if (len > MAX_KEYLEN + CTR_RFC3686_NONCE_SIZE) {
		C3_ERROR(
			"Invalid key length, len > 32");
		return -EINVAL;
	}

	ctx->c3_key = kmalloc(
		len - CTR_RFC3686_NONCE_SIZE, GFP_DMA);
	if (!ctx->c3_key) {
		C3_ERROR(
			"Cant allocate key");
		return -ENOMEM;
	}
	memcpy(
		ctx->nonce, key + (len - CTR_RFC3686_NONCE_SIZE),
		CTR_RFC3686_NONCE_SIZE);
	ctx->c3_key_len = len - CTR_RFC3686_NONCE_SIZE;
	memcpy(
		ctx->c3_key, key, ctx->c3_key_len);

#ifdef CRYPTOAPI_DEBUG
	C3_DBG("AES RFC3686 CTR KEY:");
	C3_DUMP_HEX(ctx->c3_key, ctx->c3_key_len);
	C3_DBG("NONCE:");
	C3_DUMP_HEX(ctx->nonce, 4);
#endif

	return 0;
}

/* ===========================================================================
 * ===========================================================================
 */

/* AES CBC,CTR SETKEY */
static inline int c3_aes_setkey(
	struct crypto_ablkcipher *cipher, const unsigned char *key,
	unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(
		cipher);
	struct c3_aes_context *ctx = crypto_tfm_ctx(
		tfm);

	if (len > MAX_KEYLEN) {
		C3_ERROR(
			"Invalid key length, len > 32");
		return -EINVAL;
	}

	ctx->c3_key = kmalloc(len, GFP_DMA);
	if (!ctx->c3_key) {
		C3_ERROR(
			"Cant allocate key");
		return -ENOMEM;
	}
	memcpy(
		ctx->c3_key, key, len);
	ctx->c3_key_len = len;

#ifdef CRYPTOAPI_DEBUG
	C3_DBG("AES CBC/CTR KEY:");
	C3_DUMP_HEX(ctx->c3_key, ctx->c3_key_len);
#endif

	return 0;
}

/* ===========================================================================
 * ===========================================================================
 */

/* HMAC (SHA 1) SETKEY */
static inline int c3_hmac_sha1_setkey(
	struct crypto_ahash *hash, const unsigned char *key, unsigned int len)
{
	struct crypto_tfm *tfm = crypto_ahash_tfm(
		hash);
	struct c3_hash_context *ctx = crypto_tfm_ctx(
		tfm);

	ctx->c3_key = kmalloc(len, GFP_DMA);
	if (!ctx->c3_key) {
		C3_ERROR(
			"Mem Alloc failed");
		return -ENOMEM;
	}
	memcpy(
		ctx->c3_key, key, len);
	ctx->c3_key_len = len;

#ifdef CRYPTOAPI_DEBUG
	C3_DBG("HMAC SHA 1 KEY");
	C3_DUMP_HEX(ctx->c3_key, ctx->c3_key_len);
#endif

	return 0;
}

/* ===========================================================================
 * ===========================================================================
 */

/* Setup Cipher request */
static int c3_setup_crypto(
	struct ablkcipher_request *req, unsigned char op, unsigned char type,
	unsigned char mode)
{
	int err;
#ifdef CRYPTOAPI_DEBUG
	unsigned char *ip;
	int i = 0;
#endif
	struct c3_aes_context *ctx = crypto_tfm_ctx(
		req->base.tfm);
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(
		req);

	req_ctx->sg_src = req->src;
	req_ctx->sg_dst = req->dst;
	req_ctx->nbytes = req->nbytes;
	req_ctx->mode = mode;
	req_ctx->copy = 0;
	req_ctx->src_chained = 0;
	req_ctx->dst_chained = 0;

	/* RFC 3686 : AES-CTR does not require the plaintext to be padded to a
	 * multiple of the block size. But C3 requires it to be a multiple of
	 * 16 bytes, so we manually add padding in case of
	 * AES-CTR and AES-RFC3686-CTR */

	if (mode == MODE_RFC3686_CTR) {
		/* set up counter block:
		 * Nonce(4) + IV(8) + Counter(4) = 16 bytes */
		req_ctx->c3_ivec = kmalloc(
			CTR_RFC3686_NONCE_SIZE + CTR_RFC3686_IV_SIZE + 4,
			GFP_DMA);
		if (!req_ctx->c3_ivec) {
			C3_ERROR(
				"Cant allocate ivec");
			return -ENOMEM;
		}
		memcpy(req_ctx->c3_ivec, ctx->nonce, CTR_RFC3686_NONCE_SIZE);
		memcpy(req_ctx->c3_ivec + CTR_RFC3686_NONCE_SIZE, req->info,
			CTR_RFC3686_IV_SIZE);

		/* initialize counter portion of counter block - from ctr.c */
		*(__be32 *) (req_ctx->c3_ivec + CTR_RFC3686_NONCE_SIZE
			+ CTR_RFC3686_IV_SIZE) = cpu_to_be32(
			1);
	} else { /* JUST CBC,CTR FOR NOW */
		req_ctx->c3_ivec = kmalloc(AES_IV_LENGTH, GFP_DMA);
		if (!req_ctx->c3_ivec) {
			C3_ERROR(
				"Cant allocate ivec");
			return -ENOMEM;
		}
		memcpy(req_ctx->c3_ivec, req->info, AES_IV_LENGTH);
	}

	req_ctx->num_src_sg = count_sg(
		req_ctx->sg_src, req_ctx->nbytes, &req_ctx->src_chained);
	if (req_ctx->sg_src == req_ctx->sg_dst) {
		req_ctx->num_dst_sg = req_ctx->num_src_sg;
		req_ctx->dst_chained = req_ctx->src_chained;
	} else
		req_ctx->num_dst_sg
			= count_sg(
				req_ctx->sg_dst, req_ctx->nbytes,
				&req_ctx->dst_chained);

	/* TODO:FIXME */
	if (req_ctx->num_dst_sg != req_ctx->num_src_sg) {
		C3_ERROR(
			"Error in I/O Scatterlists, num_dst_sg != num_src_sg");
		return -ENOTSUPP;
	}

#ifdef CRYPTOAPI_DEBUG
	ip = kmalloc(req->nbytes, GFP_ATOMIC);
	i = sg_copy_to_buffer(req->src, count_sg_total(req->src,
			req->nbytes), ip,
		req->nbytes);
	if (i != req->nbytes)
		C3_ERROR("Error : copying to temp buffer, "
			"copied = %d, total = %d", i, req->nbytes);
	C3_DBG("Input:");
	C3_DUMP_HEX(ip, req->nbytes);
	kfree(ip);
#endif

	if (mode == MODE_CTR || mode == MODE_RFC3686_CTR) {
		if (op == OP_ENC)
			err = c3_AES_CTR_encrypt_sg(
				ctx->c3_key, ctx->c3_key_len, req_ctx->c3_ivec,
				0, callback_aes, (void *) req);
		else
			err = c3_AES_CTR_decrypt_sg(
				ctx->c3_key, ctx->c3_key_len, req_ctx->c3_ivec,
				0, callback_aes, (void *) req);
	} else { /* JUST CBC FOR NOW */
		if (op == OP_ENC)
			err = c3_AES_CBC_encrypt_sg(
				ctx->c3_key, ctx->c3_key_len, req_ctx->c3_ivec,
				0, callback_aes, (void *) req);
		else
			err = c3_AES_CBC_decrypt_sg(
				ctx->c3_key, ctx->c3_key_len, req_ctx->c3_ivec,
				0, callback_aes, (void *) req);
	}

	if (!err) {
		/* Operation queued or in progress */
		if (req->base.complete)
			return -EINPROGRESS; /* Callback set */
		else {
			/* No callback set */
			C3_DBG(
				"No callback set");
			callback_aes(
				(void *) req, err); /* For cleanup */
			return err;
		}
	} else {
		/* Internal Error in C3 or queue full */
		if (err == 1)
			return -EAGAIN;
		else
			return err;
	}
}

/* ===========================================================================
 * ===========================================================================
 */

/* Complete Hash Digest request */
static inline int c3_setup_hash_digest(
	struct ahash_request *req, unsigned char mode)
{
	int err = 0;
#ifdef CRYPTOAPI_DEBUG
	unsigned char *ip;
	int i = 0;
#endif

	struct c3_hash_context *ctx = crypto_tfm_ctx(
		req->base.tfm);
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(
		req);

	if (!req->result) {
		C3_ERROR(
			"Null Result Buffer");
		return -EINVAL;
	}

	req_ctx->sg_src = req->src;
	req_ctx->result = req->result;
	req_ctx->src_chained = 0;
	req_ctx->nbytes = req->nbytes;
	req_ctx->temp_result = NULL;

	req_ctx->num_src_sg = count_sg(
		req->src, req->nbytes, &req_ctx->src_chained);
	req_ctx->c3_ctx = NULL;
	req_ctx->type = 4; /* Digest */
	req_ctx->copy = 0;

#ifdef CRYPTOAPI_DEBUG
	ip = kmalloc(req->nbytes, GFP_ATOMIC);
	i = sg_copy_to_buffer(req->src, count_sg_total(req->src,
			req->nbytes), ip,
		req->nbytes);
	if (i != req->nbytes)
		C3_ERROR("Error : copying to temp buffer,"
			"copied = %d, total = %d", i, req->nbytes);
	C3_DBG("Input");
	C3_DUMP_HEX(ip, req->nbytes);
	kfree(ip);
#endif

	C3_DBG(
		"Input size = %d", req->nbytes);

	if (mode == MODE_SHA1)
		err = c3_SHA1_sg(
			callback_hash, (void *) req);
	else {
		if (ctx->c3_key)
			err = c3_SHA1_HMAC_sg(
				ctx->c3_key, ctx->c3_key_len, callback_hash,
				(void *) req);
		else {
			C3_ERROR(
				"No HMAC SHA 1 key set");
			return -ENODATA;
		}
	}

	if (!err) {
		if (req->base.complete)
			/* Operation queued or in progress */
			return -EINPROGRESS; /* Callback set */
		else {
			/* No callback set */
			C3_DBG(
				"No callback set");
			callback_hash(
				(void *) req, err); /* For cleanup */
			return err;
		}
	} else {
		if (err == 1)
			/* Internal Error in C3 or queue full */
			return -EAGAIN;
		else
			return err;
	}
}

/* ===========================================================================
 * ===========================================================================
 */

/* Hash Init Request */
static inline int c3_setup_hash_init(
	struct ahash_request *req, unsigned char mode)
{
	int err = 0;

	struct c3_hash_context *ctx = crypto_tfm_ctx(
		req->base.tfm);
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(
		req);

	req_ctx->c3_ctx = kzalloc(C3_DRIVER_DIGEST_CTX_MAX_SIZE, GFP_DMA);
	if (!req_ctx->c3_ctx) {
		C3_ERROR(
			"Mem Alloc failed");
		return -ENOMEM;
	}

	req_ctx->type = 1; /* INIT */
	req_ctx->temp_result = NULL;

	if (mode == MODE_SHA1)
		err = c3_SHA1_init(
			req_ctx->c3_ctx, callback_hash, (void *) req);
	else {
		if (ctx->c3_key)
			err = c3_SHA1_HMAC_init(
				req_ctx->c3_ctx, ctx->c3_key, ctx->c3_key_len,
				callback_hash, (void *) req);
		else {
			C3_ERROR(
				"No HMAC SHA 1 key set");
			return -ENODATA;
		}
	}

	if (!err) {
		if (req->base.complete)
			/* Operation queued or in progress */
			return -EINPROGRESS; /* Callback set */
		else {
			/* No callback set */
			C3_DBG(
				"No callback set");
			callback_hash(
				(void *) req, err); /* For cleanup */
			return err;
		}
	} else {
		if (err == 1)
			/* Internal Error in C3 or queue full */
			return -EAGAIN;
		else
			return err;
	}
}

/* ===========================================================================
 * ===========================================================================
 */

/* Hash Update Request */
static inline int c3_setup_hash_update(
	struct ahash_request *req, unsigned char mode)
{
	int err;
#ifdef CRYPTOAPI_DEBUG
	unsigned char *ip;
	int i = 0;
#endif

	struct c3_hash_context *ctx = crypto_tfm_ctx(
		req->base.tfm);
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(
		req);

	req_ctx->nbytes = req->nbytes;

	if (!req_ctx->nbytes)
		return 0;

	req_ctx->src_chained = 0;

	req_ctx->sg_src = req->src;
	req_ctx->num_src_sg = count_sg(
		req_ctx->sg_src, req_ctx->nbytes, &req_ctx->src_chained);
	req_ctx->type = 2; /* Update */
	req_ctx->temp_result = NULL;
	req_ctx->copy = 0;

#ifdef CRYPTOAPI_DEBUG
	ip = kmalloc(req->nbytes, GFP_ATOMIC);
	i = sg_copy_to_buffer(req->src, count_sg_total(req->src,
			req->nbytes), ip,
		req->nbytes);
	if (i != req->nbytes)
		C3_ERROR("Error : copying to temp buffer,"
			"copied = %d, total = %d", i, req->nbytes);
	C3_DBG("Input");
	C3_DUMP_HEX(ip, req->nbytes);
	kfree(ip);
#endif

	if (mode == MODE_SHA1)
		err = c3_SHA1_append_sg(
			callback_hash, (void *) req);
	else {
		if (ctx->c3_key)
			err = c3_SHA1_HMAC_append_sg(
				callback_hash, (void *) req);
		else {
			C3_ERROR("No HMAC SHA 1 key set");
			return -ENODATA;
		}
	}

	if (!err) {
		/* Operation queued or in progress */
		if (req->base.complete)
			return -EINPROGRESS; /* Callback set */
		else {
			/* No callback set */
			C3_DBG("No callback set");
			/* For cleanup or sync operations */
			callback_hash((void *) req, err);
			return err;
		}
	} else {
		/* Internal Error in C3 or queue full */
		if (err == 1)
			return -EAGAIN;
		else
			return err;
	}
}

/* ===========================================================================
 * ===========================================================================
 */

/* Hash Final Request */
static inline int c3_setup_hash_final(
	struct ahash_request *req, unsigned char mode)
{
	int err;

	struct c3_hash_context *ctx = crypto_tfm_ctx(
		req->base.tfm);
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(
		req);

	if (req->result)
		req_ctx->result = req->result;
	else {
		C3_ERROR("Null Result Buffer");
		return -EINVAL;
	}

	req_ctx->temp_result = NULL;
	req_ctx->type = 3; /* Final */

	if (mode == MODE_SHA1)
		err = c3_SHA1_end(
			req_ctx->result, req_ctx->c3_ctx, callback_hash,
			(void *) req);
	else {
		if (ctx->c3_key)
			err = c3_SHA1_HMAC_end(
				req_ctx->result, req_ctx->c3_ctx, ctx->c3_key,
				ctx->c3_key_len, callback_hash, (void *) req);
		else {
			C3_ERROR("No HMAC SHA 1 key set");
			return -ENODATA;
		}
	}

	if (!err) {
		/* Operation queued or in progress */
		if (req->base.complete)
			return -EINPROGRESS; /* Callback set */
		else {
			/* No callback set */
			C3_DBG("No callback set");
			callback_hash(
				(void *) req, err); /* For cleanup */
			return err;
		}
	} else {
		/* Internal Error in C3 or queue full */
		if (err == 1)
			return -EAGAIN;
		else
			return err;
	}
}

/* SHA 1 */
static inline int c3_sha1_init(struct ahash_request *req)
{
	return c3_setup_hash_init(req, MODE_SHA1);
}

static inline int c3_sha1_update(struct ahash_request *req)
{
	return c3_setup_hash_update(req, MODE_SHA1);
}

static inline int c3_sha1_final(struct ahash_request *req)
{
	return c3_setup_hash_final(req, MODE_SHA1);
}

static inline int c3_sha1_digest(struct ahash_request *req)
{
	return c3_setup_hash_digest(req, MODE_SHA1);
}

/* HMAC (SHA 1 ) */
static inline int c3_hmac_sha1_init(struct ahash_request *req)
{
	return c3_setup_hash_init(req, MODE_HMAC_SHA1);
}

static inline int c3_hmac_sha1_update(struct ahash_request *req)
{
	return c3_setup_hash_update(req, MODE_HMAC_SHA1);
}

static inline int c3_hmac_sha1_final(struct ahash_request *req)
{
	return c3_setup_hash_final(req, MODE_HMAC_SHA1);
}

static inline int c3_hmac_sha1_digest(struct ahash_request *req)
{
	return c3_setup_hash_digest(req, MODE_HMAC_SHA1);
}

/* AES CBC */
static inline int c3_encrypt_aes_cbc(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_ENC, TYPE_AES, MODE_CBC);
}

static inline int c3_decrypt_aes_cbc(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_DEC, TYPE_AES, MODE_CBC);
}

/* AES CTR */
static inline int c3_encrypt_aes_ctr(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_ENC, TYPE_AES, MODE_CTR);
}

static inline int c3_decrypt_aes_ctr(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_DEC, TYPE_AES, MODE_CTR);
}

/* AES RFC3686 CTR (FOR IPSEC) */
static inline int c3_encrypt_aes_rfc3686_ctr(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_ENC, TYPE_AES, MODE_RFC3686_CTR);
}

static inline int c3_decrypt_aes_rfc3686_ctr(struct ablkcipher_request *req)
{
	return c3_setup_crypto(req, OP_DEC, TYPE_AES, MODE_RFC3686_CTR);
}

/* Init/Exit */
static int c3_cra_aes_init(struct crypto_tfm *tfm)
{
	/* TODO */
	tfm->crt_ablkcipher.reqsize = sizeof(struct c3_aes_request_context);
	return 0;
}

static void c3_cra_aes_exit(struct crypto_tfm *tfm)
{
	/* TODO */
	struct c3_aes_context *ctx = crypto_tfm_ctx(tfm);
	kfree(ctx->c3_key);
}

static int c3_cra_hash_init(struct crypto_tfm *tfm)
{
	/* TODO */
	crypto_ahash_set_reqsize(
		__crypto_ahash_cast(
			tfm), sizeof(struct c3_hash_request_context));
	return 0;
}

static void c3_cra_hash_exit(struct crypto_tfm *tfm)
{
	/* TODO */
	struct c3_hash_context *ctx = crypto_tfm_ctx(
		tfm);

	kfree(ctx->c3_key);
}

/* Algorithms */
static struct c3_aes_template c3_aes_templates[] = {

	{
		.name = "cbc(aes)", .drv_name = "c3_aes_cbc", .bsize =
		AES_BLOCK_SIZE,
		.ablkcipher = {
			.ivsize = AES_IV_LENGTH,
			.geniv = "eseqiv",
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = c3_aes_setkey,
			.encrypt = c3_encrypt_aes_cbc,
			.decrypt = c3_decrypt_aes_cbc,
		},
	},

	{
		.name = "ctr(aes)", .drv_name = "c3_aes_ctr", .bsize =
		AES_BLOCK_SIZE,
		.ablkcipher = {
			.ivsize = AES_IV_LENGTH,
			.geniv = "eseqiv",
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = c3_aes_setkey,
			.encrypt = c3_encrypt_aes_ctr,
			.decrypt = c3_decrypt_aes_ctr,
		},
	},

	{
		.name = "rfc3686(ctr(aes))", .drv_name = "c3_aes_rfc3686_ctr",
		.bsize = 1,
		.ablkcipher = {
			.ivsize = CTR_RFC3686_IV_SIZE,
			.geniv = "seqiv",
			.min_keysize = AES_MIN_KEY_SIZE +
			CTR_RFC3686_NONCE_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE +
			CTR_RFC3686_NONCE_SIZE,
			.setkey = c3_aes_ctr_setkey,
			.encrypt = c3_encrypt_aes_rfc3686_ctr,
			.decrypt = c3_decrypt_aes_rfc3686_ctr,
		},
	},
};

static struct c3_hash_template c3_hash_templates[] = {

	{
		.name = "sha1", .drv_name = "c3_sha1",
		.ahash = {
			.init = c3_sha1_init,
			.update = c3_sha1_update,
			.final = c3_sha1_final,
			.digest = c3_sha1_digest,
			.halg.digestsize = SHA1_DIGEST_SIZE,
			.halg.base = {
				.cra_name = "sha1",
				.cra_driver_name = "c3_sha1",
				.cra_priority = C3_PRIORITY,
				.cra_flags =
				CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize =
				sizeof(struct c3_hash_context),
				.cra_alignmask = 3,
				.cra_type = &crypto_ahash_type,
				.cra_module = THIS_MODULE,
				.cra_init = c3_cra_hash_init,
				.cra_exit = c3_cra_hash_exit,
			},
		},
	},

	{
		.name = "hmac(sha1)", .drv_name = "c3_hmac(sha1)",
		.ahash = {
			.init = c3_hmac_sha1_init,
			.update = c3_hmac_sha1_update,
			.final = c3_hmac_sha1_final,
			.digest = c3_hmac_sha1_digest,
			.setkey = c3_hmac_sha1_setkey,
			.halg.digestsize = SHA1_DIGEST_SIZE,
			.halg.base = {
				.cra_name = "hmac(sha1)",
				.cra_driver_name = "c3_hmac_sha1",
				.cra_priority = C3_PRIORITY,
				.cra_flags =
				CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize =
				sizeof(struct c3_hash_context),
				.cra_alignmask = 3,
				.cra_type = &crypto_ahash_type,
				.cra_module = THIS_MODULE,
				.cra_init = c3_cra_hash_init,
				.cra_exit = c3_cra_hash_exit,
			},
		},
	},
};

static int c3_unregister_alg(void)
{
	int i = 0, err = 0;

	for (i = 0; i < 3; i++) {
		err = crypto_unregister_alg(
			&((c3_aes_templates[i]).alg));
		if (err)
			C3_ERROR("Cannot unregister C3 %s, error = %d",
				c3_aes_templates[i].name, err);

		C3_INF("C3 %s unregistered", c3_aes_templates[i].name);

		if (i < 2) {
			err = crypto_unregister_ahash(
				&((c3_hash_templates[i]) .ahash));
			if (err)
				C3_ERROR("Cannot unregister C3 %s, error=%d",
					c3_hash_templates[i].name, err);

			C3_INF("C3 %s unregistered", c3_hash_templates[i].name);
		}
	}

	return err;
}

/* AES Algorithm common properties */
static int c3_aes_alloc(struct c3_aes_template *t)
{
	int err;

	snprintf(
		t->alg.cra_name, CRYPTO_MAX_ALG_NAME, "%s", t->name);
	snprintf(
		t->alg.cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s", t->drv_name);
	t->alg.cra_priority = C3_PRIORITY;
	t->alg.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC;
	t->alg.cra_blocksize = t->bsize;
	t->alg.cra_ctxsize = sizeof(struct c3_aes_context);
	t->alg.cra_alignmask = 3;
	t->alg.cra_type = &crypto_ablkcipher_type;
	t->alg.cra_module = THIS_MODULE;
	t->alg.cra_u.ablkcipher = t->ablkcipher;
	t->alg.cra_init = c3_cra_aes_init;
	t->alg.cra_exit = c3_cra_aes_exit;

	/* Register AES Alg */
	err = crypto_register_alg(
		&(t->alg));
	if (err) {
		C3_ERROR(
			"Cannot register alg %s", t->name);
		c3_unregister_alg();
	}

	return err;
}

static int c3_hash_alloc(struct c3_hash_template *t)
{
	int err;

	/* Register HASH Alg */
	err = crypto_register_ahash(
		&t->ahash);
	if (err) {
		C3_ERROR(
			"Cannot register alg %s", t->name);
		c3_unregister_alg();
	}

	return err;
}

static int __init c3_mod_init(void)
{
	int err = -ENOMEM;
	int i = 0;

	C3_INF("Allocating AES [CBC,CTR,RFC3686(CTR)], "
		"SHA 1, HMAC(SHA 1) algorithms");
	C3_INF("Note: All operations are Asynchronous");

	for (i = 0; i < 3; i++) {
		err = c3_aes_alloc(&c3_aes_templates[i]);
		if (err) {
			C3_ERROR("Cannot allocate AES %s",
				c3_aes_templates[i].name);
			goto error;
		}
		C3_INF("C3 %s registered", c3_aes_templates[i].name);
	}

	for (i = 0; i < 2; i++) {
		err = c3_hash_alloc(&c3_hash_templates[i]);
		if (err) {
			C3_ERROR("Cannot allocate SHA1 and HMAC(SHA1)");
			goto error;
		}
		C3_INF("C3 %s registered", c3_hash_templates[i].name);
	}

	return 0;

error:
	err = c3_unregister_alg();
	return err;
}

static void __exit c3_mod_exit(void)
{
	C3_INF("Unregistering algorithms AES (CBC and CTR),SHA 1,HMAC(SHA1)");
	C3_INF("Unloading module");
	c3_unregister_alg();
	C3_INF("Exiting");
}

module_init(c3_mod_init);
module_exit(c3_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ST");
MODULE_DESCRIPTION("C3 Linux-Cryptoapi Integration");
