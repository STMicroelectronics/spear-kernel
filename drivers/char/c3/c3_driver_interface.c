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
 * - 2009-09-08 MGR Added PKA v.7 support.
 * - 2010-01-21 MGR Moved from consistent_sync(...) to dma_map_single(NULL,..)
 * - 2010-05-20 SK: Added extra functions to integrate C3 with scatterlists and
 * Linux CryptoAPI
 * - 2010-09-16 SK: Re-wrote DMA handling part.
 * - 2010-09-16 SK: Removed IPSec requests. Support added through the kernel
 * CryptoAPI.
 * - 2011-21-02 SK: Removed Sp, GP, MPR functions.
 * ----------------------------------------------------------------- */

/* TODO:
 * 1. Change CDD SG implementation to match the linux kernel Scatterlist API.
 * 2. SG Alignment issues.
 * 3. Replace Macros ...
 * 4. Export all the algos
 */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"

#include "c3_driver_core.h"
#include "c3_driver_interface.h"
#include "c3_irq.h"
#include "c3_mpcm.h"

#ifdef CRYPTOAPI

/* Count and return the number of entries in the scatterlist including chaining
 * sg (len = 0)
 * Needed for sg_copy_to/from_buffer */
inline unsigned int count_sg_total(struct scatterlist *sg_list, int nbytes)
{
	int sg_nents = 0;
	struct scatterlist *sg = sg_list;

	while (nbytes) {
		sg_nents++;
		if (sg->length > nbytes)
			break;
		nbytes -= sg->length;
		sg = sg_next(sg);
	}

	C3_DBG("No of total entries in sg = %d", sg_nents);

	return sg_nents;
}
EXPORT_SYMBOL(count_sg_total);

/* Count and return the number of entries in the scatterlist excluding chaining
 * sg (len != 0) */
inline int count_sg(struct scatterlist *sg_list, int nbytes, int *chained)
{
	struct scatterlist *sg = sg_list;
	int sg_nents = 0;

	*chained = 0;
	while (nbytes > 0) {
		sg_nents++;
		nbytes -= sg->length;
		if (!sg_is_last(sg) && (sg + 1)->length == 0) {
			*chained = 1;
			C3_DBG("Scatterlist is chained");
		}
		sg = scatterwalk_sg_next(sg);
	}

	C3_DBG("No of entries in sg = %d", sg_nents);

	return sg_nents;
}
EXPORT_SYMBOL(count_sg);

/* Unmap chainned SG list */
inline void c3_unmap_sg_chain(struct scatterlist *sg,
			      enum dma_data_direction dir)
{
	while (sg) {
		dma_unmap_sg(NULL, sg, 1, dir);
		sg = scatterwalk_sg_next(sg);
	}
}
EXPORT_SYMBOL(c3_unmap_sg_chain);

/* Unmap SG list */
inline void c3_unmap_sg(struct scatterlist *sg, unsigned int num_sg,
			enum dma_data_direction dir,
			int chained)
{
	if (chained)
		c3_unmap_sg_chain(sg, dir);
	else
		dma_unmap_sg(NULL, sg, num_sg, dir);
}
EXPORT_SYMBOL(c3_unmap_sg);

/* Map SG list */
inline int c3_map_sg(struct scatterlist *sg, unsigned int nents,
		     enum dma_data_direction dir,
		     int chained)
{
	int n = 0;

	if (chained) {
		while (sg) {
			n = dma_map_sg(NULL, sg, 1, dir);
			if (!n) {
				C3_ERROR("Cannot map SG");
				return -EINVAL;
			}
			n++;
			sg = scatterwalk_sg_next(sg);
		}
	} else {
		n = dma_map_sg(NULL, sg, nents, dir);
		if (!n) {
			C3_ERROR("Cannot map SG");
			return -EINVAL;
		}
	}
	return n;
}
EXPORT_SYMBOL(c3_map_sg);

/*
 *
 *-----------------------------------------------------------------------------
 * CRYPTOAPI DMA SETUP FOR CIPHERS (AES-CBC,CTR,RFC3686CTR modes for now)
 *-----------------------------------------------------------------------------
 */

/* TODO 1: */
/* Temporary: In case of misaligned Input Scatterlist, the input data is copied
 * into a linear buffer allocated using kmalloc
 * (guaranteed to be 4 byte aligned for 32 bit CPUs). The output is placed in
 * the same location and copied back to the
 * output scatterlist. In case linear memory cannot be allocated atomically,
 * the request will fail. */

/* TODO 2: Split and rewrite ? */

/* TODO 3: Not tested for sg_src != sg_dst. Also, num_src_sg = num_dst_sg. */

static int c3_setup_crypto_dma(struct c3_aes_request_context *req_ctx)
{
	unsigned int len = 0;
	unsigned int ptr;
	int i = 0, j = 0;

	/* count can be <= num_sg. See Documentation/dma-api.txt */
	int count_src = 0, count_dst = 0;
	struct scatterlist *sg;
	struct scatterlist *sg_d = NULL;

	struct scatterlist *sg_src = req_ctx->sg_src;
	struct scatterlist *sg_dst = req_ctx->sg_dst;

	unsigned int num_sg = req_ctx->num_src_sg;
	unsigned int num_bytes = req_ctx->nbytes;

	len = num_bytes;

	if (len % AES_BLOCK_SIZE != 0 && req_ctx->mode == MODE_CBC) {
		C3_ERROR("AES-CBC:Input len is not a multiple of 16");
		return -EINVAL;
	}

	/* Checking if source and destination scatterlists are aligned */
	SG_WALK(sg_src, sg, num_sg, i) {
		ptr = (unsigned long)(page_address(sg_page(sg))) + (sg)->offset;
		if (ptr % 4 != 0) {
			/* TODO:Not 4 byte aligned */
			req_ctx->copy = 1;
			break;
		}
	}

	if (sg_src != sg_dst && !req_ctx->copy) {
		SG_WALK(sg_dst, sg, num_sg, i) {
			ptr = (unsigned long)(page_address(sg_page(sg))) +
			      (sg)->offset;
			if (ptr % 4 != 0) {
				/* TODO:Not 4 byte aligned */
				req_ctx->copy = 1;
				break;
			}
		}
	}

	if (!req_ctx->copy) {
		/* All SG entries (input and output) are aligned to 4 bytes */
		C3_DBG("All aligned");

		req_ctx->num_dma_buf = num_sg;
		req_ctx->dma_buf =
			kmalloc(req_ctx->num_dma_buf *
				sizeof(struct c3_crypto_dma),
				GFP_ATOMIC);
		if (!req_ctx->dma_buf) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		if (sg_src == sg_dst) {
			count_src = c3_map_sg(sg_src, num_sg, DMA_BIDIRECTIONAL,
					      req_ctx->src_chained);
			if (count_src < 0)
				return count_src;
		} else {
			count_src = c3_map_sg(sg_src, num_sg, DMA_TO_DEVICE,
					      req_ctx->src_chained);
			count_dst = c3_map_sg(sg_dst, num_sg, DMA_FROM_DEVICE,
					      req_ctx->dst_chained);
			if (count_src < 0 || count_dst < 0)
				return -EINVAL;

			sg_d = sg_dst;  /* For the next loop */
		}

		SG_WALK(sg_src, sg, count_src, j) {
			len = min(num_bytes, sg_dma_len(sg));
			num_bytes -= len;

			if (len % AES_BLOCK_SIZE == 0) {
				/* Normal case */
				(req_ctx->dma_buf[j]).dma_input =
					sg_dma_address(sg);

				if (sg_src == sg_dst)
					(req_ctx->dma_buf[j]).dma_output =
						(req_ctx->dma_buf[j]).dma_input;
				else
					(req_ctx->dma_buf[j]).dma_output =
						sg_dma_address(sg_d);

				(req_ctx->dma_buf[j]).copy = 0;
				(req_ctx->dma_buf[j]).temp_buffer = NULL;
				(req_ctx->dma_buf[j]).dma_temp_buffer = 0;
				(req_ctx->dma_buf[j]).dma_len = len;
			} else {
				/* Manually adding padding because C3 requires
				 * it and copying to a linear buffer */

				(req_ctx->dma_buf[j]).pad = AES_BLOCK_SIZE -
							    (len %
							     AES_BLOCK_SIZE);
				(req_ctx->dma_buf[j]).temp_buffer = kzalloc(
					len + (req_ctx->dma_buf[j]).pad,
					GFP_ATOMIC); /* For padding */
				if (!(req_ctx->dma_buf[j]).temp_buffer) {
					C3_ERROR("Out of memory");
					c3_unmap_sg(sg_src, num_sg,
						    DMA_BIDIRECTIONAL,
						    req_ctx->src_chained);
					return -ENOMEM;
				}
				/* Copy just the request */
				i = sg_copy_to_buffer(
						sg, 1,
						(req_ctx->dma_buf[j]).
						temp_buffer, len);
				if (i != len) {
					C3_ERROR(
						"Error in copying to tmp buf");
					c3_unmap_sg(sg_src, num_sg,
						    DMA_BIDIRECTIONAL,
						    req_ctx->src_chained);
					return -EAGAIN;
				}

				(req_ctx->dma_buf[j]).dma_temp_buffer =
					dma_map_single(
						NULL,
						(req_ctx->dma_buf[j]).
						temp_buffer, len +
						(req_ctx->dma_buf[j]).
						pad,
						DMA_BIDIRECTIONAL);
				if (dma_mapping_error(NULL,
						      (req_ctx->dma_buf[j]).
						      dma_temp_buffer)) {
					C3_ERROR("DMA mapping error");
					return C3_ERR;
				}

				(req_ctx->dma_buf[j]).dma_input =
					(req_ctx->dma_buf[j]).dma_temp_buffer;

				/* In case sg_src != sg_dst, we still have to
				 * output to a temp buffer because otherwise C3
				 * will write the
				 * complete output back to the output sg. We
				 * only want the initial data  */
				(req_ctx->dma_buf[j]).dma_output =
					(req_ctx->dma_buf[j]).dma_temp_buffer;
				(req_ctx->dma_buf[j]).copy = 1;
				(req_ctx->dma_buf[j]).dma_len = len +
								(req_ctx->
								 dma_buf[j]).
								pad;
			}
			/* Runs same as "for_each_sg" sg_src loop */
			if (sg_src != sg_dst)
				sg_d = scatterwalk_sg_next(sg_d);
		}
	} else {
		/* TODO: Not 4 byte aligned,copying whole input into a linear
		 * buffer. The result is also put into that buffer. */
		req_ctx->num_dma_buf = 1;
		req_ctx->dma_buf = kmalloc(sizeof(struct c3_crypto_dma),
					   GFP_ATOMIC);
		if (!req_ctx->dma_buf) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		if (num_bytes % AES_BLOCK_SIZE == 0) {
			req_ctx->dma_buf->pad = 0;
		} else {
			/* AES-CTR,AES-RFC3686-CTR */
			req_ctx->dma_buf->pad = AES_BLOCK_SIZE -
				(num_bytes % AES_BLOCK_SIZE);
		}

		req_ctx->dma_buf->temp_buffer = kzalloc(
			num_bytes + req_ctx->dma_buf->pad,
			GFP_ATOMIC);
		if (!req_ctx->dma_buf->temp_buffer) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		/* Copy just the request */
		i = sg_copy_to_buffer(sg_src, count_sg_total(sg_src,
							     num_bytes),
				      req_ctx->dma_buf->temp_buffer, num_bytes);
		if (i != num_bytes) {
			C3_ERROR("Error copying from scatterlist to buffer.");
			return -EAGAIN;
		}

		req_ctx->dma_buf->dma_temp_buffer = dma_map_single(
			NULL, req_ctx->dma_buf->temp_buffer, num_bytes +
			req_ctx->dma_buf->pad, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(NULL,
				      req_ctx->dma_buf->dma_temp_buffer)) {
			C3_ERROR("DMA mapping error");
			return C3_ERR;
		}
		req_ctx->dma_buf->dma_input = req_ctx->dma_buf->dma_temp_buffer;
		req_ctx->dma_buf->dma_output =
			req_ctx->dma_buf->dma_temp_buffer;
		req_ctx->dma_buf->dma_len = num_bytes + req_ctx->dma_buf->pad;
		req_ctx->dma_buf->copy = 1;
	}

	return 0;
}


/*
 *
 *-----------------------------------------------------------------------------
 * CRYPTOAPI DMA SETUP FOR HASH (HMAC-SHA1, SHA1 for now)
 *-----------------------------------------------------------------------------
 */

/* TODO 1: */
/* Temporary: In case of misaligned Input Scatterlist, the input data is copied
 * into a linear buffer allocated using kmalloc
 * (guaranteed to be 4 byte aligned for 32 bit CPUs). In case linear memory
 * cannot be allocated atomically, the request will fail. */

static int c3_setup_hash_dma(struct c3_hash_request_context *req_ctx)
{
	unsigned int len = 0;
	unsigned int ptr;
	int i = 0;
	/* count can be <= num_sg. See Documentation/dma-api.txt */
	int count = 0;

	struct scatterlist *sg;
	struct scatterlist *sg_src = req_ctx->sg_src;

	unsigned int input_len = req_ctx->nbytes;

	len = input_len;

	/* Setup Input */

	SG_WALK(sg_src, sg, req_ctx->num_src_sg, i) {
		ptr = (unsigned long)(page_address(sg_page(sg))) + sg->offset;
		if (ptr % 4 != 0 ||
		    ((sg->length) % 4 != 0 && !sg_is_last(sg))) {
			/* TODO:Not 4 byte aligned or len % 4 != 0 except last
			 */
			req_ctx->copy = 1;
			break;
		}
	}

	if (!req_ctx->copy) {
		/* All SG entries are aligned to 4 bytes, Normal DMA */

		C3_DBG("Input aligned");

		req_ctx->num_dma_buf = req_ctx->num_src_sg;
		req_ctx->dma_buf =
			kmalloc(req_ctx->num_dma_buf *
				sizeof(struct c3_crypto_dma),
				GFP_ATOMIC);
		if (!req_ctx->dma_buf) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		count = c3_map_sg(sg_src, req_ctx->num_src_sg, DMA_TO_DEVICE,
				  req_ctx->src_chained);
		if (count < 0)
			return count;

		SG_WALK(sg_src, sg, count, i) {
			/* Normal case */
			len = min(input_len, sg_dma_len(sg));
			input_len -= len;

			/* Last append can be of any length */
			if ((len % 4 != 0 && !sg_is_last(sg))) {
				C3_ERROR(
					"Scatterlist Input len is not a "
					"multiple of 4, len = %d", len);
				return -EINVAL;
			}

			(req_ctx->dma_buf[i]).dma_input = sg_dma_address(sg);
			(req_ctx->dma_buf[i]).dma_len = len;
		}
		req_ctx->dma_buf->copy = 0;
		req_ctx->dma_buf->temp_buffer = NULL;
	} else {
		/* TODO: Not 4 byte aligned,copying whole input into a linear
		 * buffer. */
		req_ctx->num_dma_buf = 1;
		req_ctx->dma_buf = kmalloc(sizeof(struct c3_crypto_dma),
			GFP_ATOMIC);
		if (!req_ctx->dma_buf) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		req_ctx->dma_buf->temp_buffer = kmalloc(input_len,
			GFP_ATOMIC);
		if (!req_ctx->dma_buf->temp_buffer) {
			C3_ERROR("Out of memory");
			return -ENOMEM;
		}

		/* Copy the request */
		i = sg_copy_to_buffer(sg_src, count_sg_total(sg_src,
							     input_len),
				      req_ctx->dma_buf->temp_buffer, input_len);
		if (i != input_len) {
			C3_ERROR("Error copying from scatterlist to buffer.");
			return -EAGAIN;
		}

		req_ctx->dma_buf->dma_temp_buffer = dma_map_single(
			NULL, req_ctx->dma_buf->temp_buffer, input_len,
			DMA_TO_DEVICE);
		if (dma_mapping_error(NULL,
				      req_ctx->dma_buf->dma_temp_buffer)) {
			C3_ERROR("DMA mapping error");
			return C3_ERR;
		}

		req_ctx->dma_buf->dma_input = req_ctx->dma_buf->dma_temp_buffer;
		req_ctx->dma_buf->dma_len = input_len;
		req_ctx->dma_buf->copy = 1;
	}

	/* Setup Output */

	if (req_ctx->type == 4) {       /* Digest */
		if (((unsigned long)(req_ctx->result)) % 4 == 0) {
			/* Output is aligned, Normal DMA. */
			C3_DBG("Output aligned");
			req_ctx->dma_temp_result = dma_map_single(
				NULL, req_ctx->result, C3_DRIVER_SHA1_OUT_LEN,
				DMA_FROM_DEVICE);
			if (dma_mapping_error(NULL,
					      req_ctx->dma_temp_result)) {
				C3_ERROR("DMA mapping error");
				return C3_ERR;
			}
			req_ctx->temp_result = NULL;
		} else {
			/* Output is unaligned. Copy to new buffer. */
			req_ctx->temp_result = kmalloc(C3_DRIVER_SHA1_OUT_LEN,
				GFP_ATOMIC);
			if (!req_ctx->temp_result) {
				C3_ERROR("Out of memory");
				return -ENOMEM;
			}
			req_ctx->dma_temp_result = dma_map_single(
				NULL, req_ctx->temp_result,
				C3_DRIVER_SHA1_OUT_LEN,
				DMA_FROM_DEVICE);
			if (dma_mapping_error(NULL,
					      req_ctx->dma_temp_result)) {
				C3_ERROR("DMA mapping error");
				return C3_ERR;
			}
		}
	}

	return 0;
}

#endif

/* ----------------------------------------------------------------------------
 * FUNCTIONS
 * --------------------------------------------------------------------------*/

#if defined(C3_LOCAL_MEMORY_ON) && defined(MOVE_CHANNEL_INFO)

unsigned int c3_write_local_memory(
	unsigned char *input,
	unsigned int output,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(MOVE_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, len, DMA_TO_DEVICE);

	MOVE(len, c3_dma_0->dma_addr, output + LOCAL_MEMORY_ADDRESS, prgmem);

	STOP(prgmem);

	C3_END(prgmem);

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_read_local_memory(
	unsigned int input,
	unsigned char *output,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(MOVE_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, output, len, DMA_FROM_DEVICE);

	MOVE(
		len,
		input + LOCAL_MEMORY_ADDRESS,
		c3_dma_0->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

}

#endif  /* #ifdef C3_LOCAL_MEMORY_ON */

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                       AES                               ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_AES_randomize(
	unsigned char *seed,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(AES_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	/* SIZE = 16 bytes */
	C3_PREPARE_DMA(c3_dma_0, seed, 16, DMA_BIDIRECTIONAL);

	AES_RANDOMIZE(
		c3_dma_0->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else
	return C3_ERR;
#endif

}       /*unsigned int c3_AES_randomize*/

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CBC_encrypt(
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
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else{
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
				c3_dma_2->dma_addr,
				C3_DRIVER_AES_IVEC_SIZE,
				prgmem);

		if (input == output) {
			MPCM_EXECUTE_SD(
					MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					input_len,
					prgmem);
		}

#else

		if (flags & C3_FVE) {
			unsigned int lba_size_swap = cpu_to_be32(lba_size);
			struct c3_dma_t *c3_dma_lba;
			void *p_lba_size_swap = &lba_size_swap;
			C3_PREPARE_DMA(c3_dma_lba, p_lba_size_swap, 4,
					DMA_TO_DEVICE);

			AES_ENCRYPT_CBC_FVE_START(
					key_len,
					flags ^ C3_FVE,
					c3_dma_1->dma_addr,
					c3_dma_2->dma_addr,
					c3_dma_lba->dma_addr,
					prgmem);
		} else {
			AES_ENCRYPT_CBC_START(
					key_len,
					flags,
					c3_dma_1->dma_addr,
					c3_dma_2->dma_addr,
					prgmem);
		}

		if (input == output) {
			AES_ENCRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_ENCRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);
		}

#endif

		STOP(prgmem);
		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}       /* unsigned int c3_AES_CBC_encrypt */
EXPORT_SYMBOL(c3_AES_CBC_encrypt);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CBC_decrypt(
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
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{

#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif
		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
				c3_dma_2->dma_addr,
				C3_DRIVER_AES_IVEC_SIZE,
				prgmem);

		if (input == output) {
			MPCM_EXECUTE_SD(
					MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					input_len,
					prgmem);
		}

#else

		if (flags & C3_FVE) {
			struct c3_dma_t *c3_dma_lba;
			unsigned int lba_size_swap = cpu_to_be32(lba_size);
			void *p_lba_size_swap = &lba_size_swap;
			C3_PREPARE_DMA(c3_dma_lba, p_lba_size_swap, 4,
					DMA_TO_DEVICE);

			AES_DECRYPT_CBC_FVE_START(
					key_len,
					flags ^ C3_FVE,
					c3_dma_1->dma_addr,
					c3_dma_2->dma_addr,
					c3_dma_lba->dma_addr,
					prgmem);
		} else {
			AES_DECRYPT_CBC_START(
					key_len,
					flags,
					c3_dma_1->dma_addr,
					c3_dma_2->dma_addr,
					prgmem);
		}

		if (input == output) {
			AES_DECRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_DECRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);
		}

#endif

		STOP(prgmem);
		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CBC_decrypt);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef CRYPTOAPI
/*
 *
 *----------------------------------------------------------------------------
 * CRYPTOAPI: AES-CBC Encrypt (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_AES_CBC_encrypt_sg(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))

	int err2 = 0;
	int i = 0;
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	struct ablkcipher_request *req =
		(struct ablkcipher_request *)callback_param;
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(req);

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_0, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

#if defined(MPCM_CHANNEL_INFO)

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
		c3_dma_0->dma_addr,
		key_len,
		prgmem);
	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);
#else
	AES_ENCRYPT_CBC_START(key_len, flags, c3_dma_0->dma_addr,
			      c3_dma_1->dma_addr,
			      prgmem);
#endif

	err2 = c3_setup_crypto_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++) {

#if defined(MPCM_CHANNEL_INFO)
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX,
			(req_ctx->dma_buf[i]).dma_input,
			(req_ctx->dma_buf[i]).dma_output,
			(req_ctx->dma_buf[i]).dma_len,
			prgmem);
#else
		AES_ENCRYPT_CBC_APPEND((req_ctx->dma_buf[i]).dma_len,
				       (req_ctx->dma_buf[i]).dma_input,
				       (req_ctx->dma_buf[i]).dma_output,
				       prgmem);
#endif
	}

	STOP(prgmem);
	C3_END(prgmem);
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CBC_encrypt_sg);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/*
 *
 *----------------------------------------------------------------------------
 * CRYPTOAPI: AES-CBC Decrypt (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_AES_CBC_decrypt_sg(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))

	int err2 = 0;
	int i = 0;
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	struct ablkcipher_request *req =
		(struct ablkcipher_request *)callback_param;
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(req);

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_0, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

#if defined(MPCM_CHANNEL_INFO)
	MPCM_EXECUTE_S(
		MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX,
		c3_dma_0->dma_addr,
		key_len,
		prgmem);

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);
#else
	AES_DECRYPT_CBC_START(key_len, flags, c3_dma_0->dma_addr,
			      c3_dma_1->dma_addr,
			      prgmem);
#endif

	err2 = c3_setup_crypto_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++) {

#if defined(MPCM_CHANNEL_INFO)
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX,
			(req_ctx->dma_buf[i]).dma_input,
			(req_ctx->dma_buf[i]).dma_output,
			(req_ctx->dma_buf[i]).dma_len,
			prgmem);
#else
		AES_DECRYPT_CBC_APPEND((req_ctx->dma_buf[i]).dma_len,
				       (req_ctx->dma_buf[i]).dma_input,
				       (req_ctx->dma_buf[i]).dma_output,
				       prgmem);
#endif
	}

	STOP(prgmem);
	C3_END(prgmem);
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CBC_decrypt_sg);

#endif  /* #ifdef CRYPTOAPI */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_ECB_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_ECB))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_2, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		if (input == output) {

			MPCM_EXECUTE_SD(
					MPCM_AES_ECB_ENC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_ECB_ENC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_2->dma_addr,
					input_len,
					prgmem);
		}

#else

		AES_ENCRYPT_ECB_START(
				key_len,
				flags,
				c3_dma_1->dma_addr,
				prgmem);
		if (input == output) {

			AES_ENCRYPT_ECB_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_ENCRYPT_ECB_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_2->dma_addr,
					prgmem);
		}
#endif

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_ECB_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_ECB))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{

#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_2, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)
		MPCM_EXECUTE_S(
				MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		if (input == output) {

			MPCM_EXECUTE_SD(
					MPCM_AES_ECB_DEC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_ECB_DEC_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_2->dma_addr,
					input_len,
					prgmem);

		}
#else

		AES_DECRYPT_ECB_START(
				key_len,
				flags,
				c3_dma_1->dma_addr,
				prgmem);
		if (input == output) {

			AES_DECRYPT_ECB_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_DECRYPT_ECB_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_2->dma_addr,
					prgmem);

		}

#endif

		STOP(
				prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,  /*just for prototype compatibility*/
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CTR))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{

#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
				c3_dma_2->dma_addr,
				C3_DRIVER_AES_IVEC_SIZE,
				prgmem);
		if (input == output) {
			MPCM_EXECUTE_SD(
					MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					input_len,
					prgmem);
		}

#else

		AES_ENCRYPT_CTR_START(
				key_len,
				flags,
				c3_dma_1->dma_addr,
				c3_dma_2->dma_addr,
				prgmem);

		if (input == output) {
			AES_ENCRYPT_CTR_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_ENCRYPT_CTR_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);
		}

#endif

		STOP(
				prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CTR_encrypt);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_CTR_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int lba_size,  /*just for prototype compatibility*/
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CTR))

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	if ((key_len <= MIN_AES_KEY_LEN) || (key_len > MAX_AES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{

#ifdef AES_CHANNEL_INFO
		C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#else
		C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
#endif

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

#if defined(MPCM_CHANNEL_INFO)

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
				c3_dma_1->dma_addr,
				key_len,
				prgmem);

		MPCM_EXECUTE_S(
				MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
				c3_dma_2->dma_addr,
				C3_DRIVER_AES_IVEC_SIZE,
				prgmem);
		if (input == output) {

			MPCM_EXECUTE_SD(
					MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					input_len,
					prgmem);
		} else {
			MPCM_EXECUTE_SD(
					MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					input_len,
					prgmem);
		}

#else

		AES_DECRYPT_CTR_START(
				key_len,
				flags,
				c3_dma_1->dma_addr,
				c3_dma_2->dma_addr,
				prgmem);

		if (input == output) {

			AES_DECRYPT_CTR_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			AES_DECRYPT_CTR_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);
		}

#endif

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif
}
EXPORT_SYMBOL(c3_AES_CTR_decrypt);

#ifdef CRYPTOAPI
/*
 *
 *----------------------------------------------------------------------------
 * CRYPTOAPI: AES-CTR Encrypt (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_AES_CTR_encrypt_sg(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CTR))

	int err2 = 0;
	int i = 0;
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	struct ablkcipher_request *req =
		(struct ablkcipher_request *)callback_param;
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(req);

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_0, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

#if defined(MPCM_CHANNEL_INFO)

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
		c3_dma_0->dma_addr,
		key_len,
		prgmem);

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);

#else
	AES_ENCRYPT_CTR_START(key_len, flags, c3_dma_0->dma_addr,
			      c3_dma_1->dma_addr,
			      prgmem);
#endif

	err2 = c3_setup_crypto_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++) {

#if defined(MPCM_CHANNEL_INFO)
		MPCM_EXECUTE_SD(
			MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
			(req_ctx->dma_buf[i]).dma_input,
			(req_ctx->dma_buf[i]).dma_output,
			(req_ctx->dma_buf[i]).dma_len,
			prgmem);
#else
		AES_ENCRYPT_CTR_APPEND((req_ctx->dma_buf[i]).dma_len,
				       (req_ctx->dma_buf[i]).dma_input,
				       (req_ctx->dma_buf[i]).dma_output,
				       prgmem);
#endif
	}

	STOP(prgmem);
	C3_END(prgmem);
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CTR_encrypt_sg);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/*
 *
 *----------------------------------------------------------------------------
 * CRYPTOAPI: AES-CTR Decrypt (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_AES_CTR_decrypt_sg(
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	unsigned int flags,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CTR))

	int err2 = 0;
	int i = 0;
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	struct ablkcipher_request *req =
		(struct ablkcipher_request *)callback_param;
	struct c3_aes_request_context *req_ctx = ablkcipher_request_ctx(req);

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_0, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

#if defined(MPCM_CHANNEL_INFO)

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
		c3_dma_0->dma_addr,
		key_len,
		prgmem);

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);

#else

	AES_DECRYPT_CTR_START(key_len, flags, c3_dma_0->dma_addr,
			      c3_dma_1->dma_addr,
			      prgmem);
#endif

	err2 = c3_setup_crypto_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++) {

#if defined(MPCM_CHANNEL_INFO)
		MPCM_EXECUTE_SD(
			MPCM_AES_CTR_APPEND_PROGRAM_INDEX,
			(req_ctx->dma_buf[i]).dma_input,
			(req_ctx->dma_buf[i]).dma_output,
			(req_ctx->dma_buf[i]).dma_len,
			prgmem);
#else
		AES_DECRYPT_CTR_APPEND((req_ctx->dma_buf[i]).dma_len,
				       (req_ctx->dma_buf[i]).dma_input,
				       (req_ctx->dma_buf[i]).dma_output,
				       prgmem);
#endif
	}

	STOP(prgmem);
	C3_END(prgmem);
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_AES_CTR_decrypt_sg);

#endif  /* #ifdef CRYPTOAPI */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_XTS_encrypt(
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
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_3, output, input_len, DMA_FROM_DEVICE);
	}

#if defined(MPCM_CHANNEL_INFO)
	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		key_len,
		prgmem);

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_2->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);

	if (input == output) {
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			input_len,
			prgmem);
	} else {
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX,
			c3_dma_0->dma_addr,
			c3_dma_3->dma_addr,
			input_len,
			prgmem);
	}

#else

	if (flags & C3_FVE) {
		struct c3_dma_t *c3_dma_lba;
		unsigned int lba_size_swap = cpu_to_be32(lba_size);
		void *p_lba_size_swap = &lba_size_swap;
		C3_PREPARE_DMA(c3_dma_lba, p_lba_size_swap, 4, DMA_TO_DEVICE);

		AES_ENCRYPT_XTS_FVE_START(
			key_len,
			flags ^ C3_FVE,
			c3_dma_1->dma_addr,
			c3_dma_2->dma_addr,
			c3_dma_lba->dma_addr,
			prgmem);
	} else {
		AES_ENCRYPT_XTS_START(
			key_len,
			flags,
			c3_dma_1->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);
	}
	if (input == output) {
		AES_ENCRYPT_XTS_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		AES_ENCRYPT_XTS_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_3->dma_addr,
			prgmem);
	}

#endif

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_AES_XTS_encrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_AES_XTS_decrypt(
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
)
{

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&	\
	defined(USE_MPCM_AES_CBC))
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

#ifdef AES_CHANNEL_INFO
	C3_START(AES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#else
	C3_START(MPCM_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);
#endif

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_AES_IVEC_SIZE, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_3, output, input_len, DMA_FROM_DEVICE);
	}

#if defined(MPCM_CHANNEL_INFO)
	MPCM_EXECUTE_S(
		MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		key_len,
		prgmem);

	MPCM_EXECUTE_S(
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX,
		c3_dma_2->dma_addr,
		C3_DRIVER_AES_IVEC_SIZE,
		prgmem);
	if (input == output) {
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			input_len,
			prgmem);
	} else {
		MPCM_EXECUTE_SD(
			MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX,
			c3_dma_0->dma_addr,
			c3_dma_3->dma_addr,
			input_len,
			prgmem);
	}

#else

	if (flags & C3_FVE) {
		unsigned int lba_size_swap = cpu_to_be32(lba_size);
		struct c3_dma_t *c3_dma_lba;
		void *p_lba_size_swap = &lba_size_swap;
		C3_PREPARE_DMA(c3_dma_lba, p_lba_size_swap, 4, DMA_TO_DEVICE);

		AES_DECRYPT_XTS_FVE_START(
			key_len,
			flags ^ C3_FVE,
			c3_dma_1->dma_addr,
			c3_dma_2->dma_addr,
			c3_dma_lba->dma_addr,
			prgmem);
	} else {
		AES_DECRYPT_XTS_START(
			key_len,
			flags,
			c3_dma_1->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);
	}

	AES_DECRYPT_XTS_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		c3_dma_0->dma_addr,
		prgmem);

#endif

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_AES_XTS_decrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                       DES                               ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_DES_CBC_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if ((key_len <= MIN_DES_KEY_LEN) || (key_len > MAX_DES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
		C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_DES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

		DES_ENCRYPT_CBC_START(
				key_len,
				c3_dma_1->dma_addr,
				c3_dma_2->dma_addr,
				prgmem);

		if (input == output) {

			DES_ENCRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			DES_ENCRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);
		}

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}       /* unsigned int c3_DES_CBC_encrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_CBC_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	if ((key_len <= MIN_DES_KEY_LEN) || (key_len > MAX_DES_KEY_LEN)) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
		C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_DES_IVEC_SIZE,
				DMA_TO_DEVICE);

		if (input == output)
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_BIDIRECTIONAL);
		else {
			C3_PREPARE_DMA(c3_dma_0, input, input_len,
					DMA_TO_DEVICE);
			C3_PREPARE_DMA(c3_dma_3, output, input_len,
					DMA_FROM_DEVICE);
		}

		DES_DECRYPT_CBC_START(
				key_len,
				c3_dma_1->dma_addr,
				c3_dma_2->dma_addr,
				prgmem);
		if (input == output) {

			DES_DECRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_0->dma_addr,
					prgmem);
		} else {
			DES_DECRYPT_CBC_APPEND(
					input_len,
					c3_dma_0->dma_addr,
					c3_dma_3->dma_addr,
					prgmem);

		}

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}       /* unsigned int c3_DES_CBC_decrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_ECB_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, output, input_len, DMA_FROM_DEVICE);
	}

	DES_ENCRYPT_ECB_START(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);
	if (input == output) {

		DES_ENCRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		DES_ENCRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);

	}

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_DES_ECB_encrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_DES_ECB_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, output, input_len, DMA_FROM_DEVICE);
	}

	DES_DECRYPT_ECB_START(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);
	if (input == output) {

		DES_DECRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		DES_DECRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);

	}

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_DES_ECB_decrypt */

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                       3DES                              ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_3DES_CBC_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_3DES_IVEC_SIZE, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_3, output, input_len, DMA_FROM_DEVICE);
	}

	TRIPLE_DES_ENCRYPT_CBC_START(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);
	if (input == output) {
		TRIPLE_DES_ENCRYPT_CBC_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		TRIPLE_DES_ENCRYPT_CBC_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_3->dma_addr,
			prgmem);
	}

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_3DES_CBC_encrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_CBC_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *ivec,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;
	struct c3_dma_t *c3_dma_3 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, ivec, C3_DRIVER_3DES_IVEC_SIZE, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_3, output, input_len, DMA_FROM_DEVICE);
	}

	TRIPLE_DES_DECRYPT_CBC_START(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);
	if (input == output) {

		TRIPLE_DES_DECRYPT_CBC_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		TRIPLE_DES_DECRYPT_CBC_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_3->dma_addr,
			prgmem);

	}

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_TRIPLE_DES_CBC_decrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_ECB_encrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

	if (input == output)
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, output, input_len, DMA_FROM_DEVICE);
	}

	TRIPLE_DES_ENCRYPT_ECB_START(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);
	if (input == output) {

		TRIPLE_DES_ENCRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		TRIPLE_DES_ENCRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);
	}

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_TRIPLE_DES_ECB_encrypt */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_3DES_ECB_decrypt(
	unsigned char *input,
	unsigned char *output,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(DES_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(DES_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

	if (input == output) {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_BIDIRECTIONAL);
	} else {
		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_2, output, input_len, DMA_FROM_DEVICE);
	}

	TRIPLE_DES_DECRYPT_ECB_START(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);
	if (input == output) {

		TRIPLE_DES_DECRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_0->dma_addr,
			prgmem);
	} else {
		TRIPLE_DES_DECRYPT_ECB_APPEND(
			input_len,
			c3_dma_0->dma_addr,
			c3_dma_2->dma_addr,
			prgmem);
	}

	STOP(
		prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_3DES_ECB_decrypt */

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        SHA1                             ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA1_init(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);

	SHA1_INIT(prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_SHA1_init */
EXPORT_SYMBOL(c3_SHA1_init);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA1_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		C3_UH_PATCH();

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif
}
EXPORT_SYMBOL(c3_SHA1_append);

#ifdef CRYPTOAPI
/*
 *
 *----------------------------------------------------------------------------
 * CRYPTOAPI: SHA1 Append (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_SHA1_append_sg(
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(UH_CHANNEL_INFO)

	int i = 0, err2 = 0;
	struct c3_dma_t *c3_dma_0 = NULL;

	struct ahash_request *req = (struct ahash_request *)callback_param;
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(req);

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, req_ctx->c3_ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_BIDIRECTIONAL);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr, UH_CHANNEL_INFO.channel_idx, prgmem);

	err2 = c3_setup_hash_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++)
		SHA1_APPEND((req_ctx->dma_buf[i]).dma_len,
			    (req_ctx->dma_buf[i]).dma_input,
			    prgmem);

	C3_UH_PATCH();

	SAVE_CONTEXT(c3_dma_0->dma_addr, UH_CHANNEL_INFO.channel_idx, prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}
EXPORT_SYMBOL(c3_SHA1_append_sg);
#endif

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_end(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA1_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA1_INIT(prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA1_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}       /* unsigned int c3_SHA1_end */
EXPORT_SYMBOL(c3_SHA1_end);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA1_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA1_INIT(prgmem);

	SHA1_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA1_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}       /* unsigned int c3_SHA1 */
EXPORT_SYMBOL(c3_SHA1);

#ifdef CRYPTOAPI
/*
 *----------------------------------------------------------------------------
 * CRYPTOAPI: SHA1 Digest (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_SHA1_sg(
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(UH_CHANNEL_INFO)

	int i = 0, err2 = 0;

	struct ahash_request *req = (struct ahash_request *)callback_param;
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(req);

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	SHA1_INIT(prgmem);

	err2 = c3_setup_hash_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++)
		SHA1_APPEND((req_ctx->dma_buf[i]).dma_len,
			    (req_ctx->dma_buf[i]).dma_input,
			    prgmem);

	SHA1_END(req_ctx->dma_temp_result, prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_SHA1_sg);

#endif

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_init(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (!key_len) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}
	{

		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_FROM_DEVICE);
		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		SHA1_HMAC_INIT(
				key_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_SHA1_HMAC_init);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA1_HMAC_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		C3_UH_PATCH();

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}

#else

	return C3_ERR;

#endif

}
EXPORT_SYMBOL(c3_SHA1_HMAC_append);

#ifdef CRYPTOAPI
/*
 *----------------------------------------------------------------------------
 * CRYPTOAPI: HMAC SHA1 Append (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_SHA1_HMAC_append_sg(
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	int i = 0, err2 = 0;
	struct c3_dma_t *c3_dma_0 = NULL;

	struct ahash_request *req = (struct ahash_request *)callback_param;
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(req);

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, req_ctx->c3_ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_BIDIRECTIONAL);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr, UH_CHANNEL_INFO.channel_idx, prgmem);

	err2 = c3_setup_hash_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++)
		if ((req_ctx->dma_buf[i]).dma_len)
			SHA1_HMAC_APPEND((req_ctx->dma_buf[i]).dma_len,
					 (req_ctx->dma_buf[i]).dma_input,
					 prgmem);

	C3_UH_PATCH();

	SAVE_CONTEXT(c3_dma_0->dma_addr, UH_CHANNEL_INFO.channel_idx, prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}
EXPORT_SYMBOL(c3_SHA1_HMAC_append_sg);
#endif

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC_end(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output, C3_DRIVER_SHA1_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA1_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA1_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_SHA1_HMAC_end */
EXPORT_SYMBOL(c3_SHA1_HMAC_end);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA1_HMAC(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output, C3_DRIVER_SHA1_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA1_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	SHA1_HMAC_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA1_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_SHA1_HMAC */
EXPORT_SYMBOL(c3_SHA1_HMAC);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef CRYPTOAPI
/*
 *----------------------------------------------------------------------------
 * CRYPTOAPI: HMAC SHA1 Digest (scatterlist support)
 *----------------------------------------------------------------------------
 */

unsigned int c3_SHA1_HMAC_sg(
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;

	int i = 0, err2 = 0;

	struct ahash_request *req = (struct ahash_request *)callback_param;
	struct c3_hash_request_context *req_ctx = ahash_request_ctx(req);

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, key, key_len, DMA_TO_DEVICE);

	SHA1_HMAC_INIT(key_len, c3_dma_0->dma_addr, prgmem);

	err2 = c3_setup_hash_dma(req_ctx);
	if (err2) {
		C3_ERROR("Setup DMA error");
		return err2;
	}

	for (i = 0; i < req_ctx->num_dma_buf; i++)
		if ((req_ctx->dma_buf[i]).dma_len)
			SHA1_HMAC_APPEND((req_ctx->dma_buf[i]).dma_len,
					 (req_ctx->dma_buf[i]).dma_input,
					 prgmem);

	SHA1_HMAC_END(key_len, c3_dma_0->dma_addr, req_ctx->dma_temp_result,
		      prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}       /* unsigned int c3_SHA1_HMAC */
EXPORT_SYMBOL(c3_SHA1_HMAC_sg);

#endif

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        SHA256                           ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA256_init(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);

	SHA256_INIT(prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA256_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		C3_UH_PATCH();

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_end(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA256_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA256_INIT(
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA256_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA256_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA256_INIT(
		prgmem);

	SHA256_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA256_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_init(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (!key_len) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
			       DMA_FROM_DEVICE);
		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		SHA256_HMAC_INIT(
			key_len,
			c3_dma_1->dma_addr,
			prgmem);

		SAVE_CONTEXT(
			c3_dma_0->dma_addr,
			UH_CHANNEL_INFO.channel_idx,
			prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
		C3_PREPARE_DMA(c3_dma_1, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);

		RESTORE_CONTEXT(
				c3_dma_1->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA256_HMAC_APPEND(
				input_len,
				c3_dma_0->dma_addr,
				prgmem);

		C3_UH_PATCH();

		SAVE_CONTEXT(
				c3_dma_1->dma_addr,
				UH_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC_end(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, output, C3_DRIVER_SHA256_OUT_LEN,
		       DMA_FROM_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, key, key_len, DMA_TO_DEVICE);

	SHA256_HMAC_INIT(
		key_len,
		c3_dma_2->dma_addr,
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_1->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA256_HMAC_END(
		key_len,
		c3_dma_2->dma_addr,
		c3_dma_0->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA256_HMAC(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO) && !defined(C3_SPEARBASIC_REVA) &&	\
	!defined(C3_SPEARBASIC_REVBA)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output, C3_DRIVER_SHA256_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA256_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	SHA256_HMAC_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA256_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        SHA384                           ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA384_init(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);

	SHA384_INIT(prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}       /* unsigned int c3_SHA384_init */

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA384_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_end(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA384_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA384_INIT(prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA384_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA384_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA384_INIT(prgmem);

	SHA384_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA384_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_init(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (!key_len) {
		printk(C3_KERN_ERR "zero len key not supported\n");
		return C3_ERR;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_FROM_DEVICE);
		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		SHA384_HMAC_INIT(
				key_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA384_HMAC_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC_end(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, output, C3_DRIVER_SHA384_OUT_LEN,
		       DMA_FROM_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, key, key_len, DMA_TO_DEVICE);

	SHA384_HMAC_INIT(
		key_len,
		c3_dma_2->dma_addr,
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_1->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA384_HMAC_END(
		key_len,
		c3_dma_2->dma_addr,
		c3_dma_0->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA384_HMAC(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA384_OUT_LEN,
		       DMA_FROM_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, key, key_len, DMA_TO_DEVICE);

	SHA384_HMAC_INIT(
		key_len,
		c3_dma_2->dma_addr,
		prgmem);

	SHA384_HMAC_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA384_HMAC_END(
		key_len,
		c3_dma_2->dma_addr,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        SHA512                           ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_SHA512_init(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);

	SHA512_INIT(prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " input len is not multiple of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA512_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_end(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA512_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA512_INIT(prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA512_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output, C3_DRIVER_SHA512_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA512_INIT(prgmem);

	SHA512_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA512_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_init(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (!key_len) {
		printk(C3_KERN_ERR "Invalid key\n");
		return C3_ERR;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);
		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_FROM_DEVICE);
		C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

		SHA512_HMAC_INIT(
				key_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}
#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	if (input_len % 4) {
		printk(C3_KERN_ERR " len is not module of 4\n");
		return C3_ERR;
	}

	if (!input_len) {
		printk(C3_KERN_INFO "zero len data not supported\n");
		return C3_SKIP;
	}

	{
		C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback,
				callback_param);

		C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
				DMA_BIDIRECTIONAL);
		C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

		RESTORE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		SHA512_HMAC_APPEND(
				input_len,
				c3_dma_1->dma_addr,
				prgmem);

		SAVE_CONTEXT(
				c3_dma_0->dma_addr,
				UH2_CHANNEL_INFO.channel_idx,
				prgmem);

		STOP(prgmem);

		C3_END(prgmem);
	}

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC_end(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output, C3_DRIVER_SHA512_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA512_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH2_CHANNEL_INFO.channel_idx,
		prgmem);

	SHA512_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_SHA512_HMAC(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH2_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH2_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output, C3_DRIVER_SHA512_OUT_LEN,
		       DMA_FROM_DEVICE);

	SHA512_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	SHA512_HMAC_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	SHA512_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        MD5                              ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_MD5_init(
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);

	MD5_INIT(
		prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_BIDIRECTIONAL);
	C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	MD5_APPEND(
		input_len,
		c3_dma_1->dma_addr,
		prgmem);

	C3_UH_PATCH();

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_end(
	unsigned char *output,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output,
		C3_DRIVER_MD5_OUT_LEN, DMA_FROM_DEVICE);

	MD5_INIT(
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	MD5_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, output,
		C3_DRIVER_MD5_OUT_LEN, DMA_FROM_DEVICE);

	MD5_INIT(prgmem);

	MD5_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	MD5_END(
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_init(
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_FROM_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);

	MD5_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_append(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *ctx,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_BIDIRECTIONAL);
	C3_PREPARE_DMA(c3_dma_1, input, input_len, DMA_TO_DEVICE);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	MD5_HMAC_APPEND(
		input_len,
		c3_dma_1->dma_addr,
		prgmem);

	C3_UH_PATCH();

	SAVE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC_end(
	unsigned char *output,
	unsigned char *ctx,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, ctx, C3_DRIVER_DIGEST_CTX_MAX_SIZE,
		       DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output,
		C3_DRIVER_MD5_OUT_LEN, DMA_FROM_DEVICE);

	MD5_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	RESTORE_CONTEXT(
		c3_dma_0->dma_addr,
		UH_CHANNEL_INFO.channel_idx,
		prgmem);

	MD5_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_MD5_HMAC(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *output,
	unsigned char *key,
	unsigned int key_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(UH_CHANNEL_INFO)
	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(UH_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output,
		C3_DRIVER_MD5_OUT_LEN, DMA_FROM_DEVICE);

	MD5_HMAC_INIT(
		key_len,
		c3_dma_1->dma_addr,
		prgmem);

	MD5_HMAC_APPEND(
		input_len,
		c3_dma_0->dma_addr,
		prgmem);

	MD5_HMAC_END(
		key_len,
		c3_dma_1->dma_addr,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        PKA                              ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_PKA_monty_exp(
	unsigned char *sec_data,
	unsigned int sec_data_len,
	unsigned char *pub_data,
	unsigned char *result,
	unsigned int pub_data_len,      /* same as result len */
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, sec_data, sec_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, pub_data, pub_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, result, pub_data_len, DMA_FROM_DEVICE);

	PKA_MONTY_EXP(
		c3_dma_0->dma_addr,
		sec_data_len,
		c3_dma_1->dma_addr,
		pub_data_len,
		sca_cm,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_monty_par(
	unsigned char *data,
	unsigned int data_len,
	unsigned char *monty_parameter,
	unsigned int monty_parameter_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, data, data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, monty_parameter, monty_parameter_len,
		       DMA_FROM_DEVICE);

	PKA_MONTY_PAR(
		c3_dma_0->dma_addr,
		data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mod_exp(
	unsigned char *sec_data,
	unsigned int sec_data_len,
	unsigned char *pub_data,
	unsigned char *result,
	unsigned int pub_data_len,      /* same as result len */
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, sec_data, sec_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, pub_data, pub_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, result, pub_data_len, DMA_FROM_DEVICE);

	PKA_MOD_EXP(
		c3_dma_0->dma_addr,
		sec_data_len,
		c3_dma_1->dma_addr,
		pub_data_len,
		sca_cm,
		c3_dma_2->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_mul(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ECC_MUL(
		c3_dma_0->dma_addr,
		in_data_len,
		sca_cm,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_monty_mul(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ECC_MONTY_MUL(
		c3_dma_0->dma_addr,
		in_data_len,
		sca_cm,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                        PKA7                             ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_PKA_ecc_dsa_sign(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ECC_DSA_SIGN(
		c3_dma_0->dma_addr,
		in_data_len,
		sca_cm,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_ecc_dsa_verify(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, 4, DMA_FROM_DEVICE);

	PKA_ECC_DSA_VERIFY(
		c3_dma_0->dma_addr,
		in_data_len,
		sca_cm,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mul(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MUL(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_add(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_ADD(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_sub(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_SUB(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_cmp(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, 4, DMA_FROM_DEVICE);

	PKA_ARITH_CMP(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_red(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MOD_RED(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_inv(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MOD_INV(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_add(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MOD_ADD(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_mod_sub(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MOD_SUB(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_arith_monty_mul(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_ARITH_MONTY_MUL(
		c3_dma_0->dma_addr,
		in_data_len,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_rsa_crt(
	unsigned char *in_data,
	unsigned int in_data_len,
	unsigned char *result,
	unsigned int result_len,
	sca_t sca_cm,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, in_data_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_RSA_CRT(
		c3_dma_0->dma_addr,
		in_data_len,
		sca_cm,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mem_reset(
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	PKA_MEM_RESET(prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_PKA_mem_read(
	unsigned char *in_data,
	unsigned char *result,
	unsigned int result_len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(PKA_CHANNEL_INFO) && defined(C3_PKA7)

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;

	C3_START(PKA_CHANNEL_INFO.ids_idx, prgmem, callback, callback_param);

	C3_PREPARE_DMA(c3_dma_0, in_data, 8, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, result, result_len, DMA_FROM_DEVICE);

	PKA_MEM_READ(
		c3_dma_0->dma_addr,
		c3_dma_1->dma_addr,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif

}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||                         RNG                             ||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| */

unsigned int c3_RNG(
	unsigned char *buffer,
	unsigned int len,
	c3_callback_ptr_t callback,
	void *callback_param
)
{

#if defined(RNG_CHANNEL_INFO)

	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(RNG_CHANNEL_INFO.ids_idx, prgmem, callback,
		 callback_param);

	C3_PREPARE_DMA(c3_dma_0, buffer, len, DMA_FROM_DEVICE);

	RNG(
		c3_dma_0->dma_addr,
		len,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

#else

	return C3_ERR;

#endif
}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

C3_INIT_FUNCTION(NULL);
C3_END_FUNCTION(NULL);

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
