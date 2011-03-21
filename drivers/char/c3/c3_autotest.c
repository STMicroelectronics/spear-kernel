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
 * C3 driver TEST functions
 *
 * ST - 2007-08-08
 * Alessandro Miglietti
 *
 * - 2010-09-16 SK: Keys/Ivec are now allocated using kmalloc.
 * - 2011-02-21 SK: Removed typdef
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include <linux/delay.h>

#include "c3_settings.h"
#include "c3_common.h"
#include "c3_autotest.h"

#include "c3_driver_core.h"
#include "c3_driver_interface.h"
#include "c3_irq.h"
#include "c3_mpcm.h"

#ifdef	CONFIG_AUTO_TEST

unsigned char *des_ivec, *des_key, *des3_key;
unsigned char *sha1_digest, *sha1_hmac_key;
unsigned char *sha512_digest, *sha512_hmac_key;
unsigned char *aes_ivec, *aes128_key, *aes256_key;

/* ------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define AES_XCBC_MAC_96_OUTPUT_LEN	((unsigned int)12)
#define PERF_TEST_ITER_NUM		((unsigned int)10000)

/* -------------------------------------------------------------------
 * MACROS
 * ----------------------------------------------------------------- */

#define LAUNCH_TEST(function, message)					\
	do {								\
		if (function() == C3_OK) {				\
			printk(C3_KERN_INFO message ": OK\n");		\
		} else {						\
			printk(C3_KERN_ERR message ": Failed\n");	\
			return C3_ERR;					\
		}							\
	} while (0)
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define LAUNCH_PERF_TEST(function, size, message)			\
	do {								\
		int res;						\
		res = function(size);					\
		if (res == -1) {					\
			printk(C3_KERN_ERR message ": Failed\n");	\
			return C3_ERR;					\
		}							\
		if (res / 1000 > 0) {					\
			printk(						\
				C3_KERN_INFO message			\
				" [buffer size: %d] throughput "	\
				": %d KBps\n", size,			\
				((PERF_TEST_ITER_NUM * size) / (res / 1000))\
			);						\
		} else {						\
			if (res != 0) {					\
				printk(					\
					C3_KERN_INFO message		\
					" [buffer size: %d] throughput "\
					": %d KBps\n", size, 1000 *	\
					((PERF_TEST_ITER_NUM * size) / (res))\
				);					\
			} else {					\
				printk(					\
					C3_KERN_INFO message		\
					" [buffer size: %d] throughput "\
					": - KBps\n", size		\
				);					\
			}						\
		}							\
	} while (0)

/* -------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

/* ===================================================================
 *
 * AES_XCBC_MAC_96
 *
 * ================================================================ */

#ifdef USE_MPCM_AES_XCBC_MAC_96

static int __c3_MPCM_AES_XCBC_MAC_96
(
	unsigned char *input,
	unsigned int input_len,
	unsigned char *key,
	unsigned int key_len,
	unsigned char *output,
	c3_callback_ptr_t callback,
	void *callback_param
)
{
	unsigned int len_append;
	unsigned int len_end;

	struct c3_dma_t *c3_dma_0 = NULL;
	struct c3_dma_t *c3_dma_1 = NULL;
	struct c3_dma_t *c3_dma_2 = NULL;

	C3_START(
		MPCM_CHANNEL_INFO.ids_idx,
		prgmem,
		callback,
		callback_param);

	C3_PREPARE_DMA(c3_dma_0, input, input_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_1, key, key_len, DMA_TO_DEVICE);
	C3_PREPARE_DMA(c3_dma_2, output,
		AES_XCBC_MAC_96_OUTPUT_LEN,
		DMA_FROM_DEVICE);

	if (input_len > 16) {
		len_append = (input_len - 1) & ~0xF;
		len_end = (input_len - 1) % 16 + 1;

	} else {
		len_end = input_len;
		len_append = 0;
	}

	MPCM_EXECUTE_S(
		MPCM_AES_XCBC_MAC_96_KEY_LOAD_PROGRAM_INDEX,
		c3_dma_1->dma_addr,
		key_len,
		prgmem);

	if (len_append) {
		MPCM_EXECUTE_S(
			MPCM_AES_XCBC_MAC_96_APPEND_PROGRAM_INDEX,
			c3_dma_0->dma_addr,
			len_append,
			prgmem);

	}

	/*MPCM_EXECUTE_SD(
	 MPCM_AES_XCBC_MAC_96_END_PROGRAM_INDEX,
	 C3BUS_MAP_ADDR(input + len_append),
	 C3BUS_MAP_ADDR(output),
	 len_end,
	 prgmem);*/

	/* TODO:CHECK */
	MPCM_EXECUTE_SD(
		MPCM_AES_XCBC_MAC_96_END_PROGRAM_INDEX,
		(c3_dma_0->dma_addr + len_append),
		c3_dma_2->dma_addr,
		len_end,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* Test structures & constants */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define __TEST_AES_XCBC_MAC_96_MAX_KEY_LEN		((unsigned int)128)
#define __TEST_AES_XCBC_MAC_96_MAX_MESSAGE_LEN		((unsigned int)8192)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

struct test_AES_XCBC_MAC_96_s {
		unsigned char key[__TEST_AES_XCBC_MAC_96_MAX_KEY_LEN];
		unsigned int key_len;
		unsigned char message[__TEST_AES_XCBC_MAC_96_MAX_MESSAGE_LEN];
		unsigned int message_len;
		unsigned char ref_output[AES_XCBC_MAC_96_OUTPUT_LEN];
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_1 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{

	},
	0,
	{
		0x75, 0xf0, 0x25, 0x1d, 0x52, 0x8a,
		0xc0, 0x1c, 0x45, 0x73, 0xdf, 0xd5
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_2 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0x00, 0x01, 0x02
	},
	3,
	{
		0x5b, 0x37, 0x65, 0x80, 0xae, 0x2f,
		0x19, 0xaf, 0xe7, 0x21, 0x9c, 0xee
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_3 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0xd2, 0xa2, 0x46, 0xfa, 0x34, 0x9b,
		0x68, 0xa7, 0x99, 0x98, 0xa4, 0x39
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_4 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13
	},
	20,
	{
		0x47, 0xf5, 0x1b, 0x45, 0x64, 0x96,
		0x62, 0x15, 0xb8, 0x98, 0x5c, 0x63
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_5 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
	},
	32,
	{
		0xf5, 0x4f, 0x0e, 0xc8, 0xd2, 0xb9,
		0xf3, 0xd3, 0x68, 0x07, 0x73, 0x4b
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static struct test_AES_XCBC_MAC_96_s test_AES_XCBC_MAC_96_vector_6 = {
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	},
	16,
	{
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
		0x20, 0x21
	},
	34,
	{
		0xbe, 0xcb, 0xb3, 0xbc, 0xcd, 0xb5,
		0x18, 0xa3, 0x06, 0x77, 0xd5, 0x48
	}
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* Test functions */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int __test_MPCM_AES_XCBC_MAC_96
(
	struct test_AES_XCBC_MAC_96_s *test_AES_XCBC_MAC_96
)
{
	unsigned int err = C3_ERR;
	int message_len = test_AES_XCBC_MAC_96->message_len;
	int key_len = test_AES_XCBC_MAC_96->key_len;
	unsigned char *message = NULL;
	unsigned char *key = NULL;
	unsigned char *output = NULL;
	int cmp = 0;

	message = kmalloc(message_len, GFP_DMA);
	key = kmalloc(key_len, GFP_DMA);
	output = kmalloc(AES_XCBC_MAC_96_OUTPUT_LEN, GFP_DMA);

	if (!message || !key || !output)
		goto exit_fun;

	memcpy(message, test_AES_XCBC_MAC_96->message, message_len);
	memcpy(key, test_AES_XCBC_MAC_96->key, key_len);

	err = __c3_MPCM_AES_XCBC_MAC_96(
		message,
		message_len,
		key,
		key_len,
		output,
		NULL,
		NULL
	);

	if (err != C3_OK)
		goto exit_fun;

	cmp = memcmp(
		output,
		test_AES_XCBC_MAC_96->ref_output,
		AES_XCBC_MAC_96_OUTPUT_LEN
	);

	if (cmp)
		err = C3_ERR;

exit_fun:
	kfree(message);
	kfree(key);
	kfree(output);

	return err;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int c3_autotest_MPCM_AES_XCBC_MAC_96(void)
{
	if (
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_1) ==
			C3_OK) &&
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_2) ==
			C3_OK) &&
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_3) ==
			C3_OK) &&
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_4) ==
			C3_OK) &&
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_5) ==
			C3_OK) &&
		(__test_MPCM_AES_XCBC_MAC_96(&test_AES_XCBC_MAC_96_vector_6) ==
			C3_OK)) {

		return C3_OK;
	}

	return C3_ERR;
}

#endif

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* ===================================================================
 *
 * LOCAL MEMORY
 *
 * ================================================================ */

#if defined(C3_LOCAL_MEMORY_ON) && defined(MOVE_CHANNEL_INFO)

static unsigned int c3_autotest_local_memory(void)
{
	unsigned char *buf1;
	unsigned char *buf2;
	unsigned int size = 32;
	unsigned int err = C3_OK;

	buf1 = kmalloc(size, GFP_DMA);
	buf2 = kmalloc(size, GFP_DMA);

	if (!buf1 || !buf2) {
		printk(C3_KERN_ERR "Error:Cannot allocate memory\n");
		return -1;
	}

	memset(buf1, 0xc, size);
	memset(buf2, 0xf, size);

	if ((c3_write_local_memory(buf1, 0, size, NULL, 0)) != C3_OK) {
		err = C3_ERR;
		goto cleanup;
	}
	if ((c3_read_local_memory(0, buf2, size, NULL, 0)) != C3_OK) {
		err = C3_ERR;
		goto cleanup;
	}

	if (memcmp(buf1, buf2, size)) {
		err = C3_ERR;
		C3_ERROR("Local Memory Test Error");
		print_hex_dump(KERN_ERR, "Buf1:",
			DUMP_PREFIX_ADDRESS, 16, 1, buf1, size, 0);
		print_hex_dump(KERN_ERR, "Buf2:",
			DUMP_PREFIX_ADDRESS, 16, 1, buf2, size, 0);
	}

cleanup:
	kfree(buf1);
	kfree(buf2);
	return err;
}

#endif

/* ===================================================================
 *
 * DES/3DES
 *
 * ================================================================ */

#if defined(DES_CHANNEL_INFO)

static int c3_autotest_DES_CBC(int size)
{
	unsigned char *buf = NULL;

	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;
	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if (c3_DES_CBC_encrypt(
			buf,
			buf,
			size,
			des_key,
			8,
			des_ivec,
			NULL,
			NULL) != C3_OK) {
			kfree(buf);
			return -1;
		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int c3_autotest_3DES_CBC(int size)
{
	unsigned char *buf = NULL;

	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);
	if (!buf)
		return -1;

	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if (c3_3DES_CBC_encrypt(
			buf,
			buf,
			size,
			des3_key,
			24,
			des_ivec,
			NULL,
			NULL) != C3_OK) {
			kfree(buf);
			return -1;
		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

#endif

/* ===================================================================
 *
 * AES
 *
 * ================================================================ */

#if defined(AES_CHANNEL_INFO) ||                   \
	(defined(MPCM_CHANNEL_INFO) && defined(USE_MPCM_AES_CBC))

static int c3_autotest_AES128_CBC(int size)
{
	unsigned char *buf = NULL;

	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);
	if (!buf)
		return -1;
	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if (c3_AES_CBC_encrypt(
			buf,
			buf,
			size,
			aes128_key,
			16,
			aes_ivec, 0, 0,
			NULL,
			NULL) != C3_OK
		) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int c3_autotest_AES256_CBC
(
	int size
)
{
	unsigned char *buf = NULL;

	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;

	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if (c3_AES_CBC_encrypt(
			buf,
			buf,
			size,
			aes256_key,
			32,
			aes_ivec, 0, 0,
			NULL,
			NULL) != C3_OK) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

#endif

/* ===================================================================
 *
 * SHA1
 *
 * ================================================================ */

#if defined(UH_CHANNEL_INFO)

static int c3_autotest_SHA1_HMAC
(
	int size
)
{
	unsigned char *buf = NULL;
	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;

	memset(buf, 0xf, size);


	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if ((c3_SHA1_HMAC(
			buf,
			size,
			sha1_digest,
			sha1_hmac_key,
			C3_DRIVER_SHA1_OUT_LEN,
			NULL,
			NULL) != C3_OK)) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int c3_autotest_SHA1
(
	int size
)
{
	unsigned char *buf = NULL;
	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;

	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if ((c3_SHA1(
			buf,
			size,
			sha1_digest,
			NULL,
			NULL) != C3_OK)) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);

	return timer_end - timer_start;

}

#endif

/* ===================================================================
 *
 * SHA512
 *
 * ================================================================ */

#if defined(UH2_CHANNEL_INFO)

static int c3_autotest_SHA512_HMAC
(
	int size
)
{
	unsigned char *buf = NULL;
	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;

	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if ((c3_SHA512_HMAC(
			buf,
			size,
			sha512_digest,
			sha512_hmac_key,
			64,
			NULL,
			NULL) != C3_OK)) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);
	return timer_end - timer_start;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int c3_autotest_SHA512
(
	int size
)
{
	unsigned char *buf = NULL;
	int i;

	unsigned int timer_start;
	unsigned int timer_end;

	buf = kmalloc(size, GFP_DMA);

	if (!buf)
		return -1;
	memset(buf, 0xf, size);

	C3_GET_TICK_COUNT(timer_start);

	for (i = 0; i < PERF_TEST_ITER_NUM; i++) {
		if ((c3_SHA512(
			buf,
			size,
			sha512_digest,
			NULL,
			NULL) != C3_OK)) {
			kfree(buf);
			return -1;

		}

	}

	C3_GET_TICK_COUNT(timer_end);

	kfree(buf);

	return timer_end - timer_start;

}

#endif

#endif /* ifdef CONFIG_AUTO_TEST*/

/* ===================================================================
 *
 * AUTOTEST Function: it launches all the tests
 *
 * ================================================================ */

unsigned int c3_autotest(void)
{
#ifdef CONFIG_AUTO_TEST

	int s;



#if defined(C3_LOCAL_MEMORY_ON) && defined(MOVE_CHANNEL_INFO)
	LAUNCH_TEST(c3_autotest_local_memory, "Local memory test");
#endif

#if defined(UH_CHANNEL_INFO)

	sha1_digest = kmalloc(C3_DRIVER_SHA1_OUT_LEN, GFP_DMA);
	sha1_hmac_key = kmalloc(C3_DRIVER_SHA1_OUT_LEN, GFP_DMA);

	if (!sha1_digest || !sha1_hmac_key)
		return -1;

	memset(sha1_digest, 0x1, C3_DRIVER_SHA1_OUT_LEN);
	memset(sha1_hmac_key, 0x2, C3_DRIVER_SHA1_OUT_LEN);


	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_SHA1, s, "SHA1 test");

	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_SHA1_HMAC, s, "SHA1_HMAC test");

	kfree(sha1_digest);
	kfree(sha1_hmac_key);

#endif

#if defined(UH2_CHANNEL_INFO)

	sha512_digest = kmalloc(64, GFP_DMA);
	sha512_hmac_key = kmalloc(64, GFP_DMA);

	if (!sha512_digest || !sha512_hmac_key)
		return -1;

	memset(sha512_digest, 0x1, 64);
	memset(sha512_hmac_key, 0x2, 64);


	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_SHA512, s, "SHA512 test");

	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_SHA512_HMAC, s, "SHA512HMAC test");

	kfree(sha512_digest);
	kfree(sha512_hmac_key);

#endif


#if defined(DES_CHANNEL_INFO)

	des_ivec = kmalloc(8, GFP_DMA);
	des_key = kmalloc(8, GFP_DMA);
	des3_key = kmalloc(24, GFP_DMA);

	if (!des_ivec || !des_key || !des3_key)
		return -1;

	memset(des_ivec, 0x1, 8);
	memset(des_key, 0x2, 8);
	memset(des3_key, 0x3, 24);

	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_DES_CBC, s, "DES_CBC test");

	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_3DES_CBC, s, "3DES_CBC test");

	kfree(des_ivec);
	kfree(des_key);
	kfree(des3_key);

#endif

#if defined(MPCM_CHANNEL_INFO) && defined(USE_MPCM_AES_XCBC_MAC_96)
	LAUNCH_TEST(c3_autotest_MPCM_AES_XCBC_MAC_96,
		"MPCM AES-XCBC-MAC-96 test");
#endif

#if defined(AES_CHANNEL_INFO) || (defined(MPCM_CHANNEL_INFO) &&\
	defined(USE_MPCM_AES_CBC))

	aes_ivec = kmalloc(16, GFP_DMA);
	aes128_key = kmalloc(16, GFP_DMA);
	aes256_key = kmalloc(32, GFP_DMA);

	if (!aes_ivec || !aes128_key || !aes256_key)
		return -1;

	memset(aes_ivec, 0x1, 16);
	memset(aes128_key, 0x2, 16);
	memset(aes256_key, 0x3, 32);


	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_AES128_CBC, s, "AES128_CBC test");

	for (s = 64; s <= 8192; s = s * 2)
		LAUNCH_PERF_TEST(c3_autotest_AES256_CBC, s, "AES256_CBC test");

	kfree(aes_ivec);
	kfree(aes128_key);
	kfree(aes256_key);

#endif

#endif

	return C3_OK;

}
