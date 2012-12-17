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
 * C3 character device driver instructions mapping structure
 *
 * ST - 2007-10-19
 * Alessandro Miglietti
 *
 * - 2009-09-08 MGR Added PKA v.7 support.
 * - 2011-21-02 SK: Removed SP, GP functions.
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_char_dev_driver_instructions.h"
#include "c3_driver_interface.h"

/* --------------------------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------------------  */

c3_instr_params_t c3_instr_params[] = {
/* --------------------------------- AES ---------------------------*/
{ C3_CDD_AES_randomize, &c3_AES_randomize, C3_CIPHER_RANDOMIZE_FUNCTION, 0,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ } }, { C3_CDD_AES_CBC_decrypt, &c3_AES_CBC_decrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_AES_CBC_encrypt, &c3_AES_CBC_encrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_AES_ECB_encrypt, &c3_AES_ECB_encrypt,
	C3_CIPHER_AES_ECB_FUNCTION, 3,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} }, { C3_CDD_AES_ECB_decrypt, &c3_AES_ECB_decrypt,
	C3_CIPHER_AES_ECB_FUNCTION, 3,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} }, { C3_CDD_AES_CTR_decrypt, &c3_AES_CTR_decrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_AES_CTR_encrypt, &c3_AES_CTR_encrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_AES_XTS_decrypt, &c3_AES_XTS_decrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_AES_XTS_encrypt, &c3_AES_XTS_encrypt,
	C3_CIPHER_AES_CBC_CTR_FUNCTION, 4,
#ifdef USE_AES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_AES_IVEC_SIZE }, /* Ivec */
	} },
/* --------------------------------- DES ---------------------------*/
{ C3_CDD_DES_CBC_decrypt, &c3_DES_CBC_decrypt, C3_CIPHER_DES_CBC_FUNCTION, 4,
#ifdef USE_DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_DES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_DES_CBC_encrypt, &c3_DES_CBC_encrypt,
	C3_CIPHER_DES_CBC_FUNCTION, 4,
#ifdef USE_DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_DES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_DES_ECB_encrypt, &c3_DES_ECB_encrypt,
	C3_CIPHER_DES_ECB_FUNCTION, 3,
#ifdef USE_DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} }, { C3_CDD_DES_ECB_decrypt, &c3_DES_ECB_decrypt,
	C3_CIPHER_DES_ECB_FUNCTION, 3,
#ifdef USE_DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} },
/* -------------------------------- 3DES ---------------------------*/
{ C3_CDD_3DES_CBC_decrypt, &c3_3DES_CBC_decrypt, C3_CIPHER_DES_CBC_FUNCTION, 4,
#ifdef USE_3DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_3DES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_3DES_CBC_encrypt, &c3_3DES_CBC_encrypt,
	C3_CIPHER_DES_CBC_FUNCTION, 4,
#ifdef USE_3DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	{ C3_BUFFER_INPUT, 5, -1, C3_DRIVER_3DES_IVEC_SIZE }, /* Ivec */
	} }, { C3_CDD_3DES_ECB_encrypt, &c3_3DES_ECB_encrypt,
	C3_CIPHER_DES_ECB_FUNCTION, 3,
#ifdef USE_3DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} }, { C3_CDD_3DES_ECB_decrypt, &c3_3DES_ECB_decrypt,
	C3_CIPHER_DES_ECB_FUNCTION, 3,
#ifdef USE_3DES_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 2, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 } /* Key */
	} },
/* -------------------------------- SHA1 ---------------------------*/
{ C3_CDD_HASH_SHA1_init, &c3_SHA1_init, C3_HASH_INIT_FUNCTION, 1,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA1_append, &c3_SHA1_append,
	C3_HASH_APPEND_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA1_end, &c3_SHA1_end, C3_HASH_END_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA1_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA1, &c3_SHA1, C3_HASH_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA1_OUT_LEN }, /* Output */
	} }, { C3_CDD_HMAC_SHA1_init, &c3_SHA1_HMAC_init,
	C3_HMAC_INIT_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 1, 2, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA1_append, &c3_SHA1_HMAC_append,
	C3_HMAC_APPEND_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HMAC_SHA1_end, &c3_SHA1_HMAC_end, C3_HMAC_END_FUNCTION,
	3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA1_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 2, 3, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA1, &c3_SHA1_HMAC, C3_HMAC_FUNCTION, 3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA1_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	} },
/* -------------------------------- SHA256 ---------------------------*/
{ C3_CDD_HASH_SHA256_init, &c3_SHA256_init, C3_HASH_INIT_FUNCTION, 1,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA256_append, &c3_SHA256_append,
	C3_HASH_APPEND_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA256_end, &c3_SHA256_end, C3_HASH_END_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA256_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA256, &c3_SHA256, C3_HASH_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA256_OUT_LEN }, /* Output */
	} }, { C3_CDD_HMAC_SHA256_init, &c3_SHA256_HMAC_init,
	C3_HMAC_INIT_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 1, 2, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA256_append, &c3_SHA256_HMAC_append,
	C3_HMAC_APPEND_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HMAC_SHA256_end, &c3_SHA256_HMAC_end,
	C3_HMAC_END_FUNCTION, 3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA256_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 2, 3, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA256, &c3_SHA256_HMAC, C3_HMAC_FUNCTION, 3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA256_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	} },
/* -------------------------------- SHA384 ---------------------------*/
{ C3_CDD_HASH_SHA384_init, &c3_SHA384_init, C3_HASH_INIT_FUNCTION, 1,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA384_append, &c3_SHA384_append,
	C3_HASH_APPEND_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA384_end, &c3_SHA384_end, C3_HASH_END_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA384_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA384, &c3_SHA384, C3_HASH_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA384_OUT_LEN }, /* Output */
	} }, { C3_CDD_HMAC_SHA384_init, &c3_SHA384_HMAC_init,
	C3_HMAC_INIT_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 1, 2, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA384_append, &c3_SHA384_HMAC_append,
	C3_HMAC_APPEND_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HMAC_SHA384_end, &c3_SHA384_HMAC_end,
	C3_HMAC_END_FUNCTION, 3,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA384_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 2, 3, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA384, &c3_SHA384_HMAC, C3_HMAC_FUNCTION, 3,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA384_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	} },
/* -------------------------------- SHA512 ---------------------------*/
{ C3_CDD_HASH_SHA512_init, &c3_SHA512_init, C3_HASH_INIT_FUNCTION, 1,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA512_append, &c3_SHA512_append,
	C3_HASH_APPEND_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA512_end, &c3_SHA512_end, C3_HASH_END_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA512_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_SHA512, &c3_SHA512, C3_HASH_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA512_OUT_LEN }, /* Output */
	} }, { C3_CDD_HMAC_SHA512_init, &c3_SHA512_HMAC_init,
	C3_HMAC_INIT_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 1, 2, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA512_append, &c3_SHA512_HMAC_append,
	C3_HMAC_APPEND_FUNCTION, 2,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HMAC_SHA512_end, &c3_SHA512_HMAC_end,
	C3_HMAC_END_FUNCTION, 3,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_SHA512_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 2, 3, 0 }, /* Key */
	} }, { C3_CDD_HMAC_SHA512, &c3_SHA512_HMAC, C3_HMAC_FUNCTION, 3,
#ifdef USE_UH2_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_SHA512_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	} },
/* -------------------------------- MD5 ---------------------------*/
{ C3_CDD_HASH_MD5_init, &c3_MD5_init, C3_HASH_INIT_FUNCTION, 1,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_MD5_append, &c3_MD5_append, C3_HASH_APPEND_FUNCTION,
	2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_MD5_end, &c3_MD5_end, C3_HASH_END_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_MD5_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HASH_MD5, &c3_MD5, C3_HASH_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_MD5_OUT_LEN }, /* Output */
	} }, { C3_CDD_HMAC_MD5_init, &c3_MD5_HMAC_init, C3_HMAC_INIT_FUNCTION,
	2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 1, 2, 0 }, /* Key */
	} }, { C3_CDD_HMAC_MD5_append, &c3_MD5_HMAC_append,
	C3_HMAC_APPEND_FUNCTION, 2,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	} }, { C3_CDD_HMAC_MD5_end, &c3_MD5_HMAC_end, C3_HMAC_END_FUNCTION, 3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, -1, C3_DRIVER_MD5_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 1, -1, C3_DRIVER_DIGEST_CTX_MAX_SIZE }, /* CTX */
	{ C3_BUFFER_INPUT, 2, 3, 0 }, /* Key */
	} }, { C3_CDD_HMAC_MD5, &c3_MD5_HMAC, C3_HMAC_FUNCTION, 3,
#ifdef USE_UH_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Input */
	{ C3_BUFFER_OUTPUT, 2, -1, C3_DRIVER_MD5_OUT_LEN }, /* Output */
	{ C3_BUFFER_INPUT, 3, 4, 0 }, /* Key */
	} },
/* -------------------------------- PKA  ---------------------------*/
{ C3_CDD_PKA_monty_exp, &c3_PKA_monty_exp, C3_PKA_EXP_FUNCTION, 3,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Sec data */
	{ C3_BUFFER_INPUT, 2, 4, 0 }, /* Pub data */
	{ C3_BUFFER_OUTPUT, 3, 4, 0 }, /* Result */
	} }, { C3_CDD_PKA_monty_par, &c3_PKA_monty_par,
	C3_PKA_MONTY_PAR_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Monty param */
	} }, { C3_CDD_PKA_mod_exp, &c3_PKA_mod_exp, C3_PKA_EXP_FUNCTION, 3,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* Sec data */
	{ C3_BUFFER_INPUT, 2, 4, 0 }, /* Pub data */
	{ C3_BUFFER_OUTPUT, 3, 4, 0 }, /* Result */
	} }, { C3_CDD_PKA_ecc_mul, &c3_PKA_ecc_mul, C3_PKA_ECC_MUL_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result */
	} }, { C3_CDD_PKA_ecc_monty_mul, &c3_PKA_ecc_monty_mul,
	C3_PKA_ECC_MUL_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} },
/* -------------------------------- PKA7  -------------------------*/
{ C3_CDD_PKA_ecc_dsa_sign, &c3_PKA_ecc_dsa_sign, C3_PKA_ECC_DSA_SIGN_FUNCTION,
	2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_ecc_dsa_verify, &c3_PKA_ecc_dsa_verify,
	C3_PKA_ECC_DSA_VERIFY_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, -1, 4 }, /* Result data */
	} }, { C3_CDD_PKA_arith_mul, &c3_PKA_arith_mul,
	C3_PKA_ARITH_MUL_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_add, &c3_PKA_arith_add,
	C3_PKA_ARITH_ADD_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_sub, &c3_PKA_arith_sub,
	C3_PKA_ARITH_SUB_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_cmp, &c3_PKA_arith_cmp,
	C3_PKA_ARITH_CMP_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, -1, 4 }, /* Result data */
	} }, { C3_CDD_PKA_arith_mod_red, &c3_PKA_arith_mod_red,
	C3_PKA_ARITH_MOD_RED_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_mod_inv, &c3_PKA_arith_mod_inv,
	C3_PKA_ARITH_MOD_INV_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_mod_add, &c3_PKA_arith_mod_add,
	C3_PKA_ARITH_MOD_ADD_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_mod_sub, &c3_PKA_arith_mod_sub,
	C3_PKA_ARITH_MOD_SUB_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_arith_monty_mul, &c3_PKA_arith_monty_mul,
	C3_PKA_ARITH_MONTY_MUL_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_rsa_crt, &c3_PKA_rsa_crt, C3_PKA_RSA_CRT_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, 1, 0 }, /* In data */
	{ C3_BUFFER_OUTPUT, 2, 3, 0 }, /* Result data */
	} }, { C3_CDD_PKA_mem_reset, &c3_PKA_mem_reset,
	C3_PKA_MEM_RESET_FUNCTION, 0,
#ifdef USE_PKA_SCATTER_GATHER
	0,
#else
	0,
#endif
	{ } }, { C3_CDD_PKA_mem_read, &c3_PKA_mem_read,
	C3_PKA_MEM_READ_FUNCTION, 2,
#ifdef USE_PKA_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_INPUT, 0, -1, 8 }, /* In data */
	{ C3_BUFFER_OUTPUT, 1, 2, 0 }, /* Result data */
	} },
/* ---------------------------- RNG -----------------------------------*/
{ C3_CDD_RNG, &c3_RNG, C3_RNG_FUNCTION, 1,
#ifdef USE_RNG_SCATTER_GATHER
	1,
#else
	0,
#endif
	{ { C3_BUFFER_OUTPUT, 0, 1, 0 }, /* Buffer */
	} },

};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
