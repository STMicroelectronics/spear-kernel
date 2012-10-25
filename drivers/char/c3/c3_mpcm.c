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
 * C3 driver mpcm
 *
 * ST - 2007-08-01
 * Alessandro Miglietti
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"
#include "c3_driver_core.h"
#include "c3_irq.h"
#include "c3_mpcm.h"
#include "c3_mpcm_firmware.h"

/*-------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------- */

#define MPCM_RAM_START_ADDRESS                              ((int)32)

/*-------------------------------------------------------------------
 * MACROS
 * ----------------------------------------------------------------- */

#define LOAD_FIRMWARE(code, index)					\
	do {								\
		unsigned char *fw ;					\
		unsigned char firmware[] = code;			\
		unsigned int err;					\
		fw = kmalloc(sizeof firmware, GFP_DMA);			\
		if (!fw) {						\
			C3_ERROR("Cannot alloc memory for MPCM firmware"); \
			return -1;					\
		}							\
		memcpy(fw, firmware, sizeof firmware);			\
		err = c3_mpcm_download_firmware(			\
			index,						\
			ram_index,					\
			fw,						\
			sizeof firmware);				\
		if (err != C3_OK) {					\
			return err;					\
		}							\
		ram_index += (sizeof firmware) / 8;			\
	} while (0)

/*-------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

#ifdef MPCM_CHANNEL_INFO

static unsigned int c3_mpcm_download_firmware
(
	unsigned int program_index,
	unsigned int mpcm_ram_address,
	unsigned char *firmware,
	unsigned int firmware_size /* in bytes */
)
{
	struct c3_dma_t *c3_dma_0 = NULL;

	C3_START(
		MPCM_CHANNEL_INFO.ids_idx,
		prgmem,
		NULL,
		NULL);

	C3_PREPARE_DMA(c3_dma_0, firmware, firmware_size, DMA_TO_DEVICE);

	MPCM_DOWNLOAD(
		program_index,
		mpcm_ram_address,
		c3_dma_0->dma_addr,
		firmware_size,
		prgmem);

	STOP(prgmem);

	C3_END(prgmem);

	return C3_OK;

} /* unsigned int c3_mpcm_download_firmware */

#endif

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_mpcm_init(void)
{

#ifdef MPCM_CHANNEL_INFO

	unsigned int ram_index = MPCM_RAM_START_ADDRESS;

#if defined(USE_MPCM_AES_ECB) || \
	defined(USE_MPCM_AES_CBC) || \
	defined(USE_MPCM_AES_CTR)

	LOAD_FIRMWARE(
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_CODE,
		MPCM_AES_CBC_CTR_ECB_KEY_LOAD_PROGRAM_INDEX);

#endif


#if defined(USE_MPCM_AES_ECB) || \
	defined(USE_MPCM_AES_CBC)

	LOAD_FIRMWARE(
		MPCM_AES_CBC_ECB_KEY_SCHEDULE_CODE,
		MPCM_AES_CBC_ECB_KEY_SCHEDULE_PROGRAM_INDEX);

#endif

#if defined(USE_MPCM_AES_CBC) || defined(USE_MPCM_AES_CTR)

	LOAD_FIRMWARE(
		MPCM_AES_CBC_CTR_INIT_IVEC_CODE,
		MPCM_AES_CBC_CTR_INIT_IVEC_PROGRAM_INDEX);

#endif

#ifdef USE_MPCM_AES_ECB

	LOAD_FIRMWARE(
		MPCM_AES_ECB_ENC_APPEND_CODE,
		MPCM_AES_ECB_ENC_APPEND_PROGRAM_INDEX);

	LOAD_FIRMWARE(
		MPCM_AES_ECB_DEC_APPEND_CODE,
		MPCM_AES_ECB_DEC_APPEND_PROGRAM_INDEX);
#endif

#ifdef USE_MPCM_AES_CBC

	LOAD_FIRMWARE(
		MPCM_AES_CBC_ENC_APPEND_CODE,
		MPCM_AES_CBC_ENC_APPEND_PROGRAM_INDEX);

	LOAD_FIRMWARE(
		MPCM_AES_CBC_DEC_APPEND_CODE,
		MPCM_AES_CBC_DEC_APPEND_PROGRAM_INDEX);
#endif


#ifdef USE_MPCM_AES_CTR

	LOAD_FIRMWARE(
		MPCM_AES_CTR_APPEND_CODE,
		MPCM_AES_CTR_APPEND_PROGRAM_INDEX);

#endif

#ifdef USE_MPCM_AES_XCBC_MAC_96

	LOAD_FIRMWARE(
		MPCM_AES_XCBC_MAC_96_KEY_LOAD_CODE,
		MPCM_AES_XCBC_MAC_96_KEY_LOAD_PROGRAM_INDEX);

	LOAD_FIRMWARE(
		MPCM_AES_XCBC_MAC_96_APPEND_CODE,
		MPCM_AES_XCBC_MAC_96_APPEND_PROGRAM_INDEX);

	LOAD_FIRMWARE(
		MPCM_AES_XCBC_MAC_96_END_CODE,
		MPCM_AES_XCBC_MAC_96_END_PROGRAM_INDEX);

#endif

#endif

	return C3_OK;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
