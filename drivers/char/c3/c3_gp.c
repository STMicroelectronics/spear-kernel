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
 * C3 driver GP channel interface
 *
 * ST - 2008-11-06
 * Alessandro Miglietti
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------- */

#include "c3_common.h"
#include "c3_gp.h"
#include "c3_gp_firmware.h"
#include "c3_driver_interface.h"

/* --------------------------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------------------- */

#ifdef GP_CHANNEL_INFO

static unsigned int __firmware_to_load[] = {
#ifdef USE_C3_GP_FW_IPSEC
	C3_GP_FW_IPSEC;
#else
	C3_GP_FW_TEST;
#endif
};

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static c3_GP_debug_area_t *__dbg_in_area;
static c3_GP_debug_area_t *__dbg_out_area;

static int __dbg_in_area_dma_handle;
static int __dbg_out_area_dma_handle;

#endif /* #ifdef GP_CHANNEL_INFO */

/* --------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

unsigned int c3_GP_init(void)
{
#ifdef GP_CHANNEL_INFO

	unsigned int *__firmware = NULL;

	__firmware = kmalloc(sizeof __firmware_to_load, GFP_KERNEL);

	if (!__firmware) {
		printk(
			C3_KERN_ERR
			"Cannot allocate memory for GP channel firmware\n");
		goto failure;
	}

	memcpy(__firmware, __firmware_to_load, sizeof __firmware_to_load);

	__dbg_in_area = dma_alloc_coherent(
		NULL,
		sizeof(c3_GP_debug_area_t),
		&__dbg_in_area_dma_handle,
		GFP_DMA);

	__dbg_out_area = dma_alloc_coherent(
		NULL,
		sizeof(c3_GP_debug_area_t),
		&__dbg_out_area_dma_handle,
		GFP_DMA);

	if (!__dbg_out_area || !__dbg_in_area) {
		printk(
			C3_KERN_ERR
			"Cannot allocate DMA-safe memory for GP channel debug area\n");
		goto failure;
	}

	__dbg_out_area->read_idx = __dbg_in_area->read_idx = 0;
	__dbg_out_area->write_idx = __dbg_in_area->write_idx = 0;
	__dbg_out_area->buf_size = __dbg_in_area->buf_size =
					   C3_GP_DEBUG_BUF_SIZE;

	if (c3_GP_setup(
		    __dbg_in_area_dma_handle,           /* Phys address */
		    __dbg_out_area_dma_handle,          /* Phys address */
		    sizeof(c3_GP_debug_area_t),
		    NULL,
		    NULL) != C3_OK) {
		printk(C3_KERN_ERR "Cannot setup gp channel\n");
		goto failure;

	} /* if (c3_GP_load_firmware (... */

	if (c3_GP_load_firmware(
		    (unsigned char *)__firmware,
		    sizeof __firmware_to_load,
		    NULL,
		    NULL) != C3_OK) {
		printk(C3_KERN_ERR "Cannot load firmware\n");
		goto failure;

	} /* if (c3_GP_load_firmware (... */

	kfree(__firmware);

#ifdef USE_C3_GP_FW_IPSEC

	if (c3_ipsec_init() != C3_OK) {
		printk(C3_KERN_ERR "Cannot init IPsec channel\n");
		goto failure;

	}       /* if (c3_ipsec_init() != C3_OK) */

#else           /* #ifndef USE_C3_GP_FW_IPSEC */

	{
		unsigned char buf[16];
		int i;

		memset(buf, 0xfe, sizeof(buf));

		if (c3_GP_run(
			    0,
			    buf,
			    sizeof buf,
			    NULL,
			    NULL) != C3_OK) {
			printk(C3_KERN_ERR "Cannot run test firmware\n");
			goto failure;

		} /* if (c3_GP_run_firmware (... */

		for (i = 0; i < sizeof buf; i++) {
			if (buf[i] != 0xff) {
				printk(
					C3_KERN_ERR
					"Test firmware not executed correctly (try compiling the driver with GP channel disabled)\n");
				goto failure;

			}       /* if (buf != 0xff) */

		}               /* for (i=0;i<sizeof buf; i++) */

	}

#endif  /* #ifdef USE_C3_GP_FW_IPSEC */

	return C3_OK;

failure:

	kfree(__firmware);

	c3_GP_cleanup();
	return C3_ERR;

#endif  /* #ifdef GP_CHANNEL_INFO */

	return C3_OK;

} /* unsigned int c3_gp_init() */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3_GP_cleanup(void)
{

#ifdef GP_CHANNEL_INFO

	if (__dbg_in_area) {
		dma_free_coherent(
			NULL,
			sizeof(c3_GP_debug_area_t),
			__dbg_in_area,
			__dbg_in_area_dma_handle);

		__dbg_in_area = NULL;

	} /* if (__dbg_in_area) */

	if (__dbg_out_area) {
		dma_free_coherent(
			NULL,
			sizeof(c3_GP_debug_area_t),
			__dbg_out_area,
			__dbg_out_area_dma_handle);

		__dbg_out_area = NULL;

	} /* if (__dbg_out_area) */

#endif /* #ifdef GP_CHANNEL_INFO */

} /* void c3_GP_cleanup */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_debug_send_message(
	unsigned char *message,
	unsigned int size
)
{

#ifdef GP_CHANNEL_INFO

	if (size > __dbg_in_area->buf_size)
		return C3_ERR;

	if ((__dbg_in_area->write_idx + size) > __dbg_in_area->buf_size) {
		memcpy(
			&__dbg_in_area->buf[__dbg_in_area->write_idx],
			message,
			__dbg_in_area->buf_size - __dbg_in_area->write_idx);

		memcpy(
			&__dbg_in_area->buf[0],
			&message[__dbg_in_area->buf_size -
				 __dbg_in_area->write_idx],
			size -
			(__dbg_in_area->buf_size - __dbg_in_area->write_idx));

	} /* if ((__dbg_in_area->write_idx + size) > __dbg_in_area->buf_size) */
	else{
		memcpy(
			&__dbg_in_area->buf[__dbg_in_area->write_idx],
			message,
			size);
	}

	__dbg_in_area->write_idx =
		(__dbg_in_area->write_idx + size) % __dbg_in_area->buf_size;

#endif  /* #ifdef GP_CHANNEL_INFO */

	return C3_OK;

} /* unsigned int c3_GP_debug_send_message */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_debug_receive_message(
	unsigned char *message,
	unsigned int *size
)
{

#ifdef GP_CHANNEL_INFO

	int wr_idx;

	wr_idx = __dbg_out_area->write_idx;

	if (wr_idx != __dbg_out_area->read_idx) {

		if (wr_idx > __dbg_out_area->read_idx) {
			*size = wr_idx - __dbg_out_area->read_idx;

			memcpy(
				message,
				&__dbg_out_area->buf[__dbg_out_area->read_idx],
				*size);

		} else {
			*size = __dbg_out_area->buf_size -
				__dbg_out_area->read_idx;

			memcpy(
				message,
				&__dbg_out_area->buf[__dbg_out_area->read_idx],
				*size);

			memcpy(
				&message[*size],
				&__dbg_out_area->buf[0],
				wr_idx);

			*size += wr_idx;

		}

		__dbg_out_area->read_idx = wr_idx;


	} else
		*size = 0;

#endif  /* #ifdef GP_CHANNEL_INFO */

	return C3_OK;

} /* unsigned int c3_GP_debug_receive_message */
