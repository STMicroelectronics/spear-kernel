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
 * C3 driver types
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2010-09-16 SK: Added struct "c3_dma_t" for dma_map/unmap
 * ----------------------------------------------------------------- */

#ifndef __C3_TYPES_H__
#define __C3_TYPES_H__

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define C3_OK                                                           ((int)0)
#define C3_ERR                                                          ((int)1)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_PROGRAM_MEMORY_MAX_SIZE                      ((unsigned int)4096)
#define C3_PROGRAM_QUEUE_SIZE                           ((unsigned int)32)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/*-------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------- */

typedef enum {
	sca_none = 0x00,
	sca_spa = 0x01,
	sca_dpa = 0x02,
} sca_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef void (*c3_callback_ptr_t)(void *param, unsigned int status);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef int (*c3_init_function_ptr_t)(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef void (*c3_end_function_ptr_t)(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


typedef struct {
	unsigned int address;
	unsigned int size;

} c3_hw_scatter_list_element_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	unsigned char *original_pointer;
	unsigned int scatter_list_size;
	c3_hw_scatter_list_element_t *hw_scatter_list;
	struct page **__k_pages;

} c3_map_info_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* FOR dma_map/unmap_single() */
struct c3_dma_t {
	struct list_head list;
	void *cpu_addr;
	dma_addr_t dma_addr;
	size_t size;
	enum dma_data_direction direction;
};
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	unsigned int *program_memory;
	unsigned int program_memory_phys;
	unsigned int program_memory_dma_handle;
	unsigned int used_memory;
	unsigned int status;
	c3_callback_ptr_t c3_callback;
	void *c3_callback_param;
	/*volatile*/ int sync;
	struct semaphore sem;
	struct c3_dma_t c3_dma_list;
} program_t;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

typedef struct {
	program_t program_queue[C3_PROGRAM_QUEUE_SIZE];
	unsigned int start_index;
	unsigned int current_index;
	unsigned int end_index;
	spinlock_t spinlock;
	unsigned int c3_device_busy;

} programs_buffer_t;


/* --------------------------------------------------------------------
 * MACROS
 * ----------------------------------------------------------------  */

/* Change endianess */
#define C3_CONV_INT_32(x) \
	(((x & 0xff000000) >> 24) | \
	 ((x & 0x00ff0000) >> 8) | \
	 ((x & 0x0000ff00) << 8) | \
	 ((x & 0x000000ff) << 24))

/* Change endianess */
#define C3_CONV_INT_64(x) \
	( \
		((x & 0xff00000000000000LL) >> 56) | \
		((x & 0x00ff000000000000LL) >> 40) | \
		((x & 0x0000ff0000000000LL) >> 24) | \
		((x & 0x000000ff00000000LL) >> 8) | \
		((x & 0x00000000ff000000LL) << 8) | \
		((x & 0x0000000000ff0000LL) << 24) | \
		((x & 0x000000000000ff00LL) << 40) | \
		((x & 0x00000000000000ffLL) << 56))

#endif /* __C3_TYPES_H__ */
