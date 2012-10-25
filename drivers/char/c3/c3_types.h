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
 * --------------------------------------------------------------------
 */

#ifndef __C3_TYPES_H__
#define __C3_TYPES_H__

#include <linux/completion.h>
#include <linux/dma-attrs.h>

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define C3_OK						((int)0)
#define C3_ERR						((int)1)
#define C3_SKIP						((int)2)

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
	struct completion completion;
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
	unsigned long flags;
} programs_buffer_t;

#endif /* __C3_TYPES_H__ */
