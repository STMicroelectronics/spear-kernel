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
 * C3 driver common definitions
 *
 * THIS HEADER MUST BE INCLUDED AT THE BEGINNING OF ALL THE SOURCE FILES
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2009-12-19 MGR Moved consistent_sync(...) to dma_map_single(NULL, ...)
 * - 2010-09-16 SK: Re-wrote DMA handling part(contiguous buffers and S/G lists)
 * ----------------------------------------------------------------- */

#ifndef __C3_COMMON_H__
#define __C3_COMMON_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

/* For do_gettimeofday */
#include <linux/time.h>
#include <linux/sched.h>

/* For init and exit routine */
#include <linux/module.h>
#include <linux/moduleparam.h>

/* For kmalloc */
#include <linux/kernel.h>

/* For dma */
#include <linux/dma-mapping.h>
#include <linux/dma-attrs.h>

/* For in_interrupt */
#include <linux/hardirq.h>

/* For clock */
#include <linux/clk.h>

/* Linked list */
#include <linux/list.h>

#include "c3_settings.h"
#include "c3_operations.h"
#include "c3_busapi.h"
#include "c3_types.h"
#include "c3_registers.h"

#ifdef DEBUG
#define CRYPTOAPI_DEBUG
#endif

#if defined(CONFIG_C3_CRYPTOAPI_INTEGRATION) || defined(ENABLE_CRYPTOAPI)
#define CRYPTOAPI
#endif

/* --------------------------------------------------------------------
 * MACROS
 * ----------------------------------------------------------------  */

#define C3_INIT_FUNCTION(f) \
	c3_init_function_ptr_t c3_init_function = f;

#define C3_END_FUNCTION(f) \
	c3_end_function_ptr_t c3_end_function = f;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_GET_TICK_COUNT(a) \
	do { \
		struct timeval time; \
		do_gettimeofday(&time);	\
		a = time.tv_usec + time.tv_sec * 1000000; \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_START(ids, prgmem, callback, callback_param)			\
	unsigned int prg_index;						\
	program_t *program;						\
	unsigned int err, ret;						\
	unsigned int ids_index = ids;					\
	err = c3_get_program(&program, &prg_index, ids_index);		\
	if (err == C3_OK) {						\
		unsigned int *prgmem = program->program_memory;		\
		program->c3_callback = callback;			\
		INIT_LIST_HEAD(&(program->c3_dma_list.list));		\
		if (callback) {						\
			program->c3_callback_param = callback_param;	\
		}							\
		else {							\
			init_completion(&(program->completion));	\
		}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 * NESTED MACROS (...)
 * :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 */

#define C3_END(prgmem)							\
		program->used_memory = prgmem - program->program_memory;\
		err = c3_enqueue_program(prg_index, ids_index);		\
		if (err != C3_OK) {					\
			return err;					\
		}							\
	}								\
	else {								\
		return err;						\
	}								\
	if (!program->c3_callback) {					\
		ret = wait_for_completion_interruptible(		\
			&(program->completion));			\
			if (!ret)					\
				return ret;				\
		INIT_COMPLETION(program->completion);			\
		return c3_check_status(program->status);		\
	}								\
	return C3_OK;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_TO_CPU_32(x)  le32_to_cpu(x)
#define CPU_TO_C3_32(x)  cpu_to_le32(x)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef SG_ENABLED

/* Set Scatterlist */
#define CACHE_SYNC_AND_SET_SCATTER_LIST(addr, len, dir, dma_c3_t)	\
	do {								\
		int scatter_list_size_in_bytes;				\
		/* Remove Scatter list flag */				\
		addr = (unsigned char *)(((unsigned long)addr) & ~1);	\
		scatter_list_size_in_bytes =				\
			(((c3_map_info_t *)addr)->scatter_list_size) *	\
			sizeof(c3_hw_scatter_list_element_t);		\
		/* Change address to HW Scatter List address - */	\
		/*TODO:Writeback/invalidate original pointers ? */	\
		addr = (unsigned char *)((unsigned long)		\
			(((c3_map_info_t *)(addr))->hw_scatter_list)	\
			| 1);						\
		dma_c3_t->cpu_addr = (void *)(((unsigned long)addr) & ~1); \
		dma_c3_t->size = scatter_list_size_in_bytes;		\
		dma_c3_t->direction = dir;				\
		/* DMA address of SG | 1 (last bit=SG flag). */		\
		/* See C3 Version 3 Design Specification*/		\
		dma_c3_t->dma_addr =					\
		dma_map_single(NULL, (void *)(((unsigned long)addr) & ~1), \
			scatter_list_size_in_bytes,			\
			dir) | 1;					\
	} while (0)

#else

#define CACHE_SYNC_AND_SET_SCATTER_LIST(addr, len, dir, dma_c3_t)	\
	do {								\
		C3_ERROR("Scatter/Gather lists disabled or not present");\
		return -EINVAL;						\
	} while (0)

#endif
#define C3_UNMAP_ALL() do {						\
	struct list_head *pos, *l, *q;					\
	struct c3_dma_t *tmp;						\
	l = &((program->c3_dma_list).list);				\
	if (!list_empty(l)) {						\
		list_for_each_safe(pos, q, l) {				\
			tmp = list_entry(pos, struct c3_dma_t, list);	\
			dma_unmap_single(NULL, tmp->dma_addr, tmp->size \
					, tmp->direction);		\
			list_del(pos);					\
			kfree(tmp);					\
									\
		}							\
	}								\
} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* Alloc and Map buffer/SG list for DMA.
 * Also add it to a linked list of mapped buffers for the current request */
#define C3_PREPARE_DMA(dma_c3_t, addr_cpu, len, dir)			\
	do {								\
		if ((addr_cpu) == NULL) {				\
			C3_UNMAP_ALL();					\
			return C3_ERR;					\
		}							\
									\
		dma_c3_t = kmalloc(sizeof(struct c3_dma_t), GFP_ATOMIC);\
		if (!dma_c3_t) {					\
			C3_ERROR("Cannot alloc mem");			\
			return -ENOMEM;					\
		}							\
		if ((((unsigned long)addr_cpu) & 1) != 1) {		\
			dma_c3_t->cpu_addr = (void *)addr_cpu;		\
			dma_c3_t->size = len;				\
			dma_c3_t->direction = dir;			\
			dma_c3_t->dma_addr =				\
				dma_map_single(NULL, addr_cpu, len, dir);\
		}							\
		else {							\
			CACHE_SYNC_AND_SET_SCATTER_LIST(addr_cpu, len,	\
				dir, dma_c3_t);				\
		}							\
		if (dma_mapping_error(NULL, dma_c3_t->dma_addr)) {	\
			C3_ERROR("DMA mapping error");			\
			return C3_ERR;					\
		}							\
		list_add(&(dma_c3_t->list), &(program->c3_dma_list.list)); \
	} while (0)

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define INC_INDEX(index)						\
	do {								\
		index++;						\
		if (index >= C3_PROGRAM_QUEUE_SIZE)			\
			index = 0;					\
	} while (0)							\


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_KERN_DEBUG   KERN_DEBUG "[C3 DEBUG] - "
#define C3_KERN_INFO    KERN_INFO "[C3 INFO] - "
#define C3_KERN_ERR     KERN_ERR "[C3 ERROR] - "
#define C3_KERN_WARN    KERN_WARNING "[C3 WARNING] - "

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#ifdef DEBUG
#define DUMP_PROGRAM(prg, size)						\
	do {								\
		int i;							\
		printk(C3_KERN_DEBUG "Program:\n");			\
		for (i = 0; i < size; i++)				\
			printk(C3_KERN_DEBUG "%08x\n", prg[i]);		\
		printk(C3_KERN_DEBUG "END\n");				\
	} while (0)

#else
#define DUMP_PROGRAM(prg, size);
#endif

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#endif /* __C3_COMMON_H__ */
