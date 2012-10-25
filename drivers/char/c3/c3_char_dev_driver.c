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
 * C3 character device driver
 *
 * ST - 2007-05-18
 * Alessandro Miglietti
 *
 * - 2009-09-08 MGR Added PKA v.7 support.
 * - 2010-02-09 MGR Bugfix: split AES and DES functions.
 * - 2010-09-16 SK: Changed S/G list implementation.(TODO)
 * - 2011-02-21 SK: Removed ioctl/fsync SP and GP functions.
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>

#include "c3_char_dev_driver.h"
#include "c3_driver_interface.h"
#include "c3_driver_core.h"
#include "c3_char_dev_driver_instructions.h"
#include "c3_common.h"

/* --------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------  */

typedef struct {
		struct semaphore sem;
		volatile unsigned int c3_status;
		volatile unsigned int c3_stop;
		volatile unsigned int c3_clean;
		c3_cdd_param_list_t k_param_list;

} callback_param_t;

/* --------------------------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------------------  */

static int _c3_major_ = -1;
static const struct file_operations _c3_device_ops_;

extern c3_instr_params_t c3_instr_params[];

/* --------------------------------------------------------------------
 * MACROS
 * ---------------------------------------------------------------- */

#define MAX_BUFFER_SIZE 0x10000

/* --------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------  */

static inline
void _c3_unmap_user(unsigned int write_enable, c3_map_info_t *map_info)
{
	unsigned long i;
	struct page **pages = (struct page **) map_info->__k_pages;

	if (pages) {
		for (i = 0; i < map_info->scatter_list_size; i++) {
			if (write_enable)
				SetPageDirty(pages[i]);
			page_cache_release(pages[i]);

		}

	}

	kfree(pages);

	kfree(map_info->hw_scatter_list);

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static inline
unsigned int _c3_map_user(unsigned int write_enable,
	unsigned char *user_buffer, unsigned int len, c3_map_info_t *map_info)
{
	int nr_pages;
	struct page **pages;
	unsigned long i;
	unsigned int page_count;

	map_info->original_pointer = user_buffer;
	map_info->hw_scatter_list = NULL;
	map_info->__k_pages = NULL;

	/* Allocate data structure */
	page_count = ((((unsigned int) user_buffer) & ~PAGE_MASK) + len
		+ ~PAGE_MASK) >> PAGE_SHIFT;

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] Page count: %d\n", page_count);
#endif

	pages = kmalloc(page_count * sizeof(struct page *), GFP_KERNEL);

	if (!pages) {
		printk(
			C3_KERN_ERR
			"[CDD] Cannot allocate \"pages\" structure\n");
		return C3_ERR;

	}

	map_info->__k_pages = pages;

	/* get user pages */
	down_read(&current->mm->mmap_sem);

	nr_pages = get_user_pages(current, current->mm,
		((unsigned int) user_buffer) & PAGE_MASK, page_count,
		write_enable, 0, pages, NULL);

	up_read(&current->mm->mmap_sem);

	map_info->scatter_list_size = nr_pages;

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] get_user_pages returns: %d\n", nr_pages);
#endif

	/* Check if it is a success */
	if (page_count != nr_pages)
		goto _map_failure;

	/* Fill scatter list descriptors */
	map_info->hw_scatter_list = kmalloc((nr_pages)
		* sizeof(c3_hw_scatter_list_element_t), GFP_DMA);

#ifdef DEBUG
	printk(
		C3_KERN_DEBUG "[CDD] HW scatter list address: %p\n",
		map_info->hw_scatter_list);
#endif

	for (i = 0; i < nr_pages; i++) {
		int page_len;

		map_info->hw_scatter_list[i].address = page_to_phys(pages[i])
			+ (i == 0 ? (((unsigned long) user_buffer) & ~PAGE_MASK)
				: 0);

		page_len = ((map_info->hw_scatter_list[i].address & PAGE_MASK)
			+ PAGE_SIZE) - map_info->hw_scatter_list[i].address;

		if (page_len >= len)
			page_len = len;

		map_info->hw_scatter_list[i].size = page_len;
		len -= page_len;

#ifdef DEBUG
		printk(C3_KERN_DEBUG "[CDD] Page[%d]\n", (int)i);

		printk(
			C3_KERN_DEBUG "[CDD]  - Address: %p\n",
			(void *)map_info->hw_scatter_list[i].address);

		printk(
			C3_KERN_DEBUG "[CDD]  - Size: %d\n",
			(int)map_info->hw_scatter_list[i].size);
#endif

	}

	/* Last element of SG list. See C3 Version 3 Design Specification */
	map_info->hw_scatter_list[nr_pages - 1].size = 0;

	return C3_OK;

_map_failure:

	_c3_unmap_user(write_enable, map_info);

	return C3_ERR;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static unsigned int _c3_user2kernel(c3_buffer_t *buffer,
	unsigned int scatter_gather_enable)
{
	int copy = 0;

	if (buffer->size >= MAX_BUFFER_SIZE) {
		printk(C3_KERN_ERR "size is greater than boundary condit\n");
		return C3_ERR;
	}

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] User ->Kernel process\n");

	printk(
		C3_KERN_DEBUG "[CDD] User ptr: %p\n",
		buffer->user_ptr);
#endif

	if (scatter_gather_enable) {
		if ((((unsigned int) buffer->user_ptr) % 4) == 0
			&& (buffer->size > C3_SCATTER_GATHER_THRESOLD)) {
#ifdef DEBUG
			printk(C3_KERN_DEBUG
				"[CDD] Trying to create scatter list\n");
#endif
			buffer->kernel_ptr = kmalloc(sizeof(c3_map_info_t),
				GFP_KERNEL);
			if (!buffer->kernel_ptr) {
				C3_ERROR("Cannot alloc mem");
				return C3_ERR;

			}

			if (_c3_map_user(
				((buffer->type == C3_BUFFER_OUTPUT) ? 1 : 0),
				buffer->user_ptr, buffer->size,
				(c3_map_info_t *) buffer->kernel_ptr)
				!= C3_OK) {
				kfree(buffer->kernel_ptr);
#ifdef DEBUG
				printk(C3_KERN_ERR
					"[CDD] Unable to map pages.\n");
				return C3_ERR;
#endif
			} else {
				/* Set First bit of the PTR: it will be
				 * considered a PTR to a Scatter List */
				buffer->kernel_ptr
					= (unsigned char *) ((unsigned int)
						buffer->kernel_ptr | 1);

#ifdef DEBUG
				printk(C3_KERN_DEBUG
					"[CDD] Map info structure address:"
					" %p\n",
					buffer->kernel_ptr);
#endif

				return C3_OK;

			}

		}

	}

	buffer->kernel_ptr = kmalloc(buffer->size, GFP_DMA);
	if (!buffer->kernel_ptr) {
		printk(
			C3_KERN_ERR
			"[CDD] Cannot allocate memory - size: %d bytes\n",
			buffer->size);

		return C3_ERR;
	}

#ifdef DEBUG

	printk(
		C3_KERN_DEBUG "[CDD] Kernel ptr: %p\n",
		buffer->kernel_ptr);

	printk(
		C3_KERN_DEBUG "[CDD] Size: %d\n",
		(unsigned int)buffer->size);

#endif
	copy = copy_from_user(buffer->kernel_ptr, buffer->user_ptr,
		buffer->size);

	if (copy) {
		printk(
			C3_KERN_ERR
			"[CDD]: copy_from_user Cannot copy full data, bytes NOT"
			" copied = %d, total size = %d\n", copy, buffer->size);
		return C3_ERR;
	}
#ifdef DEBUG
	else {
/*		int cnt = 0;
		printk(C3_KERN_DEBUG "[CDD] Kernel buffer:\n");
		for (cnt = 0; cnt < buffer->size / 4; cnt++)
			printk(
				C3_KERN_DEBUG "\t[@%d] %08x\n", cnt,
				cpu_to_be32((unsigned int)(((unsigned int *)
				buffer->kernel_ptr)[cnt])));*/
	}
#endif  /* DEBUG */

	return C3_OK;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static unsigned int _c3_kernel2user(c3_buffer_t *buffer)
{
	unsigned int err = C3_OK;
	int copy = 0;

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD]  Kernel ->User process\n");
#endif

	if ((((unsigned int) buffer->kernel_ptr) & 1) == 1) {
#ifdef DEBUG
		printk(
			C3_KERN_DEBUG
			"SCATTERLIST : [CDD] Unmapping user buffer\n");
#endif
		_c3_unmap_user((buffer->type == C3_BUFFER_OUTPUT) ? 1 : 0,
			(c3_map_info_t *) (((unsigned int) buffer->kernel_ptr)
				& ~1));

		if ((unsigned char *) (((unsigned int)
			buffer->kernel_ptr) & ~1)) {
			kfree((unsigned char *) (((unsigned int)
				buffer->kernel_ptr) & ~1));
			buffer->kernel_ptr = NULL;
		}

		return C3_OK;

	} else {

#ifdef DEBUG

		printk(
			C3_KERN_DEBUG "[CDD] User ptr: %pn",
			buffer->user_ptr);

		printk(
			C3_KERN_DEBUG "[CDD] Kernel ptr: %p\n",
			buffer->kernel_ptr);

		printk(
			C3_KERN_DEBUG "[CDD] Size: %d\n",
			(unsigned int)buffer->size);

		printk(
			C3_KERN_DEBUG "[CDD] Type: %d\n",
			(unsigned int)buffer->type);

#endif

		if (buffer->type == C3_BUFFER_OUTPUT) {
			copy = copy_to_user(buffer->user_ptr,
				buffer->kernel_ptr, buffer->size);
			if (copy) {
				C3_ERROR(
					"Copy to User: Bytes NOT Copied = %d,"
					"Total size = %d", copy, buffer->size);
				err = C3_ERR;

			}
		}

		kfree(buffer->kernel_ptr);
		buffer->kernel_ptr = NULL;
	}

	return err;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static void _c3_free_buffers(c3_cdd_param_list_t *k_param_list)
{
	int i;

	for (i = 0; i < k_param_list->buffer_list.buffers_number; i++) {
		if (k_param_list->buffer_list.buffers[i].kernel_ptr) {
			/* Last bit is used to indicate if the ptr is
			 * referred to a Scatter/gather list */
			kfree((unsigned char *) ((unsigned int)
			(k_param_list->buffer_list.buffers[i].kernel_ptr)
			& ~1));
		}

	}
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_cdd_init(void)
{
	/* Register the character device :                                */
	_c3_major_ = register_chrdev(C3_MAJOR, "C3", &_c3_device_ops_);

	if (_c3_major_ < 0) {
		printk(C3_KERN_ERR "[CDD] Failed registering device driver\n");

		return C3_ERR;

	}

	return C3_OK;
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3_cdd_cleanup(void)
{

	if (_c3_major_ >= 0) {
		printk(
			C3_KERN_INFO
			"[CDD] Unregistering character device driver\n");
		unregister_chrdev(_c3_major_, "C3");

	}
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int _c3_cdd_open_(struct inode *inode_ptr, struct file *file_ptr)
{
	/* Device minor number. We use it to identify the channel in use.
	 * (not used right now)*/
	/* int minor = (int) MINOR(inode_ptr->i_rdev); */

	/* Returned value :*/
	int status = 0;

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] Device driver \"open\" function\n");
#endif

	/* TODO */
	return status;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int _c3_cdd_release_(struct inode *inode_ptr, struct file *file_ptr)
{
#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] Device driver \"close\" function\n");
#endif
	/* TODO */
	return 0;
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int prepare_param_list(c3_cdd_param_list_t *k_param_list)
{
	unsigned int i = 0, j;

	while (c3_instr_params[i].instruction != -1) {
		/* TODO: use an hash table! */
		if (k_param_list->instruction == c3_instr_params[i].instruction)
			break;

		i++;
	}

	if (c3_instr_params[i].instruction == -1) {
#ifdef DEBUG
		printk(
			C3_KERN_DEBUG "[CDD] BUG: Instruction not supported\n");
#endif
		return C3_ERR;

	}

	k_param_list->buffer_list.buffers_number
		= c3_instr_params[i].params_number;

	for (j = 0; j < c3_instr_params[i].params_number; j++)
		k_param_list->buffer_list.buffers[j].kernel_ptr = NULL;

	/* Copy buffers from user space */
	for (j = 0; j < c3_instr_params[i].params_number; j++) {

#ifdef DEBUG
		printk(
			C3_KERN_DEBUG "[CDD] Processing param %d\n",
			c3_instr_params[i].params_indexes[j].ptr_index);
#endif

		k_param_list->buffer_list.buffers[j].user_ptr
			= (unsigned char *) k_param_list->
			params[c3_instr_params[i].params_indexes[j].ptr_index];

		if (c3_instr_params[i].params_indexes[j].size_index != -1) {
			k_param_list->buffer_list.buffers[j].size
				= k_param_list->
				params[c3_instr_params[i].params_indexes[j].
				       size_index];

		} else {
			k_param_list->buffer_list.buffers[j].size
				= c3_instr_params[i].params_indexes[j].size;

		}

		k_param_list->buffer_list.buffers[j].type
			= c3_instr_params[i].params_indexes[j].type;

		if (_c3_user2kernel(&k_param_list->buffer_list.buffers[j],
			c3_instr_params[i].scatter_gather_enable) != C3_OK) {
			printk(C3_KERN_ERR "[CDD] Cannot copy buffer\n");
			return C3_ERR;

		}
		k_param_list->
		params[c3_instr_params[i].params_indexes[j].ptr_index]
			= (unsigned int)
			k_param_list->buffer_list.buffers[j].kernel_ptr;

	}

	return C3_OK;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static unsigned int execute_program(c3_cdd_param_list_t *k_param_list,
	c3_callback_ptr_t callback, void *callback_param)
{
	unsigned int c3_status;
	int i = 0;

	/* Call the right C3 driver function */
	while (c3_instr_params[i].fun) {
		if (c3_instr_params[i].instruction ==
			k_param_list->instruction) {
			switch (c3_instr_params[i].function_type) {
			case C3_CIPHER_RANDOMIZE_FUNCTION:
				c3_status
					= ((c3_cipher_randomize_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						callback, callback_param);
				break;

			case C3_CIPHER_AES_CBC_CTR_FUNCTION:
				c3_status
					= ((c3_cipher_aes_cbc_ctr_function_t)(
						c3_instr_params[i] .fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						(unsigned char *)
						k_param_list->params[5],
						(unsigned int)
						k_param_list->params[6],
						(unsigned int)
						k_param_list->params[7],
						callback, callback_param);
				break;

			case C3_CIPHER_AES_ECB_FUNCTION:
				c3_status
					= ((c3_cipher_aes_ecb_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						(unsigned int)
						k_param_list->params[5],
						callback, callback_param);
				break;

			case C3_CIPHER_DES_CBC_FUNCTION:
				c3_status
					= ((c3_cipher_des_cbc_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						(unsigned char *)
						k_param_list->params[5],
						callback, callback_param);
				break;

			case C3_CIPHER_DES_ECB_FUNCTION:
				c3_status
					= ((c3_cipher_des_ecb_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						callback, callback_param);
				break;

			case C3_HASH_INIT_FUNCTION:
				c3_status
					= ((c3_hash_init_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						callback, callback_param);
				break;

			case C3_HASH_APPEND_FUNCTION:
				c3_status
					= ((c3_hash_append_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_HASH_END_FUNCTION:
				c3_status
					= ((c3_hash_end_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						callback, callback_param);
				break;

			case C3_HASH_FUNCTION:
				c3_status
					= ((c3_hash_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_HMAC_INIT_FUNCTION:
				c3_status
					= ((c3_hmac_init_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_HMAC_APPEND_FUNCTION:
				c3_status
					= ((c3_hmac_append_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_HMAC_END_FUNCTION:
				c3_status
					= ((c3_hmac_end_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_HMAC_FUNCTION:
				c3_status
					= ((c3_hmac_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						callback, callback_param);
				break;

			case C3_PKA_EXP_FUNCTION:
				c3_status
					= ((c3_pka_exp_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned char *)
						k_param_list->params[3],
						(unsigned int)
						k_param_list->params[4],
						(sca_t) k_param_list->params[5],
						callback, callback_param);
				break;

			case C3_PKA_MONTY_PAR_FUNCTION:
				c3_status
					= ((c3_pka_monty_par_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ECC_MUL_FUNCTION:
				c3_status
					= ((c3_pka_ecc_mul_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						(sca_t) k_param_list->params[4],
						callback, callback_param);
				break;

			case C3_PKA_ECC_DSA_SIGN_FUNCTION:
				c3_status
					= ((c3_pka_ecc_dsa_sign_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						(sca_t) k_param_list->params[4],
						callback, callback_param);
				break;

			case C3_PKA_ECC_DSA_VERIFY_FUNCTION:
				c3_status
					= ((c3_pka_ecc_dsa_verify_function_t)(
						c3_instr_params[i] .fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(sca_t) k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MUL_FUNCTION:
				c3_status
					= ((c3_pka_arith_mul_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_ADD_FUNCTION:
				c3_status
					= ((c3_pka_arith_add_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_SUB_FUNCTION:
				c3_status
					= ((c3_pka_arith_sub_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_CMP_FUNCTION:
				c3_status
					= ((c3_pka_arith_cmp_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MOD_RED_FUNCTION:
				c3_status
					= ((c3_pka_arith_mod_red_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MOD_INV_FUNCTION:
				c3_status
					= ((c3_pka_arith_mod_inv_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MOD_ADD_FUNCTION:
				c3_status
					= ((c3_pka_arith_mod_add_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MOD_SUB_FUNCTION:
				c3_status
					= ((c3_pka_arith_mod_sub_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_ARITH_MONTY_MUL_FUNCTION:
				c3_status
					= ((c3_pka_arith_monty_mul_function_t)(
						c3_instr_params[i].fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						callback, callback_param);
				break;

			case C3_PKA_RSA_CRT_FUNCTION:
				c3_status
					= ((c3_pka_rsa_crt_function_t)(
						c3_instr_params[i]. fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						(unsigned char *)
						k_param_list->params[2],
						(unsigned int)
						k_param_list->params[3],
						(sca_t) k_param_list->params[4],
						callback, callback_param);
				break;

			case C3_PKA_MEM_RESET_FUNCTION:
				c3_status = ((c3_pka_mem_reset_function_t)(
					c3_instr_params[i].fun))(callback,
					callback_param);
				break;

			case C3_PKA_MEM_READ_FUNCTION:
				c3_status
					= ((c3_pka_mem_read_function_t)(
						c3_instr_params[i] .fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned char *)
						k_param_list->params[1],
						(unsigned int)
						k_param_list->params[2],
						callback, callback_param);
				break;

			case C3_RNG_FUNCTION:
				c3_status
					= ((c3_rng_function_t)(
						c3_instr_params[i] .fun))(
						(unsigned char *)
						k_param_list->params[0],
						(unsigned int)
						k_param_list->params[1],
						callback, callback_param);
				break;

			default:
				printk(
					C3_KERN_ERR
					"[CDD] BUG: Fn Type Not supported\n");
				return C3_ERR;

			}

			return c3_status;
		}

		i++;

	}

	printk(C3_KERN_ERR "[CDD] Instruction not supported\n");
	return C3_ERR;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define MAX_CB_PARAMS       1024

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int idx;
static callback_param_t callback_param[MAX_CB_PARAMS];

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static void callback(void *param, unsigned int status)
{
#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] Device driver \"callback\" function\n");
	printk(C3_KERN_DEBUG "[CDD] Status: %d\n", status);
#endif

	((callback_param_t *) param)->c3_stop = 1;

	if (!((callback_param_t *) param)->c3_clean) {
#ifdef DEBUG
		printk(C3_KERN_DEBUG "[CDD] Waking up driver\n");
#endif
		((callback_param_t *) param)->c3_status = status;
		up(&((callback_param_t *) param)->sem);
	} else {
#ifdef DEBUG
		printk(
			C3_KERN_DEBUG
			"[CDD] Freeing kernel buffers (in driver callback)\n");
#endif
		_c3_free_buffers(&((callback_param_t *) param)->k_param_list);
	}
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* TODO:Warning stack size */
c3_cdd_param_list_t k_param_list;

static ssize_t _c3_cdd_write_(struct file *file_ptr, const char *buff_ptr,
	size_t size, loff_t *off_ptr)
{
	c3_cdd_param_list_t *u_param_list = (c3_cdd_param_list_t *) buff_ptr;
	callback_param_t *cb_param;
	int i;
	int err;
	unsigned int c3_status;

	cb_param = &callback_param[idx++];

	if (idx == MAX_CB_PARAMS)
		idx = 0;

	sema_init(&cb_param->sem, 0);

#ifdef DEBUG
	printk(C3_KERN_DEBUG "[CDD] Device driver \"write\" function\n");
#endif

	if ((sizeof(c3_cdd_param_list_t) != size) || (copy_from_user(
		&k_param_list, u_param_list, sizeof(c3_cdd_param_list_t)))) {
		printk(C3_KERN_ERR "[CDD] Invalid argument\n");
		return -EINVAL;

	}

	k_param_list.buffer_list.buffers_number = 0;

	/* Prepare param list */
	if (prepare_param_list(&k_param_list)) {
		printk(C3_KERN_ERR "[CDD] Error/s reading parameter list\n");

		err = -EINVAL;
		goto quit_write;

	}

	cb_param->c3_status = C3_ERR;
	cb_param->c3_clean = 0;
	cb_param->c3_stop = 0;

	c3_status = execute_program(&k_param_list, callback, (void *) cb_param);
	if (c3_status == C3_ERR) {
		printk(C3_KERN_ERR "[CDD] Cannot execute program\n");
		err = -EIO;
		goto quit_write;
	} else if (c3_status == C3_SKIP) {
		err = -ENOTSUPP;
		goto quit_write;
	}

	/* Wait the execution of the C3 program */
	if (down_interruptible(&cb_param->sem)) {
		local_irq_disable();
		if (!cb_param->c3_stop) {
#ifdef DEBUG
			printk(C3_KERN_WARN "[CDD] C3 processing suspended\n");
#endif

			memcpy(&cb_param->k_param_list, &k_param_list,
				sizeof k_param_list);
			cb_param->c3_clean = 1;
			local_irq_enable();

			return -ERESTARTSYS;
		} else {
			local_irq_enable();
#ifdef DEBUG
			printk(
				C3_KERN_DEBUG
				"[CDD] Signal received, but C3 is idle\n");
#endif
			cb_param->c3_status = C3_OK;
		}

	}

	if (cb_param->c3_status != C3_OK) {
		printk(C3_KERN_ERR "[CDD] Error executing program\n");
		err = -EIO;
		goto quit_write;
	}

	/* Copy back buffers to user space and free kernel buffers */
	for (i = 0; i < k_param_list.buffer_list.buffers_number; i++) {
		if (_c3_kernel2user(&k_param_list.buffer_list.buffers[i])
			!= C3_OK) {
			printk(C3_KERN_ERR "[CDD] Cannot copy buffer\n");

			err = -EIO;
			goto quit_write;

		}

	}

	return size;

quit_write:

	_c3_free_buffers(&k_param_list);

	return err;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static const struct file_operations _c3_device_ops_ = {
	.open = _c3_cdd_open_,
	.release = _c3_cdd_release_,
	.read = NULL,
	.write = _c3_cdd_write_
};

