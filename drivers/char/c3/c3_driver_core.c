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
 * C3 driver core
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * - 2009-09-17 MGR Version is now 1.2
 * - 2010-01-21 MGR Version is now 1.3
 * - 2010-02-11 MGR Now date and time of build are printed.
 * - 2010-09-16 SK: Removed IPSec requests. Use kernel CryptoApi.
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"
#include "c3_irq.h"
#include "c3_driver_core.h"
#include "c3_driver_interface.h"
#include "c3_char_dev_driver.h"
#include "c3_mpcm.h"
#include "c3_autotest.h"

/* --------------------------------------------------------------------
 * MACROS
 * ----------------------------------------------------------------  */

MODULE_AUTHOR("ST Microelectronics");
MODULE_DESCRIPTION("C3 Device Driver");
MODULE_LICENSE("GPL");

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#define C3_DRIVER_VERSION\
	"2.0"

/*-------------------------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------------------- */

programs_buffer_t programs_buffer[C3_IDS_NUMBER];

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int *c3_base_address;
unsigned int c3_bus_index;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

extern c3_init_function_ptr_t c3_init_function;
extern c3_end_function_ptr_t c3_end_function;

/* TODO: For get_clk .... C3 clock is registered with .dev_id not .con_id
 * arch/arm/mach-spear13xx/clock.c */
static struct device dev = {
	.init_name	= "c3",
	.release	= c3_release,
};

/*-------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

unsigned int c3_busy(unsigned int ids_index)
{
	unsigned int status;

	c3_read_register(
		c3_base_address,
		C3_IDS0_offset + C3_ID_size * ids_index +
		C3_ID_SCR_offset,
		&status);

	return (status & C3_ID_BUSY_mask) == C3_ID_IS_mask;

} /* unsigned int c3_busy */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3_reset(void)
{
	unsigned int reg_val;
	int i;

	if (c3_base_address) {
		printk(C3_KERN_DEBUG "Resetting C3\n");

		/* Reset */
		c3_read_register(
			c3_base_address,
			C3_SYS_SCR_offset,
			&reg_val);

		c3_write_register(
			c3_base_address,
			C3_SYS_SCR_offset,
			reg_val | C3_SYS_ARST_mask);

		for (i = 0; i < C3_IDS_NUMBER; i++) {
			/* Enable interrupt on error */
			/* Enable interrupt on stop */
			/* Disable single step */
			c3_read_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				&reg_val);

			c3_write_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				(reg_val | C3_CONTROL_ERROR_INT_mask |
				 C3_CONTROL_STOP_INT_mask) &
				(~C3_CONTROL_SSE_mask));


		}

	}

	for (i = 0; i < C3_IDS_NUMBER; i++) {
		programs_buffer[i].start_index =
			programs_buffer[i].current_index =
				programs_buffer[i].end_index = 0;

		programs_buffer[i].c3_device_busy = 0;
	}

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_get_program(
	program_t **program,
	unsigned int *prg_index,
	unsigned int ids_index
)
{
	int queue_size;

	spin_lock_irqsave(&programs_buffer[ids_index].spinlock,
		programs_buffer[ids_index].flags);

	queue_size = programs_buffer[ids_index].end_index -
		     programs_buffer[ids_index].start_index;

	if (queue_size < 0)
		queue_size = C3_PROGRAM_QUEUE_SIZE + queue_size;

	if (queue_size >= (C3_PROGRAM_QUEUE_SIZE - 1)) {

#ifdef DEBUG
		printk(C3_KERN_WARN "Cannot enqueue request\n");
#endif

		spin_unlock_irqrestore(&programs_buffer[ids_index].spinlock,
			programs_buffer[ids_index].flags);

		return C3_ERR;

	} /* if (queue_size > C3_PROGRAM_QUEUE_SIZE) */

	*prg_index = programs_buffer[ids_index].end_index;
	*program = &programs_buffer[ids_index].program_queue
		[programs_buffer[ids_index].end_index];

	return C3_OK;

} /* static void c3_start_program */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_enqueue_program(
	unsigned int prg_index,
	unsigned int ids_index
)
{
	if (programs_buffer[ids_index].program_queue[prg_index].used_memory >
	    C3_PROGRAM_MEMORY_MAX_SIZE) {

#ifdef DEBUG
		printk(
			C3_KERN_WARN
			"Cannot copy c3 program in DMA-safe memory "
			"(program too big)\n");
#endif

		spin_unlock_irqrestore(&programs_buffer[ids_index].spinlock,
			programs_buffer[ids_index].flags);

		return C3_ERR;

	}

/*	DUMP_PROGRAM(
		programs_buffer[ids_index].
			program_queue[prg_index].program_memory,
		programs_buffer[ids_index].
			program_queue[prg_index].used_memory);*/

	INC_INDEX(programs_buffer[ids_index].end_index);

	if (!programs_buffer[ids_index].c3_device_busy) {
		programs_buffer[ids_index].current_index = prg_index;
		programs_buffer[ids_index].c3_device_busy = 1;

		c3_write_register(
			c3_base_address,
			C3_IDS0_offset + C3_ID_size * ids_index +
			C3_ID_IP_offset,
			programs_buffer[ids_index].
			program_queue[programs_buffer[ids_index].current_index]
			.program_memory_phys);


#ifdef DEBUG
/*		printk(
			C3_KERN_DEBUG "Program starts at %08x\n",
			programs_buffer[ids_index].
			program_queue[programs_buffer[ids_index].current_index]
			.program_memory_phys);*/
#endif

	} /* if (!programs_buffer[ids_index].c3_device_busy) */
	else{

#ifdef DEBUG
		printk(C3_KERN_WARN "C3 device is busy: request enqueued\n");
#endif

	}

	spin_unlock_irqrestore(&programs_buffer[ids_index].spinlock,
		programs_buffer[ids_index].flags);

	return C3_OK;

} /* static void  c3_start_program */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static void  __c3_cleanup(void)
{
	int i, j;
	unsigned int reg_val;

	c3_cdd_cleanup();

	if (c3_base_address) {

		for (i = 0; i < C3_IDS_NUMBER; i++) {
			/* Disable interrupt or error */
			c3_read_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				&reg_val);

			c3_write_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				reg_val & ~C3_CONTROL_ERROR_INT_mask);

			/* Disable interrupt or stop */
			c3_read_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				&reg_val);

			c3_write_register(
				c3_base_address,
				C3_IDS0_offset + C3_ID_size * i +
				C3_ID_SCR_offset,
				reg_val & ~C3_CONTROL_STOP_INT_mask);

		}       /* for (j = 0; j< C3_IDS_NUMBER; j++) */

	}               /* if (c3_base_address) */

	c3irq_cleanup();

	if (c3_bus_index)
		c3bus_cleanup(c3_bus_index);

	 /* if (c3_bus_index) */

	for (j = 0; j < C3_IDS_NUMBER; j++) {
		for (i = 0; i < C3_PROGRAM_QUEUE_SIZE; i++) {
			if (programs_buffer[j].
				program_queue[i].program_memory) {
				dma_free_coherent(
					NULL,
					C3_PROGRAM_MEMORY_MAX_SIZE,
					programs_buffer[j].program_queue[i]
					.program_memory,
					programs_buffer[j].program_queue[i]
					.program_memory_dma_handle);
			}
		}
	}

	if (c3_end_function)
		c3_end_function();

} /* static void __c3_cleanup ()*/

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int __init c3_init(void)
{
	int i, j, ret;
	struct clk *c3_clk;

#ifdef C3_LOCAL_MEMORY_ON
	unsigned int local_memory_size;
#endif

	printk(
		C3_KERN_INFO
		"Crypto Channel Controller (c) ST Microelectronics\n");
	printk(C3_KERN_INFO "Driver version = %s\n", C3_DRIVER_VERSION);
	printk(C3_KERN_INFO "Built on %s at %s\n", __DATE__, __TIME__);

	/* Get and Enable clock */
	ret = device_register(&dev);
	if (ret)
		printk(C3_KERN_ERR "Error registering device, err = %d\n", ret);

	c3_clk = clk_get(&dev, "c3");
	if (IS_ERR(c3_clk)) {
		ret = PTR_ERR(c3_clk);
		printk(C3_KERN_ERR "Cannot get C3 CLK, err = %d\n", ret);
		goto failure;
	}

	ret = clk_enable(c3_clk);
	if (ret) {
		printk(C3_KERN_ERR "Cannot enable C3 CLK\n");
		goto failure;
	}

	/* Bus init */
	if (c3bus_init() != C3_OK) {
		printk(C3_KERN_ERR "Error during bus initialization\n");
		goto failure;

	}

	/* Peripherals initialization */
	for (i = 0; i < c3bus_periph_number(); i++) {
		unsigned int *tmp_address;
		unsigned int c3_hwid;

		tmp_address = (unsigned int *)c3bus_get_address(i);

		c3_read_register(
			tmp_address,
			C3_SYS_HWID_offset,
			&c3_hwid);

		printk(
			C3_KERN_INFO "C3 device found (HID: %x)\n",
			c3_hwid);

		if ((c3_hwid & C3_HID_MASK) == (C3_HID & C3_HID_MASK)) {
			c3_base_address = tmp_address;
			c3_bus_index = i;

			if (c3irq_init(i, C3_HID) != C3_OK) {
				printk(C3_KERN_ERR "Cannot initialize driver\n");
				goto failure;
			}

			break;

		}

	}

	if (i == c3bus_periph_number()) {
		printk(C3_KERN_ERR "C3 device not found (HID: %x)\n", C3_HID);
		goto failure;

	}

	for (j = 0; j < C3_IDS_NUMBER; j++) {
		/* Internal structures initialization */
		spin_lock_init(&programs_buffer[j].spinlock);

		programs_buffer[j].start_index = programs_buffer[j].end_index =
							 0;

		for (i = 0; i < C3_PROGRAM_QUEUE_SIZE; i++) {
			programs_buffer[j].program_queue[i].program_memory =
				NULL;
			programs_buffer[j].program_queue[i].program_memory_phys
				= 0;
			programs_buffer[j].program_queue[i].
			program_memory_dma_handle = 0;
		}

		/* Programs memory allocation */
		for (i = 0; i < C3_PROGRAM_QUEUE_SIZE; i++) {
			programs_buffer[j].program_queue[i].program_memory =
				dma_alloc_coherent(
					NULL,
					C3_PROGRAM_MEMORY_MAX_SIZE,
					&programs_buffer[j].program_queue[i].
						program_memory_dma_handle,
					GFP_DMA);

			if (!(programs_buffer[j].
				program_queue[i].program_memory)) {
				printk(
					C3_KERN_ERR
					"Cannot allocate DMA-safe "
					"program memory\n");
				goto failure;

			}

			programs_buffer[j].program_queue[i].program_memory_phys
				=
					programs_buffer[j].program_queue[i].
					program_memory_dma_handle;
		}
	}

	/* Reset C3 */
	c3_reset();

	/* Load MPCM firmware */
	if (c3_mpcm_init() != C3_OK) {
		printk(C3_KERN_ERR "Cannot load MPCM firmware\n");
		__c3_cleanup();

		return -1;

	}

	if (c3_init_function && (c3_init_function() == C3_ERR)) {
		printk(C3_KERN_ERR "Cannot init module\n");
		__c3_cleanup();

		return -1;

	}

#ifdef USE_AES_SCATTER_GATHER

	printk(
		C3_KERN_INFO
		"Using scatter & gather for AES operations "
		"(threshold: %u bytes)\n",
		C3_SCATTER_GATHER_THRESOLD);


#endif

#ifdef C3_LOCAL_MEMORY_ON

	/* Get the size of the internal memory */
	c3_read_register(
		c3_base_address,
		C3_HIF_MSIZE_offset + C3_HIF_offset,
		&local_memory_size);

	if (local_memory_size < LOCAL_MEMORY_SIZE) {
		printk(C3_KERN_ERR
		       "Local memory size: %d bytes\n"
		       "Expected value: %d bytes\n",
		       local_memory_size,
		       LOCAL_MEMORY_SIZE);

		goto failure;
	}

	/* Set memory address */
	c3_write_register(
		c3_base_address,
		C3_HIF_MBAR_offset + C3_HIF_offset,
		LOCAL_MEMORY_ADDRESS);

	/* Enable local memory */
	c3_write_register(
		c3_base_address,
		C3_HIF_MCR_offset + C3_HIF_offset,
		C3_HIF_ENABLE_MEM_mask);

#endif  /* #ifdef C3_LOCAL_MEMORY_ON */


	if (c3_cdd_init() != C3_OK) {
		printk(C3_KERN_ERR "Cannot register character device driver\n");
		goto failure;

	}

	if (c3_autotest() != C3_OK) {
		printk(C3_KERN_ERR "AUTOTEST failed\n");
		goto failure;

	}

	return 0;

failure:
	printk(C3_KERN_ERR "Cannot init module\n");
	__c3_cleanup();

	return -1; /* TODO: codify errors */

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static void __exit c3_cleanup(void)
{
	__c3_cleanup();
	device_unregister(&dev);

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* To prevent warnings when driver is unloaded */
void c3_release(struct device *dev)
{
	/* TODO */
}

/* -------------------------------------------------------------------
 * MISCELLANEOUS
 * ---------------------------------------------------------------- */

module_init(c3_init);
module_exit(c3_cleanup);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
