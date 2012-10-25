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
 * C3 driver IRQ functions
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 *
 * 2007-04-26 am  Bug fixing
 * 2010-09-16 SK: Unmap buffers before returning/callback.
 * 2011-03-08 SK: Changed sem,sync to completion.
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * --------------------------------------------------------------------
 */

#include "c3_common.h"

#include "c3_irq.h"

#include <linux/version.h>
#include <linux/interrupt.h>

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------  */

#define _C3_IRQ_ID ((unsigned int)0x43330000)

/* -------------------------------------------------------------------
 * MACROS
 * ---------------------------------------------------------------- */

#define _IRET_OK return IRQ_HANDLED
#define _IRET_NOK return IRQ_NONE

/*-------------------------------------------------------------------
 * VARIABLES
 * ----------------------------------------------------------------- */

static unsigned char irq;
static int irq_initialized;

static unsigned int __tasklet_ids_index;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

extern programs_buffer_t programs_buffer[C3_IDS_NUMBER];

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

extern unsigned int *c3_base_address;
extern unsigned int c3_bus_index;

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

DECLARE_TASKLET(
	c3_tasklet,
	c3_acknowledge_request,
	(unsigned long)0
	);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */


/* -------------------------------------------------------------------
* FUNCTIONS
* ----------------------------------------------------------------- */

irqreturn_t _c3irq_ISR_(
	int irq,
	void *periph_id_ptr
)
{

	/* Tag identifying the C3 hardware */
	if ((unsigned int)periph_id_ptr != _C3_IRQ_ID)
		_IRET_NOK;

	if (c3_irq_complete_request() != C3_OK)
		_IRET_NOK;
	else
		_IRET_OK;
}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

int c3irq_init(
	unsigned int bus_index,
	unsigned int periph_id
)
{
	/* Read bus configuration to get the interrupt number */
	irq = c3bus_interrupt_line(bus_index);

	/* Connect the ISR */
	if (request_irq(
		    (unsigned int)irq,
		    _c3irq_ISR_,
		    IRQF_DISABLED | IRQF_SHARED,
		    "C3",
		    (void *)((unsigned int)_C3_IRQ_ID))) {
		printk(C3_KERN_ERR "Failed connecting the ISR"
			"(irq: %d)\n", irq);

		return C3_ERR;

	} else {
		irq_initialized = 1;
		return C3_OK;
	}
}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

inline unsigned int c3_check_status(unsigned int status)
{
	if (status & (C3_IDS_ERROR_mask |
		      C3_IDS_BUS_ERROR_mask |
		      C3_IDS_CHANNEL_ERROR_mask))
		return C3_ERR;
	else
		return C3_OK;
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/* BH:Tasklet */
void c3_acknowledge_request(unsigned long data)
{
	unsigned int c3_res_status;
	program_t *current_program;
	unsigned int status;
	struct list_head *pos, *l, *q;
	struct c3_dma_t *tmp;

	status = programs_buffer[__tasklet_ids_index].program_queue
		[programs_buffer[__tasklet_ids_index].start_index].status;

	c3_res_status = c3_check_status(status);

	while (programs_buffer[__tasklet_ids_index].start_index !=
	       programs_buffer[__tasklet_ids_index].current_index) {
#ifdef DEBUG

		/*printk(
			C3_KERN_DEBUG "Tasklet [%d]: %x\n",
			programs_buffer[__tasklet_ids_index].start_index,
			programs_buffer[__tasklet_ids_index]
			.program_queue[programs_buffer[__tasklet_ids_index].
				       start_index].status);*/
#endif
		current_program =
			&programs_buffer[__tasklet_ids_index].program_queue
			[programs_buffer[__tasklet_ids_index].start_index];

		/* UNMAP */
		l = &((current_program->c3_dma_list).list);
		list_for_each_safe(pos, q, l) {
			tmp = list_entry(pos, struct c3_dma_t, list);
			dma_unmap_single(NULL, tmp->dma_addr, tmp->size,
					 tmp->direction);
			list_del(pos);
			kfree(tmp);
		}

		if (current_program->c3_callback) {
			current_program->c3_callback(
				current_program->c3_callback_param,
				c3_res_status);
		} else {
			/* No callback set. Wakeup process */
			complete(&current_program->completion);
		}

		INC_INDEX(programs_buffer[__tasklet_ids_index].start_index);
	}
}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

static int __compute_ids_index(unsigned int interrupt_field)
{
	int count = 0;

	while (interrupt_field) {
		interrupt_field = interrupt_field >> 1;
		count++;
	}

	return count - 1;

}

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_irq_complete_request(void)
{
	unsigned int interrupt_field;
	unsigned int status;
	int ids_index;

	program_t *current_program;

	/* Is our c3? */
	c3_read_register(
		c3_base_address,
		C3_SYS_SCR_offset,
		&status);

	interrupt_field = ((status & C3_SYS_ISDn_mask) >> C3_SYS_ISDn_shift);

	ids_index = __compute_ids_index(interrupt_field);

	if (ids_index == -1 || ids_index >= C3_IDS_NUMBER)
		return C3_ERR;

	/* Get status */
	c3_read_register(
		c3_base_address,
		C3_IDS0_offset + C3_ID_size * ids_index +
		C3_ID_SCR_offset,
		&status);

	/* Clear interrupt */
	status |= C3_ID_IS_mask;

	c3_write_register(
		c3_base_address,
		C3_IDS0_offset + C3_ID_size * ids_index +
		C3_ID_SCR_offset,
		status);

	current_program =
		&programs_buffer[ids_index].program_queue
		[programs_buffer[ids_index].current_index];

	INC_INDEX(programs_buffer[ids_index].current_index);

	if (programs_buffer[ids_index].end_index !=
	    programs_buffer[ids_index].current_index) {
		/* Programs queue is NOT empty
		 * Execute the program */
		c3_write_register(
			c3_base_address,
			C3_IDS0_offset + C3_ID_size * ids_index +
			C3_ID_IP_offset,
			programs_buffer[ids_index].program_queue
			[programs_buffer[ids_index].current_index].
			program_memory_phys);

	} else {
		programs_buffer[ids_index].c3_device_busy = 0;
	}

	current_program->status = status;

	__tasklet_ids_index = ids_index;
	tasklet_schedule(&c3_tasklet);

	return C3_OK;

} /* static void c3_irq_complete_request */

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3irq_cleanup(void)
{
	if (irq_initialized) {
		/* Kill the tasklet */
		tasklet_kill(&c3_tasklet);
		/* Release the irq */
		free_irq(irq, (void *)((unsigned int)_C3_IRQ_ID));

	}
}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
