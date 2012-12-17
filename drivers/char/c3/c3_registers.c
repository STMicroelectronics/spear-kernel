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
 * C3 driver registers functions
 *
 * ST - 2007-09-10
 * Alessandro Miglietti
 *
 * - 2011-03-03 SK: Replaced raw read/write with (ioread32/iowrite32).
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"
#include "c3_registers.h"

/* -------------------------------------------------------------------
* FUNCTIONS
* ----------------------------------------------------------------- */

inline void c3_write_register
(
	unsigned int *address,
	unsigned int offset,
	unsigned int data
)
{
	iowrite32(CPU_TO_C3_32(data),
		(void *)(address + (offset / sizeof(unsigned int))));
}

/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

inline void c3_read_register
(
	unsigned int *address,
	unsigned int offset,
	unsigned int *data
)
{
	*data = C3_TO_CPU_32(ioread32((void *)
		(address + offset / sizeof(unsigned int))));
}
