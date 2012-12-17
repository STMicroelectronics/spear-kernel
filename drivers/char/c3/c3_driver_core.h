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
 * ----------------------------------------------------------------- */

#ifndef __C3_DRIVER_CORE_H__
#define __C3_DRIVER_CORE_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------  */

#include "c3_common.h"

/*-------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

unsigned int c3_get_program
(
	program_t **program,
	unsigned int *prg_index,
	unsigned int ids_index
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_enqueue_program
(
	unsigned int prg_index,
	unsigned int ids_index
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3_reset(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_busy(unsigned int ids_index);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
void c3_release(struct device *dev);


#endif /* __C3_DRIVER_CORE_H__ */
