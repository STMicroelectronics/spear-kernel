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
 * C3 driver Spear platform functions
 *
 * ST - 2007-04-10
 * Alessandro Miglietti
 * ----------------------------------------------------------------- */

#ifndef __C3_SPEAR_H__
#define __C3_SPEAR_H__

/* --------------------------------------------------------------------
 * INCLUDES
 * ----------------------------------------------------------------- */

/* For __init :                                                     */
#include <linux/init.h>
#include <linux/io.h>

/* ===================================================================
 *
 * INITIALIZATION AND CLEANUP
 *
 * ================================================================ */

unsigned int __init c3spear_init(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3spear_cleanup(unsigned int periph_id);

/* ===================================================================
 *
 * ASSIGNEMENTS
 *
 * ================================================================ */

unsigned int *c3spear_get_address(unsigned int periph_index);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3spear_periph_number(void);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3spear_interrupt_line(unsigned int spear_index);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

#endif /* __C3_SPEAR_H__ */
