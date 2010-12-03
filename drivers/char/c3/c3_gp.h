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

#ifndef __C3_GP_H__
#define __C3_GP_H__

/* --------------------------------------------------------------------
 * CONSTANTS
 * ----------------------------------------------------------------- */

#define C3_GP_DEBUG_BUF_SIZE                    ((unsigned int)2048)

/* --------------------------------------------------------------------
 * TYPES
 * ----------------------------------------------------------------- */

typedef struct {
	int read_idx;
	int write_idx;
	int buf_size;
	unsigned char buf[C3_GP_DEBUG_BUF_SIZE];

} c3_GP_debug_area_t;

/* --------------------------------------------------------------------
 * FUNCTIONS
 * ----------------------------------------------------------------- */

unsigned int c3_GP_init
(
	void
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

void c3_GP_cleanup
(
	void
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_debug_send_message
(
	unsigned char *message,
	unsigned int size
);

/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

unsigned int c3_GP_debug_receive_message
(
	unsigned char *message,
	unsigned int *size
);

#endif /* __C3_GP_H__ */
