/*
 * Helpers for open firmware matrix keyboard bindings
 *
 * Copyright (C) 2012 Google, Inc
 *
 * Author:
 *	Olof Johansson <olof@lixom.net>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/input/matrix_keypad.h>
#include <linux/export.h>
#include <linux/gfp.h>
#include <linux/slab.h>

/**
 * matrix_keypad_of_build_keymap - convert platform DT keymap into matrix keymap
 * @idev: pointer to struct input_dev; used for getting keycode, keybit and
 * keycodemax.
 * @row_shift: number of bits to shift row value by to advance to the next
 * line in the keymap
 * @propname: Device Tree property name to be used for reading keymap. If passed
 * as NULL, "linux,keymap" is used.
 *
 * This function creates an array of keycodes, by reading propname property from
 * device tree passed, that is suitable for using in a standard matrix keyboard
 * driver that uses row and col as indices.
 *
 * Expectation from user driver: idev must be initialized with following fields:
 * dev.parent, keycode, keybit and keycodemax.
 */
int matrix_keypad_of_build_keymap(struct input_dev *idev,
		unsigned int row_shift, const char *propname)
{
	struct device *dev = idev->dev.parent;
	struct device_node *np = dev->of_node;
	unsigned short *keycode;
	const __be32 *prop;
	unsigned int proplen, i, size, col_range = 1 << row_shift, index;

	if (!np || !idev)
		return -ENODEV;

	if (!propname)
		propname = "linux,keymap";

	prop = of_get_property(np, propname, &proplen);
	if (!prop) {
		dev_err(dev, "OF: %s property not defined in %s\n", propname,
				np->full_name);
		return -ENODEV;
	}

	if (proplen % sizeof(u32)) {
		dev_warn(dev, "Malformed keycode property %s in %s\n", propname,
				np->full_name);
		return -EINVAL;
	}

	size = proplen / sizeof(u32);
	if (size > idev->keycodemax) {
		dev_err(dev, "OF: %s size overflow\n", propname);
		return -EINVAL;
	}

	keycode = idev->keycode;
	for (i = 0; i < size; i++) {
		unsigned int key = be32_to_cpup(prop + i);
		unsigned int row = KEY_ROW(key);
		unsigned int col = KEY_COL(key);
		unsigned short code = KEY_VAL(key);

		if (col >= col_range) {
			dev_err(dev, "OF: %s: column %x overflowed its range %d\n",
					propname, col, col_range);
			return -EINVAL;
		}

		index = MATRIX_SCAN_CODE(row, col, row_shift);
		if (index > idev->keycodemax) {
			dev_err(dev, "OF: %s index overflow\n", propname);
			return -EINVAL;
		}
		keycode[index] = code;
		__set_bit(code, idev->keybit);
	}
	__clear_bit(KEY_RESERVED, idev->keybit);

	return 0;
}
EXPORT_SYMBOL_GPL(matrix_keypad_of_build_keymap);
