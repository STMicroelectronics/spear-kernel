/*
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * SPEAr clk - Common routines
 */

#include <linux/clk-private.h>
#include <linux/clk-provider.h>
#include <linux/types.h>
#include "clk.h"

long clk_round_rate_index(struct clk_hw *hw, unsigned long drate,
		unsigned long parent_rate, clk_calc_rate calc_rate, u8 rtbl_cnt,
		int *index)
{
	unsigned long prev_rate, rate = 0;

	for (*index = 0; *index < rtbl_cnt; (*index)++) {
		prev_rate = rate;
		rate = calc_rate(hw, parent_rate, *index);
		if (drate < rate) {
			/* previous clock was best */
			if (*index) {
				rate = prev_rate;
				(*index)--;
			}
			break;
		}
	}

	if ((*index) == rtbl_cnt)
		(*index)--;

	return rate;
}

struct clk *clk_lookup_subtree(const char *name, struct clk *clk)
{
	struct clk *ret_clk;
	struct clk *child;
	struct hlist_node *tmp;

	if (!strcmp(clk->name, name))
		return clk;

	hlist_for_each_entry(child, tmp, &clk->children, child_node) {
		ret_clk = clk_lookup_subtree(name, child);
		if (ret_clk)
			return ret_clk;
	}

	return NULL;
}
