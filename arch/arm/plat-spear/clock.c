/*
 * arch/arm/plat-spear/clock.c
 *
 * Clock framework for SPEAr platform
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <plat/clock.h>

static DEFINE_SPINLOCK(clocks_lock);
static LIST_HEAD(root_clks);
#ifdef CONFIG_DEBUG_FS
static LIST_HEAD(clocks);
#endif

static void propagate_rate(struct clk *);
#ifdef CONFIG_DEBUG_FS
static int clk_debugfs_reparent(struct clk *);
#endif

static int clk_is_enabled(struct clk *clk)
{
	unsigned int val;

	if (clk->flags & ALWAYS_ENABLED)
		return 1;

	BUG_ON(!clk->en_reg);
	val = readl(clk->en_reg);
	val = val & (1 << clk->en_reg_bit);

	if (unlikely(clk->flags & RESET_TO_ENABLE))
		return !val;
	else
		return !!val;
}

static int generic_clk_enable(struct clk *clk)
{
	unsigned int val;

	if (!clk->en_reg)
		return -EFAULT;

	val = readl(clk->en_reg);
	if (unlikely(clk->flags & RESET_TO_ENABLE))
		val &= ~(1 << clk->en_reg_bit);
	else
		val |= 1 << clk->en_reg_bit;

	writel(val, clk->en_reg);

	return 0;
}

static void generic_clk_disable(struct clk *clk)
{
	unsigned int val;

	if (!clk->en_reg)
		return;

	val = readl(clk->en_reg);
	if (unlikely(clk->flags & RESET_TO_ENABLE))
		val |= 1 << clk->en_reg_bit;
	else
		val &= ~(1 << clk->en_reg_bit);

	writel(val, clk->en_reg);
}

/* generic clk ops */
static struct clkops generic_clkops = {
	.enable = generic_clk_enable,
	.disable = generic_clk_disable,
};

/* returns current programmed clocks clock info structure */
static struct pclk_info *pclk_info_get(struct clk *clk)
{
	unsigned int val, i;
	struct pclk_info *info = NULL;

	val = (readl(clk->pclk_sel->pclk_sel_reg) >> clk->pclk_sel_shift)
		& clk->pclk_sel->pclk_sel_mask;

	for (i = 0; i < clk->pclk_sel->pclk_count; i++) {
		if (clk->pclk_sel->pclk_info[i].pclk_val == val)
			info = &clk->pclk_sel->pclk_info[i];
	}

	return info;
}

/*
 * Set Update pclk, and pclk_info of clk and add clock sibling node to current
 * parents children list
 */
static void clk_reparent(struct clk *clk, struct pclk_info *pclk_info)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	list_del(&clk->sibling);
	list_add(&clk->sibling, &pclk_info->pclk->children);

	clk->pclk = pclk_info->pclk;
	spin_unlock_irqrestore(&clocks_lock, flags);

#ifdef CONFIG_DEBUG_FS
	clk_debugfs_reparent(clk);
#endif
}

void do_clk_disable(struct clk *clk)
{
	if (!clk)
		return;

	if (!clk->usage_count) {
		WARN_ON(1);
		return;
	}

	clk->usage_count--;

	if (clk->usage_count == 0) {
		/*
		 * Surely, there are no active childrens or direct users
		 * of this clock
		 */
		if (clk->pclk)
			do_clk_disable(clk->pclk);

		if (clk->ops && clk->ops->disable)
			clk->ops->disable(clk);
	}
}

int do_clk_enable(struct clk *clk)
{
	int ret = 0;

	if (!clk)
		return -EFAULT;

	if (clk->usage_count == 0) {
		if (clk->pclk) {
			ret = do_clk_enable(clk->pclk);
			if (ret)
				goto err;
		}
		if (clk->ops && clk->ops->enable) {
			ret = clk->ops->enable(clk);
			if (ret) {
				if (clk->pclk)
					do_clk_disable(clk->pclk);
				goto err;
			}
			/*
			 * Now that we have enabled the clock please
			 * recalc
			 */
			if (clk->recalc)
				clk->recalc(clk);
		}
	}
	clk->usage_count++;
err:
	return ret;
}

/*
 * clk_enable - inform the system when the clock source should be running.
 * @clk: clock source
 *
 * If the clock can not be enabled/disabled, this should return success.
 *
 * Returns success (0) or negative errno.
 */
int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&clocks_lock, flags);
	ret = do_clk_enable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

/*
 * clk_disable - inform the system when the clock source is no longer required.
 * @clk: clock source
 *
 * Inform the system that a clock source is no longer required by
 * a driver and may be shut down.
 *
 * Implementation detail: if the clock source is shared between
 * multiple drivers, clk_enable() calls must be balanced by the
 * same number of clk_disable() calls for the clock source to be
 * disabled.
 */
void clk_disable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	do_clk_disable(clk);
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

/**
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source.
 *		 This is only valid once the clock source has been enabled.
 * @clk: clock source
 */
unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long flags, rate;

	spin_lock_irqsave(&clocks_lock, flags);
	rate = clk->rate;
	spin_unlock_irqrestore(&clocks_lock, flags);

	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

/**
 * clk_set_parent - set the parent clock source for this clock
 * @clk: clock source
 * @parent: parent clock source
 *
 * Returns success (0) or negative errno. clk usage_count must be zero
 * before calling this function.
 */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int i, found = 0, val = 0;
	unsigned long flags;

	if (!clk || !parent)
		return -EFAULT;
	if (clk->usage_count)
		return -EPERM;
	if (clk->pclk == parent)
		return 0;
	if (!clk->pclk_sel)
		return -EPERM;

	/* check if requested parent is in clk parent list */
	for (i = 0; i < clk->pclk_sel->pclk_count; i++) {
		if (clk->pclk_sel->pclk_info[i].pclk == parent) {
			found = 1;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	spin_lock_irqsave(&clocks_lock, flags);
	/* reflect parent change in hardware */
	val = readl(clk->pclk_sel->pclk_sel_reg);
	val &= ~(clk->pclk_sel->pclk_sel_mask << clk->pclk_sel_shift);
	val |= clk->pclk_sel->pclk_info[i].pclk_val << clk->pclk_sel_shift;
	writel(val, clk->pclk_sel->pclk_sel_reg);
	spin_unlock_irqrestore(&clocks_lock, flags);

	/* reflect parent change in software */
	clk_reparent(clk, &clk->pclk_sel->pclk_info[i]);

	return 0;
}
EXPORT_SYMBOL(clk_set_parent);

/**
 * clk_set_rate - set the clock rate for a clock source
 * @clk: clock source
 * @rate: desired clock rate in Hz
 *
 * Returns success (0) or negative errno.
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	/* TODO */
	return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

/* registers clock in platform clock framework */
void clk_register(struct clk_lookup *cl)
{
	struct clk *clk;
	unsigned long flags;

	if (!cl || !cl->clk)
		return;
	clk = cl->clk;

	spin_lock_irqsave(&clocks_lock, flags);

	INIT_LIST_HEAD(&clk->children);
	if (clk->flags & ALWAYS_ENABLED)
		clk->ops = NULL;
	else if (!clk->ops)
		clk->ops = &generic_clkops;

	/* root clock don't have any parents */
	if (!clk->pclk && !clk->pclk_sel) {
		list_add(&clk->sibling, &root_clks);
	} else if (clk->pclk && !clk->pclk_sel) {
		/* add clocks with only one parent to parent's children list */
		list_add(&clk->sibling, &clk->pclk->children);
	} else {
		/* clocks with more than one parent */
		struct pclk_info *pclk_info;

		pclk_info = pclk_info_get(clk);
		if (!pclk_info) {
			pr_err("CLKDEV: invalid pclk info of clk with"
					" %s dev_id and %s con_id\n",
					cl->dev_id, cl->con_id);
		} else {
			clk->pclk = pclk_info->pclk;
			list_add(&clk->sibling, &pclk_info->pclk->children);
		}
	}

	spin_unlock_irqrestore(&clocks_lock, flags);

	/* debugfs specific */
#ifdef CONFIG_DEBUG_FS
	list_add(&clk->node, &clocks);
	clk->cl = cl;
#endif

	/* add clock to arm clockdev framework */
	clkdev_add(cl);
}

/**
 * propagate_rate - recalculate and propagate all clocks to children
 *
 * Recalculates all children clocks
 */
void propagate_rate(struct clk *pclk)
{
	struct clk *clk, *_temp;

	list_for_each_entry_safe(clk, _temp, &pclk->children, sibling) {
		/* recalc and propogate only if clk is enabled */
		if (clk_is_enabled(clk)) {
			if (clk->recalc)
				clk->recalc(clk);
			propagate_rate(clk);
		}
	}
}

/*All recalc functions are called with lock held */

/*
 * calculates current programmed rate of pll1
 *
 * In normal mode
 * rate = (2 * M[15:8] * Fin)/(N * 2^P)
 *
 * In Dithered mode
 * rate = (2 * M[15:0] * Fin)/(256 * N * 2^P)
 */
void pll_clk_recalc(struct clk *clk)
{
	struct pll_clk_config *config = clk->private_data;
	unsigned int num = 2, den = 0, val, mode = 0;

	mode = (readl(config->mode_reg) >> config->masks->mode_shift) &
		config->masks->mode_mask;

	val = readl(config->cfg_reg);
	/* calculate denominator */
	den = (val >> config->masks->div_p_shift) & config->masks->div_p_mask;
	den = 1 << den;
	den *= (val >> config->masks->div_n_shift) & config->masks->div_n_mask;

	/* calculate numerator & denominator */
	if (!mode) {
		/* Normal mode */
		num *= (val >> config->masks->norm_fdbk_m_shift) &
			config->masks->norm_fdbk_m_mask;
	} else {
		/* Dithered mode */
		num *= (val >> config->masks->dith_fdbk_m_shift) &
			config->masks->dith_fdbk_m_mask;
		den *= 256;
	}

	BUG_ON(!den);

	clk->rate = (((clk->pclk->rate/10000) * num) / den) * 10000;
}

/* calculates current programmed rate of ahb or apb bus */
void bus_clk_recalc(struct clk *clk)
{
	struct bus_clk_config *config = clk->private_data;
	unsigned int div;

	div = ((readl(config->reg) >> config->masks->shift) &
			config->masks->mask) + 1;

	BUG_ON(!div);

	clk->rate = (unsigned long)clk->pclk->rate / div;
}

/*
 * calculates current programmed rate of auxiliary synthesizers
 * used by: UART, FIRDA
 *
 * Fout from synthesizer can be given from two equations:
 * Fout1 = (Fin * X/Y)/2
 * Fout2 = Fin * X/Y
 *
 * Selection of eqn 1 or 2 is programmed in register
 */
void aux_clk_recalc(struct clk *clk)
{
	struct aux_clk_config *config = clk->private_data;
	unsigned int num = 1, den = 1, val, eqn;

	val = readl(config->synth_reg);

	eqn = (val >> config->masks->eq_sel_shift) &
		config->masks->eq_sel_mask;
	if (eqn == config->masks->eq1_mask)
		den *= 2;

	/* calculate numerator */
	num = (val >> config->masks->xscale_sel_shift) &
		config->masks->xscale_sel_mask;

	/* calculate denominator */
	den *= (val >> config->masks->yscale_sel_shift) &
		config->masks->yscale_sel_mask;

	BUG_ON(!den);

	clk->rate = (((clk->pclk->rate/10000) * num) / den) * 10000;
}

/*
 * calculates current programmed rate of gpt synthesizers
 * Fout from synthesizer can be given from below equations:
 * Fout= Fin/((2 ^ (N+1)) * (M+1))
 */
void gpt_clk_recalc(struct clk *clk)
{
	struct gpt_clk_config *config = clk->private_data;
	unsigned int div = 1, val;

	val = readl(config->synth_reg);
	div += (val >> config->masks->mscale_sel_shift) &
		config->masks->mscale_sel_mask;
	div *= 1 << (((val >> config->masks->nscale_sel_shift) &
				config->masks->nscale_sel_mask) + 1);

	BUG_ON(!div);

	clk->rate = (unsigned long)clk->pclk->rate / div;
}

/*
 * calculates current programmed rate of clcd synthesizer
 * Fout from synthesizer can be given from below equation:
 * Fout= Fin/2*div (division factor)
 * div is 17 bits:-
 *	0-13 (fractional part)
 *	14-16 (integer part)
 * To calculate Fout we left shift val by 14 bits and divide Fin by
 * complete div (including fractional part) and then right shift the
 * result by 14 places.
 */
void clcd_clk_recalc(struct clk *clk)
{
	struct clcd_clk_config *config = clk->private_data;
	unsigned int div = 1;
	unsigned long prate;
	unsigned int val;

	val = readl(config->synth_reg);
	div = (val >> config->masks->div_factor_shift) &
		config->masks->div_factor_mask;

	prate = clk->pclk->rate / 1000; /* first level division, make it KHz */

	BUG_ON(!div);

	clk->rate = (((unsigned long)prate << 12) / (2 * div)) >> 12;
	clk->rate *= 1000;
}

/*
 * Used for clocks that always have value as the parent clock divided by a
 * fixed divisor
 */
void follow_parent(struct clk *clk)
{
	unsigned int div_factor = (clk->div_factor < 1) ? 1 : clk->div_factor;

	clk->rate = clk->pclk->rate/div_factor;
}

/**
 * recalc_root_clocks - recalculate and propagate all root clocks
 *
 * Recalculates all root clocks (clocks with no parent), which if the
 * clock's .recalc is set correctly, should also propagate their rates.
 */
void recalc_root_clocks(void)
{
	struct clk *pclk;
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	list_for_each_entry(pclk, &root_clks, sibling) {
		/*
		 * It doesn't make a sense to recalc and propogate rate
		 * if clk is not enabled in hw.
		 */
		if (clk_is_enabled(pclk)) {
			if (pclk->recalc)
				pclk->recalc(pclk);
			propagate_rate(pclk);
		}
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
}

#ifdef CONFIG_DEBUG_FS
/*
 *	debugfs support to trace clock tree hierarchy and attributes
 */
static struct dentry *clk_debugfs_root;
static int clk_debugfs_register_one(struct clk *c)
{
	int err;
	struct dentry *d, *child;
	struct clk *pa = c->pclk;
	char s[255];
	char *p = s;

	if (c) {
		if (c->cl->con_id)
			p += sprintf(p, "%s", c->cl->con_id);
		if (c->cl->dev_id)
			p += sprintf(p, "%s", c->cl->dev_id);
	}
	d = debugfs_create_dir(s, pa ? pa->dent : clk_debugfs_root);
	if (!d)
		return -ENOMEM;
	c->dent = d;

	d = debugfs_create_u32("usage_count", S_IRUGO, c->dent,
			(u32 *)&c->usage_count);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	d = debugfs_create_u32("rate", S_IRUGO, c->dent, (u32 *)&c->rate);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	d = debugfs_create_x32("flags", S_IRUGO, c->dent, (u32 *)&c->flags);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}
	return 0;

err_out:
	d = c->dent;
	list_for_each_entry(child, &d->d_subdirs, d_u.d_child)
		debugfs_remove(child);
	debugfs_remove(c->dent);
	return err;
}

static int clk_debugfs_register(struct clk *c)
{
	int err;
	struct clk *pa = c->pclk;

	if (pa && !pa->dent) {
		err = clk_debugfs_register(pa);
		if (err)
			return err;
	}

	if (!c->dent) {
		err = clk_debugfs_register_one(c);
		if (err)
			return err;
	}
	return 0;
}

static int __init clk_debugfs_init(void)
{
	struct clk *c;
	struct dentry *d;
	int err;

	d = debugfs_create_dir("clock", NULL);
	if (!d)
		return -ENOMEM;
	clk_debugfs_root = d;

	list_for_each_entry(c, &clocks, node) {
		err = clk_debugfs_register(c);
		if (err)
			goto err_out;
	}
	return 0;
err_out:
	debugfs_remove_recursive(clk_debugfs_root);
	return err;
}
late_initcall(clk_debugfs_init);

static int clk_debugfs_reparent(struct clk *c)
{
	debugfs_remove(c->dent);
	return clk_debugfs_register_one(c);
}
#endif /* CONFIG_DEBUG_FS */
