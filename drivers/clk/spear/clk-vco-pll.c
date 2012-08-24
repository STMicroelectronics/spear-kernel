/*
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * VCO-PLL clock implementation
 */

#define pr_fmt(fmt) "clk-vco-pll: " fmt

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include "clk.h"

/*
 * DOC: VCO-PLL clock
 *
 * VCO and PLL rate are derived from following equations:
 *
 * In normal mode
 * vco = (2 * M[15:8] * Fin)/N
 *
 * In Dithered mode
 * vco = (2 * M[15:0] * Fin)/(256 * N)
 *
 * pll_rate = pll/2^p
 *
 * vco and pll are very closely bound to each other, "vco needs to program:
 * mode, m & n" and "pll needs to program p", both share common enable/disable
 * logic.
 *
 * clk_register_vco_pll() registers instances of both vco & pll.
 * CLK_SET_RATE_PARENT flag is forced for pll, as it will always pass its
 * set_rate to vco. A single rate table exists for both the clocks, which
 * configures m, n and p.
 */

/* PLL_CTR register masks */
#define PLL_MODE_NORMAL		0
#define PLL_MODE_FRACTION	1
#define PLL_MODE_DITH_DSM	2
#define PLL_MODE_DITH_SSM	3
#define PLL_MODE_MASK		3
#define PLL_MODE_SHIFT		3
#define PLL_ENABLE		2

#define PLL_LOCK_SHIFT		0
#define PLL_LOCK_MASK		1

/* PLL FRQ register masks */
#define PLL_NORM_FDBK_M_MASK	0xFF
#define PLL_NORM_FDBK_M_SHIFT	24
#define PLL_DITH_FDBK_M_MASK	0xFFFF
#define PLL_DITH_FDBK_M_SHIFT	16
#define PLL_DIV_P_MASK		0x7
#define PLL_DIV_P_SHIFT		8
#define PLL_DIV_N_MASK		0xFF
#define PLL_DIV_N_SHIFT		0

#define to_clk_vco(_hw) container_of(_hw, struct clk_vco, hw)
#define to_clk_pll(_hw) container_of(_hw, struct clk_pll, hw)
static inline unsigned long vco_calc_rate(struct clk_hw *hw,
		unsigned long parent_rate, int index);

static inline unsigned long pll_calc_rate(struct clk_hw *hw,
		unsigned long parent_rate, int index)
{
	struct clk_pll *pll = to_clk_pll(hw);

	return (parent_rate / (1 << pll->rtbl[index].p));
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long *parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct clk_vco *vco = pll->vco;
	unsigned long prate = *parent_rate;
	unsigned long rate = 0;
	unsigned long vco_parent_rate;
	int i, unused;
	struct {
		unsigned long desired_prate;
		unsigned long rate;
	} *clk_table;

	rate = clk_round_rate_index(hw, drate, prate, pll_calc_rate,
			pll->rtbl_cnt, &unused);

	if (rate == drate)
		return rate;

	vco_parent_rate =
		__clk_get_rate(__clk_get_parent(__clk_get_parent(hw->clk)));

	clk_table = kzalloc(sizeof(*clk_table) * vco->rtbl_cnt, GFP_KERNEL);
	if (!clk_table) {
		pr_err("not able to allocate clk table\n");
		return -ENOMEM;
	}

	/* iterate over each vco output to see if it can provide the
	 * required pll clk
	 */
	for (i = 0; i < vco->rtbl_cnt; i++) {
		prate = vco_calc_rate(&vco->hw, vco_parent_rate, i);
		rate = clk_round_rate_index(hw, drate, prate,
				pll_calc_rate, pll->rtbl_cnt, &unused);
		if (rate == drate)
			break;

		/* store the outcomes to be parsed later for best clock */
		clk_table[i].desired_prate = prate;
		clk_table[i].rate = rate;
	}

	if (rate == drate) {
		*parent_rate = prate;
		goto out;
	}

	rate = 0;
	for (i = 0; i < vco->rtbl_cnt; i++) {
		if ((drate - clk_table[i].rate) < (drate - rate)) {
			rate = clk_table[i].rate;
			*parent_rate = clk_table[i].desired_prate;
		}
	}

out:
	kfree(clk_table);
	return rate;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw, unsigned long
		parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	unsigned long flags = 0;
	unsigned int p;

	if (pll->vco->lock)
		spin_lock_irqsave(pll->vco->lock, flags);

	p = readl_relaxed(pll->vco->cfg_reg);

	if (pll->vco->lock)
		spin_unlock_irqrestore(pll->vco->lock, flags);

	p = (p >> PLL_DIV_P_SHIFT) & PLL_DIV_P_MASK;

	return parent_rate / (1 << p);
}

static int clk_pll_set_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct pll_rate_tbl *rtbl = pll->rtbl;
	unsigned long flags = 0, val;
	int i = 0;

	clk_round_rate_index(hw, drate, parent_rate, pll_calc_rate,
			pll->rtbl_cnt, &i);

	if (pll->vco->lock)
		spin_lock_irqsave(pll->vco->lock, flags);

	val = readl_relaxed(pll->vco->cfg_reg);
	val &= ~(PLL_DIV_P_MASK << PLL_DIV_P_SHIFT);
	val |= (rtbl[i].p & PLL_DIV_P_MASK) << PLL_DIV_P_SHIFT;
	writel_relaxed(val, pll->vco->cfg_reg);

	if (pll->vco->lock)
		spin_unlock_irqrestore(pll->vco->lock, flags);

	return 0;
}

static struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pll_recalc_rate,
	.round_rate = clk_pll_round_rate,
	.set_rate = clk_pll_set_rate,
};

static inline unsigned long vco_calc_rate(struct clk_hw *hw,
		unsigned long parent_rate, int index)
{
	struct clk_vco *vco = to_clk_vco(hw);
	unsigned long rate;
	unsigned int mode;

	mode = vco->rtbl[index].mode ? 256 : 1;
	rate = (((2 * parent_rate / 10000) * vco->rtbl[index].m) /
			(mode * vco->rtbl[index].n));

	return rate * 10000;
}

static long clk_vco_round_rate(struct clk_hw *hw, unsigned long drate,
		unsigned long *prate)
{
	struct clk_vco *vco = to_clk_vco(hw);
	int unused;

	return clk_round_rate_index(hw, drate, *prate, vco_calc_rate,
			vco->rtbl_cnt, &unused);
}

static unsigned long clk_vco_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_vco *vco = to_clk_vco(hw);
	unsigned long flags = 0;
	unsigned int num = 2, den = 0, val, mode = 0;

	if (vco->lock)
		spin_lock_irqsave(vco->lock, flags);

	mode = (readl_relaxed(vco->mode_reg) >> PLL_MODE_SHIFT) & PLL_MODE_MASK;

	val = readl_relaxed(vco->cfg_reg);

	if (vco->lock)
		spin_unlock_irqrestore(vco->lock, flags);

	den = (val >> PLL_DIV_N_SHIFT) & PLL_DIV_N_MASK;

	/* calculate numerator & denominator */
	if (!mode) {
		/* Normal mode */
		num *= (val >> PLL_NORM_FDBK_M_SHIFT) & PLL_NORM_FDBK_M_MASK;
	} else {
		/* Dithered mode */
		num *= (val >> PLL_DITH_FDBK_M_SHIFT) & PLL_DITH_FDBK_M_MASK;
		den *= 256;
	}

	if (!den) {
		WARN(1, "%s: denominator can't be zero\n", __func__);
		return 0;
	}

	return (((parent_rate / 10000) * num) / den) * 10000;
}

/* Configures new clock rate of vco */
static int clk_vco_set_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long prate)
{
	struct clk_vco *vco = to_clk_vco(hw);
	struct vco_rate_tbl *rtbl = vco->rtbl;
	unsigned long flags = 0, val;
	int i;

	clk_round_rate_index(hw, drate, prate, vco_calc_rate,
			vco->rtbl_cnt, &i);
	if (vco->lock)
		spin_lock_irqsave(vco->lock, flags);

	val = readl_relaxed(vco->mode_reg);
	val &= ~(PLL_MODE_MASK << PLL_MODE_SHIFT);
	val |= (rtbl[i].mode & PLL_MODE_MASK) << PLL_MODE_SHIFT;
	writel_relaxed(val, vco->mode_reg);

	val = readl_relaxed(vco->cfg_reg);
	val &= ~(PLL_DIV_N_MASK << PLL_DIV_N_SHIFT);
	val |= (rtbl[i].n & PLL_DIV_N_MASK) << PLL_DIV_N_SHIFT;

	val &= ~(PLL_DITH_FDBK_M_MASK << PLL_DITH_FDBK_M_SHIFT);
	if (rtbl[i].mode)
		val |= (rtbl[i].m & PLL_DITH_FDBK_M_MASK) <<
			PLL_DITH_FDBK_M_SHIFT;
	else
		val |= (rtbl[i].m & PLL_NORM_FDBK_M_MASK) <<
			PLL_NORM_FDBK_M_SHIFT;

	writel_relaxed(val, vco->cfg_reg);

	if (vco->lock)
		spin_unlock_irqrestore(vco->lock, flags);

	return 0;
}

static struct clk_ops clk_vco_ops = {
	.recalc_rate = clk_vco_recalc_rate,
	.round_rate = clk_vco_round_rate,
	.set_rate = clk_vco_set_rate,
};

struct clk *clk_register_vco_pll(const char *vco_name, const char *pll_name,
		const char *vco_gate_name, const char *parent_name,
		unsigned long flags, void __iomem *mode_reg, void __iomem
		*cfg_reg, struct vco_rate_tbl *vco_rtbl, u8 vco_rtbl_cnt,
		struct pll_rate_tbl *pll_rtbl, u8 pll_rtbl_cnt,
		spinlock_t *lock, struct clk **pll_clk,
		struct clk **vco_gate_clk)
{
	struct clk_vco *vco;
	struct clk_pll *pll;
	struct clk *vco_clk, *tpll_clk, *tvco_gate_clk;
	struct clk_init_data vco_init, pll_init;
	const char **vco_parent_name;

	if (!vco_name || !pll_name || !parent_name || !mode_reg || !cfg_reg ||
			!vco_rtbl || !vco_rtbl_cnt ||
			!pll_rtbl || !pll_rtbl_cnt) {
		pr_err("Invalid arguments passed");
		return ERR_PTR(-EINVAL);
	}

	vco = kzalloc(sizeof(*vco), GFP_KERNEL);
	if (!vco) {
		pr_err("could not allocate vco clk\n");
		return ERR_PTR(-ENOMEM);
	}

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll) {
		pr_err("could not allocate pll clk\n");
		goto free_vco;
	}

	/* struct clk_vco assignments */
	vco->mode_reg = mode_reg;
	vco->cfg_reg = cfg_reg;
	vco->rtbl = vco_rtbl;
	vco->rtbl_cnt = vco_rtbl_cnt;
	vco->lock = lock;
	vco->hw.init = &vco_init;

	pll->vco = vco;
	pll->rtbl = pll_rtbl;
	pll->rtbl_cnt = pll_rtbl_cnt;
	pll->hw.init = &pll_init;

	if (vco_gate_name) {
		tvco_gate_clk = clk_register_gate(NULL, vco_gate_name,
				parent_name, 0, mode_reg, PLL_ENABLE, 0, lock);
		if (IS_ERR_OR_NULL(tvco_gate_clk))
			goto free_pll;

		if (vco_gate_clk)
			*vco_gate_clk = tvco_gate_clk;
		vco_parent_name = &vco_gate_name;
	} else {
		vco_parent_name = &parent_name;
	}

	vco_init.name = vco_name;
	vco_init.ops = &clk_vco_ops;
	vco_init.flags = flags;
	vco_init.parent_names = vco_parent_name;
	vco_init.num_parents = 1;

	pll_init.name = pll_name;
	pll_init.ops = &clk_pll_ops;
	pll_init.flags = CLK_SET_RATE_PARENT;
	pll_init.parent_names = &vco_name;
	pll_init.num_parents = 1;

	vco_clk = clk_register(NULL, &vco->hw);
	if (IS_ERR_OR_NULL(vco_clk))
		goto free_pll;

	tpll_clk = clk_register(NULL, &pll->hw);
	if (IS_ERR_OR_NULL(tpll_clk))
		goto free_pll;

	if (pll_clk)
		*pll_clk = tpll_clk;

	return vco_clk;

free_pll:
	kfree(pll);
free_vco:
	kfree(vco);

	pr_err("Failed to register vco pll clock\n");

	return ERR_PTR(-ENOMEM);
}
