/*
 * arch/arm/plat-spear/cpufreq.c
 *
 * CPU Frequency Scaling for SPEAr platform
 *
 * Copyright (C) 2010 ST Microelectronics
 * Deepak Sikri<deepak.sikri@st.com>
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/system.h>

#define CPU_CLK_MAX_STEP	10

#ifdef CONFIG_ARCH_SPEAR13XX
#if defined(CONFIG_CPU_SPEAR1300) || defined(CONFIG_CPU_SPEAR1310_REVA) || \
	defined (CONFIG_CPU_SPEAR900)
#define SPEAR13XX_MIN_CPU_FREQ	200000
#define SPEAR13XX_MAX_CPU_FREQ	500000

static u32 spear13xx_cpu_freq[] = {
	200000, /* 200 MHZ */
	250000, /* 250 MHZ */
	332000, /* 332 MHZ */
	400000, /* 400 MHZ */
	500000, /* 500 MHZ */
};
#endif /* CONFIG_ARCH_SPEAR13XX and !CONFIG_CPU_SPEAR1340 */

#ifdef CONFIG_CPU_SPEAR1340
#define SPEAR1340_MIN_CPU_FREQ	166000
#define SPEAR1340_MAX_CPU_FREQ	600000

static u32 spear1340_cpu_freq[] = {
	166000, /* 166 MHZ */
	200000, /* 200 MHZ */
	250000, /* 250 MHZ */
	332000, /* 332 MHZ */
	400000, /* 400 MHZ */
	500000, /* 500 MHZ */
	600000, /* 600 MHZ */
};

#endif /* CONFIG_CPU_SPEAR1340 */
#endif /* CONFIG_ARCH_SPEAR13XX */

#if defined(CONFIG_ARCH_SPEAR6XX) || defined(CONFIG_ARCH_SPEAR3XX)
#define MIN_CPU_FREQ	166000
#define MAX_CPU_FREQ	332000

static u32 spear_cpu_freq[] = {
	166000, /* 166 MHZ */
	266000, /* 266 MHZ */
	332000, /* 332 MHZ */
};
#endif

static struct cpufreq_frequency_table spear_freq_tbl[CPU_CLK_MAX_STEP];
static struct clk *cpu_clk;

int spear_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, spear_freq_tbl);
}

static unsigned int spear_cpufreq_get(unsigned int cpu)
{
	return cpu ? 0 : clk_get_rate(cpu_clk) / 1000;
}

static struct clk *spear1340_cpu_get_possible_parent(unsigned long newfreq)
{
	int pclk;
	struct clk *sys_pclk;
	/*
	 * In SPEAr1340, cpu clk's parent sys clk can take input from
	 * following sources
	 */
	const char *sys_clk_src[] = {
		"sys_synth_clk",
		"pll1_clk",
		"pll2_clk",
		"pll3_clk",
	};

	/*
	 * As sys clk can have multiple source with their own range
	 * limitation so we choose possible sources accordingly
	 */
	if (newfreq <= 250000000)
		pclk = 0; /* src is sys_synth_clk */
	else if (newfreq <= 600000000)
		pclk = 2; /* src is pll2_clk */
	else
		return ERR_PTR(-EINVAL);

	/* Get parent to sys clock */
	sys_pclk = clk_get(NULL, sys_clk_src[pclk]);
	if (IS_ERR(sys_pclk))
		pr_err("SPEAr1340: Failed to get %s clock\n",
				sys_clk_src[pclk]);

	return sys_pclk;
}

static int spear1340_set_cpu_rate(struct clk *cpu_clk, struct clk *sys_pclk,
		unsigned long newfreq)
{
	struct clk *sys_clk;
	int ret = 0;

	sys_clk = clk_get_parent(cpu_clk);
	if (IS_ERR(sys_clk)) {
		pr_err("failed to get cpu's parent (sys) clock\n");
		return PTR_ERR(sys_clk);
	}

	/*
	 * Set the rate of the source clock before changing the parent
	 * Note: newfreq = intended cpu_clk * 2 in case of SPEAr1340
	 */
	ret = clk_set_rate(sys_pclk, newfreq);
	if (ret) {
		pr_err("SPEAr1340: Failed to set sys clk rate to %lu\n",
				newfreq);
		return ret;
	}

	ret = clk_set_parent(sys_clk, sys_pclk);
	if (ret) {
		pr_err("SPEAr1340: Failed to set sys clk parent\n");
		return ret;
	}

	return 0;
}

static bool slow_mode_required(struct clk *clk)
{
	struct clk *sys_pclk;

	if (cpu_is_spear1340()) {
		sys_pclk = clk_get(NULL, "sys_synth_clk");
		if (IS_ERR(sys_pclk))
			WARN(1, "couldn't get system synthesizer clk");
		else
			clk_put(sys_pclk);
		/*
		 * slow mode not required if cpu is on synth.
		 * Also to be on safe side let system change to slow
		 * mode if sys_pclk has error
		 */
		return (clk == sys_pclk) ? false: true;
	} else if (arch_is_spear13xx()) {
		return true;
	} else {
		/*
		 * case of spear3xx/6xx is separatly handled as we need
		 * to put ddr into self refresh before changing pll rate
		 */
		return false;
	}
}

static int spear_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int index, ret, slow_mode;
	unsigned long newfreq, srcfreq;
	struct clk *srcclk;

	if (policy->cpu != 0)
		return -EINVAL;

	if (cpufreq_frequency_table_target(policy, spear_freq_tbl,
				target_freq, relation, &index))
		return -EINVAL;

	freqs.old = spear_cpufreq_get(0);
	freqs.cpu = policy->cpu;

	if (freqs.old == target_freq)
		return 0;

	newfreq = spear_freq_tbl[index].frequency * 1000;
	if (cpu_is_spear1340()) {
		/*
		 * SPEAr1340 is special in the sense that due to the
		 * possibility of multiple clock sources for cpu clk's
		 * parent we can have different clock source for
		 * different frequency of cpu clk.
		 * Hence we need to choose one from amongst these
		 * possible clock sources.
		 */
		srcclk = spear1340_cpu_get_possible_parent(newfreq);
		if (IS_ERR(srcclk)) {
			pr_err("Failed to get src clk\n");
			return PTR_ERR(srcclk);
		}

		/* SPEAr1340: src clk is always 2 * intended cpu clk */
		srcfreq = newfreq * 2;
	} else {
		/*
		 * Rest: src clock to be altered is ancestor of cpu
		 * clock. Hence we can directly work on cpu clk
		 */
		srcclk = cpu_clk;
		srcfreq = newfreq;
	}

	/*
	 * In SPEAr1340, we cannot use newfreq directly because we need
	 * to actually access a source clock (clk) which might not be
	 * ancestor of cpu at present.
	 * Hence in SPEAr1340 we would operate on source clock directly
	 * before switching cpu clock to it.
	 */
	srcfreq = clk_round_rate(srcclk, srcfreq);
	if (srcfreq < 0) {
		pr_err("CPU Freq: clk_round_rate failed for cpu src clock\n");
		return srcfreq;
	}

	freqs.new = srcfreq / 1000;
	freqs.new /= cpu_is_spear1340() ? 2 : 1;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	slow_mode = slow_mode_required(srcclk);
	if (slow_mode) {
		ret = arch_change_mode(SYS_MODE_SLOW);
		if (ret) {
			pr_err("couldn't cange system to slow mode\n");
			return ret;
		}
	}

	if (cpu_is_spear1340())
		ret = spear1340_set_cpu_rate(cpu_clk, srcclk, srcfreq);
	else
		ret = clk_set_rate(cpu_clk, srcfreq);

	/* Get current rate after clk_set_rate, in case of failure */
	if (ret) {
		pr_err("CPU Freq: cpu clk_set_rate failed: %d\n", ret);
		freqs.new = clk_get_rate(cpu_clk) / 1000;
	}

	/* Now switch back to normal mode */
	if (slow_mode) {
		ret = arch_change_mode(SYS_MODE_NORMAL);
		if (ret) {
			pr_err("Couldnot change back to normal mode\n");
			BUG();
		}
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	return ret;
}

static int spear_cpufreq_init(struct cpufreq_policy *policy)
{
	int i = 0, table_len = 0;
	u32 *cpu_freq_table;

	if (policy->cpu != 0)
		return -EINVAL;

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		policy->cpuinfo.min_freq = SPEAR1340_MIN_CPU_FREQ;
		policy->cpuinfo.max_freq = SPEAR1340_MAX_CPU_FREQ;
		cpu_freq_table = spear1340_cpu_freq;
		table_len = ARRAY_SIZE(spear1340_cpu_freq);
#endif
	} else if (arch_is_spear13xx()) {
#ifdef CONFIG_ARCH_SPEAR13XX
		policy->cpuinfo.min_freq = SPEAR13XX_MIN_CPU_FREQ;
		policy->cpuinfo.max_freq = SPEAR13XX_MAX_CPU_FREQ;
		cpu_freq_table = spear13xx_cpu_freq;
		table_len = ARRAY_SIZE(spear13xx_cpu_freq);
#endif
	} else if (arch_is_spear3xx() || arch_is_spear6xx()) {
#if defined(CONFIG_ARCH_SPEAR3XX) || defined(CONFIG_ARCH_SPEAR6XX)
		policy->cpuinfo.min_freq = MIN_CPU_FREQ;
		policy->cpuinfo.max_freq = MAX_CPU_FREQ;
		cpu_freq_table = spear_cpu_freq;
		table_len = ARRAY_SIZE(spear_cpu_freq);
#endif
	} else {
		pr_err("Error: No valid cpu found\n");
		return -EINVAL;
	}

	policy->cur = policy->min = policy->max = spear_cpufreq_get(0);

	for (i = 0; i < table_len; i++) {
		spear_freq_tbl[i].index = i;
		spear_freq_tbl[i].frequency = cpu_freq_table[i];
	}

	spear_freq_tbl[i].index = i;
	spear_freq_tbl[i].frequency = CPUFREQ_TABLE_END;
	if (!cpufreq_frequency_table_cpuinfo(policy, spear_freq_tbl))
		cpufreq_frequency_table_get_attr(spear_freq_tbl,
				policy->cpu);

	policy->cpuinfo.transition_latency = 300*1000; /*300 us*/

	return 0;
}

static int spear_cpufreq_exit(struct cpufreq_policy *policy)
{
	clk_put(cpu_clk);
	return 0;
}

static struct freq_attr *spear_cpufreq_attr[] = {
	 &cpufreq_freq_attr_scaling_available_freqs,
	 NULL,
};

static struct cpufreq_driver spear_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= spear_cpufreq_verify,
	.target		= spear_cpufreq_target,
	.get		= spear_cpufreq_get,
	.init		= spear_cpufreq_init,
	.exit		= spear_cpufreq_exit,
	.name		= "spear_cpufreq",
	.attr		= spear_cpufreq_attr,
};

static int __init spear_cpufreq_register(void)
{
	return cpufreq_register_driver(&spear_driver);
}

arch_initcall(spear_cpufreq_register);
