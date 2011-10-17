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

#define CPU_CLK		"cpu_clk"

#ifdef CONFIG_ARCH_SPEAR13XX
#define MIN_CPU_FREQ	200000
#define MAX_CPU_FREQ	500000

static u32 spear_cpu_freq[] = {
	200000, /* 200 MHZ */
	250000, /* 250 MHZ */
	332000, /* 332 MHZ */
	400000, /* 400 MHZ */
	500000, /* 500 MHZ */
};
#elif defined(CONFIG_ARCH_SPEAR6XX) || defined(CONFIG_ARCH_SPEAR3XX)
#define MIN_CPU_FREQ	166000
#define MAX_CPU_FREQ	332000

static u32 spear_cpu_freq[] = {
	166000, /* 166 MHZ */
	266000, /* 266 MHZ */
	332000, /* 333 MHZ */
};
#endif

static struct
	cpufreq_frequency_table spear_freq_tbl[ARRAY_SIZE(spear_cpu_freq) + 1];
static struct clk *cpu_clk;

int spear_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, spear_freq_tbl);
}

static unsigned int spear_cpufreq_get(unsigned int cpu)
{
	return cpu ? 0 : clk_get_rate(cpu_clk) / 1000;
}

static int spear_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int index, ret;
	long newfreq;

	if (policy->cpu != 0)
		return -EINVAL;

	if (cpufreq_frequency_table_target(policy, spear_freq_tbl,
				target_freq, relation, &index))
		return -EINVAL;

	freqs.old = spear_cpufreq_get(0);
	freqs.cpu = policy->cpu;

	if (freqs.old == target_freq)
		return 0;

	newfreq = clk_round_rate(cpu_clk, spear_cpu_freq[index] * 1000);
	if (newfreq < 0) {
		pr_err("CPU Freq: clk_round_rate failed: %ld\n", newfreq);
		return newfreq;
	}

	freqs.new = newfreq / 1000;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/* Get current rate after clk_set_rate, for both success and failure */
	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("CPU Freq: cpu clk_set_rate failed: %d\n", ret);
		freqs.new = clk_get_rate(cpu_clk) / 1000;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	return ret;
}

static int spear_cpufreq_init(struct cpufreq_policy *policy)
{
	int i = 0;

	if (policy->cpu != 0)
		return -EINVAL;

	cpu_clk = clk_get(NULL, CPU_CLK);
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	policy->cpuinfo.min_freq = MIN_CPU_FREQ;
	policy->cpuinfo.max_freq = MAX_CPU_FREQ;
	policy->cur = policy->min = policy->max = spear_cpufreq_get(0);

	for (i = 0; i < ARRAY_SIZE(spear_cpu_freq); i++) {
		spear_freq_tbl[i].index = i;
		spear_freq_tbl[i].frequency = spear_cpu_freq[i];
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
