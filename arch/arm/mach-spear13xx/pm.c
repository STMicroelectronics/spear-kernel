/*
 * arch/arm/mach-spear13xx/pm.c
 *
 * SPEAr13xx Power Management source file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Deepak Sikri <deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>
#include <mach/generic.h>
#include <mach/suspend.h>

static void (*spear_sram_sleep)(suspend_state_t state, unsigned long *saveblk);
static int pcm_set_cfg;

static void memcpy_decr_ptr(void *dest, void *src, u32 len)
{
	int i;

	for (i = 0; i < len ; i++)
		*((u32 *)(dest - (i<<2))) = *((u32 *)(src + (i<<2)));
}

static int spear_pm_on(void)
{
	cpu_do_idle();

	return 0;
}

/*
 * This function call is made post the CPU suspend is done.
 */
static int spear_sys_suspend(unsigned long arg)
{
	suspend_state_t state = (suspend_state_t)(arg);
	/* Flush the cache */
	flush_cache_all();
	outer_disable();
	outer_sync();
	/* Jump to the suspend routines in sram */
	spear_sram_sleep(state, (unsigned long *)cpu_resume);

	return 0;
}

static int spear_pm_sleep(suspend_state_t state)
{
	/* Move the cpu into suspend */
	cpu_suspend((unsigned long)state, spear_sys_suspend);

	/* Resume operations */
	outer_resume();
	/* Explicit set all the power domain to on */
	writel((readl(VA_PCM_CFG) | pcm_set_cfg),
		VA_PCM_CFG);

	return 0;
}

static int spear_pm_prepare(void)
{
	disable_hlt();

	return 0;
}

static int spear_pm_enter(suspend_state_t state)
{
	u32 pm_cfg = readl(VA_PCM_CFG);

	/*
	 * Set up default config to source gpio interrupt through GIC
	 * This would be valid for standby mode.
	 */
	if (of_machine_is_compatible("st,spear1310"))
		pm_cfg |= PWR_DOM_ON_1310;
	else
		pm_cfg |= PWR_DOM_ON;

	writel((pm_cfg & (~(1 << 2))), VA_PCM_CFG);

	switch (state) {
	case PM_SUSPEND_ON:
		return spear_pm_on();
	case PM_SUSPEND_STANDBY:
		break;
	case PM_SUSPEND_MEM:
		/*
		 * Set ddr_phy_no_shutoff to 0 in order to select
		 * the SPEAr DDR pads, DDRIO_VDD1V8_1V5_OFF
		 * and DDRIO_VDD1V2_OFF, to be used to control
		 * the lines for the switching of the DDRPHY to the
		 * external power supply.
		 */
		if (of_machine_is_compatible("st,spear1310"))
			pm_cfg &= (unsigned long)DDR_PHY_NO_SHUTOFF_CFG_1310;
		else
			pm_cfg &= (unsigned long)DDR_PHY_NO_SHUTOFF_CFG;
		/*
		 * Set up the Power Domains specific registers.
		 * 1. Setup the wake up enable of the desired sources.
		 * 2. Set the wake up trigger field to zero
		 * 3. Clear config_ack and config_bad
		 * 4. Set sw_config equal to ack_power_state
		 * The currrent S2R operations enable all the wake up
		 * sources by default.
		 */
		writel(pm_cfg | pcm_set_cfg, VA_PCM_CFG);
		/* Set up the desired wake up state */
		pm_cfg = readl(VA_PCM_WKUP_CFG);
		/* Set the states for all power island on */
		writel(pm_cfg | PCM_SET_WAKEUP_CFG, VA_PCM_WKUP_CFG);
		/* Set the VA_SWITCH_CTR to Max Restart Current */
		writel(SWITCH_CTR_CFG, VA_SWITCH_CTR);
		break;
	default:
		return -EINVAL;
	}

	return spear_pm_sleep(state);
}

/*
 *	spear_pm_finish - Finish up suspend sequence.
 *
 *	This is called after we wake back up (or if entering the sleep state
 *	failed).
 */
static void spear_pm_finish(void)
{
	enable_hlt();
}

/*
 *	spear_pm_valid_state- check the valid states in PM for the SPEAr
 *	platform
 */
static int spear_pm_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static struct platform_suspend_ops spear_pm_ops = {
	.prepare	= spear_pm_prepare,
	.enter		= spear_pm_enter,
	.finish		= spear_pm_finish,
	.valid		= spear_pm_valid_state,
};

static int __init spear_pm_init(void)
{
	void *sram_dest = (void *)SRAM_START_VA;
	void *sram_limit_va = (void *)SRAM_LIMIT_VA;
	void *spear_sleep_fn_addr;
	int spear_sleep_mode_sz;
	void __iomem *mpmc_regs_base;
	struct clk *aor_clk, *sysram0_clk;

	//pcm_set_cfg = GPIO_WKUP | RTC_WKUP | ETH_WKUP | USB_WKUP |
	pcm_set_cfg = GPIO_WKUP | RTC_WKUP | ETH_WKUP |
		PWR_DOM_ON;

	if (of_machine_is_compatible("st,spear1340")) {
		spear_sleep_mode_sz = spear1340_sleep_mode_sz;
		spear_sleep_fn_addr = (void *)spear1340_sleep_mode;
	} else if (of_machine_is_compatible("st,spear1310")) {
		spear_sleep_fn_addr = (void *)spear1310_sleep_mode;
		spear_sleep_mode_sz = spear1310_sleep_mode_sz;
		pcm_set_cfg &= ~(PWR_DOM_ON | USB_WKUP);
		pcm_set_cfg |= (PWR_DOM_ON_1310 | RAS_WKUP);
	} else
		return -ENODEV;

	/* In case the suspend code size is more than sram size return */
	if ((spear_sleep_mode_sz + (MPMC_REG_CNT << 2) +
				spear_wakeup_sz) > (sram_limit_va - sram_dest))
		return	-ENOMEM;

	sysram0_clk = clk_get(NULL, "sysram0_clk");
	if (IS_ERR(sysram0_clk)) {
		pr_err("%s: Error getting System RAM-0 clock\n", __func__);
		return PTR_ERR(sysram0_clk);
	}
	clk_prepare_enable(sysram0_clk);

	aor_clk = clk_get(NULL, "sysram1_clk");
	if (IS_ERR(aor_clk)) {
		pr_err("%s: Error getting AOR clock\n", __func__);
		clk_disable_unprepare(sysram0_clk);
		return PTR_ERR(aor_clk);
	}
	clk_prepare_enable(aor_clk);

	memcpy(sram_dest, (void *)spear_wakeup, spear_wakeup_sz);
	/* Increment destination pointer by the size copied*/
	sram_dest += roundup(spear_wakeup_sz, 4);
	/* Copy the Sleep code on to the SRAM after the wake code */
	spear_sram_sleep = memcpy(sram_dest, (void *)spear_sleep_fn_addr,
			spear_sleep_mode_sz);
	/* Copy in the MPMC registers at the end of SRAM */
	mpmc_regs_base = ioremap(A9SM_AND_MPMC_BASE, 1024);
	memcpy_decr_ptr(sram_limit_va , mpmc_regs_base, MPMC_REG_CNT);
	iounmap(mpmc_regs_base);
	/* Setup the pm ops */
	suspend_set_ops(&spear_pm_ops);

	return 0;
}
arch_initcall(spear_pm_init);
