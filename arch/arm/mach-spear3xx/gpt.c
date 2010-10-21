/*
 * arch/arm/mach-spear3xx/gpt.c
 *
 * General Purpose Timer Framework for SPEAr3xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Deepak Sikri<deepak.sikri@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/gpt.h>
#include <mach/spear.h>

static struct spear_timer spear3xx_gp_timers[] = {
	{
		.id = 1,
		.phys_base = (SPEAR3XX_ICM3_TMR0_BASE),
		.irq = SPEAR3XX_IRQ_BASIC_GPT1_1
	}, {
		.id = 2,
		.phys_base = (SPEAR3XX_ICM3_TMR0_BASE + 0x80),
		.irq = SPEAR3XX_IRQ_BASIC_GPT1_2
	}, {
		.id = 3,
		.phys_base = (SPEAR3XX_ICM3_TMR1_BASE),
		.irq = SPEAR3XX_IRQ_BASIC_GPT2_1
	}, {
		.id = 4,
		.phys_base = (SPEAR3XX_ICM3_TMR1_BASE + 0x80),
		.irq = SPEAR3XX_IRQ_BASIC_GPT2_2
	},
};

static const char *gpt_parent_clk_name[] __initdata = {
	"pll3_48m_clk",
	NULL
};

static struct clk *gpt_parent_clocks[2];
static struct spear_timer *gpt;
static const int timer_count = ARRAY_SIZE(spear3xx_gp_timers);
static spinlock_t timer_lock;

/*
 * static helper functions
 */
static inline u16 gpt_read_reg(struct spear_timer *timer, u32 reg)
{
	return readw(timer->io_base + (reg & 0xff));
}

static void gpt_write_reg(struct spear_timer *timer, u32 reg,
						u16 value)
{
	writew(value, timer->io_base + (reg & 0xff));
}

static void gpt_reset(struct spear_timer *timer)
{
	gpt_write_reg(timer, GPT_CTRL_OFF, 0x0);
	gpt_write_reg(timer, GPT_LOAD_OFF, 0x0);
	gpt_write_reg(timer, GPT_INT_OFF, GPT_STATUS_MATCH);
}

static int gpt_enable(struct spear_timer *timer)
{
	if (!timer)
		return -ENODEV;

	if (timer->enabled)
		return 0;

	clk_enable(timer->clk);
	timer->enabled = 1;

	return 0;
}

static int gpt_disable(struct spear_timer *timer)
{
	if (!timer)
		return -ENODEV;

	if (!timer->enabled)
		return -EINVAL;

	clk_disable(timer->clk);

	timer->enabled = 0;

	return 0;
}

static void gpt_prepare(struct spear_timer *timer)
{
	gpt_enable(timer);
	gpt_reset(timer);
}

/* Functions exported for application */
/*
 * gpt_request - request for a timer
 *
 * This function returns a timer which is available, returns NULL if none is
 * available
 */
static struct spear_timer *gpt_request(void)
{
	struct spear_timer *timer = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&timer_lock, flags);
	for (i = 0; i < timer_count; i++) {
		if (gpt[i].reserved)
			continue;

		timer = &gpt[i];
		timer->reserved = 1;
		break;
	}
	spin_unlock_irqrestore(&timer_lock, flags);

	if (timer != NULL)
		gpt_prepare(timer);

	return timer;
}

/*
 * Applications should free the timer when not in use. The function
 * disables the timer clock.
 */
static int gpt_free(struct spear_timer *timer)
{
	if (!timer)
		return -ENODEV;

	gpt_disable(timer);
	timer->reserved = 0;

	return 0;
}

/*
 * Return the functional clock of the timer at which it is operating.
 * Functional clock represents the actual clock at which the timer is
 * operating. The count period depends on this clock
 */
static struct clk *gpt_get_clk(struct spear_timer *timer)
{
	if (!timer)
		return NULL;

	return timer->clk;
}

/* The timer shall start operating only after calling this function */
static int gpt_start(struct spear_timer *timer)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_CTRL_OFF);
	if (!(val & GPT_CTRL_ENABLE)) {
		val |= GPT_CTRL_ENABLE;
		gpt_write_reg(timer, GPT_CTRL_OFF, val);
	}

	return 0;
}

/* The function can be used to stop the timer operation */
static int gpt_stop(struct spear_timer *timer)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_CTRL_OFF);
	if (val & GPT_CTRL_ENABLE) {
		val &= ~GPT_CTRL_ENABLE;
		gpt_write_reg(timer, GPT_CTRL_OFF, val);
	}

	return 0;
}

/*
 * The Source specifies the clock which has to be selected as the functional
 * clock of timer. The current framework sets it as 48 MHz (PLL3) output.
 */
static int gpt_set_source(struct spear_timer *timer, int source)
{
	if (!timer)
		return -ENODEV;

	clk_disable(timer->clk);
	clk_set_parent(timer->clk, gpt_parent_clocks[source]);
	clk_enable(timer->clk);

	/*
	 * When the functional clock disappears, too quick writes seem to
	 * cause an abort.
	 */
	mdelay(1);

	return 0;
}

/*
 * The timer will be programmed as per the value in load. The time period will
 * depend on the clock and prescaler selected. The autoreload parameter
 * decides, whether the count will be reloaded
 */
static int gpt_set_load_start(struct spear_timer *timer, int autoreload,
		u16 load)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_CTRL_OFF);
	if (autoreload)
		val &= ~GPT_CTRL_MODE_AR;
	else
		val |= GPT_CTRL_MODE_AR;

	val |= GPT_CTRL_ENABLE;

	gpt_write_reg(timer, GPT_LOAD_OFF, load);
	gpt_write_reg(timer, GPT_CTRL_OFF, val);

	return 0;
}

/*
 * If application wants to enable the interrupt on count match then the
 * function should be called with enable set to 1.
 */
static int gpt_set_match(struct spear_timer *timer, int enable)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_CTRL_OFF);
	if (enable)
		val |= GPT_CTRL_MATCH_INT;
	else
		val &= ~GPT_CTRL_MATCH_INT;
	gpt_write_reg(timer, GPT_CTRL_OFF, val);

	return 0;
}

/*
 * This function can be used to program the prescaler for the timer. Its value
 * can be from 1 to 8 signifying division by 2^prescaler
 */
static int gpt_set_prescaler(struct spear_timer *timer, int prescaler)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_CTRL_OFF);
	if (prescaler >= 0x00 && prescaler <= 0x08) {
		val &= 0xFFF0;
		val |= prescaler;
	} else {
		printk(KERN_ERR "Invalid prescaler\n");
	}

	gpt_write_reg(timer, GPT_CTRL_OFF, val);
	return 0;
}

/*
 * This function can be used to read the interrupt status of timer.
 * Currently only match interrupt, at offset 0, is part of this.
 */
static int gpt_read_status(struct spear_timer *timer)
{
	u16 val;

	if (!timer)
		return -ENODEV;

	val = gpt_read_reg(timer, GPT_INT_OFF);

	return val;
}

/*
 * The function can be used to clear the match interrupt status by writing
 * a value 0.
 */
static int gpt_write_status(struct spear_timer *timer, u16 value)
{
	if (!timer)
		return -ENODEV;

	gpt_write_reg(timer, GPT_INT_OFF, value);
	return 0;
}

struct spear_timer_ops timer_ops = {
	.timer_set_source = gpt_set_source,
	.timer_start = gpt_start,
	.timer_stop = gpt_stop,
	.timer_get_clk = gpt_get_clk,
	.timer_read_status = gpt_read_status,
	.timer_write_status = gpt_write_status,
	.timer_set_load_start = gpt_set_load_start,
	.timer_set_prescalar = gpt_set_prescaler,
	.timer_set_match = gpt_set_match,
	.timer_request = gpt_request,
	.timer_free = gpt_free,
};

static int __init spear_gpt_init(void)
{
	struct spear_timer *timer;
	int i;
	int mtu; /* timer unit */
	char clk_name[16];
	char gpt_name[16];

	spin_lock_init(&timer_lock);
	gpt = spear3xx_gp_timers;

	for (i = 0; gpt_parent_clk_name[i] != NULL; i++)
		gpt_parent_clocks[i] = clk_get(NULL, gpt_parent_clk_name[i]);

	for (i = 0; i < timer_count; i++) {
		timer = &gpt[i];
		/* each mtu has 2 timers with same set of clocks */
		mtu = i >> 1;
		sprintf(gpt_name, "gpt%d_%d", mtu + 1, i);

		if (!request_mem_region(timer->phys_base, 0x80, gpt_name)) {
			pr_err("%s:cannot get IO addr\n", __func__);
			return -EBUSY;
		}

		timer->io_base = (void __iomem *)
				ioremap(timer->phys_base, 0x80);
		if (!timer->io_base) {
			pr_err("%s:ioremap failed for gpt\n", __func__);
			iounmap(timer->io_base);
			return -EBUSY;
		}

		sprintf(clk_name, "gpt%d", mtu + 1);
		timer->clk = clk_get_sys(clk_name, NULL);
	}

	return 0;
}

arch_initcall(spear_gpt_init);
