/*
 * arch/arm/mach-spear13xx/include/mach/generic.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2012 ST Microelectronics
 * Viresh Kumar <viresh.linux@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <asm/mach/time.h>
#include <sound/designware_i2s.h>

/* Add spear13xx structure declarations here */
extern struct sys_timer spear13xx_timer;
extern struct adc_plat_data adc_pdata;
extern struct pl022_ssp_controller pl022_plat_data;
extern struct dw_dma_platform_data dmac_plat_data0;
extern struct dw_dma_platform_data dmac_plat_data1;
extern struct dw_dma_slave cf_dma_priv;
extern struct dw_dma_slave nand_read_dma_priv;
extern struct dw_dma_slave nand_write_dma_priv;
extern struct db9000fb_mach_info clcd_plat_info;

/* Add spear13xx family function declarations here */
void __init spear_setup_of_timer(void);
void __init spear13xx_map_io(void);
void __init spear13xx_dt_init_irq(void);
void __init spear13xx_l2x0_init(void);
int audio_clk_config(struct i2s_clk_config_data *config);
void i2s_clk_init(void);
bool dw_dma_filter(struct dma_chan *chan, void *slave);
int spear13xx_eth_phy_clk_cfg(struct platform_device *pdev);
void spear_restart(char, const char *);
void spear13xx_secondary_startup(void);
void spear13xx_reserve_mem(void);

#ifdef CONFIG_MACH_SPEAR1310
void __init spear1310_clk_init(void);
int spear1310_otg_phy_init(void);
#else
static inline void spear1310_clk_init(void) {}
#endif

#ifdef CONFIG_MACH_SPEAR1340
void __init spear1340_clk_init(void);
int spear1340_otg_phy_init(void);
int __init spear1340_sys_clk_init(void);
#else
static inline void spear1340_clk_init(void) {}
static inline int spear1340_sys_clk_init(void) {}
#endif

#endif /* __MACH_GENERIC_H */
