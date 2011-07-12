/* 
 * Android SPEAr battery driver.
 * Based on goldfish_battery.c
 * Author: Vincenzo Frascino <vincenzo.frascino@st.com>
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <plat/adc.h>
#include <linux/gpio.h>
#include <mach/pm_common.h>
#include <linux/spear_adc_usr.h>
#include <linux/spear_adc.h>

#define CURRENT_CHANNEL	0
#define VOLTAGE_CHANNEL	1

#define VOLTAGE_MAX	2100
#define VOLTAGE_MIN	1500

static int device = 0;

static struct adc_chan_config cur_cfg;
static struct adc_chan_config vol_cfg;

static int spear_battery_bat_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val);

static int spear_battery_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val);

static struct platform_device *spear_battery_pdev;

static enum power_supply_property spear_battery_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property spear_battery_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply spear_battery_bat = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties 	=  spear_battery_battery_props,
	.num_properties = ARRAY_SIZE(spear_battery_battery_props),
	.get_property	= spear_battery_bat_get_property,
	.use_for_apm 	= 1,
};

static struct power_supply spear_battery_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties 	=  spear_battery_ac_props,
	.num_properties = ARRAY_SIZE(spear_battery_ac_props),
	.get_property	= spear_battery_ac_get_property,
};

static struct power_supply spear_battery_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties 	=  spear_battery_ac_props,
	.num_properties = ARRAY_SIZE(spear_battery_ac_props),
	.get_property	= spear_battery_ac_get_property,
};


static int spear_battery_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int spear_battery_bat_get_charge_property(int channel) {

	unsigned int returnValue = 0;
	
	if (!channel) {
		spear_adc_get_data(&device,  ADC_CHANNEL0, &returnValue, 1);
	} else {
		spear_adc_get_data(&device,  ADC_CHANNEL1, &returnValue, 1);
	}

	return returnValue;
	
}	

static int spear_battery_bat_get_status_property(void) {

	int st1 = 0;
	int st2 = 0;
	int swd = 0;
	
	/* Verify if BATTERY_ST1_GPIO and BATTERY_ST2_GPIO are correctly set */
        if (gpio_is_valid(BATTERY_ST1_GPIO) && gpio_is_valid(BATTERY_ST2_GPIO)  && gpio_is_valid(BATTERY_SWC_GPIO)) {
                /* Get the BATTERY_ST1_GPIO and BATTERY_ST2_GPIO values */
                st1 = gpio_get_value(BATTERY_ST1_GPIO);
		st2 = gpio_get_value(BATTERY_ST2_GPIO);
		swd = gpio_get_value(BATTERY_SWC_GPIO);
        }else
             	printk(KERN_WARNING "SPEAr Power Management: PM Invalid BATTERY_ST1_GPIO and BATTERY_ST2_GPIO values");
	
	/* Determine Battery Status */
	switch (st1) {
		case 0:
			if (st2 == 0)
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				if (swd == 0)
					return POWER_SUPPLY_STATUS_FULL;
				else
					return POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case 1:
			if (st2 == 0)
				return POWER_SUPPLY_STATUS_CHARGING;
			else
				return POWER_SUPPLY_STATUS_UNKNOWN;
		default:
			break;
	}
	
	return -1;
}

static int spear_battery_bat_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = spear_battery_bat_get_status_property();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = ((spear_battery_bat_get_charge_property(VOLTAGE_CHANNEL) - VOLTAGE_MIN) * 100) / (VOLTAGE_MAX - VOLTAGE_MIN);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = 2400;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = spear_battery_bat_get_charge_property(CURRENT_CHANNEL);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 20;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = 2100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = spear_battery_bat_get_charge_property(VOLTAGE_CHANNEL);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = 1500;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int __init spear_battery_init(void)
{
	int ret = 0;
	int adc_err = 0;
	
	spear_adc_chan_get(&device, ADC_CHANNEL0);
	spear_adc_chan_get(&device, ADC_CHANNEL1);
	
	/* Configure ADC for x coordinate*/
	cur_cfg.chan_id	= ADC_CHANNEL0;
	cur_cfg.avg_samples	= SAMPLE64;
	cur_cfg.scan_rate	= 5000; /* micro second*/
	cur_cfg.scan_rate_fixed = false;/* dma only available for chan-0 */
	adc_err = spear_adc_chan_configure(&device, &cur_cfg);
	if (adc_err < 0) {
		printk("Err in ADC config of x coordinate=%d\n", adc_err);
		return -1;
	}

	/* Configure ADC for y coordinate*/
	vol_cfg.chan_id	= ADC_CHANNEL1;
	vol_cfg.avg_samples	= SAMPLE64;
	vol_cfg.scan_rate	= 5000; /* micro second*/
	vol_cfg.scan_rate_fixed = false;/* dma is only available for chan-0 */

	adc_err = spear_adc_chan_configure(&device, &vol_cfg);
	if (adc_err < 0) {
		printk("Err in ADC config of y coordinate=%d\n", adc_err);
		return -1;
	}
	
	spear_battery_pdev = platform_device_register_simple("battery", 0, NULL, 0);
	if (IS_ERR(spear_battery_pdev))
		return PTR_ERR(spear_battery_pdev);
	ret = power_supply_register(&spear_battery_pdev->dev, &spear_battery_bat);
	if (ret)
		goto bat_failed;
	ret = power_supply_register(&spear_battery_pdev->dev, &spear_battery_ac);
	if (ret)
		goto ac_failed;
	ret = power_supply_register(&spear_battery_pdev->dev, &spear_battery_usb);
	if (ret)
		goto usb_failed;
	
	printk(KERN_INFO "spear_battery: android SPEAr battery driver loaded\n");
	goto success;

bat_failed:
	power_supply_unregister(&spear_battery_bat);
ac_failed:
	power_supply_unregister(&spear_battery_ac);
usb_failed:
	power_supply_unregister(&spear_battery_usb);
	platform_device_unregister(spear_battery_pdev);
success:
	return ret;
}

static void __exit spear_battery_exit(void)
{
	
	power_supply_unregister(&spear_battery_bat);
	power_supply_unregister(&spear_battery_ac);
	power_supply_unregister(&spear_battery_usb);
	platform_device_unregister(spear_battery_pdev);
	
	spear_adc_chan_put(&device, ADC_CHANNEL0);
	spear_adc_chan_put(&device, ADC_CHANNEL1);
	
	printk(KERN_INFO "spear_battery: android SPEAr battery driver unloaded\n");
}

MODULE_AUTHOR("Vincenzo Frascino <vincenzo.frascino@st.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPEAr battery driver for Linux and Android battery service");
module_init(spear_battery_init);
module_exit(spear_battery_exit);
