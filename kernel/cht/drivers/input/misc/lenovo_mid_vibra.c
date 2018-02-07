/*
 *  intel_mid_vibra.c - Intel Vibrator for Intel CherryTrail platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: B, Jayachandran <jayachandran.b@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */



#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include <linux/input/intel_mid_vibra.h>
#include <linux/mfd/intel_soc_pmic.h>
#include "mid_vibra.h"

struct lenovo_vibra_info {
	int     enabled;
	long time;
	struct mutex	lock;
	struct device	*dev;
	void __iomem	*shim;
	unsigned long *base_unit;
	unsigned long *duty_cycle;
	unsigned int max_base_unit;
	u8  max_duty_cycle;

	void (*enable)(struct lenovo_vibra_info *info);
	void (*disable)(struct lenovo_vibra_info *info);
	int (*pwm_configure)(struct lenovo_vibra_info *info, bool enable);

	const struct attribute_group *vibra_attr_group;
	struct delayed_work dworker;
};


#define LENOVO_VIBRA_MAX_BASEUNIT	0xFF
#define LENOVO_VIBRA_MAX_TIMEDIVISOR	0xFF
#define LENOVO_VIBRA_DEFAULT_TIMEDIVISOR	0x80

/************************register map***********************************/
#define CHRLEDCTRL_REG (0x1f|(0x5e<<8))

#define CHRLEDF(val)	((val&0x3)<<0x4)	//00:1/4Hz, 01:1/2Hz, 10:1Hz, 11:2Hz
#define CHRLEDI(val)	((val&0x3)<<0x2)	//00:10mA, 01:1mA, 10:2.5mA, 11:5mA
#define SWLEDON(val)	((val&0x1)<<0x1) //0:turn off circuit, 1:turn on circuit /(sw control)
#define CHRLEDFN(val)	((val&0x1)<<0x0) //0:hw, 1:sw

#define CHRLEDF_MASK	(0x3<<0x4)
#define CHRLEDI_MASK	(0x3<<0x2)
#define SWLEDON_MASK	(0x1<<0x1)
#define CHRLEDFN_MASK	(0x1<<0x0)

#define CHRLEDFSM_REG	(0x20|(0x5e<<8))

#define LEDEFF(val)		((val&0x3)<<0x1)	//00:1/256 Duty Cycle, 01:256/256 Duty Cycle, 10:Blinking, 11:Breathing
#define CHRLEDCIP(val)	((val&0x1)<<0x0) //1:charging 0:none

#define LEDEFF_MASK		(0x3<<0x1)
#define CHRLEDCIP_MASK	(0x1<<0x0)

#define CHRLEDPWM_REG	(0x21|(0x5e<<8))

#define CHRLEDDUTY(val)	(val&0xff)	//duty cycle 00:1/256 ff:256:256


/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/
static unsigned long mid_vibra_base_unit;
static unsigned long mid_vibra_duty_cycle = LENOVO_VIBRA_DEFAULT_TIMEDIVISOR;

static ssize_t lenovo_vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lenovo_vibra_info *info = dev_get_drvdata(dev);
	int reg1, reg2, reg3;
	reg1 = intel_soc_pmic_readb(CHRLEDCTRL_REG);
	reg2 = intel_soc_pmic_readb(CHRLEDFSM_REG);
	reg3 = intel_soc_pmic_readb(CHRLEDPWM_REG);

	return sprintf(buf, "%d,%d,%d,%d\n", info->enabled, reg1, reg2, reg3);

}

static ssize_t lenovo_vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct lenovo_vibra_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	//if (vibrator_enable == info->enabled)
	//	return len;
	else if (vibrator_enable <= 0)
		info->disable(info);
	else{
		info->time = vibrator_enable;
		info->enable(info);
	}

	return len;
}

static DEVICE_ATTR(vibrator, S_IRUGO | S_IWUSR,
		   lenovo_vibra_show_vibrator, lenovo_vibra_set_vibrator);
static DEVICE_ULONG_ATTR(pwm_baseunit, S_IRUGO | S_IWUSR,
				 mid_vibra_base_unit);
static DEVICE_ULONG_ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR,
				 mid_vibra_duty_cycle);

static struct attribute *vibra_attrs[] = {
	&dev_attr_vibrator.attr,
	&dev_attr_pwm_baseunit.attr.attr,
	&dev_attr_pwm_ontime_div.attr.attr,
	0,
};

static const struct attribute_group vibra_attr_group = {
	.attrs = vibra_attrs,
};

/*** Module ***/
#if CONFIG_PM
static int lenovo_vibra_runtime_suspend(struct device *dev)
{
	struct lenovo_vibra_info *info = dev_get_drvdata(dev);

	pr_info("In %s\n", __func__);
	info->pwm_configure(info, false);
	return 0;
}

static int lenovo_vibra_runtime_resume(struct device *dev)
{
	pr_info("In %s\n", __func__);
	return 0;
}

static void lenovo_vibra_complete(struct device *dev)
{
	pr_info("In %s\n", __func__);
	lenovo_vibra_runtime_resume(dev);
}

static const struct dev_pm_ops lenovo_mid_vibra_pm_ops = {
	.prepare = lenovo_vibra_runtime_suspend,
	.complete = lenovo_vibra_complete,
	.runtime_suspend = lenovo_vibra_runtime_suspend,
	.runtime_resume = lenovo_vibra_runtime_resume,
};
#endif

static int lenovo_vibra_pmic_pwm_configure(struct lenovo_vibra_info *info, bool enable)
{
	u8 clk_div;
	u8 duty_cyc;

	if (enable) {
		if (mid_vibra_duty_cycle < 0 || mid_vibra_duty_cycle > LENOVO_VIBRA_MAX_TIMEDIVISOR)
			duty_cyc = LENOVO_VIBRA_DEFAULT_TIMEDIVISOR;
		else
			duty_cyc = mid_vibra_duty_cycle;

		intel_soc_pmic_writeb(CHRLEDPWM_REG, CHRLEDDUTY(duty_cyc));
		intel_soc_pmic_writeb(CHRLEDFSM_REG, LEDEFF(1));
		intel_soc_pmic_writeb(CHRLEDCTRL_REG, CHRLEDF(1)|CHRLEDI(1)|SWLEDON(1)|CHRLEDFN(1));
#if 0
		/* disable PWM before updating clock div*/
		intel_soc_pmic_writeb(CRYSTALCOVE_PMIC_PWM1_CLKDIV_REG, 0);

		/* validate the input values */
		if (*info->base_unit > info->max_base_unit) {
			*info->base_unit = info->max_base_unit;
			pr_err("%s:base_unit i/p is greater than max using max",
								__func__);
		}
		if (*info->duty_cycle > info->max_duty_cycle) {
			*info->duty_cycle = info->max_duty_cycle;
			pr_err("%s:duty_cycle i/p greater than max", __func__);
		}

		clk_div = *info->base_unit;
		duty_cyc = *info->duty_cycle;

		clk_div = clk_div | CRYSTALCOVE_PMIC_PWM_ENABLE;
		intel_soc_pmic_writeb(CRYSTALCOVE_PMIC_PWM1_DUTYCYC_REG,
						duty_cyc);
		intel_soc_pmic_writeb(CRYSTALCOVE_PMIC_PWM1_CLKDIV_REG,
						clk_div);
#endif
	} else {

		intel_soc_pmic_writeb(CHRLEDCTRL_REG, CHRLEDF(1)|CHRLEDI(0)|SWLEDON(0)|CHRLEDFN(1));
#if 0
		/*disable PWM block */
		clk_div =  intel_soc_pmic_readb(
					CRYSTALCOVE_PMIC_PWM1_CLKDIV_REG);
		intel_soc_pmic_writeb(CRYSTALCOVE_PMIC_PWM1_CLKDIV_REG,
				      (clk_div & ~CRYSTALCOVE_PMIC_PWM_ENABLE));
#endif
	}
//	clk_div =  intel_soc_pmic_readb(CRYSTALCOVE_PMIC_PWM1_CLKDIV_REG);
//	duty_cyc =  intel_soc_pmic_readb(CRYSTALCOVE_PMIC_PWM1_DUTYCYC_REG);
	pr_info("%s: clk_div_reg = %#x, duty_cycle_reg = %#x\n",
						__func__, clk_div, duty_cyc);
	return 0;
}

static void lenovo_vibra_disable(struct lenovo_vibra_info *info)
{
	pr_err("%s: Disable", __func__);
	mutex_lock(&info->lock);
	cancel_delayed_work_sync(&info->dworker);
	info->enabled = false;
	info->pwm_configure(info, false);
	pm_runtime_put(info->dev);
	mutex_unlock(&info->lock);
}

static void lenovo_vibra_drv_enable(struct lenovo_vibra_info *info)
{
	pr_info("%s: Enable", __func__);
	mutex_lock(&info->lock);
	cancel_delayed_work_sync(&info->dworker);
	pm_runtime_get_sync(info->dev);
	info->pwm_configure(info, true);
	info->enabled = true;
	schedule_delayed_work(&info->dworker,msecs_to_jiffies(info->time));
	mutex_unlock(&info->lock);
}

struct lenovo_vibra_info *lenovo_mid_vibra_setup(struct device *dev,
				 struct mid_vibra_pdata *data)
{
	struct lenovo_vibra_info *info;
	pr_info("probe data div %x, base %x",
							data->time_divisor,
							data->base_unit);

	info =  devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: no memory for driver context", __func__);
		return NULL;
	}

	info->dev = dev;
	mutex_init(&info->lock);
	info->vibra_attr_group = &vibra_attr_group;
	mid_vibra_base_unit = data->base_unit;
	mid_vibra_duty_cycle = data->time_divisor;
	info->base_unit = &mid_vibra_base_unit;
	info->duty_cycle = &mid_vibra_duty_cycle;
	info->enable = lenovo_vibra_drv_enable;
	info->disable = lenovo_vibra_disable;

	return info;
}

static void lenovo_mid_vibra_work(struct work_struct *work)
{
	struct lenovo_vibra_info * info = container_of(work,struct lenovo_vibra_info,dworker.work);

	info->enabled = false;
	info->pwm_configure(info, false);
	pm_runtime_put(info->dev);
}

static int lenovo_mid_plat_vibra_probe(struct platform_device *pdev)
{
	struct lenovo_vibra_info *info;
	struct device *dev = &pdev->dev;
	struct mid_vibra_pdata *data;
	int ret;

	data = (struct mid_vibra_pdata *)dev_get_platdata(dev);
	if (!data) {
		pr_err("Invalid driver data\n");
		return -ENODEV;
	}

	info = lenovo_mid_vibra_setup(dev, data);
	if (!info)
		return -ENODEV;

	info->pwm_configure = lenovo_vibra_pmic_pwm_configure;
	info->max_base_unit = LENOVO_VIBRA_MAX_BASEUNIT;
	info->max_duty_cycle = LENOVO_VIBRA_MAX_TIMEDIVISOR;

	ret = sysfs_create_group(&dev->kobj, info->vibra_attr_group);
	if (ret) {
		pr_err("could not register sysfs files\n");
		return ret;
	}

	INIT_DELAYED_WORK(&info->dworker, lenovo_mid_vibra_work);

	platform_set_drvdata(pdev, info);

	pr_info("%s: vibra probe success\n", __func__);
	return ret;
}

static int lenovo_mid_plat_vibra_remove(struct platform_device *pdev)
{
	struct lenovo_vibra_info *info = platform_get_drvdata(pdev);
	sysfs_remove_group(&info->dev->kobj, info->vibra_attr_group);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver plat_vibra_driver = {
	.driver = {
		.name = "lenovo_mid_pmic_vibra",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &lenovo_mid_vibra_pm_ops,
#endif
	},
	.probe = lenovo_mid_plat_vibra_probe,
	.remove = lenovo_mid_plat_vibra_remove,
};

static struct mid_vibra_pdata plat_vibra_data = {
	.time_divisor = 50,
	.base_unit = 100,

};

static struct platform_device plat_vibra_device = {
	.name		= "lenovo_mid_pmic_vibra",
	.id		= 0,
	.dev		= {
		.platform_data	= &plat_vibra_data,
	},
};

/**
* lenovo_mid_vibra_init - Module init function
*
* Registers platform
* Init all data strutures
*/
static int __init lenovo_mid_vibra_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&plat_vibra_driver);
	if (ret)
		pr_err("Platform driver register failed\n");

	ret = platform_device_register(&plat_vibra_device);
	if(ret){
		pr_err("Platform device register failed (%d)\n", ret);
		return ret;
	}

	return ret;
}

/**
* lenovo_mid_vibra_exit - Module exit function
*
* Unregisters platform
* Frees all data strutures
*/
static void __exit lenovo_mid_vibra_exit(void)
{
	platform_driver_unregister(&plat_vibra_driver);
	platform_device_unregister(&plat_vibra_device);
	pr_debug("lenovo_mid_vibra driver exited\n");
	return;
}

late_initcall(lenovo_mid_vibra_init);
module_exit(lenovo_mid_vibra_exit);

MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("lenovo <lenovo@lenovo>");
