#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include <linux/mfd/intel_soc_pmic.h>
#include "timed_output.h"


#define TIMED_VIBRATOR_NAME "vibrator"

#define TIMED_VIBRATOR_DEFAULT_DUTY	0xf0
#define TIMED_VIBRATOR_MAX_DUTY	0xff
#define TIMED_VIBRATOR_MAX_TIMEOUT	60000

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

struct timed_vibrator_platform_data {
	const char *name;
	int 	max_timeout;
};

struct timed_vibrator_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	//spinlock_t lock;
	unsigned vibrator;
	int max_timeout;
	u8 *duty;
	struct mutex	lock;
	struct work_struct	work;
};

static unsigned long timed_vibrator_duty = TIMED_VIBRATOR_DEFAULT_DUTY;

static DEVICE_ULONG_ATTR(duty, S_IRUGO | S_IWUSR, timed_vibrator_duty);

static struct attribute *vibra_attrs[] = {
	(struct attribute *)&dev_attr_duty.attr,
	NULL,
};

static const struct attribute_group vibra_attr_group = {
	.attrs = vibra_attrs,
};

static void vibrator_enable(struct timed_output_dev *dev, int value);

static int vibrator_pwm_init(void)
{
	pr_info("%s\n", __func__);

	intel_soc_pmic_writeb(CHRLEDPWM_REG, CHRLEDDUTY(0));
	intel_soc_pmic_writeb(CHRLEDFSM_REG, LEDEFF(1));
	intel_soc_pmic_writeb(CHRLEDCTRL_REG, CHRLEDF(1)|CHRLEDI(0)|SWLEDON(0)|CHRLEDFN(1));

	return 0;
}

static int vibrator_pwm_config(struct timed_vibrator_data *data, bool enable)
{
	u8 duty_cyc = 0;

	if (enable) {
		if (*data->duty < 0 || *data->duty > TIMED_VIBRATOR_MAX_DUTY)
			duty_cyc = TIMED_VIBRATOR_DEFAULT_DUTY;
		else
			duty_cyc = *data->duty;

		intel_soc_pmic_writeb(CHRLEDPWM_REG, CHRLEDDUTY(duty_cyc));
		intel_soc_pmic_writeb(CHRLEDFSM_REG, LEDEFF(1));
		intel_soc_pmic_writeb(CHRLEDCTRL_REG, CHRLEDF(1)|CHRLEDI(3)|SWLEDON(1)|CHRLEDFN(1));
	} else {
		intel_soc_pmic_writeb(CHRLEDCTRL_REG, CHRLEDF(1)|CHRLEDI(0)|SWLEDON(0)|CHRLEDFN(1));
	}

	pr_info("%s: duty_cycle = %#x\n", __func__, duty_cyc);

	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct timed_vibrator_data *data =
		container_of(timer, struct timed_vibrator_data, timer);

	pr_err("%s\n", __FUNCTION__);

	//vibrator_pwm_config(data, 0);// can't access pmic because this maybe lead to sleep, then panic
	schedule_work(&data->work);

	return HRTIMER_NORESTART;
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct timed_vibrator_data	*data =
		container_of(dev, struct timed_vibrator_data, dev);

	if (hrtimer_active(&data->timer)) {
		ktime_t r = hrtimer_get_remaining(&data->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct timed_vibrator_data	*data =
		container_of(dev, struct timed_vibrator_data, dev);

	pr_info("%s, value:%d\n", __func__, value);
	//unsigned long	flags;

	//spin_lock_irqsave(&data->lock, flags);
	mutex_lock(&data->lock);

	/* cancel previous timer and set VIBRATOR according to value */
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work);
	vibrator_pwm_config(data, !!value);

	if (value > 0) {
		if (value > data->max_timeout)
			value = data->max_timeout;

		hrtimer_start(&data->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	//spin_unlock_irqrestore(&data->lock, flags);
	mutex_unlock(&data->lock);

}

static void timed_vibrator_work(struct work_struct *work)
{
	struct timed_vibrator_data *data = container_of(work, struct timed_vibrator_data, work);
	vibrator_pwm_config(data, 0);
}

static int timed_vibrator_probe(struct platform_device *pdev)
{
	struct timed_vibrator_platform_data *pdata = pdev->dev.platform_data;
	struct timed_vibrator_data *vibrator_data;
	struct device *dev = &pdev->dev;
	int ret;

	if (!pdata)
		return -EBUSY;

	vibrator_data = kzalloc(sizeof(struct timed_vibrator_data), GFP_KERNEL);
	if (!vibrator_data)
		return -ENOMEM;

	hrtimer_init(&vibrator_data->timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	vibrator_data->timer.function = vibrator_timer_func;

	//spin_lock_init(&vibrator_data->lock);
	mutex_init(&vibrator_data->lock);

	vibrator_data->dev.name = pdata->name;
	vibrator_data->dev.get_time = vibrator_get_time;
	vibrator_data->dev.enable = vibrator_enable;
	ret = timed_output_dev_register(&vibrator_data->dev);
	if (ret < 0) {
		pr_err("%s, could not register timed output device\n", __FUNCTION__);
		goto err_out1;
	}
	
	vibrator_data->max_timeout = pdata->max_timeout;
	vibrator_data->duty = (u8*)&timed_vibrator_duty;
	vibrator_pwm_init();

	ret = sysfs_create_group(&dev->kobj, &vibra_attr_group);
	if (ret) {
		pr_err("%s, could not register sysfs files\n", __FUNCTION__);
		goto err_out2;
	}

	INIT_WORK(&vibrator_data->work, timed_vibrator_work);

	platform_set_drvdata(pdev, vibrator_data);

	return 0;

err_out2:
	timed_output_dev_unregister(&vibrator_data->dev);
err_out1:
	kfree(vibrator_data);

	return ret;
}

static int timed_vibrator_remove(struct platform_device *pdev)
{
	struct timed_vibrator_data *vibrator_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	vibrator_pwm_config(vibrator_data, 0);
	timed_output_dev_unregister(&vibrator_data->dev);
	sysfs_remove_group(&dev->kobj, &vibra_attr_group);
	kfree(vibrator_data);

	return 0;
}

#if CONFIG_PM
static int timed_vibrator_suspend(struct device *dev)
{
	//struct timed_vibrator_data *vibrator_data = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	//vibrator_pwm_config(vibrator_data, 0);  //maybe can not access pmic at this place !!!!
	return 0;
}

static int timed_vibrator_resume(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void timed_vibrator_complete(struct device *dev)
{
	pr_info("%s\n", __func__);
}

static const struct dev_pm_ops timed_vibrator_pm_ops = {
	.prepare = timed_vibrator_suspend,
	.complete = timed_vibrator_complete,
	.runtime_suspend = timed_vibrator_suspend,
	.runtime_resume = timed_vibrator_resume,
};
#endif

static struct platform_driver timed_vibrator_driver = {
	.probe		= timed_vibrator_probe,
	.remove		= timed_vibrator_remove,
	.driver		= {
		.name		= TIMED_VIBRATOR_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &timed_vibrator_pm_ops,
#endif
	},
};

struct timed_vibrator_platform_data timed_vibrator_pdata = {
	.name = "vibrator",
	.max_timeout = TIMED_VIBRATOR_MAX_TIMEOUT,
};

static struct platform_device timed_vibrator_device = {
	.name	= TIMED_VIBRATOR_NAME,
	.id		= 0,
	.dev		= {
		.platform_data	= &timed_vibrator_pdata,
	},
};

static int __init timed_vibrator_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&timed_vibrator_driver);
	if (ret){
		pr_err("%s, fail:%d\n", __func__, ret);
		return ret;
	}

	ret = platform_device_register(&timed_vibrator_device);
	if(ret){
		pr_err("%s, fail:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static void __exit timed_vibrator_exit(void)
{
	platform_driver_unregister(&timed_vibrator_driver);
	platform_device_unregister(&timed_vibrator_device);
	return;
}

late_initcall(timed_vibrator_init);
module_exit(timed_vibrator_exit);

//module_platform_driver(timed_vibrator_driver);

MODULE_AUTHOR("guoyc1 <guoyc1@lenovo.com>");
MODULE_DESCRIPTION("Timed vibrator driver");
MODULE_LICENSE("GPL");
