/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/version.h>
#include <linux/usb/otg.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/completion.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/extcon.h>
#include <linux/power/bq2589x_reg.h>
#include <asm/intel_em_config.h>
#include "power_supply_charger.h"
/* liza1 0429 */
#include "power_supply.h"

#define CHARGER_TASK_JIFFIES	(HZ * 75)    /* 35sec */
#define CHARGER_HOST_JIFFIES	(HZ * 75)     /* 35sec */
#define FULL_THREAD_JIFFIES		(HZ * 30)     /* 30sec */
#define TEMP_THREAD_JIFFIES		(HZ * 30)     /* 30sec */

#define VBUS_DET_TIMEOUT msecs_to_jiffies(50) /* 50 msec */
#define BATTERY_NEAR_FULL(a)			((a * 98)/100)
#define dev_dbg  dev_err   //liulc1 add
#define dev_warn  dev_err
#define dev_info  dev_err

struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
	int    i2c_adp_num;
	int    main_charger_flag; 
	enum   bq2589x_part_no part_no;
	int    revision;

	struct mutex event_lock;
	struct completion vbus_detect;
	struct wake_lock wakelock;    	/* prevent platform from going to S3 when charging */
	
	struct delayed_work chrg_task_wrkr;
	struct delayed_work chrg_boost_wrkr;
	struct delayed_work chrg_full_wrkr;
	struct work_struct irq_work;

	struct power_supply usb;
	struct usb_phy *transceiver;

	enum power_supply_charger_cable_type cable_type;
	int cc;
	int cv;
	int inlmt;
	int max_cc;
	int max_cv;
	int max_temp;
	int min_temp;
	int iterm;
	int batt_status;
	int bat_health;
	int cntl_state;
	int cntl_state_max;
	int irq;
	bool is_charger_enabled;
	bool is_charging_enabled;
	bool a_bus_enable;
	bool is_pwr_good;
	bool boost_mode;
	bool online;
	bool present;

	struct bq2589x_platform_data * platform_data;
};
//========================================================
 float T[181]={
-10,    -9.5,   -9,     -8.5,   -8,     -7.5,   -7,     -6.5,   -6,     -5.5,   -5,     -4.5,   -4,     -3.5,   -3,     -2.5,   -2,     -1.5,   -1,     -0.5,   0,      0.5,    1,      1.5,    2,      2.5,    3,      3.5,    4,      4.5,    5,      5.5,    6,      6.5,    7,      7.5,    8,      8.5,    9,      9.5,    10,     10.5,   11,     11.5,   12,     12.5,   13,     13.5,   14,     14.5,   15,     15.5,   16,     16.5,   17,     17.5,   18,     18.5,   19,     19.5,   20,     20.5,   21,     21.5,   22,     22.5,   23,     23.5,   24,     24.5,   25,     25.5,   26,     26.5,   27,     27.5,   28,     28.5,   29,     29.5,   30,     30.5,   31,     31.5,   32,     32.5,   33,     33.5,   34,     34.5,   35,     35.5,   36,     36.5,   37,     37.5,   38,     38.5,   39,     39.5,   40,     40.5,   41,     41.5,   42,     42.5,   43,     43.5,   44,     44.5,   45,     45.5,   46,     46.5,   47,     47.5,   48,     48.5,   49,     49.5,   50,     50.5,   51,     51.5,   52,     52.5,   53,     53.5,   54,     54.5,   55,     55.5,   56,     56.5,   57,     57.5,   58,     58.5,   59,     59.5,   60,     60.5,   61,     61.5,   62,     62.5,   63,     63.5,   64,     64.5,   65,     65.5,   66,     66.5,   67,     67.5,   68,     68.5,   69,     69.5,   70,     70.5,   71,     71.5,   72,     72.5,   73,     73.5,   74,     74.5,   75,     75.5,   76,     76.5,   77,     77.5,   78,     78.5,   79,     79.5,   80 };

float R[181]={
271.7,  264.4,  257.1,  250.2,  243.2,  236.1,  230.2,  224.1,  218.1,  212.3,  206.5,  200.5,  195.6,  190.5,  185.4,  180.1,  175.8,  171.1,  166.8,  162.3,  158.2,  154.2,  150.2,  146.4,  142.6,  139.1,  135.4,  132.1,  128.6,  125.4,  122.3,  119.1,  116.2,  113.3,  110.5,  107.5,  105.1,  102.5,  100.1,  97.6,   95.2,   92.5,   90.7,   88.5,   86.3,   84.1,   82.2,   80.1,   78.4,   76.5,   74.7,   72.9,   71.2,   69.9,   68.1,   66.2,   64.8,   63.5,   61.9,   60.5,   59.1,   57.7,   56.4,   55.2,   53.9,   52.7,   51.5,   50.3,   49.2,   48.1,   47.1,   46.1,   45.1,   44.1,   43.1,   42.1,   41.1,   40.2,   39.3,   38.5,   37.6,   36.9,   36.1,   35.3,   34.5,   33.8,   33.1,   32.4,   31.7,   31.1,   30.3,   29.7,   29.1,   28.6,   27.9,   27.3,   26.7,   26.2,   25.7,   25.1,   24.6,   24.1,   23.6,   23.1,   22.6,   22.1,   21.7,   21.3,   20.9,   20.5,   20.1,   19.7,   19.3,   18.9,   18.5,   18.1,   17.8,   17.5,   17.1,   16.7,   16.4,   16.1,   15.8,   15.5,   15.2,   14.9,   14.6,   14.3,   14.1,   13.8,   13.5,   13.3,   13.1,   12.8,   12.5,   12.3,   12.1,   11.9,   11.6,   11.3,   11.1,   10.9,   10.8,   10.6,   10.4,   10.2,   10.1,   9.9,    9.7,    9.5,    9.3,    9.2,    9.1,    8.9,    8.7,    8.6,    8.4,    8.2,    8.1,    8,      7.9,    7.7,    7.5,    7.4,    7.3,    7.2,    7.1,    6.9,    6.7,    6.6,    6.5,    6.4,    6.3,    6.2,    6.1,    6,      5.9,    5.8,    5.7,    5.6,    5.5 };
//========================================================
int boost_flag=0;
int sch_flag=0;
int bf=0;
int cnt=0;
int charger_type=0;
static struct bq2589x *g_bq=NULL;
static struct bq2589x *g_bq_sec=NULL;

static struct power_supply *fg_psy_main;
static struct power_supply *fg_psy_sec;

extern struct i2c_adapter *wcove_pmic_i2c_adapter;

static enum power_supply_property bq2589x_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_ENABLE_CHARGING,
	POWER_SUPPLY_PROP_ENABLE_CHARGER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
	POWER_SUPPLY_PROP_CABLE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_MAX_TEMP,
	POWER_SUPPLY_PROP_MIN_TEMP
};
//liulc1 add
extern int is_bat_cf(int m);
extern void reset_fg(int m);
extern int intel_soc_pmic_readb(int reg);
extern int intel_soc_pmic_writeb(int reg, u8 val);
int bq2589x_enter_ship_mode(void);
int bq2589x_set_vindpm(struct bq2589x *bq, int volt);
static int bq2589x_disable_chr(struct bq2589x *bq);
void  disable_pumpx(void);
void enable_pumpx(void);
void  boost_fun(void);
void config_bq25892(void);
extern int read_rsoc(int m);
extern int read_bat_ichg(int m);
extern int read_bat(int m);
extern int read_battery_temp(int h);
int bq2589x_adc_read_temperature_sec(void);
int bq2589x_adc_read_temperature(void);
int bq2589x_adc_read_battery_volt(void);
int bq2589x_adc_read_battery_volt_sec(void);
int bq2589x_adc_read_charge_current_sec(void);
int bq2589x_adc_read_charge_current(void);
int bq2589x_adc_read_sys_volt(void);
int bq2589x_adc_read_sys_volt_sec(void);

static inline int bq2589x_set_inlmt(struct bq2589x *bq, int inlmt);
int bq2589x_adc_read_vbus_volt(void);
int bq2589x_pumpx_enable(int enable);
int bq2589x_pumpx_increase_volt(void);
int bq2589x_pumpx_increase_volt_done(void);
int bq2589x_pumpx_decrease_volt(void);
int bq2589x_pumpx_decrease_volt_done(void);
//liulc1 end

int get_charger_type(void)
{
	return charger_type;
}

EXPORT_SYMBOL_GPL(get_charger_type);

static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq2589x *bq = data;

	if (bq->client->adapter == wcove_pmic_i2c_adapter) {
		if ((psy->type == POWER_SUPPLY_TYPE_BATTERY) && (!strcmp(psy->name, "bq27x00_main-1"))) {
			fg_psy_main = psy;
			dev_info(bq->dev, "get battery fg_psy_main!\n");
			return 1;
		}
	} else {
		if ((psy->type == POWER_SUPPLY_TYPE_BATTERY) && (!strcmp(psy->name, "bq27x00_sec-0"))) {
			fg_psy_sec = psy;
			dev_info(bq->dev, "get battery fg_psy_sec!\n");
			return 1;
		}
	}
	return 0;
}

static struct power_supply *get_fg_chip_psy(struct bq2589x *bq)
{
	class_for_each_device(power_supply_class, NULL, bq,	check_batt_psy);
}

static int fg_get_property(struct bq2589x *bq, enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (bq->client->adapter == wcove_pmic_i2c_adapter) {
		if (!fg_psy_main)
			get_fg_chip_psy(bq);

		if (fg_psy_main) {
			ret = fg_psy_main->get_property(fg_psy_main, psp, &val);
			if (!ret)
				return val.intval;
		}
	} else {
		if (!fg_psy_sec)
			get_fg_chip_psy(bq);

		if (fg_psy_sec) {
			ret = fg_psy_sec->get_property(fg_psy_sec, psp, &val);
			if (!ret)
				return val.intval;
		}
	}
	return ret;
}

int bq2589x_get_battery_health(struct bq2589x *bq)
{
	int temp, vnow;

	dev_info(&bq->client->dev, "+%s\n", __func__);

	/* Report the battery health w.r.t battery temperature from FG */
	temp = fg_get_property(bq, POWER_SUPPLY_PROP_TEMP);
	if (temp == -ENODEV || temp == -EINVAL) {
		dev_err(&bq->client->dev, "Failed to read batt profile\n");
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	temp /= 10;
	if ((temp <= bq->min_temp) || (temp > bq->max_temp))
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	/* read the battery voltage */
	vnow = fg_get_property(bq, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vnow == -ENODEV || vnow == -EINVAL) {
		dev_err(&bq->client->dev, "Can't read voltage from FG\n");
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	/* convert voltage into millivolts */
	vnow /= 1000;
	dev_warn(&bq->client->dev, "vnow = %d\n", vnow);

	if (vnow > bq->max_cv)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	dev_info(&bq->client->dev, "-%s\n", __func__);
	return POWER_SUPPLY_HEALTH_GOOD;
}

#define NR_RETRY_CNT		3

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(bq->client, reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT) {
			continue;
		} else {
			*data = (u8)ret;
			break;
		}
	}
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	} else
		return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(bq->client, reg, data);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}
	if (ret < 0)
		dev_err(bq->dev, "failed to write 0x%.2x\n", reg);

	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2589x_read_byte(bq, &tmp, reg);
	if (ret) {
		return ret;
	}
	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, tmp);
}

static int bq2589x_detect_device(struct bq2589x* bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}
	return ret;
}

static int bq2589x_exit_hiz_mode(struct bq2589x* bq)
{
	int ret;
	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read reg 0x00.\n");
		return 0;
	}

	if (val & BQ2589X_ENHIZ_MASK) {
		dev_warn(&bq->client->dev, "bq charger IC in Hi-Z mode\n");
		val &= ~BQ2589X_ENHIZ_MASK;

		ret = bq2589x_write_byte(bq, BQ2589X_REG_00, val);
		if (ret < 0) {
			dev_err(bq->dev, "failed to write reg 0x00.\n");
			return 0;
		}
		msleep(150);
	} else {
		dev_info(&bq->client->dev, "bq charger IC is not in Hi-Z\n");
	}

	return ret;
}

static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	val=bq->usb.type;
	return  val;
#if 0

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "failed to get vbus type.\n");
		return 0;
	}

	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	if ( bq->part_no == BQ25892 ) {
		if ( (val==BQ2589X_VBUS_NONE) || (val==BQ2589X_VBUS_OTG) || (val==BQ2589X_VBUS_USB_SDP) ) {
			return val;
		} else if (val==BQ2589X_VBUS_USB_CDP) {     /* 010: Adapter(3.25A) */
			return BQ2589X_VBUS_USB_DCP;
		} else {
			dev_err(bq->dev, "Error BQ25892 vbus type: %d.\n", ret);
			return 0;
		}
	} else {   /* bq25890 or bq25895 */
		return val;
	}
#endif

}

static int bq2589x_charge_status(struct bq2589x * bq)
{
        u8 val_m = 0;
        u8 val_s = 0;
        u8 val = 0;
        u8 val2 = 0;
	
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;

        bq2589x_read_byte(g_bq, &val_m, BQ2589X_REG_0B);
        bq2589x_read_byte(g_bq_sec, &val_s, BQ2589X_REG_0B);
	val_m &= BQ2589X_CHRG_STAT_MASK;
	val_m >>= BQ2589X_CHRG_STAT_SHIFT;
	val_s &= BQ2589X_CHRG_STAT_MASK;
	val_s >>= BQ2589X_CHRG_STAT_SHIFT;

	val2=(int)(val_m+val_s);
	if(val2==0)
		val=0;
	else if(val2==6)
		val=3;
	else if(val2>0 && val2<6)
		val=2;
	else if(val2>6)
		val=4;
	
        switch(val){
                case BQ2589X_CHRG_STAT_PRECHG:
                case BQ2589X_CHRG_STAT_FASTCHG:
                        return POWER_SUPPLY_STATUS_CHARGING;
                case BQ2589X_CHRG_STAT_CHGDONE:
                        return  POWER_SUPPLY_STATUS_FULL;
                case BQ2589X_CHRG_STAT_IDLE:
                        return POWER_SUPPLY_STATUS_NOT_CHARGING;
                default:
                        return POWER_SUPPLY_STATUS_UNKNOWN;
        }
}


static irqreturn_t bq2589x_irq_isr(int irq, void *data)
{
	struct bq2589x *bq = data;

	/**TODO: This hanlder will be used for charger Interrupts */
	dev_dbg(&bq->client->dev,
		"IRQ Handled for charger interrupt: %d\n", irq);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t bq2589x_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}


static enum power_supply_type lenovo_get_power_supply_type(
		enum power_supply_charger_cable_type cable)
{

	switch (cable) {

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return POWER_SUPPLY_TYPE_USB_CDP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		return POWER_SUPPLY_TYPE_USB_ACA;
	case POWER_SUPPLY_CHARGER_TYPE_AC:
		return POWER_SUPPLY_TYPE_MAINS;
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
	default:
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_USB;
}

static int bq2589x_get_charger_health(struct bq2589x * bq)
{
	int ret_status, ret_fault, ret;

	if (bq->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = bq2589x_read_byte(bq, &ret_fault, BQ2589X_REG_0C);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read reg 0x0C.!.\n");
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	ret = bq2589x_read_byte(bq, &ret_status, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read reg 0x0B.!.\n");
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (!(ret_status & BQ2589X_PG_STAT_MASK) &&
	((ret_fault & BQ2589X_FAULT_CHRG_MASK) == (BQ2589X_FAULT_CHRG_INPUT << BQ2589X_FAULT_CHRG_SHIFT)))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	if (!(ret_status & BQ2589X_PG_STAT_MASK) && ((ret_status & BQ2589X_VBUS_STAT_MASK) == 0))
		return POWER_SUPPLY_HEALTH_DEAD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static inline int bq2589x_set_cc(struct bq2589x *bq, int curr)
{
	enum bq2589x_vbus_type type = bq2589x_get_vbus_type(bq);
	u8 ichg;

	bq->platform_data->charge_param[type].ichg = curr;
	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);
}

static inline int bq2589x_set_cv(struct bq2589x *bq, int volt)
{
	enum bq2589x_vbus_type type = bq2589x_get_vbus_type(bq);
	u8 val;

	bq->platform_data->charge_param[type].vreg = volt;
	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}

static inline int bq2589x_set_iterm(struct bq2589x *bq, int curr)
{
	u8 iterm;
	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}

static inline int bq2589x_set_inlmt(struct bq2589x *bq, int inlmt)
{
	enum bq2589x_vbus_type type = bq2589x_get_vbus_type(bq);
	u8 val;
	int timeout;
	bq->inlmt = inlmt;
	bq->platform_data->charge_param[type].ilim = inlmt;

	val = (inlmt - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
#if 0
	/* Wait for VBUS if inlimit > 0 */
	if (inlmt > 0) {
		timeout = wait_for_completion_timeout(&bq->vbus_detect, VBUS_DET_TIMEOUT);
		if (timeout == 0)
			dev_warn(&bq->client->dev, "VBUS Detect timedout. Setting INLIMIT");
	}
#endif
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}

static int bq2589x_enable_chr(struct bq2589x *bq)
{
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
}

static int bq2589x_disable_chr(struct bq2589x *bq)
{
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
}

static int bq2589x_enable_otg(struct bq2589x *bq)
{
    u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
    u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
}

static int bq2589x_enable_hw_term(struct bq2589x *bq, bool hw_term_en)
{
	int ret = 0;
	u8 val;

	dev_info(&bq->client->dev, "%s\n", __func__);

	/* Disable and enable charging to restart the charging */
	ret = bq2589x_disable_chr(bq);
	if (ret < 0) {
		dev_warn(&bq->client->dev, "Disable charger failed, ret: %d\n", ret);
		return ret;
	}
	ret = bq2589x_disable_otg(bq);
	if (ret < 0) {
		dev_warn(&bq->client->dev, "Disable OTG failed, ret: %d\n", ret);
		return ret;
	}

	/*
	 * Enable the HW termination. When disabled the HW termination, battery
	 * was taking too long to go from charging to full state. HW based
	 * termination could cause the battery capacity to drop but it would
	 * result in good battery life.
	 */
	if (hw_term_en)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);
	if (ret) {
		dev_warn(&bq->client->dev, "TERM CTRL I2C write failed!\n");
	}
	return ret;
}

/*
 * chip->event_lock need to be acquired before calling this function
 * to avoid the race condition
 * wdt_duration: BQ2589X_WDT_DISABLE/BQ2589X_WDT_40S/BQ2589X_WDT_80S/BQ2589X_WDT_160S
 */
static int bq2589x_program_timers(struct bq2589x *bq,
				int wdt_duration, bool sfttmr_enable)      //int sfttmr_duration ?
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_07);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read reg 0x07.!\n");
		return ret;
	}

	/* Program the time with duration passed */
	val &= ~BQ2589X_WDT_MASK;
	val |= (wdt_duration << BQ2589X_WDT_SHIFT) & BQ2589X_WDT_MASK;

	/* Enable/Disable the safety timer */
	if (sfttmr_enable)
		val |= BQ2589X_EN_TIMER_MASK;
	else
		val &= ~BQ2589X_EN_TIMER_MASK;

	/* Program the TIMER CTRL register */
	ret = bq2589x_write_byte(bq, BQ2589X_REG_07, val);
	if (ret < 0)
		dev_warn(&bq->client->dev, "TIMER CTRL I2C write failed\n");

	return ret;
}

static inline int bq2589x_enable_charging(struct bq2589x *bq, bool val)
{
	int ret, regval;
	enum bq2589x_vbus_type type;

	dev_warn(&bq->client->dev, "%s:%d\n", __func__, val);
#if 0
	if(g_bq!=NULL&&bq->main_charger_flag == 1)
	{
		ret = bq2589x_program_timers(g_bq, BQ2589X_WDT_160S, false);
		if (ret < 0) {
			dev_err(&g_bq->client->dev, "bq2589x_enable_charging failed: %d\n", ret);
			return ret;
		}
	}
#endif
	if (val) {
		ret = bq2589x_enable_chr(bq);
	} else {
		ret = bq2589x_disable_chr(bq);
	}
	if (ret < 0) {
		dev_err(&bq->client->dev, "enable_charging failed: %d, val=%d\n", ret, val);
		return ret;
	}

	if (val) {
		/* Schedule the charger task worker now */
		//schedule_delayed_work(&bq->chrg_task_wrkr, 0);

		/* Prevent system from entering s3 while charger is connected */
		//if (!wake_lock_active(&bq->wakelock))
		//	wake_lock(&bq->wakelock);
	} else {

		if((bq->main_charger_flag == 1)&&(sch_flag==1)&&(bq->usb.type==6))
       		 {
                	boost_flag=0;
                	sch_flag=0;
                	cnt=0;
                	printk("===bq25892====cancel sch_flag=%d\n",sch_flag);
                	cancel_delayed_work_sync(&bq->chrg_boost_wrkr);

       		 }
       		 if((g_bq!=NULL)&&(g_bq_sec!=NULL))
       		 {
                	bq2589x_set_vindpm(g_bq,4500);
                	bq2589x_set_vindpm(g_bq_sec,4500);
                	bq2589x_set_inlmt(g_bq_sec,100);
                	bq2589x_disable_chr(g_bq_sec);
                	bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);
	        }

		/* Release the wake lock */
		if(bq->main_charger_flag == 1)
		{
			if (wake_lock_active(&g_bq->wakelock))
				wake_unlock(&g_bq->wakelock);
			cancel_delayed_work_sync(&bq->chrg_task_wrkr);
		}
		/* Cancel the worker since it need not run when charging is not happening */
		cancel_delayed_work_sync(&bq->chrg_full_wrkr);

		/* Read the status to know about input supply */
		type = bq2589x_get_vbus_type(bq);

		/* If no charger connected, cancel the workers */
		if (type != BQ2589X_VBUS_OTG) {
			dev_info(&bq->client->dev, "NO charger connected\n");
			//cancel_delayed_work_sync(&bq->chrg_task_wrkr);
		}
	}
	return ret;
}

static inline int bq2589x_enable_charger(struct bq2589x *bq, bool val)
{
	int ret = 0;
#ifndef CONFIG_RAW_CC_THROTTLE
	dev_warn(&bq->client->dev, "%s:%d\n", __func__, val);
	/*stop charger for throttle state 3, by putting it in HiZ mode*/
	if (bq->cntl_state == 0x3) {
		bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT);
		if (ret < 0) {
			dev_warn(&bq->client->dev, "Input src cntl write failed!\n");
		} else {
			ret = bq2589x_enable_charging(bq, val);
		}
	}
#else
	dev_info(&bq->client->dev, "%s:%d!!!\n", __func__, val);
	if (val) {
		      	bq2589x_enable_chr(bq);
		ret = bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT);
	} else {
		     bq2589x_disable_chr(bq);
		ret = bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT);
	}
	if (ret < 0) {
		dev_err(&bq->client->dev, "bq2589x_enable_charger failed: %d, val = %d\n", ret, val);
	}
#endif
#if 1

	if((bq->main_charger_flag == 1)&&(val))
	{
		bq2589x_set_cc(g_bq,0);
		if(read_rsoc(1)==0||(read_bat(1)<3350000 && read_rsoc(1)!=0))
		{
			reset_fg(1);	
			msleep(100);
		}
	}
	else if((bq->main_charger_flag == 0)&&(val))
	{
		bq2589x_set_cc(g_bq_sec,0);
                if(read_rsoc(0)==0||(read_bat(2)<3350000 && read_rsoc(0)!=0))
		{
			reset_fg(0);	
                        msleep(100);
		}
	}
	

	if(g_bq==NULL||g_bq_sec==NULL)  return ret;
	bq2589x_write_byte(g_bq,0x05,0x71);
	bq2589x_write_byte(g_bq_sec,0x05,0x71);
	if(val && bq->main_charger_flag == 1)
	{
		if(!wake_lock_active(&g_bq->wakelock))
                        wake_lock(&g_bq->wakelock);
         	 bq2589x_program_timers(g_bq, BQ2589X_WDT_160S, false);
		 schedule_delayed_work(&bq->chrg_task_wrkr, 0);
	}
	if((bq->main_charger_flag == 1)&&(sch_flag==0)&&(val==1)&&(bq->usb.type==6))
                {
                        sch_flag=1;
			printk("bq25892=start==chrg_boost_wrkr\n");
                        schedule_delayed_work(&bq->chrg_boost_wrkr, 0);

                }
#endif
	return ret;
}

static int bq2589x_usb_set_property(struct power_supply *psy,
             enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq2589x *bq = container_of(psy, struct bq2589x, usb);
	int ret = 0;

	//dev_info(&bq->client->dev, "%s %d val=%d.\n", __func__, psp, val->intval);
	mutex_lock(&bq->event_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		bq->present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		bq->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		bq->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		bq->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		bq->max_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		bq->min_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		bq->cable_type = val->intval;
		charger_type=bq->cable_type;
		bq->usb.type = lenovo_get_power_supply_type(bq->cable_type);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		bq->cntl_state_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		if (val->intval < bq->cntl_state_max)
			bq->cntl_state = val->intval;
		else
			ret = -EINVAL;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		if(bq->usb.type==4)
                {
                        if(bq->main_charger_flag == 1)
                                ret = bq2589x_set_cc(bq, 300);
                        else
                                ret = bq2589x_set_cc(bq, 200);
                }
                else if(bq->usb.type==6)
                {
			break;
                        if(bq->main_charger_flag == 1)
                                ret = bq2589x_set_cc(bq, 1200);
                        else
                                ret = bq2589x_set_cc(bq, 800);
                }
		else
		{
                        if(bq->main_charger_flag == 1)
                                ret = bq2589x_set_cc(bq, 300);
                        else
                                ret = bq2589x_set_cc(bq, 200);
		}

		if (!ret)
			bq->cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		if(val->intval==0)
			ret = bq2589x_set_cv(bq, 0);
		else
			ret = bq2589x_set_cv(bq, 4352);
		if (!ret)
			bq->cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		ret = bq2589x_set_iterm(bq,256);
		if (!ret)
			bq->iterm = 256;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:       /* Hiz */
		ret = bq2589x_enable_charger(bq, val->intval);
		if (ret < 0) {
			dev_err(&bq->client->dev, "Error(%d) in %s charger", ret,
				(val->intval ? "enable" : "disable"));
		} else
			bq->is_charger_enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:      /* CHG_CONFIG */
		if(get_charger_type()!=0)
			break;
		bq2589x_enable_hw_term(bq, val->intval);
		mutex_unlock(&bq->event_lock);
		ret = bq2589x_enable_charging(bq, val->intval);
		if (ret < 0)
			dev_err(&bq->client->dev, "Error(%d) in %s charging", ret,
				(val->intval ? "enable" : "disable"));
		else
			bq->is_charging_enabled = val->intval;

		if (!val->intval)
			cancel_delayed_work_sync(&bq->chrg_full_wrkr);
		return ret; 
		break;
	case POWER_SUPPLY_PROP_INLMT:
		if(bq->usb.type==4)
		{
			if(bq->main_charger_flag == 1)
				ret = bq2589x_set_inlmt(bq, 300);
			else
				ret = bq2589x_set_inlmt(bq, 200);
		}
		else if(bq->usb.type==6)
		{
			if(bq->main_charger_flag == 1)
                                ret = bq2589x_set_inlmt(bq, 2000);
                        else
                                ret = bq2589x_set_inlmt(bq, 100);
		}
		else
		{
			if(bq->main_charger_flag == 1)
                                ret = bq2589x_set_inlmt(bq, 300);
                        else
                                ret = bq2589x_set_inlmt(bq, 200);
		}

		if (!ret)
			bq->inlmt = val->intval;
		break;
	default:
		ret = -ENODATA;
	}

	mutex_unlock(&bq->event_lock);
	return ret;
}

static int bq2589x_usb_get_property(struct power_supply *psy,
             enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq2589x *bq = container_of(psy, struct bq2589x, usb);

//	dev_info(&bq->client->dev, "%s %d\n", __func__, psp);
	mutex_lock(&bq->event_lock);

	switch(psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq->online;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		val->intval = bq->max_cc;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		val->intval = bq->max_cv;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = bq->cable_type;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		val->intval = bq->max_temp;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		val->intval = bq->min_temp;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2589x_get_charger_health(bq);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = (bq2589x_get_charger_health(bq) == POWER_SUPPLY_HEALTH_GOOD);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = bq->cntl_state_max;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = bq->cntl_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		val->intval = bq->cc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		val->intval = bq->cv;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		val->intval = bq->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = bq->iterm;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		if (bq->boost_mode)
			val->intval = false;
		else {

				val->intval=1;
				//val->intval = (bq->is_charging_enabled && (bq2589x_charge_status(bq)>1));
		}
		break;
	default:
		mutex_unlock(&bq->event_lock);
		return -ENODATA;
	}

	mutex_unlock(&bq->event_lock);
	//dev_info(&bq->client->dev, "%s %d, val = %d\n", __func__, psp, val->intval);
	return 0;
}

char *bq25892_supplied_to_main[] = {
	"intel_fuel_gauge",
	"bq27x00_main-1",
};

char *bq25892_supplied_to_sec[] = {
	"intel_fuel_gauge",
	"bq27x00_sec-0",
};

static int bq2589x_psy_register(struct bq2589x *bq)
{
	int ret;

	if (bq->client->adapter == wcove_pmic_i2c_adapter) {   /* Main charger/battery */
		bq->usb.name = "usb-main";
		bq->usb.supplied_to = bq25892_supplied_to_main;
		bq->usb.num_supplicants = ARRAY_SIZE(bq25892_supplied_to_main);
	} else {                                               /* Second charger/battery */
		bq->usb.name = "usb-sec";
		bq->usb.supplied_to = bq25892_supplied_to_sec;
		bq->usb.num_supplicants = ARRAY_SIZE(bq25892_supplied_to_sec);
	}
	bq->usb.type = POWER_SUPPLY_TYPE_USB;
	bq->usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
	bq->usb.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->usb.properties = bq2589x_charger_props;
	bq->usb.get_property = bq2589x_usb_get_property;
	bq->usb.set_property = bq2589x_usb_set_property;
	bq->usb.external_power_changed = NULL;

	ret = power_supply_register(bq->dev, &bq->usb);
	if(ret < 0){
		dev_err(bq->dev,"%s:failed to register usb psy:%d\n",__func__,ret);
		return ret;
	}

	ret = power_supply_register_charger(&bq->usb);
	if (ret) {
		dev_err(bq->dev, "failed:power supply charger register\n");
		power_supply_unregister(&bq->usb);
		return ret;
	}

	return 0;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
	power_supply_unregister_charger(&bq->usb);
    power_supply_unregister(&bq->usb);
}

static void bq2589x_full_worker(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, chrg_full_wrkr.work);

	dev_info(bq->dev, "schedule bq2589x_full_worker!\n");

	//power_supply_changed(NULL);

	/* schedule the thread to let the framework know about FULL */
	schedule_delayed_work(&bq->chrg_full_wrkr, FULL_THREAD_JIFFIES);
}

/* This function should be called with the mutex held */
int bq2589x_reset_wdt_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

/*
 *This function will modify the VINDPM when FORCE_VINDPM(REG0D[7])=1
 */
int bq2589x_set_vindpm(struct bq2589x *bq, int volt)
{
	enum bq2589x_vbus_type type = bq2589x_get_vbus_type(bq);
	u8 val;

	if(bq->platform_data->force_vindpm){
		bq->platform_data->charge_param[type].vlim = volt;
		val = (volt - BQ2589X_VINDPM_BASE)/BQ2589X_VINDPM_LSB;
		return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
	}
	return 0;
}

static void bq2589x_boost_worker(struct work_struct *work)
{
        struct bq2589x *bq = container_of(work, struct bq2589x, chrg_boost_wrkr.work);
     	int v1,v2;

	v1=read_bat(1)/1000;
	v2=read_bat(2)/1000;
	printk("bq25892=================main_bat=%d sec_bat=%d\n",v1,v2); 
	 if((v1 > 3500 && v2 > 3500)&&(boost_flag==0))
        {
		msleep(100);
		boost_fun();
		schedule_delayed_work(&g_bq->chrg_boost_wrkr,0);
        }
	else
	{
		config_bq25892();	
	 	schedule_delayed_work(&g_bq->chrg_boost_wrkr, 60*HZ);
	}

}


void  boost_fun(void)
{
		int vbus=0;

		//if(cnt==0)
                        enable_pumpx();
                cnt++;
                bq2589x_pumpx_increase_volt();
                while(bq2589x_pumpx_increase_volt_done()!=0);
                if(cnt==3)
                {

                        vbus=bq2589x_adc_read_vbus_volt();
                        printk("===bq25892====2-vbus=%d\n",vbus);
                        if(vbus>11000)
                        {
                                boost_flag=1;
				disable_pumpx();
                        }
                }
                if(cnt==6)
                {
                                boost_flag=1;
                                cnt=0;
				disable_pumpx();
                }

}


void  disable_pumpx(void)
{
		bq2589x_pumpx_enable(0);
       		bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_DISABLE<<BQ2589X_ENHIZ_SHIFT);
        	bq2589x_enable_chr(g_bq);
        	bq2589x_enable_chr(g_bq_sec);
}

void enable_pumpx(void)
{
		bq2589x_enable_chr(g_bq);
                bq2589x_set_cc(g_bq,500);
                bq2589x_set_inlmt(g_bq,500);
                bq2589x_set_vindpm(g_bq,4500);
                bq2589x_set_cc(g_bq_sec,0);
                bq2589x_set_inlmt(g_bq_sec,100);
                bq2589x_set_vindpm(g_bq_sec,4500);
                bq2589x_disable_chr(g_bq_sec);
                bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);
                bq2589x_pumpx_enable(1);
}

int charger_temp(int y)
{
	int i,x,z,w;
	
	w=30*y;
	z=300-4*y;
	x=w*100/z;

	printk("bq25892 charger_temp   x=%d  w=%d  z=%d\n",x,w,z);	
	for(i=0;i<181;i++)
	{
		if(x>=R[i]*100)
			return T[i]*10;
		else
			continue;
	}
	return x;

}

void config_bq25892(void)
{
		int vbus=0,t1=0,t2=0,y1,y2;
		int  charger_main,charger_sec;
#if 0
		if(yt3_hw_ver!=0)
		{
			y1=bq2589x_adc_read_temperature();
			y2=bq2589x_adc_read_temperature_sec();
			charger_main=charger_temp(y1);
			charger_sec=charger_temp(y2);
			printk("==bq25892===y1=%d y2=%d charger_main_temp=%d charger_sec_temp=%d\n", y1,y2,charger_main,charger_sec);
		}
#endif
		t1=read_battery_temp(1)/100;
		t2=read_battery_temp(0)/100;

		vbus=bq2589x_adc_read_vbus_volt();
        	printk("===bq25892====1-vbus=%d  t1=%d t2=%d\n",vbus,t1,t2);
		if((read_rsoc(0)<100)&&(t2>=0&&t2<=50))
		{
		    	bq2589x_enable_chr(g_bq_sec);
	                bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_DISABLE<<BQ2589X_ENHIZ_SHIFT);
		}
		if((read_rsoc(1)<100)&&(t1>=0&&t1<=50))
		{
			bq2589x_enable_chr(g_bq);
	                bq2589x_update_bits(g_bq,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_DISABLE<<BQ2589X_ENHIZ_SHIFT);

		}
		if(t1<0||t1>50)
		{
	                bq2589x_disable_chr(g_bq);
		}
		if(t2<0||t2>50)
		{
	                bq2589x_disable_chr(g_bq_sec);
		}

		if((is_bat_cf(1)==1)&&(read_rsoc(1)==100))
		{
	                bq2589x_disable_chr(g_bq);

		}
		
		if((is_bat_cf(0)==1)&&(read_rsoc(0)==100))
		{
	                bq2589x_disable_chr(g_bq_sec);

		}

		if((is_bat_cf(1)==1)&&(read_rsoc(1)==100)&&(is_bat_cf(0)==1)&&(read_rsoc(0)==100))		
		{
			if(wake_lock_active(&g_bq->wakelock))
		          	wake_unlock(&g_bq->wakelock);
			printk("unlock====bq25892\n");
		}

		if(vbus>11000)
		{
		    if(t1>=0&&t1<15)
		     {
			
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,1024);
			
		     }
		    else if(t1>=15&&t1<45)
		    {
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,3000);

		    }
		    else if(t1>=45&&t1<=50)
		    {
                    	bq2589x_set_cv(g_bq,4208);
                    	bq2589x_set_cc(g_bq,1024);

		     }

		    if(t2>=0&&t2<15)
                     {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,600);


                     }
                    else if(t2>=15&&t2<45)
                    {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,2000);

                    }
                    else if(t2>=45&&t2<=50)
                    {
                    	bq2589x_set_cv(g_bq_sec,4208);
                    	bq2589x_set_cc(g_bq_sec,600);

                     }

		    bq2589x_set_inlmt(g_bq,1200);		
		    bq2589x_set_vindpm(g_bq,4500);
		    bq2589x_set_inlmt(g_bq_sec,800);		
		    bq2589x_set_vindpm(g_bq_sec,4500);
		}
                else if(vbus>8000&&vbus<11000)
		{
		    if(t1>=0&&t1<15)
		     {
			
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,1024);
			
		     }
		    else if(t1>=15&&t1<45)
		    {
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,3000);

		    }
		    else if(t1>=45&&t1<=50)
		    {
                    	bq2589x_set_cv(g_bq,4208);
                    	bq2589x_set_cc(g_bq,1024);

		     }

		    if(t2>=0&&t2<15)
                     {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,600);


                     }
                    else if(t2>=15&&t2<45)
                    {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,2000);

                    }
                    else if(t2>=45&&t2<=50)
                    {
                    	bq2589x_set_cv(g_bq_sec,4208);
                    	bq2589x_set_cc(g_bq_sec,600);

                     }

		    bq2589x_set_inlmt(g_bq,1200);		
		    bq2589x_set_vindpm(g_bq,4500);
		    bq2589x_set_inlmt(g_bq_sec,800);		
		    bq2589x_set_vindpm(g_bq_sec,4500);

		}
		else if(vbus>6000&&vbus<8000)
		{
		    if(t1>=0&&t1<15)
		     {
			
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,1024);
			
		     }
		    else if(t1>=15&&t1<45)
		    {
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,3000);

		    }
		    else if(t1>=45&&t1<=50)
		    {
                    	bq2589x_set_cv(g_bq,4208);
                    	bq2589x_set_cc(g_bq,1024);

		     }

		    if(t2>=0&&t2<15)
                     {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,600);


                     }
                    else if(t2>=15&&t2<45)
                    {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,2000);

                    }
                    else if(t2>=45&&t2<=50)
                    {
                    	bq2589x_set_cv(g_bq_sec,4208);
                    	bq2589x_set_cc(g_bq_sec,600);

                     }

		    bq2589x_set_inlmt(g_bq,1200);		
		    bq2589x_set_vindpm(g_bq,4500);
		    bq2589x_set_inlmt(g_bq_sec,800);		
		    bq2589x_set_vindpm(g_bq_sec,4500);

		}
		else if(vbus>4000&&vbus<6000)
		{
		    if(t1>=0&&t1<15)
		     {
			
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,1024);
			
		     }
		    else if(t1>=15&&t1<45)
		    {
                    	bq2589x_set_cv(g_bq,4352);
                    	bq2589x_set_cc(g_bq,1200);

		    }
		    else if(t1>=45&&t1<=50)
		    {
                    	bq2589x_set_cv(g_bq,4208);
                    	bq2589x_set_cc(g_bq,1024);

		     }

		    if(t2>=0&&t2<15)
                     {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,600);


                     }
                    else if(t2>=15&&t2<45)
                    {
                    	bq2589x_set_cv(g_bq_sec,4352);
                    	bq2589x_set_cc(g_bq_sec,800);

                    }
                    else if(t2>=45&&t2<=50)
                    {
                    	bq2589x_set_cv(g_bq_sec,4208);
                    	bq2589x_set_cc(g_bq_sec,600);

                     }

		    bq2589x_set_inlmt(g_bq,1200);		
		    bq2589x_set_vindpm(g_bq,4500);
		    bq2589x_set_inlmt(g_bq_sec,800);		
		    bq2589x_set_vindpm(g_bq_sec,4500);

		}
                   power_supply_changed(&g_bq->usb);


}
static void bq2589x_task_worker(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, chrg_task_wrkr.work);
	int ret, jiffy = CHARGER_TASK_JIFFIES, vbatt;
	static int prev_health = POWER_SUPPLY_HEALTH_GOOD;
	int curr_health;
	u8 vindpm, val;
	dev_info(&bq->client->dev, "%s\n", __func__);
	/* Reset the WDT */
	if(g_bq==NULL|| g_bq_sec==NULL) return;

	if(bq->main_charger_flag == 1)
	{
		mutex_lock(&g_bq->event_lock);
		ret = bq2589x_reset_wdt_timer(g_bq);
		//bq2589x_program_timers(g_bq, BQ2589X_WDT_80S, false);		
		mutex_unlock(&g_bq->event_lock);
		if (ret < 0)
			dev_warn(&g_bq->client->dev, "WDT reset failed: %d.!\n", ret);
	}

	/*
	 * If we have an OTG device connected, no need to modify the VINDPM
	 * check for Hi-Z
	 */
	if (bq->boost_mode) {
		bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1  add
		jiffy = CHARGER_HOST_JIFFIES;
		goto sched_task_work;
	}

#ifndef CONFIG_RAW_CC_THROTTLE
	if (!(bq->cntl_state == 0x3)) {
		/* Clear the charger from Hi-Z */
		ret = bq2589x_exit_hiz_mode(bq);
		if (ret < 0)
			dev_warn(&bq->client->dev, "HiZ clear failed:\n");
	}
#else
	ret = bq2589x_exit_hiz_mode(bq);
	if (ret < 0)
		dev_warn(&bq->client->dev, "HiZ clear failed.\n");
#endif

	/* Modify the VINDPM */

	/* read the battery voltage */
	vbatt = fg_get_property(bq, POWER_SUPPLY_PROP_VOLTAGE_OCV);
	if (vbatt == -ENODEV || vbatt == -EINVAL) {
		//dev_err(&bq->client->dev, "Can't read voltage from FG\n");
		goto sched_task_work;
	}

	/* convert voltage into millivolts */
	vbatt /= 1000;
	//dev_warn(&bq->client->dev, "vbatt = %d\n", vbatt);

	/* The charger vindpm voltage changes are causing charge current
	 * throttle resulting in a prolonged changing time.
	 * Hence disabling dynamic vindpm update  for bq24296 chip.
	*/


	/*
	 * BQ driver depends upon the charger interrupt to send notification
	 * to the framework about the HW charge termination and then framework
	 * starts to poll the driver for declaring FULL. Presently the BQ
	 * interrupts are not coming properly, so the driver would notify the
	 * framework when battery is nearing FULL.
	*/
//	if (vbatt >= BATTERY_NEAR_FULL(bq->max_cv))
//		power_supply_changed(NULL);

sched_task_work:

	curr_health = bq2589x_get_battery_health(bq);
	if (prev_health != curr_health) {
		power_supply_changed(&bq->usb);
		//dev_warn(&bq->client->dev, "%s health status %d", __func__, prev_health);
	}
	prev_health = curr_health;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret) {
		dev_err(bq->dev, "BQ2589X_0B read failure:%d\n", ret);
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	if (BQ2589X_CHRG_STAT_CHGDONE == (val >> BQ2589X_CHRG_STAT_SHIFT)) {
		//power_supply_changed(&bq->usb);
		//dev_warn(&bq->client->dev, "%s battery full.\n", __func__);
	}

	schedule_delayed_work(&bq->chrg_task_wrkr, jiffy);
}

static void bq2589x_resume_charging(struct bq2589x *bq)
{
#if 0
	if (bq->inlmt)
		bq2589x_set_inlmt(bq, bq->inlmt);
	if (bq->cc)
		bq2589x_set_cc(bq, bq->cc);
	if (bq->cv)
		bq2589x_set_cv(bq, bq->cv);
	if (bq->is_charging_enabled)
		bq2589x_enable_charging(bq, true);
	if (bq->is_charger_enabled)
		bq2589x_enable_charger(bq, true);
#endif

}

static void bq2589x_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	u8 temp = 0;
	int ret;



	if(g_bq==NULL||g_bq_sec==NULL)   return;
	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		dev_err(bq->dev, "IRQ 0B failure:%d\n", ret);
		return;
	}
	
	if((bq->main_charger_flag == 0 && g_bq->boost_mode == true) && (status==0x02||status==0x06))
	{
	        bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1  add
	}

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) {
		dev_err(bq->dev, "IRQ 0C failure: %d\n", ret);
		return;
	}
	dev_info(bq->dev, "irq_workfunc, REG0B=0x%x, REG0C=0x%x\n", status, fault);

	/*
	 * On VBUS detect set completion to wake waiting thread. On VBUS
	 * disconnect, re-init completion so that setting INLIMIT would be
	 * delayed till VBUS is detected.
	 */
	temp = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	if (bq->part_no == BQ25892) {
		if ( (temp == 1) || (temp == 2) )    /* SDP or Adapter */
			complete(&bq->vbus_detect);
		else
			reinit_completion(&bq->vbus_detect);
	} else if (bq->part_no == BQ25890) {
	}

	temp = (status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	if (temp == BQ2589X_CHRG_STAT_CHGDONE) {
		dev_warn(&bq->client->dev, "HW termination happened!\n");
		//mutex_lock(&bq->event_lock);
		//bq2589x_enable_hw_term(bq, false);
		//bq2589x_resume_charging(bq);
		//mutex_unlock(&bq->event_lock);
		/* schedule the thread to let the framework know about FULL */
		if((bq->main_charger_flag == 1)&&(read_rsoc(1)<100)&&(read_bat(1) > 4300000))
			reset_fg(1);
		else if((bq->main_charger_flag == 0)&&(read_rsoc(0)<100)&&(read_bat(2) > 4300000))	
			reset_fg(0);
		//schedule_delayed_work(&bq->chrg_full_wrkr, 0);
	}

	if(fault & BQ2589X_FAULT_WDT_MASK) {
		dev_info(bq->dev, "%s:Watchdog Reset occured!\n", __func__);
		if (bq->is_charger_enabled) {
			if((g_bq!=NULL)&&(bq->main_charger_flag == 1))
			{
				bq2589x_program_timers(g_bq, BQ2589X_WDT_160S, false);
				//mutex_lock(&g_bq->event_lock);
				//bq2589x_resume_charging(g_bq);
				//mutex_unlock(&g_bq->event_lock);
			}
		} else {
			//dev_info(&bq->client->dev, "No charger connected !!!!\n");
		}
	}

	if(fault & BQ2589X_FAULT_BOOST_MASK) {
		dev_info(bq->dev, "%s:Boost Fault occured!\n", __func__);
	}

	temp = (fault & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT; 
	if(temp == BQ2589X_FAULT_CHRG_INPUT) {
		dev_info(bq->dev, "%s:input fault occurred, VBUS OVP or VBAT < VBUS < 3.8V\n", __func__);
	} else if(temp == BQ2589X_FAULT_CHRG_THERMAL) {
		dev_info(bq->dev, "%s:thermal fault occurred\n", __func__);
		//TODO:stop charger
	} else if(temp == BQ2589X_FAULT_CHRG_TIMER) {
		dev_info(bq->dev, "%s:charge timeout fault occurred\n", __func__);
		//TODO:stop charger
	}

	if(fault & BQ2589X_FAULT_BAT_MASK) {
		dev_info(bq->dev, "%s:battery OVP fault occurred\n", __func__);
		//TODO:
		if(bq->main_charger_flag == 0 && g_bq->boost_mode == true)
			{
			  bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1  add
			}
	}    

	temp = (fault & BQ2589X_FAULT_NTC_MASK) >> BQ2589X_FAULT_NTC_SHIFT;
	if(bq->part_no == BQ25895){
		if(temp == BQ2589X_FAULT_NTC_TSCOLD)
			dev_info(bq->dev, "%s:bq25895 TS cold fault occurred\n", __func__);
		else if(temp == BQ2589X_FAULT_NTC_TSHOT)
			dev_info(bq->dev, "%s:bq25895 TS hot fault occurred\n", __func__);
	} else {      //25890/892
		if (temp == BQ2589X_FAULT_NTC_WARM)
			dev_info(bq->dev, "%s:bq25890/2 TS warm fault occurred\n", __func__);
		else if (temp == BQ2589X_FAULT_NTC_COOL)
			dev_info(bq->dev, "%s:bq25890/2 TS cool fault occurred\n", __func__);
		else if (temp == BQ2589X_FAULT_NTC_COLD)
			dev_info(bq->dev, "%s:bq25890/2 TS cold fault occurred\n", __func__);
		else if (temp == BQ2589X_FAULT_NTC_HOT)
			dev_info(bq->dev, "%s:bq25890/2 TS Hot fault occurred\n", __func__);
	}

	temp = (status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	if (temp != BQ2589X_CHRG_STAT_CHGDONE) {
		//power_supply_changed(&bq->usb);
	}
}

/**************************************************************/
static int bq2589x_update_charge_params(struct bq2589x *bq)
{
	enum bq2589x_vbus_type type = bq2589x_get_vbus_type(bq);
	int ret;
	u8 vlim;
	u8 ilim;
	u8 ichg;
	u8 vreg;
	u8 val;

	if(type == BQ2589X_VBUS_OTG)
        return 0;
	vlim = bq->platform_data->charge_param[type].vlim;
	ilim = bq->platform_data->charge_param[type].ilim;
	ichg = bq->platform_data->charge_param[type].ichg;
	vreg = bq->platform_data->charge_param[type].vreg;

	if(bq->platform_data->force_vindpm){
		val = (vlim - BQ2589X_VINDPM_BASE)/BQ2589X_VINDPM_LSB;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
		if (ret)
			return ret;
	}

	val = (ilim - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
	if (ret)
		return ret;

	val = (ichg - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, val << BQ2589X_ICHG_SHIFT);
	if (ret)
		return ret;

	val = (vreg - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}

static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt )
{
    u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE)/BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	
	return bq2589x_update_bits(bq,BQ2589X_REG_0A,BQ2589X_BOOSTV_MASK,val);
}

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr )
{
	u8 temp;

    if(curr  == 500)
        temp = BQ2589X_BOOST_LIM_500MA;
    else if(curr == 700)
        temp = BQ2589X_BOOST_LIM_700MA;
    else if(curr == 1100)
        temp = BQ2589X_BOOST_LIM_1100MA;
    else if(curr == 1600)
        temp = BQ2589X_BOOST_LIM_1600MA;
    else if(curr == 1800)
        temp = BQ2589X_BOOST_LIM_1800MA;
    else if(curr == 2100)
        temp = BQ2589X_BOOST_LIM_2100MA;
    else if(curr == 2400)
        temp = BQ2589X_BOOST_LIM_2400MA;
    else
        temp = BQ2589X_BOOST_LIM_1300MA;

    return bq2589x_update_bits(bq,BQ2589X_REG_0A,BQ2589X_BOOST_LIM_MASK,temp << BQ2589X_BOOST_LIM_SHIFT);
}


/* This function should be called with the mutex held */
static int bq2589x_turn_otg_vbus(struct bq2589x *bq, bool votg_on)
{
	int ret = 0;

	dev_info(&bq->client->dev, "%s %d\n", __func__, votg_on);
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
	if (votg_on) {
			if(!wake_lock_active(&bq->wakelock))
	                        wake_lock(&bq->wakelock);
			gpio_direction_output(bq->platform_data->gpio_ce, 1);	
			bq2589x_update_bits(g_bq,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_DISABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1 add 
			bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1  add
			bq2589x_set_inlmt(g_bq_sec,100);
			 bq2589x_disable_chr(g_bq_sec);
			/* Program the timers */
#if 1
			if(g_bq!=NULL)
			{
				ret = bq2589x_program_timers(g_bq, BQ2589X_WDT_160S, false);
				if (ret < 0) {
					dev_warn(&g_bq->client->dev, "TIMER enable failed %s\n", __func__);
					goto i2c_write_fail;
				}
			}
#endif
			/* Configure the charger in OTG mode */
			ret = bq2589x_enable_otg(bq);
			if (ret < 0) {
				dev_warn(&bq->client->dev, "enable OTG failed!\n");
				goto i2c_write_fail;
			}
			ret = bq2589x_set_otg_current(bq, 500);
			if (ret < 0) {
				dev_warn(&bq->client->dev, "set otg current failed!\n");
				goto i2c_write_fail;
			}
			ret = bq2589x_set_otg_volt(bq, 4998);          //return fail ???
			if (ret < 0) {
				dev_warn(&bq->client->dev, "set otg volt failed!\n");
				goto i2c_write_fail;
			}
  
			bq->boost_mode = true;
			msleep(100);
			bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);  //liulc1  add
			/* Schedule the charger task worker now */
			schedule_delayed_work(&bq->chrg_task_wrkr, 0);
	} else {
			if (wake_lock_active(&bq->wakelock))
	                        wake_unlock(&bq->wakelock);
                                gpio_direction_output(bq->platform_data->gpio_ce, 0);

			/* Clear the charger from the OTG mode */
			ret = bq2589x_disable_otg(bq);
			if (ret < 0) {
				dev_warn(&bq->client->dev, "disable OTG failed!\n");
				goto i2c_write_fail;
			}
			bq->boost_mode = false;
			/* Cancel the charger task worker now */
			cancel_delayed_work_sync(&bq->chrg_task_wrkr);
	}
	/* Drive the gpio to turn ON/OFF the VBUS */

	return ret;
i2c_write_fail:
	dev_err(&bq->client->dev, "%s: Failed\n", __func__);
	return ret;
}

static void bq2589x_usb_otg_enable(struct usb_phy *phy, int on)
{
	struct bq2589x *bq = g_bq;
	int ret;

	mutex_lock(&bq->event_lock);
	ret =  bq2589x_turn_otg_vbus(bq, on);
	mutex_unlock(&bq->event_lock);
	if (ret < 0)
		dev_err(&bq->client->dev, "VBUS mode(%d) failed!\n", on);
}

static inline int bq2589x_register_otg_vbus(struct bq2589x *bq)
{
	int ret;

	bq->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!bq->transceiver) {
		dev_err(&bq->client->dev, "Failed to get the USB transceiver!\n");
		return -EINVAL;
	}
	bq->transceiver->set_vbus = bq2589x_usb_otg_enable;

	return 0;
}

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;
	u8 temp;

	// ico enable
	if(bq->platform_data->en_ico)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_ICOEN_MASK,BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_ICOEN_MASK,BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT);
	if(ret < 0) return ret;
	// hvdcp enable
	if(bq->platform_data->en_hvdcp)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_HVDCPEN_MASK,BQ2589X_HVDCP_ENABLE << BQ2589X_HVDCPEN_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_HVDCPEN_MASK,BQ2589X_HVDCP_DISABLE << BQ2589X_HVDCPEN_SHIFT);
	if(ret < 0) return ret;
	// max charge enable
	if(bq->platform_data->en_maxcharge)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_MAXCEN_MASK,BQ2589X_MAXC_ENABLE << BQ2589X_MAXCEN_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_MAXCEN_MASK,BQ2589X_MAXC_DISABLE << BQ2589X_MAXCEN_SHIFT);
	if(ret < 0) return ret;

	// auto dpdm
	if(bq->platform_data->auto_dpdm)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_AUTO_DPDM_EN_MASK,BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_AUTO_DPDM_EN_MASK,BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT);
	if(ret < 0) return ret;
	// adc scan mode
	if(bq->platform_data->adc_continue_convert)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK,BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK,BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
	if(ret < 0) return ret;
	// watchdog 
	if(bq->platform_data->enable_watchdog)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_WDT_MASK, (u8)((bq->platform_data->watchdog_timeout - BQ2589X_WDT_BASE)/BQ2589X_WDT_LSB)<< BQ2589X_WDT_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_WDT_MASK, BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT);
	if(ret < 0) return ret;
	// charger timer
	if(bq->platform_data->enable_charge_timer)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_EN_TIMER_MASK, BQ2589X_CHG_TIMER_ENABLE << BQ2589X_EN_TIMER_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_EN_TIMER_MASK, BQ2589X_CHG_TIMER_DISABLE << BQ2589X_EN_TIMER_SHIFT);
	if(ret < 0) return ret;

	if(bq->platform_data->charge_timeout == 5)
		temp = BQ2589X_CHG_TIMER_5HOURS;
	else if(bq->platform_data->charge_timeout == 8)   
		temp = BQ2589X_CHG_TIMER_8HOURS;
	else if(bq->platform_data->charge_timeout == 20)   
		temp = BQ2589X_CHG_TIMER_8HOURS;
	else
		temp = BQ2589X_CHG_TIMER_12HOURS;

	ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_CHG_TIMER_MASK, temp << BQ2589X_CHG_TIMER_SHIFT);
	if(ret < 0) return ret;
	// boost freq
	if(bq->platform_data->boost_frequency == 1500)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_BOOST_FREQ_MASK,BQ2589X_BOOST_FREQ_1500K << BQ2589X_BOOST_FREQ_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_BOOST_FREQ_MASK,BQ2589X_BOOST_FREQ_500K << BQ2589X_BOOST_FREQ_SHIFT);
	if(ret < 0) return ret;
	// boost voltage
	ret = bq2589x_set_otg_volt(bq,bq->platform_data->boost_voltage);
	if(ret < 0) return ret;
	// boost current limit
	ret = bq2589x_set_otg_current(bq,bq->platform_data->boost_ilimit);
	if(ret < 0) return ret;

	// IR compensation
	temp = (bq->platform_data->ir_comp_resistance - BQ2589X_BAT_COMP_BASE)/BQ2589X_BAT_COMP_LSB;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_08,BQ2589X_BAT_COMP_MASK,temp << BQ2589X_BAT_COMP_SHIFT);
	if(ret < 0) return ret;

	temp = (bq->platform_data->ir_comp_vclamp - BQ2589X_VCLAMP_BASE)/BQ2589X_VCLAMP_LSB;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_08,BQ2589X_VCLAMP_MASK,temp << BQ2589X_VCLAMP_SHIFT);
	if(ret < 0) return ret;
	// thermal regulation
	if(bq->platform_data->thermal_regulation_threshold == 60)
		temp = BQ2589X_TREG_60C;
	else if(bq->platform_data->thermal_regulation_threshold == 80)
		temp = BQ2589X_TREG_80C;
	else if(bq->platform_data->thermal_regulation_threshold == 100)
		temp = BQ2589X_TREG_100C;
	else
        temp = BQ2589X_TREG_120C;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_08,BQ2589X_TREG_MASK,temp << BQ2589X_TREG_SHIFT);
	if(ret < 0) return ret;

	//vset
	if(bq->platform_data->jeita_vset == 150)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_09,BQ2589X_JEITA_VSET_MASK, BQ2589X_JEITA_VSET_N150MV << BQ2589X_JEITA_VSET_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_09,BQ2589X_JEITA_VSET_MASK, BQ2589X_JEITA_VSET_VREG << BQ2589X_JEITA_VSET_SHIFT);
	if(ret < 0) return ret;

	// iset
	if(bq->platform_data->jeita_iset == 50)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_JEITA_ISET_MASK, BQ2589X_JEITA_ISET_50PCT << BQ2589X_JEITA_ISET_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_JEITA_ISET_MASK, BQ2589X_JEITA_ISET_20PCT << BQ2589X_JEITA_ISET_SHIFT);
	if(ret < 0) return ret;

	// system min voltage
	temp = (bq->platform_data->sys_min_voltage - BQ2589X_SYS_MINV_BASE)/BQ2589X_SYS_MINV_LSB;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_03,BQ2589X_SYS_MINV_MASK,temp << BQ2589X_SYS_MINV_SHIFT);
	if(ret < 0) return ret;
	// enable termination detect
	if(bq->platform_data->enable_termination)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_EN_TERM_MASK, BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_EN_TERM_MASK, BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT);
	if(ret < 0) return ret;

	// termination current
	temp = (bq->platform_data->termination_current - BQ2589X_ITERM_BASE)/BQ2589X_ITERM_LSB;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_05,BQ2589X_ITERM_MASK,temp << BQ2589X_ITERM_SHIFT);
	if(ret < 0) return ret;
	// precharge voltage
	if (bq->platform_data->precharge_voltage == 2800)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,BQ2589X_BATLOWV_2800MV << BQ2589X_BATLOWV_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,BQ2589X_BATLOWV_3000MV << BQ2589X_BATLOWV_SHIFT);
	if(ret < 0) return ret;

	if (bq->platform_data->recharge_threshold == 200)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_06,BQ2589X_VRECHG_MASK,BQ2589X_VRECHG_200MV << BQ2589X_VRECHG_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_06,BQ2589X_VRECHG_MASK,BQ2589X_VRECHG_100MV << BQ2589X_VRECHG_SHIFT);
	if(ret < 0) return ret;

	// ilimit pin
	if(bq->platform_data->enable_ilimit_pin)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_00,BQ2589X_ENILIM_MASK, BQ2589X_ENILIM_ENABLE << BQ2589X_ENILIM_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_00,BQ2589X_ENILIM_MASK, BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
	if(ret < 0) return ret;
	//vindpm offset
	temp = (bq->platform_data->vindpm_offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
	ret = bq2589x_update_bits(bq,BQ2589X_REG_01,BQ2589X_VINDPMOS_MASK,temp << BQ2589X_VINDPMOS_SHIFT);
	if(ret < 0) return ret;
	// force vindpm?
	if(bq->platform_data->force_vindpm)
		ret = bq2589x_update_bits(bq,BQ2589X_REG_0D,BQ2589X_FORCE_VINDPM_MASK, BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT);
	else
		ret = bq2589x_update_bits(bq,BQ2589X_REG_0D,BQ2589X_FORCE_VINDPM_MASK, BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT);
	if(ret < 0) return ret;
	// charge profile
	ret = bq2589x_update_charge_params(bq);

	return ret;
}

static int bq2589x_init_main(struct bq2589x *bq)
{
	bq2589x_write_byte(bq,0x00,0x66);
	bq2589x_write_byte(bq,0x01,0xc6);
	bq2589x_write_byte(bq,0x02,0x10);
	bq2589x_write_byte(bq,0x03,0x5e);
	bq2589x_write_byte(bq,0x04,0x30);
	bq2589x_write_byte(bq,0x05,0x71);
	bq2589x_write_byte(bq,0x06,0x82);
	bq2589x_write_byte(bq,0x07,0xcc);
	bq2589x_write_byte(bq,0x08,0x03);
	bq2589x_write_byte(bq,0x09,0x54);
	bq2589x_write_byte(bq,0x0a,0x70);
	bq2589x_write_byte(bq,0x0d,0x93);
	bq2589x_write_byte(bq,0x14,0x45);

}

static int bq2589x_init_sec(struct bq2589x *bq)
{
        bq2589x_write_byte(bq,0x00,0xc0);
        bq2589x_write_byte(bq,0x01,0xc6);
        bq2589x_write_byte(bq,0x02,0x10);
        bq2589x_write_byte(bq,0x03,0x0e);
        bq2589x_write_byte(bq,0x04,0x0);
        bq2589x_write_byte(bq,0x05,0x71);
        bq2589x_write_byte(bq,0x06,0x82);
        bq2589x_write_byte(bq,0x07,0xcc);
        bq2589x_write_byte(bq,0x08,0x03);
        bq2589x_write_byte(bq,0x09,0x54);
        bq2589x_write_byte(bq,0x0a,0x70);
        bq2589x_write_byte(bq,0x0d,0x93);
        bq2589x_write_byte(bq,0x14,0x45);

}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		bq2589x_read_byte(bq, &val, addr);
		dev_info(bq->dev, "[0x%.2x] = 0x%.2x\n", addr, val);
	}
	return 0;
}

static DEVICE_ATTR(registers, S_IRUGO, bq2589x_show_registers, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};

//liulc1 add
extern int chao=1000;
static ssize_t u_read(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);


        return snprintf(buf, PAGE_SIZE, "%d\n", chao);
}

static ssize_t u_write(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        struct bq2589x *bq = dev_get_drvdata(dev);

	sscanf(buf, "%d", &chao);


        return count;
}

static DEVICE_ATTR(chao_charger, S_IRUGO | S_IWUSR | S_IWGRP, u_read, u_write);

static ssize_t read_mchg_ce(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
	int ch;
		
	ch=intel_soc_pmic_readb(0x5e2f);
	if(ch == -EIO)
	{
		printk("bq25892=====read-0x5e2f  err!!!\n");	
		ch=-1;
	}
		

        return snprintf(buf, PAGE_SIZE, "%d\n", ch);
}

static ssize_t write_mchg_ce(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
	int  ch,ret;

        sscanf(buf, "%d", &ch);
	if(ch==0)
	{
		ret=intel_soc_pmic_writeb(0x5e2f,0x00);
		if(ret)
          	     printk("bq25892===write-0x5e2f err!!!\n");
	}

        return count;
}

static DEVICE_ATTR(mchg_ce, S_IRUGO | S_IWUSR | S_IWGRP, read_mchg_ce, write_mchg_ce);


static ssize_t enter_ship_mode(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
	int ch;
        sscanf(buf, "%d", &ch);
	if(ch==1)
	{
		printk("=====bq25892===enter_ship_mode\n");
		bq2589x_enter_ship_mode();
	}

        return count;
}

static DEVICE_ATTR(enter_ship_mode, S_IRUGO | S_IWUSR | S_IWGRP,NULL, enter_ship_mode);


static ssize_t read_vbus(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
	int vbus=0;
	
	vbus=bq2589x_adc_read_vbus_volt();

        return snprintf(buf, PAGE_SIZE, "%d\n", vbus);
}
static DEVICE_ATTR(chao_vbus, S_IRUGO | S_IWUSR | S_IWGRP, read_vbus, NULL);

static ssize_t read_ichg_main(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int ichg=0;

        ichg=read_bat_ichg(1);

        return snprintf(buf, PAGE_SIZE, "%d\n", ichg);
}
static DEVICE_ATTR(chao_ichg_main, S_IRUGO | S_IWUSR | S_IWGRP, read_ichg_main, NULL);

static ssize_t read_ichg_sec(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int ichg=0;

        ichg=read_bat_ichg(2);

        return snprintf(buf, PAGE_SIZE, "%d\n", ichg);
}
static DEVICE_ATTR(chao_ichg_sec, S_IRUGO | S_IWUSR | S_IWGRP, read_ichg_sec, NULL);


static ssize_t read_temp_main(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int temp=0;
        temp=read_battery_temp(1)/10;
        return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static DEVICE_ATTR(chao_temp_main, S_IRUGO | S_IWUSR | S_IWGRP, read_temp_main, NULL);
static ssize_t read_temp_sec(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int temp=0;
        temp=read_battery_temp(0)/10;

        return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static DEVICE_ATTR(chao_temp_sec, S_IRUGO | S_IWUSR | S_IWGRP, read_temp_sec, NULL);

static ssize_t read_capacity_main(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int capacity=0;

        capacity=read_rsoc(1);

        return snprintf(buf, PAGE_SIZE, "%d\n", capacity);
}
static DEVICE_ATTR(chao_capacity_main, S_IRUGO | S_IWUSR | S_IWGRP, read_capacity_main, NULL);

static ssize_t read_capacity_sec(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int capacity=0;
        
        capacity=read_rsoc(0);

        return snprintf(buf, PAGE_SIZE, "%d\n", capacity);
}
static DEVICE_ATTR(chao_capacity_sec, S_IRUGO | S_IWUSR | S_IWGRP, read_capacity_sec, NULL);

static ssize_t read_volt_main(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int volt=0;

        volt=read_bat(1);

        return snprintf(buf, PAGE_SIZE, "%d\n", volt);
}
static DEVICE_ATTR(chao_volt_main, S_IRUGO | S_IWUSR | S_IWGRP, read_volt_main, NULL);



static ssize_t read_volt_sec(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
        int volt=0;

        volt=read_bat(2);

        return snprintf(buf, PAGE_SIZE, "%d\n", volt);
}
static DEVICE_ATTR(chao_volt_sec, S_IRUGO | S_IWUSR | S_IWGRP, read_volt_sec, NULL);

static ssize_t read_pmic_ver(struct device *dev,struct device_attribute *attr, char *buf)
{
        struct bq2589x *bq = dev_get_drvdata(dev);
	u8  temp=0x00;

       temp=intel_soc_pmic_readb(0x5FC6);  
	
        return snprintf(buf, PAGE_SIZE, "%x\n", temp);
}
static DEVICE_ATTR(pmic_payload_ver, S_IRUGO | S_IWUSR | S_IWGRP, read_pmic_ver, NULL);


static struct attribute *chao_bq2589x_attributes[] = {
        &dev_attr_chao_charger.attr,
        &dev_attr_chao_vbus.attr,
        &dev_attr_chao_ichg_main.attr,
        &dev_attr_chao_ichg_sec.attr,
        &dev_attr_chao_temp_main.attr,
        &dev_attr_chao_temp_sec.attr,
        &dev_attr_chao_capacity_main.attr,
        &dev_attr_chao_capacity_sec.attr,
        &dev_attr_chao_volt_main.attr,
        &dev_attr_chao_volt_sec.attr,
        &dev_attr_pmic_payload_ver.attr,
        &dev_attr_enter_ship_mode.attr,
        &dev_attr_mchg_ce.attr,
        NULL,
};

static const struct attribute_group chao_bq2589x_attr_group = {
        .attrs = chao_bq2589x_attributes,
};

void dump_mainchg_reg(struct bq2589x *bq)
{
        u8 addr;
        u8 val;

        for (addr = 0x0; addr <= 0x14; addr++) {
                bq2589x_read_byte(bq, &val, addr);
                dev_info(bq->dev, "[0x%.2x] = 0x%.2x\n", addr, val);
        }
}
//liulc1 end

static int bq2589x_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq2589x *bq;

	int ret;
	int irqn = 0;

	bq = kzalloc(sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s:out of memory\n", __func__);
		return -ENOMEM;
	}
	

	bq->dev = &client->dev;
	bq->client = client;
	bq->i2c_adp_num = client->adapter->nr;

	ret = bq2589x_detect_device(bq);
	if(ret == 0){
		if(bq->part_no == BQ25890) {
			dev_info(bq->dev, "%s: bq25890 charger detected,revision:%d\n", __func__, bq->revision);
		} else if (bq->part_no == BQ25892) {
			dev_info(bq->dev, "%s: bq25892 charger detected,revision:%d\n", __func__, bq->revision);
		} else if (bq->part_no == BQ25895) {
			dev_info(bq->dev, "%s: bq25895 charger detected,revision:%d\n", __func__, bq->revision);
		} else {
			dev_err(bq->dev, "%s: non-compatible bq_device detected\n", __func__);
			kfree(bq);
			return -ENODEV;
		}
	} else {
		dev_err(bq->dev, "%s: no charger device detected!\n", __func__);
		kfree(bq);
		return -ENODEV;
	}

	if (client->adapter == wcove_pmic_i2c_adapter) {       /* Main charger: PMIC i2c */
		dev_info(&client->dev, "bq2589x probe PMIC I2C:%d(Main).\n", bq->i2c_adp_num);
		g_bq = bq;
		dump_mainchg_reg(g_bq);
		bq2589x_init_main(g_bq);  //liulc1 add
		bq->main_charger_flag = 1;
		wake_lock_init(&bq->wakelock, WAKE_LOCK_SUSPEND, "bq25892_main");
	} else {                                               /* Second charger: i2c0 */
		dev_info(&client->dev, "bq2589x probe I2C:%d(Second).\n", bq->i2c_adp_num);
		bq->main_charger_flag = 0;
		g_bq_sec = bq;
		bq2589x_init_sec(g_bq_sec); //liulc1  add
		wake_lock_init(&bq->wakelock, WAKE_LOCK_SUSPEND, "bq25892_sec");
	}
	bq->platform_data = client->dev.platform_data;
	if(!bq->platform_data){
		dev_err(bq->dev,"%s: Failed to get platform_data!",__func__);
		goto err_0;
	}

	if (0 != bq->platform_data->gpio_otg) {
		ret = gpio_request(bq->platform_data->gpio_otg, "bq2589x otg pin");
		if(ret) {
			dev_err(bq->dev,"%s: %d gpio request failed\n", __func__, bq->platform_data->gpio_otg);
			goto err_1;
		}
		// otg disabled by default
		gpio_direction_output(bq->platform_data->gpio_otg, 0);
	}
	//printk("=========bq25892=======bq2589x_charger_probe   yt3_hw_ver= %d\n",yt3_hw_ver);
	if(bq->main_charger_flag == 0)
	{
		ret = gpio_request(bq->platform_data->gpio_ce, "bq2589x chip-enable pin");
                    if (ret) {
                        	printk("bq2589x_charger_probe  gpio request failed bq->platform_data->gpio_ce\n");
                                goto err_dev;
                        }
                       gpio_direction_output(bq->platform_data->gpio_ce, 1);  


	}

	if (0 != bq->platform_data->gpio_irq) {       /* second charger */
		ret = gpio_request(bq->platform_data->gpio_irq, "bq2589x irq pin");
		if (ret) {
			dev_err(bq->dev,"%s: %d gpio request failed\n", __func__, bq->platform_data->gpio_irq);
			goto err_2;
		}
		gpio_direction_input(bq->platform_data->gpio_irq);

		irqn = gpio_to_irq(bq->platform_data->gpio_irq);
		if(irqn < 0) {
			dev_err(bq->dev,"%s:%d gpio_to_irq failed\n",__func__,irqn);
			ret = irqn;
			goto err_3;
		}
		client->irq = irqn;
		bq->irq = irqn;
		printk("Second bq charger IRQ = %d!\n", bq->irq);    /* 140 */
	} else {                                     /* main charger */
		bq->irq = client->irq;
		printk("Main bq charger IRQ = %d!\n", bq->irq);      /* 468 */
	}

	i2c_set_clientdata(client, bq);
#if 0
	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_3;
	}
#endif
	INIT_WORK(&bq->irq_work, bq2589x_irq_workfunc);
	INIT_DELAYED_WORK(&bq->chrg_full_wrkr, bq2589x_full_worker);
	INIT_DELAYED_WORK(&bq->chrg_task_wrkr, bq2589x_task_worker);
	if(bq->main_charger_flag == 1)
		INIT_DELAYED_WORK(&bq->chrg_boost_wrkr, bq2589x_boost_worker);
	mutex_init(&bq->event_lock);
	init_completion(&bq->vbus_detect);
	
        if(bq->main_charger_flag == 1)
	{
		ret = sysfs_create_group(&bq->dev->kobj, &chao_bq2589x_attr_group);
		if (ret) {
                	dev_err(bq->dev, "failed to chao_charger sysfs. err: %d\n", ret);
        	}

	}
	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_dev;
	}

	if (client->adapter == wcove_pmic_i2c_adapter) {
		ret = request_threaded_irq(client->irq, bq2589x_irq_isr, bq2589x_interrupt, IRQF_TRIGGER_FALLING, "pmic_bq25892", bq);
	} else {
		ret = request_threaded_irq(client->irq, bq2589x_irq_isr, bq2589x_interrupt, IRQF_TRIGGER_FALLING, "bq2589x_sec", bq);
	}
	if (ret) {
		dev_err(bq->dev, "%s:Request IRQ %d failed: %d\n", __func__,client->irq, ret);
		goto err_irq;
	}
	if (0 != bq->platform_data->gpio_ce) {   		/* all set, enable charger */
		gpio_direction_output(bq->platform_data->gpio_ce, 0);
	}

	bq->bat_health = POWER_SUPPLY_HEALTH_GOOD;
	bq->max_cc = 1800;
	bq->max_cv = 4350;
	bq->max_temp = 50;
	bq->min_temp = 0;
	bq2589x_set_vindpm(bq,4500); //liulc1 add 
	/* Init Runtime PM State */
	pm_runtime_put_noidle(&bq->client->dev);
	pm_schedule_suspend(&bq->client->dev, MSEC_PER_SEC);

	ret = bq2589x_psy_register(bq);
	if(ret) {
		goto err_irq;
	}
	if(bq->main_charger_flag == 1)
		ret = bq2589x_register_otg_vbus(bq);
	if (ret) {
		dev_err(&bq->client->dev, "REGISTER OTG VBUS failed!\n");
	}


	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
err_3:
	i2c_set_clientdata(client, NULL);
	if (bq->platform_data->gpio_irq)
		gpio_free(bq->platform_data->gpio_irq);
err_2:
	if (bq->platform_data->gpio_otg)
		gpio_free(bq->platform_data->gpio_otg);
err_1:
	if (bq->platform_data->gpio_ce)
		gpio_free(bq->platform_data->gpio_ce);
err_dev:
//	kfree(bq->platform_data);
err_0:
	bq2589x_psy_unregister(bq);
	kfree(bq);
	g_bq = NULL;
	return ret;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

//	if (!bq->pdata->slave_mode) {
		bq2589x_psy_unregister(bq);
//	}

	if (bq->platform_data->gpio_irq)
		gpio_free(bq->platform_data->gpio_irq);
	if (bq->platform_data->gpio_otg)
		gpio_free(bq->platform_data->gpio_otg);
	if (bq->platform_data->gpio_ce)
		gpio_free(bq->platform_data->gpio_ce);
	free_irq(client->irq, bq);

	i2c_set_clientdata(bq->client, NULL);
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	cancel_work_sync(&bq->irq_work);
//	kfree(bq->platform_data);
	wake_lock_destroy(&bq->wakelock);
	kfree(bq);
	g_bq = NULL;
	return 0;
}

/* interfaces that can be called by other module */
int bq2589x_adc_start(bool oneshot)
{
    u8 val;
    int ret;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
    ret = bq2589x_read_byte(g_bq,&val,BQ2589X_REG_02);
    if(ret < 0){
        dev_err(g_bq->dev,"%s failed to read register 0x02:%d\n",__func__,ret);
        return ret;
    }

    if(((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
        return 0;      //is doing continuous scan
    if(oneshot)
        ret = bq2589x_update_bits(g_bq,BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
    else
        ret = bq2589x_update_bits(g_bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(void)   //stop continue scan 
{
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
    return bq2589x_update_bits(g_bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);


int bq2589x_adc_read_battery_volt(void)
{
    uint8_t val;
    int volt;
    int ret;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_0E);
    if(ret < 0){
        dev_err(g_bq->dev,"read battery voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);

int bq2589x_adc_read_battery_volt_sec(void)
{
    uint8_t val;
    int volt;
    int ret;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
        ret = bq2589x_read_byte(g_bq_sec, &val, BQ2589X_REG_0E);
    if(ret < 0){
        dev_err(g_bq_sec->dev,"read battery voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
        return volt;
    }
}


int bq2589x_adc_read_sys_volt(void)
{
    uint8_t val;
    int volt;
    int ret;
    u8  data;

        if(g_bq==NULL||g_bq_sec==NULL)  return 0;
        bq2589x_read_byte(g_bq,&data,0x02);
        data|=0x80;
        data&=0xbf;
        bq2589x_write_byte(g_bq,0x02,data);
        msleep(200);

	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_0F);
	bq2589x_read_byte(g_bq,&data,0x02);
        data&=0x3f;
        bq2589x_write_byte(g_bq,0x02,data);

    if(ret < 0){
        dev_err(g_bq->dev,"read system voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
	volt+=2304;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_sys_volt_sec(void)
{
    uint8_t val;
    int volt;
    int ret;
    u8  data;

        if(g_bq==NULL||g_bq_sec==NULL)  return 0;
        bq2589x_read_byte(g_bq_sec,&data,0x02);
        data|=0x80;
        data&=0xbf;
        bq2589x_write_byte(g_bq_sec,0x02,data);
        msleep(200);

        ret = bq2589x_read_byte(g_bq_sec, &val, BQ2589X_REG_0F);
        bq2589x_read_byte(g_bq_sec,&data,0x02);
        data&=0x3f;
        bq2589x_write_byte(g_bq_sec,0x02,data);

    if(ret < 0){
        dev_err(g_bq->dev,"read system voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
	volt+=2304;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt_sec);


int bq2589x_adc_read_vbus_volt(void)
{
    uint8_t val;
    int volt;
    int ret;
    u8  data;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
        bq2589x_read_byte(g_bq,&data,0x02);
        data|=0x80;
        data&=0xbf;
        bq2589x_write_byte(g_bq,0x02,data);
        msleep(200);

	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_11);
	bq2589x_read_byte(g_bq,&data,0x02);
        data&=0x3f;
        bq2589x_write_byte(g_bq,0x02,data);

    if(ret < 0){
        dev_err(g_bq->dev,"read vbus voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
	volt+=2600; //liulc1 add
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

//==============liulc1 add===================================

int bq2589x_pumpx_enable(int enable)
{

    u8 val;
    int ret;

    if(enable)
        val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
    else
        val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

    ret = bq2589x_update_bits(g_bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

    return ret;

}

EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(void)
{

    u8 val;
    int ret;

    val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;
    ret = bq2589x_update_bits(g_bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

    return ret;
}

EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);


int bq2589x_pumpx_increase_volt_done(void)
{

    u8 val;
    int ret;

	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_09);
    if(ret) return ret;

    if(val & BQ2589X_PUMPX_UP_MASK) 
        return 1;   // not finished
    else
        return 0;   // pumpx up finished
}

EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);


int bq2589x_pumpx_decrease_volt(void)
{
    u8 val;
    int ret;

    val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;
    ret = bq2589x_update_bits(g_bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

    return ret;   
}

EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);


int bq2589x_pumpx_decrease_volt_done(void)
{

    u8 val;
    int ret;

	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_09);
    if(ret) return ret;
    if(val & BQ2589X_PUMPX_DOWN_MASK) 
        return 1;   // not finished
    else
        return 0;   // pumpx down finished
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

//==================================================
int bq2589x_adc_read_temperature(void)
{
    uint8_t val;
    int temp;
    int ret;
    u8  data;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;

        bq2589x_read_byte(g_bq,&data,0x02);
        data|=0x80;
        data&=0xbf;
        bq2589x_write_byte(g_bq,0x02,data);
        msleep(200);


	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_10);

	bq2589x_read_byte(g_bq,&data,0x02);
        data&=0x3f;
        bq2589x_write_byte(g_bq,0x02,data);

    if(ret < 0){
        dev_err(g_bq->dev,"read temperature failed :%d\n",ret);
        return ret;
    }
    else{
        temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB/1000;
        return temp;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_temperature_sec(void)
{
    uint8_t val;
    int temp;
    int ret;
    u8  data;
        if(g_bq==NULL||g_bq_sec==NULL)  return 0;

        bq2589x_read_byte(g_bq_sec,&data,0x02);
        data|=0x80;
        data&=0xbf;
        bq2589x_write_byte(g_bq_sec,0x02,data);
        msleep(200);

        ret = bq2589x_read_byte(g_bq_sec, &val, BQ2589X_REG_10);
	bq2589x_read_byte(g_bq,&data,0x02);
        data&=0x3f;
        bq2589x_write_byte(g_bq,0x02,data);

    if(ret < 0){
        dev_err(g_bq_sec->dev,"read temperature failed :%d\n",ret);
        return ret;
    }
    else{
        temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB/1000;
        return temp;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature_sec);


int bq2589x_adc_read_charge_current(void)
{
    uint8_t val;
    int volt;
    int ret;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_12);
    if(ret < 0){
        dev_err(g_bq->dev,"read charge current failed :%d\n",ret);
        return ret;
    }
    else{
        volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_adc_read_charge_current_sec(void)
{
    uint8_t val;
    int volt;
    int ret;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
        ret = bq2589x_read_byte(g_bq_sec, &val, BQ2589X_REG_12);
    if(ret < 0){
        dev_err(g_bq_sec->dev,"read charge current failed :%d\n",ret);
        return ret;
    }
    else{
        volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
        return volt;
    }
}

void bq2589x_start_charging(void)
{
	bq2589x_update_charge_params(g_bq);
	bq2589x_enable_chr(g_bq);    // in case of charger enable bit is cleared due to fault
	if (g_bq->platform_data->gpio_ce)
		gpio_direction_output(g_bq->platform_data->gpio_ce, 0);
}
EXPORT_SYMBOL_GPL(bq2589x_start_charging);

void bq2589x_stop_charging(void)
{
	bq2589x_disable_chr(g_bq);   // in case of charger enable bit is cleared due to fault
	if (g_bq->platform_data->gpio_ce)
		gpio_direction_output(g_bq->platform_data->gpio_ce, 1);
}
EXPORT_SYMBOL_GPL(bq2589x_stop_charging);

int bq2589x_force_dpdm(void)
{
	int ret;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(g_bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if(ret) return ret;

	mdelay(10);//TODO: how much time needed to finish dpdm detect?
	return bq2589x_update_charge_params(g_bq);
}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);

int bq2589x_reset_chip(void)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(g_bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	if(ret < 0) return ret;

	mdelay(100);

	bq2589x_init_device(g_bq);
	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_ship_mode(void)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;
	if(g_bq==NULL||g_bq_sec==NULL)  return 0;
//liulc1 add 1029
	bq2589x_reset_wdt_timer(g_bq);
	bq2589x_update_bits(g_bq,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);
	bq2589x_update_bits(g_bq_sec,BQ2589X_REG_00,BQ2589X_ENHIZ_MASK,BQ2589X_HIZ_ENABLE<<BQ2589X_ENHIZ_SHIFT);
	msleep(100);
//liulc1 end
	ret = bq2589x_update_bits(g_bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	ret = bq2589x_update_bits(g_bq_sec, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	return ret;
	//any other work to do?
}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);

int bq2589x_get_hiz_mode(u8* state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(g_bq, &val, BQ2589X_REG_00);
	if (ret) return ret;
	*state = val & BQ2589X_ENHIZ_MASK >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

/* interface for other module end */

static const struct i2c_device_id bq2589x_id[] = {
	{ "bq25890", BQ25890 },
	{ "bq25892", BQ25892 },
	{ "bq25895", BQ25895 },
	{ "bq25892-main",   BQ25892 },
	{ "bq25892-second", BQ25892 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_id);

#ifdef CONFIG_PM
static int bq2589x_suspend(struct device *dev)
{
	struct bq2589x *bq = dev_get_drvdata(dev);

	if (bq->irq > 0) {
		/*
		 * Once the WDT is expired all bq24192 registers gets
		 * set to default which means WDT is programmed to 40s
		 * and if there is no charger connected, no point
		 * feeding the WDT. Since reg07[1] is set to default,
		 * charger will interrupt SOC every 40s which is not
		 * good for S3. In this case we need to free chgr_int_n
		 * interrupt so that no interrupt from charger wakes
		 * up the platform in case of S3. Interrupt will be
		 * re-enabled on charger connect.
		 */
		free_irq(bq->irq, bq);
	}

	//dev_dbg(&bq->client->dev, "bq2589x suspend!\n");
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	int ret;

	if (bq->irq > 0) {
		if (bq->client->adapter == wcove_pmic_i2c_adapter) {
			ret = request_threaded_irq(bq->irq, bq2589x_irq_isr, bq2589x_interrupt, IRQF_TRIGGER_FALLING, "pmic_bq25892", bq);
		} else {
			ret = request_threaded_irq(bq->irq, bq2589x_irq_isr, bq2589x_interrupt, IRQF_TRIGGER_FALLING, "bq2589x_sec", bq);
		}
		if (ret) {
			dev_err(bq->dev, "%s:Request IRQ %d failed: %d!!!\n", __func__, bq->irq, ret);
		} else {
			//dev_warn(bq->dev, "%s:Request IRQ %d success!!!\n", __func__, bq->irq);
		}
	}
	//dev_dbg(&bq->client->dev, "bq2589x resume!\n");
	return 0;
}
#else
#define bq2589x_suspend	NULL
#define bq2589x_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bq2589x_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq2589x_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq2589x_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define bq2589x_runtime_suspend	NULL
#define bq2589x_runtime_resume	NULL
#define bq2589x_runtime_idle	NULL
#endif

static int bq2589x_charger_shutdown(struct i2c_client *client)
{
 struct bq2589x *bq = i2c_get_clientdata(client);
 int ret;

 	if((bq->boost_mode) && (bq->main_charger_flag == 1)) {
 		mutex_lock(&bq->event_lock);
 		ret = bq2589x_turn_otg_vbus(bq, 0);
 		mutex_unlock(&bq->event_lock);
 		dev_err(&bq->client->dev, "disable vbus how!\n");
 		if (ret < 0)
 			dev_err(&bq->client->dev, "VBUS mode 0. failed!\n");
	 	}

 	return 0;
}

static const struct dev_pm_ops bq2589x_pm_ops = {
	.suspend		= bq2589x_suspend,
	.resume			= bq2589x_resume,
	.runtime_suspend	= bq2589x_runtime_suspend,
	.runtime_resume		= bq2589x_runtime_resume,
	.runtime_idle		= bq2589x_runtime_idle,
};

static struct i2c_driver bq2589x_charger = {
	.probe		= bq2589x_charger_probe,
	.remove		= bq2589x_charger_remove,
	.shutdown 	= bq2589x_charger_shutdown,
	.id_table	= bq2589x_id,
	.driver		= {
		.name	= "bq2589x",
		.owner	= THIS_MODULE,
		.pm	= &bq2589x_pm_ops,
//		.of_match_table = bq2589x_match_table,
	},
};
module_i2c_driver(bq2589x_charger);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("How Wang <how.wang@intel.com>");
