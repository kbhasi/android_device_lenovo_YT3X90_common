/*
 * platform_lp5521.c: lp5521 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/i2c.h>
#include <linux/platform_data/leds-lp55xx.h>

#define LED_DYNAMIC_I2C
#define LED_LP5521_NAME			"lp5521"

#define gpio_southwest_NUM      98
#define gpio_north_NUM  73
#define gpio_east_NUM   27
#define gpio_southeast_NUM      86

#define gpio_southwest_base     (ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base         (gpio_southwest_base - gpio_north_NUM)
#define gpio_east_base          (gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base             (gpio_east_base - gpio_southeast_NUM)

#define LP5521_EN_GPIO			(GP_CAMERASB_02+gpio_north_base)
#define GP_CAMERASB_02  		46


static struct lp55xx_led_config lp5521_led[] = {
       [0] = {
	       .name ="red",
	       .chan_nr = 0,
	       .led_current = 50,	//default 5mA
	       .max_current = 200,	//max 20mA
       },
       [1] = {
           .name = "green",
	       .chan_nr = 1,
	       .led_current = 50,
	       .max_current = 200,
       },
       [2] = {
	       .name = "blue",
	       .chan_nr = 2,
	       .led_current = 50,
	       .max_current = 200,
       },
};

static struct lp55xx_platform_data __initdata lp5521_platform_data = {
       .label = "lp5521_led",
       .led_config     = &lp5521_led[0],
       .num_channels   = 3,
       .clock_mode     = LP55XX_CLOCK_EXT,
		.enable_gpio = LP5521_EN_GPIO,
};

static int __init lp5521_platform_init(void)
{
#ifdef LED_DYNAMIC_I2C
    int i2c_busnum = 1;
#else
	int i2c_busnum = 2;
#endif
    struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    void *pdata = NULL;
	printk(KERN_ERR "lp5521_platform_init\n");

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, LED_LP5521_NAME, strlen(LED_LP5521_NAME));

    i2c_info.addr = 0x32;

    printk(KERN_ERR "lp5521_platform_init I2C bus = %d, name = %16.16s, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.addr);

    i2c_info.platform_data = &lp5521_platform_data;
#ifdef LED_DYNAMIC_I2C
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
		if(i2c_new_device(adapter,&i2c_info)){
			printk(KERN_ERR "add new i2c device %s , addr 0x%x\n", LED_LP5521_NAME,i2c_info.addr);
			return 0;
		}else{
			printk(KERN_ERR "add new i2c device %s , addr 0x%x fail !!!\n", LED_LP5521_NAME,i2c_info.addr);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail\n",__func__,i2c_busnum);
		return -EINVAL;
	}
#else
    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
#endif
}

#ifdef LED_DYNAMIC_I2C
device_initcall(lp5521_platform_init);
#else
fs_initcall(lp5521_platform_init);
#endif

