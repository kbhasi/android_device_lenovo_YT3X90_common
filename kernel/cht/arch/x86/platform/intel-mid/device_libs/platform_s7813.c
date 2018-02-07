/*
 * platform_ekth3250.c: ekth3250 platform data initilization file
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
#include "platform_s7813.h"
#include <linux/input/synaptics_dsx.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/input/synaptics_dsx.h>
#include <linux/i2c.h>



#define TM1940 (1)
#define TM2448 (2)

#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define	gpio_southeast_NUM	86

#define	gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define	gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base		(gpio_east_base - gpio_southeast_NUM)

#define	GPIO_ALERT	77
#define FST_SPI_CS2_B  7
#define SYNAPTICS_MODULE TM2448

static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state);

#define SYNAPTICS_I2C_DEVICE
#define DSX_I2C_ADDR 0x20
#define DSX_ATTN_GPIO (gpio_southeast_base + GPIO_ALERT)
#define DSX_ATTN_MUX_NAME "gpmc_ad15.gpio_39"
#define DSX_POWER_GPIO -1
#define DSX_POWER_MUX_NAME "mcspi1_cs3.gpio_140"
#define DSX_POWER_ON_STATE 1
#define DSX_POWER_DELAY_MS 160
#define DSX_RESET_GPIO (FST_SPI_CS2_B + gpio_southwest_base)
#define DSX_RESET_ON_STATE 0
#define DSX_RESET_DELAY_MS 100
#define DSX_RESET_ACTIVE_MS 20
#define DSX_IRQ_FLAGS IRQF_TRIGGER_FALLING
//static unsigned char regulator_name[] = "";
static unsigned int cap_button_codes[] = {};
static unsigned int vir_button_codes[] = {};
static unsigned char pwr_reg_name[] = "";
static unsigned char bus_reg_name[] = "";

static struct synaptics_dsx_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};

static struct synaptics_dsx_button_map vir_button_map = {
	.nbuttons = ARRAY_SIZE(vir_button_codes) / 5,
	.map = vir_button_codes,
};
static struct synaptics_dsx_board_data dsx_board_data = {
	.irq_gpio = DSX_ATTN_GPIO,
	//.irq_flags = DSX_IRQ_FLAGS,
	//.irq_flags = IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING,
	.irq_flags = IRQF_ONESHOT | IRQ_TYPE_LEVEL_LOW,
	.power_gpio = DSX_POWER_GPIO,
	.power_on_state = DSX_POWER_ON_STATE,
	.power_delay_ms = DSX_POWER_DELAY_MS,
	.reset_gpio = DSX_RESET_GPIO,
	.reset_on_state = DSX_RESET_ON_STATE,
	.reset_delay_ms = DSX_RESET_DELAY_MS,
	.reset_active_ms = DSX_RESET_ACTIVE_MS,
	.max_y_for_2d = -1,
	.ub_i2c_addr = -1,
	//.gpio_config = synaptics_gpio_setup,
	//.regulator_name = regulator_name,
	.pwr_reg_name = pwr_reg_name,
	.bus_reg_name = bus_reg_name,
	.cap_button_map = &cap_button_map,
	.vir_button_map = &vir_button_map,
#ifdef SYNAPTICS_SPI_DEVICE
	.byte_delay_us = DSX_SPI_BYTE_DELAY_US,
	.block_delay_us = DSX_SPI_BLOCK_DELAY_US,
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_10
	.swap_axes = 1,
	.x_flip = 1,
	.y_flip = 0,
#endif
};


void *s7813_platform_data(void *info)
{
#if 0
        static struct synaptics_dsx_platform_data dsx_platformdata = {
	    //.irq_flags = IRQF_TRIGGER_FALLING | IRQCHIP_ONESHOT_SAFE,
	    .irq_type = IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING,    
	    .irq_gpio = DSX_ATTN_GPIO,
	    .reset_delay_ms = 100,
	    .reset_gpio = DSX_RESET_GPIO,
 	    .gpio_config = synaptics_gpio_setup,
 	    .cap_button_map = &tm2448_cap_button_map,
	};
	return &dsx_platformdata;
#endif 

	return &dsx_board_data;

}

static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	printk("%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, configure, dir, state);

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		
	        if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}



#define VPROG3B         (0x9B)  
#define VPROG3B_VSEL    (0xCB)   //3V0  /*VOUT =  0.25 + 61 * 0.05 = 3.3V*/  
#define VPROG5A         (0xA0)  
#define VPROG5A_VSEL    (0xD0)   //1V8  /*VOUT =  0.25 + 31 * 0.05 = 1.8V*/

#define VPROG_ENABLE 0x2
#define VPROG_DISABLE 0x0

void touch_set_pmic_power(bool flag)
{
   u8 reg_value[2] = {VPROG_DISABLE, VPROG_ENABLE};

   intel_soc_pmic_writeb(VPROG3B_VSEL, 61);         
   intel_soc_pmic_writeb(VPROG5A_VSEL, 31);    


   intel_soc_pmic_writeb(VPROG3B, reg_value[flag]);
   intel_soc_pmic_writeb(VPROG5A, reg_value[flag]);

   printk("%s:enble V3P3 and V1P8 for touch\n", __func__);

}
EXPORT_SYMBOL_GPL(touch_set_pmic_power);
static int __init synaptics_platform_init(void)
{
    int i2c_busnum = 5;
    struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    void *pdata = NULL;
	printk(KERN_ERR "synaptics_platform_init\n");

    //intel_scu_ipc_msic_vprog3(1);

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, I2C_DRIVER_NAME, strlen(I2C_DRIVER_NAME));

    i2c_info.addr = 0x38;

    printk(KERN_ERR "synaptics_platform_init I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    pdata = s7813_platform_data(&i2c_info);

    if(pdata != NULL)
        i2c_info.platform_data = pdata;
    else
        printk(KERN_ERR "%s, pdata is NULL\n", __func__);
#if 1
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
		if(i2c_new_device(adapter,&i2c_info)){
			printk(KERN_ERR "add new i2c device %s , addr 0x%x\n", I2C_DRIVER_NAME,i2c_info.addr);
			return 0;
		}else{
			printk(KERN_ERR "add new i2c device %s , addr 0x%x fail !!!\n", I2C_DRIVER_NAME,i2c_info.addr);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail\n",__func__,i2c_busnum);
		return -EINVAL;
	}
#else
    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
#endif
}

device_initcall(synaptics_platform_init);

