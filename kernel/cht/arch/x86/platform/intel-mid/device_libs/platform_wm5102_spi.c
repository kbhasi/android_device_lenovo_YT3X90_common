/*
 * platform_wm5102.c: wm51020 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/platform_sst_audio.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_device.h>
#include <linux/sfi.h>
#include <linux/spi/spi.h>
#include <asm/intel-mid.h>
#include "platform_wm5102.h"
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>

#define CODEC_GPIO_BASE			10


static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  100, .key = KEY_MEDIA },
	{ .max =  290 , .key = KEY_VOLUMEUP },
	{ .max =  680 , .key = KEY_VOLUMEDOWN },
};

static struct arizona_micd_config wm5102_micd_config[]={
	{ 0        , 1 , 0 },
	{ ARIZONA_ACCDET_SRC	     ,  2  ,1},	
};

static struct arizona_pdata wm5102_pdata  = {
	.reset = 496,  
	.ldoena = 495,  
	.clk32k_src = ARIZONA_32KZ_MCLK2,
	.irq_gpio=505,
	.irq_flags = IRQF_TRIGGER_LOW  | IRQF_ONESHOT,
	.gpio_base = CODEC_GPIO_BASE,
	.micd_pol_gpio = CODEC_GPIO_BASE + 2,
        .jd_wake_time = 10,
	.init_mic_delay = 30,
	.micd_detect_debounce=500,
	.micd_bias_start_time = 1,
       .micd_rate = 6,
       .micd_configs= wm5102_micd_config,
        .num_micd_configs=ARRAY_SIZE(wm5102_micd_config),
        .micd_ranges=micd_ctp_ranges,
        .num_micd_ranges=ARRAY_SIZE(micd_ctp_ranges),
        .micd_force_micbias=1,
        .inmode = {
            [0]= ARIZONA_INMODE_DIFF, /*IN1L for Headset*/
            [1]= ARIZONA_INMODE_DIFF,
            [2]= ARIZONA_INMODE_DIFF,
         },
        .micbias={
           [0]={ /*MICBIAS1*/
                 .mV =2800 ,
                 .ext_cap=1,
                 .discharge =1 ,
                 .soft_start =0,             
                 .bypass =0,
           },
	   [1]={ /*MICBIAS2*/
                 .mV =2800 ,
                 .ext_cap=1,
                 .discharge =1 ,
                 .soft_start =0,          
                 .bypass =0,
           },
	   [2]={ /*MICBIAS3*/
                 .mV =2800 ,
                 .ext_cap=1,
                 .discharge =1 ,
                 .soft_start =0,                
                 .bypass =0,
           },
        },
};

static struct regulator_consumer_supply dc1v8_consumers[] = {

	REGULATOR_SUPPLY("AVDD", "spi1.0"), /* wm5102 */
	REGULATOR_SUPPLY("DBVDD1", "spi1.0"), /* wm5102 */
	REGULATOR_SUPPLY("LDOVDD", "spi1.0"),
	
	REGULATOR_SUPPLY("DBVDD2", "wm5102-codec"),
	REGULATOR_SUPPLY("DBVDD3", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDL", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "spi1.0"),
};
static struct regulator_init_data dc1v8_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(dc1v8_consumers),
	.consumer_supplies = dc1v8_consumers,
};

static struct fixed_voltage_config dc1v8vdd_pdata = {
	.supply_name = "DC_1V8",
	.microvolts = 1800000,
	.init_data = &dc1v8_data,
	.gpio = -1,
};

static struct platform_device dc1v8_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data = &dc1v8vdd_pdata,
	},
};

static struct spi_board_info __initdata wm5102_board_info = {
	.platform_data = &wm5102_pdata,
};


static void byt_audio_register_spi_dev(void)
{
//    struct spi_board_info spi_info;

    memset(&wm5102_board_info, 0, sizeof(wm5102_board_info));
    strncpy(wm5102_board_info.modalias, "wm5102", sizeof("wm5102"));
    wm5102_board_info.irq = 0 ;
    wm5102_board_info.bus_num = 1;
    wm5102_board_info.chip_select = 0;
    wm5102_board_info.max_speed_hz = 5000000;   
    wm5102_board_info.platform_data = &wm5102_pdata;
    
    pr_info("SPI bus=%d, name=%16.16s, irq=0x%2x, max_freq=%d, cs=%d, pdata=0x%x\n",
                wm5102_board_info.bus_num,
                wm5102_board_info.modalias,
                wm5102_board_info.irq,
                wm5102_board_info.max_speed_hz,
                wm5102_board_info.chip_select,
		wm5102_board_info.platform_data);

    spi_register_board_info(&wm5102_board_info, 1);
}


static int __init byt_audio_platform_init(void)
{
	struct platform_device *pdev;
	int ret;

	pr_info("in %s\n", __func__);


	platform_device_register(&dc1v8_device);

	byt_audio_register_spi_dev();

	return 0;
}
device_initcall(byt_audio_platform_init);


