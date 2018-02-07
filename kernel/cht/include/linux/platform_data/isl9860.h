/*
 * LP8755 High Performance Power Management Unit Driver:System Interface Driver
 *
 *			Copyright (C) 2012 Texas Instruments
 *
 * Author: Daniel(Geon Si) Jeong <daniel.jeong@ti.com>
 *             G.Shark Jeong <gshark.jeong@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _ISL9860_H
#define _ISL9860_H

#include <linux/regulator/consumer.h>

#define ISL9860_NAME "isl9860-regulator"
extern struct isl9860 *isl9860data;
int isl9860_ext_write_byte(u8 reg, u8 data);
int isl9860_ext_read_byte(u8 reg);
struct isl9860_rom_data {
	u8 addr;
	u8 val;
};

/**
 * struct lp855x_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @device_control : value of DEVICE CONTROL register
 * @initial_brightness : initial value of backlight brightness
 * @period_ns : platform specific pwm period value. unit is nano.
		Only valid when mode is PWM_BASED.
 * @size_program : total size of lp855x_rom_data
 * @rom_data : list of new eeprom/eprom registers
 */
struct isl9860_platform_data {
	const char *name;
	u8 device_control;
	u8 initial_brightness;
	unsigned int period_ns;
	int size_program;
	struct isl9860_rom_data *rom_data;
};
#endif
