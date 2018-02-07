/*
 * platform_camera_regulator.c: camera regulator platform device
 *  initilization file
 *
 * (C) Copyright 2011 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/acpi.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/platform_device.h>
#include <linux/regulator/intel_whiskey_cove_pmic.h>

static struct regulator_consumer_supply v1p8sx_consumer[] = {
	REGULATOR_SUPPLY("v1p8sx", "INT33BE:00"),
	REGULATOR_SUPPLY("v1p8sx", "INT33FB:00"),
};

static struct regulator_consumer_supply v2p8sx_consumer[] = {
	REGULATOR_SUPPLY("v2p8sx", "INT33BE:00"),
	REGULATOR_SUPPLY("v2p8sx", "INT33FB:00"),
};

/* v1p8sx regulator */
static struct regulator_init_data v1p8sx_data = {
	.constraints = {
		.min_uV = 1620000,
		.max_uV = 1980000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p8sx_consumer),
	.consumer_supplies	= v1p8sx_consumer,
};

/* v2p8sx regulator */
static struct regulator_init_data v2p8sx_data = {
	.constraints = {
		.min_uV			= 2565000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v2p8sx_consumer),
	.consumer_supplies	= v2p8sx_consumer,
};


/*************************************************************
*
* WCOVE Camera related regulator
*
*************************************************************/
static struct regulator_init_data wcove_v1p8sx_data;
static struct regulator_init_data wcove_v2p8sx_data;

static struct wcove_regulator_info wcove_v1p8sx_info = {
	.init_data = &wcove_v1p8sx_data,
};

static struct wcove_regulator_info wcove_v2p8sx_info = {
	.init_data = &wcove_v2p8sx_data,
};

static void intel_setup_whiskey_cove_camera_regulators(void)
{
	memcpy((void *)&wcove_v1p8sx_data, (void *)&v1p8sx_data,
			sizeof(struct regulator_init_data));
	memcpy((void *)&wcove_v2p8sx_data, (void *)&v2p8sx_data,
			sizeof(struct regulator_init_data));

	/* register SD card regulator for whiskey cove PMIC */
	intel_soc_pmic_set_pdata("wcove_regulator", &wcove_v1p8sx_info,
		sizeof(struct wcove_regulator_info), WCOVE_ID_V1P8SX + 1);
	intel_soc_pmic_set_pdata("wcove_regulator", &wcove_v2p8sx_info,
		sizeof(struct wcove_regulator_info), WCOVE_ID_V2P8SX + 1);
}

static int __init camera_regulator_init(void)
{
	intel_setup_whiskey_cove_camera_regulators();

	return 0;
}
fs_initcall_sync(camera_regulator_init);
