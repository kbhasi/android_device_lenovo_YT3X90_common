/*
 * platform_ov5693.c: ov5693 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/atomisp_gmin_platform.h>

#include "atomisp_gmin_pmic_regs.h"
#include "platform_ov5693.h"

#include <linux/vlv2_plat_clock.h>

//#define __USE_REGULATOR__


/* workround - pin defined for cht */
#define CAMERA_1_RESET	(341+148-98) /* CAMERASB10 */
#define CAMERA_1_PWDN	(341+152-98) /* CAMERASB07 */
#ifdef __DVDD_GPIO_MODE__
#define CAMERA_1P2_EN	147 /* CAMERASB06*/
#endif



/* Should be defined in vlv2_plat_clock API, isn't: */
#define VLV2_CLK_19P2MHZ 0
#define VLV2_CLK_ON      1
#define VLV2_CLK_OFF     2
#define OSC_CAM1_CLK 1
#define OSC_CAM0_CLK 0



static int camera_vprog1_on = 0;
static int camera_reset = -1;
static int camera_power_down = -1;
#ifdef __DVDD_GPIO_MODE__
static int camera_1p2_en = -1;
#endif

#ifdef __USE_REGULATOR__
static struct regulator *vprog4d_reg = NULL;
static struct regulator *v1p8sx_reg = NULL;
static struct regulator *v1p2a_reg = NULL;
#endif
/*
 * OV5693 platform data
 */
static int log_enable = 0;

#define OV5693_PLAT_LOG(a, ...) \
       do { \
               if (log_enable) \
                       printk(a,## __VA_ARGS__); \
       } while (0)


 
static void ov5693_verify_gpio_power(void)
{
 #if 0
       OV5693_PLAT_LOG("CAMERA: start check ov5693 gpio\n");
       OV5693_PLAT_LOG("CAMERA_0_RESET: %d %x\n", camera_reset, gpio_get_value(camera_reset));
       OV5693_PLAT_LOG("CAMERA_0_PWDN: %d %x\n", camera_power_down, gpio_get_value(camera_power_down));
 #ifdef __DVDD_GPIO_MODE__
       OV5693_PLAT_LOG("CAMERA_DVDD_EN: %d %x \n", camera_1p2_en, gpio_get_value(camera_1p2_en));
 #else
       OV5693_PLAT_LOG("VPROG4D reg:%x %x\r\n", VPROG_1P2A,intel_soc_pmic_readb(VPROG_1P2A));
       OV5693_PLAT_LOG("VPROG4D_SEL reg:%x %x\r\n", VPROG_1P2A_VSEL,intel_soc_pmic_readb(VPROG_1P2A_VSEL));
 #endif
       OV5693_PLAT_LOG("VPROG4D reg:%x %x\r\n", VPROG4D,intel_soc_pmic_readb(VPROG4D));
       OV5693_PLAT_LOG("VPROG4D_SEL reg:%x %x\r\n", VPROG4D_VSEL,intel_soc_pmic_readb(VPROG4D_VSEL));
       //OV5693_PLAT_LOG("VPROG5B reg:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG5B));
       //OV5693_PLAT_LOG("VPROG5B_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG5B_VSEL));
       OV5693_PLAT_LOG("VPROG1P8 reg:%x %x\r\n", VPROG_1P8V, intel_soc_pmic_readb(VPROG_1P8V));
       //OV5693_PLAT_LOG("VPROG2P8 reg:%x %x\r\n", VPROG_2P8V, intel_soc_pmic_readb(VPROG_2P8V));
 #endif

       return;
}

static int ov5693_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	if (flag)
		gpio_direction_output(camera_power_down, 1);
	else
		gpio_direction_output(camera_power_down, 0);

	usleep_range(1000, 1500);
	return 0;
}

/*
 * WORKAROUND:
 * This func will return 0 since MCLK is enabled by BIOS
 * and will be always on event if set MCLK failed here.
 * TODO: REMOVE WORKAROUND, err should be returned when
 * set MCLK failed.
 */
static int ov5693_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
        int ret = 0;

        OV5693_PLAT_LOG("return directly %s %d new clock flag=%d VLV2_CLK_19P2MHZ:%d\r\n", __func__, __LINE__, flag, VLV2_CLK_19P2MHZ);

#if 1
		
        if (flag)
                ret = vlv2_plat_set_clock_freq(OSC_CAM1_CLK, VLV2_CLK_19P2MHZ);
        if (ret)
                return ret;
		/*Disable CAMERA 0 clock as bug in ACPI table*/
		vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
        return vlv2_plat_configure_clock(OSC_CAM1_CLK,
                                         flag ? VLV2_CLK_ON : VLV2_CLK_OFF);

#endif
}
#ifdef __USE_REGULATOR__
/*power control use regulator_xxxx*/
static int ov5693_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
#ifdef __DVDD_GPIO_MODE__
	int pin = CAMERA_1P2_EN;
#endif
        OV5693_PLAT_LOG("%s %d flag=%d\r\n", __func__, __LINE__, flag);
	camera_reset = CAMERA_1_RESET;
#ifdef __DVDD_GPIO_MODE__
	camera_1p2_en = pin;
#endif

	if (flag) {
		if (!camera_vprog1_on) {
			/*case 4*/
			/*1. AVDD rising first*/
                        OV5693_PLAT_LOG("%s %d AVDD Rising \r\n", __func__, __LINE__);
			if (vprog4d_reg) {
				ret = regulator_enable(vprog4d_reg);
				if (ret)
                                        OV5693_PLAT_LOG("platform ov5693 enable v1p8sx failed\r\n");
				usleep_range(1000, 1500);
			} else {
                                OV5693_PLAT_LOG("%s %d platform ov5693 vprog4d_reg NULL  \r\n", __func__, __LINE__);
			}
		
			/*2. Xshut down rising*/
                        OV5693_PLAT_LOG("%s %d XSHUTDWN Rising\r\n", __func__, __LINE__);
			ret = gpio_direction_output(camera_reset, 1);
			if (ret)
                                OV5693_PLAT_LOG("%s %d reset rising fail\r\n", __func__, __LINE__);
			usleep_range(1000, 1500);
			
			/*3. DOVDD rising*/
                        OV5693_PLAT_LOG("%s %d DOVDD Rising\r\n", __func__, __LINE__);
			if (v1p8sx_reg) {
				ret = regulator_enable(v1p8sx_reg);
				if (ret)
                                        OV5693_PLAT_LOG("platform ov5693 enable v1p8sx failed\r\n");
			} else {
                               OV5693_PLAT_LOG("platform ov5693 v1p8sx_reg NULL  %s %d\r\n", __func__, __LINE__);
			}
			usleep_range(1000, 1500);
			
			/*4. enable DVDD 1.2v power */
                        OV5693_PLAT_LOG("%s %d DVDD Rising\r\n", __func__, __LINE__);
#ifdef __DVDD_GPIO_MODE__
			ret = gpio_direction_output(camera_1p2_en, 1); /*DVDD enable*/
			if (ret) {
				pr_err("platform ov5693 %s: failed to set gpio(pin %d) direction\n",
					__func__, camera_1p2_en);

				return ret;
			}
#else
			if (v1p2a_reg) {
			ret = regulator_enable(v1p2a_reg);
			if (ret)
                            OV5693_PLAT_LOG("platform ov5693 enable v1p2a_reg failed\r\n");
			} else {
                            OV5693_PLAT_LOG("platform ov5693 v1p2a_reg NULL  %s %d\r\n", __func__, __LINE__);
			}
#endif
			camera_vprog1_on = 1;
			usleep_range(10000, 11000);
		}
	} else {
		if (camera_vprog1_on) {
			/*1. disable DVDD 1.2v power */
                        OV5693_PLAT_LOG("%s %d DVDD OFF\r\n", __func__, __LINE__);
#ifdef __DVDD_GPIO_MODE__
			ret = gpio_direction_output(camera_1p2_en, 0);
			if (ret)
                                OV5693_PLAT_LOG("%s %d power off DVDD failed\r\n", __func__, __LINE__);
#else
			if (v1p2a_reg) {
				ret = regulator_disable(v1p2a_reg);
				if (ret)
                                        OV5693_PLAT_LOG("platform ov5693 disable v1p2a failed\r\n");
				usleep_range(1000, 1500);
			} else {
                                OV5693_PLAT_LOG("platform ov5693 regulator 1p2a is NULL\r\n");
			}
#endif
			usleep_range(1000, 1500);
			
			/*2 disable DOVDD*/
                        OV5693_PLAT_LOG("%s %d DOVDD OFF\r\n", __func__, __LINE__)
			if (v1p8sx_reg) {
				ret = regulator_disable(v1p8sx_reg);
				if (ret)
                                    OV5693_PLAT_LOG("platform ov5693 disable v1p8sx failed\r\n");
				usleep_range(1000, 1500);
			} else {
                               OV5693_PLAT_LOG("platform ov5693 regulator 1p8sx is NULL\r\n");
			}
			
			/*3. xShutdown falling*/
                        OV5693_PLAT_LOG("%s %d XSHUTDOWN OFF\r\n", __func__, __LINE__);
			ret = gpio_direction_output(camera_reset, 0);
			if (ret)
                                OV5693_PLAT_LOG("%s %d reset rising fail\r\n", __func__, __LINE__);
			usleep_range(1000, 1500);			

			/*4. disable AVDD */
                        OV5693_PLAT_LOG("%s %d AVDD OFF\r\n", __func__, __LINE__);
			if (vprog4d_reg) {
				ret = regulator_disable(vprog4d_reg);
				if (ret)
                                        OV5693_PLAT_LOG("platform ov5693 disable vprog4d failed\r\n");
			} else {
                               OV5693_PLAT_LOG("platform ov5693 regulator vprog4d is NULL\r\n");

			}
			usleep_range(1000, 1500);
			camera_vprog1_on = 0;
                        OV5693_PLAT_LOG("-------------power OFF check start-----------\r\n");
			ov5693_verify_gpio_power();
                        OV5693_PLAT_LOG("-------------power OFF check end-----------\r\n");
		}
	}

	return 0;
}

#else
/*power control use camera_set_pmic_power*/
static int ov5693_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
#ifdef __DVDD_GPIO_MODE__
	int pin = CAMERA_1P2_EN;
#endif
        OV5693_PLAT_LOG("%s %d flag=%d\r\n", __func__, __LINE__, flag);      
#ifdef __DVDD_GPIO_MODE__
	camera_1p2_en = pin;
#endif

	if (flag) {
		if (!camera_vprog1_on) {
			/*case 4*/
			/*1. AVDD rising*/
                        OV5693_PLAT_LOG("%s %d AVDD Rising\r\n", __func__, __LINE__);
			ret = camera_set_pmic_power(CAMERA_VPROG4D, true); /* AVDD rising first*/
			if (ret)
                                OV5693_PLAT_LOG("platform ov5693 enable v1p8sx failed\r\n");
			usleep_range(1000, 1500);
			
			/*2. Xshut down rising*/
                        OV5693_PLAT_LOG("%s %d XSHUTDOWN Rising\r\n", __func__, __LINE__);
			ret = gpio_direction_output(camera_reset, 1);
			if (ret)
                                OV5693_PLAT_LOG("%s %d reset rising fail\r\n", __func__, __LINE__);
			usleep_range(1000, 1500);		
			
			/*3. DOVDD rising*/
                        OV5693_PLAT_LOG("%s %d DOVDD Rising\r\n", __func__, __LINE__);
			ret = camera_set_pmic_power(CAMERA_1P8V, true); /*DOVDD on*/
			if (ret)
                                OV5693_PLAT_LOG("platform ov5693 enable vprog4d failed\r\n");
			usleep_range(1000, 1500);
			
			/*4. enable DVDD 1.2v power */
                        OV5693_PLAT_LOG("%s %d DVDD Rising\r\n", __func__, __LINE__);
#ifdef __DVDD_GPIO_MODE__
			ret = gpio_direction_output(camera_1p2_en, 1); /*DVDD enable*/
			if (ret) {
				pr_err("platform ov5693 %s: failed to set gpio(camera_1p2_en %d) direction\n",
					__func__, camera_1p2_en);

				return ret;
			}
#else
			ret = camera_set_pmic_power(CAMERA_1P2A, true); 	/* DVDD drop */
#endif
			usleep_range(1000, 1500);
			camera_vprog1_on = 1;
			usleep_range(10000, 11000);
		}
	} else {
		if (camera_vprog1_on) {
			/*1. disable DVDD 1.2v power */
                        OV5693_PLAT_LOG("%s %d DVDD OFF\r\n", __func__, __LINE__);
#ifdef __DVDD_GPIO_MODE__
			ret = gpio_direction_output(camera_1p2_en, 0);
			if (ret)
                                OV5693_PLAT_LOG("%s %d power off DVDD failed\r\n", __func__, __LINE__);
#else
			ret = camera_set_pmic_power(CAMERA_1P2A, false); 	/* DVDD drop */
#endif
			usleep_range(1000, 1500);
			
			/*2. disable DOVDD*/
                        OV5693_PLAT_LOG("%s %d DOVDD OFF\r\n", __func__, __LINE__);
			ret = camera_set_pmic_power(CAMERA_1P8V, false); 	/* DOVDD drop */
			if (ret)
                                OV5693_PLAT_LOG("platform ov5693 disable v1p8sx failed\r\n");
			usleep_range(1000, 1500);
			
			/*3. xShutdown falling*/
                        OV5693_PLAT_LOG("%s %d xShutdown OFF\r\n", __func__, __LINE__);
			ret = gpio_direction_output(camera_reset, 0);
			if (ret)
                                OV5693_PLAT_LOG("%s %d reset rising fail\r\n", __func__, __LINE__);
			usleep_range(1000, 1500);
			
			/*4. AVDD falling*/
                        OV5693_PLAT_LOG("%s %d AVDD OFF\r\n", __func__, __LINE__);
			ret = camera_set_pmic_power(CAMERA_VPROG4D, false); 		/* AVDD drop last*/
			if (ret)
                                OV5693_PLAT_LOG("platform ov5693 disable v1p8sx failed\r\n");
			usleep_range(1000, 1500);
			
			/*workaround to turnoff v2p8sx*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			ret = camera_set_pmic_power(CAMERA_1P8V, false);

			camera_vprog1_on = 0;
                        OV5693_PLAT_LOG("-------------power OFF check start-----------\r\n");
			ov5693_verify_gpio_power();
                         OV5693_PLAT_LOG("-------------power OFF check end-----------\r\n");
		}
	}

	return 0;
}
#endif

static int ov5693_csi_configure(struct v4l2_subdev *sd, int flag)
{
        OV5693_PLAT_LOG("%s %d flag=%d\r\n", __func__, __LINE__, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov5693_platform_init(struct i2c_client *client)
{
	int ret;
        OV5693_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
	ret = gpio_request(CAMERA_1_PWDN, "camera_1_pwdn");
	if (ret) {
		pr_err("platform ov5693 %s: failed to request PWDN gpio(pin %d) keep going\n",
			__func__, CAMERA_1_PWDN);
	}
	camera_power_down = CAMERA_1_PWDN;
	ret = gpio_direction_output(camera_power_down, 0);
	if (ret) {
			pr_err("%s shut down camera camera_power_down failed", __func__);
	}

	ret = gpio_request(CAMERA_1_RESET, "camera_1_reset");
	if (ret) {
		pr_err("platform ov5693 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, CAMERA_1_RESET);
	}
	camera_reset = CAMERA_1_RESET;
	ret = gpio_direction_output(camera_reset, 0);
	if (ret) {
			pr_err("%s shut down camera camera_reset failed", __func__);
	}
#ifdef __DVDD_GPIO_MODE__
	ret = gpio_request(CAMERA_1P2_EN, "camera_1_reset");
	if (ret) {
		pr_err("platform ov5693 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, CAMERA_1P2_EN);
	}
	camera_1p2_en = CAMERA_1P2_EN;
	ret = gpio_direction_output(camera_1p2_en, 0);
	if (ret) {
			pr_err("%s shut down camera camera_1p2_en failed", __func__);
	}
#endif

#ifdef __USE_REGULATOR__
	v1p2a_reg = regulator_get(&client->dev, "V1P8SX");
	if (IS_ERR(v1p2a_reg)) {
		dev_err(&client->dev, "v1p8sx regulator_get failed\n");
		return PTR_ERR(v1p2a_reg);
	}

	v1p8sx_reg = regulator_get(&client->dev, "V1P8SX");
	if (IS_ERR(v1p8sx_reg)) {
		dev_err(&client->dev, "v1p8sx regulator_get failed\n");
		return PTR_ERR(v1p8sx_reg);
	}

	vprog4d_reg = regulator_get(&client->dev, "VPROG4D");
	if (IS_ERR(vprog4d_reg)) {
		regulator_put(vprog4d_reg);
		dev_err(&client->dev, "VPROG4D regulator_get failed\n");
		return PTR_ERR(vprog4d_reg);
	}
#endif

	/*workaround to turnoff v2p8sx*/
	ret = camera_set_pmic_power(CAMERA_2P8V, false);
	ret = camera_set_pmic_power(CAMERA_1P8V, false);
	vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
	vlv2_plat_configure_clock(OSC_CAM1_CLK, VLV2_CLK_OFF); 
	return 0;
}

static int ov5693_platform_deinit(void)
{
        OV5693_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
#ifdef __USE_REGULATOR__
	if (v1p2a_reg)
		regulator_put(v1p2a_reg);
	if (v1p8sx_reg)
		regulator_put(v1p8sx_reg);
	if (vprog4d_reg)
		regulator_put(vprog4d_reg);
#endif
	return 0;
}


static struct camera_sensor_platform_data ov5693_sensor_platform_data = {
	.gpio_ctrl	= ov5693_gpio_ctrl,
	.flisclk_ctrl	= ov5693_flisclk_ctrl,
	.power_ctrl	= ov5693_power_ctrl,
	.csi_cfg	= ov5693_csi_configure,
	.platform_init = ov5693_platform_init,
	.platform_deinit = ov5693_platform_deinit,
};

void *ov5693_platform_data(void *info)
{
	printk("%s %d\r\n", __func__, __LINE__);
	camera_reset = -1;
#ifdef __USE_REGULATOR__
	camera_1p2_en = -1;
#endif
	return &ov5693_sensor_platform_data;
}

EXPORT_SYMBOL_GPL(ov5693_platform_data);

