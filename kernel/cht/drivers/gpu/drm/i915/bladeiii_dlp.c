/*
 * Copyright © 2006-2009 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *
 */


#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/regulator/consumer.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include <drm/intel_dual_display.h>


#define SII9293_I2C_BUS		4

#define SII9293_RST			280
#define DLP_ON				397
#define DLP_POWER_ON		390

#define PMIC_VSWITCH_REG	0x6E69
#define PMIC_VSWITCH1_EN	0x10

static volatile int bladeiii_dlp_dummy_status;

extern void intel_hdmi_hot_plug(struct intel_encoder *intel_encoder);
//extern bool intel_hpd_irq_event(struct drm_device *dev,
//				struct drm_connector *connector);

extern int dlp3430_init(void);
extern int dlp3430_power_on(void);
extern int dlp3430_power_off(void);
static int deviceid = 0;

static inline int
_exec_i2c_transfer(struct i2c_adapter *adapter, struct i2c_msg *msg, u8 msg_cnt)
{
	int ret = 0;
	u8 retries = 5;

#if 0
	if (msg_cnt == 1) {
		DRM_INFO("%s: slave_add = 0x%x, msg_flags = 0x%x, reg_offset = 0x%x, msg_len = %u, data = 0x%x\n",
			__func__, msg->addr, msg->flags, msg->buf[0], msg->len, msg->buf[1]);
	}
#endif
	do {
		ret =  i2c_transfer(adapter, msg, msg_cnt);
		DRM_DEBUG_KMS("retries = %u, i2c_transfer return %u\n", retries, ret);
		if (ret == msg_cnt) {
			DRM_DEBUG_KMS("%s: i2c_transfer succeed.\n", __func__);
			break;
		} else if (ret == -EAGAIN)
			usleep_range(1000, 2500);
		else if (ret != 1) {
			DRM_ERROR("i2c transfer failed %d\n", ret);
			break;
		}
	} while (--retries);

	if (retries == 0)
		DRM_ERROR("retry i2c transfer failed %d\n", ret);

	usleep_range(1000, 2000);
	return ret;
}

static inline u8
_hdmi_ic_read_reg(struct i2c_adapter *adapter, u8 addr, u8 reg)
{
	struct i2c_msg msg[2];
	u8 buffer = 0;

	msg[0].flags = 0;
	msg[0].addr = addr;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].flags = I2C_M_RD;
	msg[1].addr = addr;
	msg[1].len = 1;
	msg[1].buf = &buffer;

	_exec_i2c_transfer(adapter, msg, 2);

	return buffer;
}

static inline int
_hdmi_ic_write_reg(struct i2c_adapter *adapter, u8 addr, u8 reg, u8 value)
{
	struct i2c_msg msg;
	u8 buffer[3] = {0};

	msg.flags = 0;
	msg.addr = addr;
	msg.len = 2;
	msg.buf = buffer;
	buffer[0] = reg;
	buffer[1] = value;

	return _exec_i2c_transfer(adapter, &msg, 1);
}

static int
bladeiii_dlp_power_init(struct drm_device *dev)
{
	static struct regulator *sii9293_vcc_1v0;
	int ret = 0;
	u8 val = 0;

	/* VSWITCH1 control for 1P8V_DLP_PMU */
	val = intel_soc_pmic_readb(PMIC_VSWITCH_REG);
	DRM_DEBUG_KMS("%s: read PMIC 0x%x = 0x%x.", __func__, PMIC_VSWITCH_REG, val);
	val &= ~PMIC_VSWITCH1_EN;
	intel_soc_pmic_writeb(PMIC_VSWITCH_REG, val);

	sii9293_vcc_1v0 = regulator_get(dev->dev, "VPROG6A");
	if (IS_ERR(sii9293_vcc_1v0)) {
		DRM_ERROR("get regulator VPROG6A failed.\n");
		ret = PTR_ERR(sii9293_vcc_1v0);
		goto out;
	}

	ret = regulator_set_voltage(sii9293_vcc_1v0, 1050000, 1050000);
	if (ret) {
		DRM_ERROR("set VPROG6A voltage failed.\n");
		goto out;
	}

	ret = regulator_enable(sii9293_vcc_1v0);
	if (ret) {
		DRM_ERROR("enable VPROG6A failed.\n");
		goto out;
	}
	msleep(50);

	ret = regulator_disable(sii9293_vcc_1v0);
	if (ret) {
		DRM_ERROR("disable VPROG6A failed.\n");
		goto out;
	}

	DRM_DEBUG_KMS("VPROG6A and VSWITCH1 has already disabled.\n");

out:
	regulator_put(sii9293_vcc_1v0);
	return ret;
}

static int
bladeiii_dlp_power_enable(struct drm_device *dev)
{
	static struct regulator *sii9293_vcc_1v0;
	int ret = 0;
	u8 val = 0;

	/* VSWITCH1 control for 1P8V_DLP_PMU */
	val = intel_soc_pmic_readb(PMIC_VSWITCH_REG);
	DRM_DEBUG_KMS("%s: read PMIC 0x%x = 0x%x.", __func__, PMIC_VSWITCH_REG, val);
	val |= PMIC_VSWITCH1_EN;
	intel_soc_pmic_writeb(PMIC_VSWITCH_REG, val);

	DRM_DEBUG_KMS("try to enable VPROG6A for SII9293_VCC_1V0\n");
	sii9293_vcc_1v0 = regulator_get(dev->dev, "VPROG6A");
	if (IS_ERR(sii9293_vcc_1v0)) {
		DRM_ERROR("get regulator VPROG6A failed.\n");
		ret = PTR_ERR(sii9293_vcc_1v0);
		goto out;
	}

	ret = regulator_set_voltage(sii9293_vcc_1v0, 1050000, 1050000);
	if (ret) {
		DRM_ERROR("set VPROG6A voltage failed.\n");
		goto out;
	}

	ret = regulator_enable(sii9293_vcc_1v0);
	if (ret) {
		DRM_ERROR("enable VPROG6A failed.\n");
		goto out;
	}

	DRM_DEBUG_KMS("VPROG6A for SII9293_VCC_1V0 has already enabled.\n");

out:
	regulator_put(sii9293_vcc_1v0);
	return ret;
}

static int
bladeiii_dlp_power_disable(struct drm_device *dev)
{
	static struct regulator *sii9293_vcc_1v0;
	int ret = 0;
	u8 val = 0;

	/* VSWITCH1 control for 1P8V_DLP_PMU */
	val = intel_soc_pmic_readb(PMIC_VSWITCH_REG);
	DRM_DEBUG_KMS("%s: read PMIC 0x%x = 0x%x.", __func__, PMIC_VSWITCH_REG, val);
	val &= ~PMIC_VSWITCH1_EN;
	intel_soc_pmic_writeb(PMIC_VSWITCH_REG, val);

	DRM_DEBUG_KMS("try to disable VPROG6A for SII9293_VCC_1V0\n");
	sii9293_vcc_1v0 = regulator_get(dev->dev, "VPROG6A");
	if (IS_ERR(sii9293_vcc_1v0)) {
		DRM_ERROR("get regulator VPROG6A failed.\n");
		ret = PTR_ERR(sii9293_vcc_1v0);
		goto out;
	}

	ret = regulator_disable(sii9293_vcc_1v0);
	if (ret) {
		DRM_ERROR("disable VPROG6A failed.\n");
		goto out;
	}

	DRM_DEBUG_KMS("VPROG6A for SII9293_VCC_1V0 has already disabled.\n");

out:
	regulator_put(sii9293_vcc_1v0);
	return ret;
}


static int
bladeiii_dlp_init_gpio(void)
{
	DRM_DEBUG_KMS("%s: enable DLP_POWER_ON Pin.\n", __func__);
	gpio_direction_output(DLP_POWER_ON, 1);
	msleep(50);

	DRM_DEBUG_KMS("%s: enable DLP_ON Pin.\n", __func__);
	gpio_direction_output(DLP_ON, 1);
	msleep(50);

	DRM_DEBUG_KMS("%s: reset SII9293.\n", __func__);
	gpio_direction_output(SII9293_RST, 1);
	msleep(10);
	gpio_direction_output(SII9293_RST, 0);
	msleep(50);
	gpio_direction_output(SII9293_RST, 1);

	/* wait for DLP */
	msleep(1000);
	return 0;
}

static int
bladeiii_dlp_deinit_gpio(void)
{
	DRM_DEBUG_KMS("%s: disable DLP_POWER_ON Pin.\n", __func__);
	gpio_direction_output(DLP_POWER_ON, 0);
	msleep(50);

	DRM_DEBUG_KMS("%s: disable DLP_ON Pin.\n", __func__);
	gpio_direction_output(DLP_ON, 0);
	msleep(50);

	DRM_DEBUG_KMS("%s: disable SII9293 reset.\n", __func__);
	gpio_direction_output(SII9293_RST, 0);
	msleep(10);

	return 0;
}

static int
bladeiii_init_hdmi_ic(struct drm_device *dev)
{
	struct i2c_adapter *adapter = NULL;
	struct i2c_msg msg;
	u8 transmit_buffer[4] = {0};
	int ret = 0;

	DRM_DEBUG_KMS("%s In\n", __func__);

	adapter = i2c_get_adapter(SII9293_I2C_BUS);
	if (!adapter) {
		DRM_ERROR("i2c_get_adapter(%u) failed.\n", SII9293_I2C_BUS);
		ret = -EPERM;
		goto out;
	}

#if 1
	/* get device ID */
	deviceid = _hdmi_ic_read_reg(adapter, (0x64 >> 1), 0x3);
	deviceid = (deviceid << 8) | _hdmi_ic_read_reg(adapter, (0x64 >> 1), 0x2);
	DRM_INFO("%s: get device ID is 0x%x\n", __func__, deviceid);
#endif

	msg.flags = 0;
	msg.buf = transmit_buffer;
	msg.len = 2;

	/* 0x64 */
	msg.addr = 0x64>>1;

	/* HDCP reset */
	transmit_buffer[0] = 0x08;
	transmit_buffer[1] = 0x05;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* Auto audio FIFO reset when audio FIFO error detect */
	transmit_buffer[0] = 0x07;
	transmit_buffer[1] = 0x40;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* switch related */
	transmit_buffer[0] = 0x09;
	transmit_buffer[1] = 0x91;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* HPD control related HPD high */
	transmit_buffer[0] = 0x0B;
	transmit_buffer[1] = 0x01;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x49;
	transmit_buffer[1] = 0x00;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x4A;
	transmit_buffer[1] = 0x00;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x5F;
	transmit_buffer[1] = 0x00;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* 0x68 */
	msg.addr = 0x68>>1;
	transmit_buffer[0] = 0x3D;
	transmit_buffer[1] = 0x01;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x27;
	transmit_buffer[1] = 0xF9;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x26;
	transmit_buffer[1] = 0x40;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x29;
	transmit_buffer[1] = 0x04;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* 0x64 */
	msg.addr = 0x64>>1;
	transmit_buffer[0] = 0x05;
	transmit_buffer[1] = 0x01;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x05;
	transmit_buffer[1] = 0x00;
	_exec_i2c_transfer(adapter, &msg, 1);

	/* 0xD0 */
	msg.addr = 0xD0>>1;
	transmit_buffer[0] = 0x6C;
	transmit_buffer[1] = 0x3F;
	_exec_i2c_transfer(adapter, &msg, 1);

	transmit_buffer[0] = 0x70;
	transmit_buffer[1] = 0xC8;
	_exec_i2c_transfer(adapter, &msg, 1);

out:
	return 0;
}

/*
 * Called in intel_hdmi_init()
 */
int bladeiii_dlp_init(struct drm_device *dev)
{
	int ret = 0;

	DRM_DEBUG_KMS("%s: init bladeiii_dlp_dummy_status to 0\n", __func__);
	bladeiii_dlp_dummy_status = 0;

	/* request gpio */
	ret = gpio_request(DLP_POWER_ON, "DLP_POWER_ON");
	if (ret < 0) {
		DRM_ERROR("gpio_request %d failed. ret = %d\n", DLP_POWER_ON, ret);
	}

	ret = gpio_request(DLP_ON, "DLP_ON");
	if (ret < 0) {
		DRM_ERROR("gpio_request %d failed. ret = %d\n", DLP_ON, ret);
	}

	ret = gpio_request(SII9293_RST, "SII9293_RST");
	if (ret < 0) {
		DRM_ERROR("gpio_request %d failed. ret = %d\n", SII9293_RST, ret);
	}

	bladeiii_dlp_deinit_gpio();
	bladeiii_dlp_power_init(dev);

	return 0;
}

/*
 * Called when request to turn on DLP.
 */
int bladeiii_dlp_power_on(struct drm_device *dev)
{
	DRM_DEBUG_KMS("Open DLP.");
	bladeiii_dlp_dummy_status = 1;

	bladeiii_dlp_power_enable(dev);
	bladeiii_dlp_init_gpio();
	bladeiii_init_hdmi_ic(dev);

	dlp3430_power_on();

	return 0;
}

/*
 * Called in the delayed_work when request to turn on DLP. (intel_hdmi_hot_plug_delayed_work())
 */
int bladeiii_dlp_open(struct drm_device *dev)
{
	dlp3430_init();

	return 0;
}

/*
 * Called when request to close DLP.
 */
int bladeiii_dlp_close(struct drm_device *dev)
{
	DRM_DEBUG_KMS("Close DLP.");
       if(bladeiii_dlp_dummy_status==0)
            return 0;
	dlp3430_power_off();
	bladeiii_dlp_deinit_gpio();
	bladeiii_dlp_power_disable(dev);

	bladeiii_dlp_dummy_status = 0;

	return 0;
}
/*
int bladeiii_dlp_dummy_hotplug(struct drm_device *dev)
{
	struct intel_connector *intel_connector;
	struct intel_encoder *intel_encoder;
	struct drm_connector *connector;

	DRM_INFO("%s: send DLP dummy hotplug event.", __func__);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->connector_type == DRM_MODE_CONNECTOR_HDMIA) {
			DRM_DEBUG_KMS("%s: get HDMI connector\n", __func__);

			intel_connector = to_intel_connector(connector);
			intel_encoder = intel_connector->encoder;

			intel_hdmi_hot_plug(intel_encoder);
			intel_hpd_irq_event(dev, connector);
			drm_kms_helper_hotplug_event(dev);
		}
	}

	return 0;
}
*/
int bladeiii_get_sii9293_deviceId(void)
{
	DRM_INFO("%s In\n", __func__);
	if (bladeiii_dlp_dummy_status == 0) {
            DRM_INFO("%s dlp not on\n", __func__);
            return -1;
        }
    DRM_INFO("%s ,deviceid:%x\n", __func__, deviceid);
	return deviceid;
}

int bladeiii_get_dummy_status(void)
{
	return bladeiii_dlp_dummy_status;
}
