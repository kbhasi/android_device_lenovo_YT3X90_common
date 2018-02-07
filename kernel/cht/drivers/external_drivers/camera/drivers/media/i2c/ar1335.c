/*
 * Support for OmniVision ar1335 8MP camera sensor.
 *
 * Copyright (c) 2011 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <media/v4l2-device.h>
#include <linux/videodev2.h>
#include <asm/intel-mid.h>
#include <linux/firmware.h>
#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/atomisp_gmin_platform.h>
#include "ar1335.h"
#include <linux/mfd/intel_soc_pmic.h>

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include "ar1335_bld_otp.c"
#define AR1335_BIN_FACTOR_MAX	2
#define AR1335_OTP_INPUT_NAME "ar1335_otp.bin"

#define to_ar1335_sensor(sd) container_of(sd, struct ar1335_device, sd)

#define DEBUG_VERBOSE	(1<<0)
#define DEBUG_GAIN_EXP	(1<<1)
#define DEBUG_INTG_FACT	(1<<2)
#define DEBUG_OTP	(1<<4)
#define DEBUG_DISABLE_GAIN_EXP (1<<5)
static unsigned int debug = 0x00;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define AR1335_DEFAULT_LOG_LEVEL 10
//#define AR1335_DEFAULT_LOG_LEVEL 1
static unsigned int log_level = AR1335_DEFAULT_LOG_LEVEL;
module_param(log_level, int, 0644);

#define AR1335_LOG(level, a, ...) \
	do { \
		if (level < log_level) \
			printk(a,## __VA_ARGS__); \
	} while (0)


struct ar1335_device * global_dev;
static unsigned int ctrl_value;
static int dw9761_vcm_ctrl(const char *val, struct kernel_param *kp);
static int dw9761_t_focus_abs(struct v4l2_subdev *sd, s32 value);

module_param_call(vcm_ctrl, dw9761_vcm_ctrl, param_get_uint,
				&ctrl_value, S_IRUGO | S_IWUSR);

static int ar1335_raw_size;
static int ar1335_otp_size;
static unsigned char ar1335_raw[DATA_BUF_SIZE];
static unsigned char ar1335_otp_data[DATA_BUF_SIZE];

static int op_dump_otp;
static int ar1335_dump_otp(const char *val, struct kernel_param *kp);
module_param_call(dump_otp, ar1335_dump_otp, param_get_uint,
				&op_dump_otp, S_IRUGO | S_IWUSR);

//#define GAIN_1024
//#define GAIN_512
#define GAIN_256

typedef struct {
    int gain_value;
    int reg_value;
}gain_reg_pair_t;

static void get_combination_gains(int gain, int *again, int *dgain, u16 * reg);

static int ar1335_dump_otp(const char *val, struct kernel_param *kp)
{
	int ret;
        ret = DW9761B_eflash_otp_save(ar1335_raw, ar1335_raw_size, AR1335_SAVE_RAW_DATA);
	if(ret != 0)
		OTP_LOG("Fail to save ar1335 RAW data\n");

        ret = DW9761B_eflash_otp_save(ar1335_otp_data, ar1335_otp_size, AR1335_SAVE_OTP_DATA);
	if(ret != 0)
		OTP_LOG("Fail to save ar1335 OTP data\n");

	return 0;
}

static int
ar1335_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[AR1335_SHORT_MAX];
	int err, i;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > AR1335_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0, sizeof(msg));
	memset(data, 0, sizeof(data));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == AR1335_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int ar1335_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == num_msg)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_err(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		//msleep(20);
		mdelay(1);
		goto again;
	}

	return ret;
}

static int
ar1335_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != AR1335_8BIT && data_length != AR1335_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == AR1335_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* AR1335_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu(val);
	}

	ret = ar1335_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/*
 * ar1335_write_reg_array - Initializes a list of MT9M114 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ar1335_flush_reg_array, __ar1335_buf_reg_array() and
 * __ar1335_write_reg_is_consecutive() are internal functions to
 * ar1335_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ar1335_flush_reg_array(struct i2c_client *client,
				     struct ar1335_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ar1335_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ar1335_buf_reg_array(struct i2c_client *client,
				   struct ar1335_write_ctrl *ctrl,
				   const struct ar1335_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case AR1335_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case AR1335_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		AR1335_LOG(2, "next type:%d\r\n", next->type);
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= AR1335_MAX_WRITE_BUF_SIZE)
		__ar1335_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ar1335_write_reg_is_consecutive(struct i2c_client *client,
				   struct ar1335_write_ctrl *ctrl,
				   const struct ar1335_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ar1335_write_reg_array(struct i2c_client *client,
				   const struct ar1335_reg *reglist)
{
	const struct ar1335_reg *next = reglist;
	struct ar1335_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != AR1335_TOK_TERM; next++) {
		switch (next->type & AR1335_TOK_MASK) {
		case AR1335_TOK_DELAY:
			err = __ar1335_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ar1335_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ar1335_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ar1335_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ar1335_flush_reg_array(client, &ctrl);
}


static int dw9761_write8(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9761_device *dev = to_dw9761_device(sd);
	struct i2c_msg msg;
	int ret;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DW9761_I2C_ADDR;
	msg.len = 2;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if(ret == 1)
		return 0;

	return ret;
}


static int dw9761_write16(struct v4l2_subdev *sd, int reg, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9761_device *dev = to_dw9761_device(sd);
	struct i2c_msg msg;
	int ret;

	memset(&msg, 0 , sizeof(msg));
	msg.addr = DW9761_I2C_ADDR;
	msg.len = 3;
	msg.buf = dev->buffer;
	msg.buf[0] = reg;
	msg.buf[1] = val >> 8;
	msg.buf[2] = val & 0xFF;

	ret  = i2c_transfer(client->adapter, &msg, 1);

	if(ret == 1)
		return 0;

	return ret;
}

static int dw9761_read8(struct v4l2_subdev *sd, int reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct dw9761_device *dev = to_dw9761_device(sd);
	struct i2c_msg msg[2];
	int r;

	memset(msg, 0 , sizeof(msg));
	msg[0].addr = DW9761_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = dev->buffer;
	msg[0].buf[0] = reg;

	msg[1].addr = DW9761_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = dev->buffer;

	r = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (r != ARRAY_SIZE(msg))
		return -EIO;

	return dev->buffer[0];
}
static int dw9761_busy_check(struct v4l2_subdev *sd)
{
	int wait_loop=50;
	char busy_status=0;
	int ret_val=0;
	while(wait_loop--)
	{
		busy_status = dw9761_read8(sd, DW9761_STATUS);
		if(busy_status & 0x01)
		{
			usleep_range(10,20);
			continue;
		}
		else
			break;
	}
	if(wait_loop==0)
		ret_val=-1;

	return ret_val;
}

static int dw9761_power_up(struct v4l2_subdev *sd)
{
	struct dw9761_device *dev = to_dw9761_device(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int r;

	r = dw9761_read8(sd, DW9761_INFO);
	AR1335_LOG(2,"dw9761 info read:%x\n", r);
	if (r != 0xf4)
		return r;

	r = dw9761_write8(sd, DW9761_MODE, 0x01);
	if (r)
		return r;

	r = dw9761_write8(sd, DW9761_VCM_FREQ, 0x020);
	if (r)
		return r;

	r = dw9761_write8(sd, DW9761_VCM_PRELOAD, 0x00);
	if (r)
		return r;

	r = dw9761_write8(sd, DW9761_CONTROL, 0x00);
	if (r)
		return r;

	dev->initialized = true;
	v4l2_info(client, "detected dw9761\n");
	return 0;
}

static int dw9761_power_down(struct v4l2_subdev *sd)
{
	int r;

	dw9761_busy_check(sd);
	r = dw9761_write8(sd, DW9761_CONTROL, 0x01);
	if (r)
		return r;

	return 0;
}

static void dw9761_dump_regs(struct v4l2_subdev *sd)
{
	int i,r;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);

	for (i = 0; i < 8; i ++) {
		r = dw9761_read8(sd, i);
		AR1335_LOG(2,"%s %d reg:%d data:0x%x\n", __func__, __LINE__, i, r);
	}
}

static int dw9761_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct dw9761_device *dev = to_dw9761_device(sd);
	int r;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	if (!dev->initialized)
		return -ENODEV;

	value = clamp(value, 0, DW9761_MAX_FOCUS_POS);
	r = dw9761_write16(sd, DW9761_VCM_CURRENT, value);
	if (r < 0)
		return r;

	getnstimeofday(&dev->focus_time);
	dev->focus = value;

	return 0;
}


static int dw9761_vcm_ctrl(const char *val, struct kernel_param *kp)
{
	int ret;
	int rv = param_set_int(val, kp);
	struct i2c_client *client = v4l2_get_subdevdata(&(global_dev->sd));
	int again, dgain;
	u16 reg;

	AR1335_LOG(2,"%s %d\n", __func__, __LINE__);

	if (rv)
		return rv;
		/* Enable power */
	switch ((ctrl_value >>16) & 0x0ffff) {
		case 0:
			AR1335_LOG(2,"group hold\r\n");
			ret = ar1335_write_reg(client, AR1335_8BIT,
					AR1335_GROUP_ACCESS,
					AR1335_GROUP_ACCESS_HOLD_START);			
			break;
		case 1:
			AR1335_LOG(2,"group update\r\n");
			ret = ar1335_write_reg(client, AR1335_8BIT,
					AR1335_GROUP_ACCESS,
					AR1335_GROUP_ACCESS_HOLD_END);	
			break;
		case 2:
			AR1335_LOG(2,"update gain\r\n");
			ret = ar1335_write_reg(client, AR1335_16BIT,
					AR1335_AGC_ADJ,
					ctrl_value & 0xffff);	
			break;
		case 3:
			AR1335_LOG(2,"update integration time\r\n");
			ret = ar1335_write_reg(client, AR1335_16BIT,
					AR1335_LONG_EXPO,
					ctrl_value & 0xffff);				
			break;
		case 4:
			AR1335_LOG(2,"stream off\r\n");
			get_combination_gains(ctrl_value&0xffff, &again, &dgain, &reg); 

			AR1335_LOG(2,"gain:%x again:%x, dgain:%x, reg:%x\r\n", ctrl_value&0xffff, again, dgain, reg);

			break;
		case 5:
			AR1335_LOG(2,"read register\r\n");
			dw9761_dump_regs(&(global_dev->sd));
			break;
		case 0x10:
			AR1335_LOG(2,"VCM regulator on\r\n");
			intel_soc_pmic_writeb(0x9A, 2);
			AR1335_LOG(2,"values:%x", intel_soc_pmic_readb(0x9A));
			break;
		case 0x11:
			AR1335_LOG(2,"VCM regulator off\r\n");
			intel_soc_pmic_writeb(0x9A, 0);
			AR1335_LOG(2,"values:%x", intel_soc_pmic_readb(0x9A));
			break;
		case 0x12:
			{
				int ver = 0;
				AR1335_LOG(2,"read register\r\n");
				dw9761_busy_check(&(global_dev->sd));
				ver = dw9761_read8(&(global_dev->sd), DW9761_INFO);
				AR1335_LOG(2,"ver:%x\r\n", ver);
			}
			break;
		case 0x13:
			AR1335_LOG(2,"set vcm: %x\r\n", ctrl_value & 0xffff);
			dw9761_write16(&(global_dev->sd), DW9761_VCM_CURRENT, ctrl_value & 0xffff);
			break;
               case 0x14:
                       {
                               int i;
                               u16 val = 0;
                               reg = ctrl_value & 0xffff;
                               for (i = 0; i < 32; i++) {
                                       DW9761B_eflash_read_reg(client, AR1335_8BIT, reg+i, &val);
                                       AR1335_LOG(2,"%s %d reg:%x val:%x  %c\n", __func__, __LINE__, reg + i, val, val);
                               }
                       }
                       break;
               case 0x15:
                       ret = DW9761B_eflash_otp_read(client, ar1335_raw, &ar1335_raw_size);
                       if(!ret)
                               AR1335_LOG(2,"DW9761B flash read OK\n");
                       else
                               AR1335_LOG(2,"DW9761B flash read Failed\n");
                       break;
	}
	AR1335_LOG(2,"%s %d enable power----\n", __func__, __LINE__);
	
	
	return 0;
}


static int ar1335_g_priv_int_data(struct v4l2_subdev *sd,
				  struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;
	int i;
	u8 * pdata;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	if (dev->otp_data == NULL) {
		dev_err(&client->dev, "OTP data not available");
		return -1;
	}
	/* Correct read_size value only if bigger than maximum */
	if (read_size > DATA_BUF_SIZE)
		read_size = DATA_BUF_SIZE;

	pdata = (u8 *) dev->otp_data;

	for (i = 0;i < 10; i ++) AR1335_LOG(2,"%d %x\n", i, pdata[i]);

	AR1335_LOG(2,"%s %d read:%d\n", __func__, __LINE__, read_size);
	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		dev_err(&client->dev, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}
out:

	/* Return correct size */
	priv->size = DATA_BUF_SIZE;
	AR1335_LOG(2,"%s %d qurry size:%d\n", __func__, __LINE__, priv->size);

	return 0;
}

static int __ar1335_get_max_fps_index(
				const struct ar1335_fps_setting *fps_settings)
{
	int i;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (fps_settings[i].fps == 0)
			break;
	}

	return i - 1;
}
#if 0
static int __ar1335_update_frame_timing(struct v4l2_subdev *sd, int exposure,
			u16 *hts, u16 *vts)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/* Increase the VTS to match exposure + 14 */
	if (exposure > *vts - AR1335_INTEGRATION_TIME_MARGIN)
		*vts = (u16) exposure + AR1335_INTEGRATION_TIME_MARGIN;

	ret = ar1335_write_reg(client, AR1335_16BIT, AR1335_TIMING_HTS, *hts);
	if (ret)
		return ret;

	return ar1335_write_reg(client, AR1335_16BIT, AR1335_TIMING_VTS, *vts);
}
#endif

#ifdef GAIN_1024
#define GAIN_HAL_1X 1024
static const gain_reg_pair_t gain_pair[] ={
    {1024, 0x10}, {1088, 0x11}, {1152, 0x12}, {1216, 0x13}, {1280, 0x14}, {1344, 0x15}, {1408, 0x16}, {1472, 0x17},
    {1536, 0x18}, {1600, 0x19}, {1664, 0x1a}, {1728, 0x1b}, {1792, 0x1c}, {1856, 0x1d}, {1920, 0x1e}, {1984, 0x1f},
    {2048, 0x20}, {2176, 0x21}, {2304, 0x22}, {2432, 0x23}, {2560, 0x24}, {2688, 0x25}, {2816, 0x26}, {2944, 0x27},
    {3072, 0x28}, {3200, 0x29}, {3328, 0x2a}, {3456, 0x2b}, {3584, 0x2c}, {3712, 0x2d}, {3840, 0x2e}, {3968, 0x2f},
    {4096, 0x30}, {4352, 0x31}, {4608, 0x32}, {4864, 0x33}, {5120, 0x34}, {5376, 0x35}, {5632, 0x36}, {5888, 0x37},
    {6144, 0x38}, {6400, 0x39}, {6656, 0x3a}, {6912, 0x3b}, {7168, 0x3c}, {7424, 0x3d}, {7680, 0x3e}, {7936, 0x3f},
};
#endif

#ifdef GAIN_512
#define GAIN_HAL_1X 512
static const gain_reg_pair_t gain_pair[] ={
    { 512, 0x10}, { 544, 0x11}, { 576, 0x12}, { 608, 0x13}, { 640, 0x14}, { 672, 0x15}, { 704, 0x16}, { 736, 0x17},
    { 768, 0x18}, { 800, 0x19}, { 832, 0x1a}, { 864, 0x1b}, { 896, 0x1c}, { 928, 0x1d}, { 960, 0x1e}, { 992, 0x1f},
    {1024, 0x20}, {1088, 0x21}, {1152, 0x22}, {1216, 0x23}, {1280, 0x24}, {1344, 0x25}, {1408, 0x26}, {1472, 0x27},
    {1536, 0x28}, {1600, 0x29}, {1664, 0x2a}, {1728, 0x2b}, {1792, 0x2c}, {1856, 0x2d}, {1920, 0x2e}, {1984, 0x2f},
    {2048, 0x30}, {2176, 0x31}, {2304, 0x32}, {2432, 0x33}, {2560, 0x34}, {2688, 0x35}, {2816, 0x36}, {2944, 0x37},
    {3072, 0x38}, {3200, 0x39}, {3328, 0x3a}, {3456, 0x3b}, {3584, 0x3c}, {3712, 0x3d}, {3840, 0x3e}, {3968, 0x3f},
};
#endif

#ifdef GAIN_256
#define GAIN_HAL_1X 256
static const gain_reg_pair_t gain_pair[] ={
    { 256, 0x10}, { 272, 0x11}, { 288, 0x12}, { 304, 0x13}, { 320, 0x14}, { 336, 0x15}, { 352, 0x16}, { 368, 0x17},
    { 384, 0x18}, { 400, 0x19}, { 416, 0x1a}, { 432, 0x1b}, { 448, 0x1c}, { 464, 0x1d}, { 480, 0x1e}, { 496, 0x1f},
    { 512, 0x20}, { 544, 0x21}, { 576, 0x22}, { 608, 0x23}, { 640, 0x24}, { 672, 0x25}, { 704, 0x26}, { 736, 0x27},
    { 768, 0x28}, { 800, 0x29}, { 832, 0x2a}, { 864, 0x2b}, { 896, 0x2c}, { 928, 0x2d}, { 960, 0x2e}, { 992, 0x2f},
    {1024, 0x30}, {1088, 0x31}, {1152, 0x32}, {1216, 0x33}, {1280, 0x34}, {1344, 0x35}, {1408, 0x36}, {1472, 0x37},
    {1536, 0x38}, {1600, 0x39}, {1664, 0x3a}, {1728, 0x3b}, {1792, 0x3c}, {1856, 0x3d}, {1920, 0x3e}, {1984, 0x3f},
};
#endif

#define GAIN_PAIR_SIZE (sizeof(gain_pair)/sizeof(gain_reg_pair_t))
static int search_again(int again)
{
	int i;

	if (again < gain_pair[1].gain_value)
		return 0;
    for (i = 1; i < GAIN_PAIR_SIZE; i++) {
		if (again < gain_pair[i].gain_value)
			return i - 1;
	}

	return GAIN_PAIR_SIZE-1;
}

static void get_combination_gains(int gain, int *again, int *dgain, u16 * reg)
{
	int again_value, dgain_value;
	int index;
	index = search_again(gain);

	again_value = gain_pair[index].gain_value;
	*again = again_value;
	dgain_value = 0x1ff & (((gain << 6) + (again_value >> 1))/ again_value);
	AR1335_LOG(2, "org:%x gain<<6: %x again_value:%d\n", gain, ((gain << 6) + (again_value >> 1)), again_value);
	*dgain = dgain_value;

	AR1335_LOG(2, "search:%d reg:%x temp:%x dgain:%x\n", search_again(gain),
			gain_pair[search_again(gain)].reg_value,
			((gain <<13 )/ (*again)), *dgain);
	*reg   = gain_pair[index].reg_value | (dgain_value << 7);

}
static void get_gain_reg(int a_gain, int d_gain, unsigned short * reg)
{
	int again, dgain;

	if (a_gain <= gain_pair[GAIN_PAIR_SIZE - 1].gain_value) {
                AR1335_LOG(2, "using again*dgain a_gain:%d d_gain:%d result:%d\n",
                        a_gain, d_gain, (a_gain * d_gain) >> 8);
                a_gain = (a_gain * d_gain) >> 8;
		get_combination_gains(a_gain, &again, &dgain, reg);
		AR1335_LOG(2, "a-gain: %d d-gain:%d again:%d dgain:%d, reg:%x\n", a_gain, d_gain, again, dgain, *reg);
	} else {
		AR1335_LOG(2, "using again fixed to maximum, only dgain change\n");
		dgain = 0x1ff & ((d_gain << 6) / GAIN_HAL_1X);
		again = gain_pair[GAIN_PAIR_SIZE - 1].gain_value;
		*reg = (gain_pair[GAIN_PAIR_SIZE - 1].reg_value) | (dgain << 7);
		AR1335_LOG(2, "a-gain:%d d-gain:%d again:%d dgain:%d reg:%x\n", a_gain, d_gain, again, dgain, *reg);
	}
}

unsigned char group_start[] = {0x01, 0x04, 0x01};
unsigned char exposure_data[] = {0x02, 0x02, 0x0a, 0x11};
unsigned char gain_data[] = {0x30, 0x5e, 0x20, 0x10};
unsigned char group_end[] = {0x01, 0x04, 0x00};
struct i2c_msg exp_msg[4];

static int ar1335_set_exposure(struct v4l2_subdev *sd, int exposure, int gain,
				int dig_gain)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg;
	int ret;


	/* Validate exposure:  cannot exceed 16bit value */
	exposure = clamp_t(int, exposure, 0, AR1335_MAX_EXPOSURE_VALUE);

	/* Validate gain: must not exceed maximum 8bit value */
	gain = clamp_t(int, gain, 0, AR1335_MAX_GAIN_VALUE);

        get_gain_reg(gain, dig_gain, &reg);
        exp_msg[0].addr = client->addr;
        exp_msg[0].flags = 0;
        exp_msg[0].len = sizeof(group_start);
        exp_msg[0].buf = group_start;

        exposure_data[2] = (exposure >> 8) & 0xff;
        exposure_data[3] = exposure & 0xff;

        exp_msg[1].addr = client->addr;
        exp_msg[1].flags = 0;
        exp_msg[1].len = sizeof(exposure_data);
        exp_msg[1].buf = exposure_data;
      
        gain_data[2] = (reg >> 8) & 0xff;
        gain_data[3] = reg & 0xff;
       
        exp_msg[2].addr = client->addr;
        exp_msg[2].flags = 0;
        exp_msg[2].len = sizeof(gain_data);
        exp_msg[2].buf = gain_data;

        exp_msg[3].addr = client->addr;
        exp_msg[3].flags = 0;
        exp_msg[3].len = sizeof(group_end);
        exp_msg[3].buf = group_end;
        ret = i2c_transfer(client->adapter, exp_msg, 4);

        if (ret == 4)
                ret = 0;
        else ret = -1;

	/* Updated the device variable. These are the current values. */
	dev->gain = gain;
        dev->digital_gain = dig_gain;
	dev->exposure = exposure;
	


	if (debug & DEBUG_GAIN_EXP) {
		u16 val_exp, val_again;

		AR1335_LOG(2,"%s %d exposure:%d(0x%x) gain:%d(8x%x)\n", __func__,
			 __LINE__, exposure, exposure, gain, gain);
		ar1335_read_reg(client, AR1335_16BIT, AR1335_LONG_EXPO, &val_exp);
		AR1335_LOG(2,"%s %d exposure(dec):%d\n", __func__,
			 __LINE__, val_exp);

		ar1335_read_reg(client, AR1335_16BIT, AR1335_AGC_ADJ, &val_again);

		AR1335_LOG(2,"%s %d val_again:(dec):%d\n", __func__,
			__LINE__, val_again);

	}

	return ret;
}

static int ar1335_s_exposure(struct v4l2_subdev *sd,
			      struct atomisp_exposure *exposure)
{
       int ret = 0;
       struct ar1335_device *dev = to_ar1335_sensor(sd);

       if (debug & DEBUG_DISABLE_GAIN_EXP) {
               AR1335_LOG(2,"gain and exposure setting is disabled\n");
               return 0;
       } else {
               mutex_lock(&dev->input_lock);
               ret = ar1335_set_exposure(sd, exposure->integration_time[0],
                                exposure->gain[0], exposure->gain[1]);
               mutex_unlock(&dev->input_lock);

       }
       return ret;
}

static long ar1335_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ar1335_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ar1335_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ar1335_init_registers(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	dev->basic_settings_list = NULL;
	ar1335_write_reg_array(client, ar1335_Reset);
	ar1335_write_reg_array(client, corrections_recommended);
	ar1335_write_reg_array(client, pixel_timing_recommended);
	ar1335_write_reg_array(client, analog_setup_recommended);
	ar1335_write_reg_array(client, mipi_timing_990M);
	ar1335_write_reg_array(client, pll_setup_990M);
	ar1335_write_reg_array(client, array_setup_4208x3120_12fps);
	ar1335_write_reg_array(client, Defect_correction);
	ar1335_write_reg_array(client, LSC_off);
	ar1335_write_reg_array(client, Stop_streaming);
	return 0;
}

static int ar1335_init(struct v4l2_subdev *sd, u32 val)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = ar1335_init_registers(sd);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static void ar1335_uninit(struct v4l2_subdev *sd)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	dev->exposure = 0;
	dev->gain     = 0;
	dev->digital_gain = 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	int ret;

	AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
	/* Enable power */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;


	/* Enable clock */
	AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
	/* Release reset */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	/* Minumum delay is 8192 clock cycles before first i2c transaction,
	 * which is 1.37 ms at the lowest allowed clock rate 6 MHz */
	msleep(20);
	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int __ar1335_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
//	int times = 0;

	if (on == 0) {
		AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
		ar1335_uninit(sd);
		ret = dw9761_power_down(sd);
                if (ret != 0)
                       AR1335_LOG(2,"disable VCM failed, %s %d\r\n", __func__, __LINE__);
		ret = power_down(sd);

		dev->power = 0;
	} else {
		AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
		ret = power_up(sd);
		AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
		if (ret)
			return ret;
		/* shunyong: disable VCM for PO */
		ret = dw9761_power_up(sd);
		if (ret) {
			power_down(sd);
			return ret;
		}

		ret = ar1335_init_registers(sd);
		AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
		if (ret) {
			power_down(sd);
			return ret;
		}		
		dev->power = 1;
	}

	return ret;
}

static int ar1335_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	/*power off first as ACPI has bug, when front camera power on
	it will turn on back camera power*/
	if (on) {
		AR1335_LOG(2, "turn off power for WA\r\n");
		__ar1335_s_power(sd, 0);
	}

	AR1335_LOG(2, "%s %d\r\n", __func__, __LINE__);
	mutex_lock(&dev->input_lock);
	ret = __ar1335_s_power(sd, on);
	mutex_unlock(&dev->input_lock);

	/*
	 * FIXME: Compatibility with old behaviour: return to preview
	 * when the device is power cycled.
	 */
	if (!ret && on)
		v4l2_ctrl_s_ctrl(dev->run_mode, ATOMISP_RUN_MODE_PREVIEW);

	return ret;
}


static int ar1335_get_intg_factor(struct v4l2_subdev *sd,
				  struct camera_mipi_info *info,
				  const struct ar1335_reg *reglist)
{
	/*shunyong: disable get_intg for PO*/
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const int ext_clk = 19200000; /* MHz */
	struct atomisp_sensor_mode_data *m = &info->data;
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	const struct ar1335_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];
	u16 pll_multiplier;
	u16 pre_pll_clk_div1;
	u16 vt_sys_clk_div;
	u16 vt_pix_clk_div;
	u16 row_speed;
	u16 val;

	int ret;

	
	memset(&info->data, 0, sizeof(info->data));

	ar1335_read_reg(client, AR1335_16BIT, PLL_MULTIPLIER1, &pll_multiplier);
	pll_multiplier &= 0xff;
	ar1335_read_reg(client, AR1335_16BIT, PRE_PLL_CLK_DIV1, &pre_pll_clk_div1);
	pre_pll_clk_div1 &= 0x3f;
	ar1335_read_reg(client, AR1335_16BIT, VT_SYS_CLK_DIV, &vt_sys_clk_div);
	ar1335_read_reg(client, AR1335_16BIT, VT_PIX_CLK_DIV, &vt_pix_clk_div);
	ar1335_read_reg(client, AR1335_16BIT, ROW_SPEED, &row_speed);
	row_speed &= 0x07;

	m->vt_pix_clk_freq_mhz = (ext_clk/ (pre_pll_clk_div1 * vt_sys_clk_div * vt_pix_clk_div * row_speed)) * pll_multiplier * 2;
	
	AR1335_LOG(2, "pll_multiplier:%x pre_pll_clk_div1:%x vt_sys_clk_div:%x vt_pix_clk_div:%x row_speed:%x\r\n", pll_multiplier,
			pre_pll_clk_div1, vt_sys_clk_div, vt_pix_clk_div, row_speed);
	AR1335_LOG(2, "pixel clk:%d \r\n", m->vt_pix_clk_freq_mhz);

	/* HTS and VTS */
	m->line_length_pck = res->fps_options[dev->fps_index].pixels_per_line;
	m->frame_length_lines = res->fps_options[dev->fps_index].lines_per_frame;

	m->coarse_integration_time_min = 2;
	m->coarse_integration_time_max_margin = AR1335_INTEGRATION_TIME_MARGIN;

	/* OV Sensor do not use fine integration time. */
	m->fine_integration_time_min = 0;
	m->fine_integration_time_max_margin = 0;

	/*
	 * read_mode indicate whether binning is used for calculating
	 * the correct exposure value from the user side. So adapt the
	 * read mode values accordingly.
	 */
	m->read_mode = res->bin_factor_x ?
		AR1335_READ_MODE_BINNING_ON : AR1335_READ_MODE_BINNING_OFF;

	m->binning_factor_x = res->bin_factor_x ? 2 : 1;
	m->binning_factor_y = res->bin_factor_y ? 2 : 1;

	/* Get the cropping and output resolution to ISP for this mode. */
	ret = ar1335_read_reg(client, AR1335_16BIT, AR1335_HORIZONTAL_START_H, &val);
	if (ret)
		return ret;
	m->crop_horizontal_start =  val;

	ret = ar1335_read_reg(client, AR1335_16BIT, AR1335_VERTICAL_START_H, &val);
	if (ret)
		return ret;
	m->crop_vertical_start = val;

	ret = ar1335_read_reg(client, AR1335_16BIT, AR1335_HORIZONTAL_OUTPUT_SIZE_H, &val);
	if (ret)
		return ret;
	m->output_width = val;
	m->output_width = m->output_width - ISP_PADDING_W; /*remove ISP padding, real output*/

	ret = ar1335_read_reg(client, AR1335_16BIT, AR1335_VERTICAL_OUTPUT_SIZE_H, &val);
	if (ret)
		return ret;
	m->output_height = val;
	m->output_height = m->output_height - ISP_PADDING_H;

	/*
	 * As ar1335 is central crop, we calculate for 3264x2448 to meet IQ/OTP
	 * requirement
	 */
	if (res->bin_factor_x) {
		/*consider output padding*/
		m->crop_horizontal_start = (AR1335_ISP_MAX_WIDTH - ((m->output_width + ISP_PADDING_W) << res->bin_factor_x))/2;
	} else {
		m->crop_horizontal_start = (AR1335_ISP_MAX_WIDTH - m->output_width)/2;
	}
	m->crop_horizontal_end = AR1335_ISP_MAX_WIDTH - m->crop_horizontal_start - 1;

	if (res->bin_factor_y) {
		/*consider output padding*/
		m->crop_vertical_start = (AR1335_ISP_MAX_HEIGHT - ((m->output_height + ISP_PADDING_H) << res->bin_factor_y))/2;
	} else {
		m->crop_vertical_start = (AR1335_ISP_MAX_HEIGHT - m->output_height )/2;
	}

	m->crop_vertical_end = AR1335_ISP_MAX_HEIGHT - m->crop_vertical_start - 1;

	if(debug & DEBUG_INTG_FACT) {
		AR1335_LOG(2, "%s %d vt_pix_clk_freq_mhz:%d line_length_pck:%d frame_length_lines:%d\n", __func__, __LINE__,
				m->vt_pix_clk_freq_mhz, m->line_length_pck,	m->frame_length_lines);
		AR1335_LOG(2, "%s %d coarse_intg_min:%d coarse_intg_max_margin:%d fine_intg_min:%d fine_intg_max_margin:%d\n",
				__func__, __LINE__,
				m->coarse_integration_time_min, m->coarse_integration_time_max_margin,
				m->fine_integration_time_min, m->fine_integration_time_max_margin);
		AR1335_LOG(2, "%s %d crop_x_start:%d crop_y_start:%d crop_x_end:%d crop_y_end:%d \n", __func__, __LINE__,
				m->crop_horizontal_start, m->crop_vertical_start, m->crop_horizontal_end, m->crop_vertical_end);
		AR1335_LOG(2, "%s %d output_width:%d output_height:%d\n", __func__, __LINE__, m->output_width, m->output_height);
	}

	return 0;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
/* tune this value so that the DVS resolutions get selected properly,
 * but make sure 16:9 does not match 4:3.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 500
static int distance(struct ar1335_resolution const *res, const u32 w,
				const u32 h)
{
	unsigned int w_ratio = ((res->width<<13)/w);
	unsigned int h_ratio = ((res->height<<13)/h);
	int match   = abs(((w_ratio<<13)/h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)
		|| (match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/*
 * Returns the nearest higher resolution index.
 * @w: width
 * @h: height
 * matching is done based on enveloping resolution and
 * aspect ratio. If the aspect ratio cannot be matched
 * to any index, -1 is returned.
 */
static int nearest_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	const struct ar1335_resolution *tmp_res = NULL;
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	if((dev->curr_res_table==ar1335_res_preview)||(dev->curr_res_table==ar1335_res_still))
	if(((w == 736)&&(h == 496))
		||((w == 720)&&(h == 480))
		||((w == 640)&&(h == 480))
		||((w == 656)&&(h == 496))
		||((w == 352)&&(h == 288))
		||((w == 368)&&(h == 304))
		||((w == 176)&&(h == 144))
		||((w == 192)&&(h == 160))){
		w=2104;
		h=1560;
	}
	if(dev->curr_res_table==ar1335_res_preview){
		if(((w == 1280)&&(h == 720))||
			((w == 1024)&&(h == 768))||
			((w == 1296)&&(h == 736))||
			((w == 1040)&&(h == 784))
			)
		{
			w=2104;
			h=1184;
		}
	}
	if(dev->curr_res_table==ar1335_res_still){
		if(((w == 1920)&&(h == 1080))||
			((w == 1936)&&(h == 1096)))
		{
			w=4208;
			h=2368;
		}
	}

	for (i = 0; i < dev->entries_curr_table; i++) {
		tmp_res = &dev->curr_res_table[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

       if (idx == -1) {
               /*not found size of same ratio; choose the size just bigger than */
               for (i = 0; i < dev->entries_curr_table; i++) {
                       tmp_res = &dev->curr_res_table[i];
                       if ((tmp_res->width >= w) && (tmp_res->height >= h)) {
                               idx =  i;
                               break;
                       }
               }
       }


	return idx;
}

static int get_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	for (i = 0; i < dev->entries_curr_table; i++) {
		if (w != dev->curr_res_table[i].width)
			continue;
		if (h != dev->curr_res_table[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int __ar1335_try_mbus_fmt(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt)
{
	int idx;
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	if (!fmt)
		return -EINVAL;

	if ((fmt->width > AR1335_RES_WIDTH_MAX) ||
	    (fmt->height > AR1335_RES_HEIGHT_MAX)) {
		fmt->width = AR1335_RES_WIDTH_MAX;
		fmt->height = AR1335_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(sd, fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller resolutions.
		 * If it fails, it means the requested resolution is higher than we
		 * can support. Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = dev->entries_curr_table - 1;
                AR1335_LOG(2,"%s %d try w:%d h:%d got w:%d h:%d\n", __func__, __LINE__,
                        fmt->width,  fmt->height,
                        dev->curr_res_table[idx].width, dev->curr_res_table[idx].height);

		fmt->width = dev->curr_res_table[idx].width;
		fmt->height = dev->curr_res_table[idx].height;
	}

	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ar1335_try_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	int r;

	mutex_lock(&dev->input_lock);
	r = __ar1335_try_mbus_fmt(sd, fmt);
	mutex_unlock(&dev->input_lock);

	return r;
}

static int ar1335_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct camera_mipi_info *ar1335_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 hts, vts;
	int ret;
	const struct ar1335_resolution *res;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	ar1335_info = v4l2_get_subdev_hostdata(sd);
	if (ar1335_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);

	ret = __ar1335_try_mbus_fmt(sd, fmt);
	if (ret)
		goto out;

	dev->fmt_idx = get_resolution_index(sd, fmt->width, fmt->height);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		ret = -EINVAL;
		goto out;
	}

	/* Sets the default FPS */
	dev->fps_index = 0;

	/* Get the current resolution setting */
	res = &dev->curr_res_table[dev->fmt_idx];

	/* Write the selected resolution table values to the registers */
	ret = ar1335_write_reg_array(client, res->regs);
	if (ret)
		goto out;

	/* Frame timing registers are updates as part of exposure */
	hts = res->fps_options[dev->fps_index].pixels_per_line;
	vts = res->fps_options[dev->fps_index].lines_per_frame;
        AR1335_LOG(2,"%s %d name:%s width:%d height:%d hts:%d vts:%d\n", __func__, __LINE__,
                    res->desc, res->width, res->height, hts, vts);
        ar1335_set_exposure(sd, dev->exposure, dev->gain, dev->digital_gain);
	ret = ar1335_get_intg_factor(sd, ar1335_info, dev->basic_settings_list);

out:
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ar1335_g_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar1335_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);

	/* check sensor chip ID - are same for both 8865 and 8835 modules */
	ret = ar1335_read_reg(client, AR1335_16BIT, AR1335_CHIP_ID_HIGH, id);
	dev_info(&client->dev, "chip_id = 0x%4.4x\n", *id);

	if (ret)
		return ret;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	/* This always reads as 0x8865, even on 8835. */
	dev_info(&client->dev, "chip_id = 0x%4.4x\n", *id);
	if (*id != AR1335_CHIP_ID)
		return -ENODEV;

	return 0;
}

/*
 * ar1335 stream on/off
 */
static int ar1335_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 reg301a;
	int ret;

	mutex_lock(&dev->input_lock);
       ar1335_read_reg(client, AR1335_16BIT, 0x301a, &reg301a);
       reg301a |= 1 << 14;
       ar1335_write_reg(client, AR1335_16BIT, 0x301a, reg301a);


	if (enable) {
		AR1335_LOG(2, "%s %d enable with desire register:%d\r\n", __func__, __LINE__, enable);
		ret = ar1335_write_reg_array(client, Start_streaming);
	} else {
		AR1335_LOG(2, "%s %d enable:%d\r\n", __func__, __LINE__, enable);
		ret = ar1335_write_reg_array(client, Stop_streaming);
	}

	if (ret != 0) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}

	dev->streaming = enable;

	mutex_unlock(&dev->input_lock);

	return 0;
}

/*
 * ar1335 enum frame size, frame intervals
 */
static int ar1335_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[index].width;
	fsize->discrete.height = dev->curr_res_table[index].height;
	fsize->reserved[0] = dev->curr_res_table[index].used;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar1335_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int fmt_index;
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	const struct ar1335_resolution *res;

	mutex_lock(&dev->input_lock);

	/*
	 * since the isp will donwscale the resolution to the right size,
	 * find the nearest one that will allow the isp to do so important to
	 * ensure that the resolution requested is padded correctly by the
	 * requester, which is the atomisp driver in this case.
	 */
	fmt_index = nearest_resolution_index(sd, fival->width, fival->height);
	if (-1 == fmt_index)
		fmt_index = dev->entries_curr_table - 1;

	res = &dev->curr_res_table[fmt_index];

	/* Check if this index is supported */
	if (index > __ar1335_get_max_fps_index(res->fps_options)) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = res->fps_options[index].fps;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar1335_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ar1335_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//const struct firmware *fw;
	u8 sensor_revision = 0;
	u16 sensor_id;
	int ret;

	AR1335_LOG(2,"%s %d pdata=%lx client->name:%s adapter:%d adapter_name:%s\n", __func__, __LINE__, (unsigned long)pdata,
		client->name, client->adapter->nr,  client->adapter->name);
	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "ar1335 platform init err\n");
			return ret;
		}
	}

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	ret = __ar1335_s_power(sd, 1);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "ar1335 power-up err.\n");
		return ret;
	}

	dev->otp_data = NULL;
       ret = DW9761B_eflash_otp_read(client, ar1335_raw, &ar1335_raw_size);
       if (!ret) {
           AR1335_LOG(1,"ar1335 otp read done\n");
           ret = DW9761B_eflash_otp_trans(ar1335_raw, ar1335_raw_size, ar1335_otp_data, &ar1335_otp_size);
           if (!ret) {
               AR1335_LOG(1,"ar1335 otp trans done\n");
               dev->otp_data = ar1335_otp_data;
           } else
               AR1335_LOG(1,"ar1335 otp trans failed\n");
        }
	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	/* config & detect sensor */
	ret = ar1335_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ar1335_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	/* power off sensor */
	ret = __ar1335_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(client, "ar1335 power-down err.\n");
		return ret;
	}

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__ar1335_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
ar1335_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int
ar1335_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fse->min_width = dev->curr_res_table[index].width;
	fse->min_height = dev->curr_res_table[index].height;
	fse->max_width = dev->curr_res_table[index].width;
	fse->max_height = dev->curr_res_table[index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar1335_get_pad_format(struct ar1335_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &sensor->format;
}

static int
ar1335_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	fmt->format = *__ar1335_get_pad_format(dev, fh, fmt->pad, fmt->which);

	return 0;
}

static int
ar1335_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ar1335_get_pad_format(dev, fh, fmt->pad, fmt->which);

	*format = fmt->format;

	return 0;
}

static int ar1335_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar1335_device *dev = container_of(
		ctrl->handler, struct ar1335_device, ctrl_handler);

	/* input_lock is taken by the control framework, so it
	 * doesn't need to be taken here.
	 */

	/* We only handle V4L2_CID_RUN_MODE for now. */
	switch (ctrl->id) {
	case V4L2_CID_RUN_MODE:
		switch (ctrl->val) {
		case ATOMISP_RUN_MODE_VIDEO:
			dev->curr_res_table = ar1335_res_video;
			dev->entries_curr_table = ARRAY_SIZE(ar1335_res_video);
			break;
		case ATOMISP_RUN_MODE_STILL_CAPTURE:
			dev->curr_res_table = ar1335_res_still;
			dev->entries_curr_table = ARRAY_SIZE(ar1335_res_still);
			break;
		default:
			dev->curr_res_table = ar1335_res_preview;
			dev->entries_curr_table = ARRAY_SIZE(ar1335_res_preview);
		}

		dev->fmt_idx = 0;
		dev->fps_index = 0;
		return 0;
	/* shunyong: disable focus when PO */
	case V4L2_CID_FOCUS_ABSOLUTE:
		return dw9761_t_focus_abs(&dev->sd, ctrl->val);
	}
	return -EINVAL; /* Should not happen. */
}

static int ar1335_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar1335_device *dev = container_of(
		ctrl->handler, struct ar1335_device, ctrl_handler);

	switch (ctrl->id) {
		/* shunyong, disable Focus when PO */
		case V4L2_CID_FOCUS_STATUS: {
			static const struct timespec move_time = {
				/* The time required for focus motor to move the lens */
				.tv_sec = 0,
				.tv_nsec = 60000000,
			};
			struct dw9761_device *dw9761 = &(dev->dw9761);
			struct timespec current_time, finish_time, delta_time;

			getnstimeofday(&current_time);
			finish_time = timespec_add(dw9761->focus_time, move_time);
			delta_time = timespec_sub(current_time, finish_time);
			if (delta_time.tv_sec >= 0 && delta_time.tv_nsec >= 0) {
				/* VCM motor is not moving */
				ctrl->val = ATOMISP_FOCUS_HP_COMPLETE |
					ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
			} else {
				/* VCM motor is still moving */
				ctrl->val = ATOMISP_FOCUS_STATUS_MOVING |
					ATOMISP_FOCUS_HP_IN_PROGRESS;
			}
			return 0;
		}
        break;

		case V4L2_CID_BIN_FACTOR_HORZ:
		case V4L2_CID_BIN_FACTOR_VERT: {
			ctrl->val = ctrl->id == V4L2_CID_BIN_FACTOR_HORZ ?
				dev->curr_res_table[dev->fmt_idx].bin_factor_x: dev->curr_res_table[dev->fmt_idx].bin_factor_y;

			AR1335_LOG(1,"bin-factor for ISP:%d\n",  ctrl->val);
			return 0;
               case V4L2_CID_EXPOSURE_ABSOLUTE:
                       ctrl->val = dev->exposure;
                       return 0;
               case V4L2_CID_LINK_FREQ:
                       ctrl->val = dev->curr_res_table[dev->fmt_idx].mipi_freq * 1000;
                       AR1335_LOG(2,"V4L2_CID_LINK_FREQ query:%d\n", ctrl->val);
                       return 0;
		}
	}

	return 0;
}

static int
ar1335_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	const struct ar1335_resolution *res;

	mutex_lock(&dev->input_lock);

	/* Return the currently selected settings' maximum frame interval */
	res = &dev->curr_res_table[dev->fmt_idx];

	interval->interval.numerator = 1;
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;

	mutex_unlock(&dev->input_lock);

	return 0;
}

#if 0
static int ar1335_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ar1335_s_frame_interval(sd, interval);
	mutex_unlock(&dev->input_lock);

	return ret;
}
#endif

static int ar1335_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ar1335_device *dev = to_ar1335_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = dev->curr_res_table[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_video_ops ar1335_video_ops = {
	.s_stream = ar1335_s_stream,
	.enum_framesizes = ar1335_enum_framesizes,
	.enum_frameintervals = ar1335_enum_frameintervals,
	.enum_mbus_fmt = ar1335_enum_mbus_fmt,
	.try_mbus_fmt = ar1335_try_mbus_fmt,
	.g_mbus_fmt = ar1335_g_mbus_fmt,
	.s_mbus_fmt = ar1335_s_mbus_fmt,
	.g_frame_interval = ar1335_g_frame_interval,
	//.s_frame_interval = ar1335_s_frame_interval,
};

static const struct v4l2_subdev_sensor_ops ar1335_sensor_ops = {
	.g_skip_frames	= ar1335_g_skip_frames,
};

static const struct v4l2_subdev_core_ops ar1335_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.s_power = ar1335_s_power,
	.ioctl = ar1335_ioctl,
	.init = ar1335_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops ar1335_pad_ops = {
	.enum_mbus_code = ar1335_enum_mbus_code,
	.enum_frame_size = ar1335_enum_frame_size,
	.get_fmt = ar1335_get_pad_format,
	.set_fmt = ar1335_set_pad_format,
};

static const struct v4l2_subdev_ops ar1335_ops = {
	.core = &ar1335_core_ops,
	.video = &ar1335_video_ops,
	.pad = &ar1335_pad_ops,
	.sensor = &ar1335_sensor_ops,
};

static int ar1335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1335_device *dev = to_ar1335_sensor(sd);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = ar1335_s_ctrl,
	.g_volatile_ctrl = ar1335_g_ctrl,
};

static const char * const ctrl_run_mode_menu[] = {
	NULL,
	"Video",
	"Still capture",
	"Continuous capture",
	"Preview",
};

static const struct v4l2_ctrl_config ctrl_run_mode = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_RUN_MODE,
	.name = "run mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 1,
	.def = 4,
	.max = 4,
	.qmenu = ctrl_run_mode_menu,
};

static const struct v4l2_ctrl_config ctrls[] = {
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_EXPOSURE_ABSOLUTE,
		.name = "exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.max = 0xffff,
                .step = 0x01,
                .flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops = &ctrl_ops,
		.id = V4L2_CID_FOCUS_ABSOLUTE,
		.name = "Focus absolute",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.max = DW9761_MAX_FOCUS_POS,
	}, {
		/* This one is junk: see the spec for proper use of this CID. */
		.ops = &ctrl_ops,
		.id = V4L2_CID_FOCUS_STATUS,
		.name = "Focus status",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.max = 100,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	}, {
		/* This is crap. For compatibility use only. */
		.ops = &ctrl_ops,
		.id = V4L2_CID_FOCAL_ABSOLUTE,
		.name = "Focal lenght",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = (AR1335_FOCAL_LENGTH_NUM << 16) | AR1335_FOCAL_LENGTH_DEM,
		.max = (AR1335_FOCAL_LENGTH_NUM << 16) | AR1335_FOCAL_LENGTH_DEM,
		.step = 1,
		.def = (AR1335_FOCAL_LENGTH_NUM << 16) | AR1335_FOCAL_LENGTH_DEM,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	}, {
		/* This one is crap, too. For compatibility use only. */
		.ops = &ctrl_ops,
		.id = V4L2_CID_FNUMBER_ABSOLUTE,
		.name = "F-number",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = (AR1335_F_NUMBER_DEFAULT_NUM << 16) | AR1335_F_NUMBER_DEM,
		.max = (AR1335_F_NUMBER_DEFAULT_NUM << 16) | AR1335_F_NUMBER_DEM,
		.step = 1,
		.def = (AR1335_F_NUMBER_DEFAULT_NUM << 16) | AR1335_F_NUMBER_DEM,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	}, {
		/*
		 * The most utter crap. _Never_ use this, even for
		 * compatibility reasons!
		 */
		.ops = &ctrl_ops,
		.id = V4L2_CID_FNUMBER_RANGE,
		.name = "F-number range",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = (AR1335_F_NUMBER_DEFAULT_NUM << 24) | (AR1335_F_NUMBER_DEM << 16) | (AR1335_F_NUMBER_DEFAULT_NUM << 8) | AR1335_F_NUMBER_DEM,
		.max = (AR1335_F_NUMBER_DEFAULT_NUM << 24) | (AR1335_F_NUMBER_DEM << 16) | (AR1335_F_NUMBER_DEFAULT_NUM << 8) | AR1335_F_NUMBER_DEM,
		.step = 1,
		.def = (AR1335_F_NUMBER_DEFAULT_NUM << 24) | (AR1335_F_NUMBER_DEM << 16) | (AR1335_F_NUMBER_DEFAULT_NUM << 8) | AR1335_F_NUMBER_DEM,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	}, {
		.ops = &ctrl_ops,
		.id = V4L2_CID_BIN_FACTOR_HORZ,
		.name = "Horizontal binning factor",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.max = AR1335_BIN_FACTOR_MAX,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops = &ctrl_ops,
		.id = V4L2_CID_BIN_FACTOR_VERT,
		.name = "Vertical binning factor",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.max = AR1335_BIN_FACTOR_MAX,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
       }, {
               .ops = &ctrl_ops,
               .id = V4L2_CID_LINK_FREQ,
               .name = "Link Frequency",
               .type = V4L2_CTRL_TYPE_INTEGER,
               .min = 1,
               .max = 1500000 * 1000,
               .step = 1,
               .def = 1,
               .flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,

	}
};

static int ar1335_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ar1335_device *dev;
	unsigned int i;
	int ret;
	void *ovpdev;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	/* shunyong, disable focus when PO */

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ar1335_ops);

    ovpdev = client->dev.platform_data;
#ifdef CONFIG_GMIN_INTEL_MID
       if (ACPI_COMPANION(&client->dev))
               ovpdev = gmin_camera_platform_data(&dev->sd,
                                                  ATOMISP_INPUT_FORMAT_RAW_10,
                                                  atomisp_bayer_order_grbg);
#endif

	//if (ret < 0)
	//	goto out_free;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
		ret = ar1335_s_config(&dev->sd, client->irq,
				      ovpdev);
		if (ret)
			goto out_free;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	ret = v4l2_ctrl_handler_init(&dev->ctrl_handler, ARRAY_SIZE(ctrls) + 1);
	if (ret) {
		ar1335_remove(client);
		return ret;
	}

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	dev->run_mode = v4l2_ctrl_new_custom(&dev->ctrl_handler,
					     &ctrl_run_mode, NULL);

	for (i = 0; i < ARRAY_SIZE(ctrls); i++)
		v4l2_ctrl_new_custom(&dev->ctrl_handler, &ctrls[i], NULL);

	if (dev->ctrl_handler.error) {
		ar1335_remove(client);
		return dev->ctrl_handler.error;
	}

	/* Use same lock for controls as for everything else. */
	dev->ctrl_handler.lock = &dev->input_lock;
	dev->sd.ctrl_handler = &dev->ctrl_handler;
	v4l2_ctrl_handler_setup(&dev->ctrl_handler);

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ar1335_remove(client);
		return ret;
	}


       if (ACPI_HANDLE(&client->dev))
                       ret = atomisp_register_i2c_module(&dev->sd, ovpdev, RAW_CAMERA);

	global_dev = dev;
	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	return 0;

out_free:

	AR1335_LOG(2, "%s %d\n", __func__, __LINE__);
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

static const struct i2c_device_id ar1335_id[] = {
	{AR1335_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ar1335_id);

static struct acpi_device_id ar1335_acpi_match[] = {
	{ "INT33FB" },
	{},
};

MODULE_DEVICE_TABLE(acpi, ar1335_acpi_match);

static struct i2c_driver ar1335_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AR1335_NAME,
		.acpi_match_table = ACPI_PTR(ar1335_acpi_match),
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
	.id_table = ar1335_id,
};

static __init int ar1335_init_mod(void)
{
	AR1335_LOG(2,"%s %d\n", __func__, __LINE__);
	return i2c_add_driver(&ar1335_driver);
}

static __exit void ar1335_exit_mod(void)
{
	AR1335_LOG(2,"%s %d\n", __func__, __LINE__);
	i2c_del_driver(&ar1335_driver);
}

module_init(ar1335_init_mod);
module_exit(ar1335_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Omnivision AR1335 sensors");
MODULE_LICENSE("GPL");
