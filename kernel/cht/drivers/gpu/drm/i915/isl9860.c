/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/platform_data/isl9860.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>


struct isl9860 {
	const char *chipname;
	struct i2c_client *client;
	struct device *dev;
	struct isl9860_platform_data *pdata;
};
struct isl9860 *isl9860data;


int isl9860_ext_write_byte(u8 reg, u8 data)
{
	if (isl9860data == NULL)
		return -EINVAL;

	return i2c_smbus_write_byte_data(isl9860data->client, reg, data);
}

int isl9860_ext_read_byte(u8 reg)
{
	int ret;

	if (isl9860data == NULL)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(isl9860data->client, reg);
	if (ret < 0)
		dev_err(isl9860data->dev, "failed to read 0x%.2x\n", reg);

	return ret;
}
#if 0
static int isl9860_write_byte(struct isl9860 *isl, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(isl->client, reg, data);
}

static int isl9860_write_data(struct isl9860 *isl,  u8 *val,int len)
{
#define	I2C_RETRY_TIMES		2
	int err;
	int trytimes;
	struct i2c_msg msg[1];
	unsigned char data[9];
	int i;

    struct i2c_client *client=isl->client;
	if (!client->adapter)
		return -ENODEV;
    if(len >9) len=9;
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = len;
	msg->buf = data;
	for(i=0;i<len;i++)
		data[i]=val[i];
	/*
	data[4] = (val >> 0) & 0xFF;
	data[3] = (val >> 8) & 0xFF;
	data[2] = (val >> 16) & 0xFF;
	data[1] = (val >> 24) & 0xFF;
	*/
	trytimes=0;
	while (trytimes < I2C_RETRY_TIMES) {
		trytimes++;
		err = i2c_transfer(client->adapter, msg, 1);
		if(err>=0)
			break;
		printk(KERN_ERR "[drm]%s: i2c transfer err %d.\n", __func__, err);
		msleep(50);
	}
	if (err >= 0) {
		dev_warn(isl->dev,"isl9860_write_data:R[0x%0x] = ",val[0]);
		if(len>1)
		for(i=1;i<len;i++)
		{dev_warn(isl->dev," ,0x%x ", val[i]);}
		dev_warn(isl->dev,"\n");
		return 0;
	}

	//printk(KERN_ERR "[drm]%s: i2c transfer err %d.\n", __func__, err);
	return err;

}

static int isl9860_read_reg(struct isl9860*isl,
				u8 reg,u8 *rev, int recvsize)
{
	int i, retval;
    struct i2c_client *client = isl->client;
	/* Send the address */
	retval = i2c_master_send(client, &reg, 1);
	if (retval != 1) {
		pr_err("isl9860: I2C send retval=%d\n",retval);
		return retval;
	}
	msleep(5);
	for (i = 0; i < 5; i++)
	{
		retval = i2c_master_recv(client, &rev[0], recvsize);
		if (retval == recvsize) {
			break;
		} else {
			msleep(5);
			dev_warn(isl->dev,"isl9860: Retry count is %d\n", i);
		}
	}
	if (retval != recvsize){
		pr_err("isl9860: Read from device failed\n");
		return retval;
	}

	/*for(i=0;i<recvsize;i++)
		printk(KERN_ERR "%s rev[%d]=0x%x\n",__func__,i,rev[i]);*/
	return retval;
}

static isl9860_read_data(struct isl9860*isl,u8 reg ,bool single)
{
	struct i2c_client *client = isl->client;
	struct i2c_msg msg[2];
    unsigned char data[2];
    int ret;

    if (!client->adapter)
		return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = sizeof(reg);
    msg[1].addr =client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = data;
    if (single)
		msg[1].len = 1;
    else
		msg[1].len = 2;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ret < 0)
    {
		dev_err(isl->dev,"read data failed =%d",ret);
		return ret;
    }
    if (!single)
		ret =0;      // ret = get_unaligned_le16(data);
    else
		ret = data[0];

	dev_err(isl->dev,"read data ok =%d",ret);
    return ret;
}


#endif


static int isl9860_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct isl9860 *isl;
	struct isl9860_platform_data *pdata = cl->dev.platform_data;
	//struct device_node *node = cl->dev.of_node;
	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&cl->dev, " i2c_check_functionality failed.");
		return -EIO;
	}

	/* FIXME: If the platform has more than one LP855x chip then need
	 * to port the code here.
	 */
	if (isl9860data)
		return -ENODEV;

	isl = devm_kzalloc(&cl->dev, sizeof(struct isl9860), GFP_KERNEL);
	if (!isl)
		return -ENOMEM;

	isl9860data = isl;

	dev_err(&cl->dev, " isl9860_probe enter 2");
	isl->client = cl;
	isl->dev = &cl->dev;
	isl->pdata = pdata;
	isl->chipname = id->name;
	i2c_set_clientdata(cl, isl);


	dev_warn(&cl->dev, " isl9860_probe enter exit 0");

	return 0;
}

static int isl9860_remove(struct i2c_client *cl)
{
	dev_warn(&cl->dev, " isl9860_remove enter exit 0");
	return 0;
}

static const struct of_device_id isl9860_dt_ids[] = {
	{ .compatible = "isl,isl9860", },
	{ }
};
MODULE_DEVICE_TABLE(of, isl9860_dt_ids);

static const struct i2c_device_id isl9860_ids[] = {
	{"isl9860", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl9860_ids);

static struct i2c_driver isl9860_i2c_driver = {
	.driver = {
		   .name = "isl9860",
		   .of_match_table = of_match_ptr(isl9860_dt_ids),
	},
	.probe = isl9860_probe,
	.remove = isl9860_remove,
	.id_table = isl9860_ids,
};

struct isl9860_platform_data isl9860_platform_data = {
	.name = "isl9860",
	.device_control = 0,
	.initial_brightness = 0,
	.period_ns = 5000000, /* 200 Hz */
	.size_program = 1,
	//.rom_data = isl9860_rom_data,
};

#define isl_I2C_BUS_NUM			0
void *isl9860_get_platform_data(void)
{
	return (void *)&isl9860_platform_data;
}

static struct i2c_board_info __initdata isl9860_i2c_device = {
	I2C_BOARD_INFO("isl9860", 0x29),
	//I2C_BOARD_INFO("isl9860", 0x36),
};

static int __init isl9860_i2c_platform_init(void)
{
	void *pdata = NULL;
	int ret = 0;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client = NULL;

	//intel_scu_ipc_msic_vprog3(1);
	printk(KERN_ERR "[drm]%s: isl9860 i2c device register.\n", __func__);
    pdata = isl9860_get_platform_data();

    if(pdata != NULL)
        isl9860_i2c_device.platform_data = pdata;
    else
        printk("%s, pdata is NULL\n", __func__);

	adapter = i2c_get_adapter(isl_I2C_BUS_NUM);
	if (!adapter) {
		printk(KERN_ERR "i2c_get_adapter(%u) failed.\n", isl_I2C_BUS_NUM);
		ret = -EPERM;
		goto out;
	}

	client = i2c_new_device(adapter, &isl9860_i2c_device);
	if (client == NULL) {
		printk(KERN_ERR "i2c_new_device(%u) failed.\n", isl_I2C_BUS_NUM);
		ret = -EPERM;
	}

out:
	return ret;
}

device_initcall(isl9860_i2c_platform_init);


static int __init isl9860_i2c_bus_init(void)
{
	printk(KERN_ERR "[drm]%s: isl9860 i2c driver register.\n", __func__);
	return i2c_add_driver(&isl9860_i2c_driver);
}
module_init(isl9860_i2c_bus_init);

static void __exit isl9860_i2c_bus_exit(void)
{
	printk(KERN_ERR "[drm]%s: isl9860 i2c driver delete.\n", __func__);
	i2c_del_driver(&isl9860_i2c_driver);

	return;
}
module_exit(isl9860_i2c_bus_exit);


MODULE_DESCRIPTION("ISL isl9860");
MODULE_AUTHOR("zhurong fu <fuzr1@lenovo.com>");
MODULE_LICENSE("GPL");
