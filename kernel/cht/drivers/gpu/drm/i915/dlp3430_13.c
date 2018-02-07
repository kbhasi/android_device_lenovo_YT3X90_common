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
#include <linux/platform_data/dlp3430.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <drm/drmP.h>
#include <drm/intel_dual_display.h>
#define BRIGHT_MAX 0x29e
#define BRIGHT_MIN 0X64
static  DEFINE_MUTEX(dlp_mutex);
//extern int simulate_valleyview_irq_handler(bool enable,int (*cb)(int));
extern void drm_sysfs_power_key_up(void);
static int dlp_status =0;
//static int debug_enable =0;
static bool dlp_poweron =false;
extern int   dlp3430_init(void);
struct dlp3430 {
	const char *chipname;
	struct i2c_client *client;
	struct device *dev;
	struct dlp3430_platform_data *pdata;
};

struct dlp3430 *dlpdata;
struct dlp3430_rom_data *rom_data;
struct kobject *dlp3430_kobj;
//extern struct intel_dual_display_status dual_display_status;

static int init_brightness =128;
static int init_angle =0;
static int init_localbrightness =32;
extern bool get_dlpstatus(int status);
extern void *dlp3430_get_platform_data(void);
//irq context


void dlp3430_sysfs_onoff_event(int status)
{
	char *event_string = "DLPSTATUS";

	char *envp[] = { event_string, NULL };
	/* if(status) envp[1]="1";
	   else     envp[1]="0";
	   printk("generating dlponoff event\n");*/
	if(dlpdata==NULL)
		return;
	kobject_uevent_env(&dlpdata->dev->kobj, KOBJ_CHANGE, envp);
}

bool get_dlpstatus(int status)
{
	printk("%s---status=%d\n",__func__,status);
	mutex_lock(&dlp_mutex);
	if(status==110 && dlp_status ==1)
		dlp_status= 0;
	if(status==111 && dlp_status ==0)
		dlp_status =1;
	 mutex_unlock(&dlp_mutex);
     return dlp_status;
}
#ifdef CONFIG_HDMI_DLP
extern int get_edpstatus(void);
#endif
int get_edpstatus(void)
{
	int ret=0;
	mutex_lock(&dlp_mutex);
#if 0
	if(dual_display_status.pipea_status == PIPE_ON)
		ret= 1;
	else
		ret= 0;
#endif
	mutex_unlock(&dlp_mutex);
	return ret;
}
/* FIXME: If the platform has more than one LP855x chip then need
 * to port the code here.
 */


int dlp3430_ext_write_byte(u8 reg, u8 data)
{
	if (dlpdata == NULL)
		return -EINVAL;

	return i2c_smbus_write_byte_data(dlpdata->client, reg, data);
}

int dlp3430_ext_read_byte(u8 reg)
{
	int ret;

	if (dlpdata == NULL)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(dlpdata->client, reg);
	if (ret < 0)
		dev_err(dlpdata->dev, "failed to read 0x%.2x\n", reg);

	return ret;
}
/*
static int dlp3430_write_byte(struct dlp3430 *dlp, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(dlp->client, reg, data);
}
*/
static int dlp3430_write_data(struct dlp3430 *dlp,  u8 *val,int len)
{
#define	I2C_RETRY_TIMES		2
	int err;
	int trytimes;
	struct i2c_msg msg[1];
	unsigned char data[9];
	int i;
	
	struct i2c_client *client=dlp->client;
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
		dev_warn(dlp->dev,"dlp3430_write_data:R[0x%0x] = ",val[0]);
		if(len>1)
		for(i=1;i<len;i++)
		{dev_warn(dlp->dev," ,0x%x ", val[i]);}
		  dev_warn(dlp->dev,"\n");
		return 0;
	}

	//printk(KERN_ERR "[drm]%s: i2c transfer err %d.\n", __func__, err);
	return err;
}

static int dlp3430_read_reg(struct dlp3430*dlp,
				u8 reg,u8 *rev, int recvsize)
{
	int i, retval;
       struct i2c_client *client = dlp->client;
	/* Send the address */
	retval = i2c_master_send(client, &reg, 1);
	if (retval != 1) {
		pr_err("dlp3430: I2C send retval=%d\n",retval);
		return retval;
	}
	msleep(5);
	for (i = 0; i < 5; i++)	{
		retval = i2c_master_recv(client, &rev[0], recvsize);
		if (retval == recvsize) {
			break;
		} else {
			msleep(5);
			dev_warn(dlp->dev,"dlp3430: Retry count is %d\n", i);
		}
	}
	if (retval != recvsize){
		pr_err("dlp3430: Read from device failed\n");
		return retval;
	}
	
	/*for(i=0;i<recvsize;i++)
		printk(KERN_ERR "%s rev[%d]=0x%x\n",__func__,i,rev[i]);*/
	return retval;
}

static int dlp3430_read_data(struct dlp3430*dlp,u8 reg ,bool single)
{
	struct i2c_client *client = dlp->client;
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
		dev_err(dlp->dev,"read data failed =%d",ret);
		return ret;
	}
	if (!single)
		ret =0;      // ret = get_unaligned_le16(data);
	else
		ret = data[0];

	dev_err(dlp->dev,"read data ok =%d",ret);
	return ret;
}
#ifdef CONFIG_HDMI_DLP
static int
bladeiii_dlp_power_ctrl(struct drm_device *dev,bool enable)
{
	static struct regulator * sii9293_vcc_1v0;
	int ret = 0;

	/* FIXME need to add VSWITCH1 control for 1P8V_DLP_PMU */

	DRM_INFO("try to enable VPROG6A for SII9293_VCC_1V0\n");
	if(sii9293_vcc_1v0 == NULL){
		sii9293_vcc_1v0 = regulator_get(dev->dev, "VPROG6A");
		if (IS_ERR(sii9293_vcc_1v0)) {
			DRM_ERROR("get regulator VPROG6A failed.\n");
			return -1;
		}
	}
	if(enable){
		ret = regulator_set_voltage(sii9293_vcc_1v0, 1800000, 1800000);
		if (ret) {
			DRM_ERROR("set VPROG6A voltage failed.\n");
			return ret;
		}

		ret = regulator_enable(sii9293_vcc_1v0);
		if (ret) {
			DRM_ERROR("enable VPROG6A failed.\n");
			return ret;
		}
	}
	else{
		ret = regulator_disable(sii9293_vcc_1v0);
		if (ret) {
			DRM_ERROR("disable VPROG6A failed.\n");
			return ret;
		}
	}
	
	return 0;
}
#endif

#define DLP_ON			397
#define DLP_POWER_ON		390
extern int dlp3430_power_on(struct drm_device *dev);
extern int dlp3430_power_off(struct drm_device *dev);
int dlp3430_power_on(struct drm_device *dev)
{    
	//int times=100;
	int ret=0;
	//int state;
	u8 data[9]={0};

	printk(KERN_ERR "[drm]%s In.\n", __func__);

	msleep(1);  //wait for the dlp to be ready to init the i2c controller
   	if(dlpdata==NULL)
		return 1;
#ifdef CONFIG_HDMI_DLP	
	bladeiii_dlp_power_ctrl(dev,1);
#endif 
	gpio_direction_output(DLP_POWER_ON, 1);
	msleep(50);
	gpio_direction_output(DLP_ON, 1);
	msleep(500);
	
   #if 1  
	data[0]=0xe5;
	data[1]=0x2c;
	data[2]=0x70;
	data[3]=0x01;
	data[4]=0x40;
	ret = dlp3430_write_data(dlpdata,data,5);
	if(ret) {
		printk("[drm]%s: write i2c data failed. %d.\n", __func__, ret);
	}
	msleep(5);
	data[0]=0xe6;
	data[1]=0x0b;
	data[2]=0xff;  //0xff not ok fuzr on 2014-4-14
	data[3]=0x00;
	data[4]=0x00;
	ret = dlp3430_write_data(dlpdata,data,5);
	if(ret) {
		printk("[drm]%s: write i2c data failed. %d.\n", __func__, ret);
	}
	msleep(5);

#endif	  

	return 0;
}

int dlp3430_power_off(struct drm_device *dev)
{
	printk(KERN_ERR "[drm]%s In.\n", __func__);
//	gpio_direction_output(154, 0);
	msleep(1000);  //wait for the dlp to be ready to init the i2c controller
	if(dlp_poweron == false) {
		return 0;
	}
	if(dlpdata == NULL) {
		printk(KERN_ERR "[drm]%s: dlpdata == NULL.\n", __func__);
	}

	gpio_direction_output(DLP_POWER_ON, 0);
	msleep(40);  //request by ti 2014-09-08 change from 20 to 40ms
	gpio_direction_output(DLP_ON,0);

#ifdef CONFIG_HDMI_DLP
	bladeiii_dlp_power_ctrl(dev,0);
#endif
	dlp_poweron =false;
	dlp3430_sysfs_onoff_event(0);
	printk("[drm]%s: trace 0001\n", __func__);
	//dev_warn(dlpdata->dev,"%s\n",__func__);
   return 0;
}
static int dlp3430_set_light_brightness(u8 brightness);
static int dlp3430_set_light_brightness(u8 brightness)//brightness 0~255
{
       //set led max current
	u8 data[7];
	int ret=-1;
	u32 brightness_in_mA;
	u16 brightness_msb,brightness_lsb;
	brightness_in_mA=brightness*BRIGHT_MAX/255;
	if(brightness_in_mA > BRIGHT_MAX)
		brightness_in_mA = BRIGHT_MAX;
	else if(brightness_in_mA < BRIGHT_MIN)
		brightness_in_mA = BRIGHT_MIN;
	brightness_msb = (brightness_in_mA>>8)&0xff;
	brightness_lsb = brightness_in_mA&0xff;
	data[0]=0x54; //cmd to set the current value
	data[1]=brightness_lsb;
	data[2]=brightness_msb;
	data[3]=brightness_lsb;
	data[4]=brightness_msb;
	data[5]=brightness_lsb;
	data[6]=brightness_msb;
	ret= dlp3430_write_data(dlpdata,data,7);
	return ret;
}

static int dlp3430_get_light_brightness(u8 * brightness);
static int dlp3430_get_light_brightness(u8 * brightness)//brightness 0~255
{
	//set led max current
	int ret=-1;
	u8 data[7];
	u32 brightness_in_mA;
	//u16 brightness_msb,brightness_lsb;
	data[0]=0x55;  //cmd to read the current value
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=0;

	if(6!=dlp3430_read_reg(dlpdata,data[0],&data[1],6)) {
		*brightness =0;
	} else {
			ret =0;
			brightness_in_mA = data[1]+(data[2]<<8 & 0xff00);
			*brightness = brightness_in_mA *255/BRIGHT_MAX;
			printk("dlp get brightness =%d\n",*brightness);
		}

		return ret;
	}

	static int dlp3430_set_pitch_angle(int angle);
	static int dlp3430_set_pitch_angle(int angle)//-40 ---+40
	{
		int ret=-1;
		//set the pitch angle
		u8 data[3];
		if (angle>40 || angle <-40)
			return ret;
		msleep(5);
		data[0]=0x1a;
		data[1]=0x1;
		dlp3430_write_data(dlpdata,data,2);
		msleep(5);
		data[0]=0xBB; //cmd to set the pitch angle
		if(angle>0)
			data[2]=angle & 0x7f;
		else
			data[2]=angle & 0xff;//(((0-angle) & 0x7f) |0x80);
		data[1]=0x0;

		ret= dlp3430_write_data(dlpdata,data,3);
		msleep(30);
		data[0]=0x1a;
		data[1]=0x0;
		dlp3430_write_data(dlpdata,data,2);
		return ret;
	}

	static int dlp3430_get_pitch_angle(int * angle);
	static int dlp3430_get_pitch_angle(int * angle)//-40 ---+40
	{
		int ret=-1;
		//set led max current
		u8 data[3];


		data[0]=0xBC;  //cmd to read the pitch angle
		data[1]=0;
		data[2]=0;

		if(2!=dlp3430_read_reg(dlpdata,data[0],&data[1],2))	{
			*angle =0;
		} else {
			// if(data[2]&0x80)
			//*angle =0-(data[2] & 0x7f);
			//else
			*angle =(data[2] );
			ret =0;
			printk("dlp get angle =%d\n",*angle);
		}

		return ret;
	}

static int dlp3430_set_local_brightness(int local_brightness)//0--255
{
	int ret=-1;
	//set the local_brightness
	u8 data[3];
	if (local_brightness>255 || local_brightness <0)
		return ret;
	data[0]=0x80; //cmd to set the local brightness
	data[2]=local_brightness & 0xff;

	data[1]=0x11;

	return dlp3430_write_data(dlpdata,data,3);
}

static int dlp3430_get_local_brightness(int * local_brightness);
static int dlp3430_get_local_brightness(int * local_brightness)//
{
	int ret=-1;
	//get local brigntness strength
	u8 data[4];


	data[0]=0x81;  //get local brigntness strength
	data[1]=0;
	data[2]=0;
	data[3]=0;   

	if(3!=dlp3430_read_reg(dlpdata,data[0],&data[1],3)) {
		*local_brightness =0;
	} else {
		*local_brightness= data[2] ;
		ret =0;
		printk("dlp get local_brightness =%d,data[1]=%x data[2]=%x data[3]=%x\n",*local_brightness,data[1],data[2],data[3]);
	}
	return ret;
}

static int dlp3430_get_temperature(int * temperature)
{
	//get the temperature
	// we only get the 2bytes ,byte 1 is the lsbyte, byte2 is msbyte
	// and bits 15-12 is 0 , if bit 11 is 1 , it is negative or it is positive
	int ret=-1;
	u8 data[3];
	data[0]=0xd6;   //to get the temperature for the dlp
	if(2!=dlp3430_read_reg(dlpdata,data[0],&data[1],2))	{
		*temperature =-0x80000;
	} else {
		if(0x08 & data[2]) {
			*temperature = 0-(0x7ff & (data[2] <<8)) -data[1];
		} else {
			*temperature = (0x7ff & (data[2] <<8)) + data[1];
			ret =0;
		}
		printk("dlp get temperature =%d, ret =%d\n",*temperature,ret);
	}
	return ret;
}

static int dlp3430_open(struct inode *inode, struct file *filp)
{
	rom_data = kzalloc(sizeof(struct dlp3430_rom_data), GFP_KERNEL);

    if (rom_data == NULL) {
        printk("Not enough memory to initialize device\n");
        return -ENOMEM;
    }
    return 0;
}

static int dlp3430_release(struct inode *inode, struct file *filp)
{
    kfree(rom_data);
    rom_data=NULL;
    return 0;
}


static long
dlp3430_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    void __user *argp = (void __user *)arg;
    int value0;
    int value1 ;
    u8 data[4]={0};

    /* Verify user arguments. */
    if (_IOC_TYPE(cmd) != DLP3430_IOC_MAGIC)
       return -ENOTTY;

    switch (cmd) {
		case  DLP3430_SET_HIGH:
			printk("DLP3430_SET_HIGH---dlp3430_poweron\n");
			// dlp3430_power_on();
			err=simulate_valleyview_irq_handler(1,get_dlpstatus);
			if (arg == 0) {
				printk("user space arg not supplied\n");
				err = -EFAULT;

			}

			break;

		case  DLP3430_SET_LOW:
			printk("DLP3430_SET_LOW--power_off\n");
			//dlp3430_power_off();
			err=simulate_valleyview_irq_handler(0,get_dlpstatus);
			if (arg == 0) {
				pr_err("user space arg not supplied\n");
				err = -EFAULT;

			}

			break;

		case DLP3430_WRITE_STRUCT:
			printk("DLP3430_WRITE\n");
			if (copy_from_user(rom_data, argp, sizeof(struct dlp3430_rom_data)))
				return -EFAULT;


			break;

		case DLP3430_READ_STRUCT:
			printk("DLP3430_READ\n");

			if (copy_to_user(argp, rom_data, sizeof(struct dlp3430_rom_data)))
				return -EFAULT;

			break;

		case DLP3430_WRITE_INT:
			printk("DLP3430_WRITE_INT\n");
			if(copy_from_user(&value0,argp,sizeof(int)))
				return -EFAULT;
			printk("value0 = %d\n",value0);
			break;

		case DLP3430_READ_INT:
			printk("DLP3430_READ_INT\n");
			value1=101;
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_SET_BRIGHTNESS:
			printk("DLP3430_SET_BRIGHTNESS\n");
			if(copy_from_user(&value0,argp,sizeof(int)))
				return -EFAULT;
			printk("value0 = %d\n",value0);
			init_brightness = value0;
			if(dlp_poweron==true)
				err= dlp3430_set_light_brightness(value0);
			break;

		case DLP3430_GET_BRIGHTNESS:
			printk("DLP3430_GET_BRIGHTNESS\n");
			if(dlp_poweron==true) {
				err=dlp3430_get_light_brightness((u8 *)data);
				value1=data[0];
			} else
				value1=init_brightness;
			printk("dlp get brightness=%d\n",value1);
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_SET_ANGLE:
			printk("DLP3430_SET_ANGLE\n");
			if(copy_from_user(&value0,argp,sizeof(int)))
				return -EFAULT;
			printk("value0 = %d\n",value0);
			init_angle=value0;
			if(dlp_poweron==true)
				err=dlp3430_set_pitch_angle(value0);
			break;

		case DLP3430_GET_ANGLE:
			printk("DLP3430_GET_ANGLE--dlp_status=%d\n",dlp_status);

			if(dlp_poweron==true) {
				err= dlp3430_get_pitch_angle(&value1);
			} else {
				value1 =init_angle;
			}
			printk("get angle =%d\n",value1);
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_GET_TEMPERATURE:
			printk("DLP3430_GET_TEMPERATURE\n");
			if(dlp_poweron==true)
				err= dlp3430_get_temperature(&value1);
			else
				value1=420;
			printk("get temperature =%d\n",value1);
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_GET_DLP_STATUS:
			printk("DLP3430_GET_DLP_STATUS\n");
			value1 =dlp_poweron?1:0;
			printk("get dlp status =%d\n",value1);
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_GET_EDP_STATUS:
			printk("DLP3430_GET_EDP_STATUS\n");

			value1 =get_edpstatus();
			printk("get edp status =%d\n",value1);
			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		case DLP3430_SET_EDP_ON:
			printk("DLP3430_SET_EDP_ON\n");
			if(get_edpstatus()==0)
				drm_sysfs_power_key_up();
			break;
		case DLP3430_SET_LOCAL_BRIGHTNESS:
			printk("DLP3430_SET_LOCAL_BRIGHTNESS\n");
			if(copy_from_user(&value0,argp,sizeof(int)))
				return -EFAULT;
			printk("value0 = %d\n",value0);
			init_localbrightness = value0;
			if(dlp_poweron==true)
			{
				err=dlp3430_set_local_brightness(value0);
			}
			break;

		case DLP3430_GET_LOCAL_BRIGHTNESS:


			if(dlp_poweron==true)
			{ err= dlp3430_get_local_brightness(&value1);}
			else
			{ value1 =init_localbrightness;}

			printk("get local brightness =%d\n",value1);

			if (copy_to_user(argp, &value1, sizeof(int)))
				return -EFAULT;
			break;
		default:
			printk("Invalid ioctl command.\n");
			return -ENOTTY;
	}
	return err;
}

static const struct file_operations dlp3430_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dlp3430_ioctl,
	.open = dlp3430_open,
	.release = dlp3430_release
};

static struct miscdevice dlp3430_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dlp3430",
	.fops = &dlp3430_fops
};

int dlp3430_init(void)
{	
	u8 data[9];
	int ret=-1;
	printk(KERN_ERR "[drm]%s In.\n", __func__);
	if(dlpdata==NULL) {
		printk(KERN_ERR "%s--error for i2c driver not init \n",__func__);
		return -1;
	}
#if 1
	data[0]=0xde;// cmd to select the read region
	data[1]=0xb0;
	ret = dlp3430_write_data(dlpdata,data,2);
	if(ret) {
		printk("[drm]%s: write i2c data failed. %d.\n", __func__, ret);
	}
	data[0]=0xdf;  //cmd to select the read size to 8.
	data[1]=0x08;
	data[2]=0x00;

	ret = dlp3430_write_data(dlpdata,data,3);
	if(ret) {
		printk("[drm]%s: write i2c data failed. %d.\n", __func__, ret);
	}
	msleep(5);
	data[0]=0xe3; //try to read oem byte[0-7]

	if(8==dlp3430_read_reg(dlpdata,data[0],&data[1],8))
	{
		printk("dlp3430 read 0xe3 ok lookdata[1]=%x--data[7]=%x\n",data[1],data[7]);
	}
	else
	{
		printk("dlp3430 read 0xe3 err lookdata[1]=%x--data[7]=%x\n",data[1],data[7]);
	}

	msleep(5);
	data[0]=0x22;
	ret=dlp3430_write_data(dlpdata,data,2);//try to select the look
	//if(debug_enable)
	{
		data[0]=0xeb;
		data[1]=0x0d;
		data[2]=0x0;
		data[3]=0x00;
		data[4]=0x00;
		data[5]=0x1;
		data[6]=0x1;
		msleep(5);

		ret=dlp3430_write_data(dlpdata,data,7);
		data[0]=0xed;
		data[1]=0;
		data[2]=0;
		msleep(5);
		if(4==dlp3430_read_reg(dlpdata,data[0],&data[1],4))
		{
			printk(" dlp3430_init debug data1=%x,data2=%x,data3=%x,data4=%x\n",data[1],data[2],data[3],data[4]);
		}
		else
		{
			printk("dlp3430_init debug return err\n");
		}
		data[0]=0xec;
		data[1]=0xd0;
		msleep(5);
		ret=dlp3430_write_data(dlpdata,data,2);
	}
	msleep(5);
	data[0] = 0x86; //2014-07-08 TI require for CCA to 0 to pass the red axis BLADEIIB-1148
	data[1] =0;
	ret=dlp3430_write_data(dlpdata,data,2);

	//set led max current
	data[0]=0x5c;
	data[1]=0x9e;
	data[2]=0x02;
	data[3]=0x9e;
	data[4]=0x02;
	data[5]=0x9e;
	data[6]=0x02;
	msleep(5);
	dlp3430_write_data(dlpdata,data,7);
	msleep(5);
#endif
	//Input ImageSize 854*480
	data[0]=0x2e;
	data[1]=0x56;
	data[2]=0x03;
	data[3]=0xe0;
	data[4]=0x01;
	dlp3430_write_data(dlpdata,data,5);
	msleep(5);

	//Display size 854*480
	data[0]=0x12;
	data[1]=0x56;
	data[2]=0x03;
	data[3]=0xe0;
	data[4]=0x01;
	dlp3430_write_data(dlpdata,data,5);
	msleep(5);

	//ImageCrop
	data[0]=0x10;
	data[1]=0x00;
	data[2]=0x00;
	data[3]=0x00;
	data[4]=0x00;
	data[5]=0x56;
	data[6]=0x03;
	data[7]=0xe0;
	data[8]=0x01;
	dlp3430_write_data(dlpdata,data,9);
	msleep(5);

	//InputSourceSelect  DSI
	data[0]=0x05;
	data[1]=0x00;
	dlp3430_write_data(dlpdata,data,2);
	msleep(5);

	//VideoSourceFormatSelect RGB888
	data[0]=0x07;
	data[1]=0x00;
	dlp3430_write_data(dlpdata,data,2);
	msleep(5);

	//set keystore to enable to control, optical throw rate=1.65 DMP =1.0
	data[0]=0x88;
	data[1]=0x01;
	data[2]=0xA6;
	data[3]=0x01;
	data[4]=0x00;
	data[5]=0x01;

	dlp3430_write_data(dlpdata,data,6);
	msleep(5);
	data[0] =0x50;  //write led output control method
	data[1] =0x01;  //enable cAIC algorithm
	dlp3430_write_data(dlpdata,data,2);
	msleep(5);
	data[0]=0x84;//write CALC LUMEN GAIN 1.375 
	data[1]=0x00;// 
	data[2]=0x2c;//
	data[3]=0x00;
	dlp3430_write_data(dlpdata,data,4);
	msleep(5);
	dlp3430_set_local_brightness(init_localbrightness);
	msleep(5);
	dlp3430_set_light_brightness(init_brightness);
	msleep(5);
	dlp3430_set_pitch_angle(init_angle);

	dlp_poweron=true;
	dlp3430_sysfs_onoff_event(0);
	return 0;

}

static ssize_t dlp3430_brightness_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	u8 brightness=0;
	printk (KERN_INFO "%s \n", __func__);
	dlp3430_get_light_brightness(&brightness);
	p += sprintf(p,"The brightness =%d\n",brightness);

	return (p-buf);
}
static ssize_t dlp3430_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 bright;
	bright = simple_strtol(buf,NULL,0);

	printk("%s: write  bright %d ..\n", __func__,bright);
	dlp3430_set_light_brightness(bright);	  
	return count;
}


static DEVICE_ATTR(brightness, S_IRUGO| S_IWUSR, dlp3430_brightness_show, dlp3430_brightness_store);

static ssize_t dlp3430_angle_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int angle;

	printk (KERN_INFO "%s \n", __func__);
	dlp3430_get_pitch_angle(&angle);
	p += sprintf(p,"The Angle =%d\n",angle);

	return (p-buf);
}
static ssize_t dlp3430_angle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int angle;
	angle = simple_strtol(buf,NULL,0);
	printk("%s: write  angle %d ..\n", __func__,angle);
	dlp3430_set_pitch_angle(angle);	  
	return count;
}

static DEVICE_ATTR(angle, S_IRUGO| S_IWUSR, dlp3430_angle_show, dlp3430_angle_store);

static ssize_t dlp3430_get_test_i2c(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* int ret;
	//struct dlp3430 *dlp = dev_get_drvdata(dev);

	// ret =dlp3430_ext_read_byte(0x06);
	return scnprintf(buf, PAGE_SIZE, "%s\n", dlpdata->chipname);
	*/
	return 0;
}

static ssize_t dlp3430_set_test_i2c(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	int ret,i=0;
	u32 data1,data2,data3,data4,data5,data6,data7,data8,reg;
	u8 data[10];
	char *p1,*p2;
	char str[10][5]={{0}};
	//char str0[4]={0},str1[4]={0},str2[4]={0},str3[4]={0},str4[4]={0},str5[4]={0},str6[4]={0},str7[4]={0},str8[4]={0},str9[4]={0},str10[4]={0},str11[4]={0};
	//char cmd[6] ={0};

	// u8 len;
	int cnt;
	char lbuf[50];
	//struct dlp3430 *dlp = dev_get_drvdata(dev);
	dev_warn(dlpdata->dev,"echo buf=%s\n",buf);
	ret = copy_from_user(lbuf,buf,count);
	dev_warn(dlpdata->dev,"echo buf=%s\n",lbuf);
	p1=(char*)buf;
	i=0;
	do
	{
		if(p1==NULL)break;
		p2=strchr(p1,',');
		if(p2!=NULL)
			strncpy(str[i],p1,strlen(p1)-strlen(p2));
		else
			strncpy(str[i],p1,strlen(p1));

		if(p2!=NULL)
		{ 
			p1=p2+1;}
		else
			p1=NULL;

		i++;
	}while(i<10);


	//sscanf(buf,"%[^,] ,%[^,] ,%[^,],%[^,] ,%[^,] ,%[^,] ,%[^,] ,%[^,] ,%[^,], %[^,]",str0,str1,str2,str3,str4,str5,str6,str7,str8,str9);
	//dev_warn(dlpdata->dev,"split to =%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",str0,str1,str2,str3,str4,str5,str6,str7,str8,str9);
	reg = simple_strtol(str[0],NULL,0);
	data1= simple_strtol(str[1],NULL,0);
	data2= simple_strtol(str[2],NULL,0);
	data3= simple_strtol(str[3],NULL,0);
	data4= simple_strtol(str[4],NULL,0);
	data5= simple_strtol(str[5],NULL,0);
	data6= simple_strtol(str[5],NULL,0);
	data7= simple_strtol(str[7],NULL,0);
	data8= simple_strtol(str[8],NULL,0);
	cnt= simple_strtol(str[9],NULL,0);

	dev_warn(dlpdata->dev,"  reg =%x,data1=%x,data2=%x,data3=%x,data4=%x,data5=%x,data6=%x,data7=%x,data8=%x,cnt=%d\n",reg,data1,data2,data3,data4,data5,data6,data7,data8,cnt);
	data[0]=reg;
	data[1]=data1;
	data[2]=data2;
	data[3]=data3;
	data[4]=data4;
	data[5]=data5;
	data[6]=data6;
	data[7]=data7;
	data[8]=data8;
	if(i>9)i=9;
	dlp3430_write_data(dlpdata,&data[0],i);

	return count;
}


static DEVICE_ATTR(test_i2c, S_IRUGO | S_IWUSR, dlp3430_get_test_i2c, dlp3430_set_test_i2c);

static ssize_t dlp3430_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int temperature;

	printk (KERN_INFO "%s \n", __func__);
	dlp3430_get_temperature(&temperature);
	p += sprintf(p,"The Temperature =%d\n",temperature);

	return (p-buf);
}
static ssize_t dlp3430_temperature_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int angle;
	angle = simple_strtol(buf,NULL,0);
	printk("%s:  no use for this function%d ..\n", __func__,angle);
	return count;
}


static DEVICE_ATTR(temperature, S_IRUGO|S_IWUSR, dlp3430_temperature_show,dlp3430_temperature_store);


static ssize_t dlp3430_get_reg_id(struct device *dev, struct device_attribute *atattr, char *buf)
{
	int ret;
	//u8 data[5];
	ret = dlp3430_read_data(dlpdata,0x06,true);

	return scnprintf(buf, PAGE_SIZE, "%s=%d\n", dlpdata->chipname,ret);
}

static ssize_t dlp3430_set_reg_id(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	int ret;
	unsigned char reg;
	reg = simple_strtol(buf,NULL,0);
	ret = dlp3430_read_data(dlpdata,reg,true);
	dev_warn(dlpdata->dev,"read the reg[%x]=%x\n",reg,ret);	
	return count;
}

static DEVICE_ATTR(reg_id, S_IRUGO| S_IWUSR, dlp3430_get_reg_id, dlp3430_set_reg_id);

static struct attribute *dlp3430_attributes[] = {
	&dev_attr_reg_id.attr,
	&dev_attr_test_i2c.attr,
	&dev_attr_brightness.attr,
	&dev_attr_angle.attr,
	&dev_attr_temperature.attr,
	NULL,
};

static const struct attribute_group dlp3430_attr_group = {
	.attrs = dlp3430_attributes,
};

#ifdef CONFIG_OF
static int dlp3430_parse_dt(struct device *dev, struct device_node *node)
{
	struct dlp3430_platform_data *pdata;
	int rom_length;
	dev_warn(dev, " dlp3430_parse_dt enter ");
	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}
	dev_warn(dev, " dlp3430_parse_dt enter 1");
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	of_property_read_string(node, "bl-name", &pdata->name);
	of_property_read_u8(node, "dev-ctrl", &pdata->device_control);
	of_property_read_u8(node, "init-brt", &pdata->initial_brightness);
	of_property_read_u32(node, "pwm-period", &pdata->period_ns);

	/* Fill ROM platform data if defined */
	rom_length = of_get_child_count(node);
	if (rom_length > 0) {
		struct dlp3430_rom_data *rom;
		struct device_node *child;
		int i = 0;

		rom = devm_kzalloc(dev, sizeof(*rom) * rom_length, GFP_KERNEL);
		if (!rom)
			return -ENOMEM;

		for_each_child_of_node(node, child) {
			of_property_read_u8(child, "rom-addr", &rom[i].addr);
			of_property_read_u8(child, "rom-val", &rom[i].val);
			i++;
		}

		pdata->size_program = rom_length;
		pdata->rom_data = &rom[0];
	}
	dev_warn(dev, " dlp3430_parse_dt exit ");
	dev->platform_data = pdata;

	return 0;
}
#else
static int dlp3430_parse_dt(struct device *dev, struct device_node *node)
{
	dev_warn(dev, " a dlp3430_parse_dt enter ");
	return -EINVAL;
}
#endif


static int dlp3430_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct dlp3430 *dlp;
	struct dlp3430_platform_data *pdata = cl->dev.platform_data;
	struct device_node *node = cl->dev.of_node;
	int ret;

	printk(KERN_ERR "[drm]%s In.\n", __func__);
	ret = gpio_request(DLP_POWER_ON, "DLP_POWER_ON");
	if (ret < 0) {
		DRM_ERROR("gpio_request %d failed. ret = %d\n", DLP_POWER_ON, ret);
	}

	ret = gpio_request(DLP_ON, "DLP_ON");
	if (ret < 0) {
		DRM_ERROR("gpio_request %d failed. ret = %d\n", DLP_ON, ret);
	} 
	dev_err(&cl->dev, " dlp3430_probe enter ");
	if (!pdata) {
		ret = dlp3430_parse_dt(&cl->dev, node);
		if (ret < 0)
			return ret;

		pdata = cl->dev.platform_data;
	}

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&cl->dev, " i2c_check_functionality failed.");
		return -EIO;
	}

	/* FIXME: If the platform has more than one LP855x chip then need
	 * to port the code here.
	 */
	if (dlpdata)
		return -ENODEV;

	dlp = devm_kzalloc(&cl->dev, sizeof(struct dlp3430), GFP_KERNEL);
	if (!dlp)
		return -ENOMEM;

	dlpdata = dlp;

	dev_err(&cl->dev, " dlp3430_probe enter 2");
	dlp->client = cl;
	dlp->dev = &cl->dev;
	dlp->pdata = pdata;
	dlp->chipname = id->name;
	i2c_set_clientdata(cl, dlp);

	ret = misc_register(&dlp3430_dev);

	if (ret < 0)
		goto err;

	dlp3430_kobj = kobject_create_and_add("dlp3430", NULL);


	ret = sysfs_create_group(dlp3430_kobj, &dlp3430_attr_group);
	if (ret) {
		dev_err(dlp->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_sysfs;
	}
	dev_warn(&cl->dev, " dlp3430_probe enter exit 0");

	return 0;

err_sysfs:
	dev_warn(&cl->dev, " dlp3430_probe enter exit err");

	return ret;
err:	  
	kfree(dlp);
	dlp=NULL;
	dev_warn(&cl->dev, " dlp3430_probe enter exit err for register misc");
	return ret;
}

static int dlp3430_remove(struct i2c_client *cl)
{
	struct dlp3430 *lp = i2c_get_clientdata(cl);
	misc_deregister(&dlp3430_dev);
	dev_warn(&cl->dev, " dlp3430_remove enter exit 0");
	sysfs_remove_group(&lp->dev->kobj, &dlp3430_attr_group);
	return 0;
}

static const struct of_device_id dlp3430_dt_ids[] = {
	{ .compatible = "ti,dlp3430", },
	{ }
};
MODULE_DEVICE_TABLE(of, dlp3430_dt_ids);

static const struct i2c_device_id dlp3430_ids[] = {
	{"dlp3430", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, dlp3430_ids);

static struct i2c_driver dlp3430_i2c_driver = {
	.driver = {
		.name = "dlp3430",
		.of_match_table = of_match_ptr(dlp3430_dt_ids),
	},
	.probe = dlp3430_probe,
	.remove = dlp3430_remove,
	.id_table = dlp3430_ids,
};

//module_i2c_driver(dlp3430_driver);


static struct dlp3430_rom_data dlp3430_rom_data[1] = {{0x10, 0x84}};

struct dlp3430_platform_data dlp3430_platform_data = {
	.name = "dlp3430",
	.device_control = 0,
	.initial_brightness = 0,
	.period_ns = 5000000, /* 200 Hz */
	.size_program = 1,
	.rom_data = dlp3430_rom_data,
};

#define DLP_I2C_BUS_NUM			4
void *dlp3430_get_platform_data(void)
{
	return (void *)&dlp3430_platform_data;
}

static struct i2c_board_info __initdata dlp3430_i2c_device = {
	I2C_BOARD_INFO("dlp3430", 0x1b),
	//I2C_BOARD_INFO("dlp3430", 0x36),
};
#if 1
static int __init dlp3430_i2c_platform_init(void)
{
	//int i2c_busnum = 5;
	//struct i2c_board_info i2c_info;
	void *pdata = NULL;
	int ret = 0;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client = NULL;

	//intel_scu_ipc_msic_vprog3(1);
#if 0
	memset(void * s,int c,size_t count)(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, "dlp3430", strlen("dlp3430"));

	i2c_info.addr = 0x38;

	pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
			i2c_busnum,
			i2c_info.type,
			i2c_info.irq,
			i2c_info.addr);
#endif
	printk(KERN_ERR "[drm]%s: dlp3430 i2c device register.\n", __func__);
	pdata = dlp3430_get_platform_data();

	if(pdata != NULL)
		dlp3430_i2c_device.platform_data = pdata;
	else
		printk("%s, pdata is NULL\n", __func__);

#if 0
	ret = i2c_register_board_info(i2c_busnum, &dlp3430_i2c_device, 1);
	if(ret) {
		printk("[drm]%s: dlp3430 i2c device register error %d.\n", __func__, ret);
	}
#else
	adapter = i2c_get_adapter(DLP_I2C_BUS_NUM);
	if (!adapter) {
		printk(KERN_ERR "i2c_get_adapter(%u) failed.\n", DLP_I2C_BUS_NUM);
		ret = -EPERM;
		goto out;
	}

	client = i2c_new_device(adapter, &dlp3430_i2c_device);
	if (client == NULL) {
		printk(KERN_ERR "i2c_new_device(%u) failed.\n", DLP_I2C_BUS_NUM);
		ret = -EPERM;
	}
#endif

out:
	return ret;
}

//fs_initcall(dlp3430_i2c_platform_init);
device_initcall(dlp3430_i2c_platform_init);


static int __init dlp3430_i2c_bus_init(void)
{
	printk(KERN_ERR "[drm]%s: dlp3430 i2c driver register.\n", __func__);
	return i2c_add_driver(&dlp3430_i2c_driver);
}
module_init(dlp3430_i2c_bus_init);

static void __exit dlp3430_i2c_bus_exit(void)
{
	printk(KERN_ERR "[drm]%s: dlp3430 i2c driver delete.\n", __func__);
	i2c_del_driver(&dlp3430_i2c_driver);

	return;
}
module_exit(dlp3430_i2c_bus_exit);
#endif

MODULE_DESCRIPTION("Texas Instruments DLP3430");
MODULE_AUTHOR("zhurong fu <fuzr1@lenovo.com>");
MODULE_LICENSE("GPL");
