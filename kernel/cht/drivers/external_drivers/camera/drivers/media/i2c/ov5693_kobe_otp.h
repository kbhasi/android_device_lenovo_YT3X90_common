#ifndef _OV5693_KOBE_OTP_H_
#define _OV5693_KOBE_OTP_H_
/*
#ifdef __KERNEL__
#define OTP_LOG printk
#else
#include <stdio.h>
#include <string.h>

#define OTP_LOG printf

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
#endif
*/

//#define CAMERA_OTP_DEBUG
#ifdef CAMERA_OTP_DEBUG
#define OTP_LOG(fmt, args...) printk(fmt, ##args)
#else
#define OTP_LOG(fmt, args...) do { } while (0)
#endif


/*****OV5693 OTP Save START*****/
#define OV5693_SAVE_OTP_DATA		"/data/ov5693_parsed.otp"
#define OV5693_SAVE_RAW_DATA		"/data/ov5693.otp"

#define DATA_BUF_SIZE 1024

#define BG_Ratio_Typical 280
#define RG_Ratio_Typical 283


#define OV5693_OTP_DEBUG_LEVEL 1


/*****OV5693 OTP conversion START*****/
static int ov5693_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val);

static int ov5693_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val);



#endif
