/*
 * Support for OmniVision ar1335 8MP camera sensor.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * ar1335 Data format conversion
 *
 * include AWB&LSC of light source table and AF table
 *
 */

#ifndef __KERNEL__
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define OTP_LOG printf
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
#else
#define OTP_LOG printk
#endif

#define DEBUG 0x1
#define DATA_BUF_SIZE 1024

#define ENULL_DW9761B	-200
#define ENULL_OTP 	-201

#define SNR_OTP_START	0x0400
#define SNR_OTP_END		0x066D

#define DW9761B_BYTE_MAX	32
#define DW9761B_SHORT_MAX	16

#define AR1335_SAVE_RAW_DATA                "/data/ar1335.otp"
#define AR1335_SAVE_OTP_DATA                "/data/ar1335_parsed.otp"
static int DW9761B_eflash_otp_trans(const unsigned char *ar1335_data_ptr, const int ar1335_size, unsigned char *otp_data_ptr, int *otp_size);
#ifdef __KERNEL__
static int DW9761B_eflash_otp_read(struct i2c_client *client, u8 *ar1335_data_ptr, u32 *ar1335_size);
#endif


enum dw9761b_tok_type {
	DW9761B_8BIT  = 0x0001,
	DW9761B_16BIT = 0x0002,
	DW9761B_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	DW9761B_TOK_DELAY  = 0xfe00	/* delay token for reg list */
};

