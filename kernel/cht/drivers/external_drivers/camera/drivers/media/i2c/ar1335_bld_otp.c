/*
 * Support for OmniVision ar1335 13MP camera sensor.
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * ar1335 Data format conversion
 *
 * include AWB&LSC of light source table and AF table 
 *
 */

#include "ar1335_bld_otp.h"

#ifdef __KERNEL__
static u32 DW9761B_eflash_otp_save(const u8 *databuf, u32 data_size, const u8 *filp_name)
{
	struct file *fp=NULL;
	mm_segment_t fs;
	loff_t pos;

	fp=filp_open(filp_name,O_CREAT|O_RDWR,0644);
	if(IS_ERR(fp))
		return -1;

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(fp, databuf, data_size, &pos);
	set_fs(fs);

	filp_close(fp,NULL);

	return 0;
}

static int
DW9761B_eflash_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[DW9761B_SHORT_MAX];
	int err, i;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > DW9761B_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0, sizeof(msg));
	memset(data, 0, sizeof(data));

	msg[0].addr = DW9761_I2C_EFLASH_ADDR;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = DW9761_I2C_EFLASH_ADDR;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == DW9761B_8BIT) {
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



static int DW9761B_eflash_otp_read(struct i2c_client *client, u8 *DW9761B_eflash_data_ptr, u32 *DW9761B_eflash_size)
{
	int i, ret;
	int address_start = SNR_OTP_START;
	int address_end = SNR_OTP_END;
	u16 val;

	printk("DW9761B_eflash OTP debug\n");



	for (i = 0; i <= (address_end - address_start + 1); i ++) {
		ret = DW9761B_eflash_read_reg(client, DW9761B_8BIT, i + address_start, &val);
		if (ret) {
			*DW9761B_eflash_size = 0;
			printk("read failed\n");
			return -1;
		}
		DW9761B_eflash_data_ptr[i] = (u8) (val & 0xff);
		//printk("addres:%x data:%x\n", i + address_start, val);
	}

	*DW9761B_eflash_size = address_end - address_start + 1;
	printk("DW9761B_eflash OTP read done\n");
	return 0;
}
#endif
//e.g for crc checksum table.
static uint16_t crc16table[] =
{
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

typedef struct otp_from_eeprom {
	//module info section
	uint8_t flag;  //0x0400
	uint8_t storage_location; //0x0401
	uint8_t lenovo_pn[10]; //0x0402-0x04b
	uint8_t id_defination[8]; //0x040c-0x0413
	uint8_t version[3]; //0x414-0x416
	uint8_t sensor_fuse_id[16]; //0x0417-0x0426
	uint8_t project_sku; // 0x0427
	uint8_t production_data[6];//0x0428-0x042d
	uint8_t mod_reserved[2]; //0x042E-F
	uint8_t mod_checksum[2]; //0x0430-0x0431
	//AF section
	uint8_t af_flag;//0x0432
	uint8_t af_direction;//0x0433
	uint8_t no_define[2];//0x0434-0x0435
	uint8_t af_infinity[2];//0x0436-0x0437
	uint8_t af_macro[2]; //0x0438- 0x0439
	uint8_t inf_scan_range[2]; //0x043a-b
	uint8_t macro_scan_range[2]; //0x043c-d
	uint8_t fast_af_setting[6]; //0x043E-0x0443
	uint8_t af_reserved[2]; //0x0444-0x0445
	uint8_t af_checksum[2]; //0x0446-7
	//awb& lsc light source 1
	uint8_t awb_lsc_source1_flag; //0x0448
	uint8_t awb_lsc_source1_version[3]; //0x0449-0x044b
	uint8_t source1_light[2]; //0x044c-0x044d
	//LSC of light source1
	uint8_t source1_lsc_w; //0x044e
	uint8_t source1_lsc_h; //0x044f
	uint8_t source1_lsc_fraq;//0x0450
	uint8_t source1_lsc_channel1[63];//0x0451-0x048f
	uint8_t source1_lsc_channel2[63];//0x0490-0x04ce
	uint8_t source1_lsc_channel3[63];//0x04cf-0x050d
	uint8_t source1_lsc_channel4[63];//0x050e-0x054c
	uint8_t source1_awb_channel1[2];//0x054d-e
	uint8_t source1_awb_channel2[2];//0x54f-50
	uint8_t source1_awb_channel3[2];//0x0551-2
	uint8_t source1_awb_channel4[2];//0x0553-4
	uint8_t awb_lsc_source1_reserved[2];//0x0555-6
	uint8_t awb_lsc_source1_checksum[2];//0x0557-8
	//awb& lsc light source 2
	uint8_t awb_lsc_source2_flag; //0x0559
	uint8_t awb_lsc_source2_version[3]; //0x055A-c
	uint8_t source2_light[2];//0x055d-e
	//LSC of light source2
	uint8_t source2_lsc_w; //0x055f
	uint8_t source2_lsc_h; //0x0560
	uint8_t source2_lsc_fraq;//0x0561
	uint8_t source2_lsc_channel1[63]; //0x0562-0x05a0
	uint8_t source2_lsc_channel2[63]; //0x05a1-0x05df
	uint8_t source2_lsc_channel3[63]; //0x05e0-0x061e
	uint8_t source2_lsc_channel4[63]; //0x061f-0x065d
	uint8_t source2_awb_channel1[2];//0x065e-f
	uint8_t source2_awb_channel2[2];//0x0660-1
	uint8_t source2_awb_channel3[2];//0x0662-3
	uint8_t source2_awb_channel4[2];//0x0664-5
	uint8_t awb_lsc_source2_reserved[2];//0x0666-7
	uint8_t intel_checksum[2]; //0x0668-9
	uint8_t awb_lsc_source2_checksum[2]; //0x066a-b
	//total checksum for 0x0400 -0x066b
	uint8_t total_checksum[2]; // 0x066c-d

}otp_from_eeprom_t;

int translate(void *pint, uint32_t incount, void *pout, uint32_t *outcount)
{
	uint8_t index, t, n_pos, n_lights, lsc_width, lsc_height, major_version, minor_version;
	uint16_t checksum_16 = 0;
	uint16_t i;
	uint16_t a_size, crc = 0;
	uint16_t sum_from_otp = 0;
	uint32_t checksum_mod, checksum_af, checksum_source1, checksum_source2, checksum_total,checksum_sunny_modified;
	uint8_t* t_pout = (uint8_t *)pout;
	int16_t * af_point;
	uint8_t* light_point;
	uint8_t * lsc_point1;
	uint8_t* lsc_point2;
	uint16_t* awb_point;

	otp_from_eeprom_t *t_pint = (otp_from_eeprom_t *)pint;

	if (pint == NULL && pout == NULL){
		OTP_LOG("error---pint or pout is null\n");
		return -1;
	}

	n_lights = t_pint->awb_lsc_source1_version[2];
	n_pos = 0x01;      // only  have one toward ,so set 0x01; 0 if N/A
	lsc_width = t_pint->source1_lsc_w;
	lsc_height = t_pint->source1_lsc_h;
	major_version = t_pint->awb_lsc_source1_version[0];
	minor_version = t_pint->awb_lsc_source1_version[1];

	//check sum module info
	checksum_mod = 0;
	for (i = 0; i< 48; i++){
		checksum_mod += *((uint8_t *)t_pint + i);
	}
	sum_from_otp = ((t_pint->mod_checksum[0] << 8) | t_pint->mod_checksum[1]);
	checksum_16 = (uint16_t)(0xffff & checksum_mod);
	if (checksum_16 != sum_from_otp){
		OTP_LOG("error--module info check sum is error expected:%x read:%x\n", checksum_16, sum_from_otp);
		return -1;
	}

	//check sum af
	checksum_af = 0;
	for (i = 0; i < 20; i++){
		checksum_af += *((uint8_t *)t_pint + 50 + i);
	}

	sum_from_otp = ((t_pint->af_checksum[0] << 8) | t_pint->af_checksum[1]);
	checksum_16 = (uint16_t)(0xffff & checksum_af);
	if (checksum_16 != sum_from_otp){
		OTP_LOG("error--af check sum is error! expected:%x read:%x\n", checksum_16, sum_from_otp);
		return -1;
	}

	//check sum awb lsc source 1
	checksum_source1 = 0;
	for (i = 0; i < 271; i++){
		checksum_source1 += *((uint8_t *)t_pint + 72 + i);
	}
	sum_from_otp = ((t_pint->awb_lsc_source1_checksum[0] << 8) | t_pint->awb_lsc_source1_checksum[1]);
	checksum_16 = (uint16_t)(0xffff & checksum_source1);
	if (checksum_16 != sum_from_otp){
		OTP_LOG("error--source1 check sum is error! expected:%x read:%x\n", checksum_16, sum_from_otp);
		return -1;
	}

	//check sum awb lsc source 2 if exist
	checksum_source2 = 0;
	for (i = 0; i < 273; i++){
		checksum_source2 += *((uint8_t *)t_pint + 345 + i);
	}
	sum_from_otp = ((t_pint->awb_lsc_source2_checksum[0] << 8) | t_pint->awb_lsc_source2_checksum[1]);
	checksum_16 = (uint16_t)(0xffff & checksum_source2);
	if (checksum_16 != sum_from_otp){
		OTP_LOG("error--source2 check sum is error! expected:%x read:%x\n", checksum_16, sum_from_otp);
		return -1;
	}

	//total check sum
	checksum_total = checksum_mod + checksum_af + checksum_source2 + checksum_source1 +
		t_pint->mod_checksum[0] + t_pint->mod_checksum[1] +
		t_pint->af_checksum[0] + t_pint->af_checksum[1] +
		t_pint->awb_lsc_source1_checksum[0] + t_pint->awb_lsc_source1_checksum[1] +
		t_pint->awb_lsc_source2_checksum[0] + t_pint->awb_lsc_source2_checksum[1];
	sum_from_otp = ((t_pint->total_checksum[0] << 8) | t_pint->total_checksum[1]);
	checksum_16 = (uint16_t)(0xffff & checksum_total);
	if (checksum_16 != sum_from_otp){
		OTP_LOG("error--total check sum is error! expected:%x read:%x\n", checksum_16, sum_from_otp);
		return -1;
	}

	/*
	 *As sunny change the calculation of all data check from intel CRC
	 *to to Sunny method. We recalculate it here
	 *the method is
	 *SUM(0x0433~0x0443,0x0449~0x0554,0x055A~0x0665) & 0xffff
	 */
	checksum_sunny_modified = 0;
	for (i = 0x433; i <= 0x443 ; i++) {
		checksum_sunny_modified += *((uint8_t *)t_pint + i - 0x400);
	}

	for (i = 0x449; i <= 0x554 ; i++) {
		checksum_sunny_modified += *((uint8_t *)t_pint + i - 0x400);
	}

	for (i = 0x55a; i <= 0x665; i++) {
		checksum_sunny_modified += *((uint8_t *)t_pint + i - 0x400);
	}
	checksum_sunny_modified &= 0xffff;
	sum_from_otp = ((t_pint->intel_checksum[0]) | (t_pint->intel_checksum[1] << 8));
	if (checksum_sunny_modified != sum_from_otp){
		OTP_LOG("intel checksum failed! expected: %x read:%x\n", checksum_sunny_modified, sum_from_otp);
		return -1;
	}
	//ouput to pout

	*(t_pout) = major_version;//major_version,offset = 0;
	*(t_pout + 1) = minor_version;//minor_version,offset = 1;
	*(t_pout + 2) = n_pos;//n_pos,offset = 2;
	*(t_pout + 3) = t_pint->no_define[0];//vcm_lps_bits,offset = 3;
	*(t_pout + 4) = t_pint->no_define[1]; //vcm_bits,offset = 4,
	af_point = (int16_t *)(t_pout + 5);
	*af_point = (int16_t)(((t_pint->af_macro[0] & 0x03) << 8) | t_pint->af_macro[1]);//af near = macro
	*(af_point + 1) = (int16_t)(((t_pint->af_infinity[0] & 0x03) << 8) | t_pint->af_infinity[1]);//af far = infinity  
	*(af_point + 2) = *(af_point + 1); // start = af far
	*(af_point + 3) = *af_point; //stop = af near
	light_point = (uint8_t*)(af_point + 4); //n_lights

	*light_point = t_pint->awb_lsc_source1_version[2];//n_lights
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(light_point + 1 + i) = t_pint->source1_light[0];
		}
		if (i == 1){
			*(light_point + 1 + i) = t_pint->source2_light[0];
		}
	}                             //ciex
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(light_point + n_lights + 1 + i) = t_pint->source1_light[1];
		}
		if (i == 1){
			*(light_point + n_lights + 1 + i) = t_pint->source2_light[1];
		}
	}                             //ciey
	*(light_point + 2 * n_lights + 1) = lsc_width;
	*(light_point + 2 * n_lights + 2) = lsc_height;
	*(light_point + 2 * n_lights + 3) = t_pint->source1_lsc_fraq; //fraq source1;
	lsc_point1 = (light_point + 2 * n_lights + 4); //lsc source1
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point1 + i*lsc_width + t) = t_pint->source1_lsc_channel1[i*lsc_width + t];
	}   //channel 1 source 1 ok
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point1 + lsc_width*lsc_height + i*lsc_width + t) = t_pint->source1_lsc_channel2[i*lsc_width + t];
	}   //channel 2 source 1
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point1 + 2 * lsc_width*lsc_height + i*lsc_width + t) = t_pint->source1_lsc_channel3[i*lsc_width + t];
	}   //channel 3 source 1
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point1 + 3 * lsc_width*lsc_height + i*lsc_width + t) = t_pint->source1_lsc_channel4[i*lsc_width + t];
	}   //channel 4 source 1

	*(lsc_point1 + 4 * lsc_width*lsc_height) = t_pint->source2_lsc_fraq; //fraq source2;
	lsc_point2 = lsc_point1 + 4 * lsc_width*lsc_height + 1;
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++){
			*(lsc_point2 + i*lsc_width + t) = t_pint->source2_lsc_channel1[i*lsc_width + t];
		}
	}   //channel 1 source 2
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point2 + lsc_width*lsc_height + i*lsc_width + t) = t_pint->source2_lsc_channel2[i*lsc_width + t];
	}   //channel 2 source 2
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point2 + 2 * lsc_width*lsc_height + i*lsc_width + t) = t_pint->source2_lsc_channel3[i*lsc_width + t];
	}   //channel 3 source 2
	for (i = 0; i < lsc_height; i++){
		for (t = 0; t < lsc_width; t++)
			*(lsc_point2 + 3 * lsc_width*lsc_height + i*lsc_width + t) = t_pint->source2_lsc_channel4[i*lsc_width + t];
	}   //channel 4 source 2

	awb_point = (uint16_t*)(lsc_point2 + 4 * lsc_width*lsc_height);//awb
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(awb_point + i) = (uint16_t)((t_pint->source1_awb_channel1[0]) | (t_pint->source1_awb_channel1[1] << 8));
		}
		if (i == 1){
			*(awb_point + i) = (uint16_t)((t_pint->source2_awb_channel1[0]) | (t_pint->source2_awb_channel1[1] << 8));
		}
	}   //awb channel 1, including source1 & source2;if existed
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(awb_point + n_lights + i) = (uint16_t)((t_pint->source1_awb_channel2[0]) | (t_pint->source1_awb_channel2[1] << 8));
		}
		if (i == 1){
			*(awb_point + n_lights + i) = (uint16_t)((t_pint->source2_awb_channel2[0]) | (t_pint->source2_awb_channel2[1] << 8));
		}
	}   //awb channel 2, including source1 & source2;if existed
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(awb_point + 2 * n_lights + i) = (uint16_t)((t_pint->source1_awb_channel3[0]) | (t_pint->source1_awb_channel3[1] << 8));
		}
		if (i == 1){
			*(awb_point + 2 * n_lights + i) = (uint16_t)((t_pint->source2_awb_channel3[0]) | (t_pint->source2_awb_channel3[1] << 8));
		}
	}   //awb channel 3,including source1 & source2;if existed
	for (i = 0; i < n_lights; i++){
		if (i == 0){
			*(awb_point + 3 * n_lights + i) = (uint16_t)((t_pint->source1_awb_channel4[0]) | (t_pint->source1_awb_channel4[1] << 8));
		}
		if (i == 1){
			*(awb_point + 3 * n_lights + i) = (uint16_t)((t_pint->source2_awb_channel4[0]) | (t_pint->source2_awb_channel4[1] << 8));
		}
	}   //awb channel 4;including source1 & source2;if existed

	//checksum the last one
	a_size = 8 + 8 * n_pos + 11 * n_lights + 4 * n_lights*lsc_width*lsc_height; // before all data size of intel camera
	for (i = 0; i < a_size; i++)
	{
		index = crc ^ t_pout[i];
		crc = (crc >> 8) ^ crc16table[index];
	}
	*(awb_point + 4 * n_lights) = crc;

	//dump some important info for debug
	OTP_LOG("otp_test--module info flag : %x--af flag:%x--source1 flag:%x--source2 flag:%x\n", t_pint->flag, t_pint->af_flag, t_pint->awb_lsc_source1_flag, t_pint->awb_lsc_source2_flag);
	OTP_LOG("otp_test--ID defination-supplier:%x--sensor:%x--lens:%x-vcm:%x \n", t_pint->id_defination[0], t_pint->id_defination[1], t_pint->id_defination[2], t_pint->id_defination[3]);
	OTP_LOG("otp_test--project sku:%x\n", t_pint->project_sku);
	OTP_LOG("otp_test--project data factory:%x--year:%x-month:%x-day:%x-hour:%x-min:%x \n", t_pint->production_data[0], t_pint->production_data[1], t_pint->production_data[2], t_pint->production_data[3], t_pint->production_data[4], t_pint->production_data[5]);

	*outcount = a_size + 2;
	//is ok
	return 0;

}

static int DW9761B_eflash_otp_trans(const unsigned char *DW9761B_eflash_data_ptr, const int DW9761B_eflash_size, unsigned char *otp_data_ptr, int *otp_size)
{
	int ret = 0;

	if(DW9761B_eflash_data_ptr == NULL || DW9761B_eflash_size == 0)
		return ENULL_DW9761B;

	if(otp_data_ptr == NULL || otp_size == 0)
		return ENULL_OTP;

	ret = translate((void *)DW9761B_eflash_data_ptr, DW9761B_eflash_size, otp_data_ptr, otp_size);
	/* initialize DW9761B_eflash_data_group_pos */

	return ret;
}

