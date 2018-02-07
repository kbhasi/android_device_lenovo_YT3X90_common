#include "ov5693_kobe_otp.h"

struct otp_struct {
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int light_rg;
	int light_bg;
	int user_data[5];
	int lenc[62];
	int VCM_start;
	int VCM_end;
};
// return: 0, read otp success
// 1, read otp failed
static struct i2c_client *client_for_otp;

static unsigned int debug_level = 1;
module_param(debug_level, int, 0644);

static unsigned int disable_otp = 0;
module_param(disable_otp, int, 0644);



#define OV5693_OTP_DEBUG(level, a, ...) \
	do { \
		if (level < debug_level) \
			printk(a,## __VA_ARGS__); \
	} while (0)

static unsigned char ov5693_lsc_array[64] = {0x58, 0x00};
static int ov5693_r_gain, ov5693_g_gain, ov5693_b_gain;
static int otp_data_valid = 0;


void OV5693_write_i2c(u16 addr, int data)
{
	ov5693_write_reg(client_for_otp, OV5693_8BIT, addr, data);
}

void OV5693_write_16bit_i2c(u16 addr, int data)
{
       ov5693_write_reg(client_for_otp, OV5693_16BIT, addr, data);
}

void Delay(int times)
{
	msleep(times);
}

int OV5693_read_i2c(int addr)
{
	int ret;
	u16 val;

	ret = ov5693_read_reg(client_for_otp, OV5693_8BIT, addr, &val);

	if(ret) {
		printk("read otp from %d failed\n", addr);
	}

	return val;
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_wb(int index)
{
	int flag, i;
	int bank, address;
	// select bank index
	bank = 0xc0 | index;
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	// read flag
	address = 0x3d00;
	flag = OV5693_read_i2c(address);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	if (flag == 0x00)
	{
		return 0;
	}
	else if (flag & 0x80)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_lenc(int index)
{
	int flag, i, bank;
	int address;
	// select bank: 4, 8, 12
	bank = 0xc0 | (index * 4);
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer

	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	// read flag
	address = 0x3d00;
	flag = OV5693_read_i2c(address);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	if (flag == 0x00)
	{
		return 0;
	}
	else if (flag & 0x80)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}
// index: index of otp group. (1, 2, 3)
// code: 0 for start code, 1 for stop code
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_VCM(int index, int code)
{
	int flag, i, bank;
	int address;
	// select bank: 16
	bank = 0xc0 + 16;
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	// read flag
	address = 0x3d00 + (index-1)*4 + code*2;
	flag = OV5693_read_i2c(address);
	flag = flag & 0xc0;
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	if (flag == 0x00)
	{
		return 0;
	}
	else if (flag & 0x80)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}
// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_wb(int index, struct otp_struct *otp_ptr)
{
	int i, bank;
	int address;
	int temp;
	// select bank index
	bank = 0xc0 | index;
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	address = 0x3d00;
	(*otp_ptr).module_integrator_id = OV5693_read_i2c(address + 1);
	(*otp_ptr).lens_id = OV5693_read_i2c(address + 2);
	(*otp_ptr).production_year = OV5693_read_i2c(address + 3);
	(*otp_ptr).production_month = OV5693_read_i2c(address + 4);
	(*otp_ptr).production_day = OV5693_read_i2c(address + 5);
	temp = OV5693_read_i2c(address + 10);
	(*otp_ptr).rg_ratio = (OV5693_read_i2c(address + 6)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV5693_read_i2c(address + 7)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV5693_read_i2c(address + 8) <<2)+ ((temp>>2) &
	0x03);
	(*otp_ptr).light_bg = (OV5693_read_i2c(address + 9)<<2) + (temp & 0x03);
	(*otp_ptr).user_data[0] = OV5693_read_i2c(address + 11);
	(*otp_ptr).user_data[1] = OV5693_read_i2c(address + 12);
	(*otp_ptr).user_data[2] = OV5693_read_i2c(address + 13);
	(*otp_ptr).user_data[3] = OV5693_read_i2c(address + 14);
	(*otp_ptr).user_data[4] = OV5693_read_i2c(address + 15);
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	return 0;
}
// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_lenc(int index, struct otp_struct *otp_ptr)
{
	int bank, i;
	int address;
	// select bank: 4, 8, 12
	bank = 0xc0 | (index * 4);
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	address = 0x3d01;
	for(i=0;i<15;i++)
	{
		(* otp_ptr).lenc[i]=OV5693_read_i2c(address);
		address++;
	}
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	// select 2nd bank
	bank++;
	OV5693_write_i2c(0x3d84, bank);
	// read otp
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	address = 0x3d00;
	for(i=15;i<31;i++)
	{
		(* otp_ptr).lenc[i]=OV5693_read_i2c(address);
		address++;
	}
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	// select 3rd bank
	bank++;
	OV5693_write_i2c(0x3d84, bank);
	// read otp
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	address = 0x3d00;
	for(i=31;i<47;i++)
	{
		(* otp_ptr).lenc[i]=OV5693_read_i2c(address);
		address++;
	}
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	// select 4th bank
	bank++;
	OV5693_write_i2c(0x3d84, bank);
	// read otp
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	address = 0x3d00;
	for(i=47;i<62;i++)
	{
		(* otp_ptr).lenc[i]=OV5693_read_i2c(address);
		address++;
	}
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}


	OV5693_OTP_DEBUG(2, "\n");
	return 0;
}
// index: index of otp group. (1, 2, 3)
// code: 0 start code, 1 stop code
// return: 0
int read_otp_VCM(int index, int code, struct otp_struct * otp_ptr)
{
	int vcm, i, bank;
	int address;
	// select bank: 16
	bank = 0xc0 + 16;
	OV5693_write_i2c(0x3d84, bank);
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(1);
	// read flag
	address = 0x3d00 + (index-1)*4 + code*2;
	vcm = OV5693_read_i2c(address);
	vcm = (vcm & 0x03) + (OV5693_read_i2c(address+1)<<2);
	if(code==1)
	{
		(* otp_ptr).VCM_end = vcm;
	}
	else
	{
		(* otp_ptr).VCM_start = vcm;
	}
	// clear otp buffer
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
       ov5693_r_gain = R_gain;
       ov5693_g_gain = G_gain;
       ov5693_b_gain = B_gain;

	return 0;
}
// otp_ptr: pointer of otp_struct
int update_lenc(struct otp_struct * otp_ptr)
{
       int i;

       for (i = 0;i < 62; i++) {
               ov5693_lsc_array[i + 2] = (*otp_ptr).lenc[i];
        }

	return 0;
}

void update_otp_to_sensor(void)
{
       struct i2c_msg lsc_msg;
       int ret = 0;

       if (disable_otp) {
               OV5693_OTP_DEBUG(0, "OV5693 OTP is disabled by module params\n");
               return;
       }
       if (!otp_data_valid) {
               OV5693_OTP_DEBUG(0, "OV5693 OTP is not valid\n");
               return;
       }

       if (otp_data_valid) {
               int temp;
               /*LSC Data update*/
               temp = OV5693_read_i2c(0x5000);
               temp = 0x80 | temp;

               lsc_msg.addr = client_for_otp->addr;
               lsc_msg.flags = 0;
               lsc_msg.len = sizeof(ov5693_lsc_array);
               lsc_msg.buf = ov5693_lsc_array;
               ret = i2c_transfer(client_for_otp->adapter, &lsc_msg, 1);
               if (ret != 1)
                       OV5693_OTP_DEBUG(0, "OV5693 LSC Write Failed\n");
               OV5693_write_i2c(0x5000, temp);

               if (ov5693_r_gain>0x400)
               {
                       OV5693_write_16bit_i2c(0x3400,ov5693_r_gain);
               }
               if (ov5693_g_gain>0x400)
               {
                       OV5693_write_16bit_i2c(0x3402,ov5693_g_gain);
               }
               if (ov5693_b_gain>0x400)
               {
                       OV5693_write_16bit_i2c(0x3404,ov5693_b_gain);
               }
               if (debug_level >OV5693_OTP_DEBUG_LEVEL) {
                       int i;

                       for (i = 0; i < sizeof(ov5693_lsc_array); i++) {
                               OV5693_OTP_DEBUG(2, "%x ", ov5693_lsc_array[i]);
                               if (i % 8 == 7) OV5693_OTP_DEBUG(2, "\n");
                       }
                       OV5693_OTP_DEBUG(2, "ov5693_r_gain: %x ", ov5693_r_gain);
                       OV5693_OTP_DEBUG(2, "ov5693_g_gain: %x ", ov5693_g_gain);
                       OV5693_OTP_DEBUG(2, "ov5693_b_gain: %x ", ov5693_b_gain);
               }
       }
}

// call this function after OV5693 initialization
// return value: 0 update success
// 1, no OTP
int update_otp_wb(void)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;
	// R/G and B/G of current camera module is read out from sensorOTP
	// check first OTP with valid data
	for(i=1;i<=3;i++)
	{
		temp = check_otp_wb(i);
		if (temp == 2)
		{
			otp_index = i;
			break;
		}
	}
	if (i>3)
	{
		// no valid wb OTP data
		OV5693_OTP_DEBUG(0, "%s %d no valid awb data\n", __func__, __LINE__);
		return 1;
	}
	OV5693_OTP_DEBUG(2, "%s %d find awb index:%d\n", __func__, __LINE__, otp_index);

	read_otp_wb(otp_index, &current_otp);

	OV5693_OTP_DEBUG(2, "%s %d light_rg:%d rg_ratio:%d light_bg:%d bg_ratio:%d\n", __func__, __LINE__,
						current_otp.light_rg, current_otp.rg_ratio,
						current_otp.light_bg, current_otp.bg_ratio);

	if(current_otp.light_rg==0)
	{
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	}
	else
	{
		rg = current_otp.rg_ratio * ((current_otp.light_rg +512) / 1024);
	}
	if(current_otp.light_bg==0)
	{
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	}
	else
	{
		bg = current_otp.bg_ratio * ((current_otp.light_bg +512) / 1024);
	}
	//calculate G gain
	//0x400 = 1x gain
	if(bg < BG_Ratio_Typical)
	{
		if (rg< RG_Ratio_Typical)
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		}
		else
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else
	{
		if (rg < RG_Ratio_Typical)
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R )
			{
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else
			{
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}
	OV5693_OTP_DEBUG(2, "%s %d find awb R_gain:%d G_gain:%d B_gain:%d\n", __func__, __LINE__,
						R_gain, G_gain, B_gain);
	update_awb_gain(R_gain, G_gain, B_gain);
	return 0;
}
// call this function after OV5693 initialization
// return value: 0 update success
// 1, no OTP
int update_otp_lenc(void)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=3;i++)
	{
		temp = check_otp_lenc(i);
	if (temp == 2)
	{
		otp_index = i;
		break;
	}
	}
	if (i>3)
	{
		// no valid LSC OTP data
		OV5693_OTP_DEBUG(0, "%s %d no valid LENS\n", __func__, __LINE__);
		return 1;
	}
	OV5693_OTP_DEBUG(2, "%s %d find LENS index:%d\n", __func__, __LINE__, otp_index);
	read_otp_lenc(otp_index, &current_otp);
	update_lenc(&current_otp);
	// success
	return 0;
}
// call this function after OV5693 initialization
// return value: 1 use CP data from REG3D0A
// 2 use Module data from REG3D0A
// 0 data ErRoR
int update_blc_ratio(void)
{
	int K;
	int temp;
	OV5693_write_i2c(0x3d84, 0xdf);
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);

	K = OV5693_read_i2c(0x3d0b);
	if (K != 0)
	{
		if (K >= 0x15 && K <= 0x40)
		{
			// auto load mode
			temp = OV5693_read_i2c(0x4008);
			temp &= 0xfb;
			OV5693_write_i2c(0x4008, temp);
			temp = OV5693_read_i2c(0x4000);
			temp &= 0xf7;
			OV5693_write_i2c(0x4000, temp);
			return 2;
		}
	}
	K = OV5693_read_i2c(0x3d0a);
	if (K >= 0x10 && K <= 0x40)
	{
		// manual load mode
		OV5693_write_i2c(0x4006, K);
		temp = OV5693_read_i2c(0x4008);
		temp &= 0xfb;
		OV5693_write_i2c(0x4008, temp);
		temp = OV5693_read_i2c(0x4000);
		temp |= 0x08;
		OV5693_write_i2c(0x4000, temp);
		return 1;
	}
	else
	{
		// set to default
		OV5693_write_i2c(0x4006, 0x20);
		temp = OV5693_read_i2c(0x4008);
		temp &= 0xfb;
		OV5693_write_i2c(0x4008, temp);
		temp = OV5693_read_i2c(0x4000);
		temp |= 0x08;
		OV5693_write_i2c(0x4000, temp);
		return 0;
	}
}

void Read_OTP_All_Banks(void)
{
	int i, j;

	unsigned char otp_data[16];

	for (i = 0; i < 16; i++) {
		OV5693_write_i2c(0x3d84, 0xc0|i); //read bank 0
	// read otp into buffer
		OV5693_write_i2c(0x3d81, 0x01);
		Delay(5);
		for (j = 0; j < 16; j++) {
			otp_data[j] = OV5693_read_i2c(0x3d00 + j); //fuse id first byte
		}
		OV5693_OTP_DEBUG(2, "bank %d:\n", i);
		OV5693_OTP_DEBUG(2, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n",
			otp_data[0],otp_data[1],otp_data[2],otp_data[3],
			otp_data[4],otp_data[5],otp_data[6],otp_data[7],
			otp_data[8],otp_data[9],otp_data[10],otp_data[11],
			otp_data[12],otp_data[13],otp_data[14],otp_data[15]);
	}
}
int Read_OTP(struct i2c_client *client)
{
	int Sensor_type, i;

	if (disable_otp) {
		OV5693_OTP_DEBUG(0, "OV5693 OTP is disabled by module params\n");
		return -1;
	}

	client_for_otp = client;
	ov5693_write_reg(client, OV5693_8BIT, OV5693_SW_STREAM,
				OV5693_START_STREAMING);

	if (debug_level > 5)
		Read_OTP_All_Banks();

	OV5693_write_i2c(0x3d84, 0xc0); //read bank 0
	// read otp into buffer
	OV5693_write_i2c(0x3d81, 0x01);
	Delay(5);
	Sensor_type = OV5693_read_i2c(0x3d00); //fuse id first byte
	//or if(Sensor_type == 0x93)
	if(Sensor_type != 0x3A) {
		OV5693_OTP_DEBUG(2, "%s %d sensor type failed:%x\n", __func__, __LINE__, Sensor_type);
		return 1;
	}
	if(1==update_otp_wb()) {
		OV5693_OTP_DEBUG(2, "%s %d update_otp_wb failed\n", __func__, __LINE__);
		return 1; //have no wb data
	}
	if(1==update_otp_lenc()) {
		OV5693_OTP_DEBUG(2, "%s %d update_otp_lenc failed\n", __func__, __LINE__);
		return 1; //have no lens data
	}
        otp_data_valid = 1;
	for (i=0;i<16;i++)
	{
		OV5693_write_i2c(0x3d00 + i, 0x00);
	}
	//ov5693_write_reg(client, OV5693_8BIT, OV5693_SW_STREAM,
	//			OV5693_STOP_STREAMING);
       ov5693_write_reg(client, OV5693_8BIT, OV5693_SW_STREAM,
                               OV5693_STOP_STREAMING);

	return 0;
}


