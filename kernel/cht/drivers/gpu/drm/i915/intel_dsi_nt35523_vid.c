/*
 * Copyright © 2013 Intel Corporation
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
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "intel_dsi_nt35523_vid.h"
#include <linux/mutex.h>
#include <linux/mfd/intel_soc_pmic.h>
extern bool bk_status ;
extern int chv_get_lcd_id(void);
extern int dsi_vc_send_long(struct intel_dsi *intel_dsi, int channel,
			    u8 data_type, const u8 *data, int len);
#define PMIC_VSWITCH_REG	0x6E69
#define PMIC_VSWITCH2_EN	0x20
#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE

struct mutex lcd_mutex;
/* adjust the effect level and index and save the cmd for backlight on after backlight off ++++*/
static u8 cabc_on_ce_on[] = {0x55,0x82}; // CABC on and CE on
static u8 cabc_on_ce_off[] = {0x55,0x02}; // CABC on and CE off  // yuyao modify 0x01->0x02 on 20150731
static u8 cabc_off_ce_on[]={0x55,0x80}; //CABC off and CE on
static u8 cabc_off_ce_off[]={0x55,0x00}; //CABC off and CE off
static u8* save_cmd=cabc_on_ce_off;     // Turn off CE , will be replaced by Technicolor.
/* adjust the effect level and index and save the cmd for backlight on after backlight off -----*/
static int auo_nt33523_set_gamma(int newgamma,struct intel_dsi *intel_dsi)
{
	u8 cmd1[]={ 0xF0,0x55,0xAA,0x52,0x08,0x02};
	u8 cmd2[]={ 0xB0,0xC0};

	//R(+) MCR cmd
	u8 cmd3[]={ 0xD1,0x00,0x60,0x00,0x65,0x00,0x80,0x00,0x9E,0x00,0xAF,0x00,0xCD,0x00,0xEB,0x01,0x17};
	u8 cmd4[]={ 0xD2,0x01,0x3D,0x01,0x75,0x01,0xA3,0x01,0xE7,0x02,0x1F,0x02,0x20,0x02,0x50,0x02,0x80};
	u8 cmd5[]={ 0xD3,0x02,0x9A,0x02,0xB5,0x02,0xC3,0x02,0xCD,0x02,0xD2,0x02,0xD4,0x02,0xD5,0x02,0xD6};
	u8 cmd6[]={ 0xD4,0x02,0xD7,0x02,0xD8};
	//G(+) MCR cmd
	u8 cmd7[]={ 0xD5,0x01,0x55,0x01,0x59,0x01,0x5F,0x01,0x64,0x01,0x6A,0x01,0x75,0x01,0x7F,0x01,0x92};
	u8 cmd8[]={ 0xD6,0x01,0xA4,0x01,0xC4,0x01,0xE0,0x02,0x11,0x02,0x3B,0x02,0x3D,0x02,0x65,0x02,0x91};
	u8 cmd9[]={ 0xD7,0x02,0xAC,0x02,0xC8,0x02,0xD5,0x02,0xE0,0x02,0xE3,0x02,0xE6,0x02,0xE7,0x02,0xE8};
	u8 cmd10[]={ 0xD8,0x02,0xE9,0x02,0xEA};
	//B(+) MCR cmd
	u8 cmd11[]={ 0xD9,0x01,0xB9,0x01,0xBA,0x01,0xBE,0x01,0xC0,0x01,0xC3,0x01,0xC9,0x01,0xD0,0x01,0xDB};
	u8 cmd12[]={ 0xDD,0x01,0xE6,0x01,0xFA,0x02,0x0E,0x02,0x31,0x02,0x53,0x02,0x54,0x02,0x79,0x02,0xA4};
	u8 cmd13[]={ 0xDE,0x02,0xBF,0x02,0xDC,0x02,0xEA,0x02,0xF5,0x02,0xFA,0x02,0xFD,0x02,0xFE,0x02,0xFF};
	u8 cmd14[]={ 0xDF,0x03,0x00,0x03,0x01};

	//R(+) MCR cmd
	u8 cmd23[]={ 0xD1,0x00,0x00,0x00,0x23,0x00,0x50,0x00,0x71,0x00,0x8A,0x00,0xB3,0x00,0xD4,0x01,0x06};
	u8 cmd24[]={ 0xD2,0x01,0x2E,0x01,0x6D,0x01,0x9D,0x01,0xE6,0x02,0x20,0x02,0x21,0x02,0x53,0x02,0x8A};
	u8 cmd25[]={ 0xD3,0x02,0xAE,0x02,0xDF,0x03,0x00,0x03,0x2F,0x03,0x4C,0x03,0x6F,0x03,0x84,0x03,0xA2};
	u8 cmd26[]={ 0xD4,0x03,0xC8,0x03,0xFF};
	//G(+) MCR cmd
	u8 cmd27[]={ 0xD5,0x00,0x00,0x00,0x22,0x00,0x4F,0x00,0x6F,0x00,0x89,0x00,0xB1,0x00,0xD2,0x01,0x04};
	u8 cmd28[]={ 0xD6,0x01,0x2D,0x01,0x6B,0x01,0x9B,0x01,0xE4,0x02,0x1E,0x02,0x1F,0x02,0x51,0x02,0x87};
	u8 cmd29[]={ 0xD7,0x02,0xAB,0x02,0xDD,0x02,0xFF,0x03,0x2D,0x03,0x49,0x03,0x6A,0x03,0x83,0x03,0xA0};
	u8 cmd30[]={ 0xD8,0x03,0xC3,0x03,0xFF};
	//B(+) MCR cmd
	u8 cmd31[]={ 0xD9,0x00,0x9F,0x00,0xA8,0x00,0xB8,0x00,0xC7,0x00,0xD5,0x00,0xEF,0x01,0x04,0x01,0x2A};
	u8 cmd32[]={ 0xDD,0x01,0x49,0x01,0x7D,0x01,0xA7,0x01,0xEA,0x02,0x20,0x02,0x21,0x02,0x52,0x02,0x88};
	u8 cmd33[]={ 0xDE,0x02,0xAC,0x02,0xE0,0x03,0x06,0x03,0x41,0x03,0x7C,0x03,0xD9,0x03,0xE0,0x03,0xEE};
	u8 cmd34[]={ 0xDF,0x03,0xFB,0x03,0xFF};

	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd1,sizeof(cmd1));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd2,sizeof(cmd2));
	if(newgamma>0)
	{
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd3,sizeof(cmd3));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd4,sizeof(cmd4));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd5,sizeof(cmd5));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd6,sizeof(cmd6));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd7,sizeof(cmd7));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd8,sizeof(cmd8));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd9,sizeof(cmd9));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd10,sizeof(cmd10));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd11,sizeof(cmd11));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd12,sizeof(cmd12));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd13,sizeof(cmd13));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd14,sizeof(cmd14));
	}
	else
	{
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd23,sizeof(cmd23));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd24,sizeof(cmd24));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd25,sizeof(cmd25));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd26,sizeof(cmd26));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd27,sizeof(cmd27));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd28,sizeof(cmd28));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd29,sizeof(cmd29));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd30,sizeof(cmd30));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd31,sizeof(cmd31));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd32,sizeof(cmd32));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd33,sizeof(cmd33));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd34,sizeof(cmd34));
	}
	return 0;
}

static int auo_nt35523_get_effect_index(char *name)
{
	int i = 0;
	struct lcd_effect_data *effect_data= auo_nt35523_data.lcd_effects;

	for(i = 0; i < effect_data->supported_effects; i++)
		if(!strcmp(name, effect_data->lcd_effects[i].name))
			return i;

	return -1;
}
static int auo_nt35523_set_effect(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int  ret = 0;
	int effect_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_effect_data *effect_data= auo_nt35523_data.lcd_effects;
	//struct lcd_effect effect = effect_data->lcd_effects[effect_index];

	//struct lcd_effect_cmd effect_cmd = effect.lcd_effect_cmds[level];
	//int cmd_nums = effect_cmd.cmd_nums;
	//struct lcd_cmd cmd;
	dsi->hs = 1;

	printk("[LCD]:==jinjt==%s line=%d,effect_index=%d,level=%d\n",__func__,__LINE__,effect_index,level);
//	if(level < 0 || level > effect.max_level)
//		return -EINVAL;

	//store the effect level
	effect_data->lcd_effects[effect_index].current_level = level;

    if(effect_index == 1)
    {
        if(level == 0)
        {
            save_cmd = cabc_off_ce_off;
        }
        else if(level ==1)
        {
            save_cmd = cabc_on_ce_off;
        }
        else if(level ==2)
        {
           save_cmd = cabc_off_ce_on;
        }
        else if(level ==3)
        {
           save_cmd = cabc_on_ce_on;

        }
    }
   else if(effect_index == 2)
    {
        if(level == 0)
        {
           save_cmd = cabc_off_ce_off;
        }
        else
        {
            save_cmd = cabc_off_ce_on;
        }

    }
//if(auo_nt35523_panel_device.status == OFF) return ret;

    dsi_vc_send_long(dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE, save_cmd, 2);
	DRM_DEBUG_DRIVER("[LCD]: %s line=%d effect_index=%d level=%d\n",__func__,__LINE__,effect_index,level);
	return ret;

}

static int auo_nt35523_get_current_level(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= auo_nt35523_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}

static int auo_nt35523_set_mode(struct hal_panel_ctrl_data *hal_panel_ctrl, struct intel_dsi *dsi)
{
	int i = 0, ret = 0;
	int mode_index = hal_panel_ctrl->index;
	int level = hal_panel_ctrl->level;
	struct lcd_mode_data *mode_data= auo_nt35523_data.lcd_modes;
	struct lcd_mode mode = mode_data->lcd_modes[mode_index];
	struct lcd_mode_cmd mode_cmd = mode.lcd_mode_cmds[level];
	int cmd_nums = mode_cmd.cmd_nums;
	struct lcd_cmd cmd;
	dsi->hs = true;

	/*if(level < 0 || level > effect.max_level)*/
		/*return -EINVAL;*/

	for(i = 0; i < cmd_nums; i++){
		cmd = mode_cmd.lcd_cmds[i];
		dsi_vc_dcs_write(dsi, 0, cmd.cmds, cmd.len);
	}

	//store the effect level
	mode_data->lcd_modes[mode_index].mode_status = 1;

	return ret;

}
int auo_nt35523_get_supported_mode(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_mode_data *mode_data= auo_nt35523_data.lcd_modes;
	struct lcd_mode *mode ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_mode *hal_lcd_mode;

	if(index >= mode_data->supported_modes)
		return -EINVAL;

	mode= &mode_data->lcd_modes[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_mode = &hal_panel_data->mode[index];
	/*hal_lcd_mode->mode_status = mode->mode_status;*/

	return 0;
}
static int auo_nt35523_get_supported_effect(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= auo_nt35523_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}
static int auo_nt35523_get_effect_levels(struct hal_panel_ctrl_data *hal_panel_ctrl)
{
	int index = hal_panel_ctrl->index;
	struct lcd_effect_data *effect_data= auo_nt35523_data.lcd_effects;
	struct lcd_effect *effect ;
	struct hal_panel_data *hal_panel_data;
	struct hal_lcd_effect *hal_lcd_effect;

	if(index >= effect_data->supported_effects)
		return -EINVAL;

	effect= &effect_data->lcd_effects[index];
	hal_panel_data = &hal_panel_ctrl->panel_data;
	hal_lcd_effect = &hal_panel_data->effect[index];
	hal_lcd_effect->level = effect->current_level;

	return 0;
}
static struct lcd_panel_dev auo_nt35523_panel_device = {
	.name = "NT35523_AUO_2560x1600_8",
	.status = OFF,
	.set_effect = auo_nt35523_set_effect,
	.get_current_level = auo_nt35523_get_current_level,
	.get_effect_index_by_name = auo_nt35523_get_effect_index,
	.set_mode = auo_nt35523_set_mode,
	.get_supported_mode = auo_nt35523_get_supported_mode,
	.get_supported_effect = auo_nt35523_get_supported_effect,
	.get_effect_levels = auo_nt35523_get_effect_levels,
	.set_gamma = auo_nt33523_set_gamma,
};
#endif

void  auo_nt35523_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	DRM_DEBUG_DRIVER("\n");
	if (pipe == 0) {
		connector->display_info.width_mm = 216;
		connector->display_info.height_mm = 135;
	}

	return;
}

bool nt35523_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */
	DRM_DEBUG_DRIVER("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 1;
	intel_dsi->clock_stop = 0;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->dual_link = MIPI_DUAL_LINK_FRONT_BACK;
	intel_dsi->pixel_overlap = 0;
	intel_dsi->pipe= PIPE_A; //use pipeA by default

#if 0
	intel_dsi->video_mode_format = VIDEO_MODE_NON_BURST_WITH_SYNC_EVENTS;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
#endif
	intel_dsi->operation_mode = INTEL_DSI_VIDEO_MODE;
	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->init_count = 0x7d0;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->hs_to_lp_count = 0x32;
	intel_dsi->lp_byte_clk = 6;
	intel_dsi->bw_timer = 0x40c;
	intel_dsi->clk_lp_to_hs_count = 0x40;
	intel_dsi->clk_hs_to_lp_count = 0x18;
	//intel_dsi->dphy_reg = 0x3f1f7b18;

        intel_dsi->dphy_reg = 0x3f1f7b2c;


	intel_dsi->port = 0; /* PORT_A by default */
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->escape_clk_div = 0;
	//intel_dsi->video_frmt_cfg_bits =0;// DISABLE_VIDEO_BTA;
        intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;

	intel_dsi->backlight_off_delay = 20;
	intel_dsi->backlight_on_delay = 20;
	intel_dsi->panel_on_delay = 50;
	intel_dsi->panel_off_delay = 50;
	intel_dsi->panel_pwr_cycle_delay = 20;

#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	auo_nt35523_panel_device.dsi = intel_dsi;
	lenovo_lcd_panel_register(&auo_nt35523_panel_device);
	mutex_init(&lcd_mutex);
#endif
	return true;
}



struct drm_display_mode *auo_nt35523_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	mutex_lock(&lcd_mutex);
	mode = dev_priv->vbt.lfp_lvds_vbt_mode;

	DRM_DEBUG_KMS("get mode clock %d\n",mode->clock);

	mode->vrefresh = 60;
	mode->hdisplay = 1600;
	mode->vdisplay = 2560;

	/* Calculate */
	//mode->hsync_start = mode->hdisplay + 240;
	//mode->hsync_end = mode->hsync_start + 16;
	//mode->htotal = mode->hsync_end + 160;

	mode->hsync_start = mode->hdisplay + 208;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end + 144;
	mode->vsync_start = mode->vdisplay + 16;
	mode->vsync_end = mode->vsync_start + 8;
	mode->vtotal = mode->vsync_end+24;
	mode->clock = mode->htotal*mode->vtotal*mode->vrefresh/1000;
	printk(" fuzr mode=%x--%s\n",mode->clock,__func__);
	//mode->clock = 310000; //intel 4078 cr need set the clok to 310000 //fuzr1 2015-08-06
	 mutex_unlock(&lcd_mutex);
	return mode;
}

static void auo_nt35523_panel_reset(struct intel_dsi_device *dsi)
{
	// now use generic panel reset
	return;
}

static void auo_nt35523_disable_panel_power(struct intel_dsi_device *dsi)
{
	return;
}

static void auo_nt35523_send_otp_cmds(struct intel_dsi_device *dsi)
{
	/*for sending panel specified initial sequence*/
    int lcd_id;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	//AUO TE pin config
	u8 page0[]={0xf0,0x55,0xaa,0x52,0x08,0x00};
	u8 te_enable[] ={0xc0,0x0D};
	//AUO END

   //u8 effect_register[] = {0x55,0x81};  // 8=CE on 1=CABC UI mode on
	u8 dimming_ctrl[]={0x53,0x2c};//24 ->2c , cabc dimming on
	u8 dimming_step[] = { 0xd6,0x11,0x11}; //0x11=4frame
#if 1
	//Novatek ESD param
	u8 esd_data1[]={0xff,0xaa,0x55,0xa5,0x80};
	u8 esd_data2[]={0x6f,0x0f};
	u8 esd_data3[]={0xf7,0x1};
	u8 esd_data4[]={0xff,0xaa,0x55,0xa5,0x0};
	u8 esd_data5[]={0x62,0x1};
	//Novatek END
#endif
	//CE param
	u8 cmd1[]={ 0xF0,0x55,0xAA,0x52,0x08,0x00};
	u8 cmd2[]={ 0xCC,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 cmd3[]={ 0xCE,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
	u8 cmd4[]={ 0xD1,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E};
	u8 cmd5[]={ 0xD2,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E};
	u8 cmd6[]={ 0xD7,0x80,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01};
	u8 cmd7[]={ 0xD8,0x01,0x01,0x02,0x02,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00};

	//u8 cmd8[]={0xbb,0x93,0x93}; //pwm frequency adjust

	//Source EQ off
	u8 cmd9[]={0xb8,0x3,0x6,0x0,0x0};//fuzr1 require by yuyao 2015-07-23

	//INX change VGH/VGL and Gate EQ
	u8 cc1[]={0xf0,0x55,0xaa,0x52,0x08,0x01};  //切到page1

	u8 cc2[]={0xb3,0x2f,0x2f};

	u8 cc3[]={0xb4,0x0f,0x0f};

	u8 cc4[]={0xb9,0x35,0x35};

	u8 cc5[]={0xf0,0x55,0xaa,0x52,0x08,0x03};  //切到page3

	u8 cc6[]={0xb0,0x00,0x00,0x00,0x00};
	//INX END
	u8 cmda[]={0XE7,0XF3,0XE6,0XDf,0XD9,0Xd4,0XCe,0Xc8,0Xc2,0Xc2,0XBd};//yuyao modify by CABC PWM  adjust test
	u8 cmdb[]={0Xd5,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};//yuyao modify by CABC PWM  adjust test
	lcd_id= chv_get_lcd_id();

	mutex_lock(&lcd_mutex);
	intel_dsi->hs=1;
	//dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,effect_register,sizeof(effect_register));

	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,save_cmd,2);//when backlight on to use the saved effect parameter
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,dimming_ctrl,sizeof(dimming_ctrl));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,page0,sizeof(page0));
#if 1
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,esd_data1, sizeof(esd_data1));
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,esd_data2, sizeof(esd_data2));
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,esd_data3, sizeof(esd_data3));
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,esd_data4, sizeof(esd_data4));
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,esd_data5, sizeof(esd_data5));
#endif

	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd1,sizeof(cmd1));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd2,sizeof(cmd2));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd3,sizeof(cmd3));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd4,sizeof(cmd4));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd5,sizeof(cmd5));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd6,sizeof(cmd6));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd7,sizeof(cmd7));
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd9,sizeof(cmd9));

	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmda,sizeof(cmda));// yuyao modify by CABC PWM adjust test
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmdb,sizeof(cmdb));// yuyao modify by CABC PWM adjust test
	dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,dimming_step,sizeof(dimming_step));
	if(lcd_id==0)
	{
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,page0,sizeof(page0));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,te_enable,sizeof(te_enable));
		// dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd8,sizeof(cmd8));
	    //dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cmd9,sizeof(cmd9));
	}
	else
	{
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc1,sizeof(cc1));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc2,sizeof(cc2));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc3,sizeof(cc3));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc4,sizeof(cc4));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc5,sizeof(cc5));
		dsi_vc_send_long(intel_dsi,0,MIPI_DSI_GENERIC_LONG_WRITE,cc6,sizeof(cc6));
	}

	mutex_unlock(&lcd_mutex);
	return;
}
extern int isl9860_ext_write_byte(u8 reg, u8 data);
extern int isl9860_ext_read_byte(u8 reg);
void nt35523_enable(struct intel_dsi_device *dsi)
{
	int d1,d2,d3;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	auo_nt35523_send_otp_cmds(dsi);
	mutex_lock(&lcd_mutex);
	DRM_DEBUG_DRIVER("\n");
	intel_dsi->hs=0;
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);
	mdelay(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);
	d1=isl9860_ext_read_byte(0x6);
	d2=isl9860_ext_read_byte(0x8);
	d3=isl9860_ext_read_byte(0x9);
	DRM_DEBUG_DRIVER(" fuzr0 0x6=%d,0x8=%d,0x9=%d\n",d1,d2,d3);
    isl9860_ext_write_byte(0x6,0x9);

	mdelay(2);
    isl9860_ext_write_byte(0x8,0x2);
	mdelay(2);
	isl9860_ext_write_byte(0x9,0x2);

	d1=isl9860_ext_read_byte(0x6);
	d2=isl9860_ext_read_byte(0x8);
	d3=isl9860_ext_read_byte(0x9);
	DRM_DEBUG_DRIVER(" fuzr1 0x6=%d,0x8=%d,0x9=%d\n",d1,d2,d3);

	mutex_unlock(&lcd_mutex);
}

void nt35523_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	auo_nt35523_panel_device.status = OFF;
#endif
	mutex_lock(&lcd_mutex);
	intel_dsi->hs=0;
	DRM_DEBUG_DRIVER("\n");
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);
	mdelay(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);

	mutex_unlock(&lcd_mutex);
}

void auo_nt35523_enable_bklt(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	u8 br_ctrl[2]={0x53,0x2c};//24 ->2c , cabc dimming
	DRM_DEBUG_DRIVER("\n");

	intel_dsi->hs=1;
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,br_ctrl, 2);
}

void auo_nt35523_disable_bklt(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	u8 br_ctrl[2]={0x53,0x0};
	DRM_DEBUG_DRIVER("\n");
	intel_dsi->hs=1;
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,br_ctrl, 2);
}

void auo_nt35523_set_brightness(struct intel_dsi_device *dsi,u32 level)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	u8 data[2]={0x51,0};
	mutex_lock(&lcd_mutex);
	intel_dsi->hs=1;
	if(level>0xff)
		level=0xff;

	data[1] = level;
	dsi_vc_send_long(intel_dsi, 0, MIPI_DSI_GENERIC_LONG_WRITE,data, 2);
	DRM_DEBUG_DRIVER("[LCD]: %s line=%d, level=%d\n",__func__, __LINE__,  level);
	mutex_unlock(&lcd_mutex);
}

void bklt_power_uevent(struct drm_device *dev,char*  poweron)
{
	char event_string[128] = "SCREENON=";
	char *envp[] = { event_string, NULL };
	strncat(event_string, poweron,16);
	DRM_DEBUG("mainscreen onoff:%s \n", event_string);
	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, envp);
}

void auo_nt35523_power_on(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
    u8 val = 0;
    mutex_lock(&lcd_mutex);
	DRM_DEBUG_KMS("\n");
	/* enable VSWITCH2 for LCDIO */
	val = intel_soc_pmic_readb(PMIC_VSWITCH_REG);
	DRM_DEBUG_KMS("%s: read PMIC 0x%x = 0x%x.",
			__func__, PMIC_VSWITCH_REG, val);
	val |= PMIC_VSWITCH2_EN;
	intel_soc_pmic_writeb(PMIC_VSWITCH_REG, val);
	msleep(15);

	/* enable LCD BIAS for BladeIII 10A */
#if 0
	DRM_INFO("%s: enable LCD power.\n", __func__);
	/* LCD_BIAS_ENP -- GPIO BK20 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5030, 0x8102);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5034, 0x4c00000);
	msleep(15);

	/* LCD_BIAS_ENN -- GPIO BM20 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5020, 0x8102);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5024, 0x4c00000);
	msleep(20);
	/* hardcode to do LCD reset for BladeIII 10A */
	DRM_INFO("%s: do LCD reset.\n", __func__);
	/* LCD_RESET_FVP -- K22 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x5428, 0x8102);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x542c, 0x4c00000);
	msleep(10);

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x5428, 0x8100);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x542c, 0x4c00000);
	msleep(20);

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x5428, 0x8102);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x542c, 0x4c00000);
	msleep(50);

#else

    msleep(20);
    /* hardcode to do LCD reset for BladeIII 10A */
    DRM_DEBUG("%s: do LCD reset.\n", __func__);
    /* LCD_RESET_FVP -- K22 */
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x5428, 0x8102);
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x542c, 0x4c00000);
    msleep(10);

    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x5428, 0x8100);
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x542c, 0x4c00000);
    msleep(20);

    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x5428, 0x8102);
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
                    0x542c, 0x4c00000);
    msleep(50);
    DRM_INFO("%s: enable LCD power.\n", __func__);
    /* LCD_BIAS_ENP -- GPIO BK20 */
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
                    0x5030, 0x8102);
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
                    0x5034, 0x4c00000);
    msleep(15);

    /* LCD_BIAS_ENN -- GPIO BM20 */
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
                    0x5020, 0x8102);
    vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
                    0x5024, 0x4c00000);
    msleep(20);

#endif
#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	auo_nt35523_panel_device.status = ON;
    bk_status=true;
    DRM_DEBUG("%s--ON\n",__func__);
#endif
    bklt_power_uevent(dev,"1");
	mutex_unlock(&lcd_mutex);
	return;
}

void auo_nt35523_power_off(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u8 val = 0;
#ifdef  CONFIG_LENOVO_DISPLAY_FEATURE
	auo_nt35523_panel_device.status = OFF;
    bk_status=false;
    DRM_DEBUG("%s--OFF\n",__func__);
#endif
	mutex_lock(&lcd_mutex);
	DRM_DEBUG_KMS("\n");

	/* pull down panel reset pin */
	msleep(100);
	/* LCD_RESET_FVP -- K22 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x5428, 0x8100);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
			0x542c, 0x4c00000);
	msleep(10);

	/* disable LCD BIAS for BladeIII 10A */
	DRM_INFO("%s: disable LCD power.\n", __func__);
	/* LCD_BIAS_ENN -- GPIO BM20 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5020, 0x8100);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5024, 0x4c00000);
	msleep(10);

	/* LCD_BIAS_ENP -- GPIO BK20 */
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5030, 0x8100);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_SC,
			0x5034, 0x4c00000);
	msleep(10);

	/* disable VSWITCH2 for LCDIO */
	val = intel_soc_pmic_readb(PMIC_VSWITCH_REG);
	DRM_DEBUG_KMS("%s: read PMIC 0x%x = 0x%x.",
			__func__, PMIC_VSWITCH_REG, val);
	val &= ~PMIC_VSWITCH2_EN;
	intel_soc_pmic_writeb(PMIC_VSWITCH_REG, val);
        bklt_power_uevent(dev,"0");
	mutex_unlock(&lcd_mutex);
	return;
}


/* Callbacks. We might not need them all. */

struct intel_dsi_dev_ops auo_nt35523_dsi_display_ops = {
	.init = nt35523_init,
	.get_info = auo_nt35523_vid_get_panel_info,
	.get_modes = auo_nt35523_get_modes,
	.panel_reset = auo_nt35523_panel_reset,
	.disable_panel_power = auo_nt35523_disable_panel_power,
	.send_otp_cmds = auo_nt35523_send_otp_cmds,
	.enable = nt35523_enable,
	.disable = nt35523_disable,
	.power_on = auo_nt35523_power_on,
	.power_off = auo_nt35523_power_off,
	.enable_backlight = auo_nt35523_enable_bklt,
	.disable_backlight = auo_nt35523_disable_bklt,
	.set_brightness = auo_nt35523_set_brightness,
};

