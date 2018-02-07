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
#include "dsi_mod_ti_dpp3430.h"

extern int dlp3430_init(void);
extern int dlp3430_power_on(struct drm_device *dev);
extern int dlp3430_power_off(struct drm_device *dev);


void  dpp3430_vid_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector)
		return;

	DRM_DEBUG_KMS("\n");
	if (pipe == 0) {
		connector->display_info.width_mm = 216;
		connector->display_info.height_mm = 135;
	}

	return;
}

bool dpp3430_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	//struct mipi_phy_config config;
	//bool ret = false;

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
	DRM_DEBUG_KMS("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 2;

	intel_dsi->eotp_pkt = 1;
	intel_dsi->clock_stop = 0;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->pixel_overlap = 0;
	intel_dsi->dual_link = MIPI_DUAL_LINK_NONE;
	intel_dsi->port = 0;
	intel_dsi->pipe= PIPE_A;
	intel_dsi->dsi_bpp = 24;
	intel_dsi->operation_mode = INTEL_DSI_VIDEO_MODE;
	intel_dsi->video_mode_format = VIDEO_MODE_NON_BURST_WITH_SYNC_EVENTS;
	intel_dsi->escape_clk_div = 0;
	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->init_count = 0x7d0;
	intel_dsi->bw_timer = 0x820;
	intel_dsi->video_frmt_cfg_bits = 0x8;

	intel_dsi->port_bits = 0;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 1;
	intel_dsi->clk_lp_to_hs_count = 0xa;
	intel_dsi->clk_hs_to_lp_count = 0x14;

	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 1;
	intel_dsi->clk_lp_to_hs_count = 0xa;
	intel_dsi->clk_hs_to_lp_count = 0x14;

	intel_dsi->dphy_reg = 0x3f0c3511;

#if 0
 	config.tclk_prepare = 69;
	//config.tclk_trail = 80;
	config.tclk_trail = 69;
	config.tclk_prepare_clkzero = 400;
	//config.ths_prepare = 69;
	config.ths_prepare = 96; 
	config.ths_trail = 70;
	//config.ths_prepare_hszero = (255 + 69);
	config.ths_prepare_hszero = (365 + 96);//raw form ti 372+93; overflow ,after ti confirm 365 + 99; wait ti confirm;
	ret = intel_dsi_generate_phy_reg(intel_dsi, &config);
	printk("mipi_timming intel_dsi->dphy_reg:%x;ret :%d\n ",intel_dsi->dphy_reg,ret  );
	if (ret == false) {
		DRM_ERROR("intel_dsi_generate_phy_reg failed.\n");
		intel_dsi->hs_to_lp_count = 0x46;
		intel_dsi->lp_byte_clk = 1;
		intel_dsi->clk_lp_to_hs_count = 0xa;
		intel_dsi->clk_hs_to_lp_count = 0x14;
		//intel_dsi->dphy_reg = 0x2c0c340e;
		intel_dsi->dphy_reg = 0x3f0c3511;
	}
#endif


	intel_dsi->backlight_off_delay = 20;
	intel_dsi->backlight_on_delay = 10;
	intel_dsi->panel_on_delay = 10;
	intel_dsi->panel_off_delay = 10;
	intel_dsi->panel_pwr_cycle_delay = 10;

	intel_dsi->pclk = (854 + 64 + 10 + 34) * (480 + 8 + 2 + 6) * 60 / 1000;

	return true;
}

//void dpp3430_create_resources(struct intel_dsi_device *dsi) { }
void dpp3430_enable(struct intel_dsi_device *dsi)
{


	DRM_INFO("====>dpp3430_enable\n");	
	dlp3430_init();
}

void dpp3430_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	DRM_INFO("====>dpp3430_disable\n");

	dlp3430_power_off(dev);
}

void dpp3430_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	DRM_INFO("====>dpp3430_reset\n");

	dlp3430_power_on(dev);
}



struct drm_display_mode *dpp3430_get_modes(struct intel_dsi_device *dsi)
{

	struct drm_display_mode *mode = NULL;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	mode = dev_priv->vbt.lfp_lvds_vbt_mode;
	DRM_DEBUG_KMS("\n");

	mode->hdisplay = 854;
	mode->hsync_start = mode->hdisplay + 64; //fp
	mode->hsync_end = mode->hsync_start + 10;	//26; //sync
	mode->htotal = mode->hsync_end + 34;  //bp

	mode->vdisplay = 480;
	mode->vsync_start = mode->vdisplay + 8;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 6;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	return mode;
}


/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops ti_dpp3430_dsi_display_ops = {
	.init = dpp3430_init,
	.get_info = dpp3430_vid_get_panel_info,
	.get_modes = dpp3430_get_modes,
	.enable = dpp3430_enable,
	.disable = dpp3430_disable,
	.panel_reset = dpp3430_reset,
};
