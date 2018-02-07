/*
 * Copyright (C) 2013 Intel Corporation
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
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 */
#ifndef __DSI_MOD_NOV_NT35523_H__
#define __DSI_MOD_NOV_NT35523_H__

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"
#include "lenovo_lcd_panel.h"

#define DELAY_TYPE		0xff

static struct lcd_cmd cabcoff_cmds[] = {
	{{0x53, 0x24}, 2},
	{{0x55, 0x00}, 2},
};

static struct lcd_cmd cabcon_ui_cmds[] = {
	{{0x53, 0x24}, 2},
	{{0x55, 0x01}, 2},

};

static struct lcd_cmd cabcon_still_cmds[] = {
	{{0x53, 0x24}, 2},
	{{0x55, 0x02}, 2},

};

static struct lcd_cmd cabcon_movingi_cmds[] = {
	{{0x53, 0x24}, 2},
	{{0x55, 0x03}, 2},

};


static struct lcd_cmd suspend_cmds[] = {
	{{0x28}, 1},
	{{120}, DELAY_TYPE},
	{{0x10}, 1},
};

static struct lcd_cmd resume_cmds[] = {
	{{0x11}, 1},
	{{120}, DELAY_TYPE},
	{{0x29}, 1},
};


static struct lcd_cmd ceoff_cmds[]  = {
};

static struct lcd_cmd ceon_cmds[] = {
};

static struct lcd_effect_cmd cabc_effect_cmds[] = {
	{ ARRAY_SIZE(cabcoff_cmds), cabcoff_cmds},
	{ ARRAY_SIZE(cabcon_ui_cmds), cabcon_ui_cmds},
	{ ARRAY_SIZE(cabcon_still_cmds), cabcon_still_cmds},
	{ ARRAY_SIZE(cabcon_movingi_cmds), cabcon_movingi_cmds},
};

static struct lcd_effect_cmd suspend_effect_cmds[] = {
	{ ARRAY_SIZE(suspend_cmds), suspend_cmds},
	{ ARRAY_SIZE(resume_cmds), resume_cmds},
};

static struct lcd_effect_cmd ce_effect_cmds[] = {
	{ARRAY_SIZE(ceoff_cmds), ceoff_cmds},
	{ARRAY_SIZE(ceon_cmds), ceon_cmds},
};
#if 1
static struct lcd_mode_cmd standard_mode_cmds[] = {
    {ARRAY_SIZE(cabcon_ui_cmds), cabcon_ui_cmds},
    {ARRAY_SIZE(ceon_cmds), ceon_cmds},
};
static struct lcd_mode_cmd camera_mode_cmds[] = {
    {ARRAY_SIZE(cabcoff_cmds), cabcoff_cmds},
    {ARRAY_SIZE(ceoff_cmds), ceoff_cmds},
};
static struct lcd_mode_cmd blank_mode_cmds[] = {
    //{ARRAY_SIZE(ceoff_cmds), ceoff_cmds},
    //{ARRAY_SIZE(ceon_cmds), ceon_cmds},
};
static struct lcd_mode auo_nt35523_mode[] = {
    {"standard",  standard_mode_cmds,1},
    {"camera",  camera_mode_cmds,1},
    {"blank",  blank_mode_cmds,0},
};
static struct lcd_mode_data auo_nt35523_mode_data = { auo_nt35523_mode, ARRAY_SIZE(auo_nt35523_mode)};
#endif

static struct lcd_effect auo_nt35523_effect[] = {
	{"cabc", ARRAY_SIZE(cabc_effect_cmds), 0, cabc_effect_cmds},
	{"ce", ARRAY_SIZE(ce_effect_cmds), 1, ce_effect_cmds},
	{"suspend", ARRAY_SIZE(suspend_effect_cmds), 1, suspend_effect_cmds},
};

static struct lcd_effect_data auo_nt35523_effect_data = { auo_nt35523_effect, ARRAY_SIZE(auo_nt35523_effect)};

static struct lcd_data auo_nt35523_data = {
	&auo_nt35523_effect_data,
	&auo_nt35523_mode_data,
	//NULL,
};


#endif /* __DSI_MOD_NOV_NT35523_H__ */
