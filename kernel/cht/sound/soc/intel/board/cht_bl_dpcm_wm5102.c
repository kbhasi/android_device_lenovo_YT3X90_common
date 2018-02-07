/*
 *  cht_bl_wm5102.c - ASoc Machine driver for Intel Baytrail Baylake MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define DEBUG
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/pm_runtime.h>
#include <asm/platform_cht_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/mfd/arizona/registers.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <sound/soc-dai.h>
#include "../../codecs/wm5102.h"
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>
//#define BLADE3_13_SUB_WOOFER

static struct wake_lock wakelock_offload;

#define BYT_PLAT_CLK_3_HZ	19200000
#define WM5102_MAX_SYSCLK_1 49152000 /*max sysclk for 4K family*/
#define WM5102_MAX_SYSCLK_2 45158400 /*max sysclk for 11.025K family*/

static struct snd_soc_codec *wm5102;

static int previous_bias_level =SND_SOC_BIAS_OFF;

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2

/*Add by karl for enable the wm5102 gpio5*/
#define EXT_SPEAKER_ENABLE_PIN 10    // GPIO1
//Time sequence is not acceptable by using CODEC_GPIO1, so change SPK_EN to Soc gpio, codec gpio1 control remove later
#define EXT_SPEAKER_ENABLE_SOC_GPIO 489
// before DVT2-2 yt3_hw_ver =0,  otherwise yt3_hw_ver=1
extern unsigned int yt3_hw_ver; 

#ifdef BLADE3_13_SUB_WOOFER  //13A subspeaker control PIN GPIO2
static int sub_woofer_pa_enable_pin = 11;
#endif

struct cht_mc_private {
#ifdef CONFIG_SND_SOC_COMMS_SSP
	struct cht_comms_mc_private comms_ctl;
#endif /* CONFIG_SND_SOC_COMMS_SSP */
};

#ifdef CONFIG_SND_SOC_COMMS_SSP
static inline struct cht_comms_mc_private *kcontrol2ctl(struct snd_kcontrol *kcontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct cht_comms_mc_private *ctl = &(ctx->comms_ctl);
	return ctl;
}

int cht_get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct cht_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;

	return 0;
}

int cht_set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct cht_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_bt_sco_master_mode)
		ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];
	return 0;
}

int cht_get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct cht_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}

int cht_set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct cht_comms_mc_private *ctl = kcontrol2ctl(kcontrol);
	if (ucontrol->value.integer.value[0] != ctl->ssp_modem_master_mode)
		ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];
	return 0;
}
#endif /* CONFIG_SND_SOC_COMMS_SSP */


#ifdef BLADE3_13_SUB_WOOFER
static int Sub_woofer_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpio_set_value(sub_woofer_pa_enable_pin,1);
		udelay(3);
		gpio_set_value(sub_woofer_pa_enable_pin,0);
		udelay(3);
		gpio_set_value(sub_woofer_pa_enable_pin,1);
		udelay(3);
		gpio_set_value(sub_woofer_pa_enable_pin,0);
		udelay(3);
		gpio_set_value(sub_woofer_pa_enable_pin,1);
		pr_info("Platform sub woofer turned ON\n");
	}

	else {
		gpio_set_value(sub_woofer_pa_enable_pin,0);
		pr_info("Platform sub woofer turned OFF\n");
	}

	return 0;
}
#endif

/*add by karlsun for enable SPK_EN*/
static int Blade3_Spk_en_speaker_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		printk("arizona machine on %s()\n", __func__);
		if(yt3_hw_ver == 0) //before dvt2-2
		    gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN , 1);
                else{
		    gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 1);
		    udelay(2);
		    gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 0);
		    udelay(2);
		    gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 1);
                }
        } else {		
		printk("arizona machine off %s()\n", __func__);
		if(yt3_hw_ver == 0)
		    gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN , 0);
		else
		    gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 0);
    }
	return 0;
}

/*End by karl*/



static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
/*add by karlsun for enable SPK_EN*/	
	SND_SOC_DAPM_SPK("Ext Spk", Blade3_Spk_en_speaker_event),
#ifdef BLADE3_13_SUB_WOOFER
	SND_SOC_DAPM_SPK("Sub Spk", Sub_woofer_control),		
#endif
/*end by karl*/	
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			NULL, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_audio_map[] = {

/*add by karl for start*/	
	{"Headphone", NULL, "HPOUT1L"},
	{"Headphone", NULL, "HPOUT1R"},
	
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},
	
	{"Ext Spk", NULL, "HPOUT2L"},
	{"Ext Spk", NULL, "HPOUT2R"},

	{"Headset Mic", NULL, "MICBIAS1"},
	{"Headset Mic", NULL, "MICBIAS2"},
	{"IN2L", NULL, "Headset Mic"},
	{"Int Mic", NULL, "MICBIAS3"},
	{"IN1L", NULL, "Int Mic"},
#ifdef BLADE3_13_SUB_WOOFER
	{"Sub Spk", NULL, "EPOUT"},
#endif	

/*end*/

	{"AIF1 Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx" },
	{"codec_in1", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "AIF1 Capture"},

	{"ssp0 Tx", NULL, "modem_out"},
	{"modem_in", NULL, "ssp0 Rx" },

	{"ssp1 Tx", NULL, "bt_fm_out"},
	{"bt_fm_in", NULL, "ssp1 Rx" },

	{"AIF1 Playback", NULL, "Platform Clock"},
	{"AIF1 Capture", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
#ifdef BLADE3_13_SUB_WOOFER	
	SOC_DAPM_PIN_SWITCH("Sub Spk"),
#endif	
};


 

static int cht_config_5102_clks(struct snd_soc_codec *wm5102_codec, int sr)
{
	int ret;
	int sr_mult = (sr % 4000 == 0) ? (WM5102_MAX_SYSCLK_1/sr) : (WM5102_MAX_SYSCLK_2/sr);

	pr_info("%s:\n", __func__);

	/*Open MCLK before Set Codec CLK*/
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_ON);
	/*reset FLL1*/
	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE, 0, 0);
	snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
				ARIZONA_FLL_SRC_NONE, 0, 0);

	ret = snd_soc_codec_set_pll(wm5102_codec, WM5102_FLL1,
					ARIZONA_CLK_SRC_MCLK1, //TODO Check if clock from AP is connected to MCLK1
					BYT_PLAT_CLK_3_HZ,
					sr * sr_mult);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);		
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
			ARIZONA_CLK_SYSCLK,
			ARIZONA_CLK_SRC_FLL1,
			sr * sr_mult,
			SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set AYNCCLK: %d\n", ret);
		
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(wm5102_codec,
					ARIZONA_CLK_OPCLK, 0,
					sr	* sr_mult,
					SND_SOC_CLOCK_OUT);
	if (ret != 0) {
		dev_err(wm5102_codec->dev, "Failed to set OPCLK: %d\n", ret);		
		return ret;
	}

	return 0;
}

static int cht_free_5102_clks(struct snd_soc_codec *codec)
{
	pr_info("%s: \n", __func__);

	snd_soc_codec_set_pll(codec, WM5102_FLL1_REFCLK,
		ARIZONA_FLL_SRC_NONE, 0, 0);

	snd_soc_codec_set_pll(codec, WM5102_FLL1,
		ARIZONA_FLL_SRC_NONE, 0, 0);

	/*Open MCLK before Set Codec CLK*/
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);
	return 0;
}

static int cht_compr_set_params(struct snd_compr_stream *cstream)
{
	return 0;
}

static int cht_compr_startup(struct snd_compr_stream *cstream)
{
	printk("%s, offload_pm_get\n", __func__);
	wake_lock(&wakelock_offload);
        return 0;
}

static void cht_compr_shutdown(struct snd_compr_stream *cstream)
{
	printk("%s, offload_pm_put\n", __func__);
	wake_unlock(&wakelock_offload);
}


static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);

	/* proceed only if dai is valid */
	if (strncmp(codec_dai->name, "wm5102-aif1", 11))
	{
		pr_debug("invalide codec dai name");
		return 0;
	}

	/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
	 * 4 active slots as 0xF
	 */
#if 1
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4,
			SNDRV_PCM_FORMAT_GSM);
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}
#endif

	/* TDM slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}

	if (ret < 0) {
		pr_err("can't set codec sysclk: %d\n", ret);
		return ret;
	}
	return 0;
}

static int cht_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	int ret;

	if (!wm5102)
		return 0;

	if (dapm->dev != wm5102->dev)
		return 0;

	pr_debug ("cht_set_bias_level level is %d\n", level);

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (previous_bias_level != SND_SOC_BIAS_STANDBY)
			break;      
		ret = cht_config_5102_clks(wm5102,48000);
		if (ret < 0) {
			pr_err("failed to wm5102_snd_startup_clk\n");
		}
		break;

	default:
		break;
	}

	printk("%s, done\n", __func__);
	return 0;
}

static int cht_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	int ret;

	if (!wm5102)
		return 0;

	if (dapm->dev != wm5102->dev)
		return 0;

	pr_debug ("cht_set_bias_level_post level is %d previous_bias_level is %d\n",level,previous_bias_level);

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
        if(previous_bias_level < SND_SOC_BIAS_PREPARE)
            break;
			 ret= cht_free_5102_clks(wm5102);
			 if (ret < 0) {
				pr_err("failed to wm5102_snd_shutdown_clk\n"); 			 
			 }

		break;

	default:
		break;
	}
	previous_bias_level = level;
	printk("%s, done\n", __func__);
	return 0;
}


#ifdef CONFIG_SND_SOC_COMMS_SSP
static int cht_comms_dai_link_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *str_runtime;

	str_runtime = substream->runtime;

	WARN(!substream->pcm, "BYT Comms Machine: ERROR NULL substream->pcm\n");

	if (!substream->pcm)
		return -EINVAL;

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	switch (substream->pcm->device) {
	case BYT_COMMS_BT:
		str_runtime->hw = BYT_COMMS_BT_hw_param;
		break;

	case BYT_COMMS_MODEM:
		str_runtime->hw = BYT_COMMS_MODEM_hw_param;
		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device = %d\n",
		       substream->pcm->device);
	}
	return snd_pcm_hw_constraint_integer(str_runtime,
					 SNDRV_PCM_HW_PARAM_PERIODS);
}

static int cht_comms_dai_link_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);
	struct cht_comms_mc_private *ctl = &(ctx->comms_ctl);

	int ret = 0;
	unsigned int tx_mask, rx_mask;
	unsigned int nb_slot = 0;
	unsigned int slot_width = 0;
	unsigned int tristate_offset = 0;
	unsigned int device = substream->pcm->device;

	pr_debug("ssp_bt_sco_master_mode %d\n", ctl->ssp_bt_sco_master_mode);
	pr_debug("ssp_modem_master_mode %d\n", ctl->ssp_modem_master_mode);

	switch (device) {
	case BYT_COMMS_BT:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_1
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SSP_DAI_SCMODE_1 |
					  SND_SOC_DAIFMT_NB_NF |
					  (ctl->ssp_bt_sco_master_mode ?
					   SND_SOC_DAIFMT_CBM_CFM :
					   SND_SOC_DAIFMT_CBS_CFS));

		if (ret < 0) {
			pr_err("BYT Comms Machine: Set FMT Fails %d\n",
				ret);
			return -EINVAL;
		}

		/*
		 * BT SCO SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 16
		 * tristate = 1
		 * ssp_frmsync_timing_bit = 0
		 * (NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 * ssp_frmsync_timing_bit = 1
		 * (NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * ssp_psp_T2 = 1
		 * (Dummy start offset = 1 bit clock period)
		 */
		nb_slot = BYT_SSP_BT_SLOT_NB_SLOT;
		slot_width = BYT_SSP_BT_SLOT_WIDTH;
		tx_mask = BYT_SSP_BT_SLOT_TX_MASK;
		rx_mask = BYT_SSP_BT_SLOT_RX_MASK;

		if (ctl->ssp_bt_sco_master_mode)
			tristate_offset = BIT(TRISTATE_BIT);
		else
			tristate_offset = BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);
		break;

	case BYT_COMMS_MODEM:
		/*
		 * set cpu DAI configuration
		 * frame_format = PSP_FORMAT
		 * ssp_serial_clk_mode = SSP_CLK_MODE_0
		 * ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH
		 */
		ret = snd_soc_dai_set_fmt(cpu_dai,
						SND_SOC_DAIFMT_I2S |
						SSP_DAI_SCMODE_0 |
						SND_SOC_DAIFMT_NB_NF |
						(ctl->ssp_modem_master_mode ?
						SND_SOC_DAIFMT_CBM_CFM :
						SND_SOC_DAIFMT_CBS_CFS));
		if (ret < 0) {
			pr_err("BYT Comms Machine:  Set FMT Fails %d\n", ret);
			return -EINVAL;
		}

		/*
		 * Modem Mixing SSP Config
		 * ssp_active_tx_slots_map = 0x01
		 * ssp_active_rx_slots_map = 0x01
		 * frame_rate_divider_control = 1
		 * data_size = 32
		 * Master:
		 *	tristate = 3
		 *	ssp_frmsync_timing_bit = 1, for MASTER
		 *	(NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM)
		 * Slave:
		 *	tristate = 1
		 *	ssp_frmsync_timing_bit = 0, for SLAVE
		 *	(NEXT_FRMS_ASS_AFTER_END_OF_T4)
		 *
		 */
		nb_slot = BYT_SSP_MIXING_SLOT_NB_SLOT;
		slot_width = BYT_SSP_MIXING_SLOT_WIDTH;
		tx_mask = BYT_SSP_MIXING_SLOT_TX_MASK;
		rx_mask = BYT_SSP_MIXING_SLOT_RX_MASK;

		tristate_offset = BIT(TRISTATE_BIT) |
		    BIT(FRAME_SYNC_RELATIVE_TIMING_BIT);

		break;
	default:
		pr_err("BYT Comms Machine: bad PCM Device ID = %d\n", device);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask,
				   rx_mask, nb_slot, slot_width);

	if (ret < 0) {
		pr_err("BYT Comms Machine:  Set TDM Slot Fails %d\n", ret);
		return -EINVAL;
	}

	ret = snd_soc_dai_set_tristate(cpu_dai, tristate_offset);
	if (ret < 0) {
		pr_err("BYT Comms Machine: Set Tristate Fails %d\n", ret);
		return -EINVAL;
	}

	pr_debug("BYT Comms Machine: slot_width = %d\n",
	     slot_width);
	pr_debug("BYT Comms Machine: tx_mask = %d\n",
	     tx_mask);
	pr_debug("BYT Comms Machine: rx_mask = %d\n",
	     rx_mask);
	pr_debug("BYT Comms Machine: tristate_offset = %d\n",
	     tristate_offset);

	return 0;
}

static int cht_comms_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct cht_comms_mc_private *ctl = &(ctx->comms_ctl);

	unsigned int device = substream->pcm->device;

	pr_debug("%s substream->runtime->rate %d\n",
		__func__,
		substream->runtime->rate);

	/* select clock source (if master) */
	/* BT SCO: CPU DAI is master */
	/* FM: CPU DAI is master */
	/* BT_VOIP: CPU DAI is master */
	if ((device == BYT_COMMS_BT && ctl->ssp_bt_sco_master_mode) ||
	    (device == BYT_COMMS_MODEM && ctl->ssp_modem_master_mode)) {
		snd_soc_dai_set_sysclk(cpu_dai, SSP_CLK_ONCHIP,
				substream->runtime->rate, 0);
	}

	return 0;
}
#endif  /* CONFIG_SND_SOC_COMMS_SSP */

static int cht_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret = 0;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
#ifdef CONFIG_SND_SOC_COMMS_SSP
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
#endif

	pr_info("Enter:%s\n", __func__);

	wm5102 = codec;
	

	/* Set codec bias level */
	cht_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;


	ret = snd_soc_add_card_controls(card, cht_mc_controls,
					ARRAY_SIZE(cht_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

#ifdef CONFIG_SND_SOC_COMMS_SSP
	/* Add Comms specific controls */
	if(ctx){
		ctx->comms_ctl.ssp_bt_sco_master_mode = false;
		ctx->comms_ctl.ssp_modem_master_mode = false;

		ret = snd_soc_add_card_controls(card, byt_ssp_comms_controls,
					ARRAY_SIZE(byt_ssp_comms_controls));
	}

	if (ret) {
		pr_err("unable to add COMMS card controls\n");
		return ret;
	}
#endif /* CONFIG_SND_SOC_COMMS_SSP */

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1L");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUT1R");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTLP");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTLN");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRP");
	snd_soc_dapm_ignore_suspend(dapm, "SPKOUTRN");

	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Playback");
	snd_soc_dapm_ignore_suspend(dapm, "AIF2 Capture");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Ext Spk");

	snd_soc_dapm_ignore_suspend(&card->dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(&card->dapm, "Int Mic");
#ifdef BLADE3_13_SUB_WOOFER
	snd_soc_dapm_ignore_suspend(dapm, "Sub Spk");
	snd_soc_dapm_enable_pin(dapm, "Sub Spk");
	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");
#endif
	snd_soc_dapm_sync(dapm);
	snd_soc_update_bits(codec, 0x0c00, 0xffff, 0x2901);
	pr_info("exit %s\n", __func__);

	return ret;
}

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 4;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static const struct snd_soc_pcm_stream cht_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};


static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int cht_aif1_cpu_startup(struct snd_pcm_substream *substream)
{
	printk("%s\n", __func__);
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops cht_aif1_cpu_ops = {
	.startup = cht_aif1_cpu_startup,
};

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};	


static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static int cht_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops cht_8k_16k_ops = {
	.startup = cht_8k_16k_startup,
	.hw_params = cht_aif1_hw_params,
};
static struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_compr_ops cht_compr_ops = {
	.set_params = cht_compr_set_params,
	.startup = cht_compr_startup,
	.shutdown = cht_compr_shutdown,	
};


#ifdef CONFIG_SND_SOC_COMMS_SSP
static struct snd_soc_ops cht_comms_dai_link_ops = {
	.startup = cht_comms_dai_link_startup,
	.hw_params = cht_comms_dai_link_hw_params,
	.prepare = cht_comms_dai_link_prepare,
};
#endif /* CONFIG_SND_SOC_COMMS_SSP */
static struct snd_soc_dai_link cht_dailink[] = {

	[CHT_DPCM_AUDIO] = 
        {
		.name = "Cherrytrail Audio Port",
		.stream_name = "Cherrytrail Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_cpu_ops,
	},
	[CHT_DPCM_DB] = 
        {
		.name = "Cherrytrail DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_cpu_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_COMPR] = 
        {
		.name = "Cherrytrail Compressed Port",
		.stream_name = "Cherrytrail Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.compr_ops = &cht_compr_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_LL] = {
		.name = "Cherrytrail LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "Lowlatency-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_cpu_ops,
	},
	[CHT_DPCM_VOIP] = 
        {
		.name = "Cherrytrail VOIP Port",
		.stream_name = "Cherrytrail Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &cht_8k_16k_ops,
		.dynamic = 1,
	},
	[CHT_DPCM_PROBE] = 
        {
		.name = "Cherrytrail Probe Port",
		.stream_name = "Cherrytrail Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.playback_count = 8,
		.capture_count = 8,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Cherrytrail Codec-Loop Port",
		.stream_name = "Cherrytrail Codec-Loop",
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail Modem-Loop Port",
		.stream_name = "Cherrytrail Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail BTFM-Loop Port",
		.stream_name = "Cherrytrail BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	/* Back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "wm5102-aif1",
		.codec_name = "wm5102-codec",
		.init = cht_init,
		.be_hw_params_fixup = cht_codec_fixup,
		.ignore_suspend = 1,
		.ops = &cht_be_ssp2_ops,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_cht_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_cht_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_cht_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_cht_prepare NULL
#define snd_cht_complete NULL
#define snd_cht_poweroff NULL
#endif
static int snd_cht_mc_late_probe(struct snd_soc_card *card)
{
	wake_lock_init(&wakelock_offload, WAKE_LOCK_SUSPEND, "offload_wake_lock");
	return 0;
}
/* SoC card */
static struct snd_soc_card snd_soc_card_cht = {
	.name = "cherrytrailaud",
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.late_probe = snd_cht_mc_late_probe,
	.set_bias_level = cht_set_bias_level,
	.set_bias_level_post = cht_set_bias_level_post,
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_audio_map),
};

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct cht_mc_private *drv;
	printk ("Entry %s\n", __func__);

	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF); //force off the MCLK1

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

#ifdef BLADE3_13_SUB_WOOFER
    ret_val = gpio_request_one(sub_woofer_pa_enable_pin,GPIOF_DIR_OUT| GPIOF_INIT_LOW, "Sub-woofer");
    if(ret_val!=0)
	{
        pr_err("snd_soc_register_card sub woofer request %d\n", ret_val);
		//return -EPROBE_DEFER;
	}
#endif

/*add by karl for enable spk enable*/
	pr_info("%s yt3_hw_ver = %d \n", __func__, yt3_hw_ver);
	if(yt3_hw_ver == 0){
		ret_val = gpio_request_one(EXT_SPEAKER_ENABLE_PIN,GPIOF_DIR_OUT| GPIOF_INIT_LOW, "SPK_EN");
		if(ret_val!=0){
		    pr_err("snd_soc_register_cardSPK_EN request %d\n", ret_val);
		    //return -EPROBE_DEFER;
		}
		gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN, 0);
	}else{
		ret_val = gpio_request_one(EXT_SPEAKER_ENABLE_SOC_GPIO,GPIOF_DIR_OUT| GPIOF_INIT_LOW, "SPK_EN");
		if(ret_val!=0){
		    pr_err("snd_soc_register_card SOC_SPK_EN request %d\n", ret_val);
		    //return -EPROBE_DEFER;
		}
		gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 0);
	}
/*add end*/



	/* register the soc card */
	snd_soc_card_cht.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_cht, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_cht);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_cht);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static int snd_cht_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	pr_info("In %s\n", __func__);

	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_cht_mc_shutdown(struct platform_device *pdev)
{
	int ret_val;
	pr_info("In %s turn off SPK. \n", __func__);
  
	if(yt3_hw_ver == 0)
	    gpio_set_value_cansleep(EXT_SPEAKER_ENABLE_PIN , 0);
	else
	    gpio_set_value(EXT_SPEAKER_ENABLE_SOC_GPIO, 0);
    
	ret_val = gpio_request_one(99, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "codec clk");
	if (ret_val)
		pr_err("snd_cht_mc_shutdown() codec clk gpio config failed\n");

	ret_val = gpio_request_one(136, GPIOF_DIR_OUT|GPIOF_INIT_LOW, "codec 32k");
	if (ret_val)
		pr_err("snd_cht_mc_shutdown() codec clk gpio config failed\n");
}

const struct dev_pm_ops snd_cht_mc_pm_ops = {
	.prepare = snd_cht_prepare,
	.complete = snd_cht_complete,
	.poweroff = snd_cht_poweroff,
};

static const struct acpi_device_id cht_mc_acpi_ids[] = {
	{ "AMCR0F28", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, cht_mc_acpi_ids);

static struct platform_driver snd_cht_mc_driver = { //Done
	.driver = {
		.owner = THIS_MODULE,
		.name = "cht_wm5102",
		.pm = &snd_cht_mc_pm_ops,
		//.acpi_match_table = ACPI_PTR(byt_mc_acpi_ids),
	},
	.probe = snd_cht_mc_probe,
	.remove = snd_cht_mc_remove,
	.shutdown = snd_cht_mc_shutdown,
};

static int __init snd_cht_driver_init(void)
{
	pr_info("Cherrytrail Machine Driver cht_wm5102 registerd\n");
	return platform_driver_register(&snd_cht_mc_driver);
}
late_initcall(snd_cht_driver_init);

static void __exit snd_cht_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_cht_mc_driver);
}
module_exit(snd_cht_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) CHT Machine driver");
MODULE_AUTHOR("Omair Md Abdullah <omair.m.abdullah@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht_wm5102");
