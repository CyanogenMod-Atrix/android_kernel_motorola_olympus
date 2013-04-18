/*
 * motsnd.c  --  SoC audio for Motorola Android Platform
 *
 * Author: Motorola
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/cpcap_audio_platform_data.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include <asm/mach-types.h>

#include "tegra20_das.h"
#include "tegra20_i2s.h"
#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"
#include "../codecs/cpcap.h"




#define MOTSND_DEBUG 
#ifdef MOTSND_DEBUG
#define MOTSND_DEBUG_LOG(args...) printk(KERN_INFO "ALSA MOTSND:" args)
#else
#define MOTSND_DEBUG_LOG(args...)
#endif

/*global variable for PM to track call state*/
extern int g_is_call_mode;

#define DPLL_ABE_RATE_SPDIF	90315789
struct mot_card_data {
	struct tegra_asoc_utils_data util_data;
};


static int motsnd_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
        struct mot_card_data *card_data =snd_soc_card_get_drvdata(card);
	int ret;
	unsigned long rate;
	struct clk *dpll_abe_ck;
	int srate, mclk, mclk_change;
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;
	if( card_data  )
	{
	ret = tegra_asoc_utils_set_rate(&card_data->util_data, srate, mclk);

	if (ret < 0) {
		printk(KERN_ERR  "Can't configure clocks\n");
		return ret;
	}
	}else {
		printk(KERN_ERR  "card_data is NULL\n");
	}
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
//	if (mclk_change) {
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}
//	}
	/* Make Das connection */

	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dac-dap path\n");
		return ret;
	}



	return ret;
}

static struct snd_soc_ops motsnd_ops = {
	.hw_params = motsnd_hw_params,
};

static int motsnd_voice_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	/* Make Das connection */
	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_2);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_2,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dac-dap path\n");
		return ret;
	}
	return ret;
}

static struct snd_soc_ops motsnd_voice_ops = {
	.hw_params = motsnd_voice_hw_params,
};

static int motsnd_incall_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
//		mcbsp3_i2s1_pin_mux_switch(1);
	}
	/* Make Das connection */

	ret = tegra20_das_connect_dap_to_dap(TEGRA20_DAS_DAP_ID_2,
					TEGRA20_DAS_DAP_SEL_DAP3, true,0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dap path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dap(TEGRA20_DAS_DAP_ID_3,
					TEGRA20_DAS_DAP_SEL_DAP2, false,0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dap path\n");
		return ret;
	}

	if(!g_is_call_mode)
		g_is_call_mode = true;

	return 0;
}

static void motsnd_incall_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
//		mcbsp3_i2s1_pin_mux_switch(0);
	}
	/* Make Das connection */
	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dac-dap path\n");
		return ret;
	}

	if(g_is_call_mode)
		g_is_call_mode = false;


}
static int motsnd_incall_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
}

static struct snd_soc_ops motsnd_incall_ops = {
	.startup = motsnd_incall_startup,
	.shutdown = motsnd_incall_shutdown,
	.hw_params = motsnd_incall_hw_params,
};

static int motsnd_fm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
}

static struct snd_soc_ops motsnd_fm_ops = {
	.hw_params = motsnd_fm_hw_params,
};

static int motsnd_bt_incall_startup(struct snd_pcm_substream *substream)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if(!g_is_call_mode)
		g_is_call_mode = true;
}

static void motsnd_bt_incall_shutdown(struct snd_pcm_substream *substream)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if(g_is_call_mode)
		g_is_call_mode = false;
}

static int motsnd_bt_incall_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);


	/* MDM6600 */
	/* Set cpu DAI configuration */
/*	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_IF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}
*/
	/* Make Das connection */

	ret = tegra20_das_connect_dap_to_dap(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAP3, true,0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dap path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dap(TEGRA20_DAS_DAP_ID_3,
					TEGRA20_DAS_DAP_SEL_DAP4, false,0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dap path\n");
		return ret;
	}
	return ret;
}

static struct snd_soc_ops motsnd_bt_incall_ops = {
	.startup = motsnd_bt_incall_startup,
	.shutdown = motsnd_bt_incall_shutdown,
	.hw_params = motsnd_bt_incall_hw_params,
};

/*Fix ME: BT SCO Bringup for AP20 (e.g VR app)*/
static int motsnd_btvoice_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);


	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Make Das connection */
	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
			TEGRA20_DAS_DAP_ID_4);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
			TEGRA20_DAS_DAP_SEL_DAC2);
	if (ret < 0) {
		printk(KERN_ERR  "failed to set dac-dap path\n");
               return ret;
       }

	return ret;
}

static struct snd_soc_ops motsnd_btvoice_ops = {
	.hw_params = motsnd_btvoice_hw_params,
};

static int motsnd_spdif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct mot_card_data *card_data = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&card_data->util_data, srate, mclk);
	if (err < 0) {
		if (!(card_data->util_data.set_mclk % min_mclk))
			mclk = card_data->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&card_data->util_data, 1);

	return 0;
}

static struct snd_soc_ops motsnd_spdif_ops = {
	.hw_params = motsnd_spdif_hw_params,
};

static int motsnd_bpvoice_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_IB_NF |
					  SND_SOC_DAIFMT_CBM_CFM);

		if (ret < 0) {
			printk(KERN_ERR "can't set cpu DAI configuration\n");
			return ret;
		}
	} else {
		ret = -EIO;
		printk(KERN_ERR "%s: voice_type = %u, not valid modem",
			__func__, pdata->voice_type);
		return ret;
	}
	/* Set cpu DAI configuration */
/*	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     0, 0);
	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");
*/	return ret;
}

static struct snd_soc_ops motsnd_bpvoice_ops = {
	.hw_params = motsnd_bpvoice_hw_params,
};

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
/*	unsigned int be_id = rtd->dai_link->be_id;
	unsigned int threshold;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		channels->min = 2;
		threshold = 2;
		break;
	case OMAP_ABE_DAI_BT_VX:
		channels->min = 1;
		threshold = 1;
		break;
	default:
		threshold = 1;
		break;
	}

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
			SNDRV_PCM_FORMAT_S16_LE);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);
*/
	return 0;
}
static const struct snd_soc_dapm_widget tegra_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", NULL),
};

static const struct snd_soc_dapm_route tegra_audio_map[] = {
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
};

static const struct snd_kcontrol_new tegra_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int motsnd_tegra_mm_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	int ret;


	ret = snd_soc_add_controls(codec, tegra_controls,
				   ARRAY_SIZE(tegra_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, tegra_dapm_widgets,
					ARRAY_SIZE(tegra_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, tegra_audio_map,
				ARRAY_SIZE(tegra_audio_map));


	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTR");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int motsnd_cpcap_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static int motsnd_cpcap_voice_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static struct snd_soc_dai_driver dai[] = {
{
	.name = "MODEM",
	.playback = {
		.stream_name = "VoiceCall-DL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "VoiceCall-UL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "FMDummy",
	.playback = {
		.stream_name = "FM-Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
/*
#ifdef CONFIG_SND_OMAP_SOC_HDMI
{
	.name = "HDMI",
	.playback = {
		.stream_name = "HDMI-Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
},
#endif
*/
};
/*
static const char *mm1_be[] = {
		OMAP_ABE_BE_MM_EXT0,

};
*/

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link motsnd_dai[] = {
{
	.name = "Multimedia LP",
	.stream_name = "Multimedia",
	.codec_name = "cpcap_audio",
	.cpu_dai_name = "tegra20-i2s.0",
	.codec_dai_name = "cpcap stdac",
	.platform_name = "tegra-pcm-audio",
	.init = motsnd_tegra_mm_init,
//	.dynamic = 1,
//	.supported_be = mm1_be,
//	.num_be = ARRAY_SIZE(mm1_be),
//	.fe_playback_channels = 2,
	.ops = &motsnd_ops,
//	.ignore_suspend = 1,
},
{
	.name = "Voice",
	.stream_name = "Tegra-i2s.1",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap codec",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_voice_ops,
	.ignore_suspend = 1,
},

{
	.name = "VoiceCall",
	.stream_name = "Tegra-i2s.1",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap in-call",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_incall_ops,
	.ignore_suspend = 1,
},
/*
{
	.name = "FMRadio",
	.stream_name = "FMAudio",
	.cpu_dai_name = "FMDummy",
	.codec_dai_name = "cpcap fm",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_fm_ops,
	.ignore_suspend = 1,
},
*/
#ifdef CONFIG_SND_SOC_TEGRA20_SPDIF
{
	.name = "SPDIF",
	.stream_name = "SPDIF PCM",
	.codec_name = "spdif-dit.0",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-spdif",
	.codec_dai_name = "dit-hifi",
	.ops = &motsnd_spdif_ops,
},
#endif

{
	.name = "BTCall",
	.stream_name = "Modem-BT",
	.cpu_dai_name = "MODEM",
	.codec_dai_name = "cpcap bt-call",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_bt_incall_ops,
	.ignore_suspend = 1,
},

{
	.name = "BTVoice",
	.stream_name = "Tegra-BT",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap bt",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_btvoice_ops,
	.ignore_suspend = 1,
},
/*
{
	.name = "BPVoice",
	.stream_name = "Tegra-BP",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "BPVoice",
	.platform_name = "tegra-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_bpvoice_ops,
	.ignore_suspend = 1,
},
*/
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mot = {
	.name = "motsnd",
	.dai_link = motsnd_dai,
	.num_links = ARRAY_SIZE(motsnd_dai),
};


static struct platform_device *mot_snd_device;

static int __init motsnd_soc_init(void)
{
	int ret;
        struct mot_card_data *card_data;
	pr_info("ENTER: MOTSND SoC init\n");
	mot_snd_device = platform_device_alloc("soc-audio", -1);
	if (!mot_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	snd_soc_register_dais(&mot_snd_device->dev, dai, ARRAY_SIZE(dai));

	card_data = kzalloc(sizeof(struct mot_card_data), GFP_KERNEL);
	if (!card_data) {
		printk(KERN_ERR "Can't allocate tegra asoc utils data\n");
		return -ENOMEM;
	}

	ret = tegra_asoc_utils_init(&card_data->util_data,&mot_snd_device->dev, &snd_soc_mot);
	if (ret)
		goto err_free;
	platform_set_drvdata(mot_snd_device, &snd_soc_mot);
	snd_soc_card_set_drvdata(&snd_soc_mot, card_data);

	ret = platform_device_add(mot_snd_device);
	if (ret)
		goto err;

	return 0;

err:
	tegra_asoc_utils_fini(&card_data->util_data);
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(mot_snd_device);
err_free:
	kfree(card_data);
	return ret;
}
module_init(motsnd_soc_init);

static void __exit motsnd_soc_exit(void)
{
	platform_device_unregister(mot_snd_device);
}
module_exit(motsnd_soc_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("ALSA SoC MOTSND");
MODULE_LICENSE("GPL");
