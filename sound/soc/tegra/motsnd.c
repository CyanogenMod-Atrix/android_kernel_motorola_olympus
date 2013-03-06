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

#define ABE_BYPASS
//#define MOTSND_CONFIG_ENABLE_ABE
//#define MOTSND_CONFIG_ENABLE_SPDIF

#include "../codecs/cpcap.h"

#define MOTSND_DEBUG 1
#ifdef MOTSND_DEBUG
#define MOTSND_DEBUG_LOG(args...) printk(KERN_INFO "ALSA MOTSND:" args)
#else
#define MOTSND_DEBUG_LOG(args...)
#endif

#define DPLL_ABE_RATE_SPDIF	90315789

#define DRV_NAME "tegra-snd-olympus"

struct tegra_olympus {
	struct tegra_asoc_utils_data util_data;
};

static int motsnd_hw_params(struct snd_pcm_substream *substream, 	//verified
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_olympus *card_data = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err, ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	if (card_data==NULL) {
		dev_err(card->dev, "card_data is NULL\n");
		return -1;
	}

	/* Set codec clock configuration */
	srate = params_rate(params);
	mclk = 128 * srate;

	printk("%s: mclk (%d) = 128 * srate (%d);\n", __func__, mclk, srate);

	err = tegra_asoc_utils_set_rate(&card_data->util_data, srate, mclk);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
/*	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);*/
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	ret = tegra20_das_connect_dac_to_dap(0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(0,0);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dac-dap path\n");
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
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_A |
				SND_SOC_DAIFMT_IB_NF |
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

	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_2);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_3,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dac-dap path\n");
		return ret;
	}

	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");

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

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
	//	mcbsp3_i2s1_pin_mux_switch(1);
	}
	return 0;
}

static void motsnd_incall_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		//mcbsp3_i2s1_pin_mux_switch(0);
	}
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

static int motsnd_btvoice_hw_params(struct snd_pcm_substream *substream,
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
	} else if (pdata->voice_type == VOICE_TYPE_QC) {
		/* MDM6600 */
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_IB_IF |
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

	ret = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dap-dac path\n");
		return ret;
	}

	ret = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (ret < 0) {
		printk(KERN_ERR "failed to set dac-dap path\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops motsnd_btvoice_ops = {
	.hw_params = motsnd_btvoice_hw_params,
};

static int motsnd_bt_incall_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
//	int ret;
	
	pr_info("ENTER: %s\n", __func__);

//	return ret;
	return 0;
};

static struct snd_soc_ops motsnd_bt_incall_ops = {
	.hw_params = motsnd_bt_incall_hw_params,
};

static int motsnd_spdif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
};

static struct snd_soc_ops motsnd_spdif_ops = {
	.hw_params = motsnd_spdif_hw_params,
};

/*
static int motsnd_cpcap_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}*/

static int motsnd_cpcap_voice_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);

	return 0;
}

static const struct snd_soc_dapm_widget tegra_dapm_widgets[] = {	//verified
	SND_SOC_DAPM_SPK("Int Spk", NULL),
};

static const struct snd_soc_dapm_route tegra_audio_map[] = { 	//verified
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
};

static const struct snd_kcontrol_new tegra_controls[] = {	//verified
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int motsnd_tegra_mm_init(struct snd_soc_pcm_runtime *rtd) {	//verified
	
	int ret;

	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	pr_info("ENTER: %s\n",__func__);

	ret = snd_soc_add_controls(codec, tegra_controls, ARRAY_SIZE(tegra_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, tegra_dapm_widgets, ARRAY_SIZE(tegra_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, tegra_audio_map, ARRAY_SIZE(tegra_audio_map));

	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
	snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	snd_soc_dapm_sync(dapm);
	pr_info("EXIT: %s\n",__func__);

	return 0;
}

static struct snd_soc_dai_driver dai[] = { 	//verified
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
}
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link motsnd_dai[] = {		//verified
{
	.name = "Multimedia LP",
	.stream_name = "Multimedia",
	.codec_name = "cpcap_audio",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-i2s.0",
	.codec_dai_name = "cpcap stdac",
//	.ignore_suspend = ,
//	.symmetric_rates = ,
	.init = motsnd_tegra_mm_init,
	.ops = &motsnd_ops,
},
{
	.name = "Voice",
	.stream_name = "Tegra-i2s.1",
	.codec_name = "cpcap_audio",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap codec",
	.ignore_suspend = 1,
//	.symmetric_rates = ,
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_voice_ops,
},
{
	.name = "VoiceCall",
	.stream_name = "Tegra-i2s.1",
	.codec_name = "cpcap_audio",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap in-call",
	.ignore_suspend = 1,
//	.symmetric_rates = ,
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_incall_ops,
},
{
	.name = "SPDIF",
	.stream_name = "SPDIF PCM",
	.codec_name = "spdif-dit.0",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-spdif",
	.codec_dai_name = "dit-hifi",
//	.ignore_suspend = ,
//	.symmetric_rates = ,
//	.init = ,
	.ops = &motsnd_spdif_ops,
},
{
	.name = "BTCall",
	.stream_name = "Modem-BT",
	.codec_name = "cpcap_audio",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "MODEM",
	.codec_dai_name = "cpcap bt-call",
	.ignore_suspend = 1,
//	.symmetric_rates = ,
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_bt_incall_ops,
},
{
	.name = "BTVoice",
	.stream_name = "TEGRA-BT",
	.codec_name = "cpcap_audio",
	.platform_name = "tegra-pcm-audio",
	.cpu_dai_name = "tegra20-i2s.1",
	.codec_dai_name = "cpcap bt",
	.ignore_suspend = 1,
//	.symmetric_rates = ,
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_btvoice_ops,
},
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mot = { 	//verified
	.name = "motsnd",
//	.long_name = "Motorola OLYMPUS",
	.dai_link = motsnd_dai,
	.num_links = ARRAY_SIZE(motsnd_dai),
};

static struct platform_device *mot_snd_device;

static int __init motsnd_soc_init(void)	//verified
{
	struct snd_soc_card *card = &snd_soc_mot;
	struct tegra_olympus *olympus;
	int ret;

	pr_info("ENTER: MOTSND SoC init\n");

	mot_snd_device = platform_device_alloc("soc-audio", -1);
	if (!mot_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	olympus = kzalloc(sizeof(struct tegra_olympus), GFP_KERNEL);
	if (!olympus) {
		dev_err(&mot_snd_device->dev, "Can't allocate tegra asoc utils data\n");
		return -ENOMEM;
	}

	ret = tegra_asoc_utils_init(&olympus->util_data, &mot_snd_device->dev, card);
	if (ret) {
		dev_err(&mot_snd_device->dev, "Can't do tegra_asoc_utils_init()\n");
		goto err_free_olympus;
	}
	pr_info("MOTSND SoC init: snd_soc_register_dais\n");
	snd_soc_register_dais(&mot_snd_device->dev, dai, ARRAY_SIZE(dai));

	pr_info("MOTSND SoC init: platform_set_drvdata\n");
//	snd_soc_card_set_drvdata(card, mot_snd_device);
	platform_set_drvdata(mot_snd_device, card);

	pr_info("%s: mot_snd_device: id: %d, name: %s\n", __func__, mot_snd_device->id, mot_snd_device->name);
	
	pr_info("MOTSND SoC init: platform_device_add\n");
	ret = platform_device_add(mot_snd_device);
	if (ret) 
		goto err; 

	return 0;

err:
	tegra_asoc_utils_fini(&olympus->util_data);
	printk(KERN_ERR "Unable to add platform device\n");
err_free_olympus:
	platform_device_put(mot_snd_device);
	kfree(olympus);
	return ret;
}
module_init(motsnd_soc_init);

static void __exit motsnd_soc_exit(void)	//verified
{
	platform_device_unregister(mot_snd_device);
}
module_exit(motsnd_soc_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("ALSA SoC MOTSND");
MODULE_LICENSE("GPL");
