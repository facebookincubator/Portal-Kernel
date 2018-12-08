/*
 * esxxx-codec.c  --  Audience dummy codec
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Sheetal Garg <sheetal.garg@knowles.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include "esxxx-codec.h"
#include "escore.h"
#include "es705_escore.h"
#include "es705-export.h"

#define I2S_HW_PARAMS		es705_i2s_hw_params
#define I2S_SHUTDOWN		es705_i2s_shutdown

int (*i2s_hw_params)(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
		= I2S_HW_PARAMS;

void (*i2s_shutdown)(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
		= I2S_SHUTDOWN;

struct mutex hw_params_mutex;
static struct escore_i2s_dai_data i2s_dai_data[ES_NUM_CODEC_I2S_DAIS];
int total_num_of_ports;

static void esxxx_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int id = DAI_INDEX(dai->id);

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
			dai->name, dai->id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		escore->i2s_dai_data[id].rx_ch_tot = 0;
	else
		escore->i2s_dai_data[id].tx_ch_tot = 0;

	if (escore->can_mpsleep)
		escore->can_mpsleep = 0;
	i2s_shutdown(substream, dai);
}

static int esxxx_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	int id = DAI_INDEX(dai->id);

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d, fmt = %x\n",
			__func__, dai->name, dai->id, fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* es705 as master */
		i2s_dai_data[id].port_mode = ES_PCM_PORT_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* es705 as slave */
		i2s_dai_data[id].port_mode = ES_PCM_PORT_SLAVE;
		break;
	default:
		dev_err(codec->dev, "%s(): unsupported DAI clk mode\n",
				__func__);
		return -EINVAL;
	}
	dev_dbg(codec->dev, "%s(): clk mode = %d\n", __func__,
			i2s_dai_data[id].port_mode);

	return 0;
}


static int esxxx_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	struct escore_api_access *api_access;
	int rc = 0;
	int channels;
	int bps = 0;
	int rate = 0;
	int id = DAI_INDEX(dai->id);
	u16 clock_control = 0;
	int port_map = 0;
	u8 pcm_port[] = { ES705_PCM_PORT_A,
		ES705_PCM_PORT_B,
		ES705_PCM_PORT_C };

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
			dai->name, dai->id);
	pr_debug("%s(): dai->name = %s, dai->id = %d\n", __func__,
			dai->name, dai->id);

	mutex_lock(&hw_params_mutex);
	channels = params_channels(params);
	switch (channels) {
	case 1:
	case 2:
	case 3:
	case 4:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			escore->i2s_dai_data[id].rx_ch_tot = channels;
			escore->i2s_dai_data[id].rx_ch_act = 0;
		} else {
			escore->i2s_dai_data[id].tx_ch_tot = channels;
			escore->i2s_dai_data[id].tx_ch_act = 0;
		}
		break;
	default:
		dev_err(codec->dev,
				"%s(): unsupported number of channels, %d\n",
				__func__, channels);
		mutex_unlock(&hw_params_mutex);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_A_LAW:
		bps = 0x0207;
		break;
	case SNDRV_PCM_FORMAT_MU_LAW:
		bps = 0x0107;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		bps = 0xF;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		bps = 0x13;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
		bps = 0x17;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		bps = 0x1F;
		break;
	default:
		dev_err(codec->dev, "%s(): Unsupported format :%d\n",
				__func__, params_format(params));
		break;
	}

	if (bps) {
		api_access = &escore->api_access[ES705_PORT_WORD_LEN];

		/* Update the Port info in write command */
		api_access->write_msg[0] |=  ES_API_WORD(ES705_SET_DEV_PARAM_ID,
				(pcm_port[id] << 8));

		rc = escore_write_locked(codec, ES705_PORT_WORD_LEN, bps);
		if (rc) {
			pr_err("%s(): Preparing write message failed %d\n",
					__func__, rc);
			mutex_unlock(&hw_params_mutex);
			return rc;
		}
		/* Clear the Port info in write command */
		api_access->write_msg[0] &= ES_API_WORD(ES705_SET_DEV_PARAM_ID,
				0x00ff);
	}

	switch (params_rate(params)) {
	case 8000:
		rate = 8;
		break;
	case 11025:
		rate = 11;
		break;
	case 12000:
		rate = 12;
		break;
	case 16000:
		rate = 16;
		break;
	case 22050:
		rate = 22;
		break;
	case 24000:
		rate = 24;
		break;
	case 32000:
		rate = 32;
		break;
	case 44100:
		rate = 44;
		break;
	case 48000:
		rate = 48;
		break;
	case 96000:
		rate = 96;
		break;
	case 192000:
		rate = 192;
		break;
	default:
		pr_err("%s: Unsupported sampling rate %d\n", __func__,
				params_rate(params));
		mutex_unlock(&hw_params_mutex);
		return -EINVAL;
	}
	dev_dbg(codec->dev, "%s(): params_rate(params) = %d\n",
			__func__, params_rate(params));

	api_access = &escore->api_access[ES705_PORT_CLOCK_CONTROL];

	/* Update the Port info in write command */
	api_access->write_msg[0] |=  ES_API_WORD(ES705_SET_DEV_PARAM_ID,
			(pcm_port[id] << 8));

	clock_control  = rate | (i2s_dai_data[id].port_mode << 8);

	rc = escore_write_locked(codec, ES705_PORT_CLOCK_CONTROL,
				clock_control);
	if (rc) {
		pr_err("%s(): Preparing write message failed %d\n",
				__func__, rc);
		mutex_unlock(&hw_params_mutex);
		return rc;
	}

	/* Clear the Port info in write command */
	api_access->write_msg[0] &= ES_API_WORD(ES705_SET_DEV_PARAM_ID, 0x00ff);

	/*
	 ** To enter into MP_SLEEP mode during playback, a minimum 3Mhz clock
	 ** is required. For that, minimum 48Khz sample rate, 32 bit word length
	 ** and 2 channels are required.
	 */
	escore->can_mpsleep = (rate == 48) && (bps == 0x1F) && (channels == 2);

	switch (dai->id) {
	case ES_I2S_PORTA:
		port_map = PORT_A_TO_D;
		break;
	case ES_I2S_PORTB:
		port_map = PORT_B_TO_D;
		break;
	case ES_I2S_PORTC:
		port_map = PORT_C_TO_D;
		break;
	}

	if (escore->can_mpsleep || escore->dhwpt_enabled)
		escore->dhwpt_cmd = (ES_DHWPT_CMD << 16) | port_map;
	else
		escore->dhwpt_cmd = 0;

	dev_dbg(codec->dev, "%s(): params_channels(params) = %d\n", __func__,
			channels);
	rc = i2s_hw_params(substream, params, dai);
	mutex_unlock(&hw_params_mutex);

	return rc;
}

static int esxxx_codec_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_info("%s : event = 0x%x\n", __func__, event);
	return 0;
}

static int esxxx_codec_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_info("%s : event = 0x%x\n", __func__, event);
	return 0;
}
static const struct snd_soc_dapm_widget esxxx_dapm_widgets[] = {

	SND_SOC_DAPM_AIF_OUT_E("PCM0.0 TX", "PORTA Capture", 0,
			SND_SOC_NOPM, 1, 0, esxxx_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM0.0 RX", "PORTA Playback", 0,
			SND_SOC_NOPM, 1, 0, esxxx_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM1.0 TX", "PORTB Capture", 0,
			SND_SOC_NOPM, 1, 0, esxxx_codec_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM1.0 RX", "PORTB Playback", 0,
			SND_SOC_NOPM, 1, 0, esxxx_codec_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("PCM0.0 Tx Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("PCM1.0 Tx Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("PCM0.0 Rx Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("PCM1.0 Rx Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIC("PDMI0", NULL),
	SND_SOC_DAPM_MIC("PDMI1", NULL),
	SND_SOC_DAPM_OUTPUT("PCM_SINK"),
	SND_SOC_DAPM_OUTPUT("PCM1_SINK"),
};

static const struct snd_soc_dapm_route esxxx_dapm_routes[] = {
	{"PCM0.0 TX", NULL, "PCM0.0 Tx Mixer"},
	{"PCM0.0 Tx Mixer", NULL, "PDMI0"},
	{"PCM_SINK", NULL, "PCM0.0 Rx Mixer"},
	{"PCM0.0 Rx Mixer", NULL, "PCM0.0 RX"},
	{"PCM1.0 TX", NULL, "PCM1.0 Tx Mixer"},
	{"PCM1.0 Tx Mixer", NULL, "PDMI1"},
	{"PCM1_SINK", NULL, "PCM1.0 Rx Mixer"},
	{"PCM1.0 Rx Mixer", NULL, "PCM1.0 RX"},
};

static int esxxx_add_dapm_widgets(struct snd_soc_codec *codec)
{
	int rc = 0;
	rc = snd_soc_dapm_new_controls(snd_soc_codec_get_dapm(codec),
			esxxx_dapm_widgets, ARRAY_SIZE(esxxx_dapm_widgets));
	return rc;
}

static int esxxx_add_dapm_routes(struct snd_soc_codec *codec)
{
	int rc = 0;
	rc = snd_soc_dapm_add_routes(snd_soc_codec_get_dapm(codec),
			esxxx_dapm_routes, ARRAY_SIZE(esxxx_dapm_routes));
	return rc;
}

static int esxxx_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	int rc = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	snd_soc_codec_get_dapm(codec)->bias_level = level;

	return rc;
}

static int esxxx_codec_probe(struct snd_soc_codec *codec)
{
	int rc = 0;
	struct escore_priv *es705 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s()\n", __func__);
	es705->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);

	esxxx_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	/* Add codec controls */
	rc = es705_remote_add_codec_controls(codec);

	/* Add dapm widgets */
	rc = esxxx_add_dapm_widgets(codec);

	/* Add dapm routes */
	rc = esxxx_add_dapm_routes(codec);

	mutex_init(&hw_params_mutex);
	es705->i2s_dai_data = i2s_dai_data;
	es705->i2s_dai_ops.hw_params = esxxx_i2s_hw_params;
	es705->i2s_dai_ops.set_fmt   = esxxx_i2s_set_fmt;
	es705->i2s_dai_ops.shutdown = esxxx_i2s_shutdown;
	return 0;
}

static int  esxxx_codec_remove(struct snd_soc_codec *codec)
{
	struct escore_priv *es705 = snd_soc_codec_get_drvdata(codec);

	esxxx_set_bias_level(codec, SND_SOC_BIAS_OFF);

	kfree(es705);
	es705 = NULL;

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_esxxx = {
	.probe =	esxxx_codec_probe,
	.remove =	esxxx_codec_remove,
	.set_bias_level =	esxxx_set_bias_level,
};
