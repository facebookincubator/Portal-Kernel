/*
 * es-n100.h  --  Audience esn100 ALSA SoC Dummy Audio driver
 * only for VoiceQ and CVQ
 *
 * Copyright 2011 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <sound/soc.h>
#include <linux/time.h>

#include "escore.h"
#include "escore-i2s.h"
#include "escore-cdev.h"
#include "escore-vs.h"
#include "escore-version.h"

#include "es-n100.h"
#include "es-n100-routes.h"
#include "es-n100-access.h"
#define DEBUG
/* Global structures */
/*** TODO: Add I2S and Slimbus support for pass-through route ***/
#define ESN100_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define ESN100_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |\
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |\
			SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)

/*** TODO: Add I2S and Slimbus support for pass-through route ***/
struct snd_soc_dai_driver esn100_dai[] = {

#if defined(CONFIG_SND_SOC_ES_I2S)
	{
		.name = "earSmart-porta",
		.id = ES_I2S_PORTA,
		.capture = {
			.stream_name = "PORTA Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ESN100_RATES,
			.formats = ESN100_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
	{
		.name = "earSmart-portb",
		.id = ES_I2S_PORTB,
		.capture = {
			.stream_name = "PORTB Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ESN100_RATES,
			.formats = ESN100_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
#endif
};

/*** KControl enums ***/
static const char * const esn100_vs_power_state_texts[] = {
	"None", "Sleep", "MP_Sleep", "MP_Cmd", "Normal", "Overlay", "Low_Power"
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
	, "VS_Streaming"
#endif
};

static const struct soc_enum esn100_vs_power_state_enum =
	SOC_ENUM_SINGLE(ESN100_POWER_STATE, 0,
			ARRAY_SIZE(esn100_vs_power_state_texts),
			esn100_vs_power_state_texts);

static const char * const esn100_power_level_texts[] = {
	"0 [Min]", "1", "2", "3", "4", "5", "6 [Max, Def]"
};

static const struct soc_enum esn100_power_level_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(esn100_power_level_texts),
			esn100_power_level_texts);

static const char * const esn100_vs_event_texts[] = {
	"No Event", "Codec Event", "VS Keyword Event",
};
static const struct soc_enum esn100_vs_event_enum =
	SOC_ENUM_SINGLE(ESN100_VOICE_SENSE_EVENT, 0,
			ARRAY_SIZE(esn100_vs_event_texts),
			esn100_vs_event_texts);

static const char * const esn100_vs_training_mode_texts[] = {
	"Detect Keyword", "N/A", "Train User-defined Keyword",
};

static const struct soc_enum esn100_vs_training_mode_enum =
	SOC_ENUM_SINGLE(ESN100_VOICE_SENSE_TRAINING_MODE, 0,
			ARRAY_SIZE(esn100_vs_training_mode_texts),
			esn100_vs_training_mode_texts);

static const char * const esn100_vs_training_status_texts[] = {
	"busy", "Success", "Utterance Long", "Utterance Short",
	"Verification Failed", "Failed Bad length", "Noise", "Level too low",
};

static const struct soc_enum esn100_vs_training_status_enum =
	SOC_ENUM_SINGLE(ESN100_VOICE_SENSE_TRAINING_STATUS, 0,
			ARRAY_SIZE(esn100_vs_training_status_texts),
			esn100_vs_training_status_texts);

static const char * const esn100_vs_training_record_texts[] = {
	"Start_0", "Start_1", "Start_2",
};

static const struct soc_enum esn100_vs_training_record_enum =
	SOC_ENUM_SINGLE(ESN100_VOICE_SENSE_TRAINING_RECORD, 0,
			ARRAY_SIZE(esn100_vs_training_record_texts),
			esn100_vs_training_record_texts);

static const char * const esn100_streaming_mode_texts[] = {
	"CVQ", "Non-CVQ",
};

static const struct soc_enum esn100_streaming_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(esn100_streaming_mode_texts),
			esn100_streaming_mode_texts);

/*** KControl function callbacks ***/
static int esn100_get_power_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = escore_priv.escore_power_state;

	return 0;
}

static int esn100_wakeup(struct escore_priv *es705)
{
	int rc = 0;

	rc = escore_wakeup(es705);

	return rc;
}

static int esn100_start_int_osc(void)
{
	int rc = 0;
	int retry = MAX_RETRY_TO_SWITCH_TO_LOW_POWER_MODE;
	struct escore_priv *escore = &escore_priv;

	dev_info(escore_priv.dev, "%s()\n", __func__);

	/* Start internal Osc. */
	rc = escore_write(escore->codec, ESN100_VS_INT_OSC_MEASURE_START, 0);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): OSC Measure Start fail %d\n", __func__, rc);
		return rc;
	}

	/* Poll internal Osc. status */
	do {
		/*
		 * Wait 20ms each time before reading
		 * up to 100ms
		 */
		msleep(20);
		rc = escore_read(escore->codec, ESN100_VS_INT_OSC_MEASURE_STATUS);

		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): OSC Measure Read Status fail %d\n",
				__func__, rc);
			break;
		}
		dev_dbg(escore_priv.dev,
			"%s(): OSC Measure Status = 0x%04x\n",
			__func__, rc);
	} while (rc && --retry);

	if (rc > 0) {
		dev_err(escore_priv.dev,
			"%s(): Unexpected OSC Measure Status = 0x%04x\n",
			__func__, rc);
		dev_err(escore_priv.dev,
			"%s(): Can't switch to Low Power Mode\n",
			__func__);
	}

	return rc;
}

static int esn100_power_transition(int next_power_state,
				unsigned int set_power_state_cmd)
{
	struct escore_priv *escore = &escore_priv;
	int rc = 0;
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
	u32 es_get_power_state = ES_GET_POWER_STATE << 16;
	int resp;
#endif

	while (next_power_state != escore->escore_power_state) {
		switch (escore->escore_power_state) {
		case ES_SET_POWER_STATE_SLEEP:
			/* Wakeup Chip */
			rc = esn100_wakeup(escore);
			if (rc) {
				pr_err("%s(): Wakeup failed: %d\n",
						__func__, rc);
				goto power_transition_exit;
			}
			escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;
			break;
		case ES_SET_POWER_STATE_NORMAL:
			/* Either switch to Sleep or VS Overlay mode */
#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
			if (next_power_state == ES_SET_POWER_STATE_SLEEP) {
				escore->escore_power_state =
					ES_SET_POWER_STATE_SLEEP;
				escore_set_vs_download_req(escore, true);
			} else {
				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_OVERLAY;
			}

#else
			if (next_power_state == ES_SET_POWER_STATE_SLEEP) {
				escore->escore_power_state =
					ES_SET_POWER_STATE_SLEEP;
				/* Set flag to Wait for API Interrupt */
				if (escore->pdata->gpioa_gpio != -1)
					escore_set_api_intr_wait(escore);
			 } else
				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_OVERLAY;

#endif

#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
			if ((escore_get_vs_download_req(escore) == false) &&
			    (escore->escore_power_state ==
					ES_SET_POWER_STATE_VS_OVERLAY)) {
				/*
				 * VS binary already copied.
				 * So No need to re-download Binary.
				 * But send the Overlay Power State Command
				 */
				rc = escore_write(escore->codec, set_power_state_cmd,
						   ES_POWER_STATE_VS_OVERLAP);
			} else {
				rc = escore_write(escore->codec, set_power_state_cmd,
						escore->escore_power_state);
			}
#else
			rc = escore_write(escore->codec, set_power_state_cmd,
					escore->escore_power_state);
#endif
			if (rc) {
				pr_err("%s(): Failed to set power state :%d\n",
					__func__, rc);
				escore->escore_power_state =
					ES_SET_POWER_STATE_NORMAL;
				goto power_transition_exit;
			}

			/* VS fw download */
			if (escore->escore_power_state ==
					ES_SET_POWER_STATE_VS_OVERLAY) {
				/* wait esn100 SBL mode */
				msleep(50);

#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
				if (escore_get_vs_download_req(escore)
								== true) {
					rc = escore_vs_load(&escore_priv);
					if (rc) {
						pr_err("%s(): VS fw download fail %d\n",
						       __func__, rc);
						escore->escore_power_state =
						ES_SET_POWER_STATE_NORMAL;
						goto power_transition_exit;
					}
					escore_set_vs_download_req(escore,
								   false);
				} else {
					/* Setup the Event response */
					cmd = (ES_SET_EVENT_RESP << 16) | \
						escore->pdata->gpio_b_irq_type;
					rc = escore_cmd_nopm(escore, cmd,
									&resp);
					if (rc < 0) {
						pr_err("%s(): Error %d in setting event response\n",
								__func__, rc);
						goto power_transition_exit;
					}

				}
#else
				rc = escore_vs_load(&escore_priv);
				if (rc) {
					pr_err("%s(): vs fw download fail %d\n",
					       __func__, rc);
					goto power_transition_exit;
				}
			} else if (escore->pdata->gpioa_gpio != -1) {
				/* Wait for API Interrupt to confirm
				 * that device is in sleep mode */
				rc = escore_api_intr_wait_completion(escore);
				if (rc) {
					pr_err("%s(): Sleep wait failed %d\n",
						__func__, rc);
					goto power_transition_exit;
				}
				/* Reset Interrupt mode after sleep */
				escore->cmd_compl_mode = ES_CMD_COMP_POLL;
			}
#endif
			break;
		case ES_SET_POWER_STATE_VS_OVERLAY:
			/* Either switch to VS low power or Normal mode */
			if (next_power_state == ES_SET_POWER_STATE_VS_LOWPWR) {
				/* Start internal oscillator */
				rc = esn100_start_int_osc();
				if (rc)
					goto power_transition_exit;

				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_LOWPWR;

			} else {
				escore->escore_power_state =
					ES_SET_POWER_STATE_NORMAL;
				escore->mode = STANDARD;
			}

			rc = escore_write(escore->codec, set_power_state_cmd,
					escore->escore_power_state);
			if (rc) {
				pr_err("%s(): Power state cmd write fail %d\n",
				       __func__, rc);
				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_OVERLAY;
				goto power_transition_exit;
			}

#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
			/* give some time for NS to settle */
			msleep(50);
#endif

			if (escore->escore_power_state ==
					ES_SET_POWER_STATE_VS_LOWPWR) {
				/* Disable the clocks */
				if (escore->pdata->esxxx_clk_cb)
					escore->pdata->esxxx_clk_cb(0);
				if (escore->pdata->gpioa_gpio != -1)
					escore->cmd_compl_mode =
							ES_CMD_COMP_POLL;
			}
			break;
		case ES_SET_POWER_STATE_VS_LOWPWR:
			/* Wakeup Chip */
			rc = esn100_wakeup(&escore_priv);
			if (rc) {
				dev_err(escore_priv.dev,
						"%s(): esn100 wakeup fail %d\n",
						__func__, rc);
				goto power_transition_exit;
			}
			escore_priv.escore_power_state =
				ES_SET_POWER_STATE_VS_OVERLAY;
			break;
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
		case ES_POWER_STATE_VS_STREAMING:
			/* send the set power state to overlay  */
			rc = escore_cmd(&escore_priv,
					((ES_SET_POWER_STATE << 16) |
					ES_SET_POWER_STATE_VS_OVERLAY), &resp);
			if (rc)
				dev_err(escore_priv.dev, "%s(): Power state change to Overlay fail, rc = %d",
					__func__, rc);

			rc = escore_cmd(&escore_priv, es_get_power_state,
					&resp);
			if (rc < 0)
				dev_err(escore_priv.dev, "Failed to get power level, rc = %d",
					rc);

			if (resp != (es_get_power_state |
					ES_SET_POWER_STATE_VS_OVERLAY)) {
				dev_err(escore_priv.dev, "Failed to change the power state to overlay");
				goto power_transition_exit;
			}

			escore_priv.escore_power_state =
				ES_SET_POWER_STATE_VS_OVERLAY;
			break;
#endif
		default:
			pr_err("%s(): Unsupported state in esn100\n", __func__);
			rc = -EINVAL;
			goto power_transition_exit;
		}
	}
	dev_dbg(escore_priv.dev, "%s(): Power state change successful\n",
		__func__);
power_transition_exit:
	return rc;
}

static int esn100_put_power_control_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;
	int rc = 0;
	struct escore_priv *escore = &escore_priv;

	value = ucontrol->value.enumerated.item[0];

	mutex_lock(&escore->access_lock);
	rc = escore_pm_get_sync();
	if (rc < 0) {
		pr_err("%s(): pm_get_sync failed :%d\n", __func__, rc);
		goto exit;
	}

	dev_dbg(escore_priv.dev, "%s(): Current state:%s, Requested state:%s\n",
		__func__,
		esn100_vs_power_state_texts[escore_priv.escore_power_state],
		esn100_vs_power_state_texts[value]);

	if (value == 0 || value == ES_SET_POWER_STATE_MP_SLEEP ||
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
		value == ES_POWER_STATE_VS_STREAMING ||
#endif
		value == ES_SET_POWER_STATE_MP_CMD) {
		dev_err(escore_priv.dev, "%s(): Unsupported state in esn100\n",
			__func__);
		goto exit;
	}

	rc = esn100_power_transition(value, reg);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): esn100_power_transition() failed %d\n",
			__func__, rc);
	}

	escore_pm_put_autosuspend();

exit:
	mutex_unlock(&escore->access_lock);
	return rc;
}

static int esn100_get_power_level_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	u32 es_get_power_level = ESN100_GET_POWER_LEVEL << 16;
	u32 rspn = 0;
	int rc;

	rc = escore_cmd_locked(&escore_priv, es_get_power_level, &rspn);
	if (rc < 0) {
		dev_err(escore_priv.dev, "codec reg read err %d()", rc);
		return rc;
	}

	ucontrol->value.enumerated.item[0] = rspn & 0x0000ffff;
	dev_dbg(escore_priv.dev, "%s: Response 0x%08X", __func__, rspn);
	return 0;
}

static int esn100_put_power_level_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static void esn100_switch_route(long route_index)
{
	struct escore_priv *esn100 = &escore_priv;
	int rc;

	if (route_index >= ROUTE_MAX) {
		dev_dbg(esn100->dev, "%s(): new esn100_internal_route = %ld is out of range\n",
			 __func__, route_index);
		return;
	}

	dev_dbg(esn100->dev, "%s(): switch current esn100_internal_route = %ld to new route = %ld\n",
		__func__, esn100->internal_route_num, route_index);
	esn100->internal_route_num = route_index;
	rc = escore_write_block(esn100,
			  esn100_route_config[esn100->internal_route_num].route);
}

static int esn100_put_internal_route(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	esn100_switch_route(ucontrol->value.integer.value[0]);
	return 0;
}

static int esn100_get_internal_route(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *esn100 = &escore_priv;

	ucontrol->value.integer.value[0] = esn100->internal_route_num;

	return 0;
}

static int esn100_put_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;
	struct escore_priv *escore = &escore_priv;

	value = ucontrol->value.integer.value[0];

	rc = escore_write_locked(escore->codec, reg, value);
	if (rc) {
		dev_err(escore->dev, "%s(): Set Preset fail %d\n",
			__func__, rc);
		return rc;
	}

	escore_priv.preset = value;

	return rc;
}

static int esn100_get_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.preset;

	return 0;
}

static int esn100_put_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.enumerated.item[0];
	rc = escore_write_locked(escore->codec, reg, value);

	return 0;
}

static int esn100_get_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;

	value = escore_read_locked(escore->codec, reg);

	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int esn100_get_event_status(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&escore_priv.escore_event_type_mutex);

	ucontrol->value.enumerated.item[0] = escore_priv.escore_event_type;

	/* Reset the event status after read */
	escore_priv.escore_event_type = ES_NO_EVENT;

	mutex_unlock(&escore_priv.escore_event_type_mutex);

	return 0;
}

static int esn100_get_rdb_size(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] =
				escore_priv.datablock_dev.rdb_read_count;
	return 0;
}

/*** KControl definitions ***/
static struct snd_kcontrol_new esn100_digital_ext_snd_controls[] = {
	SOC_ENUM_EXT("ESN100 Power State", esn100_vs_power_state_enum,
		     esn100_get_power_control_enum,
		     esn100_put_power_control_enum),
	SOC_ENUM_EXT("Power Level", esn100_power_level_enum,
				esn100_get_power_level_value,
				esn100_put_power_level_value),
	SOC_SINGLE_EXT("Internal Route",
		       SND_SOC_NOPM, 0, 100, 0, esn100_get_internal_route,
		       esn100_put_internal_route),
	SOC_SINGLE_EXT("Preset",
		       ESN100_PRESET, 0, 65535, 0, esn100_get_preset_value,
		       esn100_put_preset_value),
	SOC_ENUM_EXT("Voice Sense Status",
		     esn100_vs_event_enum,
		     esn100_get_control_enum, NULL),
	SOC_ENUM_EXT("Voice Sense Training Mode",
			 esn100_vs_training_mode_enum,
			 escore_vs_get_control_enum,
			 escore_vs_put_control_enum),
	SOC_ENUM_EXT("Voice Sense Training Status",
		     esn100_vs_training_status_enum,
		     esn100_get_control_enum, NULL),
	SOC_SINGLE_EXT("Voice Sense Training Model Length",
			ESN100_VOICE_SENSE_TRAINING_MODEL_LENGTH, 0, 75, 0,
			escore_vs_get_control_value,
			NULL),
	SOC_ENUM_EXT("Voice Sense Training Record",
		     esn100_vs_training_record_enum,
		     NULL, esn100_put_control_enum),
	SOC_SINGLE_EXT("Voice Sense Detect Sensitivity",
			ESN100_VOICE_SENSE_DETECTION_SENSITIVITY, 0, 10, 0,
			escore_vs_get_control_value,
			escore_vs_put_control_value),
	SOC_SINGLE_EXT("Voice Activity Detect Sensitivity",
			ESN100_VOICE_ACTIVITY_DETECTION_SENSITIVITY, 0, 10, 0,
			escore_vs_get_control_value,
			escore_vs_put_control_value),
	SOC_SINGLE_EXT("Continuous Voice Sense Preset",
		       ESN100_CVS_PRESET, 0, 65535, 0,
		       escore_get_cvs_preset_value,
		       escore_put_cvs_preset_value),
	SOC_SINGLE_EXT("Get RDB data size",
			   SND_SOC_NOPM, 0, 65535, 0,
			   esn100_get_rdb_size, NULL),
	SOC_SINGLE_EXT("ESN100 Get Event Status",
			   SND_SOC_NOPM, 0, 65535, 0,
			   esn100_get_event_status, NULL),
	SOC_SINGLE_EXT("CVQ Activate Keywords",
			SND_SOC_NOPM, 0, 31, 0,
			escore_get_vs_activate_keyword,
			escore_put_vs_activate_keyword),
	SOC_SINGLE_EXT("CVQ Sleep",
		       SND_SOC_NOPM, 0, 1, 0,
		       escore_get_vs_sleep,
		       escore_put_vs_sleep),
	SOC_ENUM_EXT("Streaming Mode", esn100_streaming_mode_enum,
			   escore_get_streaming_mode,
			   escore_put_streaming_mode),
	SOC_SINGLE_EXT("VS keyword length",
		       SND_SOC_NOPM, 0, 65535, 0,
		       escore_get_vs_keyword_length,
		       escore_put_vs_keyword_length),
	SOC_SINGLE_EXT("KW Overrun Error",
		       0, 0, 65535, 0, escore_get_keyword_overrun,
		       NULL),
};

int remote_add_codec_control(void *ptr)
{
	int rc;
	struct snd_soc_codec *codec = ptr;

	struct escore_priv *escore = &escore_priv;
	escore->codec = codec;
/*
	wait_for_completion(&escore->fw_download);
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	rc = snd_soc_add_codec_controls(codec, esn100_digital_ext_snd_controls,
				ARRAY_SIZE(esn100_digital_ext_snd_controls));
#else
	rc = snd_soc_add_controls(codec, esn100_digital_ext_snd_controls,
				ARRAY_SIZE(esn100_digital_ext_snd_controls));
#endif
	if (rc)
		dev_err(codec->dev,
			"%s(): esn100_digital_ext_snd_controls fail %d\n",
			__func__, rc);
	return rc;
}

int esn100_remote_add_codec_controls(struct snd_soc_codec *codec)
{
	int rc = 0;
	static struct task_struct *add_kcontrol_thread;

	pr_debug("%s:%d Initializing Kcontrols\n", __func__, __LINE__);
	add_kcontrol_thread = kthread_run(remote_add_codec_control,
					(void *) codec,
					"codec control thread");
	if (IS_ERR_OR_NULL(add_kcontrol_thread)) {
		pr_err("%s(): can't create thread to add codec control = %p\n",
			__func__, add_kcontrol_thread);
		rc = -ENOMEM;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(esn100_remote_add_codec_controls);

int esn100_init(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense;
	if(!escore)
		return 0;

	escore->dai = esn100_dai;
	escore_priv.dai_nr = ES_NUM_CODEC_DAIS;
	escore_priv.api_addr_max = ES_API_ADDR_MAX;
	escore_priv.api_access = esn100_api_access;
	escore_priv.es_cvs_preset = ESN100_PREST_SINGLE_DOWNLOAD,

	escore_vs_init(&escore_priv);

	voice_sense = (struct escore_voice_sense *)escore_priv.voice_sense;
	voice_sense->vs_irq = false;
	escore_priv.fw_requested = 0;

	return 1;
}
EXPORT_SYMBOL_GPL(esn100_init);
