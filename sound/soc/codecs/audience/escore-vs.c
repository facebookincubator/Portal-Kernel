/*
 * escore-vs.c  --  Audience Voice Sense component ALSA Audio driver
 *
 * Copyright 2013 Audience, Inc.
 *
 * Author: Greg Clemson <gclemson@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "escore.h"
#include "escore-vs.h"
#include "es705_escore.h"

#define VS_KCONTROL(method, type) \
	int escore_vs_##method##_control_##type(struct snd_kcontrol *kcontrol,\
			struct snd_ctl_elem_value *ucontrol)		\
{									\
	struct escore_priv *escore = &escore_priv;			\
	if (escore->mode != VOICESENSE) {				\
		dev_warn(escore->dev, "%s(): Not in VS mode\n",		\
				__func__);				\
		return 0;						\
	}								\
	return escore_##method##_control_##type(kcontrol, ucontrol);	\
}

VS_KCONTROL(get, value)
VS_KCONTROL(put, value)
VS_KCONTROL(get, enum)
VS_KCONTROL(put, enum)

static void escore_vs_init_intr(struct escore_priv *escore);
static int escore_vs_detect_cmd(struct escore_priv *escore);

static int escore_vs_sleep(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	u32 cmd, rsp;
	int rc;
#ifdef CONFIG_SND_SOC_ES_CVQ_TIME_MEASUREMENT
	struct timespec cvq_sleep_start;
	struct timespec cvq_sleep_end;
	struct timespec cvq_sleep_time;
	struct timespec wdb_start;
	struct timespec wdb_end;
	struct timespec wdb_time;
#endif

	dev_dbg(escore->dev, "%s()\n", __func__);
	if (voice_sense->vs_irq != true)
		escore_vs_init_intr(escore);

	es_cvq_profiling(&cvq_sleep_start);

	cmd = (ES_SET_EVENT_RESP << 16) | \
	      escore->pdata->gpio_b_irq_type;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc < 0) {
		pr_err("%s(): Error %d in setting event response\n",
				__func__, rc);
		goto vs_sleep_err;
	}
	if (voice_sense->es_vs_keyword_length) {
		cmd = ((ES_SET_VS_KW_LENGTH << 16) |
				voice_sense->es_vs_keyword_length);
		rc = escore_cmd_nopm(escore, cmd, &rsp);
		if (rc) {
			dev_err(escore->dev, "%s(): kw length cmd fail %d\n",
					__func__, rc);
			goto vs_sleep_err;
		}
	}

	es_cvq_profiling(&wdb_start);

	/* Reconfig API Interrupt mode */
	rc = escore_reconfig_api_intr(escore);
	if (rc)
		goto vs_sleep_err;

	/* write background model and keywords files */
	rc = escore_vs_write_bkg_and_keywords(escore);
	if (rc) {
		dev_err(escore->dev,
			"%s(): datablock write fail rc = %d\n",
			__func__, rc);
		goto vs_sleep_err;
	}

	es_cvq_profiling(&wdb_end);

	usleep_range(20000, 20050);

	rc = escore_vs_detect_cmd(escore);
	if (rc)
		goto vs_sleep_err;

	es_cvq_profiling(&cvq_sleep_end);

#ifdef CONFIG_SND_SOC_ES_CVQ_TIME_MEASUREMENT
	wdb_time = (timespec_sub(wdb_end, wdb_start));
	dev_info(escore->dev, "BKG and KW write time = %lu.%03lu sec\n",
			wdb_time.tv_sec, (wdb_time.tv_nsec)/1000000);
	cvq_sleep_time = (timespec_sub(cvq_sleep_end, cvq_sleep_start));
	dev_info(escore->dev, "Total CVQ sleep time = %lu.%03lu sec\n",
		cvq_sleep_time.tv_sec, (cvq_sleep_time.tv_nsec)/1000000);
#endif

vs_sleep_err:
	return rc;
}

static int escore_vs_detect_cmd(struct escore_priv *escore)
{
	u32 cmd, rsp;
	int rc;

	cmd = ES_SET_ALGO_PARAM_ID << 16 | ES_VS_PROCESSING_MOE;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): Set Algo Param ID cmd fail %d\n",
			__func__, rc);
		return rc;
	}

	cmd = ES_SET_ALGO_PARAM << 16 | ES_VS_DETECT_KEYWORD;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): Set Algo Param cmd fail %d\n",
			__func__, rc);
		return rc;
	}
	return rc;
}

int escore_vs_wakeup(struct escore_priv *escore)
{
	int rc = 0;
#if (defined CONFIG_SND_SOC_ES817_PREMIUM) || \
	(defined CONFIG_SND_SOC_ES817_PLUS)
	u32 cmd, rsp, sync_rc;
	es_profiling_var(switching_time);
#endif
	dev_dbg(escore->dev, "%s()\n", __func__);

	if (escore->escore_power_state == ES_SET_POWER_STATE_NORMAL) {
		dev_dbg(escore->dev, "%s() Already in normal mode\n", __func__);
		goto out;
	}

#if (defined CONFIG_SND_SOC_ES817_PREMIUM) || \
	(defined CONFIG_SND_SOC_ES817_PLUS)
	es_profiling_start(&switching_time);
	rc = escore_wakeup(escore);
	if (rc) {
		dev_err(escore->dev, "%s() wakeup failed rc = %d\n",
				__func__, rc);
		goto vs_wakeup_err;
	}
	escore->escore_power_state = ES_SET_POWER_STATE_VS_OVERLAY;
#endif
	if (escore->intr_recvd) {
		/* Total 60 ms Delay is required for firmware to be ready
		 * after wakeup */
		msleep(60 - escore->delay.wakeup_to_vs);
		dev_dbg(escore->dev,
			"%s() Keeping chip in CVQ command mode\n", __func__);
		goto out;
	}
#if (defined CONFIG_SND_SOC_ES817_PREMIUM) || \
	(defined CONFIG_SND_SOC_ES817_PLUS)
	/* Set flag to Wait for API Interrupt */
	escore_set_api_intr_wait(escore);

	/* change power state to Normal*/
	cmd = (ES_SET_POWER_STATE << 16) | ES_SET_POWER_STATE_NORMAL;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc < 0) {
		dev_err(escore->dev, "%s() - failed sync cmd resume %d\n",
			__func__, rc);
		goto vs_wakeup_err;
	}
	/* command response sent immediate after power state change fails
	 * Added sync command and avoiding its response */
#ifdef CONFIG_SND_SOC_ES_SPI
	INC_DISABLE_FW_RECOVERY_USE_CNT(escore);
	cmd = ES_SYNC_CMD << 16;
	sync_rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (sync_rc < 0) {
		dev_dbg(escore->dev,
				"%s(): 0x%08x failed sync cmd %d\n",
				__func__, rsp, sync_rc);
	}
	DEC_DISABLE_FW_RECOVERY_USE_CNT(escore);
#endif

	if (escore->pdata->gpioa_gpio != -1) {
		/* Wait for API Interrupt to confirm
		 * that device is ready to accept commands */
		rc = escore_api_intr_wait_completion(escore);
		if (rc)
			goto vs_wakeup_err;

		/* Reconfig API Interrupt mode */
		rc = escore_reconfig_api_intr(escore);
		if (rc)
			goto vs_wakeup_err;
	} else
		msleep(escore->delay.vs_to_normal);
	es_profiling_end(&switching_time);
	print_profiling_time(switching_time, "VS to NS switch time");
#else
	rc = escore->boot_ops.bootup(escore);
	if (rc < 0) {
		dev_err(escore->dev, "%s() - VS-NS switching failed %d\n",
			__func__, rc);
		goto vs_wakeup_err;
	}
#endif

	escore->mode = STANDARD;
	escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;

out:
vs_wakeup_err:
	return rc;
}

static int escore_cvq_sleep_thread(void *ptr)
{
	struct escore_priv *escore = (struct escore_priv *)ptr;
	int rc;
#if (defined CONFIG_SND_SOC_ES817_PREMIUM) || \
	(defined CONFIG_SND_SOC_ES817_PLUS)
	u32 cmd, rsp, sync_rc;
#endif

	rc = escore_vs_sleep(&escore_priv);
	if (rc != -EABORT)
		goto escore_cvq_sleep_thread_exit;

	dev_dbg(escore->dev, "%s() CVQ sleep aborted\n", __func__);

	/* Set flag to Wait for API Interrupt */
	escore_set_api_intr_wait(escore);

#if (defined CONFIG_SND_SOC_ES817_PREMIUM) || \
	(defined CONFIG_SND_SOC_ES817_PLUS)
	/* Change power state to Normal */
	if (escore->mode != VOICESENSE_PENDING) {
		if (escore->mode == SBL)
			/* BAS-3232: Change power state to Normal, send
			 * command with SR bit disable on PRI interface,
			 * but device won't send any response back */
			cmd = ES_SET_POWER_STATE_CMD << 16 |
					ES_SET_POWER_STATE_NORMAL;
		else
			/* BAS-3232: Change power state to Normal, send
			 * command with SR bit enable on PRI interface */
			cmd = ES_SET_POWER_STATE << 16 |
					ES_SET_POWER_STATE_NORMAL;
		/* handle endianness */
		cmd = escore->bus.ops.cpu_to_bus(escore, cmd);
		rc = escore->bus.ops.write(escore, &cmd, sizeof(cmd));
		update_cmd_history(escore->bus.ops.bus_to_cpu(escore,
							cmd), 0);
	/* command response sent immediate after power state change fails
	 * Added sync command and avoiding its response */
#ifdef CONFIG_SND_SOC_ES_SPI
	INC_DISABLE_FW_RECOVERY_USE_CNT(escore);
	cmd = ES_SYNC_CMD << 16;
	sync_rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (sync_rc < 0) {
		dev_dbg(escore->dev,
				"%s(): 0x%08x failed sync cmd %d\n",
				__func__, rsp, sync_rc);
	}
	DEC_DISABLE_FW_RECOVERY_USE_CNT(escore);
#endif
		if (rc) {
			pr_err("%s() PRI INTF write is failed, rc = %d\n",
					__func__, rc);
			goto escore_cvq_sleep_thread_exit;
		}
	}

	if (escore->pdata->gpioa_gpio == -1) {
		if (escore->mode == VOICESENSE)
			/* BAS-3232: Delay required before reading power
			 * state in case of VOICESENSE mode:
			 * es75x: 90 ms
			 * es70x / es80x: 50 ms */
			msleep(escore->delay.vs_to_normal);
		else
			/* BAS-3232: Total delay required before reading power
			 * state is 60 ms in case of SBL and VOICESENSE_PENDING
			 * mode */
			msleep(60);

		cmd = ES_GET_POWER_STATE << 16;
		rc = escore_cmd_nopm(escore, cmd, &rsp);
		if (rc < 0) {
			pr_err("%s() Get Power State failed rc = %d\n",
							__func__, rc);
			goto escore_cvq_sleep_thread_exit;
		}
		if (rsp != ES_PS_NORMAL) {
			pr_err("%s() Power state is not normal, rsp = %x\n",
					__func__, rsp);
			rc = -EINVAL;
			goto escore_cvq_sleep_thread_exit;
		}
	} else {
		/* Wait for API Interrupt to confirm that
		 * device is in normal mode */
		rc = escore_api_intr_wait_completion(escore);
		if (rc)
			goto escore_cvq_sleep_thread_exit;
	}
	escore_priv.escore_power_state = ES_SET_POWER_STATE_NORMAL;
#else
	/* Change power state to Normal.
	 * For Bellamy case we simply need to download NS firmwarw
	 */

	rc = escore->boot_ops.bootup(escore);
	if (rc < 0)
		pr_err("%s() Get Power State failed rc = %d\n",
				__func__, rc);
#endif
escore_cvq_sleep_thread_exit:
	return rc;
}

static int escore_voicesense_sleep(struct escore_priv *escore)
{

	return escore_cvq_sleep_thread((void *)escore);
}

int escore_put_vs_sleep(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];

	mutex_lock(&escore->access_lock);
	if (value) {

		rc = escore_pm_get_sync();
		if (rc < 0) {
			pr_err("%s(): pm_get_sync failed :%d\n", __func__, rc);
			mutex_unlock(&escore->access_lock);
			return rc;
		}

		rc = escore_cvq_sleep_thread(escore);
		escore_pm_put_autosuspend();

	} else
		rc = escore_vs_wakeup(&escore_priv);

	mutex_unlock(&escore->access_lock);
	return rc;
}

int escore_get_vs_sleep(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int escore_put_cvs_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];

	rc = escore_put_control_value(kcontrol, ucontrol);

	if (!rc)
		voice_sense->cvs_preset = value;

	return rc;
}

int escore_get_cvs_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	ucontrol->value.integer.value[0] = voice_sense->cvs_preset;

	return 0;
}

int escore_vs_sleep_enable(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;
	return voice_sense->vs_active_keywords;
}

int escore_put_vs_keyword_length(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	unsigned int value;
	int rc = 0;
	u32 cmd, rsp;

	value = ucontrol->value.integer.value[0];

	voice_sense->es_vs_keyword_length = value;

	if (!escore_vs_sleep_enable(&escore_priv)) {
		cmd = ((ES_SET_VS_KW_LENGTH << 16) |
				voice_sense->es_vs_keyword_length);
		rc = escore_cmd_locked(&escore_priv, cmd, &rsp);
		if (rc) {
			dev_err(escore_priv.dev, "%s(): kw length cmd fail %d\n",
					__func__, rc);
		}
	}

	return rc;
}

int escore_get_vs_keyword_length(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	ucontrol->value.integer.value[0] = voice_sense->es_vs_keyword_length;

	return 0;
}

int escore_get_keyword_overrun(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	u32 es_get_keyword_overrun = ESCORE_GET_KEYWORD_OVERRUN_ERROR << 16;
	u32 rspn = 0;

	rc = escore_cmd_locked(&escore_priv, es_get_keyword_overrun,
				&rspn);
	if (rc < 0) {
		dev_err(escore_priv.dev, "Failed to set the keyword length %d()",
				rc);
		return rc;
	}

	ucontrol->value.enumerated.item[0] = rspn & 0x0000ffff;
	dev_dbg(escore_priv.dev, "%s: Response 0x%08X", __func__,
				rspn);

	return 0;
}

int escore_put_vs_activate_keyword(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	unsigned int value;
	int rc = 0;
	u32 cmd, rsp;

	value = ucontrol->value.integer.value[0];

	mutex_lock(&escore->access_lock);

	/* send algo init command */
	cmd = ES_SET_ALGO_PARAM_ID << 16 | ES705_ALGO_INIT_ID;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): CVQ Algo init fail %d\n",
				__func__, rc);
		goto exit;
	}
	cmd = ES_SET_ALGO_PARAM << 16 | ES705_ALGO_RESET;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): CVQ Algo init fail %d\n",
				__func__, rc);
		goto exit;
	}

	rc = escore_vs_request_keywords(escore, value);
	if (rc)
		goto exit;
	rc = escore_vs_sleep(&escore_priv);
	if (rc)
		goto exit;
exit:
	mutex_unlock(&escore->access_lock);
	return rc;
}

int escore_get_vs_activate_keyword(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore_priv.voice_sense;
	ucontrol->value.integer.value[0] = voice_sense->vs_active_keywords;

	return 0;
}

int escore_put_cvq_start(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	u32 cmd, rsp;
	int rc = 0;
	/* send reset command */
#ifdef CONFIG_SND_SOC_ES817_BASE
	cmd = ES705_KW_DETECTION_STATE << 16 | ES705_KW_DETECTION_START;
#else
	cmd = ES705_KW_DETECTION_STATE << 16 | ES705_CVQ_RESET_VALUE;
#endif
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): CVQ reset fail %d\n",
				__func__, rc);
		goto exit;
	}

	if (escore->bargein_state == PB)
		escore->bargein_state = CVQ_PB;
	else
		escore->bargein_state = CVQ;
exit:
	return rc;
}

int escore_get_cvq_start(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int escore_put_cvq_stop(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	u32 cmd, rsp;
	int rc = 0;
	/* send CVQ stop command here */
	cmd = ES705_KW_DETECTION_STATE << 16 | ES705_KW_DETECTION_STOP;
	rc = escore_cmd_nopm(escore, cmd, &rsp);
	if (rc) {
		dev_err(escore->dev, "%s(): KW detection stop fail %d\n",
				__func__, rc);
		return rc;
	}
	if (escore->bargein_state == CVQ_PB)
		escore->bargein_state = PB;
	else
		escore->bargein_state = Enable;
	return rc;
}

int escore_get_cvq_stop(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static ssize_t escore_vs_status_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int ret = 0;
	unsigned int value = 0;
	char *status_name = "Voice Sense Status";
	/* Disable vs status read for interrupt to work */
	struct escore_priv *escore = &escore_priv;
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;

	mutex_lock(&voice_sense->vs_event_mutex);

	value = voice_sense->vs_get_event;
	/* Reset the detection status after read */
	voice_sense->vs_get_event = ES_NO_EVENT;

	mutex_unlock(&voice_sense->vs_event_mutex);

	ret = snprintf(buf, PAGE_SIZE, "%s=0x%04x\n", status_name, value);

	return ret;
}

static DEVICE_ATTR(vs_status, 0440, escore_vs_status_show, NULL);

static struct attribute *vscore_sysfs_attrs[] = {
	&dev_attr_vs_status.attr,
	NULL
};

static struct attribute_group vscore_sysfs = {
	.attrs = vscore_sysfs_attrs
};

int escore_vs_request_firmware(struct escore_priv *escore,
				const char *vs_filename)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;

	return request_firmware((const struct firmware **)&voice_sense->vs,
			      vs_filename, escore->dev);
}

void escore_vs_release_firmware(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;

	release_firmware(voice_sense->vs);
}

int escore_vs_request_bkg(struct escore_priv *escore, const char *bkg_filename)
{
	struct escore_voice_sense *voice_sense =
		(struct escore_voice_sense *) escore->voice_sense;
	int rc = 0;

	rc = request_firmware((const struct firmware **)&voice_sense->bkg,
			bkg_filename, escore->dev);
	if (rc) {
		dev_err(escore->dev, "%s(): request_firmware(%s) failed %d\n",
				__func__, bkg_filename, rc);
		return  rc;
	}
	return  rc;
}

void escore_vs_release_bkg(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;

	release_firmware(voice_sense->bkg);
}

int escore_vs_request_keywords(struct escore_priv *escore, unsigned int value)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;
	int rc = 0;
	int i;
#ifdef CONFIG_SND_SOC_ES817_BASE
	char kw_filename[] = "audience-vs-kw-10.bin";
#else
	char kw_filename[] = "audience-vs-kw-1.bin";
#endif
	int size = sizeof(kw_filename)/sizeof(char);

	/* Keyword number is based on bit positions. For an example, for keyword
	 * number 3, value is 100B i.e. 4 in decimal. Similarly for keywords 4
	 * and 5, 4th and 5th bit is set i.e. 11000 i.e. 18 (bit position is
	 * started from 1 here).
	 *
	 * Flow of function:
	 *	- Release already requested keyword
	 *	- Request new keyword if it is selected
	 *	- In case of error, release all keywords
	 */

	for (i = 0; i < MAX_NO_OF_VS_KW; i++) {

		/* Release keyword if it is requested before. This is required
		 * for kernel 3.10 in which if old keyword is not released,
		 * keyword data remains unchanged.
		 */
		if (voice_sense->vs_active_keywords & BIT(i)) {
			dev_dbg(escore->dev, "%s(): release kw = %d\n",
							__func__, i + 1);
			release_firmware(voice_sense->kw[i]);
			voice_sense->vs_active_keywords &= (~BIT(i));
		}

		/* If keyword is not set, do not request it. */
		if (!(value & BIT(i)))
			continue;

		/* Request selected keywords */
		snprintf(kw_filename, size, "audience-vs-kw-%d.bin", i + 1);
		dev_dbg(escore->dev, "%s(): kw filename = %s\n",
				__func__, kw_filename);
		rc = request_firmware(
				(const struct firmware **)&(voice_sense->kw[i]),
				kw_filename, escore->dev);
		if (rc) {
			dev_err(escore->dev, "%s(): request kw(%d) failed %d\n",
					__func__, i + 1, rc);
			goto request_firmware_kw_exit;
		}
		voice_sense->vs_active_keywords |= BIT(i);
	}

	return rc;

request_firmware_kw_exit:

	/* In case of failure, release all keywords */
	for (i = 0; i < MAX_NO_OF_VS_KW; i++) {
		if (voice_sense->vs_active_keywords & BIT(i)) {
			dev_dbg(escore->dev, "%s(): release kw %d\n",
							__func__, i + 1);
			release_firmware(voice_sense->kw[i]);
		}
	}
	voice_sense->vs_active_keywords = 0;

	return  rc;
}

int escore_vs_write_bkg_and_keywords(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;
	int rc;
	int i;

	rc = escore_datablock_open(escore);
	if (rc) {
		dev_err(escore->dev, "%s(): can't open datablock device = %d\n",
			__func__, rc);
		goto escore_vs_write_bkg_keywords_exit;
	}

	for (i = 0; i < MAX_NO_OF_VS_KW; i++) {
		if (!(voice_sense->vs_active_keywords & (1 << i)))
			continue;
		dev_dbg(escore->dev, "%s(): Write kw = %d\n", __func__, i + 1);
		rc = escore_datablock_write(escore, voice_sense->kw[i]->data,
						voice_sense->kw[i]->size);
		if ((rc < 0) || (rc < voice_sense->kw[i]->size)) {
			dev_err(escore->dev,
				"%s(): kw %d write failed rc = %d\n",
				__func__, i + 1, rc);
			goto escore_vs_write_bkg_keywords_exit;
		}
	}

	escore_datablock_close(escore);

	return 0;

escore_vs_write_bkg_keywords_exit:
	escore_datablock_close(escore);
	return rc;
}

static int escore_vs_isr(struct notifier_block *self, unsigned long action,
		void *dev)
{
	struct escore_priv *escore = (struct escore_priv *)dev;
	char *event[] = { "ACTION=ADNC_KW_DETECT", NULL };
	struct escore_voice_sense *voice_sense =
		(struct escore_voice_sense *) escore->voice_sense;
#ifndef CONFIG_SND_SOC_ES_VS_STREAMING
	u32 es_set_power_level = ES_SET_POWER_LEVEL << 16 | ES_POWER_LEVEL_6;
	u32 resp;
#endif
	int rc = 0, cmd, rsp;
#ifdef CONFIG_SND_SOC_ES817_PCR_68_EXTENDED
	u32 KW_ID, event_rsp, retry = 30;
#endif

	dev_info(escore->dev, "%s(): Event: 0x%04x\n", __func__, (u32)action);

	if (!(action & ES_VS_INTR_EVENT)) {
		dev_dbg(escore->dev, "%s(): Invalid event callback 0x%04x\n",
				__func__, (u32) action);
		return NOTIFY_DONE;
	}
#ifdef CONFIG_SND_SOC_ES817_PCR_68_EXTENDED
	mutex_lock(&escore->access_lock);
	while (retry--) {
		cmd = ES_GET_EVENT << 16;
		rc = escore_cmd_nopm(escore, cmd, &event_rsp);
		event_rsp &= ES_MASK_INTR_EVENT;
		if ((event_rsp & ES_MASK_KW_END_INTR_EVENT) == ES_MASK_KW_END_INTR_EVENT)
			break;
		msleep(30);
	}
	mutex_unlock(&escore->access_lock);
	KW_ID = event_rsp & ES_MASK_KW_ID_INTR_EVENT;
	dev_info(escore->dev, "KW_ID is 0x%04x\n", KW_ID);
#ifdef CONFIG_SND_SOC_ES817_BASE
	if ((KW_ID == ES_NO_EVENT) || (KW_ID == ES_KW_ID_ONE_INTR_EVENT)) {
#else
	if (KW_ID == ES_NO_EVENT) {
#endif
		dev_dbg(escore->dev, "%s(): Invalid Keyword ID 0x%04x\n",
				__func__, KW_ID);
		return NOTIFY_DONE;
	}
	mutex_lock(&escore->escore_event_type_mutex);
	escore->escore_event_type = event_rsp;
	mutex_unlock(&escore->escore_event_type_mutex);
	dev_info(escore->dev, "%s(): VS event detected 0x%04x\n",
				__func__, event_rsp);
#else
	dev_info(escore->dev, "%s(): VS event detected 0x%04x\n",
				__func__, (u32) action);
#endif
	if (voice_sense->cvs_preset != 0xFFFF && voice_sense->cvs_preset != 0) {
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
		/* Chip wakes up in VS Streaming mode */
		escore->escore_power_state = ES_POWER_STATE_VS_STREAMING;
#else
		escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;
#endif
		escore->mode = STANDARD;
	}

	mutex_lock(&voice_sense->vs_event_mutex);
#ifdef CONFIG_SND_SOC_ES817_PCR_68_EXTENDED
	voice_sense->vs_get_event = event_rsp;
#else
	voice_sense->vs_get_event = action;
#endif
	mutex_unlock(&voice_sense->vs_event_mutex);

	/* If CVS preset is set (other than 0xFFFF), earSmart chip is
	 * in CVS mode. To make it switch from internal to external
	 * oscillator, send power level command with highest power
	 * level
	 */
	if (voice_sense->cvs_preset != 0xFFFF &&
			voice_sense->cvs_preset != 0) {
#ifndef CONFIG_SND_SOC_ES_VS_STREAMING
		/* Following command will set the power level to 6,
		   any subsequent preset will switch the oscillator
		   to external */
		rc = escore_cmd_locked(escore, es_set_power_level, &resp);
		if (rc < 0) {
			pr_err("%s(): Error setting power level %d\n",
			       __func__, rc);
			goto voiceq_isr_exit;
		}
		usleep_range(2000, 2005);
#endif
		/* Enable the clock before switching to external oscillator */
		if (escore->pdata->esxxx_clk_cb)
			escore->pdata->esxxx_clk_cb(1);

		/* Each time earSmart chip comes in BOSKO mode after
		 * VS detect, CVS mode will be disabled */
		voice_sense->cvs_preset = 0;
	}
	kobject_uevent_env(&escore->dev->kobj, KOBJ_CHANGE, event);

	return NOTIFY_OK;

voiceq_isr_exit:
	return NOTIFY_DONE;
}

static struct notifier_block escore_vs_intr_cb = {
	.notifier_call = escore_vs_isr,
	.priority = ES_NORMAL,
};

void escore_vs_init_intr(struct escore_priv *escore)
{
	escore_register_notify(escore_priv.irq_notifier_list,
			&escore_vs_intr_cb);
	((struct escore_voice_sense *)escore->voice_sense)->vs_irq = true;
}

int escore_vs_load(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;
	int rc = 0;

	BUG_ON(voice_sense->vs->size == 0);
	escore->mode = VOICESENSE_PENDING;

	/* Reset Mode to Polling */
	escore->cmd_compl_mode = ES_CMD_COMP_POLL;
	INC_DISABLE_FW_RECOVERY_USE_CNT(escore);

	if (!escore->boot_ops.setup || !escore->boot_ops.finish) {
		dev_err(escore->dev,
			"%s(): boot setup or finish function undefined\n",
			__func__);
		rc = -EIO;
		goto escore_vs_uart_open_failed;
	}

	if (escore->bus.ops.high_bw_open) {
		rc = escore->bus.ops.high_bw_open(escore);
		if (rc) {
			dev_err(escore->dev, "%s(): high_bw_open failed %d\n",
				__func__, rc);
			goto escore_vs_uart_open_failed;
		}
	}

#if (!defined CONFIG_SND_SOC_ES817_PREMIUM) && \
	(!defined CONFIG_SND_SOC_ES817_PLUS)
	escore_gpio_reset(escore);
#endif
	rc = escore->boot_ops.setup(escore);
	if (rc) {
		dev_err(escore->dev, "%s(): fw download start error %d\n",
			__func__, rc);
		goto escore_vs_fw_download_failed;
	}
	rc = escore_is_sleep_aborted(escore);
	if (rc == -EABORT)
		goto escore_sleep_aborted;

	dev_dbg(escore->dev, "%s(): write vs firmware image\n", __func__);

	/* Set flag to Wait for API Interrupt */
	escore_set_api_intr_wait(escore);

	rc = escore->bus.ops.high_bw_write(escore,
		((char *)voice_sense->vs->data) , voice_sense->vs->size);
	if (rc) {
		dev_err(escore->dev, "%s(): vs firmware image write error %d\n",
			__func__, rc);
		rc = -EIO;
		goto escore_vs_fw_download_failed;
	}

	escore->mode = VOICESENSE;

	if (((struct escore_voice_sense *)escore->voice_sense)->vs_irq != true)
		escore_vs_init_intr(escore);

	/* Wait for API Interrupt to confirm
	 * that firmware is ready to accept command */
	if (escore->pdata->gpioa_gpio != -1) {
		rc = escore_api_intr_wait_completion(escore);
		if (rc)
			goto escore_vs_fw_download_failed;
	}
	rc = escore->boot_ops.finish(escore);
	if (rc) {
		dev_err(escore->dev,
			"%s() vs fw download finish error %d\n",
			__func__, rc);
		goto escore_vs_fw_download_failed;
	}

	rc = escore_is_sleep_aborted(escore);
	if (rc == -EABORT)
		goto escore_sleep_aborted;

	/* Reconfig API Interrupt mode */
	rc = escore_reconfig_api_intr(escore);
	if (rc)
		goto escore_vs_fw_download_failed;

	dev_dbg(escore->dev, "%s(): fw download done\n", __func__);

escore_sleep_aborted:
escore_vs_fw_download_failed:
	if (escore->bus.ops.high_bw_close) {
		int ret = 0;
		ret = escore->bus.ops.high_bw_close(escore);
		if (ret) {
			dev_err(escore->dev, "%s(): high_bw_close failed %d\n",
				__func__, ret);
			rc = ret;
		}
	}
escore_vs_uart_open_failed:
	DEC_DISABLE_FW_RECOVERY_USE_CNT(escore);
	return rc;
}

int escore_vs_init_sysfs(struct escore_priv *escore)
{
	return sysfs_create_group(&escore->dev->kobj, &vscore_sysfs);
}

void escore_vs_exit(struct escore_priv *escore)
{
	struct escore_voice_sense *voice_sense =
			(struct escore_voice_sense *) escore->voice_sense;
	int i;
	for (i = 0; i < MAX_NO_OF_VS_KW; i++) {
		if (voice_sense->vs_active_keywords & BIT(i)) {
			dev_dbg(escore->dev, "%s(): release kw = %d\n",
							__func__, i + 1);
			release_firmware(voice_sense->kw[i]);
		}
	}
	kfree(voice_sense);
	escore_vs_release_firmware(escore);
}

int escore_vs_init(struct escore_priv *escore)
{
	int rc = 0;
	struct escore_voice_sense *voice_sense;
	voice_sense = (struct escore_voice_sense *)
			kmalloc(sizeof(struct escore_voice_sense), GFP_KERNEL);
	if (!voice_sense) {
		rc = -ENOMEM;
		goto voice_sense_alloc_err;
	}

	escore->voice_sense = (void *)voice_sense;

	/* Initialize variables */
	voice_sense->cvs_preset = 0;
	voice_sense->vs_active_keywords = 0;
	voice_sense->es_vs_keyword_length = 0;

	mutex_init(&voice_sense->vs_event_mutex);

	rc = escore_vs_init_sysfs(escore);
	if (rc) {
		dev_err(escore_priv.dev,
			"failed to create core sysfs entries: %d\n", rc);
		goto sysfs_init_err;
	}

	escore->vs_ops.escore_is_voicesense_sleep_enable =
					escore_vs_sleep_enable;
	escore->vs_ops.escore_voicesense_sleep = escore_voicesense_sleep;
	escore->vs_ops.escore_voicesense_wakeup = escore_vs_wakeup;

	rc = escore_vs_request_firmware(escore, escore->vs_filename);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): request_firmware(%s) failed %d\n",
			__func__, escore->vs_filename, rc);
		goto request_vs_firmware_error;
	}

	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): request_firmware of bkg failed %d\n",
			__func__, rc);
		goto vs_request_bkg_err;
	}
	return rc;
vs_request_bkg_err:
	escore_vs_release_firmware(escore);
request_vs_firmware_error:
sysfs_init_err:
	kfree(voice_sense);
voice_sense_alloc_err:
	return rc;
}
