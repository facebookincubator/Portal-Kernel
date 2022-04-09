/*
 * dbmdx-snd-codec.c -- DSPG DBMDX codec driver
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/version.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <dt-bindings/sound/dbmdx/dbmd7_fw_api.h>
#include <trace/events/fb_audio.h>

#define TLV_HEADER_SIZE			8
#define ALGO_SIZE_MAX			1024
#define MAX_SUPPORTED_TDM0_CHANNELS	16
#define MAX_SUPPORTED_TDM1_CHANNELS	2
#define MAX_SUPPORTED_TDM2_CHANNELS	16

#define DBMDX_USAGE_NEW_UC		(0)
#define DBMDX_USAGE_SET			(+1)
#define DBMDX_USAGE_RELEASE		(-1)

#define DBMDX_UC_NONE			(0)
#define DBMDX_UC_FORCE			(-1)

#define DBMDX_FW_DEFAULT  (0)
#define DBMDX_FW_3P       (1)
#define DBMDX_FW_1p       (2)

struct dbmdx_codec_ctx {
	struct snd_soc_codec *codec;
	struct device *mfd_dev;
	struct dbmdx_private *mfd_priv;
	struct mutex lock;

	/* Flags */
	unsigned int flg_amp_playback:1;
	unsigned int flg_amp_capture:1;

	/* in use counter */
	int in_use;

	/* use case */
	int usecase;			/* Desired use-case */
	int usecase_last;		/* Last use-case */
	bool skip_usecase_setting;	/* Skip use-case settings */

	/* Notifier */
	struct notifier_block nb;
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
#define MY_SND_SOC_BYTES_TLV(xname, xcount, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READWRITE | \
		  SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK, \
	.tlv.c = (dbmdx_snd_soc_bytes_tlv_callback), \
	.info = snd_soc_bytes_info_ext, \
	.private_value = (unsigned long)&(struct soc_bytes_ext) \
		{.max = xcount, .get = xhandler_get, .put = xhandler_put, } }
#endif

static int dbmdx_get_switch_playback(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	struct snd_soc_dapm_widget_list *wl = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wl->widgets[0];
# else
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
#endif
	struct snd_soc_component *comp = widget->dapm->component;
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = sctx->flg_amp_playback;

	return 0;
}

static int dbmdx_put_switch_playback(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	struct snd_soc_dapm_widget_list *wl = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wl->widgets[0];
#else
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
#endif
	struct snd_soc_component *comp = widget->dapm->component;
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(comp);
	int connect;

	sctx->flg_amp_playback = !!ucontrol->value.integer.value[0];
	connect = sctx->flg_amp_playback;

	snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, connect, NULL);

	return 0;
}

static void dbmdx_fw_override_update(struct dbmdx_codec_ctx *sctx, int type)
{
	struct dbmdx_private *p = sctx->mfd_priv;
	struct dbmdx_platform_data *pdata = p->pdata;
	struct device *dev = sctx->codec->dev;
	int ret = 0;
	bool trigger_timeout = false;
	const char *temp = NULL;

	mutex_lock(&sctx->lock);
	if (!p->fw_alive) {
		dev_info(p->dev, "Firmware can't be updated while loading firmware");
		mutex_unlock(&sctx->lock);
		return;
	}
	temp = pdata->primary_firmware_name;
	switch (type) {
	case DBMDX_FW_3P:
		if (strcmp(pdata->primary_firmware_name,
			   pdata->primary_firmware_name_3p)) {
			pdata->primary_firmware_name =
			  pdata->primary_firmware_name_3p;
			trigger_timeout = true;
		}
		break;
	case DBMDX_FW_1p:
		if (strcmp(pdata->primary_firmware_name,
			   pdata->primary_firmware_name_1p)) {
			pdata->primary_firmware_name =
			  pdata->primary_firmware_name_1p;
			trigger_timeout = true;
		}
		break;
	default:
		break;
	}
	/*
	 * Trigger timeout as workaround. This will trigger dspg firmware crash
	 * recovery and loads dspg firmware from the updated primary firmware
	 * name
	 * TODO: Replace this to release and reload new firmware without
	 *       triggering timeout error
	 */
	if (trigger_timeout) {
		dev_info(dev, "Updating primary fw to %s\n",
				pdata->primary_firmware_name);
		ret = dbmdx_force_crash_fw(p->dev, true);
		if (ret < 0) {

			dev_err(p->dev, "%s: Error %d writing register %d\n",
					__func__, ret, TEST_TIMEOUT_ERROR);
			/*
			 * failed to reload firmware,
			 * don't update firmware name,
			 * so that next trigger can reload firmware
			 */
			pdata->primary_firmware_name = temp;
		}
	}
	mutex_unlock(&sctx->lock);
}

/* dbmdx_usecase_update(uc, clients)
 *
 * True table:
 * +----+-----+---------------------------------------------------------------+
 * + uc |usage|                  Comment                                      |
 * +----+-----+---------------------------------------------------------------+
 * |  N |  0  | Requested UC #N, will be apply instantly                      |
 * +----+-----+---------------------------------------------------------------+
 * | -1 |  0  | Uncoditional apply (refresh) last remembered UC, first apply  |
 * |    |     | idle UC then set desired UC.                                  |
 * |    |     | Note: This slove error case MB_ERROR__BUFFER_OVF_B_CLI0       |
 * +----+-----+---------------------------------------------------------------+
 * |  X | +1  | New client appears, UC will not be changed.                   |
 * +----+-----+---------------------------------------------------------------+
 * |  X | -1  | Client disappears, UC will not be changed.                    |
 * +----+-----+---------------------------------------------------------------+
 */
static int dbmdx_usecase_update(struct dbmdx_codec_ctx *sctx,
	int usecase, int usage)
{
	struct dbmdx_private *p = sctx->mfd_priv;
	struct device *dev = sctx->codec->dev;
	u32 ack_val;
	int reg = SET_ACTIVE_USECASE_ID;
	int old_in_use, ret = 0;
	bool do_idle = false;
	bool do_apply = false;

	dev_dbg(dev, "%s: in_use %d ; usage %d ; uc %d (cur/last UC %d/%d)\n",
		__func__, sctx->in_use, usage, usecase, sctx->usecase,
		sctx->usecase_last);

	mutex_lock(&sctx->lock);

	old_in_use = sctx->in_use;
	sctx->in_use += usage;

	/* Check for negative values of 'in_use' */
	if (sctx->in_use < 0) {
		dev_err(dev, "Unbalanced in_use ref. counting: %d => %d\n",
			usage, sctx->in_use);
		sctx->in_use = 0;
	}

	if (usecase == DBMDX_UC_FORCE)
		/* Switch in IDLE then set desired UC
		 * NOTE: This fix of error: READ_UNDERRUN || BUFFER_OVF
		 */
		do_apply = do_idle = true;
	else if (usage == DBMDX_USAGE_NEW_UC) {
		/* Remember new UC as desired UC */
		sctx->usecase = usecase;
		do_apply = true;
	}

	/* Skip apply new UC if not required */
	if (!do_apply)
		goto exit;

	/* Force switch, even if current UC is the same */
	if (do_idle)
		dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_ADDON_FB, reg, 0, &ack_val);

	/* Write real state of use-case in DBMDX chip */
	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_ADDON_FB, reg, sctx->usecase, &ack_val);
	if (ret < 0) {
		dev_err(dev, "%s: Error %d writing register %d\n",
			__func__, ret, reg);
		goto exit;
	} else
		sctx->usecase_last = sctx->usecase;

	trace_fba_usecase(sctx->usecase);

	dev_dbg(dev, "%s: Reg %d written with UC%d (in/last %d/%d), ack %d\n",
		__func__, reg, sctx->usecase, usecase, sctx->usecase_last,
		ack_val);

	/* TODO: In the future must be add wait until full start new usecase */

exit:
	mutex_unlock(&sctx->lock);

	return ret;
}

static int dbmdx_addon_register_tlv_get(struct snd_kcontrol *kcontrol,
			unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	struct device *dev = sctx->codec->dev;
	u32 hdr[2];
	u8 *buffer = NULL;
	int ret;

	if (!p)
		return -EAGAIN;

	if (!dbmdx_core_is_ready(p)) {
		dev_err(dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (size > params->max)
		return -EINVAL;

	ret = dbmdx_read_buf_reg32(p, PRIORITY_RUN_ON_IDLE_ISR, MSG_ID_ADDON_FB,
				   GET_ALGO_PARAM, 0, &buffer, 0);
	if (ret < 0)
		return ret;

	if (!buffer)
		return -ENOMEM;

	hdr[0] = SNDRV_CTL_TLVT_CONTAINER;
	hdr[1] = min_t(u32, ret, size - TLV_HEADER_SIZE);

	if (copy_to_user(bytes, hdr, TLV_HEADER_SIZE)) {
		ret = -EFAULT;
		goto exit;
	}

	bytes += TLV_HEADER_SIZE / sizeof(*bytes);
	if (copy_to_user(bytes, buffer, hdr[1])) {
		ret = -EFAULT;
		goto exit;
	}

exit:
	kfree(buffer);
	return hdr[1] + TLV_HEADER_SIZE;
}

static int dbmdx_addon_register_tlv_set(struct snd_kcontrol *kcontrol,
			const unsigned int __user *bytes, unsigned int size)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	struct device *dev = sctx->codec->dev;

	int ret, ack_val;
	u8 *buffer;

	if (!p)
		return -EAGAIN;

	if (!dbmdx_core_is_ready(p)) {
		dev_err(dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	/*return error if there is no data*/
	if (size <= TLV_HEADER_SIZE)
		return -EINVAL;

	size = size - TLV_HEADER_SIZE;

	if (size > params->max)
		return -EINVAL;

	buffer = kzalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* copy only the data which starts after TLV Header */
	if (copy_from_user(buffer, (char *)bytes + TLV_HEADER_SIZE, size)) {
		ret = -EFAULT;
		goto exit;
	}

	ret = dbmdx_send_message_ack(p, MSG_ID_ADDON_FB,
		     SET_ALGO_PARAM, buffer, size, &ack_val);

	if (ret < 0)
		dev_err(dev, "%s: Error sending TLV message\n", __func__);
	else if (ack_val < 0) {
		dev_err(dev, "%s: Error sending TLV message, returned ack value %d\n",
			__func__, ack_val);
		ret = -1;
	} else
		ret = size;

exit:
	kfree(buffer);
	return ret;

}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
static int dbmdx_snd_soc_bytes_tlv_callback(struct snd_kcontrol *kcontrol,
		    int op_flag, unsigned int size, unsigned int __user *tlv)
{
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	unsigned int count = size < params->max ? size : params->max;
	int ret = -ENXIO;

	switch (op_flag) {
	case SNDRV_CTL_TLV_OP_READ:
		ret = dbmdx_addon_register_tlv_get(kcontrol, tlv, count);
		break;
	case SNDRV_CTL_TLV_OP_WRITE:
		ret = dbmdx_addon_register_tlv_set(kcontrol, tlv, count);
		break;
	}
	return ret;
}
#endif

static int dbmdx_addon_fw_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	int ret = 0;
	u32 val;

	if (!p)
		return -EAGAIN;

	val = ucontrol->value.integer.value[0];
	dbmdx_fw_override_update(sctx, val);
	p->cur_firmware = val;

	return ret;
}

static int dbmdx_addon_fw_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;

	if (!p)
		return -EAGAIN;

	ucontrol->value.integer.value[0] = p->cur_firmware;

	return 0;
}

static int dbmdx_addon_register_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct device *dev = sctx->codec->dev;
	unsigned int reg = mc->reg;
	struct drv_message drv_msg;
	int ret = 0, ack_val;
	u32 val;
	struct mixer_ctl ctl;

	if (!p)
		return -EAGAIN;

	if (!dbmdx_core_is_ready(p)) {
		dev_err(dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	val = ucontrol->value.integer.value[0];

	drv_msg.msg_id = DBMDX_MIXER_CTRL_MODIFIED;
	ctl.reg = reg;
	ctl.val = val;
	drv_msg.msg = (void *)&ctl;
	dbmdx_notifier_call(p->dev, DBMDX_DRV_MESSAGE, &drv_msg);

	if (reg == SET_ACTIVE_USECASE_ID) {
		/* Remember desired usecase and apply if possible */
		dbmdx_usecase_update(sctx, val, DBMDX_USAGE_NEW_UC);
		return 0;
	}

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				     MSG_ID_ADDON_FB, reg, val, &ack_val);

	if (ret < 0)
		dev_err(dev, "%s: Error %d writing register %d\n",
							__func__, ret, reg);
	else if (ack_val < 0) {
		dev_err(dev, "%s: Error Register %d written %d, ack value %d\n",
						__func__, reg, val, ack_val);
		ret = -1;
	} else
		dev_dbg(dev, "%s: Register %d written %d, ack value %d\n",
						__func__, reg, val, ack_val);

	return ret;
}

static int dbmdx_addon_register_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct device *dev = sctx->codec->dev;
	unsigned int reg = mc->reg;
	u32 val;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!dbmdx_core_is_ready(p)) {
		dev_err(dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = dbmdx_read_register32(p, MSG_ID_ADDON_FB, reg, &val);
	if (ret < 0)
		dev_err(dev, "%s: Error %d reading register %d\n",
							__func__, ret, reg);
	else
		dev_dbg(dev, "%s: Register %d read, value %d\n",
							__func__, reg, val);

	ucontrol->value.integer.value[0] = val;
	return ret;
}

static int dbmdx_remember_currect_use_case(struct dbmdx_codec_ctx *sctx)
{
	int ret, val, reg = SET_ACTIVE_USECASE_ID;

	ret = dbmdx_read_register32(sctx->mfd_priv, MSG_ID_ADDON_FB, reg, &val);
	if (!ret)
		sctx->usecase = val;

	return ret;
}

static int dbmdx_is_alive_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct dbmdx_codec_ctx *sctx = snd_soc_component_get_drvdata(component);
	struct dbmdx_private *p = sctx->mfd_priv;
	struct device *dev = sctx->codec->dev;

	if (!p)
		return -EAGAIN;

	ucontrol->value.integer.value[0] = !!p->fw_alive;

	if (!ucontrol->value.integer.value[0])
		dev_err(dev, "%s: device not ready\n", __func__);

	return 0;
}

static const struct snd_kcontrol_new dbmdx_snd_playback_controls[] = {
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM, 0, 1, 0,
		dbmdx_get_switch_playback, dbmdx_put_switch_playback),
};

static const struct snd_soc_dapm_widget dbmdx_snd_widgets[] = {
	/*
	 * Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("DBMDX tdm1-aif-capture", NULL,
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DBMDX tdm1-aif-playback", NULL,
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DBMDX tdm0-aif-capture", "DBMDX TDM0 Capture",
		0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DBMDX tdm0-aif-playback", "DBMDX TDM0 Playback",
		0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER("DBMDX Playback", SND_SOC_NOPM, 0, 0,
		dbmdx_snd_playback_controls,
		ARRAY_SIZE(dbmdx_snd_playback_controls)),
};

static const struct snd_soc_dapm_route dbmdx_snd_routes[] = {
	{"tdm1-aif-playback", NULL, "Playback"},
	{"Playback", "Switch", "tdm0-aif-playback"},
	{"tdm0-aif-capture", NULL, "tdm1-aif-capture"},
	{"TDM1 Playback", NULL, "tdm1-aif-playback"},
	{"tdm1-aif-capture", NULL, "TDM1 Capture"},
};

static const struct snd_kcontrol_new dbmdx_snd_controls[] = {
	SOC_SINGLE_EXT("Addon Version Number", ADDON_VERSION_NUMBER, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Algorithm Version Number", ALGO_VERSION_NUMBER, 0,
		0xffff,	0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Usecase Manager", SET_ACTIVE_USECASE_ID, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Mic Gain", SET_MIC_GAIN_ID, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Mic Data Prop", SET_MIC_CAPTURE_PROPERTIES, 0,
		0xffffffff, 0, dbmdx_addon_register_get,
		dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Reset clock counter", RESET_CLOCK_COUNTER, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Read clock counter", READ_CLOCK_COUNTER, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM0 Prop", SET_TDM0_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM1 Prop", SET_TDM1_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM2 Prop", SET_TDM2_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM3 Prop", SET_TDM3_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM4 Prop", SET_TDM4_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM5 Prop", SET_TDM5_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM6 Prop", SET_TDM6_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TDM7 Prop", SET_TDM7_DATA_PROPERTIES, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("HDMI AEC Ref", SET_HDMI_AEC_REF, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Internal Speaker", SET_INTERNAL_SPEAKER, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Voice UL", SET_VOICE_UL, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Voice DL", SET_VOICE_DL, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Voice Assistant", SET_VOICE_ASSISTANT, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Voice Assistant Ref", SET_VOICE_ASSISTANT_REF, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("HDMI AEC Ref Channels", SET_HDMI_REF_CHANNELS, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("AIB Metadata", SET_AIB_METADATA, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Mono Unprocessed Mic", SET_UNPROCESSED_MIC, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
        SOC_SINGLE_EXT("Privacy Mode", SET_PRIVACY_MODE, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("HDMI Connect", SET_HDMI_CONNECT, 0, 0xffffffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
	MY_SND_SOC_BYTES_TLV("Algorithm param", ALGO_SIZE_MAX, NULL,
		NULL),
#else
	SND_SOC_BYTES_TLV("Algorithm param", ALGO_SIZE_MAX,
		dbmdx_addon_register_tlv_get, dbmdx_addon_register_tlv_set),
#endif
	SOC_SINGLE_EXT("Is Alive", 0, 0, 0xffffffff,
		0, dbmdx_is_alive_get, NULL),
	SOC_SINGLE_EXT("Speaker Volume Level", SET_SPK_VOLUME_LEVEL, 0, 0xffff,
		0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Voice Call Type", SET_VOICE_CALL_TYPE, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Algo Bypass", SET_ALGO_BYPASS, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
#ifdef CONFIG_DBMDX_TILT
	SOC_SINGLE_EXT("Tilt Angle", SET_TILT_ANGLE, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
#endif
#ifdef CONFIG_DBMDX_TV_DETECT
	SOC_SINGLE_EXT("TVD Algo Enable", SET_TVD_ALGO_ENABLE, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TVD Trigger", SET_TVD_TRIGGER, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TVD State", GET_TVD_STATE, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TVD Sticky Flag", GET_TVD_STICKY_FLAG, 0,
			0xffff, 0, dbmdx_addon_register_get,
			dbmdx_addon_register_set),
	SOC_SINGLE_EXT("TVD Sticky Playback Gain",
			GET_TVD_STICKY_PLAYBACK_GAIN, 0, 0xffffffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
#endif
	SOC_SINGLE_EXT("Ringtone Playback", SET_RINGTONE_PLAYBACK, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Platform ID", SET_PLATFORM_ID, 0, 0xffffffff,
			0, dbmdx_addon_register_get, NULL),
	SOC_SINGLE_EXT("Override FW For Algo", SET_FW_OVERRIDE, 0, 0xf,
			0, dbmdx_addon_fw_get, dbmdx_addon_fw_set),
	SOC_SINGLE_EXT("STAX Regression Control", STAX_REGRESSION_CTRL, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("AWB Cmd Size", STAX_AWB_CMD_SIZE, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),
	SOC_SINGLE_EXT("Output Error Report", OUTPUT_ISSUE_REPORT, 0, 0xffff,
			0, dbmdx_addon_register_get, dbmdx_addon_register_set),


};

static int dbmdx_message_handler(struct dbmdx_codec_ctx *sctx,
	struct fw_message *msg)
{
	struct device *dev = sctx->codec->dev;
	u8 id = msg->header.id;
	u8 reg = msg->header.reg_num;
	u32 info0 = msg->reg_info[0];
	struct dbmdx_private *p = sctx->mfd_priv;

	if (id == MSG_ID_FW && reg == REG_FW__ERROR_REPORT) {
		switch (info0) {
		case MB_ERROR__READ_UNDERRUN:
		case MB_ERROR__BUFFER_OVF_B_CLI0:
			if (sctx->skip_usecase_setting
				|| (p->cur_firmware == DBMDX_FW_1p)) {
				dbmdx_send_uevent(p,
					"FIRMWARE_STATE=DSPG_ALGO_ERR");
				return 0;
			}
			else
				/* Instantly apply last desired UC */
				dbmdx_usecase_update(sctx,
					DBMDX_UC_FORCE, DBMDX_USAGE_NEW_UC);

			dev_err(dev, "Due FW error 0x%04x, Apply UC%d\n",
				info0, sctx->usecase);
			break;
		}
	}
	if (id == MSG_ID_ADDON_FB) {
		switch (reg) {
		case FB_ADDON_USECASE_MANAGER_READY:
			dev_info(dev, "Usecase manager is ready\n");
			break;
		}
	}
	return 0;
}

static int dbmdx_codec_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct dbmdx_codec_ctx *sctx = container_of(nb, typeof(*sctx), nb);
	struct dbmdx_private *p = sctx->mfd_priv;

	switch (action) {
	case DBMDX_EV_CRASH:
		break;

	case DBMDX_EV_STARTUP:
		if (sctx->usecase < 0)
			/* Remember current usecase as desired */
			dbmdx_remember_currect_use_case(sctx);
		break;

	case DBMDX_EV_RECOVERY:
		if (sctx->skip_usecase_setting ||
			(p->cur_firmware == DBMDX_FW_1p))
			break;

		/* TODO: remove once DSP sends message on ready */
		msleep(DBMDX_MSLEEP_USECASE_SET_WAIT);
		/* Instantly apply last desired UC */
		dbmdx_usecase_update(sctx, DBMDX_UC_FORCE, DBMDX_USAGE_NEW_UC);
		break;

	case DBMDX_EV_MESSAGE:
		dbmdx_message_handler(sctx, data);
		break;
	}

	return 0;
}

static int dbmdx_codec_dt(struct snd_soc_codec *codec)
{
	struct dbmdx_codec_ctx *sctx = snd_soc_codec_get_drvdata(codec);
	const struct device *dev = codec->dev;
	const struct device_node *np = dev->of_node;
	const char *init_uc = "dspg,initial-usecase";
	const char *skip_uc = "dspg,skip-usecase-setting";
	s32 sval;
	int rval;

	if (!np)
		return 0;

	rval = of_property_read_s32(np, init_uc, &sval);
	if (rval == -EOVERFLOW) {
		/* Check for default UC read for FW on boot */
		if (of_property_read_bool(np, init_uc))
			sctx->usecase = -1; /* Use FW default UC as desired */

	} else if (!rval)
		sctx->usecase = sval;	/* Set DTS specific desired UC */

	if (of_property_read_bool(np, skip_uc))
		/* skip usecase setting after fw crash recovery */
		sctx->skip_usecase_setting = true;

	return 0;
}

static int dbmdx_codec_probe(struct snd_soc_codec *codec)
{
	struct dbmdx_codec_ctx *sctx;
	int ret;

	sctx = kzalloc(sizeof(*sctx), GFP_KERNEL);
	if (!sctx)
		return -ENOMEM;

	snd_soc_codec_set_drvdata(codec, sctx);
	sctx->codec = codec;
	sctx->mfd_dev = codec->dev->parent;
	sctx->mfd_priv = dev_get_drvdata(sctx->mfd_dev);

	mutex_init(&sctx->lock);

	ret = dbmdx_codec_dt(codec);
	if (ret) {
		dev_info(codec->dev, "DT parser failed\n");
		return ret;
	}

	sctx->nb.notifier_call = dbmdx_codec_notify;
	dbmdx_notifier_register(sctx->mfd_dev, &sctx->nb);

	/* Remember current usecase as desired */
	if (dbmdx_core_is_ready(sctx->mfd_priv) && sctx->usecase < 0)
		dbmdx_remember_currect_use_case(sctx);

	codec->component.name_prefix = "DBMDX";

	dev_info(codec->dev, "%s:\n", __func__);
	return 0;
}

static int dbmdx_codec_remove(struct snd_soc_codec *codec)
{
	struct dbmdx_codec_ctx *sctx = snd_soc_codec_get_drvdata(codec);

	kfree(sctx);

	dev_dbg(codec->dev, "%s\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int dbmdx_codec_suspend(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);
	return 0;
}

static int dbmdx_codec_resume(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);
	return 0;
}

static int dbmdx_codec_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int dbmdx_codec_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}
#endif
static int dbmdx_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	dev_dbg(codec->dev, "%s: level %d\n", __func__, (int)level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_dbmdx = {
	.probe   = dbmdx_codec_probe,
	.remove  = dbmdx_codec_remove,
	.suspend = dbmdx_codec_suspend,
	.resume  = dbmdx_codec_resume,
	.set_bias_level = dbmdx_set_bias_level,
	.reg_cache_size = 0,
	.reg_word_size = 0,
	.reg_cache_default = NULL,
	.ignore_pmdown_time = true,
	.component_driver = {
		.controls		= dbmdx_snd_controls,
		.num_controls		= ARRAY_SIZE(dbmdx_snd_controls),
		.dapm_widgets		= dbmdx_snd_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(dbmdx_snd_widgets),
		.dapm_routes		= dbmdx_snd_routes,
		.num_dapm_routes	= ARRAY_SIZE(dbmdx_snd_routes),
	},
};

static int dbmdx_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct dbmdx_codec_ctx *sctx = snd_soc_codec_get_drvdata(dai->codec);

	dbmdx_usecase_update(sctx, DBMDX_UC_NONE, DBMDX_USAGE_SET);

	return 0;
}

static void dbmdx_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct dbmdx_codec_ctx *sctx = snd_soc_codec_get_drvdata(dai->codec);

	dbmdx_usecase_update(sctx, DBMDX_UC_NONE, DBMDX_USAGE_RELEASE);
}

static const struct snd_soc_dai_ops dbmdx_ops = {
	.startup = dbmdx_startup,
	.shutdown = dbmdx_shutdown,
};

static const struct dev_pm_ops dbmdx_codec_pm_ops = {
	SET_RUNTIME_PM_OPS(dbmdx_codec_runtime_suspend,
			   dbmdx_codec_runtime_resume, NULL)
};

/* DBMDX codec DAI: */
static struct snd_soc_dai_driver dbmdx_codec_dai[] = {
	{
		.id = 0,
		.name = "DBMDX TDM0",
		.playback = {
			.stream_name = "DBMDX TDM0 Playback",
			.channels_min	= 1,
			.channels_max	= MAX_SUPPORTED_TDM0_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "DBMDX TDM0 Capture",
			.channels_min	= 1,
			.channels_max	= MAX_SUPPORTED_TDM0_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &dbmdx_ops,
	},
	{
		.id = 1,
		.name = "DBMDX TDM1",
		.playback = {
			.stream_name = "DBMDX TDM1 Playback",
			.channels_min	= 2,
			.channels_max	= MAX_SUPPORTED_TDM1_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "DBMDX TDM1 Capture",
			.channels_min	= 2,
			.channels_max	= MAX_SUPPORTED_TDM1_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.id = 2,
		.name = "DBMDX TDM2",
		.playback = {
			.stream_name = "DBMDX TDM2 Playback",
			.channels_min	= MAX_SUPPORTED_TDM2_CHANNELS,
			.channels_max	= MAX_SUPPORTED_TDM2_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "DBMDX TDM2 Capture",
			.channels_min	= MAX_SUPPORTED_TDM2_CHANNELS,
			.channels_max	= MAX_SUPPORTED_TDM2_CHANNELS,
			.rates		=
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000,
			.formats	= SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static int dbmdx_codec_platform_probe(struct platform_device *pdev)
{
	/* FIXME: usage_count is required to be two for
	 * pm_runtime_force_resume to invoke runtime_resume callback.
	 * Hence use of pm_runtime_get_no_resume twice.
	 * Need to check if its kernel version specific issue.
	 */

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	dev_set_name(&pdev->dev, pdev->dev.driver->name);

	/* DBMDX snd codec registration */
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_dbmdx,
			dbmdx_codec_dai, ARRAY_SIZE(dbmdx_codec_dai));
}

static int dbmdx_codec_platform_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	/*DBMDX snd codec unregister */
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id dbmdx_codec_match[] = {
	{ .compatible = "dspg,dbmdx-codec", },
	{},
};

static struct platform_driver dbmdx_codec_driver = {
	.driver = {
		.name = "dbmdx-codec",
		.of_match_table = dbmdx_codec_match,
		.pm = &dbmdx_codec_pm_ops,
	},

	.probe = dbmdx_codec_platform_probe,
	.remove = dbmdx_codec_platform_remove,
};

module_platform_driver(dbmdx_codec_driver);

MODULE_DESCRIPTION("DSPG DBMDX Codec Driver");
MODULE_AUTHOR("DSP Group");
MODULE_LICENSE("GPL v2");

