/*
 * es705.c  --  Audience eS705 ALSA SoC Audio driver
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Greg Clemson <gclemson@audience.com>
 *
 * Code Updates:
 *       Genisim Tsilker <gtsilker@audience.com>
 *            - Code refactoring
 *            - FW download functions update
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include "esxxx.h"
#include "es705_escore.h"
#include "es705-routes.h"
#include "escore.h"
#include "escore-slim.h"
#include "escore-spi.h"
#include "escore-uart.h"
#include "escore-uart-common.h"
#include "escore-i2c.h"
#include "escore-i2s.h"
#include "escore-cdev.h"
#include "escore-vs.h"
#include "escore-version.h"
#ifdef CONFIG_SND_SOC_ES8XX_AS_CODEC
#include "esxxx-codec.h"
#endif

/* local function proto type */
/* static int es705_dev_rdb(struct escore_priv *es705, void *buf, int len);
static int es705_dev_wdb(struct escore_priv *es705, const void *buf, int len);
*/

#define ES705_CMD_ACCESS_WR_MAX 2
#define ES705_CMD_ACCESS_RD_MAX 2

#if defined(CONFIG_SND_SOC_ES_SLIM)
static int es705_slim_be_id[ES_NUM_CODEC_SLIM_DAIS] = {
	ES_SLIM_2_CAP, /* for ES_SLIM_1_PB tx from es705 */
	ES_SLIM_3_PB, /* for ES_SLIM_1_CAP rx to es705 */
	ES_SLIM_3_CAP, /* for ES_SLIM_2_PB tx from es705 */
	-1, /* for ES_SLIM_2_CAP */
	-1, /* for ES_SLIM_3_PB */
	-1, /* for ES_SLIM_3_CAP */
};
#endif

#include "es705-access.h"

/* Route state for Internal state management */
enum es705_power_state {
ES705_POWER_BOOT,
ES705_POWER_SLEEP,
ES705_POWER_SLEEP_PENDING,
ES705_POWER_AWAKE
};

static const char * const es705_power_state_texts[] = {
	"None", "Sleep", "MP_Sleep", "MP_Cmd", "Normal", "Overlay", "Low_Power"
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
	, "VS_Streaming"
#endif
};

/* codec private data TODO: move to runtime init */
struct escore_priv escore_priv = {
	.probe = es705_core_probe,
	.set_streaming = es705_set_streaming,
	.es_vs_route_preset = ES705_DMIC0_VS_ROUTE_PREST,
	.es_vs_tuning_route_preset = ES705_DMIC0_VS_TUNING_ROUTE_PREST,
#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
	.es_cvs_preset = ES705_DMIC0_CVS_PREST_SINGLE_DOWNLOAD,
#else
	.es_cvs_preset = ES705_DMIC0_CVS_PREST,
#endif
	.es_wdb_max_size = ES705_WDB_MAX_SIZE,
	.flag.rx1_route_enable = 0,
	.flag.tx1_route_enable = 0,
	.flag.rx2_route_enable = 0,

	.flag.vs_enable = 0,
	.ap_tx1_ch_cnt = 2,

	.escore_power_state = ES_SET_POWER_STATE_NORMAL,
	.streamdev.intf = ES_UART_INTF,
	.flag.ns = 1,
	.flag.sleep_enable = 0, /*Auto sleep disabled default*/
	.sleep_delay = 3000,
	.wake_count = 0,
	.flag.sleep_abort = 0,
	.delay.wakeup_to_normal = ES705_DELAY_WAKEUP_TO_NORMAL,
	.delay.wakeup_to_vs = ES705_DELAY_WAKEUP_TO_VS,
	.delay.vs_to_normal = ES705_DELAY_VS_TO_NORMAL,
};

const char *esxxx_mode[] = {
	"SBL",
	"STANDARD",
	"VOICESENSE",
};

struct snd_soc_dai_driver es705_dai[];

/* indexed by ES705 INTF number */
static const u32 es705_streaming_cmds[] = {
	[ES_INVAL_INTF] = 0x00000000,	/* ES_NULL_INTF */
	[ES_SLIM_INTF]  = 0x90250200,	/* ES_SLIM_INTF */
	[ES_I2C_INTF]   = 0x90250000,	/* ES_I2C_INTF  */
	[ES_SPI_INTF]   = 0x90250300,	/* ES_SPI_INTF  */
	[ES_UART_INTF]  = 0x90250100,	/* ES_UART_INTF */
};

#define ES705_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			SNDRV_PCM_RATE_44100)
#define ES705_SLIMBUS_RATES (SNDRV_PCM_RATE_48000)

#define ES705_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |\
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |\
			SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)
#define ES705_SLIMBUS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S16_BE)

struct snd_soc_dai_driver es705_dai[] = {
#if defined(CONFIG_SND_SOC_ES_SLIM)
	{
		.name = "es705-slim-rx1",
		.id = ES705_SLIM_1_PB,
		.playback = {
			.stream_name = "SLIM_PORT-1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es705-slim-tx1",
		.id = ES705_SLIM_1_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es705-slim-rx2",
		.id = ES705_SLIM_2_PB,
		.playback = {
			.stream_name = "SLIM_PORT-2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es705-slim-tx2",
		.id = ES705_SLIM_2_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es705-slim-rx3",
		.id = ES705_SLIM_3_PB,
		.playback = {
			.stream_name = "SLIM_PORT-3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es705-slim-tx3",
		.id = ES705_SLIM_3_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-3 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_SLIMBUS_RATES,
			.formats = ES705_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
#endif
#if defined(CONFIG_SND_SOC_ES_I2S)
	{
		.name = "earSmart-porta",
		.id = ES_I2S_PORTA,
		.playback = {
			.stream_name = "PORTA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_RATES,
			.formats = ES705_FORMATS,
		},
		.capture = {
			.stream_name = "PORTA Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_RATES,
			.formats = ES705_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
	{
		.name = "earSmart-portb",
		.id = ES_I2S_PORTB,
		.playback = {
			.stream_name = "PORTB Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_RATES,
			.formats = ES705_FORMATS,
		},
		.capture = {
			.stream_name = "PORTB Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES705_RATES,
			.formats = ES705_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
#endif
};

static struct escore_slim_dai_data slim_dai_data[ES_NUM_CODEC_SLIM_DAIS];
static struct escore_slim_ch slim_rx[ES_SLIM_RX_PORTS];
static struct escore_slim_ch slim_tx[ES_SLIM_TX_PORTS];

#ifdef CONFIG_ARCH_MSM8974
/*Slimbus channel map for APQ8074*/
static int es705_slim_rx_port_to_ch[ES_SLIM_RX_PORTS] = {
		152, 153, 154, 155, 134, 135
};
static int es705_slim_tx_port_to_ch[ES_SLIM_TX_PORTS] = {
		156, 157, 144, 145, 146, 147
};
#else
/*Slimbus channel map for APQ8060*/
static int es705_slim_rx_port_to_ch[ES_SLIM_RX_PORTS] = {
		152, 153, 154, 155, 134, 135
};
static int es705_slim_tx_port_to_ch[ES_SLIM_TX_PORTS] = {
		156, 157, 138, 139, 143, 144
};
#endif

#define CODEC_ID "es705-codec"
#define CODEC_GEN0_ID "es705-codec-gen0"
#define CODEC_INTF_ID "es705-codec-intf"

#define CODEC_ID_ES704 "es704-codec"
#define CODEC_GEN0_ID_ES704 "es704-codec-gen0"
#define CODEC_INTF_ID_ES704 "es704-codec-intf"

#define CODEC_ID_ES804 "es804-codec"
#define CODEC_GEN0_ID_ES804 "es804-codec-gen0"
#define CODEC_INTF_ID_ES804  "es804-codec-intf"

int es705_dev_rdb(struct escore_priv *es705, void *buf, int len)
{
	dev_dbg(es705->dev, "%s - default\n", __func__);
	return 0;
}

int es705_dev_wdb(struct escore_priv *es705, const void *buf, int len)
{
	dev_dbg(es705->dev, "%s - default\n", __func__);
	return 0;
}

int es705_switch_route(long route_index)
{
	struct escore_priv *es705 = &escore_priv;
	char *event_start[] = { "ACTION=ADNC_UC_START", NULL };
	char *event_stop[] = { "ACTION=ADNC_UC_STOP", NULL };
	int rc;
	es_profiling_var(switch_route);
	if (route_index >= ROUTE_MAX) {
		dev_dbg(es705->dev,
		"%s(): new es705_internal_route = %ld is out of range\n",
		__func__, route_index);
		return -EINVAL;
	}

	dev_dbg(es705->dev,
	"%s(): switch current es705_internal_route = %ld to new route = %ld\n",
	__func__, es705->internal_route_num, route_index);

	es705->internal_route_num = route_index;

	if (!route_index) {
		kobject_uevent_env(&es705->dev->kobj, KOBJ_CHANGE, event_stop);
		return 0;
	}

	es_profiling_start(&switch_route);

	rc = escore_write_block(es705,
			  es705_route_config[es705->internal_route_num].route);
	if (!rc)
		kobject_uevent_env(&es705->dev->kobj, KOBJ_CHANGE, event_start);

	es_profiling_end(&switch_route);
	print_profiling_time(switch_route, "switch route");

	return rc;
}

static int es705_get_route_status(struct escore_priv *escore, int algo_id)
{
	int ret = 0, value = 0;
	u32 api_word = 0;
	api_word = ES705_ROUTE_STATE | algo_id;

	ret = escore_cmd_locked(escore, api_word, &value);
	if (ret < 0) {
		pr_err("%s(): escore_cmd() ret:%d", __func__, ret);
		goto out;
	}
	ret = (value & 0xFF);

out:
	return ret;
}


static ssize_t es705_route_status_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret = 0;
	unsigned int value = 0;
	struct escore_priv *escore = &escore_priv;
	char *status_name = "Route Status";
	int i;
	const char *status_text[] = {
		"Active",
		"Muting",
		"Switching",
		"Unmuting",
		"Inactive",
	};
	const char *algo_type_text[] = {
		"VP",
		"MM",
		"AF",
		"PT",
		"VQ",
	};
	const u8 algo_id[5] = {
		[0] = 0,
		[1] = 1,
		[2] = 2,
		[3] = 3,
		[4] = 4,
	};

	for (i = 0; i < sizeof(algo_id); i++) {
		value = es705_get_route_status(escore,
				algo_id[i]);
		if (value < 0) {
			pr_err("%s(): Get Route Status Failed: %d\n",
					__func__, value);
			ret = value;
			goto out;
		}
		ret += snprintf(buf + ret, PAGE_SIZE,
				"%s for Algo %s = %s\n",
				status_name, algo_type_text[i],
				status_text[value]);
	}
out:
	return ret;
}

static DEVICE_ATTR(route_status, 0440, es705_route_status_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/route_status */

static ssize_t es705_route_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct escore_priv *es705 = &escore_priv;

	dev_dbg(es705->dev, "%s(): route=%ld\n",
		__func__, es705->internal_route_num);
	return snprintf(buf, PAGE_SIZE, "route=%ld\n",
			es705->internal_route_num);
}

static DEVICE_ATTR(route, 0440, es705_route_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/route */

static ssize_t es705_rate_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct escore_priv *es705 = &escore_priv;

	dev_dbg(es705->dev, "%s(): rate=%ld\n", __func__, es705->internal_rate);
	return snprintf(buf, PAGE_SIZE, "rate=%ld\n",
			es705->internal_rate);
}

static DEVICE_ATTR(rate, 0440, es705_rate_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/rate */

static ssize_t es705_power_state_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct escore_priv *es705 = &escore_priv;

	dev_dbg(es705->dev, "%s(): power state = %s\n", __func__,
			es705_power_state_texts[es705->escore_power_state]);
	return snprintf(buf, PAGE_SIZE, "power state = %s\n",
			es705_power_state_texts[es705->escore_power_state]);
}

static DEVICE_ATTR(power_state, 0440, es705_power_state_show, NULL);

static ssize_t es705_get_pm_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", escore_priv.pm_status ?
			"on" : "off");
}

static ssize_t es705_set_pm_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	pr_info("%s(): requested - %s\n", __func__, buf);
	if (!strncmp(buf, "on", 2))
		escore_pm_status_update(ES_PM_ON);
	else if (!strncmp(buf, "off", 3))
		escore_pm_status_update(ES_PM_OFF);
	return count;

}
static DEVICE_ATTR(pm_enable, 0660, es705_get_pm_enable, es705_set_pm_enable);

#define SIZE_OF_VERBUF 256
/* TODO: fix for new read/write. use es705_read() instead of BUS ops */
static ssize_t es705_fw_version_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int idx = 0;
	unsigned int value;
	char versionbuffer[SIZE_OF_VERBUF];
	char *verbuf = versionbuffer;
	struct escore_priv *escore = &escore_priv;

	memset(verbuf, 0, SIZE_OF_VERBUF);

	mutex_lock(&escore->access_lock);
	value = escore_read(NULL, ES705_FW_FIRST_CHAR);
	*verbuf++ = (value & 0x00ff);
	for (idx = 0; idx < (SIZE_OF_VERBUF-2); idx++) {
		value = escore_read(NULL, ES705_FW_NEXT_CHAR);
		*verbuf++ = (value & 0x00ff);
		if (!value)
			break;
	}
	mutex_unlock(&escore->access_lock);
	/* Null terminate the string*/
	*verbuf = '\0';
	dev_info(dev, "Audience fw ver %s\n", versionbuffer);
	return snprintf(buf, PAGE_SIZE, "FW Version = %s\n", versionbuffer);
}

static DEVICE_ATTR(fw_version, 0440, es705_fw_version_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/fw_version */

static ssize_t es705_clock_on_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int ret = 0;

	return ret;
}

static DEVICE_ATTR(clock_on, 0440, es705_clock_on_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/clock_on */

static ssize_t es705_gpio_reset_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct escore_priv *es705 = &escore_priv;
	dev_dbg(es705->dev, "%s(): GPIO reset\n", __func__);
	es705->mode = SBL;
	escore_gpio_reset(es705);
	dev_dbg(es705->dev, "%s(): Ready for STANDARD download by proxy\n",
		__func__);
	return count;
}

static DEVICE_ATTR(gpio_reset, 0640, NULL, es705_gpio_reset_set);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/gpio_reset */


static ssize_t es705_overlay_mode_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct escore_priv *es705 = &escore_priv;
	int rc;
	int value = ES705_SET_POWER_STATE_VS_OVERLAY;

	dev_dbg(es705->dev, "%s(): Set Overlay mode\n", __func__);

	es705->mode = SBL;
	rc = escore_write_locked(NULL, ES705_POWER_STATE , value);
	if (rc) {
		dev_err(escore_priv.dev, "%s(): Set Overlay mode fail %d\n",
			__func__, rc);
	} else {
		msleep(50);
		escore_priv.escore_power_state =
			ES705_SET_POWER_STATE_VS_OVERLAY;
		/* wait until es705 SBL mode activating */
		dev_dbg(es705->dev, "%s(): Ready for VOICESENSE download by proxy\n",
		__func__);
		dev_info(es705->dev, "%s(): After successful VOICESENSE download,"
			"Enable Event Intr to Host\n",
			__func__);
	}
	return count;
}

static DEVICE_ATTR(overlay_mode, 0640, NULL, es705_overlay_mode_set);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/overlay_mode */

static ssize_t es705_vs_event_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct escore_priv *es705 = &escore_priv;
	int rc;
	int value = ES705_SYNC_INTR_RISING_EDGE;
	dev_dbg(es705->dev, "%s(): Enable Voice Sense Event to Host\n",
		__func__);

	es705->mode = VOICESENSE;
	/* Enable Voice Sense Event INTR to Host */
	rc = escore_write_locked(NULL, ES705_EVENT_RESPONSE, value);
	if (rc)
		dev_err(es705->dev, "%s(): Enable Event Intr fail %d\n",
			__func__, rc);
	return count;
}

static DEVICE_ATTR(vs_event, 0640, NULL, es705_vs_event_set);
/* /sys/devices/platform/msm_slim_ctrl.1/earSmart-codec-gen0/vs_event */

static ssize_t escore_version_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ESCORE_VERSION);
}
static DEVICE_ATTR(escore_version, 0440, escore_version_show, NULL);

static ssize_t escore_debug_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct escore_priv *es705 = &escore_priv;
	int ret;

	if (es705->debug_buff != NULL) {
		memcpy(buf, es705->debug_buff, DBG_BLK_SIZE);
		kfree(es705->debug_buff);
		es705->debug_buff = NULL;
		return DBG_BLK_SIZE;
	}

	mutex_lock(&es705->access_lock);

	ret = escore_pm_get_sync();
	if (ret < 0) {
		pr_err("%s(): pm_get_sync failed :%d\n", __func__, ret);
		mutex_unlock(&es705->access_lock);
		return ret;
	}

	ret = read_debug_data(buf);
	escore_pm_put_autosuspend();

	mutex_unlock(&es705->access_lock);
	return ret;
}
static DEVICE_ATTR(escore_debug, 0440, escore_debug_show, NULL);

static ssize_t es705_get_profile_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			escore_priv.flag.profile_enable ? "on" : "off");
}

static ssize_t es705_set_profile_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{

	struct escore_priv *escore = &escore_priv;

	pr_info("%s(): requested - %s\n", __func__, buf);
	if (!strncmp(buf, "1", 1))
		escore->flag.profile_enable = ES_PROFILING_ON;
	else if (!strncmp(buf, "0", 1))
		escore->flag.profile_enable = ES_PROFILING_OFF;
	return count;

}
static DEVICE_ATTR(profile_enable, 0660,
		es705_get_profile_enable, es705_set_profile_enable);

static struct attribute *core_sysfs_attrs[] = {
	&dev_attr_route_status.attr,
	&dev_attr_route.attr,
	&dev_attr_rate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_clock_on.attr,
	&dev_attr_gpio_reset.attr,
	&dev_attr_overlay_mode.attr,
	&dev_attr_vs_event.attr,
	&dev_attr_pm_enable.attr,
	&dev_attr_escore_version.attr,
	&dev_attr_power_state.attr,
	&dev_attr_escore_debug.attr,
	&dev_attr_profile_enable.attr,
	NULL
};

static struct attribute_group core_sysfs = {
	.attrs = core_sysfs_attrs
};

static int es705_fw_download(struct escore_priv *es705, int fw_type)
{
	int rc = 0;
	struct escore_voice_sense *voice_sense =
		(struct escore_voice_sense *) es705->voice_sense;

	es_profiling_var(fw_setup);
	es_profiling_var(fw_data);
	es_profiling_var(fw_finish);
	es_profiling_var(fw_load);

	dev_info(es705->dev, "%s(): firmware download type %d begin\n",
							__func__, fw_type);
	if (fw_type != VOICESENSE && fw_type != STANDARD && fw_type != DOLBY) {
		dev_err(es705->dev, "%s(): Unknown firmware type (%d)\n",
			__func__, fw_type);
		goto es705_fw_download_failed;
	}

	if (!es705->boot_ops.setup || !es705->boot_ops.finish) {
		dev_err(es705->dev, "%s(): boot setup or finish func undef\n",
								__func__);
		goto es705_fw_download_failed;
	}

	/* Reset Mode to Polling */
	es705->cmd_compl_mode = ES_CMD_COMP_POLL;

	es_profiling_start(&fw_load);

	if (es705->bus.ops.high_bw_open) {
		rc = es705->bus.ops.high_bw_open(es705);
		if (rc) {
			dev_err(es705->dev, "%s(): high_bw_open failed %d\n",
			__func__, rc);
			goto es705_high_bw_open_failed;
		}
	}
	/* Reset es814 and ready for firmware download.
	 * During VS to NS transition, at time of NS firmware download
	 * if we reset chip before high bw bus setup, firmware download
	 * fails in first attempt.
	 */
	es_profiling_start(&fw_setup);
	escore_gpio_reset(es705);
	rc = es705->boot_ops.setup(es705);
	if (rc) {
		dev_err(es705->dev, "%s(): firmware download start error %d\n",
								__func__, rc);
		goto es705_fw_download_failed;
	}
	es_profiling_end(&fw_setup);
	/* Set flag to Wait for API Interrupt */
	escore_set_api_intr_wait(es705);
	if (fw_type == VOICESENSE) {
		es_profiling_start(&fw_data);
		rc = es705->bus.ops.high_bw_write(es705,
				(char *)voice_sense->vs->data,
				voice_sense->vs->size);
	} else if (fw_type == DOLBY) {
		es_profiling_start(&fw_data);
		rc = es705->bus.ops.high_bw_write(es705,
				(char *)es705->dolby->data,
				es705->dolby->size);
	} else {
		es_profiling_start(&fw_data);
		rc = es705->bus.ops.high_bw_write(es705,
				(char *)es705->standard->data,
				es705->standard->size);
	}
	if (rc) {
		dev_err(es705->dev, "%s(): firmware write error %d\n",
								__func__, rc);
		rc = -EIO;
		goto es705_fw_download_failed;
	}
	es_profiling_end(&fw_data);

	dev_info(es705->dev, "%s(): firmware write done\n", __func__);

	es_profiling_start(&fw_finish);
	/* Wait for API Interrupt to confirm
	 * that firmware is ready to accept command */
	if (es705->pdata->gpioa_gpio != -1) {
		rc = escore_api_intr_wait_completion(es705);
		if (rc)
			goto es705_fw_download_failed;
	}
	rc = es705->boot_ops.finish(es705);
	if (rc) {
		dev_err(es705->dev,
			"%s() firmware download finish error %d\n",
			__func__, rc);
		goto es705_fw_download_failed;
	}
	es705->mode = fw_type;
	es_profiling_end(&fw_finish);
	es_profiling_end(&fw_load);

	print_profiling_time(fw_setup, "fw_download setup");
	print_profiling_time(fw_data, "fw_download data");
	print_profiling_time(fw_finish, "fw_download finish");
	print_profiling_time(fw_load, "fw_download load");

	dev_info(es705->dev, "%s(): firmware download type %d done\n",
			__func__, fw_type);

	/* Reconfig API Interrupt mode */
	rc = escore_reconfig_api_intr(es705);
	if (rc)
		goto es705_fw_download_failed;

es705_fw_download_failed:
	if (es705->bus.ops.high_bw_close) {
		int ret = 0;
		ret = es705->bus.ops.high_bw_close(es705);
		if (ret) {
			dev_err(es705->dev, "%s(): high_bw_close failed %d\n",
				__func__, ret);
			rc = ret;
		}
	}

es705_high_bw_open_failed:
	return rc;
}

void es705_slim_setup(struct escore_priv *escore_priv)
{
	int i;
	int ch_cnt;
	dev_dbg(escore_priv->dev, "%s():\n", __func__);

	escore_priv->init_slim_slave(escore_priv);

	/* allocate ch_num array for each DAI */
	for (i = 0; i < (ARRAY_SIZE(es705_dai)); i++) {
		switch (es705_dai[i].id) {
		case ES_SLIM_1_PB:
		case ES_SLIM_2_PB:
		case ES_SLIM_3_PB:
			ch_cnt = es705_dai[i].playback.channels_max;
			break;
		case ES_SLIM_1_CAP:
		case ES_SLIM_2_CAP:
		case ES_SLIM_3_CAP:
			ch_cnt = es705_dai[i].capture.channels_max;
			break;
		default:
				continue;
		}
		escore_priv->slim_dai_data[i].ch_num =
			kzalloc((ch_cnt * sizeof(unsigned int)), GFP_KERNEL);
	}
	/* front end for RX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_PB)].ch_num[0] =
						es705_slim_rx_port_to_ch[0];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_PB)].ch_num[1] =
						es705_slim_rx_port_to_ch[1];
	/* back end for RX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_CAP)].ch_num[0] =
						es705_slim_tx_port_to_ch[2];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_CAP)].ch_num[1] =
						es705_slim_tx_port_to_ch[3];
	/* front end for TX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_CAP)].ch_num[0] =
						es705_slim_tx_port_to_ch[0];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_CAP)].ch_num[1] =
						es705_slim_tx_port_to_ch[1];
	/* back end for TX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_PB)].ch_num[0] =
						es705_slim_rx_port_to_ch[4];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_PB)].ch_num[1] =
						es705_slim_rx_port_to_ch[5];
	/* front end for RX2 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_PB)].ch_num[0] =
						es705_slim_rx_port_to_ch[2];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_PB)].ch_num[1] =
						es705_slim_rx_port_to_ch[3];
	/* back end for RX2 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_CAP)].ch_num[0] =
						es705_slim_tx_port_to_ch[4];
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_CAP)].ch_num[1] =
						es705_slim_tx_port_to_ch[5];
}


static int es705_channel_dir(int dai_id)
{
	int dir = ES_SLIM_CH_UND;

	if (dai_id == ES_SLIM_1_PB ||
			dai_id == ES_SLIM_2_PB ||
			dai_id == ES_SLIM_3_PB) {
		dir = ES_SLIM_CH_RX;
	} else if (dai_id == ES_SLIM_1_CAP ||
			dai_id == ES_SLIM_2_CAP ||
			dai_id == ES_SLIM_3_CAP)  {
		dir = ES_SLIM_CH_TX;
	}

	return dir;
}

int es705_slim_sleep(struct escore_priv *escore)
{
	/* TODO:
	 * Add the code for slimbus
	 */
	return 0;
}

int es705_slim_wakeup(struct escore_priv *escore)
{
	/* TODO:
	 * Add the code for slimbus
	 */
	return 0;
}

int es705_i2c_sleep(struct escore_priv *escore)
{
	/* TODO:
	 * Add the code for i2c
	 */
	return 0;
}

int es705_i2c_wakeup(struct escore_priv *escore)
{
	/* TODO:
	 * Add the code for i2c
	 */
	return 0;
}

int es705_bootup(struct escore_priv *es705)
{
	u8 retry = ES705_FW_DOWNLOAD_MAX_RETRY;
	int rc, cmd;
	u32 rsp;
	mutex_lock(&es705->pm_mutex);
	mutex_unlock(&es705->pm_mutex);

	INC_DISABLE_FW_RECOVERY_USE_CNT(es705);
	do {
		pr_info("%s() VS start\n", __func__);
		rc = es705_fw_download(es705, VOICESENSE);
	} while (rc && retry--);

	if (rc) {
		dev_err(es705->dev, "%s(): STANDARD fw download error %d\n",
								__func__, rc);
	}
	DEC_DISABLE_FW_RECOVERY_USE_CNT(es705);
	if (!rc)
		es705->chip_enable = 1;
	return rc;
}

static int es705_vs_to_ns_switch(struct escore_priv *es705)
{
	dev_dbg(es705->dev, "%s(): VS -> NS Switing\n", __func__);
	return es705_bootup(es705);
}

static int es705_wakeup(struct escore_priv *es705)
{
	return escore_wakeup(es705);
}

static int es705_put_control_value(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = escore_priv.codec; */
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];
	rc = escore_write_locked(NULL, reg, value);

	return 0;
}

static int es705_get_control_value(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	/* struct snd_soc_codec *codec = escore_priv.codec; */
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = escore_read_locked(NULL, reg);
	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int es705_put_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.enumerated.item[0];
	rc = escore_write_locked(NULL, reg, value);

	return 0;
}

static int es705_get_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	unsigned int reg = e->reg;
	int value;

	value = escore_read_locked(NULL, reg);
	if (value < 0) {
		dev_err(escore_priv.dev,
			"%s(): es705 get control enum fail %d\n",
			__func__, value);

		return value;
	}

	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int es705_put_chip_enable(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];
	if (escore->chip_enable == value) {
		pr_debug("Already set %d\n", escore->chip_enable);
		return rc;
	}
	mutex_lock(&escore->access_lock);
	if (value) {
		rc = escore_pm_get_sync();
		if (rc < 0) {
			pr_err("%s(): pm_get_sync failed :%d\n", __func__, rc);
			mutex_unlock(&escore->access_lock);
			return rc;
		}
		rc = escore->boot_ops.bootup(escore);
		if (rc < 0) {
			pr_err("Chip enable fail %d\n", rc);
			mutex_unlock(&escore->access_lock);
			return rc;
		}
		escore_pm_put_autosuspend();
		escore->pm_enable = 1;
	} else {
		escore_gpio_reset(escore);
		escore->pm_enable = 0;
	}
	escore->chip_enable = value;
	mutex_unlock(&escore->access_lock);

	return rc;
}

static int es705_get_chip_enable(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.chip_enable;
	return 0;
}


static int es705_put_bargein_state(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];
	if (escore->bargein_state == value) {
		pr_debug("Ignoring same value %d\n", escore->bargein_state);
		return rc;
	}
	mutex_lock(&escore->access_lock);
	escore->bargein_state = value;
	mutex_unlock(&escore->access_lock);

	return rc;
}

static int es705_get_bargein_state(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.bargein_state;
	return 0;
}

static int es705_put_chip_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *escore = &escore_priv;
	unsigned int value;
	int rc = 0;
	u32 cmd, rsp;

	value = ucontrol->value.integer.value[0];
	if (escore->chip_mode == value) {
		pr_debug("Ignoring same value %d\n", escore->bargein_state);
		return rc;
	}
	mutex_lock(&escore->access_lock);
	if (value == VOIP) {
		/* download NS binary */
		rc = es705_fw_download(escore, STANDARD);
		if (rc) {
			pr_err("NS fw download fail %d\n", rc);
			return rc;
		}
		escore->bargein_state = Disable;
	} else {
		if (escore->chip_mode == VOIP) {
			/* download VS binary */
			rc = es705_fw_download(escore, VOICESENSE);
			if (rc) {
				pr_err("VS fw download fail %d\n", rc);
				return rc;
			}
		}
		if (value == BARGEIN)
			escore->bargein_state = Enable;
		/* Training route will be set from MW */
		else if (value == TRAINING)
			escore->bargein_state = Disable;
	}

	escore->chip_mode = value;
	mutex_unlock(&escore->access_lock);

	if (value == BARGEIN) {
		/* switch to bargein route */
		rc = es705_switch_route(ROUTE_BARGEIN);
		/* send asra route */
#if !defined CONFIG_SND_SOC_ES817_BASE
		cmd = ES_SET_CVS_PRESET << 16 | ES705_ASR_PRESET;
		rc = escore_cmd_nopm(escore, cmd, &rsp);
		if (rc) {
			dev_err(escore->dev,
					"%s(): Set CVS Preset cmd fail %d\n",
					__func__, rc);
			return rc;
		}
#endif
	}
	return rc;
}

static int es705_get_chip_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.chip_mode;
	return 0;
}

static int sde_put_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	struct escore_priv *es804 = &escore_priv;
	unsigned int reg = e->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.enumerated.item[0];

	rc = escore_write_locked(NULL, reg, value);
	if (rc < 0) {
		dev_err(es804->dev,
				"%s(): sde put control enum fail %d\n",
				__func__, rc);

		return rc;
	}

	if (value)
		es804->mode = AF;
	else
		es804->mode = STANDARD;

	return 0;
}

static int es705_get_power_control_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = escore_priv.escore_power_state;

	return 0;
}

static int es705_start_int_osc(void)
{
	int rc = 0;
	int retry = MAX_RETRY_TO_SWITCH_TO_LOW_POWER_MODE;

	dev_info(escore_priv.dev, "%s()\n", __func__);

	/* Start internal Osc. */
	rc = escore_write(NULL, ES705_VS_INT_OSC_MEASURE_START, 0);
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
		rc = escore_read(NULL, ES705_VS_INT_OSC_MEASURE_STATUS);

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
static int es705_power_transition(int next_power_state,
				unsigned int set_power_state_cmd)
{
	struct escore_priv *escore = &escore_priv;
	int rc = 0;

	while (next_power_state != escore->escore_power_state) {
		switch (escore->escore_power_state) {
		case ES_SET_POWER_STATE_SLEEP:
			/* Wakeup Chip */
			rc = es705_wakeup(escore);
			if (rc) {
				pr_err("%s(): Wakeup failed: %d\n",
						__func__, rc);
				goto power_transition_exit;
			}
			escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;
			break;
		case ES_SET_POWER_STATE_NORMAL:
			/* Either switch to Sleep or VS Overlay mode */
			if (next_power_state == ES_SET_POWER_STATE_SLEEP)
				escore->escore_power_state =
					ES_SET_POWER_STATE_SLEEP;
			else
				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_OVERLAY;

			/* Set flag to Wait for API Interrupt */
			escore_set_api_intr_wait(escore);

			rc = escore_write(NULL, set_power_state_cmd,
					escore->escore_power_state);
#ifdef CONFIG_SND_SOC_ES_SPI
			int cmd, rsp, sync_rc;
			INC_DISABLE_FW_RECOVERY_USE_CNT(escore);
			/* Chip will not be able to get the interrupt */
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
				pr_err("%s(): Failed to set power state :%d\n",
					__func__, rc);
				escore->escore_power_state =
					ES_SET_POWER_STATE_NORMAL;
				goto power_transition_exit;
			}

			/* Wait for API Interrupt to confirm
			 * the power transition */
			if (escore->pdata->gpioa_gpio != -1) {
				rc = escore_api_intr_wait_completion(escore);
				if (rc)
					goto power_transition_exit;
			}

			/* VS fw download */
			if (escore->escore_power_state ==
					ES_SET_POWER_STATE_VS_OVERLAY) {
				/* wait es705 SBL mode */
				msleep(50);

				rc = escore_vs_load(&escore_priv);
				if (rc) {
					pr_err("%s(): vs fw download fail %d\n",
					       __func__, rc);
					goto power_transition_exit;
				}
			}
			break;
		case ES_SET_POWER_STATE_VS_OVERLAY:
			/* Either switch to VS low power or Normal mode */
			if (next_power_state == ES_SET_POWER_STATE_VS_LOWPWR) {
				/* Start internal oscillator */
				rc = es705_start_int_osc();
				if (rc)
					goto power_transition_exit;

				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_LOWPWR;

			} else {
				/* Set flag to Wait for API Interrupt */
				escore_set_api_intr_wait(escore);

				escore->escore_power_state =
					ES_SET_POWER_STATE_NORMAL;
				escore->mode = STANDARD;
			}

			rc = escore_write(NULL, set_power_state_cmd,
					escore->escore_power_state);
#ifdef CONFIG_SND_SOC_ES_SPI
			INC_DISABLE_FW_RECOVERY_USE_CNT(escore);
			/* Chip will not be able to get the interrupt */
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
				pr_err("%s(): Power state cmd write fail %d\n",
				       __func__, rc);
				escore->escore_power_state =
					ES_SET_POWER_STATE_VS_OVERLAY;
				goto power_transition_exit;
			}


			if (escore->escore_power_state ==
					ES_SET_POWER_STATE_VS_LOWPWR) {
				/* Disable the clocks */
				if (escore->pdata->esxxx_clk_cb)
					escore->pdata->esxxx_clk_cb(0);
				if (escore->pdata->gpioa_gpio != -1)
					escore->cmd_compl_mode =
							ES_CMD_COMP_POLL;
			} else if ((escore->pdata->gpioa_gpio != -1) &&
				(escore->escore_power_state ==
				ES_SET_POWER_STATE_NORMAL)) {
				/* Wait for API Interrupt to confirm
				 * the power transition */
				rc = escore_api_intr_wait_completion(escore);
				if (rc)
					goto power_transition_exit;
				/* Reconfig API Interrupt mode */
				rc = escore_reconfig_api_intr(escore);
				if (rc)
					goto power_transition_exit;
			} else if (escore->escore_power_state ==
				ES_SET_POWER_STATE_NORMAL) {
				/* Add delay before power transition */
				msleep(escore->delay.vs_to_normal);
			}
			break;
		case ES_SET_POWER_STATE_VS_LOWPWR:
			/* Wakeup Chip */
			rc = es705_wakeup(&escore_priv);
			if (rc) {
				dev_err(escore_priv.dev,
						"%s(): es705 wakeup fail %d\n",
						__func__, rc);
				goto power_transition_exit;
			}
			escore_priv.escore_power_state =
				ES_SET_POWER_STATE_VS_OVERLAY;
			break;
		default:
			pr_err("%s(): Unsupported state in es705\n", __func__);
			rc = -EINVAL;
			goto power_transition_exit;
		}
	}
	dev_dbg(escore_priv.dev, "%s(): Power state change successful\n",
		__func__);
power_transition_exit:
	return rc;
}

static int es705_put_power_control_enum(struct snd_kcontrol *kcontrol,
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
		es705_power_state_texts[escore_priv.escore_power_state],
		es705_power_state_texts[value]);

	if (value == 0 || value == ES_SET_POWER_STATE_MP_SLEEP ||
#ifdef CONFIG_SND_SOC_ES_VS_STREAMING
		value == ES_POWER_STATE_VS_STREAMING ||
#endif
		value == ES_SET_POWER_STATE_MP_CMD) {
		dev_err(escore_priv.dev, "%s(): Unsupported state in es705\n",
			__func__);
		goto exit;
	}

	rc = es705_power_transition(value, reg);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): es705_power_transition() failed %d\n",
			__func__, rc);
	}

	escore_pm_put_autosuspend();

exit:
	mutex_unlock(&escore->access_lock);
	return rc;
}

static int es705_get_rx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.flag.rx1_route_enable;
	dev_dbg(escore_priv.dev, "%s(): rx1_route_enable = %d\n",
		__func__, escore_priv.flag.rx1_route_enable);

	return 0;
}

static int es705_put_rx1_route_enable_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int change = 0;

	if (escore_priv.flag.rx1_route_enable !=
			ucontrol->value.integer.value[0]) {
		change = 1;
		escore_priv.flag.rx1_route_enable =
			ucontrol->value.integer.value[0];
	}

	if (change) {
		mutex_lock(&escore_priv.access_lock);
		if (escore_priv.flag.rx1_route_enable)
			escore_pm_get_sync();
		else
			escore_pm_put_autosuspend();
		mutex_unlock(&escore_priv.access_lock);
	}

	dev_dbg(escore_priv.dev, "%s(): rx1_route_enable = %d\n",
		__func__, escore_priv.flag.rx1_route_enable);

	return 0;
}

static int es705_get_power_level_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	u32 es_get_power_level = ES705_GET_POWER_LEVEL << 16;
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

static int es705_put_power_level_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

/* This function must be called with access_lock acquired */
static int es705_change_power_state_normal(struct escore_priv *escore)
{
	int rc = 0;

	pr_debug("%s\n", __func__);

	/* If codec event is detected in CVQ mode, firmware remains in CVQ mode.
	 * At this point CVQ resume triggered by escore_pm_get_sync() (called
	 * from ISR) doesn't do resume and return 0. Hence firmware remains in
	 * CVQ mode. Because of RPM get sync call, RPM status is RPM_ACTIVE.
	 * Now before suspend is executed if any audio usecase is run, because
	 * of RPM_ACTIVE status, runtime resume will not be called and all
	 * audio related commands will be sent in CVQ mode.
	 * This check brings chip into BOSKO mode (Normal Power State) before
	 * sending any audio related commands.
	 */
	if (escore->escore_power_state == ES_SET_POWER_STATE_VS_OVERLAY)
		rc = es705_vs_to_ns_switch(escore);

	return rc;
}

static int es705_get_tx1_route_enable_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.flag.tx1_route_enable;

	dev_dbg(escore_priv.dev, "%s(): tx1_route_enable = %d\n",
			__func__, escore_priv.flag.tx1_route_enable);

	return 0;
}

static int es705_put_tx1_route_enable_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int change = 0;

	if (escore_priv.flag.tx1_route_enable !=
			ucontrol->value.integer.value[0]) {
		change = 1;
		escore_priv.flag.tx1_route_enable =
			ucontrol->value.integer.value[0];
	}

	if (change) {
		mutex_lock(&escore_priv.access_lock);
		if (escore_priv.flag.tx1_route_enable) {
			escore_pm_get_sync();
			if (escore_priv.pm_status == ES_PM_ON)
				es705_change_power_state_normal(&escore_priv);
		} else
			escore_pm_put_autosuspend();
		mutex_unlock(&escore_priv.access_lock);
	}

	dev_dbg(escore_priv.dev, "%s(): tx1_route_enable = %d\n",
			__func__, escore_priv.flag.tx1_route_enable);

	return 0;
}

static int es705_get_rx2_route_enable_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.flag.rx2_route_enable;
	dev_dbg(escore_priv.dev, "%s(): rx2_route_enable = %d\n",
			__func__, escore_priv.flag.rx2_route_enable);

	return 0;
}

static int es705_put_rx2_route_enable_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int change = 0;

	if (escore_priv.flag.rx2_route_enable !=
			ucontrol->value.integer.value[0]) {
		change = 1;
		escore_priv.flag.rx2_route_enable =
			ucontrol->value.integer.value[0];
	}

	if (change) {
		mutex_lock(&escore_priv.access_lock);
		if (escore_priv.flag.rx2_route_enable) {
			escore_pm_get_sync();
			es705_change_power_state_normal(&escore_priv);
		} else
			escore_pm_put_autosuspend();
		mutex_unlock(&escore_priv.access_lock);
	}

	dev_dbg(escore_priv.dev, "%s(): rx2_route_enable = %d\n",
			__func__, escore_priv.flag.rx2_route_enable);

	return 0;
}

static int es705_get_ns_value(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	dev_dbg(escore_priv.dev, "%s(): NS = %d\n",
		__func__, escore_priv.flag.ns);
	ucontrol->value.enumerated.item[0] = escore_priv.flag.ns;

	return 0;
}

static int es705_put_ns_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): NS = %d\n", __func__, value);

	escore_priv.flag.ns = value;

	/* 0 = NS off, 1 = NS on*/
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_NS_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_NS_OFF_PRESET);

	return rc;
}

static int es705_get_sw_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_sw_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): SW = %d\n", __func__, value);

	/* 0 = off, 1 = on*/
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_SW_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_SW_OFF_PRESET);

	return rc;
}

static int es705_get_sts_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_sts_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): STS = %d\n", __func__, value);

	/* 0 = off, 1 = on*/
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_STS_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_STS_OFF_PRESET);

	return rc;
}

static int es705_get_rx_ns_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_rx_ns_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): RX_NS = %d\n", __func__, value);

	/* 0 = off, 1 = on*/
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_RX_NS_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_RX_NS_OFF_PRESET);

	return rc;
}

static int es705_get_wnf_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_wnf_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): WNF = %d\n", __func__, value);

	/* 0 = off, 1 = on */
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_WNF_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_WNF_OFF_PRESET);

	return rc;
}

static int es705_get_bwe_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_bwe_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): BWE = %d\n", __func__, value);

	/* 0 = off, 1 = on */
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_BWE_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_BWE_OFF_PRESET);

	return rc;
}

static int es705_get_avalon_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_avalon_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): Avalon Wind Noise = %d\n",
		__func__, value);

	/* 0 = off, 1 = on */
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_AVALON_WN_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_AVALON_WN_OFF_PRESET);

	return rc;
}

static int es705_get_vbb_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_vbb_value(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): Virtual Bass Boost = %d\n",
		__func__, value);

	/* 0 = off, 1 = on */
	if (value)
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_VBB_ON_PRESET);
	else
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_VBB_OFF_PRESET);

	return rc;
}

static int es705_get_aud_focus(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	dev_dbg(escore_priv.dev, "%s(): Focus = %d\n",
		__func__, escore_priv.flag.af_mode);

	if (escore_priv.mode != AF) {
		pr_err("%s(): Chip not in AF state\n", __func__);
		return -EAGAIN;
	}

	ucontrol->value.enumerated.item[0] = escore_priv.flag.af_mode;

	return 0;
}

static int es705_put_aud_focus(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	int rc = 0;
	dev_dbg(escore_priv.dev, "%s(): Focus = %d\n", __func__, value);

	if (escore_priv.mode != AF) {
		pr_err("%s(): Chip not in AF state\n", __func__);
		return -EAGAIN;
	}

	escore_priv.flag.af_mode = value;

	if (value == ES705_AUD_FOCUS_NARRATOR) {
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_AUD_FOCUS_NARRATOR_PRESET);
	} else if (value == ES705_AUD_FOCUS_SCENE) {
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_AUD_FOCUS_SCENE_PRESET);
	} else if (value == ES705_AUD_FOCUS_NARRATION) {
		rc = escore_write_locked(NULL, ES705_PRESET,
			ES705_AUD_FOCUS_NARRATION_PRESET);
	} else
		rc = escore_write_locked(NULL, ES705_PRESET, 0);

	return rc;
}

/* Get for streming is not avaiable. Tinymix "set" method first executes get
 * and then put method. Thus dummy get method is implemented. */
static int es705_get_streaming_select(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = -1;

	return 0;
}

int es705_set_streaming(struct escore_priv *escore, int value)
{
	u32 resp;
	return escore_cmd_locked(escore,
		es705_streaming_cmds[escore->streamdev.intf] | value, &resp);
}

int es705_remote_route_enable(struct snd_soc_dai *dai)
{
	dev_dbg(escore_priv.dev, "%s():dai->name = %s dai->id = %d\n",
		__func__, dai->name, dai->id);

	switch (dai->id) {
	case ES705_SLIM_1_PB:
		return escore_priv.flag.rx1_route_enable;
	case ES705_SLIM_1_CAP:
		return escore_priv.flag.tx1_route_enable;
	case ES705_SLIM_2_PB:
		return escore_priv.flag.rx2_route_enable;
	default:
		return 0;
	}
}
EXPORT_SYMBOL_GPL(es705_remote_route_enable);

static int es705_put_internal_route(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	return es705_switch_route(ucontrol->value.integer.value[0]);
}

static int es705_get_internal_route(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *es705 = &escore_priv;

	ucontrol->value.integer.value[0] = es705->internal_route_num;

	return 0;
}

static int escore_get_sync_words(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *es705 = &escore_priv;

	ucontrol->value.integer.value[0] = es705->es_sync_words;

	return 0;
}

static int escore_put_sync_words(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	unsigned int value;
	int rc = 0, mode = -1;
	u32 cmd_resp;
	struct escore_priv *escore = &escore_priv;

	value = ucontrol->value.integer.value[0];

	/* Value is not changed */
	if (escore_priv.es_sync_words == value)
		goto exit;

	escore_priv.es_sync_words = value;

	/* non-zero value and Dolby firmware is not downloaded */
	if (value && escore->mode != DOLBY) {
		/* firmware is not requested before */
		if (!escore->dolby) {
			rc = request_firmware(
				(const struct firmware **)&escore->dolby,
				escore->fw_dolby_filename, escore->dev);
			if (rc) {
				pr_err("%s(): request_firmware(%s) fail %d\n",
				__func__, escore->fw_dolby_filename, rc);
				goto exit;
			}
		}
		mode = DOLBY;
	} /* sync_words reset and current firmware is dolby,
	   * set mode to download NS binary */
	else if (!value && escore->mode == DOLBY)
		mode = STANDARD;

	/* If firmware download is required */
	if (mode != -1) {
		INC_DISABLE_FW_RECOVERY_USE_CNT(escore);
		mutex_lock(&escore->access_lock);
		rc = es705_fw_download(escore, mode);
		mutex_unlock(&escore->access_lock);
		DEC_DISABLE_FW_RECOVERY_USE_CNT(escore);
	}

	/* If the firmware is DOLBY, send the sync word write command  */
	if (escore->mode == DOLBY) {
		/* Sending no of sync words */
		rc = escore_cmd(escore, (ES_SET_SYNC_WORDS << 16 |
					escore_priv.es_sync_words),
				&cmd_resp);
		if (rc) {
			dev_err(escore_priv.dev,
					"%s(): Set Sync words fail %d\n",
					__func__, rc);
		}
	}
exit:
	return rc;
}

static int es705_put_internal_rate(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *es705 = &escore_priv;
	const u32 *rate_macro = NULL;
	int rc = 0;
	dev_dbg(es705->dev, "%s:internal_rate  %d ucontrol %d ",
		__func__, (int)es705->internal_rate,
		(int)ucontrol->value.enumerated.item[0]);

	switch (ucontrol->value.enumerated.item[0]) {
	case RATE_NB:
		rate_macro =
			es705_route_config[es705->internal_route_num].nb;
		break;
	case RATE_WB:
		rate_macro =
			es705_route_config[es705->internal_route_num].wb;
		break;
	case RATE_SWB:
		rate_macro =
			es705_route_config[es705->internal_route_num].swb;
		break;
	case RATE_FB:
		rate_macro =
			es705_route_config[es705->internal_route_num].fb;
		break;
	default:
		break;
	}

	if (!rate_macro) {
		dev_err(es705->dev, "%s(): internal rate, %d, out of range\n",
			__func__, ucontrol->value.enumerated.item[0]);
		return -EINVAL;
	}


	rc = escore_write_block(es705, rate_macro);
	es705->internal_rate = ucontrol->value.enumerated.item[0];

	return rc;
}

static int es705_get_internal_rate(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *es705 = &escore_priv;

	ucontrol->value.enumerated.item[0] = es705->internal_rate;
	dev_dbg(es705->dev, "%s():es705->internal_rate = %d ucontrol = %d\n",
		__func__, (int)es705->internal_rate,
		(int)ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es705_put_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];

	rc = escore_write_locked(NULL, reg, value);
	if (rc) {
		dev_err(escore_priv.dev, "%s(): Set Preset fail %d\n",
			__func__, rc);
		return rc;
	}

	escore_priv.preset = value;

	return rc;
}
#ifdef CONFIG_ARCH_MSM
/* Uplink and downlink preset controls begin */
static int es705_put_preset_uplink_array_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_multi_mixer_control *mc =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int count;
	int rc = 0;

	for (count = 0; count < ES705_PRESET_ARRAY_SIZE; count++) {
		escore_priv.preset_uplink_array[count] =
			ucontrol->value.integer.value[count];

		if (ucontrol->value.integer.value[count] == 0)
			break;

		rc = escore_write_locked(NULL, reg,
			escore_priv.preset_uplink_array[count]);
		if (rc) {
			dev_err(escore_priv.dev, "%s(): Set Preset fail %d\n",
				__func__, rc);
			return rc;
		}
		dev_dbg(escore_priv.dev, "%s(): Set Preset value:%d\n",
			__func__, escore_priv.preset_uplink_array[count]);
		usleep_range(10000, 10000);
	}
	return rc;
}

static int es705_get_preset_uplink_array_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	unsigned int count;
	for (count = 0; count < ES705_PRESET_ARRAY_SIZE; count++) {
		ucontrol->value.integer.value[count] =
			escore_priv.preset_uplink_array[count];
	}
	return 0;
}

static int es705_put_preset_downlink_array_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_multi_mixer_control *mc =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int count;
	int rc = 0;

	for (count = 0; count < ES705_PRESET_ARRAY_SIZE; count++) {
		escore_priv.preset_downlink_array[count] =
				ucontrol->value.integer.value[count];

		if (ucontrol->value.integer.value[count] == 0)
			break;

		rc = escore_write_locked(NULL, reg,
			escore_priv.preset_downlink_array[count]);
		if (rc) {
			dev_err(escore_priv.dev, "%s(): Set Preset fail %d\n",
				__func__, rc);
			return rc;
		}
		dev_dbg(escore_priv.dev, "%s(): Set Preset value:%d\n",
			__func__, escore_priv.preset_downlink_array[count]);
		usleep_range(10000, 10000);
	}
	return rc;
}

static int es705_get_preset_downlink_array_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	unsigned int count;
	for (count = 0; count < ES705_PRESET_ARRAY_SIZE; count++) {
		ucontrol->value.integer.value[count] =
				escore_priv.preset_downlink_array[count];
	}
	return 0;
}
#endif

static int es705_get_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.preset;

	return 0;
}
static int es705_get_audio_custom_profile(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int es705_put_audio_custom_profile(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
/*	int index = ucontrol->value.integer.value[0];

	if (index < ES705_CUSTOMER_PROFILE_MAX)
		escore_write_block(&escore_priv,
				  &es705_audio_custom_profiles[index][0]);
*/
	return 0;
}

static int es705_ap_put_tx1_ch_cnt(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	escore_priv.ap_tx1_ch_cnt = ucontrol->value.enumerated.item[0] + 1;
	return 0;
}

static int es705_ap_get_tx1_ch_cnt(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct escore_priv *es705 = &escore_priv;

	ucontrol->value.enumerated.item[0] = es705->ap_tx1_ch_cnt - 1;
	return 0;
}

static const char * const es705_ap_tx1_ch_cnt_texts[] = {
	"One", "Two"
};
static const struct soc_enum es705_ap_tx1_ch_cnt_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es705_ap_tx1_ch_cnt_texts),
			es705_ap_tx1_ch_cnt_texts);

static const struct soc_enum es705_power_state_enum =
	SOC_ENUM_SINGLE(ES705_POWER_STATE, 0,
			ARRAY_SIZE(es705_power_state_texts),
			es705_power_state_texts);

/* generic gain translation */
static int es705_index_to_gain(int min, int step, int index)
{
	return	min + (step * index);
}
static int es705_gain_to_index(int min, int step, int gain)
{
	return	(gain - min) / step;
}

/* dereverb gain */
static int es705_put_dereverb_gain_value(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	if (ucontrol->value.integer.value[0] <= 12) {
		value = es705_index_to_gain(-12, 1,
				  ucontrol->value.integer.value[0]);
		rc = escore_write_locked(NULL, reg, value);
	}

	return rc;
}

static int es705_get_dereverb_gain_value(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = escore_read_locked(NULL, reg);
	ucontrol->value.integer.value[0] = es705_gain_to_index(-12, 1, value);

	return 0;
}

/* bwe high band gain */
static int es705_put_bwe_high_band_gain_value(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	if (ucontrol->value.integer.value[0] <= 30) {
		value = es705_index_to_gain(-10, 1,
				  ucontrol->value.integer.value[0]);
		rc = escore_write_locked(NULL, reg, value);
	}

	return 0;
}

static int es705_get_bwe_high_band_gain_value(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;

	value = escore_read_locked(NULL, reg);
	ucontrol->value.integer.value[0] = es705_gain_to_index(-10, 1, value);

	return 0;
}

static int sde_get_control_value(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int value;

	if (escore_priv.mode != AF) {
		pr_err("%s(): Chip not in AF state\n", __func__);
		return -EINVAL;
	}

	value = escore_read_locked(NULL, reg);
	if (value < 0) {
		dev_err(escore_priv.dev, "%s(): %s failed %d\n", __func__,
			kcontrol->id.name, value);
		return value;
	}
	ucontrol->value.integer.value[0] =  value;

	return 0;
}

static const char * const es705_mic_config_texts[] = {
	"CT Multi-mic", "FT Multi-mic", "DV 1-mic", "EXT 1-mic", "BT 1-mic",
	"CT ASR Multi-mic", "FT ASR Multi-mic", "EXT ASR 1-mic", "FT ASR 1-mic",
};
static const struct soc_enum es705_mic_config_enum =
	SOC_ENUM_SINGLE(ES705_MIC_CONFIG, 0,
			ARRAY_SIZE(es705_mic_config_texts),
			es705_mic_config_texts);

static const char * const es705_aec_mode_texts[] = {
	"Off", "On", "rsvrd2", "rsvrd3", "rsvrd4", "On half-duplex"
};
static const struct soc_enum es705_aec_mode_enum =
	SOC_ENUM_SINGLE(ES705_AEC_MODE, 0, ARRAY_SIZE(es705_aec_mode_texts),
			es705_aec_mode_texts);

static const char * const es705_internal_rate_text[] = {
	"NB", "WB", "SWB", "FB"
};
static const struct soc_enum es705_internal_rate_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es705_internal_rate_text),
			es705_internal_rate_text);

static const char * const es705_streaming_mode_texts[] = {
	"CVQ", "Non-CVQ",
};
static const struct soc_enum es705_streaming_mode_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es705_streaming_mode_texts),
			es705_streaming_mode_texts);

static const char * const es705_off_on_texts[] = {
	"Off", "On"
};
static const char * const es80X_duo_mode_texts[] = {
	"Narrator", "Scene"
};
static const char * const es705_audio_focus_texts[] = {
	"disabled", "Narrator", "Scene", "Narration"
};
static const struct soc_enum es705_veq_enable_enum =
	SOC_ENUM_SINGLE(ES705_VEQ_ENABLE, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_dereverb_enable_enum =
	SOC_ENUM_SINGLE(ES705_DEREVERB_ENABLE, 0,
			ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_bwe_enable_enum =
	SOC_ENUM_SINGLE(ES705_BWE_ENABLE, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_bwe_post_eq_enable_enum =
	SOC_ENUM_SINGLE(ES705_BWE_POST_EQ_ENABLE, 0,
			ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_algo_processing_enable_enum =
	SOC_ENUM_SINGLE(ES705_ALGO_PROCESSING, 0,
			ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_ns_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_audio_focus_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_audio_focus_texts),
			es705_audio_focus_texts);
static const struct soc_enum es705_rx_enable_enum =
	SOC_ENUM_SINGLE(ES705_RX_ENABLE, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es80X_af_duo_mode_enum =
	SOC_ENUM_SINGLE(ES80X_AF_DUO_MODE, 0,
			ARRAY_SIZE(es80X_duo_mode_texts),
			es80X_duo_mode_texts);
static const struct soc_enum es80X_enable_sde_mode_enum =
	SOC_ENUM_SINGLE(ES80X_ENABLE_SDE_MODE, 0,
			ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_sw_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_sts_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_rx_ns_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_wnf_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_bwe_preset_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_avalon_wn_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);
static const struct soc_enum es705_vbb_enable_enum =
	SOC_ENUM_SINGLE(ES705_PRESET, 0, ARRAY_SIZE(es705_off_on_texts),
			es705_off_on_texts);

static int es705_get_rdb_size(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] =
				escore_priv.datablock_dev.rdb_read_count;
	return 0;
}

static int es705_get_event_status(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&escore_priv.escore_event_type_mutex);

	ucontrol->value.enumerated.item[0] = escore_priv.escore_event_type;

	/* Reset the event status after read */
	escore_priv.escore_event_type = ES_NO_EVENT;

	mutex_unlock(&escore_priv.escore_event_type_mutex);

	return 0;
}

static const char * const es705_vs_wakeup_keyword_texts[] = {
	"Default", "One", "Two", "Three", "Four"
};
static const struct soc_enum es705_vs_wakeup_keyword_enum =
	SOC_ENUM_SINGLE(ES705_VOICE_SENSE_SET_KEYWORD, 0,
			ARRAY_SIZE(es705_vs_wakeup_keyword_texts),
			es705_vs_wakeup_keyword_texts);

static const char * const es705_vs_event_texts[] = {
	"No Event", "Codec Event", "VS Keyword Event",
};
static const struct soc_enum es705_vs_event_enum =
	SOC_ENUM_SINGLE(ES705_VOICE_SENSE_EVENT, 0,
			ARRAY_SIZE(es705_vs_event_texts),
			es705_vs_event_texts);

static const char * const es705_vs_training_status_texts[] = {
	"busy", "Success", "Utterance Long", "Utterance Short",
	"Verification Failed", "Failed Bad length", "Noise", "Level too low",
};
static const struct soc_enum es705_vs_training_status_enum =
	SOC_ENUM_SINGLE(ES705_VOICE_SENSE_TRAINING_STATUS, 0,
			ARRAY_SIZE(es705_vs_training_status_texts),
			es705_vs_training_status_texts);

static const char * const es705_vs_training_record_texts[] = {
	"Start_0", "Start_1", "Start_2",
};


static const char * const es705_vs_stored_keyword_texts[] = {
	"Put", "Get", "Clear"
};

static const struct soc_enum es705_vs_stored_keyword_enum =
	SOC_ENUM_SINGLE(ES705_VS_STORED_KEYWORD, 0,
			ARRAY_SIZE(es705_vs_stored_keyword_texts),
			es705_vs_stored_keyword_texts);

static const struct soc_enum es705_vs_training_record_enum =
	SOC_ENUM_SINGLE(ES705_VOICE_SENSE_TRAINING_RECORD, 0,
			ARRAY_SIZE(es705_vs_training_record_texts),
			es705_vs_training_record_texts);

static const char * const es705_vs_training_mode_texts[] = {
	"Detect Keyword", "N/A", "Train User-defined Keyword",
};

static const struct soc_enum es705_vs_training_mode_enum =
	SOC_ENUM_SINGLE(ES705_VOICE_SENSE_TRAINING_MODE, 0,
			ARRAY_SIZE(es705_vs_training_mode_texts),
			es705_vs_training_mode_texts);

static const char * const es705_power_level_texts[] = {
	"0 [Min]", "1", "2", "3", "4", "5", "6 [Max, Def]"
};

static const struct soc_enum es705_power_level_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(es705_power_level_texts),
			es705_power_level_texts);

static const char * const es705_runtime_pm_texts[] = {
	"Disable", "Enable"
};

static const struct soc_enum es705_runtime_pm_enum =
	SOC_ENUM_SINGLE(ES705_RUNTIME_PM, 0,
			ARRAY_SIZE(es705_runtime_pm_texts),
			es705_runtime_pm_texts);

static const char * const es705_bargein_state[] = {
	"Enable", "Disable", "CVQ" , "PB", "CVQ_PB"
};

static const char * const es705_chip_mode[] = {
	"BARGEIN", "TRAINING", "VOIP"
};

static const struct soc_enum es705_bargein_state_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(es705_bargein_state),
			es705_bargein_state);

static const struct soc_enum es705_chip_mode_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(es705_chip_mode),
			es705_chip_mode);

static struct snd_kcontrol_new es705_digital_ext_snd_controls[] = {
	SOC_ENUM_EXT("ES705 Power State", es705_power_state_enum,
		     es705_get_power_control_enum,
		     es705_put_power_control_enum),
	SOC_ENUM_EXT("Power Level", es705_power_level_enum,
				es705_get_power_level_value,
				es705_put_power_level_value),
	SOC_SINGLE_EXT("ES705 RX1 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es705_get_rx1_route_enable_value,
		       es705_put_rx1_route_enable_value),
	SOC_SINGLE_EXT("ES705 TX1 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es705_get_tx1_route_enable_value,
		       es705_put_tx1_route_enable_value),
	SOC_SINGLE_EXT("ES705 RX2 Enable", SND_SOC_NOPM, 0, 1, 0,
		       es705_get_rx2_route_enable_value,
		       es705_put_rx2_route_enable_value),
	SOC_ENUM_EXT("Mic Config", es705_mic_config_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_ENUM_EXT("AEC Mode", es705_aec_mode_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_ENUM_EXT("VEQ Enable", es705_veq_enable_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_ENUM_EXT("Dereverb Enable", es705_dereverb_enable_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_SINGLE_EXT("Dereverb Gain",
		       ES705_DEREVERB_GAIN, 0, 100, 0,
		       es705_get_dereverb_gain_value,
		       es705_put_dereverb_gain_value),
	SOC_ENUM_EXT("BWE Enable", es705_bwe_enable_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_SINGLE_EXT("BWE High Band Gain",
		       ES705_BWE_HIGH_BAND_GAIN, 0, 100, 0,
		       es705_get_bwe_high_band_gain_value,
		       es705_put_bwe_high_band_gain_value),
	SOC_ENUM_EXT("BWE Post EQ Enable", es705_bwe_post_eq_enable_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_SINGLE_EXT("SLIMbus Link Multi Channel",
		       ES705_SLIMBUS_LINK_MULTI_CHANNEL, 0, 65535, 0,
		       es705_get_control_value, es705_put_control_value),
	SOC_ENUM_EXT("Algorithm Processing", es705_algo_processing_enable_enum,
		     es705_get_control_enum, es705_put_control_enum),
	SOC_SINGLE_EXT("Internal Route",
		       SND_SOC_NOPM, 0, 100, 0, es705_get_internal_route,
		       es705_put_internal_route),
	SOC_ENUM_EXT("Internal Rate", es705_internal_rate_enum,
		      es705_get_internal_rate,
		      es705_put_internal_rate),
#ifdef CONFIG_ARCH_MSM
	SOC_SINGLE_MULTI_EXT("Preset Array Uplink",
		      ES705_PRESET, 0, 65535, 0, ES705_PRESET_ARRAY_SIZE,
		      es705_get_preset_uplink_array_value,
		      es705_put_preset_uplink_array_value),
	SOC_SINGLE_MULTI_EXT("Preset Array Downlink",
		      ES705_PRESET, 0, 65535, 0, ES705_PRESET_ARRAY_SIZE,
		      es705_get_preset_downlink_array_value,
		      es705_put_preset_downlink_array_value),
#endif
	SOC_SINGLE_EXT("Preset",
		       ES705_PRESET, 0, 65535, 0, es705_get_preset_value,
		       es705_put_preset_value),
	SOC_SINGLE_EXT("Audio Custom Profile",
		       SND_SOC_NOPM, 0, 100, 0, es705_get_audio_custom_profile,
		       es705_put_audio_custom_profile),
	SOC_ENUM_EXT("ES705-AP Tx Channels", es705_ap_tx1_ch_cnt_enum,
		     es705_ap_get_tx1_ch_cnt, es705_ap_put_tx1_ch_cnt),
	SOC_ENUM_EXT("Voice Sense Status",
		     es705_vs_event_enum,
		     escore_vs_get_control_enum, NULL),
	SOC_ENUM_EXT("Voice Sense Training Mode",
			 es705_vs_training_mode_enum,
			 escore_vs_get_control_enum,
			 escore_vs_put_control_enum),
	SOC_ENUM_EXT("Voice Sense Training Status",
		     es705_vs_training_status_enum,
		     escore_vs_get_control_enum, NULL),
	SOC_SINGLE_EXT("Voice Sense Training Model Length",
			ES705_VOICE_SENSE_TRAINING_MODEL_LENGTH, 0, 75, 0,
			escore_vs_get_control_value,
			NULL),
	SOC_ENUM_EXT("Voice Sense Training Record",
		     es705_vs_training_record_enum,
		     NULL, escore_vs_put_control_enum),
	SOC_SINGLE_EXT("Voice Sense Detect Sensitivity",
			ES705_VOICE_SENSE_DETECTION_SENSITIVITY, 0, 10, 0,
			escore_vs_get_control_value,
			escore_vs_put_control_value),
	SOC_SINGLE_EXT("Voice Activity Detect Sensitivity",
			ES705_VOICE_ACTIVITY_DETECTION_SENSITIVITY, 0, 10, 0,
			escore_vs_get_control_value,
			escore_vs_put_control_value),
	SOC_SINGLE_EXT("Continuous Voice Sense Preset",
		       ES705_CVS_PRESET, 0, 65535, 0,
		       escore_get_cvs_preset_value,
		       escore_put_cvs_preset_value),
	SOC_ENUM_EXT("Runtime PM", es705_runtime_pm_enum,
		     escore_get_runtime_pm_enum,
		     escore_put_runtime_pm_enum),
	SOC_ENUM_EXT("Noise Suppression", es705_ns_enable_enum,
		       es705_get_ns_value,
		       es705_put_ns_value),
	SOC_SINGLE_EXT("Audio Focus Enable", SND_SOC_NOPM, 0, 1, 0,
			escore_get_af_status, escore_put_af_status),
	SOC_ENUM_EXT("Audio Focus Mode", es705_audio_focus_enum,
		       es705_get_aud_focus,
		       es705_put_aud_focus),
	SOC_SINGLE_EXT("Enable/Disable Streaming PATH/Endpoint",
		       ES705_FE_STREAMING, 0, 65535, 0,
		       es705_get_streaming_select,
		       es705_put_control_value),
	SOC_ENUM_EXT("RX Enable", es705_rx_enable_enum,
		       es705_get_control_enum,
		       es705_put_control_enum),
	SOC_ENUM_EXT("Stereo Widening", es705_sw_enable_enum,
		       es705_get_sw_value,
		       es705_put_sw_value),
	SOC_ENUM_EXT("Speech Time Stretching", es705_sts_enable_enum,
			   es705_get_sts_value,
			   es705_put_sts_value),
	SOC_ENUM_EXT("RX Noise Suppression", es705_rx_ns_enable_enum,
			   es705_get_rx_ns_value,
			   es705_put_rx_ns_value),
	SOC_ENUM_EXT("Wind Noise Filter", es705_wnf_enable_enum,
			   es705_get_wnf_value,
			   es705_put_wnf_value),
	SOC_ENUM_EXT("BWE Preset", es705_bwe_preset_enable_enum,
			   es705_get_bwe_value,
			   es705_put_bwe_value),
	SOC_ENUM_EXT("AVALON Wind Noise", es705_avalon_wn_enable_enum,
			   es705_get_avalon_value,
			   es705_put_avalon_value),
	SOC_ENUM_EXT("Virtual Bass Boost", es705_vbb_enable_enum,
			   es705_get_vbb_value,
			   es705_put_vbb_value),
	SOC_SINGLE_EXT("Get RDB data size",
			   SND_SOC_NOPM, 0, 65535, 0,
			   es705_get_rdb_size, NULL),
	SOC_SINGLE_EXT("ES705 Get Event Status",
			   SND_SOC_NOPM, 0, 65535, 0,
			   es705_get_event_status, NULL),
	SOC_SINGLE_EXT("CVQ Activate Keywords",
			SND_SOC_NOPM, 0, 2047, 0,
			escore_get_vs_activate_keyword,
			escore_put_vs_activate_keyword),
	SOC_SINGLE_EXT("CVQ Sleep",
		       SND_SOC_NOPM, 0, 1, 0,
		       escore_get_vs_sleep,
		       escore_put_vs_sleep),
	SOC_ENUM_EXT("Streaming Mode", es705_streaming_mode_enum,
			   escore_get_streaming_mode,
			   escore_put_streaming_mode),
	SOC_SINGLE_EXT("VS keyword length",
		       SND_SOC_NOPM, 0, 65535, 0,
		       escore_get_vs_keyword_length,
		       escore_put_vs_keyword_length),
	SOC_SINGLE_EXT("KW Overrun Error",
		       0, 0, 65535, 0, escore_get_keyword_overrun,
		       NULL),
	SOC_ENUM_EXT("AF Duo Mode", es80X_af_duo_mode_enum,
		       es705_get_control_enum,
		       es705_put_control_enum),
	SOC_ENUM_EXT("SDE Mode Enable", es80X_enable_sde_mode_enum,
			es705_get_control_enum,
			sde_put_control_enum),
	SOC_SINGLE_EXT("SDE Bearing Value", ES80X_SDE_BEARING_VALUE, 0, 360, 0,
			sde_get_control_value,
			NULL),
	SOC_SINGLE_EXT("SDE Salience Value", ES80X_SDE_SALIENCE_VALUE, 0, 32767,
			0, sde_get_control_value,
			NULL),
	SOC_SINGLE_EXT("SDE RMS Estimate Value", ES80X_SDE_RMS_ESTIMATE_VALUE,
			0, 32767, 0, sde_get_control_value,
			NULL),
	SOC_SINGLE_EXT("Dolby Sync Count",
			SND_SOC_NOPM, 0, 65535, 0, escore_get_sync_words,
			escore_put_sync_words),
	SOC_SINGLE_EXT("Chip Enable",
		       SND_SOC_NOPM, 0, 1, 0,
		       es705_get_chip_enable,
		       es705_put_chip_enable),
	SOC_ENUM_EXT("BargeIn State", es705_bargein_state_enum,
				es705_get_bargein_state,
				es705_put_bargein_state),
	SOC_ENUM_EXT("Chip Mode", es705_chip_mode_enum,
				es705_get_chip_mode,
				es705_put_chip_mode),
	SOC_SINGLE_EXT("CVQ Start",
		       SND_SOC_NOPM, 0, 1, 0,
		       escore_get_cvq_start,
		       escore_put_cvq_start),
	SOC_SINGLE_EXT("CVQ Stop",
		       SND_SOC_NOPM, 0, 1, 0,
		       escore_get_cvq_stop,
		       escore_put_cvq_stop),
};

static int es705_slim_set_channel_map(struct snd_soc_dai *dai,
				      unsigned int tx_num,
				      unsigned int *tx_slot,
				      unsigned int rx_num,
				      unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = &escore_priv;
	int id = dai->id;
	int i;
	int rc = 0;

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	if (id == ES_SLIM_1_PB ||
	    id == ES_SLIM_2_PB ||
	    id == ES_SLIM_3_PB) {
		escore->slim_dai_data[DAI_INDEX(id)].ch_tot = rx_num;
		escore->slim_dai_data[DAI_INDEX(id)].ch_act = 0;
		for (i = 0; i < rx_num; i++)
			escore->slim_dai_data[DAI_INDEX(id)].ch_num[i] =
				rx_slot[i];
	} else if (id == ES_SLIM_1_CAP ||
		 id == ES_SLIM_2_CAP ||
		 id == ES_SLIM_3_CAP) {
		escore->slim_dai_data[DAI_INDEX(id)].ch_tot = tx_num;
		escore->slim_dai_data[DAI_INDEX(id)].ch_act = 0;
		for (i = 0; i < tx_num; i++)
			escore->slim_dai_data[DAI_INDEX(id)].ch_num[i] =
				tx_slot[i];
	}

	return rc;
}

#if defined(CONFIG_ARCH_MSM)
int es705_slim_get_channel_map(struct snd_soc_dai *dai,
			       unsigned int *tx_num, unsigned int *tx_slot,
			       unsigned int *rx_num, unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = &escore_priv;
	struct escore_slim_ch *rx = escore->slim_rx;
	struct escore_slim_ch *tx = escore->slim_tx;
	int id = dai->id;
	int i;
	int rc = 0;

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	if (id == ES_SLIM_1_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_1_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_2_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_2_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_3_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_3_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_1_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_1_CAP_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_2_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_2_CAP_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_3_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_3_CAP_OFFSET + i].ch_num;
	}

	return rc;
}
#endif

int remote_esxxx_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

int remote_esxxx_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

int es705_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

int es705_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct escore_priv *escore = &escore_priv;
	mutex_lock(&escore->access_lock);

	if (escore->chip_mode == BARGEIN) {
		if (escore->bargein_state == CVQ)
			escore->bargein_state = CVQ_PB;
		else
			escore->bargein_state = PB;
	}
	mutex_unlock(&escore->access_lock);
	return 0;
}

void es705_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct escore_priv *escore = &escore_priv;
	mutex_lock(&escore->access_lock);
	if (escore->chip_mode == BARGEIN && escore->bargein_state != Disable) {
		if (escore->bargein_state == CVQ_PB)
			escore->bargein_state = CVQ;
		else
			escore->bargein_state = Enable;
	}
	mutex_unlock(&escore->access_lock);
}

int remote_add_codec_control(void *ptr)
{
	int rc;
	struct snd_soc_codec *codec = ptr;
	struct escore_priv *escore = &escore_priv;

	wait_for_completion(&escore->fw_download);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	rc = snd_soc_add_codec_controls(codec, es705_digital_ext_snd_controls,
				ARRAY_SIZE(es705_digital_ext_snd_controls));
#else
	rc = snd_soc_add_controls(codec, es705_digital_ext_snd_controls,
				ARRAY_SIZE(es705_digital_ext_snd_controls));
#endif
	if (rc)
		dev_err(codec->dev,
			"%s(): es705_digital_ext_snd_controls fail %d\n",
			__func__, rc);
	return rc;
}

int es705_remote_add_codec_controls(struct snd_soc_codec *codec)
{
	int rc = 0;
	static struct task_struct *add_kcontrol_thread;

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

irqreturn_t es705_cmd_completion_isr(int irq, void *data)
{
	struct escore_priv *escore = (struct escore_priv *)data;

	BUG_ON(!escore);

	pr_debug("%s(): API Interrupt received\n", __func__);
	/* Complete if expected */
	if (escore->wait_api_intr) {
		pr_debug("%s(): API Interrupt completion.\n",
				__func__);
		escore->wait_api_intr = 0;
		complete(&escore->cmd_compl);
	}
	return IRQ_HANDLED;
}

irqreturn_t es705_irq_work(int irq, void *data)
{
	struct escore_priv *escore = (struct escore_priv *)data;
	int rc;
	u32 event_type, cmd = 0;
	/*
	 * Getting interrupt while gpio_reset during
	 * fw download. During fw_download mutex access
	 * lock is not free, that is needed by this function.
	 * After fw download it continue to the same call
	 * and chip will not get the response for that
	 * interrupt. So, Avoid the interrupt if
	 * chip is in SBL mode
	 */
	mutex_lock(&escore->gpio_reset_mutex);
	if (escore->mode == SBL) {
		pr_debug("chip is in SBL mode\n");
		mutex_unlock(&escore->gpio_reset_mutex);
		return IRQ_HANDLED;
	}

	mutex_unlock(&escore->gpio_reset_mutex);
	/* Delay required for firmware to be ready in case of CVQ mode */
	msleep(50);

	if (!escore) {
		pr_err("%s(): Invalid IRQ data\n", __func__);
		return IRQ_HANDLED;
	}

	escore->intr_recvd = 1;

repeat:
	rc = wait_event_interruptible(escore->irq_waitq,
			(escore->dev->power.is_suspended != true));
	if (unlikely(rc == -ERESTARTSYS)) {
		pr_err("%s(): Signal received, try again\n", __func__);
		goto repeat;
	}

	mutex_lock(&escore->access_lock);
	INC_DISABLE_FW_RECOVERY_USE_CNT(escore);

	rc = escore_pm_get_sync();
	if (rc < 0) {
		dev_err(escore_priv.dev, "%s(): pm_get_sync() failed :%d\n",
								__func__, rc);
		mutex_unlock(&escore->access_lock);
		goto err_pm_get_sync;
	}
	/* Enable the clocks if not enabled */
	if (escore_priv.pdata->esxxx_clk_cb) {
		escore_priv.pdata->esxxx_clk_cb(1);
		/* Setup for clock stabilization delay */
		msleep(ES_PM_CLOCK_STABILIZATION);
	}

	cmd = ES_GET_EVENT << 16;
	rc = escore_cmd_nopm(escore, cmd, &event_type);
	if (rc < 0) {
		pr_err("%s(): Error reading IRQ event: %d\n", __func__, rc);
		mutex_unlock(&escore->access_lock);
		goto irq_exit;
	}
	event_type &= ES_MASK_INTR_EVENT;
	mutex_lock(&escore->escore_event_type_mutex);
	escore->escore_event_type = event_type;
	mutex_unlock(&escore->escore_event_type_mutex);
	mutex_unlock(&escore->access_lock);

	if (event_type != ES_NO_EVENT) {
		pr_debug("%s(): Notify subscribers about 0x%04x event\n",
				__func__, event_type);
		blocking_notifier_call_chain(escore->irq_notifier_list,
				event_type, escore);
	}

irq_exit:
	escore_pm_put_autosuspend();
err_pm_get_sync:
	mutex_lock(&escore->access_lock);
	DEC_DISABLE_FW_RECOVERY_USE_CNT(escore);
	if (rc < 0) {
		rc = escore_fw_recovery(escore, FORCED_FW_RECOVERY_ON);
		if (rc < 0)
			dev_err(escore->dev,
					"%s() Firmware recovery failed %d\n",
					__func__, rc);
	}

	mutex_unlock(&escore->access_lock);
	escore->intr_recvd = 0;

	return IRQ_HANDLED;
}

static BLOCKING_NOTIFIER_HEAD(es705_irq_notifier_list);

static struct esxxx_platform_data *es705_populate_dt_pdata(struct device *dev)
{
	struct esxxx_platform_data *pdata;
	u32 val = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "%s(): platform data allocation failed\n",
			__func__);
		goto err;
	}
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "%s(): get reset_gpio failed\n", __func__);
		goto dt_populate_err;
	}
	dev_dbg(dev, "%s(): reset gpio %d\n", __func__, pdata->reset_gpio);

#ifdef CONFIG_SND_SOC_ES705_EXTCLK_OVER_GPIO
	pdata->extclk_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,extclk-gpio", 0);
	if (pdata->extclk_gpio < 0) {
		dev_err(dev, "%s(): get extclk_gpio failed\n", __func__);
		goto dt_populate_err;
	}
	dev_dbg(dev, "%s(): extclk gpio %d\n", __func__, pdata->extclk_gpio);
#endif

#if defined(CONFIG_SND_SOC_ES_UART) || \
	defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_UART) || \
	defined(CONFIG_SND_SOC_ES_WAKEUP_UART)

	if (of_property_read_u32(dev->of_node, "adnc,ext_clk_rate", &val)) {
		dev_err(dev, "%s(): Default ext_clk_index = %d \n",
				__func__, ES_EXT_CLK_19200KHZ);
		pdata->ext_clk_rate = ES_EXT_CLK_19200KHZ;
	} else {
		dev_dbg(dev, "%s(): ext_clk_index = %d\n",
				__func__, val);
		pdata->ext_clk_rate = val;
	}

#endif

	pdata->esxxx_clk_cb = NULL;

/* API Interrupt registration */
#ifdef CONFIG_SND_SOC_ES_GPIO_A
	dev_dbg(dev, "%s(): gpioa configured\n", __func__);
	pdata->gpioa_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,gpioa-gpio", 0);
	if (pdata->gpioa_gpio < 0) {
		dev_err(dev, "%s(): get gpioa_gpio failed\n", __func__);
		pdata->gpioa_gpio = -1;
	}
#else
	pdata->gpioa_gpio = -1;
#endif
	dev_dbg(dev, "%s(): gpioa gpio %d\n", __func__, pdata->gpioa_gpio);

	pdata->gpiob_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,gpiob-gpio", 0);
	if (pdata->gpiob_gpio < 0) {
		dev_err(dev, "%s(): get gpiob_gpio failed\n", __func__);
		pdata->gpiob_gpio = -1;
	}
	dev_dbg(dev, "%s(): gpiob gpio %d\n", __func__, pdata->gpiob_gpio);

	pdata->wakeup_gpio = of_get_named_gpio(dev->of_node,
					     "adnc,wakeup-gpio", 0);
	if (pdata->wakeup_gpio < 0) {
		dev_err(dev, "%s(): get wakeup_gpio failed\n", __func__);
		pdata->wakeup_gpio = -1;
	}
	dev_dbg(dev, "%s(): wakeup gpio %d\n", __func__, pdata->wakeup_gpio);

	of_property_read_u32(dev->of_node, "adnc,enable_hs_uart_intf", &val);
	if (val == 1)
		pdata->enable_hs_uart_intf = true;
	else
		pdata->enable_hs_uart_intf = false;

	if (of_property_read_u32(dev->of_node, "adnc,gpio_b_irq_type", &val)) {
		dev_dbg(dev, "%s(): Default gpio_b_irq_type = %d\n",
					__func__, ES_RISING_EDGE);
		pdata->gpio_b_irq_type = ES_RISING_EDGE;
	} else {
		dev_dbg(dev, "%s(): gpio_b_irq_type = %d\n",
					__func__, val);
		pdata->gpio_b_irq_type = val;
	}

	return pdata;

dt_populate_err:
	devm_kfree(dev, pdata);
err:
	return NULL;
}

static int es705_enable_ext_clk(int enable)
{
	int rc = 0;

	if (enable) {
		rc = clk_prepare_enable(escore_priv.esxxx_ext_clk);
			if (rc)
				dev_err(escore_priv.dev, "clk_prepare_enable failed: %d\n",
						rc);
	} else {
		clk_disable_unprepare(escore_priv.esxxx_ext_clk);
	}
	return rc;
}

int es705_core_probe(struct device *dev)
{
	struct esxxx_platform_data *pdata;
	int rc = 0;
#ifdef CONFIG_SND_SOC_ES817_BASE
	const char *fw_filename = NULL;
	const char *vs_filename = "audience/es817/base/audience-es817-vs.bin";
#elif defined(CONFIG_SND_SOC_ES817_PLUS)
	const char *fw_filename = NULL;
	const char *vs_filename = "audience/es817/plus/audience-es817-vs.bin";
#elif defined(CONFIG_SND_SOC_ES817_PREMIUM)
	const char *fw_filename = \
		"audience/es817/premium/audience-es817-ns.bin";
	const char *vs_filename = \
		"audience/es817/premium/audience-es817-vs.bin";
#endif
	const char *fw_af_filename = NULL;
	const char *fw_dolby_filename = NULL;

#ifndef CONFIG_SND_SOC_ES_ASYNC_FW_LOAD
	struct escore_voice_sense *voice_sense;
#endif /* CONFIG_SND_SOC_ES_ASYNC_FW_LOAD */

	if (dev->of_node) {
		dev_info(dev, "Platform data from device tree\n");
		pdata = es705_populate_dt_pdata(dev);
		dev->platform_data = pdata;
	} else {
		dev_info(dev, "Platform data from board file\n");
		pdata = dev->platform_data;
	}

	if (pdata == NULL) {
		dev_err(dev, "%s(): pdata is NULL", __func__);
		rc = -EIO;
		goto pdata_error;
	}

	escore_priv.dev = dev;
	escore_priv.pdata = pdata;
	escore_priv.fw_requested = 0;
	if (pdata->esxxx_clk_cb)
		pdata->esxxx_clk_cb(1);

	escore_priv.flag.profile_enable = ES_PROFILING_OFF;
	escore_priv.boot_ops.bootup = es705_bootup;
	escore_priv.dai = es705_dai;
	escore_priv.dai_nr = ES_NUM_CODEC_DAIS;
	escore_priv.api_addr_max = ES_API_ADDR_MAX;
	escore_priv.api_access = es705_api_access;
	escore_priv.irq_notifier_list = &es705_irq_notifier_list;
	escore_priv.fw_af_filename = fw_af_filename;
	escore_priv.fw_dolby_filename = fw_dolby_filename;
#if defined(CONFIG_SND_SOC_ES_VS)
	escore_priv.vs_filename = vs_filename;
#endif /*CONFIG_SND_SOC_ES_VS*/

/*	escore_priv.dev_rdb = es705_dev_rdb,
	escore_priv.dev_wdb = es705_dev_wdb,
*/
/*	escore_priv.reg_cache = a300_reg_cache;*/

#if (defined(CONFIG_SND_SOC_ES_I2S) && !defined(CONFIG_SND_SOC_ES8XX_AS_CODEC))
	escore_priv.i2s_dai_ops.startup = es705_i2s_startup;
	escore_priv.i2s_dai_ops.shutdown = es705_i2s_shutdown;
#endif

#if defined(CONFIG_SND_SOC_ES8XX_AS_CODEC)
	escore_priv.flag.is_codec = 1;
	escore_priv.soc_codec_dev_escore = &soc_codec_dev_esxxx;
#else
	escore_priv.flag.is_codec = 0;
#endif
	escore_priv.non_vs_sleep_state = ES_SET_POWER_STATE_SLEEP;
	escore_priv.cmd_compl_mode = ES_CMD_COMP_POLL;
	escore_priv.wait_api_intr = 0;
	escore_priv.chip_enable = 0;
	if (escore_priv.pri_intf == ES_SLIM_INTF) {
		escore_priv.slim_rx = slim_rx;
		escore_priv.slim_tx = slim_tx;
		escore_priv.slim_dai_data = slim_dai_data;
		escore_priv.slim_setup = es705_slim_setup;

		escore_priv.slim_rx_ports = ES_SLIM_RX_PORTS;
		escore_priv.slim_tx_ports = ES_SLIM_TX_PORTS;
		escore_priv.codec_slim_dais = ES_NUM_CODEC_SLIM_DAIS;

		escore_priv.slim_tx_port_to_ch_map = es705_slim_tx_port_to_ch;
		escore_priv.slim_rx_port_to_ch_map = es705_slim_rx_port_to_ch;

#if defined(CONFIG_ARCH_MSM)
		escore_priv.slim_dai_ops.get_channel_map =
			es705_slim_get_channel_map;
#endif
		escore_priv.slim_dai_ops.set_channel_map =
			es705_slim_set_channel_map;

#if defined(CONFIG_SND_SOC_ES_SLIM)
		/* Initialization of be_id goes here if required */
		escore_priv.slim_be_id = es705_slim_be_id;
#else
		escore_priv.slim_be_id = NULL;
#endif

		/* Initialization of _remote_ routines goes here if required */
		escore_priv.remote_cfg_slim_rx = NULL;
		escore_priv.remote_cfg_slim_tx = NULL;
		escore_priv.remote_close_slim_rx = NULL;
		escore_priv.remote_close_slim_tx = NULL;

		/* Set local_slim_ch_cfg to 0 for digital only codec */
		escore_priv.flag.local_slim_ch_cfg = 0;
		escore_priv.channel_dir = es705_channel_dir;

		escore_priv.sleep  = es705_slim_sleep;
		escore_priv.wakeup = es705_slim_wakeup;
		escore_priv.slim_setup(&escore_priv);

/*	}	else if (escore_priv.intf == ES_I2C_INTF) {

		escore_priv.sleep  = es705_i2c_sleep;
		escore_priv.wakeup = es705_i2c_wakeup;
*/
	}
#if defined(CONFIG_SND_SOC_ES_UART_STREAMDEV)
		escore_priv.streamdev = es_uart_streamdev;
#elif defined(CONFIG_SND_SOC_ES_SPI_STREAMDEV)
		escore_priv.streamdev = es_spi_streamdev;
#endif

	mutex_init(&escore_priv.pm_mutex);
	mutex_init(&escore_priv.wake_mutex);
	mutex_init(&escore_priv.abort_mutex);
	mutex_init(&escore_priv.streaming_mutex);
	mutex_init(&escore_priv.msg_list_mutex);
	mutex_init(&escore_priv.datablock_dev.datablock_mutex);
	mutex_init(&escore_priv.datablock_dev.datablock_read_mutex);
	mutex_init(&escore_priv.escore_event_type_mutex);
	mutex_init(&escore_priv.gpio_reset_mutex);

	init_waitqueue_head(&escore_priv.stream_in_q);
	init_waitqueue_head(&escore_priv.irq_waitq);
	init_completion(&escore_priv.cmd_compl);
	INIT_LIST_HEAD(&escore_priv.msg_list);

#ifdef CONFIG_SND_SOC_ES_SPI_SENSOR_HUB
	/* Add initialization for sensor input devices */
	rc = es705_sensor_hub_init_data_driver(&escore_priv);
	if (rc != 0) {
		dev_err(escore_priv.dev,
		    "[SPI]: %s - could not create input device\n",
		    __func__);
		goto sens_init_err;
	}
#endif
	rc = sysfs_create_group(&escore_priv.dev->kobj, &core_sysfs);
	if (rc) {
		dev_err(escore_priv.dev, "%s(): failed to create core sysfs entries: %d\n",
			__func__, rc);
	}

#ifdef CONFIG_SND_SOC_ES705_EXTCLK_OVER_GPIO
	dev_dbg(es705_priv.dev, "%s(): extclk_gpio = %d\n",
		__func__, pdata->extclk_gpio);
	rc = devm_gpio_request(dev, pdata->extclk_gpio, "es705_extclk");
	if (rc < 0) {
		dev_warn(es705_priv.dev, "%s(): es705_extclk already requested",
			 __func__);
	} else {
		rc = gpio_direction_output(pdata->extclk_gpio, 1);
		if (rc < 0) {
			dev_err(es705_priv.dev,
				"%s(): es705_extclk direction fail %d\n",
				__func__, rc);
			goto extclk_gpio_direction_error;
		}
		pdata->esxxx_clk_cb = es705_enable_ext_clk;
		pdata->esxxx_clk_cb(1);
	}
#endif

#ifdef CONFIG_SND_SOC_ES705_EXTCLK
	/* Register for Clock */
	escore_priv.esxxx_ext_clk = clk_get(escore_priv.dev, "esxxx_ext_clk");
	if (IS_ERR(escore_priv.esxxx_ext_clk)) {
		dev_err(escore_priv.dev, "clk_get failed\n");
		rc = PTR_ERR(escore_priv.esxxx_ext_clk);
		return rc;
	}

	pdata->esxxx_clk_cb = es705_enable_ext_clk;
	pdata->esxxx_clk_cb(1);
#endif

	dev_dbg(escore_priv.dev, "%s(): wakeup_gpio = %d\n", __func__,
		pdata->wakeup_gpio);

	if (pdata->wakeup_gpio != -1) {
		rc = devm_gpio_request(dev, pdata->wakeup_gpio, "es705_wakeup");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_wakeup request fail %d\n",
				__func__, rc);
			goto exit;
		}
		rc = gpio_direction_output(pdata->wakeup_gpio, 1);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_wakeup direction fail %d\n",
				__func__, rc);
			goto exit;
		}
	} else {
		dev_warn(escore_priv.dev, "%s(): es705_wakeup undefined\n",
			 __func__);
	}

#ifndef CONFIG_SND_SOC_ES_GPIO_A
	dev_dbg(dev, "%s(): gpioa not configured\n", __func__);
	pdata->gpioa_gpio = -1;
#endif
	dev_dbg(escore_priv.dev, "%s(): gpioa_gpio = %d\n", __func__,
		pdata->gpioa_gpio);
	if (pdata->gpioa_gpio != -1) {
		rc = devm_gpio_request(dev, pdata->gpioa_gpio, "es705_gpioa");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_gpioa request fail %d\n",
				__func__, rc);
			goto exit;
		}
		rc = gpio_direction_input(pdata->gpioa_gpio);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_gpioa direction fail %d\n",
				__func__, rc);
			goto exit;
		}
		/* Fix value for IRQ Type */
		pdata->gpio_a_irq_type = ES_FALLING_EDGE;
	} else {
		dev_warn(escore_priv.dev, "%s(): es705_gpioa undefined\n",
			 __func__);
	}
	dev_dbg(escore_priv.dev, "%s(): gpiob_gpio = %d\n", __func__,
		pdata->gpiob_gpio);

	if (pdata->gpiob_gpio != -1) {
		rc = devm_gpio_request(dev, pdata->gpiob_gpio, "es705_gpiob");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_gpiob request fail %d\n",
				__func__, rc);
			goto exit;
		}
		rc = gpio_direction_input(pdata->gpiob_gpio);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): es705_gpiob direction fail %d\n",
				__func__, rc);
			goto exit;
		}
	} else {
		dev_warn(escore_priv.dev, "%s(): es705_gpiob undefined\n",
			 __func__);
	}

#ifndef CONFIG_SND_SOC_ES817_BASE
#ifdef CONFIG_SND_SOC_ES_ASYNC_FW_LOAD
	rc = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			fw_filename, escore_priv.dev,
			GFP_KERNEL, &escore_priv, escore_load_std_fw);
#else
	rc = request_firmware((const struct firmware **)&escore_priv.standard,
			      fw_filename, escore_priv.dev);
#endif
	if (rc) {
		dev_err(escore_priv.dev, "%s(): request_firmware(%s) failed %d\n",
			__func__, fw_filename, rc);
		goto request_firmware_error;
	}
#endif
#ifdef CONFIG_SND_SOC_ES814_4M_BAUD_UART
	rc = request_firmware((const struct firmware **)&escore_priv.stnd1st,
				fw_fn_1st_stg, escore_priv.dev);
	if (rc) {
		dev_err(escore_priv.dev, "%s(): request_firmware 1st stg (%s) failed %d\n",
				__func__, fw_fn_1st_stg, rc);
		goto request_firmware_error;
	}
#endif
#if defined(CONFIG_SND_SOC_ES_VS) && \
	!defined(CONFIG_SND_SOC_ES_ASYNC_FW_LOAD)
	rc = escore_vs_init(&escore_priv);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): voice sense init failed %d\n",
			__func__, rc);
		goto vs_init_error;
	}

	voice_sense = (struct escore_voice_sense *)escore_priv.voice_sense;

	voice_sense->vs_irq = false;
#ifdef CONFIG_SND_SOC_ES_AVOID_REPEAT_FW_DOWNLOAD
	voice_sense->vs_download_req = true,
#endif
	escore_priv.fw_requested = 1;
#endif /*CONFIG_SND_SOC_ES_VS && !CONFIG_SND_SOC_ES_ASYNC_FW_LOAD*/

#ifndef CONFIG_SND_SOC_ES_GPIO_A
	dev_dbg(dev, "%s(): gpioa not configured\n", __func__);
	pdata->gpioa_gpio = -1;
#endif
	/* API Interrupt registration */
	if (pdata->gpioa_gpio != -1) {
		rc = request_threaded_irq(gpio_to_irq(pdata->gpioa_gpio), NULL,
				es705_cmd_completion_isr, IRQF_TRIGGER_FALLING,
				"es705-cmd-completion-isr", &escore_priv);
		if (rc < 0) {
			pr_err("%s() API interrupt registration failed :%d",
					__func__, rc);
			goto api_intr_error;
		}
	}
	dev_dbg(dev, "%s(): gpioa gpio %d\n", __func__, pdata->gpioa_gpio);

	if (pdata->gpiob_gpio != -1) {
		rc = request_threaded_irq(gpio_to_irq(pdata->gpiob_gpio),
					  NULL,
					  es705_irq_work,
					  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					  "es705_irq_work", &escore_priv);
		if (rc) {
			dev_err(escore_priv.dev,
				"%s(): event request_irq() fail %d\n",
				__func__, rc);
			goto event_irq_request_error;
		}
		rc = irq_set_irq_wake(gpio_to_irq(pdata->gpiob_gpio), 1);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): set event irq wake fail %d\n",
				__func__, rc);
			disable_irq(gpio_to_irq(pdata->gpiob_gpio));
			free_irq(gpio_to_irq(pdata->gpiob_gpio),
				 &escore_priv);
			goto event_irq_wake_error;
		}

		/* Disable the interrupt till needed */
		if (pdata->gpiob_gpio != -1)
			disable_irq(gpio_to_irq(pdata->gpiob_gpio));

	} else {
		dev_warn(escore_priv.dev, "%s(): es705_gpiob undefined\n",
			 __func__);
	}

	dev_dbg(escore_priv.dev, "%s(): reset_gpio = %d\n", __func__,
		pdata->reset_gpio);
	if (pdata->reset_gpio != -1) {
		rc = devm_gpio_request(dev, pdata->reset_gpio, "es705_reset");
		if (rc < 0) {
			dev_warn(escore_priv.dev, "%s(): es705_reset already requested",
				 __func__);
		} else {
#ifdef CONFIG_SND_SOC_ES817_CICERO
			rc = gpio_direction_output(pdata->reset_gpio, 0);
#else
			rc = gpio_direction_output(pdata->reset_gpio, 1);
#endif
			if (rc < 0) {
				dev_err(escore_priv.dev,
					"%s(): es705_reset direction fail %d\n",
					__func__, rc);
				goto reset_gpio_direction_error;
			}
			/* reset the device */
			if (!escore_priv.flag.reset_done)
				escore_gpio_reset(&escore_priv);
		}
	} else {
		dev_warn(escore_priv.dev, "%s(): es705_reset undefined\n",
			 __func__);
	}

	return rc;

event_irq_wake_error:
event_irq_request_error:
api_intr_error:
reset_gpio_direction_error:
#if !defined(CONFIG_SND_SOC_ES_ASYNC_FW_LOAD)
#if defined(CONFIG_SND_SOC_ES_VS)
vs_init_error:
	escore_vs_exit(&escore_priv);
#endif /*CONFIG_SND_SOC_ES_VS*/
	release_firmware(escore_priv.standard);
#endif /*CONFIG_SND_SOC_ES_ASYNC_FW_LOAD*/
request_firmware_error:
exit:
#ifdef CONFIG_SND_SOC_ES_SPI_SENSOR_HUB
sens_init_err:
#endif

pdata_error:
	dev_dbg(escore_priv.dev, "%s(): exit with error\n", __func__);
	return rc;
}
#ifdef CONFIG_ARCH_MSM
const struct slim_device_id escore_slim_id[] = {

#if defined(CONFIG_SND_SOC_ES804_ESCORE)
	{ CODEC_ID_ES804, ESCORE_DEVICE_NONE },
	{ CODEC_INTF_ID_ES804, ESCORE_INTERFACE_DEVICE },
	{ CODEC_GEN0_ID_ES804, ESCORE_GENERIC_DEVICE },
#else
	/* es705 */
	{ CODEC_ID, ESCORE_DEVICE_NONE },
	{ CODEC_INTF_ID, ESCORE_INTERFACE_DEVICE },
	{ CODEC_GEN0_ID, ESCORE_GENERIC_DEVICE },
	/* es704 */
	{ CODEC_ID_ES704, ESCORE_DEVICE_NONE},
	{ CODEC_INTF_ID_ES704, ESCORE_INTERFACE_DEVICE },
	{ CODEC_GEN0_ID_ES704, ESCORE_GENERIC_DEVICE},
#endif
	{  }
};
MODULE_DEVICE_TABLE(slim, escore_slim_id);
#endif
static __init int es705_init(void)
{
	int rc = 0;

	pr_debug("%s()", __func__);

	escore_priv.device_name  = "elemental-addr";
	escore_priv.interface_device_name  = "slim-ifd";
	escore_priv.interface_device_elem_addr_name  =
					"slim-ifd-elemental-addr";

	mutex_init(&escore_priv.access_lock);
	mutex_init(&escore_priv.intf_probed_mutex);
	init_completion(&escore_priv.fw_download);
#if defined(CONFIG_SND_SOC_ES_ASYNC_FW_LOAD)
	init_completion(&escore_priv.request_fw);
#endif
	escore_platform_init();
#if defined(CONFIG_SND_SOC_ES_I2C)
	escore_priv.pri_intf = ES_I2C_INTF;
#elif defined(CONFIG_SND_SOC_ES_SLIM)
	escore_priv.pri_intf = ES_SLIM_INTF;
#elif defined(CONFIG_SND_SOC_ES_UART)
	escore_priv.pri_intf = ES_UART_INTF;
#elif defined(CONFIG_SND_SOC_ES_SPI)
	escore_priv.pri_intf = ES_SPI_INTF;
#endif

#if defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_I2C)
	escore_priv.high_bw_intf = ES_I2C_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SLIM)
	escore_priv.high_bw_intf = ES_SLIM_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_UART)
	escore_priv.high_bw_intf = ES_UART_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SPI)
	escore_priv.high_bw_intf = ES_SPI_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_DEFAULT)
	escore_priv.high_bw_intf = escore_priv.pri_intf;
#endif

#if defined(CONFIG_SND_SOC_ES_WAKEUP_UART)
	escore_priv.wakeup_intf = ES_UART_INTF;
#endif

#if defined(CONFIG_SND_SOC_ES_I2C) || \
		defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_I2C)
	rc = escore_i2c_init();
#endif
#if defined(CONFIG_SND_SOC_ES_SPI) || \
		defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SPI)
	rc = escore_spi_init();
#endif
#if defined(CONFIG_SND_SOC_ES_SLIM) || \
		defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SLIM)
	rc = escore_slimbus_init();
#endif
#if defined(CONFIG_SND_SOC_ES_UART) || \
		defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_UART)
	rc = escore_uart_bus_init(&escore_priv);
#if defined(CONFIG_SND_SOC_ES814_4M_BAUD_UART)
	escore_priv.enable_4m_baud = true;
#endif
#endif
	if (rc) {
		pr_err("Error registering Audience driver: %d\n", rc);
		goto INIT_ERR;
	}

#if defined(CONFIG_SND_SOC_ES_CDEV)
	rc = escore_cdev_init(&escore_priv);
	if (rc) {
		pr_err("Error enabling CDEV interface: %d\n", rc);
		goto INIT_ERR;
	}
#endif
INIT_ERR:
	return rc;
}
module_init(es705_init);

static __exit void es705_exit(void)
{
	if (escore_priv.flag.af)
		release_firmware(escore_priv.af);
	if (escore_priv.fw_requested) {
		release_firmware(escore_priv.standard);
	}
#if defined(CONFIG_SND_SOC_ES_UART_STREAMDEV)
	escore_cdev_cleanup(&escore_priv);
#endif

#if defined(CONFIG_SND_SOC_ES_I2C) || \
		defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_I2C)
	i2c_del_driver(&escore_i2c_driver);
#endif

#if defined(CONFIG_SND_SOC_ES705_I2C)
	i2c_del_driver(&es705_i2c_driver);
#else
	/* no support from QCOM to unregister
	 * slim_driver_unregister(&es705_slim_driver);
	 */
#endif

#ifdef CONFIG_SND_SOC_ES_SPI_SENSOR_HUB
	es705_sensor_hub_remove_data_driver(&escore_priv);
#endif
	escore_vs_exit(&escore_priv);
}
module_exit(es705_exit);


MODULE_DESCRIPTION("ASoC ES705 driver");
MODULE_AUTHOR("Greg Clemson <gclemson@audience.com>");
MODULE_AUTHOR("Genisim Tsilker <gtsilker@audience.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:earSmart-codec");
#if defined(CONFIG_SND_SOC_ES804_ESCORE)
MODULE_FIRMWARE("audience-es804-fw.bin");
#else
MODULE_FIRMWARE("audience-es705-fw.bin");
#endif

#ifdef CONFIG_SND_SOC_ES817_PREMIUM
MODULE_FIRMWARE("audience/es817/premium/audience-es817-ns.bin");
MODULE_FIRMWARE("audience/es817/premium/audience-es817-vs.bin");
#endif
