/*
 * TAS5760 ASoC speaker amplifier driver
 *
 * Copyright (c) 2015 Imagination Technolgies Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/fb_event.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <uapi/linux/fb_event.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "tas5760.h"

struct tas5760_hw_params {
	bool pbtl_mode;
	bool pbtl_right_channel;
	bool volume_fading;
	unsigned int digital_clip;
	unsigned int digital_boost;
	unsigned int analog_gain;
	unsigned int pwm_rate;
	unsigned int vol_left;
	unsigned int vol_right;
	unsigned int oce_thresh;
	int digital_clip_valid:1;
	int digital_boost_valid:1;
	int analog_gain_valid:1;
	int pwm_rate_valid:1;
	int vol_left_valid:1;
	int vol_right_valid:1;
	int oce_thresh_valid:1;
};

struct tas5760_private {
	struct i2c_client *client;
	struct regmap *regmap;
	unsigned int format;
	struct tas5760_hw_params hw_params;
	struct gpio_desc *amp_sdz_gpio;
	struct regulator *pwr;
	struct clk *mclk;
	int faultz_irq;
	bool dynamic_clk;
	bool dynamic_sd;
	s32 off_latency;			/* Turn off latency [us] */
};

static bool tas5760_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x07:
	case 0x09 ... 0x0F:
		return false;
	default:
		return true;
	}
}

static bool tas5760_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS5760_DEVICE_ID:
	case TAS5760_ERR_STATUS:
		return true;
	}

	return false;
}

static bool tas5760_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS5760_DEVICE_ID:
		return false;
	}

	return tas5760_accessible_reg(dev, reg);
}

static int tas5760_set_sysclk(struct snd_soc_dai *dai,
			      int clk_id, unsigned int freq, int dir)
{
	struct device *dev = dai->codec->dev;

	dev_dbg(dev, "clkid:%d, freq:%d, dir:%d", clk_id, freq, dir);
	return 0;
}

static int tas5760_check_id(struct tas5760_private *priv)
{
	struct device *dev = &priv->client->dev;
	int ret, id;

	/* Identify the device */
	ret = regmap_read(priv->regmap, TAS5760_DEVICE_ID, &id);
	if (ret < 0)
		return ret;

	if (id != 0x00) {
		dev_err(dev, "TAS5760 codec could not be identified");
		return -ENODEV;
	}

	dev_dbg(dev, "Identified TAS5760 codec chip\n");

	return 0;
}

static int tas5760_apply_hw_params(struct tas5760_private *priv)
{
	struct i2c_client *client = priv->client;
	struct tas5760_hw_params *hwp = &priv->hw_params;
	struct device *dev = &client->dev;
	int ret, val;
	u32 vol;

	dev_dbg(dev, "Apply HW Parameters\n");

	/* BTL/PBTL mode configuration and Channel Selection for PBTL mode */
	vol = TAS5760_PBTL_EN_VAL(hwp->pbtl_mode);
	vol |= TAS5760_PBTL_CH_VAL(hwp->pbtl_right_channel);
	ret = regmap_update_bits(priv->regmap, TAS5760_ANALOG_CTRL,
				TAS5760_PBTL_MASK, vol);
	if (ret < 0)
		return ret;

	/* Volume Fading */
	vol = TAS5760_VOL_FADE_VAL(hwp->volume_fading);
	ret = regmap_update_bits(priv->regmap, TAS5760_VOL_CTRL,
				TAS5760_VOL_FADE_MASK, vol);
	if (ret)
		return ret;

	/* Digital Clip level */
	if (hwp->digital_clip_valid) {
		if (hwp->digital_clip > TAS5760_DIGITAL_CLIP_MAX_VAL) {
			dev_err(dev, "Invalid value for digital clip\n");
			hwp->digital_clip = 0;
		}

		ret = regmap_update_bits(priv->regmap,
			TAS5760_DIGITAL_CLIP_CTRL1, TAS5760_DIGITAL_CLIP_MASK,
			TAS5760_DIGITAL_CLIP1_VAL(hwp->digital_clip));
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, TAS5760_DIGITAL_CLIP_CTRL2,
			TAS5760_DIGITAL_CLIP2_VAL(hwp->digital_clip));
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(priv->regmap,
			TAS5760_POWER_CTRL, TAS5760_DIGITAL_CLIP_MASK,
			TAS5760_DIGITAL_CLIP3_VAL(hwp->digital_clip));
		if (ret)
			return ret;
	}

	/* Digital Boost */
	if (hwp->digital_boost_valid) {
		switch (hwp->digital_boost) {
		/* Values in dB */
		case 0:
			val = 0x00;
			break;
		case 6:
			val = 0x01;
			break;
		case 12:
			val = 0x02;
			break;
		case 18:
			val = 0x03;
			break;
		default:
			hwp->digital_boost = 0;
			return -EINVAL;
		}
		ret = regmap_update_bits(priv->regmap, TAS5760_DIGITAL_CTRL,
				TAS5760_DIGITAL_BOOST_MASK,
				TAS5760_DIGITAL_BOOST_VAL(val));
		if (ret)
			return ret;
	}

	/* Analog gain */
	if (hwp->analog_gain_valid) {
		switch (hwp->analog_gain) {
		/* Values in 0.1 dBV*/
		case 192:
			val = 0x0;
			break;
		case 226:
			val = 0x01;
			break;
		case 250:
			val = 0x02;
			break;
		default:
			dev_err(dev, "Invalid value for analog gain\n");
			hwp->analog_gain = 0;
			return -EINVAL;
		}
		ret = regmap_update_bits(priv->regmap,
			TAS5760_ANALOG_CTRL, TAS5760_ANALOG_GAIN_MASK,
			TAS5760_ANALOG_GAIN_VAL(val));
		if (ret)
			return ret;
	}

	/* PWM rate */
	if (hwp->pwm_rate_valid) {
		if (hwp->pwm_rate > TAS5760_PWM_RATE_MAX) {
			hwp->pwm_rate = 0;
			return -EINVAL;
		}

		ret = regmap_update_bits(priv->regmap,
			TAS5760_ANALOG_CTRL, TAS5760_PWM_RATE_MASK,
			TAS5760_PWM_RATE_VAL(hwp->pwm_rate));
		if (ret)
			return ret;
	}

	/* Set Default Volume */
	if (hwp->vol_left_valid) {
		ret = regmap_update_bits(priv->regmap,
			TAS5760_VOL_CTRL_L, TAS5760_VOL_CTRL_MASK,
			hwp->vol_left);
		if (ret)
			return ret;
	}

	if (hwp->vol_right_valid) {
		ret = regmap_update_bits(priv->regmap,
			TAS5760_VOL_CTRL_R, TAS5760_VOL_CTRL_MASK,
			hwp->vol_right);
		if (ret)
			return ret;
	}

	/* Set default value for OCE Thresh */
	if (hwp->oce_thresh_valid) {
		ret = regmap_update_bits(priv->regmap,
			TAS5760_ERR_STATUS, TAS5760_OCE_THRESH_MASK,
			TAS5760_OCE_THRESH_VAL(hwp->oce_thresh));
		if (ret)
			return ret;
	}

	return 0;
}

static int tas5760_digital_mute(struct tas5760_private *priv, bool mute)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Digital mute: %d\n", mute);

	/*
	 * Mute Left/Right Channel
	 * 0: The right channel is not muted
	 * 1: The right channel is muted (In software mute, most analog
	 *    and digital blocks remain active and the speaker amplifier
	 *    outputs transition to a 50/50 duty cycle.)
	 */
	return regmap_update_bits(priv->regmap,
		TAS5760_VOL_CTRL, TAS5760_MUTE_MASK,
		mute ? TAS5760_MUTE_MASK : 0);
}

static inline int tas5760_spk_sleep(struct tas5760_private *priv, bool sleep)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "SPK Sleep: %d\n", sleep);

	/*
	 * Sleep Mode
	 * 0: Device is not in sleep mode.
	 * 1: Device is placed in sleep mode (In this mode, the power
	 *    stage is disabled to reduce quiescent power consumption over a
	 *    50/50 duty cycle mute, while low-voltage blocks remain on
	 *    standby. This reduces the time required to resume playback
	 *    when compared with entering and exiting full shut down.).
	 */
	return regmap_update_bits(priv->regmap, TAS5760_POWER_CTRL,
		TAS5760_SPK_SLEEP_MASK, sleep ? TAS5760_SPK_SLEEP_MASK : 0);
}

static inline int tas5760_spk_shutdown_hw(struct tas5760_private *priv, bool sd)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "HW - SPK Shutdown: %d\n", sd);

	if (!priv->amp_sdz_gpio)
		return -ENODEV;

	gpiod_set_value(priv->amp_sdz_gpio, sd);

	return 0;
}

static inline int tas5760_spk_shutdown_sw(struct tas5760_private *priv, bool sd)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "SW - SPK Shutdown: %d\n", sd);

	/*
	 * Speaker Shutdown
	 * 0: Speaker amplifier is shut down (This is the lowest power
	 *    mode available when the device is connected to power supplies.
	 *    In this mode, circuitry in both the DVDD and PVDD domain are
	 *    powered down to minimize power consumption.).
	 * 1: Speaker amplifier is not shut down.
	 */
	return regmap_update_bits(priv->regmap, TAS5760_POWER_CTRL,
		TAS5760_SPK_SD_MASK, sd ? 0 : TAS5760_SPK_SD_MASK);
}

static int tas5760_power_enable(struct tas5760_private *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	dev_dbg(dev, "Power Enable\n");

	ret = regulator_enable(priv->pwr);
	if (ret) {
		dev_err(dev, "error power regulator enable: %d\n", ret);
		return ret;
	}

	/* Delay between Power enable and first I2C transaction */
	msleep(100);

	/*
	 * Just in case Fault Recovery has left state in SW SHUTDOWN,
	 * set the cached register value back to "deactivate SW SHUTDOWN".
	 *
	 * Rationale is that a power supply cycling might have helped to
	 * clear the FAULT condition.
	 */
	tas5760_spk_shutdown_sw(priv, false);

	/* Apply all latest hw parameters */
	regcache_cache_only(priv->regmap, false);
	regcache_mark_dirty(priv->regmap);
	ret = regcache_sync(priv->regmap);
	if (ret)
		dev_err(dev, "regmap sync failed\n");

	if (priv->mclk && !priv->dynamic_clk)
		clk_prepare_enable(priv->mclk);

	if (!priv->dynamic_sd)
		tas5760_spk_shutdown_hw(priv, false);

	return 0;
}

static int tas5760_power_disable(struct tas5760_private *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Power Disable\n");

	/* Referenced sections from the datasheet:
	 *	9.2.1.2.2
	 *	8.4.2.1
	 */
	tas5760_digital_mute(priv, true);
	tas5760_spk_sleep(priv, true);

	usleep_range(priv->off_latency, priv->off_latency + 10000);

	regcache_cache_only(priv->regmap, true);

	tas5760_spk_shutdown_hw(priv, true);

	if (priv->mclk && !priv->dynamic_clk)
		clk_disable_unprepare(priv->mclk);

	regulator_disable(priv->pwr);

	return 0;
}

static irqreturn_t tas5760_faultz_irq(int irq, void *dev_id)
{
	struct tas5760_private *priv = dev_id;
	struct device *dev = &priv->client->dev;
	int err, val;

	err = regmap_read(priv->regmap, TAS5760_ERR_STATUS, &val);
	if (err < 0)
		dev_err(dev, "AMP faultz event: Status read, err = %d\n", err);

	val &= (TAS5760_IRQ_STATUS_MASK_LATCH | TAS5760_IRQ_STATUS_CLKE_MASK);
	if (!val)
		return IRQ_HANDLED;

	dev_dbg(dev, "IRQ Status: %s %s %s %s\n",
		(val & TAS5760_IRQ_STATUS_CLKE_MASK) ? "CLKE" : "",
		(val & TAS5760_IRQ_STATUS_OCE_MASK) ? "OCE" : "",
		(val & TAS5760_IRQ_STATUS_DCE_MASK) ? "DCE" : "",
		(val & TAS5760_IRQ_STATUS_OTE_MASK) ? "OTE" : "");

	/* Mute */
	tas5760_digital_mute(priv, 1);

	/* Check for latching irq status */
	if (val & TAS5760_IRQ_STATUS_MASK_LATCH) {
		/* Toggle bit #SPK_SD */
		tas5760_spk_shutdown_sw(priv, 0);
		tas5760_spk_shutdown_sw(priv, 1);
		/* Unmute after clear latching event */
		tas5760_digital_mute(priv, 0);
	}

	err = fb_event_log_int(FBE_TAS5760_FAULTZ, val);
	if (err)
		dev_dbg_ratelimited(dev, "Unable to add fb event, err=%d\n",
				err);

	return IRQ_HANDLED;
}

static int tas5760_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = dai->codec->dev;

	dev_dbg(dev, "format:%d\n", format);
	priv->format = format;

	return 0;
}

static int tas5760_dai_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	return 0;
}

static int tas5760_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = &priv->client->dev;
	int ret, val;

	dev_dbg(dev, "DAI Digital Mute: %d\n", mute);

	if (mute && priv->faultz_irq > 0)
		disable_irq(priv->faultz_irq);

	ret = tas5760_digital_mute(priv, !!mute);

	if (!mute && priv->faultz_irq > 0) {
		/* Clear interrupt status */
		regmap_read(priv->regmap, TAS5760_ERR_STATUS, &val);
		enable_irq(priv->faultz_irq);
	}

	return ret;
}

static int tas5760_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = &priv->client->dev;
	int ret;

	dev_dbg(dev, "TAS5760 Startup DAI\n");

	if (priv->mclk && priv->dynamic_clk) {
		ret = clk_prepare_enable(priv->mclk);
		if (ret)
			return ret;
	}

	if (priv->dynamic_sd)
		tas5760_spk_shutdown_hw(priv, false);

	tas5760_spk_sleep(priv, false);

	return 0;
}

static void tas5760_dai_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Shutdown DAI\n");

	tas5760_spk_sleep(priv, true);

	if (priv->dynamic_sd) {
		usleep_range(priv->off_latency, priv->off_latency + 10000);
		tas5760_spk_shutdown_hw(priv, true);
	}

	if (priv->mclk && priv->dynamic_clk)
		clk_disable_unprepare(priv->mclk);
}

static int tas5760_dai_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Trigger DAI: %d Cmd: %d\n", dai->id, cmd);

	return 0;
}

static int tas5760_dt_parse_hw_params(struct tas5760_private *priv)
{
	struct tas5760_hw_params *hwp = &priv->hw_params;
	struct device *dev = &priv->client->dev;
	struct device_node *np = dev->of_node;
	int ret;

	np = of_get_child_by_name(np, "hw-params");
	if (!np) {
		dev_warn(dev, "No hardware config for init\n");
		return 0;
	}

	/* BTL/PBTL mode configuration */
	hwp->pbtl_mode = of_property_read_bool(np, "pbtl-mode");
	dev_dbg(dev, "pbtl mode:%d\n", hwp->pbtl_mode);

	/* Channel Selection for PBTL mode */
	hwp->pbtl_right_channel = of_property_read_bool(np,
							"pbtl-right-channel");
	dev_dbg(dev, "pbtl right channel:%d\n", hwp->pbtl_right_channel);

	/* Volume Fading */
	hwp->volume_fading = of_property_read_bool(np, "volume-fading");
	dev_dbg(dev, "volume fading:%d\n", hwp->volume_fading);

	/* Digital Clip level */
	ret = of_property_read_u32(np, "digital-clip", &hwp->digital_clip);
	if (!ret)
		hwp->digital_clip_valid = 1;

	/* Digital Boost */
	ret = of_property_read_u32(np, "digital-boost", &hwp->digital_boost);
	if (!ret)
		hwp->digital_boost_valid = 1;

	/* Analog gain */
	ret = of_property_read_u32(np, "analog-gain", &hwp->analog_gain);
	if (!ret)
		hwp->analog_gain_valid = 1;

	/* PWM rate */
	ret = of_property_read_u32(np, "pwm-rate", &hwp->pwm_rate);
	if (!ret) {
		hwp->pwm_rate_valid = 1;
		if (hwp->pwm_rate > TAS5760_PWM_RATE_MAX)
			hwp->pwm_rate = TAS5760_PWM_RATE_MAX;
	}

	/* Set Default Volume */
	ret = of_property_read_u32(np, "vol-left", &hwp->vol_left);
	if (!ret)
		hwp->vol_left_valid = 1;

	ret = of_property_read_u32(np, "vol-right", &hwp->vol_right);
	if (!ret)
		hwp->vol_right_valid = 1;

	ret = of_property_read_u32(np, "oce-thresh", &hwp->oce_thresh);
	if (!ret)
		hwp->oce_thresh_valid = 1;
	hwp->oce_thresh = clamp_val(hwp->oce_thresh, 0, 3);

	return 0;
}

static ssize_t mclk_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);
	int ret = 0;

	if (size < 1)
		return -EINVAL;

	if (buf[0] == '1')
		ret = clk_prepare_enable(priv->mclk);
	else
		clk_disable_unprepare(priv->mclk);

	return ret ?: size;
}
static DEVICE_ATTR_WO(mclk);

static ssize_t mute_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);
	int ret;

	if (size < 1)
		return -EINVAL;

	ret = tas5760_digital_mute(priv, buf[0] == '1');

	return ret ?: size;
}
static DEVICE_ATTR_WO(mute);

static ssize_t sleep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);
	int ret;

	if (size < 1)
		return -EINVAL;

	ret = tas5760_spk_sleep(priv, buf[0] == '1');

	return ret ?: size;
}
static DEVICE_ATTR_WO(sleep);

static ssize_t shutdown_sw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);
	int ret;

	if (size < 1)
		return -EINVAL;

	ret = tas5760_spk_shutdown_sw(priv, buf[0] == '1');

	return ret ?: size;
}
static DEVICE_ATTR_WO(shutdown_sw);

static ssize_t shutdown_hw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);
	int ret;

	if (size < 1)
		return -EINVAL;

	ret = tas5760_spk_shutdown_hw(priv, buf[0] == '1');

	return ret ?: size;
}
static DEVICE_ATTR_WO(shutdown_hw);

static ssize_t delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, val;

	if (size < 1)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &val);
	if (!ret)
		usleep_range(val, val + 100);

	return ret ?: size;
}
static DEVICE_ATTR_WO(delay);

static struct attribute *tas5760_attrs[] = {
	&dev_attr_mclk.attr,
	&dev_attr_mute.attr,
	&dev_attr_sleep.attr,
	&dev_attr_shutdown_sw.attr,
	&dev_attr_shutdown_hw.attr,
	&dev_attr_delay.attr,
	NULL,
};

static const struct attribute_group tas5760_group = {
	.attrs = tas5760_attrs,
};

static const struct i2c_device_id tas5760_i2c_id[] = {
	{ "tas5760", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5760_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id tas5760_dt_ids[] = {
	{ .compatible = "ti,tas5760", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5760_dt_ids);
#endif

static int tas5760_probe(struct snd_soc_codec *codec)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(codec);
	struct device *dev = &priv->client->dev;
	int ret;

	dev_dbg(dev, "Codec probe\n");

	tas5760_spk_shutdown_hw(priv, true);

	ret = regulator_enable(priv->pwr);
	if (ret) {
		dev_err(dev, "error power regulator enable: %d\n", ret);
		return ret;
	}

	/* Delay between Power enable and first I2C transaction */
	msleep(100);

	tas5760_spk_sleep(priv, true);
	tas5760_digital_mute(priv, true);
	ret = tas5760_apply_hw_params(priv);
	if (ret)
		dev_err(dev, "Apply HW parameters fail: %d\n", ret);

	if (priv->mclk && !priv->dynamic_clk)
		clk_prepare_enable(priv->mclk);

	if (!priv->dynamic_sd)
		tas5760_spk_shutdown_hw(priv, false);

	return 0;
}

static int tas5760_remove(struct snd_soc_codec *codec)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(codec);

	tas5760_power_disable(priv);

	return 0;
};

/* TAS5760 controls */
static const DECLARE_TLV_DB_SCALE(tas5760_dac_tlv, -10350, 50, 1);

static const struct snd_kcontrol_new tas5760_controls[] = {
	SOC_DOUBLE_R_TLV("Channel 1/2 Playback Volume",
				TAS5760_VOL_CTRL_L, TAS5760_VOL_CTRL_R,
				0, 0xff, 0, tas5760_dac_tlv),
};

static struct snd_soc_codec_driver soc_codec_dev_tas5760 = {
	.probe		= tas5760_probe,
	.remove		= tas5760_remove,
	.controls	= tas5760_controls,
	.num_controls	= ARRAY_SIZE(tas5760_controls),
};

static const struct snd_soc_dai_ops tas5760_dai_ops = {
	.set_sysclk	= tas5760_set_sysclk,
	.set_fmt	= tas5760_set_dai_fmt,
	.hw_params	= tas5760_dai_hw_params,
	.digital_mute	= tas5760_dai_digital_mute,
	.startup	= tas5760_dai_startup,
	.shutdown	= tas5760_dai_shutdown,
	.trigger	= tas5760_dai_trigger,
};

static const struct regmap_config tas5760_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= TAS5760_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
	.volatile_reg	= tas5760_volatile_reg,
	.writeable_reg	= tas5760_writeable_reg,
	.readable_reg	= tas5760_accessible_reg,
};

static struct snd_soc_dai_driver tas5760_dai = {
	.name = "tas5760-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S18_3LE |
				SNDRV_PCM_FMTBIT_U20_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &tas5760_dai_ops,
};

static int tas5760_suspend(struct device *dev)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);

	return tas5760_power_disable(priv);
}

static int tas5760_resume(struct device *dev)
{
	struct tas5760_private *priv = dev_get_drvdata(dev);

	return tas5760_power_enable(priv);
}

static int tas5760_dt_parse(struct tas5760_private *priv)
{
	struct i2c_client *client = priv->client;
	struct device *dev = &client->dev;
	s32 val;
	int ret;

	/* Initialize the register maps for tas5760 */
	priv->regmap = devm_regmap_init_i2c(client, &tas5760_regmap);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev, "Failed to create regmap: %ld\n",
			PTR_ERR(priv->regmap));
		return PTR_ERR(priv->regmap);
	}

	/* Get the amp_power regulator */
	priv->pwr = devm_regulator_get(dev, "pwr");
	if (IS_ERR(priv->pwr)) {
		dev_err(dev, "error requesting power regulator: %ld\n",
			 PTR_ERR(priv->pwr));
		return PTR_ERR(priv->pwr);
	}

	/* Get the amp_sdz gpio */
	priv->amp_sdz_gpio = devm_gpiod_get(dev, "amp_sdz", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->amp_sdz_gpio)) {
		if (PTR_ERR(priv->amp_sdz_gpio) == -EPROBE_DEFER)
			return PTR_ERR(priv->amp_sdz_gpio);

		dev_err(dev, "error requesting amp_sdz_gpio: %ld\n",
			 PTR_ERR(priv->amp_sdz_gpio));
		priv->amp_sdz_gpio = NULL;
	}

	priv->faultz_irq = of_irq_get_byname(dev->of_node, "faultz");
	if (priv->faultz_irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	priv->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->mclk)) {
		if (PTR_ERR(priv->mclk) == -EPROBE_DEFER)
			return PTR_ERR(priv->mclk);
		priv->mclk = NULL;
	}

	ret = of_property_read_s32(dev->of_node, "off-latency-us", &val);
	if (!ret)
		priv->off_latency = val;

	priv->dynamic_clk = of_property_read_bool(dev->of_node, "dynamic-clk");
	priv->dynamic_sd = of_property_read_bool(dev->of_node,
		"dynamic-shutdown");

	return 0;
}

static int tas5760_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct tas5760_private *priv;
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(dev, "TAS5760 Probe - I2C Client\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->client = client;

	ret = tas5760_dt_parse(priv);
	if (ret)
		return ret;

	ret = tas5760_dt_parse_hw_params(priv);
	if (ret)
		return ret;

	if (priv->amp_sdz_gpio) {
		/* Set the direction as output, and keep it in shutdown */
		gpiod_direction_output(priv->amp_sdz_gpio, 1);
	}

	ret = regulator_enable(priv->pwr);
	if (ret) {
		dev_err(dev, "error power regulator enable: %ld\n",
			 PTR_ERR(priv->pwr));
		ret = PTR_ERR(priv->pwr);
		goto exit_pwr_enable;
	}

	/* Delay between Power enable and first I2C transaction */
	msleep(100);

	/* Identify the device */
	ret = tas5760_check_id(priv);
	if (ret < 0)
		goto exit_chip_id;

	if (priv->faultz_irq > 0) {
		ret = devm_request_threaded_irq(dev, priv->faultz_irq, NULL,
			tas5760_faultz_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			"tas5760_faultz", priv);
		if (ret < 0) {
			dev_err(dev, "Unable to request irq(%d), ret:%d)\n",
					priv->faultz_irq, ret);
			goto exit_irq;
		}

		disable_irq(priv->faultz_irq);

		dev_info(dev, "faultz irq %d\n", priv->faultz_irq);
	}

	/* balance reference counting of regulator */
	regulator_disable(priv->pwr);

	regcache_cache_bypass(priv->regmap, false);
	regcache_cache_only(priv->regmap, false);

	ret = sysfs_create_group(&dev->kobj, &tas5760_group);
	if (ret)
		goto exit_sysfs;

	return snd_soc_register_codec(&client->dev, &soc_codec_dev_tas5760,
				      &tas5760_dai, 1);

exit_sysfs:
exit_irq:
exit_chip_id:
	regulator_disable(priv->pwr);

exit_pwr_enable:
	return ret;
}

static int tas5760_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static void tas5760_i2c_shutdown(struct i2c_client *client)
{
	struct tas5760_private *priv = dev_get_drvdata(&client->dev);

	if (regulator_is_enabled(priv->pwr)) {
		regulator_disable(priv->pwr);

		/* Delay between amplifier power off and device power off */
		msleep(100);
	}
}

static const struct dev_pm_ops tas5760_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tas5760_suspend, tas5760_resume)
};


static struct i2c_driver tas5760_i2c_driver = {
	.driver = {
		.name	= "tas5760",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tas5760_dt_ids),
		.pm = &tas5760_i2c_pm_ops,
	},
	.id_table	= tas5760_i2c_id,
	.probe		= tas5760_i2c_probe,
	.remove		= tas5760_i2c_remove,
	.shutdown	= tas5760_i2c_shutdown,
};

module_i2c_driver(tas5760_i2c_driver);

MODULE_AUTHOR("Imagination Technologies Pvt Ltd.");
MODULE_DESCRIPTION("Texas Instruments TAS5760 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
