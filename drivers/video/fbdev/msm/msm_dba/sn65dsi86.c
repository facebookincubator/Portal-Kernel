/*
 * drivers/video/fbdev/msm/msm_dba/sn65dsi86.c
 *
 * Copyright (c) 2016, MM Solutions Corporation.
 *
 * Author:
 *	Dinko Mironov <dmironov@mm-sol.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include "../mdss_panel.h"

#define  SN65DSI86_DEVICE_ID				0x00
#define  SN65DSI86_DEVICE_REV				0x08
#define  SN65DSI86_SOFT_RESET				0x09
#define  SN65DSI86_PLL_REFCLK_CFG			0x0A
#define  SN65DSI86_PLL_EN				0x0D
#define  SN65DSI86_DSI_CFG1				0x10
#define  SN65DSI86_DSI_CFG2				0x11
#define  SN65DSI86_DSI_CHA_CLK_RANGE			0x12
#define  SN65DSI86_DSI_CHB_CLK_RANGE			0x13
#define  SN65DSI86_VIDEO_CHA_LINE_LOW			0x20
#define  SN65DSI86_VIDEO_CHA_LINE_HIGH			0x21
#define  SN65DSI86_VIDEO_CHB_LINE_LOW			0x22
#define  SN65DSI86_VIDEO_CHB_LINE_HIGH			0x23
#define  SN65DSI86_CHA_VERT_DISP_SIZE_LOW		0x24
#define  SN65DSI86_CHA_VERT_DISP_SIZE_HIGH		0x25
#define  SN65DSI86_CHA_HSYNC_PULSE_WIDTH_LOW		0x2C
#define  SN65DSI86_CHA_HSYNC_PULSE_WIDTH_HIGH		0x2D
#define  SN65DSI86_CHA_VSYNC_PULSE_WIDTH_LOW		0x30
#define  SN65DSI86_CHA_VSYNC_PULSE_WIDTH_HIGH		0x31
#define  SN65DSI86_CHA_HORIZONTAL_BACK_PORCH		0x34
#define  SN65DSI86_CHA_VERTICAL_BACK_PORCH		0x36
#define  SN65DSI86_CHA_HORIZONTAL_FRONT_PORCH		0x38
#define  SN65DSI86_CHA_VERTICAL_FRONT_PORCH		0x3a
#define  SN65DSI86_COLOR_BAR_CFG			0x3c
#define  SN65DSI86_FRAMING_CFG				0x5a
#define  SN65DSI86_DP_18BPP_EN				0x5b
#define  SN65DSI86_GPIO_CTRL_CFG			0x5f
#define  SN65DSI86_I2C_ADDR_CLAIM			0x60
#define  SN65DSI86_DP_SSC_CFG				0x93
#define  SN65DSI86_DP_CFG				0x94
#define  SN65DSI86_TRAINING_CFG				0x95
#define  SN65DSI86_ML_TX_MODE				0x96

#define SN65FLG_POWER			0
#define SN65FLG_ENABLE			1
#define SN65FLG_CONTINUOUS_RUN		2
#define SN65FLG_POWER_REMEMBER		3
#define SN65FLG_ENABLE_REMEMBER		4
#define SN65FLG_TE_AUTO_ENABLE		5

#define PrHex7				"%02X %02X %02X %02X %02X %02X %02X"

#define VAL2CHAR(val, n)		(((val) >> (n*8)) & 0xFF)

enum i2c_command {
	I2C_WRITE = 0,
	I2C_READ,
	I2C_POLL,
};

struct sn65dsi86_panel_timings {
	int32_t h_width;
	int32_t h_sync_width;
	int32_t h_front_porch;
	int32_t h_back_porch;
	int32_t v_height;
	int32_t v_sync_width;
	int32_t v_front_porch;
	int32_t v_back_porch;
	int32_t bpp;
};

struct sn65dsi86_reg {
	unsigned char cmd;	/* enum i2c_command */
	unsigned char addr;
	unsigned char val;
	unsigned char msleep;
} __packed;

struct sn65dsi86_reg_seq {
	struct sn65dsi86_reg *seq;
	int count;
};

struct sn65dsi86_ctx {
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;
	struct gpio_desc *pwr_gpio;
	struct gpio_desc *rst_gpio;
	struct gpio_desc *te_gpio;
	u32 aux_i2c_addr;
	unsigned long flags;
	struct mdss_panel_ops ops;
	struct sn65dsi86_panel_timings panel_timings;
	struct sn65dsi86_reg_seq seq_init;
	struct sn65dsi86_reg_seq seq_enable;
	struct sn65dsi86_reg_seq seq_disable;
};

static inline int sn65dsi86_reg_write(struct sn65dsi86_ctx *ctx,
					unsigned int addr, unsigned int val)
{
	return regmap_write(ctx->regmap, addr, val);
}

static inline int sn65dsi86_reg_read(struct sn65dsi86_ctx *ctx,
					unsigned int addr, unsigned int *val)
{
	return regmap_read(ctx->regmap, addr, val);
}

static const struct regmap_config sn65dsi86_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int sn65dsi86_touch_en(struct sn65dsi86_ctx *ctx, int enable)
{
	if (IS_ERR(ctx->te_gpio))
		return PTR_ERR(ctx->te_gpio);

	if (enable)
		gpiod_direction_input(ctx->te_gpio);
	else
		gpiod_direction_output(ctx->te_gpio, enable);

	return 0;
}

static int sn65dsi86_gpio_init(struct sn65dsi86_ctx *ctx, int enable)
{
	enable = !!enable;

	if (!IS_ERR(ctx->pwr_gpio))
		gpiod_direction_output(ctx->pwr_gpio, enable);

	if (!IS_ERR(ctx->rst_gpio))
		gpiod_direction_output(ctx->rst_gpio, enable);

	if (!IS_ERR(ctx->te_gpio))
		sn65dsi86_touch_en(ctx, enable);

	return 0;
}

static int sn65dsi86_power(struct sn65dsi86_ctx *ctx, int enable)
{
	if (enable && !test_and_set_bit(SN65FLG_POWER, &ctx->flags)) {
		if (!IS_ERR(ctx->rst_gpio)) {
			/* reset the panel */
			gpiod_set_value(ctx->rst_gpio, 0);
		}

		if (!IS_ERR(ctx->pwr_gpio)) {
			gpiod_set_value(ctx->pwr_gpio, 1);
			udelay(300);
		}

		if (!IS_ERR(ctx->rst_gpio)) {
			gpiod_set_value(ctx->rst_gpio, 1);
			udelay(200);
		}

		/* Software reset */
		sn65dsi86_reg_write(ctx, SN65DSI86_SOFT_RESET, 0x01);
		usleep_range(10000, 12000);
	} else if (!enable && test_and_clear_bit(SN65FLG_POWER, &ctx->flags)) {
		if (!IS_ERR(ctx->pwr_gpio))
			gpiod_set_value(ctx->pwr_gpio, 0);

		if (!IS_ERR(ctx->rst_gpio))
			gpiod_set_value(ctx->rst_gpio, 0);
	}

	return 0;
}

static bool sn65dsi86_check_id(struct sn65dsi86_ctx *ctx)
{
	const u8 dsi86_id[] = { 0x36, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20 };
	unsigned int i, val = 0, err = 0;
	u8 cid[7];

	for (i = 0; i < ARRAY_SIZE(dsi86_id); i++) {
		sn65dsi86_reg_read(ctx, i, &val);
		cid[i] = val & 0xff;
	}

	err = memcmp(cid, dsi86_id, sizeof(cid));
	if (err) {
		dev_err(&ctx->client->dev,
			"Chip Id: "PrHex7" (must be "PrHex7")",
			cid[0], cid[1], cid[2], cid[3], cid[4], cid[5], cid[6],
			dsi86_id[0], dsi86_id[1], dsi86_id[2], dsi86_id[3],
			dsi86_id[4], dsi86_id[5], dsi86_id[6]);
	}

	return err == 0;
}

static int sn65dsi86_apply_seq(struct sn65dsi86_ctx *ctx,
	struct sn65dsi86_reg_seq *seq)
{
	struct sn65dsi86_reg *s = seq->seq;
	int i, err = 0;

	for (i = 0; i < seq->count; i++, s++) {
		unsigned int val;

		if (s->cmd == I2C_WRITE)
			sn65dsi86_reg_write(ctx, s->addr, s->val);
		else if (s->cmd == I2C_READ)
			sn65dsi86_reg_read(ctx, s->addr, &val);
		else if (s->cmd == I2C_POLL) {
			unsigned int mask = 1UL << (s->val & 0x0F);
			unsigned int state = (s->val & 0x80) ? mask : 0x00;
			unsigned long ctime = jiffies;

			do {
				val = 0;
				err = sn65dsi86_reg_read(ctx, s->addr, &val);
				if (!err && (val & mask) == state)
					break;
				schedule();
			} while ((jiffies - ctime) < HZ);
		}

		if (s->msleep)
			usleep_range(s->msleep * 1000, s->msleep * 1000 + 500);
	}

	return err;
}

static int sn65dsi86_set_timings(struct sn65dsi86_ctx *ctx,
	struct sn65dsi86_panel_timings *pt)
{
	/* CHA_ACTIVE_LINE_LENGTH */
	sn65dsi86_reg_write(ctx, SN65DSI86_VIDEO_CHA_LINE_LOW,
				VAL2CHAR(pt->h_width, 0));
	sn65dsi86_reg_write(ctx, SN65DSI86_VIDEO_CHA_LINE_HIGH,
				VAL2CHAR(pt->h_width, 1));
	usleep_range(10000, 12000);

	/* CHA_VERTICAL_DISPLAY_SIZE */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VERT_DISP_SIZE_LOW,
				VAL2CHAR(pt->v_height, 0));
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VERT_DISP_SIZE_HIGH,
				VAL2CHAR(pt->v_height, 1));
	usleep_range(10000, 12000);

	/* CHA_HSYNC_PULSE_WIDTH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_HSYNC_PULSE_WIDTH_LOW,
				VAL2CHAR(pt->h_sync_width, 0));
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_HSYNC_PULSE_WIDTH_HIGH, 0x80);
	usleep_range(10000, 12000);

	/* CHA_VSYNC_PULSE_WIDTH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VSYNC_PULSE_WIDTH_LOW,
				VAL2CHAR(pt->v_sync_width, 0));
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VSYNC_PULSE_WIDTH_HIGH, 0x80);
	usleep_range(10000, 12000);

	/* CHA_HORIZONTAL_BACK_PORCH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_HORIZONTAL_BACK_PORCH,
				VAL2CHAR(pt->h_back_porch, 0));
	usleep_range(10000, 12000);

	/* CHA_VERTICAL_BACK_PORCH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VERTICAL_BACK_PORCH,
				VAL2CHAR(pt->v_back_porch, 0));
	usleep_range(10000, 12000);

	/* CHA_HORIZONTAL_FRONT_PORCH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_HORIZONTAL_FRONT_PORCH,
				VAL2CHAR(pt->h_front_porch, 0));
	usleep_range(10000, 12000);

	/* CHA_VERTICAL_FRONT_PORCH */
	sn65dsi86_reg_write(ctx, SN65DSI86_CHA_VERTICAL_FRONT_PORCH,
				VAL2CHAR(pt->v_front_porch, 0));
	usleep_range(10000, 12000);

	/* DP-18BPP Enable */
	switch (pt->bpp) {
	case 18:
		sn65dsi86_reg_write(ctx, SN65DSI86_DP_18BPP_EN, 0x01);
		break;
	case 24:
		sn65dsi86_reg_write(ctx, SN65DSI86_DP_18BPP_EN, 0x00);
		break;
	}

	usleep_range(10000, 15000);

	return 0;
}

static int sn65dsi86_video_enable(struct sn65dsi86_ctx *ctx)
{
	int err = 0;

	if (test_bit(SN65FLG_ENABLE, &ctx->flags))
		return 0;

	mutex_lock(&ctx->lock);

	err = sn65dsi86_apply_seq(ctx, &ctx->seq_init);
	if (err)
		goto fail;

	err = sn65dsi86_set_timings(ctx, &ctx->panel_timings);
	if (err)
		goto fail;

	err = sn65dsi86_apply_seq(ctx, &ctx->seq_enable);
	if (err)
		goto fail;

	if (ctx->aux_i2c_addr) {
		unsigned char addr = ctx->aux_i2c_addr << 1;

		sn65dsi86_reg_write(ctx, SN65DSI86_I2C_ADDR_CLAIM, addr);
	}

	if (test_bit(SN65FLG_TE_AUTO_ENABLE, &ctx->flags))
		sn65dsi86_touch_en(ctx, 0);

	set_bit(SN65FLG_ENABLE, &ctx->flags);

fail:
	mutex_unlock(&ctx->lock);

	return err;
}

static int sn65dsi86_video_disable(struct sn65dsi86_ctx *ctx)
{
	int err = 0;

	if (!test_bit(SN65FLG_ENABLE, &ctx->flags))
		return 0;

	mutex_lock(&ctx->lock);

	err = sn65dsi86_apply_seq(ctx, &ctx->seq_disable);
	if (err)
		goto fail;

	if (test_bit(SN65FLG_TE_AUTO_ENABLE, &ctx->flags))
		sn65dsi86_touch_en(ctx, 0);

	clear_bit(SN65FLG_ENABLE, &ctx->flags);

fail:
	mutex_unlock(&ctx->lock);

	return err;
}

static int sn65dsi86_power_on(struct mdss_panel_ops *ops, bool on)
{
	struct sn65dsi86_ctx *ctx =
		container_of(ops, struct sn65dsi86_ctx, ops);

	if (!ctx)
		return -EINVAL;

	dev_err(&ctx->client->dev, "Power %s\n", on ? "Enable" : "Disable");
	return sn65dsi86_power(ctx, on);
}

static int sn65dsi86_video_on(struct mdss_panel_ops *ops, bool on,
	struct mdss_panel_info *cfg)
{
	struct sn65dsi86_ctx *ctx =
		container_of(ops, struct sn65dsi86_ctx, ops);
	struct sn65dsi86_panel_timings *pt;
	int err;

	if (!ctx)
		return -EINVAL;

	dev_err(&ctx->client->dev, "Video %s\n", on ? "Enable" : "Disable");
	pt = &ctx->panel_timings;

	if (on) {
		if (cfg) {
			pt->h_width = cfg->xres;
			pt->h_sync_width = cfg->lcdc.h_pulse_width;
			pt->h_front_porch = cfg->lcdc.h_front_porch;
			pt->h_back_porch = cfg->lcdc.h_back_porch;
			pt->v_height = cfg->yres;
			pt->v_sync_width = cfg->lcdc.v_pulse_width;
			pt->v_front_porch = cfg->lcdc.v_front_porch;
			pt->v_back_porch = cfg->lcdc.v_back_porch;
		}

		err = sn65dsi86_video_enable(ctx);
	} else {
		err = sn65dsi86_video_disable(ctx);
	}

	return err;
}

#ifdef CONFIG_PM
static int sn65dsi86_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sn65dsi86_ctx *ctx = i2c_get_clientdata(client);

	clear_bit(SN65FLG_ENABLE_REMEMBER, &ctx->flags);
	clear_bit(SN65FLG_POWER_REMEMBER, &ctx->flags);

	if (test_bit(SN65FLG_ENABLE, &ctx->flags)) {
		set_bit(SN65FLG_ENABLE_REMEMBER, &ctx->flags);
		sn65dsi86_video_disable(ctx);
	}

	if (test_bit(SN65FLG_POWER, &ctx->flags)) {
		set_bit(SN65FLG_POWER_REMEMBER, &ctx->flags);
		sn65dsi86_power(ctx, 0);
	}

	return 0;
}

static int sn65dsi86_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sn65dsi86_ctx *ctx = i2c_get_clientdata(client);

	if (test_bit(SN65FLG_POWER_REMEMBER, &ctx->flags))
		sn65dsi86_power(ctx, 1);

	if (test_bit(SN65FLG_ENABLE_REMEMBER, &ctx->flags))
		sn65dsi86_video_enable(ctx);

	clear_bit(SN65FLG_ENABLE_REMEMBER, &ctx->flags);
	clear_bit(SN65FLG_POWER_REMEMBER, &ctx->flags);

	return 0;
}
#endif

static int sn65dsi86_parse_display_of(struct device_node *display_node,
						struct sn65dsi86_ctx *ctx)
{
	int32_t ret = 0;

	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-panel-width",
					&ctx->panel_timings.h_width);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-h-pulse-width",
					&ctx->panel_timings.h_sync_width);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-h-front-porch",
					&ctx->panel_timings.h_front_porch);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-h-back-porch",
					&ctx->panel_timings.h_back_porch);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-panel-height",
					&ctx->panel_timings.v_height);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-v-pulse-width",
					&ctx->panel_timings.v_sync_width);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-v-front-porch",
					&ctx->panel_timings.v_front_porch);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-v-back-porch",
					&ctx->panel_timings.v_back_porch);
	ret |= of_property_read_u32(display_node, "qcom,mdss-dsi-bpp",
					&ctx->panel_timings.bpp);

	return ret;
}

static int sn65dsi86_parse_reg_seq(struct sn65dsi86_ctx *ctx,
	const struct device_node *np, char *name, struct sn65dsi86_reg_seq *seq)
{
	struct sn65dsi86_reg *s;
	const unsigned char *data;
	int i, dlen;

	seq->count = 0;
	seq->seq = NULL;

	data = of_get_property(np, name, &dlen);
	if (!data || !dlen)
		return -ENODATA;

	dlen >>= 2;
	s = devm_kzalloc(&ctx->client->dev, sizeof(*seq) * dlen, GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	seq->count = dlen;
	seq->seq = s;
	for (i = 0; i < dlen; i++, s++) {
		s->cmd = *data++;
		s->addr = *data++;
		s->val = *data++;
		s->msleep = *data++;
	}

	return 0;
}

static int sn65dsi86_parser(struct sn65dsi86_ctx *ctx, struct device_node *np)
{
	struct device *dev = &ctx->client->dev;
	struct device_node *dsi_pan_node = NULL;
	int err, addr;

	err = sn65dsi86_parse_reg_seq(ctx, np, "sn65dsix6,reg-init",
		&ctx->seq_init);
	if (err) {
		dev_err(dev, "Register init sequance missing!\n");
		return -ENODATA;
	}

	err = sn65dsi86_parse_reg_seq(ctx, np, "sn65dsix6,reg-enable",
		&ctx->seq_enable);
	if (err) {
		dev_err(dev, "Register enable sequance missing!\n");
		return -ENODATA;
	}

	err = sn65dsi86_parse_reg_seq(ctx, np, "sn65dsix6,reg-disable",
		&ctx->seq_disable);
	if (err) {
		dev_err(dev, "Register disable sequance missing!\n");
		return -ENODATA;
	}

	dsi_pan_node = of_parse_phandle(np, "dsi_panel_node", 0);
	if (!dsi_pan_node) {
		dev_err(dev, "no panel node in DT\n");
		return -ENODATA;
	}

	err = sn65dsi86_parse_display_of(dsi_pan_node, ctx);
	if (err) {
		dev_err(dev, "at least one panel timing is missing!\n");
		return -EFAULT;
	}

	if (of_property_read_bool(np, "continuous-running"))
		set_bit(SN65FLG_CONTINUOUS_RUN, &ctx->flags);

	if (of_property_read_bool(np, "touch-auto-enable"))
		set_bit(SN65FLG_TE_AUTO_ENABLE, &ctx->flags);

	err = of_property_read_u32(np, "aux-i2c-addr", &addr);
	if (!err && addr > 0 && addr <= 0x7F)
		ctx->aux_i2c_addr = addr;

	ctx->pwr_gpio = devm_gpiod_get(dev, "platform-pwr", GPIOD_ASIS);
	ctx->rst_gpio = devm_gpiod_get(dev, "platform-en", GPIOD_ASIS);
	ctx->te_gpio = devm_gpiod_get(dev, "platform-touch-en", GPIOD_ASIS);

	return 0;
}

static int sn65dsi86_panel_register(struct sn65dsi86_ctx *ctx)
{
	struct mdss_panel_ops *ops = &ctx->ops;
	static const char *drv_name = "sn65dsi86";

	strlcpy(ops->name, drv_name, sizeof(ops->name));
	ops->instance_id = 0;
	ops->power_on = sn65dsi86_power_on;
	ops->video_on = sn65dsi86_video_on;

	return mdss_panel_ops_add(ops);
}

static int sn65dsi86_panel_unregister(struct sn65dsi86_ctx *ctx)
{
	return mdss_panel_ops_del(&ctx->ops);
}

static ssize_t sn65dsi86_color_pattern_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);
	unsigned int val;
	int rc;

	rc = sn65dsi86_reg_read(ctx, SN65DSI86_COLOR_BAR_CFG, &val);
	if (rc)
		return -EIO;

	if (val & 0x10)
		return snprintf(buf, PAGE_SIZE, "%d\n", val & 0x7);

	return snprintf(buf, PAGE_SIZE, "disable\n");
}

static ssize_t sn65dsi86_color_pattern_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);
	int val = buffer ? buffer[0] - '0' : -1;
	int rc = 0;

	if (val >= 0 && val <= 7)
		rc = sn65dsi86_reg_write(ctx, SN65DSI86_COLOR_BAR_CFG,
					 val | 0x10);
	else
		rc = sn65dsi86_reg_write(ctx, SN65DSI86_COLOR_BAR_CFG, 0x00);

	return !rc ? count : -EIO;
}
static DEVICE_ATTR(color_pattern, S_IRUGO | S_IWUSR,
	sn65dsi86_color_pattern_show, sn65dsi86_color_pattern_store);

static ssize_t sn65dsi86_aux_i2c_addr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", ctx->aux_i2c_addr);
}

static ssize_t sn65dsi86_aux_i2c_addr_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);

	if (sscanf(buffer, "0x%02x", &ctx->aux_i2c_addr) != 1)
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(aux_i2c_addr, S_IRUGO | S_IWUSR,
	sn65dsi86_aux_i2c_addr_show, sn65dsi86_aux_i2c_addr_store);

static ssize_t sn65dsi86_aux_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);
	unsigned int val;
	int rc;

	rc = sn65dsi86_reg_read(ctx, SN65DSI86_I2C_ADDR_CLAIM, &val);
	if (rc)
		return -EIO;

	return snprintf(buf, PAGE_SIZE, "%d\n", val & 0x1);
}

static ssize_t sn65dsi86_aux_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);
	int rc, val = ctx->aux_i2c_addr << 1;

	if (buffer && buffer[0] == '1')
		val |= 0x01;

	rc = sn65dsi86_reg_write(ctx, SN65DSI86_I2C_ADDR_CLAIM, val);
	if (rc)
		return -EIO;

	return count;
}
static DEVICE_ATTR(aux_enable, S_IRUGO | S_IWUSR,
	sn65dsi86_aux_enable_show, sn65dsi86_aux_enable_store);

static ssize_t sn65dsi86_touch_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);

	if (!IS_ERR(ctx->te_gpio))
		return PTR_ERR(ctx->te_gpio);

	return snprintf(buf, PAGE_SIZE, "%d\n", gpiod_get_value(ctx->te_gpio));
}

static ssize_t sn65dsi86_touch_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	struct sn65dsi86_ctx *ctx = dev_get_drvdata(dev);

	return sn65dsi86_touch_en(ctx, buffer && buffer[0] == '1') ? : count;
}
static DEVICE_ATTR(touch_enable, S_IRUGO | S_IWUSR,
	sn65dsi86_touch_enable_show, sn65dsi86_touch_enable_store);

static const struct attribute *sn65dsi86_dev_attrs[] = {
	&dev_attr_color_pattern.attr,
	&dev_attr_aux_i2c_addr.attr,
	&dev_attr_aux_enable.attr,
	&dev_attr_touch_enable.attr,
	NULL,
};

static int sn65dsi86_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sn65dsi86_ctx *ctx;
	int err;

	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!err) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	ctx = devm_kzalloc(&client->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	i2c_set_clientdata(client, ctx);
	ctx->client = client;

	mutex_init(&ctx->lock);

	err = sn65dsi86_parser(ctx, client->dev.of_node);
	if (err)
		goto no_of_data;

	ctx->regmap = devm_regmap_init_i2c(client, &sn65dsi86_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		err = PTR_ERR(ctx->regmap);
		dev_err(&client->dev, "regmap init failed\n");
		goto no_regmap;
	}

	sn65dsi86_gpio_init(ctx, test_bit(SN65FLG_CONTINUOUS_RUN, &ctx->flags));

	if (test_bit(SN65FLG_CONTINUOUS_RUN, &ctx->flags)) {
		dev_err(&ctx->client->dev, "Continuous running\n");
		set_bit(SN65FLG_POWER, &ctx->flags);
		set_bit(SN65FLG_ENABLE, &ctx->flags);
	}

	sn65dsi86_power(ctx, 1);

	err = !sn65dsi86_check_id(ctx);
	if (err) {
		dev_err(&client->dev, "Invalid chip id\n");
		err = -EPROBE_DEFER;
		goto no_device;
	}

	sn65dsi86_video_enable(ctx);

	err = sn65dsi86_panel_register(ctx);
	if (err) {
		dev_err(&client->dev, "Can't register as MSM DBA driver\n");
		goto no_dba_driver;
	}

	err = sysfs_create_files(&client->dev.kobj, sn65dsi86_dev_attrs);

	return 0;

no_dba_driver:
no_device:
	if (!test_bit(SN65FLG_CONTINUOUS_RUN, &ctx->flags)) {
		sn65dsi86_gpio_init(ctx, 0);

		if (!IS_ERR(ctx->te_gpio))
			gpiod_direction_input(ctx->te_gpio);

		if (!IS_ERR(ctx->rst_gpio))
			gpiod_direction_input(ctx->rst_gpio);
	}

no_regmap:

no_of_data:
	mutex_destroy(&ctx->lock);

	return err;
}

static int sn65dsi86_i2c_remove(struct i2c_client *client)
{
	struct sn65dsi86_ctx *ctx = i2c_get_clientdata(client);

	sn65dsi86_panel_unregister(ctx);
	sn65dsi86_video_disable(ctx);
	sn65dsi86_power(ctx, 0);

	if (!IS_ERR(ctx->te_gpio))
		gpiod_direction_input(ctx->te_gpio);

	if (!IS_ERR(ctx->rst_gpio))
		gpiod_direction_input(ctx->rst_gpio);

	if (!IS_ERR(ctx->pwr_gpio))
		gpiod_direction_input(ctx->rst_gpio);

	mutex_destroy(&ctx->lock);

	return 0;
}

static const struct i2c_device_id sn65dsi86_id_table[] = {
	{"sn65dsi86_dsi2edp", 0},
	{},
};

static const struct dev_pm_ops sn65dsi86_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sn65dsi86_suspend, sn65dsi86_resume)
};

static struct i2c_driver sn65dsi86_i2c_drv = {
	.driver = {
		.name = "sn65dsi86_dsi2edp",
		.owner = THIS_MODULE,
		.pm = &sn65dsi86_i2c_pm_ops,
	},
	.probe = sn65dsi86_i2c_probe,
	.remove = sn65dsi86_i2c_remove,
	.id_table = sn65dsi86_id_table,
};

static int __init sn65dsi86_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&sn65dsi86_i2c_drv);
	if (err)
		pr_err("sn65dsi86_dsi2edp: Failed to add i2c client driver\n");

	return err;
}

static void __exit sn65dsi86_i2c_client_exit(void)
{
	i2c_del_driver(&sn65dsi86_i2c_drv);
}

subsys_initcall(sn65dsi86_i2c_client_init);
module_exit(sn65dsi86_i2c_client_exit);

MODULE_AUTHOR("Dinko Mironov <dmironov@mm-sol.com>");
MODULE_DESCRIPTION(" TI bridge SN65DSI86 DSI to eDP");
MODULE_LICENSE("GPL");
