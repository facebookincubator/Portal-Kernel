// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * datasheet: http://www.ti.com/lit/ds/symlink/sn65dsi86.pdf
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/component.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>

#define SN_DEVICE_REV_REG			0x08
#define SN_DPPLL_SRC_REG			0x0A
#define  DPPLL_CLK_SRC_DSICLK			BIT(0)
#define  REFCLK_FREQ_MASK			GENMASK(3, 1)
#define  REFCLK_FREQ(x)				((x) << 1)
#define  DPPLL_SRC_DP_PLL_LOCK			BIT(7)
#define SN_PLL_ENABLE_REG			0x0D
#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define SN_DSIA_CLK_FREQ_REG			0x12
#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define  CHA_HSYNC_POLARITY			BIT(7)
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define  CHA_VSYNC_POLARITY			BIT(7)
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define SN_COLOR_BAR_REG			0x3C
#define  COLOR_BAR_ENABLE			BIT(4)
#define  COLOR_BAR_PATTERN			GENMASK(2, 0)
#define SN_ENH_FRAME_REG			0x5A
#define  VSTREAM_ENABLE				BIT(3)
#define SN_DATA_FORMAT_REG			0x5B
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_ADDR_15_8_REG			0x75
#define SN_AUX_ADDR_7_0_REG			0x76
#define SN_AUX_LENGTH_REG			0x77
#define SN_AUX_CMD_REG				0x78
#define  AUX_CMD_SEND				BIT(0)
#define  AUX_CMD_REQ(x)				((x) << 4)
#define SN_AUX_RDATA_REG(x)			(0x79 + (x))
#define SN_SSC_CONFIG_REG			0x93
#define  DP_NUM_LANES_MASK			GENMASK(5, 4)
#define  DP_NUM_LANES(x)			((x) << 4)
#define SN_DATARATE_CONFIG_REG			0x94
#define  DP_DATARATE_MASK			GENMASK(7, 5)
#define  DP_DATARATE(x)				((x) << 5)
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)

#define DP_SYMBOL_ERROR_COUNT_LANE0		0x210
#define DP_SYMBOL_ERROR_COUNT_LANE1		0x212
#define DP_SYMBOL_ERROR_COUNT_LANE2		0x214
#define DP_SYMBOL_ERROR_COUNT_LANE3		0x216

#define MIN_DSI_CLK_FREQ_MHZ	40

/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

struct ti_sn_bridge {
	struct device			*dev;
	struct regmap			*regmap;
	struct drm_dp_aux		aux;
	struct drm_bridge		bridge;
	struct drm_connector		connector;
	bool				connector_override;
	bool				panel_attached;
	bool				lp11_init;
	bool				continuous_splash;
	struct dentry			*debugfs;
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	struct clk			*refclk;
	struct drm_panel		*panel;
	struct gpio_desc		*enable_gpio;
	struct gpio_desc		*pwr_gpio;
	struct regulator_bulk_data	supplies[SN_REGULATOR_SUPPLY_NUM];
};

static const struct regmap_range ti_sn_bridge_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xFF },
};

static const struct regmap_access_table ti_sn_bridge_volatile_table = {
	.yes_ranges = ti_sn_bridge_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ti_sn_bridge_volatile_ranges),
};

static const struct regmap_config ti_sn_bridge_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &ti_sn_bridge_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static void ti_sn_bridge_write_u16(struct ti_sn_bridge *pdata,
				   unsigned int reg, u16 val)
{
	regmap_write(pdata->regmap, reg, val & 0xFF);
	regmap_write(pdata->regmap, reg + 1, val >> 8);
}

static int __maybe_unused ti_sn_bridge_resume(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	if (!pdata->panel_attached)
		return 0;

	gpiod_set_value(pdata->pwr_gpio, 1);

	ret = regulator_bulk_enable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret) {
		DRM_ERROR("failed to enable supplies %d\n", ret);
		return ret;
	}

	gpiod_set_value(pdata->enable_gpio, 1);

	DRM_DEBUG("Resume returning %d\n", ret);
	return ret;
}

static int __maybe_unused ti_sn_bridge_suspend(struct device *dev)
{
	struct ti_sn_bridge *pdata = dev_get_drvdata(dev);
	int ret;

	if (!pdata->panel_attached)
		return 0;

	gpiod_set_value(pdata->enable_gpio, 0);

	ret = regulator_bulk_disable(SN_REGULATOR_SUPPLY_NUM, pdata->supplies);
	if (ret)
		DRM_ERROR("failed to disable supplies %d\n", ret);

	gpiod_set_value(pdata->pwr_gpio, 0);

	DRM_DEBUG("Suspend returning %d\n", ret);
	return ret;
}

static const struct dev_pm_ops ti_sn_bridge_pm_ops = {
	SET_RUNTIME_PM_OPS(ti_sn_bridge_suspend, ti_sn_bridge_resume, NULL)
};

static int status_show(struct seq_file *s, void *data)
{
	struct ti_sn_bridge *pdata = s->private;
	unsigned int reg, val;

	seq_puts(s, "STATUS REGISTERS:\n");

	pm_runtime_get_sync(pdata->dev);

	/* IRQ Status Registers, see Table 31 in datasheet */
	for (reg = 0xf0; reg <= 0xf8; reg++) {
		regmap_read(pdata->regmap, reg, &val);
		seq_printf(s, "[0x%02x] = 0x%08x\n", reg, val);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}

static int status_open(struct inode *inode, struct file *file)
{
	return single_open(file, status_show, inode->i_private);
}

static const struct file_operations status_fops = {
	.open = status_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
};

static int symbol_err_cnt_show(struct seq_file *s, void *data)
{
	struct ti_sn_bridge *pdata = s->private;
	unsigned int reg, val;
	unsigned char lval, hval;
	int retval;

	seq_puts(s, "DP AUX SYMBOL_ERROR_COUNT_LANE:\n");

	pm_runtime_get_sync(pdata->dev);
	for (reg = DP_SYMBOL_ERROR_COUNT_LANE0;
			reg <= DP_SYMBOL_ERROR_COUNT_LANE3; reg += 2) {
		val = 0;

		retval = drm_dp_dpcd_readb(&pdata->aux, reg+1, &hval);
		if (retval != 1) {
			pr_err("Failed to read MSB reg 0x%x\n", reg+1);
			continue;
		}

		if (!(hval >> 7)) {
			pr_warn("Invalid symbol error count reg 0x%x=0x%x\n",
				reg+1, hval);
			seq_printf(s, "[0x%02x] = 0x%04x\n", reg+1, val);
			continue;
		}

		retval = drm_dp_dpcd_readb(&pdata->aux, reg, &lval);
		if (retval != 1) {
			pr_err("Failed to read LSB reg 0x%x\n", reg);
			continue;
		}

		val = ((hval &= 0x7F) << 8) | lval;
		seq_printf(s, "[0x%02x - 0x%02x] = 0x%04x\n", reg, reg+1, val);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}

static int symbol_err_cnt_open(struct inode *inode, struct file *file)
{
	return single_open(file, symbol_err_cnt_show, inode->i_private);
}

static const struct file_operations symbol_err_cnt_fops = {
	.open = symbol_err_cnt_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
};

static int tst_crc_show(struct seq_file *s, void *data)
{
	struct ti_sn_bridge *pdata = s->private;
	unsigned int reg, val;
	unsigned char lval, hval;
	int retval;

	seq_puts(s, "DP AUX TEST_SINK_MISC:\n");

	pm_runtime_get_sync(pdata->dev);

	retval = drm_dp_dpcd_readb(&pdata->aux, DP_TEST_SINK_MISC, &hval);
	if (retval != 1) {
		seq_puts(s, "Failed to read TEST_CRC_COUNT\n");
		goto end;
	}
	if (!(hval & DP_TEST_CRC_SUPPORTED)) {
		seq_printf(s, "Test CRC is not supported 0x%x\n", hval);
		goto end;
	}
	val = hval & DP_TEST_COUNT_MASK;
	seq_printf(s, "CRC Count = %d\n", val);

	for (reg = DP_TEST_CRC_R_CR; reg <= DP_TEST_CRC_B_CB; reg += 2) {
		val = 0;

		retval = drm_dp_dpcd_readb(&pdata->aux, reg+1, &hval);
		if (retval != 1) {
			pr_err("Failed to read MSB reg 0x%x\n", reg+1);
			continue;
		}

		retval = drm_dp_dpcd_readb(&pdata->aux, reg, &lval);
		if (retval != 1) {
			pr_err("Failed to read LSB reg 0x%x\n", reg);
			continue;
		}

		val = (hval << 8) | lval;
		seq_printf(s, "[0x%02x - 0x%02x] = 0x%04x\n", reg, reg+1, val);
	}

end:
	pm_runtime_put(pdata->dev);
	return 0;
}

static int test_crc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tst_crc_show, inode->i_private);
}

static const struct file_operations test_crc_fops = {
	.open = test_crc_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
};

static int test_crc_enable_get(void *data, u64 *val)
{
	int ret = -EINVAL;
	struct ti_sn_bridge *pdata = data;
	unsigned char value;

	pm_runtime_get_sync(pdata->dev);

	ret = drm_dp_dpcd_readb(&pdata->aux, DP_TEST_SINK, &value);

	pm_runtime_put(pdata->dev);

	*val = value;
	if (ret != 1) {
		pr_err("Failed to read TEST_SINK_START status\n");
		return ret;
	}

	return 0;
}

static int test_crc_enable_set(void *data, u64 val)
{
	int ret = -EINVAL;
	struct ti_sn_bridge *pdata = data;

	pm_runtime_get_sync(pdata->dev);

	if (val)
		ret = drm_dp_dpcd_writeb(&pdata->aux,
				DP_TEST_SINK, DP_TEST_SINK_START);
	else
		ret = drm_dp_dpcd_writeb(&pdata->aux, DP_TEST_SINK, 0);

	if (ret != 1) {
		pr_err("Failed to %s TEST_SINK_START\n",
			val?"start":"stop");
		goto end;
	}

	ret = 0;
end:
	pm_runtime_put(pdata->dev);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_test_crc_enable, test_crc_enable_get,
			test_crc_enable_set, "%llu\n");

static int color_bar_enable_get(void *data, u64 *val)
{
	struct ti_sn_bridge *pdata = data;
	unsigned int value;
	int ret;

	pm_runtime_get_sync(pdata->dev);
	ret = regmap_read(pdata->regmap, SN_COLOR_BAR_REG, &value);
	pm_runtime_put(pdata->dev);

	*val = (value & COLOR_BAR_ENABLE) ? 1 : 0;
	return ret;
}

static int color_bar_enable_set(void *data, u64 val)
{
	struct ti_sn_bridge *pdata = data;
	unsigned int value = 0;
	int ret;

	/* Color bar pattern 0~7 */
	if (val) {
		value = val - 1;
		if (value > 7)
			value = 7;
		value |= COLOR_BAR_ENABLE;

		pr_err("COLOR_BAR_ENABLE=0x%lx val=0x%llx --> value=0x%x\n",
			COLOR_BAR_ENABLE, val, value);
	}

	pm_runtime_get_sync(pdata->dev);
	ret = regmap_write(pdata->regmap, SN_COLOR_BAR_REG, value);
	pm_runtime_put(pdata->dev);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_color_bar_enable, color_bar_enable_get,
			color_bar_enable_set, "%llu\n");

static void ti_sn_debugfs_init(struct ti_sn_bridge *pdata)
{
	pdata->debugfs = debugfs_create_dir(dev_name(pdata->dev), NULL);

	debugfs_create_file("status", 0600, pdata->debugfs, pdata,
			&status_fops);
	debugfs_create_file("symbol_err_cnt", 0600, pdata->debugfs, pdata,
			&symbol_err_cnt_fops);
	debugfs_create_file("test_crc", 0600, pdata->debugfs, pdata,
			&test_crc_fops);
	debugfs_create_file("test_crc_enable", 0600, pdata->debugfs, pdata,
			&fops_test_crc_enable);
	debugfs_create_file("color_bar_enable", 0600, pdata->debugfs, pdata,
			&fops_color_bar_enable);
}

static void ti_sn_debugfs_remove(struct ti_sn_bridge *pdata)
{
	debugfs_remove_recursive(pdata->debugfs);
	pdata->debugfs = NULL;
}

/* Connector funcs */
static struct ti_sn_bridge *bridge_to_ti_sn_bridge(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ti_sn_bridge, bridge);
}

static struct ti_sn_bridge *
connector_to_ti_sn_bridge(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = NULL;

	if (connector->private)
		pdata = bridge_to_ti_sn_bridge(connector->private);

	if (pdata && pdata->connector_override)
		return pdata;
	else
		return container_of(connector, struct ti_sn_bridge, connector);
}

static int ti_sn_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = connector_to_ti_sn_bridge(connector);

	DRM_DEBUG("Calling drm_panel_get_modes()\n");
	return drm_panel_get_modes(pdata->panel);
}

static enum drm_mode_status
ti_sn_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	DRM_DEBUG("mode->clock=%d\n", mode->clock);

	/* maximum supported resolution is 4K at 60 fps */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs ti_sn_bridge_connector_helper_funcs = {
	.get_modes = ti_sn_bridge_connector_get_modes,
	.mode_valid = ti_sn_bridge_connector_mode_valid,
};

static enum drm_connector_status
ti_sn_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	/**
	 * TODO: Currently if drm_panel is present, then always
	 * return the status as connected. Need to add support to detect
	 * device state for hot pluggable scenarios.
	 */

	DRM_DEBUG("connector_status = %d\n", connector_status_connected);
	return connector_status_connected;
}

static const struct drm_connector_funcs ti_sn_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ti_sn_bridge_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int ti_sn_bridge_parse_regulators(struct ti_sn_bridge *pdata)
{
	unsigned int i;
	const char * const ti_sn_bridge_supply_names[] = {
		"vcca", "vcc", "vccio", "vpll",
	};

	for (i = 0; i < SN_REGULATOR_SUPPLY_NUM; i++)
		pdata->supplies[i].supply = ti_sn_bridge_supply_names[i];

	return devm_regulator_bulk_get(pdata->dev, SN_REGULATOR_SUPPLY_NUM,
				       pdata->supplies);
}

static int ti_sn_bridge_attach(struct drm_bridge *bridge)
{
	int ret, val;
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "ti_sn_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	if (!pdata->connector_override) {
		ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &ti_sn_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
		if (ret) {
			DRM_ERROR("Failed to initialize connector with drm\n");
			return ret;
		}

		drm_connector_helper_add(&pdata->connector,
				 &ti_sn_bridge_connector_helper_funcs);
		drm_mode_connector_attach_encoder(&pdata->connector,
				bridge->encoder);
	}

	/*
	 * TODO: ideally finding host resource and dsi dev registration needs
	 * to be done in bridge probe. But some existing DSI host drivers will
	 * wait for any of the drm_bridge/drm_panel to get added to the global
	 * bridge/panel list, before completing their probe. So if we do the
	 * dsi dev registration part in bridge probe, before populating in
	 * the global bridge list, then it will cause deadlock as dsi host probe
	 * will never complete, neither our bridge probe. So keeping it here
	 * will satisfy most of the existing host drivers. Once the host driver
	 * is fixed we can move the below code to bridge probe safely.
	 */
	host = of_find_mipi_dsi_host_by_node(pdata->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	/* TODO: setting to 4 lanes always for now */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO_BLLP |
		  MIPI_DSI_MODE_VIDEO_EOF_BLLP;


	/* check if continuous dsi clock is required or not */
	pm_runtime_get_sync(pdata->dev);
	regmap_read(pdata->regmap, SN_DPPLL_SRC_REG, &val);
	pm_runtime_put_sync(pdata->dev);
	if (!(val & DPPLL_CLK_SRC_DSICLK))
		dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	pdata->dsi = dsi;

	/* attach panel to bridge, if not already attached */
	if (!pdata->connector_override)
		drm_panel_attach(pdata->panel, &pdata->connector);

	pdata->panel_attached = true;

	DRM_DEBUG("Attach successfully\n");
	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_host:
	if (!pdata->connector_override)
		drm_connector_cleanup(&pdata->connector);
	DRM_ERROR("Attach failed %d\n", ret);
	return ret;
}

static void ti_sn_bridge_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	DRM_DEBUG("Disabling bridge\n");
	drm_panel_disable(pdata->panel);

	/* disable video stream */
	regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE, 0);
	/* semi auto link training mode OFF */
	regmap_write(pdata->regmap, SN_ML_TX_MODE_REG, 0);
	/* disable DP PLL */
	regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 0);

	drm_panel_unprepare(pdata->panel);
}

static u32 ti_sn_bridge_get_dsi_freq(struct ti_sn_bridge *pdata)
{
	u32 bit_rate_khz, clk_freq_khz;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	bit_rate_khz = mode->clock *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	clk_freq_khz = bit_rate_khz / (pdata->dsi->lanes * 2);

	DRM_DEBUG("clk_freq_khz=%dhz\n", clk_freq_khz);
	return clk_freq_khz;
}

/* clk frequencies supported by bridge in Hz in case derived from REFCLK pin */
static const u32 ti_sn_bridge_refclk_lut[] = {
	12000000,
	19200000,
	26000000,
	27000000,
	38400000,
};

/* clk frequencies supported by bridge in Hz in case derived from DACP/N pin */
static const u32 ti_sn_bridge_dsiclk_lut[] = {
	468000000,
	384000000,
	416000000,
	486000000,
	460800000,
};

static void ti_sn_bridge_set_refclk_freq(struct ti_sn_bridge *pdata)
{
	int i;
	u32 refclk_rate;
	const u32 *refclk_lut;
	size_t refclk_lut_size;

	if (pdata->refclk) {
		refclk_rate = clk_get_rate(pdata->refclk);
		refclk_lut = ti_sn_bridge_refclk_lut;
		refclk_lut_size = ARRAY_SIZE(ti_sn_bridge_refclk_lut);
		clk_prepare_enable(pdata->refclk);
	} else {
		refclk_rate = ti_sn_bridge_get_dsi_freq(pdata) * 1000;
		refclk_lut = ti_sn_bridge_dsiclk_lut;
		refclk_lut_size = ARRAY_SIZE(ti_sn_bridge_dsiclk_lut);
	}

	/* for i equals to refclk_lut_size means default frequency */
	for (i = 0; i < refclk_lut_size; i++)
		if (refclk_lut[i] == refclk_rate)
			break;

	DRM_DEBUG("refclk=%d\n", refclk_rate);
	regmap_update_bits(pdata->regmap, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK,
			   REFCLK_FREQ(i));
}

/**
 * LUT index corresponds to register value and
 * LUT values corresponds to dp data rate supported
 * by the bridge in Mbps unit.
 */
static const unsigned int ti_sn_bridge_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

static void ti_sn_bridge_set_dsi_dp_rate(struct ti_sn_bridge *pdata)
{
	unsigned int bit_rate_mhz, clk_freq_mhz, dp_rate_mhz;
	unsigned int val, i;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/* set DSIA clk frequency */
	bit_rate_mhz = (mode->clock / 1000) *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	clk_freq_mhz = bit_rate_mhz / (pdata->dsi->lanes * 2);

	/* for each increment in val, frequency increases by 5MHz */
	val = (MIN_DSI_CLK_FREQ_MHZ / 5) +
		(((clk_freq_mhz - MIN_DSI_CLK_FREQ_MHZ) / 5) & 0xFF);
	regmap_write(pdata->regmap, SN_DSIA_CLK_FREQ_REG, val);

	/* set DP data rate */
	dp_rate_mhz = ((bit_rate_mhz / pdata->dsi->lanes) * DP_CLK_FUDGE_NUM) /
							DP_CLK_FUDGE_DEN;
	for (i = 0; i < ARRAY_SIZE(ti_sn_bridge_dp_rate_lut) - 1; i++)
		if (ti_sn_bridge_dp_rate_lut[i] > dp_rate_mhz)
			break;

	DRM_DEBUG("dp_rate_mhz=%d bit_rate=%d dp_rate=%d\n",
			dp_rate_mhz, bit_rate_mhz, DP_DATARATE(i));
	regmap_update_bits(pdata->regmap, SN_DATARATE_CONFIG_REG,
			   DP_DATARATE_MASK, DP_DATARATE(i));
}

static void ti_sn_bridge_set_video_timings(struct ti_sn_bridge *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	u8 hsync_polarity = 0, vsync_polarity = 0;

	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		hsync_polarity = CHA_HSYNC_POLARITY;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vsync_polarity = CHA_VSYNC_POLARITY;

	ti_sn_bridge_write_u16(pdata, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,
			       mode->hdisplay);
	ti_sn_bridge_write_u16(pdata, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,
			       mode->vdisplay);
	regmap_write(pdata->regmap, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->hsync_end - mode->hsync_start) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->hsync_end - mode->hsync_start) >> 8) & 0x7F) |
		     hsync_polarity);
	regmap_write(pdata->regmap, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->vsync_end - mode->vsync_start) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->vsync_end - mode->vsync_start) >> 8) & 0x7F) |
		     vsync_polarity);

	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		     (mode->htotal - mode->hsync_end) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_BACK_PORCH_REG,
		     (mode->vtotal - mode->vsync_end) & 0xFF);

	regmap_write(pdata->regmap, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		     (mode->hsync_start - mode->hdisplay) & 0xFF);
	regmap_write(pdata->regmap, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		     (mode->vsync_start - mode->vdisplay) & 0xFF);

	usleep_range(10000, 10500); /* 10ms delay recommended by spec */
}

static void ti_sn_bridge_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);
	unsigned int val;
	int ret;

	/* Power on the bridge if not powered on by the host DSI */
	if (!pdata->lp11_init)
		pm_runtime_get_sync(pdata->dev);

	/* configure bridge ref_clk */
	ti_sn_bridge_set_refclk_freq(pdata);

	/*
	 * If continuous_splash is defined, skip remaining init
	 */
	if (!pdata->lp11_init && pdata->continuous_splash)
		return;

	/* in case drm_panel is connected then HPD is not supported */
	regmap_update_bits(pdata->regmap, SN_HPD_DISABLE_REG, HPD_DISABLE,
			HPD_DISABLE);

	/* DSI_A lane config */
	val = CHA_DSI_LANES(4 - pdata->dsi->lanes);
	regmap_update_bits(pdata->regmap, SN_DSI_LANES_REG,
			   CHA_DSI_LANES_MASK, val);

	/* DP lane config */
	val = DP_NUM_LANES(pdata->dsi->lanes - 1);
	regmap_update_bits(pdata->regmap, SN_SSC_CONFIG_REG, DP_NUM_LANES_MASK,
			   val);

	/* set dsi/dp clk frequency value */
	ti_sn_bridge_set_dsi_dp_rate(pdata);

	/* enable DP PLL */
	regmap_write(pdata->regmap, SN_PLL_ENABLE_REG, 1);

	ret = regmap_read_poll_timeout(pdata->regmap, SN_DPPLL_SRC_REG, val,
				       val & DPPLL_SRC_DP_PLL_LOCK, 1000,
				       50 * 1000);
	if (ret) {
		DRM_ERROR("DP_PLL_LOCK polling failed (%d)\n", ret);
		return;
	}

	/**
	 * The SN65DSI86 only supports ASSR Display Authentication method and
	 * this method is enabled by default. An eDP panel must support this
	 * authentication method. We need to enable this method in the eDP panel
	 * at DisplayPort address 0x0010A prior to link training.
	 */
	drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
			   DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);

	/* Semi auto link training mode */
	regmap_write(pdata->regmap, SN_ML_TX_MODE_REG, 0x0A);
	ret = regmap_read_poll_timeout(pdata->regmap, SN_ML_TX_MODE_REG, val,
				       val == ML_TX_MAIN_LINK_OFF ||
				       val == ML_TX_NORMAL_MODE, 1000,
				       500 * 1000);
	if (ret) {
		DRM_ERROR("Training complete polling failed (%d)\n", ret);
		return;
	} else if (val == ML_TX_MAIN_LINK_OFF) {
		DRM_ERROR("Link training failed, link is off\n");
		return;
	}

	/* config video parameters */
	ti_sn_bridge_set_video_timings(pdata);

	/* enable video stream */
	regmap_update_bits(pdata->regmap, SN_ENH_FRAME_REG, VSTREAM_ENABLE,
			   VSTREAM_ENABLE);

	drm_panel_enable(pdata->panel);
	DRM_DEBUG("Enable success\n");
}

static void ti_sn_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	/* As per datasheet, bridge should be powered on
	 * during lp11 DSI signal i.e. between TD6, and TD7
	 * TD6 = minimum time after the start of lp11 = 0ns
	 * TD7 = minimum time before the end of lp11 = 100us
	 * DSI from host is started after bridge pre-enable
	 * but before bridge enable.
	 * Therefore don't power on the bridge here.
	 */

	drm_panel_prepare(pdata->panel);
}

static void ti_sn_bridge_post_disable(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);

	pm_runtime_put_sync(pdata->dev);

	DRM_DEBUG("Bridge is disabled\n");
}

static struct drm_connector_funcs override_funcs;
static struct drm_connector_helper_funcs override_helper_private;

static int ti_sn_bridge_connector_init(struct drm_bridge *bridge,
					struct drm_connector *connector)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	if (connector->encoder != bridge->encoder) {
		DRM_ERROR(
		   "bridge and connector need attach to the same encoder\n");
		return -EINVAL;
	}

	if (!pdata->connector_override) {
		DRM_ERROR("Connector override is disabled\n");
		return 0;
	}

	connector->private = bridge;

	/*
	 * Make a copy of drm_connector_funcs and drm_connector_helper_funcs. To
	 * make sure other KMS components won't be broken. For example, if other
	 * connectors share the implementation for ->funs, overwriting this will
	 * break other connectors.
	 */
	override_funcs = *connector->funcs;
	override_funcs.detect = ti_sn_bridge_connector_detect;
	connector->funcs = &override_funcs;

	override_helper_private = *connector->helper_private;
	override_helper_private.get_modes = ti_sn_bridge_connector_get_modes;
	override_helper_private.mode_valid = ti_sn_bridge_connector_mode_valid;
	connector->helper_private = &override_helper_private;

	/* attach panel to bridge */
	drm_panel_attach(pdata->panel, connector);

	DRM_DEBUG("Connecteor override initialized\n");
	return 0;
}

/* Function is called by host DSI to power on the bridge during lp11 signal
 * It is called every time the bridge resumes from suspend. It is not called
 * the first time during boot, when continuous splash is enabled.
 */
static void ti_sn_bridge_power_on(struct drm_bridge *bridge)
{
	struct ti_sn_bridge *pdata = bridge_to_ti_sn_bridge(bridge);

	if (!pdata)
		return;

	if (!pdata->lp11_init)
		pdata->lp11_init = 1;

	pm_runtime_get_sync(pdata->dev);
}

static const struct drm_bridge_funcs ti_sn_bridge_funcs = {
	.attach = ti_sn_bridge_attach,
	.pre_enable = ti_sn_bridge_pre_enable,
	.enable = ti_sn_bridge_enable,
	.disable = ti_sn_bridge_disable,
	.post_disable = ti_sn_bridge_post_disable,
	.connector_init = ti_sn_bridge_connector_init,
	.power_on = ti_sn_bridge_power_on,
};

static struct ti_sn_bridge *aux_to_ti_sn_bridge(struct drm_dp_aux *aux)
{
	return container_of(aux, struct ti_sn_bridge, aux);
}

static ssize_t ti_sn_aux_transfer(struct drm_dp_aux *aux,
				  struct drm_dp_aux_msg *msg)
{
	struct ti_sn_bridge *pdata = aux_to_ti_sn_bridge(aux);
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 request_val = AUX_CMD_REQ(msg->request);
	u8 *buf = (u8 *)msg->buffer;
	unsigned int val;
	int ret, i;

	if (msg->size > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		regmap_write(pdata->regmap, SN_AUX_CMD_REG, request_val);
		break;
	default:
		return -EINVAL;
	}

	regmap_write(pdata->regmap, SN_AUX_ADDR_19_16_REG,
		     (msg->address >> 16) & 0xF);
	regmap_write(pdata->regmap, SN_AUX_ADDR_15_8_REG,
		     (msg->address >> 8) & 0xFF);
	regmap_write(pdata->regmap, SN_AUX_ADDR_7_0_REG, msg->address & 0xFF);

	regmap_write(pdata->regmap, SN_AUX_LENGTH_REG, msg->size);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++)
			regmap_write(pdata->regmap, SN_AUX_WDATA_REG(i),
				     buf[i]);
	}

	regmap_write(pdata->regmap, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);

	ret = regmap_read_poll_timeout(pdata->regmap, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 200,
				       50 * 1000);
	if (ret)
		return ret;

	ret = regmap_read(pdata->regmap, SN_AUX_CMD_STATUS_REG, &val);
	if (ret)
		return ret;
	else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
		 || (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
		 || (val & AUX_IRQ_STATUS_AUX_SHORT))
		return -ENXIO;

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE)
		return msg->size;

	for (i = 0; i < msg->size; i++) {
		unsigned int val;
		ret = regmap_read(pdata->regmap, SN_AUX_RDATA_REG(i),
				  &val);
		if (ret)
			return ret;

		WARN_ON(val & ~0xFF);
		buf[i] = (u8)(val & 0xFF);
	}

	return msg->size;
}

static int ti_sn_bridge_parse_dsi_host(struct ti_sn_bridge *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}

	return 0;
}

static int ti_sn_bind(struct device *dev, struct device *master, void *data)
{
	return 0;
}

static void ti_sn_unbind(struct device *dev, struct device *master, void *data)
{
}

static const struct component_ops ti_sn_comp_ops = {
	.bind = ti_sn_bind,
	.unbind = ti_sn_unbind,
};

static int ti_sn_bridge_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct ti_sn_bridge *pdata;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct ti_sn_bridge),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->regmap = devm_regmap_init_i2c(client,
					     &ti_sn_bridge_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		DRM_ERROR("regmap i2c init failed\n");
		return PTR_ERR(pdata->regmap);
	}

	pdata->dev = &client->dev;

	ret = drm_of_find_panel_or_bridge(pdata->dev->of_node, 1, 0,
					  &pdata->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return ret;
	}

	dev_set_drvdata(&client->dev, pdata);

	pdata->connector_override = of_property_read_bool(
				pdata->dev->of_node, "connector-override");

	pdata->continuous_splash = of_property_read_bool(
				pdata->dev->of_node, "continuous-splash");

	pdata->enable_gpio = devm_gpiod_get(pdata->dev, "enable",
					    GPIOD_ASIS);
	if (IS_ERR(pdata->enable_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->enable_gpio);
		return ret;
	}

	if (gpiod_get_direction(pdata->enable_gpio) != GPIOF_DIR_OUT)
		gpiod_direction_output(pdata->enable_gpio, GPIOD_OUT_LOW);


	pdata->pwr_gpio = devm_gpiod_get(pdata->dev, "platform-pwr",
					    GPIOD_ASIS);
	if (IS_ERR(pdata->pwr_gpio)) {
		DRM_ERROR("failed to get platform-pwr gpio from DT\n");
		ret = PTR_ERR(pdata->pwr_gpio);
		return ret;
	}

	if (gpiod_get_direction(pdata->pwr_gpio) != GPIOF_DIR_OUT)
		gpiod_direction_output(pdata->pwr_gpio, GPIOD_OUT_LOW);

	ret = ti_sn_bridge_parse_regulators(pdata);
	if (ret) {
		DRM_ERROR("failed to parse regulators\n");
		return ret;
	}

	pdata->refclk = devm_clk_get(pdata->dev, "refclk");
	if (IS_ERR(pdata->refclk)) {
		ret = PTR_ERR(pdata->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		DRM_DEBUG_KMS("refclk not found\n");
		pdata->refclk = NULL;
	}

	ret = ti_sn_bridge_parse_dsi_host(pdata);
	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->aux.name = "ti-sn65dsi86-aux";
	pdata->aux.dev = pdata->dev;
	pdata->aux.transfer = ti_sn_aux_transfer;
	drm_dp_aux_register(&pdata->aux);

	pdata->bridge.funcs = &ti_sn_bridge_funcs;
	pdata->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&pdata->bridge);

	ret = component_add(pdata->dev, &ti_sn_comp_ops);
	if (ret)
		pr_err("component add failed, rc=%d\n", ret);

	ti_sn_debugfs_init(pdata);

	DRM_DEBUG("probe success\n");
	return 0;
}

static int ti_sn_bridge_remove(struct i2c_client *client)
{
	struct ti_sn_bridge *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	ti_sn_debugfs_remove(pdata);

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id ti_sn_bridge_id[] = {
	{ "ti,sn65dsi86", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ti_sn_bridge_id);

static const struct of_device_id ti_sn_bridge_match_table[] = {
	{.compatible = "ti,sn65dsi86"},
	{},
};
MODULE_DEVICE_TABLE(of, ti_sn_bridge_match_table);

static struct i2c_driver ti_sn_bridge_driver = {
	.driver = {
		.name = "ti_sn65dsi86",
		.of_match_table = ti_sn_bridge_match_table,
		.pm = &ti_sn_bridge_pm_ops,
	},
	.probe = ti_sn_bridge_probe,
	.remove = ti_sn_bridge_remove,
	.id_table = ti_sn_bridge_id,
};
module_i2c_driver(ti_sn_bridge_driver);

MODULE_AUTHOR("Sandeep Panda <spanda@codeaurora.org>");
MODULE_DESCRIPTION("sn65dsi86 DSI to eDP bridge driver");
MODULE_LICENSE("GPL v2");
