/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#define LT8911_REGULATOR_SUPPLY_NUM		2

/*
 * AUX Related definition
 */
#define LT8911_AUX_CMD_WRITE			0x80
#define LT8911_AUX_CMD_READ			0x90
#define LT8911_AUX_MAX_MSG_LENGTH		16

/*
 * Register Definition
 */
#define LT8911_AUX_ADDR_REG			0xa62b
#define LT8911_PLL_LOCK_REG			0x8737

/*
 * DP related definition
 */
#define DP_SYMBOL_ERROR_COUNT_LANE0		0x210
#define DP_SYMBOL_ERROR_COUNT_LANE1		0x212
#define DP_SYMBOL_ERROR_COUNT_LANE2		0x214
#define DP_SYMBOL_ERROR_COUNT_LANE3		0x216

#define LT8911_PAGE_CONTROL			0xff
#define LT8911_LINK_TRAIN_CHECK_LOOP_COUNT	10

#define LT8911_DEFINE_DEBUGFS_FUNC(__name__)				\
static int __name__ ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name__ ## _show, inode->i_private);	\
}									\
static const struct file_operations __name__ ## _fops = {		\
	.owner = THIS_MODULE,						\
	.open = __name__ ## _open,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = single_release,					\
}

enum edp_lane_cnt {
	LANE_CNT1 = 1,
	LANE_CNT2 = 2,
	LANE_CNT4 = 4,
};

enum edp_color_depth {
	COLOR_6BIT,
	COLOR_8BIT,
};

enum dither_mode {
	DITHER_8B6B = 0xd0,
	DITHER_DISABLED = 0x00,
};

enum edp_video_mode {
	VIDEO_TEST = 0x50,
	VIDEO_NORMAL = 0x10,
};

/*******************************************************
 * 1. LT8911EXB I2C device address
 *    a) the address is depending on pin31(ADDR),
 *       if pin31(ADDR) is 3.3V, the address is 0x2d(7bit).
 *       if pin31(ADDR) is 0V, the address is 0x29(7bit).
 * 2. IIC data rate should less than 100Kbps.
 * 3. Should reset LT8911EXB by pull-low(keep more than 30ms)
 *    and then pull-high pin(RSTN) .
 * 5. Requirement for MIPI source:
 *    a) MIPI DSI.
 *    b) Video mode.
 *    c) non-burst mod.
 *    d) sync event or puls.
 *    e) clock should be continuous.
 *********************************************************/
struct lt8911exb {
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
	struct gpio_desc		*reset_gpio;
	enum edp_lane_cnt		edp_lane_cnt;
	enum edp_color_depth		edp_color_depth;
	struct regulator_bulk_data	supplies[LT8911_REGULATOR_SUPPLY_NUM];
};

static const struct regmap_range_cfg lt8911exb_ranges[] = {
	{
		.name = "register_range",
		.range_min =  0,
		.range_max = 0xdcff,
		.selector_reg = LT8911_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt8911exb_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt8911exb_ranges,
	.num_ranges = ARRAY_SIZE(lt8911exb_ranges),
};

/*
 * Power on sequence:
 *      VDD ON -> 1ms -> VCC ON -> 1ms -> Reset High
 *      Total should be 15ms or higher
 */
static int __maybe_unused lt8911exb_resume(struct device *dev)
{
	struct lt8911exb *pdata = dev_get_drvdata(dev);
	int ret, i;

	if (!pdata->panel_attached || !pdata->lp11_init)
		return 0;

	for (i = 0; i < LT8911_REGULATOR_SUPPLY_NUM; i++) {
		ret = regulator_enable(pdata->supplies[i].consumer);

		if (ret) {
			DRM_ERROR("failed to enable supplies[%d] %d\n", i, ret);
			for (i = i - 1; i >= 0; i--)
				regulator_disable(pdata->supplies[i].consumer);
			return ret;
		}
	}

	usleep_range(10000, 11000);

	gpiod_set_value(pdata->reset_gpio, 1);

	DRM_DEBUG("Resume returning %d\n", ret);
	return ret;
}

/*
 * Power off sequence
 *      VCC Off -> 10ms -> VDD off. Reset doesn't matter.
 */
static int __maybe_unused lt8911exb_suspend(struct device *dev)
{
	struct lt8911exb *pdata = dev_get_drvdata(dev);
	int ret;

	if (!pdata->panel_attached)
		return 0;

	gpiod_set_value(pdata->reset_gpio, 0);

	usleep_range(1000, 2000);

	ret = regulator_bulk_disable(LT8911_REGULATOR_SUPPLY_NUM,
						pdata->supplies);
	if (ret)
		DRM_ERROR("failed to disable supplies %d\n", ret);

	DRM_DEBUG("Suspend returning %d\n", ret);
	return ret;
}

static const struct dev_pm_ops lt8911exb_pm_ops = {
	SET_RUNTIME_PM_OPS(lt8911exb_suspend, lt8911exb_resume, NULL)
};

static int status_show(struct seq_file *s, void *data)
{
	struct lt8911exb *pdata = s->private;
	unsigned char value[3];
	unsigned int reg, val;
	unsigned int pages[10] = { 0x81, 0x82, 0x85, 0x87, 0xA0,
				  0xA6, 0xA8, 0xAC, 0xD0, 0xD8 };
	unsigned int dpcd_reg[2] = { 0x206, 0x207 };
	int i, j, retval;

	seq_puts(s, "DUMP REGISTERS:\n");

	pm_runtime_get_sync(pdata->dev);

	for (j = 0; j < 10; j++) {
		seq_printf(s, "====DUMP (0x%04x) Page =====\n", pages[j]<<8);
		for (i = 0; i < 0xff; i++) {
			reg = (pages[j] << 8 | i);
			if (regmap_read(pdata->regmap, reg, &val)) {
				pr_err("Failed to read reg 0x%x\n", reg);
				continue;
			}
			seq_printf(s, "%04x: %02x\n", reg, val);
		}
	}

	seq_puts(s, "DPCD REGISTERS:\n");
	for (i = 0; i < 2; i++) {
		retval = drm_dp_dpcd_read(&pdata->aux, dpcd_reg[i], value, 1);
		if (retval != 1) {
			pr_err("Failed to read reg %d\n", dpcd_reg[i]);
			continue;
		}

		seq_printf(s, "[%02d] = %02d\n",
				dpcd_reg[i], value[0]);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}
LT8911_DEFINE_DEBUGFS_FUNC(status);

static int symbol_err_cnt_show(struct seq_file *s, void *data)
{
	struct lt8911exb *pdata = s->private;
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
LT8911_DEFINE_DEBUGFS_FUNC(symbol_err_cnt);

static int test_crc_show(struct seq_file *s, void *data)
{
	struct lt8911exb *pdata = s->private;
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
LT8911_DEFINE_DEBUGFS_FUNC(test_crc);

static int test_crc_enable_get(void *data, u64 *val)
{
	int ret = -EINVAL;
	struct lt8911exb *pdata = data;
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
	struct lt8911exb *pdata = data;

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

static int pattern_enable_get(void *data, u64 *val)
{
	struct lt8911exb *pdata = data;
	unsigned int value;
	int ret;

	pm_runtime_get_sync(pdata->dev);
	ret = regmap_read(pdata->regmap, 0xa827, &value);
	pm_runtime_put(pdata->dev);

	if (!ret)
		*val = !!(value & 0x40);

	return ret;
}

static int pattern_enable_set(void *data, u64 val)
{
	struct lt8911exb *pdata = data;
	unsigned int value = 0;
	int ret = 0;

	/*
	 * 0: disable pattern
	 * 1: Use pixel data from register
	 * 2: black and white check box
	 * 3: white crossing in black background
	 * 4: black and white vertical bar
	 * 5: black and white horizontal bar
	 * 6: black wire in white background
	 */
	pm_runtime_get_sync(pdata->dev);
	if (val) {
		value = val - 1;
		if (value > 6)
			value = 6;
		ret = regmap_write(pdata->regmap, 0xa824, 0x50 | value);
		if (ret) {
			DRM_ERROR("pattern enable failed (0xa824) %d\n", ret);
			goto exit;
		}
		ret = regmap_write(pdata->regmap, 0xd026, 0x97);
		if (ret) {
			DRM_ERROR("pattern enable failed (0xd026) %d\n", ret);
			goto exit;
		}
		ret = regmap_write(pdata->regmap, 0xa827, 0x50);
		if (ret) {
			DRM_ERROR("pattern enable failed (0xa827) %d\n", ret);
			goto exit;
		}

		pr_info("pattern=0x%llx enabled\n", val);
	} else {
		ret = regmap_write(pdata->regmap, 0xa824, 0x50);
		if (ret) {
			DRM_ERROR("pattern disable failed (0xa824) %d\n", ret);
			goto exit;
		}
		ret = regmap_write(pdata->regmap, 0xd026, 0x17);
		if (ret) {
			DRM_ERROR("pattern disable failed (0xd026) %d\n", ret);
			goto exit;
		}
		ret = regmap_write(pdata->regmap, 0xa827, 0x10);
		if (ret) {
			DRM_ERROR("pattern disable failed (0xa827) %d\n", ret);
			goto exit;
		}
	}

exit:
	pm_runtime_put(pdata->dev);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(pattern_enable_fops, pattern_enable_get,
			pattern_enable_set, "%llu\n");

static int dsi_error_get(void *data, u64 *val)
{
	struct lt8911exb *pdata = data;
	unsigned int value;
	int ret;

	pm_runtime_get_sync(pdata->dev);
	ret = regmap_read(pdata->regmap, 0xd084, &value);
	pm_runtime_put(pdata->dev);

	if (!ret)
		*val = value;

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(dsi_error_fops, dsi_error_get,
			NULL, "%llx\n");

static int video_check_show(struct seq_file *s, void *data)
{
	struct lt8911exb *pdata = s->private;
	unsigned int val = 0;
	u32 reg = 0;

	pm_runtime_get_sync(pdata->dev);
	/* mipi byte clk check*/
	regmap_write(pdata->regmap, 0x851d, 0x00); /* FM select byte clk */
	regmap_write(pdata->regmap, 0x8540, 0xf7);
	regmap_write(pdata->regmap, 0x8541, 0x30);
	regmap_write(pdata->regmap, 0x85a1, 0x02); /* video chech from mipi */
	regmap_write(pdata->regmap, 0x8109, 0x7d); /* video check rst */
	regmap_write(pdata->regmap, 0x8109, 0xfd);

	msleep(30);

	regmap_read(pdata->regmap, 0x8550, &val);
	if (val == 0x03) {
		regmap_read(pdata->regmap, 0x854d, &val);
		reg = val;
		regmap_read(pdata->regmap, 0x854e, &val);
		reg = reg * 256 + val;
		regmap_read(pdata->regmap, 0x854f, &val);
		reg = reg * 256 + val;

		seq_printf(s, "mipi hs clk: %dkhz\n", reg);
	} else {
		seq_puts(s, "mipi clk unstable\n");
	}

	/* mipi vtotal check*/
	regmap_read(pdata->regmap, 0x8576, &val);
	reg = val;
	regmap_read(pdata->regmap, 0x8577, &val);
	reg = reg * 256 + val;

	seq_printf(s, "vtotal: %d\n", reg);

	/* mipi word count check*/
	regmap_read(pdata->regmap, 0xd082, &val);
	reg = val;
	regmap_read(pdata->regmap, 0xd083, &val);
	reg = reg * 256 + val;
	reg = reg / 3;

	seq_printf(s, "hact: %d\n", reg);

	/* mipi Vact check*/
	regmap_read(pdata->regmap, 0xd085, &val);
	reg = val;
	regmap_read(pdata->regmap, 0xd086, &val);
	reg = reg * 256 + val;

	seq_printf(s, "vact: %d\n", reg);

	/* mipi video data type*/
	regmap_read(pdata->regmap, 0xd084, &val);
	seq_printf(s, "mipi data type: 0x%x\n", val);

	/* mipi lane settle and SOT*/
	regmap_read(pdata->regmap, 0xd088, &val);
	seq_printf(s, "lane0 settle: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08a, &val);
	seq_printf(s, "lane1 settle: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08c, &val);
	seq_printf(s, "lane2 settle: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08e, &val);
	seq_printf(s, "lane3 settle: 0x%x\n", val);

	regmap_read(pdata->regmap, 0xd089, &val);
	seq_printf(s, "lane0 sot: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08b, &val);
	seq_printf(s, "lane1 sot: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08d, &val);
	seq_printf(s, "lane2 sot: 0x%x\n", val);
	regmap_read(pdata->regmap, 0xd08f, &val);
	seq_printf(s, "lane3 sot: 0x%x\n", val);
	pm_runtime_put(pdata->dev);

	return 0;
}
LT8911_DEFINE_DEBUGFS_FUNC(video_check);

static int htotal_check_show(struct seq_file *s, void *data)
{
	int i;
	u32 reg = 0;
	unsigned int val = 0;
	struct lt8911exb *pdata = s->private;

	pm_runtime_get_sync(pdata->dev);
	for (i = 0; i < 30; i++) {
		regmap_read(pdata->regmap, 0x8590, &val);
		reg = val;
		regmap_read(pdata->regmap, 0x8591, &val);
		reg = reg * 256 + val;

		seq_printf(s, "htotal in sysclk: %d\n", reg);
		msleep(5);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}
LT8911_DEFINE_DEBUGFS_FUNC(htotal_check);

static int pcr_check_show(struct seq_file *s, void *data)
{
	int i;
	struct lt8911exb *pdata = s->private;
	unsigned int reg_d094h, reg_d095h, reg_d096h, reg_d097h;

	pm_runtime_get_sync(pdata->dev);
	for (i = 0; i < 30; i++) {
		regmap_read(pdata->regmap, 0xd094, &reg_d094h);
		regmap_read(pdata->regmap, 0xd095, &reg_d095h);
		regmap_read(pdata->regmap, 0xd096, &reg_d096h);
		regmap_read(pdata->regmap, 0xd097, &reg_d097h);

		seq_printf(s, "mk: 0x%x, 0x%x, 0x%x, 0x%x,\n",
			reg_d094h, reg_d095h, reg_d096h, reg_d097h);
		msleep(5);
	}
	pm_runtime_put(pdata->dev);

	return 0;
}
LT8911_DEFINE_DEBUGFS_FUNC(pcr_check);

static void lt8911exb_debugfs_init(struct lt8911exb *pdata)
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
	debugfs_create_file("pattern_enable", 0600, pdata->debugfs, pdata,
			&pattern_enable_fops);
	debugfs_create_file("dsi_error", 0600, pdata->debugfs, pdata,
			&dsi_error_fops);
	debugfs_create_file("video_check", 0600, pdata->debugfs, pdata,
			&video_check_fops);
	debugfs_create_file("htotal_check", 0600, pdata->debugfs, pdata,
			&htotal_check_fops);
	debugfs_create_file("pcr_check", 0600, pdata->debugfs, pdata,
			&pcr_check_fops);
}

static void lt8911exb_debugfs_remove(struct lt8911exb *pdata)
{
	debugfs_remove_recursive(pdata->debugfs);
	pdata->debugfs = NULL;
}

/* Connector funcs */
static struct lt8911exb *bridge_to_lt8911exb(struct drm_bridge *bridge)
{
	return container_of(bridge, struct lt8911exb, bridge);
}

static struct lt8911exb *
connector_to_lt8911exb(struct drm_connector *connector)
{
	struct lt8911exb *pdata = NULL;

	if (connector->private)
		pdata = bridge_to_lt8911exb(connector->private);

	if (pdata && pdata->connector_override)
		return pdata;
	else
		return container_of(connector, struct lt8911exb, connector);
}

static int lt8911exb_connector_get_modes(struct drm_connector *connector)
{
	struct lt8911exb *pdata = connector_to_lt8911exb(connector);

	DRM_DEBUG("Calling drm_panel_get_modes()\n");
	return drm_panel_get_modes(pdata->panel);
}

static enum drm_mode_status
lt8911exb_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	DRM_DEBUG("mode->clock=%d\n", mode->clock);

	/* maximum supported resolution is 4K at 60 fps */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs lt8911exb_connector_helper_funcs = {
	.get_modes = lt8911exb_connector_get_modes,
	.mode_valid = lt8911exb_connector_mode_valid,
};

static enum drm_connector_status
lt8911exb_connector_detect(struct drm_connector *connector, bool force)
{
	/**
	 * TODO: Currently if drm_panel is present, then always
	 * return the status as connected. Need to add support to detect
	 * device state for hot pluggable scenarios.
	 */

	DRM_DEBUG("connector_status = %d\n", connector_status_connected);
	return connector_status_connected;
}

static const struct drm_connector_funcs lt8911exb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = lt8911exb_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int lt8911exb_parse_regulators(struct lt8911exb *pdata)
{
	unsigned int i;
	const char * const lt8911exb_supply_names[] = {
		"vdd", "vcc",
	};
	int ret;

	for (i = 0; i < LT8911_REGULATOR_SUPPLY_NUM; i++)
		pdata->supplies[i].supply = lt8911exb_supply_names[i];

	ret = devm_regulator_bulk_get(pdata->dev, LT8911_REGULATOR_SUPPLY_NUM,
				       pdata->supplies);
	if (ret) {
		DRM_ERROR("%s: regulator get error\n", __func__);
		return ret;
	}

	if (!pdata->lp11_init && pdata->continuous_splash) {
		/* Need to enable the regulstors here */
		ret = regulator_bulk_enable(LT8911_REGULATOR_SUPPLY_NUM,
						pdata->supplies);
		if (ret)
			DRM_ERROR("failed to enable supplies %d\n", ret);
	}

	return ret;
}

static int lt8911exb_attach(struct drm_bridge *bridge)
{
	int ret;
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "lt8911exb",
						   .channel = 0,
						   .node = NULL,
						 };

	if (!pdata->connector_override) {
		ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &lt8911exb_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
		if (ret) {
			DRM_ERROR("Failed to initialize connector with drm\n");
			return ret;
		}

		drm_connector_helper_add(&pdata->connector,
				 &lt8911exb_connector_helper_funcs);
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

static void lt8911exb_disable(struct drm_bridge *bridge)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

	DRM_DEBUG("Disabling bridge\n");
	drm_panel_disable(pdata->panel);

	/* disable eDP tx */
	regmap_write(pdata->regmap, 0x8212, 0);

	drm_panel_unprepare(pdata->panel);
}

static void lt8911exb_set_refclk_freq(struct lt8911exb *pdata)
{
	if (pdata->refclk)
		clk_prepare_enable(pdata->refclk);
}

static void lt8911exb_mipi_video_cfg(struct lt8911exb *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/* vtotal */
	regmap_write(pdata->regmap, 0xd00d, mode->vtotal / 256);
	regmap_write(pdata->regmap, 0xd00e, mode->vtotal % 256);

	/* vactive */
	regmap_write(pdata->regmap, 0xd00f, mode->vdisplay / 256);
	regmap_write(pdata->regmap, 0xd010, mode->vdisplay % 256);

	/* htotal */
	regmap_write(pdata->regmap, 0xd011, mode->htotal / 256);
	regmap_write(pdata->regmap, 0xd012, mode->htotal % 256);

	/* hactive */
	regmap_write(pdata->regmap, 0xd013, mode->hdisplay / 256);
	regmap_write(pdata->regmap, 0xd014, mode->hdisplay % 256);

	/* virtical sync */
	regmap_write(pdata->regmap, 0xd015,
			(mode->vsync_end - mode->vsync_start) & 0xFF);

	/* horizontal sync */
	regmap_write(pdata->regmap, 0xd016,
			(mode->hsync_end - mode->hsync_start) & 0xFF);

	/* VFP */
	regmap_write(pdata->regmap, 0xd017,
			(mode->vsync_start - mode->vdisplay) / 256);
	regmap_write(pdata->regmap, 0xd018,
			(mode->vsync_start - mode->vdisplay) % 256);

	/* HFP */
	regmap_write(pdata->regmap, 0xd019,
			(mode->hsync_start - mode->hdisplay) / 256);
	regmap_write(pdata->regmap, 0xd01a,
			(mode->hsync_start - mode->hdisplay) % 256);
}

static void lt8911exb_edp_video_cfg(struct lt8911exb *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	int value;

	/* Use video timing from register */
	regmap_write(pdata->regmap, 0xa82d, 0x88);

	/* htotal */
	regmap_write(pdata->regmap, 0xa805, mode->htotal / 256);
	regmap_write(pdata->regmap, 0xa806, mode->htotal % 256);

	/* h_start = hsync + hbp */
	value = (mode->hsync_end - mode->hsync_start) +
			(mode->htotal - mode->hsync_end);
	regmap_write(pdata->regmap, 0xa807, value / 256);
	regmap_write(pdata->regmap, 0xa808, value % 256);

	/* horizontal sync */
	regmap_write(pdata->regmap, 0xa809,
			(mode->hsync_end - mode->hsync_start) / 256);
	regmap_write(pdata->regmap, 0xa80a,
			(mode->hsync_end - mode->hsync_start) % 256);

	/* hactive */
	regmap_write(pdata->regmap, 0xa80b, mode->hdisplay / 256);
	regmap_write(pdata->regmap, 0xa80c, mode->hdisplay % 256);

	/* vtotal */
	regmap_write(pdata->regmap, 0xa80d, mode->vtotal / 256);
	regmap_write(pdata->regmap, 0xa80e, mode->vtotal % 256);

	/* v_start = vsync + vbp */
	value = (mode->vsync_end - mode->vsync_start) +
			(mode->vtotal - mode->vsync_end);
	regmap_write(pdata->regmap, 0xa811, value / 256);
	regmap_write(pdata->regmap, 0xa812, value % 256);

	/* virtical sync */
	regmap_write(pdata->regmap, 0xa814,
			(mode->vsync_end - mode->vsync_start) & 0xFF);

	/* vactive */
	regmap_write(pdata->regmap, 0xa815, mode->vdisplay / 256);
	regmap_write(pdata->regmap, 0xa816, mode->vdisplay % 256);
}

static u32 lt8911exb_get_pixel_freq(struct lt8911exb *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	return mode->clock;
}

static int lt8911exb_reg_init(struct lt8911exb *pdata)
{
	int i;
	int ret;
	unsigned int val;
	u8 pcr_m;
	u8 pcr_pll_postdiv;
	u32 pixel_clk = lt8911exb_get_pixel_freq(pdata);

	/* By default 4 lanes are enabled */
	const struct reg_sequence seq1[] = {
		{ 0x8105, 0x06 }, /* SW reset */
		{ 0x8143, 0x00 },
		{ 0x8144, 0x1f },
		{ 0x8149, 0x7f },

		{ 0x825a, 0x0e }, /* GPIO test output */

		{ 0x851d, 0x00 }, /* Input clock: Ad_mlrxa_byte_clk */
		{ 0x8540, 0xf7 }, /* RGD_CLK_DET_THR to 32.767MHz */
		{ 0x8541, 0x30 }, /* RGD_FREQ_CHG_THR_SEL 511KHz */

		{ 0x8212, 0xff }, /* default: 4 MIPI lane */

		/* mipi Rx analog */
		{ 0x8232, 0x51 },
		{ 0x8235, 0x62 }, /* MIPI LVDS reference current: 37.5uA */
		{ 0x823a, 0x77 }, /* MIPI lane 0/1 rx clk lane eq: 12.5dB */
		{ 0x823b, 0x77 }, /* MIPI lane 2/3 rx clk lane eq: 12.5dB */
		{ 0x824c, 0x0c },
		{ 0x824d, 0x00 },

		{ 0x826a, 0x40 },
		{ 0x826b, 0x40 },
	};

	const struct reg_sequence seq2[] = {
		/* dessc pll digital */
		{ 0x85a9, 0x31 },
		{ 0x85aa, 0x17 },
		{ 0x85ab, 0xba },
		{ 0x85ac, 0xe1 },
		{ 0x85ad, 0x47 },
		{ 0x85ae, 0x01 },
		{ 0x85ae, 0x11 },

		/* Digital Top */
		{ 0x85c0, 0x01 },
		{ 0x85b0, 0x00 }, /* default 8bit, disable dither mode */

		/* mipi Rx Digital */
		{ 0xd000, 0x00 }, /* default 4 lanes */
		{ 0xd002, 0x08 }, /* settle */
		{ 0xd008, 0x00 }, /* BGR */

		{ 0xd00c, 0x80 }, /* fifo position */
		{ 0xd01c, 0x80 },
		{ 0xd024, 0x70 }, /* pcr mode */

		{ 0xd02d, 0x30 },
		{ 0xd031, 0x0a },

		/* stage1 hs mode */
		{ 0xd025, 0x90 }, /* line limit */
		{ 0xd02a, 0x3a }, /* step in limit */
		{ 0xd021, 0x4f }, /* hs_step */
		{ 0xd022, 0xff },

		/* stage2 de mode */
		{ 0xd00a, 0x02 }, /* de adjust pre line */
		{ 0xd038, 0x02 }, /* de_threshold 1 */
		{ 0xd039, 0x04 }, /* de_threshold 2 */
		{ 0xd03a, 0x08 }, /* de_threshold 3 */
		{ 0xd03b, 0x10 }, /* de_threshold 4 */
		{ 0xd03f, 0x02 }, /* de_step 1 */
		{ 0xd040, 0x04 }, /* de_step 2 */
		{ 0xd041, 0x08 }, /* de_step 3 */
		{ 0xd042, 0x10 }, /* de_step 4 */

		/*stage2 hs mode*/
		{ 0xd01e, 0x01 }, /* hs threshold */
		{ 0xd023, 0xf0 }, /* hs step */
		{ 0xd02b, 0x80 }, /* stable out */
	};

	const struct reg_sequence seq3[] = {
		{ 0x8103, 0x7b }, /* PCR reset */
		{ 0x8103, 0xff },

		{ 0x8719, 0x31 },
		{ 0x8202, 0x42 },
		{ 0x8203, 0x00 },
		{ 0x8203, 0x01 },
		{ 0x8109, 0xfc }, /* pll reset */
		{ 0x8109, 0xfd },
		{ 0x870c, 0x11 }, /* cal en = 1 */
	};

	const struct reg_sequence seq4[] = {
		/* tx phy */
		{ 0x8211, 0x00 },
		{ 0x8213, 0x10 },
		{ 0x8214, 0x0c },
		{ 0x8214, 0x08 },
		{ 0x8213, 0x30 },
		{ 0x820e, 0x25 },
		{ 0x8212, 0xff },
		{ 0x8040, 0x22 },
	};

	ret = regmap_multi_reg_write(pdata->regmap, seq1, ARRAY_SIZE(seq1));
	if (ret)
		return ret;

	if (pdata->dsi->lanes == LANE_CNT2) {
		regmap_write(pdata->regmap, 0x8145, 0xf7);
		regmap_write(pdata->regmap, 0x8146, 0xf6);
		regmap_write(pdata->regmap, 0x8212, 0x33);
	}

	if (pixel_clk < 88000) {
		/* 0x44:pre-div = 2, pixel_clk=44~ 88MHz */
		regmap_write(pdata->regmap, 0x826e, 0x82);
		pcr_pll_postdiv = 0x08;
	} else {
		/* 0x40:pre-div = 1, pixel_clk =88~176MHz */
		regmap_write(pdata->regmap, 0x826e, 0x81);
		pcr_pll_postdiv = 0x04;
	}

	pcr_m = pixel_clk * pcr_pll_postdiv / 25 / 1000;
	pcr_m++;  /* V1.7: pcr_m should over than actual pcr_m */
	DRM_DEBUG("%s: pcr_m = 0x%x\n", __func__, pcr_m);

	ret = regmap_multi_reg_write(pdata->regmap, seq2, ARRAY_SIZE(seq2));
	if (ret)
		return ret;

	if (pdata->dsi->lanes == LANE_CNT2)
		regmap_write(pdata->regmap, 0xd000, 0x02);

	ret = regmap_write(pdata->regmap, 0xd026, pcr_m);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(pdata->regmap, seq3, ARRAY_SIZE(seq3));
	if (ret)
		return ret;

	for (i = 0; i < 5; i++) { /* Check Tx PLL */
		usleep_range(5000, 6000);
		ret = regmap_read(pdata->regmap, LT8911_PLL_LOCK_REG, &val);
		if (ret)
			continue;

		if (val & 0x02) {
			DRM_INFO("%s: LT8911 tx pll locked\n", __func__);
			break;
		}

		DRM_DEBUG("%s: LT8911 tx pll unlocked\n", __func__);
		regmap_write(pdata->regmap, 0x870c, 0x10); /* cal en = 0 */
		regmap_write(pdata->regmap, 0x8109, 0xfc); /* pll reset */
		regmap_write(pdata->regmap, 0x8109, 0xfd);
		regmap_write(pdata->regmap, 0x870c, 0x11); /* cal en = 1 */
	}

	ret = regmap_multi_reg_write(pdata->regmap, seq4, ARRAY_SIZE(seq4));
	if (ret)
		return ret;

	/* eDP Tx Digital */
	ret = regmap_write(pdata->regmap, 0xa827, VIDEO_NORMAL);
	if (ret)
		return ret;

	if (pdata->edp_color_depth == COLOR_6BIT) {
		ret = regmap_write(pdata->regmap, 0x85b0, DITHER_8B6B);
		if (ret)
			return ret;

		ret = regmap_write(pdata->regmap, 0xa817, 0x00);
		if (ret)
			return ret;

		ret = regmap_write(pdata->regmap, 0xa818, 0x00);
		if (ret)
			return ret;
	}

	ret = regmap_write(pdata->regmap, 0xa000, 0x08);
	if (ret)
		return ret;

	ret = regmap_write(pdata->regmap, 0xa001, 0x00);
	if (ret)
		return ret;

	return 0;
}

static int lt8911exb_tx_swing_preset(struct lt8911exb *pdata)
{
	int ret;

	const struct reg_sequence lane01[] = {
		{ 0x8222, 0x81 }, /* lane 0 swing */
		{ 0x8223, 0xa0 },
		{ 0x8224, 0x80 }, /* lane 0 pre-emphase */
		{ 0x8225, 0x00 },

		{ 0x8226, 0x81 }, /* lane 1 swing */
		{ 0x8227, 0xa0 },
		{ 0x8228, 0x80 }, /* lane 1 pre-emphase */
		{ 0x8229, 0x00 },
	};

	const struct reg_sequence lane23[] = {
		{ 0x822a, 0x81 }, /* lane 2 swing */
		{ 0x822b, 0xa0 },
		{ 0x822c, 0x80 }, /* lane 2 pre-emphase */
		{ 0x822d, 0x00 },

		{ 0x822e, 0x81 }, /* lane 3 swing */
		{ 0x822f, 0xa0 },
		{ 0x8230, 0x80 }, /* lane 3 pre-emphase */
		{ 0x8231, 0x00 },
	};

	ret = regmap_multi_reg_write(pdata->regmap, lane01, ARRAY_SIZE(lane01));
	if (ret)
		return ret;

	if (pdata->edp_lane_cnt == LANE_CNT4) {
		ret = regmap_multi_reg_write(pdata->regmap, lane23,
							ARRAY_SIZE(lane23));
		if (ret)
			return ret;
	}

	return ret;
}

static int lt8911exb_link_train(struct lt8911exb *pdata)
{
	int ret;

	const struct reg_sequence seq1[] = {
		{ 0xa62a, 0x00 },
		{ 0x8107, 0xfe },
		{ 0x8107, 0xff },
		{ 0x810a, 0xfc },
		{ 0x810a, 0xfe },
		{ 0xa82d, 0x80 },
		{ 0x8517, 0xc0 },
	};

	const struct reg_sequence seq2[] = {
		{ 0xac00, 0x60 },
		{ 0xac01, 0x0a },
		{ 0xac0c, 0x05 },
		{ 0xac0c, 0x45 },
	};

	ret = regmap_multi_reg_write(pdata->regmap, seq1, ARRAY_SIZE(seq1));
	if (ret)
		return ret;

	/* eDP number of lanes */
	ret = regmap_write(pdata->regmap, 0x851a,
				(unsigned int)pdata->edp_lane_cnt);
	if (ret)
		return ret;


	return regmap_multi_reg_write(pdata->regmap, seq2, ARRAY_SIZE(seq2));
}

static int lt8911exb_link_train_result_check(struct lt8911exb *pdata)
{
	int i;
	int ret;
	unsigned int val;

	for (i = 0; i < LT8911_LINK_TRAIN_CHECK_LOOP_COUNT; i++) {
		ret = regmap_read(pdata->regmap, 0xac82, &val);
		if (ret)
			continue;

		if (val & 0x20) {
			if ((val & 0x1f) == 0x1e) {
				ret = regmap_read(pdata->regmap, 0xac82, &val);
				if (ret)
					continue;
				DRM_INFO("%s: edp link train successed: 0x%x\n",
								__func__, val);
			} else {
				/* Soft link if link training fails */
				regmap_read(pdata->regmap, 0xac82, &val);
				DRM_ERROR("%s: edp link train failed: 0x%x\n",
								__func__, val);
				regmap_write(pdata->regmap, 0xac00, 0x00);
				regmap_write(pdata->regmap, 0xac01, 0x0a);
				regmap_write(pdata->regmap, 0xac14, 0x80);
				regmap_write(pdata->regmap, 0xac14, 0x81);
				msleep(30);
				ret = regmap_write(pdata->regmap, 0xac14, 0x84);
				msleep(30);
				ret = regmap_write(pdata->regmap, 0xac14, 0xc0);
				DRM_ERROR("%s: train fail but enable Video\n",
								__func__);
			}

			regmap_read(pdata->regmap, 0xac83, &val);
			DRM_INFO("%s: panel link rate: 0x%x\n", __func__, val);
			regmap_read(pdata->regmap, 0xac84, &val);
			DRM_INFO("%s: panel link count: 0x%x\n", __func__, val);
			break;
		}

		DRM_DEBUG("%s: link train on going...\n", __func__);
		msleep(100);
	}

	return ret;
}

static void lt8911exb_enable(struct drm_bridge *bridge)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

	/* Power on the bridge if not powered on by the host DSI */
	if (!pdata->lp11_init)
		pm_runtime_get_sync(pdata->dev);

	/* configure bridge ref_clk */
	lt8911exb_set_refclk_freq(pdata);

	/*
	 * If continuous_splash is defined, skip remaining init
	 */
	if (!pdata->lp11_init && pdata->continuous_splash)
		return;

	lt8911exb_mipi_video_cfg(pdata);
	lt8911exb_edp_video_cfg(pdata);
	lt8911exb_reg_init(pdata);
	lt8911exb_tx_swing_preset(pdata);
	lt8911exb_link_train(pdata);
	lt8911exb_link_train_result_check(pdata);

	drm_panel_enable(pdata->panel);
	DRM_DEBUG("Enable success\n");
}

static void lt8911exb_pre_enable(struct drm_bridge *bridge)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

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

static void lt8911exb_post_disable(struct drm_bridge *bridge)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);

	pm_runtime_put_sync(pdata->dev);

	DRM_DEBUG("Bridge is disabled\n");
}

static struct drm_connector_funcs override_funcs;
static struct drm_connector_helper_funcs override_helper_private;

static int lt8911exb_connector_init(struct drm_bridge *bridge,
					struct drm_connector *connector)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

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
	override_funcs.detect = lt8911exb_connector_detect;
	connector->funcs = &override_funcs;

	override_helper_private = *connector->helper_private;
	override_helper_private.get_modes = lt8911exb_connector_get_modes;
	override_helper_private.mode_valid = lt8911exb_connector_mode_valid;
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
static void lt8911exb_power_on(struct drm_bridge *bridge)
{
	struct lt8911exb *pdata = bridge_to_lt8911exb(bridge);

	if (!pdata)
		return;

	if (!pdata->lp11_init)
		pdata->lp11_init = 1;

	pm_runtime_get_sync(pdata->dev);
}

static const struct drm_bridge_funcs lt8911exb_funcs = {
	.attach = lt8911exb_attach,
	.pre_enable = lt8911exb_pre_enable,
	.enable = lt8911exb_enable,
	.disable = lt8911exb_disable,
	.post_disable = lt8911exb_post_disable,
	.connector_init = lt8911exb_connector_init,
	.power_on = lt8911exb_power_on,
};

static struct lt8911exb *aux_to_lt8911exb(struct drm_dp_aux *aux)
{
	return container_of(aux, struct lt8911exb, aux);
}



static ssize_t lt8911exb_aux_transfer(struct drm_dp_aux *aux,
				  struct drm_dp_aux_msg *msg)
{
	struct lt8911exb *pdata = aux_to_lt8911exb(aux);
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u8 *buf = (u8 *)msg->buffer;
	u8 cmd;
	int i;
	unsigned int val;
	int ret;

	/* TODO: currently only supports 1 byte size transfer */
	if (msg->size > LT8911_AUX_MAX_MSG_LENGTH)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
		cmd = LT8911_AUX_CMD_WRITE;
		break;
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		cmd = LT8911_AUX_CMD_READ;
		break;
	default:
		return -EINVAL;
	}

	/* Soft link train */
	regmap_write(pdata->regmap, 0xac00, 0);
	regmap_write(pdata->regmap, 0xa62a, 1);

	/* Set address and command */
	regmap_write(pdata->regmap, LT8911_AUX_ADDR_REG,
		     ((msg->address >> 16) & 0xF) | cmd); /* CMD */
	regmap_write(pdata->regmap, LT8911_AUX_ADDR_REG,
		     (msg->address >> 8) & 0xFF);
	regmap_write(pdata->regmap, LT8911_AUX_ADDR_REG,
			msg->address & 0xFF);
	/* data length = msg->size - 1 */
	regmap_write(pdata->regmap, LT8911_AUX_ADDR_REG, msg->size - 1);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++) {
			ret = regmap_write(pdata->regmap,
					LT8911_AUX_ADDR_REG, buf[i]);
			if (ret)
				return ret;
		}
	}

	regmap_write(pdata->regmap, 0xa62c, 0); /* start */

	msleep(20); /* more than 10ms wait */

	ret = regmap_read(pdata->regmap, 0xa625, &val);
	if (ret)
		return ret;

	switch (val & 0x0f) {
	case 0x0c:
		if (request == DP_AUX_NATIVE_WRITE ||
				request == DP_AUX_I2C_WRITE)
			break;

		ret = regmap_read(pdata->regmap, 0xa639, &val);
		if (ret)
			return ret;

		if (val & 0x20) {
			int size = val & 0x1F;

			if (size != msg->size + 1) {
				DRM_ERROR("Error: Incorrect size received\n");
				return -ENXIO;
			}

			/* Read ACK */
			ret = regmap_read(pdata->regmap, 0xa62b, &val);
			if (ret)
				return ret;

			for (i = 0; i < msg->size; i++) {
				ret = regmap_read(pdata->regmap, 0xa62b, &val);
				if (ret)
					return ret;
				buf[i] = (u8)(val & 0xFF);
			}
		} else {
			DRM_ERROR("Error: No Replay\n");
			return -ENXIO;
		}
		break;
	case 0x0a:
		DRM_ERROR("Error: Reply Nack\n");
		return -ENXIO;
	case 0x09:
		DRM_ERROR("Error: Reply Defer\n");
		return -ENXIO;
	default:
		DRM_ERROR("Error: No Reply\n");
		return -ENXIO;
	}

	return msg->size;
}

static int lt8911exb_parse_dsi_host(struct lt8911exb *pdata)
{
	struct device_node *np = pdata->dev->of_node;

	pdata->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!pdata->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}

	return 0;
}

static int lt8911exb_parse_panel(struct lt8911exb *pdata)
{
	int ret;
	struct device_node *endpoint;

	ret = drm_of_find_panel_or_bridge(pdata->dev->of_node, 1, 0,
					  &pdata->panel, NULL);
	if (ret) {
		DRM_ERROR("could not find any panel node\n");
		return ret;
	}

	pdata->edp_lane_cnt = LANE_CNT4; /* Set default 4 lane */

	endpoint = of_graph_get_endpoint_by_regs(pdata->dev->of_node, 1, -1);
	if (endpoint != NULL) {
		int data_lanes;

		data_lanes = of_property_count_u32_elems(endpoint,
							"data-lanes");
		of_node_put(endpoint);

		switch (data_lanes) {
		case LANE_CNT1:
		case LANE_CNT2:
		case LANE_CNT4:
			pdata->edp_lane_cnt = data_lanes;
			break;
		default:
			DRM_ERROR("Invalid data lanes %d\n", data_lanes);
			break;
		}
	}

	pdata->edp_color_depth = COLOR_8BIT; /* Default 8bit */
	if (of_property_read_bool(pdata->dev->of_node, "edp-color-6bit-mode"))
		pdata->edp_color_depth = COLOR_6BIT;

	return 0;
}

static int lt8911exb_read_device_rev(struct lt8911exb *pdata)
{
	unsigned int reg_8100h = 0;
	unsigned int reg_8101h = 0;
	unsigned int reg_8102h = 0;
	int ret = 0;

	ret = regmap_write(pdata->regmap, 0x8107, 0x7f);
	if (ret) {
		DRM_ERROR("read_device_rev failed\n");
		return ret;
	}

	ret = regmap_read(pdata->regmap, 0x8100, &reg_8100h);
	if (ret) {
		DRM_ERROR("read_device_rev failed\n");
		return ret;
	}

	ret = regmap_read(pdata->regmap, 0x8101, &reg_8101h);
	if (ret) {
		DRM_ERROR("read_device_rev failed\n");
		return ret;
	}

	ret = regmap_read(pdata->regmap, 0x8102, &reg_8102h);
	if (ret) {
		DRM_ERROR("read_device_rev failed\n");
		return ret;
	}

	DRM_INFO("%s: lt8911exb chip id: 0x%x, 0x%x, 0x%x\n",
		__func__, reg_8100h, reg_8101h, reg_8102h);

	return ret;
}

static int lt8911exb_parse_dt(struct lt8911exb *pdata)
{
	int ret;

	pdata->connector_override = of_property_read_bool(
				pdata->dev->of_node, "connector-override");

	pdata->continuous_splash = of_property_read_bool(
				pdata->dev->of_node, "continuous-splash");

	ret = lt8911exb_parse_regulators(pdata);
	if (ret) {
		DRM_ERROR("failed to parse regulators\n");
		return ret;
	}

	pdata->reset_gpio = devm_gpiod_get(pdata->dev, "reset",
					    GPIOD_ASIS);
	if (IS_ERR(pdata->reset_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(pdata->reset_gpio);
		return ret;
	}

	if (gpiod_get_direction(pdata->reset_gpio) != GPIOF_DIR_OUT)
		gpiod_direction_output(pdata->reset_gpio, GPIOD_OUT_HIGH);

	return 0;
}

static int lt8911exb_bind(struct device *dev,
				struct device *master, void *data)
{
	return 0;
}

static void lt8911exb_unbind(struct device *dev,
				struct device *master, void *data)
{
}

static const struct component_ops lt8911exb_comp_ops = {
	.bind = lt8911exb_bind,
	.unbind = lt8911exb_unbind,
};

static int lt8911exb_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct lt8911exb *pdata;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct lt8911exb),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->regmap = devm_regmap_init_i2c(client,
					     &lt8911exb_regmap_config);
	if (IS_ERR(pdata->regmap)) {
		DRM_ERROR("regmap i2c init failed\n");
		return PTR_ERR(pdata->regmap);
	}

	pdata->dev = &client->dev;
	dev_set_drvdata(&client->dev, pdata);

	ret = lt8911exb_parse_dt(pdata);
	if (ret) {
		DRM_ERROR("parse dt error\n");
		return ret;
	}

	ret = lt8911exb_read_device_rev(pdata);
	if (ret) {
		DRM_ERROR("bridge IC not supported\n");
		return ret;
	}

	ret = lt8911exb_parse_panel(pdata);
	if (ret) {
		DRM_ERROR("panel parse failed\n");
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

	ret = lt8911exb_parse_dsi_host(pdata);
	if (ret)
		return ret;

	pm_runtime_enable(pdata->dev);

	i2c_set_clientdata(client, pdata);

	pdata->aux.name = "lt8911exb-aux";
	pdata->aux.dev = pdata->dev;
	pdata->aux.transfer = lt8911exb_aux_transfer;
	drm_dp_aux_register(&pdata->aux);

	pdata->bridge.funcs = &lt8911exb_funcs;
	pdata->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&pdata->bridge);

	ret = component_add(pdata->dev, &lt8911exb_comp_ops);
	if (ret)
		pr_err("component add failed, rc=%d\n", ret);

	lt8911exb_debugfs_init(pdata);

	DRM_DEBUG("probe success\n");
	return 0;
}

static int lt8911exb_remove(struct i2c_client *client)
{
	struct lt8911exb *pdata = i2c_get_clientdata(client);

	if (!pdata)
		return -EINVAL;

	lt8911exb_debugfs_remove(pdata);

	of_node_put(pdata->host_node);

	pm_runtime_disable(pdata->dev);

	if (pdata->dsi) {
		mipi_dsi_detach(pdata->dsi);
		mipi_dsi_device_unregister(pdata->dsi);
	}

	drm_bridge_remove(&pdata->bridge);

	return 0;
}

static struct i2c_device_id lt8911exb_id[] = {
	{ "lt,lt8911exb", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, lt8911exb_id);

static const struct of_device_id lt8911exb_match_table[] = {
	{.compatible = "lt,lt8911exb"},
	{},
};
MODULE_DEVICE_TABLE(of, lt8911exb_match_table);

static struct i2c_driver lt8911exb_driver = {
	.driver = {
		.name = "lt8911exb",
		.of_match_table = lt8911exb_match_table,
		.pm = &lt8911exb_pm_ops,
	},
	.probe = lt8911exb_probe,
	.remove = lt8911exb_remove,
	.id_table = lt8911exb_id,
};
module_i2c_driver(lt8911exb_driver);

MODULE_DESCRIPTION("Lontium bridge IC LT8911exb");
MODULE_LICENSE("GPL v2");
