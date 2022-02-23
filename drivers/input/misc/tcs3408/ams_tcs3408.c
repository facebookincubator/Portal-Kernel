/*
 * Copyright by ams AG
 * All rights are reserved.
 *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING
 * THE SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY
 * EXCLUDED.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#include <linux/time.h>

#include <linux/i2c/ams/ams_tcs3408_reg.h>
#include <linux/i2c/ams/ams_tcs3408.h>
#include "ams_i2c.h"
#include "ams_tcs3408_als.h"
#include "ams_tcs3408_fifo.h"
#include "ams_tcs3408_flicker.h"

#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#undef TEST
#undef DEBUG

#ifdef TEST
static u8 refcnt;
#endif
/*
 * ID, REVID, AUXID
 */
struct device_ids {
	uint8_t device;
	uint8_t rev;
	uint8_t aux;
};

static struct device_ids const dev_ids[] = {
	{.device = 0x18, .rev = 0x00, .aux = 0x00},
	{.device = 0x18, .rev = 0x31, .aux = 0x02},
	{.device = 0x18, .rev = 0x11, .aux = 0x02},
	{.device = 0x18, .rev = 0x51, .aux = 0x02},
	{.device = 0x18, .rev = 0x01, .aux = 0xc3},
	{.device = 0x18, .rev = 0x11, .aux = 0x02},
	{.device = 0x18, .rev = 0x53, .aux = 0x48},
};

static char const *dev_names[] = {
	"tmd49073s",
	"tcs3701",
	"tcs3707",
	"tcs34077s",
	"tmd4910",
	"tcs3408",
	"tcs34087S",
};

static struct i2c_device_id tcs3408_idtable[] = {
	{ "tmd49073s", 0 },
	{ "tcs3701", 0 },
	{ "tcs3707", 0 },
	{ "tcs34077s", 0 },
	{ "tmd4910", 0 },
	{ "tcs3408", 0},
	{ "tcs34087S", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tcs3408_idtable);

int tcs3408_i2c_read(struct i2c_client *client, char reg, char *buf, int len)
{
	//What does this do? Where is it used?
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/*  SMUX	*/
enum als_channel_config {
	ALS_CH_CFG_1 = 1,
	ALS_CH_CFG_2,
	ALS_CH_CFG_3,
	ALS_CH_CFG_4,
	ALS_CH_CFG_5,
	ALS_CH_CFG_6,
	ALS_CH_CFG_MAX = ALS_CH_CFG_6,
	ALS_CH_CFG_DFLT = ALS_CH_CFG_5
};

#define SMUX_SIZE (20)
#define STR1_MAX (16)
#define STR2_MAX (256)

static uint8_t smux_colors[SMUX_SIZE] = {
	0x14, 0x25, 0x23, 0x41,
	0x33, 0x12, 0x14, 0x24,
	0x53, 0x23, 0x15, 0x14,
	0x32, 0x44, 0x21, 0x23,
	0x13, 0x54, 0x00, 0x76
};

enum smux_colors {
	SMUX_CLEAR = 1,
	SMUX_RED,
	SMUX_GREEN,
	SMUX_BLUE,
	SMUX_WIDEBAND,
	SMUX_FLICKER,
	SMUX_PROX,
};

static void get_smux_config(struct ams_chip *chip, uint8_t *cfg)
{
	uint8_t i, j, tmp, shift;

	for (i = 0; i < SMUX_SIZE; ++i) {
		for (j = 0; j < 2; ++j) {
			if (j)
				shift = 4;
			else
				shift = 0;
			tmp = ((smux_colors[i] & (0x0F << shift)) >> shift) - 1;
			cfg[i] |= (chip->channels[tmp] & 0x0F) << shift;
		}
	}
}

static void print_smux_config(struct ams_chip *chip, uint8_t *config)
{
	char strtmp[STR1_MAX] = "";
	char smux_str[STR2_MAX] = "";
	int i;

	for (i = 0; i < SMUX_SIZE; ++i) {
		snprintf(strtmp, ARRAY_SIZE(strtmp),
				"0x%02x ", config[i]);
		strlcat(smux_str, strtmp, ARRAY_SIZE(strtmp));
	}
	dev_info(&chip->client->dev, "smux: %s\n", smux_str);
}

static void set_smux_config(struct ams_chip *chip, uint8_t *config)
{
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_CFG6, 0x10);
	ams_i2c_reg_blk_write(chip->client,
			TCS3408_REG_RAM_START, config,
			SMUX_SIZE);
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_ENABLE, 0x11);
	udelay(200);
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_CFG6, 0x00);
};

static void configure_mux(struct ams_chip *chip)
{
	uint8_t mux_config[20] = { 0 };

	get_smux_config(chip, mux_config);
	print_smux_config(chip, mux_config);
	set_smux_config(chip, mux_config);
}

static void channel_config_work(struct work_struct *work)
{
	u8 save = 0;

	if (work) {
		struct ams_chip *chip = NULL;

		chip = container_of(work, struct ams_chip, channel_config_work);
		AMS_MUTEX_LOCK(&chip->lock);
		save = i2c_smbus_read_byte_data(chip->client,
				TCS3408_REG_ENABLE);
		configure_mux(chip);
		i2c_smbus_write_byte_data(chip->client,
				TCS3408_REG_ENABLE, save);
		AMS_MUTEX_UNLOCK(&chip->lock);
	}
}

static uint32_t get_clock_rate(struct ams_chip *chip)
{
	uint8_t buffer[3] = { 0 };
	uint32_t timeval;
	long freq;
	struct timespec old_time = { 0 };
	struct timespec new_time = { 0 };

	ams_i2c_modify(chip->client, chip->shadow, 0xA9, 0x10, 0x10);
	ams_i2c_modify(chip->client, chip->shadow, TCS3408_REG_CFG1,
			0x80, 0x80); //set s4s mode
	ams_i2c_write(chip->client, chip->shadow, TCS3408_REG_ENABLE, 0x07);
	ams_i2c_write(chip->client, chip->shadow, 0x20, 0x00);
	ams_i2c_write(chip->client, chip->shadow, 0x21, 0x00);
	getnstimeofday(&old_time);
	msleep(50);
	getnstimeofday(&new_time);
	ams_i2c_write(chip->client, chip->shadow, 0x20, 0x00);
	ams_i2c_write(chip->client, chip->shadow, 0x21, 0x00);
	ams_i2c_modify(chip->client, chip->shadow, 0x45, 0x40, 0x40);
	ams_i2c_blk_read(chip->client, 0x25, buffer, 3);
	ams_i2c_modify(chip->client, chip->shadow, 0x45, 0x40, 0x00);
	ams_i2c_write(chip->client, chip->shadow, TCS3408_REG_ENABLE, 0x00);
	ams_i2c_modify(chip->client, chip->shadow, TCS3408_REG_CFG1,
			0x80, 0x00);
	ams_i2c_modify(chip->client, chip->shadow, 0xA9, 0x10, 0x00);
	timeval = ((buffer[2] << 16) & 0x00FF0000) | ((buffer[1] << 8)
			& 0x0000FF00) | (buffer[0] & 0x000000FF);
	if (new_time.tv_nsec > old_time.tv_nsec) {
		new_time.tv_sec = new_time.tv_sec - old_time.tv_sec;
		new_time.tv_nsec = new_time.tv_nsec - old_time.tv_nsec;
	} else {
		new_time.tv_sec = new_time.tv_sec - 1 - old_time.tv_sec;
		new_time.tv_nsec = new_time.tv_nsec +
				(1000000000 - old_time.tv_nsec);
	}
	freq = new_time.tv_nsec / timeval;
	freq = freq / 8;
	freq = (1000 * 1000 * 1000) / freq;
	dev_info(&chip->client->dev, "Freq is %ld\n", freq);
	return freq;
}

/*  Attributes  */
static ssize_t channel_config_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	u8 als_ch_cfg;

	ret = kstrtol(buf, 0, (long *)(&(als_ch_cfg)));
	if (ret != 0)
		dev_err(&chip->client->dev, "kstrtol() error.\n");

	if ((als_ch_cfg != chip->channel_config) &&
			(als_ch_cfg >= 1 && als_ch_cfg <= 6)) {

		dev_info(&chip->client->dev, "set als_ch_cfg to %d\n",
				als_ch_cfg);
		chip->channel_config = als_ch_cfg;

		channel_config_work(&chip->channel_config_work);
	}

	return count;
}

static ssize_t all_regs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	uint8_t i;
	uint8_t j;
	int len = 0;
	uint8_t byte = 0;

	AMS_MUTEX_LOCK(&chip->lock);
	for (i = 0x80; i >= 0x80; i += 8) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%#X: ", i);
		for (j = 0; j < 8; ++j) {
			byte = i2c_smbus_read_byte_data(chip->client, i + j);
			len += scnprintf(buf + len, PAGE_SIZE - len,
					"%#x\t\t", byte);
		}
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	AMS_MUTEX_UNLOCK(&chip->lock);
	return len;
}

static ssize_t cal_clock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	chip->frequency = get_clock_rate(chip);

	return count;
}

static ssize_t attribute_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);
static ssize_t attribute_show(struct device *dev, struct device_attribute *attr,
		char *buf);
#define AMS_ATTR(name) \
		DEVICE_ATTR(name, 0644, attribute_show, attribute_store)

static DEVICE_ATTR_RO(all_regs);
static DEVICE_ATTR_WO(channel_config);
static DEVICE_ATTR_WO(cal_clock);
static AMS_ATTR(green_channel);
static AMS_ATTR(blue_channel);
static AMS_ATTR(red_channel);
static AMS_ATTR(clear_channel);
static AMS_ATTR(wideband_channel);
static AMS_ATTR(prox_channel);
static AMS_ATTR(flicker_channel);

static struct attribute *dbg_attrs[] = {
	&dev_attr_all_regs.attr,
	&dev_attr_cal_clock.attr,
	NULL,
};

static struct attribute *mux_attrs[] = { //Assumed same order as smux_colors
	&dev_attr_clear_channel.attr,
	&dev_attr_red_channel.attr,
	&dev_attr_green_channel.attr,
	&dev_attr_blue_channel.attr,
	&dev_attr_wideband_channel.attr,
	&dev_attr_flicker_channel.attr,
	&dev_attr_prox_channel.attr,
	&dev_attr_channel_config.attr,
	NULL,
};

static ssize_t attribute_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	uint32_t i;
	uint8_t channel;
	struct ams_chip *chip = dev_get_drvdata(dev);

	for (i = 0; i < (sizeof(mux_attrs) / sizeof(struct attribute *) - 1);
			++i) {
		if (!strncmp(attr->attr.name, mux_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			ret = kstrtol(buf, 0, (long *)(&(channel)));
			if (channel >= 0 && channel <= 7)
				chip->channels[i] = channel;
			AMS_MUTEX_UNLOCK(&chip->lock);
			return count;
		}
	}

	return count;
}

static ssize_t attribute_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	int ret = 0;
	uint32_t i;

	for (i = 0; i < (sizeof(mux_attrs) / sizeof(struct attribute *) - 1);
			++i) {
		if (!strncmp(attr->attr.name, mux_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			ret = snprintf(buf, PAGE_SIZE, "%d\n",
					chip->channels[i]);
			AMS_MUTEX_UNLOCK(&chip->lock);
			return ret;
		}
	}
	return ret;
}

static const struct attribute_group mux_attrs_group = {
	.name = "mux",
	.attrs = mux_attrs,
};

static const struct attribute_group dbg_attrs_group = {
	.name = "dbg",
	.attrs = dbg_attrs,
};

static const struct attribute_group *groups[] = {
	&mux_attrs_group,
	&dbg_attrs_group,
	NULL,
};

/*  IRQ Handling	*/
static uint8_t get_status(struct ams_chip *chip)
{
	ams_i2c_read(chip->client, TCS3408_REG_STATUS,
			&chip->shadow[TCS3408_REG_STATUS]);
	return chip->shadow[TCS3408_REG_STATUS];
}

static void irq_callback(struct ams_chip *chip,
		struct feature_callbacks *callbacks, uint8_t status)
{
	if (callbacks) {
		if (callbacks->irq)
			callbacks->irq(chip, status);
	}
}

static int irq_handler(struct ams_chip *chip)
{
	uint8_t status;

	AMS_MUTEX_LOCK(&chip->lock);
	status = get_status(chip);
	if (!status) {
		dev_info(&chip->client->dev, "No status to handle\n");
		AMS_MUTEX_UNLOCK(&chip->lock);
		return 0;
	}
	/* clear the interrupts we'll process */
	ams_i2c_write_direct(chip->client, TCS3408_REG_STATUS, status);
	irq_callback(chip, chip->als, status);
	irq_callback(chip, chip->fifo, status);
	irq_callback(chip, chip->flicker, status);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return 1;
}

static irqreturn_t tcs3408_irq(int irq, void *handle)
{
	struct ams_chip *chip = handle;
	struct device *dev = &chip->client->dev;

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
	} else if (irq_handler(chip)) {
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int poll_irq_thread(void *context)
{
	struct ams_chip *chip = (struct ams_chip *)context;
	int us_sleep = 0;

	AMS_MUTEX_LOCK(&chip->lock);
	// Poll period is interpreted in units of 100 usec
	us_sleep = chip->pdata->poll_period * 100;
	dev_info(&chip->client->dev,
			"Starting tcs3408 irq polling thread, period: %u us\n",
			us_sleep);
	AMS_MUTEX_UNLOCK(&chip->lock);
	while (!kthread_should_stop()) {
		(void) irq_handler(chip);
		usleep_range(us_sleep, us_sleep + us_sleep/10);
	}
	return 0;
}

static int init_irq(struct ams_chip *chip)
{
	unsigned long default_irq_trigger;
	int ret;

	if (chip->pdata->poll_period == 0) {
		/*** Use Interrupt I/O instead of polled ***/
		/***** Setup GPIO IRQ handler *****/
		default_irq_trigger =
			irqd_get_trigger_type(irq_get_irq_data(
					chip->client->irq));
		ret = devm_request_threaded_irq(&chip->client->dev,
				chip->client->irq,
				NULL, &tcs3408_irq,
				default_irq_trigger |
				IRQF_SHARED |
				IRQF_ONESHOT,
				dev_name(&chip->client->dev), chip);
		if (ret) {
			dev_err(&chip->client->dev, "Failed to request irq %d\n",
					chip->client->irq);
			return -ENODEV;
		}
		ams_i2c_write_direct(chip->client, TCS3408_REG_STATUS, 0x00);
	} else {
		/*** Use Polled I/O instead of interrupt ***/
		chip->poll_irq = kthread_run(poll_irq_thread,
								(void *)chip,
								"irq_poll");
		if (IS_ERR(chip->poll_irq)) {
			dev_err(&chip->client->dev, "Error starting IRQ polling thread.\n");
			return -ENODEV;
		}
	}

	return 0;
}

/*  Device Initialization   */
static int get_id(struct ams_chip *chip, struct device_ids *ids)
{
	ams_i2c_read(chip->client, TCS3408_REG_AUXID, &ids->aux);
	ams_i2c_read(chip->client, TCS3408_REG_REVID, &ids->rev);
	ams_i2c_read(chip->client, TCS3408_REG_ID, &ids->device);

	return 0;
}

#ifdef CONFIG_OF
#define PROPERTY_READ_u32(name, value, pdata) \
	if (!of_property_read_u32(pdata->of_node, #name, &value)) \
		pdata->parameters.name = value

static int init_device_tree(struct tcs3408_i2c_platform_data *pdata)
{   //Read from device tree overlay
	uint32_t val;
	uint32_t mux[7];
	int i;

	if (!pdata->of_node)
		return 0;
	if (!of_property_read_u32(pdata->of_node, "poll_x100us_period", &val))
		pdata->poll_period = val;
	if (!of_property_read_u32_array(pdata->of_node, "smux", mux, 7)) {
		for (i = 0; i < 7; ++i)
			pdata->parameters.smux[i] = (uint8_t)mux[i];
	}

	return 0;
}

//generic tcs3408 device @TODO make this specific?
static const struct of_device_id of_match[] = {
	{ .compatible = "ams,tcs3408" },
	{ }
};
MODULE_DEVICE_TABLE(of, of_match);
#endif

static int compare_ids(const struct device_ids *expected,
		struct device_ids *actual)
{
	if ((expected->device == actual->device)
			&& (expected->rev == actual->rev)) {
		return 0;
	}

	return 1;
}

static void init_mux(struct ams_chip *chip)
{
	uint8_t i;

	for (i = 0; i < 7; ++i) {
		if (chip->pdata)
			chip->channels[i] = chip->pdata->parameters.smux[i];
		else
			chip->channels[i] = i + 1;
	}
	configure_mux(chip);
	INIT_WORK(&chip->channel_config_work, channel_config_work);
}

static int init_features(struct ams_chip *chip)
{
	if (ams_tcs3408_init_als(chip))
		goto als_failed;
	if (ams_tcs3408_init_flicker(chip))
		goto flicker_failed;
	if (ams_tcs3408_init_fifo(chip))
		goto fifo_failed;
	if (init_irq(chip))
		goto irq_failed;
	return 0;

irq_failed:
	ams_tcs3408_remove_fifo(chip);
fifo_failed:
	ams_tcs3408_remove_flicker(chip);
flicker_failed:
	ams_tcs3408_remove_als(chip);
als_failed:
	return -ENODEV;
}

static void remove_features(struct ams_chip *chip)
{
	ams_tcs3408_remove_fifo(chip);
	ams_tcs3408_remove_flicker(chip);
	ams_tcs3408_remove_als(chip);
}

static int init_input_device(struct ams_chip *chip)
{
	chip->input = devm_input_allocate_device(&chip->client->dev);
	chip->input->name = "tcs3408";
	chip->input->id.bustype = BUS_I2C;
	input_set_drvdata(chip->input, chip);
	if (input_register_device(chip->input)) {
		dev_err(&chip->client->dev, "%s: Can't register input device '%s'\n",
				__func__, chip->pdata->name);
		return -ENODEV;
	}
	if (sysfs_create_groups(&chip->input->dev.kobj, groups))
		dev_err(&chip->client->dev, "Error creating syfs attribute group.\n");
	return 0;
}

static void power_device(struct ams_chip *chip, uint8_t power)
{   //Set PON bit
	if (power) {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x01, 0x01);
	} else {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x01, 0x00);
	}
}

static int validate_device(struct ams_chip *chip)
{   //Verify chip IDs and determine device type
	struct device *dev = &chip->client->dev;
	struct device_ids ids;
	int i;
	int j = ARRAY_SIZE(dev_ids);

	if (!i2c_check_functionality(chip->client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		return -EOPNOTSUPP;
	}
	get_id(chip, &ids);
	dev_info(dev, "%s: device id:%02x device revid:%02x device aux:%02x\n",
			 __func__, ids.device, ids.rev, ids.aux);
	for (i = 0; i < ARRAY_SIZE(dev_ids); i++) {
		if (!compare_ids(&dev_ids[i], &ids)) {
			j = i;
			break;
		}
	}
	if (j < ARRAY_SIZE(dev_names)) {
		dev_info(dev, "%s: '%s revid_2: 0x%x' detected\n", __func__,
			dev_names[j], ids.aux);
		chip->device_index = j;
		return 0;
	}
	dev_err(dev, "%s: chip ID not supported\n", __func__);
	return -EOPNOTSUPP;
}

static int get_platform_data(struct ams_chip *chip)
{   //Get platform data and initialize
	struct device *dev = &chip->client->dev;
	struct tcs3408_i2c_platform_data *platform_data = dev->platform_data;

#ifdef CONFIG_OF
	if (!platform_data) {
		platform_data = devm_kzalloc(dev,
				sizeof(struct tcs3408_i2c_platform_data),
				GFP_KERNEL);
		if (!platform_data) {
			dev_info(dev, "still no platform data\n");
			return -ENOMEM;
		}
		if (of_match_device(of_match, &chip->client->dev)) {
			platform_data->of_node = chip->client->dev.of_node;
			init_device_tree(platform_data);
		}
	}
#endif
	chip->pdata = platform_data;

	return 0;
}

int tcs3408_power_init(struct ams_chip *chip)
{
	int ret = 0;

	chip->vdd = regulator_get(&chip->client->dev, "vdd");
	if (IS_ERR(chip->vdd)) {
		ret = PTR_ERR(chip->vdd);
		dev_err(&chip->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(chip->vdd) > 0) {
		ret = regulator_set_voltage(chip->vdd,
				1800000,
				1800000);
		if (ret) {
			dev_err(&chip->client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}
	return 0;

reg_vdd_put:
	regulator_put(chip->vdd);
	return ret;
}

int tcs3408_power_enable(struct ams_chip *chip, enum tcs3408_pwr_state state)
{
	int ret = 0;

	if (state == POWER_ON) {
		ret = regulator_enable(chip->vdd);
		dev_err(&chip->client->dev,
				"Regulator powered on ret=%d\n",
				ret);
	} else {
		ret = regulator_disable(chip->vdd);
				dev_err(&chip->client->dev,
				"Regulator powered off ret=%d\n",
				ret);
	}
	return ret;
}

static int tcs3408_pinctrl_init(struct device *dev)
{
	struct pinctrl_state *active;
	int rc;
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get(dev);
	active = pinctrl_lookup_state(pinctrl, "tcs3408_active");
	rc = pinctrl_select_state(pinctrl, active);
	if (rc)
		dev_err(dev, "tcs3408 failed to set pin state, rc=%d\n", rc);
	return rc;
}

/*  Linux Driver API	*/
static int device_probe(struct i2c_client *client,
			const struct i2c_device_id *idp)
{
	static struct ams_chip *chip;
	struct device *dev = &client->dev;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	mutex_init(&chip->lock);
	AMS_MUTEX_LOCK(&chip->lock);
	chip->client = client;
	tcs3408_power_init(chip);
	tcs3408_power_enable(chip, POWER_ON);
	tcs3408_pinctrl_init(dev);
	if (!validate_device(chip)) {
		if (!get_platform_data(chip)) {
			i2c_set_clientdata(client, chip);
			chip->frequency = get_clock_rate(chip);
			power_device(chip, 1);
			if (!init_input_device(chip)) {
				init_mux(chip);
				if (!init_features(chip)) {
					AMS_MUTEX_UNLOCK(&chip->lock);
					return 0;
				}
			}
			i2c_set_clientdata(client, NULL);
		}
	}
	dev_err(dev, "Probe failed\n");
	AMS_MUTEX_UNLOCK(&chip->lock);
	return 1;
}

static int device_suspend(struct device *dev)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;
	/* Blocks if work is active */
	ams_tcs3408_remove_fifo(chip);
	cancel_work_sync(&chip->channel_config_work);
	if (chip->wake_irq)
		irq_set_irq_wake(chip->client->irq, 1);
	else if (chip->powered)
		dev_info(dev, "powering off\n");
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int device_resume(struct device *dev)
{   //do we need this?
	return 0; //
}

static int device_remove(struct i2c_client *client)
{
	struct ams_chip *chip = i2c_get_clientdata(client);

	if (chip->gpiod_interrupt)
		devm_free_irq(&client->dev, client->irq, chip);
	if (chip->pdata->poll_period != 0)
		(void)kthread_stop(chip->poll_irq);
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	sysfs_remove_groups(&chip->input->dev.kobj, groups);
	remove_features(chip);

	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend = device_suspend,
	.resume  = device_resume,
};

static struct i2c_driver ams_driver = {
	.driver = {
		.name = "tcs3408",
		.pm = &pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(of_match),
#endif
	},
	.id_table = tcs3408_idtable,
	.probe = device_probe,
	.remove = device_remove,
};

module_i2c_driver(ams_driver);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.3");
