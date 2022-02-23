/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c/ams/ams_tcs3408.h>
#include <linux/i2c/ams/ams_tcs3408_reg.h>
#include "ams_i2c.h"
#include "ams_tcs3408_als.h"
#include "ams_tcs3408_fifo.h"

#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>
static struct sensors_classdev als_sensors_cdev = {
	.name = "tcs3408-als",
	.vendor = "AMS",
	.enabled = 0,
	.sensors_enable = NULL,
};
#endif

struct als_chip {
	struct feature_callbacks callbacks;
	struct input_dev *input;
	struct ams_chip *chip;
	uint8_t gain;
	uint32_t step;
	uint32_t time;
	uint8_t wait;
	uint8_t channel_disable;
	uint8_t channels;
	uint8_t persist;
	uint8_t valid;
	uint8_t mode;
	struct {
		int32_t ct;
		int32_t offset;
		int32_t clear;
		int32_t red;
		int32_t green;
		int32_t blue;
		int32_t scale;
		int32_t dgf;
	} coef;
	uint32_t delta_p;
	bool enabled;
	bool auto_gain;
#ifdef CONFIG_QUALCOMM_AP
	struct sensors_classdev cdev;
#endif
};

struct als_channels {
	int32_t red;
	int32_t green;
	int32_t blue;
	int32_t clear;
	int32_t wideband;
};

static struct als_chip *get_als_chip(struct ams_chip *chip)
{
	return container_of(chip->als, struct als_chip, callbacks);
}


static void set_als_thresh(struct ams_chip *chip, uint16_t high, uint16_t low)
{
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_AILTL, (uint8_t)(low & 0x00FF));
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_AILTH, (uint8_t)((low & 0xFF00) >> 8));
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_AIHTL, (uint8_t)(high & 0x00FF));
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_AIHTH, (uint8_t)((high & 0xFF00) >> 8));
}

static void set_als_gain(struct ams_chip *chip, uint8_t gain)
{
	struct als_chip *als = get_als_chip(chip);

	als->gain = gain;
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_CFG1, als->gain);
}

#define MIN_ASTEP   278 //2.78 us * 100
static void set_als_step(struct ams_chip *chip, uint32_t step)
{
	struct als_chip *als = get_als_chip(chip);

	als->step = step;
	step = ((step * 100) + (MIN_ASTEP / 2)) /
			MIN_ASTEP - 1; //Round to nearest
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_ASTEPL, (uint8_t)((uint16_t)step & 0x00FF));
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_ASTEPH, (uint8_t)(((uint16_t)step &
					0xFF00) >> 8));
}

static uint32_t get_als_step(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);
	uint32_t step;

	ams_i2c_read(chip->client, TCS3408_REG_ASTEPL,
			&chip->shadow[TCS3408_REG_ASTEPL]);
	ams_i2c_read(chip->client, TCS3408_REG_ASTEPH,
			&chip->shadow[TCS3408_REG_ASTEPH]);
	step = (chip->shadow[TCS3408_REG_ASTEPL] & 0x00FF) |
			(chip->shadow[TCS3408_REG_ASTEPH] << 8);
	als->step = ((step + 1) * MIN_ASTEP + 50) / 100;

	return (step + 1) * MIN_ASTEP; //returns in us * 100
}

static void set_als_wait(struct ams_chip *chip, uint8_t wait)
{
	struct als_chip *als = get_als_chip(chip);

	als->wait = wait;
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_WTIME, wait);
}

static void set_als_time(struct ams_chip *chip, uint32_t time)
{
	struct als_chip *als = get_als_chip(chip);
	int step;

	als->time = time;
	step = get_als_step(chip);
	time = ((time * 100) + (step / 2)) / step - 1;
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_ATIME, time);
}

static uint32_t get_als_time(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);
	uint32_t time;

	ams_i2c_read(chip->client, TCS3408_REG_ATIME,
			&chip->shadow[TCS3408_REG_ATIME]);
	time = chip->shadow[TCS3408_REG_ATIME];
	als->time = (time + 1) * ((get_als_step(chip) + 50) / 100);
	return als->time; //round to nearest step size
}

static uint32_t get_als_gain(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);
	uint32_t gain = 0;
	uint8_t i;

	ams_i2c_read(chip->client, TCS3408_REG_CFG1,
			&chip->shadow[TCS3408_REG_CFG1]);
	als->gain = chip->shadow[TCS3408_REG_CFG1];
	if (als->gain) {
		gain = 1;
		for (i = 1; i < als->gain; ++i)
			gain = gain * 2;
	}
	return gain;
}

static void set_als_persistence(struct ams_chip *chip, uint8_t persistence)
{
	struct als_chip *als = get_als_chip(chip);

	ams_i2c_modify(chip->client, chip->shadow,
		TCS3408_REG_PERS, TCS3408_MASK_APERS, persistence);
	als->persist = persistence;
}

static void enable_als(struct ams_chip *chip, uint8_t enable)
{
	struct als_chip *als = get_als_chip(chip);

	if (enable) {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x2, 0x2);
	} else {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x02, 0x00);
	}
	als->enabled = enable;
}

static int als_autogain(struct ams_chip *chip, int data)
{
	struct als_chip *als = get_als_chip(chip);
	int astep;
	int again;

	get_als_gain(chip);
	again = als->gain;
	get_als_step(chip);
	astep = als->step;
	if (als->auto_gain) {
		if ((data > (8 * astep / 10)) && again > 0) {
			// decrement gain
			again -= 1;
			dev_info(&chip->client->dev, "%s: dec gain to %d\n",
					__func__, again);
		} else if ((data < (astep / 10)) && again < 12) {
			// increment gain
			again += 1;
			dev_info(&chip->client->dev, "%s: inc gain to %d\n",
					__func__, again);
		} else {
			return 0;
		}
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x03, 0x01);
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_CFG1, TCS3408_MASK_AGAIN, again);
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x03, 0x03);
		als->gain = again;
		return 1;
	}
	return 0;
}

void read_als_data(struct ams_chip *chip, struct als_channels *channels)
{
	ams_i2c_blk_read(chip->client, TCS3408_REG_ADATAL0,
			&chip->shadow[TCS3408_REG_ADATAL0], 12);
	ams_i2c_read(chip->client, TCS3408_REG_ASTATUS,
			&chip->shadow[TCS3408_REG_ASTATUS]);
	channels->clear = *(uint16_t *)&chip->shadow[TCS3408_REG_ADATAL0];
	channels->red = *(uint16_t *)&chip->shadow[TCS3408_REG_ADATAL1];
	channels->green = *(uint16_t *)&chip->shadow[TCS3408_REG_ADATAL2];
	channels->blue = *(uint16_t *)&chip->shadow[TCS3408_REG_ADATAL3];
	channels->wideband = *(uint16_t *)&chip->shadow[TCS3408_REG_ADATAL4];
}

static void als_fifo(struct ams_chip *chip, uint8_t *data, int size)
{
	struct als_chip *als = get_als_chip(chip);

	if (size >= 4096)
		ams_tcs3408_fifo_reset(chip);
	if (!(size % 256)) {
		input_report_abs(als->input, ABS_MISC, size);
		input_sync(als->input);
	}
}

static int compute_lux(struct ams_chip *chip, struct als_channels *channels)
{
	struct als_chip *als = get_als_chip(chip);
	uint32_t cpl;
	int lux;
	uint32_t gain;

	gain = get_als_gain(chip);
	if (!gain) { //gain = 0.5
		cpl = get_als_time(chip) / 2;
	} else {
		cpl = get_als_time(chip) * gain;
	}
	if (!cpl)
		return -ENODEV;
	lux = als->coef.dgf * ((als->coef.clear * channels->clear) +
		(als->coef.red * channels->red) +
		(als->coef.green * channels->green) +
		(als->coef.blue * channels->blue)) / cpl;
	if (lux > 0)
		return lux;
	return -ENODEV;
}

static uint32_t compute_temperature(struct ams_chip *chip,
		struct als_channels *channels)
{
	struct als_chip *als = get_als_chip(chip);
	int ir;

	ir = (channels->red + channels->green + channels->blue -
			channels->clear) / 2;
	channels->red = channels->red - ir;
	channels->blue = channels->blue - ir;
	channels->green = channels->green - ir;

	return (uint32_t)(((als->coef.ct * channels->blue) /
			channels->red) + als->coef.offset);
}

static ssize_t fifo_read(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct ams_chip *chip = dev_get_drvdata(dev);
	int read;

	AMS_MUTEX_LOCK(&chip->lock);
	read = ams_tcs3408_read_fifo(chip, buf, size);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return read;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	uint8_t enable;

	ret = kstrtol(buf, 0, (long *)(&(enable)));
	if (ret != 0)
		dev_err(&chip->client->dev, "kstrtol() error.\n");
	enable_als(chip, enable);
	return count;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct als_chip *als = get_als_chip(chip);
	int ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", als->enabled);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
		char *buf);
#define ALS_DATA_ATTR(name) \
	DEVICE_ATTR(name, 0644, data_show, NULL)

static ALS_DATA_ATTR(ch0);
static ALS_DATA_ATTR(ch1);
static ALS_DATA_ATTR(ch2);
static ALS_DATA_ATTR(ch3);
static ALS_DATA_ATTR(ch4);
static ALS_DATA_ATTR(ch5);
static ALS_DATA_ATTR(lux);
static ALS_DATA_ATTR(temperature);
static ALS_DATA_ATTR(valid);

static ssize_t coef_show(struct device *dev, struct device_attribute *attr,
		char *buf);

#define ALS_COEF_ATTR(name) \
	DEVICE_ATTR(name, 0644, coef_show, NULL)


static BIN_ATTR_RO(fifo, PAGE_SIZE);
static ALS_COEF_ATTR(red);
static ALS_COEF_ATTR(green);
static ALS_COEF_ATTR(blue);
static ALS_COEF_ATTR(clear);
static ALS_COEF_ATTR(ct);
static ALS_COEF_ATTR(offset);
static ALS_COEF_ATTR(scale);
static ALS_COEF_ATTR(dgf);

static ssize_t config_show(struct device *dev, struct device_attribute *attr,
		char *buf);
static ssize_t config_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);
#define ALS_CFG_ATTR(name) \
	DEVICE_ATTR(name, 0644, config_show, config_store)

static ALS_CFG_ATTR(step);
static ALS_CFG_ATTR(gain);
static ALS_CFG_ATTR(time);
static ALS_CFG_ATTR(auto_gain);
static ALS_CFG_ATTR(wtime);
static ALS_CFG_ATTR(persist);
static ALS_CFG_ATTR(channels);
static ALS_CFG_ATTR(mode);

static DEVICE_ATTR_RW(enable);

static struct attribute *config_attrs[] = {
	&dev_attr_step.attr,
	&dev_attr_gain.attr,
	&dev_attr_time.attr,
	&dev_attr_auto_gain.attr,
	&dev_attr_wtime.attr,
	&dev_attr_persist.attr,
	&dev_attr_channels.attr,
	&dev_attr_mode.attr,
	&dev_attr_enable.attr,
	NULL,
};

enum als_config {
	STEP_CFG = 0,
	GAIN_CFG,
	TIME_CFG,
	AUTO_GAIN_CFG,
	WTIME_CFG,
	PERSIST_CFG,
	CHANNELS_CFG,
	MODE_CFG
};

static ssize_t config_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct als_chip *als = get_als_chip(chip);
	int ret = 0;
	uint32_t i;

	for (i = 0; i < (sizeof(config_attrs) / sizeof(struct attribute *) - 1);
			++i) {
		if (!strncmp(attr->attr.name, config_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			switch (i) {
			case STEP_CFG:
				get_als_step(chip);
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->step);
				break;
			case GAIN_CFG:
				get_als_gain(chip);
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->gain);
				break;
			case TIME_CFG:
				get_als_time(chip);
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->time);
				break;
			case AUTO_GAIN_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->auto_gain);
				break;
			case WTIME_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->wait);
				break;
			case PERSIST_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->persist);
				break;
			case CHANNELS_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->channels);
				break;
			case MODE_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						als->mode);
				break;
			default:
				ret = snprintf(buf, PAGE_SIZE, "not found\n");
				break;
			}
			AMS_MUTEX_UNLOCK(&chip->lock);
			return ret;
		}
	}
	return ret;
}

static ssize_t config_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct als_chip *als = get_als_chip(chip);
	uint32_t i = 0;
	uint32_t data;

	if (!kstrtol(buf, 0, (long *)(&(data)))) {
		for (i = 0; i < (sizeof(config_attrs) /
				sizeof(struct attribute *) - 1);
				++i) {
			if (!strncmp(attr->attr.name, config_attrs[i]->name,
				strlen(attr->attr.name))) {
				AMS_MUTEX_LOCK(&chip->lock);
				switch (i) {
				case STEP_CFG:
					set_als_step(chip, data);
					break;
				case GAIN_CFG:
					if (data < 12)
						set_als_gain(chip,
							(uint8_t)data);
					break;
				case TIME_CFG:
					set_als_time(chip, data);
					break;
				case AUTO_GAIN_CFG:
					als->auto_gain = !!data;
					break;
				case WTIME_CFG:
					set_als_wait(chip, data);
					break;
				case PERSIST_CFG:
					set_als_persistence(chip,
							(uint8_t)data);
					break;
				case CHANNELS_CFG:
					als->channels = data;
					break;
				case MODE_CFG:
					als->mode = data;
					break;
				default:
					break;
				}
				AMS_MUTEX_UNLOCK(&chip->lock);
				return count;
			}
		}
	}
	return count;
}

static struct attribute *data_attrs[] = {
	&dev_attr_ch0.attr,
	&dev_attr_ch1.attr,
	&dev_attr_ch2.attr,
	&dev_attr_ch3.attr,
	&dev_attr_ch4.attr,
	&dev_attr_ch5.attr,
	&dev_attr_valid.attr,
	&dev_attr_lux.attr,
	&dev_attr_temperature.attr,
	NULL,
};

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct als_chip *als = get_als_chip(chip);
	int ret = 0;
	uint32_t i;

	for (i = 0; i < (sizeof(data_attrs) / sizeof(struct attribute *) - 1);
			++i) {
		if (!strncmp(attr->attr.name, data_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			if (i < 6) {
				uint16_t ch;

				ch = i2c_smbus_read_word_data(chip->client,
						TCS3408_REG_ADATAL0 + (i * 2));
				ret = snprintf(buf, PAGE_SIZE, "%d\n", ch);
				AMS_MUTEX_UNLOCK(&chip->lock);
				return ret;
			}
			if (i == 6) { //show valid
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->valid);
			}
			if (i == 7) { //show lux
				struct als_channels channels;

				read_als_data(chip, &channels);
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						compute_lux(chip, &channels));
			}
			if (i == 8) {
				struct als_channels channels;

				read_als_data(chip, &channels);
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						compute_temperature(chip,
						&channels));
			}
			AMS_MUTEX_UNLOCK(&chip->lock);
			return ret;

		}
	}
	return ret;
}

static struct bin_attribute *data_bin_attrs[] = {
	&bin_attr_fifo,
	NULL,
};

static struct attribute *coef_attrs[] = {
	&dev_attr_red.attr,
	&dev_attr_blue.attr,
	&dev_attr_green.attr,
	&dev_attr_clear.attr,
	&dev_attr_ct.attr,
	&dev_attr_offset.attr,
	&dev_attr_scale.attr,
	&dev_attr_dgf.attr,
	NULL,
};

enum ams_coefs {
	COEF_RED = 0,
	COEF_BLUE,
	COEF_GREEN,
	COEF_CLEAR,
	COEF_CT,
	COEF_OFFSET,
	COEF_SCALE,
	COEF_DGF,
};

static ssize_t coef_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct als_chip *als = get_als_chip(chip);
	int ret = 0;
	uint32_t i;

	for (i = 0; i < (sizeof(coef_attrs) / sizeof(struct attribute *) - 1);
			++i) {
		if (!strncmp(attr->attr.name, coef_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			switch (i) {
			case COEF_RED:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.red);
				break;
			case COEF_GREEN:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.green);
				break;
			case COEF_BLUE:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.blue);
				break;
			case COEF_CLEAR:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.clear);
				break;
			case COEF_CT:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.ct);
				break;
			case COEF_DGF:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.dgf);
					break;
			case COEF_OFFSET:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.offset);
				break;
			case COEF_SCALE:
				ret = snprintf(buf, PAGE_SIZE,
						"%d\n", als->coef.scale);
				break;
			default:
				ret = snprintf(buf, PAGE_SIZE,
						"not found\n");
				break;
			}
			AMS_MUTEX_UNLOCK(&chip->lock);
			return ret;

		}
	}
	return ret;
}

static const struct attribute_group config_group = {
	.name = "config",
	.attrs = config_attrs,
};

static const struct attribute_group data_group = {
	.name = "data",
	.attrs = data_attrs,
	.bin_attrs = data_bin_attrs,
};

static const struct attribute_group coef_group = {
	.name = "coefficients",
	.attrs = coef_attrs,
};

static const struct attribute_group *als_groups[] = {
	&config_group,
	&data_group,
	&coef_group,
	NULL,
};

static void report_als(struct ams_chip *chip, int value)
{
	struct als_chip *als = get_als_chip(chip);

	if (als->input) {
		input_report_abs(als->input, ABS_MISC, value);
		input_sync(als->input);
	}
}

static int get_new_thresh(struct ams_chip *chip, uint16_t *high,
		uint16_t *low, uint16_t value)
{
	struct als_chip *als = get_als_chip(chip);
	uint32_t margin;
	uint16_t time;

	get_als_time(chip);
	time = chip->shadow[TCS3408_REG_ATIME];
	margin = value * als->delta_p / 100;
	if (value + margin >= (time * 8 / 10))
		*high = value + margin;
	else
		*high = (time * 8 / 10);
	if (value > margin)
		*low = value - margin;
	else
		*low = 0;
	dev_info(&chip->client->dev, "High: %d, Low: %d\n", *high, *low);
	return 1;
}

static void als_irq(struct ams_chip *chip, uint8_t status)
{
	uint8_t als_status = 0;
	struct als_chip *als = get_als_chip(chip);
	struct als_channels channels;
	int lux;
	int temperature;
	uint16_t high, low;

	if (status & TCS3408_MASK_AINT) {
		ams_i2c_read(chip->client, TCS3408_REG_STATUS2,
					&chip->shadow[TCS3408_REG_STATUS2]);
		als_status = chip->shadow[TCS3408_REG_STATUS2];
		if (status & TCS3408_MASK_ASAT) {
			if (als_status & 0x18) { //Als saturation
				report_als(chip, (als_status & 0x18) * -1);
				als->valid = 0;
				return;
			}
		}
		if (als_status & 0x40) {	//ALS Valid
			read_als_data(chip, &channels);
			if (!als_autogain(chip, channels.clear)) {
				if (get_new_thresh(chip, &high, &low,
						channels.clear))
					set_als_thresh(chip, high, low);
				lux = compute_lux(chip, &channels);
				temperature = compute_temperature(
						chip, &channels);
				report_als(chip, lux);
				if (lux > 0) {
					als->valid = 1;
					return;
				}
			} else {
				set_als_thresh(chip, 0x0000, 0xFFFF);
			}
		} else {
			dev_info(&chip->client->dev, "ALS not valid %x\n",
					als_status);
			read_als_data(chip, &channels);
			report_als(chip, -1); //Invalid
		}
		als->valid = 0;
	}
}

static void config_als(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);

	set_als_thresh(chip, 0x0000, 0xFFFF);
	set_als_persistence(chip, als->persist);
	set_als_step(chip, als->step);
	set_als_wait(chip, als->wait);
	set_als_gain(chip, als->gain);
	set_als_time(chip, als->time);
}

static int als_open(struct input_dev *input)
{
	struct ams_chip *chip = dev_get_drvdata(&input->dev);
	struct als_chip *als = get_als_chip(chip);

	ams_i2c_read(chip->client, TCS3408_REG_ENABLE,
			&chip->shadow[TCS3408_REG_ENABLE]);
	if (!(chip->shadow[TCS3408_REG_ENABLE] & 0x40)) { //als enabled
		config_als(chip);
		if (als->mode) {
			ams_tcs3408_fifo_reset(chip);
			ams_tcs3408_enable_fifo(chip, als->channels);
		} else {
			ams_i2c_modify(chip->client, chip->shadow,
					TCS3408_REG_INTENAB, TCS3408_MASK_AIEN,
							TCS3408_MASK_AIEN);
		}
		dev_info(&chip->client->dev, "opening ALS\n");
		enable_als(chip, 1);
	}

	return 0;
}

static void als_close(struct input_dev *input)
{
	struct ams_chip *chip = dev_get_drvdata(&input->dev);
	struct als_chip *als = get_als_chip(chip);

	if (!als->mode) {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_INTENAB, TCS3408_MASK_AIEN, 0);
	} else {
		ams_tcs3408_enable_fifo(chip, FIFO_OFF);
	}
	dev_info(&chip->client->dev, "closing ALS\n");
	enable_als(chip, 0);
}

#ifdef CONFIG_QUALCOMM_AP
static int als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct als_chip *als = container_of(sensors_cdev,
			struct als_chip, cdev);

	if (enable)
		als->input->open(als->input);
	else
		als->input->close(als->input);
	return 0;
}
#endif

static int init_als_input(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);

	als->input = devm_input_allocate_device(&chip->client->dev);
	if (!als->input)
		return -ENODEV;
	als->input->name = "als";
	als->input->id.bustype = BUS_I2C;
	set_bit(EV_ABS, als->input->evbit);
	set_bit(ABS_MISC, als->input->absbit);
	input_set_abs_params(als->input, ABS_MISC, 0, 65535, 0, 0);
	als->input->open = als_open;
	als->input->close = als_close;
	input_set_drvdata(als->input, chip);
	if (input_register_device(als->input)) {
		dev_err(&chip->client->dev, "%s: Can't register input device '%s'\n",
				__func__, als->input->name);
		return -ENODEV;
	}
	if (sysfs_create_groups(&als->input->dev.kobj, als_groups))
		dev_err(&chip->client->dev, "Error creating sysfs attribute group.\n");
#ifdef CONFIG_QUALCOMM_AP
	als->cdev = als_sensors_cdev;
	als->cdev.sensors_enable = als_set_enable;
	if (sensors_classdev_register(&als->input->dev, &als->cdev)) {
		dev_err(&chip->client->dev, "sensors class register failed.\n");
		return -ENODEV;
	}
#endif
	return 0;
}

#define READ_ALS_PROPERTY(name) { \
	if (!of_property_read_s32(node, #name, &value)) { \
		als->name = value; \
	} else { \
		als->name = 0; \
	} \
}
#define READ_ALS_COEF(name) { \
	if (!of_property_read_s32(node, #name, &value)) { \
		als->coef.name = value; \
	} else { \
		als->coef.name = 0; \
	} \
}

static int init_device_tree(struct ams_chip *chip)
{
	struct device_node *node  = of_get_child_by_name(
			chip->pdata->of_node, "als");
	struct als_chip *als = get_als_chip(chip);
	int value = 0;

	if (node) { // Configuration
		READ_ALS_PROPERTY(gain)
		READ_ALS_PROPERTY(step)
		READ_ALS_PROPERTY(time)
		READ_ALS_PROPERTY(auto_gain)
		READ_ALS_PROPERTY(wait)
		READ_ALS_PROPERTY(persist)
		READ_ALS_PROPERTY(delta_p)
		READ_ALS_PROPERTY(channels)
		READ_ALS_PROPERTY(mode)
		node = of_get_child_by_name(node, "coefficient");
		if (node) { // Coefficients
			READ_ALS_COEF(red)
			READ_ALS_COEF(green)
			READ_ALS_COEF(blue)
			READ_ALS_COEF(clear)
			READ_ALS_COEF(ct)
			READ_ALS_COEF(dgf)
			READ_ALS_COEF(offset)
			READ_ALS_COEF(scale)
		} else { //default coefficients
			//Need default coefficients
		}
		return 1;
	}
	return 0;
}

void ams_tcs3408_remove_als(struct ams_chip *chip)
{
	struct als_chip *als = get_als_chip(chip);

	if (als->input) {
		sysfs_remove_link(&chip->input->dev.kobj, "als");
		sysfs_remove_groups(&als->input->dev.kobj, als_groups);
		input_unregister_device(als->input);
	}
}

int ams_tcs3408_init_als(struct ams_chip *chip)
{
	static struct als_chip als = { {0} };

	if (chip->input) {
		als.chip = chip;
		chip->als = &als.callbacks;
		if (init_device_tree(chip)) {
			als.callbacks.irq = als_irq;
			als.callbacks.fifo = als_fifo;
			if (init_als_input(chip))
				return -ENODEV;
			if (sysfs_create_link(&chip->input->dev.kobj,
					&als.input->dev.kobj, "als"))
				return -ENODEV;
		}
		return 0;
	}
	return -ENODEV;
}
