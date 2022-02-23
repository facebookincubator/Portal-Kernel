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
#include <linux/time.h>
#include <linux/i2c/ams/ams_tcs3408.h>
#include <linux/i2c/ams/ams_tcs3408_reg.h>
#include "ams_i2c.h"
#include "ams_tcs3408_flicker.h"
#include "ams_tcs3408_fifo.h"
#include "ams_fft.h"

#ifdef CONFIG_QUALCOMM_AP
#include <linux/sensors.h>
static struct sensors_classdev flicker_cdev = {
	.name = "tcs3408-flicker",
	.vendor = "AMS",
	.enabled = 0,
	.sensors_enable = NULL,
};
#endif

#define FLICKER_VALUE ABS_MISC
#define FIFO_DEPTH  ABS_VOLUME

enum flicker_mode {
	HARDWARE_MODE = 0,
	SOFTWARE_RAW = 1,
	SOFTWARE_FFT = 2,
	SOFTWARE_HZ = 3
};

struct flicker_chip {
	struct feature_callbacks callbacks;
	struct work_struct work;
	struct input_dev *input;
	uint8_t gain;
	uint16_t rate;
	uint16_t time;
	uint32_t frequency;
	bool enabled;
	enum flicker_mode mode;
	struct ams_chip *chip;
#ifdef CONFIG_QUALCOMM_AP
	struct sensors_classdev cdev;
#endif
};

static struct flicker_chip *get_flicker_chip(struct ams_chip *chip)
{
	return container_of(chip->flicker, struct flicker_chip, callbacks);
}

static void report_flicker(struct ams_chip *chip, int type, int value)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	if (flicker->input) {
		input_report_abs(flicker->input, type, value);
		input_sync(flicker->input);
	}
}

static void set_sample_rate(struct ams_chip *chip, uint16_t rate)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	if (flicker->enabled)
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x40, 0x00);
	flicker->time = (chip->frequency / (2 * rate)) - 1;
	flicker->rate = rate;
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_FD_TIME, flicker->time & 0xFF);
	ams_i2c_modify(chip->client, chip->shadow,
			TCS3408_REG_FD_CONFIG, 0x03, (flicker->time >> 8)
					& 0x03);
	if (flicker->enabled)
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, 0x40, 0x40);
}

static void set_gain(struct ams_chip *chip, uint8_t gain)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	if (gain <= 11) {
		if (flicker->enabled) {
			ams_i2c_modify(chip->client, chip->shadow,
					TCS3408_REG_ENABLE, 0x40, 0x00);
		}
		flicker->gain = gain;
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_FD_CONFIG, 0xFC,
				(flicker->gain << 3));
		if (flicker->enabled) {
			ams_i2c_modify(chip->client, chip->shadow,
					TCS3408_REG_ENABLE, 0x40, 0x40);
		}
	}
}

static void enable_flicker(struct ams_chip *chip, uint8_t enable)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	if (enable) {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, TCS3408_MASK_FDEN,
				TCS3408_MASK_FDEN);
		flicker->enabled = 1;
	} else {
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_ENABLE, TCS3408_MASK_FDEN, 0x00);
		flicker->enabled = 0;
	}
}

static int flicker_autogain(struct ams_chip *chip, uint16_t *data, int size)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);
	uint8_t gain = flicker->gain;
	int i;
	int max = 0;

	for (i = 0; i < size; ++i) {
		if (data[i] > max)
			max = data[i];
	}
	if ((max < (flicker->time / 3)) && gain < 12) {
		gain++;
		dev_info(&chip->client->dev, "%s: inc flicker gain to %d\n",
				__func__, gain);
	} else if ((max >= flicker->time) && gain > 0) {
		gain--;
		dev_info(&chip->client->dev, "%s: dec flicker gain to %d\n",
				__func__, gain);
	} else {
		return 0;
	}
	set_gain(chip, gain);
	return 1;
}

static struct timespec old_time = { 0 };

int get_fft(struct ams_chip *chip, char *out)
{
	ssize_t size;
	static int16_t buffer[2048] = { 0 };

	memset(buffer, 0, 4096);
	size = ams_tcs3408_read_fifo(chip, (char *)buffer, 4096);
	if (size < 4096)
		return 0;
	ams_i2c_modify(chip->client, chip->shadow,
		TCS3408_REG_ENABLE, TCS3408_MASK_FDEN, 0);
	ams_tcs3408_fifo_reset(chip);
	ams_i2c_modify(chip->client, chip->shadow,
		TCS3408_REG_ENABLE, TCS3408_MASK_FDEN, TCS3408_MASK_FDEN);
	getnstimeofday(&old_time);
	report_flicker(chip, FIFO_DEPTH, 0);
	if (!flicker_autogain(chip, buffer, 2048)) {
		if (ams_rfft(buffer, 2048)) {
			ams_get_magnitude(buffer, (uint16_t *)out, 1024);
			return 1024;
		}
	}
	return 0;
}

static uint32_t get_sqrt(uint32_t x)
{
	uint32_t result;
	uint32_t tmp;

	result = 0;
	tmp = (1 << 30);
	while (tmp > x)
		tmp >>= 2;
	while (tmp != 0) {
		if (x >= (result + tmp)) {
			x -= result + tmp;
			result += 2 * tmp;
		}
		result >>= 1;
		tmp >>= 2;
	}
	return result;
}

static int get_stdev(uint16_t *buff, int mean, int size)
{
	int i;
	uint32_t sum = 0;

	for (i = 0; i < size; ++i)
		sum += ((buff[i] - mean) * (buff[i] - mean));
	sum = sum / (size - 1);
	return get_sqrt(sum);
}

static int get_mean(uint16_t *buff, int size)
{
	int i;
	int sum = 0;

	for (i = 0; i < size; ++i)
		sum += buff[i];
	return sum / size;
}

static int get_frequency(struct ams_chip *chip)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);
	static uint16_t buf[1024] = { 0 };
	int max = 0;
	int mean;
	int stdev;
	int i;

	if (get_fft(chip, (char *)buf)) {
		buf[0] = 0;
		for (i = 0; i < 1024; ++i) {
			if (buf[i] > buf[max])
				max = i;
		}
		mean = get_mean(buf, 1024);
		stdev = get_stdev(buf, mean, 1024);
		if (buf[max] > (mean + (stdev * 6)))
			return (max * flicker->frequency / 2048);
	}
	return 0;
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
	enable_flicker(chip, enable);

	return count;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct flicker_chip *flicker = get_flicker_chip(chip);
	int ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", flicker->enabled);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t fft_read(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct ams_chip *chip = dev_get_drvdata(dev);
	int read;

	if (size < (sizeof(uint16_t) * 1024)) {
		dev_info(dev, "%s: Size %zd is less than size of fft\n",
				__func__, size);
		return -EAGAIN;
	}
	AMS_MUTEX_LOCK(&chip->lock);
	read = get_fft(chip, buf) * sizeof(uint16_t);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return read;
}

static ssize_t raw_read(struct file *fp, struct kobject *kobj,
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

static ssize_t frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct ams_chip *chip = dev_get_drvdata(dev);
	int frequency;

	AMS_MUTEX_LOCK(&chip->lock);
	frequency = get_frequency(chip);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", frequency);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return ret;
}
static ssize_t config_show(struct device *dev, struct device_attribute *attr,
		char *buf);
static ssize_t config_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);
#define FLICKER_CFG_ATTR(name) \
	DEVICE_ATTR(name, 0644, config_show, config_store)

static FLICKER_CFG_ATTR(gain);
static FLICKER_CFG_ATTR(sample_rate);
static FLICKER_CFG_ATTR(mode);
static DEVICE_ATTR_RW(enable);
static BIN_ATTR_RO(fft, PAGE_SIZE);
static BIN_ATTR_RO(raw, PAGE_SIZE);
static DEVICE_ATTR_RO(frequency);

static struct attribute *config_attrs[] = {
	&dev_attr_gain.attr,
	&dev_attr_sample_rate.attr,
	&dev_attr_mode.attr,
	&dev_attr_enable.attr,
	NULL,
};

enum flicker_config {
	GAIN_CFG = 0,
	SAMPLE_RATE_CFG,
	MODE_CFG,
};

static ssize_t config_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct flicker_chip *flicker = get_flicker_chip(chip);
	int ret = 0;
	uint32_t i;

	for (i = 0; i < (sizeof(config_attrs) /
			sizeof(struct attribute *) - 1); ++i) {
		if (!strncmp(attr->attr.name, config_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			switch (i) {
			case GAIN_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						flicker->gain);
				break;
			case SAMPLE_RATE_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						flicker->rate);
				break;
			case MODE_CFG:
				ret = snprintf(buf, PAGE_SIZE, "%d\n",
						flicker->mode);
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
	struct flicker_chip *flicker = get_flicker_chip(chip);
	uint32_t i = 0;
	int data;
	int rc;

	rc = kstrtoint(buf, 0, (int *)&data);
	if (rc != 0)
		return -ENODEV;
	for (i = 0; i < (sizeof(config_attrs) /
			sizeof(struct attribute *) - 1); ++i) {
		if (!strncmp(attr->attr.name, config_attrs[i]->name,
				strlen(attr->attr.name))) {
			AMS_MUTEX_LOCK(&chip->lock);
			switch (i) {
			case GAIN_CFG:
				set_gain(chip, (uint8_t)data);
				break;
			case SAMPLE_RATE_CFG:
				set_sample_rate(chip, data);
				break;
			case MODE_CFG:
				if (data >= 0 && data <= 3)
					flicker->mode = data;
				break;
			default:
				break;
			}
			AMS_MUTEX_UNLOCK(&chip->lock);
			return count;
		}
	}
	return count;
}

static const struct attribute_group config_group = {
	.name = "config",
	.attrs = config_attrs,
};

static struct attribute *data_attrs[] = {
	&dev_attr_frequency.attr,
	NULL,
};

static struct bin_attribute *data_bin_attrs[] = {
	&bin_attr_fft,
	&bin_attr_raw,
	NULL,
};

static struct attribute_group data_group = {
	.name = "data",
	.attrs = data_attrs,
	.bin_attrs = data_bin_attrs,
};

static const struct attribute_group *flicker_groups[] = {
	&config_group,
	&data_group,
	NULL,
};

/*static int check_flicker_detect(struct ams_chip *chip)*/
/*{*/
	/*ams_i2c_read(chip->client, TCS3408_REG_STATUS5,*/
			/*&chip->shadow[TCS3408_REG_STATUS5]);*/
	/*return (chip->shadow[TCS3408_REG_STATUS5]);*/
/*}*/

static uint8_t get_flicker_status(struct ams_chip *chip)
{
	ams_i2c_read(chip->client, TCS3408_REG_FD_STATUS,
			&chip->shadow[TCS3408_REG_FD_STATUS]);
	return (chip->shadow[TCS3408_REG_FD_STATUS] & 0x3F);
}


static void flicker_work_cb(struct work_struct *work)
{
	struct ams_chip *chip = NULL;
	struct flicker_chip *flicker = NULL;
	int frequency;

	if (work) {
		flicker = container_of(work, struct flicker_chip, work);
		if (flicker)
			chip = flicker->chip;
	}
	if (flicker->input) {
		switch (flicker->mode) {
		case SOFTWARE_RAW:
		case SOFTWARE_FFT:
			report_flicker(chip, FIFO_DEPTH, 4096);
			break;
		case SOFTWARE_HZ:
			frequency = get_frequency(chip);
			if (frequency)
				report_flicker(chip, FLICKER_VALUE, frequency);
			break;
		case HARDWARE_MODE:
		default:
			//Something wrong happened
			break;
		}
	}
}

static void flicker_fifo(struct ams_chip *chip, uint8_t *data, int size)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);
	struct timespec time = { 0 };
	struct timespec tmp = { 0 };
	long sample_rate = 0;

	if (size >= 4096) {
		getnstimeofday(&time);
		if (time.tv_nsec > old_time.tv_nsec) {
			tmp.tv_sec = time.tv_sec - old_time.tv_sec;
			tmp.tv_nsec = time.tv_nsec - old_time.tv_nsec;
		} else {
			tmp.tv_sec = time.tv_sec - 1 - old_time.tv_sec;
			tmp.tv_nsec = time.tv_nsec +
			(1000000000 - old_time.tv_nsec);
		}
		sample_rate = tmp.tv_sec * 1000 * 1000 * 1000 + tmp.tv_nsec;
		sample_rate = sample_rate / (size / 2);
		dev_info(&chip->client->dev,
				"sample period: %ld nsec\n", sample_rate);
		sample_rate = (1000 * 1000 * 1000) / sample_rate;
		flicker->frequency = sample_rate;
		dev_info(&chip->client->dev,
				"Sample Rate: %ld Hz\n", sample_rate);
		schedule_work(&flicker->work);
	} else if (size > 256 && flicker->mode == SOFTWARE_RAW) {
		report_flicker(chip, FIFO_DEPTH, size);
	}
}

void flicker_irq(struct ams_chip *chip, uint8_t status)
{
	uint8_t flicker_status = 0;
	struct flicker_chip *flicker = get_flicker_chip(chip);

	if (!flicker->mode) {
		if (status & TCS3408_MASK_ASAT) {
			ams_i2c_read(chip->client, TCS3408_REG_STATUS2,
					&chip->shadow[TCS3408_REG_STATUS2]);
			status = chip->shadow[TCS3408_REG_STATUS2];
			if (status & 0x03) { //FD saturation
				report_flicker(chip, FLICKER_VALUE, -1);
				return;
			}
		}
		flicker_status = get_flicker_status(chip);
		dev_info(&chip->client->dev, "status: %x\n", flicker_status);
		if (flicker_status & 0x20) { //FD measurement valid
			ams_i2c_write(chip->client, chip->shadow,
					0xDB, 0x20); //Clear bit
			if (flicker_status & 0x08) { //120Hz valid
				ams_i2c_write(chip->client, chip->shadow,
					0xDB, 0x08); //Clear bit
				if (flicker_status & 0x02) { //120Hz detected
					report_flicker(chip,
					FLICKER_VALUE, 120);
					return;
				}
			}
			if (flicker_status & 0x04) { //100Hz valid
				ams_i2c_write(chip->client, chip->shadow,
						0xDB, 0x04); //clear bit
				if (flicker_status & 0x01) { //100Hz detected
					report_flicker(chip,
					FLICKER_VALUE, 100);
					return;
				}
			}
		}
		report_flicker(chip, FLICKER_VALUE, 0);
	}
}

static void configure_flicker(struct ams_chip *chip)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	set_sample_rate(chip, flicker->rate);
	set_gain(chip, flicker->gain);
	ams_i2c_write(chip->client, chip->shadow,
		0xCC, 0x02); //make sure we're not in 8-bit mode
	ams_i2c_modify(chip->client, chip->shadow,
			TCS3408_REG_CFG8, 0x08, 0x08); //must disable AGC
}

static void enable_flicker_interrupts(struct ams_chip *chip, uint8_t enable)
{
	ams_i2c_modify(chip->client, chip->shadow, TCS3408_REG_INTENAB,
		TCS3408_MASK_SIEN, (!!enable) << TCS3408_SIEN_SHIFT);
	ams_i2c_modify(chip->client, chip->shadow, TCS3408_REG_CFG9,
		TCS3408_MASK_SIEN_FD_SHIFT,
		(!!enable) << TCS3408_SIEN_FD_SHIFT);
}

static int flicker_open(struct input_dev *input)
{
	struct ams_chip *chip = dev_get_drvdata(&input->dev);
	struct flicker_chip *flicker = get_flicker_chip(chip);

	ams_i2c_read(chip->client, TCS3408_REG_ENABLE,
			&chip->shadow[TCS3408_REG_ENABLE]);
	if (!(chip->shadow[TCS3408_REG_ENABLE] & 0x02)) { //als enabled
		configure_flicker(chip);
		if (flicker->mode) { //Use FIFO
			ams_tcs3408_fifo_reset(chip);
			ams_tcs3408_enable_fifo(chip, FIFO_FLICKER);
			report_flicker(chip, FIFO_DEPTH, 0);
			report_flicker(chip, FLICKER_VALUE, 0);
		} else { //Use HW flicker
			ams_tcs3408_enable_fifo(chip, 0);
			enable_flicker_interrupts(chip, 1);
		}
		dev_info(&chip->client->dev, "opening flicker\n");
		enable_flicker(chip, 1);
		getnstimeofday(&old_time);
	}

	return 0;
}

static void flicker_close(struct input_dev *input)
{
	struct ams_chip *chip = dev_get_drvdata(&input->dev);
	struct flicker_chip *flicker = get_flicker_chip(chip);

	cancel_work_sync(&flicker->work);
	if (!flicker->mode)
		enable_flicker_interrupts(chip, 0);
	dev_info(&chip->client->dev, "closing flicker\n");
	enable_flicker(chip, 0);
}

#ifdef CONFIG_QUALCOMM_AP
static int flicker_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct flicker_chip *flicker = container_of(sensors_cdev,
			struct flicker_chip, cdev);

	if (enable)
		flicker->input->open(flicker->input);
	else
		flicker->input->close(flicker->input);
	return 0;
}
#endif

static int init_flicker_input(struct ams_chip *chip)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	flicker->input = devm_input_allocate_device(&chip->client->dev);
	if (!flicker->input)
		return -ENODEV;
	flicker->input->name = "flicker";
	flicker->input->id.bustype = BUS_I2C;
	set_bit(EV_ABS, flicker->input->evbit);
	set_bit(FLICKER_VALUE, flicker->input->absbit);
	set_bit(FIFO_DEPTH, flicker->input->absbit);
	input_set_abs_params(flicker->input, FLICKER_VALUE, 0, 65535, 0, 0);
	input_set_abs_params(flicker->input, FIFO_DEPTH, 0, 65535, 0, 0);
	flicker->input->open = flicker_open;
	flicker->input->close = flicker_close;
	input_set_drvdata(flicker->input, chip);
	if (input_register_device(flicker->input)) {
		dev_err(&chip->client->dev,
				"%s: Can't register input device '%s'\n",
				__func__, flicker->input->name);
		return -ENODEV;
	}
	if (sysfs_create_groups(&flicker->input->dev.kobj, flicker_groups))
		dev_err(&chip->client->dev, "Error creating sysfs attribute group.\n");
#ifdef CONFIG_QUALCOMM_AP
	flicker->cdev = flicker_cdev;
	flicker->cdev.sensors_enable = flicker_set_enable;
	if (sensors_classdev_register(&flicker->input->dev, &flicker->cdev)) {
		dev_err(&chip->client->dev, "sensors class register failed\n");
		return -ENODEV;
	}
#endif
	return 0;
}

#define READ_FLICKER_PROPERTY(name) { \
	if (!of_property_read_u32(node, #name, &value)) { \
		flicker->name = value; \
	} else { \
		flicker->name = 0; \
	} \
}

static int init_device_tree(struct ams_chip *chip)
{
	struct device_node *node = of_get_child_by_name
			(chip->pdata->of_node, "flicker");
	struct flicker_chip *flicker = get_flicker_chip(chip);
	int value = 0;

	if (node) {
		READ_FLICKER_PROPERTY(gain)
		READ_FLICKER_PROPERTY(rate)
		READ_FLICKER_PROPERTY(mode)
		return 1;
	}
	return 0;
}

void ams_tcs3408_remove_flicker(struct ams_chip *chip)
{
	struct flicker_chip *flicker = get_flicker_chip(chip);

	cancel_work_sync(&flicker->work);
	if (flicker->input) {
		sysfs_remove_link(&chip->input->dev.kobj, "flicker");
		sysfs_remove_groups(&flicker->input->dev.kobj, flicker_groups);
		input_unregister_device(flicker->input);
	}
}

int ams_tcs3408_init_flicker(struct ams_chip *chip)
{
	static struct flicker_chip flicker = { {0} };

	if (chip->input) {
		flicker.chip = chip;
		chip->flicker = &flicker.callbacks;
		if (init_device_tree(chip)) {
			flicker.callbacks.irq = flicker_irq;
			if (init_flicker_input(chip))
				return -ENODEV;
			if (sysfs_create_link(&chip->input->dev.kobj,
					&flicker.input->dev.kobj, "flicker"))
				//do nothing
			flicker.callbacks.fifo = flicker_fifo;
			INIT_WORK(&flicker.work, flicker_work_cb);
		}
		return 0;
	}
	return -ENODEV;
}
