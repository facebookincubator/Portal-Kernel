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
#include <linux/i2c/ams/ams_tcs3408.h>
#include <linux/i2c/ams/ams_tcs3408_reg.h>
#include "ams_i2c.h"
#include <linux/kfifo.h>
#include "ams_tcs3408_als.h"
#include "ams_tcs3408_fifo.h"

#define STR1_MAX (16)
#define STR2_MAX (256)
#define MAX_FIFO_LEN (1024)

struct fifo_chip {
	struct feature_callbacks callbacks;
	struct work_struct work;
	uint8_t channels;
	uint8_t map;
	uint8_t level;
	uint32_t k_len;
	uint32_t overflow;
	uint32_t koverflow;
	uint8_t max;
	enum fifo_mode mode;
	struct ams_chip *chip;
};

static uint8_t data[256];
static DECLARE_KFIFO(ams_fifo, u8, 2 * PAGE_SIZE);

static struct fifo_chip *get_fifo_chip(struct ams_chip *chip)
{
	return (struct fifo_chip *)chip->fifo;
}

void set_fifo_map(struct ams_chip *chip, uint8_t map)
{
	struct fifo_chip *fifo = get_fifo_chip(chip);

	fifo->map = map;
	ams_i2c_write(chip->client, chip->shadow,
			TCS3408_REG_FIFO_MAP, fifo->map);
	ams_i2c_modify(chip->client, chip->shadow,
			0xD7, 0x80, 0x00);
}

static void fifo_reset(struct ams_chip *chip)
{
	struct fifo_chip *fifo = get_fifo_chip(chip);

	kfifo_reset(&ams_fifo);
	fifo->koverflow = 0;
	fifo->overflow = 0;
	ams_i2c_set_field(chip->client, chip->shadow,
			TCS3408_REG_CONTROL, 0x1, 0x1, 0x1);
}

static void enable_fifo(struct ams_chip *chip, uint8_t channels)
{
	struct fifo_chip *fifo = get_fifo_chip(chip);

	if (channels >= 8) {
		set_fifo_map(chip, 0);
		fifo->channels = 1;
		ams_i2c_modify(chip->client, chip->shadow,
				0xD7, 0x80, 0x80); //fd FIFO mode on
	} else if (channels >= 7) {
		set_fifo_map(chip, 0x80);
		fifo->channels = 1;
	} else if (channels > 0) {
		set_fifo_map(chip, 0xFE & (~(0xFE << channels)));
		fifo->channels = channels;
	} else {
		set_fifo_map(chip, 0);
		fifo->channels = 0;
		ams_i2c_modify(chip->client, chip->shadow,
				TCS3408_REG_INTENAB, 0x04, 0x00);
		return;
	}
	ams_i2c_modify(chip->client, chip->shadow,
			TCS3408_REG_INTENAB, 0x04, 0x04); //fifo interrupt
	fifo_reset(chip);
}

static int get_kfifo_len(void)
{
	return kfifo_len(&ams_fifo);
}

ssize_t read_fifo(struct ams_chip *chip, char *buf, int size)
{
	int len;

	if (kfifo_len(&ams_fifo) >= size) {
		len = kfifo_out(&ams_fifo, buf, size);
	} else {
		dev_info(&chip->client->dev, "%s: fifo_len: %d < 256\n",
				__func__, kfifo_len(&ams_fifo));
		len = 0;
	}
	return len;
}

static void fifo_work_cb(struct work_struct *work)
{
	struct fifo_chip *fifo = NULL;

	if (work)
		fifo = container_of(work, struct fifo_chip, work);
}

static ssize_t fifo_read(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct ams_chip *chip = dev_get_drvdata(dev);
	int read;

	if ((size < 256) && !chip->agc_disable) {
		dev_info(dev, "%s: Error size: %zd is less than 256B\n",
				__func__, size);
		return -EAGAIN;
	}

	AMS_MUTEX_LOCK(&chip->lock);
	read = read_fifo(chip, buf, size);
	AMS_MUTEX_UNLOCK(&chip->lock);
	return read;
}

static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->lock);
	fifo_reset(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return count;
}

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct fifo_chip *fifo = get_fifo_chip(chip);
	ssize_t ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", fifo->level);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t overflowx_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct fifo_chip *fifo = get_fifo_chip(chip);
	ssize_t ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", fifo->overflow);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t k_overflowx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct fifo_chip *fifo = get_fifo_chip(chip);
	ssize_t ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", fifo->koverflow);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t k_isfull_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct fifo_chip *fifo = get_fifo_chip(chip);
	ssize_t ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			get_kfifo_len() == fifo->max ? 1 : 0);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t k_len_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	struct fifo_chip *fifo = get_fifo_chip(chip);
	ssize_t ret;
	int len;

	AMS_MUTEX_LOCK(&chip->lock);
	len = get_kfifo_len();
	ret = snprintf(buf, PAGE_SIZE, "%d\n", len);
	if (len == fifo->max)
		dev_err(&chip->client->dev,
				"kfifo_len == KFIFO_MAX [%d > %d]\n",
				len, fifo->max);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t threshold_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;
	uint8_t map;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_i2c_read(chip->client, TCS3408_REG_CFG8,
			&chip->shadow[TCS3408_REG_CFG8]);
	map = chip->shadow[TCS3408_REG_CFG8];
	ret = snprintf(buf, PAGE_SIZE, "%d\n", ((map & 0xC0) >> 6));
	AMS_MUTEX_UNLOCK(&chip->lock);

	return ret;
}

static ssize_t threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	u8 map = 0;
	int rc;

	rc = kstrtoint(buf, 0, (int *)&data);
	if (rc != 0)
		return -ENODEV;

	AMS_MUTEX_LOCK(&chip->lock);
	ams_i2c_modify(chip->client, chip->shadow, TCS3408_REG_CFG8,
			0xC0, ((map << 6) & 0xC0));
	AMS_MUTEX_UNLOCK(&chip->lock);
	return count;
}

static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_RW(threshold);
static BIN_ATTR_RO(fifo, PAGE_SIZE);
static DEVICE_ATTR_RO(level);
static DEVICE_ATTR_RO(overflowx);
static DEVICE_ATTR_RO(k_isfull);
static DEVICE_ATTR_RO(k_overflowx);
static DEVICE_ATTR_RO(k_len);

static struct attribute *attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_level.attr,
	&dev_attr_overflowx.attr,
	&dev_attr_k_isfull.attr,
	&dev_attr_k_overflowx.attr,
	&dev_attr_k_len.attr,
	&dev_attr_threshold.attr,
	NULL,
};

static struct bin_attribute *bin_attrs[] = {
	&bin_attr_fifo,
	NULL,
};

static const struct attribute_group attrs_group = {
	.name = "fifo",
	.attrs = attrs,
	.bin_attrs = bin_attrs,
};

static const struct attribute_group *fifo_groups[] = {
	&attrs_group,
	NULL,
};

static void fifo_callback(struct ams_chip *chip,
		struct feature_callbacks *callbacks, uint8_t *data, int size)
{
	if (callbacks) {
		if (callbacks->fifo)
			callbacks->fifo(chip, data, size);
	}
}

void handle_fifo_data(struct ams_chip *chip, uint8_t *data, int size)
{
	struct fifo_chip *fifo = get_fifo_chip(chip);

	if (!fifo->map && fifo->channels == 1) { //flicker
		fifo_callback(chip, chip->flicker, data, size);
	} else if (fifo->map & 0x80) {
		//not implemented
	} else if ((fifo->map & 0xFE) && fifo->channels > 0) { //als
		fifo_callback(chip, chip->als, data, size);
	} else {
		//error
	}
}

static void fifo_irq(struct ams_chip *chip, uint8_t status)
{
	int size;
	uint8_t val;
	struct fifo_chip *fifo = get_fifo_chip(chip);

	if (status & TCS3408_MASK_FINT) {
		val = chip->shadow[TCS3408_REG_STATUS6];
		val = (val & TCS3408_MASK_FIFO_OV) >> TCS3408_FIFO_OV_SHIFT;
		fifo->overflow += val;
		if (val > 0) {
			dev_err(&chip->client->dev,
					"fifo overflow [0x%x]\n", val);
		}
		ams_i2c_read(chip->client, TCS3408_REG_FIFO_LVL,
				&fifo->level);
		size = (fifo->level - (fifo->level % fifo->channels)) * 2;
		memset(&data, 0x0, sizeof(data));
		ams_i2c_blk_read(chip->client, TCS3408_REG_FDATAL,
				(uint8_t *)&data, size);
		kfifo_in(&ams_fifo, data, size);
		if (kfifo_is_full(&ams_fifo))
			kfifo_reset(&ams_fifo);
		handle_fifo_data(chip, data, kfifo_len(&ams_fifo));
	}
}

void ams_tcs3408_enable_fifo(struct ams_chip *chip, enum fifo_mode mode)
{
	enable_fifo(chip, mode);
}

ssize_t ams_tcs3408_read_fifo(struct ams_chip *chip, char *buf, int size)
{
	return read_fifo(chip, buf, size);
}

void ams_tcs3408_fifo_reset(struct ams_chip *chip)
{
	fifo_reset(chip);
}

void ams_tcs3408_remove_fifo(struct ams_chip *chip)
{
	struct fifo_chip *fifo = get_fifo_chip(chip);

	cancel_work_sync(&fifo->work);
	if (!chip->input)
		return;
	sysfs_remove_groups(&chip->input->dev.kobj, fifo_groups);
	fifo_reset(chip);
}

int ams_tcs3408_init_fifo(struct ams_chip *chip)
{
	static struct fifo_chip fifo = { {0} };

	fifo.callbacks.irq = fifo_irq;
	chip->fifo = &fifo.callbacks;
	fifo.chip = chip;
	fifo_reset(chip);
	INIT_WORK(&fifo.work, fifo_work_cb);
	INIT_KFIFO(ams_fifo);
	enable_fifo(chip, FIFO_OFF);
	if (sysfs_create_groups(&chip->input->dev.kobj, fifo_groups)) {
		dev_err(&chip->input->dev,
				"Error creating sysfs attribute group.\n");
	}
	return 0;
}
