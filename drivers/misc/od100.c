/*
 * This file is the Occupancy Detect PIR motion sensor driver.
 *
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>

#define OD100_BOARD_ID		0x00
#define OD100_BLD_VER		0x01
#define OD100_APP_MAJOR		0x02
#define OD100_APP_MINOR		0x03

#define OD100_ZM_CONFIG0	0x05
#define OD100_ZM_CONFIG1	0x06
#define OD100_ZM_CONFIG2	0x07

#define OD100_MOTION_STATUS	0x1b
#define OD100_BL_CMD		0x1c
#define OD100_BL_RESP		0x1d
#define OD100_BL_CKSUM		0x1e
#define OD100_BL_LEN		0x1f
/* Bootloader Data	 RW	(64 bytes; to 0x5F) */
#define OD100_BL_DATA		0x20

/* Boot Loader Commands */
#define START_BL		0x01
#define APP_CS_CHK		0x02
#define START_APP		0x03
#define WR_DATA			0x04
#define FL_ERASE		0x05

/* Boot Loader Responses */
#define _ERASE_OK		0x33
#define _WR_OK			0x44
#define _READY			0x99
#define _APP_CTRL		0xAA
#define _NO_WR			0xBB
#define _BAD_APP		0xCE

#define OD100_NUM_PIR_SENSORS		3

/* Device ID is 32 bits
 * 16 bits Major ID followed by 16 bits Minor ID
 */
#define OD100_DEVICE_ID_MAJOR		0x0D00
#define OD100_DEVICE_ID_MAJOR_SHIFT	16
#define OD100_DEVICE_ID_MINOR_SHIFT	0

/* Firmware version is 32 bits
 * 16 bits bootloader version, followed by app version
 * App version has 8 bits major ID followed by 8 bits minor ID
 */
#define OD100_FWR_BL_VER_SHIFT		16
#define OD100_FWR_APP_MAJOR_SHIFT	8
#define OD100_FWR_APP_MINOR_SHIFT	0

#define OD100_CHIP_NAME			"OD100"

/* Hold the reset for 5 usec */
#define OD100_RESET_DELAY_US		5
/* Bootloader delay before it's ready for I2C comm */
#define OD100_BLD_STARTUP_DELAY_US	10000
/*
 * App delay before it's ready for I2C comm.
 * (200ms = (33ms for checksum calc + 167ms for startup)) * 10 for test.
 */
#define OD100_APP_STARTUP_DELAY_MS	200
/* Entire flash erase delay */
#define OD100_CHIP_ERASE_DELAY_MS	2000
/* Block flash delay */
#define OD100_BLOCK_FLASH_DELAY_US	4000

/* Flash block size could range from 2 to 256 bytes */
#define OD100_FLASH_BLOCK_SZ		32

/* Firmware major and minor version index into firmware buffer */
#define OD100_APP_MAJOR_INDEX	112
#define OD100_APP_MINOR_INDEX	113

/* Wake lock timeout */
#define OD100_WAKE_LOCK_TIMEOUT_MS	200

/* Filter out disabled sensor notification */
#define FILTER_OUT_DISABLED_SENSOR

struct od100_chip {
	char	chipname[10];
	struct	i2c_client *client;
	struct	mutex mutex;
	struct	wake_lock lock;
	struct	delayed_work init_work;
	struct	workqueue_struct *od100_wq;
	struct	work_struct fwupdate_work;
	u8	hw_board_id;
	u32	firmware_ver;
	u32	device_id;
	u32	sensor_enable;
	u32	sensor_detect;
	u8	sensor_range[OD100_NUM_PIR_SENSORS];
	u8	sensor_sensitivity[OD100_NUM_PIR_SENSORS];
	int	reset_gpio;
	int	reset_active_polarity;
	const char *fw_name;
	bool	disable_wake;
};

static int od100_do_firmware_update(struct od100_chip *chip,
				    const struct firmware *fw, bool force)
{
	struct i2c_client *client = chip->client;
	int retval = 0;
	unsigned char val[3];
	int i, j;
	unsigned char check, block_size;
	unsigned int current_version, new_version;

	retval = i2c_smbus_read_byte_data(client, OD100_APP_MAJOR);

	if (retval < 0) {
		dev_err(&client->dev, "fw major version no. read error, %d\n",
			retval);
		goto exit_update;
	}

	current_version = retval << 8;

	retval = i2c_smbus_read_byte_data(client, OD100_APP_MINOR);

	if (retval < 0) {
		dev_err(&client->dev, "fw minor version no. read error, %d\n",
			retval);
		goto exit_update;
	}

	current_version |= retval;

	/* Make sure we won't overrun the fw buffer */
	if (fw->size <= OD100_APP_MINOR_INDEX) {
		dev_err(&client->dev, "Unexpectedly small fw buffer size %d\n",
			(int)fw->size);
		goto exit_update;
	}

	new_version = (fw->data[OD100_APP_MAJOR_INDEX] << 8) +
		      fw->data[OD100_APP_MINOR_INDEX];

	dev_info(&client->dev, "Current fw version is %02X.%02X\n",
		 (current_version & 0xff00) >> 8, (current_version & 0xff));

	dev_info(&client->dev, "Candidate version is %02X.%02X\n",
		 (new_version & 0xff00) >> 8, (new_version & 0xff));

	/* Unless it's a force update, compare the version */
	if (!force && (new_version <= current_version))
		return 0;

	dev_info(&client->dev, "Updating to version %02X.%02X...\n",
		 (new_version & 0xff00) >> 8, (new_version & 0xff));

	/* Disable interrupt during firmware update */
	disable_irq(client->irq);

	/*
	 * Jump to the bootloader: Program magic number 0xE97A41
	 * and issue START_BL command
	 */
	val[0] = 0xE9;
	val[1] = 0x7A;
	val[2] = 0x41;
	i2c_smbus_write_i2c_block_data(client, OD100_BL_DATA, 3, val);

	/* Allow 10 msec to run through bootloader */
	i2c_smbus_write_byte_data(client, OD100_BL_CMD, START_BL);
	usleep_range(OD100_BLD_STARTUP_DELAY_US,
		     OD100_BLD_STARTUP_DELAY_US * 2);

	/* Check the response for bootloader ready */
	if (i2c_smbus_read_byte_data(client, OD100_BL_RESP) != _READY) {
		dev_err(&client->dev, "Failed to jump into bootloader\n");
		retval = -1;

		goto exit_update;
	}

	/*
	 *  Erase the flash area: Program magic number 0xDA63BF
	 * and issue FL_ERASE command
	 */
	val[0] = 0xDA;
	val[1] = 0x63;
	val[2] = 0xBF;
	i2c_smbus_write_i2c_block_data(client, OD100_BL_DATA, 3, val);

	/* Start chip erase and wait until it finishes */
	i2c_smbus_write_byte_data(client, OD100_BL_CMD, FL_ERASE);
	msleep(OD100_CHIP_ERASE_DELAY_MS);

	/* Check the response for erase success */
	if (i2c_smbus_read_byte_data(client, OD100_BL_RESP) != _ERASE_OK) {
		dev_err(&client->dev, "Failed to erase app flash area\n");
		retval = -1;

		goto exit_update;
	}

	/* Flash OD100_FLASH_BLOCK_SZ bytes at a time */
	block_size = OD100_FLASH_BLOCK_SZ;
	for (i = 0; i < (int)fw->size; i += OD100_FLASH_BLOCK_SZ) {
		if ((i + OD100_FLASH_BLOCK_SZ) > (int)fw->size)
			block_size = (int)fw->size % OD100_FLASH_BLOCK_SZ;

		dev_dbg(&client->dev, "%5d of %d  %02d\n", i,
			(int)fw->size, block_size);

		check = block_size;

		for (j = 0; j < block_size; j++)
			check += fw->data[i + j];

		/* Checksum + block length + image data block + send WR_DATA */
		i2c_smbus_write_byte_data(client, OD100_BL_CKSUM, (~check + 1));
		i2c_smbus_write_byte_data(client, OD100_BL_LEN, block_size);
		i2c_smbus_write_i2c_block_data(client, OD100_BL_DATA,
					       block_size, &(fw->data[i]));
		i2c_smbus_write_byte_data(client, OD100_BL_CMD, WR_DATA);

		/* Wait for each block flash completion */
		usleep_range(OD100_BLOCK_FLASH_DELAY_US,
			     OD100_BLOCK_FLASH_DELAY_US * 2);

		/* Check the response for block flash */
		retval = i2c_smbus_read_byte_data(client, OD100_BL_RESP);
		if (retval != _WR_OK) {
			if (retval == _NO_WR) {
				dev_err(&client->dev,
					"Block flash checksum mismatch (%d)\n",
					retval);
			} else {
				dev_err(&client->dev,
					"Unknown response during flash (%d)\n",
					retval);
			}
			retval = -1;
			goto exit_update;
		}
	}

	/*
	 * Upon update completion, issue START_APP command to jump to app
	 */
	i2c_smbus_write_byte_data(client, OD100_BL_CMD, START_APP);
	msleep(OD100_APP_STARTUP_DELAY_MS);

	/* Check the response for booting into app */
	retval = i2c_smbus_read_byte_data(client, OD100_BL_RESP);
	if (retval != _APP_CTRL) {
		if (retval == _BAD_APP) {
			dev_err(&client->dev,
				"Failed to start app (checksum mismatch)\n");
		} else {
			dev_err(&client->dev,
				"Unknown response starting app (%d)\n",
				retval);
		}
		retval = -1;
	/* Success */
	} else {
		chip->firmware_ver = (chip->firmware_ver & 0xffff0000) |
				     new_version;
		retval = 0;
	}

exit_update:
	/* Clear any pending OD interrupt status */
	i2c_smbus_read_byte_data(client, OD100_MOTION_STATUS);
	enable_irq(client->irq);

	dev_info(&client->dev, "OD100 update %s\n",
		 ((retval == 0) ? "success" : "failure"));

	return retval;
}

static int od100_firmware_check(struct od100_chip *chip, bool force_update)
{
	struct i2c_client *client = chip->client;
	const struct firmware *fw;
	int error;
	u8 status;

	/* Get the firmware image */
	error = request_firmware(&fw, chip->fw_name, &client->dev);
	if (error) {
		dev_err(&client->dev, "unable to retrieve firmware %s: %d\n",
			chip->fw_name, error);
		return error;
	}

	dev_info(&client->dev, "Retrieve firmware %s\n", chip->fw_name);

	/* By now, we should have jumped to app. Read response */
	error = i2c_smbus_read_byte_data(chip->client, OD100_BL_RESP);
	if (error < 0)
		goto exit;

	status = (u8)error;

	/* Print current status read for debugging */
	dev_info(&client->dev, "Read response 0x%02x\n", status);

	/*
	 * We are running in the app correctly or failed to jump into app.
	 * Regardless, firmware update can commence.
	 */
	if ((status == _APP_CTRL) || (status == _BAD_APP)) {
		error = od100_do_firmware_update(chip, fw, force_update);
	} else {
		dev_err(&client->dev,
			"Aborting fw update with unexpected response 0x%02x\n",
			status);
	}

exit:
	release_firmware(fw);

	return error;
}

static int od100_detect(struct od100_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;
	u8 bootloader_ver;
	u8 app_major;
	u8 app_minor;
	u8 buf[4];

	/* Check if we need to update firmware. Wait until it's finished */
	od100_firmware_check(chip, false);

	/* Read first four registers */
	ret = i2c_smbus_read_i2c_block_data(client, OD100_BOARD_ID, 4, buf);
	if (ret < 0)
		goto error;

	chip->hw_board_id = buf[0];
	bootloader_ver = buf[1];
	app_major = buf[2];
	app_minor = buf[3];

	/* Minor id of device id is same as hw_board_id */
	chip->device_id = ((OD100_DEVICE_ID_MAJOR <<
				OD100_DEVICE_ID_MAJOR_SHIFT) |
				(chip->hw_board_id <<
				OD100_DEVICE_ID_MINOR_SHIFT));

	chip->firmware_ver = (bootloader_ver <<
				OD100_FWR_BL_VER_SHIFT) |
				(app_major << OD100_FWR_APP_MAJOR_SHIFT) |
				(app_minor << OD100_FWR_APP_MINOR_SHIFT);
	snprintf(chip->chipname, sizeof(chip->chipname), OD100_CHIP_NAME);

	return 0;

error:
	dev_dbg(&client->dev, "OD100 not found\n");

	return ret;
}

static void od100_update_config_reg(struct od100_chip *chip, int pir)
{
	u8 val;

	if ((pir < 0) || (pir >= OD100_NUM_PIR_SENSORS))
		return;

	val = (chip->sensor_sensitivity[pir] << 4) |
	      (chip->sensor_range[pir] << 1) |
	      !!(chip->sensor_enable & (1 << pir));

	i2c_smbus_write_byte_data(chip->client, OD100_ZM_CONFIG0 + pir, val);
}

static void od100_init_work(struct work_struct *work)
{
	int i;

	struct od100_chip *chip =
		container_of(work, struct od100_chip, init_work.work);

	if (!od100_detect(chip)) {
		/* Initialize config regs */
		for (i = 0; i < OD100_NUM_PIR_SENSORS; i++)
			od100_update_config_reg(chip, i);

		chip->sensor_detect =
			i2c_smbus_read_byte_data(chip->client,
						 OD100_MOTION_STATUS);
		enable_irq(chip->client->irq);
	}
}

static void od100_fwupdate_work(struct work_struct *work)
{
	struct od100_chip *chip =
		container_of(work, struct od100_chip, fwupdate_work);

	/* Force fwupdate */
	od100_firmware_check(chip, true);
}

/*
 * Chip on / off functions are called while keeping mutex except probe
 * or remove phase
 */
static void od100_device_on(struct od100_chip *chip)
{
	/* Bring the device out of reset */
	gpio_set_value(chip->reset_gpio, chip->reset_active_polarity);
	udelay(OD100_RESET_DELAY_US);
	gpio_set_value(chip->reset_gpio, !chip->reset_active_polarity);

	/* Allow time to run through bootloader */
	usleep_range(OD100_BLD_STARTUP_DELAY_US,
		     OD100_BLD_STARTUP_DELAY_US * 2);

	/* Attempt to stat application */
	i2c_smbus_write_byte_data(chip->client, OD100_BL_CMD, START_APP);

	/* Check the command response after some required delay  */
	queue_delayed_work(chip->od100_wq, &chip->init_work,
			   msecs_to_jiffies(OD100_APP_STARTUP_DELAY_MS));

}

static void od100_device_off(struct od100_chip *chip)
{
	gpio_set_value(chip->reset_gpio, chip->reset_active_polarity);
}

/* This is threaded irq handler */
static irqreturn_t od100_irq(int irq, void *data)
{
	struct od100_chip *chip = data;

	/*
	 * The current firmware does not honor sensor_enable yet and always
	 * have all three sensors enabled. To reduce noise at the Android
	 * HAL level, filter out activities at disabled sensors and do not
	 * send sysfs notifications. Remove this once firmware supports
	 * sensor enable feature (ALOHASW-811).
	 */
#ifdef FILTER_OUT_DISABLED_SENSOR
	u32 sensor_detect = i2c_smbus_read_byte_data(chip->client,
						     OD100_MOTION_STATUS);

	/* Check for activities on enabled sensor only */
	if ((sensor_detect & chip->sensor_enable) !=
		(chip->sensor_detect & chip->sensor_enable)) {
		sysfs_notify(&chip->client->dev.kobj, NULL, "sensor_detect");
	}
	chip->sensor_detect = sensor_detect;
#else
	chip->sensor_detect = i2c_smbus_read_byte_data(chip->client,
						       OD100_MOTION_STATUS);
	sysfs_notify(&chip->client->dev.kobj, NULL, "sensor_detect");
#endif

	/*
	 * If wake feature is not disabled, wake up the system and prevent
	 * it from entering suspend again for a short while.
	 */
	if (!chip->disable_wake) {
		wake_lock_timeout(&chip->lock,
				  msecs_to_jiffies(OD100_WAKE_LOCK_TIMEOUT_MS));
	}

	return IRQ_HANDLED;
}

static ssize_t od100_device_id_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct od100_chip *chip =  dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%08x\n", chip->device_id);
}

static ssize_t od100_firmware_ver_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct od100_chip *chip =  dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%08x\n", chip->firmware_ver);
}

static ssize_t od100_sensor_num_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", OD100_NUM_PIR_SENSORS);
}

static ssize_t od100_sensor_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct od100_chip *chip =  dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->sensor_enable);
}

static ssize_t od100_sensor_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	unsigned long value;
	int ret, i;

	ret = kstrtoul(buf, 0, &value);
	if (ret)
		return ret;

	if (value >= (1 << OD100_NUM_PIR_SENSORS))
		return -EINVAL;

	mutex_lock(&chip->mutex);
	chip->sensor_enable = value;
	for (i = 0; i < OD100_NUM_PIR_SENSORS; i++)
		od100_update_config_reg(chip, i);
	mutex_unlock(&chip->mutex);

	return len;
}

static ssize_t od100_sensor_detect_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct od100_chip *chip =  dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&chip->mutex);
	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		       (chip->sensor_detect & chip->sensor_enable));
	mutex_unlock(&chip->mutex);

	return ret;

}

static ssize_t od100_sensor_range_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	ssize_t size = 0;
	int i;

	for (i = 0; i < OD100_NUM_PIR_SENSORS - 1; i++) {
		size += snprintf(&buf[size], PAGE_SIZE - size, "%d ",
				 chip->sensor_range[i]);
	}
	size += snprintf(&buf[size], PAGE_SIZE - size, "%d\n",
			 chip->sensor_range[i]);
	return size;
}

static ssize_t od100_sensor_range_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	int range[OD100_NUM_PIR_SENSORS];
	int i, idx = 0;
	const char *p = buf;

	while (*p && (idx < 3)) {
		if (isdigit(*p)) {
			unsigned long value;
			char num[2] = {0};

			/*
			 * Assume the number is a single digit.
			 * Need to do this as kstrtoul returns
			 * -EINVAL if it sees a space character.
			 */
			num[0] = *p;
			if (kstrtoul(num, 10, &value))
				return -EINVAL;

			/* Check the valid target step range */
			if ((value >= 0) && (value < 8)) {
				range[idx++] = value;
				p++;
			} else {
				return -EINVAL;
			}
		} else {
			p++;
		}
	}

	mutex_lock(&chip->mutex);
	for (i = 0; i < OD100_NUM_PIR_SENSORS; i++) {
		chip->sensor_range[i] = range[i];
		od100_update_config_reg(chip, i);
	}
	mutex_unlock(&chip->mutex);

	return len;
}

static ssize_t od100_sensor_sensitivity_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	ssize_t size = 0;
	int i;

	for (i = 0; i < OD100_NUM_PIR_SENSORS - 1; i++) {
		size += snprintf(&buf[size], PAGE_SIZE - size, "%d ",
				 chip->sensor_sensitivity[i]);
	}
	size += snprintf(&buf[size], PAGE_SIZE - size, "%d\n",
			 chip->sensor_sensitivity[i]);
	return size;
}

static ssize_t od100_sensor_sensitivity_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	int sensitivity[OD100_NUM_PIR_SENSORS];
	int i, idx = 0;
	const char *p = buf;

	while (*p && (idx < 3)) {
		if (isdigit(*p)) {
			unsigned long value;
			char num[2] = {0};

			/*
			 * Assume the number is a single digit.
			 * Need to do this as kstrtoul returns
			 * -EINVAL if it sees a space character.
			 */
			num[0] = *p;
			if (kstrtoul(p, 10, &value))
				return -EINVAL;

			/* Check the valid target step range */
			if ((value >= 0) && (value < 8)) {
				sensitivity[idx++] = value;
				p++;
			} else {
				return -EINVAL;
			}
		} else {
			p++;
		}
	}

	mutex_lock(&chip->mutex);
	for (i = 0; i < OD100_NUM_PIR_SENSORS; i++) {
		chip->sensor_sensitivity[i] = sensitivity[i];
		od100_update_config_reg(chip, i);
	}
	mutex_unlock(&chip->mutex);

	return len;
}

static ssize_t od100_disable_wake_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct od100_chip *chip =  dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->disable_wake);
}

static ssize_t od100_disable_wake_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 0, &value);
	if (ret)
		return ret;

	/* Only 0 or 1 */
	if (value > 1)
		return -EINVAL;

	chip->disable_wake = value;

	return len;
}

static ssize_t od100_firmware_update_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t len)
{
	struct od100_chip *chip = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 0, &value);
	if (ret)
		return ret;

	/* Write 1 to start update */
	if (value != 1)
		return -EINVAL;

	queue_work(chip->od100_wq, &chip->fwupdate_work);

	return len;
}

static DEVICE_ATTR(device_id, S_IRUGO, od100_device_id_show, NULL);
static DEVICE_ATTR(firmware_ver, S_IRUGO, od100_firmware_ver_show, NULL);
static DEVICE_ATTR(sensor_num, S_IRUGO, od100_sensor_num_show, NULL);
static DEVICE_ATTR(sensor_enable, S_IRUGO | S_IWUSR, od100_sensor_enable_show,
		   od100_sensor_enable_store);
static DEVICE_ATTR(sensor_detect, S_IRUGO, od100_sensor_detect_show, NULL);
static DEVICE_ATTR(sensor_range, S_IRUGO | S_IWUSR, od100_sensor_range_show,
		   od100_sensor_range_store);
static DEVICE_ATTR(disable_wake, S_IRUGO | S_IWUSR, od100_disable_wake_show,
		   od100_disable_wake_store);
static DEVICE_ATTR(sensor_sensitivity, S_IRUGO | S_IWUSR,
		   od100_sensor_sensitivity_show,
		   od100_sensor_sensitivity_store);
static DEVICE_ATTR(firmware_update, S_IWUSR, NULL, od100_firmware_update_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_device_id.attr,
	&dev_attr_firmware_ver.attr,
	&dev_attr_sensor_num.attr,
	&dev_attr_sensor_enable.attr,
	&dev_attr_sensor_detect.attr,
	&dev_attr_sensor_range.attr,
	&dev_attr_disable_wake.attr,
	&dev_attr_sensor_sensitivity.attr,
	&dev_attr_firmware_update.attr,
	NULL
};

static struct attribute_group od100_attribute_group = {
	.attrs = sysfs_attrs
};

static int od100_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct od100_chip *chip;
	int i, err;
	enum of_gpio_flags flags;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client  = client;

	mutex_init(&chip->mutex);
	wake_lock_init(&chip->lock, WAKE_LOCK_SUSPEND, "od100_wake_lock");
	INIT_DELAYED_WORK(&chip->init_work, od100_init_work);
	chip->od100_wq = create_singlethread_workqueue("od100_wq");
	INIT_WORK(&chip->fwupdate_work, od100_fwupdate_work);

	chip->disable_wake = 0;
	chip->sensor_enable = 0;
	chip->sensor_detect = 0;
	for (i = 0; i < OD100_NUM_PIR_SENSORS; i++) {
		chip->sensor_range[i] = 0;
		chip->sensor_sensitivity[i] = 0;
	}
	chip->fw_name = NULL;

	/* Get the reset gpio */
	chip->reset_gpio = of_get_named_gpio_flags(client->dev.of_node,
						   "qcom,reset", 0, &flags);

	if (!gpio_is_valid(chip->reset_gpio) ||
	    gpio_request(chip->reset_gpio, "reset")) {
		dev_err(&client->dev, "Failed to get reset gpio\n");
		err = -EINVAL;
		goto fail0;
	}

	chip->reset_active_polarity = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	if (of_property_read_string(client->dev.of_node, "fw_name",
				    &chip->fw_name)) {
		dev_warn(&chip->client->dev, "\"fw_name\" path not found\n");
	} else {
		dev_info(&chip->client->dev, "\"fw_name\" = %s\n",
			 chip->fw_name);
	}

	/* Place the chip in reset */
	if (gpio_direction_output(chip->reset_gpio,
				  chip->reset_active_polarity)) {
		dev_err(&client->dev, "Failed to set dir for reset\n");
		goto fail1;
	}

	/* Start chip */
	od100_device_on(chip);

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);

	err = sysfs_create_group(&chip->client->dev.kobj,
				 &od100_attribute_group);
	if (err < 0) {
		dev_err(&chip->client->dev, "Sysfs registration failed\n");
		goto fail1;
	}

	err = request_threaded_irq(client->irq, NULL, od100_irq,
				   (IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				   "od100", chip);

	if (err) {
		dev_err(&client->dev, "could not get IRQ %d\n",
			client->irq);
		goto fail2;
	}

	/* Set wake up enable */
	device_init_wakeup(&client->dev, 1);

	/* We will enable IRQ when MCU boots through */
	disable_irq(client->irq);

	return err;

fail2:
	sysfs_remove_group(&chip->client->dev.kobj,
			   &od100_attribute_group);

fail1:
	gpio_free(chip->reset_gpio);

fail0:
	cancel_delayed_work_sync(&chip->init_work);
	cancel_work_sync(&chip->fwupdate_work);
	flush_workqueue(chip->od100_wq);
	destroy_workqueue(chip->od100_wq);
	wake_lock_destroy(&chip->lock);
	mutex_destroy(&chip->mutex);

	return err;
}

static int od100_remove(struct i2c_client *client)
{
	struct od100_chip *chip = i2c_get_clientdata(client);

	/* Clear wake up enable */
	device_init_wakeup(&client->dev, 0);

	gpio_free(chip->reset_gpio);
	free_irq(client->irq, chip);

	sysfs_remove_group(&chip->client->dev.kobj,
			   &od100_attribute_group);

	cancel_delayed_work_sync(&chip->init_work);
	cancel_work_sync(&chip->fwupdate_work);
	flush_workqueue(chip->od100_wq);
	destroy_workqueue(chip->od100_wq);
	wake_lock_destroy(&chip->lock);
	mutex_destroy(&chip->mutex);

	if (!pm_runtime_suspended(&client->dev))
		od100_device_off(chip);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int od100_suspend(struct device *dev)
{
	struct od100_chip *chip = dev_get_drvdata(dev);

	if (chip->disable_wake)
		return 0;

	if (device_may_wakeup(dev))
		enable_irq_wake(chip->client->irq);

	return 0;
}

static int od100_resume(struct device *dev)
{
	struct od100_chip *chip = dev_get_drvdata(dev);

	if (chip->disable_wake)
		return 0;

	if (device_may_wakeup(dev))
		disable_irq_wake(chip->client->irq);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int od100_runtime_suspend(struct device *dev)
{
	return 0;
}

static int od100_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct i2c_device_id od100_id[] = {
	{"od100", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, od100_id);

static const struct dev_pm_ops od100_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(od100_suspend, od100_resume)
	SET_RUNTIME_PM_OPS(od100_runtime_suspend, od100_runtime_resume, NULL)
};

static struct i2c_driver od100_driver = {
	.driver	 = {
		.name	= "od100",
		.owner	= THIS_MODULE,
		.pm	= &od100_pm_ops,
	},
	.probe	  = od100_probe,
	.remove	  = od100_remove,
	.id_table = od100_id,
};

module_i2c_driver(od100_driver);

MODULE_DESCRIPTION("OD100 Occupancy Detect PIR motion sensor");
MODULE_LICENSE("GPL v2");
