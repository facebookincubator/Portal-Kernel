/*!
 * @section LICENSE
 * Copyright (c) 2019~2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_driver.c
 * @date     2020/11/10
 * @version  1.3.14
 *
 * @brief    bma4xy Linux Driver
 */

/*********************************************************************/
/* System header files */
/*********************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/*********************************************************************/
/* Own header files */
/*********************************************************************/
#include "bma4xy_driver.h"
#include "bs_log.h"
/*********************************************************************/
/* Local macro definitions */
/*********************************************************************/
#define DRIVER_VERSION "1.3.14"
#define SLOPE_INTERRUPT REL_DIAL
/*********************************************************************/
/* Global data */
/*********************************************************************/
#define MS_TO_US(msec)		UINT32_C((msec) * 1000)

enum BMA4XY_SENSOR_INT_MAP {
	BMA4XY_FFULL_INT = 8,
	BMA4XY_FWM_INT = 9,
	BMA4XY_DRDY_INT = 10,
};

/**
 * enum bma4xy_config_func - Enumerations to select the sensors/interrupt
 */
enum bma4xy_config_func {
	BMA4XY_STEP_DETECTOR_SENSOR = 0,
	BMA4XY_STEP_COUNTER_SENSOR = 1,
	BMA4XY_ANY_MOTION_SENSOR = 2,
	BMA4XY_NO_MOTION_SENSOR = 3,
	BMA4XY_SINGLETAP_SENSOR = 4,
	BMA4XY_DOUBLETAP_SENSOR = 5,
	BMA4XY_TRIPLETAP_SENSOR = 6,
	BMA4XY_ACTIVITY_SENSOR = 7,
	BMA4XY_FIFO_FULL = 8,
	BMA4XY_FIFO_WM = 9,
};

/**
 * enum bma4xy_int_status0 - Enumerations corresponding to status0 registers
 */
enum bma4xy_int_status0 {
	TAP_OUT = 0x01,
	STEP_DET_OUT = 0x02,
	ACTIVITY_OUT = 0x04,
	ANY_MOTION_OUT = 0x10,
	NO_MOTION_OUT = 0x20,
	ERROR_INT_OUT = 0x80,
};

/**
 * enum bma4xy_int_status1 - Enumerations corresponding to status1 registers
 */
enum bma4xy_int_status1 {
	FIFOFULL_OUT = 0x01,
	FIFOWATERMARK_OUT = 0x02,
	MAG_DRDY_OUT = 0x20,
	ACC_DRDY_OUT = 0x80,
};

/*! bma4 fifo analyse return err status */
enum BMA4_FIFO_ANALYSE_RETURN_T {
	FIFO_OVER_READ_RETURN = -10,
	FIFO_SENSORTIME_RETURN = -9,
	FIFO_SKIP_OVER_LEN = -8,
	FIFO_M_A_OVER_LEN = -5,
	FIFO_M_OVER_LEN = -3,
	FIFO_A_OVER_LEN = -1
};

/**
 * bma4xy_i2c_delay_us - Adds a delay in units of microsecs.
 *
 * @usec: Delay value in microsecs.
 */
static void bma4xy_i2c_delay_us(u32 usec, void *intf_ptr)
{

	if (usec <= (MS_TO_US(20)))

		/* Delay range of usec to usec + 1 millisecs
		 * required due to kernel limitation
		 */
		usleep_range(usec, usec + 1000);
	else
		msleep(usec/1000);
}

/**
 * bma4xy_check_chip_id - Checks the chip-id of the sensor.
 * @client_data : Instance of client data.
 *
 * Return: Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data)
{

	u8 chip_id[2] = {0};
	int err = 0;

	err = client_data->device.bus_read(
		BMA4_CHIP_ID_ADDR, chip_id, 2, &client_data->device);
	if (err) {
		PERR("chip id read failed");
		return -EIO;
	}

	if(chip_id[0] != BMA456H_CHIP_ID)
	{
		PERR("chip id not match");
		return -EIO;
	}

	PINFO("chip_id=0x%x rev_id=0x%x\n", chip_id[0], chip_id[1]);
	return 0;
}

static unsigned char fifo_data[1024];
/**
 * calculate_fifo_frame_length - Function to calculate the frame length of fifo
 * @fifo_index: Frame header data
 * @fifo_frmbytes_extend: Length of the fifo frame
 */
static void calculate_fifo_frame_length(u8 fifo_index,
	u8 *fifo_frmbytes_extend)
{
	switch (fifo_index) {
	case BMA4_FIFO_HEAD_SKIP_FRAME:
		*fifo_frmbytes_extend = 1;
		break;
	case BMA4_FIFO_HEAD_M_A:
		*fifo_frmbytes_extend = BMA4_FIFO_MA_LENGTH;
		break;
	case BMA4_FIFO_HEAD_A:
		*fifo_frmbytes_extend = BMA4_FIFO_A_LENGTH;
		break;
	case BMA4_FIFO_HEAD_M:
		*fifo_frmbytes_extend = BMA4_FIFO_M_LENGTH;
		break;
	case BMA4_FIFO_HEAD_SENSOR_TIME:
		*fifo_frmbytes_extend = BMA4_SENSOR_TIME_LENGTH;
		break;
	default:
		*fifo_frmbytes_extend = 0;
		break;
	};
}

/**
 * bma4xy_parse_fifo_data - Function to parse the fifo data from sensor.
 *
 * @client_data: Instance of client data structure.
 * @fifo_length: Length of the fifo buffer to be parsed.
 * @fifo_data: Data from the fifo buffer.
 *
 * Return: Status of the function.
 * * 0			- OK
 * * Negative value	- Failed
 */
int bma4xy_parse_fifo_data(struct bma4xy_client_data *client_data,
	u8 *fifo_data, u16 fifo_length)
{
	u8 frame_head = 0;
	u16 fifo_index = 0;
	s8 last_return_st = 0;
	u8 fifo_frmbytes_extend = 0;
	int err = 0;
	struct bma4_mag_xyzr mag;
	struct bma4_accel acc;

	memset(&acc, 0, sizeof(acc));
	memset(&mag, 0, sizeof(mag));

	for (fifo_index = 0; fifo_index < fifo_length;) {
		/*this use in the Qualcomm platform i2c limit 256 bytes*/
		if (fifo_index < 256) {
			calculate_fifo_frame_length(
				fifo_data[fifo_index], &fifo_frmbytes_extend);
			if ((fifo_index + 1+fifo_frmbytes_extend) > 255)
				fifo_index = 256;
		}
		if ((fifo_index > 256) && (fifo_index < 512)) {
			calculate_fifo_frame_length(
				fifo_data[fifo_index], &fifo_frmbytes_extend);
			if ((fifo_index + 1+fifo_frmbytes_extend) > 511)
				fifo_index = 512;
		}
		if ((fifo_index > 512) && (fifo_index < 768)) {
			calculate_fifo_frame_length(
				fifo_data[fifo_index], &fifo_frmbytes_extend);
			if ((fifo_index + 1 + fifo_frmbytes_extend) > 767)
				fifo_index = 768;
		}
		frame_head = fifo_data[fifo_index];
		switch (frame_head) {
			/*skip frame 0x40 22 0x84*/
		case BMA4_FIFO_HEAD_SKIP_FRAME:
		/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + 1 > fifo_length) {
				last_return_st = FIFO_SKIP_OVER_LEN;
				break;
			}
			/*skip_frame_cnt = fifo_data[fifo_index];*/
			fifo_index = fifo_index + 1;
		break;
		case BMA4_FIFO_HEAD_M_A:
		{/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + MA_BYTES_FRM > fifo_length) {
				last_return_st = FIFO_M_A_OVER_LEN;
				break;
			}
			mag.x = fifo_data[fifo_index + 1] << 8|
				fifo_data[fifo_index + 0];
			mag.y = fifo_data[fifo_index + 3] << 8 |
				fifo_data[fifo_index + 2];
			mag.z = fifo_data[fifo_index + 5] << 8 |
				fifo_data[fifo_index + 4];
			mag.r = fifo_data[fifo_index + 7] << 8 |
				fifo_data[fifo_index + 6];
			acc.x = fifo_data[fifo_index + 9] << 8 |
				fifo_data[fifo_index + 8];
			acc.y = fifo_data[fifo_index + 11] << 8 |
				fifo_data[fifo_index + 10];
			acc.z = fifo_data[fifo_index + 13] << 8 |
				fifo_data[fifo_index + 12];
			fifo_index = fifo_index + MA_BYTES_FRM;
			break;
		}
		case BMA4_FIFO_HEAD_A:
		{	/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + A_BYTES_FRM > fifo_length) {
				last_return_st = FIFO_A_OVER_LEN;
				break;
			}
			acc.x = fifo_data[fifo_index + 1] << 8 |
				fifo_data[fifo_index + 0];
			acc.y = fifo_data[fifo_index + 3] << 8 |
				fifo_data[fifo_index + 2];
			acc.z = fifo_data[fifo_index + 5] << 8 |
				fifo_data[fifo_index + 4];
			fifo_index = fifo_index + A_BYTES_FRM;
			break;
		}
		case BMA4_FIFO_HEAD_M:
		{	/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + M_BYTES_FRM > fifo_length) {
				last_return_st = FIFO_M_OVER_LEN;
				break;
			}
			mag.x = fifo_data[fifo_index + 1] << 8 |
				fifo_data[fifo_index + 0];
			mag.y = fifo_data[fifo_index + 3] << 8 |
				fifo_data[fifo_index + 2];
			mag.z = fifo_data[fifo_index + 5] << 8 |
				fifo_data[fifo_index + 4];
			mag.r = fifo_data[fifo_index + 7] << 8 |
				fifo_data[fifo_index + 6];
			fifo_index = fifo_index + M_BYTES_FRM;
			break;
		}
		/* sensor time frame*/
		case BMA4_FIFO_HEAD_SENSOR_TIME:
		{
			/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + 3 > fifo_length) {
				last_return_st = FIFO_SENSORTIME_RETURN;
				break;
			}
			/*fifo sensor time frame index + 3*/
			fifo_index = fifo_index + 3;
			break;
		}
		case BMA4_FIFO_HEAD_OVER_READ_MSB:
		{
			/*fifo data frame index + 1*/
			fifo_index = fifo_index + 1;
			if (fifo_index + 1 > fifo_length) {
				last_return_st = FIFO_OVER_READ_RETURN;
				break;
			}
			if (fifo_data[fifo_index] == BMA4_FIFO_HEAD_OVER_READ_MSB) {
				/*fifo over read frame index + 1*/
				fifo_index = fifo_index + 1;
				break;
			}
			last_return_st = FIFO_OVER_READ_RETURN;
		break;
		}
		default:
			last_return_st = 1;
		break;
		}
			if (last_return_st)
				break;
	}
	return err;
}

/**
 * chip_id_show - sysfs callback for reading the chip id of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t chip_id_show(struct device *dev,	struct device_attribute *attr,
							char *buf)
{
	u8 chip_id[2] = {0};
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = client_data->device.bus_read(
		BMA4_CHIP_ID_ADDR, chip_id, 2, &client_data->device);
	if (err) {
		PERR("chip id read failed");
		return -EIO;
	}

	return snprintf(buf, 96, "%d\n", chip_id[0]);
}
static ssize_t scale_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int scale_value[] = {16384,8192,4096,2048};
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 16, "%d\n", scale_value[acc_config.range]);
}

static ssize_t delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 96, "%d\n", atomic_read(&client_data->delay));
}

static ssize_t delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data = 0;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (data > BMA456_MAX_DELAY)
		data = BMA456_MAX_DELAY;
	atomic_set(&client_data->delay, (unsigned int) data);

	return count;
}



/**
 * acc_op_mode_show - sysfs callback which tells whether accelerometer is
 * enabled or disabled.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_dev *bma4_dev = &client_data->device;

	return snprintf(buf, 96, "%d\n", bma4_dev->en_count);
}

static int enable_store_impl(struct bma4xy_client_data *client_data,
					unsigned int op_mode) {
	struct bma4_dev *bma4_dev = &client_data->device;
	int enable_count = bma4_dev->en_count;
	int err = 0;

	if (op_mode == 0) {
		if (enable_count == 1)
			cancel_delayed_work_sync(&client_data->work);
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
	} else if (op_mode == 1) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		if (!err && enable_count == 0) {
			schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(atomic_read(&client_data->delay)));
		}
	}
	return err;
}

/**
 * acc_op_mode_store - sysfs callback which enables or disables the
 * accelerometer.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 * Accelerometer will not be disabled unless all the features related to
 * accelerometer are disabled.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	enable_store_impl(client_data, (unsigned int) op_mode);

	client_data->pw.acc_pm = op_mode;
	return count;
}

/**
 * acc_value_show - sysfs read callback which gives the
 * raw accelerometer value from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bma4_accel data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma4_read_accel_xyz(&data, &client_data->device);
	if (err < 0)
		return err;
	return snprintf(buf, 48, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

/**
 * acc_range_show - sysfs read callback which gives the
 * accelerometer range which is set in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t range_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 16, "%d\n", acc_config.range);
}

/**
 * acc_range_store - sysfs write callback which sets the
 * accelerometer range to be set in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t range_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long acc_range;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	err = kstrtoul(buf, 10, &acc_range);
	if (err)
		return err;
	err = bma4_get_accel_config(&acc_config, &client_data->device);
	acc_config.range = (u8)(acc_range);
	err += bma4_set_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("faliled");
		return -EIO;
	}
	return count;
}

/**
 * acc_odr_show - sysfs read callback which gives the
 * accelerometer output data rate of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_odr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	client_data->acc_odr = acc_config.odr;
	return snprintf(buf, 16, "%d\n", client_data->acc_odr);
}

/**
 * acc_odr_store - sysfs write callback which sets the
 * accelerometer output data rate in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_odr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long acc_odr;
	u8 data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &acc_odr);
	if (err)
		return err;
	data = (u8)acc_odr;
	if (acc_odr == 4)
		data = 0x74;
	else
		data |= 0xA0;
	err = bma4_read_regs(0x40, &data, 1, &client_data->device);
	if (err) {
		PERR("faliled");
		return -EIO;
	}
	PDEBUG("acc odr = %d", (u8)acc_odr);
	client_data->acc_odr = acc_odr;
	return count;
}

/**
 * selftest_show - sysfs read callback which gives the
 * accelerometer self test result of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 64, "notrun:2 pass:0 fail:1 current_state->%d\n",
		client_data->selftest);
}

/**
 * selftest_store - sysfs write callback which performs the
 * accelerometer self test in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t selftest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	u8 result = 1;
	unsigned long data;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	err = bma4_perform_accel_selftest(&result, &client_data->device);
	if (err) {
		PERR("selftest execution failed err=%d", err);
		return err;
	}

	if (result == 0) {
		client_data->selftest = result;
		PDEBUG("selftest passed\n");
	} else {
		client_data->selftest = 1;
		PDEBUG("selftest failed with result : %d\n", result);
	}
	return count;
}

/**
 * axis_remap_show - sysfs read callback which reads the format remap axis
 * configureation from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t axis_remap_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_remap axis_remap_data;
	int err = 0;

	err = bma456h_get_remap_axes(&axis_remap_data, &client_data->device);

	return snprintf(buf, 64, "axis_remap x:%d, y:%d, z:%d\n",
		axis_remap_data.x,
		axis_remap_data.y,
		axis_remap_data.z);
}

/**
 * axis_remap_store - sysfs write callback which performs the
 * axis remapping to the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t axis_remap_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int data[3];
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;
	struct bma4_remap axis_remap_data;

	err = sscanf(buf, "%d %d %d",
		&data[0], &data[1], &data[2]);

	if (err != 3) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	axis_remap_data.x = data[0];
	axis_remap_data.y = data[1];
	axis_remap_data.z = data[2];

	PDEBUG("axis_remap x:%d, y:%d, z:%d",
		axis_remap_data.x,
		axis_remap_data.y,
		axis_remap_data.z);

	err = bma456h_set_remap_axes(&axis_remap_data, &client_data->device);

	if (err) {
		PERR("axis remap failed %d", err);
		return err;
	}
	PINFO("axis remap successfully done");
	return count;
}

/**
 * foc_show - sysfs read callback which notifies the format which
 * is to be used to perform accelerometer FOC in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t foc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64,
		"Use echo x_axis y_axis z_axis sign > foc\n");
}

/**
 * foc_store - sysfs write callback which performs the
 * accelerometer FOC in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t foc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma4_accel_foc_g_value g_value = {0};
	unsigned int data[4];
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	err = sscanf(buf, "%d %d %d %d",
		&data[0], &data[1], &data[2], &data[3]);

	if (err != 4) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	g_value.x = data[0];
	g_value.y = data[1];
	g_value.z = data[2];
	g_value.sign = data[3];
	PDEBUG("g_value.x=%d, g_value.y=%d, g_value.z=%d g_value.sign=%d",
		g_value.x, g_value.y, g_value.z, g_value.sign);
	err = bma4_perform_accel_foc(&g_value, &client_data->device);
	if (err) {
		PERR("FOC accel failed %d", err);
		return err;
	}
	PINFO("FOC Accel successfully done");
	return count;
}

/**
 * config_function_show - sysfs read callback which gives the list of
 * enabled features in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t config_function_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}

	return snprintf(buf, PAGE_SIZE,
		"step_detector0=%d\nstep_counter1=%d\nany_mot2=%d\n"
		"no_mot3=%d\nsingle_tap4=%d\ndouble_tap5=%d\ntriple_tap6=%d\n"
		"activity7=%d\nfifo_full_int8=%d\nfifo_wm_int9=%d\n",
		client_data->stepdet_en, client_data->stepcounter_en,
		client_data->anymotion_en, client_data->nomotion_en,
		client_data->single_tap_en, client_data->double_tap_en,
		client_data->triple_tap_en, client_data->activity_en,
		client_data->fifo_full_int_en, client_data->fifo_wm_int_en);
}

/**
 * config_function_store - sysfs write callback which enable or disable
 * the features in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t config_function_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int config_func = 0;
	int enable = 0;
	u16 feature = 0;
	u16 map_int = 0;
	struct bma456h_any_no_mot_config any_no_mot;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	PDEBUG("config_func = %d, enable=%d", config_func, enable);
	if (ret != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	if (config_func < 0 || config_func > 15)
		return -EINVAL;
	switch (config_func) {

	case BMA4XY_STEP_DETECTOR_SENSOR:
		feature = BMA456H_STEP_DETECTOR_EN;
		map_int = BMA456H_STEP_CNTR_INT;
		if (bma456h_step_detector_enable(
			enable, &client_data->device) < 0) {
			PERR("set BMA4XY_STEP_DETECTOR_SENSOR error");
			return -EINVAL;
		}
		client_data->stepdet_en = enable;
		break;
	case BMA4XY_STEP_COUNTER_SENSOR:
		feature = BMA456H_STEP_COUNTER_EN;
		map_int = BMA456H_STEP_CNTR_INT;
		client_data->stepcounter_en = enable;
		break;
	case BMA4XY_ANY_MOTION_SENSOR:
		ret = bma456h_get_any_mot_config(&any_no_mot,
			&client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(20), &client_data->device.intf_ptr);
		map_int = BMA456H_ANY_MOT_INT;
		feature = BMA456H_ANY_MOTION_ALL_AXIS_EN;
		if (enable == 1) {
			any_no_mot.duration = 0xFA;
			ret = bma456h_set_any_mot_config(&any_no_mot,
				&client_data->device);
		}
		client_data->anymotion_en = enable;
		break;
	case BMA4XY_NO_MOTION_SENSOR:
		ret = bma456h_get_no_mot_config(&any_no_mot,
			&client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(20), &client_data->device.intf_ptr);
		map_int = BMA456H_NO_MOT_INT;
		feature = BMA456H_NO_MOTION_ALL_AXIS_EN;
		if (enable == 1) {
			any_no_mot.duration = 0xFA;
			ret = bma456h_set_no_mot_config(&any_no_mot,
				&client_data->device);
		}
		client_data->nomotion_en = enable;
		break;
	case BMA4XY_SINGLETAP_SENSOR:
		feature = BMA456H_SINGLE_TAP_EN;
		map_int = BMA456H_TAP_OUT_INT;
		client_data->single_tap_en = enable;
		break;
	case BMA4XY_DOUBLETAP_SENSOR:
		feature = BMA456H_DOUBLE_TAP_EN;
		map_int = BMA456H_TAP_OUT_INT;
		client_data->double_tap_en = enable;
		break;
	case BMA4XY_TRIPLETAP_SENSOR:
		feature = BMA456H_TRIPLE_TAP_EN;
		map_int = BMA456H_TAP_OUT_INT;
		client_data->triple_tap_en = enable;
		break;
	case BMA4XY_ACTIVITY_SENSOR:
		feature = BMA456H_STEP_ACTIVITY_EN;
		map_int = BMA456H_ACTIVITY_INT;
		client_data->activity_en = enable;
		break;
	case BMA4XY_FIFO_FULL:
		map_int = BMA4_FIFO_FULL_INT;
		client_data->fifo_full_int_en = enable;
		break;
	case BMA4XY_FIFO_WM:
		map_int = BMA4_FIFO_WM_INT;
		client_data->fifo_wm_int_en = enable;
		break;
	default:
		PERR("Invalid sensor handle: %d", config_func);
		return -EINVAL;
	}

	ret = bma456h_feature_enable(feature, enable, &client_data->device);
	if (ret)
		PERR("enabling features failed");

	ret = bma456h_map_interrupt(BMA4_INTR1_MAP, map_int,
			enable, &client_data->device);
	if (ret)
		PERR("interrupt mapping failed");

	return count;
}

/**
 * fifo_length_show - sysfs read callback which gives the length of FIFO data
 * available in the sensor in the units of bytes.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t fifo_length_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u16 fifo_bytecount = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma4_get_fifo_length(&fifo_bytecount, &client_data->device);
	if (err) {
		PERR("read falied");
		return err;
	}
	return snprintf(buf, 96, "%d\n", fifo_bytecount);
}

/**
 * fifo_flush_store - sysfs write callback which flushes the fifo data
 * in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t fifo_flush_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &enable);
	if (err)
		return err;
	if (enable)
		err = bma4_set_command_register(0xb0, &client_data->device);
	if (err) {
		PERR("write failed");
		return -EIO;
	}
	return count;
}

/**
 * acc_fifo_enable_show - sysfs read callback which shows the enable or
 * disable status of the accelerometer FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_fifo_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u8 acc_fifo_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma4_get_fifo_config(&acc_fifo_enable, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 16, "%x\n", acc_fifo_enable);
}

/**
 * acc_fifo_enable_store - sysfs write callback enables or
 * disables the accelerometer FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_fifo_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	unsigned char acc_fifo_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	acc_fifo_enable = (unsigned char)data;
	err = bma4_set_fifo_config(
		BMA4_FIFO_ACCEL, acc_fifo_enable, &client_data->device);
	if (err) {
		PERR("faliled");
		return -EIO;
	}
	client_data->acc_fifo_enable = acc_fifo_enable;
	return count;
}

/**
 * fifo_config_show - sysfs read callback which reads and provide the
 * sensor fifo configuration.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t fifo_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 fifo_config;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma4_get_fifo_config(&fifo_config, &client_data->device);
	if (err) {
		PERR("read error");
		return err;
	}
	return snprintf(buf, 96, "fifo_config 0x%x\n", fifo_config);
}

/**
 * fifo_config_store - sysfs write callback which stores the fifo
 * configuration of sensor from the user.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t fifo_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int data[2] = {0};
	u8 fifo_config = 0;
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = sscanf(buf, "%11X %11d", &data[0], &data[1]);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	fifo_config = (u8)data[0];
	PDEBUG("fifo config 0x%x enable =%d", fifo_config, data[1]);
	err = bma4_set_fifo_config(fifo_config, data[1], &client_data->device);
	if (err) {
		PERR("write faliled");
		return err;
	}
	return count;
}

/**
 * load_config_stream_show - sysfs read callback which gives the loaded
 * config stream in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t load_config_stream_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 48, "config stream %s\n",
		client_data->config_stream_name);
}

/**
 * bma4xy_init_after_config_stream_load - Initializes and remaps the axis
 * of the sensor.
 *
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
int bma4xy_init_after_config_stream_load(
	struct bma4xy_client_data *client_data)
{
	int err = 0;
	u8 int_enable = 0x0a;
	u8 latch_enable = 0x01;
	u8 int1_map = 0xff;

	err = bma4_write_regs(
	BMA4_INT_MAP_1_ADDR, &int1_map, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
	err += bma4_write_regs(
	BMA4_INT1_IO_CTRL_ADDR, &int_enable, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
	err += bma4_write_regs(
	BMA4_INTR_LATCH_ADDR, &latch_enable, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
	if (err)
		PERR("map and enable interrupt1 failed err=%d", err);

	return err;
}

int bma4xy_init_data_rdy(
	struct bma4xy_client_data *client_data, int enable)
{
	int err = 0;
	u8 data_rdy_enable = 0x04;
	u8 int_enable = 0x0a;
	u8 latch_enable = 0x01;

	u8 data_rdy_disable = 0x00;
	u8 int_disable = 0x00;
	u8 latch_disable = 0x00;

	if(enable)
	{
		err = bma4_write_regs(
		BMA4_INT_MAP_DATA_ADDR, &data_rdy_enable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
		err += bma4_write_regs(
		BMA4_INT1_IO_CTRL_ADDR, &int_enable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
		err += bma4_write_regs(
		BMA4_INTR_LATCH_ADDR, &latch_enable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);

		if (err)
			PERR("map and enable data rdy interrupt1 failed err=%d", err);
	}
	else
	{

		err = bma4_write_regs(
		BMA4_INT_MAP_DATA_ADDR, &data_rdy_disable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
		err += bma4_write_regs(
		BMA4_INT1_IO_CTRL_ADDR, &int_disable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
		err += bma4_write_regs(
		BMA4_INTR_LATCH_ADDR, &latch_disable, 1, &client_data->device);
		bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);

		if (err)
			PERR("map and enable data rdy interrupt1 failed err=%d", err);

	}

	return err;
}

/**
 * bma4xy_init_fifo_config - Initializes the fifo configuration of the sensor.
 *
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
int bma4xy_init_fifo_config(
	struct bma4xy_client_data *client_data)
{
	int err = 0;

	err = bma4_set_fifo_config(
		BMA4_FIFO_HEADER, BMA4_ENABLE, &client_data->device);
	if (err)
		PERR("enable fifo header failed err=%d", err);
	err = bma4_set_fifo_config(
		BMA4_FIFO_TIME, BMA4_ENABLE, &client_data->device);
	if (err)
		PERR("enable fifo timer failed err=%d", err);
	return err;
}

/**
 * bma4xy_update_config_stream - Loads the config stream in the sensor.
 *
 * @client_data: Instance of client data structure.
 * @option: Option to choose different tbin images.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
int bma4xy_update_config_stream(
	struct bma4xy_client_data *client_data, int choose)
{
	char *name = "bma456h_config_stream";
	int err = 0;
	u8 crc_check = 0;

	PDEBUG("choose the config_stream %s", name);
	if (choose == 3) {
		err = bma456h_write_config_file(&client_data->device);
		if (!err)
			name = "bma456h_config_stream";
		client_data->config_stream_name = name;

		if (err) {
			PERR("download config stream failed %d", err);
			return err;
		}

		bma4xy_i2c_delay_us(MS_TO_US(200), &client_data->device.intf_ptr);
		err = bma4_read_regs(BMA4_INTERNAL_STAT,
				&crc_check, 1, &client_data->device);
		if (err)
			PERR("reading CRC failer");
		if (crc_check != BMA4_ASIC_INITIALIZED)
			PERR("crc check error %x", crc_check);
	}

	return err;
}

/**
 * load_config_stream_store - sysfs write callback which loads the
 * config stream in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t load_config_stream_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long choose = 0;
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &choose);
	if (err)
		return err;

	err = bma4xy_update_config_stream(client_data, choose);
	if (err) {
		PERR("config_stream load error");
		return count;
	}
	err = bma4xy_init_after_config_stream_load(client_data);
	if (err) {
		PERR("bma4xy_init_after_config_stream_load error");
		return count;
	}
	return count;
}

/**
 * fifo_watermark_show - sysfs read callback which reads and
 * provide the watermark level of fifo in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t fifo_watermark_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u16 data;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma4_get_fifo_wm(&data, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 48, "%d\n", data);
}

/**
 * fifo_watermark_store - sysfs write callback which stores the
 * watermark level of fifo in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t fifo_watermark_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	unsigned char fifo_watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	fifo_watermark = (unsigned char)data;
	err = bma4_set_fifo_wm(fifo_watermark, &client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * fifo_data_frame_show - sysfs read callback which reads the fifo data
 * according to the fifo length.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t fifo_data_frame_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;
	u16 fifo_bytecount = 0;

	if (!client_data->mag_fifo_enable && !client_data->acc_fifo_enable) {
		PERR("no sensor selected for fifo\n");
		return -EINVAL;
	}
	err = bma4_get_fifo_length(&fifo_bytecount, &client_data->device);
	if (err < 0) {
		PERR("read fifo_len err=%d", err);
		return -EINVAL;
	}
	if (fifo_bytecount == 0)
		return 0;
	err =  bma4_read_regs(BMA4_FIFO_DATA_ADDR,
		buf, fifo_bytecount, &client_data->device);
	if (err) {
		PERR("read fifo leght err");
		return -EINVAL;
	}
	return fifo_bytecount;
}

/**
 * reg_sel_show - sysfs read callback which provides the register
 * address selected.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t reg_sel_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	return snprintf(buf, 64, "reg=0X%02X, len=%d\n",
		client_data->reg_sel, client_data->reg_len);
}

/**
 * reg_sel_store - sysfs write callback which stores the register
 * address to be selected.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t reg_sel_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	err = sscanf(buf, "%11X %11d",
		&client_data->reg_sel, &client_data->reg_len);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	return count;
}

/**
 * reg_val_show - sysfs read callback which shows the register
 * value which is read from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t reg_val_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	err = bma4_read_regs(client_data->reg_sel, reg_data,
		client_data->reg_len, &client_data->device);
	if (err < 0) {
		PERR("Reg op failed");
		return err;
	}
	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += snprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

/**
 * reg_val_store - sysfs write callback which stores the register
 * value which is to be written in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t reg_val_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 reg_data[128];
	int i, j, status, digit;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	status = 0;
	for (i = j = 0; i < count && j < client_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		PDEBUG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->reg_len)
		j = client_data->reg_len;
	else if (j < client_data->reg_len) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	PDEBUG("Reg data read as");
	for (i = 0; i < j; ++i)
		PDEBUG("%d", reg_data[i]);
	err = bma4_read_regs(client_data->reg_sel, reg_data,
			client_data->reg_len, &client_data->device);
	if (err < 0) {
		PERR("Reg op failed");
		return err;
	}
	return count;
}

/**
 * driver_version_show - sysfs read callback which provides the driver
 * version.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t driver_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 128,
		"Driver version: %s\n", DRIVER_VERSION);
}

/**
 * config_file_version_show - sysfs read callback which gives the
 * configuration id of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t config_file_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u16 min_version = 0;
	u16 maj_version = 0;

	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma456h_get_version_config(&maj_version,
			&min_version, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 128,
		"Config_stream version :%d.%d\n",
		 maj_version, min_version);
}

/**
 * avail_sensor_show - sysfs read callback which provides the sensor-id
 * to the user.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t avail_sensor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 avail_sensor = 4561;/* bma456h */

	return snprintf(buf, 32, "%d\n", avail_sensor);
}

/**
 * step_counter_val_show - sysfs read callback which reads and provide
 * output value of step-counter sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t step_counter_val_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u32 step_counter_val = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = bma456h_step_counter_output(
	&step_counter_val, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	PDEBUG("val %u", step_counter_val);
	if (client_data->err_int_trigger_num == 0) {
		client_data->step_counter_val = step_counter_val;
		PDEBUG("report val %u", client_data->step_counter_val);
		err = snprintf(buf, 96, "%u\n", client_data->step_counter_val);
		client_data->step_counter_temp = client_data->step_counter_val;
	} else {
		PDEBUG("after err report val %u",
			client_data->step_counter_val + step_counter_val);
		err = snprintf(buf, 96, "%u\n",
			client_data->step_counter_val + step_counter_val);
		client_data->step_counter_temp =
			client_data->step_counter_val + step_counter_val;
	}
	return err;
}

/**
 * step_counter_watermark_show - sysfs read callback which reads and
 * provide the watermark level of step-counter sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t step_counter_watermark_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u16 watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);


	err = bma456h_step_counter_get_watermark(
	&watermark, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 32, "%d\n", watermark);
}


/**
 * step_counter_watermark_store - sysfs write callback which stores the
 * watermark level of step-counter in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t step_counter_watermark_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long step_watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &step_watermark);
	if (err)
		return err;
	PDEBUG("watermark step_counter %ld", step_watermark);

	err = bma456h_step_counter_set_watermark(
	step_watermark, &client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * step_counter_parameter_show - sysfs read callback which reads the
 * parameters of the step-counter in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t step_counter_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_stepcounter_settings setting;

	err = bma456h_stepcounter_get_parameter(&setting, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}

	return snprintf(buf, PAGE_SIZE,
	"parameter1 =0x%x parameter2= 0x%x\n"
	"parameter3 = 0x%x parameter4 = 0x%x\n"
	"parameter5 = 0x%x parameter6 = 0x%x\n"
	"parameter7 = 0x%x parameter8 = 0x%x\n"
	"parameter9 = 0x%x parameter10 = 0x%x\n"
	"parameter11 = 0x%x parameter12 = 0x%x\n"
	"parameter13 = 0x%x parameter14 = 0x%x\n"
	"parameter15 = 0x%x parameter16 = 0x%x\n"
	"parameter17 = 0x%x parameter18 = 0x%x\n"
	"parameter19 = 0x%x parameter20 = 0x%x\n"
	"parameter21 = 0x%x parameter22 = 0x%x\n"
	"parameter23 = 0x%x parameter24 = 0x%x\n"
	"parameter25 = 0x%x\n",
	setting.param1, setting.param2, setting.param3, setting.param4,
	setting.param5, setting.param6, setting.param7, setting.param8,
	setting.param9, setting.param10, setting.param11, setting.param12,
	setting.param13, setting.param14, setting.param15, setting.param16,
	setting.param17, setting.param18, setting.param19, setting.param20,
	setting.param21, setting.param22, setting.param23, setting.param24,
	setting.param25);
}

/**
 * step_counter_parameter_store - sysfs write callback which stores the
 * step-counter
 * parameters settings in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t step_counter_parameter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	unsigned int data[25] = {0};
	struct bma456h_stepcounter_settings setting;

	err = sscanf(buf,
	"%11x %11x %11x %11x %11x %11x %11x %11x\n"
	"%11x %11x %11x %11x %11x %11x %11x %11x\n"
	"%11x %11x %11x %11x %11x %11x %11x %11x\n"
	"%11x\n",
	&data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6],
	&data[7], &data[8], &data[9], &data[10], &data[11], &data[12],
	&data[13],
	&data[14], &data[15], &data[16], &data[17], &data[18], &data[19],
	&data[20],
	&data[21], &data[22], &data[23], &data[24]);
	if (err != 25) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	setting.param1 = (u16)data[0];
	setting.param2 = (u16)data[1];
	setting.param3 = (u16)data[2];
	setting.param4 = (u16)data[3];
	setting.param5 = (u16)data[4];
	setting.param6 = (u16)data[5];
	setting.param7 = (u16)data[6];
	setting.param8 = (u16)data[7];
	setting.param9 = (u16)data[8];
	setting.param10 = (u16)data[9];
	setting.param11 = (u16)data[10];
	setting.param12 = (u16)data[11];
	setting.param13 = (u16)data[12];
	setting.param14 = (u16)data[13];
	setting.param15 = (u16)data[14];
	setting.param16 = (u16)data[15];
	setting.param17 = (u16)data[16];
	setting.param18 = (u16)data[17];
	setting.param19 = (u16)data[18];
	setting.param20 = (u16)data[19];
	setting.param21 = (u16)data[20];
	setting.param22 = (u16)data[21];
	setting.param23 = (u16)data[22];
	setting.param24 = (u16)data[23];
	setting.param25 = (u16)data[24];

	err = bma456h_stepcounter_set_parameter(&setting, &client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * step_counter_reset_store - sysfs write callback which resets the
 * step-counter value in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t step_counter_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long reset_counter;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &reset_counter);
	if (err)
		return err;
	PDEBUG("reset_counter %ld", reset_counter);

	err = bma456h_reset_step_counter(&client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	client_data->step_counter_val = 0;
	client_data->step_counter_temp = 0;
	return count;
}

/**
 * anymotion_config_show - sysfs read callback which reads the
 * any-motion configuration from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t anymotion_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;

	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE,
	"duration =0x%x threshold= 0x%x\n",
	any_motion.duration, any_motion.threshold);
}

/**
 * anymotion_config_store - sysfs write callback which writes the
 * any-motion configuration in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t anymotion_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;
	unsigned int data[2] = {0};

	err = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (err != 3) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	memset(&any_motion, 0, sizeof(any_motion));
	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	err = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	memset(&any_motion, 0, sizeof(any_motion));
	any_motion.duration = (u16)data[0];
	any_motion.threshold = (u16)data[1];

	err = bma456h_set_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * any_motion_enable_axis_show - sysfs read callback that tells which
 * axis(x,y and z) of any-motion sensor is enabled.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t any_motion_enable_axis_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, PAGE_SIZE,	"anymotion_enable_axis = %d\n",
		client_data->any_motion_axis);
}

/**
 * any_motion_enable_axis_store - sysfs write callback which enable or
 * disable the x,y and z axis of any-motion sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t any_motion_enable_axis_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long data;
	u8 enable_axis;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	PDEBUG("enable_axis %ld", data);
	enable_axis = (u8)data;
	client_data->any_motion_axis = enable_axis;

	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * no_motion_enable_axis_show - sysfs read callback that tells which
 * axis is enabled in the no_motion sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t no_motion_enable_axis_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, PAGE_SIZE,
	"nomotion_enable_axis = %d\n",
	client_data->no_motion_axis);
}

/**
 * no_motion_enable_axis_store - sysfs write callback which enable or
 * disable the x,y and z axis of no-motion sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t no_motion_enable_axis_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long data;
	u8 enable_axis;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	PDEBUG("enable_axis %ld", data);
	enable_axis = (u8)data;
	client_data->no_motion_axis = enable_axis;

	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 * nomotion_config_show - sysfs read callback which reads the
 * no-motion configuration from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t nomotion_config_show(struct device *dev,
	struct device_attribute *atte, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config no_motion;

	err = bma456h_get_no_mot_config(&no_motion, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE,
	"duration =0x%x threshold= 0x%x\n",
	no_motion.duration, no_motion.threshold);
}

/**
 * nomotion_config_store - sysfs write callback which writes the
 * no-motion configuration in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t nomotion_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[2] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config no_motion;

	err = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	memset(&no_motion, 0, sizeof(no_motion));

	err = bma456h_get_no_mot_config(&no_motion, &client_data->device);

	no_motion.duration = (u16)data[0];
	no_motion.threshold = (u16)data[1];

	err = bma456h_set_no_mot_config(&no_motion, &client_data->device);
	if (err) {
		PERR("write failed");
		return err;
	}
	return count;
}

/**
 *  bma456h_config_feature - Function to configure the features in the sensor.
 *  @client_data : Pointer to client data structure.
 */
int bma456h_config_feature(struct bma4xy_client_data *client_data)
{
	int err = 0;
	u16 feature = 0;
	struct bma456h_any_no_mot_config any_no_mot;

	/* For one shot mode handling */
	any_no_mot.duration = 0xFA;
	if (client_data->anymotion_en == BMA4_ENABLE)
		err = bma456h_feature_enable(client_data->any_motion_axis,
				BMA4_ENABLE, &client_data->device);
	else if (client_data->nomotion_en == BMA4_ENABLE)
		err = bma456h_feature_enable(client_data->no_motion_axis,
				BMA4_ENABLE, &client_data->device);

	bma4xy_i2c_delay_us(MS_TO_US(2), &client_data->device.intf_ptr);
	if (client_data->stepcounter_en == BMA4_ENABLE)
		feature = feature | BMA456H_STEP_COUNTER_EN;
	if (client_data->activity_en == BMA4_ENABLE)
		feature = feature | BMA456H_STEP_ACTIVITY_EN;
	if (client_data->single_tap_en == BMA4_ENABLE)
		feature = feature | BMA456H_SINGLE_TAP_EN;
	if (client_data->double_tap_en == BMA4_ENABLE)
		feature = feature | BMA456H_DOUBLE_TAP_EN;
	if (client_data->triple_tap_en == BMA4_ENABLE)
		feature = feature | BMA456H_TRIPLE_TAP_EN;
	err = bma456h_feature_enable(feature, BMA4_ENABLE, &client_data->device);
	if (err)
		PERR("set feature err");

	return err;

}

/**
 * bma4xy_reinit_after_error_interrupt - Re-initialize and configure the sensor
 * after an error interrupt.
 *
 * @client_data: Instance of the client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
int bma4xy_reinit_after_error_interrupt(struct bma4xy_client_data *client_data)
{
	int err = 0;
	u8 data = 0;
	u8 crc_check = 0;

	client_data->err_int_trigger_num += 1;
	client_data->step_counter_val = client_data->step_counter_temp;
	/*reset the bma4xy*/
	err = bma4_set_command_register(0xB6, &client_data->device);
	if (!err)
		PDEBUG("reset chip");
	/*reinit the fifo config*/
	err = bma4xy_init_fifo_config(client_data);
	if (err)
		PERR("fifo init failed");
	else
		PDEBUG("fifo init success");
	/*reload the config_stream*/
	err = bma4_write_config_file(&client_data->device);
	if (err)
		PERR("download config stream failer");
	else
		PDEBUG("download config stream success");
	bma4xy_i2c_delay_us(MS_TO_US(200), &client_data->device.intf_ptr);
	err = bma4_read_regs(BMA4_INTERNAL_STAT,
	&crc_check, 1, &client_data->device);
	if (err)
		PERR("reading CRC failer");
	else
		PDEBUG("Reading CRC success");
	if (crc_check != BMA4_ASIC_INITIALIZED)
		PERR("crc check error %x", crc_check);
	/*reconfig interrupt and remap*/
	err = bma4xy_init_after_config_stream_load(client_data);
	if (err)
		PERR("reconfig interrupt and remap error");
	else
		PDEBUG("reconfig interrupt and remap success");

	/*reinit the feature*/

	err = bma456h_config_feature(client_data);
	if (err)
		PERR("reinit the virtual sensor error");
	else
		PDEBUG("reinit success");
	/*reinit acc*/
	if (client_data->acc_odr != 0) {
		data = client_data->acc_odr;
		if (data == 4)
			data = 0x74;
		else
			data |= 0xA0;

		err = bma4_read_regs(0x40, &data, 1, &client_data->device);
		if (err)
			PERR("set acc_odr faliled");
		bma4xy_i2c_delay_us(MS_TO_US(2), &client_data->device.intf_ptr);
	}
	if (client_data->pw.acc_pm == 1)
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
	if (err)
		PERR("set acc_op_mode failed");
	else
		PDEBUG("set acc_op_mode success");
	bma4xy_i2c_delay_us(MS_TO_US(2), &client_data->device.intf_ptr);
	err = bma4_set_fifo_config(BMA4_FIFO_ACCEL,
		client_data->acc_fifo_enable, &client_data->device);
	if (err)
		PERR("set acc_fifo_enable faliled");
	else
		PDEBUG("acc_fifo_enable success");
	bma4xy_i2c_delay_us(MS_TO_US(5), &client_data->device.intf_ptr);
	return 0;
}

/**
 * err_int_show - sysfs read callback which gives the error interrupt
 * if there is any error in initialization.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t err_int_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 128, "please check sensor normal working\n");
}

/**
 * err_int_store - sysfs write callback which reinitializes the sensor if
 * any error interrupt occurs.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t err_int_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	err = bma4xy_reinit_after_error_interrupt(client_data);
	if (err)
		return err;
	return count;
}

static ssize_t slope_duration_show(struct device *dev,
	struct device_attribute *atte, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;

	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion config read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE,	"%d\n", any_motion.duration);
}

static ssize_t slope_duration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;
	unsigned long duration;

	err = kstrtoul(buf, 10, &duration);
	if (err) {
		PERR("kstrtoul error (%d)\n", err);
		return err;
	}

	if (duration > 8192) {
		PERR("duration over-range (0-8191)\n");
		return -EINVAL;
	}

	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion config read failed (err = %d)\n", err);
		return -EINVAL;
	}

	any_motion.duration = (u16)duration;

	err = bma456h_set_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion write duration failed (err = %d)\n", err);
		return err;
	}

	return count;
}

static ssize_t slope_threshold_show(struct device *dev,
	struct device_attribute *atte, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;

	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion config read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE,	"%d\n", any_motion.threshold);
}

static ssize_t slope_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456h_any_no_mot_config any_motion;
	unsigned long threshold;

	err = kstrtoul(buf, 10, &threshold);
	if (err) {
		PERR("kstrtoul error (%d)\n", err);
		return err;
	}

	if (threshold > 2048) {
		PERR("threshold over-range (0-2047)\n");
		return -EINVAL;
	}

	err = bma456h_get_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion config read failed (err = %d)\n", err);
		return -EINVAL;
	}

	any_motion.threshold = (u16)threshold;

	err = bma456h_set_any_mot_config(&any_motion, &client_data->device);
	if (err) {
		PERR("any motion write threshold failed (err = %d)\n", err);
		return err;
	}

	return count;
}

static ssize_t en_sig_motion_show(struct device *dev,
	struct device_attribute *atte, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, PAGE_SIZE,	"%d\n", client_data->anymotion_en);
}

static int bma4xy_set_en_sig_motion(struct bma4xy_client_data *client_data,
					int en)
{
	int err = 0;
	int enable = (en >= BMA4_ENABLE) ? BMA4_ENABLE : BMA4_DISABLE;

	if (enable == BMA4_ENABLE) {
		// Enable the accel first before proceeding to enable the feature.
		err = enable_store_impl(client_data, BMA4_ENABLE);
		if (err) {
			PERR("en_sig_motion: Unable to enable the accel (err=%d)\n", err);
			return err;
		}
	}

	err = bma456h_feature_enable(client_data->any_motion_axis,
			enable, &client_data->device);
	if (err) {
		PERR("en_sig_motion: Feature toogle failed (en=%d,err=%d)\n", en, err);
		goto err_feature_enable;
	}

	err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_ANY_MOT_INT,
			enable, &client_data->device);
	if (err) {
		PERR("en_sig_motion: Interrupt mapping failed (en=%d,err=%d)\n", en, err);
		goto interrupt_map_err;
	}

	if (enable == BMA4_DISABLE) {
		// Remove the earlier enable te once the feature's been disabled.
		enable_store_impl(client_data, BMA4_DISABLE);
	}

	client_data->anymotion_en = en;
	return 0;

interrupt_map_err:
	bma456h_feature_enable(client_data->any_motion_axis,
			!enable, &client_data->device);
err_feature_enable:
	if (enable == BMA4_ENABLE)
		enable_store_impl(client_data, BMA4_DISABLE);
	return err;
}

static ssize_t en_sig_motion_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	u8 enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtou8(buf, 10, &enable);
	if (err) {
		PERR("kstrtou8 failed (err=%d)\n", err);
		return err;
	}

	/* An enable setting of 2 is only used as a workaround to let this
	 * feature be enabled all the time.
	 * Note that using an enable value of 2 will result in input events to
	 * never be sent.
	 */
	if ((enable == 0) || (enable == 1) || (enable == 2))
		bma4xy_set_en_sig_motion(client_data, enable);

	return count;
}

static ssize_t bandwidth_show(struct device *dev,
	struct device_attribute *atte, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("accel config read failed");
		return err;
	}
	return snprintf(buf, 16, "%d\n", acc_config.bandwidth);
}

static ssize_t bandwidth_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;
	u8 bandwidth;

	err = kstrtou8(buf, 10, &bandwidth);
	if (err) {
		PERR("kstrtou8 failed (err=%d)\n", err);
		return err;
	}

	err = bma4_get_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("accel config read failed");
		return err;
	}

	acc_config.bandwidth = bandwidth;
	err = bma4_set_accel_config(&acc_config, &client_data->device);
	if (err) {
		PERR("setting the accel bandwidth config failed (err=%d)\n", err);
		return err;
	}

	return count;
}

static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(scale);
static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(delay);
static DEVICE_ATTR_RO(value);
static DEVICE_ATTR_RW(range);
static DEVICE_ATTR_RW(acc_odr);
static DEVICE_ATTR_RW(acc_fifo_enable);
static DEVICE_ATTR_WO(fifo_flush);
static DEVICE_ATTR_RW(selftest);
static DEVICE_ATTR_RO(avail_sensor);
static DEVICE_ATTR_RO(fifo_length);
static DEVICE_ATTR_RW(fifo_watermark);
static DEVICE_ATTR_RW(fifo_config);
static DEVICE_ATTR_RW(load_config_stream);
static DEVICE_ATTR_RW(reg_sel);
static DEVICE_ATTR_RW(reg_val);
static DEVICE_ATTR_RO(driver_version);
static DEVICE_ATTR_RO(config_file_version);
static DEVICE_ATTR_RO(fifo_data_frame);
static DEVICE_ATTR_RW(foc);
static DEVICE_ATTR_RW(axis_remap);
static DEVICE_ATTR_RW(config_function);
static DEVICE_ATTR_RO(step_counter_val);
static DEVICE_ATTR_RW(step_counter_watermark);
static DEVICE_ATTR_RW(step_counter_parameter);
static DEVICE_ATTR_WO(step_counter_reset);
static DEVICE_ATTR_RW(no_motion_enable_axis);
static DEVICE_ATTR_RW(nomotion_config);
static DEVICE_ATTR_RW(anymotion_config);
static DEVICE_ATTR_RW(any_motion_enable_axis);
static DEVICE_ATTR_RW(err_int);
static DEVICE_ATTR_RW(slope_duration);
static DEVICE_ATTR_RW(slope_threshold);
static DEVICE_ATTR_RW(en_sig_motion);
static DEVICE_ATTR_RW(bandwidth);

static struct attribute *bma4xy_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_scale.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_acc_odr.attr,
	&dev_attr_acc_fifo_enable.attr,
	&dev_attr_selftest.attr,
	&dev_attr_avail_sensor.attr,
	&dev_attr_foc.attr,
	&dev_attr_axis_remap.attr,
	&dev_attr_fifo_length.attr,
	&dev_attr_fifo_watermark.attr,
	&dev_attr_fifo_flush.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_load_config_stream.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_config_file_version.attr,
	&dev_attr_reg_sel.attr,
	&dev_attr_reg_val.attr,
	&dev_attr_config_function.attr,
	&dev_attr_fifo_config.attr,
	&dev_attr_step_counter_val.attr,
	&dev_attr_step_counter_watermark.attr,
	&dev_attr_step_counter_parameter.attr,
	&dev_attr_step_counter_reset.attr,
	&dev_attr_nomotion_config.attr,
	&dev_attr_no_motion_enable_axis.attr,
	&dev_attr_anymotion_config.attr,
	&dev_attr_any_motion_enable_axis.attr,
	&dev_attr_err_int.attr,
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_en_sig_motion.attr,
	&dev_attr_bandwidth.attr,
	NULL
};

static struct attribute_group bma4xy_attribute_group = {
	.attrs = bma4xy_attributes
};

#ifdef CONFIG_SIG_MOTION
static struct attribute *bma4xy_sig_motion_attributes[] = {
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_en_sig_motion.attr,
	&dev_attr_bandwidth.attr,
	NULL
};
static struct attribute_group bma4xy_sig_motion_attribute_group = {
	.attrs = bma4xy_sig_motion_attributes
};
#endif

#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)
/**
 * bma4xy_fifowm_int_handle - Handles fifo water mark interrupt.
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0			- OK
 * * Negative value	- Failed
 */
static void bma4xy_fifowm_int_handle(struct bma4xy_client_data *client_data)
{
	int err = 0;
	u16 fifo_bytecount = 0;

	err = bma4_get_fifo_length(&fifo_bytecount, &client_data->device);
	if (fifo_bytecount == 0 || err) {
		PERR("read fifo length zero");
		return;
	}
	if (fifo_bytecount > FIFO_DATA_BUFSIZE_BMA4XY) {
		PERR("read fifo length bigger than 1024 length =%d",
		fifo_bytecount);
		return;
	}
	memset(fifo_data, 0, 1024);
	err = bma4_read_regs(BMA4_FIFO_DATA_ADDR, fifo_data,
			fifo_bytecount + 4, &client_data->device);

	if (err)
		PERR("read fifo err");
	err = bma4xy_parse_fifo_data(client_data, fifo_data,
	client_data->fifo_bytecount + 4);
	if (err)
		PERR("analyze handle failed:%d\n", err);
}

/**
 * bma4xy_uc_function_handle - Handles feature interrupts
 *
 * @client_data : Instance of client data.
 * @status : Interrupt register value
 */
static void bma4xy_uc_function_handle(
	struct bma4xy_client_data *client_data, u8 status)
{
	int err = 0;

	if (status & ERROR_INT_OUT) {
		PDEBUG("into reinit_error_interrupt from uc_func");
		err = bma4xy_reinit_after_error_interrupt(client_data);
		if (err)
			PERR("reinit failed");
	}
	input_event(client_data->feat_input, EV_MSC, REL_FEAT_STATUS,
		(u32)(status));
	PDEBUG("%x", (u32)(status));
	input_sync(client_data->feat_input);
}

static void bma4xy_work_func(struct work_struct *work)
{

    struct bma4xy_client_data *client_data = container_of((struct delayed_work *)work,
			struct bma4xy_client_data, work);
	struct bma4_accel data;
	int err = 0;

	unsigned long delay = msecs_to_jiffies(atomic_read(&client_data->delay));
	err = bma4_read_accel_xyz(&data, &client_data->device);
	if (err < 0)
	{
		pr_err("cannot read acc value\n");
	}
	else
	{
		input_report_abs(client_data->acc_input, ABS_X, data.x);
		input_report_abs(client_data->acc_input, ABS_Y, data.y);
		input_report_abs(client_data->acc_input, ABS_Z, data.z);
		input_sync(client_data->acc_input);
	}

	schedule_delayed_work(&client_data->work, delay);
}

/**
 *  bma4xy_irq_work_func - Bottom half handler for feature interrupts.
 *  @work : Work data for the workqueue handler.
 */
static void bma4xy_irq_work_func(struct work_struct *work)
{
	struct bma4xy_client_data *client_data = container_of(work,
	struct bma4xy_client_data, irq_work);
	unsigned char int_status[2] = {0, 0};
	struct bma456h_out_state out_state;
	int err = 0;

	/*read the interrut status two register*/
	err = bma4_read_regs(BMA4_INT_STAT_0_ADDR, int_status, 2,
		&client_data->device);
	if (err)
		return;
	PDEBUG("int_status0 = 0x%x int_status1 =0x%x",
		int_status[0], int_status[1]);

	if ((int_status[1] & ACC_DRDY_OUT) == ACC_DRDY_OUT)
	{
		PDEBUG("Data rdy interrupt occured");
	}

	if ((int_status[0] & STEP_DET_OUT) == STEP_DET_OUT)
		PDEBUG("step Detect interrupt occured");
	if ((int_status[0] & ACTIVITY_OUT) == ACTIVITY_OUT) {
		bma456h_output_state(&out_state, &client_data->device);
		PDEBUG("step Activity type 0x%x detected", out_state.activity_type);
	}
	if ((int_status[0] & ERROR_INT_OUT) == ERROR_INT_OUT)
		PDEBUG("error interrupt occured");
	if ((int_status[0] & ANY_MOTION_OUT) == ANY_MOTION_OUT) {
		PDEBUG("any motion interrupt occured");
		if (client_data->anymotion_en == 1) {
			/* Close the sig sensor.
			 * It will be open again if APP wants.
			 */
			bma4xy_set_en_sig_motion(client_data, 0);

			input_report_rel(client_data->dev_interrupt,
				SLOPE_INTERRUPT, 1);
			input_sync(client_data->dev_interrupt);
		}
	}
	if ((int_status[0] & NO_MOTION_OUT) == NO_MOTION_OUT) {
		PDEBUG("no motion interrupt occured");
		client_data->nomotion_en = 0;

		err = bma456h_feature_enable(client_data->no_motion_axis,
			BMA4_DISABLE, &client_data->device);
		if (err)
			PDEBUG("no motion feature enabled failed");

		err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_NO_MOT_INT,
			BMA4_DISABLE, &client_data->device);
		if (err)
			PDEBUG("no motion interrupt dis mapping failed");
	}
	if ((int_status[0] & TAP_OUT) == TAP_OUT) {
		bma456h_output_state(&out_state, &client_data->device);
		if (out_state.single_tap)
			PDEBUG("single tap detected ");
		if (out_state.double_tap)
			PDEBUG("double tap detected ");
		if (out_state.triple_tap)
			PDEBUG("triple tap detected ");
	}

	if (int_status[1] & FIFOFULL_OUT) {
		PDEBUG("fifo full interrupt occured");
		bma4xy_fifowm_int_handle(client_data);
		client_data->fifo_full_int_en = 0;
		err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA4_FIFO_FULL_INT,
			BMA4_DISABLE,
			&client_data->device);
		if (err)
			PDEBUG("fifo full interrupt dis mapping failed");
	}
	if (int_status[1] & FIFOWATERMARK_OUT) {
		PDEBUG("fifo watermark interrupt occured");
		bma4xy_fifowm_int_handle(client_data);
		client_data->fifo_wm_int_en = 0;

		err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA4_FIFO_WM_INT,
			BMA4_DISABLE,
			&client_data->device);
		if (err)
			PDEBUG("fifo watermark interrupt dis mapping failed");
	}
	if (int_status[0])
		bma4xy_uc_function_handle(client_data, (u8)int_status[0]);
}

/**
 * bma4xy_irq_handle - IRQ handler function.
 * @irq : Number of irq line.
 * @handle : Instance of client data.
 *
 * Return : Status of IRQ function.
 */
static irqreturn_t bma4xy_irq_handle(int irq, void *handle)
{
	struct bma4xy_client_data *client_data = handle;

	schedule_work(&client_data->irq_work);
	return IRQ_HANDLED;
}

/**
 * bma4xy_request_irq - Allocates interrupt resources and enables the
 * interrupt line and IRQ handling.
 *
 * @client_data: Instance of the client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bma4xy_request_irq(struct bma4xy_client_data *client_data)
{
	int err = 0;

	PDEBUG("BMA4xy gpio interrupt :%d\n", client_data->IRQ);
	err = request_irq(client_data->IRQ, bma4xy_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, client_data);
	if (err < 0)
		return err;
	INIT_WORK(&client_data->irq_work, bma4xy_irq_work_func);
	return err;
}
#endif

/**
 * bma4xy_acc_input_init - Register the accelerometer input device in the
 * system.
 * @client_data : Instance of client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bma4xy_acc_input_init(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_MSC, REL_FEAT_STATUS);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->acc_input = dev;
	return 0;
}

/**
 * bma4xy_acc_input_destroy - Un-register the Accelerometer input device from
 * the system.
 *
 * @client_data :Instance of client data.
 */
static void bma4xy_acc_input_destroy(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev = client_data->acc_input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/**
 * bma4xy_uc_function_input_init - Register the accelerometer input device
 * in the system.
 * @client_data : Instance of client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bma4xy_uc_function_input_init(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;
	dev->name = SENSOR_NAME_FEAT;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_MSC, REL_FEAT_STATUS);
	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->feat_input = dev;
	return 0;
}

/**
 * bma4xy_uc_function_input_destroy - Un-register the feature input device
 * from the system.
 * @client_data : Instance of client data.
 */
static void bma4xy_uc_function_input_destroy(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev = client_data->acc_input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/**
 * bma4xy_probe - Does Sensor initialization
 * @dev: Device instance
 * @client_data : Instance of client data.
 */
int bma4xy_probe(struct bma4xy_client_data *client_data, struct device *dev)
{
	struct input_dev *dev_interrupt;
	int err = 0;

	PINFO("function entrance");
	bma4xy_i2c_delay_us(MS_TO_US(100), &client_data->device.intf_ptr);

	/* check chip id */
	err = bma4xy_check_chip_id(client_data);
	if (!err) {
		PINFO("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -ENODEV;
		goto exit_err_clean;
	}

	dev_set_drvdata(dev, client_data);
	if (client_data == NULL)
		goto exit_err_clean;
	client_data->dev = dev;
	client_data->device.delay_us = bma4xy_i2c_delay_us;
	client_data->selftest = 2;

	/*acc input device init */
	err = bma4xy_acc_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;
	/* sysfs node creation */
	err = sysfs_create_group(&client_data->acc_input->dev.kobj,
			&bma4xy_attribute_group);
	if (err < 0)
		goto exit_err_clean;
	err = bma4xy_uc_function_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	err = bma456h_init(&client_data->device);
	client_data->any_motion_axis = BMA456H_ANY_MOTION_ALL_AXIS_EN;
	client_data->no_motion_axis = BMA456H_NO_MOTION_ALL_AXIS_EN;

	if (err < 0)
		PERR("init failed %d", err);
	err = bma4_set_command_register(0xB6, &client_data->device);
	if (!err)
		PDEBUG("reset chip");

	#ifdef CONFIG_SIG_MOTION
	dev_interrupt = input_allocate_device();
	if (!dev_interrupt) {
		PERR("Unable to allocate input device for bma4xy");
		goto exit_err_clean;
	}
	dev_interrupt->name = "bma_interrupt";
	dev_interrupt->id.bustype = BUS_I2C;
	input_set_capability(dev_interrupt, EV_REL,
		SLOPE_INTERRUPT);
	input_set_drvdata(dev_interrupt, client_data);

	err = input_register_device(dev_interrupt);
	if (err < 0) {
		PERR("Failed to register input device for bma_interrupt");
		goto exit_err_free_device;
	}
	err = sysfs_create_group(&dev_interrupt->dev.kobj,
			&bma4xy_sig_motion_attribute_group);
	if (err < 0) {
		PERR("Failed to create sysfs group for bma_interrupt");
		goto exit_err_unregister_device;
	}
	client_data->dev_interrupt = dev_interrupt;
	#endif

	bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);

	/* Request irq and config */
	#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)
	err = bma4xy_request_irq(client_data);
	if (err < 0)
		PERR("Request irq failed");

	#endif

	INIT_DELAYED_WORK(&client_data->work, bma4xy_work_func);
	atomic_set(&client_data->delay, BMA456_MAX_DELAY);

	err = bma4xy_init_fifo_config(client_data);
	if (err)
		PERR("fifo init failed");

	#ifdef BMA4XY_LOAD_CONFIG_FILE_IN_INIT
	err = bma4xy_update_config_stream(client_data, 3);
	if (err)
		PERR("config_stream load error");

	err = bma4xy_init_after_config_stream_load(client_data);
	if (err)
		PERR("bma4xy_init_after_config_stream_load error");

	#endif

	#ifdef BMA4XY_ENABLE_DATA_RDY_INT
	bma4xy_init_data_rdy(client_data,1);
	#endif

	PINFO("sensor %s probed successfully", SENSOR_NAME);
	return 0;
#ifdef CONFIG_SIG_MOTION
exit_err_unregister_device:
	input_unregister_device(dev_interrupt);
exit_err_free_device:
	input_free_device(dev_interrupt);
#endif
exit_err_clean:
	if (err) {
		if (client_data != NULL)
			kfree(client_data);
		return err;
	}
	return err;
}

static int pre_suspend_encount;

/**
 * bma4xy_suspend - This function puts the driver and device to suspend mode.
 * @dev : Instance of the device.
 *
 * Return : Status of the suspend function.
 * * 0 - OK.
 * * Negative value : Error.
 */
int bma4xy_suspend(struct device *dev)
{
	struct bma4xy_client_data *client_data = dev_get_drvdata(dev);
	struct bma4_dev *bma4_dev = &client_data->device;
	int err = 0;
	int curr_encount = 0;

	pre_suspend_encount = bma4_dev->en_count;
	curr_encount = bma4_dev->en_count;
	if (curr_encount) {
		while (curr_encount > 0) {
			bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
			curr_encount = bma4_dev->en_count;
		}
		cancel_delayed_work_sync(&client_data->work);
	}

	PINFO("suspend function entrance");
	atomic_set(&client_data->in_suspend, 1);

	return err;
}
/* Lint -save -e19 */
EXPORT_SYMBOL(bma4xy_suspend);
/* Lint -restore */

/**
 * bma4xy_resume - This function is used to bring back device from suspend mode
 * @dev : Instance of the device.
 *
 * Return : Status of the suspend function.
 * * 0 - OK.
 * * Negative value : Error.
 */
int bma4xy_resume(struct device *dev)
{
	int err = 0;
	struct bma4xy_client_data *client_data = dev_get_drvdata(dev);
	struct bma4_dev *bma4_dev = &client_data->device;

	if (pre_suspend_encount) {
		while (bma4_dev->en_count < pre_suspend_encount)
			bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		schedule_delayed_work(&client_data->work,
			msecs_to_jiffies(atomic_read(&client_data->delay)));
	}
	pre_suspend_encount = 0;

	PINFO("resume function entrance");
	atomic_set(&client_data->in_suspend, 0);

	return err;
}
/* Lint -save -e19 */
EXPORT_SYMBOL(bma4xy_resume);
/* Lint -restore */

/**
 * bma4xy_remove - This function removes the driver from the device.
 * @dev : Instance of the device.
 *
 * Return : Status of the suspend function.
 * * 0 - OK.
 * * Negative value : Error.
 */
int bma4xy_remove(struct device *dev)
{
	int err = 0;
	struct bma4xy_client_data *client_data = dev_get_drvdata(dev);

	if (client_data != NULL) {
		bma4xy_i2c_delay_us(BMA4XY_I2C_WRITE_DELAY_TIME,
			&client_data->device.intf_ptr);
		sysfs_remove_group(&client_data->acc_input->dev.kobj,
			&bma4xy_attribute_group);
		bma4xy_acc_input_destroy(client_data);
		bma4xy_uc_function_input_destroy(client_data);
		kfree(client_data);
	}
	return err;
}
/* Lint -save -e19 */
EXPORT_SYMBOL(bma4xy_remove);
/* Lint -restore */
