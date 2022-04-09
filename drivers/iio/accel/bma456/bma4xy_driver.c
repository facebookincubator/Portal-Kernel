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
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

/*********************************************************************/
/* Own header files */
/*********************************************************************/
#include "bma4xy_driver.h"
#include "bs_log.h"
/*********************************************************************/
/* Local macro definitions */
/*********************************************************************/
#define DRIVER_VERSION "1.3.14"
#define BMA456_ACCEL_CHIP_ID 0x16
/*********************************************************************/
/* Global data */
/*********************************************************************/
#define MS_TO_US(msec)		UINT32_C((msec) * 1000)

enum bma4xy_accel_axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
};

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

/* Available sampling frequencies in IIO floating value format
 * val is the whole number part, val2 is the fractional part
 * 0.78, 1.5, 3.1, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600
 * The sampling frequency is used by the HAL to know how often
 * to poll for data.
 */
static const struct {
	int val;
	int val2;
} bma456_accel_samp_freq_table[] = {{0, 780000},
					{1, 500000},
					{3, 100000},
					{6, 250000},
					{12, 500000},
					{25, 0},
					{50, 0},
					{100, 0},
					{200, 0},
					{400, 0},
					{800, 0},
					{1600, 0} };


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
/*
static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data)
{

	//PINFO("chip_id=0x%x rev_id=0x%x\n", chip_id[1], chip_id[0]);
	return 0;
}
*/

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
        struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	err = client_data->device.bus_read(
			BMA4_CHIP_ID_ADDR, chip_id, 2, &client_data->device);
	if (err) {
		PERR("chip id read failed");
		return -EIO;
	}

	return snprintf(buf, 96, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[1], chip_id[0]);
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
static ssize_t acc_op_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char acc_op_mode;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	err = bma4_get_accel_enable(&acc_op_mode, &client_data->device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 96, "1 mean enable now is %d\n", acc_op_mode);
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
static ssize_t acc_op_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long op_mode;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	if (op_mode == 2 &&
	(client_data->stepdet_en == 0) &&
	(client_data->stepcounter_en == 0) &&
	(client_data->double_tap_en == 0) &&
	(client_data->anymotion_en == 0) &&
	(client_data->nomotion_en == 0) &&
	(client_data->single_tap_en == 0) &&
	(client_data->triple_tap_en == 0)) {
		err = bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
		PDEBUG("acc_op_mode %ld", op_mode);
	} else if (op_mode == 0) {
		err = bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
		PDEBUG("acc_op_mode %ld", op_mode);
	}
	if (err) {
		PERR("failed");
		return err;
	}
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
static ssize_t acc_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bma4_accel data;
	int err;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
static ssize_t acc_range_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
static ssize_t acc_range_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long acc_range;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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

	ret = bma456h_map_interrupt(BMA4_INTR1_MAP, map_int, enable,
												&client_data->device);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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

/**
 * bma4xy_init_fifo_config - Initializes the fifo configuration of the sensor.
 *
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
int bma4xy_init_fifo_config(struct bma4xy_client_data *client_data)
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
	int err = 0;
	u8 reg_data[128], i;
	int pos;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	err = bma4_read_regs(client_data->reg_sel, reg_data, client_data->reg_len, 					&client_data->device);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	err = bma4_read_regs(client_data->reg_sel, reg_data, client_data->reg_len,
			&client_data->device);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	err = bma456h_get_version_config(&maj_version, &min_version,
					&client_data->device);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
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
	if (client_data->pw.acc_pm == 0)
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
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	err = bma4xy_reinit_after_error_interrupt(client_data);
	if (err)
		return err;
	return count;
}

static IIO_DEVICE_ATTR(chip_id, S_IRUGO, chip_id_show, NULL, 0);
static IIO_DEVICE_ATTR(acc_op_mode, S_IRUGO|S_IWUSR, acc_op_mode_show,
		acc_op_mode_store, 0);
static IIO_DEVICE_ATTR(acc_value, S_IRUGO, acc_value_show, NULL, 0);
static IIO_DEVICE_ATTR(acc_range, S_IRUGO|S_IWUSR, acc_range_show,
		acc_range_store, 0);
static IIO_DEVICE_ATTR(acc_odr, S_IRUGO|S_IWUSR, acc_odr_show, acc_odr_store, 0);
static IIO_DEVICE_ATTR(acc_fifo_enable, S_IRUGO|S_IWUSR, acc_fifo_enable_show,
		acc_fifo_enable_store, 0);
static IIO_DEVICE_ATTR(fifo_flush, S_IWUSR, NULL, fifo_flush_store, 0);
static IIO_DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR, selftest_show,
		selftest_store, 0);
static IIO_DEVICE_ATTR(avail_sensor, S_IRUGO, avail_sensor_show, NULL, 0);
static IIO_DEVICE_ATTR(fifo_length, S_IRUGO, fifo_length_show, NULL, 0);
static IIO_DEVICE_ATTR(fifo_watermark, S_IRUGO|S_IWUSR, fifo_watermark_show,
		fifo_watermark_store, 0);
static IIO_DEVICE_ATTR(fifo_config, S_IRUGO|S_IWUSR, fifo_config_show,
		fifo_config_store, 0);
static IIO_DEVICE_ATTR(load_config_stream, S_IRUGO|S_IWUSR,
		load_config_stream_show, load_config_stream_store, 0);
static IIO_DEVICE_ATTR(reg_sel, S_IRUGO|S_IWUSR, reg_sel_show, reg_sel_store, 0);
static IIO_DEVICE_ATTR(reg_val, S_IRUGO|S_IWUSR, reg_val_show, reg_val_store, 0);
static IIO_DEVICE_ATTR(driver_version, S_IRUGO, driver_version_show, NULL, 0);
static IIO_DEVICE_ATTR(config_file_version, S_IRUGO,
		config_file_version_show, NULL, 0);
static IIO_DEVICE_ATTR(fifo_data_frame, S_IRUGO,
		fifo_data_frame_show, NULL, 0);
static IIO_DEVICE_ATTR(foc, S_IRUGO|S_IWUSR, foc_show, foc_store, 0);
static IIO_DEVICE_ATTR(axis_remap, S_IRUGO|S_IWUSR, axis_remap_show,
		axis_remap_store, 0);
static IIO_DEVICE_ATTR(config_function, S_IRUGO|S_IWUSR, config_function_show,
		config_function_store, 0);
static IIO_DEVICE_ATTR(step_counter_val, S_IRUGO,
		step_counter_val_show, NULL, 0);
static IIO_DEVICE_ATTR(step_counter_watermark, S_IRUGO|S_IWUSR,
		step_counter_watermark_show, step_counter_watermark_store, 0);
static IIO_DEVICE_ATTR(step_counter_parameter, S_IRUGO|S_IWUSR,
		step_counter_parameter_show, step_counter_parameter_store, 0);
static IIO_DEVICE_ATTR(step_counter_reset, S_IWUSR, NULL,
		step_counter_reset_store, 0);
static IIO_DEVICE_ATTR(no_motion_enable_axis, S_IRUGO|S_IWUSR,
		no_motion_enable_axis_show, no_motion_enable_axis_store, 0);
static IIO_DEVICE_ATTR(nomotion_config, S_IRUGO|S_IWUSR, nomotion_config_show,
		nomotion_config_store, 0);
static IIO_DEVICE_ATTR(anymotion_config, S_IRUGO|S_IWUSR, anymotion_config_show,
		anymotion_config_store, 0);
static IIO_DEVICE_ATTR(any_motion_enable_axis, S_IRUGO|S_IWUSR,
		any_motion_enable_axis_show, any_motion_enable_axis_store, 0);
static IIO_DEVICE_ATTR(err_int, S_IRUGO|S_IWUSR, err_int_show, err_int_store, 0);

static struct attribute *bma4xy_attributes[] = {
	&iio_dev_attr_chip_id.dev_attr.attr,
	&iio_dev_attr_acc_op_mode.dev_attr.attr,
	&iio_dev_attr_acc_value.dev_attr.attr,
	&iio_dev_attr_acc_range.dev_attr.attr,
	&iio_dev_attr_acc_odr.dev_attr.attr,
	&iio_dev_attr_acc_fifo_enable.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_avail_sensor.dev_attr.attr,
	&iio_dev_attr_foc.dev_attr.attr,
	&iio_dev_attr_axis_remap.dev_attr.attr,
	&iio_dev_attr_fifo_length.dev_attr.attr,
	&iio_dev_attr_fifo_watermark.dev_attr.attr,
	&iio_dev_attr_fifo_flush.dev_attr.attr,
	&iio_dev_attr_driver_version.dev_attr.attr,
	&iio_dev_attr_load_config_stream.dev_attr.attr,
	&iio_dev_attr_fifo_data_frame.dev_attr.attr,
	&iio_dev_attr_config_file_version.dev_attr.attr,
	&iio_dev_attr_reg_sel.dev_attr.attr,
	&iio_dev_attr_reg_val.dev_attr.attr,
	&iio_dev_attr_config_function.dev_attr.attr,
	&iio_dev_attr_fifo_config.dev_attr.attr,
	&iio_dev_attr_step_counter_val.dev_attr.attr,
	&iio_dev_attr_step_counter_watermark.dev_attr.attr,
	&iio_dev_attr_step_counter_parameter.dev_attr.attr,
	&iio_dev_attr_step_counter_reset.dev_attr.attr,
	&iio_dev_attr_nomotion_config.dev_attr.attr,
	&iio_dev_attr_no_motion_enable_axis.dev_attr.attr,
	&iio_dev_attr_anymotion_config.dev_attr.attr,
	&iio_dev_attr_any_motion_enable_axis.dev_attr.attr,
	&iio_dev_attr_err_int.dev_attr.attr,
	NULL
};

static struct attribute_group bma4xy_attribute_group = {
	.attrs = bma4xy_attributes
};

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
				fifo_bytecount + 4,
				&client_data->device);

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

	PDEBUG("%x", (u32)(status));
}

/**
 * bm4xy_accel_handle_interrupt - Handles and clears the interrupt
 *
 * @client_data : Instance of client data.
 */
static void bm4xy_accel_handle_interrupt(struct bma4xy_client_data *client_data)
{
	unsigned char int_status[2] = {0, 0};
	struct bma456h_out_state out_state;
	int err = 0;

	/*read the interrupt status two register*/
	err = bma4_read_regs(BMA4_INT_STAT_0_ADDR, int_status, 2,
				&client_data->device);
	if (err)
		return;

	/* This function is called from thread bottom handler
	 * debug here will flood the the log slowing down the system
	 */
	//PDEBUG("int_status0 = 0x%x int_status1 =0x%x",
	//	int_status[0], int_status[1]);

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
		client_data->anymotion_en = 0;

		err = bma456h_feature_enable(client_data->any_motion_axis,
					BMA4_DISABLE,
					&client_data->device);
		if (err)
			PDEBUG("any motion feature enabled failed");

		err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_ANY_MOT_INT,
					BMA4_DISABLE,
					&client_data->device);
		if (err)
			PDEBUG("any motion interrupt dis mapping failed");
	}
	if ((int_status[0] & NO_MOTION_OUT) == NO_MOTION_OUT) {
		PDEBUG("no motion interrupt occured");
		client_data->nomotion_en = 0;

		err = bma456h_feature_enable(client_data->no_motion_axis,
					BMA4_DISABLE,
					&client_data->device);
		if (err)
			PDEBUG("no motion feature enabled failed");

		err = bma456h_map_interrupt(BMA4_INTR1_MAP, BMA456H_NO_MOT_INT,
					BMA4_DISABLE,
					&client_data->device);
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
 * bma4xy_irq_handle - Top half of the IRQ handler (producer)
 * @irq : Number of irq line.
 * @private : IIO device
 *
 * Return : Status of IRQ function.
 */
static irqreturn_t bma4xy_accel_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	if (client_data->accel_iio_trigger_state) {
                iio_trigger_poll(client_data->indio_trig);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

/**
 * bma4xy_irq_work_func - Threaded half of the interrupt handler (producer)
 * @irq : Number of irq line.
 * @private : IIO device
 *
 * Return : Status of IRQ function.
 */
static irqreturn_t bma4xy_accel_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	bm4xy_accel_handle_interrupt(client_data);
	return IRQ_HANDLED;
}

/**
 * bma4xy_accel_buffer_preenable - IIO callback during buffer preenable
 * @iio_dev : IIO device
 *
 * Return : 0 on Success or non-zero on failure
 */
static int bma4xy_accel_buffer_preenable(struct iio_dev *indio_dev)
{
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	return bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
}

/**
 * bma4xy_accel_buffer_postenable - IIO callback during buffer postenable
 * @iio_dev : IIO device
 *
 * Return : 0 on Success or non-zero on failure
 */
static int bma4xy_accel_buffer_postenable(struct iio_dev *indio_dev)
{
#if 0
	int ret;
	unsigned char acc_fifo_enable = 1;
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	if (client_data->acc_fifo_enable) {
		PERR("FIFO already enabled\n");
		return 0;
	}

	ret = bma4_set_fifo_config(
		BMA4_FIFO_ACCEL, acc_fifo_enable, &client_data->device);
	if (ret) {
		PERR("FIFO enabled failed");
		return -EIO;
	}

        client_data->acc_fifo_enable = acc_fifo_enable;
#endif

	return iio_triggered_buffer_postenable(indio_dev);
}

/**
 * bma4xy_accel_buffer_predisable - IIO callback during buffer predisable
 * @iio_dev : IIO device
 *
 * Return : 0 on Success or non-zero on failure
 */
static int bma4xy_accel_buffer_predisable(struct iio_dev *indio_dev)
{
#if 0
	int ret;
	unsigned char acc_fifo_enable = 0;
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);


	if (client_data->acc_fifo_enable == 0) {
		PERR("FIFO already disabled\n");
	}

	ret = bma4_set_fifo_config(
		BMA4_FIFO_ACCEL, acc_fifo_enable, &client_data->device);
	if (ret) {
		PERR("FIFO enabled failed");
		return -EIO;
	}

	client_data->acc_fifo_enable = acc_fifo_enable;
#endif

	return iio_triggered_buffer_predisable(indio_dev);;
}

/**
 * bma4xy_accel_buffer_postdisable - IIO callback during buffer postdisable
 * @iio_dev : IIO device
 *
 * Return : 0 on Success or non-zero on failure
 */
static int bma4xy_accel_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	return bma4_set_accel_enable(BMA4_DISABLE, &client_data->device);
}

static const struct iio_buffer_setup_ops bma4xy_accel_buffer_ops = {
	.preenable = bma4xy_accel_buffer_preenable,
	.postenable = bma4xy_accel_buffer_postenable,
	.predisable = bma4xy_accel_buffer_predisable,
	.postdisable = bma4xy_accel_buffer_postdisable,
};

/**
 * bma4xy_accel_trigger_handler - IIO poll callback to push event to userspace.
				The consumer part of the IIO interrupt.
 * @irq : Number of the irq
 * @p : IIO trigger poll funtion data
 *
 * Return : Status of IRQ function.
 */
static irqreturn_t bma4xy_accel_trigger_handler(int irq, void *p)
{
	struct bma4_accel data;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
	int bit, i = 0;
	int err;

	err = bma4_read_accel_xyz(&data, &client_data->device);
	if (err < 0)
		goto err_read;

	for_each_set_bit(bit, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch(i) {
			case 0:
				client_data->buffer[i] = data.x;
				break;
			case 1:
				client_data->buffer[i] = data.y;
				break;
			case 2:
				client_data->buffer[i] = data.z;
				break;
			default:
				PERR("bma4xy_accel_trigger_handler() - invalid index\n");
		};

		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, client_data->buffer,
					   pf->timestamp);
err_read:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

/**
 * bma4xy_accel_trigger_set_state - IIO callback to enable/disable interrupt
 * @trig : IIO trigger
 * @state : state of the trigger requested
 *
 * Return : Status of IRQ function.
 */
static int bma4xy_accel_trigger_set_state(struct iio_trigger *trig,
					  bool state)
{
	int ret;
	u8 data_rdy = 0x0;
        u8 int_enable = 0x0;
        u8 latch_enable = 0x0;
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	if (client_data->accel_iio_trigger_state == state)
		return 0;

	if (state) {
		data_rdy = 0x04;
		int_enable = 0x0a;
		latch_enable = 0x01;
	}

        ret = bma4_write_regs(BMA4_INT_MAP_DATA_ADDR,
				&data_rdy, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
        ret += bma4_write_regs(BMA4_INT1_IO_CTRL_ADDR,
				&int_enable, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
	ret += bma4_write_regs(BMA4_INTR_LATCH_ADDR,
				&latch_enable, 1, &client_data->device);
	bma4xy_i2c_delay_us(MS_TO_US(1), &client_data->device.intf_ptr);
        if (ret) {
                PERR("Set data_rdy interrupt to %d failed err=%d",
				state, ret);
		return ret;
	}

	client_data->accel_iio_trigger_state = state;
	PDEBUG("bma4xy_accel_trigger_set_state()=%d\n",
			client_data->accel_iio_trigger_state);
	return ret;
}

static const struct iio_trigger_ops bma4xy_accel_trigger_ops = {
	.set_trigger_state = bma4xy_accel_trigger_set_state,
	.owner = THIS_MODULE,
};
#endif /* defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2) */

#define BMA4XY_ACCEL_CHANNEL(_axis, bits) {                             \
        .type = IIO_ACCEL,                                              \
        .modified = 1,                                                  \
        .channel2 = IIO_MOD_##_axis,                                    \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |          \
				BIT(IIO_CHAN_INFO_SAMP_FREQ),           \
        .scan_index = AXIS_##_axis,                                     \
        .scan_type = {                                                  \
                .sign = 's',                                            \
                .realbits = (bits),                                     \
                .storagebits = 16,                                      \
                .shift = 16 - (bits),                                   \
                .endianness = IIO_CPU,                                   \
        },                                                              \
}

#define BMA4XY_ACCEL_CHANNELS(bits) {					\
	BMA4XY_ACCEL_CHANNEL(X, bits),					\
	BMA4XY_ACCEL_CHANNEL(Y, bits),					\
	BMA4XY_ACCEL_CHANNEL(Z, bits),					\
	IIO_CHAN_SOFT_TIMESTAMP(3),					\
}

static const struct iio_chan_spec bma456_accel_channels[] =
	BMA4XY_ACCEL_CHANNELS(16);

/**
 * bma4xy_acc_read_raw - IIO callback to read data via polling
 * @indio_dev : IIO device
 * @chan : IIO channel to read
 * @val : IIO whole number part of the float value
 * @val2 : IIO fractional part of the float value
 * @mask : IIO bit mask of the value
 *
 * Return : IIO value type, or negative value on error
 */
static int bma4xy_acc_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);
	struct bma4_accel data;
	int scale_value[] = {598, 1197, 2394, 4788};
	int acc_odr = client_data->acc_odr - 1;
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;

		err = bma4_read_accel_xyz(&data, &client_data->device);
		if (err < 0)
			return err;

		/* Enable accel, if reading the first time using polling */
		if (data.x == 0 && data.y == 0 && data.z == 0) {
			bma4_set_accel_enable(BMA4_ENABLE, &client_data->device);
			return -EBUSY;
		}
		PDEBUG("[Raw %d] %hd %hd %hd\n",
				chan->channel2 - 1, data.x, data.y, data.z);
		switch(chan->channel2 - 1) {
			case 0:
				*val = data.x;
				return IIO_VAL_INT;
			case 1:
				*val = data.y;
				return IIO_VAL_INT;
			case 2:
				*val = data.z;
				return IIO_VAL_INT;
			default:
				PERR("bma4xy_acc_read_raw() - invalid index\n");
		};
		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = scale_value[client_data->acc_range];
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = bma456_accel_samp_freq_table[acc_odr].val;
		*val2 = bma456_accel_samp_freq_table[acc_odr].val2;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info bma4xy_accel_info = {
	.attrs			= &bma4xy_attribute_group,
	.read_raw		= bma4xy_acc_read_raw,
	.driver_module		= THIS_MODULE,
};

/**
 * bma4xy_probe - Does Sensor initialization
 * @dev: Device instance
 * @client_data : Instance of client data.
 */
int bma4xy_probe(struct device *dev, u8 dev_id, int irq,
	void (*init_client_data)(struct bma4xy_client_data *client_data))
{
	int err = 0;
	struct bma4xy_client_data *client_data;
	struct iio_dev *indio_dev;

	PINFO("function entrance");
	/* check chip id */
	if (dev_id != BMA456_ACCEL_CHIP_ID) {
		PERR("Check ID failed expecting 0x16 but got 0x%x", dev_id);
		err = -EINVAL;
		goto exit_err_clean;
	}

	/* Allocate an IIO device */
	indio_dev = devm_iio_device_alloc(dev, sizeof(*client_data));
	if (!indio_dev)
		goto exit_err_clean;

	client_data = iio_priv(indio_dev);

	/* Initialize interface specific client data*/
	client_data->IRQ = irq;
	if (init_client_data)
		init_client_data(client_data);

	indio_dev->dev.parent = dev;
        indio_dev->channels = bma456_accel_channels;
        indio_dev->num_channels = ARRAY_SIZE(bma456_accel_channels);
        indio_dev->name = SENSOR_NAME;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->info = &bma4xy_accel_info;
	dev_set_drvdata(dev, indio_dev);

	client_data->dev = dev;
	client_data->device.delay_us = bma4xy_i2c_delay_us;
	client_data->selftest = 2;
	client_data->acc_range = 1;
	client_data->acc_odr = 8;

	err = bma456h_init(&client_data->device);
	client_data->any_motion_axis = BMA456H_NO_MOTION_ALL_AXIS_EN;
	client_data->no_motion_axis = BMA456H_ANY_MOTION_ALL_AXIS_EN;
	if (err < 0)
		PERR("init failed %d", err);
	err = bma4_set_command_register(0xB6, &client_data->device);
	if (!err)
		PDEBUG("reset chip");

	bma4xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);

#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)
	/* Setup IIO Trigger Buffer */
	err = iio_triggered_buffer_setup(indio_dev,
				&iio_pollfunc_store_time,
				bma4xy_accel_trigger_handler,
				&bma4xy_accel_buffer_ops);
	if (err < 0) {
		PERR("Accel triggered buffer setup failed\n");
		goto exit_err_clean;
	}

	/* Request irq */
	err = devm_request_threaded_irq(dev, client_data->IRQ,
			bma4xy_accel_irq_handler,
			bma4xy_accel_irq_thread_handler,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME,
			indio_dev);
	if (err < 0)
		PERR("Request irq failed");

#endif /* defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2) */

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

	PINFO("sensor %s probed successfully", SENSOR_NAME);

#if defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2)
	/* Setup IIO Trigger */
	client_data->indio_trig = devm_iio_trigger_alloc(
					indio_dev->dev.parent,
					"%s-dev%d",
					indio_dev->name,
					indio_dev->id);
	if (!client_data->indio_trig) {
		PERR("Accel trigger alloc failed\n");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	client_data->indio_trig->dev.parent = indio_dev->dev.parent;
	client_data->indio_trig->ops = &bma4xy_accel_trigger_ops;
	iio_trigger_set_drvdata(client_data->indio_trig, indio_dev);
	err = iio_trigger_register(client_data->indio_trig);
	if (err) {
		PERR("iio trigger register failed\n");
		goto exit_err_clean;
	}

	return iio_device_register(indio_dev);
#else
	return err;
#endif /* defined(BMA4XY_ENABLE_INT1) || defined(BMA4XY_ENABLE_INT2) */

exit_err_clean:
	PINFO("sensor %s probed failed %d", SENSOR_NAME, err);
	return err;
}

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
	int err = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	PINFO("suspend function entrance");
	err = enable_irq_wake(client_data->IRQ);
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
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	PINFO("resume function entrance");
	err = disable_irq_wake(client_data->IRQ);
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
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bma4xy_client_data *client_data = iio_priv(indio_dev);

	if (client_data != NULL) {
		bma4xy_i2c_delay_us(BMA4XY_I2C_WRITE_DELAY_TIME,
					&client_data->device.intf_ptr);

		iio_device_unregister(indio_dev);
	}
	return err;
}
/* Lint -save -e19 */
EXPORT_SYMBOL(bma4xy_remove);
/* Lint -restore */



