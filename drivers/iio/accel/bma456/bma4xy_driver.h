/*!
 * @section LICENSE
 * Copyright (c) 2019~2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_driver.h
 * @date     2020/11/10
 * @version  1.3.13
 *
 * @brief   The head file of BMA4XY device driver core code
 */
#ifndef _BMA4XY_DRIVER_H
#define _BMA4XY_DRIVER_H

/*********************************************************************/
/* System header files */
/*********************************************************************/

#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include "bma4_defs.h"
#include "bma4.h"

/*********************************************************************/
/* Own header files */
/*********************************************************************/
/* BMA4xy variants. Only one should be enabled */

#include "bma456h.h"

/*********************************************************************/
/* Sensor name definition */
/*********************************************************************/
#define SENSOR_NAME "bma456h"
/* Name of the feature input device */
#define SENSOR_NAME_FEAT "bma4xy_feat"

/* Generic */
#define BMA4XY_ENABLE_INT1				1
#define CHECK_CHIP_ID_TIME_MAX			5
#define FIFO_DATA_BUFSIZE_BMA4XY		1024
#define BMA4XY_LOAD_CONFIG_FILE_IN_INIT 1
#define BMA4XY_I2C_WRITE_DELAY_TIME		1
#define BMA4XY_MAX_RETRY_I2C_XFER		(10)
#define BMA4XY_MAX_RETRY_WAIT_DRDY		(100)
#define BMA4XY_MAX_RETRY_WAKEUP			(5)
#define BMA4XY_DELAY_MIN				(1)
#define BMA4XY_DELAY_DEFAULT			(200)
#define BYTES_PER_LINE					(16)
#define REL_FEAT_STATUS					1
#define BMA4XY_INIT_DELAY_TIME_US		100

/* Fifo definition */
#define A_BYTES_FRM      6
#define M_BYTES_FRM      8
#define MA_BYTES_FRM     14

/* bma4xy power mode */
struct pw_mode {
	u8 acc_pm;
	u8 mag_pm;
};

struct bma4xy_client_data {
	struct bma4_dev device;
	struct device *dev;
	u8 mag_fifo_enable;
	u8 acc_fifo_enable;
	u32 fifo_bytecount;
	struct pw_mode pw;
	u8 acc_odr;
	u8 acc_range;
#ifdef BMA4XY_AUX_INTERFACE_SUPPORT
	u8 mag_odr;
	u8 mag_chip_id;
#endif
	int IRQ;
	u8 gpio_pin;
	struct work_struct irq_work;
	char *config_stream_name;
	int reg_sel;
	int reg_len;
	struct delayed_work delay_work_sig;
	struct delayed_work delay_work_any_motion;
	struct delayed_work delay_work_no_motion;
	bool any_motion_trigger;
	bool no_motion_trigger;
	u16 delay_time;
	atomic_t in_suspend;
	u8 selftest;
	u8 stepdet_en;
	u8 stepcounter_en;
	u8 anymotion_en;
	u8 nomotion_en;
	u8 single_tap_en;
	u8 double_tap_en;
	u8 triple_tap_en;
	u8 activity_en;
	u8 fifo_full_int_en;
	u8 fifo_wm_int_en;
	u8 err_int_trigger_num;
	u32 step_counter_val;
	u32 step_counter_temp;
	u16 any_motion_axis;
	u16 no_motion_axis;
	struct iio_trigger *indio_trig;
	bool accel_iio_trigger_state;
	s16 buffer[8];
};

/**
 * bma4xy_probe - This is the probe function for bma4xy sensor.
 * Called from the I2C driver probe function to initialize the sensor.
 *
 * @dev : Structure instance of device.
 * @dev_id : Chip ID
 * @irq : Device irq
 * @init_client_data : Function pointer to initialize client data.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bma4xy_probe(struct device *dev, u8 dev_id, int irq,
	void (*init_client_data)(struct bma4xy_client_data *client_data));

/**
 * bma4xy_suspend - This function puts the driver and device to suspend mode.
 *
 * @dev : Structure instance of device.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bma4xy_suspend(struct device *dev);

/**
 * bma4xy_resume - This function is used to bring back device from suspend mode.
 *
 * @dev  : Structure instance of device.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bma4xy_resume(struct device *dev);

/**
 * bma4xy_remove - This function removes the driver from the device.
 *
 * @dev : Structure instance of device.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bma4xy_remove(struct device *dev);
#endif
