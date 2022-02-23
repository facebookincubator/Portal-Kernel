/*****************************************************************************
 *
 * Copyright (c) 2015 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation
 * ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/math64.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include "mcubeclass.h"

#define MCUBE_FUNC_DEBUG
#define MCUBE_LOG_DEBUG
#define MCUBE_ERROR_DEBUG

#define COMPATIBLE "mcube,mc3xxx"
#define MC3XXX_DEV_NAME       "mc3xxx"
#define MC3XXX_DEV_VERSION    "2.1.0"
#define MC3XXX_INPUT_NAME     "accelerometer"
#define MC3XXX_I2C_ADDR       0x4c

#ifdef MCUBE_FUNC_DEBUG
	#define MC_FUNC_PRINT(x...)        printk(x)
#else
	#define MC_FUNC_PRINT(x...)
#endif

#ifdef MCUBE_LOG_DEBUG
	#define MC_LOG_PRINT(x...)        printk(x)
#else
	#define MC_LOG_PRINT(x...)
#endif

#ifdef MCUBE_ERROR_DEBUG
	#define MC_ERR_PRINT(x...)        printk(x)
#else
	#define MC_ERR_PRINT(x...)
#endif

/* register address define */
#define MC3XXX_XOUT_REG		0x00
#define MC3XXX_YOUT_REG		0x01
#define MC3XXX_ZOUT_REG			0x02
#define MC3XXX_TILT_REG			0x03
#define MC3XXX_OPSTAT_REG		0x04
#define MC3XXX_SC_REG			0x05
#define MC3XXX_INTEN_REG		0x06
#define MC3XXX_MODE_REG		0x07
#define MC3XXX_SAMPR_REG		0x08
#define MC3XXX_TAPEN_REG		0x09
#define MC3XXX_TAPP_REG			0x0A
#define MC3XXX_DROP_REG		0x0B
#define MC3XXX_SHDB_REG		0x0C
#define MC3XXX_XOUT_EX_L_REG	0x0D
#define MC3XXX_XOUT_EX_H_REG	0x0E
#define MC3XXX_YOUT_EX_L_REG	0x0F
#define MC3XXX_YOUT_EX_H_REG	0x10
#define MC3XXX_ZOUT_EX_L_REG	0x11
#define MC3XXX_ZOUT_EX_H_REG	0x12

#define MC3XXX_CHIPID_REG		0x18

#define MC3XXX_OUTCFG_REG		0x20
#define MC3XXX_XOFFL_REG		0x21
#define MC3XXX_XOFFH_REG		0x22
#define MC3XXX_YOFFL_REG		0x23
#define MC3XXX_YOFFH_REG		0x24
#define MC3XXX_ZOFFL_REG		0x25
#define MC3XXX_ZOFFH_REG		0x26
#define MC3XXX_XGAIN_REG		0x27
#define MC3XXX_YGAIN_REG		0x28
#define MC3XXX_ZGAIN_REG		0x29

#define MC3XXX_SHAKE_TH_REG	0x2B
#define MC3XXX_UD_Z_TH_REG		0x2C
#define MC3XXX_UD_X_TH_REG		0x2D
#define MC3XXX_RL_Z_TH_REG		0x2E
#define MC3XXX_RL_Y_TH_REG		0x2F
#define MC3XXX_FB_Z_TH_REG		0x30
#define MC3XXX_DROP_TH_REG		0x31
#define MC3XXX_TAP_TH_REG		0x32

#define MC3XXX_PCODE_REG		0x3B

/* Mode */
#define MC3XXX_MODE_AUTO			0
#define MC3XXX_MODE_WAKE			1
#define MC3XXX_MODE_SNIFF			2
#define MC3XXX_MODE_STANDBY		3

/* Range */
#define MC3XXX_RANGE_2G			0
#define MC3XXX_RANGE_4G			1
#define MC3XXX_RANGE_8G			2
#define MC3XXX_RANGE_12G			2
#define MC3XXX_RANGE_16G			3

/* Resolution */
#define MC3XXX_RES_6BIT				0
#define MC3XXX_RES_7BIT				1
#define MC3XXX_RES_8BIT				2
#define MC3XXX_RES_10BIT			3
#define MC3XXX_RES_12BIT			4
#define MC3XXX_RES_14BIT			5

/* Bandwidth */
#define MC3XXX_BW_512HZ			0
#define MC3XXX_BW_256HZ			1
#define MC3XXX_BW_128HZ			2
#define MC3XXX_BW_64HZ				3
#define MC3XXX_BW_32HZ				4
#define MC3XXX_BW_16HZ				5
#define MC3XXX_BW_8HZ				6

/* Product code */
#define MC3XXX_PCODE_3210			0x90
#define MC3XXX_PCODE_3230			0x19
#define MC3XXX_PCODE_3250			0x88
#define MC3XXX_PCODE_3410			0xA8
#define MC3XXX_PCODE_3410N			0xB8
#define MC3XXX_PCODE_3430			0x29
#define MC3XXX_PCODE_3430N			0x39
#define MC3XXX_PCODE_3510			0x40
#define MC3XXX_PCODE_3530			0x30
#define MC3XXX_PCODE_3216			0x10
#define MC3XXX_PCODE_3236			0x60
#define MC3XXX_PCODE_3416			0xA0

#define MC3XXX_PCODE_RESERVE_1		0x20
#define MC3XXX_PCODE_RESERVE_2		0x11
#define MC3XXX_PCODE_RESERVE_3		0x21
#define MC3XXX_PCODE_RESERVE_4		0x61
#define MC3XXX_PCODE_RESERVE_5		0xA0
#define MC3XXX_PCODE_RESERVE_6		0xE0
#define MC3XXX_PCODE_RESERVE_7		0x91
#define MC3XXX_PCODE_RESERVE_8		0xA1
#define MC3XXX_PCODE_RESERVE_9		0xE1

#define MC3XXX_PCODE_RESERVE_10	0x99

#define MC3XXX_AXIS_X			0
#define MC3XXX_AXIS_Y			1
#define MC3XXX_AXIS_Z			2
#define MC3XXX_AXES_NUM		3

/* 1g constant value */
#define GRAVITY_1G_VALUE			16384

/* Initial value */
#define MC3XXX_RANGE_SET		MC3XXX_RANGE_8G  /* +/-8g */
#define MC3XXX_RESO_SET			MC3XXX_RES_14BIT /* 14bit */
#define MC3XXX_BW_SET			MC3XXX_BW_128HZ /* 128HZ  */
#define MC3XXX_MAX_DELAY		200
#define ABSMIN_1_5G				(-3 * GRAVITY_1G_VALUE / 2)
#define ABSMAX_1_5G				(3 * GRAVITY_1G_VALUE / 2)
#define ABSMIN_2G				(-2 * GRAVITY_1G_VALUE)
#define ABSMAX_2G				(2 * GRAVITY_1G_VALUE)
#define ABSMIN_8G				(-8 * GRAVITY_1G_VALUE)
#define ABSMAX_8G				(8 * GRAVITY_1G_VALUE)

/* MC3XXX power supply VDD 1.7V-3.6V VIO 1.7-3.6V */
#define MC3XXX_VDD_MIN_UV			2000000
#define MC3XXX_VDD_MAX_UV			3400000
#define MC3XXX_VIO_MIN_UV			1800000
#define MC3XXX_VIO_MAX_UV			3400000

/* Polling delay in msecs */
#define POLL_INTERVAL_MIN_MS		10
#define POLL_INTERVAL_MAX_MS		4000
#define POLL_DEFAULT_INTERVAL_MS	200

/* Interrupt delay in msecs */
#define MC3XXX_INT_MAX_DELAY		64

#define MC3XXX_MAX_RETRY_I2C_XFER	(100)
#define I2C_RETRY_DELAY()		usleep_range(1000, 2000)

/* End type */
enum mc3xxx_sensor_type {
	MC3XXX_LOW_END = 0,
	MC3XXX_HIGH_END
};

enum mc3xxx_placement {
	MC3XXX_TOP_LEFT_DOWN = 0,	/* 0: top, left-down */
	MC3XXX_TOP_RIGHT_DOWN,		/* 1: top, reight-down */
	MC3XXX_TOP_RIGHT_UP,		/* 2: top, right-up */
	MC3XXX_TOP_LEFT_UP,		/* 3: top, left-up */
	MC3XXX_BOTTOM_LEFT_DOWN,	/* 4: bottom, left-down */
	MC3XXX_BOTTOM_RIGHT_DOWN,	/* 5: bottom, right-down */
	MC3XXX_BOTTOM_RIGHT_UP,		/* 6: bottom, right-up */
	MC3XXX_BOTTOM_LEFT_UP		/* 7: bottom, left-up */
};

struct mc3xxx_hwmsen_convert {
	signed char sign[3];
	unsigned char map[3];
};

struct mc3xxx_acc {
	signed int x;
	signed int y;
	signed int z;
};

struct mc3xxx_platform_data {
	int poll_interval;
	int gpio_int;
	unsigned int int_flag;
	unsigned char place;
	bool int_en;
};

struct mc3xxx_suspend_state {
	bool powerEn;
};

struct mc3xxx_pcode_data {
	unsigned char pcode;
	unsigned char chipid;
	unsigned char mpol;
	bool mcfm12;
	bool mcfm3x;
};

struct mc3xxx_data {
	struct i2c_client *mc3xxx_client;
	atomic_t delay;
	atomic_t enable;
	struct input_dev *input;

	struct mcube_dev *bst_acc;

	struct mc3xxx_acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct mutex en_count_mutex;
	struct workqueue_struct *data_wq;
	struct delayed_work delay_work;
	struct work_struct irq_work;
	struct regulator *vdd;

	struct hrtimer	poll_timer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct mc3xxx_platform_data *pdata;
	struct mc3xxx_suspend_state suspend_state;

	unsigned int int_flag;
	int IRQ;
	struct mc3xxx_pcode_data pcode_data;
	unsigned short gain;
	unsigned char endtype;
	bool power_enabled;
	int cal_status;
	int cal_count;
	int en_count;
};

static const struct mc3xxx_hwmsen_convert mc3xxx_cvt[] = {
	{{1, 1, 1}, {MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z} },
	{{-1, 1, 1}, {MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z} },
	{{-1, -1, 1}, {MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z} },
	{{1, -1, 1}, {MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z} },
	{{-1, 1, -1}, {MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z} },
	{{1, 1, -1}, {MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z} },
	{{1, -1, -1}, {MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z} },
	{{-1, -1, -1}, {MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z} },
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h);
static void mc3xxx_late_resume(struct early_suspend *h);
#endif

static int mc3xxx_open_init(struct i2c_client *client,
	struct mc3xxx_data *data);
static int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode);
static int mc3xxx_get_mode(struct i2c_client *client, unsigned char *mode);
static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on);

static int mc3xxx_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -EIO;
	*data = dummy & 0x000000ff;

	return 0;
}

static int mc3xxx_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -EIO;
	udelay(2);
	return 0;
}

static int mc3xxx_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;

	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -EIO;
	return 0;
}

static int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode)
{
	unsigned char data;
	int ret = -1;
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_LOG_PRINT("%s: %d\n", __func__, mode);

	mutex_lock(&mc3xxx->mode_mutex);

	if (mode == MC3XXX_MODE_STANDBY) {
		data = 0x43;
		ret = mc3xxx_smbus_write_byte(client, MC3XXX_MODE_REG, &data);
	} else if (mode == MC3XXX_MODE_WAKE) {
		data = 0xe9;
		ret = mc3xxx_smbus_write_byte(client, MC3XXX_MODE_REG, &data);
	}
	mutex_unlock(&mc3xxx->mode_mutex);

	return ret;
}

static int mc3xxx_get_mode(struct i2c_client *client, unsigned char *mode)
{
	unsigned char data;
	int ret = -1;

	ret = mc3xxx_smbus_read_byte(client, MC3XXX_MODE_REG, &data);
	*mode = data & 0x03;

	return ret;
}

static int mc3xxx_check_product_code(struct i2c_client *client,
	struct mc3xxx_data *mc3xxx)
{
	const unsigned char auto_probe_addr[] = {0x4C, 0x6C};
	const unsigned char auto_probe_count =  (sizeof(auto_probe_addr) /
				sizeof(auto_probe_addr[0]));

	int i;
	int ret = 0;
	unsigned char pcode = 0;
	unsigned char chipid = 0;
	unsigned char mpol = 0;

	for (i = 0; i < auto_probe_count; i++) {
		client->addr = auto_probe_addr[i];

		ret = mc3xxx_smbus_read_byte(client, MC3XXX_PCODE_REG, &pcode);
		if (ret) {
			MC_ERR_PRINT("%s: read pcode error: %x\n",
				__func__, ret);
			continue;
		}
		pcode = (pcode & 0x31);/*for mensa */

		ret = mc3xxx_smbus_read_byte(client,
			MC3XXX_CHIPID_REG, &chipid);
		if (ret) {
			MC_ERR_PRINT("%s: read chipid error: %x\n",
				__func__, ret);
			continue;
		}

		ret = mc3xxx_smbus_read_byte(client, 0x2a, &mpol);
		if (ret) {
			MC_ERR_PRINT("%s: read mpol error: %x\n",
				__func__, ret);
			continue;
		}

		if (chipid == 0xA0) {/* for mensa */
			mc3xxx->pcode_data.pcode = pcode;
			mc3xxx->pcode_data.chipid = chipid;
			mc3xxx->pcode_data.mpol = mpol;
			mc3xxx->pcode_data.mcfm12 = (chipid >= 0xc0) &&
				(chipid <= 0xcf);
			mc3xxx->pcode_data.mcfm3x = (chipid == 0x20) ||
					((chipid >= 0x22) && (chipid <= 0x2f));

			MC_LOG_PRINT("%s: 0x%x, 0x%x\n", __func__,
				pcode, chipid);
			return 0;
		}
	}
	return -EINVAL;
}

static int mc3xxx_set_endtype(struct i2c_client *client,
	struct mc3xxx_data *mc3xxx)
{
	int err = 0;

	switch (mc3xxx->pcode_data.pcode) {
	case MC3XXX_PCODE_3230:
	case MC3XXX_PCODE_3430:
	case MC3XXX_PCODE_3430N:
	case MC3XXX_PCODE_3530:
	case MC3XXX_PCODE_3236:
		mc3xxx->endtype = MC3XXX_LOW_END;
		break;

	case MC3XXX_PCODE_3210:
	case MC3XXX_PCODE_3250:
	case MC3XXX_PCODE_3410:
	case MC3XXX_PCODE_3410N:
	case MC3XXX_PCODE_3510:
	case MC3XXX_PCODE_3216:
	case MC3XXX_PCODE_3416:
	case MC3XXX_PCODE_RESERVE_1:
		mc3xxx->endtype = MC3XXX_HIGH_END;
		break;

	default:
		err = -1;
		break;
	}

	return err;
}

static int mc3xxx_init_configure(struct i2c_client *client,
	struct mc3xxx_data *mc3xxx)
{
	int ret = 0;
	unsigned char tmp = 0;

	MC_FUNC_PRINT("%s called\n", __func__);

	/* set sample rate */
	if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x) {
		switch (mc3xxx->pcode_data.mpol & 0xc0) {
		case 0:
			tmp = 0;
			break;
		case 0x40:
			tmp = 0x08;
			break;
		case 0x80:
			tmp = 0x09;
			break;
		case 0xc0:
			tmp = 0x0a;
			break;
		}
	}
	ret += mc3xxx_smbus_read_byte_block(client, MC3XXX_SAMPR_REG, &tmp, 1);
	tmp = (tmp&0xf8)|0x05;
	ret += mc3xxx_smbus_write_byte(client, MC3XXX_SAMPR_REG, &tmp);

	/* configure range */
	if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x) {
		if (mc3xxx->endtype == MC3XXX_LOW_END)
			tmp = 0x02;
		else
			tmp = 0x25;
	} else if (mc3xxx->endtype == MC3XXX_LOW_END)
		tmp = 0x32;
	else
		tmp = 0x3f;

	tmp = 0x29;/* for mensa */
	ret += mc3xxx_smbus_write_byte(client, MC3XXX_OUTCFG_REG, &tmp);

	/* set gain value */
	if (mc3xxx->endtype == MC3XXX_LOW_END) {
		if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x)
			mc3xxx->gain = 64;
		else
			mc3xxx->gain = 86;
	} else
		mc3xxx->gain = 1024;

	mc3xxx->gain = 4096;/* for mensa */
	return ret;
}

static void mc3xxx_remap_sensor_data(struct mc3xxx_data *mc3xxx,
	struct mc3xxx_acc *acc)
{
	int tmp[3];
	const struct mc3xxx_hwmsen_convert *pCvt =
		&mc3xxx_cvt[mc3xxx->pdata->place];

	if (mc3xxx->pcode_data.pcode == MC3XXX_PCODE_3250) {
		tmp[0] = acc->y;
		tmp[1] = -acc->x;
		tmp[2] = acc->z;
	} else {
		if (mc3xxx->pcode_data.mpol & 0x01)
			tmp[0] = -acc->x;
		else
			tmp[0] = acc->x;
		if (mc3xxx->pcode_data.mpol & 0x02)
			tmp[1] = -acc->y;
		else
			tmp[1] = acc->y;
		tmp[2] = acc->z;
	}

	tmp[0] = tmp[0] * GRAVITY_1G_VALUE / mc3xxx->gain;
	tmp[1] = tmp[1] * GRAVITY_1G_VALUE / mc3xxx->gain;
	tmp[2] = tmp[2] * GRAVITY_1G_VALUE / mc3xxx->gain;

	acc->x = pCvt->sign[MC3XXX_AXIS_X] * tmp[pCvt->map[MC3XXX_AXIS_X]];
	acc->y = pCvt->sign[MC3XXX_AXIS_Y] * tmp[pCvt->map[MC3XXX_AXIS_Y]];
	acc->z = pCvt->sign[MC3XXX_AXIS_Z] * tmp[pCvt->map[MC3XXX_AXIS_Z]];
}

static int mc3xxx_read_accel_xyz(struct i2c_client *client,
	struct mc3xxx_acc *acc)
{
	int comres = 0;
	unsigned char data[6];
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	if (mc3xxx->endtype == MC3XXX_HIGH_END) {
		comres = mc3xxx_smbus_read_byte_block(client,
			MC3XXX_XOUT_EX_L_REG, data, 6);

		acc->x = (signed short)((data[1]<<8) | data[0]);
		acc->y = (signed short)((data[3]<<8) | data[2]);
		acc->z = (signed short)((data[5]<<8) | data[4]);
	} else {
		comres = mc3xxx_smbus_read_byte_block(client,
				MC3XXX_XOUT_REG, data, 3);

		acc->x = (signed char)data[0];
		acc->y = (signed char)data[1];
		acc->z = (signed char)data[2];
	}
	acc->x = acc->x/4;
	acc->y = acc->y/4;
	acc->z = acc->z/4;

	mc3xxx_remap_sensor_data(mc3xxx, acc);

	return comres;
}

static void mc3xxx_report_axis_data(struct mc3xxx_data *mc3xxx,
	struct mc3xxx_acc *value)
{
	ktime_t ts;
	int err;

	ts = ktime_get_boottime();
	err = mc3xxx_read_accel_xyz(mc3xxx->mc3xxx_client, value);
	if (err < 0) {
		dev_err(&mc3xxx->mc3xxx_client->dev,
			"read accel data failed! err = %d\n", err);
		return;
	}

	input_report_abs(mc3xxx->input, ABS_X, value->x/4);
	input_report_abs(mc3xxx->input, ABS_Y, (-1) * value->y/4);
	input_report_abs(mc3xxx->input, ABS_Z, (-1) * value->z/4);
	input_sync(mc3xxx->input);
}

static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *mc3xxx = container_of((struct delayed_work *)work,
			struct mc3xxx_data, delay_work);
	struct mc3xxx_acc value;

	mc3xxx_report_axis_data(mc3xxx, &value);
	mutex_lock(&mc3xxx->value_mutex);
	mc3xxx->value = value;
	mutex_unlock(&mc3xxx->value_mutex);
}

static ssize_t mc3xxx_place_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	mc3xxx->pdata->place = data;

	return count;
}

static ssize_t mc3xxx_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value, rel;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	rel = sscanf(buf, "%3d %3d", &address, &value);
	if (rel != 2) {
		MC_ERR_PRINT("%s: sscanf fails\n", __func__);
		return -EINVAL;
	}

	if (mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client,
				(unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;
	return count;
}

static ssize_t mc3xxx_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x40];
	int i;

	MC_FUNC_PRINT("%s called\n", __func__);

	for (i = 0; i < 0x40; i++) {
		mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, i, reg+i);

		count += snprintf(&buf[count], PAGE_SIZE,
			"0x%x: %x\n", i, reg[i]);
	}
	return count;
}

static ssize_t mc3xxx_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	if (mc3xxx_get_mode(mc3xxx->mc3xxx_client, &data) < 0)
		return snprintf(buf, PAGE_SIZE, "Read error\n");

	return snprintf(buf, PAGE_SIZE, "0x%x\n", data);
}

static ssize_t mc3xxx_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (mc3xxx_set_mode(mc3xxx->mc3xxx_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mc3xxx_value_cache_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct mc3xxx_data *mc3xxx = input_get_drvdata(input);
	struct mc3xxx_acc acc_value;

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&mc3xxx->value_mutex);
	acc_value = mc3xxx->value;
	mutex_unlock(&mc3xxx->value_mutex);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t mc3xxx_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct mc3xxx_data *mc3xxx = input_get_drvdata(input);
	struct mc3xxx_acc acc_value;

	MC_FUNC_PRINT("%s called\n", __func__);

	mc3xxx_read_accel_xyz(mc3xxx->mc3xxx_client, &acc_value);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t mc3xxx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mc3xxx->delay));

}

static ssize_t mc3xxx_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "0x%x, 0x%x\n",
		mc3xxx->pcode_data.pcode, mc3xxx->pcode_data.chipid);
}

static ssize_t mc3xxx_place_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int place;

	MC_FUNC_PRINT("%s called\n", __func__);

	place = mc3xxx->pdata->place;

	return snprintf(buf, PAGE_SIZE, "%d\n", place);
}

static ssize_t mc3xxx_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data < POLL_INTERVAL_MIN_MS)
		data = POLL_INTERVAL_MIN_MS;
	if (data > POLL_INTERVAL_MAX_MS)
		data = POLL_INTERVAL_MAX_MS;
	atomic_set(&mc3xxx->delay, (unsigned int) data);

	return count;
}

static ssize_t mc3xxx_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mc3xxx->enable));

}

static void mc3xxx_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&mc3xxx->enable);
	unsigned char tmp = 0;

	MC_LOG_PRINT("%s: %d, %d\n", __func__, enable, pre_enable);

	mutex_lock(&mc3xxx->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			if (mc3xxx_power_ctl(mc3xxx, true)) {
				dev_err(dev, "power failed\n");
				goto mutex_exit;
			}

			if (mc3xxx_open_init(mc3xxx->mc3xxx_client,
				mc3xxx) < 0) {
				dev_err(dev, "set init failed\n");
				goto mutex_exit;
			}

			if (mc3xxx->pdata->int_en) {
				tmp = 0x80; /* Enable ACQ_INT for Merak */
				mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client,
					MC3XXX_INTEN_REG, &tmp);

				enable_irq(mc3xxx->IRQ);
			} else {
				hrtimer_start(&mc3xxx->poll_timer,
					ns_to_ktime(atomic_read(&mc3xxx->delay)
					* 1000000UL), HRTIMER_MODE_REL);
			}
			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_WAKE);

			atomic_set(&mc3xxx->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_STANDBY);

			if (mc3xxx->pdata->int_en) {
				disable_irq(mc3xxx->IRQ);

				tmp = 0; /* Disable ACQ_INT for Merak */
				mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client,
					MC3XXX_INTEN_REG, &tmp);
			} else {
				hrtimer_cancel(&mc3xxx->poll_timer);
				cancel_work_sync(&mc3xxx->delay_work.work);

				atomic_set(&mc3xxx->enable, 0);
				if (mc3xxx_power_ctl(mc3xxx, false)) {
					dev_err(dev, "power failed\n");
					goto mutex_exit;
				}
			}
		}
	}

mutex_exit:
	mutex_unlock(&mc3xxx->enable_mutex);
	dev_dbg(&client->dev,
		"set enable: en=%d, en_state=%d, use_int=%d\n",
		enable, atomic_read(&mc3xxx->enable),
		mc3xxx->pdata->int_en);
}

static ssize_t mc3xxx_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	unsigned long data;
	int error;

	MC_FUNC_PRINT("%s called\n", __func__);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	mutex_lock(&mc3xxx->en_count_mutex);
	if (data == 1) {
		mc3xxx->en_count++;
		if (mc3xxx->en_count == 1) {
			mc3xxx_set_enable(dev, 1);
		}
	} else if (data == 0) {
		if (mc3xxx->en_count == 0) {
			dev_warn(dev, "Calling accel disable without first enabling it!");
			mutex_unlock(&mc3xxx->en_count_mutex);
			return -EINVAL;
		}
		mc3xxx->en_count--;
		if (mc3xxx->en_count == 0) {
			mc3xxx_set_enable(dev, 0);
		}
	}
	mutex_unlock(&mc3xxx->en_count_mutex);

	return count;
}

static ssize_t mc3xxx_cal_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int data;

	MC_FUNC_PRINT("%s called\n", __func__);

	data = mc3xxx->cal_status;
	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t mc3xxx_cal_status_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called, buf = %s\n", __func__, buf);
	error = kstrtoint(buf, 10, &data);
	if (error)
		return error;

	mc3xxx->cal_status = data;
	return count;
}

static ssize_t mc3xxx_cal_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int data;

	MC_FUNC_PRINT("%s called\n", __func__);

	data = mc3xxx->cal_count;
	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t mc3xxx_cal_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data == 1) {
		mc3xxx->cal_status = -1;
		input_report_abs(mc3xxx->input, ABS_THROTTLE,
			++(mc3xxx->cal_count));
		input_sync(mc3xxx->input);
	}
	return count;
}

static DEVICE_ATTR(op_mode, 0640,
		mc3xxx_mode_show, mc3xxx_mode_store);
static DEVICE_ATTR(value, 0440,
		mc3xxx_value_show, NULL);
static DEVICE_ATTR(value_cache, 0440,
		mc3xxx_value_cache_show, NULL);
static DEVICE_ATTR(delay, 0660,
		mc3xxx_delay_show, mc3xxx_delay_store);
static DEVICE_ATTR(enable, 0660,
		mc3xxx_enable_show, mc3xxx_enable_store);
static DEVICE_ATTR(reg, 0640,
		mc3xxx_register_show, mc3xxx_register_store);
static DEVICE_ATTR(chip_id, 0440,
		mc3xxx_chip_id_show, NULL);
static DEVICE_ATTR(place, 0660,
		mc3xxx_place_show, mc3xxx_place_store);
static DEVICE_ATTR(cal_h, 0660,
		mc3xxx_cal_h_show, mc3xxx_cal_h_store);
static DEVICE_ATTR(cal_status, 0660,
		mc3xxx_cal_status_show, mc3xxx_cal_status_store);

static struct attribute *mc3xxx_attributes[] = {
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_value_cache.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_reg.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_place.attr,
	&dev_attr_cal_h.attr,
	&dev_attr_cal_status.attr,
	NULL
};

static struct attribute_group mc3xxx_attribute_group = {
	.attrs = mc3xxx_attributes
};

static void mc3xxx_irq_work_func(struct work_struct *work)
{
	struct mc3xxx_data *mc3xxx = container_of((struct work_struct *)work,
			struct mc3xxx_data, irq_work);
	struct mc3xxx_acc value;

	unsigned char status = 0;

	mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, MC3XXX_TILT_REG, &status);

	mc3xxx_report_axis_data(mc3xxx, &value);

	mutex_lock(&mc3xxx->value_mutex);
	mc3xxx->value = value;
	mutex_unlock(&mc3xxx->value_mutex);
}

static irqreturn_t mc3xxx_irq_handler(int irq, void *handle)
{
	struct mc3xxx_data *data = handle;

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->mc3xxx_client == NULL)
		return IRQ_HANDLED;

	queue_work(data->data_wq, &data->irq_work);

	return IRQ_HANDLED;
}

static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on)
{
	int ret = 0;

	MC_FUNC_PRINT("%s called\n", __func__);

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		data->power_enabled = on;
	} else {
		dev_info(&data->mc3xxx_client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int mc3xxx_power_init(struct mc3xxx_data *data)
{
	int ret = 0;

	MC_FUNC_PRINT("%s called\n", __func__);

	data->vdd = regulator_get(&data->mc3xxx_client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				1800000,
				1800000);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int mc3xxx_power_deinit(struct mc3xxx_data *data)
{
	int ret;

	MC_FUNC_PRINT("%s called\n", __func__);

	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);

	regulator_put(data->vdd);

	return 0;
}

#ifdef CONFIG_OF
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	MC_FUNC_PRINT("%s called\n", __func__);

	rc = of_property_read_u32(np, "mcube,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	}
	pdata->poll_interval = temp_val;

	rc = of_property_read_u32(np, "mcube,place", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read sensor place parameter\n");
		return rc;
	}
	if (temp_val > 7 || temp_val < 0) {
		dev_err(dev, "Invalid place parameter, use default value 0\n");
		pdata->place = 0;
	} else {
		pdata->place = temp_val;
	}

	rc = of_property_read_u32(np,
				"mcube,use-interrupt", &temp_val);
	if (rc) {
		pdata->int_en = 0;
		if (rc != -EINVAL) {
			dev_err(dev, "Unable to read intr parameter\n");
			return rc;
		}
	} else
		pdata->int_en = temp_val;

	pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
				"mcube,gpio-int", 0, &pdata->int_flag);
	return 0;
}
#else
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static int mc3xxx_open_init(struct i2c_client *client,
	struct mc3xxx_data *data)
{
	int err;

	MC_FUNC_PRINT("%s called\n", __func__);
	err = mc3xxx_set_mode(client, MC3XXX_MODE_STANDBY);
	if (err < 0) {
		dev_err(&client->dev, "set mode error\n");
		return err;
	}

	err = mc3xxx_set_endtype(client, data);
	if (err < 0) {
		dev_err(&client->dev, "set endtype error\n");
		return err;
	}

	err = mc3xxx_init_configure(client, data);
	if (err < 0) {
		dev_err(&client->dev, "init configure error\n");
		return err;
	}

	return 0;
}

static int mc3xxx_get_interrupt_gpio(const struct mc3xxx_data *data,
			const unsigned int gpio)
{
	struct i2c_client *client = data->mc3xxx_client;
	int err;

	if (!gpio_is_valid(gpio)) {
		dev_err(&client->dev,
			"gpio(%d) is invalid,\n", gpio);
		return -EINVAL;
	}

	err = gpio_request(gpio, "mc3xxx_gpio_int");
	if (err) {
		dev_err(&client->dev,
			"Unable to request gpio %d, err=%d\n",
			gpio, err);
		return err;
	}

	err = gpio_direction_input(gpio);
	if (err) {
		dev_err(&client->dev,
			"Unable to set gpio direction %d, err=%d\n",
			gpio, err);
		gpio_free(gpio);
		return err;
	}

	client->irq = gpio_to_irq(gpio);
	dev_dbg(&client->dev, "Interrupt gpio=%d, irq=%d\n", gpio, client->irq);

	return 0;
}

static enum hrtimer_restart mc3xxx_timer_func(struct hrtimer *timer)
{
	struct mc3xxx_data *data = container_of(timer,
			struct mc3xxx_data, poll_timer);

	queue_work(data->data_wq, &data->delay_work.work);
	hrtimer_forward_now(&data->poll_timer,
			ns_to_ktime(atomic_read(&data->delay) * 1000000UL));

	return HRTIMER_RESTART;
}

static void mc3xxx_pinctrl_state(struct mc3xxx_data *data,
			bool active)
{
}

static int mc3xxx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct mc3xxx_data *data;
	struct input_dev *dev;
	struct mc3xxx_platform_data *pdata;
	struct mcube_dev  *dev_acc;

	MC_FUNC_PRINT("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MC_ERR_PRINT("%s: i2c_check_functionality error\n",
			__func__);
		dev_err(&client->dev, "i2c_check_functionality error\n");
		err = -EPERM;
		goto exit;
	}
	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if (!data) {
		MC_ERR_PRINT("%s: alloc mc3xxx_data error\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			MC_ERR_PRINT("%s: failed to allcated memory\n",
				__func__);
			dev_err(&client->dev, "Failed to allcated memory\n");
			err = -ENOMEM;
			goto kfree_exit;
		}
		err = mc3xxx_parse_dt(&client->dev, pdata);
		if (err) {
			MC_ERR_PRINT("%s: failed to parse device tree\n",
				__func__);
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto pdata_free_exit;
		}
	} else {
		pdata = client->dev.platform_data;
		MC_ERR_PRINT("%s: use platform data\n", __func__);
		dev_err(&client->dev, "Use platform data\n");
	}

	if (!pdata) {
		MC_ERR_PRINT("%s: cannot get device platform data\n",
			__func__);
		dev_err(&client->dev, "Cannot get device platform data\n");
		err = -EINVAL;
		goto kfree_exit;
	}
	data->pdata = pdata;
	i2c_set_clientdata(client, data);
	data->mc3xxx_client = client;

	err = mc3xxx_power_init(data);
	if (err) {
		MC_ERR_PRINT("%s: failed to get sensor regulators\n",
			__func__);
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto free_i2c_clientdata_exit;
	}
	err = mc3xxx_power_ctl(data, true);
	if (err) {
		MC_ERR_PRINT("%s: failed to enable sensor power\n",
			__func__);
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}
	msleep(20);
	/* read and check product code */
	if (mc3xxx_check_product_code(client, data) < 0) {
		MC_ERR_PRINT("%s: can't detect mCube's g-sensor\n",
			__func__);
		err = -EINVAL;
		goto disable_power_exit;
	}

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	mutex_init(&data->en_count_mutex);

	err = mc3xxx_open_init(client, data);
	if (err < 0) {
		MC_ERR_PRINT("%s: failed to initialize\n", __func__);
		err = -EINVAL;
		goto disable_power_exit;
	}

	if (pdata->int_en) {
		data->int_flag = pdata->int_flag;
		err = mc3xxx_get_interrupt_gpio(data, pdata->gpio_int);
		if (err) {
			dev_err(&client->dev,
				"Failed to get interrupt gpio, err=%d\n",
				err);
			err = -EINVAL;
			goto disable_power_exit;
		}

		data->IRQ = client->irq;
		if (!data->int_flag)
			data->int_flag = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

		dev_dbg(&client->dev, "IRQ=%d, int_flag=0x%x\n",
			data->IRQ, data->int_flag);

		err = request_irq(data->IRQ, mc3xxx_irq_handler,
			data->int_flag, "mc3xxx", data);
		if (err) {
			dev_err(&client->dev,  "Could not request irq\n");
			goto free_interrupt_gpio;
		}
		disable_irq(data->IRQ);
		INIT_WORK(&data->irq_work, mc3xxx_irq_work_func);
	} else {
		hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);
		data->poll_timer.function = mc3xxx_timer_func;
		INIT_WORK(&data->delay_work.work, mc3xxx_work_func);
	}

	data->data_wq = alloc_workqueue("mc3xxx_data_work",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);

	if (!data->data_wq) {
		MC_ERR_PRINT("%s: cannot get create workqueue\n",
			__func__);
		dev_err(&client->dev, "Cannot get create workqueue!\n");
		goto free_irq_exit;
	}

	atomic_set(&data->delay, POLL_DEFAULT_INTERVAL_MS);
	atomic_set(&data->enable, 0);

	dev = devm_input_allocate_device(&client->dev);
	if (!dev) {
		MC_ERR_PRINT("%s: cannot allocate input device\n", __func__);
		dev_err(&client->dev, "Cannot allocate input device\n");
		err = -ENOMEM;
		goto destroy_workqueue_exit;
	}

	/* only value events reported */
	dev->name = "accel";
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_MISC);

	if (data->endtype == MC3XXX_HIGH_END) {
		input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_THROTTLE, ABSMIN_8G,
			ABSMAX_8G, 0, 0);
	} else {
		input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
		input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
		input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
		input_set_abs_params(dev, ABS_THROTTLE, ABSMIN_2G,
			ABSMAX_2G, 0, 0);
	}
	data->cal_count = 0;

	input_set_drvdata(dev, data);
	err = input_register_device(dev);
	if (err < 0) {
		MC_ERR_PRINT("%s: cannot register input device\n", __func__);
		dev_err(&client->dev, "Cannot register input device\n");
		goto destroy_workqueue_exit;
	}

	data->input = dev;

	err = sysfs_create_group(&data->input->dev.kobj,
			&mc3xxx_attribute_group);
	if (err < 0) {
		MC_ERR_PRINT("%s: cannot create sysfs for mc3xxx\n", __func__);
		dev_err(&client->dev, "Cannot create sysfs for mc3xxx\n");
		goto destroy_workqueue_exit;
	}

	dev_acc = mcube_allocate_device();
	if (!dev_acc) {
		err = -ENOMEM;
		goto remove_sysfs_exit;

	}
	dev_acc->name = "mc3xxx-accel";

	mcube_set_drvdata(dev_acc, data);

	err = mcube_register_device(dev_acc);
	if (err < 0)
		goto remove_sysfs_exit;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mc3xxx_early_suspend;
	data->early_suspend.resume = mc3xxx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	dev_notice(&client->dev, "mc3xxx driver probe successfully\n");
	MC_LOG_PRINT("%s: mc3xxx(0x%x) probe OK\n", __func__,
		data->pcode_data.pcode);

	mc3xxx_pinctrl_state(data, false);
	mc3xxx_power_ctl(data, false);
	return 0;

remove_sysfs_exit:
	sysfs_remove_group(&data->input->dev.kobj,
		&mc3xxx_attribute_group);
destroy_workqueue_exit:
	destroy_workqueue(data->data_wq);
free_irq_exit:
free_interrupt_gpio:
	if (pdata->int_en)
		gpio_free(pdata->gpio_int);
disable_power_exit:
	mc3xxx_power_ctl(data, false);
deinit_power_exit:
	mc3xxx_power_deinit(data);
free_i2c_clientdata_exit:
	i2c_set_clientdata(client, NULL);
pdata_free_exit:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	data->pdata = NULL;
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h)
{
	struct mc3xxx_data *data =
		container_of(h, struct mc3xxx_data, early_suspend);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
		if (!data->pdata->int_en)
			cancel_delayed_work_sync(&data->delay_work);
	}
	mutex_unlock(&data->enable_mutex);
}

static void mc3xxx_late_resume(struct early_suspend *h)
{
	struct mc3xxx_data *data =
		container_of(h, struct mc3xxx_data, early_suspend);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_WAKE);
		if (!data->pdata->int_en)
			queue_delayed_work(data->data_wq,
				&data->delay_work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static int mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	if (data->input)
		sysfs_remove_group(&data->input->dev.kobj,
				&mc3xxx_attribute_group);

	destroy_workqueue(data->data_wq);
	mc3xxx_set_enable(&client->dev, 0);
	mc3xxx_power_deinit(data);
	i2c_set_clientdata(client, NULL);
	if (data->pdata && (client->dev.of_node))
		devm_kfree(&client->dev, data->pdata);
	data->pdata = NULL;

	kfree(data);

	return 0;
}

void mc3xxx_shutdown(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
	mutex_unlock(&data->enable_mutex);
}

#ifdef CONFIG_PM

#else

#define mc3xxx_suspend      NULL
#define mc3xxx_resume       NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id mc3xxx_id[] = {
	{ MC3XXX_DEV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static const struct of_device_id mc3xxx_of_match[] = {
	{ .compatible = COMPATIBLE, },
	{ },
};

static struct i2c_driver mc3xxx_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = MC3XXX_DEV_NAME,
		.of_match_table = mc3xxx_of_match,
	},
	.id_table   = mc3xxx_id,
	.probe      = mc3xxx_probe,
	.remove     = mc3xxx_remove,
	.shutdown   = mc3xxx_shutdown,
};

static int __init mc3xxx_init(void)
{
	MC_FUNC_PRINT(" %s called\n", __func__);
	return i2c_add_driver(&mc3xxx_driver);
}

static void __exit mc3xxx_exit(void)
{
	MC_FUNC_PRINT("%s called\n", __func__);
	i2c_del_driver(&mc3xxx_driver);
}

module_init(mc3xxx_init);
module_exit(mc3xxx_exit);

MODULE_AUTHOR("mCube-inc");
MODULE_DESCRIPTION("MC3XXX ACCELEROMETER SENSOR DRIVER");
MODULE_LICENSE("GPL v2");
