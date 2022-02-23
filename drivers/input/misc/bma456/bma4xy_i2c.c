/**
 * @section LICENSE
 * (C) Copyright 2018~2019 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_i2c.c
 * @date     2020/11/10
 * @version  1.3.14
 *
 * @brief    bma4xy I2C bus Driver
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/input.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bma4xy_driver.h"
#include "bs_log.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/
static struct i2c_client *bma4xy_i2c_client;
static int bma4xy_i2c_read(struct i2c_client *client,
	u8 reg_addr, u8 *data, u16 len)
{
	s32 retry;

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};
	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
							BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}

	if (retry >= BMA4XY_MAX_RETRY_I2C_XFER) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

static int bma4xy_i2c_write(struct i2c_client *client,
	u8 reg_addr, const u8 *data, u16 len)
{
	s32 retry;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};
	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		PERR("Allocate mem failed\n");
		return -ENOMEM;
	}
	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);
	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, &msg, 1) > 0)
			break;
		usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
							BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}
	kfree(msg.buf);
	if (retry >= BMA4XY_MAX_RETRY_I2C_XFER) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}


static s8 bma4xy_i2c_read_wrapper(u8 reg_addr, u8 *data, u32 len, void *int_ptr)
{
	int err;

	err = bma4xy_i2c_read(bma4xy_i2c_client, reg_addr, data, len);
	if (!err)
		return 0;
	else
		return -EIO;
}

static s8 bma4xy_i2c_write_wrapper(u8 reg_addr, const u8 *data, u32 len,
	void *int_ptr)
{
	int err;

	err = bma4xy_i2c_write(bma4xy_i2c_client, reg_addr, data, len);
	if (!err)
		return 0;
	else
		return -EIO;
}

static int bma4xy_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;
	u8 dev_id;
	struct bma4xy_client_data *client_data = NULL;

	PDEBUG("entrance");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (bma4xy_i2c_client == NULL) {
		bma4xy_i2c_client = client;
	} else {
		PERR("this driver does not support multiple clients");
		err = -EBUSY;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bma4xy_client_data),
						GFP_KERNEL);
	if (client_data == NULL) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}
	/* h/w init */
	dev_id = BMA4_I2C_INTF;
	client_data->device.intf_ptr = &dev_id;
	client_data->device.variant = BMA45X_VARIANT;
	client_data->device.intf = BMA4_I2C_INTF;
	client_data->device.bus_read = bma4xy_i2c_read_wrapper;
	client_data->device.bus_write = bma4xy_i2c_write_wrapper;
	client_data->device.read_write_len = 4;
	client_data->IRQ = client->irq;
	return bma4xy_probe(client_data, &client->dev);

exit_err_clean:
	if (err)
		bma4xy_i2c_client = NULL;
	return err;
}

static int bma4xy_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = bma4xy_remove(&client->dev);
	bma4xy_i2c_client = NULL;
	return err;
}

static const struct i2c_device_id bma4xy_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma4xy_id);
static const struct of_device_id bma4xy_of_match[] = {
	{ .compatible = "bosch,bmaxxx", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma4xy_of_match);

static SIMPLE_DEV_PM_OPS(bma4xy_pm_ops, bma4xy_suspend, bma4xy_resume);

static struct i2c_driver bma4xy_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "bma456h",
		.of_match_table = bma4xy_of_match,
		.pm = &bma4xy_pm_ops,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bma4xy_id,
	.probe = bma4xy_i2c_probe,
	.remove = bma4xy_i2c_remove,
};

static int __init BMA4xy_init(void)
{
	return i2c_add_driver(&bma4xy_driver);
}

static void __exit BMA4xy_exit(void)
{
	i2c_del_driver(&bma4xy_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA4XY SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMA4xy_init);
module_exit(BMA4xy_exit);
