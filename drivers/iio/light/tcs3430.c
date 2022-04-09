/*
 * tcs3430.c - Support for TAOS TCS3430 Color and ALS sensor
 *
 * Copyright (c) 2013 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * TODO: interrupt support, thresholds
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pm.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>

#define TCS3430_DRV_NAME "tcs3430"

#define TCS3430_ENABLE		0x80
#define TCS3430_ATIME		0x81
#define TCS3430_WTIME		0x83
#define TCS3430_AILT		0x84
#define TCS3430_AIHT		0x86
#define TCS3430_PERS		0x8c
#define TCS3430_CFG0		0x8d
#define TCS3430_CFG1		0x90
#define TCS3430_ID		0x92
#define TCS3430_STATUS		0x93

#define TCS3430_CH0DATA		0x94 /* z */
#define TCS3430_CH1DATA		0x96 /* y */
#define TCS3430_CH2DATA		0x98 /* ir1 */
#define TCS3430_CH3DATA		0x9a /* x or ir2 */

#define TCS3430_STATUS_AVALID BIT(0)
#define TCS3430_ENABLE_AEN BIT(1)
#define TCS3430_ENABLE_PON BIT(0)
#define TCS3430_CFG1_AGAIN_MASK (BIT(0) | BIT(1))

struct tcs3430_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 enable;
	u8 cfg1;
	u8 atime;
	u16 buffer[8]; /* 4 16-bit channels + 64-bit timestamp */
};

#define TCS3430_INTENSITY_CHANNEL(_idx, _addr, _mod) { \
	.type = IIO_INTENSITY, \
	.modified = 1, \
	.address = _addr, \
	.channel2 = _mod, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.scan_index = _idx, \
}

static const struct iio_chan_spec tcs3430_channels[] = {
	TCS3430_INTENSITY_CHANNEL(0, TCS3430_CH3DATA, IIO_MOD_X),
	TCS3430_INTENSITY_CHANNEL(1, TCS3430_CH1DATA, IIO_MOD_Y),
	TCS3430_INTENSITY_CHANNEL(2, TCS3430_CH0DATA, IIO_MOD_Z),
	TCS3430_INTENSITY_CHANNEL(3, TCS3430_CH2DATA, IIO_MOD_LIGHT_IR),
	/* illuminance */
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)
				| BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBSCALE)
				| BIT(IIO_CHAN_INFO_INT_TIME),
		.address = TCS3430_CH1DATA,
	},
	IIO_CHAN_SOFT_TIMESTAMP(4),

};

static const int tcs3430_agains[] = { 1, 4, 16, 64 };

static int tcs3430_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct tcs3430_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&data->lock);
		ret = i2c_smbus_read_word_data(data->client, chan->address);
		mutex_unlock(&data->lock);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = tcs3430_agains[data->cfg1 & TCS3430_CFG1_AGAIN_MASK];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		*val2 = (data->atime + 1) * 2780;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static int tcs3430_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct tcs3430_data *data = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val2 != 0)
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(tcs3430_agains); i++) {
			if (val == tcs3430_agains[i]) {
				data->cfg1 &= ~TCS3430_CFG1_AGAIN_MASK;
				data->cfg1 |= i;
				return i2c_smbus_write_byte_data(data->client,
					TCS3430_CFG1, data->cfg1);
			}
		}
		return -EINVAL;
	case IIO_CHAN_INFO_INT_TIME:
		if (val != 0)
			return -EINVAL;
		for (i = 0; i < 256; i++) {
			if (val2 == (i + 1) * 2780) {
				data->atime = i;
				return i2c_smbus_write_word_data(data->client,
					TCS3430_ATIME, data->atime);
			}

		}
		return -EINVAL;
	}
	return -EINVAL;
}

static const struct iio_info tcs3430_info = {
	.read_raw = tcs3430_read_raw,
	.write_raw = tcs3430_write_raw,
	.driver_module = THIS_MODULE,
};

static int tcs3430_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct tcs3430_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &tcs3430_info;
	indio_dev->name = TCS3430_DRV_NAME;
	indio_dev->channels = tcs3430_channels;
	indio_dev->num_channels = ARRAY_SIZE(tcs3430_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = i2c_smbus_read_byte_data(data->client, TCS3430_ID);
	if (ret < 0) {
		dev_err(&client->dev, "read ID failed\n");
		return ret;
	}

	if (ret == 0xdc) {
		dev_info(&client->dev, "TCS3430 found\n");
	}
	else {
		dev_err(&client->dev, "TCS3430 id not found\n");
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(data->client, TCS3430_CFG1);
	if (ret < 0)
		return ret;
	data->cfg1 = ret;

	ret = i2c_smbus_read_byte_data(data->client, TCS3430_ATIME);
	if (ret < 0)
		return ret;
	data->atime = ret;

	ret = i2c_smbus_read_byte_data(data->client, TCS3430_ENABLE);
	if (ret < 0)
		return ret;

	/* enable device */
	data->enable = TCS3430_ENABLE_PON | TCS3430_ENABLE_AEN;
	ret = i2c_smbus_write_byte_data(data->client, TCS3430_ENABLE,
		data->enable);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&data->client->dev, indio_dev);
}

static int tcs3430_powerdown(struct tcs3430_data *data)
{
	return i2c_smbus_write_byte_data(data->client, TCS3430_ENABLE,
		data->enable & ~(TCS3430_ENABLE_AEN | TCS3430_ENABLE_PON));
}

static int tcs3430_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	tcs3430_powerdown(iio_priv(indio_dev));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tcs3430_suspend(struct device *dev)
{
	struct tcs3430_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));

	return tcs3430_powerdown(data);
}

static int tcs3430_resume(struct device *dev)
{
	struct tcs3430_data *data = iio_priv(i2c_get_clientdata(
		to_i2c_client(dev)));

	return i2c_smbus_write_byte_data(data->client, TCS3430_ENABLE,
		data->enable | (TCS3430_ENABLE_AEN | TCS3430_ENABLE_PON));
}
#endif

static SIMPLE_DEV_PM_OPS(tcs3430_pm_ops, tcs3430_suspend, tcs3430_resume);

static const struct i2c_device_id tcs3430_id[] = {
	{ "tcs3430", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcs3430_id);

static struct i2c_driver tcs3430_driver = {
	.driver = {
		.name	= TCS3430_DRV_NAME,
		.pm	= &tcs3430_pm_ops,
	},
	.probe		= tcs3430_probe,
	.remove		= tcs3430_remove,
	.id_table	= tcs3430_id,
};
module_i2c_driver(tcs3430_driver);

MODULE_AUTHOR("Peter Meerwald <pmeerw@pmeerw.net>");
MODULE_AUTHOR("Usha NS <usns@sales.fb.com>");
MODULE_DESCRIPTION("TCS3430 ALS");
MODULE_LICENSE("GPL");
