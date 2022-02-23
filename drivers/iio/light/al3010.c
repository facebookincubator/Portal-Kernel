 /* AL3010 - Dyna Image Ambient Light Sensor
 *
 * Copyright (c) 2014, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for AL3010 (7-bit I2C slave address 0x1C).
 *
 * TODO: interrupt support, thresholds
 *
 * This driver is base on the original driver for AL3010 that was distributed
 * by Intel Corporation.
 *
 * last change: 2016/06/03
 * editor: Kaku Kuo <kaku.kuo@dyan-image.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AL3010_DRV_NAME "al3010"

#define AL3010_REG_SYSTEM			0x00
#define	AL3010_REG_CONFIG		0x10
#define	AL3010_REG_DATA_LOW	0x0c

#define	AL3010_GAIN_MASK	(BIT(6) | BIT(5) | BIT(4))
#define	AL3010_GAIN_SHIFT	4

#define AL3010_CONFIG_DISABLE	0x00
#define AL3010_CONFIG_ENABLE	0x01

#define AL3010_SCALE_AVAILABLE "1.1872 0.2968 0.0742 0.0186"

enum al3010_range {
	AL3010_RANGE_1, /* 77806 lx */
	AL3010_RANGE_2, /* 19452 lx  */
	AL3010_RANGE_3, /* 4863  lx  */
	AL3010_RANGE_4  /* 1216  lx  */
};

static const int al3010_scales[][2] = {
	{0, 1187200}, {0, 296800}, {0, 74200}, {0, 18600}
};

struct al3010_data {
	struct i2c_client *client;
    struct regulator *vreg;
};

/* Set the default calibration factor to be a reasonable value for all skews.
 * This will be overridden by calibration done in factory.
 */
static int al3010_cali = 1300;

/*
 * set sensor range to 1216 lux as the device is primarily
 * meant for indoor use.
 */
static int al3010_range = AL3010_RANGE_4;

static const struct iio_chan_spec al3010_channels[] = {
	{
		.type	= IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int al3010_set_gain(struct al3010_data *data, int gain)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, AL3010_REG_CONFIG,
		(gain<<AL3010_GAIN_SHIFT)&AL3010_GAIN_MASK);

	return ret;
}

static int al3010_set_mode(struct al3010_data *data, int mode)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, AL3010_REG_SYSTEM, mode);

	return ret;
}

static int al3010_get_mode(struct al3010_data *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(data->client, AL3010_REG_SYSTEM);

	return ret;
}

static int al3010_get_adc_value(struct al3010_data *data)
{
	int ret;

	ret = i2c_smbus_read_word_data(data->client, AL3010_REG_DATA_LOW);

	return ret;
}

static int al3010_get_lux(struct al3010_data *data)
{
	int ret;
	long int ret64;

	ret = al3010_get_adc_value(data);
	ret64 = ret;
	ret = (ret64 * (al3010_scales[al3010_range][1]/100) *
		al3010_cali) / 1000000;

	return ret;
}

/* lux */
static ssize_t al3010_show_lux(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct al3010_data *data = iio_priv(dev_to_iio_dev(dev));
	int ret;

	/* No LUX data if not operational */
	if (al3010_get_mode(data) != AL3010_CONFIG_ENABLE)
		return -EBUSY;

	ret = al3010_get_lux(data);

	return snprintf(buf, 10, "%d\n", ret);
}

/* calibration */
static ssize_t al3010_show_calibration(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", al3010_cali);
}

static ssize_t al3010_store_calibration(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct al3010_data *data = iio_priv(indio_dev);
	int stdls, lux;
	char tmp[10] = {0};
	int n = sscanf(buf, "%d %9s", &stdls, tmp);

	if (n < 1 || n > 2)
		return -EINVAL;

	if (!strncmp(tmp, "-setcv", 6)) {
		al3010_cali = stdls;
		return count;
	}

	if (stdls < 0) {
		dev_err(dev, "Std light source: [%d] < 0\n", stdls);
		return -EINVAL;
	}

	lux = al3010_get_lux(data);
	al3010_cali = stdls * 100 / lux;

	return count;
}

static IIO_CONST_ATTR(in_illuminance_scale_available, AL3010_SCALE_AVAILABLE);

static DEVICE_ATTR(in_illuminance_input, S_IRUGO, al3010_show_lux, NULL);

static DEVICE_ATTR(in_illuminance_calibration, S_IRUGO | S_IWUSR,
		   al3010_show_calibration, al3010_store_calibration);

static struct attribute *al3010_attributes[] = {
	&dev_attr_in_illuminance_input.attr,
	&dev_attr_in_illuminance_calibration.attr,
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group al3010_attribute_group = {
	.attrs = al3010_attributes,
};

static int al3010_init(struct al3010_data *data)
{
	int err = 0;

	err = al3010_set_mode(data, AL3010_CONFIG_ENABLE);
	if (err) {
		dev_err(&data->client->dev,
			"%s: al3010_set_mode returned error %d\n",
			__func__, err);
		return err;
	}

	err = al3010_set_gain(data, al3010_range);
	if (err) {
		dev_err(&data->client->dev,
			"%s: al3010_set_range returned error %d\n",
			__func__, err);
		return err;
	}

	return 0;
}

static int al3010_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val,
		int *val2, long mask)
{
	struct al3010_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:

		ret = i2c_smbus_read_word_data(data->client,
				AL3010_REG_DATA_LOW);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = i2c_smbus_read_byte_data(data->client,
				AL3010_REG_CONFIG);
		if (ret < 0)
			return ret;

		ret = (ret & AL3010_GAIN_MASK) >> AL3010_GAIN_SHIFT;
		if (ret >= ARRAY_SIZE(al3010_scales))
			return -EINVAL;
		*val = al3010_scales[ret][0];
		*val2 = al3010_scales[ret][1];

		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static int al3010_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val,
		int val2, long mask)
{
	struct al3010_data *data = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		for (i = 0; i < ARRAY_SIZE(al3010_scales); i++) {
			if (val == al3010_scales[i][0] &&
			    val2 == al3010_scales[i][1])
				return al3010_set_gain(data, i);
		}
		break;
	}
	return -EINVAL;
}

static const struct iio_info al3010_info = {
	.driver_module	= THIS_MODULE,
	.read_raw	= al3010_read_raw,
	.write_raw	= al3010_write_raw,
	.attrs		= &al3010_attribute_group,
};

static int al3010_get_dt_supply(struct device *dev,
		struct al3010_data *pdata)
{
    int ret = 0;

	if (!dev || !pdata) {
		pr_err("invalid input param dev:%pK pdata:%pK\n", dev, pdata);
		return -EINVAL;
	}

    pdata->vreg = regulator_get(dev, "vdd");
    if (IS_ERR(pdata->vreg)) {
        pr_err("regulator get failed\n");
        pdata->vreg = NULL;
        goto end;
    }

    ret = regulator_enable(pdata->vreg);
    if (ret) {
        pr_err("Failed to enable VDD supply\n");
    }

end:
    return ret;
}

static int al3010_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct al3010_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &al3010_info;
	indio_dev->name = AL3010_DRV_NAME;
	indio_dev->channels = al3010_channels;
	indio_dev->num_channels = ARRAY_SIZE(al3010_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = al3010_get_dt_supply(&client->dev, data);
	if (ret) {
		pr_err("failed to get dt supply\n");
		goto err_dt_parse;
	}

    //Wait for power stable
    msleep(50);

	ret = al3010_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "al3010 chip init failed\n");
		return ret;
	}

	return devm_iio_device_register(&client->dev, indio_dev);

err_dt_parse:
    devm_iio_device_free(&client->dev, indio_dev);

	return ret;
}

static int al3010_remove(struct i2c_client *client)
{
	return i2c_smbus_write_byte_data(client, AL3010_REG_SYSTEM, 0x00);
}

static const struct i2c_device_id al3010_id[] = {
	{"al3010", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static const struct of_device_id al3010_of_match[] = {
	{ .compatible = "lsc,al3010" },
	{ },
};
MODULE_DEVICE_TABLE(of, al3010_of_match);

static struct i2c_driver al3010_driver = {
	.driver = {
		.name = AL3010_DRV_NAME,
		.of_match_table = of_match_ptr(al3010_of_match),
	},
	.probe		= al3010_probe,
	.remove		= al3010_remove,
	.id_table	= al3010_id,
};

module_i2c_driver(al3010_driver);

MODULE_AUTHOR("Kaku Kuo <kaku.kuo@dyna-image.com");
MODULE_DESCRIPTION("AL3010 Ambient Light Sensor driver");
MODULE_LICENSE("GPL v2");
