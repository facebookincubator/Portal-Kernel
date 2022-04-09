/*
 * This file is part of the AL3201 sensor driver.
 *
 * Copyright (c) 2011 Liteon-semi Corporation
 *
 * Contact: YC Hou <yc_hou@liteon-semi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: al3201.c
 *
 * Summary:
 *	AL3201 sensor dirver for kernel version 2.6.36/4.4
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 06/28/11 YC		 Original Creation (Test version:1.0)
 * 06/28/11 YC       Change dev name to dyna for demo purpose (ver 1.5).
 * 08/29/11 YC       Add engineer mode. Change version to 1.6.
 * 09/26/11 YC       Add calibration compensation function and add not power up
 *                   prompt. Change version to 1.7.
 * 03/06/17 KP       Rework to build on kernel version 4.4 and change version to 1.8
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define AL3201_DRV_NAME		"al3201"
#define DRIVER_VERSION		"1.8"

#define AL3201_NUM_CACHABLE_REGS	5

#define AL3201_RAN_COMMAND	0x01
#define AL3201_RAN_MASK		0x01
#define AL3201_RAN_SHIFT	(0)

#define AL3201_POW_COMMAND	0x00
#define AL3201_POW_MASK		0x03
#define AL3201_POW_UP		0x03
#define AL3201_POW_DOWN		0x00
#define AL3201_POW_SHIFT	(0)

#define	AL3201_ADC_LSB		0x0c
#define	AL3201_ADC_MSB		0x0d

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif

static u8 al3201_reg[AL3201_NUM_CACHABLE_REGS] =
	{0x00,0x01,0x02,0x0c,0x0d};

static int al3201_range[2] = {32768,8192};

struct al3201_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[AL3201_NUM_CACHABLE_REGS];
};

static int cali = 100;


#define AL3201_SCALE_AVAILABLE "0.032 0.008"

static const int al3201_scales[][2] = {
        {0, 32768}, {0, 8192}
};

/*
 * register access helpers
 */
static int __al3201_read_reg(struct al3201_data *data,
		u32 reg, u8 mask, u8 shift)
{
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __al3201_write_reg(struct al3201_data *data,
		u32 reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0;
	u8 tmp;

	if (reg >= AL3201_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(data->client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3201_get_range(struct al3201_data *data)
{
	int tmp;
	tmp = __al3201_read_reg(data, AL3201_RAN_COMMAND,
			AL3201_RAN_MASK, AL3201_RAN_SHIFT);;
	return al3201_range[tmp];
}

static int al3201_set_range(struct al3201_data *data, int range)
{
	return __al3201_write_reg(data, AL3201_RAN_COMMAND,
			AL3201_RAN_MASK, AL3201_RAN_SHIFT, range);
}

/* resolution */
/*
static int al3201_get_resolution(struct al3201_data *data)
{
	return (al3201_get_range(data)) ? 2:8;
}
*/

/* power_state */
static int al3201_set_power_state(struct al3201_data *data, int state)
{
	return __al3201_write_reg(data, AL3201_POW_COMMAND,
			AL3201_POW_MASK, AL3201_POW_SHIFT,
			state ? AL3201_POW_UP : AL3201_POW_DOWN);
}
/*
static int al3201_get_power_state(struct al3201_data *data)
{
	u8 cmdreg = data->reg_cache[AL3201_POW_COMMAND];
	return (cmdreg & AL3201_POW_MASK) >> AL3201_POW_SHIFT;
}
*/

static int al3201_get_adc_value(struct al3201_data *data)
{
	int lsb, msb, range;
	u32 val;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(data->client, AL3201_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(data->client, AL3201_ADC_MSB);

	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = al3201_get_range(data);

	val = (u32)((msb << 8 | lsb) >> (range == 32768 ? 1:3));
	val *= cali;

	return (val / 100);
}

static const struct iio_chan_spec al3201_channels[] = {
        {
                .type   = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				/*| BIT(IIO_CHAN_INFO_SCALE) */
				| BIT(IIO_CHAN_INFO_PROCESSED),
        }
};

/* calibration */
static ssize_t al3201_show_calibration(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", cali);
}

static ssize_t al3201_store_calibration(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct al3201_data *data = iio_priv(indio_dev);
	int stdls, lux;
	char tmp[10] = {0};
	int n = sscanf(buf, "%d %9s", &stdls, tmp);

	if (n < 1 || n > 2)
		return -EINVAL;

	if (!strncmp(tmp, "-setcv", 6)) {
		cali = stdls;
		return count;
	}

	if (stdls < 0) {
		dev_err(dev, "Std light source: [%d] < 0\n", stdls);
		return -EINVAL;
	}

	lux = al3201_get_adc_value(data);
	cali = stdls * 100 / lux;

	return count;
}

static DEVICE_ATTR(in_illuminance_calibration, S_IRUGO | S_IWUSR,
		   al3201_show_calibration, al3201_store_calibration);

static IIO_CONST_ATTR(in_illuminance_scale_available, AL3201_SCALE_AVAILABLE);

static struct attribute *al3201_attributes[] = {
	&dev_attr_in_illuminance_calibration.attr,
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group al3201_attribute_group = {
        .attrs = al3201_attributes,
};

static int al3201_read_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan, int *val,
                            int *val2, long mask)
{
        struct al3201_data *data = iio_priv(indio_dev);
        int ret;

        switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		*val = al3201_get_adc_value(data);
		return IIO_VAL_INT;
        case IIO_CHAN_INFO_RAW:
                /*
                 * ALS ADC value is stored in two adjacent registers:
                 * - low byte of output is stored at AL3201_ADC_LSB
                 * - high byte of output is stored at AL3201_ADC_LSB + 1
                 */
                ret = i2c_smbus_read_word_data(data->client,
                                               AL3201_ADC_LSB);
                if (ret < 0)
                        return ret;
                *val = ret;
                return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = al3201_get_range(data);
                return IIO_VAL_INT_PLUS_MICRO;
        }
        return -EINVAL;
}


static int al3201_write_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan, int val,
                             int val2, long mask)
{
        struct al3201_data *data = iio_priv(indio_dev);
        int i;

        switch (mask) {
        case IIO_CHAN_INFO_SCALE:
                for (i = 0; i < ARRAY_SIZE(al3201_scales); i++) {
                        if (val == al3201_scales[i][0] &&
                            val2 == al3201_scales[i][1])
				return al3201_set_range(data, val2);
                }
                break;
        }

        return -EINVAL;
}

static int al3201_init(struct al3201_data *data)
{
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(data->client, al3201_reg[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	al3201_set_range(data, 0);
	al3201_set_power_state(data, 1);
	return 0;
}

static const struct iio_info al3201_info = {
        .driver_module  = THIS_MODULE,
        .read_raw       = al3201_read_raw,
        .write_raw      = al3201_write_raw,
        .attrs          = &al3201_attribute_group,
};

/*
 * I2C layer
 */
static int al3201_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct al3201_data *data;
	struct iio_dev *indio_dev;
	int ret;


	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	mutex_init(&data->lock);

        indio_dev->dev.parent = &client->dev;
        indio_dev->info = &al3201_info;
        indio_dev->name = AL3201_DRV_NAME;
        indio_dev->channels = al3201_channels;
        indio_dev->num_channels = ARRAY_SIZE(al3201_channels);
        indio_dev->modes = INDIO_DIRECT_MODE;

        ret = al3201_init(data);
        if (ret < 0) {
                dev_err(&client->dev, "al3201 chip init failed\n");
                return ret;
        }
        return devm_iio_device_register(&client->dev, indio_dev);
}

static int al3201_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct al3201_data *data = iio_priv(indio_dev);

	al3201_set_power_state(data, 0);
	return 0;
}

static const struct i2c_device_id al3201_id[] = {
	{ "al3201", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3201_id);

static const struct of_device_id al3201_of_match[] = {
        { .compatible = "al3201", .data = NULL, },
        { }
};
MODULE_DEVICE_TABLE(of, al3201_of_match);

static struct i2c_driver al3201_driver = {
	.driver = {
		.name	= AL3201_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(al3201_of_match),
	},
	.probe	= al3201_probe,
	.remove	= al3201_remove,
	.id_table = al3201_id,
};

module_i2c_driver(al3201_driver);

MODULE_AUTHOR("YC Hou, LiteOn-semi corporation.");
MODULE_DESCRIPTION("AL3201 light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

