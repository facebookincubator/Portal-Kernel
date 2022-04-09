/*
 * orient_gpio.c        reports x,y,z axis depending on GPIO
 *
 * Copyright (C) 2017 Facebook Inc.
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
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/iio/iio.h>

#define ORTG_DRV_NAME         "orient_gpio"

enum ortg_orientation {
	ORTG_LANDSCAPE = 0,
	ORTG_PORTRAIT,
	ORTG_MAX_COUNT
};

struct key_data {
	int gpio;
	int orientation_type;
	int orientation;
	int active_low;
	unsigned int debounce;
	unsigned int irq;
	struct delayed_work work;
	void *pdata;
};

struct ortg_data {
	int default_orientation;
	struct platform_device *pdev;
	struct key_data keys[ORTG_MAX_COUNT];
};

static int ortg_gpio_get_state(struct key_data *pkey)
{
	int state = gpio_get_value_cansleep(pkey->gpio);

	if (state < 0)
		return state;

	return (state ? 1 : 0) ^ pkey->active_low;
}

static void ortg_gpio_work_func(struct work_struct *work)
{
	struct key_data *pkey =
			container_of(work, struct key_data, work.work);
	struct ortg_data *data = pkey->pdata;
	struct device *dev = &data->pdev->dev;
	int state = ortg_gpio_get_state(pkey);

	if (state < 0) {
		dev_err(dev, "failed to get gpio state\n");
		return;
	}

	pkey->orientation = state;
}

static irqreturn_t ortg_gpio_isr(int irq, void *priv)
{
	struct key_data *pkey = priv;

	WARN_ON(irq != pkey->irq);

	mod_delayed_work(system_wq,
			 &pkey->work,
			 msecs_to_jiffies(pkey->debounce));

	return IRQ_HANDLED;
}

static int ortg_init_key(struct ortg_data *data, int index,
			 enum of_gpio_flags flags, const char *desc)
{
	struct device *dev = &data->pdev->dev;
	struct key_data *pkey = &data->keys[index];
	unsigned long irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	unsigned int debounce  = 15;
	int state, ret = -1;

	if (!gpio_is_valid(pkey->gpio))
		goto error_ret;

	pkey->pdata = data;
	pkey->orientation_type = index;
	pkey->active_low = flags & OF_GPIO_ACTIVE_LOW;
	state = ortg_gpio_get_state(pkey);
	if (state >= 0)
		pkey->orientation = state;

	ret = gpio_set_debounce(pkey->gpio, debounce * 1000);
	if (ret < 0) {
		dev_err(dev, "Set Debounce failed %d\n", ret);
			pkey->debounce = debounce;
	}

	pkey->irq = gpio_to_irq(pkey->gpio);
	if (pkey->irq < 0) {
		ret = pkey->irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, ret %d\n",
			pkey->gpio, ret);
		goto error_ret;
	}

	INIT_DELAYED_WORK(&pkey->work, ortg_gpio_work_func);

	ret = devm_request_any_context_irq(dev, pkey->irq, ortg_gpio_isr,
					   irqflags, desc, pkey);
	if (ret < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			pkey->irq, ret);
		goto error_ret;
	}

	return 0;
error_ret:
	return ret;
}

static int ortg_get_axis(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan)
{
	struct ortg_data *data = iio_priv(indio_dev);
	int landscape = data->keys[ORTG_LANDSCAPE].orientation;
	int portrait = data->keys[ORTG_PORTRAIT].orientation;
	enum ortg_orientation orientation;
	const int axis[2][3] = {
		{ 0, 9, 0},	/* x, y, z describing landscape */
		{ 9, 0, 0}	/* portrait */
	};

	if (landscape && !portrait)
		orientation = ORTG_LANDSCAPE;
	else if (!landscape && portrait)
		orientation = ORTG_PORTRAIT;
	else {
		orientation = data->default_orientation;
		if (orientation >= ORTG_MAX_COUNT)
			return -EINVAL;
	}

	return axis[orientation][chan->channel2 - 1];
}

static int ortg_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = ortg_get_axis(indio_dev, chan);
		if (ret < 0)
			goto error_ret;
		*val = ret;
		ret = IIO_VAL_INT;
		break;
	}

error_ret:
	return ret;
}

#define ORTG_ACCEL_CHAN(axis)						\
	{								\
		.type = IIO_ACCEL,					\
		.modified = 1,						\
		.channel2 = IIO_MOD_##axis,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
	}

static const struct iio_chan_spec ortg_channels[] = {
	ORTG_ACCEL_CHAN(X), ORTG_ACCEL_CHAN(Y), ORTG_ACCEL_CHAN(Z),
};

static const struct iio_info ortg_info = {
	.read_raw = &ortg_read_raw,
	.driver_module = THIS_MODULE,
};

static int ortg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct ortg_data *data;
	int i, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	indio_dev->channels = ortg_channels;
	indio_dev->num_channels = ARRAY_SIZE(ortg_channels);
	indio_dev->name = ORTG_DRV_NAME;
	indio_dev->info = &ortg_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	platform_set_drvdata(pdev, indio_dev);
	data->pdev = pdev;

	of_property_read_u32(dev->of_node, "default-orientation",
			     &data->default_orientation);

	for (i = 0; i < ORTG_MAX_COUNT; i++) {
		const char *desc = i ? "portrait" : "landscape";
		enum of_gpio_flags flags;

		data->keys[i].gpio = of_get_named_gpio_flags(dev->of_node,
							     desc, 0, &flags);
		ret = ortg_init_key(data, i, flags, desc);
		if (ret < 0) {
			dev_err(dev, "Fail to init key %d\n", i);
			goto error_ret;
		}
	}

	if (data->keys[0].orientation == data->keys[1].orientation
	    && data->default_orientation == ORTG_MAX_COUNT) {
		dev_err(dev, "No/both jumper connencted, and no default " \
			"orientation selected. Exiting without error.\n");
		goto error_ret;
	}

	return iio_device_register(indio_dev);
error_ret:
	return ret;
}

static int ortg_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct platform_device_id ortg_id[] = {
	{"ortg", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, ortg_id);

static const struct of_device_id ortg_of_match[] = {
	{ .compatible = "orient_gpio", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, ortg_of_match);

static struct platform_driver ortg_driver = {
	.driver = {
		.name = "ortg",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ortg_of_match),
	},
	.probe = ortg_probe,
	.remove = ortg_remove,
	.id_table = ortg_id,
};
module_platform_driver(ortg_driver);

MODULE_DESCRIPTION("Orientation GPIO driver");
MODULE_LICENSE("GPL v2");
