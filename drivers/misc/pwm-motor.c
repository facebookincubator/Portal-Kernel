/*
 * pwm-motor.c - Driver for motors connected to PWM lines.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * Author: Kamil Debski <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/qpnp/pwm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define MAX_PWM 255

struct pwm_motor_ctx {
	struct mutex lock;
	struct pwm_device *pwm;
	unsigned int pwm_value;
	int sleep_gpio;
	int ph_gpio;
	int sleep_status;
	int ph_status;
};

static int  __set_pwm(struct pwm_motor_ctx *ctx, unsigned long pwm)
{
	unsigned long duty;
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (ctx->pwm_value == pwm)
		goto exit_set_pwm_err;

	duty = pwm * ctx->pwm->period / MAX_PWM;
	ret = pwm_config(ctx->pwm, duty, ctx->pwm->period);

	if (ret)
		goto exit_set_pwm_err;

	if (pwm == 0)
		pwm_disable(ctx->pwm);

	if (ctx->pwm_value == 0) {
		ret = pwm_enable(ctx->pwm);
		if (ret)
			goto exit_set_pwm_err;
	}

	ctx->pwm_value = pwm;

exit_set_pwm_err:
	mutex_unlock(&ctx->lock);
	return ret;
}


static ssize_t set_ph_gpio(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);
	unsigned long ph_value;

	if (kstrtoul(buf, 10, &ph_value) || ph_value > 2 || ph_value < 0)
		return -EINVAL;

	gpio_set_value(ctx->ph_gpio, ph_value);

	ctx->ph_status = ph_value;

	return count;
}

static ssize_t show_ph_gpio(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ctx->ph_status);
}


static ssize_t set_sleep_gpio(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);
	unsigned long sleep_value;

	if (kstrtoul(buf, 10, &sleep_value) ||
	    sleep_value > 2 || sleep_value < 0)
		return -EINVAL;

	gpio_set_value(ctx->sleep_gpio, sleep_value);

	ctx->sleep_status = sleep_value;

	return count;
}

static ssize_t show_sleep_gpio(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ctx->sleep_status);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;
	int ret;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_PWM)
		return -EINVAL;

	ret = __set_pwm(ctx, pwm);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_pwm(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ctx->pwm_value);
}


static DEVICE_ATTR(ph_gpio, S_IRUGO | S_IWUSR, show_ph_gpio, set_ph_gpio);
static DEVICE_ATTR(sleep_gpio, S_IRUGO | S_IWUSR, show_sleep_gpio, set_sleep_gpio);
static DEVICE_ATTR(pwm1, S_IRUGO | S_IWUSR, show_pwm, set_pwm);

static struct attribute *dev_ph_attrs[] = {
	&dev_attr_ph_gpio.attr,
	NULL,
};

static struct attribute_group dev_ph_attr_grp = {
	.attrs = dev_ph_attrs,
};

static struct attribute *dev_sleep_attrs[] = {
	&dev_attr_sleep_gpio.attr,
	NULL,
};

static struct attribute_group dev_sleep_attr_grp = {
	.attrs = dev_sleep_attrs,
};


static struct attribute *dev_pwm1_attrs[] = {
	&dev_attr_pwm1.attr,
	NULL,
};

static struct attribute_group dev_pwm1_attr_grp = {
	.attrs = dev_pwm1_attrs,
};

static int motor_gpio_configure(struct pwm_motor_ctx *ctx)
{
	int err = 0;

	/* request sleep gpio */
	if (gpio_is_valid(ctx->sleep_gpio)) {
		err = gpio_request(ctx->sleep_gpio, "sleep gpio");
		if (err) {
			pr_err("Failed to request sleep gpio\n");
			goto err;
		}
		err = gpio_direction_output(ctx->sleep_gpio, 0);
		if (err) {
			pr_err("set_direction for sleep gpio failed");
			goto err;
		}
	}

	/* request ph gpio */
	if (gpio_is_valid(ctx->ph_gpio)) {
		err = gpio_request(ctx->ph_gpio, "ph gpio");
		if (err) {
			pr_err("Failed to request ph gpio");
			goto err;
		}
		err = gpio_direction_output(ctx->ph_gpio, 0);
		if (err) {
			pr_err("set_direction for ph gpio failed");
			goto err;
		}
	}

	return 0;

err:
	if (gpio_is_valid(ctx->ph_gpio))
		gpio_free(ctx->ph_gpio);

	if (gpio_is_valid(ctx->sleep_gpio))
		gpio_free(ctx->sleep_gpio);

	return err;
}

static int pwm_motor_probe(struct platform_device *pdev)
{
	struct pwm_motor_ctx *ctx;
	int duty_cycle = 0;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	ctx->pwm = devm_of_pwm_get(&pdev->dev, pdev->dev.of_node, NULL);
	if (IS_ERR(ctx->pwm)) {
		pr_err("Could not get PWM\n");
		return PTR_ERR(ctx->pwm);
	}

	platform_set_drvdata(pdev, ctx);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "qcom,pwm-period", &ctx->pwm->period);
	if (ret)
		ctx->pwm->period = 0;

	ctx->sleep_gpio = of_get_named_gpio(pdev->dev.of_node, "sleep-gpio", 0);
	if (ctx->sleep_gpio < 0) {
		pr_err("Failed to get sleep gpio\n");
		return -EINVAL;
	}

	ctx->ph_gpio = of_get_named_gpio(pdev->dev.of_node, "ph-gpio", 0);
	if (ctx->ph_gpio < 0) {
		pr_err("Failed to get ph gpio\n");
		return -EINVAL;
	}

	ret = motor_gpio_configure(ctx);
	if (ret < 0) {
		pr_err("Failed to configure gpio\n");
		return ret;
	}

	ctx->pwm_value = 0;
	ctx->sleep_status = 0;
	ctx->ph_status = 0;

	ret = pwm_change_mode(ctx->pwm, PM_PWM_MODE_PWM);
	if (ret < 0) {
		pr_err("Failed to set PWM mode, ret = %d\n", ret);
		return ret;
	}

	ret = pwm_config(ctx->pwm, duty_cycle, ctx->pwm->period);
	if (ret) {
		pr_err("Failed to configure PWM %d\n", ret);
		return ret;
	}

	/* Enbale PWM output */
	ret = pwm_enable(ctx->pwm);
	if (ret) {
		pr_err("Failed to enable PWM\n");
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_pwm1_attr_grp);
	if (ret) {
		pr_err("Failed to create pwm1 sysfs\n");
		goto sysfs_err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_sleep_attr_grp);
	if (ret) {
		pr_err("Failed to create sleep sysfs\n");
		goto sysfs_err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_ph_attr_grp);
	if (ret) {
		pr_err("Failed to create ph sysfs\n");
		goto sysfs_err;
	}

	return 0;

sysfs_err:
	pwm_disable(ctx->pwm);
	return ret;
}

static int pwm_motor_remove(struct platform_device *pdev)
{
	struct pwm_motor_ctx *ctx = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &dev_pwm1_attr_grp);
	sysfs_remove_group(&pdev->dev.kobj, &dev_sleep_attr_grp);
	sysfs_remove_group(&pdev->dev.kobj, &dev_ph_attr_grp);

	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_motor_suspend(struct device *dev)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);

	if (ctx->pwm_value)
		pwm_disable(ctx->pwm);

	return 0;
}

static int pwm_motor_resume(struct device *dev)
{
	struct pwm_motor_ctx *ctx = dev_get_drvdata(dev);
	unsigned long duty;
	int ret;

	if (ctx->pwm_value == 0)
		return 0;

	duty = ctx->pwm_value * ctx->pwm->period / MAX_PWM;
	ret = pwm_config(ctx->pwm, duty, ctx->pwm->period);
	if (ret)
		return ret;

	return pwm_enable(ctx->pwm);
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_motor_pm, pwm_motor_suspend, pwm_motor_resume);

static const struct of_device_id of_pwm_motor_match[] = {
	{ .compatible = "pwm-motor", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_motor_match);

static struct platform_driver pwm_motor_driver = {
	.probe		= pwm_motor_probe,
	.remove		= pwm_motor_remove,
	.driver	= {
		.name		= "pwm-motor",
		.pm		= &pwm_motor_pm,
		.of_match_table	= of_pwm_motor_match,
	},
};

module_platform_driver(pwm_motor_driver);

MODULE_AUTHOR("Kamil Debski <k.debski@samsung.com>");
MODULE_AUTHOR("Kevin Wei <kewei@sales.fb.com>");
MODULE_ALIAS("platform:pwm-motor");
MODULE_DESCRIPTION("PWM MOTOR driver");
MODULE_LICENSE("GPL");
