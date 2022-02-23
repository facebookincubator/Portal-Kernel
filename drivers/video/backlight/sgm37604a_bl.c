/*
 * SGM7604a backlight LED driver
 *
 * Copyright 2019 Facebook Ltd.
 *
 * Author: Konstantin Buhchev <kbuhchev@fb.com>
 *         Barry Yang <yangjiangzhu@longcheer.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>

#define MAX_BRIGHTNESS				0xFFF
#define MAX_USER_BRIGHTNESS			0xFF

/* sgm37604a Register Map */
#define SGM37604A_CTL_BACKLIGHT_RESET_REG	0x01
#define SGM37604A_CTL_ENABLE_REG		0x10
#define SGM37604A_CTL_BACKLIGHT_MODE_REG	0x11
#define SGM37604A_CTL_BRIGHTNESS_MSB_REG	0x19
#define SGM37604A_CTL_BRIGHTNESS_LSB_REG	0x1A
#define SGM37604A_CTL_BACKLIGHT_CURRENT_REG	0x1B

#define SGM37604A_MODE_REGISTER_MASK		0x70
#define SGM37604A_MODE_REGISTER_CONTROL		0x00
#define SGM37604A_MODE_PWM_CONTROL		0x20
#define SGM37604A_MODE_COMBINED_CONTROL		0x40

#define SGM37604A_MODE_DEVICE_EN		0x01
#define SGM37604A_MODE_LEDS_EN			0x1E

static int sgm37604a_backlight_set_mode(struct i2c_client *client, int mode)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client,
				       SGM37604A_CTL_BACKLIGHT_MODE_REG);
	if (ret < 0) {
		dev_err(&client->dev, "Read control reg failed : %d\n", ret);
		return ret;
	}

	if (!((mode ^ ret) & SGM37604A_MODE_REGISTER_MASK))
		/* Bootloader has already initialized backlight. */
		return 0;

	/*
	 * According to SGM37604A datasheet:
	 * For glitch free operation, Brightness Contror register(0x11)
	 * should only be programmed while register Enable(0x10) is 0x01.
	 */
	ret = i2c_smbus_write_byte_data(client, SGM37604A_CTL_ENABLE_REG,
						SGM37604A_MODE_DEVICE_EN);
	if (ret) {
		dev_err(&client->dev, "Disable LEDs failed : %d\n", ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client,
					SGM37604A_CTL_BACKLIGHT_MODE_REG,
					mode & SGM37604A_MODE_REGISTER_MASK);
	if (ret) {
		dev_err(&client->dev, "Set mode failed : %d\n", ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, SGM37604A_CTL_ENABLE_REG,
			SGM37604A_MODE_LEDS_EN | SGM37604A_MODE_DEVICE_EN);
	if (ret) {
		dev_err(&client->dev, "Enable LEDs failed : %d\n", ret);
		return ret;
	}

	return ret;
}

static int sgm37604a_set_brightness(struct backlight_device *bl)
{
	int ret = 0;
	struct i2c_client *client = bl->dev.platform_data;
	u32 level = min(bl->props.brightness, MAX_USER_BRIGHTNESS)
				* MAX_BRIGHTNESS / MAX_USER_BRIGHTNESS;

	ret = i2c_smbus_write_byte_data(client,
			SGM37604A_CTL_BRIGHTNESS_LSB_REG, level & 0x0F);
	if (ret) {
		dev_err(&client->dev, "Set brigtness LSB failed : %d\n", ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client,
				SGM37604A_CTL_BRIGHTNESS_MSB_REG, level >> 4);
	if (ret)
		dev_err(&client->dev, "Set brigtness MSB failed : %d\n", ret);

	return ret;
}

static int sgm37604a_get_brightness(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops sgm37604a_ops = {
	.update_status = sgm37604a_set_brightness,
	.get_brightness = sgm37604a_get_brightness
};

static int sgm37604a_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret;
	struct backlight_device *bl;
	struct backlight_properties props;

	ret = sgm37604a_backlight_set_mode(client,
					   SGM37604A_MODE_REGISTER_CONTROL);
	if (ret) {
		dev_err(&client->dev, "Init failed, err : %d\n", ret);
		return ret;
	}

	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_USER_BRIGHTNESS;

	bl = devm_backlight_device_register(&client->dev, "sgm37604a",
						&client->dev, client,
						&sgm37604a_ops, &props);

	if (IS_ERR_OR_NULL(bl)) {
		dev_err(&client->dev, "Failed to register backlight: %ld\n",
								PTR_ERR(bl));
		return -ENODEV;
	}

	bl->dev.platform_data = client;

	return 0;
}

static int sgm37604a_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sgm37604a_i2c_id[] = {
	{ "sgm37604a", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm37604a_i2c_id);

static struct i2c_driver sgm37604a_i2c_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = "sgm37604a",
		   },

	.id_table = sgm37604a_i2c_id,

	.probe = sgm37604a_probe,
	.remove = sgm37604a_remove,
};

module_i2c_driver(sgm37604a_i2c_driver);

MODULE_AUTHOR("Konstantin Buhchev <kbuhchev@fb.com>");
MODULE_DESCRIPTION("SGM37604a Driver");
MODULE_LICENSE("GPL");
