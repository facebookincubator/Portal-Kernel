/*
 * ptn36502.c
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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/usb/ptn36502.h>

#define USB_US_TX_RX_CONTROL	0x04
#define USB_DS_TX_RX_CONTROL	0x05
#define DP_LINK_CONTROL		0x06
#define DP_LANE0_TX_RX_CONTROL	0x07
#define DP_LANE1_TX_RX_CONTROL	0x08
#define DP_LANE2_TX_RX_CONTROL	0x09
#define DP_LANE3_TX_RX_CONTROL	0x0a
#define MODE_CONTROL1		0x0b
#define DEVICE_CONTROL		0x0d

static struct regmap *ptn_regmap;

static const struct regmap_config ptn36502_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct of_device_id ptn36502_match[] = {
	{ .compatible = "nxp,ptn36502" },
	{},
};
MODULE_DEVICE_TABLE(of, ptn36502_match);

static void ptn36502_init(void)
{
	regmap_write(ptn_regmap, DEVICE_CONTROL, 0x81);
	regmap_write(ptn_regmap, MODE_CONTROL1, 0x08);
	regmap_write(ptn_regmap, DP_LINK_CONTROL, 0x0e);
	regmap_write(ptn_regmap, USB_US_TX_RX_CONTROL, 0x52);
	regmap_write(ptn_regmap, USB_DS_TX_RX_CONTROL, 0x52);
	regmap_write(ptn_regmap, DP_LANE0_TX_RX_CONTROL, 0x29);
	regmap_write(ptn_regmap, DP_LANE1_TX_RX_CONTROL, 0x29);
	regmap_write(ptn_regmap, DP_LANE2_TX_RX_CONTROL, 0x29);
	regmap_write(ptn_regmap, DP_LANE3_TX_RX_CONTROL, 0x29);
}

void ptn36502_setmode(enum mode_select mode, enum orientation ori)
{
	if (!ptn_regmap)
		return;

	ptn36502_init();

	if (ori == 0)
		return;

	pr_debug("[PTN] mode: %d, orienation: %d\n", mode, ori);

	switch (mode) {
	case USB3_ONLY:
		regmap_write(ptn_regmap, MODE_CONTROL1,
				ori == POSITIVE ? 0x01 : 0x21);
		break;
	case DP4_LANE:
		regmap_write(ptn_regmap, MODE_CONTROL1,
				ori == POSITIVE ? 0x08 : 0x28);
		regmap_write(ptn_regmap, MODE_CONTROL1,
				ori == POSITIVE ? 0x0b : 0x2b);
		regmap_write(ptn_regmap, DP_LINK_CONTROL, 0x06);
		break;
	}
}
EXPORT_SYMBOL(ptn36502_setmode);

static int ptn36502_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int err, val;

	dev_dbg(dev, "[PTN] PTN36502 probe\n");

	val = i2c_smbus_read_byte_data(client, 0x0);
	if (val < 0)
		return val;

	if (val != 0x2) {
		dev_err(dev, "Failed to get redriver id: 0x%x\n", val);
		return -ENODEV;
	}

	val = i2c_smbus_read_byte_data(client, 0x1);
	if (val < 0)
		return val;

	if (val < 0x11)
		dev_warn(dev, "Old redriver version detected : 0x%x\n", val);

	ptn_regmap = devm_regmap_init_i2c(client, &ptn36502_regmap);
	if (IS_ERR(ptn_regmap)) {
		err = PTR_ERR(ptn_regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", err);
		return err;
	}

	return 0;
}

static int ptn36502_remove(struct i2c_client *client)
{
	pr_debug("[PTN] PTN36502 remove");
	return 0;
}

static const struct i2c_device_id ptn36502_id[] = {
	{ "ptn36502" },
	{},
};
MODULE_DEVICE_TABLE(i2c, ptn36502_id);

static struct i2c_driver ptn36502_driver = {
	.driver = {
		.name = "ptn36502",
		.of_match_table = of_match_ptr(ptn36502_match),
	},
	.probe = ptn36502_probe,
	.remove = ptn36502_remove,
	.id_table = ptn36502_id,
};

module_i2c_driver(ptn36502_driver);

MODULE_AUTHOR("Jason Hung <jahung@sales.fb.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PTN36502 USB re-driver");
