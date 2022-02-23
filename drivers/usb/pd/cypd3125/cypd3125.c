/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#include "cypd-common.h"
#include "cypd-core.h"
#include "cypd-flash.h"
#include "cypd3125.h"

struct cypd_data *cypd3125_data;

static ssize_t cypd3125_chipid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cypd_data *cypd3125 = dev_get_drvdata(dev);
	CY_RET_STAT stat;
	u8 read_buf[2];
	u16 len = 0;
	u16 silicon_id = 0x00;


	stat = ccg_read(cypd3125, CCG_REG_SIID_LSB, read_buf, sizeof(read_buf));

	if (stat == CY_SUCCESS_RESP) {
		silicon_id = (read_buf[1] << 8 | read_buf[0]);

		CYPD_DEBUG("silicon_id:%x\n", silicon_id);

		len += snprintf(buf + len, PAGE_SIZE - len, "%x\n", silicon_id);
	} else{
		len += snprintf(buf + len, PAGE_SIZE - len, "none\n");
	}

	return len;

}

static DEVICE_ATTR(chipid, S_IRUGO, cypd3125_chipid_show, NULL);

static struct attribute *cypd3215_attributes[] = {
	&dev_attr_chipid.attr,
	NULL
};

static struct attribute_group cypd3125_attribute_group = {
	.attrs = cypd3215_attributes
};

static int cypd_get_fw_info(struct cypd_data *cypd_data)
{
	CY_RET_STAT stat;
	u8 buf[64];
	u16 len = 2;
	u16 silicon_id = 0x00;

	stat = ccg_read(cypd_data, CCG_REG_SIID_LSB, buf, len);

	if (stat == CY_SUCCESS_RESP) {
		silicon_id = (buf[1] << 8 | buf[0]);

		CYPD_DEBUG("silicon_id:%x", silicon_id);

		if (silicon_id == CYPD3125_CHIP_ID)
			return CY_SUCCESS_RESP;
		else
			return CY_ERROR_SILICON_ID_MISMATCH;
	}

	CYPD_ERROR("CY_ERROR_SILICON_ID_MISMATCH");

	return CY_ERROR_SILICON_ID_MISMATCH;
}

static int cypd3125_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct cypd_platform_data *pdata;
	struct cypd_data *cypd_data;
	struct device *dev = &client->dev;

	pr_err("%s: enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check_functionality failed\n");
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			pr_err("Failed to allocate memory for platform data");
			return -ENOMEM;
		}

	} else {
		pr_err("!client->dev.of_node");
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		pr_err("no ts platform data found");
		return -EINVAL;
	}

	cypd_data = devm_kzalloc(&client->dev, sizeof(*cypd_data), GFP_KERNEL);
	if (!cypd_data)
		return -ENOMEM;

	cypd3125_data = cypd_data;
	cypd_data->dev = dev;
	cypd_data->client = client;
	mutex_init(&cypd_data->lock);
	cypd_data->pdata = pdata;
	i2c_set_clientdata(client, cypd_data);

	dev_set_drvdata(&client->dev, cypd_data);

	cypd_data->cypd_workqueue = create_singlethread_workqueue("cypd_wq");
	if (cypd_data->cypd_workqueue == NULL)
		pr_err("failed to create cs workqueue");

	spin_lock_init(&cypd_data->irq_lock);

	ret = cypd_get_fw_info(cypd_data);
	if (ret != CY_SUCCESS_RESP) {
		dev_err(cypd_data->dev, "get_fw_info failed - %d\n", ret);
		return ret;
	}

	ret = cypd_create_sysfs(client, &cypd3125_attribute_group);
	if (ret)
		pr_err("create sysfs node fail");

#ifdef SC_AUTO_UPGRADE_EN
	ret = cypd_fwupg_init(cypd_data);
	if (ret)
		pr_err("init fw upgrade fail");
#endif
	pr_info("%s probe completed successfully!\n", __func__);
	return 0;

}

static int cypd3125_remove(struct i2c_client *client)
{
	struct cypd_data *cypd_data = i2c_get_clientdata(client);

	pr_info("%s: enter\n", __func__);

	cypd_remove_sysfs(client, &cypd3125_attribute_group);

	free_irq(cypd_data->irq, cypd_data);

	if (gpio_is_valid(cypd_data->pdata->sw_gpio))
		gpio_free(cypd_data->pdata->sw_gpio);

	if (gpio_is_valid(cypd_data->pdata->irq_gpio))
		gpio_free(cypd_data->pdata->irq_gpio);

	if (cypd_data->cypd_workqueue)
		destroy_workqueue(cypd_data->cypd_workqueue);

	devm_kfree(&client->dev, cypd_data);
	cypd3125_data = NULL;

	return 0;
}

static const struct i2c_device_id cypd_i2c_id[] = {
	{CYPD3125_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cypd_i2c_id);

static const struct of_device_id of_cypd_match[] = {
	{.compatible = "Cypress,cypd3125"},
	{},
};
MODULE_DEVICE_TABLE(of, of_cypd_match);

static struct i2c_driver cypd_driver = {
	.driver = {
		.name = CYPD3125_I2C_NAME,
		.of_match_table = of_cypd_match,
	},
	.probe = cypd3125_i2c_probe,
	.remove = cypd3125_remove,
	.id_table = cypd_i2c_id,
};
module_i2c_driver(cypd_driver);

MODULE_DESCRIPTION("CYPD3125 Driver");
MODULE_LICENSE("GPL v2");

