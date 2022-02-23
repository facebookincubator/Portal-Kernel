/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#include "cypd-core.h"
#include "cypd-common.h"

int ccg_read(struct cypd_data *uc, u16 rab, u8 *data, u32 len)
{
	struct i2c_client *client = uc->client;
	const struct i2c_adapter_quirks *quirks = client->adapter->quirks;
	unsigned char buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0x0,
			.len = sizeof(buf),
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = data,
		},
	};
	u32 rlen, rem_len = len, max_read_len = len;
	int status;

	/* check any max_read_len limitation on i2c adapter */
	if (quirks && quirks->max_read_len)
		max_read_len = quirks->max_read_len;

	pm_runtime_get_sync(uc->dev);
	while (rem_len > 0) {
		msgs[1].buf = &data[len - rem_len];
		rlen = min_t(u16, rem_len, max_read_len);
		msgs[1].len = rlen;
		put_unaligned_le16(rab, buf);
		status = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (status < 0) {
			dev_err(uc->dev, "i2c_transfer failed %d\n", status);
			pm_runtime_put_sync(uc->dev);
			return CY_ERROR_I2C_READ_DATA_FAILED;
		}
		rab += rlen;
		rem_len -= rlen;
	}

	pm_runtime_put_sync(uc->dev);
	return CY_SUCCESS_RESP;
}


int ccg_write(struct cypd_data *uc, u16 rab, const u8 *data, u32 len)
{
	struct i2c_client *client = uc->client;
	unsigned char *buf;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0x0,
		}
	};
	int status;

	buf = kzalloc(len + sizeof(rab), GFP_KERNEL);
	if (!buf)
		return CY_ERROR_I2C_WRITE_DATA_FAILED;

	put_unaligned_le16(rab, buf);
	memcpy(buf + sizeof(rab), data, len);

	msgs[0].len = len + sizeof(rab);
	msgs[0].buf = buf;

	pm_runtime_get_sync(uc->dev);
	status = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (status < 0) {
		dev_err(uc->dev, "i2c_transfer failed %d\n", status);
		pm_runtime_put_sync(uc->dev);
		kfree(buf);
		return CY_ERROR_I2C_WRITE_DATA_FAILED;
	}

	pm_runtime_put_sync(uc->dev);
	kfree(buf);
	return CY_SUCCESS_RESP;
}

int cypd_parse_dt(struct device *dev, struct cypd_platform_data *pdata)
{

	struct device_node *np = dev->of_node;

	/* swtich, irq gpio info */
	pdata->sw_gpio = of_get_named_gpio_flags(np, "cypd3125,sw-gpio", 0,
						 &pdata->sw_gpio_flags);
	if (pdata->sw_gpio < 0)
		pr_err("Unable to get reset_gpio");

	pdata->irq_gpio = of_get_named_gpio_flags(np, "cypd3125,irq-gpio", 0,
						  &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		pr_err("Unable to get irq_gpio");

	pr_err(" %s irq gpio:%d, switch gpio:%d", __func__,
		pdata->irq_gpio, pdata->sw_gpio);

	return 0;
}

int cypd_gpio_configure(struct cypd_data *data)
{
	int ret = 0;

	pr_err(" %s irq gpio:%d, switch gpio:%d", __func__,
		data->pdata->irq_gpio, data->pdata->sw_gpio);


	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "cs_irq_gpio");
		if (ret) {
			pr_err("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->pdata->irq_gpio);
		if (ret) {
			pr_err("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request sw gpio */
	if (gpio_is_valid(data->pdata->sw_gpio)) {
		ret = gpio_request(data->pdata->sw_gpio, "cypd_sw_gpio");
		if (ret) {
			pr_err("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->sw_gpio, 1);
		if (ret) {
			pr_err("[GPIO]set_direction for reset gpio failed");
			goto err_sw_gpio_dir;
		}
	}


	return 0;

err_sw_gpio_dir:
	if (gpio_is_valid(data->pdata->sw_gpio))
		gpio_free(data->pdata->sw_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:

	return ret;
}

int cypd_gpio_free(struct cypd_data *data)
{
	int ret = 0;

	pr_err(" %s irq gpio:%d, switch gpio:%d", __func__,
		data->pdata->irq_gpio, data->pdata->sw_gpio);


	if (gpio_is_valid(data->pdata->sw_gpio))
		gpio_free(data->pdata->sw_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	return ret;
}

int cypd_get_ic_information(struct cypd_data *data)
{
	return 0;
}


int cypd_read_pddata(struct cypd_data *data)
{

	return 0;
}

int cypd_create_sysfs(struct i2c_client *client,
	struct attribute_group *cypd_attribute_group)
{
	int ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, cypd_attribute_group);
	if (ret < 0) {
		pr_err("Can't create sysfs entries\n");
		return ret;
	}

	return ret;
}

int cypd_remove_sysfs(struct i2c_client *client,
	struct attribute_group *cypd_attribute_group)
{
	sysfs_remove_group(&client->dev.kobj, cypd_attribute_group);

	return 0;
}

irqreturn_t cypd_interrupt(int irq, void *data)
{
	int ret = 0;
	struct cypd_data *cypd_data = (struct cypd_data *)data;

	if (!cypd_data) {
		pr_err("[INTR]: Invalid cypd_data");
		return IRQ_HANDLED;
	}

	ret = cypd_read_pddata(cypd_data);
	if (ret == 0) {
		pr_err("[INTR]: got cypd_data");
	} else{
		pr_err("[INTR]: get cypd_data failed");
		return IRQ_HANDLED;
	}


	return IRQ_HANDLED;
}

int cypd_irq_registration(struct cypd_data *cypd_data)
{
	int ret = 0;
	struct cypd_platform_data *pdata = cypd_data->pdata;

	cypd_data->irq = gpio_to_irq(pdata->irq_gpio);
	pr_err("%s irq in cypd_data:%d irq in client:%d",
		__func__, cypd_data->irq,
		cypd_data->client->irq);

	if (cypd_data->irq != cypd_data->client->irq)
		pr_err("IRQs are inconsistent, please check <interrupts> & <cypd3125,irq-gpio> in DTS");

	if (pdata->irq_gpio_flags == 0)
		pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;
	pr_err("%s irq flag:%x", __func__, pdata->irq_gpio_flags);

	ret = request_threaded_irq(cypd_data->irq, NULL,
			cypd_interrupt, pdata->irq_gpio_flags | IRQF_ONESHOT,
			cypd_data->client->name, cypd_data);
	return ret;
}

