/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#ifndef __LINUX_CYPD_CORE_H__
#define __LINUX_CYPD_CORE_H__
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include "cypd-flash.h"


#define CYPD_I2C_RETRIES 2
#define CYPD_I2C_RETRY_DELAY 2



struct cypd_platform_data {
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 sw_gpio;
	u32 sw_gpio_flags;
};

struct cypd_data {
	struct device *dev;
	struct i2c_client *client;
	struct cypd_platform_data *pdata;
	struct mutex lock; /* to sync between user and driver thread */
	struct workqueue_struct *cypd_workqueue;
	struct work_struct fwupg_work;
	spinlock_t irq_lock;
	int irq;
	bool suspended;
	bool fw_loading;
	bool irq_disabled;
	struct ccg_dev_info info;
	u32 fw_version;
	/* CCG HPI communication flags */
	unsigned long flags;
	int port_num;
	struct work_struct work;
};

int ccg_read(struct cypd_data *uc, u16 rab, u8 *data, u32 len);
int ccg_write(struct cypd_data *uc, u16 rab, const u8 *data, u32 len);
int cypd_parse_dt(struct device *dev, struct cypd_platform_data *pdata);
int cypd_gpio_configure(struct cypd_data *data);
int cypd_gpio_free(struct cypd_data *data);
int cypd_get_ic_information(struct cypd_data *data);
int cypd_create_sysfs(struct i2c_client *client,
	struct attribute_group *cypd_attribute_group);
int cypd_remove_sysfs(struct i2c_client *client,
	struct attribute_group *cypd_attribute_group);
int cypd_irq_registration(struct cypd_data *cy_data);


#endif /* __CYPD3125_H__ */
