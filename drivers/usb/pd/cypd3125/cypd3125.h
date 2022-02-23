/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#ifndef __LINUX_CYPD3125_H__
#define __LINUX_CYPD3125_H__

#define CYPD3125_I2C_NAME "cypd3125"
#define CYPD3125_CHIP_ID  0x1d04

extern int cypd_fwupg_init(struct cypd_data *cypd_data);
#endif /* __CYPD3125_H__ */
