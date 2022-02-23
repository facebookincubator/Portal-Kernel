/*
 * tc358840.c - Toshiba HDMI-CSI bridge driver
 *
 * Author:       Plamen Valev <pvalev@mm-sol.com>
 * Copyright:   (C) 2017 MM Solution AD
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

/*#define DEBUG*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include "tc358840_regs.h"


#define TC358840_MAX_EDID_SIZE			256
#define TC358840_RESET_ALL_PARTS		0
#define TC358840_RESET_WO_HDMI			1

/*#define TC358840_INTERRUPT_REQUEST*/

/* device-config */
/* config_A
 *	Video
 *		1690x3040@30fps @RGB888 (set to 1680 by mistake)
 *		Real data is 4056x3040@30fps @RAW10.
 *	Audio
 *		Input: HDMI 4 channel stereo 48kHz 16bit
 *		Output: I2S PCM 4 channel 32bit slots  48kHz 16bit
 */
/* config_B
 *	Video
 *		1690x3040@30fps @RGB888
 *		Real data is 4056x3040@30fps @RAW10.
 *	Audio
 *		Input: HDMI 8 channel 7.1 48kHz 24bit
 *		Output: I2S TDO 8 channel 32bit slots 48kHz 24bit
 */
/* config_C
 *	Video
 *		1750x3120@30fps @RGB888
 *		Real data is 4200x3120@30fps @RAW10.
 *	Audio
 *		Input: HDMI 4 channel stereo 48kHz 24bit
 *		Output: I2S PCM 4 channel 32bit slots 48kHz 24bit
 */
/* config_D
 *	Video
 *		1680x3016@30fps @RGB888
 *		Real data is 4032x3016@30fps @RAW10.
 *	Audio
 *		Input: HDMI 4 channel stereo 48kHz 24bit
 *		Output: TDM 4 channel 32bit slots 48kHz 24bit
 */
/* config_E
 *	Video
 *		1680x3016@30fps @RGB888
 *		Real data is 4032x3016@30fps @RAW10.
 *	Audio
 *		Input: HDMI 4 channel stereo 48kHz 24bit
 *		Output: TDM 4 channel 32bit slots 48kHz 24bit
 */
enum tc358840_device_config {
	TC358840_DEVICE_CONFIG_A = 0,
	TC358840_DEVICE_CONFIG_B,
	TC358840_DEVICE_CONFIG_C,
	TC358840_DEVICE_CONFIG_D,
	TC358840_DEVICE_CONFIG_E
};

struct tc358840_priv {
	struct i2c_client *client;
	char edid[TC358840_MAX_EDID_SIZE];
	uint16_t edid_size;
#ifdef TC358840_INTERRUPT_REQUEST
	int interrupt;
#endif
	int reset_gpio;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	struct clk *clk;
	enum tc358840_device_config device_config;
};

#define MIN(a, b) ((a) < (b) ? (a):(b))

static struct tc358840_priv *ptc358840_devdata;

static bool enabled;
module_param(enabled, bool, 0444);
MODULE_PARM_DESC(enabled, "Enabled");

#ifdef DEBUG
static int tc358840_color_bar(struct i2c_client *client);
#endif
static int tc358840_init_hdmi_subsystem(struct i2c_client *client);
static int tc358840_initial_config(struct i2c_client *client);
static int tc358840_init_all_subsystems(struct i2c_client *client);
static int tc358840_swreset(struct i2c_client *client, int what);
static int tc358840_hwreset(struct i2c_client *client);

static uint8_t edid_config_a[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x00, 0x00, 0xEE, 0x20, 0x00, 0x00,
	0x2B, 0x1B, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78,
	0x02, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26,
	0x12, 0x45, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x2C, 0x41,
	0x90, 0x80, 0x60, 0xE0, 0x24, 0xB0, 0x04, 0x04,
	0x21, 0x02, 0xA8, 0x30, 0x01, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18,
	0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
	0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x91,
	0x02, 0x03, 0x22, 0x40, 0x23, 0x0B, 0x07, 0x07,
	0x83, 0x09, 0x00, 0x00, 0x75, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x08, 0x3C, 0x20, 0xC0, 0x68, 0x01,
	0x02, 0x03, 0x00, 0x41, 0x4F, 0xCC, 0xA8, 0x10,
	0xB8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF
};

static uint8_t edid_config_b[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x00, 0x00, 0xEE, 0x20, 0x00, 0x00,
	0x2B, 0x1B, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78,
	0x02, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26,
	0x12, 0x45, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x2C, 0x41,
	0x9A, 0x80, 0x60, 0xE0, 0x24, 0xB0, 0x04, 0x04,
	0x21, 0x02, 0xA8, 0x30, 0x01, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18,
	0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
	0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x87,
	0x02, 0x03, 0x22, 0x40, 0x23, 0x0F, 0x07, 0x07,
	0x83, 0x2F, 0x00, 0x00, 0x75, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x08, 0x3C, 0x20, 0xC0, 0x68, 0x01,
	0x02, 0x03, 0x00, 0x41, 0x4F, 0xCC, 0xA8, 0x10,
	0xB8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5
};

static uint8_t edid_config_c[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x00, 0x00, 0xEE, 0x20, 0x00, 0x00,
	0x2B, 0x1B, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78,
	0x02, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26,
	0x12, 0x45, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x78, 0x45,
	0xE0, 0x80, 0x60, 0x30, 0x24, 0xC0, 0x04, 0x04,
	0x21, 0x02, 0xA8, 0x30, 0x01, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18,
	0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
	0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x91,
	0x02, 0x03, 0x22, 0x40, 0x23, 0x0B, 0x07, 0x07,
	0x83, 0x09, 0x00, 0x00, 0x75, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x08, 0x3C, 0x20, 0xC0, 0x68, 0x01,
	0x02, 0x03, 0x00, 0x41, 0x4F, 0xCC, 0xA8, 0x10,
	0xB8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF
};

static uint8_t edid_config_d[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x00, 0x00, 0xEE, 0x20, 0x00, 0x00,
	0x2B, 0x1B, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78,
	0x02, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26,
	0x12, 0x45, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xAA, 0x40,
	0x90, 0x80, 0x60, 0xC8, 0x24, 0xB0, 0x04, 0x04,
	0x21, 0x02, 0xA8, 0x30, 0x01, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18,
	0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
	0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x2C,
	0x02, 0x03, 0x22, 0x40, 0x23, 0x0B, 0x07, 0x07,
	0x83, 0x09, 0x00, 0x00, 0x75, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x08, 0x3C, 0x20, 0xC0, 0x68, 0x01,
	0x02, 0x03, 0x00, 0x41, 0x4F, 0xCC, 0xA8, 0x10,
	0xB8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF
};

static uint8_t edid_config_e[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x00, 0x00, 0xEE, 0x20, 0x00, 0x00,
	0x2B, 0x1B, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78,
	0x02, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26,
	0x12, 0x45, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xAA, 0x40,
	0x90, 0x80, 0x60, 0xC8, 0x24, 0xB0, 0x04, 0x04,
	0x21, 0x02, 0xA8, 0x30, 0x01, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18,
	0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
	0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x2C,
	0x02, 0x03, 0x22, 0x40, 0x23, 0x0B, 0x07, 0x07,
	0x83, 0x09, 0x00, 0x00, 0x75, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x08, 0x3C, 0x20, 0xC0, 0x68, 0x01,
	0x02, 0x03, 0x00, 0x41, 0x4F, 0xCC, 0xA8, 0x10,
	0xB8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF
};

static uint8_t tc358840_i2c_read_reg8(struct i2c_client *client,
					uint16_t reg)
{
	uint8_t rval;
	uint8_t wbuf[] = {
		(reg >> 8) & 0xFF, (reg >> 0) & 0xFF,
	};
	uint8_t rbuf[1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = wbuf,
			.len = sizeof(wbuf)
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = sizeof(rbuf)
		},
	};
	memset(rbuf, 0, sizeof(rbuf));

	if (i2c_transfer(client->adapter, msg,
			 ARRAY_SIZE(msg)) !=  ARRAY_SIZE(msg))
		dev_dbg(&client->dev, "i2c read error\n");

	rval = rbuf[0];

	dev_dbg(&client->dev, "%s: Reg:0x%04x, Val:0x%02x\n",
			__func__, reg, rval);

	return rval;
}

static int tc358840_i2c_write_reg8(struct i2c_client *client,
					uint16_t reg, uint8_t val)
{
	uint8_t wbuf[] = {
		(reg >>  8) & 0xFF, (reg >>  0) & 0xFF,
		(val >>  0) & 0xFF
	};

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = wbuf,
			.len = sizeof(wbuf)
		}
	};

	dev_dbg(&client->dev, "%s: Reg:0x%04x, Val:0x%02x\n",
			__func__, reg, val);

	return i2c_transfer(client->adapter, msg,
			ARRAY_SIZE(msg)) != ARRAY_SIZE(msg) ? -EREMOTEIO : 0;
}

static uint16_t tc358840_i2c_read_reg16(struct i2c_client *client,
					uint16_t reg)
{
	uint16_t rval;
	uint8_t wbuf[] = {
		(reg >> 8) & 0xFF, (reg >> 0) & 0xFF,
	};
	uint8_t rbuf[2];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = wbuf,
			.len = sizeof(wbuf)
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = sizeof(rbuf)
		},
	};
	memset(rbuf, 0, sizeof(rbuf));

	if (i2c_transfer(client->adapter, msg,
			 ARRAY_SIZE(msg)) !=  ARRAY_SIZE(msg))
		dev_dbg(&client->dev, "i2c read error\n");

	rval = ((rbuf[0] <<  0) | (rbuf[1] <<  8));

	dev_dbg(&client->dev, "%s: Reg:0x%04x, Val:0x%04x\n",
			__func__, reg, rval);

	return rval;
}

static int tc358840_i2c_write_reg16(struct i2c_client *client,
					uint16_t reg, uint16_t val)
{
	uint8_t wbuf[] = {
		(reg >>  8) & 0xFF, (reg >>  0) & 0xFF,
		(val >>  0) & 0xFF, (val >>  8) & 0xFF
	};

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = wbuf,
			.len = sizeof(wbuf)
		}
	};

	dev_dbg(&client->dev, "%s: writing i2c bus Reg:0x%04x, Val:0x%04x\n",
			__func__, reg, val);

	return i2c_transfer(client->adapter, msg,
			ARRAY_SIZE(msg)) != ARRAY_SIZE(msg) ? -EREMOTEIO : 0;
}

#ifdef DEBUG
static uint32_t tc358840_i2c_read_reg32(struct i2c_client *client,
					uint16_t reg)
{
	s32 rval;
	u8 wbuf[] = {
		(reg >> 8) & 0xFF, (reg >> 0) & 0xFF,
	};
	u8 rbuf[4];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) & ~I2C_M_RD,
			.buf = wbuf,
			.len = sizeof(wbuf)
		},
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) |  I2C_M_RD,
			.buf = rbuf,
			.len = sizeof(rbuf)
		},
	};
	memset(rbuf, 0, sizeof(rbuf));

	if (i2c_transfer(client->adapter, msg,
			 ARRAY_SIZE(msg)) !=  ARRAY_SIZE(msg))
		dev_dbg(&client->dev, "i2c read error\n");

	rval = ((rbuf[0] <<  0) | (rbuf[1] <<  8) |
		(rbuf[2] << 16) | (rbuf[3] << 24));

	dev_dbg(&client->dev, "%s: reading i2c bus Reg:0x%04x, Val:0x%08x\n",
			__func__, reg, rval);

	return rval;
}
#endif /*DEBUG*/

static s32 tc358840_i2c_write_reg32(struct i2c_client *client,
					uint16_t reg, uint32_t val)
{
	uint8_t wbuf[] = {
		(reg >>  8) & 0xFF, (reg >>  0) & 0xFF,
		(val >>  0) & 0xFF, (val >>  8) & 0xFF,
		(val >> 16) & 0xFF, (val >> 24) & 0xFF
	};

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = wbuf,
			.len = sizeof(wbuf)
		}
	};

	dev_dbg(&client->dev, "%s: writing i2c bus Reg:0x%04x, Val:0x%08x\n",
			__func__, reg, val);

	return i2c_transfer(client->adapter, msg,
			ARRAY_SIZE(msg)) != ARRAY_SIZE(msg) ? -EREMOTEIO : 0;
}

static ssize_t tc358840_show_revid(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return snprintf((char *)buf, PAGE_SIZE, "0x%X\n",
			tc358840_i2c_read_reg16(client, TC358840_CHIPID_ADDR));
}
static DEVICE_ATTR(revid, S_IRUGO, tc358840_show_revid, NULL);

static ssize_t tc358840_show_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return snprintf((char *)buf, PAGE_SIZE, "0x%X\n",
			tc358840_i2c_read_reg8(client, TC358840_SYS_STATUS));
}
static DEVICE_ATTR(status, S_IRUGO, tc358840_show_status, NULL);


static ssize_t tc358840_store_confctrl0(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int val;
	int rc;

	rc = kstrtoint(buf, 16, &val);
	if (rc != 0)
		return -EINVAL;

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, val);

	return count;
}

static ssize_t tc358840_show_confctrl0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return snprintf(buf, PAGE_SIZE, "0x%X\n",
			tc358840_i2c_read_reg16(client, TC358840_CONFCTL0));
}
static DEVICE_ATTR(confctrl0, S_IRUGO | S_IWUSR,
		tc358840_show_confctrl0, tc358840_store_confctrl0);

static ssize_t tc358840_store_sysctrl(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int val;
	int rc;

	rc = kstrtoint(buf, 16, &val);
	if (rc != 0)
		return -EINVAL;

	tc358840_i2c_write_reg16(client, TC358840_SYSCTL, val);

	return count;
}

static ssize_t tc358840_show_sysctrl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
			tc358840_i2c_read_reg16(client, TC358840_SYSCTL));
}
static DEVICE_ATTR(sysctrl, S_IRUGO | S_IWUSR,
		tc358840_show_sysctrl, tc358840_store_sysctrl);

static ssize_t tc358840_store_edid(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	if (count > TC358840_MAX_EDID_SIZE) {
		dev_err(dev, "Input over %d chars long %zu\n",
				TC358840_MAX_EDID_SIZE, count);
		return -EINVAL;
	}

	if (count < 8) {
		switch (tc->device_config) {
		case TC358840_DEVICE_CONFIG_A:
			tc->edid_size = sizeof(edid_config_a);
			memcpy(tc->edid, edid_config_a, tc->edid_size);
			break;
		case TC358840_DEVICE_CONFIG_B:
			tc->edid_size = sizeof(edid_config_b);
			memcpy(tc->edid, edid_config_b, tc->edid_size);
			break;
		case TC358840_DEVICE_CONFIG_C:
			tc->edid_size = sizeof(edid_config_c);
			memcpy(tc->edid, edid_config_c, tc->edid_size);
			break;
		case TC358840_DEVICE_CONFIG_D:
			tc->edid_size = sizeof(edid_config_d);
			memcpy(tc->edid, edid_config_d, tc->edid_size);
			break;
		case TC358840_DEVICE_CONFIG_E:
			tc->edid_size = sizeof(edid_config_e);
			memcpy(tc->edid, edid_config_e, tc->edid_size);
			break;
		default:
			/* config_A */
			tc->edid_size = sizeof(edid_config_a);
			memcpy(tc->edid, edid_config_a, tc->edid_size);
		}
	} else {
		memcpy(tc->edid, buf, count);
		tc->edid_size = count;
	}

	tc358840_init_hdmi_subsystem(client);

	dev_dbg(&client->dev, "%s: edid size=%d", __func__, tc->edid_size);

	return count;
}

static ssize_t tc358840_show_edid(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc358840_priv *tc = i2c_get_clientdata(client);
	int edid_size = MIN(tc->edid_size, PAGE_SIZE);

	dev_dbg(&client->dev, "%s: edid size=%d", __func__, edid_size);

	memcpy(buf, tc->edid, edid_size);

	return edid_size;
}
static DEVICE_ATTR(edid, S_IRUGO | S_IWUSR,
		tc358840_show_edid, tc358840_store_edid);

static ssize_t tc358840_start_transfer(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);

	dev_dbg(&client->dev, "%s: buf[0]=%d count=%zu",
		__func__, buf[0], count);
	if (buf[0] == '0') {
		tc358840_swreset(client, TC358840_RESET_WO_HDMI);
	} else if (buf[0] == '1') {
		tc358840_swreset(client, TC358840_RESET_WO_HDMI);
		tc358840_init_all_subsystems(client);
	}

	return count;
}
static DEVICE_ATTR(start, S_IWUSR, NULL, tc358840_start_transfer);

#ifdef DEBUG
static ssize_t tc358840_store_colorbar(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);

	dev_dbg(&client->dev, "%s: buf[0]=%d count=%zu",
		__func__, buf[0], count);
	tc358840_hwreset(client);
	tc358840_swreset(client, TC358840_RESET_ALL_PARTS);
	tc358840_color_bar(client);

	return count;
}
static DEVICE_ATTR(colorbar, S_IWUSR, NULL, tc358840_store_colorbar);

static ssize_t tc358840_store_reg(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int r, regaddr, regaddr2, reglen, val;
	int matched;

	dev_dbg(&client->dev, "%s: buf=%s len=%zu", __func__, buf, count);

	if (buf[0] == 'A') {
		matched = sscanf(buf, "A%dx%x-%x",
				&reglen, &regaddr, &regaddr2);
		if ((matched == 3) && (regaddr < regaddr2)) {
			for (r = regaddr; r <= regaddr2; r += (reglen/8)) {
				switch (reglen) {
				case 8:
				dev_dbg(&client->dev,
					"%s: %x = 0x%02X\n", __func__,
					regaddr,
					tc358840_i2c_read_reg8(client, r));
					break;
				case 16:
				dev_dbg(&client->dev,
					"%s: %x = 0x%04X\n",
					__func__, regaddr,
					tc358840_i2c_read_reg16(client, r));
					break;
				case 32:
				dev_dbg(&client->dev,
					"%s: %x = 0x%08X\n",
					__func__, regaddr,
					tc358840_i2c_read_reg32(client, r));
					break;
				default:
				dev_err(&client->dev,
					"%s: Unknown register size %d",
					__func__, r);
					break;
				}
			}
		} else {
			dev_err(&client->dev,
				"%s: Mismatched input string. expect: A{8|16|32}xREGNUM-REGNUM",
				__func__);
		}
	} else if (buf[0] == 'R') {
		matched = sscanf(buf, "R%dx%x", &reglen, &regaddr);
		if (matched == 2) {
			switch (reglen) {
			case 8:
			dev_dbg(&client->dev, "%s: %x = 0x%02X\n",
				__func__, regaddr,
				tc358840_i2c_read_reg8(client, regaddr));
				break;
			case 16:
			dev_dbg(&client->dev,
				"%s: %x = 0x%04X\n",
				__func__, regaddr,
				tc358840_i2c_read_reg16(client, regaddr));
				break;
			case 32:
			dev_dbg(&client->dev,
				"%s: %x = 0x%08X\n",
				__func__, regaddr,
				tc358840_i2c_read_reg32(client, regaddr));
				break;
			default:
			dev_err(&client->dev,
				"%s: Unknown register size %d",
				__func__, reglen);
				break;
			}
		} else {
			dev_err(&client->dev,
				"%s: Mismatched input string. expect: R{8|16|32}xREGNUM",
				__func__);
		}
	} else if (buf[0] == 'W') {
		matched = sscanf(buf, "W%dx%xx%x", &reglen, &regaddr, &val);
		if (matched == 3) {
			switch (reglen) {
			case 8:
				tc358840_i2c_write_reg8(client, regaddr, val);
				break;
			case 16:
				tc358840_i2c_write_reg16(client, regaddr, val);
				break;
			case 32:
				tc358840_i2c_write_reg32(client, regaddr, val);
				break;
			default:
				dev_err(&client->dev,
					"%s: Unknown register size %d",
					__func__, reglen);
				break;
			}
		} else {
			dev_err(&client->dev,
				"%s: Mismatched input string. expect: W{8|16|32}xREGNUMxVAL",
				__func__);
		}
	} else {
		dev_err(&client->dev,
			"%s: Mismatched input string. expect: {A|R|W}{8|16|32}xREGNUM[-REGNUM][xVAL]",
			__func__);
	}

	return count;
}
static DEVICE_ATTR(reg, S_IWUSR, NULL, tc358840_store_reg);
#endif /*DEBUG*/

static void tc358840_register_sysfs(struct i2c_client *client)
{
	/* Register sysfs files */
	device_create_file(&client->dev, &dev_attr_revid);
	device_create_file(&client->dev, &dev_attr_status);
	device_create_file(&client->dev, &dev_attr_sysctrl);
	device_create_file(&client->dev, &dev_attr_confctrl0);
	device_create_file(&client->dev, &dev_attr_edid);
	device_create_file(&client->dev, &dev_attr_start);
#ifdef DEBUG
	device_create_file(&client->dev, &dev_attr_colorbar);
	device_create_file(&client->dev, &dev_attr_reg);
#endif
}

static void tc358840_dump_hdmi_status(void)
{
	uint16_t val = 0;

	if (ptc358840_devdata) {
		dev_dbg(&ptc358840_devdata->client->dev, "%s", __func__);

		val = tc358840_i2c_read_reg16(
			ptc358840_devdata->client, TC358840_SYS_STATUS);

		/* Bit description TC358840_SYS_STATUS
			Bit[7]: S_SYNC     --> Input video Sync Signal Status
			Bit[6]: S_AVMUTE   --> AVMUTE Status
			Bit[5]: S_HDCP     --> HDCP Status
			Bit[4]: S_HDMI     --> HDMI Status
			Bit[3]: S_PHY_SCDT --> PHY DE Detect Status
			Bit[2]: S_PHY_PLL  --> PHY PLL Status
			Bit[1]: S_TMDS     --> TMDS input amplitude status
			Bit[0]: S_DDC5V    --> DDCP Power input STatus
		*/

		switch (val)
		{
			case 0x0:
				dev_err(&ptc358840_devdata->client->dev,
				"%s: HDMI cable not connected during bootup!, status = %x\n",
				__func__, val);
			break;

			case 0x09:
				dev_err(&ptc358840_devdata->client->dev,
			     "%s: HDMI cable disconnected!, status = %x\n", __func__, val);
			break;

			case 0x9F:
			case 0xBF:
				dev_info(&ptc358840_devdata->client->dev,
				"%s: HDMI cable connected, status = %x\n", __func__, val);
			break;

			default:
				dev_info(&ptc358840_devdata->client->dev,
			    "%s: Invalid HDMI connect status: %x \n", __func__, val);
		}
	}
}

void tc358840_start_stream(void)
{
	uint16_t reg = 0;

	if (ptc358840_devdata) {
		dev_dbg(&ptc358840_devdata->client->dev, "%s", __func__);

		tc358840_dump_hdmi_status();

		reg = tc358840_i2c_read_reg16(
			ptc358840_devdata->client, TC358840_FV_CNT_LO);
		if (!reg)
			dev_err(&ptc358840_devdata->client->dev,
				"%s: HOST NOT PRESENT!!! Camera will crash!\n",
				__func__);
		tc358840_swreset(ptc358840_devdata->client,
			TC358840_RESET_WO_HDMI);
		tc358840_init_all_subsystems(ptc358840_devdata->client);
	}
}
EXPORT_SYMBOL(tc358840_start_stream);

void tc358840_stop_stream(void)
{
	if (ptc358840_devdata) {
		dev_dbg(&ptc358840_devdata->client->dev, "%s", __func__);
		tc358840_swreset(ptc358840_devdata->client,
			TC358840_RESET_WO_HDMI);
	}
}
EXPORT_SYMBOL(tc358840_stop_stream);

static inline bool tc358840_is_hdmi(struct i2c_client *client)
{
	return tc358840_i2c_read_reg8(client, TC358840_SYS_STATUS) &
		MASK_S_HDMI;
}

static inline bool tc358840_tx_5v(struct i2c_client *client)
{
	return tc358840_i2c_read_reg8(client, TC358840_SYS_STATUS) &
		MASK_S_DDC5V;
}

static inline bool tc358840_is_signal(struct i2c_client *client)
{
	return tc358840_i2c_read_reg8(client, TC358840_SYS_STATUS) &
		MASK_S_TMDS;
}

static inline bool tc358840_is_sync(struct i2c_client *client)
{
	return tc358840_i2c_read_reg8(client, TC358840_SYS_STATUS) &
		MASK_S_SYNC;
}

inline void tc358840_clear_all_irqs(struct i2c_client *client)
{
	uint16_t indx;

	for (indx = TC358840_SYS_INT; indx <= TC358840_MISC_INT; indx++)
		tc358840_i2c_write_reg8(client, indx, 0xFF);

	tc358840_i2c_write_reg16(client, TC358840_INTMASK, 0xFFFF);
}

static int tc358840_init_splitter(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg16(client, TC358840_STX0_CTRL, 0x0001);
	tc358840_i2c_write_reg16(client, TC358840_STX0_PACKETID1, 0x3424);
	tc358840_i2c_write_reg16(client, TC358840_STX0_FPX, 0x0000); /*from 0*/

	switch (tc->device_config) {
	case TC358840_DEVICE_CONFIG_A:
		/*to 1679*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x068F);
		break;
	case TC358840_DEVICE_CONFIG_B:
		/*to 1679*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x068F);
		break;
	case TC358840_DEVICE_CONFIG_C:
		 /*to 1759*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x06DF);
		break;
	case TC358840_DEVICE_CONFIG_D:
		 /*to 1679*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x068F);
		break;
	case TC358840_DEVICE_CONFIG_E:
		 /*to 1679*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x068F);
		break;
	default:
		/* config_A */
		 /*to 1679*/
		tc358840_i2c_write_reg16(client, TC358840_STX0_LPX, 0x068F);
	}
	tc358840_i2c_write_reg16(client, TC358840_STX1_CTRL, 0x0100);
	tc358840_i2c_write_reg16(client, TC358840_STX1_FPX, 0x0000);
	dev_dbg(&client->dev, "%s() - exit\n", __func__);

	return 0;
}

static int tc358840_init_csin(struct i2c_client *client, uint8_t intf)
{
	uint16_t offs = CSITX0_BASE_ADDR;
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	if (intf)
		offs = CSITX1_BASE_ADDR;

	tc358840_i2c_write_reg32(client, TC358840_CSITX_CLKEN+offs,
		0x00000001);
	switch (tc->device_config) {
	case TC358840_DEVICE_CONFIG_A:
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x0003203D);
		break;
	case TC358840_DEVICE_CONFIG_B:
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x0003203D);
		break;
	case TC358840_DEVICE_CONFIG_C:
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x00032042);
		break;
	case TC358840_DEVICE_CONFIG_D:
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x0003203D);
		break;
	case TC358840_DEVICE_CONFIG_E:
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x0003203D);
		break;
	default:
		/* config_A */
		tc358840_i2c_write_reg32(client, TC358840_PLLCONF+offs,
			0x0003203D);
	}
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN+offs,
		0x00000001);
	msleep(20);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN+offs,
		0x00000003);
	tc358840_i2c_write_reg32(client, TC358840_PPICLKEN+offs,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_MODECONF+offs,
		0x00000007);
	tc358840_i2c_write_reg32(client, TC358840_LANEEN+offs,
		0x00000014);
	tc358840_i2c_write_reg32(client, TC358840_LINEINITCNT+offs,
		0x00001388);
	tc358840_i2c_write_reg32(client, TC358840_HSTOCNT+offs,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_INTEN+offs,
		0x007F0101);
	tc358840_i2c_write_reg32(client, TC358840_HSREADCNT+offs,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_HSWRITECNT+offs,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_PERIRSTCNT+offs,
		0x00001000);
	tc358840_i2c_write_reg32(client, TC358840_FUNCMODE+offs,
		0x00000160);
	tc358840_i2c_write_reg32(client, TC358840_INPUTTOCNT+offs,
		0x000000C8);
	tc358840_i2c_write_reg32(client, TC358840_HSYNCSTOPCNT+offs,
		0x0000002A);
	tc358840_i2c_write_reg32(client, TC358840_VHDELAY+offs,
		0x00000527);
	tc358840_i2c_write_reg32(client, TC358840_APPERRMASK+offs,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_LPTXTIMECNT+offs,
		0x00000005);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_HEADERCNT+offs,
		0x00230205);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_TRAILCNT+offs,
		0x000c0007);
	tc358840_i2c_write_reg32(client, TC358840_THS_HEADERCNT+offs,
		0x00120006);
	tc358840_i2c_write_reg32(client, TC358840_TWAKEUP+offs,
		0x00005A00);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_POSTCNT+offs,
		0x0000000E);
	tc358840_i2c_write_reg32(client, TC358840_THS_TRAILCNT+offs,
		0x000C0008);
	tc358840_i2c_write_reg32(client, TC358840_HSTXVREGCNT+offs,
		0x00000020);
	tc358840_i2c_write_reg32(client, TC358840_HSTXVREGEN+offs,
		0x0000001F);
	tc358840_i2c_write_reg32(client, TC358840_CSITX_START+offs,
		0x00000001);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_init_hdmi_phy(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg8(client, TC358840_PHY_CTL, 0x03);
	tc358840_i2c_write_reg8(client, 0x8412, 0x10);
	tc358840_i2c_write_reg8(client, TC358840_PHY_EQ_CTL, 0x01);
	tc358840_i2c_write_reg8(client, TC358840_APPL_CTL, 0x31);
	tc358840_i2c_write_reg8(client, TC358840_DDCIO_CTL, 0x01);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}
static int tc358840_init_hdmi_clock(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg16(client, TC358840_SYS_FREQ0, 0x12C0);
	tc358840_i2c_write_reg8(client, TC358840_LOCK_REF_FREQA, 0x00);
	tc358840_i2c_write_reg16(client, TC358840_LOCK_REF_FREQB, 0x0753);
	tc358840_i2c_write_reg8(client, TC358840_NCO_F0_MOD, 0x02);
	tc358840_i2c_write_reg16(client, TC358840_SCLK_CSC0, 0x12C0);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}
static int tc358840_init_hdmi_audio(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg8(client, TC358840_HPD_CTL, MASK_HPD_CTL0);

	switch (tc->device_config) {
	case TC358840_DEVICE_CONFIG_A:
		tc358840_i2c_write_reg16(client, TC358840_I2SIOCTL,
					MASK_I2SCTL_1|MASK_I2SCTL_2);
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		break;
	case TC358840_DEVICE_CONFIG_B:
		/* TODO: Update channel order once DC mic order is known. */
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(LFE, FC));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FRC, FLC));
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		break;
	case TC358840_DEVICE_CONFIG_C:
		tc358840_i2c_write_reg16(client, TC358840_I2SIOCTL,
					MASK_I2SCTL_1|MASK_I2SCTL_2);
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		break;
	case TC358840_DEVICE_CONFIG_D:
		tc358840_i2c_write_reg16(client, TC358840_I2SIOCTL,
					MASK_I2SCTL_1|MASK_I2SCTL_2);
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		break;
	case TC358840_DEVICE_CONFIG_E:
		tc358840_i2c_write_reg16(client, TC358840_I2SIOCTL,
					MASK_I2SCTL_1|MASK_I2SCTL_2);
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		break;
	default:
		/* config_A */
		tc358840_i2c_write_reg16(client, TC358840_I2SIOCTL,
					MASK_I2SCTL_1|MASK_I2SCTL_2);
		tc358840_i2c_write_reg8(client, TC358840_SDO0_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(FR, FL));
		tc358840_i2c_write_reg8(client, TC358840_SDO1_ASSIGN,
					HDMI_TO_SDO_RLCHANNELS(RR, RL));
		tc358840_i2c_write_reg8(client, TC358840_SDO2_ASSIGN,
					SDO_RLCHANNELS_MUTE);
		tc358840_i2c_write_reg8(client, TC358840_SDO3_ASSIGN,
					SDO_RLCHANNELS_MUTE);
	}

	tc358840_i2c_write_reg8(client, TC358840_FORCE_MUTE, 0x00);
	tc358840_i2c_write_reg8(client, TC358840_AUTO_CMD0, 0xFF);
	tc358840_i2c_write_reg8(client, TC358840_AUTO_CMD1, 0x0C);

	tc358840_i2c_write_reg8(client, TC358840_FS_MUTE, 0x00);

	tc358840_i2c_write_reg8(client, TC358840_SDO_MODE1,
				MASK_SDO_24BIT_D0BIT|MASK_SDO_FMT_LEFT);

	tc358840_i2c_write_reg32(client, TC358840_NCO_48F0A, 0x020C49BA);
	tc358840_i2c_write_reg32(client, TC358840_NCO_44F0A, 0x01E1B08A);

	tc358840_i2c_write_reg8(client, TC358840_HDMIAUDIO_MODE, 0x00);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_init_hdmi_edid(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);
	uint16_t edidlen;
	uint16_t indx;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	edidlen = tc->edid_size;
	dev_info(&client->dev, "%s() - edid length: %d\n", __func__, edidlen);

	tc358840_i2c_write_reg8(client, TC358840_EDID_MODE, MASK_RAM_DDC2B);
	tc358840_i2c_write_reg8(client, TC358840_EDID_LEN1, edidlen & 0xFF);
	tc358840_i2c_write_reg8(client, TC358840_EDID_LEN2,
					(edidlen >> 8) & 0x07);

	for (indx = 0; indx < edidlen; indx++)
		tc358840_i2c_write_reg8(client, TC358840_EDID_RAM + indx,
					tc->edid[indx]);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_init_hdmi(struct i2c_client *client)
{

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_init_hdmi_phy(client);
	tc358840_init_hdmi_clock(client);
	tc358840_init_hdmi_audio(client);
	tc358840_init_hdmi_edid(client);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static uint16_t tc358840_verify_chipid(struct i2c_client *client)
{
	uint16_t id = 0;

	id = tc358840_i2c_read_reg16(client, TC358840_CHIPID_ADDR);
	if (id != TC358840_CHIPID) {
		dev_err(&client->dev, "Invalid chip ID 0x%04X\n", id);
		return -ENODEV;
	}

	dev_dbg(&client->dev, "TC358840 ChipID 0x%02x, Revision 0x%02x\n",
		(id & MASK_CHIPID) >> 8, id & MASK_REVID);
	return 0;
}

static int tc358840_powerup(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	gpio_set_value_cansleep(tc->reset_gpio, 0);
	msleep(100);
	gpio_set_value_cansleep(tc->reset_gpio, 1);
	msleep(20);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_hwreset(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	gpio_set_value_cansleep(tc->reset_gpio, 0);
	msleep(200);
	gpio_set_value_cansleep(tc->reset_gpio, 1);
	msleep(20);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_swreset(struct i2c_client *client, int what)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, 0x8004);

	/* Reset bridge subsystems */
	if (what == TC358840_RESET_WO_HDMI)
		tc358840_i2c_write_reg16(client, TC358840_SYSCTL, 0x3E00);
	else
		tc358840_i2c_write_reg16(client, TC358840_SYSCTL, 0x3F00);

	msleep(20);
	tc358840_i2c_write_reg16(client, TC358840_SYSCTL, 0x0000);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_init_vout(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg8(client, TC358840_VOUT_FMT,
			MASK_422FMT_HDMITHROUGH | MASK_OUTFMT_THROUGH);
	tc358840_i2c_write_reg8(client, TC358840_VOUT_FIL, 0x12);
	tc358840_i2c_write_reg8(client, TC358840_VOUT_SYNC0, MASK_MODE_2);
	tc358840_i2c_write_reg8(client, TC358840_VOUT_CSC,
			MASK_CSC_MODE_OFF | MASK_COLOR_RGB_FULL);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

#ifdef DEBUG
int tc358840_color_bar(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL0,
		0x8004);
	tc358840_i2c_write_reg16(client, TC358840_SYSCTL,
		0x3F00);
	tc358840_i2c_write_reg16(client, TC358840_SYSCTL,
		0x0000);
	tc358840_i2c_write_reg16(client, TC358840_CONFCTL1,
		0x0008);
	tc358840_i2c_write_reg16(client, TC358840_CB_CTL,
		0x0009);

	/*DSI Tx1 (32-bit Registers)*/
	tc358840_i2c_write_reg32(client, TC358840_CSITX_CLKEN+CSITX1_BASE_ADDR,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_PLLCONF+CSITX1_BASE_ADDR,
		0x00032025);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN+CSITX1_BASE_ADDR,
		0x00000001);
	msleep(20);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN+CSITX1_BASE_ADDR,
		0x00000003);

	/*DSI Tx0  (32-bit Registers)*/
	tc358840_i2c_write_reg32(client, TC358840_CSITX_CLKEN,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_PLLCONF,
		0x00031028);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN,
		0x00000001);
	msleep(20);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN,
		0x00000003);
	tc358840_i2c_write_reg32(client, TC358840_PPICLKEN,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_MODECONF,
		0x00000006);
	tc358840_i2c_write_reg32(client, TC358840_LANEEN,
		0x00000014);
	tc358840_i2c_write_reg32(client, TC358840_LINEINITCNT,
		0x00001388);
	tc358840_i2c_write_reg32(client, TC358840_HSTOCNT,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_INTEN,
		0x007F0101);
	tc358840_i2c_write_reg32(client, TC358840_BTATOCNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_PERI_BTACNT,
		0x00005000);
	tc358840_i2c_write_reg32(client, TC358840_LP_READCNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_LP_WRITECNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_HSREADCNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_HSWRITECNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_PERIRSTCNT,
		0x00001000);
	tc358840_i2c_write_reg32(client, TC358840_LRXHTOCNT,
		0x00010000);
	tc358840_i2c_write_reg32(client, TC358840_FUNCMODE,
		0x00000160);
	tc358840_i2c_write_reg32(client, TC358840_RX_VC_EN,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_INPUTTOCNT,
		0x000000C8);
	tc358840_i2c_write_reg32(client, TC358840_HSYNCSTOPCNT,
		0x0000002A);
	tc358840_i2c_write_reg32(client, TC358840_VHDELAY,
		0x000005E8);
	tc358840_i2c_write_reg32(client, TC358840_DSITXMODE,
		0x00000001);
	tc358840_i2c_write_reg32(client, TC358840_HSYNCWIDTH,
		0x00000087);
	tc358840_i2c_write_reg32(client, TC358840_HBPR,
		0x0000008A);
	tc358840_i2c_write_reg32(client, TC358840_LPRX_INT_MASK,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_LPRX_FIFO_INTCOUNT,
		0x00000015);
	tc358840_i2c_write_reg32(client, TC358840_APPERRMASK,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_LPRX_ERR_MASK,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_LPTX_DONE_MASK,
		0x00000000);
	tc358840_i2c_write_reg32(client, TC358840_LPTXTIMECNT,
		0x00000006);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_HEADERCNT,
		0x00260205);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_TRAILCNT,
		0x000D0008);
	tc358840_i2c_write_reg32(client, TC358840_THS_HEADERCNT,
		0x00140006);
	tc358840_i2c_write_reg32(client, TC358840_TWAKEUP,
		0x00005300);
	tc358840_i2c_write_reg32(client, TC358840_TCLK_POSTCNT,
		0x0000000F);
	tc358840_i2c_write_reg32(client, TC358840_THS_TRAILCNT,
		0x000D0008);
	tc358840_i2c_write_reg32(client, TC358840_HSTXVREGCNT,
		0x00000020);
	tc358840_i2c_write_reg32(client, TC358840_HSTXVREGEN,
		0x0000001F);
	tc358840_i2c_write_reg32(client, TC358840_BTA_COUNT,
		0x00040003);
	tc358840_i2c_write_reg32(client, TC358840_DPHY_TX_ADJUST,
		0x00000002);
	tc358840_i2c_write_reg32(client, TC358840_CSITX_START,
		0x00000001);

	/*REM	Split Control (16-bit)*/
	tc358840_i2c_write_reg16(client, TC358840_STX0_CTRL,
		0x0001);
	tc358840_i2c_write_reg16(client, TC358840_STX0_FPX,
		0x0000);
	tc358840_i2c_write_reg16(client, TC358840_STX0_LPX,
		0x068F);
	tc358840_i2c_write_reg16(client, TC358840_STX1_CTRL,
		0x0100);
	tc358840_i2c_write_reg16(client, TC358840_STX1_FPX,
		0x0000);

	/*REM	ColoBar Generation (16-bit)*/
	tc358840_i2c_write_reg16(client, TC358840_CB_CTL,
		0x000D);
	tc358840_i2c_write_reg16(client, TC358840_CB_HSW,
		0x00A5);
	tc358840_i2c_write_reg16(client, TC358840_CB_VSW,
		0x003F);
	tc358840_i2c_write_reg16(client, TC358840_CB_HTOTOAL,
		0x073D);
	tc358840_i2c_write_reg16(client, TC358840_CB_VTOTOAL,
		0x0C22);
	tc358840_i2c_write_reg16(client, TC358840_CB_HACT,
		0x0690);
	tc358840_i2c_write_reg16(client, TC358840_CB_VACT,
		0x0BE0);
	tc358840_i2c_write_reg16(client, TC358840_CB_HSTART,
		0x00A9);
	tc358840_i2c_write_reg16(client, TC358840_CB_VSTART,
		0x0040);

	/*End ofColorBar Init*/
	tc358840_i2c_write_reg16(client, TC358840_CONFCTL0,
		0x8005);
	tc358840_i2c_write_reg16(client, TC358840_CONFCTL1,
		0x0000);
	msleep(20);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}
#endif /*DEBUG*/

static int tc358840_init_hdmi_subsystem(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_hwreset(client);
	tc358840_initial_config(client);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_init_all_subsystems(struct i2c_client *client)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);
	unsigned int val;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL1, 0x0008);
	tc358840_i2c_write_reg16(client, TC358840_INTSTATUS, 0x0FFF);
	tc358840_i2c_write_reg16(client, TC358840_INTMASK, 0x0FFF);


	tc358840_init_csin(client, 0);

	/* Enable CSI1 MIPI clock */
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN + CSITX1_BASE_ADDR,
		0x00000001);
	msleep(20);
	tc358840_i2c_write_reg32(client, TC358840_MIPICLKEN + CSITX1_BASE_ADDR,
		0x00000003);

	tc358840_init_splitter(client);
	tc358840_init_hdmi(client);
	tc358840_init_vout(client);

	/* End of configuration */
	tc358840_i2c_write_reg8(client, TC358840_INIT_END, 0x01);
	msleep(1000);

	/* Re-enable bridge subsystems */
	switch (tc->device_config) {
	case TC358840_DEVICE_CONFIG_A:
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, 0x8935);
		break;
	case TC358840_DEVICE_CONFIG_B:
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, 0x813D);
		break;
	case TC358840_DEVICE_CONFIG_C:
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, 0x893D);
		break;
	case TC358840_DEVICE_CONFIG_D:
		val = MASK_TX_MSEL | MASK_AUDCHNUM_4 | MASK_I2SDLYOPT
			| MASK_ABUFEN | MASK_AUDOUTSEL_TDM
			| MASK_AUTOINDEX | MASK_VTX0EN;
		dev_dbg(&client->dev, "%s: reg ctl0: 0x%X\n", __func__, val);
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, val);
		break;
	case TC358840_DEVICE_CONFIG_E:
		val = MASK_TX_MSEL | MASK_AUDCHNUM_4 | MASK_I2SDLYOPT
			| MASK_ABUFEN | MASK_AUDOUTSEL_TDM
			| MASK_AUTOINDEX | MASK_VTX0EN;
		dev_dbg(&client->dev, "%s: reg ctl0: 0x%X\n", __func__, val);
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, val);
		break;
	default:
		/* config_A */
		tc358840_i2c_write_reg16(client, TC358840_CONFCTL0, 0x8935);
	}

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL1, 0x0000);
	msleep(20);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

static int tc358840_initial_config(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc358840_swreset(client, TC358840_RESET_ALL_PARTS);

	tc358840_i2c_write_reg16(client, TC358840_CONFCTL1, 0x0008);
	tc358840_i2c_write_reg16(client, TC358840_INTSTATUS, 0x0FFF);
	tc358840_i2c_write_reg16(client, TC358840_INTMASK, 0x0FFF);

	tc358840_init_hdmi(client);

	/* End of configuration */
	tc358840_i2c_write_reg8(client, TC358840_INIT_END, 0x01);
	msleep(1000);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}

#ifdef TC358840_INTERRUPT_REQUEST
static inline void tc358840_csi_irq(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);
	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static inline void tc358840_amute_irq(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);
	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static inline void tc358840_hdmi_irq(struct i2c_client *client)
{
	uint8_t reg0, reg1;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	reg0 = tc358840_i2c_read_reg8(client, TC358840_HDMI_INT0);
	reg1 = tc358840_i2c_read_reg8(client, TC358840_HDMI_INT1);
	tc358840_i2c_write_reg8(client, TC358840_HDMI_INT0, reg0);
	tc358840_i2c_write_reg8(client, TC358840_HDMI_INT1, reg1);

	dev_dbg(&client->dev, "%s() - TC358840_HDMI_INT0: 0x%02x\n",
		__func__, reg0);
	dev_dbg(&client->dev, "%s() - TC358840_HDMI_INT1: 0x%02x\n",
		__func__, reg1);

	if (reg0 & MASK_MISC)
		dev_dbg(&client->dev, "%s() - MISC interrupt\n", __func__);
	if (reg0 & MASK_KEY)
		dev_dbg(&client->dev, "%s() - KEY-EDID interrupt\n", __func__);

	if (reg1 & MASK_SYS)
		dev_dbg(&client->dev, "%s() - SYS\n", __func__);
	if (reg1 & MASK_CLK)
		dev_dbg(&client->dev, "%s() - CLK\n", __func__);
	if (reg1 & MASK_PACKET)
		dev_dbg(&client->dev, "%s() - PACKET\n", __func__);
	if (reg1 & MASK_ACBIT)
		dev_dbg(&client->dev, "%s() - ACBIT\n", __func__);
	if (reg1 & MASK_AUD)
		dev_dbg(&client->dev, "%s() - AUD\n", __func__);
	if (reg1 & MASK_ERR)
		dev_dbg(&client->dev, "%s() - ERR\n", __func__);
	if (reg1 & MASK_HDCP)
		dev_dbg(&client->dev, "%s() - HDCP\n", __func__);
	if (reg1 & MASK_GBD)
		dev_dbg(&client->dev, "%s() - GBD\n", __func__);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static inline void tc358840_sys_irq(struct i2c_client *client)
{
	uint8_t reg;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	reg = tc358840_i2c_read_reg8(client, TC358840_SYS_INT);
	tc358840_i2c_write_reg8(client, TC358840_SYS_INT, reg);

	dev_dbg(&client->dev, "%s() - TC358840_SYS_INT: 0x%02x\n",
		__func__, reg);

	if (reg & MASK_ACR_CTS)
		dev_dbg(&client->dev, "%s() - Receive CTS update interrupt\n",
			__func__);
	if (reg & MASK_ACRN)
		dev_dbg(&client->dev, "%s() - Receive N update interrupt\n",
			__func__);
	if (reg & MASK_DVI)
		dev_dbg(&client->dev,
			"%s() - HDMI→DVI change detection interrupt\n",
			__func__);
	if (reg & MASK_HDMI)
		dev_dbg(&client->dev,
			"%s() - DVI→HDMI change detection interrupt\n",
			__func__);
	if (reg & MASK_NOPMBDET)
		dev_dbg(&client->dev,
			"%s() - No DataIsland Preamble detection interrupt\n",
			__func__);
	if (reg & MASK_DPMBDET)
		dev_dbg(&client->dev,
			"%s() - With DataIsland Preamble detection interrupt\n",
			__func__);
	if (reg & MASK_TMDS)
		dev_dbg(&client->dev,
			"%s() - TMDS amplitude change interrupt\n",
			__func__);
	if (reg & MASK_DDC)
		dev_dbg(&client->dev,
			"%s() - DDC power change detection interrupt\n",
			__func__);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static inline void tc358840_cec_irq(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);
	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static inline void tc358840_ir_irq(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s() - enter\n", __func__);
	dev_dbg(&client->dev, "%s() - exit\n", __func__);
}

static irqreturn_t tc358840_irq_handler(int irq, void *data)
{
	struct tc358840_priv *tc = (struct tc358840_priv *) data;
	struct i2c_client *client = tc->client;
	uint16_t reg;

	return IRQ_HANDLED;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	reg = tc358840_i2c_read_reg16(client, TC358840_INTSTATUS);

	if ((reg & MASK_CSITX1_INT) && (reg & MASK_CSITX0_INT))
		tc358840_csi_irq(client);

	if (reg & MASK_AMUTE_INT)
		tc358840_amute_irq(client);

	if (reg & MASK_HDMI_INT)
		tc358840_hdmi_irq(client);

	if (reg & MASK_SYS_INT)
		tc358840_sys_irq(client);

	if ((reg & MASK_CEC_EINT) ||
	    (reg & MASK_CEC_TINT) ||
	    (reg & MASK_CEC_RINT))
		tc358840_cec_irq(client);

	if ((reg & MASK_IR_EINT) || (reg & MASK_IR_DINT))
		tc358840_ir_irq(client);

	tc358840_i2c_write_reg16(client, TC358840_INTSTATUS, reg);

	dev_dbg(&client->dev, "%s() - 0x%04x\n", __func__, reg);

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return IRQ_HANDLED;
}
#endif /*TC358840_INTERRUPT_REQUEST*/

#ifdef CONFIG_OF
static void tc358840_set_pins_state(struct tc358840_priv *tc, bool flag)
{
	dev_dbg(&tc->client->dev, "%s() - enter\n", __func__);

	if (flag) {
		if (!IS_ERR_OR_NULL(tc->pins_active))
			pinctrl_select_state(tc->pinctrl, tc->pins_active);
	} else {
		if (!IS_ERR_OR_NULL(tc->pins_sleep))
			pinctrl_select_state(tc->pinctrl, tc->pins_sleep);
	}

	dev_dbg(&tc->client->dev, "%s() - exit\n", __func__);
}

static int tc358840_clk_control(struct tc358840_priv *tc, int enable)
{
	int ret = 0;

	dev_dbg(&tc->client->dev, "%s() - enter\n", __func__);

	if (enable) {
		ret = clk_prepare_enable(tc->clk);
		if (ret)
			dev_err(&tc->client->dev,
					"%s: clk_prepare_enable failed: %d\n",
					__func__, ret);
	} else
		clk_disable_unprepare(tc->clk);

	dev_dbg(&tc->client->dev, "%s() - exit\n", __func__);
	return ret;
}

static int of_tc358840(struct i2c_client *client, struct device_node *node)
{
	struct tc358840_priv *tc = i2c_get_clientdata(client);
	int ret;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (tc->reset_gpio == 0)
		return -ENODEV;

	dev_dbg(&client->dev, "%s() - reset_gpio = %d\n",
			__func__, tc->reset_gpio);
#ifdef TC358840_INTERRUPT_REQUEST
	tc->interrupt = irq_of_parse_and_map(node, 0);
	if (tc->interrupt <= 0)
		return -ENODEV;

	dev_dbg(&client->dev, "%s() - interrupt = %d\n",
			__func__, tc->interrupt);
#endif
	tc->pinctrl = devm_pinctrl_get(&tc->client->dev);
	if (!IS_ERR(tc->pinctrl)) {
		tc->pins_active = pinctrl_lookup_state(tc->pinctrl, "active");
		tc->pins_sleep = pinctrl_lookup_state(tc->pinctrl, "sleep");
	}

	tc->clk = devm_clk_get(&client->dev, "tc358840xbg_clk");
	if (IS_ERR(tc->clk)) {
		ret = PTR_ERR(tc->clk);
		dev_err(&client->dev, "%s: clk_get failed, ret = %d\n",
				__func__, ret);
	}

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;
}
#endif

static int tc358840_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
#endif
	struct tc358840_priv *tc;
	int ret = 0;
	const char *device_config;

	dev_dbg(&client->dev, "%s() - enter\n", __func__);

	tc = devm_kzalloc(&client->dev,
			sizeof(struct tc358840_priv), GFP_KERNEL);
	if (tc == NULL)
		return -ENOMEM;

	tc->client = client;
	i2c_set_clientdata(client, tc);

#ifdef CONFIG_OF
	tc358840_set_pins_state(tc, true);

	if (of_tc358840(client, np)) {
		dev_err(&client->dev, "%s() - Cannot get platform data\n",
			__func__);
	}
#endif
#ifdef TC358840_INTERRUPT_REQUEST
	ret = devm_request_threaded_irq(&client->dev, client->irq,
			NULL, tc358840_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"tc358840", tc);
	if (ret) {
		dev_err(&client->dev,
			"%s() - Couldn't request interrupt=%d: err=%d\n",
			__func__, client->irq, ret);
		/* return ret; */
	}
#endif
	ret = devm_gpio_request_one(&client->dev, tc->reset_gpio,
					GPIOF_OUT_INIT_HIGH, "tc358840-reset");
	if (ret) {
		dev_err(&client->dev,
			"Failed to request Reset GPIO-%d: err=%d\n",
			tc->reset_gpio, ret);
		/* return ret; */
	}

	of_property_read_string(np, "device-config", &device_config);
	if (!strcmp("config_A", device_config))
		tc->device_config = TC358840_DEVICE_CONFIG_A;
	else if (!strcmp("config_B", device_config))
		tc->device_config = TC358840_DEVICE_CONFIG_B;
	else if (!strcmp("config_C", device_config))
		tc->device_config = TC358840_DEVICE_CONFIG_C;
	else if (!strcmp("config_D", device_config))
		tc->device_config = TC358840_DEVICE_CONFIG_D;
	else if (!strcmp("config_E", device_config))
		tc->device_config = TC358840_DEVICE_CONFIG_E;

    dev_info(&client->dev, "%s: TC358840_DEVICE_%s \n",
		__func__, device_config);

	tc358840_clk_control(tc, 1);
	msleep(100);
	tc358840_powerup(client);
	msleep(100);
	tc358840_swreset(client, false);

	switch (tc->device_config) {
	case TC358840_DEVICE_CONFIG_A:
		tc->edid_size = sizeof(edid_config_a);
		memcpy(tc->edid, edid_config_a, tc->edid_size);
		break;
	case TC358840_DEVICE_CONFIG_B:
		tc->edid_size = sizeof(edid_config_b);
		memcpy(tc->edid, edid_config_b, tc->edid_size);
		break;
	case TC358840_DEVICE_CONFIG_C:
		tc->edid_size = sizeof(edid_config_c);
		memcpy(tc->edid, edid_config_c, tc->edid_size);
		break;
	case TC358840_DEVICE_CONFIG_D:
		tc->edid_size = sizeof(edid_config_d);
		memcpy(tc->edid, edid_config_d, tc->edid_size);
		break;
	case TC358840_DEVICE_CONFIG_E:
		tc->edid_size = sizeof(edid_config_e);
		memcpy(tc->edid, edid_config_e, tc->edid_size);
		break;
	default:
		/* config_A */
		tc->edid_size = sizeof(edid_config_a);
		memcpy(tc->edid, edid_config_a, tc->edid_size);
	}

	/* Check the TC358840 Chip ID and Revision ID */
	ret = tc358840_verify_chipid(client);
	if (ret) {
		tc358840_clk_control(tc, 0);
		goto err_mem_free;
	}

	tc358840_register_sysfs(client);

	tc358840_initial_config(client);

	ptc358840_devdata = tc;
	enabled = true;

	dev_dbg(&client->dev, "%s() - exit\n", __func__);
	return 0;

err_mem_free:
	devm_kfree(&client->dev, tc);
	return ret;
}

static int tc358840_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tc358840_i2c_id_table[] = {
	{"tc358840", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tc358840_i2c_id_table);

#ifdef CONFIG_OF
static const struct of_device_id tc358840_i2c_match_table[] = {
	{ .compatible = "toshiba,tc358840xbg" },
	{ },
};
#else
#define tc358840_i2c_match_table NULL
#endif

static struct i2c_driver tc358840_i2c_driver = {
	.driver = {
		.name = "tc358840",
		.owner = THIS_MODULE,
		.of_match_table = tc358840_i2c_match_table,
	},
	.probe    = tc358840_i2c_probe,
	.remove   = tc358840_i2c_remove,
	.id_table = tc358840_i2c_id_table,
};

static int __init tc358840_init(void)
{
	int rval;

	rval = i2c_add_driver(&tc358840_i2c_driver);
	if (rval < 0)
		pr_err("tc358840 i2c driver registration failed\n");

	return rval;
}

static void __exit tc358840_exit(void)
{
	i2c_del_driver(&tc358840_i2c_driver);
}

module_init(tc358840_init);
module_exit(tc358840_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Toshiba HDMI-CSI bridge driver");
MODULE_AUTHOR("Plamen Valev <pvalev@mm-sol.com>");
