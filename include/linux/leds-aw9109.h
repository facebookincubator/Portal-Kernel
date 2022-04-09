/*
 * leds-aw9109.c   aw9109 led module
 *
 * Version: 1.0.1
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _AW9109_H_
#define _AW9109_H_

#define MAX_I2C_BUFFER_SIZE 65536

#define AW9109_ID 0xb223

#define PATTERN_LEN 30

#define AW9109_CH_MIN 2
#define AW9109_CH_MAX 10
#define AW9109_CH_MIDDLE ((AW9109_CH_MIN + AW9109_CH_MAX) / 2)

enum aw9109_chip_state {
	ST_RESET = 0,
	ST_ACTIVE
};

struct aw9109 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct work_struct brightness_work;
	struct mutex lock;
	int reset_gpio;
	int irq_gpio;
	unsigned int chipid;
	enum aw9109_chip_state chip_state;
	int imax;
	char pattern[PATTERN_LEN];
	const char *ba_pattern;
	unsigned last_bar_brightness[AW9109_CH_MAX + 1];
};

#endif

