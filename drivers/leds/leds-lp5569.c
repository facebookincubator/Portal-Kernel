/*
 * lp5569.c - LP5569 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *          Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include "leds-lp55xx-common.h"

#define LP5569_PROGRAM_LENGTH		32	/* bytes */
/* Memory is used like this:
 * 0x00 engine 1 program
 * 0x10 engine 2 program
 * 0x20 engine 3 program
 * 0x30 engine 1 muxing info
 * 0x40 engine 2 muxing info
 * 0x50 engine 3 muxing info
 */
#define LP5569_MAX_LEDS			9

/* Registers */

/* CONFIG Register */
#define LP5569_REG_CONFIG		0x00
#define LP5569_ENABLE			0x40

/* LED_ENGINE_CONTROL1 Register */
#define LP5569_REG_EXEC			0x01
#define LP5569_EXEC_ENG1_M		0xC0
#define LP5569_EXEC_ENG2_M		0x30
#define LP5569_EXEC_ENG3_M		0x0C
#define LP5569_EXEC_M			0xFC
#define LP5569_RUN_ENG1			0x80
#define LP5569_RUN_ENG2			0x20
#define LP5569_RUN_ENG3			0x08

/* LED_ENGINE_CONTROL2 Register */
#define LP5569_REG_OP_MODE		0x02
#define LP5569_MODE_ENG1_M		0xC0
#define LP5569_MODE_ENG2_M		0x30
#define LP5569_MODE_ENG3_M		0x0C
#define LP5569_LOAD_ENG1		0x40
#define LP5569_LOAD_ENG2		0x10
#define LP5569_LOAD_ENG3		0x04

/* LEDx_CONTROL Register */
#define LP5569_REG_LED_CTRL_BASE	0x07
#define LP5569_MF_MAPPING_MASK		0xE0
#define LP5569_MF_MAPPING_SHIFT		5

#define LP5569_REG_LED_PWM_BASE		0x16
#define LP5569_REG_LED_CURRENT_BASE	0x22

/* MISC Register */
#define LP5569_REG_MISC			0x2F
#define LP5569_AUTO_INC			0x40
#define LP5569_PWR_SAVE			0x20
#define LP5569_CP_AUTO			0x18

#define LP5569_REG_ENG_STATUS		0x3C
#define LP5569_ENG_STATUS_MASK		0x07

/* RESET Register */
#define LP5569_REG_RESET		0x3F
#define LP5569_RESET			0xFF

#define LP5569_REG_MASTER_FADER_BASE	0x46
#define LP5569_REG_CH1_PROG_START	0x4B
#define LP5569_REG_CH2_PROG_START	0x4C
#define LP5569_REG_CH3_PROG_START	0x4D
#define LP5569_REG_PROG_PAGE_SEL	0x4F
#define LP5569_REG_PROG_MEM		0x50

/* Bit description in registers */
#define LP5569_ADC_SHORTCIRC_LIM	80

/* Memory Page Selection */
#define LP5569_PAGE_ENG1		0
#define LP5569_PAGE_ENG2		1
#define LP5569_PAGE_ENG3		2
#define LP5569_PAGE_MUX1		3
#define LP5569_PAGE_MUX2		4
#define LP5569_PAGE_MUX3		5

#define LP5569_ENG1_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG1_M) == LP5569_LOAD_ENG1)
#define LP5569_ENG2_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG2_M) == LP5569_LOAD_ENG2)
#define LP5569_ENG3_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG3_M) == LP5569_LOAD_ENG3)

#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))

enum lp5569_chip_id {
	LP5569,
};

static int lp5569_init_program_engine(struct lp55xx_chip *chip);

static inline void lp5569_wait_opmode_done(void)
{
	usleep_range(1000, 2000);
}

static void lp5569_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	led->led_current = led_current;
	lp55xx_write(led->chip, LP5569_REG_LED_CURRENT_BASE + led->chan_nr,
		     led_current);
}

static int lp5569_post_init_device(struct lp55xx_chip *chip)
{
	int ret;
	int reg = 0;

	ret = lp55xx_write(chip, LP5569_REG_CONFIG, LP5569_ENABLE);
	if (ret)
		return ret;

	/* Chip startup time is 500 us, 1 - 2 ms gives some margin */
	usleep_range(1000, 2000);

	reg |= LP5569_AUTO_INC | LP5569_PWR_SAVE | LP5569_CP_AUTO;
	reg |= chip->pdata->clock_mode;

	ret = lp55xx_write(chip, LP5569_REG_MISC, reg);
	if (ret)
		return ret;

	return lp5569_init_program_engine(chip);
}

static void lp5569_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5569_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5569_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5569_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5569_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5569_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5569_LOAD_ENG3,
	};

	lp55xx_update_bits(chip, LP5569_REG_OP_MODE, mask[idx], val[idx]);

	lp5569_wait_opmode_done();
}

static void lp5569_load_engine_and_select_page(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 page_sel[] = {
		[LP55XX_ENGINE_1] = LP5569_PAGE_ENG1,
		[LP55XX_ENGINE_2] = LP5569_PAGE_ENG2,
		[LP55XX_ENGINE_3] = LP5569_PAGE_ENG3,
	};

	lp5569_load_engine(chip);

	lp55xx_write(chip, LP5569_REG_PROG_PAGE_SEL, page_sel[idx]);
}

static void lp5569_stop_all_engines(struct lp55xx_chip *chip)
{
	lp55xx_write(chip, LP5569_REG_OP_MODE, 0);
	lp5569_wait_opmode_done();
}

static void lp5569_stop_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5569_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5569_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5569_MODE_ENG3_M,
	};

	lp55xx_update_bits(chip, LP5569_REG_OP_MODE, mask[idx], 0);

	lp5569_wait_opmode_done();
}

static void lp5569_turn_off_channels(struct lp55xx_chip *chip)
{
	int i;

	for (i = 0; i < LP5569_MAX_LEDS; i++)
		lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + i, 0);
}

static void lp5569_run_engine(struct lp55xx_chip *chip, bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp5569_stop_engine(chip);
		lp5569_turn_off_channels(chip);
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(chip, LP5569_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(chip, LP5569_REG_EXEC, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5569_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5569_MODE_ENG1_M) | LP5569_RUN_ENG1;
		exec = (exec & ~LP5569_EXEC_ENG1_M) | LP5569_RUN_ENG1;
	}

	if (LP5569_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5569_MODE_ENG2_M) | LP5569_RUN_ENG2;
		exec = (exec & ~LP5569_EXEC_ENG2_M) | LP5569_RUN_ENG2;
	}

	if (LP5569_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5569_MODE_ENG3_M) | LP5569_RUN_ENG3;
		exec = (exec & ~LP5569_EXEC_ENG3_M) | LP5569_RUN_ENG3;
	}

	lp55xx_write(chip, LP5569_REG_OP_MODE, mode);
	lp5569_wait_opmode_done();

	lp55xx_update_bits(chip, LP5569_REG_EXEC, LP5569_EXEC_M, exec);
}

static int lp5569_init_program_engine(struct lp55xx_chip *chip)
{
	int i;
	int j;
	int ret;
	u8 status;
	/* one pattern per engine setting LED MUX start and stop addresses */
	static const u8 pattern[][LP5569_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};

	/* hardcode 32 bytes of memory for each engine from program memory */
	ret = lp55xx_write(chip, LP5569_REG_CH1_PROG_START, 0x00);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5569_REG_CH2_PROG_START, 0x10);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5569_REG_CH3_PROG_START, 0x20);
	if (ret)
		return ret;

	/* write LED MUX address space for each engine */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5569_load_engine_and_select_page(chip);

		for (j = 0; j < LP5569_PROGRAM_LENGTH; j++) {
			ret = lp55xx_write(chip, LP5569_REG_PROG_MEM + j,
					pattern[i - 1][j]);
			if (ret)
				goto out;
		}
	}

	lp5569_run_engine(chip, true);

	/* Let the programs run for couple of ms and check the engine status */
	usleep_range(3000, 6000);
	lp55xx_read(chip, LP5569_REG_ENG_STATUS, &status);
	status &= LP5569_ENG_STATUS_MASK;

	if (status != LP5569_ENG_STATUS_MASK) {
		dev_err(&chip->cl->dev,
			"cound not configure LED engine, status = 0x%.2x\n",
			status);
		ret = -1;
	}

out:
	lp5569_stop_all_engines(chip);
	return ret;
}

static int lp5569_update_program_memory(struct lp55xx_chip *chip,
					const u8 *data, size_t size)
{
	u8 pattern[LP5569_PROGRAM_LENGTH] = {0};
	unsigned cmd;
	char c[3];
	int nrchars;
	int ret;
	int offset = 0;
	int i = 0;

	while ((offset < size - 1) && (i < LP5569_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(data + offset, "%2s%n ", c, &nrchars);
		if (ret != 1)
			goto err;

		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto err;

		pattern[i] = (u8)cmd;
		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto err;

	for (i = 0; i < LP5569_PROGRAM_LENGTH; i++) {
		ret = lp55xx_write(chip, LP5569_REG_PROG_MEM + i, pattern[i]);
		if (ret)
			return -EINVAL;
	}

	return size;

err:
	dev_err(&chip->cl->dev, "wrong pattern format\n");
	return -EINVAL;
}

static void lp5569_firmware_loaded(struct lp55xx_chip *chip)
{
	const struct firmware *fw = chip->fw;

	if (fw->size > LP5569_PROGRAM_LENGTH) {
		dev_err(&chip->cl->dev, "firmware data size overflow: %zu\n",
			fw->size);
		return;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5569_load_engine_and_select_page(chip);
	lp5569_update_program_memory(chip, fw->data, fw->size);
}

static ssize_t show_engine_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	enum lp55xx_engine_mode mode = chip->engines[nr - 1].mode;

	switch (mode) {
	case LP55XX_ENGINE_RUN:
		return snprintf(buf, PAGE_SIZE, "run\n");
	case LP55XX_ENGINE_LOAD:
		return snprintf(buf, PAGE_SIZE, "load\n");
	case LP55XX_ENGINE_DISABLED:
	default:
		return snprintf(buf, PAGE_SIZE, "disabled\n");
	}
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;

	if (!strncmp(buf, "run", 3)) {
		lp5569_run_engine(chip, true);
		engine->mode = LP55XX_ENGINE_RUN;
	} else if (!strncmp(buf, "load", 4)) {
		lp5569_stop_engine(chip);
		lp5569_load_engine(chip);
		engine->mode = LP55XX_ENGINE_LOAD;
	} else if (!strncmp(buf, "disabled", 8)) {
		lp5569_stop_engine(chip);
		engine->mode = LP55XX_ENGINE_DISABLED;
	}

	mutex_unlock(&chip->lock);

	return len;
}
store_mode(1)
store_mode(2)
store_mode(3)

static int lp5569_mux_parse(const char *buf, u16 *mux, size_t len)
{
	u16 tmp_mux = 0;
	int i;

	len = min_t(int, len, LP5569_MAX_LEDS);

	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = len;
			break;
		default:
			return -EINVAL;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static void lp5569_mux_to_array(u16 led_mux, char *array)
{
	int i, pos = 0;

	for (i = 0; i < LP5569_MAX_LEDS; i++)
		pos += snprintf(array + pos, LP5569_MAX_LEDS + 1,
				"%x", LED_ACTIVE(led_mux, i));

	array[pos] = '\0';
}

static ssize_t show_engine_leds(struct device *dev,
			    struct device_attribute *attr,
			    char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	char mux[LP5569_MAX_LEDS + 1];

	lp5569_mux_to_array(chip->engines[nr - 1].led_mux, mux);

	return snprintf(buf, PAGE_SIZE, "%s\n", mux);
}
show_leds(1)
show_leds(2)
show_leds(3)

static int lp5569_load_mux(struct lp55xx_chip *chip, u16 mux, int nr)
{
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	int ret;
	u8 mux_page[] = {
		[LP55XX_ENGINE_1] = LP5569_PAGE_MUX1,
		[LP55XX_ENGINE_2] = LP5569_PAGE_MUX2,
		[LP55XX_ENGINE_3] = LP5569_PAGE_MUX3,
	};

	lp5569_load_engine(chip);

	ret = lp55xx_write(chip, LP5569_REG_PROG_PAGE_SEL, mux_page[nr]);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5569_REG_PROG_MEM, (u8)(mux >> 8));
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5569_REG_PROG_MEM + 1, (u8)(mux));
	if (ret)
		return ret;

	engine->led_mux = mux;
	return 0;
}

static ssize_t store_engine_leds(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	u16 mux = 0;
	ssize_t ret;

	if (lp5569_mux_parse(buf, &mux, len))
		return -EINVAL;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	ret = -EINVAL;

	if (engine->mode != LP55XX_ENGINE_LOAD)
		goto leave;

	if (lp5569_load_mux(chip, mux, nr))
		goto leave;

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}
store_leds(1)
store_leds(2)
store_leds(3)

static ssize_t store_engine_load(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	lp5569_load_engine_and_select_page(chip);
	ret = lp5569_update_program_memory(chip, buf, len);

	mutex_unlock(&chip->lock);

	return ret;
}
store_load(1)
store_load(2)
store_load(3)

#define show_fader(nr)						\
static ssize_t show_master_fader##nr(struct device *dev,	\
			    struct device_attribute *attr,	\
			    char *buf)				\
{								\
	return show_master_fader(dev, attr, buf, nr);		\
}

#define store_fader(nr)						\
static ssize_t store_master_fader##nr(struct device *dev,	\
			     struct device_attribute *attr,	\
			     const char *buf, size_t len)	\
{								\
	return store_master_fader(dev, attr, buf, len, nr);	\
}

static ssize_t show_master_fader(struct device *dev,
				 struct device_attribute *attr,
				 char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	u8 val;

	mutex_lock(&chip->lock);
	ret = lp55xx_read(chip, LP5569_REG_MASTER_FADER_BASE + nr - 1, &val);
	mutex_unlock(&chip->lock);

	if (ret == 0)
		ret = snprintf(buf, PAGE_SIZE, "%u\n", val);

	return ret;
}
show_fader(1)
show_fader(2)
show_fader(3)

static ssize_t store_master_fader(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 0xff)
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = lp55xx_write(chip, LP5569_REG_MASTER_FADER_BASE + nr - 1,
			   (u8)val);
	mutex_unlock(&chip->lock);

	if (ret == 0)
		ret = len;

	return ret;
}
store_fader(1)
store_fader(2)
store_fader(3)

static ssize_t show_master_fader_leds(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int i, ret, pos = 0;
	u8 val;

	mutex_lock(&chip->lock);

	for (i = 0; i < LP5569_MAX_LEDS; i++) {
		ret = lp55xx_read(chip, LP5569_REG_LED_CTRL_BASE + i, &val);
		if (ret)
			goto leave;

		val = (val & LP5569_MF_MAPPING_MASK)
			>> LP5569_MF_MAPPING_SHIFT;
		if (val > 7) {
			ret = -EINVAL;
			goto leave;
		}
		buf[pos++] = val + '0';
	}
	buf[pos++] = '\n';
	ret = pos;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t store_master_fader_leds(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int i, n, ret;
	u8 val;

	n = min_t(int, len, LP5569_MAX_LEDS);

	mutex_lock(&chip->lock);

	for (i = 0; i < n; i++) {
		if (buf[i] >= '0' && buf[i] <= '7') {
			val = (buf[i] - '0') << LP5569_MF_MAPPING_SHIFT;
			ret = lp55xx_update_bits(chip,
						 LP5569_REG_LED_CTRL_BASE + i,
						 LP5569_MF_MAPPING_MASK,
						 val);
			if (ret)
				goto leave;
		} else {
			ret = -EINVAL;
			goto leave;
		}
	}
	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

static void lp5569_led_brightness_work(struct work_struct *work)
{
	struct lp55xx_led *led = container_of(work, struct lp55xx_led,
					      brightness_work);
	struct lp55xx_chip *chip = led->chip;

	mutex_lock(&chip->lock);
	lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + led->chan_nr,
		     led->brightness);
	mutex_unlock(&chip->lock);
}

static LP55XX_DEV_ATTR_RW(engine1_mode, show_engine1_mode, store_engine1_mode);
static LP55XX_DEV_ATTR_RW(engine2_mode, show_engine2_mode, store_engine2_mode);
static LP55XX_DEV_ATTR_RW(engine3_mode, show_engine3_mode, store_engine3_mode);
static LP55XX_DEV_ATTR_RW(engine1_leds, show_engine1_leds, store_engine1_leds);
static LP55XX_DEV_ATTR_RW(engine2_leds, show_engine2_leds, store_engine2_leds);
static LP55XX_DEV_ATTR_RW(engine3_leds, show_engine3_leds, store_engine3_leds);
static LP55XX_DEV_ATTR_WO(engine1_load, store_engine1_load);
static LP55XX_DEV_ATTR_WO(engine2_load, store_engine2_load);
static LP55XX_DEV_ATTR_WO(engine3_load, store_engine3_load);
static LP55XX_DEV_ATTR_RW(master_fader1, show_master_fader1,
			  store_master_fader1);
static LP55XX_DEV_ATTR_RW(master_fader2, show_master_fader2,
			  store_master_fader2);
static LP55XX_DEV_ATTR_RW(master_fader3, show_master_fader3,
			  store_master_fader3);
static LP55XX_DEV_ATTR_RW(master_fader_leds, show_master_fader_leds,
			  store_master_fader_leds);

static struct attribute *lp5569_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_engine1_leds.attr,
	&dev_attr_engine2_leds.attr,
	&dev_attr_engine3_leds.attr,
	&dev_attr_master_fader1.attr,
	&dev_attr_master_fader2.attr,
	&dev_attr_master_fader3.attr,
	&dev_attr_master_fader_leds.attr,
	NULL,
};

static const struct attribute_group lp5569_group = {
	.attrs = lp5569_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5569_cfg = {
	.reset = {
		.addr = LP5569_REG_RESET,
		.val  = LP5569_RESET,
	},
	.enable = {
		.addr = LP5569_REG_CONFIG,
		.val  = LP5569_ENABLE,
	},
	.max_channel  = LP5569_MAX_LEDS,
	.post_init_device   = lp5569_post_init_device,
	.brightness_work_fn = lp5569_led_brightness_work,
	.set_led_current    = lp5569_set_led_current,
	.firmware_cb        = lp5569_firmware_loaded,
	.run_engine         = lp5569_run_engine,
	.dev_attr_group     = &lp5569_group,
};

static int lp5569_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
	struct lp55xx_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device_node *np = client->dev.of_node;

	if (!pdata) {
		if (np) {
			pdata = lp55xx_of_populate_pdata(&client->dev, np);
			if (IS_ERR(pdata))
				return PTR_ERR(pdata);

			/* If enable_gpio is not valid, try to get regulator */
			if (!gpio_is_valid(pdata->enable_gpio))
				pdata->reg =
					devm_regulator_get(&client->dev, "en");

		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5569_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

	if (!IS_ERR_OR_NULL(pdata->reg)) {
		ret = regulator_enable(pdata->reg);
		if (ret) {
			dev_err(&client->dev, "cannot enable reg\n");
			goto err_init;
		}
	}

	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	dev_info(&client->dev, "%s Programmable led chip found\n", id->name);

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}

	return 0;

err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return ret;
}

static int lp5569_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5569_stop_all_engines(chip);
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static const struct i2c_device_id lp5569_id[] = {
	{ "lp5569",  LP5569 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lp5569_id);

#ifdef CONFIG_OF
static const struct of_device_id of_lp5569_leds_match[] = {
	{ .compatible = "ti,lp5569", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp5569_leds_match);
#endif

static struct i2c_driver lp5569_driver = {
	.driver = {
		.name	= "lp5569x",
		.of_match_table = of_match_ptr(of_lp5569_leds_match),
	},
	.probe		= lp5569_probe,
	.remove		= lp5569_remove,
	.id_table	= lp5569_id,
};

module_i2c_driver(lp5569_driver);

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_AUTHOR("Tienchu Chang <tichang@sales.fb.com>");
MODULE_DESCRIPTION("LP5569 LED engine");
MODULE_LICENSE("GPL");