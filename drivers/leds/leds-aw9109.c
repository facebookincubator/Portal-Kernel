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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/leds-aw9109.h>
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW9109_I2C_NAME "aw9109_led"

#define AW9109_VERSION "v1.0.1"

#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		5
#define AW_READ_CHIPID_RETRIES		5
#define AW_READ_CHIPID_RETRY_DELAY	5

#define REG_RSTR	0x00
#define REG_GCR		0x01

#define REG_LER1	0x50
#define REG_LER2	0x51
#define REG_LCR		0x52
#define REG_PMR		0x53
#define REG_RMR		0x54
#define REG_CTRS1	0x55
#define REG_CTRS2	0x56
#define REG_IMAX1	0x57
#define REG_IMAX2	0x58
#define REG_IMAX3	0x59
#define REG_IMAX4	0x5a
#define REG_IMAX5	0x5b
#define REG_TIER	0x5c
#define REG_INTVEC	0x5d
#define REG_LISR2	0x5e
#define REG_SADDR	0x5f

#define REG_PCR		0x60
#define REG_CMDR	0x61
#define REG_RA		0x62
#define REG_RB		0x63
#define REG_RC		0x64
#define REG_RD		0x65
#define REG_R1		0x66
#define REG_R2		0x67
#define REG_R3		0x68
#define REG_R4		0x69
#define REG_R5		0x6a
#define REG_R6		0x6b
#define REG_R7		0x6c
#define REG_R8		0x6d
#define REG_GRPMSK1	0x6e
#define REG_GRPMSK2	0x6f

#define REG_TCR		0x70
#define REG_TAR		0x71
#define REG_TDR		0x72
#define REG_TDATA	0x73
#define REG_TANA	0x74
#define REG_TKST	0x75
#define REG_FEXT	0x76
#define REG_WP		0x7d
#define REG_WADDR	0x7e
#define REG_WDATA	0x7f


#define ALL_CHANNELS 0x7FC /* for i2c commands LER1, CTRS1, GRP */

/* aw9109 commands */
#define CMD_SETSTEPTMRI_HALF_MS 0x9F00 /* ramp step time all channels 0.5ms */
#define CMD_SETPWMI 0xA000
#define CMD_RAMPI_FADE_OUT 0xC000
#define CMD_RAMPI_FADE_IN 0xE000
#define CMD_CH_OFFSET 8 /* channel mask field offset for most commands */

/* fading factor when moving from center led for bar attr */
#define LED_BRIGHTNESS_GRANULE 51 /* 255/5 */

enum aw9109_program_control_mode {
	PROGMD_I2C,
	PROGMD_RELOAD_EXEC,
	PROGMD_RUN
};

enum aw9109_sram_program_run_mode {
	RUNMD_HOLD,
	RUNMD_STEP,
	RUNMD_RUN,
	RUNMD_REPEAT
};

/* aw9109 register read/write access */
#define REG_NONE_ACCESS		0
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)
#define AW9109_REG_MAX		0xFF

const unsigned char aw9109_reg_access[AW9109_REG_MAX] = {
	[REG_RSTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_GCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_LER1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_LER2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_LCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PMR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_RMR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_CTRS1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_CTRS2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_IMAX1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_IMAX2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_IMAX3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_IMAX4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_IMAX5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TIER] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_INTVEC] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_LISR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_SADDR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_CMDR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_RA] = REG_RD_ACCESS,
	[REG_RB] = REG_RD_ACCESS,
	[REG_RC] = REG_RD_ACCESS,
	[REG_RD] = REG_RD_ACCESS,
	[REG_R1] = REG_RD_ACCESS,
	[REG_R2] = REG_RD_ACCESS,
	[REG_R3] = REG_RD_ACCESS,
	[REG_R4] = REG_RD_ACCESS,
	[REG_R5] = REG_RD_ACCESS,
	[REG_R6] = REG_RD_ACCESS,
	[REG_R7] = REG_RD_ACCESS,
	[REG_R8] = REG_RD_ACCESS,
	[REG_GRPMSK1] = REG_RD_ACCESS,
	[REG_GRPMSK2] = REG_RD_ACCESS,
	[REG_TCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TAR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TDR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TDATA] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TANA] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_TKST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_FEXT] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_WP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_WADDR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_WDATA] = REG_RD_ACCESS | REG_WR_ACCESS
};


/******************************************************
 *
 * aw9109 led parameter
 *
 ******************************************************/
/* The definition of each time described as shown in figure.
 *        /-----------\
 *       /      |      \
 *      /|      |      |\
 *     / |      |      | \-----------
 *       |hold_time_ms |      |
 *       |             |      |
 * rise_time_ms  fall_time_ms |
 *                       off_time_ms
 */
#define ROM_CODE_MAX 255

/*
 * Default pattern in kernel firmware
 */
#define DEFAULT_AW9109_BA_PATTERN "aw9109/aw9109_boot.pat"

/*
 * rise_time_ms = 1500
 * hold_time_ms = 500
 * fall_time_ms = 1500
 * off_time_ms = 1500
 */
static int led_code_len = 7;
static int led_code[ROM_CODE_MAX] = {
	0xbf00, 0x9f05, 0xfffa, 0x3c7d, 0xdffa, 0x3cbb, 0x2,
};

/******************************************************
 *
 * aw9109 i2c write/read
 *
 ******************************************************/
static int i2c_write(struct aw9109 *aw9109,
		     unsigned char addr, unsigned int reg_data)
{
	int ret;
	unsigned char wbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr = aw9109->i2c->addr,
			.flags = 0,
			.len = 3,
			.buf = wbuf,
		},
	};

	wbuf[0] = addr;
	wbuf[1] = (unsigned char)(reg_data >> 8);
	wbuf[2] = (unsigned char)(reg_data & 0x00ff);

	ret = i2c_transfer(aw9109->i2c->adapter, msgs, 1);
	if (ret < 0)
		pr_err("%s: i2c write error: %d\n", __func__, ret);

	return ret;
}

static int i2c_read(struct aw9109 *aw9109,
		    unsigned char addr, unsigned int *reg_data)
{
	int ret;
	unsigned char rbuf[512] = {0};
	unsigned int get_data;

	struct i2c_msg msgs[] = {
		{
			.addr = aw9109->i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = aw9109->i2c->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = rbuf,
		},
	};

	ret = i2c_transfer(aw9109->i2c->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	get_data = (unsigned int)(rbuf[0] << 8);
	get_data |= (unsigned int)(rbuf[1]);

	*reg_data = get_data;

	return ret;
}

static int aw9109_i2c_write(struct aw9109 *aw9109,
			    unsigned char reg_addr, unsigned int reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw9109, reg_addr, reg_data);
		if (ret < 0)
			pr_err("%s: i2c_write cnt=%d error=%d\n",
			       __func__, cnt, ret);
		else
			break;
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw9109_i2c_read(struct aw9109 *aw9109,
			   unsigned char reg_addr, unsigned int *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw9109, reg_addr, reg_data);
		if (ret < 0)
			pr_err("%s: i2c_read cnt=%d error=%d\n",
			       __func__, cnt, ret);
		else
			break;
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw9109_i2c_write_bits(struct aw9109 *aw9109,
				 unsigned char reg_addr,
				 unsigned int mask, unsigned int reg_data)
{
	unsigned int reg_val;

	aw9109_i2c_read(aw9109, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw9109_i2c_write(aw9109, reg_addr, reg_val);

	return 0;
}

static inline void aw9109_enable_led_module(struct aw9109 *aw9109, int enable)
{
	aw9109_i2c_write_bits(aw9109, REG_GCR, 0xFFFE, enable);
}

static void aw9109_set_current(struct aw9109 *aw9109, unsigned int cur)
{
	unsigned int reg;
	reg = (cur << 12) | (cur << 8) | (cur << 4) | cur;

	/* IMAX1-LED1~LED2 Current */
	aw9109_i2c_write(aw9109, REG_IMAX1, reg);
	/* IMAX2-LED3~LED6 Current */
	aw9109_i2c_write(aw9109, REG_IMAX2, reg);
	/* IMAX3-LED7~LED9 Current */
	aw9109_i2c_write(aw9109, REG_IMAX3, reg);
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw9109_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw9109 *aw9109 = context;
	int i;
	unsigned char ram_cnt = 0;
	unsigned int ram_temp[256] = {0};

	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw9109->pattern);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw9109->pattern,
		cont ? cont->size : 0);

	for (i = 0; i < cont->size; i++)
		pr_info("%s: addr:0x%04x, data:0x%02x\n",
			__func__, i, *(cont->data + i));

	aw9109_enable_led_module(aw9109, 0);
	aw9109_enable_led_module(aw9109, 1);

	/* LED Config */
	/* LER1-LED1~LED9 Enable */
	aw9109_i2c_write(aw9109, REG_LER1, 0x07fc);
	aw9109_set_current(aw9109, aw9109->imax);
	/* CTRS1-LED1~LED9: SRAM Control */
	aw9109_i2c_write(aw9109, REG_CTRS1, 0x0000);

	/* LED SRAM Hold Mode */
	aw9109_i2c_write(aw9109, REG_PMR, PROGMD_I2C);
	aw9109_i2c_write(aw9109, REG_RMR, RUNMD_HOLD);

	/* Load LED SRAM */
	/* WADDR-SRAM Load Addr */
	aw9109_i2c_write(aw9109, REG_WADDR, 0x0000);
	for (i = 0; i < cont->size; i += 2) {
		ram_temp[ram_cnt] = *(cont->data + i) << 8;
		ram_temp[ram_cnt] |= *(cont->data + i + 1);
		ram_cnt++;
	}
	for (i = 0; i < ram_cnt; i++)
		aw9109_i2c_write(aw9109, REG_WDATA, ram_temp[i]);

	/* LED SRAM Run */
	/* SADDR-SRAM Run Start Addr:0 */
	aw9109_i2c_write(aw9109, REG_SADDR, 0x0000);
	aw9109_i2c_write(aw9109, REG_PMR, PROGMD_RELOAD_EXEC);
	aw9109_i2c_write(aw9109, REG_RMR, RUNMD_RUN);

	release_firmware(cont);

	pr_info("%s: fw update complete\n", __func__);
}

static int aw9109_ram_update(struct aw9109 *aw9109)
{
	const struct firmware *fw_entry;
	int ret = request_firmware(&fw_entry, aw9109->pattern, aw9109->dev);

	if (ret == 0)
		aw9109_ram_loaded(fw_entry, aw9109);
	return ret;
}

/******************************************************
 *
 * aw9109 led
 *
 ******************************************************/
static void aw9109_brightness_work(struct work_struct *work)
{
	struct aw9109 *aw9109 = container_of(work, struct aw9109,
					     brightness_work);

	if (aw9109->cdev.brightness > aw9109->cdev.max_brightness)
		aw9109->cdev.brightness = aw9109->cdev.max_brightness;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		goto unlock;
	}

	aw9109_enable_led_module(aw9109, 0);

	if (aw9109->cdev.brightness) {
		aw9109_enable_led_module(aw9109, 1);

		/* LED Config */
		/* LER1-LED1~LED9 Enable */
		aw9109_i2c_write(aw9109, REG_LER1, 0x07fc);
		aw9109_set_current(aw9109, aw9109->imax);
		/* CTRS1-LED1~LED9: i2c Control */
		aw9109_i2c_write(aw9109, REG_CTRS1, 0x07fc);

		/* LED Control */
		/* CMDR-LED1~LED9 */
		aw9109_i2c_write(aw9109, REG_CMDR,
				 0xBF00 | aw9109->cdev.brightness);
	}

unlock:
	mutex_unlock(&aw9109->lock);
}

static void aw9109_set_brightness(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct aw9109 *aw9109 = container_of(cdev, struct aw9109, cdev);

	aw9109->cdev.brightness = brightness;

	schedule_work(&aw9109->brightness_work);
}

static void aw9109_led_blink(struct aw9109 *aw9109, unsigned char blink)
{
	unsigned char i;

	aw9109_enable_led_module(aw9109, 0);

	if (blink) {
		aw9109_enable_led_module(aw9109, 1);

		/* LED Config */
		/* LER1-LED1~LED9 Enable */
		aw9109_i2c_write(aw9109, REG_LER1, 0x07fc);
		aw9109_set_current(aw9109, aw9109->imax);
		/* CTRS1-LED1~LED9: SRAM Control */
		aw9109_i2c_write(aw9109, REG_CTRS1, 0x0000);

		/* LED SRAM Hold Mode */
		aw9109_i2c_write(aw9109, REG_PMR, PROGMD_I2C);
		aw9109_i2c_write(aw9109, REG_RMR, RUNMD_HOLD);

		/* Load LED SRAM */
		/* WADDR-SRAM Load Addr */
		aw9109_i2c_write(aw9109, REG_WADDR, 0x0000);
		for (i = 0; i < led_code_len; i++)
			aw9109_i2c_write(aw9109, REG_WDATA, led_code[i]);

		/* LED SRAM Run */
		/* SADDR-SRAM Run Start Addr:0 */
		aw9109_i2c_write(aw9109, REG_SADDR, 0x0000);
		aw9109_i2c_write(aw9109, REG_PMR, PROGMD_RELOAD_EXEC);
		aw9109_i2c_write(aw9109, REG_RMR, RUNMD_RUN);
	}
}

static void aw9109_led_bar(struct aw9109 *aw9109, unsigned int level)
{
	int i;

	aw9109_enable_led_module(aw9109, 1);

	/* LED Config */
	/* LER1-LED1~LED9 Enable */
	aw9109_i2c_write(aw9109, REG_LER1, ALL_CHANNELS);
	aw9109_set_current(aw9109, aw9109->imax);
	/* CTRS1-LED1~LED9: i2c Control */
	aw9109_i2c_write(aw9109, REG_CTRS1, ALL_CHANNELS);

	/* 0.5ms ramp step time for fading */
	aw9109_i2c_write(aw9109, REG_CMDR, CMD_SETSTEPTMRI_HALF_MS);

	for (i = AW9109_CH_MIN; i <= AW9109_CH_MAX; i++) {
		int led_diff;
		/*
		 * Calculate new led brightness for led, gradually decrease
		 * brightness from center
		 * Center led = level
		 * 1st leds from center = level - K / 5
		 * 2nd leds from center = level - K / 5 * 2
		 * where level - desired new level,
		 * brightness - desired center led MAX brightness
		 */
		int led_bightness = level - LED_BRIGHTNESS_GRANULE *
				abs(i - AW9109_CH_MIDDLE);
		led_bightness = clamp(led_bightness, 0,
				(int)aw9109->cdev.max_brightness);

		led_diff = led_bightness - aw9109->last_bar_brightness[i];

		/*
		 * Force led brightness to saved level to make sure we
		 * are at expected level before issuing fading command,
		 * otherwise brightness will be out of sync
		 * Need to do that for case if previous fading is not
		 * finished yet or led brightness has been changed in
		 * any way
		 */
		aw9109_i2c_write(aw9109, REG_CMDR,
			CMD_SETPWMI | i << CMD_CH_OFFSET
			| aw9109->last_bar_brightness[i]);

		if (led_diff) {
			/* fade to new level */
			aw9109_i2c_write(aw9109, REG_CMDR,
				(led_diff > 0
					? CMD_RAMPI_FADE_IN
					: CMD_RAMPI_FADE_OUT)
				| i << CMD_CH_OFFSET | abs(led_diff));

			aw9109->last_bar_brightness[i] = led_bightness;
		}
	}
}

static void aw9109_led_clear(struct aw9109 *aw9109)
{
	aw9109_enable_led_module(aw9109, 0);
	aw9109_enable_led_module(aw9109, 1);
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw9109_parse_dt(struct device *dev, struct aw9109 *aw9109,
			   struct device_node *np)
{
	aw9109->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw9109->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided,will not HW reset device\n",
			__func__);
		return -EINVAL;
	}

	aw9109->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw9109->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided\n",__func__);
		return -EINVAL;
	}

	if (of_property_read_string(np, "ba_pattern", &aw9109->ba_pattern))
		aw9109->ba_pattern = DEFAULT_AW9109_BA_PATTERN;

	dev_info(dev, "%s: reset gpio provided ok\n", __func__);

	return 0;
}

static int aw9109_hw_reset(struct aw9109 *aw9109)
{
	pr_info("%s enter\n", __func__);

	if (aw9109 && gpio_is_valid(aw9109->reset_gpio)) {
		gpio_set_value_cansleep(aw9109->reset_gpio, 0);
		msleep(20);
		aw9109->chip_state = ST_RESET;
		gpio_set_value_cansleep(aw9109->reset_gpio, 1);
		msleep(20);
		aw9109->chip_state = ST_ACTIVE;
	} else {
		dev_err(aw9109->dev, "%s: failed\n", __func__);
	}
	return 0;
}

static int aw9109_hw_off(struct aw9109 *aw9109)
{
	pr_info("%s enter\n", __func__);

	if (aw9109 && gpio_is_valid(aw9109->reset_gpio)) {
		gpio_set_value_cansleep(aw9109->reset_gpio, 0);
		msleep(20);
		aw9109->chip_state = ST_RESET;
	} else {
		dev_err(aw9109->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw9109_read_chipid(struct aw9109 *aw9109)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned int reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw9109_i2c_read(aw9109, REG_RSTR, &reg_val);
		if (ret < 0) {
			dev_err(aw9109->dev,
				"%s: failed to read AW9109_REG_ID: %d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_val) {
		case 0xb223:
			pr_info("%s aw9109 detected\n", __func__);
				aw9109->chipid = AW9109_ID;
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw9109_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	unsigned int databuf[2] = {0, 0};
	int ret;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		ret = -EINVAL;
		goto unlock;
	}

	ret = sscanf(buf, "%x %x", &databuf[0], &databuf[1]);
	if (ret != 2) {
		dev_err(aw9109->dev, "sscanf failed, bailing out %s\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	aw9109_i2c_write(aw9109, databuf[0], databuf[1]);

	ret = count;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned int reg_val = 0;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		mutex_unlock(&aw9109->lock);
		return -EINVAL;
	}

	for (i = 0; i < AW9109_REG_MAX; i++) {
		if (!(aw9109_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw9109_i2c_read(aw9109, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%04x\n", i, reg_val);
	}

	mutex_unlock(&aw9109->lock);

	return len;
}

static ssize_t aw9109_hwen_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	int ret;
	unsigned int databuf[1] = {0};

	mutex_lock(&aw9109->lock);
	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret) {
		dev_err(aw9109->dev, "kstrtouint fail, bailing out %s\n",
			__func__);
		goto unlock;
	}

	if (databuf[0] == 1)
		aw9109_hw_reset(aw9109);
	else
		aw9109_hw_off(aw9109);

	ret = count;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_hwen_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	ssize_t len = 0;

	mutex_lock(&aw9109->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			aw9109->chip_state);
	mutex_unlock(&aw9109->lock);

	return len;
}

static ssize_t aw9109_ram_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	int ret;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (count >= PATTERN_LEN) {
		dev_err(aw9109->dev, "too long pattern name, bailing out %s\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	ret = sscanf(buf, "%s", aw9109->pattern);
	if (ret != 1) {
		dev_err(aw9109->dev, "sscanf failed, bailing out %s\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	aw9109_ram_update(aw9109);

	ret = count;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_ram_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	ssize_t len = 0;

	mutex_lock(&aw9109->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "%s\n", aw9109->pattern);
	mutex_unlock(&aw9109->lock);

	return len;
}

static ssize_t aw9109_blink_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	int ret;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		ret = -EINVAL;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret) {
		dev_err(aw9109->dev, "kstrtouint failed, bailing out %s\n",
			__func__);
		goto unlock;
	}

	aw9109_led_blink(aw9109, databuf[0]);

	ret = len;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_blink_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);

	mutex_lock(&aw9109->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw9109_blink()\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "echo 0 > blink\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "echo 1 > blink\n");
	mutex_unlock(&aw9109->lock);

	return len;
}

static ssize_t aw9109_bar_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	unsigned int databuf[1] = {0};
	int ret;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		ret = -EINVAL;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret) {
		dev_err(aw9109->dev, "kstrtouint failed, bailing out %s\n",
			__func__);
		goto unlock;
	}

	if (databuf[0] >= 255) {
		dev_err(aw9109->dev, "invalid bar value, bailing out %s\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	aw9109_led_bar(aw9109, databuf[0]);

	ret = count;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_bar_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);

	mutex_lock(&aw9109->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw9109_bar()\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "echo $NUM > bar\n");
	mutex_unlock(&aw9109->lock);

	return len;
}

static ssize_t aw9109_clear_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);
	unsigned int databuf[1];
	int ret;

	mutex_lock(&aw9109->lock);
	if (aw9109->chip_state == ST_RESET) {
		dev_err(aw9109->dev, "device in reset, bailing out %s\n",
				__func__);
		ret = -EINVAL;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret) {
		dev_err(aw9109->dev, "kstrtouint failed, bailing out %s\n",
			__func__);
		goto unlock;
	}

	if (databuf[0])
		aw9109_led_clear(aw9109);

	ret = count;

unlock:
	mutex_unlock(&aw9109->lock);

	return ret;
}

static ssize_t aw9109_clear_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw9109 *aw9109 = container_of(led_cdev, struct aw9109, cdev);

	mutex_lock(&aw9109->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw9109_clear()\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "echo 1 > clear\n");
	mutex_unlock(&aw9109->lock);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw9109_reg_show,
		   aw9109_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw9109_hwen_show,
		   aw9109_hwen_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw9109_ram_show,
		   aw9109_ram_store);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, aw9109_blink_show,
		   aw9109_blink_store);
static DEVICE_ATTR(bar, S_IWUSR | S_IRUGO, aw9109_bar_show,
		   aw9109_bar_store);
static DEVICE_ATTR(clear, S_IWUSR | S_IRUGO, aw9109_clear_show,
		   aw9109_clear_store);

static struct attribute *aw9109_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_ram.attr,
	&dev_attr_blink.attr,
	&dev_attr_bar.attr,
	&dev_attr_clear.attr,
	NULL
};

static struct attribute_group aw9109_attribute_group = {
	.attrs = aw9109_attributes
};


/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw9109_parse_led_cdev(struct aw9109 *aw9109,
				 struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw9109,name",
					      &aw9109->cdev.name);
		if (ret < 0) {
			dev_err(aw9109->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw9109,imax",
					   &aw9109->imax);
		if (ret < 0) {
			dev_err(aw9109->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw9109,brightness",
					   &aw9109->cdev.brightness);
		if (ret < 0) {
			dev_err(aw9109->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw9109,max_brightness",
					   &aw9109->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw9109->dev,
				"Failure reading max brightness, ret = %d\n",
				ret);
			goto free_pdata;
		}
	}

	INIT_WORK(&aw9109->brightness_work, aw9109_brightness_work);

	aw9109->cdev.brightness_set = aw9109_set_brightness;
	ret = led_classdev_register(aw9109->dev, &aw9109->cdev);
	if (ret) {
		dev_err(aw9109->dev,
			"unable to register led ret=%d\n", ret);
		goto free_pdata;
	}

	ret = sysfs_create_group(&aw9109->cdev.dev->kobj,
				 &aw9109_attribute_group);
	if (ret) {
		dev_err(aw9109->dev, "led sysfs ret: %d\n", ret);
		goto free_class;
	}

	return 0;

free_class:
	led_classdev_unregister(&aw9109->cdev);
free_pdata:
	return ret;
}

static irqreturn_t aw9109_irq(int irq, void *data)
{
	struct aw9109 *aw9109 = data;
	unsigned int reg_val = 0;

	mutex_lock(&aw9109->lock);
	aw9109_i2c_read(aw9109, REG_LISR2, &reg_val);
	mutex_unlock(&aw9109->lock);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * ram update used for during kernel boot
 *
 *****************************************************/
static void aw9109_ram_loaded2(const struct firmware *cont, void *context)
{
	struct aw9109 *aw9109 = context;

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw9109->pattern);
		return;
	}

	mutex_lock(&aw9109->lock);
	aw9109_hw_reset(aw9109);
	aw9109_ram_loaded(cont, context);
	mutex_unlock(&aw9109->lock);
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw9109_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aw9109 *aw9109;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags;
	int ret;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw9109 = devm_kzalloc(&i2c->dev, sizeof(struct aw9109), GFP_KERNEL);
	if (aw9109 == NULL)
		return -ENOMEM;

	aw9109->dev = &i2c->dev;
	aw9109->i2c = i2c;

	mutex_init(&aw9109->lock);
	i2c_set_clientdata(i2c, aw9109);

	/* aw9109 rst & int */
	if (np) {
		ret = aw9109_parse_dt(&i2c->dev, aw9109, np);
		if (ret < 0) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err;
		}
	} else {
		aw9109->reset_gpio = -1;
		aw9109->irq_gpio = -1;
	}

	if (gpio_is_valid(aw9109->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9109->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw9109_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err;
		}
	}

	if (gpio_is_valid(aw9109->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw9109->irq_gpio,
					    GPIOF_DIR_IN, "aw9109_irq");
		if (ret) {
			dev_err(&i2c->dev, "%s: irq gpio request failed\n",
				__func__);
			goto err;
		}

		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
                                        gpio_to_irq(aw9109->irq_gpio),
                                        NULL, aw9109_irq, irq_flags,
                                        "aw9109", aw9109);
		if (ret) {
			dev_err(&i2c->dev, "%s: irq request failed\n",
				__func__);
			goto err;
		}
	}

	/* hardware reset */
	aw9109_hw_reset(aw9109);

	/* aw9109 chip id */
	ret = aw9109_read_chipid(aw9109);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw9109_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	dev_set_drvdata(&i2c->dev, aw9109);

	aw9109_parse_led_cdev(aw9109, np);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s error creating led class dev\n",
			__func__);
		goto err_sysfs;
	}

	aw9109_hw_off(aw9109);

	if (aw9109->ba_pattern)
		request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG,
				       aw9109->ba_pattern, aw9109->dev,
				       GFP_KERNEL, aw9109, aw9109_ram_loaded2);

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
err_id:
err:
	return ret;
}

static int aw9109_i2c_remove(struct i2c_client *i2c)
{
	struct aw9109 *aw9109 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	if (gpio_is_valid(aw9109->reset_gpio))
		devm_gpio_free(&i2c->dev, aw9109->reset_gpio);

	return 0;
}

static const struct i2c_device_id aw9109_i2c_id[] = {
	{ AW9109_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw9109_i2c_id);

static const struct of_device_id aw9109_dt_match[] = {
	{ .compatible = "awinic,aw9109_led" },
	{ },
};

static struct i2c_driver aw9109_i2c_driver = {
	.driver = {
		.name = AW9109_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw9109_dt_match),
	},
	.probe = aw9109_i2c_probe,
	.remove = aw9109_i2c_remove,
	.id_table = aw9109_i2c_id,
};


static int __init aw9109_i2c_init(void)
{
	int ret = 0;

	pr_info("aw9109 driver version %s\n", AW9109_VERSION);

	ret = i2c_add_driver(&aw9109_i2c_driver);
	if (ret) {
		pr_err("fail to add aw9109 device into i2c\n");
		return ret;
	}

	return 0;
}
fs_initcall(aw9109_i2c_init);


static void __exit aw9109_i2c_exit(void)
{
	i2c_del_driver(&aw9109_i2c_driver);
}
module_exit(aw9109_i2c_exit);


MODULE_FIRMWARE(DEFAULT_AW9109_BA_PATTERN);
MODULE_DESCRIPTION("AW9109 LED Driver");
MODULE_LICENSE("GPL v2");
