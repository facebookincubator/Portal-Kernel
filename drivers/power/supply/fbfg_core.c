/*
 * Fbfg fuel gauge driver
 *
 * Copyright (c) 2021, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[fbfg_core] %s: " fmt, __func__

#include <asm/unaligned.h>

#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/fbfg_sbs.h>
#include <linux/power/fbfg_utils.h>
#include <linux/power/fbfg_prop.h>

#define CREATE_TRACE_POINTS
#include <trace/events/fbfg_battery.h>

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define fbfg_dbg pr_debug
#define fbfg_info pr_info
#define fbfg_log pr_info
#define fbfg_err pr_err


static void fbfg_update_status(struct fbfg_chip *chip);

/**
 * Do not add additional power supply property unless it is accessed by
 * the Android healthHal, other kernel driver, or used uding mfg proccess.
 */
static enum power_supply_property fbfg_props[] = {
	POWER_SUPPLY_PROP_CAPACITY, // Accessed by Health Hal
	POWER_SUPPLY_PROP_CHARGE_COUNTER, // Accessed by Health Hal
	POWER_SUPPLY_PROP_CHARGE_FULL, // Accessed by Health Hal
	POWER_SUPPLY_PROP_CHARGING_CURRENT, // Accessed by Charger IC
	POWER_SUPPLY_PROP_CHARGING_VOLTAGE, // Accessed by Charger IC
	POWER_SUPPLY_PROP_CURRENT_AVG, // Accessed by Health Hal
	POWER_SUPPLY_PROP_CURRENT_NOW, // Accessed by Health Hal
	POWER_SUPPLY_PROP_CYCLE_COUNT, // Accessed by Health Hal
	POWER_SUPPLY_PROP_HEALTH, // Accessed by Health Hal
	POWER_SUPPLY_PROP_PRESENT, // Accessed by Health Hal
	POWER_SUPPLY_PROP_REAL_CAPACITY, // Accessed during mfg proccess
	POWER_SUPPLY_PROP_SET_SHIP_MODE, // Accessed during mfg proccess
	POWER_SUPPLY_PROP_STATUS, // Accessed by Health Hal
	POWER_SUPPLY_PROP_TECHNOLOGY, // Accessed by Health Hal
	POWER_SUPPLY_PROP_TEMP, // Accessed by Health Hal
	POWER_SUPPLY_PROP_UPDATE_NOW, // Accessed by Charger IC
	POWER_SUPPLY_PROP_VOLTAGE_NOW, // Accessed by Health Hal
};

static int fbfg_prop_is_writeable(struct power_supply *psy,
				enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int __fbfg_read_word_raw(struct fbfg_chip *chip, u8 reg, u16 *val)
{
	s32 ret;
	int retries = 1;

	if (!chip && !chip->client) {
		fbfg_err("chip or client is null");
		*val = 0;
		return -EINVAL;
	}

	retries += chip->i2c_retry_limit;
	while (retries > 0) {
		ret = i2c_smbus_read_word_data(chip->client, reg);
		if (ret >= 0)
			break;
		retries--;
		msleep(100);
	}

	if (ret < 0) {
		fbfg_err("i2c read word fail: can't read from reg 0x%02X\n",
			reg);
		return ret;
	}

	*val = (u16)le16_to_cpu(ret);
	fbfg_dbg("reg (0x%02x) Val (0x%04x)", reg, *val);
	return 0;
}

static int __fbfg_write_word_raw(struct fbfg_chip *chip, u8 reg, u16 val)
{
	s32 ret;
	int retries = 1;

	if (!chip && !chip->client) {
		fbfg_err("chip or client is null");
		return 0;
	}

	retries += chip->i2c_retry_limit;

	while (retries > 0) {
		ret = i2c_smbus_write_word_data(chip->client,
			reg, le16_to_cpu(val));
		if (ret >= 0)
			break;
		retries--;
	}

	if (ret < 0) {
		fbfg_err("i2c write word failed: addr: 0x%02X val 0x%02X\n",
		       reg, val);
	}

	return 0;
}

static int fbfg_update_word(struct fbfg_chip *chip,
	enum FBFG_SBS_REG_IDX reg_idx, u16 mask, u16 value)
{
	u16 tmp;
	int ret;

	if (!chip && !chip->sbs_reg_data) {
		fbfg_err("Chip reg data is not available.");
		return -EINVAL;
	}

	if (reg_idx < 0 || reg_idx >= FBFG_SBS_REG_NR) {
		fbfg_err("Invalid reg idx value(%d)", reg_idx);
		return -EINVAL;
	}

	ret = __fbfg_read_word_raw(chip,
			chip->sbs_reg_data[reg_idx].addr, &tmp);
	if (ret < 0)
		return ret;

	tmp = ret & ~mask;
	tmp |= value & mask;

	return __fbfg_write_word_raw(chip,
			chip->sbs_reg_data[reg_idx].addr, tmp);
}

static int fbfg_read_word(struct fbfg_chip *chip,
	enum FBFG_SBS_REG_IDX reg_idx, int *val)
{
	int ret;
	u16 val_raw;

	if (!chip && !chip->sbs_reg_data) {
		fbfg_err("Chip reg data is not available.");
		*val = 0;
		return -EINVAL;
	}

	if (reg_idx < 0 || reg_idx >= FBFG_SBS_REG_NR) {
		fbfg_err("Invalid reg idx value(%d)", reg_idx);
		return -EINVAL;
	}

	ret = __fbfg_read_word_raw(chip,
			chip->sbs_reg_data[reg_idx].addr, &val_raw);
	if (ret < 0)
		return -EINVAL;

	if (chip->sbs_reg_data[reg_idx].min_value < 0)
		*val = (int)((s16)val_raw);
	else
		*val = (int)val_raw;

	return ret;
}

static int fbfg_write_word(struct fbfg_chip *chip,
	enum FBFG_SBS_REG_IDX reg_idx, u16 val) {

	if (!chip && !chip->sbs_reg_data) {
		fbfg_err("Chip reg data is not available.");
		return -EINVAL;
	}

	if (reg_idx < 0 || reg_idx >= FBFG_SBS_REG_NR) {
		fbfg_err("Invalid reg idx value(%d)", reg_idx);
		return -EINVAL;
	}
	return __fbfg_write_word_raw(chip,
		chip->sbs_reg_data[reg_idx].addr, val);
}

static int fbfg_read_block(struct fbfg_chip *chip, u8 cmd, u8 *buf, u8 len)
{
	u8 t_buf[I2C_SMBUS_BLOCK_MAX];
	struct i2c_msg msg[2];
	int ret;

	if (len > I2C_SMBUS_BLOCK_MAX)
		return -EINVAL;

	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &cmd;
	msg[0].len = 1;
	msg[1].addr = chip->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = t_buf;
	msg[1].len = len + 1;

	ret = i2c_transfer(chip->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		fbfg_err("Failed to fg read block:%d\n", ret);
		return ret;
	}

	if (t_buf[0] > len)
		return -EIO;

	memcpy(buf, &t_buf[1], t_buf[0]);
	return t_buf[0];
}

static int fbfg_read_mba(struct fbfg_chip *chip, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 t_buf[I2C_SMBUS_BLOCK_MAX + 3];
	struct i2c_msg msg[2];

	mutex_lock(&chip->mba_lock);

	if (len > I2C_SMBUS_BLOCK_MAX)
		len = I2C_SMBUS_BLOCK_MAX;

	ret = fbfg_write_word(chip, FBFG_SBS_REG_MFR_ACC, cmd);
	if (ret < 0)
		return ret;

	msleep(20);
	msg[0].addr = chip->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &chip->sbs_reg_data[FBFG_SBS_REG_MFR_DATA].addr;
	msg[0].len = 1;
	msg[1].addr = chip->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = t_buf;
	msg[1].len = len + 1;

	ret = i2c_transfer(chip->client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&chip->mba_lock);
	if (ret < 0) {
		fbfg_err("block read failed. ret(%d)", ret);
		return ret;
	}

	/* When reading a ManufacturerData() or dataflash
	 * the first byte will be the byte count
	 */
	memcpy(buf, &t_buf[1], len);

	return 0;
}

static int __fbfg_prop_read(struct fbfg_chip *chip,
	struct fbfg_prop_desc prop_desc,
	struct fbfg_propval *propval)
{
	int ret;
	u16 val_raw;
	int mult = (prop_desc.mult ? prop_desc.mult : 1);
	int add = (prop_desc.add ? prop_desc.add : 0);

	if (!prop_desc.name)
		return -ENODATA;

	if (prop_desc.op_type == SMBUS_WORD) {
		ret = __fbfg_read_word_raw(chip, prop_desc.addr, &val_raw);
		if (ret)
			return ret;

		if (prop_desc.offset > 16)
			return -EINVAL;
		else if (prop_desc.offset)
			val_raw = val_raw >> prop_desc.offset;

		if (prop_desc.mask)
			val_raw = val_raw & prop_desc.mask;

		switch (prop_desc.data_type) {
		case GG_U1:
		case GG_U2:
			propval->val.uintvals[0] =
			(uint32_t)((int)val_raw + add) * mult;
			break;
		case GG_S2:
			propval->val.intvals[0] =
			(int)((s16)(val_raw) + add) * mult;
			break;
		default:
				fbfg_dbg("Invalid data type for prop data.(%s)",
					prop_desc.name);
		}
		propval->len = 1;
		return 0;

	} else if (prop_desc.op_type == SMBUS_BLOCK) {
		ret = fbfg_read_block(chip, prop_desc.addr,
			propval->val.buf, prop_desc.block_sz);

		if (ret < 0 || ret > prop_desc.block_sz)
			return -EINVAL;

		propval->val.buf[ret] = '\0';
		propval->len = ret;

		if (prop_desc.data_type == GG_U4) {
			propval->val.uintvals[0] =
				(uint32_t)get_unaligned_le32(propval->val.buf);
			if (prop_desc.mask)
				propval->val.uintvals[0] =
				propval->val.uintvals[0] & prop_desc.mask;
		}
	} else if (prop_desc.op_type == SMBUS_MAC) {
		u8 tmpbuf[I2C_SMBUS_BLOCK_MAX];
		size_t bufsize =
			prop_desc.num_param
			* fbfg_prop_type_size(prop_desc.data_type);
		int i;
		int itr_max = (prop_desc.num_param == PARAM_NUM_CELL ?
				PARAM_NUM_FOUR : prop_desc.num_param);

		if (prop_desc.offset + bufsize > I2C_SMBUS_BLOCK_MAX)
			return -EINVAL;

		ret = fbfg_read_mba(chip,
			prop_desc.addr, tmpbuf, sizeof(tmpbuf));

		if (ret < 0) {
			fbfg_info("mba read failed");
			return -EINVAL;
		}

		propval->len = 0;
		for (i = 0; i < itr_max; i++) {
			switch (prop_desc.data_type) {
			case GG_U1:
				propval->val.uintvals[i] =
				  (int)(tmpbuf[prop_desc.offset + i * 1])
				  * mult;
			break;
			case GG_S1:
				propval->val.intvals[i] =
				  (int)(int8_t)
				  (tmpbuf[prop_desc.offset + i * 1])
				   * mult;
				break;
			case GG_U2:
				propval->val.uintvals[i] =
				  (uint32_t)(uint16_t)
				  get_unaligned_le16(
				  &tmpbuf[prop_desc.offset + i * 2])
				   * mult;
				break;
			case GG_S2:
				propval->val.intvals[i] =
				  (int)(int16_t)
				  get_unaligned_le16(
				  &tmpbuf[prop_desc.offset + i * 2])
				  * mult;
				break;
			case GG_U4:
				propval->val.uintvals[i] =
				  (uint32_t)(uint32_t)
				  get_unaligned_le32(
				  &tmpbuf[prop_desc.offset + i * 4])
				   * mult;
				break;
			default:
				break;
			}
			propval->len++;
		}
	}
	return 0;
}


static int print_params_to_sysfs(char *buf,
	struct fbfg_prop_desc prop_desc, struct fbfg_propval prop_val)
{
	int num_param = prop_desc.num_param;
	enum FBFG_PROP_VAL_TYPE data_type = prop_desc.data_type;

	if (data_type == GG_ST)
		return snprintf(buf, PAGE_SIZE, "%s\n",
			prop_val.val.buf);

	switch (num_param) {
	case PARAM_NUM_ONE:
		switch (data_type) {
		case GG_S1:
			/*FALLTHROUGH*/
		case GG_S2:
			return snprintf(buf, PAGE_SIZE, "%d\n",
			prop_val.val.intvals[0]);
			/*FALLTHROUGH*/
		case GG_U1:
			/*FALLTHROUGH*/
		case GG_U2:
			/*FALLTHROUGH*/
		case GG_U4:
			return snprintf(buf, PAGE_SIZE, "%u\n",
			prop_val.val.uintvals[0]);
			/*FALLTHROUGH*/
		default:
			return -EINVAL;
		}
		break;
	case PARAM_NUM_CELL:
	case PARAM_NUM_FOUR:
		switch (data_type) {
		case GG_U1:
			/*FALLTHROUGH*/
		case GG_U2:
			/*FALLTHROUGH*/
		case GG_U4:
			return snprintf(buf, PAGE_SIZE, "%u, %u, %u, %u\n",
			prop_val.val.uintvals[0],
			prop_val.val.uintvals[1],
			prop_val.val.uintvals[2],
			prop_val.val.uintvals[3]);
			/*FALLTHROUGH*/
		case GG_S1:
			/*FALLTHROUGH*/
		case GG_S2:
			return snprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
			prop_val.val.intvals[0],
			prop_val.val.intvals[1],
			prop_val.val.intvals[2],
			prop_val.val.intvals[3]);
			/*FALLTHROUGH*/
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static int fbfg_sbs_prop_read(struct fbfg_chip *chip,
	enum FBFG_SBS_PROP_IDX prop_idx, struct fbfg_propval *propval)
{
	if (prop_idx < 0 || prop_idx > FBFG_SBS_PROP_NR)
		return -EINVAL;

	return __fbfg_prop_read(chip, chip->sbs_prop_descs[prop_idx],
		propval);
}

static ssize_t fbfg_sbs_prop_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fbfg_chip *chip = i2c_get_clientdata(client);
	struct fbfg_prop_attribute *prop_attr = container_of(devattr,
		struct fbfg_prop_attribute, da);
	enum FBFG_SBS_PROP_IDX prop_idx = prop_attr->fbfg_prop_idx;
	struct fbfg_prop_desc prop_desc;
	struct fbfg_propval propval;
	int ret;

	if (!chip->sbs_prop_descs)
		return snprintf(buf, PAGE_SIZE, "na\n");

	prop_desc = chip->sbs_prop_descs[prop_idx];
	if (!prop_desc.name)
		return snprintf(buf, PAGE_SIZE, "na\n");

	ret = fbfg_sbs_prop_read(chip, prop_idx, &propval);
	if (ret < 0)
		return ret;

	return print_params_to_sysfs(buf, prop_desc, propval);
}



static int fbfg_prop_read(struct fbfg_chip *chip,
	enum FBFG_PROP_IDX prop_idx, struct fbfg_propval *propval)
{
	if (prop_idx < 0 || prop_idx > FBFG_PROP_NR)
		return -EINVAL;

	return __fbfg_prop_read(chip, chip->prop_descs[prop_idx],
		propval);
}
static ssize_t fbfg_prop_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fbfg_chip *chip = i2c_get_clientdata(client);
	struct fbfg_prop_attribute *prop_attr = container_of(devattr,
		struct fbfg_prop_attribute, da);
	enum FBFG_PROP_IDX prop_idx = prop_attr->fbfg_prop_idx;
	struct fbfg_prop_desc prop_desc;
	struct fbfg_propval propval;
	int ret;

	if (!chip->prop_descs)
		return snprintf(buf, PAGE_SIZE, "na\n");

	prop_desc = chip->prop_descs[prop_idx];
	if (!prop_desc.name)
		return snprintf(buf, PAGE_SIZE, "na\n");

	ret = fbfg_prop_read(chip, prop_idx, &propval);
	if (ret < 0)
		return ret;

	return print_params_to_sysfs(buf, prop_desc, propval);
}

static int fbfg_get_raw_batt_sts(struct fbfg_chip *chip,
	struct fbfg_batt_sts *batt_sts)
{
	int ret, flags;

	if (!batt_sts)
		return -EINVAL;

	ret = fbfg_read_word(chip, FBFG_SBS_REG_BATT_STS, &flags);
	if (ret) {
		fbfg_err("Read FBFG_SBS_REG_BATT_STS failed. ret(%d)", ret);
		return ret;
	}

	batt_sts->batt_fd = !!(flags & FBFG_FLAGS_FD);
	batt_sts->batt_fc = !!(flags & FBFG_FLAGS_FC);
	batt_sts->batt_dsg = !!(flags & FBFG_FLAGS_DSG);
	batt_sts->batt_init = !!(flags & FBFG_FLAGS_INIT);
	batt_sts->batt_rca = !!(flags & FBFG_FLAGS_RCA);
	return ret;
}

/**
 * return status of battery.
 *  If smbus comm to read BATT_STS fails, use cached value to
 *  determine the status. If smbus cmm fails, cached value is
 * invalid or if fg init is not initialized yet,
 * return POWER_SUPPLY_STATUS_UNKNOWN.
 */
static int fbfg_get_batt_status(struct fbfg_chip *chip)
{
	int ret;
	struct fbfg_batt_sts batt_sts;
	int curr_now;

	if (!chip->batt_ok)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = fbfg_get_raw_batt_sts(chip, &batt_sts);

	if (ret != 0 || !batt_sts.batt_init) {
		fbfg_err("Could not read status, ret=%d.\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (batt_sts.batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	if (batt_sts.batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = fbfg_read_word(chip, FBFG_SBS_REG_CURR_NOW, &curr_now);
	if (ret) {
		fbfg_err("Unable to read FBFG_SBS_REG_CURR_NOW ret=%d.\n",
			ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (curr_now > 0)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int fbfg_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct fbfg_chip *chip = power_supply_get_drvdata(psy);
	struct fbfg_platform *pdata = chip->pdata;

	int ret = 0;
	int prop_val;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pdata->fake_soc_en) {
			val->intval = pdata->fake_soc;
			break;
		}
		if (!chip->batt_ok) {
			val->intval = DEFAULT_RSOC;
			break;
		}
		val->intval = pdata->soc;
		break;
	case POWER_SUPPLY_PROP_REAL_CAPACITY:
		val->intval = pdata->real_soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = pdata->remaining_cap;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = pdata->charge_full;
		break;
	case POWER_SUPPLY_PROP_CHARGING_CURRENT:
		val->intval = pdata->chg_curr;
		break;
	case POWER_SUPPLY_PROP_CHARGING_VOLTAGE:
		val->intval = pdata->chg_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (!chip->batt_ok) {
			val->intval = 0;
			break;
		}
		ret = fbfg_read_word(chip, FBFG_SBS_REG_CURR_AVG, &prop_val);
		if (ret) {
			fbfg_err("Unable to read current now.");
			prop_val = 0;
		}
		// Convert mA to uA
		val->intval = prop_val * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!chip->batt_ok) {
			val->intval = 0;
			break;
		}
		ret = fbfg_read_word(chip, FBFG_SBS_REG_CURR_NOW, &prop_val);
		if (!ret) {
			prop_val *= 1000;
			val->intval = prop_val;
			pdata->curr_now = prop_val;
		} else {
			val->intval = pdata->curr_now;
			fbfg_err("Unable to read current now.");
		}
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = pdata->cycle_cnt;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (!chip->batt_ok) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			if (chip->init_retry_count > chip->init_retry_limit)
				val->intval = POWER_SUPPLY_HEALTH_DEAD;
			break;
		}

		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (pdata->pf_status != 0)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fbfg_get_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->technology;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (pdata->fake_temp) {
			val->intval = pdata->fake_temp;
			break;
		}
		if (!chip->batt_ok) {
			val->intval = DEFAULT_TEMP;
			break;
		}
		ret = fbfg_read_word(chip, FBFG_SBS_REG_TEMP, &prop_val);
		// Convert 0.1K to 0.1C
		if (ret) {
			val->intval = pdata->temp;
			break;
		}
		prop_val -= 2730;
		/**  If the temperature is more than 64.0C,
		 * it is likely a false reading. Mark error and return
		 * cached temperature data.
		 */
		if (prop_val > 640) {
			chip->err_temp_cnt++;
			prop_val = pdata->temp;
		}
		pdata->temp = prop_val;
		val->intval = prop_val;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!chip->batt_ok) {
			val->intval = 0;
			break;
		}
		ret = fbfg_read_word(chip, FBFG_SBS_REG_VOLT_NOW, &prop_val);
		if (!ret) {
			prop_val *= 1000;
			val->intval = prop_val;
			pdata->volt_now = prop_val;
		} else {
			val->intval = pdata->volt_now;
			fbfg_err("Unable to read voltage now.");
		}
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		/* No Op */
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fbfg_set_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   const union power_supply_propval *val)
{
	struct fbfg_chip *chip = power_supply_get_drvdata(psy);
	struct fbfg_platform *pdata = chip->pdata;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		if (pdata->fake_temp != val->intval) {
			pdata->fake_temp = val->intval;
			power_supply_changed(chip->power_supply);
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (val->intval < 0)
			pdata->fake_soc_en = false;
		else
			pdata->fake_soc_en = true;
		pdata->fake_soc = val->intval;
		power_supply_changed(chip->power_supply);
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		if (chip->batt_ok) {
			alarm_cancel(&chip->monitor_alarm);
			cancel_delayed_work_sync(&chip->monitor_work);
			fbfg_update_status(chip);
			/**
			 * Make sure we don't hold wakelock
			 * even in case the work thread has been
			 * canceled
			 */
			if (chip->pm_stay_awake) {
				chip->pm_stay_awake = false;
				pm_relax(chip->dev);
			}
			alarm_start_relative(&chip->monitor_alarm,
				ms_to_ktime(chip->monitor_period_ms));
		}
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		if (val->intval) {
			ret = fbfg_write_word(chip,
				FBFG_SBS_REG_MFR_ACC, FBFG_MAC_CMD_SHUTDOWN);
			if (ret) {
				fbfg_err("1/2 shipmode subcommand failed");
				break;
			}
			ret = fbfg_write_word(chip,
				 FBFG_SBS_REG_MFR_ACC, FBFG_MAC_CMD_SHUTDOWN);
			if (ret) {
				fbfg_err("2/2 shipmode subcommand failed");
				break;
			}
			fbfg_info("Completed sending shipmode commands");
		}
		break;
	default:
		break;
	}
	return 0;
}

static void fbfg_update_status(struct fbfg_chip *chip)
{
	int ret, val;
	struct fbfg_propval propval;
	bool safety_sts_chg = false;
	bool notif_sts_chg = false;
	struct fbfg_platform *pdata = chip->pdata;

	if (!chip->batt_ok)
		return;

	mutex_lock(&chip->data_lock);
	ret = fbfg_read_word(chip, FBFG_SBS_REG_REL_SOC, &val);
	if (!ret && pdata->real_soc != val) {
		pdata->real_soc = val;
		notif_sts_chg = true;
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_REM_CAP, &val);
	if (!ret) {
		// Convert mAh to uAh
		val = val * 1000;
		if (pdata->remaining_cap != val) {
			pdata->remaining_cap = val;
			notif_sts_chg = true;
		}
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_FCC, &val);
	if (!ret) {
		// Convert from mAh to uAh
		val *= 1000;
		if (pdata->charge_full != val) {
			pdata->charge_full = val;
			notif_sts_chg = true;
		}
	}

	if (chip->soc_remap) {
		ret = fbfg_utils_soc_static_remap(pdata->real_soc,
			pdata->charge_full,
			pdata->remaining_cap, &val);
	} else {
		ret = 0;
		val = pdata->soc;
	}

	if (!ret && pdata->soc != val) {
		pdata->soc = val;
		notif_sts_chg = true;
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_CYC_CNT, &val);
	if (!ret && pdata->cycle_cnt != val) {
		pdata->cycle_cnt = val;
		notif_sts_chg = true;
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_TEMP, &val);
	if (!ret) {
		// Convert 0.1K to 0.1C
		val -= 2730;
		if (pdata->temp != val) {
			pdata->temp = val;
			notif_sts_chg = true;
		}
	}

	val = fbfg_get_batt_status(chip);
	if (val != pdata->batt_status) {
		pdata->batt_status = val;
		notif_sts_chg = true;
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_CHG_CURR, &val);
	if (!ret) {
		//Convert mA to uA
		val *= 1000;
		if (pdata->chg_curr != val) {
			pdata->chg_curr = val;
			notif_sts_chg = true;
		}
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_CHG_VOLT, &val);
	if (!ret) {
		//Convert mV to uV
		val *= 1000;
		if (pdata->chg_volt != val) {
			pdata->chg_volt = val;
			notif_sts_chg = true;
		}
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_CURR_NOW, &val);
	if (!ret) {
		// Convert mA to uA
		val *= 1000;
		if (pdata->curr_now != val) {
		// Log&notify only when the current is changed by more than 5mA.
			if (abs(pdata->curr_now - val) > 5000)
				notif_sts_chg = true;
			pdata->curr_now = val;
		}
	}

	ret = fbfg_read_word(chip, FBFG_SBS_REG_VOLT_NOW, &val);
	if (!ret) {
		// Convert mV to uV
		val *= 1000;
		if (pdata->volt_now != val) {
		// Log&notify only when the voltage is changed by more than 5mV.
			if (abs(pdata->volt_now - val) > 5000)
				notif_sts_chg = true;
			pdata->volt_now = val;
		}
	}

	ret = fbfg_prop_read(chip, FBFG_PROP_SAFETY_STATUS, &propval);
	if (!ret && pdata->safety_status != propval.val.uintvals[0]) {
		pdata->safety_status = propval.val.uintvals[0];
		safety_sts_chg = true;
	}

	ret = fbfg_prop_read(chip, FBFG_PROP_OP_STATUS, &propval);
	if (!ret && pdata->op_status != propval.val.uintvals[0]) {
		pdata->op_status = propval.val.uintvals[0];
		safety_sts_chg = true;
	}

	ret = fbfg_prop_read(chip, FBFG_PROP_PF_STATUS, &propval);
	if (!ret && pdata->pf_status != propval.val.uintvals[0]) {
		pdata->pf_status = propval.val.uintvals[0];
		safety_sts_chg = true;
	}

	ret = fbfg_prop_read(chip, FBFG_PROP_GAUGE_ALERT, &propval);
	if (!ret && pdata->gauge_alert != propval.val.uintvals[0]) {
		pdata->gauge_alert = propval.val.uintvals[0];
		safety_sts_chg = true;
	}
	mutex_unlock(&chip->data_lock);

	if (notif_sts_chg) {
		// TODO: T102947939 Send Uvent from FG driver.
		// power_supply_changed(chip->power_supply);
		fbfg_log("RSOC:%d, DSOC:%d ChgCurr:%d ChgVolt:%d",
			pdata->real_soc, pdata->soc, pdata->chg_curr,
			pdata->chg_volt);
		fbfg_log("Curr:%d Volt:%d Temp:%d\n",
			pdata->curr_now, pdata->volt_now, pdata->temp);
		trace_fbfg_update_status(pdata->real_soc, pdata->soc,
			pdata->chg_curr, pdata->chg_volt, pdata->curr_now,
			pdata->volt_now, pdata->temp);
	}

	if (safety_sts_chg) {
		fbfg_log("SafetySTS:0x%04x OP_STS:0x%04x Gauge_Alert:0x%04x\n",
			pdata->safety_status, pdata->op_status,
			pdata->gauge_alert);
		trace_fbfg_safety_status(pdata->safety_status,
			pdata->op_status, pdata->gauge_alert);
	}
}

static void fbfg_monitor_workfunc(struct work_struct *work)
{
	int ret;
	struct fbfg_chip *chip =
		container_of(work, struct fbfg_chip, monitor_work.work);

	if (!chip->batt_ok)
		return;

	if (!chip->resume_completed) {
		ret = wait_for_completion_timeout(
			&chip->resume_done,
			msecs_to_jiffies(FBFG_RESUME_TIMEOUT_MS));
		if (!ret) {
			fbfg_err("resume didn't happen\n");
			goto exit;
		}
	}
	fbfg_update_status(chip);

exit:
	alarm_start_relative(&chip->monitor_alarm,
		ms_to_ktime(chip->monitor_period_ms));

	chip->pm_stay_awake = false;
	pm_relax(chip->dev);
}


static enum alarmtimer_restart fbfg_monitor_alarmcb(struct alarm *alarm,
							ktime_t now)
{
	struct fbfg_chip *chip =
		container_of(alarm, struct fbfg_chip, monitor_alarm);

	pm_stay_awake(chip->dev);
	chip->pm_stay_awake = true;
	schedule_delayed_work(&chip->monitor_work, 0);

	return ALARMTIMER_NORESTART;
}

static int fbfg_log_hw_info(struct fbfg_chip *chip)
{
	int ret;
	struct fbfg_propval propval;

	ret = fbfg_prop_read(chip, FBFG_PROP_FIRMWARE_VERSION, &propval);
	if (!ret)
		fbfg_log("FW Ver: 0x0%02x", propval.val.uintvals[0]);

	ret = fbfg_prop_read(chip, FBFG_PROP_FIRMWARE_SUBVER, &propval);
	if (!ret)
		fbfg_log("FW subver: 0x0%02x", propval.val.uintvals[0]);

	ret = fbfg_sbs_prop_read(chip, FBFG_SBS_PROP_MAF_NAME, &propval);
	if (!ret)
		fbfg_log("Manufacturer name: %s", propval.val.buf);

	ret = fbfg_sbs_prop_read(chip, FBFG_SBS_PROP_DEV_NAME, &propval);
	if (!ret)
		fbfg_log("Device name: %s", propval.val.buf);

	ret = fbfg_sbs_prop_read(chip, FBFG_SBS_PROP_DEV_CHEM, &propval);
	if (!ret) {
		fbfg_log("Device chem: %s", propval.val.buf);
		if (!strcmp(propval.val.buf, "LION"))
			chip->pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		else
			chip->pdata->technology = POWER_SUPPLY_TECHNOLOGY_LIPO;

	}
	return 0;
}

static int fbfg_init(struct fbfg_chip *chip)
{
	int ret;
	int real_soc;
	int batt_status;
	int prop_idx;

	fbfg_info("start.");

	/* Set capacity mode to zero, to report in unit of mAh */
	ret = fbfg_update_word(chip, FBFG_SBS_REG_BATT_MODE,
		FBFG_CAP_MODE, 0);
	if (ret)
		fbfg_err("Unable to change cap mode");

	chip->resume_completed = true;
	chip->batt_ok = true;

	if (chip->soc_remap) {
		batt_status = fbfg_get_batt_status(chip);
		ret = fbfg_read_word(chip, FBFG_SBS_REG_REL_SOC, &real_soc);
		if (!ret) {
			fbfg_err("Unable to read rsoc");
			fbfb_utils_set_init_status(real_soc, batt_status);
		}
	}

	for (prop_idx = 0; prop_idx < FBFG_PROP_NR; prop_idx++) {
		chip->prop_das[prop_idx].fbfg_prop_idx = prop_idx;
		if (!chip->prop_descs || !chip->prop_descs[prop_idx].name)
			continue;

		sysfs_attr_init(&chip->prop_da[prop_idx].da.attr);
		chip->prop_das[prop_idx].da.attr.mode = 0444;
		chip->prop_das[prop_idx].da.attr.name =
			fbfg_prop_names[prop_idx].name;
		chip->prop_das[prop_idx].da.show = fbfg_prop_show;
		chip->prop_das[prop_idx].da.store = NULL;

		ret = device_create_file(chip->dev,
			&chip->prop_das[prop_idx].da);
		if (ret)
			fbfg_err("Can't create sysfs file %s\n",
				fbfg_prop_names[prop_idx].name);
	}

	for (prop_idx = 0; prop_idx < FBFG_SBS_PROP_NR; prop_idx++) {
		sysfs_attr_init(&chip->sbs_prop_das[prop_idx].da.attr);
		chip->sbs_prop_das[prop_idx].da.attr.mode = 0444;
		chip->sbs_prop_das[prop_idx].da.attr.name =
			fbfg_sbs_props_data[prop_idx].name;
		chip->sbs_prop_das[prop_idx].da.show = fbfg_sbs_prop_show;
		chip->sbs_prop_das[prop_idx].da.store = NULL;
		chip->sbs_prop_das[prop_idx].fbfg_prop_idx = prop_idx;
		ret = device_create_file(chip->dev,
			&chip->sbs_prop_das[prop_idx].da);
		if (ret)
			fbfg_err("Can't create sysfs file %s\n",
				fbfg_sbs_props_data[prop_idx].name);
	}

	fbfg_update_status(chip);
	device_init_wakeup(chip->dev, true);
	chip->pm_stay_awake =  false;

	init_completion(&chip->resume_done);

	alarm_init(&chip->monitor_alarm,
			ALARM_BOOTTIME, fbfg_monitor_alarmcb);

	alarm_start_relative(&chip->monitor_alarm,
			ms_to_ktime(chip->monitor_period_ms));

	ret = fbfg_log_hw_info(chip);
	if (ret)
		fbfg_err("Error parsing hw data");

	fbfg_log("fuel gauge driver initialization completed\n");
	return 0;
}

static void fbfg_delayed_init_workfunc(struct work_struct *work)
{
	struct fbfg_chip *chip =
		container_of(work, struct fbfg_chip, delayed_init_work.work);
	struct fbfg_batt_sts batt_sts;
	int ret;

	fbfg_info("Start delayed init cnt:%d", chip->init_retry_count);
	ret = fbfg_get_raw_batt_sts(chip, &batt_sts);

	if (!ret && batt_sts.batt_init) {
		fbfg_init(chip);
		return;
	}

	chip->init_retry_count++;
	if (chip->init_retry_count > chip->init_retry_limit) {
		fbfg_err("Battery not detected, giving up!\n");
		return;
	}

	schedule_delayed_work(&chip->delayed_init_work, 5 * HZ);
}

static int fbfg_parse_dt(struct fbfg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int ret;

	if (!node) {
		fbfg_dbg("no device tree node\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "fb,init_max_retries",
					&chip->init_retry_limit);
	if (ret < 0) {
		fbfg_dbg("no fb,init_max_retries definition\n");
		chip->init_retry_limit = DEFAULT_MAX_INIT_RETRY_COUNT;
	}

	ret = of_property_read_u32(node, "renesas,i2c_retry_limit",
					&chip->i2c_retry_limit);
	if (ret < 0)
		fbfg_dbg("no renesas,i2c_retry_limit definition\n");

	ret = of_property_read_u32(node, "fb,monitor_period_ms",
					&chip->monitor_period_ms);
	if (ret < 0) {
		fbfg_dbg("no fb,monitor_period_ms set to default val. %d\n",
			DEFAULT_POLL_INTERVAL_MS);
		chip->monitor_period_ms = DEFAULT_POLL_INTERVAL_MS;
	}

	ret = of_property_read_u32(node, "fb,num_cells",
					&chip->num_cells);
	if (ret < 0) {
		fbfg_dbg("no fb,num_cells. Assume battery has 4 cells.\n");
		chip->num_cells = DEFAULT_NUM_CELLS;
	}
	ret = fbfg_utils_init(node);
	if (!ret)
		chip->soc_remap = true;
	else
		chip->soc_remap = false;

	fbfg_info("init_retry_max:%d i2c_retry_limit:%d\n",
		chip->init_retry_limit, chip->i2c_retry_limit);
	fbfg_info("monitor_period_ms:%d num_cells:%d soc_remap:%s\n",
		chip->monitor_period_ms, chip->num_cells,
		chip->soc_remap ? "true" : "false");
	return 0;
}

static int fbfg_psy_register(struct fbfg_chip *chip)
{
	struct power_supply_config fg_psy_cfg = {};

	chip->psy_desc.name = "fbfg_battery";
	chip->psy_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->psy_desc.properties = fbfg_props;
	chip->psy_desc.num_properties = ARRAY_SIZE(fbfg_props);
	chip->psy_desc.get_property = fbfg_get_property;
	chip->psy_desc.set_property = fbfg_set_property;
	chip->psy_desc.property_is_writeable = fbfg_prop_is_writeable;

	fg_psy_cfg.drv_data = chip;
	fg_psy_cfg.num_supplicants = 0;
	chip->power_supply =
		devm_power_supply_register(chip->dev,
			&chip->psy_desc, &fg_psy_cfg);
	if (IS_ERR(chip->power_supply)) {
		fbfg_err("Failed to register power_supply");
		return PTR_ERR(chip->power_supply);
	}
	return 0;
}

static const struct of_device_id fbfg_match_table[] = {
	{
		.compatible = "fb,fbfg_sbs",
		.data = NULL,
	},
	{
		.compatible = "fb,fbfg_raj240045",
		.data = &raj240045_props_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, fbfg_match_table);

enum fbfg_device {
	RAJ240045,
};

static const struct i2c_device_id fbfg_id[] = {
	{ "fb,fbfg_sbs", 0 },
	{ "fb,fbfg_raj240045", RAJ240045},
	{},
};
MODULE_DEVICE_TABLE(i2c, fbfg_id);

static int fbfg_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	struct fbfg_chip *chip;
	struct fbfg_platform *pdata;
	const struct of_device_id *match_id;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);

	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		devm_kfree(&client->dev, chip);
		return -ENOMEM;
	}
	chip->pdata = pdata;
	pdata->temp = DEFAULT_TEMP;
	pdata->fake_temp = 0;
	pdata->real_soc = DEFAULT_RSOC;
	pdata->soc = DEFAULT_RSOC;

	pdata->safety_status = -ENODATA;
	pdata->op_status = -ENODATA;
	pdata->gauge_alert = -ENODATA;


	match_id = of_match_device(fbfg_match_table, &client->dev);
	if (!match_id) {
		fbfg_err("Could not find a matching device");
		return -ENODEV;
	}

	chip->sbs_reg_data = fbfg_sbs_data;
	chip->sbs_prop_descs = fbfg_sbs_props_data;

	chip->prop_descs = (struct fbfg_prop_desc *)match_id->data;

	i2c_set_clientdata(client, chip);

	ret = fbfg_parse_dt(chip);
	if (ret < 0)
		fbfg_err("Failed to parse DT parameters, ret:%d\n", ret);

	mutex_init(&chip->data_lock);
	mutex_init(&chip->mba_lock);

	INIT_DELAYED_WORK(&chip->monitor_work, fbfg_monitor_workfunc);
	INIT_DELAYED_WORK(&chip->delayed_init_work, fbfg_delayed_init_workfunc);

	fbfg_psy_register(chip);

	fbfg_log("Fb fgprobe successfully. Comtatible:%s\n",
		match_id->compatible);

	schedule_delayed_work(&chip->delayed_init_work, 0);
	return 0;
}

static int fbfg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fbfg_chip *chip = i2c_get_clientdata(client);

	chip->resume_completed = false;
	reinit_completion(&chip->resume_done);

	return 0;
}

static int fbfg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fbfg_chip *chip = i2c_get_clientdata(client);

	chip->resume_completed = true;
	complete(&chip->resume_done);
	power_supply_changed(chip->power_supply);

	return 0;
}

static int fbfg_remove(struct i2c_client *client)
{
	struct fbfg_chip *chip = i2c_get_clientdata(client);
	int prop_idx;

	power_supply_unregister(chip->power_supply);
	mutex_destroy(&chip->mba_lock);
	devm_kfree(chip->dev, chip->pdata);

	for (prop_idx = 0; prop_idx < FBFG_PROP_NR; prop_idx++)
		device_remove_file(chip->dev, &chip->prop_das[prop_idx].da);

	for (prop_idx = 0; prop_idx < FBFG_SBS_PROP_NR; prop_idx++)
		device_remove_file(chip->dev, &chip->sbs_prop_das[prop_idx].da);

	device_init_wakeup(chip->dev, false);

	return 0;
}

static void fbfg_shutdown(struct i2c_client *client)
{
	pr_info("Fbfg driver shutdown!\n");
}

static const struct dev_pm_ops fbfg_pm_ops = {
	.resume = fbfg_resume,
	.suspend = fbfg_suspend,
};

static struct i2c_driver fbfg_driver = {
	.driver	= {
		.name   = "fbfg-battery",
		.owner  = THIS_MODULE,
		.of_match_table = fbfg_match_table,
		.pm     = &fbfg_pm_ops,
	},
	.id_table       = fbfg_id,

	.probe          = fbfg_probe,
	.remove		= fbfg_remove,
	.shutdown	= fbfg_shutdown,

};

module_i2c_driver(fbfg_driver);

MODULE_DESCRIPTION("Fb fg Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Facebook");
