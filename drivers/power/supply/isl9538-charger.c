/*
 * Battery charger driver for Renesas ISL9538
 *
 * Copyright (C) 2018-2020 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#define CREATE_TRACE_POINTS
#include <trace/events/isl9538.h>

#include <linux/power/isl9538-charger.h>
#include <soc/qcom/restart.h>


#include <linux/delay.h>

#define DEFAULT_POLL_INTERVAL_NORMAL 5
#define DEFAULT_POLL_INTERVAL_MAINTENANCE 10
#define DEFAULT_POLL_INTERVAL_LOWBAT 1
#define DEFAULT_LOWBAT_CAPACITY 1

#define DEFAULT_HI_CHG_CLEAR_THR 95

#define DEFAULT_UPA_MONITOR_DELAY_MS 3000

/* 500 ms from alarm timer until driver resumes */
#define ISL9538_RESUME_TIMEOUT_MS 500

#define MAX_SYSTEM_VOLT_UPLIMIT 18304000
#define MAX_SYSTEM_CURR 6080000

/* Registers */
#define ISL9538_REG_CHG_CURRENT 0x14
#define ISL9538_REG_SYS_VOLTAGE_MAX 0x15
#define ISL9538_REG_T1_T2 0x38
#define ISL9538_REG_CONTROL0 0x39
#define ISL9538_REG_INFO 0x3a
#define ISL9538_REG_ADAPTER_CURRENT2 0x3b
#define ISL9538_REG_CONTROL1 0x3c
#define ISL9538_REG_CONTROL2 0x3d
#define ISL9538_REG_SYS_VOLTAGE_MIN 0x3e
#define ISL9538_REG_ADAPTER_CURRENT1 0x3f
#define ISL9538_REG_PROCHOT_AC 0x47
#define ISL9538_REG_PROCHOT_DC 0x48
#define ISL9538_REG_INPUT_VOLTAGE 0x4b
#define ISL9538_REG_CONTROL3 0x4c
#define ISL9538_REG_INFO2 0x4d
#define ISL9538_REG_CONTROL4 0x4e
#define ISL9538_REG_MANUFACTURER_ID 0xfe
#define ISL9538_REG_DEVICE_ID 0xff

/* Information2 Register */
#define ISL9538_STATE_MACHINE_MASK 0xf00
#define ISL9538_INFO2_CHARGE 5
#define ISL9538_INFO2_ACHRG 0xb
/* 12: BATGONE pin status (0 = Battery is present, 1 = No battery) */
#define ISL9538_INFORMATION2_BATGONE_PIN (1 << 12)
/* 14: ACOK pin status (0 = No adapter, 1 = Adapter is present) */
#define ISL9538_INFORMATION2_ACOK_PIN (1 << 14)

/* ChargeCurrentLimit Register */
#define ISL9538_CHARGE_CURRENT_MASK 0x1FFC
#define ISL9538_CHARGE_CURRENT_SHIFT 2
#define ISL9538_CHARGE_CURRENT_BASE 0
#define ISL9538_CHARGE_CURRENT_LSB 4

/* MaxSystemVoltage Register */
#define ISL9538_MAX_SYS_VOLT_MASK 0x7FF8
#define ISL9538_MAX_SYS_VOLT_SHIFT 3
#define ISL9538_MAX_SYS_VOLT_BASE 0
#define ISL9538_MAX_SYS_VOLT_LSB 8

/* AdapterCurrentLimit 1&2 Register */
#define ISL9538_ADAPTER_CURRENT_MASK 0x1FFC
#define ISL9538_ADAPTER_CURRENT_SHIFT 2
#define ISL9538_ADAPTER_CURRENT_BASE 0
#define ISL9538_ADAPTER_CURRENT_LSB 4

/* MinSystemVoltage Register */
#define ISL9538_MIN_SYS_VOLT_MASK 0x3F00
#define ISL9538_MIN_SYS_VOLT_SHIFT 8
#define ISL9538_MIN_SYS_VOLT_BASE 0
#define ISL9538_MIN_SYS_VOLT_LSB 256

/* Control0 Register */
#define ISL9538_SMBUS_TIMEOUT (1 << 7)

/* Control1: AMON & BMON */
#define ISL9538_C1_DISABLE_MON (1 << 5)
#define ISL9538_C1_SELECT_BMON (1 << 4)
#define ISL9538_C1_VSYS (1 << 2)

/* Control2: Trickle Charging Current */
#define ISL9538_C2_TRICKLE_CHARGE_CURR_SHIFT 14
#define ISL9538_C2_TRICKLE_CHARGE_CURR_MASK 0xC000

/* Default values */
#define DEFAULT_MANUFACTURER_ID 0x0049
#define DEFAULT_DEVICE_ID 0x000C

#define MAX_TRICKLE_CHARGE_COUNT	10

/*
 * Control3: AMON/BMON direction.
 * 0: adapter/charging, 1:discharging
 */
#define ISL9538_C3_AMON_BMON_DIRECTION (1 << 3)
#define ISL9538_C3_AUTO_CHARGING_MODE (1 << 7)

#define SWITCH_STATE_UVENT "SWITCH_STATE=%d"
#define INVALID_CHARGER_DEVNAME "invalid_charger"

#define ADC_RETRY_CNT 5

static bool isl9538_charger_is_present(struct isl9538 *charger);
static int isl9538_config_charger(struct isl9538 *charger);
static int isl9538_read_adapter_vadc_uv(struct isl9538 *charger);
static int isl9538_estimate_adapter_voltage(struct isl9538 *charger, int vadc_uv);
static int isl9538_charger_is_charging(struct isl9538 *charger);
static int isl9538_config_current_limits(struct isl9538 *charger,
	bool zero_limits);
static int isl9538_charger_enable_vsys(struct isl9538 *charger, bool enable);
static bool isl9538_charger_is_vsys_enabled(struct isl9538 *charger);

static const char default_isl9538_supplied_to[] = "bq40z50_battery";


static inline struct isl9538 *to_isl9538(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

static enum power_supply_property isl9538_charger_properties[] = {
	/* Keep in alphabetical order */
	POWER_SUPPLY_PROP_ADAPTER_CURRENT_LIMIT_1,
	POWER_SUPPLY_PROP_ADAPTER_CURRENT_LIMIT_2,
	POWER_SUPPLY_PROP_ADAPTER_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONTROL_3,
	POWER_SUPPLY_PROP_DISABLE_CHARGING,
	POWER_SUPPLY_PROP_MAX_SYSTEM_VOLTAGE,
	POWER_SUPPLY_PROP_MIN_ADAPTER_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_MIN_SYSTEM_VOLTAGE,
	POWER_SUPPLY_PROP_MONITOR_GAUGE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VSYS_STATUS,
};

static int isl9538_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	/* Keep in alphabetical order */
	case POWER_SUPPLY_PROP_CHARGE_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_DISABLE_CHARGING:
	case POWER_SUPPLY_PROP_MAX_SYSTEM_VOLTAGE:
	case POWER_SUPPLY_PROP_MIN_ADAPTER_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_MONITOR_GAUGE:
	case POWER_SUPPLY_PROP_VSYS_STATUS:
		return 1;
	default:
		break;
	}

	return 0;
}

static inline int __isl9538_write_word(struct i2c_client *client, u8 reg,
				     u16 value)
{
	return i2c_smbus_write_word_data(client, reg, le16_to_cpu(value));
}

static inline int __isl9538_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret = i2c_smbus_read_word_data(client, reg);

	return ret < 0 ? ret : le16_to_cpu(ret);
}

static int isl9538_update_word(struct i2c_client *client, u8 reg, u16 mask,
			       u16 value)
{
	unsigned int tmp;
	int ret;

	ret = __isl9538_read_word(client, reg);
	if (ret < 0)
		return ret;

	tmp = ret & ~mask;
	tmp |= value & mask;

	return __isl9538_write_word(client, reg, tmp);
}

static void isl9538_charger_dump_state(struct isl9538 *charger)
{
	struct isl9538_platform *pdata = charger->pdata;

	dev_dbg(&charger->client->dev, "Disable charge user(%d) battful(%d) adt(%d) batthi(%d)",
		charger->user_disable_charging,
		charger->battfull_disable_charging,
		charger->adapter_disable_charging,
		charger->batthi_disable_charging);
	dev_dbg(&charger->client->dev, "monitor_gauge (%d)",
		charger->monitor_gauge);
	dev_dbg(&charger->client->dev, "Charger curr(uA):%d volt(uV):%d",
		pdata->charge_current, pdata->charge_voltage);
	dev_dbg(&charger->client->dev, "Adapter voltage(uV)):%d ",
		pdata->adapter_uv);
	dev_dbg(&charger->client->dev, "User charge current limit(uA):%d ",
		pdata->user_current_limit);
}

static int isl9538_disable_watchdog_timer(struct isl9538 *charger)
{
	int ret = 0;

	ret = isl9538_update_word(charger->client, ISL9538_REG_CONTROL0,
			ISL9538_SMBUS_TIMEOUT, ISL9538_SMBUS_TIMEOUT);

	return ret;
}


static int isl9538_set_charge_voltage(struct isl9538 *charger, int volt)
{
	struct isl9538_platform *pdata = charger->pdata;
	int ret;
	u16 reg_val;
	int volt_mv;

	/*
	 * Don't set system voltage max when input volt is 0
	 * to keep using wall power while charger is connected
	 */
	if (pdata->prev_charge_voltage == volt || !volt)
		return 0;

	volt_mv = volt / 1000;
	if (volt_mv < ISL9538_MAX_SYS_VOLT_BASE)
		volt_mv = ISL9538_MAX_SYS_VOLT_BASE;

	volt_mv -= ISL9538_MAX_SYS_VOLT_BASE;
	reg_val = volt_mv / ISL9538_MAX_SYS_VOLT_LSB;
	reg_val <<= ISL9538_MAX_SYS_VOLT_SHIFT;

	ret = isl9538_update_word(charger->client, ISL9538_REG_SYS_VOLTAGE_MAX,
				  ISL9538_MAX_SYS_VOLT_MASK, reg_val);
	if (ret < 0) {
		dev_err(&charger->client->dev, "Failed set charge voltage.");
		return ret;
	}

	pdata->prev_charge_voltage = volt;
	trace_set_charge_voltage(volt);
	return ret;
}

static int isl9538_set_charge_current(struct isl9538 *charger, int curr)
{
	int ret;
	u16 reg_val;
	int curr_ma = curr / 1000;

	if (curr_ma < ISL9538_CHARGE_CURRENT_BASE)
		curr_ma = ISL9538_CHARGE_CURRENT_BASE;

	curr_ma -= ISL9538_CHARGE_CURRENT_BASE;
	reg_val = curr_ma / ISL9538_CHARGE_CURRENT_LSB;
	reg_val <<= ISL9538_CHARGE_CURRENT_SHIFT;

	ret = isl9538_update_word(charger->client, ISL9538_REG_CHG_CURRENT,
				  ISL9538_CHARGE_CURRENT_MASK, reg_val);
	trace_set_charge_current(curr_ma);

	return ret;
}

static int isl9538_set_adapter_current_limit_1(struct isl9538 *charger,
					       int curr)
{
	int ret;
	u16 reg_val;
	int curr_ma = curr / 1000;

	if (curr_ma < ISL9538_ADAPTER_CURRENT_BASE)
		curr_ma = ISL9538_ADAPTER_CURRENT_BASE;

	curr_ma -= ISL9538_ADAPTER_CURRENT_BASE;
	reg_val = curr_ma / ISL9538_ADAPTER_CURRENT_LSB;
	reg_val <<= ISL9538_ADAPTER_CURRENT_SHIFT;

	ret = isl9538_update_word(charger->client, ISL9538_REG_ADAPTER_CURRENT1,
				  ISL9538_ADAPTER_CURRENT_MASK, reg_val);

	return ret;
}

static int isl9538_set_adapter_current_limit_2(struct isl9538 *charger,
					       int curr)
{
	int ret;
	u16 reg_val;
	int curr_ma = curr / 1000;

	if (curr_ma < ISL9538_ADAPTER_CURRENT_BASE)
		curr_ma = ISL9538_ADAPTER_CURRENT_BASE;

	curr_ma -= ISL9538_ADAPTER_CURRENT_BASE;
	reg_val = curr_ma / ISL9538_ADAPTER_CURRENT_LSB;
	reg_val <<= ISL9538_ADAPTER_CURRENT_SHIFT;

	ret = isl9538_update_word(charger->client, ISL9538_REG_ADAPTER_CURRENT2,
				  ISL9538_ADAPTER_CURRENT_MASK, reg_val);

	return ret;
}

static inline int isl9538_enable_charging(struct isl9538 *charger)
{
	int curr, ret = 0;
	struct isl9538_platform *pdata = charger->pdata;

	if (pdata->user_current_limit &&
		(pdata->user_current_limit < pdata->charge_current))
		curr = pdata->user_current_limit;
	else
		curr = pdata->charge_current;

	if (curr != pdata->prev_charge_current)
		ret = isl9538_set_charge_current(charger, curr);
	pdata->prev_charge_current = curr;
	if (!charger->charging_en)
		trace_enable_charging(true);
	charger->charging_en = true;

	return ret;
}

static inline int isl9538_disable_charging(struct isl9538 *charger)
{
	int ret = 0;

	if (!charger->charging_en)
		return ret;
	ret = isl9538_set_charge_current(charger, 0);
	trace_enable_charging(false);
	charger->pdata->prev_charge_current = 0;
	charger->charging_en = false;

	return ret;
}

static int isl9538_config_charger(struct isl9538 *charger)
{
	struct isl9538_platform *pdata = charger->pdata;
	int ret = 0;

	if (charger->adapter_disable_charging ||
		!isl9538_charger_is_present(charger))
		ret = isl9538_config_current_limits(charger, true);
	else
		ret = isl9538_config_current_limits(charger, false);

	if (ret < 0)
		dev_err(&charger->client->dev,
			"Failed to config current limits: %d\n", ret);

	mutex_lock(&charger->voltage_lock);
	ret = isl9538_set_charge_voltage(charger,
				 pdata->charge_voltage);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to write charger voltage : %d\n", ret);
		mutex_unlock(&charger->voltage_lock);
		return ret;
	}
	mutex_unlock(&charger->voltage_lock);

	mutex_lock(&charger->current_lock);
	if (!isl9538_charger_is_present(charger) ||
		charger->user_disable_charging ||
		charger->battfull_disable_charging ||
		charger->adapter_disable_charging ||
		charger->batthi_disable_charging) {
		ret = isl9538_disable_charging(charger);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"Disabling charging failed: %d\n", ret);
	} else {
		ret = isl9538_enable_charging(charger);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"Enabling charging failed: %d\n", ret);

	}
	mutex_unlock(&charger->current_lock);

	return ret;
}

static int isl9538_config_current_limits(struct isl9538 *charger,
	bool zero_limits)
{
	struct isl9538_platform *pdata = charger->pdata;
	int ret = 0;
	uint32_t adapter_curr_limit_1 = zero_limits ?
			0 : pdata->adapter_curr_limit_1;
	uint32_t adapter_curr_limit_2 = zero_limits ?
			0 : pdata->adapter_curr_limit_2;

	ret = isl9538_set_adapter_current_limit_1(
		charger, adapter_curr_limit_1);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to write charger adapter current limit 1 : %d\n",
			ret);
		return ret;
	}

	ret = isl9538_set_adapter_current_limit_2(
		charger, adapter_curr_limit_2);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to write charger adapter current limit 2 : %d\n",
			ret);
		return ret;
	}
	return ret;
}

static int isl9538_config_trickle_charge(struct isl9538 *charger)
{
	struct isl9538_platform *pdata = charger->pdata;
	u16 reg_val;
	int ret;

	reg_val = pdata->trickle_charge_curr;
	reg_val <<= ISL9538_C2_TRICKLE_CHARGE_CURR_SHIFT;

	ret = isl9538_update_word(charger->client, ISL9538_REG_CONTROL2,
			ISL9538_C2_TRICKLE_CHARGE_CURR_MASK, reg_val);

	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to write trickle charge current : %d\n",
			ret);
		return ret;
	}

	return ret;
}

static bool isl9538_charger_is_present(struct isl9538 *charger)
{
	int ac = 0;

	ac = __isl9538_read_word(charger->client, ISL9538_REG_INFO2);
	if (ac < 0) {
		dev_err(&charger->client->dev,
			"Failed to read charger status : %d\n", ac);
		return false;
	}

	return (ac & ISL9538_INFORMATION2_ACOK_PIN) ? true : false;
}

static int isl9538_charger_is_charging(struct isl9538 *charger)
{
	int ret;
	int ch_status;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_INFO2);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read charger status : %d\n", ret);
		return ret;
	}

	ch_status = (ret & ISL9538_STATE_MACHINE_MASK) >> 8;
	if (ch_status == ISL9538_INFO2_ACHRG
		|| ch_status == ISL9538_INFO2_CHARGE)
		return 1;
	else
		return 0;
}

/**
 * isl9538_read_charge_current_limit - Returns charging current limit in uA
 */
static int isl9538_read_charge_current_limit(struct isl9538 *charger)
{
	int ret;
	int ch_curr_limit_ma;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_CHG_CURRENT);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read charger current : %d\n", ret);
		return ret;
	}

	ch_curr_limit_ma = (ret & ISL9538_CHARGE_CURRENT_MASK) >>
			ISL9538_CHARGE_CURRENT_SHIFT;

	ch_curr_limit_ma *= ISL9538_CHARGE_CURRENT_LSB;

	return ch_curr_limit_ma * 1000;
}

/**
 * isl9538_read_max_system_voltage- Returns the maximum system voltage in uV
 */
static int isl9538_read_max_system_voltage(struct isl9538 *charger)
{
	int ret;
	int ch_max_sys_volt_mv;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_SYS_VOLTAGE_MAX);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read max voltage : %d\n", ret);
		return ret;
	}

	ch_max_sys_volt_mv =
		(ret & ISL9538_MAX_SYS_VOLT_MASK) >> ISL9538_MAX_SYS_VOLT_SHIFT;

	ch_max_sys_volt_mv *= ISL9538_MAX_SYS_VOLT_LSB;

	return ch_max_sys_volt_mv * 1000;
}

/**
 * isl9538_read_min_system_voltage - Returns the minimum system voltage in uV
 */
static int isl9538_read_min_system_voltage(struct isl9538 *charger)
{
	int ret;
	int ch_min_sys_volt_mv;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_SYS_VOLTAGE_MIN);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read min voltage : %d\n", ret);
		return ret;
	}

	ch_min_sys_volt_mv =
		(ret & ISL9538_MIN_SYS_VOLT_MASK) >> ISL9538_MIN_SYS_VOLT_SHIFT;

	ch_min_sys_volt_mv *= ISL9538_MIN_SYS_VOLT_LSB;

	return ch_min_sys_volt_mv * 1000;
}

/**
 * isl9538_read_adapter_current_limit_1- Returns adapter current limit 1 in uA
 */
static int isl9538_read_adapter_current_limit_1(struct isl9538 *charger)
{
	int ret;
	int ch_curr_limit_ma;

	ret = __isl9538_read_word(charger->client,
			ISL9538_REG_ADAPTER_CURRENT1);

	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read current limit1 : %d\n", ret);
		return ret;
	}

	ch_curr_limit_ma = (ret & ISL9538_ADAPTER_CURRENT_MASK) >>
			ISL9538_ADAPTER_CURRENT_SHIFT;

	ch_curr_limit_ma *= ISL9538_ADAPTER_CURRENT_LSB;

	return ch_curr_limit_ma * 1000;
}

/* Returns adapter current limit 2 in the unit of uA */
static int isl9538_read_adapter_current_limit_2(struct isl9538 *charger)
{
	int ret;
	int ch_curr_limit_ma;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client,
			ISL9538_REG_ADAPTER_CURRENT2);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read current limit2 : %d\n", ret);
		return ret;
	}

	ch_curr_limit_ma = (ret & ISL9538_ADAPTER_CURRENT_MASK) >>
			ISL9538_ADAPTER_CURRENT_SHIFT;

	ch_curr_limit_ma *= ISL9538_ADAPTER_CURRENT_LSB;

	return ch_curr_limit_ma * 1000;
}

static int isl9538_read_control_3(struct isl9538 *charger)
{
	int ret;
	int ch_control;

	if (!isl9538_charger_is_present(charger))
		return 0;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_CONTROL3);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read reg control3 : %d\n", ret);
		return ret;
	}

	ch_control = ret;

	return ch_control;
}

static int isl9538_enable_trickle_charge(struct isl9538 *charger)
{
	struct isl9538_platform *pdata = charger->pdata;
	int tricke_current;
	int ret;

	switch (pdata->trickle_charge_curr) {
	case ISL_TRICKLE_CHARGE_CURR_128MA:
		tricke_current = 128000;
		break;
	case ISL_TRICKLE_CHARGE_CURR_64MA:
		tricke_current = 64000;
		break;
	case ISL_TRICKLE_CHARGE_CURR_512MA:
		tricke_current = 512000;
		break;
	default:
		tricke_current = 256000;
	}

	ret = isl9538_config_trickle_charge(charger);
	if (ret < 0) {
		dev_err(&charger->client->dev, "Failed to set trickle charge\n");
		return ret;
	}

	mutex_lock(&charger->current_lock);
	pdata->charge_current = tricke_current;
	ret = isl9538_enable_charging(charger);
	mutex_unlock(&charger->current_lock);

	if (ret < 0)
		dev_err(&charger->client->dev,
			"Enabling trickle charging failed: %d\n", ret);

	return ret;
}


static void isl9538_upa_monitor_workfunc(struct work_struct *work)
{
	struct isl9538 *charger =
		container_of(work, struct isl9538, upa_monitor_work.work);
	struct isl9538_platform *pdata = charger->pdata;
	char *switch_state_events[2];
	int vadc_uv;
	bool notif_upa = false;
	int i;
	int cumulated_uv = 0;
	int cumulated_uv_avrg = 0;
	int threshold_resolution_uv = 0;

	mutex_lock(&charger->current_lock);

	/*
	 * The adapter_disable_charging should be set to true when
	 * charger is disconnected. Otherwise, charging would start
	 * immediately when charger is connected, which might cause
	 * over loading on UPA (Under Powered Adapter)
	 * By setting adapter_disable_charging is true, UPA condistion
	 * should not use adapter_disable_charging, so added upa flag
	 * to represent only UPA condition.
	 */
	if (isl9538_charger_is_present(charger)) {
		/*
		 * Simple check to see if device is evt1 or proto
		 */
		if (charger->min_adapter_charge_uv) {
			for (i = 0; i < ADC_RETRY_CNT; i++) {
				vadc_uv = isl9538_read_adapter_vadc_uv(charger);
				pdata->adapter_uv =
				isl9538_estimate_adapter_voltage(charger, vadc_uv);
				cumulated_uv += pdata->adapter_uv;
				cumulated_uv_avrg = cumulated_uv / (i + 1);
				threshold_resolution_uv =
				(cumulated_uv_avrg * 5) / 100;

				if (((cumulated_uv_avrg + threshold_resolution_uv)
					<= pdata->adapter_uv) ||
					((cumulated_uv_avrg - threshold_resolution_uv)
					>= pdata->adapter_uv)) {
					notif_upa = true;
					break;
				}
				msleep(20);
			}

			if (cumulated_uv_avrg < charger->min_adapter_charge_uv)
				notif_upa = true;

			if (notif_upa)
				charger->adapter_disable_charging = true;
			else {
				charger->adapter_disable_charging = false;
			}
		} else {
			charger->adapter_disable_charging = false;
		}
	} else {
		charger->adapter_disable_charging = true;
	}

	switch_state_events[0] = kasprintf(GFP_KERNEL, SWITCH_STATE_UVENT,
			notif_upa ? 1 : 0);
	if (!switch_state_events[0]) {
		dev_err(&charger->client->dev,
			"Failed to allocate memory for switch state event.");
		/**
		 * Even if memory allocation failed,
		 * call monitor work to reconfigure charging parameters.
		 */
		goto out;
	}

	switch_state_events[1] = NULL;
	kobject_uevent_env(&charger->invalid_charger_dev.kobj,
			KOBJ_CHANGE, switch_state_events);
	kfree(switch_state_events[0]);

out:
	charger->wrong_charger_state = notif_upa;
	mutex_unlock(&charger->current_lock);
	trace_upa_monitor_work( vadc_uv,
		pdata->adapter_uv,
		charger->min_adapter_charge_uv,
		charger->adapter_disable_charging,
		notif_upa);

	if (charger->monitor_gauge)
		cancel_delayed_work_sync(&charger->monitor_work);
	schedule_delayed_work(&charger->monitor_work, 0);
}

static bool isl9538_need_trickle_charge(struct power_supply *fg_psy,
						struct isl9538 *charger) {
	union power_supply_propval data;
	int ret;

	if (!fg_psy) {
		dev_err(&charger->client->dev,
			"fg psy is null\n");
		return true;
	}

	ret = power_supply_get_property(fg_psy, POWER_SUPPLY_PROP_STATUS,
			&data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to get POWER_SUPPLY_PROP_STATUS\n");
		return false;
	}

	if (data.intval != POWER_SUPPLY_STATUS_UNKNOWN)
		return false;

	ret = power_supply_get_property(fg_psy, POWER_SUPPLY_PROP_HEALTH,
			&data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to get POWER_SUPPLY_PROP_HEALTH\n");
		return false;
	}

	if (data.intval != POWER_SUPPLY_HEALTH_UNKNOWN)
		return false;
	return true;
}

static void isl9538_monitor_workfunc(struct work_struct *work)
{
	struct isl9538 *charger =
		container_of(work, struct isl9538, monitor_work.work);
	struct isl9538_platform *pdata = charger->pdata;
	struct power_supply *psy;
	union power_supply_propval data;
	bool is_lowbatt = false;
	bool is_hibatt = false;
	int vadc_uv;
	int ret;

	/* Check if adapter voltage level.
	 * Ideally level should only be chekced when adapter dock and undock
	 * events happen. As a safety net, check them periodically.
	*/
	if (isl9538_charger_is_present(charger)) {
		/*
		 * Simple check to see if device is evt1 or proto
		 */
		if(charger->min_adapter_charge_uv) {
			vadc_uv = isl9538_estimate_adapter_voltage(charger,
					isl9538_read_adapter_vadc_uv(charger));

			if (vadc_uv < charger->min_adapter_charge_uv)
				charger->adapter_disable_charging = true;
			else
				charger->adapter_disable_charging = false;
		} else {
			charger->adapter_disable_charging = false;
		}
	}

	psy = power_supply_get_by_name(pdata->fg_psy_name);

	if (isl9538_need_trickle_charge(psy, charger)) {
		dev_err(&charger->client->dev,
			"Fg is in a unknown state or not available\n");

		/* Start trickle charging */
		if (!charger->trickle_charge_count) {
			if (isl9538_enable_trickle_charge(charger) != 0)
				goto retry;
		}

		/* Check if we need to disable trickle charging */
		if (charger->trickle_charge_count < MAX_TRICKLE_CHARGE_COUNT) {
			charger->trickle_charge_count++;
			goto retry;
		} else if (charger->trickle_charge_count ==
				MAX_TRICKLE_CHARGE_COUNT) {
			// charger->trickle_charge_count++ to avoid set
			// disable charging continuously
			charger->trickle_charge_count++;
			isl9538_disable_charging(charger);
			dev_err(&charger->client->dev,
				"Failed to trickle charge battery, giving up!\n");
			return;
		}

		/* Do nothing to waiting for psy recovery */
		return;
	} else {
		if (charger->trickle_charge_count) {
			dev_info(&charger->client->dev,
				"Trickle charged successfully at count:%d\n",
				charger->trickle_charge_count);
			charger->trickle_charge_count = 0;
		}
	}

	if (charger->suspended) {
		ret = wait_for_completion_timeout(
			&charger->resume_done,
			msecs_to_jiffies(ISL9538_RESUME_TIMEOUT_MS));
		if (!ret) {
			dev_err(&charger->client->dev,
				"resume didn't happen\n");
		}
	}

	/* Update fg cache data */
	data.intval = 1;
	ret = power_supply_set_property(psy,
			POWER_SUPPLY_PROP_UPDATE_NOW, &data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to set POWER_SUPPLY_PROP_UPDATE_NOW\n");
		goto exit;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to get POWER_SUPPLY_PROP_STATUS\n");
		goto exit;
	}

	/* Disable charging if battery is full */
	if (data.intval == POWER_SUPPLY_STATUS_FULL) {
		mutex_lock(&charger->current_lock);
		charger->battfull_disable_charging = true;
		mutex_unlock(&charger->current_lock);
	} else {
		mutex_lock(&charger->current_lock);
		charger->battfull_disable_charging = false;
		mutex_unlock(&charger->current_lock);
	}

	// Get charge current limit
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGING_CURRENT,
					&data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to get POWER_SUPPLY_PROP_CHARGING_CURRENT\n");
		goto exit;
	}

	if (data.intval > MAX_SYSTEM_CURR)
		data.intval = MAX_SYSTEM_CURR;

	mutex_lock(&charger->current_lock);
	pdata->charge_current = data.intval;
	mutex_unlock(&charger->current_lock);

	// Get max system voltage
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGING_VOLTAGE,
					&data);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to get POWER_SUPPLY_PROP_CHARGING_VOLTAGE\n");
		goto exit;
	}

	if (data.intval > MAX_SYSTEM_VOLT_UPLIMIT)
		data.intval = MAX_SYSTEM_VOLT_UPLIMIT;

	mutex_lock(&charger->voltage_lock);
	pdata->charge_voltage = data.intval;
	mutex_unlock(&charger->voltage_lock);

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY,
					&data);
	if (!ret && data.intval <= pdata->lowbatt_cap)
		is_lowbatt = true;

	/* W/A for T105332033.
	 * Disable charging when capacity reaches hi_chg_lmt.
	 * Start polling at interval_hibatt_s when SOC level reaches to
	 * hi_poll_thr.
	 */
	if (pdata->hi_chg_lmt) {
		int real_soc;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_REAL_CAPACITY,
					&data);
		if (!ret) {
			real_soc = data.intval;
			if (real_soc >= pdata->hi_chg_lmt) {
				mutex_lock(&charger->current_lock);
				charger->batthi_disable_charging = true;
				mutex_unlock(&charger->current_lock);
			}

			if (charger->batthi_disable_charging && real_soc <= pdata->hi_clr_thr) {
				mutex_lock(&charger->current_lock);
				charger->batthi_disable_charging = false;
				mutex_unlock(&charger->current_lock);
			}

			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS,
					&data);
			if (!ret)
				if (data.intval == POWER_SUPPLY_STATUS_CHARGING)
					if (real_soc >= pdata->hi_poll_thr && real_soc < pdata->hi_chg_lmt)
						is_hibatt = true;
		}

		if (charger->battfull_disable_charging) {
			ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_HEALTH, &data);
			if (ret)
				dev_err(&charger->client->dev,
					"Failed to get POWER_SUPPLY_PROP_HEALTH\n");
			else if (data.intval != POWER_SUPPLY_HEALTH_DEAD &&
				data.intval != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE &&
				data.intval != POWER_SUPPLY_HEALTH_UNKNOWN &&
				isl9538_charger_is_vsys_enabled(charger) &&
				real_soc != 0) {
				dev_info(&charger->client->dev,
					"Batt is not PF and full. Disable vsys now");
				isl9538_charger_enable_vsys(charger, false);
			}
		} else
			isl9538_charger_enable_vsys(charger, true);
	}

	ret = isl9538_config_charger(charger);

	if (ret < 0)
		dev_err(&charger->client->dev, "Monitor work Failed to configure charger");

	isl9538_charger_dump_state(charger);

	power_supply_changed(charger->charger);

exit:
	power_supply_put(psy);

retry:
	if (charger->monitor_gauge) {
		u64 interval;
		if (charger->battfull_disable_charging &&
			isl9538_charger_is_present(charger)) {
			interval = pdata->interval_maintenance_s;
		} else {
			if (is_lowbatt)
				interval = pdata->interval_lowbatt_s;
			else if (is_hibatt)
				interval = pdata->interval_hibatt_s;
			else
				interval = pdata->interval_normal_s;
		}
		alarm_start_relative(&charger->monitor_alarm,
			ms_to_ktime(interval*MSEC_PER_SEC));
	}

	pm_relax(&charger->client->dev);
}

static enum alarmtimer_restart isl9538_monitor_alarmcb(struct alarm *alarm,
							ktime_t now)
{
	struct isl9538 *charger =
		container_of(alarm, struct isl9538, monitor_alarm);

	pm_stay_awake(&charger->client->dev);
	schedule_delayed_work(&charger->monitor_work, 0);

	return ALARMTIMER_NORESTART;
}

static bool isl9538_charger_is_vsys_enabled(struct isl9538 *charger)
{
	int ret;

	ret = __isl9538_read_word(charger->client, ISL9538_REG_CONTROL1);

	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to read charger status : %d\n", ret);
		/* Vsys is enabled by default return true */
		return true;
	}
	if (ret & ISL9538_C1_VSYS)
		return false;

	return true;
}

static int isl9538_charger_enable_vsys(struct isl9538 *charger, bool enable)
{
	u16 reg_val = 0;
	int ret;

	if (!enable)
		reg_val = ISL9538_C1_VSYS;

	ret = isl9538_update_word(charger->client, ISL9538_REG_CONTROL1,
			ISL9538_C1_VSYS, reg_val);

	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Failed to write vsys ctrl bit : %d\n",
			ret);
		return ret;
	}
	if (charger->vsys_sts != enable) {
		charger->vsys_sts = enable;
		trace_enable_vsys(enable);
		dev_info(&charger->client->dev, "%s: vsys Status(%d)",
			__func__, charger->vsys_sts);
	}
	return ret;

}


static int isl9538_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct isl9538 *charger = to_isl9538(psy);

	/* Always return current and voltage information in unit of uA and mV */
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CURRENT_LIMIT:
		val->intval = isl9538_read_charge_current_limit(charger);
		break;
	case POWER_SUPPLY_PROP_ADAPTER_CURRENT_LIMIT_1:
		val->intval = isl9538_read_adapter_current_limit_1(charger);
		break;
	case POWER_SUPPLY_PROP_ADAPTER_CURRENT_LIMIT_2:
		val->intval = isl9538_read_adapter_current_limit_2(charger);
		break;
	case POWER_SUPPLY_PROP_CONTROL_3:
		val->intval = isl9538_read_control_3(charger);
		break;
	case POWER_SUPPLY_PROP_MONITOR_GAUGE:
		val->intval = charger->monitor_gauge;
		break;
	case POWER_SUPPLY_PROP_MAX_SYSTEM_VOLTAGE:
		val->intval = isl9538_read_max_system_voltage(charger);
		break;
	case POWER_SUPPLY_PROP_MIN_SYSTEM_VOLTAGE:
		val->intval = isl9538_read_min_system_voltage(charger);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = isl9538_charger_is_present(charger) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		switch (isl9538_charger_is_charging(charger)) {
		case 1:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 0:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_DISABLE_CHARGING:
		val->intval = charger->user_disable_charging ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_VSYS_STATUS:
		val->intval = isl9538_charger_is_vsys_enabled(charger) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_ADAPTER_VOLTAGE:
		if(charger->min_adapter_charge_uv)
			val->intval = isl9538_estimate_adapter_voltage(charger,
							isl9538_read_adapter_vadc_uv(charger));
		else
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_MIN_ADAPTER_CHARGE_VOLTAGE:
		val->intval = charger->min_adapter_charge_uv;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int isl9538_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct isl9538 *charger = to_isl9538(psy);
	struct isl9538_platform *pdata = charger->pdata;
	int setval;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_DISABLE_CHARGING:
		if (val->intval) {
			mutex_lock(&charger->current_lock);
			charger->user_disable_charging = true;
			mutex_unlock(&charger->current_lock);
			break;
		} else {
			mutex_lock(&charger->current_lock);
			charger->user_disable_charging = false;
			mutex_unlock(&charger->current_lock);
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT_LIMIT:
		setval = val->intval;
		if (setval > MAX_SYSTEM_CURR)
			setval = MAX_SYSTEM_CURR;
		else if (setval < 0)
			setval = 0;

		mutex_lock(&charger->current_lock);
		pdata->user_current_limit = setval;
		mutex_unlock(&charger->current_lock);
		break;
	case POWER_SUPPLY_PROP_MAX_SYSTEM_VOLTAGE:
		if (setval > MAX_SYSTEM_VOLT_UPLIMIT)
			setval = MAX_SYSTEM_VOLT_UPLIMIT;

		mutex_lock(&charger->voltage_lock);
		pdata->charge_voltage = setval;
		mutex_unlock(&charger->voltage_lock);
		break;
	case POWER_SUPPLY_PROP_MONITOR_GAUGE:
		if (val->intval) {
			charger->monitor_gauge = true;
			schedule_delayed_work(&charger->monitor_work, 0);
		} else {
			charger->monitor_gauge = false;
			alarm_cancel(&charger->monitor_alarm);
			cancel_delayed_work_sync(&charger->monitor_work);
		}
		break;
	case POWER_SUPPLY_PROP_VSYS_STATUS:
		if (val->intval)
			ret = isl9538_charger_enable_vsys(charger, true);
		else
			ret = isl9538_charger_enable_vsys(charger, false);
		if (ret < 0)
			dev_err(&charger->client->dev, "Failed to set vsys\n");
		break;
	case POWER_SUPPLY_PROP_MIN_ADAPTER_CHARGE_VOLTAGE:
		if (val->intval) {
			mutex_lock(&charger->current_lock);
			charger->min_adapter_charge_uv = val->intval;
			mutex_unlock(&charger->current_lock);
		}
		break;
	default:
		return -EPERM;
	}

	ret = isl9538_config_charger(charger);
	if (ret < 0) {
		dev_err(&charger->client->dev, "Failed to configure charger");
		return ret;
	}
	isl9538_charger_dump_state(charger);
	power_supply_changed(psy);
	return 0;
}

static struct isl9538_platform *isl9538_parse_dt_data(struct i2c_client *client)
{
	struct isl9538_platform *pdata;
	struct device_node *np = client->dev.of_node;
	u32 val;
	int ret;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "isl,adapter-current-limit-2", &val);
	if (!ret)
		pdata->adapter_curr_limit_2 = val * 1000;

	ret = of_property_read_u32(np, "isl,adapter-current-limit-1", &val);
	if (!ret)
		pdata->adapter_curr_limit_1 = val * 1000;

	ret = of_property_read_u32(np, "isl,trickle-charging-current", &val);
	if (!ret)
		pdata->trickle_charge_curr = val;
	else
		// Set trickle charge current value as hw default value, 256mA.
		pdata->trickle_charge_curr = ISL_TRICKLE_CHARGE_CURR_256MA;

	ret = of_property_read_u32(np, "isl,interval-normal",
		&pdata->interval_normal_s);
	if (ret)
		pdata->interval_normal_s = DEFAULT_POLL_INTERVAL_NORMAL;

	ret = of_property_read_u32(np, "isl,interval-maintenance",
		&pdata->interval_maintenance_s);
	if (ret)
		pdata->interval_maintenance_s = DEFAULT_POLL_INTERVAL_MAINTENANCE;

	ret = of_property_read_u32(np, "isl,interval-lowbatt",
		&pdata->interval_lowbatt_s);
	if (ret)
		pdata->interval_lowbatt_s = DEFAULT_POLL_INTERVAL_LOWBAT;

	ret = of_property_read_u32(np, "isl,lowbat-capacity",
		&pdata->lowbatt_cap);
	if (ret)
		pdata->lowbatt_cap = DEFAULT_LOWBAT_CAPACITY;

	ret = of_property_read_u32(client->dev.of_node,
			"isl,hi-chg-lmt", &pdata->hi_chg_lmt);
	if (ret) {
		pdata->hi_chg_lmt = 0;
		pdata->hi_poll_thr = 0;
		pdata->interval_hibatt_s = DEFAULT_POLL_INTERVAL_NORMAL;
	} else {
		ret = of_property_read_u32(np, "isl,hi-poll-thr",
			&pdata->hi_poll_thr);
		if (ret || pdata->hi_poll_thr > pdata->hi_chg_lmt)
			pdata->hi_poll_thr = pdata->hi_chg_lmt - 2;

		ret = of_property_read_u32(np, "isl,hi-poll-s",
			&pdata->interval_hibatt_s);
		if (ret)
			pdata->interval_hibatt_s = DEFAULT_POLL_INTERVAL_NORMAL;

		ret = of_property_read_u32(np, "isl,hi-clr-thr",
			&pdata->hi_clr_thr);
		if (ret)
			pdata->hi_clr_thr = DEFAULT_HI_CHG_CLEAR_THR;
	}

	dev_info(&client->dev,
		"%s: hi_chg_lmt %d, hi_poll_thr %d, hi_clr_thr %d, interval_hibatt_s %ds\n",
		__func__, pdata->hi_chg_lmt,
		pdata->hi_poll_thr,
		pdata->hi_clr_thr,
		pdata->interval_hibatt_s);

	ret = of_property_read_string(np, "isl,fg_psy_name",
		&pdata->fg_psy_name);
	if (ret)
		pdata->fg_psy_name = default_isl9538_supplied_to;

	dev_info(&client->dev,
		"%s: interval norm %ds, main %ds, low %ds, low cap %d\n",
		__func__, pdata->interval_normal_s,
		pdata->interval_maintenance_s,
		pdata->interval_lowbatt_s,
		pdata->lowbatt_cap);

	dev_info(&client->dev, "battery psy name %s",
		pdata->fg_psy_name);

	return pdata;
}

static void isl9538_get_min_adpter_volt_charge(struct isl9538 *charger)
{
	struct device_node *np = charger->client->dev.of_node;
	u32 val;
	int ret;

	ret = of_property_read_u32(np, "isl,min-charge-adapter-voltage", &val);
	if (!ret)
		charger->min_adapter_charge_uv = val * 1000;
	else {
		dev_err(&charger->client->dev, "isl,min-charge-adapter-voltage entry not found.");
		charger->min_adapter_charge_uv = 0;
	}
}

static int isl9538_read_adapter_vadc_uv(struct isl9538 *charger)
{
	struct qpnp_vadc_result result;
	int ret;

	if (!charger->vadc_dev)
		return -EINVAL;

	ret = qpnp_vadc_read(charger->vadc_dev, charger->channel_num, &result);
	if (ret < 0)
		pr_err("failure to read vadc channel=18\n");

	return (int) result.physical;
}

static struct isl9538_platform *isl9538_get_pdata(struct i2c_client *client)
{
	struct isl9538_platform *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	// Default setting for 4S battery with Terry PSU
	pdata->adapter_curr_limit_2 = 3800000;
	pdata->adapter_curr_limit_1 = 2368000;

	// Set trickle charge current value as hw default value, 256mA.
	pdata->trickle_charge_curr = ISL_TRICKLE_CHARGE_CURR_256MA;
	return pdata;
}

static int isl9538_estimate_adapter_voltage(struct isl9538 *charger, int vadc_uv)
{
	int level;

	if (!charger->vadc_dev)
		return 0;

	if (vadc_uv < 0) {
		pr_err("Invalid vadc value.\n");
		return 0;
	}

	for (level = 0; level < charger->num_adapter_uv; level++)
		if (vadc_uv >= charger->adapter_vadc_uvs[level])
			break;

	if (level == charger->num_adapter_uv) {
		pr_err("Failed to convert vadc to adapter voltage.(vadc:%d)\n",
			vadc_uv);
		return 0;
	}

	return charger->adapter_available_uvs[level];
}

/* irq handler */
static irqreturn_t isl9538_irq(int irq, void *data)
{
	struct isl9538 *charger = data;
	struct i2c_client *client = charger->client;
	int ret;
	int vadc_uv;

	if (charger->suspended) {
		ret = wait_for_completion_timeout(
			&charger->resume_done,
			msecs_to_jiffies(ISL9538_RESUME_TIMEOUT_MS));
		if (!ret) {
			dev_err(&charger->client->dev,
				"resume didn't happen\n");
		}
	}

	if (charger->min_adapter_charge_uv)
		vadc_uv = isl9538_read_adapter_vadc_uv(charger);

	cancel_delayed_work_sync(&charger->upa_monitor_work);
	if (gpiod_get_value(charger->acok)) {
		ret = isl9538_charger_enable_vsys(charger, true);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to enable vsys\n");
			return ret;
		}

		/**
		 * Schedule monitor work to check adapter voltage.
		 * Based on adatper voltage, it enables or disables charging
		 */
		if (charger->monitor_gauge)
			cancel_delayed_work_sync(&charger->monitor_work);

		/**
		 * Adapter current limits on HW are restored to default value
		 * when chager is reconnected.
		 * Set it to 0 until we know proper values to program them.
		 */
		ret = isl9538_config_current_limits(charger, true);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to set current limits\n");
			return ret;
		}

		/*
		 * Charger voltage is reset when charger is connected
		 * if PSYS is not enabled. So reset the prev_charge_voltage
		 * flag here, so that the proper charging voltage can be set
		 */
		charger->pdata->prev_charge_voltage = 0;

		schedule_delayed_work(&charger->monitor_work, 0);

		schedule_delayed_work(&charger->upa_monitor_work,
			msecs_to_jiffies(DEFAULT_UPA_MONITOR_DELAY_MS));
		trace_is_charger_present(true,
				vadc_uv,
				isl9538_estimate_adapter_voltage(charger, vadc_uv));
	} else {
		schedule_delayed_work(&charger->upa_monitor_work, 0);
		trace_is_charger_present(false,	vadc_uv,
				isl9538_estimate_adapter_voltage(charger, vadc_uv));
		charger->pdata->prev_charge_voltage = 0;
		charger->pdata->prev_charge_current = 0;
	}
	power_supply_changed(charger->charger);
	return IRQ_HANDLED;
}

static int np_get_adapter_vadc_map(struct isl9538 *charger,
								char *prop_name)
{
	struct i2c_client *client = charger->client;
	struct device_node *np;
	int prop_len, i;
	u32 *array;

	if (!client)
		return -EINVAL;

	np = client->dev.of_node;
	if (!of_find_property(np, prop_name, &prop_len)) {
		dev_err(&client->dev, "missing %s\n", prop_name);
		return -EINVAL;
	}

	prop_len /= sizeof(u32);
	if (prop_len % 2) {
		dev_err(&client->dev, "bad length %d\n", prop_len);
		return -EINVAL;
	}
	prop_len /= 2;

	charger->adapter_available_uvs = devm_kzalloc(&client->dev,
			prop_len * sizeof(charger->adapter_available_uvs[0]),
			GFP_KERNEL);
	if (!charger->adapter_available_uvs)
		return -ENOMEM;

	charger->adapter_vadc_uvs = devm_kzalloc(&client->dev,
			prop_len * sizeof(charger->adapter_vadc_uvs[0]),
			GFP_KERNEL);
	if (!charger->adapter_vadc_uvs)
		return -ENOMEM;

	charger->num_adapter_uv = prop_len;

	array = devm_kzalloc(&client->dev,
			prop_len * sizeof(u32) * 2, GFP_KERNEL);

	if (!array)
		return -ENOMEM;

	of_property_read_u32_array(np, prop_name, array, prop_len * 2);
	for (i = 0; i < prop_len; i++) {
		charger->adapter_available_uvs[i] = array[2 * i] * 1000;
		charger->adapter_vadc_uvs[i] = array[2 * i + 1] * 1000;
		dev_info(&client->dev, "%s: %d - %d\n", __func__,
					array[2 * i], array[2 * i + 1]);
	}

	devm_kfree(&client->dev, array);
	return 0;
}

static ssize_t state_show(struct device *d,
				 struct device_attribute *attr,
				 char *buf)
{
	struct isl9538 *charger;

	if (!d)
		return -EINVAL;

	charger = (struct isl9538 *)(d->platform_data);

	if (!charger)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		charger->wrong_charger_state ? 1 : 0);
}

static DEVICE_ATTR(state, 0444, state_show, NULL);

static struct attribute *switch_attrs[] = {
	NULL,
};

ATTRIBUTE_GROUPS(switch);

static struct bus_type switch_subsys = {
	.name				= "switch",
	.dev_groups			= switch_groups,
};

static int isl9538_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret;
	int irq;
	struct isl9538 *charger;
	struct qpnp_vadc_chip *vadc_dev;
	struct power_supply_desc *supply_desc;
	struct power_supply_config psy_cfg = {};
	char *name;
	u32 val;

	vadc_dev = qpnp_get_vadc(&client->dev, "adapter");

	if (IS_ERR(vadc_dev)) {
		ret = PTR_ERR(vadc_dev);
		if (ret != -EPROBE_DEFER) {
			pr_err("vadc property missing, ret=%d\n", ret);
			vadc_dev = NULL;
		} else {
			return -EPROBE_DEFER;
		}
	}

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->current_lock);
	mutex_init(&charger->voltage_lock);

	charger->pdata = client->dev.platform_data;
	charger->user_disable_charging = false;
	charger->battfull_disable_charging = false;
	charger->wrong_charger_state = false;
	charger->adapter_disable_charging = false;
	charger->batthi_disable_charging = false;
	charger->vsys_sts = false;

	if (!charger->pdata) {
		if (IS_ENABLED(CONFIG_OF) && client->dev.of_node)
			charger->pdata = isl9538_parse_dt_data(client);
		else
			charger->pdata = isl9538_get_pdata(client);
	}

	if (!charger->pdata) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}
	charger->pdata->prev_charge_voltage = 0;
	charger->pdata->prev_charge_current = 0;

	name = (char *)charger->pdata->name;
	if (!name) {
		name = devm_kasprintf(&client->dev, GFP_KERNEL, "isl9538@%s",
				      dev_name(&client->dev));
		if (!name) {
			dev_err(&client->dev, "Failed to alloc device name\n");
			return -ENOMEM;
		}
	}
	charger->client = client;

	supply_desc = &charger->charger_desc;

	supply_desc->name = name;
	supply_desc->type = POWER_SUPPLY_TYPE_MAINS;
	supply_desc->properties = isl9538_charger_properties;
	supply_desc->num_properties = ARRAY_SIZE(isl9538_charger_properties);
	supply_desc->get_property = isl9538_charger_get_property;
	supply_desc->set_property = isl9538_charger_set_property;
	supply_desc->property_is_writeable =
		isl9538_charger_property_is_writeable;

	psy_cfg.supplied_to = (char **)(&charger->pdata->fg_psy_name);
	psy_cfg.num_supplicants = 1;
	psy_cfg.of_node = client->dev.of_node;
	psy_cfg.drv_data = charger;

	i2c_set_clientdata(client, charger);

	ret = __isl9538_read_word(client, ISL9538_REG_MANUFACTURER_ID);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read manufacturer id : %d\n",
			ret);
		return ret;
	} else if (ret != DEFAULT_MANUFACTURER_ID) {
		dev_err(&client->dev,
			"manufacturer id mismatch. 0x0049 != 0x%04x\n", ret);
		return -ENODEV;
	}

	ret = __isl9538_read_word(client, ISL9538_REG_DEVICE_ID);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read manufacturer id : %d\n",
			ret);
		return ret;
	} else if (ret != DEFAULT_DEVICE_ID) {
		dev_err(&client->dev, "device id mismatch. 0x000C != 0x%04x\n",
			ret);
		return -ENODEV;
	}

	ret = isl9538_disable_watchdog_timer(charger);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to disable watchdog timer");
		return ret;
	}

	INIT_DELAYED_WORK(&charger->monitor_work, isl9538_monitor_workfunc);
	INIT_DELAYED_WORK(&charger->upa_monitor_work,
		isl9538_upa_monitor_workfunc);

	alarm_init(&charger->monitor_alarm,
			ALARM_BOOTTIME, isl9538_monitor_alarmcb);
	init_completion(&charger->resume_done);

	charger->charger =
		devm_power_supply_register(&client->dev, supply_desc, &psy_cfg);
	if (IS_ERR(charger->charger)) {
		ret = PTR_ERR(charger->charger);
		dev_err(&client->dev, "Failed to register power supply: %d\n",
			ret);
		return ret;
	}

	charger->acok = devm_gpiod_get(&client->dev, "acok", GPIOD_IN);
	if (IS_ERR(charger->acok)) {
		dev_err(&client->dev, "Error requesting acok gpio %ld\n",
			PTR_ERR(charger->acok));
	}

	irq = gpiod_to_irq(charger->acok);
	if (irq) {
		dev_err(&client->dev, "Succeed requesting irq:%d\n", irq);
		ret = devm_request_threaded_irq(
			&client->dev, irq, NULL, isl9538_irq,
			(IRQF_ONESHOT | IRQF_TRIGGER_RISING |
			 IRQF_TRIGGER_FALLING),
			"acok_irq", charger);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to request irq : %d\n",
				ret);
		} else {
			dev_err(&client->dev, "Request irq successful irq:%d\n",
				irq);
			charger->irq = irq;
		}
	}

	// Log the initial status of vsys
	charger->vsys_sts = isl9538_charger_is_vsys_enabled(charger);
	trace_enable_vsys(charger->vsys_sts);
	dev_info(&client->dev, "%s: vsys Status(%d)", __func__,
		charger->vsys_sts);

	if (isl9538_charger_is_present(charger))
		isl9538_charger_enable_vsys(charger, true);

	if (vadc_dev) {
		charger->vadc_dev = vadc_dev;
		ret = of_property_read_u32(client->dev.of_node,
				"isl,channel-num", &val);
		if (ret) {
			charger->channel_num = -1;
			charger->vadc_dev = NULL;
		} else
			charger->channel_num  = val;

		ret = np_get_adapter_vadc_map(charger,
				"isl,adapter-voltage-vadc-map");
		if (ret < 0) {
			pr_err("Error parsing isl,adapter-voltage-vadc-map ret=%d\n",
					ret);
			charger->vadc_dev = NULL;
		}
		isl9538_get_min_adpter_volt_charge(charger);
	}

	ret = subsys_virtual_register(&switch_subsys, NULL);
	if (ret) {
		pr_err("%s: subsys_virtual_register() failed ret(%d)\n",
			__func__, ret);
		return ret;
	}

	charger->invalid_charger_dev.bus = &switch_subsys;
	dev_set_name(&charger->invalid_charger_dev,
				"%s", INVALID_CHARGER_DEVNAME);

	ret = device_register(&charger->invalid_charger_dev);
	if (ret) {
		pr_err("%s: device_register() failed ret(%d)\n",
			__func__, ret);
		put_device(&charger->invalid_charger_dev);
		return ret;
	}

	charger->invalid_charger_dev.platform_data = (void *)charger;
	ret = device_create_file(&charger->invalid_charger_dev,
				&dev_attr_state);
	if (ret) {
		pr_err("%s: device_create_file() failed ret(%d)\n",
			__func__, ret);
		return ret;
	}

	/* Finished driver init */
	charger->monitor_gauge = true;

	device_init_wakeup(&charger->client->dev, true);

	/* Start periodic polling */
	schedule_delayed_work(&charger->upa_monitor_work, 0);

	return 0;
}

static void isl9538_charger_shutdown(struct i2c_client *client)
{
	bool connected;
	struct isl9538 *charger = i2c_get_clientdata(client);

	if (!charger)
		return;

	connected = isl9538_charger_is_present(charger) ? true : false;

	msm_set_charger_state(connected);
}

static int isl9538_charger_remove(struct i2c_client *client)
{
	struct isl9538 *charger = i2c_get_clientdata(client);

	if (!charger)
		return 0;

	device_init_wakeup(&charger->client->dev, false);

	return 0;
}

static int isl9538_charger_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl9538 *charger = i2c_get_clientdata(client);

	if (!charger || !charger->monitor_gauge)
		return 0;

	charger->suspended = false;

	if (charger->wakeup)
		disable_irq_wake(charger->irq);

	complete_all(&charger->resume_done);

	return 0;
}

static int isl9538_charger_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl9538 *charger = i2c_get_clientdata(client);

	if (!charger || !charger->monitor_gauge)
		return 0;

	charger->suspended = true;

	charger->wakeup = device_may_wakeup(dev);
	if (charger->wakeup)
		enable_irq_wake(charger->irq);

	reinit_completion(&charger->resume_done);
	return 0;
}

static const struct dev_pm_ops isl9538_charger_pm_ops = {
	.resume = isl9538_charger_resume,
	.suspend = isl9538_charger_suspend,
};

static const struct i2c_device_id isl9538_charger_id[] = {
	{ "isl9538-charger", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, isl9538_charger_id);

static const struct of_device_id isl9538_match_ids[] = {
	{ .compatible = "renesas,isl9538", },
	{ /* end */ }
};
MODULE_DEVICE_TABLE(of, isl9538 _match_ids);

static struct i2c_driver isl9538_charger_driver = {
	.driver = {
		.name = "isl9538-charger",
		.of_match_table = isl9538_match_ids,
		.pm = &isl9538_charger_pm_ops,
	},
	.probe = isl9538_charger_probe,
	.id_table = isl9538_charger_id,
	.shutdown = isl9538_charger_shutdown,
	.remove = isl9538_charger_remove,
};

module_i2c_driver(isl9538_charger_driver);

MODULE_DESCRIPTION("isl9538 battery charging driver");
MODULE_LICENSE("GPL v2");
