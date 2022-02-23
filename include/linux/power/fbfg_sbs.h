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

#ifndef _FB_FG_SBS_H
#define _FB_FG_SBS_H

#include <linux/power/fbfg_prop.h>

#define FG_FLAGS_UTD BIT(27) /* Undertemperature during discharge */
#define FG_FLAGS_UTC BIT(26) /* Undertemperature during charge */
#define FG_FLAGS_OTD BIT(13) /* Overtemperature during discharge */
#define FG_FLAGS_OTC BIT(12) /* Overtemperature during charge */

#define SOC_MIN_RECHARGE_THR 90
#define SOC_MAX_RECHARGE_THR 99
#define SOC_MIN_LOW_THR 1
#define SOC_MAX_LOW_THR 10

#define SOC_LOWBAT_LEVEL 1
#define SOC_FULL_LEVEL 100

#define DEFAULT_MAX_INIT_RETRY_COUNT 10
#define DEFAULT_POLL_INTERVAL_MS 60000

/* 500 ms from alarm timer until driver resumes */
#define FBFG_RESUME_TIMEOUT_MS 500

#define MANUFACTURER_NAME_LENGTH 20
#define DEVICE_NAME_LENGTH 20
#define DEVICE_CHEM_LENGTH 20

/* BatterStatus() Bit */
#define FBFG_FLAGS_FD BIT(4)
#define FBFG_FLAGS_FC BIT(5)
#define FBFG_FLAGS_DSG BIT(6)
#define FBFG_FLAGS_INIT BIT(7)
#define FBFG_FLAGS_RCA BIT(9)

/* BatteryMode() Bit */
#define FBFG_CAP_MODE BIT(15)

/* Ship mode*/
#define FBFG_MAC_CMD_SHUTDOWN 0x0010

#define DEFAULT_TEMP 280
#define DEFAULT_RSOC 50
#define DEFAULT_NUM_CELLS 4

enum FBFG_SBS_REG_IDX {
	FBFG_SBS_REG_BATT_MODE,
	FBFG_SBS_REG_AT_RAT,
	FBFG_SBS_REG_RATE_FULL,
	FBFG_SBS_REG_RATE_EMPTY,
	FBFG_SBS_REG_RATE_OK,
	FBFG_SBS_REG_TEMP,
	FBFG_SBS_REG_VOLT_NOW,
	FBFG_SBS_REG_CURR_NOW,
	FBFG_SBS_REG_CURR_AVG,
	FBFG_SBS_REG_MAX_ERR,
	FBFG_SBS_REG_REL_SOC,
	FBFG_SBS_REG_ABS_SOC,
	FBFG_SBS_REG_REM_CAP,
	FBFG_SBS_REG_FCC,
	FBFG_SBS_REG_RTE,
	FBFG_SBS_REG_AVG_TTE,
	FBFG_SBS_REG_AVG_TTF,
	FBFG_SBS_REG_CHG_CURR,
	FBFG_SBS_REG_CHG_VOLT,
	FBFG_SBS_REG_BATT_STS,
	FBFG_SBS_REG_CYC_CNT,
	FBFG_SBS_REG_DESIGN_CAP,
	FBFG_SBS_REG_DESIGN_VOLT,
	FBFG_SBS_REG_SPEC_INFO,
	FBFG_SBS_REG_MFG_DATE,
	FBFG_SBS_REG_SER_NUM,
	FBFG_SBS_REG_MFR_ACC,
	FBFG_SBS_REG_MAN_NAME,
	FBFG_SBS_REG_DEV_NAME,
	FBFG_SBS_REG_DEV_CHEM,
	FBFG_SBS_REG_MFR_DATA,
	FBFG_SBS_REG_NR,
};

#define FBFG_CHIP_ENTRY(_addr, _min_value, _max_value) { \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

struct fbfg_reg_data {
	u8 addr;
	int min_value;
	int max_value;
};

struct fbfg_reg_data fbfg_sbs_data[FBFG_SBS_REG_NR] = {
	/* Numeric Data */
	[FBFG_SBS_REG_BATT_MODE] = FBFG_CHIP_ENTRY(0x03, 0, 2),
	[FBFG_SBS_REG_AT_RAT] = FBFG_CHIP_ENTRY(0x04, -32768, 32767),
	[FBFG_SBS_REG_RATE_FULL] = FBFG_CHIP_ENTRY(0x05, 0, 65535),
	[FBFG_SBS_REG_RATE_EMPTY] = FBFG_CHIP_ENTRY(0x06, 0, 65535),
	[FBFG_SBS_REG_RATE_OK] = FBFG_CHIP_ENTRY(0x07, 0, 65535),
	[FBFG_SBS_REG_TEMP] = FBFG_CHIP_ENTRY(0x08, 0, 65535),
	[FBFG_SBS_REG_VOLT_NOW] = FBFG_CHIP_ENTRY(0x09, 0, 65535),
	[FBFG_SBS_REG_CURR_NOW] = FBFG_CHIP_ENTRY(0x0A, -32768, 32767),
	[FBFG_SBS_REG_CURR_AVG] = FBFG_CHIP_ENTRY(0x0B, -32768, 32767),
	[FBFG_SBS_REG_MAX_ERR] = FBFG_CHIP_ENTRY(0x0C, 0, 100),
	[FBFG_SBS_REG_REL_SOC] = FBFG_CHIP_ENTRY(0x0D, 0, 65535),
	[FBFG_SBS_REG_ABS_SOC] = FBFG_CHIP_ENTRY(0x0E, 0, 65535),
	[FBFG_SBS_REG_REM_CAP] = FBFG_CHIP_ENTRY(0x0F, 0, 65535),
	[FBFG_SBS_REG_FCC] = FBFG_CHIP_ENTRY(0x10, 0, 65535),
	[FBFG_SBS_REG_RTE] = FBFG_CHIP_ENTRY(0x11, 0, 65535),
	[FBFG_SBS_REG_AVG_TTE] = FBFG_CHIP_ENTRY(0x12, 0, 65535),
	[FBFG_SBS_REG_AVG_TTF] = FBFG_CHIP_ENTRY(0x13, 0, 65535),
	[FBFG_SBS_REG_CHG_CURR] = FBFG_CHIP_ENTRY(0x14, 0, 65535),
	[FBFG_SBS_REG_CHG_VOLT] = FBFG_CHIP_ENTRY(0x15, 0, 65535),
	[FBFG_SBS_REG_BATT_STS] = FBFG_CHIP_ENTRY(0x16, 0, 65535),
	[FBFG_SBS_REG_CYC_CNT] = FBFG_CHIP_ENTRY(0x17, 0, 65535),
	[FBFG_SBS_REG_DESIGN_CAP] = FBFG_CHIP_ENTRY(0x18, 0, 65535),
	[FBFG_SBS_REG_DESIGN_VOLT] = FBFG_CHIP_ENTRY(0x19, 0, 65535),
	[FBFG_SBS_REG_SPEC_INFO] = FBFG_CHIP_ENTRY(0x1A, 0, 65535),
	[FBFG_SBS_REG_MFG_DATE] = FBFG_CHIP_ENTRY(0x1B, 0, 65535),
	[FBFG_SBS_REG_SER_NUM] = FBFG_CHIP_ENTRY(0x1C, 0, 65535),

	/* BLCOK READ  */
	[FBFG_SBS_REG_MFR_ACC] = FBFG_CHIP_ENTRY(0x00, 0, 0),
	[FBFG_SBS_REG_MAN_NAME] = FBFG_CHIP_ENTRY(0x20, 0, 0),
	[FBFG_SBS_REG_DEV_NAME] = FBFG_CHIP_ENTRY(0x21, 0, 0),
	[FBFG_SBS_REG_DEV_CHEM] = FBFG_CHIP_ENTRY(0x22, 0, 0),
	[FBFG_SBS_REG_MFR_DATA] = FBFG_CHIP_ENTRY(0x23, 0, 0),
};

enum FBFG_SBS_PROP_IDX {
	FBFG_SBS_PROP_MAF_NAME,
	FBFG_SBS_PROP_DEV_NAME,
	FBFG_SBS_PROP_DEV_CHEM,
	FBFG_SBS_PROP_NR,
};

struct fbfg_prop_desc fbfg_sbs_props_data[FBFG_SBS_PROP_NR] = {
	[FBFG_SBS_PROP_MAF_NAME] = {"manufacturer_name", SMBUS_BLOCK,
		0x20, 0,
		GG_ST, PARAM_NUM_ONE, 0, MANUFACTURER_NAME_LENGTH},
	[FBFG_SBS_PROP_DEV_NAME] = {"device_name", SMBUS_BLOCK,
		0x21, 0,
		GG_ST, PARAM_NUM_ONE, 0, DEVICE_NAME_LENGTH},
	[FBFG_SBS_PROP_DEV_CHEM] = {"device_chemistry", SMBUS_BLOCK,
		0x22, 0,
		GG_ST, PARAM_NUM_ONE, 0, DEVICE_CHEM_LENGTH},
};

struct fbfg_batt_sts {
	bool batt_init; /* Initialized */
	bool batt_dsg;  /* Discharging */
	bool batt_rca;
	bool batt_fc;
	bool batt_fd;	/* full depleted */
};

struct fbfg_platform {
	int soc;
	int real_soc;
	int remaining_cap;
	int charge_full;
	int cycle_cnt;
	int batt_health;
	int batt_status;
	int chg_curr;
	int chg_volt;
	int curr_now;
	int volt_now;

	int temp;
	int fake_temp;
	bool fake_soc_en;
	int fake_soc;

	int technology;

	// Safety Related
	int safety_status;
	int op_status;
	int pf_status;
	int gauge_alert;
};

struct fbfg_prop_attribute {
	enum FBFG_PROP_IDX fbfg_prop_idx;
	struct device_attribute da;
};

struct fbfg_chip {
	struct device *dev;
	struct i2c_client *client;
	int i2c_retry_limit;
	int num_cells;

	struct fbfg_reg_data *sbs_reg_data;
	struct fbfg_prop_desc *prop_descs;
	struct fbfg_prop_desc *sbs_prop_descs;

	struct fbfg_platform *pdata;
	struct mutex data_lock;
	struct mutex mba_lock;

	struct delayed_work monitor_work;
	struct delayed_work delayed_init_work;

	int init_retry_count;
	int init_retry_limit;

	bool batt_ok;
	bool soc_remap;

	struct power_supply *power_supply;
	struct power_supply_desc psy_desc;

	bool resume_completed;
	struct completion resume_done;

	struct alarm monitor_alarm;
	bool pm_stay_awake;
	int monitor_period_ms;

	int err_temp_cnt;
	struct fbfg_prop_attribute prop_das[FBFG_PROP_NR];
	struct fbfg_prop_attribute sbs_prop_das[FBFG_SBS_PROP_NR];
};

#endif
