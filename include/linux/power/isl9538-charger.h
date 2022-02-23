/*
 * Copyright (C) 2020 Facebook Inc.
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

#ifndef __CHARGER_ISL9538_H_
#define __CHARGER_ISL9538_H_

#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <dt-bindings/power/renesas,isl9538.h>

struct isl9538_platform {
	uint32_t charge_current; /* Charging current limit in uA */
	uint32_t user_current_limit; /* User requested charging current lmt uA */
	uint32_t charge_voltage; /* Charging voltage limit in uV */
	uint32_t prev_charge_voltage; /* Prev. charging voltage limit in uV */
	uint32_t prev_charge_current; /* Prev. charging current limit in uA */
	uint32_t adapter_curr_limit_2; /* Adapter current limit 2 in uA */
	uint32_t adapter_curr_limit_1; /* Adapter current limit 1 in uA */
	uint32_t trickle_charge_curr;
	uint32_t interval_normal_s; /* charging interval for normal state */
	uint32_t interval_lowbatt_s; /* charging interval for low battery state */
	uint32_t interval_maintenance_s; /* charging interval for maintenance mode */
	uint32_t lowbatt_cap; /* battery capacity to use interval_lowbat_s */
	uint32_t hi_chg_lmt;
	uint32_t hi_clr_thr;
	uint32_t hi_poll_thr;
	uint32_t interval_hibatt_s;
	int adapter_uv;

	const char *name;
	const char *fg_psy_name;
};

struct isl9538 {
	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct i2c_client *client;
	struct isl9538_platform *pdata;
	struct mutex current_lock;
	struct mutex voltage_lock;
	struct delayed_work monitor_work;
	struct delayed_work upa_monitor_work;
	bool user_disable_charging;
	bool battfull_disable_charging;
	bool batthi_disable_charging;
	bool adapter_disable_charging;
	bool wrong_charger_state;
	bool monitor_gauge;
	bool suspended;
	bool wakeup;
	unsigned int irq;
	bool charging_en;
	bool vsys_sts;
	struct qpnp_vadc_chip *vadc_dev;
	int channel_num;
	int *adapter_available_uvs;
	int *adapter_vadc_uvs;
	int num_adapter_uv;
	int trickle_charge_count;

	/* Min adapter volt to enable charging */
	int min_adapter_charge_uv;

	struct gpio_desc *acok;

	struct alarm monitor_alarm;
	struct completion resume_done;
	struct device invalid_charger_dev;
};


#endif /* __CHARGER_ISL9538_H_ */
