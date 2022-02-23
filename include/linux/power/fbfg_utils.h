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


#ifndef _FB_FG_UTILS_H
#define _FB_FG_UTILS_H

#define SOC_MIN_RECHARGE_THR 90
#define SOC_MAX_RECHARGE_THR 99
#define SOC_MIN_LOW_THR 1
#define SOC_MAX_LOW_THR 10

#define SOC_LOWBAT_LEVEL 1
#define SOC_FULL_LEVEL 100

#define DEFAULT_SOC_RECHARGE_THR 95
#define DEFAULT_SOC_LOW_THR 3

struct fbfg_utils {
	int soc_recharge_thr; /* soc recharge threshold in percentage (ex 95) */
	int soc_low_thr; /* soc low battery threshold in percentage (ex 3) */
	int soc_extra_thr; /* extra threshold for dsoc 100% */
	bool soc_static_remap; /* remap soc */
};

#ifdef CONFIG_BATTERY_FBFG
int fbfg_utils_soc_static_remap(u16 gsoc, int full_charge,
	int rem_cap, int *val);
void fbfb_utils_set_init_status(int gsoc, int batt_status);
int fbfg_utils_init(struct device_node *node);
#else

inline int fbfg_utils_init(struct device_node *node)
{
	return -ENODEV;
}

inline int fbfg_utils_soc_static_remap(u16 gsoc, int full_charge,
	int rem_cap, int *val)
{
	*val = gsoc;
	return 0;
}

inline void fbfb_utils_set_init_status(int gsoc, int batt_status) {}
#endif

#endif
