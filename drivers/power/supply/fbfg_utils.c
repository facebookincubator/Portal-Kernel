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

#define pr_fmt(fmt) "[fbfg-utils] %s: " fmt, __func__

#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/power/fbfg_utils.h>
#include <linux/slab.h>


static struct fbfg_utils *fbfg_data;

static int fbfg_utils_get_dsoc_from_remcap(int src_b, int src_t, int des_b,
	int des_t, int full_charge, int rem_cap, u16 *val)
{
	int bottom_cap; /* capatity for 0 ~ src_b - 1 */
	int top_cap;	/* capacity for 0 ~ src_t */
	u16 dsoc;	/* display soc */

	/*
	 * to remove bottom area, get capacity
	 * for 0 ~ src_b - 1 area
	 */
	bottom_cap = (src_b - 1) * full_charge / 100;

	/*
	 * Capacity for 0 ~ src_t
	 */
	top_cap = src_t *full_charge / 100;

	if (rem_cap < bottom_cap || bottom_cap == top_cap) {
		pr_err("abnormal rem_cap %d, bottom %d, top %d -> return min\n",
			rem_cap, bottom_cap, top_cap);
		*val = des_b;
		return 0;
	}

	/*
	 * Get remaining capacity range in des_b ~ des_t range
	 * The range is des_t - des_b + 1
	 */
	dsoc = (((rem_cap - bottom_cap) *
			(des_t - des_b + 1)) /
			(top_cap - bottom_cap)) + des_b;

	/* dsoc should not be larger then des_t */
	if (dsoc > des_t)
		dsoc = des_t;

	pr_debug("rem_cap = %d, full_charge = %d, dsoc = %d\n",
		rem_cap, full_charge, dsoc);

	*val = dsoc;

	return 0;
}

/*
 * Remap gsoc using the threshold values
 */
int fbfg_utils_soc_static_remap(u16 gsoc, int full_charge,
	int rem_cap, int *val)
{
	int ret = 0;
	u16 dsoc = 0;

	if (!fbfg_data)
		return -ENXIO;

	if (gsoc <= fbfg_data->soc_low_thr) {
		if (gsoc == 0)
			dsoc = 0;
		else
			dsoc = SOC_LOWBAT_LEVEL;
	} else if (gsoc >= fbfg_data->soc_recharge_thr) {
		if (fbfg_data->soc_extra_thr) {
			if (gsoc >= fbfg_data->soc_recharge_thr
				+ fbfg_data->soc_extra_thr) {
				dsoc = SOC_FULL_LEVEL;
				fbfg_data->soc_extra_thr = 0;
			} else {
				dsoc = SOC_FULL_LEVEL - 1;
			}
		} else {
			dsoc = SOC_FULL_LEVEL;
		}
	} else {
		/*
		 * Map rem_cap in range soc_low_thr + 1 ~ soc_recharge_thr -1 to
		 * to (SOC_LOWBAT_LEVE + 1) ~ (SOC_FULL_LEVEL - 1)
		 */
		ret = fbfg_utils_get_dsoc_from_remcap(
			fbfg_data->soc_low_thr + 1,
			fbfg_data->soc_recharge_thr - 1,
			SOC_LOWBAT_LEVEL + 1, SOC_FULL_LEVEL - 1,
			full_charge, rem_cap, &dsoc);

		if (ret < 0) {
			pr_err("fbfg_utils_get_dsoc_from_remcap error rc=%d\n",
					ret);
			return ret;
		}

		/*
		 * When dsoc reaches SOC_FULL_LEVEL -1, it requires to
		 * one more gsoc capacity level to become SOC_FULL_LEVEL.
		 * This is to gaurantee that SOC_FULL_LEVEL can be maintained
		 * for at least one full gsoc capacity.
		 */
		if (dsoc == SOC_FULL_LEVEL - 1)
			fbfg_data->soc_extra_thr = 1;
	}

	pr_debug("gsoc = %d, dsoc = %d\n", gsoc, dsoc);

	*val = dsoc;

	return 0;
}

void fbfb_utils_set_init_status(int gsoc, int batt_status)
{
	if (!fbfg_data) {
		pr_err("Initialize fbfg_data first");
		return;
	}

	/*
	 * If gsoc is the same as recharge threshold and charging
	 * was not complete, then extra threshold must have been set
	 * before reboot. In that case, set the extra threshold to manage
	 * the correct 100% dsoc capacity
	 */
	if (gsoc == fbfg_data->soc_recharge_thr && batt_status
		!= POWER_SUPPLY_STATUS_FULL)
		fbfg_data->soc_extra_thr = 1;
}

int fbfg_utils_init(struct device_node *node)
{
	int ret;
	bool static_remap = false;

	if (!node) {
		pr_err("node is null. Failed fb fg utils initialziation");
		return -ENXIO;
	}

	fbfg_data = kzalloc(sizeof(struct fbfg_utils), GFP_KERNEL);
	if (!fbfg_data)
		return -ENOMEM;

	ret = of_property_read_u32(node, "fb,fg-soc-recharge-thr",
					&fbfg_data->soc_recharge_thr);
	if (ret < 0) {
		pr_debug("no fb,fg-soc-recharge-thr definition\n");
		fbfg_data->soc_recharge_thr = DEFAULT_SOC_RECHARGE_THR;
	}

	ret = of_property_read_u32(node, "fb,fg-soc-low-thr",
					&fbfg_data->soc_low_thr);
	if (ret < 0) {
		pr_debug("no fb,fg-soc-low-thr definition\n");
		fbfg_data->soc_low_thr = DEFAULT_SOC_LOW_THR;
	}

	/*
	 * We check the fg-soc-static-remap feature only when
	 * the related parameters are valid
	 */
	if (fbfg_data->soc_recharge_thr >= SOC_MIN_RECHARGE_THR &&
			fbfg_data->soc_recharge_thr <= SOC_MAX_RECHARGE_THR &&
			fbfg_data->soc_low_thr >= SOC_MIN_LOW_THR &&
			fbfg_data->soc_low_thr <= SOC_MAX_LOW_THR) {
		static_remap = of_property_read_bool(node,
			"fb,fg-soc-static-remap");
	}

	pr_info("soc_recharge_thr:%d soc_low_thr:%d static_remap:%d",
		fbfg_data->soc_recharge_thr, fbfg_data->soc_low_thr,
			(int)static_remap);

	if (static_remap)
		return 0;

	return -ENODEV;
}
