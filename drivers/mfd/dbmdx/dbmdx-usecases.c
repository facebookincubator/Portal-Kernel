/*
 * dbmdx-usecases.c -- DSPG DBMDX usecases common functions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmdx-common-operations.h>
#include <linux/mfd/dbmdx/dbmdx-utils.h>

#include <linux/mfd/dbmdx/dbmdx-usecases.h>

struct usecase_config *usecases_map[] = {
};

int dbmdx_usecase_manager(struct dbmdx_private *p, u32 cmd)
{
	dev_info(p->dev,
		"%s:Usecase Manager Command: 0x%x\n", __func__, cmd);

	return 0;
}

int dbmdx_get_number_of_registered_usecases(void)
{
	int num_usecases = sizeof(usecases_map) /
				sizeof(struct usecase_config *);
	return num_usecases;
}

int hw_stop_usecase(struct dbmdx_private *p,
					struct usecase_config *uc_config)
{
	if (!uc_config || !uc_config->use_hw_usecase_stop)
		return 0;

	if (!p || !p->pdata || !(p->cur_wakeup_gpio < 0))
		return -EIO;
	gpio_set_value(p->cur_wakeup_gpio, p->pdata->wakeup_set_value);

	dev_info(p->dev, "%s: %d==>gpio%d\n", __func__,
		p->pdata->wakeup_set_value, p->cur_wakeup_gpio);

	return 0;
}

int hw_stop_usecase_prepare(struct dbmdx_private *p,
					struct usecase_config *uc_config)
{
	if (!uc_config || !uc_config->use_hw_usecase_stop)
		return 0;

	if (!p || !p->pdata || !(p->cur_wakeup_gpio < 0))
		return -EIO;

	gpio_set_value(p->cur_wakeup_gpio, !(p->pdata->wakeup_set_value));

	dev_info(p->dev, "%s: %d==>gpio%d\n", __func__,
		!(p->pdata->wakeup_set_value), p->cur_wakeup_gpio);

	return 0;
}

int switch_to_usecase(struct dbmdx_private *p, struct usecase_config *uc_config)
{
	struct usecase_config *active_usecase;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	active_usecase = p->primary_flags.active_usecase;

	if (uc_config == NULL && active_usecase == NULL) {
		dev_info(p->dev, "%s: Transition from IDLE to IDLE\n",
								__func__);
		return 0;
	} else if (uc_config == NULL) {
		ret = active_usecase->usecase_exit(p, active_usecase);
		if (ret < 0) {
			dev_err(p->dev, "%s: Error exiting usecase %s\n",
				__func__, active_usecase->usecase_name);
			return ret;
		}
		p->primary_flags.active_usecase = NULL;
		return 0;
	} else if (active_usecase == uc_config) {
		dev_err(p->dev, "%s: Usecase us already running\n", __func__);
		return -EINVAL;
	}

	if (active_usecase != NULL) {
		ret = active_usecase->usecase_exit(p, active_usecase);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error exiting usecase %s\n",
				__func__, active_usecase->usecase_name);
			return ret;
		}
	}

	p->primary_flags.active_usecase = NULL;

	hw_stop_usecase_prepare(p, uc_config);

	ret = uc_config->usecase_enter(p, uc_config);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error starting usecase %s\n",
			__func__, uc_config->usecase_name);
		return ret;
	}

	p->primary_flags.active_usecase = uc_config;

	return 0;
}

int find_usecase_by_name(struct dbmdx_private *p, const char *usecase_name,
			struct usecase_config **usecase)
{
	struct usecase_config *cur_usecase;
	int ind;
	int n;
	int num_usecases;

	num_usecases = dbmdx_get_number_of_registered_usecases();

	n = strlen(usecase_name);

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		return -EINVAL;
	}

	if (usecase_name[n-1] == '\n')
		n = n - 1;

	if (!n) {
		dev_err(p->dev,	"%s: No usecase name supplied\n", __func__);
		return -EINVAL;
	}

	*usecase = NULL;
	/* IDLE Usecase */
	if (!strncmp(usecase_name, "uc_idle", 7))
		return 0;

	for (ind = 0; ind < num_usecases; ind++) {
		cur_usecase = usecases_map[ind];

		if (!cur_usecase)
			continue;
		if (n != strlen(cur_usecase->usecase_name))
			continue;
		if (!strncmp(usecase_name, cur_usecase->usecase_name, n)) {
			/* Verify HW Rev */
			if (p->pdata->hw_rev == DBMDX_DEFAULT_HW_REV)
				break;
			if (p->pdata->hw_rev != cur_usecase->hw_rev)
				continue;
			break;
		}
	}

	if (ind >= num_usecases || !cur_usecase) {
		dev_err(p->dev, "%s: Unsupported usecase %s\n", __func__,
								usecase_name);
		return -EINVAL;
	}

	*usecase = cur_usecase;

	return 0;
}
