/*
 * dbmdx-clocks.c - DSPG DBMDX Clock Management
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
#include <linux/mutex.h>
#include <linux/clk.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif /* CONFIG_OF */
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>

static const char *dbmdx_of_clk_names[DBMDX_NR_OF_CLKS] = {
	"dbmdx_constant_clk",
	"dbmdx_master_clk",
};

#ifdef CONFIG_OF
static const char *dbmdx_of_clk_rate_names[DBMDX_NR_OF_CLKS] = {
	"constant-clk-rate",
	"master-clk-rate",
};
#endif

long dbmdx_clk_set_rate(struct dbmdx_private *p,
			       enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];
	int rate = p->pdata->clock_rates[dbmdx_clk];

	if (clk && (rate != -1))
		return clk_set_rate(clk, rate);

	return customer_dbmdx_clk_set_rate(p, dbmdx_clk);
}

unsigned long dbmdx_clk_get_rate(struct dbmdx_private *p,
					enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];
	int rate = p->pdata->clock_rates[dbmdx_clk];

	if (clk)
		return clk_get_rate(clk);

	if (rate)
		return rate;

	return customer_dbmdx_clk_get_rate(p, dbmdx_clk);
}

int dbmdx_clk_enable(struct dbmdx_private *p,
			    enum dbmdx_clocks dbmdx_clk)
{
	int ret = 0;
	struct clk *clk = p->clocks[dbmdx_clk];

	if (clk)
		ret = clk_prepare_enable(clk);
	else
		ret = customer_dbmdx_clk_enable(p, dbmdx_clk);

	if (ret < 0)
		dev_err(p->dev, "%s: %s clock enable failed\n",
			__func__,
			dbmdx_of_clk_names[dbmdx_clk]);
	else
		ret = 0;

	return ret;
}

int dbmdx_clk_disable(struct dbmdx_private *p,
			     enum dbmdx_clocks dbmdx_clk)
{
	struct clk *clk = p->clocks[dbmdx_clk];

	if (clk)
		clk_disable_unprepare(clk);
	else
		customer_dbmdx_clk_disable(p, dbmdx_clk);

	return 0;
}

#ifdef CONFIG_OF

int dbmdx_of_get_clk_info(struct dbmdx_private *p,
				 struct device_node *np,
				 enum dbmdx_clocks dbmdx_clk)
{
	int ret;
	int rate, rrate;
	struct clk *clk;

	ret = of_property_read_u32(np,
				   dbmdx_of_clk_rate_names[dbmdx_clk],
				   &rate);
	if (ret != 0) {
		dev_info(p->dev,
			 "%s: no %s definition in device-tree\n",
			 __func__,
			 dbmdx_of_clk_rate_names[dbmdx_clk]);
		rate = -1;
	} else
		dev_info(p->dev,
			 "%s: using %s at %dHZ from device-tree\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk],
			 rate);

	clk = clk_get(p->dev, dbmdx_of_clk_names[dbmdx_clk]);
	if (IS_ERR(clk)) {
		dev_info(p->dev,
			 "%s: no %s definition in device-tree\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk]);
		/* nothing in the device tree */
		clk = NULL;
	} else {
		/* If clock rate not specified in dts, try to detect */
		if (rate == -1) {
			rate = clk_get_rate(clk);
			dev_info(p->dev,
				 "%s: using %s at %dHZ\n",
				 __func__,
				 dbmdx_of_clk_names[dbmdx_clk],
				 rate);
		} else {
			/* verify which rate can be set */
			rrate = clk_round_rate(clk, rate);
			if (rrate !=  rate) {
				dev_info(p->dev,
					 "%s: rounded rate %d to %d\n",
					 __func__,
					 rate, rrate);
				rate = rrate;
			}
		}
	}
	p->clocks[dbmdx_clk] = clk;
	p->pdata->clock_rates[dbmdx_clk] = rate;

	return 0;
}
#else
int dbmdx_platform_get_clk_info(struct dbmdx_private *p,
				 enum dbmdx_clocks dbmdx_clk)
{
	int rate, rrate;
	struct clk *clk;
	struct dbmdx_platform_data *pdata = p->pdata;

	rate = pdata->clock_rates[dbmdx_clk];
	dev_info(p->dev,
		 "%s: using %s at %dHZ\n",
		 __func__,
		 dbmdx_of_clk_names[dbmdx_clk],
		 rate);

	clk = clk_get(p->dev, dbmdx_of_clk_names[dbmdx_clk]);
	if (IS_ERR(clk)) {
		dev_info(p->dev,
			 "%s: no %s definition\n",
			 __func__,
			 dbmdx_of_clk_names[dbmdx_clk]);
		/* nothing in the device tree */
		clk = NULL;
	} else {
		/* If clock rate not specified in dts, try to detect */
		if (rate == -1) {
			rate = clk_get_rate(clk);
			dev_info(p->dev,
				 "%s: using %s at %dHZ\n",
				 __func__,
				 dbmdx_of_clk_names[dbmdx_clk],
				 rate);
		} else {
			/* verify which rate can be set */
			rrate = clk_round_rate(clk, rate);
			if (rrate !=  rate) {
				dev_info(p->dev,
					 "%s: rounded rate %d to %d\n",
					 __func__,
					 rate, rrate);
				rate = rrate;
			}
		}
	}
	p->clocks[dbmdx_clk] = clk;
	p->pdata->clock_rates[dbmdx_clk] = rate;

	return 0;
}

#endif

