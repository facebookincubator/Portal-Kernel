/*
 * dbmdx-clocks.h  --  DBMDX Chip Interfaces API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_CLOCKS_H
#define _DBMDX_CLOCKS_H

long dbmdx_clk_set_rate(struct dbmdx_private *p,
			       enum dbmdx_clocks dbmdx_clk);

unsigned long dbmdx_clk_get_rate(struct dbmdx_private *p,
					enum dbmdx_clocks dbmdx_clk);

int dbmdx_clk_enable(struct dbmdx_private *p,
			    enum dbmdx_clocks dbmdx_clk);

int dbmdx_clk_disable(struct dbmdx_private *p,
			     enum dbmdx_clocks dbmdx_clk);

#ifdef CONFIG_OF
int dbmdx_of_get_clk_info(struct dbmdx_private *p,
				 struct device_node *np,
				 enum dbmdx_clocks dbmdx_clk);
#else
int dbmdx_platform_get_clk_info(struct dbmdx_private *p,
				 enum dbmdx_clocks dbmdx_clk);
#endif /* CONFIG_OF */

#endif /* _DBMDX_CHIP_INTERFACES_H */
