/*
 * dbmdx-of.h  --  DSPG DBMDX Device Tree Data API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_OF_H
#define _DBMDX_OF_H


int dbmdx_read_config_from_devtree(struct dbmdx_private *p);

int dbmdx_deinit_devtree_config(struct dbmdx_private *p);

#endif /* DBMDX_OF_H */

