/*
 * dbmdx-sysfs.h -- DSPG DBMDX SysFS Interface
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_SYSFS_H
#define _DBMDX_SYSFS_H

int dbmdx_init_common_sysfs_group(struct dbmdx_private *p);

int dbmdx_remove_common_sysfs_group(struct dbmdx_private *p);

#endif /* DBMDX_SYSFS_H */

