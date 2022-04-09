/*
 * dbmdx-cdev-core.h  --  DBMDX CDEV interface common functions
 *
 * Copyright (c) 2017, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DBMDX_CDEV_CORE_H
#define _DBMDX_CDEV_CORE_H

#include <linux/cdev.h>

struct dbmdx_cdev {
	struct cdev cdev;
	struct device *dev;
};

int dbmdx_cdev_init(void);
void dbmdx_cdev_exit(void);

struct dbmdx_cdev *dbmdx_cdev_from_inode(struct inode *inode);

struct dbmdx_cdev *dbmdx_cdev_from_file(struct file *file);

int dbmdx_cdev_create(struct dbmdx_cdev *cdev, struct device *parent,
	const struct file_operations *fops, void *drvdata, char *fmt, ...);

int dbmdx_cdev_destroy(struct dbmdx_cdev *cdev);

#endif
