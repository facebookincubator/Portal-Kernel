/*!
 * @section LICENSE
 * Copyright (c) 2015 mCube, Inc.  All rights reserved.
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename mcubeclass.c
 *
 * @brief
 * The core code of mcube device driver
 */

#ifndef _MCUBECLASS_H
#define _MCUBECLASS_H

#ifdef __KERNEL__
#include <linux/time.h>
#include <linux/list.h>
#else
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/types.h>
#endif

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mod_devicetable.h>

struct mcube_dev {
	const char *name;
	int (*open)(struct mcube_dev *dev);
	void (*close)(struct mcube_dev *dev);
	struct mutex mutex;
	struct device dev;
	struct list_head node;
};

#define to_mcube_dev(d) container_of(d, struct mcube_dev, dev)

struct mcube_dev *mcube_allocate_device(void);
void mcube_free_device(struct mcube_dev *dev);

static inline struct mcube_dev *mcube_get_device(struct mcube_dev *dev)
{
	return dev ? to_mcube_dev(get_device(&dev->dev)) : NULL;
}

static inline void mcube_put_device(struct mcube_dev *dev)
{
	if (dev)
		put_device(&dev->dev);
}

static inline void *mcube_get_drvdata(struct mcube_dev *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void mcube_set_drvdata(struct mcube_dev *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

int __must_check mcube_register_device(struct mcube_dev *dev);
void mcube_unregister_device(struct mcube_dev *dev);

void mcube_reset_device(struct mcube_dev *dev);

extern struct class mcube_class;

#endif
