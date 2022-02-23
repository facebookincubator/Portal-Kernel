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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/compiler.h>
#include <linux/compat.h>
#include "mcubeclass.h"

static LIST_HEAD(mcube_dev_list);

/*
 * mcube_mutex protects access to both mcube_dev_list and input_handler_list.
 * This also causes mcube_[un]register_device and mcube_[un]register_handler
 * be mutually exclusive which simplifies locking in drivers implementing
 * input handlers.
 */
static DEFINE_MUTEX(mcube_mutex);

static void mcube_dev_release(struct device *device)
{
	struct mcube_dev *dev = to_mcube_dev(device);

	if (dev != NULL)
		kfree(dev);
	module_put(THIS_MODULE);
}

#ifdef CONFIG_PM
static int mcube_dev_suspend(struct device *dev)
{
	return 0;
}

static int mcube_dev_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops mcube_dev_pm_ops = {
	.suspend    = mcube_dev_suspend,
	.resume     = mcube_dev_resume,
	.poweroff   = mcube_dev_suspend,
	.restore    = mcube_dev_resume,
};
#endif /* CONFIG_PM */

static const struct attribute_group *mcube_dev_attr_groups[] = {
	NULL
};

static struct device_type mcube_dev_type = {
	.groups      = mcube_dev_attr_groups,
	.release = mcube_dev_release,
#ifdef CONFIG_PM
	.pm      = &mcube_dev_pm_ops,
#endif
};

struct class mcube_class = {
	.name        = "mcube",
	.owner       = THIS_MODULE,
	.dev_release = mcube_dev_release,
};
EXPORT_SYMBOL_GPL(mcube_class);

/**
 * mcube_allocate_device - allocate memory for new input device
 *
 * Returns prepared struct mcube_dev or NULL.
 *
 * NOTE: Use mcube_free_device() to free devices that have not been
 * registered; mcube_unregister_device() should be used for already
 * registered devices.
 */
struct mcube_dev *mcube_allocate_device(void)
{
	struct mcube_dev *dev;

	dev = kzalloc(sizeof(struct mcube_dev), GFP_KERNEL);
	if (dev) {
		dev->dev.type = &mcube_dev_type;
		dev->dev.class = &mcube_class;
		device_initialize(&dev->dev);
		mutex_init(&dev->mutex);
		INIT_LIST_HEAD(&dev->node);
		__module_get(THIS_MODULE);
	}
	return dev;
}
EXPORT_SYMBOL(mcube_allocate_device);

/**
 * mcube_free_device - free memory occupied by mcube_dev structure
 * @dev: input device to free
 *
 * This function should only be used if mcube_register_device()
 * was not called yet or if it failed. Once device was registered
 * use mcube_unregister_device() and memory will be freed once last
 * reference to the device is dropped.
 *
 * Device should be allocated by mcube_allocate_device().
 *
 * NOTE: If there are references to the input device then memory
 * will not be freed until last reference is dropped.
 */
void mcube_free_device(struct mcube_dev *dev)
{
	if (dev)
		mcube_put_device(dev);
}
EXPORT_SYMBOL(mcube_free_device);

/**
 * mcube_register_device - register device with input core
 * @dev: device to be registered
 *
 * This function registers device with input core. The device must be
 * allocated with mcube_allocate_device() and all it's capabilities
 * set up before registering.
 * If function fails the device must be freed with mcube_free_device().
 * Once device has been successfully registered it can be unregistered
 * with mcube_unregister_device(); mcube_free_device() should not be
 * called in this case.
 */
int mcube_register_device(struct mcube_dev *dev)
{
	const char *path;
	int error;

	/*
	 * If delay and period are pre-set by the driver, then autorepeating
	 * is handled by the driver itself and we don't do it in input.c.
	 */
	dev_set_name(&dev->dev, dev->name);

	error = device_add(&dev->dev);
	if (error)
		return error;

	path = kobject_get_path(&dev->dev.kobj, GFP_KERNEL);
	pr_info("%s as %s\n",
			dev->name ? dev->name : "Unspecified device",
			path ? path : "N/A");
	kfree(path);
	error = mutex_lock_interruptible(&mcube_mutex);
	if (error) {
		device_del(&dev->dev);
		return error;
	}

	list_add_tail(&dev->node, &mcube_dev_list);

	mutex_unlock(&mcube_mutex);
	return 0;
}
EXPORT_SYMBOL(mcube_register_device);

/**
 * mcube_unregister_device - unregister previously registered device
 * @dev: device to be unregistered
 *
 * This function unregisters an input device. Once device is unregistered
 * the caller should not try to access it as it may get freed at any moment.
 */
void mcube_unregister_device(struct mcube_dev *dev)
{
	int error;

	error = mutex_lock_interruptible(&mcube_mutex);
	if (error)
		return;

	list_del_init(&dev->node);
	mutex_unlock(&mcube_mutex);
	device_unregister(&dev->dev);
}
EXPORT_SYMBOL(mcube_unregister_device);

static int __init mcube_init(void)
{
	int err;
	/*mcube class register*/
	err = class_register(&mcube_class);
	if (err) {
		pr_err("unable to register mcube_dev class\n");
		return err;
	}
	return err;
}

static void __exit mcube_exit(void)
{
	/*mcube class*/
	class_unregister(&mcube_class);
}

/*subsys_initcall(mcube_init);*/

MODULE_AUTHOR("contact@bosch-sensortec.com");
MODULE_DESCRIPTION("BST CLASS CORE");
MODULE_LICENSE("GPL");

module_init(mcube_init);
module_exit(mcube_exit);
