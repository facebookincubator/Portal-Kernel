/*
 * DSPG DBMDX codec driver character device interface
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mfd/dbmdx/dbmdx-cdev-core.h>

#define DBMDX_CDEV_MINOR	0
#define DBMDX_CDEV_COUNT	8

#define LONG_BITS		(sizeof(long) * 8)

struct dbmdx_cdevs_ctx {
	struct class *class;
	dev_t major, minor;
	unsigned long indexes[ALIGN(DBMDX_CDEV_COUNT, LONG_BITS) / LONG_BITS];
};

struct dbmdx_cdevs_ctx dbmdx_cdevs = {NULL};

static int dbmdx_cdev_get_index(char *name)
{
	struct dbmdx_cdevs_ctx *cdevs = &dbmdx_cdevs;
	int i;

	/* Requred index of reserved character devices */
	if (sscanf(name, "dbmdx-%d", &i) == 1)
		return i;

	/* Find first zero index */
	for (i = 0; i < ARRAY_SIZE(cdevs->indexes); i++)
		if (cdevs->indexes[i] != ~0UL )
			return ffz(cdevs->indexes[i]);

	return 255;
}

static int dbmdx_cdev_create_name(struct dbmdx_cdev *cdev, char *name,
	struct device *parent, const struct file_operations *fops, void *data)
{
	struct dbmdx_cdevs_ctx *cdevs = &dbmdx_cdevs;
	int minor, err = -EINVAL;
	dev_t devno;

	/* Register charecter device region for DBMDX */
	err = dbmdx_cdev_init();
	if (err)
		return err;

	cdev_init(&cdev->cdev, fops);
	cdev->cdev.owner = THIS_MODULE;

	devno = MKDEV(cdevs->major, dbmdx_cdev_get_index(name));

	err = cdev_add(&cdev->cdev, devno, 1);
	if (err) {
		pr_err("failed to add cdev=%04x error: %d", devno, err);
		goto exit_cdev_add;
	}

	devno = cdev->cdev.dev;
	cdev->dev = device_create(cdevs->class, parent, devno, data, name);
	if (IS_ERR(cdev->dev)) {
		err = PTR_ERR(cdev->dev);
		pr_err("device_create cdev=%04x failed: %d\n", devno, err);
		goto exit_dev_create;
	}

	minor = MINOR(cdev->cdev.dev);
	set_bit(minor % LONG_BITS, &cdevs->indexes[minor / LONG_BITS]);

	return 0;

exit_dev_create:
	cdev_del(&cdev->cdev);
exit_cdev_add:
	return err;
}

struct dbmdx_cdev *dbmdx_cdev_from_inode(struct inode *inode)
{
	struct cdev *cdev = inode ? inode->i_cdev : NULL;

	return cdev ? container_of(cdev, struct dbmdx_cdev, cdev) : NULL;
}
EXPORT_SYMBOL(dbmdx_cdev_from_inode);

struct dbmdx_cdev *dbmdx_cdev_from_file(struct file *file)
{
	struct cdev *cdev = file ? file_inode(file)->i_cdev : NULL;

	return cdev ? container_of(cdev, struct dbmdx_cdev, cdev) : NULL;
}
EXPORT_SYMBOL(dbmdx_cdev_from_file);

int dbmdx_cdev_create(struct dbmdx_cdev *cdev, struct device *parent,
	const struct file_operations *fops, void *drvdata, char *fmt, ...)
{
	va_list args;
	char *name;
	int ret;

	if (!cdev || !fmt || !fops)
		return -EINVAL;

	va_start(args, fmt);
	name = kvasprintf(GFP_KERNEL, fmt, args);
	va_end(args);

	if (!name)
		return -ENOMEM;

	ret = dbmdx_cdev_create_name(cdev, name, parent, fops, drvdata);

	kfree(name);

	return ret;
}
EXPORT_SYMBOL(dbmdx_cdev_create);

int dbmdx_cdev_destroy(struct dbmdx_cdev *cdev)
{
	struct dbmdx_cdevs_ctx *cdevs = &dbmdx_cdevs;
	int minor;

	if (!cdev->dev || !cdev->cdev.dev)
		return -ENODEV;

	minor = MINOR(cdev->cdev.dev);
	clear_bit(minor % LONG_BITS, &cdevs->indexes[minor / LONG_BITS]);

	device_destroy(cdevs->class, cdev->cdev.dev);
	cdev_del(&cdev->cdev);

	memset(cdev, 0x00, sizeof(*cdev));

	return 0;
}
EXPORT_SYMBOL(dbmdx_cdev_destroy);

int dbmdx_cdev_init(void)
{
	struct dbmdx_cdevs_ctx *cdevs = &dbmdx_cdevs;
	static const char *cdev_name = "dbmdx";
	dev_t devno;
	int err;

	if (cdevs->class)
		return 0;

	err = alloc_chrdev_region(&devno, 0, DBMDX_CDEV_COUNT, cdev_name);
	if (err) {
		pr_err("unable to allocate char dev = %d", err);
		return err;
	}
	cdevs->major = MAJOR(devno);

	/* register device class */
	cdevs->class = class_create(THIS_MODULE, cdev_name);
	if (IS_ERR(cdevs->class)) {
		err = PTR_ERR(cdevs->class);
		cdevs->class = NULL;
		pr_err("unable to create %s class = %d\n", cdev_name, err);
		return err;
	}

	/* Permanent reserved for: AUDIO, LP LOG, HF0_LOG, HF1_LOG */
	set_bit(0, cdevs->indexes);
	set_bit(1, cdevs->indexes);
	set_bit(2, cdevs->indexes);
	set_bit(3, cdevs->indexes);

	return 0;
}
EXPORT_SYMBOL(dbmdx_cdev_init);

void dbmdx_cdev_exit(void)
{
	struct dbmdx_cdevs_ctx *cdevs = &dbmdx_cdevs;

	class_destroy(cdevs->class);
	unregister_chrdev_region(MKDEV(cdevs->major, 0), DBMDX_CDEV_COUNT);
}
EXPORT_SYMBOL(dbmdx_cdev_exit);

