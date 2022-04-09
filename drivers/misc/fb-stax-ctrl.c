/*
 * STAX interface driver DSPG, ADSP
 *
 * Copyright (c) 2021, Facebook Inc. All rights reserved.
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
/*
 * This driver exposes a common char dev node for Stax clients to
 * communicate with DSPs - DSPG, ADSP
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>

#include <uapi/sound/fb-stax-ctrl.h>
#include <sound/fb-stax-adapter-if.h>
#include <uapi/sound/fb-stax-ctrl-internal.h>
#include <uapi/sound/fb-stax-dspg.h>

struct stax_ctx {
	struct miscdevice mdev;
	struct stax_dspg_ctx *dspg_stax_ctx_ptr;
};

struct stax_file_ctx {
	struct stax_dspg_file_ctx *dspg_file_ctx_ptr;	/* for dspg */
};

static struct stax_ctx stax_core;

int stax_adapter_check(struct stax_adapter_ops *adap,
		struct device *dev)
{
	int rval;

	if (!adap) {
		dev_err(dev, "%s: Invalid adapter error\n", __func__);
		return -ENODEV;
	}

	rval =  adap->is_ready(adap) ? 0 : -EPERM;
	if (rval)
		dev_err(dev, "%s: adapter is not ready error:%d\n", __func__,
				rval);
	return rval;
}

int stax_blob_xfer(struct stax_adapter_ops *adap,
	struct stax_blob_buffer *blob, struct device *dev,
	bool snd_and_rcv)
{
	int size, rval = 0;
	void *buf;
	char *data;
	int app_type;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	if (!adap->is_active(adap)) {
		dev_err(dev, "%s: error adapter is not active\n", __func__);
		return -EPERM;
	}

	buf = blob->buf;
	size = blob->len;
	app_type = blob->app_type;
	data = (char *)buf;

	STAX_HEX_DUMP("stax: xfer: snd: ", buf, size);

	rval = adap->blob_xfer(adap, buf, size, 1, app_type, snd_and_rcv);

	if (snd_and_rcv)
		STAX_HEX_DUMP("stax: xfer: rcv: ", buf, size);

	return rval;
}

int stax_blob_put(struct stax_blob_buffer *blob,
		struct stax_ctrl_desc *value, struct device *dev)
{
	int max_payload_size = blob->size;
	int payload_len = value->len;

	if (payload_len > max_payload_size) {
		dev_err(dev, "%s:error: payload_len:%d > max_payload_size\n",
				__func__, payload_len);
		return -ENOSPC;
	}

	STAX_HEX_DUMP("stax: put: value: ", value,
			sizeof(*value) + payload_len);

	blob->len = payload_len;

	if (payload_len)
		memcpy(blob->buf, value->data, payload_len);

	STAX_HEX_DUMP("stax: put: blob: ", blob->buf, payload_len);

	return payload_len;
}

int stax_blob_get(struct stax_blob_buffer *blob,
		struct device *dev)
{
	struct stax_ctrl_desc *value = blob->value;
	int max_payload_size = blob->size;
	int payload_len = blob->len;

	if (payload_len > max_payload_size) {
		dev_err(dev, "%s:error: payload_len:%d > max_payload_size\n",
				__func__, payload_len);
		return -ENOBUFS;
	}

	STAX_HEX_DUMP("stax: get: blob:  ", blob->buf, payload_len);

	value->len = payload_len;

	if (payload_len)
		memcpy(value->data, blob->buf, payload_len);

	STAX_HEX_DUMP("stax: get: value: ",
			value, sizeof(*value) + payload_len);

	return payload_len;
}

static long stax_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct stax_file_ctx *private_data_ptr
		= (struct stax_file_ctx *)file->private_data;
	struct stax_dspg_file_ctx *fstax = private_data_ptr->dspg_file_ctx_ptr;
	struct device *dev = stax_core.mdev.this_device;
	void __user *data;
	int rval = -EINVAL;
	uint32_t device_id = 99;

	if (is_compat_task())
		data = compat_ptr(arg);
	else
		data = (void __user *)arg;

	if (copy_from_user(&device_id, data, sizeof(device_id))) {
		dev_err(dev, "%s:copy DEVICE ID from user FAILED\n",
			__func__);
		return -EFAULT;
	}

	switch (device_id) {
	case STAX_DEVICE_DSPG:
		rval = stax_dspg_ioctl(fstax, cmd, data);
		break;
	default:
		dev_err(dev, "%s: WRONG DEVICE ID = 0x%x\n",
			__func__, device_id);
		break;
	}

	return rval;
}

static int stax_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct stax_ctx *stax = dev_get_drvdata(mdev->this_device);
	struct device *dev = stax->mdev.this_device;
	struct stax_file_ctx *fstax;
	int rval;

	fstax = kcalloc(1, sizeof(*fstax), GFP_KERNEL);
	if (!fstax) {
		dev_err(dev, "%s: mem alloc failed!\n", __func__);
		return -ENOMEM;
	}

	fstax->dspg_file_ctx_ptr = stax_dspg_open(stax->dspg_stax_ctx_ptr);
	if (!fstax->dspg_file_ctx_ptr) {
		dev_err(dev, "%s: stax_open failed!\n", __func__);
		rval = -ENOMEM;
		goto open_fail;
	}

	file->private_data = fstax;

	return 0;

open_fail:
	kfree(fstax);

	return rval;
}

static int stax_release(struct inode *inode, struct file *file)
{
	struct stax_file_ctx *fstax = file->private_data;

	stax_dspg_release(fstax->dspg_file_ctx_ptr);

	kfree(fstax);
	return 0;
}

static const struct file_operations stax_fops = {
	.owner = THIS_MODULE,
	.open = stax_open,
	.release = stax_release,
	.unlocked_ioctl = stax_ioctl,
	.compat_ioctl = stax_ioctl,
};

static ssize_t stax_dspg_attr_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stax_ctx *stax = dev_get_drvdata(dev);
	struct stax_dspg_ctx *stax_dspg = stax->dspg_stax_ctx_ptr;
	struct stax_adapter_ops *adap = stax_dspg->adapter;
	uint32_t ver;
	int rval;

	rval = adap->get_version(adap, &ver);
	if (rval) {
		dev_err(dev, "%s: get verion failed:%d\n", __func__, rval);
		return rval;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ver);
}

static DEVICE_ATTR(version, S_IRUSR, stax_dspg_attr_version_show, NULL);

static const struct attribute *stax_attrs[] = {
	&dev_attr_version.attr,
	NULL,
};

static struct stax_ctx stax_core = {
	.mdev = {
		.name = "stax-ctrl",
		.minor = MISC_DYNAMIC_MINOR,
		.fops = &stax_fops,
	},
};

static int __init stax_core_init(void)
{
	struct stax_ctx *stax = &stax_core;
	struct device *dev;
	int rval;

	rval = misc_register(&stax->mdev);
	if (rval) {
		pr_err("Can't create STAX misc device! %s\n", stax->mdev.name);
		return rval;
	}

	dev = stax->mdev.this_device;
	stax->dspg_stax_ctx_ptr = stax_dspg_init(&stax->mdev);

	dev_set_drvdata(dev, stax);

	if (sysfs_create_files(&dev->kobj, stax_attrs))
		dev_err(dev, "STAX failed sysfs_create_files\n");

	return 0;
}

static void __exit stax_core_exit(void)
{
	struct stax_ctx *stax = &stax_core;

	misc_deregister(&stax->mdev);
}

module_init(stax_core_init);
module_exit(stax_core_exit);

MODULE_DESCRIPTION("Stax driver DSPG ADSP");
MODULE_AUTHOR("Facebook");
MODULE_LICENSE("GPL v2");
