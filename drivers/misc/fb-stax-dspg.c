/*
 * STAX DSPG interface driver
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

static struct stax_dspg_ctx stax_ctx_dspg;

static int stax_dspg_blob_clean(struct stax_blob_buffer *blob)
{
	blob->len = 0;
	blob->value->len = 0;
	return 0;
}

static int stax_dspg_blob_free(struct stax_blob_buffer *blob)
{
	if (blob->buf)
		vfree(blob->buf);

	if (blob->value)
		vfree(blob->value);

	blob->buf = NULL;
	blob->value = NULL;
	blob->size = 0;
	blob->len = 0;

	return 0;
}

static int stax_dspg_blob_alloc(struct stax_blob_buffer *blob,
	size_t blob_size, struct device *dev)
{
	stax_dspg_blob_free(blob);

	blob->buf = vmalloc(blob_size);
	if (!blob->buf) {
		dev_err(dev, "%s: blob buf mem alloc failed\n", __func__);
		return -ENOMEM;
	}

	blob->size = blob_size;

	blob->value = vmalloc(sizeof(blob->value) + blob_size);
	if (!blob->value) {
		dev_err(dev, "%s: blob val mem alloc failed\n", __func__);
		vfree(blob->buf);
		return -ENOMEM;
	}

	stax_dspg_blob_clean(blob);

	return 0;
}

static int stax_dspg_blob_from_user(struct stax_dspg_file_ctx *fstax,
	struct stax_ctrl_desc __user *usr_data, struct device *dev)
{
	struct stax_ctrl_desc *value = fstax->blob.value;

	if (!usr_data) {
		dev_err(dev, "%s:Error, Invalid mem address\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(value, usr_data, sizeof(*usr_data))) {
		dev_err(dev, "%s:copy from user failed\n", __func__);
		return -EFAULT;
	}

	if (value->len > fstax->blob.size) {
		dev_err(dev, "%s:Error, value->len:%d > fstax->blob.size\n",
				__func__, value->len);
		return -EINVAL;
	}

	if (value->len)
		if (copy_from_user(value->data, usr_data->data, value->len)) {
			dev_err(dev, "%s:copy from user failed!\n", __func__);
			return -EFAULT;
		}

	return 0;
}

static int stax_dspg_ioctl_value_set(struct stax_dspg_file_ctx *fstax,
	struct stax_ctrl_desc __user *data)
{
	struct stax_dspg_ctx *stax = fstax->stax;
	struct stax_blob_buffer blob = fstax->blob;
	struct stax_adapter_ops *adap = stax->adapter;
	struct device *dev = stax->mdev->this_device;
	int rval;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	rval  = stax_dspg_blob_from_user(fstax, data, dev);
	if (rval) {
		dev_err(dev, "%s: stax_dspg_blob_from_user failed\n", __func__);
		return rval;
	}

	rval = stax_blob_put(&blob, fstax->blob.value, dev);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_put failed\n", __func__);
		return rval;
	}

	/* The flag 'false' to send the blob */
	rval = stax_blob_xfer(adap, &blob, dev, false);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_xfer failed\n", __func__);
		return rval;
	}

	return 0;
}

static int stax_dspg_ioctl_value_get(struct stax_dspg_file_ctx *fstax,
	struct stax_ctrl_desc __user *data)
{
	struct stax_dspg_ctx *stax = fstax->stax;
	struct stax_blob_buffer blob = fstax->blob;
	struct stax_adapter_ops *adap = stax->adapter;
	struct device *dev = stax->mdev->this_device;
	int rval;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	rval = stax_dspg_blob_from_user(fstax, data, dev);
	if (rval) {
		dev_err(dev, "%s: stax_dspg_blob_from_use failed\n", __func__);
		return rval;
	}

	rval = stax_blob_put(&blob, fstax->blob.value, dev);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_put failed\n", __func__);
		return rval;
	}

	/* set the blob size as the expected response length */
	blob.len = fstax->blob.value->resp_len;

	/* The flag 'true' to receive the blob */
	rval = stax_blob_xfer(adap, &blob, dev, true);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_xfer failed\n", __func__);
		return rval;
	}

	rval = stax_blob_get(&blob, dev);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_get failed\n", __func__);
		return rval;
	}

	rval += sizeof(struct stax_ctrl_desc);
	if (copy_to_user(data, blob.value, rval)) {
		dev_err(dev, "%s: mem copy to user failed\n", __func__);
		return -EFAULT;
	}

	return rval < 0 ? rval : 0;
}

static int stax_dspg_ioctl_value_set_get(struct stax_dspg_file_ctx *fstax,
	struct stax_ctrl_desc __user *data)
{
	struct device *dev = fstax->stax->mdev->this_device;
	int rval;

	rval = stax_dspg_ioctl_value_set(fstax, data);
	if (rval < 0) {
		dev_err(dev, "%s: stax_dspg_ioctl_value_set failed\n",
			__func__);
		return rval;
	}

	rval = stax_dspg_ioctl_value_get(fstax, data);
	if (rval < 0) {
		dev_err(dev, "%s: stax_dspg_ioctl_value_get failed\n",
			__func__);
		return rval;
	}

	return rval;
}

long stax_dspg_ioctl(struct stax_dspg_file_ctx *fstax,
	unsigned int cmd, void __user *data)
{
	struct stax_dspg_ctx *stax = fstax->stax;
	int rval = -EINVAL;

	mutex_lock(&stax->lock);

	switch (cmd) {
	case STAX_IOCTL_SET_PARAM_BLOB:
		rval = stax_dspg_ioctl_value_set(fstax, data);
		break;
	case STAX_IOCTL_GET_PARAM_BLOB:
		rval = stax_dspg_ioctl_value_get(fstax, data);
		break;
	case STAX_IOCTL_SET_GET_CMD_BLOB:
		rval = stax_dspg_ioctl_value_set_get(fstax, data);
		break;
	}

	mutex_unlock(&stax->lock);

	return rval;
}

int stax_dspg_register(struct stax_adapter_ops *adap)
{
	struct stax_dspg_ctx *stax = &stax_ctx_dspg;
	struct device *dev = stax->mdev->this_device;
	int rval = -EINVAL;

	mutex_lock(&stax->lock);

	if (stax->adapter) {
		dev_err(dev, "%s:Stax dspg Adapter is already registered!\n",
				__func__);
		rval = -EBUSY;
		goto exit;
	}

	if (!adap->is_ready)
		dev_err(dev, "%s: Invalid stax dspg Adapter callback is_ready\n",
				__func__);
	else if (!adap->is_active)
		dev_err(dev, "%s: Invalid stax dspg Adapter callback is_active\n",
				__func__);
	else if (!adap->get_version)
		dev_err(dev, "%s: Invalid stax dspg Adapter callback get_version\n",
				__func__);
	else if (!adap->blob_size)
		dev_err(dev, "%s: Invalid stax dspg Adapter callback blob_size\n",
				__func__);
	else if (!adap->blob_xfer)
		dev_err(dev, "%s: Invalid stax dspg Adapter callback blob_xfer\n",
				__func__);
	else {
		stax->adapter = adap;
		dev_err(dev, "%s: Stax DSPG adapter registration success!\n",
			__func__);
		rval = 0;
	}

exit:
	mutex_unlock(&stax->lock);
	return rval;
}
EXPORT_SYMBOL(stax_dspg_register);

int stax_dspg_unregister(struct stax_adapter_ops *adap)
{
	struct stax_dspg_ctx *stax = &stax_ctx_dspg;
	struct device *dev = stax->mdev->this_device;
	int rval = 0;

	mutex_lock(&stax->lock);

	rval = (stax->adapter == adap) ? 0 : -EINVAL;
	if (rval)
		dev_err(dev, "%s: unregister failed, invalid handler\n",
			__func__);
	else
		stax->adapter = NULL;

	mutex_unlock(&stax->lock);

	return rval;
}
EXPORT_SYMBOL(stax_dspg_unregister);

struct stax_dspg_file_ctx *stax_dspg_open(struct stax_dspg_ctx *dspg_ctx)
{
	struct device *dev = dspg_ctx->mdev->this_device;
	struct stax_dspg_file_ctx *fstax_dspg;
	int rval;

	fstax_dspg = kcalloc(1, sizeof(*fstax_dspg), GFP_KERNEL);
	if (!fstax_dspg) {
		dev_err(dev, "%s: mem alloc failed!\n", __func__);
		return NULL;
	}

	fstax_dspg->stax = dspg_ctx;

	dspg_ctx->blob_size = STAX_DSPG_BLOB_MAX_SIZE;

	rval = stax_dspg_blob_alloc(&fstax_dspg->blob,
				dspg_ctx->blob_size, dev);
	if (rval) {
		dev_err(dev, "%s: stax_dspg_blob_alloc failed!\n",
			__func__);
		goto no_blob;
	}

	return fstax_dspg;

no_blob:
	kfree(fstax_dspg);

	return NULL;
}

int stax_dspg_release(struct stax_dspg_file_ctx *dspg_file_ctx)
{
	stax_dspg_blob_free(&dspg_file_ctx->blob);
	kfree(dspg_file_ctx);
	return 0;
}

struct stax_dspg_ctx *stax_dspg_init(struct miscdevice *mdev)
{
	mutex_init(&stax_ctx_dspg.lock);
	stax_ctx_dspg.mdev = mdev;
	return &stax_ctx_dspg;
}
