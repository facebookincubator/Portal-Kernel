/*
 * STAX ADSP interface driver
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

#include <sound/fb-stax-adapter-if.h>
#include <uapi/sound/fb-stax-adsp.h>

static struct stax_ctx stax_ctx_adsp;

static int stax_adsp_ioctl_render_stream_get_status(struct stax_file_ctx *fstax, struct		stax_ctrl_desc __user *data)
{
	struct stax_ctx *stax = fstax->stax;
	struct stax_blob_buffer blob = fstax->blob;
	struct stax_adapter_ops *adap = stax->adapter;
	struct device *dev = stax->mdev->this_device;
	int rval;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	rval  = stax_blob_from_user(fstax, data, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adsp_blob_from_user failed\n", __func__);
		return rval;
	}

	rval = stax_blob_put(&blob, fstax->blob.value, dev);
	if (rval < 0) {
		dev_err(dev, "%s: stax_blob_put failed\n", __func__);
		return rval;
	}

	blob.len = fstax->blob.value->resp_len;
	rval = stax_render_stream_get_status(adap, &blob, dev);

	return rval;
}

static int stax_adsp_ioctl_value_set(struct stax_file_ctx *fstax,
			struct stax_ctrl_desc __user *data)
{
	struct stax_ctx *stax = fstax->stax;
	struct stax_blob_buffer blob = fstax->blob;
	struct stax_adapter_ops *adap = stax->adapter;
	struct device *dev = stax->mdev->this_device;
	int rval;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	rval  = stax_blob_from_user(fstax, data, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adsp_blob_from_user failed\n", __func__);
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

static int stax_adsp_ioctl_value_get(struct stax_file_ctx *fstax,
		struct stax_ctrl_desc __user *data)
{
	struct stax_ctx *stax = fstax->stax;
	struct stax_blob_buffer blob = fstax->blob;
	struct stax_adapter_ops *adap = stax->adapter;
	struct device *dev = stax->mdev->this_device;
	int rval;

	rval = stax_adapter_check(adap, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adapter_check failed\n", __func__);
		return rval;
	}

	rval = stax_blob_from_user(fstax, data, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adsp_blob_from_use failed\n", __func__);
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

static int stax_adsp_ioctl_value_set_get(struct stax_file_ctx *fstax,
			struct stax_ctrl_desc __user *data)
{
	struct device *dev = fstax->stax->mdev->this_device;
	int rval;

	rval = stax_adsp_ioctl_value_set(fstax, data);
	if (rval < 0) {
		dev_err(dev, "%s: stax_adsp_ioctl_value_set failed\n", __func__);
		return rval;
	}

	rval = stax_adsp_ioctl_value_get(fstax, data);
	if (rval < 0) {
		dev_err(dev, "%s: stax_adsp_ioctl_value_get failed\n", __func__);
		return rval;
	}

	return rval;
}

long stax_adsp_ioctl(struct stax_file_ctx *fstax,
		unsigned int cmd, void __user *data)
{
	struct stax_ctx *stax = fstax->stax;
	int rval = -EINVAL;

	mutex_lock(&stax->lock);

	switch (cmd) {
	case STAX_IOCTL_SET_PARAM_BLOB:
		rval = stax_adsp_ioctl_value_set(fstax, data);
		break;
	case STAX_IOCTL_GET_PARAM_BLOB:
		rval = stax_adsp_ioctl_value_get(fstax, data);
    break;
	case STAX_IOCTL_SET_GET_CMD_BLOB:
		rval = stax_adsp_ioctl_value_set_get(fstax, data);
		break;
	case STAX_IOCTL_GET_RENDER_STREAM_STATUS:
		rval = stax_adsp_ioctl_render_stream_get_status(fstax, data);
		break;
	}

	mutex_unlock(&stax->lock);

	return rval;
}

int stax_adsp_register(struct stax_adapter_ops *adap)
{
	struct stax_ctx *stax = &stax_ctx_adsp;
	struct device *dev = stax->mdev->this_device;
	int rval = -EINVAL;

	mutex_lock(&stax->lock);

	if (stax->adapter) {
		dev_err(dev, "%s:Stax adsp Adapter is already registered!\n",
		__func__);
		rval = -EBUSY;
		goto exit;
	}

	if (!adap->is_ready)
		dev_err(dev, "%s: Invalid stax adsp Adapter callback is_ready\n",
		__func__);
	else if (!adap->is_active)
		dev_err(dev, "%s: Invalid stax adsp Adapter callback is_active\n",
		__func__);
	else if (!adap->get_version)
		dev_err(dev, "%s: Invalid stax adsp Adapter callback get_version\n",
		__func__);
	else if (!adap->blob_size)
		dev_err(dev, "%s: Invalid stax adsp Adapter callback blob_size\n",
		__func__);
	else if (!adap->blob_xfer)
		dev_err(dev, "%s: Invalid stax adsp Adapter callback blob_xfer\n",
		__func__);
	else {
		stax->adapter = adap;
		dev_err(dev, "%s: Stax ADSP adapter registration success!\n", __func__);
		rval = 0;
	}

exit:
	mutex_unlock(&stax->lock);
	return rval;
}
EXPORT_SYMBOL(stax_adsp_register);

int stax_adsp_unregister(struct stax_adapter_ops *adap)
{
	struct stax_ctx *stax = &stax_ctx_adsp;
	struct device *dev = stax->mdev->this_device;
	int rval = 0;

	mutex_lock(&stax->lock);

	rval = (stax->adapter == adap) ? 0 : -EINVAL;
	if (rval)
		dev_err(dev, "%s: unregister failed, invalid handler\n", __func__);
	else
		stax->adapter = NULL;

	mutex_unlock(&stax->lock);

	return rval;
}
EXPORT_SYMBOL(stax_adsp_unregister);

struct stax_file_ctx *stax_adsp_open(struct stax_ctx *adsp_ctx)
{
	struct device *dev = adsp_ctx->mdev->this_device;
	struct stax_file_ctx *fstax_adsp;
	struct stax_adapter_ops *adap;
	int rval;

	mutex_lock(&adsp_ctx->lock);

	adap = adsp_ctx->adapter;

	/* Exit if adapter is not registered yet */
	if (!adap) {
		mutex_unlock(&adsp_ctx->lock);
		return ERR_PTR(-ENODEV);
	}

	__module_get(adap->module);
	get_device(adap->dev);

	mutex_unlock(&adsp_ctx->lock);

	fstax_adsp = kcalloc(1, sizeof(*fstax_adsp), GFP_KERNEL);
	if (!fstax_adsp) {
		dev_err(dev, "%s: mem alloc failed!\n", __func__);
		rval = -ENOMEM;
		goto err_no_mem;
	}

	fstax_adsp->stax = adsp_ctx;

	adsp_ctx->blob_size = STAX_ADSP_BLOB_MAX_SIZE;

	rval = stax_blob_alloc(&fstax_adsp->blob, adsp_ctx->blob_size, dev);
	if (rval) {
		dev_err(dev, "%s: stax_adsp_blob_alloc failed!\n", __func__);
		goto no_blob;
	}

	return fstax_adsp;

no_blob:
	kfree(fstax_adsp);

err_no_mem:
	put_device(adap->dev);
	module_put(adap->module);

	return ERR_PTR(rval);
}

int stax_adsp_release(struct stax_file_ctx *adsp_file_ctx)
{
	struct stax_adapter_ops *adap = adsp_file_ctx->stax->adapter;

	put_device(adap->dev);
	module_put(adap->module);

	stax_blob_free(&adsp_file_ctx->blob);
	kfree(adsp_file_ctx);
	return 0;
}

struct stax_ctx *stax_adsp_init(struct miscdevice *mdev)
{
	mutex_init(&stax_ctx_adsp.lock);
	stax_ctx_adsp.mdev = mdev;
	return &stax_ctx_adsp;
}
