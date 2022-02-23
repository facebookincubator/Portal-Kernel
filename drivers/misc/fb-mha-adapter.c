/*
 * DSPG DBMDX MH Acousting adapter misc device
 *
 * Copyright (c) 2019, Facebook Inc. All rights reserved.
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
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>

#include <sound/fb-mha-adapter-if.h>
#include <uapi/sound/fb-mha-adapter.h>

#define MHADAP_TIMEOUT_MS		1000

/* Shows that adapter instance is read for usege. Blob buffer is allocatged. */
#define MHADAP_FLG_READY		0
/* User-space adapter is is connected and algorithm is in enabled */
#define MHADAP_FLG_ENABLED		1
/* Until this flag is set it means there is a command waiting to be executed
 * by user-space. This is identified with POLLMSG. Command must be read
 * with ioctl MHADAP_IOCTL_CMD_GET then executed and mark as complete with
 * ioctl MHADAP_IOCTL_CMD_COMPLETE. If this cycle is not performed for
 * MHADAP_TIMEOUT_MS, this flag is automatically cleared and -ETIMEOUT
 * is returned.
 */
#define MHADAP_FLG_COMMAND		2
/* Indicates whether the last command has been successfully executed. */
#define MHADAP_FLG_CMD_SUCCESS		3
/* Indicates that driver is already probed. A single instance can be probed. */
#define MHADAP_FLG_PROBED		4
/* Permanent skip loading firmware tuning parameters */
#define MHADAP_FLG_SKIP_FW_TUNING	5

struct mh_adapter_xfer {
	size_t size;			/* Blob size */
	size_t used;			/* Blob used size */
	loff_t pos_r;			/* Read position */
	loff_t pos_w;			/* Write position */
	void *blob;
};

struct mh_adapter_param {
	struct list_head item;
	struct mha_parameter p;
};

struct mh_adapter_value {
	struct list_head item;
	struct mha_value v;
};

struct mh_adapter_ctx {
	struct mha_adapter_ops adap;	/* Addapter ops callback */
	struct platform_device *pdev;	/* Platform device structure */
	struct miscdevice mdev;		/* Registered misc device structure */
	struct mh_adapter_xfer xfer;	/* Blob buffer used for commands */
	struct completion compl;	/* Command completition event */
	struct mutex lock;		/* Lock all I/O operations */
	struct list_head params;	/* FW parameters */
	struct mh_adapter_param *ep;	/* Current enumerate parameters */
	enum mh_adapter_commands cmd;	/* Current command for performing */
	wait_queue_head_t wake;		/* Wakeup of file ops poll callback */
	unsigned long flags;		/* Bit field mask of internal flags */
	uint32_t version;		/* Algorithm version */
	const char *fw_name;		/* Firmware name */
};

static struct mh_adapter_ctx mha_adapter;

static int mhadap_fw_free(struct mh_adapter_ctx *mhadap)
{
	struct mh_adapter_param *p, *ptmp;

	mutex_lock(&mhadap->lock);

	list_for_each_entry_safe(p, ptmp, &mhadap->params, item) {
		list_del(&p->item);
		kfree(p);
	}

	mutex_unlock(&mhadap->lock);

	return 0;
}

static inline int mhadap_payload_size(struct mha_parameter *param)
{
	return mha_payload_size(param->type, param->count);
}

static int mhadap_command(struct mh_adapter_ctx *mhadap,
	enum mh_adapter_commands cmd)
{
	int rval, to;

	if (!test_bit(MHADAP_FLG_ENABLED, &mhadap->flags))
		return -EPERM;

	if (test_bit(MHADAP_FLG_COMMAND, &mhadap->flags))
		return -EBUSY;

	mutex_lock(&mhadap->lock);

	mhadap->cmd = cmd;
	clear_bit(MHADAP_FLG_CMD_SUCCESS, &mhadap->flags);
	set_bit(MHADAP_FLG_COMMAND, &mhadap->flags);
	init_completion(&mhadap->compl);
	wake_up_interruptible_all(&mhadap->wake);

	to = msecs_to_jiffies(MHADAP_TIMEOUT_MS);
	rval = wait_for_completion_interruptible_timeout(&mhadap->compl, to);
	if (!rval) {
		clear_bit(MHADAP_FLG_COMMAND, &mhadap->flags);
		rval = -ETIMEDOUT;
	} else if (rval < 0)
		clear_bit(MHADAP_FLG_COMMAND, &mhadap->flags);
	else
		rval = 0;

	mutex_unlock(&mhadap->lock);

	return rval;
}

static bool mhadap_is_ready(struct mha_adapter_ops *adap)
{
	struct mh_adapter_ctx *mhadap = adap->private;

	return !!test_bit(MHADAP_FLG_READY, &mhadap->flags);
}

static bool mhadap_is_active(struct mha_adapter_ops *adap)
{
	struct mh_adapter_ctx *mhadap = adap->private;

	return !!test_bit(MHADAP_FLG_ENABLED, &mhadap->flags);
}

static int mhadap_get_version(struct mha_adapter_ops *adap, uint32_t *ver)
{
	struct mh_adapter_ctx *mhadap = adap->private;

	if (!ver)
		return -EINVAL;

	*ver = mhadap->version;

	return 0;
}

static int mhadap_blob_size(struct mha_adapter_ops *adap)
{
	struct mh_adapter_ctx *mhadap = adap->private;
	struct mh_adapter_xfer *xfer = &mhadap->xfer;

	return xfer->size;
}

static int mhadap_blob_xfer(struct mha_adapter_ops *adap, void *blob,
	size_t size, int count, bool snd_and_rcv)
{
	struct mh_adapter_ctx *mhadap = adap->private;
	struct mh_adapter_xfer *xfer = &mhadap->xfer;
	int cmd, rval;

	if (size > xfer->size)
		return -EINVAL;

	xfer->blob = blob;
	xfer->used = size;
	xfer->pos_r = 0;
	xfer->pos_w = 0;

	cmd = snd_and_rcv ? MHADAP_CMD_LOAD : MHADAP_CMD_STORE;
	rval = mhadap_command(mhadap, cmd);
	if (rval)
		return rval;

	xfer->blob = NULL;
	xfer->used = 0;
	xfer->pos_r = 0;
	xfer->pos_w = 0;

	return size;
}

static int mhadap_fw_start(struct mha_adapter_ops *adap, int params, int values)
{
	struct mh_adapter_ctx *mhadap = adap->private;

	return mhadap_fw_free(mhadap);
}

static int mhadap_fw_param(struct mha_adapter_ops *adap,
	struct mha_parameter *p)
{
	struct mh_adapter_ctx *mhadap = adap->private;
	struct mh_adapter_param *param;

	param = kmalloc(sizeof(*param), GFP_KERNEL);
	if (!param)
		return -ENOMEM;

	memcpy(&param->p, p, sizeof(*p));
	INIT_LIST_HEAD(&param->item);

	mutex_lock(&mhadap->lock);
	list_add_tail(&param->item, &mhadap->params);
	mutex_unlock(&mhadap->lock);

	return 0;
}

static int mhadap_fw_finish(struct mha_adapter_ops *adap)
{
	struct mh_adapter_ctx *mhadap = adap->private;

	return mhadap_command(mhadap, MHADAP_CMD_FIRMWARE);
}

static int mhadap_fw_tuning(struct mha_adapter_ops *adap, bool enable)
{
	struct mh_adapter_ctx *mhadap = adap->private;
	enum mh_adapter_commands cmd;

	if (test_bit(MHADAP_FLG_SKIP_FW_TUNING, &mhadap->flags))
		return -EPERM;

	cmd = enable ? MHADAP_CMD_FW_TUNING_START : MHADAP_CMD_FW_TUNING_STOP;

	return mhadap_command(mhadap, cmd);
}

static int mhadap_ioctl_init_blob(struct mh_adapter_ctx *mhadap,
	uint32_t __user *data)
{
	struct mh_adapter_xfer *xfer = &mhadap->xfer;
	uint32_t size;

	if (test_bit(MHADAP_FLG_READY, &mhadap->flags))
		return -EBUSY;

	if (get_user(size, data))
		return -EFAULT;

	xfer->size = size;

	set_bit(MHADAP_FLG_READY, &mhadap->flags);

	return 0;
}

static int mhadap_ioctl_set_version(struct mh_adapter_ctx *mhadap,
	uint __user *data)
{
	uint32_t ver;

	if (get_user(ver, data))
		return -EFAULT;

	mhadap->version = ver;

	return 0;
}

static int mhadap_ioctl_enable(struct mh_adapter_ctx *mhadap,
	bool __user *data)
{
	struct mha_adapter_ops *adap = &mhadap->adap;
	bool enable;

	if (get_user(enable, data))
		return -EFAULT;

	if (enable)
		set_bit(MHADAP_FLG_ENABLED, &mhadap->flags);
	else
		clear_bit(MHADAP_FLG_ENABLED, &mhadap->flags);

	return mha_notify(adap, enable ? MHA_NOTIFY_START : MHA_NOTIFY_STOP);
}

static int mhadap_ioctl_command(struct mh_adapter_ctx *mhadap,
	enum mh_adapter_commands __user *data, bool non_block)
{
	if (!test_bit(MHADAP_FLG_COMMAND, &mhadap->flags)) {
		if (non_block)
			return -EAGAIN;

		wait_event_interruptible(mhadap->wake,
			test_bit(MHADAP_FLG_COMMAND, &mhadap->flags));

		if (signal_pending(current))
			return -ERESTARTSYS;

		if (!test_bit(MHADAP_FLG_COMMAND, &mhadap->flags))
			return -EAGAIN;
	}

	if (put_user(mhadap->cmd, data))
		return -EFAULT;

	return 0;
}

static int mhadap_ioctl_complete(struct mh_adapter_ctx *mhadap,
	bool __user *data)
{
	bool success = true;

	if (!test_and_clear_bit(MHADAP_FLG_COMMAND, &mhadap->flags))
		return -ENOENT;

	if (data && get_user(success, data))
		return -EFAULT;

	if (success)
		set_bit(MHADAP_FLG_CMD_SUCCESS, &mhadap->flags);

	complete_all(&mhadap->compl);

	return 0;
}

static int mhadap_ioctl_count_params(struct mh_adapter_ctx *mhadap,
	int __user *data)
{
	struct mh_adapter_param *p;
	int count = 0;

	if (!data)
		return -EINVAL;

	list_for_each_entry(p, &mhadap->params, item)
		count++;

	return put_user(count, data);
}

static int mhadap_ioctl_enum_params(struct mh_adapter_ctx *mhadap,
	struct mha_parameter __user *data)
{
	struct mh_adapter_param *param = mhadap->ep;
	struct list_head *list = &mhadap->params;
	int payload_size;

	/* Check for NULL as argument */
	if (!data) {
		/* Restarting enumeration at the beginning */
		mhadap->ep = NULL;
		return 0;
	}

	if (!param)
		param = list_first_entry_or_null(list, typeof(*param), item);
	else if (!list_is_last(&param->item, list))
		param = list_next_entry(param, item);
	else
		param = NULL;

	if (!param)
		return -ENODATA;

	mhadap->ep = param;
	payload_size = mhadap_payload_size(&param->p);
	if (payload_size < 0)
		return payload_size;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return payload_size;
}

static long mhadap_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct mh_adapter_ctx *mhadap = file->private_data;
	struct device *dev = mhadap->mdev.this_device;
	bool non_block = !!(file->f_flags & O_NONBLOCK);
	void __user *data;

	if (is_compat_task())
		data = compat_ptr(arg);
	else
		data = (void __user *)arg;

	switch (cmd) {
	case MHADAP_IOCTL_INIT_BLOB:
		return mhadap_ioctl_init_blob(mhadap, data);
	case MHADAP_IOCTL_SET_VERSION:
		return mhadap_ioctl_set_version(mhadap, data);
	case MHADAP_IOCTL_ENABLE:
		return mhadap_ioctl_enable(mhadap, data);
	case MHADAP_IOCTL_CMD_GET:
		return mhadap_ioctl_command(mhadap, data, non_block);
	case MHADAP_IOCTL_CMD_COMPLETE:
		return mhadap_ioctl_complete(mhadap, data);
	case MHADAP_IOCTL_COUNT_PARAMS:
		return mhadap_ioctl_count_params(mhadap, data);
	case MHADAP_IOCTL_ENUM_PARAMS:
		return mhadap_ioctl_enum_params(mhadap, data);
	}

	dev_err(dev, "Invalid ioctl %08X\n", cmd);

	return -EINVAL;
}

static ssize_t mhadap_read(struct file *f,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	struct mh_adapter_ctx *mhadap = f->private_data;
	struct mh_adapter_xfer *xfer = &mhadap->xfer;
	void *blob = xfer->blob;
	size_t avail = xfer->used;
	ssize_t rval;

	if (!blob || !avail)
		return -EPERM;

	*ppos = xfer->pos_r;
	rval = simple_read_from_buffer(ubuf, count, ppos, blob, avail);
	xfer->pos_r = *ppos;

	return rval;
}

static ssize_t mhadap_write(struct file *f,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct mh_adapter_ctx *mhadap = f->private_data;
	struct mh_adapter_xfer *xfer = &mhadap->xfer;
	void *blob = xfer->blob;
	size_t avail = xfer->used;
	ssize_t rval;

	if (!blob || !avail)
		return -EPERM;

	*ppos = xfer->pos_w;
	rval = simple_write_to_buffer(blob, avail, ppos, ubuf, count);
	xfer->pos_w = *ppos;

	return rval;
}

static unsigned int mhadap_poll(struct file *f, struct poll_table_struct *wait)
{
	struct mh_adapter_ctx *adap = f->private_data;
	struct mh_adapter_xfer *xfer = &adap->xfer;
	unsigned int mask = 0;

	poll_wait(f, &adap->wake, wait);

	if (test_bit(MHADAP_FLG_COMMAND, &adap->flags))
		mask |= POLLMSG;

	if (xfer->used > xfer->pos_r)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/*
 * This misc file could be open only once, becasue function mha_register()
 * allows a single MH adapter registration.
 */
static int mhadap_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct mh_adapter_ctx *mhadap;
	struct mh_adapter_xfer *xfer;

	mhadap = container_of(mdev, struct mh_adapter_ctx, mdev);
	mhadap->adap.dev = &mhadap->pdev->dev;

	file->private_data = mhadap;
	mhadap->adap.private = mhadap;
	mhadap->flags = 0;
	mhadap->version = 0;
	xfer = &mhadap->xfer;
	xfer->used = 0;
	xfer->pos_r = 0;
	xfer->pos_w = 0;

	init_completion(&mhadap->compl);
	init_waitqueue_head(&mhadap->wake);

	return 0;
}

static int mhadap_release(struct inode *inode, struct file *file)
{
	struct mh_adapter_ctx *mhadap = file->private_data;

	if (test_and_clear_bit(MHADAP_FLG_ENABLED, &mhadap->flags))
		mha_notify(&mhadap->adap, MHA_NOTIFY_CRASH);

	clear_bit(MHADAP_FLG_READY, &mhadap->flags);

	/* Blocks file closing until complete and last I/O operation if exist */
	mutex_lock(&mhadap->lock);
	mutex_unlock(&mhadap->lock);

	mhadap_fw_free(mhadap);

	mhadap->flags = 0;

	return 0;
}

static const struct file_operations mhadap_fops = {
	.owner = THIS_MODULE,
	.open = mhadap_open,
	.release = mhadap_release,
	.unlocked_ioctl = mhadap_ioctl,
	.compat_ioctl = mhadap_ioctl,
	.read = mhadap_read,
	.write = mhadap_write,
	.poll =  mhadap_poll,
};

static struct mh_adapter_ctx mha_adapter = {
	.mdev = {
		.name = "mha-adapter",
		.minor = MISC_DYNAMIC_MINOR,
		.fops = &mhadap_fops,
		.parent = NULL,
	},
	.adap = {
		.is_ready = mhadap_is_ready,
		.is_active = mhadap_is_active,
		.get_version = mhadap_get_version,
		.blob_size = mhadap_blob_size,
		.blob_xfer = mhadap_blob_xfer,
		.fw_start = mhadap_fw_start,
		.fw_param = mhadap_fw_param,
		.fw_finish = mhadap_fw_finish,
		.fw_tuning = mhadap_fw_tuning,
	},
};

static void mh_adapter_firmware(const struct firmware *fw, void *context)
{
	struct mh_adapter_ctx *mhadap = context;
	struct device *dev = &mhadap->pdev->dev;

	if (!fw) {
		dev_err(dev, "FW: Invalid firmware!\n");
		return;
	}

	dev_info(dev, "MHA FW: name %s ; size %zd ; data %p\n",
			mhadap->fw_name, fw->size, fw->data);

	mha_firmware(dev, &mhadap->adap, (void *)fw->data, fw->size);

	release_firmware(fw);
}

static int mh_adapter_dt_parser(struct device *dev,
	struct mh_adapter_ctx *mha)
{
	struct device_node *np = dev->of_node;
	int rval;

	rval = of_property_read_string(np, "fb,firmware-name", &mha->fw_name);
	if (rval)
		mha->fw_name = NULL;

	if (of_property_read_bool(np, "fb,skip-fw-tuning"))
		set_bit(MHADAP_FLG_SKIP_FW_TUNING, &mha->flags);

	return 0;
}

static int mh_adapter_probe(struct platform_device *pdev)
{
	struct mh_adapter_ctx *mhadap = &mha_adapter;
	struct device *dev = &pdev->dev;
	int rval;

	if (test_and_set_bit(MHADAP_FLG_PROBED, &mhadap->flags))
		return -EBUSY;

	mhadap->pdev = pdev;
	dev_set_drvdata(dev, mhadap);

	mutex_init(&mhadap->lock);

	INIT_LIST_HEAD(&mhadap->params);

	rval = mh_adapter_dt_parser(dev, mhadap);
	if (rval)
		goto exit;

	mhadap->mdev.parent = dev;
	rval = misc_register(&mhadap->mdev);
	if (rval) {
		pr_err("Can't create MH Acoustics adapter misc!\n");
		goto exit;
	}

	rval = mha_register(&mhadap->adap);
	if (rval)
		 goto exit_no_adapter;

	if (mhadap->fw_name) {
		rval = request_firmware_nowait(THIS_MODULE, true,
			mhadap->fw_name, dev, GFP_KERNEL, mhadap,
			mh_adapter_firmware);
		if (rval) {
			dev_info(dev, "MHA FW: Request failed: %d\n", rval);
			goto exit;
		}

		dev_info(dev, "MHA FW: Successfully requested: %s\n",
			mhadap->fw_name);
	}

	return 0;

exit_no_adapter:
	misc_deregister(&mhadap->mdev);

exit:
	clear_bit(MHADAP_FLG_PROBED, &mhadap->flags);

	return rval;
}

static int mh_adapter_remove(struct platform_device * pdev)
{
	struct mh_adapter_ctx *mhadap = &mha_adapter;

	mha_unregister(&mhadap->adap);
	misc_deregister(&mhadap->mdev);

	clear_bit(MHADAP_FLG_PROBED, &mhadap->flags);

	return 0;
}

static const struct of_device_id mh_adapter_of_match[] = {
	{ .compatible = "fb,mha-adapter", },
	{},
};

static struct platform_driver mh_adapter_driver = {
	.driver = {
		.name = "fb,mha-adapter",
		.of_match_table = mh_adapter_of_match,
		.owner = THIS_MODULE,
	},
	.probe = mh_adapter_probe,
	.remove = mh_adapter_remove,
};

module_platform_driver(mh_adapter_driver);

MODULE_DESCRIPTION("Misc MH Acoustics adapter driver");
MODULE_AUTHOR("Dinko Mironov <dmironov@fb.com>");
MODULE_LICENSE("GPL v2");
