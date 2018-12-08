/*
 * This file is the kernel driver for presence detection using camera.
 *
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <uapi/misc/pd_camera.h>

/* Wake lock timeout */
#define PD_WAKE_LOCK_TIMEOUT_MS	200

/* Timeout value for presence detection */
#define PD_TIMEOUT_IN_SECS	12

#define PD_DEFAULT_DRIVER_MODE	PD_DRIVER_MODE_IRQ

#define PD_CAM_FLG_SESSION_LOCK	0

/* State should only be modified in pd_camera_update_state function */
struct pd_camera_state {
	u32	detect_enable;
	u32	detect_value;
	enum pd_driver_mode driver_mode;
	u32	camera_in_pd_mode;
	u32	actual_detect_enable;
	bool	irq_enabled;
};

struct pd_camera_ctx {
	struct miscdevice misc_dev;
	unsigned long flags;
	struct  platform_device *pdev;
	struct  mutex mutex;
	struct	wake_lock wlock;
	struct  completion enable_changed_compl;
	wait_queue_head_t enable_change_wait_queue;
	int	irq;
	enum	of_gpio_flags irq_active;
	struct	workqueue_struct *pd_camera_wq;
	struct	delayed_work timeout_work;
	struct pd_camera_state state;
	u32	last_user_read_detect_enable;
	u32	pd_events_since_last_read;
};

static int pd_camera_irq_request(struct pd_camera_ctx *ctx, bool enable);

/* Expected to be called with lock held
 */
static int pd_camera_update_state(struct pd_camera_ctx *ctx,
		enum pd_driver_mode driver_mode, u32 detect_enable,
		u32 detect_value, u32 camera_in_pd_mode)
{
	bool mode_changed = (ctx->state.driver_mode != driver_mode);
	bool value_changed = (ctx->state.detect_value != detect_value);
	bool enable_changed = (ctx->state.detect_enable != detect_enable);
	u32 actual_detect_enable = detect_enable & camera_in_pd_mode;
	bool actual_enable_changed = (ctx->state.actual_detect_enable !=
					actual_detect_enable);
	int ret = 0;

	if (!mode_changed && !value_changed && !enable_changed &&
			!actual_enable_changed) {
		return 0;
	}

	if ((driver_mode == PD_DRIVER_MODE_IRQ) && value_changed) {
		pr_err("%s: not allowed\n", __func__);
		return -EPERM;
	}

	if (mode_changed || actual_enable_changed) {
		/* Reset detect value */
		detect_value = 0;
		value_changed = (ctx->state.detect_value != detect_value);
	}

	if ((driver_mode == PD_DRIVER_MODE_IRQ) && actual_detect_enable) {
		if (!ctx->state.irq_enabled) {
			ret = pd_camera_irq_request(ctx, true);
			if (ret)
				return ret;
			ctx->state.irq_enabled = true;
		}
	} else {
		if (ctx->state.irq_enabled) {
			ret = pd_camera_irq_request(ctx, false);
			if (ret)
				return ret;
			ctx->state.irq_enabled = false;
		}
	}


	if (value_changed && detect_value) {
		ctx->pd_events_since_last_read++;
		/* Create/modify delayed work to timeout presence detection */
		mod_delayed_work(ctx->pd_camera_wq, &ctx->timeout_work,
			msecs_to_jiffies(PD_TIMEOUT_IN_SECS*MSEC_PER_SEC));
	}

	ctx->state.driver_mode = driver_mode;
	ctx->state.detect_enable = detect_enable;
	ctx->state.detect_value = detect_value;
	ctx->state.camera_in_pd_mode = camera_in_pd_mode;
	ctx->state.actual_detect_enable = actual_detect_enable;

	/* Do notifications if necessary */
	if (value_changed && detect_enable)
		sysfs_notify(&ctx->pdev->dev.kobj, NULL, "detect_value");

	if (enable_changed && test_bit(PD_CAM_FLG_SESSION_LOCK, &ctx->flags)) {
		complete(&ctx->enable_changed_compl);
		wake_up(&ctx->enable_change_wait_queue);
	}

	return 0;
}

static void pd_camera_timeout_work(struct work_struct *work)
{
	struct pd_camera_ctx *ctx =
		container_of(work, struct pd_camera_ctx, timeout_work.work);
	pr_debug("%s: timeout\n", __func__);
	mutex_lock(&ctx->mutex);
	pd_camera_update_state(ctx, ctx->state.driver_mode,
			ctx->state.detect_enable,
			0, ctx->state.camera_in_pd_mode);
	mutex_unlock(&ctx->mutex);
}

/* This is threaded irq handler */
static irqreturn_t pd_camera_thread_irq(int irq, void *data)
{
	struct pd_camera_ctx *ctx = data;

	mutex_lock(&ctx->mutex);
	pd_camera_update_state(ctx, ctx->state.driver_mode,
			ctx->state.detect_enable,
			1, ctx->state.camera_in_pd_mode);
	mutex_unlock(&ctx->mutex);
	/*
	 * Wake up the system and prevent it from entering suspend
	 * again for a short while.
	 */
	wake_lock_timeout(&ctx->wlock,
			  msecs_to_jiffies(PD_WAKE_LOCK_TIMEOUT_MS));

	return IRQ_HANDLED;
}

static int pd_camera_irq_request(struct pd_camera_ctx *ctx, bool enable)
{
	int ret;
	int isp_irq = gpio_to_irq(ctx->irq);

	if (!isp_irq) {
		dev_err(&ctx->pdev->dev, "%s: Unable to get irq for gpio=%d",
			__func__, ctx->irq);
		return -EFAULT;
	}
	if (enable) {
		int edge = ctx->irq_active;
		uint32_t flags = IRQF_SHARED | IRQF_ONESHOT;

		flags |= edge ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
		ret = request_threaded_irq(isp_irq, NULL,
				pd_camera_thread_irq,
				flags, "pd_camera", ctx);

		if (ret) {
			dev_err(&ctx->pdev->dev,
				"%s: Unable to claim irq %d, err = %d\n",
					__func__, isp_irq, ret);
			return -EFAULT;
		}
		dev_dbg(&ctx->pdev->dev, "isp_irq = %d\n", isp_irq);
	} else {
		free_irq(isp_irq, ctx);
		dev_dbg(&ctx->pdev->dev, "freed isp_irq = %d\n", isp_irq);
	}
	return 0;
}

static ssize_t pd_camera_detect_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n", ctx->state.detect_enable);
	return ret;
}

static ssize_t pd_camera_detect_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct pd_camera_ctx *ctx = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 0, &value);
	if (ret)
		return ret;

	if (value & ~1U) /* Allow only 0 or 1*/
		return -EINVAL;

	mutex_lock(&ctx->mutex);
	ret = pd_camera_update_state(ctx, ctx->state.driver_mode, value,
			ctx->state.detect_value, ctx->state.camera_in_pd_mode);
	mutex_unlock(&ctx->mutex);

	if (ret)
		return ret;
	else
		return len;
}

static ssize_t pd_camera_detect_value_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		       (ctx->state.detect_value & ctx->state.detect_enable));

	return ret;

}

static ssize_t pd_camera_driver_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		       ctx->state.driver_mode);

	return ret;

}

static ssize_t pd_camera_in_pd_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			ctx->state.camera_in_pd_mode);

	return ret;
}

static ssize_t pd_camera_actual_detect_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			ctx->state.actual_detect_enable);

	return ret;
}


static ssize_t pd_camera_irq_enabled_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			ctx->state.irq_enabled);

	return ret;
}

static ssize_t pd_camera_events_since_last_read_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pd_camera_ctx *ctx =  dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&ctx->mutex);
	ret = snprintf(buf, PAGE_SIZE, "%d\n",
			ctx->pd_events_since_last_read);
	ctx->pd_events_since_last_read = 0;
	mutex_unlock(&ctx->mutex);

	return ret;
}

static DEVICE_ATTR(detect_enable, S_IRUGO | S_IWUSR,
			pd_camera_detect_enable_show,
			pd_camera_detect_enable_store);
static DEVICE_ATTR(detect_value, S_IRUGO,
			pd_camera_detect_value_show,
			NULL);
static DEVICE_ATTR(driver_mode, S_IRUGO,
			pd_camera_driver_mode_show,
			NULL);
static DEVICE_ATTR(camera_in_pd_mode, S_IRUGO | S_IWUSR,
			pd_camera_in_pd_mode_show,
			NULL);
static DEVICE_ATTR(actual_detect_enable, S_IRUGO,
			pd_camera_actual_detect_enable_show, NULL);
static DEVICE_ATTR(irq_enabled, S_IRUGO,
			pd_camera_irq_enabled_show, NULL);
static DEVICE_ATTR(pd_events_since_last_read, S_IRUGO,
			pd_camera_events_since_last_read_show, NULL);


static struct attribute *sysfs_attrs[] = {
	&dev_attr_detect_enable.attr,
	&dev_attr_detect_value.attr,
	&dev_attr_driver_mode.attr,
	&dev_attr_camera_in_pd_mode.attr,
	&dev_attr_actual_detect_enable.attr,
	&dev_attr_irq_enabled.attr,
	&dev_attr_pd_events_since_last_read.attr,
	NULL,
};

static struct attribute_group pd_camera_attribute_group = {
	.attrs = sysfs_attrs
};


static int pd_camera_open(struct inode *inode, struct file *filp)
{
	struct pd_camera_ctx *ctx = container_of(filp->private_data,
				struct pd_camera_ctx, misc_dev);

	pr_debug("%s\n", __func__);
	if (test_and_set_bit(PD_CAM_FLG_SESSION_LOCK, &ctx->flags))
		return -EBUSY;
	filp->private_data = ctx;

	mutex_lock(&ctx->mutex);
	ctx->last_user_read_detect_enable = 0;
	mutex_unlock(&ctx->mutex);

	return 0;
}

static int pd_camera_release(struct inode *inode, struct file *filp)
{
	struct pd_camera_ctx *ctx = filp->private_data;

	pr_debug("%s\n", __func__);
	clear_bit(PD_CAM_FLG_SESSION_LOCK, &ctx->flags);

	return 0;
}

long pd_camera_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret = 0;
	struct pd_camera_ctx *ctx = file->private_data;
	void __user *argp = (void __user *) arg;
	enum pd_driver_mode driver_mode;
	int cam_pd_mode;
	struct pd_get_enable_msg msg;

	pr_debug("%s: cmd=%d\n", __func__, cmd);

	switch (cmd) {
	case PD_CAM_SET_DRIVER_MODE:
		if (copy_from_user(&driver_mode, argp, sizeof(driver_mode)))
			return -EFAULT;
		if ((driver_mode != PD_DRIVER_MODE_IRQ) &&
				(driver_mode != PD_DRIVER_MODE_MANUAL))
			return -EINVAL;
		pr_debug("%s: detect mode=%d\n", __func__, driver_mode);
		mutex_lock(&ctx->mutex);
		ret = pd_camera_update_state(ctx, driver_mode,
			ctx->state.detect_enable,
			ctx->state.detect_value, ctx->state.camera_in_pd_mode);
		mutex_unlock(&ctx->mutex);
		return ret;
	case PD_CAM_SET_CAM_PD_MODE:
		if (copy_from_user(&cam_pd_mode, argp, sizeof(cam_pd_mode)))
			return -EFAULT;
		if (cam_pd_mode & ~1U) /* Allow only 0 or 1*/
			return -EINVAL;
		pr_debug("%s: cam mode=%d\n", __func__, cam_pd_mode);
		mutex_lock(&ctx->mutex);
		ret = pd_camera_update_state(ctx, ctx->state.driver_mode,
			ctx->state.detect_enable,
			ctx->state.detect_value, cam_pd_mode);
		mutex_unlock(&ctx->mutex);
		return ret;
	case PD_CAM_PRESENCE_DETECTED:
		pr_debug("%s: presence detected\n", __func__);
		mutex_lock(&ctx->mutex);
		ret = pd_camera_update_state(ctx, ctx->state.driver_mode,
			ctx->state.detect_enable,
			1, ctx->state.camera_in_pd_mode);
		mutex_unlock(&ctx->mutex);
		return ret;
	case PD_CAM_GET_ENABLE:
		if (copy_from_user(&msg, argp, sizeof(msg)))
			return -EFAULT;
		if ((msg.cur_val & ~1U) || (msg.timeout_in_ms < -1))
			return -EINVAL;
		mutex_lock(&ctx->mutex);
		if (ctx->state.detect_enable != msg.cur_val) {
			ret = ctx->state.detect_enable;
			ctx->last_user_read_detect_enable = ret;
			mutex_unlock(&ctx->mutex);
			pr_debug("%s: cur=%d new=%d\n", __func__,
					msg.cur_val, ret);
			return ret;
		}
		mutex_unlock(&ctx->mutex);
		if (msg.timeout_in_ms > 0) {
			ret = wait_for_completion_interruptible_timeout(
					&ctx->enable_changed_compl,
					msecs_to_jiffies(msg.timeout_in_ms));
		} else if (msg.timeout_in_ms < 0) {
			ret = wait_for_completion_interruptible(
					&ctx->enable_changed_compl);
		}
		pr_debug("%s: ret=%d new val=%d\n", __func__,
					ret, ctx->state.detect_enable);
		mutex_lock(&ctx->mutex);
		ret = ctx->state.detect_enable;
		ctx->last_user_read_detect_enable = ret;
		mutex_unlock(&ctx->mutex);
		return ret;
	default:
		pr_err("%s: invalid cmd=%d\n", __func__, cmd);
		return -EINVAL;
	}
	return ret;
}

static unsigned int pd_camera_poll(struct file *file, poll_table *wait)
{
	struct pd_camera_ctx *ctx = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &ctx->enable_change_wait_queue, wait);
	if (ctx->last_user_read_detect_enable != ctx->state.detect_enable)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

const struct file_operations pd_camera_fops = {
	.owner = THIS_MODULE,
	.open = pd_camera_open,
	.unlocked_ioctl = pd_camera_ioctl,
	.compat_ioctl = pd_camera_ioctl,
	.release = pd_camera_release,
	.poll = pd_camera_poll,
};

static int pd_camera_probe(struct platform_device *pdev)
{
	struct pd_camera_ctx *ctx;
	int err;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pdev = pdev;
	platform_set_drvdata(pdev, ctx);

	mutex_init(&ctx->mutex);
	wake_lock_init(&ctx->wlock, WAKE_LOCK_SUSPEND, "pd_camera_wake_lock");
	init_completion(&ctx->enable_changed_compl);
	init_waitqueue_head(&ctx->enable_change_wait_queue);

	ctx->state.detect_enable = 0;
	ctx->state.detect_value = 0;
	ctx->state.camera_in_pd_mode = 0;
	ctx->state.actual_detect_enable = 0;
	ctx->state.irq_enabled = 0;
	ctx->state.driver_mode = PD_DEFAULT_DRIVER_MODE;

	ctx->last_user_read_detect_enable = 0;
	ctx->pd_events_since_last_read = 0;

	/* Get the irq */
	ctx->irq = of_get_named_gpio_flags(pdev->dev.of_node, "gpio-irq", 0,
						&ctx->irq_active);
	pr_debug("%s: %d 0x%x\n", __func__, ctx->irq, ctx->irq_active);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	err = sysfs_create_group(&pdev->dev.kobj,
				 &pd_camera_attribute_group);
	if (err < 0) {
		dev_err(&pdev->dev, "Sysfs registration failed\n");
		goto fail;
	}

	/* Set wake up enable */
	device_init_wakeup(&pdev->dev, 1);

	INIT_DELAYED_WORK(&ctx->timeout_work, pd_camera_timeout_work);
	ctx->pd_camera_wq = create_singlethread_workqueue("pd_camera_wq");


	ctx->misc_dev.minor = MISC_DYNAMIC_MINOR;
	ctx->misc_dev.name = PD_CAMERA_DEV_NAME;
	ctx->misc_dev.fops = &pd_camera_fops;

	err = misc_register(&ctx->misc_dev);
	if (err)
		goto fail;

	dev_set_drvdata(ctx->misc_dev.this_device, ctx);


	pr_info("%s: probe completed with ret=%d\n", __func__, err);

	return err;

fail:
	wake_lock_destroy(&ctx->wlock);
	mutex_destroy(&ctx->mutex);
	devm_kfree(&pdev->dev, ctx);

	return err;
}

static int pd_camera_remove(struct platform_device *pdev)
{
	struct pd_camera_ctx *ctx = platform_get_drvdata(pdev);

	/* Clear wake up enable */
	device_init_wakeup(&pdev->dev, 0);

	if (ctx->state.detect_enable)
		pd_camera_irq_request(ctx, false);

	sysfs_remove_group(&pdev->dev.kobj,
			   &pd_camera_attribute_group);

	cancel_delayed_work_sync(&ctx->timeout_work);
	flush_workqueue(ctx->pd_camera_wq);
	destroy_workqueue(ctx->pd_camera_wq);

	wake_lock_destroy(&ctx->wlock);
	mutex_destroy(&ctx->mutex);
	devm_kfree(&pdev->dev, ctx);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pd_camera_suspend(struct device *dev)
{
	struct pd_camera_ctx *ctx = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(ctx->irq);

	return 0;
}

static int pd_camera_resume(struct device *dev)
{
	struct pd_camera_ctx *ctx = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(ctx->irq);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int pd_camera_runtime_suspend(struct device *dev)
{
	return 0;
}

static int pd_camera_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops pd_camera_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pd_camera_suspend, pd_camera_resume)
	SET_RUNTIME_PM_OPS(pd_camera_runtime_suspend,
				pd_camera_runtime_resume, NULL)
};

static const struct platform_device_id pd_camera_id_table[] = {
	{ "pd_camera", },
	{},
};

static struct platform_driver pd_camera_plat_driver = {
	.probe = pd_camera_probe,
	.remove = pd_camera_remove,
	.id_table = pd_camera_id_table,
	.driver = {
		.name = "pd_camera",
		.owner = THIS_MODULE,
		.pm = &pd_camera_pm_ops,
	},
};

module_platform_driver(pd_camera_plat_driver);
MODULE_DESCRIPTION("Driver for presence detection using camera");
MODULE_LICENSE("GPL v2");
