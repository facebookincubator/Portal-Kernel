/*
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/fb_privacy.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "../qseecom_kernel.h"
#include "key_listener.h"
#include "privacy.h"
#include "led_control.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fb_privacy.h>

#define TZAPP_PRIV_NAME "privacy"
/* Size of QSEECOM aligned send+rsp structures */
#define QSEECOM_SBUFF_SIZE 128

#define MAX_ACK_TIMEOUT			10000 /* 10 seconds */

#define MAX_QSEE_READY_RETRY		100
#define QSEE_READY_SLEEP_MS		5

static struct fb_privacy_context *fbp_ctx;

static BLOCKING_NOTIFIER_HEAD(fbp_notifier_list);
int fbp_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&fbp_notifier_list, nb);
}

int fbp_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&fbp_notifier_list, nb);
}

bool fbp_enter_privacy(struct fbp_notifier_info *info)
{
	if (info->new_state == PRIVACY_ON)
		return true;

	return false;
}

bool fbp_exit_privacy(struct fbp_notifier_info *info)
{
	if (info->new_state == PRIVACY_OFF)
		return true;

	return false;
}

static int fbp_start_tz_app(void)
{
	int ret;
	if (!fbp_ctx->fbp_qhandle_priv) {
		ret = qseecom_start_app(&fbp_ctx->fbp_qhandle_priv,
			TZAPP_PRIV_NAME, QSEECOM_SBUFF_SIZE);
		if (ret)
			pr_err("qseecom_start_app(%s) failed, err:%d\n",
				TZAPP_PRIV_NAME, ret);
	}

	return ret;
}

static int fbp_send_tz_cmd(uint8_t cmd, uint8_t value)
{
	struct fbp_send_cmd *req_buf;
	struct fbp_cmd_rsp *rsp_buf;
	int ret;
	int retry = MAX_QSEE_READY_RETRY;

	req_buf = (struct fbp_send_cmd *)(fbp_ctx->fbp_qhandle_priv->sbuf);
	req_buf->cmd = cmd;
	req_buf->value = value;

	rsp_buf = (struct fbp_cmd_rsp *)(fbp_ctx->fbp_qhandle_priv->sbuf +
			QSEECOM_ALIGN(sizeof(struct fbp_send_cmd)));

	while (retry && !qseecom_ready()) {
		retry--;
		msleep(QSEE_READY_SLEEP_MS);
	}

	ret = qseecom_send_command(fbp_ctx->fbp_qhandle_priv,
			req_buf, QSEECOM_ALIGN(sizeof(struct fbp_send_cmd)),
			rsp_buf, QSEECOM_ALIGN(sizeof(struct fbp_cmd_rsp)));

	if (ret)
		pr_err("%s: fbp_qseecom cmd err:%d status:%d\n", __func__,
				ret, rsp_buf->status);

	if (rsp_buf->status < 0)
		ret = -EIO;
	else if (rsp_buf->status > 0)
		/* Required by CMD_GET_PRIVACY */
		ret = rsp_buf->status;

	trace_fbp_send_tz_cmd(cmd, value, rsp_buf->status, ret);
	return ret;
}

static ssize_t fbp_privacy_show(char *buf, enum fbp_cmds cmd)
{
	uint8_t value;
	int ret;

	mutex_lock(&fbp_ctx->mutex);

	switch (cmd) {
	case CMD_LED_STATE:
		value = get_led_state();
		break;
	case CMD_LED_BRIGHTNESS:
		value = get_led_brightness();
		break;
	case CMD_GET_PRIVACY:
		value = fbp_ctx->privacy;
		break;
	default:
		pr_err("invalid cmd %d\n", cmd);
		ret = -EINVAL;
		goto out;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%hhu\n", value);
out:
	mutex_unlock(&fbp_ctx->mutex);
	return ret;
}

static void fbp_key_event_callback(void *arg);
static int fbp_kthread(void *arg);

static ssize_t
fbp_bootstrap_store(const char *buf)
{
	int ret;
	bool state;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	if (!state)
		return -EINVAL;

	if (fbp_ctx->legacy_hw)
		return -EINVAL;

	mutex_lock(&fbp_ctx->mutex);

	if (fbp_ctx->bootstrap) {
		ret = -EINVAL;
		goto out;
	}

	ret = fbp_start_tz_app();
	if (ret)
		goto out;

	/* Hardware can boot in privacy mode */
	ret = fbp_send_tz_cmd(CMD_GET_PRIVACY, 0);
	if (ret < 0)
		goto out;

	fbp_ctx->privacy = ret;

	/*
	 * Send delayed notification to all listerners when booting in
	 * privacy mode.
	 *
	 * fbp_enter_privacy : returns true
	 * fbp_exit_privacy : returns false
	 */
	if (ret) {
		struct fbp_notifier_info info;

		info.new_state = ret;
		blocking_notifier_call_chain(&fbp_notifier_list,
				FBP_PRECHANGE, (void *)&info);
		blocking_notifier_call_chain(&fbp_notifier_list,
				FBP_POSTCHANGE, (void *)&info);
		store_led_state();
	}

	fbp_ctx->fbp_task = kthread_create(fbp_kthread, NULL, "fb_privacy");

	if (IS_ERR(fbp_ctx->fbp_task)) {
		ret = PTR_ERR(fbp_ctx->fbp_task);
		goto out;
	}

	wake_up_process(fbp_ctx->fbp_task);

	ret = fbp_register_key_callback(fbp_key_event_callback, NULL);
	if (ret)
		goto out;

	ret = fbp_send_tz_cmd(CMD_SET_ALLOW, 1);
	if (ret)
		goto out;

	fbp_ctx->bootstrap = state;
out:
	mutex_unlock(&fbp_ctx->mutex);
	trace_fbp_bootstrap(fbp_ctx, get_led_state(), get_led_brightness());
	return ret;
}

static int
fbp_set_led_state(uint8_t state)
{
	int ret;

	if (state >= LED_STATE_MAX)
		return -EINVAL;

	if (get_led_state() == state)
		return 0;

	ret = set_led_state(state);
	if (ret)
		return ret;

	trace_fbp_state_change(fbp_ctx, get_led_state(), get_led_brightness());
	return ret;
}

static int fbp_set_brightness(uint8_t brightness)
{
	int ret;

	ret = set_led_brightness(brightness);
	if (ret)
		return ret;

	trace_fbp_state_change(fbp_ctx, get_led_state(), get_led_brightness());
	return ret;
}

static int _fbp_set_privacy(uint8_t state)
{
	struct fbp_notifier_info info;
	int ret;

	info.new_state = state;

	blocking_notifier_call_chain(&fbp_notifier_list,
				FBP_PRECHANGE, (void *)&info);

	ret = fbp_send_tz_cmd(CMD_SET_PRIVACY, state);
	if (ret) {
		blocking_notifier_call_chain(&fbp_notifier_list,
				FBP_CHANGE_FAIL, (void *)&info);
		return ret;
	}

	blocking_notifier_call_chain(&fbp_notifier_list,
				FBP_POSTCHANGE, (void *)&info);

	if (fbp_enter_privacy(&info))
		pr_info("Entering privacy\n");
	else
		pr_info("Exiting privacy\n");

	fbp_ctx->privacy = state;
	trace_fbp_state_change(fbp_ctx, get_led_state(), get_led_brightness());
	return ret;
}

bool fbp_get_privacy(void)
{
	if (fbp_ctx->privacy == PRIVACY_ON)
		return true;
	return false;
}

static int fbp_set_privacy(uint8_t state)
{
	int ret;

	if (state != PRIVACY_ON)
		return -EINVAL;

	if (fbp_ctx->privacy == state)
		return 0;

	spin_lock(&fbp_ctx->event_lock);

	if (fbp_ctx->fbp_event == true) {
		spin_unlock(&fbp_ctx->event_lock);
		return -EBUSY;
	}

	fbp_ctx->fbp_event = true;
	spin_unlock(&fbp_ctx->event_lock);

	ret = fbp_send_tz_cmd(CMD_SET_ALLOW, 0);
	if (ret)
		return ret;

	ret = _fbp_set_privacy(state);

	spin_lock(&fbp_ctx->event_lock);
	fbp_ctx->fbp_event = false;
	spin_unlock(&fbp_ctx->event_lock);

	if (fbp_send_tz_cmd(CMD_SET_ALLOW, 1))
		pr_err("Unable to re-enable change allow\n");

	return ret;
}

static ssize_t
fbp_privacy_store(const char *buf, size_t count, enum fbp_cmds cmd)
{
	uint8_t value;
	int ret;

	if (cmd >= CMD_GET_PRIVACY) {
		ret = -EINVAL;
		goto out_trace;
	}

	ret = kstrtou8(buf, 10, &value);
	if (ret)
		goto out_trace;

	mutex_lock(&fbp_ctx->mutex);

	if (!fbp_ctx->bootstrap) {
		ret = -EBUSY;
		goto out;
	}

	/* State management */
	if (cmd == CMD_LED_STATE)
		ret = fbp_set_led_state(value);
	else if (cmd == CMD_LED_BRIGHTNESS)
		ret = fbp_set_brightness(value);
	else
		ret = fbp_set_privacy(value);

	if (!ret)
		ret = count;

out:
	mutex_unlock(&fbp_ctx->mutex);
out_trace:
	trace_fbp_privacy_store(cmd, value, ret);
	return ret;
}

static ssize_t fbp_led_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return fbp_privacy_show(buf, CMD_LED_STATE);
}

static ssize_t fbp_led_state_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	return fbp_privacy_store(buf, count, CMD_LED_STATE);
}

static ssize_t fbp_led_brightness_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return fbp_privacy_show(buf, CMD_LED_BRIGHTNESS);
}

static ssize_t fbp_led_brightness_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	return fbp_privacy_store(buf, count, CMD_LED_BRIGHTNESS);
}

static ssize_t fbp_privacy_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return fbp_privacy_show(buf, CMD_GET_PRIVACY);
}

static ssize_t fbp_privacy_state_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	return fbp_privacy_store(buf, count, CMD_SET_PRIVACY);
}

static ssize_t fbp_privacy_ack_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"%u\n", atomic_read(&fbp_ctx->fbp_ack));
}

static ssize_t fbp_privacy_ack_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	atomic_set(&fbp_ctx->fbp_ack, 1);
	trace_fbp_ack_store(1);
	wake_up(&fbp_ctx->waitqueue);
	return count;
}

static ssize_t fbp_ack_timeout_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	unsigned int timeout;

	mutex_lock(&fbp_ctx->mutex);
	timeout = fbp_ctx->fbp_ack_timeout;
	mutex_unlock(&fbp_ctx->mutex);
	return scnprintf(buf, PAGE_SIZE, "%u\n", timeout);
}

static ssize_t fbp_ack_timeout_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int timeout;
	int ret;

	ret = kstrtouint(buf, 10, &timeout);
	if (ret)
		return ret;

	if (timeout > MAX_ACK_TIMEOUT)
		return -EINVAL;

	mutex_lock(&fbp_ctx->mutex);
	fbp_ctx->fbp_ack_timeout = timeout;
	mutex_unlock(&fbp_ctx->mutex);
	trace_fbp_timeout_store(timeout);
	return count;
}

static struct kobj_attribute fbp_privacy_state_attr =
__ATTR(fbp_privacy_state, 0660, fbp_privacy_state_show,
					fbp_privacy_state_store);

static struct kobj_attribute fbp_led_state_attr =
__ATTR(fbp_led_state, 0660, fbp_led_state_show, fbp_led_state_store);

static struct kobj_attribute fbp_led_brightness_attr =
__ATTR(fbp_led_brightness, 0660, fbp_led_brightness_show,
					fbp_led_brightness_store);

static struct kobj_attribute fbp_ack_timeout_attr =
__ATTR(fbp_ack_timeout, 0660, fbp_ack_timeout_show,
					fbp_ack_timeout_store);

static struct kobj_attribute fbp_privacy_ack_attr =
__ATTR(fbp_privacy_ack, 0660, fbp_privacy_ack_show,
					fbp_privacy_ack_store);

static struct attribute *fbp_attrs[] = {
	&fbp_privacy_state_attr.attr,
	&fbp_led_state_attr.attr,
	&fbp_led_brightness_attr.attr,
	&fbp_privacy_ack_attr.attr,
	&fbp_ack_timeout_attr.attr,
	NULL,
};

static int fbp_boot_state_show(char *buf, const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%hhu\n", fbp_ctx->bootstrap);
}

static int
fbp_boot_state_store(const char *buf, const struct kernel_param *kp)
{
	return fbp_bootstrap_store(buf);
}

static const struct kernel_param_ops param_ops_fbp_boot_state = {
	.set = fbp_boot_state_store,
	.get = fbp_boot_state_show,
};
module_param_cb(fbp_boot_state, &param_ops_fbp_boot_state, NULL, 0644);

static struct attribute_group fbp_attr_group = {
	.attrs = fbp_attrs,
};

static void fbp_key_event_callback(void *arg)
{
	spin_lock(&fbp_ctx->event_lock);

	if (fbp_ctx->fbp_event == true) {
		spin_unlock(&fbp_ctx->event_lock);
		return;
	}

	fbp_ctx->fbp_event = true;
	spin_unlock(&fbp_ctx->event_lock);
	trace_fbp_key_event_callback(true);
	wake_up(&fbp_ctx->waitqueue);
}

static int fbp_kthread(void *arg)
{
	int ret;

	while (!kthread_should_stop()) {
		if (kthread_should_park())
			kthread_parkme();

		wait_event_interruptible(fbp_ctx->waitqueue,
				(fbp_ctx->fbp_event ||
				kthread_should_stop() ||
				kthread_should_park()));

		if (!fbp_ctx->fbp_event)
			continue;

		mutex_lock(&fbp_ctx->mutex);

		/* Ignore event if we can't set change allow */
		if (fbp_send_tz_cmd(CMD_SET_ALLOW, 0)) {
			mutex_unlock(&fbp_ctx->mutex);
			spin_lock(&fbp_ctx->event_lock);
			fbp_ctx->fbp_event = false;
			spin_unlock(&fbp_ctx->event_lock);
			continue;
		}

		if (fbp_ctx->privacy == PRIVACY_ON) {
			/* Ask TZ to clean internal state */
			ret = _fbp_set_privacy(PRIVACY_OFF);
			if (ret)
				goto loop_end;
			restore_led_state();
			sysfs_notify(fbp_ctx->module_kobj, NULL,
					"fbp_privacy_state");
		} else {
			fbp_ctx->privacy = PRIVACY_TRANSITION;
			trace_fbp_state_change(fbp_ctx, get_led_state(),
					       get_led_brightness());

			sysfs_notify(fbp_ctx->module_kobj, NULL,
					"fbp_privacy_state");

			if (fbp_ctx->fbp_ack_timeout) {
				unsigned int timeout = fbp_ctx->fbp_ack_timeout;

				mutex_unlock(&fbp_ctx->mutex);

				wait_event_timeout(fbp_ctx->waitqueue,
					atomic_read(&fbp_ctx->fbp_ack) == 1,
					msecs_to_jiffies(timeout));

				mutex_lock(&fbp_ctx->mutex);
			}

			ret = _fbp_set_privacy(PRIVACY_ON);
			if (ret) {
				fbp_ctx->privacy = PRIVACY_OFF;
				trace_fbp_state_change(fbp_ctx,
						       get_led_state(),
						       get_led_brightness());
			}
			store_led_state();
			sysfs_notify(fbp_ctx->module_kobj, NULL,
					"fbp_privacy_state");
		}

loop_end:
		atomic_set(&fbp_ctx->fbp_ack, 0);

		spin_lock(&fbp_ctx->event_lock);
		fbp_ctx->fbp_event = false;
		spin_unlock(&fbp_ctx->event_lock);

		if (fbp_send_tz_cmd(CMD_SET_ALLOW, 1))
			pr_err("Unable to re-enable change allow\n");

		mutex_unlock(&fbp_ctx->mutex);
	}

	return 0;
}

int fbp_shutdown_apps(void)
{
	int ret;

	if (!fbp_ctx->bootstrap)
		return -EBUSY;

	if (!fbp_ctx->fbp_qhandle_priv)
		return 0;

	ret = qseecom_shutdown_app(&fbp_ctx->fbp_qhandle_priv);
	if (ret)
		return ret;

	/* Privacy app shutdown guarantees privacy on */
	fbp_ctx->privacy = PRIVACY_ON;

	fbp_unregister_key_callback();
	kthread_stop(fbp_ctx->fbp_task);

	/* This will allow the driver to start fresh */
	fbp_ctx->bootstrap = 0;
	fbp_ctx->fbp_qhandle_priv = NULL;
	trace_fbp_shutdown_apps(fbp_ctx, get_led_state(), get_led_brightness());
	return ret;
}
EXPORT_SYMBOL(fbp_shutdown_apps);

static int setup_sysfs(void)
{
	int ret;

	fbp_ctx->module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!fbp_ctx->module_kobj)
		return -ENOENT;

	ret = sysfs_create_group(fbp_ctx->module_kobj, &fbp_attr_group);
	if (ret)
		return ret;

	return 0;
}

static int __init fbp_privacy_init(void)
{
	int ret;
	struct device_node *np;
	uint8_t brightness = 0;

	fbp_ctx = kzalloc(sizeof(struct fb_privacy_context), GFP_KERNEL);
	if (!fbp_ctx)
		return -ENOMEM;

	spin_lock_init(&fbp_ctx->event_lock);
	mutex_init(&fbp_ctx->mutex);

	np = of_find_node_by_name(NULL, "fb_privacy");
	if (!np) {
		pr_err("can't find fb_privacy node\n");
		return -ENODEV;
	}

	ret = of_property_read_u8(np, "fb,default-brightness", &brightness);
	if (ret)
		pr_err("%s: Brighness not set, resolving to default",
		       __func__);

	ret = fbp_led_init(brightness);
	if (ret) {
		pr_err("Unable to setup LEDs %d\n", ret);
		return ret;
	}

	fbp_ctx->legacy_hw = of_property_read_bool(np, "fb,legacy-hardware");

	init_waitqueue_head(&fbp_ctx->waitqueue);

	ret = setup_sysfs();
	if (ret)
		pr_err("Unable to setup sysfs, err: %d\n", ret);


	return ret;
}
late_initcall(fbp_privacy_init);
