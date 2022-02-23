/*
 * Copyright (C) 2019-2020 Facebook Inc.
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
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/pm_opp.h>
#include <linux/pm_wakeup.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/ktime.h>

#include "privacy.h"
#include "led_control.h"
#include "silego.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fb_privacy.h>

#define MAX_ACK_TIMEOUT			10000 /* 10 seconds */
#define MAX_WAKEUP_TIMEOUT			 5000 /*  5 seconds */

static struct fb_privacy_context *fbp_ctx;

static void fbp_event_handler_callback(void *arg);
static int fbp_kthread(void *arg);
static unsigned int bl_state;

static int __init fbp_bl_state(char *str)
{
	int ret;

	ret = kstrtouint(str, 0, &bl_state);
	if (ret)
		return ret;

	return bl_state;
}
__setup("fbp.bl_state=", fbp_bl_state);

static bool fbp_should_enter_privacy(void)
{
	if (fbp_input_verify_hw_state(PRIVACY_ON) && fbp_ctx->privacy == PRIVACY_OFF)
		return true;
	else
		return false;
}

static bool fbp_should_exit_privacy(void)
{
	if (fbp_input_verify_hw_state(PRIVACY_OFF) && fbp_ctx->privacy == PRIVACY_ON)
		return true;
	else
		return false;
}

/**
 * This is required to sync the driver when booting up in
 * privacy in case of stateless input handler
 */
static void fbp_init_privacy_state(void)
{
	if (get_hw_privacy_state() == PRIVACY_ON)
		fbp_ctx->privacy = PRIVACY_ON;
	else if (bl_state == PRIVACY_ON)
		atomic_inc(&fbp_ctx->fbp_transitions);
}

static ssize_t fbp_privacy_show(char *buf, enum fbp_cmds cmd)
{
	uint8_t value;
	int ret;

	mutex_lock(&fbp_ctx->mutex);

	switch (cmd) {
	case CMD_LED_STATE:
		value = fbp_get_led_state();
		break;
	case CMD_LED_BRIGHTNESS:
		value = fbp_get_led_brightness();
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

static int _fbp_set_led_state(uint8_t state)
{
	int ret;

	if (state >= LED_STATE_MAX)
		return -EINVAL;

	if (fbp_get_led_state() == state)
		return 0;

	ret = fbp_set_led_state(state);
	if (ret)
		return ret;

	trace_fbp_state_change(fbp_ctx, fbp_get_led_state(),
			       fbp_get_led_brightness());
	return ret;
}

static int _fbp_set_brightness(uint8_t brightness)
{
	int ret;

	ret = fbp_set_led_brightness(brightness);
	if (ret)
		return ret;

	trace_fbp_state_change(fbp_ctx, fbp_get_led_state(),
			       fbp_get_led_brightness());
	return ret;
}

static int _fbp_set_privacy(uint8_t state)
{
	/*
	 * In case of failure to enter HW privacy we still enter SW privacy
	 * The thread will try again to get us in sync in all other scenarios
	 */
	if (state == PRIVACY_ON) {
			if (!toggle_enter_privacy()) {
				pr_err("%s: Failed toggle privacy \n", __func__);
				return -EINVAL;
			}

		pr_info("Entering privacy\n");
	} else {
		if (get_hw_privacy_state() == PRIVACY_ON) {
			pr_err("%s: Failed exiting privacy \n", __func__);
			return -EINVAL;
		}

		pr_info("Exiting privacy\n");
	}

	fbp_ctx->privacy = state;
	trace_fbp_state_change(fbp_ctx, fbp_get_led_state(),
			       fbp_get_led_brightness());
	return 0;
}

bool fbp_get_privacy(void)
{
	if (fbp_ctx->privacy == PRIVACY_ON)
		return true;
	return false;
}

static int fbp_set_privacy(uint8_t state)
{
	if (state != PRIVACY_ON)
		return -EINVAL;

	if (fbp_ctx->privacy == state)
		return 0;

	atomic_inc(&fbp_ctx->fbp_event);
	wake_up(&fbp_ctx->waitqueue);

	return 0;
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

	/* State management */
	if (cmd == CMD_LED_STATE)
		ret = _fbp_set_led_state(value);
	else if (cmd == CMD_LED_BRIGHTNESS)
		ret = _fbp_set_brightness(value);
	else
		ret = fbp_set_privacy(value);

	if (!ret)
		ret = count;

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

static ssize_t fbp_privacy_transitions_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"%u\n", atomic_read(&fbp_ctx->fbp_transitions));
}

static ssize_t fbp_transition_time_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"%u\n", atomic_read(&fbp_ctx->fbp_transition_time));
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

static struct kobj_attribute fbp_privacy_transitions_attr =
__ATTR(fbp_privacy_transitions, 0440, fbp_privacy_transitions_show, NULL);

static struct kobj_attribute fbp_transition_time_attr =
__ATTR(fbp_transition_time, 0440, fbp_transition_time_show, NULL);

static struct attribute *fbp_attrs[] = {
	&fbp_privacy_state_attr.attr,
	&fbp_led_state_attr.attr,
	&fbp_led_brightness_attr.attr,
	&fbp_privacy_ack_attr.attr,
	&fbp_ack_timeout_attr.attr,
	&fbp_privacy_transitions_attr.attr,
	&fbp_transition_time_attr.attr,
	NULL,
};

module_param_cb(fbp_boot_state, NULL, NULL, 0644);

static struct attribute_group fbp_attr_group = {
	.attrs = fbp_attrs,
};

static void fbp_event_handler_callback(void *arg)
{
	atomic_inc(&fbp_ctx->fbp_event);
	wake_up(&fbp_ctx->waitqueue);
	trace_fbp_key_event_callback(true);
}

static void fbp_set_privacy_at_boot_callback(enum fbp_privacy_state state)
{
	if (state == PRIVACY_ON) {
		atomic_inc(&fbp_ctx->fbp_event);
		wake_up(&fbp_ctx->waitqueue);
	}
}

static int fbp_kthread(void *arg)
{
	int ret;
	ktime_t start, end;
	long transition_time;

	while (!kthread_should_stop()) {
		if (kthread_should_park())
			kthread_parkme();

		wait_event_interruptible(fbp_ctx->waitqueue,
					 (atomic_read(&fbp_ctx->
							      fbp_event) >= 1 ||
					  kthread_should_stop() ||
					  kthread_should_park()));

		if (atomic_read(&fbp_ctx->fbp_event) == 0)
			continue;

		mutex_lock(&fbp_ctx->mutex);
		/* We do not expect the privacy handling to take more than MAX_WAKEUP_TIMEOUT msecs.
		 * Release the wakelock automatically after that time has elapsed.
		 */
		start = ktime_get();

		__pm_wakeup_event(fbp_ctx->wake_lock, MAX_WAKEUP_TIMEOUT);

		atomic_dec(&fbp_ctx->fbp_event);

		if(fbp_should_exit_privacy()) {
			ret = _fbp_set_privacy(PRIVACY_OFF);
			if (ret)
				goto loop_end;

			sysfs_notify(fbp_ctx->module_kobj, NULL,
					     "fbp_privacy_state");
			atomic_inc(&fbp_ctx->fbp_transitions);
		} else if(fbp_should_enter_privacy()) {
			fbp_ctx->privacy = PRIVACY_TRANSITION;
			trace_fbp_state_change(fbp_ctx, fbp_get_led_state(),
					       fbp_get_led_brightness());

			atomic_set(&fbp_ctx->fbp_ack, 0);
			sysfs_notify(fbp_ctx->module_kobj, NULL,
				     "fbp_privacy_state");

			if (fbp_ctx->fbp_ack_timeout) {
				unsigned int timeout = fbp_ctx->fbp_ack_timeout;

				mutex_unlock(&fbp_ctx->mutex);

				wait_event_timeout(fbp_ctx->waitqueue,
						   atomic_read(&fbp_ctx->
							       fbp_ack) == 1,
						   msecs_to_jiffies(timeout));

				mutex_lock(&fbp_ctx->mutex);
			}

			ret = _fbp_set_privacy(PRIVACY_ON);

			if (ret) {
				fbp_ctx->privacy = PRIVACY_OFF;
				trace_fbp_state_change(fbp_ctx,
						fbp_get_led_state(),
						fbp_get_led_brightness());
			} else {
				end = ktime_get();
				transition_time = ktime_us_delta(end, start);

				atomic_set(&fbp_ctx->fbp_transition_time, transition_time);

				atomic_inc(&fbp_ctx->fbp_transitions);
			}

			sysfs_notify(fbp_ctx->module_kobj, NULL,
				     "fbp_privacy_state");
		}

	loop_end:
		mutex_unlock(&fbp_ctx->mutex);

		/*
		 * We could have been woken up in the internal wait by either
		 * the timer expiry/ack from userspace or by switch toggle.
		 * Switch_event would be set if we came because of switch toggle
		 * and setting privacy would fail. We still need to repeat the
		 * process in case the event happened after setting privacy.
		 */
	}

	return 0;
}

static int fbp_setup_sysfs(void)
{
	int ret;

	fbp_ctx->module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!fbp_ctx->module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		return -ENOENT;
	}

	ret = sysfs_create_group(fbp_ctx->module_kobj, &fbp_attr_group);
	if (ret) {
		pr_err("%s: sysfs_create_group() failed: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int privacy_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		if (get_hw_privacy_state() != fbp_ctx->privacy) {
			pr_warn("%s: event %lu: Privacy state mismatch! Applying fix\n",
							__func__, event);
			atomic_inc(&fbp_ctx->fbp_event);
			wake_up(&fbp_ctx->waitqueue);
			trace_fbp_key_event_callback(true);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block privacy_pm_notifier = {
	.notifier_call = privacy_pm_event,
};

static int __init fbp_privacy_init(void)
{
	int ret;
	struct device_node *np;
	uint8_t brightness = 0;
	const char *led_name = NULL;

	np = of_find_node_by_name(NULL, "fb_privacy");
	if (!np) {
		pr_err("%s: can't find fb_privacy node\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u8(np, "fb,default-brightness", &brightness);
	pr_info("%s: brightness set to %d\n", __func__, brightness);
	if (ret)
		pr_err("%s: Brighness not set, resolving to default",
		       __func__);

	ret = of_property_read_string(np, "fb,privacy-led-trigger", &led_name);
	if (ret)
		pr_err("%s: Privacy-led-trigger is not set, resolving to default",
		       __func__);

	fbp_ctx = kzalloc(sizeof(struct fb_privacy_context), GFP_KERNEL);
	if (!fbp_ctx)
		return -ENOMEM;

	mutex_init(&fbp_ctx->mutex);

	ret = fbp_led_init(brightness, led_name);
	if (ret) {
		pr_err("%s: Unable to setup LEDs %d\n", __func__, ret);
		return ret;
	}

	ret = setup_gpio(np, fbp_ctx);
	if (ret) {
		pr_err("%s: GPIO setup failed %d\n", __func__, ret);
		return ret;
	}

	init_waitqueue_head(&fbp_ctx->waitqueue);

	fbp_ctx->wake_lock = wakeup_source_create("privacy_wl");
	if (!fbp_ctx->wake_lock) {
		pr_err("%s: Unable to create wake lock\n", __func__);
		return -ENOMEM;
	} else {
		wakeup_source_add(fbp_ctx->wake_lock);
	}

	ret = fbp_input_event_init(np);
	if (ret) {
		pr_err("%s: Event handler setup failed %d\n", __func__, ret);
		return ret;
	}

	/* Try to read from HW if input handler is stateless */
	ret = fbp_register_bootup_state_callback(fbp_set_privacy_at_boot_callback);
	if (ret)
		fbp_init_privacy_state();

	fbp_ctx->fbp_task = kthread_create(fbp_kthread, NULL, "fb_privacy");
	if (IS_ERR(fbp_ctx->fbp_task)) {
		pr_err("%s: Unable to create fb_privacy kthread\n", __func__);
		ret = PTR_ERR(fbp_ctx->fbp_task);
		return ret;
	}

	wake_up_process(fbp_ctx->fbp_task);

	ret = fbp_register_event_callback(fbp_event_handler_callback, NULL);
	if (ret) {
		pr_err("%s: Unable to register fbp_event_handler callback\n", __func__);
		return -EINVAL;
	}

	ret = fbp_setup_sysfs();
	if (ret)
		pr_err("%s: Unable to setup sysfs, err: %d\n", __func__, ret);

	/* If change_allow exists, enable now */
	enable_change_allow();

	register_pm_notifier(&privacy_pm_notifier);

	/* If change_allow exists, enable now */
	enable_change_allow();

	return ret;
}
late_initcall(fbp_privacy_init);

static const struct of_device_id fb_privacy_match[] = {
	{ .compatible = "fb_privacy",},
	{},
};
MODULE_DEVICE_TABLE(of, fb_privacy_match);

static struct platform_driver fb_privacy_driver = {
	.driver	= {
		.name = "fb_privacy",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(fb_privacy_match),
	},
};
module_platform_driver(fb_privacy_driver);
