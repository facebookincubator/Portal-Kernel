/*
 * Copyright (C) 2018 Facebook Inc.
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

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include "key_listener.h"

#include <trace/events/fb_privacy.h>

static struct fbp_listener_state state;

int fbp_unregister_key_callback(void)
{
	if (!state.init_complete)
		return -EPERM;

	spin_lock(&state.state_lock);
	state.key_fn = NULL;
	state.key_fn_data = NULL;
	spin_unlock(&state.state_lock);
	trace_fbp_unregister_callback(&state);
	return 0;
}

int fbp_register_key_callback(void (*fn)(void *), void *data)
{
	if (!state.init_complete)
		return -EPERM;

	spin_lock(&state.state_lock);
	state.key_fn = fn;
	state.key_fn_data = data;
	spin_unlock(&state.state_lock);
	trace_fbp_register_callback(&state);
	return 0;
}

/* Called in interrupt context */
void fbp_report_key_event(u32 key_code, u32 event)
{
	if (!state.init_complete)
		return;

	if (key_code != state.key_code)
		return;

	event = !!event;

	spin_lock(&state.state_lock);

	/*
	 * hrtimer expiry will race with the key release event.
	 * While the spinlock serializes them, ordering cannot be
	 * guaranteed. To be consistent, an expired timer should
	 * invalidate any subsequent key release events and vice
	 * versa.
	 */
	if (event != state.next_event)
		goto out;

	if (event == KEY_PRESS) {
		if (hrtimer_is_queued(&state.release_timer))
			goto out;

		if (state.release_timeout)
			hrtimer_start(&state.release_timer,
				ms_to_ktime(state.release_timeout),
				HRTIMER_MODE_REL);

		state.next_event = KEY_RELEASE;
	} else {
		if (hrtimer_active(&state.release_timer))
			hrtimer_cancel(&state.release_timer);

		if (state.key_fn)
			state.key_fn(state.key_fn_data);

		state.next_event = KEY_PRESS;
	}

out:
	spin_unlock(&state.state_lock);
	trace_fbp_report_key_event(&state, event);
}

static enum hrtimer_restart fbp_key_press_handler(struct hrtimer *timer)
{
	spin_lock(&state.state_lock);
	state.next_event = KEY_PRESS;
	spin_unlock(&state.state_lock);

	trace_fbp_release_timer(&state);
	return HRTIMER_NORESTART;
}

static int __init fbp_key_listener_init(void)
{
	struct device_node *np;
	int ret;

	spin_lock_init(&state.state_lock);

	np = of_find_node_by_name(NULL, "fb_privacy");
	if (!np) {
		pr_err("can't find fb_privacy node\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "fb,release_timeout",
					&state.release_timeout);
	if (ret) {
		pr_err("%s: Release timeout not found in DT, err: %d\n",
						__func__, ret);
		return ret;
	}

	ret = of_property_read_u32(np, "fb,key_code",
					&state.key_code);
	if (ret) {
		pr_err("%s: Privacy key code not found in DT, err: %d\n",
						__func__, ret);
		return ret;
	}

	hrtimer_init(&state.release_timer,
			CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	state.release_timer.function = fbp_key_press_handler;
	state.next_event = KEY_PRESS;

	/*
	 * All initializes operations must be completed before key
	 * event handling can begin.
	 */
	mb();

	state.init_complete = true;
	return ret;
}
early_initcall(fbp_key_listener_init);
