/*
 * Copyright (C) 2018-2020 Facebook Inc.
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
#include <linux/input.h>
#include <linux/slab.h>
#include <trace/events/fb_privacy.h>

#include "button_handler.h"
#include "privacy.h"

static struct fbp_button_handler_state state;

int fbp_unregister_event_callback(void)
{
	if (!state.init_complete)
		return -EPERM;

	spin_lock(&state.state_lock);
	state.button_fn = NULL;
	state.button_fn_data = NULL;
	spin_unlock(&state.state_lock);
	trace_fbp_unregister_callback(&state);
	return 0;
}

int fbp_register_event_callback(void (*fn)(void *), void *data)
{
	if (!state.init_complete)
		return -EPERM;

	spin_lock(&state.state_lock);
	if (state.button_fn != NULL) {
		spin_unlock(&state.state_lock);
		WARN_ON(1);
		return -EAGAIN;
	}

	state.button_fn = fn;
	state.button_fn_data = data;
	spin_unlock(&state.state_lock);
	trace_fbp_register_callback(&state);
	return 0;
}

int fbp_register_bootup_state_callback(void (*fn)(enum fbp_privacy_state state))
{
	return -ENOSYS;
}

/*
 * The button event is stateless. We should assume that we can always
 * enter or exit privacy.
 */
int fbp_input_verify_hw_state(enum fbp_privacy_state state)
{
	return true;
}

static void button_event(struct input_handle *handle, unsigned int type,
			unsigned int code, int data)
{
	if (!(type == EV_SW && code == SW_PRIVACY))
		return;

	if (!state.init_complete)
		return;

	trace_fbp_button_event(data);

	if (data == BUTTON_RELEASE) {
		spin_lock(&state.state_lock);
		if (state.button_fn)
			state.button_fn(NULL);
		spin_unlock(&state.state_lock);
	}
}

static int button_connect(struct input_handler *handler, struct input_dev *dev,
								 const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "button_state";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	return 0;

 err_unregister_handle:
	input_unregister_handle(handle);
 err_free_handle:
	kfree(handle);
	return error;
}

static void button_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static void button_start(struct input_handle *handle)
{
	return;
}

static const struct input_device_id button_ids[] = {
{
	.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
	.evbit = { BIT_MASK(EV_SW) },
	.swbit = { [BIT_WORD(SW_PRIVACY)] = BIT_MASK(SW_PRIVACY) },
},
{ }
};

static struct input_handler button_handler = {
	.name = "button_state",
	.event = button_event,
	.start = button_start,
	.connect = button_connect,
	.disconnect = button_disconnect,
	.id_table = button_ids,
};

int fbp_input_event_init(struct device_node *np)
{
	int ret;

	spin_lock_init(&state.state_lock);

	ret = input_register_handler(&button_handler);
	if (ret)
		return ret;

	state.init_complete = true;
	return ret;
}
