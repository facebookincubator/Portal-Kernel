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

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <trace/events/fb_privacy.h>

#include "switch_handler.h"
#include "silego.h"

static struct fbp_switch_handler_state switch_data;
static uint8_t camera_shutter_closed = U8_MAX;
static uint8_t privacy_enabled = U8_MAX;

static enum fbp_switch_pos switch_position_state(void)
{
	enum fbp_switch_pos pos;

	if (camera_shutter_closed == U8_MAX || privacy_enabled == U8_MAX) {
		pos = SHUTTER_UNDETERMINED;
		return pos;
	}

	if (privacy_enabled)
		pos = SHUTTER_PRIVACY;
	else if (camera_shutter_closed ^ privacy_enabled)
		pos = SHUTTER_CLOSED;
	else
		pos = SHUTTER_OPEN;

	return pos;
}

static void shutter_state_event(struct input_handle *handle, unsigned int type,
			unsigned int code, int data)
{
	if (!(type == EV_SW && (code == SW_CAMERA_LENS_COVER || code == SW_PRIVACY)))
		return;

	if (!switch_data.init_complete)
		return;

	if (code == SW_CAMERA_LENS_COVER)
		camera_shutter_closed = data;
	else if (code == SW_PRIVACY)
		privacy_enabled = data;

	spin_lock(&switch_data.state_lock);
	if (switch_data.switch_handler_fn)
		switch_data.switch_handler_fn(NULL);
	spin_unlock(&switch_data.state_lock);

	trace_fbp_report_event_change(switch_position_state());
}

static int shutter_state_connect(struct input_handler *handler, struct input_dev *dev,
								 const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "shutter_state";

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

static void shutter_state_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static void shutter_state_start(struct input_handle *handle)
{
	enum fbp_switch_pos switch_pos;

	spin_lock_irq(&handle->dev->event_lock);

	if (test_bit(EV_SW, handle->dev->evbit) &&
			test_bit(SW_CAMERA_LENS_COVER, handle->dev->swbit))
		camera_shutter_closed = test_bit(SW_CAMERA_LENS_COVER, handle->dev->sw);

	if (test_bit(EV_SW, handle->dev->evbit) &&
			test_bit(SW_PRIVACY, handle->dev->swbit))
		privacy_enabled = test_bit(SW_PRIVACY, handle->dev->sw);

	spin_unlock_irq(&handle->dev->event_lock);
	/*
	 * Trigger a switch change event here in case the device is booting up
	 * with switch in privacy position.
	 */
	switch_pos = switch_position_state();
	if (switch_pos != SHUTTER_UNDETERMINED) {
		if (switch_pos == SHUTTER_PRIVACY && switch_data.bootup_switch_handler_fn)
			switch_data.bootup_switch_handler_fn(PRIVACY_ON);
		else
			switch_data.bootup_switch_handler_fn(PRIVACY_OFF);

		switch_data.bootup_switch_handler_fn = NULL;
	}
}

int fbp_unregister_event_callback(void)
{
	if (!switch_data.init_complete)
		return -EPERM;

	spin_lock(&switch_data.state_lock);
	switch_data.switch_handler_fn = NULL;
	switch_data.switch_handler_data = NULL;
	spin_unlock(&switch_data.state_lock);
	trace_fbp_unregister_callback(&switch_data);
	return 0;
}

int fbp_register_event_callback(void (*fn)(void *), void *data)
{
	if (!switch_data.init_complete)
		return -EPERM;

	spin_lock(&switch_data.state_lock);
	if (switch_data.switch_handler_fn != NULL) {
		spin_unlock(&switch_data.state_lock);
		WARN_ON(1);
		return -EAGAIN;
	}

	switch_data.switch_handler_fn = fn;
	switch_data.switch_handler_data = data;
	spin_unlock(&switch_data.state_lock);
	trace_fbp_register_callback(&switch_data);
	return 0;
}

int fbp_register_bootup_state_callback(void (*fn)(enum fbp_privacy_state state))
{
	switch_data.bootup_switch_handler_fn = fn;

	return 0;
}

int fbp_input_verify_hw_state(enum fbp_privacy_state state)
{
	enum fbp_switch_pos switch_pos;

	switch_pos = switch_position_state();

	switch (state) {
		case PRIVACY_ON:
			return switch_pos == SHUTTER_PRIVACY;
		case PRIVACY_OFF:
			return (switch_pos == SHUTTER_CLOSED || switch_pos == SHUTTER_OPEN);
		default:
			return false;
	}
}

static const struct input_device_id shutter_state_ids[] = {
{
	.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
	.evbit = { BIT_MASK(EV_SW) },
	.swbit = { [BIT_WORD(SW_CAMERA_LENS_COVER)] = BIT_MASK(SW_CAMERA_LENS_COVER) },
},
{
	.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
	.evbit = { BIT_MASK(EV_SW) },
	.swbit = { [BIT_WORD(SW_PRIVACY)] = BIT_MASK(SW_PRIVACY) },
},
{ }
};

static struct input_handler shutter_state_handler = {
	.name = "shutter_state",
	.event = shutter_state_event,
	.start = shutter_state_start,
	.connect = shutter_state_connect,
	.disconnect = shutter_state_disconnect,
	.id_table = shutter_state_ids,
};

int fbp_input_event_init(struct device_node *np)
{
	int ret;

	spin_lock_init(&switch_data.state_lock);

	ret = input_register_handler(&shutter_state_handler);
	if (ret)
		return ret;

	switch_data.init_complete = true;
	return 0;
}
