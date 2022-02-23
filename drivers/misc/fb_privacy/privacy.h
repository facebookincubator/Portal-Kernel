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

#ifndef _PRIVACY_H
#define _PRIVACY_H

#include <linux/pm_wakeup.h>

enum fbp_cmds {
	CMD_LED_STATE = 0,
	CMD_LED_BRIGHTNESS,
	CMD_SET_PRIVACY,
	CMD_GET_PRIVACY,
	CMD_MAX,
};

enum fbp_led_state {
	LED_DEFAULT = 0,
	LED_IN_CALL,
	LED_STATE_MAX,
};

enum fbp_privacy_state {
	PRIVACY_OFF = 0,
	PRIVACY_ON,
	PRIVACY_TRANSITION,
	PRIVACY_STATE_MAX,
};

struct fbp_send_cmd {
	uint8_t cmd;
	uint8_t value;
};

struct fb_privacy_context {
	enum fbp_privacy_state privacy;
	struct mutex mutex;
	struct kobject *module_kobj;
	struct task_struct *fbp_task;
	wait_queue_head_t waitqueue;
	atomic_t fbp_event;
	atomic_t fbp_ack;
	atomic_t fbp_transitions;
	atomic_t fbp_transition_time;
	unsigned int fbp_ack_timeout;
	struct wakeup_source *wake_lock;
};

extern bool fbp_get_privacy(void);

/**
 * Following set of functions need to be defined by any new
 * type of input handler. Only a single type of input handler
 * can be active at a time.
 */

/**
 * fbp_register_event_callback() - Register for input event
 * @fn: Function to be called when event is triggered
 * @arg: Arguments to be passed to caller
 *
 * The input handler needs to call this fn when
 * we receive an external trigger like button press or
 * switch change.
 */
extern int fbp_register_event_callback(void (*fn)(void *), void *arg);

/**
 * fbp_register_event_callback() - Unregister for input event
 *
 * Privacy driver will call this when the driver is unloading or
 * stopping for anyother reason.
 *
 */
extern int fbp_unregister_event_callback(void);

/**
 * fbp_register_bootup_state_callback() - Register for input state at bootup
 * @fn: Function to be called when bootup state is determined
 *
 * The input handler needs to call fn when it determines the boot
 * state of the device. The callback needs to happen before the start
 * of events being generated. return ENOSYS in case input handler is stateless.
 */
extern int fbp_register_bootup_state_callback(void (*fn)(enum fbp_privacy_state state));

/**
 * fbp_input_verify_hw_state() - Verify HW state to input
 * @state: Privacy state to compare against.
 *
 * This function is used by the Privacy driver to confirm
 * if the HW state matches the privacy state. This only applies
 * to input devices which have a HW state eg: switch. For all
 * other input devices it should always return true.
 */
extern int fbp_input_verify_hw_state(enum fbp_privacy_state state);

/**
 * fbp_input_event_init() - Perform init functionality
 * @np: Device node for reading any information from device tree.
 *
 * This function is to be used for any init calls for the input
 * handler. This is called as part of late init.
 */
extern int fbp_input_event_init(struct device_node *np);
#endif
