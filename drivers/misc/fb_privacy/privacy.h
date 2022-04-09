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

#ifndef _PRIVACY_H
#define _PRIVACY_H

enum fbp_cmds {
	CMD_LED_STATE = 0,
	CMD_LED_BRIGHTNESS,
	CMD_SET_PRIVACY,
	CMD_GET_PRIVACY,
	CMD_SET_ALLOW,
	CMD_MAX,
};

enum fbp_led_state {
	LED_DEFAULT = 0,
	LED_IN_CALL,
	LED_BLUE,
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

struct fbp_cmd_rsp {
	int32_t status;
};

struct fb_privacy_context {
	enum fbp_privacy_state privacy;
	struct qseecom_handle *fbp_qhandle_priv;
	struct qseecom_handle *fbp_qhandle_led;
	struct mutex mutex;
	bool bootstrap;

	struct kobject *module_kobj;
	struct task_struct *fbp_task;
	wait_queue_head_t waitqueue;
	spinlock_t event_lock;
	bool fbp_event;
	atomic_t fbp_ack;
	atomic_t fbp_transition_time;
	unsigned int fbp_ack_timeout;
	bool legacy_hw;
};

extern bool fbp_get_privacy(void);
#endif
