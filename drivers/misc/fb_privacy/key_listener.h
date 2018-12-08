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

#define KEY_RELEASE		0
#define KEY_PRESS		1

struct fbp_listener_state {
	spinlock_t state_lock;
	struct hrtimer release_timer;
	u32 release_timeout;
	u32 key_code;
	void (*key_fn)(void *);
	void *key_fn_data;
	u32 next_event;
	bool init_complete;
};

extern int fbp_register_key_callback(void (*fn)(void *), void *arg);
extern int fbp_unregister_key_callback(void);
