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
#ifndef _SWITCH_HANDLER_H
#define _SWITCH_HANDLER_H

#include "privacy.h"

enum fbp_switch_pos {
	SHUTTER_OPEN = 0,
	SHUTTER_CLOSED,
	SHUTTER_PRIVACY,
	SHUTTER_UNDETERMINED,
};

struct fbp_switch_handler_state {
	spinlock_t state_lock;
	void (*switch_handler_fn)(void *);
	void *switch_handler_data;
	void (*bootup_switch_handler_fn)(enum fbp_privacy_state);
	bool init_complete;
	enum fbp_switch_pos state;
};
#endif
