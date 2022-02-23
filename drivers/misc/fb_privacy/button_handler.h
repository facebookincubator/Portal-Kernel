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

#ifndef _BUTTON_HANDLER_H
#define _BUTTON_HANDLER_H

#define BUTTON_RELEASE		0
#define BUTTON_PRESS		1

struct fbp_button_handler_state {
	spinlock_t state_lock;
	void (*button_fn)(void *);
	void *button_fn_data;
	bool init_complete;
};
#endif
