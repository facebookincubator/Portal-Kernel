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

#ifndef _FB_EVENT_H_
#define _FB_EVENT_H_

#ifdef CONFIG_FB_EVENT
int fb_event_log_int(int32_t type, int value);
#else
static inline int fb_event_log_int(int32_t type, int value)
{
	return 0;
}
#endif

#endif /* _FB_EVENT_H_ */
