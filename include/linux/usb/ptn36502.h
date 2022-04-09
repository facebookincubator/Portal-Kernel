/*
 * ptn36502.c
 *
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
 *
 */

#ifndef _PTN36502_H
#define _PTN36502D_H

enum mode_select {
	USB3_ONLY = 0,
	DP4_LANE,
};

enum orientation {
	UNPLUG = 0,
	POSITIVE,
	NEGATIVE,
};

#if defined(CONFIG_USB_PTN36502)
void ptn36502_setmode(enum mode_select mode, enum orientation ori);
#else
static inline void ptn36502_setmode(enum mode_select mode, enum orientation ori) {}
#endif
#endif /* _PTN36502_H */
