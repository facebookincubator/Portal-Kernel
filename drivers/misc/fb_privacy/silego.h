/*
 * Copyright (C) 2020 Facebook Inc.
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
#ifndef _SILEGO_H
#define _SILEGO_H

enum privacy_gpios {
	GPIO_PRIVACY_ENTER = 0,
	GPIO_MIC_PRIVACY,
	GPIO_CAM_PRIVACY,
	GPIO_MAX,
};

struct gpio_state {
	u32 gpio[GPIO_MAX];
	u32 gpio_change_allow;
};

extern bool toggle_enter_privacy(void);
extern int setup_gpio(struct device_node *np, void *fbp_ctx);
extern enum fbp_privacy_state get_hw_privacy_state(void);
extern void enable_change_allow(void);

#endif
