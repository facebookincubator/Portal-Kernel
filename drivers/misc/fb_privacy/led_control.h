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

#ifndef _LED_CONTROL_H
#define _LED_CONTROL_H

#include <linux/leds.h>
#include "privacy.h"

#define LED_OFF			0
#define MAX_LED_BRIGHTNESS	100
#define MIN_LED_BRIGHTNESS	5
#define DEFAULT_LED_BRIGHTNESS	7

struct led_controller {
	struct led_trigger *fbp_led[LED_STATE_MAX];
	enum fbp_led_state current_led_state;
	uint8_t current_brightness;
	uint8_t default_brightness;
};

extern int set_led_brightness(uint8_t);
extern int set_led_state(enum fbp_led_state);
extern int fbp_led_init(uint8_t);
extern void store_led_state(void);
extern void restore_led_state(void);
extern uint8_t get_led_brightness(void);
extern enum fbp_led_state get_led_state(void);
#endif
