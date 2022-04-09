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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "led_control.h"

#define LED_NAME_GREEN "green"
#define LED_NAME_BLUE "blue"

#define LED_BRIGHTNESS_RANGE   0xFF

struct led_controller *led_ctrl;

static void __set_led_brightness(enum fbp_led_state state,
				uint8_t brightness)
{
	if (state == LED_DEFAULT)
		return;

	led_ctrl->current_led_state = (brightness == LED_OFF) ?
		LED_DEFAULT : state;
	led_ctrl->current_brightness = brightness;

	if (fbp_get_privacy())
		return;

	brightness = brightness * LED_BRIGHTNESS_RANGE / 100;
	led_trigger_event(led_ctrl->fbp_led[state], brightness);
}

int set_led_brightness(uint8_t brightness)
{
	if (brightness > MAX_LED_BRIGHTNESS ||
	    brightness < MIN_LED_BRIGHTNESS)
		return -EINVAL;

	if (led_ctrl->current_led_state == LED_DEFAULT)
		return -EINVAL;

	__set_led_brightness(led_ctrl->current_led_state, brightness);

	return 0;
}

uint8_t get_led_brightness(void)
{
	return led_ctrl->current_brightness;
}

enum fbp_led_state get_led_state(void)
{
	return led_ctrl->current_led_state;
}

static void reset_led(void)
{
	led_trigger_event(led_ctrl->fbp_led[LED_IN_CALL], LED_OFF);
	led_trigger_event(led_ctrl->fbp_led[LED_BLUE], LED_OFF);
}

void store_led_state(void)
{
	reset_led();
}

void restore_led_state(void)
{
	__set_led_brightness(led_ctrl->current_led_state,
			     led_ctrl->current_brightness);
}

int set_led_state(enum fbp_led_state state)
{
	int ret = 0;

	switch (state) {
	case LED_DEFAULT:
	{
		__set_led_brightness(led_ctrl->current_led_state, LED_OFF);
		break;
	}
	case LED_IN_CALL:
	{
		__set_led_brightness(led_ctrl->current_led_state, LED_OFF);
		__set_led_brightness(LED_IN_CALL, led_ctrl->default_brightness);
		break;
	}
	case LED_BLUE:
	{
		__set_led_brightness(led_ctrl->current_led_state, LED_OFF);
		__set_led_brightness(LED_BLUE, led_ctrl->default_brightness);
		break;
	}
	default:
		ret = -EINVAL;
	}

	return ret;
}

int fbp_led_init(uint8_t brightness)
{
	led_ctrl = kzalloc(sizeof(struct led_controller), GFP_KERNEL);
	if (!led_ctrl)
		return -ENOMEM;

	led_trigger_register_simple(LED_NAME_GREEN,
				    &led_ctrl->fbp_led[LED_IN_CALL]);
	if (!led_ctrl->fbp_led[LED_IN_CALL]) {
		pr_err("%s:Failed to register led interface\n", __func__);
		return -EINVAL;
	}

	led_trigger_register_simple(LED_NAME_BLUE,
				    &led_ctrl->fbp_led[LED_BLUE]);
	if (!led_ctrl->fbp_led[LED_BLUE]) {
		pr_err("%s:Failed to register led interface\n", __func__);
		return -EINVAL;
	}

	if (brightness > MAX_LED_BRIGHTNESS || brightness < MIN_LED_BRIGHTNESS)
		led_ctrl->default_brightness = DEFAULT_LED_BRIGHTNESS;
	else
		led_ctrl->default_brightness = brightness;

	led_ctrl->current_brightness = led_ctrl->default_brightness;
	reset_led();
	return 0;
}
