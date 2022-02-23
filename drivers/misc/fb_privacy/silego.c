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

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>

#include "silego.h"
#include "privacy.h"
static struct gpio_state gpio_state;

int setup_gpio(struct device_node *np, void *fbp_ctx)
{
	int ret;
	enum privacy_gpios gpio_idx;

	ret = of_property_read_u32_array(np, "fb,gpio_pins",
					 gpio_state.gpio, GPIO_MAX);
	if (ret)
		goto exit;

	for (gpio_idx = GPIO_PRIVACY_ENTER; gpio_idx < GPIO_MAX;) {
		ret = gpio_request(gpio_state.gpio[gpio_idx], NULL);
		gpio_idx++;
		if (ret)
			goto err;
	}

	gpio_state.gpio_change_allow = of_get_named_gpio(np, "fb,change-allow-gpio", 0);
	if (gpio_is_valid(gpio_state.gpio_change_allow)) {
		ret = gpio_request(gpio_state.gpio_change_allow, NULL);
		if (ret)
			goto err;

		ret = gpio_direction_output(gpio_state.gpio_change_allow, 0);
		if (ret)
			goto err;
	}

	ret = gpio_direction_output(gpio_state.gpio[GPIO_PRIVACY_ENTER], 0);
	if (ret)
		goto err;

	ret = gpio_direction_input(gpio_state.gpio[GPIO_CAM_PRIVACY]);
	if (ret)
		goto err;

	ret = gpio_direction_input(gpio_state.gpio[GPIO_MIC_PRIVACY]);
	if (ret)
		goto err;

	return 0;
err:
	for (gpio_idx--; gpio_idx >= GPIO_PRIVACY_ENTER; gpio_idx--)
		gpio_free(gpio_state.gpio[gpio_idx]);
exit:
	return ret;
}

bool toggle_enter_privacy(void)
{
	/* Send a pulse */
	gpio_set_value(gpio_state.gpio[GPIO_PRIVACY_ENTER], 1);
	udelay(1);
	gpio_set_value(gpio_state.gpio[GPIO_PRIVACY_ENTER], 0);

	/*
	 * Ensure that both the camera and mic GPIOs are low before
	 * declaring that privacy is ON
	 */
	if (!gpio_get_value(gpio_state.gpio[GPIO_CAM_PRIVACY]) &&
		 !gpio_get_value(gpio_state.gpio[GPIO_MIC_PRIVACY]))
		return true;

	return false;
}

enum fbp_privacy_state get_hw_privacy_state(void)
{
	/*
	 * Since SW is responsible for turning on privacy, be more
	 * stringent with checks to determine if privacy is enabled/not.
	 * That is, even if one of the GPIOs is set high at that moment,
	 * determine that privacy is disabled.
	 */
	if (!gpio_get_value(gpio_state.gpio[GPIO_CAM_PRIVACY]) &&
		 !gpio_get_value(gpio_state.gpio[GPIO_MIC_PRIVACY]))
		return PRIVACY_ON;
	else
		return PRIVACY_OFF;
}

void enable_change_allow(void)
{
	if(gpio_is_valid(gpio_state.gpio_change_allow))
		gpio_set_value(gpio_state.gpio_change_allow, 1);
}
