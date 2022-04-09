/*
 * MELFAS MIP4 Touchscreen
 *
 * Copyright (C) 2000-2017 MELFAS Inc.
 *
 *
 * mip4_ts_mod.c : Model dependent functions
 *
 * Version : 2017.05.01
 */

#include "mip4_ts.h"

/*
* Pre-run config
*/
int mip4_ts_startup_config(struct mip4_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if (info->disable_esd) {
		dev_dbg(&info->client->dev, "%s - disable_esd\n", __func__);
		mip4_ts_disable_esd_alert(info);
	}

	//////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	// ...

	//
	//////////////////////////

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
* Config regulator
*/
int mip4_ts_config_regulator(struct mip4_ts_info *info)
{
	int ret = 0;

#ifdef CONFIG_REGULATOR
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	info->regulator_vd33 = regulator_get(&info->client->dev, "vd33");

	if (IS_ERR_OR_NULL(info->regulator_vd33)) {
		dev_err(&info->client->dev, "%s [ERROR] regulator_get : vd33\n", __func__);
		ret = PTR_ERR(info->regulator_vd33);
	} else {
		dev_dbg(&info->client->dev, "%s - regulator_get : vd33\n", __func__);

		ret = regulator_set_voltage(info->regulator_vd33, 3300000, 3300000);
		if (ret) {
			dev_dbg(&info->client->dev,
				"%s [ERROR] regulator_set_voltage : vd33\n",
				__func__);
		} else {
			dev_dbg(&info->client->dev, "%s - regulator_set_voltage : 3300000\n", __func__);
		}
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
#endif

	return ret;
}

/*
* Control regulator
*/
int mip4_ts_control_regulator(struct mip4_ts_info *info, int enable)
{
#ifdef CONFIG_REGULATOR
	int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	dev_dbg(&info->client->dev, "%s - switch : %d\n", __func__, enable);

	if (info->power == enable) {
		dev_dbg(&info->client->dev, "%s - skip\n", __func__);
		goto exit;
	}

	//////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	if (IS_ERR_OR_NULL(info->regulator_vd33)) {
		dev_err(&info->client->dev, "%s [ERROR] regulator_vd33 not found\n", __func__);
		goto exit;
	}

	if (enable) {
		ret = regulator_enable(info->regulator_vd33);
		if (ret) {
			dev_err(&info->client->dev, "%s [ERROR] regulator_enable : vd33\n", __func__);
			goto error;
		} else {
			dev_dbg(&info->client->dev, "%s - regulator_enable\n", __func__);
		}

#ifdef CONFIG_OF
		if (!IS_ERR_OR_NULL(info->pinctrl)) {
			ret = pinctrl_select_state(info->pinctrl, info->pins_enable);
			if (ret < 0) {
				dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state : pins_enable\n", __func__);
			}
		} else {
			dev_dbg(&info->client->dev,
				"%s [ERROR] pinctrl not found\n", __func__);
		}
#endif /* CONFIG_OF */
	} else {
		if (regulator_is_enabled(info->regulator_vd33)) {
			regulator_disable(info->regulator_vd33);
			dev_dbg(&info->client->dev, "%s - regulator_disable\n", __func__);
		}

#ifdef CONFIG_OF
		if (!IS_ERR_OR_NULL(info->pinctrl)) {
			ret = pinctrl_select_state(info->pinctrl, info->pins_disable);
			if (ret < 0) {
				dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state : pins_disable\n", __func__);
			}
		} else {
			dev_dbg(&info->client->dev,
				"%s [ERROR] pinctrl not found\n", __func__);
		}
#endif /* CONFIG_OF */
	}

	//
	//////////////////////////

	info->power = enable;

exit:
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return ret;
#else
	return 0;
#endif /* CONFIG_REGULATOR */
}

/*
* Turn off power supply
*/
int mip4_ts_power_off(struct mip4_ts_info *info)
{
	int __maybe_unused ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	//////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	/* Use CE pin */
	if (info->gpio_ce) {
		gpio_direction_output(info->gpio_ce, 0);
		dev_dbg(&info->client->dev, "%s - gpio_ce : 0\n", __func__);
	}

	/* Use VD33 regulator */
	mip4_ts_control_regulator(info, 0);

	/* Use VD33_EN pin */
	if (info->gpio_vd33_en) {
		gpio_direction_output(info->gpio_vd33_en, 0);
		dev_dbg(&info->client->dev, "%s - gpio_vd33_en : 0\n", __func__);
	}

#ifdef CONFIG_OF
	/* Use pinctrl */
#if 0
	if (!IS_ERR_OR_NULL(info->pinctrl)) {
		ret = pinctrl_select_state(info->pinctrl, info->pins_disable);
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state : pins_disable\n", __func__);
		} else {
			dev_dbg(&info->client->dev, "%s - pinctrl_select_state : disable\n", __func__);
		}
	}
#endif
#endif /* CONFIG_OF */

	//
	//////////////////////////

	usleep_range(1 * 1000, 2 * 1000);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
* Turn on power supply
*/
int mip4_ts_power_on(struct mip4_ts_info *info)
{
	int __maybe_unused ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	//////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	/* Use VD33 regulator */
	mip4_ts_control_regulator(info, 1);

	/* Use VD33_EN pin */
	if (info->gpio_vd33_en) {
		gpio_direction_output(info->gpio_vd33_en, 1);
		dev_dbg(&info->client->dev, "%s - gpio_vd33_en : 1\n", __func__);
	}

	/* Use CE pin */
	if (info->gpio_ce) {
		gpio_direction_output(info->gpio_ce, 1);
		dev_dbg(&info->client->dev, "%s - gpio_ce : 1\n", __func__);
	}

#ifdef CONFIG_OF
	/* Use pinctrl */
#if 0
	if (!IS_ERR_OR_NULL(info->pinctrl)) {
		ret = pinctrl_select_state(info->pinctrl, info->pins_enable);
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state : pins_enable\n", __func__);
		} else {
			dev_dbg(&info->client->dev, "%s - pinctrl_select_state : enable\n", __func__);
		}
	}
#endif
#endif /* CONFIG_OF */

	//
	//////////////////////////

#if !USE_STARTUP_WAITING
	msleep(200);
#endif

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
* Clear touch input event status
*/
void mip4_ts_clear_input(struct mip4_ts_info *info)
{
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Screen */
	for (i = 0; i < MAX_FINGER_NUM; i++) {
		/////////////////////////////////
		// PLEASE MODIFY HERE !!!
		//

		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

		input_report_key(info->input_dev, BTN_TOUCH, 0);
		//input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

		//input_mt_sync(info->input_dev);

		info->touch_state[i] = 0;

		//
		/////////////////////////////////
	}
	input_sync(info->input_dev);

	/* Key */
	if (info->key_enable == true) {
		for (i = 0; i < info->key_num; i++) {
			input_report_key(info->input_dev, info->key_code[i], 0);
		}
	}
	input_sync(info->input_dev);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/*
* Input event handler - Report input event
*/
void mip4_ts_input_event_handler(struct mip4_ts_info *info, u8 sz, u8 *buf)
{
	int i;
	int type;
	int id;
	int hover = 0;
	int palm = 0;
	int state = 0;
	int x, y, z;
	int size = 0;
	int pressure_stage = 0;
	int pressure = 0;
	int touch_major = 0;
	int touch_minor = 0;
	int finger_id = 0;
	int finger_cnt = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	//print_hex_dump(KERN_ERR, MIP4_TS_DEVICE_NAME " Event Packet : ", DUMP_PREFIX_OFFSET, 16, 1, buf, sz, false);

	if (info->gesture_wakeup_mode) {
		dev_notice(&info->client->dev, "wakeup by touch\n");
		info->gesture_wakeup_mode = false;
		input_report_key(info->input_dev, KEY_WAKEUP, 1);
		input_sync(info->input_dev);
		input_report_key(info->input_dev, KEY_WAKEUP, 0);
		input_sync(info->input_dev);

		/* Let's block until all released */
		info->skip_until_release = true;
	}

	for (i = 0; i < sz; i += info->event_size) {
		u8 *packet = &buf[i];

		/* Event format & type */
		if ((info->event_format == 0) || (info->event_format == 1)) {
			type = (packet[0] & 0x40) >> 6;
		} else if (info->event_format == 3) {
			type = (packet[0] & 0xF0) >> 4;
		} else {
			dev_err(&info->client->dev, "%s [ERROR] Unknown event format [%d]\n", __func__, info->event_format);
			goto exit;
		}
		dev_dbg(&info->client->dev, "%s - Type[%d]\n", __func__, type);

		/* Report input event */
		if (type == MIP4_EVENT_INPUT_TYPE_SCREEN) {
			/* Screen event */
			if (info->event_format == 0) {
				state = (packet[0] & 0x80) >> 7;
				hover = (packet[0] & 0x20) >> 5;
				palm = (packet[0] & 0x10) >> 4;
				id = (packet[0] & 0x0F) - 1;
				x = ((packet[1] & 0x0F) << 8) | packet[2];
				y = (((packet[1] >> 4) & 0x0F) << 8) | packet[3];
				pressure = packet[4];
				size = packet[5];
				touch_major = packet[5];
				touch_minor = packet[5];
			} else if (info->event_format == 1) {
				state = (packet[0] & 0x80) >> 7;
				hover = (packet[0] & 0x20) >> 5;
				palm = (packet[0] & 0x10) >> 4;
				id = (packet[0] & 0x0F) - 1;
				x = ((packet[1] & 0x0F) << 8) | packet[2];
				y = (((packet[1] >> 4) & 0x0F) << 8) | packet[3];
				pressure = packet[4];
				size = packet[5];
				touch_major = packet[6];
				touch_minor = packet[7];
			} else if (info->event_format == 3) {
				id = (packet[0] & 0x0F) - 1;
				hover = (packet[1] & 0x04) >> 2;
				palm = (packet[1] & 0x02) >> 1;
				state = (packet[1] & 0x01);
				x = ((packet[2] & 0x0F) << 8) | packet[3];
				y = (((packet[2] >> 4) & 0x0F) << 8) | packet[4];
				z = packet[5];
				size = packet[6];
				pressure_stage = (packet[7] & 0xF0) >> 4;
				pressure = ((packet[7] & 0x0F) << 8) | packet[8];
				touch_major = packet[9];
				touch_minor = packet[10];
			} else {
				dev_err(&info->client->dev, "%s [ERROR] Unknown event format [%d]\n", __func__, info->event_format);
				goto exit;
			}

			if (!((id >= 0) && (id < MAX_FINGER_NUM))) {
				dev_err(&info->client->dev, "%s [ERROR] Unknown finger id [%d]\n", __func__, id);
				continue;
			}

			/////////////////////////////////
			// PLEASE MODIFY HERE !!!
			//

			/* Report screen event */
			if (state == 1) {

				info->touch_state[id] = 1;

				if (info->skip_until_release)
					/* Ignore events */
					continue;

				/* Press or move event*/
				input_mt_slot(info->input_dev, id);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);

				input_report_key(info->input_dev, BTN_TOUCH, 1);
				//input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);

				input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
				input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
				input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, touch_minor);

				//input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, id);
				//input_mt_sync(info->input_dev);

				dev_dbg(&info->client->dev, "%s - Screen : ID[%d] X[%d] Y[%d] Z[%d] Major[%d] Minor[%d] Size[%d] Pressure[%d] Palm[%d] Hover[%d]\n", __func__, id, x, y, pressure, touch_major, touch_minor, size, pressure, palm, hover);
			} else if (state == 0) {

				info->touch_state[id] = 0;

				/* Final release event */
				finger_cnt = 0;
				for (finger_id = 0; finger_id < MAX_FINGER_NUM; finger_id++) {
					if (info->touch_state[finger_id] != 0) {
						finger_cnt++;
						break;
					}
				}

				if (info->skip_until_release && finger_cnt <= 0) {
					/* Now going back to normal mode from next touch */
					info->skip_until_release = false;
					continue;
				}

				/* Release event */
				input_mt_slot(info->input_dev, id);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

				//input_report_key(info->input_dev, BTN_TOUCH, 0);
				//input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

				//input_mt_sync(info->input_dev);


				dev_dbg(&info->client->dev, "%s - Screen : ID[%d] Release\n", __func__, id);

				if (finger_cnt <= 0) {
					input_report_key(info->input_dev, BTN_TOUCH, 0);
					//input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

					dev_dbg(&info->client->dev, "%s - Screen : Release\n", __func__);
				}
			} else {
				dev_err(&info->client->dev, "%s [ERROR] Unknown event state [%d]\n", __func__, state);
				goto exit;
			}

			//
			/////////////////////////////////
		} else if (type == MIP4_EVENT_INPUT_TYPE_KEY) {
			/* Key event */
			if ((info->event_format == 0) || (info->event_format == 1)) {
				id = (packet[0] & 0x0F) - 1;
				state = (packet[0] & 0x80) >> 7;
			} else if (info->event_format == 3) {
				id = (packet[0] & 0x0F) - 1;
				state = (packet[1] & 0x01);
			} else {
				dev_err(&info->client->dev, "%s [ERROR] Unknown event format [%d]\n", __func__, info->event_format);
				goto exit;
			}

			/* Report key event */
			if ((id >= 0) && (id < info->key_num)) {
				/////////////////////////////////
				// PLEASE MODIFY HERE !!!
				//

				int keycode = info->key_code[id];

				input_report_key(info->input_dev, keycode, state);

				dev_dbg(&info->client->dev, "%s - Key : ID[%d] Code[%d] Event[%d]\n", __func__, id, keycode, state);

				//
				/////////////////////////////////
			} else {
				dev_err(&info->client->dev, "%s [ERROR] Unknown key id [%d]\n", __func__, id);
				continue;
			}
		} else if (type == MIP4_EVENT_INPUT_TYPE_PROXIMITY) {
			/* Proximity event */

			/////////////////////////////////
			// PLEASE MODIFY HERE !!!
			//

			state = (packet[1] & 0x01);
			z = packet[5];

			dev_dbg(&info->client->dev, "%s - Proximity : State[%d] Value[%d]\n", __func__, state, z);

			//
			/////////////////////////////////
		} else {
			dev_err(&info->client->dev, "%s [ERROR] Unknown event type [%d]\n", __func__, type);
			goto exit;
		}
	}

	input_sync(info->input_dev);

exit:
	dev_dbg(&info->client->dev, "%s - Event buffer size: (%d) Processed buffer size(%d)\n", __func__, sz, i);
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/*
* Wake-up gesture event handler
*/
int mip4_ts_gesture_wakeup_event_handler(struct mip4_ts_info *info, int gesture_code)
{
	u8 wbuf[4];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/////////////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	/* Report wake-up event */
	dev_dbg(&info->client->dev, "%s - gesture[%d]\n", __func__, gesture_code);

	info->wakeup_gesture_code = gesture_code;

	switch (gesture_code) {
	case MIP4_EVENT_GESTURE_C:
	case MIP4_EVENT_GESTURE_W:
	case MIP4_EVENT_GESTURE_V:
	case MIP4_EVENT_GESTURE_M:
	case MIP4_EVENT_GESTURE_S:
	case MIP4_EVENT_GESTURE_Z:
	case MIP4_EVENT_GESTURE_O:
	case MIP4_EVENT_GESTURE_E:
	case MIP4_EVENT_GESTURE_V_90:
	case MIP4_EVENT_GESTURE_V_180:
	case MIP4_EVENT_GESTURE_FLICK_RIGHT:
	case MIP4_EVENT_GESTURE_FLICK_DOWN:
	case MIP4_EVENT_GESTURE_FLICK_LEFT:
	case MIP4_EVENT_GESTURE_FLICK_UP:
	case MIP4_EVENT_GESTURE_DOUBLE_TAP:
		/* Example : emulate power key */
		input_report_key(info->input_dev, KEY_WAKEUP, 1);
		input_sync(info->input_dev);
		input_report_key(info->input_dev, KEY_WAKEUP, 0);
		input_sync(info->input_dev);
		break;
	default:
		/* Re-enter gesture wake-up mode */
		wbuf[0] = MIP4_R0_CTRL;
		wbuf[1] = MIP4_R1_CTRL_POWER_STATE;
		wbuf[2] = MIP4_CTRL_POWER_LOW;
		if (mip4_ts_i2c_write(info, wbuf, 3)) {
			dev_err(&info->client->dev, "%s [ERROR] mip4_ts_i2c_write\n", __func__);
			goto error;
		}
		break;
	}

	//
	/////////////////////////////////

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	return 1;
}

/*
* Config GPIO
*/
int mip4_ts_config_gpio(struct mip4_ts_info *info)
{
	int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/* Interrupt */
	dev_dbg(&info->client->dev, "%s - gpio_intr[%d]\n", __func__, info->gpio_intr);
	if (info->gpio_intr) {
		ret = gpio_request(info->gpio_intr, "irq-gpio");
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] gpio_request : irq-gpio\n", __func__);
			goto error;
		} else {
			gpio_direction_input(info->gpio_intr);

			/* Set IRQ */
			info->client->irq = gpio_to_irq(info->gpio_intr);
			info->irq = info->client->irq;
			dev_dbg(&info->client->dev, "%s - gpio_to_irq : irq[%d]\n", __func__, info->irq);
		}
	}

	/* CE (Optional) */
	dev_dbg(&info->client->dev, "%s - gpio_ce[%d]\n", __func__, info->gpio_ce);
	if (info->gpio_ce) {
		ret = gpio_request(info->gpio_ce, "ce-gpio");
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] gpio_request : ce-gpio\n", __func__);
		} else {
			gpio_direction_output(info->gpio_ce, 1);
		}
	}

	/* VD33_EN (Optional) */
	dev_dbg(&info->client->dev, "%s - gpio_vd33_en[%d]\n", __func__, info->gpio_vd33_en);
	if (info->gpio_vd33_en) {
		ret = gpio_request(info->gpio_vd33_en, "vd33_en-gpio");
		if (ret < 0) {
			dev_dbg(&info->client->dev,
				"%s [ERROR] gpio_request : vd33_en-gpio\n",
				__func__);
		} else {
			gpio_direction_output(info->gpio_vd33_en, 1);
		}
	}

#ifdef CONFIG_OF
	/* Pinctrl (Optional) */
#if 0
	if (!IS_ERR_OR_NULL(info->pinctrl)) {
		ret = pinctrl_select_state(info->pinctrl, info->pins_enable);
		if (ret < 0) {
			dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state : pins_enable\n", __func__);
		} else {
			dev_dbg(&info->client->dev, "%s - pinctrl_select_state : enable\n", __func__);
		}
	}
#endif
#endif

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
/*
* Parse device tree
*/
int mip4_ts_parse_devicetree(struct device *dev, struct mip4_ts_info *info)
{
	struct device_node *np = dev->of_node;
	int ret;

	dev_dbg(dev, "%s [START]\n", __func__);

	/////////////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	/* Get Interrupt GPIO */
	ret = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(&info->client->dev, "%s [ERROR] of_get_named_gpio : irq-gpio\n", __func__);
		info->gpio_intr = 0;
	} else {
		info->gpio_intr = ret;
	}
	dev_dbg(dev, "%s - gpio_intr[%d]\n", __func__, info->gpio_intr);

	/* Get CE GPIO (Optional) */
	ret = of_get_named_gpio(np, "ce-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(&info->client->dev, "%s [ERROR] of_get_named_gpio : ce-gpio\n", __func__);
		info->gpio_ce = 0;
	} else {
		info->gpio_ce = ret;
	}
	dev_dbg(dev, "%s - gpio_ce[%d]\n", __func__, info->gpio_ce);

	/* Get VD33_EN GPIO (Optional) */
	ret = of_get_named_gpio(np, "vd33_en-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(&info->client->dev, "%s [ERROR] of_get_named_gpio : vd33_en-gpio\n", __func__);
		info->gpio_vd33_en = 0;
	} else {
		info->gpio_vd33_en = ret;
	}
	dev_dbg(dev, "%s - gpio_vd33_en[%d]\n", __func__, info->gpio_vd33_en);

	/* Get a flag for touch wakeup feature */
	info->touch_wakeup_on = of_property_read_bool(np, "touch-wakeup-on");

	/* Get Pinctrl (Optional) */
	info->pinctrl = devm_pinctrl_get(&info->client->dev);
	if (IS_ERR(info->pinctrl)) {
		dev_dbg(&info->client->dev,
			"%s [ERROR] devm_pinctrl_get\n", __func__);
	} else {
		info->pins_enable = pinctrl_lookup_state(info->pinctrl, "enable");
		if (IS_ERR(info->pins_enable)) {
			dev_err(&info->client->dev, "%s [ERROR] pinctrl_lookup_state : enable\n", __func__);
		}

		info->pins_disable = pinctrl_lookup_state(info->pinctrl, "disable");
		if (IS_ERR(info->pins_disable)) {
			dev_err(&info->client->dev, "%s [ERROR] pinctrl_lookup_state : disable\n", __func__);
		}
	}

	//
	/////////////////////////////////

	/* Config GPIO */
	ret = mip4_ts_config_gpio(info);
	if (ret) {
		dev_err(&info->client->dev, "%s [ERROR] mip4_ts_config_gpio\n", __func__);
		goto error;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}
#endif

/*
* Config input interface
*/
void mip4_ts_config_input(struct mip4_ts_info *info)
{
	struct input_dev *input_dev = info->input_dev;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/////////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	/* Screen */
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	set_bit(BTN_TOUCH, input_dev->keybit);
	//set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	//input_mt_init_slots(input_dev, MAX_FINGER_NUM);
	input_mt_init_slots(input_dev, MAX_FINGER_NUM, INPUT_MT_DIRECT);

	//input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);

	/*
	 * Even when firmware's in bad state, we should at least register driver
	 * with proper values to make touch working after firmware update.
	 * Set default screen size if the values are zero
	 */
	if (info->max_x == 0)
		info->max_x = INPUT_TOUCH_MAX_X;
	if (info->max_y == 0)
		info->max_y = INPUT_TOUCH_MAX_Y;

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, INPUT_TOUCH_MINOR_MAX, 0, 0);

	/* Key */
	//set_bit(KEY_BACK, input_dev->keybit);
	//set_bit(KEY_MENU, input_dev->keybit);

	//info->key_code[0] = KEY_BACK;
	//info->key_code[1] = KEY_MENU;

	set_bit(KEY_WAKEUP, input_dev->keybit);

	//
	/////////////////////////////

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

