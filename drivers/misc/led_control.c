/*
 * This is a kernel module to communicate with TZ to control camera status LED.
 * The module exposes the familiar leds class interface for userspace to control
 * the color of camera status led. The LED controller can be controlled only
 * from TZ for security reasons and this module uses the QSEECom interface to
 * communicate with TZ.
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

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/fb_privacy.h>

#include "qseecom_kernel.h"

/* Size of QSEECOM aligned send+rsp structures */
#define QSEECOM_SBUFF_SIZE 128

#define LED_BRIGHTNESS 0

/* LED control modes */
#define LED_OFF                  0x0
#define LED_FULL_ON              0x1
#define LED_ON                   0x2
#define LED_BLINK                0x3
#define MODE_MASK                0x3

#define WR_CMD_MASK                       0x80
#define GRP_LED_CMD_MASK                  0x40
#define GRP_COLOR_CMD_MASK                0x20
#define RESERVED_MASK                     0x10
#define CMD_MASK                          0x0F
#define READ_COMMAND                      0x80
#define GRP_LED_CMD                       0x40
#define GRP_COLOR_CMD                     0x20
#define FULL_REG_MASK                     0xFF

#define MAX_NAME_LENGTH 15
static char ledapp_name[MAX_NAME_LENGTH];

enum LED_POS {
	LED_1,
	LED_2,
	LED_CAM,
	LED_NONE
};

enum LED_COLOR {
	COLOR_RED,
	COLOR_GREEN,
	COLOR_BLUE,
	COLOR_NONE
};

enum led_cmds {
	MODE,
	BRIGHTNESS,
	BLINK_BRIGHTNESS,
	BLINK_FREQUENCY,
	MAX_CMD_ID = 5
};


struct led_control_send_cmd {
	uint8_t cmd_id;
	uint8_t led_id;
	uint8_t color;
	uint8_t value;
};

struct led_control_cmd_rsp {
	int32_t status;
};

struct led_control_cfg {
	struct led_classdev cdev;
	struct led_control_send_cmd cmd;
	struct led_control_device *lcdev;
};

struct led_control_device {
	struct qseecom_handle *lchandle;
	struct led_control_cfg *cfg;
	int num_leds;
};

static int led_control_initialize(struct qseecom_handle *lchandle)
{
	struct led_control_send_cmd *req_buf;
	struct led_control_cmd_rsp *rsp_buf;
	int i, ret = 0;

	req_buf = (struct led_control_send_cmd *)(lchandle->sbuf);
	rsp_buf = (struct led_control_cmd_rsp *)(lchandle->sbuf +
			QSEECOM_ALIGN(sizeof(struct led_control_send_cmd)));

	req_buf->cmd_id = GRP_COLOR_CMD | BRIGHTNESS;
	req_buf->led_id = 1 << LED_1 | 1 << LED_2 | 1 << LED_CAM;
	req_buf->value = 0;
	for (i = COLOR_RED; i < COLOR_NONE; i++) {
		req_buf->color = i;
		ret |= qseecom_send_command(lchandle, req_buf,
			QSEECOM_ALIGN(sizeof(struct led_control_send_cmd)),
			rsp_buf,
			QSEECOM_ALIGN(sizeof(struct led_control_cmd_rsp)));
	}

	if (strncmp(ledapp_name, "tlc59116", MAX_NAME_LENGTH) == 0) {
		req_buf->value = LED_ON;
		req_buf->cmd_id = GRP_COLOR_CMD | MODE;

		for (i = COLOR_RED; i < COLOR_NONE; i++) {
			req_buf->color = i;
			ret |= qseecom_send_command(lchandle, req_buf,
			       QSEECOM_ALIGN(sizeof
					     (struct led_control_send_cmd)),
			       rsp_buf,
			       QSEECOM_ALIGN(sizeof
					     (struct led_control_cmd_rsp)));
		}

		if (ret)
			pr_err("%s: qseecom cmd err:%d\n", __func__, ret);
	}
	return ret;
}

/* Set LED brightness level */
static void led_control_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	int ret = 0;
	struct led_control_send_cmd *req_buf;
	struct led_control_cmd_rsp *rsp_buf;
	struct led_control_cfg *cfg =
		container_of(led_cdev, struct led_control_cfg, cdev);
	struct led_control_device *lcdev = cfg->lcdev;

	if (!lcdev->lchandle) {
		ret = fbp_shutdown_apps();
		if (ret) {
			pr_err("privacy apps shutdown failed, err:%d\n", ret);
			return;
		}

		/* All apps closed, start the new app */
		ret = qseecom_start_app(&lcdev->lchandle,
					ledapp_name, QSEECOM_SBUFF_SIZE);

		if (ret) {
			pr_err("%s app start failed, err:%d\n",
			       ledapp_name, ret);
			lcdev->lchandle = NULL;
			return;
		}

		ret = led_control_initialize(lcdev->lchandle);
		if (ret) {
			pr_err("led init failed, err:%d\n", ret);
			qseecom_shutdown_app(&lcdev->lchandle);
			lcdev->lchandle = NULL;
			return;
		}
	}

	req_buf = (struct led_control_send_cmd *)(lcdev->lchandle->sbuf);

	req_buf->cmd_id = BRIGHTNESS;

	if (cfg->cmd.cmd_id <= 3)
		req_buf->led_id = LED_CAM;
	else if (cfg->cmd.cmd_id <= 6)
		req_buf->led_id = LED_1;
	else
		req_buf->led_id = LED_2;

	if (cfg->cmd.cmd_id % 3 == 1)
		req_buf->color = COLOR_RED;
	else if (cfg->cmd.cmd_id % 3 == 2)
		req_buf->color = COLOR_GREEN;
	else
		req_buf->color = COLOR_BLUE;

	req_buf->value = (u8)brightness;
	cfg->cmd.value = (u8)brightness;

	rsp_buf = (struct led_control_cmd_rsp *)
	    (lcdev->lchandle->sbuf +
	     QSEECOM_ALIGN(sizeof(struct led_control_send_cmd)));

	ret = qseecom_send_command(lcdev->lchandle, req_buf,
				  QSEECOM_ALIGN(sizeof
						(struct led_control_send_cmd)),
				  rsp_buf,
				  QSEECOM_ALIGN(sizeof
						(struct led_control_cmd_rsp)));
	if (ret)
		pr_err("%s: qseecom cmd err:%d status:%d\n", __func__,
			ret, rsp_buf->status);
}

/* Get LED brightness level */
static enum led_brightness
	 led_control_get_brightness(struct led_classdev *led_cdev)
{
	struct led_control_cfg *cfg =
		container_of(led_cdev, struct led_control_cfg, cdev);

	return cfg->cmd.value;
}

static int led_control_probe(struct platform_device *pdev)
{
	struct led_control_device *lcdev;
	struct led_control_cfg *cfg;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	const char *app_name;
	int num_leds;
	int i = 0;
	int ret = 0;

	if (!np)
		return -ENODEV;

	num_leds = of_get_child_count(np);
	if (num_leds == 0)
		return -ENODEV;

	lcdev = devm_kzalloc(&pdev->dev, sizeof(*lcdev), GFP_KERNEL);
	if (!lcdev)
		return -ENOMEM;

	cfg = devm_kzalloc(&pdev->dev, sizeof(*cfg) * num_leds, GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;
	lcdev->cfg = cfg;
	platform_set_drvdata(pdev, lcdev);

	for_each_child_of_node(np, child) {
		of_property_read_string(child, "fb,label", &cfg[i].cdev.name);
		if (!cfg[i].cdev.name)
			continue;
		ret = of_property_read_u8(child, "fb,led-id",
					  &cfg[i].cmd.cmd_id);
		if (ret)
			continue;
		cfg[i].lcdev = lcdev;
		cfg[i].cdev.brightness_set = led_control_set_brightness;
		cfg[i].cdev.brightness_get = led_control_get_brightness;
		ret = led_classdev_register(&pdev->dev, &cfg[i].cdev);
		if (ret)
			dev_err(&pdev->dev, "%s: %s registration failed, err:%d\n",
				__func__, cfg[i].cdev.name, ret);
		i++;
	}
	lcdev->num_leds = i;

	np = of_find_node_by_name(NULL, "fb_privacy");
	if (!np) {
		dev_err(&pdev->dev, "can't find fb_privacy node\n");
		return -ENODEV;
	}

	ret = of_property_read_string(np, "fb,ledapp_name", &app_name);
	if (ret) {
		dev_err(&pdev->dev, "%s: App name not found in DT, err: %d\n",
						__func__, ret);
		return ret;
	}
	strlcpy(ledapp_name, app_name, MAX_NAME_LENGTH);

	return ret;
}

static int led_control_remove(struct platform_device *pdev)
{
	int i, ret = 0;
	struct led_control_device *lcdev = platform_get_drvdata(pdev);

	for (i = 0; i < lcdev->num_leds; i++) {
		led_classdev_unregister(&lcdev->cfg[i].cdev);
		if (ret)
			dev_err(&pdev->dev, "%s: %s unregistration failed, err:%d\n",
				__func__, lcdev->cfg[i].cdev.name, ret);
	}

	if (lcdev->lchandle) {
		ret = qseecom_shutdown_app(&lcdev->lchandle);
		if (ret)
			pr_err("qseecom_shutdown_app failed, err:%d\n", ret);
	}
	return 0;
}

static const struct of_device_id led_control_dt_match[] = {
	{.compatible = "fb,led_control"},
	{},
};

static struct platform_driver led_control_platform_driver = {
	.probe = led_control_probe,
	.remove = led_control_remove,
	.driver = {
		.name = "led_control",
		.owner = THIS_MODULE,
		.of_match_table = led_control_dt_match,
	},
};

module_platform_driver(led_control_platform_driver);

MODULE_DESCRIPTION("Module to communicate with TZ to control camera status LED");
MODULE_LICENSE("GPL v2");
