/*
 * This file is for interface with kernel driver for
 * presence detection using camera.
 *
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 */

#define PD_CAMERA_DEV_NAME	"pd_camera"

#define PD_CAMERA_IOCTL_MAGIC	'P'

enum pd_driver_mode {
	PD_DRIVER_MODE_IRQ,
	PD_DRIVER_MODE_MANUAL,
};


struct pd_get_enable_msg {
	int cur_val;
	int timeout_in_ms;
};

#define PD_CAM_SET_DRIVER_MODE		_IOW(PD_CAMERA_IOCTL_MAGIC, 0x11,\
						enum pd_driver_mode)
#define PD_CAM_SET_CAM_PD_MODE		_IOW(PD_CAMERA_IOCTL_MAGIC, 0x12,\
						int)
#define PD_CAM_PRESENCE_DETECTED	_IO(PD_CAMERA_IOCTL_MAGIC, 0x13)
#define PD_CAM_GET_ENABLE		_IOW(PD_CAMERA_IOCTL_MAGIC, 0x14,\
						struct pd_get_enable_msg)
