
/*
 * iaxxx-odsp.h  --  IAXXX odsp header file
 *
 * Copyright 2017 Knowles, Inc.
 *
 * Author: Sharada Kumar <Sharada.Kumar@knowles.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_ODSP_H__
#define __IAXXX_ODSP_H__

struct iaxxx_plugin_info {
	uint32_t plg_idx;
	uint32_t pkg_id;
	uint32_t block_id;
	uint32_t inst_id;
	uint32_t priority;
};

struct iaxxx_plugin_param {
	uint32_t inst_id;
	uint32_t param_id;
	uint32_t param_val;
	uint8_t block_id;

};

struct iaxxx_plugin_param_blk {
	uint32_t inst_id;
	uint32_t param_size;
	uint64_t param_blk;
	uint8_t block_id;
	uint32_t id;
};

struct iaxxx_plugin_create_cfg {
	char file_name[256];
	uint32_t inst_id;
	uint32_t cfg_size;
	uint64_t cfg_val;
	uint8_t block_id;
};

struct iaxxx_set_event {
	uint8_t inst_id;
	uint32_t event_enable_mask;
	uint32_t block_id;
};

struct iaxxx_evt_info {
	uint16_t src_id;
	uint16_t event_id;
	uint16_t dst_id;
	uint32_t dst_opaque;
};

struct iaxxx_get_event {
	uint16_t event_id;
	uint32_t data;
};

struct iaxxx_pkg_mgmt_info {
	char pkg_name[256];
	uint32_t pkg_id;
	uint32_t proc_id;
};

/* IOCTL Magic character */
#define IAXXX_IOCTL_MAGIC 'I'

/* Create IOCTL */
#define ODSP_PLG_CREATE _IO(IAXXX_IOCTL_MAGIC, 0x11)
#define ODSP_PLG_RESET _IO(IAXXX_IOCTL_MAGIC, 0x12)
#define ODSP_PLG_ENABLE _IO(IAXXX_IOCTL_MAGIC, 0x13)
#define ODSP_PLG_DISABLE _IO(IAXXX_IOCTL_MAGIC, 0x14)
#define ODSP_PLG_DESTROY _IO(IAXXX_IOCTL_MAGIC, 0x15)
#define ODSP_PLG_SET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x16)
#define ODSP_PLG_GET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x17)
#define ODSP_PLG_SET_PARAM_BLK _IO(IAXXX_IOCTL_MAGIC, 0x18)
#define ODSP_PLG_SET_CREATE_CFG _IO(IAXXX_IOCTL_MAGIC, 0x19)
#define ODSP_PLG_SET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1A)
#define ODSP_EVENT_SUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1B)
#define ODSP_GET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1C)
#define ODSP_EVENT_UNSUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1D)
#define ODSP_LOAD_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x1E)
#define ODSP_UNLOAD_KW_MODEL _IOW(IAXXX_IOCTL_MAGIC, 0x1F, \
		struct iaxxx_plugin_param_blk)
#define ODSP_START_RECOGNITION _IOW(IAXXX_IOCTL_MAGIC, 0x20, uint32_t)
#define ODSP_STOP_RECOGNITION _IOW(IAXXX_IOCTL_MAGIC, 0x21, uint32_t)
#define ODSP_GET_KW_RECOGNIZE_BITMAP _IOR(IAXXX_IOCTL_MAGIC, 0x22, uint32_t)
#endif
