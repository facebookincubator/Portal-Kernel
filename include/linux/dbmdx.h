/*
 * dbmdx-interface.h  --  DBMDX interface definitions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_H
#define _DBMDX_H

#include <linux/device.h>
#include <sound/soc.h>

#define MAX_NUM_OF_INTERFACES 4
#define MAX_NUM_OF_FW_IDS 4
#define MAX_NUM_OF_ADDONS 5
#define MEMSET_CMDS_SIZE 5


enum dbmdx_host_speeds {
	DBMDX_SPEED_PREBOOT = 0,
	DBMDX_SPEED_NORMAL,
	DBMDX_SPEED_BUFFERING,
	DBMDX_SPEED_MAX,
	DBMDX_NR_OF_SPEEDS,
};

struct host_speed {
	u32	cfg;
	u32	uart_baud;
	u32	i2c_rate;
	u32	spi_rate;
};

enum dbmdx_clocks {
	DBMDX_CLK_CONSTANT = 0,
	DBMDX_CLK_MASTER,
	DBMDX_NR_OF_CLKS,
};

enum dbmdx_interface_type {
	DBMDX_PREBOOT_INTERFACE = 0,
	DBMDX_BOOT_INTERFACE,
	DBMDX_CMD_INTERFACE,
	DBMDX_DEBUG_INTERFACE,
	DBMDX_MAX_INTERFACES,
};

struct fw_register {
	u32 cmd;
	u32 priority;
	u32 id;
	u32 reg_num;
	u32 val1;
	u32 val2;
	u32 val3;
};

struct dbmdx_platform_data {
	int				gpio_wakeup;
	int				gpio_reset;
	int				gpio_host_wakeup;
	int				gpio_rx_ready;
	int				wakeup_disabled;
	int				wakeup_set_value;
	int				send_wakeup_seq;
	int				use_gpio_for_wakeup;

	u32				rx_ready_host_wakeup_gpio;


	const char			*cd_interfaces[MAX_NUM_OF_INTERFACES];
	const char			*primary_firmware_name;
	const char			*primary_firmware_name_default;
	const char			*primary_firmware_name_3p;
	const char			*primary_firmware_name_1p;
	const char			*primary_preboot_firmware_name;
	const char			*add_on_fw_names[MAX_NUM_OF_ADDONS];
	int				num_of_fw_addons;
	int				firmware_id[MAX_NUM_OF_FW_IDS];
	int				firmware_id_num;
	int				clock_rates[DBMDX_NR_OF_CLKS];

	int				feature_va;

	struct fw_register		*init_cfg_reg_list;
	u32				init_cfg_reg_list_items;


	int				uart_low_speed_enabled;


	struct host_speed		host_speed_cfg[DBMDX_NR_OF_SPEEDS];

	unsigned int			recovery_disabled;
	u32				multi_interface_support;
	u32				boot_options;
	u32				amodel_options;
	u32				hw_rev;

	int				interfaces[DBMDX_MAX_INTERFACES];
	u16				post_pll_div;
	u32				interface_enabled;
	void				*cdev_pdata;
	u32				memset_params[MEMSET_CMDS_SIZE];
	u32				platform_id;
};

#endif
