
/*
* iaxxx-core.h  --  Knowles ALSA SoC Audio PCM driver header
*
* Copyright 2017 Knowles Corporation.
*
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef __IAXXX_CORE_H__
#define __IAXXX_CORE_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>

typedef int (*iaxxx_cb_func_ptr_t)(struct device *dev);
typedef int (*iaxxx_cb_bc_func_ptr_t)(struct device *dev, u32 iaxxx_spi_speed);
struct workqueue_struct;
/* SRB is at the end of the memory map */
#define IAXXX_RBDT_NUM_ENTRIES	32
#define IAXXX_BLOCK_0 0 /* Update for CM4 PROC */
#define IAXXX_BLOCK_1 1 /* Update for HMD PROC */
#define IAXXX_BLOCK_2 2 /* Update for DMX PROC */
#define IAXXX_MAX_MODELS 3

#define IAXXX_FLG_STARTUP		0
#define IAXXX_FLG_PRIVACY		1
#define IAXXX_FLG_FW_READY		2
#define IAXXX_FLG_FW_CRASH		3
#define IAXXX_FLG_PM_SUSPEND		4
#define IAXXX_FLG_SUSPEND_ON_PRIVACY	5
#define IAXXX_FLG_SPI_BLOCK_CLIENTS	6
#define IAXXX_FLG_SPI_BLOCK_CORE	7
#define IAXXX_FLG_RESUME_BY_STARTUP	8
#define IAXXX_FLG_RESUME_BY_RECOVERY	9
#define IAXXX_FLG_BOOT_COMPLETE		10
#define IAXXX_FLG_DMIC_OFF		11
#define IAXXX_FLG_WAKEUP_EVENT		12
#define IAXXX_FLG_SYSTEM_SUSPEND	13
#define IAXXX_FLG_PM_AWAKE		14

struct iaxxx_client {
	struct regmap *regmap;
};

/* Probed Bus */
enum iaxxx_bus {
	IAXXX_SPI = 0,
	IAXXX_I2C,
	IAXXX_UART,
};

enum iaxxx_events {
	IAXXX_EV_UNKNOWN = 0,		/* Reserve value 0 */
	IAXXX_EV_APP_MODE,		/* FW entered in application mode */
	IAXXX_EV_STARTUP,		/* First ready system startup */
	IAXXX_EV_RECOVERY,		/* Recovery complete after fw crash */
	IAXXX_EV_CRASH,			/* Notify for FW crash */
	IAXXX_EV_ROUTE_ACTIVE,		/* Audio routing path is done */
	IAXXX_EV_PACKAGE,		/* Loaded plugin */
};

enum iaxxx_plugin_state {
	IAXXX_PLUGIN_UNLOADED = 0,
	IAXXX_PLUGIN_LOADED,
	IAXXX_PLUGIN_ACTIVE,
	IAXXX_PLUGIN_RESET,
};

enum iaxxx_pkg_state {
	IAXXX_PKG_UNLOADED = 0,
	IAXXX_PKG_LOADED,
};

struct iaxxx_plug_inst_data {
	int proc_id;
	enum iaxxx_plugin_state plugin_inst_state;
};

struct iaxxx_pkg_data {
	int proc_id;
	enum iaxxx_pkg_state pkg_state;
};

struct iaxxx_block_err {
	uint32_t plg_inst_id[3];
	uint32_t error[3];
};

struct kw_model_info {
	uint32_t kw_loaded_bitmap;
	uint32_t kw_recognize_bitmap;
	uint32_t kw_id[IAXXX_MAX_MODELS];
};

struct iaxxx_system_state {
	struct iaxxx_plug_inst_data plgin[32];
	struct iaxxx_pkg_data pkg[32];
	struct iaxxx_block_err err;
	struct kw_model_info kw_info;
};

enum {
	IAXXX_DBGLOG_CM4,
	IAXXX_DBGLOG_HMD,
	IAXXX_DBGLOG_DMX,
	IAXXX_CRASHLOG_CM4,
	IAXXX_CRASHLOG_HMD,
	IAXXX_CRASHLOG_DMX,
	IAXXX_MAX_LOG
};

struct iaxxx_crashlog_header {
	uint32_t log_type;
	uint32_t log_addr;
	uint32_t log_size;
};

struct iaxxx_crashlog {
	struct iaxxx_crashlog_header header[IAXXX_MAX_LOG];
	char *log_buffer;
	uint32_t log_buffer_size;
	uint32_t logs_read;
};

/**
 * Description of driver private data
 *
 * @dev: device pointer
 * @regmap: the device register map
 * @regmap_config: regmap configuration data
 * @regmap_init: bus specific regmap init function
 * @sys_rbdt: cached copy of the Register Block Descriptor Table
 */
struct iaxxx_priv {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_config *regmap_config;
	int (*regmap_init_bus)(struct iaxxx_priv *priv);
	int (*bulk_read)(struct device *dev, uint32_t address,
				void *buf, size_t len);
	int (*raw_write)(void *context,
			const void *reg,
			 const void *val, size_t val_len);

	uint32_t sys_rbdt[2*IAXXX_RBDT_NUM_ENTRIES];

	/* GPIOs */
	int reset_gpio;
	int event_gpio;
	u32 spi_app_speed;

	/* Regulators */
	struct regulator *vdd_io;
	struct regulator *vdd_core;

	/* External Clock */
	struct clk *ext_clk;

	/* Register map locks */
	struct mutex srb_lock;

	/* Register plugin locks */
	struct mutex plugin_lock;

	/* Event work queue */
	struct mutex event_work_lock;
	struct mutex event_queue_lock;
	struct work_struct event_work_struct;
	struct workqueue_struct *event_workq;
	struct wakeup_source *event_ws;

	/* Core kthread */
	struct task_struct *thread;
	struct kthread_worker worker;
	struct kthread_work fw_update_work;
	struct kthread_work fw_crash_work;
	struct kthread_work runtime_work;
	struct kthread_work park_work;

	void *tunnel_data;
	/* Event Manager */
	struct mutex event_lock;

	enum iaxxx_bus bus;

	/* For factory testing */
	struct work_struct dev_fw_update_test_work;
	struct work_struct dev_cmem_test_work;
	struct completion bootup_done;
	struct completion cmem_done;
	bool test_result;
	iaxxx_cb_bc_func_ptr_t spi_speed_setup;
	iaxxx_cb_func_ptr_t reset_cb;
	uint32_t fw_retry_count;
	struct mutex test_mutex;
	struct iaxxx_evt_queue *event_queue;

	/* Showing mode, boot (0) or app (1) */
	bool is_application_mode;
	bool cm4_crashed;
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	struct iaxxx_raw_bus_ops *raw_ops;
	struct iaxxx_reg_dump_priv *reg_dump;
#endif
	void *intf_priv;
	bool dump_log;

	void *dfs_node;

	/* Notifiers */
	struct srcu_notifier_head core_notifier_list;
	struct notifier_block notifier_core;
	struct notifier_block notifier_pm;

	/* iaxxx core flags for atomic bit field operations */
	unsigned long flags;

	/* Synchronize suspend/resume on this */
	struct iaxxx_system_state *iaxxx_state;
	uint32_t core_crashed;
	uint32_t crash_count;
	bool route_status;
	struct iaxxx_crashlog *crashlog;
	struct mutex crashdump_lock;

	/* FW Crash info */
	int fw_crash_reasons;
	int recovery_try_count;
	int try_count;
};

static inline struct iaxxx_priv *to_iaxxx_priv(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
							int id);
int iaxxx_core_plg_get_param(struct device *dev, uint32_t param_reg,
                                uint32_t param_val_reg, uint32_t param_id,
                                uint32_t *param_val, uint32_t block_id);
int iaxxx_core_plg_set_param(struct device *dev, uint32_t param_reg,
                                uint32_t param_val_reg, uint32_t param_id,
                                uint32_t param_val, uint32_t block_id);
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
				uint32_t priority, uint32_t pkg_id,
				uint32_t plg_idx, uint8_t block_id);
int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
				uint8_t is_enable, uint8_t block_id);
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id);
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
		uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
		char *file);
int iaxxx_core_set_param_blk(struct device *dev, uint32_t inst_id,
			uint32_t blk_size, void *ptr_blk, uint32_t block_id,
			uint32_t slot_id);
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
		uint32_t *data);
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id);
int iaxxx_core_rt_resume(struct device *dev);
int iaxxx_core_rt_suspend(struct device *dev);
int iaxxx_core_dev_resume(struct device *dev);
int iaxxx_core_dev_suspend(struct device *dev);
int iaxxx_package_load(struct device *dev, const char *pkg_name,
					uint32_t pkg_id, uint32_t *proc_id);
int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v);
int iaxxx_unload_kw_model(struct device *dev, uint32_t inst_id,
		uint32_t blk_id, uint32_t id);
int iaxxx_start_recognition(struct device *dev, uint32_t id);
int iaxxx_stop_recognition(struct device *dev, uint32_t id);
int iaxxx_get_kw_recognize_bitmap(struct device *dev, uint32_t *bitmap);
#endif /*__IAXXX_CORE_H__ */
