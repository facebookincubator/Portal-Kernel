/*
 * dbmdx-interface.h  --  DBMDX interface definitions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_INTERFACE_H
#define _DBMDX_INTERFACE_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/kfifo.h>
#include <linux/dbmdx.h>
#include <sound/dbmdx-export.h>
#include <dt-bindings/sound/dbmdx/dbmdx-fw-interface.h>

#if defined(CONFIG_SND_SOC_DBMDX_COMPAT)
#include "dbmdx-compat.h"
#else
#include <linux/atomic.h>
#endif

#if defined(CONFIG_DBMDX_NO_DTS_SUPPORT)
#if IS_ENABLED(CONFIG_OF)
#undef CONFIG_OF
#endif /* CONFIG_OF */
#endif

#define DRIVER_VERSION				"5.003.6"


#ifndef DBMDX_PRIMARY_FIRMWARE_NAME
#define DBMDX_PRIMARY_FIRMWARE_NAME		"dbmdx_primary_fw.bin"
#endif

#ifndef DBMDX_PRIMARY_PREBOOT_FIRMWARE_NAME
#define DBMDX_PRIMARY_PREBOOT_FIRMWARE_NAME	"dbmdx_primary_preboot_fw.bin"
#endif

#ifndef DBMDX_VT_GRAM_NAME
#define DBMDX_VT_GRAM_NAME			"voice_grammar.bin"
#endif

#ifndef DBMDX_VT_NET_NAME
#define DBMDX_VT_NET_NAME			"voice_net.bin"
#endif

#ifndef DBMDX_VT_AMODEL_NAME
#define DBMDX_VT_AMODEL_NAME			"voice_amodel.bin"
#endif

#ifndef DBMDX_VC_GRAM_NAME
#define DBMDX_VC_GRAM_NAME			"vc_grammar.bin"
#endif

#ifndef DBMDX_VC_NET_NAME
#define DBMDX_VC_NET_NAME			"vc_net.bin"
#endif

#ifndef DBMDX_VC_AMODEL_NAME
#define DBMDX_VC_AMODEL_NAME			"vc_amodel.bin"
#endif

#ifndef DBMDX_VC_OKG_NAME
#define DBMDX_VC_OKG_NAME			"okg_amodel.bin"
#endif

#ifndef DBMDX_ASRP_AEC_PARAM_FILE_NAME
#define DBMDX_ASRP_AEC_PARAM_FILE_NAME		"asrp_params_aecnr.bin"
#endif

#define DBMDX_FW_DBG_STRING_LENGTH		200

#define MAX_REQ_SIZE				8192

#define DBMDX_AMODEL_HEADER_SIZE		12
#define DBMDX_AMODEL_MAX_CHUNKS			3

#define DBMDX_MSLEEP_AFTER_RESET_32K		300

#define DBMDX_USLEEP_RESET_TOGGLE		10000

#define DBMDX_MSLEEP_SPI_D7_AFTER_RESET_32K	85
#define DBMDX_MSLEEP_SPI_WAKEUP			50
#define DBMDX_MSLEEP_REQUEST_FW_FAIL		200
#define DBMDX_MSLEEP_IS_ALIVE			20

#define DBMDX_MSLEEP_RX_ACK_TIMEOUT		500
#define DBMDX_MSLEEP_TASK_CREATE_TIMEOUT	2000
/* Wait time for memset to complete */
#define DBMDX_MSLEEP_MEMSET			100

/* Wait time for usecase setting after firmware load */
#define DBMDX_MSLEEP_USECASE_SET_WAIT		10
#define DBMDX_USLEEP_SPI_D7_AFTER_BOOT		10000
#define DBMDX_USLEEP_SPI_D7_AFTER_RESET		15000
#define DBMDX_USLEEP_SPI_CMD_AFTER_BOOT		1000
#define DBMDX_USLEEP_AFTER_SBOOT_CMD		100
#define DBMDX_USLEEP_BEFORE_CHECKSUM_READ	10000

#define DBMDX_DEFAULT_HW_REV			0x0000

#define DBMDX_BOOT_MODE_NORMAL_BOOT		0x0000
#define DBMDX_BOOT_MODE_RESET_DISABLED		0x0001

#define DBMDX_BOOT_OPT_SEND_PREBOOT		0x0001
#define DBMDX_BOOT_OPT_VA_NO_UART_SYNC		0x0002
#define DBMDX_BOOT_OPT_NO_I2C_FREQ_CALLBACK	0x0004
#define DBMDX_BOOT_OPT_DONT_SENT_SBL		0x0008
#define DBMDX_BOOT_OPT_DONT_SET_PLL		0x0020
#define DBMDX_BOOT_OPT_DONT_CLR_CRC		0x0040
#define DBMDX_BOOT_OPT_DONT_VERIFY_CRC		0x0080
#define DBMDX_BOOT_OPT_DONT_SEND_START_BOOT	0x0100
#define DBMDX_BOOT_OPT_VERIFY_CHIP_ID		0x0200
#define DBMDX_BOOT_OPT_DONT_SEND_INIT_LOCK_CMD	0x0400
#define DBMDX_BOOT_OPT_FW_CONTAINS_CHECKSUM	0x0800
#define DBMDX_BOOT_OPT_USE_REV2_BOOT_CMD	0x1000
#define DBMDX_BOOT_OPT_SEND_FW_BY_COMMAND	0x2000


#define DBMDX_AMODEL_DEFAULT_OPTIONS		0x0000
#define DBMDX_AMODEL_INCLUDES_HEADERS		0x0001
#define DBMDX_AMODEL_SVT_ENCODING		0x0002
#define DBMDX_AMODEL_SINGLE_FILE_NO_HEADER	0x0004

#define DBMDX_FIRMWARE_ID_DBMD2			0xdbd2
#define DBMDX_FIRMWARE_ID_DBMD4			0xdbd4
#define DBMDX_FIRMWARE_ID_DBMD6			0xdbd6
#define DBMDX_FIRMWARE_ID_DBMD7_0		0xdb70
#define DBMDX_FIRMWARE_ID_DBMD7_1		0xdb71
#define DBMDX_FIRMWARE_ID_DBMD8			0xdbd8

#define BOOT_MEMSET_START_ADDR			0x00400000
#define BOOT_MEMSET_END_ADDR			0x007CFFFF
#define BOOT_MEMSET_CMD_OPCODE_6		0xDBD70306
#define BOOT_MEMSET_CMD_OPCODE_7		0xDBD70607

#define DBMDX_MAX_FW_CMD_LENGTH			8192
#define DBMDX_MAX_DUMP_LENGTH			512
#define MAX_NUM_OF_PENDING_FW_MSG		32
#define MB_DATA_CHUNK				4096
#define MB_BUFFER_CHUNK				4096

#define HOST_TDM_RX_MB_IDX		MB_INDEX_TDM_0_RX_ID
#define HOST_TDM_TX_MB_IDX		MB_INDEX_TDM_0_TX_ID

#ifdef REV_A_BOARD_CONFIG
#define CODEC_TDM_RX_IDX		1
#define CODEC_TDM_TX_IDX		1
#define CODEC_TDM_RX_MB_IDX		MB_INDEX_TDM_1_RX_ID
#define CODEC_TDM_TX_MB_IDX		MB_INDEX_TDM_1_TX_ID
#else
#define CODEC_TDM_RX_IDX		3
#define CODEC_TDM_TX_IDX		3
#define CODEC_TDM_RX_MB_IDX		MB_INDEX_TDM_3_RX_ID
#define CODEC_TDM_TX_MB_IDX		MB_INDEX_TDM_3_TX_ID
#endif

#define HOST_TDM_RX_IDX			0
#define HOST_TDM_TX_IDX			0


#define CODEC_TDM_RX_BLK_NUM		1
#define CODEC_TDM_TX_BLK_NUM		1

#define HOST_TDM_RX_BLK_NUM		0
#define HOST_TDM_TX_BLK_NUM		0

#define DBMDX_DEFAULT_RX_READY_HOST_WAKEUP_GPIO_VAL 0xd2d3

#define LOGS_CORE_LP_MB_ID			0x8013
#define LOGS_CORE_HF1_MB_ID			0x8012
#define LOGS_CORE_HF0_MB_ID			0x8011


struct chip_interface;

enum dbmdx_firmware_active {
	/* firmware pre-boot */
	DBMDX_FW_PRE_BOOT = 0,
	/* firmware was powered off */
	DBMDX_FW_POWER_OFF,
	/* primary firmware is active */
	DBMDX_FW_PRIMARY,
	/* max supported firmwares */
	DBMDX_FW_MAX
};


enum dbmdx_audio_channel_operation {
	AUDIO_CHANNEL_OP_COPY = 0,
	AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2,
	AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4,
	AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4,
	AUDIO_CHANNEL_OP_TRUNCATE_2_TO_1,
	AUDIO_CHANNEL_OP_TRUNCATE_4_TO_1,
	AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2,
	AUDIO_CHANNEL_OP_MAX = AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2
};

enum dbmdx_chip {
	DBMDX_CHIP_PRIMARY = 0,
	DBMDX_NR_OF_CHIP_TYPES,
};


struct primary_flags {
	int		rx_irq_inuse;
	int		host_wakeup_irq_inuse;
	atomic_t	rx_ack;
	atomic_t	task_create_in_process;
	bool		task_waiting_for_task_handle;
	bool		task_waiting_for_queue_id;
	u32		current_task_handle;
	u32		current_queue_id;
	struct usecase_config *active_usecase;
	struct usecase_config *user_selected_usecase;
	unsigned int	active_usecase_id;
	const char	*last_loaded_asrp_params_file_name;
	unsigned int	idle_usecase_active;
	bool		logs_enabled;
};

enum dbmdx_load_amodel_mode {
	LOAD_AMODEL_PRIMARY = 0,
	LOAD_AMODEL_2NDARY = 1,
	LOAD_AMODEL_CUSTOM,
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	LOAD_AMODEL_OKG = 4,
	LOAD_AMODEL_MAX = LOAD_AMODEL_OKG
#else
	LOAD_AMODEL_MAX = LOAD_AMODEL_CUSTOM
#endif
};

enum dbmdx_states {
	DBMDX_IDLE = 0,
	DBMDX_DETECTION = 1,
	DBMDX_BUFFERING = 2,
	DBMDX_UART_RECORDING = 3,
	DBMDX_SLEEP_PLL_ON = 4,
	DBMDX_SLEEP_PLL_OFF = 5,
	DBMDX_HIBERNATE = 6,
	DBMDX_STREAMING = 7,	/* CUSTOM STATE */
	DBMDX_DETECTION_AND_STREAMING = 8,	/* CUSTOM STATE */
	DBMDX_NR_OF_STATES,
};

enum dbmdx_bus_interface {
	DBMDX_INTERFACE_NONE = 0,
	DBMDX_INTERFACE_I2C,
	DBMDX_INTERFACE_SPI,
	DBMDX_INTERFACE_UART,
	DBMDX_NR_OF_INTERFACES,
};

enum dbmdx_notify_events {
	DBMDX_EV_NOTHING = 0,		/* Reserve value */
	DBMDX_EV_STARTUP,		/* First ready system startup */
	DBMDX_EV_RECOVERY,		/* Recovery complete after fw crash */
	DBMDX_EV_CRASH,			/* Notify for FW crash */
	DBMDX_EV_MESSAGE,		/* Received message from chip */
	DBMDX_DRV_MESSAGE,		/* notify across dbmdx sub module drivers */
};

enum dbmdx_power_modes {
	/*
	 * no firmware is active and the device is booting
	 */
	DBMDX_PM_BOOTING = 0,
	/*
	 * a firmware is active and the device can be used
	 */
	DBMDX_PM_ACTIVE,
	/*
	 * a firmware is active and the device is going to sleep
	 */
	DBMDX_PM_FALLING_ASLEEP,
	/*
	 * chip is sleeping and needs to be woken up to be functional
	 */
	DBMDX_PM_SLEEPING,
	/* number of PM states */
	DBMDX_PM_STATES,
};

enum dbmdx_dbg_dump_level {
	DBMDX_DBG_DUMP_LEVEL_NO_DUMP = 0,
	DBMDX_DBG_DUMP_LEVEL_MSG_DUMP = 1,
	DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP = 2,
};

enum dbmdx_drv_messages {
	DBMDX_MIXER_CTRL_MODIFIED,
};

struct dbmdx_private {
	struct dbmdx_platform_data		*pdata;
	/* lock for private data */
	struct mutex			p_lock;
	enum dbmdx_firmware_active	active_fw;
	enum dbmdx_firmware_active	active_fw_chip;
	enum dbmdx_bus_interface	active_interface;
	enum dbmdx_interface_type	active_interface_type_primary;
	enum dbmdx_chip			active_chip;
	enum dbmdx_chip			usr_selected_chip;
	int				host_wakeup_irq;
	int				rx_irq;
	struct platform_device		*pdev;
	struct device			*dev;

	const struct firmware		*primary_fw;
	const struct firmware		*primary_preboot_fw;
	const struct firmware		*addon_fw[MAX_NUM_OF_ADDONS];
	int				num_of_loaded_fw_addons;

	bool				asleep;
	bool				device_ready;
	bool				fw_alive;
	bool				fw_crash;
	bool				do_fw_compl;
	bool				do_suspend;
	bool				mfd_devices;
	struct clk			*clocks[DBMDX_NR_OF_CLKS];
	struct regulator		*vregulator;
	unsigned int			clk_type;
	struct primary_flags		primary_flags;
	bool				sleep_disabled;
	int				debug_mode;
	int				dump_traffic;
	u8				*fw_cmd_buffer;
	u32				chip_revision;


	int				cur_reset_gpio;
	int				cur_wakeup_gpio;
	int				cur_wakeup_disabled;
	int				cur_wakeup_set_value;
	int				cur_send_wakeup_seq;
	int				cur_use_gpio_for_wakeup;
	int				cur_firmware_id[MAX_NUM_OF_FW_IDS];
	int				cur_firmware_id_num;
	u32				cur_boot_options;

	u32				boot_mode;
	u32				hw_usecase_stop_enabled;

	struct task_struct		*task_workq;
	struct task_struct		*task_irq;
	struct workqueue_struct		*dbmdx_workq;
	struct delayed_work		fw_state_work;
	struct delayed_work		fw_reload_work;
	struct completion		fw_load_complete;
	wait_queue_head_t		rx_ack_wq;
	wait_queue_head_t		host_event_wq;

	/* limit request size of audio data from the firmware */
	unsigned long				rxsize;

	/* sysfs */
	struct class				*ns_class;
	struct device				*dbmdx_dev;
	/* common helper functions */
	void (*reset_set)(struct dbmdx_private *p);
	void (*reset_release)(struct dbmdx_private *p);
	void (*reset_sequence)(struct dbmdx_private *p);
	void (*wakeup_set)(struct dbmdx_private *p);
	void (*wakeup_release)(struct dbmdx_private *p);
	int (*can_wakeup)(struct dbmdx_private *p);
	void (*lock)(struct dbmdx_private *p);
	void (*unlock)(struct dbmdx_private *p);
	int (*verify_checksum)(struct dbmdx_private *p,
			       const u8 *expect, const u8 *got, size_t size);
	unsigned long (*clk_get_rate)(struct dbmdx_private *p,
				      enum dbmdx_clocks clk);
	long (*clk_set_rate)(struct dbmdx_private *p,
			     enum dbmdx_clocks clk);
	int (*clk_enable)(struct dbmdx_private *p, enum dbmdx_clocks clk);
	int (*clk_disable)(struct dbmdx_private *p, enum dbmdx_clocks clk);

	event_cb		event_callback;

	/* interface to the chip */
	struct chip_interface			*chip;
	struct chip_interface		**interfaces;
	enum dbmdx_bus_interface	*interface_types;
	unsigned int			nr_of_interfaces;
	void *interface_data;
	void *cdev_data;

	/* Notifiers */
	struct srcu_notifier_head notifier_list;
	struct notifier_block nb_core;

	/* capture algorithm */
	bool algo_enabled;
	int cur_firmware;
	/* manual crash triggered, avoid events */
	bool disable_log_event;
};

/*
 * main interface between the core layer and the chip
 */
struct chip_interface {
	/* wait till booting is allowed */
	int (*can_boot)(struct dbmdx_private *p);
	/* prepare prebooting */
	int (*prepare_preboot)(struct dbmdx_private *p);
	/* prepare booting (e.g. increase speed) */
	int (*prepare_boot)(struct dbmdx_private *p);
	/* send firmware to the chip and boot it */
	int (*boot)(struct dbmdx_private *p, const void *fw_data[],
			size_t fw_size[], const void *checksum,
			unsigned int num_of_fw_files,
			size_t chksum_len, int load_fw);
	/* finish booting */
	int (*finish_boot)(struct dbmdx_private *p);
	/* dump chip state */
	int (*dump)(struct dbmdx_private *p, char *buf);
	/* set VA firmware ready, (e.g. lower speed) */
	int (*set_primary_firmware_ready)(struct dbmdx_private *p);
	/* Enable/Disable Transport layer (UART/I2C/SPI) */
	void (*transport_enable)(struct dbmdx_private *p, bool enable);
	/* read data from the chip */
	ssize_t (*read)(struct dbmdx_private *p, void *buf, size_t len);
	/* write data to the chip */
	ssize_t (*write)(struct dbmdx_private *p, const void *buf, size_t len);
	/* verify boot checksum */
	int (*verify_boot_checksum)(struct dbmdx_private *p,
	const void *checksum, size_t chksum_len);
	/* prepare buffering of audio data (e.g. increase speed) */
	int (*prepare_buffering)(struct dbmdx_private *p);
	/* read audio data */
	int (*read_audio_data)(struct dbmdx_private *p,
		void *buf,
		size_t samples,
		bool to_read_metadata,
		size_t *available_samples,
		size_t *data_offset);
	/* finish buffering of audio data (e.g. lower speed) */
	int (*finish_buffering)(struct dbmdx_private *p);
	/* prepare amodel loading (e.g. increase speed) */
	int (*prepare_amodel_loading)(struct dbmdx_private *p);
	/* load acoustic model */
	int (*load_amodel)(struct dbmdx_private *p,  const void *data,
			   size_t size, int num_of_chunks, size_t *chunk_sizes,
			   const void *checksum, size_t chksum_len,
			   enum dbmdx_load_amodel_mode load_amodel_mode);
	/* finish amodel loading (e.g. lower speed) */
	int (*finish_amodel_loading)(struct dbmdx_private *p);
	/* Get Read Chunk Size */
	u32 (*get_read_chunk_size)(struct dbmdx_private *p);
	/* Get Write Chunk Size */
	u32 (*get_write_chunk_size)(struct dbmdx_private *p);
	/* Set Read Chunk Size */
	int (*set_read_chunk_size)(struct dbmdx_private *p, u32 size);
	/* Set Write Chunk Size */
	int (*set_write_chunk_size)(struct dbmdx_private *p, u32 size);
	/* Resume Chip Interface */
	void (*resume)(struct dbmdx_private *p);
	/* Suspend Chip Interface */
	void (*suspend)(struct dbmdx_private *p);
	/* Return true if there is leading byte before each read operation */
	bool (*interface_adds_leading_byte_on_read)(struct dbmdx_private *p);
	/* private data */
	void *pdata;
};

/* dbmdx inter driver communication message */
struct drv_message {
	u32	msg_id;
	void	*msg;
};

struct mixer_ctl {
	u32 reg;
	u32 val;
};

#define DBMDX_CORE_DEV(dev) \
	((dev)->class ? (dev) : ((dev)->type ? (dev)->parent : (dev)))
#define DBMDX_PRIV(dev) \
	((struct dbmdx_private *)dev_get_drvdata(DBMDX_CORE_DEV(dev)))

bool dbmdx_core_is_ready(struct dbmdx_private *p);
int dbmdx_request_and_load_fw(struct dbmdx_private *p);
int dbmdx_device_init(struct device *dev);
int dbmdx_device_exit(struct device *dev);
void dbmdx_send_uevent(struct dbmdx_private *p, char *buf);
int dbmdx_schedule_work(struct dbmdx_private *p, struct work_struct *work);
int dbmdx_schedule_delayed_work(struct dbmdx_private *p,
				struct delayed_work *work, unsigned long delay);
int dbmdx_cell_force_suspend(struct device *dev, void *data);
int dbmdx_cell_force_resume(struct device *dev, void *data);
int dbmdx_notifier_register(struct device *dev, struct notifier_block *nb);
int dbmdx_notifier_unregister(struct device *dev, struct notifier_block *nb);
int dbmdx_notifier_call(struct device *dev, unsigned long val, void *v);
int dbmdx_core_suspend(struct dbmdx_private *p);
int dbmdx_core_resume(struct dbmdx_private *p);
int dbmdx_force_crash_fw(struct device *dev, int disable_log_events);
int dbmdx_event_log_int(struct device *dev, int32_t type, int value);

#endif /* _DBMDX_INTERFACE_H */
