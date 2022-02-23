/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Luca Hsu <luca_hsu@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 *
 */
#ifndef _ILITEK_COMMON_H_
#define _ILITEK_COMMON_H_
/* Includes of headers ------------------------------------------------------*/
#include <linux/sched.h>
/* Extern define ------------------------------------------------------------*/
//driver information
#define DERVER_VERSION_MAJOR 										5
#define DERVER_VERSION_MINOR 										7
#define CUSTOMER_ID 												0
#define MODULE_ID							3
#define PLATFORM_ID													0
#define PLATFORM_MODULE												0
#define ENGINEER_ID													0
#define ILITEK_TP_MODE_APPLICATION									0x5A
#define ILITEK_TP_MODE_BOOTLOADER									0x55

#define ILITEK_TP_CMD_WRITE_ENABLE									0xC4
#define ILITEK_TP_CMD_WRITE_DATA									0xC3
#define ILITEK_TP_CMD_SLEEP											0x30
#define ILITEK_TP_CMD_GET_TOUCH_INFORMATION							0x10
//error code
#define ILITEK_TP_UPGRADE_FAIL										(-5)
#define ILITEK_I2C_TRANSFER_ERR										(-4)
#define ILITEK_TP_CHANGETOBL_ERR									(-3)
#define ILITEK_TP_CHANGETOAP_ERR									(-2)
#define ILITEK_FAIL                                                 (-1)
#define ILITEK_SUCCESS                                              (0)
#define ILITEK_ERR_LOG_LEVEL 										(1)
#define ILITEK_INFO_LOG_LEVEL 										(3)
#define ILITEK_DEBUG_LOG_LEVEL 										(4)
#define ILITEK_DEFAULT_LOG_LEVEL									(3)

#define ILITEK_KEYINFO_HEADER                                       4
#define ILITEK_KEYINFO_FORMAT_LENGTH                                5
#define ILITEK_KEYINFO_FIRST_PACKET                                 29
#define ILITEK_KEYINFO_OTHER_PACKET                                 25
#define ILITEK_KEYINFO_FORMAT_ID                                    0
#define ILITEK_KEYINFO_FORMAT_X_MSB                                 1
#define ILITEK_KEYINFO_FORMAT_X_LSB                                 2
#define ILITEK_KEYINFO_FORMAT_Y_MSB                                 3
#define ILITEK_KEYINFO_FORMAT_Y_LSB                                 4
//hex parse define
#define HEX_FWVERSION_ADDRESS               						0x0C
#define HEX_FWVERSION_OFFSET					    32
#define HEX_FWVERSION_SIZE                  						8
#define HEX_DATA_FLASH_ADDRESS              						0x22
#define HEX_DATA_FLASH_SIZE                 						3
#define HEX_KERNEL_VERSION_ADDRESS          						0x6
#define HEX_KERNEL_VERSION_SIZE             						6
#define HEX_MEMONY_MAPPING_VERSION_SIZE     						3
#define HEX_MEMONY_MAPPING_VERSION_ADDRESS  						0x0
#define HEX_FLASH_BLOCK_NUMMER_ADDRESS      						80
#define HEX_FLASH_BLOCK_NUMMER_SIZE         						1
#define HEX_FLASH_BLOCK_INFO_ADDRESS        						84
#define HEX_FLASH_BLOCK_INFO_SIZE           						3
#define HEX_FLASH_BLOCK_END_ADDRESS         						123
#define HEX_MEMONY_MAPPING_FLASH_SIZE       						128
#define UPGRADE_LENGTH_BLV1_8               						2048   

#define ILITEK_IOCTL_MAX_TRANSFER									5000
#define ILITEK_HEX_UPGRADE_SIZE										256 * 1024 + 32
#define ILITEK_SUPPORT_MAX_POINT									40
#define REPORT_ADDRESS_COUNT										61
#define REPORT_ADDRESS_ALGO_MODE									62
#define REPORT_ADDRESS_CHECKSUM										63
#define REPORT_ADDRESS_PACKET_ID									0x0
#define REPORT_FORMAT0_PACKET_MAX_POINT								10
#define REPORT_FORMAT0_PACKET_LENGTH								5
#define REPORT_FORMAT1_PACKET_MAX_POINT								10
#define REPORT_FORMAT1_PACKET_LENGTH								6
#define REPORT_FORMAT2_PACKET_MAX_POINT								7
#define REPORT_FORMAT2_PACKET_LENGTH								8
#define REPORT_FORMAT3_PACKET_MAX_POINT								7
#define REPORT_FORMAT3_PACKET_LENGTH								8

#define debug_level(level, fmt, arg...) do {\
	if (level <= ilitek_data->ilitek_log_level_value) {\
		if (level == ILITEK_ERR_LOG_LEVEL) {\
			printk(" %s ERR  line = %d %s : "fmt, "ILITEK", __LINE__, __func__, ##arg);\
		} else if (level == ILITEK_INFO_LOG_LEVEL) {\
			printk(" %s INFO line = %d %s : "fmt, "ILITEK", __LINE__, __func__, ##arg);\
		} else if (level == ILITEK_DEBUG_LOG_LEVEL) {\
			printk(" %s DEBUG line = %d %s : "fmt, "ILITEK", __LINE__, __func__, ##arg);\
		} \
	} \
} while (0)

#define tp_log_err(fmt, arg...) debug_level(ILITEK_ERR_LOG_LEVEL, fmt, ##arg)
#define tp_log_info(fmt, arg...) debug_level(ILITEK_INFO_LOG_LEVEL, fmt, ##arg)
#define tp_log_debug(fmt, arg...) debug_level(ILITEK_DEBUG_LOG_LEVEL, fmt, ##arg)
/* Extern typedef -----------------------------------------------------------*/
struct ilitek_protocol_info {
    uint32_t ver;
    unsigned char ver_major;
    unsigned char ver_mid;
    unsigned char ver_minor;
    uint32_t cmd_count;
};

struct ilitek_touch_info {
	uint16_t id;
	uint16_t x;
	uint16_t y;
	uint16_t p;
	uint16_t w;
	uint16_t h;
	uint16_t status;
};

struct ilitek_key_info {
	int32_t id;
	int32_t x;
	int32_t y;
	int32_t status;
	int32_t flag;
};

struct ilitek_block_info {
	uint32_t start;
	uint32_t end;
	uint16_t ic_crc;
	uint16_t dri_crc;
	bool chk_crc;
};

struct ilitek_panel_info {
	int max_x;
	int min_x;
	int max_y;
	int min_y;
	int blk_num;
	int slave_num;
};

struct ilitek_node_info {
	uint16_t data;
	bool data_st;	//data status
	uint16_t max;
	bool max_st;	//max status
	uint16_t min;
	bool min_st;	//min status
	uint8_t type;	
};

struct ilitek_uniformity_info {
	uint16_t max_thr;
	uint16_t min_thr;	
	uint16_t up_fail;
	uint16_t low_fail;
	uint16_t win1_fail;
	uint16_t win1_thr;
	uint16_t win2_fail;
	uint16_t win2_thr;
	bool bench;
	bool bench_status;
	bool allnode_raw;
	bool allnode_win1;
	bool allnode_win2;
	bool allnode_status;
	bool allnode_raw_status;
	bool allnode_win1_status;
	bool allnode_win2_status;
	struct ilitek_node_info *ben;
	struct ilitek_node_info *raw;
	struct ilitek_node_info *win1;
	struct ilitek_node_info *win2;
};

struct ilitek_ic_info {
	uint8_t mode;
	uint16_t crc;
};

struct ilitek_mp_info {
	bool open;
	uint16_t open_min_thr;
	bool Short;
	uint8_t dump1;
	uint8_t dump2;
	uint16_t short_max_thr;
	bool unifo;
	struct ilitek_uniformity_info uni;
};

struct ilitek_upgrade_info {
    uint8_t filename[256];
    uint32_t hexfilesize;
    uint32_t ap_start_addr;
    uint32_t df_start_addr;
    uint32_t exaddr;
    uint32_t ap_end_addr;
    uint32_t df_end_addr;
    uint32_t ap_check;
    uint32_t df_check;
    uint32_t total_check;
    uint8_t hex_fw_ver[HEX_FWVERSION_SIZE];
    uint8_t hex_ic_type[HEX_KERNEL_VERSION_SIZE];
    bool hex_info_flag;
    bool df_tag_exist;
    uint32_t map_ver;
    uint32_t blk_num;
};

struct ilitek_ts_data {
	int format;
	int (*process_and_report)(void);
    int ilitek_log_level_value;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator *vdd;
	struct regulator *vdd_i2c;
	struct regulator *vcc_io;
	int irq_gpio;
	int reset_gpio;
	int touch_id;
	bool system_suspend;
	uint8_t firmware_ver[8];
	uint8_t mcu_ver[8];
	uint8_t bl_ver[4];
	int upgrade_FW_info_addr;
	uint8_t upgrade_mcu_ver[4];
	int protocol_ver;
	int tp_max_x;
	int tp_max_y;
	int tp_min_x;
	int tp_min_y;
	int screen_max_x;
	int screen_max_y;
	int screen_min_x;
	int screen_min_y;
	int max_tp;
	int max_btn;
	int x_ch;
	int y_ch;
	int keycount;
	int key_xlen;
	int key_ylen;
	struct ilitek_key_info keyinfo[10];
	bool irq_status;
	bool irq_trigger;
	struct task_struct *irq_thread;
	spinlock_t irq_lock;
	bool is_touched;
	bool touch_key_hold_press;
	int touch_flag[ILITEK_SUPPORT_MAX_POINT];
	int slave_count;
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	bool force_update;
	bool has_df;
	int page_number;
	struct task_struct *update_thread;
	bool firmware_updating;
	bool operation_protection;
	bool unhandle_irq;
#ifdef ILITEK_GESTURE
	bool enable_gesture;
#endif

#ifdef ILITEK_ESD_PROTECTION
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
	bool esd_check;
	unsigned long esd_delay;
#endif
	struct kobject *ilitek_func_kobj;
	struct mutex ilitek_mutex;
	bool ilitek_repeat_start;
	bool ilitek_exit_report;
	uint32_t irq_trigger_count;
	uint32_t irq_handle_count;
	struct ilitek_protocol_info ptl;
    struct ilitek_block_info *blk;
    struct ilitek_ic_info *ic;
	struct ilitek_upgrade_info upg;
	int reset_time;	
#ifdef ILITEK_TUNING_NODE
	bool debug_node_open;

	int debug_data_frame;
	wait_queue_head_t inq;
	uint8_t debug_buf[1024][64];
	struct mutex ilitek_debug_mutex;
#endif
	struct ilitek_touch_info tp[ILITEK_SUPPORT_MAX_POINT];
	struct ilitek_mp_info mp;
};
/* Extern macro -------------------------------------------------------------*/
#define CEIL(n, d) ((n % d) ? (n / d) + 1 : (n / d ))
/* Extern variables ---------------------------------------------------------*/
extern uint32_t irq_handle_count;
extern char ilitek_driver_information[];
extern struct ilitek_ts_data *ilitek_data;
/* Extern function prototypes -----------------------------------------------*/
/* Extern functions ---------------------------------------------------------*/
extern void ilitek_resume(void);
extern void ilitek_suspend(void);

extern int ilitek_main_probe(struct ilitek_ts_data *ilitek_data);
extern int ilitek_main_remove(struct ilitek_ts_data *ilitek_data);
extern void ilitek_reset(int delay);
extern int ilitek_poll_int(void);
extern int ilitek_i2c_write(uint8_t *cmd, int length);
extern int ilitek_i2c_read(uint8_t *data, int length);
extern int ilitek_i2c_write_and_read(uint8_t *cmd,
			int write_len, int delay, uint8_t *data, int read_len);

extern void ilitek_irq_enable(void);
extern void ilitek_irq_disable(void);
extern int ilitek_read_tp_info(void);

#ifdef ILITEK_UPDATE_FW
extern int ilitek_boot_upgrade(void);
#endif
extern int ilitek_upgrade_frimware(uint32_t df_startaddr, uint32_t df_endaddr, uint32_t df_checksum,
			uint32_t ap_startaddr, uint32_t ap_endaddr, uint32_t ap_checksum, unsigned char *CTPM_FW);
extern int32_t ilitek_check_busy(int32_t count, int32_t delay, int32_t type);
#ifdef ILITEK_TUNING_MESSAGE
extern bool ilitek_debug_flag;
#endif
#ifdef ILITEK_TOOL
extern int ilitek_create_tool_node(void);
extern int ilitek_remove_tool_node(void);
#endif
extern int32_t ilitek_parse_hex_file(uint32_t *df_startaddr,
			uint32_t *df_endaddr, uint32_t *df_checksum,
			uint32_t *ap_startaddr, uint32_t *ap_endaddr,
			uint32_t *ap_checksum, int32_t hex_len,
			uint8_t *CTPM_FW, uint8_t *save_hex_content);
#endif
