/*
 * siw_touch_hal_prd.c - SiW touch hal driver for PRD
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Sungyeal Park <parksy5@siliconworks.co.kr>
 *         Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "siw_touch_cfg.h"

#if defined(__SIW_SUPPORT_PRD)	//See siw_touch_cfg.h

#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/linkage.h>
#include <linux/syscalls.h>
#include <linux/namei.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/fs.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"
#include "siw_touch_event.h"
#include "siw_touch_gpio.h"
#include "siw_touch_irq.h"
#include "siw_touch_sys.h"

/*
 * [PRD]
 * Touch IC Production Test
 * This supports a method to verify the quality of Touch IC using input/output file test.
 *
 * 1. Input setup file(spec file)
 * 2. Self testing
 * 3. Output result file(self test result)
 * 4. Verify the result
 *
 */

#define __SIW_SUPPORT_PRD_SET_SD

#if defined(__SIW_SUPPORT_PRD_SET_SD)
//#define __SIW_SUPPORT_PRD_SET_SD_ONLY
#endif

enum {
	PRD_DATA_NAME_SZ	= 128,
	/* */
	PRD_LINE_NUM		= (1<<10),
//	PRD_PATH_SIZE		= (1<<6),		//64
//	PRD_BURST_SIZE		= (1<<9),		//512
	/* */
	MAX_LOG_FILE_COUNT	= (4),
	MAX_LOG_FILE_SIZE	= (10 * (1<<20)),	/* 10M byte */
	/* */
	MAX_TEST_CNT			= 2,
};

enum {
	PRD_RAWDATA_SZ_POW	= 1,
	PRD_RAWDATA_SIZE	= (1<<PRD_RAWDATA_SZ_POW),
	/* */
	PRD_M1_COL_SIZE		= (1<<1),
	/* */
	PRD_LOG_BUF_SIZE	= (1<<10),	//1K
	PRD_BUF_SIZE		= (8<<10),	//8K fixed, PAGE_SIZE can be adjust by kernel
	/* */
	PRD_DEBUG_BUF_SIZE	= (336),
	/* */
	PRD_BUF_DUMMY		= 128,		//dummy for avoiding memory panic
	PRD_APP_INFO_SIZE	= 32,
};

enum {
	IMG_OFFSET_IDX_NONE = 0,
	IMG_OFFSET_IDX_RAW,
	IMG_OFFSET_IDX_BASELINE_EVEN,
	IMG_OFFSET_IDX_BASELINE_ODD,
	IMG_OFFSET_IDX_DELTA,
	IMG_OFFSET_IDX_LABEL,
	IMG_OFFSET_IDX_F_DELTA,
	IMG_OFFSET_IDX_DEBUG,
	IMG_OFFSET_IDX_MAX,
};

enum {
	TIME_INFO_SKIP = 0,
	TIME_INFO_WRITE,
};

enum {
	/* need to tune test */
	U3_M2_RAWDATA_TEST = 0,
	U3_M1_RAWDATA_TEST,
	U0_M2_RAWDATA_TEST,
	U0_M1_RAWDATA_TEST,
	/* */
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	U3_BLU_JITTER_TEST,
	/* */
	U3_JITTER_TEST,
	U0_JITTER_TEST,
	UX_INVALID,

	/* for sd_test_flag */
	OPEN_SHORT_RESULT_DATA_IDX = 24,
	OPEN_SHORT_RESULT_RAWDATA_IDX,

	U0_JITTER_M1,
};

enum {
	U3_TEST_PRE_CMD = 0x3,
	U0_TEST_PRE_CMD = 0x0,
};

enum {
	NO_TEST_POST_CMD = 0,
	OPEN_SHORT_ALL_TEST_POST_CMD,
	OPEN_NODE_TEST_POST_CMD,
	SHORT_NODE_TEST_POST_CMD,
	M2_RAWDATA_TEST_POST_CMD = 5,
	M1_RAWDATA_TEST_POST_CMD,
	JITTER_TEST_POST_CMD = 12,
};

enum {
	LINE_FILTER_OPTION	= (0x40000),
};

enum {
	U3_M2_RAWDATA_TEST_FLAG		= (1<<U3_M2_RAWDATA_TEST),
	U3_M1_RAWDATA_TEST_FLAG		= (1<<U3_M1_RAWDATA_TEST),
	U0_M2_RAWDATA_TEST_FLAG		= (1<<U0_M2_RAWDATA_TEST),
	U0_M1_RAWDATA_TEST_FLAG 	= (1<<U0_M1_RAWDATA_TEST),
	/* */
	OPEN_SHORT_NODE_TEST_FLAG	= (1<<OPEN_SHORT_ALL_TEST),
	U3_BLU_JITTER_TEST_FLAG 	= (1<<U3_BLU_JITTER_TEST),
	U3_JITTER_TEST_FLAG			= (1<<U3_JITTER_TEST),
	U0_JITTER_TEST_FLAG			= (1<<U0_JITTER_TEST),
	/* */
	OPEN_SHORT_RESULT_DATA_FLAG		= (1<<OPEN_SHORT_RESULT_DATA_IDX),
	OPEN_SHORT_RESULT_RAWDATA_FLAG	= (1<<OPEN_SHORT_RESULT_RAWDATA_IDX),

	U0_JITTER_M1_FLAG			= (1<<U0_JITTER_M1),
};

enum {
	/*
	 * [Caution]
	 * Do not touch this ordering
	 */
	PRD_SYS_EN_IDX_SD = 0,
	PRD_SYS_EN_IDX_DELTA,
	PRD_SYS_EN_IDX_LABEL,
	PRD_SYS_EN_IDX_RAWDATA_PRD,
	//
	PRD_SYS_EN_IDX_RAWDATA_TCM,	//4
	PRD_SYS_EN_IDX_RAWDATA_AIT,
	PRD_SYS_EN_IDX_BASE,
	PRD_SYS_EN_IDX_DEBUG_BUF,
	//
	PRD_SYS_EN_IDX_LPWG_SD,		//8
	PRD_SYS_EN_IDX_FILE_TEST,
	PRD_SYS_EN_IDX_APP_RAW,
	PRD_SYS_EN_IDX_APP_BASE,
	//
	PRD_SYS_EN_IDX_APP_LABEL,	//12
	PRD_SYS_EN_IDX_APP_DELTA,
	PRD_SYS_EN_IDX_APP_DEBUG_BUF,
	PRD_SYS_EN_IDX_APP_END,
	//
	PRD_SYS_EN_IDX_APP_INFO,	//16
	//
	PRD_SYS_ATTR_MAX,
};

enum {
	PRD_SYS_EN_SD					= (1<<PRD_SYS_EN_IDX_SD),
	PRD_SYS_EN_DELTA				= (1<<PRD_SYS_EN_IDX_DELTA),
	PRD_SYS_EN_LABEL				= (1<<PRD_SYS_EN_IDX_LABEL),
	PRD_SYS_EN_RAWDATA_PRD			= (1<<PRD_SYS_EN_IDX_RAWDATA_PRD),
	//
	PRD_SYS_EN_RAWDATA_TCM			= (1<<PRD_SYS_EN_IDX_RAWDATA_TCM),
	PRD_SYS_EN_RAWDATA_AIT			= (1<<PRD_SYS_EN_IDX_RAWDATA_AIT),
	PRD_SYS_EN_BASE					= (1<<PRD_SYS_EN_IDX_BASE),
	PRD_SYS_EN_DEBUG_BUF			= (1<<PRD_SYS_EN_IDX_DEBUG_BUF),
	//
	PRD_SYS_EN_LPWG_SD				= (1<<PRD_SYS_EN_IDX_LPWG_SD),
	PRD_SYS_EN_FILE_TEST			= (1<<PRD_SYS_EN_IDX_FILE_TEST),
	PRD_SYS_EN_APP_RAW				= (1<<PRD_SYS_EN_IDX_APP_RAW),
	PRD_SYS_EN_APP_BASE				= (1<<PRD_SYS_EN_IDX_APP_BASE),
	//
	PRD_SYS_EN_APP_LABEL			= (1<<PRD_SYS_EN_IDX_APP_LABEL),
	PRD_SYS_EN_APP_DELTA			= (1<<PRD_SYS_EN_IDX_APP_DELTA),
	PRD_SYS_EN_APP_DEBUG_BUF		= (1<<PRD_SYS_EN_IDX_APP_DEBUG_BUF),
	PRD_SYS_EN_APP_END				= (1<<PRD_SYS_EN_IDX_APP_END),
	//
	PRD_SYS_EN_APP_INFO				= (1<<PRD_SYS_EN_IDX_APP_INFO),
};

#define PRD_SYS_ATTR_EN_FLAG 		(0 |	\
									PRD_SYS_EN_SD |	\
									PRD_SYS_EN_DELTA |	\
									PRD_SYS_EN_LABEL |	\
									PRD_SYS_EN_RAWDATA_PRD |	\
									PRD_SYS_EN_RAWDATA_TCM |	\
									PRD_SYS_EN_RAWDATA_AIT |	\
									PRD_SYS_EN_BASE |	\
									PRD_SYS_EN_DEBUG_BUF |	\
									PRD_SYS_EN_LPWG_SD |	\
									PRD_SYS_EN_FILE_TEST |	\
									PRD_SYS_EN_APP_RAW |	\
									PRD_SYS_EN_APP_BASE |	\
									PRD_SYS_EN_APP_LABEL |	\
									PRD_SYS_EN_APP_DELTA | 	\
									PRD_SYS_EN_APP_DEBUG_BUF |	\
									PRD_SYS_EN_APP_END |	\
									PRD_SYS_EN_APP_INFO |	\
									0)

struct siw_hal_prd_img_cmd {
	u32 raw;
	u32 baseline_even;
	u32 baseline_odd;
	u32 delta;
	u32 label;
	u32 f_delta;
	u32 debug;
};

struct siw_hal_prd_img_offset {
	u32 raw;
	u32 baseline_even;
	u32 baseline_odd;
	u32 delta;
	u32 label;
	u32 f_delta;
	u32 debug;
};

struct siw_hal_prd_param {
	int chip_type;
	const char **name;	/* to make group */
	//
	u32 cmd_type;
	u32 addr[IMG_OFFSET_IDX_MAX];
	//
	u32 row;
	u32 col;
	u32 col_add;
	u32 ch;
	u32 m1_col;
	u32 m1_cnt;
	u32 m2_cnt;
	//
	struct siw_touch_second_screen second_scr;
	//
	u32 sysfs_off_flag;
	//
	u32 sd_test_flag;
	u32 lpwg_sd_test_flag;
};

struct siw_hal_prd_ctrl {
	u32 m2_row_col_size;
	u32 m2_row_col_buf_size;
	u32 m1_row_col_size;
	//
	u32 m2_frame_size;
	u32 m1_frame_size;
	//
	u32 delta_size;
	//
	u32 label_tmp_size;
	u32 debug_buf_size;
};

struct siw_hal_prd_tune {
	u32 ch;
	u32 code_size;
	//
	u32 code_l_goft_offset;
	u32 code_l_m1_oft_offset;
	u32 code_l_g1_oft_offset;
	u32 code_l_g2_oft_offset;
	u32 code_l_g3_oft_offset;
	//
	u32 code_r_goft_offset;
	u32 code_r_m1_oft_offset;
	u32 code_r_g1_oft_offset;
	u32 code_r_g2_oft_offset;
	u32 code_r_g3_oft_offset;
};

struct siw_hal_prd_sd_cmd {
	u32 cmd_open_node;
	u32 cmd_short_node;
	u32 cmd_m2_rawdata;
	u32 cmd_m1_rawdata;
	u32 cmd_jitter;
	u32 cmd_u0_jitter;
};

enum _SIW_PRD_DBG_MASK_FLAG {
	PRD_DBG_NONE				= 0,
	PRD_DBG_OPEN_SHORT_DATA		= (1U<<0),
};

#if defined(__SIW_SUPPORT_PRD_SET_SD)
struct siw_hal_prd_sd_param {
	int u3_m2[2];	/* [0] = lower, [1] = upper */
	int u0_m2[2];
	int u0_m1[2];
	int u3_blu_jitter[2];
	int u3_jitter[2];
	int u0_jitter[2];
	int last_type;
	int last_val[2];
};
#endif	/* __SIW_SUPPORT_PRD_SET_SD */

struct siw_hal_prd_data {
	struct device *dev;
	char name[PRD_DATA_NAME_SZ];
	struct siw_hal_prd_sd_cmd sd_cmd;
	u32 dbg_mask;
	/* */
	struct siw_hal_prd_param param;
	struct siw_hal_prd_ctrl ctrl;
	struct siw_hal_prd_tune tune;
	/* */
	struct siw_hal_prd_img_cmd img_cmd;
	struct siw_hal_prd_img_offset img_offset;
	/* */
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param sd_param;
#endif
	/* */
	int prd_app_mode;
	/* */
	u8 *buf_src;
	int buf_size;
	int16_t	*m2_buf_even_rawdata;
	int16_t	*m2_buf_odd_rawdata;
	/* */
	int16_t	*m1_buf_even_rawdata;
	int16_t	*m1_buf_odd_rawdata;
	int16_t *m1_buf_tmp;
	/* */
	int16_t *open_buf_result_rawdata;
	int16_t	*short_buf_result_rawdata;
	int16_t	*open_buf_result_data;
	int16_t	*short_buf_result_data;
	/* */
	int image_lower;
	int image_upper;
	/* */
	int16_t	*buf_delta;
	int16_t	*buf_debug;
	u8	*buf_label_tmp;
	u8	*buf_label;
	/* */
	int sysfs_flag;
	int sysfs_done;
	/* */
	int mon_flag;
	/* */
	char log_buf[PRD_LOG_BUF_SIZE + PRD_BUF_DUMMY];
	char line[PRD_LINE_NUM + PRD_BUF_DUMMY];
	char buf_write[PRD_BUF_SIZE + PRD_BUF_DUMMY];
};

enum {
	PRD_CMD_TYPE_1 = 0,		//new type: base only
	PRD_CMD_TYPE_2 = 1,		//old type: base_even, base_odd
};

#define PRD_OFFSET_QUIRK_SET(_idx, _offset)		(((_idx)<<24) | ((_offset) & 0x00FFFFFF))
#define PRD_OFFSET_QUIRK_GET_IDX(_addr)			((_addr) >> 24)
#define PRD_OFFSET_QUIRK_GET_OFFSET(_addr)		((_addr) & 0x00FFFFFF)

#define __PRD_PARAM_DIMENSION(_row, _col, _col_add, _ch, _m1_col, _m1_cnt, _m2_cnt)	\
		.row = (_row), .col = (_col), .col_add = (_col_add), .ch = (_ch),	\
		.m1_col = (_m1_col), .m1_cnt = (_m1_cnt), .m2_cnt = (_m2_cnt)

#define __PRD_2ND_SCR(_bound_i, _bound_j)	\
		.second_scr = { _bound_i, _bound_j }

static const char *prd_param_name_lg4894_k[] = {
	"L0W53K6P", NULL
};

static const char *prd_param_name_lg4894_lv[] = {
	"L0W53LV5", "L0W50LV3", NULL
};

static const char *prd_param_name_lg4894_sf[] = {
	"L0W53SF3", NULL
};

static const char *prd_param_name_lg4895_k[] = {
	"LPW49K5", NULL
};

static const char *prd_param_name_lg4946_g[] = {
	"L0L53P1", "L0W53P1", NULL
};

enum {
	SD_FLAG_LG4894 		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_JITTER_TEST_FLAG |	\
							U3_BLU_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_DATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4894 = 	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							U0_JITTER_TEST_FLAG |	\
							0),
};

enum {
	SD_FLAG_LG4895		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_BLU_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_DATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4895 =	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							0),
};

enum {
	SD_FLAG_LG4946		=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							OPEN_SHORT_RESULT_RAWDATA_FLAG |	\
							0),
	LPWG_SD_FLAG_LG4946 = 	(0 |	\
							U0_M2_RAWDATA_TEST_FLAG |	\
							U0_M1_RAWDATA_TEST_FLAG |	\
							0),
};

enum {
	__SD_FLAG_SW49XXX	=	(0	|	\
							U3_M2_RAWDATA_TEST_FLAG |	\
							OPEN_SHORT_NODE_TEST_FLAG |	\
							U3_JITTER_TEST_FLAG |	\
							OPEN_SHORT_RESULT_RAWDATA_FLAG |	\
							0),

	__LPWG_SD_FLAG_SW49XXX	=	(0 |	\
								U0_M2_RAWDATA_TEST_FLAG |	\
								U0_M1_RAWDATA_TEST_FLAG |	\
								U0_JITTER_TEST_FLAG |	\
								0),

	SD_FLAG_SW49105			= __SD_FLAG_SW49XXX | OPEN_SHORT_RESULT_DATA_FLAG,
	LPWG_SD_FLAG_SW49105	= __LPWG_SD_FLAG_SW49XXX,

	SD_FLAG_SW49406			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49406	= __LPWG_SD_FLAG_SW49XXX,

	SD_FLAG_SW49407			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49407	= __LPWG_SD_FLAG_SW49XXX | U0_JITTER_M1_FLAG,

	SD_FLAG_SW49408			= __SD_FLAG_SW49XXX,
	LPWG_SD_FLAG_SW49408	= __LPWG_SD_FLAG_SW49XXX,
};

enum {
	SD_FLAG_SW1828			=	(0	|	\
								U3_M2_RAWDATA_TEST_FLAG |	\
								OPEN_SHORT_NODE_TEST_FLAG |	\
								U3_JITTER_TEST_FLAG |	\
								0),
	LPWG_SD_FLAG_SW1828		=	(0 |	\
								U0_M2_RAWDATA_TEST_FLAG |	\
								U0_M1_RAWDATA_TEST_FLAG |	\
								U0_JITTER_TEST_FLAG |	\
								0),
};

static const struct siw_hal_prd_param prd_params[] = {
	/*
	 * LG4894 group
	 */
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_k,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA8C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_ODD, 0xCD2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD95),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xE83),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xBCF),
			0,
		},
		__PRD_PARAM_DIMENSION(26, 15, 1, 32, PRD_M1_COL_SIZE, 2, 2),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_lv,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA8C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_ODD, 0xCD2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD95),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xE83),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xBCF),
			0,
		},
		__PRD_PARAM_DIMENSION(26, 15, 1, 32, PRD_M1_COL_SIZE, 2, 2),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = prd_param_name_lg4894_sf,
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA02),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xB22),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xC42),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xD96),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = (PRD_SYS_EN_DEBUG_BUF|PRD_SYS_EN_APP_DEBUG_BUF),
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	{	.chip_type = CHIP_LG4894,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xA02),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xB22),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xC42),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xD96),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = (PRD_SYS_EN_DEBUG_BUF|PRD_SYS_EN_APP_DEBUG_BUF),
		.sd_test_flag = SD_FLAG_LG4894,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4894,
	},
	/*
	 * LG4895 group
	 */
	{	.chip_type = CHIP_LG4895,
		.name = prd_param_name_lg4895_k,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xE4E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xF80),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x10E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(34, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4895,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4895,
	},
	{	.chip_type = CHIP_LG4895,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xD1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xE4E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xF80),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x10E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(34, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4895,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4895,
	},
	/*
	 * LG4946 group
	 */
	{	.chip_type = CHIP_LG4946,
		.name = prd_param_name_lg4946_g,
		.cmd_type = PRD_CMD_TYPE_2,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4946,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4946,
	},
	{	.chip_type = CHIP_LG4946,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_LG4946,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_LG4946,
	},
	/*
	 * SW49105 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49105,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB85),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF19),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xB0F),
			0,
		},
		__PRD_PARAM_DIMENSION(30, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49105,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49105,
	},
	/*
	 * SW49406 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49406,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB82),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC2),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF16),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xADA),
			0,
		},
		__PRD_PARAM_DIMENSION(32, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49406,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49406,
	},
	/*
	 * SW49407 group
	 */
	{	.chip_type = CHIP_SW49407,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xF1C),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0x104E),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0x1180),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0x12E8),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xCCF),
			0,
		},
		__PRD_PARAM_DIMENSION(32+2, 18, 0, 32+2, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(1, 4),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49407,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49407,
	},
	/*
	 * SW49408 group (Not fixed)
	 */
	{	.chip_type = CHIP_SW49408,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xB85),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xCA5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xDC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xF19),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DEBUG, 0xB0F),
			0,
		},
		__PRD_PARAM_DIMENSION(30, 18, 0, 32, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = 0,
		.sd_test_flag = SD_FLAG_SW49408,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW49408,
	},
	/*
	 * SW1828 group
	 */
	{	.chip_type = CHIP_SW1828,
		.name = NULL,	//NULL meas 'Last & Default'
		.cmd_type = PRD_CMD_TYPE_1,
		.addr = {
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_RAW, 0xACF),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_BASELINE_EVEN, 0xC0F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_DELTA, 0xD4F),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_LABEL, 0xEC5),
			PRD_OFFSET_QUIRK_SET(IMG_OFFSET_IDX_F_DELTA, 0x7FD),
			0,
		},
		__PRD_PARAM_DIMENSION(20, 32, 0, 48, PRD_M1_COL_SIZE, 1, 1),
		__PRD_2ND_SCR(0, 0),
		.sysfs_off_flag = (PRD_SYS_EN_DEBUG_BUF|PRD_SYS_EN_APP_DEBUG_BUF),
		.sd_test_flag = SD_FLAG_SW1828,
		.lpwg_sd_test_flag = LPWG_SD_FLAG_SW1828,
	},
	/*
	 * End
	 */
	{ 0, },
};

#define __RESULT_FLAG_CAL(_result_all, result, flag)	\
		((_result_all) | ((result) << (flag)))

#define RAW_OFFSET_EVEN(_addr)		(((_addr) >> 16) & 0xFFFF)
#define RAW_OFFSET_ODD(_addr)		((_addr) & 0xFFFF)

#define siw_prd_buf_snprintf(_buf, _size, _fmt, _args...) \
		__siw_snprintf(_buf, PRD_BUF_SIZE, _size, _fmt, ##_args)

#define siw_prd_log_buf_snprintf(_buf, _size, _fmt, _args...) \
		__siw_snprintf(_buf, PRD_LOG_BUF_SIZE, _size, _fmt, ##_args)


enum {
	CMD_TEST_EXIT = 0,
	CMD_TEST_ENTER,
};

enum {
	CMD_RAWDATA_PRD = 1,
	CMD_RAWDATA_TCM,
	CMD_RAWDATA_AIT,
	CMD_AIT_BASEDATA,
	CMD_FILTERED_DELTADATA,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_BLU_JITTER,
	CMD_DEBUG_BUF,
	CMD_MAX,
};

#define __PRD_GET_DATA_CMD_SET(_idx)	[_idx] = #_idx

static const char *prd_get_data_cmd_name[] = {
	[0] = "(none)",
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_PRD),
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_TCM),
	__PRD_GET_DATA_CMD_SET(CMD_RAWDATA_AIT),
	__PRD_GET_DATA_CMD_SET(CMD_AIT_BASEDATA),
	__PRD_GET_DATA_CMD_SET(CMD_FILTERED_DELTADATA),
	__PRD_GET_DATA_CMD_SET(CMD_DELTADATA),
	__PRD_GET_DATA_CMD_SET(CMD_LABELDATA),
	__PRD_GET_DATA_CMD_SET(CMD_BLU_JITTER),
	__PRD_GET_DATA_CMD_SET(CMD_DEBUG_BUF),
};

static const char *prd_cmp_tool_str[][2] = {
	[U3_M2_RAWDATA_TEST] = {
		[0] = "U3_M2_Lower",
		[1] = "U3_M2_Upper",
	},
	[U0_M2_RAWDATA_TEST] = {
		[0] = "U0_M2_Lower",
		[1] = "U0_M2_Upper",
	},
	[U0_M1_RAWDATA_TEST] = {
		[0] = "U0_M1_Lower",
		[1] = "U0_M1_Upper",
	},
	[U3_BLU_JITTER_TEST] = {
		[0] = "U3_Blu_Jitter_Lower",
		[1] = "U3_Blu_Jitter_Upper",
	},
	[U3_JITTER_TEST] = {
		[0] = "U3_Jitter_Lower",
		[1] = "U3_Jitter_Upper",
	},
	[U0_JITTER_TEST] = {
		[0] = "U0_Jitter_Lower",
		[1] = "U0_Jitter_Upper",
	},
};

enum{
	M1_EVEN_DATA = 0,
	M1_ODD_DATA,
	M2_EVEN_DATA,
	M2_ODD_DATA ,
	LABEL_DATA,
	DEBUG_DATA,
	};

/* PRD RAWDATA test RESULT Save On/Off CMD */
enum {
	RESULT_OFF = 0,
	RESULT_ON,
};

/* TCM Memory Access Select */
//tc_mem_sel(0x0457) RAW : 0 , BASE1 : 1 , BASE2 : 2 , BASE3: 3
enum {
	TCM_MEM_RAW = 0,
	TCM_MEM_BASE1,
	TCM_MEM_BASE2,
	TCM_MEM_BASE3,
};

/* AIT Algorithm Engine HandShake CMD */
enum {
	IT_IMAGE_NONE = 0,
	IT_IMAGE_RAW,
	IT_IMAGE_BASELINE,
	IT_IMAGE_DELTA,
	IT_IMAGE_LABEL,
	IT_IMAGE_FILTERED_DELTA,
	IT_IMAGE_RESERVED,
	IT_IMAGE_DEBUG,
	IT_DONT_USE_CMD = 0xEE,
	IT_WAIT = 0xFF,
};

/* AIT Algorithm Engine HandShake Status */
enum {
	RS_READY    = 0xA0,
	RS_NONE     = 0x05,
	RS_LOG      = 0x77,
	RS_IMAGE	= 0xAA
};

enum {
	PRD_TIME_STR_SZ = 64,
	PRD_TMP_FILE_NAME_SZ = PATH_MAX,
};

enum {
	PRD_TUNE_DISPLAY_TYPE_1 = 1,
	PRD_TUNE_DISPLAY_TYPE_2,
};

static char __prd_in_file[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_in.txt";
static char __prd_in_file_m[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_in_mfts.txt";
static char __prd_out_file[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out.txt";
static char __prd_out_file_mo_aat[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_aat.txt";
static char __prd_out_file_mo_mfo[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mfo.txt";
static char __prd_out_file_mo_mfl[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mfl.txt";
static char __prd_out_file_mo_mcv[PRD_TMP_FILE_NAME_SZ] =	\
			"/sdcard/siw/prd_out_mfts_mcv.txt";

#if defined(MODULE)
/* use eg. prd_if=arc1 to change name */
module_param_string(prd_if, __prd_in_file, sizeof(__prd_in_file), 0);

/* use eg. prd_if_m=arc1 to change name */
module_param_string(prd_if_m, __prd_in_file_m, sizeof(__prd_in_file_m), 0);

/* use eg. prd_of=arc1 to change name */
module_param_string(prd_of, __prd_out_file, sizeof(__prd_out_file), 0);

/* use eg. prd_of_mo_aat=arc1 to change name */
module_param_string(prd_of_mo_aat, __prd_out_file_mo_aat, sizeof(__prd_out_file_mo_aat), 0);

/* use eg. prd_of_mo_mfo=arc1 to change name */
module_param_string(prd_of_mo_mfo, __prd_out_file_mo_mfo, sizeof(__prd_out_file_mo_mfo), 0);

/* use eg. prd_of_mo_mfl=arc1 to change name */
module_param_string(prd_of_mo_mfl, __prd_out_file_mo_mfl, sizeof(__prd_out_file_mo_mfl), 0);

/* use eg. prd_of_mo_mcv=arc1 to change name */
module_param_string(prd_of_mo_mcv, __prd_out_file_mo_mcv, sizeof(__prd_out_file_mo_mcv), 0);
#endif


static u32 t_prd_dbg_mask = 0;

/* usage
 * (1) echo <value> > /sys/module/{Siw Touch Module Name}/parameters/s_prd_dbg_mask
 * (2) insmod {Siw Touch Module Name}.ko s_prd_dbg_mask=<value>
 */
module_param_named(s_prd_dbg_mask, t_prd_dbg_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);


enum {
	PRD_SHOW_FLAG_DISABLE_PRT_RAW	= (1<<0),
};

#define t_prd_info(_prd, fmt, args...)	\
		__t_dev_info(_prd->dev, "%s : " fmt, _prd->name, ##args)

#define t_prd_err(_prd, fmt, args...)	\
		__t_dev_err(_prd->dev, "%s : " fmt, _prd->name, ##args)

#define t_prd_warn(_prd, fmt, args...)	\
		__t_dev_warn(_prd->dev, "%s : " fmt, _prd->name, ##args)

#define t_prd_dbg(condition, _prd, fmt, args...)	\
		do {	\
			if (unlikely(t_prd_dbg_mask & (condition)))	\
				t_prd_info(_prd, fmt, ##args);	\
		} while (0)

#define t_prd_dbg_base(_prd, fmt, args...)	\
		t_prd_dbg(DBG_BASE, _prd, fmt, ##args)

#define t_prd_dbg_trace(_prd, fmt, args...)	\
		t_prd_dbg(DBG_TRACE, _prd, fmt, ##args)

#define t_prd_info_flag(_prd, _flag, fmt, args...)	\
		do { \
			if (!(_flag & PRD_SHOW_FLAG_DISABLE_PRT_RAW)){ \
				t_prd_info(_prd, fmt, ##args); \
			} \
		} while(0)

#define t_prd_dbg_base_flag(_prd, _flag, fmt, args...)	\
		do { \
			if (!(_flag & PRD_SHOW_FLAG_DISABLE_PRT_RAW)){ \
				t_prd_dbg_base(_prd, fmt, ##args); \
			} \
		} while(0)

#define siw_prd_sysfs_err_invalid_param(_prd)	\
		t_prd_err(_prd, "Invalid param\n");


static int prd_chip_reset(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	int ret;

//	mutex_lock(&ts->lock);
	ret = siw_ops_reset(ts, HW_RESET_SYNC);
//	mutex_unlock(&ts->lock);

	return ret;
}

static int prd_chip_info(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	return siw_ops_ic_info(ts);
}

static int prd_chip_driving(struct device *dev, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;

	return siw_ops_tc_driving(ts, mode);
}

static ssize_t prd_kernel_write(struct file *file,
				const char *buf, size_t count,
			    loff_t *pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	/* The cast to a user pointer is valid due to the set_fs() */
	res = vfs_write(file, (__force const char __user *)buf, count, pos);
	set_fs(old_fs);

	return res;
}

static int __prd_boot_mode_is_err(struct device *dev, int boot_mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	if ((boot_mode > MINIOS_MFTS_CURVED) || (boot_mode < NORMAL_BOOT)) {
		t_prd_err(prd, "boot mode(%d) not support mode\n", boot_mode);
		return -EINVAL;
	}
	return 0;
}

static int __used prd_chk_file_mode(struct siw_hal_prd_data *prd,
						struct file *filp,
						char *fname)
{
	struct dentry *fentry = filp->f_path.dentry;

	if (filp->f_flags & (O_RDWR|O_WRONLY)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0))
		if (!(filp->f_mode & FMODE_CAN_WRITE)) {
			t_prd_err(prd, "file not writeable : %s\n", fname);
			return -EINVAL;
		}
#endif
	}

	if (filp->f_flags & (O_RDWR|O_RDONLY)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0))
		if (!(filp->f_mode & FMODE_CAN_READ)) {
			t_prd_err(prd, "file not readable : %s\n", fname);
			return -EINVAL;
		}
#endif

		if (S_ISDIR(fentry->d_inode->i_mode)) {
			t_prd_err(prd, "%s is folder\n", fname);
			return -EOPNOTSUPP;
		}

		if (S_ISLNK(fentry->d_inode->i_mode)) {
			t_prd_err(prd, "%s is link\n", fname);
			return -EOPNOTSUPP;
		}
	}
	return 0;
}

static struct file __used *prd_vfs_file_open(struct siw_hal_prd_data *prd,
				char *fname, int flags, umode_t mode)
{
	struct file *filp;

	filp = filp_open((const char *)fname, flags, mode);
	if (IS_ERR(filp)) {
		int ret = (int)PTR_ERR(filp);
		t_prd_err(prd, "file[%s, 0x%X, 0x%X] open failed, %d\n",
				fname, flags, mode, ret);
		return NULL;
	}

	return filp;
}

static int __used prd_vfs_file_chk(struct siw_hal_prd_data *prd,
				char *fname,
				int flags,
				umode_t mode,
				loff_t *size)
{
	struct file *filp;
	int ret = 0;

	filp = prd_vfs_file_open(prd, fname, flags, mode);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	if (size) {
		*size = vfs_llseek(filp, 0, SEEK_END);
	}

	ret = prd_chk_file_mode(prd, filp, fname);
	if (ret < 0) {
		goto out_close;
	}

out_close:
	filp_close(filp, 0);

out:
	return ret;
}

static void __used prd_vfs_file_close(struct file *filp, fl_owner_t id)
{
	filp_close(filp, id);
}


/*
 * [Safety and Compatibility]
 * Using call_usermodehelper is recommended
 * because direct access to file system in kernel space is not good way.
 */
#define __PRD_USE_USERMODE_HELPER

static inline char *prd_find_name_pos(char *pathname)
{
	char *pos, *tpos;

	pos = strrchr(pathname, '/');
	tpos = (pos) ? pos + 1 : pathname;

	return tpos;
}

#if defined(__PRD_USE_USERMODE_HELPER)
#define t_prd_err_uh(_prd, _ret, fmt, args...)	\
		t_prd_err(_prd, "user mode helper" fmt " failed, %d\n",	\
				##args, _ret)

#define t_prd_dbg_uh(_prd, fmt, args...)	\
		t_prd_dbg_trace(_prd, "user mode helper" fmt " done\n", ##args)

static int prd_vfs_uh_remove(struct siw_hal_prd_data *prd,
				char *pathname)
{
	char *argv[4] = { "/bin/rm", NULL, NULL, NULL };
	char *envp[1] = { NULL };
	int ret = 0;

	argv[1] = (char *)pathname;

//	envp[0] = "HOME=/";
//	envp[1] = "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin";

	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	if (ret < 0) {
		t_prd_err_uh(prd, ret, "[%s %s]", argv[0], argv[1]);
	}
	t_prd_dbg_uh(prd, "[%s %s]", argv[0], argv[1]);

	return ret;
}

static int prd_vfs_uh_rename(struct siw_hal_prd_data *prd,
				char *pathname,
				char *pathname_new)
{
	char *argv[4] = { "/bin/mv", NULL, NULL, NULL };
	char *envp[1] = { NULL };
	int ret = 0;

	argv[1] = (char *)pathname;
	argv[2] = (char *)pathname_new;

//	envp[0] = "HOME=/";
//	envp[1] = "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin";

	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	if (ret < 0) {
		t_prd_err_uh(prd, ret, "[%s %s %s]", argv[0], argv[1], argv[2]);
	}
	t_prd_dbg_uh(prd, "[%s %s %s]", argv[0], argv[1], argv[2]);

	return ret;
}

static int prd_vfs_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	struct file *filp = NULL;
	char *tmp = NULL;
	char *pathname = NULL;
	int ret = 0;

	tmp = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp) {
		return -ENOMEM;
	}

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	pathname = d_path(&filp->f_path, tmp, PAGE_SIZE);
	if (IS_ERR(pathname)) {
		ret = -ENOMEM;
		goto out;
	}

	prd_vfs_file_close(filp, 0);

	if (ret < 0) {
		goto out;
	}

	/*
	 * Step 2. Remove target file
	 */
	ret = prd_vfs_uh_remove(prd, pathname);

out:
	kfree(tmp);
	return ret;
}

static int prd_vfs_rename(struct siw_hal_prd_data *prd,
					char *fname,
					char *fname_new)
{
	struct file *filp = NULL;
	char *tmp = NULL;
	char *tmp_new = NULL;
	char *pathname = NULL;
	char *pathname_new = NULL;
	char *wpos, *rpos;
	int ret = 0;

	tmp = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp) {
		return -ENOMEM;
	}

	tmp_new = (char *)kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp_new) {
		kfree(tmp);
		return -ENOMEM;
	}

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	pathname = d_path(&filp->f_path, tmp, PAGE_SIZE);
	if (IS_ERR(pathname)) {
		ret = -ENOMEM;
		goto out;
	}

	prd_vfs_file_close(filp, 0);

	if (ret < 0) {
		goto out;
	}

	/*
	 * Step 2. pathname_new
	 */
	pathname_new = tmp_new;
	strncpy(pathname_new, pathname, strlen(pathname));

	wpos = prd_find_name_pos(pathname_new);
	rpos = prd_find_name_pos(fname_new);

	memcpy(wpos, rpos, strlen(rpos));

	/*
	 * Step 3. Change name
	 */
	ret = prd_vfs_uh_rename(prd, pathname, pathname_new);

out:
	kfree(tmp_new);
	kfree(tmp);
	return ret;
}
#else	/* __PRD_USE_USERMODE_HELPER */
static int prd_vfs_do_remove(struct siw_hal_prd_data *prd,
				struct dentry *fentry,
				char *fname)
{
	struct dentry *dir = NULL;
	int ret = 0;

	dir = dget_parent(fentry);

	mutex_lock(&dir->d_inode->i_mutex);

	spin_lock(&fentry->d_lock);

	t_prd_dbg_trace(prd, "unhashed 0x%X, d_inode %p\n",
			d_unhashed(fentry), fentry->d_inode);

	if (d_unhashed(fentry) && fentry->d_inode) {
		spin_unlock(&fentry->d_lock);
		goto out;
	}

	dget_dlock(fentry);
	__d_drop(fentry);
	spin_unlock(&fentry->d_lock);

	vfs_unlink(dir->d_inode, fentry);

	t_prd_dbg_trace(prd, "vfs_unlink done\n");

out:
	mutex_unlock(&dir->d_inode->i_mutex);
	return ret;
}

static int prd_vfs_do_rename(struct siw_hal_prd_data *prd,
				struct dentry *fentry,
				char *fname_new)
{
	struct dentry *dir = NULL;
	struct inode *d_inode = NULL;
	struct dentry *fentry_new = NULL;
	char *base_name = NULL;
	int ret = 0;

	dir = dget_parent(fentry);
	d_inode = dir->d_inode;

	lock_rename(dir, dir);

	base_name = prd_find_name_pos(fname_new);
	t_prd_dbg_trace(prd, "base name : %s\n", base_name);

	fentry_new = lookup_one_len(base_name, dir, strlen(base_name));
	if (IS_ERR(fentry_new)) {
		ret = PTR_ERR(fentry_new);
		t_prd_err(prd, "lookup_one_len for fentry_new failed, %d\n", ret);
		goto out;
	}

	ret = vfs_rename(d_inode, fentry, d_inode, fentry_new);

out:
	unlock_rename(dir, dir);

	return ret;
}

static int prd_vfs_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	struct file *filp = NULL;
	struct dentry *fentry = NULL;
	int ret = 0;

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		return -ENOENT;
	}

	fentry = filp->f_path.dentry;

	prd_vfs_file_close(filp, 0);

	/*
	 * Step 2. Remove target file
	 */
	ret = prd_vfs_do_remove(prd, fentry, fname);

	return ret;
}

static int prd_vfs_rename(struct siw_hal_prd_data *prd,
					char *fname,
					char *fname_new)
{
	struct file *filp = NULL;
	struct dentry *fentry = NULL;
	int ret = 0;

	/*
	 * Step 1. Get fentry data
	 */
	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		return -ENOENT;
	}

	fentry = filp->f_path.dentry;

	prd_vfs_file_close(filp, 0);

	/*
	 * Step 2. Change name
	 */
	ret = prd_vfs_do_rename(prd, fentry, fname_new);

	return ret;
}

#endif	/* __PRD_USE_USERMODE_HELPER */


#define t_prd_dbg_file_modify(_prd, fmt, args...)	\
		t_prd_dbg_trace(_prd, "try to " fmt "\n", ##args)

#define t_prd_dbg_file_remove(_prd, fmt, args...)	\
		t_prd_dbg_file_modify(_prd, "remove " fmt, ##args)

#define t_prd_dbg_file_rename(_prd, fmt, args...)	\
		t_prd_dbg_file_modify(_prd, "rename " fmt, ##args)

static int __used prd_vfs_file_remove(struct siw_hal_prd_data *prd,
				char *fname)
{
	int ret = 0;

	t_prd_dbg_file_remove(prd, "%s", fname);

	ret = prd_vfs_remove(prd, fname);
	if (ret < 0) {
		if (ret != -ENOENT) {
			t_prd_err(prd, "failed to remove file : %s, %d\n", fname, ret);
		}
		goto out;
	}
	t_prd_info(prd, "file removed : %s\n", fname);

out:
	return 0;
}

static int __used prd_vfs_file_rename(struct siw_hal_prd_data *prd,
				char *fname, char *fname_new)
{
	int ret = 0;

	t_prd_dbg_file_rename(prd, "%s -> %s", fname, fname_new);

	ret = prd_vfs_rename(prd, fname, fname_new);
	if (ret < 0) {
		if (ret != -ENOENT) {
			t_prd_err(prd, "failed to rename file : %s -> %s, %d\n",
					fname, fname_new, ret);
		}
		goto out;
	}
	t_prd_info(prd, "file renamed : %s -> %s\n",
			fname, fname_new);

out:
	return 0;
}

static int prd_log_file_size_check(struct siw_hal_prd_data *prd)
{
	char *prd_out_fname[] = {
		[NORMAL_BOOT]			= __prd_out_file,
		[MINIOS_AAT]			= __prd_out_file_mo_aat,
		[MINIOS_MFTS_FOLDER]	= __prd_out_file_mo_mfo,
		[MINIOS_MFTS_FLAT]		= __prd_out_file_mo_mfl,
		[MINIOS_MFTS_CURVED]	= __prd_out_file_mo_mcv,
	};
	struct device *dev = prd->dev;
	char *fname = NULL;
	loff_t file_size = 0;
	int i = 0;
	char *buf1 = NULL;
	char *buf2 = NULL;
	int boot_mode = 0;
	int ret = 0;

	boot_mode = siw_touch_boot_mode_check(dev);

	if (__prd_boot_mode_is_err(dev, boot_mode)) {
		return -EINVAL;
	}

	buf1 = touch_getname();
	if (buf1 == NULL) {
		t_prd_err(prd, "failed to allocate name buffer 1\n");
		return -ENOMEM;
	}

	buf2 = touch_getname();
	if (buf2 == NULL) {
		t_prd_err(prd, "failed to allocate name buffer 2\n");
		touch_putname(buf1);
		return -ENOMEM;
	}

	fname = (char *)prd_out_fname[boot_mode];

	ret = prd_vfs_file_chk(prd, fname, O_RDONLY, 0666, &file_size);
	if (ret < 0) {
		goto out;
	}

	if (file_size <= MAX_LOG_FILE_SIZE) {
		goto out;
	}

	t_prd_info(prd, "file size : %s, %lld > MAX_LOG_FILE_SIZE(%d)\n",
			fname, file_size, MAX_LOG_FILE_SIZE);

	for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
		if (i == 0) {
			snprintf(buf1, PATH_MAX, "%s", fname);
		} else {
			snprintf(buf1, PATH_MAX, "%s.%d", fname, i);
		}

		if (i == (MAX_LOG_FILE_COUNT - 1)) {
			ret = prd_vfs_file_remove(prd, buf1);
			if (ret < 0) {
				goto out;
			}
			t_prd_info(prd, "file removed : %s\n", buf1);
		} else {
			snprintf(buf2, PATH_MAX, "%s.%d", fname, (i + 1));

			ret = prd_vfs_file_rename(prd, buf1, buf2);
			if (ret < 0) {
				goto out;
			}
		}
	}

out:
	touch_putname(buf2);
	touch_putname(buf1);

	return ret;
}

static int prd_do_write_file(struct siw_hal_prd_data *prd,
				char *fname,
				char *data,
				int write_time)
{
//	struct device *dev = prd->dev;
	struct file *filp = NULL;
	loff_t offset = 0;
	char time_string[PRD_TIME_STR_SZ] = {0, };
	int twlen = 0;
	int nwlen = 0;
	int ret = 0;

	filp = prd_vfs_file_open(prd, fname,
				O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	ret = prd_chk_file_mode(prd, filp, fname);
	if (ret < 0) {
		goto out_close;
	}

	if (write_time == TIME_INFO_WRITE) {
		struct timespec my_time;
		struct tm my_date;

		my_time = current_kernel_time();
		time_to_tm(my_time.tv_sec,
				sys_tz.tz_minuteswest * 60 * (-1),
				&my_date);
		snprintf(time_string, 64,
			"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
			my_date.tm_mon + 1,
			my_date.tm_mday, my_date.tm_hour,
			my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);
		ret = prd_kernel_write(filp, (const char *)time_string,
					strlen(time_string), &offset);
		if (ret < 0) {
			t_prd_err(prd, "failed to write file : %s(%d), %d \n",
					fname, (int)offset, ret);
			goto out_close;
		}
		twlen += strlen(time_string);
		nwlen += ret;
	}
	ret = prd_kernel_write(filp, (const char *)data,
				strlen(data), &offset);
	if (ret < 0) {
		t_prd_err(prd, "failed to write file : %s(%d), %d \n",
				fname, (int)offset, ret);
		goto out_close;
	}
	twlen += strlen(data);
	nwlen += ret;

out_close:
	filp_close(filp, 0);
	if (ret >= 0) {
		t_prd_info(prd, "file write done : %s(%d, %d)\n",
				fname, twlen, nwlen);
	}

out:
	return ret;
}

static int prd_write_file(struct siw_hal_prd_data *prd, char *data, int write_time)
{
	char *prd_out_fname[] = {
		[NORMAL_BOOT]			= __prd_out_file,
		[MINIOS_AAT]			= __prd_out_file_mo_aat,
		[MINIOS_MFTS_FOLDER]	= __prd_out_file_mo_mfo,
		[MINIOS_MFTS_FLAT]		= __prd_out_file_mo_mfl,
		[MINIOS_MFTS_CURVED]	= __prd_out_file_mo_mcv,
	};
	struct device *dev = prd->dev;
	char *fname = NULL;
	int boot_mode = 0;
	int ret = 0;

	boot_mode = siw_touch_boot_mode_check(dev);
	if (__prd_boot_mode_is_err(dev, boot_mode)) {
		return -EINVAL;
	}

	fname = (char *)prd_out_fname[boot_mode];

	ret = prd_do_write_file(prd, fname, data, write_time);

	return ret;
}

/*
 * Conrtol LCD Backlightness
 */
static int prd_set_blu(struct device *dev)
{
	int backlightness;

//	LCD brightness ON 742ms -> LCD brightness OFF 278mms -> LCD brightness ON 278ms

	backlightness = siw_touch_sys_get_panel_bl(dev);

	touch_msleep(742);

	siw_touch_sys_set_panel_bl(dev, 0);

	touch_msleep(278);

	siw_touch_sys_set_panel_bl(dev, backlightness);

	touch_msleep(278);

	return 0;
}

static int prd_write_test_mode(struct siw_hal_prd_data *prd, int type)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_sd_cmd *sd_cmd = &prd->sd_cmd;
	u32 testmode = 0;
	int retry = 40;
	u32 rdata = 0x01;
	u32 line_filter_option = LINE_FILTER_OPTION;
	int waiting_time = 200;
	int addr = reg->tc_tsp_test_ctl;
	int ret = 0;

	switch (touch_chip_type(ts)) {
	case CHIP_SW1828:
		line_filter_option = 0;
		switch (type) {
		case U3_JITTER_TEST:
			waiting_time = 6000;
			break;
		}
		break;
	case CHIP_SW49407:
		line_filter_option = 0;
		break;
	}

	switch (type) {
	case U3_M2_RAWDATA_TEST:
		testmode = (U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_m2_rawdata;
		break;
	case U0_M2_RAWDATA_TEST:
		testmode = (U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_m2_rawdata;
		break;
	case U0_M1_RAWDATA_TEST:
		testmode = (U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_m1_rawdata;
		break;
	/* */
	case OPEN_NODE_TEST:
		testmode = (U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_open_node;
		waiting_time = 10;
		break;
	case SHORT_NODE_TEST:
		testmode = (U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_short_node;
		waiting_time = 1000;
		break;
	/* */
	case U3_BLU_JITTER_TEST:
		testmode = ((U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_jitter) | line_filter_option;
		waiting_time = 10;
		break;
	case U3_JITTER_TEST:
		testmode = ((U3_TEST_PRE_CMD << 8) + sd_cmd->cmd_jitter) | line_filter_option;
		break;
	case U0_JITTER_TEST:
		testmode = ((U0_TEST_PRE_CMD << 8) + sd_cmd->cmd_u0_jitter) | line_filter_option;
		break;
	default:
		t_prd_err(prd, "unsupported test mode, %d\n", type);
		return -EINVAL;
	}

	/* TestType Set */
	ret = siw_hal_write_value(dev, addr, testmode);
	if (ret < 0) {
		return ret;
	}
	t_prd_info(prd, "write testmode[%04Xh] = %x\n",
		addr, testmode);
	touch_msleep(waiting_time);

	if(type == U3_BLU_JITTER_TEST) {
		prd_set_blu(dev);
	}

	/* Check Test Result - wait until 0 is written */
	addr = reg->tc_tsp_test_status;
	do {
		touch_msleep(50);
		ret = siw_hal_read_value(dev, addr, &rdata);
		if (ret < 0) {
			return ret;
		}
		if (ret >= 0) {
			t_prd_dbg_base(prd, "rdata[%04Xh] = 0x%x\n",
				addr, rdata);
		}
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		t_prd_err(prd, "ProductionTest Type [%d] Time out\n", type);
		goto out;
	}

	return 1;

out:
	t_prd_err(prd, "Write test mode failed\n");
	return 0;
}

static int prd_spec_file_read_ext(struct siw_hal_prd_data *prd)
{
	char *prd_in_fname[] = {
		[0]	= __prd_in_file,
		[1]	= __prd_in_file_m,
	};
	struct device *dev = prd->dev;
	struct file *filp = NULL;
	loff_t offset = 0;
	char *fname;
	int path_idx = 0;
	int ret = 0;

	path_idx = !!(siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER);
	fname = (char *)prd_in_fname[path_idx];

	filp = prd_vfs_file_open(prd, fname, O_RDONLY, 0);
	if (filp == NULL) {
		ret = -ENOENT;
		goto out;
	}

	kernel_read(filp, offset, (char *)prd->line, sizeof(prd->line));

	filp_close(filp, 0);

	t_prd_info(prd, "file detected : %s\n", fname);

out:
	return ret;
}

static int prd_spec_file_read(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	const struct firmware *fwlimit = NULL;
	const char *path[2] = {
		ts->panel_spec,
		ts->panel_spec_mfts
	};
	char *fname;
	int path_idx = 0;
	int ret = 0;

	path_idx = !!(siw_touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER);
	fname = (char *)path[path_idx];

	if (ts->panel_spec == NULL || ts->panel_spec_mfts == NULL) {
		t_prd_err(prd, "panel spec file name is null\n");
		siw_prd_buf_snprintf(prd->buf_write,
					0,
					"[E] panel spec file name is null\n");
		ret = -EINVAL;
		goto out;
	}

	ret = request_firmware(&fwlimit, fname, dev);
	if (ret < 0) {
		t_prd_err(prd, "request file is failed in normal mode\n");
		siw_prd_buf_snprintf(prd->buf_write,
					0,
					"[E] request file is failed in normal mode\n");
		goto out;
	}

	if (fwlimit->data == NULL) {
		ret = -EFAULT;
		t_prd_err(prd, "fwlimit->data is NULL\n");
		siw_prd_buf_snprintf(prd->buf_write,
					0,
					"[E] fwlimit->data is NULL\n");
		goto out;
	}

	strlcpy(prd->line, fwlimit->data, sizeof(prd->line));

out:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

static int __used prd_get_limit(struct siw_hal_prd_data *prd,
		char *breakpoint, int *limit)
{
	struct device *dev = prd->dev;
	int p = 0;
	int q = 0;
	int q_limit;
	int cipher = 1;
	char *found;
	int boot_mode = 0;
	int value = ~0;
	int ret = 0;

	if (limit)
		*limit = ~0;

	if (breakpoint == NULL) {
		ret = -EINVAL;
		goto out;
	}

	boot_mode = siw_touch_boot_mode_check(dev);
	if ((boot_mode > MINIOS_MFTS_CURVED) ||
		(boot_mode < NORMAL_BOOT)) {
		ret = -EINVAL;
		goto out;
	}

	ret = prd_spec_file_read_ext(prd);
	if (ret < 0) {
		ret = prd_spec_file_read(prd);
		if (ret < 0)
			goto out;
	}

	found = strnstr(prd->line, breakpoint, sizeof(prd->line));
	if (found == NULL) {
		t_prd_err(prd,
			"failed to find %s, spec file is wrong\n",
			breakpoint);
		ret = -EFAULT;
		goto out;
	}
	q = found - prd->line;

	q_limit = ARRAY_SIZE(prd->line);
	while (q < q_limit) {
		if (prd->line[q] == ',') {
			value = 0;

			cipher = 1;
			for (p = 1; (prd->line[q - p] >= '0') &&
					(prd->line[q - p] <= '9'); p++) {
				value += ((prd->line[q - p] - '0') * cipher);
				cipher *= 10;
			}

			if (limit)
				*limit = value;

			t_prd_info(prd, "scanning spec file done, %s = %d\n",
					breakpoint, value);

			break;
		}
		q++;
	}

	if (value == ~0) {
		t_prd_err(prd, "%s found, but getting num failed\n", breakpoint);
		ret = -EFAULT;
		goto out;
	}

out:
	return ret;
}

static int prd_os_result_rawdata_get(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u16 open_data_offset = 0;
	u16 short_data_offset = 0;
	u32 os_result_offset;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->prd_open3_short_offset,
				&os_result_offset);
	if (ret < 0) {
		goto out;
	}
	open_data_offset = os_result_offset & 0xFFFF;
	short_data_offset = (os_result_offset >> 16) & 0xFFFF;

	switch (type) {
	case OPEN_NODE_TEST:
		prd_write_file(prd, "\n[OPEN_NODE_TEST Result Rawdata]\n",
			TIME_INFO_SKIP);
		t_prd_info(prd, "Open Node Raw Data Offset = %x\n",
			short_data_offset);
		t_prd_info(prd, "[OPEN_NODE_TEST Result Rawdata]\n");

		//offset write
		ret = siw_hal_write_value(dev, reg->serial_data_offset,
				open_data_offset);
		if (ret < 0) {
			goto out;
		}

		//read open raw data
		ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr,
				prd->open_buf_result_rawdata,
				ctrl->m2_row_col_buf_size * PRD_RAWDATA_SIZE);
		if (ret < 0) {
			goto out;
		}
		break;
	case SHORT_NODE_TEST:
		prd_write_file(prd, "\n[SHORT_NODE_TEST Result Rawdata]\n",
			TIME_INFO_SKIP);
		t_prd_info(prd, "Short Node Raw Data Offset = %x\n",
			short_data_offset);
		t_prd_info(prd, "[SHORT_NODE_TEST Result Rawdata]\n");

		//offset write
		ret = siw_hal_write_value(dev, reg->serial_data_offset,
				short_data_offset);
 		if (ret < 0) {
			goto out;
		}

		//read short raw data
		ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr,
				prd->short_buf_result_rawdata,
				//(ctrl->m1_row_col_size * 2) * PRD_RAWDATA_SIZE);
				ctrl->m2_row_col_buf_size * PRD_RAWDATA_SIZE);
		if (ret < 0) {
			goto out;
		}
		break;
	}
out:
	return ret;
}

static int prd_os_result_data_get(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u16 open_short_data_offset = 0;
	u32 os_result_offset;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->prd_tune_result_offset,
				&os_result_offset);
	if (ret < 0) {
		goto out;
	}
	open_short_data_offset = os_result_offset & 0xFFFF;

	switch (type) {
	case OPEN_NODE_TEST:
		prd_write_file(prd, "\n[OPEN_NODE_TEST Result Data]\n",
			TIME_INFO_SKIP);
		t_prd_info(prd, "Open Node Raw Offset = %x\n",
			open_short_data_offset);
		t_prd_info(prd, "[OPEN_NODE_TEST Result Data]\n");

		//offset write
		ret = siw_hal_write_value(dev, reg->serial_data_offset,
				open_short_data_offset);
		if (ret < 0) {
			goto out;
		}

		//read open data
		ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr,
				prd->open_buf_result_data,
				ctrl->m1_row_col_size * PRD_RAWDATA_SIZE);
		if (ret < 0) {
			goto out;
		}
		break;
	case SHORT_NODE_TEST:
		prd_write_file(prd, "\n[SHORT_NODE_TEST Result Data]\n",
			TIME_INFO_SKIP);
		t_prd_info(prd, "Short Node Data Offset = %x\n",
			open_short_data_offset);
		t_prd_info(prd, "[SHORT_NODE_TEST Result Data]\n");

		//offset write
		ret = siw_hal_write_value(dev, reg->serial_data_offset,
				open_short_data_offset);
		if (ret < 0) {
			goto out;
		}

		//read short data
		ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr,
				prd->short_buf_result_data,
				ctrl->m1_row_col_size * PRD_RAWDATA_SIZE);
		if (ret < 0) {
			goto out;
		}
		break;
	}
out:
	return ret;
}

enum {
	PRD_OS_RESULT_TMP_BUF_SZ = 64,
};

static int prd_print_os_data(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	int i = 0, j = 0;
	int size = 0;
	int log_size = 0;
	u16 *os_buf = NULL;
	int col_size = 0;
	char *log_buf = prd->log_buf;

	memset(prd->buf_write, 0, PRD_BUF_SIZE);

	switch (type) {
	case OPEN_NODE_TEST:
		col_size = param->m1_col;
		os_buf = prd->open_buf_result_data;
		break;
	case SHORT_NODE_TEST:
		col_size = param->m1_col;
		os_buf = prd->short_buf_result_data;
		break;
	}

	size = siw_prd_buf_snprintf(prd->buf_write, size, "\n   : ");
	log_size = siw_prd_log_buf_snprintf(log_buf, log_size, "   : ");

	for (i = 0; i < col_size; i++) {
		size += siw_prd_buf_snprintf(prd->buf_write,
		size, "[%2d] ", i);
		log_size += siw_prd_log_buf_snprintf(log_buf,
		log_size, "[%2d] ", i);
	}
	t_prd_info(prd, "%s\n", log_buf);

	for (i = 0; i < param->row; i++) {
		log_size = 0;

		size += siw_prd_buf_snprintf(prd->buf_write,
			size,  "\n[%2d] ", i);
		log_size += siw_prd_log_buf_snprintf(log_buf,
				log_size,  "[%2d] ", i);
		for (j = 0; j < col_size; j++) {
			//if (j == param->col)
			//	continue;
			size += siw_prd_buf_snprintf(prd->buf_write,
				size, "%4X ", os_buf[i * col_size + j]);
			log_size += siw_prd_log_buf_snprintf(log_buf,
				log_size, "%4X ", os_buf[i * col_size + j]);
		}
		t_prd_info(prd, "%s\n", log_buf);
	}
	size += siw_prd_buf_snprintf(prd->buf_write, size, "\n");
	prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	memset(prd->buf_write, 0, size);

	return size;
}

static int prd_print_os_rawdata(struct siw_hal_prd_data *prd, u8 type)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int i = 0, j = 0;
	int size = 0;
	int log_size = 0;
	u16 *os_buf = NULL;
	int os_buf_size = ctrl->m2_row_col_buf_size;
	int col_size = 0;
	char *log_buf = prd->log_buf;
	u16 short_temp_buf[os_buf_size];

	memset(prd->buf_write, 0, PRD_BUF_SIZE);

	switch (type) {
	case OPEN_NODE_TEST:
		col_size = param->col + param->col_add;
		os_buf = prd->open_buf_result_rawdata;
		break;
	case SHORT_NODE_TEST:
		col_size = 4;
		os_buf = prd->short_buf_result_rawdata;
		break;
	}

	size = siw_prd_buf_snprintf(prd->buf_write, size, "\n   : ");
	log_size = siw_prd_log_buf_snprintf(log_buf, log_size, "   : ");

	for (i = 0; i < col_size; i++) {
		size += siw_prd_buf_snprintf(prd->buf_write,
		size, "[%2d] ", i);
		log_size += siw_prd_log_buf_snprintf(log_buf,
		log_size, "[%2d] ", i);
	}
	t_prd_info(prd, "%s\n", log_buf);

	/*
	 * short data replace algorithm
	 */
	if(type == SHORT_NODE_TEST) {
		memcpy(short_temp_buf, os_buf, sizeof(short_temp_buf));
		memset(os_buf, 0, os_buf_size);

		for (i = 0; i < col_size; i++) {
			for (j = 0; j < param->row; j++) {
				os_buf[j * col_size + i] = short_temp_buf[i*param->row + j];
			}
		}
	}

	for (i = 0; i < param->row; i++) {
		log_size = 0;

		size += siw_prd_buf_snprintf(prd->buf_write,
			size, "\n[%2d] ", i);
		log_size += siw_prd_log_buf_snprintf(log_buf,
				log_size, "[%2d] ", i);
		for (j = 0; j < col_size; j++) {
			if (j == param->col)
				continue;
			size += siw_prd_buf_snprintf(prd->buf_write,
				size, "%4d ", os_buf[i * col_size + j]);
			log_size += siw_prd_log_buf_snprintf(log_buf,
					log_size, "%4d ", os_buf[i * col_size + j]);
		}
		t_prd_info(prd, "%s\n", log_buf);
	}

	size += siw_prd_buf_snprintf(prd->buf_write, size, "\n");
	prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	memset(prd->buf_write, 0, size);

	return size;
}

static int prd_os_xline_result_read(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	int ret = 0;


	if(param->sd_test_flag & OPEN_SHORT_RESULT_DATA_FLAG) {
		//open short fail reason "result data"
		ret = prd_os_result_data_get(prd, type);
		prd_print_os_data(prd, type);
	}

	if(param->sd_test_flag & OPEN_SHORT_RESULT_RAWDATA_FLAG) {
		//open short fail reason "result raw data"
		ret = prd_os_result_rawdata_get(prd, type);
		prd_print_os_rawdata(prd, type);
	}
	return ret;
}


static int prd_do_open_short_test(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 open_result = 0;
	u32 short_result = 0;
	u32 openshort_all_result = 0;
	int type = 0;
	int ret = 0;

	/* Test Type Write */
	ret = prd_write_file(prd, "\n\n[OPEN_SHORT_ALL_TEST]\n", TIME_INFO_SKIP);
	if (ret < 0) {
		return ret;
	}

	/* 1. open_test */
	type = OPEN_NODE_TEST;
	ret = prd_write_test_mode(prd, type);
	if (ret < 0) {
		return ret;
	}
	if (!ret) {
		t_prd_err(prd, "write test mode failed\n");
		return 0x3;
	}

	t_prd_dbg_base(prd, "result resister:%d \n", reg->tc_tsp_test_pf_result);

	ret = siw_hal_read_value(dev,
				reg->tc_tsp_test_pf_result,
				&open_result);
	if (ret < 0) {
		return ret;
	}
	t_prd_info(prd, "open result = %d\n", open_result);

	if (open_result | (prd->dbg_mask & PRD_DBG_OPEN_SHORT_DATA)) {
		ret = prd_os_xline_result_read(prd, type);
		if (ret < 0) {
			return ret;
		}
	}

	/* 2. short_test */
	type = SHORT_NODE_TEST;
	ret = prd_write_test_mode(prd, type);
	if (ret < 0) {
		return ret;
	}
	if (!ret) {
		t_prd_err(prd, "write test mode failed\n");
		return 0x3;
	}

	ret = siw_hal_read_value(dev,
				reg->tc_tsp_test_pf_result,
				&short_result);
	t_prd_info(prd, "short result = %d\n", short_result);

	if (short_result | (prd->dbg_mask & PRD_DBG_OPEN_SHORT_DATA)) {
		ret = prd_os_xline_result_read(prd, type);
	}

	openshort_all_result = (open_result | (short_result<<1));

	return openshort_all_result;
}

static int prd_open_short_test(struct siw_hal_prd_data *prd)
{
	int ret;

	ret = prd_do_open_short_test(prd);
	if (ret < 0) {
		t_prd_err(prd, "prd_open_short_test failed, %d\n",
				 ret);
	}

	return ret;
}

static int prd_print_pre(struct siw_hal_prd_data *prd, char *buf,
				int size, int row_size, int col_size,
				const char *name)
{
	char *log_buf = prd->log_buf;
	int i;
	int log_size = 0;

	t_prd_info(prd, "-------- %s(%d %d) --------\n",
				name, row_size, col_size);

	size += siw_prd_buf_snprintf(buf, size,
				"-------- %s(%d %d) --------\n",
				name, row_size, col_size);

	/* print a frame data */
	size += siw_prd_buf_snprintf(buf, size, "   : ");
	log_size += siw_prd_log_buf_snprintf(log_buf,
						log_size,  "   : ");

	for (i = 0; i < col_size; i++) {
		size += siw_prd_buf_snprintf(buf, size, "[%2d] ", i);
		log_size += siw_prd_log_buf_snprintf(log_buf,
						log_size,  "[%2d] ", i);
	}
	t_prd_info(prd, "%s\n", log_buf);

	return size;
}

static int prd_print_post(struct siw_hal_prd_data *prd, char *buf,
				int size, int min, int max)
{
	size += siw_prd_buf_snprintf(buf, size, "\n");

	size += siw_prd_buf_snprintf(buf, size,
				"\nRawdata min : %d , max : %d\n",
				min, max);
	t_prd_info(prd, "Rawdata min : %d , max : %d\n",
				min, max);

	return size;
}

enum {
	PRD_PRT_TYPE_U8 = 0,
	PRD_PRT_TYPE_S16,
	PRD_PRT_TYPE_MAX,
};

static int prd_print_xxx(struct siw_hal_prd_data *prd, char *buf,
				int size, void *rawdata_buf,
				int row_size, int col_size,
				const char *name, int opt, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	char *log_buf = prd->log_buf;
	u8 *rawdata_u8 = rawdata_buf;
	int16_t *rawdata_s16 = rawdata_buf;
	int curr_raw;
	int i, j;
	int col_i = 0;
	int col_add = (opt) ? param->col_add : 0;
	int log_size = 0;
	int min = 9999;
	int max = 0;

	if (type >= PRD_PRT_TYPE_MAX) {
		t_prd_err(prd, "invalid print type, %d\n", type);
		return -EINVAL;
	}

	if (rawdata_buf == NULL) {
		t_prd_err(prd, "print failed: NULL buf\n");
		return -EFAULT;
	}

	size = prd_print_pre(prd, buf, size, row_size, col_size, name);

	col_i = 0;
	for (i = 0; i < row_size; i++) {
		size += siw_prd_buf_snprintf(buf, size, "\n[%2d] ", i);

		log_size = 0;
		memset(log_buf, 0, sizeof(prd->log_buf));
		log_size += siw_prd_log_buf_snprintf(log_buf,
						log_size,  "[%2d] ", i);

		if (type == PRD_PRT_TYPE_S16) {
			rawdata_s16 = &((int16_t *)rawdata_buf)[col_i];
		} else {
			rawdata_u8 = &((u8 *)rawdata_buf)[col_i];
		}
		for (j = 0; j < col_size; j++) {
			if (type == PRD_PRT_TYPE_S16) {
				curr_raw = *rawdata_s16++;
			} else {
				curr_raw = *rawdata_u8++;
			}

			size += siw_prd_buf_snprintf(buf, size,
						"%4d ", curr_raw);

			log_size += siw_prd_buf_snprintf(log_buf,
							log_size,
							"%4d ", curr_raw);

			if (curr_raw && (curr_raw < min)) {
				min = curr_raw;
			}
			if (curr_raw > max) {
				max = curr_raw;
			}
		}
		t_prd_info(prd, "%s\n", log_buf);

		col_i += (col_size + col_add);
	}

	size = prd_print_post(prd, buf, size, min, max);

	return size;
}

static int prd_print_u8(struct siw_hal_prd_data *prd, char *buf,
				int size, u8 *rawdata_buf_u8,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx(prd, buf, size, (void *)rawdata_buf_u8,
				row_size, col_size, name, opt, PRD_PRT_TYPE_U8);
}

static int prd_print_s16(struct siw_hal_prd_data *prd, char *buf,
				int size, int16_t *rawdata_buf_u16,
				int row_size, int col_size,
				const char *name, int opt)
{
	return prd_print_xxx(prd, buf, size, (void *)rawdata_buf_u16,
				row_size, col_size, name, opt, PRD_PRT_TYPE_S16);
}

static int prd_print_rawdata(struct siw_hal_prd_data *prd,
			char *buf, int type, int size, int opt)
{
	struct siw_hal_prd_param *param = &prd->param;
	int16_t *rawdata_buf_16 = NULL;
	u8 *rawdata_buf_u8 = NULL;
	const char *name = NULL;
	int row_size = param->row;
	int col_size = param->col;

	switch (type) {
	case M1_EVEN_DATA:
		col_size = param->m1_col;
		rawdata_buf_16 = prd->m1_buf_even_rawdata;
		name = "EVEN Data";
		break;
	case M1_ODD_DATA:
		col_size = param->m1_col;
		rawdata_buf_16 = prd->m1_buf_odd_rawdata;
		name = "ODD Data";
		break;
	case M2_EVEN_DATA:
		rawdata_buf_16 = prd->m2_buf_even_rawdata;
		name = "EVEN Data";
		break;
	case M2_ODD_DATA:
		rawdata_buf_16 = prd->m2_buf_odd_rawdata;
		name = "ODD Data";
		break;
	case LABEL_DATA:
		rawdata_buf_u8 = prd->buf_label;
		name = "LABEL Data";
		break;
	case DEBUG_DATA:
		rawdata_buf_16 = prd->buf_debug;
		name = "DEBUG Data";
		break;
	default:
		t_prd_warn(prd, "prd_print_rawdata: invalud type, %d\n", type);
		break;
	}

	if (rawdata_buf_16) {
		size = prd_print_s16(prd, buf, size, rawdata_buf_16,
				row_size, col_size, name, opt);
	} else if (rawdata_buf_u8) {
		size = prd_print_u8(prd, buf, size, rawdata_buf_u8,
				row_size, col_size, name, opt);
	}

	return size;
}

#define STR_SPEC_BY_PARAM	"(p)"

/*
*	return "result Pass:0 , Fail:1"
*/
static int prd_compare_tool(struct siw_hal_prd_data *prd,
				int test_cnt, int16_t **buf,
				int row, int col, int type, int opt)
{
	struct siw_hal_prd_param *param = &prd->param;
//	struct device *dev = prd->dev;
//	struct siw_ts *ts = to_touch_core(dev);
	struct siw_touch_second_screen *second_screen = NULL;
	int16_t *raw_buf;
	int16_t *raw_curr;
	int i, j ,k;
	int col_i;
	int col_add = (opt) ? param->col_add : 0;
	int size = 0;
	int curr_raw;
	int curr_lower, curr_upper;
	int raw_err = 0;
	int	result = 0;
	int spec_by_param = 0;
	int cnt = 0;

	spec_by_param = !!(opt & (1<<16));
	opt &= 0xFFFF;

	col_add = (opt) ? param->col_add : 0;

	if (!test_cnt) {
		t_prd_err(prd, "zero test count\n");
		result = 1;
		size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"zero test count\n");
		goto out;
	}

	curr_lower = prd->image_lower;
	curr_upper = prd->image_upper;

	t_prd_info(prd, "lower %d, upper %d %s\n",
		curr_lower, curr_upper,
		(spec_by_param) ? STR_SPEC_BY_PARAM : "");
	size += siw_prd_buf_snprintf(prd->buf_write,
				size,
				"lower %d, upper %d %s\n",
				curr_lower, curr_upper,
				(spec_by_param) ? STR_SPEC_BY_PARAM : "");

	if (type != U0_M1_RAWDATA_TEST) {
		second_screen = &param->second_scr;
	}

	for (k = 0; k < test_cnt; k++) {
		t_prd_info(prd,
			"-------- Compare[%d/%d] --------\n",
			k, test_cnt);
		size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"-------- Compare[%d/%d] --------\n",
					k, test_cnt);
		raw_buf = buf[k];
		if (raw_buf == NULL) {
			t_prd_err(prd, "compare failed: NULL buf\n");
			result = 1;
			size += siw_prd_buf_snprintf(prd->buf_write,
					size,
					"compare failed: NULL buf\n");
			goto out;
		}
		col_i = 0;

		for (i = 0; i < row; i++) {
			raw_curr = &raw_buf[col_i];

			for (j = 0; j < col; j++) {
				curr_raw = *raw_curr++;

				if ((curr_raw >= curr_lower) &&
					(curr_raw <= curr_upper)) {
					t_prd_dbg_trace(prd,
						"cnt %d, %d(lower) <= %d <= %d(upper) \n",
						cnt, curr_lower, curr_raw, curr_upper);
					cnt++;
					continue;
				}

				raw_err = 1;
				if ((second_screen != NULL) &&
					(i <= second_screen->bound_i) &&
					(j <= second_screen->bound_j)) {
					raw_err = !!(curr_raw)<<1;	/* 2 or 0 */
				}

				if (raw_err) {
					result = 1;
					t_prd_info(prd,
						"F [%d][%d] = %d(%d)\n",
						i, j, curr_raw, raw_err);
					size += siw_prd_buf_snprintf(prd->buf_write,
								size,
								"F [%d][%d] = %d(%d)\n",
								i, j, curr_raw, raw_err);
				} else {
				#if 0
					t_prd_info(prd,
						"G [%d][%d] = %d\n",
						i, j, curr_raw);
				#endif
				}
			}
			col_i += (col + col_add);
		}

		if (!result) {
			t_prd_info(prd,
				"(none)\n");
			size += siw_prd_buf_snprintf(prd->buf_write,
						size,
						"(none)\n");
		}
	}

//	t_prd_info(prd, "type %d, result %d\n", type, result);

out:
	return result;
}

/*
 * Rawdata compare result
 * Pass : reurn 0
 * Fail : return 1
 */
static int prd_compare_rawdata(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
#endif
	int spec_by_param = 0;
//	struct device *dev = prd->dev;
	/* spec reading */
	char lower_str[64] = {0, };
	char upper_str[64] = {0, };
	int16_t *rawdata_buf[MAX_TEST_CNT] = {
		[0] = prd->m2_buf_even_rawdata,
		[1] = prd->m2_buf_odd_rawdata,
	};
	int row_size = param->row;
	int col_size = param->col;
	int lower = 0, upper = 0;
	int invalid = 0;
	int test_cnt = 0;
	int sel_m1 = 0;
	int opt = 1;
	int ret = 0;

	if ((type >= UX_INVALID) && !prd_cmp_tool_str[type]) {
		t_prd_err(prd, "invalid type, %d\n", type);
		return -EINVAL;
	}

	snprintf(lower_str, sizeof(lower_str), prd_cmp_tool_str[type][0]);
	snprintf(upper_str, sizeof(upper_str), prd_cmp_tool_str[type][1]);

	switch (type) {
	case U3_M2_RAWDATA_TEST:
		/* fall through */
	case U0_M2_RAWDATA_TEST:
		/* fall through */
	case U3_BLU_JITTER_TEST:
		/* fall through */
	case U3_JITTER_TEST:
		break;

	case U0_JITTER_TEST:
		sel_m1 = !!(param->lpwg_sd_test_flag & U0_JITTER_M1_FLAG);
		break;

	case U0_M1_RAWDATA_TEST:
		sel_m1 = 1;
		break;

	default:
		t_prd_err(prd, "unsupported test mode, %d\n", type);
		return -EINVAL;
	}


	if (sel_m1) {
		row_size = param->row;
		col_size = param->m1_col;
		rawdata_buf[0] = prd->m1_buf_even_rawdata;
		rawdata_buf[1] = prd->m1_buf_odd_rawdata;
		test_cnt = param->m1_cnt;
		opt = 0;
	} else {
		test_cnt = param->m2_cnt;
	}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	switch (type) {
	case U3_M2_RAWDATA_TEST:
		lower = sd_param->u3_m2[0];
		upper = sd_param->u3_m2[1];
		break;
	case U0_M2_RAWDATA_TEST:
		lower = sd_param->u0_m2[0];
		upper = sd_param->u0_m2[1];
		break;
	case U0_M1_RAWDATA_TEST:
		lower = sd_param->u0_m1[0];
		upper = sd_param->u0_m1[1];
		break;
	case U3_BLU_JITTER_TEST:
		lower = sd_param->u3_blu_jitter[0];
		upper = sd_param->u3_blu_jitter[1];
		break;
	case U3_JITTER_TEST:
		lower = sd_param->u3_jitter[0];
		upper = sd_param->u3_jitter[1];
		break;
	case U0_JITTER_TEST:
		lower = sd_param->u0_jitter[0];
		upper = sd_param->u0_jitter[1];
		break;
	}
	if (lower || upper) {
		spec_by_param = (1<<16);
	}
#endif

	if (spec_by_param) {
		t_prd_info(prd, "type %d, skip reading spec file\n", type);
		goto skip_file;
	}

#if !defined(__SIW_SUPPORT_PRD_SET_SD_ONLY)
	ret = prd_get_limit(prd,
				lower_str, &lower);
	if (ret < 0) {
		return ret;
	}

	ret = prd_get_limit(prd,
				upper_str, &upper);
	if (ret < 0) {
		return ret;
	}
#endif

skip_file:
	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	if (invalid) {
		t_prd_err(prd, "wrong condition: type %d, lower %d, upper %d %s\n",
			type, lower, upper,
			(spec_by_param) ? STR_SPEC_BY_PARAM : "");

		siw_prd_buf_snprintf(prd->buf_write, 0,
			"wrong condition: type %d, lower %d, upper %d %s\n",
			type, lower, upper,
			(spec_by_param) ? STR_SPEC_BY_PARAM : "");

		return -EINVAL;
	}

	prd->image_lower = lower;
	prd->image_upper = upper;

	ret = prd_compare_tool(prd, test_cnt,
				rawdata_buf, row_size, col_size, type,
				opt | spec_by_param);

	return ret;
}

/*
 * SIW TOUCH IC F/W Stop HandShake
 */
static int prd_stop_firmware(struct siw_hal_prd_data *prd, u32 wdata, int flag)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 read_val;
	u32 check_data = 0;
	int try_cnt = 0;
	int ret = 0;

	/*
	 * STOP F/W to check
	 */
	ret = siw_hal_write_value(dev, reg->prd_ic_ait_start_reg, wdata);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#if 0
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#endif
	t_prd_info_flag(prd, flag, "check_data : %x\n", check_data);

	try_cnt = 1000;
	do {
		--try_cnt;
		if (try_cnt == 0) {
			t_prd_err(prd, "[ERR] get_data->try_cnt == 0\n");
			ret = -ETIMEDOUT;
			goto out;
		}
		siw_hal_read_value(dev, reg->prd_ic_ait_data_readystatus, &read_val);
		t_prd_dbg_base_flag(prd, flag, "Read RS_IMAGE = [%x] , OK RS_IMAGE = [%x]\n",
			read_val, (u32)RS_IMAGE);
		touch_msleep(2);
	} while (read_val != (u32)RS_IMAGE);

out:
	return ret;
}

static int prd_start_firmware(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 const cmd = IT_IMAGE_NONE;
	u32 check_data = 0;
	u32 read_val = 0;
	int ret = 0;

	/* Release F/W to operate */
	ret = siw_hal_write_value(dev, reg->prd_ic_ait_start_reg, cmd);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#if 0
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_start_reg, &check_data);
	if (ret < 0) {
		goto out;
	}
#endif
	t_prd_dbg_base(prd, "check_data : %x\n", check_data);

	//for test
	ret = siw_hal_read_value(dev, reg->prd_ic_ait_data_readystatus, &read_val);
	if (ret < 0) {
		goto out;
	}
	t_prd_dbg_base(prd, "Read RS_IMAGE = [%x]\n", read_val);

out:
	return ret;
}

static int prd_read_rawdata(struct siw_hal_prd_data *prd, int type)
{
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	int __m1_frame_size = ctrl->m1_frame_size;
	int __m2_frame_size = ctrl->m2_frame_size;
	u32 raw_offset_info = 0;
	u32 raw_data_offset[MAX_TEST_CNT] = {0, };
	int16_t *buf_rawdata[MAX_TEST_CNT] = {
		[0] = prd->m2_buf_even_rawdata,
		[1] = prd->m2_buf_odd_rawdata,
	};
	int16_t *tmp_buf = prd->m1_buf_tmp;
	int16_t *raw_buf;
	int addr = reg->prd_m1_m2_raw_offset;
	int sel_m1 = 0;
	int i, j = 0;
	int ret = 0;

	if (__m1_frame_size & 0x3)
		__m1_frame_size = (((__m1_frame_size >> 2) + 1) << 2);
	if (__m2_frame_size & 0x3)
		__m2_frame_size = (((__m2_frame_size >> 2) + 1) << 2);

	ret = siw_hal_read_value(dev,
				addr,
				&raw_offset_info);
	if (ret < 0) {
		goto out;
	}

	switch (touch_chip_type(ts)) {
	case CHIP_SW1828:
		raw_data_offset[0] = RAW_OFFSET_ODD(raw_offset_info);
		raw_data_offset[1] = RAW_OFFSET_EVEN(raw_offset_info);
		break;
	default:
		raw_data_offset[0] = RAW_OFFSET_EVEN(raw_offset_info);
		raw_data_offset[1] = RAW_OFFSET_ODD(raw_offset_info);
		break;
	}
	t_prd_info(prd, "test[%04Xh] type %d, even offset %04Xh, odd offset %04Xh\n",
		addr, type, raw_data_offset[0], raw_data_offset[1]);

	switch (type) {
	case U3_BLU_JITTER_TEST:
		/* fall through */
	case U3_JITTER_TEST:
		/* fall through */
	case U3_M2_RAWDATA_TEST:
		/* fall through */
	case U0_M2_RAWDATA_TEST:
		break;

 	case U0_JITTER_TEST:
		sel_m1 = !!(param->lpwg_sd_test_flag & U0_JITTER_M1_FLAG);
		break;

	case U0_M1_RAWDATA_TEST:
		sel_m1 = 1;
		break;

	}

	if (sel_m1) {
		buf_rawdata[0] = prd->m1_buf_even_rawdata;
		buf_rawdata[1] = prd->m1_buf_odd_rawdata;

		for (i = 0; i < param->m1_cnt; i++) {
			raw_buf = buf_rawdata[i];

			if (raw_buf == NULL) {
				t_prd_err(prd, "reading rawdata(%d) failed: NULL buf\n", type);
				ret = -EFAULT;
				goto out;
			}

			/* raw data offset write */
			ret = siw_hal_write_value(dev,
						reg->serial_data_offset,
						raw_data_offset[i]);
			if (ret < 0) {
				goto out;
			}

			/* raw data read */
			memset(raw_buf, 0, __m1_frame_size);
			memset(tmp_buf, 0, __m1_frame_size);

			ret = siw_hal_reg_read(dev,
						reg->data_i2cbase_addr,
						(void *)tmp_buf, __m1_frame_size);
			if (ret < 0) {
				goto out;
			}

			for (j = 0; j < param->row; j++) {
				raw_buf[j<<1]   = tmp_buf[j];
				raw_buf[(j<<1)+1] = tmp_buf[param->row+j];
			}

		}
	} else {
		for (i = 0; i < param->m2_cnt; i++) {
			/* raw data offset write */
			ret = siw_hal_write_value(dev,
						reg->serial_data_offset,
						raw_data_offset[i]);
			if (ret < 0) {
				goto out;
			}

			if (buf_rawdata[i] == NULL) {
				t_prd_err(prd, "reading rawdata(%d) failed: NULL buf\n", type);
				ret = -EFAULT;
				goto out;
			}

			/* raw data read */
			memset(buf_rawdata[i], 0, __m2_frame_size);

			ret = siw_hal_reg_read(dev,
						reg->data_i2cbase_addr,
						(void *)buf_rawdata[i], __m2_frame_size);
			if (ret < 0) {
				goto out;
			}
		}
	}

out:
	return ret;
}

static void prd_tune_display(struct siw_hal_prd_data *prd, char *tc_tune_code,
			int offset, int type, int result_on)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
//	struct device *dev = prd->dev;
	char *log_buf = prd->log_buf;
	char *temp;
	int code_size = tune->code_size;
	int size = 0;
	int i = 0;

	temp = kzalloc(code_size, GFP_KERNEL);
	if (temp == NULL) {
		t_prd_err(prd, "tune_display failed: NULL buf\n");
		return;
	}

	switch (type) {
	case PRD_TUNE_DISPLAY_TYPE_1:
		size = snprintf(log_buf, code_size,
					"GOFT tune_code_read : ");
		if ((tc_tune_code[offset] >> 4) == 1) {
			temp[offset] = tc_tune_code[offset] - (0x1 << 4);
			size += snprintf(log_buf + size,
						code_size - size,
						" %d  ", temp[offset]);
		} else {
			size += snprintf(log_buf + size,
						code_size - size,
						"-%d  ", tc_tune_code[offset]);
		}
		t_prd_info(prd, "%s\n", log_buf);
		size += snprintf(log_buf + size, code_size - size, "\n");
		if (result_on == RESULT_ON) {
			prd_write_file(prd, log_buf, TIME_INFO_SKIP);
		}
		break;
	case PRD_TUNE_DISPLAY_TYPE_2:
		size = snprintf(log_buf, code_size,
					"LOFT tune_code_read : ");
		for (i = 0; i < tune->ch; i++) {
			if ((tc_tune_code[offset+i]) >> 5 == 1) {
				temp[offset+i] =
					tc_tune_code[offset+i] - (0x1 << 5);
				size += snprintf(log_buf + size,
							code_size - size,
							" %d  ", temp[offset+i]);
			} else {
				size += snprintf(log_buf + size,
							code_size - size,
							"-%d  ",
							tc_tune_code[offset+i]);
			}
		}
		t_prd_info(prd, "%s\n", log_buf);
		size += snprintf(log_buf + size, code_size - size, "\n");
		if (result_on == RESULT_ON) {
			prd_write_file(prd, log_buf, TIME_INFO_SKIP);
		}
		break;
	default:
		t_prd_err(prd, "unknown tune type\n");
		break;
	}

	kfree(temp);
}

/*
*	tune code result check
*/
static int prd_do_read_tune_code(struct siw_hal_prd_data *prd, u8 *buf, int type, int result_on)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 tune_code_offset;
	u32 offset;
	int ret = 0;

	ret = siw_hal_read_value(dev,
				reg->prd_tune_result_offset,
				&tune_code_offset);
	if (ret < 0) {
		goto out;
	}
	offset = (tune_code_offset >> 16) & 0xFFFF;

	t_prd_info(prd, "tune_code_offset = %Xh", offset);
//	t_prd_dbg_base(prd, "tune_code_offset = %Xh", offset);

	ret = siw_hal_write_value(dev,
				reg->serial_data_offset,
				offset);
	if (ret < 0) {
		goto out;
	}
	ret = siw_hal_reg_read(dev,
				reg->data_i2cbase_addr,
				(void *)buf, tune->code_size);
	if (ret < 0) {
		goto out;
	}

	if (result_on == RESULT_ON) {
		prd_write_file(prd, "\n[Read Tune Code]\n", TIME_INFO_SKIP);
	}

	switch (type) {
	case U0_M1_RAWDATA_TEST:
		prd_tune_display(prd, buf,
					tune->code_l_goft_offset,
					PRD_TUNE_DISPLAY_TYPE_1,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_goft_offset,
					PRD_TUNE_DISPLAY_TYPE_1,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_l_m1_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_m1_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		break;
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
		prd_tune_display(prd, buf,
					tune->code_l_goft_offset + 1,
					PRD_TUNE_DISPLAY_TYPE_1,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_goft_offset + 1,
					PRD_TUNE_DISPLAY_TYPE_1,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_l_g1_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_l_g2_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_l_g3_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_g1_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_g2_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		prd_tune_display(prd, buf,
					tune->code_r_g3_oft_offset,
					PRD_TUNE_DISPLAY_TYPE_2,
					result_on);
		break;
	}
	if (result_on == RESULT_ON) {
		prd_write_file(prd, "\n", TIME_INFO_SKIP);
	}

out:
	return ret;
}

static int prd_read_tune_code(struct siw_hal_prd_data *prd, int type, int result_on)
{
	struct siw_hal_prd_tune *tune = &prd->tune;
	u8 *buf;
	int ret;

	buf = kzalloc(tune->code_size, GFP_KERNEL);
	if (buf == NULL) {
		t_prd_err(prd, "reading tune code failed: NULL buf\n");
		return -ENOMEM;
	}

	ret = prd_do_read_tune_code(prd, buf, type, result_on);

	kfree(buf);

	return ret;
}

/*
 * print Frame Data & Compare rawdata & save result
 * return result Pass:0 Fail:1
 */
static int prd_conrtol_rawdata_result(struct siw_hal_prd_data *prd, int type, int result_on)
{
	struct siw_hal_prd_param *param = &prd->param;
	int print_type[2] = {0, };
	int i,test_cnt;
	int sel_m1 = 0;
	int opt = 1;
	int size = 0;
	int result = 0;

	switch (type) {
	case U3_M2_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
	case U3_BLU_JITTER_TEST:
	case U3_JITTER_TEST:
		break;

	case U0_JITTER_TEST:
		sel_m1 = !!(param->lpwg_sd_test_flag & U0_JITTER_M1_FLAG);
		break;

	case U0_M1_RAWDATA_TEST:
		sel_m1 = 1;
		break;

	default:
		t_prd_err(prd, "conrtol_rawdata_result Type not defined, %d\n", type);
		return 1;
	}

	if (sel_m1) {
		print_type[0] = M1_EVEN_DATA;
		print_type[1] = M1_ODD_DATA;
		test_cnt = param->m1_cnt;
		opt = 0;
	} else {
		print_type[0] = M2_EVEN_DATA;
		print_type[1] = M2_ODD_DATA;
		test_cnt = param->m2_cnt;
	}

#if 1
	/* print Raw Data */
	for (i = 0; i < test_cnt; i++) {
		size = prd_print_rawdata(prd, prd->buf_write, print_type[i], size, opt);
	}
	if (size)
		size += siw_prd_buf_snprintf(prd->buf_write, size, "\n\n");

	if (result_on == RESULT_ON) {
		/* Frame Data write to Result File */
		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);

		memset(prd->buf_write, 0, PRD_BUF_SIZE);

		/* rawdata compare result(pass : 0 fail : 1) */
		result = prd_compare_rawdata(prd, type);
		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	}
#else
	/* print Raw Data */
	for (i = 0; i < test_cnt; i++) {
		size = prd_print_rawdata(prd, prd->buf_write, print_type[i], size, opt);

		if (size)
			size += siw_prd_buf_snprintf(prd->buf_write, size, "\n\n");

		if (result_on == RESULT_ON) {
			/* Frame Data write to Result File */
			prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);

			memset(prd->buf_write, 0, PRD_BUF_SIZE);
		}
	}

	if (result_on == RESULT_ON) {
		/* rawdata compare result(pass : 0 fail : 1) */
		result = prd_compare_rawdata(prd, type);
		prd_write_file(prd, prd->buf_write, TIME_INFO_SKIP);
	}
#endif

	return result;
}

/*
 * Index "result_on" -> Save Result File ON: 1 , OFF: 0
 * return - Pass: 0 , Fail: 1
 */
static int prd_do_rawdata_test(struct siw_hal_prd_data *prd,
				int type, int result_on)
{
//	struct device *dev = prd->dev;
	char *info_str = NULL;
	char *sprt_str = NULL;
	char test_type[32] = {0, };
	int ret = 0;

	switch (type) {
	case U3_M2_RAWDATA_TEST:
		info_str = "========U3_M2_RAWDATA_TEST========";
		if (result_on) {
			sprt_str = "[U3_M2_RAWDATA_TEST]";
		}
		break;
	case U3_BLU_JITTER_TEST:
		info_str = "========U3_BLU_JITTER_TEST========";
		if (result_on) {
			sprt_str= "[U3_BLU_JITTER_TEST]";
		}
		break;
	case U3_JITTER_TEST:
		info_str = "========U3_JITTER_TEST========";
		if (result_on) {
			sprt_str= "[U3_JITTER_TEST]";
		}
		break;
	case U0_JITTER_TEST:
		info_str = "========U0_JITTER_TEST========";
		if (result_on) {
			sprt_str= "[U0_JITTER_TEST]";
		}
		break;

	case U0_M2_RAWDATA_TEST:
		info_str = "========U0_M2_RAWDATA_TEST========";
		if (result_on) {
			sprt_str = "[U0_M2_RAWDATA_TEST]";
		}
		break;
	case U0_M1_RAWDATA_TEST:
		info_str = "========U0_M1_RAWDATA_TEST========";
		if (result_on) {
			sprt_str = "[U0_M1_RAWDATA_TEST]";
		}
		break;

	default:
		t_prd_err(prd, "Test Type not defined, %d\n", type);
		return 1;
	}

	if (info_str) {
		t_prd_info(prd, "%s\n", info_str);
	}
	if (sprt_str) {
		snprintf(test_type, sizeof(test_type), "\n\n%s\n", sprt_str);
		/* Test Type Write */
		prd_write_file(prd, test_type, TIME_INFO_SKIP);
	}

	/* Test Start & Finish Check */
	ret = prd_write_test_mode(prd, type);
	if (ret < 0) {
		return ret;
	}
	if (!ret) {
		return 1;
	}

	/* Read Frame Data */
	prd_read_rawdata(prd, type);

	/* Compare Spec & Print & Save result*/
	ret = prd_conrtol_rawdata_result(prd, type, result_on);

	/* tune code result check */
	switch (type) {
	case U3_M2_RAWDATA_TEST:
	case U3_M1_RAWDATA_TEST:
	case U0_M2_RAWDATA_TEST:
	case U0_M1_RAWDATA_TEST:
		prd_read_tune_code(prd, type, result_on);
		break;
	default:
		break;
	}

	return ret;
}

static int prd_rawdata_test(struct siw_hal_prd_data *prd,
				int type, int result_on)
{
	int ret;

	ret = prd_do_rawdata_test(prd, type, result_on);
	if (ret < 0) {
		t_prd_err(prd, "prd_rawdata_test(%d) failed, %d\n",
				type, ret);
	}

	return ret;
}

static void prd_firmware_version_log(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_fw_info *fw = &chip->fw;
	char *log_buf = prd->log_buf;
	int boot_mode = 0;
	int ret = 0;

	memset(log_buf, 0, sizeof(prd->log_buf));

	boot_mode = siw_touch_boot_mode_check(dev);
	if (boot_mode >= MINIOS_MFTS_FOLDER)
		ret = prd_chip_info(dev);

	ret = snprintf(log_buf, PRD_LOG_BUF_SIZE,
				"======== Firmware Info ========\n");
	if (fw->version_ext) {
		ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
					"version : %08X\n",
					fw->version_ext);
	} else {
		ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
					"version : v%d.%02d\n",
					fw->v.version.major,
					fw->v.version.minor);
	}
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"revision : %d\n",
				fw->revision);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"product id : %s\n",
				fw->product_id);

	prd_write_file(prd, log_buf, TIME_INFO_SKIP);
}

static void prd_ic_run_info_print(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	char *log_buf = prd->log_buf;
	int ret = 0;
	u32 rdata[4] = {0};

	memset(log_buf, 0, sizeof(prd->log_buf));

	ret = siw_hal_reg_read(dev,
				reg->info_lot_num,
				(void *)&rdata, sizeof(rdata));

	ret = snprintf(log_buf, PRD_LOG_BUF_SIZE,
				"\n===== Production Info =====\n");
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"lot : %d\n", rdata[0]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"serial : 0x%X\n", rdata[1]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"date : 0x%X 0x%X\n",
				rdata[2], rdata[3]);
	ret += snprintf(log_buf + ret, PRD_LOG_BUF_SIZE - ret,
				"date : %04d.%02d.%02d %02d:%02d:%02d Site%d\n",
				rdata[2] & 0xFFFF, (rdata[2] >> 16 & 0xFF),
				(rdata[2] >> 24 & 0xFF), rdata[3] & 0xFF,
				(rdata[3] >> 8 & 0xFF),
				(rdata[3] >> 16 & 0xFF),
				(rdata[3] >> 24 & 0xFF));

	prd_write_file(prd, log_buf, TIME_INFO_SKIP);
}


static int prd_ic_exception_check(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	int boot_mode = 0;

	boot_mode = siw_touch_boot_mode_check(dev);

	return boot_mode;
}

static int prd_write_test_control(struct siw_hal_prd_data *prd, u32 mode)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_hal_reg *reg = chip->reg;
	u32 test_mode_enter_cmt = mode;
	u32 test_mode_enter_check = -1;
	unsigned int delay_ms = 30;
	int addr = reg->prd_tc_test_mode_ctl;
	int ret = 0;

	ret = siw_hal_write_value(dev,
				addr,
				test_mode_enter_cmt);
	if (ret < 0) {
		goto out;
	}
	t_prd_info(prd, "wr prd_tc_test_mode_ctl[%04Xh] = %d\n",
		addr, test_mode_enter_cmt);

	touch_msleep(delay_ms);

	ret = siw_hal_read_value(dev,
				addr,
				&test_mode_enter_check);
	if (ret < 0) {
		goto out;
	}
	t_prd_info(prd, "rd prd_tc_test_mode_ctl[%04Xh] = %d\n",
		addr, test_mode_enter_check);

out:
	return ret;
}

#define __PRD_LOG_VIA_SHELL

static int prd_show_do_sd(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	struct siw_hal_prd_param *param = &prd->param;
	int rawdata_ret = 0;
	int openshort_ret = 0;
	int blu_jitter_ret = 0;
	int u3_jitter_ret = 0;
	int size = 0;
	int ret;

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	/*
	 * U3_M2_RAWDATA_TEST
	 * rawdata - pass : 0, fail : 1
	 * rawdata tunecode - pass : 0, fail : 2
	 */
	if(param->sd_test_flag & U3_M2_RAWDATA_TEST_FLAG) {
		rawdata_ret = prd_rawdata_test(prd, U3_M2_RAWDATA_TEST, RESULT_ON);
		if (rawdata_ret < 0) {
			goto out;
		}
	}

	/*
	U3_JITTER_TEST
	BLU Jitter - pass : 0, fail : 1
	This will be enabled later.
	*/
	if(param->sd_test_flag & U3_JITTER_TEST_FLAG) {
		u3_jitter_ret = prd_rawdata_test(prd, U3_JITTER_TEST, RESULT_ON);
		if (u3_jitter_ret < 0) {
			goto out;
		}
	}

	/*
	 * U3_BLU_JITTER_TEST
	 * BLU Jitter - pass : 0, fail : 1
	 * : if you want to BLU_JITTER_TEST You should be able to control LCD backlightness
	 *   BLU_JITTER_TEST Start CMD Write ->
	 *   LCD brightness ON 742ms -> LCD brightness OFF 278mms -> LCD brightness ON 278ms ->
	 *   Read RawData & Compare with BLU Jitter Spec
	 */
	if(param->sd_test_flag & U3_BLU_JITTER_TEST_FLAG) {
		blu_jitter_ret = prd_rawdata_test(prd, U3_BLU_JITTER_TEST, RESULT_ON);
		if (blu_jitter_ret < 0) {
			goto out;
		}
	 }

	/*
	 * OPEN_SHORT_ALL_TEST
	 * open - pass : 0, fail : 1
	 * short - pass : 0, fail : 2
	 */
	if(param->sd_test_flag & OPEN_SHORT_NODE_TEST_FLAG) {
		openshort_ret = prd_open_short_test(prd);
		if (openshort_ret < 0) {
			goto out;
		}
	}

	size += siw_snprintf(buf, size,
				"\n========RESULT=======\n");
	if(param->sd_test_flag & U3_M2_RAWDATA_TEST_FLAG) {
		if (!rawdata_ret) {
			size += siw_snprintf(buf, size,
						"Raw Data : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"Raw Data : Fail\n");
		}
	}


	if(param->sd_test_flag & U3_JITTER_TEST_FLAG) {
		if (!u3_jitter_ret) {
			size += siw_snprintf(buf, size,
						"U3 Jitter : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"U3 Jitter : Fail\n");
		}
	}

	if(param->sd_test_flag & U3_BLU_JITTER_TEST_FLAG) {
		if (!blu_jitter_ret) {
			size += siw_snprintf(buf, size,
						"Blu Jitter : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"Blu Jitter : Fail\n");
		}
	}


	if(param->sd_test_flag & OPEN_SHORT_NODE_TEST_FLAG) {
		if (!openshort_ret) {
			size += siw_snprintf(buf, size,
						"Channel Status : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"Channel Status : Fail (open:%s/short:%s)\n",
						((openshort_ret & 0x1) == 0x1) ? "F" : "P",
						((openshort_ret & 0x2) == 0x2) ? "F" : "P");
		}
	}

	prd_write_file(prd, buf, TIME_INFO_SKIP);
	t_prd_info(prd, "%s \n", buf);
	prd_write_test_control(prd, CMD_TEST_EXIT);

out:
	return size;
}

static ssize_t prd_show_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;
	int ret = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_snprintf(buf, size,
					"LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}

	/* ic rev check - MINIOS mode, MFTS mode check */
	ret = prd_ic_exception_check(prd, buf);
	if (ret > 0) {
		t_prd_err(prd, "ic exception(%d) detected, test canceled\n", ret);
		size += siw_snprintf(buf, size,
					"ic exception(%d) detected, test canceled\n", ret);
		goto out;
	}

	siw_touch_mon_pause(dev);

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	/* file create , time log */
	prd_write_file(prd, "\nShow_sd Test Start", TIME_INFO_SKIP);
	prd_write_file(prd, "\n", TIME_INFO_WRITE);

	t_prd_info(prd, "show_sd test begins\n");

	prd_firmware_version_log(prd);
	prd_ic_run_info_print(prd);

	mutex_lock(&ts->lock);
	size = prd_show_do_sd(prd, buf);
	mutex_unlock(&ts->lock);

	prd_write_file(prd, "Show_sd Test End\n", TIME_INFO_WRITE);
	prd_log_file_size_check(prd);

	t_prd_info(prd, "show_sd test terminated\n\n");

	prd_chip_driving(dev, LCD_MODE_U3);
	prd_chip_reset(dev);

	siw_touch_mon_resume(dev);

out:
	return (ssize_t)size;
}

static int prd_show_prd_get_data_raw_core(struct device *dev,
					u8*buf, int size,
					u32 cmd, u32 offset, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	if (!offset) {
		t_prd_err(prd, "raw core(cmd %d) failed: zero offset\n", cmd);
		ret = -EFAULT;
		goto out;
	}

	if (!size) {
		t_prd_err(prd, "raw core(cmd %d) failed: zero size\n", cmd);
		ret = -EFAULT;
		goto out;
	}

	if (buf == NULL) {
		t_prd_err(prd, "raw core(cmd %d) failed: NULL buf\n", cmd);
		ret = -EFAULT;
		goto out;
	}

	if (cmd != IT_DONT_USE_CMD) {
		ret = prd_stop_firmware(prd, cmd, flag);
		if (ret < 0) {
			goto out;
		}
	}

	ret = siw_hal_write_value(dev,
				reg->serial_data_offset,
				offset);
	if (ret < 0) {
		goto out;
	}

	memset(buf, 0, size);

	ret = siw_hal_reg_read(dev, reg->data_i2cbase_addr,
					(void *)buf,
					size);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_raw_prd(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	t_prd_info(prd, "======== CMD_RAWDATA_PRD ========\n");

	mutex_lock(&ts->lock);

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	if (chip->lcd_mode == LCD_MODE_U3) {
		ret = prd_rawdata_test(prd, U3_M2_RAWDATA_TEST, RESULT_OFF);
	} else if (chip->lcd_mode == LCD_MODE_U0) {
		ret = prd_rawdata_test(prd, U0_M2_RAWDATA_TEST, RESULT_OFF);
	} else {
		t_prd_err(prd, "LCD mode is not U3 or U0!! current mode = %d\n",
				chip->lcd_mode);
		ret = -EINVAL;
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	if (ret < 0) {
		goto out;
	}

	ret = prd_write_test_control(prd, CMD_TEST_EXIT);
	if (ret < 0) {
		goto out;
	}

out:
	mutex_unlock(&ts->lock);

	prd_chip_driving(dev, LCD_MODE_U3);
	prd_chip_reset(dev);

	return ret;
}

static int prd_show_prd_get_data_raw_tcm(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int __m2_frame_size = ctrl->m2_frame_size;
	void *buf = prd->m2_buf_even_rawdata;
	int buf_size = prd->ctrl.m2_row_col_buf_size;
	int log_size = 0;
	int ret = 0;

	/* 	LCD mode check 	*/
#if 0
	if (chip->lcd_mode != LCD_MODE_U3) {
		t_prd_info(prd, "LCD mode is not U3. Test Result : Fail\n");
		goto out;
	}
#endif

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== rawdata(tcm) ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== rawdata(tcm) ========\n");
#else
	t_prd_info(prd, "======== CMD_RAWDATA_TCM ========\n");
#endif

	if (buf == NULL) {
		t_prd_err(prd, "getting raw tcm failed: NULL buf\n");
		return -EFAULT;
	}

	/* TCM Offset write 0 */
	ret = siw_hal_write_value(dev, reg->prd_serial_tcm_offset, 0);
	if (ret < 0) {
		goto out;
	}

	/*
	 * TCM Memory Access Select
	 * tc_mem_sel(0x0457) "RAW" or "BASE1" or "BASE2" or "BASE3"
	 */
	ret = siw_hal_write_value(dev, reg->prd_tc_mem_sel, TCM_MEM_RAW);
	if (ret < 0){
		goto out;
	}

	/* 	Read Rawdata	*/
	memset(buf, 0, buf_size);
	ret = siw_hal_reg_read(dev,
				reg->prd_tcm_base_addr,
				buf,
				__m2_frame_size);
	if (ret < 0){
		goto out;
	}

	/*	Print RawData Buffer	*/
	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

out:
	return ret;
}

static int prd_show_prd_get_data_do_raw_ait(struct device *dev, u8*buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->m2_buf_even_rawdata;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				prd->img_cmd.raw, prd->img_offset.raw, flag);
}

static int prd_show_prd_get_data_raw_ait(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== rawdata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== rawdata ========\n");
#else
	t_prd_info(prd, "======== CMD_RAWDATA_AIT ========\n");
#endif

	ret = prd_show_prd_get_data_do_raw_ait(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

	ret = prd_start_firmware(prd);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_ait_basedata(struct device *dev,
					u8*buf, int size, int step, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u32 ait_cmd[MAX_TEST_CNT] = {prd->img_cmd.baseline_even, \
								prd->img_cmd.baseline_odd};
	u32 ait_offset[MAX_TEST_CNT] = {prd->img_offset.baseline_even, \
									prd->img_offset.baseline_odd};
	int16_t *buf_rawdata[MAX_TEST_CNT] = { \
		[0] = prd->m2_buf_even_rawdata, \
		[1] = prd->m2_buf_odd_rawdata, \
	};
	u8 *pbuf = (buf) ? buf : (u8 *)buf_rawdata[step];

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				ait_cmd[step], ait_offset[step], flag);
}

static int prd_show_prd_get_data_ait_basedata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u32 buf_type[MAX_TEST_CNT] = {M2_EVEN_DATA, M2_ODD_DATA};
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int i = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== basedata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== basedata ========\n");
#else
	t_prd_info(prd, "======== CMD_AIT_BASEDATA ========\n");
#endif

	for (i = 0; i < param->m2_cnt; i++) {
		ret = prd_show_prd_get_data_do_ait_basedata(dev,
					NULL,
					size,
					i,
					0);
		if (ret < 0) {
			goto out;
		}

		log_size = prd_print_rawdata(prd, prd->buf_write, buf_type[i], log_size, 0);
	#if !defined(__PRD_LOG_VIA_SHELL)
		log_size = 0;
	#endif

		ret = prd_start_firmware(prd);
		if (ret < 0) {
			goto out;
		}
	}

out:
	return ret;
}

static int prd_show_prd_get_data_do_filtered_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_even_rawdata;
	int size_rd = (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				prd->img_cmd.f_delta, prd->img_offset.f_delta, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_delta[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_filtered_deltadata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== filtered deltadata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== filtered deltadata ========\n");
#else
	t_prd_info(prd, "======== CMD_FILTERED_DELTADATA ========\n");
#endif

	ret = prd_show_prd_get_data_do_filtered_deltadata(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

	ret = prd_start_firmware(prd);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
};

static int prd_show_prd_get_data_do_deltadata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int16_t *pbuf = (buf) ? (int16_t *)buf : prd->m2_buf_even_rawdata;
	int size_rd = (ctrl->delta_size<<PRD_RAWDATA_SZ_POW);
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_delta, size_rd,
				prd->img_cmd.delta, prd->img_offset.delta, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_delta[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;
}

static int prd_show_prd_get_data_deltadata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== deltadata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== deltadata ========\n");
#else
	t_prd_info(prd, "======== CMD_DELTADATA ========\n");
#endif

	ret = prd_show_prd_get_data_do_deltadata(dev,
				(u8 *)prd->m2_buf_even_rawdata,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, M2_EVEN_DATA, log_size, 0);

	ret = prd_start_firmware(prd);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;

}

static int prd_show_prd_get_data_do_labeldata(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *pbuf = (buf) ? buf : prd->buf_label;
	int size_rd = ctrl->label_tmp_size;
	int param_col = param->col;
	int row, col;
	int i;
	int ret = 0;

	ret = prd_show_prd_get_data_raw_core(dev, (u8 *)prd->buf_label_tmp, size_rd,
				prd->img_cmd.label, prd->img_offset.label, flag);
	if (ret < 0) {
		goto out;
	}

	memset(pbuf, 0, size);

	for (i = 0; i < ctrl->m2_row_col_size; i++){
		row = i / param_col;
		col = i % param_col;
		pbuf[i] = prd->buf_label_tmp[(row + 1)*(param_col + 2) + (col + 1)];
	}

out:
	return ret;

}

static int prd_show_prd_get_data_labeldata(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = ctrl->m2_row_col_size;
	int log_size = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== labeldata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== labeldata ========\n");
#else
	t_prd_info(prd, "======== CMD_LABELDATA ========\n");
#endif

	ret = prd_show_prd_get_data_do_labeldata(dev,
				(u8 *)prd->buf_label,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, LABEL_DATA, log_size, 0);

	ret = prd_start_firmware(prd);
	if (ret < 0) {
		goto out;
	}

out:
	return ret;
}

static int prd_show_prd_get_data_blu_jitter(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	t_prd_info(prd, "======== CMD_BLU_JITTER ========\n");

	mutex_lock(&ts->lock);

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		goto out;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	ret = prd_rawdata_test(prd, U3_BLU_JITTER_TEST, RESULT_OFF);

	siw_touch_irq_control(dev, INTERRUPT_ENABLE);

	if (ret < 0) {
		goto out;
	}

	ret = prd_write_test_control(prd, CMD_TEST_EXIT);
	if (ret < 0) {
		goto out;
	}

out:
	mutex_unlock(&ts->lock);

	prd_chip_driving(dev, LCD_MODE_U3);

	prd_chip_reset(dev);

	return ret;
}

static int prd_show_prd_get_data_do_debug_buf(struct device *dev, u8 *buf, int size, int flag)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u8 *pbuf = (buf) ? buf : (u8 *)prd->buf_debug;

	return prd_show_prd_get_data_raw_core(dev, pbuf, size,
				IT_DONT_USE_CMD, prd->img_offset.debug, flag);
}

static int prd_show_prd_get_data_debug_buf(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	int size = (ctrl->debug_buf_size<<PRD_RAWDATA_SZ_POW);
	int log_size = 0;
	int ret = 0;

#if defined(__PRD_LOG_VIA_SHELL)
	t_prd_info(prd, "======== debugdata ========\n");
	log_size += siw_prd_buf_snprintf(prd->buf_write, log_size,
					"======== debugdata ========\n");
#else
	t_prd_info(prd, "======== CMD_DEBUGDATA ========\n");
#endif

	ret = prd_show_prd_get_data_do_debug_buf(dev,
				(u8 *)prd->buf_debug,
				size,
				0);
	if (ret < 0) {
		goto out;
	}

	prd_print_rawdata(prd, prd->buf_write, DEBUG_DATA, log_size, 0);

out:
	return ret;
}


static ssize_t prd_show_prd_get_data(struct device *dev, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	switch (type) {
	case CMD_RAWDATA_PRD:
		ret = prd_show_prd_get_data_raw_prd(dev);
		break;
	case CMD_RAWDATA_TCM:
		ret = prd_show_prd_get_data_raw_tcm(dev);
		break;
	case CMD_RAWDATA_AIT:
		ret = prd_show_prd_get_data_raw_ait(dev);
		break;
	case CMD_AIT_BASEDATA:
		ret = prd_show_prd_get_data_ait_basedata(dev);
		break;
	case CMD_FILTERED_DELTADATA:
		ret = prd_show_prd_get_data_filtered_deltadata(dev);
		break;
	case CMD_DELTADATA:
		ret = prd_show_prd_get_data_deltadata(dev);
		break;
	case CMD_LABELDATA:
		ret = prd_show_prd_get_data_labeldata(dev);
		break;
	case CMD_BLU_JITTER:
		ret = prd_show_prd_get_data_blu_jitter(dev);
		break;
	case CMD_DEBUG_BUF:
		ret = prd_show_prd_get_data_debug_buf(dev);
		break;
	default:
		t_prd_err(prd, "invalid get_data request CMD");
		ret = -EINVAL;
		break;
	};

	return ret;
}

static ssize_t prd_show_get_data_common(struct device *dev, char *buf, int type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;
	int ret = 0;

	siw_touch_mon_pause(dev);
	ret = prd_show_prd_get_data(dev, type);
	siw_touch_mon_resume(dev);
	if (ret < 0){
		t_prd_err(prd, "prd_show_prd_get_data(%d) failed, %d\n",
			type, ret);
	}

	/*
	 * to prepare the response for APP I/F (not fixed)
	 */
#if defined(__PRD_LOG_VIA_SHELL)
	switch (type) {
	case CMD_BLU_JITTER:
		break;
	default:
		size += siw_snprintf(buf, size, "%s\n", prd->buf_write);
		break;
	}
#endif
	size += siw_snprintf(buf, size, "Get Data[%s] result:\n",
				prd_get_data_cmd_name[type]);
	size += siw_snprintf(buf, size, "%s\n",
				(ret < 0) ? "Fail" : "Pass");
	return (ssize_t)size;
}

static ssize_t prd_show_delta(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_DELTADATA);
}

static ssize_t prd_show_label(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return (ssize_t)size;
	}
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_LABELDATA);
}

static ssize_t prd_show_rawdata_prd(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_PRD);
}

static ssize_t prd_show_rawdata_tcm(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_TCM);
}

static ssize_t prd_show_rawdata_ait(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return (ssize_t)size;
	}
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_RAWDATA_AIT);
}

static ssize_t prd_show_debug_buf(struct device *dev, char *buf)
{
	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_DEBUG_BUF);
}

static ssize_t prd_show_basedata(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U3) {
		size += siw_snprintf(buf, size,
					"Current LCD mode(%d) is not U3, halted\n",
					chip->lcd_mode);
		return size;
	}

	return (ssize_t)prd_show_get_data_common(dev, buf, CMD_AIT_BASEDATA);
}

static int prd_show_do_lpwg_sd(struct siw_hal_prd_data *prd, char *buf)
{
	struct device *dev = prd->dev;
	struct siw_hal_prd_param *param = &prd->param;
	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int u0_jitter_ret = 0;
	u32 sd_u0_test = 0;
	int size = 0;
	int ret = 0;

	ret = prd_write_test_control(prd, CMD_TEST_ENTER);
	if (ret < 0) {
		return ret;
	}

	siw_touch_irq_control(dev, INTERRUPT_DISABLE);

	ret = prd_chip_driving(dev, LCD_MODE_STOP);
	if (ret < 0) {
		goto out;
	}

	/*
	 * U0_M2_RAWDATA_TEST & U0_M1_RAWDATA_TEST
	 * rawdata - pass : 0, fail : 1
	 * rawdata tunecode - pass : 0, fail : 2
	 */
	if(param->lpwg_sd_test_flag & U0_M2_RAWDATA_TEST_FLAG) {
		m2_rawdata_ret = prd_rawdata_test(prd, U0_M2_RAWDATA_TEST, RESULT_ON);
		if (m2_rawdata_ret < 0) {
			goto out;
		}
	}

	if(param->lpwg_sd_test_flag & U0_M1_RAWDATA_TEST_FLAG) {
		m1_rawdata_ret = prd_rawdata_test(prd, U0_M1_RAWDATA_TEST, RESULT_ON);
		if (m1_rawdata_ret < 0) {
			goto out;
		}
	}

	/*
	 * U0_JITTER_TEST
	 * BLU Jitter - pass : 0, fail : 1
	 * This will be enabled later.
	*/
	if(param->lpwg_sd_test_flag & U0_JITTER_TEST_FLAG) {
		u0_jitter_ret = prd_rawdata_test(prd, U0_JITTER_TEST, RESULT_ON);
		if (u0_jitter_ret < 0) {
			goto out;
		}
	}

	size = siw_snprintf(buf, size, "========RESULT=======\n");

	sd_u0_test = param->lpwg_sd_test_flag &	\
			 (U0_M2_RAWDATA_TEST_FLAG | U0_M1_RAWDATA_TEST_FLAG);

	if (sd_u0_test) {
		if (!m1_rawdata_ret && !m2_rawdata_ret) {
			size += siw_snprintf(buf, size,
						"LPWG RawData : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"LPWG RawData : Fail ");
			switch (sd_u0_test) {
			case (U0_M2_RAWDATA_TEST_FLAG | U0_M1_RAWDATA_TEST_FLAG):
				size += siw_snprintf(buf, size,
					"(m2 %d, m1 %d)\n", m2_rawdata_ret, m1_rawdata_ret);
				break;
			case U0_M2_RAWDATA_TEST_FLAG:
				size += siw_snprintf(buf, size,
					"(m2 %d)\n", m2_rawdata_ret);
				break;
			case U0_M1_RAWDATA_TEST_FLAG:
				size += siw_snprintf(buf, size,
					"(m1 %d)\n", m1_rawdata_ret);
				break;
			}
		}
	}

	if(param->lpwg_sd_test_flag & U0_JITTER_TEST_FLAG) {
		if (!u0_jitter_ret) {
			size += siw_snprintf(buf, size,
						"U0 Jitter : Pass\n");
		} else {
			size += siw_snprintf(buf, size,
						"U0 Jitter : Fail\n");
		}
	}

	prd_write_file(prd, buf, TIME_INFO_SKIP);

out:
	prd_chip_driving(dev, LCD_MODE_U3);

	prd_chip_reset(dev);

	return size;
}

static ssize_t prd_show_lpwg_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;

	/* LCD mode check */
	if (chip->lcd_mode != LCD_MODE_U0) {
		size = siw_snprintf(buf, size,
					"LCD mode is not U0. Test Result : Fail\n");
		goto out;
	}

	siw_touch_mon_pause(dev);

	/* file create , time log */
	prd_write_file(prd, "\nShow_lpwg_sd Test Start", TIME_INFO_SKIP);
	prd_write_file(prd, "\n", TIME_INFO_WRITE);

	t_prd_info(prd, "show_lpwg_sd test begins\n");

	prd_firmware_version_log(prd);
	prd_ic_run_info_print(prd);

	mutex_lock(&ts->lock);
	size = prd_show_do_lpwg_sd(prd, buf);
	mutex_unlock(&ts->lock);

	prd_write_file(prd, "Show_lpwg_sd Test End\n", TIME_INFO_WRITE);
	prd_log_file_size_check(prd);

	t_prd_info(prd, "show_lpwg_sd test terminated\n");

	siw_touch_mon_resume(dev);

out:
	return (ssize_t)size;
}


#define __PRD_FILE_RW_TEST

#if defined(__PRD_FILE_RW_TEST)

enum {
	PRD_FILE_TEST_OPTION_0	= 0,
	PRD_FILE_TEST_OPTION_1,
	PRD_FILE_TEST_OPTION_2,
	PRD_FILE_TEST_OPTION_MAX,
};

static void __prd_show_file_test_usage(struct siw_hal_prd_data *prd)
{
	t_prd_info(prd, "[Usage]\n");
	t_prd_info(prd, " Check : echo 0 {fname} > file_test\n");
	t_prd_info(prd, " Write : echo 1 {fname} > file_test\n");
	t_prd_info(prd, "       : Remove old file and create new one\n");
	t_prd_info(prd, " Write : echo 2 {fname} > file_test\n");
	t_prd_info(prd, "       : Rename old file and create new one\n");
}

static ssize_t prd_show_file_test(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	__prd_show_file_test_usage(prd);

	return 0;
}

static ssize_t prd_store_file_test(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	char *fname = NULL;
	int value = 0;
	int ret = 0;

	fname = touch_getname();
	if (fname == NULL) {
		t_prd_err(prd, "failed to allocate fname\n");
		return -ENOMEM;
	}

	if (sscanf(buf, "%d %s", &value, fname) <= 0) {
		siw_prd_sysfs_err_invalid_param(prd);
		__prd_show_file_test_usage(prd);
		goto out;
	}
	if (value >= PRD_FILE_TEST_OPTION_MAX) {
		t_prd_err(prd, "invalude option, %d\n", value);
		__prd_show_file_test_usage(prd);
		goto out;
	}
	if (fname == NULL) {
		t_prd_err(prd, "target file name is NULL\n");
		__prd_show_file_test_usage(prd);
		goto out;
	}

	ret = prd_vfs_file_chk(prd, fname, O_RDONLY, 0666, NULL);
	if (ret < 0) {
		/* */
	}

	/* Check mode */
	if (value == PRD_FILE_TEST_OPTION_0) {
		goto out;
	}

	if (!ret) {
		if (value == PRD_FILE_TEST_OPTION_2) {	/* Rename */
			char *buf1 = NULL;
			char *buf2 = NULL;
			int i;

			buf1 = touch_getname();
			if (buf1 == NULL) {
				t_prd_err(prd, "failed to allocate name buf1\n");
				ret = -ENOMEM;
				goto out;
			}

			buf2 = touch_getname();
			if (buf2 == NULL) {
				t_prd_err(prd, "failed to allocate name buf2\n");
				ret = -ENOMEM;
				touch_putname(buf1);
				goto out;
			}

			for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
				if (i == 0) {
					snprintf(buf1, PATH_MAX, "%s", fname);
				} else {
					snprintf(buf1, PATH_MAX, "%s.%d", fname, i);
				}

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					ret = prd_vfs_file_remove(prd, buf1);
					if (ret < 0) {
						break;
					}
				} else {
					snprintf(buf2, PATH_MAX, "%s.%d", fname, (i + 1));

					ret = prd_vfs_file_rename(prd, buf1, buf2);
					if (ret < 0) {
						break;
					}
				}
			}

			touch_putname(buf2);
			touch_putname(buf1);

			if (ret < 0) {
				goto out;
			}
		} else {	/* Remove */
			ret = prd_vfs_file_remove(prd, fname);
			if (ret < 0) {
				goto out;
			}
		}
	}

	ret = prd_do_write_file(prd, fname, "[File Write Test]\n", TIME_INFO_WRITE);
	if (ret < 0) {
		t_prd_err(prd, "File Write Test failed\n");
	}

out:
	touch_putname(fname);

	return (ssize_t)count;
}
#else	/* __PRD_FILE_RW_TEST */
static ssize_t prd_show_file_test(struct device *dev, char *buf)
{
	t_dev_info(dev, "Nop ...\n");
	return 0;
}

static ssize_t prd_store_file_test(struct device *dev,
				const char *buf, size_t count)
{
	t_dev_info(dev, "Nop ...\n");
	return (ssize_t)count;
}
#endif	/* __PRD_FILE_RW_TEST */

enum {
	REPORT_END_RS_NG = 0x05,
	REPORT_END_RS_OK = 0xAA,
};

enum {
	REPORT_OFF = 0,
	REPORT_RAW,
	REPORT_BASE,
	REPORT_DELTA,
	REPORT_LABEL,
	REPORT_DEBUG_BUF,
	REPORT_MAX,
};

static const char *prd_app_mode_str[] = {
	[REPORT_OFF]		= "OFF",
	[REPORT_RAW]		= "RAW",
	[REPORT_BASE]		= "BASE",
	[REPORT_LABEL]		= "LABEL",
	[REPORT_DELTA]		= "DELTA",
	[REPORT_DEBUG_BUF]	= "DEBUG_BUF",
};

static ssize_t prd_show_app_op_end(struct device *dev, char *buf, int prev_mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int ret = 0;

	buf[0] = REPORT_END_RS_OK;
	if (prev_mode != REPORT_OFF) {
		prd->prd_app_mode = REPORT_OFF;
		ret = prd_start_firmware(prd);
		if (ret < 0) {
			t_prd_err(prd, "prd_start_firmware failed, %d\n", ret);
			buf[0] = REPORT_END_RS_NG;
		}
	}

	return 1;
}

static ssize_t prd_show_app_operator(struct device *dev, char *buf, int mode)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *pbuf = (u8 *)prd->m2_buf_even_rawdata;
	int size = (ctrl->m2_row_col_size<<PRD_RAWDATA_SZ_POW);
	int flag = PRD_SHOW_FLAG_DISABLE_PRT_RAW;
	int prev_mode = prd->prd_app_mode;
//	int ret = 0;

	if (mode < REPORT_MAX) {
		t_prd_dbg_base(prd, "show app mode : %s(%d), 0x%X\n",
				prd_app_mode_str[mode], mode, flag);
	}

	if (mode == REPORT_OFF) {
		size = prd_show_app_op_end(dev, buf, prev_mode);
		goto out;
	}

	if (mode < REPORT_MAX) {
		prd->prd_app_mode = mode;
	}

	switch (mode) {
	case REPORT_RAW:
		prd_show_prd_get_data_do_raw_ait(dev, pbuf, size, flag);
		break;
	case REPORT_BASE:
		prd_show_prd_get_data_do_ait_basedata(dev, pbuf, size, 0, flag);
		break;
	case REPORT_DELTA:
		prd_show_prd_get_data_do_deltadata(dev, pbuf, size, flag);
		break;
	case REPORT_LABEL:
		size = ctrl->m2_row_col_size;
		pbuf = (u8 *)prd->buf_label,
		prd_show_prd_get_data_do_labeldata(dev, pbuf, size, flag);
		break;
	case REPORT_DEBUG_BUF:
		size = ctrl->debug_buf_size;
		pbuf = (u8 *)prd->buf_debug,
		prd_show_prd_get_data_do_debug_buf(dev, pbuf, size, flag);
		break;
	default:
		t_prd_err(prd, "unknown mode, %d\n", mode);
		if (prev_mode != REPORT_OFF) {
			prd_show_app_op_end(dev, buf, prev_mode);
			siw_touch_mon_resume(dev);
		}
		size = 0;
		break;
	}

	if (size) {
		memcpy(buf, pbuf, size);
	}

out:
	if (touch_test_prd_quirks(ts, PRD_QUIRK_RAW_RETURN_MODE_VAL)) {
		return (ssize_t)mode;
	}

	return (ssize_t)size;
}

static ssize_t prd_show_app_raw(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_RAW);
}

static ssize_t prd_show_app_base(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_BASE);
}

static ssize_t prd_show_app_label(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_LABEL);
}

static ssize_t prd_show_app_delta(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_DELTA);
}

static ssize_t prd_show_app_debug_buf(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_DEBUG_BUF);
}

static ssize_t prd_show_app_end(struct device *dev, char *buf)
{
	return prd_show_app_operator(dev, buf, REPORT_OFF);
}

static ssize_t prd_show_app_info(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	int temp = prd->sysfs_flag;

	memset(buf, 0, PRD_APP_INFO_SIZE);

	buf[0] = (temp & 0xff);
	buf[1] = ((temp >> 8) & 0xff);
	buf[2] = ((temp >> 16) & 0xff);
	buf[3] = ((temp >> 24) & 0xff);

	buf[8] = param->row;
	buf[9] = param->col;
	buf[10] = param->col_add;
	buf[11] = param->ch;
	buf[12] = param->m1_col;
	buf[13] = param->cmd_type;
	buf[14] = param->second_scr.bound_i;
	buf[15] = param->second_scr.bound_j;

	t_prd_info(prd,
		"prd info: f %08Xh, r %d, c %d, ca %d, ch %d, m1 %d, cmd %d, bi %d, bj %d\n",
		temp, param->row, param->col, param->col_add, param->ch, param->m1_col,
		param->cmd_type, param->second_scr.bound_i, param->second_scr.bound_j);

	if (prd->mon_flag) {
		siw_touch_mon_resume(dev);
	} else {
		siw_touch_mon_pause(dev);
	}
	prd->mon_flag = !prd->mon_flag;

	return PRD_APP_INFO_SIZE;
}

static ssize_t prd_show_dbg_mask(struct device *dev, char *buf)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int size = 0;

	size += siw_snprintf(buf, size,
				"prd_dbg_mask %08Xh\n\n",
				prd->dbg_mask);

	size += siw_snprintf(buf, size,
				"Usage:\n");
	size += siw_snprintf(buf, size,
				" prd->dbg_mask  : echo 0 {mask_value} > prd_dbg_mask\n");
	size += siw_snprintf(buf, size,
				" t_prd_dbg_mask : echo 9 {mask_value} > prd_dbg_mask\n");

	return (ssize_t)size;
}

static void prd_store_dbg_mask_usage(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	t_prd_info(prd, "Usage:\n");
	t_prd_info(prd, " prd->dbg_mask  : echo 0 {mask_value(hex)} > prd_dbg_mask\n");
	t_prd_info(prd, " t_prd_dbg_mask : echo 9 {mask_value(hex)} > prd_dbg_mask\n");
}

static ssize_t prd_store_dbg_mask(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int type = 0;
	u32 old_value, new_value = 0;

	if (sscanf(buf, "%d %X", &type, &new_value) <= 0) {
		t_prd_err(prd, "Invalid param\n");
		prd_store_dbg_mask_usage(dev);
		return count;
	}

	switch (type) {
	case 0 :
		old_value = prd->dbg_mask;
		prd->dbg_mask = new_value;
		t_prd_info(prd, "prd->dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	case 9 :
		old_value = t_prd_dbg_mask;
		t_prd_dbg_mask = new_value;
		t_dev_info(dev, "t_prd_dbg_mask changed : %08Xh -> %08xh\n",
			old_value, new_value);
		break;
	default :
		prd_store_dbg_mask_usage(dev);
		break;
	}

	return count;
}

#if defined(__SIW_SUPPORT_PRD_SET_SD)
#define siw_prd_set_sd_sprintf(_buf, _title) \
		sprintf(_buf, "%-20s(%d) -", #_title, _title)

#define t_prd_info_set_sd(_prd, _title) \
		t_prd_info(_prd, "  %-20s - %d\n", #_title, _title)

static int prd_show_set_sd_item(char *buf, int size, char *title, int *value)
{
	int lower = value[0];
	int upper = value[1];
	int disabled = 0;
	int invalid = 0;

	disabled = !!(!lower && !upper);

	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	size = siw_snprintf(buf, size,
				" %s lower %d, upper %d %s\n",
				title, lower, upper,
				(disabled) ? "(disabled)" :
				(invalid) ? "(invalid)" : "(enabled)");

	return size;
}

static ssize_t prd_show_set_sd(struct device *dev, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	char title[32] = {0, };
	int valid = 0;
	int disabled = 0;
	int invalid = 0;
	int type;
	int lower, upper;
	int size = 0;

	size += siw_snprintf(buf, size, "[sd]\n");

	if(param->sd_test_flag & U3_M2_RAWDATA_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U3_M2_RAWDATA_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u3_m2);
	}
	if(param->sd_test_flag & U3_BLU_JITTER_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U3_BLU_JITTER_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u3_blu_jitter);
	}
	if(param->sd_test_flag & U3_JITTER_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U3_JITTER_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u3_jitter);
	}

	size += siw_snprintf(buf, size, "[lpwg_sd]\n");

	if(param->lpwg_sd_test_flag & U0_M2_RAWDATA_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U0_M2_RAWDATA_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u0_m2);
	}
	if(param->lpwg_sd_test_flag & U0_M1_RAWDATA_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U0_M1_RAWDATA_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u0_m1);
	}
	if(param->lpwg_sd_test_flag & U0_JITTER_TEST_FLAG) {
		siw_prd_set_sd_sprintf(title, U0_JITTER_TEST);
		size += prd_show_set_sd_item(buf, size, title, sd_param->u0_jitter);
	}

	type = sd_param->last_type;
	lower = sd_param->last_val[0];
	upper = sd_param->last_val[1];
	if ((type >=0) && (type < UX_INVALID)) {
		if (param->sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
		if (param->lpwg_sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
	}

	disabled = !!(valid && !lower && !upper);

	invalid |= !valid;
	invalid |= !!(lower < 0);
	invalid |= !!(upper < 0);
	invalid |= !!(lower >= upper);

	size += siw_snprintf(buf, size,
				"\nLast:\n");
	size += siw_snprintf(buf, size,
				" type %d, lower %d, upper %d %s\n",
				type,lower, upper,
				(disabled) ? "(disabled)" :
				(invalid) ? "(invalid)" : "(enabled)");

	size += siw_snprintf(buf, size,
				"\nUsage:\n");
	size += siw_snprintf(buf, size,
				" echo {type} {lower} {upper} > set_sd\n\n");

	return (ssize_t)size;
}

static void prd_store_set_sd_usage(struct device *dev)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;

	t_prd_info(prd, "Usage:\n");
	t_prd_info(prd, " echo {type} {lower} {upper} > set_sd\n");
	t_prd_info(prd, " [type for sd]\n");
	if(param->sd_test_flag & U3_M2_RAWDATA_TEST_FLAG) {
		t_prd_info_set_sd(prd, U3_M2_RAWDATA_TEST);
	}
	if(param->sd_test_flag & U3_BLU_JITTER_TEST_FLAG) {
		t_prd_info_set_sd(prd, U3_BLU_JITTER_TEST);
	}
	if(param->sd_test_flag & U3_JITTER_TEST_FLAG) {
		t_prd_info_set_sd(prd, U3_JITTER_TEST);
	}
	t_prd_info(prd, " [type for lpwd_sd]\n");
	if(param->lpwg_sd_test_flag & U0_M2_RAWDATA_TEST_FLAG) {
		t_prd_info_set_sd(prd, U0_M2_RAWDATA_TEST);
	}
	if(param->lpwg_sd_test_flag & U0_M1_RAWDATA_TEST_FLAG) {
		t_prd_info_set_sd(prd, U0_M1_RAWDATA_TEST);
	}
	if(param->lpwg_sd_test_flag & U0_JITTER_TEST_FLAG) {
		t_prd_info_set_sd(prd, U0_JITTER_TEST);
	}
}

static ssize_t prd_store_set_sd(struct device *dev,
				const char *buf, size_t count)
{
	struct siw_ts *ts = to_touch_core(dev);
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = &prd->param;
	struct siw_hal_prd_sd_param *sd_param = &prd->sd_param;
	int type = 0;
	int valid = 0;
	int lower = 0, upper = 0;

	if (sscanf(buf, "%d %d %d", &type, &lower, &upper) <= 0) {
		t_prd_err(prd, "Invalid param\n");
		prd_store_set_sd_usage(dev);
		return count;
	}

	if ((type >=0) && (type < UX_INVALID)) {
		if (param->sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
		if (param->lpwg_sd_test_flag & (1<<type)) {
			valid |= !!(1<<type);
		}
	}

	sd_param->last_type = type;
	sd_param->last_val[0] = lower;
	sd_param->last_val[1] = upper;

	if (!valid) {
		prd_store_set_sd_usage(dev);
		return count;
	}

	switch (type) {
	/* items of sd */
	case U3_M2_RAWDATA_TEST:
		sd_param->u3_m2[0] = lower;
		sd_param->u3_m2[1] = upper;
		break;
	case U3_BLU_JITTER_TEST:
		sd_param->u3_blu_jitter[0] = lower;
		sd_param->u3_blu_jitter[1] = upper;
		break;
	case U3_JITTER_TEST:
		sd_param->u3_jitter[0] = lower;
		sd_param->u3_jitter[1] = upper;
		break;

	/* items of lpwg_sd */
	case U0_M2_RAWDATA_TEST:
		sd_param->u0_m2[0] = lower;
		sd_param->u0_m2[1] = upper;
		break;
	case U0_M1_RAWDATA_TEST:
		sd_param->u0_m1[0] = lower;
		sd_param->u0_m1[1] = upper;
		break;
	case U0_JITTER_TEST:
		sd_param->u0_jitter[0] = lower;
		sd_param->u0_jitter[1] = upper;
		break;
	}

	if (prd_cmp_tool_str[type]) {
		t_prd_info(prd, "set_sd: %s %d, %s %d\n",
			prd_cmp_tool_str[type][0], lower,
			prd_cmp_tool_str[type][0], upper);
	} else {
		t_prd_info(prd, "set_sd: type %d, lower %d, upper %d\n",
			type, lower, upper);
	}

	return count;
}
#endif /* __SIW_SUPPORT_PRD_SET_SD */


#if defined(__SIW_ATTR_PERMISSION_ALL)
#define __TOUCH_PRD_PERM	(S_IRUGO | S_IWUGO)
#else
#define __TOUCH_PRD_PERM	(S_IRUGO | S_IWUSR | S_IWGRP)
#endif

#define SIW_TOUCH_HAL_PRD_ATTR(_name, _show, _store)	\
		__TOUCH_ATTR(_name, __TOUCH_PRD_PERM, _show, _store)

#define _SIW_TOUCH_HAL_PRD_T(_name)	\
		touch_attr_##_name

static SIW_TOUCH_HAL_PRD_ATTR(sd, prd_show_sd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(delta, prd_show_delta, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(label, prd_show_label, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata_prd, prd_show_rawdata_prd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata_tcm, prd_show_rawdata_tcm, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(rawdata, prd_show_rawdata_ait, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(base, prd_show_basedata, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(debug_buf, prd_show_debug_buf, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(lpwg_sd, prd_show_lpwg_sd, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(file_test, prd_show_file_test, prd_store_file_test);

static SIW_TOUCH_HAL_PRD_ATTR(prd_app_raw, prd_show_app_raw, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_base, prd_show_app_base, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_label, prd_show_app_label, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_delta, prd_show_app_delta, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_debug_buf, prd_show_app_debug_buf, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_end, prd_show_app_end, NULL);
static SIW_TOUCH_HAL_PRD_ATTR(prd_app_info, prd_show_app_info, NULL);

static SIW_TOUCH_HAL_PRD_ATTR(prd_dbg_mask, prd_show_dbg_mask, prd_store_dbg_mask);

#if defined(__SIW_SUPPORT_PRD_SET_SD)
static SIW_TOUCH_HAL_PRD_ATTR(set_sd, prd_show_set_sd, prd_store_set_sd);
#endif

struct siw_hal_prd_attribute {
	struct attribute *attr;
	int flag;
};

#define _SIW_TOUCH_HAL_PRD_LIST_T(_name, _flag)	\
	{ .attr = &_SIW_TOUCH_HAL_PRD_T(_name).attr, .flag = _flag, }

static const struct siw_hal_prd_attribute siw_hal_prd_attribute_list_all[] = {
	_SIW_TOUCH_HAL_PRD_LIST_T(sd, PRD_SYS_EN_SD),
	_SIW_TOUCH_HAL_PRD_LIST_T(delta, PRD_SYS_EN_DELTA),
	_SIW_TOUCH_HAL_PRD_LIST_T(label, PRD_SYS_EN_LABEL),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata_prd, PRD_SYS_EN_RAWDATA_PRD),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata_tcm, PRD_SYS_EN_RAWDATA_TCM),
	_SIW_TOUCH_HAL_PRD_LIST_T(rawdata, PRD_SYS_EN_RAWDATA_AIT),
	_SIW_TOUCH_HAL_PRD_LIST_T(base, PRD_SYS_EN_BASE),
	_SIW_TOUCH_HAL_PRD_LIST_T(debug_buf, PRD_SYS_EN_DEBUG_BUF),
	_SIW_TOUCH_HAL_PRD_LIST_T(lpwg_sd, PRD_SYS_EN_LPWG_SD),
	_SIW_TOUCH_HAL_PRD_LIST_T(file_test, PRD_SYS_EN_FILE_TEST),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_raw, PRD_SYS_EN_APP_RAW),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_base, PRD_SYS_EN_APP_BASE),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_label, PRD_SYS_EN_APP_LABEL),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_delta, PRD_SYS_EN_APP_DELTA),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_debug_buf, PRD_SYS_EN_APP_DEBUG_BUF),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_end, PRD_SYS_EN_APP_END),
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_app_info, PRD_SYS_EN_APP_INFO),
	/* */
	_SIW_TOUCH_HAL_PRD_LIST_T(prd_dbg_mask, -1),
#if defined(__SIW_SUPPORT_PRD_SET_SD)
	_SIW_TOUCH_HAL_PRD_LIST_T(set_sd, PRD_SYS_EN_SD),
#endif
	/* */
	{ .attr = NULL, .flag = 0, },	/* end mask */
};

enum {
	PRD_SYS_ATTR_SIZE = ARRAY_SIZE(siw_hal_prd_attribute_list_all),
};

static struct attribute *siw_hal_prd_attribute_list[PRD_SYS_ATTR_SIZE + 1];

static const struct attribute_group __used siw_hal_prd_attribute_group = {
	.attrs = siw_hal_prd_attribute_list,
};

static int siw_hal_prd_create_group(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct kobject *kobj = &ts->kobj;
	struct siw_hal_prd_attribute *prd_attr;
	struct attribute **attr_actual = siw_hal_prd_attribute_list;
	int sysfs_flag = prd->sysfs_flag;
	int attr_size = PRD_SYS_ATTR_SIZE;
	int i = 0;
	int j = 0;
	int added = 0;
	int ret = 0;

	prd_attr = (struct siw_hal_prd_attribute *)siw_hal_prd_attribute_list_all;

	memset(siw_hal_prd_attribute_list, 0, sizeof(siw_hal_prd_attribute_list));

	for(i = 0; i < attr_size; i++){
		added = 0;
		if ((prd_attr->flag == -1) ||
			(prd_attr->flag & sysfs_flag)){
			attr_actual[j] = prd_attr->attr;
			j++;
			added = 1;
		}

		if (prd_attr->attr == NULL) {
			break;
		}

		t_dev_dbg_base(dev, "prd sysfs %02d(%20s) %s\n",
			i, prd_attr->attr->name,
			(added) ? "added" : "not supported");

		prd_attr++;
	}

	ret = sysfs_create_group(kobj, &siw_hal_prd_attribute_group);
	if (ret >= 0) {
		prd->sysfs_done = 1;
	}

	return ret;
}

static void siw_hal_prd_remove_group(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct kobject *kobj = &ts->kobj;

	if (prd->sysfs_done) {
		prd->sysfs_done = 0;
		sysfs_remove_group(kobj, &siw_hal_prd_attribute_group);
	}
}

static void siw_hal_prd_free_buffer(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	t_prd_dbg_base(prd, "[param free buffer]\n");

	if (prd->buf_src != NULL) {
		t_prd_info(prd, "buffer released: %p(%d)\n",
			prd->buf_src, prd->buf_size);
		kfree(prd->buf_src);

		prd->buf_src = NULL;
		prd->buf_size = 0;

		prd->m2_buf_even_rawdata = NULL;
		prd->m2_buf_odd_rawdata = NULL;

		prd->m1_buf_even_rawdata = NULL;
		prd->m1_buf_odd_rawdata = NULL;
		prd->m1_buf_tmp = NULL;

		prd->open_buf_result_rawdata = NULL;
		prd->short_buf_result_rawdata = NULL;
		prd->open_buf_result_data = NULL;
		prd->short_buf_result_data = NULL;

		prd->buf_delta = NULL;
		prd->buf_debug = NULL;

		prd->buf_label_tmp = NULL;
		prd->buf_label = NULL;
	}
}

static int siw_hal_prd_alloc_buffer(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u8 *buf;
	size_t buf_addr;
	int total_size;

	t_prd_dbg_base(prd, "[param alloc buffer]\n");

	total_size = (ctrl->m2_row_col_buf_size<<2);
	total_size += (ctrl->m1_row_col_size<<2);
	total_size += ctrl->m1_row_col_size;
	total_size += ctrl->delta_size;
	total_size += ctrl->debug_buf_size;
	total_size += ctrl->label_tmp_size;
	total_size += ctrl->m2_row_col_size;

	buf = kzalloc(total_size + 128, GFP_KERNEL);
	if (buf == NULL) {
		t_prd_err(prd, "falied to allocate buffer(%d)\n", total_size);
		return -ENOMEM;
	}

	t_prd_info(prd, "buffer allocted: %p(%d)\n", buf, total_size);

	prd->buf_src = buf;
	prd->buf_size = total_size;

#if 1
	buf_addr = (size_t)buf;
	if (buf_addr & 0x03) {
		buf_addr += 0x03;
		buf_addr &= ~0x03;
		buf = (u8 *)buf_addr;
		t_prd_info(prd, "buffer align tuned: %p\n", buf);
	}
#endif

	prd->m2_buf_even_rawdata = (int16_t *)buf;
	buf += ctrl->m2_row_col_buf_size;
	prd->m2_buf_odd_rawdata = (int16_t *)buf;
	buf += ctrl->m2_row_col_buf_size;

	prd->m1_buf_even_rawdata = (int16_t *)buf;
	buf += ctrl->m1_row_col_size;
	prd->m1_buf_odd_rawdata = (int16_t *)buf;
	buf += ctrl->m1_row_col_size;
	prd->m1_buf_tmp = (int16_t *)buf;
	buf += ctrl->m1_row_col_size;

	prd->open_buf_result_rawdata = (int16_t *)buf;
	buf += ctrl->m2_row_col_buf_size;
	prd->short_buf_result_rawdata = (int16_t *)buf;
	buf += ctrl->m2_row_col_buf_size;
	prd->open_buf_result_data = (int16_t *)buf;
	buf += ctrl->m1_row_col_size;
	prd->short_buf_result_data = (int16_t *)buf;
	buf += ctrl->m1_row_col_size;

	prd->buf_delta = (int16_t *)buf;
	buf += ctrl->delta_size;

	prd->buf_debug = (int16_t *)buf;
	buf += ctrl->debug_buf_size;

	prd->buf_label_tmp = (u8 *)buf;
	buf += ctrl->label_tmp_size;

	prd->buf_label = (u8 *)buf;
//	buf += ctrl->m2_row_col_size;

	return 0;
}

static void siw_hal_prd_parse_ctrl(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_ctrl *ctrl = &prd->ctrl;
	u32 row_size = param->row;
	u32 col_size = param->col;

	t_prd_dbg_base(prd, "[param parse ctrl]\n");

	ctrl->m2_row_col_size = (row_size * col_size);
	ctrl->m2_row_col_buf_size = (row_size * (col_size + param->col_add));
	ctrl->m1_row_col_size = (row_size * param->m1_col);
	ctrl->m2_frame_size = (ctrl->m2_row_col_buf_size<<PRD_RAWDATA_SZ_POW);
	ctrl->m1_frame_size = (ctrl->m1_row_col_size<<PRD_RAWDATA_SZ_POW);
	ctrl->delta_size = ((row_size + 2) * (col_size + 2));
	ctrl->label_tmp_size = ((row_size + 2) * (col_size + 2));
	ctrl->debug_buf_size = PRD_DEBUG_BUF_SIZE;

	t_prd_dbg_base(prd, "ctrl: m2_row_col %d, m2_row_col_buf %d, m1_row_col %d\n",
		ctrl->m2_row_col_size, ctrl->m2_row_col_buf_size, ctrl->m1_row_col_size);
	t_prd_dbg_base(prd, "ctrl: m2_frame %d, m1_frame %d\n",
		ctrl->m2_frame_size, ctrl->m1_frame_size);
	t_prd_dbg_base(prd, "ctrl: delta %d, label_tmp %d\n",
		ctrl->delta_size, ctrl->label_tmp_size);
}

static void siw_hal_prd_parse_tune(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_tune *tune = &prd->tune;
	u32 ch = param->ch;

	t_prd_dbg_base(prd, "[param parse tune]\n");

	tune->ch = ch;
	tune->code_size = ((ch<<3)+4);

	tune->code_l_goft_offset = 0;
	tune->code_l_m1_oft_offset = (1<<1);
	tune->code_l_g1_oft_offset = (tune->code_l_m1_oft_offset + ch);
	tune->code_l_g2_oft_offset = (tune->code_l_g1_oft_offset + ch);
	tune->code_l_g3_oft_offset = (tune->code_l_g2_oft_offset + ch);

	tune->code_r_goft_offset = (tune->code_l_g3_oft_offset + ch);
	tune->code_r_m1_oft_offset = (tune->code_r_goft_offset + 2);
	tune->code_r_g1_oft_offset = (tune->code_r_m1_oft_offset + ch);
	tune->code_r_g2_oft_offset = (tune->code_r_g1_oft_offset + ch);
	tune->code_r_g3_oft_offset = (tune->code_r_g2_oft_offset + ch);

	t_prd_dbg_base(prd, "tune: ch %d, code %d\n",
		ch, tune->code_size);
	t_prd_dbg_base(prd, "tune: l_goft %Xh, l_m1_oft %Xh\n",
		tune->code_l_goft_offset,
		tune->code_l_m1_oft_offset);
	t_prd_dbg_base(prd, "tune: l_g1_oft %Xh, l_g2_oft %Xh, l_g3_oft %Xh\n",
		tune->code_l_g1_oft_offset,
		tune->code_l_g2_oft_offset,
		tune->code_l_g3_oft_offset);
	t_prd_dbg_base(prd, "tune: r_goft %Xh, r_m1_oft %Xh\n",
		tune->code_r_goft_offset,
		tune->code_r_m1_oft_offset);
	t_prd_dbg_base(prd, "tune: r_g1_oft %Xh, r_g2_oft %Xh, r_g3_oft %Xh\n",
		tune->code_r_g1_oft_offset,
		tune->code_r_g2_oft_offset,
		tune->code_r_g3_oft_offset);
}

static void siw_hal_prd_set_offset(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	u32 idx, addr, old_addr, flag;
	int i;

	t_prd_dbg_base(prd, "[param parse set offset]\n");

	flag = 0;
	for (i = 0; i < IMG_OFFSET_IDX_MAX ; i++) {
		if (!param->addr[i]) {
			continue;
		}

		idx = PRD_OFFSET_QUIRK_GET_IDX(param->addr[i]);
		addr = PRD_OFFSET_QUIRK_GET_OFFSET(param->addr[i]);

		if (flag & (1<<idx)) {
			t_prd_warn(prd, "dupliacted: %Xh (%d)\n", addr, idx);
			continue;
		}
		flag |= (1<<idx);

		switch (idx) {
		case IMG_OFFSET_IDX_RAW:
			old_addr = prd->img_offset.raw;
			prd->img_offset.raw = addr;
			break;
		case IMG_OFFSET_IDX_BASELINE_EVEN:
			old_addr = prd->img_offset.baseline_even;
			prd->img_offset.baseline_even = addr;
			break;
		case IMG_OFFSET_IDX_BASELINE_ODD:
			old_addr = prd->img_offset.baseline_odd;
			prd->img_offset.baseline_odd = addr;
			break;
		case IMG_OFFSET_IDX_DELTA:
			old_addr = prd->img_offset.delta;
			prd->img_offset.delta = addr;
			break;
		case IMG_OFFSET_IDX_LABEL:
			old_addr = prd->img_offset.label;
			prd->img_offset.label = addr;
			break;
		case IMG_OFFSET_IDX_F_DELTA:
			old_addr = prd->img_offset.f_delta;
			prd->img_offset.f_delta = addr;
			break;
		case IMG_OFFSET_IDX_DEBUG:
			old_addr = prd->img_offset.debug;
			prd->img_offset.debug = addr;
			break;
		default:
			old_addr = 0;
			addr = 0;
			t_prd_info(prd, "unknown idx: %Xh (%d)\n", addr, idx);
			break;
		}

		if (addr) {
			t_prd_info(prd, "debug offset: %Xh(%d)\n", addr, idx);
		}
	}
}

static void siw_hal_prd_set_cmd(struct device *dev, int cmd_type)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_img_cmd *img_cmd = &prd->img_cmd;
	char *type_name[] = {
		[PRD_CMD_TYPE_1] = "PRD_CMD_TYPE_1",
		[PRD_CMD_TYPE_2] = "PRD_CMD_TYPE_2",
	};

	t_prd_dbg_base(prd, "[param parse set cmd]\n");

	t_prd_info(prd, "cmd type: %s\n", type_name[cmd_type]);

	if (cmd_type == PRD_CMD_TYPE_2) {
		img_cmd->raw = IT_IMAGE_RAW;
		img_cmd->baseline_even = IT_IMAGE_BASELINE;
		img_cmd->baseline_odd = IT_IMAGE_BASELINE + 1;
		img_cmd->delta = IT_IMAGE_DELTA + 1;
		img_cmd->label = IT_IMAGE_LABEL + 1;
		img_cmd->f_delta = IT_IMAGE_FILTERED_DELTA + 1;
		img_cmd->debug = IT_IMAGE_DEBUG + 1;
		return;
	}

	img_cmd->raw = IT_IMAGE_RAW;
	img_cmd->baseline_even = IT_IMAGE_BASELINE;
	img_cmd->baseline_odd = IT_IMAGE_BASELINE;
	img_cmd->delta = IT_IMAGE_DELTA;
	img_cmd->label = IT_IMAGE_LABEL;
	img_cmd->f_delta = IT_IMAGE_FILTERED_DELTA;
	img_cmd->debug = IT_IMAGE_DEBUG;
}

static void siw_hal_prd_set_sd_cmd(struct siw_hal_prd_data *prd)
{
	struct device *dev = prd->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_sd_cmd *sd_cmd = &prd->sd_cmd;

	sd_cmd->cmd_open_node = OPEN_NODE_TEST_POST_CMD;
	sd_cmd->cmd_short_node = SHORT_NODE_TEST_POST_CMD;
	sd_cmd->cmd_m2_rawdata = M2_RAWDATA_TEST_POST_CMD;
	sd_cmd->cmd_m1_rawdata = M1_RAWDATA_TEST_POST_CMD;
	sd_cmd->cmd_jitter = JITTER_TEST_POST_CMD;
	sd_cmd->cmd_u0_jitter = JITTER_TEST_POST_CMD;

	switch (touch_chip_type(ts)) {
	case CHIP_SW49407:
		sd_cmd->cmd_open_node = 1;
		sd_cmd->cmd_short_node = 2;
		sd_cmd->cmd_m2_rawdata = 5;
		sd_cmd->cmd_m1_rawdata = 3;
		sd_cmd->cmd_jitter = 6;
		sd_cmd->cmd_u0_jitter = 4;
		break;
	case CHIP_LG4946:
		sd_cmd->cmd_jitter = 10;
		sd_cmd->cmd_u0_jitter = 10;
		break;
	}

	t_prd_info(prd,
		"cmd_open_node  : %d\n",
		sd_cmd->cmd_open_node);
	t_prd_info(prd,
		"cmd_short_node : %d\n",
		sd_cmd->cmd_short_node);
	t_prd_info(prd,
		"cmd_m2_rawdata : %d\n",
		sd_cmd->cmd_m2_rawdata);
	t_prd_info(prd,
		"cmd_m1_rawdata : %d\n",
		sd_cmd->cmd_m1_rawdata);
	t_prd_info(prd,
		"cmd_jitter 	 : %d\n",
		sd_cmd->cmd_jitter);
	t_prd_info(prd,
		"cmd_jitter(u0)  : %d\n",
		sd_cmd->cmd_u0_jitter);
}

static void siw_hal_prd_parse_work(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	t_prd_dbg_base(prd, "[param parse work]\n");

	siw_hal_prd_set_offset(dev, param);

	siw_hal_prd_set_cmd(dev, param->cmd_type);

	siw_hal_prd_set_sd_cmd(prd);

	prd->sysfs_flag = PRD_SYS_ATTR_EN_FLAG & ~param->sysfs_off_flag;		//Disable quirk bits

	t_prd_info(prd, "sysfs flag: %Xh (%Xh)\n", prd->sysfs_flag, param->sysfs_off_flag);
}

static void siw_hal_prd_show_param(struct device *dev, struct siw_hal_prd_param *param)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	int i;

	t_prd_dbg_base(prd, "[current param]\n");

	t_prd_dbg_base(prd, "param: chip_type %Xh, cmd_type %d\n",
			param->chip_type,
			param->cmd_type);
	if (param->name != NULL) {
		char *name;
		int name_idx = 0;
		while (1) {
			name = (char *)param->name[name_idx];
			if (name == NULL) {
				break;
			}
			t_prd_dbg_base(prd,
				"param: name[%d] %s\n",
				name_idx, name);
			name_idx++;
		}
	}

	for (i = 0; i < IMG_OFFSET_IDX_MAX; i++) {
		if (!param->addr[i])
			continue;

		t_prd_dbg_base(prd, "param: addr[%d] %Xh\n",
			i, param->addr[i]);
	}

	t_prd_info(prd, "param: row %d, col %d\n",
		param->row, param->col);
	t_prd_dbg_base(prd, "param: col_add %d, ch %d\n",
		param->col_add, param->ch);
	t_prd_dbg_base(prd, "param: m1_col %d, m1_cnt %d, m2_cnt %d\n",
		param->m1_col, param->m1_cnt, param->m2_cnt);

	t_prd_dbg_base(prd, "param: sysfs_off_flag %Xh\n",
		param->sysfs_off_flag);
}

static int siw_hal_prd_parse_param(struct device *dev, struct siw_hal_prd_param *param)
{
	siw_hal_prd_show_param(dev, param);

	siw_hal_prd_parse_ctrl(dev, param);

	siw_hal_prd_parse_tune(dev, param);

	siw_hal_prd_parse_work(dev, param);

	return siw_hal_prd_alloc_buffer(dev);
}

static int siw_hal_prd_init_param(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;
	struct siw_hal_prd_param *param = (struct siw_hal_prd_param *)prd_params;
	int len, found;
	int idx;
	int ret;

	idx = 0;
	while (1) {
		if (!param->chip_type) {
			break;
		}

		if (param->chip_type == touch_chip_type(ts)) {
			found = !!(param->name == NULL);
			if (!found) {
				char *name;
				int name_idx = 0;
				while (1) {
					name = (char *)param->name[name_idx];
					if (name == NULL) {
						break;
					}
					len = strlen(name);
					found = !strncmp(fw->product_id, name, len);
					if (found) {
						break;
					}
					name_idx++;
				}
			}

			if (found) {
				t_prd_dbg_base(prd, "%s[%s] param %d selected\n",
					touch_chip_name(ts), fw->product_id, idx);

				ret = siw_hal_prd_parse_param(dev, param);
				if (ret < 0) {
					t_prd_err(prd, "%s[%s] param parsing failed, %d\n",
						touch_chip_name(ts), fw->product_id, ret);
				} else {
					memcpy(&prd->param, param, sizeof(*param));
				}
				return ret;
			}
		}

		param++;
		idx++;
	}

	t_prd_err(prd, "%s[%s] param not found\n",
		touch_chip_name(ts), fw->product_id);

	return -EFAULT;
}

static void siw_hal_prd_free_param(struct device *dev)
{
	siw_hal_prd_free_buffer(dev);
}

static struct siw_hal_prd_data *siw_hal_prd_alloc(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd;

	prd = touch_kzalloc(dev, sizeof(*prd), GFP_KERNEL);
	if (!prd) {
		t_dev_err(dev,
				"failed to allocate memory for prd\n");
		goto out;
	}

	snprintf(prd->name, sizeof(prd->name)-1, "%s-prd", dev_name(dev));

	t_dev_dbg_base(dev, "create prd[%s] (0x%X)\n",
				prd->name, (int)sizeof(*prd));

	prd->dev = ts->dev;

	ts->prd = prd;

#if defined(__SIW_SUPPORT_PRD_SET_SD)
	prd->sd_param.last_type = UX_INVALID;
#endif

	return prd;

out:
	return NULL;
}

static void siw_hal_prd_free(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_prd_data *prd = (struct siw_hal_prd_data *)ts->prd;

	if (prd) {
		t_dev_dbg_base(dev, "free prd[%s]\n", prd->name);

		ts->prd = NULL;
		touch_kfree(dev, prd);
	}
}

static int siw_hal_prd_create_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;
	struct siw_hal_prd_data *prd;
	int ret = 0;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_err(dev, "Invalid kobject\n");
		return -EINVAL;
	}

	prd = siw_hal_prd_alloc(dev);
	if (!prd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = siw_hal_prd_init_param(dev);
	if (ret < 0) {
		/* Just skip sysfs generation */
		goto out_skip;
	}

	ret = siw_hal_prd_create_group(dev);
	if (ret < 0) {
		t_dev_err(dev, "%s prd sysfs register failed, %d\n",
				touch_chip_name(ts), ret);
		goto out_sysfs;
	}

	t_dev_dbg_base(dev, "%s prd sysfs registered\n",
			touch_chip_name(ts));

out_skip:
	return 0;

out_sysfs:
	siw_hal_prd_free_param(dev);

	siw_hal_prd_free(dev);

out:
	return ret;
}

static void siw_hal_prd_remove_sysfs(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct device *idev = &ts->input->dev;
	struct kobject *kobj = &ts->kobj;

	if (kobj->parent != idev->kobj.parent) {
		t_dev_err(dev, "Invalid kobject\n");
		return;
	}

	if (ts->prd == NULL) {
		return;
	}

	siw_hal_prd_remove_group(dev);

	siw_hal_prd_free_param(dev);

	siw_hal_prd_free(dev);

	t_dev_dbg_base(dev, "%s prd sysfs unregistered\n",
			touch_chip_name(ts));
}

int siw_hal_prd_sysfs(struct device *dev, int on_off)
{
	if (on_off == DRIVER_INIT) {
		return siw_hal_prd_create_sysfs(dev);
	}

	siw_hal_prd_remove_sysfs(dev);
	return 0;
}

int siw_hal_set_prd_file(struct device *dev, const char *path, int idx)
{
	char *prd_file_group[] = {
		[0]	= __prd_in_file,
		[1]	= __prd_in_file_m,
		[2] = __prd_out_file,
		[3] = __prd_out_file_mo_aat,
		[4] = __prd_out_file_mo_mfo,
		[5] = __prd_out_file_mo_mfl,
		[6] = __prd_out_file_mo_mcv,
	};

	if (!path) {
		t_dev_err(dev, "NULL prd path (%d)\n", idx);
		return -EINVAL;
	}

	if (idx >= ARRAY_SIZE(prd_file_group)) {
		t_dev_err(dev, "wrong prd file index, %d\n", idx);
		return -EINVAL;
	}

	t_dev_info(dev, "prd file setup: %s -> %s\n",
			prd_file_group[idx], path);
	memcpy(prd_file_group[idx], path, strlen(path));

	return 0;
}

__siw_setup_str("siw_prd_if=", prd_setup_if, __prd_in_file);
__siw_setup_str("siw_prd_if_m=", prd_setup_if_m, __prd_in_file_m);

__siw_setup_str("siw_prd_of_=", prd_setup_of, __prd_out_file);
__siw_setup_str("siw_prd_of_mo_aat=", prd_setup_of_aat, __prd_out_file_mo_aat);
__siw_setup_str("siw_prd_of_mo_mfo=", prd_setup_of_mfo, __prd_out_file_mo_mfo);
__siw_setup_str("siw_prd_of_mo_mfl=", prd_setup_of_mfl, __prd_out_file_mo_mfl);
__siw_setup_str("siw_prd_of_mo_mcv=", prd_setup_of_mcv, __prd_out_file_mo_mcv);

#endif	/* __SIW_SUPPORT_PRD */


