/*
 * dbmdx-log.h  --  DBMDX CDEV interface common functions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_LOG_H
#define _DBMDX_LOG_H

#include "dbmdx-cdev-core.h"

/* DBMDX_FIFO_TYPE_LP_XXX are connected to my_work_t my_work */
enum dbmdx_fifo_type {
	/* audio */
	DBMDX_FIFO_TYPE_AUDIO = 0,
	/* core lp logger */
	DBMDX_FIFO_TYPE_LP_LOG,
	/* core hf0 logger */
	DBMDX_FIFO_TYPE_HF0_LOG,
	/* core hf1 logger */
	DBMDX_FIFO_TYPE_HF1_LOG,

	DBMDX_FIFO_TYPE_MAX
};

#define DBMDX_MAX_NUM_KFIFO DBMDX_FIFO_TYPE_MAX
#define MAX_NUM_OF_WORKS DBMDX_FIFO_TYPE_MAX

enum mb_buffering_mode {
	BUFFERING_DISABLED = 0,
	BUFFERING_COUNTING = 1,
	BUFFERING_UNTIL_MB_EMPTY = 2,
};

struct mb_buf_work_data {
	struct			work_struct mb_buf_work;
	struct			dbmdx_private *p;
	enum mb_buffering_mode	buffering_mode;
	u32			sample_rate;
	u32			sample_width;
	u32			num_channels;
	u32			mailbox_id;
	struct kfifo		*samples_fifo;
	enum dbmdx_fifo_type	fifo_type;
};


struct dbmdx_fifo_db {
	struct kfifo		samples_kfifo;
	void			*samples_kfifo_buf;
	unsigned int		samples_kfifo_buf_size;
};

struct dbmdx_cdev_pdata {
	u32			core_log_watermark;
	u32			fifo_en_mask;
};

struct dbmdx_cdev_private {
	struct dbmdx_cdev_pdata	*pdata;
	struct device		*dev;
	struct dbmdx_cdev	cdev[DBMDX_MAX_NUM_KFIFO];
	struct dbmdx_fifo_db	fifo_db[DBMDX_MAX_NUM_KFIFO];
	struct mb_buf_work_data mb_buffering_work[MAX_NUM_OF_WORKS];
	wait_queue_head_t       wait[MAX_NUM_OF_WORKS];
	u32			host_if_config_value;
};

void dbmdx_start_debug_core_mb_buffering(struct dbmdx_private *p,
							u16 mailbox_id);
#endif /* _DBMDX_LOG_H */
