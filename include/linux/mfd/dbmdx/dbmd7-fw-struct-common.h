/**
 * dbmd7-fw-struct-common.h - Common FW structs definition
 *
 * Copyright (C) 2016 DSP Group
 * All rights reserved.
 *
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef _DBMD7_FW_STRUCT_COMMON_H
#define _DBMD7_FW_STRUCT_COMMON_H


/************************************************************************/


/************************************************************************/

union tdm_cfg1 {
	int val;
	struct	{
		/* 0-7	bit for each TDM */
		u32 tdm_sel:8;

		/* 8-11	audio sampling frequency 0:
		 * 16kHz, 1:44.1kHz, 2:48kHz, 3:96kHz, 4:8kHz, 5:32kHz
		 */
		u32 smp_freq:4;

		/* 12-15	0:1ch, other 1-15 ch. */
		u32 nch:4;

		/* 16-18	The bits number which should be notified
		 * "frame ready" to the task
		 */
		u32 notify_bit:3;

		/* 19		0-use ri[0], 1-use ri[1] */
		u32 mb_read_client:1;

		/* 20-21	0:8 bits, 1:16 bits, 2: 24 bits, 3: 32btis */
		u32 smp_width:2;

		/* 22-24	R/W function */
		u32 tdm_rw_policy:3;

		/* 25-27	DM clock / decimation factor */
		u32 mb_cfg:3;

		/* 28		number of FIFO stages to R/W each interrupt */
		u32 smps_per_int:1;

		/* 29		switch pins to GPIO */
		u32 gpio_cfg:1;

		/* 30-31	select core to handle TDM interrupts */
		u32 core4int:2;
	} part;
};

/************************************************************************/
union tdm_activation_reg {
	u32 val;
	struct {
		/* 0: disable TDM Rx blocks 0 - 7
		 * 1: enable TDM Rx blocks  0 - 7
		 */
		u32 rx_tdm_en_bits_field:8;

		/* 0: disable TDM Tx blocks 0 - 7
		 * 1: enable TDM Tx blocks  0 - 7
		 */
		u32 tx_tdm_en_bits_field:8;

		u32 reset_mic_mb:4;

		u32 tdm_int_handling_core:2;
	} part;
};

/************************************************************************/

struct tdm_config_setup {
	/* word 0 */
	union tdm_cfg1 tdm_cfg_bits;

	/* word 1 - 4 */
	/* 4 x 32 bits */
	u32 tdm_hw_config[4];

	/* word 5 */
	/* 32 bits */
	u32 frame_len;

	/* word 6 */
	/* 32 bits */
	u32 task_handler;
};


/************************************************************************/
union audio_attrib {
	u32 val;
	struct audio_prop {
		/* 0-3	audio sampling frequency 0:
		 * 16kHz, 1:44.1kHz, 2:48kHz, 3:96kHz, 4:8kHz, 5:32kHz
		 */
		u32 smp_freq:4;

		/* 4-7		0:1ch, other 1-15 ch. */
		u32 nch:4;

		/* 8-10		The bits number which should be notified
		 * "frame ready" to the task
		 */
		u32 notify_bit:3;

		/* 11		0-use ri[0], 1-use ri[1] */
		u32 mb_read_client:1;

		/* 12-13	0:8 bits, 1:16 bits, 2: 24 bits, 3: 32bits */
		u32 smp_width:2;

		/* 14		auxiliary bit for various purposes */
		u32 aux1:1;

		/* 15		mb full for client 0 */
		u32 mb_overflow0:1;

		/* 16		mb full for client 1 */
		u32 mb_overflow1:1;

		/*  17-22	(1+smp_width)*nch */
		u32 smp_q:6;

		/* 23		mark that this is a TDM Tx buffer */
		u32 tdm_tx:1;

		/* 24		don't increment read counter & index */
		u32 read_disable:1;

		/* 25 */
		u32 write_disable:1;
	} part;
};

/************************************************************************/

struct conn_point {
	union audio_attrib attrib;
	int smp_count;
	union {
		short *p16;
		long *p32;
	} ptr[8];
};

/************************************************************************/

struct fw_mb {
	/* signature for detecting over-run */
	short signature;
	/* keep own ID for recording */
	short mb_id;

	/* wi also used as semaphore */
	int ri[2];
	int wi;

	int len[2];
	int wcnt;
	int rcnt[2];
	int wcnt_lim;
	int rcnt_lim;
	union audio_attrib attrib;

	int thd;
	int read_task_to_call;
	int write_task_to_call;
	int frame_alignment;
	int backlog;

	int part1_nreq_r;
	int part1_nreq_w;

	int notify_read_cnt;
	int notify_write_cnt;

	char recording_status;
	char debug_status;

	/* to remove warning: ISO C forbids zero-size array 'data' */
	char *data;
};

/************************************************************************/

struct general_task_descriptor {
	int task_addr, stack_size;
	void *param;
	int priority;
	char task_name[10];
};

/************************************************************************/


#define TASK_DESCRIPTOR(struct_name, task_func_name, task_name_string, stk_size, tparam, pri) \
const struct	{						\
	void (*task_addr)(void *);				\
	int stack_size, param, priority;			\
	char task_name[sizeof(task_name_string)];		\
} struct_name = {task_func_name, stk_size, tparam, pri, task_name_string};

/************************************************************************/


union queue_cmd_and_data {
	u32 val;
	struct {
		u32 data:24;
		u32 cmd:8;
	} part;
};

/************************************************************************/

union mb_ctrl_struct {
	u32 val;
	struct ctrl_prop_t {
		u32 notify_bit:4;
		u32 mb_read_client:1;
		u32 aux1:1;
		u32 mb_overflow0:1;
		u32 mb_overflow1:1;
		u32 dis_overflow:1;
		u32 read_disable:1;
		u32 write_disable:1;
		u32 use_c_pack_unpack:1;
	} part;
};

/************************************************************************/

struct mailbox_data_attrib {
	union audio_attrib	audio;
	union mb_ctrl_struct    control;
};



#endif /* _DBMD7_FW_STRUCT_COMMON_H */

