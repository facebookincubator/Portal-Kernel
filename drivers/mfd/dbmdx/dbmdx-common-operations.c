/*
 * dbmdx-common-operations.c - DSPG DBMDX Common FW operations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <linux/wait.h>
#include <sound/initval.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/fb_event.h>
#include <uapi/linux/fb_event.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-utils.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmdx-common-operations.h>
#include <linux/mfd/dbmdx/dbmdx-cdev.h>

const u32 fw_attrib_smp_rate_to_hz[] = {
		16000, 44100, 48000, 96000, 8000, 32000};

#define PRESET_PARAM_CLK_SEL_COMMAND		0x4011420
#define PRESET_PARAM_PLL_FB_DIV_VAL		0x40e0810
#define PRESET_PARAM_PLL_PRE_DIV_VAL		0x4121100

/* Driver added structs */

union mic_config {
	uint32_t val;
	struct {
		u32 ddf:8;
		u32 dm_clock:8;
		u32 sampling_rate:4;
		u32 mic_num_hw_vad:3;
		u32 enable_hw_vad:1;
		u32 ddf_int_core:2;
		u32 configure_ddf:1;
		u32 reconfigure_gpio:1;
		u32 reserved:4;
	} conf;
};

union operation_mode {
	u32 val;
	struct {
		u32 param1:4;
		u32 param2:4;
		u32 param3:8;
		u32 param4:8;
		u32 sub_mode:8;
	} conf;
};

struct sensory_detection_msg {
	u16 model;
	u16 word_id;
	u32 end_phrase_to_detect_duration;
	u32 phrase_duration;
	u32 time_stamp;
};

union create_task_word {
	u32 val;
	struct	{
		u32 number:24;
		u32 vensor_id:4;
		u32 not_valid:2;
		u32 core:2;
		} part;
};

union mb_create_by_attr_word1 {
	u32 val;
	struct {
		u32 mailbox_id:16;
		u32 core:2;
		u32 not_valid1:10;
		u32 mem_loc:2;
		u32 not_valid2:2;
	} part;
};

union mb_create_by_size_word1 {
	u32 val;
	struct {
		u32 mb_id:16;
		u32 core:2;
		u32 not_used:14;
	} part;
};

union mb_create_by_size_word2 {
	u32 val;
	struct {
		u32 size:24;
		u32 not_used:4;
		u32 mem_location:2;
		u32 not_used1:2;
	} part;
};


int dbmdx_get_fw_attrib_dm_clock_freq(u32 dm_clock_freq_val,
					u8 *fw_attrib_dm_clock)
{
	int ret = 0;

	switch (dm_clock_freq_val) {
	case 512:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_512kHz;
		break;
	case 768:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_768kHz;
		break;
	case 1024:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_1024kHz;
		break;
	case 1536:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_1536kHz;
		break;
	case 2048:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_2048kHz;
		break;
	case 2304:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_2304kHz;
		break;
	case 3072:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_3072kHz;
		break;
	default:
		*fw_attrib_dm_clock = ATTRIB_DM_CLOCK_1024kHz;
		ret = -EINVAL;
		break;
	}

	return ret;
}


int dbmdx_get_fw_attrib_sample_rate(u32 sample_rate_hz_val,
					u8 *fw_attrib_sample_rate)
{
	int ret = 0;

	switch (sample_rate_hz_val) {
	case 8000:
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_8K;
		break;
	case 16000:
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_16K;
		break;
	case 32000:
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_32K;
		break;
	case 44100:
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_44K1;
		break;
	case 48000:
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_48K;
		break;
	default:
		ret = -EINVAL;
		*fw_attrib_sample_rate = ATTRIB_SMP_RATE_48K;
		break;
	}

	return ret;
}

int dbmdx_get_fw_attrib_sample_width(u32 sample_width,
					u8 *fw_attrib_sample_width)
{
	int ret = 0;

	switch (sample_width) {
	case 8:
		*fw_attrib_sample_width = ATTRIB_SMP_WIDTH_8_BITS;
		break;
	case 16:
		*fw_attrib_sample_width = ATTRIB_SMP_WIDTH_16_BITS;
		break;
	case 24:
		*fw_attrib_sample_width = ATTRIB_SMP_WIDTH_24_BITS;
		break;
	case 32:
		*fw_attrib_sample_width = ATTRIB_SMP_WIDTH_32_BITS;
		break;
	default:
		ret = -EINVAL;
		*fw_attrib_sample_width = ATTRIB_SMP_WIDTH_8_BITS;
		break;
	}

	return ret;
}

int dbmdx_get_fw_attrib_tdm_tx_mb_index(u32 tdm_tx,
				u8 *fw_attrib_tdm_tx_mb_idx)
{
	int ret = 0;

	switch (tdm_tx) {
	case 0:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_0_TX_ID;
		break;
	case 1:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_1_TX_ID;
		break;
	case 2:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_2_TX_ID;
		break;
	case 3:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_3_TX_ID;
		break;
	case 4:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_4_TX_ID;
		break;
	case 5:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_5_TX_ID;
		break;
	case 6:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_6_TX_ID;
		break;
	case 7:
		*fw_attrib_tdm_tx_mb_idx = MB_INDEX_TDM_7_TX_ID;
		break;

	default:
		ret = -EINVAL;
		*fw_attrib_tdm_tx_mb_idx = REG_FW__TDM_CONFIG_LAST;
		break;
	}

	return ret;
}

int dbmdx_get_fw_attrib_tdm_rx_mb_index(u32 tdm_rx,
				u8 *fw_attrib_tdm_rx_mb_idx)
{
	int ret = 0;

	switch (tdm_rx) {
	case 0:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_0_RX_ID;
		break;
	case 1:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_1_RX_ID;
		break;
	case 2:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_2_RX_ID;
		break;
	case 3:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_3_RX_ID;
		break;
	case 4:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_4_RX_ID;
		break;
	case 5:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_5_RX_ID;
		break;
	case 6:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_6_RX_ID;
		break;
	case 7:
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_7_RX_ID;
		break;

	default:
		ret = -EINVAL;
		*fw_attrib_tdm_rx_mb_idx = MB_INDEX_TDM_0_RX_ID;
		break;
	}

	return ret;
}

int dbmdx_check_if_alive(struct dbmdx_private *p)
{
	u32 fw_health_status = 0;
	int ret = 0;
	union fb_dspg_event event = {
		.event = FB_DSPG_EVENT_FW_ERROR,
		.status = FB_DSPG_STAT_FAILED,
		.param = FB_ERROR_DSPG_NOT_RESPONDING,
	};

	unsigned long stimeout = jiffies +
				msecs_to_jiffies(DBMDX_MSLEEP_IS_ALIVE);

	do {

		/* check if firmware is still alive */
		ret = dbmdx_read_register32(p, MSG_ID_ADDON_FB,
					GET_DSPG_HEALTH_STAT, &fw_health_status);
		if (fw_health_status != 0)
			break;
		if (ret == 0)
			break;
	} while (time_before(jiffies, stimeout));

	if (fw_health_status != 0) {
		dev_err(p->dev, "%s: Firmware is dead error:0x%x\n",
			__func__, fw_health_status);
		event.param = fw_health_status;
		ret = -EFAULT;
	} else if (ret != 0)
		dev_err(p->dev, "%s: Firmware is dead and stopped responding\n",
			__func__);
	else
		return ret;

	dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);

	return ret;
}


int dbmdx_fw_print_host_message(struct dbmdx_private *p, u8 *fw_str)
{
	int ret = 0;
	u32 len = strlen(fw_str);
	u32 ack_val;

	dev_info(p->dev, "%s: len: %d bytes\n", __func__, len);

	ret = dbmdx_send_message_ack(p, MSG_ID_FW, REG_FW__PRINT_HOST_MESSAGE,
					fw_str, len, &ack_val);

	if (ret < 0)
		dev_err(p->dev, "%s: Error sending FW print cmd\n", __func__);

	return ret;

}
static int dbmdx_process_fw_reported_error(struct dbmdx_private *p,
						u32 error_code)
{
	dev_err(p->dev, "%s: FW Reported ERROR(0x%04x)!!!\n",
							__func__, error_code);
	return 0;
}

static int dbmdx_process_fb_addon_message(struct dbmdx_private *p,
						u32 state_val)
{
	switch (state_val) {

	case FB_ADDON_USECASE_MANAGER_READY:
		complete(&p->fw_load_complete);
		break;
	}
	return 0;
}

static int dbmdx_set_new_rtos_task_task_handle(struct dbmdx_private *p,
						u32 task_handle)
{
	if (!atomic_read(&(p->primary_flags.task_create_in_process))) {
		dev_err(p->dev,	"%s: Unexpected report of Task HANDLE\n",
			__func__);
		return -EINVAL;
	}

	if (!p->primary_flags.task_waiting_for_task_handle) {
		dev_err(p->dev,	"%s: Unexpected report of Task HANDLE (2)\n",
			__func__);
		return -EINVAL;
	}

	p->primary_flags.task_waiting_for_task_handle = false;
	p->primary_flags.current_task_handle = task_handle;

	/* Check if queue_id has been already reported */
	if (!p->primary_flags.task_waiting_for_queue_id) {
		if (!atomic_read(&(p->primary_flags.task_create_in_process))) {
			dev_err(p->dev, "%s No active ask creation proc!!!\n",
				__func__);
		} else {
			atomic_set(&p->primary_flags.task_create_in_process, 0);
			dev_dbg(p->dev, "%s Signal that task was created\n",
				__func__);
			wake_up_interruptible(&p->host_event_wq);
		}

	}

	return 0;
}

static int dbmdx_set_new_rtos_task_queue_id(struct dbmdx_private *p,
						u32 queue_id)
{
	if (!atomic_read(&(p->primary_flags.task_create_in_process))) {
		dev_err(p->dev,	"%s: Unexpected report of Queue ID\n",
			__func__);
		return -EINVAL;
	}

	if (!p->primary_flags.task_waiting_for_queue_id) {
		dev_err(p->dev,	"%s: Unexpected report of Queue ID (2)\n",
			__func__);
		return -EINVAL;
	}

	p->primary_flags.task_waiting_for_queue_id = false;
	p->primary_flags.current_queue_id = queue_id;

	/* Check if queue_id has been already reported */
	if (!p->primary_flags.task_waiting_for_task_handle) {
		if (!atomic_read(&(p->primary_flags.task_create_in_process))) {
			dev_err(p->dev, "%s No active ask creation proc!!!\n",
				__func__);
		} else {
			atomic_set(&p->primary_flags.task_create_in_process, 0);
			dev_dbg(p->dev, "%s Signal that task was created\n",
				__func__);
			wake_up_interruptible(&p->host_event_wq);
		}
	}

	return 0;
}

static int dbmdx_process_fw_message_fw_reg(struct dbmdx_private *p,
					struct fw_message *message)
{
	int ret = -EINVAL;

	switch (message->header.id) {
	case MSG_ID_FW:
		switch (message->header.reg_num) {
		case REG_FW__ERROR_REPORT:
			ret = dbmdx_process_fw_reported_error(p,
							message->reg_info[0]);
			if (ret < 0)
				dev_err(p->dev,
				"%s: Error processing FW reported error\n",
					__func__);
			break;
		default:
			dev_err(p->dev,
			"%s: Unsupported pending FW message (reg=0x%x)\n",
					__func__, message->header.reg_num);
			break;

		}
		break;
	case MSG_ID_RTOS:
		switch (message->header.reg_num) {
		case REG_RTOS__CREATE:
			ret = dbmdx_set_new_rtos_task_task_handle(p,
							message->reg_info[0]);
			if (ret < 0)
				dev_err(p->dev,
				"%s: Error setting rtos task TASK_HANDLE\n",
					__func__);
			break;
		case REG_RTOS__REPORT_TASK_QUEUE:
			ret = dbmdx_set_new_rtos_task_queue_id(p,
							message->reg_info[0]);
			if (ret < 0)
				dev_err(p->dev,
				"%s: Error setting rtos task QUEUE_ID\n",
					__func__);
			break;
		default:
			dev_err(p->dev,
			"%s: Unsupported pending RTOS message (reg=0x%x)\n",
					__func__, message->header.reg_num);
			break;

		}
		break;
	case MSG_ID_MAILBOX:
		dev_info(p->dev, "%s: register number %x  mailbox %x\n",
					__func__, message->header.reg_num,
					(message->reg_info[0] & 0x0000ffff));
		switch (message->header.reg_num) {
		case UART_DEBUG_WAIT_COMPLETE:
			dbmdx_start_debug_core_mb_buffering(p,
					(message->reg_info[0] & 0x0000ffff));
			ret = 0;
			break;
		}
		break;
	case MSG_ID_ADDON_FB:
		dev_info(p->dev, "%s: register number %x  mailbox %x\n",
					__func__, message->header.reg_num,
					(message->reg_info[0] & 0x0000ffff));
		ret = dbmdx_process_fb_addon_message(p,
					message->header.reg_num);
		break;
	default:
		dev_err(p->dev,	"%s: Unsupported pending message ID2 (0x%x)\n",
					__func__, message->header.id);
		break;
	}

	return ret;
}

static int dbmdx_process_fw_message_sensory(struct dbmdx_private *p,
					struct fw_message *message)
{
	u32 message_len = message->var_msg_len_bytes;
	struct sensory_detection_msg sensory_msg;

	if (message->header.m_type != MSG_TYPE__SEND_VAR_LAN_MSG) {
		dev_err(p->dev,	"%s: Unsupported Sensory message type (0x%x)\n",
					__func__, message->header.m_type);
		return -EINVAL;
	}
	if (message_len < sizeof(struct sensory_detection_msg)) {
		dev_err(p->dev,
			"%s: Recieved message is to short: %u (min %zd) bytes",
					__func__,
					message_len,
					sizeof(struct sensory_detection_msg));
		return -EINVAL;
	}

	memcpy(&sensory_msg, message->var_msg,
			sizeof(struct sensory_detection_msg));

	dev_info(p->dev, "%s:=== Sensory Voice Wakeup ===\n", __func__);
	dev_info(p->dev, "%s:\t\tmodel: 0x%04X\n",
				__func__, sensory_msg.model);
	dev_info(p->dev, "%s:\t\tword_id: 0x%04X\n",
					__func__, sensory_msg.word_id);
	dev_info(p->dev, "%s:\t\tend_phrase_to_detect_duration: 0x%08X\n",
			__func__, sensory_msg.end_phrase_to_detect_duration);
	dev_info(p->dev, "%s:\t\tphrase_duration: 0x%08X\n",
					__func__, sensory_msg.phrase_duration);
	dev_info(p->dev, "%s:\t\t time_stamp: 0x%08X\n",
					__func__, sensory_msg.time_stamp);

	return 0;
}


static int dbmdx_parse_fw_message(struct dbmdx_private *p,
				struct fw_message *message)
{
	int ret = -1;

	switch (message->header.id) {
	case MSG_ID_FW:
	case MSG_ID_MAILBOX:
	case MSG_ID_RTOS:
	case MSG_ID_RES:
	case MSG_ID_ADDON_FB:
		ret = dbmdx_process_fw_message_fw_reg(p, message);
		if (ret < 0)
			dev_err(p->dev,
				"%s: Error processing fw msg (id)\n",
				__func__);
		break;
	case MSG_ID_ADDON_SENSORY:
		ret = dbmdx_process_fw_message_sensory(p, message);
		if (ret < 0)
			dev_err(p->dev,
				"%s: Error processing sensory fw msg\n",
				__func__);
		break;
	default:
		dev_err(p->dev,	"%s: Unsupported FW  message ID1:0x%x\n",
			__func__, message->header.id);
		ret = -EINVAL;
		break;
	}

	dbmdx_notifier_call(p->dev, DBMDX_EV_MESSAGE, message);

	if (message->header.id == MSG_ID_FW &&
			message->header.reg_num == REG_FW__ERROR_REPORT) {
		union fb_dspg_event event = {
			.event = FB_DSPG_EVENT_FW_ERROR,
			.status = FB_DSPG_STAT_FAILED,
			.param = message->reg_info[0],
		};

		dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);
	}

	return ret;
}


int dbmdx_process_pending_messages(struct dbmdx_private *p)
{
	int ret;
	bool message_is_pending = false;
	struct fw_message pending_message[MAX_NUM_OF_PENDING_FW_MSG];
	int i;

	int num_messages_processed = 0, num_messages_parsed = 0;

	while (1) {
		if (num_messages_processed >= MAX_NUM_OF_PENDING_FW_MSG) {
			dev_warn(p->dev,
				"%s: Reached Max num of pending messages %d\n",
				__func__, num_messages_processed);
			break;
		}

		ret = dbmdx_get_pending_message(p, &message_is_pending,
				&(pending_message[num_messages_processed]));

		if (ret < 0) {
			dev_err(p->dev, "%s: Error getting pending message\n",
				__func__);
			return ret;
		}

		if (!message_is_pending) {
			dev_info(p->dev, "%s: Got %d pending messages\n",
				__func__, num_messages_processed);
			break;
		}

		num_messages_processed++;
	}

	for (i = 0; i < num_messages_processed; i++) {
		ret = dbmdx_parse_fw_message(p, &(pending_message[i]));
		if (ret < 0)
			dev_err(p->dev, "%s: Error parsing pending message\n",
				__func__);
		else
			num_messages_parsed++;

		dbmdx_free_fw_message(p, &(pending_message[i]));
	}

	dev_info(p->dev, "%s: Processed: %d, Parsed: %d messages\n",
		 __func__, num_messages_processed, num_messages_parsed);

	return 0;
}

int dbmdx_tdm_setup(struct dbmdx_private *p, u8 reg_num,
			union tdm_cfg1 *new_tdm_config,
			u32 frame_len,
			u32 task_handler,
			u8 tdm_mode,
			u8 mb_mem_location)
{
	char fw_str[DBMDX_FW_DBG_STRING_LENGTH];
	int ret = 0;
	u32 ack_val;
	/*	TDM  mailbox */
	struct tdm_config_setup tdm_config;

	dev_info(p->dev, "%s: task_handler: 0x%x\n", __func__, task_handler);

	sprintf(fw_str, "TDM reg %x, freq %u, %u ch's, %u bits, frm %u\n",
		reg_num,
		fw_attrib_smp_rate_to_hz[new_tdm_config->part.smp_freq],
		new_tdm_config->part.nch,
		8 * (new_tdm_config->part.smp_width + 1), frame_len);

	dev_info(p->dev, "%s: %s\n", __func__, fw_str);

	dbmdx_fw_print_host_message(p, fw_str);

	sprintf(fw_str, "TDM blk %x, notify %x, mode %x, core %x\n",
	    new_tdm_config->part.tdm_sel, new_tdm_config->part.notify_bit,
	    tdm_mode, new_tdm_config->part.core4int);

	dev_info(p->dev, "%s: %s\n", __func__, fw_str);
	dbmdx_fw_print_host_message(p, fw_str);

	tdm_config.tdm_cfg_bits.val = 0;
	tdm_config.frame_len = 0;

	if (tdm_mode == TDM_MASTER) {
		if (new_tdm_config->part.smp_width ==
		    ATTRIB_SMP_WIDTH_16_BITS) {
			tdm_config.tdm_hw_config[0] = 0x800012;	/* CFG */
			tdm_config.tdm_hw_config[1] = 0x41004;	/* PH12 */
			tdm_config.tdm_hw_config[2] = 0x100f001f; /* FRCFG */
		} else if (new_tdm_config->part.smp_width ==
			   ATTRIB_SMP_WIDTH_32_BITS) {
			tdm_config.tdm_hw_config[0] = 0x800012;	/* CFG */
			tdm_config.tdm_hw_config[1] = 0x73007; /* PH12 */
			tdm_config.tdm_hw_config[2] = 0x101f003f; /* FRCFG */
		}
	} else {
		if (new_tdm_config->part.smp_width ==
		    ATTRIB_SMP_WIDTH_16_BITS) {
			tdm_config.tdm_hw_config[0] = 0x800015;	/* CFG */
			tdm_config.tdm_hw_config[1] = 0x41004; /* PH12 */
			tdm_config.tdm_hw_config[2] = 0x100f001f; /* FRCFG */
		} else if (new_tdm_config->part.smp_width ==
			   ATTRIB_SMP_WIDTH_32_BITS) {
			tdm_config.tdm_hw_config[0] = 0x800015;	/* CFG */
			tdm_config.tdm_hw_config[1] = 0x02027;  /* PH12 */
			/* tdm_config.tdm_hw_config[1] = 0x73007;*/  /* PH12 */
			tdm_config.tdm_hw_config[2] = 0x101f003f; /* FRCFG */
		}
	}

	tdm_config.tdm_hw_config[3] = 0;
	tdm_config.tdm_cfg_bits.val = new_tdm_config->val;
	tdm_config.tdm_cfg_bits.part.tdm_sel =
		(1 << new_tdm_config->part.tdm_sel);

	frame_len = frame_len |
			(((u32)mb_mem_location << TDM_MB_MEM_LOC_BITN) &
						TDM_MB_MEM_LOC_MASK);
	tdm_config.frame_len = frame_len;

	tdm_config.task_handler = task_handler;

	dev_info(p->dev,
	"%s: tdm_cfg: = 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X\n",
		__func__,
		tdm_config.tdm_cfg_bits.val,
		tdm_config.tdm_hw_config[0],
		tdm_config.tdm_hw_config[1],
		tdm_config.tdm_hw_config[2],
		tdm_config.tdm_hw_config[3],
		tdm_config.frame_len,
		tdm_config.task_handler);

	/* Send message */
	ret = dbmdx_send_message_ack(p, MSG_ID_FW, reg_num, (u8 *)&tdm_config,
					sizeof(tdm_config), &ack_val);
	if (ret < 0)
		dev_err(p->dev, "%s: Error configuring TDM\n", __func__);
	else
		dev_info(p->dev, "%s: TDM has been successfully configured\n",
			__func__);


	return ret;
}

/*
 *
 *
 * tdm_rx_enable_bits  TDM Rx blocks 0-7,
 *			0: disable, 1: enable
 *
 * tdm_tx_enable_bits  TDM Tx blocks 0-7, 0:
 *			0: disable, 1: enable
 * mic_mb_enable_bits  Enable microphone's mailbox.
 *			Setting bits 16 to 19 will enable
 *			microphone's mailbox number 1 to 4
 *			respectively.
 */
int dmbdx_tdm_activation(struct dbmdx_private *p,
			u8 tdm_rx_enable_bits,
			u8 tdm_tx_enable_bits,
			u8 mic_mb_enable_bits)
{
	int	ret;
	u32	ack_val;
	union	tdm_activation_reg tdm_activation;

	tdm_activation.val = 0;
	tdm_activation.part.rx_tdm_en_bits_field = tdm_rx_enable_bits;
	tdm_activation.part.tx_tdm_en_bits_field = tdm_tx_enable_bits;
	tdm_activation.part.reset_mic_mb = mic_mb_enable_bits;

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, REG_FW__TDM_ACTIVATION,
					tdm_activation.val, &ack_val);


	if (ret < 0) {
		dev_err(p->dev, "%s: Error activating TDM  (val=0x%08x)\n",
			__func__, tdm_activation.val);
		return ret;
	} else {
		dev_info(p->dev, "%s: TDM was activated (val=0x%08x)\n",
			__func__, tdm_activation.val);
	}

	return 0;
}

int dmbdx_tdm_reset_configuration(struct dbmdx_private *p, u8 tdm_mb_idx)
{
	int	ret = 0;
	u32	ack_val;

	dev_dbg(p->dev, "%s: tdm_mb:0x%x\n", __func__, tdm_mb_idx);

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, tdm_mb_idx,
					0, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error reseting TDM MB\n", __func__);
		return ret;
	}

	return 0;
}

int dmbdx_create_task(struct dbmdx_private *p, u32 task_number, u8 vendor,
			u8 execution_core, u32 *task_handle, u32 *queue_id)
{
	int ret;
	u32 ack_val;
	union create_task_word task_word;

	dev_info(p->dev,
	    "%s: task = 0x%x core %d\n", __func__, task_number, execution_core);

	if (!atomic_add_unless(&(p->primary_flags.task_create_in_process),
									1, 1)) {
		dev_err(p->dev,	"%s: Concurrent task creation is forbidden\n",
			__func__);
		return -EINVAL;
	}

	p->primary_flags.task_waiting_for_task_handle = true;
	p->primary_flags.task_waiting_for_queue_id = true;

	task_word.val = 0;
	task_word.part.number = task_number;
	task_word.part.vensor_id = vendor;
	task_word.part.core = execution_core;

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_RTOS, REG_RTOS__CREATE,
					task_word.val, &ack_val);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error during task creation request\n",
			__func__);
		p->primary_flags.task_waiting_for_task_handle = false;
		p->primary_flags.task_waiting_for_queue_id = false;
		atomic_set(&(p->primary_flags.task_create_in_process), 0);
		return -EINVAL;
	}

	ret = wait_event_interruptible_timeout(p->host_event_wq,
		!atomic_read(&(p->primary_flags.task_create_in_process)),
			msecs_to_jiffies(DBMDX_MSLEEP_TASK_CREATE_TIMEOUT));

	if (ret == 0) {
		/* timeout */
		dev_err(p->dev, "%s: Timeout while waiting for Task Handle\n",
			__func__);
		p->primary_flags.task_waiting_for_task_handle = false;
		p->primary_flags.task_waiting_for_queue_id = false;
		atomic_set(&(p->primary_flags.task_create_in_process), 0);

		return -ETIMEDOUT;
	} else if (ret < 0) {
		/* Wait Interrupted */
		dev_err(p->dev, "%s: Wait for Task Handle was interrupted\n",
			__func__);
		p->primary_flags.task_waiting_for_task_handle = false;
		p->primary_flags.task_waiting_for_queue_id = false;
		atomic_set(&(p->primary_flags.task_create_in_process), 0);
		return -EIO;
	}

	*task_handle = p->primary_flags.current_task_handle;
	*queue_id = p->primary_flags.current_queue_id;

	dev_info(p->dev,
		"%s: Task was successfully created: ID: 0x%x, VID: 0x%x, Core: 0x%x, Handle: 0x%x, QID: 0x%x\n",
		__func__, task_word.part.number, task_word.part.vensor_id,
		task_word.part.core, *task_handle, *queue_id);

	return 0;
}

void dbmdx_get_task_handles(struct dbmdx_private *p,
					u32 *task_id, u32 *queue_id)
{
	dev_err(p->dev, "%s:\n", __func__);

	msleep(20);
	/* TODO: dbmdx_get_pending_word(p, task_id); */
	*task_id = 0;
	msleep(20);
	/* TODO:  dbmdx_get_pending_word(p, queue_id); */
	*queue_id = 0;
	msleep(20);

	dev_info(p->dev, "%s: task_id<%x> queue_id<%x>",
			__func__, *task_id,  *queue_id);
	dev_info(p->dev, "%s: Finish.\n", __func__);
}

int dbmdx_create_data_container_mailbox(struct dbmdx_private *p,
					u32 mailbox_id, u8 mb_mem_location,
					u32 mb_size, u8 owner_core)
{
	/* mailbox attribues */
	int ret;
	u32 ack_val;

	dev_info(p->dev, "%s:\n", __func__);

	mailbox_id = mailbox_id |
			((u32)owner_core << MB_CREATE_WORD1_OWNER_CORE_BITN);

	mb_size = mb_size | ((u32)mb_mem_location << MB_MEM_TYPE_BITS);

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__CREATE_MB,
					mailbox_id, mb_size, &ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error creating mailbox\n", __func__);
		return ret;
	} else
		dev_info(p->dev,
			"%s: Mailbox was successfully created mb_size 0x%x\n",
			__func__, mb_size);

	return ret;
}

int dbmdx_create_generic_mailbox(struct dbmdx_private *p, u32 mailbox_id,
				u8 core, u32 number_of_channels,
				u32 frame_len_w, u32 frame_len_r,
				u32 fw_attrib_smp_rate,
				u32 fw_attrib_smp_width, u32 notification_bit,
				u8  mem_map)
{
	/* mailbox attribues */
	union audio_attrib mail_box_attrib;
	union mb_create_by_attr_word1 mb_word1;

	char fw_str[DBMDX_FW_DBG_STRING_LENGTH];
	int ret;
	u32 ack_val;

	dev_info(p->dev, "%s: mailbox_id: 0x%x\n", __func__, mailbox_id);

	snprintf(fw_str, DBMDX_FW_DBG_STRING_LENGTH,
		"Create MB #%x, ch %d, frm %d %d, %d Hz, %d bits, notify %d memmap %d\n",
		mailbox_id, number_of_channels, frame_len_w,
		frame_len_r, fw_attrib_smp_rate_to_hz[fw_attrib_smp_rate],
		8*(fw_attrib_smp_width+1), notification_bit, mem_map);

	dbmdx_fw_print_host_message(p, fw_str);

	mail_box_attrib.val = 0;
	mail_box_attrib.part.smp_freq = fw_attrib_smp_rate;
	mail_box_attrib.part.smp_width = fw_attrib_smp_width;
	mail_box_attrib.part.nch = number_of_channels;
	mail_box_attrib.part.notify_bit = notification_bit;

	mb_word1.val = 0;
	mb_word1.part.mailbox_id = mailbox_id;
	mb_word1.part.core = core;
	mb_word1.part.mem_loc = mem_map;

	if ((mailbox_id & 0xFFFF) < 5)
		mail_box_attrib.part.aux1 = 1;

	ret = dbmdx_write_register96_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__CREATE_MB,
					mb_word1.val, mail_box_attrib.val,
					(frame_len_w << 16) | frame_len_r,
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error creating mailbox\n", __func__);
		return ret;
	}


	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_MAILBOX, REG_MB__WATERMARK, mailbox_id,
				frame_len_r, &ack_val);
	if (ret < 0)
		dev_err(p->dev,	"%s: Error creating mailbox\n", __func__);
	else
		dev_info(p->dev, "%s: Mailbox was successfully created\n",
			__func__);

	return ret;
}

/*
 *
 * sample_requency   0:16kHz, 1:44.1kHz, 2:48kHz, 3:96kHz, 4:8kHz, 5:32kHz
 * num_channels      0:1ch, other 1-15 ch.
 * read_client_number
 * sample_width      0:8 bits, 1:16 bits, 2: 24 bits, 3:32bits
 *
 * return		0:On success Errcode otherwise
 */
int dmbdx_mailbox_attr_set(struct dbmdx_private *p,
				u32 mailbox_id,
				u32	sample_requency,
				u32	num_channels,
				u32	read_client_number,
				u32	sample_width)
{
	u32 ack_val;
	int ret;
	union audio_attrib mailbox_attribute;

	mailbox_attribute.val = 0;
	mailbox_attribute.part.smp_freq = sample_requency;
	mailbox_attribute.part.nch = num_channels;
	mailbox_attribute.part.notify_bit = 0;
	mailbox_attribute.part.mb_read_client = read_client_number;
	mailbox_attribute.part.smp_width = sample_width;

	dev_info(p->dev, "%s: 0x%08X\n", __func__, mailbox_attribute.val);

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__ATTRIBUTE,
					mailbox_id, mailbox_attribute.val,
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error settings MB Attributes\n", __func__);
		return ret;
	}

	return 0;
}

int dmbdx_mailbox_attr_disable_overflow_set(struct dbmdx_private *p,
					u32 mailbox_id, bool disable_overflow)
{
	u32 ack_val;
	int ret;
	struct mailbox_data_attrib mailbox_attribute;

	mailbox_attribute.audio.val = 0;
	mailbox_attribute.control.val = 0;

	/* Read MB attributes */
	ret = dbmdx_read_register64_index(p, MSG_ID_MAILBOX, REG_MB__ATTRIBUTE,
					mailbox_id, (u64 *)&mailbox_attribute);

	if (ret < 0) {
		dev_err(p->dev, "%s: Error reading MB 0x%x Attribute\n",
			 __func__, mailbox_id);
		return ret;
	}


	dev_info(p->dev,
		"%s: MB_ID<0x%x> aud<0x%x> cntrl<0x%x> Overflow Dis.<%d>\n",
		__func__, mailbox_id, mailbox_attribute.audio.val,
		mailbox_attribute.control.val, disable_overflow);

	/* Modify */
	if (disable_overflow) {
		if (mailbox_attribute.control.part.dis_overflow == 0) {
			mailbox_attribute.control.part.dis_overflow = 1;
		} else {
			dev_info(p->dev,
				"%s: MB Overflow is already disabled\n",
				__func__);
			return 0;
		}
	} else {
		if (mailbox_attribute.control.part.dis_overflow) {
			mailbox_attribute.control.part.dis_overflow = 0;
		} else {
			dev_info(p->dev,
				"%s: MB Overflow is already enabled\n",
				__func__);
			return 0;
		}
	}

	/* Update MB attributes */
	ret = dbmdx_write_register96_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__ATTRIBUTE,
					mailbox_id, mailbox_attribute.audio.val,
					mailbox_attribute.control.val,
					&ack_val);
	if (ret < 0)
		dev_err(p->dev, "%s: Error updating MB 0x%x Attribute\n",
			__func__, mailbox_id);

	return ret;
}

int dbmdx_create_mailbox_by_size(struct dbmdx_private *p, u32 mailbox_id,
				u8 mb_core, u32 mailbox_size,
				u8  mailbox_mem_location,
				u32 frame_len_w, u32 frame_len_r,
				u32 fw_attrib_smp_rate,
				u32 fw_attrib_smp_width,
				u32 number_of_channels,
				u32 read_client_number)
{
	int ret = -1;
	u32 ack_val;
	union mb_create_by_size_word1 mb_word1;
	union mb_create_by_size_word2 mb_word2;

	mb_word1.val = 0;
	mb_word1.part.mb_id = mailbox_id;
	mb_word1.part.core = mb_core;

	mb_word2.val = 0;
	mb_word2.part.size = mailbox_size;
	mb_word2.part.mem_location = mailbox_mem_location;

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__CREATE_MB,
					mb_word1.val, mb_word2.val, &ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Create mailbox ID <%x>\n",
		    __func__, mb_word1.part.mb_id);
		return ret;
	}

	ret = dmbdx_mailbox_attr_set(p, mailbox_id, fw_attrib_smp_rate,
					number_of_channels, read_client_number,
					fw_attrib_smp_width);

	if (ret < 0) {
		dev_err(p->dev,	"%s: Error settings MB Attributes\n", __func__);
		return ret;
	}

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__FRAME_ALIGNMENT,
					mailbox_id,
					((frame_len_w << 16) | frame_len_r),
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: REG_MB__FRAME_ALIGNMENT\n", __func__);
		return ret;
	}

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__WATERMARK,
					mailbox_id, frame_len_r, &ack_val);
	if (ret < 0) {
		dev_err(p->dev,	"%s: REG_MB__WATERMARK\n", __func__);
		return ret;
	}

	return ret;
}

int dmbdx_delete_mailbox(struct dbmdx_private *p, u8 mailbox_id)
{

	int	ret;
	u32	ack_val;

	dev_dbg(p->dev, "%s:  mailbox_id: 0x%x\n", __func__, mailbox_id);

	ret = dbmdx_write_register16_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__DELETE_MB,
					mailbox_id, &ack_val);

	if (ret < 0)
		dev_err(p->dev,	"%s: Error deleting mailbox 0x%x\n",
			 __func__, mailbox_id);

	return ret;
}

int dmbdx_mic_config(struct dbmdx_private *p,
		    u8 ddf,
		    u8 dm_clock,
		    u8 sampling_rate,
		    u8 mic_num_hw_vad,
		    u8 enable_hw_vad,
		    u8 ddf_int_core,
		    u8 configure_ddf,
		    u8 reconfigure_gpio)
{
	u32 ack_val;
	int ret;
	union mic_config mic_config_word;

	mic_config_word.val = 0;
	mic_config_word.conf.ddf = ddf;
	mic_config_word.conf.dm_clock = dm_clock;
	mic_config_word.conf.sampling_rate = sampling_rate;
	mic_config_word.conf.mic_num_hw_vad = mic_num_hw_vad;
	mic_config_word.conf.enable_hw_vad = enable_hw_vad;
	mic_config_word.conf.ddf_int_core = ddf_int_core;
	mic_config_word.conf.configure_ddf = configure_ddf;
	mic_config_word.conf.reconfigure_gpio = reconfigure_gpio;

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_FW, REG_FW__CONFIG_MICS,
				mic_config_word.val, &ack_val);
	if (ret < 0)
		dev_err(p->dev,
			"%s: Error configuring microphone: : 0x%08x\n",
			__func__, mic_config_word.val);
	else
		dev_info(p->dev,
			"%s: Microphone was successfully configured: 0x%08x\n",
			__func__, mic_config_word.val);
	return ret;
}



#define OPERATION_MODE_SUBMODE_GETSET_CURRENT_PRESET	0
#define OPERATION_MODE_SUBMODE_PROGRAM_PRESET		1
#define OPERATION_MODE_SUBMODE_COPY_PRESET		3
#define OPERATION_MODE_SUBMODE_MODIFY_PRESET_PARAM	4


int dmbdx_opmode_set_current_preset_num(struct dbmdx_private *p,
						u8 preset_number)
{
	union operation_mode mode;
	int ret;
	u32 ack_val;

	mode.val = 0;

	dev_info(p->dev, "%s: 0x%02x\n", __func__, preset_number);

	mode.conf.sub_mode = OPERATION_MODE_SUBMODE_GETSET_CURRENT_PRESET;
	mode.conf.param1 = preset_number;
	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_FW,
					REG_FW__SYS_OPERATION_MODE, mode.val,
					&ack_val);
	if (ret < 0)
		dev_err(p->dev,
			"%s: Error setting opmode preset number: 0x%02x\n",
			__func__, preset_number);
	else
		dev_info(p->dev,
			"%s: Opmode preset number was set to: 0x%02x\n",
			__func__, preset_number);

	return ret;
}

int dmbdx_opmode_copy_preset(struct dbmdx_private *p, u8 src_preset,
				u8 dst_preset)
{
	union operation_mode mode;
	int ret;
	u32 ack_val;

	mode.val = 0;

	mode.conf.sub_mode = OPERATION_MODE_SUBMODE_COPY_PRESET;
	mode.conf.param1 = src_preset;
	mode.conf.param2 = dst_preset;

	dev_info(p->dev, "%s: Copy opmode preset from:%d to:%d  (val=0x%08x)\n",
				__func__, src_preset, dst_preset, mode.val);

	/* This WO sub-mode copies the content of preset number “param1”
	 * into preset number “param2”. --> copy preset 0 to preset 1
	 */
	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_FW,
					REG_FW__SYS_OPERATION_MODE, mode.val,
					&ack_val);
	if (ret < 0)
		dev_err(p->dev,
		"%s: Error copying opmode preset (from:%d to:%d  val=0x%08x)\n",
			__func__, src_preset, dst_preset, mode.val);
	else
		dev_info(p->dev,
		"%s: Opmode preset was successfully copied (from:%d to:%d  val=0x%08x)\n",
			__func__, src_preset, dst_preset, mode.val);

	return ret;
}

/*
 *
 * master_clock_hz - [e.g. 24576000]
 * param sample_rate - [e.g. 8000,16000,32000,48000]
 * param num_of_channels - 1,2
 * param sample_width - 16,32
 * param clock_select - 0 - MSTR_OR_OSC_CLK
 *			1 - TDM0_SCLK
 */
int dmbdx_clock_config(struct dbmdx_private *p,
			u32 master_clock_hz,
			u32 sample_rate_hz,
			u8 num_of_channels,
			u8 sample_width,
			u32 clock_select,
			u8 src_preset,
			u8 dst_preset)
{
	int	ret;
	u32 ack_val;
	u32 tdm_clock;
	u32 pre_div_clock;

	tdm_clock = sample_rate_hz * num_of_channels * sample_width;

	dev_info(p->dev,
		"%s:sample_rate: %d Hz sample_width:%d tdm_clock 0x%08X\n",
		__func__, sample_rate_hz, sample_width, tdm_clock);

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, REG_FW__CLK_IN_FREQ,
					master_clock_hz, tdm_clock, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error writing register\n", __func__);
		return ret;
	}

	ret = dmbdx_opmode_copy_preset(p, src_preset, dst_preset);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error copying opmode preset from %d to %d\n",
			__func__, (int)src_preset, (int)dst_preset);
		return ret;
	}

	pre_div_clock = (tdm_clock / 128000) - 1;

	dev_info(p->dev, "%s: pre_div_clock 0x%x\n", __func__, pre_div_clock);

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_FW, REG_FW__SYS_OPERATION_MODE,
					(PRESET_PARAM_PLL_PRE_DIV_VAL |
								dst_preset),
					pre_div_clock, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting PLL Pre divider\n",
			__func__);
		return ret;
	}

	/*0xca8 ?????? 0xb40 */
	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_FW, REG_FW__SYS_OPERATION_MODE,
					(PRESET_PARAM_PLL_FB_DIV_VAL |
								dst_preset),
					0xb40, &ack_val);
	if (ret < 0) {
		dev_err(p->dev,  "%s: Error setting PLL FB divider\n",
			__func__);
		return ret;
	}

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_LOOP,
					MSG_ID_FW, REG_FW__SYS_OPERATION_MODE,
					(PRESET_PARAM_CLK_SEL_COMMAND |
								dst_preset),
					clock_select, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting Clk Sel.\n", __func__);
		return ret;
	}

	ret = dmbdx_opmode_set_current_preset_num(p, dst_preset);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting preset to %d\n",
			__func__, (int)dst_preset);
		return ret;
	}

	return ret;
}

static int dbmdx_load_params_file_to_buf(struct dbmdx_private *p,
					const char *params_file_name,
					char	**file_buf,
					ssize_t *file_buf_size,
					bool pad_to_word)
{
	int ret;
	struct firmware	*param_file_fw = NULL;
	char *data_buf = NULL;
	u32 buffer_size_padded;


	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!params_file_name[0]) {
		dev_err(p->dev, "%s: Unknown Params file name\n", __func__);
		return -ENOENT;
	}

	dev_dbg(p->dev, "%s: loading %s\n", __func__, params_file_name);

	ret = request_firmware((const struct firmware **)&param_file_fw,
				params_file_name,
				p->dbmdx_dev);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to request Params File Name(%d)\n",
			__func__, ret);
		return -ENOENT;
	}

	dev_info(p->dev, "%s: Params file %s was loaded, size %zu bytes\n",
			__func__, params_file_name, param_file_fw->size);


	if (!param_file_fw->size) {
		dev_warn(p->dev, "%s Params file size is 0. Ignore...\n",
			__func__);
		ret = -EIO;
		goto release;
	}

	/* If padding should be used */
	if (pad_to_word) {
		buffer_size_padded = param_file_fw->size & 0xFFFFFFFC;
		if (buffer_size_padded != param_file_fw->size)
			buffer_size_padded += 4;
	} else {
		buffer_size_padded = param_file_fw->size;
	}

	data_buf = vzalloc(buffer_size_padded);
	if (!data_buf) {
		dev_err(p->dev,	"%s: Cannot allocate memory for params file\n",
			__func__);
		return -ENOMEM;
	}

	memcpy(data_buf, param_file_fw->data, param_file_fw->size);

	*file_buf_size = param_file_fw->size;
	*file_buf = data_buf;

release:
	if (param_file_fw)
		release_firmware(param_file_fw);

	return ret;
}

int dbmdx_upload_params_file(struct dbmdx_private *p,
				const char *params_file_name,
				u32 mailbox_id,
				u32 core_id,
				u8 mb_mem_location)
{
	int ret = 0, ret2 = 0;
	char *data_buf = NULL;
	ssize_t data_buf_size = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_info(p->dev, "%s: filename: %s\n", __func__,
					params_file_name);

	ret = dbmdx_load_params_file_to_buf(p, params_file_name, &data_buf,
							&data_buf_size, true);

	if (ret < 0) {
		dev_err(p->dev, "%s: failed to load params file\n", __func__);
		goto out;
	}

	ret = dbmdx_create_data_container_mailbox(p, mailbox_id,
						CACHED_SHARED_MEMORY,
						data_buf_size,
						core_id);
	if (ret < 0) {
		dev_err(p->dev, "%s: failed to create MB for params file\n",
			__func__);
		goto out_mem;
	}

	/* prepare the chip interface for param file loading */
	ret = p->chip->prepare_amodel_loading(p);
	if (ret != 0) {
		dev_err(p->dev,
			"%s: failed to prepare for param file loading\n",
			__func__);
		goto out_mem;
	}

	ret = dbmdx_send_buf_to_mb(p, data_buf, data_buf_size, mailbox_id);

	if (ret) {
		dev_err(p->dev, "%s: Sending params failed\n",
			__func__);
		ret = -EIO;
		goto out_finish_loading;
	}

	ret = dbmdx_check_if_alive(p);
	if (ret) {
		dev_err(p->dev, "%s: fw is dead\n", __func__);
		ret = -EIO;
		goto out_finish_loading;
	}

	dev_info(p->dev, "%s: Params file: %s was sent successfully\n",
		__func__, params_file_name);

out_finish_loading:
	/* finish A-Model loading */
	ret2 = p->chip->finish_amodel_loading(p);
	if (ret2 != 0)
		dev_err(p->dev, "%s: failed to finish params loading\n",
			__func__);

out_mem:
	vfree(data_buf);
out:
	return ret;
}

/*
 * This register sets the RTOS task handle which should read
 * samples from the mailbox. When a frame of samples are ready
 * for reading (according to the mailbox's threshold) FW calls
 * xTaskNotify using eAction=eSetBits to wake up the task
 * and initiate the frame processing.  ulValue is set according
 * to the configured notification bit number (see mailbox
 * attributes).
 */
int dmbdx_connect_read_task_to_mailbox(struct dbmdx_private *p,
					u32 task_handle, u32 mailbox_id)
{
	u32 ack_val;
	int ret = 0;

	return dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX, REG_MB__RTOS_READ_TASK,
					mailbox_id, task_handle,
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error during connecting task to mb\n",
			__func__);
		return ret;
	}

	return 0;
}

/*
 * This register generates two RTOS function calls,
 * xQueueGenericSend and xTaskNotify. The notification resumes
 * the task execution so it can read the message waiting for it
 * in the queue
 *
 */
int dmbdx_task_send_cmd_and_notify_task(struct dbmdx_private *p,
					u32 task_id, u32 queue_id,
					u32 mailbox_id, u32 cmd)
{
	u32 ack_val;
	int ret = 0;

	ret = dbmdx_write_register96_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_RTOS,
					REG_RTOS__SEND_QUEUE_NOTIFY,
					task_id, queue_id,
					TASK_CONFIG_CREATE_CMD_AND_DATA(cmd,
								mailbox_id),
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error during notifying task\n", __func__);
		return ret;
	}

	return 0;
}

int dbmdx_write_fw_reg(struct dbmdx_private *p,
				struct fw_register *fw_reg)
{
	int ret = 0;
	u32 ack_val;

	switch (fw_reg->cmd) {
	case WRITE_REG_TYPE_16:
		ret = dbmdx_write_register16_ack(p,
						fw_reg->priority,
						fw_reg->id,
						fw_reg->reg_num,
						fw_reg->val1,
						&ack_val);
		break;
	case WRITE_REG_TYPE_32:
		ret = dbmdx_write_register32_ack(p,
						fw_reg->priority,
						fw_reg->id,
						fw_reg->reg_num,
						fw_reg->val1,
						&ack_val);
		break;
	case WRITE_REG_TYPE_64:
		ret = dbmdx_write_register64_ack(p,
						fw_reg->priority,
						fw_reg->id,
						fw_reg->reg_num,
						fw_reg->val1,
						fw_reg->val2,
						&ack_val);
		break;
	case WRITE_REG_TYPE_96:
		ret = dbmdx_write_register96_ack(p,
						fw_reg->priority,
						fw_reg->id,
						fw_reg->reg_num,
						fw_reg->val1,
						fw_reg->val2,
						fw_reg->val3,
						&ack_val);
		break;
	case WRITE_REG_MSLEEP_MSEC:
		msleep(fw_reg->val1);
		break;
	default:
		break;
	}
	return ret;
}


int dbmdx_read_fw_reg(struct dbmdx_private *p,
				struct fw_register *fw_reg, u64 *read_val)
{
	int ret = 0;
	u16 read_val_16 = 0;
	u32 read_val_32 = 0;

	switch (fw_reg->cmd) {
	case READ_REG_TYPE_16:
		ret = dbmdx_read_register16(p, fw_reg->id, fw_reg->reg_num,
						&read_val_16);
		*read_val = (u64)read_val_16;
		break;
	case READ_REG_TYPE_32:
		ret = dbmdx_read_register32(p, fw_reg->id, fw_reg->reg_num,
						&read_val_32);
		*read_val = (u64)read_val_32;
		break;
	case READ_REG_TYPE_32_INDEX:
		ret = dbmdx_read_register32_index(p, fw_reg->id,
						fw_reg->reg_num, fw_reg->val1,
						&read_val_32);
		*read_val = (u64)read_val_32;
		break;
	case READ_REG_TYPE_64:
		ret = dbmdx_read_register64(p, fw_reg->id, fw_reg->reg_num,
						read_val);
			break;
	case READ_REG_TYPE_64_INDEX:
		ret = dbmdx_read_register64_index(p, fw_reg->id,
						fw_reg->reg_num, fw_reg->val1,
						read_val);
		break;
	default:
		break;
	}
	return ret;
}


int dbmdx_send_init_cfg_reg_list(struct dbmdx_private *p)
{
	int ret = 0;
	int i = 0;

	for (i = 0; i < p->pdata->init_cfg_reg_list_items; i++) {
		ret = dbmdx_write_fw_reg(p, &(p->pdata->init_cfg_reg_list[i]));
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error sending item #%d from init_cfg_reg_list\n",
				__func__, i);
			return ret;
		}
	}

	return ret;
}
