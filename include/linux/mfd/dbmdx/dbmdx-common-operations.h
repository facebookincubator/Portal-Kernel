/*
 * dbmdx-fw-common-operations.h  --  DBMDX Chip FW operations
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_FW_COMMON_OPERATIONS_H
#define _DBMDX_FW_COMMON_OPERATIONS_H

#include "dbmd7-fw-struct-common.h"

#define PRESET_PARAM_CLK_SEL_MSTR_OR_OSC_CLK	0
#define PRESET_PARAM_CLK_SEL_TDM0_SCLK		1

int dbmdx_get_fw_attrib_dm_clock_freq(u32 dm_clock_freq_val,
					u8 *fw_attrib_dm_clock);
int dbmdx_get_fw_attrib_sample_width(u32 sample_width,
					u8 *fw_attrib_sample_width);

int dbmdx_get_fw_attrib_sample_rate(u32 sample_rate_val,
					u8 *fw_attrib_sample_rate);
int dbmdx_get_fw_attrib_tdm_tx_mb_index(u32 tdm_tx,
				u8 *fw_attrib_tdm_tx_mb_idx);

int dbmdx_get_fw_attrib_tdm_rx_mb_index(u32 tdm_rx,
				u8 *fw_attrib_tdm_rx_mb_idx);

int dbmdx_check_if_alive(struct dbmdx_private *p);

int dbmdx_fw_print_host_message(struct dbmdx_private *p, u8 *fw_str);

int dmbdx_create_task(struct dbmdx_private *p, u32 task_number, u8 vendor,
			u8 execution_core, u32 *task_handle, u32 *queue_id);

int dbmdx_process_pending_messages(struct dbmdx_private *p);

int dmbdx_clock_config(struct dbmdx_private *p,
			u32 master_clock_hz,
			u32 sample_rate_hz,
			u8 num_of_channels,
			u8 sample_width,
			u32 clock_select,
			u8 src_preset,
			u8 dst_preset);

int dbmdx_tdm_setup(struct dbmdx_private *p, u8 reg_num,
			union tdm_cfg1 *tdm_config,
			u32 frame_len,
			u32 task_handler,
			u8 tdm_mode,
			u8 mb_mem_location);

int dmbdx_tdm_activation(struct dbmdx_private *p,
			u8 tdm_rx_enable_bits,
			u8 tdm_tx_enable_bits,
			u8 tdm_enable_mic_mb);

int dmbdx_tdm_reset_configuration(struct dbmdx_private *p, u8 tdm_mb_idx);


int dbmdx_create_generic_mailbox(struct dbmdx_private *p, u32 mailbox_id,
				u8 core, u32 number_of_channels,
				u32 frame_len_w, u32 frame_len_r,
				u32 fw_attrib_smp_rate,
				u32 fw_attrib_smp_width, u32 notification_bit,
				u8  mem_map);

int dbmdx_create_data_container_mailbox(struct dbmdx_private *p,
					u32 mailbox_id, u8 mb_mem_location,
					u32 mb_size, u8 owner_core);

int dmbdx_mailbox_attr_set(struct dbmdx_private *p,
				u32 mailbox_id,
				u32	sample_requency,
				u32	num_channels,
				u32	read_client_number,
				u32	sample_width);

int dmbdx_mailbox_attr_disable_overflow_set(struct dbmdx_private *p,
					u32 mailbox_id, bool disable_overflow);


int dbmdx_create_mailbox_by_size(struct dbmdx_private *p, u32 mailbox_id,
				u8 mb_core, u32 mailbox_size,
				u8  mailbox_mem_location,
				u32 frame_len_w, u32 frame_len_r,
				u32 fw_attrib_smp_rate,
				u32 fw_attrib_smp_width,
				u32 number_of_channels,
				u32 read_client_number);

int dmbdx_delete_mailbox(struct dbmdx_private *p, u8 mailbox_id);


void dbmdx_get_task_handles(struct dbmdx_private *p,
				u32 *task_id, u32 *que_id);

int dmbdx_mic_config(struct dbmdx_private *p,
			u8 ddf,
			u8 dm_clock,
			u8 sampling_rate,
			u8 mic_num_hw_vad,
			u8 enable_hw_vad,
			u8 ddf_int_core,
			u8 configure_ddf,
			u8 reconfigure_gpio);

int dmbdx_opmode_set_current_preset_num(struct dbmdx_private *p,
						u8 preset_number);

int dbmdx_upload_params_file(struct dbmdx_private *p,
				const char *params_file_name,
				u32 mailbox_id, u32 core_id,
				u8 mb_mem_location);

int dmbdx_connect_read_task_to_mailbox(struct dbmdx_private *p,
					u32 task_handle,
					u32 mailbox_id);

int dmbdx_task_send_cmd_and_notify_task(struct dbmdx_private *p,
					u32 task_id,
					u32 queue_id,
					u32 mailbox_id,
					u32 cmd);

int dbmdx_send_init_cfg_reg_list(struct dbmdx_private *p);

int dbmdx_write_fw_reg(struct dbmdx_private *p,
				struct fw_register *fw_reg);

int dbmdx_read_fw_reg(struct dbmdx_private *p,
				struct fw_register *fw_reg,
				u64 *read_val);

int dbmdx_core_mb_buffering(struct dbmdx_private *p, u16 mailbox_id);


#endif /* _DBMDX_FW_COMMON_OPERATIONS_H */

