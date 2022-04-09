/*
 * dbmdx-fw-cmd-interface.h  --  DBMDX Chip Interfaces API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_FW_CMD_INTERFACE_H
#define _DBMDX_FW_CMD_INTERFACE_H

#define DBMDX_PROT_HEADER_SIZE			8

struct fw_message_header {
	u8	priority;
	u8	m_type;
	u8	id;
	u8	reg_num;
	union {
		u16	sub_mode;
		u16	reg_value;
	};
};

struct fw_message {
	struct fw_message_header	header;
	u32				reg_info[2];
	u32				var_msg_len_bytes;
	u8				*var_msg;
};

int dbmdx_rtos_create_task(struct dbmdx_private *p,
				u32	task_id,
				u32	stack_size,
				u32	optional,
				u32	core_id_priority);

int dbmdx_send_message(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len);

int dbmdx_send_message_ack(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len,
			u32	*ack_val);

int dbmdx_read_register16(struct dbmdx_private *p,
				u8	id,
				u8	reg_num,
				u16	*read_val);

int dbmdx_read_register32(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u32	*read_val);

int dbmdx_read_register32_index(struct dbmdx_private *p,
				u8	id,
				u8	reg_num,
				u32	idx,
				u32	*read_val);

int dbmdx_read_register64(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u64	*read_val);

int dbmdx_read_register64_index(struct dbmdx_private *p,
				u8	id,
				u8	reg_num,
				u32	idx,
				u64	*read_val);

int dbmdx_write_register16(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u16	reg_val);

int dbmdx_write_register16_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u16	reg_val,
				u32	*ret_val);

int dbmdx_write_register32(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val);

int dbmdx_write_register32_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val,
				u32	*ret_val);

int dbmdx_write_register64(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2);

int dbmdx_write_register64_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	*ret_val);

int dbmdx_write_register96(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	reg_val_3);

int dbmdx_write_register96_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	reg_val_3,
				u32	*ack_val);

int dbmdx_get_pending_message(struct dbmdx_private *p,
				bool *message_is_pending,
				struct fw_message *pending_message);

void dbmdx_free_fw_message(struct dbmdx_private *p,
				struct fw_message *message);

int dbmdx_send_buf_to_mb(struct dbmdx_private *p,
			char	*buf,
			size_t buf_size,
			u32 mailbox_id);

int dbmdx_read_buf_reg32(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u32	reg_val_1,
			u8	**buf,
			u32	buf_len);

int dbmdx_read_buf_reg64(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u32	reg_val_1,
			u32	reg_val_2,
			u8	**buf,
			u32	buf_len);

int dbmdx_read_buf_msg(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len,
			u8	**buf,
			u32	buf_len);

#endif /* DBMDX_FW_CMD_INTERFACE_H */

