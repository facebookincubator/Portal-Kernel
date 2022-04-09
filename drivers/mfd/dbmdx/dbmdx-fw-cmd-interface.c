/*
 * dbmdx-fw-cmd-interface.c - DSPG DBMDX FW CMD Interface API
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
#include <linux/pm_runtime.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-utils.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmd7-fw-struct-common.h>

#define DBMDX_READ_BUFFER_MAX_SIZE 16
#define DBMDX_WRITE_VAR_LEN_MAX_SIZE 3

#define MESSAGE_HEADER_INIT(_priority, _id, _reg_num, _m_type) do {\
	message.header.priority = _priority;\
	message.header.id = _id;\
	message.header.reg_num = _reg_num;\
	message.header.m_type = _m_type;\
} while (0)

static bool dbmdx_spi_access(struct dbmdx_private *p)
{
	struct task_struct *task = current;

	return p->fw_alive || task == p->task_workq || task == p->task_irq;
}

static void dbmdx_rx_ack_prepare(struct dbmdx_private *p)
{
	p->primary_flags.rx_irq_inuse = 1;
	atomic_set(&(p->primary_flags.rx_ack), 0);
}

static int dbmdx_rx_ack_wait_blocking(struct dbmdx_private *p)
{
	int ret = 0;

	ret = wait_event_timeout(p->rx_ack_wq,
			atomic_read(&(p->primary_flags.rx_ack)),
				msecs_to_jiffies(DBMDX_MSLEEP_RX_ACK_TIMEOUT));

	if (ret == 0) {
		dbmdx_schedule_delayed_work(p, &p->fw_state_work, 0);
		dev_err(p->dev, "%s: Timeout while waiting for RX ACK\n",
			__func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static void dbmdx_rx_ack_clear(struct dbmdx_private *p)
{
	p->primary_flags.rx_irq_inuse = 0;
	atomic_set(&(p->primary_flags.rx_ack), 0);
}


static int dump_fw_message(struct dbmdx_private *p,
				struct fw_message *fw_msg,
				char *buf, u32 buf_len)
{
	int off = 0;
	u16 cmd_size = 0;

	off += snprintf(buf, buf_len, "Pri:0x%x, ", fw_msg->header.priority);

	if (off >= buf_len)
		goto out_dump;

	switch (fw_msg->header.id) {
	case MSG_ID_FW:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x - FW), ",
					fw_msg->header.id);
		break;
	case MSG_ID_ADDON_SENSORY:
		off += snprintf(buf + off, buf_len - off,
					"ID:(0x%x - Sensory), ",
					fw_msg->header.id);
		break;
	case MSG_ID_ADDON_ASRP:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x - ASRP), ",
							fw_msg->header.id);
		break;
	case MSG_ID_MAILBOX:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x - MB), ",
							fw_msg->header.id);
		break;
	case MSG_ID_RTOS:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x - RTOS), ",
							fw_msg->header.id);
		break;
	case MSG_ID_RES:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x - RES), ",
							fw_msg->header.id);
		break;
	default:
		off += snprintf(buf + off, buf_len - off, "ID:(0x%x), ",
							fw_msg->header.id);
		break;
	}

	if (off >= buf_len)
		goto out_dump;

	switch (fw_msg->header.m_type) {
	case MSG_TYPE__WRITE16:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - WRITE16), RegNum:(0x%x), RegVal:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->header.reg_value);
		cmd_size = sizeof(u32);
		break;
	case MSG_TYPE__WRITE16_ACK:
		off += snprintf(buf + off, buf_len - off,
		"Type:(0x%x - WRITE16_ACK), RegNum:(0x%x), RegVal:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->header.reg_value);
		cmd_size = sizeof(u32);
		break;
	case MSG_TYPE__WRITE32:
		off += snprintf(buf + off, buf_len - off,
				"Type:(0x%x - WRITE32), RegNum:(0x%x), ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num);
		if (off >= buf_len)
			goto out_dump;

		switch (fw_msg->header.sub_mode) {
		case MSG_FMT_SUBMODE_GET_32BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET32), RegVal:(0x%x), ",
				fw_msg->header.sub_mode,
				fw_msg->reg_info[0]);
			cmd_size = sizeof(u32)*2;
			break;
		case MSG_FMT_SUBMODE_GET_64BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET64), RegVal_W1:(0x%x), RegVal_W2:(0x%x), ",
				fw_msg->header.sub_mode,
				fw_msg->reg_info[0],
				fw_msg->reg_info[1]);
			cmd_size = sizeof(u32)*3;
			break;
		default:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x- Unknown), ",
				fw_msg->header.sub_mode);
			cmd_size = sizeof(u32);
			break;
		}
		break;
	case MSG_TYPE__WRITE32_ACK:
		off += snprintf(buf + off, buf_len - off,
				"Type:(0x%x - WRITE32_ACK), RegNum:(0x%x), ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num);
		if (off >= buf_len)
			goto out_dump;

		switch (fw_msg->header.sub_mode) {
		case MSG_FMT_SUBMODE_GET_32BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET32), RegVal:(0x%x), ",
				fw_msg->header.sub_mode,
				fw_msg->reg_info[0]);
			cmd_size = sizeof(u32)*2;
			break;
		case MSG_FMT_SUBMODE_GET_64BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET64), RegVal_W1:(0x%x), RegVal_W2:(0x%x), ",
				fw_msg->header.sub_mode,
				fw_msg->reg_info[0],
				fw_msg->reg_info[1]);
			cmd_size = sizeof(u32)*3;
			break;
		default:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x- Unknown), ",
				fw_msg->header.sub_mode);
			cmd_size = sizeof(u32);
			break;
		}
		break;
	case MSG_TYPE__SEND_VAR_LAN_MSG:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - VAR_LEN_MSG), RegNum:(0x%x), VarLength:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->var_msg_len_bytes);
		cmd_size = (sizeof(u32)*2) + fw_msg->var_msg_len_bytes;
		break;
	case MSG_TYPE__SEND_VAR_LAN_MSG_ACK:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - VAR_LEN_MSG_ACK), RegNum:(0x%x), VarLength:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->var_msg_len_bytes);
		cmd_size = (sizeof(u32)*2) + fw_msg->var_msg_len_bytes;
		break;
	case MSG_TYPE__GET_MESSAGE:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - GET_MESSAGE), RegNum:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num);
		if (off >= buf_len)
			goto out_dump;

		switch (fw_msg->header.sub_mode) {
		case MSG_FMT_SUBMODE_GET_16BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET16), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_32BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET32), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_PENDING_MESSAGE:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET_PENDING_MESSAGE), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_64BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET64), ",
				fw_msg->header.sub_mode);
			break;
		default:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - Unknown), ",
				fw_msg->header.sub_mode);
			break;
		}
		cmd_size = sizeof(u32);
		break;
	case MSG_TYPE__GET_MESSAGE4INDEX:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - GET_MESSAGE4INDEX), RegNum:(0x%x), Index:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->reg_info[0]);
		if (off >= buf_len)
			goto out_dump;

		switch (fw_msg->header.sub_mode) {
		case MSG_FMT_SUBMODE_GET_16BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET16), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_32BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET32), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_PENDING_MESSAGE:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET_PENDING_MESSAGE), ",
				fw_msg->header.sub_mode);
			break;
		case MSG_FMT_SUBMODE_GET_64BIT:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - GET64), ",
				fw_msg->header.sub_mode);
			break;
		default:
			off += snprintf(buf + off, buf_len - off,
				"Submode:(0x%x - Unknown), ",
				fw_msg->header.sub_mode);
			break;
		}
		cmd_size = sizeof(u32)*2;
		break;
	default:
		off += snprintf(buf + off, buf_len - off,
			"Type:(0x%x - UNKNOWN), RegNum:(0x%x), RegVal:(0x%x) ",
				fw_msg->header.m_type,
				fw_msg->header.reg_num,
				fw_msg->header.reg_value);
		cmd_size = sizeof(u32);
		return -EINVAL;
	}

	if (off >= buf_len)
		goto out_dump;

	off += snprintf(buf + off, buf_len - off, "CmdSize: (%d bytes) ",
							(int)cmd_size);

out_dump:
	 return off;
}


/*
 * Read ACKED message
 */
static int dbmdx_read_cmd_ack(struct dbmdx_private *p,
			struct fw_message_header *message,
			u32	*read_val)
{
	u8 read_buf[DBMDX_READ_BUFFER_MAX_SIZE + 1] = {0};
	u8 *read_buf_ptr;
	int ret;
	int bytes_to_read = 8;

	if (!dbmdx_spi_access(p))
		return -EPERM;

	if (p->chip->interface_adds_leading_byte_on_read(p)) {
		read_buf_ptr = &(read_buf[1]);
		bytes_to_read += 1;
	} else {
		read_buf_ptr = read_buf;
	}

	ret = p->chip->read(p, read_buf, bytes_to_read);

	if (ret != bytes_to_read) {
		dev_err(p->dev, "%s: register 0x%04x failed(%d)\n",
				__func__, message->reg_num, ret);
		return -EIO;
	}

	if (read_buf_ptr[2] !=  message->reg_num) {
		dev_err(p->dev,
			"%s:read value of register 0x%04x failed <%d>\n",
				__func__, message->reg_num, ret);
		dev_err(p->dev, "%s: %02x:%02x:%02x:%02x:\n",
				__func__, read_buf_ptr[0], read_buf_ptr[1],
				read_buf_ptr[2], read_buf_ptr[3]);
		return -EIO;

	}

	*read_val = 0;
	*read_val = bytearr_to_u32((u8 *)(read_buf_ptr) + 4);

	if (p->dump_traffic > DBMDX_DBG_DUMP_LEVEL_NO_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;
		int i;

		off += snprintf(buf, DBMDX_MAX_DUMP_LENGTH,
				"Register: 0x%04x value: 0x%08x, ",
						message->reg_num, *read_val);

		if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP) {
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
								"Recv <==[");

			for (i = 0; i < bytes_to_read; i++) {
				if (off >= DBMDX_MAX_DUMP_LENGTH)
					break;
				off += snprintf(buf + off,
						DBMDX_MAX_DUMP_LENGTH - off,
						"%02x ", read_buf[i]);
			}
			if (off < DBMDX_MAX_DUMP_LENGTH)
				off += snprintf(buf + off,
						DBMDX_MAX_DUMP_LENGTH - off,
						"]");
		}

		dev_info(p->dev, "%s: %s\n", __func__, buf);
	}

	return 0;
}

static int dbmdx_read_register_ext(struct dbmdx_private *p,
				struct fw_message *message,
				u64	*read_val,
				u32	*message_header_ret)
{
	u32 cmd[2];
	u8 read_buf[DBMDX_READ_BUFFER_MAX_SIZE + 1] = {0};
	int bytes_to_read = 0;
	u8 *read_buf_ptr;
	u8 cmd_size;
	int ret;

	if (!dbmdx_spi_access(p))
		return -EPERM;

	*read_val = 0;
	*message_header_ret = 0;

	/* Set Message Type */
	cmd[0] = (message->header.m_type << MSG_TYPE_BITN);

	/* Set message ID */
	cmd[0] |= (message->header.id << MSG_ID_BITN);

	/* Set Register number */
	cmd[0] |= (message->header.reg_num << MSG_REGNUM_BITN);

	cmd[0] |= message->header.sub_mode;

	switch (message->header.m_type) {
	case MSG_TYPE__GET_MESSAGE:
		cmd_size = sizeof(u32);
		break;
	case MSG_TYPE__GET_MESSAGE4INDEX:
		cmd[1] = message->reg_info[0];
		cmd_size = sizeof(cmd);
		break;
	default:
		dev_err(p->dev, "%s: Illegal Read Message Type: %d\n",
				__func__, message->header.m_type);
		return -EINVAL;
	}


	/* read according to sub mode */
	switch (message->header.sub_mode) {
	case MSG_FMT_SUBMODE_GET_16BIT:
		bytes_to_read = sizeof(u32); /*word0*/
		break;
	case MSG_FMT_SUBMODE_GET_32BIT:
		/* Fall through */
	case MSG_FMT_SUBMODE_GET_PENDING_MESSAGE:
		bytes_to_read = sizeof(u32)*2; /*word0,1*/
		break;
	case MSG_FMT_SUBMODE_GET_64BIT:
		bytes_to_read = sizeof(u32)*3; /*word0,1,2*/
		break;
	default:
		 dev_err(p->dev, "%s: Illegal Read Message SubMode: %d\n",
			 __func__, message->header.sub_mode);
		 return -EINVAL;
	}

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;
		int i;

		off += snprintf(buf, DBMDX_MAX_DUMP_LENGTH, "Send ==> [");

		for (i = 0; i < cmd_size; i++) {
			if (off >= DBMDX_MAX_DUMP_LENGTH)
				break;
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
					"%02x ", ((u8 *)cmd)[i]);
		}
		if (off < DBMDX_MAX_DUMP_LENGTH)
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
					"] (cmd_size: %d, bytes_to_read: %d)",
					cmd_size, bytes_to_read);

		dev_info(p->dev, "%s: %s\n", __func__, buf);
	}

	dbmdx_rx_ack_prepare(p);

	ret = p->chip->write(p, (u8 *)cmd, cmd_size);

	if (ret != cmd_size) {
		dev_err(p->dev,
			"%s: Error sending read register 0x%04x request\n",
			__func__, message->header.reg_num);
		return -EIO;
	}

	ret = dbmdx_rx_ack_wait_blocking(p);
	dbmdx_rx_ack_clear(p);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error waiting for RX ACK\n", __func__);
		return -EIO;
	}

	if (p->chip->interface_adds_leading_byte_on_read(p)) {
		read_buf_ptr = &(read_buf[1]);
		bytes_to_read += 1;
	} else {
		read_buf_ptr = read_buf;
	}

	ret = p->chip->read(p, read_buf, bytes_to_read);
	if (ret != bytes_to_read) {
		dev_err(p->dev, "%s: register 0x%04x failed(%d)\n",
				__func__, message->header.reg_num, ret);
		return -EIO;
	}

	*message_header_ret = bytearr_to_u32((u8 *)read_buf_ptr);

	switch (message->header.sub_mode) {
	case MSG_FMT_SUBMODE_GET_16BIT:
		*read_val = bytearr_to_u16((u8 *)read_buf_ptr);
		break;
	case MSG_FMT_SUBMODE_GET_PENDING_MESSAGE: /* temporary - need to fix */
	case MSG_FMT_SUBMODE_GET_32BIT:
		*read_val = bytearr_to_u32((u8 *)(read_buf_ptr) + 4);
		break;
	case MSG_FMT_SUBMODE_GET_64BIT:
		*read_val =  bytearr_to_u64((u8 *)(read_buf_ptr) + 4);
		break;
	}
	if (p->dump_traffic > DBMDX_DBG_DUMP_LEVEL_NO_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;
		int i;

		off += snprintf(buf, DBMDX_MAX_DUMP_LENGTH,
			"Reg: 0x%04x Value: 0x%08llx, (MSG Header: 0x%08x), ",
						message->header.reg_num,
						*read_val,
						*message_header_ret);

		if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP) {
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
								"Recv <==[");

			for (i = 0; i < bytes_to_read; i++) {
				if (off >= DBMDX_MAX_DUMP_LENGTH)
					break;
				off += snprintf(buf + off,
						DBMDX_MAX_DUMP_LENGTH - off,
						"%02x ", read_buf[i]);
			}
			if (off < DBMDX_MAX_DUMP_LENGTH)
				off += snprintf(buf + off,
						DBMDX_MAX_DUMP_LENGTH - off,
						"]");
		}

		dev_info(p->dev, "%s: %s\n", __func__, buf);
	}

	return 0;
}


static int dbmdx_read_register(struct dbmdx_private *p,
			struct fw_message *message,
			u64	*read_val)
{
	u32	message_header_ret;
	return dbmdx_read_register_ext(p, message,
					read_val, &message_header_ret);
}

static int dbmdx_write_register(struct dbmdx_private *p,
				struct fw_message *fw_msg)
{

	u32	cmd[DBMDX_WRITE_VAR_LEN_MAX_SIZE + 1]; /*word0 excluded*/
	int	ret;
	u32	cmd_size = 0;
	u32	cmd_size_padded;
	u8	*cmd_buffer = (u8 *)cmd;

	if (!dbmdx_spi_access(p))
		return -EPERM;

	/* Set Priority */
	cmd[0] = (fw_msg->header.priority << MSG_PROC_IDLE_BITN);

	/* Set Message Type */
	cmd[0] |= (fw_msg->header.m_type << MSG_TYPE_BITN);

	/* Set message ID */
	cmd[0] |= (fw_msg->header.id << MSG_ID_BITN);

	/* Set Register number */
	cmd[0] |= (fw_msg->header.reg_num << MSG_REGNUM_BITN);

	switch (fw_msg->header.m_type) {
	case MSG_TYPE__WRITE16:
		/* Fall through */
	case MSG_TYPE__WRITE16_ACK:
		cmd_size = sizeof(u32);
		cmd[0] |= fw_msg->header.reg_value;
		break;
	case MSG_TYPE__WRITE32:
		/* Fall through */
	case MSG_TYPE__WRITE32_ACK:
		/* Write 32/64 bit value to a register, no ack.  */
		cmd[0] |= fw_msg->header.sub_mode;

		/* according to sub_mode */
		switch (fw_msg->header.sub_mode) {
		case MSG_FMT_SUBMODE_GET_32BIT:
			cmd[1] = fw_msg->reg_info[0];
			cmd_size = sizeof(u32)*2;
			break;
		case MSG_FMT_SUBMODE_GET_64BIT:
			cmd[1] = fw_msg->reg_info[0];
			cmd[2] = fw_msg->reg_info[1];
			cmd_size = sizeof(u32)*3;
			break;
		default:
			dev_err(p->dev,
				"%s: Illegal Submode:%d for Msg Type: %d\n",
				__func__,
				fw_msg->header.sub_mode,
				fw_msg->header.m_type);
			return -EINVAL;
		}
		break;
	case MSG_TYPE__SEND_VAR_LAN_MSG:
		/* Fall through */
	case MSG_TYPE__SEND_VAR_LAN_MSG_ACK:

		/* word0 + word1 + message length in bytes */
		cmd_size = (sizeof(u32)*2) + fw_msg->var_msg_len_bytes;
		cmd[1] = cmd_size;

		cmd_buffer = (u8 *)(p->fw_cmd_buffer);

		/* padding to word size*/
		cmd_size_padded = cmd_size & 0xFFFFFFFC;
		if (cmd_size_padded != cmd_size) {
			cmd_size_padded += 4;
			memset(cmd_buffer , 0, cmd_size_padded);
			cmd_size = cmd_size_padded;
		}

		if (cmd_size_padded > DBMDX_MAX_FW_CMD_LENGTH) {
			dev_err(p->dev,
				"%s: Var Cmd exceeds max Var Cmd len:%d>%d\n",
				__func__,
				DBMDX_MAX_FW_CMD_LENGTH,
				cmd_size_padded);
			return -EINVAL;
		}

		/* Copy command header */
		memcpy(cmd_buffer, cmd, sizeof(u32)*2);
		memcpy(cmd_buffer + sizeof(u32)*2,
			fw_msg->var_msg, fw_msg->var_msg_len_bytes);
		break;
	default:
		 dev_err(p->dev,
			 "%s: Illegal Message Type: %d\n",
			 __func__, fw_msg->header.m_type);
		 return -EINVAL;
	 }

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;
		int i;

		off += snprintf(buf, DBMDX_MAX_DUMP_LENGTH, "Send ==> [");

		for (i = 0; i < cmd_size; i++) {
			if (off >= DBMDX_MAX_DUMP_LENGTH)
				break;
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
					"%02x ", ((u8 *)cmd_buffer)[i]);
		}
		if (off < DBMDX_MAX_DUMP_LENGTH)
			off += snprintf(buf + off, DBMDX_MAX_DUMP_LENGTH - off,
					"] (cmd_size: %d)",
					cmd_size);

		dev_info(p->dev, "%s: %s\n", __func__, buf);
	}

	ret = p->chip->write(p, (u8 *)cmd_buffer, cmd_size);

	if (ret != cmd_size) {
		dev_err(p->dev,
		 "%s: Error sending FW command: [0x%08x 0x%08x], cmd_size=%u\n",
			__func__, cmd[0], cmd[1], cmd_size);
		return -EIO;
	}

	return 0;
}

static int dbmdx_write_register_ack(struct dbmdx_private *p,
			struct fw_message	*fw_msg,
			u32	*ack_val)
{
	int ret;

	dbmdx_rx_ack_prepare(p);

	ret = dbmdx_write_register(p, fw_msg);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error writing value to register\n", __func__);
		return ret;
	}
	ret = dbmdx_rx_ack_wait_blocking(p);
	dbmdx_rx_ack_clear(p);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error waiting for RX ACK\n", __func__);
		return -EIO;
	}

	ret = dbmdx_read_cmd_ack(p, &(fw_msg->header), ack_val);

	return ret;
}

/*
 * word0 word1	word2	word3	word4	word5	word6...word N
 *  Message type 3	Payload length	Task ID	Stack depth *
 *  optional parameter	Code-ID & priority	Task name *
 *
 */
int dbmdx_rtos_create_task(struct dbmdx_private *p,
			u32	task_id,
			u32	stack_size,
			u32	optional,
			u32	core_id_priority)
{

	int ret;
	u32 write_reg_arr[4];
	u32 ack_val;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, MSG_ID_RTOS, REG_RTOS__CREATE,
					MSG_TYPE__SEND_VAR_LAN_MSG_ACK);

	message.var_msg = (u8 *)write_reg_arr;
	message.var_msg_len_bytes = sizeof(u32)*4;

	write_reg_arr[0] = task_id;	/*task id*/
	write_reg_arr[1] = stack_size;	/*stack depth*/
	write_reg_arr[2] = optional;	/*optional*/
	write_reg_arr[3] = core_id_priority;	/*priority*/

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_send_message(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__SEND_VAR_LAN_MSG);

	message.var_msg_len_bytes = msg_len;
	message.var_msg = msg;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register(p, &message);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

static int _dbmdx_send_message_ack(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len,
			u32	*ack_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__SEND_VAR_LAN_MSG_ACK);

	message.var_msg_len_bytes = msg_len;
	message.var_msg = msg;

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, ack_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	return ret;

}

int dbmdx_send_message_ack(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len,
			u32	*ack_val)
{
	int ret;

	p->lock(p);
	ret = _dbmdx_send_message_ack(p, id, reg_num, msg, msg_len, ack_val);
	p->unlock(p);

	return ret;
}

int dbmdx_read_register16(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u16	*read_val)
{
	int	ret = 0;
	u64	reg_val;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__GET_MESSAGE);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_16BIT;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register(p, &message, &reg_val);
	*read_val = (u16)reg_val;

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_read_register32(struct dbmdx_private *p,
		u8	id,
		u8	reg_num,
		u32	*read_val)
{
	int ret = 0;
	u64 reg_val;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__GET_MESSAGE);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_32BIT;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register(p, &message, &reg_val);

	*read_val = (u32)reg_val;

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_read_register32_index(struct dbmdx_private *p,
				u8	id,
				u8	reg_num,
				u32	idx,
				u32	*read_val)
{
	int ret = 0;
	u64 reg_val;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__GET_MESSAGE4INDEX);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_32BIT;
	message.reg_info[0] = idx;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register(p, &message, &reg_val);

	*read_val = (u32)reg_val;

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_read_register64(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u64	*read_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__GET_MESSAGE);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register(p, &message, read_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_read_register64_index(struct dbmdx_private *p,
				u8	id,
				u8	reg_num,
				u32	idx,
				u64	*read_val)
{
	int ret = 0;
	u64 reg_val;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, id, reg_num, MSG_TYPE__GET_MESSAGE4INDEX);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;
	message.reg_info[0] = idx;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register(p, &message, &reg_val);

	*read_val = reg_val;

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

static int _dbmdx_write_register16(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u16	reg_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE16);
	message.header.reg_value = reg_val;

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register(p, &message);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	return ret;

}

int dbmdx_write_register16(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u16	reg_val)
{
	int ret = 0;

	p->lock(p);
	ret = _dbmdx_write_register16(p, priority, id, reg_num, reg_val);
	p->unlock(p);

	return ret;
}

int dbmdx_write_register16_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u16	reg_val,
				u32	*ret_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE16_ACK);
	message.header.reg_value = reg_val;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, ret_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

int dbmdx_write_register32(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u32	reg_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE32);
	message.header.sub_mode = MSG_FMT_SUBMODE_GET_32BIT;
	message.reg_info[0] = reg_val;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}


	ret = dbmdx_write_register(p, &message);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	p->unlock(p);
	return ret;

}

static int _dbmdx_write_register32_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val,
				u32	*ret_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE32_ACK);
	message.header.sub_mode = MSG_FMT_SUBMODE_GET_32BIT;
	message.reg_info[0] = reg_val;

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, ret_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	return ret;

}

int dbmdx_write_register32_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val,
				u32	*ret_val)
{
	int ret = 0;

	p->lock(p);
	ret = _dbmdx_write_register32_ack(p, priority, id,
					 reg_num, reg_val, ret_val);
	p->unlock(p);

	return ret;
}

int dbmdx_write_register64(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE32);
	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;
	message.reg_info[0] = reg_val_1;
	message.reg_info[1] = reg_val_2;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register(p, &message);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}
out:
	p->unlock(p);
	return ret;
}

static int _dbmdx_write_register64_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	*ret_val)
{
	int ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num, MSG_TYPE__WRITE32_ACK);
	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;
	message.reg_info[0] = reg_val_1;
	message.reg_info[1] = reg_val_2;

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, ret_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}

out:
	return ret;
}

int dbmdx_write_register64_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	*ret_val)
{
	int ret = 0;

	p->lock(p);
	ret = _dbmdx_write_register64_ack(p, priority, id, reg_num,
					  reg_val_1, reg_val_2, ret_val);
	p->unlock(p);

	return ret;
}

int dbmdx_write_register96(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	reg_val_3)
{
	int ret = 0;
	u32 write_reg_arr[3];
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num,
					MSG_TYPE__SEND_VAR_LAN_MSG);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;
	write_reg_arr[0] = reg_val_1;
	write_reg_arr[1] = reg_val_2;
	write_reg_arr[2] = reg_val_3;
	message.var_msg = (u8 *)write_reg_arr;
	message.var_msg_len_bytes = sizeof(u32)*3;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register(p, &message);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}
out:
	p->unlock(p);
	return 0;
}

int dbmdx_write_register96_ack(struct dbmdx_private *p,
				u8	priority,
				u8	id,
				u8	reg_num,
				u32	reg_val_1,
				u32	reg_val_2,
				u32	reg_val_3,
				u32	*ack_val)
{
	int ret = 0;
	u32 write_reg_arr[3];
	struct fw_message message;

	MESSAGE_HEADER_INIT(priority, id, reg_num,
					MSG_TYPE__SEND_VAR_LAN_MSG_ACK);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_64BIT;
	write_reg_arr[0] = reg_val_1;
	write_reg_arr[1] = reg_val_2;
	write_reg_arr[2] = reg_val_3;
	message.var_msg = (u8 *)write_reg_arr;
	message.var_msg_len_bytes = sizeof(u32)*3;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_write_register_ack(p, &message, ack_val);

	if (ret < 0) {
		dev_err(p->dev, "%s: Failed (ret=%d)\n", __func__, ret);
		goto out;
	}
out:
	p->unlock(p);
	return 0;
}

static int _dbmdx_read_data(struct dbmdx_private *p,
			u8 *buf, u32 bytes_to_read)
{
	int ret;

	ret = p->chip->read(p, buf, bytes_to_read);

	if (ret != bytes_to_read) {
		dev_err(p->dev,
			"%s: read data timed out after %d bytes\n",
				__func__, ret);
		return -EIO;
	}

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_TRAFFIC_DUMP) {
		char dump_buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;
		int i;

		off += snprintf(buf, DBMDX_MAX_DUMP_LENGTH, "Recv <==[");

		for (i = 0; i < bytes_to_read; i++) {
			if (off >= DBMDX_MAX_DUMP_LENGTH)
				break;
			off += snprintf(dump_buf + off,
					DBMDX_MAX_DUMP_LENGTH - off,
					"%02x ", dump_buf[i]);
		}
		if (off < DBMDX_MAX_DUMP_LENGTH)
			off += snprintf(dump_buf + off,
					DBMDX_MAX_DUMP_LENGTH - off,
					"]");

		 dev_info(p->dev, "%s: %s\n", __func__, dump_buf);
	}

	return 0;
}

/*
 * Message Type == MSG_TYPE__GET_MESSAGE(0)
 * Sub Mode == MSG_FMT_SUBMODE_GET_PENDING_MESSAGE(1)
 *
 * FW writes a pending message.
 * The return message is zero in case there is no pending message
 * Message type -0, sub mode 3 for 32-bit status
 *
 * Message type - 2 for a variable length message. In this
 * sub-mode, the register number field is not applicable.
 */
int dbmdx_get_pending_message(struct dbmdx_private *p,
				bool *message_is_pending,
				struct fw_message *pending_message)
{
	int	ret = -1;
	u64	message_ret = 0;
	u32	message_header_ret = 0;
	struct fw_message message;

	MESSAGE_HEADER_INIT(0, MSG_ID_FW, 0, MSG_TYPE__GET_MESSAGE);

	message.header.sub_mode = MSG_FMT_SUBMODE_GET_PENDING_MESSAGE;

	p->lock(p);

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, &message, buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: SendMessage: %s\n", __func__, buf);
	}

	ret = dbmdx_read_register_ext(p, &message, &message_ret,
						&message_header_ret);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error reading register\n", __func__);
	} else if (!message_header_ret) {
		/* no pending message*/
		dev_err(p->dev, "%s: No pending messages\n", __func__);
		*message_is_pending = false;
		ret = 0;
		goto out;
	}

	*message_is_pending = true;
	memset(&message, 0, sizeof(message));

	pending_message->header.priority = 0;
	pending_message->header.m_type =
			((message_header_ret & MSG_TYPE_MASK) >> MSG_TYPE_BITN);
	pending_message->header.id =
			((message_header_ret & MSG_ID_MASK) >> MSG_ID_BITN);
	pending_message->header.reg_num =
			((message_header_ret & MSG_REGNUM_MASK) >>
							MSG_REGNUM_BITN);
	pending_message->header.sub_mode =
			((message_header_ret & MSG_SUBMODE_MASK));

	switch (pending_message->header.m_type) {
	case MSG_TYPE__WRITE32:
		if (pending_message->header.sub_mode !=
					MSG_FMT_SUBMODE_GET_32BIT) {
			/* error sub mode must be GET_32BIT*/
			dev_err(p->dev,
				 "%s: MSG_FMT_SUBMODE_GET_32BIT MISSING\n",
				 __func__);
			ret = -EINVAL;
		} else {
			pending_message->reg_info[0] = message_ret;
			ret = 0;
		}
		break;
	case MSG_TYPE__SEND_VAR_LAN_MSG:
		pending_message->var_msg_len_bytes = message_ret - 8;
		pending_message->var_msg =
				kmalloc(pending_message->var_msg_len_bytes + 8,
					GFP_KERNEL);
		if (!pending_message->var_msg) {
			dev_err(p->dev,	"%s: Error allocating memory\n",
				__func__);
			ret = -ENOMEM;
			break;
		}
		ret = _dbmdx_read_data(p, pending_message->var_msg,
					pending_message->var_msg_len_bytes);
		if (ret < 0)
			dev_err(p->dev,
				 "%s: Error reading VAR_LAN msg payload\n",
				 __func__);
		break;
	default:
		dev_err(p->dev,	"%s: Unknown pending message type (%d)\n",
			 __func__, (int)(pending_message->header.m_type));
		ret = -EINVAL;
		break;
	}

	if (p->dump_traffic >= DBMDX_DBG_DUMP_LEVEL_MSG_DUMP) {
		char buf[DBMDX_MAX_DUMP_LENGTH];
		int off = 0;

		off = dump_fw_message(p, pending_message,
					buf, DBMDX_MAX_DUMP_LENGTH);

		dev_info(p->dev, "%s: Pending Msg: %s\n", __func__, buf);
	}

out:
	p->unlock(p);
	return ret;
}

void dbmdx_free_fw_message(struct dbmdx_private *p,
				struct fw_message *message)
{
	if (message && message->var_msg &&
		((message->header.m_type ==  MSG_TYPE__SEND_VAR_LAN_MSG) ||
		(message->header.m_type ==  MSG_TYPE__SEND_VAR_LAN_MSG_ACK)) &&
		(message->var_msg != p->fw_cmd_buffer))
		kfree(message->var_msg);
}

/* Buffer should be word padded */
int dbmdx_send_buf_to_mb(struct dbmdx_private *p,
			char	*buf,
			size_t buf_size,
			u32 mailbox_id)
{
	int ret = 0;
	 /* 2 words for cmd + word for msg_id */
	int chunk_size = MB_DATA_CHUNK - 12;
	int bytes_to_read;
	size_t cur_idx = 0;
	char *chunk_buf;
	u32 ack_val;

	if (buf_size % 4) {
		dev_err(p->dev, "%s: Buffer is not word padded\n", __func__);
		return -EINVAL;
	}

	chunk_buf = vmalloc(MB_DATA_CHUNK);
	if (!chunk_buf) {
		dev_err(p->dev,	"%s: Cannot allocate memory for chunk buffer\n",
			__func__);
		return -ENOMEM;
	}

	memcpy(chunk_buf, &mailbox_id, sizeof(u32));

	bytes_to_read = chunk_size;

	p->lock(p);
	while (1) {

		if (buf_size - cur_idx <  chunk_size)
			bytes_to_read = buf_size - cur_idx;

		if (!bytes_to_read)
			break;

		memcpy(chunk_buf + 4, buf + cur_idx, bytes_to_read);

		ret = _dbmdx_send_message_ack(p, MSG_ID_MAILBOX,
						REG_MB__APPEND_DATA,
						chunk_buf, bytes_to_read + 4,
						&ack_val);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Failed to send data chunk\n",
				__func__);
			ret = -EIO;
			break;
		}
		cur_idx += bytes_to_read;

	}
	p->unlock(p);

	vfree(chunk_buf);
	return ret;
}

static int _dbmdx_read_buf(struct dbmdx_private *p,
			  u8	**buf,
			  u32	buf_len,
			  u32	size)
{
	int ret, is_alloc = 0;
	u32 sz, real_sz;

	if (!buf)
		return -EINVAL;

	if ((signed)size < 0)
		return -EPERM;

	size -= DBMDX_PROT_HEADER_SIZE;
	if ((signed)size < 0)
		return -ENODATA;

	if (!size)
		return 0;

	real_sz = size;
	size = ALIGN(size, sizeof(u32));

	if (!*buf) {
		buf_len = size;
		*buf = kzalloc(buf_len, GFP_KERNEL);
		if (!*buf)
			return -ENOMEM;
		is_alloc = 1;
	}

	sz = ALIGN(min_t(u32, size, buf_len), sizeof(u32));
	ret = _dbmdx_read_data(p, *buf, sz);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error reading data\n", __func__);
		goto exit;
	}

exit:
	if (ret < 0 && is_alloc) {
		kfree(*buf);
		*buf = NULL;
	}

	return ret < 0 ? ret : real_sz;
}

int dbmdx_read_buf_reg32(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u32	reg_val_1,
			u8	**buf,
			u32	buf_len)
{
	u32 ack_val;
	int ret;

	p->lock(p);

	ret = _dbmdx_write_register32_ack(p, priority, id, reg_num,
					  reg_val_1, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting register 0x%02x\n",
			__func__, reg_num);
		goto exit;
	}

	ret = _dbmdx_read_buf(p, buf, buf_len, ack_val);

exit:
	p->unlock(p);

	return ret;
}

int dbmdx_read_buf_reg64(struct dbmdx_private *p,
			u8	priority,
			u8	id,
			u8	reg_num,
			u32	reg_val_1,
			u32	reg_val_2,
			u8	**buf,
			u32	buf_len)
{
	u32 ack_val;
	int ret;

	p->lock(p);

	ret = _dbmdx_write_register64_ack(p, priority, id, reg_num,
					  reg_val_1, reg_val_2, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting register 0x%02x\n",
			__func__, reg_num);
		goto exit;
	}

	ret = _dbmdx_read_buf(p, buf, buf_len, ack_val);

exit:
	p->unlock(p);

	return ret;
}

int dbmdx_read_buf_msg(struct dbmdx_private *p,
			u8	id,
			u8	reg_num,
			u8	*msg,
			u32	msg_len,
			u8	**buf,
			u32	buf_len)
{
	u32 ack_val;
	int ret;

	p->lock(p);

	ret = _dbmdx_send_message_ack(p, id, reg_num, msg, msg_len, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error setting register 0x%02x\n",
			__func__, reg_num);
		goto exit;
	}

	ret = _dbmdx_read_buf(p, buf, buf_len, ack_val);

exit:
	p->unlock(p);

	return ret;
}
