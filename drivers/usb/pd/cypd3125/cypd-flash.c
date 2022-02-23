/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#include <linux/mutex.h>

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>


#include <asm/unaligned.h>

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/types.h>

#include "cypd-common.h"
#include "cypd-flash.h"
#include "cypd-core.h"

struct cypd_data *cypd_dev_data;

static int CyUsGetFwInfoFromDev(u8 *mode_p, u16 *flash_row_size)
{
	CY_RET_STAT stat;
	u8 buf[64];

	stat = ccg_read(cypd_dev_data, CCG_REG_DEVICE_MODE, buf, sizeof(buf));
	if (stat == CY_SUCCESS_RESP) {
		CYPD_DEBUG(" respone buf[0]:0x%x\n", buf[0]);
		if ((buf[0] & 0x03) < 3)
			*mode_p = buf[0] & 0x03;
		else
			return CY_ERROR_INVALID_FW_MODE;

		if (buf[0] & 0x70)
			*flash_row_size = 256;
		else
			*flash_row_size = 128;

		return CY_SUCCESS_RESP;
	} else
		return stat;
}

static int CyGetSiliconIDFromCCGxDev(u16 *silicon_id)
{
	CY_RET_STAT stat;
	u8 buf[64];
	u16 len = 2;

	stat = ccg_read(cypd_dev_data, CCG_REG_SIID_LSB, buf, len);

	if (stat == CY_SUCCESS_RESP) {
		*silicon_id = (buf[1] << 8 | buf[0]);
		return CY_SUCCESS_RESP;
	}
	return stat;
}

static int CyGetSiliconIDFromFW(u8 fw_no, u16 *silicon_id, u8 *app_ver)
{
	CY_RET_STAT stat = CY_SUCCESS_RESP;

	const struct firmware *fw = NULL;
	struct device *dev = cypd_dev_data->dev;
	int fw_info_start = 488;
	int fw_info_len = 24;
	u8 wr_buf[32] = {0};

	if (request_firmware(&fw, ccg_fw_names[fw_no], dev) != 0) {
		CYPD_ERROR("Failed to open cyacd file %s\n",
			ccg_fw_names[fw_no]);
		return false;
	}

	/********************************************************************
	 *The header of this file has the following format:
	 *[4 bytes Silicon ID] [1 byte Silicon rev] [1 byte checksum type]
	 *
	 *   1D0411AD0000\r\n Total length: 12+2 = 14
	 *
	 *The flash lines have the following format:
	 *[1 byte array ID] [2 bytes row number]
	 *[2 bytes data length] [N bytes of data]
	 *[1 byte checksum]
	 *******************************************************************/

	/*****************************************************************
	 * CCG firmware image (.cyacd) file line format
	 *
	 * :00rrrrllll[dd....]cc/r/n
	 *
	 * :00  header
	 * rrrr is row number to flash				(4 char)
	 * llll is data len to flash				(4 char)
	 * dd   is a data field represents one byte of data	(256 char)
	 * cc   is checksum					(2 char)
	 * \r\n newline
	 *
	 * Total length: 3 + 4 + 4 + 256 + 2 + 2 = 271
	 *
	 *****************************************************************/

	/*Fw info start 14 + 271 + 204 = 489*/
	/*FW info: da 7 1 33 62 6e 2 0 ad 11 4 1d */

	/*get silicon ID*/
	if (hex2bin(wr_buf, fw->data+fw_info_start, fw_info_len)) {
		stat = CY_ERROR_INVALID_FW;
		goto out_release_firmware;
	}

	*app_ver = wr_buf[7];
	*silicon_id = (wr_buf[10] | wr_buf[11] << 8);


out_release_firmware:
	release_firmware(fw);

	return stat;
}

CY_RET_STAT CyUsReadResponse(u8 *resp_p)
{
	CY_RET_STAT stat;
	u8 buf[64];
	u16 len = 1;

	len = 1;
	if (ccg_read(cypd_dev_data, CCG_REG_INTR_STAT,
		buf, len) != CY_SUCCESS_RESP)
		return CY_ERROR_I2C_READ_DATA_FAILED;

	if ((buf[0] & 0x01) != 0) {
		len = 2;
		if (ccg_read(cypd_dev_data, CCG_REG_RESP_TYPE,
			buf, len) != CY_SUCCESS_RESP)
			return CY_ERROR_I2C_READ_RESPONSE_FAILED;

		if (buf[1] != 0) {
			CYPD_ERROR("Invalid response length %d\n", buf[1]);
			return CY_ERROR_INVALID_RESP_REG_LEN;
		}

		*resp_p = buf[0];

		switch (buf[0]) {
		case 0:
				CYPD_DEBUG("No response\n");
			break;
		case 2:
				CYPD_DEBUG("Success response\n");
			break;
		case 3:
				CYPD_DEBUG("Flash data available\n");
			break;
		case 5:
				CYPD_DEBUG("Invalid command\n");
			break;
		case 7:
				CYPD_DEBUG("Flash Op failed\n");
			break;
		case 8:
				CYPD_DEBUG("Bad firmware\n");
			break;
		case 9:
				CYPD_DEBUG("Bad arguments\n");
			break;
		case 10:
				CYPD_DEBUG("Not supported\n");
			break;
		case 128:
				CYPD_DEBUG("Reset complete event\n");
			break;
		case 129:
				CYPD_DEBUG("Message queue overflow\n");
			return CY_ERROR_MESSAGE_QUEUE_OVERFLOW;
		case 1:
		case 4:
		case 6:
		default:
				CYPD_ERROR("cypd Invalid response\n");
			return CY_ERROR_INVALID_RESPONSE;
		}

		buf[0] = 0x01;
		len = 1;
		stat = ccg_write(cypd_dev_data, CCG_REG_INTR_STAT, buf, len);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR("Clearing interrupt failed with status\n");
			return CY_ERROR_INTR_REG_CLEAR_FAILED;
		}
	}

	return CY_SUCCESS_RESP;
}

CY_RET_STAT CyUsDeviceReset(u16 delay)
{
	u8 buf[64];
	u16 len = 0;

	buf[0] = CCG4_RESET_SIG;
	buf[1] = CCG4_RESET_DEV_CMD;
	len = 2;

	CYPD_FUNC_ENTER();


	if (ccg_write(cypd_dev_data, CCG_REG_RESET_RQT,
		buf, len) != CY_SUCCESS_RESP) {
		CYPD_ERROR("Failed to initiate device reset\n");
		return CY_ERROR_DEVICE_RESET_FAILED;
	}
	msleep(delay);

	if (CyUsReadResponse(buf) != CY_SUCCESS_RESP) {
		CYPD_ERROR("cypd Error: Response read failed\n");
		return CY_ERROR_DEVICE_RESET_FAILED;//device reset failed
	}

	CYPD_FUNC_EXIT();
	return CY_SUCCESS_RESP;

}

CY_RET_STAT CyUsPDPortDisable(void)
{
	u8 buf[4];
	u16 len = 0;

	CYPD_FUNC_ENTER();

	buf[0] = 0;
	len = 1;
	if (ccg_write(cypd_dev_data, CCG_REG_PDPORT_EN,
		buf, len) != CY_SUCCESS_RESP) {
		CYPD_ERROR("cypd Error: Failed to disable PD ports\n");
		return CY_ERROR_PD_PORT_DISABLE_FAILED;	// Port disable failed
	}
	msleep(1500);
	return CY_SUCCESS_RESP;
}

CY_RET_STAT CyUsPDPortStatus(void)
{
	CY_RET_STAT stat;
	u8 buf[64];

	stat = ccg_read(cypd_dev_data, CCG_REG_PDPORT_EN, buf, sizeof(buf));
	if (stat == CY_SUCCESS_RESP)
		CYPD_DEBUG("PD Port Status buf[0]:0x%x\n", buf[0]);
	else
		CYPD_ERROR("Failed to get PD ports status\n");
	return stat;
}


CY_RET_STAT CyUSJumpToBoot(void)
{
	CY_RET_STAT stat;
	u8 buf[64] = {0};
	u16 flash_row_size, len = 0;
	u8 cur_mode = 0;
	u8 prev_mode = 0;

	stat = CyUsGetFwInfoFromDev(&cur_mode, &flash_row_size);
	if (stat != CY_SUCCESS_RESP)
		return stat;

	CYPD_DEBUG("cur_mode:%d\n", cur_mode);

	if (cur_mode == 0)
		return CY_SUCCESS_RESP;

#ifdef CYPD_DEBUG_EN
	CyUsPDPortStatus();
#endif

	CYPD_DEBUG("Disabling PD ports before doing device reset\n");

	stat = CyUsPDPortDisable();
	if (stat != CY_SUCCESS_RESP)
		return stat;

	stat = CyUsReadResponse(buf);
	if (stat != CY_SUCCESS_RESP)
		return stat;

#ifdef CYPD_DEBUG_EN
	CyUsPDPortStatus();
#endif


	CYPD_DEBUG("jump to boot!\n");

	buf[0] = CCG4_JUMP_TO_BOOT_SIG;
	len = 1;

	if (ccg_write(cypd_dev_data, CCG_REG_JUMP_TO_BOOT,
		buf, len) != CY_SUCCESS_RESP)
		return CY_ERROR_I2C_WRITE_DATA_FAILED;
	msleep(100);

	stat = CyUsReadResponse(buf);
	if (stat != CY_SUCCESS_RESP)
		return stat;

	if (buf[0] != CCG4_HPI_RESET_COMPLETE) {
		CYPD_ERROR("Bad response to enter flashing mode command\n");
		if (buf[0] == CCG4_JUMP_TO_BOOT_SIG)
			return CY_ERROR_JUMP_TO_BOOT_FAILED;
	}

	prev_mode = cur_mode;

	stat = CyUsGetFwInfoFromDev(&cur_mode, &flash_row_size);
	if (stat != CY_SUCCESS_RESP)
		return stat;
	CYPD_DEBUG("prev_mode:%d cur_mode:%d\n", prev_mode, cur_mode);
	CYPD_ERROR("prev_mode:%d cur_mode:%d\n", prev_mode, cur_mode);
	if (cur_mode != 0)
		return CY_ERROR_JUMP_TO_BOOT_FAILED;

	return CY_SUCCESS_RESP;

}

CY_RET_STAT CyUsEnterFlashingMode(void)
{
	u8 buf[64];
	u16 len = 0;
	CY_RET_STAT stat;

	CYPD_FUNC_ENTER();

	buf[0] = CCG4_FLASH_ENTER_SIG;
	len = 1;

	if (ccg_write(cypd_dev_data, CCG_REG_FLASHMODE_ENTER,
		buf, len) != CY_SUCCESS_RESP)
		return CY_ERROR_I2C_WRITE_DATA_FAILED;

	stat = CyUsReadResponse(buf);
	if (stat != CY_SUCCESS_RESP) {

		CYPD_ERROR(" CyUsReadResponse failed!!\n");
		return stat;
	}

	CYPD_DEBUG(" Response buf[0]:0x%x\n",  buf[0]);


	if ((buf[0] != CCG4_HPI_SUCCESS_RESPONSE) && (buf[0] !=
		CCG4_HPI_RESET_COMPLETE)) {
		CYPD_ERROR("Bad response to enter flashing mode command\n");
		return CY_ERROR_ENTER_FLASH_MODE_FAILED;
	}
	return CY_SUCCESS_RESP;
}

CY_RET_STAT CyUsWriteFlashRow(u16 row_num, u8 *data, u16 flash_row_size)
{
	struct i2c_client *client = cypd_dev_data->client;
	u8 cmdbuf[4];
	u16 len = 0;
	u8 buf[CCG3_ROW_SIZE + 2];
	CY_RET_STAT stat;
	int ret;

	/* Copy the data into the flash read/write memory. */
	put_unaligned_le16(REG_FLASH_RW_MEM, buf);
	memcpy(buf + 2, data, CCG3_ROW_SIZE);

	mutex_lock(&cypd_dev_data->lock);

	/* First write the data to the flash write memory. */
	ret = i2c_master_send(client, buf, CCG3_ROW_SIZE + 2);
	if (ret != CCG3_ROW_SIZE + 2) {
		CYPD_ERROR("row:%d write memory failed\n", row_num);
		mutex_unlock(&cypd_dev_data->lock);
		stat = CY_ERROR_I2C_WRITE_DATA_FAILED;
		return stat;
	}

	msleep(20);

	/* Use the FLASH_ROW_READ_WRITE register to trigger */
	/* writing of data to the desired flash row */
	cmdbuf[0] = CCG_FLASH_RW_SIG;
	cmdbuf[1] = CCG_FLASH_WRITE_CMD;
	cmdbuf[2] = (row_num & 0xFF);
	cmdbuf[3] = (row_num >> 8);
	len = 4;

	if (ccg_write(cypd_dev_data, CCG_REG_FLASH_SIG,
		cmdbuf, len) != CY_SUCCESS_RESP)
		return CY_ERROR_I2C_WRITE_DATA_FAILED;

	/* Allow a delay for the flash write to complete. */
	if ((row_num == FW1_METADATA_ROW) || (row_num == FW2_METADATA_ROW))
		msleep(300);
	else
		msleep(60);

	mutex_unlock(&cypd_dev_data->lock);

	stat = CyUsReadResponse(cmdbuf);
	if (stat != CY_SUCCESS_RESP)
		return stat;		// Response read failed

	return CY_SUCCESS_RESP;
}

CY_RET_STAT CyUsReadFlashRow(u16 row_num,  u8 *buffer, u16 flash_row_size)
{
	u8 cmdbuf[4];
	u16 len = 0;
	CY_RET_STAT stat;
	/* Initiate the flash read command. */
	cmdbuf[0] = CCG_FLASH_RW_SIG;
	cmdbuf[1] = CCG_FLASH_READ_CMD;
	cmdbuf[2] = (row_num & 0xFF);
	cmdbuf[3] = (row_num >> 8);
	len = 4;

	if (ccg_write(cypd_dev_data, CCG_REG_FLASH_SIG, cmdbuf,
		len) != CY_SUCCESS_RESP)
		return CY_ERROR_I2C_WRITE_DATA_FAILED;

		/* Allow a delay for the flash read to complete. */
	msleep(300);

	stat = CyUsReadResponse(cmdbuf);
	if (stat != CY_SUCCESS_RESP)
		return stat;		// Response read failed

	if (cmdbuf[0] != CCG4_FLASH_READ_RESPONSE) {
		CYPD_ERROR("Bad response to command cmdbuf[0]:0x%x\n",
		cmdbuf[0]);
		return CY_ERROR_FLASH_ROW_READ_RESP_FAILED;
		// Flash read response return Bad
	}

	stat = ccg_read(cypd_dev_data, CCG_FLASH_RW_MEM,
		buffer, flash_row_size);

	return stat;

}

bool compare_buffers(u8 *buffer1, u8 *buffer2, u16 size)
{
	u16 i = 0;
	/*if (memcmp(buffer1, buffer2, size))
	 *return true;
	 */
	for (i = 0; i < size; i++) {
		if (buffer1[i] != buffer2[i]) {
			CYPD_ERROR("buffer1[%d]:%d , buffer2[%d]:%d ", i,
				buffer1[i], i, buffer2[i]);
			return false;

		}
	}
	return true;
}

static int CyUsEraseMetadataOnDev(enum enum_flash_mode flash_mode,
	u16 flash_row_size)
{
	CY_RET_STAT stat;
	u8 buffer[256] = {0};
	u16 fw1_meta_row = 0x3FF;
	u16 fw2_meta_row = 0x3FE;

	CYPD_FUNC_ENTER();

	memset(buffer, 0, 256);
	if (flash_mode == FW1) {
		stat = CyUsWriteFlashRow(fw1_meta_row, buffer, flash_row_size);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" Erease FW1 Response read failed\n");
			return stat;	//Response Read failed
		}
	}

	if (flash_mode == FW2) {
		stat = CyUsWriteFlashRow(fw2_meta_row, buffer, flash_row_size);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" Erease FW2 Response read failed\n");
			return stat;	//Response Read failed
		}
	}

	if (flash_mode == FW1_FW2) {
		stat = CyUsWriteFlashRow(fw1_meta_row, buffer, flash_row_size);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" Erease FW1 Response read failed\n");
			return stat;	//Response Read failed
		}
		stat = CyUsWriteFlashRow(fw2_meta_row, buffer, flash_row_size);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" Erease FW2 Response read failed\n");
			return stat;	//Response Read failed
		}

	}

	return stat;
}

int CyUsValidateFirmware(u8 fwid)
{
	u8 buf[4];
	u16 len = 0;
	CY_RET_STAT stat;

	CYPD_FUNC_ENTER();
	buf[0] = fwid;
	len = 1;
	if (ccg_write(cypd_dev_data, CCG_REG_VALIDATE_FW, buf, len)
		== CY_SUCCESS_RESP) {
		// Wait for validate to be completed.
		msleep(1000);
		stat = CyUsReadResponse(buf);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" Response read failed\n");
			return stat;	//Response Read failed
		} else if (buf[0] != CCG4_HPI_SUCCESS_RESPONSE) {
			CYPD_ERROR(" Firmware validation failed\n");
			return CY_ERROR_INVALID_FW;
		}
		return CY_SUCCESS_RESP;
	}
	CYPD_ERROR(" Failed to initiate validate firmware command\n");
	return CY_ERROR_I2C_WRITE_DATA_FAILED;
}

static CY_RET_STAT do_flash(struct cypd_data *uc, enum enum_flash_mode mode_no)
{
	CY_RET_STAT stat;
	struct device *dev = uc->dev;
	const struct firmware *fw = NULL;
	const char *p, *s;
	const char *eof;
	int err, row, len, line_sz, line_cnt = 0;
	unsigned long start_time = jiffies;
	u8 *wr_buf;
	u8 *read_buf;

	CYPD_DEBUG(" ccg_fw_names[%d]:%s\n", mode_no,  ccg_fw_names[mode_no]);

	err = request_firmware(&fw, ccg_fw_names[mode_no], dev);
	if (err) {
		dev_err(dev, "request %s failed err=%d\n",
			ccg_fw_names[mode_no], err);
		return CY_ERROR_BIN_FILE_READ_FAILED;
	}

	eof = fw->data + fw->size;

	wr_buf = kzalloc(CCG3_ROW_SIZE + 4, GFP_KERNEL);
	if (!wr_buf) {
		err = -ENOMEM;
		goto release_fw;
	}

	read_buf = kzalloc(CCG3_ROW_SIZE + 4, GFP_KERNEL);
	if (!read_buf) {
		err = -ENOMEM;
		goto release_mem_wr;
	}

	/*****************************************************************
	 * CCG firmware image (.cyacd) file line format
	 *
	 * :00rrrrllll[dd....]cc/r/n
	 *
	 * :00  header
	 * rrrr is row number to flash				(4 char)
	 * llll is data len to flash				(4 char)
	 * dd   is a data field represents one byte of data	(256 char)
	 * cc   is checksum					(2 char)
	 * \r\n newline
	 *
	 * Total length: 3 + 4 + 4 + 256 + 2 + 2 = 271
	 *
	 *****************************************************************/

	p = strnchr(fw->data, fw->size, ':');
	while (p < eof) {
		s = strnchr(p + 1, eof - p - 1, ':');

		if (!s)
			s = eof;

		line_sz = s - p;

		if (line_sz != CYACD_LINE_SIZE) {
			dev_err(dev, "Bad FW format line_sz=%d\n", line_sz);
			err =  -EINVAL;
			goto release_mem_read;
		}

		if (hex2bin(wr_buf, p + 3, CCG3_ROW_SIZE + 5)) {
			err =  -EINVAL;
			goto release_mem_read;
		}

		row = get_unaligned_be16(wr_buf);
		len = get_unaligned_be16(&wr_buf[2]);

		CYPD_DEBUG(" 0x%04X row data\n", row);

		if (len != CCG3_ROW_SIZE) {
			err =  -EINVAL;
			goto release_mem_read;
		}

		err = CyUsWriteFlashRow(row, wr_buf + 4, len);

		if ((row == 1023) || (row == 1022)) {
			stat = CyUsReadFlashRow(row, read_buf,
				CCG3_ROW_SIZE);
			msleep(20);
			if (stat == CY_SUCCESS_RESP) {
				if (!compare_buffers(read_buf, wr_buf + 4,
					CCG3_ROW_SIZE)) {
					CYPD_ERROR(" Flash on row %d\n", row);
				} else{
					CYPD_DEBUG("Flash on row:%d,success\n",
						row);
				}
			} else{
				CYPD_ERROR("row %d failed\n", row);
			}
		}

		if (err)
			goto release_mem_read;

		line_cnt++;
		p = s;
	}

	CYPD_DEBUG("total %d row flashed. time: %dms\n",
		 line_cnt, jiffies_to_msecs(jiffies - start_time));

release_mem_read:
	kfree(read_buf);

release_mem_wr:
	kfree(wr_buf);

release_fw:
	release_firmware(fw);

	if (err)
		return CY_ERROR_FLASH_ROW_WRITE_FAILED;
	return CY_SUCCESS_RESP;
}

static int ccg_fw_update_needed(struct cypd_data *uc,
				enum enum_flash_mode *flash_mode)
{
	struct device *dev = uc->dev;
	int err;
	u8 dev_mode = 0;
	struct version_info version[3];
	u16 silicon_id_dev = 0;
	u16 silicon_id_fw1 = 0;
	u16 silicon_id_fw2 = 0;
	u8 app_ver_fw1 = 0;
	u8 app_ver_fw2 = 0;
	u16 flash_row_size = 0;

	CYPD_FUNC_ENTER();

	CyGetSiliconIDFromCCGxDev(&silicon_id_dev);
	CYPD_DEBUG(" CyGetSiliconIDFromCCGxDev silicon_id_dev 0x%x\n",
		silicon_id_dev);
	CyGetSiliconIDFromFW(FW1, &silicon_id_fw1, &app_ver_fw1);
	CyGetSiliconIDFromFW(FW2, &silicon_id_fw2, &app_ver_fw2);
	CYPD_DEBUG(" silicon_id_fw1 0x%x silicon_id_fw2 0x%x\n",
		silicon_id_fw1, silicon_id_fw2);

	if ((silicon_id_fw1 != silicon_id_dev) ||
		(silicon_id_fw2 != silicon_id_dev)) {
		*flash_mode = FLASH_NOT_NEEDED;

		CYPD_ERROR("flash_mode = SID_MISMATCH");

		return 0;
	}

	CyUsGetFwInfoFromDev(&dev_mode, &flash_row_size);
	CYPD_DEBUG(" mode %d flash_row_size 0x%x\n", dev_mode, flash_row_size);

	if (dev_mode == 0) {
		*flash_mode = FW1_FW2;
		return 0;
	}


	err = ccg_read(uc, CCGX_RAB_READ_ALL_VER, (u8 *)version,
			sizeof(version));
	if (err) {
		dev_err(dev, "cypd read device version failed\n");
		return err;
	}

	CYPD_DEBUG("FW1, build:0x%x patch:0x%x ver:0x%x, app_ver_fw1:0x%x",
		version[1].app.build, version[1].app.patch,
		version[1].app.ver, app_ver_fw1);

	CYPD_DEBUG("FW2, build:0x%x patch:0x%x ver:0x%x, app_ver_fw2:0x%x",
		version[2].app.build, version[2].app.patch,
		version[2].app.ver, app_ver_fw2);

	if (app_ver_fw1 > version[1].app.ver) {
		if (app_ver_fw2 > version[2].app.ver)
			*flash_mode = FW1_FW2;
		else
			*flash_mode = FW1;
	} else{
		if (app_ver_fw2 > version[2].app.ver)
			*flash_mode = FW2;
		else
			*flash_mode = FLASH_NOT_NEEDED;
	}

	return 0;
}

CY_RET_STAT CyUpdateImage(enum enum_flash_mode flash_mode)
{
	CY_RET_STAT stat = CY_SUCCESS_RESP;
	u8  mode;
	u16 flash_row_size;

	CYPD_FUNC_ENTER();

	stat = CyUSJumpToBoot();
	if (stat != CY_SUCCESS_RESP)
		return stat;

	stat = CyUsEnterFlashingMode();
	if (stat != CY_SUCCESS_RESP)
		return stat;

#ifdef CYPD_DEBUG_EN
	stat = CyUsGetFwInfoFromDev(&mode, &flash_row_size);
	CYPD_DEBUG("Current mode %d , flash_row_size: 0x%x\n",
			mode, flash_row_size);
#endif

	stat = CyUsEraseMetadataOnDev(flash_mode, flash_row_size);
	if (stat != CY_SUCCESS_RESP)
		return stat;

	if (flash_mode == FW1_FW2) {

		stat = do_flash(cypd_dev_data, FW1);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" stat %d\n", stat);
			return stat;
		}

		stat = do_flash(cypd_dev_data,
FW2);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR("stat %d\n", stat);
			return stat;
		}

	}

	if (flash_mode == FW1) {

		stat = do_flash(cypd_dev_data, FW1);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR(" stat %d\n", stat);
			return stat;
		}
	}

	if (flash_mode == FW2) {

		stat = do_flash(cypd_dev_data,
FW2);
		if (stat != CY_SUCCESS_RESP) {
			CYPD_ERROR("stat %d\n",  stat);
			return stat;
		}

	}

	if (flash_mode == FW1_FW2) {
		stat = CyUsValidateFirmware(valid_fw1);
		if (stat == CY_SUCCESS_RESP) {
			CYPD_INFO(" FW1 is valid\n");
		} else{
			CYPD_ERROR(" FW1 is not valid\n");
			return stat;
		}
		stat = CyUsValidateFirmware(valid_fw2);
		if (stat == CY_SUCCESS_RESP) {
			CYPD_INFO(" FW2 is valid\n");
		} else{
			CYPD_ERROR(" FW2 is not valid\n");
			return stat;
		}

	}

	if (flash_mode == FW1) {
		stat = CyUsValidateFirmware(valid_fw1);
		if (stat == CY_SUCCESS_RESP) {
			CYPD_INFO(" FW1 is valid\n");
		} else{
			CYPD_ERROR(" FW1 is not valid\n");
			return stat;
		}
	}

	if (flash_mode == FW2) {
		stat = CyUsValidateFirmware(valid_fw2);
		if (stat == CY_SUCCESS_RESP) {
			CYPD_INFO(" FW2 is valid\n");
		} else{
			CYPD_ERROR(" FW2 is not valid\n");
			return stat;
		}
	}


	// Reset CCG3 and see whether it comes up again.

	CYPD_INFO(" Resetting device to start new firmware\n");

	stat = CyUsDeviceReset(1000);
	if (stat != CY_SUCCESS_RESP)
		CYPD_ERROR(" reset failed!\n");
		return stat;


	stat = CyUsGetFwInfoFromDev(&mode, &flash_row_size);
	if (stat != CY_SUCCESS_RESP)
		CYPD_ERROR(" get mode failed!\n");
		return stat;

	CYPD_INFO("currentmode:%d\n", mode);

	return CY_SUCCESS_RESP;
}

static void ccg_update_firmware(struct work_struct *work)
{
	enum enum_flash_mode flash_mode = FLASH_NOT_NEEDED;

	int status;

	cypd_dev_data = container_of(work, struct cypd_data, fwupg_work);

	status = ccg_fw_update_needed(cypd_dev_data, &flash_mode);
	if (status < 0) {
		CYPD_ERROR("ccg_fw_update_needed failed\n");
		return;
	}
	CYPD_INFO("update firmware %d\n", flash_mode);
	if (flash_mode != FLASH_NOT_NEEDED)
		CyUpdateImage(flash_mode);
}

/*****************************************************************************
 *  Name: cypd_fwupg_init
 *  Brief: upgrade function initialization
 *  Input:
 *  Output:
 *  Return: return 0 if success, otherwise return error code
 *****************************************************************************/
int cypd_fwupg_init(struct cypd_data *cypd_data)
{

	INIT_WORK(&cypd_data->fwupg_work, ccg_update_firmware);
	queue_work(cypd_data->cypd_workqueue, &cypd_data->fwupg_work);

	return 0;

}

