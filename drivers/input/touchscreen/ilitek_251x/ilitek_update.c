/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Jijie Wang <jijie_wang@ilitek.com>
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

#include "ilitek_ts.h"
#include "ilitek_common.h"
#include "ilitek_protocol.h"

#include <linux/firmware.h>
#include <linux/vmalloc.h>

static uint16_t UpdateCRC(uint16_t crc, uint8_t newbyte)
{
	char i;			// loop counter
#define CRC_POLY 0x8408		// CRC16-CCITT FCS (X^16+X^12+X^5+1)

	crc = crc ^ newbyte;

	for (i = 0; i < 8; i++) {
		if (crc & 0x01) {
			crc = crc >> 1;
			crc ^= CRC_POLY;
		} else {
			crc = crc >> 1;
		}
	}
	return crc;
}

uint16_t get_dri_crc(uint32_t startAddr,uint32_t endAddr,uint8_t input[])
{
    uint16_t CRC = 0;
    uint32_t i = 0;
    // Process each byte in the page into the running CRC
    for(i = startAddr; i < endAddr; i++)
    {
        CRC = UpdateCRC (CRC, input[i]);
    }
    return CRC;
}
static int32_t check_busy(int32_t delay)
{
	int32_t i;
	uint8_t inbuf[2], outbuf[2];
	for (i = 0; i < 1000; i++) {
		inbuf[0] = ILITEK_TP_CMD_GET_SYSTEM_BUSY;
		if (ilitek_i2c_write_and_read(inbuf, 1, delay, outbuf, 1) < 0) {
			return ILITEK_I2C_TRANSFER_ERR;
		}
		if (outbuf[0] == ILITEK_TP_SYSTEM_READY) {
			return 0;
		}
	}
	tp_log_info("check_busy error\n");
	return -1;
}

static int32_t ilitek_changetoblmode(bool mode)
{
	int32_t i = 0;
	uint8_t outbuf[64];
	if(api_protocol_set_cmd(ILITEK_TP_CMD_READ_MODE, NULL, outbuf) < ILITEK_SUCCESS)
		return ILITEK_I2C_TRANSFER_ERR;
	msleep(30);
	tp_log_info("ilitek ic. mode =%d , it's %s\n", outbuf[0],
		    ((outbuf[0] == 0x5A) ? "AP MODE" : ((outbuf[0] == ILITEK_TP_MODE_BOOTLOADER) ? "BL MODE" : "UNKNOW MODE")));
	if ((outbuf[0] == ILITEK_TP_MODE_APPLICATION && !mode) || (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER && mode)) {
		if (mode) {
			tp_log_info("ilitek change to BL mode ok already BL mode\n");
		} else {
			tp_log_info("ilitek change to AP mode ok already AP mode\n");
		}
	} else {
		for (i = 0; i < 5; i++) {
			if((ilitek_data->bl_ver[0] == 0x1 && ilitek_data->bl_ver[1] >= 0x8) || (ilitek_data->ptl.ver_major == 0x6)) {
				if(api_protocol_set_cmd(ILITEK_TP_CMD_WRITE_FLASH_ENABLE, NULL, outbuf) < ILITEK_SUCCESS) {
					return ILITEK_I2C_TRANSFER_ERR;
				}
			}
			else {
				if(api_protocol_set_cmd(ILITEK_TP_CMD_WRITE_ENABLE, NULL, outbuf) < ILITEK_SUCCESS) {
					return ILITEK_I2C_TRANSFER_ERR;
				}
			}

			msleep(20);
			if (mode) {
				if(api_protocol_set_cmd(ILITEK_TP_CMD_SET_BLMODE, NULL, outbuf) < ILITEK_SUCCESS) {
					return ILITEK_I2C_TRANSFER_ERR;
				}
			} else {
				if(api_protocol_set_cmd(ILITEK_TP_CMD_SET_APMODE, NULL, outbuf) < ILITEK_SUCCESS) {
					return ILITEK_I2C_TRANSFER_ERR;
				}
			}
			msleep(500 + i*100);
			if(api_protocol_set_cmd(ILITEK_TP_CMD_READ_MODE, NULL, outbuf) < ILITEK_SUCCESS)
				return ILITEK_I2C_TRANSFER_ERR;
			msleep(30);
			tp_log_info("ilitek ic. mode =%d , it's  %s\n", outbuf[0],
				    ((outbuf[0] == ILITEK_TP_MODE_APPLICATION) ? "AP MODE" : ((outbuf[0] == ILITEK_TP_MODE_BOOTLOADER) ? "BL MODE" : "UNKNOW MODE")));
			if ((outbuf[0] == ILITEK_TP_MODE_APPLICATION && !mode) || (outbuf[0] == ILITEK_TP_MODE_BOOTLOADER && mode)) {
				if (mode) {
					tp_log_info("ilitek change to BL mode ok\n");
				} else {
					tp_log_info("ilitek change to AP mode ok\n");
				}
				break;
			}
		}
	}
	if (i >= 5) {
		if (mode) {
			tp_log_err("change to bl mode err, 0x%X\n", outbuf[0]);
			return ILITEK_TP_CHANGETOBL_ERR;
		} else {
			tp_log_err("change to ap mode err, 0x%X\n", outbuf[0]);
			return ILITEK_TP_CHANGETOAP_ERR;
		}
	} else {
		return ILITEK_SUCCESS;
	}
}

static int32_t ilitek_upgrade_BL1_6(uint32_t df_startaddr, uint32_t df_endaddr, uint32_t df_checksum,
			     uint32_t ap_startaddr, uint32_t ap_endaddr, uint32_t ap_checksum, uint8_t *CTPM_FW)
{
	int32_t ret = 0, upgrade_status = 0, i = 0, j = 0, k = 0, tmp_ap_end_addr = 0;
	uint8_t buf[64] = { 0 };
	if (ilitek_data->bl_ver[0] >= 1 && ilitek_data->bl_ver[1] >= 4) {
		buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;
		buf[1] = 0x5A;
		buf[2] = 0xA5;
		buf[3] = 0x81;
		if (!ilitek_data->has_df) {
			tp_log_info("ilitek no df data set df_endaddr 0x1ffff\n");
			df_endaddr = 0x1ffff;
			df_checksum = 0x1000 * 0xff;
			buf[4] = df_endaddr >> 16;
			buf[5] = (df_endaddr >> 8) & 0xFF;
			buf[6] = (df_endaddr) & 0xFF;
			buf[7] = df_checksum >> 16;
			buf[8] = (df_checksum >> 8) & 0xFF;
			buf[9] = df_checksum & 0xFF;
		} else {
			buf[4] = df_endaddr >> 16;
			buf[5] = (df_endaddr >> 8) & 0xFF;
			buf[6] = (df_endaddr) & 0xFF;
			buf[7] = df_checksum >> 16;
			buf[8] = (df_checksum >> 8) & 0xFF;
			buf[9] = df_checksum & 0xFF;
		}
		tp_log_info("ilitek df_startaddr=0x%X, df_endaddr=0x%X, df_checksum=0x%X\n", df_startaddr, df_endaddr, df_checksum);
		ret = ilitek_i2c_write(buf, 10);
		if (ret < 0) {
			tp_log_err("ilitek_i2c_write\n");
		}
		msleep(20);
		j = 0;
		for (i = df_startaddr; i < df_endaddr; i += 32) {
			buf[0] = ILITEK_TP_CMD_WRITE_DATA;
			for (k = 0; k < 32; k++) {
				if (ilitek_data->has_df) {
					if ((i + k) >= df_endaddr) {
						buf[1 + k] = 0x00;
					} else {
						buf[1 + k] = CTPM_FW[i + 32 + k];
					}
				} else {
					buf[1 + k] = 0xff;
				}
			}

			ret = ilitek_i2c_write(buf, 33);
			if (ret < 0) {
				tp_log_err("ilitek_i2c_write_and_read\n");
				return ILITEK_I2C_TRANSFER_ERR;
			}
			j += 1;

#ifdef ILI_UPDATE_BY_CHECK_INT
			for (k = 0; k < 40; k++) {
				if (!(ilitek_poll_int())) {
					break;
				} else {
					mdelay(1);
				}
			}
			if (k >= 40) {
				tp_log_err("upgrade check int fail retry = %d\n", k);
			}
#else
			if ((j % (ilitek_data->page_number)) == 0) {
				mdelay(20);
			}
			mdelay(10);
#endif
			upgrade_status = ((i - df_startaddr) * 100) / (df_endaddr - df_startaddr);
			if (upgrade_status % 10 == 0) {
				tp_log_info("%c ilitek ILITEK: Firmware Upgrade(Data flash), %02d%c.\n", 0x0D, upgrade_status, '%');
			}
		}
		buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;	//0xc4
		buf[1] = 0x5A;
		buf[2] = 0xA5;
		buf[3] = 0x80;
		if (((ap_endaddr + 1) % 32) != 0) {
			tp_log_info("ap_endaddr = 0x%X\n", (int32_t)(ap_endaddr + 32 + 32 - ((ap_endaddr + 1) % 32)));
			buf[4] = (ap_endaddr + 32 + 32 - ((ap_endaddr + 1) % 32)) >> 16;
			buf[5] = ((ap_endaddr + 32 + 32 - ((ap_endaddr + 1) % 32)) >> 8) & 0xFF;
			buf[6] = ((ap_endaddr + 32 + 32 - ((ap_endaddr + 1) % 32))) & 0xFF;
			tp_log_info("ap_checksum = 0x%X\n", (int32_t)(ap_checksum + (32 + 32 - ((ap_endaddr + 1) % 32)) * 0xff));
			buf[7] = (ap_checksum + (32 + 32 - ((ap_endaddr + 1) % 32)) * 0xff) >> 16;
			buf[8] = ((ap_checksum + (32 + 32 - ((ap_endaddr + 1) % 32)) * 0xff) >> 8) & 0xFF;
			buf[9] = (ap_checksum + (32 + 32 - ((ap_endaddr + 1) % 32)) * 0xff) & 0xFF;
		} else {
			tp_log_info("32 0 ap_endaddr  32 = 0x%X\n", (int32_t)(ap_endaddr + 32));
			tp_log_info("ap_endaddr = 0x%X\n", (int32_t)(ap_endaddr + 32));
			buf[4] = (ap_endaddr + 32) >> 16;
			buf[5] = ((ap_endaddr + 32) >> 8) & 0xFF;
			buf[6] = ((ap_endaddr + 32)) & 0xFF;
			tp_log_info("ap_checksum = 0x%X\n", (int32_t)(ap_checksum + 32 * 0xff));
			buf[7] = (ap_checksum + 32 * 0xff) >> 16;
			buf[8] = ((ap_checksum + 32 * 0xff) >> 8) & 0xFF;
			buf[9] = (ap_checksum + 32 * 0xff) & 0xFF;
		}
		ret = ilitek_i2c_write(buf, 10);
		msleep(20);
		tmp_ap_end_addr = ap_endaddr;
		ap_endaddr += 32;
		j = 0;
		for (i = ap_startaddr; i < ap_endaddr; i += 32) {
			buf[0] = ILITEK_TP_CMD_WRITE_DATA;
			for (k = 0; k < 32; k++) {
				if ((i + k) > tmp_ap_end_addr) {
					buf[1 + k] = 0xff;
				} else {
					buf[1 + k] = CTPM_FW[i + k + 32];
				}
			}

			buf[0] = 0xc3;
			ret = ilitek_i2c_write(buf, 33);
			if (ret < 0) {
				tp_log_err("%s, ilitek_i2c_write_and_read\n", __func__);
				return ILITEK_I2C_TRANSFER_ERR;
			}
			j += 1;
#ifdef ILI_UPDATE_BY_CHECK_INT
			for (k = 0; k < 40; k++) {
				if (!(ilitek_poll_int())) {
					break;
				} else {
					mdelay(1);
				}
			}
			if (k >= 40) {
				tp_log_err("upgrade check int fail retry = %d\n", k);
			}
#else
			if ((j % (ilitek_data->page_number)) == 0) {
				mdelay(20);
			}
			mdelay(10);
#endif
			upgrade_status = (i * 100) / ap_endaddr;
			if (upgrade_status % 10 == 0) {
				tp_log_info("%c ilitek ILITEK: Firmware Upgrade(AP), %02d%c.\n", 0x0D, upgrade_status, '%');
			}
		}
	} else {
		tp_log_info("not support this bl version upgrade flow\n");
	}
	return 0;
}

static int32_t ilitek_upgrade_BL1_7(uint32_t df_startaddr, uint32_t df_endaddr, uint32_t ap_startaddr, uint32_t ap_endaddr, uint8_t *CTPM_FW)
{
	int32_t ret = 0, upgrade_status = 0, i = 0, k = 0, CRC_DF = 0, CRC_AP = 0;
	uint8_t buf[64] = { 0 };
	for (i = df_startaddr + 2; i < df_endaddr; i++) {
		CRC_DF = UpdateCRC(CRC_DF, CTPM_FW[i + 32]);
	}
	tp_log_info("CRC_DF = 0x%X\n", CRC_DF);
	for (i = ap_startaddr; i < ap_endaddr - 1; i++) {
		CRC_AP = UpdateCRC(CRC_AP, CTPM_FW[i + 32]);
	}
	tp_log_info("CRC_AP = 0x%x\n", CRC_AP);

	buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;	//0xc4
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x01;
	buf[4] = df_endaddr >> 16;
	buf[5] = (df_endaddr >> 8) & 0xFF;
	buf[6] = (df_endaddr) & 0xFF;
	buf[7] = CRC_DF >> 16;
	buf[8] = (CRC_DF >> 8) & 0xFF;
	buf[9] = CRC_DF & 0xFF;
	ret = ilitek_i2c_write(buf, 10);
	if (ret < 0) {
		tp_log_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	if (((df_endaddr) % 32) != 0) {
		df_endaddr += 32;
	}
	tp_log_info("write data to df mode\n");
	for (i = df_startaddr; i < df_endaddr; i += 32) {
		// we should do delay when the size is equal to 512 bytes
		buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			buf[1 + k] = CTPM_FW[i + k + 32];
		}
		if (ilitek_i2c_write(buf, 33) < 0) {
			tp_log_err("%s, ilitek_i2c_write_and_read\n", __func__);
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		upgrade_status = ((i - df_startaddr) * 100) / (df_endaddr - df_startaddr);
	}

	buf[0] = (uint8_t)0xC7;
	if (ilitek_i2c_write(buf, 1) < 0) {
		tp_log_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	buf[0] = (uint8_t)0xC7;
	ilitek_i2c_write_and_read(buf, 1, 10, buf, 4);
	tp_log_info("upgrade end write c7 read 0x%X, 0x%X, 0x%X, 0x%X\n", buf[0], buf[1], buf[2], buf[3]);
	if (CRC_DF != buf[1] * 256 + buf[0]) {
		tp_log_err("CRC DF compare error\n");
	} else {
		tp_log_info("CRC DF compare Right\n");
	}
	buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_ENABLE;	//0xc4
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x00;
	buf[4] = (ap_endaddr + 1) >> 16;
	buf[5] = ((ap_endaddr + 1) >> 8) & 0xFF;
	buf[6] = ((ap_endaddr + 1)) & 0xFF;
	buf[7] = 0;
	buf[8] = (CRC_AP & 0xFFFF) >> 8;
	buf[9] = CRC_AP & 0xFFFF;
	ret = ilitek_i2c_write(buf, 10);
	if (ret < 0) {
		tp_log_err("ilitek_i2c_write\n");
		return ILITEK_I2C_TRANSFER_ERR;
	}
	check_busy(1);
	if (((ap_endaddr + 1) % 32) != 0) {
		tp_log_info("ap_endaddr += 32\n");
		ap_endaddr += 32;
	}
	tp_log_info("write data to AP mode\n");
	for (i = ap_startaddr; i < ap_endaddr; i += 32) {
		buf[0] = (uint8_t)ILITEK_TP_CMD_WRITE_DATA;
		for (k = 0; k < 32; k++) {
			buf[1 + k] = CTPM_FW[i + k + 32];
		}
		if (ilitek_i2c_write(buf, 33) < 0) {
			tp_log_err("ilitek_i2c_write\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		upgrade_status = ((i - ap_startaddr) * 100) / (ap_endaddr - ap_startaddr);
	}
	for (i = 0; i < 20; i++) {
		buf[0] = (uint8_t)0xC7;
		if (ilitek_i2c_write(buf, 1) < 0) {
			tp_log_err("ilitek_i2c_write\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		check_busy(1);
		buf[0] = (uint8_t)0xC7;
		ret = ilitek_i2c_write_and_read(buf, 1, 10, buf, 4);
		if (ret < 0) {
			tp_log_err("ilitek_i2c_write_and_read 0xc7\n");
			return ILITEK_I2C_TRANSFER_ERR;
		}
		tp_log_info("upgrade end write c7 read 0x%X, 0x%X, 0x%X, 0x%X\n", buf[0], buf[1], buf[2], buf[3]);
		if (CRC_AP != buf[1] * 256 + buf[0]) {
			tp_log_err("CRC compare error retry\n");
		} else {
			tp_log_info("CRC compare Right\n");
			break;
		}
	}
	if (i >= 20) {
		tp_log_err("CRC compare error\n");
		return ILITEK_TP_UPGRADE_FAIL;
	}
	return 0;
}

int program_block_BL1_8(uint8_t *buffer, int block, uint32_t len) {
    int k = 0, ret = ILITEK_SUCCESS, i;
    uint16_t dri_crc = 0, ic_crc = 0;
	uint8_t *buff;

	buff = vmalloc(sizeof(uint8_t) * (len+1));
	if (NULL == buff) {
		tp_log_err("buff NULL\n");
		return -ENOMEM;
	}
	memset(buff, 0xff, len+1);
    dri_crc = get_dri_crc(ilitek_data->blk[block].start, ilitek_data->blk[block].end - 1, buffer);
    ret = api_write_flash_enable_BL1_8(ilitek_data->blk[block].start, ilitek_data->blk[block].end);
    for (i = ilitek_data->blk[block].start; i < ilitek_data->blk[block].end; i += len) //i += update_len - 1)
    {
        buff[0] = (unsigned char)ILITEK_TP_CMD_WRITE_DATA;
        for (k = 0; k < len; k++)
        {
            buff[1 + k] = buffer[i + k];
        }
		ret = ilitek_i2c_write_and_read(buff, len + 1, 1, NULL, 0);
		if (ilitek_check_busy(40, 50, ILITEK_TP_SYSTEM_BUSY) < 0)
		{
			tp_log_err("%s, Write Datas: CheckBusy Failed\n", __func__);
			return ILITEK_FAIL;
		}
        printk("ILITEK: Firmware Upgrade, %02d%c.\n",((i - ilitek_data->blk[block].start + 1) * 100) / (ilitek_data->blk[block].end - ilitek_data->blk[block].start), '%');
    }
    tp_log_info("upgrade firmware, 100%c\n", '%');

    ic_crc = api_get_block_crc(ilitek_data->blk[block].start, ilitek_data->blk[block].end, CRC_GET_FROM_FLASH);
    tp_log_info("Block:%d start:0x%x end:0x%x Real=0x%x,Get=0x%x\n\n", block, ilitek_data->blk[block].start, ilitek_data->blk[block].end, dri_crc, ic_crc);
    if (ic_crc < 0)
    {
        tp_log_err("WriteAPDatas Last: CheckBusy Failed\n");
        return ILITEK_FAIL;
    }
    if (dri_crc != ic_crc)
    {
        tp_log_err("WriteAPData: CheckSum Failed! Real=0x%x,Get=0x%x\n", dri_crc, ic_crc);
        return ILITEK_FAIL;
    }
	return ILITEK_SUCCESS;
}

int program_slave_BL1_8(uint8_t *buffer, int block, uint32_t len) {
    int ret = ILITEK_SUCCESS, i;
    uint16_t dri_crc = 0;
    bool update = false;


    ilitek_data->ic = (struct ilitek_ic_info*)kmalloc(ilitek_data->slave_count, sizeof(struct ilitek_ic_info));
    //check protocol
    if(ilitek_data->ptl.ver < 0x60000) {
        tp_log_info("It is protocol not support\n");
        return ILITEK_FAIL;
    }
    ret = api_protocol_set_testmode(true);

    ret = api_get_slave_mode(ilitek_data->slave_count);

    ret = api_get_ap_crc(ilitek_data->slave_count);

    dri_crc = get_dri_crc(ilitek_data->blk[block].start, ilitek_data->blk[block].end - 1, buffer);
    
    for(i = 0; i < ilitek_data->slave_count; i++) {
        if(dri_crc != ilitek_data->ic[i].crc) {
            tp_log_info("Check CRC fail, must FW upgrade\n Driver CRC:0x%x ,IC[%d]_CRC:0x%x\n", dri_crc, i, ilitek_data->ic[i].crc);
            update = true;
            break;
        }
        if(ilitek_data->ic[i].mode != ILITEK_TP_MODE_APPLICATION) {
            tp_log_info("Check IC Mode fail, must FW upgrade\n IC[%d]_Mode:0x%x\n", i, ilitek_data->ic[i].mode);
            update = true;
            break;
        }
    }

    if(update) {
        ret = api_set_access_slave(ilitek_data->slave_count);
        ret = api_write_flash_enable_BL1_8(ilitek_data->blk[block].start, ilitek_data->blk[block].end);
        tp_log_info("Please wait updating...\n");
        msleep(15000);
        ret = api_get_slave_mode(ilitek_data->slave_count);
        ret = api_get_ap_crc(ilitek_data->slave_count);

        dri_crc = get_dri_crc(ilitek_data->blk[block].start, ilitek_data->blk[block].end - 1, buffer);

        for(i = 0; i < ilitek_data->slave_count; i++) {
            if(dri_crc != ilitek_data->ic[i].crc) {
                tp_log_info("Check CRC fail, FW upgrade Fail\n Driver CRC:0x%x ,IC[%d]_CRC:0x%x\n", dri_crc, i, ilitek_data->ic[i].crc);
                return ILITEK_FAIL;
            }
            if(ilitek_data->ic[i].mode != ILITEK_TP_MODE_APPLICATION) {
                tp_log_info("Check IC Mode fail, FW upgrade Fail\n IC[%d]_Mode:0x%x\n", i, ilitek_data->ic[i].mode);
                return ILITEK_FAIL;
            }
        }
    }

	if(ilitek_read_tp_info() < 0) {
		tp_log_err("init read tp info error so exit\n");
		return ILITEK_FAIL;
	}
	return ILITEK_SUCCESS;
}

static int32_t ilitek_upgrade_BL1_8(uint8_t *CTPM_FW)
{
    int ret = ILITEK_SUCCESS, count = 0;
    uint32_t update_len = UPGRADE_LENGTH_BLV1_8;
    ret = api_set_data_length(update_len);
    for(count = 0; count < ilitek_data->upg.blk_num; count++) {
        if(ilitek_data->blk[count].chk_crc == false)
        {
            if(program_block_BL1_8(CTPM_FW+32, count, update_len) < 0) {
                tp_log_err("Upgrade Block:%d Fail\n", count);
                return ILITEK_FAIL;
            }
        }
    }

    if (ilitek_changetoblmode(false) == ILITEK_FAIL)
    {
        tp_log_err("Change to ap mode failed\n");
        return ILITEK_FAIL;
    }
    msleep(500);
	if(ilitek_read_tp_info() < 0) {
		tp_log_err("init read tp info error so exit\n");
		return ILITEK_FAIL;
	}
	tp_log_info("");
    if(ilitek_data->mcu_ver[0] + (ilitek_data->mcu_ver[1] << 8) == 0x2326) {
        tp_log_info("Firmware Upgrade on Slave\n");
        ret = program_slave_BL1_8(CTPM_FW+32, 0, update_len);
    }
    tp_log_info("Firmware Upgrade Success\n");
	ilitek_reset(ilitek_data->reset_time);
    return ret;
}

bool check_FW_upgrade(unsigned char *buffer) {
	uint16_t dri_crc = 0, ic_crc = 0;
	int count = 0;
	if(ilitek_data->ptl.ver >= 0x60000) {
		int update = false; 

		for(count = 0; count < ilitek_data->upg.blk_num; count++) {
			ilitek_data->blk[count].chk_crc = false;
			ic_crc = api_get_block_crc(ilitek_data->blk[count].start, ilitek_data->blk[count].end, CRC_CALCULATION_FROM_IC);
			dri_crc = get_dri_crc(ilitek_data->blk[count].start, ilitek_data->blk[count].end - 1, buffer);
			if(ic_crc == dri_crc) 
				ilitek_data->blk[count].chk_crc = true;
			if(ilitek_data->blk[count].chk_crc == false)
				update = true;
			tp_log_info("Block:%d, IC CRC:0x%x, Driver CRC:0x%x Check:%d\n", 
			count, ic_crc, dri_crc, ilitek_data->blk[count].chk_crc);
		}
		return update;
	}
	return true;
}
int32_t ilitek_upgrade_frimware(uint32_t df_startaddr, uint32_t df_endaddr, uint32_t df_checksum,
				 uint32_t ap_startaddr, uint32_t ap_endaddr, uint32_t ap_checksum, uint8_t *CTPM_FW)
{
	int32_t ret = 0, retry = 0;
	uint8_t buf[64] = { 0 };
	bool update;

Retry:
	update = true;
	if (retry < 2) {
		retry++;
	} else {
		tp_log_err("retry 2 times upgrade fail\n");
		return ret;
	}

	tp_log_info("upgrade firmware start	reset\n");
	ilitek_reset(ilitek_data->reset_time);
	if (api_protocol_set_testmode(true) < ILITEK_SUCCESS)
		goto Retry;
	ilitek_check_busy(1000, 1, ILITEK_TP_SYSTEM_BUSY);
	//check ic type
	if (api_protocol_set_cmd(ILITEK_TP_CMD_GET_KERNEL_VERSION, NULL, buf) < ILITEK_SUCCESS)
		goto Retry;
	update = check_FW_upgrade(CTPM_FW+32);
	if(update) {
		ret = ilitek_changetoblmode(true);
		if (ret) {
			tp_log_err("change to bl mode err ret = %d\n", ret);
			goto Retry;
		} else {
			tp_log_info("ilitek change to bl mode ok\n");
		}
		if (api_protocol_set_cmd(ILITEK_TP_CMD_GET_PROTOCOL_VERSION, NULL, buf) < ILITEK_SUCCESS)
			goto Retry;
		tp_log_info("bl protocol version %d.%d\n", buf[0], buf[1]);
		ilitek_data->bl_ver[0] = buf[0];
		ilitek_data->bl_ver[1] = buf[1];
		ret = api_protocol_set_cmd(ILITEK_TP_CMD_GET_KERNEL_VERSION, NULL, buf);
		df_startaddr = buf[2] * 256 * 256 + buf[3] * 256 + buf[4];
		tp_log_info("df_start_addr = %x\n", (int32_t)df_startaddr);
		ilitek_data->page_number = 16;
		buf[0] = 0xc7;
		ret = ilitek_i2c_write_and_read(buf, 1, 10, buf, 1);
		tp_log_info("0xc7 read= %x\n", buf[0]);
	}
	if((ilitek_data->bl_ver[0] == 0x1 && ilitek_data->bl_ver[1] == 0x8) || ilitek_data->ptl.ver >= 0x60000) {
		ret = ilitek_upgrade_BL1_8(CTPM_FW);
		if (ret < 0) {
			tp_log_err("ilitek_upgrade_BL1_8 err ret = %d\n", ret);
			goto Retry;
		}
	} else if(ilitek_data->bl_ver[0] == 0x1 && ilitek_data->bl_ver[1] == 0x7) {
		ret = ilitek_upgrade_BL1_7(df_startaddr, df_endaddr, ap_startaddr, ap_endaddr, CTPM_FW);
		if (ret < 0) {
			tp_log_err("ilitek_upgrade_BL1_7 err ret = %d\n", ret);
			goto Retry;
		}
	} else if(ilitek_data->bl_ver[0] == 0x1 && ilitek_data->bl_ver[1] == 0x6) {
		if (df_startaddr < df_endaddr) {
			ilitek_data->has_df = true;
		} else {
			ilitek_data->has_df = false;
		}
		ret = ilitek_upgrade_BL1_6(df_startaddr, df_endaddr, df_checksum, ap_startaddr, ap_endaddr, ap_checksum, CTPM_FW);
		if (ret < 0) {
			tp_log_err("ilitek_upgrade_BL1_6 err ret = %d\n", ret);
			goto Retry;
		}
	} else {
		tp_log_err("not support is protocol, BL protocol:%d.%d\n", ilitek_data->bl_ver[0], ilitek_data->bl_ver[1]);
		goto Retry;
	}
	tp_log_info("upgrade firmware completed	reset\n");
	ilitek_reset(ilitek_data->reset_time);

	ret = ilitek_changetoblmode(false);
	if (ret) {
		tp_log_err("change to ap mode err\n");
		goto Retry;
	} else {
		tp_log_info("ilitek change to ap mode ok\n");
	}
	ret = api_protocol_init_func();
	return 0;
}

#ifdef ILITEK_UPDATE_FW
int check_boot_hex(char *hex)
{
	int tmp = 0;

	tmp = (ilitek_data->upg.hex_fw_ver[1] << 4) +
	       ilitek_data->upg.hex_fw_ver[2];
	tp_log_info("tmp:0x%x\n", tmp);
	switch (tmp) {
	default:
	case 0x20:
		if (strcmp(hex, "FW620X.hex") == 0)
			return ILITEK_SUCCESS;
	case 0x21:
		if (strcmp(hex, "FW621X.hex") == 0)
			return ILITEK_SUCCESS;
	case 0x22:
		if (strcmp(hex, "FW622X.hex") == 0)
			return ILITEK_SUCCESS;
	case 0x10:
		if (strcmp(hex, "FW610X.hex") == 0)
			return ILITEK_SUCCESS;
	case 0x11:
		if (strcmp(hex, "FW611X.hex") == 0)
			return ILITEK_SUCCESS;
	case 0x12:
		if (strcmp(hex, "FW612X.hex") == 0)
			return ILITEK_SUCCESS;
	}
	tp_log_info("hw_name:%s and hex version:0x%x no matching.\n", hex, tmp);
	return ILITEK_FAIL;
}
int32_t ilitek_boot_upgrade(void)
{
	int32_t ret = 0, i = 0;
	uint32_t ap_startaddr = 0xFFFF, df_startaddr = 0xFFFF, ap_endaddr = 0,
		 df_endaddr = 0, ap_checksum = 0, df_checksum = 0,
		 hex_ap_crc = 0, ic_ap_crc = 0;
	uint8_t compare_version_count = 0;
	const struct firmware *fw;
	uint8_t *CTPM_FW = NULL, buf[4] = {0};
	uint8_t *save_hex_content = NULL;
	bool older_fw = false;

	/* buf size if even */
	CTPM_FW = vmalloc(ILITEK_HEX_UPGRADE_SIZE);
	memset(CTPM_FW, 0xFF, ILITEK_HEX_UPGRADE_SIZE);
	if (!(CTPM_FW)) {
		tp_log_err("alloctation CTPM_FW memory failed\n");
		return ILITEK_FAIL;
	}
	if (gpio_is_valid(ilitek_data->touch_id)) {
		tp_log_info("touch id found, use touch_id to determine vendor and fw_name");
		/* touch_id 0: Mutto (FW62XX.hex), 1: TPK (FW61XX.hex) . */
		if (gpio_get_value(ilitek_data->touch_id) == 0)
			strlcpy(fw_name, "FW62XX.hex", 20);
		else {
			if (ilitek_hwmajor >= 0xA5 && ilitek_hwminor == 0x01)
				strlcpy(fw_name, "FW63XX.hex", 20);
			else
				strlcpy(fw_name, "FW61XX.hex", 20);
		}
	}

	ret = request_firmware(&fw, fw_name, &ilitek_data->client->dev);
	if (ret) {
		tp_log_err("request_firmware \"%s\" fail\n", fw_name);
		ret = ILITEK_FAIL;
		goto out;
	} else {
		save_hex_content = vmalloc(fw->size);
		if (!(save_hex_content)) {
			tp_log_err("alloctation save_hex_content memory failed\n");
			goto out;
		}
		memcpy(save_hex_content, fw->data, fw->size);
		msleep(20);

	}
	ret = ilitek_parse_hex_file(&df_startaddr, &df_endaddr, &df_checksum,
			&ap_startaddr, &ap_endaddr, &ap_checksum, fw->size,
			CTPM_FW, save_hex_content);
	if (gpio_is_valid(ilitek_data->touch_id) == 0 && check_boot_hex(fw_name)
			!= ILITEK_SUCCESS)
		goto out;

	compare_version_count = 8;
	tp_log_info("ap_startaddr=0x%X, ap_endaddr=0x%X, ap_checksum=0x%X\n",
			ap_startaddr, ap_endaddr, ap_checksum);
	tp_log_info("df_startaddr=0x%X, df_endaddr=0x%X, df_checksum=0x%X\n",
			df_startaddr, df_endaddr, df_checksum);
	tp_log_info("compare_version_count = %d\n", compare_version_count);
	tp_log_info("force_update=%d\n", ilitek_data->force_update);
	if (!(ilitek_data->force_update)) {
		for (i = 0; i < compare_version_count; i++) {
			tp_log_info("ilitek_data.firmware_ver[%d] = %d, firmware_ver[%d] = %d\n",
					i, ilitek_data->firmware_ver[i], i,
					ilitek_data->upg.hex_fw_ver[i]);
			if (ilitek_data->upg.hex_fw_ver[i] ==
					ilitek_data->firmware_ver[i]) {
				tp_log_info("\n");
			} else if (ilitek_data->upg.hex_fw_ver[i] >
					ilitek_data->firmware_ver[i]) {
				tp_log_info("\n");
				break;
			} else {
				tp_log_info("\n");
				older_fw = true;
				break;
			}
		}

		/* FW versions match. Check if the CRC rev matches as well. */
		if (older_fw) {
			tp_log_info("Firmware version is older. Do not upgrade.\n");
			ret = ILITEK_SUCCESS;
			goto out;
		} else if (i == compare_version_count) {
			for (i = ap_startaddr; i < ap_endaddr - 1; i++)
				hex_ap_crc = UpdateCRC(hex_ap_crc, CTPM_FW[i + 32]);
			buf[0] = (uint8_t)0xC7;
			ilitek_i2c_write_and_read(buf, 1, 10, buf, sizeof(buf));
			ic_ap_crc = buf[0] + (buf[1] << 8);
			tp_log_info("hex_ap_crc=0x%x, ic_ap_crc=0x%x\n", hex_ap_crc, ic_ap_crc);
			/* Compare hex_ap_crc and ic_ap_crc
			 * hex_ap_crc : CRC value of the hex file in
			 *          the image, calculated by the driver
			 * ic_ap_crc: current FW CRC in the chip
			 */
			if (hex_ap_crc != ic_ap_crc) {
				tp_log_info("Firmware version is a match, but CRC isn't. Upgrade FW.\n");
			} else {
				tp_log_info("Firmware version and CRC are a match. Do not upgrade.\n");
				ret = ILITEK_SUCCESS;
				goto out;
			}
		}
	}
	ret = ILITEK_FAIL;
	if ((ilitek_data->upg.hex_ic_type[0] != 0 ||
		ilitek_data->upg.hex_ic_type[1] != 0) &&
		(ilitek_data->mcu_ver[0] != ilitek_data->upg.hex_ic_type[0] ||
		 ilitek_data->mcu_ver[1] != ilitek_data->upg.hex_ic_type[1])) {
		tp_log_info("upgrade file mismatch!!! ic is ILI%02X%02X, upgrade file is ILI%02X%02X\n",
			     ilitek_data->mcu_ver[1], ilitek_data->mcu_ver[0],
			     ilitek_data->upg.hex_ic_type[1],
			     ilitek_data->upg.hex_ic_type[0]);
	} else if (ilitek_data->upg.hex_ic_type[0] == 0 &&
			ilitek_data->upg.hex_ic_type[1] == 0 &&
			ilitek_data->mcu_ver[0] != 0x03 &&
			ilitek_data->mcu_ver[0] != 0x09) {
		tp_log_info("upgrade file  mismatch!!! ic is ILI%02X%02X, upgrade file is maybe ILI230X\n", ilitek_data->mcu_ver[1],
				ilitek_data->mcu_ver[0]);
	} else {
		ret = ilitek_upgrade_frimware(df_startaddr, df_endaddr, df_checksum, ap_startaddr, ap_endaddr, ap_checksum, CTPM_FW);
	}
out:
	if (CTPM_FW) {
		vfree(CTPM_FW);
		CTPM_FW = NULL;
	}
	if (save_hex_content) {
		vfree(save_hex_content);
		save_hex_content = NULL;
	}
	if (ret < 0) {
		tp_log_err("upgrade fail\n");
	}
	return ret;
}
#endif
