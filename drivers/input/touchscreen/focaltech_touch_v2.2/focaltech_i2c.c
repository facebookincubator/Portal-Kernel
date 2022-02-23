/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include "focaltech_core.h"

#define I2C_RETRY_NUMBER		3

static DEFINE_MUTEX(i2c_rw_access);

int fts_i2c_read(struct i2c_client *client, char *writebuf,
		 int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int i = 0;

	mutex_lock(&i2c_rw_access);

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = 0,
					.len = writelen,
					.buf = writebuf,
				}, {
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = fts_data->rbuf,
				},
			};
			for (i = 0; i < I2C_RETRY_NUMBER; i++) {
				ret = i2c_transfer(client->adapter, msgs, 2);
				if (ret < 0)
					FTS_ERROR("[IIC]: i2c_transfer(write) error, ret=%d!!", ret);
				else
					break;
			}
		} else {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = fts_data->rbuf,
				},
			};
			for (i = 0; i < I2C_RETRY_NUMBER; i++) {
				ret = i2c_transfer(client->adapter, msgs, 1);
				if (ret < 0)
					FTS_ERROR("[IIC]: i2c_transfer(read) error, ret=%d!!", ret);
				else
					break;
			}
		}
	}
	memcpy(readbuf, fts_data->rbuf, readlen);
	mutex_unlock(&i2c_rw_access);
	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;
	int i = 0;

	mutex_lock(&i2c_rw_access);
	memcpy(fts_data->rbuf, writebuf, writelen);
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		for (i = 0; i < I2C_RETRY_NUMBER; i++) {
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				FTS_ERROR("%s: i2c_transfer(write) error, ret=%d", __func__, ret);
			else
				break;
		}
	}
	mutex_unlock(&i2c_rw_access);

	return ret;
}

int fts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	u8 buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fts_i2c_write(client, buf, sizeof(buf));
}

int fts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

void fts_i2c_hid2std(struct i2c_client *client)
{
	int ret = 0;
	u8 buf[3] = {0xeb, 0xaa, 0x09};

	ret = fts_i2c_write(client, buf, 3);
	if (ret < 0) {
		FTS_ERROR("hid2std cmd write fail");
	} else {
		udelay(10);
		buf[0] = buf[1] = buf[2] = 0;
		ret = fts_i2c_read(client, NULL, 0, buf, 3);
		if (ret < 0)
			FTS_ERROR("hid2std cmd read fail");
		else if ((buf[0] == 0xeb) && (buf[1] == 0xaa)
			  && (buf[2] == 0x08))
			FTS_DEBUG("hidi2c change to stdi2c successful");
		else
			FTS_ERROR("hidi2c change to stdi2c fail");
	}
}

int fts_i2c_init(void)
{
	FTS_FUNC_ENTER();

	FTS_FUNC_EXIT();
	return 0;
}

int fts_i2c_exit(void)
{
	FTS_FUNC_ENTER();

	FTS_FUNC_EXIT();
	return 0;
}

