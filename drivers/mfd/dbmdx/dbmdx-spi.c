/*
 * dbmdx-spi.c - DSPG DBMDX SPI interface driver
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/* #define DEBUG */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/firmware.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-spi.h>

#define DEFAULT_SPI_WRITE_CHUNK_SIZE	8
#define MAX_SPI_WRITE_CHUNK_SIZE	0x40000
#define DEFAULT_SPI_READ_CHUNK_SIZE	8
#define MAX_SPI_READ_CHUNK_SIZE		8192

static const u8 checksum_cmd[] = {0xDB, 0xD7, 0x00, 0x0E};


static DECLARE_WAIT_QUEUE_HEAD(dbmdx_wq);

int spi_set_speed(struct dbmdx_private *p, int index)
{

	struct spi_device *spi = to_spi_device(p->dev);
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;
	int ret = 0;
	u32 bits_per_word = 0;
	u32 spi_rate = 0;
	u16 spi_mode = SPI_MODE_0;

	if (index >= DBMDX_NR_OF_SPEEDS) {
		dev_err(p->dev, "%s: Invalid speed index %x\n",
			__func__, index);
		return -EINVAL;
	}

	spi_rate = p->pdata->host_speed_cfg[index].spi_rate -
		(p->pdata->host_speed_cfg[index].spi_rate % 1000);

	bits_per_word = p->pdata->host_speed_cfg[index].spi_rate % 100;

	spi_mode = (u16)(((p->pdata->host_speed_cfg[index].spi_rate % 1000) -
				bits_per_word) / 100);


	if (bits_per_word != 8 && bits_per_word != 16 && bits_per_word != 32)
		bits_per_word = 8;

	if (spi_mode == 0)
		spi_mode = SPI_MODE_0;
	else if (spi_mode == 1)
		spi_mode = SPI_MODE_1;
	else if (spi_mode == 2)
		spi_mode = SPI_MODE_2;
	else if (spi_mode == 3)
		spi_mode = SPI_MODE_3;
	else
		spi_mode = SPI_MODE_0;

	if (spi->max_speed_hz != spi_rate || spi->mode != spi_mode) {

		spi->max_speed_hz = spi_rate;
		spi->mode = spi_mode;

		spi->bits_per_word = bits_per_word;

		spi_data->bits_per_word = spi->bits_per_word;
		spi_data->bytes_per_word = spi->bits_per_word / 8;

		dev_info(p->dev,
			"%s Update SPI Max Speed to %d Hz, bpw: %d, mode: %d\n",
			__func__,
			spi->max_speed_hz,
			spi->bits_per_word,
			spi->mode);

		ret = spi_setup(spi);
		if (ret < 0)
			dev_err(p->dev, "%s:failed %x\n", __func__, ret);
	}

	return ret;
}


ssize_t read_spi_data(struct dbmdx_private *p, void *buf, size_t len)
{
	struct spi_device *spi = to_spi_device(p->dev);
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;
	size_t count = spi_data->read_chunk_size;
	u32 bytes_per_word = spi_data->bytes_per_word;
	u8 *recv = spi_data->recv;
	ssize_t i;
	size_t pad_size = 0;
	int ret;
	u8 *d = (u8 *)buf;
	/* if stuck for more than 10s, something is wrong */
	unsigned long timeout = jiffies + msecs_to_jiffies(10000);

	for (i = 0; i < len; i += count) {
		if ((i + count) > len) {
			count = len - i;
			if (count % (size_t)bytes_per_word != 0)
				pad_size = (size_t)bytes_per_word -
				(size_t)(count % (size_t)bytes_per_word);
			count = count + pad_size;
		}

		ret =  spi_read(spi, recv, count);
		if (ret < 0) {
			dev_err(p->dev, "%s: spi_read failed\n",
				__func__);
			i = -EIO;
			goto out;
		}
		memcpy(d + i, recv, count - pad_size);

		if (!time_before(jiffies, timeout)) {
			dev_err(p->dev,
				"%s: read data timed out after %zd bytes\n",
				__func__, i);
			i = -ETIMEDOUT;
			goto out;
		}
	}

	return len;
out:
	return i;
}


ssize_t write_spi_data(struct dbmdx_private *p, const u8 *buf,
			      size_t len)
{
	struct spi_device *spi = to_spi_device(p->dev);
	int rc;

	rc = spi_write(spi, buf, len);
	if (rc != 0) {
		dev_err(p->dev, "%s(): error %d writing SR\n",
				__func__, rc);
		return rc;
	} else
		return len;
}


ssize_t send_spi_data(struct dbmdx_private *p, const void *buf,
			      size_t len)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;
	u32 bytes_per_word = spi_data->bytes_per_word;
	int ret = 0;
	const u8 *cmds = (const u8 *)buf;
	size_t to_copy = len;
	size_t max_size = (size_t)(spi_data->write_chunk_size);
	size_t pad_size = 0;
	size_t cur_send_size = 0;
	u8 *send = spi_data->send;

	while (to_copy > 0) {
		if (to_copy < max_size) {
			memset(send, 0, max_size);
			memcpy(send, cmds, to_copy);

			if (to_copy % (size_t)bytes_per_word != 0)
				pad_size = (size_t)bytes_per_word -
				(size_t)(to_copy % (size_t)bytes_per_word);

			cur_send_size = to_copy + pad_size;
		} else {
			memcpy(send, cmds, max_size);
			cur_send_size = max_size;
		}

		ret = write_spi_data(p, send, cur_send_size);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: send_spi_data failed ret=%d\n",
				__func__, ret);
			break;
		}
		to_copy -= (ret - pad_size);
		cmds += (ret - pad_size);
	}

	return len - to_copy;
}

int spi_verify_boot_checksum(struct dbmdx_private *p,
	const void *checksum, size_t chksum_len)
{
	int ret;
	u8 rx_checksum[10] = {0};

	if (!checksum)
		return 0;

	if (chksum_len > 8) {
		dev_err(p->dev, "%s: illegal checksum length\n", __func__);
		return -EINVAL;
	}

	ret = send_spi_data(p, checksum_cmd, sizeof(checksum_cmd));

	if (ret != sizeof(checksum_cmd)) {
		dev_err(p->dev, "%s: failed to checksum cmd\n",	__func__);
		return -EIO;
	}

	usleep_range(DBMDX_USLEEP_BEFORE_CHECKSUM_READ,
			DBMDX_USLEEP_BEFORE_CHECKSUM_READ + 1000);
	ret = read_spi_data(p, (void *)rx_checksum,
				chksum_len + sizeof(checksum_cmd) + 1);

	if (ret < 0) {
		dev_err(p->dev, "%s: could not read checksum data\n",
			__func__);
		return -EIO;
	}

	ret = p->verify_checksum(p, checksum,
			&rx_checksum[sizeof(checksum_cmd) + 1], chksum_len);
	if (ret) {
		dev_err(p->dev, "%s: checksum mismatch\n", __func__);
		return -EILSEQ;
	}

	return 0;
}

int spi_verify_chip_id(struct dbmdx_private *p)
{
	int ret;

	/* Read chip revision */
	u8 read_ver_cmd[8] = {0xDB, 0xD7, 0x01, 0x04, 0x74, 0x00, 0x00, 0x03};
	u8 return_ver[9] = {0};
	/* Answer format: "DB D7 01 04 7X DB 00 00", where X is chip revision */
	u8 expected_ver[4] = {0x70, 0xDB, 0x00, 0x00};
	u8 chip_rev = 0;

	send_spi_data(p, read_ver_cmd, 8);

	usleep_range(DBMDX_USLEEP_SPI_CMD_AFTER_BOOT,
				DBMDX_USLEEP_SPI_CMD_AFTER_BOOT + 1000);

	ret = read_spi_data(p, return_ver, 9);
	if (ret < 0) {
		dev_err(p->dev, "%s: error read_spi_data\n", __func__);
		return ret;
	}

	dev_info(p->dev, "%s: chip revision\n", __func__);

	if (memcmp(&(return_ver[1]), read_ver_cmd, 4) != 0) {
		dev_err(p->dev,
			"%s:Wrong read chip rev resp: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
				__func__,
				return_ver[1],
				return_ver[2],
				return_ver[3],
				return_ver[4],
				return_ver[5],
				return_ver[6],
				return_ver[7],
				return_ver[8]);
		return -EIO;
	}

	chip_rev = return_ver[5];

	return_ver[5] = 0x70;

	if (memcmp(&(return_ver[5]), expected_ver, 4) != 0) {
		dev_err(p->dev,
			"%s: Wrong chip rev. Received 0x%02x%02x%02x%02x Expected: 0x%02x%02x%02x%02x\n",
				__func__,
				chip_rev,
				return_ver[6],
				return_ver[7],
				return_ver[8],
				expected_ver[0],
				expected_ver[1],
				expected_ver[2],
				expected_ver[3]);
		return -EILSEQ;
	}

	return_ver[5] = chip_rev;

	memcpy(&p->chip_revision, &return_ver[5], 4);

	dev_info(p->dev, "%s: Chip rev is: 0x%08x", __func__, p->chip_revision);

	return 0;
}

static int spi_can_boot(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	return 0;
}
static int spi_prepare_preboot(struct dbmdx_private *p)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = spi_set_speed(p, DBMDX_SPEED_PREBOOT);

	return ret;
}
static int spi_prepare_boot(struct dbmdx_private *p)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = spi_set_speed(p, DBMDX_SPEED_MAX);

	return ret;
}

static int spi_finish_boot(struct dbmdx_private *p)
{
	int ret = 0;

	ret = spi_set_speed(p, DBMDX_SPEED_NORMAL);
	if (ret < 0)
		dev_err(p->dev, "%s:failed %x\n", __func__, ret);

	return ret;
}

static int spi_dump_state(struct dbmdx_private *p, char *buf)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;
	int off = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t===SPI Interface  Dump====\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSPI Write Chunk Size:\t\t%d\n",
				spi_data->write_chunk_size);
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tSPI Read Chunk Size:\t\t%d\n",
				spi_data->read_chunk_size);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tSPI DMA Min Buffer Size:\t\t%d\n",
				spi_data->dma_min_buffer_size);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tInterface resumed:\t%s\n",
			p->pdata->interface_enabled ? "ON" : "OFF");

	return off;
}

static int spi_set_primary_firmware_ready(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	return 0;
}


static void spi_transport_enable(struct dbmdx_private *p, bool enable)
{
	//int ret = 0;

	dev_dbg(p->dev, "%s (%s)\n", __func__, enable ? "ON" : "OFF");
#if 0
	if (enable) {

		ret = wait_event_interruptible(dbmdx_wq,
			p->pdata->interface_enabled);

		if (ret)
			dev_dbg(p->dev,
				"%s, waiting for interface was interrupted",
				__func__);
		else
			dev_dbg(p->dev, "%s, interface is active\n",
				__func__);
	}
#endif
	if (enable) {
		p->wakeup_set(p);
		msleep(DBMDX_MSLEEP_SPI_WAKEUP);
	} else {
		p->wakeup_release(p);
	}
}

static void spi_resume(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	spi_interface_resume(p);
}


void spi_interface_resume(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	p->pdata->interface_enabled = 1;
	//wake_up_interruptible(&dbmdx_wq);
}

static void spi_suspend(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	spi_interface_suspend(p);
}


void spi_interface_suspend(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);

	p->pdata->interface_enabled = 0;
}

static int spi_prepare_buffering(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	return 0;
}

static int spi_read_audio_data(struct dbmdx_private *p,
	void *buf,
	size_t samples,
	bool to_read_metadata,
	size_t *available_samples,
	size_t *data_offset)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	return ret;
}

static int spi_finish_buffering(struct dbmdx_private *p)
{
	int ret = 0;

	dev_dbg(p->dev, "%s\n", __func__);


	return ret;
}

static int spi_prepare_amodel_loading(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	return 0;
}

static int spi_finish_amodel_loading(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s\n", __func__);
	/* do the same as for finishing buffering */

	return 0;
}

static u32 spi_get_read_chunk_size(struct dbmdx_private *p)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;

	dev_dbg(p->dev, "%s SPI read chunk is %u\n",
		__func__, spi_data->read_chunk_size);

	return spi_data->read_chunk_size;
}

static u32 spi_get_write_chunk_size(struct dbmdx_private *p)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;
	dev_dbg(p->dev, "%s SPI write chunk is %u\n",
		__func__, spi_data->write_chunk_size);

	return spi_data->write_chunk_size;
}

static int spi_set_read_chunk_size(struct dbmdx_private *p, u32 size)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;

	if (size > MAX_SPI_READ_CHUNK_SIZE) {
		dev_err(p->dev,
			"%s Error setting SPI read chunk. Max chunk size: %u\n",
		__func__, MAX_SPI_READ_CHUNK_SIZE);
		return -EINVAL;
	} else if ((size % 4) != 0) {
		dev_err(p->dev,
			"%s Error SPI read chunk should be multiply of 4\n",
		__func__);
		return -EINVAL;
	} else if (size == 0)
		spi_data->read_chunk_size = DEFAULT_SPI_READ_CHUNK_SIZE;
	else
		spi_data->read_chunk_size = size;

	dev_dbg(p->dev, "%s SPI read chunk was set to %u\n",
		__func__, spi_data->read_chunk_size);

	return 0;
}

static int spi_set_write_chunk_size(struct dbmdx_private *p, u32 size)
{
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;

	if (size > MAX_SPI_WRITE_CHUNK_SIZE) {
		dev_err(p->dev,
			"%s Error setting SPI write chunk. Max chunk size: %u\n",
		__func__, MAX_SPI_WRITE_CHUNK_SIZE);
		return -EINVAL;
	} else if ((size % 4) != 0) {
		dev_err(p->dev,
			"%s Error SPI write chunk should be multiply of 4\n",
		__func__);
		return -EINVAL;
	} else if (size == 0)
		spi_data->write_chunk_size = DEFAULT_SPI_WRITE_CHUNK_SIZE;
	else
		spi_data->write_chunk_size = size;

	dev_dbg(p->dev, "%s SPI write chunk was set to %u\n",
		__func__, spi_data->write_chunk_size);

	return 0;
}

bool spi_interface_adds_leading_byte_on_read(struct dbmdx_private *p)
{
	return true;
}

int spi_common_probe(struct spi_device *client)
{
#ifdef CONFIG_OF
	struct  device_node *np;
#endif
	struct chip_interface *chip;
	struct dbmdx_private *p = dev_get_drvdata(&client->dev);
	struct dbmdx_platform_data *pdata = p->pdata;
	struct dbmdx_spi_data *spi_data;
	int ret;

	dev_dbg(&client->dev, "%s(): dbmdx\n", __func__);

#ifdef CONFIG_OF
	np = p->dev->of_node;
	if (!np) {
		dev_err(p->dev, "%s: no devicetree entry\n", __func__);
		ret = -EINVAL;
		goto out;
	}
#else
	if (pdata == NULL) {
		dev_err(p->dev, "%s: dbmdx, no platform data found\n",
			 __func__);
		ret = -ENODEV;
		goto out;
	}
#endif
	chip = kzalloc(sizeof(struct chip_interface), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto out;
	}
	chip->pdata = pdata;
	p->chip = chip;

	spi_data = kzalloc(sizeof(struct dbmdx_spi_data), GFP_KERNEL);
	if (!spi_data) {
		ret = -ENOMEM;
		goto out_err_mem_free;
	}
	p->interface_data = (void *)spi_data;
#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "spi-max-frequency",
		&(spi_data->spi_speed));
	if (ret && ret != -EINVAL)
		spi_data->spi_speed = 2000000;
#endif
	dev_dbg(p->dev, "%s: spi speed is %u\n", __func__, spi_data->spi_speed);

#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "read-chunk-size",
		&spi_data->read_chunk_size);
	if (ret != 0) {
		/*
		 * read-chunk-size not set, set it to default
		 */
		spi_data->read_chunk_size = DEFAULT_SPI_READ_CHUNK_SIZE;
		dev_info(p->dev,
			"%s: Setting spi read chunk to default val: %u bytes\n",
			__func__, spi_data->read_chunk_size);
	}
#endif
	if (spi_data->read_chunk_size % 4 != 0)
		spi_data->read_chunk_size += (4 - (spi_data->read_chunk_size % 4));

	if (spi_data->read_chunk_size > MAX_SPI_READ_CHUNK_SIZE)
		spi_data->read_chunk_size = MAX_SPI_READ_CHUNK_SIZE;
	if (spi_data->read_chunk_size == 0)
		spi_data->read_chunk_size = DEFAULT_SPI_READ_CHUNK_SIZE;

	dev_info(p->dev, "%s: Setting spi read chunk to %u bytes\n",
			__func__, spi_data->read_chunk_size);

#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "write-chunk-size",
		&spi_data->write_chunk_size);
	if (ret != 0) {
		/*
		 * write-chunk-size not set, set it to default
		 */
		spi_data->write_chunk_size = DEFAULT_SPI_WRITE_CHUNK_SIZE;
		dev_info(p->dev,
			"%s: Setting spi write chunk to default val: %u bytes\n",
			__func__, spi_data->write_chunk_size);
	}
#endif
	if (spi_data->write_chunk_size % 4 != 0)
		spi_data->write_chunk_size += (4 - (spi_data->write_chunk_size % 4));

	if (spi_data->write_chunk_size > MAX_SPI_WRITE_CHUNK_SIZE)
		spi_data->write_chunk_size = MAX_SPI_WRITE_CHUNK_SIZE;
	if (spi_data->write_chunk_size == 0)
		spi_data->write_chunk_size = DEFAULT_SPI_WRITE_CHUNK_SIZE;

	dev_info(p->dev, "%s: Setting spi write chunk to %u bytes\n",
			__func__, spi_data->write_chunk_size);

#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "dma_min_buffer_size",
		&spi_data->dma_min_buffer_size);
	if (ret != 0) {
		/*
		 * read-chunk-size not set, set it to default
		 */
		spi_data->dma_min_buffer_size = 0;
		dev_info(p->dev,
			"%s: Setting Min DMA Cmd Size to default: %u bytes\n",
			__func__, spi_data->dma_min_buffer_size);
	}
#endif
	if (spi_data->dma_min_buffer_size > DBMDX_SPI_CMD_PADDED_SIZE)
		spi_data->dma_min_buffer_size = DBMDX_SPI_CMD_PADDED_SIZE;
	if (spi_data->dma_min_buffer_size < 7 && spi_data->dma_min_buffer_size > 0)
		spi_data->dma_min_buffer_size = 7;

	dev_info(p->dev, "%s: Setting Min DMA Cmd Size to default: %u bytes\n",
			__func__, spi_data->dma_min_buffer_size);


	spi_data->send = kmalloc(MAX_SPI_WRITE_CHUNK_SIZE,
				 GFP_KERNEL | GFP_DMA);
	if (!spi_data->send) {
		dev_err(p->dev,
			"%s: Cannot allocate memory spi send buffer\n",
			__func__);
		goto out_err_mem_free1;
	}

	spi_data->recv = kmalloc(MAX_SPI_READ_CHUNK_SIZE, GFP_KERNEL | GFP_DMA);
	if (!spi_data->recv) {
		dev_err(p->dev,
			"%s: Cannot allocate memory spi recv buffer\n",
			__func__);
		goto out_err_mem_free2;
	}

	/* fill in chip interface functions */
	p->chip->can_boot = spi_can_boot;
	p->chip->prepare_preboot = spi_prepare_preboot;
	p->chip->prepare_boot = spi_prepare_boot;
	p->chip->finish_boot = spi_finish_boot;
	p->chip->dump = spi_dump_state;
	p->chip->set_primary_firmware_ready = spi_set_primary_firmware_ready;
	p->chip->transport_enable = spi_transport_enable;
	p->chip->read = read_spi_data;
	p->chip->write = send_spi_data;
	p->chip->verify_boot_checksum = spi_verify_boot_checksum;
	p->chip->prepare_buffering = spi_prepare_buffering;
	p->chip->read_audio_data = spi_read_audio_data;
	p->chip->finish_buffering = spi_finish_buffering;
	p->chip->prepare_amodel_loading = spi_prepare_amodel_loading;
	p->chip->finish_amodel_loading = spi_finish_amodel_loading;
	p->chip->get_write_chunk_size = spi_get_write_chunk_size;
	p->chip->get_read_chunk_size = spi_get_read_chunk_size;
	p->chip->set_write_chunk_size = spi_set_write_chunk_size;
	p->chip->set_read_chunk_size = spi_set_read_chunk_size;
	p->chip->interface_adds_leading_byte_on_read =
				spi_interface_adds_leading_byte_on_read;
	p->chip->resume = spi_resume;
	p->chip->suspend = spi_suspend;

	pdata->interface_enabled = 1;

	dev_info(&client->dev, "%s: successfully probed\n", __func__);
	ret = 0;

	goto out;
out_err_mem_free2:
	kfree(spi_data->send);
out_err_mem_free1:
	kfree(spi_data);
	p->interface_data = NULL;
out_err_mem_free:
	kfree(chip);
	p->chip = NULL;
out:
	return ret;
}

int spi_common_remove(struct spi_device *client)
{
	struct dbmdx_private *p = spi_get_drvdata(client);
	struct dbmdx_spi_data *spi_data = (struct dbmdx_spi_data *)
						p->interface_data;

	kfree(spi_data->send);
	kfree(spi_data->recv);
	kfree(p->chip);
	kfree(p);

	spi_set_drvdata(client, NULL);

	return 0;
}


