/*
 * dbmdx.c -- DSPG DBMDX MFD driver
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

#define DEBUG

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/kfifo.h>
#include <linux/vmalloc.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/mfd/core.h>
#include <linux/pm_runtime.h>
#include <linux/fb_event.h>
#include <uapi/linux/fb_event.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-of.h>
#include <linux/mfd/dbmdx/dbmdx-chip-interfaces.h>
#include <linux/mfd/dbmdx/dbmdx-clocks.h>
#include <linux/mfd/dbmdx/dbmdx-utils.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmdx-common-operations.h>
#include <linux/mfd/dbmdx/dbmdx-sysfs.h>
#include <linux/mfd/dbmdx/dbmdx-usecases.h>
#include <linux/mfd/dbmdx/dbmdx-cdev.h>
#include <sound/dbmdx-export.h>
#define CREATE_TRACE_POINTS
#include <trace/events/fb_audio.h>

#define USE_DEDICATED_WORKQUEUE
#define MIN_RETRIES_TO_WRITE_TOBUF		5
#define MAX_RETRIES_TO_WRITE_TOBUF		200
#define MAX_AMODEL_SIZE				(148 * 1024)

#define MIN_EVENT_PROCESSING_TIME_MS		500
#define FW_START_TIMEOUT			2000

#ifndef RETRY_COUNT
#define RETRY_COUNT				5
#endif

static const struct mfd_cell dbmdx_cells[] = {
	{
		.name = "dbmdx-codec",
		.of_compatible = "dspg,dbmdx-codec",
	},
	{
		.name = "dbmdx-log",
		.of_compatible = "dspg,dbmdx-log",
	},

#ifdef CONFIG_DBMDX_STAX_DSPG
	{
		.name = "dbmdx-stax",
		.of_compatible = "dspg,dbmdx-stax",
	},
#endif
#ifdef CONFIG_DBMDX_MHA
	{
		.name = "dbmdx-mha",
		.of_compatible = "dspg,dbmdx-mha",
	},
#endif
};

static const char *dbmdx_fw_names[DBMDX_FW_MAX] = {
	[DBMDX_FW_PRE_BOOT]	= "PRE_BOOT",
	[DBMDX_FW_PRIMARY]	= "PRIMARY",
	[DBMDX_FW_POWER_OFF]	= "POWER_OFF",
};

/* Global Variables */
struct dbmdx_private *dbmdx_data;
void (*g_event_callback)(int) = NULL;

int dbmdx_force_crash_fw(struct device *dev, int disable_log_event)
{
	struct dbmdx_private *p = DBMDX_PRIV(dev);
	u32 ack_val;
	int ret = 0;

	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
			MSG_ID_ADDON_FB, TEST_TIMEOUT_ERROR,
			0, &ack_val);
	if (ret < 0) {
		if (ret == -EIO) {
			dev_info(p->dev, "Firmware crash triggered successfully");
			/* reset this flag only after recovery */
			if (disable_log_event)
				p->disable_log_event = true;
			ret = 0;
		} else {
			dev_err(p->dev, "%s: Error %d writing register %d\n",
					__func__, ret, TEST_TIMEOUT_ERROR);
		}
	}
	return ret;
}


int dbmdx_event_log_int(struct device *dev, int32_t type, int value)
{
	struct dbmdx_private *p = DBMDX_PRIV(dev);

	if (!p->disable_log_event)
		fb_event_log_int(type, value);
	return 0;
}

int dbmdx_schedule_work(struct dbmdx_private *p,
				struct work_struct *work)
{
#ifdef USE_DEDICATED_WORKQUEUE
	return queue_work(p->dbmdx_workq, work);
#else
	return schedule_work(work);
#endif
}

int dbmdx_schedule_delayed_work(struct dbmdx_private *p,
				struct delayed_work *work, unsigned long delay)
{
#ifdef USE_DEDICATED_WORKQUEUE
	return queue_delayed_work(p->dbmdx_workq, work, delay);
#else
	return schedule_delayed_work(work, delay);
#endif
}
const char *dbmdx_fw_type_to_str(int fw_type)
{
	if (fw_type >= DBMDX_FW_MAX)
		return "ERROR";
	return dbmdx_fw_names[fw_type];
}

int dbmdx_cell_force_suspend(struct device *dev, void *data)
{
	pm_runtime_force_suspend(dev);
	return 0;
}

int dbmdx_cell_force_resume(struct device *dev, void *data)
{
	pm_runtime_force_resume(dev);
	return 0;
}
static int dbmdx_switch_to_chip_interface(struct dbmdx_private *p,
		enum dbmdx_interface_type interface_type)

{
	int ret = 0;
	int i;
	/* set Primary FW as active firmware */
	p->active_fw = p->active_fw_chip;

	ret = dbmdx_set_active_interface(p,
		p->pdata->interfaces[interface_type]);

	if (ret) {
		dev_err(p->dev, "%s: failed to set interface\n", __func__);
		return ret;
	}

	p->active_interface_type_primary = interface_type;
	p->cur_reset_gpio = p->pdata->gpio_reset;
	p->cur_wakeup_gpio = p->pdata->gpio_wakeup;
	p->cur_wakeup_disabled = p->pdata->wakeup_disabled;
	p->cur_wakeup_set_value = p->pdata->wakeup_set_value;
	p->cur_send_wakeup_seq = p->pdata->send_wakeup_seq;
	p->cur_use_gpio_for_wakeup = p->pdata->use_gpio_for_wakeup;
	p->cur_firmware_id_num = p->pdata->firmware_id_num;
	p->cur_boot_options = p->pdata->boot_options;

	for (i = 0; i < p->cur_firmware_id_num; i++)
		p->cur_firmware_id[i] = p->pdata->firmware_id[i];

	p->active_chip = DBMDX_CHIP_PRIMARY;

	return ret;
}


static void dbmdx_set_primary_active(struct dbmdx_private *p)
{
	/* set Primary FW as active firmware */
	p->active_fw_chip = DBMDX_FW_PRIMARY;
	/* reset all flags */
	memset(&p->primary_flags, 0, sizeof(p->primary_flags));
}

static void dbmdx_set_boot_active(struct dbmdx_private *p)
{
	/* set nothing as active firmware */
	p->active_fw_chip =  DBMDX_FW_PRE_BOOT;
	p->device_ready = false;
	p->asleep = false;
}

static void dbmdx_reset_set(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__, 0, p->cur_reset_gpio);

	gpio_set_value(p->cur_reset_gpio, 0);
}

static void dbmdx_reset_release(struct dbmdx_private *p)
{
	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__, 1, p->cur_reset_gpio);

	gpio_set_value(p->cur_reset_gpio, 1);

}

static void dbmdx_reset_sequence(struct dbmdx_private *p)
{
	dbmdx_reset_set(p);
	usleep_range(DBMDX_USLEEP_RESET_TOGGLE,
		DBMDX_USLEEP_RESET_TOGGLE + 100);
	dbmdx_reset_release(p);
}

static int dbmdx_can_wakeup(struct dbmdx_private *p)
{
	if (p->cur_wakeup_disabled)
		return 0;

	/* If use_gpio_for_wakeup equals zero than transmit operation
	itself will wakeup the chip */
	if (!p->cur_use_gpio_for_wakeup)
		return 1;

	return p->cur_wakeup_gpio < 0 ? 0 : 1;
}

static void dbmdx_wakeup_set(struct dbmdx_private *p)
{
	/* If use_gpio_for_wakeup equals zero than transmit operation
	itself will wakeup the chip */
	if (p->cur_wakeup_disabled || p->cur_wakeup_gpio < 0 ||
		!p->cur_use_gpio_for_wakeup)
		return;

	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__,
		p->cur_wakeup_set_value, p->cur_wakeup_gpio);

	gpio_set_value(p->cur_wakeup_gpio, p->cur_wakeup_set_value);
}

static void dbmdx_wakeup_release(struct dbmdx_private *p)
{
	/* If use_gpio_for_wakeup equals zero than transmit operation
	itself will wakeup the chip */
	if (p->cur_wakeup_disabled || p->cur_wakeup_gpio < 0 ||
		!p->cur_use_gpio_for_wakeup)
		return;

	dev_dbg(p->dev, "%s: %d==>gpio%d\n", __func__,
		!(p->cur_wakeup_set_value), p->cur_wakeup_gpio);

	gpio_set_value(p->cur_wakeup_gpio, !(p->cur_wakeup_set_value));
}

static void dbmdx_lock(struct dbmdx_private *p)
{
	mutex_lock(&p->p_lock);
}

static void dbmdx_unlock(struct dbmdx_private *p)
{
	mutex_unlock(&p->p_lock);
}

static int dbmdx_verify_checksum(struct dbmdx_private *p,
	const u8 *expect, const u8 *got, size_t size)
{
	int ret;

	ret = memcmp(expect, got, size);
	if (ret) {
		switch (size) {
		case 4:
			dev_info(p->dev,
				"%s: Got:      0x%02x 0x%02x 0x%02x 0x%02x\n",
				__func__,
				got[0], got[1], got[2], got[3]);
			dev_info(p->dev,
				"%s: Expected: 0x%02x 0x%02x 0x%02x 0x%02x\n",
				__func__,
				expect[0], expect[1], expect[2], expect[3]);
			break;
		default:
			break;
		}
	}
	return ret;
}

ssize_t dbmdx_send_data(struct dbmdx_private *p, const void *buf,
			       size_t len)
{
	return p->chip->write(p, buf, len);
}

int dbmdx_send_cmd(struct dbmdx_private *p, u32 command, u16 *response)
{
	int ret = 0;

	switch (p->active_fw) {
	case DBMDX_FW_PRIMARY:
		break;
	default:
		dev_err(p->dev, "%s: Don't know how to handle fw type %d\n",
			__func__, p->active_fw);
		ret = -EIO;
		break;
	}
	return ret;
}

int dbmdx_buf_to_int(const char *buf)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	return (int)val;
}

static int dbmdx_calc_fw_checksum(struct dbmdx_private *p,
					const char *fw_data,
					unsigned long fw_len,
					u8 *fw_file_checksum)
{
	u32 sum = 0;
	unsigned long pos = 0;

	for (pos = 0; pos < fw_len; pos += 4)
		sum += *(u32 *)(&fw_data[pos]);

	memcpy(fw_file_checksum, &sum, 4);

	return 0;
}

static void dbmdx_get_firmware_version(const char *data, size_t size,
				       char *buf, size_t buf_size)
{
	int i, j;

	buf[0] = 0;
	i = size - 58;
	if ((data[i]   == 0x10) && (data[i+1]  == 0x32) &&
			(data[i+2] == 0x1a) && (data[i+3]  == 0xd2)) {
		/* VQE FW */
		buf += snprintf(buf, buf_size,
				"Product %X%X%X%X Ver V%X.%X.%X%X%X%X.%X%X",
				/* PRODUCT */
				(int)(data[i+1]), (int)(data[i]),
				(int)(data[i+3]), (int)(data[i+2]),
				/* VERSION */
				(int)(data[i+5]), (int)(data[i+4]),
				(int)(data[i+7]), (int)(data[i+6]),
				(int)(data[i+9]), (int)(data[i+8]),
				(int)(data[i+11]), (int)(data[i+10]));

		snprintf(buf, buf_size,
				"Compiled at %c%c%c%c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c",
				/* DATE */
				(int)(data[i+12]), (int)(data[i+14]),
				(int)(data[i+16]), (int)(data[i+18]),
				(int)(data[i+20]), (int)(data[i+22]),
				(int)(data[i+24]), (int)(data[i+26]),
				(int)(data[i+28]), (int)(data[i+30]),
				(int)(data[i+32]),
				/* TIME */
				(int)(data[i+36]), (int)(data[i+38]),
				(int)(data[i+40]), (int) (data[i+42]),
				(int)(data[i+44]), (int)(data[i+46]),
				(int)(data[i+48]), (int)(data[i+50]));
	} else {
		/* VA FW */
		for (i = size - 13; i > 0; i--) {
			if ((data[i]   == 'v') && (data[i+2]  == 'e') &&
			    (data[i+4] == 'r') && (data[i+6]  == 's') &&
			    (data[i+8] == 'i') && (data[i+10] == 'o')) {
				for (j = 0; i + j < size; j++) {
					if (j == buf_size - 1)
						break;
					buf[j] = data[i];
					i += 2;
					if (((buf[j] > 0) && (buf[j] < 32))
					    || (buf[j] > 126))
						return;
					if (buf[j] == 0)
						buf[j] = ' ';
				}
				buf[j] = 0;
				return;
			}
		}
	}
}

void dbmdx_firmware_appear(const struct firmware *fw, void *context)
{
	struct dbmdx_private *p = context;

	if (!fw) {
		union fb_dspg_event event = {
			.event = FB_DSPG_EVENT_FW_REQUEST,
			.status = FB_DSPG_STAT_FAILED,
			.param = 0,
		};

		dev_err(p->dev, "%s: Request firmware failed: %s!\n",
			__func__, p->pdata->primary_firmware_name);

		dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);
		return;
	}

	release_firmware(fw);

	dev_info(p->dev, "%s: FW '%s' appears, start FW loading!\n",
		__func__, p->pdata->primary_firmware_name);

	/* Start FW loading. */
	dbmdx_schedule_delayed_work(p, &p->fw_reload_work, 0);
}

static int dbmdx_firmware_ready(struct dbmdx_private *p,
				const struct firmware *fw,
				const struct firmware *addon_fw[],
				unsigned int num_of_addons)
{
	char fw_version[200];
	size_t fw_size[MAX_NUM_OF_ADDONS + 1];
	const char *fw_data[MAX_NUM_OF_ADDONS + 1];
	u8 fw_checksum[(MAX_NUM_OF_ADDONS + 1) * 4];
	int i;

	int ret;

	if (!fw) {
		dev_err(p->dev, "%s: firmware request failed\n", __func__);
		return -EIO;
	}

	if (fw->size <= 4) {
		dev_err(p->dev, "%s: firmware size (%zu) invalid\n",
			__func__, fw->size);
		goto out_err;
	}
	if (p->cur_boot_options & DBMDX_BOOT_OPT_FW_CONTAINS_CHECKSUM) {
		memcpy(fw_checksum, &fw->data[fw->size - 4], 4);
		fw_size[0] = fw->size - 4;
	} else {
		dbmdx_calc_fw_checksum(p, fw->data, fw->size,
						&(fw_checksum[0]));
		fw_size[0] = fw->size;

	}

	fw_data[0] = fw->data;


	for (i = 0; i < num_of_addons; i++) {
		if (!addon_fw[i]) {
			dev_err(p->dev, "%s: addon %d is NULL\n", __func__, i);
			return -EIO;
		}
		if (addon_fw[i]->size <= 4) {
			dev_err(p->dev, "%s: firmware size (%zu) invalid\n",
				__func__, addon_fw[i]->size);
			goto out_err;
		}
		dbmdx_calc_fw_checksum(p, addon_fw[i]->data, addon_fw[i]->size,
					 &(fw_checksum[(i+1)*4]));
		fw_data[i+1] = addon_fw[i]->data;
		fw_size[i+1] = addon_fw[i]->size;
	}

	/*
	 *  read firmware version from file, not sure if this is the same
	 *  for Pr. Firmware
	 */
	memset(fw_version, 0, 200);
	dbmdx_get_firmware_version(fw->data, fw->size, fw_version, 200);
	if (strlen(fw_version) > 15)
		dev_info(p->dev, "%s: firmware: %s\n", __func__, fw_version);

	/* check if the chip interface is ready to boot */
	ret = p->chip->can_boot(p);
	if (ret)
		goto out_err;

	/* prepare boot if required */
	ret = p->chip->prepare_boot(p);
	if (ret)
		goto out_err;

	/* enable high speed clock for boot */
	p->clk_enable(p, DBMDX_CLK_MASTER);

	/* boot */
	ret = p->chip->boot(p, (void *)fw_data, fw_size, (void *)fw_checksum,
						num_of_addons + 1, 4, 1);
	if (ret)
		goto out_disable_hs_clk;

	/* disable high speed clock after boot */
	p->clk_disable(p, DBMDX_CLK_MASTER);

	/* finish boot if required */
	ret = p->chip->finish_boot(p);
	if (ret)
		goto out_err;

	ret = 0;
	goto out;

out_disable_hs_clk:
	p->clk_disable(p, DBMDX_CLK_MASTER);
out_err:
	dev_err(p->dev, "%s: firmware request failed\n", __func__);
	ret = -EIO;
out:
	return ret;
}

static int dbmdx_config_initial_settings(struct dbmdx_private *p)
{
	int ret;
	u32 fwver = 0xffffffff;
	u16 val = 0;
	u16 addon_ver = 0xffff;
	u16 algo_ver = 0xffff;
	u32 ack_val = 0;

	dev_dbg(p->dev, "%s\n", __func__);

	p->chip->transport_enable(p, true);

	p->device_ready = true;

	dbmdx_write_register16(p, PRIORITY_RUN_ON_IDLE_ISR, MSG_ID_FW,
				REG_FW__HOST_WAKEUP_GPIO,
				p->pdata->rx_ready_host_wakeup_gpio);

	usleep_range(5000, 6000);

	ret = dbmdx_check_if_alive(p);

	if (ret < 0) {
		dev_err(p->dev, "%s: FW is Dead\n", __func__);
		goto out_err;
	}

	ret = dbmdx_send_init_cfg_reg_list(p);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error sending initial configuration reglist\n",
				__func__);
		goto out_err;
	}

	ret = dbmdx_check_if_alive(p);

	if (ret < 0) {
		dev_err(p->dev, "%s: FW is Dead\n", __func__);
		goto out_err;
	}

	/* read firmware version */
	ret = dbmdx_read_register32(p, MSG_ID_FW,
					REG_FW__VERSION_NUMBER, &fwver);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read firmware version\n",
				__func__);
		goto out_err;
	}

	/* read Addon Version */
	ret = dbmdx_read_register16(p, MSG_ID_ADDON_FB,
					0, &addon_ver);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read firmware version\n",
				__func__);
		goto out_err;
	}

	/* read Algorithm Version */
	ret = dbmdx_read_register16(p, MSG_ID_ADDON_FB,
					1, &algo_ver);
	if (ret < 0) {
		dev_err(p->dev, "%s: could not read firmware version\n",
				__func__);
		goto out_err;
	}
	dev_info(p->dev,
			"%s: Pr. Firmware 0x%x Addon 0x%x Algo 0x%x ready\n",
			__func__, fwver, addon_ver, algo_ver);

	if (p->hw_usecase_stop_enabled) {
		/* read firmware version */
		ret = dbmdx_read_register16(p, MSG_ID_FW,
					REG_FW__HOST_IF_CONFIG, &val);
		if (ret < 0) {
			dev_err(p->dev, "%s: could not read HOST IF register\n",
				__func__);
			goto out_err;
		}
		/* Disable UART 0 */
		val = val & 0xFE;

		ret = dbmdx_write_register16_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, REG_FW__HOST_IF_CONFIG, val,
					&ack_val);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error configuring HOST IF register\n",
					__func__);
			goto out_err;
		}
		/* Configure user-defined ISR for AGPIO18 */
		ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, 0x60, 0, 0x2000, &ack_val);
		if (ret < 0) {
			dev_err(p->dev,
			"%s: Error configuring user defined ISR on AGPIO18\n",
				__func__);
			goto out_err;
		}
	}

	return 0;

out_err:
	p->device_ready = false;
	return ret;
}

static int spi_verify_mem_pattern(struct dbmdx_private *p)
{
	static const u8 data_read[] = {0xDB, 0xD7, 0x00, 0x04};
	u8 return_pattern[9] = {0};
	u8 pattern = (u8)p->pdata->memset_params[3];
	u8 expected_pattern[4] = {pattern, pattern, pattern, pattern};
	int ret = 0;

	ret = p->chip->write(p, data_read, sizeof(data_read));
	if (ret != sizeof(data_read)) {
		dev_err(p->dev, "%s: Error sending data read command\n",
			__func__);
		return -EIO;
	}
	ret = p->chip->write(p, &p->pdata->memset_params[1],
			     sizeof(p->pdata->memset_params[1]));
	if (ret != sizeof(p->pdata->memset_params[1])) {
		dev_err(p->dev, "%s: Error sending data read address\n",
			__func__);
		return -EIO;
	}

	usleep_range(DBMDX_USLEEP_SPI_CMD_AFTER_BOOT,
			DBMDX_USLEEP_SPI_CMD_AFTER_BOOT + 1000);
	ret = p->chip->read(p, return_pattern, 9);
	if (ret != 9) {
		dev_err(p->dev, "%s Error Reading Data\n",
			__func__);
		return -EIO;
	}
	if (memcmp(&(return_pattern[1]), data_read, 4) != 0) {
		dev_err(p->dev,
			"%s:Wrong Memory pattern resp: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
				__func__,
				return_pattern[1],
				return_pattern[2],
				return_pattern[3],
				return_pattern[4],
				return_pattern[5],
				return_pattern[6],
				return_pattern[7],
				return_pattern[8]);
		return -EIO;
	}
	if (memcmp(&(return_pattern[5]), expected_pattern, 4) != 0) {
		dev_err(p->dev,
			"%s: Wrong Memory pattern Received 0x%02x%02x%02x%02x Expected: 0x%02x%02x%02x%02x\n",
				__func__,
				return_pattern[5],
				return_pattern[6],
				return_pattern[7],
				return_pattern[8],
				expected_pattern[0],
				expected_pattern[1],
				expected_pattern[2],
				expected_pattern[3]);
		return -EILSEQ;
	}

	dev_info(p->dev,
		"%s: Memory pattern: 0x%02x verified at address:0x%04x\n",
			__func__, pattern, p->pdata->memset_params[1]);
	return 0;
}

static int dbmdx_primary_firmware_ready(struct dbmdx_private *p)
{
	int ret;
	u8 init_lock_cmd[] = {0x5A};

	dev_dbg(p->dev, "%s\n", __func__);

	p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	/* Boot Chip */
	if (p->pdata->boot_options & DBMDX_BOOT_OPT_SEND_PREBOOT) {

		ret = dbmdx_switch_to_chip_interface(p,
			DBMDX_PREBOOT_INTERFACE);
		if (ret) {
			dev_err(p->dev,
				"%s Error switching to Pr. interface\n",
				__func__);
			return ret;
		}

		dbmdx_reset_sequence(p);

		msleep(DBMDX_MSLEEP_AFTER_RESET_32K);

		p->chip->prepare_preboot(p);

		ret = p->chip->write(p, init_lock_cmd,
			sizeof(init_lock_cmd));

		if (ret != sizeof(init_lock_cmd)) {
			dev_err(p->dev,
				"%s Error sending the Init lock cmd\n",
				__func__);
			return -EIO;
		}

		/* preboot */
		ret = p->chip->write(p, p->primary_preboot_fw->data,
			p->primary_preboot_fw->size);

		if (ret != p->primary_preboot_fw->size) {
			dev_err(p->dev,
				"%s Error sending the Preboot FW\n",
				__func__);
			return -EIO;
		}

		dev_err(p->dev, "%s Preboot was sent successfully\n",
			__func__);
		usleep_range(DBMDX_USLEEP_SPI_D7_AFTER_BOOT,
			     DBMDX_USLEEP_SPI_D7_AFTER_BOOT + 5000);

		ret = p->chip->write(p, &p->pdata->memset_params[0],
				     (MEMSET_CMDS_SIZE * 4));

		if (ret != (MEMSET_CMDS_SIZE * 4)) {
			dev_err(p->dev, "%s Error sending memset param commands\n",
				__func__);
			return -EIO;
		}

		msleep(DBMDX_MSLEEP_MEMSET);
		ret = spi_verify_mem_pattern(p);
		if (ret) {
			dev_err(p->dev, "%s Memory pattern verification failed\n",
				__func__);
			return -EIO;
		}
	}

	ret = dbmdx_switch_to_chip_interface(p, DBMDX_BOOT_INTERFACE);
	if (ret) {
		dev_err(p->dev,
			"%s Error switching to BOOT interface\n",
			__func__);
		p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;
		return ret;
	}

	/* common boot */
	ret = dbmdx_firmware_ready(p, p->primary_fw, p->addon_fw,
						p->num_of_loaded_fw_addons);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not load Pr. Firmware\n", __func__);
		return -EIO;
	}

	p->boot_mode = DBMDX_BOOT_MODE_NORMAL_BOOT;

	dbmdx_set_primary_active(p);

	ret = dbmdx_switch_to_chip_interface(p, DBMDX_CMD_INTERFACE);
	if (ret) {
		dev_err(p->dev,
			"%s Error switching to CMD interface\n",
			__func__);
		return ret;
	}

	ret = dbmdx_config_initial_settings(p);
	if (ret != 0) {
		dev_err(p->dev, "%s: could not configure Pr. Firmware\n",
			__func__);
		return -EIO;
	}

	ret = p->chip->set_primary_firmware_ready(p);
	if (ret) {
		dev_err(p->dev, "%s: could not set to ready Pr. Firmware\n",
			__func__);
		return -EIO;
	}

	return 0;
}

static int dbmdx_switch_to_primary_firmware(struct dbmdx_private *p,
								bool do_reset)
{
	int ret;

	if (p->active_fw == DBMDX_FW_PRIMARY)
		return 0;

	if (do_reset)
		dbmdx_set_boot_active(p);

	dev_dbg(p->dev, "%s: switching to Pr. Firmware\n", __func__);

	p->device_ready = false;

	ret = dbmdx_primary_firmware_ready(p);
	if (ret)
		return ret;

	p->device_ready = true;

	return 0;
}

static int dbmdx_set_platform_id(struct dbmdx_private *p)
{
	unsigned long timeout;
	u32 platform_id = -1;
	u32 ack_val;
	int ret = 0;

	if (p->pdata->platform_id == -1) {
		dev_info(p->dev, "Platform ID is undefined\n");
		return ret;
	}
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
			MSG_ID_ADDON_FB, SET_PLATFORM_ID,
			p->pdata->platform_id, &ack_val);
		if (ret)
			continue;

		ret = dbmdx_read_register32(p, MSG_ID_ADDON_FB,
			SET_PLATFORM_ID, &platform_id);
		if (!ret) {
			if (platform_id == p->pdata->platform_id)
				break;
		}
		msleep(5);
	} while (time_before_eq(jiffies, timeout));

	if (!time_before_eq(jiffies, timeout)) {
		dev_err(p->dev, "Failed to set Platform ID\n");
		return -ETIMEDOUT;
	}

	return ret;
}

int dbmdx_request_and_load_fw(struct dbmdx_private *p)
{
	int ret;
	int i = 0;

	dev_dbg(p->dev, "%s %s\n", __func__, "Primary");

	/* p->lock(p); */

	dbmdx_set_boot_active(p);

	if (p->primary_fw) {
		release_firmware(p->primary_fw);
		p->primary_fw = NULL;
	}

	if (p->primary_preboot_fw) {
		release_firmware(p->primary_preboot_fw);
		p->primary_preboot_fw = NULL;
	}

	for (i = 0; i < p->num_of_loaded_fw_addons; i++) {
		if (p->addon_fw[i]) {
			release_firmware(p->addon_fw[i]);
			p->addon_fw[i] = NULL;
		}
	}

	p->num_of_loaded_fw_addons = 0;

	dev_info(p->dev, "%s: request Pr. Firmware - %s\n",
			 __func__, p->pdata->primary_firmware_name);
	ret = request_firmware((const struct firmware **)&p->primary_fw,
				       p->pdata->primary_firmware_name,
				       p->dev);
	if (ret != 0) {
		dev_err(p->dev,	"%s: failed to request Pr. Firmware\n",
				__func__);
		goto out;
	}

	/* Here we can assume that FS is ready ,so no retries */
	if (p->pdata->boot_options & DBMDX_BOOT_OPT_SEND_PREBOOT) {
		dev_info(p->dev,
			"%s: request primary preboot firmware - %s\n",
			__func__,
			p->pdata->primary_preboot_firmware_name);

		ret = request_firmware(
			(const struct firmware **)&p->primary_preboot_fw,
				p->pdata->primary_preboot_firmware_name,
				p->dev);

		if (ret != 0) {
			dev_err(p->dev,
				"%s: failed to request pr. preboot fw\n",
				__func__);
			goto out_err;
		}
	}

	for (i = 0; i < p->pdata->num_of_fw_addons; i++) {
		dev_info(p->dev, "%s: request fw addon %d: %s\n", __func__,
			i, p->pdata->add_on_fw_names[i]);

		ret = request_firmware(
			(const struct firmware **)&(p->addon_fw[i]),
				p->pdata->add_on_fw_names[i],
				p->dev);

		if (ret != 0) {
			dev_err(p->dev,
				"%s: failed to request fw addon\n",
				__func__);
			goto out_err;
		}
		p->num_of_loaded_fw_addons++;
	}

	ret = dbmdx_switch_to_chip_interface(p, DBMDX_BOOT_INTERFACE);
	if (ret) {
		dev_err(p->dev,	"%s Error switching to Pr. BOOT interface\n",
				__func__);
		ret = -EIO;
		goto out_err;
	}

	reinit_completion(&p->fw_load_complete);

	ret = dbmdx_switch_to_primary_firmware(p, 1);
	if (ret != 0) {
		dev_err(p->dev, "%s: failed to boot Pr. Firmware\n",
				__func__);
		ret = -EIO;
		goto out_err;
	}

	p->device_ready = true;

	ret = p->do_fw_compl ? wait_for_completion_timeout(&p->fw_load_complete,
		msecs_to_jiffies(FW_START_TIMEOUT)) : 1;
	if (!ret)
		return -ETIMEDOUT;
	/* send platform id from here and read and confirm platform is set */
	ret = dbmdx_set_platform_id(p);

	goto out;

out_err:
	if (p->primary_fw) {
		release_firmware(p->primary_fw);
		p->primary_fw = NULL;
	}

	if (p->primary_preboot_fw) {
		release_firmware(p->primary_preboot_fw);
		p->primary_preboot_fw = NULL;
	}

	for (i = 0; i < p->num_of_loaded_fw_addons; i++) {
		if (p->addon_fw[i]) {
			release_firmware(p->addon_fw[i]);
			p->addon_fw[i] = NULL;
		}
	}

out:
	/* p->unlock(p); */
	return ret;
}

static int dbmdx_process_host_wakeup_irq(struct dbmdx_private *p)
{
	int ret = 0;

	/* Remember task used for wake-up interrupt */
	p->task_irq = current;

	ret = dbmdx_process_pending_messages(p);
	if (ret < 0) {
		dev_warn(p->dev,
			"%s: Error during processing pending messages\n",
			__func__);
	}

	return ret;
}

void dbmdx_remote_register_event_callback(event_cb func)
{
	if (dbmdx_data)
		dbmdx_data->event_callback = func;
	else
		g_event_callback = func;
}
EXPORT_SYMBOL(dbmdx_remote_register_event_callback);

static irqreturn_t dbmdx_rx_interrupt_hard(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;

	if (p && (p->device_ready) && (p->primary_flags.rx_irq_inuse))

		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}


static irqreturn_t dbmdx_rx_interrupt_soft(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;


	if ((p->device_ready) && (p->primary_flags.rx_irq_inuse)) {
		if (!atomic_add_unless(&(p->primary_flags.rx_ack), 1, 1)) {
			dev_err(p->dev, "%s RX IRQ when ACK is set!!!\n",
				__func__);
		} else {
#ifdef DBMDX_DEBUG_IRQ
			dev_dbg(p->dev, "%s wake_up for RX Ack\n", __func__);
#endif
			wake_up(&p->rx_ack_wq);
		}
	} else {
		dev_warn(p->dev, "%s RX IRQ is not in use\n", __func__);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dbmdx_host_wakeup_interrupt_hard(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;

	if (p && (p->device_ready))

		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}


static irqreturn_t dbmdx_host_wakeup_interrupt_soft(int irq, void *dev)
{
	struct dbmdx_private *p = (struct dbmdx_private *)dev;
	int ret;

	dev_dbg(p->dev, "%s:\n", __func__);

	if ((p->device_ready)) {
		ret = dbmdx_process_host_wakeup_irq(p);
		if (ret < 0)
			dev_err(p->dev,
			"%s: Error  during processing host wakeup IRQ\n",
			__func__);
	} else {
		dev_warn(p->dev, "%s Device is not ready\n", __func__);
	}

	return IRQ_HANDLED;
}

void dbmdx_send_uevent(struct dbmdx_private *p, char *buf)
{
	char * const envp[] = { buf, NULL };
	int ret = 0;

	dev_info(p->dev, "%s: Sending uevent: %s\n",
		__func__, buf);

	ret = kobject_uevent_env(&p->dev->kobj, KOBJ_CHANGE,
		(char **)envp);

	if (ret)
		dev_err(p->dev,	"%s: Sending uevent failed %d\n",
			__func__, ret);
}
EXPORT_SYMBOL(dbmdx_send_uevent);

static int dbmdx_fw_crash(struct dbmdx_private *p)
{
	if (delayed_work_pending(&p->fw_reload_work)) {
		dev_info(p->dev, "FW Reload is currently running!\n");
		return -EBUSY;
	}

	dbmdx_schedule_delayed_work(p, &p->fw_reload_work, 0);

	return 0;
}

static void dbmdx_check_fw_state(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(work, struct dbmdx_private,
					       fw_state_work.work);
	int ret = 0;

	/* Exit if fw_reload_work is scheduled or currently being reloaded */
	if (delayed_work_pending(&p->fw_reload_work) || !p->fw_alive)
		return;

	if (p->do_suspend)
		return;

	ret = dbmdx_check_if_alive(p);
	if (!ret)
		dbmdx_schedule_delayed_work(p, &p->fw_state_work,
					    msecs_to_jiffies(1000));
	else
		dbmdx_fw_crash(p);
}

static void dbmdx_fw_reload(struct work_struct *work)
{
	struct dbmdx_private *p = container_of(work, struct dbmdx_private,
					       fw_reload_work.work);
	static int retry;
	int ret = 0;

	if (p && p->device_ready && p->primary_flags.active_usecase) {
		bool is_uc_active = false;
		struct usecase_config *active_usecase =
					p->primary_flags.active_usecase;
		active_usecase->usecase_is_active(p,
					active_usecase, &is_uc_active);
		if (!ret && is_uc_active) {
			ret = active_usecase->usecase_exit(p, active_usecase);
			if (ret < 0)
				dev_err(p->dev,
					"%s: Error exiting usecase %s\n",
					__func__,
					active_usecase->usecase_name);
		}
		p->primary_flags.active_usecase = NULL;

	}
	p->wakeup_release(p);

	p->debug_mode = 0;

	/* Send uvent for FW Dead if required and clear internal flag */
	if (p->fw_alive) {
		union fb_dspg_event event = {
			.event = FB_DSPG_EVENT_FW_CRASH,
			.status = FB_DSPG_STAT_SUCCESS,
			.param = 0,
		};

		p->fw_alive = false;
		p->fw_crash = true;

		/* Hold system awake until FW load */
		pm_stay_awake(p->dev);

		dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);
		dbmdx_notifier_call(p->dev, DBMDX_EV_CRASH, NULL);
		dbmdx_send_uevent(p, "FIRMWARE_STATE=DSPG_FW_DEAD");
		trace_fba_fw_crash("firmware dead");

		device_for_each_child(p->dev, p, dbmdx_cell_force_suspend);
		pm_runtime_force_suspend(p->dev);
	}

	p->device_ready = false;

	p->task_workq = current;
	ret = dbmdx_request_and_load_fw(p);

	if (ret && retry++ <= RETRY_COUNT) {
		p->task_workq = NULL;
		dbmdx_schedule_delayed_work(p, &p->fw_reload_work,
						    msecs_to_jiffies(1000));
		return;
	} else if (ret) {
		union fb_dspg_event event = {
			.event = FB_DSPG_EVENT_FW_RECOVERY,
			.status = FB_DSPG_STAT_FAILED,
			.param = retry,
		};

		dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);
		trace_fba_fw_crash("recovery fail");

		p->task_workq = NULL;

		WARN(1, "%s: Failed %d attempts to load firmware!!!",
			__func__, retry);
		return;
	}

	retry = 0;
	p->fw_alive = true;
	p->task_workq = NULL;

	if (p->fw_crash) {
		pm_runtime_force_resume(p->dev);
		device_for_each_child(p->dev, p, dbmdx_cell_force_resume);
	}

	if (p->fw_crash) {
		union fb_dspg_event event = {
			.event = FB_DSPG_EVENT_FW_RECOVERY,
			.status = FB_DSPG_STAT_SUCCESS,
			.param = retry,

		};

		dbmdx_event_log_int(p->dev, FBE_DBMDX, event.value);
		trace_fba_fw_crash("recovery success");
	}
	p->disable_log_event = 0;
	if (p->fw_crash)
		dbmdx_notifier_call(p->dev, DBMDX_EV_RECOVERY, NULL);
	else
		dbmdx_notifier_call(p->dev, DBMDX_EV_STARTUP, NULL);

	dbmdx_send_uevent(p, "FIRMWARE_STATE=DSPG_FW_ALIVE");

	dbmdx_schedule_delayed_work(p, &p->fw_state_work,
				    msecs_to_jiffies(5000));

	/* Relase system to allow enter in suspend */
	pm_relax(p->dev);

	p->fw_crash = false;

	/* Detect first successfully FW load and add all child devices */
	if (!p->mfd_devices) {
		p->mfd_devices = true;

		ret = mfd_add_devices(p->dev, PLATFORM_DEVID_NONE, dbmdx_cells,
				      ARRAY_SIZE(dbmdx_cells), NULL, 0, NULL);
		if (ret != 0)
			dev_err(p->dev, "fail to register client devices\n");
	}
}

static int dbmdx_core_msg_handler(struct dbmdx_private *p,
	struct fw_message *msg)
{
	struct device *dev = p->dev;
	u8 id = msg->header.id;
	u8 reg = msg->header.reg_num;
	u32 info0 = msg->reg_info[0];

	if (id == MSG_ID_FW && reg == REG_FW__ERROR_REPORT) {
		switch (info0) {
		case FW_ERROR__UART1_BUFF_FULL:
		case FW_ERROR__REC_MAILBOX_TOO_SMALL:
		case FW_ERROR__PRINTF_MB_ERROR1:
		case FW_ERROR__PRINTF_MB_SIZE_LIMIT:
		case ERROR_CODE__IH_MSG_TOO_LONG:
		case ERROR_CODE__BAD_INTERFACE_NUM1:
		case MB_ERROR__SET_RECORDING:
		case MB_ERROR__READ_UNDERRUN:
		case MB_ERROR__BUFFER_OVF_B_CLI0:
		case ADDON_ERROR_BAD_CHANNEL_ORDER:
		case ADDON_ERROR_AWB_NOT_LOADED:

			dev_warn(dev, "%s: FW Error 0x%04x. No Reload Required\n",
				__func__, info0);
			break;
		default:
			dev_err(dev, "%s: FW Error 0x%04x. Reloading\n",
				__func__, info0);
			dbmdx_fw_crash(p);
			break;
		}
	} else if (id == MSG_ID_ADDON_FB) {
		switch (reg) {
		case FB_ADDON_ALGO_ENABLED:
			p->algo_enabled = true;
			dbmdx_send_uevent(p,
				"FIRMWARE_STATE=DSPG_ALGO_ENABLED");
			break;
		case FB_ADDON_ALGO_DISABLED:
			p->algo_enabled = false;
			dbmdx_send_uevent(p,
				"FIRMWARE_STATE=DSPG_ALGO_DISABLED");
			break;
		}
	}

	return 0;
}

static int dbmdx_core_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct dbmdx_private *p = container_of(nb, typeof(*p), nb_core);

	switch (action) {
	case DBMDX_EV_MESSAGE:
		dbmdx_core_msg_handler(p, data);
		break;
	}

	return 0;
}

bool dbmdx_core_is_ready(struct dbmdx_private *p)
{
	return p->device_ready;
}
EXPORT_SYMBOL(dbmdx_core_is_ready);

int dbmdx_notifier_register(struct device *dev, struct notifier_block *nb)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	ret = srcu_notifier_chain_register(&p->notifier_list, nb);
	return ret;
}
EXPORT_SYMBOL(dbmdx_notifier_register);

int dbmdx_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	ret = srcu_notifier_chain_unregister(&p->notifier_list, nb);
	return ret;
}
EXPORT_SYMBOL(dbmdx_notifier_unregister);

int dbmdx_notifier_call(struct device *dev, unsigned long val, void *v)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return srcu_notifier_call_chain(&p->notifier_list, val, v);
}
EXPORT_SYMBOL(dbmdx_notifier_call);

int dbmdx_core_suspend(struct dbmdx_private *p)
{
	p->do_suspend = true;

	/* Stop and wait until complete FW check thread */
	cancel_delayed_work_sync(&p->fw_state_work);

	return 0;
}

int dbmdx_core_resume(struct dbmdx_private *p)
{
	p->do_suspend = false;

	/* Restart FW check thread */
	dbmdx_schedule_delayed_work(p, &p->fw_state_work,
				    msecs_to_jiffies(5000));

	return 0;
}

static int dbmdx_common_probe(struct dbmdx_private *p)
{
	int ret = 0;
	int fclk;

	dbmdx_data = p;
	dev_set_drvdata(p->dev, p);

	p->do_fw_compl = of_property_read_bool(p->dev->of_node, "fw_complete");
	init_completion(&p->fw_load_complete);

	srcu_init_notifier_head(&p->notifier_list);

	/* Register core notifier callback */
	p->nb_core.notifier_call = dbmdx_core_notify;
	dbmdx_notifier_register(p->dev, &p->nb_core);

	/* enable constant clock */
	p->clk_enable(p, DBMDX_CLK_CONSTANT);

	/* enable regulator if defined */
	if (p->vregulator) {
		ret = regulator_enable(p->vregulator);
		if (ret != 0) {
			dev_err(p->dev, "%s: Failed to enable regulator: %d\n",
				__func__, ret);
			goto err_clk_off;
		}
	}

	if (!gpio_is_valid(p->pdata->gpio_reset)) {
		dev_err(p->dev, "%s: reset gpio invalid\n", __func__);
		ret = -EINVAL;
		goto err_clk_off;
	}

	ret = gpio_request(p->pdata->gpio_reset, "DBMDX reset");
	if (ret < 0) {
		dev_err(p->dev, "%s: error requesting reset gpio\n", __func__);
		goto err_clk_off;
	}
	gpio_direction_output(p->pdata->gpio_reset, 0);
	gpio_set_value(p->pdata->gpio_reset, 0);

	if (!gpio_is_valid(p->pdata->gpio_host_wakeup)) {
		p->pdata->gpio_host_wakeup = -1;
		p->host_wakeup_irq = -1;
		dev_err(p->dev, "%s: host-wakeup gpio not available\n",
			__func__);
	} else {
		ret = gpio_request(p->pdata->gpio_host_wakeup, "DBMDX hw");
		if (ret < 0) {
			dev_err(p->dev,
				"%s: error requesting host wakeup gpio\n",
				__func__);
			goto err_gpio_free;
		}
		gpio_direction_input(p->pdata->gpio_host_wakeup);

		/* interrupt gpio */
		p->host_wakeup_irq = ret =
			gpio_to_irq(p->pdata->gpio_host_wakeup);
		if (ret < 0) {
			dev_err(p->dev, "%s: cannot map gpio to irq\n",
				__func__);
			goto err_gpio_free;
		}
	}

	if (!gpio_is_valid(p->pdata->gpio_rx_ready)) {
		p->pdata->gpio_rx_ready = -1;
		p->rx_irq = -1;
		dev_err(p->dev, "%s: rx-ready gpio not available\n", __func__);
	} else {
		ret = gpio_request(p->pdata->gpio_rx_ready, "DBMDX rx ready");
		if (ret < 0) {
			dev_err(p->dev, "%s: error requesting rx ready gpio\n",
				__func__);
			goto err_gpio_free;
		}
		gpio_direction_input(p->pdata->gpio_rx_ready);

		/* interrupt gpio */
		p->rx_irq = ret = gpio_to_irq(p->pdata->gpio_rx_ready);
		if (ret < 0) {
			dev_err(p->dev, "%s: cannot map gpio to irq\n",
				__func__);
			goto err_gpio_free;
		}
	}

	if (!gpio_is_valid(p->pdata->gpio_wakeup)) {
		dev_info(p->dev, "%s: wakeup gpio not specified\n", __func__);
		p->pdata->gpio_wakeup = -1;
	} else {
		ret = gpio_request(p->pdata->gpio_wakeup, "DBMDX wakeup");
		if (ret < 0) {
			dev_err(p->dev, "%s: error requesting wakeup gpio\n",
				__func__);
			goto err_gpio_free;
		}
		/* keep the wakeup pin high */
		gpio_direction_output(p->pdata->gpio_wakeup, 1);
	}

	if (p->rx_irq != -1) {
		ret = request_threaded_irq(p->rx_irq,
					  dbmdx_rx_interrupt_hard,
					  dbmdx_rx_interrupt_soft,
					  IRQF_TRIGGER_RISING,
					  "dbmdx_rx", p);

		if (ret < 0) {
			dev_err(p->dev, "%s: cannot get rx irq\n", __func__);
			goto err_gpio_free;
		}

	}

	if (p->host_wakeup_irq != -1) {
		ret = request_threaded_irq(p->host_wakeup_irq,
					  dbmdx_host_wakeup_interrupt_hard,
					  dbmdx_host_wakeup_interrupt_soft,
					  IRQF_TRIGGER_RISING,
					  "dbmdx_host_wakeup", p);

		if (ret < 0) {
			dev_err(p->dev, "%s: cannot get host_wakeup irq\n",
				__func__);
			goto err_gpio_free;
		}

	}

	init_waitqueue_head(&p->rx_ack_wq);
	init_waitqueue_head(&p->host_event_wq);

	/* lock init */
	mutex_init(&p->p_lock);

	/* set clock rates */
	if (p->clk_set_rate(p, DBMDX_CLK_MASTER) < 0) {
		dev_err(p->dev,
			"%s: could not set rate for master clock\n",
			__func__);
		goto err_gpio_free;
	}

	if (p->clk_set_rate(p, DBMDX_CLK_CONSTANT) < 0) {
		dev_err(p->dev,
			"%s: could not set rate for constant clock\n",
			__func__);
		goto err_gpio_free;
	}

	fclk = p->clk_get_rate(p, DBMDX_CLK_MASTER);

	p->ns_class = class_create(THIS_MODULE, "voicep");
	if (IS_ERR(p->ns_class)) {
		dev_err(p->dev, "%s: failed to create class\n", __func__);
		goto err_gpio_free;
	}

	p->dbmdx_dev = device_create(p->ns_class, NULL, 0, p, "dbmdx");
	if (IS_ERR(p->dbmdx_dev)) {
		dev_err(p->dev, "%s: could not create device\n", __func__);
		goto err_class_destroy;
	}

	ret = dbmdx_init_common_sysfs_group(p);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to create sysfs group\n",
			__func__);
		goto err_device_unregister;
	}

	p->cur_reset_gpio = p->pdata->gpio_reset;
	p->cur_wakeup_gpio = p->pdata->gpio_wakeup;

	p->fw_cmd_buffer = vmalloc(DBMDX_MAX_FW_CMD_LENGTH);
	if (!p->fw_cmd_buffer) {
		dev_err(p->dbmdx_dev,
			"%s: failed to allocate memory for cmd buffer\n",
			__func__);
		goto err_sysfs_remove_group;
	}

	ret = device_init_wakeup(p->dev, true);
	if (ret)
		goto err_no_wakeup_wource;

	/* Hold system awake until FW load */
	pm_stay_awake(p->dev);

	/* Waiting for file system and firmware appeared for the first time. */
	ret = request_firmware_nowait(THIS_MODULE, true,
		p->pdata->primary_firmware_name, p->dev,
		GFP_KERNEL, p, dbmdx_firmware_appear);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to request fw: %s\n",
			__func__, p->pdata->primary_firmware_name);
		goto err_free_resources;
	}

	dev_info(p->dev, "%s: registered DBMDX codec driver version = %s\n",
		__func__, DRIVER_VERSION);

	return 0;

err_free_resources:
err_no_wakeup_wource:
	if (p->fw_cmd_buffer)
		vfree(p->fw_cmd_buffer);
err_sysfs_remove_group:
	dbmdx_remove_common_sysfs_group(p);
err_device_unregister:
	device_unregister(p->dbmdx_dev);
err_class_destroy:
	class_destroy(p->ns_class);
err_gpio_free:
	if (p->pdata->gpio_wakeup >= 0)
		gpio_free(p->pdata->gpio_wakeup);
	if (p->pdata->gpio_host_wakeup >= 0)
		gpio_free(p->pdata->gpio_host_wakeup);
	if (p->pdata->gpio_rx_ready >= 0)
		gpio_free(p->pdata->gpio_rx_ready);
	if (p->pdata->gpio_reset >= 0)
		gpio_free(p->pdata->gpio_reset);
err_clk_off:
	/* disable constant clock */
	p->clk_disable(p, DBMDX_CLK_CONSTANT);
	return ret;
}

static void dbmdx_common_remove(struct dbmdx_private *p)
{
	int i;

	flush_workqueue(p->dbmdx_workq);
	destroy_workqueue(p->dbmdx_workq);
	dbmdx_remove_common_sysfs_group(p);
	device_unregister(p->dbmdx_dev);
	class_destroy(p->ns_class);
	if (p->pdata->gpio_wakeup >= 0)
		gpio_free(p->pdata->gpio_wakeup);
	gpio_free(p->pdata->gpio_reset);
	if (p->pdata->gpio_host_wakeup >= 0)
		gpio_free(p->pdata->gpio_host_wakeup);
	if (p->pdata->gpio_rx_ready >= 0)
		gpio_free(p->pdata->gpio_rx_ready);
	/* disable constant clock */
	p->clk_disable(p, DBMDX_CLK_CONSTANT);
	for (i = 0; i < DBMDX_NR_OF_CLKS; i++)
		if (p->clocks[i])
			clk_put(p->clocks[i]);

	if (p->fw_cmd_buffer)
		vfree(p->fw_cmd_buffer);

	device_init_wakeup(p->dev, false);
	mfd_remove_devices(p->dev);
	kfree(p);
}

#ifndef CONFIG_OF

static int verify_platform_data(struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct device *dev = p->dev;
	int i;

	if (pdata->multi_interface_support > 1)
		pdata->multi_interface_support = 1;

	if (pdata->multi_interface_support)
		dev_info(dev, "%s: Multi Interface Probe is supported\n",
			__func__);
	else
		dev_info(dev, "%s: Multi Interface Probe is not supported\n",
			__func__);

	dev_info(dev, "%s: Pr. Firmware name: %s\n",
				 __func__, pdata->primary_firmware_name);

	dev_info(dev, "%s: Pr. preboot firmware name: %s\n",
				 __func__,
				 pdata->primary_preboot_firmware_name);

	for (i = 0; i < pdata->num_of_fw_addons; i++)
		dev_info(dev, "%s: FW Addon Name %d:\t%s\n",
				__func__, i, pdata->add_on_fw_names);

	for (i = 0; i < DBMDX_NR_OF_SPEEDS; i++)
		dev_dbg(dev, "%s: Host speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			__func__,
			i,
			pdata->host_speed_cfg[i].cfg,
			pdata->host_speed_cfg[i].uart_baud,
			pdata->host_speed_cfg[i].i2c_rate,
			pdata->host_speed_cfg[i].spi_rate);

	if (pdata->wakeup_disabled > 1)
		pdata->wakeup_disabled = 1;

	dev_info(dev, "%s: using wakeup_disabled of %d\n",
		 __func__, pdata->wakeup_disabled);


	if (pdata->use_gpio_for_wakeup > 1)
		pdata->use_gpio_for_wakeup = 1;

	dev_info(dev, "%s: using use_gpio_for_wakeup of %d\n",
		 __func__, pdata->use_gpio_for_wakeup);

	if (pdata->send_wakeup_seq > 1)
		pdata->send_wakeup_seq = 1;

	dev_info(dev, "%s: using send_wakeup_seq of %d\n",
		 __func__, pdata->send_wakeup_seq);

	if (pdata->wakeup_set_value > 1)
		pdata->wakeup_set_value = 1;

	dev_info(dev, "%s: using wakeup_set_value of %d\n",
		 __func__, pdata->wakeup_set_value);

	for (i = 0; i < pdata->firmware_id_num; i++)
		dev_info(dev, "%s: using firmware_id %d: of 0x%8x\n",
			__func__, i, pdata->firmware_id[i]);

	dev_info(dev, "%s: using boot_options of 0x%8x\n",
		 __func__, pdata->boot_options);

	dev_info(dev, "%s: using amodel_options of 0x%8x\n",
		 __func__, pdata->amodel_options);

	if (pdata->recovery_disabled != 0 &&
		pdata->recovery_disabled != 1)
		pdata->recovery_disabled = 0;

	dev_info(dev,
		"%s: using recovery_disabled of %d\n",
		__func__, pdata->recovery_disabled);

	if (pdata->uart_low_speed_enabled != 0 &&
		pdata->uart_low_speed_enabled != 1)
		pdata->uart_low_speed_enabled = 0;

	dev_info(p->dev,
		"%s: using uart_low_speed_enabled of %d\n",
		 __func__, pdata->uart_low_speed_enabled);

	if (dbmdx_platform_get_clk_info(p, DBMDX_CLK_MASTER)) {
		dev_err(dev,
			"%s: failed to get master clock information\n",
			 __func__);
	}

	if (dbmdx_platform_get_clk_info(p, DBMDX_CLK_CONSTANT)) {
		dev_err(dev,
			"%s: failed to get constant clock information\n",
			 __func__);
	}


	return 0;
}

static int dbmdx_init_platform_data(struct dbmdx_private *p)
{
	int ret = 0;

	pdata = dev_get_platdata(p->dev);

	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s: Failed to get platform data\n",
			__func__);
		return -ENODEV;
	}

	p->pdata = pdata;

	ret = verify_platform_data(p);
	if (ret) {
		dev_err(p->dev, "%s: Failed to verify platform data\n",
			__func__);
		return -ENODEV;
	}

	return 0;
}

static int dbmdx_deinit_platform_data(struct dbmdx_private *p)
{
	return 0;
}

#else /* CONFIG_OF */

static int dbmdx_init_platform_data(struct dbmdx_private *p)
{
	int ret = 0;

	ret = dbmdx_read_config_from_devtree(p);
	if (ret) {
		dev_err(p->dev, "%s: failed to read device tree data\n",
			__func__);
		dbmdx_deinit_devtree_config(p);
		return ret;
	}

	return 0;
}

static int dbmdx_deinit_platform_data(struct dbmdx_private *p)
{
	return dbmdx_deinit_devtree_config(p);
}

#endif /* CONFIG_OF */

int dbmdx_device_init(struct device *dev)
{
	struct dbmdx_private *p;
	int ret = 0;

	dev_info(dev, "%s: DBMDX codec driver version = %s\n",
		__func__, DRIVER_VERSION);

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	p->dev = dev;

	ret = dbmdx_init_platform_data(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: Could not read platform data\n", __func__);
		ret = -EIO;
		goto out_err_free_private;
	}

	p->vregulator = devm_regulator_get(p->dev, "dbmdx_regulator");
	if (IS_ERR(p->vregulator)) {
		dev_info(p->dev, "%s: Can't get voltage regulator\n",
			__func__);
		p->vregulator = NULL;
	}

	p->dbmdx_workq = create_singlethread_workqueue("dbmdx-wq");
	if (!p->dbmdx_workq) {
		dev_err(p->dev, "%s: Could not create workqueue\n",
			__func__);
		ret = -EIO;
		goto out_err_deinit_pdata;
	}
	INIT_DELAYED_WORK(&p->fw_state_work, dbmdx_check_fw_state);
	INIT_DELAYED_WORK(&p->fw_reload_work, dbmdx_fw_reload);

	/* set helper functions */
	p->reset_set = dbmdx_reset_set;
	p->reset_release = dbmdx_reset_release;
	p->reset_sequence = dbmdx_reset_sequence;
	p->wakeup_set = dbmdx_wakeup_set;
	p->wakeup_release = dbmdx_wakeup_release;
	p->can_wakeup = dbmdx_can_wakeup;
	p->lock = dbmdx_lock;
	p->unlock = dbmdx_unlock;
	p->verify_checksum = dbmdx_verify_checksum;
	p->clk_get_rate = dbmdx_clk_get_rate;
	p->clk_set_rate = dbmdx_clk_set_rate;
	p->clk_enable = dbmdx_clk_enable;
	p->clk_disable = dbmdx_clk_disable;

	/* set callbacks (if already set externally) */
	if (g_event_callback)
		p->event_callback = g_event_callback;

	p->rxsize = MAX_REQ_SIZE;

	ret = dbmdx_common_probe(p);
	if (ret < 0) {
		dev_err(p->dev, "%s: probe failed\n", __func__);
		goto out_err_destroy_workqueue;
	}

	dev_info(p->dev, "%s: successfully probed\n", __func__);
	return 0;

out_err_destroy_workqueue:
	destroy_workqueue(p->dbmdx_workq);
out_err_deinit_pdata:
	dbmdx_deinit_platform_data(p);
out_err_free_private:
	kfree(p);
out:
	return ret;
}

int dbmdx_device_exit(struct device *dev)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	dbmdx_common_remove(p);

	return 0;
}
