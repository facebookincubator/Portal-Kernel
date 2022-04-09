/*
 * dbmdx-spi-d7.c - DSPG DBMD7 SPI interface driver
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
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-chip-interfaces.h>
#include <linux/mfd/dbmdx/dbmdx-spi.h>


static const u8 clr_crc_cmd[] = {0xDB, 0xD7, 0x00, 0x0F};
static const u8 init_lock_cmd[] = {0x5A};
static const u8 start_boot_cmd_rev2[] = {0xDB, 0xD7, 0x00, 0x0B};
static const u8 start_boot_cmd_rev1[] = {0xDB, 0xD7, 0x01, 0x08,
					 0xDB, 0xD7, 0x00, 0x0B,
					 0xDB, 0xD7, 0x11, 0x08,
					 0xDB, 0xD7, 0x00, 0x0B,
					 0xDB, 0xD7, 0x00, 0x0B};

static int spi_send_fw_file(struct dbmdx_private *p, const u8 *fw_data,
				size_t fw_size)
{
	ssize_t send_bytes;
	u16 val;
	u32 pos = 0;
	u8 cur_opcode = 0;
	u8 cur_opcode_opt = 0;
	u8 cmd_header_length = 0;
	u32 cmd_data_length = 0;

	if (!(p->cur_boot_options &
				DBMDX_BOOT_OPT_SEND_FW_BY_COMMAND)) {
		send_bytes = send_spi_data(p, fw_data, fw_size);
		dev_info(p->dev, "%s:Send as file, size=%zd",
						__func__, fw_size);

		if (send_bytes != fw_size) {
			dev_err(p->dev,	"%s: --> load firmware error\n",
						__func__);
			return -EIO;
		} else {
			return 0;
		}
	}

	dev_info(p->dev, "%s:Send FW by command, size=%zd",
						__func__, fw_size);

	while (pos < fw_size - 3) {
		val = *(u16 *)(&fw_data[pos]);

		if (pos >= fw_size) {
			dev_err(p->dev, "%s:%d %u", __func__,
				__LINE__, pos);
			return -EINVAL;
		}

		if (val == 0xD7DB) {
			cur_opcode_opt = fw_data[pos+2];
			cur_opcode = fw_data[pos+3];

			if (cur_opcode == 0) {
				/* 0: Load S.Boot Data:
				 *		HEADER:4
				 *		DATA_LEN:4
				 *		DATA:N
				 */
				cmd_header_length = 8;
				if (pos >= fw_size - 7) {
					dev_err(p->dev,
						"%s:FW Size mismatch: pos=%u",
						__func__, pos);
					return -EINVAL;
				}

				cmd_data_length = *(u32 *)(&fw_data[pos+4]);
			} else if (cur_opcode == 1 || cur_opcode == 4 ||
							cur_opcode == 5) {
				/* 1: Read S.Boot Status: HEADER:4 DUMMY:4
				 * 4: Read 32Bit Int: HEADER:4 ADDR:4
				 * 5: FLASH: HEADER:4 DUMMY:4
				 */
				cmd_header_length = 8;
				cmd_data_length = 0;
			} else if (cur_opcode == 2) {
				/* 2: Load To Mem:
				 *		HEADER:4
				 *		DATA_LEN:4
				 *		START_ADDR:4
				 */
				cmd_header_length = 12;
				if (pos >= fw_size - 11) {
					dev_err(p->dev,
						"%s:FW Size mismatch: pos=%u",
						__func__, pos);
					return -EINVAL;
				}
				cmd_data_length = *(u32 *)(&fw_data[pos+4]);
			} else if (cur_opcode == 3) {
				/* 3:  Write Int32: HEADER:4 ADDR:4 VALUE:4*/
				cmd_header_length = 12;
				cmd_data_length = 0;
			} else if (cur_opcode == 6 || cur_opcode == 8) {
				/* 6: Load Params:
				 *		HEADER(N):4
				 *		PARAM1:4 .. PARAM_N:4
				 *
				 * 8: HF_CORE MSG:
				 *		HEADER(N):4
				 *		PARAM1:4 .. PARAM_N:4
				 */

				cmd_header_length = 4;
				cmd_data_length = 4 * cur_opcode_opt;
			} else {
				 /* 7, 9, A-F: HEADER:4 */
				cmd_header_length = 4;
				cmd_data_length = 0;
			}

			if (pos + cmd_header_length + cmd_data_length >
								fw_size) {
				dev_err(p->dev, "%s:FW Size mismatch: pos=%u",
					__func__, pos);
				return -EINVAL;
			}

			if (p->dump_traffic > DBMDX_DBG_DUMP_LEVEL_NO_DUMP)
				dev_info(p->dev,
					"%s:Sending SBoot Cmd: Opcode: 0x%x, HdrLen: %u, DataLen: %u, pos: %u\n",
					__func__, cur_opcode,
					cmd_header_length,
					cmd_data_length, pos);

			send_bytes = send_spi_data(p, &fw_data[pos],
							cmd_header_length);
			if (send_bytes != cmd_header_length) {
				dev_err(p->dev,	"%s: --> load firmware error\n",
							__func__);
				return -EIO;
			}

			pos += cmd_header_length;

			usleep_range(DBMDX_USLEEP_AFTER_SBOOT_CMD,
					DBMDX_USLEEP_AFTER_SBOOT_CMD + 100);

			if (cmd_data_length) {
				send_bytes = send_spi_data(p,
						&fw_data[pos],
						cmd_data_length);
				if (send_bytes != cmd_data_length) {
					dev_err(p->dev,
						"%s: --> load firmware error\n",
						__func__);
					return -EIO;
				}
				pos += cmd_data_length;
			}

		} else {
			pos += 2;
			continue;
		}
	}

	dev_info(p->dev, "%s:FW has been sent successfully",
						__func__);
	return 0;
}

static int dbmd7_spi_boot(struct dbmdx_private *p, const void *fw_data[],
			size_t fw_size[], const void *checksum,
			unsigned int num_of_fw_files,
			size_t chksum_len, int load_fw)
{

	int retry = RETRY_COUNT;
	int ret = 0;
	int i;
	const u8 *cur_checksum;
	bool file_send_success = false;

	dev_info(p->dev, "%s\n", __func__);

	do {

		if (p->active_fw == DBMDX_FW_PRE_BOOT) {

			if (!(p->boot_mode & DBMDX_BOOT_MODE_RESET_DISABLED)) {
				/* reset DBMD7 chip */
				p->reset_sequence(p);
			} else {
				/* If failed and reset is disabled, break */
				if (retry != RETRY_COUNT) {
					retry = -1;
					break;
				}
			}

			/* delay before sending commands */
			if (p->clk_get_rate(p, DBMDX_CLK_MASTER) <= 32768)
				msleep(DBMDX_MSLEEP_SPI_D7_AFTER_RESET_32K);
			else
				usleep_range(DBMDX_USLEEP_SPI_D7_AFTER_RESET,
					DBMDX_USLEEP_SPI_D7_AFTER_RESET + 5000);

			ret = spi_set_speed(p, DBMDX_SPEED_MAX);
			if (ret < 0) {
				dev_err(p->dev, "%s:failed %x\n",
					__func__, ret);
				continue;
			}

			if (!(p->cur_boot_options &
				DBMDX_BOOT_OPT_DONT_SEND_INIT_LOCK_CMD)) {
				/* Lock peripheral by sending 0x5A */
				ret = send_spi_data(p, init_lock_cmd,
							sizeof(init_lock_cmd));
				if (ret != sizeof(init_lock_cmd)) {
					dev_err(p->dev,
					"%s: failed to send Init Lock Cmd\n",
						__func__);
					continue;
				}
			}

			/* verify chip id */
			if (p->cur_boot_options &
				DBMDX_BOOT_OPT_VERIFY_CHIP_ID) {
				ret = spi_verify_chip_id(p);
				if (ret < 0) {
					dev_err(p->dev,
						"%s: couldn't verify chip id\n",
						__func__);
					continue;
				}
			}

			if (!(p->cur_boot_options &
				DBMDX_BOOT_OPT_DONT_CLR_CRC)) {

				/* send CRC clear command */
				ret = send_spi_data(p, clr_crc_cmd,
					sizeof(clr_crc_cmd));
				if (ret != sizeof(clr_crc_cmd)) {
					dev_err(p->dev,
						"%s: failed to clear CRC\n",
						__func__);
					continue;
				}
			}
		}
		if (load_fw) {
			file_send_success = true;
			for (i = 0; i < num_of_fw_files; i++) {
				dev_info(p->dev, "%s Sending FW file #%d\n",
					__func__, i);
				/* send firmware */
				ret = spi_send_fw_file(p,  fw_data[i],
								fw_size[i]);
				if (ret < 0) {
					dev_err(p->dev,
					"%s: --> load firmware error (%d)\n",
						__func__, i);
					file_send_success = false;
					break;
				}

				/* verify checksum */
				if (checksum && !(p->cur_boot_options &
					DBMDX_BOOT_OPT_DONT_VERIFY_CRC)) {
					cur_checksum = (u8 *)checksum +
								i*chksum_len;
					ret = spi_verify_boot_checksum(p,
						cur_checksum, chksum_len);
					if (ret < 0) {
						dev_err(p->dev,
						"%s: couldn't verify checksum (%d)\n",
							__func__, i);
						file_send_success = false;
						break;
					}
					/* send CRC clear command */
					ret = send_spi_data(p, clr_crc_cmd,
						sizeof(clr_crc_cmd));
					if (ret != sizeof(clr_crc_cmd)) {
						dev_err(p->dev,
						"%s: failed to clear CRC (%d)\n",
						__func__, i);
						file_send_success = false;
						break;
					}
				}
			}
			if (!file_send_success)
				continue;
		}
		if (!(p->cur_boot_options &
				DBMDX_BOOT_OPT_DONT_SEND_START_BOOT)) {
			if (!(p->cur_boot_options &
					DBMDX_BOOT_OPT_USE_REV2_BOOT_CMD)) {
				/* send boot command */
				ret = send_spi_data(p, start_boot_cmd_rev1,
						sizeof(start_boot_cmd_rev1));
				if (ret != sizeof(start_boot_cmd_rev1)) {
					dev_err(p->dev,
					"%s: failed to send start boot cmd\n",
						__func__);
					continue;
				}
			} else {
				ret = send_spi_data(p, start_boot_cmd_rev2,
						sizeof(start_boot_cmd_rev2));
				if (ret != sizeof(start_boot_cmd_rev2)) {
					dev_err(p->dev,
					"%s: failed to send start boot cmd\n",
						__func__);
					continue;
				}
			}
		}

		dev_info(p->dev, "%s: ---------> firmware loaded\n",
			__func__);
		break;
	} while (--retry);

	/* no retries left, failed to boot */
	if (retry <= 0) {
		dev_err(p->dev, "%s: failed to load firmware\n", __func__);
		return -EIO;
	}

	ret = spi_set_speed(p, DBMDX_SPEED_NORMAL);
	if (ret < 0)
		dev_err(p->dev, "%s:failed %x\n", __func__, ret);

	/* wait some time */
	usleep_range(DBMDX_USLEEP_SPI_D7_AFTER_BOOT,
		DBMDX_USLEEP_SPI_D7_AFTER_BOOT + 1000);

	return ret;
}


static int dbmd7_spi_switch_to_buffering_speed(struct dbmdx_private *p,
						bool reconfigure_dsp_clock)
{

	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = spi_set_speed(p, DBMDX_SPEED_BUFFERING);

	if (ret < 0) {
		dev_err(p->dev, "%s:failed setting speed %x\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int dbmd7_spi_switch_to_normal_speed(struct dbmdx_private *p,
	bool reconfigure_dsp_clock)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = spi_set_speed(p, DBMDX_SPEED_NORMAL);

	if (ret < 0) {
		dev_err(p->dev, "%s:failed setting speed %x\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int dbmd7_spi_prepare_buffering(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmd7_spi_switch_to_buffering_speed(p, false);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to change speed to buffering\n",
			__func__);
		goto out;
	}
out:
	return ret;
}

static int dbmd7_spi_finish_buffering(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmd7_spi_switch_to_normal_speed(p, false);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to change speed to buffering\n",
			__func__);
		goto out;
	}
out:
	return ret;
}


static int dbmd7_spi_prepare_amodel_loading(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmd7_spi_switch_to_buffering_speed(p, true);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to change speed to buffering\n",
			__func__);
		goto out;
	}

out:
	return ret;
}

static int dbmd7_spi_finish_amodel_loading(struct dbmdx_private *p)
{
	int ret;

	dev_dbg(p->dev, "%s\n", __func__);

	ret = dbmd7_spi_switch_to_normal_speed(p, true);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: failed to change speed to buffering\n",
			__func__);
		goto out;
	}
out:
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int dbmdx_spi_suspend(struct device *dev)
{
	struct dbmdx_private *p = spi_get_drvdata(to_spi_device(dev));
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = dbmdx_core_suspend(p);
	if (ret)
		return ret;

	spi_interface_suspend(p);

	return 0;
}

static int dbmdx_spi_resume(struct device *dev)
{
	struct dbmdx_private *p = spi_get_drvdata(to_spi_device(dev));
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	spi_interface_resume(p);

	ret = dbmdx_core_resume(p);
	if (ret)
		goto exit_no_core;

	return 0;

exit_no_core:
	spi_interface_suspend(p);
	return ret;
}
#else
#define dbmdx_spi_suspend NULL
#define dbmdx_spi_resume NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int dbmdx_spi_runtime_suspend(struct device *dev)
{
	struct dbmdx_private *p = spi_get_drvdata(to_spi_device(dev));

	dev_dbg(dev, "%s\n", __func__);

	spi_interface_suspend(p);

	return 0;
}

static int dbmdx_spi_runtime_resume(struct device *dev)
{
	struct dbmdx_private *p = spi_get_drvdata(to_spi_device(dev));

	dev_dbg(dev, "%s\n", __func__);
	spi_interface_resume(p);

	return 0;
}
#else
#define dbmdx_spi_runtime_suspend NULL
#define dbmdx_spi_runtime_resume NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops dbmdx_spi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(dbmdx_spi_suspend, dbmdx_spi_resume)
	SET_RUNTIME_PM_OPS(dbmdx_spi_runtime_suspend,
			   dbmdx_spi_runtime_resume, NULL)
};

static int dbmd7_spi_probe(struct spi_device *client)
{
	int rc = 0;
	struct dbmdx_private *p;

	rc = dbmdx_device_init(&client->dev);
	if (rc < 0)
		return rc;

	rc = spi_common_probe(client);
	if (rc < 0)
		return rc;
	p = dev_get_drvdata(&client->dev);
	/* fill in chip interface functions */
	p->chip->prepare_amodel_loading = dbmd7_spi_prepare_amodel_loading;
	p->chip->finish_amodel_loading = dbmd7_spi_finish_amodel_loading;
	p->chip->prepare_buffering = dbmd7_spi_prepare_buffering;
	p->chip->finish_buffering = dbmd7_spi_finish_buffering;
	p->chip->boot = dbmd7_spi_boot;

	dbmdx_interface_probe(p);

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);

	return rc;
}

static int dbmd7_spi_remove(struct spi_device *client)
{
	pm_runtime_disable(&client->dev);
	dbmdx_device_exit(&client->dev);
	spi_common_remove(client);

	return 0;
}

static const struct of_device_id dbmd_7_spi_of_match[] = {
	{ .compatible = "dspg,dbmd7-spi", },
	{},
};

#ifdef CONFIG_SND_SOC_DBMDX
MODULE_DEVICE_TABLE(of, dbmd_7_spi_of_match);
#endif

static const struct spi_device_id dbmd_7_spi_id[] = {
	{ "dbmdx-spi", 0 },
	{ "dbmd7-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, dbmd_7_spi_id);

static struct spi_driver dbmd_7_spi_driver = {
	.driver = {
		.name = "dbmd_7-spi",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dbmd_7_spi_of_match,
#endif
		.pm = &dbmdx_spi_pm,
	},
	.probe =    dbmd7_spi_probe,
	.remove =   dbmd7_spi_remove,
	.id_table = dbmd_7_spi_id,
};

static int __init dbmd_7_modinit(void)
{
	return spi_register_driver(&dbmd_7_spi_driver);
}
module_init(dbmd_7_modinit);

static void __exit dbmd_7_exit(void)
{
	spi_unregister_driver(&dbmd_7_spi_driver);
}
module_exit(dbmd_7_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("DSPG DBMD7 spi interface driver");
MODULE_LICENSE("GPL");
