/*
 * touch_sw42101.c - SiW touch driver glue for SW42101
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"

#define CHIP_ID						"4211"
#define CHIP_DEVICE_NAME			"SW42101"
#define CHIP_COMPATIBLE_NAME		"siw,sw42101"
#define CHIP_DEVICE_DESC			"SiW Touch SW42101 Driver"

#define CHIP_TYPE					CHIP_SW42101

#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U3 |	\
									0)

#define CHIP_FW_SIZE				(120<<10)

#define CHIP_FLAGS					(0 |	\
									TOUCH_SKIP_ESD_EVENT |	\
									TOUCH_SKIP_RESET_PIN |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)


#define CHIP_INPUT_ID_BUSTYPE		BUS_I2C
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#define __CHIP_QUIRK_ADD			0

#define CHIP_QUIRKS					(0 |	\
									CHIP_QUIRK_NOT_SUPPORT_ASC |	\
									CHIP_QUIRK_NOT_SUPPORT_LPWG |	\
									CHIP_QUIRK_NOT_SUPPORT_WATCH |	\
									CHIP_QUIRK_NOT_SUPPORT_IME |	\
									__CHIP_QUIRK_ADD |	\
									0)

#define CHIP_BUS_TYPE				BUS_IF_I2C
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				-1
#define CHIP_BPW					-1
#define CHIP_MAX_FREQ				-1
#define CHIP_TX_HDR_SZ				I2C_BUS_TX_HDR_SZ
#define CHIP_RX_HDR_SZ				I2C_BUS_RX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			I2C_BUS_TX_DUMMY_SZ
#define CHIP_RX_DUMMY_SZ			I2C_BUS_RX_DUMMY_SZ

static const struct siw_hal_reg_quirk chip_reg_quirks[] = {
	{ .old_addr = TC_VERSION, .new_addr = 0x001, },
	{ .old_addr = INFO_CHIP_VERSION, .new_addr = 0x002, },
	{ .old_addr = TC_PRODUCT_ID1, .new_addr = 0x003, },
	{ .old_addr = SPR_CHIP_TEST, .new_addr = 0x400, },
	{ .old_addr = SPR_BOOT_STS, .new_addr = 0x401, },
	{ .old_addr = TC_DEVICE_CTL, .new_addr = 0x402, },
	{ .old_addr = TC_INTERRUPT_CTL, .new_addr = 0x403, },
	{ .old_addr = IME_STATE, .new_addr = 0x404, },
	{ .old_addr = TCI_ENABLE_W, .new_addr = 0x405, },
	{ .old_addr = SWIPE_ENABLE_W, .new_addr = 0x406, },
	{ .old_addr = SPR_SUBDISP_STS, .new_addr = 0x407, },
	{ .old_addr = TC_DRIVE_CTL, .new_addr = 0x408, },
	{ .old_addr = TC_IC_STATUS, .new_addr = 0x300, },
	{ .old_addr = TC_STS, .new_addr = 0x301, },
	/* */
	{ .old_addr = (1<<31), .new_addr = 0, },	/* switch : don't show log */
	/* */
	{ .old_addr = ~0, .new_addr = ~0 },		// End signal
};


#if defined(__SIW_CONFIG_OF)
/*
 * of_device_is_compatible(dev->of_node, CHIP_COMPATIBLE_NAME)
 */
static const struct of_device_id chip_match_ids[] = {
	{ .compatible = CHIP_COMPATIBLE_NAME },
	{ },
};
#else
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 1920,
	CHIP_MAX_Y			= 1080,
	CHIP_MAX_PRESSURE	= 255,
	CHIP_MAX_WIDTH		= 15,
	CHIP_MAX_ORI		= 1,
	CHIP_MAX_ID			= 10,
	/* */
	CHIP_HW_RST_DELAY	= 1000,
	CHIP_SW_RST_DELAY	= 90,
};

#define CHIP_PIN_RESET			0
#define CHIP_PIN_IRQ			0
#define CHIP_PIN_MAKER			-1
#define CHIP_PIN_VDD			-1
#define CHIP_PIN_VIO			-1

#if (CHIP_PIN_RESET == 0) || (CHIP_PIN_IRQ == 0)
	#error Assign external pin & flag first!!!
#endif
#endif	/* __SIW_CONFIG_OF */


enum {
	LTR_NONE,
	LTR_TRIG,
	LTR_WAIT = 0xFF,
};

struct sw42101_log_ctrl {
	u32 mask;
	u8 trigger;
};

struct sw42101_flash_ctrl {
	u32 addr;
	u16 size;
	u8 status;
	u8 cmd;
};

enum {
	BIN_VER_OFFSET_POS = (CHIP_FW_SIZE - 0x14),			//0x0001_DFEC
	BIN_PID_OFFSET_POS = (CHIP_FW_SIZE - 0x10),			//0x0001_DFF0
};

enum {
	RS_READY	= 0xA0,
	RS_NONE		= 0x05,
	RS_LOG		= 0x77,
	RS_IMAGE	= 0xAA,
};

#define FW_STS_DELAY		25
#define FW_STS_COUNT		200

#define FW_VERIFY_DELAY		5
#define FW_VERIFY_COUNT		1000

static int sw42101_condition_wait(struct device *dev,
					u32 addr, u32 *value, u32 expect,
				    u32 mask, u32 delay, u32 retry)
{
	u32 data = 0;
	int ret = 0;

	do {
		touch_msleep(delay);

		ret = siw_hal_read_value(dev, addr, &data);
		if ((ret >= 0) && ((data & mask) == expect)) {
			if (value)
				*value = data;
			return 0;
		}
	} while (--retry);

	if (value) {
		*value = data;
	}

	t_dev_err(dev,
		"wait fail: addr[%04Xh] data[%08Xh], "
		"mask[%08Xh], expect[%08Xh]\n",
		addr, data, mask, expect);

	return -EPERM;
}

static int sw42101_fwup_check(struct device *dev, u8 *fw_buf)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct siw_ts *ts = chip->ts;
	struct siw_hal_fw_info *fw = &chip->fw;
	struct siw_hal_tc_version *bin_ver;
	u32 bin_ver_offset = BIN_VER_OFFSET_POS;
	u32 bin_pid_offset = BIN_PID_OFFSET_POS;
	u32 dev_major = 0;
	u32 dev_minor = 0;
	char pid[12] = {0, };
	u32 bin_major = 0;
	u32 bin_minor = 0;
//	u32 bin_raw = 0;
	int update = 0;
//	int ret = 0;

	dev_major = fw->v.version.major;
	dev_minor = fw->v.version.minor;

	if (atomic_read(&chip->boot) == IC_BOOT_FAIL) {
		update |= (1<<7);
		goto out;
	}

	if (!dev_major && !dev_minor){
		t_dev_err(dev, "fw can not be 0.0!! Check your panel connection!!\n");
		return 0;
	}

	memcpy(pid, &fw_buf[bin_pid_offset], 8);
	t_dev_dbg_base(dev, "pid %s\n", pid);

	bin_ver = (struct siw_hal_tc_version *)&fw_buf[bin_ver_offset];
	bin_major = bin_ver->major;
	bin_minor = bin_ver->minor;

	t_dev_info(dev, "FW compare: bin-ver: %d.%02d (%s)\n",
			bin_major, bin_minor, pid);

	t_dev_info(dev, "FW compare: dev-ver: %d.%02d (%s)\n",
			dev_major, dev_minor, fw->product_id);

	if (ts->force_fwup) {
		update |= (1<<0);
	} else {
		if (bin_major > dev_major) {
			update |= (1<<1);
		} else if (bin_major == dev_major) {
			if (bin_minor > dev_minor) {
				update |= (1<<2);
			}
		}
	}

	if (memcmp(pid, fw->product_id, 8)) {
		t_dev_err(dev,
			"FW compare: bin-pid[%s] != dev-pid[%s], halted (up %02X, fup %02X)\n",
			pid, fw->product_id, update, ts->force_fwup);
		return -EINVAL;
	}

out:
	t_dev_info(dev,
		"FW compare: up %02X, fup %02X\n",
		update, ts->force_fwup);

	return update;
}

static int sw42101_fwup_mode(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct sw42101_log_ctrl log_ctrl_w = { 0, };
	struct sw42101_log_ctrl log_ctrl_r = { 0, };
	u32 chk_resp, data;
	int ret = 0;

	ret = siw_hal_read_value(dev, 0x610, (void *)&data);
	if (ret < 0) {
		goto out;
	}
	if ((data & 0xFF) == 2) {
		goto out;
	}

	log_ctrl_w.trigger = LTR_NONE;
	ret = siw_hal_reg_write(dev, 0x1010, (void *)&log_ctrl_w, sizeof(log_ctrl_w));
	if (ret < 0) {
		goto out;
	}

	t_dev_dbg_base(dev, "FW upgrade: mode trigger set\n");

	ret = siw_hal_reg_read(dev, 0x1010, (void *)&log_ctrl_r, sizeof(log_ctrl_r));
	if (ret < 0) {
		goto out;
	}

	if (log_ctrl_r.trigger != LTR_NONE) {
		t_dev_err(dev, "FW upgrade: failed - mode trigger, %d\n",
			log_ctrl_r.trigger);
		ret = -EPERM;
		goto out;
	}

	t_dev_dbg_base(dev, "FW upgrade: mode trigger done\n");

	data = 0x02;	//M_TOUCH_DFUP
	ret = siw_hal_reg_write(dev, 0x610, (void *)&data, 1);	//1-byte
	if (ret < 0) {
		goto out;
	}

	t_dev_dbg_base(dev, "FW upgrade: DFUP change set\n");

	chk_resp = RS_READY;
	ret = sw42101_condition_wait(dev, 0x600, &data,
				chk_resp, 0xFF,
				FW_STS_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_0),
				FW_STS_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - DFUP change(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_dbg_base(dev, "FW upgrade: DFUP change done\n");

out:
	return ret;
}

#define FW_WR_OFFSET		0x2000
#define FW_WR_END			(CHIP_FW_SIZE + FW_WR_OFFSET)
#define FW_WR_SIZE			128

#define FW_WR_CMD			0x03

#define FW_WR_LOG_UNIT		(8<<10)

static inline void __down_delay(void)
{
	usleep_range(10, 10);
}

static int sw42101_fwup_down(struct device *dev, u8 *fw_buf, int fw_size)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
	struct sw42101_flash_ctrl ctrl = { 0, };
	u8 *fw_data = fw_buf;
	int fw_offset = FW_WR_OFFSET;
	int fw_size_org, fw_dn_size, fw_dn_percent;
	int fw_pos, curr_size;
	u32 chk_resp, data;
	int ret = 0;

	t_dev_info(dev, "FW upgrade: downloading...\n");

	fw_size_org = fw_size;
	fw_dn_size = 0;

	fw_pos = 0;
	while (fw_size) {
		t_dev_dbg_base(dev, "FW upgrade: fw_pos[%06Xh ...] = %02X %02X %02X %02X ...\n",
				fw_pos,
				fw_data[0], fw_data[1], fw_data[2], fw_data[3]);

		curr_size = min(fw_size, FW_WR_SIZE);

		ret = siw_hal_reg_write(dev, 0x6000, (void *)fw_data, curr_size);
		if (ret < 0) {
			t_dev_err(dev, "FW upgrade: failed - fw-down(%06Xh, %06Xh), %d\n",
				fw_pos, fw_offset, ret);
			goto out;
		}

		__down_delay();

		/*
		 * Flash write
		 */
		t_dev_dbg_base(dev, "FW upgrade: ctrl: a %06Xh, s %Xh\n",
			fw_offset, curr_size);

		ctrl.addr = fw_offset;
		ctrl.size = curr_size;
		ctrl.status = 0;
		ctrl.cmd = FW_WR_CMD;
		ret = siw_hal_reg_write(dev, 0x1400, (void *)&ctrl, sizeof(ctrl));
		if (ret < 0) {
			t_dev_err(dev,
				"FW upgrade: failed - fw-down command(%06Xh, %06Xh), %d\n",
				fw_pos, fw_offset, ret);
			goto out;
		}

		chk_resp = RS_READY;
		ret = sw42101_condition_wait(dev, 0x600, &data,
				chk_resp, 0xFF,
				FW_STS_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_1),
				FW_STS_COUNT);
		if (ret < 0) {
			t_dev_err(dev,
				"FW upgrade: failed - fw-down status(%06Xh, %06Xh, %Xh, %08Xh), %d\n",
				fw_pos, fw_offset, chk_resp, data, ret);
			goto out;
		}

		fw_dn_size += curr_size;
		fw_offset += curr_size;
		fw_data += curr_size;
		fw_pos += curr_size;
		fw_size -= curr_size;

		if (fw_dn_size && !(fw_dn_size & (FW_WR_LOG_UNIT-1))) {
			fw_dn_percent = (fw_dn_size * 100);
			fw_dn_percent /= fw_size_org;

			t_dev_info(dev, "FW upgrade: downloading...(%d%c)\n", fw_dn_percent, '%');
		}
	}

out:
	return ret;
}

static int sw42101_fwup_core(struct device *dev, u8 *fw_buf, int fw_size)
{
	u8 *rd_buf;
	int ret = 0;

	rd_buf = (u8 *)kmalloc(fw_size, GFP_KERNEL);
	if (rd_buf == NULL) {
		t_dev_err(dev, "failed to allocate rd_buf\n");
		return -ENOMEM;
	}
	memcpy(rd_buf, fw_buf, fw_size);

	ret = sw42101_fwup_down(dev, fw_buf, fw_size);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - fw-down, %d\n", ret);
		goto out;
	}

out:
	kfree(rd_buf);

	return ret;
}

static int sw42101_fwup_post(struct device *dev)
{
	struct siw_touch_chip *chip = to_touch_chip(dev);
//	struct siw_ts *ts = chip->ts;
	struct siw_hal_reg *reg = chip->reg;
	u32 chk_resp, data;
	int ret = 0;

	data = 0x11;
	ret = siw_hal_reg_write(dev, 0x610, (void *)&data, 1);	//1-byte
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - reset, %d\n", ret);
		goto out;
	}

	touch_msleep(5000);

	ret = siw_hal_read_value(dev, 0x150, &data);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Wait until TC_DEVICE_CTL becomes initial state(zero)
	 */
	chk_resp = 0;
	ret = sw42101_condition_wait(dev, reg->tc_device_ctl, &data,
				chk_resp, 0xFF,
				FW_VERIFY_DELAY + hal_dbg_delay(chip, HAL_DBG_DLY_FW_2),
				FW_VERIFY_COUNT);
	if (ret < 0) {
		t_dev_err(dev, "FW upgrade: failed - restart status(%Xh), %08Xh\n",
			chk_resp, data);
		goto out;
	}

	t_dev_info(dev, "FW upgrade: post done\n");

out:
	return ret;
}

static int __used sw42101_fwup_upgrade(struct device *dev,
					u8 *fw_buf, int fw_size, int retry)
{
	int fw_size_max = CHIP_FW_SIZE;
	int ret = 0;

	if (fw_size != fw_size_max) {
		t_dev_err(dev, "FW upgrade: wrong file size - %Xh, shall be '%Xh'\n",
			fw_size, fw_size_max);
		return -EFAULT;
	}

	ret = sw42101_fwup_mode(dev);
	if (ret < 0) {
		goto out;
	}

	ret = sw42101_fwup_core(dev, fw_buf, fw_size);
	if (ret < 0) {
		goto out;
	}

	ret = sw42101_fwup_post(dev);

out:
	return ret;
}


/* use eg. cname=arc1 to change name */
static char chip_name[32] = CHIP_DEVICE_NAME;
module_param_string(cname, chip_name, sizeof(chip_name), 0);

/* use eg. dname=arc1 to change name */
static char chip_drv_name[32] = SIW_TOUCH_NAME;
module_param_string(dname, chip_drv_name, sizeof(chip_drv_name), 0);

/* use eg. iname=arc1 to change input name */
static char chip_idrv_name[32] = SIW_TOUCH_INPUT;
module_param_string(iname, chip_idrv_name, sizeof(chip_idrv_name), 0);

static const struct siw_touch_pdata chip_pdata = {
	/* Configuration */
	.chip_id			= CHIP_ID,
	.chip_name			= chip_name,
	.drv_name			= chip_drv_name,
	.idrv_name			= chip_idrv_name,
	.owner				= THIS_MODULE,
	.chip_type			= CHIP_TYPE,
	.mode_allowed		= CHIP_MODE_ALLOWED,
	.fw_size			= CHIP_FW_SIZE,
	.flags				= CHIP_FLAGS,	/* Caution : MSB(bit31) unavailable */
	.irqflags			= CHIP_IRQFLAGS,
	.quirks				= CHIP_QUIRKS,
	/* */
	.bus_info			= {
		.bus_type			= CHIP_BUS_TYPE,
		.buf_size			= CHIP_BUF_SIZE,
		.spi_mode			= CHIP_SPI_MODE,
		.bits_per_word		= CHIP_BPW,
		.max_freq			= CHIP_MAX_FREQ,
		.bus_tx_hdr_size	= CHIP_TX_HDR_SZ,
		.bus_rx_hdr_size	= CHIP_RX_HDR_SZ,
		.bus_tx_dummy_size	= CHIP_TX_DUMMY_SZ,
		.bus_rx_dummy_size	= CHIP_RX_DUMMY_SZ,
	},
#if defined(__SIW_CONFIG_OF)
	.of_match_table 	= of_match_ptr(chip_match_ids),
#else
	.pins				= {
		.reset_pin		= CHIP_PIN_RESET,
		.reset_pin_pol	= OF_GPIO_ACTIVE_LOW,
		.irq_pin		= CHIP_PIN_IRQ,
		.maker_id_pin	= CHIP_PIN_MAKER,
		.vdd_pin		= CHIP_PIN_VDD,
		.vio_pin		= CHIP_PIN_VIO,
	},
	.caps				= {
		.max_x			= CHIP_MAX_X,
		.max_y			= CHIP_MAX_Y,
		.max_pressure	= CHIP_MAX_PRESSURE,
		.max_width		= CHIP_MAX_WIDTH,
		.max_orientation = CHIP_MAX_ORI,
		.max_id			= CHIP_MAX_ID,
		.hw_reset_delay	= CHIP_HW_RST_DELAY,
		.sw_reset_delay	= CHIP_SW_RST_DELAY,
	},
#endif
	/* Input Device ID */
	.i_id				= {
		.bustype		= CHIP_INPUT_ID_BUSTYPE,
		.vendor 		= CHIP_INPUT_ID_VENDOR,
		.product 		= CHIP_INPUT_ID_PRODUCT,
		.version 		= CHIP_INPUT_ID_VERSION,
	},
	/* */
	//See 'siw_hal_get_default_ops' [siw_touch_hal.c]
	.ops				= NULL,
	/* */
	//See 'siw_hal_get_tci_info' [siw_touch_hal.c]
	.tci_info			= NULL,
	.tci_reset_area		= NULL,
	.tci_qcover_open	= NULL,
	.tci_qcover_close	= NULL,
	//See 'siw_hal_get_swipe_info' [siw_touch_hal.c]
	.swipe_ctrl			= NULL,
	//See 'store_ext_watch_config_font_position' [siw_touch_hal_watch.c]
	.watch_win			= NULL,
	//See 'siw_setup_operations' [siw_touch.c]
	.reg_quirks			= (void *)chip_reg_quirks,
	//function quirks
	.fquirks = {
		.fwup_check		= sw42101_fwup_check,
		.fwup_upgrade	= sw42101_fwup_upgrade,
	},
};

static struct siw_touch_chip_data chip_data = {
	.pdata = &chip_pdata,
	.bus_drv = NULL,
};

siw_chip_module_init(CHIP_DEVICE_NAME,
				chip_data,
				CHIP_DEVICE_DESC,
				"kimhh@siliconworks.co.kr");


__siw_setup_str("siw_chip_name=", siw_setup_chip_name, chip_name);
__siw_setup_str("siw_drv_name=", siw_setup_drv_name, chip_drv_name);
__siw_setup_str("siw_idrv_name=", siw_setup_idrv_name, chip_idrv_name);



