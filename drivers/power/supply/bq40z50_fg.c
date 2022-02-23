/*
 * bq40z50 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (c) 2020, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[bq40z50] %s: " fmt, __func__
#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define CREATE_TRACE_POINTS
#include <trace/events/bq40z50_fg.h>
#include <asm/unaligned.h>

#define bq_info pr_info
#define bq_dbg pr_debug
#define bq_err pr_err
#define bq_log pr_err

#define MAX_INIT_RETRY_COUNT 10

#define INVALID_REG_ADDR 0xFF
#define NUM_CELLS 4
#define MANUFACTURER_NAME_LENGTH 20
#define DEVICE_NAME_LENGTH 20

#define CHARGING_STATUS_MASK 0x3FFF7F
#define OPERATION_STATUS_MASK 0xFFFF

#define FG_FLAGS_FD BIT(4)
#define FG_FLAGS_FC BIT(5)
#define FG_FLAGS_DSG BIT(6)
#define FG_FLAGS_INIT BIT(7)
#define FG_FLAGS_RCA BIT(9)
#define FG_FLAGS_UTD BIT(27) /* Undertemperature during discharge */
#define FG_FLAGS_UTC BIT(26) /* Undertemperature during charge */
#define FG_FLAGS_OTD BIT(13) /* Overtemperature during discharge */
#define FG_FLAGS_OTC BIT(12) /* Overtemperature during charge */

#define SOC_MIN_RECHARGE_THR 90
#define SOC_MAX_RECHARGE_THR 99
#define SOC_MIN_LOW_THR 1
#define SOC_MAX_LOW_THR 10

#define SOC_LOWBAT_LEVEL 1
#define SOC_FULL_LEVEL 100

#define POLL_INTERVAL_MS 60000

/* 500 ms from alarm timer until driver resumes */
#define BQ40Z50_RESUME_TIMEOUT_MS 500

enum bq_fg_reg_idx {
	BQ_FG_REG_MAC = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_CURR_NOW,	/* Current Now */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_BATT_STATUS,	/* BatteryStatus */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_REMAIN_CAP,	/* Remaining Capacity */
	BQ_FG_REG_CYCLE_COUNT,	/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DESIGN_CAP,	/* Design Capacity */
	BQ_FG_REG_MBA,		/* ManufacturerBlockAccess*/
	BQ_FG_REG_CELL_VOLT1,	/* Cell Voltage 1 */
	BQ_FG_REG_CELL_VOLT2,	/* Cell Voltage 2 */
	BQ_FG_REG_CELL_VOLT3,	/* Cell Voltage 3 */
	BQ_FG_REG_CELL_VOLT4,	/* Cell Voltage 4 */
	BQ_FG_REG_CHG_CURR,	/* Charging Current */
	BQ_FG_REG_CHG_VOLT,	/* Charging Voltage */
	BQ_FG_REG_BATT_MODE,	/* BatteryMode */
	BQ_FG_REG_SS,           /* Safety Status */
	BQ_FG_REG_MAN_NAME,     /* Manufacturer Name */
	BQ_FG_REG_DEV_NAME,     /* Device Name */
	NUM_REGS,
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_OP_STATUS = 0x0000,
	FG_MAC_CMD_DEV_TYPE = 0x0001,
	FG_MAC_CMD_FW_VER = 0x0002,
	FG_MAC_CMD_HW_VER = 0x0003,
	FG_MAC_CMD_IF_SIG = 0x0004,
	FG_MAC_CMD_CHEM_ID = 0x0006,
	FG_MAC_CMD_SHUTDOWN = 0x0010,
	FG_MAC_CMD_GAUGING = 0x0021,
	FG_MAC_CMD_SEAL = 0x0030,
	FG_MAC_CMD_DEV_RESET = 0x0041,
	FG_MAC_CMD_SS = 0x0051,
	FG_MAC_CMD_PF_STATUS = 0x0053,
	FG_MAC_CMD_OPER_STATUS = 0x0054,
	FG_MAC_CMD_CHARGING_STATUS = 0x0055,
	FG_MAC_CMD_LDB_1 = 0x0060,
	FG_MAC_CMD_LDB_2 = 0x0061,
	FG_MAC_CMD_LDB_3 = 0x0062,
	FG_MAC_CMD_LDB_4 = 0x0063,
	FG_MAC_CMD_LDB_5 = 0x0064,
	FG_MAC_CMD_LDB_6 = 0x0065,
	FG_MAC_CMD_LDB_7 = 0x0066,
	FG_MAC_CMD_LDB_8 = 0x0067,
	FG_MAC_CMD_LDB_9 = 0x0068,
	FG_MAC_CMD_LDB_10 = 0x0069,
	FG_MAC_CMD_LDB_11 = 0x006a,
	FG_MAC_CMD_LDB_12 = 0x006b,
	FG_MAC_DA_STATUS_2 = 0x0072,
	FG_MAC_CMD_CB_STATUS = 0x0076,
};

enum bq_fg_ldb1_offsets {
	FG_MAX_CELL_VOLTAGE = 0,
	FG_MIN_CELL_VOLTAGE = 8,
	FG_MAX_DELTA_CELL_VOLT = 16,
	FG_MAX_CHARGE_CURRENT = 18,
	FG_MAX_DISCHARGE_CURRENT = 20,
	FG_MAX_AVG_DSG_CURRENT = 22,
	FG_MAX_AVG_DSG_POWER = 24,
	FG_MAX_TEMP_CELL = 26,
	FG_MIN_TEMP_CELL = 27,
	FG_MAX_DELTA_CELL_TEMP = 28,
	FG_MAX_TEMP_INT_SENSOR = 29,
	FG_MIN_TEMP_INT_SENSOR = 30,
	FG_MAX_TEMP_FET = 31,
};

enum bq_fg_ldb2_offsets {
	FG_NUM_SHUTDOWNS = 0,
};

enum bq_fg_ldb3_offsets {
	FG_FW_RUNTIME = 0,
};

enum bq_fg_ldb4_offsets {
	FG_NUM_COV_EVENTS = 0,
	FG_LAST_COV_EVENT = 2,
	FG_NUM_CUV_EVENTS = 4,
	FG_LAST_CUV_EVENT = 6,
	FG_NUM_OCD1_EVENTS = 8,
	FG_LAST_OCD1_EVENT = 10,
	FG_NUM_OCD2_EVENTS = 12,
	FG_LAST_OCD2_EVENT = 14,
	FG_NUM_OCC1_EVENTS = 16,
	FG_LAST_OCC1_EVENT = 18,
	FG_NUM_OCC2_EVENTS = 20,
	FG_LAST_OCC2_EVENT = 22,
	FG_NUM_AOLD_EVENTS = 24,
	FG_LAST_AOLD_EVENT = 26,
	FG_NUM_ASCD_EVENTS = 28,
	FG_LAST_ASCD_EVENT = 30,
};

enum bq_fg_ldb5_ofsets {
	FG_NUM_ASCC_EVENTS = 0,
	FG_LAST_ASCC_EVENT = 2,
	FG_NUM_OTC_EVENTS = 4,
	FG_LAST_OTC_EVENT = 6,
	FG_NUM_OTD_EVENTS = 8,
	FG_LAST_OTD_EVENT = 10,
	FG_NUM_OTF_EVENTS = 12,
	FG_LAST_OTF_EVENT = 14,
	FG_NUMBER_VCT = 16,
	FG_LAST_VCT = 18,
};

enum bq_fg_ldb6_ofsets {
	FG_TIME_SPENT_UT_A = 0,
	FG_TIME_SPENT_UT_B = 4,
	FG_TIME_SPENT_UT_C = 8,
	FG_TIME_SPENT_UT_D = 12,
	FG_TIME_SPENT_UT_E = 16,
	FG_TIME_SPENT_UT_F = 20,
	FG_TIME_SPENT_UT_G = 24,
	FG_TIME_SPENT_UT_H = 28,
};

enum bq_fg_ldb7_ofsets {
	FG_TIME_SPENT_LT_A = 0,
	FG_TIME_SPENT_LT_B = 4,
	FG_TIME_SPENT_LT_C = 8,
	FG_TIME_SPENT_LT_D = 12,
	FG_TIME_SPENT_LT_E = 16,
	FG_TIME_SPENT_LT_F = 20,
	FG_TIME_SPENT_LT_G = 24,
	FG_TIME_SPENT_LT_H = 28,
};

enum bq_fg_ldb8_ofsets {
	FG_TIME_SPENT_STL_A = 0,
	FG_TIME_SPENT_STL_B = 4,
	FG_TIME_SPENT_STL_C = 8,
	FG_TIME_SPENT_STL_D = 12,
	FG_TIME_SPENT_STL_E = 16,
	FG_TIME_SPENT_STL_F = 20,
	FG_TIME_SPENT_STL_G = 24,
	FG_TIME_SPENT_STL_H = 28,
};

enum bq_fg_ldb9_ofsets {
	FG_TIME_SPENT_RT_A = 0,
	FG_TIME_SPENT_RT_B = 4,
	FG_TIME_SPENT_RT_C = 8,
	FG_TIME_SPENT_RT_D = 12,
	FG_TIME_SPENT_RT_E = 16,
	FG_TIME_SPENT_RT_F = 20,
	FG_TIME_SPENT_RT_G = 24,
	FG_TIME_SPENT_RT_H = 28,
};

enum bq_fg_ldb10_ofsets {
	FG_TIME_SPENT_STH_A = 0,
	FG_TIME_SPENT_STH_B = 4,
	FG_TIME_SPENT_STH_C = 8,
	FG_TIME_SPENT_STH_D = 12,
	FG_TIME_SPENT_STH_E = 16,
	FG_TIME_SPENT_STH_F = 20,
	FG_TIME_SPENT_STH_G = 24,
	FG_TIME_SPENT_STH_H = 28,
};

enum bq_fg_ldb11_ofsets {
	FG_TIME_SPENT_HT_A = 0,
	FG_TIME_SPENT_HT_B = 4,
	FG_TIME_SPENT_HT_C = 8,
	FG_TIME_SPENT_HT_D = 12,
	FG_TIME_SPENT_HT_E = 16,
	FG_TIME_SPENT_HT_F = 20,
	FG_TIME_SPENT_HT_G = 24,
	FG_TIME_SPENT_HT_H = 28,
};

enum bq_fg_ldb12_ofsets {
	FG_TIME_SPENT_OT_A = 0,
	FG_TIME_SPENT_OT_B = 4,
	FG_TIME_SPENT_OT_C = 8,
	FG_TIME_SPENT_OT_D = 12,
	FG_TIME_SPENT_OT_E = 16,
	FG_TIME_SPENT_OT_F = 20,
	FG_TIME_SPENT_OT_G = 24,
	FG_TIME_SPENT_OT_H = 28,
};

typedef enum GG_VAL_TYPE {
	GG_I2,	/* Signed 16-bit integer. */
	GG_I1,	/* Signed 8-bit integer. */
	GG_U2,	/* Unsigned 16-bit integer. */
	GG_U4,	/* Unsigned 32-bit integer. */
} GG_VAL_TYPE;

enum bq_fg_device {
	BQ40Z50,
};

static const unsigned char *device2str[] = {
	"bq40z50",
};

static u8 bq40z50_regs[NUM_REGS] = {
	0x00, /* CONTROL */
	0x08, /* TEMP */
	0x09, /* VOLT */
	0x0A, /* CURRENT NOW */
	0x0B, /* AVG CURRENT */
	0x16, /* FLAGS */
	0x12, /* Time to empty */
	0x13, /* Time to full */
	0x10, /* Full charge capacity */
	0x0F, /* Remaining Capacity */
	0x17, /* CycleCount */
	0x0D, /* State of Charge */
	0x4F, /* State of Health */
	0x18, /* Design Capacity */
	0x44, /* ManufacturerBlockAccess*/
	0x3F, /* CellVoltage1 */
	0x3E, /* CellVoltage2 */
	0x3D, /* CellVoltage3 */
	0x3C, /* CellVoltage4 */
	0x14, /* ChargingCurrent */
	0x15, /* ChargingVoltage */
	0x03, /* BatteryMode */
	0x50, /* SafetyStatus */
	0x20, /* Manufacturer Name */
	0x21, /* Device Name */
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;

	struct mutex data_lock;

	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

	u8 manuf_name[MANUFACTURER_NAME_LENGTH+1];
	u8 device_name[DEVICE_NAME_LENGTH+1];

	/* status tracking */
	bool batt_fc;
	bool batt_fd;	/* full depleted */
	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */
	bool batt_init; /* 0 - Init in progress, 1 - Gauge init Done */
	int seal_state;	/* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_volt_now;  /* Voltage now in uV */
	int batt_curr_now;  /* Current now in uA */
	bool report_in_mAh;

	int soc_recharge_thr; /* soc recharge threshold in percentage (ex 95) */
	int soc_low_thr; /* soc low battery threshold in percentage (ex 3) */
	int soc_extra_thr; /* extra threshold for dsoc 100% */
	bool soc_static_remap; /* remap soc */

	/* Cached values */
	int op_status;
	int pf_status;
	int charging_status;
	int batt_mode;
	int chg_volt; /* Charging voltage */
	int chg_curr; /* Charging current */
	int batt_status; /* Battery status */
	int batt_cell1_volt;
	int batt_cell2_volt;
	int batt_cell3_volt;
	int batt_cell4_volt;
	int batt_pwr_status; /* Battery power status */
	int batt_cap_lvl; /* Battery capacity level */
	int batt_soc;
	int batt_soc_real; /* State of Charge from gauge */
	int batt_ss;    /* Battery safety status */
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_temp; /* Temperature in 0.1C */
	int batt_cyclecnt; /* cycle count */
	int safety_status;

	bool batt_ok;
	int batt_detect_count;

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct delayed_work monitor_work;
	struct delayed_work delayed_init_work;

	struct power_supply *fg_psy;
	struct power_supply_desc fg_psy_d;

	struct alarm monitor_alarm;
	struct completion resume_done;

	bool pm_stay_awake;
};

static int fg_read_remaining_capacity(struct bq_fg_chip *bq, int *val);
static int fg_read_fcc(struct bq_fg_chip *bq, int *val);
static int fg_read_temperature(struct bq_fg_chip *bq, int *val);
static void fg_update_status(struct bq_fg_chip *bq);

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		bq_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
		       val, reg);
		return ret;
	}

	return 0;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;

	ret = i2c_smbus_write_block_data(client, reg, len, buf);

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	ret = __fg_read_word(bq->client, reg, val);

	return ret;
}

static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	ret = __fg_write_word(bq->client, reg, val);

	return ret;
}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	ret = __fg_write_block(bq->client, reg, data, len);

	return ret;
}

static int fg_read_mba(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 t_buf[I2C_SMBUS_BLOCK_MAX + 3];
	struct i2c_msg msg[2];

	if (len > I2C_SMBUS_BLOCK_MAX)
		len = I2C_SMBUS_BLOCK_MAX;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(20);

	msg[0].addr = bq->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &bq->regs[BQ_FG_REG_MBA];
	msg[0].len = 1;
	msg[1].addr = bq->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = t_buf;
	msg[1].len = len + 3;

	ret = i2c_transfer(bq->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;


	/* When reading a MANUFACTURER_BLOCK_ACCESS or dataflash
	 * the first 3 bytes will be the byte count + command or address
	 */
	memcpy(buf, &t_buf[3], len);

	return 0;
}

static int fg_read_block(struct bq_fg_chip *bq, u8 cmd, u8 *buf, u8 len)
{
	u8 t_buf[I2C_SMBUS_BLOCK_MAX];
	struct i2c_msg msg[2];
	int ret;

	if (len > I2C_SMBUS_BLOCK_MAX)
		return -EINVAL;

	msg[0].addr = bq->client->addr;
	msg[0].flags = 0;
	msg[0].buf = &cmd;
	msg[0].len = 1;
	msg[1].addr = bq->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = t_buf;
	msg[1].len = len + 1;

	ret = i2c_transfer(bq->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		bq_err("Failed to fg read block:%d\n", ret);
		return ret;
	}

	if (t_buf[0] > len)
		return -EIO;

	memcpy(buf, &t_buf[1], t_buf[0]);
	return t_buf[0];
}

static void fg_enable_shutdown_mode(struct bq_fg_chip *bq)
{
	int ret;

	/* If the gauge is sealed, send MAC Shutdown command twice,
	 * back to back.
	 */

	/* If the gauge is unsealed and the MAC Shutdown() command is sent twice
	 * in a row, the gauge will execute the shutdown sequence immediately
	 * and skip the normal delay sequence.
	 */

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_MAC], FG_MAC_CMD_SHUTDOWN);
	if (ret < 0) {
		bq_err("Failed to send the first shutdown subcommand:%d\n",
				ret);
		return;
	}

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_MAC], FG_MAC_CMD_SHUTDOWN);
	if (ret < 0)
		bq_err("Failed to send the second shutdown subcommand:%d\n",
				ret);
}

static int fg_read_fw_version(struct bq_fg_chip *bq)
{
	int ret;
	u8 buf[36];

	mutex_lock(&bq->data_lock);
	ret = fg_read_mba(bq, FG_MAC_CMD_FW_VER, buf, 11);
	mutex_unlock(&bq->data_lock);
	if (ret < 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		return -EINVAL;
	}

	bq_log("FW Ver:%04X, Build:%04X\n", buf[2] << 8 | buf[3],
	       buf[4] << 8 | buf[5]);
	bq_log("Ztrack Ver:%04X\n", buf[7] << 8 | buf[8]);

	return 0;
}

static int fg_read_ss(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u8 buf[4];

	ret = fg_read_mba(bq, FG_MAC_CMD_SS, buf, sizeof(buf));
	if (ret < 0) {
		bq_err("Failed to read safety status:%d\n", ret);
		return ret;
	}

	*val = get_unaligned_le32(buf);

	if (bq->safety_status != *val) {
		bq->safety_status = *val;
		trace_read_safety_status(bq->safety_status);
	}

	return 0;
}

static int fg_read_chrg_status(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u8 buf[4];

	ret = fg_read_mba(bq, FG_MAC_CMD_CHARGING_STATUS, buf, sizeof(buf));
	if (ret < 0) {
		bq_err("Failed to read charging status:%d\n", ret);
		return ret;
	}

	*val = get_unaligned_le32(buf);

	if (bq->charging_status != *val) {
		bq->charging_status = *val;
		trace_read_charging_status(bq->charging_status &
						CHARGING_STATUS_MASK);
	}

	return 0;
}

static int fg_read_param_to_sysfs(struct bq_fg_chip *bq, void *buf,
				size_t bufsize, size_t offset, int cmd)
{
	int ret;
	u8 tmpbuf[I2C_SMBUS_BLOCK_MAX];

	if ((offset + bufsize) > I2C_SMBUS_BLOCK_MAX)
		return -EINVAL;

	mutex_lock(&bq->data_lock);
	ret = fg_read_mba(bq, cmd, tmpbuf, sizeof(tmpbuf));
	mutex_unlock(&bq->data_lock);
	if (ret < 0) {
		bq_err("Failed to read param:%d\n", ret);
		return ret;
	}

	memcpy(buf, &tmpbuf[offset], bufsize);

	return 0;
}

static int fg_read_manuf_name(struct bq_fg_chip *bq)
{
	int ret;

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_MAN_NAME],
				bq->manuf_name, MANUFACTURER_NAME_LENGTH);
	if (ret < 0) {
		bq_err("Failed to read manufacturer name:%d\n", ret);
		return ret;
	}

	if (ret > MANUFACTURER_NAME_LENGTH)
		return -EINVAL;

	bq->manuf_name[ret] = '\0';

	return 0;
}

static int fg_read_device_name(struct bq_fg_chip *bq)
{
	int ret;

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_DEV_NAME],
					bq->device_name, DEVICE_NAME_LENGTH);
	if (ret < 0) {
		bq_err("Failed to read device name:%d\n", ret);
		return ret;
	}

	if (ret > DEVICE_NAME_LENGTH)
		return -EINVAL;

	bq->device_name[ret] = '\0';

	return 0;
}

static int fg_read_battery_status(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 flags = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0) {
		bq_err("Could not read battery status, ret = %d\n", ret);
		return ret;
	}

	*val = flags;

	return 0;
}

static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0) {
		bq_err("Could not read status, ret = %d\n", ret);
		return ret;
	}

	bq->batt_fc = !!(flags & FG_FLAGS_FC);
	bq->batt_fd = !!(flags & FG_FLAGS_FD);
	bq->batt_rca = !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg = !!(flags & FG_FLAGS_DSG);
	bq->batt_init = !!(flags & FG_FLAGS_INIT);

	return 0;
}

static int fg_read_op_status(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 op_status = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_MAC], &op_status);
	if (ret < 0) {
		bq_err("Could not read operation status, ret = %d\n", ret);
		return ret;
	}

	if (bq->op_status != op_status) {
		bq->op_status = op_status;
		trace_read_operation_status(op_status &
						OPERATION_STATUS_MASK);
	}

	*val = op_status;

	return 0;
}

static int fg_read_batt_mode(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 batt_mode = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_MODE], &batt_mode);
	if (ret < 0) {
		bq_err("Could not read battery mode, ret = %d\n", ret);
		return ret;
	}

	*val = batt_mode;

	return 0;
}

#ifdef BQ40Z50_VERBOSE_DEBUG
static void print_raw_data(struct bq_fg_chip *bq, int remcap, int fullcap, int dsoc)
{
	u16 gsoc;
	u16 volt;
	u16 chg_volt;
	u16 chg_curr;
	int temp;
	int ret;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &gsoc);
	if (ret < 0) {
		bq_err("Could not read RSOC, ret = %d\n", ret);
		return;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("Could not read voltage, ret = %d\n", ret);
		return;
	}

	ret = fg_read_temperature(bq, &temp);
	if (ret < 0) {
		bq_err("Could not read temperature, ret = %d\n", ret);
		return;
	}

	if (remcap == -1) {
		ret = fg_read_remaining_capacity(bq, &remcap);
		if (ret < 0) {
			bq_err("Could not read remcap, ret = %d\n", ret);
			return;
		}
	}

	if (fullcap == -1) {
		ret = fg_read_fcc(bq, &fullcap);
		if (ret < 0) {
			bq_err("Could not read fullcap, ret = %d\n", ret);
			return;
		}
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_VOLT], &chg_volt);
	if (ret < 0) {
		bq_err("Could not read charging voltage, ret = %d\n", ret);
		return;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_CURR], &chg_curr);
	if (ret < 0) {
		bq_err("Could not read charging current, ret = %d\n", ret);
		return;
	}

	bq_err("GAUGE_INFO: %d, %d, %d.%d, %d.%d, %d.%d, %d.%d, %d, %d\n",
		remcap/1000, fullcap/1000, temp / 10, temp % 10,
		volt / 1000, volt % 1000, chg_volt / 1000, chg_volt % 1000,
		chg_curr / 1000, chg_curr % 1000, gsoc, dsoc);
}
#else
static inline void print_raw_data(struct bq_fg_chip *bq,
				int remcap, int fullcap, int dsoc) {}
#endif

/*
 * This would return display soc for remaining capacity
 * within src_b ~ src_t by maping to des_b ~ des_t
 */
static int fg_get_dsoc_from_remcap(struct bq_fg_chip *bq,
		int src_b, int src_t, int des_b, int des_t, u16 *val)
{
	int rcap;
	int fcap;
	int bottom_cap; /* capatity for 0 ~ src_b - 1 */
	int top_cap;	/* capacity for 0 ~ src_t */
	u16 dsoc;	/* display soc */
	int ret;

	ret = fg_read_remaining_capacity(bq, &rcap);
	if (ret < 0)
		return ret;

	ret = fg_read_fcc(bq, &fcap);
	if (ret < 0)
		return ret;

	/*
	 * to remove bottom area, get capacity
	 * for 0 ~ src_b - 1 area
	 */
	bottom_cap = (src_b - 1) * fcap / 100;

	/*
	 * Capacity for 0 ~ src_t
	 */
	top_cap = src_t * fcap / 100;

	if (rcap < bottom_cap || bottom_cap == top_cap) {
		bq_err("abnormal rcap %d, bottom %d, top %d -> return min\n",
			rcap, bottom_cap, top_cap);
		*val = des_b;
		return 0;
	}

	/*
	 * Get remaining capacity range in des_b ~ des_t range
	 * The range is des_t - des_b + 1
	 */
	dsoc = (((rcap - bottom_cap) *
			(des_t - des_b + 1)) /
			(top_cap - bottom_cap)) + des_b;

	/* dsoc should not be larger then des_t */
	if (dsoc > des_t)
		dsoc = des_t;

	bq_dbg("rcap = %d, fcap = %d, dsoc = %d\n", rcap, fcap, dsoc);

	print_raw_data(bq, rcap, fcap, dsoc);

	*val = dsoc;

	return 0;
}

/*
 * Remap gsoc using the threshold values
 */
static int fg_soc_static_remap(struct bq_fg_chip *bq, u16 gsoc, int *val)
{
	int ret = 0;
	u16 dsoc = 0;

	if (gsoc <= bq->soc_low_thr) {
		if (gsoc == 0)
			dsoc = 0;
		else
			dsoc = SOC_LOWBAT_LEVEL;
		print_raw_data(bq, -1, -1, dsoc);
	} else if (gsoc >= bq->soc_recharge_thr) {
		if (bq->soc_extra_thr) {
			if (gsoc >= bq->soc_recharge_thr + bq->soc_extra_thr) {
				dsoc = SOC_FULL_LEVEL;
				bq->soc_extra_thr = 0;
			} else {
				dsoc = SOC_FULL_LEVEL - 1;
			}
		} else {
			dsoc = SOC_FULL_LEVEL;
		}
		print_raw_data(bq, -1, -1, dsoc);
	} else {
		/*
		 * Map rcap in range soc_low_thr + 1 ~ soc_recharge_thr -1 to
		 * to (SOC_LOWBAT_LEVE + 1) ~ (SOC_FULL_LEVEL - 1)
		 */
		ret = fg_get_dsoc_from_remcap(bq,
			bq->soc_low_thr + 1, bq->soc_recharge_thr - 1,
			SOC_LOWBAT_LEVEL + 1, SOC_FULL_LEVEL - 1, &dsoc);
		if (ret < 0) {
			bq_err("fg_get_dsoc_from_remcap error rc = %d\n",
								ret);
			return ret;
		}

		/*
		 * When dsoc reaches SOC_FULL_LEVEL -1, it requires to
		 * one more gsoc capacity level to become SOC_FULL_LEVEL.
		 * This is to gaurantee that SOC_FULL_LEVEL can be maintained
		 * for at least one full gsoc capacity.
		 */
		if (dsoc == SOC_FULL_LEVEL - 1)
			bq->soc_extra_thr = 1;
	}

	bq_dbg("gsoc = %d, dsoc = %d\n", gsoc, dsoc);

	*val = dsoc;

	return 0;
}

static int fg_read_rsoc(struct bq_fg_chip *bq, int *val)
{
	int ret = 0;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("Could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	if (bq->soc_static_remap)
		ret = fg_soc_static_remap(bq, soc, val);
	else
		*val = soc;

	if (bq->batt_soc != *val)
		trace_read_rsoc(*val);

	return ret;
}

static int fg_read_rsoc_real(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("Could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	*val = soc;

	return ret;
}

/**
 * fg_read_temperature - Returns temperature in 0.1C.
 */
static int fg_read_temperature(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 temp = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_err("Could not read temperature, ret = %d\n", ret);
		return ret;
	}

	/* Value of temp is in range of 0 to 65535. Its unit is 0.1k */
	*val = temp - 2730;

	return ret;
}

/**
 *  fg_read_cell_volt - Returns the cell # voltage in uV.
 */
static int fg_read_cell_volt(struct bq_fg_chip *bq, int index, int *val)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[index], &volt);
	if (ret < 0) {
		bq_err("Could not read cell voltage, ret = %d\n", ret);
		return ret;
	}

	*val = volt * 1000;

	return 0;
}

/**
 * fg_read_volt - Returns the sum of the measured cell voltages in uV.
 */
static int fg_read_volt(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("Could not read voltage, ret = %d\n", ret);
		return ret;
	}

	*val = volt * 1000;

	return ret;
}

/**
 * fg_read_charging_voltage - Returns charging voltage in uV.
 */
static int fg_read_charging_voltage(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 chg_volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_VOLT], &chg_volt);
	if (ret < 0) {
		bq_err("Could not read charging voltage, ret = %d\n", ret);
		return ret;
	}

	*val = chg_volt * 1000;

	return 0;
}

/**
 * fg_read_current - Set curr to the measured current from the
 * coulomb counter in uA.
 */
static int fg_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	u16 curr_now = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CURR_NOW], &curr_now);
	if (ret < 0) {
		bq_err("Could not read current, ret = %d\n", ret);
		return ret;
	}

	/* The value of current is in the range of -32767~32768 */
	*curr = (int)((s16)curr_now) * 1000;

	return ret;
}

/**
 * fg_read_charging_current - Returns the desired charging current in uA.
 */
static int fg_read_charging_current(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 chg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CHG_CURR], &chg_curr);
	if (ret < 0) {
		bq_err("Could not read charging current, ret = %d\n", ret);
		return ret;
	}

	*val = chg_curr * 1000;

	return 0;
}

/**
 * fg_set_capm - Set CAPM bit in battery mode register to 0.
 */
static int fg_set_capm(struct bq_fg_chip *bq)
{
	int ret;
	int batt_mode;

	ret = fg_read_batt_mode(bq, &batt_mode);
	if (ret < 0) {
		bq_err("Could not read battery mode %d\n", ret);
		return ret;
	}

	if (!(batt_mode & 0x8000)) {
		/* Already using mAh, nothing to do */
		return 0;
	}

	batt_mode &= ~(1UL << 15);
	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_BATT_MODE], batt_mode);
	if (ret < 0) {
		bq_err("Could not write battery mode, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

/**
 * fg_read_fcc() - Returns the predicted battery capacity when fully
 * charged in uAh.
 */
static int fg_read_fcc(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 fcc;

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_err("FCC command not supported!\n");
		*val = 0;
		return 0;
	}

	if (!bq->report_in_mAh) {
		ret = fg_set_capm(bq);
		if (ret != 0) {
			bq_err("Could not set capm, ret = %d\n", ret);
			return ret;
		}

		bq->report_in_mAh = true;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0) {
		bq_err("Could not read FCC, ret=%d\n", ret);
		return ret;
	}

	*val = fcc * 1000;

	return 0;
}

/**
 * fg_read_design_capacity - Returns the theoretical pack capacity
 * when fully charged in uAh.
 */
static int fg_read_design_capacity(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 dc_mah;

	if (bq->regs[BQ_FG_REG_DESIGN_CAP] == INVALID_REG_ADDR) {
		bq_err("DesignCapacity command not supported!\n");
		*val = 0;
		return 0;
	}

	if (!bq->report_in_mAh) {
		ret = fg_set_capm(bq);
		if (ret != 0) {
			bq_err("Could not set capm, ret = %d\n", ret);
			return ret;
		}

		bq->report_in_mAh = true;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_DESIGN_CAP], &dc_mah);

	if (ret < 0) {
		bq_err("Could not read DC, ret=%d\n", ret);
		return ret;
	}

	*val = dc_mah * 1000;

	return 0;
}

/**
 * fg_read_remaining_capacity - Returns the predicted remaining
 * battery capacity in uAh.
 */
static int fg_read_remaining_capacity(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 rm_mah;

	if (bq->regs[BQ_FG_REG_REMAIN_CAP] == INVALID_REG_ADDR) {
		bq_err("RemainingCapacity command not supported!\n");
		*val = 0;
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_REMAIN_CAP], &rm_mah);

	if (ret < 0) {
		bq_err("Could not read DC, ret=%d\n", ret);
		return ret;
	}

	*val = rm_mah * 1000;

	return 0;
}

static int fg_read_cyclecount(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CYCLE_COUNT] == INVALID_REG_ADDR) {
		bq_err("Cycle Count not supported!\n");
		return -EINVAL;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CYCLE_COUNT], &cc);

	if (ret < 0) {
		bq_err("Could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	*val = cc;

	return 0;
}

/**
 * fg_read_time_to_empty - Returns the predicted minutes of run
 * time based on AverageCurrent() in seconds.
 */
static int fg_read_time_to_empty(struct bq_fg_chip *bq, int *val)
{
	int ret;
	u16 tte_min;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_err("Time To Empty not supported!\n");
		return -EINVAL;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte_min);

	if (ret < 0) {
		bq_err("Could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	*val = tte_min * 60;

	return ret;
}

static int fg_get_batt_status(struct bq_fg_chip *bq, int *val)
{
	int ret;

	ret = fg_read_status(bq);
	if (ret != 0) {
		bq_err("Could not read status, ret=%d -> Use cached values\n", ret);
	}

	if (bq->batt_fc)
		*val = POWER_SUPPLY_STATUS_FULL;
	else if (bq->batt_dsg)
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bq->batt_curr_now > 0)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int fg_get_batt_capacity_level(struct bq_fg_chip *bq, int *val)
{
	int ret;

	ret = fg_read_status(bq);
	if (ret != 0) {
		bq_err("Could not read status, ret=%d use cached values\n", ret);
	}

	if (bq->batt_fc)
		*val = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (bq->batt_rca)
		*val = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (bq->batt_fd)
		*val = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		*val = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	return 0;
}

static int fg_read_update_pfstatus(struct bq_fg_chip *bq)
{
	int ret = 0;
	__le32 val;

	ret = fg_read_param_to_sysfs(bq, &val, sizeof(val), 0,
						FG_MAC_CMD_PF_STATUS);
	if (ret < 0) {
		bq_err("Failed to read pf status:%d\n", ret);
		return ret;
	}

	if (bq->pf_status != __le32_to_cpu(val)) {
		bq->pf_status = __le32_to_cpu(val);
		trace_read_pf_status(bq->pf_status);
	}

	return 0;
}

static int fg_get_batt_init(struct bq_fg_chip *bq)
{
	int ret;

	ret = fg_read_status(bq);
	if (ret != 0) {
		bq_err("Could not read status, ret=%d\n", ret);
		return -EINVAL;
	}

	return bq->batt_init;
}

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_REAL_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/*POWER_SUPPLY_PROP_HEALTH,*/ /*implement it in battery power_supply*/
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_UPDATE_NOW,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_CELL_VOLTAGE_1,
	POWER_SUPPLY_PROP_CELL_VOLTAGE_2,
	POWER_SUPPLY_PROP_CELL_VOLTAGE_3,
	POWER_SUPPLY_PROP_CELL_VOLTAGE_4,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGING_CURRENT,
	POWER_SUPPLY_PROP_CHARGING_VOLTAGE,
	POWER_SUPPLY_PROP_REMAINING_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_BATTERY_MODE,
	POWER_SUPPLY_PROP_MANUFACCESS,
	POWER_SUPPLY_PROP_BATTERY_STATUS,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int fg_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACCESS:
		val->intval = bq->op_status;
		break;
	case POWER_SUPPLY_PROP_BATTERY_MODE:
		val->intval = bq->batt_mode;
		break;
	case POWER_SUPPLY_PROP_REMAINING_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq->batt_rm;
		break;
	case POWER_SUPPLY_PROP_CHARGING_VOLTAGE:
		val->intval = bq->chg_volt;
		break;
	case POWER_SUPPLY_PROP_CHARGING_CURRENT:
		val->intval = bq->chg_curr;
		break;
	case POWER_SUPPLY_PROP_CELL_VOLTAGE_1:
		val->intval = bq->batt_cell1_volt;
		break;
	case POWER_SUPPLY_PROP_CELL_VOLTAGE_2:
		val->intval = bq->batt_cell2_volt;
		break;
	case POWER_SUPPLY_PROP_CELL_VOLTAGE_3:
		val->intval = bq->batt_cell3_volt;
		break;
	case POWER_SUPPLY_PROP_CELL_VOLTAGE_4:
		val->intval = bq->batt_cell4_volt;
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* Not in ship mode as long as device is active */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_BATTERY_STATUS:
		val->intval = bq->batt_status;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!bq->batt_ok) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		val->intval = bq->batt_pwr_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!bq->batt_ok) {
			val->intval = 0;
			break;
		}
		ret = fg_read_volt(bq, &val->intval);
		mutex_lock(&bq->data_lock);
		if (ret == 0) {
			bq->batt_volt_now = val->intval;
		} else {
			val->intval = bq->batt_volt_now;
			ret = 0;
		}
		mutex_unlock(&bq->data_lock);

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!bq->batt_ok) {
			val->intval = 0;
			break;
		}
		mutex_lock(&bq->data_lock);
		ret = fg_read_current(bq, &val->intval);
		if (ret == 0) {
			bq->batt_curr_now = val->intval;
		} else {
			val->intval = bq->batt_curr_now;
			ret = 0;
		}
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		if (!bq->batt_ok) {
			val->intval = 50;
			break;
		}
		val->intval = bq->batt_soc;
		break;
	case POWER_SUPPLY_PROP_REAL_CAPACITY:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		val->intval = bq->batt_soc_real;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = bq->batt_cap_lvl;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}
		val->intval = bq->batt_temp;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		if (!bq->batt_ok) {
			val->intval = 0;
			break;
		}
		ret = fg_read_time_to_empty(bq, &val->intval);
		if (ret == 0) {
			bq->batt_tte = val->intval;
		} else {
			val->intval = bq->batt_tte;
			ret = 0;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq->batt_fcc;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq->batt_dc;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bq->batt_cyclecnt;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (!bq->batt_ok) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			if (bq->batt_detect_count > MAX_INIT_RETRY_COUNT)
				val->intval = POWER_SUPPLY_HEALTH_DEAD;
			break;
		}

		fg_read_update_pfstatus(bq);
		if (bq->pf_status != 0) {
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		} else if (bq->batt_ss & FG_FLAGS_UTD ||
			bq->batt_ss & FG_FLAGS_UTC) {
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		} else if (bq->batt_ss & FG_FLAGS_OTD ||
			bq->batt_ss & FG_FLAGS_OTC) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = bq->manuf_name;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->device_name;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int fg_set_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(bq->fg_psy);
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		if (bq->batt_ok) {
			alarm_cancel(&bq->monitor_alarm);
			cancel_delayed_work_sync(&bq->monitor_work);

			fg_update_status(bq);

			/*
			 * Make sure we don't hold wakelock
			 * even in case the work thread has been
			 * canceled
			 */
			if (bq->pm_stay_awake) {
				WARN(1, "Call pm_relax to release wakelock\n");
				bq->pm_stay_awake = false;
				pm_relax(bq->dev);
			}

			alarm_start_relative(&bq->monitor_alarm,
				ms_to_ktime(POLL_INTERVAL_MS));
		}
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		if (val->intval)
			fg_enable_shutdown_mode(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fg_prop_is_writeable(struct power_supply *psy,
				enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int fg_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config fg_psy_cfg = {};

	bq->fg_psy_d.name = "bq40z50_battery";
	bq->fg_psy_d.type = POWER_SUPPLY_TYPE_BATTERY; //POWER_SUPPLY_TYPE_BMS;
	bq->fg_psy_d.properties = fg_props;
	bq->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy_d.get_property = fg_get_property;
	bq->fg_psy_d.set_property = fg_set_property;
	bq->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = bq;
	fg_psy_cfg.num_supplicants = 0;
	bq->fg_psy =
		devm_power_supply_register(bq->dev, &bq->fg_psy_d, &fg_psy_cfg);
	if (IS_ERR(bq->fg_psy)) {
		bq_err("Failed to register fg_psy");
		return PTR_ERR(bq->fg_psy);
	}
	return 0;
}

static void fg_psy_unregister(struct bq_fg_chip *bq)
{
	power_supply_unregister(bq->fg_psy);
}

static ssize_t fg_read_pfstatus(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	if (fg_read_update_pfstatus(bq))
		bq_err("Updating pfstatus failed. Return cached PF val");

	return snprintf(buf, PAGE_SIZE, "%d\n",
				bq->pf_status);
}

static ssize_t fg_read_charging_status(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret = 0;
	__le32 val;

	ret = fg_read_param_to_sysfs(bq, &val, sizeof(val), 0,
						FG_MAC_CMD_CHARGING_STATUS);
	if (ret < 0) {
		bq_err("Failed to read charging status:%d\n", ret);
		return ret;
	}

	if (bq->charging_status != __le32_to_cpu(val)) {
		bq->charging_status = __le32_to_cpu(val);
		trace_read_charging_status(bq->charging_status);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n",
				__le32_to_cpu(val));
}

static ssize_t fg_read_state_of_health(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret = 0;
	u16 soh;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOH], &soh);
	if (ret < 0) {
		bq_err("Could not read battery state of health, ret = %d\n",
									ret);
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", __le16_to_cpu(soh));
}

static const struct fg_ldb_prop {
	const char *name;
	int ldb_offs;
	int ldbi; /* lifetime data block index */
	int num_params; /* Number of values to extract and print */
	GG_VAL_TYPE data_type;
	int multiplier; /* Convert raw data to appropriate units */
} fg_ldb_props[] = {
	{ "cell_balance_time", 0 /* offset 0 */, FG_MAC_CMD_CB_STATUS,
		NUM_CELLS, GG_U4, 1},
	{ "ts_temperature1", 2 /* offset 2 */, FG_MAC_DA_STATUS_2,
		1, GG_I2, 1},
	{ "ts_temperature2", 4 /* offset 2 */, FG_MAC_DA_STATUS_2,
		1, GG_I2, 1},
	{ "ts_temperature3", 6 /* offset 2 */, FG_MAC_DA_STATUS_2,
		1, GG_I2, 1},
	{ "ts_temperature4", 8 /* offset 2 */, FG_MAC_DA_STATUS_2,
		1, GG_I2, 1},
	{ "cell_max_volt", FG_MAX_CELL_VOLTAGE, FG_MAC_CMD_LDB_1,
		NUM_CELLS, GG_I2, 1000},
	{ "cell_min_volt", FG_MIN_CELL_VOLTAGE, FG_MAC_CMD_LDB_1,
		NUM_CELLS, GG_I2, 1000},
	{ "max_delta_cell_volt", FG_MAX_DELTA_CELL_VOLT, FG_MAC_CMD_LDB_1,
		1, GG_I2, 1000},
	{ "max_charge_current", FG_MAX_CHARGE_CURRENT, FG_MAC_CMD_LDB_1,
		1, GG_I2, 1000},
	{ "max_discharge_current", FG_MAX_DISCHARGE_CURRENT, FG_MAC_CMD_LDB_1,
		1, GG_I2, 1000},
	{ "max_avg_dsg_current", FG_MAX_AVG_DSG_CURRENT, FG_MAC_CMD_LDB_1,
		1, GG_I2, 1000},
	{ "max_avg_dsg_power", FG_MAX_AVG_DSG_POWER, FG_MAC_CMD_LDB_1,
		1, GG_I2, 1},
	{ "max_temp_cell", FG_MAX_TEMP_CELL, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "min_temp_cell", FG_MIN_TEMP_CELL, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "max_delta_cell_temp", FG_MAX_DELTA_CELL_TEMP, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "max_temp_int_sensor", FG_MAX_TEMP_INT_SENSOR, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "min_temp_int_sensor", FG_MIN_TEMP_INT_SENSOR, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "max_temp_fet", FG_MAX_TEMP_FET, FG_MAC_CMD_LDB_1,
		1, GG_I1, 1}, /* vendor specific, use Celsius deliberately */
	{ "number_shutdowns", FG_NUM_SHUTDOWNS, FG_MAC_CMD_LDB_2,
		1, GG_I1, 1},
	{ "tot_fw_runtime", FG_FW_RUNTIME, FG_MAC_CMD_LDB_3,
		1, GG_U4, 1},
	{ "time_spent_ut_a", FG_TIME_SPENT_UT_A, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_b", FG_TIME_SPENT_UT_B, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_c", FG_TIME_SPENT_UT_C, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_d", FG_TIME_SPENT_UT_D, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_e", FG_TIME_SPENT_UT_E, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_f", FG_TIME_SPENT_UT_F, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_g", FG_TIME_SPENT_UT_G, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_ut_h", FG_TIME_SPENT_UT_H, FG_MAC_CMD_LDB_6,
		1, GG_U4, 1},
	{ "time_spent_lt_a", FG_TIME_SPENT_LT_A, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_b", FG_TIME_SPENT_LT_B, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_c", FG_TIME_SPENT_LT_C, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_d", FG_TIME_SPENT_LT_D, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_e", FG_TIME_SPENT_LT_E, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_f", FG_TIME_SPENT_LT_F, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_g", FG_TIME_SPENT_LT_G, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_lt_h", FG_TIME_SPENT_LT_H, FG_MAC_CMD_LDB_7,
		1, GG_U4, 1},
	{ "time_spent_stl_a", FG_TIME_SPENT_STL_A, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_b", FG_TIME_SPENT_STL_B, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_c", FG_TIME_SPENT_STL_C, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_d", FG_TIME_SPENT_STL_D, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_e", FG_TIME_SPENT_STL_E, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_f", FG_TIME_SPENT_STL_F, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_g", FG_TIME_SPENT_STL_G, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_stl_h", FG_TIME_SPENT_STL_H, FG_MAC_CMD_LDB_8,
		1, GG_U4, 1},
	{ "time_spent_rt_a", FG_TIME_SPENT_RT_A, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_b", FG_TIME_SPENT_RT_B, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_c", FG_TIME_SPENT_RT_C, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_d", FG_TIME_SPENT_RT_D, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_e", FG_TIME_SPENT_RT_E, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_f", FG_TIME_SPENT_RT_F, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_g", FG_TIME_SPENT_RT_G, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_rt_h", FG_TIME_SPENT_RT_H, FG_MAC_CMD_LDB_9,
		1, GG_U4, 1},
	{ "time_spent_sth_a", FG_TIME_SPENT_STH_A, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_b", FG_TIME_SPENT_STH_B, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_c", FG_TIME_SPENT_STH_C, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_d", FG_TIME_SPENT_STH_D, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_e", FG_TIME_SPENT_STH_E, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_f", FG_TIME_SPENT_STH_F, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_g", FG_TIME_SPENT_STH_G, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_sth_h", FG_TIME_SPENT_STH_H, FG_MAC_CMD_LDB_10,
		1, GG_U4, 1},
	{ "time_spent_ht_a", FG_TIME_SPENT_HT_A, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_b", FG_TIME_SPENT_HT_B, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_c", FG_TIME_SPENT_HT_C, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_d", FG_TIME_SPENT_HT_D, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_e", FG_TIME_SPENT_HT_E, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_f", FG_TIME_SPENT_HT_F, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_g", FG_TIME_SPENT_HT_G, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ht_h", FG_TIME_SPENT_HT_H, FG_MAC_CMD_LDB_11,
		1, GG_U4, 1},
	{ "time_spent_ot_a", FG_TIME_SPENT_OT_A, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_b", FG_TIME_SPENT_OT_B, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_c", FG_TIME_SPENT_OT_C, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_d", FG_TIME_SPENT_OT_D, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_e", FG_TIME_SPENT_OT_E, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_f", FG_TIME_SPENT_OT_F, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_g", FG_TIME_SPENT_OT_G, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "time_spent_ot_h", FG_TIME_SPENT_OT_H, FG_MAC_CMD_LDB_12,
		1, GG_U4, 1},
	{ "num_cov_events", FG_NUM_COV_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_cov_event", FG_LAST_COV_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_cuv_events", FG_NUM_CUV_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_cuv_event", FG_LAST_CUV_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_ocd1_events", FG_NUM_OCD1_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_ocd1_event", FG_LAST_OCD1_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_ocd2_events", FG_NUM_OCD2_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_ocd2_event", FG_LAST_OCD2_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_occ1_events", FG_NUM_OCC1_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_occ1_event", FG_LAST_OCC1_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_occ2_events", FG_NUM_OCC2_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_occ2_event", FG_LAST_OCC2_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_aold_events", FG_NUM_AOLD_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_aold_event", FG_LAST_AOLD_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_ascd_events", FG_NUM_ASCD_EVENTS, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "last_ascd_event", FG_LAST_ASCD_EVENT, FG_MAC_CMD_LDB_4,
		1, GG_U2, 1},
	{ "num_ascc_events", FG_NUM_ASCC_EVENTS, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "last_ascc_event", FG_LAST_ASCC_EVENT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "num_otc_events", FG_NUM_OTC_EVENTS, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "last_otc_event", FG_LAST_OTC_EVENT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "num_otd_events", FG_NUM_OTD_EVENTS, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "last_otd_event", FG_LAST_OTD_EVENT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "num_otf_events", FG_NUM_OTF_EVENTS, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "last_otf_event", FG_LAST_OTF_EVENT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "number_vct", FG_NUMBER_VCT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
	{ "last_vct", FG_LAST_VCT, FG_MAC_CMD_LDB_5,
		1, GG_U2, 1},
};

int gg_type_size(int type)
{
	switch (type) {
	case GG_U4: return 4;
	case GG_U2: return 2;
	case GG_I2: return 2;
	case GG_I1: return 1;
	}

	return 0;
}

static int print_params_to_sysfs(char *buf, u8 *param, int mult,
					int data_type, int num_param)
{
	switch (num_param) {
	case 1:
		switch (data_type) {
		case GG_I1:
			return snprintf(buf, PAGE_SIZE, "%d\n",
			(int8_t)(*param)*mult);
		case GG_U4:
			return snprintf(buf, PAGE_SIZE, "%u\n",
			(uint32_t)get_unaligned_le32(param)*mult);
		case GG_U2:
			return snprintf(buf, PAGE_SIZE, "%u\n",
			(uint16_t)get_unaligned_le16(param)*mult);
		case GG_I2:
			return snprintf(buf, PAGE_SIZE, "%d\n",
			(int16_t)get_unaligned_le16(param)*mult);
		/* FALLTHROUGH */
		default:
			return -EINVAL;
		}
	case 4:
		switch (data_type) {
		case GG_I1:
			return snprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
			(int8_t)param[0]*mult,
			(int8_t)param[1]*mult,
			(int8_t)param[2]*mult,
			(int8_t)param[3]*mult);
		case GG_U4:
			return snprintf(buf, PAGE_SIZE, "%u, %u, %u, %u\n",
			(uint32_t)get_unaligned_le32(&param[0])*mult,
			(uint32_t)get_unaligned_le32(&param[4])*mult,
			(uint32_t)get_unaligned_le32(&param[8])*mult,
			(uint32_t)get_unaligned_le32(&param[12])*mult);
		case GG_U2:
			return snprintf(buf, PAGE_SIZE, "%u, %u, %u, %u\n",
			(uint16_t)get_unaligned_le16(&param[0])*mult,
			(uint16_t)get_unaligned_le16(&param[2])*mult,
			(uint16_t)get_unaligned_le16(&param[4])*mult,
			(uint16_t)get_unaligned_le16(&param[6])*mult);
		case GG_I2:
			return snprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
			(int16_t)get_unaligned_le16(&param[0])*mult,
			(int16_t)get_unaligned_le16(&param[2])*mult,
			(int16_t)get_unaligned_le16(&param[4])*mult,
			(int16_t)get_unaligned_le16(&param[6])*mult);
		/* FALLTHROUGH */
		default:
			return -EINVAL;
		}
	/* FALLTHROUGH */
	default:
		return -EINVAL;
	}
}

static ssize_t fg_read_ldb_prop(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	int i = 0;
	u8 tmpbuf[NUM_CELLS*4]; /* Total 16 bytes to hold 4 parameters */

	memset(tmpbuf, 0, sizeof(tmpbuf));

	for (i = 0; i < ARRAY_SIZE(fg_ldb_props); i++) {
		if (strcmp(attr->attr.name, fg_ldb_props[i].name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(fg_ldb_props)) {
		bq_err("Attribute not listed in the table!\n");
		return -EINVAL;
	}

	ret = fg_read_param_to_sysfs(i2c_get_clientdata(to_i2c_client(dev)),
		&tmpbuf[0], fg_ldb_props[i].num_params *
					gg_type_size(fg_ldb_props[i].data_type),
		fg_ldb_props[i].ldb_offs, fg_ldb_props[i].ldbi);
	if (ret < 0)
		return ret;

	return print_params_to_sysfs(buf, tmpbuf, fg_ldb_props[i].multiplier,
		fg_ldb_props[i].data_type, fg_ldb_props[i].num_params);
}


static ssize_t fg_read_ldb_prop_temp(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	int i = 0;
	u8 tmpbuf[2];

	memset(tmpbuf, 0, sizeof(tmpbuf));

	for (i = 0; i < ARRAY_SIZE(fg_ldb_props); i++) {
		if (strcmp(attr->attr.name, fg_ldb_props[i].name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(fg_ldb_props)) {
		bq_err("Attribute not listed in the table!\n");
		return -EINVAL;
	}
	if (fg_ldb_props[i].num_params != 1) {
		bq_err("Invalid num_params value: Exp=1 Val=%d",
			fg_ldb_props[i].num_params);
		return -EINVAL;
	}

	if (fg_ldb_props[i].data_type != GG_I2) {
		bq_err("Invalid num_params value. Ex=%d Val=%d",
			GG_I2, fg_ldb_props[i].data_type);
		return -EINVAL;
	}

	ret = fg_read_param_to_sysfs(i2c_get_clientdata(to_i2c_client(dev)),
		&tmpbuf[0], gg_type_size(GG_I2),
		fg_ldb_props[i].ldb_offs, fg_ldb_props[i].ldbi);

	if (ret < 0)
		return ret;

	/* Value of temp is in range of 0 to 65535. Its unit is 0.1k */
	/* Convert it to 0.1C */
	return snprintf(buf, PAGE_SIZE, "%d\n",
			((int16_t)get_unaligned_le16(tmpbuf) - 2730));
}

static DEVICE_ATTR(cell_max_volt, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(cell_min_volt, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_delta_cell_volt, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_charge_current, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_discharge_current, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_avg_dsg_current, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_avg_dsg_power, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_temp_cell, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(min_temp_cell, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_delta_cell_temp, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_temp_int_sensor, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(min_temp_int_sensor, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(max_temp_fet, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(number_shutdowns, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(pf_status, 0444, fg_read_pfstatus, NULL);
static DEVICE_ATTR(charging_status, 0444, fg_read_charging_status, NULL);
static DEVICE_ATTR(tot_fw_runtime, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(number_vct, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_vct, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(cell_balance_time, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(state_of_health, 0444, fg_read_state_of_health, NULL);
static DEVICE_ATTR(ts_temperature1, 0444, fg_read_ldb_prop_temp, NULL);
static DEVICE_ATTR(ts_temperature2, 0444, fg_read_ldb_prop_temp, NULL);
static DEVICE_ATTR(ts_temperature3, 0444, fg_read_ldb_prop_temp, NULL);
static DEVICE_ATTR(ts_temperature4, 0444, fg_read_ldb_prop_temp, NULL);
static DEVICE_ATTR(time_spent_ut_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ut_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_lt_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_stl_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_rt_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_sth_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ht_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_a, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_b, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_c, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_d, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_e, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_f, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_g, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(time_spent_ot_h, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_cov_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_cov_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_cuv_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_cuv_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_ocd1_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_ocd1_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_ocd2_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_ocd2_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_occ1_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_occ1_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_occ2_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_occ2_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_aold_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_aold_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_ascd_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_ascd_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_ascc_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_ascc_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_otc_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_otc_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_otd_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_otd_event, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(num_otf_events, 0444, fg_read_ldb_prop, NULL);
static DEVICE_ATTR(last_otf_event, 0444, fg_read_ldb_prop, NULL);

static struct attribute *fg_attributes[] = {
	&dev_attr_cell_max_volt.attr,
	&dev_attr_cell_min_volt.attr,
	&dev_attr_max_delta_cell_volt.attr,
	&dev_attr_max_charge_current.attr,
	&dev_attr_max_discharge_current.attr,
	&dev_attr_max_avg_dsg_current.attr,
	&dev_attr_max_avg_dsg_power.attr,
	&dev_attr_max_temp_cell.attr,
	&dev_attr_min_temp_cell.attr,
	&dev_attr_max_delta_cell_temp.attr,
	&dev_attr_max_temp_int_sensor.attr,
	&dev_attr_min_temp_int_sensor.attr,
	&dev_attr_max_temp_fet.attr,
	&dev_attr_number_shutdowns.attr,
	&dev_attr_pf_status.attr,
	&dev_attr_charging_status.attr,
	&dev_attr_tot_fw_runtime.attr,
	&dev_attr_number_vct.attr,
	&dev_attr_last_vct.attr,
	&dev_attr_cell_balance_time.attr,
	&dev_attr_state_of_health.attr,
	&dev_attr_ts_temperature1.attr,
	&dev_attr_ts_temperature2.attr,
	&dev_attr_ts_temperature3.attr,
	&dev_attr_ts_temperature4.attr,
	&dev_attr_time_spent_ut_a.attr,
	&dev_attr_time_spent_ut_b.attr,
	&dev_attr_time_spent_ut_c.attr,
	&dev_attr_time_spent_ut_d.attr,
	&dev_attr_time_spent_ut_e.attr,
	&dev_attr_time_spent_ut_f.attr,
	&dev_attr_time_spent_ut_g.attr,
	&dev_attr_time_spent_ut_h.attr,
	&dev_attr_time_spent_lt_a.attr,
	&dev_attr_time_spent_lt_b.attr,
	&dev_attr_time_spent_lt_c.attr,
	&dev_attr_time_spent_lt_d.attr,
	&dev_attr_time_spent_lt_e.attr,
	&dev_attr_time_spent_lt_f.attr,
	&dev_attr_time_spent_lt_g.attr,
	&dev_attr_time_spent_lt_h.attr,
	&dev_attr_time_spent_stl_a.attr,
	&dev_attr_time_spent_stl_b.attr,
	&dev_attr_time_spent_stl_c.attr,
	&dev_attr_time_spent_stl_d.attr,
	&dev_attr_time_spent_stl_e.attr,
	&dev_attr_time_spent_stl_f.attr,
	&dev_attr_time_spent_stl_g.attr,
	&dev_attr_time_spent_stl_h.attr,
	&dev_attr_time_spent_rt_a.attr,
	&dev_attr_time_spent_rt_b.attr,
	&dev_attr_time_spent_rt_c.attr,
	&dev_attr_time_spent_rt_d.attr,
	&dev_attr_time_spent_rt_e.attr,
	&dev_attr_time_spent_rt_f.attr,
	&dev_attr_time_spent_rt_g.attr,
	&dev_attr_time_spent_rt_h.attr,
	&dev_attr_time_spent_sth_a.attr,
	&dev_attr_time_spent_sth_b.attr,
	&dev_attr_time_spent_sth_c.attr,
	&dev_attr_time_spent_sth_d.attr,
	&dev_attr_time_spent_sth_e.attr,
	&dev_attr_time_spent_sth_f.attr,
	&dev_attr_time_spent_sth_g.attr,
	&dev_attr_time_spent_sth_h.attr,
	&dev_attr_time_spent_ht_a.attr,
	&dev_attr_time_spent_ht_b.attr,
	&dev_attr_time_spent_ht_c.attr,
	&dev_attr_time_spent_ht_d.attr,
	&dev_attr_time_spent_ht_e.attr,
	&dev_attr_time_spent_ht_f.attr,
	&dev_attr_time_spent_ht_g.attr,
	&dev_attr_time_spent_ht_h.attr,
	&dev_attr_time_spent_ot_a.attr,
	&dev_attr_time_spent_ot_b.attr,
	&dev_attr_time_spent_ot_c.attr,
	&dev_attr_time_spent_ot_d.attr,
	&dev_attr_time_spent_ot_e.attr,
	&dev_attr_time_spent_ot_f.attr,
	&dev_attr_time_spent_ot_g.attr,
	&dev_attr_time_spent_ot_h.attr,
	&dev_attr_num_cov_events.attr,
	&dev_attr_last_cov_event.attr,
	&dev_attr_num_cuv_events.attr,
	&dev_attr_last_cuv_event.attr,
	&dev_attr_num_ocd1_events.attr,
	&dev_attr_last_ocd1_event.attr,
	&dev_attr_num_ocd2_events.attr,
	&dev_attr_last_ocd2_event.attr,
	&dev_attr_num_occ1_events.attr,
	&dev_attr_last_occ1_event.attr,
	&dev_attr_num_occ2_events.attr,
	&dev_attr_last_occ2_event.attr,
	&dev_attr_num_aold_events.attr,
	&dev_attr_last_aold_event.attr,
	&dev_attr_num_ascd_events.attr,
	&dev_attr_last_ascd_event.attr,
	&dev_attr_num_ascc_events.attr,
	&dev_attr_last_ascc_event.attr,
	&dev_attr_num_otc_events.attr,
	&dev_attr_last_otc_event.attr,
	&dev_attr_num_otd_events.attr,
	&dev_attr_last_otd_event.attr,
	&dev_attr_num_otf_events.attr,
	&dev_attr_last_otf_event.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static void fg_update_status(struct bq_fg_chip *bq)
{
	int ret;
	int val;
	bool print_log = false;

	if (!bq->batt_ok)
		return;
	mutex_lock(&bq->data_lock);

	ret = fg_read_op_status(bq, &val);
	if (!ret)
		bq->op_status = val;

	ret = fg_read_batt_mode(bq, &val);
	if (!ret)
		bq->batt_mode = val;

	bq->chg_volt = fg_read_charging_voltage(bq, &val);
	if (!ret)
		bq->chg_volt = val;

	ret = fg_read_charging_current(bq, &val);
	if (!ret)
		bq->chg_curr = val;

	ret = fg_read_battery_status(bq, &val);
	if (!ret)
		bq->batt_status = val;

	ret = fg_read_cell_volt(bq, BQ_FG_REG_CELL_VOLT1, &val);
	if (!ret)
		bq->batt_cell1_volt = val;

	ret = fg_read_cell_volt(bq, BQ_FG_REG_CELL_VOLT2, &val);
	if (!ret)
		bq->batt_cell2_volt = val;

	ret = fg_read_cell_volt(bq, BQ_FG_REG_CELL_VOLT3, &val);
	if (!ret)
		bq->batt_cell3_volt = val;

	ret = fg_read_cell_volt(bq, BQ_FG_REG_CELL_VOLT4, &val);
	if (!ret)
		bq->batt_cell4_volt = val;

	ret = fg_get_batt_status(bq, &val);
	if (!ret)
		bq->batt_pwr_status = val;

	ret = fg_get_batt_capacity_level(bq, &val);
	if (!ret)
		bq->batt_cap_lvl = val;

	ret = fg_read_temperature(bq, &val);
	if (!ret && bq->batt_temp != val) {
		bq->batt_temp = val;
		print_log = true;
	}

	ret = fg_read_fcc(bq, &val);
	if (!ret)
		bq->batt_fcc = val;

	ret = fg_read_design_capacity(bq, &val);
	if (!ret)
		bq->batt_dc = val;

	ret = fg_read_cyclecount(bq, &val);
	if (!ret)
		bq->batt_cyclecnt = val;

	ret = fg_read_rsoc(bq, &val);
	if (!ret && bq->batt_soc != val) {
		bq->batt_soc = val;
		print_log = true;
	}

	ret = fg_read_rsoc_real(bq, &val);
	if (!ret)
		bq->batt_soc_real = val;

	ret = fg_read_volt(bq, &val);
	if (!ret && bq->batt_volt_now != val) {
		bq->batt_volt_now = val;
		print_log = true;
	}

	ret = fg_read_current(bq, &val);
	if (!ret && bq->batt_curr_now != val) {
		bq->batt_curr_now = val;
		print_log = true;
	}

	ret = fg_read_remaining_capacity(bq, &val);
	if (!ret)
		bq->batt_rm = val;

	ret = fg_read_ss(bq, &val);
	if (!ret && bq->batt_ss != val) {
		bq->batt_ss = val;
		print_log = true;
	}

	ret = fg_read_chrg_status(bq, &val);
	if (!ret)
		bq->charging_status = val;

	mutex_unlock(&bq->data_lock);
	if (print_log)
		bq_log("RSOC:%d, Volt:%d, Current:%d, Temperature:%d SS%d\n",
		bq->batt_soc, bq->batt_volt_now, bq->batt_curr_now,
			bq->batt_temp, bq->batt_ss);
}

static void fg_monitor_workfunc(struct work_struct *work)
{
	int ret;
	struct bq_fg_chip *bq =
		container_of(work, struct bq_fg_chip, monitor_work.work);

	if (!bq->batt_ok)
		return;

	if (!bq->resume_completed) {
		ret = wait_for_completion_timeout(
			&bq->resume_done,
			msecs_to_jiffies(BQ40Z50_RESUME_TIMEOUT_MS));
		if (!ret) {
			bq_err("resume didn't happen\n");
			goto exit;
		}
	}
	fg_update_status(bq);

exit:
	alarm_start_relative(&bq->monitor_alarm,
		ms_to_ktime(POLL_INTERVAL_MS));

	bq->pm_stay_awake = false;
	pm_relax(bq->dev);
}

static enum alarmtimer_restart fg_monitor_alarmcb(struct alarm *alarm,
							ktime_t now)
{
	struct bq_fg_chip *bq =
		container_of(alarm, struct bq_fg_chip, monitor_alarm);

	pm_stay_awake(bq->dev);
	bq->pm_stay_awake = true;
	schedule_delayed_work(&bq->monitor_work, 0);

	return ALARMTIMER_NORESTART;
}

static void determine_initial_status(struct bq_fg_chip *bq)
{
	int gsoc = 0;
	int status = 0;

	fg_read_rsoc_real(bq, &gsoc);
	fg_get_batt_status(bq, &status);

	/*
	 * If gsoc is the same as recharge threshold and charging
	 * was not complete, then extra threshold must have been set
	 * before reboot. In that case, set the extra threshold to manage
	 * the correct 100% dsoc capacity
	 */
	if (gsoc == bq->soc_recharge_thr && status != POWER_SUPPLY_STATUS_FULL)
		bq->soc_extra_thr = 1;

	fg_update_status(bq);
}

static int fg_parse_dt(struct bq_fg_chip *bq)
{
	struct device_node *node = bq->dev->of_node;
	int ret;

	if (!node) {
		bq_dbg("no device tree node\n");
		return -ENXIO;
	}

	ret = of_property_read_u32(node, "ti,fg-soc-recharge-thr",
					&bq->soc_recharge_thr);
	if (ret < 0)
		bq_dbg("no ti,fg-soc-recharge-thr definition\n");

	ret = of_property_read_u32(node, "ti,fg-soc-low-thr",
					&bq->soc_low_thr);
	if (ret < 0)
		bq_dbg("no ti,fg-soc-low-thr definition\n");

	/*
	 * We check the fg-soc-static-remap feature only when
	 * the related parameters are valid
	 */
	if (bq->soc_recharge_thr >= SOC_MIN_RECHARGE_THR &&
			bq->soc_recharge_thr <= SOC_MAX_RECHARGE_THR &&
			bq->soc_low_thr >= SOC_MIN_LOW_THR &&
			bq->soc_low_thr <= SOC_MAX_LOW_THR)
		bq->soc_static_remap = of_property_read_bool(node,
						"ti,fg-soc-static-remap");

	return 0;
}

static bool bq_fg_detect_battery(struct bq_fg_chip *bq)
{
	int ret;

	ret = fg_get_batt_init(bq);
	if (ret < 0) {
		bq_err("Failed to read battery status, ret = %d\n", ret);
		return false;
	} else if (ret == 0) {
		bq_err("bq fuel gauge initialization is in progress");
		return false;
	}

	bq_log("bq fuel gauge initialization is complete");

	ret = fg_read_fw_version(bq);
	if (ret) {
		bq_err("Failed to get fw version, ret = %d\n", ret);
		return false;
	}

	ret = fg_read_manuf_name(bq);
	if (ret) {
		bq_err("Failed to get manufacturer name, ret = %d\n", ret);
		return ret;
	}

	ret = fg_read_device_name(bq);
	if (ret) {
		bq_err("Failed to get device name, ret = %d\n", ret);
		return ret;
	}

	return true;
}

static int bq_fg_init(struct bq_fg_chip *bq)
{
	int ret;

	/* Set CAPM to zero, to report in mAh */
	ret = fg_set_capm(bq);
	if (ret == 0)
		bq->report_in_mAh = true;

	bq->resume_completed = true;

	device_init_wakeup(bq->dev, 1);

	determine_initial_status(bq);

	bq_log("bq fuel gauge battery detected, %s\n", device2str[bq->chip]);

	ret = sysfs_create_group(&bq->dev->kobj, &fg_attr_group);
	if (ret)
		bq_err("Failed to register sysfs, err:%d\n", ret);

	alarm_init(&bq->monitor_alarm,
			ALARM_BOOTTIME, fg_monitor_alarmcb);
	init_completion(&bq->resume_done);

	alarm_start_relative(&bq->monitor_alarm,
			ms_to_ktime(POLL_INTERVAL_MS));
	bq->batt_ok = true;
	return 0;
}

/**
 * When any single voltage at pack pin < Vstartup (2.1V), battery will enter
 * shutdown mode from normal mode automatically.  To wakeup battery method is
 * that send charging signal and charge battery, let all of single voltage at
 * pack pin > 2.1V.  From experiment result, we use worse case of 6.2V battery
 * which calculate 4 cells, it recovery gauge i2c communication take 3~5
 * seconds.  Therefore, we use MAX_INIT_RETRY_COUNT to waiting gauge i2c
 * communication recovery.
 */
static void fg_delayed_init_workfunc(struct work_struct *work)
{
	struct bq_fg_chip *bq =
		container_of(work, struct bq_fg_chip, delayed_init_work.work);

	if (bq_fg_detect_battery(bq) == true) {
		bq_fg_init(bq);
		return;
	}

	bq->batt_detect_count++;
	if (bq->batt_detect_count > MAX_INIT_RETRY_COUNT)
		bq_err("Battery not detected, giving up!\n");
	else
		schedule_delayed_work(&bq->delayed_init_work, 5 * HZ);
}

static int bq_fg_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	struct bq_fg_chip *bq;
	u8 *regs;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->batt_soc = -ENODATA;
	bq->batt_ss = -ENODATA;
	bq->batt_fcc = -ENODATA;
	bq->batt_rm = -ENODATA;
	bq->batt_dc = -ENODATA;
	bq->batt_volt_now = -ENODATA;
	bq->batt_temp = -ENODATA;
	bq->batt_curr_now = -ENODATA;
	bq->batt_cyclecnt = -ENODATA;
	bq->op_status = -ENODATA;
	bq->pf_status = -ENODATA;
	bq->charging_status = -ENODATA;
	bq->batt_mode = -ENODATA;
	bq->chg_volt = -ENODATA;
	bq->chg_curr = -ENODATA;
	bq->batt_status = -ENODATA;
	bq->batt_cell1_volt = -ENODATA;
	bq->batt_cell2_volt = -ENODATA;
	bq->batt_cell3_volt = -ENODATA;
	bq->batt_cell4_volt = -ENODATA;
	bq->batt_pwr_status = -ENODATA;
	bq->batt_cap_lvl = -ENODATA;

	bq->fake_soc = -EINVAL;
	bq->fake_temp = -EINVAL;
	bq->safety_status = -EINVAL;

	if (bq->chip == BQ40Z50) {
		regs = bq40z50_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq40z50_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	ret = fg_parse_dt(bq);
	if (ret < 0) {
		bq_err("Failed to parse DT parameters, ret = %d\n", ret);
	}

	mutex_init(&bq->data_lock);

	INIT_DELAYED_WORK(&bq->monitor_work, fg_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->delayed_init_work, fg_delayed_init_workfunc);

	fg_psy_register(bq);

	bq_log("bq fuel gauge probe successfully, %s\n", device2str[bq->chip]);

	schedule_delayed_work(&bq->delayed_init_work, 0);
	return 0;
}

static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	bq->resume_completed = false;

	reinit_completion(&bq->resume_done);
	return 0;
}

static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	bq->resume_completed = true;

	power_supply_changed(bq->fg_psy);

	complete(&bq->resume_done);

	return 0;
}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	device_init_wakeup(bq->dev, false);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);

	sysfs_remove_group(&bq->dev->kobj, &fg_attr_group);

	return 0;
}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_info("bq fuel gauge driver shutdown!\n");
}

static const struct of_device_id bq_fg_match_table[] = {
	{
		.compatible = "ti,bq40z50",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bq_fg_id[] = {
	{ "bq40z50", BQ40Z50 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume = bq_fg_resume,
	.suspend = bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver	= {
		.name   = "bq_fg",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
	},
	.id_table       = bq_fg_id,

	.probe          = bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ40Z50 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
