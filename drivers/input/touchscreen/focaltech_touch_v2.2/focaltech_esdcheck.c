#include "focaltech_core.h"

#if FTS_ESDCHECK_EN
#define ESDCHECK_WAIT_TIME	1000
#define LCD_ESD_PATCH		0

struct fts_esdcheck_st {
	u8 mode			: 1;
	u8 suspend		: 1;
	u8 proc_debug		: 1;
	u8 intr			: 1;
	u8 unused		: 4;
	u8 flow_work_hold_cnt;
	u8 flow_work_cnt_last;
	u32 hardware_reset_cnt;
	u32 i2c_nack_cnt;
	u32 i2c_dataerror_cnt;
};

static struct fts_esdcheck_st fts_esdcheck_data;

#if LCD_ESD_PATCH

int lcd_need_reset;
static int tp_need_recovery; /* LCD reset cause Tp reset */
int idc_esdcheck_lcderror(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 val = 0;
	struct i2c_client *client = ts_data->client;

	FTS_DEBUG("[ESD]Check LCD ESD");
	if ((tp_need_recovery == 1) && (lcd_need_reset == 0)) {
		tp_need_recovery = 0;
		/* LCD reset, need recover TP state */
		fts_tp_state_recovery(client);
	}

	ret = fts_i2c_read_reg(client, FTS_REG_ESD_SATURATE, &val);
	if (ret < 0) {
		FTS_ERROR("[ESD]: Read ESD_SATURATE(0xED) failed ret=%d!", ret);
		return -EIO;
	}

	if (val == 0xAA) {
		FTS_INFO("LCD ESD, Execute LCD reset!");
		lcd_need_reset = 1;
		tp_need_recovery = 1;
	}

	return 0;
}
#endif

static int fts_esdcheck_tp_reset(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	fts_esdcheck_data.flow_work_hold_cnt = 0;
	fts_esdcheck_data.hardware_reset_cnt++;

	fts_reset_proc(200);
	fts_tp_state_recovery(ts_data->client);

	FTS_FUNC_EXIT();
	return 0;
}

static bool get_chip_id(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int i = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	struct i2c_client *client = ts_data->client;
	u8 chip_id = ts_data->ic_info.ids.chip_idh;

	for (i = 0; i < 3; i++) {
		reg_addr = FTS_REG_CHIP_ID;
		ret = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
		if (ret < 0) {
			FTS_ERROR("[ESD]: Read Reg 0xA3 failed ret = %d!!", ret);
			fts_esdcheck_data.i2c_nack_cnt++;
		} else {
			if (reg_value == chip_id)
				break;
			else if (reg_value != chip_id)
				fts_esdcheck_data.i2c_dataerror_cnt++;
		}
		msleep(20);
	}

	/* if can't get correct data in 3 times, then need hardware reset */
	if (i >= 3) {
		FTS_ERROR("[ESD]: Read Chip id 3 times failed, need execute TP reset!!");
		return 1;
	}

	return 0;
}

static bool get_flow_cnt(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	struct i2c_client *client = ts_data->client;

	reg_addr = FTS_REG_FLOW_WORK_CNT;
	ret = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (ret < 0) {
		FTS_ERROR("[ESD]: Read Reg 0x91 failed ret = %d!!", ret);
		fts_esdcheck_data.i2c_nack_cnt++;
	} else {
		if (reg_value == fts_esdcheck_data.flow_work_cnt_last)
			fts_esdcheck_data.flow_work_hold_cnt++;
		else
			fts_esdcheck_data.flow_work_hold_cnt = 0;
		fts_esdcheck_data.flow_work_cnt_last = reg_value;
	}


	if (fts_esdcheck_data.flow_work_hold_cnt >= 5) {
		FTS_DEBUG("[ESD]: Flow Work Cnt(reg0x91) keep a value for 5 times, need execute TP reset!!");
		return 1;
	}

	return 0;
}

static int esdcheck_algorithm(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	bool hardware_reset = 0;
	struct i2c_client *client = ts_data->client;

	/* 1. esdcheck is interrupt, then return */
	if (fts_esdcheck_data.intr == 1) {
		FTS_DEBUG("[ESD]: In interrupt state, not check esd, return immediately!!");
		return 0;
	}

	/* 2. check power state, if suspend, no need check esd */
	if (fts_esdcheck_data.suspend == 1) {
		FTS_DEBUG("[ESD]: In suspend, not check esd, return immediately!!");
		return 0;
	}


	if (fts_esdcheck_data.proc_debug == 1) {
		FTS_INFO("[ESD]: In apk or adb command mode, not check esd, return immediately!!");
		return 0;
	}

	/* 4. In factory mode, can't check esd */
	reg_addr = FTS_REG_WORKMODE;
	ret = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (ret < 0) {
		fts_esdcheck_data.i2c_nack_cnt++;
	} else if ((reg_value & 0x70) !=  FTS_REG_WORKMODE_WORK_VALUE) {
		FTS_DEBUG("[ESD]: not in work mode, no check esd, return immediately!!");
		return 0;
	}

	/* 5. IDC esd check lcd  default:close */
#if LCD_ESD_PATCH
	idc_esdcheck_lcderror(ts_data);
#endif

	/* 6. Get Chip ID */
	hardware_reset = get_chip_id(ts_data);

	if (!hardware_reset)
		hardware_reset = get_flow_cnt(ts_data);

	/* 8. If need hardware reset, then handle it here */
	if (hardware_reset == 1)
		fts_esdcheck_tp_reset(ts_data);

	FTS_DEBUG("[ESD]: NoACK=%d, Error Data=%d, Hardware Reset=%d",
		   fts_esdcheck_data.i2c_nack_cnt,
		   fts_esdcheck_data.i2c_dataerror_cnt,
		   fts_esdcheck_data.hardware_reset_cnt);
	return 0;
}
static void esdcheck_func(struct work_struct *work)
{
	u8 val = 0;
	struct fts_ts_data *ts_data = container_of(work,
					struct fts_ts_data,
					esdcheck_work.work);

	FTS_FUNC_ENTER();
	if (fts_esdcheck_data.mode == ENABLE) {
		if (ts_data->ic_info.is_incell) {
			fts_i2c_read_reg(ts_data->client,
				FTS_REG_ESDCHECK_DISABLE, &val);
			if (val == 0XA5) {
				fts_esdcheck_data.mode = DISABLE;
				return;
			}
		}
		esdcheck_algorithm(ts_data);
		queue_delayed_work(ts_data->ts_workqueue,
			&ts_data->esdcheck_work,
			msecs_to_jiffies(ESDCHECK_WAIT_TIME));
	}
	FTS_FUNC_EXIT();
}
int fts_esdcheck_set_intr(bool intr)
{
	/* interrupt don't add debug message */
	fts_esdcheck_data.intr = intr;
	return 0;
}

int fts_esdcheck_get_status(void)
{
	/* interrupt don't add debug message */
	return fts_esdcheck_data.mode;
}

int fts_esdcheck_proc_busy(bool proc_debug)
{
	fts_esdcheck_data.proc_debug = proc_debug;
	return 0;
}

int fts_esdcheck_switch(bool enable)
{
	struct fts_ts_data *ts_data = fts_data;

	if (fts_esdcheck_data.mode == ENABLE) {
		if (enable) {
			FTS_DEBUG("[ESD]: ESD check start!!");
			fts_esdcheck_data.flow_work_hold_cnt = 0;
			fts_esdcheck_data.flow_work_cnt_last = 0;
			queue_delayed_work(ts_data->ts_workqueue,
				&ts_data->esdcheck_work,
				msecs_to_jiffies(ESDCHECK_WAIT_TIME));
		} else {
			FTS_DEBUG("[ESD]: ESD check stop!!");
			cancel_delayed_work(&ts_data->esdcheck_work);
		}
	} else {
		FTS_DEBUG("[ESD]: ESD should disable!!");
		cancel_delayed_work(&ts_data->esdcheck_work);
	}

	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_suspend(void)
{
	FTS_FUNC_ENTER();
	fts_esdcheck_switch(DISABLE);
	fts_esdcheck_data.suspend = 1;
	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_resume(void)
{
	FTS_FUNC_ENTER();
	fts_esdcheck_switch(ENABLE);
	fts_esdcheck_data.suspend = 0;
	FTS_FUNC_EXIT();
	return 0;
}

static ssize_t fts_esdcheck_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_DEBUG("enable esdcheck");
		fts_esdcheck_data.mode = ENABLE;
		fts_esdcheck_switch(ENABLE);
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_DEBUG("disable esdcheck");
		fts_esdcheck_data.mode = DISABLE;
		fts_esdcheck_switch(DISABLE);
	}
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t fts_esdcheck_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int count;
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Esd check: %s\n",
				fts_esdcheck_get_status() ? "On" : "Off");
	mutex_unlock(&input_dev->mutex);

	return count;
}

/* sysfs esd node
 * read example:  cat  fts_esd_mode        ---read esd mode
 * write example: echo 01 > fts_esd_mode   ---make esdcheck enable
 *
 */
static DEVICE_ATTR(fts_esd_mode, 0640,
		fts_esdcheck_show, fts_esdcheck_store);

static struct attribute *fts_esd_mode_attrs[] = {
	&dev_attr_fts_esd_mode.attr,
	NULL,
};

static struct attribute_group fts_esd_group = {
	.attrs = fts_esd_mode_attrs,
};

int fts_create_esd_sysfs(struct i2c_client *client)
{
	int ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, &fts_esd_group);
	if (ret != 0) {
		FTS_ERROR("fts_create_esd_sysfs(sysfs) create failed!");
		sysfs_remove_group(&client->dev.kobj, &fts_esd_group);
		return ret;
	}
	return 0;
}

int fts_esdcheck_init(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	if (ts_data->ts_workqueue) {
		INIT_DELAYED_WORK(&ts_data->esdcheck_work, esdcheck_func);
	} else {
		FTS_ERROR("fts workqueue is NULL, can't run esd check function");
		return -EINVAL;
	}

	memset((u8 *)&fts_esdcheck_data, 0, sizeof(struct fts_esdcheck_st));

	fts_esdcheck_data.mode = ENABLE;
	fts_esdcheck_switch(ENABLE);
	fts_create_esd_sysfs(ts_data->client);
	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	FTS_FUNC_EXIT();
	return 0;
}
#endif /* FTS_ESDCHECK_EN */
