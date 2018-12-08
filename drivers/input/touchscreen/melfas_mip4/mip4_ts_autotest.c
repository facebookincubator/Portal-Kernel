/*
 * mip4_ts_autotest.c
 *
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include "mip4_ts.h"
#include <stdarg.h>

#define MIP4_TEST_CASE_NUM (sizeof(mip4_ts_autotest_cases)/sizeof(int))
#define MIP4_TIMEOUT (10000)	/* 10 secs */
#define MIP4_DELTA_DIFF 4
#define mip4_is_edge(_x_, _y_, _r_, _c_) ((_y_) == 0 || (_x_) == 0 || \
			 (_y_) == ((_r_)-1) || (_x_) == ((_c_)-1))
#define mip4_get(_x_, _y_, _c_) uptr[(_y_)*(_c_) + (_x_)]

/* Judge Ranges */
#define MIP4_ABS_MIN 1000
#define MIP4_ABS_MAX 9300
#define MIP4_DELTA_MIN -1500
#define MIP4_DELTA_MAX 1500
#define MIP4_DELTA_EDGE_MIN -1900
#define MIP4_DELTA_EDGE_MAX 1900
#define MIP4_JITTER_MAX 1024
#define MIP4_ABS_DIFF_MIN 9000
#define MIP4_ABS_DIFF_MAX 4000

/* #define DUMP_RAW_DATA */

/*
 * _in_: input ptr (unsigned char *)
 * _sz_: number of bytes per data (1, 2, or 4)
 * _sign_: sign of input. 1: signed data, 0: unsigned data
 * _out_: output pointer 4 bytes.
 */
#define get_data(_in_, _sz_, _sign_, _out_)				\
	switch ((_sz_)) {						\
	case 1:								\
		if ((_sign_))						\
			(*(signed int *)((_out_))) =			\
					(*(signed char *)((_in_)));	\
		else							\
			(*(unsigned int *)((_out_))) =			\
					(*(unsigned char *)((_in_)));	\
		break;							\
	case 2: {							\
			unsigned short tmp =				\
				(*((_in_)) | ((*((_in_)+1)) << 8));	\
			if ((_sign_))					\
				(*(signed int *)((_out_))) =		\
						(signed short)tmp;	\
			else						\
				(*(unsigned int *)((_out_))) = tmp;	\
		}							\
		break;							\
	case 4: {							\
			unsigned int tmp =				\
				(*((_in_)) | ((*((_in_)+1)) << 8) |	\
				((*((_in_)+2)) << 16) |			\
				((*((_in_)+3)) << 24));			\
			if ((_sign_))					\
				(*(signed int *)((_out_))) =		\
						(signed int)(tmp);	\
			else						\
				(*(unsigned int *)((_out_))) = tmp;	\
		}							\
		break;							\
	}

enum {
	TEST_CM_RAWDATA = 0x02,
	TEST_CM_JITTER = 0x03,
	TEST_CM_ABS_DIFFER = 0x04
};

static int mip4_ts_autotest_cases[] = {
	TEST_CM_RAWDATA /* 2 */,
	TEST_CM_JITTER /* 3 */,
	TEST_CM_ABS_DIFFER /* 4 */
};

enum {
	MIP4_FORMAT_ROW_NUM,
	MIP4_FORMAT_COL_NUM,
	MIP4_FORMAT_BUFFER_COL_NUM,
	MIP4_FORMAT_ROTATE,
	MIP4_FORMAT_KEY_NUM,
	MIP4_FORMAT_DATA_TYPE,
	MIP4_FORMAT_LENGTH
};

/*
 * Fill print_buf with the test information
 */
static int mip4_printk(struct mip4_ts_info *info, const char *s, ...)
{
	int index;
	int cnt;

	index = strnlen(info->print_buf, PAGE_SIZE);
	if (index < PAGE_SIZE) {
		va_list ap;

		va_start(ap, s);
		cnt = vscnprintf(&info->print_buf[index],
						PAGE_SIZE - index, s, ap);
		va_end(ap);
	}

	return cnt;
}

/*
 * Wait until device is ready
 * - timeout in ms
 */
static int mip4_ts_wait_for_ready(struct mip4_ts_info *info, int timeout)
{
	int rc;
	u8 wbuf[2] = {MIP4_R0_CTRL, MIP4_R1_CTRL_READY_STATUS};
	u8 status = 0;
	int cnt = timeout / 5; /* Convert ms to 5ms count */

	if (!cnt)
		cnt = 1;

	while (cnt--) {
		rc = mip4_ts_i2c_read(info, wbuf, 2, &status, 1);
		if (!rc && status == MIP4_CTRL_STATUS_READY)
			break;

		usleep_range(5*1000, 10*1000);
	}

	if (status != MIP4_CTRL_STATUS_READY) {
		mip4_printk(info,
			"%s: [ERROR] ready wait timeout\n", __func__);
		return 1;
	}

	return 0;
}

/*
 * Set mode (test or normal)
 */
static int mip4_ts_set_mode(struct mip4_ts_info *info, int mode)
{
	int rc;
	u8 wbuf[3] = {MIP4_R0_CTRL, MIP4_R1_CTRL_MODE};

	wbuf[2] = mode;

	rc = mip4_ts_i2c_write(info, wbuf, 3);
	if (rc) {
		mip4_printk(info,
			"%s [ERROR] write mode %d\n", __func__, mode);
		return rc;
	}

	return mip4_ts_wait_for_ready(info, MIP4_TIMEOUT);
}

/*
 * Set test type to run
 */
static int mip4_ts_set_test_type(struct mip4_ts_info *info, int type)
{
	int rc;
	u8 wbuf[3] = {MIP4_R0_TEST, MIP4_R1_TEST_TYPE};

	wbuf[2] = type;

	rc = mip4_ts_i2c_write(info, wbuf, 3);
	if (rc) {
		mip4_printk(info,
			"%s [ERROR] write type %d (%d)\n", __func__, type, rc);
		return rc;
	}

	return mip4_ts_wait_for_ready(info, MIP4_TIMEOUT);
}

/*
 * Read data format for specific test type
 */
static int mip4_ts_get_data_format(struct mip4_ts_info *info, u8 *df)
{
	u8 wbuf[2] = {MIP4_R0_TEST, MIP4_R1_TEST_DATA_FORMAT};
	int rc;

	rc = mip4_ts_i2c_read(info, wbuf, 2, df, MIP4_FORMAT_LENGTH);
	if (rc)
		mip4_printk(info,
			"%s [ERROR] read failed %d\n", __func__, rc);

	return rc;
}

/*
 * get buffer address for specific test type
 */
static int mip4_ts_get_buffer_addrss(struct mip4_ts_info *info, int *ba)
{
	u8 wbuf[2] = {MIP4_R0_TEST, MIP4_R1_TEST_BUF_ADDR};
	u8 rbuf[2];
	int rc;

	rc = mip4_ts_i2c_read(info, wbuf, 2, rbuf, 2);
	if (!rc)
		*ba = rbuf[0] | (rbuf[1] << 8);
	else
		mip4_printk(info,
			"%s [ERROR] read failed %d\n", __func__, rc);

	return rc;
}

#ifdef DUMP_RAW_DATA
static void dump_raw_data(struct mip4_ts_info *info, int sign,
			u8 sz, int row, int col, int rot, int type)
{
	int x;
	int y;
	int index;

	pr_cont("DUMP RAW DATA (Test Type = %d)\n", type);

	for (y = 0; y < row; y++) {
		pr_cont("%02d: ", y);

		for (x = 0; x < col; x++) {
			if (rot)
				index = x*row + (row - y - 1);
			else
				index = y*col + x;

			if (sign)
				pr_cont("%d ",
					info->image_buf[index]);
			else
				pr_cont("%u ",
					(unsigned int)info->image_buf[index]);
		}
		pr_cont("\n");
	}
}
#endif

/*
 * Get the raw resule for the test type
 */
static int mip4_ts_get_result(struct mip4_ts_info *info,
				u8 *df, int buffer_addr, int type)
{
	u8 sz;
	u8 sign;
	int row;
	int col;
	int x;
	int y;
	int buf_col_addr;
	int buf_col_size;
	int col_size;
	u8 wbuf[2];
	u8 *rbuf;
	int rc;
	int index;

	sign = (df[MIP4_FORMAT_DATA_TYPE] & 0x80) >> 7;
	sz = df[MIP4_FORMAT_DATA_TYPE] & 0x7f;
	row = df[MIP4_FORMAT_ROW_NUM];
	col = df[MIP4_FORMAT_COL_NUM];
	buf_col_size = df[MIP4_FORMAT_BUFFER_COL_NUM] * sz;
	col_size = col * sz;

	rbuf = kmalloc(col_size, GFP_KERNEL);
	if (!rbuf) {
		mip4_printk(info,
			"%s: [ERROR] memory alloc failed\n", __func__);
		return 1;
	}

	for (y = 0; y < row; y++) {
		buf_col_addr = buffer_addr | (buf_col_size * y);

		wbuf[0] = (buf_col_addr >> 8) & 0xff;
		wbuf[1] = buf_col_addr & 0xff;

		rc = mip4_ts_i2c_read(info, wbuf, 2, rbuf, col_size);
		if (rc) {
			mip4_printk(info,
				"%s: read failed\n", __func__);
			break;
		}

		for (x = 0; x < col; x++) {
			if (df[MIP4_FORMAT_ROTATE])
				index = x*row + (row - y - 1);
			else
				index = y*col + x;

			get_data(&rbuf[x * sz], sz,
					sign, &info->image_buf[index]);
		}
	}

#ifdef DUMP_RAW_DATA
	dump_raw_data(info, sign, sz, row, col, df[MIP4_FORMAT_ROTATE], type);
#endif
	kfree(rbuf);

	return rc;
}

/*
 * Judge the raw data with given ranges
 */
static int mip4_ts_judge_range(struct mip4_ts_info *info, const char *test,
		int sign, int row, int col, int min, int max)
{
	int *ptr = (int *)info->image_buf;
	unsigned int *uptr = (unsigned int *)info->image_buf;
	int x, y;
	int index;
	int rc = 0;
	unsigned long err_count = 0;
	unsigned long max_err = 10;

	mip4_printk(info, "    %s: range %d(0x%x) ~ %d(0x%x)\n",
			test, min, min, max, max);

	for (y = 0; y < row; y++) {
		index = y * col;
		for (x = 0; x < col; x++, index++) {
			if ((sign && (ptr[index] < min || ptr[index] > max)) ||
				(!sign && (uptr[index] < min ||
							uptr[index] > max))) {
				if (++err_count < max_err)
					mip4_printk(info,
						"    [ERROR] out of range (y=%d,x=%d) 0x%x\n",
						y, x, uptr[index]);
				rc = 1;
			}
		}
	}

	if (rc == 0)
		mip4_printk(info, "    %s: SUCCESS\n", test);
	else
		mip4_printk(info,
			"    %s: FAIL (total error = %lu)\n",
			test, err_count);

	return rc;
}

/*
 * Judge the raw data with given min / max
 */
static int mip4_ts_judge_differ(struct mip4_ts_info *info, const char *test,
		int sign, int row, int col, long min, long max)
{
	int *ptr = (int *)info->image_buf;
	unsigned int *uptr = (unsigned int *)info->image_buf;
	int x, y;
	int index;
	int rc = 0;
	long min_data = LONG_MAX;
	long max_data = LONG_MIN;

	mip4_printk(info, "    %s: Differ Min: %d(0x%x) Differ Max: %d(0x%x)\n",
			test, min, min, max, max);

	for (y = 0; y < row; y++) {
		index = y * col;
		for (x = 0; x < col; x++, index++) {
			if (sign) {
				if (ptr[index] < min_data)
					min_data = ptr[index];
				if (ptr[index] > max_data)
					max_data = ptr[index];
			} else {
				if (uptr[index] < min_data)
					min_data = uptr[index];
				if (uptr[index] > max_data)
					max_data = uptr[index];
			}
		}
	}

	mip4_printk(info, "    min_data = %ld, max_data = %ld\n",
				min_data, max_data);

	if ((max_data - min_data) > max) {
		mip4_printk(info,
			"    [ERROR] Max Diff %ld should be less than %d\n",
					max_data - min_data, max);
		rc = 1;
	}
	if (min_data < min) {
		mip4_printk(info,
			"    [ERROR] Min %ld should be bigger than %d\n",
					min_data, min);
		rc = 1;
	}
	if (rc == 0)
		mip4_printk(info, "    %s: SUCCESS\n", test);
	else
		mip4_printk(info, "    %s: FAIL\n", test);

	return rc;
}

/*
 * Judge deta between raw data
 * Use different range for edge and normal area
 */
static int mip4_ts_judge_delta_abs(struct mip4_ts_info *info, int sign,
		int row, int col, int min, int max, int min_edge, int max_edge)
{
	int	x;
	int	y;
	int	diff;
	unsigned int *uptr = (unsigned int *)info->image_buf;
	int	rc = 0;
	unsigned long err_count = 0;
	unsigned long max_err = 10;

	mip4_printk(info, "\n    Delta ABS: range %d(0x%x) ~ %d(0x%x), Edge %d(0x%x) ~ %d(0x%x)\n",
			min, min, max, max,
			min_edge, min_edge, max_edge, max_edge);
#ifdef DUMP_RAW_DATA
	pr_cont("Raw Delta ABS\n");
#endif
	for (y = 0; y < row - MIP4_DELTA_DIFF ; y++) {
#ifdef DUMP_RAW_DATA
	pr_cont("%02d: ", y);
#endif
		for (x = 0; x < col; x++) {

			diff = mip4_get(x, y, col) - mip4_get(x, y+3, col);

			if (y == 0) {
				int cend = col-1;
				int rend = row-1;

				if (x == 0)
					diff += mip4_get(cend, rend-3, col) -
						mip4_get(cend, rend, col);
				else
					diff += mip4_get(x-1, rend-3, col) -
						mip4_get(x-1, rend, col);
			}

#ifdef DUMP_RAW_DATA
			pr_cont("%d ", diff);
#endif

			if (mip4_is_edge(x, y, row, col)) {
				if (diff < min_edge || diff > max_edge) {
					if (++err_count < max_err)
						mip4_printk(info,
							"[ERROR]Judge delta, edge fail diff = %x, y = %d, x = %d, (%x, %x)\n",
							diff, y, x, min_edge, max_edge);
					rc = 1;
				}
			} else {
				if (diff < min || diff > max) {
					if (++err_count < max_err)
						mip4_printk(info,
							"[ERROR] Judge delta, fail diff = %x, y = %d, x = %d, (%x, %x)\n",
							diff, y, x, min, max);
					rc = 1;
				}
			}
		}
#ifdef DUMP_RAW_DATA
		pr_cont("\n");
#endif
	}

	if (rc == 0)
		mip4_printk(info, "    Delta ABS: SUCCESS\n");
	else
		mip4_printk(info,
			"    Delta ABS: FAIL (toal error = %lu)\n",
			err_count);
	return rc;
}

/*
 * Judge the test result based on the test type
 */
static int mip4_ts_judge_result(struct mip4_ts_info *info, u8 *df, int type)
{
	int sign;
	int row;
	int col;
	int rc = 0;

	sign = (df[MIP4_FORMAT_DATA_TYPE] & 0x80) >> 7;
	row = df[MIP4_FORMAT_ROW_NUM];
	col = df[MIP4_FORMAT_COL_NUM];

	switch (type) {
	case TEST_CM_RAWDATA:
		if (mip4_ts_judge_range(info, "ABS", sign, row, col,
				MIP4_ABS_MIN, MIP4_ABS_MAX))
			rc = 1;

		if (mip4_ts_judge_delta_abs(info, sign, row, col,
				MIP4_DELTA_MIN, MIP4_DELTA_MAX,
				MIP4_DELTA_EDGE_MIN, MIP4_DELTA_EDGE_MAX))
			rc = 1;
		break;
	case TEST_CM_JITTER:
		/* Min is don't care. Only Max is interesting */
		if (mip4_ts_judge_range(info, "Jitter", sign, row, col,
				sign ? INT_MIN : 0, MIP4_JITTER_MAX))
			rc = 1;
		break;
	case TEST_CM_ABS_DIFFER:
		if (mip4_ts_judge_differ(info, "ABS Differ", sign,
			row, col, MIP4_ABS_DIFF_MIN, MIP4_ABS_DIFF_MAX))
			rc = 1;
		break;
	}

	return rc;
}

/*
 * Run alto test with predefined test cases
 */
ssize_t mip4_ts_sys_test_auto(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mip4_ts_info *info = dev_get_drvdata(dev);
	u8 df[MIP4_FORMAT_LENGTH];
	u8 wbuf[3];
	int ba;
	int i;
	int rc = 0;
	int result = 0;

	info->print_buf[0] = 0;

	disable_irq(info->irq);

	/* disable touch event */
	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP4_CTRL_TRIGGER_NONE;
	if ((rc = mip4_ts_i2c_write(info, wbuf, 3))) {
		mip4_printk(info,
			"%s [ERROR] Disable event\n", __func__);
		goto out;
	}

	if ((rc = mip4_ts_set_mode(info, MIP4_CTRL_MODE_TEST)))
		goto out;

	mip4_printk(info, "<Touch AUTO TEST>\n");

	for (i = 0; i < MIP4_TEST_CASE_NUM; i++) {
		mip4_printk(info, "\nTest Type %d Started\n",
			mip4_ts_autotest_cases[i]);
		rc = mip4_ts_set_test_type(info, mip4_ts_autotest_cases[i]);
		if (rc) {
			mip4_printk(info,
				"%s: set_test_type failed\n", __func__);
			break;
		}

		rc = mip4_ts_get_data_format(info, df);
		if (rc) {
			mip4_printk(info,
				"%s: get_data_format failed\n", __func__);
			break;
		}

		rc = mip4_ts_get_buffer_addrss(info, &ba);
		if (rc) {
			mip4_printk(info,
				"%s: get_buffer_address failed\n", __func__);
			break;
		}

		rc = mip4_ts_get_result(info, df, ba,
					mip4_ts_autotest_cases[i]);
		if (rc) {
			mip4_printk(info,
				"%s: get_test_result failed\n", __func__);
			break;
		}

		rc = mip4_ts_judge_result(info, df, mip4_ts_autotest_cases[i]);
		if (rc)
			result = 1;
	}

out:
	mip4_ts_set_mode(info, MIP4_CTRL_MODE_NORMAL);

	/* enable touch event */
	wbuf[0] = MIP4_R0_CTRL;
	wbuf[1] = MIP4_R1_CTRL_EVENT_TRIGGER_TYPE;
	wbuf[2] = MIP4_CTRL_TRIGGER_INTR;
	if ((rc = mip4_ts_i2c_write(info, wbuf, 3)))
		mip4_printk(info, "%s [ERROR] Enable event\n",
							__func__);

	enable_irq(info->irq);

	if (!rc && !result)
		mip4_printk(info, "Final Result: PASS\n");
	else
		mip4_printk(info, "Final Result: FAIL\n");

	return snprintf(buf, PAGE_SIZE, info->print_buf);
}

