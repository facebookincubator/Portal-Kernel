#include "focaltech_core.h"

#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG		8
#define PROC_SET_SLAVE_ADDR		10
#define PROC_HW_RESET			11
#define PROC_NAME			"ftxxxx-debug"
#define PROC_WRITE_BUF_SIZE		256
#define PROC_READ_BUF_SIZE		256

enum {
	RWREG_OP_READ = 0,
	RWREG_OP_WRITE = 1,
};
static struct rwreg_operation_t {
	int type;	/*  0: read, 1: write */
	int reg;	/*  register */
	int len;	/*  read/write length */
	int val;	/*  length = 1; read: return value, write: op return */
	int res;	/*  0: success, otherwise: fail */
	char *opbuf;
} rw_op;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))

static ssize_t fts_debug_write(struct file *filp, const char __user *buff,
			size_t count, loff_t *ppos)
{
	u8 writebuf[PROC_WRITE_BUF_SIZE] = {0};
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	char tmp[25];
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_client *client = ts_data->client;

	if ((count == 0) || (count > PROC_WRITE_BUF_SIZE)) {
		FTS_ERROR("apk proc wirte count(%d) fail", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(&writebuf, buff, count)) {
		FTS_ERROR("[APK]: copy from user error!!");
		return -EFAULT;
	}

	ts_data->proc_opmode = writebuf[0];

	switch (ts_data->proc_opmode) {
	case PROC_SET_TEST_FLAG:
		FTS_INFO("[APK]: PROC_SET_TEST_FLAG = %x!!", writebuf[1]);
#if FTS_ESDCHECK_EN
		if (writebuf[1] == 0)
			fts_esdcheck_switch(ENABLE);
		else
			fts_esdcheck_switch(DISABLE);
#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0)
			FTS_ERROR("[APK]: write iic error!!");
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0)
			FTS_ERROR("[APK]: write iic error!!");
		break;
	case PROC_SET_SLAVE_ADDR:
#if (FTS_CHIP_TYPE == _FT8201)
		FTS_INFO("Original i2c addr 0x%x", client->addr << 1);
		if (writebuf[1] != client->addr) {
			client->addr = writebuf[1];
		FTS_INFO("Change i2c addr 0x%x to 0x%x", client->addr << 1,
			writebuf[1] << 1);
		}
#endif
		break;

	case PROC_HW_RESET:
		snprintf(tmp, PAGE_SIZE, "%s", writebuf + 1);
		tmp[buflen - 1] = '\0';
		if (strncmp(tmp, "focal_driver", 12) == 0) {
			FTS_INFO("APK execute HW Reset");
			fts_reset_proc(1);
		}
		break;

	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = buflen - 1;
		if (writelen > 0) {
			ret = fts_i2c_write(client, writebuf + 1, writelen);
			if (ret < 0)
				FTS_ERROR("[APK]: write iic error!!");
		}
		break;
	default:
		break;
	}

	if (ret < 0)
		return ret;
	else
		return count;
}

static ssize_t fts_debug_read(struct file *filp, char __user *buff,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 buf[PROC_READ_BUF_SIZE] = { 0 };
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_client *client = ts_data->client;

	if ((count == 0) || (count > PROC_READ_BUF_SIZE)) {
		FTS_ERROR("apk proc read count(%d) fail", (int)count);
		return -EINVAL;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif

	switch (ts_data->proc_opmode) {
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESDCHECK_EN
			fts_esdcheck_proc_busy(0);
#endif
			FTS_ERROR("[APK]: read iic error!!");
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESDCHECK_EN
			fts_esdcheck_proc_busy(0);
#endif
			FTS_ERROR("[APK]: read iic error!!");
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif

	if (copy_to_user(buff, buf, num_read_chars)) {
		FTS_ERROR("[APK]: copy to user error!!");
		return -EFAULT;
	}

	return num_read_chars;
}

static const struct file_operations fts_proc_fops = {
	.owner  = THIS_MODULE,
	.read   = fts_debug_read,
	.write  = fts_debug_write,
};
#else
static int fts_debug_write(struct file *filp, const char __user *buff,
			unsigned long len, void *data)
{
	int ret = 0;
	u8 writebuf[PROC_WRITE_BUF_SIZE] = { 0 };
	int buflen = len;
	int writelen = 0;
	char tmp[25];
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_client *client = ts_data->client;

	if ((count == 0) || (count > PROC_WRITE_BUF_SIZE)) {
		FTS_ERROR("apk proc wirte count(%d) fail", (int)count);
		return -EINVAL;
	}

	if (copy_from_user(&writebuf, buff, buflen)) {
		FTS_ERROR("[APK]: copy from user error!!");
		return -EFAULT;
	}

	ts_data->proc_opmode = writebuf[0];

	switch (ts_data->proc_opmode) {
	case PROC_SET_TEST_FLAG:
		FTS_DEBUG("[APK]: PROC_SET_TEST_FLAG = %x!!", writebuf[1]);
#if FTS_ESDCHECK_EN
		if (writebuf[1] == 0)
			fts_esdcheck_switch(ENABLE);
		else
			fts_esdcheck_switch(DISABLE);
#endif
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0)
			FTS_ERROR("[APK]: write iic error!!n");
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(client, writebuf + 1, writelen);
		if (ret < 0)
			FTS_ERROR("[APK]: write iic error!!");
		break;
	case PROC_SET_SLAVE_ADDR:
#if (FTS_CHIP_TYPE == _FT8201)
		ret = client->addr;
		FTS_DEBUG("Original i2c addr 0x%x ", ret << 1);
		if (writebuf[1] != client->addr) {
			client->addr = writebuf[1];
			FTS_DEBUG("Change i2c addr 0x%x to 0x%x", ret << 1,
				   writebuf[1] << 1);
		}
#endif
		break;

	case PROC_HW_RESET:
		snprintf(tmp, PAGE_SIZE, "%s", writebuf + 1);
		tmp[buflen - 1] = '\0';
		if (strncmp(tmp, "focal_driver", 12) == 0) {
			FTS_INFO("Begin HW Reset");
			fts_reset_proc(1);
		}
		break;

	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		if (writelen > 0) {
			ret = fts_i2c_write(client, writebuf + 1, writelen);
			if (ret < 0)
				FTS_ERROR("[APK]: write iic error!!");
		}
		break;
	default:
		break;
	}

	if (ret < 0)
		return ret;
	else
		return len;
}

static int fts_debug_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int ret = 0;
	u8 buf[PROC_READ_BUF_SIZE] = { 0 };
	int num_read_chars = 0;
	int readlen = 0;
	struct fts_ts_data *ts_data = fts_data;
	struct i2c_client *client = ts_data->client;

	if ((count == 0) || (count > PROC_READ_BUF_SIZE)) {
		FTS_ERROR("apk proc read count(%d) fail", (int)count);
		return -EINVAL;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	switch (ts_data->proc_opmode) {
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESDCHECK_EN
			fts_esdcheck_proc_busy(0);
#endif
			FTS_ERROR("[APK]: read iic error!!");
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
#if FTS_ESDCHECK_EN
			fts_esdcheck_proc_busy(0);
#endif
			FTS_ERROR("[APK]: read iic error!!");
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif

	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif

int fts_create_apk_debug_channel(struct fts_ts_data *ts_data)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	ts_data->proc = proc_create(PROC_NAME, 0644, NULL, &fts_proc_fops);
#else
	ts_data->proc = create_proc_entry(PROC_NAME, 0644, NULL);
#endif
	if (ts_data->proc == NULL) {
		FTS_ERROR("create proc entry fail");
		return -ENOMEM;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	ts_data->proc->write_proc = fts_debug_write;
	ts_data->proc->read_proc = fts_debug_read;
#endif
	return 0;
}
void fts_release_apk_debug_channel(struct fts_ts_data *ts_data)
{
	if (ts_data->proc) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	proc_remove(ts_data->proc);
#else
	remove_proc_entry(PROC_NAME, NULL);
#endif
	}
}

static ssize_t fts_hw_reset_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = fts_data->input_dev;
	ssize_t count = 0;

	mutex_lock(&input_dev->mutex);
	fts_reset_proc(1);
	count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
	mutex_unlock(&input_dev->mutex);

	return count;
}

static ssize_t fts_irq_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_INFO("[EX-FUN]enable irq");
		fts_irq_enable();
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_INFO("[EX-FUN]disable irq");
		fts_irq_disable();
	}
	mutex_unlock(&input_dev->mutex);
	return count;
}

static ssize_t fts_tpfwver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;
	struct i2c_client *client = ts_data->client;
	ssize_t num_read_chars = 0;
	u8 fwver = 0;

	mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	if (fts_i2c_read_reg(client, FTS_REG_FW_VER, &fwver) < 0) {
		num_read_chars = snprintf(buf, PAGE_SIZE,
		"I2c transfer error!\n");
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif
	if ((fwver == 0xFF) || (fwver == 0x00))
		num_read_chars = snprintf(buf, PAGE_SIZE,
					"get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE,
		"%02x\n", fwver);

	mutex_unlock(&input_dev->mutex);
	return num_read_chars;
}

static ssize_t fts_tprwreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count;
	int i;
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (rw_op.len < 0) {
		count = snprintf(buf, PAGE_SIZE, "Invalid cmd line\n");
	} else if (rw_op.len == 1) {
		if (rw_op.type == RWREG_OP_READ) {
			if (rw_op.res == 0) {
				count = snprintf(buf, PAGE_SIZE,
						"Read %02X: %02X\n",
						rw_op.reg, rw_op.val);
			} else {
				count = snprintf(buf, PAGE_SIZE,
						"Read %02X failed, ret: %d\n",
						rw_op.reg, rw_op.res);
			}
		} else {
			if (rw_op.res == 0) {
				count = snprintf(buf, PAGE_SIZE,
						"Write %02X, %02X success\n",
						rw_op.reg, rw_op.val);
			} else {
				count = snprintf(buf, PAGE_SIZE,
						"Write %02X failed, ret: %d\n",
						rw_op.reg, rw_op.res);
			}
		}
	} else {
		if (rw_op.type == RWREG_OP_READ) {
			count = snprintf(buf, PAGE_SIZE,
					"Read Reg: [%02X]-[%02X]\n",
					rw_op.reg, rw_op.reg + rw_op.len);
			count += snprintf(buf + count, PAGE_SIZE, "Result: ");
			if (rw_op.res) {
				count += snprintf(buf + count,
						PAGE_SIZE, "failed, ret: %d\n",
						rw_op.res);
			} else {
				if (rw_op.opbuf) {
					for (i = 0; i < rw_op.len; i++) {
						count += snprintf(buf + count,
							PAGE_SIZE, "%02X ",
							rw_op.opbuf[i]);
					}
		count += snprintf(buf + count, PAGE_SIZE, "\n");
				}
			}
		} else {
			count = snprintf(buf, PAGE_SIZE,
					"Write Reg: [%02X]-[%02X]\n",
					rw_op.reg, rw_op.reg + rw_op.len - 1);
			count += snprintf(buf + count, PAGE_SIZE,
					"Write Data: ");
			if (rw_op.opbuf) {
				for (i = 1; i < rw_op.len; i++) {
					count += snprintf(buf + count,
							PAGE_SIZE, "%02X ",
							rw_op.opbuf[i]);
				}
				count += snprintf(buf + count,
						PAGE_SIZE, "\n");
			}
			if (rw_op.res) {
				count += snprintf(buf + count, PAGE_SIZE,
						"Result: failed, ret: %d\n",
						rw_op.res);
			} else {
				count += snprintf(buf + count, PAGE_SIZE,
						"Result: success\n");
			}
		}
	}
	mutex_unlock(&input_dev->mutex);

	return count;
}

static int shex_to_int(const char *hex_buf, int size)
{
	int i;
	int base = 1;
	int value = 0;
	char single;

	for (i = size - 1; i >= 0; i--) {
		single = hex_buf[i];

		if ((single >= '0') && (single <= '9'))
			value += (single - '0') * base;
		else if ((single >= 'a') && (single <= 'z'))
			value += (single - 'a' + 10) * base;
		else if ((single >= 'A') && (single <= 'Z'))
			value += (single - 'A' + 10) * base;
		else
			return -EINVAL;
		base *= 16;
	}

	return value;
}


static u8 shex_to_u8(const char *hex_buf, int size)
{
	return (u8)shex_to_int(hex_buf, size);
}
/*
 * Format buf:
 * [0]: '0' write, '1' read(reserved)
 * [1-2]: addr, hex
 * [3-4]: length, hex
 * [5-6]...[n-(n+1)]: data, hex
 */
static int fts_parse_buf(const char *buf, size_t cmd_len)
{
	int length;
	int i;
	char *tmpbuf;

	rw_op.reg = shex_to_u8(buf + 1, 2);
	length = shex_to_int(buf + 3, 2);

	if (buf[0] == '1') {
		rw_op.len = length;
		rw_op.type = RWREG_OP_READ;
		FTS_DEBUG("read %02X, %d bytes", rw_op.reg, rw_op.len);
	} else {
		if (cmd_len < (length * 2 + 5)) {
			pr_err("data invalided!\n");
			return -EINVAL;
		}
		FTS_DEBUG("write %02X, %d bytes", rw_op.reg, length);

		/* first byte is the register addr */
		rw_op.type = RWREG_OP_WRITE;
		rw_op.len = length + 1;
	}

	if (rw_op.len > 0) {
		tmpbuf = kzalloc(rw_op.len, GFP_KERNEL);
		if (!tmpbuf) {
			FTS_ERROR("allocate memory failed!\n");
			return -ENOMEM;
		}

		if (rw_op.type == RWREG_OP_WRITE) {
			tmpbuf[0] = rw_op.reg & 0xFF;
			FTS_DEBUG("write buffer: ");
			for (i = 1; i < rw_op.len; i++) {
				tmpbuf[i] = shex_to_u8(buf + 5 + i * 2 - 2, 2);
				FTS_DEBUG("buf[%d]: %02X", i,
					   tmpbuf[i] & 0xFF);
			}
		}
		rw_op.opbuf = tmpbuf;
	}

	return rw_op.len;
}

static ssize_t fts_tprwreg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct input_dev *input_dev = fts_data->input_dev;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t cmd_length = 0;

	mutex_lock(&input_dev->mutex);
	cmd_length = count - 1;

	kfree(rw_op.opbuf);

	FTS_DEBUG("cmd len: %d, buf: %s", (int)cmd_length, buf);
	/* compatible old ops */
	if (cmd_length == 2) {
		rw_op.type = RWREG_OP_READ;
		rw_op.len = 1;

		rw_op.reg = shex_to_int(buf, 2);
	} else if (cmd_length == 4) {
		rw_op.type = RWREG_OP_WRITE;
		rw_op.len = 1;
		rw_op.reg = shex_to_int(buf, 2);
		rw_op.val = shex_to_int(buf + 2, 2);

	} else if (cmd_length < 5) {
		FTS_ERROR("Invalid cmd buffer");
		mutex_unlock(&input_dev->mutex);
		return -EINVAL;
	} else {
		rw_op.len = fts_parse_buf(buf, cmd_length);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	if (rw_op.len < 0) {
		FTS_ERROR("cmd buffer error!");
	} else {
		if (rw_op.type == RWREG_OP_READ) {
			if (rw_op.len == 1) {
				u8 reg, val;

				reg = rw_op.reg & 0xFF;
				rw_op.res = fts_i2c_read_reg(client, reg, &val);
				rw_op.val = val;
			} else {
				char reg;

				reg = rw_op.reg & 0xFF;
				rw_op.res = fts_i2c_read(client, &reg, 1,
							 rw_op.opbuf,
							 rw_op.len);
			}

			if (rw_op.res < 0) {
				FTS_ERROR("Could not read 0x%02x", rw_op.reg);
			} else {
				FTS_INFO("read 0x%02x, %d bytes successful",
					  rw_op.reg, rw_op.len);
				rw_op.res = 0;
			}
		} else {
			if (rw_op.len == 1) {
				u8 reg, val;

				reg = rw_op.reg & 0xFF;
				val = rw_op.val & 0xFF;
				rw_op.res = fts_i2c_write_reg(client,
							reg, val);
			} else {
				rw_op.res = fts_i2c_write(client, rw_op.opbuf,
							  rw_op.len);
			}
			if (rw_op.res < 0) {
				FTS_ERROR("Could not write 0x%02x", rw_op.reg);
			} else {
				FTS_INFO("Write 0x%02x, %d bytes successful",
					  rw_op.val, rw_op.len);
				rw_op.res = 0;
			}
		}
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif
	mutex_unlock(&input_dev->mutex);

	return count;
}

/*
 * fts_upgrade_bin interface
 */
static ssize_t fts_fwupgradebin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[FILE_NAME_LENGTH];
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;
	struct i2c_client *client = ts_data->client;

	if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
		FTS_ERROR("fw bin name's length(%d) fail", (int)count);
		return -EINVAL;
	}

	memcpy(fwname, buf, count);
	fwname[count] = '\0';

	FTS_INFO("upgrade with bin file through sysfs node");
	mutex_lock(&input_dev->mutex);
	ts_data->fw_loading = 1;
	fts_irq_disable();
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(DISABLE);
#endif

	fts_upgrade_bin(client, fwname, 0);

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(ENABLE);
#endif
	fts_irq_enable();
	ts_data->fw_loading = 0;
	mutex_unlock(&input_dev->mutex);

	return count;
}

/*
 * fts_force_upgrade interface
 */
static ssize_t fts_fwforceupg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[FILE_NAME_LENGTH];
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;
	struct i2c_client *client = ts_data->client;

	if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
		FTS_ERROR("fw bin name's length(%d) fail", (int)count);
		return -EINVAL;
	}
	memset(fwname, 0, sizeof(fwname));
	snprintf(fwname, PAGE_SIZE, "%s", buf);
	fwname[count - 1] = '\0';

	FTS_INFO("force upgrade through sysfs node");
	mutex_lock(&input_dev->mutex);
	ts_data->fw_loading = 1;
	fts_irq_disable();
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(DISABLE);
#endif

	fts_upgrade_bin(client, fwname, 1);

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(ENABLE);
#endif
	fts_irq_enable();
	ts_data->fw_loading = 0;
	mutex_unlock(&input_dev->mutex);

	return count;
}

/*
 * fts_driver_info interface
 */
static ssize_t fts_driverinfo_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_data *ts_data = fts_data;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev = ts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	count += snprintf(buf + count, PAGE_SIZE,
			 "Driver Ver:%s\n", FTS_DRIVER_VERSION);
	count += snprintf(buf + count, PAGE_SIZE,
			 "Resolution:(%d,%d)~(%d,%d)\n", pdata->x_min,
			  pdata->y_min, pdata->x_max, pdata->y_max);

	count += snprintf(buf + count, PAGE_SIZE, "Max Touchs:%d\n",
			  pdata->max_touch_number);

	count += snprintf(buf + count, PAGE_SIZE,
			 "reset gpio:%d,int gpio:%d,irq:%d\n",
			  pdata->reset_gpio, pdata->irq_gpio, ts_data->irq);

	count += snprintf(buf + count, PAGE_SIZE, "IC ID:0x%02x%02x\n",
			  ts_data->ic_info.ids.chip_idh,
			  ts_data->ic_info.ids.chip_idl);
	mutex_unlock(&input_dev->mutex);

	return count;
}

/*
 * fts_dump_reg interface
 */
static ssize_t fts_dumpreg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0;
	u8 val = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(1);
#endif
	fts_i2c_read_reg(client, FTS_REG_POWER_MODE, &val);
	count += snprintf(buf + count, PAGE_SIZE, "Power Mode:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_FW_VER, &val);
	count += snprintf(buf + count, PAGE_SIZE, "FW Ver:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_LIC_VER, &val);
	count += snprintf(buf + count, PAGE_SIZE,
			 "LCD Initcode Ver:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_IDE_PARA_VER_ID, &val);
	count += snprintf(buf + count, PAGE_SIZE, "Param Ver:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_IDE_PARA_STATUS, &val);
	count += snprintf(buf + count, PAGE_SIZE,
			 "Param status:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_VENDOR_ID, &val);
	count += snprintf(buf + count, PAGE_SIZE, "Vendor ID:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_LCD_BUSY_NUM, &val);
	count += snprintf(buf + count, PAGE_SIZE,
			 "LCD Busy Number:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &val);
	count += snprintf(buf + count, PAGE_SIZE,
			 "Gesture Mode:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_CHARGER_MODE_EN, &val);
	count += snprintf(buf + count, PAGE_SIZE, "charge stat:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_INT_CNT, &val);
	count += snprintf(buf + count, PAGE_SIZE, "INT count:0x%02x\n", val);

	fts_i2c_read_reg(client, FTS_REG_FLOW_WORK_CNT, &val);
	count += snprintf(buf + count, PAGE_SIZE, "ESD count:0x%02x\n", val);
#if FTS_ESDCHECK_EN
	fts_esdcheck_proc_busy(0);
#endif

	mutex_unlock(&input_dev->mutex);
	return count;
}

/* get the fw version  example:cat fw_version */
static DEVICE_ATTR(fts_fw_version, 0440, fts_tpfwver_show,
		   NULL);
static DEVICE_ATTR(fts_rw_reg, 0640, fts_tprwreg_show,
		   fts_tprwreg_store);
/*  upgrade from fw bin file   example:echo "*.bin" > fts_upgrade_bin */
static DEVICE_ATTR(fts_upgrade_bin, 0220, NULL,
		   fts_fwupgradebin_store);
static DEVICE_ATTR(fts_force_upgrade, 0220, NULL,
		   fts_fwforceupg_store);
static DEVICE_ATTR(fts_driver_info, 0440, fts_driverinfo_show,
		   NULL);
static DEVICE_ATTR(fts_dump_reg, 0440, fts_dumpreg_show,
		   NULL);
static DEVICE_ATTR(fts_hw_reset, 0440, fts_hw_reset_show,
		   NULL);
static DEVICE_ATTR(fts_irq, 0640, NULL, fts_irq_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_fts_fw_version.attr,
	&dev_attr_fts_rw_reg.attr,
	&dev_attr_fts_dump_reg.attr,
	&dev_attr_fts_upgrade_bin.attr,
	&dev_attr_fts_force_upgrade.attr,
	&dev_attr_fts_driver_info.attr,
	&dev_attr_fts_hw_reset.attr,
	&dev_attr_fts_irq.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

static DEVICE_ATTR(firmware_version, 0444, fts_tpfwver_show, NULL);

static struct attribute *fts_version_attribute[] = {
	&dev_attr_firmware_version.attr,
	NULL
};

static struct attribute_group fts_version_attribute_group = {
	.attrs = fts_version_attribute
};

struct kobject *fw_ver_kobj;

int fts_create_sysfs(struct i2c_client *client)
{
	int ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (ret) {
		FTS_ERROR("[EX]: fts_attribute_group: sysfs_create_group() failed!!");
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -ENOMEM;
	} else {
		FTS_INFO("[EX]: fts_attribute_group: sysfs_create_group() succeeded!!");
	}

	fw_ver_kobj = kobject_create_and_add("touchscreen", NULL);
	if (fw_ver_kobj == NULL) {
		FTS_ERROR("[EX]: kobject_create_and_add() failed!!");
		return -ENOMEM;
	}

	ret = sysfs_create_group(fw_ver_kobj, &fts_version_attribute_group);
	if (ret) {
		FTS_ERROR("[EX]: fts_version_attribute_group: sysfs_create_group() failed");
		kobject_put(fw_ver_kobj);
	}
	return ret;
}

int fts_remove_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	if (fw_ver_kobj != NULL) {
		sysfs_remove_group(fw_ver_kobj, &fts_version_attribute_group);
		kobject_del(fw_ver_kobj);
		kobject_put(fw_ver_kobj);
	}
	return 0;
}
