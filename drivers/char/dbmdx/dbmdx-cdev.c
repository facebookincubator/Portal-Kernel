/*
 * DSPG DBMDX codec driver character device interface
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

#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/signal.h>
#else
#include <linux/sched/signal.h>
#endif

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmdx-common-operations.h>
#include <linux/mfd/dbmdx/dbmdx-cdev.h>


#define MAX_KFIFO_BUFFER_SIZE_MONO		(32768 * 8) /* >8 seconds */
#define MAX_KFIFO_BUFFER_SIZE_STEREO		(MAX_KFIFO_BUFFER_SIZE_MONO * 2)
#define MAX_KFIFO_BUFFER_SIZE_4CH		(MAX_KFIFO_BUFFER_SIZE_MONO * 4)
#define MAX_SUPPORTED_CHANNELS			4
#define MAX_KFIFO_BUFFER_SIZE			MAX_KFIFO_BUFFER_SIZE_MONO

#define VOICE_BUF_IN_SEC 5
#define DEFAULT_CORE_LOG_WATERMARK	1000
#define DEFAULT_FIFO_EN_MASK		0


static struct dbmdx_private *dbmdx_p;
static DEFINE_MUTEX(mutex_critical_sec);
static atomic_t cdev_opened[DBMDX_MAX_NUM_KFIFO] = {
	ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0) };

static void get_fifo_type_from_chdev_minor(u16 minor,
					enum dbmdx_fifo_type *fifo_type)
{
	switch (minor) {
	case 1:
		*fifo_type = DBMDX_FIFO_TYPE_LP_LOG;
		break;
	case 2:
		*fifo_type = DBMDX_FIFO_TYPE_HF0_LOG;
		break;
	case 3:
		*fifo_type = DBMDX_FIFO_TYPE_HF1_LOG;
		break;
	default:
		*fifo_type = DBMDX_FIFO_TYPE_AUDIO;
		break;
	}
}

static void get_fifo_type_from_mb_id(u16 mailbox_id,
					enum dbmdx_fifo_type *fifo_type)
{
	switch (mailbox_id) {
	case LOGS_CORE_LP_MB_ID:
		*fifo_type = DBMDX_FIFO_TYPE_LP_LOG;
		break;
	case LOGS_CORE_HF0_MB_ID:
		*fifo_type = DBMDX_FIFO_TYPE_HF0_LOG;
		break;
	case LOGS_CORE_HF1_MB_ID:
		*fifo_type = DBMDX_FIFO_TYPE_HF1_LOG;
		break;
	default:
		*fifo_type = DBMDX_FIFO_TYPE_AUDIO;
		break;
	}
}

static void get_mb_buf_work_from_fifo_type(struct dbmdx_private *p,
					enum dbmdx_fifo_type fifo_type,
					struct mb_buf_work_data **mb_buf_work)
{
	struct dbmdx_cdev_private *cdev_priv = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	switch (fifo_type) {
	case DBMDX_FIFO_TYPE_LP_LOG:
		*mb_buf_work = &(cdev_priv->mb_buffering_work[1]);
		break;
	case DBMDX_FIFO_TYPE_HF0_LOG:
		*mb_buf_work = &(cdev_priv->mb_buffering_work[2]);
		break;
	case DBMDX_FIFO_TYPE_HF1_LOG:
		*mb_buf_work = &(cdev_priv->mb_buffering_work[3]);
		break;
	default:
		*mb_buf_work = &(cdev_priv->mb_buffering_work[0]);
		break;
	}
}

static void get_kfifo_from_fifo_type(struct dbmdx_private *p,
					enum dbmdx_fifo_type fifo_type,
					struct kfifo **fifo)
{
	struct dbmdx_cdev_private *cdev_priv = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	switch (fifo_type) {
	case DBMDX_FIFO_TYPE_LP_LOG:
		*fifo = &(cdev_priv->fifo_db[1].samples_kfifo);
		break;
	case DBMDX_FIFO_TYPE_HF0_LOG:
		*fifo = &(cdev_priv->fifo_db[2].samples_kfifo);
		break;
	case DBMDX_FIFO_TYPE_HF1_LOG:
		*fifo = &(cdev_priv->fifo_db[3].samples_kfifo);
		break;
	default:
		*fifo = &(cdev_priv->fifo_db[0].samples_kfifo);
		break;
	}
}

static int dbmdx_mailbox_buffering(struct mb_buf_work_data *mb_buf_data)
{
	u32	ack_val;
	u8	bytes_per_sample = 1; /*ATTRIB_SMP_WIDTH_8_BITS*/
	int	ret = -1;
	u32	samples_count;
	u8	*buf;
	u32	chunk_samples;
	u32	kfifo_space;
	struct dbmdx_private *p = mb_buf_data->p;
	struct dbmdx_cdev_private *cdev_priv = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	dev_info(p->dev, "%s Start buffering from MB#<0x%x>\n",
					__func__, mb_buf_data->mailbox_id);

	buf = vmalloc(MB_BUFFER_CHUNK * mb_buf_data->num_channels);
	if (!buf)
		return -ENOMEM;

	switch (mb_buf_data->sample_width) {
	case ATTRIB_SMP_WIDTH_16_BITS:
		bytes_per_sample = 2;
		break;
	case ATTRIB_SMP_WIDTH_24_BITS:
		bytes_per_sample = 3;
		break;
	case ATTRIB_SMP_WIDTH_32_BITS:
		bytes_per_sample = 4;
		break;
	}

	chunk_samples = MB_BUFFER_CHUNK / bytes_per_sample;
	samples_count = VOICE_BUF_IN_SEC * mb_buf_data->num_channels *
			(mb_buf_data->sample_rate / chunk_samples);


	/* Set to the correct mailbox transfer size*/
	ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_MAILBOX, REG_MB__TEMP_BUFF_LEN,
				MB_BUFFER_CHUNK * mb_buf_data->num_channels,
				&ack_val);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Error setting register REG_MB__TEMP_BUFF_LEN\n",
			__func__);
		goto out_free_mem;
	}

	while ((mb_buf_data->buffering_mode == BUFFERING_UNTIL_MB_EMPTY) ||
		(mb_buf_data->buffering_mode ==
				BUFFERING_COUNTING && samples_count--)) {

		ret = dbmdx_read_buf_reg64(p, PRIORITY_RUN_ON_IDLE_ISR,
			MSG_ID_MAILBOX, REG_MB__INPLACE_READ_DATA,
			mb_buf_data->mailbox_id, chunk_samples,
			&buf, MB_BUFFER_CHUNK * mb_buf_data->num_channels);
		if (ret < 0) {
			dev_info(p->dev, "%s Error reading data\n", __func__);
			break;
		}

		ack_val = ret;
		kfifo_space = kfifo_avail(mb_buf_data->samples_fifo);

		dev_dbg(p->dev, "%s: len<%d> kfifo_space<%d> mode<%d>\n",
			__func__, ack_val, kfifo_space,
			mb_buf_data->buffering_mode);

		if (kfifo_space < ack_val)
			ack_val = kfifo_space;

		if (ack_val) {
			kfifo_in(mb_buf_data->samples_fifo, buf, ack_val);
		} else if (mb_buf_data->buffering_mode ==
						BUFFERING_UNTIL_MB_EMPTY) {
			dev_info(p->dev, "%s MB is empty, stop buffering...\n",
				__func__);
			break;
		}
		msleep(50);
	}

	/* Re configure threshold for this core/mailbox */
	if (mb_buf_data->fifo_type != DBMDX_FIFO_TYPE_AUDIO) {
		ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX,
					REG_MB__WATERMARK,
					mb_buf_data->mailbox_id,
					cdev_priv->pdata->core_log_watermark,
					&ack_val);
		if (ret < 0) {
			dev_err(p->dev, "%s: Error updating log watermark\n",
				__func__);
			goto out_free_mem;
		}

	}
	wake_up_interruptible_all(&cdev_priv->wait[mb_buf_data->fifo_type]);

out_free_mem:
	vfree(buf);
	mb_buf_data->buffering_mode = BUFFERING_DISABLED;
	return ret;
}


static void dbmdx_mb_buffering_work(struct work_struct *work)
{
	struct mb_buf_work_data *mb_buf_work =
				(struct mb_buf_work_data *)work;

	dbmdx_mailbox_buffering(mb_buf_work);
}

static void dbmdx_init_buffering_works(struct dbmdx_private *p)
{
	int i;
	struct dbmdx_cdev_private *cdev_priv = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	for (i = 0; i < MAX_NUM_OF_WORKS; i++)
		INIT_WORK(
		(struct work_struct *)&(cdev_priv->mb_buffering_work[i]),
			dbmdx_mb_buffering_work);
}

void dbmdx_start_debug_core_mb_buffering(struct dbmdx_private *p,
							u16 mailbox_id)
{
	enum dbmdx_fifo_type fifo_type;
	struct mb_buf_work_data *mb_buf_work = NULL;
	struct kfifo *samples_fifo = NULL;

	dev_info(p->dev, "%s: mailbox<%x>\n", __func__, mailbox_id);

	get_fifo_type_from_mb_id(mailbox_id, &fifo_type);
	get_mb_buf_work_from_fifo_type(p, fifo_type, &mb_buf_work);
	get_kfifo_from_fifo_type(p, fifo_type, &samples_fifo);


	mb_buf_work->p = p;
	mb_buf_work->buffering_mode = BUFFERING_UNTIL_MB_EMPTY;
	mb_buf_work->sample_rate = 16000;
	mb_buf_work->sample_width = ATTRIB_SMP_WIDTH_8_BITS;
	mb_buf_work->num_channels = 1;
	mb_buf_work->mailbox_id = mailbox_id;
	mb_buf_work->fifo_type = fifo_type;
	mb_buf_work->samples_fifo = samples_fifo;

	schedule_work((struct work_struct *)mb_buf_work);
}

/**
 * Configure watermark for 3 core mailboxes Total buffer length
 * is configured via the .dtsi
 */
#define PRINTF_BUFFER_LENGTH_WITH_LOGS	0x3800

int dbmdx_enable_core_logging(struct dbmdx_private *p,
				enum dbmdx_fifo_type fifo_type,
				bool use_polling_method)
{
	int  ret;
	u32  ack_val = 0;
	u32  core_log_watermark = 0;
	u16  cur_mb_id = LOGS_CORE_LP_MB_ID;
	struct dbmdx_cdev_private *cdev_priv = NULL;
	struct mb_buf_work_data *mb_buf_work = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	switch (fifo_type) {
	case DBMDX_FIFO_TYPE_LP_LOG:
		cur_mb_id = LOGS_CORE_LP_MB_ID;
		break;
	case DBMDX_FIFO_TYPE_HF0_LOG:
		cur_mb_id = LOGS_CORE_HF0_MB_ID;
		break;
	case DBMDX_FIFO_TYPE_HF1_LOG:
		cur_mb_id = LOGS_CORE_HF1_MB_ID;
		break;
	default:
		dev_err(p->dev, "%s: Inappropriate FIFO type: %d\n",
			__func__, (int)fifo_type);
		return -EINVAL;
	}
	mutex_lock(&mutex_critical_sec);
	if (p->primary_flags.logs_enabled == false) {
		dev_info(p->dev, "%s: Core Logging is disabled, enabling...\n",
				__func__);

		ret = dbmdx_read_register32(p,
					MSG_ID_FW, REG_FW__HOST_IF_CONFIG,
					&cdev_priv->host_if_config_value);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error reading REG_FW__HOST_IF_CONFIG\n",
				__func__);
			mutex_unlock(&mutex_critical_sec);
			return ret;
		}

		ret = dmbdx_mailbox_attr_disable_overflow_set(p,
					LOGS_CORE_LP_MB_ID, true);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error disabling MB overflow (LP)\n",
				__func__);
			mutex_unlock(&mutex_critical_sec);
			return ret;
		}

		ret = dmbdx_mailbox_attr_disable_overflow_set(p,
					LOGS_CORE_HF0_MB_ID, true);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error disabling MB overflow (HF0)\n",
				__func__);
			mutex_unlock(&mutex_critical_sec);
			return ret;
		}

		ret = dmbdx_mailbox_attr_disable_overflow_set(p,
					LOGS_CORE_HF1_MB_ID, true);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error disabling MB overflow (HF1)\n",
				__func__);
			mutex_unlock(&mutex_critical_sec);
			return ret;
		}

		ret = dbmdx_write_register32(p, PRIORITY_RUN_ON_IDLE_ISR,
				MSG_ID_FW, REG_FW__HOST_IF_CONFIG, 0x778);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error updating REG_FW__HOST_IF_CONFIG\n",
				__func__);
			mutex_unlock(&mutex_critical_sec);
			return ret;
		}

		p->primary_flags.logs_enabled = true;
	}
	mutex_unlock(&mutex_critical_sec);
	/* Read all existing logs from core using polling method*/

	core_log_watermark = 0;

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX,	REG_MB__WATERMARK,
					cur_mb_id, core_log_watermark,
					&ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error updating core_log_watermark\n",
			__func__);
		return ret;
	}
	ret = dmbdx_mailbox_attr_disable_overflow_set(p,
							cur_mb_id, true);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error disabling MB overflow\n",
				__func__);
		return ret;
	}

	dbmdx_start_debug_core_mb_buffering(p, cur_mb_id);

	get_mb_buf_work_from_fifo_type(p, fifo_type, &mb_buf_work);

	flush_work(&mb_buf_work->mb_buf_work);

	/* for polling remove watermark */
	if (!use_polling_method) {
		core_log_watermark = cdev_priv->pdata->core_log_watermark;

		ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
						MSG_ID_MAILBOX,
						REG_MB__WATERMARK,
						cur_mb_id, core_log_watermark,
						&ack_val);
		if (ret < 0) {
			dev_err(p->dev,
				"%s: Error updating core_log_watermark\n",
				__func__);
			return ret;
		}

		ret = dmbdx_mailbox_attr_disable_overflow_set(p,
							cur_mb_id, false);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error enabling MB overflow\n",
				__func__);
			return ret;
		}
	}

	return ret;
}

int dbmdx_disable_core_logging_functionality(struct dbmdx_private *p)
{
	int ret = 0;
	u32 ack_val = 0;
	struct dbmdx_cdev_private *cdev_priv = NULL;

	cdev_priv = (struct dbmdx_cdev_private *)p->cdev_data;

	dev_info(p->dev, "%s\n", __func__);

	if (p && p->primary_flags.logs_enabled) {
		p->primary_flags.logs_enabled = false;

		ret = dbmdx_write_register32_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_FW, REG_FW__HOST_IF_CONFIG,
					cdev_priv->host_if_config_value,
					&ack_val);
	}
	return ret;
}

static int dbmdx_set_core_logging_to_polling(struct dbmdx_private *p,
					enum dbmdx_fifo_type fifo_type)
{
	int  ret;
	u32  ack_val = 0;
	u16  cur_mb_id = LOGS_CORE_LP_MB_ID;
	struct mb_buf_work_data *mb_buf_work = NULL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	switch (fifo_type) {
	case DBMDX_FIFO_TYPE_LP_LOG:
		cur_mb_id = LOGS_CORE_LP_MB_ID;
		break;
	case DBMDX_FIFO_TYPE_HF0_LOG:
		cur_mb_id = LOGS_CORE_HF0_MB_ID;
		break;
	case DBMDX_FIFO_TYPE_HF1_LOG:
		cur_mb_id = LOGS_CORE_HF1_MB_ID;
		break;
	default:
		dev_err(p->dev, "%s: Inappropriate FIFO type: %d\n",
			__func__, (int)fifo_type);
		return -EINVAL;
	}

	get_mb_buf_work_from_fifo_type(p, fifo_type, &mb_buf_work);

	if (mb_buf_work->buffering_mode != BUFFERING_DISABLED) {
		mb_buf_work->buffering_mode = BUFFERING_DISABLED;
		flush_work(&mb_buf_work->mb_buf_work);
	}

	ret = dbmdx_write_register64_ack(p, PRIORITY_RUN_ON_IDLE_ISR,
					MSG_ID_MAILBOX,	REG_MB__WATERMARK,
					cur_mb_id, 0, &ack_val);
	if (ret < 0) {
		dev_err(p->dev, "%s: Error updating core_log_watermark\n",
			__func__);
		return ret;
	}

	ret = dmbdx_mailbox_attr_disable_overflow_set(p,
							cur_mb_id, true);
	if (ret < 0) {
		dev_err(p->dev,	"%s: Error disabling MB overflow\n", __func__);
		return ret;
	}

	return ret;
}


/* Access to the audio buffer is controlled through "audio_owner". Either the
 * character device or the ALSA-capture device can be opened.
 */
static int dbmdx_record_open(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	enum dbmdx_fifo_type fifo_type;
	int ret = 0;

	if (minor >= DBMDX_MAX_NUM_KFIFO)
		return -EBUSY;

	dev_dbg(dbmdx_p->dev, "%s: minor=%u\n", __func__, minor);

	file->private_data = dbmdx_p;

	if (!atomic_add_unless(&cdev_opened[minor], 1, 1))
		return -EBUSY;

	get_fifo_type_from_chdev_minor(minor, &fifo_type);

	ret = dbmdx_enable_core_logging(dbmdx_p, fifo_type, false);
	if (ret < 0) {
		dev_err(dbmdx_p->dev, "%s: Error enabling core logging\n",
			__func__);
		atomic_dec(&cdev_opened[minor]);
		return -EIO;
	}

	return 0;
}

static int dbmdx_record_release(struct inode *inode, struct file *file)
{
	unsigned int minor = MINOR(inode->i_rdev);
	enum dbmdx_fifo_type fifo_type;
	int ret = 0;

	dev_dbg(dbmdx_p->dev, "%s: minor=%u\n", __func__, minor);

	get_fifo_type_from_chdev_minor(minor, &fifo_type);

	ret = dbmdx_set_core_logging_to_polling(dbmdx_p, fifo_type);
	if (ret < 0) {
		dev_err(dbmdx_p->dev,
			"%s: Error setting core logging to polling\n",
			__func__);
		ret = -EIO;
		goto out;
	}
out:
	atomic_dec(&cdev_opened[minor]);

	return ret;
}

static unsigned int dbmdx_record_poll(struct file *file,
					struct poll_table_struct *wait)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;
	struct dbmdx_cdev_private *cdev_priv = p->cdev_data;
	enum dbmdx_fifo_type fifo_type;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	unsigned int mask = 0;

	get_fifo_type_from_chdev_minor(minor, &fifo_type);

	poll_wait(file, &cdev_priv->wait[fifo_type], wait);

	if (kfifo_len(&cdev_priv->fifo_db[fifo_type].samples_kfifo) > 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}


/*
 * read out of the kfifo as much as was requested or if requested more
 * as much as is in the FIFO
 */
ssize_t read_from_kfifo_non_blocking(struct dbmdx_private *p,
				char __user *buf,
				size_t count_want, loff_t *f_pos,
				struct kfifo *samples_kfifo)
{
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail;
	unsigned int copied;
	int ret;

	dev_dbg(p->dev, "%s: count_want:%zu f_pos:%lld\n",
			__func__, count_want, *f_pos);

	avail = kfifo_len(samples_kfifo);

	if (avail == 0)
		return -EAGAIN;

	if (count_want > avail)
		to_copy = avail;

	ret = kfifo_to_user(samples_kfifo, buf, to_copy, &copied);
	if (ret)
		return -EIO;

	not_copied = count_want - copied;
	*f_pos = *f_pos + (count_want - not_copied);

	return count_want - not_copied;
}

/*
 * read out of the kfifo as much as was requested and block until all
 * data is available or a timeout occurs
 */
ssize_t read_from_kfifo_blocking_timeout(struct dbmdx_private *p,
		char __user *buf, size_t count_want, loff_t *f_pos,
		struct kfifo *samples_kfifo, u16 timeout_ms)
{
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail = 0;
	unsigned int copied, total_copied = 0;
	int ret;
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	dev_dbg(p->dev, "%s: count_want:%zu f_pos:%lld\n",
			__func__, count_want, *f_pos);

	while ((total_copied < count_want) &&
			time_before(jiffies, timeout)) {

		avail = kfifo_len(samples_kfifo);

		if (avail == 0)
			msleep(50);

		if (avail > 0) {
			to_copy = avail;
			if (count_want - total_copied < avail)
				to_copy = count_want - total_copied;

			ret = kfifo_to_user(samples_kfifo,
				buf + total_copied, to_copy, &copied);
			if (ret)
				return -EIO;

			total_copied += copied;
		}
	}

	if (avail && (total_copied < count_want))
		dev_err(p->dev, "dbmdx: timeout during reading\n");

	not_copied = count_want - total_copied;
	*f_pos = *f_pos + (count_want - not_copied);

	dev_dbg(p->dev,
		"%s: count_want:%zu total copied:%u, not copied %zu\n",
			__func__, count_want, total_copied, not_copied);

	return count_want - not_copied;
}

ssize_t read_from_kfifo_blocking(struct dbmdx_private *p,
		char __user *buf, size_t count_want, loff_t *f_pos,
		struct kfifo *samples_kfifo, enum dbmdx_fifo_type fifo_type)
{
	struct dbmdx_cdev_private *cdev_priv = p->cdev_data;
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail;
	unsigned int copied;
	int ret;

	dev_dbg(p->dev, "%s: count_want:%zu f_pos:%lld\n",
			__func__, count_want, *f_pos);

	avail = kfifo_len(samples_kfifo);

	if (avail == 0) {
		wait_event_interruptible(cdev_priv->wait[fifo_type],
			(kfifo_len(samples_kfifo) > 0));

		if (signal_pending(current))
			return -ERESTARTSYS;

		avail = kfifo_len(samples_kfifo);
	}

	if (count_want > avail)
		to_copy = avail;

	ret = kfifo_to_user(samples_kfifo, buf, to_copy, &copied);
	if (ret)
		return -EIO;

	not_copied = count_want - copied;
	*f_pos = *f_pos + (count_want - not_copied);

	return count_want - not_copied;
}


static ssize_t dbmdx_record_read(struct file *file, char __user *buf,
				 size_t count_want, loff_t *f_pos)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;
	unsigned int minor = MINOR(file->f_inode->i_rdev);
	enum dbmdx_fifo_type fifo_type;
	struct kfifo *samples_fifo = NULL;

	get_fifo_type_from_chdev_minor(minor, &fifo_type);
	get_kfifo_from_fifo_type(p, fifo_type, &samples_fifo);
	if (file->f_flags & O_NONBLOCK)
		return read_from_kfifo_non_blocking(p, buf, count_want,
					f_pos, samples_fifo);
	else
		return read_from_kfifo_blocking(p, buf, count_want,
					f_pos, samples_fifo, fifo_type);
}

static const struct file_operations dbmdx_cdev_fops = {
	.owner   = THIS_MODULE,
	.open    = dbmdx_record_open,
	.release = dbmdx_record_release,
	.read    = dbmdx_record_read,
	.poll    = dbmdx_record_poll,
	.llseek  = noop_llseek,
};



static int dbmdx_chrdev_platform_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct dbmdx_cdev_pdata *cdev_pdata = NULL;
	struct dbmdx_cdev_private *cdev_priv = NULL;
	struct dbmdx_private *p = dev_get_drvdata(pdev->dev.parent);
	int enable_bits;
#ifdef CONFIG_OF
	struct  device_node *np;
#endif

	dbmdx_p = p;

#ifdef CONFIG_OF
	np = p->dev->of_node;
	if (!np) {
		dev_err(p->dev, "%s: no devicetree entry\n", __func__);
		return -EINVAL;
	}

	cdev_pdata = kzalloc(sizeof(struct dbmdx_cdev_pdata), GFP_KERNEL);
	if (!cdev_pdata)
		return -ENOMEM;

	p->pdata->cdev_pdata = cdev_pdata;
#else
	if (p->pdata == NULL || p->pdata->cdev_pdata == NULL) {
		dev_err(p->dev, "%s: dbmdx, no cdev platform data found\n",
			 __func__);
		ret = -ENODEV;
		goto out;
	}
#endif
	cdev_priv = kzalloc(sizeof(struct dbmdx_cdev_private), GFP_KERNEL);

	if (!cdev_priv) {
		ret = -ENOMEM;
		goto out_err_mem_free;
	}

	cdev_priv->pdata = (void *)cdev_pdata;
	cdev_priv->dev = &pdev->dev;
	p->cdev_data = (void *)cdev_priv;

#ifdef CONFIG_OF
	ret = of_property_read_u32(np, "core_log_watermark",
		&(cdev_pdata->core_log_watermark));
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid core_log_watermark\n", __func__);
		goto out_err_mem_free;
	} else if (ret ==  -EINVAL) {
		dev_err(p->dev,
			"%s: no core_log_watermark def in device-tree.\n",
			__func__);
		cdev_pdata->core_log_watermark = DEFAULT_CORE_LOG_WATERMARK;
	}

	ret = of_property_read_u32(np, "fifo_en_mask",
		&cdev_pdata->fifo_en_mask);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid fifo_en_mask\n", __func__);
		goto out_err_mem_free;
	} else if (ret ==  -EINVAL) {
		dev_err(p->dev,
			"%s: no fifo_en_mask def in device-tree.\n",
			__func__);
		cdev_pdata->fifo_en_mask = DEFAULT_FIFO_EN_MASK;
	}
#endif

	if (cdev_pdata->fifo_en_mask == 0) {
		dev_info(p->dev, "%s: No Fifo enabled, exit...\n", __func__);
		return 0;
	}
	enable_bits = cdev_pdata->fifo_en_mask;

	for (i = 0; i < DBMDX_MAX_NUM_KFIFO; i++) {
		struct dbmdx_fifo_db *fdb = &cdev_priv->fifo_db[i];
		const int fsize = MAX_KFIFO_BUFFER_SIZE;

		init_waitqueue_head(&cdev_priv->wait[i]);

		if (!(enable_bits & BIT(i)))
			continue;

		ret = dbmdx_cdev_create(&cdev_priv->cdev[i], pdev->dev.parent,
			&dbmdx_cdev_fops, cdev_priv, "dbmdx-%d", i);
		if (ret)
			break;

		dev_info(p->dev, "%s fifo_num<%d> enabled\n", __func__, i);

		fdb->samples_kfifo_buf_size = fsize;
		fdb->samples_kfifo_buf = kmalloc(fsize, GFP_KERNEL);
		if (!fdb->samples_kfifo_buf) {
			ret = -ENOMEM;
			break;
		}

		kfifo_init(&fdb->samples_kfifo, fdb->samples_kfifo_buf, fsize);
	}
	if (ret)
		goto out_err_fifo_release;

	dbmdx_init_buffering_works(p);

	/* FIXME: usage_count is required to be two for
	 * pm_runtime_force_resume to invoke runtime_resume callback.
	 * Hence use of pm_runtime_get_no_resume twice.
	 * Need to check if its kernel version specific issue.
	 */
	pm_runtime_get_noresume(cdev_priv->dev);
	pm_runtime_get_noresume(cdev_priv->dev);
	pm_runtime_set_active(cdev_priv->dev);
	pm_runtime_enable(cdev_priv->dev);

	platform_set_drvdata(pdev, cdev_priv);

	return 0;

out_err_fifo_release:
	for (i = 0; i < DBMDX_MAX_NUM_KFIFO; i++) {
		struct dbmdx_fifo_db *fdb = &cdev_priv->fifo_db[i];

		dbmdx_cdev_destroy(&cdev_priv->cdev[i]);
		kfree(fdb->samples_kfifo_buf);
		kfifo_free(&fdb->samples_kfifo);
	}
out_err_mem_free:
#ifdef CONFIG_OF
	kfree(cdev_pdata);
#endif
	kfree(cdev_priv);

	return ret;
}

static int dbmdx_chrdev_platform_remove(struct platform_device *pdev)
{
	struct dbmdx_cdev_pdata *cdev_pdata = NULL;
	struct dbmdx_cdev_private *cdev_priv = platform_get_drvdata(pdev);
	struct dbmdx_private *p = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	int enable_bits;
	int fifo_num;

	cdev_pdata = cdev_priv->pdata;
	enable_bits = cdev_pdata->fifo_en_mask;

	pm_runtime_disable(cdev_priv->dev);

	if (cdev_priv == NULL || cdev_pdata == NULL) {
		dev_err(dev, "%s: Ch device not initialized\n", __func__);
		return -EINVAL;
	}

	for (fifo_num = 0; fifo_num < DBMDX_MAX_NUM_KFIFO; fifo_num++) {
		if (enable_bits & 0x01)	{
			dev_info(dev, "%s fifo_num<%d> enabled\n",
				__func__, fifo_num);
			kfifo_free(&cdev_priv->fifo_db[fifo_num].samples_kfifo);

			dbmdx_cdev_destroy(&cdev_priv->cdev[fifo_num]);
		}
		enable_bits = enable_bits>>1;
	}

	dbmdx_p = NULL;
#ifdef CONFIG_OF
	kfree(cdev_pdata);
	p->pdata->cdev_pdata = NULL;
#endif
	kfree(cdev_priv);
	p->cdev_data = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int dbmdx_cdev_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int dbmdx_cdev_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}
#endif

static const struct dev_pm_ops dbmdx_cdev_pm_ops = {
	SET_RUNTIME_PM_OPS(dbmdx_cdev_runtime_suspend,
			   dbmdx_cdev_runtime_resume, NULL)
};

static const struct of_device_id dbmdx_cdev_match[] = {
	{ .compatible = "dspg,dbmdx-log", },
	{},
};

static struct platform_driver dbmdx_chrdev_driver = {
	.driver = {
		.name = "dbmdx-log",
		.of_match_table = dbmdx_cdev_match,
		.pm = &dbmdx_cdev_pm_ops,
	},

	.probe = dbmdx_chrdev_platform_probe,
	.remove = dbmdx_chrdev_platform_remove,
};

module_platform_driver(dbmdx_chrdev_driver);

MODULE_DESCRIPTION("DSPG DBMDX Character Device driver");
MODULE_AUTHOR("DSP Group");
MODULE_LICENSE("GPL v2");

