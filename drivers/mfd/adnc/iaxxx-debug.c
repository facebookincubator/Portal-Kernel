/*
 * iaxxx-debug.c  --  iaxxx debug support
 *
 * Copyright 2017 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kdev_t.h>
#include <linux/mfd/adnc/iaxxx-debug-intf.h>
#include <linux/circ_buf.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"

#define GET_INTF_PRIV(iaxxx) \
	((!iaxxx || !iaxxx->intf_priv) ? NULL : iaxxx->intf_priv)
#define IAXXX_I2C		0
#define IAXXX_SPI		1
#define IAXXX_MAX_LOG_SIZE	50

struct iaxxx_debug_data {
	struct iaxxx_priv *priv;
	struct iaxxx_cdev raw_cdev;
	struct iaxxx_cdev regdump_cdev;
	struct iaxxx_cdev crashdump_cdev;
};

static ssize_t raw_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf = NULL;

	pr_debug("%s() called\n", __func__);

	if (!iaxxx) {
		pr_err("Invalid private pointer");
		rc = -EINVAL;
		goto raw_read_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("failed to copy user data of len: %d\n", (u32)count);
		rc = -ENOMEM;
		goto raw_read_err;
	}

	rc = iaxxx->raw_ops->read(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("failed to read data: %d", rc);
		rc = -EIO;
	} else {
		rc = copy_to_user(buf, kbuf, count);
		if (rc) {
			pr_err("failed to copy response to userspace %d", rc);
			rc = -EIO;
			goto raw_read_err;
		}

		rc = count;
	}

raw_read_err:
	kfree(kbuf);
	return rc;
}

static ssize_t raw_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	int rc = 0;
	void *kbuf;

	pr_debug("%s() called\n", __func__);

	if (!iaxxx) {
		pr_err("Invalid private pointer");
		rc = -EINVAL;
		goto raw_write_err;
	}

	kbuf = memdup_user(buf, count);
	if (!kbuf) {
		pr_err("failed to copy user data of len: %d\n", (u32)count);
		rc = -ENOMEM;
		goto raw_write_err;
	}

	rc = iaxxx->raw_ops->write(iaxxx, kbuf, count);
	if (rc < 0) {
		pr_err("failed to write data: %d", rc);
		rc = -EIO;
	} else {
		rc = count;
	}

	kfree(kbuf);

raw_write_err:
	return rc;
}

static long raw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct iaxxx_priv * const iaxxx
			= (struct iaxxx_priv *)file->private_data;
	int ret = 0;
	u8 bus_config;

	pr_debug("%s() called\n", __func__);

	if (iaxxx == NULL) {
		pr_err("Invalid private pointer");
		return -EINVAL;
	}

	if (!pm_runtime_active(iaxxx->dev))
		return -ENXIO;

	switch (cmd) {
	case IAXXX_BUS_CONFIG:
#if defined(CONFIG_MFD_IAXXX_SPI)
		bus_config = IAXXX_SPI;
#elif defined(CONFIG_MFD_IAXXX_I2C)
		bus_config = IAXXX_I2C;
#endif
		ret = copy_to_user((void __user *)arg, &bus_config,
								sizeof(u8));
		if (ret) {
			pr_err("failed to copy response to userspace %d", ret);
			ret = -EIO;
		}
		break;
	default:
		pr_err("Invalid ioctl command received %x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long raw_compat_ioctl(struct file *filp, unsigned int cmd,
						unsigned long arg)
{
	return raw_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int raw_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("raw device open called\n");

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, raw_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;

	return 0;
}

static int raw_release(struct inode *inode, struct file *filp)
{
	pr_debug("raw device release called\n");

	filp->private_data = NULL;
	return 0;
}

static ssize_t regdump_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	char *kbuf;
	int err;

	dev_dbg(iaxxx->dev, "%s() called\n", __func__);
	if (!iaxxx)
		return -EINVAL;
	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -EFAULT;
	err = copy_from_user(kbuf, buf, count);
	if (err) {
		dev_err(iaxxx->dev, "%s() Copy error\n", __func__);
		return -EINVAL;
	}
	if (!strncmp(kbuf, "clear", (count - 1))) {
		spin_lock(&iaxxx->reg_dump->ring_lock);
		iaxxx->reg_dump->head = 0;
		iaxxx->reg_dump->tail = 0;
		spin_unlock(&iaxxx->reg_dump->ring_lock);
	} else
		dev_err(iaxxx->dev, "%s() Invalid command\n", __func__);
	return count;
}

static ssize_t regdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *iaxxx = (struct iaxxx_priv *)filp->private_data;
	struct iaxxx_reg_dump_priv *reg_dump;
	struct iaxxx_register_log log;
	char kbuf[IAXXX_MAX_LOG_SIZE];
	ssize_t bytes_written = 0;

	dev_dbg(iaxxx->dev, "%s() called\n", __func__);
	/* Return if no priv structure */
	if (!iaxxx)
		return -EINVAL;
	if (!iaxxx->reg_dump)
		return -EINVAL;

	reg_dump = iaxxx->reg_dump;

	/* reading first time or last time read is complete */
	if (!*f_pos) {
		bytes_written += scnprintf(kbuf, PAGE_SIZE,
				"R/W:\t[TimeStamp]\t0xAddress\t0xValue\n");
		if (copy_to_user(buf, kbuf, strlen(kbuf))) {
			kfree(kbuf);
			return -EFAULT;
		}
	}
	/* Copy to user as many log messages fits into count */
	while ((count - bytes_written) > IAXXX_MAX_LOG_SIZE) {
		size_t pos = bytes_written;

		spin_lock(&reg_dump->ring_lock);
		if (!CIRC_CNT(reg_dump->head, reg_dump->tail,
				IAXXX_BUF_MAX_LEN)) {
			spin_unlock(&reg_dump->ring_lock);
			break;
		}
		/* Increment read index and align with ring buffer boundary */
		log = reg_dump->log[reg_dump->tail];
		reg_dump->tail = (reg_dump->tail + 1) % IAXXX_BUF_MAX_LEN;
		spin_unlock(&reg_dump->ring_lock);

		bytes_written += scnprintf(kbuf, IAXXX_MAX_LOG_SIZE,
			"%c:\t[%lu.%03lu]\t0x%08x\t0x%08x\n",
			log.op == IAXXX_READ ? 'R' : 'W',
			log.timestamp.tv_sec,
			log.timestamp.tv_nsec / (1000*1000),
			log.addr, log.val);

		/* Copy data to user buffer */
		if (copy_to_user(buf + pos, kbuf, strlen(kbuf))) {
			bytes_written = -EFAULT;
			break;
		}
	}

	if (bytes_written > 0)
		*f_pos += bytes_written;

	return bytes_written;
}

static int regdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("%s()\n", __func__);

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode");
		return -ENODEV;
	}

	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, regdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch register dump private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int regdump_release(struct inode *inode, struct file *filp)
{
	pr_debug("%s() called\n", __func__);

	filp->private_data = NULL;
	return 0;
}

static int crashdump_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_debug_data *intf_priv;
	struct iaxxx_priv *priv;

	pr_debug("%s()\n", __func__);

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode");
		return -ENODEV;
	}
	intf_priv = container_of((inode)->i_cdev,
			struct iaxxx_debug_data, crashdump_cdev.cdev);

	if (intf_priv == NULL) {
		pr_err("Unable to fetch crash dump private data");
		return -ENODEV;
	}

	priv = intf_priv->priv;
	filp->private_data = priv;
	return 0;
}

static int crashdump_release(struct inode *inode, struct file *filp)
{
	pr_debug("%s() called\n", __func__);

	filp->private_data = NULL;
	return 0;
}

static ssize_t crashdump_read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	struct iaxxx_priv *priv = (struct iaxxx_priv *)filp->private_data;
	ssize_t size;

	dev_dbg(priv->dev, "%s() called\n", __func__);

	/* Return if no priv structure */
	if (!priv)
		return -EINVAL;
	if (!priv->crashlog)
		return -EINVAL;
	if (!priv->crashlog->log_buffer)
		return -EINVAL;
	mutex_lock(&priv->crashdump_lock);
	/* Read done */
	if (priv->crashlog->logs_read == priv->crashlog->log_buffer_size) {
		priv->crashlog->logs_read = 0;
		mutex_unlock(&priv->crashdump_lock);
		return 0;
	}

	size = priv->crashlog->log_buffer_size - priv->crashlog->logs_read;
	size = min_t(size_t, count, size);

	/* Copy data to user buffer */
	if (copy_to_user(buf, priv->crashlog->log_buffer
				+ priv->crashlog->logs_read, size)) {
		mutex_unlock(&priv->crashdump_lock);
		return -EFAULT;
	}
	priv->crashlog->logs_read += size;
	mutex_unlock(&priv->crashdump_lock);

	return size;
}

static const struct file_operations raw_fops = {
	.owner = THIS_MODULE,
	.open = raw_open,
	.read = raw_read,
	.write = raw_write,
	.unlocked_ioctl = raw_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = raw_compat_ioctl,
#endif
	.release = raw_release,
};

static const struct file_operations regdump_fops = {
	.owner = THIS_MODULE,
	.open = regdump_open,
	.read = regdump_read,
	.write = regdump_write,
	.release = regdump_release,
};

static const struct file_operations crashdump_fops = {
	.owner = THIS_MODULE,
	.open = crashdump_open,
	.read = crashdump_read,
	.release = crashdump_release,
};

int iaxxx_debug_init(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = NULL;
	int err;

	pr_debug("%s: initializing debug interface", __func__);

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	intf_priv = devm_kzalloc(priv->dev, sizeof(*intf_priv), GFP_KERNEL);
	if (!intf_priv)
		return -ENOMEM;

	priv->intf_priv = intf_priv;
	intf_priv->priv = priv;

	err = iaxxx_cdev_create(&intf_priv->raw_cdev, priv->dev,
		&raw_fops, intf_priv, IAXXX_CDEV_DEBUG);
	if (err) {
		pr_err("%s() error in creating the raw char device\n",
				__func__);
		err = -EIO;
		goto raw_cdev_err;
	}

	err = iaxxx_cdev_create(&intf_priv->regdump_cdev, priv->dev,
		&regdump_fops, intf_priv, IAXXX_CDEV_REGDUMP);
	if (err) {
		pr_err("error in creating the char device");
		err = -EIO;
		goto regdump_cdev_err;
	}

	err = iaxxx_cdev_create_name(&intf_priv->crashdump_cdev, priv->dev,
				&crashdump_fops, intf_priv, "crashdump");
	if (err) {
		pr_err("error in creating the char device");
		err = -EIO;
		goto crashdump_cdev_err;
	}

	priv->raw_ops = kmalloc(sizeof(struct iaxxx_raw_bus_ops), GFP_KERNEL);
	if (!priv->raw_ops) {
		err = -ENOMEM;
		goto raw_mem_alloc_err;
	}
	return err;
raw_mem_alloc_err:
	iaxxx_cdev_destroy(&intf_priv->crashdump_cdev);
crashdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev);
regdump_cdev_err:
	iaxxx_cdev_destroy(&intf_priv->raw_cdev);
raw_cdev_err:

	devm_kfree(priv->dev, intf_priv);
	priv->intf_priv = NULL;
	return err;
}
EXPORT_SYMBOL(iaxxx_debug_init);

int iaxxx_debug_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_debug_data *intf_priv = priv->intf_priv;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	iaxxx_cdev_destroy(&intf_priv->raw_cdev);
	iaxxx_cdev_destroy(&intf_priv->regdump_cdev);
	devm_kfree(priv->dev, intf_priv);

	kfree(priv->raw_ops);

	return 0;
}
EXPORT_SYMBOL(iaxxx_debug_exit);
