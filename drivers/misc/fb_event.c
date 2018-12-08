/*
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fb_event.h>
#include <uapi/linux/fb_event.h>

#define FBEVENTS_DEV_NAME "fbevent"
#define FB_EVENT_COUNT_MAX 16
#define FLG_OPEN_LOCK 0

struct fb_event_item {
	struct list_head item;
	struct fb_event data;
};

struct fb_event_private {
	wait_queue_head_t pollq;
	struct miscdevice misc_dev;
	struct list_head eventq;
	unsigned long flags;
	atomic_t fb_event_count;
	spinlock_t lock;
};

static struct fb_event_private *fb_event_ctx;

int fb_event_log_int(int32_t type, int value)
{
	struct fb_event_private *priv = fb_event_ctx;
	struct fb_event_item *event = NULL;
	gfp_t gfp = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;
	unsigned long flags;
	struct timeval tv;
	int ret = 0;

	if (!priv)
		return -EAGAIN;

	spin_lock_irqsave(&priv->lock, flags);
	if ((atomic_read(&priv->fb_event_count) > FB_EVENT_COUNT_MAX - 1) &&
			!list_empty(&priv->eventq)) {
		event = list_first_entry(&priv->eventq,
				struct fb_event_item, item);
		list_del(&event->item);
		atomic_dec(&priv->fb_event_count);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	if (!event) {
		event = kmalloc(sizeof(*event), gfp);
		if (!event)
			return -ENOMEM;
	}

	INIT_LIST_HEAD(&event->item);
	do_gettimeofday(&tv);
	event->data.tv_sec = tv.tv_sec;
	event->data.tv_usec = tv.tv_usec;
	event->data.type = type;
	event->data.value = value;

	spin_lock_irqsave(&priv->lock, flags);
	list_add_tail(&event->item, &priv->eventq);
	spin_unlock_irqrestore(&priv->lock, flags);

	atomic_inc(&priv->fb_event_count);

	wake_up_all(&priv->pollq);

	return ret;
}
EXPORT_SYMBOL(fb_event_log_int);

static int fb_event_open(struct inode *inode, struct file *filep)
{
	struct fb_event_private *priv = container_of(filep->private_data,
			struct fb_event_private, misc_dev);

	filep->private_data = priv;

	if (test_and_set_bit(FLG_OPEN_LOCK, &priv->flags))
		return -EBUSY;

	return 0;
}

static int fb_event_release(struct inode *inode, struct file *filep)
{
	struct fb_event_private *priv = filep->private_data;

	clear_bit(FLG_OPEN_LOCK, &priv->flags);

	return 0;
}

static ssize_t fb_event_read(struct file *filep, char *buffer,
		size_t length, loff_t *offset)
{
	struct fb_event_private *priv = filep->private_data;
	struct fb_event_item *event;
	const size_t size = sizeof(event->data);
	unsigned long flags;
	size_t written = 0;
	int err = 0;

	if (length < sizeof(event->data))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	while (!err && length >= size && !list_empty(&priv->eventq)) {
		event = list_first_entry(&priv->eventq,
				struct fb_event_item, item);
		list_del(&event->item);
		spin_unlock_irqrestore(&priv->lock, flags);

		err = copy_to_user(buffer, &event->data, size);

		kfree(event);
		atomic_dec(&priv->fb_event_count);
		WARN_ON(atomic_read(&priv->fb_event_count) < 0);

		if (!err) {
			length -= size;
			written += size;
			*offset += size;
			buffer += size;
		}
		spin_lock_irqsave(&priv->lock, flags);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return written;
}

static unsigned int fb_event_poll(struct file *filep,
		struct poll_table_struct *wait)
{
	struct fb_event_private *priv = filep->private_data;
	unsigned long flags;
	unsigned int ret = 0;

	poll_wait(filep, &priv->pollq, wait);

	spin_lock_irqsave(&priv->lock, flags);
	ret |= list_empty(&priv->eventq) ? 0 : POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

const struct file_operations fbevent_fops = {
	.owner = THIS_MODULE,
	.read = fb_event_read,
	.poll = fb_event_poll,
	.open = fb_event_open,
	.release = fb_event_release,
};

static int fb_event_init(void)
{
	struct fb_event_private *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->pollq);

	atomic_set(&priv->fb_event_count, 0);
	INIT_LIST_HEAD(&priv->eventq);

	priv->misc_dev.minor = MISC_DYNAMIC_MINOR;
	priv->misc_dev.name = FBEVENTS_DEV_NAME;
	priv->misc_dev.fops = &fbevent_fops;

	ret = misc_register(&priv->misc_dev);
	if (ret)
		pr_err("Unable to register misc device, ret=%d\n", ret);

	fb_event_ctx = priv;

	return ret;
}

static void fb_event_exit(void)
{
	struct fb_event_private *priv;
	struct fb_event_item *event;
	unsigned long flags;

	priv = xchg(&fb_event_ctx, NULL);

	misc_deregister(&priv->misc_dev);

	spin_lock_irqsave(&priv->lock, flags);
	while (!list_empty(&priv->eventq)) {
		event = list_first_entry(&priv->eventq,
				struct fb_event_item, item);
		list_del(&event->item);
		kfree(event);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	list_del_init(&priv->eventq);

	kfree(priv);
}

module_init(fb_event_init);
module_exit(fb_event_exit);

MODULE_AUTHOR("Kiril Petrov <kpetrov@mm-sol.com>");
MODULE_DESCRIPTION("Facebook event driver");
MODULE_LICENSE("GPL");
