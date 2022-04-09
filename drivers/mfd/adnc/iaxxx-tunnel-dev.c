/*
 * iaxxx-tunnel-dev.c -- iaxxx Tunneling Service device node
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#define pr_fmt(fmt) "iaxxx : %s:%d, " fmt "\n", __func__, __LINE__

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <uapi/linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/pm_wakeup.h>
#include <linux/of.h>
#include <linux/alarmtimer.h>

#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel-priv.h"
#include "iaxxx-tunnel-dev.h"
#include <trace/events/fb_audio.h>

#define PBUFF_SIZE			(1024*1024)	/* Producer Buffer */
#define UBUFF_SIZE			(256*1024)	/* Client Fifo */

#define TNL_SYNC_MODE 0
#define TNL_ASYNC_MODE 1
#define MAX_PACKET_SIZE 512

#define CHECK_SEQUENCE

#define IAXXX_TUNNEL_TERMINATE_TIMEOUT	1000

#define IAXXX_TFLG_FW_CRASH		0
#define IAXXX_TFLG_RUN_THREAD		1 /* WA: This is temporary solution */
#define IAXXX_TFLG_ADJ_CLIENT		2 /* Trigger adjust tunnels by client */
#define IAXXX_TFLG_ADJ_THREAD		3 /* Trigger adjust tunnels by thread */
#define IAXXX_TFLG_THRESHOLD		4 /* Register for threshold */
#define IAXXX_TFLG_DRAIN_BIT		5
#define IAXXX_TFLG_ALLOW_TUNNELS	6
#define IAXXX_TFLG_DISABLE_ALL		7 /* Unsubscribe all tunnels */

#define TUNNEL_EVT_SYSID		0x2670

/* Min size would be Q15 format (256 bytes) + header size */
#define IAXXX_MIN_TUNNEL_SIZE		(256+sizeof(struct iaxxx_tunnel_header))

#define IAXXX_TMR_INTERVAL_MS		16 /* 2 frames */

struct iaxxx_circ_buf {
	char *buf;
	int head;
	int tail;
	int size;
};

struct iaxxx_tunnel_client {
	struct iaxxx_tunnel_data *tunnel_data;
	struct iaxxx_circ_buf user_circ;
	struct list_head node;
	wait_queue_head_t wq;
	unsigned long tid_flag;
	struct wakeup_source ws;
	char ws_name[255];
};

struct iaxxx_tunnel_data {
	struct device *dev; /* mfd device */
	struct iaxxx_priv *priv;

	struct iaxxx_cdev tunnel_cdev;

	struct iaxxx_circ_buf stream_circ;

	struct list_head list;
	spinlock_t lock;

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	struct dentry *dfs;
	int debug_enabled;
	int debug_saved;
	unsigned long debug_flag;
	struct iaxxx_circ_buf debug_stream_circ;
	struct iaxxx_tunnel_client debug_client;
	bool iaxxx_debug_initialized;
	size_t debug_rsize;
	int debug_start_idx;
	struct mutex debug_lock;
	struct completion termination_done;
#endif

	int client_no;
	atomic_t event_occurred;
	uint32_t tunnel_seq_no[TNLMAX];
	unsigned long tunnel_packet_no[TNLMAX];
	unsigned long tunnel_total_packet_no;
	uint32_t tunnel_seq_err[TNLMAX];
	unsigned long tunnel_magic_errcnt;
	struct task_struct *producer_thread;
	struct task_struct *consumer_thread;
	wait_queue_head_t consumer_wq;
	wait_queue_head_t producer_wq;
	struct notifier_block crash_notifier_fbp;
	unsigned long flags;	/* IAXXX_TFLG_....... */
	struct alarm tmr;
};

static phys_addr_t iaxxx_prod_buf;
static size_t iaxxx_prod_buf_size;

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
enum {
	DEBUG_FLAG_DISABLED,
	DEBUG_FLAG_ENABLED,
	DEBUG_FLAG_ENABLED_DEFER
};

#endif

static const uint32_t *tunnel_src_debug;
static uint32_t tunnel_src_debug_size;

static unsigned long tunnel_flags;

#define is_valid_tid(__id__) ((__id__) < ARRAY_SIZE(iaxxx_tunnel_src))
#define is_valid_size(__size__) ((__size__) > 0 && \
			(__size__) <= MAX_PACKET_SIZE && \
			!((__size__) & 3))

#define circ_cnt(__c__) \
	(CIRC_CNT((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space(__c__) \
	(CIRC_SPACE((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_cnt_to_end(__c__) \
	(CIRC_CNT_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

#define circ_space_to_end(__c__) \
	(CIRC_SPACE_TO_END((__c__)->head, (__c__)->tail, (__c__)->size))

/*
 * Init head / tail
 */
static inline void circ_init(struct iaxxx_circ_buf *circ)
{
	circ->head = circ->tail = 0;
}

/*
 * Update tail
 */
static inline void circ_peek_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->tail = (circ->tail + (size >> 2)) & (circ->size - 1);
}

/*
 * Update head pointer
 */
static inline void circ_fill_finish(struct iaxxx_circ_buf *circ, int size)
{
	smp_wmb(); /* Make sure data is copied before updating indexes */
	circ->head = (circ->head + (size >> 2)) & (circ->size - 1);
}

/*
 * Return maximum contiguous free buffer
 */
static inline int circ_get_free_buffer(struct iaxxx_circ_buf *circ, void **buf)
{
	int size = circ_space_to_end(circ) << 2;

	if (size > 0)
		*buf = &circ->buf[circ->head << 2];

	return size;
}

/*
 * Fill buf with the data in circular queue but no tail moving
 */
static inline int circ_peek_data(struct iaxxx_circ_buf *circ,
						void *buf, int size)
{
	u8 *p = buf;

	int len = min(size, circ_cnt(circ) << 2);
	int first_len =
		min(len, circ_cnt_to_end(circ) << 2);

	smp_rmb(); /* Make sure fetching indexes before data copy */
	memcpy(p, circ->buf + (circ->tail << 2), first_len);
	if (first_len < len)
		memcpy(p + first_len, circ->buf, len - first_len);

	return len;
}

/*
 * Push data to circular queue
 */
static inline void circ_in(struct iaxxx_circ_buf *circ,
					const void *buf, int size)
{
	void *user_buf = NULL;
	int len;

	len = circ_get_free_buffer(circ, &user_buf);
	if (len < size) {
		if (user_buf)
			memcpy(user_buf, buf, len);
		memcpy(circ->buf, buf + len, size - len);
	} else if (user_buf) {
		memcpy(user_buf, buf, size);
	}

	circ_fill_finish(circ, size);
}

/*
 * Copy data from circular buffer to user buffer
 */
static int circ_copy_to_user(struct iaxxx_circ_buf *circ,
		char __user *buf, int size)
{
	int len;
	int ret;

	len = min(size, circ_cnt_to_end(circ) << 2);
	ret = copy_to_user(buf, circ->buf + (circ->tail << 2), len);
	if (ret)
		return -EFAULT;

	if (size > len) {
		ret = copy_to_user(buf + len, circ->buf, size - len);
		if (ret)
			return -EFAULT;
	}

	circ_peek_finish(circ, size);

	return 0;
}

/*
 * If user buffer is greater then circular queue data size,
 * then copy all data to user buffer.
 * If user buffer is less than circular queue data size,
 * then check each packet and copy only whole packet can
 * fit into user buffer so that there's no split packet
 * stored in user buffer.
 */
static int circ_to_user(struct iaxxx_circ_buf *circ, char __user *buf,
		size_t count, unsigned int *copied)
{
	struct iaxxx_tunnel_header header;
	int hdr_size = sizeof(struct iaxxx_tunnel_header);
	int size;
	unsigned int sum = 0;
	int cnt = min_t(int, circ_cnt(circ) << 2, count);

	if (count > cnt) {
		/*
		 * If user buffer is big enough, copy all
		 * since the circular buffer has already
		 * packet aligned
		 */
		if (circ_copy_to_user(circ, buf, cnt))
			return -EFAULT;

		*copied = cnt;
		return 0;
	}

	/* User buffer is small, so check the packet one by one */
	while (1) {
		size = circ_peek_data(circ, &header, hdr_size);
		if (size < hdr_size)
			break;

		size += header.size;
		if ((circ_cnt(circ) << 2) < size)
			break;

		if (count < (sum + size))
			break;

		if (circ_copy_to_user(circ, buf + sum, size))
			break;

		sum += size;
	}

	if (sum == 0)
		return -EFAULT;

	*copied = sum;
	return 0;
}

/*
 * Parse packet header and return tunnel ID and data len
 */
static inline int parse_header(struct iaxxx_tunnel_header *header, u16 *tid)
{
	if (header->magic[0] != TUNNEL_MAGIC1 ||
		header->magic[1] != TUNNEL_MAGIC2 ||
		header->magic[2] != TUNNEL_MAGIC3 ||
		header->magic[3] != TUNNEL_MAGIC4 ||
		!is_valid_tid(header->tunnel_id) ||
		!is_valid_size(header->size)) {
		pr_debug("Fault packet: magic: %x %x %x %x, tid = %d, size = %d",
			header->magic[0], header->magic[1],
			header->magic[2], header->magic[3],
			header->tunnel_id, header->size);
		return -EINVAL;
	}

	*tid = header->tunnel_id;
	return header->size;
}

static unsigned long adjust_tunnels_new(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_tunnel_client *client;
	unsigned long new = 0;

	spin_lock(&t_intf_priv->lock);
	list_for_each_entry_rcu(client, &t_intf_priv->list, node)
		new |= client->tid_flag;
	spin_unlock(&t_intf_priv->lock);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	if (t_intf_priv->iaxxx_debug_initialized &&
	    t_intf_priv->debug_enabled == DEBUG_FLAG_ENABLED)
		new |= t_intf_priv->debug_client.tid_flag;
#endif

	return new;
}

/*
 * Adjust tunnel settings by comparing new & old flags
 */
static void adjust_tunnels(struct iaxxx_tunnel_data *t_intf_priv)
{
	struct iaxxx_priv *const priv = (struct iaxxx_priv *)t_intf_priv->priv;
	unsigned long *flags = &t_intf_priv->flags;
	unsigned long changes, old, new = 0;
	int id, pos;

	if (!test_and_clear_bit(IAXXX_TFLG_ADJ_THREAD, flags))
		return;

	old = tunnel_flags;

	if (!test_and_clear_bit(IAXXX_TFLG_DISABLE_ALL, flags))
		new = adjust_tunnels_new(t_intf_priv);

	if (new == old)
		return;

	changes = (old ^ new) & ((1 << TNLMAX) - 1);

	while (changes) {
		id = ffs(changes) - 1;
		pos = (1 << id);

		if (new & pos) {
			pr_notice("setup tunnel %d, src %x, mode %s, enc %s\n",
				id, iaxxx_tunnel_src[id].src & 0xffff,
				(iaxxx_tunnel_src[id].mode == TNL_SYNC_MODE) ?
					"SYNC" : "ASYNC",
				(iaxxx_tunnel_src[id].enc == TNL_ENC_AFLOAT) ?
				"AFLOAT" : "Q15");

			/* Init sequence no */
			t_intf_priv->tunnel_seq_no[id] = 0;

			if (iaxxx_tunnel_setup(priv, id,
					iaxxx_tunnel_src[id].src & 0xffff,
					iaxxx_tunnel_src[id].mode,
					iaxxx_tunnel_src[id].enc))
				pr_err("%s: iaxxx_tunnel_setup failed\n",
						__func__);
		} else {
			pr_notice("terminate tunnel %d\n", id);
			if (iaxxx_tunnel_terminate(priv, id))
				pr_err("iaxxx_tunnel_terminate failed\n");
		}
		changes &= ~pos;

		old ^= pos;
	}

	tunnel_flags = old;

	if (tunnel_flags) {
		/* Kick alarm to start streaming tunnel data */
		if (t_intf_priv->tmr.state ==
				ALARMTIMER_STATE_INACTIVE) {

			/* Flush buffer for the first time */
			iaxxx_flush_tunnel_fw_buff(priv);

			atomic_set(&t_intf_priv->event_occurred, 0);
			alarm_start(&t_intf_priv->tmr,
				ktime_add_ms(ktime_get(),
					IAXXX_TMR_INTERVAL_MS));
			trace_fba_alarm_state(1);
		}
	} else {
		alarm_cancel(&t_intf_priv->tmr);
		trace_fba_alarm_state(0);
	}
}

static void adjust_tunnels_do(struct iaxxx_tunnel_data *tpriv,
				struct iaxxx_tunnel_client *client)
{
	if (client &&
	    test_and_clear_bit(IAXXX_TFLG_ADJ_CLIENT, &tpriv->flags))
		set_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags);

	if (test_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags))
		wake_up(&tpriv->producer_wq);
}

/*
 * Get tunnel data from codec and fill * into circular buffer
 */
static int producer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_priv *priv = t_intf_priv->priv;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;
	int bytes;
	void *buf;
	while (1) {
		/* Get a free contiguous buffer */
		if (kthread_should_stop()) {
			set_bit(IAXXX_TFLG_DISABLE_ALL, &t_intf_priv->flags);
			set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);
			pr_debug("thread should stop\n");
			break;
		}

		if (kthread_should_park()) {
			/* if required, perform tunnel adjust before park */
			adjust_tunnels(t_intf_priv);

			pr_debug("parking producer thread\n");
			trace_fba_producer_state(0);
			kthread_parkme();
			trace_fba_producer_state(1);
			continue;
		}

		if (!test_bit(IAXXX_TFLG_ALLOW_TUNNELS,
				&t_intf_priv->flags)) {
			dev_info(t_intf_priv->dev,
				"Waiting until tunnels is allowed...\n");
			wait_event(t_intf_priv->producer_wq,
				kthread_should_stop() ||
				kthread_should_park() ||
				test_bit(IAXXX_TFLG_ALLOW_TUNNELS,
					&t_intf_priv->flags));

			/* Check for reasons stop, park or other */
			if (!test_bit(IAXXX_TFLG_ALLOW_TUNNELS,
					&t_intf_priv->flags))
				continue;

			set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);

			dev_info(t_intf_priv->dev, "Tunnels allowed!\n");
		}

		if (test_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags))
			adjust_tunnels(t_intf_priv);

		size = circ_get_free_buffer(circ, &buf);

		WARN_ONCE(size < 4, "No more buffer");

		if (size >= 4) {
			/* Get the data max size words */
			bytes = iaxxx_tunnel_read(priv,
					buf, size >> 2,
					tunnel_flags & (1<<TNL_CVQ)) << 2;

			pr_debug("%s: buf = %p sz = %d, bytes = %d, cnt = %d\n",
					__func__, buf, size,
					bytes, circ_cnt(circ) << 2);
			if (bytes < 0) {
				iaxxx_fw_crash(priv->dev,
					IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF);
				kthread_park(t_intf_priv->producer_thread);
				continue;
			}

			if (bytes > 0) {
				circ_fill_finish(circ, bytes);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
				/*
				 * To detect buffer wrap around,
				 * keep increasing receive size until
				 * it reaches to total buffer size
				 */

				if (t_intf_priv->debug_rsize
					< iaxxx_prod_buf_size)
					t_intf_priv->debug_rsize += bytes;
#endif

				/*
				 * Wakeup consumer only when client
				 * is registered.
				 * If only debug client (client_no==0)
				 * just keep matching tail with head
				 */
				if (t_intf_priv->client_no > 0)
					wake_up(&t_intf_priv->consumer_wq);
				else
					circ->tail = circ->head;

				pr_debug("%s: tnl producer wait for event\n",
							__func__);
			}

		}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
		/* Confirme if no channels are open */
		if (!tunnel_flags)
			complete(&t_intf_priv->termination_done);
#endif
		wait_event(t_intf_priv->producer_wq,
			atomic_read(&t_intf_priv->event_occurred) > 0 ||
			kthread_should_stop() ||
			kthread_should_park() ||
			test_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags));

		if (atomic_read(&t_intf_priv->event_occurred) > 0)
			atomic_dec(&t_intf_priv->event_occurred);
	}

	adjust_tunnels(t_intf_priv);

	return 0;
}

/*
 * Attach a client to tunnel stream
 */
static int tunneling_attach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	client->user_circ.buf = vmalloc(UBUFF_SIZE);
	if (!client->user_circ.buf)
		return -ENOMEM;

	client->user_circ.size = UBUFF_SIZE >> 2;
	circ_init(&client->user_circ);

	client->tunnel_data = tunnel_data;

	spin_lock(&tunnel_data->lock);
	list_add_tail_rcu(&client->node, &tunnel_data->list);
	tunnel_data->client_no++;
	spin_unlock(&tunnel_data->lock);

	return 0;
}

/*
 * Detach a client from tunnel stream
 */
static int tunneling_detach_client(struct iaxxx_tunnel_data *tunnel_data,
			struct iaxxx_tunnel_client *client)
{
	spin_lock(&tunnel_data->lock);
	list_del_rcu(&client->node);
	tunnel_data->client_no--;
	spin_unlock(&tunnel_data->lock);
	synchronize_rcu();

	vfree(client->user_circ.buf);

	return 0;
}

/*
 * Return tunned id for given src end
 */
static int tunnel_find_id(int src)
{
	int     i;

	for (i = 0; i < ARRAY_SIZE(iaxxx_tunnel_src); i++)
		if (iaxxx_tunnel_src[i].src == src)
			return i;
	return -EINVAL;
}

/*
 * Copy buffer to a client's buffer
 */
static void tunnel_copy_to_client(struct iaxxx_tunnel_client *client,
				u16 tid, int count)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;
	int size;

	if (!is_valid_tid(tid) || !test_bit(tid, &client->tid_flag))
		return;

	if ((circ_space(&client->user_circ) << 2) < count)
		/* drop the current packet */
		return;

	size = min(count, circ_cnt_to_end(circ) << 2);
	circ_in(&client->user_circ, circ->buf + (circ->tail << 2), size);
	if (size < count)
		circ_in(&client->user_circ, circ->buf, count - size);

	wake_up(&client->wq);
}

#ifdef CHECK_SEQUENCE
static void check_packet(struct iaxxx_tunnel_data *t_intf_priv,
			u16 tid, uint32_t seq_no)
{
	if (t_intf_priv->tunnel_seq_no[tid] == 0) {
		t_intf_priv->tunnel_seq_no[tid] = seq_no;
		return;
	}

	if ((seq_no - t_intf_priv->tunnel_seq_no[tid]) != 1) {
		pr_debug("Sequence number error id = %d old = %x, new = %x\n",
			tid,
			t_intf_priv->tunnel_seq_no[tid],
			seq_no);
		t_intf_priv->tunnel_seq_err[tid] +=
			seq_no - t_intf_priv->tunnel_seq_no[tid];
	}

	t_intf_priv->tunnel_seq_no[tid] = seq_no;
	t_intf_priv->tunnel_packet_no[tid]++;
	t_intf_priv->tunnel_total_packet_no++;
}
#endif

/*
 * Consume circular buffer and feed data to each client kfifo
 */
static int consumer_thread(void *arg)
{
	struct iaxxx_tunnel_data *t_intf_priv = arg;
	struct iaxxx_tunnel_client *client;
	u16 tid;
	int rc;
	int size;
	int sz_used;
	struct iaxxx_tunnel_header hdr;
	int hdr_size = sizeof(hdr);
	struct iaxxx_circ_buf *circ = &t_intf_priv->stream_circ;

	while (1) {
		if (kthread_should_stop())
			break;

		if (kthread_should_park()) {
			kthread_parkme();
			continue;
		}

		size = 0;
		sz_used = circ_cnt(circ) << 2;
		if (sz_used >= hdr_size) {
			circ_peek_data(circ, &hdr, hdr_size);

			size = parse_header(&hdr, &tid);
			if (size < 0) {
				/* Increase magic error */
				t_intf_priv->tunnel_magic_errcnt++;

				/* Packet header error */
				circ_peek_finish(circ, 4);
				continue;
			}
			if (size > 0)
				size += hdr_size;
		}

		if (size > 0 && sz_used >= size) {
			rcu_read_lock();
			/* Fill to each client's fifo */
			list_for_each_entry_rcu(client,
						&t_intf_priv->list, node)
				tunnel_copy_to_client(client, tid, size);
			rcu_read_unlock();
			circ_peek_finish(circ, size);

#ifdef CHECK_SEQUENCE
			if (t_intf_priv->client_no)
				check_packet(t_intf_priv, tid, hdr.seq_no);
#endif

			continue;
		}

		if (sz_used == 0) {
			wait_event(t_intf_priv->consumer_wq,
				(circ_cnt(circ) > 0) ||
				kthread_should_stop() ||
				kthread_should_park());
			continue;
		}

		/* We got some data but not enough */
		rc = wait_event_timeout(t_intf_priv->consumer_wq,
			((circ_cnt(circ) << 2) > sz_used) ||
			kthread_should_stop() ||
			kthread_should_park(),
			HZ);

		sz_used = circ_cnt(circ) << 2;
		if (!rc && sz_used < size) {
			pr_err("%s: Consume invalid packet in circ queue\n",
				__func__);
			circ_peek_finish(circ, sz_used);
		}
	}

	return 0;
}

/*
 * Request to setup a tunnel to producer
 */
static int tunnel_setup(struct iaxxx_tunnel_client *client, uint32_t src,
				uint32_t mode, uint32_t encode)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	int id;
	int rc = 0;

	id = tunnel_find_id(src);
	if (id < 0)
		return -EINVAL;

	if (!test_and_set_bit(id, &client->tid_flag))
		set_bit(IAXXX_TFLG_ADJ_CLIENT, &t_intf_priv->flags);

	pr_debug("%s: tid: %x src: %x client flag: %lx client flag: %lx\n",
		__func__, id, src, client->tid_flag, client->tid_flag);

	return rc;
}

/*
 * Terminate a tunnel for a client (Producer will terminate the tunnel)
 */
static int tunnel_term(struct iaxxx_tunnel_client *client, uint32_t src)
{
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	int id;
	int rc = 0;

	id = tunnel_find_id(src);
	if (id < 0)
		return -EINVAL;

	if (test_and_clear_bit(id, &client->tid_flag))
		set_bit(IAXXX_TFLG_ADJ_CLIENT, &t_intf_priv->flags);

	pr_debug("%s: tid: %x src: %x client flag: %lx client flag: %lx\n",
		__func__, id, src, client->tid_flag, client->tid_flag);

	return rc;
}

/*
 * Setup all sources in the list
 */
static int tunnel_setup_list(struct iaxxx_tunnel_client *client,
				const uint32_t *src, int num)
{
	int	i;
	int	rc = 0;
	uint32_t mode;
	uint32_t encode;

	for (i = 0; i < num; i++) {
		rc = tunnel_setup(client, src[i], mode, encode);
		if (rc) {
			pr_err("tunnel setup fail for src = %d\n", src[i]);
			break;
		}
	}

	if (i < num) {
		for (--i; i >= 0; i--)
			tunnel_term(client, src[i]);
	}

	return rc;
}

/*
 * Terminate all sources in the list
 */
static int tunnel_term_list(struct iaxxx_tunnel_client *client,
			const uint32_t *src, int num)
{
	int	i;
	int	rc = 0;

	for (i = 0; i < num; i++)
		rc |= tunnel_term(client, src[i]);

	if (rc)
		return -EIO;

	return 0;
}

static ssize_t tunneling_read(struct file *filp, char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct iaxxx_tunnel_client *client = filp->private_data;
	struct iaxxx_circ_buf *user_circ = &client->user_circ;
	unsigned int copied;
	int ret;

	/* Return error if the user buffer is already less than min size */
	if (count < IAXXX_MIN_TUNNEL_SIZE)
		return -EFAULT;

	if (!circ_cnt(user_circ)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(client->wq,
					circ_cnt(user_circ));
		if (ret == -ERESTARTSYS)
			return -ERESTARTSYS;
	}
	ret = circ_to_user(user_circ, buf, count, &copied);

	return ret ? ret : copied;
}

static int tunneling_open(struct inode *inode, struct file *filp)
{
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_client *client;
	int rc;

	pr_debug("tunneling device open called");

	if ((inode)->i_cdev == NULL) {
		pr_err("Failed to retrieve cdev from inode");
		return -ENODEV;
	}

	t_intf_priv = container_of((inode)->i_cdev,
				struct iaxxx_tunnel_data, tunnel_cdev.cdev);
	if (t_intf_priv == NULL) {
		pr_err("Unable to fetch tunnel private data");
		return -ENODEV;
	}

	if (!pm_runtime_enabled(t_intf_priv->dev) ||
	    !pm_runtime_active(t_intf_priv->dev))
		return -EIO;

	client = kzalloc(sizeof(struct iaxxx_tunnel_client),
				GFP_KERNEL | __GFP_NOWARN);
	if (!client)
		return -ENOMEM;

	rc = tunneling_attach_client(t_intf_priv, client);
	if (rc) {
		kfree(client);
		return rc;
	}

	init_waitqueue_head(&client->wq);
	snprintf(client->ws_name, sizeof(client->ws_name),
		 "iaxxx CS2 %s", current->comm);
	wakeup_source_init(&client->ws, client->ws_name);

	filp->private_data = client;

	/*
	 * When char device /dev/tunnel0 is open and event IAXXX_EV_ROUTE_ACTIVE
	 * has not appeared so far, must be ensured that the system will work.
	 */
	set_bit(IAXXX_TFLG_ALLOW_TUNNELS, &t_intf_priv->flags);

	return 0;
}

static long tunnel_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct iaxxx_tunnel_client * const client =
			(struct iaxxx_tunnel_client *)filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct tunlMsg msg;
	int ret = 0;

	if (!priv) {
		pr_err("Unable to fetch tunnel private data\n");
		return -EINVAL;
	}
	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}

	if (!pm_runtime_enabled(t_intf_priv->dev) ||
	    !pm_runtime_active(t_intf_priv->dev)) {
		dev_err(priv->dev, "FW state not valid %s()\n", __func__);
		return -EIO;
	}

	if (arg != 0 && _IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE)
		&& _IOC_SIZE(cmd) == sizeof(struct tunlMsg)) {
		if (copy_from_user(&msg, (void __user *)arg,
					sizeof(struct tunlMsg))) {
			pr_err("parameter copy from user failed\n");
			return -EFAULT;
		}

		pr_debug("cmd: %x, TunlEP: %x TunlSrc: %x",
			cmd, (uint32_t)msg.tunlEP, (uint32_t)msg.tunlSrc);
	}

	switch (cmd) {
	case TUNNEL_SETUP:
		ret = tunnel_setup(client, msg.tunlSrc, msg.tunlMode,
						msg.tunlEncode);
		if (ret) {
			pr_err("Unable to setup tunnel");
			return	-EINVAL;
		}
		break;
	case TUNNEL_TERMINATE:
		ret = tunnel_term(client, msg.tunlSrc);
		if (ret) {
			pr_err("Unable to terminate tunnel");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_SUBSCRIBE_META:
		ret = tunnel_setup_list(client, tunnel_src_meta,
					ARRAY_SIZE(tunnel_src_meta));
		if (ret) {
			pr_err("Unable to setup meta tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_META:
		ret = tunnel_term_list(client, tunnel_src_meta,
					ARRAY_SIZE(tunnel_src_meta));
		if (ret) {
			pr_err("Unable to terminate meta tunnels");
			ret = -EINVAL;
		}
		break;
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	case TUNNEL_SUBSCRIBE_ALL:
		ret = tunnel_setup_list(client, tunnel_src_all,
					ARRAY_SIZE(tunnel_src_all));
		if (ret) {
			pr_err("Unable to setup all tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_ALL:
		ret = tunnel_term_list(client, tunnel_src_all,
					ARRAY_SIZE(tunnel_src_all));
		if (ret) {
			pr_err("Unable to terminate all tunnels");
			ret = -EINVAL;
		}
		break;
#endif
	case TUNNEL_SUBSCRIBE_CVQ:
		ret = tunnel_setup_list(client, tunnel_src_cvq,
					ARRAY_SIZE(tunnel_src_cvq));
		if (ret) {
			pr_err("Unable to setup cvq tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_CVQ:
		ret = tunnel_term_list(client, tunnel_src_cvq,
					ARRAY_SIZE(tunnel_src_cvq));
		if (ret) {
			pr_err("Unable to terminate cvq tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_SUBSCRIBE_META_DOA:
		ret = tunnel_setup(client, TNL_SRC_DOA,
				TNL_ASYNC_MODE, TNL_ENC_AFLOAT);
		if (ret) {
			pr_err("Unable to setup doa tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_META_DOA:
		ret = tunnel_term(client, TNL_SRC_DOA);
		if (ret) {
			pr_err("Unable to setup doa tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_SUBSCRIBE_META_VQ:
		ret = tunnel_setup(client, TNL_SRC_VQ_CONFIDENCE,
				TNL_ASYNC_MODE, TNL_ENC_AFLOAT);
		if (ret) {
			pr_err("Unable to setup vq tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_META_VQ:
		ret = tunnel_term(client, TNL_SRC_VQ_CONFIDENCE);
		if (ret) {
			pr_err("Unable to setup vq tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_SUBSCRIBE_CS2:
		ret = tunnel_setup(client, TNL_SRC_CS_OUT2,
				TNL_ASYNC_MODE, TNL_ENC_Q15);
		if (ret) {
			pr_err("Unable to setup CS2 tunnels");
			ret = -EINVAL;
		} else {
			/*
			 * Thread can't freeze until ioctl returns, and suspend
			 * will be blocked after this call
			 */
			__pm_stay_awake(&client->ws);
		}
		break;
	case TUNNEL_UNSUBSCRIBE_CS2:
		ret = tunnel_term(client, TNL_SRC_CS_OUT2);
		if (ret) {
			pr_err("Unable to terminate CS2 tunnels");
			ret = -EINVAL;
		} else {
			__pm_relax(&client->ws);
		}
		break;
	case TUNNEL_SUBSCRIBE_AEC_REF1:
		ret = tunnel_setup(client, TNL_SRC_AEC_REF1_Q15,
				TNL_ASYNC_MODE, TNL_ENC_Q15);
		if (ret) {
			pr_err("Unable to setup AEC ref 1 tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_AEC_REF1:
		ret = tunnel_term(client, TNL_SRC_AEC_REF1_Q15);
		if (ret) {
			pr_err("Unable to terminate AEC ref 1 tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_SUBSCRIBE_AEC_REF2:
		ret = tunnel_setup(client, TNL_SRC_AEC_REF2_Q15,
				TNL_ASYNC_MODE, TNL_ENC_Q15);
		if (ret) {
			pr_err("Unable to setup AEC ref 2 tunnels");
			ret = -EINVAL;
		}
		break;
	case TUNNEL_UNSUBSCRIBE_AEC_REF2:
		ret = tunnel_term(client, TNL_SRC_AEC_REF2_Q15);
		if (ret) {
			pr_err("Unable to terminate AEC ref 2 tunnels");
			ret = -EINVAL;
		}
		break;
	default:
		pr_err("Invalid ioctl command received %x", cmd);
		ret = -EINVAL;
	}

	/* Wakeup producer thread only one time if it is required */
	adjust_tunnels_do(t_intf_priv, client);

	return ret;
}

#ifdef CONFIG_COMPAT
static long tunnel_compat_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	return tunnel_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int tunnel_release(struct inode *inode, struct file *filp)
{
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_tunnel_client *client;

	pr_debug("tunneling device release called");

	if (inode == NULL) {
		pr_err("invalid inode pointer");
		goto error;
	}

	if ((inode)->i_cdev == NULL) {
		pr_err("invalid cdev pointer in inode");
		goto error;
	}

	t_intf_priv = container_of((inode)->i_cdev, struct iaxxx_tunnel_data,
					tunnel_cdev.cdev);
	if (t_intf_priv == NULL) {
		pr_err("unable to fetch tunnel private data");
		goto error;
	}

	client = filp->private_data;
	if (client) {
		tunneling_detach_client(t_intf_priv, client);
		set_bit(IAXXX_TFLG_ADJ_THREAD, &t_intf_priv->flags);
		adjust_tunnels_do(t_intf_priv, NULL);
	}

	wakeup_source_trash(&client->ws);

	/* ignore threadfn return value */
	pr_debug("stopping stream kthread");


	kfree(client);
	filp->private_data = NULL;

error:
	return 0;
}

static unsigned int tunnel_poll(struct file *filep,
	struct poll_table_struct *wait)
{
	struct iaxxx_tunnel_client *const client = filep->private_data;
	unsigned int mask = 0;

	poll_wait(filep, &client->wq, wait);
	if (circ_cnt(&client->user_circ))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations tunneling_fops = {
	.owner = THIS_MODULE,
	.open = tunneling_open,
	.read = tunneling_read,
	.unlocked_ioctl	= tunnel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= tunnel_compat_ioctl,
#endif
	.release = tunnel_release,
	.poll = tunnel_poll,
};

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
static int iaxxx_tunnel_cnt_clients(struct iaxxx_tunnel_data *tpriv, int tid)
{
	struct iaxxx_tunnel_client *client;
	unsigned long count = 0;

	spin_lock(&tpriv->lock);
	list_for_each_entry_rcu(client, &tpriv->list, node)
		if (test_bit(tid, &client->tid_flag))
			count++;
	spin_unlock(&tpriv->lock);

	return count;
}

/*
 * Show tunnel status, enable, num of clients etc.
 */
static ssize_t iaxxx_tunnel_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	unsigned long flags = tunnel_flags;
	int id = 0;
	int index;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	scnprintf(buf, PAGE_SIZE,
		"Tunnel ID\tSRC\t\t\tEnable\tMode\tEncode\tDebug\tClients\tNo Packets\tError Packets\n");
	for (id = 0; id < ARRAY_SIZE(iaxxx_tunnel_src); id++) {
		int enable = !!(flags & (1 << id));
		index = strnlen(buf, PAGE_SIZE);
		scnprintf(&buf[index], PAGE_SIZE - index,
			"    %02d    \t%-10s(%04x)\t%d\t%s\t%s\t%s\t%d\t%lu\t\t%d\n",
			id,
			iaxxx_tunnel_src[id].name,
			iaxxx_tunnel_src[id].src & 0xffff,
			enable,
			enable ? ((iaxxx_tunnel_src[id].mode == TNL_SYNC_MODE) ?
				"SYNC" : "ASYNC") : "-",
			enable ? ((iaxxx_tunnel_src[id].enc == TNL_ENC_AFLOAT) ?
				"AFLOAT" : "Q15") : "-",
			t_intf_priv->debug_client.tid_flag & (1 << id) ?
				"Yes" :
				"No ",
			iaxxx_tunnel_cnt_clients(t_intf_priv, id),
			t_intf_priv->tunnel_packet_no[id],
			t_intf_priv->tunnel_seq_err[id]);
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nTotal Packets: %lu\n",
		t_intf_priv->tunnel_total_packet_no);

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
		"\nProd circ   buf = %pK, size = %d, head = %d, tail = %d\n",
		t_intf_priv->stream_circ.buf,
		t_intf_priv->stream_circ.size << 2,
		t_intf_priv->stream_circ.head,
		t_intf_priv->stream_circ.tail);

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t iaxxx_tunnel_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	if (tunnel_flags)
		return -EPERM;

	/* Clear packet history */
	memset(t_intf_priv->tunnel_packet_no, 0,
		sizeof(t_intf_priv->tunnel_packet_no));
	memset(t_intf_priv->tunnel_seq_err, 0,
		sizeof(t_intf_priv->tunnel_seq_err));
	t_intf_priv->tunnel_total_packet_no = 0;

	return count;
}

static DEVICE_ATTR(tunnel_status, S_IRUSR | S_IWUSR,
		iaxxx_tunnel_status_show, iaxxx_tunnel_status_store);

/*
 * Get the information on producer circular queue
 */
static ssize_t iaxxx_tunnel_circ_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	struct iaxxx_circ_buf *circ;
	int index;
	int cnt;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	circ = &t_intf_priv->stream_circ;

	cnt = circ_cnt_to_end(circ) << 2;

	scnprintf(buf, PAGE_SIZE,
		"Circular: head %d, tail %d, cnt %d, space %d\n",
		circ->head, circ->tail, cnt, circ_space(circ) << 2);

	index = strnlen(buf, PAGE_SIZE);

	return hex_dump_to_buffer(circ->buf + (circ->tail << 2), cnt, 16, 1,
			&buf[index], PAGE_SIZE - index, false) + index;
}
static DEVICE_ATTR(tunnel_circ, S_IRUSR, iaxxx_tunnel_circ_show, NULL);

/*
 * Show packet header magic error
 */
static ssize_t iaxxx_tunnel_header_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE,
		"Packet Header Error Count = %ld\n",
		t_intf_priv->tunnel_magic_errcnt);
}
static DEVICE_ATTR(tunnel_header_errcnt, S_IRUSR,
			iaxxx_tunnel_header_errcnt_show, NULL);

/*
 * Show sequence number error
 */
static ssize_t iaxxx_tunnel_seqno_errcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	struct iaxxx_tunnel_data *t_intf_priv;
	int index;
	int id;
	uint64_t sum = 0;

	if (!priv)
		return -EINVAL;

	t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv)
		return -EINVAL;

	scnprintf(buf, PAGE_SIZE,
		"Tunnel ID\tSeqno Error Count\n");
	for (id = 0; id < ARRAY_SIZE(iaxxx_tunnel_src); id++) {
		index = strnlen(buf, PAGE_SIZE);
		scnprintf(&buf[index], PAGE_SIZE - index,
			"    %02d    \t%d\n",
			id,
			t_intf_priv->tunnel_seq_err[id]);
		sum += t_intf_priv->tunnel_seq_err[id];
	}

	index = strnlen(buf, PAGE_SIZE);
	scnprintf(&buf[index], PAGE_SIZE - index,
			"Total error: %llu\n", sum);

	return strnlen(buf, PAGE_SIZE);
}
static DEVICE_ATTR(tunnel_seqno_errcnt, S_IRUSR,
			iaxxx_tunnel_seqno_errcnt_show, NULL);

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_tunnel_status.attr,
	&dev_attr_tunnel_circ.attr,
	&dev_attr_tunnel_header_errcnt.attr,
	&dev_attr_tunnel_seqno_errcnt.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
};


static int iaxxx_debug_dump_open(struct inode *inode, struct file *file)
{
	struct iaxxx_tunnel_data *t_intf_priv = inode->i_private;
	struct device *dev = t_intf_priv->dev;
	int flag = 0;

	/* Only one instance is supported */
	if (t_intf_priv->debug_enabled != DEBUG_FLAG_ENABLED ||
		!t_intf_priv->iaxxx_debug_initialized ||
		(flag = test_and_set_bit(0, &t_intf_priv->debug_flag))) {
		pr_err("dump open failed, en = %d, init = %d, flag = %d\n",
			t_intf_priv->debug_enabled,
			t_intf_priv->iaxxx_debug_initialized,
			flag);
		return -EPERM;
	}

	file->private_data = t_intf_priv;

	/* Stop the producer thread */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		kthread_park(t_intf_priv->producer_thread);

	/* Copy circular queue structure for retrieving producer queue */
	t_intf_priv->debug_stream_circ = t_intf_priv->stream_circ;

	if (t_intf_priv->debug_rsize < iaxxx_prod_buf_size)
		/* Buffer is not overwrapped */
		t_intf_priv->debug_stream_circ.tail =
			t_intf_priv->debug_start_idx;
	else
		/* Will retrieve all queues starting from head+1 */
		t_intf_priv->debug_stream_circ.tail =
			(t_intf_priv->debug_stream_circ.head + 1) %
				t_intf_priv->debug_stream_circ.size;

	return nonseekable_open(inode, file);
}

static int iaxxx_debug_dump_release(struct inode *inode, struct file *file)
{
	struct iaxxx_tunnel_data *t_intf_priv = inode->i_private;
	struct device *dev = t_intf_priv->dev;

	/* clear debug receive size */
	t_intf_priv->debug_rsize = 0;

	/* set the start index */
	t_intf_priv->debug_start_idx = t_intf_priv->stream_circ.head;

	/*
	 * Check IAXXX_TFLG_RUN_THREAD flag and unpark the producer
	 * only it's set. If the flag is not set, that means the thread
	 * is stopped so we should not unpark it.
	 */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		kthread_unpark(t_intf_priv->producer_thread);

	clear_bit(0, &t_intf_priv->debug_flag);

	return 0;
}

static ssize_t iaxxx_debug_dump_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct iaxxx_tunnel_data *t_intf_priv = file->private_data;
	struct iaxxx_circ_buf *circ = &t_intf_priv->debug_stream_circ;
	struct iaxxx_tunnel_header header;
	int hdr_size = sizeof(struct iaxxx_tunnel_header);
	int size;
	u16 tid;
	unsigned int sum = 0;
	int cnt = min_t(int, circ_cnt(circ) << 2, count);

	if (cnt == 0)
		return 0;

	if (count > cnt) {
		/*
		 * If user buffer is big enough, copy all
		 * since the circular buffer has already
		 * packet aligned
		 */
		if (circ_copy_to_user(circ, buf, cnt))
			return -EFAULT;

		*ppos += cnt;
		return cnt;
	}

	while (1) {
		size = circ_peek_data(circ, &header, hdr_size);

		if (size < hdr_size)
			break;

		size = parse_header(&header, &tid);
		if (size < 0) {
			circ_peek_finish(circ, 4);
			continue;
		}

		size += hdr_size;
		if ((circ_cnt(circ) << 2) < size)
			break;

		if (count < (sum + size))
			break;

		if (circ_copy_to_user(circ, buf + sum, size))
			break;

		sum += size;
	}

	*ppos += sum;

	return sum;
}

static int iaxxx_debug_enable_get(void *data, u64 *val)
{
	struct iaxxx_tunnel_data *t_intf_priv = data;

	*val = t_intf_priv->debug_enabled;

	return 0;
}


static int iaxxx_debug_do_enable_set(struct iaxxx_priv *priv, bool flag)
{
	int ret = 0;
	struct iaxxx_tunnel_data *t_intf_priv = priv->tunnel_data;

	if (!t_intf_priv->iaxxx_debug_initialized) {
		/* If tunneling is not ready, just mark and return */
		if (flag)
			t_intf_priv->debug_enabled = DEBUG_FLAG_ENABLED_DEFER;
		else
			t_intf_priv->debug_enabled = DEBUG_FLAG_DISABLED;
		trace_fba_debug_dump_state(t_intf_priv->debug_enabled);
		return 0;
	}

	if (flag == t_intf_priv->debug_enabled)
		/* Skip if in defer enable mode or same flag */
		return 0;

	/* Return duplicate enable and disable */
	if ((flag && t_intf_priv->debug_client.tid_flag) ||
		(!flag && !t_intf_priv->debug_client.tid_flag))
		return 0;

	if (flag) {
		/* clear debug receive size */
		t_intf_priv->debug_rsize = 0;

		/* set the start index */
		t_intf_priv->debug_start_idx = t_intf_priv->stream_circ.head;

		ret = tunnel_setup_list(&t_intf_priv->debug_client,
			tunnel_src_debug, tunnel_src_debug_size);
	} else {
		reinit_completion(&t_intf_priv->termination_done);

		ret = tunnel_term_list(&t_intf_priv->debug_client,
			tunnel_src_debug, tunnel_src_debug_size);
		if (!ret) {
			/*
			 * Make sure producer wakes up and
			 * handle tunnel_termination_done
			 */
			wake_up(&t_intf_priv->producer_wq);

			if (wait_for_completion_interruptible_timeout(
				&t_intf_priv->termination_done,
				msecs_to_jiffies(
					IAXXX_TUNNEL_TERMINATE_TIMEOUT) <= 0)) {
				dev_err(priv->dev,
					"tunnel term failed or timeout\n");
			}
		}
	}

	if (ret) {
		t_intf_priv->debug_enabled = 0;
		trace_fba_debug_dump_state(t_intf_priv->debug_enabled);
		pr_err("Unable to setup all tunnels");
		ret = -EINVAL;
	} else {
		t_intf_priv->debug_enabled = flag;
		trace_fba_debug_dump_state(t_intf_priv->debug_enabled);
	}

	/* Wakeup producer thread only one time if it is required */
	adjust_tunnels_do(t_intf_priv, &t_intf_priv->debug_client);

	return ret;
}

static int iaxxx_debug_enable_set(void *data, u64 val)
{
	struct iaxxx_tunnel_data *t_intf_priv = data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	bool flag = (val == (u64)1);
	int ret;

	mutex_lock(&t_intf_priv->debug_lock);
	ret = iaxxx_debug_do_enable_set(priv, flag);
	mutex_unlock(&t_intf_priv->debug_lock);

	return ret;
}

/**
 * iaxxx_debug_enable_work - work thread for enabling debug channel
 *
 */
static void iaxxx_debug_enable(struct iaxxx_tunnel_data *t_intf_priv)
{
	pr_debug("iaxxx debug tunnel kicked\n");
	mutex_lock(&t_intf_priv->debug_lock);
	t_intf_priv->iaxxx_debug_initialized = true;
	if (t_intf_priv->debug_enabled) {
		struct iaxxx_priv *priv =
				(struct iaxxx_priv *)t_intf_priv->priv;
		if (iaxxx_debug_do_enable_set(priv, true))
			t_intf_priv->iaxxx_debug_initialized = false;
	}
	mutex_unlock(&t_intf_priv->debug_lock);
}

/*
 * debugfs for debug audio data support
 */
static const struct file_operations iaxxx_debug_dump_fops = {
	.open = iaxxx_debug_dump_open,
	.read = iaxxx_debug_dump_read,
	.release = iaxxx_debug_dump_release,
};

DEFINE_SIMPLE_ATTRIBUTE(iaxxx_debug_enable_fops, iaxxx_debug_enable_get,
			iaxxx_debug_enable_set, "%llu\n");
#endif /* #ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP */

/*
 * Init early stage before firmware download
 */
int iaxxx_tunnel_dev_init_early(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	struct dentry *root;
#endif

	t_intf_priv = devm_kzalloc(priv->dev, sizeof(*t_intf_priv), GFP_KERNEL);
	if (!t_intf_priv)
		return -ENOMEM;

	priv->tunnel_data = t_intf_priv;
	t_intf_priv->priv = priv;

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	mutex_init(&t_intf_priv->debug_lock);
	t_intf_priv->debug_saved = -1;
	root = iaxxx_dfs_get_root();
	if (root) {
		t_intf_priv->debug_client.tunnel_data = t_intf_priv;
		debugfs_create_file("debug_dump", S_IRUSR, root,
				t_intf_priv, &iaxxx_debug_dump_fops);
		debugfs_create_file("debug_enable", S_IRUSR | S_IWUSR, root,
				t_intf_priv, &iaxxx_debug_enable_fops);
	}
#endif
	clear_bit(IAXXX_TFLG_DRAIN_BIT, &t_intf_priv->flags);
	return 0;
}

static void iaxxx_tunnel_stop(struct iaxxx_tunnel_data *priv)
{
	struct iaxxx_circ_buf *circ = &priv->stream_circ;
	int i;

	if (!test_and_clear_bit(IAXXX_TFLG_RUN_THREAD, &priv->flags))
		return;

	kthread_park(priv->producer_thread);
	kthread_park(priv->consumer_thread);

	circ_init(circ);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	/* clear debug receive size */
	priv->debug_rsize = 0;
	/* set the start index */
	priv->debug_start_idx = 0;
#endif

	atomic_set(&priv->event_occurred, 0);
	for (i = 0; i < TNLMAX; i++) {
		priv->tunnel_seq_err[i] = 0;
		priv->tunnel_seq_no[i] = 0;
		priv->tunnel_packet_no[i] = 0;
	}
	priv->tunnel_total_packet_no = 0;
	priv->tunnel_magic_errcnt = 0;
}

static int iaxxx_tunnel_start(struct iaxxx_tunnel_data *priv)
{
	int err = 0;

	if (test_and_set_bit(IAXXX_TFLG_RUN_THREAD, &priv->flags))
		return -EEXIST;

	/*
	 * When we start tunnels, the IAXXX_TFLG_DISABLE_ALL
	 * should have been cleared. Check if tunnel_flags
	 * are still set when exiting and make sure the tunnels
	 * are not cleared due to IAXXX_TFLG_DISABLE_ALL
	 */
	clear_bit(IAXXX_TFLG_DISABLE_ALL, &priv->flags);

	set_bit(IAXXX_TFLG_ADJ_THREAD, &priv->flags);

	kthread_unpark(priv->producer_thread);
	kthread_unpark(priv->consumer_thread);

	return err;
}

static int iaxxx_notifier_cb(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct iaxxx_tunnel_data *priv = container_of(nb,
			struct iaxxx_tunnel_data, crash_notifier_fbp);
	int ret = 0;

	switch (val) {
	case IAXXX_EV_STARTUP:
		pm_runtime_get(priv->dev);
		break;

	case IAXXX_EV_RECOVERY:
		break;

	case IAXXX_EV_CRASH:
		set_bit(IAXXX_TFLG_FW_CRASH, &priv->flags);
		break;

	case IAXXX_EV_ROUTE_ACTIVE:
		set_bit(IAXXX_TFLG_ALLOW_TUNNELS, &priv->flags);
		wake_up_process(priv->producer_thread);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
		if (!priv->iaxxx_debug_initialized)
			iaxxx_debug_enable(priv);
#endif
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int iaxxx_tunnel_suspend(struct device *dev)
{
	struct iaxxx_priv *priv = dev_get_drvdata(dev);
	struct iaxxx_tunnel_data *tpriv = priv->tunnel_data;

	if (!pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		/*
		 * This indicate point where lower layer has problem and
		 * all SPI transactions and works with lower layer must
		 * be blocked because is forbidden.
		 */
		dev_info(dev, "Forced suspend requested by HW!");
	}

	if (!test_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags)) {
		set_bit(IAXXX_TFLG_DISABLE_ALL, &tpriv->flags);
		set_bit(IAXXX_TFLG_ADJ_THREAD, &tpriv->flags);
	}

	iaxxx_tunnel_stop(tpriv);

	return 0;
}

static int iaxxx_tunnel_resume(struct device *dev)
{
	struct iaxxx_priv *priv = dev_get_drvdata(dev);
	struct iaxxx_tunnel_data *tpriv = priv->tunnel_data;

	if (!pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		/*
		 * This indicate point where lower layer is already
		 * recovered and ready to continue work.
		 * Could be resumed all pending SPI transactions.
		 */
		dev_info(dev, "Forced resume requested by HW!");
	}

	if (test_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags)) {
		clear_bit(IAXXX_TFLG_ALLOW_TUNNELS, &tpriv->flags);
		tunnel_flags = 0;
	}

	iaxxx_tunnel_start(tpriv);

	clear_bit(IAXXX_TFLG_FW_CRASH, &tpriv->flags);

	return 0;
}

static int iaxxx_tunnel_dev_suspend(struct device *dev)
{
	/* Don't suspend if PM is disabled or not active */
	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return 0;

	return iaxxx_tunnel_suspend(dev);
}

static int iaxxx_tunnel_dev_resume(struct device *dev)
{
	/* Don't suspend if PM is disabled or not active */
	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return 0;

	return iaxxx_tunnel_resume(dev);
}

static enum alarmtimer_restart
iaxxx_tunnel_tmr_handler(struct alarm *a, ktime_t t)
{
	struct iaxxx_tunnel_data *t_intf_priv =
		container_of(a, struct iaxxx_tunnel_data, tmr);

	atomic_inc(&t_intf_priv->event_occurred);
	wake_up(&t_intf_priv->producer_wq);

	if (!tunnel_flags) {
		trace_fba_alarm_state(0);
		return ALARMTIMER_NORESTART;
	}

	alarm_start(a, ktime_add_ms(t, IAXXX_TMR_INTERVAL_MS));

	return ALARMTIMER_RESTART;
}

/*
 * Init remaining stuffs
 */
static int iaxxx_tunnel_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	struct iaxxx_priv *priv = NULL;
	int err;

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("No device data found\n");
		return -EINVAL;
	}

	pr_debug("%s: initializing tunneling", __func__);

	if (of_property_read_bool(dev->parent->of_node, "adnc,stereo-aec")) {
		tunnel_src_debug = tunnel_src_debug_stereo;
		tunnel_src_debug_size = ARRAY_SIZE(tunnel_src_debug_stereo);
	} else {
		tunnel_src_debug = tunnel_src_debug_mono;
		tunnel_src_debug_size = ARRAY_SIZE(tunnel_src_debug_mono);
	}

	dev_set_drvdata(dev, priv);
	t_intf_priv = priv->tunnel_data;
	t_intf_priv->dev = &pdev->dev;

	if (iaxxx_prod_buf) {
		/* If reserved memory exists, use it */
		t_intf_priv->stream_circ.buf = phys_to_virt(iaxxx_prod_buf);
		t_intf_priv->stream_circ.size =
			iaxxx_prod_buf_size >> 2;
		pr_debug("%s: carvout %p, %zd\n",
			__func__, t_intf_priv->stream_circ.buf,
			iaxxx_prod_buf_size);
	} else {
		/* If no reserved, allocate default memory */
		t_intf_priv->stream_circ.buf = devm_kmalloc(priv->dev,
						PBUFF_SIZE, GFP_KERNEL);
		t_intf_priv->stream_circ.size = PBUFF_SIZE >> 2;
		iaxxx_prod_buf_size = PBUFF_SIZE;
	}

	if (!t_intf_priv->stream_circ.buf) {
		err = -ENOMEM;
		goto error_circ_buf;
	}

	/* Initialize client structure */
	INIT_LIST_HEAD(&t_intf_priv->list);
	spin_lock_init(&t_intf_priv->lock);

	err = iaxxx_cdev_create(&t_intf_priv->tunnel_cdev, dev,
		&tunneling_fops, t_intf_priv, IAXXX_CDEV_TUNNEL);
	if (err) {
		pr_err("error in creating the char device");
		err = -EIO;
		goto error_cdev;
	}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	init_completion(&t_intf_priv->termination_done);
#endif

	/* Create producer thread */
	init_waitqueue_head(&t_intf_priv->producer_wq);
	t_intf_priv->producer_thread = kthread_run(producer_thread,
			t_intf_priv, "iaxxx tunnel producer thread");
	if (IS_ERR(t_intf_priv->producer_thread)) {
		pr_err("Cannot create producer thread\n");
		err = PTR_ERR(t_intf_priv->producer_thread);
		goto error_producer_thread;
	}

	/* Create consumer thread */
	init_waitqueue_head(&t_intf_priv->consumer_wq);
	t_intf_priv->consumer_thread = kthread_run(consumer_thread,
			t_intf_priv, "iaxxx tunnel consumer thread");
	if (IS_ERR(t_intf_priv->consumer_thread)) {
		pr_err("Cannot create consumer thread\n");
		err = PTR_ERR(t_intf_priv->consumer_thread);
		goto error_consumer_thread;
	}

	kthread_park(t_intf_priv->producer_thread);
	kthread_park(t_intf_priv->consumer_thread);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	t_intf_priv->debug_enabled = DEBUG_FLAG_ENABLED_DEFER;
	if (sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group))
		pr_err("Cannot create tunnel sysfs\n");
#endif

	pr_debug("streaming cdev initialized.\n");

	t_intf_priv->crash_notifier_fbp.notifier_call = iaxxx_notifier_cb;
	err = iaxxx_fw_notifier_register
		(priv->dev, &t_intf_priv->crash_notifier_fbp);
	if (err) {
		dev_err(dev, "%s: failed to register for fw notifier\n",
				__func__);
		goto error_consumer_thread;
	}

	alarm_init(&t_intf_priv->tmr, ALARM_BOOTTIME, iaxxx_tunnel_tmr_handler);

	pm_runtime_enable(dev);

	return 0;
error_consumer_thread:
	kthread_stop(t_intf_priv->producer_thread);
error_producer_thread:
error_cdev:
error_circ_buf:
	if (err && t_intf_priv)
		kfree(t_intf_priv);

	return err;
}

static int iaxxx_tunnel_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = NULL;
	struct iaxxx_tunnel_data *t_intf_priv = NULL;
	int ret = 0;

	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;

	ret = iaxxx_fw_notifier_unregister
		(priv->dev, &t_intf_priv->crash_notifier_fbp);
	if (ret)
		dev_err(dev, "%s: failed to unregister for fw notifier\n",
				__func__);
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	sysfs_remove_group(&priv->dev->kobj, &iaxxx_attr_group);
#endif
	kthread_stop(t_intf_priv->producer_thread);
	kthread_stop(t_intf_priv->consumer_thread);
	kfree(t_intf_priv);
	return 0;
}

int iaxxx_tunnel_signal_event(struct iaxxx_priv *priv)
{
	return 0;
}
EXPORT_SYMBOL(iaxxx_tunnel_signal_event);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
static int __init iaxxx_reserve_audio_buffer(char *p)
{
	char *old_p = p;
	unsigned long start, size;

	if (!p)
		return 0;

	size = memparse(p, &p);

	/* Check if value and power of 2 */
	if (p == old_p || !is_power_of_2(size))
		return 0;

	if (*p != '$')
		return 0;

	start = memparse(p + 1, &p);

	if (!iaxxx_prod_buf &&
		!memblock_reserve(start, size)) {
		iaxxx_prod_buf = start;
		iaxxx_prod_buf_size = size;
	}

	return 0;
}
early_param("audio_buffer", iaxxx_reserve_audio_buffer);
#endif

static const struct dev_pm_ops iaxxx_tunnel_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_tunnel_suspend, iaxxx_tunnel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_tunnel_dev_suspend,
					iaxxx_tunnel_dev_resume)
};

static const struct of_device_id iaxxx_tunnel_dt_match[] = {
	{.compatible = "knowles,iaxxx-tunnel-celldrv"},
	{}
};

static struct platform_driver iaxxx_tunnel_driver = {
	.probe  = iaxxx_tunnel_dev_probe,
	.remove = iaxxx_tunnel_dev_remove,
	.driver = {
		.name = "iaxxx-tunnel-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_tunnel_dt_match,
		.pm = &iaxxx_tunnel_pm_ops,
	},
};

static int __init iaxxx_tunnel_init(void)
{
	int ret;

	ret = platform_driver_register(&iaxxx_tunnel_driver);
	if (ret)
		pr_err("Failed to register tunnel platform driver");
	return ret;
}

static void __exit iaxxx_tunnel_exit(void)
{
	platform_driver_unregister(&iaxxx_tunnel_driver);
}

module_init(iaxxx_tunnel_init);
module_exit(iaxxx_tunnel_exit);
