/*
 * iaxxx-evnt-mgr.c -- IAxxx Event Manager Service
 *
 * Copyright 2016 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#define pr_fmt(fmt) "iaxxx : %s:%d, " fmt, __func__, __LINE__

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include "iaxxx.h"
#include "iaxxx-tunnel-priv.h"
#include <trace/events/fb_audio.h>

struct iaxxx_event_subscription {
	u16 event_dst;
	void *userdata;
	iaxxx_evnt_mgr_callback event_handler;
};

static unsigned long vt_count;

static void iaxxx_add_event(struct iaxxx_priv *priv,
		const struct iaxxx_event *evt)
{
	int w_index;

	priv->event_queue->w_index++;
	w_index = priv->event_queue->w_index;
	if (w_index == IAXXX_MAX_EVENTS) {
		pr_info("%s Event Index reach last\n", __func__);
		w_index = 0;
		priv->event_queue->w_index = w_index;
	}
	if (priv->event_queue->w_index == priv->event_queue->r_index) {
		pr_info("%s Event buffer is full\n", __func__);
		priv->event_queue->r_index++;
	}
	priv->event_queue->event_info[w_index].event_id = evt->event_id;
	priv->event_queue->event_info[w_index].data = evt->src_opaque;
	if (priv->event_queue->r_index == IAXXX_MAX_EVENTS)
		priv->event_queue->r_index = 0;
	pr_debug("%s written index %d\n", __func__, priv->event_queue->w_index);
}

/*
 * uevent format:
 *   "ACTION=IAXXX_VQ_EVENT\0
 *    EVENT_ID=1\0
 *    EVENT_DATA=12345678\0"
 */
int iaxxx_event_handler(struct iaxxx_priv *priv, struct iaxxx_event *evt)
{
	struct device *dev = priv->dev;
	char action[] = "ACTION=IAXXX_VQ_EVENT";
	char eid[14];
	char edata[20];
	char timestamp[64];
	char *event[] = {action, eid, edata, timestamp, NULL};
	static bool recognize;
	unsigned long bitmap;
	struct timeval now;

	do_gettimeofday(&now);
	snprintf(timestamp, sizeof(timestamp), "TIMESTAMP=%lld",
			(((long long)now.tv_sec * 1000000LL) +
			(long long)now.tv_usec));

	if (WARN_ON(!evt) || WARN_ON(!priv->tunnel_data))
		return -EINVAL;

	/* Check for tunnel event and notify producer thread*/
	if (evt->event_src == TNL_EVENT_SYSID) {
		iaxxx_tunnel_signal_event(priv);
		return 0;
	}

	if (evt->event_src == IAXXX_CM4_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_CRASH_EVENT_ID) {
		dev_err(dev, "FW crash %s: src:0x%.04x, proc id:0x%.04X\n",
			__func__, evt->event_src, evt->src_opaque);
		iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT);
		return -EIO;
	}

	/*
	 * If its KW detection event and event is for slot id. If KW is in
	 * recognize state continue, else return. Discard the other
	 * KW detection events also for that particular KW if its in
	 * un-recognize state
	 */
	if (evt->event_id == 0) {
		if (evt->src_opaque >= IAXXX_MAX_MODELS) {
			dev_err(dev, "Invalid slot id %d\n", evt->src_opaque);
			return -EINVAL;
		}
		bitmap = priv->iaxxx_state->kw_info.kw_recognize_bitmap;
		if (!test_bit(evt->src_opaque, &bitmap))
			recognize = false;
		else {
			recognize = true;
			/* Event has slot id, but upper layes need KW id */
			evt->src_opaque = priv->
				iaxxx_state->kw_info.kw_id[evt->src_opaque];
		}
	}
	if (!recognize)
		return 0;

	/* Hold the system awake for a while until user-space served events */
	if (test_and_clear_bit(IAXXX_FLG_WAKEUP_EVENT, &priv->flags))
		__pm_wakeup_event(priv->event_ws, IAXXX_WAKEUP_EVENT_MS);

	snprintf(eid, sizeof(eid), "EVENT_ID=%hx", evt->event_id);
	snprintf(edata, sizeof(edata), "EVENT_DATA=%x", evt->src_opaque);

	dev_dbg(dev, "%s: src:0x%.04x, id:0x%.04X\n",
			__func__, evt->event_src, evt->event_id);
	mutex_lock(&priv->event_queue_lock);
	iaxxx_add_event(priv, evt);
	mutex_unlock(&priv->event_queue_lock);
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, event);

	/*
	 * Normal event_id sequence: 0->1->2->3->5
	 * VT would be successfuly only when we get 5
	 */
	if (evt->event_id == 5)
		trace_fba_voice_trigger(++vt_count);

	return 0;
}

int iaxxx_evnt_mgr_unsubscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_unsubscribe);

int iaxxx_evnt_mgr_subscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst,
					u32 dst_opaque, void *userdata,
					iaxxx_evnt_mgr_callback cb_func_ptr)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_subscribe);
