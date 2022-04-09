/*
 * DSPG DBMDX Stax interface driver
 *
 * Copyright (c) 2021, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <sound/fb-stax-adapter-if.h>
#include <uapi/sound/fb-stax-ctrl-internal.h>
#include <uapi/sound/fb-stax-dspg.h>
#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <dt-bindings/sound/dbmdx/dbmd7_fw_api.h>
#include <uapi/linux/fb_event.h>
#include <linux/fb_event.h>

#define DBMDX_STAX_BUFFER		STAX_DSPG_BLOB_MAX_SIZE
#define DBMDX_STAX_SET_GET_TIMEOUT_MS	100
#define DBMDX_STAX_SET_GET_TIMEOUT_REGRESSION_MS 10000

#define DBMDX_STAX_FLG_READY		0
#define DBMDX_STAX_FLG_ACTIVE		1
#define DBMDX_STAX_FLG_FIRMWARE		2
#define DBMDX_STAX_FLG_ENABLED		3
#define DBMDX_STAX_FLG_SKIP_FW_TUNING	4

struct dbmdx_stax_packet {
	uint32_t count;
	char data[];
};

struct dbmdx_stax_ctx {
	struct stax_adapter_ops adap;
	struct platform_device *pdev;
	struct device *mfd_dev;
	struct dbmdx_private *mfd_priv;
	struct notifier_block nb;
	struct dbmdx_stax_packet *pck;
	unsigned long flags;
	const char *fw_name;
	u32 get_set_timeout_ms;
};

static void dbmdx_stax_update_active(struct dbmdx_stax_ctx *stax)
{
	/* Checks is everything registered and ready for work */
	if (!test_bit(DBMDX_STAX_FLG_READY, &stax->flags))
		return;

	/* Checks is firmware ready */
	if (!test_bit(DBMDX_STAX_FLG_FIRMWARE, &stax->flags))
		return;

	/* Check is algorithm enabled */
	if (test_bit(DBMDX_STAX_FLG_ENABLED, &stax->flags)) {
		/* Activate STAX adapter */
		set_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags);
	} else {
		/* Deactivate STAX adapter */
		clear_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags);
	}
}

static int dbmdx_stax_read32(struct device *dev,
	struct dbmdx_private *mfd, u8 reg, u32 *val)
{
	int rval;

	rval = dbmdx_read_register32(mfd, MSG_ID_ADDON_FB, reg, val);
	if (rval < 0) {
		dev_err(dev, "Error %d on reading register %d\n", rval, reg);
		return rval;
	}

	dev_dbg(dev, "Register %d read, value %d\n", reg, *val);

	return 0;
}

static int dbmdx_stax_wait_for_ready(struct dbmdx_stax_ctx *stax,
				int timeout_ms)
{
	struct dbmdx_private *mfd = stax->mfd_priv;
	unsigned long timeout;
	s32 ready;
	int rval;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		rval = dbmdx_read_register32(mfd, MSG_ID_ADDON_FB,
					SET_GET_PARAM_READY, &ready);
		if (!rval && (ready & 0x1))
			break;

		usleep_range(2000, 2100);

		if (!test_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags))
			return -EPERM;

	} while (time_before_eq(jiffies, timeout));

	if (rval)
		return rval;

	if (ready & 0x1)
		return 0;

	if (!time_before_eq(jiffies, timeout)) {
		dev_err(&stax->pdev->dev, "timeout SET_GET_PARAM_READY\n");
		return -ETIMEDOUT;
	}

	return -EBUSY;
}

static bool dbmdx_stax_is_ready(struct stax_adapter_ops *adap)
{
	struct dbmdx_stax_ctx *stax = adap->private;

	return !!test_bit(DBMDX_STAX_FLG_READY, &stax->flags);
}

static bool dbmdx_stax_is_active(struct stax_adapter_ops *adap)
{
	struct dbmdx_stax_ctx *stax = adap->private;

	return !!test_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags);
}

static int dbmdx_stax_get_version(struct stax_adapter_ops *adap,
		uint32_t *ver)
{
	struct dbmdx_stax_ctx *stax = adap->private;
	struct device *dev = &stax->pdev->dev;
	struct dbmdx_private *mfd = stax->mfd_priv;

	return dbmdx_stax_read32(dev, mfd, ALGO_VERSION_NUMBER, ver);
}

static int dbmdx_blob_size(struct stax_adapter_ops *adap)
{
	return DBMDX_STAX_BUFFER - sizeof(struct dbmdx_stax_packet);
}

static int dbmdx_blob_xfer(struct stax_adapter_ops *adap, void *blob,
	size_t size, int count, int app_type, bool snd_and_rcv)
{
/* #define DBMDX_STAX_TEST */
	struct dbmdx_stax_ctx *stax = adap->private;
	struct dbmdx_stax_packet *pck = stax->pck;
	s32 ack_val;
	u8 *buf;
	int rval;

#ifdef DBMDX_STAX_TEST
	struct device *dev = &stax->pdev->dev;
	int i;
#else
	struct dbmdx_private *p = stax->mfd_priv;
#endif
	if (!test_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags))
		return -EPERM;

	/* Poll ready flag until complete filling the return buffer */
	rval = dbmdx_stax_wait_for_ready(stax, stax->get_set_timeout_ms);
	if (rval)
		return rval;

	if (size + sizeof(*pck) > DBMDX_STAX_BUFFER)
		return -EINVAL;

	memcpy(pck->data, blob, size);
	pck->count = count;
	buf = (u8 *)pck;
	size += sizeof(*pck);

#ifdef DBMDX_STAX_TEST
	dev_err(dev, "### size:%d\n", (int)size);
	for (i = 0; i < size; i++)
		dev_err(dev, "### %d\n", buf[i]);
	ack_val = 0;

	if (snd_and_rcv) {
		for (i = 0; i < size; i++)
			buf[i] = 0x5;
		memcpy(blob, pck->data, size - sizeof(*pck));
	}
	dev_err(dev, " %s: ### Exit ###\n", __func__);

#else
	if (snd_and_rcv) {
		rval = dbmdx_read_buf_msg(p, MSG_ID_ADDON_FB,
			GET_ALGO_PARAM, buf, size, &buf, size);
		if (rval >= 0)
			memcpy(blob, pck->data, size - sizeof(*pck));
	} else {
		rval = dbmdx_send_message_ack(p, MSG_ID_ADDON_FB,
			SET_ALGO_PARAM, buf, size, &ack_val);
		if (rval >= 0 && ack_val)
			rval = -EIO;
	}
#endif
	size -= sizeof(*pck);

	return rval < 0 ? rval : size;
}

static int dbmdx_fw_tuning(struct stax_adapter_ops *adap, bool enable)
{
	struct dbmdx_stax_ctx *stax = adap->private;

	if (enable && test_bit(DBMDX_STAX_FLG_SKIP_FW_TUNING, &stax->flags))
		return -EPERM;

	return 0;
}

static int dbmdx_stax_msg_handler(struct dbmdx_stax_ctx *stax,
	struct fw_message *msg)
{
	u8 id = msg->header.id;
	u8 reg = msg->header.reg_num;
	union fb_stax_event event;

	if (id == MSG_ID_ADDON_FB) {
		switch (reg) {
		case FB_ADDON_ALGO_ENABLED:
			set_bit(DBMDX_STAX_FLG_ENABLED, &stax->flags);
			dbmdx_stax_update_active(stax);
			break;
		case FB_ADDON_ALGO_DISABLED:
			clear_bit(DBMDX_STAX_FLG_ENABLED, &stax->flags);
			dbmdx_stax_update_active(stax);
			break;
		case FB_ADDON_AWB_LOADED:
			event.event = FBE_STAX_AWB_LOAD_COMPLETE;
			event.device_id = FBE_STAX_DEVICE_DSPG;
			dbmdx_event_log_int(stax->mfd_dev, FBE_STAX,
						event.value);
		}
	}

	return 0;
}

static int dbmdx_driver_msg_handler(struct dbmdx_stax_ctx *stax,
	struct drv_message *msg)
{
	struct mixer_ctl *ctl = NULL;

	switch (msg->msg_id) {
	case DBMDX_MIXER_CTRL_MODIFIED:
		ctl = (struct mixer_ctl *)msg->msg;
		switch (ctl->reg) {
		case STAX_REGRESSION_CTRL:
			if (ctl->val) {
				stax->get_set_timeout_ms =
					DBMDX_STAX_SET_GET_TIMEOUT_REGRESSION_MS;
			} else {
				stax->get_set_timeout_ms =
					DBMDX_STAX_SET_GET_TIMEOUT_MS;
			}
			break;
		}
		break;
	}
	return 0;
}

static int dbmdx_stax_notify(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct dbmdx_stax_ctx *stax = container_of(nb, typeof(*stax), nb);

	switch (action) {
	case DBMDX_EV_CRASH:
		/* Simulate disabled alogorithm and deactivated adapter */
		clear_bit(DBMDX_STAX_FLG_ENABLED, &stax->flags);
		clear_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags);
		break;
	case DBMDX_EV_RECOVERY:
		break;
	case DBMDX_EV_MESSAGE:
		dbmdx_stax_msg_handler(stax, data);
		break;
	case DBMDX_DRV_MESSAGE:
		dbmdx_driver_msg_handler(stax, data);
	}
	return 0;
}

struct stax_adapter_ops dbmdx_stax_adapter = {
	.is_ready = dbmdx_stax_is_ready,
	.is_active = dbmdx_stax_is_active,
	.get_version = dbmdx_stax_get_version,
	.blob_size = dbmdx_blob_size,
	.blob_xfer = dbmdx_blob_xfer,
	.fw_tuning = dbmdx_fw_tuning,
};

static void dbmdx_stax_firmware(const struct firmware *fw, void *context)
{
	struct dbmdx_stax_ctx *stax = context;
	struct device *dev = &stax->pdev->dev;

	if (!fw) {
		dev_err(dev, "FW: Invalid firmware!\n");
		return;
	}

	dev_info(dev, "STAX FW: name %s ; size %zd ; data %p\n",
			stax->fw_name, fw->size, fw->data);
	release_firmware(fw);

	set_bit(DBMDX_STAX_FLG_FIRMWARE, &stax->flags);

	dbmdx_stax_update_active(stax);
}

static int dbmdx_stax_dt_parser(struct device *dev, struct dbmdx_stax_ctx *stax)
{
	struct device_node *np = dev->of_node;
	int rval;

	rval = of_property_read_string(np, "dspg,firmware-name",
					&stax->fw_name);
	if (rval)
		stax->fw_name = NULL;

	if (of_property_read_bool(np, "dspg,skip-fw-tuning"))
		set_bit(DBMDX_STAX_FLG_SKIP_FW_TUNING, &stax->flags);

	return 0;
}

static int stax_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dbmdx_stax_ctx *stax;
	int rval;

	stax = devm_kzalloc(dev, sizeof(*stax), GFP_KERNEL);
	if (!stax)
		return -ENOMEM;

	platform_set_drvdata(pdev, stax);

	rval = dbmdx_stax_dt_parser(dev, stax);
	if (rval)
		return rval;

	stax->pdev = pdev;
	stax->mfd_dev = pdev->dev.parent;
	stax->mfd_priv = dev_get_drvdata(pdev->dev.parent);
	stax->adap = dbmdx_stax_adapter;
	stax->adap.private = stax;
	stax->get_set_timeout_ms = DBMDX_STAX_SET_GET_TIMEOUT_MS;
	stax->pck = devm_kzalloc(dev, DBMDX_STAX_BUFFER, GFP_KERNEL);
	if (!stax->pck)
		return -ENOMEM;

	stax->nb.notifier_call = dbmdx_stax_notify;
	dbmdx_notifier_register(stax->mfd_dev, &stax->nb);
	rval = stax_dspg_register(&stax->adap);
	if (rval)
		goto no_stax_register;

	if (stax->fw_name) {
		rval = request_firmware_nowait(THIS_MODULE, true, stax->fw_name,
				dev, GFP_KERNEL, stax, dbmdx_stax_firmware);
		if (rval) {
			dev_info(dev, "STAX FW: Request failed: %d\n", rval);
			goto no_firmware;
		}

		dev_info(dev, "STAX FW: Successfully requested: %s\n",
				stax->fw_name);
	} else
		set_bit(DBMDX_STAX_FLG_FIRMWARE, &stax->flags);

	/* In case of msg is sent before registering local msg handler */
	if (stax->mfd_priv->algo_enabled)
		set_bit(DBMDX_STAX_FLG_ENABLED, &stax->flags);

	set_bit(DBMDX_STAX_FLG_READY, &stax->flags);

	dbmdx_stax_update_active(stax);

	return 0;

no_firmware:
	stax_dspg_unregister(&stax->adap);

no_stax_register:
	dbmdx_notifier_unregister(stax->mfd_dev, &stax->nb);

	clear_bit(DBMDX_STAX_FLG_READY, &stax->flags);
	clear_bit(DBMDX_STAX_FLG_ACTIVE, &stax->flags);

	return rval;
}

static int stax_platform_remove(struct platform_device *pdev)
{
	struct dbmdx_stax_ctx *stax = platform_get_drvdata(pdev);

	stax_dspg_unregister(&stax->adap);

	return 0;
}

static const struct of_device_id stax_of_match[] = {
	{ .compatible = "dspg,dbmdx-stax", },
	{},
};

static struct platform_driver stax_driver = {
	.driver = {
		.name = "dbmdx-stax",
		.of_match_table = stax_of_match,
		.owner = THIS_MODULE,
	},
	.probe = stax_platform_probe,
	.remove = stax_platform_remove,
};

module_platform_driver(stax_driver);

MODULE_DESCRIPTION("DBMDX stax driver");
MODULE_AUTHOR("Murali Ganji <muraliganji@fb.com>");
MODULE_LICENSE("GPL v2");
