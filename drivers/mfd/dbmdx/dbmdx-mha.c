/*
 * DSPG DBMDX MH Acousting interface driver
 *
 * Copyright (c) 2019, Facebook Inc. All rights reserved.
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

#include <sound/fb-mha-adapter-if.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <dt-bindings/sound/dbmdx/dbmd7_fw_api.h>

#define DBMDX_MHA_BUFFER		1024
#define DBMDX_MHA_SET_GET_TIMEOUT_MS	100

#define DBMDX_MHA_FLG_READY		0
#define DBMDX_MHA_FLG_ACTIVE		1
#define DBMDX_MHA_FLG_FIRMWARE		2
#define DBMDX_MHA_FLG_ENABLED		3
#define DBMDX_MHA_FLG_SKIP_FW_TUNING	4

struct dbmdx_mha_packet {
	uint32_t count;
	char data[];
};

struct dbmdx_mha_ctx {
	struct mha_adapter_ops adap;
	struct platform_device *pdev;
	struct device *mfd_dev;
	struct dbmdx_private *mfd_priv;
	struct notifier_block nb;
	struct dbmdx_mha_packet *pck;
	unsigned long flags;
	const char *fw_name;
};

static void dbmdx_mha_update_active(struct dbmdx_mha_ctx *mha)
{
	/* Checks is everything registered and ready for work */
	if (!test_bit(DBMDX_MHA_FLG_READY, &mha->flags))
		return;

	/* Checks is firmware ready */
	if (!test_bit(DBMDX_MHA_FLG_FIRMWARE, &mha->flags))
		return;

	/* Check is algorithm enabled */
	if (test_bit(DBMDX_MHA_FLG_ENABLED, &mha->flags)) {
		/* Activate MHA adapter and send notification for START */
		if (!test_and_set_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags))
			mha_notify(&mha->adap, MHA_NOTIFY_START);
	} else {
		/* Deactivate MHA adapter and send notification for STOP */
		if (test_and_clear_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags))
			mha_notify(&mha->adap, MHA_NOTIFY_STOP);
	}
}

static int dbmdx_mha_read32(struct device *dev,
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

static int dbmdx_mha_wait_for_ready(struct dbmdx_mha_ctx *mha, int timeout_ms)
{
	struct dbmdx_private *mfd = mha->mfd_priv;
	unsigned long timeout;
	s32 ready;
	int rval;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		rval = dbmdx_read_register32(mfd, MSG_ID_ADDON_FB,
					SET_GET_PARAM_READY, &ready);
		if (!rval && (ready & 0x1))
			break;

		msleep(10);

		if (!test_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags))
			return -EPERM;

	} while (time_before_eq(jiffies, timeout));

	if (rval)
		return rval;

	if (ready & 0x1)
		return 0;

	if (!time_before_eq(jiffies, timeout)) {
		dev_err(&mha->pdev->dev, "timeout SET_GET_PARAM_READY err:%d dsp ret:%x\n",
			rval, ready);
		return -ETIMEDOUT;
	}

	return -EBUSY;
}

static bool dbmdx_mha_is_ready(struct mha_adapter_ops *adap)
{
	struct dbmdx_mha_ctx *mha = adap->private;

	return !!test_bit(DBMDX_MHA_FLG_READY, &mha->flags);
}

static bool dbmdx_mha_is_active(struct mha_adapter_ops *adap)
{
	struct dbmdx_mha_ctx *mha = adap->private;

	return !!test_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags);
}

static int dbmdx_mha_get_version(struct mha_adapter_ops *adap, uint32_t *ver)
{
	struct dbmdx_mha_ctx *mha = adap->private;
	struct device *dev = &mha->pdev->dev;
	struct dbmdx_private *mfd = mha->mfd_priv;

	return dbmdx_mha_read32(dev, mfd, ALGO_VERSION_NUMBER, ver);
}

static int dbmdx_blob_size(struct mha_adapter_ops *adap)
{
	return DBMDX_MHA_BUFFER - sizeof(struct dbmdx_mha_packet);
}

static int dbmdx_blob_xfer(struct mha_adapter_ops *adap, void *blob,
	size_t size, int count, bool snd_and_rcv)
{
	struct dbmdx_mha_ctx *mha = adap->private;
	struct dbmdx_private *p = mha->mfd_priv;
	struct dbmdx_mha_packet *pck = mha->pck;
	s32 ack_val;
	u8 *buf;
	int rval;

	if (!test_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags))
		return -EPERM;

	/* Poll ready flag until complete filling the return buffer */
	rval = dbmdx_mha_wait_for_ready(mha, DBMDX_MHA_SET_GET_TIMEOUT_MS);
	if (rval)
		return rval;

	if (size + sizeof(*pck) > DBMDX_MHA_BUFFER)
		return -EINVAL;

	memcpy(pck->data, blob, size);
	pck->count = count;
	buf = (u8 *)pck;
	size += sizeof(*pck);

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

	size -= sizeof(*pck);

	return rval < 0 ? rval : size;
}

static int dbmdx_fw_tuning(struct mha_adapter_ops *adap, bool enable)
{
	struct dbmdx_mha_ctx *mha = adap->private;

	if (enable && test_bit(DBMDX_MHA_FLG_SKIP_FW_TUNING, &mha->flags))
		return -EPERM;

	return 0;
}

static int dbmdx_mha_msg_handler(struct dbmdx_mha_ctx *mha,
	struct fw_message *msg)
{
	u8 id = msg->header.id;
	u8 reg = msg->header.reg_num;

	if (id == MSG_ID_ADDON_FB) {
		switch (reg) {
		case FB_ADDON_ALGO_ENABLED:
			set_bit(DBMDX_MHA_FLG_ENABLED, &mha->flags);
			dbmdx_mha_update_active(mha);
			break;
		case FB_ADDON_ALGO_DISABLED:
			clear_bit(DBMDX_MHA_FLG_ENABLED, &mha->flags);
			dbmdx_mha_update_active(mha);
			break;
		}
	}

	return 0;
}

static int dbmdx_mha_notify(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct dbmdx_mha_ctx *mha = container_of(nb, typeof(*mha), nb);

	switch (action) {
	case DBMDX_EV_CRASH:
		/* Simulate disabled alogorithm and deactivated adapter */
		clear_bit(DBMDX_MHA_FLG_ENABLED, &mha->flags);
		clear_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags);
		mha_notify(&mha->adap, MHA_NOTIFY_CRASH);
		break;
	case DBMDX_EV_RECOVERY:
		break;
	case DBMDX_EV_MESSAGE:
		dbmdx_mha_msg_handler(mha, data);
		break;
	}

	return 0;
}

struct mha_adapter_ops dbmdx_mha_adapter = {
	.is_ready = dbmdx_mha_is_ready,
	.is_active = dbmdx_mha_is_active,
	.get_version = dbmdx_mha_get_version,
	.blob_size = dbmdx_blob_size,
	.blob_xfer = dbmdx_blob_xfer,
	.fw_tuning = dbmdx_fw_tuning,
};

static void dbmdx_mha_firmware(const struct firmware *fw, void *context)
{
	struct dbmdx_mha_ctx *mha = context;
	struct device *dev = &mha->pdev->dev;

	if (!fw) {
		dev_err(dev, "FW: Invalid firmware!\n");
		return;
	}

	dev_info(dev, "MHA FW: name %s ; size %zd ; data %p\n",
			mha->fw_name, fw->size, fw->data);

	mha_firmware(dev, &mha->adap, (void *)fw->data, fw->size);

	release_firmware(fw);

	set_bit(DBMDX_MHA_FLG_FIRMWARE, &mha->flags);

	dbmdx_mha_update_active(mha);
}

static int dbmdx_mha_dt_parser(struct device *dev, struct dbmdx_mha_ctx *mha)
{
	struct device_node *np = dev->of_node;
	int rval;

	rval = of_property_read_string(np, "dspg,firmware-name", &mha->fw_name);
	if (rval)
		mha->fw_name = NULL;

	if (of_property_read_bool(np, "dspg,skip-fw-tuning"))
		set_bit(DBMDX_MHA_FLG_SKIP_FW_TUNING, &mha->flags);

	return 0;
}

static int mha_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dbmdx_mha_ctx *mha;
	int rval;

	mha = devm_kzalloc(dev, sizeof(*mha), GFP_KERNEL);
	if (!mha)
		return -ENOMEM;

	platform_set_drvdata(pdev, mha);

	rval = dbmdx_mha_dt_parser(dev, mha);
	if (rval)
		return rval;

	mha->pdev = pdev;
	mha->mfd_dev = pdev->dev.parent;
	mha->mfd_priv = dev_get_drvdata(pdev->dev.parent);
	mha->adap = dbmdx_mha_adapter;
	mha->adap.private = mha;

	mha->pck = devm_kzalloc(dev, DBMDX_MHA_BUFFER, GFP_KERNEL);
	if (!mha->pck)
		return -ENOMEM;

	mha->nb.notifier_call = dbmdx_mha_notify;
	dbmdx_notifier_register(mha->mfd_dev, &mha->nb);

	rval = mha_register(&mha->adap);
	if (rval)
		goto no_mha_register;

	if (mha->fw_name) {
		rval = request_firmware_nowait(THIS_MODULE, true, mha->fw_name,
				dev, GFP_KERNEL, mha, dbmdx_mha_firmware);
		if (rval) {
			dev_info(dev, "MHA FW: Request failed: %d\n", rval);
			goto no_firmware;
		}

		dev_info(dev, "MHA FW: Successfully requested: %s\n", mha->fw_name);
	} else
		set_bit(DBMDX_MHA_FLG_FIRMWARE, &mha->flags);

	/* In case of msg is sent before registering local msg handler */
	if (mha->mfd_priv->algo_enabled)
		set_bit(DBMDX_MHA_FLG_ENABLED, &mha->flags);

	set_bit(DBMDX_MHA_FLG_READY, &mha->flags);

	dbmdx_mha_update_active(mha);

	return 0;

no_firmware:
	mha_unregister(&mha->adap);

no_mha_register:
	dbmdx_notifier_unregister(mha->mfd_dev, &mha->nb);

	clear_bit(DBMDX_MHA_FLG_READY, &mha->flags);
	clear_bit(DBMDX_MHA_FLG_ACTIVE, &mha->flags);

	return rval;
}

static int mha_platform_remove(struct platform_device *pdev)
{
	struct dbmdx_mha_ctx *mha = platform_get_drvdata(pdev);

	mha_unregister(&mha->adap);

	return 0;
}

static const struct of_device_id mha_of_match[] = {
	{ .compatible = "dspg,dbmdx-mha", },
	{},
};

static struct platform_driver mha_driver = {
	.driver = {
		.name = "dbmdx-mha",
		.of_match_table = mha_of_match,
		.owner = THIS_MODULE,
	},
	.probe = mha_platform_probe,
	.remove = mha_platform_remove,
};

module_platform_driver(mha_driver);

MODULE_DESCRIPTION("DBMDX MH Acoustics driver");
MODULE_AUTHOR("Dinko Mironov <dmironov@fb.com>");
MODULE_LICENSE("GPL v2");
