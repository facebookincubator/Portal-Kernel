/*
 * include/uapi/sound/mh-acoustics.h
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

#ifndef _MH_ACOUSTICS_ADAPTER_IF_H
#define _MH_ACOUSTICS_ADAPTER_IF_H

#include <linux/kernel.h>

#include <uapi/sound/fb-mha.h>

struct mha_fw_header;

enum mha_adapter_notify {
	MHA_NOTIFY_NOTHING = 0,
	MHA_NOTIFY_CRASH,
	MHA_NOTIFY_START,
	MHA_NOTIFY_STOP,
};

struct mha_adapter_ops {
	struct device *dev;
	struct module *module;
	void *private;

	/* runtime power management */
	int (*pm_get)(struct mha_adapter_ops *);
	int (*pm_put)(struct mha_adapter_ops *);

	bool (*is_ready)(struct mha_adapter_ops *);
	bool (*is_active)(struct mha_adapter_ops *);

	int (*get_version)(struct mha_adapter_ops *, uint32_t *version);

	int (*blob_size)(struct mha_adapter_ops *);
	int (*blob_xfer)(struct mha_adapter_ops *, void *blob,
			size_t size, int count, bool snd_and_rcv);

	int (*fw_start)(struct mha_adapter_ops *, int params, int values);
	int (*fw_param)(struct mha_adapter_ops *, struct mha_parameter *);
	int (*fw_finish)(struct mha_adapter_ops *);
	int (*fw_tuning)(struct mha_adapter_ops *, bool);
};

int mha_register(struct mha_adapter_ops *);
int mha_unregister(struct mha_adapter_ops *);
int mha_notify(struct mha_adapter_ops *, enum mha_adapter_notify notify);
int mha_payload_size(enum mha_data_types type, int count);
int mha_firmware(struct device *, struct mha_adapter_ops *,
		const struct mha_fw_header *, size_t);

#endif /* _MH_ACOUSTICS_ADAPTER_IF_H */
