/*
 * Copyright (c) 2020, Facebook Inc. All rights reserved.
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

#ifndef _RENDER_ADAPTER_IF_H
#define _RENDER_ADAPTER_IF_H

#include <linux/kernel.h>

#include <uapi/sound/fb-render.h>

struct ren_fw_header;

enum ren_adapter_notify {
	REN_NOTIFY_NOTHING = 0,
	REN_NOTIFY_CRASH,
	REN_NOTIFY_START,
	REN_NOTIFY_STOP,
};

struct ren_adapter_ops {
	void *private;

	bool (*is_ready)(struct ren_adapter_ops *);
	bool (*is_active)(struct ren_adapter_ops *);

	int (*get_version)(struct ren_adapter_ops *, uint32_t *version);

	int (*blob_size)(struct ren_adapter_ops *);
	int (*blob_xfer)(struct ren_adapter_ops *, void *blob,
			size_t size, int count, bool snd_and_rcv);

	int (*fw_start)(struct ren_adapter_ops *, int params, int values);
	int (*fw_param)(struct ren_adapter_ops *, struct ren_parameter *);
	int (*fw_finish)(struct ren_adapter_ops *);
};

int render_register(struct ren_adapter_ops *);
int render_unregister(struct ren_adapter_ops *);
int render_notify(struct ren_adapter_ops *, enum ren_adapter_notify notify);
int render_payload_size(enum ren_data_types type, int count);
int render_firmware(struct device *, struct ren_adapter_ops *,
		const struct ren_fw_header *, size_t);

#endif /* _RENDER_ACOUSTICS_ADAPTER_IF_H */
