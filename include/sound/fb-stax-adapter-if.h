/*
 * include/sound/fb-stax-adapter-if.h
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

#ifndef _STAX_ADAPTER_IF_H
#define _STAX_ADAPTER_IF_H

#include <linux/kernel.h>

struct stax_adapter_ops {
	void *private;

	bool (*is_ready)(struct stax_adapter_ops *);
	bool (*is_active)(struct stax_adapter_ops *);

	int (*get_version)(struct stax_adapter_ops *, uint32_t *version);

	int (*blob_size)(struct stax_adapter_ops *);
	int (*blob_xfer)(struct stax_adapter_ops *, void *blob,
			size_t size, int count, int app_type, bool snd_and_rcv);

	int (*fw_start)(struct stax_adapter_ops *, int params, int values);
	int (*fw_finish)(struct stax_adapter_ops *);
	int (*fw_tuning)(struct stax_adapter_ops *, bool);
};

#endif /* _STAX_ADAPTER_IF_H */
