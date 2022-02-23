/*
 * include/uapi/sound/fb-stax-dspg.h
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

#ifndef _UAPI_FB_STAX_DSPG_H
#define _UAPI_FB_STAX_DSPG_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <uapi/sound/fb-stax-ctrl-internal.h>

#define STAX_DSPG_BLOB_MAX_SIZE     (1024 * 8)

int stax_dspg_register(struct stax_adapter_ops *ops);
int stax_dspg_unregister(struct stax_adapter_ops *ops);

long stax_dspg_ioctl(struct stax_file_ctx *fstax,
					unsigned int cmd, void __user *data);
struct stax_file_ctx *stax_dspg_open(struct stax_ctx *dspg_ctx);
int stax_dspg_release(struct stax_file_ctx *dspg_file_ctx);
struct stax_ctx *stax_dspg_init(struct miscdevice *mdev);
#endif /* __UAPI_FB_STAX_DSPG_H */
