/*
 * include/uapi/sound/fb-stax.h
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

#ifndef _UAPI_FB_STAX_CTRL_INTERNAL_H
#define _UAPI_FB_STAX_CTRL_INTERNAL_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <uapi/sound/fb-stax-ctrl.h>

#ifdef STAX_DEBUG
#define STAX_HEX_DUMP(_name_, _buf_, _size_) \
	print_hex_dump(KERN_ERR, _name_, DUMP_PREFIX_OFFSET, \
		32, 1, (u8 *)(_buf_), (_size_), false)
#else
#define STAX_HEX_DUMP(_name_, _buf_, _size_)	do {} while (0)
#endif

struct stax_blob_buffer {
	struct stax_ctrl_desc *value;		/* Auxilary value buffer */
	size_t size;				/* Total size of buffer */
	int len;				/* Length of values */
	int app_type;				/* App_type */
	u8 *buf;				/* Pointer to the buffer */
};

struct stax_ctx {
	struct mutex lock;
	struct miscdevice *mdev;
	struct stax_adapter_ops *adapter;
	size_t blob_size;
};

struct stax_file_ctx {
	struct stax_ctx *stax;        /*Context of dspg or adsp*/
	struct stax_blob_buffer blob; /*Local blob buffer */
};

int stax_adapter_check(struct stax_adapter_ops *adap,
		struct device *dev);
int stax_blob_xfer(struct stax_adapter_ops *adap,
	struct stax_blob_buffer *blob, struct device *dev,
	bool snd_and_rcv);
int stax_render_stream_get_status(struct stax_adapter_ops *adap,
    struct stax_blob_buffer *blob, struct device *dev);
int stax_blob_put(struct stax_blob_buffer *blob,
		struct stax_ctrl_desc *value, struct device *dev);
int stax_blob_get(struct stax_blob_buffer *blob,
		struct device *dev);
int stax_blob_alloc(struct stax_blob_buffer *blob,
        size_t blob_size, struct device *dev);
int stax_blob_free(struct stax_blob_buffer *blob);
int stax_blob_from_user(struct stax_file_ctx *fstax,
        struct stax_ctrl_desc __user *usr_data, struct device *dev);

#endif /* __UAPI_FB_STAX_CTRL_INTERNAL_H */
