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

int stax_adapter_check(struct stax_adapter_ops *adap,
		struct device *dev);
int stax_blob_xfer(struct stax_adapter_ops *adap,
	struct stax_blob_buffer *blob, struct device *dev,
	bool snd_and_rcv);
int stax_blob_put(struct stax_blob_buffer *blob,
		struct stax_ctrl_desc *value, struct device *dev);
int stax_blob_get(struct stax_blob_buffer *blob,
		struct device *dev);

#endif /* __UAPI_FB_STAX_CTRL_INTERNAL_H */
