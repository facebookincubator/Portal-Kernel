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

#ifndef _UAPI_FB_STAX_CTRL_H
#define _UAPI_FB_STAX_CTRL_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Stax devices */
typedef enum stax_device {
	STAX_DEVICE_DSPG = 0,
	STAX_DEVICE_ADSP,
	STAX_DEVICE_HIFI
} stax_device;

struct stax_ctrl_desc {
	uint32_t device;
	uint32_t app_type;
	uint32_t len;		/* blob size */
	uint32_t resp_len;
	char data[0];
} __packed;

#define STAX_IOCTL_MAGIC 'e'

/*   Send blob data which contain one or more parameters
 *
 * Return value:
 *   0       - On success
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - STAX algorithm in firmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define STAX_IOCTL_SET_PARAM_BLOB	_IOW(STAX_IOCTL_MAGIC, 0, \
						struct stax_ctrl_desc)

/*   Get blob data which contain one or more parameters
 *
 * Return value:
 *   0       - On success
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - STAX algorithm in firmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define STAX_IOCTL_GET_PARAM_BLOB	_IOR(STAX_IOCTL_MAGIC, 1, \
						struct stax_ctrl_desc)

/*   Set and Get cmd blob is a blocking call. Driver does two operations
 *   first it sets blob and then gets blob.
 *
 * Return value:
 *   0       - On success and blob has output data.
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - STAX algorithm in firmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define STAX_IOCTL_SET_GET_CMD_BLOB	_IOR(STAX_IOCTL_MAGIC, 2, \
						struct stax_ctrl_desc)

#ifdef STAX_DEBUG
#define STAX_HEX_DUMP(_name_, _buf_, _size_) \
	print_hex_dump(KERN_ERR, _name_, DUMP_PREFIX_OFFSET, \
		32, 1, (u8 *)(_buf_), (_size_), false)
#else
#define STAX_HEX_DUMP(_name_, _buf_, _size_)	do {} while (0)
#endif

#endif /* __UAPI_FB_STAX_CTRL_H */
