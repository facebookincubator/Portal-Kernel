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

#ifndef _UAPI_RENDER_H
#define _UAPI_RENDER_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Max count of channels per parameter */
#define REN_MAX_CHANNELS		16

#define REN_PARAM_ERROR			((uint8_t)0xEE)

enum ren_data_types {
	mkBool,
	mkChar8,
	mkChar16,
	mkString8,	/* for a string type the length attribute is the max.
			 * Length, while for char its the exact length
			 */
	mkString16,
	mkInt16,
	mkInt32,
	mkInt64,
	mkFloat16,
	mkFloat32,
	mkFloat64,
	mkInt16C,
	mkInt32C,
	mkInt64C,
	mkFloat16C,
	mkFloat32C,
	mkFloat64C,
	mkVoid32Ptr,
	mkVoid64Ptr
};

enum ren_flags {
	REN_ACCESS_READ = (1 << 0),
	REN_ACCESS_WRITE = (1 << 1),
};

struct ren_data_type {
	enum ren_data_types type;	/* Parameter data type */
	uint8_t size;			/* Size in bytes for one element */
	char name[16];			/* Parameter name, 0 terminated */
} __packed;

struct ren_parameter {
	uint32_t param;			/* Parameter Id */
	enum ren_data_types type;	/* Parameter data type */
	uint32_t count;			/* Count of elements */
	uint8_t flag;			/* Flags */
	char name[64];			/* Parameter name, 0 terminated */
	uint8_t ch_cnt;			/* Count of channels [0..16] */
	uint8_t ch_map[REN_MAX_CHANNELS]; /* List of valid channel indexes */
} __packed;

struct ren_value {
	uint32_t param;			/* Parameter Id */
	enum ren_data_types type;	/* Parameter data type */
	uint32_t count;			/* Count of elements */
	uint8_t channel;		/* Channel */
	char data[0];
} __packed;

struct ren_blob_state {
	uint32_t size;			/* Maximum size of blob in bytes */
	uint32_t count;			/* Count of values */
	uint32_t pos_r;			/* Read pointer of blob */
	uint32_t pos_w;			/* Write pointer of blob */
} __packed;

struct ren_xfer {
	uint32_t total;			/* Total parameters */
	uint32_t ready;			/* Ready for get parameters */
	uint32_t transmit;		/* Count of transmited parameters */
	uint32_t receive;		/* Count of recevied parameters */
	uint32_t no_access;		/* No right access */
	uint32_t failed;		/* Failed */
	uint32_t blob_size;		/* Blob size in bytes */
} __packed;

struct ren_list_id {
	int count;
	uint16_t id[];
} __packed;

#define REN_IOCTL_MAGIC 'd'

/* Get version of REN Acoustic algorithm */
#define REN_IOCTL_VERSION		_IOR(REN_IOCTL_MAGIC, 0, uint)

/* Get data type info by Id */
#define REN_IOCTL_TYPE_BY_ID		_IOR(REN_IOCTL_MAGIC, 1, \
						struct ren_data_type)

/* Get data type info by Name */
#define REN_IOCTL_TYPE_BY_NAME		_IOR(REN_IOCTL_MAGIC, 2, \
						struct ren_data_type)

/* Enumerate data type, restarts with NULL pointer */
#define REN_IOCTL_TYPE_ENUM		_IOR(REN_IOCTL_MAGIC, 3, \
						struct ren_data_type)

/* Get count of known parameters */
#define REN_IOCTL_PARAM_COUNT		_IOR(REN_IOCTL_MAGIC, 4, int)

/* Find parameter info by Id */
#define REN_IOCTL_PARAM_BY_ID		_IOWR(REN_IOCTL_MAGIC, 5, \
						struct ren_parameter)

/* Find parameter info by Name */
#define REN_IOCTL_PARAM_BY_NAME		_IOWR(REN_IOCTL_MAGIC, 6, \
						struct ren_parameter)

/* Enumerate parameters, restart with NULL pointer */
#define REN_IOCTL_PARAM_ENUM		_IOR(REN_IOCTL_MAGIC, 7, \
						struct ren_parameter)

/* Add parameter in list of known parameters */
#define REN_IOCTL_PARAM_ADD		_IOW(REN_IOCTL_MAGIC, 8, \
						struct ren_parameter)

/* Delete specific parameter by Id */
#define REN_IOCTL_PARAM_DEL_BY_ID	_IOW(REN_IOCTL_MAGIC, 9, int)

/* Delete all parameters */
#define REN_IOCTL_PARAM_DEL_ALL		_IO(REN_IOCTL_MAGIC, 10)

/* Direct write one parameter value and add it in write cache */
#define REN_IOCTL_VALUE_SET		_IOW(REN_IOCTL_MAGIC, 13, \
						struct ren_value)

/* Direct read one parameter value
 *
 * Return value:
 *   0       - On success
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - Render algoruthm in frmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define REN_IOCTL_VALUE_GET		_IOR(REN_IOCTL_MAGIC, 14, \
						struct ren_value)

/* Group - Clean blob buffer */
#define REN_IOCTL_GROUP_CLEAN		_IO(REN_IOCTL_MAGIC, 15)

/* Group - Get state of current blob buffer */
#define REN_IOCTL_GROUP_STATE		_IOR(REN_IOCTL_MAGIC, 16, \
						struct ren_blob_state)

/* Group - Store parameters through the adapter. Send blob buffer.
 *
 * Return value:
 *   > 0     - On success return size of blob
 *   ENOENT  - No more parameters for store
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - Render algoruthm in frmware doesn't work
 *   EFAULT  - Can't copy data to user-space
 */
#define REN_IOCTL_GROUP_STORE		_IO(REN_IOCTL_MAGIC, 18)

/* Group - Load parameters through the adapter. Send then receive blob 
 * buffer.
 *
 * Return value:
 *   > 0     - On success return size of blob
 *   ENOENT  - No more parameters for load
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - Render algoruthm in frmware doesn't work
 *   EFAULT  - Can't copy data to user-space
 */
#define REN_IOCTL_GROUP_LOAD		_IO(REN_IOCTL_MAGIC, 19)

/* Group - Serialize parameter value into the blob buffer
 *
 * Return value:
 *   0       - On success
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - Render algoruthm in frmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define REN_IOCTL_GROUP_PUT		_IOW(REN_IOCTL_MAGIC, 20, \
						struct ren_value)

/* Group - De-serialize parameter value from the blob buffer
 *
 * Return value:
 *   0       - On success
 *   EINVAL  - Valid data: pointer is NULL
 *   EBUSY   - Render algoruthm in frmware doesn't work
 *   EPREM   - Access permission is invalid, only readble
 *   EFAULT  - Can't copy data to user-space
 */
#define REN_IOCTL_GROUP_GET		_IOR(REN_IOCTL_MAGIC, 21, \
						struct ren_value)

/* Firmware - Create new set of FW metadata
 *
 * Return value:
 *   0       - On success
 *   ENOMEM  - Insufficient memory
 */
#define REN_IOCTL_FW_NEW		_IO(REN_IOCTL_MAGIC, 22)

/* Firmware - Store parameter in the new firmware
 *
 * Return value:
 *   0       - On success
 *   ENODATA - No active firmware available.
 *   ENOMEM  - Insufficient memory
 */
#define REN_IOCTL_FW_PARAM		_IOW(REN_IOCTL_MAGIC, 23, \
							struct ren_parameter)

/* Firmware - Store value in the new firmware
 *
 * Return value:
 *   0       - On success
 *   ENODATA - No backup firmware available.
 *   ENOMEM  - Insufficient memory
 */
#define REN_IOCTL_FW_VALUE		_IOW(REN_IOCTL_MAGIC, 24, \
							struct ren_value)

/* Firmware - Prepare FW binary from current active list of parameters and
 *            set of values. Replaces the current active FW if available 
 *            with the newly prepared FW.
 *
 * Return value:
 *   0       - On success
 *   ENOMEM  - Insufficient memory
 */
#define REN_IOCTL_FW_PREPARE		_IO(REN_IOCTL_MAGIC, 25)

/* Firmware - Starting add parameter values from new group ID
 *
 * Return value:
 *   >0      - Count of writed values
 *    0      - There is no values assigned to this ID
 *   <0      - Errors
 */
#define REN_IOCTL_FW_GROUP_ID		_IOW(REN_IOCTL_MAGIC, 26, int)

/* Firmware - Set list of groups used as default
 *
 * Return value:
 *   0       - On success
 */
#define REN_IOCTL_FW_DEFAULT		_IOW(REN_IOCTL_MAGIC, 27, \
							struct ren_list_id)

/* Firmware - Serialize all parameter values from specific group_id
 *            into the blob buffer.
 *
 * Return value:
 *   >0      - Return count of parameters in specified group id
 *   EINVAL  - Argument is invalid
 *   EFAULT  - Can't get argument value
 *   ENOENT  - Missing entries for specified group ID
 */
#define REN_IOCTL_GROUP_ID_PUT		_IOW(REN_IOCTL_MAGIC, 29, int)

#endif /* _UAPI_RENDER_H */
