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

#ifndef _UAPI_MH_ACOUSTICS_ADAPTER_H
#define _UAPI_MH_ACOUSTICS_ADAPTER_H

#include "fb-mha.h"

enum mh_adapter_commands {
	MHADAP_CMD_NOTHING = 0,
	MHADAP_CMD_FIRMWARE,
	MHADAP_CMD_FW_TUNING_START,
	MHADAP_CMD_FW_TUNING_STOP,
	MHADAP_CMD_STORE,
	MHADAP_CMD_LOAD,
};

#define MHADAP_IOCTL_MAGIC		'D'

/* Initialize size of BLOB buffer */
#define MHADAP_IOCTL_INIT_BLOB		_IOW(MHADAP_IOCTL_MAGIC, 0, uint32_t)

/* Set current algorithm version */
#define MHADAP_IOCTL_SET_VERSION	_IOW(MHADAP_IOCTL_MAGIC, 1, uint32_t)

/* Set Enable/Disable of algorithm */
#define MHADAP_IOCTL_ENABLE		_IOW(MHADAP_IOCTL_MAGIC, 2, bool)

/* Read current command when on POLLMSG  */
#define MHADAP_IOCTL_CMD_GET		_IOR(MHADAP_IOCTL_MAGIC, 3, \
						enum mh_adapter_commands)

#define MHADAP_IOCTL_CMD_COMPLETE	_IOW(MHADAP_IOCTL_MAGIC, 4, bool)

#define MHADAP_IOCTL_COUNT_PARAMS	_IOR(MHADAP_IOCTL_MAGIC, 5, int)

#define MHADAP_IOCTL_ENUM_PARAMS	_IOR(MHADAP_IOCTL_MAGIC, 6, \
						struct mha_parameter)

#endif /* _UAPI_MH_ACOUSTICS_ADAPTER_H */
