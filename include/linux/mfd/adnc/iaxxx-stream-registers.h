/*
 * iaxxx-stream-registers.h
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#ifndef __IAXXX_STREAM_REGISTER_H__
#define __IAXXX_STREAM_REGISTER_H__

#include "iaxxx-register-defs-stream-header.h"
#include "iaxxx-register-defs-stream-group.h"

#define IAXXX_STR_GRP_STR_CTRL_REG(I) \
	(IAXXX_STR_GRP_STR_CTRL_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_STATUS_REG(I) \
	(IAXXX_STR_GRP_STR_STATUS_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_FORMAT_REG(I) \
	(IAXXX_STR_GRP_STR_FORMAT_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#define IAXXX_STR_GRP_STR_SYNC_REG(I) \
	(IAXXX_STR_GRP_STR_SYNC_ADDR + (4 * (I) * IAXXX_STR_GRP_REG_NUM))

#endif /* __IAXXX_STREAM_REGISTER_H__ */
