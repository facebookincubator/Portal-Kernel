/*
 * iaxxx-tunnel-registers.h
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

#ifndef __IAXXX_TUNNEL_REGISTER_H__
#define __IAXXX_TUNNEL_REGISTER_H__

#include "iaxxx-register-defs-tunnel-header.h"
#include "iaxxx-register-defs-in-tunnel-group.h"
#include "iaxxx-register-defs-out-tunnel-group.h"

#define IAXXX_IN_TNL_GRP_TNL_CTRL_REG(I) \
	(IAXXX_IN_TNL_GRP_TNL_CTRL_ADDR + (4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_FRAME_REG(I) \
	(IAXXX_IN_TNL_GRP_TNL_FRAME_ADDR + (4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_FORMAT_REG(I) \
	(IAXXX_IN_TNL_GRP_FORMAT_ADDR + (4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_CTRL_REG(I) \
	(IAXXX_OUT_TNL_GRP_TNL_CTRL_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_FRAME_REG(I) \
	(IAXXX_OUT_TNL_GRP_TNL_FRAME_ADDR + \
	(4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_CONNECT_REG(I) \
	(IAXXX_OUT_TNL_GRP_CONNECT_ADDR + (4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_STATUS_REG(I) \
	(IAXXX_OUT_TNL_GRP_STATUS_ADDR + (4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(I) \
	(IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_ADDR + \
	 (4 * (I) * IAXXX_IN_TNL_GRP_REG_NUM))

#define IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_REG(I) \
	(IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_ADDR + \
	 (4 * (I) * IAXXX_OUT_TNL_GRP_REG_NUM))

#endif /* __IAXXX_TUNNEL_REGISTER_H__ */
