/*
 * iaxxx-register-defs-in-tunnel-group.h -- IAxxx register definition
 *
 * Copyright (c) 2016 Knowles, inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_REGISTER_DEFS_IN_TNL_GRP_H__
#define __IAXXX_REGISTER_DEFS_IN_TNL_GRP_H__

/*** The base address for this set of registers ***/
#define IAXXX_IN_TNL_GRP_REGS_ADDR (0x03000030)

/*** IN_TNL_GRP_TNL_CTRL (0x03000030) ***/
/*
 * Tunnel Control Register.
 */
#define IAXXX_IN_TNL_GRP_TNL_CTRL_ADDR (0x03000030)
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MASK_VAL 0x00000003
#define IAXXX_IN_TNL_GRP_TNL_CTRL_RMASK_VAL 0x00000003
#define IAXXX_IN_TNL_GRP_TNL_CTRL_WMASK_VAL 0x00000003
#define IAXXX_IN_TNL_GRP_TNL_CTRL_RESET_VAL 0x00000000

/*
 * Tunnel direction
 * 0 - Rx (host to device)
 * 1 - Tx (device to host)
 */
#define IAXXX_IN_TNL_GRP_TNL_CTRL_DIR_MASK 0x00000001
#define IAXXX_IN_TNL_GRP_TNL_CTRL_DIR_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_CTRL_DIR_POS 0
#define IAXXX_IN_TNL_GRP_TNL_CTRL_DIR_SIZE 1
#define IAXXX_IN_TNL_GRP_TNL_CTRL_DIR_DECL 0

/*
 * Tunnel operation mode
 * 0 - Sync (Does not repeat current frames. Halts sink till new frame is
 * available.)
 * 1 - Async (Repeats current frame when new frame is not provided in time.
 * Does not affect sink flow.)
 */
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MODE_MASK 0x00000002
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MODE_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MODE_POS 1
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MODE_SIZE 1
#define IAXXX_IN_TNL_GRP_TNL_CTRL_MODE_DECL 1

/*** IN_TNL_GRP_FORMAT (0x03000034) ***/
/*
 */
#define IAXXX_IN_TNL_GRP_FORMAT_ADDR (0x03000034)
#define IAXXX_IN_TNL_GRP_FORMAT_MASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_FORMAT_RMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_FORMAT_WMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_FORMAT_RESET_VAL 0x00000000

/*
 * Encoding of the data contained in the
 * frame buffer coming out of the endpoint
 */
#define IAXXX_IN_TNL_GRP_FORMAT_ENCODING_MASK 0x000000ff
#define IAXXX_IN_TNL_GRP_FORMAT_ENCODING_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_FORMAT_ENCODING_POS 0
#define IAXXX_IN_TNL_GRP_FORMAT_ENCODING_SIZE 8
#define IAXXX_IN_TNL_GRP_FORMAT_ENCODING_DECL (7 : 0)

/*
 * Sample rate of the data contained in the
 * frame buffer coming out of the endpoint
 */
#define IAXXX_IN_TNL_GRP_FORMAT_SAMPLE_RATE_MASK 0x0000ff00
#define IAXXX_IN_TNL_GRP_FORMAT_SAMPLE_RATE_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_FORMAT_SAMPLE_RATE_POS 8
#define IAXXX_IN_TNL_GRP_FORMAT_SAMPLE_RATE_SIZE 8
#define IAXXX_IN_TNL_GRP_FORMAT_SAMPLE_RATE_DECL (15 : 8)

/*
 * Number of bytes contained in the
 * frame buffer coming out of the endpoint.
 */
#define IAXXX_IN_TNL_GRP_FORMAT_LENGTH_MASK 0xffff0000
#define IAXXX_IN_TNL_GRP_FORMAT_LENGTH_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_FORMAT_LENGTH_POS 16
#define IAXXX_IN_TNL_GRP_FORMAT_LENGTH_SIZE 16
#define IAXXX_IN_TNL_GRP_FORMAT_LENGTH_DECL (31 : 16)

/*** IN_TNL_GRP_TNL_NFRAME_DROPS (0x03000038) ***/
/*
 * Number of Frames drops.
 */
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_ADDR (0x03000038)
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_MASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_RMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_WMASK_VAL 0x00000000
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_RESET_VAL 0x00000000

/*
 * Number of Frames drops.
 */
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_TNL_NFRAME_DROPS_CNT_MASK 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_TNL_NFRAME_DROPS_CNT_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_TNL_NFRAME_DROPS_CNT_POS 0
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_TNL_NFRAME_DROPS_CNT_SIZE 32
#define IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_TNL_NFRAME_DROPS_CNT_DECL (31 : 0)

/*** IN_TNL_GRP_TNL_NSENT_TO_HOST (0x0300003c) ***/
/*
 * Number of Frames sent to host.
 */
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_ADDR (0x0300003c)
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_MASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_RMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_WMASK_VAL 0x00000000
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_RESET_VAL 0x00000000

/*
 * Number of Frames sent to host.
 */
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_TNL_NSENT_TO_HOST_CNT_MASK 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_TNL_NSENT_TO_HOST_CNT_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_TNL_NSENT_TO_HOST_CNT_POS 0
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_TNL_NSENT_TO_HOST_CNT_SIZE 32
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TO_HOST_TNL_NSENT_TO_HOST_CNT_DECL (31 : 0)

/*** IN_TNL_GRP_TNL_NSENT (0x03000040) ***/
/*
 * Number of Frames sent.
 */
#define IAXXX_IN_TNL_GRP_TNL_NSENT_ADDR (0x03000040)
#define IAXXX_IN_TNL_GRP_TNL_NSENT_MASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_RMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_WMASK_VAL 0x00000000
#define IAXXX_IN_TNL_GRP_TNL_NSENT_RESET_VAL 0x00000000

/*
 * Number of Frames sent.
 */
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TNL_NSENT_CNT_MASK 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TNL_NSENT_CNT_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TNL_NSENT_CNT_POS 0
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TNL_NSENT_CNT_SIZE 32
#define IAXXX_IN_TNL_GRP_TNL_NSENT_TNL_NSENT_CNT_DECL (31 : 0)

/*** IN_TNL_GRP_TNL_NRECVD (0x03000044) ***/
/*
 * Number of Frames received.
 */
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_ADDR (0x03000044)
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_MASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_RMASK_VAL 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_WMASK_VAL 0x00000000
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_RESET_VAL 0x00000000

/*
 * Number of Frames received.
 */
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_TNL_NRECVD_CNT_MASK 0xffffffff
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_TNL_NRECVD_CNT_RESET_VAL 0x0
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_TNL_NRECVD_CNT_POS 0
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_TNL_NRECVD_CNT_SIZE 32
#define IAXXX_IN_TNL_GRP_TNL_NRECVD_TNL_NRECVD_CNT_DECL (31 : 0)

/* Number of registers in the module */
#define IAXXX_IN_TNL_GRP_REG_NUM 6

#endif /* __IAXXX_REGISTER_DEFS_IN_TNL_GRP_H__ */
