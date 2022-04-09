/*
* iaxxx-register-defs-af.h -- IAXXX register defination for AF
*
* Copyright (c) 2016 Knowles Corporation.
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

#ifndef __IAXXX_REGISTER_DEFS_AF_H__
#define __IAXXX_REGISTER_DEFS_AF_H__

/*** IAXXX_AF_WCPT_WALL_CLOCK_RD_0 (0x40000140) ***/
/* Lower bits to read for the Wall Clock Value.*/
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR (0x40000140)
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_MASK_VAL 0xffffffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_RMASK_VAL 0xffffffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_WMASK_VAL 0x00000000
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_RESET_VAL 0x00000000

/* Wall Clock Value to read (Lower bits) */
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_VAL_MASK 0xffffffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_VAL_RESET_VAL 0x0
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_VAL_POS 0
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_VAL_SIZE 32
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_0_VAL_DECL 32

/*** IAXXX_AF_WCPT_WALL_CLOCK_RD_1 (0x40000144) ***/
/* Upper bits to read for the Wall Clock Value. */
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_ADDR (0x40000144)
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_MASK_VAL 0x000fffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_RMASK_VAL 0x000fffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_WMASK_VAL 0x00000000
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_RESET_VAL 0x00000000

/* Wall Clock Value to read (Upper bits).
 * The write to this field triggers the
 * transfer of {WCPT_WALL_CLOCK_1, WCPT_WALL_CLOCK_0}.
 */
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_VAL_MASK 0x000fffff
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_VAL_RESET_VAL 0x0
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_VAL_POS 0
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_VAL_SIZE 20
#define IAXXX_AF_WCPT_WALL_CLOCK_RD_1_VAL_DECL 20

#endif /* __IAXXX_REGISTER_DEFS_AF_H__ */
