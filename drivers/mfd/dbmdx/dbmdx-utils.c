/*
 * dbmdx-utils.c - DSPG DBMDX Common Utils
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <linux/version.h>


#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-utils.h>


u64 bytearr_to_u64(u8 *buf)
{
	return  ((u64)(buf[0])) | ((u64)(buf[1]) << 8) |
		((u64)(buf[2]) << 16) | ((u64)(buf[3]) << 24) |
		((u64)(buf[4]) << 32) | ((u64)(buf[5]) << 40) |
		((u64)(buf[6]) << 48) | ((u64)(buf[7]) << 56);
}

u32 bytearr_to_u32(u8 *buf)
{
	return  ((u64)(buf[0])) | ((u64)(buf[1]) << 8) |
		((u64)(buf[2]) << 16) | ((u64)(buf[3]) << 24);
}

u16 bytearr_to_u16(u8 *buf)
{
	return  ((u64)(buf[0])) | ((u64)(buf[1]) << 8);
}
