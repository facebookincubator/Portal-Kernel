/*
 * Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#ifndef _FB_EVENT_UAPI_H_
#define _FB_EVENT_UAPI_H_

#include <linux/time.h>

/* Register Interrupt status */
#define TAS5760_IRQ_STATUS_OTE_SHIFT    0
#define TAS5760_IRQ_STATUS_OTE_MASK     (1 << TAS5760_IRQ_STATUS_OTE_SHIFT)

#define TAS5760_IRQ_STATUS_DCE_SHIFT    1
#define TAS5760_IRQ_STATUS_DCE_MASK     (1 << TAS5760_IRQ_STATUS_DCE_SHIFT)

#define TAS5760_IRQ_STATUS_OCE_SHIFT    2
#define TAS5760_IRQ_STATUS_OCE_MASK     (1 << TAS5760_IRQ_STATUS_OCE_SHIFT)

#define TAS5760_IRQ_STATUS_CLKE_SHIFT   3
#define TAS5760_IRQ_STATUS_CLKE_MASK    (1 << TAS5760_IRQ_STATUS_CLKE_SHIFT)

#define TAS5760_IRQ_STATUS_OCET_SHIFT   4
#define TAS5760_IRQ_STATUS_OCET_MASK    (3 << TAS5760_IRQ_STATUS_OCET_SHIFT)

union fb_iaxxx_fw_crash {
	int32_t value;
	struct {
		int32_t status:8;	/* IAXXX_RECOVERY_xxxx */
		int32_t reasons:8;	/* IAXXX_FW_CRASH_xxx */
		int32_t try_count:8;	/* count */
	};
} __packed;

enum fb_event_type {
	FBE_NONE = 0,
	FBE_TAS5760_FAULTZ,
	FBE_IAXXX_CRASH,
	FBE_IAXXX_FW_LD_FAIL,
};

struct fb_event {
	int64_t tv_sec;
	int64_t tv_usec;
	int32_t type;
	int32_t value;
} __packed;

#endif /* _FB_EVENT_UAPI_H_ */
