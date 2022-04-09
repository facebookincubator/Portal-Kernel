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

enum fb_dspg_events {
	FB_DSPG_EVENT_FW_CRASH = 0x01,
	FB_DSPG_EVENT_FW_RECOVERY,
	FB_DSPG_EVENT_FW_REQUEST,
	FB_DSPG_EVENT_FW_ERROR,
};

enum fb_dspg_status {
	FB_DSPG_STAT_SUCCESS = 0x0,
	FB_DSPG_STAT_FAILED,
};

union fb_dspg_event {
	int32_t value;
	struct {
		int32_t event:8;		/* enum fb_dspg_events */
		int32_t status:8;		/* enum fb_dspg_status */
		int16_t param:16;
	};
} __packed;

enum fb_stax_device_id {
	FBE_STAX_DEVICE_ADSP,
	FBE_STAX_DEVICE_DSPG,
	FBE_STAX_DEVICE_HIFI
};

enum fb_stax_events {
	FBE_STAX_AWB_LOAD_COMPLETE,
};

union fb_stax_event {
	int32_t value;
	struct {
		int16_t event:8;		/* enum fb_stax_events */
		int16_t device_id:4;		/* enum fb_stax_device_id */
	};
} __packed;

enum fb_adsp_event_id {
	/* Playback direction */
	FB_ADSP_EVENT_TX_OVERFLOW,	/* More filled buffers */
	FB_ADSP_EVENT_TX_UNDERFLOW,	/* Insufficient filled buffers */

	/* Capture direction */
	FB_ADSP_EVENT_RX_OVERFLOW,	/* Insufficient empty buffer */
	FB_ADSP_EVENT_RX_UNDERFLOW,	/* More empty buffers but no data */
	FB_ADSP_EVENT_QDSP_ERROR,	/* Internal QDSP error state */
	FB_ADSP_EVENT_QDSP_DOWN,	/* An ADSP down/crash event field */
	FB_ADSP_EVENT_QDSP_UP,		/* An ADSP up/recovery event field */
	FB_ADSP_EVENT_MAX,		/* Helper for event range restriction */
};

union fb_adsp_event {
	int32_t value;
	struct {
		int32_t event:8;	/* enum fb_adsp_event_id */
		int32_t dir:8;		/* 0: PLAYBACK, 1: CAPTURE */
		int32_t pcm:8;		/* PCM device ID */
		int32_t be_id:8;	/* Backend ID */
	};
	/* Polymorphic version for ADSP internal errors */
	struct {
		int32_t event_err:8;	/* enum fb_adsp_event_id */
		int32_t state_err:24;	/* adsp error state */
	};
} __packed;

/* Used for the FB_ADSP_EVENT_QDSP_DOWN and FB_ADSP_EVENT_QDSP_UP events */
enum fb_adsp_fw_states {
	FB_ADSP_FW_UNKNOWN = 0,	/* Unknown FW state, not logged */
	FB_ADSP_FW_CRASH,	/* An ADSP FW crash state */
	FB_ADSP_FW_RECOVERY,	/* An ADSP FW recovery state */
	FB_ADSP_FW_STARTUP,	/* An ADSP FW intial startup state */
	FB_ADSP_FW_MAX,		/* Helper for FW state range restriction */
};

enum fb_event_type {
	FBE_NONE = 0,
	FBE_TAS5760_FAULTZ,
	FBE_IAXXX_CRASH,
	FBE_IAXXX_FW_LD_FAIL,
	FBE_DBMDX,
	FBE_ADSP,
	/* Software event used to pass STAX algo specific
	 * information to event collector
	 */
	FBE_STAX,
	FBE_MAX,
};

struct fb_event {
	int64_t tv_sec;
	int64_t tv_usec;
	int32_t type;
	int32_t value;
} __packed;

#endif /* _FB_EVENT_UAPI_H_ */
