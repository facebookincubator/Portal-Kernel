/*
 * Fbfg fuel gauge driver trace
 *
 * Copyright (c) 2021, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM fbfg_battery

#if !defined(_TRACE_FBFG_BATTERY_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FBFG_BATTERY_H

#include <linux/tracepoint.h>

TRACE_EVENT(fbfg_update_status,

	TP_PROTO(int rsoc, int dsoc, int chg_curr, int chg_volt,
		int curr_now, int volt_now, int temp),

	TP_ARGS(rsoc, dsoc, chg_curr, chg_volt, curr_now, volt_now, temp),

	TP_STRUCT__entry(
		__field(int, rsoc)
		__field(int, dsoc)
		__field(int, chg_curr)
		__field(int, chg_volt)
		__field(int, curr_now)
		__field(int, volt_now)
		__field(int, temp)
	),

	TP_fast_assign(
		__entry->rsoc = rsoc;
		__entry->dsoc = dsoc;
		__entry->chg_curr = chg_curr;
		__entry->chg_volt = chg_volt;
		__entry->curr_now = curr_now;
		__entry->volt_now = volt_now;
		__entry->temp = temp;
	),

	TP_printk("RSOC:%d DSOC:%d ChgVolt:%d ChgCurr:%d "
		"Curr:%d Volt:%d Temp:%d",
		__entry->rsoc,
		__entry->dsoc,
		__entry->chg_curr,
		__entry->chg_volt,
		__entry->curr_now,
		__entry->volt_now,
		__entry->temp)
);

TRACE_EVENT(fbfg_safety_status,

	TP_PROTO(int safety_status, int op_status, int gauge_alert),

	TP_ARGS(safety_status, op_status, gauge_alert),

	TP_STRUCT__entry(
		__field(int, safety_status)
		__field(int, op_status)
		__field(int, gauge_alert)
	),

	TP_fast_assign(
		__entry->safety_status = safety_status;
		__entry->op_status = op_status;
		__entry->gauge_alert = gauge_alert;
	),

	TP_printk("SafetyStatus:0x%04x OP_STS:0x%04x Gauge_Alert:0x%04x",
		__entry->safety_status,
		__entry->op_status,
		__entry->gauge_alert)
);

#endif /* _TRACE_FBFG_BATTERY_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
