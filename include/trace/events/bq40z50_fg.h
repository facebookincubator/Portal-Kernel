#undef TRACE_SYSTEM
#define TRACE_SYSTEM bq40z50_fg

#if !defined(_TRACE_BQ40Z50_FG_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BQ40Z50_FG_H

#include <linux/tracepoint.h>

TRACE_EVENT(read_rsoc,

	TP_PROTO(int rsoc),

	TP_ARGS(rsoc),

	TP_STRUCT__entry(__field(int, rsoc)),

	TP_fast_assign(__entry->rsoc = rsoc;),

	TP_printk("rsoc value = %d", __entry->rsoc)
);

TRACE_EVENT(read_safety_status,

	TP_PROTO(int ss),

	TP_ARGS(ss),

	TP_STRUCT__entry(__field(int, ss)),

	TP_fast_assign(__entry->ss = ss;),

	TP_printk("safety status value = %d", __entry->ss)
);

TRACE_EVENT(read_pf_status,

	TP_PROTO(int pf),

	TP_ARGS(pf),

	TP_STRUCT__entry(__field(int, pf)),

	TP_fast_assign(__entry->pf = pf;),

	TP_printk("pf status value = %d", __entry->pf)
);

TRACE_EVENT(read_operation_status,

	TP_PROTO(int operation_status),

	TP_ARGS(operation_status),

	TP_STRUCT__entry(__field(int, operation_status)),

	TP_fast_assign(__entry->operation_status = operation_status;),

	TP_printk("operation status value = %x", __entry->operation_status)
);

TRACE_EVENT(read_charging_status,

	TP_PROTO(int charge_status),

	TP_ARGS(charge_status),

	TP_STRUCT__entry(__field(int, charge_status)),

	TP_fast_assign(__entry->charge_status = charge_status;),

	TP_printk("charging status value = %x", __entry->charge_status)
);


#endif /* _TRACE_BQ40Z50_FG_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
