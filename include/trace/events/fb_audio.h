#undef TRACE_SYSTEM
#define TRACE_SYSTEM fb_audio

#if !defined(_FB_AUDIO_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _FB_AUDIO_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(fba_voice_trigger,

	TP_PROTO(unsigned long count),

	TP_ARGS(count),

	TP_STRUCT__entry(
		__field(unsigned long, count)
	),

	TP_fast_assign(
		__entry->count = count;
	),

	TP_printk("count = %lu", __entry->count)
);

TRACE_EVENT(fba_fw_crash,

	TP_PROTO(char *reason),

	TP_ARGS(reason),

	TP_STRUCT__entry(
		__field(char*, reason)
	),

	TP_fast_assign(
		__entry->reason = reason;
	),

	TP_printk("%s", __entry->reason)
);

DECLARE_EVENT_CLASS(fba_state,

	TP_PROTO(int on),

	TP_ARGS(on),

	TP_STRUCT__entry(
		__field(int, on)
	),

	TP_fast_assign(
		__entry->on = on;
	),

	TP_printk("on = %d", __entry->on)
);

DEFINE_EVENT(fba_state, fba_producer_state,
	TP_PROTO(int on),
	TP_ARGS(on)
);

DEFINE_EVENT(fba_state, fba_debug_dump_state,
	TP_PROTO(int on),
	TP_ARGS(on)
);

DEFINE_EVENT(fba_state, fba_alarm_state,
	TP_PROTO(int on),
	TP_ARGS(on)
);

TRACE_EVENT(fba_usecase,

	TP_PROTO(int usecase),

	TP_ARGS(usecase),

	TP_STRUCT__entry(
		__field(int, usecase)
	),

	TP_fast_assign(
		__entry->usecase = usecase;
	),

	TP_printk("%d", __entry->usecase)
);

#endif /* #define _FB_AUDIO_TRACE_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>

