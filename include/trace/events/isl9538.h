#undef TRACE_SYSTEM
#define TRACE_SYSTEM isl9538

#if !defined(_TRACE_ISL9538_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ISL9538_H

#include <linux/tracepoint.h>

TRACE_EVENT(is_charger_present,

	TP_PROTO(bool connected, int vad_voltage, int est_adapter_voltage),

	TP_ARGS(connected, vad_voltage, est_adapter_voltage),

	TP_STRUCT__entry(
			__field(bool, connected)
			__field(int, vad_voltage)
			__field(int, est_adapter_voltage)
	),

	TP_fast_assign(
			__entry->connected = connected;
			__entry->vad_voltage = vad_voltage;
			__entry->est_adapter_voltage = est_adapter_voltage;
	),

	TP_printk("charger connected=%hhu, vad_voltage=%d, "
				 "est_adapter_voltage=%d",
				__entry->connected,
				__entry->vad_voltage,
				__entry->est_adapter_voltage)
);

TRACE_EVENT(enable_charging,

	TP_PROTO(bool charging),

	TP_ARGS(charging),

	TP_STRUCT__entry(__field(bool, charging)),

	TP_fast_assign(__entry->charging = charging;),

	TP_printk("charger enabled=%s", __entry->charging ? "true" : "false")
);

TRACE_EVENT(enable_vsys,

	TP_PROTO(bool vsys_sts),

	TP_ARGS(vsys_sts),

	TP_STRUCT__entry(__field(bool, vsys_sts)),

	TP_fast_assign(__entry->vsys_sts = vsys_sts;),

	TP_printk("charger vsys_sts=%s",
		__entry->vsys_sts ? "enabled" : "disabled")
);

TRACE_EVENT(set_charge_voltage,

	TP_PROTO(int volt),

	TP_ARGS(volt),

	TP_STRUCT__entry(__field(int, volt)),

	TP_fast_assign(__entry->volt = volt;),

	TP_printk("set charge voltage, volt=%d", __entry->volt)
);

TRACE_EVENT(set_charge_current,

	TP_PROTO(int curr),

	TP_ARGS(curr),

	TP_STRUCT__entry(__field(int, curr)),

	TP_fast_assign(__entry->curr = curr;),

	TP_printk("set charge current, current=%d", __entry->curr)
);

TRACE_EVENT(upa_monitor_work,

	TP_PROTO(int vadc_uv, int adapter_uv, int min_charge_uv,
			bool upa_disable_charging, bool notif_upa),

	TP_ARGS(vadc_uv, adapter_uv, min_charge_uv,
			upa_disable_charging, notif_upa),

	TP_STRUCT__entry(
			__field(int, vadc_uv);
			__field(int, adapter_uv)
			__field(int, min_charge_uv)
			__field(bool, upa_disable_charging)
			__field(bool, notif_upa)
	),

	TP_fast_assign(
			__entry->vadc_uv = vadc_uv;
			__entry->adapter_uv = adapter_uv;
			__entry->min_charge_uv = min_charge_uv;
			__entry->upa_disable_charging = upa_disable_charging;
			__entry->notif_upa = notif_upa;
	),

	TP_printk("vadc_uv=%d, adapter_uv=%d, min_charge_uv=%d, "
				"upa_disable_charging=%hhu notif_upa=%hhu",
				__entry->vadc_uv,
				__entry->adapter_uv,
				__entry->min_charge_uv,
				__entry->upa_disable_charging,
				__entry->notif_upa
				)
);

#endif /* _TRACE_ISL9538_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

