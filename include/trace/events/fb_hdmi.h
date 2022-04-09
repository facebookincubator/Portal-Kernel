#undef TRACE_SYSTEM
#define TRACE_SYSTEM fb_hdmi

#if !defined(_FB_HDMI_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _FB_HDMI_H_

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(fb_state,
	TP_PROTO(int val),
	TP_ARGS(val),
	TP_STRUCT__entry(
		__field(int, val)
	),
	TP_fast_assign(
		__entry->val = val;
	),

	TP_printk("%d", __entry->val)
);

DEFINE_EVENT(fb_state, fb_hdmi_connect,
	TP_PROTO(int connected),
	TP_ARGS(connected)
);

DEFINE_EVENT(fb_state, fb_hdmi_polarity,
	TP_PROTO(int on),
	TP_ARGS(on)
);

DEFINE_EVENT(fb_state, fb_hdmi_tx_off,
	TP_PROTO(int on),
	TP_ARGS(on)
);

TRACE_EVENT(fb_hdmi_tx_on,
	TP_PROTO(u32 active_h, u32 active_v, u32 interlaced,
		u32 refresh_rate, u32 pixel_freq, u32 is_yuv,
		u32 bpp, u32 dc_enable, u32 hdr_enabled, u32 video_format),
	TP_ARGS(active_h, active_v, interlaced, refresh_rate, pixel_freq,
		is_yuv, bpp, dc_enable, hdr_enabled, video_format),
	TP_STRUCT__entry(
			__field(u32, active_h)
			__field(u32, active_v)
			__field(u32, interlaced)
			__field(u32, refresh_rate)
			__field(u32, pixel_freq)
			__field(u32, is_yuv)
			__field(u32, bpp)
			__field(u32, dc_enable)
			__field(u32, hdr_enabled)
			__field(u32, video_format)
	),
	TP_fast_assign(
			__entry->active_h = active_h;
			__entry->active_v = active_v;
			__entry->interlaced = interlaced;
			__entry->refresh_rate = refresh_rate;
			__entry->pixel_freq = pixel_freq;
			__entry->is_yuv = is_yuv;
			__entry->bpp = bpp;
			__entry->dc_enable = dc_enable;
			__entry->hdr_enabled = hdr_enabled;
			__entry->video_format = video_format;
	),

	TP_printk("%dx%d%s@%dHz %dKHz %s %dbpp dc(%d) hdr(%d) (%d)",
			__entry->active_h, __entry->active_v,
			__entry->interlaced ? "i" : "p",
			__entry->refresh_rate, __entry->pixel_freq,
			__entry->is_yuv ? "Y420" : "RGB",
			__entry->bpp,
			__entry->dc_enable,
			__entry->hdr_enabled,
			__entry->video_format)
);

#endif

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE fb_hdmi
#undef TRACE_INCLUDE_PATH

/* This part must be outside protection */
#include <trace/define_trace.h>
