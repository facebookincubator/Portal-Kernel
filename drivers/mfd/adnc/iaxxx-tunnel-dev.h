/*
 *  iaxxx-tunnel-dev.h -- iaxxx Tunneling protocol info
 *
 *  Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#ifndef __IAXX_TUNNEL_DEV_PROTOCOL__
#define __IAXX_TUNNEL_DEV_PROTOCOL__

#include "iaxxx-tunnel-priv.h"

#define TNL_SYNC_MODE		0
#define TNL_ASYNC_MODE		1

/* For defining new tunnel id with Q format */
#define TNL_SRC_Q		0x00010000

#define TUNNEL_MAGIC1 0x45
#define TUNNEL_MAGIC2 0x4D
#define TUNNEL_MAGIC3 0x4F
#define TUNNEL_MAGIC4 0x52

enum iaxxx_tunnel_src_id {
	TNL_SRC_VQ_CONFIDENCE = 0x3020,
	TNL_SRC_CS_OUT1 = 0x3060,
	TNL_SRC_CS_OUT2 = 0x3061,
	TNL_SRC_DOA = 0x3062,
	TNL_SRC_CVQ = 0x30A0,
	TNL_SRC_MIC1 = 0x4020,
	TNL_SRC_MIC2 = 0x40A0,
	TNL_SRC_MIC3 = 0x4120,
	TNL_SRC_MIC4 = 0x41A0,
	TNL_SRC_AEC_REF1 = 0x4220,
	TNL_SRC_AEC_REF2 = 0x4260,
	TNL_SRC_AEC_MIXER = 0x30E0,
	TNL_SRC_MIC1_Q15 = 0x4020 | TNL_SRC_Q,
	TNL_SRC_MIC2_Q15 = 0x40A0 | TNL_SRC_Q,
	TNL_SRC_MIC3_Q15 = 0x4120 | TNL_SRC_Q,
	TNL_SRC_MIC4_Q15 = 0x41A0 | TNL_SRC_Q,
	TNL_SRC_AEC_MIXER_Q15 = 0x30E0 | TNL_SRC_Q,
	TNL_SRC_MBC = 0x3120,
	TNL_SRC_PEQ = 0x3160,
	TNL_SRC_VP_PARAM = 0x3063,
	TNL_SRC_AEC_REF1_Q15 = 0x4220 | TNL_SRC_Q,
	TNL_SRC_AEC_REF2_Q15 = 0x4260 | TNL_SRC_Q,
};

struct iaxxx_tunnel_info {
	int src;
	int mode;
	int enc;
	char *name;
};

static const struct iaxxx_tunnel_info iaxxx_tunnel_src[] = {
	{TNL_SRC_VQ_CONFIDENCE, TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "VQ CONF"},
	{TNL_SRC_CS_OUT1,       TNL_ASYNC_MODE, TNL_ENC_Q15,    "CS OUT1"},
	{TNL_SRC_CS_OUT2,       TNL_ASYNC_MODE, TNL_ENC_Q15,    "CS OUT2"},
	{TNL_SRC_DOA,           TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "DOA"},
	{TNL_SRC_CVQ,           TNL_SYNC_MODE,  TNL_ENC_Q15,    "CVQ"},
	{TNL_SRC_MIC1,          TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "MIC1"},
	{TNL_SRC_MIC2,          TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "MIC2"},
	{TNL_SRC_MIC3,          TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "MIC3"},
	{TNL_SRC_MIC4,          TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "MIC4"},
	{TNL_SRC_AEC_REF1,      TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "AECREF1"},
	{TNL_SRC_AEC_REF2,      TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "AECREF2"},
	{TNL_SRC_AEC_MIXER,     TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "AECMIX"},
	{TNL_SRC_MIC1_Q15,      TNL_ASYNC_MODE, TNL_ENC_Q15,    "MIC1 Q15"},
	{TNL_SRC_MIC2_Q15,      TNL_ASYNC_MODE, TNL_ENC_Q15,    "MIC2 Q15"},
	{TNL_SRC_MIC3_Q15,      TNL_ASYNC_MODE, TNL_ENC_Q15,    "MIC3 Q15"},
	{TNL_SRC_MIC4_Q15,      TNL_ASYNC_MODE, TNL_ENC_Q15,    "MIC4 Q15"},
	{TNL_SRC_AEC_MIXER_Q15, TNL_ASYNC_MODE, TNL_ENC_Q15,    "AECMIX Q15"},
	{TNL_SRC_MBC,           TNL_ASYNC_MODE, TNL_ENC_Q15,    "MBC Q15"},
	{TNL_SRC_PEQ,           TNL_ASYNC_MODE, TNL_ENC_Q15,    "PEQ Q15"},
	{TNL_SRC_VP_PARAM,      TNL_ASYNC_MODE, TNL_ENC_AFLOAT, "VP PARAM"},
	{TNL_SRC_AEC_REF1_Q15,	TNL_ASYNC_MODE, TNL_ENC_Q15,    "AECREF1"},
	{TNL_SRC_AEC_REF2_Q15,	TNL_ASYNC_MODE, TNL_ENC_Q15,    "AECREF2"}
};

static const uint32_t tunnel_src_all[] = {
	TNL_SRC_VQ_CONFIDENCE,
	TNL_SRC_CS_OUT1,
	TNL_SRC_CS_OUT2,
	TNL_SRC_DOA,
	TNL_SRC_MIC1,
	TNL_SRC_MIC2,
	TNL_SRC_MIC3,
	TNL_SRC_MIC4,
	TNL_SRC_AEC_MIXER,
};

static const uint32_t tunnel_src_debug_mono[] = {
	TNL_SRC_CS_OUT1,
	TNL_SRC_CS_OUT2,
	TNL_SRC_DOA,
	TNL_SRC_MIC1_Q15,
	TNL_SRC_MIC2_Q15,
	TNL_SRC_MIC3_Q15,
	TNL_SRC_MIC4_Q15,
	TNL_SRC_AEC_MIXER_Q15,
	TNL_SRC_VP_PARAM,
	TNL_SRC_AEC_REF1_Q15,
};

static const uint32_t tunnel_src_debug_stereo[] = {
	TNL_SRC_CS_OUT1,
	TNL_SRC_CS_OUT2,
	TNL_SRC_DOA,
	TNL_SRC_MIC1_Q15,
	TNL_SRC_MIC2_Q15,
	TNL_SRC_MIC3_Q15,
	TNL_SRC_MIC4_Q15,
	TNL_SRC_VP_PARAM,
	TNL_SRC_AEC_REF1_Q15,
	TNL_SRC_AEC_REF2_Q15,
};

static const uint32_t tunnel_src_meta[] = {
	TNL_SRC_VQ_CONFIDENCE,
	TNL_SRC_DOA,
};

static const uint32_t tunnel_src_cvq[] = {
	TNL_SRC_CVQ,
};

#endif /* __IAXX_TUNNEL_DEV_PROTOCOL__ */
