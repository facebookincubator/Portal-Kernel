/*
 * es705-routes.h  --  Audience eS705 ALSA SoC Audio driver
 *
 * Copyright 2013 Audience, Inc.
 *
 * Author: Greg Clemson <gclemson@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES705_ROUTES_H
#define _ES705_ROUTES_H

struct esxxx_route_config {
	const u32 *route;
	const u32 *nb;
	const u32 *wb;
	const u32 *swb;
	const u32 *fb;
};

enum {
	ROUTE_OFF,						/* 0 */
	ROUTE_CS_VOICE_1MIC_CT,			/* 1 */
	ROUTE_CS_VOICE_2MIC_CT,			/* 2 */
	ROUTE_CS_VOICE_3MIC_CT,			/* 3 */
	ROUTE_CS_VOICE_1MIC_FT,			/* 4 */
	ROUTE_CS_VOICE_2MIC_FT,			/* 5 */
	ROUTE_CS_VOICE_3MIC_FT,			/* 6 */
	ROUTE_CS_VOICE_HEADSET,			/* 7 */
	ROUTE_CS_VOICE_1MIC_HEADPHONE,	/* 8 */
	ROUTE_VOIP_1MIC_CT,				/* 9 */
	ROUTE_VOIP_2MIC_CT,				/* 10 */
	ROUTE_VOIP_3MIC_CT,				/* 11 */
	ROUTE_VOIP_1MIC_FT,				/* 12 */
	ROUTE_VOIP_2MIC_FT,				/* 13 */
	ROUTE_VOIP_3MIC_FT,				/* 14 */
	ROUTE_VOIP_HEADSET,				/* 15 */
	ROUTE_VOIP_1MIC_HEADPHONE,		/* 16 */
	ROUTE_VOICE_ASR_1MIC,			/* 17 */
	ROUTE_VOICE_ASR_2MIC,			/* 18 */
	ROUTE_VOICE_ASR_3MIC,			/* 19 */
	ROUTE_VOICESENSE_SBUSRX4,		/* 20 */
	ROUTE_VOICESENSE_SBUSRX0,		/* 21 */
	ROUTE_VOICESENSE_PDM,			/* 22 */
	ROUTE_1CHAN_PLAYBACK,			/* 23 */
	ROUTE_2CHAN_PLAYBACK,			/* 24 */
	ROUTE_1CHAN_CAPTURE,			/* 25 */
	ROUTE_2CHAN_CAPTURE,			/* 26 */
	ROUTE_AUDIOFOCUS_2MIC,			/* 27 */
	ROUTE_AUDIOFOCUS_3MIC,			/* 28 */
	ROUTE_2MIC_NS_CT_ANALOG,		/* 29 */
	ROUTE_2MIC_NS_FT_ANALOG,		/* 30 */
	ROUTE_2MIC_NS_FO_ANALOG,		/* 31 */
	ROUTE_3MIC_NS_CT_ANALOG,		/* 32 */
	ROUTE_3MIC_NS_FT_ANALOG,		/* 33 */
	ROUTE_3MIC_NS_FO_ANALOG,		/* 34 */
	ROUTE_AEC7_2MIC_NS_FT,			/* 35 */
	ROUTE_AEC7_3MIC_NS_FT,			/* 36 */
	ROUTE_ASR_2MIC_NS_AF,			/* 37 */
	ROUTE_ASR_3MIC_NS_AF,			/* 38 */
	ROUTE_AZV_2MIC,					/* 39 */
	ROUTE_AZV_3MIC,					/* 40 */
	ROUTE_STEREO_RECORD_48K,		/* 41 */
	ROUTE_STEREO_RECORD_96K,		/* 42 */
	ROUTE_STEREO_RECORD_192K,		/* 43 */
	ROUTE_AVALON_PLAY,				/* 44 */
	ROUTE_AVALON_CAPTURE,			/* 45 */
	ROUTE_ASR_1MIC_NS_AF_WB,		/* 46 */
	ROUTE_ASR_2MIC_NS_AF_WB,		/* 47 */
	ROUTE_VOICESENSE_PDM_16K,		/* 48 */
	ROUTE_TRAINING_KEYWORD_8K,		/* 49 */
	ROUTE_TRAINING_KEYWORD_16K,		/* 50 */
	ROUTE_ASR_3MIC_SLIM_SDE_24K,		/* 51 */
	ROUTE_ASR_3MIC_SLIM_SDE_48K,		/* 52 */
	ROUTE_PT_PDM_CAP_16K,			/* 53 */
	ROUTE_3CHAN_PCM_PT,				/* 54 */
	ROUTE_1CHAN_CAP_PCM_PORTA_48K,			/* 55 */
	ROUTE_VOICE_1MIC_PORTA,				/* 56 */
	ROUTE_VOIP_3MIC,				/* 57 */
	ROUTE_VP_ALGO_OFF,				/* 58 */
	ROUTE_BARGEIN,					/* 59 */
	ROUTE_VQ,					/* 60 */
	ROUTE_MAX						/* 61 */
};

enum {
	RATE_NB,
	RATE_WB,
	RATE_SWB,
	RATE_FB,
	RATE_MAX
};

static const u32 route_off[] = {
	0xffffffff,
};

static const u32 pxx_default_power_preset[] = {
	0x90311388, /* 5000 - Def */
	0xffffffff	/* terminate */
};
static const u32 route_cs_voice_1mic_ct[] = {
	0x903103e9,
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_1mic_ct[] = {
	0x90310227, /* 551 */
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_1mic_ct[] = {
	0x9031024f, /* 591 */
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_1mic_ct[] = {
	0x90310250, /* 592 */
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_2mic_ct[] = {
	0x9031041b,	/* without PDM  */
/*	0x90310577,	 1399 with PDM*/
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_2mic_ct[] = {
	0x9031022a, /* 554 */
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_2mic_ct[] = {
	0x9031022b, /* 555 */
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_2mic_ct[] = {
	0x9031022c, /* 556 */
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_3mic_ct[] = {
	0x903157d,	/* 1405, 3 mic, 1PDM-2Analog */
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_3mic_ct[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_3mic_ct[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_3mic_ct[] = {
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_1mic_ft[] = {
	0x903103e9,
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_1mic_ft[] = {
	0x90310230, /* 560 */
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_1mic_ft[] = {
	0x9031024f, /* 591 */
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_1mic_ft[] = {
	0x90310250, /* 592 */
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_2mic_ft[] = {
	0x9031041b,	/* Analog Mic */
/*	0x90310577,	1399, 2mic, PDM mic */
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_2mic_ft[] = {
	0x9031023c, /* 572 */
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_2mic_ft[] = {
	0x9031023a, /* 570 */
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_2mic_ft[] = {
	0x9031023b, /*571 */
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_3mic_ft[] = {
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_3mic_ft[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_3mic_ft[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_3mic_ft[] = {
	0xffffffff	/* terminate */
};

static const u32 route_cs_voice_headset[] = {
	0x903103ed,
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_headset[] = {
	0x9031024e,
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_headset[] = {
	0x9031024f,
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_headset[] = {
	0xffffffff	/* terminate */
};
static const u32 route_cs_voice_1mic_headphone[] = {
	0x903103ed,
	0xffffffff	/* terminate */
};
static const u32 nb_cs_voice_1mic_headphone[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_cs_voice_1mic_headphone[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_cs_voice_1mic_headphone[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voip_1mic_ct[] = {
	0x903103e9,
	0xffffffff	/* terminate */
};
static const u32 nb_voip_1mic_ct[] = {
	0x9031024e,
	0xffffffff	/* terminate */
};
static const u32 wb_voip_1mic_ct[] = {
	0x9031024f,
	0xffffffff	/* terminate */
};
static const u32 swb_voip_1mic_ct[] = {
	0x90310250,
	0xffffffff	/* terminate */
};

static const u32 route_voip_2mic_ct[] = {
	/*	0x9031041b,     without PDM  */
	0x90310577,	/* 1399 with PDM*/
	0xffffffff	/* terminate */
};
static const u32 nb_voip_2mic_ct[] = {
	0x9031022a, /* 554 */
	0xffffffff	/* terminate */
};
static const u32 wb_voip_2mic_ct[] = {
	0x9031022b, /* 555 */
	0xffffffff	/* terminate */
};
static const u32 swb_voip_2mic_ct[] = {
	0x9031022c,
	0xffffffff	/* terminate */
};
static const u32 fb_voip_2mic_ct[] = {
	0x90310272, /* 626 */
	0xffffffff	/* terminate */
};

static const u32 route_voip_3mic_ct[] = {
	0x9031057d,	/* 1405, 3 mic, 1PDM-2Analog */
	0xffffffff	/* terminate */
};
static const u32 nb_voip_3mic_ct[] = {
	0x9031022d, /* 557 */
	0xffffffff	/* terminate */
};
static const u32 wb_voip_3mic_ct[] = {
	0x9031022e, /* 558 */
	0xffffffff	/* terminate */
};
static const u32 swb_voip_3mic_ct[] = {
	0x9031022f, /* 559 */
	0xffffffff	/* terminate */
};
static const u32 fb_voip_3mic_ct[] = {
	0x90310273, /* 627 */
	0xffffffff	/* terminate */
};

static const u32 route_voip_1mic_ft[] = {
	0x903103e9,
	0xffffffff	/* terminate */
};
static const u32 nb_voip_1mic_ft[] = {
	0x9031024e,
	0xffffffff	/* terminate */
};
static const u32 wb_voip_1mic_ft[] = {
	0x9031024f,
	0xffffffff	/* terminate */
};
static const u32 swb_voip_1mic_ft[] = {
	0x90310250,
	0xffffffff	/* terminate */
};

static const u32 route_voip_2mic_ft[] = {
/*	0x9031041b, Analog Mic */
	0x90310577, /* 1399, 2mic, PDM mic */
	0xffffffff	/* terminate */
};
static const u32 nb_voip_2mic_ft[] = {
	0x90310239, /* 570 */
	0xffffffff	/* terminate */
};
static const u32 wb_voip_2mic_ft[] = {
	0x9031023a, /* 571 */
	0xffffffff	/* terminate */
};
static const u32 swb_voip_2mic_ft[] = {
	0x9031023b, /* 572 */
	0xffffffff	/* terminate */
};
static const u32 fb_voip_2mic_ft[] = {
	0x90310277, /* 631 */
	0xffffffff	/* terminate */
};

static const u32 route_voip_3mic_ft[] = {
	0x90310595,	/* 1429, 3 mic, 1PDM-2Analog, Ter is PDM Mic */
	0xffffffff	/* terminate */
};
static const u32 nb_voip_3mic_ft[] = {
	0x90310242, /* 578 */
	0xffffffff	/* terminate */
};
static const u32 wb_voip_3mic_ft[] = {
	0x90310243, /* 579 */
	0xffffffff	/* terminate */
};
static const u32 swb_voip_3mic_ft[] = {
	0x90310244, /* 580 */
	0xffffffff	/* terminate */
};
static const u32 fb_voip_3mic_ft[] = {
	0x9031027a, /* 634 */
	0xffffffff	/* terminate */
};
static const u32 route_voip_headset[] = {
	0x903103ed,
	0xffffffff	/* terminate */
};
static const u32 nb_voip_headset[] = {
	0x9031024e, /* 590 */
	0xffffffff	/* terminate */
};
static const u32 wb_voip_headset[] = {
	0x9031024f, /* 591 */
	0xffffffff	/* terminate */
};
static const u32 swb_voip_headset[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voip_1mic_headphone[] = {
	0x903103ed,
	0xffffffff	/* terminate */
};
static const u32 nb_voip_1mic_headphone[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_voip_1mic_headphone[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_voip_1mic_headphone[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voice_asr_1mic[] = {
	0x903104b1,
	0xffffffff	/* terminate */
};
static const u32 nb_voice_asr_1mic[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_voice_asr_1mic[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_voice_asr_1mic[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voice_asr_2mic[] = {
	0x903104ca,
	0xffffffff	/* terminate */
};
static const u32 nb_voice_asr_2mic[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_voice_asr_2mic[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_voice_asr_2mic[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voice_asr_3mic[] = {
	0xffffffff	/* terminate */
};
static const u32 nb_voice_asr_3mic[] = {
	0xffffffff	/* terminate */
};
static const u32 wb_voice_asr_3mic[] = {
	0xffffffff	/* terminate */
};
static const u32 swb_voice_asr_3mic[] = {
	0xffffffff	/* terminate */
};

static const u32 route_voicesense_sbusrx4[] = {
	0xb05a0a40,	/* SBUS.Rx4 -> RxChMgr0*/
	0xb05b0000,	/* Set PathId  RxChMgr0 PATH_ID_PRI */
	0xb0640038,	/* connect RxChMgr0.o0 */
	0xb0640190,	/* connect senseVoice.i0 */
	0xb0630003,	/* set rate RxChMgr0 8k*/
	0xb0630019,	/* set rate senseVoice 8k*/
	0xb0680300,	/* set group RxChMgr0 0*/
	0xb0681900,	/* set group sensevoice 0*/
	0xb00c0900,	/* setDeviceParamID set Multichannel Link Bits*/
	0x900d0000,	/* setDeviceParam Slimbus Linked*/
	0xffffffff	/* terminate */
};

static const u32 route_voicesense_pdm[] = {
	0x90311f40,	 /* 8k route */
	0x90312008,
	0xffffffff      /* terminate */
};

static const u32 route_voicesense_sbusrx0[] = {
	0xb05a0a00,	/* SBUS.Rx0 -> RxChMgr0*/
	0xb05b0000,	/* Set PathId  RxChMgr0 PATH_ID_PRI */
	0xb0640038,	/* connect RxChMgr0.o0 */
	0xb0640190,	/* connect senseVoice.i0 */
	0xb0630003,	/* set rate RxChMgr0 8k*/
	0xb0630019,	/* set rate senseVoice 8k*/
	0xb0680300,	/* set group RxChMgr0 0*/
	0xb0681900,	/* set group sensevoice 0*/
	0xb00c0900,	/* setDeviceParamID set Multichannel Link Bits*/
	0x900d0000,	/* setDeviceParam Slimbus Linked*/
	0xffffffff	/* terminate */
};

static const u32 route_1chan_playback[] = {
	0x9031047f,
	0xffffffff	/* terminate */
};

static const u32 route_2chan_playback[] = {
	0x90310483,
	0xffffffff	/* terminate */
};

static const u32 route_1chan_capture[] = {
	0x90310480,
	0xffffffff	/* terminate */
};

static const u32 route_2chan_capture[] = {
	0x90310484,
	0xffffffff	/* terminate */
};

static const u32 route_audiofocus_2mic[] = {
	0x9031054b,
	0xffffffff	/* terminate */
};

static const u32 route_audiofocus_3mic[] = {
	0x9031054f,
	0xffffffff	/* terminate */
};

static const u32 route_2mic_ns_ct_analog[] = {
	0x9031041b, /* 1051 */
	0x9031022A, /* 554 */
	0xffffffff	/* terminate */
};

static const u32 route_2mic_ns_ft_analog[] = {
	0x9031041b, /* 1051 */
	0x90310239, /* 569 */
	0xffffffff	/* terminate */
};

static const u32 route_2mic_ns_fo_analog[] = {
	0x9031041b, /* 1051 */
	0x90310391, /* 913 */
	0xffffffff	/* terminate */
};

static const u32 route_voip_3mic[] = {
	0x9031044d, /* 1101 */
	0xffffffff	/* terminate */
};

static const u32 route_3mic_ns_ct_analog[] = {
	0x9031044d, /* 1101 */
	0x9031022D, /* 557 */
	0xffffffff	/* terminate */
};

static const u32 route_3mic_ns_ft_analog[] = {
	0x9031044d, /* 1101 */
	0x90310242, /* 578 */
	0xffffffff	/* terminate */
};

static const u32 route_3mic_ns_fo_analog[] = {
	0x9031044d, /* 1101 */
	0x9031039d, /* 925 */
	0xffffffff	/* terminate */
};

static const u32 route_aec7_2mic_ns_ft[] = {
	0x9031041b, /* 1051 */
	0x90310239, /* 569 */
	0xffffffff	/* terminate */
};

static const u32 route_aec7_3mic_ns_ft[] = {
	0xffffffff	/* terminate */
};

static const u32 route_asr_2mic_ns_af[] = {
	0x903104ca, /* 1226 */
	0x903103b1, /* 945 */
	0xffffffff	/* terminate */
};

static const u32 route_asr_3mic_ns_af[] = {
	0x903104e3, /* 1251 */
	0x903103b9, /* 953 */
	0xffffffff	/* terminate */
};

static const u32 route_azv_2mic[] = {
	0xffffffff	/* terminate */
};

static const u32 route_azv_3mic[] = {
	0xffffffff	/* terminate */
};

static const u32 route_stereo_record_48k[] = {
	0x9031049a, /* 1178 */
	0x9031028d, /* 653 */
	0xffffffff	/* terminate */
};

static const u32 route_stereo_record_96k[] = {
	0x9031049a, /* 1178 */
	0x9031029f, /* 671 */
	0xffffffff	/* terminate */
};

static const u32 route_stereo_record_192k[] = {
	0x9031049a, /* 1178 */
	0x903102ae, /* 686 */
	0xffffffff	/* terminate */
};

static const u32 route_avalon_play[] = {
	0x90310497, /* 1175 */
	0xffffffff	/* terminate */
};
static const u32 route_avalon_capture[] = {
	0x9031049a, /* 1178 */
	0xffffffff	/* terminate */
};

static const u32 route_asr_1mic_ns_af_wb[] = {
	0x903104b1, /* 1201 */
	0x9031025e, /* 606 */
	0xffffffff	/* terminate */
};

static const u32 route_asr_2mic_ns_af_wb[] = {
	0x903104ca, /* 1226 */
	0x90310261, /* 609 */
	0xffffffff	/* terminate */
};

static const u32 route_voicesense_pdm_16k[] = {
	0x90311f40,	/* 16k route */
	0x90312008,
	0xffffffff      /* terminate */
};

static const u32 route_training_keyword_8k[] = {
	0x903105a3, /* 16K Route for recording the trained keyword PCR18 */
	0xffffffff      /* terminate */
};

static const u32 route_training_keyword_16k[] = {
	0x903105a4, /* 16K Route for recording the trained keyword PCR18 */
	0xffffffff      /* terminate */
};

static const u32 route_asr_3mic_slim_sde_24k[] = {
	0x9031054e, /* 1358 - 24K Route for 3 mic ASR SDE */
	0xffffffff      /* terminate */
};

static const u32 route_asr_3mic_slim_sde_48k[] = {
	0x903105a3, /* 1443 - 48K Route for 3 mic ASR SDE */
	0xffffffff      /* terminate */
};

static const u32 route_pt_pdm_cap_16k[] = {
	0xB05A0C00,
	0xB05A000A,
	0xB05B000D,
	0xB05B0B2D,
	0xB0640038,
	0xB0640160,
	0xB0640168,
	0xB06400D0,
	0xB0450005,
	0xB0630103,
	0xB0630116,
	0xB063010D,
	0xB0680300,
	0xB0681600,
	0xB0680D00,
	0x80000000,
	0xffffffff      /* terminate */
};

static const u32 route_3chan_pcm_passthrough[] = {
	0x804e0000, /* Set smooth rate to zero */
	0x90311f45, /* 8005 - 3 channel PCM passthrough route for PCR69 */
	0xffffffff	/* terminate */
};

static const u32 route_1chan_pdm_passthrough_porta_48k[] = {
	0x804e0000,
	0xB05A0C00,
	0xB05A000A,
	0xB05B000D,
	0xB05B0A2C,
	0xB0640038,
	0xB0640160,
	0xB0640168,
	0xB06400D0,
	0xB0450005,
	0xB0630303,
	0xB0630316,
	0xB063030D,
	0xB0680300,
	0xB0681600,
	0xB0680D00,
	0x80000000,
	0xffffffff	/* terminate */

};

static const u32 route_voice_1mic_porta[] = {
	0xB05A0C00,
	0xB05A0001,
	0xB05A000A,
	0xB05A001B,
	0xB05B0000,
	0xB05B0103,
	0xB05B0A20,
	0xB05B0B22,
	0xB0640038,
	0xB0640131,
	0xB0640048,
	0xB0640130,
	0xB0640138,
	0xB06400E0,
	0xB0640139,
	0xB06400D0,
	0xB0450005,
	0xB0630003,
	0xB0630004,
	0xB017004F,
	0xB0180000,
	0xB0630013,
	0xB063000D,
	0xB063000E,
	0xB0680300,
	0xB0680400,
	0xB0681300,
	0xB0680D00,
	0xB0680E00,
	0x80000000,
	0xffffffff
};

static const u32 route_vp_algo_off[] = {
	0x9031044D,
	0x9031026D,
	0xffffffff
};

static const u32 route_bargein[] = {
	0x903117D9, /* 2Ch16K_PT */
	0xffffffff
};

static const u32 route_vq[] = {
	0x90310566,
	0xffffffff
};

static const struct esxxx_route_config es705_route_config[ROUTE_MAX] = {
	[ROUTE_OFF] = {
		.route = route_off,
	},
	[ROUTE_CS_VOICE_1MIC_CT] = {
		.route = route_cs_voice_1mic_ct,
		.nb = nb_cs_voice_1mic_ct,
		.wb = wb_cs_voice_1mic_ct,
		.swb = swb_cs_voice_1mic_ct,
	},
	[ROUTE_CS_VOICE_2MIC_CT] = {
		.route = route_cs_voice_2mic_ct,
		.nb = nb_cs_voice_2mic_ct,
		.wb = wb_cs_voice_2mic_ct,
		.swb = swb_cs_voice_2mic_ct,
	},
	[ROUTE_CS_VOICE_3MIC_CT] = {
		.route = route_cs_voice_3mic_ct,
		.nb = nb_cs_voice_3mic_ct,
		.wb = wb_cs_voice_3mic_ct,
		.swb = swb_cs_voice_3mic_ct,
	},
	[ROUTE_CS_VOICE_1MIC_FT] = {
		.route = route_cs_voice_1mic_ft,
		.nb = nb_cs_voice_1mic_ft,
		.wb = wb_cs_voice_1mic_ft,
		.swb = swb_cs_voice_1mic_ft,
	},
	[ROUTE_CS_VOICE_2MIC_FT] = {
		.route = route_cs_voice_2mic_ft,
		.nb = nb_cs_voice_2mic_ft,
		.wb = wb_cs_voice_2mic_ft,
		.swb = swb_cs_voice_2mic_ft,
	},
	[ROUTE_CS_VOICE_3MIC_FT] = {
		.route = route_cs_voice_3mic_ft,
		.nb = nb_cs_voice_3mic_ft,
		.wb = wb_cs_voice_3mic_ft,
		.swb = swb_cs_voice_3mic_ft,
	},
	[ROUTE_CS_VOICE_HEADSET] = {
		.route = route_cs_voice_headset,
		.nb = nb_cs_voice_headset,
		.wb = wb_cs_voice_headset,
		.swb = swb_cs_voice_headset,
	},
	[ROUTE_CS_VOICE_1MIC_HEADPHONE] = {
		.route = route_cs_voice_1mic_headphone,
		.nb = nb_cs_voice_1mic_headphone,
		.wb = wb_cs_voice_1mic_headphone,
		.swb = swb_cs_voice_1mic_headphone,
	},
	[ROUTE_VOIP_1MIC_CT] = {
		.route = route_voip_1mic_ct,
		.nb = nb_voip_1mic_ct,
		.wb = wb_voip_1mic_ct,
		.swb = swb_voip_1mic_ct,
	},
	[ROUTE_VOIP_2MIC_CT] = {
		.route = route_voip_2mic_ct,
		.nb = nb_voip_2mic_ct,
		.wb = wb_voip_2mic_ct,
		.swb = swb_voip_2mic_ct,
		.fb = fb_voip_2mic_ct,
	},
	[ROUTE_VOIP_3MIC_CT] = {
		.route = route_voip_3mic_ct,
		.nb = nb_voip_3mic_ct,
		.wb = wb_voip_3mic_ct,
		.swb = swb_voip_3mic_ct,
		.fb = fb_voip_3mic_ct,
	},
	[ROUTE_VOIP_1MIC_FT] = {
		.route = route_voip_1mic_ft,
		.nb = nb_voip_1mic_ft,
		.wb = wb_voip_1mic_ft,
		.swb = swb_voip_1mic_ft,
	},
	[ROUTE_VOIP_2MIC_FT] = {
		.route = route_voip_2mic_ft,
		.nb = nb_voip_2mic_ft,
		.wb = wb_voip_2mic_ft,
		.swb = swb_voip_2mic_ft,
		.fb = fb_voip_2mic_ft,
	},
	[ROUTE_VOIP_3MIC_FT] = {
		.route = route_voip_3mic_ft,
		.nb = nb_voip_3mic_ft,
		.wb = wb_voip_3mic_ft,
		.swb = swb_voip_3mic_ft,
		.fb = fb_voip_3mic_ft,
	},
	[ROUTE_VOIP_HEADSET] = {
		.route = route_voip_headset,
		.nb = nb_voip_headset,
		.wb = wb_voip_headset,
		.swb = swb_voip_headset,
	},
	[ROUTE_VOIP_1MIC_HEADPHONE] = {
		.route = route_voip_1mic_headphone,
		.nb = nb_voip_1mic_headphone,
		.wb = wb_voip_1mic_headphone,
		.swb = swb_voip_1mic_headphone,
	},
	[ROUTE_VOICE_ASR_1MIC] = {
		.route = route_voice_asr_1mic,
		.nb = nb_voice_asr_1mic,
		.wb = wb_voice_asr_1mic,
		.swb = swb_voice_asr_1mic,
	},
	[ROUTE_VOICE_ASR_2MIC] = {
		.route = route_voice_asr_2mic,
		.nb = nb_voice_asr_2mic,
		.wb = wb_voice_asr_2mic,
		.swb = swb_voice_asr_2mic,
	},
	[ROUTE_VOICE_ASR_3MIC] = {
		.route = route_voice_asr_3mic,
		.nb = nb_voice_asr_3mic,
		.wb = wb_voice_asr_3mic,
		.swb = swb_voice_asr_3mic,
	},
	[ROUTE_VOICESENSE_SBUSRX4] = {
		.route = route_voicesense_sbusrx4,
	},
	[ROUTE_VOICESENSE_SBUSRX0] = {
		.route = route_voicesense_sbusrx0,
	},
	[ROUTE_VOICESENSE_PDM] = {
		.route = route_voicesense_pdm,
	},
	[ROUTE_1CHAN_PLAYBACK] = {
		.route = route_1chan_playback,
	},
	[ROUTE_2CHAN_PLAYBACK] = {
		.route = route_2chan_playback,
	},
	[ROUTE_1CHAN_CAPTURE] = {
		.route = route_1chan_capture,
	},
	[ROUTE_2CHAN_CAPTURE] = {
		.route = route_2chan_capture,
	},
	[ROUTE_AUDIOFOCUS_2MIC] = {
		.route = route_audiofocus_2mic,
	},
	[ROUTE_AUDIOFOCUS_3MIC] = {
		.route = route_audiofocus_3mic,
	},
	[ROUTE_2MIC_NS_CT_ANALOG] = {
		.route = route_2mic_ns_ct_analog,
	},
	[ROUTE_2MIC_NS_FT_ANALOG] = {
		.route = route_2mic_ns_ft_analog,
	},
	[ROUTE_2MIC_NS_FO_ANALOG] = {
		.route = route_2mic_ns_fo_analog,
	},
	[ROUTE_3MIC_NS_CT_ANALOG] = {
		.route = route_3mic_ns_ct_analog,
	},
	[ROUTE_3MIC_NS_FT_ANALOG] = {
		.route = route_3mic_ns_ft_analog,
	},
	[ROUTE_3MIC_NS_FO_ANALOG] = {
		.route = route_3mic_ns_fo_analog,
	},
	[ROUTE_AEC7_2MIC_NS_FT] = {
	.route = route_aec7_2mic_ns_ft,
	},
	[ROUTE_AEC7_3MIC_NS_FT] = {
	.route = route_aec7_3mic_ns_ft,
	},
	[ROUTE_ASR_2MIC_NS_AF] = {
	.route = route_asr_2mic_ns_af,
	},
	[ROUTE_ASR_3MIC_NS_AF] = {
		.route = route_asr_3mic_ns_af,
	},
	[ROUTE_AZV_2MIC] = {
		.route = route_azv_2mic,
	},
	[ROUTE_AZV_3MIC] = {
		.route = route_azv_3mic,
	},
	[ROUTE_STEREO_RECORD_48K] = {
		.route = route_stereo_record_48k,
	},
	[ROUTE_STEREO_RECORD_96K] = {
		.route = route_stereo_record_96k,
	},
	[ROUTE_STEREO_RECORD_192K] = {
		.route = route_stereo_record_192k,
	},
	[ROUTE_AVALON_PLAY] = {
		.route = route_avalon_play,
	},
	[ROUTE_AVALON_CAPTURE] = {
		.route = route_avalon_capture,
	},
	[ROUTE_ASR_1MIC_NS_AF_WB] = {
		.route = route_asr_1mic_ns_af_wb,
	},
	[ROUTE_ASR_2MIC_NS_AF_WB] = {
		.route = route_asr_2mic_ns_af_wb,
	},
	[ROUTE_VOICESENSE_PDM_16K] = {
		.route = route_voicesense_pdm_16k,
	},
	[ROUTE_TRAINING_KEYWORD_8K] = {
		.route = route_training_keyword_8k,
	},
	[ROUTE_TRAINING_KEYWORD_16K] = {
		.route = route_training_keyword_16k,
	},
	[ROUTE_ASR_3MIC_SLIM_SDE_24K] = {
		.route = route_asr_3mic_slim_sde_24k,
	},
	[ROUTE_ASR_3MIC_SLIM_SDE_48K] = {
		.route = route_asr_3mic_slim_sde_48k,
	},
	[ROUTE_PT_PDM_CAP_16K] = {
		.route = route_pt_pdm_cap_16k,
	},
	[ROUTE_3CHAN_PCM_PT] = {
		.route = route_3chan_pcm_passthrough,
	},
	[ROUTE_1CHAN_CAP_PCM_PORTA_48K] = {
		.route = route_1chan_pdm_passthrough_porta_48k,
	},
	[ROUTE_VOICE_1MIC_PORTA] = {
		.route = route_voice_1mic_porta,
	},
	[ROUTE_VOIP_3MIC] = {
		.route = route_voip_3mic,
	},
	[ROUTE_VP_ALGO_OFF] = {
		.route = route_vp_algo_off,
	},
	[ROUTE_BARGEIN] = {
		.route = route_bargein,
	},
	[ROUTE_VQ] = {
		.route = route_vq,
	},
};

#endif
