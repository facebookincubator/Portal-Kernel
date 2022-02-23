/* Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __APR_AUDIO_V2_DT_BINDINGS_H__
#define __APR_AUDIO_V2_DT_BINDINGS_H__

#define AFE_PORT_ID_PRIMARY_MI2S_RX			0x1000
#define AFE_PORT_ID_PRIMARY_MI2S_TX			0x1001
#define AFE_PORT_ID_SECONDARY_MI2S_RX			0x1002
#define AFE_PORT_ID_SECONDARY_MI2S_TX			0x1003
#define AFE_PORT_ID_TERTIARY_MI2S_RX			0x1004
#define AFE_PORT_ID_TERTIARY_MI2S_TX        		0x1005
#define AFE_PORT_ID_QUATERNARY_MI2S_RX      		0x1006
#define AFE_PORT_ID_QUATERNARY_MI2S_TX      		0x1007
#define AUDIO_PORT_ID_I2S_RX                		0x1008
#define AFE_PORT_ID_DIGITAL_MIC_TX          		0x1009
#define AFE_PORT_ID_PRIMARY_PCM_RX          		0x100A
#define AFE_PORT_ID_PRIMARY_PCM_TX          		0x100B
#define AFE_PORT_ID_SECONDARY_PCM_RX        		0x100C
#define AFE_PORT_ID_SECONDARY_PCM_TX        		0x100D
#define AFE_PORT_ID_MULTICHAN_HDMI_RX       		0x100E
#define AFE_PORT_ID_SECONDARY_MI2S_RX_SD1   		0x1010
#define AFE_PORT_ID_TERTIARY_PCM_RX         		0x1012
#define AFE_PORT_ID_TERTIARY_PCM_TX         		0x1013
#define AFE_PORT_ID_QUATERNARY_PCM_RX       		0x1014
#define AFE_PORT_ID_QUATERNARY_PCM_TX       		0x1015
#define AFE_PORT_ID_QUINARY_MI2S_RX         		0x1016
#define AFE_PORT_ID_QUINARY_MI2S_TX         		0x1017
#define AFE_PORT_ID_SENARY_MI2S_RX          		0x1018
#define AFE_PORT_ID_SENARY_MI2S_TX          		0x1019

/* ID of the Internal 0 MI2S Rx port */
#define AFE_PORT_ID_INT0_MI2S_RX			0x102E
/* ID of the Internal 0 MI2S Tx port */
#define AFE_PORT_ID_INT0_MI2S_TX			0x102F
/* ID of the Internal 1 MI2S Rx port */
#define AFE_PORT_ID_INT1_MI2S_RX			0x1030
/* ID of the Internal 1 MI2S Tx port */
#define AFE_PORT_ID_INT1_MI2S_TX			0x1031
/* ID of the Internal 2 MI2S Rx port */
#define AFE_PORT_ID_INT2_MI2S_RX			0x1032
/* ID of the Internal 2 MI2S Tx port */
#define AFE_PORT_ID_INT2_MI2S_TX			0x1033
/* ID of the Internal 3 MI2S Rx port */
#define AFE_PORT_ID_INT3_MI2S_RX			0x1034
/* ID of the Internal 3 MI2S Tx port */
#define AFE_PORT_ID_INT3_MI2S_TX			0x1035
/* ID of the Internal 4 MI2S Rx port */
#define AFE_PORT_ID_INT4_MI2S_RX			0x1036
/* ID of the Internal 4 MI2S Tx port */
#define AFE_PORT_ID_INT4_MI2S_TX			0x1037
/* ID of the Internal 5 MI2S Rx port */
#define AFE_PORT_ID_INT5_MI2S_RX			0x1038
/* ID of the Internal 5 MI2S Tx port */
#define AFE_PORT_ID_INT5_MI2S_TX			0x1039
/* ID of the Internal 6 MI2S Rx port */
#define AFE_PORT_ID_INT6_MI2S_RX			0x103A
/* ID of the Internal 6 MI2S Tx port */
#define AFE_PORT_ID_INT6_MI2S_TX			0x103B

#define AFE_PORT_ID_QUINARY_PCM_RX			0x103C
#define AFE_PORT_ID_QUINARY_PCM_TX			0x103D

#define AFE_PORT_ID_SPDIF_RX				0x5000
#define AFE_PORT_ID_RT_PROXY_PORT_001_RX		0x2000
#define AFE_PORT_ID_RT_PROXY_PORT_001_TX		0x2001
#define AFE_PORT_ID_INTERNAL_BT_SCO_RX			0x3000
#define AFE_PORT_ID_INTERNAL_BT_SCO_TX			0x3001
#define AFE_PORT_ID_INTERNAL_BT_A2DP_RX			0x3002
#define AFE_PORT_ID_INTERNAL_FM_RX			0x3004
#define AFE_PORT_ID_INTERNAL_FM_TX			0x3005
/* SLIMbus Rx port on channel 0. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_RX		0x4000
/* SLIMbus Tx port on channel 0. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_0_TX		0x4001
/* SLIMbus Rx port on channel 1. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_RX		0x4002
/* SLIMbus Tx port on channel 1. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_1_TX		0x4003
/* SLIMbus Rx port on channel 2. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_RX		0x4004
/* SLIMbus Tx port on channel 2. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_2_TX		0x4005
/* SLIMbus Rx port on channel 3. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_RX		0x4006
/* SLIMbus Tx port on channel 3. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_3_TX		0x4007
/* SLIMbus Rx port on channel 4. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_RX		0x4008
/* SLIMbus Tx port on channel 4. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_4_TX		0x4009
/* SLIMbus Rx port on channel 5. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX		0x400a
/* SLIMbus Tx port on channel 5. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_TX		0x400b
/* SLIMbus Rx port on channel 6. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX		0x400c
/* SLIMbus Tx port on channel 6. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_TX		0x400d
/* SLIMbus Rx port on channel 7. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_7_RX		0x400e
/* SLIMbus Tx port on channel 7. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_7_TX		0x400f
/* SLIMbus Rx port on channel 8. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_8_RX		0x4010
/* SLIMbus Tx port on channel 8. */
#define AFE_PORT_ID_SLIMBUS_MULTI_CHAN_8_TX		0x4011
/* AFE Rx port for audio over Display port */
#define AFE_PORT_ID_HDMI_OVER_DP_RX			0x6020
/*USB AFE port */
#define AFE_PORT_ID_USB_RX				0x7000
#define AFE_PORT_ID_USB_TX				0x7001

/* Generic pseudoport 1. */
#define AFE_PORT_ID_PSEUDOPORT_01			0x8001
/* Generic pseudoport 2. */
#define AFE_PORT_ID_PSEUDOPORT_02			0x8002

/* @xreflabel{hdr:AfePortIdPrimaryAuxPcmTx}
 * Primary Aux PCM Tx port ID.
 */
#define AFE_PORT_ID_PRIMARY_PCM_TX			0x100B
/* Pseudoport that corresponds to the voice Rx path.
 * For recording, the voice Rx path samples are written to this
 * port and consumed by the audio path.
 */

#define AFE_PORT_ID_VOICE_RECORD_RX			0x8003

/* Pseudoport that corresponds to the voice Tx path.
 * For recording, the voice Tx path samples are written to this
 * port and consumed by the audio path.
 */

#define AFE_PORT_ID_VOICE_RECORD_TX			0x8004
/* Pseudoport that corresponds to in-call voice delivery samples.
 * During in-call audio delivery, the audio path delivers samples
 * to this port from where the voice path delivers them on the
 * Rx path.
 */
#define AFE_PORT_ID_VOICE2_PLAYBACK_TX			0x8002
#define AFE_PORT_ID_VOICE_PLAYBACK_TX			0x8005


/* Supported OSR clock values */
#define Q6AFE_LPASS_OSR_CLK_12_P288_MHZ			0xBB8000
#define Q6AFE_LPASS_OSR_CLK_11_P2896_MHZ		0xAC4400
#define Q6AFE_LPASS_OSR_CLK_9_P600_MHZ			0x927C00
#define Q6AFE_LPASS_OSR_CLK_8_P192_MHZ			0x7D0000
#define Q6AFE_LPASS_OSR_CLK_6_P144_MHZ			0x5DC000
#define Q6AFE_LPASS_OSR_CLK_4_P096_MHZ			0x3E8000
#define Q6AFE_LPASS_OSR_CLK_3_P072_MHZ			0x2EE000
#define Q6AFE_LPASS_OSR_CLK_2_P048_MHZ			0x1F4000
#define Q6AFE_LPASS_OSR_CLK_1_P536_MHZ			0x177000
#define Q6AFE_LPASS_OSR_CLK_1_P024_MHZ			0xFA000
#define Q6AFE_LPASS_OSR_CLK_768_kHZ			0xBB800
#define Q6AFE_LPASS_OSR_CLK_512_kHZ			0x7D000
#define Q6AFE_LPASS_OSR_CLK_DISABLE			0x0

/* Supported Bit clock values */
#define Q6AFE_LPASS_IBIT_CLK_12_P288_MHZ		0xBB8000
#define Q6AFE_LPASS_IBIT_CLK_11_P2896_MHZ		0xAC4400
#define Q6AFE_LPASS_IBIT_CLK_8_P192_MHZ			0x7D0000
#define Q6AFE_LPASS_IBIT_CLK_6_P144_MHZ			0x5DC000
#define Q6AFE_LPASS_IBIT_CLK_4_P096_MHZ			0x3E8000
#define Q6AFE_LPASS_IBIT_CLK_3_P072_MHZ			0x2EE000
#define Q6AFE_LPASS_IBIT_CLK_2_P8224_MHZ		0x2b1100
#define Q6AFE_LPASS_IBIT_CLK_2_P048_MHZ			0x1F4000
#define Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ			0x177000
#define Q6AFE_LPASS_IBIT_CLK_1_P4112_MHZ		0x158880
#define Q6AFE_LPASS_IBIT_CLK_1_P024_MHZ			0xFA000
#define Q6AFE_LPASS_IBIT_CLK_768_KHZ			0xBB800
#define Q6AFE_LPASS_IBIT_CLK_512_KHZ			0x7D000
#define Q6AFE_LPASS_IBIT_CLK_256_KHZ			0x3E800
#define Q6AFE_LPASS_IBIT_CLK_DISABLE			0x0

/* Clock ID Enumeration Define. */
/* Clock ID for Primary I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT		0x100
/* Clock ID for Primary I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_MI2S_EBIT		0x101
/* Clock ID for Secondary I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT		0x102
/* Clock ID for Secondary I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_MI2S_EBIT		0x103
/* Clock ID for Tertiary I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT		0x104
/* Clock ID for Tertiary I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_TER_MI2S_EBIT		0x105
/* Clock ID for Quartnery I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT		0x106
/* Clock ID for Quartnery I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_MI2S_EBIT		0x107
/* Clock ID for Speaker I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_IBIT		0x108
/* Clock ID for Speaker I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_EBIT		0x109
/* Clock ID for Speaker I2S OSR */
#define Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_OSR		0x10A

/* Clock ID for QUINARY  I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT		0x10B
/* Clock ID for QUINARY  I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_QUI_MI2S_EBIT		0x10C
/* Clock ID for SENARY  I2S IBIT */
#define Q6AFE_LPASS_CLK_ID_SEN_MI2S_IBIT		0x10D
/* Clock ID for SENARY  I2S EBIT */
#define Q6AFE_LPASS_CLK_ID_SEN_MI2S_EBIT		0x10E
/* Clock ID for INT0 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT0_MI2S_IBIT		0x10F
/* Clock ID for INT1 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT1_MI2S_IBIT		0x110
/* Clock ID for INT2 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT2_MI2S_IBIT		0x111
/* Clock ID for INT3 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT3_MI2S_IBIT		0x112
/* Clock ID for INT4 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT4_MI2S_IBIT		0x113
/* Clock ID for INT5 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT5_MI2S_IBIT		0x114
/* Clock ID for INT6 I2S IBIT  */
#define Q6AFE_LPASS_CLK_ID_INT6_MI2S_IBIT		0x115

/* Clock ID for QUINARY MI2S OSR CLK  */
#define Q6AFE_LPASS_CLK_ID_QUI_MI2S_OSR			0x116

/* Clock ID for Primary PCM IBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_PCM_IBIT			0x200
/* Clock ID for Primary PCM EBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_PCM_EBIT			0x201
/* Clock ID for Secondary PCM IBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_PCM_IBIT			0x202
/* Clock ID for Secondary PCM EBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_PCM_EBIT			0x203
/* Clock ID for Tertiary PCM IBIT */
#define Q6AFE_LPASS_CLK_ID_TER_PCM_IBIT			0x204
/* Clock ID for Tertiary PCM EBIT */
#define Q6AFE_LPASS_CLK_ID_TER_PCM_EBIT			0x205
/* Clock ID for Quartery PCM IBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_PCM_IBIT		0x206
/* Clock ID for Quartery PCM EBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_PCM_EBIT		0x207
/* Clock ID for Quinary PCM IBIT */
#define Q6AFE_LPASS_CLK_ID_QUIN_PCM_IBIT		0x208
/* Clock ID for Quinary PCM EBIT */
#define Q6AFE_LPASS_CLK_ID_QUIN_PCM_EBIT		0x209
/* Clock ID for QUINARY PCM OSR  */
#define Q6AFE_LPASS_CLK_ID_QUI_PCM_OSR			0x20A

/** Clock ID for Primary TDM IBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_TDM_IBIT			0x200
/** Clock ID for Primary TDM EBIT */
#define Q6AFE_LPASS_CLK_ID_PRI_TDM_EBIT			0x201
/** Clock ID for Secondary TDM IBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_TDM_IBIT			0x202
/** Clock ID for Secondary TDM EBIT */
#define Q6AFE_LPASS_CLK_ID_SEC_TDM_EBIT			0x203
/** Clock ID for Tertiary TDM IBIT */
#define Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT			0x204
/** Clock ID for Tertiary TDM EBIT */
#define Q6AFE_LPASS_CLK_ID_TER_TDM_EBIT			0x205
/** Clock ID for Quartery TDM IBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_TDM_IBIT		0x206
/** Clock ID for Quartery TDM EBIT */
#define Q6AFE_LPASS_CLK_ID_QUAD_TDM_EBIT		0x207
/** Clock ID for Quinary TDM IBIT */
#define Q6AFE_LPASS_CLK_ID_QUIN_TDM_IBIT		0x208
/** Clock ID for Quinary TDM EBIT */
#define Q6AFE_LPASS_CLK_ID_QUIN_TDM_EBIT		0x209
/** Clock ID for Quinary TDM OSR */
#define Q6AFE_LPASS_CLK_ID_QUIN_TDM_OSR	i		0x20A

/* Clock ID for MCLK1 */
#define Q6AFE_LPASS_CLK_ID_MCLK_1			0x300
/* Clock ID for MCLK2 */
#define Q6AFE_LPASS_CLK_ID_MCLK_2			0x301
/* Clock ID for MCLK3 */
#define Q6AFE_LPASS_CLK_ID_MCLK_3			0x302
/* Clock ID for MCLK4 */
#define Q6AFE_LPASS_CLK_ID_MCLK_4			0x304
/* Clock ID for Internal Digital Codec Core */
#define Q6AFE_LPASS_CLK_ID_INTERNAL_DIGITAL_CODEC_CORE	0x303
/* Clock ID for INT MCLK0 */
#define Q6AFE_LPASS_CLK_ID_INT_MCLK_0			0x305
/* Clock ID for INT MCLK1 */
#define Q6AFE_LPASS_CLK_ID_INT_MCLK_1			0x306
/*
 * Clock ID for soundwire NPL.
 * This is the clock to be used to enable NPL clock for  internal Soundwire.
 */
#define AFE_CLOCK_SET_CLOCK_ID_SWR_NPL_CLK		0x307

/* Clock ID for AHB HDMI input */
#define Q6AFE_LPASS_CLK_ID_AHB_HDMI_INPUT		0x400

/* Clock ID for SPDIF core */
#define Q6AFE_LPASS_CLK_ID_SPDIF_CORE			0x500

/* Supported LPASS CLK sources */
#define Q6AFE_LPASS_CLK_SRC_EXTERNAL			0
#define Q6AFE_LPASS_CLK_SRC_INTERNAL			1

/* Supported LPASS CLK root*/
#define Q6AFE_LPASS_CLK_ROOT_DEFAULT			0

/* Clock attribute for invalid use (reserved for internal usage) */
#define Q6AFE_LPASS_CLK_ATTRIBUTE_INVALID		0x0
/* Clock attribute for no couple case */
#define Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO		0x1
/* Clock attribute for dividend couple case */
#define Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_DIVIDEND	0x2
/* Clock attribute for divisor couple case */
#define Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_DIVISOR	0x3
/* Clock attribute for invert and no couple case */
#define Q6AFE_LPASS_CLK_ATTRIBUTE_INVERT_COUPLE_NO	0x4
/* Clock set API version */
#define Q6AFE_LPASS_CLK_CONFIG_API_VERSION		0x1

/* Enumeration for setting the I2S configuration channel_mode parameter to
 * serial data wire number 1-3 (SD3).
 */

/* member: i2s_cfg_minor_version */
#define AFE_API_VERSION_I2S_CONFIG			0x1

/* member: channel_mode */
#define AFE_PORT_I2S_SD0				0x1
#define AFE_PORT_I2S_SD1				0x2
#define AFE_PORT_I2S_SD2				0x3
#define AFE_PORT_I2S_SD3				0x4
#define AFE_PORT_I2S_QUAD01				0x5
#define AFE_PORT_I2S_QUAD23				0x6
#define AFE_PORT_I2S_6CHS				0x7
#define AFE_PORT_I2S_8CHS				0x8

/* member: mono_stereo */
#define AFE_PORT_I2S_MONO				0x0
#define AFE_PORT_I2S_STEREO				0x1

/* member: ws_src */
#define AFE_PORT_CONFIG_I2S_WS_SRC_EXTERNAL		0x0
#define AFE_PORT_CONFIG_I2S_WS_SRC_INTERNAL		0x1

/* member: sample_rate */
#define AFE_PORT_SAMPLE_RATE_8K				8000
#define AFE_PORT_SAMPLE_RATE_16K			16000
#define AFE_PORT_SAMPLE_RATE_48K			48000
#define AFE_PORT_SAMPLE_RATE_96K			96000
#define AFE_PORT_SAMPLE_RATE_176P4K			176400
#define AFE_PORT_SAMPLE_RATE_192K			192000
#define AFE_PORT_SAMPLE_RATE_352P8K			352800

#endif /* __APR_AUDIO_V2_DT_BINDINGS_H__ */
