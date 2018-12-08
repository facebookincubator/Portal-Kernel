/*
 * iaxxx-codec.c -- IAxxx CODEC driver
 *
 * Copyright 2017 Knowles Corporation
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

#define pr_fmt(fmt) "%s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm1.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm2.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm3.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm4.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm5.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-tunnel-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>

#define IAXXX_MAX_RETRY 5
#define PDM_CODEC_MASTER 1
#define IAXXX_MAX_PROC 3 /* Number of procs on D1400s */
#define IAXXX_MAX_VAL 0xFFFFFFFF
#define PCM_PORT_I2S 1
#define IAXXX_FW_DELAY (2 * 20)		/* Looks like 20 ms is enough */

static int iaxxx_calc_i2s_div(u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val);

/* Plugin struct to store param id and val
 Param ID and Param VAL registers are set as pair
*/
struct plg_param{
	u32 param_id;
	u32 param_val;
	u32 param_id_reg;
	u32 param_val_reg;
};

struct iaxxx_codec_priv {
	int is_codec_master;
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct device *dev;
	struct device *dev_parent;
	struct notifier_block nb_core;	/* Core notifier */
	/* Add entry for plg_param struct for each proc
	param id and param val registers are set as pair
	*/
	struct plg_param plugin_param[IAXXX_MAX_PROC];
	u32 portb_filter;
	u32 portc_filter;
	u32 portb_mic0_en;
	u32 portb_mic1_en;
	u32 portb_mic2_en;
	u32 portb_mic3_en;
	u32 portc_mic0_en;
	u32 portc_mic1_en;
	u32 portc_mic2_en;
	u32 portc_mic3_en;

	u32 portb_pcm_start;
	u32 portb_pcm_setup;
	u32 portc_pcm_start;
	u32 portc_pcm_setup;
	u32 portd_pcm_start;
	u32 portd_pcm_setup;

	u32 pdm_bclk;
	u32 pdm_aclk;
	u32 portb_dmic_start;
	u32 portc_dmic_start;
	u32 head_of_strm_all;
	bool is_stream_in_use[2];

	/* procfs entry for codec packet loss */
	struct snd_info_entry *root_entry;
	struct snd_info_entry *pkg_loss;

	char pkg_loss_buf[256];
};

static const u32 cic_rx_addr[] = {
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
	IAXXX_CNR0_CIC_RX_4_5_ADDR,
	IAXXX_CNR0_CIC_RX_4_5_ADDR,
	IAXXX_CNR0_CIC_RX_6_7_ADDR,
	IAXXX_CNR0_CIC_RX_6_7_ADDR,
};

static const u32 cic_rx_clr_mask[] = {
	IAXXX_CNR0_CIC_RX_0_1_CLR_0_MASK,
	IAXXX_CNR0_CIC_RX_0_1_CLR_1_MASK,
	IAXXX_CNR0_CIC_RX_2_3_CLR_2_MASK,
	IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
	IAXXX_CNR0_CIC_RX_4_5_CLR_4_MASK,
	IAXXX_CNR0_CIC_RX_4_5_CLR_5_MASK,
	IAXXX_CNR0_CIC_RX_6_7_CLR_6_MASK,
	IAXXX_CNR0_CIC_RX_6_7_CLR_7_MASK,
};

static const u32 cic_rx_m_mask[] = {
	IAXXX_CNR0_CIC_RX_0_1_M_0_MASK,
	IAXXX_CNR0_CIC_RX_0_1_M_1_MASK,
	IAXXX_CNR0_CIC_RX_2_3_M_2_MASK,
	IAXXX_CNR0_CIC_RX_2_3_M_3_MASK,
	IAXXX_CNR0_CIC_RX_4_5_M_4_MASK,
	IAXXX_CNR0_CIC_RX_4_5_M_5_MASK,
	IAXXX_CNR0_CIC_RX_6_7_M_6_MASK,
	IAXXX_CNR0_CIC_RX_6_7_M_7_MASK,
};

static const u32 cic_rx_clr_pos[] = {
	IAXXX_CNR0_CIC_RX_0_1_CLR_0_POS,
	IAXXX_CNR0_CIC_RX_0_1_CLR_1_POS,
	IAXXX_CNR0_CIC_RX_2_3_CLR_2_POS,
	IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS,
	IAXXX_CNR0_CIC_RX_4_5_CLR_4_POS,
	IAXXX_CNR0_CIC_RX_4_5_CLR_5_POS,
	IAXXX_CNR0_CIC_RX_6_7_CLR_6_POS,
	IAXXX_CNR0_CIC_RX_6_7_CLR_7_POS,
};

static const u32 cic_rx_m_pos[] = {
	IAXXX_CNR0_CIC_RX_0_1_M_0_POS,
	IAXXX_CNR0_CIC_RX_0_1_M_1_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_2_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_3_POS,
	IAXXX_CNR0_CIC_RX_4_5_M_4_POS,
	IAXXX_CNR0_CIC_RX_4_5_M_5_POS,
	IAXXX_CNR0_CIC_RX_6_7_M_6_POS,
	IAXXX_CNR0_CIC_RX_6_7_M_7_POS,
};

static const u32 dmic_enable_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_ADDR,
};

static const u32 dmic_busy_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_BUSY_ADDR,
};

enum {
	RX_0 = 0,
	RX_1,
	RX_2,
	RX_3,
	RX_4,
	RX_5,
	RX_6,
	RX_7,
	RX_8,
	RX_9,
	RX_10,
	RX_11,
	RX_12,
	RX_13,
	RX_14,
	RX_15,
	TX_0 = 16,
	TX_1,
	TX_2,
	TX_3,
	TX_4,
	TX_5,
	TX_6,
	TX_7,
	TX_8,
	TX_9,
	TX_10,
	TX_11,
	TX_12,
	TX_13,
	TX_14,
	TX_15,
};

enum {
	PCM_PORTA = 0,
	PCM_PORTB = 1,
	PCM_PORTC = 2,
	PCM_PORTD = 3,
	PCM_PORTE = 4,
	PCM_PORTF = 5,
};

enum {
	PLUGIN0 = 0,
	PLUGIN1,
	PLUGIN2,
	PLUGIN3,
	PLUGIN4,
	PLUGIN5,
	PLUGIN6,
	PLUGIN7,
	IMIX0 = 16,
	IMIX1,
	IMIX2,
	IMIX3,
	IBUF0 = 24,
	IBUF1,
	IBUF2,
	IBUF3,
};

enum {
	CIC0,
	CIC1,
	CIC2,
	CIC3,
	CIC4,
	CIC5,
	CIC6,
	CIC7,
	CIC_NONE
};

enum {
	DMIC0,
	DMIC1,
	DMIC2,
	DMIC3,
	DMIC4,
	DMIC5,
	DMIC6,
	DMIC7,
};

enum {
	IAXXX_PDM_CLK_0P_512MHZ,
	IAXXX_PDM_CLK_1P_024MHZ,
	IAXXX_PDM_CLK_1P_536MHZ,
	IAXXX_PDM_CLK_2P_832MHZ,
	IAXXX_PDM_CLK_3P_072MHZ,
	IAXXX_PDM_CLK_5P_644MHZ,
	IAXXX_PDM_CLK_6P_144MHZ,
	IAXXX_PDM_CLK_NONE
};

enum {
	IAXXX_AUD_PORT_8K,
	IAXXX_AUD_PORT_12K,
	IAXXX_AUD_PORT_16K,
	IAXXX_AUD_PORT_22_05K,
	IAXXX_AUD_PORT_24K,
	IAXXX_AUD_PORT_32K,
	IAXXX_AUD_PORT_44_1K,
	IAXXX_AUD_PORT_48K,
	IAXXX_AUD_PORT_96K,
	IAXXX_AUD_PORT_192K,
	IAXXX_AUD_PORT_NONE
};


struct iaxxx_cic_deci_table {
	u32 cic_dec;
	u32 hb_dec;
};

struct iaxxx_pdm_bit_cfg {
	u32 sample_rate;
	u32 bits_per_frame;
};

/* Table taken from FW PDM Driver */
static struct iaxxx_pdm_bit_cfg pdm_cfg[] = {
	{8000, 64},
	{16000, 64},
	{32000, 48},
	{44000, 64},
	{48000, 64},
	{88000, 64},
	{96000, 64},
};

/* This table is two dimension array of CIC decimation and Green box(Half band)
 *  values with PDM_BCLK(rows) * AUD_PORT_CLK(columns)
 */
static struct iaxxx_cic_deci_table deci_rb_enable[][IAXXX_AUD_PORT_NONE] = {
	{/* PDM_PORT_BIT_CLK_FREQ_0_512M */
	{16, 4}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{16, 2}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{8, 2}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},
	{/* PDM_PORT_BIT_CLK_FREQ_1_024M */
	{32, 4}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{16, 4}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{8, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_1_536M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{32, 4}, /* PDM_PORT_FREQ_12K  */
	{24, 4}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{16, 4}, /* PDM_PORT_FREQ_24K  */
	{12, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{16, 2}, /* PDM_PORT_FREQ_48K  */
	{8, 2}, /* PDM_PORT_FREQ_96K  */
	{8, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_2_8224M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{32, 4}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{16, 4}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/*PDM_PORT_BIT_CLK_FREQ_3_072M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{32, 4}, /* PDM_PORT_FREQ_24K  */
	{24, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{16, 4}, /* PDM_PORT_FREQ_48K  */
	{8, 4}, /* PDM_PORT_FREQ_96K  */
	{16, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/*PDM_PORT_BIT_CLK_FREQ_5_6448M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{32, 4}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_6_144M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{32, 4}, /* PDM_PORT_FREQ_48K  */
	{16, 4}, /* PDM_PORT_FREQ_96K  */
	{8, 4}, /* PDM_PORT_FREQ_192K */
	},
};

struct iaxxx_i2s_div_config {
	u32 bclk;
	u32 N;
	u32 R;
	u32 HL;
	u32 period;
};

/* I2S_ACLK_FREQ_24576*/
static struct iaxxx_i2s_div_config i2s_div_config[] = {
	/* BCLK  N  R     HL  P */
	{ 512000, 1, 4096, 12, 4 },          /* I2S_BIT_CLK_FREQ_0_512M  */
	{ 768000, 1, 4096, 8, 4 },           /* I2S_BIT_CLK_FREQ_0_768M  */
	{ 1024000, 1, 4096, 6, 4 },          /* I2S_BIT_CLK_FREQ_1_024M  */
	{ 1536000, 1, 4096, 4, 4 },          /* I2S_BIT_CLK_FREQ_1_536M  */
	{ 2048000, 1, 4096, 3, 4 },          /* I2S_BIT_CLK_FREQ_2_048M  */
	{ 3072000, 1, 4096, 2, 4 },          /* I2S_BIT_CLK_FREQ_3_072M  */
	{ 4096000, 1, 4096, 1, 6 },          /* I2S_BIT_CLK_FREQ_4_096M  */
	{ 6144000, 1, 4096, 1, 4 },          /* I2S_BIT_CLK_FREQ_6_144M  */
	{ 8192000, 1, 4096, 3, 1 },          /* I2S_BIT_CLK_FREQ_8_192M  */
};

int get_decimator_val(u32 port_bclk, u32 aud_port_clk,
			u32 *cic_dec, u32 *hb_deci)
{
	int ret = -EINVAL;

	if (port_bclk >= IAXXX_PDM_CLK_NONE)
		return ret;

	if (aud_port_clk >= IAXXX_AUD_PORT_NONE)
		return ret;

	if ((cic_dec == NULL) || (hb_deci == NULL))
		return ret;

	*cic_dec = deci_rb_enable[port_bclk][aud_port_clk].cic_dec;
	if (*cic_dec == 0) {
		/* Not supported value */
		return ret;
	}
	*cic_dec = *cic_dec - 1;

	if (deci_rb_enable[port_bclk][aud_port_clk].hb_dec == 2)
		*hb_deci = 1;
	else if (deci_rb_enable[port_bclk][aud_port_clk].hb_dec == 4)
		*hb_deci = 2;
	else
		*hb_deci = 0;
	return 0;
}


static const u32 port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_PORTD_CLK_ADDR,
	IAXXX_IO_CTRL_PORTE_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTD_FS_ADDR,
	IAXXX_IO_CTRL_PORTE_FS_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
	IAXXX_IO_CTRL_PORTE_DI_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_PORTD_DO_ADDR,
	IAXXX_IO_CTRL_PORTE_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

enum {
	IAXXX_AIF0 = 0,
	IAXXX_AIF1,
	IAXXX_AIF2,
	IAXXX_AIF3,
	IAXXX_AIF4,
	IAXXX_AIF5,
	IAXXX_NUM_CODEC_DAIS,
};

enum {
	STREAM0 = 0,
	STREAM1,
	STREAM2,
	STREAM3,
	STREAM4,
	STREAM5,
	STREAM6,
	STREAM7,
	STREAM8,
	STREAM9,
	STREAM10,
	STREAM11,
	STREAM12,
	STREAM13,
	STREAM14,
	STREAM15,
	STREAM_NONE,
};

enum Encoding_s {
	ENCODING_OPAQUE		= 0x00,
	ENCODING_AFLOAT		= 0x01,
	ENCODING_G711A		= 0x02,
	ENCODING_G711U		= 0x03,
	ENCODING_FLOAT		= 0x04,
	ENCODING_Q15		= 0x0F,
	ENCODING_Q16		= 0x10,
	ENCODING_Q17		= 0x11,
	ENCODING_Q18		= 0x12,
	ENCODING_Q19		= 0x13,
	ENCODING_Q20		= 0x14,
	ENCODING_Q21		= 0x15,
	ENCODING_Q22		= 0x16,
	ENCODING_Q23		= 0x17,
	ENCODING_Q24		= 0x18,
	ENCODING_Q25		= 0x19,
	ENCODING_Q26		= 0x1A,
	ENCODING_Q27		= 0x1B,
	ENCODING_Q28		= 0x1C,
	ENCODING_Q29		= 0x1D,
	ENCODING_Q30		= 0x1E,
	ENCODING_Q31		= 0x1F,
	ENCODING_ERROR		= 0xFFFF,
};

enum Rate_s {
	RATE_8K = 0x0,
	RATE_11P025K = 0x1,
	RATE_12K = 0x2,
	RATE_16K = 0x3,
	RATE_22P050K = 0x4,
	RATE_24K = 0x5,
	RATE_32K = 0x6,
	RATE_44P1K = 0x7,
	RATE_48K = 0x8,
	RATE_96K = 0x9,
	RATE_192K = 0xA,
	RATE_INVALID = 0xF,
};

enum Gain_Ramp_Step {
	STEP_300 = 0x12C,
	STEP_600 = 0x258,
	STEP_900 = 0x384,
	STEP_1200 = 0x4B0,
	STEP_1600 = 0x640,
	STEP_2000 = 0x7D0,
	STEP_INST = 0xFFFF,
};

static const char * const pdm_bclk_texts[] = {
	"IAXXX_PDM_CLK_0P_512MHZ",
	"IAXXX_PDM_CLK_1P_024MHZ",
	"IAXXX_PDM_CLK_1P_536MHZ",
	"IAXXX_PDM_CLK_2P_832MHZ",
	"IAXXX_PDM_CLK_3P_072MHZ",
	"IAXXX_PDM_CLK_5P_644MHZ",
	"IAXXX_PDM_CLK_6P_144MHZ",
	"IAXXX_PDM_CLK_NONE",
};

static const char * const pdm_aclk_texts[] = {
	"IAXXX_AUD_PORT_8K",
	"IAXXX_AUD_PORT_12K",
	"IAXXX_AUD_PORT_16K",
	"IAXXX_AUD_PORT_22_05K",
	"IAXXX_AUD_PORT_24K",
	"IAXXX_AUD_PORT_32K",
	"IAXXX_AUD_PORT_44_1K",
	"IAXXX_AUD_PORT_48K",
	"IAXXX_AUD_PORT_96K",
	"IAXXX_AUD_PORT_192K",
	"IAXXX_AUD_PORT_NONE"
};

static const struct soc_enum iaxxx_pdm_bclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_bclk_texts),
                        pdm_bclk_texts);

static const struct soc_enum iaxxx_pdm_aclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_aclk_texts),
                        pdm_aclk_texts);

static const char * const pdm_clr_texts[] = {
	"NONE",
	"CIC0",
	"CIC1",
	"CIC2",
	"CIC3",
	"CIC4",
	"CIC5",
	"CIC6",
};

static const struct soc_enum iaxxx_pdm_clr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_clr_texts),
                        pdm_clr_texts);

#define ENUM_NAME(NAME) (#NAME)

/* Port values are 16 bit systemID's of Ports
 * 15-12 bits are Pheripheral type which is 1
 * 11-7 bits are which pheripheral type like PCM, PDM
 * 6-0 bits are instance index of port.
 */
static const unsigned int io_port_value[] = {
	0x10E0, 0x10E1, 0x10E2, 0x10E3, 0x10E4, 0x10E5,
	0xFFFF,
	0x1100, 0x1101, 0x1102, 0x1103,
	0x1104, 0x1105, 0x1106, 0x1107,
	0x1108,
	0xFFFF,
};

static const char * const io_port_texts[] = {
	"PCM0", "PCM1", "PCM2", "PCM3", "PCM4", "PCM5",
	"SWR",
	"PDMI0", "PDMI1", "PDMI2", "PDMI3",
	"PDMI4", "PDMI5", "PDMI6", "PDMI7",
	"PDMO0",
	"NONE"
};

static const unsigned int str_id_rx_values[] = {0x0, 0x1,
	0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xFFFF };

static const unsigned int str_id_tx_values[] = { 0x8, 0x9,
	0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0xFFFF };

static const char * const str_id_rx_texts[] = {
	ENUM_NAME(STREAMID_0), ENUM_NAME(STREAMID_1), ENUM_NAME(STREAMID_2),
	ENUM_NAME(STREAMID_3), ENUM_NAME(STREAMID_4), ENUM_NAME(STREAMID_5),
	ENUM_NAME(STREAMID_6), ENUM_NAME(STREAMID_7), ENUM_NAME(STREAMID_NONE), };

static const char * const str_id_tx_texts[] = {
	ENUM_NAME(STREAMID_8), ENUM_NAME(STREAMID_9), ENUM_NAME(STREAMID_10),
	ENUM_NAME(STREAMID_11), ENUM_NAME(STREAMID_12), ENUM_NAME(STREAMID_13),
	ENUM_NAME(STREAMID_14), ENUM_NAME(STREAMID_15), ENUM_NAME(STREAMID_NONE), };

static const unsigned int strm_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xF, };

static const char * const strm_ch_idx_texts[] = {
	ENUM_NAME(STRM_CH0), ENUM_NAME(STRM_CH1), ENUM_NAME(STRM_CH2),
	ENUM_NAME(STRM_CH3), ENUM_NAME(STRM_CH4), ENUM_NAME(STRM_CH5),
	ENUM_NAME(STRM_CH6), ENUM_NAME(STRM_CH7), ENUM_NAME(STRM_CH8),
	ENUM_NAME(STRM_CH9), ENUM_NAME(STRM_CH10), ENUM_NAME(STRM_CH11),
	ENUM_NAME(STRM_CH_NONE), };

static const unsigned int port_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC,
	0xD, 0xE, 0xF, };

static const char * const port_ch_idx_texts[] = {
	ENUM_NAME(PORT_CH0), ENUM_NAME(PORT_CH1), ENUM_NAME(PORT_CH2),
	ENUM_NAME(PORT_CH3), ENUM_NAME(PORT_CH4), ENUM_NAME(PORT_CH5),
	ENUM_NAME(PORT_CH6), ENUM_NAME(PORT_CH7), ENUM_NAME(PORT_CH8),
	ENUM_NAME(PORT_CH9), ENUM_NAME(PORT_CH10), ENUM_NAME(PORT_CH11),
	ENUM_NAME(PORT_CH12), ENUM_NAME(PORT_CH13), ENUM_NAME(PORT_CH14),
	ENUM_NAME(PORT_CH15),};

static const unsigned int gain_ramp_value[] = {
	STEP_300, STEP_600, STEP_900, STEP_1200,
	STEP_1600, STEP_2000, STEP_INST, };

static const char * const gain_ramp_texts[] = {
	ENUM_NAME(STEP_300), ENUM_NAME(STEP_600), ENUM_NAME(STEP_900),
	ENUM_NAME(STEP_1200), ENUM_NAME(STEP_1600), ENUM_NAME(STEP_2000),
	ENUM_NAME(STEP_INST), };

static const unsigned int gain_rech_evnt_value[] = {
	0x0, 0x1,
};

static const char * const gain_rech_evnt_texts[] = {
	"off", "on",
};

static const unsigned int str_frm_len_values[] = {
	0x20, 0x40, 0xA0,
	0x140, 0x200, 0x280,
	0x640, 0xC80, 0x1900,

	0x40, 0x80, 0x140,
	0x280, 0x400, 0x500,
	0xC80, 0x1900, 0x3200,

	0x60, 0xC0, 0x1E0,
	0x3C0, 0x600, 0x780,
	0x12C0, 0x2580, 0x4B00,

	0x80, 0x100, 0x280,
	0x500, 0x800, 0xA00,
	0x1900, 0x3200, 0x6400,

	0xB0, 0x160, 0x372,
	0x6E0, 0xB00, 0xDC0,
	0x2260, 0x44C0, 0x8980,

	0xC0, 0x180, 0x3C0,
	0x780, 0xC00, 0xF00,
	0x2580, 0x4B00, 0x9600,

	0x180, 0x300, 0x780,
	0xF00, 0x1800, 0x1E00,
	0x4B00, 0x9600,

	0x300, 0x600, 0xF00,
	0x1E00, 0x3000, 0x3C00,
	0x9600,
};

static const char * const str_frm_len_text[] = {
	ENUM_NAME(8K_1MS), ENUM_NAME(8K_2MS), ENUM_NAME(8K_5MS),
	ENUM_NAME(8K_10MS), ENUM_NAME(8K_16MS), ENUM_NAME(8K_20MS),
	ENUM_NAME(8K_50MS), ENUM_NAME(8K_100MS), ENUM_NAME(8K_200MS),

	ENUM_NAME(16K_1MS), ENUM_NAME(16K_2MS), ENUM_NAME(16K_5MS),
	ENUM_NAME(16K_10MS), ENUM_NAME(16K_16MS), ENUM_NAME(16K_20MS),
	ENUM_NAME(16K_50MS), ENUM_NAME(16K_100MS), ENUM_NAME(16K_200MS),

	ENUM_NAME(24K_1MS), ENUM_NAME(24K_2MS), ENUM_NAME(24K_5MS),
	ENUM_NAME(24K_10MS), ENUM_NAME(24K_16MS), ENUM_NAME(24K_20MS),
	ENUM_NAME(24K_50MS), ENUM_NAME(24K_100MS), ENUM_NAME(24K_200MS),

	ENUM_NAME(32K_1MS), ENUM_NAME(32K_2MS), ENUM_NAME(32K_5MS),
	ENUM_NAME(32K_10MS), ENUM_NAME(32K_16MS), ENUM_NAME(32K_20MS),
	ENUM_NAME(32K_50MS), ENUM_NAME(32K_100MS), ENUM_NAME(32K_200MS),

	ENUM_NAME(44.1K_1MS), ENUM_NAME(44.1K_2MS), ENUM_NAME(44.1K_5MS),
	ENUM_NAME(44.1K_10MS), ENUM_NAME(44.1K_16MS), ENUM_NAME(44.1K_20MS),
	ENUM_NAME(44.1K_50MS), ENUM_NAME(44.1K_100MS), ENUM_NAME(44.1K_200MS),

	ENUM_NAME(48K_1MS), ENUM_NAME(48K_2MS), ENUM_NAME(48K_5MS),
	ENUM_NAME(48K_10MS), ENUM_NAME(48K_16MS), ENUM_NAME(48K_20MS),
	ENUM_NAME(48K_50MS), ENUM_NAME(48K_100MS), ENUM_NAME(48K_200MS),

	ENUM_NAME(96K_1MS), ENUM_NAME(96K_2MS), ENUM_NAME(96K_5MS),
	ENUM_NAME(96K_10MS), ENUM_NAME(96K_16MS), ENUM_NAME(96K_20MS),
	ENUM_NAME(96K_50MS), ENUM_NAME(96K_100MS),

	ENUM_NAME(192K_1MS), ENUM_NAME(192K_2MS), ENUM_NAME(192K_5MS),
	ENUM_NAME(192K_10MS), ENUM_NAME(192K_16MS), ENUM_NAME(192K_20MS),
	ENUM_NAME(192K_50MS),
};

static const char * const str_rate_text[] = {
	ENUM_NAME(RATE_8K),
	ENUM_NAME(RATE_11P025K),
	ENUM_NAME(RATE_12K),
	ENUM_NAME(RATE_16K),
	ENUM_NAME(RATE_22P050K),
	ENUM_NAME(RATE_24K),
	ENUM_NAME(RATE_32K),
	ENUM_NAME(RATE_44P1K),
	ENUM_NAME(RATE_48K),
	ENUM_NAME(RATE_96K),
	ENUM_NAME(RATE_192K),
	ENUM_NAME(RATE_INVALID),
};

static const unsigned int str_rate_values[] = {
	RATE_8K,
	RATE_11P025K,
	RATE_12K,
	RATE_16K,
	RATE_22P050K,
	RATE_24K,
	RATE_32K,
	RATE_44P1K,
	RATE_48K,
	RATE_96K,
	RATE_192K,
	RATE_INVALID,
};

/* supported stream encodings */
static const unsigned int str_enc_values[] = {
	ENCODING_OPAQUE,
	ENCODING_AFLOAT,
	ENCODING_G711A,
	ENCODING_G711U,
	ENCODING_FLOAT,
	ENCODING_Q15,
	ENCODING_Q16,
	ENCODING_Q17,
	ENCODING_Q18,
	ENCODING_Q19,
	ENCODING_Q20,
	ENCODING_Q21,
	ENCODING_Q22,
	ENCODING_Q23,
	ENCODING_Q24,
	ENCODING_Q25,
	ENCODING_Q26,
	ENCODING_Q27,
	ENCODING_Q28,
	ENCODING_Q29,
	ENCODING_Q30,
	ENCODING_Q31,
	ENCODING_ERROR,
};

static const char * const str_enc_text[] = {
	ENUM_NAME(ENCODING_OPAQUE),
	ENUM_NAME(ENCODING_AFLOAT),
	ENUM_NAME(ENCODING_G711A),
	ENUM_NAME(ENCODING_G711U),
	ENUM_NAME(ENCODING_FLOAT),
	ENUM_NAME(ENCODING_Q15),
	ENUM_NAME(ENCODING_Q16),
	ENUM_NAME(ENCODING_Q17),
	ENUM_NAME(ENCODING_Q18),
	ENUM_NAME(ENCODING_Q19),
	ENUM_NAME(ENCODING_Q20),
	ENUM_NAME(ENCODING_Q21),
	ENUM_NAME(ENCODING_Q22),
	ENUM_NAME(ENCODING_Q23),
	ENUM_NAME(ENCODING_Q24),
	ENUM_NAME(ENCODING_Q25),
	ENUM_NAME(ENCODING_Q26),
	ENUM_NAME(ENCODING_Q27),
	ENUM_NAME(ENCODING_Q28),
	ENUM_NAME(ENCODING_Q29),
	ENUM_NAME(ENCODING_Q30),
	ENUM_NAME(ENCODING_Q31),
	ENUM_NAME(ENCODING_ERROR),
};

static const char * const channel_off_on_text[] = {
	"Off", "On"
};

static const unsigned int channel_off_on_value[] = { 0, 1 };


static const char * const TX_0_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx0On", "Rx1Tx0On", "Rx2Tx0On", "Rx3Tx0On", "Rx4Tx0On",
	"Rx5Tx0On", "Rx6Tx0On", "Rx7Tx0On", "Rx8Tx0On", "Rx9Tx0On",
	"Rx10Tx0On", "Rx11Tx0On", "Rx12Tx0On", "Rx13Tx0On", "Rx14Tx0On",
	"Rx15Tx0On",
	"Plgin0Tx0On", "Plgin1Tx0On", "Plgin2Tx0On", "Plgin3Tx0On",
	"Plgin4Tx0On", "Plgin5Tx0On", "Plgin6Tx0On", "Plgin7Tx0On",
	"iMix0Tx0On", "iMix1Tx0On", "iMix2Tx0On", "iMix3Tx0On",
	"iBuf0Tx0On", "iBuf1Tx0On", "iBuf2Tx0On", "iBuf3Tx0On",
};

static const char * const TX_1_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx1On", "Rx1Tx1On", "Rx2Tx1On", "Rx3Tx1On",	"Rx4Tx1On",
	"Rx5Tx1On", "Rx6Tx1On", "Rx7Tx1On", "Rx8Tx1On",	"Rx9Tx1On",
	"Rx10Tx1On", "Rx11Tx1On", "Rx12Tx1On", "Rx13Tx1On",	"Rx14Tx1On",
	"Rx15Tx1On",
	"Plgin0Tx1On", "Plgin1Tx1On", "Plgin2Tx1On", "Plgin3Tx1On",
	"Plgin4Tx1On", "Plgin5Tx1On", "Plgin6Tx1On", "Plgin7Tx1On",
	"iMix0Tx1On", "iMix1Tx1On", "iMix2Tx1On", "iMix3Tx1On",
	"iBuf0Tx1On", "iBuf1Tx1On", "iBuf2Tx1On", "iBuf3Tx1On",
};

static const char * const TX_2_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx2On", "Rx1Tx2On", "Rx2Tx2On", "Rx3Tx2On",	"Rx4Tx2On",
	"Rx5Tx2On", "Rx6Tx2On", "Rx7Tx2On", "Rx8Tx2On",	"Rx9Tx2On",
	"Rx10Tx2On", "Rx11Tx2On", "Rx12Tx2On", "Rx13Tx2On",	"Rx14Tx2On",
	"Rx15Tx2On",
	"Plgin0Tx2On", "Plgin1Tx2On", "Plgin2Tx2On", "Plgin3Tx2On",
	"Plgin4Tx2On", "Plgin5Tx2On", "Plgin6Tx2On", "Plgin7Tx2On",
	"iMix0Tx2On", "iMix1Tx2On", "iMix2Tx2On", "iMix3Tx2On",
	"iBuf0Tx2On", "iBuf1Tx2On", "iBuf2Tx2On", "iBuf3Tx2On",
};

static const char * const TX_3_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx3On", "Rx1Tx3On", "Rx2Tx3On", "Rx3Tx3On",	"Rx4Tx3On",
	"Rx5Tx3On", "Rx6Tx3On", "Rx7Tx3On", "Rx8Tx3On",	"Rx9Tx3On",
	"Rx10Tx3On", "Rx11Tx3On", "Rx12Tx3On", "Rx13Tx3On",	"Rx14Tx3On",
	"Rx15Tx3On",
	"Plgin0Tx3On", "Plgin1Tx3On", "Plgin2Tx3On", "Plgin3Tx3On",
	"Plgin4Tx3On", "Plgin5Tx3On", "Plgin6Tx3On", "Plgin7Tx3On",
	"iMix0Tx3On", "iMix1Tx3On", "iMix2Tx3On", "iMix3Tx3On",
	"iBuf0Tx3On", "iBuf1Tx3On", "iBuf2Tx3On", "iBuf3Tx3On",
};

static const char * const TX_4_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx4On", "Rx1Tx4On", "Rx2Tx4On", "Rx3Tx4On",	"Rx4Tx4On",
	"Rx5Tx4On", "Rx6Tx4On", "Rx7Tx4On", "Rx8Tx4On",	"Rx9Tx4On",
	"Rx10Tx4On", "Rx11Tx4On", "Rx12Tx4On", "Rx13Tx4On",	"Rx14Tx4On",
	"Rx15Tx4On",
	"Plgin0Tx4On", "Plgin1Tx4On", "Plgin2Tx4On", "Plgin3Tx4On",
	"Plgin4Tx4On", "Plgin5Tx4On", "Plgin6Tx4On", "Plgin7Tx4On",
	"iMix0Tx4On", "iMix1Tx4On", "iMix2Tx4On", "iMix3Tx4On",
	"iBuf0Tx4On", "iBuf1Tx4On", "iBuf2Tx4On", "iBuf3Tx4On",
};

static const char * const TX_5_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx5On", "Rx1Tx5On", "Rx2Tx5On", "Rx3Tx5On",	"Rx4Tx5On",
	"Rx5Tx5On", "Rx6Tx5On", "Rx7Tx5On", "Rx8Tx5On",	"Rx9Tx5On",
	"Rx10Tx5On", "Rx11Tx5On", "Rx12Tx5On", "Rx13Tx5On",	"Rx14Tx5On",
	"Rx15Tx5On",
	"Plgin0Tx5On", "Plgin1Tx5On", "Plgin2Tx5On", "Plgin3Tx5On",
	"Plgin4Tx5On", "Plgin5Tx5On", "Plgin6Tx5On", "Plgin7Tx5On",
	"iMix0Tx5On", "iMix1Tx5On", "iMix2Tx5On", "iMix3Tx5On",
	"iBuf0Tx5On", "iBuf1Tx5On", "iBuf2Tx5On", "iBuf3Tx5On",
};

static const char * const TX_6_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx6On", "Rx1Tx6On", "Rx2Tx6On", "Rx3Tx6On",	"Rx4Tx6On",
	"Rx5Tx6On", "Rx6Tx6On", "Rx7Tx6On", "Rx8Tx6On",	"Rx9Tx6On",
	"Rx10Tx6On", "Rx11Tx6On", "Rx12Tx6On", "Rx13Tx6On",	"Rx14Tx6On",
	"Rx15Tx6On",
	"Plgin0Tx6On", "Plgin1Tx6On", "Plgin2Tx6On", "Plgin3Tx6On",
	"Plgin4Tx6On", "Plgin5Tx6On", "Plgin6Tx6On", "Plgin7Tx6On",
	"iMix0Tx6On", "iMix1Tx6On", "iMix2Tx6On", "iMix3Tx6On",
	"iBuf0Tx6On", "iBuf1Tx6On", "iBuf2Tx6On", "iBuf3Tx6On",
};

static const char * const TX_7_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx7On", "Rx1Tx7On", "Rx2Tx7On", "Rx3Tx7On",	"Rx4Tx7On",
	"Rx5Tx7On", "Rx6Tx7On", "Rx7Tx7On", "Rx8Tx7On",	"Rx9Tx7On",
	"Rx10Tx7On", "Rx11Tx7On", "Rx12Tx7On", "Rx13Tx7On",	"Rx14Tx7On",
	"Rx15Tx7On",
	"Plgin0Tx7On", "Plgin1Tx7On", "Plgin2Tx7On", "Plgin3Tx7On",
	"Plgin4Tx7On", "Plgin5Tx7On", "Plgin6Tx7On", "Plgin7Tx7On",
	"iMix0Tx7On", "iMix1Tx7On", "iMix2Tx7On", "iMix3Tx7On",
	"iBuf0Tx7On", "iBuf1Tx7On", "iBuf2Tx7On", "iBuf3Tx7On",
};

static const char * const TX_8_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx8On", "Rx1Tx8On", "Rx2Tx8On", "Rx3Tx8On",	"Rx4Tx8On",
	"Rx5Tx8On", "Rx6Tx8On", "Rx7Tx8On", "Rx8Tx8On",	"Rx9Tx8On",
	"Rx10Tx8On", "Rx11Tx8On", "Rx12Tx8On", "Rx13Tx8On",	"Rx14Tx8On",
	"Rx15Tx8On",
	"Plgin0Tx8On", "Plgin1Tx8On", "Plgin2Tx8On", "Plgin3Tx8On",
	"Plgin4Tx8On", "Plgin5Tx8On", "Plgin6Tx8On", "Plgin7Tx8On",
	"iMix0Tx8On", "iMix1Tx8On", "iMix2Tx8On", "iMix3Tx8On",
	"iBuf0Tx8On", "iBuf1Tx8On", "iBuf2Tx8On", "iBuf3Tx8On",
};

static const char * const TX_9_rx_off_on_texts[] = {
	"Off", "Rx0Tx9On", "Rx1Tx9On", "Rx2Tx9On", "Rx3Tx9On", "Rx4Tx9On",
	"Rx5Tx9On", "Rx6Tx9On", "Rx7Tx9On", "Rx8Tx9On",	"Rx9Tx9On",
	"Rx10Tx9On", "Rx11Tx9On", "Rx12Tx9On", "Rx13Tx9On",	"Rx14Tx9On",
	"Rx15Tx9On",
	"Plgin0Tx9On", "Plgin1Tx9On", "Plgin2Tx9On", "Plgin3Tx9On",
	"Plgin4Tx9On", "Plgin5Tx9On", "Plgin6Tx9On", "Plgin7Tx9On",
	"iMix0Tx9On", "iMix1Tx9On", "iMix2Tx9On", "iMix3Tx9On",
	"iBuf0Tx9On", "iBuf1Tx9On", "iBuf2Tx9On", "iBuf3Tx9On",
};

static const char * const TX_10_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx10On", "Rx1Tx10On", "Rx2Tx10On", "Rx3Tx10On",	"Rx4Tx10On",
	"Rx5Tx10On", "Rx6Tx10On", "Rx7Tx10On", "Rx8Tx10On",	"Rx9Tx10On",
	"Rx10Tx10On", "Rx11Tx10On", "Rx12Tx10On", "Rx13Tx10On",	"Rx14Tx10On",
	"Rx15Tx10On",
	"Plgin0Tx10On", "Plgin1Tx10On", "Plgin2Tx10On", "Plgin3Tx10On",
	"Plgin4Tx10On", "Plgin5Tx10On", "Plgin6Tx10On", "Plgin7Tx10On",
	"iMix0Tx10On", "iMix1Tx10On", "iMix2Tx10On", "iMix3Tx10On",
	"iBuf0Tx10On", "iBuf1Tx10On", "iBuf2Tx10On", "iBuf3Tx10On",
};

static const char * const TX_11_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx11On", "Rx1Tx11On", "Rx2Tx11On", "Rx3Tx11On",	"Rx4Tx11On",
	"Rx5Tx11On", "Rx6Tx11On", "Rx7Tx11On", "Rx8Tx11On",	"Rx9Tx11On",
	"Rx10Tx11On", "Rx11Tx11On", "Rx12Tx11On", "Rx13Tx11On",	"Rx14Tx11On",
	"Rx15Tx11On",
	"Plgin0Tx11On", "Plgin1Tx11On", "Plgin2Tx11On", "Plgin3Tx11On",
	"Plgin4Tx11On", "Plgin5Tx11On", "Plgin6Tx11On", "Plgin7Tx11On",
	"iMix0Tx11On", "iMix1Tx11On", "iMix2Tx11On", "iMix3Tx11On",
	"iBuf0Tx11On", "iBuf1Tx11On", "iBuf2Tx11On", "iBuf3Tx11On",
};

static const char * const TX_12_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx12On", "Rx1Tx12On", "Rx2Tx12On", "Rx3Tx12On", "Rx4Tx12On",
	"Rx5Tx12On", "Rx6Tx12On", "Rx7Tx12On", "Rx8Tx12On",	"Rx9Tx12On",
	"Rx10Tx12On", "Rx11Tx12On", "Rx12Tx12On", "Rx13Tx12On",	"Rx14Tx12On",
	"Rx15Tx12On",
	"Plgin0Tx12On", "Plgin1Tx12On", "Plgin2Tx12On", "Plgin3Tx12On",
	"Plgin4Tx12On", "Plgin5Tx12On", "Plgin6Tx12On", "Plgin7Tx12On",
	"iMix0Tx12On", "iMix1Tx12On", "iMix2Tx12On", "iMix3Tx12On",
	"iBuf0Tx12On", "iBuf1Tx12On", "iBuf2Tx12On", "iBuf3Tx12On",
};

static const char * const TX_13_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx13On", "Rx1Tx13On", "Rx2Tx13On", "Rx3Tx13On",	"Rx4Tx13On",
	"Rx5Tx13On", "Rx6Tx13On", "Rx7Tx13On", "Rx8Tx13On",	"Rx9Tx13On",
	"Rx10Tx13On", "Rx11Tx13On", "Rx12Tx13On", "Rx13Tx13On",	"Rx14Tx13On",
	"Rx15Tx13On",
	"Plgin0Tx13On", "Plgin1Tx13On", "Plgin2Tx13On", "Plgin3Tx13On",
	"Plgin4Tx13On", "Plgin5Tx13On", "Plgin6Tx13On", "Plgin7Tx13On",
	"iMix0Tx13On", "iMix1Tx13On", "iMix2Tx13On", "iMix3Tx13On",
	"iBuf0Tx13On", "iBuf1Tx13On", "iBuf2Tx13On", "iBuf3Tx13On",
};

static const char * const TX_14_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx14On", "Rx1Tx14On", "Rx2Tx14On", "Rx3Tx14On",	"Rx4Tx14On",
	"Rx5Tx14On", "Rx6Tx14On", "Rx7Tx14On", "Rx8Tx14On",	"Rx9Tx14On",
	"Rx10Tx14On", "Rx11Tx14On", "Rx12Tx14On", "Rx13Tx14On",	"Rx14Tx14On",
	"Rx15Tx14On",
	"Plgin0Tx14On", "Plgin1Tx14On", "Plgin2Tx14On", "Plgin3Tx14On",
	"Plgin4Tx14On", "Plgin5Tx14On", "Plgin6Tx14On", "Plgin7Tx14On",
	"iMix0Tx14On", "iMix1Tx14On", "iMix2Tx14On", "iMix3Tx14On",
	"iBuf0Tx14On", "iBuf1Tx14On", "iBuf2Tx14On", "iBuf3Tx14On",
};

static const char * const TX_15_rx_off_on_texts[] = {
	"Off",
	"Rx0Tx15On", "Rx1Tx15On", "Rx2Tx15On", "Rx3Tx15On",	"Rx4Tx15On",
	"Rx5Tx15On", "Rx6Tx15On", "Rx7Tx15On", "Rx8Tx15On",	"Rx9Tx15On",
	"Rx10Tx15On", "Rx11Tx15On", "Rx12Tx15On", "Rx13Tx15On",	"Rx14Tx15On",
	"Rx15Tx15On",
	"Plgin0Tx15On", "Plgin1Tx15On", "Plgin2Tx15On", "Plgin3Tx15On",
	"Plgin4Tx15On", "Plgin5Tx15On", "Plgin6Tx15On", "Plgin7Tx15On",
	"iMix0Tx15On", "iMix1Tx15On", "iMix2Tx15On", "iMix3Tx15On",
	"iBuf0Tx15On", "iBuf1Tx15On", "iBuf2Tx15On", "iBuf3Tx15On",
};

static const char * const iaxxx_update_block_texts[] = {
	"OFF", "ON"
};

static const struct soc_enum iaxxx_update_block_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(iaxxx_update_block_texts),
			iaxxx_update_block_texts);

static const char * const iaxxx_route_status_texts[] = {
	"InActive", "Active"
};

static const struct soc_enum iaxxx_route_status_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(iaxxx_route_status_texts),
			iaxxx_route_status_texts);

static int iaxxxcore_get_update_block0(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int iaxxxcore_get_update_block1(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
static int iaxxxcore_get_update_block2(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
static int iaxxx_get_pdm_bclk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_bclk;
	return 0;

}

static int iaxxx_put_pdm_bclk(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_bclk = ucontrol->value.enumerated.item[0];
	return 0;
}

static int iaxxx_get_pdm_aclk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_aclk;
	return 0;

}

static int iaxxx_put_pdm_aclk(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_aclk = ucontrol->value.enumerated.item[0];
	return 0;
}

static int iaxxxcore_put_update_block0(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 status = 0;
	int ret = 0;

	pr_debug("enter %s connection\n", __func__);

	if (ucontrol->value.enumerated.item[0]) {
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
				&status, IAXXX_BLOCK_0);
	}

	return ret;
}

static int iaxxxcore_put_update_block1(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 status = 0;
	int ret = 0;

	pr_debug("enter %s connection\n", __func__);

	if (ucontrol->value.enumerated.item[0]) {
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
				&status, IAXXX_BLOCK_1);
	}

	return ret;
}

static int iaxxxcore_put_update_block2(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 status = 0;
	int ret = 0;

	pr_debug("enter %s connection\n", __func__);

	if (ucontrol->value.enumerated.item[0]) {
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
				&status, IAXXX_BLOCK_2);
	}

	return ret;
}

static int iaxxx_put_route_status(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct device *dev = iaxxx->dev_parent;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;

	pr_debug("enter %s connection\n", __func__);
	if (ucontrol->value.enumerated.item[0]) {
		msleep(IAXXX_FW_DELAY);
		dev_info(dev, "Route active request\n");
		ret = iaxxx_fw_notifier_call(priv->dev,
				IAXXX_EV_ROUTE_ACTIVE, NULL);
		if (ret)
			dev_err(dev, "not able to restart tunneling\n");
	}
	priv->route_status = ucontrol->value.enumerated.item[0];

	return ret;
}

static int iaxxx_get_route_status(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	ucontrol->value.enumerated.item[0] = priv->route_status;
	return 0;

}
static const DECLARE_TLV_DB_SCALE(gn_ch_ep_tlv, -1200, 100, 0);

#define IAXXX_CH_MGR_DAPM_CTLS(channel, channel_name) \
static const struct soc_enum channel##_en_enum = \
		SOC_ENUM_SINGLE(IAXXX_CH_HDR_CH_EN_ADDR, channel,\
		ARRAY_SIZE(channel_off_on_text), channel_off_on_text); \
static const struct snd_kcontrol_new channel##_mux = \
	SOC_DAPM_ENUM(channel_name "En", channel##_en_enum)

#define IAXXX_RX_CHMGR_PORT_DAPM_CTLS(channel, channel_name) \
static const SOC_VALUE_ENUM_SINGLE_DECL(channel##_rx_port_enum, \
			IAXXX_IN_CH_GRP_CH_PORT_REG(channel), \
			IAXXX_IN_CH_GRP_CH_PORT_ID_POS, \
			(IAXXX_IN_CH_GRP_CH_PORT_ID_MASK >> \
			IAXXX_IN_CH_GRP_CH_PORT_ID_POS), \
			io_port_texts, io_port_value); \
static const struct snd_kcontrol_new channel##_port =	\
	SOC_DAPM_ENUM(channel_name "Chan Port", channel##_rx_port_enum)

#define IAXXX_TX_CHMGR_PORT_DAPM_CTLS(channel, channel_name) \
static const SOC_VALUE_ENUM_SINGLE_DECL(channel##_tx_port_enum, \
			IAXXX_OUT_CH_GRP_CH_PORT_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_PORT_ID_POS, \
			(IAXXX_OUT_CH_GRP_CH_PORT_ID_MASK >> \
			IAXXX_OUT_CH_GRP_CH_PORT_ID_POS), \
			io_port_texts, io_port_value); \
static const struct snd_kcontrol_new channel##_port =	\
	SOC_DAPM_ENUM(channel_name "Chan Port", channel##_tx_port_enum)

#define IAXXX_CH_RX_TO_TX_DAPM_CTLS(channel, channel_name) \
static const SOC_ENUM_SINGLE_DECL(channel##_rx_en_enum, \
			SND_SOC_NOPM, 0, channel##_rx_off_on_texts); \
static const struct snd_kcontrol_new channel##_rx_mux =	\
	SOC_DAPM_ENUM(channel_name "PortTxEn", channel##_rx_en_enum)

#define IAXXX_CH_MGR_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " En", SND_SOC_NOPM, 0, 0, \
						&(channel##_mux)), \
	SND_SOC_DAPM_MUX(channel_name " Port", SND_SOC_NOPM, 0, 0, \
						&(channel##_port))

#define IAXXX_CH_RX_TO_TX_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " PortMux En", SND_SOC_NOPM, 0, 0, \
						&(channel##_rx_mux))

#define IAXXXCORE_RX_CHMGR_ENUM(channel, num) \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_str_id_enum, \
			IAXXX_IN_CH_GRP_CH_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_CTRL_STR_ID_POS, \
			(IAXXX_IN_CH_GRP_CH_CTRL_STR_ID_MASK >> \
			IAXXX_IN_CH_GRP_CH_CTRL_STR_ID_POS), \
			str_id_rx_texts, str_id_rx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_str_ch_idx_enum, \
			IAXXX_IN_CH_GRP_CH_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_CTRL_CH_INDEX_POS, \
			(IAXXX_IN_CH_GRP_CH_CTRL_CH_INDEX_MASK >> \
			IAXXX_IN_CH_GRP_CH_CTRL_CH_INDEX_POS), \
			strm_ch_idx_texts, strm_ch_idx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_rmp_enum, \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS, \
			(IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS), \
			gain_ramp_texts, gain_ramp_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_evnt_enum, \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS, \
			(IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS), \
			gain_rech_evnt_texts, gain_rech_evnt_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_enc_enum, \
			IAXXX_IN_CH_GRP_OUT_FMT_REG(num), \
			IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_POS, \
			(IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_MASK >> \
			IAXXX_IN_CH_GRP_OUT_FMT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_sr_enum, \
			IAXXX_IN_CH_GRP_OUT_FMT_REG(num), \
			IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_POS, \
			(IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_MASK >> \
			IAXXX_IN_CH_GRP_OUT_FMT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_frm_len_enum, \
			IAXXX_IN_CH_GRP_OUT_FMT_REG(num), \
			IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_POS, \
			(IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_MASK >> \
			IAXXX_IN_CH_GRP_OUT_FMT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_port_ch_idx_enum, \
			IAXXX_IN_CH_GRP_CH_PORT_REG(num), \
			IAXXX_IN_CH_GRP_CH_PORT_CH_IDX_POS, \
			(IAXXX_IN_CH_GRP_CH_PORT_CH_IDX_MASK >> \
			IAXXX_IN_CH_GRP_CH_PORT_CH_IDX_POS), \
			port_ch_idx_texts, port_ch_idx_values)


IAXXXCORE_RX_CHMGR_ENUM(RX_0, 0);
IAXXXCORE_RX_CHMGR_ENUM(RX_1, 1);
IAXXXCORE_RX_CHMGR_ENUM(RX_2, 2);
IAXXXCORE_RX_CHMGR_ENUM(RX_3, 3);
IAXXXCORE_RX_CHMGR_ENUM(RX_4, 4);
IAXXXCORE_RX_CHMGR_ENUM(RX_5, 5);
IAXXXCORE_RX_CHMGR_ENUM(RX_6, 6);
IAXXXCORE_RX_CHMGR_ENUM(RX_7, 7);
IAXXXCORE_RX_CHMGR_ENUM(RX_8, 8);
IAXXXCORE_RX_CHMGR_ENUM(RX_9, 9);
IAXXXCORE_RX_CHMGR_ENUM(RX_10, 10);
IAXXXCORE_RX_CHMGR_ENUM(RX_11, 11);
IAXXXCORE_RX_CHMGR_ENUM(RX_12, 12);
IAXXXCORE_RX_CHMGR_ENUM(RX_13, 13);
IAXXXCORE_RX_CHMGR_ENUM(RX_14, 14);
IAXXXCORE_RX_CHMGR_ENUM(RX_15, 15);


#define IAXXXCORE_RX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM(channel_name "Chan Strm Id", channel##_str_id_enum), \
	SOC_ENUM(channel_name "Chan Strm Idx", channel##_str_ch_idx_enum), \
	SOC_ENUM(channel_name "Chan GnRmp", channel##_gn_rmp_enum), \
	SOC_ENUM(channel_name "Chan OpEpEnc", channel##_enc_enum), \
	SOC_ENUM(channel_name "Chan OpEpSr", channel##_sr_enum), \
	SOC_ENUM(channel_name "Chan OpEpFrLn", channel##_frm_len_enum), \
	SOC_SINGLE(channel_name "Chan Port Dir", \
			IAXXX_IN_CH_GRP_CH_PORT_REG(channel), \
			IAXXX_IN_CH_GRP_CH_PORT_CH_DIR_POS, 1, 0), \
	SOC_SINGLE_TLV(channel_name "Ch EpGain", \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(channel), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS, \
			0xFF, 0, gn_ch_ep_tlv), \
	SOC_ENUM(channel_name "Chan Port Idx", channel##_port_ch_idx_enum), \
	SOC_ENUM(channel_name "Chan GnReEvt", channel##_gn_evnt_enum), \
	SOC_SINGLE(channel_name "Chan Gain En", \
			IAXXX_CH_HDR_CH_GAIN_ADDR, channel, 1, 0)

IAXXX_CH_MGR_DAPM_CTLS(RX_0, "Rx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_1, "Rx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_2, "Rx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_3, "Rx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_4, "Rx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_5, "Rx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_6, "Rx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_7, "Rx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_8, "Rx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_9, "Rx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_10, "Rx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_11, "Rx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_12, "Rx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_13, "Rx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_14, "Rx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_15, "Rx15 Mux");

IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_0, "Rx0 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_1, "Rx1 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_2, "Rx2 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_3, "Rx3 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_4, "Rx4 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_5, "Rx5 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_6, "Rx6 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_7, "Rx7 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_8, "Rx8 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_9, "Rx9 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_10, "Rx10 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_11, "Rx11 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_12, "Rx12 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_13, "Rx13 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_14, "Rx14 Mux");
IAXXX_RX_CHMGR_PORT_DAPM_CTLS(RX_15, "Rx15 Mux");


static const unsigned int str_mstr_id_values[] = {0x0, 0x1,
	0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
	0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0xFFFF, };

static const char * const str_mstr_id_texts[] = {
	ENUM_NAME(STREAMID_0), ENUM_NAME(STREAMID_1), ENUM_NAME(STREAMID_2),
	ENUM_NAME(STREAMID_3), ENUM_NAME(STREAMID_4), ENUM_NAME(STREAMID_5),
	ENUM_NAME(STREAMID_6), ENUM_NAME(STREAMID_7), ENUM_NAME(STREAMID_8),
	ENUM_NAME(STREAMID_9), ENUM_NAME(STREAMID_10), ENUM_NAME(STREAMID_11),
	ENUM_NAME(STREAMID_12), ENUM_NAME(STREAMID_13), ENUM_NAME(STREAMID_14),
	ENUM_NAME(STREAMID_15), ENUM_NAME(STREAMID_NONE),
};

static const unsigned int strm_pwr_mode_value[] = {
	0x0, 0x1, 0x2, };

static const char * const strm_pwr_mode_texts[] = {
	ENUM_NAME(STANDARD), ENUM_NAME(LOW_POWER), ENUM_NAME(LOW_POWER_VQ), };

static const unsigned int strm_asrc_mode_value[] = {
	0x0, 0x1, 0x2, 0x3, };

static const char * const strm_asrc_mode_texts[] = {
	ENUM_NAME(ASRC_ENABLE), ENUM_NAME(ASRC_DISABLE), ENUM_NAME(REDBOX_2-1),
	ENUM_NAME(REDBOX_1-2), };

  /* stream direction */
#define IAXXXCORE_STREAM_ENUM(stream) \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_asrc_mode_enum, \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS, \
			(IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK >> \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS), \
			strm_asrc_mode_texts, strm_asrc_mode_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_mstr_str_id_enum, \
			IAXXX_STR_GRP_STR_SYNC_REG(stream), \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS, \
			(IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK >> \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS), \
			str_mstr_id_texts, str_mstr_id_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_enc_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_PORT_ENCODING_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_PORT_ENCODING_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_PORT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_sr_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_frm_len_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values)

IAXXXCORE_STREAM_ENUM(STREAM0);
IAXXXCORE_STREAM_ENUM(STREAM1);
IAXXXCORE_STREAM_ENUM(STREAM2);
IAXXXCORE_STREAM_ENUM(STREAM3);
IAXXXCORE_STREAM_ENUM(STREAM4);
IAXXXCORE_STREAM_ENUM(STREAM5);
IAXXXCORE_STREAM_ENUM(STREAM6);
IAXXXCORE_STREAM_ENUM(STREAM7);
IAXXXCORE_STREAM_ENUM(STREAM8);
IAXXXCORE_STREAM_ENUM(STREAM9);
IAXXXCORE_STREAM_ENUM(STREAM10);
IAXXXCORE_STREAM_ENUM(STREAM11);
IAXXXCORE_STREAM_ENUM(STREAM12);
IAXXXCORE_STREAM_ENUM(STREAM13);
IAXXXCORE_STREAM_ENUM(STREAM14);
IAXXXCORE_STREAM_ENUM(STREAM15);

#define IAXXXCORE_STREAM_KCTRL(stream, strm_name) \
	SOC_ENUM(strm_name "ASRC Mode", stream##_asrc_mode_enum), \
	SOC_SINGLE(strm_name "En", IAXXX_STR_HDR_STR_EN_ADDR, \
					stream, 1, 0), \
	SOC_ENUM(strm_name "Master Strm Id", stream##_mstr_str_id_enum), \
	SOC_ENUM(strm_name "Format Enc", stream##_enc_enum), \
	SOC_ENUM(strm_name "Format Sr", stream##_sr_enum), \
	SOC_ENUM(strm_name "Format FrLn", stream##_frm_len_enum), \
	SOC_SINGLE(strm_name "droop comp en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS, 1, 0), \
	SOC_SINGLE(strm_name "DC block en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS, 1, 0), \
	SOC_SINGLE(strm_name "tone gen en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_POS, 1, 0), \
	SOC_SINGLE(strm_name "Strm Dir", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_STREAM_DIR_POS, 1, 0), \
	SOC_SINGLE(strm_name "inter strm delay", \
				IAXXX_STR_GRP_STR_SYNC_REG(stream), \
				IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_POS, \
				(1<<16) - 1, 0)


static const unsigned int tx_ch_in_value[] = {
	0x4020, 0x4060, 0x40A0, 0x40E0, 0x4120, 0x4160,
	0x41A0, 0x41E0, 0x4220, 0x4260, 0x42A0, 0x42E0,
	0x4320, 0x4360, 0x43A0, 0x43E0,
	/* Plugins 0 - 7 */
	0x3020, 0x3021, 0x3022, 0x3023, 0x3024, 0x3025, 0x3026, 0x3027,
	0x3060, 0x3061, 0x3062, 0x3063, 0x3064, 0x3065, 0x3066, 0x3067,
	0x30A0, 0x30A1, 0x30A2, 0x30A3, 0x30A4, 0x30A5, 0x30A6, 0x30A7,
	0x30E0, 0x30E1, 0x30E2, 0x30E3, 0x30E4, 0x30E5, 0x30E6, 0x30E7,
	0x3120, 0x3121, 0x3122, 0x3123, 0x3124, 0x3125, 0x3126, 0x3127,
	0x3160, 0x3161, 0x3162, 0x3163, 0x3164, 0x3165, 0x3166, 0x3167,
	0x31A0, 0x31A1, 0x31A2, 0x31A3, 0x31A4, 0x31A5, 0x31A6, 0x31A7,
	0x31E0, 0x31E1, 0x31E2, 0x31E3, 0x31E4, 0x31E5, 0x31E6, 0x31E7,
	/* IMIX */
	0x3420, 0x3460, 0x34A0, 0x34E0,
	/* IBUF */
	0x3620, 0x3660, 0x36A0, 0x36E0,
	0xFFFF,

};

static const char * const tx_ch_in_texts[] = {
	ENUM_NAME(RX0_ChanMgr), ENUM_NAME(RX1_ChanMgr),
	ENUM_NAME(RX2_ChanMgr), ENUM_NAME(RX3_ChanMgr),
	ENUM_NAME(RX4_ChanMgr), ENUM_NAME(RX5_ChanMgr),
	ENUM_NAME(RX6_ChanMgr), ENUM_NAME(RX7_ChanMgr),
	ENUM_NAME(RX8_ChanMgr), ENUM_NAME(RX9_ChanMgr),
	ENUM_NAME(RX10_ChanMgr), ENUM_NAME(RX11_ChanMgr),
	ENUM_NAME(RX12_ChanMgr), ENUM_NAME(RX13_ChanMgr),
	ENUM_NAME(RX14_ChanMgr), ENUM_NAME(RX15_ChanMgr),

	ENUM_NAME(plugin0Out0), ENUM_NAME(plugin0Out1),
	ENUM_NAME(plugin0Out2), ENUM_NAME(plugin0Out3),
	ENUM_NAME(plugin0Out4), ENUM_NAME(plugin0Out5),
	ENUM_NAME(plugin0Out6), ENUM_NAME(plugin0Out7),
	ENUM_NAME(plugin1Out0), ENUM_NAME(plugin1Out1),
	ENUM_NAME(plugin1Out2), ENUM_NAME(plugin1Out3),
	ENUM_NAME(plugin1Out4), ENUM_NAME(plugin1Out5),
	ENUM_NAME(plugin1Out6), ENUM_NAME(plugin1Out7),
	ENUM_NAME(plugin2Out0), ENUM_NAME(plugin2Out1),
	ENUM_NAME(plugin2Out2), ENUM_NAME(plugin2Out3),
	ENUM_NAME(plugin2Out4), ENUM_NAME(plugin2Out5),
	ENUM_NAME(plugin2Out6), ENUM_NAME(plugin2Out7),
	ENUM_NAME(plugin3Out0), ENUM_NAME(plugin3Out1),
	ENUM_NAME(plugin3Out2), ENUM_NAME(plugin3Out3),
	ENUM_NAME(plugin3Out4), ENUM_NAME(plugin3Out5),
	ENUM_NAME(plugin3Out6), ENUM_NAME(plugin3Out7),
	ENUM_NAME(plugin4Out0), ENUM_NAME(plugin4Out1),
	ENUM_NAME(plugin4Out2), ENUM_NAME(plugin4Out3),
	ENUM_NAME(plugin4Out4), ENUM_NAME(plugin4Out5),
	ENUM_NAME(plugin4Out6), ENUM_NAME(plugin4Out7),
	ENUM_NAME(plugin5Out0), ENUM_NAME(plugin5Out1),
	ENUM_NAME(plugin5Out2), ENUM_NAME(plugin5Out3),
	ENUM_NAME(plugin5Out4), ENUM_NAME(plugin5Out5),
	ENUM_NAME(plugin5Out6), ENUM_NAME(plugin5Out7),
	ENUM_NAME(plugin6Out0), ENUM_NAME(plugin6Out1),
	ENUM_NAME(plugin6Out2), ENUM_NAME(plugin6Out3),
	ENUM_NAME(plugin6Out4), ENUM_NAME(plugin6Out5),
	ENUM_NAME(plugin6Out6), ENUM_NAME(plugin6Out7),
	ENUM_NAME(plugin7Out0), ENUM_NAME(plugin7Out1),
	ENUM_NAME(plugin7Out2), ENUM_NAME(plugin7Out3),
	ENUM_NAME(plugin7Out4), ENUM_NAME(plugin7Out5),
	ENUM_NAME(plugin7Out6), ENUM_NAME(plugin7Out7),

	ENUM_NAME(Imix0Out0), ENUM_NAME(Imix1Out0),
	ENUM_NAME(Imix2Out0), ENUM_NAME(Imix3Out0),

	ENUM_NAME(Ibuf0Out0), ENUM_NAME(Ibuf1Out0),
	ENUM_NAME(Ibuf2Out0), ENUM_NAME(Ibuf3Out0),

	ENUM_NAME(UNKNOWN),
};

#define IAXXXCORE_TX_CHMGR_ENUM(channel) \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_str_id_enum, \
			IAXXX_OUT_CH_GRP_CH_CTRL_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_CTRL_STR_ID_POS, \
			(IAXXX_OUT_CH_GRP_CH_CTRL_STR_ID_MASK >> \
			IAXXX_OUT_CH_GRP_CH_CTRL_STR_ID_POS), \
			str_id_tx_texts, str_id_tx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_str_ch_idx_enum, \
			IAXXX_OUT_CH_GRP_CH_CTRL_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_CTRL_CH_INDEX_POS, \
			(IAXXX_OUT_CH_GRP_CH_CTRL_CH_INDEX_MASK >> \
			IAXXX_OUT_CH_GRP_CH_CTRL_CH_INDEX_POS), \
			strm_ch_idx_texts, strm_ch_idx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_rmp_enum, \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS, \
			(IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK >> \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS), \
			gain_ramp_texts, gain_ramp_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_evnt_enum, \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS, \
			(IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK >> \
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS), \
			gain_rech_evnt_texts, gain_rech_evnt_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_port_ch_idx_enum, \
			IAXXX_OUT_CH_GRP_CH_PORT_REG(channel), \
			IAXXX_OUT_CH_GRP_CH_PORT_CH_IDX_POS, \
			(IAXXX_OUT_CH_GRP_CH_PORT_CH_IDX_MASK >> \
			IAXXX_OUT_CH_GRP_CH_PORT_CH_IDX_POS), \
			port_ch_idx_texts, port_ch_idx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_ip_src_id_enum, \
			IAXXX_OUT_CH_GRP_IN_CONNECT_REG(channel), \
			IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS, \
			(IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_MASK >> \
			IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS), \
			tx_ch_in_texts, tx_ch_in_value)


IAXXXCORE_TX_CHMGR_ENUM(TX_0);
IAXXXCORE_TX_CHMGR_ENUM(TX_1);
IAXXXCORE_TX_CHMGR_ENUM(TX_2);
IAXXXCORE_TX_CHMGR_ENUM(TX_3);
IAXXXCORE_TX_CHMGR_ENUM(TX_4);
IAXXXCORE_TX_CHMGR_ENUM(TX_5);
IAXXXCORE_TX_CHMGR_ENUM(TX_6);
IAXXXCORE_TX_CHMGR_ENUM(TX_7);
IAXXXCORE_TX_CHMGR_ENUM(TX_8);
IAXXXCORE_TX_CHMGR_ENUM(TX_9);
IAXXXCORE_TX_CHMGR_ENUM(TX_10);
IAXXXCORE_TX_CHMGR_ENUM(TX_11);
IAXXXCORE_TX_CHMGR_ENUM(TX_12);
IAXXXCORE_TX_CHMGR_ENUM(TX_13);
IAXXXCORE_TX_CHMGR_ENUM(TX_14);
IAXXXCORE_TX_CHMGR_ENUM(TX_15);



#define IAXXXCORE_TX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM(channel_name "Chan Strm Id", channel##_str_id_enum), \
	SOC_ENUM(channel_name "Chan Port Idx", channel##_port_ch_idx_enum), \
	SOC_SINGLE(channel_name "Chan Port Dir", \
				IAXXX_OUT_CH_GRP_CH_PORT_REG(channel), \
				IAXXX_OUT_CH_GRP_CH_PORT_CH_DIR_POS, 1, 0), \
	SOC_SINGLE_TLV(channel_name "Ch EpGain", \
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS, \
				0xFF, 0, gn_ch_ep_tlv), \
	SOC_ENUM(channel_name "Chan GnReEvt", channel##_gn_evnt_enum), \
	SOC_ENUM(channel_name "Chan GnRmp", channel##_gn_rmp_enum), \
	SOC_ENUM(channel_name "Chan Strm Idx", channel##_str_ch_idx_enum), \
	SOC_ENUM(channel_name "ip src id", channel##_ip_src_id_enum), \
	SOC_SINGLE(channel_name "Chan Gain En", \
			IAXXX_CH_HDR_CH_GAIN_ADDR, channel, 1, 0)


IAXXX_CH_MGR_DAPM_CTLS(TX_0, "Tx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_1, "Tx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_2, "Tx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_3, "Tx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_4, "Tx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_5, "Tx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_6, "Tx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_7, "Tx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_8, "Tx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_9, "Tx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_10, "Tx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_11, "Tx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_12, "Tx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_13, "Tx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_14, "Tx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_15, "Tx15 Mux");

IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_0, "Tx0 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_1, "Tx1 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_2, "Tx2 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_3, "Tx3 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_4, "Tx4 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_5, "Tx5 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_6, "Tx6 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_7, "Tx7 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_8, "Tx8 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_9, "Tx9 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_10, "Tx10 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_11, "Tx11 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_12, "Tx12 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_13, "Tx13 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_14, "Tx14 Mux");
IAXXX_TX_CHMGR_PORT_DAPM_CTLS(TX_15, "Tx15 Mux");

IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_0, "Tx0");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_1, "Tx1");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_2, "Tx2");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_3, "Tx3");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_4, "Tx4");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_5, "Tx5");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_6, "Tx6");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_7, "Tx7");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_8, "Tx8");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_9, "Tx9");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_10, "Tx10");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_11, "Tx11");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_12, "Tx12");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_13, "Tx13");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_14, "Tx14");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_15, "Tx15");

/* FIXME , this should be just integer value */
static const unsigned int plugin_ctrl_opt_value[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, };

static const char * const plugin_ctrl_opt_text[] = {
	"none", "PkgPlgin1", "PkgPlgin2", "PkgPlgin3", "PkgPlgin4",
	"PkgPlgin5", "PkgPlgin6", "PkgPlgin7", "PkgPlgin8", };

static const unsigned int plugin_origin_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, };

static const char * const plugin_origin_idx_texts[] = {
	"none", "PkgPlgin1", "PkgPlgin2", "PkgPlgin3", "PkgPlgin4",
	"PkgPlgin5", "PkgPlgin6", "PkgPlgin7", "PkgPlgin8", };

static const char * const plugin_ip_ep_texts[] = {
	ENUM_NAME(RX0_ChanMgr), ENUM_NAME(RX1_ChanMgr),
	ENUM_NAME(RX2_ChanMgr), ENUM_NAME(RX3_ChanMgr),
	ENUM_NAME(RX4_ChanMgr), ENUM_NAME(RX5_ChanMgr),
	ENUM_NAME(RX6_ChanMgr), ENUM_NAME(RX7_ChanMgr),
	ENUM_NAME(RX8_ChanMgr), ENUM_NAME(RX9_ChanMgr),
	ENUM_NAME(RX10_ChanMgr), ENUM_NAME(RX11_ChanMgr),
	ENUM_NAME(RX12_ChanMgr), ENUM_NAME(RX13_ChanMgr),
	ENUM_NAME(RX14_ChanMgr), ENUM_NAME(RX15_ChanMgr),


	ENUM_NAME(plugin0Out0), ENUM_NAME(plugin0Out1),
	ENUM_NAME(plugin0Out2), ENUM_NAME(plugin0Out3),
	ENUM_NAME(plugin0Out4), ENUM_NAME(plugin0Out5),
	ENUM_NAME(plugin0Out6), ENUM_NAME(plugin0Out7),
	ENUM_NAME(plugin1Out0), ENUM_NAME(plugin1Out1),
	ENUM_NAME(plugin1Out2), ENUM_NAME(plugin1Out3),
	ENUM_NAME(plugin1Out4), ENUM_NAME(plugin1Out5),
	ENUM_NAME(plugin1Out6), ENUM_NAME(plugin1Out7),
	ENUM_NAME(plugin2Out0), ENUM_NAME(plugin2Out1),
	ENUM_NAME(plugin2Out2), ENUM_NAME(plugin2Out3),
	ENUM_NAME(plugin2Out4), ENUM_NAME(plugin2Out5),
	ENUM_NAME(plugin2Out6), ENUM_NAME(plugin2Out7),
	ENUM_NAME(plugin3Out0), ENUM_NAME(plugin3Out1),
	ENUM_NAME(plugin3Out2), ENUM_NAME(plugin3Out3),
	ENUM_NAME(plugin3Out4), ENUM_NAME(plugin3Out5),
	ENUM_NAME(plugin3Out6), ENUM_NAME(plugin3Out7),
	ENUM_NAME(plugin4Out0), ENUM_NAME(plugin4Out1),
	ENUM_NAME(plugin4Out2), ENUM_NAME(plugin4Out3),
	ENUM_NAME(plugin4Out4), ENUM_NAME(plugin4Out5),
	ENUM_NAME(plugin4Out6), ENUM_NAME(plugin4Out7),
	ENUM_NAME(plugin5Out0), ENUM_NAME(plugin5Out1),
	ENUM_NAME(plugin5Out2), ENUM_NAME(plugin5Out3),
	ENUM_NAME(plugin5Out4), ENUM_NAME(plugin5Out5),
	ENUM_NAME(plugin5Out6), ENUM_NAME(plugin5Out7),
	ENUM_NAME(plugin6Out0), ENUM_NAME(plugin6Out1),
	ENUM_NAME(plugin6Out2), ENUM_NAME(plugin6Out3),
	ENUM_NAME(plugin6Out4), ENUM_NAME(plugin6Out5),
	ENUM_NAME(plugin6Out6), ENUM_NAME(plugin6Out7),
	ENUM_NAME(plugin7Out0), ENUM_NAME(plugin7Out1),
	ENUM_NAME(plugin7Out2), ENUM_NAME(plugin7Out3),
	ENUM_NAME(plugin7Out4), ENUM_NAME(plugin7Out5),
	ENUM_NAME(plugin7Out6), ENUM_NAME(plugin7Out7),

	ENUM_NAME(Imix0Out0), ENUM_NAME(Imix1Out0),
	ENUM_NAME(Imix2Out0), ENUM_NAME(Imix3Out0),

	ENUM_NAME(Ibuf0Out0), ENUM_NAME(Ibuf1Out0),
	ENUM_NAME(Ibuf2Out0), ENUM_NAME(Ibuf3Out0),

	ENUM_NAME(UNKNOWN),
};


static const unsigned int plugin_ip_ep_values[] = {
	0x4020, 0x4060, 0x40A0, 0x40E0, 0x4120, 0x4160, 0x41A0, 0x41E0,
	0x4220, 0x4260, 0x42A0, 0x42E0, 0x4320, 0x4360, 0x43A0, 0x43E0,

	/* Plugins 0 - 7 */
	0x3020, 0x3021, 0x3022, 0x3023, 0x3024, 0x3025, 0x3026, 0x3027,
	0x3060, 0x3061, 0x3062, 0x3063, 0x3064, 0x3065, 0x3066, 0x3067,
	0x30A0, 0x30A1, 0x30A2, 0x30A3, 0x30A4, 0x30A5, 0x30A6, 0x30A7,
	0x30E0, 0x30E1, 0x30E2, 0x30E3, 0x30E4, 0x30E5, 0x30E6, 0x30E7,
	0x3120, 0x3121, 0x3122, 0x3123, 0x3124, 0x3125, 0x3126, 0x3127,
	0x3160, 0x3161, 0x3162, 0x3163, 0x3164, 0x3165, 0x3166, 0x3167,
	0x31A0, 0x31A1, 0x31A2, 0x31A3, 0x31A4, 0x31A5, 0x31A6, 0x31A7,
	0x31E0, 0x31E1, 0x31E2, 0x31E3, 0x31E4, 0x31E5, 0x31E6, 0x31E7,
	/* IMIX */
	0x3420, 0x3460, 0x34A0, 0x34E0,
	/* IBUF */
	0x3620, 0x3660, 0x36A0, 0x36E0,
	0xFFFF,
};

static const char * const pkg_id_texts[] = {
	"none" "pkg1", "pkg2", "pkg3", "pkg4", "pkg5", "pkg6",
	"pkg7", "pkg8", "pkg9", "pkg10", "pkg11", "pkg12",
	"pkg13", "pkg14", "pkg15", "pkg16",
};

static const unsigned int pkg_id_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA,
	0xB, 0xC, 0xD, 0xE, 0xF,
};

static const char * const priority_texts[] = {
	"pri0", "pri1", "pri2", "pri3", "pri4", "pri5", "pri6", "pri7",
	"pri8", "pri9", "pri10", "pri11", "pri12", "pri13", "pri14", "pri15",
};

static const unsigned int priority_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xF,
};


static int iaxxxcore_blk0_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[0].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk0_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[0].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[0].param_id_reg = reg;

	return 0;
}

static int iaxxxcore_blk0_get_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	if (iaxxx->plugin_param[0].param_id == IAXXX_MAX_VAL) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	iaxxx->plugin_param[0].param_val_reg = reg;

	ret = iaxxx_core_plg_get_param(iaxxx->dev_parent,
			iaxxx->plugin_param[0].param_id_reg,
			iaxxx->plugin_param[0].param_val_reg,
			iaxxx->plugin_param[0].param_id,
			&iaxxx->plugin_param[0].param_val,
			IAXXX_BLOCK_0);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[0].param_val;
	return ret;
}

static int iaxxxcore_blk0_put_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	iaxxx->plugin_param[0].param_val_reg = reg;
	iaxxx->plugin_param[0].param_val = ucontrol->value.integer.value[0];

	ret = iaxxx_core_plg_set_param(iaxxx->dev_parent,
			iaxxx->plugin_param[0].param_id_reg,
			iaxxx->plugin_param[0].param_val_reg,
			iaxxx->plugin_param[0].param_id,
			iaxxx->plugin_param[0].param_val,
			IAXXX_BLOCK_0);

	pr_debug("%s\n", __func__);
	return ret;
}

static int iaxxxcore_blk1_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[1].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk1_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[1].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[1].param_id_reg = reg;

	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk1_get_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	if (iaxxx->plugin_param[1].param_id == IAXXX_MAX_VAL) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}

	iaxxx->plugin_param[1].param_val_reg = reg;
	ret = iaxxx_core_plg_get_param(iaxxx->dev_parent,
			iaxxx->plugin_param[1].param_id_reg,
			iaxxx->plugin_param[1].param_val_reg,
			iaxxx->plugin_param[1].param_id,
			&iaxxx->plugin_param[1].param_val,
			IAXXX_BLOCK_1);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[1].param_val;
	pr_debug("%s\n", __func__);
	return ret;
}

static int iaxxxcore_blk1_put_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	iaxxx->plugin_param[1].param_val_reg = reg;
	iaxxx->plugin_param[1].param_val = ucontrol->value.integer.value[0];

	ret = iaxxx_core_plg_set_param(iaxxx->dev_parent,
			iaxxx->plugin_param[1].param_id_reg,
			iaxxx->plugin_param[1].param_val_reg,
			iaxxx->plugin_param[1].param_id,
			iaxxx->plugin_param[1].param_val,
			IAXXX_BLOCK_1);

	pr_debug("%s\n", __func__);
	return ret;
}

static int iaxxxcore_blk2_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[2].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk2_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[2].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[2].param_id_reg = reg;

	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk2_get_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	if (iaxxx->plugin_param[2].param_id == IAXXX_MAX_VAL) {
		ucontrol->value.integer.value[0] = 0;
		return 0;
	}
	iaxxx->plugin_param[2].param_val_reg = reg;
	ret = iaxxx_core_plg_get_param(iaxxx->dev_parent,
			iaxxx->plugin_param[2].param_id_reg,
			iaxxx->plugin_param[2].param_val_reg,
			iaxxx->plugin_param[2].param_id,
			&iaxxx->plugin_param[2].param_val,
			IAXXX_BLOCK_2);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[2].param_val;
	pr_debug("%s\n", __func__);
	return ret;
}

static int iaxxxcore_blk2_put_param_val(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;
	int ret;

	iaxxx->plugin_param[2].param_val_reg = reg;
	iaxxx->plugin_param[2].param_val = ucontrol->value.integer.value[0];

	ret = iaxxx_core_plg_set_param(iaxxx->dev_parent,
			iaxxx->plugin_param[2].param_id_reg,
			iaxxx->plugin_param[2].param_val_reg,
			iaxxx->plugin_param[2].param_id,
			iaxxx->plugin_param[2].param_val,
			IAXXX_BLOCK_2);

	pr_debug("%s\n", __func__);
	return ret;
}
#define IAXXXCORE_PLUGIN_ENUM(plugin, plugin_name) \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ctrl_pri_enum, \
			IAXXX_PLUGIN_INS_GRP_CTRL_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS, \
			(IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK >> \
			IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS), \
			priority_texts, priority_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_inst_pkgid_enum, \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS, \
			(IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS), \
			pkg_id_texts, pkg_id_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_origin_idx_enum, \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS, \
			(IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK >> \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS), \
			plugin_origin_idx_texts, plugin_origin_idx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep0_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep1_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep2_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep3_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep4_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep5_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep6_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep7_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_POS), \
			plugin_ip_ep_texts, plugin_ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep0_enc_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_ENCODING_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_ENCODING_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep1_enc_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_ENCODING_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_ENCODING_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep2_enc_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_ENCODING_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_ENCODING_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep3_enc_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_ENCODING_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_ENCODING_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep0_sr_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep1_sr_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep2_sr_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep3_sr_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep0_frm_len_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_LENGTH_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_LENGTH_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep1_frm_len_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_LENGTH_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_LENGTH_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_1_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep2_frm_len_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_LENGTH_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_LENGTH_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_2_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_op_ep3_frm_len_enum, \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_LENGTH_POS, \
			(IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_LENGTH_MASK >> \
			IAXXX_PLUGIN_INS_GRP_OUT_3_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ctrl_option_enum, \
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_REG(plugin), 0, \
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_MASK_VAL, \
			plugin_ctrl_opt_text, plugin_ctrl_opt_value)

#define IAXXXCORE_PLUGIN_KCTRL(plugin, plugin_name) \
	SOC_SINGLE(plugin_name "Create0", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_0_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Create1", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_1_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Create2", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_2_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset0", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_0_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset1", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_1_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset2", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_2_ADDR, plugin, 1, 0), \
	SOC_ENUM(plugin_name "Origin Plugin Idx", plugin##_origin_idx_enum), \
	SOC_ENUM(plugin_name "Pkg ID", plugin##_inst_pkgid_enum), \
	SOC_ENUM(plugin_name "inst priority", plugin##_ctrl_pri_enum), \
	SOC_SINGLE_EXT(plugin_name "Blk0 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk0_get_param_id, \
			iaxxxcore_blk0_put_param_id), \
	SOC_SINGLE_EXT(plugin_name "Blk0 Param Val", \
			IAXXX_PLUGIN_INS_GRP_PARAM_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk0_get_param_val, \
			iaxxxcore_blk0_put_param_val), \
	SOC_SINGLE_EXT(plugin_name "Blk1 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk1_get_param_id, \
			iaxxxcore_blk1_put_param_id), \
	SOC_SINGLE_EXT(plugin_name "Blk1 Param Val", \
			IAXXX_PLUGIN_INS_GRP_PARAM_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk1_get_param_val, \
			iaxxxcore_blk1_put_param_val), \
	SOC_SINGLE_EXT(plugin_name "Blk2 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk2_get_param_id, \
			iaxxxcore_blk2_put_param_id), \
	SOC_SINGLE_EXT(plugin_name "Blk2 Param Val", \
			IAXXX_PLUGIN_INS_GRP_PARAM_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk2_get_param_val,\
			iaxxxcore_blk2_put_param_val), \
	SOC_ENUM(plugin_name "Op Ep3 FrLn", plugin##_op_ep3_frm_len_enum), \
	SOC_ENUM(plugin_name "Op Ep2 FrLn", plugin##_op_ep2_frm_len_enum), \
	SOC_ENUM(plugin_name "Op Ep1 FrLn", plugin##_op_ep1_frm_len_enum), \
	SOC_ENUM(plugin_name "Op Ep0 FrLn", plugin##_op_ep0_frm_len_enum), \
	SOC_ENUM(plugin_name "Op Ep3 Sr", plugin##_op_ep3_sr_enum), \
	SOC_ENUM(plugin_name "Op Ep2 Sr", plugin##_op_ep2_sr_enum), \
	SOC_ENUM(plugin_name "Op Ep1 Sr", plugin##_op_ep1_sr_enum), \
	SOC_ENUM(plugin_name "Op Ep0 Sr", plugin##_op_ep0_sr_enum), \
	SOC_ENUM(plugin_name "Op Ep3 Enc", plugin##_op_ep3_enc_enum), \
	SOC_ENUM(plugin_name "Op Ep2 Enc", plugin##_op_ep2_enc_enum), \
	SOC_ENUM(plugin_name "Op Ep1 Enc", plugin##_op_ep1_enc_enum), \
	SOC_ENUM(plugin_name "Op Ep0 Enc", plugin##_op_ep0_enc_enum), \
	SOC_ENUM(plugin_name "Ip Ep0 Conf", plugin##_ip_ep0_enum), \
	SOC_ENUM(plugin_name "Ip Ep1 Conf", plugin##_ip_ep1_enum), \
	SOC_ENUM(plugin_name "Ip Ep2 Conf", plugin##_ip_ep2_enum), \
	SOC_ENUM(plugin_name "Ip Ep3 Conf", plugin##_ip_ep3_enum),\
	SOC_ENUM(plugin_name "Ip Ep4 Conf", plugin##_ip_ep4_enum), \
	SOC_ENUM(plugin_name "Ip Ep5 Conf", plugin##_ip_ep5_enum), \
	SOC_ENUM(plugin_name "Ip Ep6 Conf", plugin##_ip_ep6_enum), \
	SOC_ENUM(plugin_name "Ip Ep7 Conf", plugin##_ip_ep7_enum), \
	SOC_ENUM(plugin_name "Cr Conf", plugin##_ctrl_option_enum)


IAXXXCORE_PLUGIN_ENUM(PLUGIN0, "Plgin0");
IAXXXCORE_PLUGIN_ENUM(PLUGIN1, "Plgin1");
IAXXXCORE_PLUGIN_ENUM(PLUGIN2, "Plgin2");
IAXXXCORE_PLUGIN_ENUM(PLUGIN3, "Plgin3");
IAXXXCORE_PLUGIN_ENUM(PLUGIN4, "Plgin4");
IAXXXCORE_PLUGIN_ENUM(PLUGIN5, "Plgin5");
IAXXXCORE_PLUGIN_ENUM(PLUGIN6, "Plgin6");
IAXXXCORE_PLUGIN_ENUM(PLUGIN7, "Plgin7");
IAXXXCORE_PLUGIN_ENUM(IBUF0, "iBuf0");
IAXXXCORE_PLUGIN_ENUM(IBUF1, "iBuf1");
IAXXXCORE_PLUGIN_ENUM(IBUF2, "iBuf2");
IAXXXCORE_PLUGIN_ENUM(IBUF3, "iBuf3");
#ifdef IAXXX_MIXER_ENABLED
IAXXXCORE_PLUGIN_ENUM(IMIX0, "iMix0");
IAXXXCORE_PLUGIN_ENUM(IMIX1, "iMix1");
IAXXXCORE_PLUGIN_ENUM(IMIX2, "iMix2");
IAXXXCORE_PLUGIN_ENUM(IMIX3, "iMix3");
#endif


static const char * const IBUF0_off_on_texts[] = {
	"Off",
	"Rx0iBuf0On", "Rx1iBuf0On", "Rx2iBuf0On", "Rx3iBuf0On",
	"Rx4iBuf0On", "Rx5iBuf0On", "Rx6iBuf0On", "Rx7iBuf0On",
	"Rx8iBuf0On", "Rx9iBuf0On", "Rx10iBuf0On", "Rx11iBuf0On",
	"Rx12iBuf0On", "Rx13iBuf0On", "Rx14iBuf0On", "Rx15iBuf0On",
	"iMix0iBuf0On", "iMix1iBuf0On", "iMix2iBuf0On", "iMix3iBuf0On",
	"Plgin0iBuf0On", "Plgin1iBuf0On", "Plgin2iBuf0On", "Plgin3iBuf0On",
	"Plgin4iBuf0On", "Plgin5iBuf0On", "Plgin6iBuf0On", "Plgin7iBuf0On",
};
static const char * const IBUF1_off_on_texts[] = {
	"Off",
	"Rx0iBuf1On", "Rx1iBuf1On", "Rx2iBuf1On", "Rx3iBuf1On",
	"Rx4iBuf1On", "Rx5iBuf1On", "Rx6iBuf1On", "Rx7iBuf1On",
	"Rx8iBuf1On", "Rx9iBuf1On", "Rx10iBuf1On", "Rx11iBuf1On",
	"Rx12iBuf1On", "Rx13iBuf1On", "Rx14iBuf1On", "Rx15iBuf1On",
	"iMix0iBuf1On", "iMix1iBuf1On", "iMix2iBuf1On", "iMix3iBuf1On",
	"Plgin0iBuf1On", "Plgin1iBuf1On", "Plgin2iBuf1On", "Plgin3iBuf1On",
	"Plgin4iBuf1On", "Plgin5iBuf1On", "Plgin6iBuf1On", "Plgin7iBuf1On",
};
static const char * const IBUF2_off_on_texts[] = {
	"Off",
	"Rx0iBuf2On", "Rx1iBuf2On", "Rx2iBuf2On", "Rx3iBuf2On",
	"Rx4iBuf2On", "Rx5iBuf2On", "Rx6iBuf2On", "Rx7iBuf2On",
	"Rx8iBuf2On", "Rx9iBuf2On", "Rx10iBuf2On", "Rx11iBuf2On",
	"Rx12iBuf2On", "Rx13iBuf2On", "Rx14iBuf2On", "Rx15iBuf2On",
	"iMix0iBuf2On", "iMix1iBuf2On", "iMix2iBuf2On", "iMix3iBuf2On",
	"Plgin0iBuf2On", "Plgin1iBuf2On", "Plgin2iBuf2On", "Plgin3iBuf2On",
	"Plgin4iBuf2On", "Plgin5iBuf2On", "Plgin6iBuf2On", "Plgin7iBuf2On",
};
static const char * const IBUF3_off_on_texts[] = {
	"Off",
	"Rx0iBuf3On", "Rx1iBuf3On", "Rx2iBuf3On", "Rx3iBuf3On",
	"Rx4iBuf3On", "Rx5iBuf3On", "Rx6iBuf3On", "Rx7iBuf3On",
	"Rx8iBuf3On", "Rx9iBuf3On", "Rx10iBuf3On", "Rx11iBuf3On",
	"Rx12iBuf3On", "Rx13iBuf3On", "Rx14iBuf3On", "Rx15iBuf3On",
	"iMix0iBuf3On", "iMix1iBuf3On", "iMix2iBuf3On", "iMix3iBuf3On",
	"Plgin0iBuf3On", "Plgin1iBuf3On", "Plgin2iBuf3On", "Plgin3iBuf3On",
	"Plgin4iBuf3On", "Plgin5iBuf3On", "Plgin6iBuf3On", "Plgin7iBuf3On",
};

#define IAXXX_IBUF_DAPM_MUX(ibuf, ibuf_name) \
	SND_SOC_DAPM_MUX(ibuf_name "En", SND_SOC_NOPM, 0, 0, &ibuf##_mux)

#define IAXXX_IBUF_EN_CTLS(ibuf, ibuf_name) \
	SOC_SINGLE(ibuf_name "Blk0En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
							ibuf, 1, 0), \
	SOC_SINGLE(ibuf_name "Blk1En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
							ibuf, 1, 0), \
	SOC_SINGLE(ibuf_name "Blk2En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_2_ADDR, \
							ibuf, 1, 0)

#define IAXXX_IBUF_DAPM_CTLS(ibuf, ibuf_name) \
static const SOC_ENUM_SINGLE_DECL(ibuf##_en_enum, \
				  SND_SOC_NOPM, ibuf, \
				  ibuf##_off_on_texts); \
static const struct snd_kcontrol_new ibuf##_mux =	\
	SOC_DAPM_ENUM(ibuf_name "Enable", ibuf##_en_enum)

static const char * const PLUGIN0_off_on_texts[] = {
	"Off",
	"iBuf0Plgin0On", "iBuf1Plgin0On", "iBuf2Plgin0On", "iBuf3Plgin0On",
	"iMix0Plgin0On", "iMix1Plgin0On", "iMix2Plgin0On", "iMix3Plgin0On",
	"Rx0Plgin0On", "Rx1Plgin0On", "Rx2Plgin0On", "Rx3Plgin0On",
	"Rx4Plgin0On", "Rx5Plgin0On", "Rx6Plgin0On", "Rx7Plgin0On",
	"Rx8Plgin0On", "Rx9Plgin0On", "Rx10Plgin0On", "Rx11Plgin0On",
	"Rx12Plgin0On", "Rx13Plgin0On", "Rx14Plgin0On", "Rx15Plgin0On",
};
static const char * const PLUGIN1_off_on_texts[] = {
	"Off",
	"iBuf0Plgin1On", "iBuf1Plgin1On", "iBuf2Plgin1On", "iBuf3Plgin1On",
	"iMix0Plgin1On", "iMix1Plgin1On", "iMix2Plgin1On", "iMix3Plgin1On",
	"Rx0Plgin1On", "Rx1Plgin1On", "Rx2Plgin1On", "Rx3Plgin1On",
	"Rx4Plgin1On", "Rx5Plgin1On", "Rx6Plgin1On", "Rx7Plgin1On",
	"Rx8Plgin1On", "Rx9Plgin1On", "Rx10Plgin1On", "Rx11Plgin1On",
	"Rx12Plgin1On", "Rx13Plgin1On", "Rx14Plgin1On", "Rx15Plgin1On",
};
static const char * const PLUGIN2_off_on_texts[] = {
	"Off",
	"iBuf0Plgin2On", "iBuf1Plgin2On", "iBuf2Plgin2On", "iBuf3Plgin2On",
	"iMix0Plgin2On", "iMix1Plgin2On", "iMix2Plgin2On", "iMix3Plgin2On",
	"Rx0Plgin2On", "Rx1Plgin2On", "Rx2Plgin2On", "Rx3Plgin2On",
	"Rx4Plgin2On", "Rx5Plgin2On", "Rx6Plgin2On", "Rx7Plgin2On",
	"Rx8Plgin2On", "Rx9Plgin2On", "Rx10Plgin2On", "Rx11Plgin2On",
	"Rx12Plgin2On", "Rx13Plgin2On", "Rx14Plgin2On", "Rx15Plgin2On",
};
static const char * const PLUGIN3_off_on_texts[] = {
	"Off",
	"iBuf0Plgin3On", "iBuf1Plgin3On", "iBuf2Plgin3On", "iBuf3Plgin3On",
	"iMix0Plgin3On", "iMix1Plgin3On", "iMix2Plgin3On", "iMix3Plgin3On",
	"Rx0Plgin3On", "Rx1Plgin3On", "Rx2Plgin3On", "Rx3Plgin3On",
	"Rx4Plgin3On", "Rx5Plgin3On", "Rx6Plgin3On", "Rx7Plgin3On",
	"Rx8Plgin3On", "Rx9Plgin3On", "Rx10Plgin3On", "Rx11Plgin3On",
	"Rx12Plgin3On", "Rx13Plgin3On", "Rx14Plgin3On", "Rx15Plgin3On",
};
static const char * const PLUGIN4_off_on_texts[] = {
	"Off",
	"iBuf0Plgin4On", "iBuf1Plgin4On", "iBuf2Plgin4On", "iBuf3Plgin4On",
	"iMix0Plgin4On", "iMix1Plgin4On", "iMix2Plgin4On", "iMix3Plgin4On",
	"Rx0Plgin4On", "Rx1Plgin4On", "Rx2Plgin4On", "Rx3Plgin4On",
	"Rx4Plgin4On", "Rx5Plgin4On", "Rx6Plgin4On", "Rx7Plgin4On",
	"Rx8Plgin4On", "Rx9Plgin4On", "Rx10Plgin4On", "Rx11Plgin4On",
	"Rx12Plgin4On", "Rx13Plgin4On", "Rx14Plgin4On", "Rx15Plgin4On",
};
static const char * const PLUGIN5_off_on_texts[] = {
	"Off",
	"iBuf0Plgin5On", "iBuf1Plgin5On", "iBuf2Plgin5On", "iBuf3Plgin5On",
	"iMix0Plgin5On", "iMix1Plgin5On", "iMix2Plgin5On", "iMix3Plgin5On",
	"Rx0Plgin5On", "Rx1Plgin5On", "Rx2Plgin5On", "Rx3Plgin5On",
	"Rx4Plgin5On", "Rx5Plgin5On", "Rx6Plgin5On", "Rx7Plgin5On",
	"Rx8Plgin5On", "Rx9Plgin5On", "Rx10Plgin5On", "Rx11Plgin5On",
	"Rx12Plgin5On", "Rx13Plgin5On", "Rx14Plgin5On", "Rx15Plgin5On",
};
static const char * const PLUGIN6_off_on_texts[] = {
	"Off",
	"iBuf0Plgin6On", "iBuf1Plgin6On", "iBuf2Plgin6On", "iBuf3Plgin6On",
	"iMix0Plgin6On", "iMix1Plgin6On", "iMix2Plgin6On", "iMix3Plgin6On",
	"Rx0Plgin6On", "Rx1Plgin6On", "Rx2Plgin6On", "Rx3Plgin6On",
	"Rx4Plgin6On", "Rx5Plgin6On", "Rx6Plgin6On", "Rx7Plgin6On",
	"Rx8Plgin6On", "Rx9Plgin6On", "Rx10Plgin6On", "Rx11Plgin6On",
	"Rx12Plgin6On", "Rx13Plgin6On", "Rx14Plgin6On", "Rx15Plgin6On",
};
static const char * const PLUGIN7_off_on_texts[] = {
	"Off",
	"iBuf0Plgin7On", "iBuf1Plgin7On", "iBuf2Plgin7On", "iBuf3Plgin7On",
	"iMix0Plgin7On", "iMix1Plgin7On", "iMix2Plgin7On", "iMix3Plgin7On",
	"Rx0Plgin7On", "Rx1Plgin7On", "Rx2Plgin7On", "Rx3Plgin7On",
	"Rx4Plgin7On", "Rx5Plgin7On", "Rx6Plgin7On", "Rx7Plgin7On",
	"Rx8Plgin7On", "Rx9Plgin7On", "Rx10Plgin7On", "Rx11Plgin7On",
	"Rx12Plgin7On", "Rx13Plgin7On", "Rx14Plgin7On", "Rx15Plgin7On",
};

#define IAXXX_PLUGIN_DAPM_MUX(plugin, plugin_name) \
	SND_SOC_DAPM_MUX(plugin_name "En", SND_SOC_NOPM, 0, 0, &plugin##_mux)
#define IAXXX_PLUGIN_EN_CTLS(plugin, plugin_name) \
	SOC_SINGLE(plugin_name "Blk0En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
							plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Blk1En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
							plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Blk2En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_2_ADDR, \
							plugin, 1, 0)

#define IAXXX_PLUGIN_DAPM_CTLS(plugin, plugin_name) \
static const SOC_ENUM_SINGLE_DECL(plugin##_en_enum, \
                                 SND_SOC_NOPM, plugin, \
                                 plugin##_off_on_texts); \
static const struct snd_kcontrol_new plugin##_mux =	\
	SOC_DAPM_ENUM(plugin_name "Enable", plugin##_en_enum)

IAXXX_PLUGIN_DAPM_CTLS(PLUGIN0, "Plgin0");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN1, "Plgin1");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN2, "Plgin2");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN3, "Plgin3");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN4, "Plgin4");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN5, "Plgin5");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN6, "Plgin6");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN7, "Plgin7");

IAXXX_IBUF_DAPM_CTLS(IBUF0, "iBuf0");
IAXXX_IBUF_DAPM_CTLS(IBUF1, "iBuf1");
IAXXX_IBUF_DAPM_CTLS(IBUF2, "iBuf2");
IAXXX_IBUF_DAPM_CTLS(IBUF3, "iBuf3");

#ifdef IAXXX_MIXER_ENABLED
#define IAXXX_IMIX_EN_CTLS(imix, imix_name) \
	SOC_SINGLE(imix_name "Blk0En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
							imix, 1, 0), \
	SOC_SINGLE(imix_name "Blk1En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
							imix, 1, 0), \
	SOC_SINGLE(imix_name "Blk2En", IAXXX_PLUGIN_HDR_ENABLE_BLOCK_2_ADDR, \
							imix, 1, 0)

#define IAXXX_IMIX_DAPM_MIXER(plugin_name) \
	SND_SOC_DAPM_MIXER(plugin_name, SND_SOC_NOPM, 0, 0, \
			&imix_mixer_ctls[0], ARRAY_SIZE(imix_mixer_ctls))

#define IAXXXCORE_IMIX_PLGIN_SWITCH(iMix) \
	SOC_DAPM_SINGLE(iMix "Plgin0 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin1 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin2 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin3 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin4 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin5 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin6 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Plgin7 Switch", SND_SOC_NOPM, 0, 1, 0)

#define IAXXXCORE_IMIX_IBUF_SWITCH(iMix) \
	SOC_DAPM_SINGLE(iMix "IBuf0 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "IBuf1 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "IBuf2 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "IBuf3 Switch", SND_SOC_NOPM, 0, 1, 0)

#define IAXXXCORE_IMIX_RX_SWITCH(iMix) \
	SOC_DAPM_SINGLE(iMix "Rx0 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx1 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx2 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx3 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx4 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx5 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx6 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx7 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx8 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx9 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx10 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx11 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx12 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx13 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx14 Switch", SND_SOC_NOPM, 0, 1, 0), \
	SOC_DAPM_SINGLE(iMix "Rx15 Switch", SND_SOC_NOPM, 0, 1, 0)

static const struct snd_kcontrol_new imix_mixer_ctls[] = {
	IAXXXCORE_IMIX_RX_SWITCH("iMix0"),
	IAXXXCORE_IMIX_RX_SWITCH("iMix1"),
	IAXXXCORE_IMIX_RX_SWITCH("iMix2"),
	IAXXXCORE_IMIX_RX_SWITCH("iMix3"),

	IAXXXCORE_IMIX_IBUF_SWITCH("iMix0"),
	IAXXXCORE_IMIX_IBUF_SWITCH("iMix1"),
	IAXXXCORE_IMIX_IBUF_SWITCH("iMix2"),
	IAXXXCORE_IMIX_IBUF_SWITCH("iMix3"),

	IAXXXCORE_IMIX_PLGIN_SWITCH("iMix0"),
	IAXXXCORE_IMIX_PLGIN_SWITCH("iMix1"),
	IAXXXCORE_IMIX_PLGIN_SWITCH("iMix2"),
	IAXXXCORE_IMIX_PLGIN_SWITCH("iMix3"),
};
#endif

static int iaxxx_put_start_pdm(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int try = IAXXX_MAX_RETRY, ret = -EINVAL;
	uint32_t port_mics_en = ucontrol->value.integer.value[0];

	if (port_mics_en > 255)
		return ret;

	/* Read DMIC enable busy reg, check for DMIC is not busy
	 * set DMIC enable only if DMIC enable busy reg is not set
	 * Check again DMIC enable busy reg is cleared by HW.
	 * DMIC enable busy will get cleared once port clock is available.
	 */
	while (snd_soc_read(codec, dmic_busy_addr[port]) && (try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: port: %d DMIC busy enable bit busy before :%d\n",
			__func__, port, __LINE__);
		return ret;
	}

	/* DMIC enable */
	snd_soc_update_bits(codec, dmic_enable_addr[port], 0xFF, port_mics_en);

	try = IAXXX_MAX_RETRY;
	/*  Read DMIC ENABLE busy  */
	while (snd_soc_read(codec, dmic_busy_addr[port]) && (try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: port: %d DMIC busy enable bit busy after :%d\n",
			__func__, port, __LINE__);
		return ret;
	}
	return 0;

}

static int iaxxx_put_start_portc(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_dmic_start =  ucontrol->value.integer.value[0];
	return iaxxx_put_start_pdm(kcontrol, ucontrol, PDM_PORTC);
}

static int iaxxx_get_start_portc(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_dmic_start;
	return 0;
}

static int iaxxx_put_start_portb(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_dmic_start =  ucontrol->value.integer.value[0];
	return iaxxx_put_start_pdm(kcontrol, ucontrol, PDM_PORTB);
}

static int iaxxx_get_start_portb(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_dmic_start;
	return 0;
}

static const char * const iaxxx_pdm_mic_texts[] = {
		"off", "on",
};

static const SOC_ENUM_SINGLE_DECL(iaxxx_pdm_mic_enum,
			SND_SOC_NOPM, 0, iaxxx_pdm_mic_texts);

static const char * const iaxxx_pdm_port_texts[] = {
	"off", "on"
};

static const SOC_ENUM_SINGLE_DECL(iaxxx_pdm_port_enum,
			SND_SOC_NOPM, 0, iaxxx_pdm_port_texts);


static const SOC_ENUM_SINGLE_DECL(iaxxx_pdm_hos_enum,
			SND_SOC_NOPM, 0, iaxxx_pdm_port_texts);

static const unsigned int pdm_start_values[] = {
	0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40,
	0x80, 0x3, 0x5, 0xA, 0xC, 0xF, 0x30, 0x50,
	0xA0, 0xC0, 0xF0, 0x55, 0xFF };

static const char * const pdm_start_texts[] = {
	"none", "mic1", "mic2", "mic3",
	"mic4", "mic5", "mic6", "mic7",
	"mic8", "mic1-2", "mic1-3", "mic2-4",
	"mic3-4", "mic1234", "mic5-6", "mic5-7",
	"mic6-8", "mic7-8", "mic5678", "mic1357", "all"};

static const struct soc_enum iaxxx_pdm_start_enum =
	SOC_VALUE_ENUM_SINGLE(SND_SOC_NOPM, 0, 0,
					ARRAY_SIZE(pdm_start_texts),
					pdm_start_texts, pdm_start_values);

static int iaxxx_pdm_mic_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int mic)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 cic_rx_id = 0;
	u32 pdm_bclk = 0;
	u8 aud_port_clk = 0;
	u32 cic_hb = 0, hb_dec = 0;
	u32 cic_ctrl = 0;
	u32 cic_rx_rt_ctrl = 0, cic_rx_reset = 0, cic_rx_dec = 0;
	int ret = -EINVAL;

	cic_rx_id = mic;
	pdm_bclk = iaxxx->pdm_bclk;
	aud_port_clk = iaxxx->pdm_aclk;

	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_ADTL_CTRL_ADDR,
			IAXXX_CIC_ADTL_RX_MASK(cic_rx_id),
			IAXXX_CIC_ADTL_RX(cic_rx_id));

	cic_rx_reset =	IAXXX_CIC_RX_RESET <<
			cic_rx_clr_pos[cic_rx_id];
	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
			cic_rx_clr_mask[cic_rx_id], cic_rx_reset);

	/* Get CIC decimation value and half band decimation value */
	ret = get_decimator_val(pdm_bclk, aud_port_clk,
				&cic_rx_dec, &hb_dec);
	if (ret)
		return ret;
	cic_rx_dec = cic_rx_dec << cic_rx_m_pos[cic_rx_id];

	/* Green box /Half band decimation filter */
	cic_hb = hb_dec << IAXXX_CNR0_CIC_HB_CIC_RX_POS(cic_rx_id);

	/* TODO, Always enable DMIC0_CLK */

	cic_ctrl = IAXXX_DMIC0_CLK << cic_rx_id;

	cic_ctrl = cic_ctrl | (IAXXX_PDM_POLARITY <<
		IAXXX_CNR0_CIC_CTRL_RX_POL_POS(cic_rx_id));
	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_CTRL_ADDR,
		IAXXX_CNR0_CIC_CTRL_RX_MASK(cic_rx_id), cic_ctrl);

	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_HB_ADDR,
		IAXXX_CNR0_CIC_HB_CIC_RX_MASK(cic_rx_id), cic_hb);

	cic_rx_rt_ctrl = IAXXX_CIC_MIC_ENABLE <<
		IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_POS(cic_rx_id);
	cic_rx_rt_ctrl = cic_rx_rt_ctrl | (IAXXX_CIC_S_DMIC_ENABLE<<
		IAXXX_CNR0_CIC_RX_RT_CTRL_S_POS(cic_rx_id));
	cic_rx_rt_ctrl = cic_rx_rt_ctrl | (IAXXX_CLK_ENABLE <<
		IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_POS(cic_rx_id));

	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
		IAXXX_CNR0_CIC_RX_RT_MASK(cic_rx_id), cic_rx_rt_ctrl);

	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
		cic_rx_m_mask[cic_rx_id], cic_rx_dec);

	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
		cic_rx_clr_mask[cic_rx_id], ~cic_rx_reset);

	return 0;
}

static int iaxxx_portb_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_mic0_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC4);
}

static int iaxxx_portb_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic0_en;
	return 0;
}

static int iaxxx_portb_mic1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_mic1_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC5);
}

static int iaxxx_portb_mic1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic1_en;
	return 0;
}

static int iaxxx_portb_mic2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_mic2_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC6);
}

static int iaxxx_portb_mic2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic2_en;
	return 0;
}

static int iaxxx_portb_mic3_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_mic3_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC7);
}

static int iaxxx_portb_mic3_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic3_en;
	return 0;
}

static int iaxxx_portc_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_mic0_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC0);
}

static int iaxxx_portc_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic0_en;
	return 0;
}

static int iaxxx_portc_mic1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);

	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_mic1_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC1);
}

static int iaxxx_portc_mic1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic1_en;
	return 0;
}

static int iaxxx_portc_mic2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_mic2_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC2);
}

static int iaxxx_portc_mic2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic2_en;
	return 0;
}

static int iaxxx_portc_mic3_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_mic3_en =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_mic_setup(kcontrol, ucontrol, CIC3);
}

static int iaxxx_portc_mic3_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic3_en;
	return 0;
}

static int iaxxx_pdm_portb_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_filter;
	return 0;
}

static int iaxxx_pdm_portc_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_filter;
	return 0;
}

static int iaxxx_pdm_port_clr(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 cic_rx_id = 0;
	u32 cic_rx_reset = 0;
	int try = IAXXX_MAX_RETRY, ret = -EINVAL;
	u32 status = 0;
	u32 cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_LOW;

	if (cic_rx_id > CIC_NONE) {
		pr_err(" Invalid cic_rx_id\n");
		return -EINVAL;
	}
	/* DISABLE PDM PORT */
	cic_rx_reset =  IAXXX_CIC_RX_RESET <<
			cic_rx_clr_pos[cic_rx_id];
	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
		cic_rx_clr_mask[cic_rx_id], cic_rx_reset);
	/* DISABLE DMIC */
	while (snd_soc_read(codec, dmic_busy_addr[PDM_PORTC]) &&
					(try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: DMIC busy enable bit busy :%d\n",
			__func__, __LINE__);
		return ret;
	}
	/* DMIC enable */
	snd_soc_update_bits(codec, dmic_enable_addr[PDM_PORTC],
		(IAXXX_DMIC_ENABLE_MASK << cic_rx_id),
		(IAXXX_DMIC_ENABLE << cic_rx_id));

	try = IAXXX_MAX_RETRY;
	/*  Always enable DMIC0_CLK */
	/*  Read DMIC ENABLE busy  */
	while (snd_soc_read(codec, dmic_busy_addr[PDM_PORTC])
					&& (try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: DMIC busy enable bit busy :%d\n",
			__func__, __LINE__);
		return ret;
	}
	if (port == PDM_PORTB) {
		snd_soc_update_bits(codec, port_clk_addr[port],
			IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
			0);

		snd_soc_update_bits(codec, port_fs_addr[port],
			IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
			0);

		snd_soc_update_bits(codec, port_di_addr[port],
			IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK,
			 0);
	}
	snd_soc_update_bits(codec, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
		IAXXX_SRB_PDMI_PORT_PWR_EN_MASK_VAL, 0x0 << cic_rx_id);

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
					IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}
	/* DISABLE I2S PORT */
	if (PDM_CODEC_MASTER) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
				IAXXX_CNR0_I2S_ENABLE_MASK(port),
				cnr0_i2s_val << port);
		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
				IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
	}
	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x0 << port));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	return 0;
}

static int iaxxx_put_pdm_portb_clr(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	iaxxx_pdm_port_clr(kcontrol, ucontrol, PDM_PORTB);
	return 0;
}

static int iaxxx_get_pdm_portb_clr(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;

}

static int iaxxx_put_pdm_portc_clr(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	iaxxx_pdm_port_clr(kcontrol, ucontrol, PDM_PORTC);
	return 0;

}

static int iaxxx_get_pdm_portc_clr(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;

}

static int iaxxx_pdm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u8 aud_port_clk = 0;
	u32 period = 0, div_val = 0, nr_val = 0, fs_sync_active = 0;
	u32 pdm_bclk = 0, port_sample_rate = 0, port_bits_per_frame = 0;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 clk_ctrl_val = 0;
	u32 ao_clk_cfg_val = 0;
	u32 ao_clk_cfg_mask = 0, status = 0;
	int ret;

	/*
	 * 1. Configure I2S master if chip is master
	 * 2. Enable AO CLK CFG
	 * 3. Configure ports registers
	 * 4. Configure CIC filter register
	 * 5. Enable Dmic clk
	 */
	if (port == PDM_PORTB) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTB_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTB_CLK_OE_MASK;
	} else {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTC_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTC_CLK_OE_MASK;
	}

	if (PDM_CODEC_MASTER) {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_PDM_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_PDM_MASTER;
		port_di_val = port_di_val | IAXXX_IO_CTRL_DI_PDM;
		port_do_val = port_do_val | IAXXX_IO_CTRL_DO;

	} else {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_PDM_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_PDM_SLAVE;
		port_di_val = port_di_val | IAXXX_IO_CTRL_DI_PDM;
		port_do_val = port_do_val | IAXXX_IO_CTRL_DO;

	}

	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
		IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x1 << port));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}
	/* Configure I2S clock */
	pdm_bclk = iaxxx->pdm_bclk;
	port_sample_rate = pdm_cfg[pdm_bclk].sample_rate;
	port_bits_per_frame = pdm_cfg[pdm_bclk].bits_per_frame - 1;
	iaxxx_calc_i2s_div(port_bits_per_frame, port_sample_rate,
				&period, &div_val, &nr_val);
	aud_port_clk = iaxxx->pdm_aclk;
	/* Store value to use while system resume */
	clk.port_bits_per_frame = port_bits_per_frame;
	clk.div_val = div_val;
	clk.nr_val = nr_val;
	clk.period = period;

	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port),
		IAXXX_CNR0_I2S_ENABLE_LOW << port);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	/*Bit 0 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S2_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 1 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_LOW);
	/* Bit 2*/
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 3 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);
	/* Bit 11:4 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
		((port_bits_per_frame) <<
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));
	/* For PDM FS is assumed 0 */
	fs_sync_active = (0 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS);
	/* Bit 19:12 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK, fs_sync_active);
	/* Bit 20 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);
	/* FS Align */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);
	/* disable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_DISABLE);
	/* Set HL value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	/* enable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_ENABLE);
	/* disable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_DISABLE);
	/* Set NR value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_WMASK_VAL, nr_val);
	/* enable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_ENABLE);
	/* Clk control */
	clk_ctrl_val = (clk_ctrl_val | ((period/2 - 1) <<
			IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS)|((period - 1)
			<< IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	snd_soc_update_bits(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL, clk_ctrl_val);

	/* AO CLK Config */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		ao_clk_cfg_mask, ao_clk_cfg_val);

	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port),
		IAXXX_CNR0_I2S_ENABLE_HIGH << port);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	snd_soc_update_bits(codec, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
		IAXXX_SRB_PDMI_PORT_PWR_EN_MASK_VAL, 0xFF);

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	/* Configure IO CTRL ports */
	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);

	/* CIC filter config */
	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, IAXXX_CIC_RX_HOST_DEF);

	return 0;
}


static int iaxxx_pdm_portb_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_filter =  ucontrol->value.integer.value[0];

	return iaxxx_pdm_port_setup(kcontrol, ucontrol, PDM_PORTB);
}

static int iaxxx_pdm_portc_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_filter =  ucontrol->value.integer.value[0];
	return iaxxx_pdm_port_setup(kcontrol, ucontrol, PDM_PORTC);
}

static int iaxxx_pdm_head_strm_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	u32 value;

	iaxxx->head_of_strm_all =  ucontrol->value.integer.value[0];
	if (iaxxx->head_of_strm_all)
		value = IAXXX_CIC_RX_HOS_ALL;
	else
		value = 0;

	/* CIC filter config */
	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, value);

	return 0;
}

static int iaxxx_pdm_head_strm_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->head_of_strm_all;
	return 0;
}

static int iaxxx_set_i2s_controller(struct snd_soc_codec *codec,
			u32 sampling_rate, bool is_pseudo, int id)
{
	return 0;
}

static int iaxxx_pcm_port_start(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int port, bool mstrclk)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	u32 word_len = 15; /* 16bit frames */
	u32 channel_val = 3; /* both channels are active */
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	u32 ao_do_val = 0;
	u32 cnr0_pcm_val = 0;
	u32 cnr0_i2s_val = 0;


	pr_debug("%s\n", __func__);

	/* Parse input values */
	word_len    = (ucontrol->value.integer.value[0] & 0x1F);
	channel_val = (ucontrol->value.integer.value[0] & 0x60) >> 5;

	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(port),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);

	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(port),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_val);

	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(port),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_val);

	ao_do_val = IAXXX_AO_DO_ENABLE;
	if (mstrclk) {
		sampling_rate = 16000;
		iaxxx_set_i2s_controller(codec, sampling_rate, false, port);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_HIGH;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}
	cnr0_pcm_val = IAXXX_CNR0_PCM_ENABLE;
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(port),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(port),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port),
		ao_do_val <<
			IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(port));
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(port),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(port));

	if (mstrclk) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			cnr0_i2s_val << port);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	return 0;
}

static int iaxxx_pcm_portb_start_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_pcm_start =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_start(kcontrol, ucontrol, PCM_PORTB, false);
}

static int iaxxx_pcm_portb_start_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_pcm_start;
	return 0;
}

static int iaxxx_pcm_portc_start_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_pcm_start =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_start(kcontrol, ucontrol, PCM_PORTC, false);
}

static int iaxxx_pcm_portc_start_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_pcm_start;
	return 0;
}

static int iaxxx_pcm_portd_start_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portd_pcm_start =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_start(kcontrol, ucontrol, PCM_PORTD, false);
}

static int iaxxx_pcm_portd_start_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portd_pcm_start;
	return 0;
}

static int iaxxx_pcm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int port, bool mstrclk)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 mode = 0;
	u32 reg_stdd_val = 0;
	u32 reg_srdd_val = 0;
	u32 reg_sflr_val = 0;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 status = 0;
	int ret;

	pr_debug("%s\n", __func__);

	if (mstrclk) {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_MASTER;

		/* TODO Double check for device master mode */
		reg_sflr_val = 1;
	} else {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_SLAVE;

		/* words per frame - 1
		* This value needs to tweaked based on platforms
		* TI codec and wand board use 2 words per frame
		*/
		reg_sflr_val = 1;
	}


	if (PCM_PORT_I2S) {
		reg_srdd_val = 1;
		reg_stdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
	} else {
		dev_err(codec->dev, "unsupported format \n");
		return -EINVAL;
	}

	port_di_val = port_di_val | IAXXX_IO_CTRL_DI;
	port_do_val = port_do_val | IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
		0x1<<port, 0x1<<port);

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	snd_soc_update_bits(codec, IAXXX_PCM_SRDD_ADDR(port),
		IAXXX_PCM0_SRDD_WMASK_VAL, reg_srdd_val);


	snd_soc_update_bits(codec, IAXXX_PCM_STDD_ADDR(port),
		IAXXX_PCM0_STDD_WMASK_VAL, reg_stdd_val);


	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(port),
		IAXXX_PCM0_MC_WMASK_VAL, mode);


	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(port),
		IAXXX_PCM0_SFLR_WMASK_VAL, reg_sflr_val);


	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);


	snd_soc_update_bits(codec, port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);


	snd_soc_update_bits(codec, port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);


	snd_soc_update_bits(codec, port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);


	return 0;
}

static int iaxxx_pcm_portb_setup_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portb_pcm_setup =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_setup(kcontrol, ucontrol, PCM_PORTB, false);
}

static int iaxxx_pcm_portb_setup_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_pcm_setup;
	return 0;
}

static int iaxxx_pcm_portc_setup_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portc_pcm_setup =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_setup(kcontrol, ucontrol, PCM_PORTC, false);
}

static int iaxxx_pcm_portc_setup_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_pcm_setup;
	return 0;
}


static int iaxxx_pcm_portd_setup_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	iaxxx->portd_pcm_setup =  ucontrol->value.integer.value[0];

	return iaxxx_pcm_port_setup(kcontrol, ucontrol, PCM_PORTD, false);
}

static int iaxxx_pcm_portd_setup_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portd_pcm_setup;
	return 0;
}

static const struct snd_kcontrol_new iaxxx_snd_controls[] = {

	IAXXXCORE_RX_CHMGR_KCTRL(RX_0, "Rx0"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_1, "Rx1"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_2, "Rx2"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_3, "Rx3"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_4, "Rx4"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_5, "Rx5"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_6, "Rx6"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_7, "Rx7"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_8, "Rx8"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_9, "Rx9"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_10, "Rx10"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_11, "Rx11"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_12, "Rx12"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_13, "Rx13"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_14, "Rx14"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_15, "Rx15"),

	IAXXXCORE_TX_CHMGR_KCTRL(TX_0, "Tx0"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_1, "Tx1"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_2, "Tx2"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_3, "Tx3"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_4, "Tx4"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_5, "Tx5"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_6, "Tx6"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_7, "Tx7"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_8, "Tx8"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_9, "Tx9"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_10, "Tx10"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_11, "Tx11"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_12, "Tx12"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_13, "Tx13"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_14, "Tx14"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_15, "Tx15"),

	IAXXXCORE_STREAM_KCTRL(STREAM0, "strm0"),
	IAXXXCORE_STREAM_KCTRL(STREAM1, "strm1"),
	IAXXXCORE_STREAM_KCTRL(STREAM2, "strm2"),
	IAXXXCORE_STREAM_KCTRL(STREAM3, "strm3"),
	IAXXXCORE_STREAM_KCTRL(STREAM4, "strm4"),
	IAXXXCORE_STREAM_KCTRL(STREAM5, "strm5"),
	IAXXXCORE_STREAM_KCTRL(STREAM6, "strm6"),
	IAXXXCORE_STREAM_KCTRL(STREAM7, "strm7"),
	IAXXXCORE_STREAM_KCTRL(STREAM8, "strm8"),
	IAXXXCORE_STREAM_KCTRL(STREAM9, "strm9"),
	IAXXXCORE_STREAM_KCTRL(STREAM10, "strm10"),
	IAXXXCORE_STREAM_KCTRL(STREAM11, "strm11"),
	IAXXXCORE_STREAM_KCTRL(STREAM12, "strm12"),
	IAXXXCORE_STREAM_KCTRL(STREAM13, "strm13"),
	IAXXXCORE_STREAM_KCTRL(STREAM14, "strm14"),
	IAXXXCORE_STREAM_KCTRL(STREAM15, "strm15"),

	IAXXXCORE_PLUGIN_KCTRL(PLUGIN0, "Plgin0"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN1, "Plgin1"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN2, "Plgin2"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN3, "Plgin3"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN4, "Plgin4"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN5, "Plgin5"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN6, "Plgin6"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN7, "Plgin7"),

#ifdef IAXXX_MIXER_ENABLED
	IAXXXCORE_PLUGIN_KCTRL(IMIX0, "iMix0"),
	IAXXXCORE_PLUGIN_KCTRL(IMIX1, "iMix1"),
	IAXXXCORE_PLUGIN_KCTRL(IMIX2, "iMix2"),
	IAXXXCORE_PLUGIN_KCTRL(IMIX3, "iMix3"),
#endif
	IAXXXCORE_PLUGIN_KCTRL(IBUF0, "iBuf0"),
	IAXXXCORE_PLUGIN_KCTRL(IBUF1, "iBuf1"),
	IAXXXCORE_PLUGIN_KCTRL(IBUF2, "iBuf2"),
	IAXXXCORE_PLUGIN_KCTRL(IBUF3, "iBuf3"),

	IAXXX_PLUGIN_EN_CTLS(PLUGIN0, "Plgin0"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN1, "Plgin1"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN2, "Plgin2"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN3, "Plgin3"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN4, "Plgin4"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN5, "Plgin5"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN6, "Plgin6"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN7, "Plgin7"),

	IAXXX_IBUF_EN_CTLS(IBUF0, "iBuf0"),
	IAXXX_IBUF_EN_CTLS(IBUF1, "iBuf1"),
	IAXXX_IBUF_EN_CTLS(IBUF2, "iBuf2"),
	IAXXX_IBUF_EN_CTLS(IBUF3, "iBuf3"),

#ifdef IAXXX_MIXER_ENABLED
	IAXXX_IMIX_EN_CTLS(IMIX0, "iMix0"),
	IAXXX_IMIX_EN_CTLS(IMIX1, "iMix1"),
	IAXXX_IMIX_EN_CTLS(IMIX2, "iMix2"),
	IAXXX_IMIX_EN_CTLS(IMIX3, "iMix3"),
#endif
	SOC_ENUM_EXT("Update Block0 Req", iaxxx_update_block_enum,
		       iaxxxcore_get_update_block0,
		       iaxxxcore_put_update_block0),

	SOC_ENUM_EXT("Update Block1 Req", iaxxx_update_block_enum,
		       iaxxxcore_get_update_block1,
		       iaxxxcore_put_update_block1),

	SOC_ENUM_EXT("Update Block2 Req", iaxxx_update_block_enum,
		       iaxxxcore_get_update_block2,
		       iaxxxcore_put_update_block2),

	SOC_ENUM_EXT("PDM BCLK", iaxxx_pdm_bclk_enum,
		       iaxxx_get_pdm_bclk,
		       iaxxx_put_pdm_bclk),

	SOC_ENUM_EXT("PDM Port ACLK", iaxxx_pdm_aclk_enum,
		       iaxxx_get_pdm_aclk,
		       iaxxx_put_pdm_aclk),

	SOC_ENUM_EXT("PDM PortB CLR", iaxxx_pdm_clr_enum,
		       iaxxx_get_pdm_portb_clr,
		       iaxxx_put_pdm_portb_clr),

	SOC_ENUM_EXT("PDM PortC CLR", iaxxx_pdm_clr_enum,
		       iaxxx_get_pdm_portc_clr,
		       iaxxx_put_pdm_portc_clr),

	SOC_ENUM_EXT("Pdm PortB Setup", iaxxx_pdm_port_enum,
			iaxxx_pdm_portb_get, iaxxx_pdm_portb_put),

	SOC_ENUM_EXT("Pdm PortC Setup", iaxxx_pdm_port_enum,
			iaxxx_pdm_portc_get, iaxxx_pdm_portc_put),

	SOC_ENUM_EXT("Pdm PortB DMic0 En", iaxxx_pdm_mic_enum,
			iaxxx_portb_mic0_get, iaxxx_portb_mic0_put),
	SOC_ENUM_EXT("Pdm PortB DMic1 En", iaxxx_pdm_mic_enum,
			iaxxx_portb_mic1_get, iaxxx_portb_mic1_put),
	SOC_ENUM_EXT("Pdm PortB DMic2 En", iaxxx_pdm_mic_enum,
			iaxxx_portb_mic2_get, iaxxx_portb_mic2_put),
	SOC_ENUM_EXT("Pdm PortB DMic3 En", iaxxx_pdm_mic_enum,
			iaxxx_portb_mic3_get, iaxxx_portb_mic3_put),

	SOC_ENUM_EXT("Pdm PortC DMic0 En", iaxxx_pdm_mic_enum,
			iaxxx_portc_mic0_get, iaxxx_portc_mic0_put),
	SOC_ENUM_EXT("Pdm PortC DMic1 En", iaxxx_pdm_mic_enum,
			iaxxx_portc_mic1_get, iaxxx_portc_mic1_put),
	SOC_ENUM_EXT("Pdm PortC DMic2 En", iaxxx_pdm_mic_enum,
			iaxxx_portc_mic2_get, iaxxx_portc_mic2_put),
	SOC_ENUM_EXT("Pdm PortC DMic3 En", iaxxx_pdm_mic_enum,
			iaxxx_portc_mic3_get, iaxxx_portc_mic3_put),
	SOC_ENUM_EXT("PDM PortC Start", iaxxx_pdm_start_enum,
			iaxxx_get_start_portc, iaxxx_put_start_portc),
	SOC_ENUM_EXT("PDM PortB Start", iaxxx_pdm_start_enum,
			iaxxx_get_start_portb, iaxxx_put_start_portb),

	SOC_ENUM_EXT("PCM PortB Setup", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portb_setup_get, iaxxx_pcm_portb_setup_put),
	SOC_ENUM_EXT("PCM PortB Start", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portb_start_get, iaxxx_pcm_portb_start_put),
	SOC_ENUM_EXT("PCM PortC Setup", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portc_setup_get, iaxxx_pcm_portc_setup_put),
	SOC_ENUM_EXT("PCM PortC Start", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portc_start_get, iaxxx_pcm_portc_start_put),
	SOC_ENUM_EXT("PCM PortD Setup", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portd_setup_get, iaxxx_pcm_portd_setup_put),
	SOC_ENUM_EXT("PCM PortD Start", iaxxx_pdm_port_enum,
		  iaxxx_pcm_portd_start_get, iaxxx_pcm_portd_start_put),

	SOC_ENUM_EXT("PDM Hos", iaxxx_pdm_hos_enum,
			iaxxx_pdm_head_strm_get, iaxxx_pdm_head_strm_put),
	SOC_ENUM_EXT("Route Status", iaxxx_route_status_enum,
		       iaxxx_get_route_status,
		       iaxxx_put_route_status),
};

static int iaxxxcore_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_debug("event 0x%x, port id: %x", event, w->id);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	{
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;
	}
	case SND_SOC_DAPM_POST_PMD:
	{
		pr_debug("SND_SOC_DAPM_POST_PMD event");
		break;
	}
	default:
		pr_err("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static int iaxxxcore_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	{
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;
	}
	case SND_SOC_DAPM_POST_PMD:
	{
		/* Disable PCM ports post streaming */
		pr_debug("SND_SOC_DAPM_POST_PMD event");

		break;
	}
	default:
		pr_debug("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static const struct snd_soc_dapm_widget iaxxx_dapm_widgets[] = {
#ifdef CONFIG_IAXXX_PCM_MODE
	SND_SOC_DAPM_AIF_IN_E("PCM0.0 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.1 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.2 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.3 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#else
	SND_SOC_DAPM_AIF_IN_E("PCM0.0 RX", "I2S TDM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.1 RX", "I2S TDM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.2 RX", "I2S TDM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.3 RX", "I2S TDM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#endif
	SND_SOC_DAPM_AIF_IN_E("PCM1.0 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.1 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.2 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.3 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM2.0 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.1 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.2 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.3 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM3.0 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.1 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.2 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.3 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM4.0 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.1 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.2 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.3 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM5.0 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.1 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.2 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.3 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#ifdef CONFIG_IAXXX_PCM_MODE
	SND_SOC_DAPM_AIF_OUT_E("PCM0.0 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.1 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.2 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.3 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#else
	SND_SOC_DAPM_AIF_OUT_E("PCM0.0 TX", "I2S TDM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.1 TX", "I2S TDM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.2 TX", "I2S TDM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.3 TX", "I2S TDM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#endif

	SND_SOC_DAPM_AIF_OUT_E("PCM1.0 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.1 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.2 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.3 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM2.0 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.1 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.2 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.3 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM3.0 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.1 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.2 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.3 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM4.0 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.1 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.2 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.3 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM5.0 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.1 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.2 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.3 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	IAXXX_CH_MGR_DAPM_MUX(RX_0, "Rx0 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_1, "Rx1 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_2, "Rx2 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_3, "Rx3 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_4, "Rx4 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_5, "Rx5 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_6, "Rx6 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_7, "Rx7 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_8, "Rx8 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_9, "Rx9 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_10, "Rx10 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_11, "Rx11 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_12, "Rx12 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_13, "Rx13 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_14, "Rx14 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_15, "Rx15 Mux"),

	IAXXX_CH_MGR_DAPM_MUX(TX_0, "Tx0 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_1, "Tx1 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_2, "Tx2 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_3, "Tx3 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_4, "Tx4 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_5, "Tx5 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_6, "Tx6 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_7, "Tx7 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_8, "Tx8 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_9, "Tx9 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_10, "Tx10 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_11, "Tx11 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_12, "Tx12 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_13, "Tx13 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_14, "Tx14 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_15, "Tx15 Mux"),

	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_0, "Tx0"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_1, "Tx1"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_2, "Tx2"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_3, "Tx3"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_4, "Tx4"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_5, "Tx5"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_6, "Tx6"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_7, "Tx7"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_8, "Tx8"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_9, "Tx9"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_10, "Tx10"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_11, "Tx11"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_12, "Tx12"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_13, "Tx13"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_14, "Tx14"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_15, "Tx15"),

	IAXXX_PLUGIN_DAPM_MUX(PLUGIN0, "Plgin0"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN1, "Plgin1"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN2, "Plgin2"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN3, "Plgin3"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN4, "Plgin4"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN5, "Plgin5"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN6, "Plgin6"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN7, "Plgin7"),

	IAXXX_IBUF_DAPM_MUX(IBUF0, "iBuf0"),
	IAXXX_IBUF_DAPM_MUX(IBUF1, "iBuf1"),
	IAXXX_IBUF_DAPM_MUX(IBUF2, "iBuf2"),
	IAXXX_IBUF_DAPM_MUX(IBUF3, "iBuf3"),

#ifdef IAXXX_MIXER_ENABLED
	IAXXX_IMIX_DAPM_MIXER("iMix0 Mixer"),
	IAXXX_IMIX_DAPM_MIXER("iMix1 Mixer"),
	IAXXX_IMIX_DAPM_MIXER("iMix2 Mixer"),
	IAXXX_IMIX_DAPM_MIXER("iMix3 Mixer"),
#endif

	SND_SOC_DAPM_OUTPUT("PCMOUTPUT1"),
	SND_SOC_DAPM_OUTPUT("PCMOUTPUT2"),

	SND_SOC_DAPM_INPUT("PCMINPUT0"),
	SND_SOC_DAPM_INPUT("PCMINPUT1"),
	SND_SOC_DAPM_INPUT("PCMINPUT2"),
	SND_SOC_DAPM_INPUT("PDMINPUT0"),
	SND_SOC_DAPM_INPUT("PDMINPUT1"),
};

#define IBUF_TO_TX_MUX_ROUTE(name) \
	{"Tx0 PortMux En", name"Tx0On", name "En"}, \
	{"Tx1 PortMux En", name"Tx1On", name "En"}, \
	{"Tx2 PortMux En", name"Tx2On", name "En"}, \
	{"Tx3 PortMux En", name"Tx3On", name "En"}, \
	{"Tx4 PortMux En", name"Tx4On", name "En"}, \
	{"Tx5 PortMux En", name"Tx5On", name "En"}, \
	{"Tx6 PortMux En", name"Tx6On", name "En"}, \
	{"Tx7 PortMux En", name"Tx7On", name "En"}, \
	{"Tx8 PortMux En", name"Tx8On", name "En"}, \
	{"Tx9 PortMux En", name"Tx9On", name "En"}, \
	{"Tx10 PortMux En", name"Tx10On", name "En"}, \
	{"Tx11 PortMux En", name"Tx11On", name "En"}, \
	{"Tx12 PortMux En", name"Tx12On", name "En"}, \
	{"Tx13 PortMux En", name"Tx13On", name "En"}, \
	{"Tx14 PortMux En", name"Tx14On", name "En"}, \
	{"Tx15 PortMux En", name"Tx15On", name "En"}

#ifdef IAXXX_MIXER_ENABLED
#define IBUF_TO_IMIX_ROUTE(name) \
	{name " Mixer", name "IBuf0 Switch", "iBuf0En"}, \
	{name " Mixer", name "IBuf1 Switch", "iBuf1En"}, \
	{name " Mixer", name "IBuf2 Switch", "iBuf2En"}, \
	{name " Mixer", name "IBuf3 Switch", "iBuf3En"}

#define IMIX_TO_IBUF_ROUTE(name) \
	{name "En", "iMix0"name"On", "iMix0 Mixer"}, \
	{name "En", "iMix1"name"On", "iMix1 Mixer"}, \
	{name "En", "iMix2"name"On", "iMix2 Mixer"}, \
	{name "En", "iMix3"name"On", "iMix3 Mixer"}

#define IMIX_TO_PLUGIN_ROUTE(name) \
	{name"En", "iMix0"name"On", "iMix0 Mixer"}, \
	{name"En", "iMix1"name"On", "iMix1 Mixer"}, \
	{name"En", "iMix2"name"On", "iMix2 Mixer"}, \
	{name"En", "iMix3"name"On", "iMix3 Mixer"}

#define IMIX_TO_TX_MUX_ROUTE(name) \
	{name " PortMux En", "iMix0"name"On", "iMix0 Mixer"}, \
	{name " PortMux En", "iMix1"name"On", "iMix1 Mixer"}, \
	{name " PortMux En", "iMix2"name"On", "iMix2 Mixer"}, \
	{name " PortMux En", "iMix3"name"On", "iMix3 Mixer"}

/* TODO fix multiple plugin outputs route*/
#define PLUGIN_TO_IMIX_ROUTE(name) \
	{name " Mixer", name "Plgin0 Switch", "Plgin0En"}, \
	{name " Mixer", name "Plgin1 Switch", "Plgin1En"}, \
	{name " Mixer", name "Plgin2 Switch", "Plgin2En"}, \
	{name " Mixer", name "Plgin3 Switch", "Plgin3En"}, \
	{name " Mixer", name "Plgin4 Switch", "Plgin4En"}, \
	{name " Mixer", name "Plgin5 Switch", "Plgin5En"}, \
	{name " Mixer", name "Plgin6 Switch", "Plgin6En"}, \
	{name " Mixer", name "Plgin7 Switch", "Plgin7En"}

#define RX_MUX_TO_IMIX_ROUTE(name) \
	{name" Mixer", name "Rx0 Switch", "Rx0 Mux En"}, \
	{name" Mixer", name "Rx1 Switch", "Rx1 Mux En"}, \
	{name" Mixer", name "Rx2 Switch", "Rx2 Mux En"}, \
	{name" Mixer", name "Rx3 Switch", "Rx3 Mux En"}, \
	{name" Mixer", name "Rx4 Switch", "Rx4 Mux En"}, \
	{name" Mixer", name "Rx5 Switch", "Rx5 Mux En"}, \
	{name" Mixer", name "Rx6 Switch", "Rx6 Mux En"}, \
	{name" Mixer", name "Rx7 Switch", "Rx7 Mux En"}, \
	{name" Mixer", name "Rx8 Switch", "Rx8 Mux En"}, \
	{name" Mixer", name "Rx9 Switch", "Rx9 Mux En"}, \
	{name" Mixer", name "Rx10 Switch", "Rx10 Mux En"}, \
	{name" Mixer", name "Rx11 Switch", "Rx11 Mux En"}, \
	{name" Mixer", name "Rx12 Switch", "Rx12 Mux En"}, \
	{name" Mixer", name "Rx13 Switch", "Rx13 Mux En"}, \
	{name" Mixer", name "Rx14 Switch", "Rx14 Mux En"}, \
	{name" Mixer", name "Rx15 Switch", "Rx15 Mux En"}

#endif

#define IBUF_TO_PLUGIN_ROUTE(name) \
	{"Plgin0En", name"Plgin0On", name "En"}, \
	{"Plgin1En", name"Plgin1On", name "En"}, \
	{"Plgin2En", name"Plgin2On", name "En"}, \
	{"Plgin3En", name"Plgin3On", name "En"}, \
	{"Plgin4En", name"Plgin4On", name "En"}, \
	{"Plgin5En", name"Plgin5On", name "En"}, \
	{"Plgin6En", name"Plgin6On", name "En"}, \
	{"Plgin7En", name"Plgin7On", name "En"}

#define PLUGIN_TO_IBUF_ROUTE(Plgin) \
	{"iBuf0En", Plgin"iBuf0On", Plgin"En"}, \
	{"iBuf1En", Plgin"iBuf1On", Plgin"En"}, \
	{"iBuf2En", Plgin"iBuf2On", Plgin"En"}, \
	{"iBuf3En", Plgin"iBuf3On", Plgin"En"}

#define PLUGIN_TO_TX_MUX_ROUTE(Plgin) \
	{"Tx0 PortMux En", Plgin"Tx0On", Plgin"En"}, \
	{"Tx1 PortMux En", Plgin"Tx1On", Plgin"En"}, \
	{"Tx2 PortMux En", Plgin"Tx2On", Plgin"En"}, \
	{"Tx3 PortMux En", Plgin"Tx3On", Plgin"En"}, \
	{"Tx4 PortMux En", Plgin"Tx4On", Plgin"En"}, \
	{"Tx5 PortMux En", Plgin"Tx5On", Plgin"En"}, \
	{"Tx6 PortMux En", Plgin"Tx6On", Plgin"En"}, \
	{"Tx7 PortMux En", Plgin"Tx7On", Plgin"En"}, \
	{"Tx8 PortMux En", Plgin"Tx8On", Plgin"En"}, \
	{"Tx9 PortMux En", Plgin"Tx9On", Plgin"En"}, \
	{"Tx10 PortMux En", Plgin"Tx10On", Plgin"En"}, \
	{"Tx11 PortMux En", Plgin"Tx11On", Plgin"En"}, \
	{"Tx12 PortMux En", Plgin"Tx12On", Plgin"En"}, \
	{"Tx13 PortMux En", Plgin"Tx13On", Plgin"En"}, \
	{"Tx14 PortMux En", Plgin"Tx14On", Plgin"En"}, \
	{"Tx15 PortMux En", Plgin"Tx15On", Plgin"En"}

#define RX_MUX_TO_IBUF_ROUTE(name) \
	{name "En", "Rx0"name"On", "Rx0 Mux En"}, \
	{name "En", "Rx1"name"On", "Rx1 Mux En"}, \
	{name "En", "Rx2"name"On", "Rx2 Mux En"}, \
	{name "En", "Rx3"name"On", "Rx3 Mux En"}, \
	{name "En", "Rx4"name"On", "Rx4 Mux En"}, \
	{name "En", "Rx5"name"On", "Rx5 Mux En"}, \
	{name "En", "Rx6"name"On", "Rx6 Mux En"}, \
	{name "En", "Rx7"name"On", "Rx7 Mux En"}, \
	{name "En", "Rx8"name"On", "Rx8 Mux En"}, \
	{name "En", "Rx9"name"On", "Rx9 Mux En"}, \
	{name "En", "Rx10"name"On", "Rx10 Mux En"}, \
	{name "En", "Rx11"name"On", "Rx11 Mux En"}, \
	{name "En", "Rx12"name"On", "Rx12 Mux En"}, \
	{name "En", "Rx13"name"On", "Rx13 Mux En"}, \
	{name "En", "Rx14"name"On", "Rx14 Mux En"}, \
	{name "En", "Rx15"name"On", "Rx15 Mux En"}

#define RX_MUX_TO_TX_MUX_ROUTE(Tx) \
	{Tx" PortMux En", "Rx0"Tx"On", "Rx0 Mux En"}, \
	{Tx" PortMux En", "Rx1"Tx"On", "Rx1 Mux En"}, \
	{Tx" PortMux En", "Rx2"Tx"On", "Rx2 Mux En"}, \
	{Tx" PortMux En", "Rx3"Tx"On", "Rx3 Mux En"}, \
	{Tx" PortMux En", "Rx4"Tx"On", "Rx4 Mux En"}, \
	{Tx" PortMux En", "Rx5"Tx"On", "Rx5 Mux En"}, \
	{Tx" PortMux En", "Rx6"Tx"On", "Rx6 Mux En"}, \
	{Tx" PortMux En", "Rx7"Tx"On", "Rx7 Mux En"}, \
	{Tx" PortMux En", "Rx8"Tx"On", "Rx8 Mux En"}, \
	{Tx" PortMux En", "Rx9"Tx"On", "Rx9 Mux En"}, \
	{Tx" PortMux En", "Rx10"Tx"On", "Rx10 Mux En"}, \
	{Tx" PortMux En", "Rx11"Tx"On", "Rx11 Mux En"}, \
	{Tx" PortMux En", "Rx12"Tx"On", "Rx12 Mux En"}, \
	{Tx" PortMux En", "Rx13"Tx"On", "Rx13 Mux En"}, \
	{Tx" PortMux En", "Rx14"Tx"On", "Rx14 Mux En"}, \
	{Tx" PortMux En", "Rx15"Tx"On", "Rx15 Mux En"}

#define RX_MUX_TO_PLUGIN_ROUTE(name) \
	{"Plgin0En", name"Plgin0On", name" Mux En"}, \
	{"Plgin1En", name"Plgin1On", name" Mux En"}, \
	{"Plgin2En", name"Plgin2On", name" Mux En"}, \
	{"Plgin3En", name"Plgin3On", name" Mux En"}, \
	{"Plgin4En", name"Plgin4On", name" Mux En"}, \
	{"Plgin5En", name"Plgin5On", name" Mux En"}, \
	{"Plgin6En", name"Plgin6On", name" Mux En"}, \
	{"Plgin7En", name"Plgin7On", name" Mux En"}

#define PORT_TO_RX_MUX_ROUTE(name) \
	{name, "PCM0", "PCM0.0 RX"}, \
	{name, "PCM0", "PCM0.1 RX"}, \
	{name, "PCM0", "PCM0.2 RX"}, \
	{name, "PCM0", "PCM0.3 RX"}, \
	{name, "PCM1", "PCM1.0 RX"}, \
	{name, "PCM1", "PCM1.1 RX"}, \
	{name, "PCM1", "PCM1.2 RX"}, \
	{name, "PCM1", "PCM1.3 RX"}, \
	{name, "PCM2", "PCM2.0 RX"}, \
	{name, "PCM2", "PCM2.1 RX"}, \
	{name, "PCM2", "PCM2.2 RX"}, \
	{name, "PCM2", "PCM2.3 RX"}, \
	{name, "PCM3", "PCM3.0 RX"}, \
	{name, "PCM3", "PCM3.1 RX"}, \
	{name, "PCM3", "PCM3.2 RX"}, \
	{name, "PCM3", "PCM3.3 RX"}, \
	{name, "PCM4", "PCM4.0 RX"}, \
	{name, "PCM4", "PCM4.1 RX"}, \
	{name, "PCM4", "PCM4.2 RX"}, \
	{name, "PCM4", "PCM4.3 RX"}, \
	{name, "PCM5", "PCM5.0 RX"}, \
	{name, "PCM5", "PCM5.1 RX"}, \
	{name, "PCM5", "PCM5.2 RX"}, \
	{name, "PCM5", "PCM5.3 RX"}, \
	{name, "PDMI0", "PDMINPUT0"}, \
	{name, "PDMI1", "PDMINPUT1"}

#define TX_MUX_TO_PORT_ROUTE(name) \
	{"PCM0.0 TX", NULL, name}, \
	{"PCM0.1 TX", NULL, name}, \
	{"PCM0.2 TX", NULL, name}, \
	{"PCM0.3 TX", NULL, name}, \
	{"PCM1.0 TX", NULL, name}, \
	{"PCM1.1 TX", NULL, name}, \
	{"PCM1.2 TX", NULL, name}, \
	{"PCM1.3 TX", NULL, name}, \
	{"PCM2.0 TX", NULL, name}, \
	{"PCM2.1 TX", NULL, name}, \
	{"PCM2.2 TX", NULL, name}, \
	{"PCM2.3 TX", NULL, name}, \
	{"PCM3.0 TX", NULL, name}, \
	{"PCM3.1 TX", NULL, name}, \
	{"PCM3.2 TX", NULL, name}, \
	{"PCM3.3 TX", NULL, name}, \
	{"PCM4.0 TX", NULL, name}, \
	{"PCM4.1 TX", NULL, name}, \
	{"PCM4.2 TX", NULL, name}, \
	{"PCM4.3 TX", NULL, name}, \
	{"PCM5.0 TX", NULL, name}, \
	{"PCM5.1 TX", NULL, name}, \
	{"PCM5.2 TX", NULL, name}, \
	{"PCM5.3 TX", NULL, name}

#define PCM_PORT_TO_OUTPUT_ROUTE(name) \
	{name, NULL, "PCM0.0 TX"}, \
	{name, NULL, "PCM0.1 TX"}, \
	{name, NULL, "PCM0.2 TX"}, \
	{name, NULL, "PCM0.3 TX"}, \
	{name, NULL, "PCM1.0 TX"}, \
	{name, NULL, "PCM1.1 TX"}, \
	{name, NULL, "PCM1.2 TX"}, \
	{name, NULL, "PCM1.3 TX"}, \
	{name, NULL, "PCM2.0 TX"}, \
	{name, NULL, "PCM2.1 TX"}, \
	{name, NULL, "PCM2.2 TX"}, \
	{name, NULL, "PCM2.3 TX"}, \
	{name, NULL, "PCM3.0 TX"}, \
	{name, NULL, "PCM3.1 TX"}, \
	{name, NULL, "PCM3.2 TX"}, \
	{name, NULL, "PCM3.3 TX"}, \
	{name, NULL, "PCM4.0 TX"}, \
	{name, NULL, "PCM4.1 TX"}, \
	{name, NULL, "PCM4.2 TX"}, \
	{name, NULL, "PCM4.3 TX"}, \
	{name, NULL, "PCM5.0 TX"}, \
	{name, NULL, "PCM5.1 TX"}, \
	{name, NULL, "PCM5.2 TX"}, \
	{name, NULL, "PCM5.3 TX"}

#define INPUT_TO_PCM_PORT_ROUTE(name) \
	{"PCM0.0 RX", NULL, name}, \
	{"PCM0.1 RX", NULL, name}, \
	{"PCM0.2 RX", NULL, name}, \
	{"PCM0.3 RX", NULL, name}, \
	{"PCM1.0 RX", NULL, name}, \
	{"PCM1.1 RX", NULL, name}, \
	{"PCM1.2 RX", NULL, name}, \
	{"PCM1.3 RX", NULL, name}, \
	{"PCM2.0 RX", NULL, name}, \
	{"PCM2.1 RX", NULL, name}, \
	{"PCM2.2 RX", NULL, name}, \
	{"PCM2.3 RX", NULL, name}, \
	{"PCM3.0 RX", NULL, name}, \
	{"PCM3.1 RX", NULL, name}, \
	{"PCM3.2 RX", NULL, name}, \
	{"PCM3.3 RX", NULL, name}, \
	{"PCM4.0 RX", NULL, name}, \
	{"PCM4.1 RX", NULL, name}, \
	{"PCM4.2 RX", NULL, name}, \
	{"PCM4.3 RX", NULL, name}, \
	{"PCM5.0 RX", NULL, name}, \
	{"PCM5.1 RX", NULL, name}, \
	{"PCM5.2 RX", NULL, name}, \
	{"PCM5.3 RX", NULL, name}

#define TX_MUX_TO_TX_PORT(name) \
	{"Tx0 Mux Port", name, "Tx0 Mux En"}, \
	{"Tx1 Mux Port", name, "Tx1 Mux En"}, \
	{"Tx2 Mux Port", name, "Tx2 Mux En"}, \
	{"Tx3 Mux Port", name, "Tx3 Mux En"}, \
	{"Tx4 Mux Port", name, "Tx4 Mux En"}, \
	{"Tx5 Mux Port", name, "Tx5 Mux En"}, \
	{"Tx6 Mux Port", name, "Tx6 Mux En"}, \
	{"Tx7 Mux Port", name, "Tx7 Mux En"}, \
	{"Tx8 Mux Port", name, "Tx8 Mux En"}, \
	{"Tx9 Mux Port", name, "Tx9 Mux En"}, \
	{"Tx10 Mux Port", name, "Tx10 Mux En"}, \
	{"Tx11 Mux Port", name, "Tx11 Mux En"}, \
	{"Tx12 Mux Port", name, "Tx12 Mux En"}, \
	{"Tx13 Mux Port", name, "Tx13 Mux En"}, \
	{"Tx14 Mux Port", name, "Tx14 Mux En"}, \
	{"Tx15 Mux Port", name, "Tx15 Mux En"}


static const struct snd_soc_dapm_route iaxxx_dapm_routes[] = {

	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT0"),
	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT1"),
	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT2"),

	/* RX Port to RX Manager */
	PORT_TO_RX_MUX_ROUTE("Rx0 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx1 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx2 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx3 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx4 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx5 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx6 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx7 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx8 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx9 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx10 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx11 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx12 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx13 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx14 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx15 Mux Port"),

	{"Rx0 Mux En", "On", "Rx0 Mux Port"},
	{"Rx1 Mux En", "On", "Rx1 Mux Port"},
	{"Rx2 Mux En", "On", "Rx2 Mux Port"},
	{"Rx3 Mux En", "On", "Rx3 Mux Port"},
	{"Rx4 Mux En", "On", "Rx4 Mux Port"},
	{"Rx5 Mux En", "On", "Rx5 Mux Port"},
	{"Rx6 Mux En", "On", "Rx6 Mux Port"},
	{"Rx7 Mux En", "On", "Rx7 Mux Port"},
	{"Rx8 Mux En", "On", "Rx8 Mux Port"},
	{"Rx9 Mux En", "On", "Rx9 Mux Port"},
	{"Rx10 Mux En", "On", "Rx10 Mux Port"},
	{"Rx11 Mux En", "On", "Rx11 Mux Port"},
	{"Rx12 Mux En", "On", "Rx12 Mux Port"},
	{"Rx13 Mux En", "On", "Rx13 Mux Port"},
	{"Rx14 Mux En", "On", "Rx14 Mux Port"},
	{"Rx15 Mux En", "On", "Rx15 Mux Port"},

	RX_MUX_TO_IBUF_ROUTE("iBuf0"),
	RX_MUX_TO_IBUF_ROUTE("iBuf1"),
	RX_MUX_TO_IBUF_ROUTE("iBuf2"),
	RX_MUX_TO_IBUF_ROUTE("iBuf3"),

#ifdef IAXXX_MIXER_ENABLED
	RX_MUX_TO_IMIX_ROUTE("iMix0"),
	RX_MUX_TO_IMIX_ROUTE("iMix1"),
	RX_MUX_TO_IMIX_ROUTE("iMix2"),
	RX_MUX_TO_IMIX_ROUTE("iMix3"),

	IMIX_TO_TX_MUX_ROUTE("Tx0"),
	IMIX_TO_TX_MUX_ROUTE("Tx1"),
	IMIX_TO_TX_MUX_ROUTE("Tx2"),
	IMIX_TO_TX_MUX_ROUTE("Tx3"),
	IMIX_TO_TX_MUX_ROUTE("Tx4"),
	IMIX_TO_TX_MUX_ROUTE("Tx5"),
	IMIX_TO_TX_MUX_ROUTE("Tx6"),
	IMIX_TO_TX_MUX_ROUTE("Tx7"),
	IMIX_TO_TX_MUX_ROUTE("Tx8"),
	IMIX_TO_TX_MUX_ROUTE("Tx9"),
	IMIX_TO_TX_MUX_ROUTE("Tx10"),
	IMIX_TO_TX_MUX_ROUTE("Tx11"),
	IMIX_TO_TX_MUX_ROUTE("Tx12"),
	IMIX_TO_TX_MUX_ROUTE("Tx13"),
	IMIX_TO_TX_MUX_ROUTE("Tx14"),
	IMIX_TO_TX_MUX_ROUTE("Tx15"),

	IMIX_TO_PLUGIN_ROUTE("Plgin0"),
	IMIX_TO_PLUGIN_ROUTE("Plgin1"),
	IMIX_TO_PLUGIN_ROUTE("Plgin2"),
	IMIX_TO_PLUGIN_ROUTE("Plgin3"),
	IMIX_TO_PLUGIN_ROUTE("Plgin4"),
	IMIX_TO_PLUGIN_ROUTE("Plgin5"),
	IMIX_TO_PLUGIN_ROUTE("Plgin6"),
	IMIX_TO_PLUGIN_ROUTE("Plgin7"),

	IMIX_TO_IBUF_ROUTE("iBuf0"),
	IMIX_TO_IBUF_ROUTE("iBuf1"),
	IMIX_TO_IBUF_ROUTE("iBuf2"),
	IMIX_TO_IBUF_ROUTE("iBuf3"),

	IBUF_TO_IMIX_ROUTE("iMix0"),
	IBUF_TO_IMIX_ROUTE("iMix1"),
	IBUF_TO_IMIX_ROUTE("iMix2"),
	IBUF_TO_IMIX_ROUTE("iMix3"),

	PLUGIN_TO_IMIX_ROUTE("iMix0"),
	PLUGIN_TO_IMIX_ROUTE("iMix1"),
	PLUGIN_TO_IMIX_ROUTE("iMix2"),
	PLUGIN_TO_IMIX_ROUTE("iMix3"),
#endif

	IBUF_TO_TX_MUX_ROUTE("iBuf0"),
	IBUF_TO_TX_MUX_ROUTE("iBuf1"),
	IBUF_TO_TX_MUX_ROUTE("iBuf2"),
	IBUF_TO_TX_MUX_ROUTE("iBuf3"),

	IBUF_TO_PLUGIN_ROUTE("iBuf0"),
	IBUF_TO_PLUGIN_ROUTE("iBuf1"),
	IBUF_TO_PLUGIN_ROUTE("iBuf2"),
	IBUF_TO_PLUGIN_ROUTE("iBuf3"),

	RX_MUX_TO_PLUGIN_ROUTE("Rx0"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx1"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx2"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx3"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx4"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx5"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx6"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx7"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx8"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx9"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx10"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx11"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx12"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx13"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx14"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx15"),

	PLUGIN_TO_IBUF_ROUTE("Plgin0"),
	PLUGIN_TO_IBUF_ROUTE("Plgin1"),
	PLUGIN_TO_IBUF_ROUTE("Plgin2"),
	PLUGIN_TO_IBUF_ROUTE("Plgin3"),
	PLUGIN_TO_IBUF_ROUTE("Plgin4"),
	PLUGIN_TO_IBUF_ROUTE("Plgin5"),
	PLUGIN_TO_IBUF_ROUTE("Plgin6"),
	PLUGIN_TO_IBUF_ROUTE("Plgin7"),

	PLUGIN_TO_TX_MUX_ROUTE("Plgin0"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin1"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin2"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin3"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin4"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin5"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin6"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin7"),

	RX_MUX_TO_TX_MUX_ROUTE("Tx0"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx1"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx2"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx3"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx4"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx5"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx6"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx7"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx8"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx9"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx10"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx11"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx12"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx13"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx14"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx15"),

	{"Tx0 Mux En", "On", "Tx0 PortMux En"},
	{"Tx1 Mux En", "On", "Tx1 PortMux En"},
	{"Tx2 Mux En", "On", "Tx2 PortMux En"},
	{"Tx3 Mux En", "On", "Tx3 PortMux En"},
	{"Tx4 Mux En", "On", "Tx4 PortMux En"},
	{"Tx5 Mux En", "On", "Tx5 PortMux En"},
	{"Tx6 Mux En", "On", "Tx6 PortMux En"},
	{"Tx7 Mux En", "On", "Tx7 PortMux En"},
	{"Tx8 Mux En", "On", "Tx8 PortMux En"},
	{"Tx9 Mux En", "On", "Tx9 PortMux En"},
	{"Tx10 Mux En", "On", "Tx10 PortMux En"},
	{"Tx11 Mux En", "On", "Tx11 PortMux En"},
	{"Tx12 Mux En", "On", "Tx12 PortMux En"},
	{"Tx13 Mux En", "On", "Tx13 PortMux En"},
	{"Tx14 Mux En", "On", "Tx14 PortMux En"},
	{"Tx15 Mux En", "On", "Tx15 PortMux En"},

	TX_MUX_TO_TX_PORT("PCM0"),
	TX_MUX_TO_TX_PORT("PCM1"),
	TX_MUX_TO_TX_PORT("PCM2"),
	TX_MUX_TO_TX_PORT("PCM3"),
	TX_MUX_TO_TX_PORT("PCM4"),
	TX_MUX_TO_TX_PORT("PCM5"),

	/* RX Port to RX Manager */
	TX_MUX_TO_PORT_ROUTE("Tx0 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx1 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx2 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx3 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx4 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx5 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx6 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx7 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx8 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx9 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx10 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx11 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx12 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx13 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx14 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx15 Mux Port"),

	PCM_PORT_TO_OUTPUT_ROUTE("PCMOUTPUT1"),
	PCM_PORT_TO_OUTPUT_ROUTE("PCMOUTPUT2"),
};

static int iaxxx_tdm_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 status = 0;
	int ret;

	pr_debug("%s\n", __func__);

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d \n", dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is Master , chip is slave */
		iaxxx->is_codec_master = 0;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_SLAVE;

		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave , chip is Master */
		iaxxx->is_codec_master = 1;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_MASTER;
		break;
	default:
		return -EINVAL;
	}

	port_di_val = port_di_val | IAXXX_IO_CTRL_DI;
	port_do_val = port_do_val | IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
		IAXXX_SRB_PCM_PORT_PWR_EN_MASK_VAL, 0x1);

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK, port_fs_val);


	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_di_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_do_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);


	snd_soc_update_bits(codec, port_do_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);

	return 0;

}


static int iaxxx_pcm_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 mode = 0;
	u32 reg_stdd_val = 0;
	u32 reg_srdd_val = 0;
	u32 reg_sflr_val = 0;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 status = 0;
	int ret;

	pr_debug("%s\n", __func__);

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d \n", dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is Master , chip is slave */
		iaxxx->is_codec_master = 0;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_SLAVE;

		/* words per frame - 1
		* This value needs to tweaked based on platforms
		* TI codec and wand board use 2 words per frame
		*/
		reg_sflr_val = 1;

		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave , chip is Master */
		iaxxx->is_codec_master = 1;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_MASTER;

		/* TODO Double check for device master mode */
		reg_sflr_val = 1;

		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		reg_srdd_val = 1;
		reg_stdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg_srdd_val = 0;
		reg_stdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_DSPFMT;
		break;
	default:
		dev_err(codec->dev, "unsupported format %u\n",
			fmt);
		return -EINVAL;
	}

	port_di_val = port_di_val | IAXXX_IO_CTRL_DI;
	port_do_val = port_do_val | IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
				0x1 << dai->id, 0x1 << dai->id);


	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	snd_soc_update_bits(codec, IAXXX_PCM_SRDD_ADDR(dai->id),
		IAXXX_PCM0_SRDD_WMASK_VAL, reg_srdd_val);


	snd_soc_update_bits(codec, IAXXX_PCM_STDD_ADDR(dai->id),
		IAXXX_PCM0_STDD_WMASK_VAL, reg_stdd_val);


	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(dai->id),
		IAXXX_PCM0_MC_WMASK_VAL, mode);


	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(dai->id),
		IAXXX_PCM0_SFLR_WMASK_VAL, reg_sflr_val);


	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_clk_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK, port_fs_val);


	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_fs_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_di_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, port_di_val);


	snd_soc_update_bits(codec, port_di_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_di_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_do_addr[dai->id],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);


	snd_soc_update_bits(codec, port_do_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, port_do_val);

	snd_soc_update_bits(codec, port_do_addr[dai->id],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);

	return 0;

}

static int iaxxx_calc_i2s_div(u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val)
{
	u32 bit_clk;
	u32 divider;
	u32 r_val;
	u32 n_val;
	u32 div_cnfg_len;
	int i = 0;

	*div_val = 0;
	*nr_val = 0;
	/* get bit_clk freq */
	bit_clk = (sampling_rate * (bits_per_frame + 1));
	/* get size of NR divider values struct */
	div_cnfg_len = ARRAY_SIZE(i2s_div_config);

	for (i = 0; i < div_cnfg_len; i++) {
		if (bit_clk == i2s_div_config[i].bclk) {
			n_val = i2s_div_config[i].N;
			r_val = i2s_div_config[i].R;
			*period = i2s_div_config[i].period;
			divider = i2s_div_config[i].HL;

			*div_val = *div_val |
				((divider << IAXXX_I2S_I2S0_HL_P_POS) &
				IAXXX_I2S_I2S0_HL_P_MASK);
			*nr_val = *nr_val |
				(((n_val << IAXXX_I2S_I2S0_NR_N_POS) &
				IAXXX_I2S_I2S0_NR_N_MASK) |
				((r_val << IAXXX_I2S_I2S0_NR_R_POS) &
				IAXXX_I2S_I2S0_NR_R_MASK));
			return 0;
		}
	}

	return -EINVAL;
}



static int iaxxx_set_i2s_cfg(struct snd_soc_dai *dai, u32 sampling_rate,
						bool is_pseudo)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	u32 bits_per_frame = 0, pcm_words_per_frame = 0;
	u32 is_i2s_mode = 0;
	u32 period = 0, div_val = 0, nr_val = 0, status = 0;
	u32 clk_ctrl_val = 0, pcm_word_len = 0;
	int ret;

	if (is_pseudo) {
		pr_err("Pseudo mode not supported\n");
		return -EINVAL;
	}

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d \n", dai->id);
		return -EINVAL;
	}

	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
		IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x1 << dai->id));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	/* CNR0_I2S_Enable  - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(dai->id),
		IAXXX_CNR0_I2S_ENABLE_LOW);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	/*Bit 0 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 1 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_LOW);

	is_i2s_mode = snd_soc_read(codec, IAXXX_PCM_MC_ADDR(dai->id));
	is_i2s_mode = is_i2s_mode &
			IAXXX_PCM0_MC_FSP_MASK;
	pcm_word_len = snd_soc_read(codec, IAXXX_PCM_SWLR_ADDR(dai->id));
	pcm_word_len = pcm_word_len & IAXXX_PCM0_SWLR_RMASK_VAL;

	/* Bit 2*/
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 19:12 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK,
		((pcm_word_len + 1) <<
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS));
	/* Bit 3 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);

	/* Bit 11:4 */
	pcm_words_per_frame = snd_soc_read(codec, IAXXX_PCM_SFLR_ADDR(dai->id));
	pcm_words_per_frame = pcm_words_per_frame &
				IAXXX_PCM0_SFLR_RMASK_VAL;

	bits_per_frame = (((pcm_word_len + 1) *
				(pcm_words_per_frame + 1)) - 1);


	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
		(bits_per_frame <<
			IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));

	/* Bit 20 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(dai->id),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);

	/* FS Align */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(dai->id),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);

	ret = iaxxx_calc_i2s_div(bits_per_frame, sampling_rate,
			&period, &div_val, &nr_val);
	if (ret) {
		pr_err("I2S div fail %s()\n", __func__);
		return ret;
	}
	/* disable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(dai->id),
		IAXXX_I2S_I2S0_HL_EN_MASK,
		IAXXX_I2S_I2S0_HL_DISABLE);
	/* Set HL value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(dai->id),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	/* enable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(dai->id),
		IAXXX_I2S_I2S0_HL_EN_MASK,
		IAXXX_I2S_I2S0_HL_ENABLE);

	/* disable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(dai->id),
		IAXXX_I2S_I2S0_NR_EN_MASK,
		IAXXX_I2S_I2S0_NR_DISABLE);
	/* Set NR value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(dai->id),
		IAXXX_I2S_I2S0_NR_MASK_VAL,
		nr_val);
	/* enable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(dai->id),
		IAXXX_I2S_I2S0_NR_EN_MASK,
		IAXXX_I2S_I2S0_NR_ENABLE);
	/* Clk control */
	clk_ctrl_val = (clk_ctrl_val | ((period/2 - 1) <<
			IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS) |
		((period - 1)<<IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	snd_soc_update_bits(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(dai->id),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL,
		clk_ctrl_val);

	return 0;
}

static int iaxxx_tdm_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	u32 ao_do_val = 0;
	u32 cnr0_pcm_val = 0;
	u32 cnr0_i2s_val = 0;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	ao_do_val = IAXXX_AO_DO_ENABLE;

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d \n", dai->id);
		return -EINVAL;
	}
	pr_debug("%s\n", __func__);
	ao_do_val = IAXXX_AO_DO_ENABLE;
	ao_do_val = IAXXX_AO_DO_ENABLE;
	if (iaxxx->is_codec_master) {
		sampling_rate = params_rate(params);
		iaxxx_set_i2s_cfg(dai, sampling_rate, false);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_HIGH;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}
	cnr0_pcm_val = IAXXX_CNR0_PCM_ENABLE;
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(dai->id),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(dai->id),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(dai->id),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(dai->id));
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(dai->id),
		cnr0_pcm_val << IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(dai->id));

	if (iaxxx->is_codec_master) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(dai->id),
			cnr0_i2s_val << dai->id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	/* TODO add logic and remove absolute values */
	snd_soc_update_bits(codec, 0x50042024, 0x0000000F, 0x0000000F);
	snd_soc_update_bits(codec, 0x50042028, 0x00000003, 0x00000003);
	snd_soc_update_bits(codec, 0x5004202c, 0x00000000, 0x00000000);
	snd_soc_update_bits(codec, 0x50042030, 0x00000000, 0x00000000);
	snd_soc_update_bits(codec, 0x50042034, 0x0000000F, 0x0000000F);
	snd_soc_update_bits(codec, 0x50042038, 0x00000000, 0x00000000);
	snd_soc_update_bits(codec, 0x50042040, 0x0000403A, 0x0000403A);
	snd_soc_update_bits(codec, 0x50042054, 0x00050007, 0x00050007);
	return 0;

}

static int iaxxx_pcm_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{

	struct snd_soc_codec *codec = dai->codec;
	u32 word_len = 0;
	u32 channel_val = 0;
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	u32 ao_do_val = 0;
	u32 cnr0_pcm_val = 0;
	u32 cnr0_i2s_val = 0;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d \n", dai->id);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	switch (params_width(params)) {
	/* word length = width-1 */
	case 16:
		word_len = 15;
		break;
	case 20:
		word_len = 19;
		break;
	case 24:
		word_len = 23;
		break;
	case 32:
		word_len = 31;
		break;
	default:
		pr_debug("Unsupported word length\n");
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case 1:
		/*set bit 0*/
		channel_val = 1;
		break;
	case 2:
		/* set bit 0 and bit 1*/
		channel_val = 3;
		break;
	default:
		pr_debug("Unsupported channels :%d\n",
				params_channels(params));
		return -EINVAL;
	}


	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(dai->id),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);

	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(dai->id),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_val);

	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(dai->id),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_val);

	ao_do_val = IAXXX_AO_DO_ENABLE;
	if (iaxxx->is_codec_master) {
		sampling_rate = params_rate(params);
		iaxxx_set_i2s_cfg(dai, sampling_rate, false);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_HIGH;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}
	cnr0_pcm_val = IAXXX_CNR0_PCM_ENABLE;
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(dai->id),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(dai->id),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(dai->id),
		ao_do_val <<
			IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(dai->id));
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(dai->id),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(dai->id));

	if (iaxxx->is_codec_master) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(dai->id),
			cnr0_i2s_val << dai->id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		iaxxx->is_stream_in_use[0] = true;
	else
		iaxxx->is_stream_in_use[1] = true;

	return 0;
}

static int iaxxx_pcm_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	unsigned int fmt = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
	int ret = 0;

	ret = iaxxx_pcm_set_fmt(dai, fmt);
	if (IS_ERR_VALUE(ret))
		pr_err("%s() PCM dai set fmt fail %d\n", __func__, ret);
	return ret;
}

static int iaxxx_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	int gain;
	u32 status = 0;
	u32 pcm_op_gn_mask = 0x30000;
	u32 pcm_op_gn_en_val = 0x3;
	int ret;

	pr_debug("%s: mute: %d\n", __func__, mute);

	/* set gain to -60db when mute is called
	 * set gain to 0db when unmute is called
	 */
	if (mute)
		gain = -60; /*0xBC*/
	else
		gain = 0;

	/* Update TX CHANNEL GAIN EN HDR REG */
	snd_soc_update_bits(codec, IAXXX_CH_HDR_CH_GAIN_ADDR, pcm_op_gn_mask,
				pcm_op_gn_en_val);

	/* Update the Gain ramp rate for TX Channel REG */
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_0),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_1),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);

	/* Update the gain based on mute unmute */
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_0),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_1),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);

	/* Update Block to set gain settings */
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	return 0;
}

static int iaxxx_tdm_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	int gain;
	u32 status = 0;
	u32 tdm_op_gn_en_val = 0xF;
	u32 tdm_op_gn_mask = 0xF0000;
	int ret;

	pr_debug("%s: mute: %d\n", __func__, mute);

	/* set gain to -60db when mute is called
	 * set gain to 0db when unmute is called
	 */
	if (mute)
		gain = -60; /*0xBC*/
	else
		gain = 0;

	/* Update TX Channel GAIN EN HDR REG for TDM*/
	snd_soc_update_bits(codec, IAXXX_CH_HDR_CH_GAIN_ADDR, tdm_op_gn_mask,
			tdm_op_gn_en_val);

	/* Update the GAIN Ramp rate for TX Channel REG */
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_0),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_1),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_2),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_3),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK, STEP_900);

	/* Update the Gain Based on mute and unmute */
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_0),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_1),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_2),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);
	snd_soc_update_bits(codec, IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(TX_3),
			IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK, gain);

	/* Update Block to set gain settings */
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	return 0;
}

static int iaxxx_tdm_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	u32 ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
	u32 ao_fs_val = IAXXX_AO_FS_DISABLE;
	u32 ao_do_val = IAXXX_AO_DO_DISABLE;
	u32 cnr0_pcm_val = IAXXX_CNR0_PCM_DISABLE;
	u32 cnr0_i2s_val = 0;

	pr_debug("%s\n", __func__);
	if (iaxxx->is_codec_master) {
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_LOW;
	}

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(dai->id),
		ao_bclk_val <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(dai->id),
		ao_fs_val <<
			IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(dai->id),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(dai->id));

	if (iaxxx->is_codec_master) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(dai->id),
			cnr0_i2s_val << dai->id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(dai->id),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(dai->id));

	return 0;

}

static int iaxxx_pcm_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	u32 ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
	u32 ao_fs_val = IAXXX_AO_FS_DISABLE;
	u32 ao_do_val = IAXXX_AO_DO_DISABLE;
	u32 cnr0_pcm_val = IAXXX_CNR0_PCM_DISABLE;
	u32 cnr0_i2s_val = 0;

	pr_debug("%s\n", __func__);
	if (iaxxx->is_codec_master) {
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_LOW;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		iaxxx->is_stream_in_use[0] = false;
	else
		iaxxx->is_stream_in_use[1] = false;

	if (iaxxx->is_stream_in_use[0] || iaxxx->is_stream_in_use[1]) {
		pr_debug("%s(): one of the stream is still active:%d\n",
				__func__, substream->stream);
		return 0;
	}


	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(dai->id),
		ao_bclk_val <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(dai->id),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(dai->id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(dai->id),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(dai->id));

	if (iaxxx->is_codec_master) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(dai->id),
			cnr0_i2s_val << dai->id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(dai->id),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(dai->id));

	return 0;

}

#define IAXXX_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

#define IAXXX_PCM_RATES SNDRV_PCM_RATE_8000_48000

static const struct snd_soc_dai_ops iaxxx_pcm_ops = {
	.set_fmt = iaxxx_pcm_set_fmt,
	.hw_params = iaxxx_pcm_hw_params,
	.startup = iaxxx_pcm_startup,
	.hw_free = iaxxx_pcm_hw_free,
	.digital_mute = iaxxx_digital_mute,
};

static const struct snd_soc_dai_ops iaxxx_tdm_ops = {
	.set_fmt = iaxxx_tdm_set_fmt,
	.hw_params = iaxxx_tdm_hw_params,
	.hw_free = iaxxx_tdm_hw_free,
	.digital_mute = iaxxx_tdm_digital_mute,
};

static struct snd_soc_dai_driver iaxxx_dai[] = {
#ifdef CONFIG_IAXXX_PCM_MODE
	{
		.name = "iaxxx-pcm0",
		.id = IAXXX_AIF0,
		.playback = {
			.stream_name = "I2S PCM0 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM0 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
#else
	{
		.name = "iaxxx-tdm0",
		.id = IAXXX_AIF0,
		.playback = {
			.stream_name = "I2S TDM0 Rx",
			.channels_min = 1,
			.channels_max = 4,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S TDM0 Tx",
			.channels_min = 1,
			.channels_max = 4,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_tdm_ops,
	},

#endif
	{
		.name = "iaxxx-pcm1",
		.id = IAXXX_AIF1,
		.playback = {
			.stream_name = "I2S PCM1 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM1 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm2",
		.id = IAXXX_AIF2,
		.playback = {
			.stream_name = "I2S PCM2 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM2 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm3",
		.id = IAXXX_AIF3,
		.playback = {
			.stream_name = "I2S PCM3 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM3 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm4",
		.id = IAXXX_AIF4,
		.playback = {
			.stream_name = "I2S PCM4 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM4 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm5",
		.id = IAXXX_AIF5,
		.playback = {
			.stream_name = "I2S PCM5 Rx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM5 Tx",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
};

struct pkt_loss_reg {
	const char *name;	/* Name of register */
	uint32_t vreg;		/* virtual register */
	uint32_t val;
	uint32_t val_old;
};

static struct pkt_loss_reg packet_loss_regs[] = {
	{ "ch1",   IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(RX_0),     0, 0 },
	{ "ch2",   IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(RX_2),     0, 0 },
	{ "ch3",   IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(RX_4),     0, 0 },
	{ "ch4",   IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(RX_6),     0, 0 },
	{ "tnl01", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(0),  0, 0 },
	{ "tnl02", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(1),  0, 0 },
	{ "tnl03", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(2),  0, 0 },
	{ "tnl04", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(3),  0, 0 },
	{ "tnl05", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(4),  0, 0 },
	{ "tnl06", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(5),  0, 0 },
	{ "tnl07", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(6),  0, 0 },
	{ "tnl08", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(7),  0, 0 },
	{ "tnl09", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(8),  0, 0 },
	{ "tnl10", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(9),  0, 0 },
	{ "tnl11", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(10), 0, 0 },
	{ "tnl12", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(11), 0, 0 },
	{ "tnl13", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(12), 0, 0 },
	{ "tnl14", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(13), 0, 0 },
	{ "tnl15", IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(14), 0, 0 },
};

static ssize_t iaxxx_codec_update_pkg_loss(struct snd_soc_codec *codec)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	struct iaxxx_priv *priv = dev_get_drvdata(codec->dev->parent);
	struct device *dev = codec->dev;
	char *buf = iaxxx->pkg_loss_buf;
	size_t sz, count = sizeof(iaxxx->pkg_loss_buf) - 1;
	int i, rc;

	memset(iaxxx->pkg_loss_buf, 0x0, sizeof(iaxxx->pkg_loss_buf));

	sz = scnprintf(buf, count, "Channel\tpaket_loss\n");
	count -= sz;
	buf += sz;

	for (i = 0; i < ARRAY_SIZE(packet_loss_regs); i++) {
		struct pkt_loss_reg *reg = &packet_loss_regs[i];
		uint32_t diff;

		reg->val_old = reg->val;
		rc = regmap_read(priv->regmap, reg->vreg, &reg->val);
		if (rc)
			break;
		diff = reg->val - reg->val_old;

		dev_dbg(dev, "vreg 0x%08X new 0x%08X old 0x%08X diff %d %s\n",
			reg->vreg, reg->val, reg->val_old, diff, reg->name);

		sz = scnprintf(buf, count, "%s\t%d\n", reg->name, diff);
		count -= sz;
		buf += sz;
	}

	return rc;
}


static ssize_t iaxxx_codec_packet_loss_read(struct snd_info_entry *entry,
				void *file_private_data, struct file *file,
				char __user *buf, size_t count, loff_t pos)
{
	struct snd_soc_codec *codec = entry->private_data;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	char *loss = iaxxx->pkg_loss_buf;

	if (!pos && iaxxx_codec_update_pkg_loss(codec))
		return -EIO;

	return simple_read_from_buffer(buf, count, &pos, loss, strlen(loss));
}

static struct snd_info_entry_ops iaxxx_codec_info_ops = {
	.read = iaxxx_codec_packet_loss_read,
};

/*
 * iaxxx_codec_info_create_codec_entry - creates iaxxx module
 * @iaxxx: Private context of iaxxx codec
 * @codec: Codec instance
 *
 * Creates iaxxx packet_loss entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
static int iaxxx_codec_info_create_codec_entry(struct snd_soc_codec *codec)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_card *card = codec->component.card;
	struct snd_info_entry *root = card->snd_card->proc_root;

	iaxxx->root_entry = snd_register_module_info(
		root->module, "iaxxx", root);
	if (!iaxxx->root_entry) {
		dev_dbg(codec->dev, "failed to create iaxxx root entry");
		goto err_no_reg_root;
	}

	iaxxx->root_entry->private_data = iaxxx;

	iaxxx->pkg_loss = snd_info_create_card_entry(
		card->snd_card, "packet_loss", iaxxx->root_entry);
	if (!iaxxx->pkg_loss) {
		dev_dbg(codec->dev, "failed to create packer_loss entry\n");
		goto err_no_pkg_loss;
	}

	iaxxx->pkg_loss->size = SZ_1K;
	iaxxx->pkg_loss->content = SNDRV_INFO_CONTENT_DATA;
	iaxxx->pkg_loss->c.ops = &iaxxx_codec_info_ops;
	iaxxx->pkg_loss->private_data = codec;

	if (snd_info_register(iaxxx->pkg_loss) < 0) {
		dev_dbg(codec->dev, "failed to register packer_loss entry\n");
		goto err_no_reg_pkg_loss;
	}

	return 0;

err_no_reg_pkg_loss:
	snd_info_free_entry(iaxxx->pkg_loss);
err_no_pkg_loss:
	snd_info_free_entry(iaxxx->root_entry);
err_no_reg_root:

	return -EFAULT;
}

static int iaxxx_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_add_codec_controls(codec, iaxxx_snd_controls,
		ARRAY_SIZE(iaxxx_snd_controls));

	snd_soc_dapm_new_controls(dapm, iaxxx_dapm_widgets,
		ARRAY_SIZE(iaxxx_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, iaxxx_dapm_routes,
		ARRAY_SIZE(iaxxx_dapm_routes));

	return 0;
}

static int iaxxx_codec_probe(struct snd_soc_codec *codec)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
	int ret;

	codec->control_data = iaxxx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret) {
		dev_err(codec->dev, "unable to set the cache io");
		return ret;
	}
#endif

	dev_info(codec->dev, "%s\n", __func__);

	iaxxx->codec = codec;

	pm_runtime_get_sync(codec->dev);

	iaxxx_add_widgets(codec);

	iaxxx_codec_info_create_codec_entry(codec);

	pm_runtime_put(codec->dev);

	return 0;
}

static int iaxxx_codec_remove(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static struct regmap *iaxxx_get_regmap(struct device *dev)
{
	return dev_get_regmap(dev->parent, NULL);
}

static struct snd_soc_codec_driver soc_codec_iaxxx = {
	.probe = iaxxx_codec_probe,
	.remove = iaxxx_codec_remove,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0))
	.get_regmap = iaxxx_get_regmap,
#endif
/*	.suspend = iaxxx_codec_suspend,	*/
/*	.resume = iaxxx_codec_resume,	*/
/*	.set_bias_level = iaxxx_codec_set_bias_level,	*/
};

extern void fb_aloha_trigger_deferred_drivers(void);

static int iaxxx_codec_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct iaxxx_codec_priv *iaxxx =
		container_of(nb, struct iaxxx_codec_priv, nb_core);
	int ret;

	switch (action) {
	case IAXXX_EV_STARTUP:
		ret = snd_soc_register_codec(iaxxx->dev, &soc_codec_iaxxx,
					iaxxx_dai, ARRAY_SIZE(iaxxx_dai));
		if (ret)
			dev_err(iaxxx->dev, "codec registration failed\n");
		else
			fb_aloha_trigger_deferred_drivers();
		break;
	}

	return 0;
}

static int iaxxx_probe(struct platform_device *pdev)
{
	struct iaxxx_codec_priv *iaxxx;
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv;

	dev_dbg(dev, "%s()\n", __func__);

	/* MFD core will provide the regmap instance */
	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("No device data found\n");
		return -EINVAL;
	}

	iaxxx = devm_kzalloc(&pdev->dev, sizeof(*iaxxx), GFP_KERNEL);
	if (iaxxx == NULL)
		return -ENOMEM;

	iaxxx->regmap = priv->regmap;
	iaxxx->dev = dev;
	iaxxx->dev_parent = dev->parent;
	iaxxx->plugin_param[0].param_id = IAXXX_MAX_VAL;
	iaxxx->plugin_param[1].param_id = IAXXX_MAX_VAL;
	iaxxx->plugin_param[2].param_id = IAXXX_MAX_VAL;
	platform_set_drvdata(pdev, iaxxx);

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "iaxxx-codec");

	iaxxx->nb_core.notifier_call = iaxxx_codec_notify;
	iaxxx_fw_notifier_register(priv->dev, &iaxxx->nb_core);

	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int iaxxx_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static int iaxxx_codec_rt_suspend(struct device *dev)
{
	/* struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(dev); */

	if (!pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		/*
		 * This indicate point where lower layer has problem and
		 * all SPI transactions and works with lower layer must
		 * be blocked because is forbidden.
		 */
		dev_info(dev, "Forced suspend requested by HW!");
	}

	return 0;
}

static int iaxxx_codec_rt_resume(struct device *dev)
{
	/* struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(dev); */

	if (!pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		/*
		 * This indicate point where lower layer is already
		 * recovered and ready to continue work.
		 * Could be resumed all pending SPI transactions.
		 */
		dev_info(dev, "Forced resume requested by HW!");
	}

	return 0;
}

static const struct dev_pm_ops iaxxx_codec_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_codec_rt_suspend, iaxxx_codec_rt_resume, NULL)
};

static const struct of_device_id iaxxx_platform_dt_match[] = {
	{.compatible = "adnc,iaxxx-codec"},
	{}
};

static struct platform_driver iaxxx_codec_driver = {
	.probe  = iaxxx_probe,
	.remove = iaxxx_remove,
	.driver = {
		.name = "iaxxx-codec",
		.of_match_table = iaxxx_platform_dt_match,
		.pm = &iaxxx_codec_pm_ops,
	},
};

module_platform_driver(iaxxx_codec_driver);

/* Module information */
MODULE_DESCRIPTION("Knowles IAXXX ALSA SoC CODEC Driver");
MODULE_LICENSE("GPL v2");
