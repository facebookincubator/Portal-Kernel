
/*
 * es-a350.c  --  Audience eS755 ALSA SoC Audio driver
 *
 * Copyright 2013 Audience, Inc.
 *
 * Author: Rajat Aggarwal <raggarwal@audience.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "escore.h"
#include "es-a350-reg.h"

static DECLARE_TLV_DB_SCALE(spkr_tlv, -3600, 200, 1);
static DECLARE_TLV_DB_SCALE(ep_tlv, -2200, 200, 1);
static DECLARE_TLV_DB_SCALE(aux_tlv, 0, 150, 0);
static DECLARE_TLV_DB_SCALE(mic_tlv, 0, 150, 0);

static const unsigned int hp_tlv[] = {
	TLV_DB_RANGE_HEAD(6),
	6, 6, TLV_DB_SCALE_ITEM(-2450, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(-2850, 0, 0),
	8, 10, TLV_DB_SCALE_ITEM(-1400, 300, 0),
	11, 15, TLV_DB_SCALE_ITEM(-600, 200, 0),
};
static const char * const micx_bias_output_voltage_text[] = {
	"1.6V", "1.8V", "2.0V", "2.2V", "2.4V", "2.6V", "2.8V", "3.0V",
};

static const unsigned int micx_bias_output_voltage_value[] = {
	0, 1, 2, 3, 4, 5, 6, 7,
};

static const char * const lo_gain_text[] = {
	"-20", "-14db", "-8db", "-6db", "-4.5db", "-2db", "0", "3",
};

static const unsigned int lo_gain_value[] = {
	0, 1, 2, 3, 4, 5, 6, 7,
};

static const struct soc_enum lol_gain_enum =
	SOC_VALUE_ENUM_SINGLE(ES_LO_L_GAIN, ES_LO_L_GAIN_SHIFT,
		ES_LO_L_GAIN_MASK, ARRAY_SIZE(lo_gain_text),
			lo_gain_text,
			lo_gain_value);

static const struct soc_enum mic0_bias_output_voltage_enum =
	SOC_VALUE_ENUM_SINGLE(ES_MB0_TRIM, ES_MB0_TRIM_SHIFT,
		ES_MB0_TRIM_MASK, ARRAY_SIZE(micx_bias_output_voltage_text),
			micx_bias_output_voltage_text,
			micx_bias_output_voltage_value);


static const struct soc_enum mic1_bias_output_voltage_enum =
	SOC_VALUE_ENUM_SINGLE(ES_MB1_TRIM, ES_MB1_TRIM_SHIFT,
		ES_MB1_TRIM_MASK, ARRAY_SIZE(micx_bias_output_voltage_text),
			micx_bias_output_voltage_text,
			micx_bias_output_voltage_value);

static const struct soc_enum mic2_bias_output_voltage_enum =
	SOC_VALUE_ENUM_SINGLE(ES_MB2_TRIM, ES_MB2_TRIM_SHIFT,
		ES_MB2_TRIM_MASK, ARRAY_SIZE(micx_bias_output_voltage_text),
			micx_bias_output_voltage_text,
			micx_bias_output_voltage_value);

static const struct soc_enum mic3_bias_output_voltage_enum =
	SOC_VALUE_ENUM_SINGLE(ES_MB3_TRIM, ES_MB3_TRIM_SHIFT,
		ES_MB3_TRIM_MASK, ARRAY_SIZE(micx_bias_output_voltage_text),
			micx_bias_output_voltage_text,
			micx_bias_output_voltage_value);

static const struct soc_enum michs_bias_output_voltage_enum =
	SOC_VALUE_ENUM_SINGLE(ES_MBHS_TRIM, ES_MBHS_TRIM_SHIFT,
		ES_MBHS_TRIM_MASK, ARRAY_SIZE(micx_bias_output_voltage_text),
			micx_bias_output_voltage_text,
			micx_bias_output_voltage_value);

static const char * const micx_zin_mode_text[] = {
	"100kohm", "50kohm", "25kohm", "Attenuate by 3dB",
};

static const struct soc_enum mic0_zin_mode_enum =
	SOC_ENUM_SINGLE(ES_MIC0_ZIN_MODE, ES_MIC0_ZIN_MODE_SHIFT,
		ARRAY_SIZE(micx_zin_mode_text), micx_zin_mode_text);

static const struct soc_enum mic1_zin_mode_enum =
	SOC_ENUM_SINGLE(ES_MIC1_ZIN_MODE, ES_MIC1_ZIN_MODE_SHIFT,
		ARRAY_SIZE(micx_zin_mode_text), micx_zin_mode_text);

static const struct soc_enum mic2_zin_mode_enum =
	SOC_ENUM_SINGLE(ES_MIC2_ZIN_MODE, ES_MIC2_ZIN_MODE_SHIFT,
		ARRAY_SIZE(micx_zin_mode_text), micx_zin_mode_text);

static const struct soc_enum mic3_zin_mode_enum =
	SOC_ENUM_SINGLE(ES_MIC3_ZIN_MODE, ES_MIC3_ZIN_MODE_SHIFT,
		ARRAY_SIZE(micx_zin_mode_text), micx_zin_mode_text);

static const struct soc_enum michs_zin_mode_enum =
	SOC_ENUM_SINGLE(ES_MICHS_ZIN_MODE, ES_MICHS_ZIN_MODE_SHIFT,
		ARRAY_SIZE(micx_zin_mode_text), micx_zin_mode_text);

const struct snd_kcontrol_new es_codec_snd_controls[] = {

	SOC_DOUBLE_R_TLV("SPKR Gain", ES_SPKRL_GAIN, ES_SPKR_R_GAIN,
			0, ES_SPKR_L_GAIN_MAX, 0, spkr_tlv),
	SOC_DOUBLE_R_TLV("AUXIN Gain", ES_AUXL_EN, ES_AUXR_EN,
			1, ES_AUXL_GAIN_MAX, 0, aux_tlv),


	SOC_SINGLE_TLV("SPKRL Gain", ES_SPKRL_GAIN, ES_SPKRL_GAIN_SHIFT,
			ES_SPKR_L_GAIN_MAX, 0, spkr_tlv),
	SOC_SINGLE_TLV("SPKR_R Gain", ES_SPKR_R_GAIN, ES_SPKR_R_GAIN_SHIFT,
			ES_SPKR_R_GAIN_MAX, 0, spkr_tlv),
	SOC_VALUE_ENUM("LO Gain", lol_gain_enum),
	SOC_SINGLE_TLV("HP Gain", ES_HP_L_GAIN, ES_HP_L_GAIN_SHIFT,
			ES_HPL_GAIN_MAX, 0, hp_tlv),
	SOC_SINGLE_TLV("EP Gain", ES_EP_GAIN, ES_EP_GAIN_SHIFT,
			ES_EP_GAIN_MAX, 0, ep_tlv),
	SOC_SINGLE_TLV("AUXINL Gain", ES_AUXL_GAIN, ES_AUXL_GAIN_SHIFT,
			ES_AUXL_GAIN_MAX, 0, aux_tlv),
	SOC_SINGLE_TLV("AUXINR Gain", ES_AUXR_GAIN, ES_AUXR_GAIN_SHIFT,
			ES_AUXR_GAIN_MAX, 0, aux_tlv),


	SOC_SINGLE_TLV("MIC0 Gain", ES_MIC0_GAIN, ES_MIC0_GAIN_SHIFT,
			ES_MIC0_GAIN_MAX, 0, mic_tlv),
	SOC_SINGLE_TLV("MIC1 Gain", ES_MIC1_GAIN, ES_MIC1_GAIN_SHIFT,
			ES_MIC1_GAIN_MAX, 0, mic_tlv),
	SOC_SINGLE_TLV("MIC2 Gain", ES_MIC2_GAIN, ES_MIC2_GAIN_SHIFT,
			ES_MIC2_GAIN_MAX, 0, mic_tlv),
	SOC_SINGLE_TLV("MIC3 Gain", ES_MIC3_GAIN, ES_MIC3_GAIN_SHIFT,
			ES_MIC3_GAIN_MAX, 0, mic_tlv),
	SOC_SINGLE_TLV("MICHS Gain", ES_MICHS_GAIN, ES_MICHS_GAIN_SHIFT,
			ES_MICHS_GAIN_MAX, 0, mic_tlv),

	SOC_SINGLE("EP Mute", ES_EP_MUTE, ES_EP_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("HP Mute", ES_HP_L_MUTE, ES_HP_L_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("SPKRL Mute", ES_SPKR_L_MUTE, ES_SPKR_L_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("SPKRR Mute", ES_SPKR_R_MUTE, ES_SPKR_R_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("LO Mute", ES_LO_L_MUTE, ES_LO_L_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("MIC 0 Mute", ES_MIC0_MUTE, ES_MIC0_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("MIC 1 Mute", ES_MIC1_MUTE, ES_MIC1_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("MIC 2 Mute", ES_MIC2_MUTE, ES_MIC2_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("MIC 3 Mute", ES_MIC3_MUTE, ES_MIC3_MUTE_SHIFT, 1, 0),
	SOC_SINGLE("MICHS Mute", ES_MICHS_MUTE, ES_MICHS_MUTE_SHIFT, 1, 0),

	SOC_VALUE_ENUM("MIC0 Bias Output Voltage",
			mic0_bias_output_voltage_enum),
	SOC_VALUE_ENUM("MIC1 Bias Output Voltage",
			mic1_bias_output_voltage_enum),
	SOC_VALUE_ENUM("MIC2 Bias Output Voltage",
			mic2_bias_output_voltage_enum),
	SOC_VALUE_ENUM("MIC3 Bias Output Voltage",
			mic3_bias_output_voltage_enum),
	SOC_VALUE_ENUM("MICHS Bias Output Voltage",
			michs_bias_output_voltage_enum),

	SOC_ENUM("MIC0 Input Impedance Mode", mic0_zin_mode_enum),
	SOC_ENUM("MIC1 Input Impedance Mode", mic1_zin_mode_enum),
	SOC_ENUM("MIC2 Input Impedance Mode", mic2_zin_mode_enum),
	SOC_ENUM("MIC3 Input Impedance Mode", mic3_zin_mode_enum),
	SOC_ENUM("MICHS Input Impedance Mode", michs_zin_mode_enum),
};


static const char * const pga0_mux_text[] = {
	"MIC-0", "MIC-1", "MIC-HS", "HPZ"
};

static const struct soc_enum pga0_mux_enum =
	SOC_ENUM_SINGLE(ES_PGA0_SEL, ES_PGA0_SEL_SHIFT,
		ARRAY_SIZE(pga0_mux_text), pga0_mux_text);

static const struct snd_kcontrol_new pga0_mux_controls =
		SOC_DAPM_ENUM("PGA0 MUX mux", pga0_mux_enum);

static const char * const pga1_mux_text[] = {
	"MIC-1", "MIC-2", "MIC-HS", "MIC-3"
};

static const struct soc_enum pga1_mux_enum =
	SOC_ENUM_SINGLE(ES_PGA1_SEL, ES_PGA1_SEL_SHIFT,
		ARRAY_SIZE(pga1_mux_text), pga1_mux_text);

static const struct snd_kcontrol_new pga1_mux_controls =
		SOC_DAPM_ENUM("PGA1 MUX mux", pga1_mux_enum);

static const char * const pga2_mux_text[] = {
	"MIC-2", "MIC-3", "AUX-R", "AUX-D"
};

static const struct soc_enum pga2_mux_enum =
	SOC_ENUM_SINGLE(ES_PGA2_SEL, ES_PGA2_SEL_SHIFT,
		ARRAY_SIZE(pga2_mux_text), pga2_mux_text);

static const struct snd_kcontrol_new pga2_mux_controls =
		SOC_DAPM_ENUM("PGA2 MUX mux", pga2_mux_enum);

static const char * const pga3_mux_text[] = {
	"MIC-3", "HPZ", "AUX-L", "AUX-D"
};

static const struct soc_enum pga3_mux_enum =
	SOC_ENUM_SINGLE(ES_PGA3_SEL, ES_PGA3_SEL_SHIFT,
		ARRAY_SIZE(pga3_mux_text), pga3_mux_text);

static const struct snd_kcontrol_new pga3_mux_controls =
		SOC_DAPM_ENUM("PGA3 MUX mux", pga3_mux_enum);

static const char * const adc_dmic0_mux_text[] = {
	"ADC", "Voice ADC", "DMIC"
};

static const struct soc_enum adc_dmic0_sel_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDMSEL0, ES_DMIC_PDMSEL0_SHIFT,
			ARRAY_SIZE(adc_dmic0_mux_text), adc_dmic0_mux_text);

static const struct snd_kcontrol_new adc_dmic0_sel_mux_controls =
		SOC_DAPM_ENUM("PDM0 IN MUX mux", adc_dmic0_sel_enum);

static const char * const adc_dmic1_mux_text[] = {
	"ADC", "DMIC"
};

static const struct soc_enum adc_dmic1_sel_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDMSEL1, ES_DMIC_PDMSEL1_SHIFT,
			ARRAY_SIZE(adc_dmic1_mux_text), adc_dmic1_mux_text);

static const struct snd_kcontrol_new adc_dmic1_sel_mux_controls =
		SOC_DAPM_ENUM("PDM1 IN MUX mux", adc_dmic1_sel_enum);

static const struct soc_enum adc_dmic2_sel_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDMSEL2, ES_DMIC_PDMSEL2_SHIFT,
			ARRAY_SIZE(adc_dmic1_mux_text), adc_dmic1_mux_text);

static const struct snd_kcontrol_new adc_dmic2_sel_mux_controls =
		SOC_DAPM_ENUM("PDM2 IN MUX mux", adc_dmic2_sel_enum);

static const struct soc_enum adc_dmic3_sel_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDMSEL3, ES_DMIC_PDMSEL3_SHIFT,
			ARRAY_SIZE(adc_dmic1_mux_text), adc_dmic1_mux_text);

static const struct snd_kcontrol_new adc_dmic3_sel_mux_controls =
		SOC_DAPM_ENUM("PDM3 IN MUX mux", adc_dmic3_sel_enum);

static const char * const dmic_mux_text[] = {
	"DMIC-0", "DMIC-1", "DMIC-2", "DMIC-3"
};

static const struct soc_enum dmic0_mux_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDM0_MIC_SEL, ES_DMIC_PDM0_MIC_SEL_SHIFT,
			ARRAY_SIZE(dmic_mux_text), dmic_mux_text);

static const struct snd_kcontrol_new dmic0_pdm_mux_controls =
		SOC_DAPM_ENUM("PDM0 DMIC MUX mux", dmic0_mux_enum);

static const struct soc_enum dmic1_mux_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDM1_MIC_SEL, ES_DMIC_PDM1_MIC_SEL_SHIFT,
			ARRAY_SIZE(dmic_mux_text), dmic_mux_text);

static const struct snd_kcontrol_new dmic1_pdm_mux_controls =
		SOC_DAPM_ENUM("PDM1 DMIC MUX mux", dmic1_mux_enum);

static const struct soc_enum dmic2_mux_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDM2_MIC_SEL, ES_DMIC_PDM2_MIC_SEL_SHIFT,
			ARRAY_SIZE(dmic_mux_text), dmic_mux_text);

static const struct snd_kcontrol_new dmic2_pdm_mux_controls =
		SOC_DAPM_ENUM("PDM2 DMIC MUX mux", dmic2_mux_enum);

static const struct soc_enum dmic3_mux_enum =
	SOC_ENUM_SINGLE(ES_DMIC_PDM3_MIC_SEL, ES_DMIC_PDM3_MIC_SEL_SHIFT,
			ARRAY_SIZE(dmic_mux_text), dmic_mux_text);

static const struct snd_kcontrol_new dmic3_pdm_mux_controls =
		SOC_DAPM_ENUM("PDM3 DMIC MUX mux", dmic3_mux_enum);


static const struct snd_kcontrol_new hp_pga_mux_control =
	SOC_DAPM_SINGLE("Switch", ES_HP_L_EN, ES_HP_L_EN_SHIFT, 1, 0);

static const struct snd_kcontrol_new ep_mix[] = {
	SOC_DAPM_SINGLE("DAC0L", ES_DAC0_L_EP_SEL, ES_DAC0_L_EP_SEL_SHIFT,
									1, 0),
	SOC_DAPM_SINGLE("DAC1L", ES_DAC1_L_EP_SEL, ES_DAC1_L_EP_SEL_SHIFT,
									1, 0),

};

static const struct snd_kcontrol_new spkrl_mix[] = {
	SOC_DAPM_SINGLE("DAC0L", ES_DAC0_L_TO_SPKR_L, ES_DAC0_L_TO_SPKR_L_SHIFT,
								1, 0),
	SOC_DAPM_SINGLE("DAC1L", ES_DAC1_L_TO_SPKR_L, ES_DAC1_L_TO_SPKR_L_SHIFT,
								 1, 0),
};

static const struct snd_kcontrol_new spkrr_mix[] = {
	SOC_DAPM_SINGLE("DAC0R", ES_DAC0_R_TO_SPKR_R, ES_DAC0_R_TO_SPKR_R_SHIFT,
								 1, 0),
	SOC_DAPM_SINGLE("DAC1R", ES_DAC1_R_TO_SPKR_R, ES_DAC1_R_TO_SPKR_R_SHIFT,
								 1, 0),
};

static const struct snd_kcontrol_new lo_l_mix[] = {
	SOC_DAPM_SINGLE("DAC0L", ES_DAC0_L_TO_LO_L, ES_DAC0_L_TO_LO_L_SHIFT,
								1, 0),
	SOC_DAPM_SINGLE("DAC1L", ES_DAC1_L_TO_LO_L, ES_DAC1_L_TO_LO_L_SHIFT,
								1, 0),
};


static int mic_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;
	pr_debug("%s() %x\n", __func__, SND_SOC_DAPM_EVENT_ON(event));

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (!strncmp(w->name, "MIC0", 4))
			snd_soc_write(codec, ES_MIC0_EN, 1);
		else if (!strncmp(w->name, "MIC1", 4))
			snd_soc_write(codec, ES_MIC1_EN, 1);
		else if (!strncmp(w->name, "MIC2", 4))
			snd_soc_write(codec, ES_MIC2_EN, 1);
		else if (!strncmp(w->name, "MIC3", 4))
			snd_soc_write(codec, ES_MIC3_EN, 1);
		else if (!strncmp(w->name, "MICHS", 5))
			snd_soc_write(codec, ES_MICHS_EN, 1);
		else if (!strncmp(w->name, "AUXINM", 6))
			snd_soc_write(codec, ES_AUXR_EN, 1);
		else if (!strncmp(w->name, "AUXINP", 6))
			snd_soc_write(codec, ES_AUXL_EN, 1);
		else if (!strncmp(w->name, "DMIC0", 5))
			snd_soc_write(codec, ES_DMIC0_EN, 1);
		else if (!strncmp(w->name, "DMIC1", 5))
			snd_soc_write(codec, ES_DMIC1_EN, 1);
		else if (!strncmp(w->name, "DMIC2", 5))
			snd_soc_write(codec, ES_DMIC2_EN, 1);
		else if (!strncmp(w->name, "DMIC3", 5))
			snd_soc_write(codec, ES_DMIC3_EN, 1);
		else {
			pr_err("%s: Invalid Mic Widget ON = %s\n",
			       __func__, w->name);
			return -EINVAL;
		}

	} else {
		if (!strncmp(w->name, "MIC0", 4))
			snd_soc_write(codec, ES_MIC0_EN, 0);
		else if (!strncmp(w->name, "MIC1", 4))
			snd_soc_write(codec, ES_MIC1_EN, 0);
		else if (!strncmp(w->name, "MIC2", 4))
			snd_soc_write(codec, ES_MIC2_EN, 0);
		else if (!strncmp(w->name, "MIC3", 4))
			snd_soc_write(codec, ES_MIC3_EN, 0);
		else if (!strncmp(w->name, "MICHS", 5))
			snd_soc_write(codec, ES_MICHS_EN, 0);
		else if (!strncmp(w->name, "AUXINM", 6))
			snd_soc_write(codec, ES_AUXR_EN, 0);
		else if (!strncmp(w->name, "AUXINP", 6))
			snd_soc_write(codec, ES_AUXL_EN, 0);
		else if (!strncmp(w->name, "DMIC0", 5))
			snd_soc_write(codec, ES_DMIC0_EN, 0);
		else if (!strncmp(w->name, "DMIC1", 5))
			snd_soc_write(codec, ES_DMIC1_EN, 0);
		else if (!strncmp(w->name, "DMIC2", 5))
			snd_soc_write(codec, ES_DMIC2_EN, 0);
		else if (!strncmp(w->name, "DMIC3", 5))
			snd_soc_write(codec, ES_DMIC3_EN, 0);
		else {
			pr_err("%s: Invalid Mic Widget OFF = %s\n",
			       __func__, w->name);
			return -EINVAL;
		}
	}
	return 0;
}

const struct snd_soc_dapm_widget es_codec_dapm_widgets[] = {

	/* Inputs */
	SND_SOC_DAPM_MIC("MIC0", mic_event),
	SND_SOC_DAPM_MIC("MIC1", mic_event),
	SND_SOC_DAPM_MIC("MIC2", mic_event),
	SND_SOC_DAPM_MIC("MIC3", mic_event),
	SND_SOC_DAPM_MIC("MICHS", mic_event),
	SND_SOC_DAPM_MIC("AUXINM", mic_event),
	SND_SOC_DAPM_MIC("AUXINP", mic_event),
	SND_SOC_DAPM_MIC("DMIC0", mic_event),
	SND_SOC_DAPM_MIC("DMIC1", mic_event),
	SND_SOC_DAPM_MIC("DMIC2", mic_event),
	SND_SOC_DAPM_MIC("DMIC3", mic_event),

	/* Outputs */
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("SPKRL", NULL),
	SND_SOC_DAPM_SPK("SPKRR", NULL),
	SND_SOC_DAPM_OUTPUT("EP"),
	SND_SOC_DAPM_LINE("AUXOUTL", NULL),

	/* Microphone bias */
	SND_SOC_DAPM_SUPPLY("MIC0 Bias", ES_MB0_MODE,
		ES_MB0_MODE_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MIC1 Bias", ES_MB1_MODE,
		ES_MB1_MODE_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MIC2 Bias", ES_MB2_MODE,
		ES_MB2_MODE_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MIC3 Bias", ES_MB3_MODE,
		ES_MB3_MODE_SHIFT, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("DMIC0 Bias", ES_DMIC_CLK_EN0,
			ES_DMIC_CLK_EN0_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DMIC1 Bias", ES_DMIC_CLK_EN1,
			ES_DMIC_CLK_EN1_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DMIC2 Bias", ES_DMIC_CLK_EN2,
			ES_DMIC_CLK_EN2_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DMIC3 Bias", ES_DMIC_CLK_EN3,
			ES_DMIC_CLK_EN3_SHIFT, 0, NULL, 0),

	SND_SOC_DAPM_MUX("PGA0 MUX", SND_SOC_NOPM, 0, 0,
		&pga0_mux_controls),
	SND_SOC_DAPM_MUX("PGA1 MUX", SND_SOC_NOPM, 0, 0,
		&pga1_mux_controls),
	SND_SOC_DAPM_MUX("PGA2 MUX", SND_SOC_NOPM, 0, 0,
		&pga2_mux_controls),
	SND_SOC_DAPM_MUX("PGA3 MUX", SND_SOC_NOPM, 0, 0,
		&pga3_mux_controls),

	SND_SOC_DAPM_MUX("PDM0 DMIC MUX", SND_SOC_NOPM, 0, 0,
		&dmic0_pdm_mux_controls),
	SND_SOC_DAPM_MUX("PDM1 DMIC MUX", SND_SOC_NOPM, 0, 0,
		&dmic1_pdm_mux_controls),
	SND_SOC_DAPM_MUX("PDM2 DMIC MUX", SND_SOC_NOPM, 0, 0,
		&dmic2_pdm_mux_controls),
	SND_SOC_DAPM_MUX("PDM3 DMIC MUX", SND_SOC_NOPM, 0, 0,
		&dmic3_pdm_mux_controls),

	SND_SOC_DAPM_MUX("PDM0 IN MUX", SND_SOC_NOPM, 0, 0,
		&adc_dmic0_sel_mux_controls),
	SND_SOC_DAPM_MUX("PDM1 IN MUX", SND_SOC_NOPM, 0, 0,
		&adc_dmic1_sel_mux_controls),
	SND_SOC_DAPM_MUX("PDM2 IN MUX", SND_SOC_NOPM, 0, 0,
		&adc_dmic2_sel_mux_controls),
	SND_SOC_DAPM_MUX("PDM3 IN MUX", SND_SOC_NOPM, 0, 0,
		&adc_dmic3_sel_mux_controls),
	/* ADC */
	SND_SOC_DAPM_ADC("ADC0", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC1", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC2", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC3", NULL, SND_SOC_NOPM, 0, 0),

	/* DAC */
	SND_SOC_DAPM_DAC("DAC0L", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC0R", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC1L", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC1R", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SWITCH("HP Output", SND_SOC_NOPM, 0, 0,
			&hp_pga_mux_control),

	/* Earphone Mixer */
	SND_SOC_DAPM_MIXER("EP MIXER", SND_SOC_NOPM, 0, 0,
		ep_mix, ARRAY_SIZE(ep_mix)),

	/* Handsfree Mixer */
	SND_SOC_DAPM_MIXER("SPKRL MIXER", SND_SOC_NOPM, 0, 0,
		spkrl_mix, ARRAY_SIZE(spkrl_mix)),
	SND_SOC_DAPM_MIXER("SPKRR MIXER", SND_SOC_NOPM, 0, 0,
		spkrr_mix, ARRAY_SIZE(spkrr_mix)),

	/* LineOut Mixer */
	SND_SOC_DAPM_MIXER("LO MIXER", SND_SOC_NOPM, 0, 0,
		lo_l_mix, ARRAY_SIZE(lo_l_mix)),

	/* Output PGAs */
	SND_SOC_DAPM_PGA("SPKRL PGA", ES_SPKR_L_ON, ES_SPKR_L_ON_SHIFT,
		0, NULL, 0),
	SND_SOC_DAPM_PGA("SPKRR PGA", ES_SPKR_R_ON, ES_SPKR_R_ON_SHIFT,
		0, NULL, 0),
	SND_SOC_DAPM_PGA("LO PGA", ES_LO_L_EN, ES_LO_L_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("EP PGA", ES_EP_EN, ES_EP_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HP PGA", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Input PGAs */
	SND_SOC_DAPM_PGA("PGA0", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("PGA3", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_PGA("PDM0_PGA", ES_DMIC_CLK_EN, ES_DMIC_CLK_EN_SHIFT,
			0, NULL, 0),
	SND_SOC_DAPM_PGA("PDM1_PGA", ES_DMIC_CLK_EN, ES_DMIC_CLK_EN_SHIFT,
			0, NULL, 0),
	SND_SOC_DAPM_PGA("PDM2_PGA", ES_DMIC_CLK_EN, ES_DMIC_CLK_EN_SHIFT,
			0, NULL, 0),
	SND_SOC_DAPM_PGA("PDM3_PGA", ES_DMIC_CLK_EN, ES_DMIC_CLK_EN_SHIFT,
			0, NULL, 0),
};

/* TODO */
static const struct snd_soc_dapm_route intercon[] = {

	/* Capture path */

	{"MIC0", NULL, "MIC0 Bias"},
	{"MIC1", NULL, "MIC1 Bias"},
	{"MIC2", NULL, "MIC2 Bias"},
	{"MIC3", NULL, "MIC3 Bias"},

	{"DMIC0", NULL, "DMIC0 Bias"},
	{"DMIC1", NULL, "DMIC1 Bias"},
	{"DMIC2", NULL, "DMIC2 Bias"},
	{"DMIC3", NULL, "DMIC3 Bias"},

	{"PGA0 MUX", "MIC-0", "MIC0"},
	{"PGA0 MUX", "MIC-1", "MIC1"},
	{"PGA0 MUX", "MIC-HS", "MICHS"},

	{"PGA1 MUX", "MIC-1", "MIC1"},
	{"PGA1 MUX", "MIC-2", "MIC2"},
	{"PGA1 MUX", "MIC-HS", "MICHS"},
	{"PGA1 MUX", "MIC-3", "MIC3"},

	{"PGA2 MUX", "MIC-2", "MIC2"},
	{"PGA2 MUX", "MIC-3", "MIC3"},
	{"PGA2 MUX", "AUX-R", "AUXINM"},

	{"PGA3 MUX", "MIC-3", "MIC3"},
	{"PGA3 MUX", "AUX-L", "AUXINP"},

	{"PDM0 DMIC MUX", "DMIC-0", "DMIC0"},
	{"PDM0 DMIC MUX", "DMIC-1", "DMIC1"},
	{"PDM0 DMIC MUX", "DMIC-2", "DMIC2"},
	{"PDM0 DMIC MUX", "DMIC-3", "DMIC3"},

	{"PDM1 DMIC MUX", "DMIC-0", "DMIC0"},
	{"PDM1 DMIC MUX", "DMIC-1", "DMIC1"},
	{"PDM1 DMIC MUX", "DMIC-2", "DMIC2"},
	{"PDM1 DMIC MUX", "DMIC-3", "DMIC3"},

	{"PDM2 DMIC MUX", "DMIC-0", "DMIC0"},
	{"PDM2 DMIC MUX", "DMIC-1", "DMIC1"},
	{"PDM2 DMIC MUX", "DMIC-2", "DMIC2"},
	{"PDM2 DMIC MUX", "DMIC-3", "DMIC3"},

	{"PDM3 DMIC MUX", "DMIC-0", "DMIC0"},
	{"PDM3 DMIC MUX", "DMIC-1", "DMIC1"},
	{"PDM3 DMIC MUX", "DMIC-2", "DMIC2"},
	{"PDM3 DMIC MUX", "DMIC-3", "DMIC3"},

	{"PGA0", NULL, "PGA0 MUX"},
	{"PGA1", NULL, "PGA1 MUX"},
	{"PGA2", NULL, "PGA2 MUX"},
	{"PGA3", NULL, "PGA3 MUX"},

	{"PDM0_PGA", NULL, "PDM0 DMIC MUX"},
	{"PDM1_PGA", NULL, "PDM1 DMIC MUX"},
	{"PDM2_PGA", NULL, "PDM2 DMIC MUX"},
	{"PDM3_PGA", NULL, "PDM3 DMIC MUX"},

	{"PDM0 IN MUX", "ADC", "PGA0"},
	{"PDM0 IN MUX", "DMIC", "PDM0_PGA"},

	{"PDM1 IN MUX", "ADC", "PGA1"},
	{"PDM1 IN MUX", "DMIC", "PDM1_PGA"},

	{"PDM2 IN MUX", "ADC", "PGA2"},
	{"PDM2 IN MUX", "DMIC", "PDM2_PGA"},

	{"PDM3 IN MUX", "ADC", "PGA3"},
	{"PDM3 IN MUX", "DMIC", "PDM3_PGA"},

	{"ADC0", NULL, "PDM0 IN MUX"},
	{"ADC1", NULL, "PDM1 IN MUX"},
	{"ADC2", NULL, "PDM2 IN MUX"},
	{"ADC3", NULL, "PDM3 IN MUX"},

	/* Playback path */

	{"SPKRL MIXER", "DAC0L", "DAC0L"},
	{"SPKRL MIXER", "DAC1L", "DAC1L"},

	{"SPKRR MIXER", "DAC0R", "DAC0R"},
	{"SPKRR MIXER", "DAC1R", "DAC1R"},

	{"EP MIXER", "DAC0L", "DAC0L"},
	{"EP MIXER", "DAC1L", "DAC1L"},

	{"LO MIXER", "DAC0L", "DAC0L"},
	{"LO MIXER", "DAC1L", "DAC1L"},

	{"HP PGA", NULL, "DAC0L"},

	{"LO PGA", NULL, "LO MIXER"},
	{"SPKRL PGA", NULL, "SPKRL MIXER"},
	{"SPKRR PGA", NULL, "SPKRR MIXER"},
	{"EP PGA", NULL, "EP MIXER"},

	{"HP Output", "Switch", "HP PGA"},

	{"AUXOUTL", NULL, "LO PGA"},
	{"HP", NULL, "HP Output"},
	{"SPKRL", NULL, "SPKRL PGA"},
	{"SPKRR", NULL, "SPKRR PGA"},
	{"EP", NULL, "EP PGA"},

};

int es_analog_add_snd_soc_controls(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_add_codec_controls(codec, es_codec_snd_controls,
			ARRAY_SIZE(es_codec_snd_controls));
	return rc;
}
int es_analog_add_snd_soc_dapm_controls(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_dapm_new_controls(&codec->dapm, es_codec_dapm_widgets,
					ARRAY_SIZE(es_codec_dapm_widgets));

	return rc;
}
int es_analog_add_snd_soc_route_map(struct snd_soc_codec *codec)
{
	int rc;

	rc = snd_soc_dapm_add_routes(&codec->dapm, intercon,
					ARRAY_SIZE(intercon));

	return rc;
}
