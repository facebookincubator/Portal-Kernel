/*
 * es-a375-reg.h  --  eS857 analog register header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*************************************************************************************
** SCRIPT GENERATED FILE - DO NOT EDIT
**
** es-a375a0-reg.h
**
** This file contains the addresses for the codec hardware fields.  Addresses
** are automatically extracted by script from the codec register spreadsheets.
**
** This file generated on 4/30/2015 at 4:30:02 PM
**
*************************************************************************************/

#ifndef __ES_A375_REG_H__
#define __ES_A375_REG_H__


#define ES_MIC0_GAIN_MAX                     0x14
#define ES_MIC1_GAIN_MAX                     0x14
#define ES_MIC2_GAIN_MAX                     0x14
#define ES_MIC3_GAIN_MAX                     0x14
#define ES_MICHS_GAIN_MAX                    0x14
#define ES_AUXL_GAIN_MAX                     0x14
#define ES_AUXR_GAIN_MAX                     0x14
#define ES_EP_GAIN_MAX                       0x0F
#define ES_HPL_GAIN_MAX                      0x0F
#define ES_HPR_GAIN_MAX                      0x0F
#define ES_HPUL_GAIN_MAX                     0x0F
#define ES_HPUR_GAIN_MAX                     0x0F

#define ES_CHIP_EN                           0x0000
#define ES_CHIP_EN_MASK                      0x01
#define ES_CHIP_EN_SHIFT                     0
#define ES_CHIP_EN_WIDTH                     1

#define ES_STANDBY                           0x0001
#define ES_STANDBY_MASK                      0x01
#define ES_STANDBY_SHIFT                     0
#define ES_STANDBY_WIDTH                     1

#define ES_MB0_MODE                          0x000F
#define ES_MB0_MODE_MASK                     0x03
#define ES_MB0_MODE_SHIFT                    0
#define ES_MB0_MODE_WIDTH                    2

#define ES_MB1_MODE                          0x0010
#define ES_MB1_MODE_MASK                     0x03
#define ES_MB1_MODE_SHIFT                    0
#define ES_MB1_MODE_WIDTH                    2

#define ES_MB2_MODE                          0x0011
#define ES_MB2_MODE_MASK                     0x03
#define ES_MB2_MODE_SHIFT                    0
#define ES_MB2_MODE_WIDTH                    2

#define ES_MB3_MODE                          0x0012
#define ES_MB3_MODE_MASK                     0x03
#define ES_MB3_MODE_SHIFT                    0
#define ES_MB3_MODE_WIDTH                    2

#define ES_MIC0_EN                           0x0013
#define ES_MIC0_EN_MASK                      0x01
#define ES_MIC0_EN_SHIFT                     0
#define ES_MIC0_EN_WIDTH                     1

#define ES_MIC0_GAIN                         0x0014
#define ES_MIC0_GAIN_MASK                    0x1F
#define ES_MIC0_GAIN_SHIFT                   0
#define ES_MIC0_GAIN_WIDTH                   5

#define ES_MIC0_MUTE                         0x0016
#define ES_MIC0_MUTE_MASK                    0x01
#define ES_MIC0_MUTE_SHIFT                   0
#define ES_MIC0_MUTE_WIDTH                   1

#define ES_MIC0_ZIN_MODE                     0x0017
#define ES_MIC0_ZIN_MODE_MASK                0x03
#define ES_MIC0_ZIN_MODE_SHIFT               0
#define ES_MIC0_ZIN_MODE_WIDTH               2

#define ES_MIC1_ZIN_MODE                     0x0018
#define ES_MIC1_ZIN_MODE_MASK                0x03
#define ES_MIC1_ZIN_MODE_SHIFT               0
#define ES_MIC1_ZIN_MODE_WIDTH               2

#define ES_MIC2_ZIN_MODE                     0x0019
#define ES_MIC2_ZIN_MODE_MASK                0x03
#define ES_MIC2_ZIN_MODE_SHIFT               0
#define ES_MIC2_ZIN_MODE_WIDTH               2

#define ES_MIC3_ZIN_MODE                     0x001A
#define ES_MIC3_ZIN_MODE_MASK                0x03
#define ES_MIC3_ZIN_MODE_SHIFT               0
#define ES_MIC3_ZIN_MODE_WIDTH               2

#define ES_MIC1_EN                           0x001B
#define ES_MIC1_EN_MASK                      0x01
#define ES_MIC1_EN_SHIFT                     0
#define ES_MIC1_EN_WIDTH                     1

#define ES_MIC1_GAIN                         0x001C
#define ES_MIC1_GAIN_MASK                    0x1F
#define ES_MIC1_GAIN_SHIFT                   0
#define ES_MIC1_GAIN_WIDTH                   5

#define ES_MIC1_MUTE                         0x001E
#define ES_MIC1_MUTE_MASK                    0x01
#define ES_MIC1_MUTE_SHIFT                   0
#define ES_MIC1_MUTE_WIDTH                   1

#define ES_AUXL_EN                           0x0026
#define ES_AUXL_EN_MASK                      0x01
#define ES_AUXL_EN_SHIFT                     0
#define ES_AUXL_EN_WIDTH                     1

#define ES_AUXL_GAIN                         0x0027
#define ES_AUXL_GAIN_MASK                    0x1F
#define ES_AUXL_GAIN_SHIFT                   0
#define ES_AUXL_GAIN_WIDTH                   5

#define ES_AUXL_MUTE                         0x0029
#define ES_AUXL_MUTE_MASK                    0x01
#define ES_AUXL_MUTE_SHIFT                   0
#define ES_AUXL_MUTE_WIDTH                   1

#define ES_MIC2_EN                           0x002A
#define ES_MIC2_EN_MASK                      0x01
#define ES_MIC2_EN_SHIFT                     0
#define ES_MIC2_EN_WIDTH                     1

#define ES_MIC2_GAIN                         0x002B
#define ES_MIC2_GAIN_MASK                    0x1F
#define ES_MIC2_GAIN_SHIFT                   0
#define ES_MIC2_GAIN_WIDTH                   5

#define ES_MIC2_MUTE                         0x002D
#define ES_MIC2_MUTE_MASK                    0x01
#define ES_MIC2_MUTE_SHIFT                   0
#define ES_MIC2_MUTE_WIDTH                   1

#define ES_AUXR_EN                           0x002E
#define ES_AUXR_EN_MASK                      0x01
#define ES_AUXR_EN_SHIFT                     0
#define ES_AUXR_EN_WIDTH                     1

#define ES_AUXR_GAIN                         0x002F
#define ES_AUXR_GAIN_MASK                    0x1F
#define ES_AUXR_GAIN_SHIFT                   0
#define ES_AUXR_GAIN_WIDTH                   5

#define ES_AUXR_MUTE                         0x0031
#define ES_AUXR_MUTE_MASK                    0x01
#define ES_AUXR_MUTE_SHIFT                   0
#define ES_AUXR_MUTE_WIDTH                   1

#define ES_MICHS_EN                          0x0042
#define ES_MICHS_EN_MASK                     0x01
#define ES_MICHS_EN_SHIFT                    0
#define ES_MICHS_EN_WIDTH                    1

#define ES_MICHS_GAIN                        0x0043
#define ES_MICHS_GAIN_MASK                   0x1F
#define ES_MICHS_GAIN_SHIFT                  0
#define ES_MICHS_GAIN_WIDTH                  5

#define ES_MICHS_MUTE                        0x0045
#define ES_MICHS_MUTE_MASK                   0x01
#define ES_MICHS_MUTE_SHIFT                  0
#define ES_MICHS_MUTE_WIDTH                  1

#define ES_EP_GAIN                           0x004D
#define ES_EP_GAIN_MASK                      0x0F
#define ES_EP_GAIN_SHIFT                     0
#define ES_EP_GAIN_WIDTH                     4

#define ES_EP_MUTE                           0x0050
#define ES_EP_MUTE_MASK                      0x01
#define ES_EP_MUTE_SHIFT                     0
#define ES_EP_MUTE_WIDTH                     1

#define ES_EP_EN                             0x0051
#define ES_EP_EN_MASK                        0x01
#define ES_EP_EN_SHIFT                       0
#define ES_EP_EN_WIDTH                       1

#define ES_DAC0_L_EP_SEL                     0x0052
#define ES_DAC0_L_EP_SEL_MASK                0x01
#define ES_DAC0_L_EP_SEL_SHIFT               0
#define ES_DAC0_L_EP_SEL_WIDTH               1

#define ES_DAC1_L_EP_SEL                     0x0053
#define ES_DAC1_L_EP_SEL_MASK                0x01
#define ES_DAC1_L_EP_SEL_SHIFT               0
#define ES_DAC1_L_EP_SEL_WIDTH               1

#define ES_HPU_L_EN                          0x0061
#define ES_HPU_L_EN_MASK                     0x01
#define ES_HPU_L_EN_SHIFT                    0
#define ES_HPU_L_EN_WIDTH                    1

#define ES_HPU_L_GAIN                        0x0062
#define ES_HPU_L_GAIN_MASK                   0x0F
#define ES_HPU_L_GAIN_SHIFT                  0
#define ES_HPU_L_GAIN_WIDTH                  4

#define ES_HPU_L_MUTE                        0x0063
#define ES_HPU_L_MUTE_MASK                   0x01
#define ES_HPU_L_MUTE_SHIFT                  0
#define ES_HPU_L_MUTE_WIDTH                  1

#define ES_HP_L_EN                           0x0078
#define ES_HP_L_EN_MASK                      0x01
#define ES_HP_L_EN_SHIFT                     0
#define ES_HP_L_EN_WIDTH                     1

#define ES_HP_L_GAIN                         0x0079
#define ES_HP_L_GAIN_MASK                    0x0F
#define ES_HP_L_GAIN_SHIFT                   0
#define ES_HP_L_GAIN_WIDTH                   4

#define ES_HP_L_MUTE                         0x007A
#define ES_HP_L_MUTE_MASK                    0x01
#define ES_HP_L_MUTE_SHIFT                   0
#define ES_HP_L_MUTE_WIDTH                   1

#define ES_MBU1_MODE                         0x00D0
#define ES_MBU1_MODE_MASK                    0x03
#define ES_MBU1_MODE_SHIFT                   0
#define ES_MBU1_MODE_WIDTH                   2

#define ES_MBU2_MODE                         0x00D1
#define ES_MBU2_MODE_MASK                    0x03
#define ES_MBU2_MODE_SHIFT                   0
#define ES_MBU2_MODE_WIDTH                   2

#define ES_MBU1_2_TRIM                       0x00D2
#define ES_MBU1_2_TRIM_MASK                  0x07
#define ES_MBU1_2_TRIM_SHIFT                 0
#define ES_MBU1_2_TRIM_WIDTH                 3

#define ES_LPADC_ADC_ON                      0x00E8
#define ES_LPADC_ADC_ON_MASK                 0x01
#define ES_LPADC_ADC_ON_SHIFT                0
#define ES_LPADC_ADC_ON_WIDTH                1

#define ES_MBHS_MODE                         0x00EA
#define ES_MBHS_MODE_MASK                    0x03
#define ES_MBHS_MODE_SHIFT                   0
#define ES_MBHS_MODE_WIDTH                   2

#define ES_MBHS_TRIM                         0x00EB
#define ES_MBHS_TRIM_MASK                    0x07
#define ES_MBHS_TRIM_SHIFT                   0
#define ES_MBHS_TRIM_WIDTH                   3

#define ES_MIC1U_EN                          0x00ED
#define ES_MIC1U_EN_MASK                     0x01
#define ES_MIC1U_EN_SHIFT                    0
#define ES_MIC1U_EN_WIDTH                    1

#define ES_MIC1U_GAIN                        0x00EE
#define ES_MIC1U_GAIN_MASK                   0x1F
#define ES_MIC1U_GAIN_SHIFT                  0
#define ES_MIC1U_GAIN_WIDTH                  5

#define ES_MIC1U_MUTE                        0x00F0
#define ES_MIC1U_MUTE_MASK                   0x01
#define ES_MIC1U_MUTE_SHIFT                  0
#define ES_MIC1U_MUTE_WIDTH                  1

#define ES_MIC2U_EN                          0x00F1
#define ES_MIC2U_EN_MASK                     0x01
#define ES_MIC2U_EN_SHIFT                    0
#define ES_MIC2U_EN_WIDTH                    1

#define ES_MIC2U_GAIN                        0x00F2
#define ES_MIC2U_GAIN_MASK                   0x1F
#define ES_MIC2U_GAIN_SHIFT                  0
#define ES_MIC2U_GAIN_WIDTH                  5

#define ES_MIC2U_MUTE                        0x00F4
#define ES_MIC2U_MUTE_MASK                   0x01
#define ES_MIC2U_MUTE_SHIFT                  0
#define ES_MIC2U_MUTE_WIDTH                  1

#define ES_MB2_TRIM                          0x00FE
#define ES_MB2_TRIM_MASK                     0x07
#define ES_MB2_TRIM_SHIFT                    0
#define ES_MB2_TRIM_WIDTH                    3

#define ES_MB3_TRIM                          0x00FF
#define ES_MB3_TRIM_MASK                     0x07
#define ES_MB3_TRIM_SHIFT                    0
#define ES_MB3_TRIM_WIDTH                    3

#define ES_MB0_TRIM                          0x0101
#define ES_MB0_TRIM_MASK                     0x07
#define ES_MB0_TRIM_SHIFT                    0
#define ES_MB0_TRIM_WIDTH                    3

#define ES_MB1_TRIM                          0x0102
#define ES_MB1_TRIM_MASK                     0x07
#define ES_MB1_TRIM_SHIFT                    0
#define ES_MB1_TRIM_WIDTH                    3

#define ES_MIC3_EN                           0x011F
#define ES_MIC3_EN_MASK                      0x01
#define ES_MIC3_EN_SHIFT                     0
#define ES_MIC3_EN_WIDTH                     1

#define ES_MIC3_GAIN                         0x0120
#define ES_MIC3_GAIN_MASK                    0x1F
#define ES_MIC3_GAIN_SHIFT                   0
#define ES_MIC3_GAIN_WIDTH                   5

#define ES_MIC3_MUTE                         0x0122
#define ES_MIC3_MUTE_MASK                    0x01
#define ES_MIC3_MUTE_SHIFT                   0
#define ES_MIC3_MUTE_WIDTH                   1

#define ES_MICHS_ZIN_MODE                    0x0127
#define ES_MICHS_ZIN_MODE_MASK               0x03
#define ES_MICHS_ZIN_MODE_SHIFT              0
#define ES_MICHS_ZIN_MODE_WIDTH              2

#define ES_AUX_ZIN_MODE                      0x0128
#define ES_AUX_ZIN_MODE_MASK                 0x03
#define ES_AUX_ZIN_MODE_SHIFT                0
#define ES_AUX_ZIN_MODE_WIDTH                2

#define ES_MICU_ZIN_MODE                     0x0129
#define ES_MICU_ZIN_MODE_MASK                0x03
#define ES_MICU_ZIN_MODE_SHIFT               0
#define ES_MICU_ZIN_MODE_WIDTH               2

#define ES_PGA0_SEL                          0x012A
#define ES_PGA0_SEL_MASK                     0x03
#define ES_PGA0_SEL_SHIFT                    0
#define ES_PGA0_SEL_WIDTH                    2

#define ES_PGA1_SEL                          0x012B
#define ES_PGA1_SEL_MASK                     0x03
#define ES_PGA1_SEL_SHIFT                    0
#define ES_PGA1_SEL_WIDTH                    2

#define ES_PGA2_SEL                          0x012C
#define ES_PGA2_SEL_MASK                     0x03
#define ES_PGA2_SEL_SHIFT                    0
#define ES_PGA2_SEL_WIDTH                    2

#define ES_PGA3_SEL                          0x012D
#define ES_PGA3_SEL_MASK                     0x03
#define ES_PGA3_SEL_SHIFT                    0
#define ES_PGA3_SEL_WIDTH                    2

#define ES_DMIC0_EN                          0x0175
#define ES_DMIC0_EN_MASK                     0x01
#define ES_DMIC0_EN_SHIFT                    0
#define ES_DMIC0_EN_WIDTH                    1

#define ES_DMIC1_EN                          0x0176
#define ES_DMIC1_EN_MASK                     0x01
#define ES_DMIC1_EN_SHIFT                    0
#define ES_DMIC1_EN_WIDTH                    1

#define ES_DMIC2_EN                          0x0177
#define ES_DMIC2_EN_MASK                     0x01
#define ES_DMIC2_EN_SHIFT                    0
#define ES_DMIC2_EN_WIDTH                    1

#define ES_DMIC3_EN                          0x0178
#define ES_DMIC3_EN_MASK                     0x01
#define ES_DMIC3_EN_SHIFT                    0
#define ES_DMIC3_EN_WIDTH                    1

#define ES_DMIC_PDM0_MIC_SEL                 0x017D
#define ES_DMIC_PDM0_MIC_SEL_MASK            0x03
#define ES_DMIC_PDM0_MIC_SEL_SHIFT           0
#define ES_DMIC_PDM0_MIC_SEL_WIDTH           2

#define ES_DMIC_PDM1_MIC_SEL                 0x017E
#define ES_DMIC_PDM1_MIC_SEL_MASK            0x03
#define ES_DMIC_PDM1_MIC_SEL_SHIFT           0
#define ES_DMIC_PDM1_MIC_SEL_WIDTH           2

#define ES_DMIC_PDM2_MIC_SEL                 0x017F
#define ES_DMIC_PDM2_MIC_SEL_MASK            0x03
#define ES_DMIC_PDM2_MIC_SEL_SHIFT           0
#define ES_DMIC_PDM2_MIC_SEL_WIDTH           2

#define ES_DMIC_PDM3_MIC_SEL                 0x0180
#define ES_DMIC_PDM3_MIC_SEL_MASK            0x03
#define ES_DMIC_PDM3_MIC_SEL_SHIFT           0
#define ES_DMIC_PDM3_MIC_SEL_WIDTH           2

#define ES_DMIC_PDMSEL0                      0x0181
#define ES_DMIC_PDMSEL0_MASK                 0x03
#define ES_DMIC_PDMSEL0_SHIFT                0
#define ES_DMIC_PDMSEL0_WIDTH                2

#define ES_DMIC_PDMSEL1                      0x0182
#define ES_DMIC_PDMSEL1_MASK                 0x01
#define ES_DMIC_PDMSEL1_SHIFT                0
#define ES_DMIC_PDMSEL1_WIDTH                1

#define ES_DMIC_PDMSEL2                      0x0183
#define ES_DMIC_PDMSEL2_MASK                 0x01
#define ES_DMIC_PDMSEL2_SHIFT                0
#define ES_DMIC_PDMSEL2_WIDTH                1

#define ES_DMIC_PDMSEL3                      0x0184
#define ES_DMIC_PDMSEL3_MASK                 0x01
#define ES_DMIC_PDMSEL3_SHIFT                0
#define ES_DMIC_PDMSEL3_WIDTH                1

#define ES_DMIC_CLK_EN0                      0x0188
#define ES_DMIC_CLK_EN0_MASK                 0x01
#define ES_DMIC_CLK_EN0_SHIFT                0
#define ES_DMIC_CLK_EN0_WIDTH                1

#define ES_DMIC_CLK_EN1                      0x0189
#define ES_DMIC_CLK_EN1_MASK                 0x01
#define ES_DMIC_CLK_EN1_SHIFT                0
#define ES_DMIC_CLK_EN1_WIDTH                1

#define ES_DMIC_CLK_EN2                      0x018A
#define ES_DMIC_CLK_EN2_MASK                 0x01
#define ES_DMIC_CLK_EN2_SHIFT                0
#define ES_DMIC_CLK_EN2_WIDTH                1

#define ES_DMIC_CLK_EN3                      0x018B
#define ES_DMIC_CLK_EN3_MASK                 0x01
#define ES_DMIC_CLK_EN3_SHIFT                0
#define ES_DMIC_CLK_EN3_WIDTH                1



#define ES_MAX_REGISTER             0x0198


int es_analog_add_snd_soc_controls(struct snd_soc_codec *codec);
int es_analog_add_snd_soc_dapm_controls(struct snd_soc_codec *codec);
int es_analog_add_snd_soc_route_map(struct snd_soc_codec *codec);

#endif
