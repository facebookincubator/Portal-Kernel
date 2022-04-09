/*
 * esxxx-codec.h  --  Audience dummy codec
 *
 * Copyright 2011 Audience, Inc.
 *
 * Author: Sheetal Garg <sheetal.garg@knowles.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _ESXXX_CODEC_H
#define _ESXXX_CODEC_H

#define ES_DHWPT_CMD                    0x9052

#define ES705_PCM_PORT_A                0xA
#define ES705_PCM_PORT_B                0xB
#define ES705_PCM_PORT_C                0xC

#define PORT_A_TO_D     0x01CC
#define PORT_B_TO_D     0x01DD
#define PORT_C_TO_D     0x01EE

#define ES_API_WORD(upper, lower) ((upper << 16) | lower)

extern struct snd_soc_codec_driver soc_codec_dev_esxxx;

#endif
