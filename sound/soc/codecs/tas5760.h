/*
 * TAS5760 speaker amplifier driver
 *
 * Copyright (C) 2015 Imagination Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _SND_SOC_CODEC_TAS5760_H_
#define _SND_SOC_CODEC_TAS5760_H_

/* Register Address Map */
#define TAS5760_DEVICE_ID		0x00
#define TAS5760_POWER_CTRL		0x01
#define TAS5760_DIGITAL_CTRL	0x02
#define TAS5760_VOL_CTRL		0x03
#define TAS5760_VOL_CTRL_L		0x04
#define TAS5760_VOL_CTRL_R		0x05
#define TAS5760_ANALOG_CTRL		0x06
#define TAS5760_ERR_STATUS		 0x08
#define TAS5760_DIGITAL_CLIP_CTRL2 0x10
#define TAS5760_DIGITAL_CLIP_CTRL1 0x11
#define TAS5760_MAX_REG TAS5760_DIGITAL_CLIP_CTRL1

/* Register Interrupt status */
#define TAS5760_IRQ_STATUS_OTE_SHIFT	0
#define TAS5760_IRQ_STATUS_OTE_MASK	(1 << TAS5760_IRQ_STATUS_OTE_SHIFT)

#define TAS5760_IRQ_STATUS_DCE_SHIFT	1
#define TAS5760_IRQ_STATUS_DCE_MASK	(1 << TAS5760_IRQ_STATUS_DCE_SHIFT)

#define TAS5760_IRQ_STATUS_OCE_SHIFT	2
#define TAS5760_IRQ_STATUS_OCE_MASK	(1 << TAS5760_IRQ_STATUS_OCE_SHIFT)

#define TAS5760_IRQ_STATUS_CLKE_SHIFT	3
#define TAS5760_IRQ_STATUS_CLKE_MASK	(1 << TAS5760_IRQ_STATUS_CLKE_SHIFT)

#define TAS5760_IRQ_STATUS_OCET_SHIFT	4
#define TAS5760_IRQ_STATUS_OCET_MASK	(3 << TAS5760_IRQ_STATUS_OCET_SHIFT)

#define TAS5760_IRQ_STATUS_MASK_LATCH	(TAS5760_IRQ_STATUS_OTE_MASK \
					| TAS5760_IRQ_STATUS_DCE_MASK \
					| TAS5760_IRQ_STATUS_OCE_MASK)

/* Register bit values/masks */
#define TAS5760_SPK_SD_MASK		(1 << 0)
#define TAS5760_SPK_SLEEP_MASK		(1 << 1)

#define TAS5760_PBTL_EN_MASK 0x80
#define TAS5760_PBTL_EN_VAL(val) ((val) << 7)
#define TAS5760_PBTL_CH_VAL(val) ((val) << 1)
#define TAS5760_PBTL_CH_MASK 0x02
#define TAS5760_PBTL_MASK (TAS5760_PBTL_EN_MASK | TAS5760_PBTL_CH_MASK)

#define TAS5760_VOL_FADE_MASK	0x80
#define TAS5760_VOL_FADE_VAL(val) ((val) << 7)

#define TAS5760_MUTE_L_MASK 0x01
#define TAS5760_MUTE_R_MASK 0x02
#define TAS5760_MUTE_MASK (TAS5760_MUTE_L_MASK | TAS5760_MUTE_R_MASK)

#define TAS5760_VOL_CTRL_MASK	0xFF

#define TAS5760_DIGITAL_CLIP_MAX_VAL 0xFFFFF
#define TAS5760_DIGITAL_CLIP_MASK 0xFC
#define TAS5760_DIGITAL_CLIP1_VAL(val) ((val) << 2)
#define TAS5760_DIGITAL_CLIP2_VAL(val) ((val) >> 6)
#define TAS5760_DIGITAL_CLIP3_VAL(val) ((val) >> 12)

#define TAS5760_DIGITAL_BOOST_MASK 0x30
#define TAS5760_DIGITAL_BOOST_VAL(val) ((val) << 4)

#define TAS5760_ANALOG_GAIN_MASK 0x0C
#define TAS5760_ANALOG_GAIN_VAL(val) ((val) << 2)

#define TAS5760_OCE_THRESH_MASK  0x30
#define TAS5760_OCE_THRESH_VAL(val) ((val) << 4)

#define TAS5760_PWM_RATE_MAX 7
#define TAS5760_PWM_RATE_MASK 0x70
#define TAS5760_PWM_RATE_VAL(val) ((val) << 4)

#endif /* _SND_SOC_CODEC_TAS5760_H_ */
