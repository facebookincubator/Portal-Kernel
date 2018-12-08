/*
 * touch_lg4894.c - SiW touch driver glue for LG4894
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/firmware.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_bus.h"

#define CHIP_ID						"4894"
#define CHIP_DEVICE_NAME			"LG4894"
#define CHIP_COMPATIBLE_NAME		"siw,lg4894"
#define CHIP_DEVICE_DESC			"SiW Touch LG4894 Driver"

#define CHIP_TYPE					CHIP_LG4894

#define CHIP_MODE_ALLOWED			(0 |	\
									LCD_MODE_BIT_U0 |	\
									LCD_MODE_BIT_U3 |	\
									LCD_MODE_BIT_STOP |	\
									0)

#define CHIP_FW_SIZE				(69<<10)

#if defined(CONFIG_MACH_ODROIDXU3)
#define CHIP_FLAG_SKIP_ESD_EVENT	TOUCH_SKIP_ESD_EVENT
#else
#define CHIP_FLAG_SKIP_ESD_EVENT	0
#endif

#define CHIP_FLAGS					(0 |	\
									CHIP_FLAG_SKIP_ESD_EVENT |	\
									0)

#define CHIP_IRQFLAGS				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)


#define CHIP_INPUT_ID_BUSTYPE		BUS_I2C
#define CHIP_INPUT_ID_VENDOR		0xABCD
#define CHIP_INPUT_ID_PRODUCT		0x9876
#define CHIP_INPUT_ID_VERSION		0x1234

#define __CHIP_QUIRK_ADD			0

#define CHIP_QUIRKS					(0 |	\
									CHIP_QUIRK_NOT_SUPPORT_ASC |	\
									CHIP_QUIRK_NOT_SUPPORT_WATCH |	\
									CHIP_QUIRK_NOT_SUPPORT_IME |	\
									__CHIP_QUIRK_ADD |	\
									0)

#define CHIP_BUS_TYPE				BUS_IF_I2C
#define	CHIP_BUF_SIZE				0
#define CHIP_SPI_MODE				-1
#define CHIP_BPW					-1
#define CHIP_MAX_FREQ				-1
#define CHIP_TX_HDR_SZ				I2C_BUS_TX_HDR_SZ
#define CHIP_RX_HDR_SZ				I2C_BUS_RX_HDR_SZ
#define CHIP_TX_DUMMY_SZ			I2C_BUS_TX_DUMMY_SZ
#define CHIP_RX_DUMMY_SZ			I2C_BUS_RX_DUMMY_SZ

#define CHIP_SENSELESS_MARGIN		(0x21)	//33 pixel : 3mm

#if defined(__SIW_CONFIG_OF)
/*
 * of_device_is_compatible(dev->of_node, CHIP_COMPATIBLE_NAME)
 */
static const struct of_device_id chip_match_ids[] = {
	{ .compatible = CHIP_COMPATIBLE_NAME },
	{ },
};
#else
enum CHIP_CAPABILITY {
	CHIP_MAX_X			= 720,
	CHIP_MAX_Y			= 1280,
	CHIP_MAX_PRESSURE	= 255,
	CHIP_MAX_WIDTH		= 15,
	CHIP_MAX_ORI		= 1,
	CHIP_MAX_ID			= 10,
	/* */
	CHIP_HW_RST_DELAY	= 210,
	CHIP_SW_RST_DELAY	= 90,
};

#define CHIP_PIN_RESET			0
#define CHIP_PIN_IRQ			0
#define CHIP_PIN_MAKER			-1
#define CHIP_PIN_VDD			-1
#define CHIP_PIN_VIO			-1

#if (CHIP_PIN_RESET == 0) || (CHIP_PIN_IRQ == 0)
	#error Assign external pin & flag first!!!
#endif
#endif	/* __SIW_CONFIG_OF */

/* use eg. cname=arc1 to change name */
static char chip_name[32] = CHIP_DEVICE_NAME;
module_param_string(cname, chip_name, sizeof(chip_name), 0);

/* use eg. dname=arc1 to change name */
static char chip_drv_name[32] = SIW_TOUCH_NAME;
module_param_string(dname, chip_drv_name, sizeof(chip_drv_name), 0);

/* use eg. iname=arc1 to change input name */
static char chip_idrv_name[32] = SIW_TOUCH_INPUT;
module_param_string(iname, chip_idrv_name, sizeof(chip_idrv_name), 0);

static const struct siw_touch_pdata chip_pdata = {
	/* Configuration */
	.chip_id			= CHIP_ID,
	.chip_name			= chip_name,
	.drv_name			= chip_drv_name,
	.idrv_name			= chip_idrv_name,
	.owner				= THIS_MODULE,
	.chip_type			= CHIP_TYPE,
	.mode_allowed		= CHIP_MODE_ALLOWED,
	.fw_size			= CHIP_FW_SIZE,
	.flags				= CHIP_FLAGS,	/* Caution : MSB(bit31) unavailable */
	.irqflags			= CHIP_IRQFLAGS,
	.quirks				= CHIP_QUIRKS,
	/* */
	.bus_info			= {
		.bus_type			= CHIP_BUS_TYPE,
		.buf_size			= CHIP_BUF_SIZE,
		.spi_mode			= CHIP_SPI_MODE,
		.bits_per_word		= CHIP_BPW,
		.max_freq			= CHIP_MAX_FREQ,
		.bus_tx_hdr_size	= CHIP_TX_HDR_SZ,
		.bus_rx_hdr_size	= CHIP_RX_HDR_SZ,
		.bus_tx_dummy_size	= CHIP_TX_DUMMY_SZ,
		.bus_rx_dummy_size	= CHIP_RX_DUMMY_SZ,
	},
#if defined(__SIW_CONFIG_OF)
	.of_match_table 	= of_match_ptr(chip_match_ids),
#else
	.pins				= {
		.reset_pin		= CHIP_PIN_RESET,
		.reset_pin_pol	= OF_GPIO_ACTIVE_LOW,
		.irq_pin		= CHIP_PIN_IRQ,
		.maker_id_pin	= CHIP_PIN_MAKER,
		.vdd_pin		= CHIP_PIN_VDD,
		.vio_pin		= CHIP_PIN_VIO,
	},
	.caps				= {
		.max_x			= CHIP_MAX_X,
		.max_y			= CHIP_MAX_Y,
		.max_pressure	= CHIP_MAX_PRESSURE,
		.max_width		= CHIP_MAX_WIDTH,
		.max_orientation = CHIP_MAX_ORI,
		.max_id			= CHIP_MAX_ID,
		.hw_reset_delay	= CHIP_HW_RST_DELAY,
		.sw_reset_delay	= CHIP_SW_RST_DELAY,
	},
#endif
	/* Input Device ID */
	.i_id				= {
		.bustype		= CHIP_INPUT_ID_BUSTYPE,
		.vendor 		= CHIP_INPUT_ID_VENDOR,
		.product 		= CHIP_INPUT_ID_PRODUCT,
		.version 		= CHIP_INPUT_ID_VERSION,
	},
	/* */
	//See 'siw_hal_get_default_ops' [siw_touch_hal.c]
	.ops				= NULL,
	/* */
	//See 'siw_hal_get_tci_info' [siw_touch_hal.c]
	.tci_info			= NULL,
	.tci_reset_area		= NULL,
	.tci_qcover_open	= NULL,
	.tci_qcover_close	= NULL,
	//See 'siw_hal_get_swipe_info' [siw_touch_hal.c]
	.swipe_ctrl			= NULL,
	//See 'store_ext_watch_config_font_position' [siw_touch_hal_watch.c]
	.watch_win			= NULL,
	//See 'siw_setup_operations' [siw_touch.c]
	.reg_quirks			= NULL,
	.senseless_margin	= CHIP_SENSELESS_MARGIN,
};

static struct siw_touch_chip_data chip_data = {
	.pdata = &chip_pdata,
	.bus_drv = NULL,
};

siw_chip_module_init(CHIP_DEVICE_NAME,
				chip_data,
				CHIP_DEVICE_DESC,
				"kimhh@siliconworks.co.kr");


__siw_setup_str("siw_chip_name=", siw_setup_chip_name, chip_name);
__siw_setup_str("siw_drv_name=", siw_setup_drv_name, chip_drv_name);
__siw_setup_str("siw_idrv_name=", siw_setup_idrv_name, chip_idrv_name);



