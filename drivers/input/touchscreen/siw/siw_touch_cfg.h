/*
 * SiW touch core driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/version.h>

#ifndef __SIW_TOUCH_CFG_H
#define __SIW_TOUCH_CFG_H

#define __SIW_SUPPORT_PINCTRL
#define __SIW_SUPPORT_PWRCTRL

//#define __SIW_SUPPORT_MISC

//#define __SIW_SUPPORT_ASC

#if defined(CONFIG_NET)
#define __SIW_SUPPORT_ABT
#endif

#define __SIW_SUPPORT_PRD

#if defined(CONFIG_TOUCHSCREEN_SIW_LG4895) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4946) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49406) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49407) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49408)
#define __SIW_SUPPORT_WATCH

#if defined(CONFIG_TOUCHSCREEN_SIW_OPT_SINGLE_SCR)
#undef __SIW_SUPPORT_WATCH
#endif
#endif

#define __SIW_SUPPORT_XFER

//#define __SIW_SUPPORT_DEBUG_OPTION

#if defined(CONFIG_ANDROID)
#define __SIW_SUPPORT_WAKE_LOCK
#endif

#if defined(CONFIG_TOUCHSCREEN_SIW_LG4894) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4895) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4946) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_LG4951) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49105) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49406) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49407) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49408)
#define __SIW_SUPPORT_UEVENT
#endif

#if defined(CONFIG_TOUCHSCREEN_SIW_SW49105) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49407) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49408)
#define __SIW_I2C_TYPE_1
#define __SIW_SPI_TYPE_1
#endif

#if defined(CONFIG_TOUCHSCREEN_SIW_SW49407) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW49408)
#define __SIW_FW_TYPE_1
#endif

//#define __SIW_SUPPORT_PM_QOS

#if defined(CONFIG_OF)
#define __SIW_CONFIG_OF
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
#define __SIW_CONFIG_EARLYSUSPEND
#elif defined(CONFIG_FB)
#define __SIW_CONFIG_FB
#endif

//#define __SIW_CONFIG_FASTBOOT

#define __SIW_CONFIG_PROX_ON_SUSPEND
#define __SIW_CONFIG_PROX_ON_RESUME

//#define __SIW_CONFIG_SHOW_TCI_INIT_VAL
//#define __SIW_CONFIG_SHOW_SWIPE_INIT_VAL

#if defined(CONFIG_TOUCHSCREEN_SIW_SW42101)
#define __SIW_BUS_ADDR_16BIT

#ifdef __SIW_SUPPORT_ABT
#undef __SIW_SUPPORT_ABT
#endif

#ifdef __SIW_CONFIG_PROX_ON_SUSPEND
#undef __SIW_CONFIG_PROX_ON_SUSPEND
#endif

#ifdef __SIW_CONFIG_PROX_ON_RESUME
#undef __SIW_CONFIG_PROX_ON_RESUME
#endif
#endif	/* CONFIG_TOUCHSCREEN_SIW_SW42101 */


#if defined(CONFIG_TOUCHSCREEN_SIW_SW1828) ||	\
	defined(CONFIG_TOUCHSCREEN_SIW_SW42101)

#else
#define __SIW_CONFIG_KNOCK
#endif

#if defined(__SIW_SUPPORT_WATCH)
#define __SIW_CONFIG_SWIPE
#endif


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0))
#define __SIW_ATTR_PERMISSION_ALL
#endif

#define __SIW_ATTR_RST_BY_READ

#define SIW_TOUCH_NAME				"siw_touch"
#define SIW_TOUCH_CORE				"siw_touch_core"
#define SIW_TOUCH_INPUT				"siw_touch_input"
#define SIW_TOUCH_EXT_WATCH			"siw_ext_watch"

#define MAX_FINGER					10
#define MAX_LPWG_CODE				128

enum _SIW_CHIP_TYPE {
	CHIP_NONE		= 0x0000,
	//
	CHIP_LG4894		= 0x0001,
	CHIP_LG4895		= 0x0002,
	CHIP_LG4946		= 0x0003,
	CHIP_LG4951		= 0x0004,
	//
	CHIP_SW1828		= 0x0080,
	//
	CHIP_SW49105	= 0x0105,
	//
	CHIP_SW49406	= 0x0406,
	CHIP_SW49407	= 0x0407,
	CHIP_SW49408	= 0x0408,
	CHIP_SW49409	= 0x0409,
	//
	CHIP_SW42101	= 0x0211,
};

//#define __SIW_TEST_IRQ_OFF

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 0))
#define mod_delayed_work	queue_delayed_work
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#define subsys_system_register(_subsys, _group)	bus_register(_subsys)
#endif

#endif	/* __SIW_TOUCH_CFG_H */

