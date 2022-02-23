/*
 * UCSI driver for Cypress CCGx Type-C controller
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
 * Author: Ajay Gupta <ajayg@nvidia.com>
 *
 * Some code borrowed from drivers/usb/typec/ucsi/ucsi_acpi.c
 */
#ifndef __LINUX_CYPD_FLASH_H__
#define __LINUX_CYPD_FLASH_H__

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <asm/unaligned.h>

#define SC_AUTO_UPGRADE_EN

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define CCGX_RAB_DEVICE_MODE			0x0000
#define CCGX_RAB_INTR_REG			0x0006
#define  DEV_INT				BIT(0)
#define  PORT0_INT				BIT(1)
#define  PORT1_INT				BIT(2)
#define  UCSI_READ_INT				BIT(7)
#define CCGX_RAB_JUMP_TO_BOOT			0x0007
#define  TO_BOOT				'J'
#define  TO_ALT_FW				'A'
#define CCGX_RAB_RESET_REQ			0x0008
#define  RESET_SIG				'R'
#define  CMD_RESET_I2C				0x0
#define  CMD_RESET_DEV				0x1
#define CCGX_RAB_ENTER_FLASHING			0x000A
#define  FLASH_ENTER_SIG			'P'
#define CCGX_RAB_VALIDATE_FW			0x000B
#define CCGX_RAB_FLASH_ROW_RW			0x000C
#define  FLASH_SIG				'F'
#define  FLASH_RD_CMD				0x0
#define  FLASH_WR_CMD				0x1
#define  FLASH_FWCT1_WR_CMD			0x2
#define  FLASH_FWCT2_WR_CMD			0x3
#define  FLASH_FWCT_SIG_WR_CMD			0x4
#define CCGX_RAB_READ_ALL_VER			0x0010
#define CCGX_RAB_READ_FW2_VER			0x0020
#define CCGX_RAB_UCSI_CONTROL			0x0039
#define CCGX_RAB_UCSI_CONTROL_START		BIT(0)
#define CCGX_RAB_UCSI_CONTROL_STOP		BIT(1)
#define CCGX_RAB_UCSI_DATA_BLOCK(offset)	(0xf000 | ((offset) & 0xff))
#define REG_FLASH_RW_MEM        0x0200
#define DEV_REG_IDX				CCGX_RAB_DEVICE_MODE
#define CCGX_RAB_PDPORT_ENABLE			0x002C
#define  PDPORT_1		BIT(0)
#define  PDPORT_2		BIT(1)
#define CCGX_RAB_RESPONSE			0x007E
#define  ASYNC_EVENT				BIT(7)

/* CCGx events & async msg codes */
#define RESET_COMPLETE		0x80
#define EVENT_INDEX		RESET_COMPLETE
#define PORT_CONNECT_DET	0x84
#define PORT_DISCONNECT_DET	0x85
#define ROLE_SWAP_COMPELETE	0x87

/* ccg firmware */
#define CYACD_LINE_SIZE         271
#define CCG3_ROW_SIZE           128
#define FW1_METADATA_ROW        0x3FF
#define FW2_METADATA_ROW        0x3FE
#define FW_CFG_TABLE_SIG_SIZE	256



enum enum_flash_mode {
	FW1, /* update primary  using bootloader */
	FW2, /* update secondary using bootloader */
	FW1_FW2, /* update primary and secondary using bootloader */
	FLASH_NOT_NEEDED,	/* update not required */
};

enum enum_active_mode {
	boot_active,
	fw1_active,
	fw2_active,
	fw_invalid,
};

enum enum_valid_id {
	valid_boot,
	valid_fw1,
	valid_fw2,
	valid_not_need,
};

static const char * const ccg_fw_names[] = {
	"ccg_one.cyacd",
	"ccg_two.cyacd"
};

struct ccg_dev_info {
#define CCG_DEVINFO_FWMODE_SHIFT (0)
#define CCG_DEVINFO_FWMODE_MASK (0x3 << CCG_DEVINFO_FWMODE_SHIFT)
#define CCG_DEVINFO_PDPORTS_SHIFT (2)
#define CCG_DEVINFO_PDPORTS_MASK (0x3 << CCG_DEVINFO_PDPORTS_SHIFT)
	u8 mode;
	u8 bl_mode;
	__le16 silicon_id;
	__le16 bl_last_row;
};

struct version_format {
	__le16 build;
	u8 patch;
	u8 ver;
#define CCG_VERSION_PATCH(x) ((x) << 16)
#define CCG_VERSION(x)	((x) << 24)
#define CCG_VERSION_MIN_SHIFT (0)
#define CCG_VERSION_MIN_MASK (0xf << CCG_VERSION_MIN_SHIFT)
#define CCG_VERSION_MAJ_SHIFT (4)
#define CCG_VERSION_MAJ_MASK (0xf << CCG_VERSION_MAJ_SHIFT)
};

/*
 * Firmware version 3.1.10 or earlier, built for NVIDIA has known issue
 * of missing interrupt when a device is connected for runtime resume
 */
#define CCG_FW_BUILD_NVIDIA	(('n' << 8) | 'v')
#define CCG_OLD_FW_VERSION	(CCG_VERSION(0x31) | CCG_VERSION_PATCH(10))

/* Altmode offset for NVIDIA Function Test Board (FTB) */
#define NVIDIA_FTB_DP_OFFSET	(2)
#define NVIDIA_FTB_DBG_OFFSET	(3)

struct version_info {
	struct version_format base;
	struct version_format app;
};

#endif
