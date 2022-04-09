/*
 * iaxxx-core.c -- IAxxx Multi-Function Device driver
 *
 * Copyright 2016 Knowles Corporation
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


/*
 * Driver To Do Items
 *
 * - Add mutex for API calls: Memory management, Event Management, etc.
 * - Define the volatile and read-only registers
 * - Use two regmaps; Core driver owns SRB, cell drivers have ARB access
 *	any SRB access for cell drivers needs to go througn an API call
 * - Enable regmap caching
 * - Verify that virtual addresses can only be accessed when mapped
 * - pull in the event manager from Athens (document on wiki)
 */

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/fb_event.h>
#include <uapi/linux/fb_event.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-af.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debuglog.h>
#include <linux/mfd/adnc/iaxxx-register-defs-stream-header.h>
#include "iaxxx.h"
#include "iaxxx-dbgfs.h"
#include "iaxxx-tunnel.h"
#include "iaxxx-debug.h"
#include "iaxxx-cdev.h"
#include <linux/circ_buf.h>
#define CREATE_TRACE_POINTS
#include <trace/events/fb_audio.h>

#define IAXXX_RESET_RETRIES		5		/* retry attempts */
#define IAXXX_RESET_HOLD_TIME		(20*1000)	/* 20 ms */
#define IAXXX_RESET_READY_DELAY		(20*1000)	/* 20 ms */
#define IAXXX_RESET_RANGE_INTERVAL	100		/* 100 us */
#define IAXXX_CLK_READY_DELAY		(5*1000)	/* 5 ms */
#define IAXXX_CLK_RANGE_INTERVAL	(100)		/* 100 us */

/* 2 retries if failed */
#define IAXXX_FW_RETRY_COUNT		2
#define IAXXX_FW_DOWNLOAD_TIMEOUT	10000		/* 10 secs */
#define IAXXX_VER_STR_SIZE		60
#define IAXXX_BYTES_IN_A_WORD		4
#define IAXXX_MAX_PLUGIN		6
#define IAXXX_MAX_PACKAGE		3

/* VDD IO 1.8V */
#define IAXXX_VDD_IO_MIN		1800000
#define IAXXX_VDD_IO_MAX		1800000

/* VDD Core 1.2V */
#define IAXXX_VDD_CORE_MIN		1200000
#define IAXXX_VDD_CORE_MAX		1200000

#define IAXXX_BYPASS_ON_VAL 0x00C199BB
#define IAXXX_BYPASS_OFF_VAL 0x0C099BB
#define IAXXX_PWR_DWN_VAL 0x01C00050
#define IAXXX_PWR_ON_VAL 0x845
#define IAXXX_PWR_STATE_RETRY	0x5

#define IAXXX_AO_MEM_ELEC_CTRL_NORMAL_READ	0x2
#define IAXXX_AO_MEM_ELEC_CTRL_VOLTAGE_RETENTION_1000	0x100

#define iaxxx_ptr2priv(ptr, item) container_of(ptr, struct iaxxx_priv, item)

#define iaxxx_work(priv, work) queue_kthread_work(&priv->worker, &priv->work)

#define iaxxx_work_flush(priv, work)	flush_kthread_work(&priv->work)

struct iaxxx_port_clk_settings clk;
static const u32 iaxxx_port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_PORTD_CLK_ADDR,
	IAXXX_IO_CTRL_PORTE_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 iaxxx_port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTD_FS_ADDR,
	IAXXX_IO_CTRL_PORTE_FS_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 iaxxx_port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
	IAXXX_IO_CTRL_PORTE_DI_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 iaxxx_port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_PORTD_DO_ADDR,
	IAXXX_IO_CTRL_PORTE_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

/* Firmware download error code */
enum {
	E_IAXXX_REGMAP_ERROR = -1,
	E_IAXXX_BOOTUP_ERROR = -2
};

/*===========================================================================
 * MFD Driver
 *===========================================================================
 */

static struct mfd_cell iaxxx_devices[] = {
	{
		.name = "iaxxx-codec",
	},
	{
		.name = "iaxxx-odsp-celldrv",
	},
	{
		.name = "iaxxx-tunnel-celldrv",
	},
};

static char *iaxxx_crash_err2str(int error)
{
	switch (error) {
	case IAXXX_FW_CRASH_EVENT:
		return "crash event";
	case IAXXX_FW_CRASH_ON_FLUSH_EVENTS:
		return "crash event when flush events";
	case IAXXX_FW_CRASH_REG_MAP_WAIT_CLEAR:
		return "until wait clear";
	case IAXXX_FW_CRASH_UPDATE_BLOCK_REQ:
		return "until update block";
	case IAXXX_FW_CRASH_TUNNEL_WRONG_BUFF:
		return "tunnel wrong buff params";
	}

	return "unknown";
}

static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type);

static int iaxxx_cell_force_resume(struct device *dev, void *data)
{
	pm_runtime_force_resume(dev);
	return 0;
}

static int iaxxx_cell_force_suspend(struct device *dev, void *data)
{
	pm_runtime_force_suspend(dev);
	return 0;
}

static int iaxxx_pdm_clk_stop(struct iaxxx_priv *priv, int port)
{
	int ret;
	int mask;
	int status;

	/* Disable Clock output */
	mask = (IAXXX_AO_CLK_CFG_PORTA_CLK_OE_MASK << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR, mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	/* Stop the clock generation */
	mask = (1 << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
					mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	ret = regmap_write(priv->regmap,
		IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_TRIGGER_MASK);
	if (ret) {
		dev_err(priv->dev, "write failed %s()\n", __func__);
		goto clk_stop_err;
	}

	/* Power disable for the PCM port */
	mask = (1 << port);
	ret = regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			mask, 0);
	if (ret) {
		dev_err(priv->dev, "update failed %s()\n", __func__);
		goto clk_stop_err;
	}
	ret = iaxxx_send_update_block_request(priv->dev, &status,
						IAXXX_BLOCK_0);
	if (ret) {
		dev_err(priv->dev, "Update blk failed %s()\n", __func__);
		goto clk_stop_err;
	}
	return 0;
clk_stop_err:
	dev_err(priv->dev, "%s failed\n", __func__);
	return ret;
}

static int iaxxx_pdm_clk_start(struct iaxxx_priv *iaxxx, int port)
{
	u32 period, div_val, nr_val, fs_sync_active;
	u32 port_bits_per_frame;
	u32 clk_ctrl_val;
	u32 ao_clk_cfg_val;
	u32 ao_clk_cfg_mask, status;
	u32 ret;

	/* Use the values set in the codec while enabling PDM clock */
	port_bits_per_frame = clk.port_bits_per_frame;
	nr_val = clk.nr_val;
	div_val = clk.div_val;
	period = clk.period;

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

	ret = regmap_update_bits(iaxxx->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
		IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x1 << port));
	if (ret)
		goto start_clk_err;
	ret = iaxxx_send_update_block_request(iaxxx->dev, &status,
			IAXXX_BLOCK_0);
	if (ret)
		goto start_clk_err;
	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
			1 << (port), 0 << port);
	if (ret)
		goto start_clk_err;
	/* I2S Trigger - Disable I2S */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL, 1);
	if (ret)
		goto start_clk_err;
	/*Bit 0 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S2_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 2*/
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	if (ret)
		goto start_clk_err;
	/* Bit 3 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Bit 11:4 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
		((port_bits_per_frame) <<
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));
	if (ret)
		goto start_clk_err;
	/* For PDM FS is assumed 0 */
	fs_sync_active = (0 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS);
	/* Bit 19:12 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK, fs_sync_active);
	if (ret)
		goto start_clk_err;
	/* Bit 20 */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);
	if (ret)
		goto start_clk_err;
	/* FS Align */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_FS_ALIGN_ADDR(port),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);
	if (ret)
		goto start_clk_err;
	/* disable hl divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Set HL value */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	if (ret)
		goto start_clk_err;
	/* enable hl divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_ENABLE);
	if (ret)
		goto start_clk_err;
	/* disable NR divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_DISABLE);
	if (ret)
		goto start_clk_err;
	/* Set NR value */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_WMASK_VAL, nr_val);
	if (ret)
		goto start_clk_err;
	/* enable NR divider */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_ENABLE);
	if (ret)
		goto start_clk_err;
	/* Clk control */
	clk_ctrl_val = (((period/2 - 1) <<
		IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS)|((period - 1)
		<< IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL, clk_ctrl_val);
	if (ret)
		goto start_clk_err;
	/* AO CLK Config */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_AO_CLK_CFG_ADDR,
		ao_clk_cfg_mask, ao_clk_cfg_val);
	if (ret)
		goto start_clk_err;
	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port), 1 << port);
	if (ret)
		goto start_clk_err;
	/* I2S Trigger - Disable I2S */
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
		IAXXX_SRB_PDMI_PORT_PWR_EN_MASK_VAL, 0xFF);
	if (ret)
		goto start_clk_err;
	ret = iaxxx_send_update_block_request(iaxxx->dev, &status,
			IAXXX_BLOCK_0);
	if (ret)
		goto start_clk_err;
	/* Configure IO CTRL ports */
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK,
		IAXXX_IO_CTRL_CLK_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK,
		 IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_fs_addr[port],
		IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK,
		IAXXX_IO_CTRL_FS_PDM_MASTER);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK, IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_di_addr[port],
		IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK,
		IAXXX_IO_CTRL_DI_PDM);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	ret = regmap_update_bits(iaxxx->regmap, iaxxx_port_do_addr[port],
		IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, IAXXX_IO_CTRL_DO);
	if (ret)
		goto start_clk_err;
	/* CIC filter config */
	ret = regmap_update_bits(iaxxx->regmap,
		IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, 1);
	if (ret)
		goto start_clk_err;
	return 0;
start_clk_err:
	pr_err("%s() failed\n", __func__);
	return ret;
}

static int iaxxx_mic_enable(struct iaxxx_priv *priv, bool enable)
{
	int ret = 0;
	int status;

	if (enable) {
		ret = iaxxx_pdm_clk_start(priv, PDM_PORTC);
		if (ret) {
			dev_err(priv->dev, "iaxxx_pdm_clk_start failed.\n");
			return ret;
		}

		ret = regmap_update_bits(priv->regmap,
				IAXXX_STR_HDR_STR_EN_ADDR,
				IAXXX_STR_HDR_STR_EN_4_REG_MASK,
				0x1 << IAXXX_STR_HDR_STR_EN_4_REG_POS);
		if (ret) {
			dev_err(priv->dev,
				"Update bits failed %s()\n", __func__);
			return ret;
		}

		ret = iaxxx_send_update_block_request(priv->dev, &status,
						IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev,
				"Update blk failed %s()\n", __func__);
			return ret;
		}
	} else {
		ret = regmap_update_bits(priv->regmap,
				IAXXX_STR_HDR_STR_EN_ADDR,
				IAXXX_STR_HDR_STR_EN_4_REG_MASK,
				0x0 << IAXXX_STR_HDR_STR_EN_4_REG_POS);
		if (ret) {
			dev_err(priv->dev,
				"Update bits failed %s()\n", __func__);
			return ret;
		}

		ret = iaxxx_send_update_block_request(priv->dev, &status,
						IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev,
				"Update blk failed %s()\n", __func__);
			return ret;
		}

		ret = iaxxx_pdm_clk_stop(priv, PDM_PORTC);
		if (ret) {
			dev_err(priv->dev, "iaxxx_pdm_clk_stop failed.\n");
			return ret;
		}
	}

	return ret;
}

static int iaxxx_do_suspend(struct iaxxx_priv *priv,
	bool is_fw_crash, bool is_privacy)
{
	struct device *dev = priv->dev;
	unsigned long *flags = &priv->flags;
	bool do_recovery = is_fw_crash && !is_privacy;
	bool suspended = false;

	/* Send broadcast event to all device children's for crash */
	if (do_recovery)
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_CRASH, NULL);

	if (!test_and_set_bit(IAXXX_FLG_PM_SUSPEND, flags)) {
		if (is_privacy && !test_and_set_bit(IAXXX_FLG_DMIC_OFF, flags))
			iaxxx_mic_enable(priv, false);

		device_for_each_child(dev, priv, iaxxx_cell_force_suspend);
		pm_runtime_force_suspend(dev);
		suspended = true;
	}

	/* Start FW recovery if pending and is allowed, otherwise remember */
	if (do_recovery)
		iaxxx_work(priv, fw_crash_work);

	/* Send suspend event to user space */
	if (suspended)
		iaxxx_send_uevent(priv, "ACTION=IAXXX_SUSPEND_EVENT");

	return 0;
}

static int iaxxx_do_resume(struct iaxxx_priv *priv,
	bool is_fw_crash, bool is_privacy)
{
	struct device *dev = priv->dev;
	unsigned long *flags = &priv->flags;
	bool resumed = false;

	if (test_and_clear_bit(IAXXX_FLG_PM_SUSPEND, flags)) {
		pm_runtime_force_resume(dev);
		device_for_each_child(dev, priv, iaxxx_cell_force_resume);
		resumed = true;
	}

	/*
	 * Following notifications must be send only when core and
	 * all its children's are successfully resumed.
	 */
	if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_STARTUP, flags)) {
		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_STARTUP, NULL);

		/* Send firmware startup event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_STARTUP_EVENT");
	} else if (test_and_clear_bit(IAXXX_FLG_RESUME_BY_RECOVERY, flags)) {
		union fb_iaxxx_fw_crash fb_event = {
			.reasons = priv->fw_crash_reasons,
			.try_count = priv->recovery_try_count,
			.status = IAXXX_FW_RECOVERY_SUCCESS,
		};

		iaxxx_fw_notifier_call(priv->dev, IAXXX_EV_RECOVERY, NULL);

		/* Send recovery event to HAL */
		iaxxx_send_uevent(priv, "ACTION=IAXXX_RECOVERY_EVENT");

		/* Send metrics to fb analytics */
		if (priv->fw_crash_reasons != IAXXX_FW_CRASH_EVENT_DEBUG)
			fb_event_log_int(FBE_IAXXX_CRASH, fb_event.value);
	}

	/* Send suspend  event to user space */
	if (resumed) {
		if (!is_privacy &&
			test_and_clear_bit(IAXXX_FLG_DMIC_OFF, flags))
			iaxxx_mic_enable(priv, true);

		iaxxx_send_uevent(priv, "ACTION=IAXXX_RESUME_EVENT");
	}

	return 0;
}

/**
 * iaxxx_gpio_free - free a gpio for a managed device
 * @dev: device to free the gpio for
 * @gpio: GPIO to free
 */
static inline void iaxxx_gpio_free(struct device *dev, unsigned int gpio)
{
	devm_gpio_free(dev, gpio);
	dev_dbg(dev, "%s(): %d\n", __func__, gpio);
}

/**
 * get_named_gpio - Gets a GPIO property from a device tree node
 */
static inline int get_named_gpio(struct device *dev, const char *name)
{
	int rc;

	rc = of_get_named_gpio(dev->of_node, name, 0);
	if (rc < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			name, dev->of_node->full_name, rc);
		return rc;
	}
	dev_dbg(dev, "%s: %s %d\n", __func__, name, rc);
	return rc;
}

/**
 * iaxxx_populate_dt_gpios - populates GPIO data from device tree
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_populate_dt_gpios(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	rc = get_named_gpio(dev, "adnc,reset-gpio");
	if (rc < 0) {
		priv->reset_gpio = -EINVAL;
		dev_err(dev, "Failed to read reset-gpio, rc = %d\n", rc);
		return rc;
	}
	priv->reset_gpio = rc;

	rc = get_named_gpio(dev, "adnc,event-gpio");
	if (rc < 0) {
		priv->event_gpio = -EINVAL;
		dev_err(dev, "Failed to read event-gpio gpio, rc = %d\n", rc);
		return rc;
	}
	priv->event_gpio = rc;

	return 0;
}

static int iaxxx_populate_dt_reg(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	priv->vdd_io = devm_regulator_get(priv->dev, "vdd-io");
	if (IS_ERR_OR_NULL(priv->vdd_io)) {
		pr_debug("unable to get vdd-io rc = %d\n",
			PTR_RET(priv->vdd_io));
		return PTR_RET(priv->vdd_io);
	}

	priv->vdd_core = devm_regulator_get(priv->dev, "vdd-core");
	if (IS_ERR_OR_NULL(priv->vdd_core)) {
		pr_debug("unable to get vdd-core rc = %d\n",
			PTR_RET(priv->vdd_core));
		return PTR_RET(priv->vdd_io);
	}

	return 0;
}

/**
 * iaxxx_populate_dt_pdata - populate platform data from device tree
 */
static int iaxxx_populate_dt_pdata(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	if (!dev->of_node) {
		dev_err(dev, "Missing device tree information\n");
		return -ENODEV;
	}

	rc = iaxxx_populate_dt_gpios(priv);
	if (rc) {
		dev_err(dev, "Failed to read GPIO data, rc = %d\n", rc);
		return rc;
	}

	rc = iaxxx_populate_dt_reg(priv);
	if (rc) {
		dev_err(dev, "Failed to read regulator data, rc = %d\n", rc);
		return rc;
	}

	return 0;
}

/**
 * iaxxx_gpio_init(): Requests the GPIO for the device
 */
static int iaxxx_gpio_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* GPIO: event (input used for interrupts) */
	/* TODD: interrupt is active HIGH so the GPIO needs pull-down */
	rc = devm_gpio_request_one(dev, priv->event_gpio,
						GPIOF_DIR_IN, "EVENT");
	if (rc < 0)
		goto err_missing_event_gpio;

	/* GPIO: reset (output used for chip reset) */
	rc = devm_gpio_request_one(dev, priv->reset_gpio,
						GPIOF_OUT_INIT_LOW, "RESET");
	if (rc < 0)
		goto err_missing_reset_gpio;

	return 0;

err_missing_reset_gpio:
	iaxxx_gpio_free(dev, priv->event_gpio);
err_missing_event_gpio:
	return rc;
}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
static void dump_to_log(struct device *dev,
		struct iaxxx_reg_dump_priv *reg_dump,
		struct iaxxx_register_log *log)
{
	spin_lock(&reg_dump->ring_lock);
	/* Check if Log buffer has space, if not increment the tail index to
	 * get buffer to overwrite oldest data
	 */
	if (!(CIRC_SPACE(reg_dump->head, reg_dump->tail, IAXXX_BUF_MAX_LEN))) {
		dev_dbg_ratelimited(dev, "%s() Register log buffer is full\n",
				__func__);
		reg_dump->tail++;
		reg_dump->tail %= IAXXX_BUF_MAX_LEN;
	}
	reg_dump->log[reg_dump->head] = *log;
	reg_dump->head++;
	/* Align with the Ring Buffer boundary */
	reg_dump->head %= IAXXX_BUF_MAX_LEN;
	spin_unlock(&reg_dump->ring_lock);
}

void register_transac_log(struct device *dev, uint32_t reg, uint32_t val,
		bool op)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_reg_dump_priv *reg_dump = priv->reg_dump;
	struct iaxxx_register_log log;

	if (!priv->reg_dump)
		return;
	if (!pm_runtime_active(dev))
		return;
	log.val = val;
	log.addr = reg;
	log.op = op;
	get_monotonic_boottime(&log.timestamp);
	/* Add the log into circular buffer */
	dump_to_log(dev, reg_dump, &log);
}
#endif

/**
 * iaxxx_reset(): Reset IAxxx device
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s(): reset iaxxx\n", __func__);

	if (!gpio_is_valid(priv->reset_gpio)) {
		dev_err(dev, "%s(): reset_gpio(%d) is an invalid gpio.\n",
						__func__, priv->reset_gpio);
		return -EIO;
	}

	if (gpio_get_value(priv->reset_gpio)) {
		gpio_set_value(priv->reset_gpio, 0);
		usleep_range(IAXXX_RESET_HOLD_TIME,
			IAXXX_RESET_HOLD_TIME + IAXXX_RESET_RANGE_INTERVAL);
	}

	gpio_set_value(priv->reset_gpio, 1);
	usleep_range(IAXXX_RESET_READY_DELAY, IAXXX_RESET_READY_DELAY +
		     IAXXX_RESET_RANGE_INTERVAL);

	return 0;
}

/**
 * iaxxx_reset_to_sbl - Boots the chip hardware
 *
 * Reset iaxxx device to sbl mode through reset_gpio
 */
static int iaxxx_reset_to_sbl(struct iaxxx_priv *priv)
{
	int rc = 0;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* Reset the chip */
	rc = iaxxx_reset(priv);
	if (rc)
		dev_err(dev, "%s: device reset failed\n", __func__);

	return rc;
}

/**
 * iaxxx_event_isr - Interrupt / Event handler
 *
 * @irq	 : interrupt number
 * @data : iaxxx private data
 */
static irqreturn_t iaxxx_event_isr(int irq, void *data)
{
	int rc;
	uint32_t count;
	bool handled = false;
	int mode;
	uint32_t status;
	struct iaxxx_priv *priv = (struct iaxxx_priv *)data;
	struct device *dev = priv->dev;

	dev_dbg_ratelimited(priv->dev, "%s: IRQ %d\n", __func__, irq);

	if (!pm_runtime_active(priv->dev))
		return IRQ_HANDLED;

	/* Any events in the event queue? */
	rc = regmap_read(priv->regmap, IAXXX_SRB_EVT_COUNT_ADDR, &count);
	if (rc)
		dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n", rc);

	if (count > 0 && priv->event_workq && !rc) {
		dev_dbg_ratelimited(priv->dev, "%s: %d event(s) avail\n",
				__func__, count);
		queue_work(priv->event_workq, &priv->event_work_struct);
		handled = true;
	} else {
		/* Read SYSTEM_STATUS to ensure that device is in
		 * Application Mode
		 */
		rc = regmap_read(priv->regmap,
				IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (rc)
			dev_err(priv->dev,
				"Failed to read SYSTEM_STATUS, rc = %d\n", rc);

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		if (mode != SYSTEM_STATUS_MODE_APPS) {
			dev_err(priv->dev,
				"Not in app mode CM4 might crashed: mode=%d\n",
				mode);
			priv->cm4_crashed = true;
			queue_work(priv->event_workq,
				&priv->event_work_struct);
			return IRQ_HANDLED;
		}
	}
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	complete_all(&priv->cmem_done);
#endif
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/**
 * iaxxx_irq_init - Initialize interrupt handling
 *
 * @priv : iaxxx private data
 */
static int iaxxx_irq_init(struct iaxxx_priv *priv)
{
	int rc;

	if (!gpio_is_valid(priv->event_gpio))
		return -ENXIO;

	rc = request_threaded_irq(gpio_to_irq(priv->event_gpio), NULL,
		iaxxx_event_isr, IRQF_TRIGGER_RISING | IRQF_ONESHOT |
		IRQF_NO_SUSPEND, "iaxxx-event-irq", priv);
	if (rc)
		return rc;

	rc = enable_irq_wake(gpio_to_irq(priv->event_gpio));
	if (rc < 0)
		pr_err("%s: enable_irq_wake() failed on %d\n", __func__, rc);

	disable_irq(gpio_to_irq(priv->event_gpio));

	return 0;
}

/**
 * iaxxx_irq_exit - Frees the event IRQ
 *
 * @priv	: iaxxx private data
 */
static void iaxxx_irq_exit(struct iaxxx_priv *priv)
{
	if (gpio_is_valid(priv->event_gpio))
		free_irq(gpio_to_irq(priv->event_gpio), priv);
}

/**
 * iaxxx_reg_enable - Enable regulators
 *
 * @priv : iaxxx private data
 */
static int iaxxx_reg_enable(struct iaxxx_priv *priv)
{
	int rc;

	if (regulator_can_change_voltage(priv->vdd_io)) {
		rc = regulator_set_voltage(priv->vdd_io,
				IAXXX_VDD_IO_MIN,
				IAXXX_VDD_IO_MAX);
		if (rc) {
			pr_debug("failed to set io voltage rc = %d\n", rc);
			goto exit_vdd_io_set;
		}
	}

	if (regulator_can_change_voltage(priv->vdd_core)) {
		rc = regulator_set_voltage(priv->vdd_core,
				IAXXX_VDD_CORE_MIN,
				IAXXX_VDD_CORE_MAX);
		if (rc) {
			pr_debug("failed to set core voltage rc = %d\n", rc);
			goto exit_vdd_core_set;
		}
	}

	rc = regulator_enable(priv->vdd_io);
	if (rc) {
		pr_debug("failed to enable vdd-io rc = %d\n", rc);
		goto exit_vdd_io_en;
	}

	rc = regulator_enable(priv->vdd_core);
	if (rc) {
		pr_debug("failed to enable vdd-core rc = %d\n", rc);
		goto exit_vdd_core_en;
	}

	return rc;

exit_vdd_core_en:
	regulator_disable(priv->vdd_io);

exit_vdd_io_en:
exit_vdd_core_set:
exit_vdd_io_set:
	return rc;
}

/**
 * iaxxx_reg_disable - Disable regulators
 *
 * @priv : iaxxx private data
 */
static int __maybe_unused iaxxx_reg_disable(struct iaxxx_priv *priv)
{
	int rc;

	rc = regulator_disable(priv->vdd_io);
	if (rc)
		pr_debug("failed to disable vdd-io rc = %d\n", rc);

	rc = regulator_disable(priv->vdd_core);
	if (rc)
		pr_debug("failed to enable vdd-core rc = %d\n", rc);

	return 0;
}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
static int iaxxx_regdump_init(struct iaxxx_priv *priv)
{
	dev_dbg(priv->dev, "%s()", __func__);

	priv->dump_log = true;
	priv->reg_dump = kzalloc(sizeof(struct iaxxx_reg_dump_priv),
			GFP_KERNEL);
	if (!priv->reg_dump)
		return -ENOMEM;

	priv->reg_dump->log = kzalloc(sizeof(struct iaxxx_register_log)
			* IAXXX_BUF_MAX_LEN, GFP_KERNEL);
	if (!priv->reg_dump->log) {
		kfree(priv->reg_dump);
		return -ENOMEM;
	}

	spin_lock_init(&priv->reg_dump->ring_lock);
	return 0;
}

void iaxxx_regdump_exit(struct iaxxx_priv *priv)
{
	kfree(priv->reg_dump->log);
	kfree(priv->reg_dump);
}
#endif

/**
 * iaxxx_send_uevent - Send uevent KOBJ_CHANGE
 *
 * @priv	: iaxxx private data
 * @type	  Type of event
 *
 * Returns 0 on success, <0 on failure.
 */
static int iaxxx_send_uevent(struct iaxxx_priv *priv, char *type)
{
	char *event[2] = {type, NULL};

	/* Send recovery event to HAL */
	return kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, event);
}

/**
 * iaxxx_do_fw_update - reset and sync target and do firmware update
 *
 * @priv	: iaxxx private data
 * Return
 *       0                    Success
 *       E_IAXXX_REGMAP_ERROR error accessing regmap
 *       E_IAXXX_BOOTUP_ERROR target bootup failure
 *
 */
static int iaxxx_do_fw_update(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t reg, mode, status;
	struct device *dev = priv->dev;

	if (priv->reset_cb)
		priv->reset_cb(dev);

	/* Verify that the device is in bootloader mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &status);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}

	mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
	dev_dbg(dev, "System Status: 0x%.08X mode: 0x%.08X\n", status, mode);
	/*TODO FW is not yet updating the system status*/
	/*WARN_ON(mode != SYSTEM_STATUS_MODE_SBL);*/

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "Device ID: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc) {
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "ROM Version: 0x%.08X\n", reg);

	/* Electrical control for Memory */
	rc = regmap_write(priv->regmap, IAXXX_AO_MEM_ELEC_CTRL_ADDR,
			  IAXXX_AO_MEM_ELEC_CTRL_NORMAL_READ |
			  IAXXX_AO_MEM_ELEC_CTRL_VOLTAGE_RETENTION_1000);
	if (rc) {
		dev_err(dev, "Electrical control register set failed, %d\n",
						rc);
		return E_IAXXX_REGMAP_ERROR;
	}
	dev_dbg(dev, "Electrical control reg success\n");

	/* Boot the device into application mode */
	rc = iaxxx_bootup(priv);
	if (rc)
		return E_IAXXX_BOOTUP_ERROR;

	/* Call SPI speed setup callback if exists */
	if (priv->spi_speed_setup)
		priv->spi_speed_setup(dev, priv->spi_app_speed);

	return 0;
}

static int iaxxx_dump_crashlogs(struct iaxxx_priv *priv)
{
	uint32_t buf_size;
	uint32_t data_written = 0;
	int i;
	uint32_t log_addr;
	uint32_t log_size;
	int ret;

	/* If memory already allocated */
	kfree(priv->crashlog->log_buffer);
	priv->crashlog->log_buffer = NULL;
	/* Calculate total crash log dump size */
	buf_size = sizeof(struct iaxxx_crashlog_header) * IAXXX_MAX_LOG;
	for (i = 0; i < IAXXX_MAX_LOG; i++)
		buf_size += priv->crashlog->header[i].log_size;
	priv->crashlog->log_buffer_size = buf_size;
	/* Allocate the memory */
	if (!priv->crashlog->log_buffer)
		priv->crashlog->log_buffer = kzalloc(buf_size, GFP_KERNEL);
	if (!priv->crashlog->log_buffer)
		return -ENOMEM;

	/* Collect the crashlogs into log buffer */
	for (i = 0; i < IAXXX_MAX_LOG; i++) {
		/* Copy header information */
		memcpy(priv->crashlog->log_buffer + data_written,
				&priv->crashlog->header[i],
				sizeof(struct iaxxx_crashlog_header));
		data_written += sizeof(struct iaxxx_crashlog_header);
		log_addr = priv->crashlog->header[i].log_addr;
		log_size = priv->crashlog->header[i].log_size;
		/* If size of the log is 0 */
		if (!log_size)
			continue;
		/* Read the logs */
		ret = priv->bulk_read(priv->dev,
			log_addr, priv->crashlog->log_buffer + data_written,
			log_size / sizeof(uint32_t));
		if (ret != log_size / sizeof(uint32_t)) {
			dev_err(priv->dev, "Not able to read Debug logs %d\n",
					ret);
			return ret;
		}
		data_written += log_size;
	}
	dev_dbg(priv->dev, "Data written 0x%x\n", data_written);
	return 0;
}

static void iaxxx_crashlog_header_read(struct iaxxx_priv *priv)
{
	int i;
	int j = 0;
	int ret;

	/* Reading the debug log address and size */
	for (i = 0; i < IAXXX_MAX_LOG / 2 ; i++) {
		priv->crashlog->header[i].log_type = i;
		ret = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUGLOG_BLOCK_0_DEBUGLOG_ADDR_ADDR + i * 8,
			&priv->crashlog->header[i].log_addr, 2);
		if (ret) {
			dev_err(priv->dev, "Log %d address fail %d\n", i, ret);
			priv->crashlog->header[i].log_addr = 0;
			priv->crashlog->header[i].log_size = 0;
		}
	}
	/* Reading the crash log address and size */
	for (i = IAXXX_CRASHLOG_CM4; i < IAXXX_MAX_LOG; i++) {
		priv->crashlog->header[i].log_type = i;
		ret = regmap_bulk_read(priv->regmap,
			IAXXX_DEBUGLOG_BLOCK_0_CRASHLOG_ADDR_ADDR + j * 8,
			&priv->crashlog->header[i].log_addr, 2);
		j++;
		if (ret) {
			dev_err(priv->dev, "Log %d address fail %d\n", i, ret);
			priv->crashlog->header[i].log_addr = 0;
			priv->crashlog->header[i].log_size = 0;
		}
	}
	for (i = 0; i < IAXXX_MAX_LOG; i++)
		dev_dbg(priv->dev, "addr 0x%x size 0x%x\n",
				priv->crashlog->header[i].log_addr,
				priv->crashlog->header[i].log_size);
}

static int iaxxx_fw_startup(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;
	int rc;

	/* Initialize "application" regmap */
	rc = iaxxx_application_regmap_init(priv);
	if (rc)
		goto err_regmap;

	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_switch_regmap(dev, priv->regmap, priv->dfs_node);
	if (rc)
		dev_err(dev, "Failed to create debugfs entry\n");

	dev_info(dev, "%s: done\n", __func__);
	return 0;

err_regmap:
	if (priv->regmap) {
		iaxxx_dfs_del_regmap(dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
	return rc;
}

static int iaxxx_fw_recovery(struct iaxxx_priv *priv)
{
	int rc;
	uint32_t kw_bitmap;

	/* Reinitialize the regmap cache */
	rc = regmap_reinit_cache(priv->regmap, priv->regmap_config);
	if (rc) {
		dev_err(priv->dev,
			"regmap cache can not be reinitialized %d\n", rc);
		regcache_cache_bypass(priv->regmap, true);
	}

	/* HAL needs this info for loading already loaded KWs */
	kw_bitmap = priv->iaxxx_state->kw_info.kw_recognize_bitmap;
	/* Clear system state */
	memset(priv->iaxxx_state, 0, sizeof(struct iaxxx_system_state));
	priv->iaxxx_state->kw_info.kw_recognize_bitmap = kw_bitmap;

	dev_info(priv->dev, "%s: Recovery done\n", __func__);

	return 0;
}

static void iaxxx_park_work(struct kthread_work *work)
{
	while (!kthread_should_park())
		schedule();

	kthread_parkme();
}

/**
 * iaxxx_fw_update_work - work thread for initializing target
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Firmware update, switch to application mode, and install codec driver
 *
 */
static void iaxxx_fw_update_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_update_work);
	struct device *dev = priv->dev;
	bool is_startup;
	int rc;

	if (WARN(pm_runtime_enabled(dev) && pm_runtime_active(dev),
		 "FW update is unacceptable\n"))
		return;

	is_startup = !test_and_set_bit(IAXXX_FLG_STARTUP, &priv->flags);

	clear_bit(IAXXX_FLG_FW_READY, &priv->flags);
	clear_bit(IAXXX_FLG_SPI_BLOCK_CORE, &priv->flags);

	rc = iaxxx_do_fw_update(priv);
	if (rc == E_IAXXX_REGMAP_ERROR) {
		goto exit_fw_fail;
	} else if (rc == E_IAXXX_BOOTUP_ERROR) {

		/* If there's device init cb, retry */
		if (!priv->reset_cb) {
			dev_err(dev, "Chip failed to boot up\n");
			goto exit_fw_fail;
		}

		if (++priv->try_count >= IAXXX_FW_RETRY_COUNT)
			goto exit_fw_fail;

		dev_err(dev, "bootup error. retry... %d\n", priv->try_count);

		if (is_startup)
			clear_bit(IAXXX_FLG_STARTUP, &priv->flags);

		iaxxx_work(priv, fw_update_work);
		return;
	}

	/* Initialize try count for next cycle of FW loading */
	dev_dbg(dev, "Chip FW recovery: retry counter reset\n");
	priv->recovery_try_count = priv->try_count;
	priv->try_count = 0;

	if (priv->fw_crash_reasons != IAXXX_FW_CRASH_EVENT_DEBUG) {
		rc = fb_event_log_int(FBE_IAXXX_FW_LD_FAIL, 0);
		if (rc)
			dev_dbg_ratelimited(priv->dev,
				"Unable to add fb event, err=%d\n", rc);
	}

	/* Send firmware ready event to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_READY_EVENT");

	rc = is_startup ? iaxxx_fw_startup(priv) : iaxxx_fw_recovery(priv);
	if (rc)
		goto exit_fw_fail;

	/* Read debug and crash log address and size */
	iaxxx_crashlog_header_read(priv);
	/* Subscribing for FW crash event */
	rc = iaxxx_core_evt_subscribe(dev, IAXXX_CM4_CTRL_MGR_SRC_ID,
			IAXXX_CRASH_EVENT_ID, 0, 0);
	if (rc)
		dev_err(dev, "failed to subscribe for crash event\n");

	set_bit(IAXXX_FLG_FW_READY, &priv->flags);

	/* Clear all exists runtime errors */
	pm_runtime_set_suspended(dev);

	iaxxx_fw_notifier_call(dev, IAXXX_EV_APP_MODE, NULL);

	if (test_and_clear_bit(IAXXX_FLG_FW_CRASH, &priv->flags)) {
		set_bit(IAXXX_FLG_RESUME_BY_RECOVERY, &priv->flags);
		if (test_and_clear_bit(IAXXX_FLG_PM_AWAKE, &priv->flags)) {
			dev_dbg(dev, "Chip FW recovery relax wakelock: %d\n",
					__LINE__);
			pm_relax(dev);
		}
	} else {
		set_bit(IAXXX_FLG_RESUME_BY_STARTUP, &priv->flags);
	}

	iaxxx_work(priv, runtime_work);

	return;

exit_fw_fail:

	if (priv->try_count >= IAXXX_FW_RETRY_COUNT) {
		dev_err(dev, "Chip FW recovery: retry limit reached: %d!\n",
				priv->try_count);
		if (test_and_clear_bit(IAXXX_FLG_PM_AWAKE, &priv->flags)) {
			dev_dbg(dev, "Chip FW recovery relax wakelock: %d\n",
					__LINE__);
			pm_relax(dev);
		}
	}

	if (is_startup) {
		clear_bit(IAXXX_FLG_STARTUP, &priv->flags);
		/* Send metrics to fb analytics */
		if (priv->fw_crash_reasons != IAXXX_FW_CRASH_EVENT_DEBUG)
			fb_event_log_int(FBE_IAXXX_FW_LD_FAIL, -1);
	} else {
		union fb_iaxxx_fw_crash fb_event = {
			.reasons = priv->fw_crash_reasons,
			.try_count = priv->recovery_try_count,
			.status = IAXXX_FW_RECOVERY_UNSUCCESS,
		};

		/* Send metrics to fb analytics */
		if (priv->fw_crash_reasons != IAXXX_FW_CRASH_EVENT_DEBUG)
			fb_event_log_int(FBE_IAXXX_CRASH, fb_event.value);
		return;
	}

	/* Send firmware fail uevent to HAL */
	iaxxx_send_uevent(priv, "ACTION=IAXXX_FW_FAIL_EVENT");

	/* Clear try counter */
	priv->try_count = 0;

	return;
}

static int iaxxx_reset_check_sbl_mode(struct iaxxx_priv *priv)
{
	uint32_t status;
	int ret = 0;
	int mode = 0;
	int mode_retry = 5;

	if (priv->reset_cb)
		priv->reset_cb(priv->dev);

	do {
		/* Verify that the device is in bootloader mode */
		ret = regmap_read(priv->regmap,
				IAXXX_SRB_SYS_STATUS_ADDR, &status);
		if (ret) {
			dev_err(priv->dev,
				"regmap_read failed, ret = %d\n", ret);
		}

		mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
		dev_dbg(priv->dev,
			"System Status: 0x%.08X mode: 0x%.08X\n",
				status, mode);
	} while (!mode && mode_retry--);

	if (!mode && !mode_retry) {
		WARN_ON(mode != SYSTEM_STATUS_MODE_SBL);
		dev_err(priv->dev,
			"SBL SYS MODE retry expired in crash dump\n");
	}

	return ret;
}

static void iaxxx_fw_crash_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, fw_crash_work);
	int ret;

	priv->crash_count++;
	dev_info(priv->dev, "iaxxx %d time crashed\n",
			priv->crash_count);

	if (priv->cm4_crashed) {
		dev_info(priv->dev, "D4100S CM4 core crashed\n");
		iaxxx_reset_check_sbl_mode(priv);
		priv->cm4_crashed =  false;
	} else {
		priv->route_status = 0;
		ret = regmap_read(priv->regmap,
				IAXXX_PROCESSOR_CRASH_STATUS,
				&priv->core_crashed);
		/* Crash status read fails, means CM4 core crashed */
		if (ret) {
			dev_info(priv->dev,
				"something terribly wrong treat it as CM4 crash %s()\n",
				__func__);
			priv->core_crashed = IAXXX_CM4_ID;
		}
		if (priv->core_crashed == IAXXX_CM4_ID) {
			dev_info(priv->dev, "D4100S CM4 core crashed\n");
			iaxxx_reset_check_sbl_mode(priv);
			priv->cm4_crashed =  false;
		} else if (priv->core_crashed == 1 << IAXXX_HMD_ID)
			dev_info(priv->dev, "D4100S HMD core crashed\n");
		else if (priv->core_crashed == 1 << IAXXX_DMX_ID)
			dev_info(priv->dev, "D4100S DMX Core Crashed\n");
		else
			dev_info(priv->dev, "D4100S Update block failed recovery\n");
	}
	/* Collect the crash logs */
	mutex_lock(&priv->crashdump_lock);
	iaxxx_dump_crashlogs(priv);
	mutex_unlock(&priv->crashdump_lock);

	/* Notify the user about crash and read crash dump log*/
	iaxxx_send_uevent(priv, "ACTION=IAXXX_CRASH_EVENT");

	/* Bypass regmap cache */
	regcache_cache_bypass(priv->regmap, true);

	iaxxx_work(priv, fw_update_work);

	return;
}

/*
 * Forced suspend/resume of Knowles devices.
 *
 * @priv - Context structure
 *
 * This is the main logic that decides whether to suspend or resume.
 * Only need to be called from the driver kthread context, never direct.
 * Must ensure sequential and one-time forced suspend/resume.
 * Remember FW crash pending flag if recovery is aborted due privacy mode.
 * Restart FW recovery when is possible until complete.
 *
 */
static void iaxxx_runtime_work(struct kthread_work *work)
{
	struct iaxxx_priv *priv = iaxxx_ptr2priv(work, runtime_work);
	unsigned long *flags = &priv->flags;
	bool is_fw_crash, is_privacy, is_sys_suspend, do_suspend, allow;

	/* Suspend due FW crash panding or aborted recovery */
	is_fw_crash = test_bit(IAXXX_FLG_FW_CRASH, flags);

	/* Suspend due privacy mode */
	is_privacy = test_bit(IAXXX_FLG_SUSPEND_ON_PRIVACY, flags) &&
		     test_bit(IAXXX_FLG_PRIVACY, flags);

	is_sys_suspend = test_bit(IAXXX_FLG_SYSTEM_SUSPEND, flags);

	/* Allow entering in suspend to start recovery procedure */
	allow = test_bit(IAXXX_FLG_BOOT_COMPLETE, flags) ||
		!test_bit(IAXXX_FLG_SUSPEND_ON_PRIVACY, flags);

	do_suspend = (is_fw_crash || is_privacy || is_sys_suspend) && allow;

	if (do_suspend)
		iaxxx_do_suspend(priv, is_fw_crash, is_privacy);
	else
		iaxxx_do_resume(priv, is_fw_crash, is_privacy);
}

int iaxxx_fw_crash(struct device *dev, enum iaxxx_fw_crash_reasons reasons)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!test_and_set_bit(IAXXX_FLG_PM_AWAKE, &priv->flags)) {
		dev_dbg(dev, "Chip FW Crash: keep device awake\n");
		pm_stay_awake(dev);
	}

	/* Backtrace of FW crash is useful information to analyze root case */
	WARN_RATELIMIT(true, "FW Crash: PM %s/%s ; reasons %d (%s) %s %08lx\n",
		pm_runtime_enabled(dev) ? " enable" : "disable",
		pm_runtime_active(dev) ? "active" : "inactive",
		reasons, iaxxx_crash_err2str(reasons),
		test_bit(IAXXX_FLG_FW_CRASH, &priv->flags) ? "- Ignored" : "",
		priv->flags);

	/* Avoid second times if currently is handled */
	if (test_and_set_bit(IAXXX_FLG_FW_CRASH, &priv->flags)) {
		static DEFINE_RATELIMIT_STATE(rl,
			DEFAULT_RATELIMIT_INTERVAL, DEFAULT_RATELIMIT_BURST);

		/* Avoiding often FW_CRASH events durgin handling FW_CRASH */
		if (!__ratelimit(&rl) && !in_interrupt())
			msleep(100);

		return -EBUSY;
	}

	priv->fw_crash_reasons = reasons;

	/* Update runtime pm status */
	iaxxx_work(priv, runtime_work);

	/* Trace firmware crash */
	if (priv->fw_crash_reasons != IAXXX_FW_CRASH_EVENT_DEBUG)
		trace_fba_fw_crash(iaxxx_crash_err2str(reasons));

	return 0;
}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
/**
 * iaxxx_fw_update_test_work - work thread for running firmware test
 *
 * @work : used to retrieve Transport Layer private structure
 * Set the regmap to sbl mode and firmware update
 * If firmware update successful, then set bootup_done
 *
 */
static void iaxxx_fw_update_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_fw_update_test_work);
	int rc;

	priv->test_result = false;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);

	if (!rc)
		priv->test_result = true;

	complete_all(&priv->bootup_done);
}

/*
 * iaxxx_abort_fw_recovery - abort current FW loading works
 *
 * @priv - context structure of driver
 *
 * FW Recovery procedure take more then 4sec.  When entering privacy mode,
 * we block the notifier call chain for the entire suspend time. Current
 * FW load procedure must be aborted. And started again when privacy complete
 *
 */
static int iaxxx_abort_fw_recovery(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	/* Not need abort if device is in active state */
	if (pm_runtime_enabled(dev) && pm_runtime_active(dev))
		return -EPERM;

	if (!test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
		return -EPERM;

	dev_info(dev, "Aborting FW recovery...\n");

	set_bit(IAXXX_FLG_SPI_BLOCK_CORE, &priv->flags);
	kthread_cancel_work_sync(&priv->runtime_work);
	kthread_cancel_work_sync(&priv->fw_crash_work);
	kthread_cancel_work_sync(&priv->fw_update_work);
	clear_bit(IAXXX_FLG_SPI_BLOCK_CORE, &priv->flags);

	dev_info(dev, "FW recovery aborted!\n");

	return 0;
}

/**
 * iaxxx_cmem_test_work - work thread for running memory test
 *
 * @work : used to retrieve Transport Layer private structure
 *
 * Wait for an event interrupt happening with timeout
 *
 */
static void iaxxx_cmem_test_work(struct work_struct *work)
{
	struct iaxxx_priv *priv = container_of(work,
			struct iaxxx_priv, dev_cmem_test_work);
	int rc;

	/* Initialize regmap for SBL */
	rc = iaxxx_sbl_regmap_init(priv);
	if (!rc)
		rc = iaxxx_do_fw_update(priv);
}

/**
 * iaxxx_test_init - initialize work items and mutex for test environment
 *
 * @priv	: iaxxx private data
 *
 */
static void iaxxx_test_init(struct iaxxx_priv *priv)
{
	INIT_WORK(&priv->dev_fw_update_test_work, iaxxx_fw_update_test_work);
	INIT_WORK(&priv->dev_cmem_test_work, iaxxx_cmem_test_work);
	mutex_init(&priv->test_mutex);
	init_completion(&priv->bootup_done);
	init_completion(&priv->cmem_done);
}
#endif

static int iaxxx_notifier_cb(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct iaxxx_priv *priv = container_of(nb,
			struct iaxxx_priv, notifier_core);

	switch (val) {
	case IAXXX_EV_CRASH:
		dev_err(priv->dev, "FW Crash broadcast event!\n");
		break;
	case IAXXX_EV_ROUTE_ACTIVE:
		/*
		 * TODO: This event must notify when boot is totally completed.
		 *	Unfortunately it's not absolutely true because KW
		 *	enrollment occurs after this notify. This makes
		 *	potential problem when we boot in privacy mode.
		 *	In this case we should delay more times. But we don't
		 *	know how much. That's why we delay until we exit
		 *	from privacy mode.
		 */

		if (test_bit(IAXXX_FLG_PRIVACY, &priv->flags))
			break;

		if (!test_and_set_bit(IAXXX_FLG_BOOT_COMPLETE, &priv->flags))
			iaxxx_work(priv, runtime_work);
		break;
	}

	return 0;
}

static int get_version_str(struct iaxxx_priv *priv, uint32_t reg, char *verbuf,
								uint32_t len)
{
	int rc;
	uint32_t addr;
	uint32_t i = 0;

	/* Read the version string address */
	rc = regmap_read(priv->regmap, reg, &addr);
	if (rc) {
		dev_err(priv->dev, "%s() String address read failed %d\n",
							__func__, rc);
		return rc;
	}
	pr_debug("%s() String address 0x%x\n", __func__, addr);

	/* Read the FW string from address read above */
	while (len > 0) {
		rc = priv->bulk_read(priv->dev, addr + i, &verbuf[i], 1);
		if (rc != 1) {
			dev_err(priv->dev, "String Read fail addr 0x%x:%d\n",
							addr + i, rc);
			return -EIO;
		}
		/* Reached NULL character, FW version string ends here */
		if ((!verbuf[i]) || (!verbuf[i + 1]) ||
				(!verbuf[i + 2]) || (!verbuf[i + 3])) {
			pr_debug("%s() String ends here\n", __func__);
			i += IAXXX_BYTES_IN_A_WORD;
			break;
		}
		/* 4 characters read, go for next 4 bytes to read */
		len -= IAXXX_BYTES_IN_A_WORD;
		i += IAXXX_BYTES_IN_A_WORD;
	}

	verbuf[IAXXX_VER_STR_SIZE - 1] = '\0';
	print_hex_dump(KERN_INFO, "Version: ", DUMP_PREFIX_OFFSET, 32, 4,
			(void *)verbuf, i, true);
	/*
	 * If not reached end of buffer and buffer is not empty,
	 * then print Firmware version.
	 */
	if (len > 0 && verbuf[0] != '\0')
		return 0;
	return -EIO;
}

/**
 * iaxxx_firmware_version_show - sys node show function for firmware version
 */
static ssize_t iaxxx_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	rc = get_version_str(priv, IAXXX_SRB_SYS_PATCH_VER_STR_ADDR, verbuf,
									len);
	if (rc) {
		dev_err(dev, "%s() FW version read fail\n", __func__);
		return -EIO;
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", verbuf);
}
static DEVICE_ATTR(fw_version, S_IRUSR, iaxxx_firmware_version_show, NULL);

/**
 * iaxxx_plugin_version_show - sys node show function for plugin version
 */
static ssize_t iaxxx_plugin_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	int i;
	uint32_t buf_len = 0;
	static const char * const plugin[] = {
		"VQ", "VP", "Buffer", "Mixer", "MBC", "PEQ"};

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < IAXXX_MAX_PLUGIN; i++) {
		rc = get_version_str(
			priv, IAXXX_PLUGIN_INS_GRP_PLUGIN_VER_STR_REG(i),
			verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Plugin version read fail\n",
								__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE, "%s:\t%s\n",
							plugin[i], verbuf);
	}
	return buf_len;
}
static DEVICE_ATTR(plugin_version, S_IRUSR, iaxxx_plugin_version_show, NULL);

/**
 * iaxxx_package_version_show - sys node show function for package version
 */
static ssize_t iaxxx_package_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	int i;
	uint32_t buf_len = 0;
	static const char * const package[] = {"Pepperoni", "Buffer", "Mixer"};
	uint32_t inst_id[] = {0, 2, 3};

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < IAXXX_MAX_PACKAGE; i++) {
		rc = get_version_str(priv,
			IAXXX_PLUGIN_INS_GRP_PACKAGE_VER_STR_REG(inst_id[i]),
			verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Package version read fail\n",
								__func__);
			return -EIO;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE, "%s:\t%s\n",
							package[i], verbuf);
	}
	return buf_len;
}
static DEVICE_ATTR(package_version, S_IRUSR, iaxxx_package_version_show, NULL);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
/**
 * iaxxx_firmware_timestamp_show - sys node show function for firmware timestamp
 */
static ssize_t iaxxx_firmware_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t fw_clk_rd[2];
	uint64_t timestamp;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "Invalid device\n");

	/* Read Firmware wall clock timestamp */
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
			fw_clk_rd, ARRAY_SIZE(fw_clk_rd));
	if (rc) {
		dev_err(dev,
			"Failed IAXXX_AF_WCPT_WALL_CLOCK_RD, rc:%d\n", rc);
		return rc;
	}
	timestamp = (((long)((fw_clk_rd[1] & 0xFFFF)) << 32) | fw_clk_rd[0]);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", timestamp);
}
static DEVICE_ATTR(fw_timestamp, S_IRUSR, iaxxx_firmware_timestamp_show, NULL);

/**
 * iaxxx_firmware_update_test_show - sys node show function for firmware test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_firmware_update_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_fw_update_test_work);
	init_completion(&priv->bootup_done);
	schedule_work(&priv->dev_fw_update_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->bootup_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));

	if (rc > 0 && priv->test_result)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(fw_update_test, S_IRUSR, iaxxx_firmware_update_test_show, NULL);

/**
 * iaxxx_cmem_test_show - sys node show function for memory test
 *
 * Trigger firmware update test work and return results
 *
 */
static ssize_t iaxxx_cmem_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	long rc;
	ssize_t count;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	mutex_lock(&priv->test_mutex);

	cancel_work_sync(&priv->dev_cmem_test_work);
	init_completion(&priv->cmem_done);
	schedule_work(&priv->dev_cmem_test_work);

	rc = wait_for_completion_interruptible_timeout(&priv->cmem_done,
			msecs_to_jiffies(IAXXX_FW_DOWNLOAD_TIMEOUT));
	if (rc > 0)
		count = scnprintf(buf, PAGE_SIZE, "SUCCESS\n");
	else
		count = scnprintf(buf, PAGE_SIZE, "FAIL\n");

	mutex_unlock(&priv->test_mutex);

	return count;
}
static DEVICE_ATTR(cmem_test, S_IRUSR, iaxxx_cmem_test_show, NULL);
#endif

/*
 * Sysfs - firmware download complete
 */
static int iaxxx_event_flush_and_enable(struct iaxxx_priv *priv)
{
	mutex_lock(&priv->event_queue_lock);

	/* Clear event queue */
	priv->event_queue->w_index = -1;
	priv->event_queue->r_index = -1;

	mutex_unlock(&priv->event_queue_lock);

	if (gpio_is_valid(priv->event_gpio))
		enable_irq(gpio_to_irq(priv->event_gpio));

	/* Clients regmap access must be enabled here */
	queue_work(priv->event_workq, &priv->event_work_struct);

	return 0;
}

int iaxxx_core_rt_suspend(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!pm_runtime_enabled(dev) && pm_runtime_active(dev)) {
		/*
		 * This indicate point where lower layer has problem and
		 * all SPI transactions and works with lower layer must
		 * be blocked because is forbidden.
		 */
		dev_info(dev, "Forced suspend requested by HW!");
	}

	set_bit(IAXXX_FLG_SPI_BLOCK_CLIENTS, &priv->flags);

	/* Block SPI transaction for iaxxx-core, in privacy mode */
	if (test_bit(IAXXX_FLG_PRIVACY, &priv->flags) &&
	    test_bit(IAXXX_FLG_SUSPEND_ON_PRIVACY, &priv->flags))
		set_bit(IAXXX_FLG_SPI_BLOCK_CORE, &priv->flags);

	if (gpio_is_valid(priv->event_gpio))
		disable_irq(gpio_to_irq(priv->event_gpio));

	return 0;
}

int iaxxx_core_rt_resume(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	/*
	 * Should not allow resume before core is ready for that.
	 * This automatically blocks its children's direct and
	 * their children too. Such files:
	 *   /dev/tunnel0
	 *   /dev//dev/iaxxx-odsp-celldrv
	 *   and many others
	 */
	if (!test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		return -EBUSY;

	clear_bit(IAXXX_FLG_SPI_BLOCK_CORE, &priv->flags);
	clear_bit(IAXXX_FLG_SPI_BLOCK_CLIENTS, &priv->flags);

	/* Reegmap access must be enabled before enable event interrupt */
	iaxxx_event_flush_and_enable(priv);

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

static int iaxxx_pm_notifier(struct notifier_block *nb,
				unsigned long event, void *unused)
{
	struct iaxxx_priv *priv = container_of(nb,
			struct iaxxx_priv, notifier_pm);

	switch (event) {
	/* Suspend notify before freeze user-space & firmware framework */
	case PM_SUSPEND_PREPARE:
		set_bit(IAXXX_FLG_SYSTEM_SUSPEND, &priv->flags);
		iaxxx_work(priv, runtime_work);
		iaxxx_work(priv, park_work);
		kthread_park(priv->worker.task);
		break;

	/* System wakeup or error during suspend */
	case PM_POST_SUSPEND:
		clear_bit(IAXXX_FLG_SYSTEM_SUSPEND, &priv->flags);
		iaxxx_work(priv, runtime_work);
		kthread_unpark(priv->worker.task);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

int iaxxx_core_dev_suspend(struct device *dev)
{
	/* Don't suspend if PM is disabled or not active */
	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return 0;

	return iaxxx_core_rt_suspend(dev);
}

int iaxxx_core_dev_resume(struct device *dev)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	/* Don't suspend if PM is disabled or not active */
	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return 0;

	/* Set wakeup source on first event */
	set_bit(IAXXX_FLG_WAKEUP_EVENT, &priv->flags);

	return iaxxx_core_rt_resume(dev);
}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
/**
 * iaxxx_pm_enable - store function for suspend and resume
 */
static ssize_t iaxxx_pm_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		dev_dbg(dev, "%s() PM Resume\n", __func__);
		set_bit(IAXXX_FLG_PRIVACY, &priv->flags);

		/* Abort current FW recovery procedure */
		iaxxx_abort_fw_recovery(priv);

		/* Start suspending and block until complete */
		iaxxx_work(priv, runtime_work);
		iaxxx_work_flush(priv, runtime_work);
	} else {
		dev_dbg(dev, "%s() PM Suspend\n", __func__);
		clear_bit(IAXXX_FLG_PRIVACY, &priv->flags);

		/* Async tring to resume */
		iaxxx_work(priv, runtime_work);
	}

	return count;
}
static DEVICE_ATTR(pm_enable, S_IWUSR, NULL, iaxxx_pm_enable);

/*
 * Sysfs - firmware crash event
 */
static ssize_t iaxxx_fw_crash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT_DEBUG);

	return count;
}
static DEVICE_ATTR(fw_crash, S_IWUSR, NULL, iaxxx_fw_crash_store);
#endif

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_plugin_version.attr,
	&dev_attr_package_version.attr,
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	&dev_attr_fw_update_test.attr,
	&dev_attr_cmem_test.attr,
	&dev_attr_fw_timestamp.attr,
	&dev_attr_pm_enable.attr,
	&dev_attr_fw_crash.attr,
#endif
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
	.name	= "iaxxx"
};

/**
 * iaxxx_device_power_init - init power
 *
 * @priv: iaxxx private data
 */
static int iaxxx_device_power_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	/* Initialize the platform data */
	rc = iaxxx_populate_dt_pdata(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize platform data: %d\n", rc);
		goto err_populate_pdata;
	}

	/* Initialize the GPIOs */
	rc = iaxxx_gpio_init(priv);
	if (rc) {
		dev_err(dev, "Failed to initialize GPIOs: %d\n", rc);
		goto err_gpio_init;
	}

	/* Initialize interrupts */
	rc = iaxxx_irq_init(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize interrupts: %d\n", rc);
		goto err_irq_init;
	}

	/* Enable regulator */
	rc = iaxxx_reg_enable(priv);
	if (rc) {
		dev_err(dev,
			"Failed to initialize interrupts: %d\n", rc);
		goto err_reg_init;
	}

	if (priv->ext_clk) {
		/* Clock enable if external clock is used */
		clk_prepare_enable(priv->ext_clk);
		usleep_range(IAXXX_CLK_READY_DELAY,
				IAXXX_CLK_READY_DELAY +
				IAXXX_CLK_RANGE_INTERVAL);
	}

	return 0;

err_reg_init:
	iaxxx_irq_exit(priv);
err_irq_init:
err_gpio_init:
err_populate_pdata:
	return rc;
}

static int iaxxx_parse_dt_data(struct iaxxx_priv *priv)
{
	struct device_node *np = priv->dev->of_node;

	if (of_property_read_bool(np, "iaxxx,suspend-on-privacy"))
		set_bit(IAXXX_FLG_SUSPEND_ON_PRIVACY, &priv->flags);

	return 0;
}

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_device_reset - called from probe to perform chip reset
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_reset(struct iaxxx_priv *priv)
{
	/* Pull the device out of reset. */
	return iaxxx_reset_to_sbl(priv);
}

int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_register(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	ret = srcu_notifier_chain_unregister(&priv->core_notifier_list, nb);
	return ret;
}

int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	return srcu_notifier_call_chain(&priv->core_notifier_list, val, v);
}

/**
 * iaxxx_device_init - called from probe to perform device initialization
 *
 * @priv: iaxxx private data
 */
int iaxxx_device_init(struct iaxxx_priv *priv)
{
	int rc;

	rc = iaxxx_parse_dt_data(priv);
	if (rc) {
		dev_err(priv->dev, "Failed to parse DT data\n");
		goto err_dt_data;
	}

	/* Init mutexes */
	mutex_init(&priv->srb_lock);
	mutex_init(&priv->event_work_lock);
	mutex_init(&priv->event_queue_lock);
	mutex_init(&priv->plugin_lock);
	mutex_init(&priv->crashdump_lock);

	init_kthread_worker(&priv->worker);
	priv->thread = kthread_run(kthread_worker_fn, &priv->worker,
				   "iaxxx-core");
	if (IS_ERR(priv->thread)) {
		dev_err(priv->dev, "Can't create kthread worker: %ld\n",
			PTR_ERR(priv->thread));
		return PTR_ERR(priv->thread);
	}

	init_kthread_work(&priv->fw_update_work, iaxxx_fw_update_work);
	init_kthread_work(&priv->fw_crash_work, iaxxx_fw_crash_work);
	init_kthread_work(&priv->runtime_work, iaxxx_runtime_work);
	init_kthread_work(&priv->park_work, iaxxx_park_work);

	/* Initialize regmap for SBL */
	rc = iaxxx_regmap_init(priv);
	if (rc)
		return rc;

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	/* Initialize the register dump */
	rc = iaxxx_regdump_init(priv);
	if (rc)
		goto err_regdump_init;
#endif

	/* Create sysfs */
	if (sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group)) {
		dev_err(priv->dev,
			"%s [ERROR] sysfs_create_group\n", __func__);
	}

	/* TODO: SYSTEM_ROM_VER_STR */

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	/* Initialize test environment */
	iaxxx_test_init(priv);
#endif

	/* Initialize the cdev interface */
	rc = iaxxx_cdev_init();
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize cdev interface: %d\n", rc);
		goto err_debug_init;
	}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	/* Initialize the debug interface */
	rc = iaxxx_debug_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to initialize debug interface: %d\n", rc);
		goto err_debug_init;
	}
#endif

	srcu_init_notifier_head(&priv->core_notifier_list);

	/* Init early stage for tunneling */
	rc = iaxxx_tunnel_dev_init_early(priv);
	if (rc) {
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);
		goto err_debug_init;
	}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	/* Add debugfs node for regmap */
	rc = iaxxx_dfs_add_regmap(priv->dev, priv->regmap, &priv->dfs_node);
	if (rc)
		dev_err(priv->dev,
			"%s: Failed to create debugfs entry\n", __func__);
#endif

	priv->notifier_pm.notifier_call = iaxxx_pm_notifier;
	priv->notifier_pm.priority = 0;
	rc = register_pm_notifier(&priv->notifier_pm);
	if (rc)
		dev_err(priv->dev, "failed to register PM notifier\n");

	priv->notifier_core.notifier_call = iaxxx_notifier_cb;
	priv->notifier_core.priority = INT_MAX;
	rc = iaxxx_fw_notifier_register(priv->dev, &priv->notifier_core);
	if (rc) {
		dev_err(priv->dev, "failed to register for core notifier\n");
		goto err_fb_notifier;
	}

	priv->event_ws = wakeup_source_register("iaxxx");
	if (!priv->event_ws) {
		dev_err(priv->dev, "failed to register wakeup source\n");
		goto err_wakeup_source;
	}

	rc = iaxxx_event_init(priv);
	if (rc) {
		dev_err(priv->dev, "Failed to initialize the event\n");
		goto err_event_init;
	}

	/*
	 * Make the device power up the chip first so that
	 * the knowles chip won't cause any i2c communication error
	 * for other devices on the same bus
	 */
	rc = iaxxx_device_power_init(priv);
	if (rc) {
		dev_err(priv->dev,
			"Failed to power up device: %d\n", rc);
		goto err_power_init;
	}

	rc = mfd_add_devices(priv->dev, -1, iaxxx_devices,
		ARRAY_SIZE(iaxxx_devices), NULL, 0, NULL);
	if (rc) {
		dev_err(priv->dev, "Failed to add cell devices\n");
		goto err_add_devices;
	}

	pm_runtime_enable(priv->dev);
	device_init_wakeup(priv->dev, true);

	/* Kick work queue for firmware loading */
	iaxxx_work(priv, fw_update_work);

	return 0;

err_add_devices:
err_power_init:
	iaxxx_event_exit(priv);
err_event_init:
	wakeup_source_unregister(priv->event_ws);
err_wakeup_source:
err_fb_notifier:
err_debug_init:
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
err_regdump_init:
#endif
	if (priv->regmap) {
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
#endif
		regmap_exit(priv->regmap);
	}
err_dt_data:
	return -EINVAL;
}

void iaxxx_device_exit(struct iaxxx_priv *priv)
{
	pm_runtime_disable(priv->dev);
	mfd_remove_devices(priv->dev);

	/* Delete the work queue */
	flush_work(&priv->event_work_struct);
	destroy_workqueue(priv->event_workq);

	flush_kthread_worker(&priv->worker);
	kthread_stop(priv->thread);

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	iaxxx_regdump_exit(priv);
#endif
	iaxxx_irq_exit(priv);
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	iaxxx_debug_exit(priv);
#endif
	iaxxx_cdev_exit();
	iaxxx_event_exit(priv);

	wakeup_source_unregister(priv->event_ws);
	device_init_wakeup(priv->dev, false);
	unregister_pm_notifier(&priv->notifier_pm);

	if (priv->regmap_config->ranges) {
		devm_kfree(priv->dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = NULL;
	}

	if (priv->regmap) {
		iaxxx_dfs_del_regmap(priv->dev, priv->regmap);
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}
}
