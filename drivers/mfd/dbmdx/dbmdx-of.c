/*
 * dbmdx-of.c - DSPG DBMDX Device Tree Data API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */

#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/version.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#ifdef CONFIG_OF_I2C
#include <linux/of_i2c.h>
#endif /* CONFIG_OF_I2C */
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-chip-interfaces.h>
#include <linux/mfd/dbmdx/dbmdx-clocks.h>




static int dbmdx_get_gpio_devtree(struct dbmdx_private *p)
{
	struct device_node *np = p->dev->of_node;

	/* reset */
	p->pdata->gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);

	p->pdata->gpio_host_wakeup = of_get_named_gpio(np,
						"host-wakeup-gpio", 0);

	p->pdata->gpio_rx_ready = of_get_named_gpio(np, "rx-ready-gpio", 0);

	/* wakeup */
	p->pdata->gpio_wakeup = of_get_named_gpio(np, "wakeup-gpio", 0);

	return 0;

}

static int dbmdx_get_primary_devtree_pdata(struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct device_node *np;
	int ret = 0;
	struct device *dev;
	int i;

	dev = p->dev;
	np = dev->of_node;


	/* read file names for the various firmwares */
	/* read name of Pr. Firmware */
	ret = of_property_read_string(np,
				      "primary-firmware-name",
				      &pdata->primary_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->primary_firmware_name = DBMDX_PRIMARY_FIRMWARE_NAME;
		dev_info(dev, "%s: using default Pr. Firmware name: %s\n",
			 __func__, pdata->primary_firmware_name);
	} else
		dev_info(dev, "%s: using device-tree Pr. Firmware name: %s\n",
			__func__, pdata->primary_firmware_name);

	pdata->primary_firmware_name_default = pdata->primary_firmware_name;
	/* read name of 3p algo. Firmware */
	ret = of_property_read_string(np,
				      "primary-firmware-name-3p",
				      &pdata->primary_firmware_name_3p);
	if (ret != 0) {
		/* set default name */
		pdata->primary_firmware_name_3p =
					pdata->primary_firmware_name_default;
		dev_info(dev, "%s: using default 3p algo. Firmware name: %s\n",
			 __func__, pdata->primary_firmware_name_3p);
	} else
		dev_info(dev, "%s: using device-tree 3p algo. Firmware name: %s\n",
			__func__, pdata->primary_firmware_name_3p);

	/* read name of 1p algo. Firmware */
	ret = of_property_read_string(np,
				      "primary-firmware-name-1p",
				      &pdata->primary_firmware_name_1p);
	if (ret != 0) {
		/* set default name */
		pdata->primary_firmware_name_1p =
				      pdata->primary_firmware_name_default;
		dev_info(dev, "%s: using default 1p algo. Firmware name: %s\n",
			 __func__, pdata->primary_firmware_name_1p);
	} else
		dev_info(dev, "%s: using device-tree 1p algo. Firmware name: %s\n",
			__func__, pdata->primary_firmware_name_1p);

	ret = of_property_read_string(np,
				      "primary-preboot-firmware-name",
				      &pdata->primary_preboot_firmware_name);
	if (ret != 0) {
		/* set default name */
		pdata->primary_preboot_firmware_name =
			DBMDX_PRIMARY_PREBOOT_FIRMWARE_NAME;
		dev_info(dev,
			"%s: using default pr. preboot firmware name: %s\n",
			 __func__, pdata->primary_preboot_firmware_name);
	} else
		dev_info(dev,
			"%s: using device-tree pr. preboot firmware name: %s\n",
			__func__, pdata->primary_preboot_firmware_name);

	ret = of_property_count_strings(np, "addon-firmware-names");
	if (ret < 0 && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'addon-firmware-names'\n",
			__func__);
		ret = -EINVAL;
		pdata->num_of_fw_addons = 0;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no 'addon-firmware-names' def. in device-tree.\n",
			__func__);
		pdata->num_of_fw_addons = 0;
	} else {
		pdata->num_of_fw_addons = ret;
		if (pdata->num_of_fw_addons > MAX_NUM_OF_ADDONS) {
			dev_warn(p->dev,
			"%s: Num of addons:%d exceeds max num: %d, trimming.\n",
				__func__, pdata->num_of_fw_addons,
				MAX_NUM_OF_ADDONS);
			pdata->num_of_fw_addons = MAX_NUM_OF_ADDONS;
		}
	}

	if (pdata->num_of_fw_addons) {
		ret = of_property_read_string_array(np, "addon-firmware-names",
						pdata->add_on_fw_names,
						pdata->num_of_fw_addons);
		if (ret != pdata->num_of_fw_addons) {
			dev_err(p->dev,
				"%s: Error reading 'addon-firwmare-names'\n",
				__func__);
			ret = -EINVAL;
			pdata->num_of_fw_addons = 0;
			goto out_err;
		}

		for (i = 0; i < pdata->num_of_fw_addons; i++)
			dev_dbg(dev, "%s: Using FW Addon Name %d:\t%s\n",
				__func__, i, pdata->add_on_fw_names[i]);
	}

	ret = dbmdx_get_fw_interfaces(p, "primary-interfaces",
		p->pdata->interfaces);

out_err:
	return ret;

}

int dbmdx_init_cfg_reg_list_populate(struct dbmdx_private *p)
{
	struct property *property = NULL;
	struct device_node *np;
	int ret = 0;
	int len_in_bytes = 0;
	int i;

	np = p->dev->of_node;
	property = of_find_property(np, "init_cfg_reg_list", &len_in_bytes);

	if (!property || !len_in_bytes) {
		dev_info(p->dev, "%s: No init_cfg_reg_list indev-tree\n",
			__func__);
		return 0;
	}

	/* List is kept in memory */
	p->pdata->init_cfg_reg_list = kzalloc(len_in_bytes, GFP_KERNEL);

	if (!p->pdata->init_cfg_reg_list)
		return -ENOMEM;

	p->pdata->init_cfg_reg_list_items = (len_in_bytes /
						sizeof(struct fw_register));

	ret = of_property_read_u32_array(np,
					"init_cfg_reg_list",
					(u32 *)p->pdata->init_cfg_reg_list,
					(len_in_bytes / sizeof(u32)));
	if (ret) {
		dev_err(p->dev, "%s: Error reading init_cfg_reg_list\n",
				__func__);
		kfree(p->pdata->init_cfg_reg_list);
		p->pdata->init_cfg_reg_list_items = 0;
		return -EINVAL;
	}

	dev_info(p->dev, "%s: using %u  init config values from dev-tree\n",
			__func__, p->pdata->init_cfg_reg_list_items);

	for (i = 0; i < p->pdata->init_cfg_reg_list_items; i++)
		dev_info(p->dev,
			"%s: \twrite reg: index<%d> cmd<0x%2x> pr<0x%x> id<0x%x> reg<0x%02x> <val1>0x%08x <val2>0x%08x <val3>0x%08x\n",
			__func__,
			i,
			p->pdata->init_cfg_reg_list[i].cmd,
			p->pdata->init_cfg_reg_list[i].priority,
			p->pdata->init_cfg_reg_list[i].id,
			p->pdata->init_cfg_reg_list[i].reg_num,
			p->pdata->init_cfg_reg_list[i].val1,
			p->pdata->init_cfg_reg_list[i].val2,
			p->pdata->init_cfg_reg_list[i].val3);

	return ret;
}


static int dbmdx_get_devtree_pdata(struct dbmdx_private *p)
{
	struct dbmdx_platform_data *pdata = p->pdata;
	struct property *property = NULL;
	struct device_node *np;
	int ret = 0;
	int len = 0;
	int i = 0;
	struct device *dev = p->dev;

	np = dev->of_node;

	if (of_find_property(np, "multi-interface-support", NULL)) {
		dev_info(dev, "%s: Multi Interface Probe is supported\n",
			__func__);
		pdata->multi_interface_support = 1;
	} else {
		dev_info(dev, "%s: Multi Interface Probe is not supported\n",
			__func__);
		pdata->multi_interface_support = 0;
	}

	ret = dbmdx_get_primary_devtree_pdata(p);
	if (ret != 0) {
		dev_err(dev, "%s: Error reading Primary device tree data\n",
				__func__);
		ret = -EINVAL;
		goto out_err;
	}

	property = of_find_property(np, "host-speeds", &len);
	if (property) {
		if (len < DBMDX_NR_OF_SPEEDS * 4) {
			dev_err(dev,
				"%s: Host speed configuration table too short\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
		ret = of_property_read_u32_array(np,
						 "host-speeds",
						 (u32 *)&pdata->host_speed_cfg,
						 DBMDX_NR_OF_SPEEDS * 4);
		if (ret) {
			dev_err(dev,
				"%s: Could not read Host speed configuration\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
		dev_info(dev,
			"%s: using %u Host speed configuration values from device-tree\n",
			__func__,
			DBMDX_NR_OF_SPEEDS);
		for (i = 0; i < DBMDX_NR_OF_SPEEDS; i++)
			dev_dbg(dev, "%s: Host speed cfg %8.8x: 0x%8.8x %u %u %u\n",
				__func__,
				i,
				pdata->host_speed_cfg[i].cfg,
				pdata->host_speed_cfg[i].uart_baud,
				pdata->host_speed_cfg[i].i2c_rate,
				pdata->host_speed_cfg[i].spi_rate);
	}

	ret = of_property_read_u32(np, "hw_revision",
		&p->pdata->hw_rev);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'hw_revision'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no hw_revision definition in device-tree.\n",
			__func__);
		p->pdata->hw_rev = DBMDX_DEFAULT_HW_REV;
	}

	ret = of_property_read_u32(np, "wakeup_disabled",
		&p->pdata->wakeup_disabled);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_disabled definition in dev-tree\n",
			__func__);
		p->pdata->wakeup_disabled = 0;
	} else {

		if (p->pdata->wakeup_disabled > 1)
			p->pdata->wakeup_disabled = 1;

		dev_info(p->dev,
			"%s: using wakeup_disabled of %d from dev-tree\n",
			 __func__, p->pdata->wakeup_disabled);

	}

	ret = of_property_read_u32(np, "use_gpio_for_wakeup",
		&p->pdata->use_gpio_for_wakeup);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'use_gpio_for_wakeup'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no use_gpio_for_wakeup definition in dev-tree\n",
			__func__);
		p->pdata->use_gpio_for_wakeup = 1;
	} else {

		if (p->pdata->use_gpio_for_wakeup > 1)
			p->pdata->use_gpio_for_wakeup = 1;

		dev_info(p->dev,
			"%s: using use_gpio_for_wakeup of %d from dev-tree\n",
			 __func__, p->pdata->use_gpio_for_wakeup);

	}

	ret = of_property_read_u32(np, "send_wakeup_seq",
		&p->pdata->send_wakeup_seq);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'send_wakeup_seq'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no send_wakeup_seq definition in device-tree\n",
			__func__);
		p->pdata->send_wakeup_seq = 0;
	} else {

		if (p->pdata->send_wakeup_seq > 1)
			p->pdata->send_wakeup_seq = 1;

		dev_info(p->dev,
			"%s: using send_wakeup_seq of %d from device-tree\n",
			 __func__, p->pdata->send_wakeup_seq);

	}

	ret = of_property_read_u32(np, "wakeup_set_value",
		&p->pdata->wakeup_set_value);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'wakeup_set_value'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no wakeup_set_value definition in device-tree\n",
			__func__);
		p->pdata->wakeup_set_value = 1;
	} else {

		if (p->pdata->wakeup_set_value > 1)
			p->pdata->wakeup_set_value = 1;

		dev_info(p->dev,
			"%s: using wakeup_set_value of %d from device-tree\n",
			 __func__, p->pdata->wakeup_set_value);

	}
	property = of_find_property(np, "firmware_id", &pdata->firmware_id_num);
	if (property) {
		pdata->firmware_id_num /= sizeof(u32);
		if (pdata->firmware_id_num > MAX_NUM_OF_FW_IDS) {
			pdata->firmware_id_num = MAX_NUM_OF_FW_IDS;
			dev_warn(dev,
			"%s: Trimming num of supported FW ID's array to %d\n",
				__func__, pdata->firmware_id_num);
		}
		ret = of_property_read_u32_array(np,
						 "firmware_id",
						 pdata->firmware_id,
						 pdata->firmware_id_num);
		if (ret) {
			dev_err(dev, "%s: Could not read FW ID array\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}
		dev_info(dev,
			"%s: using %u supported FW ID's' from dev-tree\n",
				__func__, pdata->firmware_id_num);
		for (i = 0; i < pdata->firmware_id_num; i++)
			dev_dbg(dev, "%s: FW ID %8.8x: 0x%8.8x\n",
				__func__, i, pdata->firmware_id[i]);
	} else {
		dev_info(p->dev,
		"%s: no firmware_id def. in device-tree. setting defaults\n",
			__func__);
		p->pdata->firmware_id[0] = DBMDX_FIRMWARE_ID_DBMD7_0;
		p->pdata->firmware_id[1] = DBMDX_FIRMWARE_ID_DBMD7_1;
		p->pdata->firmware_id_num = 2;

	}

	ret = of_property_read_u32(np, "boot_options",
		&p->pdata->boot_options);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'boot_options'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no boot_options definition in device-tree.\n",
			__func__);
		p->pdata->boot_options = DBMDX_BOOT_MODE_NORMAL_BOOT;
	}

	ret = of_property_read_u32(np, "amodel_options",
		&p->pdata->amodel_options);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'amodel_options'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no amodel_options definition in device-tree.\n",
			__func__);
		p->pdata->amodel_options = DBMDX_AMODEL_DEFAULT_OPTIONS;
	}

	ret = of_property_read_u32(np, "disable_recovery",
		&p->pdata->recovery_disabled);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->recovery_disabled != 0 &&
				p->pdata->recovery_disabled != 1)) {
		dev_err(p->dev, "%s: invalid 'recovery_disabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "uart_low_speed_enabled",
		&p->pdata->uart_low_speed_enabled);
	if ((ret && ret != -EINVAL) ||
			(p->pdata->uart_low_speed_enabled != 0 &&
				p->pdata->uart_low_speed_enabled != 1)) {
		dev_err(p->dev, "%s: invalid 'uart_low_speed_enabled'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	ret = of_property_read_u32(np, "rx_ready_host_wakeup_gpio",
		&p->pdata->rx_ready_host_wakeup_gpio);
	if ((ret && ret != -EINVAL)) {
		dev_err(p->dev, "%s: invalid 'rx_ready_host_wakeup_gpio'\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret ==  -EINVAL) {
		dev_info(p->dev,
			"%s: no rx_ready_host_wakeup_gpio def. in dev-tree\n",
			__func__);
		p->pdata->rx_ready_host_wakeup_gpio =
				DBMDX_DEFAULT_RX_READY_HOST_WAKEUP_GPIO_VAL;
	} else {

		dev_info(p->dev,
		"%s: using rx_ready_host_wakeup_gpio of 0x%x from dev-tree\n",
			 __func__, p->pdata->rx_ready_host_wakeup_gpio);

	}

	if (dbmdx_of_get_clk_info(p, np, DBMDX_CLK_MASTER)) {
		dev_err(dev,
			"%s: failed to get master clock information\n",
			 __func__);
	}

	if (dbmdx_of_get_clk_info(p, np, DBMDX_CLK_CONSTANT)) {
		dev_err(dev,
			"%s: failed to get constant clock information\n",
			 __func__);
	}

	ret = dbmdx_init_cfg_reg_list_populate(p);
	if (ret) {
		dev_err(p->dev, "%s: Error reading init_cfg_reg_list\n",
			__func__);
		ret = -EINVAL;
		goto out_err;
	}

	pdata->memset_params[0] = htonl(BOOT_MEMSET_CMD_OPCODE_6);
	pdata->memset_params[1] = BOOT_MEMSET_START_ADDR;
	pdata->memset_params[2] = BOOT_MEMSET_END_ADDR;
	pdata->memset_params[3] = 0;
	pdata->memset_params[4] = htonl(BOOT_MEMSET_CMD_OPCODE_7);

	property = of_find_property(np, "memset_params", &len);
	if (property) {
		if (len != ((MEMSET_CMDS_SIZE - 2) * 4)) {
			dev_err(dev,
			"%s: Invalid memset params\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}

		ret = of_property_read_u32_array(np, "memset_params",
						(u32 *)&pdata->memset_params[1],
						(len / sizeof(u32)));
		if (ret) {
			dev_err(dev, "%s: Could not read memset params\n",
				__func__);
			ret = -EINVAL;
			goto out_err;
		}

		if (pdata->memset_params[1] < BOOT_MEMSET_START_ADDR)
			pdata->memset_params[1] = BOOT_MEMSET_START_ADDR;
		if (pdata->memset_params[2] > BOOT_MEMSET_END_ADDR)
			pdata->memset_params[2] = BOOT_MEMSET_END_ADDR;

		dev_info(dev,
			"%s: using memset_params from dev-tree\n",
				__func__);

		for (i = 0; i < MEMSET_CMDS_SIZE; i++)
			dev_dbg(dev, "%s: memset_params:%8.8x\n",
				__func__, pdata->memset_params[i]);
	} else {
		dev_info(dev, "%s using default memset_params\n", __func__);
	}

	ret = of_property_read_u32(np, "platform_id",
		&pdata->platform_id);
	if (ret && ret != -EINVAL) {
		dev_err(p->dev, "%s: invalid 'platform id'\n", __func__);
		ret = -EINVAL;
		goto out_err;
	} else if (ret == -EINVAL) {
		dev_info(p->dev,
			"%s: no platform id definition in device-tree.\n",
			__func__);
		pdata->platform_id = -1;
	}
	dev_info(dev, "%s platform id:%u\n", __func__, pdata->platform_id);

	return 0;
out_err:
	return ret;
}

int dbmdx_read_config_from_devtree(struct dbmdx_private *p)
{
	struct device_node *np = p->dev->of_node;
	struct dbmdx_platform_data *pdata;
	int ret;

	if (!np) {
		dev_err(p->dev, "%s: error no devicetree entry\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	pdata = kzalloc(sizeof(struct dbmdx_platform_data), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto err;
	}

	p->pdata = pdata;

	ret = dbmdx_get_devtree_pdata(p);
	if (ret) {
		dev_err(p->dev, "%s: failed to read device tree data\n",
			__func__);
		goto out_err_free_pdata;
	}


	ret = dbmdx_get_gpio_devtree(p);
	if (ret < 0) {
		dev_err(p->dev,
			"%s: error getting gpio information from devicetree\n",
			__func__);
		ret = -ENODEV;
		goto out_err_free_pdata;
	}

	return 0;

out_err_free_pdata:
	kfree(pdata);
	p->pdata = NULL;
err:
	return ret;
}

int dbmdx_deinit_devtree_config(struct dbmdx_private *p)
{
	if (p && p->pdata) {
		kfree(p->pdata);
		p->pdata = NULL;
	}

	return 0;
}
