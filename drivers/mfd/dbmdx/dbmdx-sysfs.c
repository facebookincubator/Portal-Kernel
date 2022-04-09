/*
 * dbmdx-sysfs.c -- DSPG DBMDX SysFS Interface
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
#define LOG_EVENTS_ANALYTICS	2

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/version.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-fw-cmd-interface.h>
#include <linux/mfd/dbmdx/dbmdx-common-operations.h>
#include <linux/mfd/dbmdx/dbmdx-sysfs.h>
#include <linux/mfd/dbmdx/dbmdx-usecases.h>


/* ------------------------------------------------------------------------
 * sysfs attributes
 * ------------------------------------------------------------------------ */
static ssize_t dbmdx_fw_ver_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
#if 0
	return dbmdx_reg_show(dev, DBMDX_VA_GET_FW_VER, attr, buf);
#endif
	return snprintf(buf, PAGE_SIZE,
		"Unknown firmware (%d) loaded\n", p->active_fw);
}

static ssize_t dbmdx_reboot_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int primary = 0;
	int ret = 0;

	if (!strncmp(buf, "primary", min_t(int, size, 7)))
		primary = 1;
	else if (!strncmp(buf, "help", min_t(int, size, 4))) {
		dev_info(p->dev,
			"%s: Commands: primary | help\n",
			__func__);
		return size;
	}

	if (!primary) {
		dev_warn(p->dev, "%s: not valid mode requested: %s\n",
			__func__, buf);
		return size;
	}

	dev_info(p->dev, "%s: Reboot Start\n", __func__);

	if (p && p->device_ready && p->primary_flags.active_usecase) {
		bool is_uc_active = false;
		struct usecase_config *active_usecase =
					p->primary_flags.active_usecase;
		active_usecase->usecase_is_active(p,
					active_usecase, &is_uc_active);
		if (!ret && is_uc_active) {
			ret = active_usecase->usecase_exit(p, active_usecase);
			if (ret < 0)
				dev_err(p->dev,
					"%s: Error exiting usecase %s\n",
					__func__,
					active_usecase->usecase_name);
		}
		p->primary_flags.active_usecase = NULL;

	}
	p->wakeup_release(p);

	p->debug_mode = 0;

	ret = dbmdx_request_and_load_fw(p);
	if (ret != 0)
		return -EIO;

	return size;
}

static ssize_t dbmdx_host_speed_cfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct dbmdx_platform_data *pdata;
	char *str_p;
	char *args =  (char *)buf;
	unsigned long val;
	u32 index = 0;
	u32 type = 0;
	u32 new_value = 0;
	bool index_set = false, type_set = false, value_set = false;
	int i;

	int ret = -EINVAL;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val > 2) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "type=", 5) == 0) {
			if (strncmp(str_p+5, "cfg", 3) == 0)
				type = 0;
			else if (strncmp(str_p+5, "uart", 4) == 0)
				type = 1;
			else if (strncmp(str_p+5, "i2c", 3) == 0)
				type = 2;
			else if (strncmp(str_p+5, "spi", 3) == 0)
				type = 3;
			else {
				dev_err(p->dev, "%s: invalid type\n",
					__func__);
				ret = -EINVAL;
				goto print_usage;
			}
			type_set = true;
			continue;
		}
		if (strncmp(str_p, "value=", 6) == 0) {
			ret = kstrtoul((str_p+6), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad value\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}

			new_value = (u32)val;
			value_set = true;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!type_set) {
		dev_err(p->dev, "%s: type is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!value_set) {
		dev_err(p->dev, "%s: value is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	p->lock(p);

	if (type == 0) {
		p->pdata->host_speed_cfg[index].cfg = new_value;
		dev_info(p->dev, "%s: host_speed_cfg[%u].cfg was set to %8.8x\n",
			__func__, index, new_value);
	} else if (type == 1) {
		p->pdata->host_speed_cfg[index].uart_baud = new_value;
		dev_info(p->dev, "%s: host_speed_cfg[%u].uart_baud was set to %u\n",
			__func__, index, new_value);
	} else if (type == 2) {
		p->pdata->host_speed_cfg[index].i2c_rate = new_value;
		dev_info(p->dev, "%s: host_speed_cfg[%u].i2c_rate was set to %u\n",
			__func__, index, new_value);
	} else if (type == 3) {
		p->pdata->host_speed_cfg[index].spi_rate = new_value;
		dev_info(p->dev, "%s: host_speed_cfg[%u].spi_rate was set to %u\n",
			__func__, index, new_value);
	}

	p->unlock(p);

	for (i = 0; i < DBMDX_NR_OF_SPEEDS; i++)
		dev_info(dev, "%s: Host speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			__func__,
			i,
			pdata->host_speed_cfg[i].cfg,
			pdata->host_speed_cfg[i].uart_baud,
			pdata->host_speed_cfg[i].i2c_rate,
			pdata->host_speed_cfg[i].spi_rate);

	return size;
print_usage:
	dev_info(p->dev,
		"%s: Usage: index=[0/1/2] type=[cfg/uart/i2c/spi] value=newval\n",
		__func__);
	return ret;
}

static ssize_t dbmdx_host_speed_cfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	for (i = 0; i < DBMDX_NR_OF_SPEEDS; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tHost speed cfg %8.8x: 0x%8.8x %u %u %u\n",
			i,
			pdata->host_speed_cfg[i].cfg,
			pdata->host_speed_cfg[i].uart_baud,
			pdata->host_speed_cfg[i].i2c_rate,
			pdata->host_speed_cfg[i].spi_rate);

	return off;
}

static ssize_t dbmdx_primary_boot_options_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "Boot options: 0x%x\n",
		p->pdata->boot_options);

	return ret;
}

static ssize_t dbmdx_primary_boot_options_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->pdata->boot_options = val;

	return size;
}

static ssize_t dbmdx_debug_traffic_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "Debug Traffic Verbosity: %d\n",
		p->dump_traffic);

	return ret;
}

static ssize_t dbmdx_debug_traffic_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->dump_traffic = val;

	return size;
}

static ssize_t dbmdx_hw_usecase_stop_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "HW Usecase Stop: %d\n",
		p->hw_usecase_stop_enabled);

	return ret;
}

static ssize_t dbmdx_hw_usecase_stop_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	 p->hw_usecase_stop_enabled = val;

	return size;
}
static ssize_t dbmdx_dump_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;
	int i;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMDX Driver Ver:\t%s\n", DRIVER_VERSION);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======Interfaces Dump======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tMulti Interface Support:\t%s\n",
			pdata->multi_interface_support ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tNumber of supported interfaces:\t%d\n",
				p->nr_of_interfaces);

	for (i = 0; i < p->nr_of_interfaces; i++) {
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tInterface #%d:\n",
				i);
		if (p->interfaces[i]->dump)
			off += p->interfaces[i]->dump(p,
				buf + off);
	}

	if (p->pdata->multi_interface_support) {
		off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======Interfaces======\n");

		off += snprintf(buf + off, PAGE_SIZE - off,
		"\tPreboot:\t%d\n\tBoot:\t%d\n\tCommand:\t%d\n\tDebug:\t%d\n",
			pdata->interfaces[DBMDX_PREBOOT_INTERFACE],
			pdata->interfaces[DBMDX_BOOT_INTERFACE],
			pdata->interfaces[DBMDX_CMD_INTERFACE],
			pdata->interfaces[DBMDX_DEBUG_INTERFACE]);
	}


	/* check for features */
	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tPr. Firmware name: %s\n",
				pdata->primary_firmware_name);

	for (i = 0; i < pdata->num_of_fw_addons; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tUsing FW Addon Name %d:\t%s\n",
				i, pdata->add_on_fw_names[i]);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Common Configuration======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tHW Revision 0x%8x\n", pdata->hw_rev);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tChip Revision 0x%8x\n", p->chip_revision);

	for (i = 0; i < DBMDX_NR_OF_SPEEDS; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tHost speed cfg %8.8x: 0x%8.8x %u %u %u\n",
				i,
				pdata->host_speed_cfg[i].cfg,
				pdata->host_speed_cfg[i].uart_baud,
				pdata->host_speed_cfg[i].i2c_rate,
				pdata->host_speed_cfg[i].spi_rate);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Init Config Reg List======\n");
	for (i = 0; i < p->pdata->init_cfg_reg_list_items; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tindex<0x%x>:\t cmd<0x%2x> pr<0x%x> id<0x%x> reg<0x%02x> val1<0x%08x> val2<0x%08x> val3<0x%08x>\n",
				i,
				p->pdata->init_cfg_reg_list[i].cmd,
				p->pdata->init_cfg_reg_list[i].priority,
				p->pdata->init_cfg_reg_list[i].id,
				p->pdata->init_cfg_reg_list[i].reg_num,
				p->pdata->init_cfg_reg_list[i].val1,
				p->pdata->init_cfg_reg_list[i].val2,
				p->pdata->init_cfg_reg_list[i].val3);


	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tmaster-clk-rate of %dHZ\n",
				(int)(p->clk_get_rate(p, DBMDX_CLK_MASTER)));

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tuart_low_speed_enabled %d\n",
					pdata->uart_low_speed_enabled);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tBoot options 0x%8x\n", pdata->boot_options);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tAmodel options 0x%8x\n", pdata->amodel_options);

	for (i = 0; i < p->pdata->firmware_id_num; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
			"\tPr. Firmware_id %d: 0x%8x\n",
			i, p->pdata->firmware_id[i]);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\t=======GPIO======\n");

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tReset GPIO: 0x%8x\n", pdata->gpio_reset);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tWakeup GPIO: 0x%8x\n", pdata->gpio_wakeup);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tHost Wakeup GPIO: 0x%8x\n",
			p->pdata->gpio_host_wakeup);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tRX READY GPIO: 0x%8x\n", pdata->gpio_rx_ready);

	off += snprintf(buf + off, PAGE_SIZE - off,
			"\tRX READY HOST WAKEUP GPIO CONF: 0x%8x\n",
				pdata->rx_ready_host_wakeup_gpio);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\twakeup_disabled %d\n",
					pdata->wakeup_disabled);

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tuse_gpio_for_wakeup %d\n",
					pdata->use_gpio_for_wakeup);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tsend_wakeup_seq %d\n",
		 pdata->send_wakeup_seq);

	off += snprintf(buf + off, PAGE_SIZE - off, "\twakeup_set_value %d\n",
		 pdata->wakeup_set_value);



	return off;
}

static ssize_t dbmdx_dump_current_state(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	struct dbmdx_platform_data *pdata;

	if (!p)
		return -EAGAIN;

	pdata = p->pdata;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tDBMDX Driver Ver:\t%s\n", DRIVER_VERSION);

	off += snprintf(buf + off, PAGE_SIZE - off, "\tActive firmware:\t%s\n",
		p->active_fw == DBMDX_FW_PRIMARY ?
		"PRIMARY" : "PRE_BOOT");

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\tActive Interface :\t%s\n",
				p->active_interface == DBMDX_INTERFACE_I2C ?
				"I2C" :
				p->active_interface ==
					DBMDX_INTERFACE_SPI ?
				"SPI" :
				p->active_interface ==
					DBMDX_INTERFACE_UART ?
				"UART" : "NONE");

	off += snprintf(buf + off, PAGE_SIZE - off, "\tDevice Ready:\t%s\n",
				p->device_ready ? "Yes" : "No");

	off += snprintf(buf + off, PAGE_SIZE - off, "\tFirmware Alive:\t%s\n",
				p->fw_alive ? "Yes" : "No");

	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tDebug Mode:\t%s\n",
			p->debug_mode ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tSleep disabled:\t%s\n",
			p->sleep_disabled ? "ON" : "OFF");
	off += snprintf(buf + off, PAGE_SIZE - off,
					"\tRecovery Disabled:\t%s\n",
			p->pdata->recovery_disabled ? "ON" : "OFF");

	return off;
}

static ssize_t dbmdx_rxsize_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%lu\n", p->rxsize);
}

static ssize_t dbmdx_rxsize_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (val % 16 != 0)
		return -EINVAL;

	p->rxsize = val;

	return size;
}

static ssize_t dbmdx_rsize_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	return snprintf(buf, PAGE_SIZE, "%u\n",
				p->chip->get_read_chunk_size(p));

}

static ssize_t dbmdx_rsize_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = p->chip->set_read_chunk_size(p, (u32)val);
	p->unlock(p);

	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t dbmdx_wsize_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;


	return snprintf(buf, PAGE_SIZE, "%u\n",
			p->chip->get_write_chunk_size(p));
}

static ssize_t dbmdx_wsize_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret;
	unsigned long val;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	p->lock(p);
	ret = p->chip->set_write_chunk_size(p, (u32)val);
	p->unlock(p);

	if (ret < 0)
		return -EINVAL;

	return size;
}

static ssize_t dbmdx_wakeup_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;
	int gpio_val;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (!p->can_wakeup(p))
		ret = snprintf(buf, PAGE_SIZE, "No WakeUp GPIO\n");
	else {
		gpio_val = gpio_get_value(p->pdata->gpio_wakeup);
		ret = snprintf(buf, PAGE_SIZE, "WakeUp GPIO: %d\n", gpio_val);
	}

	return ret;
}

static ssize_t dbmdx_wakeup_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
#if 0
	struct dbmdx_private *p = dev_get_drvdata(dev);
	dbmdx_force_wake(p);
#endif
	return size;
}

static ssize_t dbmdx_hw_revision_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int ret;

	if (!p)
		return -EAGAIN;

	ret = snprintf(buf, PAGE_SIZE, "Hardware Revision: 0x%x\n",
		p->pdata->hw_rev);

	return ret;
}

static ssize_t dbmdx_hw_revision_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	if (!p)
		return -EAGAIN;

	p->pdata->hw_rev = (u32)val;

	dev_dbg(p->dev, "%s: HW revision was set to 0x%8x\n",
		__func__, p->pdata->hw_rev);

	return size;
}

static ssize_t dbmdx_init_cfg_reg_list_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	int off = 0;
	int i;

	if (!p)
		return -EAGAIN;

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t=======Init Config Reg List======\n");
	for (i = 0; i < p->pdata->init_cfg_reg_list_items; i++)
		off += snprintf(buf + off, PAGE_SIZE - off,
				"\tindex<0x%x>:\t cmd<0x%2x> pr<0x%x> id<0x%x> reg<0x%02x> val1<0x%08x> val2<0x%08x> val3<0x%08x>\n",
				i,
				p->pdata->init_cfg_reg_list[i].cmd,
				p->pdata->init_cfg_reg_list[i].priority,
				p->pdata->init_cfg_reg_list[i].id,
				p->pdata->init_cfg_reg_list[i].reg_num,
				p->pdata->init_cfg_reg_list[i].val1,
				p->pdata->init_cfg_reg_list[i].val2,
				p->pdata->init_cfg_reg_list[i].val3);

	off += snprintf(buf + off, PAGE_SIZE - off,
				"\t\nCommands Help:\n");

	off += snprintf(buf + off, PAGE_SIZE - off, "\n\t==Commands==\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<1>:\tWRITE_REG_TYPE_16\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<2>:\tWRITE_REG_TYPE_32\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<3>:\tWRITE_REG_TYPE_64\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<4>:\tWRITE_REG_TYPE_94\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<5>:\tMsleep\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<6>:\tDummy\n");

	off += snprintf(buf + off, PAGE_SIZE - off, "\n\t==Priority==\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<0>:\tPRIORITY_RUN_ON_IDLE_ISR\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<1>:\tPRIORITY_RUN_ON_IDLE_ISR\n");

	off += snprintf(buf + off, PAGE_SIZE - off, "\n\t==ID==\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<0>:\tMSG_ID_FW\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<1>:\tMSG_ID_MAILBOX\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<2>:\tMSG_ID_RTOS\n");
	off += snprintf(buf + off, PAGE_SIZE - off,
		"\t\t<3>:\tMSG_ID_RES\n");

	return off;
}

static ssize_t dbmdx_init_cfg_reg_list_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	char *str_p;
	unsigned long val;
	int ret = size;
	char *args =  (char *)buf;
	bool index_set = false, reg_set = false, val1_set = false;
	struct fw_register fw_reg = {0};
	u32 index = 0;
	int i;

	if (!p)
		return -EAGAIN;

	/* Set defaults */
	fw_reg.cmd = WRITE_REG_TYPE_16;
	fw_reg.priority = PRIORITY_RUN_ON_IDLE_LOOP;
	fw_reg.id = MSG_ID_FW;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		dev_info(p->dev, "%s: %s %s\n", __func__, args, str_p);

		if (!*str_p)
			continue;

		if (strncmp(str_p, "index=", strlen("index=")) == 0) {
			ret = kstrtoul((str_p+strlen("index=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val >= p->pdata->init_cfg_reg_list_items) {
				dev_err(p->dev, "%s: index out of range: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			index = (u32)val;
			index_set = true;
			continue;
		}
		if (strncmp(str_p, "cmd=", strlen("cmd=")) == 0) {
			ret = kstrtoul((str_p + strlen("cmd=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad cmd\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val <  1 && val > 6) {
				dev_err(p->dev, "%s: Unsupported cmd: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.cmd = (u32)val;
			continue;
		}
		if (strncmp(str_p, "pr=", strlen("pr=")) == 0) {
			ret = kstrtoul((str_p + strlen("pr=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad priority\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if ((val != PRIORITY_RUN_ON_IDLE_ISR) &&
					(val != PRIORITY_RUN_ON_IDLE_LOOP)) {
				dev_err(p->dev,
					"%s: Unsupported priority: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.priority = (u32)val;
			continue;
		}
		if (strncmp(str_p, "id=", strlen("id=")) == 0) {
			ret = kstrtoul((str_p + strlen("id=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad id\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val > 3) {
				dev_err(p->dev, "%s: Unsupported id: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.id = (u32)val;
			continue;
		}
		if (strncmp(str_p, "reg=", strlen("reg=")) == 0) {
			ret = kstrtoul((str_p + strlen("reg=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad reg\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.reg_num = (u32)val;
			reg_set = true;
			continue;
		}
		if (strncmp(str_p, "val1=", strlen("val1=")) == 0) {
			ret = kstrtoul((str_p + strlen("val1=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val1\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val1 = (u32)val;
			val1_set = true;
			continue;
		}
		if (strncmp(str_p, "val2=", strlen("val2=")) == 0) {
			ret = kstrtoul((str_p + strlen("val2=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val2\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val2 = (u32)val;
			continue;
		}
		if (strncmp(str_p, "val3=", strlen("val3=")) == 0) {
			ret = kstrtoul((str_p + strlen("val3=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val3\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val3 = (u32)val;
			continue;
		}
	}

	if (!index_set) {
		dev_err(p->dev, "%s: index is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!val1_set && fw_reg.cmd != WRITE_REG_DUMMY) {
		dev_err(p->dev, "%s: val1 is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!reg_set && fw_reg.cmd != WRITE_REG_MSLEEP_MSEC &&
		fw_reg.cmd != WRITE_REG_DUMMY) {
		dev_err(p->dev, "%s: reg is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	memcpy(&(p->pdata->init_cfg_reg_list[index]), &fw_reg, sizeof(fw_reg));

	for (i = 0; i < p->pdata->init_cfg_reg_list_items; i++)
		dev_info(p->dev,
			"%s: \tindex<0x%x>:\t cmd<0x%2x> pr<0x%x> id<0x%x> reg<0x%02x> val1<0x%08x> val2<0x%08x> val3<0x%08x>\n",
			__func__,
			i,
			p->pdata->init_cfg_reg_list[i].cmd,
			p->pdata->init_cfg_reg_list[i].priority,
			p->pdata->init_cfg_reg_list[i].id,
			p->pdata->init_cfg_reg_list[i].reg_num,
			p->pdata->init_cfg_reg_list[i].val1,
			p->pdata->init_cfg_reg_list[i].val2,
			p->pdata->init_cfg_reg_list[i].val3);

	return size;

print_usage:
	dev_err(p->dev,
		"%s: Usage: index=[0..N] cmd=[1..5] pr=[0,1] id=[0..3] reg=[] val1=[] val2=[] val3=[]\n",
		__func__);
	return ret;
}

static ssize_t dbmdx_direct_reg_access_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	char *str_p;
	unsigned long val;
	int ret = size;
	char *args =  (char *)buf;
	bool reg_set = false, val1_set = false;
	struct fw_register fw_reg = {0};
	u64 read_val = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	/* Set defaults */
	fw_reg.cmd = WRITE_REG_TYPE_16;
	fw_reg.priority = PRIORITY_RUN_ON_IDLE_LOOP;
	fw_reg.id = MSG_ID_FW;

	while ((str_p = strsep(&args, " \t")) != NULL) {

		if (!*str_p)
			continue;

		if (strncmp(str_p, "cmd=", strlen("cmd=")) == 0) {
			ret = kstrtoul((str_p + strlen("cmd=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad cmd\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val <  WRITE_REG_TYPE_16 &&
					val > READ_REG_TYPE_64_INDEX) {
				dev_err(p->dev, "%s: Unsupported cmd: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.cmd = (u32)val;
			continue;
		}
		if (strncmp(str_p, "pr=", strlen("pr=")) == 0) {
			ret = kstrtoul((str_p + strlen("pr=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad priority\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if ((val != PRIORITY_RUN_ON_IDLE_ISR) &&
					(val != PRIORITY_RUN_ON_IDLE_LOOP)) {
				dev_err(p->dev,
					"%s: Unsupported priority: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.priority = (u32)val;
			continue;
		}
		if (strncmp(str_p, "id=", strlen("id=")) == 0) {
			ret = kstrtoul((str_p + strlen("id=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad id\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			} else if (val > 4) {
				dev_err(p->dev, "%s: Unsupported id: %d\n",
					__func__, (int)val);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.id = (u32)val;
			continue;
		}
		if (strncmp(str_p, "reg=", strlen("reg=")) == 0) {
			ret = kstrtoul((str_p + strlen("reg=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad reg\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.reg_num = (u32)val;
			reg_set = true;
			continue;
		}
		if (strncmp(str_p, "val1=", strlen("val1=")) == 0) {
			ret = kstrtoul((str_p + strlen("val1=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val1\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val1 = (u32)val;
			val1_set = true;
			continue;
		}
		if (strncmp(str_p, "val2=", strlen("val2=")) == 0) {
			ret = kstrtoul((str_p + strlen("val2=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val2\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val2 = (u32)val;
			continue;
		}
		if (strncmp(str_p, "val3=", strlen("val3=")) == 0) {
			ret = kstrtoul((str_p + strlen("val3=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad val3\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val3 = (u32)val;
			continue;
		}
		if (strncmp(str_p, "index=", strlen("index=")) == 0) {
			ret = kstrtoul((str_p + strlen("index=")), 0, &val);
			if (ret) {
				dev_err(p->dev, "%s: bad index\n", __func__);
				ret = -EINVAL;
				goto print_usage;
			}
			fw_reg.val1 = (u32)val;
			continue;
		}

	}

	if (!val1_set && fw_reg.cmd < WRITE_REG_DUMMY) {
		dev_err(p->dev, "%s: val1 is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	} else if (!reg_set && fw_reg.cmd != WRITE_REG_MSLEEP_MSEC &&
		fw_reg.cmd != WRITE_REG_DUMMY) {
		dev_err(p->dev, "%s: reg is not set\n", __func__);
		ret = -EINVAL;
		goto print_usage;
	}

	if (fw_reg.cmd <= WRITE_REG_DUMMY) {
		dev_info(p->dev,
			"%s: Sending: cmd<0x%2x> pr<0x%x> id<0x%x> reg<0x%02x> val1<0x%08x> val2<0x%08x> val3<0x%08x>\n",
			__func__, fw_reg.cmd, fw_reg.priority, fw_reg.id,
			fw_reg.reg_num, fw_reg.val1, fw_reg.val2, fw_reg.val3);

		ret = dbmdx_write_fw_reg(p, &fw_reg);

		if (ret < 0) {
			dev_err(p->dev,	"%s: Error updating register\n",
				__func__);
			return ret;
		}
	} else {
		dev_info(p->dev,
			"%s: Reading: cmd<0x%2x> id<0x%x> reg<0x%02x> index<0x%08x>\n",
			__func__, fw_reg.cmd, fw_reg.id, fw_reg.reg_num,
			fw_reg.val1);

		ret = dbmdx_read_fw_reg(p, &fw_reg, &read_val);

		if (ret < 0) {
			dev_err(p->dev,	"%s: Error reading register\n",
				__func__);
			return ret;
		}
		dev_info(p->dev,
			"%s: Reg <0x%02x> value is:<0x%16llx>\n",
			__func__, fw_reg.reg_num, read_val);

	}

	return size;

print_usage:
	dev_err(p->dev,
		"%s: Usage: cmd=[1..5] pr=[0,1] id=[0..4] reg=[] val1=[] val2=[] val3=[]\n",
		__func__);
	return ret;
}



static ssize_t dbmdx_usecase_name_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n",
		p->primary_flags.user_selected_usecase == NULL ? "NONE" :
		p->primary_flags.user_selected_usecase->usecase_name);
}

static ssize_t dbmdx_usecase_name_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int ret = 0;
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct usecase_config *cur_usecase = NULL;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	dev_dbg(p->dev, "%s: usecase name - %s\n", __func__, buf);

	ret = find_usecase_by_name(p, buf, &cur_usecase);

	if (ret < 0) {
		dev_err(p->dev,
			"%s: Unsupported usecase %s\n", __func__, buf);
		ret = -EINVAL;
		goto out;
	}

	p->primary_flags.user_selected_usecase = cur_usecase;

out:
	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_usecase_params_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct usecase_config *cur_usecase =
				p->primary_flags.user_selected_usecase;
	int off = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (cur_usecase == NULL) {
		dev_err(p->dev, "%s: No user selected usecase\n", __func__);
		off += snprintf(buf, PAGE_SIZE, "No user selected usecase\n");
		return off;
	}

	off = cur_usecase->usecase_dump(p, cur_usecase,	buf, PAGE_SIZE);

	return off;

}


static ssize_t dbmdx_usecase_params_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int ret;
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct usecase_config *cur_usecase =
				p->primary_flags.user_selected_usecase;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (cur_usecase == NULL) {
		dev_err(p->dev, "%s: No usecase was selected\n", __func__);
		return -EINVAL;
	}

	ret = cur_usecase->usecase_set_params_from_str(p, cur_usecase,
								buf, size);
	if (ret < 0)
		dev_err(p->dev, "%s: Error configuring usecase %s\n", __func__,
			cur_usecase->usecase_name);

	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_active_usecase_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	struct usecase_config *cur_usecase =
				p->primary_flags.active_usecase;
	int off = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	if (cur_usecase == NULL) {
		dev_info(p->dev, "%s: No active usecase - IDLE\n", __func__);
		off += snprintf(buf, PAGE_SIZE, "IDLE Usecase\n");
		return off;
	}

	off = cur_usecase->usecase_dump(p, cur_usecase,	buf, PAGE_SIZE);

	return off;

}

static ssize_t dbmdx_active_usecase_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	struct usecase_config *cur_sel_usecase;
	struct usecase_config *active_usecase;
	int ret = 0;
	int start = 0;
	int stop = 0;
	int reset = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	cur_sel_usecase = p->primary_flags.user_selected_usecase;
	active_usecase = p->primary_flags.active_usecase;

	if  (!strncmp(buf, "start", min_t(int, size, 5))) {
		start = 1;
	} else if  (!strncmp(buf, "stop", min_t(int, size, 4))) {
		stop = 1;
	} else if  (!strncmp(buf, "reset", min_t(int, size, 5))) {
		reset = 1;
	} else if (!strncmp(buf, "help", min_t(int, size, 4))) {
		dev_info(p->dev,
			"%s: Commands: start | stop  help\n",
			__func__);
		return size;
	}

	if (start) {
		if (cur_sel_usecase == NULL) {
			dev_err(p->dev, "%s: No usecase was selected\n",
								__func__);
			return -EINVAL;
		}
		ret = switch_to_usecase(p, cur_sel_usecase);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error starting usecase %s\n",
				__func__, cur_sel_usecase->usecase_name);
			return ret;
		}
	} else if (stop) {
		ret = switch_to_usecase(p, NULL);
		if (ret < 0) {
			dev_err(p->dev,	"%s: Error stopping usecase\n",
				__func__);
			return ret;
		}
	} else if (reset) {
		if (cur_sel_usecase == NULL) {
			dev_err(p->dev, "%s: No usecase was selected\n",
								__func__);
			return -EINVAL;
		} else if (active_usecase == cur_sel_usecase) {
			ret = active_usecase->usecase_exit(p, active_usecase);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error exiting usecase %s\n",
					__func__, active_usecase->usecase_name);
			}
			ret = cur_sel_usecase->usecase_reset(p,
							active_usecase, true);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error reseting usecase %s\n",
				__func__, cur_sel_usecase->usecase_name);
			}
			p->primary_flags.active_usecase = NULL;
		} else {
			ret = cur_sel_usecase->usecase_reset(p,
							cur_sel_usecase, true);
			if (ret < 0) {
				dev_err(p->dev,
					"%s: Error reseting usecase %s\n",
				__func__, cur_sel_usecase->usecase_name);
			}
		}

		if (ret < 0)
			return ret;
	}

	return size;
}

static ssize_t dbmdx_usecase_manager_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);
	unsigned long val;
	int ret = 0;

	if (!p)
		return -EAGAIN;

	if (!p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EAGAIN;
	}

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return -EINVAL;

	ret = dbmdx_usecase_manager(p, (u32)val);

	return ret < 0 ? ret : size;
}

static ssize_t dbmdx_fw_alive_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dbmdx_private *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,
		"%s\n", p->fw_alive ? "1" : "0");
}
static ssize_t dbmdx_crash_fw_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	u32 val;
	int ret = 0;
	struct dbmdx_private *p = dev_get_drvdata(dev);

	dev_info(p->dev, "%s:manual crash firmware triggered\n", __func__);
	if (!kstrtouint(buf, 0, &val) && val) {
		ret = dbmdx_force_crash_fw(dev, val == LOG_EVENTS_ANALYTICS
						? false : true);
		if (!ret)
			return size;
	}

	return -EIO;
}

static DEVICE_ATTR(fwver, S_IRUGO,
		   dbmdx_fw_ver_show, NULL);
static DEVICE_ATTR(dump,  S_IRUGO,
		   dbmdx_dump_state, NULL);
static DEVICE_ATTR(dump_cur_state,  S_IRUGO,
		   dbmdx_dump_current_state, NULL);
static DEVICE_ATTR(reboot, S_IWUSR,
		   NULL, dbmdx_reboot_store);
static DEVICE_ATTR(host_speed_cfg, S_IRUGO | S_IWUSR,
		   dbmdx_host_speed_cfg_show, dbmdx_host_speed_cfg_store);
static DEVICE_ATTR(rsize, S_IRUGO | S_IWUSR,
		   dbmdx_rsize_show, dbmdx_rsize_store);
static DEVICE_ATTR(rxsize, S_IRUGO | S_IWUSR,
		   dbmdx_rxsize_show, dbmdx_rxsize_store);
static DEVICE_ATTR(wsize, S_IRUGO | S_IWUSR,
		   dbmdx_wsize_show, dbmdx_wsize_store);
static DEVICE_ATTR(wakeup, S_IRUGO | S_IWUSR,
		   dbmdx_wakeup_show, dbmdx_wakeup_store);
static DEVICE_ATTR(primary_boot_options, S_IRUGO | S_IWUSR,
		   dbmdx_primary_boot_options_show,
		   dbmdx_primary_boot_options_store);
static DEVICE_ATTR(debug_traffic, S_IRUGO | S_IWUSR,
		   dbmdx_debug_traffic_show,
		   dbmdx_debug_traffic_store);
static DEVICE_ATTR(hw_usecase_stop, S_IRUGO | S_IWUSR,
		   dbmdx_hw_usecase_stop_show,
		   dbmdx_hw_usecase_stop_store);
static DEVICE_ATTR(usecase_name, S_IRUGO | S_IWUSR,
		   dbmdx_usecase_name_show,
		   dbmdx_usecase_name_store);
static DEVICE_ATTR(usecase_params, S_IRUGO | S_IWUSR,
		   dbmdx_usecase_params_show,
		   dbmdx_usecase_params_store);
static DEVICE_ATTR(active_usecase, S_IRUGO | S_IWUSR,
		   dbmdx_active_usecase_show,
		   dbmdx_active_usecase_store);
static DEVICE_ATTR(usecase_manager, S_IWUSR,
		   NULL, dbmdx_usecase_manager_store);
static DEVICE_ATTR(hw_revision, S_IRUGO | S_IWUSR,
		   dbmdx_hw_revision_show,
		   dbmdx_hw_revision_store);
static DEVICE_ATTR(init_cfg_reg_list, S_IRUGO | S_IWUSR,
		   dbmdx_init_cfg_reg_list_show, dbmdx_init_cfg_reg_list_store);
static DEVICE_ATTR(direct_reg_access, S_IWUSR,
		   NULL, dbmdx_direct_reg_access_store);
static DEVICE_ATTR(fw_alive, S_IRUGO,
		   dbmdx_fw_alive_show, NULL);
static DEVICE_ATTR(crash_fw, S_IWUSR,
		   NULL, dbmdx_crash_fw_store);

static struct attribute *dbmdx_common_attributes[] = {
	&dev_attr_fwver.attr,
	&dev_attr_dump.attr,
	&dev_attr_dump_cur_state.attr,
	&dev_attr_reboot.attr,
	&dev_attr_host_speed_cfg.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_primary_boot_options.attr,
	&dev_attr_rsize.attr,
	&dev_attr_rxsize.attr,
	&dev_attr_wsize.attr,
	&dev_attr_debug_traffic.attr,
	&dev_attr_usecase_name.attr,
	&dev_attr_usecase_params.attr,
	&dev_attr_active_usecase.attr,
	&dev_attr_usecase_manager.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_hw_usecase_stop.attr,
	&dev_attr_init_cfg_reg_list.attr,
	&dev_attr_direct_reg_access.attr,
	&dev_attr_fw_alive.attr,
	&dev_attr_crash_fw.attr,
	NULL,
};


static const struct attribute_group dbmdx_common_attribute_group = {
	.attrs = dbmdx_common_attributes,
};


int dbmdx_init_common_sysfs_group(struct dbmdx_private *p)
{
	int ret;
	ret = sysfs_create_group(&p->dbmdx_dev->kobj,
				 &dbmdx_common_attribute_group);
	if (ret) {
		dev_err(p->dbmdx_dev, "%s: failed to create sysfs group\n",
			__func__);
		return ret;
	}

	return 0;
}

int dbmdx_remove_common_sysfs_group(struct dbmdx_private *p)
{
	sysfs_remove_group(&p->dev->kobj, &dbmdx_common_attribute_group);
	return 0;
}
