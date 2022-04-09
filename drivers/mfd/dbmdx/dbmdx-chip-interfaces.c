/*
 * dbmdx-chip-interfaces.c - DSPG DBMDX Chip Interfaces API
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/module.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#ifdef CONFIG_OF_I2C
#include <linux/of_i2c.h>
#endif /* CONFIG_OF_I2C */
#endif /* CONFIG_OF */
#include <linux/vmalloc.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <linux/mfd/dbmdx/dbmdx-interface.h>
#include <linux/mfd/dbmdx/dbmdx-customer.h>
#include <linux/mfd/dbmdx/dbmdx-chip-interfaces.h>

#ifdef CONFIG_OF

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned i2c_client device */
struct platform_device *of_find_platform_device_by_node(
	struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, node,
			      of_dev_node_match);
	if (!dev)
		return NULL;

	return container_of(dev, struct platform_device, dev);
}

static struct spi_device *of_find_spi_device_by_node(struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL, node, of_dev_node_match);

	if (!dev)
		return NULL;

	return to_spi_device(dev);
}

int dbmdx_find_chip_interface(struct device_node *np,
			struct chip_interface **chip,
			enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = of_find_i2c_device_by_node(np);
	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
	}

	uart_client = of_find_platform_device_by_node(np);
	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
	}

	spi_dev = of_find_spi_device_by_node(np);
	if (spi_dev) {
		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
	}

	*chip = c;

	return c ? 0 : -EINVAL;
}

int dbmdx_get_fw_interfaces(struct dbmdx_private *p,
				   const char *tag,
				   int *iarray)
{
	struct device_node *np = p->dev->of_node;
	struct property *property;
	int ret, i, nr_interfaces = 0;

	for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
		iarray[i] = -1;

	/* If multiinterface is not supported just set all interfaces to 0 */
	if (!p->pdata->multi_interface_support) {
		for (i = 0; i < DBMDX_MAX_INTERFACES; i++)
			iarray[i] = 0;
		return 0;
	}

	property = of_find_property(np, tag, &nr_interfaces);
	if (!property) {
		dev_err(p->dev,
			"%s: no valid %s entry in devicetree\n",
			__func__, tag);
		return -EINVAL;
	}
	nr_interfaces /= sizeof(u32);
	if (nr_interfaces > DBMDX_MAX_INTERFACES ||
	    nr_interfaces == 0) {
		dev_err(p->dev,
			"%s: %s min entries is %d, max is %d\n",
			__func__, tag, 1, DBMDX_MAX_INTERFACES);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, tag, iarray, nr_interfaces);
	if (ret) {
		dev_err(p->dev,
			"%s: could not read %s\n", __func__, tag);
		return -EIO;
	}

	dev_info(p->dev,
		"%s: %s uses %d interfaces from device-tree\n",
		__func__, tag, nr_interfaces);

	for (i = 0; i < DBMDX_MAX_INTERFACES; i++) {
		/* make sure all interfaces have a valid index */
		if (iarray[i] == -1)
			iarray[i] = iarray[0];
		dev_dbg(p->dev, "%s: interface %2.2x: 0x%2.2x\n",
			__func__, i, iarray[i]);
	}

	return 0;
}

static int dbmdx_interface_probe_single(struct dbmdx_private *p)
{
	int ret = 0;
	struct device_node *np = p->dev->of_node;
	struct device_node *interface_np;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_SPI;

	interface_np = of_parse_phandle(np, "cmd-interface", 0);
	if (!interface_np) {
		dev_err(p->dev, "%s: invalid command interface node\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}
#if 0
	ret = dbmdx_find_chip_interface(interface_np, &chip, &active_interface);
	if (ret == -EPROBE_DEFER)
		goto out;
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid interface phandle\n", __func__);
		goto out;
	}
#endif
	p->nr_of_interfaces = 1;

	p->interfaces = kzalloc(sizeof(struct chip_interface *), GFP_KERNEL);

	if (!(p->interfaces)) {
		dev_err(p->dev, "%s: no memory for interfaces\n", __func__);
		goto out;
	}

	p->interface_types = kzalloc(sizeof(enum dbmdx_bus_interface),
				GFP_KERNEL);

	if (!(p->interface_types)) {
		dev_err(p->dev, "%s: no memory for interface types\n",
			__func__);
		goto out;
	}

	p->interfaces[0] = p->chip;
	p->interface_types[0] = active_interface;

	p->active_interface = active_interface;

	return 0;
out:
	kfree(p->interfaces);
	kfree(p->interface_types);
	return ret;
}

static int dbmdx_interface_probe_multi(struct dbmdx_private *p)
{
	int ret = 0;
	unsigned int nr_interfaces = 0;
	int i = 0;
	struct device_node *np = p->dev->of_node;
	struct device_node *interface_np;
	struct chip_interface **interfaces = NULL;
	enum dbmdx_bus_interface	*interface_types = NULL;
	enum dbmdx_bus_interface	*new_interface_types;
	struct chip_interface **new_interfaces;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_SPI;

	do {
		interface_np = of_parse_phandle(np, "cd-interfaces", i++);
		if (!interface_np)
			continue;
#if 0
		ret = dbmdx_find_chip_interface(interface_np, &chip,
							&active_interface);
		if (ret == -EPROBE_DEFER)
			goto out;
		if (ret != 0) {
			dev_err(p->dev, "%s: invalid interface phandle\n",
				__func__);
			goto out;
		}
#endif
		new_interfaces = krealloc(interfaces,
					  sizeof(struct chip_interface *) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interfaces) {
			dev_err(p->dev, "%s: no memory for interfaces\n",
				__func__);
			goto out;
		}

		new_interface_types = krealloc(interface_types,
					  sizeof(enum dbmdx_bus_interface) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interface_types) {
			dev_err(p->dev, "%s: no memory for interface types\n",
				__func__);
			goto out;
		}

		interfaces = new_interfaces;
		interfaces[nr_interfaces] = p->chip;
		interface_types = new_interface_types;
		interface_types[nr_interfaces] = active_interface;
		nr_interfaces++;

	} while (interface_np);

	if (!nr_interfaces) {
		dev_err(p->dev, "%s: invalid nr of interfaces\n",
			__func__);
		ret = -EINVAL;
		goto out_free_interfaces;
	}

	p->nr_of_interfaces = nr_interfaces;
	p->interfaces = interfaces;
	p->interface_types = interface_types;

	dev_info(p->dev, "%s: found %u interfaces\n", __func__, nr_interfaces);


	return 0;
out_free_interfaces:
	kfree(interfaces);
	kfree(interface_types);
out:
	return ret;
}

#else

static int dbmdx_name_match(struct device *dev, void *dev_name)
{
	struct platform_device *pdev = to_platform_device(dev);

	if (!pdev || !pdev->name)
		return 0;

	return !strcmp(pdev->name, dev_name);
}

static int dbmdx_spi_name_match(struct device *dev, void *dev_name)
{
	struct spi_device *spi_dev = to_spi_device(dev);

	if (!spi_dev || !spi_dev->modalias)
		return 0;

	return !strcmp(spi_dev->modalias, dev_name);
}

static int dbmdx_i2c_name_match(struct device *dev, void *dev_name)
{
	struct i2c_client *i2c_dev = to_i2c_client(dev);

	if (!i2c_dev || !i2c_dev->name)
		return 0;

	return !strcmp(i2c_dev->name, dev_name);
}

struct i2c_client *dbmdx_find_i2c_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&i2c_bus_type, NULL, (void *)dev_name,
					 dbmdx_i2c_name_match);

	return dev ? to_i2c_client(dev) : NULL;
}

struct spi_device *dbmdx_find_spi_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL,
				(void *)dev_name, dbmdx_spi_name_match);
	return dev ? to_spi_device(dev) : NULL;
}

struct platform_device *dbmdx_find_platform_device_by_name(const char *dev_name)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL,
				(void *)dev_name, dbmdx_name_match);
	return dev ? to_platform_device(dev) : NULL;
}

int dbmdx_find_chip_interface(struct chip_interface **chip,
				enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = dbmdx_find_i2c_device_by_name("dbmdx-i2c");

	if (!i2c_client)
		i2c_client = dbmdx_find_i2c_device_by_name("dbmd7-i2c");

	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
	}

	uart_client = dbmdx_find_platform_device_by_name("dbmdx-uart");

	if (!uart_client)
		uart_client =
			dbmdx_find_platform_device_by_name("dbmd7-uart");

	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
	}

	spi_dev = dbmdx_find_spi_device_by_name("dbmdx-spi");

	if (!spi_dev)
		spi_dev = dbmdx_find_spi_device_by_name("dbmd7-spi");

	if (spi_dev) {
		/* got spi command interface */
		dev_info(&spi_dev->dev, "%s: spi interface node %p\n",
				__func__, spi_dev);

		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
	}

	*chip = c;

	return c ? 0 : -EINVAL;
}
static int dbmdx_find_chip_interface_by_name(const char *iface_name,
				struct chip_interface **chip,
				enum dbmdx_bus_interface *active_interface)
{
	struct spi_device *spi_dev;
	struct i2c_client *i2c_client;
	struct platform_device *uart_client;
	struct chip_interface *c = NULL;

	*active_interface = DBMDX_INTERFACE_NONE;

	i2c_client = dbmdx_find_i2c_device_by_name(iface_name);

	if (i2c_client) {
		/* got I2C command interface */
		c = i2c_get_clientdata(i2c_client);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_I2C;
		goto out;
	}

	uart_client = dbmdx_find_platform_device_by_name(iface_name);

	if (uart_client) {
		/* got UART command interface */
		c = dev_get_drvdata(&uart_client->dev);
		if (!c)
			return -EPROBE_DEFER;
		*active_interface = DBMDX_INTERFACE_UART;
		goto out;
	}

	spi_dev = dbmdx_find_spi_device_by_name(iface_name);

	if (spi_dev) {
		/* got spi command interface */
		dev_info(&spi_dev->dev, "%s: spi interface node %p\n",
				__func__, spi_dev);

		/* got spi command interface */
		c = spi_get_drvdata(spi_dev);
		if (!c)
			return -EPROBE_DEFER;

		*active_interface = DBMDX_INTERFACE_SPI;
		goto out;
	}
out:
	*chip = c;

	return c ? 0 : -EINVAL;
}

static int dbmdx_interface_probe_single(struct dbmdx_private *p)
{
	int ret = 0;
	struct chip_interface *chip;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_SPI;
#if 0
	ret = dbmdx_find_chip_interface(&chip, &active_interface);
	if (ret == -EPROBE_DEFER)
		goto out;
	if (ret != 0) {
		dev_err(p->dev, "%s: invalid interface phandle\n", __func__);
		goto out;
	}
#endif
	p->nr_of_interfaces = 1;

	p->interfaces = kzalloc(sizeof(struct chip_interface *), GFP_KERNEL);

	if (!(p->interfaces)) {
		dev_err(p->dev, "%s: no memory for interfaces\n", __func__);
		goto out;
	}

	p->interface_types = kzalloc(sizeof(enum dbmdx_bus_interface),
				GFP_KERNEL);

	if (!(p->interface_types)) {
		dev_err(p->dev, "%s: no memory for interface types\n",
			__func__);
		goto out;
	}

	p->interfaces[0] = p->chip;
	p->interface_types[0] = active_interface;

	p->active_interface = active_interface;

	return 0;
out:
	kfree(p->interfaces);
	kfree(p->interface_types);
	return ret;
}

static int dbmdx_interface_probe_multi(struct dbmdx_private *p)
{
	int ret = 0;
	unsigned int nr_interfaces = 0;
	int interface_ind;
	struct chip_interface **interfaces = NULL;
	enum dbmdx_bus_interface	*interface_types = NULL;
	enum dbmdx_bus_interface	*new_interface_types;
	struct chip_interface **new_interfaces;
	enum dbmdx_bus_interface active_interface = DBMDX_INTERFACE_SPI;

	if (!p->pdata->cd_interfaces) {
		dev_err(p->dev, "%s: invalid interfaces array\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	for (interface_ind = 0; p->pdata->cd_interfaces[interface_ind];
		++interface_ind) {

		const char *interface_name =
				p->pdata->cd_interfaces[interface_ind];

		if (!interface_name)
			break;
#if 0
		ret = dbmdx_find_chip_interface_by_name(interface_name, &chip,
							&active_interface);
		if (ret == -EPROBE_DEFER)
			goto out;
		if (ret != 0) {
			dev_err(p->dev, "%s: invalid interface phandle [%s]\n",
				__func__, interface_name);
			goto out;
		}
#endif
		new_interfaces = krealloc(interfaces,
					  sizeof(struct chip_interface *) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interfaces) {
			dev_err(p->dev, "%s: no memory for interfaces\n",
				__func__);
			goto out;
		}

		new_interface_types = krealloc(interface_types,
					  sizeof(enum dbmdx_bus_interface) *
					  (nr_interfaces + 1),
					  GFP_KERNEL);
		if (!new_interface_types) {
			dev_err(p->dev, "%s: no memory for interface types\n",
				__func__);
			goto out;
		}

		interfaces = new_interfaces;
		interfaces[nr_interfaces] = p->chip;
		interface_types = new_interface_types;
		interface_types[nr_interfaces] = active_interface;
		nr_interfaces++;

	}

	if (!nr_interfaces) {
		dev_err(p->dev, "%s: invalid nr of interfaces\n",
			__func__);
		ret = -EINVAL;
		goto out_free_interfaces;
	}

	p->nr_of_interfaces = nr_interfaces;
	p->interfaces = interfaces;
	p->interface_types = interface_types;

	dev_info(p->dev, "%s: found %u interfaces\n", __func__, nr_interfaces);


	return 0;
out_free_interfaces:
	kfree(interfaces);
	kfree(interface_types);
out:
	return ret;
}

#endif /* CONFIG_OF */

int dbmdx_interface_probe(struct dbmdx_private *p)
{
	/* check for features */
	if (p->pdata->multi_interface_support)
		return dbmdx_interface_probe_multi(p);

	return dbmdx_interface_probe_single(p);
}

int dbmdx_set_active_interface(struct dbmdx_private *p,
				       int interface_idx)
{
	if (p == NULL) {
		dev_err(p->dev,
			"%s: DBMDX platform was not initialized (p==NULL)\n",
			__func__);
		return -ENODEV;
	}

	if (interface_idx < 0) {
		dev_err(p->dev,	"%s: Interface is not supported\n", __func__);
		return -EINVAL;
	}

	if (interface_idx >= p->nr_of_interfaces) {
		dev_err(p->dev,
			"%s: Invalid interface index: %d (index range[0:%d]\n",
			__func__, interface_idx, p->nr_of_interfaces - 1);
		return -EINVAL;
	}

	p->chip = p->interfaces[interface_idx];
	p->active_interface = p->interface_types[interface_idx];

	dev_info(p->dev, "%s: switched to interface#: %d\n",
		__func__, interface_idx);

	return 0;
}


