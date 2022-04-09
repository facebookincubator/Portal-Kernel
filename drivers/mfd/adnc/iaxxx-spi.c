/*
 * iaxxx-spi.c -- SPI driver for Knowles IAxxx device
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include "iaxxx.h"
#include <linux/of.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#define IAXXX_SYNC_RETRY	15

/* spi large reads are failing and currently
 * our code request max read as 8k bytes
 * which translates to 2048 words
 */
#define IAXXX_MAX_SPI_SIZE	8192
#define IAXXX_SPI_PACKET_LEN	2048
#define IAXXX_REG_LEN		4
/* Padding is required to give time FW ready for data */
#define IAXXX_REG_PADDING	12
#define IAXXX_REG_LEN_WITH_PADDING (IAXXX_REG_LEN + IAXXX_REG_PADDING)
#define IAXXX_CFG_MAX_SIZE	8
#define IAXXX_WRITE_INDEX	0
#define IAXXX_READ_INDEX	1
/**
 * Description of driver private data
 *
 * @priv: IAxxx private data
 * @spi:  spi client pointer
 */
struct iaxxx_spi_priv {
	struct iaxxx_priv priv;	/* private data */
	struct spi_device *spi;
	struct pinctrl *pinctrl;
	struct pinctrl_state *spi_cs_low;
	struct pinctrl_state *spi_cs_normal;
};

static inline struct iaxxx_spi_priv *to_spi_priv(struct iaxxx_priv *priv)
{
	return priv ? container_of(priv, struct iaxxx_spi_priv, priv) : NULL;
}

static int iaxxx_spi_read(struct spi_device *spi, void *buf, int len)
{
	int rc;

	rc = spi_read(spi, buf, len);
	if (rc < 0) {
		dev_err(&spi->dev, "spi_read() failed, rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int iaxxx_spi_write(struct spi_device *spi, const void *buf, int len)
{
	int rc;

	rc = spi_write(spi, buf, len);
	if (rc < 0) {
		dev_err(&spi->dev, "spi_write() failed, rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int iaxxx_spi_cmd(struct spi_device *spi, u32 cmd, u32 *resp)
{
	int ret;

	if (!spi) {
		pr_err("%s: NULL input pointer(s)\n", __func__);
		return -EINVAL;
	}

	cmd = cpu_to_be32(cmd);
	dev_dbg(&spi->dev, "iaxxx: cmd = 0x%08x\n", cmd);

	ret = iaxxx_spi_write(spi, &cmd, sizeof(cmd));
	if (ret) {
		dev_err(&spi->dev, "Failed to send command 0x%.08X\n", cmd);
		return ret;
	}

	if (resp) {
		usleep_range(4000, 4500);
		ret = iaxxx_spi_read(spi, resp, sizeof(*resp));
		if (ret) {
			dev_err(&spi->dev, "Failed to read command response\n");
			return ret;
		}
		*resp = be32_to_cpu(*resp);
	}

	return 0;
}

static int iaxxx_spi_write_endian(struct device *dev,
				void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;
	uint32_t align_len = len;

	/* If buffer has configuration data, no need to change endian */
	if (len <= IAXXX_CFG_MAX_SIZE)
		return len;

	if (len % 4) {
		align_len = len + (len % 4);
		dev_warn(dev, "Not aligned 32bit boundary: %d aligned len %d\n",
				len, align_len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = cpu_to_be32(data);
	}

	return align_len;
}

static int iaxxx_regmap_spi_gather_raw_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	int i = 0;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[2] = { };
	uint8_t padding[IAXXX_REG_LEN_WITH_PADDING];
	void *tmp = (void *)val;

	/* Device protocol requires address to be shifted by one bit */
	uint32_t reg_addr = (*(uint32_t *)reg);

	pr_debug("%s() Register address %x\n", __func__, reg_addr);
	reg_addr = cpu_to_be32(reg_addr >> 1);
	spi_message_init(&m);

	if (reg_len > IAXXX_REG_LEN) {
		memset(&padding, 0, IAXXX_REG_LEN_WITH_PADDING);
		memcpy(&padding, &reg_addr, IAXXX_REG_LEN);
		/* Register address */
		t[i].len = IAXXX_REG_LEN_WITH_PADDING;
		t[i].tx_buf = &padding;
		spi_message_add_tail(&t[i++], &m);
	} else {
		/* Register address */
		t[i].len = sizeof(reg_addr);
		t[i].tx_buf = &reg_addr;
		spi_message_add_tail(&t[i++], &m);
	}

	/* Data */
	val_len = iaxxx_spi_write_endian(dev, tmp, val_len);
	t[i].len = val_len;
	t[i].tx_buf = tmp;
	spi_message_add_tail(&t[i++], &m);

	return spi_sync(spi, &m);
}

static int iaxxx_regmap_spi_gather_write(void *context,
					 const void *reg, size_t reg_len,
					 const void *val, size_t val_len)
{
	int i = 0;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[2] = { };
	uint8_t padding[IAXXX_REG_LEN_WITH_PADDING];

	/* Device protocol requires address to be shifted by one bit */
	uint32_t reg_addr = be32_to_cpu(*(uint32_t *)reg);

	pr_debug("%s() Register address %x\n", __func__, reg_addr);
	reg_addr = cpu_to_be32(reg_addr >> 1);

	spi_message_init(&m);

	if (reg_len > IAXXX_REG_LEN) {
		memset(&padding, 0, IAXXX_REG_LEN_WITH_PADDING);
		memcpy(&padding, &reg_addr, IAXXX_REG_LEN);
		/* Register address */
		t[i].len = IAXXX_REG_LEN_WITH_PADDING;
		t[i].tx_buf = &padding;
		spi_message_add_tail(&t[i++], &m);
	} else {
		/* Register address */
		t[i].len = sizeof(reg_addr);
		t[i].tx_buf = &reg_addr;
		spi_message_add_tail(&t[i++], &m);
	}

	/* Data */
	t[i].len = val_len;
	t[i].tx_buf = val;
	spi_message_add_tail(&t[i++], &m);

	return spi_sync(spi, &m);
}

/* regmap bus interface - write */
static int iaxxx_regmap_spi_write(void *context, const void *data, size_t count)
{
	const void *val = data + sizeof(uint32_t);
	size_t reg_len = sizeof(uint32_t);
	size_t val_len = count - reg_len;
	int rc;

	if (WARN_ON(count <= sizeof(uint32_t))
		|| WARN_ON(val_len <= IAXXX_REG_PADDING)) {
		pr_err("%s(), Error Input param put of range\n", __func__);
		return -EINVAL;
	}
	rc = iaxxx_regmap_spi_gather_write(context,
						data, reg_len, val, val_len);
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	if (to_iaxxx_priv(context)->dump_log) {
		uint32_t words =
			(val_len - IAXXX_REG_PADDING) / sizeof(uint32_t);
		uint32_t reg_addr = be32_to_cpu(*(uint32_t *)data);
		uint32_t *writebuf =
			(uint32_t *)((uint8_t *)val + IAXXX_REG_PADDING);
		struct device *dev = context;
		int i;
		for (i = 0; i < words; i++)
			register_transac_log(dev,
					reg_addr + (sizeof(uint32_t) * i),
					cpu_to_be32(writebuf[i]), IAXXX_WRITE);
	}
#endif
	return rc;
}

/* regmap bus interface - write */
static int iaxxx_spi_raw_write(void *context,
					 const void *reg,
					 const void *val, size_t val_len)
{
	size_t reg_len = IAXXX_REG_LEN_WITH_PADDING;
#ifdef DEBUG
	int i;
	uint32_t *buff = (uint32_t *)data_buff;

	for (i = 0; i < len / sizeof(uint32_t); i++)
		pr_debug("%s() index %d %x\n", __func__, i, buff[i]);
#endif
	return iaxxx_regmap_spi_gather_raw_write(context,
			reg, reg_len, val, val_len);
}

/* regmap bus interface - read */
static int iaxxx_regmap_spi_read(void *context,
				 const void *reg, size_t reg_len,
				 void *val, size_t val_len)
{
	int i = 0;
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	struct spi_transfer t[3] = { };
	int rc;

	/* For reads, most significant bit is set after address shifted */
	uint32_t reg_addr = be32_to_cpu(*(uint32_t *)reg);

	reg_addr = cpu_to_be32(reg_addr >> 1) | 0x80;

	spi_message_init(&m);

	/* Register address */
	t[i].len = sizeof(reg_addr);
	t[i].tx_buf = &reg_addr;
	spi_message_add_tail(&t[i++], &m);

	/* Padding bits (if any) */
	if (reg_len > sizeof(uint32_t)) {
		t[i].len = reg_len - sizeof(reg_addr);
		t[i].tx_buf = (uint8_t *)reg + sizeof(reg_addr);
		spi_message_add_tail(&t[i++], &m);
	}

	/* Data */
	t[i].len = val_len;
	t[i].rx_buf = val;
	spi_message_add_tail(&t[i++], &m);

	rc = spi_sync(spi, &m);
#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	if (to_iaxxx_priv(dev)->dump_log) {
		uint32_t words = val_len / sizeof(uint32_t);
		uint32_t addr = reg_addr;
		uint32_t *readbuf;
		readbuf = (uint32_t *)val;
		for (i = 0; i < words; i++)
			register_transac_log(dev, addr + (sizeof(uint32_t) * i),
					cpu_to_be32(readbuf[i]), IAXXX_READ);
	}
#endif
	return rc;
}

static int iaxxx_spi_32bit_plat_endian(struct device *dev,
				void *data_buff, uint32_t len)
{
	int i;
	uint32_t *buff = data_buff;

	if (len % 4) {
		dev_warn(dev,
			"buffer not aligned to word boundary: %d", len);
		len -= (len % 4);
	}

	for (i = 0; i < len/sizeof(uint32_t); i++) {
		u32 data = buff[i];

		buff[i] = cpu_to_be32(data);
	}

	return len;
}

/* spi bus read will perform bulk spi data read
 * operation in bytes and we have to change the
 * register endianness and set MSB bit for FW
 * to understand this operation is for read
 * and pad 12bytes of zero to give FW setup
 * the transaction.
 */
static int iaxxx_spi_bus_read(struct device *dev,
				 uint32_t reg, size_t reg_len,
				 void *val, size_t val_len)
{
	int i = 0;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message m;
	uint32_t pad[4] = {0};
	struct spi_transfer t[3] = { };

	/* For reads, most significant bit is set after address shifted */
	uint32_t reg_addr = reg;

	reg_addr = cpu_to_be32(reg_addr >> 1) | 0x80;

	spi_message_init(&m);

	/* Register address */
	t[i].len = sizeof(reg_addr);
	t[i].tx_buf = &reg_addr;
	spi_message_add_tail(&t[i++], &m);

	/* Padding bits (if any) */
	if (reg_len > sizeof(uint32_t)) {
		t[i].len = reg_len - sizeof(reg_addr);
		t[i].tx_buf = &pad;
		spi_message_add_tail(&t[i++], &m);
	}

	/* Data */
	t[i].len = val_len;
	t[i].rx_buf = val;
	spi_message_add_tail(&t[i++], &m);

	return spi_sync(spi, &m);
}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
static int iaxxx_spi_bus_raw_read(struct iaxxx_priv *priv, void *buf, int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_transfer t[2] = {};
	uint8_t reg_addr[16];
	struct spi_message m;
	uint8_t *cbuf = (uint8_t *)buf;
	int rc;
	uint8_t *val;
	uint32_t val_len;

	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return -EBUSY;

	if ((len <= IAXXX_REG_LEN_WITH_PADDING) || (len > IAXXX_MAX_SPI_SIZE)) {
		pr_err("%s() len parameter invalid\n", __func__);
		return -EINVAL;
	}

	spi_message_init(&m);
	/* Create buffer to store read data */
	val_len = len - IAXXX_REG_LEN_WITH_PADDING;
	val = kzalloc(val_len, GFP_KERNEL);
	if (val == NULL) {
		pr_err("%s() failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	/* Fetch the Register address with padding */
	memcpy(reg_addr, cbuf, IAXXX_REG_LEN_WITH_PADDING);

	/* Add Register address write message */
	t[IAXXX_WRITE_INDEX].len = IAXXX_REG_LEN_WITH_PADDING;
	t[IAXXX_WRITE_INDEX].tx_buf = (void *)reg_addr;
	spi_message_add_tail(&t[IAXXX_WRITE_INDEX], &m);

	/* Add data read message */
	t[IAXXX_READ_INDEX].len = val_len;
	t[IAXXX_READ_INDEX].rx_buf = (void *)val;
	spi_message_add_tail(&t[IAXXX_READ_INDEX], &m);

	/* Transfer the message */
	rc = spi_sync(spi, &m);
	if (rc)
		goto err;
	/* Copy the read data into buffer after reagister address */
	memcpy((uint8_t *)(buf + IAXXX_REG_LEN_WITH_PADDING), val, val_len);
err:
	kfree(val);
	return rc;
}

static int iaxxx_spi_bus_raw_write(struct iaxxx_priv *priv, const void *buf,
								int len)
{
	struct device *dev = priv->dev;
	struct spi_device *spi = to_spi_device(dev);

	if (!pm_runtime_enabled(dev) || !pm_runtime_active(dev))
		return -EBUSY;

	if ((len <= (IAXXX_REG_LEN_WITH_PADDING + IAXXX_REG_LEN))
		|| (len > IAXXX_MAX_SPI_SIZE)) {
		pr_err("%s() len parameter invalid\n", __func__);
		return -EINVAL;
	}
	return iaxxx_spi_write(spi, buf, len);
}
#endif


/* spi bus bulk read public function
 * to add pad bit fields variable required
 * for spi read operation
 */
static int iaxxx_spi_bulk_read(struct device *dev,
			uint32_t address, void *read_buf, size_t words)
{
	/* 12 bytes of zero padding is required after
		* address is sent to chip before host
		* can start reading it
		*/
	size_t reg_len = sizeof(uint32_t) + 12;
	uint32_t words_to_read;
	int rc;
	int count = 0;

	while (words > 0) {
		words_to_read = (words < IAXXX_SPI_PACKET_LEN) ?
				words : IAXXX_SPI_PACKET_LEN;

		dev_dbg(dev, "%s(): words_to_read %d\n",
					__func__, words_to_read);

		rc = iaxxx_spi_bus_read(dev, address, reg_len,
					read_buf, (words_to_read * 4));
		if (rc < 0) {
			dev_err(dev,
				"%s(): bulk read error %d\n",
				__func__, rc);
			return rc;
		}
		iaxxx_spi_32bit_plat_endian(dev,
					read_buf, (words_to_read << 2));
		address += (words_to_read * sizeof(uint32_t));
		read_buf = ((char *)read_buf) +
				(words_to_read * sizeof(uint32_t));
		words -= words_to_read;
		count += words_to_read;
	}

	return count;
}

static int iaxxx_spi_populate_dt_data(struct iaxxx_spi_priv *spi_priv)
{
	struct device *dev = spi_priv->priv.dev;
	struct device_node *node = dev->of_node;
	int rc = -EINVAL;
	u32 tmp;

	/* Check if pinctrl is defined */
	spi_priv->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(spi_priv->pinctrl)) {
		spi_priv->spi_cs_low =
			pinctrl_lookup_state(spi_priv->pinctrl,
						"spi_cs_low");
		if (IS_ERR(spi_priv->spi_cs_low))
			spi_priv->spi_cs_low = NULL;

		spi_priv->spi_cs_normal =
			pinctrl_lookup_state(spi_priv->pinctrl,
						"spi_cs_normal");
		if (IS_ERR(spi_priv->spi_cs_normal))
			spi_priv->spi_cs_normal = NULL;
	}

	/* No node means  no dts provision */
	if (node == NULL) {
		pr_err("Invalid of node");
		return 0;
	}

	rc = of_property_read_u32(node, "adnc,spi-app-speed", &tmp);
	if (rc < 0) {
		pr_err("Failed to read spi_app_speed, rc = %d\n", rc);
		return rc;
	}
	spi_priv->priv.spi_app_speed = tmp;


	return rc;

}

static struct regmap_bus regmap_spi = {
	.write = iaxxx_regmap_spi_write,
	.gather_write = iaxxx_regmap_spi_gather_write,
	.read = iaxxx_regmap_spi_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

/* Register map initialization */
static int iaxxx_spi_regmap_init(struct iaxxx_priv *priv)
{
	int ret;
	struct device *dev;
	struct regmap *regmap;
	struct iaxxx_spi_priv *spi_priv = to_spi_priv(priv);

	if (!spi_priv || !priv->regmap_config) {
		pr_err("%s: NULL input pointer(s)\n", __func__);
		return -EINVAL;
	}

	dev = &spi_priv->spi->dev;

	/* FPGA requires 12 bytes of zero padding between address and data */
	/* TODO: Move this to device tree property 'adnc,spi-zero-padding'  */
	priv->regmap_config->pad_bits = 96;

	regmap = regmap_init(dev, &regmap_spi,
					spi_priv->spi, priv->regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	priv->regmap = regmap;
	return 0;
}

static int iaxxx_spi_sync(struct iaxxx_spi_priv *spi_priv)
{
	int retry = IAXXX_SYNC_RETRY;
	int rc = 0;
	uint32_t sync_response;
	struct device *dev = spi_priv->priv.dev;
	const uint32_t SBL_SYNC_CMD = 0x80000000;
	const uint32_t CMD_REGMAP_MODE = 0x80040000;

	do {
		retry--;
		/* Populate device tree data and Reset chip to SBL */
		rc = iaxxx_device_reset(&spi_priv->priv);
		if (rc) {
			dev_err(dev, "%s device reset failed, err:%d\n", __func__, rc);
			break;
		}

		/*
		 * Set SPI as normal mode controlled by bus master
		 * if it's defined
		 */
		if (spi_priv->spi_cs_normal)
			pinctrl_select_state(spi_priv->pinctrl,
					spi_priv->spi_cs_normal);

		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, SBL_SYNC_CMD, &sync_response);
		if (rc)
			continue;

		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);

		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, 0x80720080, &sync_response);
		if (rc) {
			dev_err(dev, "%s SYNC fail, err:%d\n", __func__, rc);
		}
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, 0x80735001, &sync_response);
		if (rc)
			dev_err(dev, "%s SYNC fail, err:%d\n", __func__, rc);
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, 0x80740000, &sync_response);
		if (rc)
			dev_err(dev, "%s SYNC fail, err:%d\n", __func__, rc);
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;
		/* Send a SYNC command */
		rc = iaxxx_spi_cmd(spi_priv->spi, 0x80750000, &sync_response);
		if (rc)
			dev_err(dev, "%s SYNC fail, err:%d\n", __func__, rc);
		dev_dbg(dev, "SYNC response: 0x%.08X\n", sync_response);
		sync_response = 0;

		dev_dbg(dev, "Putting device in regmap mode\n");

		/* Switch the device into regmap mode */
		rc = iaxxx_spi_cmd(spi_priv->spi, CMD_REGMAP_MODE, NULL);
		if (rc) {
			continue;
		}
	} while (rc && retry);

	return rc;
}

static int iaxxx_spi_reset_cb(struct device *dev)
{
	struct iaxxx_spi_priv *spi_priv = dev ? dev_get_drvdata(dev) : NULL;
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi_priv || !spi)
		return rc;


	/* set SPI speed to 4.8Mhz */
	spi->max_speed_hz = IAXXX_SPI_SBL_SPEED;
	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s spi 4.8Mhz setup failed, error:%d\n",
				__func__, rc);

	rc = iaxxx_spi_sync(spi_priv);
	if (rc)
		return rc;

	usleep_range(1000, 2500);

	return rc;
}

static int iaxxx_spi_speed_setup(struct device *dev, u32 iaxxx_spi_speed)
{
	struct spi_device *spi = dev ? to_spi_device(dev) : NULL;
	int rc = -EINVAL;

	if (!spi)
		return rc;

	/* set SPI speed to 8Mhz */
	spi->max_speed_hz = iaxxx_spi_speed;
	rc = spi_setup(spi);
	if (rc)
		dev_err(dev, "%s spi speed:%d setup failed, error:%d\n",
				__func__, spi->max_speed_hz, rc);

	dev_dbg(dev, "success spi->max_speed_hz = %d\n", spi->max_speed_hz);
	return rc;
}

static int iaxxx_spi_probe(struct spi_device *spi)
{
	int rc = 0;
	struct iaxxx_spi_priv *spi_priv;
	struct device *dev = &spi->dev;
	struct clk *iaxxx_ext_clk;

	dev_dbg(dev, "%s:%d\n", __func__, __LINE__);

	/* Register for Clock */
	iaxxx_ext_clk = devm_clk_get(dev, "iaxxx_ext_clk");
	if (IS_ERR(iaxxx_ext_clk)) {
		rc = PTR_ERR(iaxxx_ext_clk);
		dev_err(dev, "%s: clk_get failed: %d\n", __func__, rc);
		return rc;
	}

	/* Create driver private-data struct */
	spi_priv = devm_kzalloc(dev, sizeof(*spi_priv), GFP_KERNEL);
	if (!spi_priv)
		return -ENOMEM;

	spi_priv->priv.iaxxx_state = devm_kzalloc(dev,
			sizeof(struct iaxxx_system_state), GFP_KERNEL);
	if (!spi_priv->priv.iaxxx_state) {
		rc = -ENOMEM;
		goto state_mem_failed;
	}

	spi_priv->priv.crashlog = devm_kzalloc(dev,
			sizeof(struct iaxxx_crashlog), GFP_KERNEL);
	if (!spi_priv->priv.crashlog) {
		rc = -ENOMEM;
		goto crash_mem_failed;
	}

	spi_priv->spi = spi;
	spi_priv->priv.dev = dev;
	spi_priv->priv.regmap_init_bus = iaxxx_spi_regmap_init;
	spi_priv->priv.bulk_read = iaxxx_spi_bulk_read;
	spi_priv->priv.raw_write = iaxxx_spi_raw_write;
	spi_priv->priv.bus = IAXXX_SPI;

	spi_set_drvdata(spi, spi_priv);

	spi_priv->priv.spi_speed_setup = iaxxx_spi_speed_setup;
	spi_priv->priv.reset_cb = iaxxx_spi_reset_cb;
	spi_priv->priv.ext_clk = iaxxx_ext_clk;
	rc = iaxxx_device_init(&spi_priv->priv);
	if (rc) {
		dev_err(dev, "%s device init failed, err:%d\n", __func__, rc);
		goto probe_failed;
	}

#ifdef CONFIG_MFD_IAXXX_DEBUG_DUMP
	spi_priv->priv.raw_ops->write = iaxxx_spi_bus_raw_write;
	spi_priv->priv.raw_ops->read = iaxxx_spi_bus_raw_read;
#endif
	spi_priv->priv.crash_count = 0;

	/* Get app SPI Speed from DT*/
	rc = iaxxx_spi_populate_dt_data(spi_priv);
	if (rc < 0) {
		dev_err(dev, "Failed to read spi_app_speed, rc = %d\n", rc);
		return rc;
	}

	return rc;

probe_failed:
	devm_kfree(&spi->dev, spi_priv->priv.crashlog);
crash_mem_failed:
	devm_kfree(&spi->dev, spi_priv->priv.iaxxx_state);
state_mem_failed:
	devm_kfree(dev, spi_priv);
	spi_set_drvdata(spi, NULL);
	return rc;
}

static int iaxxx_spi_remove(struct spi_device *spi)
{
	struct iaxxx_spi_priv *spi_priv = spi_get_drvdata(spi);

	if (spi_priv) {
		iaxxx_device_exit(&spi_priv->priv);
		devm_kfree(&spi->dev, spi_priv->priv.iaxxx_state);
		kfree(spi_priv->priv.crashlog->log_buffer);
		devm_kfree(&spi->dev, spi_priv->priv.crashlog);
		devm_kfree(&spi->dev, spi_priv);
	}
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct dev_pm_ops iaxxx_spi_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_core_rt_suspend, iaxxx_core_rt_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_core_dev_suspend, iaxxx_core_dev_resume)
};

static const struct spi_device_id iaxxx_spi_id[] = {
	{ "iaxxx-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, iaxxx_spi_id);

static const struct of_device_id iaxxx_spi_dt_match[] = {
	{
		.compatible = "knowles,iaxxx-spi",
	},
	{}
};

static struct spi_driver iaxxx_spi_driver = {
	.driver = {
		.name = "iaxxx-spi",
		.owner = THIS_MODULE,
		.pm = &iaxxx_spi_pm_ops,
		.of_match_table = iaxxx_spi_dt_match,
	},
	.probe = iaxxx_spi_probe,
	.remove = iaxxx_spi_remove,
	.id_table = iaxxx_spi_id,
/*	.device_up = iaxxx_spi_up, */
/*	.device_down = iaxxx_spi_down, */
/*	.reset_device = iaxxx_spi_reset, */
};

module_spi_driver(iaxxx_spi_driver);

MODULE_DESCRIPTION("SPI support for Knowles IAxxx");
MODULE_LICENSE("GPL");
