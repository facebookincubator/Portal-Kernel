/*
 * Copyright (c) 2017, MM Solutions AD
 *
 * Author:
 *	Kiril Petrov <kpetrov@mm-sol.com>
 *
 * Loosely based on myriad_man.c by Alexandru-Sever Horin
 *					<alexandru.horin@movidius.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>

const char M2_MVCMD_MAGIC[] = "MA2x";
const char M1_MVCMD_MAGIC[] = "MA1x";

#define MYRIAD_DEV_NAME "myriad"

#define MYRIAD_FLG_SESSION_LOCK		0
#define MYRIAD_FLG_FW_LOAD		1
#define MYRIAD_FLG_TIMEOUT		2
#define MYRIAD_FLG_PRESENT		3
#define MYRIAD_FLG_FW_MAGIC		4
#define MYRIAD_FLG_VERSION_ID	5

#define MYRIAD_WAIT_NONE		0
#define MYRIAD_WAIT_FIXED		1
#define MYRIAD_WAIT_IRQ			2

#define MYRYAD_FLASH_BOOT_MODE		0
#define MYRYAD_SPIS_BOOT_MODE		1

struct myriad_ctx {
	struct miscdevice misc_dev;
	struct spi_device *spi;
	struct completion irq_done;
	const char *magic_id;
	unsigned long flags;
	ssize_t fw_count;
	char *spi_buff;

	int gpio_reset;
	int gpio_irq;
	int gpio_irq_active;
	int gpio_pwr;
	int gpio_boot; /* enable Myriad SPIS boot mode */

	int spi_byteperword;
	int spi_boot_frequency;
	int max_boot_time_ms;
	int complete_type;
	int complete_irq_cnt;
	int complete_irq_cnt_init;

	struct timespec ts_boot;
	u32 version_id;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_id_check;
	struct pinctrl_state *pins_normal;
};

static irqreturn_t myriad_isr(int irq, void *data)
{
	struct myriad_ctx *ctx = data;
	struct timespec ts_local;

	getnstimeofday(&ts_local);
	ts_local = timespec_sub(ts_local, ctx->ts_boot);
	dev_info(&ctx->spi->dev, "Myriad booted in %d.%d seconds\n",
			(int)ts_local.tv_sec, (int)ts_local.tv_nsec);

	complete(&ctx->irq_done);

	return IRQ_HANDLED;
}

int myriad_wait_till_irq(struct myriad_ctx *ctx)
{
	struct device *dev = &ctx->spi->dev;

	while (ctx->complete_irq_cnt--) {
		dev_dbg(dev, "wait to boot\n");
		if (!wait_for_completion_timeout(&ctx->irq_done,
			msecs_to_jiffies(ctx->max_boot_time_ms))) {
			set_bit(MYRIAD_FLG_TIMEOUT, &ctx->flags);
			dev_err(dev, "Myriad NOT booted within %dms\n",
					ctx->max_boot_time_ms);
			return -ETIMEDOUT;
		}
	}

	dev_info(dev, "Myriad booted\n");

	return 0;
}

static int myriad_wait_for_complete(struct myriad_ctx *ctx)
{
	int err = 0;

	switch (ctx->complete_type) {
	case MYRIAD_WAIT_FIXED:
		msleep(ctx->max_boot_time_ms);
		break;
	case MYRIAD_WAIT_IRQ:
		err = myriad_wait_till_irq(ctx);
		break;
	default:
		break;
	}

	return err;
}

static int myriad_gpio_init(struct myriad_ctx *ctx)
{
	struct device *dev = &ctx->spi->dev;
	int edge = ctx->gpio_irq_active;
	int myriad_irq;
	int ret = 0;

	if (!ctx)
		return -ENOENT;

	myriad_irq = gpio_to_irq(ctx->gpio_irq);
	if (myriad_irq) {
		uint32_t flags = IRQF_SHARED;
		flags |= edge ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
		ret = request_irq(myriad_irq, myriad_isr,
			flags, "myriad_irq", ctx);
		if (ret) {
			dev_err(dev, "Unable to claim irq %d, err = %d\n",
				myriad_irq, ret);
			return ret;
		}
		dev_info(dev, "myriad_irq = %d\n", myriad_irq);
	}

	return ret;
}

static void myriad_gpio_release(struct myriad_ctx *ctx)
{
	if (gpio_is_valid(ctx->gpio_irq))
		free_irq(gpio_to_irq(ctx->gpio_irq), ctx);
}

static void myriad_reset(struct myriad_ctx *ctx, int flag)
{
	if (gpio_is_valid(ctx->gpio_reset))
		gpio_direction_output(ctx->gpio_reset, 0);

	if (gpio_is_valid(ctx->gpio_pwr))
		gpio_direction_output(ctx->gpio_pwr, 0);

	if (gpio_is_valid(ctx->gpio_boot))
		gpio_direction_output(ctx->gpio_boot, flag);

	if (gpio_is_valid(ctx->gpio_pwr))
		gpio_direction_output(ctx->gpio_pwr, 1);

	msleep(50);

	if (gpio_is_valid(ctx->gpio_reset))
		gpio_direction_input(ctx->gpio_reset);

	msleep(200);
}

void myriad_init_spi(struct myriad_ctx *ctx)
{
	struct spi_device *spi = ctx->spi;

	spi->mode = SPI_MODE_0;
	spi_setup(spi);
	spi->chip_select = 0;
	spi->bits_per_word = ctx->spi_byteperword;
}

/*
 * flag: 1 -> set check mode
 *       0 -> set normal mode
 */
static void myriad_set_check_mode(struct myriad_ctx *ctx, bool flag)
{
	static int backup_frequency;

	if (flag) {
		if (!IS_ERR_OR_NULL(ctx->pins_id_check)) {
			pinctrl_select_state(ctx->pinctrl, ctx->pins_id_check);
			backup_frequency = ctx->spi_boot_frequency;
			ctx->spi_boot_frequency = 1000000;
		}
	} else {
		if (!IS_ERR_OR_NULL(ctx->pins_normal)) {
			pinctrl_select_state(ctx->pinctrl, ctx->pins_normal);
			ctx->spi_boot_frequency = backup_frequency;
		}
	}
}

static int myriad_check_magic(struct myriad_ctx *ctx,
		const char *magic, unsigned char cmd,
		unsigned int address, u32 *rxval)
{
	struct device *dev = &ctx->spi->dev;
	struct spi_device *spi = ctx->spi;
	struct spi_message spi_packet_msg;
	struct spi_transfer spi_request_xfer;
	struct spi_transfer spi_response_xfer;
	unsigned char rx_data[4];
	unsigned char tx_data[16];
	unsigned int tx_size = strlen(magic) + sizeof(cmd) + sizeof(address);
	int err;

	if (!ctx)
		return -EINVAL;

	myriad_set_check_mode(ctx, true);

	memset(rx_data, 0xff, sizeof(rx_data));
	memset(tx_data, 0xff, sizeof(tx_data));

	memcpy(tx_data, magic, strlen(magic));
	memcpy(tx_data + strlen(magic), (const void *)&cmd, sizeof(cmd));
	memcpy(tx_data + strlen(magic) + sizeof(cmd), (const void *)&address,
			sizeof(address));

	myriad_reset(ctx, MYRYAD_SPIS_BOOT_MODE);

	INIT_LIST_HEAD(&spi_request_xfer.transfer_list);
	spi_request_xfer.tx_buf = tx_data;
	spi_request_xfer.rx_buf = NULL;
	spi_request_xfer.len = tx_size;
	spi_request_xfer.bits_per_word = ctx->spi_byteperword;
	spi_request_xfer.speed_hz = ctx->spi_boot_frequency;
	spi_request_xfer.cs_change = 0;
	spi_request_xfer.tx_nbits = SPI_NBITS_SINGLE;
	spi_request_xfer.rx_nbits = SPI_NBITS_SINGLE;
	spi_request_xfer.delay_usecs = 0;

	INIT_LIST_HEAD(&spi_response_xfer.transfer_list);
	spi_response_xfer.tx_buf = NULL;
	spi_response_xfer.rx_buf = rx_data;
	spi_response_xfer.len = sizeof(rx_data);
	spi_response_xfer.bits_per_word = ctx->spi_byteperword;
	spi_response_xfer.speed_hz = ctx->spi_boot_frequency;
	spi_response_xfer.cs_change = 0;
	spi_response_xfer.tx_nbits = SPI_NBITS_SINGLE;
	spi_response_xfer.rx_nbits = SPI_NBITS_SINGLE;
	spi_response_xfer.delay_usecs = 0;

	spi->mode &= ~(SPI_CPHA | SPI_CPOL);

	spi_message_init(&spi_packet_msg);
	spi_message_add_tail(&spi_request_xfer, &spi_packet_msg);
	spi_message_add_tail(&spi_response_xfer, &spi_packet_msg);
	err = spi_sync(spi, &spi_packet_msg);

	myriad_set_check_mode(ctx, false);

	if (err) {
		dev_err(dev, "spi_sync, err = %d\n", err);
		return err;
	}

	spi->mode |= SPI_CPHA | SPI_CPOL;

	pr_debug("myriad response u32: 0x%X\n", *((u32 *)rx_data));

	*rxval = *(u32 *)rx_data;

	return *(u32*)rx_data ? 0 : -ENODEV;
}

static int myriad_check_id(struct myriad_ctx *ctx, const char *magic,
			   const u32 addr, u32 *value)
{
	u32 val[4];
	int i, ret = 0;

	for (i = 0; !ret && i < ARRAY_SIZE(val); i++)
		ret = myriad_check_magic(ctx, magic, 0x82, addr, &val[i]);

	if (ret || !val[0] || val[0] == 0xFFFFFFFF)
		return -ENODEV;

	for (i = 1; i < ARRAY_SIZE(val); i++) {
		if (val[0] != val[i])
			return -ENODEV;
	}

	*value = val[0];

	return 0;
}

static int myriad_probe_on_spi(struct myriad_ctx *ctx)
{
	struct device *dev = &ctx->spi->dev;
	u32 value;
	int ret;

	clear_bit(MYRIAD_FLG_PRESENT, &ctx->flags);

	ret = myriad_check_id(ctx, M1_MVCMD_MAGIC, 0x80030044, &value);
	if (!ret) {
		dev_info(dev, "myriad1 found !\n");
		set_bit(MYRIAD_FLG_PRESENT, &ctx->flags);
		set_bit(MYRIAD_FLG_VERSION_ID, &ctx->flags);
		ctx->magic_id = M1_MVCMD_MAGIC;
		ctx->version_id = value;
		return 0;
	}

	ret = myriad_check_id(ctx, M2_MVCMD_MAGIC, 0x10f00044, &value);
	if (!ret) {
		dev_info(dev, "myriad2 found !\n");
		set_bit(MYRIAD_FLG_PRESENT, &ctx->flags);
		set_bit(MYRIAD_FLG_VERSION_ID, &ctx->flags);
		ctx->magic_id = M2_MVCMD_MAGIC;
		ctx->version_id = value;
		return 0;
	}

	dev_err(dev, "NO myriad found!\n");
	return -ENODEV;
}

static int myriad_spi_open(struct inode *inode, struct file *filp)
{
	struct myriad_ctx *ctx = container_of(filp->private_data,
			struct myriad_ctx, misc_dev);
	int err;

	if (test_and_set_bit(MYRIAD_FLG_SESSION_LOCK, &ctx->flags))
		return -EBUSY;

	filp->private_data = ctx;

	err = myriad_gpio_init(ctx);
	if (err)
		goto fail;

	myriad_init_spi(ctx);

	ctx->magic_id = 0;
	err = myriad_probe_on_spi(ctx);
	if (err)
		goto free_gpio;

	ctx->spi_buff = kzalloc(SZ_1K, GFP_KERNEL);
	if (!ctx->spi_buff) {
		err = -ENOMEM;
		goto free_gpio;
	}

	clear_bit(MYRIAD_FLG_FW_LOAD, &ctx->flags);
	clear_bit(MYRIAD_FLG_FW_MAGIC, &ctx->flags);
	ctx->fw_count = 0;
	ctx->complete_irq_cnt = ctx->complete_irq_cnt_init;
	reinit_completion(&ctx->irq_done);

	return 0;

free_gpio:
	myriad_gpio_release(ctx);
fail:
	clear_bit(MYRIAD_FLG_SESSION_LOCK, &ctx->flags);
	return err;
}

static int myriad_spi_release(struct inode *inode, struct file *filp)
{
	struct myriad_ctx *ctx = filp->private_data;
	struct spi_device *spi = ctx->spi;
	int err;

	spi->mode |= SPI_CPHA | SPI_CPOL;

	if (test_bit(MYRIAD_FLG_PRESENT, &ctx->flags) &&
	    test_bit(MYRIAD_FLG_FW_MAGIC, &ctx->flags))
		err = myriad_wait_for_complete(ctx);

	myriad_reset(ctx, MYRYAD_FLASH_BOOT_MODE);
	myriad_gpio_release(ctx);
	kfree(ctx->spi_buff);
	clear_bit(MYRIAD_FLG_SESSION_LOCK, &ctx->flags);

	return err;
}

static ssize_t myriad_spi_read(struct file *filp, char *buffer,
		size_t length, loff_t *offset)
{
	return 0;
}

static ssize_t myriad_spi_write(struct file *filp, const char *buffer,
		size_t length, loff_t *offset)
{
	struct myriad_ctx *ctx = filp->private_data;
	struct device *dev = &ctx->spi->dev;
	struct spi_message spi_packet_msg;
	struct spi_transfer spi_header_xfer;
	struct spi_device *spi = ctx->spi;
	ssize_t len, sent = 0;
	int err;

	if (!test_bit(MYRIAD_FLG_PRESENT, &ctx->flags))
		return -EINVAL;

	if (!test_and_set_bit(MYRIAD_FLG_FW_LOAD, &ctx->flags)) {
		const char *magic_id = ctx->magic_id;

		err = copy_from_user(ctx->spi_buff, buffer, strlen(magic_id));
		if (strncmp(ctx->spi_buff, magic_id, strlen(magic_id))) {
			dev_err(dev, "wrong magic fw, err = %d\n", err);
			return -EINVAL;
		}

		set_bit(MYRIAD_FLG_FW_MAGIC, &ctx->flags);

		myriad_reset(ctx, MYRYAD_SPIS_BOOT_MODE);
		ctx->complete_irq_cnt = ctx->complete_irq_cnt_init;
		reinit_completion(&ctx->irq_done);

		spi->mode &= ~(SPI_CPHA | SPI_CPOL);
	}

	while (length > 0) {
		len = min_t(size_t, length, SZ_1K);
		if (copy_from_user(ctx->spi_buff, buffer, len))
			break;
		ctx->fw_count += len;
		spi_message_init(&spi_packet_msg);

		spi_header_xfer.tx_buf = ctx->spi_buff;
		spi_header_xfer.rx_buf = NULL;
		spi_header_xfer.len = len;
		spi_header_xfer.bits_per_word = ctx->spi_byteperword;
		spi_header_xfer.speed_hz = ctx->spi_boot_frequency;
		spi_header_xfer.rx_nbits = SPI_NBITS_SINGLE;
		spi_header_xfer.tx_nbits = SPI_NBITS_SINGLE;
		spi_header_xfer.cs_change = 0;
		spi_header_xfer.delay_usecs = 0;

		spi_message_add_tail(&spi_header_xfer, &spi_packet_msg);
		err = spi_sync(spi, &spi_packet_msg);
		if (err)
			break;

		length -= len;
		sent += len;
		buffer += len;
	}

	getnstimeofday(&ctx->ts_boot);

	*offset += sent;
	dev_info(dev, "sent = %zu, count = %zu\n", sent, ctx->fw_count);

	return sent;
}

const struct file_operations spi_control_channel_fops = {
	.owner = THIS_MODULE,
	.read = myriad_spi_read,
	.write = myriad_spi_write,
	.open = myriad_spi_open,
	.release = myriad_spi_release,
};

static ssize_t version_id_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct myriad_ctx *ctx = dev_get_drvdata(dev);

	if (!test_bit(MYRIAD_FLG_VERSION_ID, &ctx->flags))
		return scnprintf(buf, PAGE_SIZE, "unknown\n");

	return scnprintf(buf, PAGE_SIZE, "0x%08X\n", ctx->version_id);
}
static DEVICE_ATTR_RO(version_id);

static struct attribute *myriad_attrs[] = {
	&dev_attr_version_id.attr,
	NULL,
};
ATTRIBUTE_GROUPS(myriad);

static int myriad_parse_dt(struct myriad_ctx *ctx, struct device_node *of_node)
{
	struct device *dev = &ctx->spi->dev;
	int ret = 0;

	ret = of_property_read_u32(of_node, "spi-boot-frequency",
			(u32 *)&ctx->spi_boot_frequency);
	if (ret) {
		dev_err(dev, "Property spi-boot-frequency not found in dts");
		return ret;
	}

	ret = of_property_read_u32(of_node, "spi-byteperword",
			(u32 *)&ctx->spi_byteperword);
	if (ret) {
		dev_err(dev, "Property spi-byteperword not found in dts");
		return ret;
	}

	ret = of_property_read_u32(of_node, "myriad,max-boot-time-ms",
			(u32 *)&ctx->max_boot_time_ms);
	if (ret) {
		dev_err(dev, "Property max-boot-time-ms not found in dts");
		return ret;
	}

	ret = of_property_read_u32(of_node, "myriad,complete_type",
			(u32 *)&ctx->complete_type);
	if (ret)
		ctx->complete_type = MYRIAD_WAIT_NONE;

	ret = of_property_read_u32(of_node, "myriad,complete_irq_cnt",
			(u32 *)&ctx->complete_irq_cnt_init);
	if (ret)
		ctx->complete_irq_cnt_init = 1;

	ctx->pinctrl = devm_pinctrl_get(&ctx->spi->dev);
	if (!IS_ERR(ctx->pinctrl)) {
		ctx->pins_id_check = pinctrl_lookup_state(ctx->pinctrl,
							  "pins_id_check");
		ctx->pins_normal = pinctrl_lookup_state(ctx->pinctrl,
							"pins_normal");
	}

	ctx->gpio_irq = of_get_named_gpio_flags(of_node, "myriad,gpio-irq", 0,
			(enum of_gpio_flags *)&ctx->gpio_irq_active);
	ctx->gpio_reset = of_get_named_gpio(of_node, "myriad,gpio-reset", 0);
	ctx->gpio_pwr = of_get_named_gpio(of_node, "myriad,gpio-pwr", 0);
	ctx->gpio_boot = of_get_named_gpio(of_node, "myriad,gpio-boot", 0);

	dev_dbg(dev, "gpio_irq         = %d\n", ctx->gpio_irq);
	dev_dbg(dev, "gpio_irq_active  = %d\n", ctx->gpio_irq_active);
	dev_dbg(dev, "gpio_reset       = %d\n", ctx->gpio_reset);
	dev_dbg(dev, "gpio_pwr         = %d\n", ctx->gpio_pwr);
	dev_dbg(dev, "gpio_boot        = %d\n", ctx->gpio_boot);

	dev_dbg(dev, "bootspeed        = %d, hz\n", ctx->spi_boot_frequency);
	dev_dbg(dev, "byteperword      = %d\n", ctx->spi_byteperword);
	dev_dbg(dev, "max-boot-time    = %d, ms\n", ctx->max_boot_time_ms);
	dev_dbg(dev, "complete_type    = %d\n", ctx->complete_type);
	dev_dbg(dev, "complete_irq_cnt = %d\n", ctx->complete_irq_cnt);

	return ret;
}

static int myriad_spi_probe(struct spi_device *spi)
{
	struct myriad_ctx *ctx;
	struct device_node *of_node = spi->dev.of_node;
	int err;

	dev_info(&spi->dev, "Probe\n");

	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->spi = spi;
	spi_set_drvdata(spi, ctx);

	err = myriad_parse_dt(ctx, of_node);
	if (err)
		goto exit_free;

	init_completion(&ctx->irq_done);

	ctx->misc_dev.minor = MISC_DYNAMIC_MINOR;
	ctx->misc_dev.name = MYRIAD_DEV_NAME;
	ctx->misc_dev.fops = &spi_control_channel_fops;
	ctx->misc_dev.groups = myriad_groups;

	err = misc_register(&ctx->misc_dev);
	if (err)
		goto exit_free;

	dev_set_drvdata(ctx->misc_dev.this_device, ctx);

	dev_info(&spi->dev, "Done\n");

	return 0;

exit_free:
	kfree(ctx);

	return err;
}

static int myriad_spi_remove(struct spi_device *spi)
{
	struct myriad_ctx *ctx = spi_get_drvdata(spi);

	misc_deregister(&ctx->misc_dev);
	kfree(ctx);

	return 0;
}

static const struct spi_device_id myriad_id[] = {
	{ "qcom_myriad_spi", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, myriad_id);

static const struct of_device_id qcom_myriad_spi_table[] = {
	{ .compatible = "myriad,spi-bus", },
	{ },
};

static struct spi_driver myriad_spi_driver = {
	.driver = {
		.name = "qcom_myriad_spi",
		.owner = THIS_MODULE,
		.of_match_table = qcom_myriad_spi_table,
	},
	.probe = myriad_spi_probe,
	.remove = myriad_spi_remove,
	.id_table = myriad_id,
};

module_spi_driver(myriad_spi_driver);

MODULE_AUTHOR("Kiril Petrov <kpetrov@mm-sol.com>");
MODULE_DESCRIPTION("Myriad flashing module");
MODULE_LICENSE("GPL");
