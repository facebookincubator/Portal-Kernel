/**
 * @section LICENSE
 * (C) Copyright 2018~2019 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_spi.c
 * @date     2020/11/10
 * @version  1.3.14
 *
 * @brief    bma4xy SPI bus Driver
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "bma4xy_driver.h"
#include "bs_log.h"

#define BMA4XY_MAX_BUFFER_SIZE      32

static struct spi_device *bma4xy_spi_client;

static s8 bma4xy_spi_write_block(u8 reg_addr, const u8 *data, u8 len)
{
	struct spi_device *client = bma4xy_spi_client;
	u8 buffer[BMA4XY_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.len = len + 1,
	};
	struct spi_message msg;

	if (len > BMA4XY_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr&0x7F;/* write: MSB = 0 */
	memcpy(&buffer[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	return spi_sync(client, &msg);
}

static s8 bma4xy_spi_read_block(u8 reg_addr, u8 *data, u16 len)
{
	struct spi_device *client = bma4xy_spi_client;
	u8 reg = reg_addr | 0x80;/* read: MSB = 1 */
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
	return spi_sync(client, &msg);
}

static s8 bma4xy_spi_write_wrapper(u8 reg_addr, const u8 *data, u32 len,
				void *intf_ptr)
{
	s8 err;

	err = bma4xy_spi_write_block(reg_addr, data, len);
	return err;
}

static s8 bma4xy_spi_read_wrapper(u8 reg_addr, u8 *data, u32 len,
				void *intf_ptr)
{
	s8 err;

	err = bma4xy_spi_read_block(reg_addr, data, len);
	return err;
}

static void bma4xy_spi_init_client_data(struct bma4xy_client_data *client_data)
{
	u8 dev_id = BMA4_SPI_INTF;

	client_data->device.intf_ptr = &dev_id;
	client_data->device.variant = BMA45X_VARIANT;
	client_data->device.intf = BMA4_SPI_INTF;
	client_data->device.bus_read = bma4xy_spi_read_wrapper;
	client_data->device.bus_write = bma4xy_spi_write_wrapper;
	client_data->device.read_write_len = 4;
}

static int bma4xy_spi_probe(struct spi_device *client)
{
	int status;
	int err = 0;
	u8 dev_id;
	u8 chip_id[2] = {0};

	if (bma4xy_spi_client == NULL)
		bma4xy_spi_client = client;
	else{
		PERR("This driver does not support multiple clients!\n");
		return -EBUSY;
	}
	client->bits_per_word = 8;
	status = spi_setup(client);
	if (status < 0) {
		PERR("spi_setup failed!\n");
		return status;
	}

	usleep_range(BMA4XY_INIT_DELAY_TIME_US,
				BMA4XY_INIT_DELAY_TIME_US + 1000);

	err = bma4xy_spi_read_wrapper(BMA4_CHIP_ID_ADDR,
				chip_id, 2, &client->dev);
	if (err) {
		PERR("Unable to read chip id");
		err = -ENODEV;
		goto exit_err_clean;
	}

	dev_id = chip_id[0];
	return bma4xy_probe(&client->dev, dev_id, client->irq,
			&bma4xy_spi_init_client_data);

exit_err_clean:
	if (err)
		bma4xy_spi_client = NULL;
	return err;
}

static int bma4xy_spi_remove(struct spi_device *client)
{
	int err = 0;

	err = bma4xy_remove(&client->dev);
	bma4xy_spi_client = NULL;
	return err;
}

static const struct spi_device_id bma4xy_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, bma4xy_id);
static const struct of_device_id bma4xy_of_match[] = {
	{ .compatible = "bosch,bma4xy", },
	{ }
};

MODULE_DEVICE_TABLE(spi, bma4xy_of_match);

static struct spi_driver bmi_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_NAME,
		.of_match_table = bma4xy_of_match,
	},
	.id_table = bma4xy_id,
	.probe    = bma4xy_spi_probe,
	.remove   = bma4xy_spi_remove,
};

static int __init bmi_spi_init(void)
{
	return spi_register_driver(&bmi_spi_driver);
}

static void __exit bmi_spi_exit(void)
{
	spi_unregister_driver(&bmi_spi_driver);
}


MODULE_AUTHOR("Contact <contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA4XY SPI DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bmi_spi_init);
module_exit(bmi_spi_exit);
