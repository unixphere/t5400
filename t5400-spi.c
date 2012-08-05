/**
 * Copyright (c) 2011 Epcos/TDK
 * Copyright (c) 2011 Unixphere
 *
 * This driver supports the Epcos/TDK T5400 digital barometric pressure
 * and temperature sensor
 *
 * The datasheet for the T5400 chip can be found here:
 * http://www.epcos.com/datasheets/C953.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include "t5400.h"

static int t5400_spi_write_byte(void *client, u8 reg, u8 value)
{
	u8 data[2] = {reg, value};
	return spi_write(client, data, 2);
}

static int t5400_spi_read_block(void *client, u8 reg, int len, char *buf)
{
	int rc = t5400_spi_write_byte(client, reg, 0);
	if (rc < 0)
		return rc;

	return spi_read(client, buf, len);
}

static int t5400_spi_read_byte(void *client, u8 reg)
{
	u8 data;
	int rc = t5400_spi_write_byte(client, reg, 0);
	if (rc < 0)
		return rc;

	rc = spi_read(client, &data, 1);
	if (rc < 0)
		return rc;

	return data;
}

static const struct t5400_bus_ops t5400_spi_bus_ops = {
	.read_block	= t5400_spi_read_block,
	.read_byte	= t5400_spi_read_byte,
	.write_byte	= t5400_spi_write_byte
};

static int __devinit t5400_spi_probe(struct spi_device *client)
{
	int error;
	struct t5400_bus data_bus = {
		.bops = &t5400_spi_bus_ops,
		.client = client,
		.id = {
			.bustype = BUS_SPI,
		},
	};

	client->bits_per_word = 8;
	error = spi_setup(client);
	if (error < 0) {
		dev_err(&client->dev, "%s: spi_setup failed!\n", __func__);
		return error;
	}

	return t5400_probe(&client->dev, &data_bus);
}

static void t5400_spi_shutdown(struct spi_device *client)
{
	t5400_disable(&client->dev);
}

static int __devexit t5400_spi_remove(struct spi_device *client)
{
	struct t5400_bus *t5400_spi = spi_get_drvdata(client);
	int error;

	error = t5400_remove(&client->dev);
	if (error < 0)
		return error;

	kfree(t5400_spi);
	return 0;
}

#if defined(CONFIG_PM)
static int t5400_spi_suspend(struct device *dev)
{
	return t5400_disable(dev);
}

static int t5400_spi_resume(struct device *dev)
{
	return t5400_enable(dev);
}
#endif

static UNIVERSAL_DEV_PM_OPS(t5400_spi_pm, t5400_spi_suspend,
					t5400_spi_resume, NULL);

static const struct spi_device_id t5400_id[] = {
	{ T5400_DRIVER, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, t5400_id);

static struct spi_driver t5400_spi_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= T5400_DRIVER,
		.pm	= &t5400_spi_pm,
	},
	.id_table	= t5400_id,
	.probe		= t5400_spi_probe,
	.shutdown	= t5400_spi_shutdown,
	.remove		= __devexit_p(t5400_spi_remove)
};

static int __init t5400_spi_init(void)
{
	return spi_register_driver(&t5400_spi_driver);
}

static void __exit t5400_spi_exit(void)
{
	spi_unregister_driver(&t5400_spi_driver);
}


MODULE_AUTHOR("Stefan Nilsson <stefan.nilsson@unixphere.com>");
MODULE_DESCRIPTION("T5400 SPI bus driver");
MODULE_LICENSE("GPL");

module_init(t5400_spi_init);
module_exit(t5400_spi_exit);
