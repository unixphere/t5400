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
#include <linux/i2c.h>
#include <linux/pm.h>
#include "t5400.h"

static int t5400_i2c_read_block(void *client, u8 reg, int len, char *buf)
{
	return i2c_smbus_read_i2c_block_data(client, reg, len, buf);
}

static int t5400_i2c_read_byte(void *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int t5400_i2c_write_byte(void *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static const struct t5400_bus_ops t5400_i2c_bus_ops = {
	.read_block	= t5400_i2c_read_block,
	.read_byte	= t5400_i2c_read_byte,
	.write_byte	= t5400_i2c_write_byte
};

static int __devinit t5400_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct t5400_bus data_bus = {
		.bops = &t5400_i2c_bus_ops,
		.client = client,
		.id = {
			.bustype = BUS_I2C,
		},
	};

	return t5400_probe(&client->dev, &data_bus);
}

static void t5400_i2c_shutdown(struct i2c_client *client)
{
	t5400_disable(&client->dev);
}

static int __devexit t5400_i2c_remove(struct i2c_client *client)
{
	return t5400_remove(&client->dev);
}

#if defined(CONFIG_PM)
static int t5400_i2c_suspend(struct device *dev)
{
	return t5400_disable(dev);
}

static int t5400_i2c_resume(struct device *dev)
{
	return t5400_enable(dev);
}
#endif

static UNIVERSAL_DEV_PM_OPS(t5400_i2c_pm, t5400_i2c_suspend,
					t5400_i2c_resume, NULL);

static const struct i2c_device_id t5400_id[] = {
	{ T5400_DRIVER, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, t5400_id);

static struct i2c_driver t5400_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= T5400_DRIVER,
		.pm	= &t5400_i2c_pm,
	},
	.id_table	= t5400_id,
	.probe		= t5400_i2c_probe,
	.shutdown	= t5400_i2c_shutdown,
	.remove		= __devexit_p(t5400_i2c_remove)
};

static int __init t5400_i2c_init(void)
{
	return i2c_add_driver(&t5400_i2c_driver);
}

static void __exit t5400_i2c_exit(void)
{
	i2c_del_driver(&t5400_i2c_driver);
}


MODULE_AUTHOR("Stefan Nilsson <stefan.nilsson@unixphere.com>");
MODULE_DESCRIPTION("T5400 I2C bus driver");
MODULE_LICENSE("GPL");

module_init(t5400_i2c_init);
module_exit(t5400_i2c_exit);
