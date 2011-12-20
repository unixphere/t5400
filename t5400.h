#ifndef _T5400_H
#define _T5400_H
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
#include <linux/input.h>

#define T5400_DRIVER	"t5400"

/**
 * enum t5400_op_mode - defines the different operation modes.
 *
 * The T5400 supports different operation modes which provides different
 * accuracy levels (and accompanied conversion time) in terms of RMS noise
 * performance.
 */
enum t5400_op_mode {
	T5400_OP_MODE_LOW,	/* ~8.8Pa RMS Noise, 2ms conv. time */
	T5400_OP_MODE_STANDARD,	/* ~6.4Pa RMS Noise, 8ms conv. time */
	T5400_OP_MODE_HIGH,	/* ~5.0Pa RMS Noise, 16ms conv. time */
	T5400_OP_MODE_U_HIGH,	/* ~4.4Pa RMS Noise, 64ms conv. time */
	T5400_OP_MODE_LAST
};

/**
 * struct t5400_platform_data - represents platform data for the t5400 driver
 * @op_mode: operation modes to be set according to enum t5400_op_mode
 * @gpio_irq: platform gpio pin connected to the t5400 irq pin
 * @gpio_init: callback for platform-specific t5400 gpio initialization,
 *	where 'activate = 1' initializes and 'activate = 0' de-initializes
 */
struct t5400_platform_data {
	enum t5400_op_mode op_mode;
	int gpio_irq;
	int (*gpio_init)(bool activate);
};

/**
 * struct t5400_bus_ops - supported bus operations for the t5400 driver
 */
struct t5400_bus_ops {
	int (*read_block)(void *client, u8 reg, int len, char *buf);
	int (*read_byte)(void *client, u8 reg);
	int (*write_byte)(void *client, u8 reg, u8 value);
};

/**
 * struct t5400_platform_data - represents the t5400 driver bus setup
 */
struct t5400_bus {
	const struct t5400_bus_ops *bops;
	void *client;
	struct input_id id;
};

int t5400_probe(struct device *dev, struct t5400_bus *data_bus);
int t5400_remove(struct device *dev);
#if defined(CONFIG_PM)
int t5400_enable(struct device *dev);
int t5400_disable(struct device *dev);
#endif

#endif
