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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include "t5400.h"

/* Max/Min barometric pressure values as stated in spec, unit = hPa */
#define T5400_ABS_MAX_PRES	1100
#define T5400_ABS_MIN_PRES	300

/* Max/Min ambient temperature values as stated in spec, unit = Celsius */
#define T5400_ABS_MAX_TEMP	80
#define T5400_ABS_MIN_TEMP	-30

/* Chip ID */
#define T5400_CHIP_ID_REG 	0x88
#define T5400_CHIP_ID_MSK	0x7F
#define T5400_CHIP_ID		0x77

/**
 * Calibration coefficients registers
 *
 * The coefficients comes in a consecutive register serie, from C1 to C8, where
 * C1-C4 are unsigned 16-bit integer values and C5-C8 are signed 16-bit values.
 */
#define T5400_C1_LSB_REG	0x8E
#define T5400_C8_MSB_REG	0x9D
#define T5400_CALIB_REGS_SIZE	16

/* Software reset  */
#define T5400_CTRL_RES_REG	0xF0
#define T5400_RESET_CMD		0x73

/**
 * Temperature/Pressure measurement cmd, with the following CTRL_CMD bitfields:
 *
 * b0: sco, set to 1 for conversion start
 * b1-b2: pt, set 00 for pressure meas and 01 for temperature meas
 * b3-b4: mode, set values according to enum t5400_op_mode
 * b5: zero, set to 0 for applying the command correctly
 * b6-b7: don't care
 */
#define T5400_CTRL_CMD_REG	0xF1
#define T5400_CONV_PRES_CMD	0x01
#define T5400_CONV_TEMP_CMD	0x03
#define T5400_MEAS_MODE_POS	3

/* Temperature/Pressure Data registers */
#define T5400_DATA_LSB_REG	0xF5
#define T5400_DATA_MSB_REG	0xF6
#define T5400_DATA_SIZE		2

/* Input poll-intervals */
#define T5400_POLL_INTERVAL	50
#define T5400_POLL_MIN		5
#define T5400_POLL_MAX		200

/* Power-on/Reset start-up time (miliseconds) */
#define T5400_START_UP_TIME	12

/* Idle mode wake-up time (miliseconds) */
#define T5400_WAKE_UP_TIME	2

/* Temperature conversion time (miliseconds) */
#define T5400_TEMP_CONV_TIME	(T5400_WAKE_UP_TIME + 4)

/* Pressure measurement conversion time (miliseconds) including the wakeup */
static const u8 pres_conv_time[T5400_OP_MODE_LAST] = {
	(T5400_WAKE_UP_TIME + 2),
	(T5400_WAKE_UP_TIME + 8),
	(T5400_WAKE_UP_TIME + 16),
	(T5400_WAKE_UP_TIME + 64)
};

enum t5400_conv_type {
	T5400_CONV_PRES,
	T5400_CONV_TEMP
};

struct t5400_calib_coef {
	u16 c1, c2, c3, c4;
	s16 c5, c6, c7, c8;
};

struct t5400_data {
	struct t5400_calib_coef coef;
	u16 raw_pres;
	s16 raw_temp;
};

struct t5400 {
	struct device *dev;
	struct input_polled_dev *input_polled;
	struct input_dev *input;
	struct work_struct work;
	struct mutex mutex;
	struct t5400_bus *bus;
	struct t5400_data data;
	enum t5400_op_mode op_mode;
	int gpio_irq;
	int (*gpio_init)(bool activate);
};

static inline int t5400_read_byte(struct t5400_bus *bus, int reg)
{
	return bus->bops->read_byte(bus->client, reg);
}

static inline int t5400_read_block(struct t5400_bus *bus, int reg,
						int len, char *buf)
{
	return bus->bops->read_block(bus->client, reg, len, buf);
}

static inline int t5400_write_byte(struct t5400_bus *bus, int reg, int val)
{
	return bus->bops->write_byte(bus->client, reg, val);
}

static int t5400_start_conv(struct t5400_bus *bus,
					enum t5400_conv_type type,
					enum t5400_op_mode op_mode)
{
	u8 rc;

	if (type == T5400_CONV_PRES)
		rc = t5400_write_byte(bus, T5400_CTRL_CMD_REG,
					T5400_CONV_PRES_CMD |
					(u8)(op_mode << T5400_MEAS_MODE_POS));
	else
		rc = t5400_write_byte(bus, T5400_CTRL_CMD_REG,
					T5400_CONV_TEMP_CMD);
	return rc;
}

static s32 __devinit t5400_get_calibration_parameters(struct t5400_bus *bus,
					struct t5400_calib_coef *coef)
{
	u16 data[T5400_CALIB_REGS_SIZE >> 1];
	int error;

	error = t5400_read_block(bus, T5400_C1_LSB_REG, T5400_CALIB_REGS_SIZE,
							(char *)data);
	if (error < T5400_CALIB_REGS_SIZE)
		return -EIO;

	coef->c1 = le16_to_cpu(data[0]);
	coef->c2 = le16_to_cpu(data[1]);
	coef->c3 = le16_to_cpu(data[2]);
	coef->c4 = le16_to_cpu(data[3]);
	coef->c5 = le16_to_cpu(data[4]);
	coef->c6 = le16_to_cpu(data[5]);
	coef->c7 = le16_to_cpu(data[6]);
	coef->c8 = le16_to_cpu(data[7]);

	return 0;
}

static int t5400_soft_reset(struct t5400_bus *bus)
{
	int error;

	error = t5400_write_byte(bus, T5400_CTRL_RES_REG, T5400_RESET_CMD);

	if (error < 0)
		return error;

	msleep(T5400_START_UP_TIME);
	return 0;
}

static void t5400_worker(struct work_struct *work)
{
	struct t5400 *t5400 = container_of(work, struct t5400, work);
	struct t5400_bus *bus = t5400->bus;
	u16 raw_pres;
	s16 raw_temp;
	int error;

	error = t5400_start_conv(bus, T5400_CONV_PRES, t5400->op_mode);
	if (error < 0)
		goto error_exit;

	msleep(pres_conv_time[t5400->op_mode]);
	error = t5400_read_block(bus, T5400_DATA_LSB_REG, T5400_DATA_SIZE,
							(char *)&raw_pres);
	if (error < T5400_DATA_SIZE)
		goto error_exit;

	error = t5400_start_conv(bus, T5400_CONV_TEMP, 0);
	if (error < 0)
		goto error_exit;

	msleep(T5400_TEMP_CONV_TIME);
	error = t5400_read_block(bus,T5400_DATA_LSB_REG, T5400_DATA_SIZE,
							(char *)&raw_temp);
	if (error < T5400_DATA_SIZE)
		goto error_exit;

	/**
	 * Store the new values upon successful consecutive readings of
	 * both pressure and temperature, else stick to the previous values.
	 */
	mutex_lock(&t5400->mutex);
	t5400->data.raw_temp = le16_to_cpu(raw_temp);
	t5400->data.raw_pres = le16_to_cpu(raw_pres);
	mutex_unlock(&t5400->mutex);

error_exit:
	schedule_work(&t5400->work);
}


static irqreturn_t t5400_irq_thread(int irq, void *dev)
{
	struct t5400 *t5400 = (struct t5400 *)dev;
	struct t5400_bus *bus = t5400->bus;
	u16 raw_pres;
	s16 raw_temp;
	int error;

	error = t5400_read_block(bus, T5400_DATA_LSB_REG, T5400_DATA_SIZE,
							(char *)&raw_pres);
	if (error < T5400_DATA_SIZE)
		goto error_exit;

	error = t5400_start_conv(bus, T5400_CONV_TEMP, 0);
	if (error < 0)
		goto error_exit;

	msleep(T5400_TEMP_CONV_TIME);
	error = t5400_read_block(bus, T5400_DATA_LSB_REG, T5400_DATA_SIZE,
							(char *)&raw_temp);
	if (error < T5400_DATA_SIZE)
		goto error_exit;

	/* Same as for the worker above */
	mutex_lock(&t5400->mutex);
	t5400->data.raw_temp = le16_to_cpu(raw_temp);
	t5400->data.raw_pres = le16_to_cpu(raw_pres);
	mutex_unlock(&t5400->mutex);

error_exit:
	error = t5400_start_conv(bus, T5400_CONV_PRES, t5400->op_mode);
	if (error < 0)
		dev_err(t5400->dev, "%s: start conv. req. failed, %d\n",
					__func__, error);
	return IRQ_HANDLED;
}

/**
 * This function converts the raw temperature to centi-celsius with optimization
 * for integer fixed-point arithmetics with two fractions embedded in the
 * integer (i.e. 27.30 will be reported as 2730 and so on).
 *
 * Formula from application note, rev_X:
 * Ta = ((c1 * Tr) / 2^24) + (c2 / 2^10)
 */
static inline int t5400_get_temperature(struct t5400_data *data)
{
	s64 temp, val;

	val = ((s64)(data->coef.c1 * data->raw_temp) * 100);
	temp = (val >> 24);
	val = ((s64)data->coef.c2 * 100);
	temp += (val >> 10);
	return (int)temp;
}

/**
 * This function converts the raw pressure to Pascal, i.e. without fractions.
 *
 * Formula from application note, rev_X:
 * Sensitivity = (c3 + ((c4 * Tr) / 2^17) + ((c5 * Tr^2) / 2^34))
 * Offset = (c6 * 2^14) + ((c7 * Tr) / 2^3) + ((c8 * Tr^2) / 2^19)
 * Pa = (Sensitivity * Pr + Offset) / 2^14
 */
static inline int t5400_get_pressure(struct t5400_data *data)
{
	s64 s, o, pres, val;

	s = (s64)data->coef.c3 ;
	val = (s64)(data->coef.c4 * data->raw_temp);
	s += (val >> 17);
	val = (s64)(data->coef.c5 * data->raw_temp * data->raw_temp);
	s += (val >> 34);

	o = (s64)data->coef.c6 << 14;
	val = (s64)(data->coef.c7 * data->raw_temp);
	o += (val >> 3);
	val = (s64)(data->coef.c8 * data->raw_temp * data->raw_temp);
	o += (val >> 19);

	pres = ((s64)(s * data->raw_pres) + o) >> 14;

	return (int)pres;
}

static void t5400_poll(struct input_polled_dev *dev)
{
	struct t5400 *t5400 = dev->private;
	int temperature;
	int pressure;

	/**
	 * Locked in order to ensure that both raw temp. & pres. values
	 * comes from the same consecutive readings.
	 */
	mutex_lock(&t5400->mutex);
	temperature = t5400_get_temperature(&t5400->data);
	pressure = t5400_get_pressure(&t5400->data);
	mutex_unlock(&t5400->mutex);

	input_report_abs(t5400->input, ABS_PRESSURE, pressure);
	input_report_abs(t5400->input, ABS_MISC, temperature);
	input_sync(t5400->input);
}

static int t5400_activate(struct t5400 *t5400)
{
	struct t5400_bus *bus = t5400->bus;
	int error;

	if (t5400->gpio_irq > 0) {
		enable_irq(t5400->gpio_irq);
		error = t5400_start_conv(bus, T5400_CONV_PRES, t5400->op_mode);
		if (error < 0)
			dev_err(t5400->dev, "%s: start conv. req. failed, %d\n",
						__func__, error);
		return error;
	} else {
		schedule_work(&t5400->work);
		return 0;
	}
}

static int t5400_deactivate(struct t5400 *t5400)
{
	struct t5400_bus *bus = t5400->bus;
	int error;

	if (t5400->gpio_irq > 0) {
		disable_irq(t5400->gpio_irq);
		error = t5400_soft_reset(bus);
		if (error < 0)
			dev_err(t5400->dev, "%s: soft reset failed, %d\n",
						__func__, error);
		return error;
	} else {
		cancel_work_sync(&t5400->work);
		return 0;
	}
}

static void t5400_open(struct input_polled_dev *dev)
{
	struct t5400 *t5400 = dev->private;
	int error;

	error = t5400_activate(t5400);
	if (error < 0)
		dev_err(t5400->dev, "%s: failed, %d\n", __func__, error);
}

static void t5400_close(struct input_polled_dev *dev)
{
	struct t5400 *t5400 = dev->private;
	int error;

	error = t5400_deactivate(t5400);
	if (error < 0)
		dev_err(t5400->dev, "%s: failed, %d\n", __func__, error);
}

static int __devinit t5400_register_polled_device(struct t5400 *t5400)
{
	struct input_polled_dev *ipoll_dev;
	struct input_dev *idev;
	int error;

	ipoll_dev = input_allocate_polled_device();
	if (!ipoll_dev)
		return -ENOMEM;

	ipoll_dev->private = t5400;
	ipoll_dev->poll = t5400_poll;
	ipoll_dev->open = t5400_open;
	ipoll_dev->close = t5400_close;
	ipoll_dev->poll_interval = T5400_POLL_INTERVAL;
	ipoll_dev->poll_interval_min = T5400_POLL_MIN;
	ipoll_dev->poll_interval_max = T5400_POLL_MAX;

	idev = ipoll_dev->input;
	idev->name = T5400_DRIVER;
	idev->phys = T5400_DRIVER "/input0";
	idev->id = t5400->bus->id;
	idev->dev.parent = t5400->dev;

	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_PRESSURE, T5400_ABS_MIN_PRES,
					T5400_ABS_MAX_PRES, 0, 0);
	input_set_abs_params(idev, ABS_MISC, T5400_ABS_MIN_TEMP,
					T5400_ABS_MAX_TEMP, 0, 0);


	error = input_register_polled_device(ipoll_dev);
	if (error < 0) {
		input_free_polled_device(ipoll_dev);
		return error;
	}

	t5400->input = ipoll_dev->input;
	t5400->input_polled = ipoll_dev;

	if (t5400->gpio_irq > 0) {
		error = request_threaded_irq(t5400->gpio_irq, NULL,
			t5400_irq_thread, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			T5400_DRIVER, t5400);
		if (error < 0) {
			dev_err(t5400->dev, "%s: irq req. failed %d, %d\n",
					__func__, t5400->gpio_irq, error);
			goto error_irq;
		}
		disable_irq(t5400->gpio_irq);
	} else {
		INIT_WORK(&t5400->work, t5400_worker);
	}

	return 0;

error_irq:
	input_unregister_polled_device(t5400->input_polled);
	return error;
}

int __devinit t5400_probe(struct device *dev, struct t5400_bus *bus)
{
	struct t5400 *t5400;
	struct t5400_platform_data *pdata = dev->platform_data;
	u8 chip_id;
	int error;

	if (pdata) {
		if (pdata->gpio_init) {
			error = pdata->gpio_init(1);
			if (error) {
				dev_err(dev, "%s: gpio init failed, %d\n",
						__func__, error);
				return -EIO;
			}
		}
	}

	error = t5400_soft_reset(bus);
	if (error < 0) {
		dev_err(dev, "%s: soft reset failed, %d\n", __func__, error);
		return -EIO;
	}

	chip_id = (T5400_CHIP_ID_MSK & t5400_read_byte(bus, T5400_CHIP_ID_REG));
	if (chip_id != T5400_CHIP_ID) {
		dev_err(dev, "%s: chip id failed, %d\n", __func__, chip_id);
		return -EINVAL;
	}

	t5400 = kzalloc(sizeof(struct t5400), GFP_KERNEL);
	if (!t5400)
		return -ENOMEM;

	dev_set_drvdata(dev, t5400);
	t5400->dev = dev;
	t5400->bus = bus;

	error = t5400_get_calibration_parameters(t5400->bus, &t5400->data.coef);
	if (error < 0) {
		dev_err(dev, "%s: failed to get calib. params, %d\n",
				__func__, error);
		goto err_free_mem;
	}

	if (pdata) {
		t5400->op_mode = pdata->op_mode;
		t5400->gpio_irq = pdata->gpio_irq;
		t5400->gpio_init = pdata->gpio_init;
	} else {
		t5400->op_mode = T5400_OP_MODE_LOW;
	}

	error = t5400_register_polled_device(t5400);
	if (error < 0)
		goto err_free_mem;

	mutex_init(&t5400->mutex);

	return 0;

err_free_mem:
	kfree(t5400);
	return error;
}
EXPORT_SYMBOL(t5400_probe);

int __devexit t5400_remove(struct device *dev)
{
	struct t5400 *t5400 = dev_get_drvdata(dev);

	if (t5400->gpio_irq > 0)
		free_irq(t5400->gpio_irq, t5400);
	input_unregister_polled_device(t5400->input_polled);
	input_free_polled_device(t5400->input_polled);
	kfree(t5400);

	return 0;
}
EXPORT_SYMBOL(t5400_remove);

#if defined(CONFIG_PM)
int t5400_disable(struct device *dev)
{
	struct t5400 *t5400 = dev_get_drvdata(dev);
	int error;

	error = t5400_deactivate(t5400);
	if (error < 0){
		dev_err(dev, "%s: deactivation failed, %d\n", __func__, error);
		return error;
	}

	if (t5400->gpio_init)
		return t5400->gpio_init(0);

	return 0;
}
EXPORT_SYMBOL(t5400_disable);

int t5400_enable(struct device *dev)
{
	struct t5400 *t5400 = dev_get_drvdata(dev);
	int error;

	if (t5400->gpio_init) {
		error = t5400->gpio_init(1);
		if (error < 0) {
			dev_err(dev, "%s: gpio init failed, %d\n",
					__func__, error);
			return error;
		}
	}
	return t5400_activate(t5400);
}
EXPORT_SYMBOL(t5400_enable);
#endif

MODULE_AUTHOR("Stefan Nilsson <stefan.nilsson@unixphere.com>");
MODULE_DESCRIPTION("T5400 driver");
MODULE_LICENSE("GPL");
