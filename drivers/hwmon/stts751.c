/*
 * STTS751 sensor driver
 *
 * Copyright (C) 2016 Istituto Italiano di Tecnologia - RBCS - EDL
 * Robotics, Brain and Cognitive Sciences department
 * Electronic Design Laboratory
 *
 * Written by Andrea Merello <andrea.merello@gmail.com>
 *
 * Based on the following drivers:
 * - LM95241 driver, which is:
 *   Copyright (C) 2008, 2010 Davide Rizzo <elpa.rizzo@gmail.com>
 * - LM90 driver, which is:
 *   Copyright (C) 2003-2010  Jean Delvare <jdelvare@suse.de>
 *
 * *******************************************************************
 * NOTE: the STTS751 can reach resolution up to 12 bit. However this
 * is not always possible/reliable.
 *
 * This is because if the device has to generate a thermal/alert
 * signal, it has to perform continuous conversions. In this case the
 * max attainable resolution depends by the conversion rate.
 * Even worse, reading the temperature with resolution better than 8
 * bits would require reading *two* temperature registers, and this is
 * exactly what you want NOT to do when the device is running
 * asynchronously. (looking at the datasheet I couldn't find any trick
 * to emulate an atomic read: no shadow registers, no any 'update' bit
 * to set..).
 *
 * So, it seems we have three choices here (feel free to suggest any
 * other..):
 *
 * 1) Don't care: once every several conversions you'll get a somewhat
 *    imprecise value.. I hate it!
 *    Tricking the user providing him/her super-precise readings that
 *    sometimes are indeed quite imprecise seems really sneaky to me.
 *
 * 2) Stop the device, perform a synchronous conversion, and start it
 *    again. I'm quite tempted to do this..
 *
 * 3) Limit the resolution to 8-bit when the sensor is running. This
 *    would be both perfectly safe and "correct".
 *
 * 4) Try to detect if we went racy, retry the reading few times, and
 *    return the best we can (eventually falling back to 3).
 *
 * Since the thermal/alert signal could be potentially an important
 * protection needed not to fry the HW, I decided that the option 2
 * is too risky (what if when we try to re-enable the sensor our smbus
 * write fails?).
 *
 * Obviously I didn't choose the one I hate, so the only remaining
 * option are 3 and 4.
 *
 * 4 is possibly slower than 3 but it seems reasonable; so I choose 4
 * ********************************************************************
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
 */


#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define DEVNAME "stts751"

static const unsigned short normal_i2c[] = {
	0x48, 0x49, 0x38, 0x39,  /* STTS751-0 */
	0x4A, 0x4B, 0x3A, 0x3B,  /* STTS751-1 */
	I2C_CLIENT_END };

#define STTS751_REG_TEMP_H	0x00
#define STTS751_REG_STATUS	0x01
#define STTS751_STATUS_TRIPH	BIT(6)
#define STTS751_STATUS_TRIPL	BIT(7)
#define STTS751_STATUS_BUSY	BIT(8)
#define STTS751_REG_TEMP_L	0x02
#define STTS751_REG_CONF	0x03
#define STTS751_CONF_RES_MASK	0x0C
#define STTS751_CONF_RES_SHIFT  2
#define STTS751_CONF_EVENT_DIS  BIT(7)
#define STTS751_CONF_STOP 	BIT(6)
#define STTS751_REG_RATE	0x04
#define STTS751_REG_HLIM_H	0x05
#define STTS751_REG_HLIM_L	0x06
#define STTS751_REG_LLIM_H	0x07
#define STTS751_REG_LLIM_L	0x08
#define STTS751_REG_ONESHOT	0x0F
#define STTS751_REG_TLIM	0x20
#define STTS751_REG_HYST	0x21
#define STTS751_REG_SMBUS_TO	0x22

#define STTS751_REG_PROD_ID	0xFD
#define STTS751_REG_MAN_ID	0xFE
#define STTS751_REG_REV_ID	0xFF

#define STTS751_0_PROD_ID	0x00
#define STTS751_1_PROD_ID	0x01
#define ST_MAN_ID		0x53

/* stick with HW defaults */
#define STTS751_THERM_DEFAULT	85000
#define STTS751_HYST_DEFAULT	10000
#define STTS751_EVENT_MAX_DEFAULT 85000
#define STTS751_EVENT_MIN_DEFAULT 0

#define STTS751_RACE_RETRY	5
#define STTS751_CONV_TIMEOUT	100 /* mS */
#define STTS751_CACHE_TIME	100 /* mS */

struct stts751_intervals_t {
	char str[8];
	int val;
};

/* HW index vs ASCII and int times in mS */
static const struct stts751_intervals_t stts751_intervals[] = {
	{.str = "16000", .val = 16000},
	{.str = "8000", .val = 8000},
	{.str = "4000", .val = 4000},
	{.str = "2000", .val = 2000},
	{.str = "1000", .val = 1000},
	{.str = "500", .val = 500},
	{.str = "250", .val = 250},
	{.str = "125", .val = 125},
	{.str = "62.5", .val = 62},
	{.str = "31.25", .val = 31}
};

/* special value to indicate to the SW to use manual mode */
#define STTS751_INTERVAL_MANUAL 0xFF

struct stts751_priv {
	struct device *dev;
	struct i2c_client *client;
	struct mutex access_lock;
	unsigned long interval;
	int res;
	bool gen_therm, gen_event;
	int event_max, event_min;
	int therm;
	int hyst;
	bool smbus_timeout;
	int temp;
	unsigned long last_update;
	u8 config;
	bool min_alert, max_alert;
	bool data_valid;

	/* Temperature is always present
	 * Depending by DT/platdata, therm, event, interval are
	 * dynamically added.
	 * There are max 4 entries plus the guard
	 */
	const struct attribute_group *groups[5];
};

static int stts751_manual_conversion(struct stts751_priv *priv)
{
	s32 ret;
	unsigned long timeout;

	/* Any value written to this reg will trigger manual conversion */
	ret = i2c_smbus_write_byte_data(priv->client,
				STTS751_REG_ONESHOT, 0xFF);
	if (ret < 0)
		return ret;

	timeout = jiffies;

	while (1) {
		ret = i2c_smbus_read_byte_data(priv->client, STTS751_REG_STATUS);
		if (ret < 0)
			return ret;
		if (!(ret & STTS751_STATUS_BUSY))
			return 0;
		if (time_after(jiffies, timeout + STTS751_CONV_TIMEOUT * HZ / 1000)) {
			dev_warn(&priv->client->dev, "conversion timed out\n");
			break;
		}
	}
	return -ETIMEDOUT;
}

/* Converts temperature in C split in integer and fractional parts, as supplied
 * by the HW, to an integer number in mC
 */
static int stts751_to_deg(s32 integer, s32 frac)
{
	s32 temp;

	/* frac part is supplied by the HW as a numbert whose bits weight, from
	 * MSB to LSB, are 2-e1, 2e-2 .. 2e-8; while stored as a regular integer
	 * it would be interpreted as usual (2e+128, 2e+64 ...), so we basically
	 * need to divide it by 256 to ajust the bits' weight.
	 * However this would squash it to zero, so let's convert in in mC (mul
	 * by 1000) right before divide.
	 */
	frac = frac * 1000 / 256;
	temp = sign_extend32(integer, 7) * 1000L + frac;

	return temp;
}

/* Converts temperature in mC to value in C split in integer and fractional
 * parts, as the HW wants.
 */
static int stts751_to_hw(int val, u8 *integer, u8 *frac)
{
	/* HW works in range -64C to +127C */
	if ((val > 127000) || (val < -64000))
		return -EINVAL;

	*integer = val / 1000;
	/* *frac = 256 * (val % 1000) */
	*frac = 256 * (long)(val - *integer * 1000) / 1000;

	return 0;
}

static int stts751_adjust_resolution(struct stts751_priv *priv)
{
	u8 res;

	switch(priv->interval) {
	case 9:
		/* 10 bits */
		res = 0;
		break;
	case 8:
		/* 11 bits */
		res = 1;
		break;
	default:
		/* 12 bits */
		res = 3;
		break;
	}

	if (priv->res == res)
		return 0;

	priv->config &= ~STTS751_CONF_RES_MASK;
	priv->config |= res << STTS751_CONF_RES_SHIFT;

	return i2c_smbus_write_byte_data(priv->client,
				STTS751_REG_CONF, priv->config);
}

static int stts751_update_temp(struct stts751_priv *priv)
{
	s32 integer1, integer2, frac;
	unsigned long sample1, sample2, timeout;
	int i;
	int ret = 0;

	mutex_lock(&priv->access_lock);

	if (priv->interval == STTS751_INTERVAL_MANUAL) {
		/* perform a one-shot on-demand conversion */
		ret = stts751_manual_conversion(priv);
		if (ret) {
			dev_warn(&priv->client->dev,
				"failed to shot conversion %x\n", ret);
			goto exit;
		}
	}

	for (i = 0; i < STTS751_RACE_RETRY; i++) {
		sample1 = jiffies;
		integer1 = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_H);

		if (integer1 < 0) {
			ret = integer1;
			dev_warn(&priv->client->dev,
				"failed to read H reg %x\n", ret);
			goto exit;
		}

		frac = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_L);

		if (frac < 0) {
			ret = frac;
			dev_warn(&priv->client->dev,
				"failed to read L reg %x\n", ret);
			goto exit;
		}

		if (priv->interval == STTS751_INTERVAL_MANUAL) {
			/* we'll look at integer2 later.. */
			integer2 = integer1;
			break;
		}

		integer2 = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_H);
		sample2 = jiffies;

		if (integer2 < 0) {
			dev_warn(&priv->client->dev,
				"failed to read H reg (2nd time) %x\n", ret);
			ret = integer2;
			goto exit;
		}

		timeout = stts751_intervals[priv->interval].val * HZ / 1000;
		timeout -= ((timeout < 10) && (timeout > 1)) ? 1 : timeout / 10;
		if ((integer1 == integer2) &&
			time_after(sample1 + timeout, sample2))
			break;

		/* if we are going on with a racy read, don't pretend to be
		 * super-precise, just use the MSBs ..
		 */
		frac = 0;
	}

exit:
	mutex_unlock(&priv->access_lock);
	if (ret)
		return ret;

	/* use integer2, because when we fallback to the "MSB-only" compromise
	 * this is the more recent one
	 */
	priv->temp = stts751_to_deg(integer2, frac);
	return ret;
}

static int stts751_set_temp_reg(struct stts751_priv *priv, int temp,
				bool is_frac, u8 hreg, u8 lreg)
{
	u8 integer, frac;
	int ret;

	if (stts751_to_hw(temp, &integer, &frac))
		return -EINVAL;

	mutex_lock(&priv->access_lock);
	ret = i2c_smbus_write_byte_data(priv->client, hreg, integer);
	if (ret)
		goto exit;
	if (is_frac)
		ret = i2c_smbus_write_byte_data(priv->client, lreg, frac);
exit:
	mutex_unlock(&priv->access_lock);

	return ret;
}

static int stts751_update_alert(struct stts751_priv *priv)
{
	int ret;

	/* not for us.. */
	if (!priv->gen_event)
		return 0;

	ret = i2c_smbus_read_byte_data(priv->client, STTS751_REG_STATUS);

	if (ret < 0)
		return ret;

	priv->max_alert = priv->max_alert || !!(ret & STTS751_STATUS_TRIPH);
	priv->min_alert = priv->min_alert || !!(ret & STTS751_STATUS_TRIPL);

	return 0;
}

static void stts751_alert(struct i2c_client *client, unsigned int data)
{
	int ret;
	struct stts751_priv *priv = i2c_get_clientdata(client);
	bool prev_max = priv->max_alert;
	bool prev_min = priv->min_alert;

	dev_dbg(&client->dev, "alert!");

	mutex_lock(&priv->access_lock);
	ret = stts751_update_alert(priv);
	if (ret < 0) {
		/* default to worst case */
		priv->max_alert = true;
		priv->min_alert = true;

		if (!(prev_max && prev_min)) {
			dev_warn(&priv->client->dev,
				"Alert received, but can't communicate to the device."
				"Something bad happening? Triggering all alarms!");
		}
	}

	if (!prev_max && priv->max_alert) {
		dev_notice(&client->dev, "got alert for HIGH temperature");

		/* unblock alert poll */
		sysfs_notify(&priv->dev->kobj, NULL, "temp1_event_max_alert");
		kobject_uevent(&priv->dev->kobj, KOBJ_CHANGE);
	}

	if (!prev_min && priv->min_alert) {
		dev_notice(&client->dev, "got alert for LOW temperature");

		/* unblock alert poll */
		sysfs_notify(&priv->dev->kobj, NULL, "temp1_event_min_alert");
		kobject_uevent(&priv->dev->kobj, KOBJ_CHANGE);
	}
	mutex_unlock(&priv->access_lock);
}

static ssize_t show_max_alert(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->max_alert);
}

static ssize_t set_max_alert(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->access_lock);
	priv->max_alert = false;
	mutex_unlock(&priv->access_lock);

	return count;
}

static ssize_t show_min_alert(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->min_alert);
}

static ssize_t set_min_alert(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->access_lock);
	priv->min_alert = false;
	mutex_unlock(&priv->access_lock);

	return count;
}

static ssize_t show_input(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	int cache_time = STTS751_CACHE_TIME * HZ / 1000;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	/* If we are in auto conversion mode adjust the cache time wrt the
	 * sample rate. We do 4X in order to get a new measure in no more than
	 * 1/4 of the sample time (that seemed reasonable to me).
	 */
	if (priv->interval != STTS751_INTERVAL_MANUAL)
		cache_time = stts751_intervals[priv->interval].val /
			4 * HZ / 1000;

	if (time_after(jiffies,	priv->last_update + cache_time) ||
		!priv->data_valid) {
		ret = stts751_update_temp(priv);
		if (ret)
			return ret;
		priv->last_update = jiffies;
		priv->data_valid = true;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->temp);
}

static ssize_t show_therm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->therm);
}


static ssize_t set_therm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;

	ret = stts751_set_temp_reg(priv, temp, false, STTS751_REG_TLIM, 0);
	if (ret)
		return ret;

	dev_dbg(dev, "setting therm %ld", temp);

	priv->therm = temp;
	return count;
}

static ssize_t show_hyst(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->hyst);
}


static ssize_t set_hyst(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;

	ret = stts751_set_temp_reg(priv, temp, false, STTS751_REG_HYST, 0);
	if (ret)
		return ret;

	dev_dbg(dev, "setting hyst %ld", temp);

	priv->therm = temp;
	return count;
}

static ssize_t show_max(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->event_max);
}

static ssize_t set_max(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;

	ret = stts751_set_temp_reg(priv, temp, true,
				STTS751_REG_HLIM_H, STTS751_REG_HLIM_L);
	if (ret)
		return ret;

	dev_dbg(dev, "setting event max %ld", temp);
	priv->event_max = temp;
	return count;
}

static ssize_t show_min(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->event_min);
}


static ssize_t set_min(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;

	ret = stts751_set_temp_reg(priv, temp, true,
				STTS751_REG_LLIM_H, STTS751_REG_LLIM_L);
	if (ret)
		return ret;

	dev_dbg(dev, "setting event min %ld", temp);

	priv->event_min = temp;
	return count;
}

static ssize_t show_interval(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%s\n",
			stts751_intervals[priv->interval].str);
}

static ssize_t set_interval(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned long val;
	int i;
	int ret = 0;
	const int len = sizeof(stts751_intervals) /
		sizeof(stts751_intervals[0]) - 1;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		if (val >= stts751_intervals[i].val)
			break;
	}

	dev_dbg(dev, "setting interval. req:%lu, idx: %d, val: %d", val, i,
		stts751_intervals[i].val);

	if (priv->interval == i)
		return count;

	mutex_lock(&priv->access_lock);

	/* speed up, lower the resolution, then modify convrate */
	if (priv->interval < i) {
		priv->interval = i;
		ret = stts751_adjust_resolution(priv);
		if (ret)
			goto exit;
	}

	ret = i2c_smbus_write_byte_data(priv->client, STTS751_REG_RATE, i);
	if (ret)
		goto exit;

	/* slow down, modify convrate, then raise resolution */
	if (priv->interval != i) {
		priv->interval = i;
		ret = stts751_adjust_resolution(priv);
		if (ret)
			goto exit;

	}
exit:
	mutex_unlock(&priv->access_lock);

	return count;
}

static int stts751_detect(struct i2c_client *new_client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;
	const char *name;
	int mfg_id, prod_id, rev_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	mfg_id = i2c_smbus_read_byte_data(new_client, ST_MAN_ID);
	if (mfg_id != ST_MAN_ID)
		return -ENODEV;

	prod_id = i2c_smbus_read_byte_data(new_client, STTS751_REG_PROD_ID);

	switch (prod_id) {
	case STTS751_0_PROD_ID:
		name = "STTS751-0";
		break;
	case STTS751_1_PROD_ID:
		name = "STTS751-1";
		break;
	default:
		return -ENODEV;
	}
	dev_info(&new_client->dev,"Chip %s detected!", name);

	rev_id = i2c_smbus_read_byte_data(new_client, STTS751_REG_REV_ID);

	if (rev_id != 0x1) {
		dev_notice(&new_client->dev, "Chip revision 0x%x is untested\n"
			"Please report whether it works to "
			"andrea.merello@gmail.com", rev_id);
	}

	strlcpy(info->type, name, I2C_NAME_SIZE);
	return 0;
}

static int stts751_init_chip(struct stts751_priv *priv)
{
	int ret;
	u8 tmp;

	priv->config = STTS751_CONF_EVENT_DIS | STTS751_CONF_STOP;
	ret = i2c_smbus_write_byte_data(priv->client, STTS751_REG_CONF,
					priv->config);
	if (ret)
		return ret;

	/* We always need to write a value consistent wrt to the resolution,
	 * otherwise the sensor does not work.
	 * If we are in manual mode, we use any value for which all resolutions
	 * are admitted. 4 is fine.
	 */
	tmp = (priv->interval == STTS751_INTERVAL_MANUAL) ? 4 : priv->interval;
	ret = i2c_smbus_write_byte_data(priv->client, STTS751_REG_RATE, tmp);
	if (ret)
		return ret;

	/* invalid, to force update */
	priv->res = -1;

	ret = stts751_adjust_resolution(priv);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(priv->client,
					STTS751_REG_SMBUS_TO,
					priv->smbus_timeout ? 0x80 : 0);
	if (ret)
		return ret;

	if (priv->interval != STTS751_INTERVAL_MANUAL) {
		/* user input will not wait for status bit, and we just
		 * provide the last read value. Make sure we really have one
		 * before claiming we are ready..
		 */
		ret = stts751_manual_conversion(priv);
		if (ret)
			return ret;

		if (priv->gen_event) {
			ret = stts751_set_temp_reg(priv, priv->event_max, true,
					STTS751_REG_HLIM_H, STTS751_REG_HLIM_L);
			if (ret)
				return ret;

			ret = stts751_set_temp_reg(priv, priv->event_min, true,
					STTS751_REG_LLIM_H, STTS751_REG_LLIM_L);
			if (ret)
				return ret;
			priv->config &= ~STTS751_CONF_EVENT_DIS;
		}

		if (priv->gen_therm) {
			ret = stts751_set_temp_reg(priv, priv->therm, false,
						STTS751_REG_TLIM, 0);
			if (ret)
				return ret;

			ret = stts751_set_temp_reg(priv, priv->hyst, false,
						STTS751_REG_HYST, 0);
			if (ret)
				return ret;
		}

		priv->config &= ~STTS751_CONF_STOP;
		ret = i2c_smbus_write_byte_data(priv->client,
						STTS751_REG_CONF, priv->config);
	}
	return ret;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_input, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_event_min, S_IWUSR | S_IRUGO, show_min, set_min, 0);
static SENSOR_DEVICE_ATTR(temp1_event_max, S_IWUSR | S_IRUGO, show_max, set_max, 0);
static SENSOR_DEVICE_ATTR(temp1_event_min_alert, S_IWUSR | S_IRUGO,
			show_min_alert, set_min_alert, 0);
static SENSOR_DEVICE_ATTR(temp1_event_max_alert, S_IWUSR | S_IRUGO,
			show_max_alert, set_max_alert, 0);
static SENSOR_DEVICE_ATTR(temp1_therm, S_IWUSR | S_IRUGO, show_therm, set_therm, 0);
static SENSOR_DEVICE_ATTR(temp1_therm_hyst, S_IWUSR | S_IRUGO, show_hyst, set_hyst, 0);
static SENSOR_DEVICE_ATTR(update_interval, S_IWUSR | S_IRUGO,
			show_interval, set_interval, 0);

/* always present */
static struct attribute *stts751_temp_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};

static struct attribute_group stts751_temp_group = {
        .attrs = stts751_temp_attrs,
};

/* present when therm pin or event pin are connected */
static struct attribute *stts751_interval_attrs[] = {
	&sensor_dev_attr_update_interval.dev_attr.attr,
	NULL
};

static struct attribute_group stts751_interval_group = {
        .attrs = stts751_interval_attrs,
};

/* present when event pin is connected */
static struct attribute *stts751_event_attrs[] = {
	&sensor_dev_attr_temp1_event_min.dev_attr.attr,
	&sensor_dev_attr_temp1_event_max.dev_attr.attr,
	&sensor_dev_attr_temp1_event_min_alert.dev_attr.attr,
	&sensor_dev_attr_temp1_event_max_alert.dev_attr.attr,
	NULL
};

static struct attribute_group stts751_event_group = {
        .attrs = stts751_event_attrs,
};

/* present when therm pin is connected */
static struct attribute *stts751_therm_attrs[] = {
	&sensor_dev_attr_temp1_therm.dev_attr.attr,
	&sensor_dev_attr_temp1_therm_hyst.dev_attr.attr,
	NULL
};

static struct attribute_group stts751_therm_group = {
        .attrs = stts751_therm_attrs,
};

static int stts751_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct stts751_priv *priv;
	int ret;
	int groups_idx = 0;
	struct device_node *of_node = client->dev.of_node;
// TODO
//	struct stts751_platform_data *pdata = dev_get_platdata(&client->dev);
//
	priv = devm_kzalloc(&client->dev, sizeof(struct stts751_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);
	mutex_init(&priv->access_lock);

	/* default to 2 samples per second */
	priv->interval = 5;
	/* default to timeout enable, as per chip default */
	priv->smbus_timeout = true;
	priv->last_update = 0;
	priv->data_valid = false;
	priv->max_alert = false;
	priv->min_alert = false;

        if (of_node) {
		priv->gen_therm = of_property_read_bool(of_node, "has-therm");
		if (priv->gen_therm) {
			ret = of_property_read_s32(of_node, "therm-val",
						&priv->therm);
			if (ret) {
				dev_warn(&client->dev,
					"thermal HW pin active, but therm-val not set");
				priv->therm = STTS751_THERM_DEFAULT;
			}

			ret = of_property_read_s32(of_node, "therm-hyst",
						&priv->hyst);
			if (ret) {
				dev_warn(&client->dev,
					"thermal HW pin active, but therm-hyst not set");
				priv->hyst = STTS751_HYST_DEFAULT;
			}
		}

		priv->gen_event = of_property_read_bool(of_node, "has-event");
		if (priv->gen_event) {
			ret = of_property_read_s32(of_node, "event-max",
						&priv->event_max);
			if (ret) {
				dev_warn(&client->dev,
					"event HW pin active, but event-max not set");
				priv->event_max = STTS751_EVENT_MAX_DEFAULT;
			}

			ret = of_property_read_s32(of_node, "event-min",
						&priv->event_min);
			if (ret) {
				dev_warn(&client->dev,
					"event HW pin active, but event-min not set");
				priv->event_min = STTS751_EVENT_MIN_DEFAULT;
			}
		}

		priv->smbus_timeout =
			!of_property_read_bool(of_node, "smbus-timeout-disable");
// TODO
//	} else if (pdata) {
//		FROM pdata: gen_event, gen_therm
//		FROM pdata: optional irq for gen_event and gen_them
	} else {
		dev_notice(&client->dev, "Neither platform data or DT data provided\n"
			"event/therm lines will be disabled");
		priv->gen_therm = false;
		priv->gen_event = false;
		priv->interval = STTS751_INTERVAL_MANUAL;
	}

	dev_dbg(&client->dev, "gen_event: %s, gen_therm: %s",
		priv->gen_event ? "YES" : "NO",
		priv->gen_therm ? "YES" : "NO");

	priv->groups[groups_idx++] = &stts751_temp_group;

	if (priv->gen_therm || priv->gen_event)
		priv->groups[groups_idx++] = &stts751_interval_group;
	else
		priv->interval = STTS751_INTERVAL_MANUAL;

	if (priv->gen_therm)
		priv->groups[groups_idx++] = &stts751_therm_group;

	if (priv->gen_event)
		priv->groups[groups_idx++] = &stts751_event_group;

	priv->groups[groups_idx] = NULL;

	ret = stts751_init_chip(priv);
	if (ret)
		return ret;

	priv->dev = devm_hwmon_device_register_with_groups(&client->dev,
							client->name, priv,
							priv->groups);
	return PTR_ERR_OR_ZERO(priv->dev);
}

static const struct i2c_device_id stts751_id[] = {
	{ "st,stts751", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stts751_id);

static struct i2c_driver stts751_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DEVNAME,
	},
	.probe		= stts751_probe,
	.id_table	= stts751_id,
	.detect		= stts751_detect,
	.alert		= stts751_alert,
	.address_list	= normal_i2c,
};

module_i2c_driver(stts751_driver);

MODULE_AUTHOR("Andrea Merello <andrea.merello@gmail.com>");
MODULE_DESCRIPTION("STTS751 sensor driver");
MODULE_LICENSE("GPL");
