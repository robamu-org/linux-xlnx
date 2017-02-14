/*
 * rtc-ab-rtcmc: RTC Driver for Abracon AB-RTCMC-32.768KHZ-EOZ9-S2-D-A-T
 *
 * Detailed datasheet of the chip is available here:
 * http://www.abracon.com/realtimeclock/AB-RTCMC-32.768kHz-EOZ9-S3-Application-Manual.pdf
 *
 * This may work for other Abracon RTC.
 *
 * Based on rtc-generic: RTC driver using the generic RTC abstraction
 *
 * Copyright (C) 2017 William Bourque, Xiphos Systems Corp. <wfb@xiphos.ca>
 * Copyright (C) 2014 Joshua Lamorie, Xiphos Systems Corp. <jpl@xiphos.ca>
 * Copyright (C) 2008 Kyle McMartin <kyle@mcmartin.ca>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bcd.h>

/* AB-RTCMC Register Offsets */

/* Control Page 00000 */
#define ABRTCMC_REG_CONTROL_I         0x00
#define ABRTCMC_REG_CONTROL_INT       0x01
#define ABRTCMC_REG_CONTROL_INTFLAG   0x02
#define ABRTCMC_REG_CONTROL_STATUS    0x03
#define ABRTCMC_REG_CONTROL_RESET     0x04

/* Clock Page 00001 */
#define ABRTCMC_REG_CLOCK_BASE 0x08

#define ABRTCMC_SECONDS           0x00
#define ABRTCMC_MINUTES           0x01
#define ABRTCMC_HOURS             0x02
#define ABRTCMC_DAYS              0x03
#define ABRTCMC_WEEKDAYS          0x04
#define ABRTCMC_MONTHS            0x05
#define ABRTCMC_YEARS             0x06

#define ABRTCMC_REG_SECONDS       (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_SECONDS)
#define ABRTCMC_REG_MINUTES       (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_MINUTES)
#define ABRTCMC_REG_HOURS         (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_HOURS)
#define ABRTCMC_REG_DAYS          (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_DAYS)
#define ABRTCMC_REG_WEEKDAYS      (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_WEEKDAYS)
#define ABRTCMC_REG_MONTHS        (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_MONTHS)
#define ABRTCMC_REG_YEARS         (ABRTCMC_REG_CLOCK_BASE + ABRTCMC_YEARS)


/* Alarm Page 00010 */
#define ABRTCMC_REG_ALARM_BASE 0x10

#define ABRTCMC_ALARM_SECOND      0x00
#define ABRTCMC_ALARM_MINUTE      0x01
#define ABRTCMC_ALARM_HOUR        0x02
#define ABRTCMC_ALARM_DAY         0x03
#define ABRTCMC_ALARM_WEEKDAY     0x04
#define ABRTCMC_ALARM_MONTH       0x05
#define ABRTCMC_ALARM_YEAR        0x06

#define ABRTCMC_REG_ALARM_SECOND  (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_SECOND)
#define ABRTCMC_REG_ALARM_MINUTE  (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_MINUTE)
#define ABRTCMC_REG_ALARM_HOUR    (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_HOUR)
#define ABRTCMC_REG_ALARM_DAY     (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_DAY)
#define ABRTCMC_REG_ALARM_WEEKDAY (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_WEEKDAY)
#define ABRTCMC_REG_ALARM_MONTH   (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_MONTH)
#define ABRTCMC_REG_ALARM_YEAR    (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_YEAR)

/* Timer Page 00011 */
#define ABRTCMC_REG_TIMER_LOW     0x18
#define ABRTCMC_REG_TIMER_HIGH    0x19

/* Temp. Page 00100 */
#define ABRTCMC_REG_TEMPERATURE   0x20

/* EEPROM User Page 00101 */
#define ABRTCMC_REG_EEPROM_USER0   0x28
#define ABRTCMC_REG_EEPROM_USER1   0x29

/* EEPROM Control Page 00110 */
#define ABRTCMC_REG_EEPROM_CONTROL 0x30
#define ABRTCMC_REG_XTAL_OFFSET    0x31
#define ABRTCMC_REG_XTAL_COEF      0x32
#define ABRTCMC_REG_XTAL_T0        0x33

/* RAM Page 00111 */
#define ABRTCMC_REG_RAM_START      0x38
#define ABRTCMC_REG_RAM_LEN        0x08

static struct i2c_driver abrtcmc_driver;

struct ab_rtcmc {
	struct rtc_device *rtc;
};

static int abrtcmc_read_regs(struct i2c_client *client, uint8_t reg,
			      uint8_t *data, size_t n)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= data
		},		/* setup read ptr */
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= n,
			.buf	= data
		}
	};

	int ret;

	data[0] = reg;
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: read error, ret=%d\n",
			__func__, ret);
		return -EIO;
	}

	return 0;
}


static int abrtcmc_write_reg(struct i2c_client *client,
			      uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };
	int err;

	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) {
		dev_err(&client->dev,
			"%s: err=%d addr=%02x, data=%02x\n",
			__func__, err, data[0], data[1]);
		return -EIO;
	}

	return 0;
}

static int abrtcmc_rtc_time_from_buffer(struct i2c_client *client, 
                                          uint8_t buf[7], struct rtc_time *tm)
{
	dev_dbg(&client->dev,
	        "%s: raw i2c sec=0x%02X, min=0x%02X, hr=0x%02X, "
	        "mday=0x%02X,  wday=0x%02X, mon=0x%02X, year=0x%02X,",
	        __func__,
	        buf[ABRTCMC_SECONDS],
	        buf[ABRTCMC_MINUTES],
	        buf[ABRTCMC_HOURS],
	        buf[ABRTCMC_DAYS],
	        buf[ABRTCMC_WEEKDAYS],
	        buf[ABRTCMC_MONTHS],
	        buf[ABRTCMC_YEARS]);
	
	/* Seconds between 0 and 59 */
	tm->tm_sec = bcd2bin(buf[ABRTCMC_SECONDS] & 0x7F);
	/* Minutes between 0 and 59 */
	tm->tm_min = bcd2bin(buf[ABRTCMC_MINUTES] & 0x7F);
	/* Only support 24 hours mode: hour mode (bit6) MUST be 0
	   Hours must be between 0 and 23 */
	tm->tm_hour = bcd2bin(buf[ABRTCMC_HOURS] & 0x3F);
	/* Day of the Month between 1 and 31 */
	tm->tm_mday = bcd2bin(buf[ABRTCMC_DAYS] & 0x3F);
	/* RTC stores week day from 1 to 7; tm_wday must be 0 to 6
	   Day of the Week between 0 and 6  */
	tm->tm_wday = (buf[ABRTCMC_WEEKDAYS] & 0x07) - 1;
	/* RTC stores month from 1 to 12; tm_mon must be 0 to 11
	   Month between 0 and 11 */
	tm->tm_mon = bcd2bin(buf[ABRTCMC_MONTHS] & 0x1F) - 1;
	/* RTC only support 2000 to 2079, it stores 0 to 79; tm_year starts
	       at 1900, so add 100 to the result
	   Year between 100 and 179 */
	tm->tm_year = bcd2bin(buf[ABRTCMC_YEARS] & 0x7f) + 100;

	/* Validate converted values from buffer make sense in rtc_time */
	if (rtc_valid_tm(tm) < 0) {
		goto invalid_values;
	}
	
	dev_dbg(&client->dev, "%s: rtc_time secs=%d, mins=%d, hours=%d, "
	        "mday=%d, mon=%d, year=%d, wday=%d\n",
	        __func__,
	        tm->tm_sec, tm->tm_min, tm->tm_hour,
	        tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	return 0;

invalid_values:
	tm->tm_sec  = 0;
	tm->tm_min  = 0;
	tm->tm_hour = 0;
	tm->tm_mday = 1;
	tm->tm_wday = 0;
	tm->tm_mon  = 0;
	tm->tm_year = 100;

	return -EINVAL;
}

static int abrtcmc_rtc_time_to_buffer(struct rtc_time *tm, uint8_t buf[7])
{
	/* Make sure the actual rtc_time passed is coherent */
	if (rtc_valid_tm(tm) < 0) {
		return -EINVAL;
	}

	/* Hour, minutes and seconds between 0 and 59 */
	buf[ABRTCMC_SECONDS] = bin2bcd(tm->tm_sec);
	buf[ABRTCMC_MINUTES] = bin2bcd(tm->tm_min);
	buf[ABRTCMC_HOURS]   = bin2bcd(tm->tm_hour);

	/* We do not support storing in 12 hours:
	   Force 24 hours mode (bit 6 == 0) on RTC */
	buf[ABRTCMC_HOURS] &= 0x3F;

	/* Day of the Month between 1 and 31 */
	buf[ABRTCMC_DAYS] = bin2bcd(tm->tm_mday);

	/* tm_mon is 0 to 11
	   RTC week day are 1 to 12: add 1 */
	buf[ABRTCMC_MONTHS] = (bin2bcd(tm->tm_mon) + 1);

	/* tm_year 0 is at at 1900
	   RTC 0 at 2000 to 2079, supports only 0 to 79:
	   Modulo 100 tm_year, should be ok until 2079 */
	buf[ABRTCMC_YEARS] = bin2bcd(tm->tm_year % 100);

	/* tm_wday must be 0 to 6
	   RTC week day is 1 to 7: add 1 */
	buf[ABRTCMC_WEEKDAYS] = (tm->tm_wday & 0x07) + 1;

	return 0;
}

static int abrtcmc_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	uint8_t buf[7];
	int ret;

	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CLOCK_BASE, buf,
	                          sizeof(buf));
	if (ret) {
		return ret;
	}

	// TODO: Add voltage checking

	/* Convert buffer into rtc_time struct */
	ret = abrtcmc_rtc_time_from_buffer(client, buf, tm);
	if (ret < 0) {
		dev_err(&client->dev, "RTC datetime values are invalid.\n");
	}

	return ret;
}

static int abrtcmc_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	size_t i;
	int ret;
	uint8_t buf[7];

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
	        "mday=%d, mon=%d, year=%d, wday=%d\n",
	        __func__,
	        tm->tm_sec, tm->tm_min, tm->tm_hour,
	        tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* Convert rtc_time to RTC buffer */
	ret = abrtcmc_rtc_time_to_buffer(tm, buf);
	if (ret) {
		return ret;
	}

	/* Write register's data */
	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		ret = abrtcmc_write_reg(client, ABRTCMC_REG_CLOCK_BASE + i,
					 buf[i]);
		if (ret) {
			return -EIO;
		}
	}

	return 0;
}

static int abrtcmc_get_time(struct device *dev, struct rtc_time *tm)
{
	return abrtcmc_get_datetime(to_i2c_client(dev), tm);
}

static int abrtcmc_set_time(struct device *dev, struct rtc_time *tm)
{
	return abrtcmc_set_datetime(to_i2c_client(dev), tm);
}

static int abrtcmc_get_alarm(struct i2c_client *client,
                               struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	uint8_t buf[7];
	int ret;
	int ctrl;

	ret = abrtcmc_read_regs(client, ABRTCMC_REG_ALARM_BASE, buf,
	                         sizeof(buf));
	if (ret) {
		return ret;
	}

	/* Convert buffer to rtc_time struct */
	ret = abrtcmc_rtc_time_from_buffer(client, buf, tm);
	if (ret != 0) {
		dev_err(&client->dev, "RTC alarm values are invalid.\n");
		return ret;
	}

	/* Checking if alarm enabled */
	ctrl = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INT, buf, 1);
	if(ctrl){
		dev_err(&client->dev, "%s: reading alarm control failed\n",
                        __func__);

		return ctrl;
	}
	alarm->enabled = (buf[0] & 0x01) ? 1 : 0;

	/* Checking if an alarm is pending */
	ctrl = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INTFLAG, buf, 1);
	if(ctrl){
		dev_err(&client->dev, "%s: reading alarm flags failed\n",
                        __func__);

		return ctrl;
	}
	alarm->pending = (buf[0] & 0x01) ? 1 : 0;

	return 0;
}

static int abrtcmc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	return abrtcmc_get_alarm(to_i2c_client(dev), alarm);
}

static int abrtcmc_set_alarm(struct i2c_client *client,
                               struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	size_t i;
	int ret;
	uint8_t buf[7];

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
	        "mday=%d, mon=%d, year=%d, wday=%d\n",
	        __func__,
	        tm->tm_sec, tm->tm_min, tm->tm_hour,
	        tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* Convert rtc_time to RTC buffer */
	ret = abrtcmc_rtc_time_to_buffer(tm, buf);
	if (ret) {
		return ret;
	}

	/* write register's data */
	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		// Add the enable bit for all registers since this is provided
		// as an absolute wakeup alarm
		ret = abrtcmc_write_reg(client, ABRTCMC_REG_ALARM_BASE + i,
					 buf[i] | 0x80);
		if (ret) {
			return -EIO;
		}
	}

	/* Enable the alarm */
	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INT, buf, 1);
	if (ret) {
		return ret;
	}

	if (alarm->enabled) {
		buf[1] = buf[0] | 0x01;
	} else {
		buf[1] = buf[0] & ~(0x01);
	}

	ret = abrtcmc_write_reg(client,ABRTCMC_REG_CONTROL_INT, buf[1]);
	if (ret) {
		return ret;
	}

	return 0;
}


static int abrtcmc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	return abrtcmc_set_alarm(to_i2c_client(dev), alarm);
}

static const struct rtc_class_ops abrtcmc_rtc_ops = {
	.read_time = abrtcmc_get_time,
	.set_time = abrtcmc_set_time,
	.read_alarm = abrtcmc_rtc_read_alarm,
	.set_alarm = abrtcmc_rtc_set_alarm,
};

/* sysfs interface */
static ssize_t abrtcmc_sysfs_show_temperature(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int ret;
	int8_t data;

	/* sysfs build from the kobj of the rtc device not from the i2c*/
	ret = abrtcmc_read_regs(to_i2c_client(dev->parent),
	                         ABRTCMC_REG_TEMPERATURE, &data, 1);
	if (ret) {
		return ret;
	}
	data -= 60;
	return sprintf(buf, "%d\n", data);
}

static DEVICE_ATTR(temperature, 0444, abrtcmc_sysfs_show_temperature, NULL);

static struct attribute *abrtcmc_attrs [] = {
	&dev_attr_temperature.attr,
	NULL
};

static const struct attribute_group abrtcmc_rtc_sysfs_files = {
	.attrs = abrtcmc_attrs,
};

static int abrtcmc_rtc_sanitize_register(struct i2c_client *client)
{
	struct rtc_time tm;
	struct rtc_wkalrm alrm;
	int ret = 0;
	uint8_t ctrl[1];

	/* Check "PON" PowerOn register : it indicates if RTC had lost power
	   If so, registers are not to be trusted:  wipe everything */
	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_STATUS, ctrl, 1);
	if (ret) {
		return ret;
	}
	/* Anything but 0 indicate RTC lost power */
	if ( (ctrl[0] & 0x20) == 0) {
		dev_dbg(&client->dev, 
		        "%s: PON bit inactive (control register is %02X)\n",  
		        __func__,
		        ctrl[0]);
		return 0;
	}
	
	dev_dbg(&client->dev, 
	        "%s: PON bit active : RTC lost power (control register is %02X)\n",  
	        __func__,
	        ctrl[0]);

	/* Set rtc_time to 'zero' value */
	tm.tm_sec  = 0;
	tm.tm_min  = 0;
	tm.tm_hour = 0;
	tm.tm_mday = 1;
	tm.tm_wday = 0;
	tm.tm_mon  = 0;
	tm.tm_year = 100;
	/* Same for alarm */
	alrm.enabled = 0;
	alrm.pending = 0;
	memcpy(&alrm.time, &tm, sizeof(struct rtc_time));

	/* Reset datetime to base value */
	ret = abrtcmc_set_datetime(client, &tm);
	if (ret) {
		return ret;
	}

	/* Reset Alarm time */
	ret = abrtcmc_set_alarm(client, &alrm);
	if (ret) {
		return ret;
	}

	/* Reset PON : it won't reset unless there is a power cut */
	ctrl[0] &= ~(0x20);
	dev_dbg(&client->dev, 
	        "%s: Reseting PON bit (control register to %02X)\n",  
	        __func__,
	        ctrl[0]);
	ret = abrtcmc_write_reg(client,ABRTCMC_REG_CONTROL_STATUS, ctrl[0]);
	if (ret) {
		return ret;
	}

	return 0;
}

static int abrtcmc_rtc_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct ab_rtcmc *abrtcmc;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	abrtcmc = devm_kzalloc(&client->dev, sizeof(struct ab_rtcmc),
				GFP_KERNEL);
	if (!abrtcmc) {
		return -ENOMEM;
	}

	dev_dbg(&client->dev, "AB-RTCMC chip found\n");

	i2c_set_clientdata(client, abrtcmc);

	/* This particular RTC initialize its register at random values (!!)
	 * They MUST be sanitized to clear any invalid values */
	ret = abrtcmc_rtc_sanitize_register(client);
	if (ret) {
		return ret;
	}

	/* Register driver to the kernel */
	abrtcmc->rtc = devm_rtc_device_register(&client->dev,
	                          abrtcmc_driver.driver.name,
	                          &abrtcmc_rtc_ops,
	                          THIS_MODULE);
	/* Deactivating interrupt */
	abrtcmc->rtc->uie_unsupported = 1;

	/* Creating sysfs */
	ret = sysfs_create_group(&abrtcmc->rtc->dev.kobj,
						&abrtcmc_rtc_sysfs_files);
	if (ret) {
		return ret;
	}

	return PTR_ERR_OR_ZERO(abrtcmc->rtc);
}

static int abrtcmc_remove(struct i2c_client *client)
{
	struct ab_rtcmc *abrtcmc;

	if (client != NULL) {
		abrtcmc = devm_kzalloc(&client->dev, sizeof(struct ab_rtcmc),
				GFP_KERNEL);
	
		if(!abrtcmc) {
			return -ENOMEM;
		}
		
		if (abrtcmc->rtc) {
			sysfs_remove_group(&abrtcmc->rtc->dev.kobj, 
		                           &abrtcmc_rtc_sysfs_files);
		}
	}

	return 0;
}

static const struct i2c_device_id abrtcmc_id[] = {
	{ "ab-rtcmc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, abrtcmc_id);


static struct i2c_driver abrtcmc_driver = {
	.driver = {
		.name = "rtc-ab-rtcmc",
	},
	.probe = abrtcmc_rtc_probe,
	.remove = abrtcmc_remove,
	.id_table = abrtcmc_id,
};

module_i2c_driver(abrtcmc_driver);

MODULE_AUTHOR("Joshua Lamorie <jpl@xiphos.ca>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AB-RTCMC RTC driver");
MODULE_ALIAS("platform:rtc-ab-rtcmc");
