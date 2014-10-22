/* rtc-ab-rtcmc: RTC Driver for Abracon AB-RTCMC-32.768KHZ-EOZ9-S2-D-A-T
 *
 * This may work for other Abracon RTC.
 *
 * Based on rtc-generic: RTC driver using the generic RTC abstraction
 *
 * Copyright (C) 2014 Joshua Lamorie, Xiphos Systems Corp. <jpl@xiphos.ca>
 * Copyright (C) 2008 Kyle McMartin <kyle@mcmartin.ca>
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

#define ABRTCMC_REG_ALARM_SECOND        (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_SECOND)
#define ABRTCMC_REG_ALARM_MINUTE        (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_MINUTE)
#define ABRTCMC_REG_ALARM_HOUR          (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_HOUR)
#define ABRTCMC_REG_ALARM_DAY           (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_DAY)
#define ABRTCMC_REG_ALARM_WEEKDAY       (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_WEEKDAY)
#define ABRTCMC_REG_ALARM_MONTH         (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_MONTH)
#define ABRTCMC_REG_ALARM_YEAR          (ABRTCMC_REG_ALARM_BASE + ABRTCMC_ALARM_YEAR)

/* Timer Page 00011 */
#define ABRTCMC_REG_TIMER_LOW         0x18
#define ABRTCMC_REG_TIMER_HIGH        0x19

/* Temp. Page 00100 */
#define ABRTCMC_REG_TEMPERATURE       0x20

/* EEPROM User Page 00101 */
#define ABRTCMC_REG_EEPROM_USER0      0x28
#define ABRTCMC_REG_EEPROM_USER1      0x29

/* EEPROM Control Page 00110 */
#define ABRTCMC_REG_EEPROM_CONTROL    0x30
#define ABRTCMC_REG_XTAL_OFFSET       0x31
#define ABRTCMC_REG_XTAL_COEF         0x32
#define ABRTCMC_REG_XTAL_T0           0x33

/* RAM Page 00111 */
#define ABRTCMC_REG_RAM_START         0x38
#define ABRTCMC_REG_RAM_LEN           0x08

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

static int abrtcmc_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	uint8_t buf[7];
	int ret;

	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CLOCK_BASE, buf, sizeof(buf));
	if (ret)
		return ret;

  // TODO: Add voltage checking

	dev_dbg(&client->dev,
		"%s: raw data is sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x,  wday=%02x, mon=%02x, year=%02x,",
		__func__,
		buf[ABRTCMC_SECONDS],
		buf[ABRTCMC_MINUTES],
		buf[ABRTCMC_HOURS],
		buf[ABRTCMC_DAYS],
		buf[ABRTCMC_WEEKDAYS],
		buf[ABRTCMC_MONTHS],
		buf[ABRTCMC_YEARS]);

	tm->tm_sec = bcd2bin(buf[ABRTCMC_SECONDS] & 0x7F);
	tm->tm_min = bcd2bin(buf[ABRTCMC_MINUTES] & 0x7F);
	tm->tm_hour = bcd2bin(buf[ABRTCMC_HOURS] & 0x3F);
	tm->tm_mday = bcd2bin(buf[ABRTCMC_DAYS] & 0x3F);
	tm->tm_wday = (buf[ABRTCMC_WEEKDAYS] & 0x07) - 1;
	tm->tm_mon = bcd2bin(buf[ABRTCMC_MONTHS] & 0x1F);
	tm->tm_year = bcd2bin(buf[ABRTCMC_YEARS] & 0x7f) + 100;

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* The clock can give out invalid datetime, but we cannot return
	 * -EINVAL otherwise hwclock will refuse to set the time on bootup. */
	if (rtc_valid_tm(tm) < 0)
		dev_err(&client->dev, "retrieved date and time is invalid.\n");

	return 0;
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

	/* hours, minutes and seconds */
	buf[ABRTCMC_SECONDS] = bin2bcd(tm->tm_sec);
	buf[ABRTCMC_MINUTES] = bin2bcd(tm->tm_min);
	buf[ABRTCMC_HOURS] = bin2bcd(tm->tm_hour) ;

	buf[ABRTCMC_DAYS] = bin2bcd(tm->tm_mday);


	/* month, 1 - 12 */
	buf[ABRTCMC_MONTHS] = bin2bcd(tm->tm_mon);

	/* year and century */
	buf[ABRTCMC_YEARS] = bin2bcd(tm->tm_year % 100);

	buf[ABRTCMC_WEEKDAYS] = (tm->tm_wday & 0x07) + 1;

	/* write register's data */
	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		ret = abrtcmc_write_reg(client, ABRTCMC_REG_CLOCK_BASE + i,
					 buf[i]);
		if (ret)
			return -EIO;
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

static int abrtcmc_get_alarm(struct i2c_client *client, struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	uint8_t buf[7];
	int ret, ctrl;

	ret = abrtcmc_read_regs(client, ABRTCMC_REG_ALARM_BASE, buf, sizeof(buf));
	if (ret)
		return ret;

	tm->tm_sec = bcd2bin(buf[ABRTCMC_ALARM_SECOND] & 0x7F);
	tm->tm_min = bcd2bin(buf[ABRTCMC_ALARM_MINUTE] & 0x7F);
	tm->tm_hour = bcd2bin(buf[ABRTCMC_ALARM_HOUR] & 0x3F);
	tm->tm_mday = bcd2bin(buf[ABRTCMC_ALARM_DAY] & 0x3F);
	tm->tm_wday = (buf[ABRTCMC_ALARM_WEEKDAY] & 0x07) - 1;
	tm->tm_mon = bcd2bin(buf[ABRTCMC_ALARM_MONTH] & 0x1F);
	tm->tm_year = bcd2bin(buf[ABRTCMC_ALARM_YEAR] & 0x7f) + 100;

	/* Checking if alarm enabled */
	ctrl = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INT, buf, 1);

	if(ctrl){
		dev_err(&client->dev, "%s: reading alarm control failed\n",
                        __func__);

		return ctrl;
	}

	alarm->enabled = (buf[0] & 0x01) ? 1 : 0 ;

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

static int abrtcmc_set_alarm(struct i2c_client *client, struct rtc_wkalrm *alarm)
{
	struct rtc_time *const tm = &alarm->time;
	size_t i;
	int ret;
	uint8_t buf[7];

	/* hours, minutes and seconds */
	buf[ABRTCMC_ALARM_SECOND]  = bin2bcd(tm->tm_sec);
	buf[ABRTCMC_ALARM_MINUTE] = bin2bcd(tm->tm_min);
	buf[ABRTCMC_ALARM_HOUR]   = bin2bcd(tm->tm_hour);

	buf[ABRTCMC_ALARM_DAY] = bin2bcd(tm->tm_mday);


	/* month, 1 - 12 */
	buf[ABRTCMC_ALARM_MONTH] = bin2bcd(tm->tm_mon);

	/* year and century */
	buf[ABRTCMC_ALARM_YEAR] = bin2bcd(tm->tm_year % 100);

	buf[ABRTCMC_ALARM_WEEKDAY] = (tm->tm_wday & 0x07) + 1;

	/* write register's data */
	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		// Add the enable bit for all registers since this is provided
		// as an absolute wakeup alarm
		ret = abrtcmc_write_reg(client, ABRTCMC_REG_ALARM_BASE + i,
					 buf[i] | 0x80);
		if (ret)
			return -EIO;
	}

	/* Enable the alarm */
	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INT, buf, 1);
	if (ret)
		return ret;

	if (alarm->enabled)
		buf[1] = buf[0] | 0x01;
	else
		buf[1] = buf[0] & ~(0x01);

	ret = abrtcmc_write_reg(client,ABRTCMC_REG_CONTROL_INT, buf[1]);
	if (ret)
		return ret;

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
	ret = abrtcmc_read_regs(to_i2c_client(dev->parent), ABRTCMC_REG_TEMPERATURE, &data, 1);
	if (ret)
		return ret;
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

static int abrtcmc_rtc_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct ab_rtcmc *abrtcmc;
	int ret;
	uint8_t buf;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	abrtcmc = devm_kzalloc(&client->dev, sizeof(struct ab_rtcmc),
				GFP_KERNEL);
	if (!abrtcmc)
		return -ENOMEM;

	dev_dbg(&client->dev, "chip found\n");

	i2c_set_clientdata(client, abrtcmc);

	abrtcmc->rtc = devm_rtc_device_register(&client->dev,
					abrtcmc_driver.driver.name,
					&abrtcmc_rtc_ops, THIS_MODULE);

	/* deactivating interrupt */
	abrtcmc->rtc->uie_unsupported = 1;

	/* Clearing Alarm flag */
	ret = abrtcmc_read_regs(client, ABRTCMC_REG_CONTROL_INTFLAG, &buf, 1);
	if (ret)
		return ret;

	buf &= ~(0x01);

	ret = abrtcmc_write_reg(client,ABRTCMC_REG_CONTROL_INTFLAG, buf);
	if (ret)
		return ret;

	/* Creating sysfs */
	ret = sysfs_create_group(&abrtcmc->rtc->dev.kobj,
						&abrtcmc_rtc_sysfs_files);

	if (ret)
		return ret;

	return PTR_ERR_OR_ZERO(abrtcmc->rtc);
}

static int
abrtcmc_remove(struct i2c_client *client)
{
	struct ab_rtcmc *abrtcmc;

	abrtcmc = devm_kzalloc(&client->dev, sizeof(struct ab_rtcmc),
				GFP_KERNEL);
	if(!abrtcmc)
		return -ENOMEM;

	sysfs_remove_group(&abrtcmc->rtc->dev.kobj, &abrtcmc_rtc_sysfs_files);

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
