/**
 * Watchdog driver for the Xiphos ProAsic3 Watchdog logic
 *
 * Copyright (C) 2012 - 2021 Xiphos Systems Corp.
 * All rights reserved.
 *
 * Author: Joshua Lamorie <joshua.lamorie@xiphos.ca>
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sysfs.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#endif

#define DRIVER_NAME "xsc_wdt"
#define DRIVER_DESCRIPTION "Xiphos ProAsic3 Watchdog"

#define REG_ENABLE1 0
#define XSC_LOGIC_WDT_ENABLE1_MAGIC 0xb45efa83
#define REG_ENABLE2 0x4
#define XSC_LOGIC_WDT_ENABLE2_MAGIC 0x50ad54ee
#define REG_NO_WAY_OUT 0x08
#define REG_TICKLE 0x0c
#define REG_COMPINT 0x10
#define REG_COMPRST 0x14
#define REG_COUNTER 0x18
#define REG_STATUS 0x20

#define XSC_LOGIC_WDT_WD_RESET_ACTIVE 0x80000000
#define XSC_LOGIC_WDT_NOWAYOUT_ACTIVE 0x4
#define XSC_LOGIC_WDT_ENABLE2_ACTIVE 0x2
#define XSC_LOGIC_WDT_ENABLE1_ACTIVE 0x1

struct xsc_logic_wdt_dev {
	dev_t dev_id;
	struct cdev cdev;
	struct device *dev;
	int irq;
	phys_addr_t base_address;
	void __iomem * base;
	struct watchdog_info wdt_info;
	int status;
	uint64_t total_timeout;
	uint64_t pre_timeout;
	uint32_t clk_freq;
	uint32_t counter_divider;
	unsigned long driver_open;
	unsigned long watchdog_active;
	unsigned int magic_close;
};

static dev_t xsc_logic_wdt_dev_id;
static struct class *xsc_logic_wdt_class;

static irqreturn_t xsc_logic_wdt_interrupt(int irq, void * dev_id)
{

	pr_alert("Interrupt received\n");
	//TODO: Send signal to owning process of watchdog device
	return IRQ_HANDLED;
}

static inline u32 xsc_logic_wdt_get_reg(struct xsc_logic_wdt_dev *xsc_logic_wdt,
					int offset)
{
	return ioread32(xsc_logic_wdt->base + offset);
}

static inline void xsc_logic_wdt_set_reg(struct xsc_logic_wdt_dev *xsc_logic_wdt,
						int offset, u32 x)
{
	pr_debug("0x%04x <- 0x%08x\n", offset, x);
	return iowrite32(x,xsc_logic_wdt->base + offset);
}

static inline void xsc_logic_wdt_enable_toggle(struct xsc_logic_wdt_dev * xsc_logic_wdt)
{
	xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_ENABLE1,XSC_LOGIC_WDT_ENABLE1_MAGIC);
	xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_ENABLE2,XSC_LOGIC_WDT_ENABLE2_MAGIC);
}

static inline uint32_t xsc_logic_wdt_get_hz(struct xsc_logic_wdt_dev * xsc_logic_wdt) {
	int wdt_clock_hz;
	wdt_clock_hz = xsc_logic_wdt->clk_freq;
	return wdt_clock_hz;
}

static inline uint64_t xsc_logic_wdt_calc_counts(struct xsc_logic_wdt_dev * xsc_logic_wdt,
							int seconds) {
	uint32_t wdt_clock_hz = xsc_logic_wdt_get_hz(xsc_logic_wdt);
	return (uint64_t)seconds * (uint64_t)wdt_clock_hz;
}

static inline int xsc_logic_wdt_calc_seconds(struct xsc_logic_wdt_dev * xsc_logic_wdt,
						uint64_t counts) {
	uint32_t wdt_clock_hz = xsc_logic_wdt_get_hz(xsc_logic_wdt);
        uint64_t counts_temp = counts;
	uint32_t div_rem;
	div_rem = do_div(counts_temp,wdt_clock_hz);

	return (int) (counts_temp);
}


static ssize_t xsc_logic_wdt_write(struct file *f, const char __user *data,
						size_t len, loff_t *ppos)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt;

	xsc_logic_wdt = (struct xsc_logic_wdt_dev *)f->private_data;

	if (!xsc_logic_wdt->watchdog_active)
		return -EINVAL;
	xsc_logic_wdt->magic_close = 0;
	if (len) {
		size_t i;
		// write to REG_TICKLE
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_TICKLE, 0);
		for (i=0;i<len;i++) {
			char c;
			if (get_user(c,data + i))
				return -EFAULT;
			if (c == 'V')
				xsc_logic_wdt->magic_close = 1;
		}
	}
	return len;
}

static long xsc_logic_wdt_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *) arg;
	int __user *p = argp;
	int rc;
	int val;
	struct xsc_logic_wdt_dev *xsc_logic_wdt;

	xsc_logic_wdt = (struct xsc_logic_wdt_dev *)f->private_data;
	xsc_logic_wdt->magic_close = 0;

	switch (cmd)
	{
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &xsc_logic_wdt->wdt_info,
			sizeof(xsc_logic_wdt->wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		val = 0;
		if (xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) & XSC_LOGIC_WDT_WD_RESET_ACTIVE)
			val = WDIOF_CARDRESET;
		return put_user(val,p);
	case WDIOC_SETOPTIONS:
		{
		if ((rc = get_user(val,p)))
			return rc;
		if (val & WDIOS_ENABLECARD) {
			if (test_and_set_bit(0,&xsc_logic_wdt->watchdog_active))
				return -EBUSY;
			// Only toggle enable if not active
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== 0)
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
			return 0;
		}
		if (val & WDIOS_DISABLECARD) {
			// Only toggle enable if not active
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== (XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
			clear_bit(0,&xsc_logic_wdt->watchdog_active);
			return 0;
		}
		}
		return -EINVAL;
	case WDIOC_KEEPALIVE:
		if (!xsc_logic_wdt->watchdog_active)
			return -EINVAL;
		// write to REG_TICKLE
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_TICKLE, 0);
		return 0;
	case WDIOC_SETTIMEOUT:
		{
		int was_active = 0;
		if (xsc_logic_wdt->watchdog_active) {
			// Disable if active
			was_active = 1;
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== (XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
			clear_bit(0,&xsc_logic_wdt->watchdog_active);
		}
		if ((rc = get_user(val,p)))
			return -EFAULT;
		xsc_logic_wdt->total_timeout = xsc_logic_wdt_calc_counts(xsc_logic_wdt, val);
		val = (int) (xsc_logic_wdt->total_timeout >> xsc_logic_wdt->counter_divider);
		// set both registers to be the same
		dev_info(xsc_logic_wdt->dev, "setting timeout to %d, "
			 "counter_divider=%d, hz=%d\n", val,
			 xsc_logic_wdt->counter_divider,
			 xsc_logic_wdt_get_hz(xsc_logic_wdt));
		xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_COMPINT,val);
		xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_COMPRST,val);
		if (was_active) {
			// Only toggle enable if not active
			if (test_and_set_bit(0,&xsc_logic_wdt->watchdog_active))
				dev_err(xsc_logic_wdt->dev, "should never get here!\n");
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== 0)
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
		}
		return 0;
		}
	case WDIOC_GETTIMEOUT:
		val = xsc_logic_wdt_calc_seconds(xsc_logic_wdt, xsc_logic_wdt->total_timeout);
		return put_user(val,p);
	case WDIOC_SETPRETIMEOUT:
		{
		uint32_t reset_timeout;
		uint32_t pretimeout;
		int was_active = 0;
		if (xsc_logic_wdt->watchdog_active) {
			// Disable if active
			was_active = 1;
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== (XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
			clear_bit(0,&xsc_logic_wdt->watchdog_active);
		}
		if ((rc = get_user(val,p)))
			return -EFAULT;
		pretimeout = (uint32_t)
			(xsc_logic_wdt_calc_counts(xsc_logic_wdt,val) >> xsc_logic_wdt->counter_divider);
		reset_timeout = (uint32_t) (xsc_logic_wdt->total_timeout >> xsc_logic_wdt->counter_divider);
		if (pretimeout > reset_timeout)
			return -EINVAL;
		xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_COMPRST,reset_timeout);
		xsc_logic_wdt_set_reg(xsc_logic_wdt,REG_COMPINT,reset_timeout-pretimeout);
		if (was_active) {
			// Only toggle enable if not active
			if (test_and_set_bit(0,&xsc_logic_wdt->watchdog_active))
				dev_err(xsc_logic_wdt->dev, "should never get here!\n");
			if ((xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) &
				(XSC_LOGIC_WDT_ENABLE2_ACTIVE | XSC_LOGIC_WDT_ENABLE1_ACTIVE))
				== 0)
				xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
		}
		}
		return 0;
	case WDIOC_GETPRETIMEOUT:
		{
		uint32_t reset_timeout;
		uint32_t interrupt_timeout;
		uint64_t pretimeout_counts;

		reset_timeout = xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COMPRST);
		interrupt_timeout = xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COMPINT);
		pretimeout_counts = (uint64_t)(reset_timeout-interrupt_timeout)
			<< xsc_logic_wdt->counter_divider;
		val = xsc_logic_wdt_calc_seconds(xsc_logic_wdt, pretimeout_counts);
		}
		return put_user(val,p);
	case WDIOC_GETTIMELEFT:
		{
		uint64_t counter;
		uint64_t timeleft_counts;
		int timeleft_seconds;
		val = xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COUNTER);
		counter = ((uint64_t)val) << xsc_logic_wdt->counter_divider;
		timeleft_counts = xsc_logic_wdt->total_timeout - counter;
		timeleft_seconds = xsc_logic_wdt_calc_seconds(xsc_logic_wdt,timeleft_counts);
		return put_user(timeleft_seconds,p);
		}
	default:
		return -EINVAL;

	}

	return -EINVAL;
}


static int xsc_logic_wdt_open(struct inode *ino, struct file *f)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt;
	int rc;

	xsc_logic_wdt = container_of(ino->i_cdev, struct xsc_logic_wdt_dev, cdev);
	dev_dbg(xsc_logic_wdt->dev, "open\n");

	if (test_and_set_bit(0, &xsc_logic_wdt->driver_open)) {
		return -EBUSY;
	}
	f->private_data = xsc_logic_wdt;
	/* 9. Request IRQ */

	rc = request_irq(xsc_logic_wdt->irq, xsc_logic_wdt_interrupt, 0, DRIVER_NAME, xsc_logic_wdt);
	if (rc < 0) {
		clear_bit(0,&xsc_logic_wdt->driver_open);
		dev_err(xsc_logic_wdt->dev, "could not request IRQ %d\n",
			xsc_logic_wdt->irq);
		return -EBUSY;
	}
	xsc_logic_wdt->magic_close = 0;
	if (test_and_set_bit(0,&xsc_logic_wdt->watchdog_active)) {
		dev_info(xsc_logic_wdt->dev, "watchdog already active at open\n");
	} else {
		xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
	}

	return 0;
}

static int xsc_logic_wdt_release(struct inode *ino, struct file *f)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt;

	xsc_logic_wdt = container_of(ino->i_cdev, struct xsc_logic_wdt_dev, cdev);
	dev_dbg(xsc_logic_wdt->dev, "close\n");

	if (xsc_logic_wdt->watchdog_active && xsc_logic_wdt->magic_close) {
		xsc_logic_wdt_enable_toggle(xsc_logic_wdt);
		clear_bit(0,&xsc_logic_wdt->watchdog_active);
	}
	xsc_logic_wdt->magic_close = 0;

	free_irq(xsc_logic_wdt->irq,xsc_logic_wdt);

	clear_bit(0,&xsc_logic_wdt->driver_open);

	return 0;
}

static const struct file_operations xsc_logic_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= xsc_logic_wdt_write,
	.unlocked_ioctl	= xsc_logic_wdt_ioctl,
	.open		= xsc_logic_wdt_open,
	.release	= xsc_logic_wdt_release,
};

static ssize_t xsc_logic_wdt_nowayout_store(struct device *dev, struct device_attribute *attr,
				const char * buf, size_t count)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	uint32_t new_value;
	if (1 == sscanf(buf, "%u",&new_value)) {
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_NO_WAY_OUT,new_value);
		return count;
	} else
		return -EINVAL;
}

static ssize_t xsc_logic_wdt_nowayout_show(struct device *dev,
						struct device_attribute *attr,
						char * buf)
{
	int val;
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);

	if (xsc_logic_wdt_get_reg(xsc_logic_wdt, REG_STATUS) & XSC_LOGIC_WDT_NOWAYOUT_ACTIVE)
		val = 1;

	return sprintf(buf, "%d\n", val);
}

static ssize_t xsc_logic_wdt_timeout_store(struct device *dev,
						struct device_attribute *attr,
						const char * buf, size_t count)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	uint32_t new_value;
	if (1 == sscanf(buf, "%u",&new_value)) {
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPRST,new_value);
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPINT,new_value);
		return count;
	} else
		return -EINVAL;
}

static ssize_t xsc_logic_wdt_timeout_show(struct device *dev,
						struct device_attribute *attr,
						char * buf)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n",xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COMPRST));
}

static ssize_t xsc_logic_wdt_pretimeout_store(struct device *dev,
						struct device_attribute *attr,
						const char * buf, size_t count)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	uint32_t new_value;
	if (1 == sscanf(buf, "%u",&new_value)) {
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPRST,new_value);
		xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPINT,new_value);
		return count;
	} else
		return -EINVAL;
}

static ssize_t xsc_logic_wdt_pretimeout_show(struct device *dev,
						struct device_attribute *attr,
						char * buf)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n",xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COMPINT));
}

static ssize_t xsc_logic_wdt_counter_show(struct device *dev,
						struct device_attribute *attr,
						char * buf)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_COUNTER));
}


static ssize_t xsc_logic_wdt_status_show(struct device *dev,
						struct device_attribute *attr,
						char * buf)
{
	struct xsc_logic_wdt_dev *xsc_logic_wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",xsc_logic_wdt_get_reg(xsc_logic_wdt,REG_STATUS));
}

static DEVICE_ATTR(nowayout,S_IRUSR|S_IWUSR,xsc_logic_wdt_nowayout_show, xsc_logic_wdt_nowayout_store);
static DEVICE_ATTR(timeout,S_IRUSR|S_IWUSR,xsc_logic_wdt_timeout_show, xsc_logic_wdt_timeout_store);
static DEVICE_ATTR(pretimeout,S_IRUSR|S_IWUSR,xsc_logic_wdt_pretimeout_show, xsc_logic_wdt_pretimeout_store);
static DEVICE_ATTR(counter,S_IRUSR,xsc_logic_wdt_counter_show, NULL);
static DEVICE_ATTR(status,S_IRUSR,xsc_logic_wdt_status_show, NULL);

static struct attribute * xsc_logic_wdt_dev_attrs[] = {
	&dev_attr_nowayout.attr,
	&dev_attr_timeout.attr,
	&dev_attr_pretimeout.attr,
	&dev_attr_counter.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group xsc_logic_wdt_dev_attr_group = {
	.attrs = xsc_logic_wdt_dev_attrs
};

static int xsc_logic_wdt_probe_or_remove(bool probe, struct platform_device *ofdev)
{
	u32 val;
        int rc = 0;
        struct resource r_irq_struct;
        struct resource r_mem_struct;
        struct resource *r_irq = &r_irq_struct;
        struct resource *r_mem = &r_mem_struct;
	struct xsc_logic_wdt_dev *xsc_logic_wdt;
	const int INIT_TIMEOUT_SECONDS = 60;
	const __be32 *of_prop_val;

	if (!probe) {
		xsc_logic_wdt = dev_get_drvdata(&ofdev->dev);
		goto remove;
	}

	/* 1. Allocate device structure */
	xsc_logic_wdt = kzalloc(sizeof(*xsc_logic_wdt), GFP_KERNEL);
	if (!xsc_logic_wdt) {
		dev_err(&ofdev->dev, "cannot kzmalloc device structure");
		rc = -ENOMEM;
		goto fail_xsc_logic_wdt_alloc;
	}
	// Set info and options
	xsc_logic_wdt->wdt_info.options = WDIOF_SETTIMEOUT |
				WDIOF_PRETIMEOUT |
				WDIOF_CARDRESET;
	xsc_logic_wdt->wdt_info.firmware_version = 0;
	strncpy(xsc_logic_wdt->wdt_info.identity, DRIVER_DESCRIPTION,
		sizeof(xsc_logic_wdt->wdt_info.identity));

	/* 2. Obtain memory region from device tree */
	rc = of_address_to_resource(ofdev->dev.of_node, 0, r_mem);
	if (rc) {
		dev_err(&ofdev->dev, "cannot get memory resource from device-tree");
		rc = -EINVAL;
		goto fail_of_memory_resource;
	}
	xsc_logic_wdt->base_address = r_mem->start;

	dev_info(&ofdev->dev, "OF address: 0x%pa\n",
			&xsc_logic_wdt->base_address);
	/* 3. Obtain IRQ from device tree */
	rc = of_irq_to_resource(ofdev->dev.of_node, 0, r_irq);
        if (rc <= 0) {
		if (rc != -EPROBE_DEFER)
			dev_err(&ofdev->dev, "no IRQ found.\n");
                goto fail_rirq;
        }
        xsc_logic_wdt->irq = r_irq->start;
        dev_info(&ofdev->dev, "OF IRQ: %d\n", xsc_logic_wdt->irq);

	/* 4. Request memory region */
	if (!request_mem_region(r_mem->start, r_mem->end - r_mem->start + 1,
			DRIVER_NAME)) {
		rc = -EBUSY;
		dev_err(&ofdev->dev, "cannot reserve memory region");
		goto fail_request_mem;
	}
        /* 5. Remap memory region */
        xsc_logic_wdt->base = ioremap(xsc_logic_wdt->base_address, 1024);
        dev_info(&ofdev->dev, "remapped to %p\n", xsc_logic_wdt->base);
        if (!xsc_logic_wdt->base) {
                dev_err(&ofdev->dev, "ioremap failed\n");
                rc = -ENOMEM;
                goto fail_ioremap;
        }

        /* 6. Set openfirmware driver data */
        dev_set_drvdata(&ofdev->dev, xsc_logic_wdt);

	// grab some information about the logic from the DTS
	of_prop_val = of_get_property(ofdev->dev.of_node, "clk-freq", NULL);
	if (of_prop_val)
		xsc_logic_wdt->clk_freq = be32_to_cpup(of_prop_val);
	of_prop_val = of_get_property(ofdev->dev.of_node, "counter-divider", NULL);
	if (of_prop_val)
		xsc_logic_wdt->counter_divider = be32_to_cpup(of_prop_val);

	/* 6.5 Initialize the logic to a sane timeout value */
	xsc_logic_wdt->total_timeout = xsc_logic_wdt_calc_counts(xsc_logic_wdt,
			INIT_TIMEOUT_SECONDS);
	val = (uint32_t) (xsc_logic_wdt->total_timeout >>
			xsc_logic_wdt->counter_divider);
	// set both registers to be the same
	dev_info(&ofdev->dev, "setting timeout to %d, "
		 "counter_divider=%d, hz=%d\n", val,
		 xsc_logic_wdt->counter_divider,
		 xsc_logic_wdt_get_hz(xsc_logic_wdt));

	xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPINT, val);
	xsc_logic_wdt_set_reg(xsc_logic_wdt, REG_COMPRST, val);

	/* 7. Initialize a character device */
	xsc_logic_wdt->dev_id = MKDEV(MAJOR(xsc_logic_wdt_dev_id),MINOR(xsc_logic_wdt_dev_id));
	cdev_init(&xsc_logic_wdt->cdev, &xsc_logic_wdt_fops);
	xsc_logic_wdt->cdev.owner = THIS_MODULE;
	rc = cdev_add(&xsc_logic_wdt->cdev, xsc_logic_wdt->dev_id, 1);
	if (rc) {
		dev_err(&ofdev->dev, "cannot add character device\n");
		goto fail_cdev;
	}

	/* 8. Create the device */
	xsc_logic_wdt->dev = device_create(xsc_logic_wdt_class, &ofdev->dev, xsc_logic_wdt->dev_id,
		xsc_logic_wdt, DRIVER_NAME "_%d", 0);
	if (IS_ERR(xsc_logic_wdt->dev)) {
		rc = PTR_ERR(xsc_logic_wdt->dev);
		goto fail_device;
	}


	/* the following may be interesting sysfs things to add:
	 *  - configure the type of signal to be send at the first timeout
	 *  - timeouts should be configured by the application...
	 */
	rc = sysfs_create_group(&xsc_logic_wdt->dev->kobj, &xsc_logic_wdt_dev_attr_group);
	if (rc) {
		dev_err(&ofdev->dev,"cannot create sysfs group\n");
		goto fail_sysfs;
	}

	return 0;

remove:
	/* If the device was never allocated, don't bother trying to remove */
	if (!xsc_logic_wdt) {
		return 0;
	}

	sysfs_remove_group(&xsc_logic_wdt->dev->kobj, &xsc_logic_wdt_dev_attr_group);

fail_sysfs:
	/* 8. Create the device */
	device_destroy(xsc_logic_wdt_class, xsc_logic_wdt->dev_id);

fail_device:
	/* 7. Initialize a character device */
	cdev_del(&xsc_logic_wdt->cdev);

fail_cdev:
        /* 6. Set openfirmware driver data */
        dev_set_drvdata(&ofdev->dev, NULL);

        /* 5. Remap memory region */
        iounmap(xsc_logic_wdt->base);

fail_ioremap:
        /* 4.  Request memory region */
        /* NB: Re-gathering the address in case this is called from of_remove*/
        rc = of_address_to_resource(ofdev->dev.of_node, 0, r_mem);
        if(rc) {
                dev_err(&ofdev->dev, "cannot get memory resource\n");
                rc = -EINVAL;
                goto fail_request_mem;
        }
        release_mem_region(r_mem->start, r_mem->end - r_mem->start + 1);

fail_request_mem:
	/* 3. Obtain IRQ from device tree */
fail_rirq:
	/* 2. obtain memory region from device tree */
fail_of_memory_resource:
	/* 1. Allocate device structure*/
	kfree(xsc_logic_wdt);

fail_xsc_logic_wdt_alloc:
	return rc;
}

static int of_probe(struct platform_device *ofdev)
{
	return(xsc_logic_wdt_probe_or_remove(true, ofdev));
}

static int of_remove(struct platform_device *ofdev)
{
	return(xsc_logic_wdt_probe_or_remove(false, ofdev));
}

/*
 * Device tree sample on the Q6 looks like:
 *  xsc_xsc_watchdog_plb_v1_0: xsc-q6s-watchdog-plb-v1@f0000000 {
 *    compatible = "xlnx,xsc-q6s-watchdog-plb-v1-1.00.a";
 *    interrupt-parent = <&xps_intc_0>;
 *    interrupts = < 22 2 >;
 *    reg = < 0xf0000000 0x1000000 >;
 *    xlnx,clk-freq = <0x47868c0>;
 *    xlnx,counter-divider = <0x2>;
 *    xlnx,family = "spartan6";
 *  } ;
 */
static struct of_device_id of_match[] = {
	{ .compatible = "xlnx,xsc-q6s-watchdog-plb-v1-1.00.a" },
	{ .compatible = "xlnx,xsc-q6s-watchdog-axi-v1-1.00.a" },
	{ .compatible = "xlnx,xsc-q7s-watchdog-plb-v1-1.00.a" },
	{ .compatible = "xlnx,xsc-q7s-watchdog-axi-v1-1.00.a" },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver of_driver = {
	.probe 		= of_probe,
	.remove		= of_remove,
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match,
	}
};

static int __init xsc_logic_wdt_init(void) {

	int result;

	pr_debug("initializing\n");

	/* Create class for this device */
	xsc_logic_wdt_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(xsc_logic_wdt_class)) {
		result = PTR_ERR(xsc_logic_wdt_class);
		goto fail0;
	}

	/* Register a contiguous set of minor numbers for our character
	 * devices */
	result = alloc_chrdev_region(&xsc_logic_wdt_dev_id, 0,
			1, DRIVER_NAME);
	if (result < 0) {
		pr_err("cannot allocate chrdev region\n");
		goto fail1;
	}

	result = platform_driver_register(&of_driver);
	if (result < 0) {
		pr_err("of_register_platform_driver returned %d\n", result);
		goto fail2;
	}

	return 0;

fail2:
	unregister_chrdev_region(xsc_logic_wdt_dev_id, 1);

fail1:
	class_destroy(xsc_logic_wdt_class);

fail0:
	return result;


}

static void __exit xsc_logic_wdt_exit(void) {

	platform_driver_unregister(&of_driver);
	unregister_chrdev_region(xsc_logic_wdt_dev_id, 1);
	class_destroy(xsc_logic_wdt_class);

}

module_init(xsc_logic_wdt_init);
module_exit(xsc_logic_wdt_exit);

MODULE_AUTHOR("Xiphos Systems Corporation <jpl@xiphos.ca>");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);

/* vim:set ts=8 noexpandtab sw=8 foldmarker={,}: */
