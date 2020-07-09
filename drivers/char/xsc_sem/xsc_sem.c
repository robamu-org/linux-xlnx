#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/gpio/consumer.h>
#include <asm/io.h>

#include <linux/firmware/xilinx/zynqmp/firmware.h>

#undef pr_fmt
#define pr_fmt(fmt) DRIVER_NAME ": %s:%d " fmt, __func__, __LINE__

#define CLASS_NAME	"xsc_sem"
#define DRIVER_NAME	"xsc_sem"
#define XSC_SEM_NAME	"xsc_sem_ip"

#define DEBUG

#define XSC_SEM_FIFO_RX		0x00
#define XSC_SEM_FIFO_TX		0x04
#define XSC_SEM_STATUS		0x08
#define XSC_SEM_CTRL		0x0C

#define XSC_SEM_TX_FULL		BIT(3)
#define XSC_SEM_RX_DATA		BIT(0)

#define XSC_SEM_CTRL_INTR	BIT(4)

struct xsc_sem {
	struct device		*dev;
	int			irq;		/* interrupt */
	struct resource		*mem;		/* physical memory */
	void __iomem		*base_addr;	/* kernel space memory */

	struct device		*io_dev;
	struct cdev		io_cdev;
	struct mutex		io_lock;

	bool			has_written;

	struct tasklet_struct	event_tasklet;
	spinlock_t              spinlock;
	wait_queue_head_t       poll_wait;      /* Poll event waiter */

	uint32_t		interrupt_state;
};

enum pcap_method {
	PCAP_METHOD_ICAP,
	PCAP_METHOD_PCAP,
};

static int set_pcap(struct device *dev, enum pcap_method method);

static struct class *xsc_sem_class;
static dev_t io_cdev_id;

static bool tx_full(struct xsc_sem *xsc_sem_dev)
{
	return !!(ioread32(xsc_sem_dev->base_addr + XSC_SEM_STATUS) & XSC_SEM_TX_FULL);
}

static bool rx_empty(struct xsc_sem *xsc_sem_dev)
{
	return !(ioread32(xsc_sem_dev->base_addr + XSC_SEM_STATUS) & XSC_SEM_RX_DATA);
}

/* Only accepted values are 'pcap' or 'icap' */
static ssize_t pcap_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct xsc_sem *xsc_sem = dev_get_drvdata(dev);
	int ret = 0;
	if (size < 4)
		return -EINVAL;

	if (strncmp(buf, "pcap", 4) == 0) {
		ret = set_pcap(dev, PCAP_METHOD_PCAP);
	}
	else if (strncmp(buf, "icap", 4) == 0) {
		// Restore interrupt state then, set ICAP
		iowrite32(xsc_sem->interrupt_state, xsc_sem->base_addr + XSC_SEM_CTRL);
		ret = set_pcap(dev, PCAP_METHOD_ICAP);
	}
	else
		return -EINVAL;

	return ret == 0 ? size : -EACCES;
}

static DEVICE_ATTR_WO(pcap);

static struct attribute *xsc_sem_ctrl_attr[] = {
	&dev_attr_pcap.attr,
	NULL,
};

static struct attribute_group xsc_sem_ctrl_attrs_group = {
	.attrs = xsc_sem_ctrl_attr,
};

static int io_dev_open(struct inode *ino, struct file *f)
{
	struct xsc_sem *dev;
	int rc = 0;

	/* retrieve the cdev associated to that cdev */
	dev = container_of(ino->i_cdev, struct xsc_sem, io_cdev);
	if (!dev)
		return -EINVAL;

	/* attach this dev to that file to follow the file operation */
	f->private_data = dev;

	/* at this step we have open the cdev and must lock it */
	rc = mutex_trylock(&dev->io_lock);

	if (rc != 1)
		return -EBUSY;

	// Activate interrupts
	iowrite32(XSC_SEM_CTRL_INTR, dev->base_addr + XSC_SEM_CTRL);
	dev->interrupt_state = XSC_SEM_CTRL_INTR;

	return 0;
}

static int io_dev_release(struct inode *ino, struct file *f)
{
	struct xsc_sem *dev;

	dev = f->private_data;
	if (!dev)
		return -EINVAL;

	/* unlock the cdev */
	mutex_unlock(&dev->io_lock);

	// Deactivate interrupts
	iowrite32(0, dev->base_addr + XSC_SEM_CTRL);
	dev->interrupt_state = 0;

	return 0;
}

static ssize_t io_dev_read(struct file *f, char __user *dst, size_t count,
			  loff_t *off)
{
	char current_str[count];
	size_t left = count;
	struct xsc_sem *dev = f->private_data;
	uint32_t val;

	if (!dev)
		return -EINVAL;

	while (left > 0) {
		if (rx_empty(dev))
			break;
		val = ioread32(dev->base_addr + XSC_SEM_FIFO_RX);
		current_str[count - left] = val;
		left--;
	}

	copy_to_user(dst, current_str, count - left);

	return count - left;
}

static ssize_t io_dev_write(struct file *f, const char __user *src,
				   size_t count, loff_t *off)
{
	char current_str[count];
	size_t left = count;
	struct xsc_sem *dev = f->private_data;

	if (!dev)
		return -EINVAL;

	copy_from_user(current_str, src, count);

	while (left > 0) {
		if (tx_full(dev))
			break;
		iowrite32(current_str[count - left],
			  dev->base_addr + XSC_SEM_FIFO_TX);
		left--;
	}

	dev->has_written = count > 0;

	return count - left;
}

static unsigned int io_dev_poll(struct file *file, poll_table *wait)
{
	struct xsc_sem *dev;
	int mask = 0;

	dev = file->private_data;
	if (!dev)
		return -EINVAL;

	poll_wait(file, &dev->poll_wait, wait);

	if (!rx_empty(dev) > 0) {
		mask |= POLLIN;
	}
	if (!tx_full(dev) && dev->has_written) {
		dev->has_written = false;
		mask |= POLLOUT;
	}

	return mask;
}

static struct file_operations io_fops = {
	.owner          = THIS_MODULE,
	.open           = io_dev_open,
	.release        = io_dev_release,
	.read           = io_dev_read,
	.write          = io_dev_write,
	.poll		= io_dev_poll,
};

static void event_dispatch(unsigned long device)
{
	struct xsc_sem	*dev = (struct xsc_sem *)device;
	unsigned long	flags = 0;

	spin_lock_irqsave(&dev->spinlock, flags);
	wake_up_interruptible(&dev->poll_wait);
	spin_unlock_irqrestore(&dev->spinlock, flags);
}

static irqreturn_t xsc_sem_dev_irq(int irq, void *device)
{
	struct xsc_sem *xsc_sem_dev = device;

	tasklet_schedule(&xsc_sem_dev->event_tasklet);

	return IRQ_HANDLED;
}

#ifdef CONFIG_ARCH_ZYNQMP
/*
 * CSU register access throught ATF. See
 * https://www.xilinx.com/support/answers/71089.html
 * for information about accessing CSU registers with ATF v2018.2.
 * This should be remove if Vivado > v2018.2 is used.
 *
 * Note that ATF must be patched for this driver to work properly.
 *
 */

#define IOCTL_WRITE_REG		0xFFFFFFFE
#define IOCTL_READ_REG		0xFFFFFFFF
#define XSC_CSU_PCAP_CTRL	0xFFCA3008

static uint32_t zynqmp_read_register(uint32_t addr, uint32_t *val)
{
	int ret;
	uint32_t ret_payload[PAYLOAD_ARG_CNT];
	const struct zynqmp_eemi_ops *eemi_ops = zynqmp_pm_get_eemi_ops();

	if (!eemi_ops || !eemi_ops->ioctl)
		return -EFAULT;

	ret = eemi_ops->ioctl(0, IOCTL_READ_REG, addr, 0, ret_payload);
	*val = ret_payload[1];

	return ret;
}

static uint32_t zynqmp_write_register(uint32_t addr, uint32_t val)
{
	int ret;
	const struct zynqmp_eemi_ops *eemi_ops = zynqmp_pm_get_eemi_ops();

	if (!eemi_ops || !eemi_ops->ioctl)
		return -EFAULT;

	ret = eemi_ops->ioctl(0, IOCTL_WRITE_REG, addr, val, NULL);

	return ret;
}

/*
 * set_pcap():
 * Manages PCAP method:
 * If method is PCAP_METHOD_ICAP, the PL will be accessible by the SEMIP
 * If method is PCAP_METHOD_PCAP, the PL will be accessible by the user
 */
static int set_pcap(struct device *dev, enum pcap_method method)
{
	uint32_t val;
	int rc = zynqmp_read_register(0xffca3008, &val);

	if (rc != 0)
		return rc;

	if (method == PCAP_METHOD_ICAP)
		return zynqmp_write_register(0xffca3008, val & ~1);
	else
		return zynqmp_write_register(0xffca3008, val | 1);
}

#elif defined(CONFIG_ARCH_ZYNQ)

/*
 * This reset is used to hold the TMR SEM IP, it won't reset the device.
 * Set value to 1 to unhold it and be able to access the AXI registers.
 */
static int set_reset(struct device *dev, uint32_t value)
{
	struct gpio_desc *desc = gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(desc)) {
		pr_err("Cannot get GPIO for reset.\n");
		return PTR_ERR(desc);
	}

	gpiod_set_value(desc, value);

	gpiod_put(desc);

	return 0;
}

/*
 * set_pcap():
 * Manages PCAP method:
 * If method is PCAP_METHOD_ICAP, the PL will be accessible by the SEMIP
 * If method is PCAP_METHOD_PCAP, the PL will be accessible by the user
 */
static int set_pcap(struct device *dev, enum pcap_method method)
{
	uint32_t value;
	void __iomem *base_addr;
	int ret;

	base_addr = ioremap(0xF8007000, 4096);

	if (!base_addr) {
		pr_err("ioremap failed\n");
		return -ENOMEM;
	}

	/*
	 * Clear bit 27 of the DEVCFG CTRL register (See
	 * https://www.xilinx.com/support/answers/66975.html)
	 */
	value = ioread32(base_addr);

	if (method == PCAP_METHOD_ICAP)
		iowrite32(value & ~(1 << 27), base_addr);
	else
		iowrite32(value | (1 << 27), base_addr);

	value = ioread32(base_addr);

	iounmap(base_addr);

	ret = set_reset(dev, 1);
	if (ret != 0)
		return ret;

	return 0;
}

#endif

static int xsc_sem_probe_or_remove(bool probe, struct platform_device *pdev)
{
	struct xsc_sem *xsc_sem_dev;
	struct device *dev = &pdev->dev;
	struct resource *r_mem; /* IO mem resources */
	struct resource r_irq_struct;
	struct resource *r_irq = &r_irq_struct;
	int rc = 0;

	if (!probe) {
		xsc_sem_dev = dev_get_drvdata(dev);
		rc = 0;
		goto remove;
	}

	pr_info("Probing\n");

	rc = set_pcap(&pdev->dev, PCAP_METHOD_ICAP);
	if (rc) {
		pr_info("Unable to initialize ICAP");
		return rc;
	}

	/* Allocate device structure */
	xsc_sem_dev = kzalloc(sizeof(*xsc_sem_dev), GFP_KERNEL);
	if (xsc_sem_dev == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, xsc_sem_dev);
	xsc_sem_dev->dev = dev;

	/* Obtain memory region from device tree */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "cannot get memory resource\n");
		rc = -EINVAL;
		goto fail_rmem;
	}
	xsc_sem_dev->mem = r_mem;

	/* Request physical memory region */
	if (!request_mem_region(xsc_sem_dev->mem->start,
				resource_size(xsc_sem_dev->mem),
				DRIVER_NAME)) {
		dev_err(dev, "cannot reserve memory region\n");
		rc = -EBUSY;
		goto fail_request_mem;
	}

	/* Map physical memory to kernel virtual address space */
	xsc_sem_dev->base_addr = ioremap(xsc_sem_dev->mem->start,
					 resource_size(xsc_sem_dev->mem));

	if (!xsc_sem_dev->base_addr) {
		dev_err(dev, "ioremap failed\n");
		rc = -ENOMEM;
		goto fail_ioremap;
	}

	mutex_init(&xsc_sem_dev->io_lock);
	spin_lock_init(&xsc_sem_dev->spinlock);
	init_waitqueue_head(&xsc_sem_dev->poll_wait);

	tasklet_init(&xsc_sem_dev->event_tasklet, event_dispatch,
		     (unsigned long) xsc_sem_dev);

	/* Obtain IRQ from device tree */
	rc = of_irq_to_resource(dev->of_node, 0, r_irq);
	if (rc <= 0) {
		dev_err(dev, "cannot get interrupt\n");
		goto fail_irq;
	}
	xsc_sem_dev->irq = r_irq->start;

	/* Associate the interrupt service routine */
	rc = devm_request_irq(dev, xsc_sem_dev->irq, xsc_sem_dev_irq, 0,
			dev_name(dev), xsc_sem_dev);

	if (rc < 0) {
		dev_err(dev, "request_irq %d failed\n", xsc_sem_dev->irq);
		goto fail_irq;
	}

	/* initialize device for sysfs */
	rc = alloc_chrdev_region(&io_cdev_id, 0, 1,
				 XSC_SEM_NAME);
	if (rc < 0) {
		dev_err(dev, "couldn't allocate chrdev region\n");
		goto fail_io_chrdev;
	}

	/* initialize the char device and attach it the file_operations */
	cdev_init(&xsc_sem_dev->io_cdev, &io_fops);
	xsc_sem_dev->io_cdev.owner = THIS_MODULE;
	/* register the char device, its minor, and how many minor it handles */
	rc = cdev_add(&xsc_sem_dev->io_cdev, io_cdev_id, 1);
	if (rc) {
		dev_err(dev, "cdev_add failed %d\n", rc);
		goto fail_io_cdev;
	}

	xsc_sem_dev->io_dev = device_create(
		xsc_sem_class,
		NULL,
		io_cdev_id,
		xsc_sem_dev,
		"%s%d", XSC_SEM_NAME, 0
	);

	if (IS_ERR(xsc_sem_dev->io_dev)) {
		rc = PTR_ERR(xsc_sem_dev->io_dev);
		goto fail_device;
	}

	rc = sysfs_create_group(&xsc_sem_dev->io_dev->kobj,
	                        &xsc_sem_ctrl_attrs_group);
	if (rc < 0) {
		dev_err(dev, "couldn't register sysfs group \n");
		goto fail_device;
	}

	pr_info("Sucessfully loaded.\n");

	return 0;

remove:
	if (!xsc_sem_dev)
		return -EINVAL;

	sysfs_remove_group(&xsc_sem_dev->io_dev->kobj, &xsc_sem_ctrl_attrs_group);

fail_device:
	/* Remove char devices */
	device_destroy(xsc_sem_class,
		       xsc_sem_dev->io_dev->devt);

	cdev_del(&xsc_sem_dev->io_cdev);

fail_io_cdev:
	unregister_chrdev_region(MAJOR(io_cdev_id), 1);

fail_io_chrdev:
fail_irq:
	iounmap(xsc_sem_dev->base_addr);

fail_ioremap:
	release_mem_region(xsc_sem_dev->mem->start,
			   resource_size(xsc_sem_dev->mem));

fail_request_mem:
fail_rmem:
	kfree(xsc_sem_dev);

	return rc;
}

static int xsc_sem_of_probe(struct platform_device *pdev)
{
	return xsc_sem_probe_or_remove(true, pdev);
}

static int xsc_sem_of_remove(struct platform_device *pdev)
{
	return xsc_sem_probe_or_remove(false, pdev);
}

static const struct of_device_id of_match[] = {
	{ .compatible = "xlnx,tmr-sem-1.0" },
	{}
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver xsc_sem_of_driver = {
	.probe	= xsc_sem_of_probe,
	.remove	= xsc_sem_of_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match
	}
};

static int __init xsc_sem_init(void)
{
	int rc;

	/* Create class for this device */
	xsc_sem_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(xsc_sem_class)) {
		rc = PTR_ERR(xsc_sem_class);
		goto fail_xsc_sem_class;
	}

	rc = platform_driver_register(&xsc_sem_of_driver);
	if (rc < 0) {
		printk(KERN_INFO DRIVER_NAME
		       ": platform_driver_register returned %d\n",
		       rc);
		goto fail_platform_driver_register;
	}

	return 0;

fail_platform_driver_register:
	class_destroy(xsc_sem_class);

fail_xsc_sem_class:
	return rc;
}

static void __exit xsc_sem_exit(void)
{
	platform_driver_unregister(&xsc_sem_of_driver);
	class_destroy(xsc_sem_class);
}

module_init(xsc_sem_init);
module_exit(xsc_sem_exit);

MODULE_DESCRIPTION("Xiphos Soft Error Manager Interface");
MODULE_AUTHOR("Xiphos Systems Corporation");
MODULE_LICENSE("GPL v2");

/* vim:set ts=8 noexpandtab sw=8 foldmarker={,}: */
