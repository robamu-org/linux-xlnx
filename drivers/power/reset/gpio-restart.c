/*
 * Toggles a GPIO pin to restart a device
 *
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Based on the gpio-poweroff driver.
 */
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/io.h>

/* Hardcoded addresses and values for restart
 *  * see gpio_poweroff_do_poweroff() */
#define XIPHOS_Q7_PA3_CRP_ADDR 0x40000000
#define XIPHOS_Q7_PA3_CRP_RESTART_VALUE 0x100

struct gpio_restart {
	struct gpio_desc *reset_gpio;
	struct notifier_block restart_handler;
	u32 active_delay_ms;
	u32 inactive_delay_ms;
	u32 wait_delay_ms;
};

static int gpio_restart_notify(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	volatile u32* register_addr = NULL;

	struct gpio_restart *gpio_restart =
		container_of(this, struct gpio_restart, restart_handler);

    /* EVIL HACK wfb 2017/02/02
	 * Q7 ProASIC3 logic / device-tree use Xilinx 'dual channel' chips
	 * Kernel supports is terrible about those; gpio function is not
	 *  able to get the correct gpio pin from device-tree.
	 * Worst, forcing the GPIO fails whenever the kernel try to acess
	 *  a gpio from the second channel (bank).
	 * The bug is probably somewhere in gpio-xilinx, dual bank was
	 *  probably never tested.
	 *
	 * We will revert buggy dual channel chip to classic single channel
	 *  in next revision of ProASIC3...
	 *
	 * In the meantime, hardcode the ProASIC3 memory address and
	 *  access that directly.
	 */

    /* Volatile so it is not optimized out */
    register_addr = (volatile u32*)ioremap(XIPHOS_Q7_PA3_CRP_ADDR, 4);
    (*register_addr) = (volatile u32)XIPHOS_Q7_PA3_CRP_RESTART_VALUE;
    /* If we survived restart failed, give the default code a try */



	/* drive it active, also inactive->active edge */
	gpiod_direction_output(gpio_restart->reset_gpio, 1);
	mdelay(gpio_restart->active_delay_ms);

	/* drive inactive, also active->inactive edge */
	gpiod_set_value(gpio_restart->reset_gpio, 0);
	mdelay(gpio_restart->inactive_delay_ms);

	/* drive it active, also inactive->active edge */
	gpiod_set_value(gpio_restart->reset_gpio, 1);

	/* give it some time */
	mdelay(gpio_restart->wait_delay_ms);

	WARN_ON(1);

	return NOTIFY_DONE;
}

static int gpio_restart_probe(struct platform_device *pdev)
{
	struct gpio_restart *gpio_restart;
	bool open_source = false;
	u32 property;
	int ret;

	gpio_restart = devm_kzalloc(&pdev->dev, sizeof(*gpio_restart),
			GFP_KERNEL);
	if (!gpio_restart)
		return -ENOMEM;

	open_source = of_property_read_bool(pdev->dev.of_node, "open-source");

	gpio_restart->reset_gpio = devm_gpiod_get(&pdev->dev, NULL,
			open_source ? GPIOD_IN : GPIOD_OUT_LOW);
	if (IS_ERR(gpio_restart->reset_gpio)) {
		dev_err(&pdev->dev, "Could net get reset GPIO\n");
		return PTR_ERR(gpio_restart->reset_gpio);
	}

	gpio_restart->restart_handler.notifier_call = gpio_restart_notify;
	gpio_restart->restart_handler.priority = 129;
	gpio_restart->active_delay_ms = 100;
	gpio_restart->inactive_delay_ms = 100;
	gpio_restart->wait_delay_ms = 3000;

	ret = of_property_read_u32(pdev->dev.of_node, "priority", &property);
	if (!ret) {
		if (property > 255)
			dev_err(&pdev->dev, "Invalid priority property: %u\n",
					property);
		else
			gpio_restart->restart_handler.priority = property;
	}

	of_property_read_u32(pdev->dev.of_node, "active-delay",
			&gpio_restart->active_delay_ms);
	of_property_read_u32(pdev->dev.of_node, "inactive-delay",
			&gpio_restart->inactive_delay_ms);
	of_property_read_u32(pdev->dev.of_node, "wait-delay",
			&gpio_restart->wait_delay_ms);

	platform_set_drvdata(pdev, gpio_restart);

	ret = register_restart_handler(&gpio_restart->restart_handler);
	if (ret) {
		dev_err(&pdev->dev, "%s: cannot register restart handler, %d\n",
				__func__, ret);
		return -ENODEV;
	}

	return 0;
}

static int gpio_restart_remove(struct platform_device *pdev)
{
	struct gpio_restart *gpio_restart = platform_get_drvdata(pdev);
	int ret;

	ret = unregister_restart_handler(&gpio_restart->restart_handler);
	if (ret) {
		dev_err(&pdev->dev,
				"%s: cannot unregister restart handler, %d\n",
				__func__, ret);
		return -ENODEV;
	}

	return 0;
}

static const struct of_device_id of_gpio_restart_match[] = {
	{ .compatible = "gpio-restart", },
	{},
};

static struct platform_driver gpio_restart_driver = {
	.probe = gpio_restart_probe,
	.remove = gpio_restart_remove,
	.driver = {
		.name = "restart-gpio",
		.of_match_table = of_gpio_restart_match,
	},
};

module_platform_driver(gpio_restart_driver);

MODULE_AUTHOR("David Riley <davidriley@chromium.org>");
MODULE_DESCRIPTION("GPIO restart driver");
MODULE_LICENSE("GPL");
