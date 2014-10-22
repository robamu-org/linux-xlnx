/*
 * This file contains code for restarting Xiphos' Q7
 *
 * Copyright (C) 2014 Xiphos Systems Corp.
 *
 * Derived from gpio-poweroff.c
 *
 * Jamie Lentin <jm@lentin.co.uk>
 * Andrew Lunn <andrew@lunn.ch>
 *
 * Copyright (C) 2012 Jamie Lentin
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <asm/system_misc.h>


static int q7_restart_gpio_num = -1;
static void (*zynq_pm_restart)(enum reboot_mode reboot_mode, const char *cmd) = NULL;

static void q7_gpio_restart(enum reboot_mode mode, const char *cmd)
{

  if (mode != REBOOT_GPIO && zynq_pm_restart) {
    zynq_pm_restart(mode,cmd);
  }

	BUG_ON(!gpio_is_valid(q7_restart_gpio_num));

	/* drive it active, also inactive->active edge */
	gpio_direction_output(q7_restart_gpio_num, 1);

	/* give it some time */
	mdelay(3000);

	WARN_ON(1);
}

static int q7_restart_probe(struct platform_device *pdev)
{
	enum of_gpio_flags flags;
	bool input = false;
	int ret;

	q7_restart_gpio_num = of_get_gpio_flags(pdev->dev.of_node, 0, &flags);
	if (!gpio_is_valid(q7_restart_gpio_num))
		return q7_restart_gpio_num;

	input = of_property_read_bool(pdev->dev.of_node, "input");

	ret = gpio_request(q7_restart_gpio_num, "poweroff-gpio");
	if (ret) {
		pr_err("%s: Could not get GPIO %d\n", __func__, q7_restart_gpio_num);
		return ret;
	}
	pr_notice("%s: Using GPIO %d for Q7 restart\n",__func__,q7_restart_gpio_num);
	if (input) {
		if (gpio_direction_input(q7_restart_gpio_num)) {
			pr_err("Could not set direction of GPIO %d to input\n",
			       q7_restart_gpio_num);
			goto err;
		}
	} else {
		if (gpio_direction_output(q7_restart_gpio_num, 0)) {
			pr_err("Could not set direction of GPIO %d\n", q7_restart_gpio_num);
			goto err;
		}
	}

  //TODO: This should be locked
	zynq_pm_restart = arm_pm_restart;
  arm_pm_restart = q7_gpio_restart;
	return 0;

err:
	gpio_free(q7_restart_gpio_num);
	return -ENODEV;
}

static int q7_restart_remove(struct platform_device *pdev)
{
	gpio_free(q7_restart_gpio_num);
	if (arm_pm_restart == q7_gpio_restart)
		arm_pm_restart = zynq_pm_restart;

	return 0;
}


static const struct of_device_id of_q7_restart_match[] = {
	{ .compatible = "q7-restart-gpio", },
	{},
};

static struct platform_driver q7_restart_driver = {
	.probe = q7_restart_probe,
	.remove = q7_restart_remove,
	.driver = {
		.name = "q7-restart-gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_q7_restart_match,
	},
};

module_platform_driver(q7_restart_driver);

MODULE_AUTHOR("Joshua Lamorie <jpl@xiphos.ca>");
MODULE_DESCRIPTION("Q7 GPIO restart driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:q7-restart-gpio");
