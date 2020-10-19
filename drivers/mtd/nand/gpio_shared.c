/*
 * drivers/mtd/nand/gpio-shared.c
 *
 * This is the GPIO control NAND driver with the possibility to share
 * some of the GPIOs between devices.
 *
 * Updated, and converted to generic GPIO based driver by Russell King.
 *
 * Written by Detlev Casanova <dec@xiphos.com>
 *   Based on the gpio-nand driver by Ben Dooks
 *
 * © 2020 Xiphos Systems Corporation
 *
 * Device driver for NAND flash that uses a memory mapped interface to
 * read/write the NAND commands and data, and GPIO pins for control signals
 * (the DT binding refers to this as "Shared GPIO assisted NAND flash")
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio.h>
#include <linux/mtd/concat.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>



#define	MAX_MTD_COUNT	16

struct mtd_info* subdevs[MAX_MTD_COUNT];

struct gpiomtd {
	struct nand_chip		nand_chip;
	struct gpio_nand_platdata	plat;
	int				offset;
	int				size;
};

struct gpiomtd_shared {
	struct gpiomtd*			gpiomtds[MAX_MTD_COUNT];
	int				dev_count;
	struct gpio_nand_platdata	plat;
};

static inline struct gpiomtd *gpio_nand_getpriv(struct mtd_info *mtd)
{
	return container_of(mtd_to_nand(mtd), struct gpiomtd, nand_chip);
}

static void gpio_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);

	if (ctrl & NAND_CTRL_CHANGE) {
		if (gpio_is_valid(gpiomtd->plat.gpio_nce))
			gpio_set_value(gpiomtd->plat.gpio_nce,
				       !(ctrl & NAND_NCE));
		gpio_set_value(gpiomtd->plat.gpio_cle, !!(ctrl & NAND_CLE));
		gpio_set_value(gpiomtd->plat.gpio_ale, !!(ctrl & NAND_ALE));
	}

	if (cmd == NAND_CMD_NONE)
		return;

	writeb(cmd, gpiomtd->nand_chip.IO_ADDR_W);
}

static int gpio_nand_devready(struct mtd_info *mtd)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);

	return gpio_get_value(gpiomtd->plat.gpio_rdy);
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_nand_id_table[] = {
	{ .compatible = "gpio-control-nand-shared" },
	{}
};
MODULE_DEVICE_TABLE(of, gpio_nand_id_table);

static int gpio_nand_get_config_of(const struct device *dev,
				   struct gpio_nand_platdata *plat)
{
	u32 val;

	if (!dev->of_node)
		return -ENODEV;

	if (!of_property_read_u32(dev->of_node, "bank-width", &val)) {
		if (val == 2) {
			plat->options |= NAND_BUSWIDTH_16;
		} else if (val != 1) {
			dev_err(dev, "invalid bank-width %u\n", val);
			return -EINVAL;
		}
	}

	// Shared GPIOs
	plat->gpio_ale = of_get_gpio(dev->of_node, 0);
	plat->gpio_cle = of_get_gpio(dev->of_node, 1);
	plat->gpio_rdy = of_get_gpio(dev->of_node, 2);
	plat->gpio_nwp = of_get_gpio(dev->of_node, 3);

	if (!of_property_read_u32(dev->of_node, "chip-delay", &val))
		plat->chip_delay = val;

	if (of_property_read_u32(dev->of_node, "xsc,concat", &plat->concat))
		plat->concat = 0;

	return 0;
}

#else /* CONFIG_OF */
static inline int gpio_nand_get_config_of(const struct device *dev,
					  struct gpio_nand_platdata *plat)
{
	return -ENOSYS;
}
#endif /* CONFIG_OF */

static inline int gpio_nand_get_config(const struct device *dev,
				       struct gpio_nand_platdata *plat)
{
	int ret = gpio_nand_get_config_of(dev, plat);

	if (!ret)
		return ret;

	if (dev_get_platdata(dev)) {
		memcpy(plat, dev_get_platdata(dev), sizeof(*plat));
		return 0;
	}

	return -EINVAL;
}

static int gpio_nand_remove(struct platform_device *pdev)
{
	int i;
	struct gpiomtd_shared *shared = platform_get_drvdata(pdev);

	for (i = 0; i < shared->dev_count; i++) {
		nand_release(nand_to_mtd(&shared->gpiomtds[i]->nand_chip));
		if (gpio_is_valid(shared->gpiomtds[i]->plat.gpio_nce))
			gpio_set_value(shared->gpiomtds[i]->plat.gpio_nce, 1);
	}

	if (gpio_is_valid(shared->plat.gpio_nwp))
		gpio_set_value(shared->plat.gpio_nwp, 0);

	return 0;
}

static void __iomem *gpio_nand_get_reg(struct device *dev, struct device_node *pp, int *offset, int *size)
{
	const __be32 *reg;
	void __iomem *dest_ptr;
	int len;
	int a_cells, s_cells;

	reg = of_get_property(pp, "reg", &len);
	if (!reg) {
		dev_dbg(dev, "device %s missing reg property.\n", pp->name);
		return IOMEM_ERR_PTR(-EBUSY);
	}

	a_cells = of_n_addr_cells(pp);
	s_cells = of_n_size_cells(pp);
	if (len / 4 != a_cells + s_cells) {
		dev_err(dev, "error parsing reg property.\n");
		return IOMEM_ERR_PTR(-EBUSY);
	}

	*offset = of_read_number(reg, a_cells);
	*size = of_read_number(reg + a_cells, s_cells);

	dev_info(dev, "using address 0x%x: 0x%x\n", *offset, *size);
	if (!devm_request_mem_region(dev, *offset, *size, dev_name(dev))) {
		dev_err(dev, "can't request region\n");
		return IOMEM_ERR_PTR(-EBUSY);
	}

	dest_ptr = devm_ioremap(dev, *offset, *size);
	if (!dest_ptr) {
		dev_err(dev, "ioremap failed\n");
		dest_ptr = IOMEM_ERR_PTR(-ENOMEM);
	}

	return dest_ptr;
}


static int
gpio_nand_probe_device(struct platform_device* pdev,
		       struct device_node *nand_np,
		       struct gpiomtd *gpiomtd)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int ret = 0;

	chip = &gpiomtd->nand_chip;

	// Specific GPIOs
	gpiomtd->plat.gpio_nce = of_get_gpio(nand_np, 0);

	chip->IO_ADDR_R = gpio_nand_get_reg(&pdev->dev, nand_np, &gpiomtd->offset, &gpiomtd->size);
	if (IS_ERR(chip->IO_ADDR_R))
		return PTR_ERR(chip->IO_ADDR_R);

	if (gpio_is_valid(gpiomtd->plat.gpio_nce)) {
		ret = devm_gpio_request(&pdev->dev, gpiomtd->plat.gpio_nce,
					"NAND NCE");
		if (ret)
			return ret;
		gpio_direction_output(gpiomtd->plat.gpio_nce, 1);
	}

	nand_set_flash_node(chip, nand_np);
	chip->IO_ADDR_W		= chip->IO_ADDR_R;
	chip->ecc.mode		= NAND_ECC_SOFT;
	chip->ecc.algo		= NAND_ECC_BCH;
	chip->options		= gpiomtd->plat.options;
	chip->chip_delay	= gpiomtd->plat.chip_delay;
	chip->cmd_ctrl		= gpio_nand_cmd_ctrl;
	chip->dev_ready		= gpio_nand_devready;

	mtd			= nand_to_mtd(chip);
	mtd->dev.parent		= &pdev->dev;

	platform_set_drvdata(pdev, gpiomtd);

	ret = nand_scan(mtd, 1);
	if (ret)
		goto err_wp;

	if (gpiomtd->plat.adjust_parts)
		gpiomtd->plat.adjust_parts(&gpiomtd->plat,
					   mtd->size);

	ret = mtd_device_register(mtd,
				  gpiomtd->plat.parts,
				  gpiomtd->plat.num_parts);

	if (ret != 0)
		goto register_fail;

	return ret;

register_fail:
	nand_release(mtd);

err_wp:
	if (gpio_is_valid(gpiomtd->plat.gpio_nwp))
		gpio_set_value(gpiomtd->plat.gpio_nwp, 0);

	return ret;
}

static int gpio_nand_probe(struct platform_device *pdev)
{
	struct gpiomtd_shared *shared;
	struct device_node *nand_np;
	struct mtd_info* concatenated;
	int ret = 0;

	if (!pdev->dev.of_node && !dev_get_platdata(&pdev->dev))
		return -EINVAL;

	shared = devm_kzalloc(&pdev->dev, sizeof(*shared), GFP_KERNEL);
	if (!shared)
		return -ENOMEM;

	ret = gpio_nand_get_config(&pdev->dev, &shared->plat);

	// Request shared GPIO
	if (gpio_is_valid(shared->plat.gpio_nwp)) {
		ret = devm_gpio_request(&pdev->dev, shared->plat.gpio_nwp,
					"NAND NWP");
		if (ret)
			return ret;
	}

	if (gpio_is_valid(shared->plat.gpio_rdy)) {
		ret = devm_gpio_request(&pdev->dev, shared->plat.gpio_rdy,
					"NAND RDY");
		if (ret)
			return ret;
		gpio_direction_input(shared->plat.gpio_rdy);
	}

	ret = devm_gpio_request(&pdev->dev, shared->plat.gpio_ale, "NAND ALE");
	if (ret)
		return ret;
	gpio_direction_output(shared->plat.gpio_ale, 0);

	ret = devm_gpio_request(&pdev->dev, shared->plat.gpio_cle, "NAND CLE");
	if (ret)
		return ret;
	gpio_direction_output(shared->plat.gpio_cle, 0);

	for_each_child_of_node(pdev->dev.of_node, nand_np) {
		struct gpiomtd *gpiomtd;

		if (strcmp(nand_np->name, "nandflash") != 0)
			continue;

		gpiomtd = devm_kzalloc(&pdev->dev, sizeof(*gpiomtd), GFP_KERNEL);

		// Set shared GPIOs
		gpiomtd->plat.gpio_ale = shared->plat.gpio_ale;
		gpiomtd->plat.gpio_cle = shared->plat.gpio_cle;
		gpiomtd->plat.gpio_nwp = shared->plat.gpio_nwp;
		gpiomtd->plat.gpio_rdy = shared->plat.gpio_rdy;

		ret = gpio_nand_probe_device(pdev, nand_np, gpiomtd);
		if (ret) {
			dev_warn(&pdev->dev, "Cannot initialize nand device %s, ignoring\n", nand_np->full_name);
			of_node_put(nand_np);
			kfree(gpiomtd);
			continue;
		}

		shared->gpiomtds[shared->dev_count] = gpiomtd;
		subdevs[shared->dev_count] = nand_to_mtd(&gpiomtd->nand_chip);
		shared->dev_count += 1;

		of_node_put(nand_np);
	}

	if (!shared->dev_count) {
		dev_warn(&pdev->dev, "No valid NAND devices found\n");
		return -ENODEV;
	}

	if (shared->plat.concat) {
		dev_info(&pdev->dev, "Concatenating all %d devices\n", shared->dev_count);
		concatenated = mtd_concat_create(
			subdevs,                 /* subdevices to concatenate */
			shared->dev_count,       /* number of subdevices      */
			"backup-nand");          /* name for the new device   */

		ret = mtd_device_register(concatenated,
						shared->plat.parts,
						shared->plat.num_parts);

		if (ret) {
			dev_info(&pdev->dev, "cannot register concat mtd device\n");
			return ret;
		}
	}

	platform_set_drvdata(pdev, shared);

	gpio_direction_output(shared->plat.gpio_nwp, 1);

	return 0;
}

static struct platform_driver gpio_nand_driver = {
	.probe		= gpio_nand_probe,
	.remove		= gpio_nand_remove,
	.driver		= {
		.name	= "gpio-nand",
		.of_match_table = of_match_ptr(gpio_nand_id_table),
	},
};

module_platform_driver(gpio_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Detlev Casanova <dec@xiphos.com>");
MODULE_DESCRIPTION("GPIO SHARED NAND Driver");
