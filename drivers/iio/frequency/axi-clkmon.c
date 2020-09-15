/*
 * Driver for the Xiphos Clock Monitoring IP Core
 *
 * Copyright (c) 2020, Xiphos Systems Corp. All rights reserved.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_device.h>

#include <linux/iio/iio.h>

#define CLKMON_CORE_ID_REG	0x00
#define CLKMON_CORE_VERSION_REG	0x04
#define CLKMON_CORE_N_CLKS	0x08
#define CLKMON_CORE_FIRST_CLK	0x0c

struct axi_clkmon {
	void __iomem *base;
	struct clk *axi_clk;
	struct clk *ref_clk;
	int num_channels;
	struct iio_chan_spec *channels;
};

static int axi_clkmon_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec
			       const *chan, int *val, int *val2, long m)
{
	struct axi_clkmon *clkmon = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_FREQUENCY:
		*val = (int) readl(clkmon->base + chan->address);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};
static const struct iio_info axi_clkmon_info = {
	.read_raw = axi_clkmon_read_raw,
};

static int axi_clkmon_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct axi_clkmon *clkmon;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int ret;
	int i;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct axi_clkmon));
	if (!indio_dev)
		return -ENOMEM;

	clkmon = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	clkmon->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(clkmon->base))
		return PTR_ERR(clkmon->base);

	clkmon->axi_clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(clkmon->axi_clk))
		return PTR_ERR(clkmon->axi_clk);

	ret = clk_prepare_enable(clkmon->axi_clk);
	if (ret)
		return ret;

	clkmon->ref_clk = devm_clk_get(&pdev->dev, "ref_clk");
	if (IS_ERR(clkmon->ref_clk))  {
		ret = PTR_ERR(clkmon->ref_clk);
		goto err_disable_axi_clk;
	}

	ret = clk_prepare_enable(clkmon->ref_clk);
	if (ret)
		goto err_disable_axi_clk;

	clkmon->num_channels = readl(clkmon->base + CLKMON_CORE_N_CLKS);

	clkmon->channels = devm_kcalloc(&pdev->dev, clkmon->num_channels,
					sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!clkmon->channels) {
		ret = -ENOMEM;
		goto err_disable_ref_clk;
	}

	for (i = 0; i < clkmon->num_channels; i++) {
		clkmon->channels[i].type = IIO_COUNT;
		clkmon->channels[i].indexed = 1;
		clkmon->channels[i].channel = i;
		clkmon->channels[i].info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
		clkmon->channels[i].address = CLKMON_CORE_FIRST_CLK + 0x4 * i;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->info = &axi_clkmon_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = clkmon->channels;
	indio_dev->num_channels = clkmon->num_channels;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_disable_ref_clk;

	platform_set_drvdata(pdev, indio_dev);

	dev_info(&pdev->dev, "successfully initialized with %d channels",
		 clkmon->num_channels);

	return 0;

err_disable_ref_clk:
	clk_disable_unprepare(clkmon->ref_clk);
err_disable_axi_clk:
	clk_disable_unprepare(clkmon->axi_clk);

	return ret;
}

static int axi_clkmon_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct axi_clkmon *clkmon = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	clk_disable_unprepare(clkmon->ref_clk);
	clk_disable_unprepare(clkmon->axi_clk);

	return 0;
}

static const struct of_device_id axi_clkmon_of_match[] = {
	{ .compatible = "xsc,axi-clkmon-1.0"},
	{ },
};
MODULE_DEVICE_TABLE(of, axi_clkmon_of_match);

static struct platform_driver axi_clkmon_of_driver = {
	.driver = {
		.name = "axi-clkmon",
		.of_match_table = axi_clkmon_of_match,
	},
	.probe = axi_clkmon_probe,
	.remove	= axi_clkmon_remove,
};
module_platform_driver(axi_clkmon_of_driver);

MODULE_AUTHOR("Signed-off-by: Liam Beguin <lvb@xiphos.com>");
MODULE_DESCRIPTION("Xiphos Clock Monitoring Driver");
MODULE_LICENSE("GPL v2");
