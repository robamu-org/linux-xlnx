/*
 * Copyright (C) 2020 Xiphos System Corporation.
 */

#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/bitfield.h>

#define REG_CONV_RESULT(ch)	(0x20 + 4 * ch)
#define BIT_CONV_VALUE		GENMASK(11, 0)
#define BIT_CONV_NACK		BIT(12)

struct xsc_padc {
	void __iomem *base_addr;
};

#define XSC_PADC_CHANNEL(n, t)					\
	{							\
		.indexed	= 1,				\
		.channel	= n,				\
		.datasheet_name	= "channel"#n,			\
		.type		= t,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	}

static const struct iio_chan_spec xsc_padc_channels[] = {
	XSC_PADC_CHANNEL(0, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(1, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(2, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(3, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(4, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(5, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(6, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(7, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(8, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(9, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(10, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(11, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(12, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(13, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(14, IIO_VOLTAGE),
	XSC_PADC_CHANNEL(15, IIO_VOLTAGE),
};

static int xsc_padc_read(struct iio_dev *indio_dev, int channel)
{
	struct xsc_padc *info = iio_priv(indio_dev);
	int index = channel / 2;
	u32 conv_reg;

	conv_reg = ioread32(info->base_addr + REG_CONV_RESULT(index));
	if (channel % 2)
		conv_reg >>= 16;

	if (FIELD_GET(BIT_CONV_NACK, conv_reg))
		return -EINVAL;
	else
		return FIELD_GET(BIT_CONV_VALUE, conv_reg);
}

static int xsc_padc_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type != IIO_VOLTAGE)
			return -EINVAL;

		*val = xsc_padc_read(indio_dev, chan->channel);
		if (*val < 0)
			return *val;

		return IIO_VAL_INT;
	default:
		break;
	}
	return -EINVAL;
}

static const struct iio_info xsc_padc_iio_info = {
	.driver_module	= THIS_MODULE,
	.read_raw	= xsc_padc_read_raw,
};

static int xsc_padc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct xsc_padc *info;
	struct resource *res;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(info->base_addr))
		return PTR_ERR(info->base_addr);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;
	indio_dev->channels = xsc_padc_channels;
	indio_dev->num_channels = ARRAY_SIZE(xsc_padc_channels);
	indio_dev->info = &xsc_padc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return iio_device_register(indio_dev);
}

static int xsc_padc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id of_xsc_padc_match[] = {
	{ .compatible = "xsc,padc",},
	{},
};

MODULE_DEVICE_TABLE(of, of_xsc_padc_match);

static struct platform_driver xsc_padc_driver = {
	.probe		= xsc_padc_probe,
	.remove		= xsc_padc_remove,
	.driver		= {
		.name	= "xsc_padc",
		.of_match_table = of_xsc_padc_match,
	},
};

module_platform_driver(xsc_padc_driver);

MODULE_DESCRIPTION("Xiphos ADC through ProASIC3");
MODULE_LICENSE("GPL v2");
