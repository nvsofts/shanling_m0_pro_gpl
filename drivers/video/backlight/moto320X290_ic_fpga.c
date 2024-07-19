#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/lcd.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

struct moto320X290_data {
	int lcd_power;
	struct lcd_device *lcd;
	struct lcd_platform_data *ctrl;
};

static int moto320X290_set_power(struct lcd_device *lcd, int power)
{
	struct moto320X290_data *dev= lcd_get_data(lcd);
	if (!power && dev->lcd_power) {
		dev->ctrl->power_on(lcd, 1);
	} else if (power && !dev->lcd_power) {
		if (dev->ctrl->reset)
			dev->ctrl->reset(lcd);

		dev->ctrl->power_on(lcd, 0);
	}
	dev->lcd_power = power;
	return 0;
}

static int moto320X290_get_power(struct lcd_device *lcd)
{
	struct moto320X290_data *dev= lcd_get_data(lcd);

	return dev->lcd_power;
}

static int moto320X290_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	return 0;
}

static struct lcd_ops moto320X290_ops = {
	.set_power = moto320X290_set_power,
	.get_power = moto320X290_get_power,
	.set_mode = moto320X290_set_mode,
};

static int moto320X290_probe(struct platform_device *pdev)
{
	int ret;
	struct moto320X290_data *dev;
	dev = kzalloc(sizeof(struct moto320X290_data), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	printk("file:%s add by zxj func:%s\n",__func__,__FILE__);
	
	dev->ctrl = pdev->dev.platform_data;
	if (dev->ctrl == NULL) {
		dev_info(&pdev->dev, "no platform data!");
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, dev);

	dev->lcd = lcd_device_register("moto320X290", &pdev->dev,
				       dev, &moto320X290_ops);
	if (IS_ERR(dev->lcd)) {
		ret = PTR_ERR(dev->lcd);
		dev->lcd = NULL;
		dev_info(&pdev->dev, "lcd device register error: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "lcd device register success\n");
	}

	if (dev->ctrl->power_on) {
		dev->ctrl->power_on(dev->lcd, 1);
		dev->lcd_power = FB_BLANK_UNBLANK;
	}

	return 0;
}

static int moto320X290_remove(struct platform_device *pdev)
{
	struct moto320X290_data *dev = dev_get_drvdata(&pdev->dev);

	if (dev->ctrl->power_on)
		dev->ctrl->power_on(dev->lcd, 0);
	lcd_device_unregister(dev->lcd);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(dev);

	return 0;
}


static struct platform_driver moto320X290_driver = {
	.driver		= {
		.name	= "moto320X290_slcd",
		.owner	= THIS_MODULE,
	},
	.probe		= moto320X290_probe,
	.remove		= moto320X290_remove,

};

static int __init moto320X290_init(void)
{
	return platform_driver_register(&moto320X290_driver);
}

static void __exit moto320X290_exit(void)
{
	platform_driver_unregister(&moto320X290_driver);
}

module_init(moto320X290_init);
module_exit(moto320X290_exit);
MODULE_LICENSE("GPL");
