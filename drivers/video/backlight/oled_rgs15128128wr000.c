/*
 *  LCD control code for truly
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

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

struct oled_rgs15128128wr000_data {
	int lcd_power;
	struct lcd_device *lcd;
	struct lcd_platform_data *ctrl;
	struct regulator *lcd_vcc_reg;
};

static int oled_rgs15128128wr000_set_power(struct lcd_device *lcd, int power)
{
	struct oled_rgs15128128wr000_data *dev= lcd_get_data(lcd);

	if (dev->ctrl->power_on) {
		dev->ctrl->power_on(lcd, power);
	}
    dev->lcd_power = power;
	return 0;
}

static int oled_rgs15128128wr000_get_power(struct lcd_device *lcd)
{
	struct oled_rgs15128128wr000_data *dev= lcd_get_data(lcd);

	return dev->lcd_power;
}

static int oled_rgs15128128wr000_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	return 0;
}

static struct lcd_ops oled_rgs15128128wr000_ops = {
	.set_power = oled_rgs15128128wr000_set_power,
	.get_power = oled_rgs15128128wr000_get_power,
	.set_mode = oled_rgs15128128wr000_set_mode,
};

static int oled_rgs15128128wr000_probe(struct platform_device *pdev)
{
	int ret;
	struct oled_rgs15128128wr000_data *dev;
	dev = kzalloc(sizeof(struct oled_rgs15128128wr000_data), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
		
	dev->ctrl = pdev->dev.platform_data;
	if (dev->ctrl == NULL) {
		dev_info(&pdev->dev, "no platform data!");
		return -EINVAL;
	}
	
	dev_set_drvdata(&pdev->dev, dev);

	dev->lcd = lcd_device_register("oled_rgs15128128wr000_slcd", &pdev->dev,
				       dev, &oled_rgs15128128wr000_ops);
	if (IS_ERR(dev->lcd)) {
		ret = PTR_ERR(dev->lcd);
		dev->lcd = NULL;
		dev_info(&pdev->dev, "lcd device register error: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "lcd device(TRULY TFT240240) register success\n");
	}

	if (dev->ctrl->power_on) {
		dev->ctrl->power_on(dev->lcd, 1);
		dev->lcd_power = 1;
	}
		
	return 0;
}

static int oled_rgs15128128wr000_remove(struct platform_device *pdev)
{
	struct oled_rgs15128128wr000_data *dev = dev_get_drvdata(&pdev->dev);

	if (dev->lcd_power)
		dev->ctrl->power_on(dev->lcd, 0);
	regulator_put(dev->lcd_vcc_reg);
	lcd_device_unregister(dev->lcd);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(dev);

	return 0;
}

#ifdef CONFIG_PM
static int oled_rgs15128128wr000_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int oled_rgs15128128wr000_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define oled_rgs15128128wr000_suspend	NULL
#define oled_rgs15128128wr000_resume	NULL
#endif

static struct platform_driver oled_rgs15128128wr000_driver = {
	.driver		= {
		.name	= "oled_rgs15128128wr000_slcd",
		.owner	= THIS_MODULE,
	},
	.probe		= oled_rgs15128128wr000_probe,
	.remove		= oled_rgs15128128wr000_remove,
	.suspend	= oled_rgs15128128wr000_suspend,
	.resume		= oled_rgs15128128wr000_resume,
};

static int __init oled_rgs15128128wr000_init(void)
{
	return platform_driver_register(&oled_rgs15128128wr000_driver);
}
module_init(oled_rgs15128128wr000_init);

static void __exit oled_rgs15128128wr000_exit(void)
{
	platform_driver_unregister(&oled_rgs15128128wr000_driver);
}
module_exit(oled_rgs15128128wr000_exit);

MODULE_DESCRIPTION("oled_rgs15128128wr000 lcd panel driver");
MODULE_LICENSE("GPL");
