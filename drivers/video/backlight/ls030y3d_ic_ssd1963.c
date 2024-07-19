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

struct sharp_tft480x800_data {
	int lcd_power;
	struct lcd_device *lcd;
	struct lcd_platform_data *ctrl;
	struct regulator *lcd_vcc_reg;
};

static int sharp_tft480x800_set_power(struct lcd_device *lcd, int power)
{
	struct sharp_tft480x800_data *dev= lcd_get_data(lcd);
	if (!power && dev->lcd_power) {
		dev->ctrl->power_on(lcd, 1);
	} else if (power && !dev->lcd_power) {
		if (dev->ctrl->reset) {
			dev->ctrl->reset(lcd);
		}
		dev->ctrl->power_on(lcd, 0);
	}
	dev->lcd_power = power;
	return 0;
}

static int sharp_tft480x800_get_power(struct lcd_device *lcd)
{
	struct sharp_tft480x800_data *dev= lcd_get_data(lcd);

	return dev->lcd_power;
}

static int sharp_tft480x800_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	return 0;
}

static struct lcd_ops sharp_tft480x800_ops = {
	.set_power = sharp_tft480x800_set_power,
	.get_power = sharp_tft480x800_get_power,
	.set_mode = sharp_tft480x800_set_mode,
};

static int sharp_tft480x800_probe(struct platform_device *pdev)
{
	int ret;
	struct sharp_tft480x800_data *dev;
	dev = kzalloc(sizeof(struct sharp_tft480x800_data), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->ctrl = pdev->dev.platform_data;
	if (dev->ctrl == NULL) {
		dev_info(&pdev->dev, "no platform data!");
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, dev);

	dev->lcd = lcd_device_register("sharp_tft480x800_slcd", &pdev->dev,
				       dev, &sharp_tft480x800_ops);
	if (IS_ERR(dev->lcd)) {
		ret = PTR_ERR(dev->lcd);
		dev->lcd = NULL;
		dev_info(&pdev->dev, "lcd device register error: %d\n", ret);
	} else {
		dev_info(&pdev->dev, "lcd device(sharp TFT480x800) register success\n");
	}

	if (dev->ctrl->power_on) {
		dev->ctrl->power_on(dev->lcd, 1);
		dev->lcd_power = FB_BLANK_UNBLANK;
	}
	return 0;
}

static int sharp_tft480x800_remove(struct platform_device *pdev)
{
	struct sharp_tft480x800_data *dev = dev_get_drvdata(&pdev->dev);

	if (dev->lcd_power)
		dev->ctrl->power_on(dev->lcd, 0);
	lcd_device_unregister(dev->lcd);
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(dev);

	return 0;
}

#ifdef CONFIG_PM
static int sharp_tft480x800_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int sharp_tft480x800_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define sharp_tft480x800_suspend	NULL
#define sharp_tft480x800_resume	NULL
#endif

static struct platform_driver sharp_tft480x800_driver = {
	.driver		= {
		.name	= "sharp_tft480x800_slcd",
		.owner	= THIS_MODULE,
	},
	.probe		= sharp_tft480x800_probe,
	.remove		= sharp_tft480x800_remove,
	.suspend	= sharp_tft480x800_suspend,
	.resume		= sharp_tft480x800_resume,
};

static int __init sharp_tft480x800_init(void)
{
	return platform_driver_register(&sharp_tft480x800_driver);
}
module_init(sharp_tft480x800_init);

static void __exit sharp_tft480x800_exit(void)
{
	platform_driver_unregister(&sharp_tft480x800_driver);
}
module_exit(sharp_tft480x800_exit);

MODULE_DESCRIPTION("SHART TFT480x800 lcd panel driver");
MODULE_LICENSE("GPL");
