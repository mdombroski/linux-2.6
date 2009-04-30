/*
 * Copyright (C) 2009 4D Electronics
 *
 * Backlight driver using GPIO.
 * Brightness can be 0 or 1
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/fb.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/gpio_backlight.h>


static int gpio_bl_update_status(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;
	struct gpio_backlight_data *data = bl_get_data(dev);
	
	int power = max(props->power, props->fb_blank);
	int bright = props->brightness;
	
	if (power)
		bright = 0;

	if (data->set_backlight)
		data->set_backlight(bright);

	return 0;
}


static int gpio_bl_get_brightness(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;

	return props->brightness;
}


static struct backlight_ops bl_ops = {
	.get_brightness		= gpio_bl_get_brightness,
	.update_status		= gpio_bl_update_status,
};


static int __devinit gpio_bl_probe(struct platform_device *pdev)
{
	struct gpio_backlight_data *pdata;
	struct backlight_device *bldev;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		return -ENODEV;
	}

	bldev = backlight_device_register("gpio-backlight", &pdev->dev, pdata, &bl_ops);
	if (IS_ERR(bldev)) {
		return PTR_ERR(bldev);
	}

	platform_set_drvdata(pdev, bldev);

	/* Power up the backlight by default on */
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.max_brightness = 1;
	bldev->props.brightness = bldev->props.max_brightness;

	backlight_update_status(bldev);

	printk("GPIO Backlight Driver Initialized.\n");
	return 0;
}


static int gpio_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct gpio_backlight_data *data = pdev->dev.platform_data;

	/* First turn off backlight */
	if (data->set_backlight)
		data->set_backlight(0);

	backlight_device_unregister(bd);

	printk("GPIO Backlight Driver Unloaded\n");
	return 0;
}


#ifdef CONFIG_PM
static int gpio_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gpio_backlight_data *data = pdev->dev.platform_data;
	
	if (data->set_backlight)
		data->set_backlight(0);

	return 0;
}

static int gpio_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);

	gpio_bl_update_status( dev );

	return 0;
}
#else
#define gpio_bl_suspend NULL
#define gpio_bl_resume NULL
#endif


static struct platform_driver gpio_bl_driver = {
	.driver = {
		.name		= "gpio-backlight",
	},
	.probe		= gpio_bl_probe,
	.remove		= __devexit_p(gpio_bl_remove),
	.suspend	= gpio_bl_suspend,
	.resume		= gpio_bl_resume,
};

static int __init gpio_bl_init(void)
{
	return platform_driver_register(&gpio_bl_driver);
}

static void __exit gpio_bl_exit(void)
{
	platform_driver_register(&gpio_bl_driver);
}

module_init(gpio_bl_init);
module_exit(gpio_bl_exit);

MODULE_AUTHOR("Matthew Dombroski");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LCD/Backlight control for Atmel AT91 series using GPIO pin");

