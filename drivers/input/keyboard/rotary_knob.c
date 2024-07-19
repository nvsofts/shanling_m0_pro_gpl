/*
 * linux/drivers/input/keyboard/gpio_keys.c
 *
 * JZ GPIO Buttons driver for JZ4740 PAVO
 *
 * User applications can access to this device via /dev/input/eventX.
 *
 * Copyright (c) 2005 - 2008  Ingenic Semiconductor Inc.
 *
 * Author: Richard <cjfeng@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <soc/gpio.h>

/*
 * rotary Buttons
 */
#ifdef CONFIG_PRODUCT_X1000_M2X
#define GPIO_VOLUME_KEY1         GPIO_PD(5) 
#define GPIO_VOLUME_KEY2 		 GPIO_PD(4) 
#elif defined(CONFIG_PRODUCT_X1000_M5S)
#define GPIO_VOLUME_KEY1         GPIO_PB(7) 
#define GPIO_VOLUME_KEY2 		 GPIO_PB(6) 
#elif defined(CONFIG_PRODUCT_X1000_Q1)
#define GPIO_VOLUME_KEY1         GPIO_PD(2) 
#define GPIO_VOLUME_KEY2 		 GPIO_PD(3) 
#elif defined(CONFIG_PRODUCT_X1000_H5)
#define GPIO_VOLUME_KEY1         GPIO_PD(4) 
#define GPIO_VOLUME_KEY2 			GPIO_PD(5) 
//±àÂëÆ÷2
#define GPIO_MENU_KEY1            GPIO_PB(22)
#define GPIO_MENU_KEY2            GPIO_PB(21)
#else
#define GPIO_VOLUME_KEY1         GPIO_PD(3) 
#define GPIO_VOLUME_KEY2 		 GPIO_PD(2) 
#endif


static struct gpio_keys_button rotary_buttons[] = {
#ifdef GPIO_VOLUME_KEY1
	{
		.gpio		= GPIO_VOLUME_KEY1,
		.code        	= KEY_1,
		.desc		= "rotary signal1",
		.active_low	= 1,
	},
#endif
#ifdef GPIO_VOLUME_KEY2

	{
		.gpio		= GPIO_VOLUME_KEY2,
		.code        	= KEY_2,
		.desc		= "rotary signal2",
		.active_low	= 1,
	},
#endif
#ifdef GPIO_MENU_KEY1
	{
		.gpio		= GPIO_MENU_KEY1,
		.code        	= KEY_3,
		.desc		= "rotary2 signal1",
		.active_low	= 1,
	},
#endif
#ifdef GPIO_MENU_KEY2
	{
		.gpio		= GPIO_MENU_KEY2,
		.code        	= KEY_4,
		.desc		= "rotary2 signal1",
		.active_low	= 1,
	},
#endif


};

static struct gpio_keys_platform_data rotary_button_data = {
	.buttons	=rotary_buttons,
	.nbuttons	= ARRAY_SIZE(rotary_buttons),
};

static struct platform_device rotary_button_device = {
	.name		= "rotary-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &rotary_button_data,
	}
};

struct rotary_knob{
	struct input_dev *rotary_input;
	struct work_struct rotary_work;
	unsigned int voladd;
	unsigned int voldec;

	struct work_struct rotary2_work;
	unsigned int menuadd;
	unsigned int menudec;
};

static void __init rotary_add_device_buttons(void)
{	
	platform_device_register(&rotary_button_device);
}
static void __init rotary_board_init(void)
{
	/* Push Buttons */
	rotary_add_device_buttons();
}

static void rotary_work_func(struct work_struct *rotary_work)
{
	struct rotary_knob *rotarydata =	container_of(rotary_work,struct rotary_knob, rotary_work);
	struct input_dev *rotary_input = rotarydata->rotary_input;
	
#ifdef CONFIG_PRODUCT_X1000_H5
	bool addflag = false;
	bool decflag = false;
	int value1 = __gpio_get_value(GPIO_VOLUME_KEY1);
	int value2 = __gpio_get_value(GPIO_VOLUME_KEY2);
	if((rotarydata->voladd == 0) && (rotarydata->voldec == 0))
		{
			if(value1 == 1 && value2 == 0)
			{
				rotarydata->voladd = 1;
			}
			else if(value1 == 0 && value2 == 1)
			{
				rotarydata->voldec = 1;
			}
		}	
		else if((rotarydata->voladd == 10) && (rotarydata->voldec == 10))
		{
			if(value1 == 0 && value2 == 1)
			{
				rotarydata->voladd = 11;
			}
			else if(value1 == 1 && value2 == 0)
			{
				rotarydata->voldec = 11;
			}
		}
		if(value1 == 0 && value2 == 0)
		{
			if(rotarydata->voladd == 1)
			{
				addflag = true;
			}
			else if(rotarydata->voldec == 1)
			{
				decflag = true;
			}
			rotarydata->voladd = 10;
			rotarydata->voldec = 10;		
		}	
		else if(value1 == 1 && value2 == 1)
		{
			if(rotarydata->voladd == 11)
			{
				addflag = true;
			}
			else if(rotarydata->voldec == 11)
			{
				decflag = true;
			}
			rotarydata->voladd = 0;
			rotarydata->voldec = 0;
		}
		if(addflag)
		{
			input_report_key(rotary_input,KEY_1,1);
			input_sync(rotary_input);
			input_report_key(rotary_input,KEY_1,0);
			input_sync(rotary_input);
		}
		else if(decflag)
		{
			input_report_key(rotary_input,KEY_2,1);
			input_sync(rotary_input);
			input_report_key(rotary_input,KEY_2,0);
			input_sync(rotary_input);
		}
#else
	int vol1value = __gpio_get_value(GPIO_VOLUME_KEY1);
	int vol2value = __gpio_get_value(GPIO_VOLUME_KEY2);
	//3?¨º??¡¥
	if(vol1value == 1 && vol2value == 1)
	{
		rotarydata->voladd= 0;
		rotarydata->voldec = 0;
	}
	//¦Ì¨²¨°?????3?
	if(rotarydata->voladd == 0 || rotarydata->voldec == 0)
	{
		if(vol1value == 0 && vol2value == 1)
		{
			rotarydata->voladd=1;
		}
		else if(vol1value == 1 && vol2value == 0)
		{
			rotarydata->voldec=1;
		}
	}
	//¦Ì¨²?t????3?
	if(rotarydata->voladd == 1 || rotarydata->voldec == 1)
	{	
		if(vol1value == 0 && vol2value == 0)
		{
				rotarydata->voladd++;
				rotarydata->voldec++;
		}
	}

	//¦Ì¨²¨¨y????3?
	if(rotarydata->voladd == 2 || rotarydata->voldec == 2)
	{
		if(vol1value == 1 && vol2value == 0)
		{
			rotarydata->voladd++;
		}
		else if(vol1value == 0 && vol2value == 1)
		{
			rotarydata->voldec++;
		}
	}
	if(rotarydata->voladd == 3)
	{
		input_report_key(rotary_input,KEY_1,1);
		input_sync(rotary_input);
		input_report_key(rotary_input,KEY_1,0);
		input_sync(rotary_input);
	}
	else if(rotarydata->voldec == 3)
	{
		input_report_key(rotary_input,KEY_2,1);
		input_sync(rotary_input);
		input_report_key(rotary_input,KEY_2,0);
		input_sync(rotary_input);
	}
#endif
}



static irqreturn_t rotary_keys_isr(int irq, void *dev_id)
{
	struct rotary_knob *rotarydata = dev_id;
	schedule_work(&rotarydata->rotary_work);
	
	return IRQ_HANDLED;
}



//**********±àÂëÆ÷**********************************
static void rotary2_work_func(struct work_struct *rotary2_work)
{
	struct rotary_knob *rotarydata =	container_of(rotary2_work,struct rotary_knob, rotary2_work);
	struct input_dev *rotary_input = rotarydata->rotary_input;
	
	bool addflag = false;
	bool decflag = false;
	int value1 =0;
	int value2 = 0;
#ifdef GPIO_MENU_KEY1
	value1 = __gpio_get_value(GPIO_MENU_KEY1);
#endif
#ifdef GPIO_MENU_KEY2
	value2 = __gpio_get_value(GPIO_MENU_KEY2);
#endif
	if((rotarydata->menuadd == 0) && (rotarydata->menudec == 0))
		{
			if(value1 == 1 && value2 == 0)
			{
				rotarydata->menuadd = 1;
			}
			else if(value1 == 0 && value2 == 1)
			{
				rotarydata->menudec = 1;
			}
		}	
		else if((rotarydata->menuadd == 10) && (rotarydata->menudec == 10))
		{
			if(value1 == 0 && value2 == 1)
			{
				rotarydata->menuadd = 11;
			}
			else if(value1 == 1 && value2 == 0)
			{
				rotarydata->menudec = 11;
			}
		}
		if(value1 == 0 && value2 == 0)
		{
			if(rotarydata->menuadd == 1)
			{
				addflag = true;
			}
			else if(rotarydata->menudec == 1)
			{
				decflag = true;
			}
			rotarydata->menuadd = 10;
			rotarydata->menudec = 10;		
		}	
		else if(value1 == 1 && value2 == 1)
		{
			if(rotarydata->menuadd == 11)
			{
				addflag = true;
			}
			else if(rotarydata->menudec == 11)
			{
				decflag = true;
			}
			rotarydata->menuadd = 0;
			rotarydata->menudec = 0;
		}
		if(addflag)
		{
			input_report_key(rotary_input,KEY_3,1);
			input_sync(rotary_input);
			input_report_key(rotary_input,KEY_3,0);
			input_sync(rotary_input);
		}
		else if(decflag)
		{
			input_report_key(rotary_input,KEY_4,1);
			input_sync(rotary_input);
			input_report_key(rotary_input,KEY_4,0);
			input_sync(rotary_input);
		}
}


static irqreturn_t rotary2_keys_isr(int irq, void *dev_id)
{
	struct rotary_knob *rotarydata = dev_id;
	schedule_work(&rotarydata->rotary2_work);
	
	return IRQ_HANDLED;
}


static int rotary_keys_probe(struct platform_device *pdev)
{

	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	struct rotary_knob *rotaryknob;
	int i, error;
	int wakeup = 0;

	rotaryknob = kzalloc(sizeof(struct rotary_knob), GFP_KERNEL);
	input = input_allocate_device();
	if (!rotaryknob && !input)
		return -ENOMEM;

	rotaryknob->rotary_input = input;
	rotaryknob->voladd = 0;
	rotaryknob->voldec = 0;
	rotaryknob->menuadd  = 0;
	rotaryknob->menudec  = 0;
	
	platform_set_drvdata(pdev, input);
	INIT_WORK(&rotaryknob->rotary_work, rotary_work_func);
	INIT_WORK(&rotaryknob->rotary2_work, rotary2_work_func);
	input->name = pdev->name;
	input->phys = "rotary-keys/input1";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0002;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) | BIT(EV_REP);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		if (gpio_is_valid(button->gpio))
		{
			error = gpio_request_one(button->gpio, GPIOF_IN, button->desc);	
			if (error < 0) {			
					pr_err("Failed to request GPIO %d, error %d\n",	button->gpio, error);
					return error;
			}
			int irq;
			irq = gpio_to_irq(button->gpio);
			if (irq < 0) {
				error = irq;
				pr_err("Unable to get irq number for GPIO %d, error %d\n",button->gpio, error);
				goto fail;
			}	
			if(i == 2 || i == 3)
			{
				error = request_any_context_irq(irq, rotary2_keys_isr,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,button->desc ? button->desc : "rotary2_keys",rotaryknob);
			}
			else
			{
				error = request_any_context_irq(irq, rotary_keys_isr,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,button->desc ? button->desc : "rotary_keys",rotaryknob);
			}
			if (error) {
				pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
					irq, error);
				goto fail;
			}
			if (button->wakeup)
				wakeup = 1;
			input_set_capability(input, button->type ?: EV_KEY, button->code);
		}
	}

#ifdef CONFIG_PRODUCT_X1000_H5
		//*****±àÂëÆ÷1******
		int gpiovalue1,gpiovalue2;
		gpiovalue1 = __gpio_get_value(GPIO_VOLUME_KEY1);
		gpiovalue2 = __gpio_get_value(GPIO_VOLUME_KEY2);
		if(gpiovalue1 == 0 && gpiovalue2 == 0) //´¦ÓÚµÍµçÆ½
		{
			rotaryknob->voladd = 10;
			rotaryknob->voldec = 10;
		}
		else if(gpiovalue1 == 1 && gpiovalue2 == 1) //´¦ÓÚ¸ßµçÆ½
		{
			rotaryknob->voladd = 0;
			rotaryknob->voldec = 0;
		}
		//*****±àÂëÆ÷2å*å*å*å*å*
		gpiovalue1 = __gpio_get_value(GPIO_MENU_KEY1);
		gpiovalue2 = __gpio_get_value(GPIO_MENU_KEY2);
		if(gpiovalue1 == 0 && gpiovalue2 == 0) //´¦ÓÚµÍµçÆ½
		{
			rotaryknob->menuadd = 10;
			rotaryknob->menudec = 10;
		}
		else if(gpiovalue1 == 1 && gpiovalue2 == 1) //´¦ÓÚ¸ßµçÆ½
		{
			rotaryknob->menuadd = 0;
			rotaryknob->menudec = 0;
		}
#endif


	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail;
	}

	device_init_wakeup(&pdev->dev, wakeup);
	return 0;

 fail:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio) , pdev);
	}

	platform_set_drvdata(pdev, NULL);
	input_free_device(input);

	return error;
}

static int rotary_keys_remove(struct platform_device *pdev)
{

	struct input_dev *input = platform_get_drvdata(pdev);
	device_init_wakeup(&pdev->dev, 0);

	input_unregister_device(input);
	return 0;
}


static int rotary_keys_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int rotary_keys_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver rotary_keys_device_driver = {
	.probe		= rotary_keys_probe,
	.remove		= rotary_keys_remove,
	.suspend	= rotary_keys_suspend,
	.resume		= rotary_keys_resume,
	.driver		= {
		.name	= "rotary-keys",
	}
};

static int __init rotary_keys_init(void)
{
	rotary_board_init();
	return platform_driver_register(&rotary_keys_device_driver);
}

static void __exit rotary_keys_exit(void)
{
	platform_device_unregister(&rotary_button_device);
	platform_driver_unregister(&rotary_keys_device_driver);
}

late_initcall(rotary_keys_init);
module_exit(rotary_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
