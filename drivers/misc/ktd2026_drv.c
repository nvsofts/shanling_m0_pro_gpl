/*
 * ak4493.c  --  audio driver for AK4493
 *
 * Copyright (C) 2018 Asahi Kasei Microdevices Corporation
 *  Author             Date      Revision      kernel version
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                   18/02/07      1.0          3.18.XX
 *                     18/03/07      1.1          4.4.XX
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>

#include "ktd2026_drv.h"



static unsigned char led_value = 0xFF;
static struct i2c_client *ktd2026_i2c_dev = NULL;



int ktd2026_i2c_write(unsigned char reg,unsigned char value)
{
	int ret = -1;
	int len = 1;
	unsigned char buf[4] = {0};
	if(ktd2026_i2c_dev == NULL)	
	{
		printk("=====ktd2026_i2c_dev is null====\n");
		return 0;
	}
	buf[0] = reg;
	buf[1] = value;
	ret = i2c_master_send(ktd2026_i2c_dev, buf, len+1);
	if (ret < len+1)
	{
		printk("%s 0x%02x err %d!\n", __func__, reg, ret);
	}
	return ret < len+1 ? ret : 0;
}


//D1----R
//D2----G
//D3----B

void ktd2026_init(void)
{
	unsigned char TempChar;
#if 0	//i2c±¨´í
	TempChar = (3<<5)|(3<<3)|(7<<0);
	ktd2026_i2c_write(0x00,TempChar);
	mdelay(20);
	TempChar = (0<<5)|(3<<3);
	ktd2026_i2c_write(0x00,TempChar);
#endif
	ktd2026_i2c_write(0x06,0x00);	//r
	ktd2026_i2c_write(0x07,0x00);	//g
	ktd2026_i2c_write(0x08,0x00);	//b
	ktd2026_i2c_write(0x04,0x00);
}


void ktd2026_control(unsigned char LedType)
{
	unsigned char TempChar;
#if defined(CONFIG_PRODUCT_X1000_CA80) || defined(CONFIG_PRODUCT_X1000_CD80)
	switch(LedType)
	{
		case LED_WHITE:
			ktd2026_i2c_write(0x06,0x38);	//r
			ktd2026_i2c_write(0x07,0x40);	//g
			ktd2026_i2c_write(0x08,0x20);	//b
			//RGB			
			TempChar = (1<<0)|(1<<2)|(1<<4);	
			break;
		
		case LED_BLACK:
			TempChar = 0;	
			break;
			
		case LED_RED:
			ktd2026_i2c_write(0x08,0x60);	//R
			TempChar = (1<<4);	
			break;
		
		case LED_GREEN:
			ktd2026_i2c_write(0x06,0x20);	//G
			TempChar = (1<<0);	
			break;

		case LED_BLUE:
			ktd2026_i2c_write(0x07,0x60);	//B
			TempChar = (1<<2);	
			break;
		
		case LED_YELLOW:
			ktd2026_i2c_write(0x08,0x60);	//R
			ktd2026_i2c_write(0x06,0x20);	//G	
			TempChar = (1<<4)|(1<<0);	
			break;		
		
		case LED_PURPLE:
			ktd2026_i2c_write(0x08,0x60);	//R		
			ktd2026_i2c_write(0x07,0x60);	//B
			TempChar = (1<<4)|(1<<2);
			break;
		
		case LED_CYAN:
			ktd2026_i2c_write(0x06,0x20);	//G
			ktd2026_i2c_write(0x07,0x60);	//B	
			TempChar = (1<<2)|(1<<0);	
			break;
		case LED_MQARED:
			ktd2026_i2c_write(0x08,0x1a);	//R 		
			ktd2026_i2c_write(0x07,0x25);	//B 
			TempChar = (1<<4)|(1<<2);
			break;
		default:
			TempChar = 0;	
			break;
	}
#else
	switch(LedType)
	{
		case LED_WHITE:
			ktd2026_i2c_write(0x06,0x38);	//r
			ktd2026_i2c_write(0x07,0x40);	//g
			ktd2026_i2c_write(0x08,0x20);	//b
			//RGB			
			TempChar = (1<<0)|(1<<2)|(1<<4);	
			break;
		
		case LED_BLACK:
			TempChar = 0;	
			break;
		
		case LED_RED:
			ktd2026_i2c_write(0x06,0x20);	//R
			TempChar = (1<<0);	
			break;
		
		case LED_GREEN:
			ktd2026_i2c_write(0x07,0x60);	//G
			TempChar = (1<<2);	
			break;
		
		case LED_BLUE:
			ktd2026_i2c_write(0x08,0x60);	//B
			TempChar = (1<<4);	
			break;
		
		case LED_YELLOW:
			ktd2026_i2c_write(0x06,0x28);	//R
			ktd2026_i2c_write(0x07,0x38);	//G 	
			TempChar = (1<<2)|(1<<0);	
			break;		
		
		case LED_PURPLE:
			ktd2026_i2c_write(0x06,0x28);	//R 		
			ktd2026_i2c_write(0x08,0x38);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		
		case LED_CYAN:
			ktd2026_i2c_write(0x07,0x38);	//G
			ktd2026_i2c_write(0x08,0x40);	//B 	
			TempChar = (1<<2)|(1<<4);	
			break;
		case LED_MQARED:
			ktd2026_i2c_write(0x06,0x1a);	//R 		
			ktd2026_i2c_write(0x08,0x25);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		default:
			TempChar = 0;	
			break;
	}
#endif
	ktd2026_i2c_write(0x04,TempChar);
	led_value = LedType;

}

static ssize_t ledktd2026_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	return sprintf(ubuf,"led_value=%d[0:RED;1:GREEN;2:BLUE;3:YELLOW;4:PURPLE;5:CYAN;6:WHITE;7:BLACK]\n",led_value);
}
static ssize_t ledktd2026_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	int value;
	int ret;
	ret = kstrtoint(ubuf,10,&value);
	if(ret < 0)
	{
		printk("===kstrtoint error==\n");
		return ret;
	}
	if(value >= LED_RED && value <=  LED_MQARED && value != led_value)
	{
		ktd2026_control(value);
	}
	return count;
}


static DEVICE_ATTR(ledktd2026, 0644, ledktd2026_show, ledktd2026_store);



static int ktd2026_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	int ret = 0;

	ktd2026_i2c_dev = i2c;
	printk("=======fun[%s]===\n",__FUNCTION__);
	ktd2026_init();
	ret = device_create_file(&i2c->dev, &dev_attr_ledktd2026);
	if(ret != 0)
	{
		dev_err(&i2c->dev,  "Failed to create xxx sysfs files: %d\n",ret);
	}
	//ktd2026_control(LED_WHITE);
	return ret;
}

static int  ktd2026_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ktd2026_i2c_id[] = {
    { "ktd2026", 0 },
};


static struct i2c_driver ktd2026_i2c_driver = {
    .driver = {
        .name = "ktd2026",
    },
    .probe = ktd2026_i2c_probe,
    .remove = ktd2026_i2c_remove,
    .id_table = ktd2026_i2c_id,
};



static int __init ktd2026_led_init(void)
{
	int ret =0;
	
	ret = i2c_add_driver(&ktd2026_i2c_driver);
    if (ret)
	{
        printk(KERN_ERR "ktd2026: failed to register i2c driver\n");
		i2c_unregister_device(ktd2026_i2c_dev);
        return ret;
    }
	return ret;
}

static void __exit ktd2026_led_exit(void)
{
    i2c_del_driver(&ktd2026_i2c_driver);
}

module_init(ktd2026_led_init);
module_exit(ktd2026_led_exit);

MODULE_DESCRIPTION("led ktd2026 driver");
MODULE_LICENSE("GPL v2");
