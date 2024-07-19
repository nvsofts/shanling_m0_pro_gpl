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
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define	LED_RED			0
#define	LED_GREEN		1
#define	LED_BLUE		2
#define	LED_YELLOW	3
#define	LED_PURPLE	4
#define	LED_CYAN		5
#define	LED_WHITE		6
#define	LED_BLACK		7
#define LED_MQARED      8

#define GPIO_I2C 1
#define LEDS_SDA GPIO_PC(4)
#define LEDS_SCL GPIO_PC(5)


static unsigned char led_mqa_value = 0xFF;
static unsigned char led_khz_value = 0xFF;





#ifdef GPIO_I2C
static void leds_iicstart(void)
{
	gpio_direction_output(LEDS_SDA,1);
	gpio_direction_output(LEDS_SCL,1);
	gpio_direction_output(LEDS_SDA,0);
	gpio_direction_output(LEDS_SCL,0);
}

static int leds_iicwr(u8 dat)
{
	u8 i, temp, temp1;
	temp1=dat;
	for(i=0;i<8;i++)
	{
		gpio_direction_output(LEDS_SCL,0);
		temp=temp1;
		temp=temp&0x80;
		if(temp==0x80)
		{
			gpio_direction_output(LEDS_SDA,1);
		}
		else
		{
			gpio_direction_output(LEDS_SDA,0);
		}
		gpio_direction_output(LEDS_SCL,1);
		temp1=temp1<<1;
	}
	gpio_direction_output(LEDS_SCL,0);
	gpio_direction_input(LEDS_SDA);
	gpio_direction_output(LEDS_SCL,1);
	while((gpio_get_value(LEDS_SDA)==1)&&(i<50))
	{
		i++;
		udelay(1);
	}
	if(i<50)
		return 0;
	return -1;
}


static void leds_iicstop(void)
{
	u8 i=0;
	gpio_direction_output(LEDS_SDA,0);
	gpio_direction_output(LEDS_SCL,0);
	gpio_direction_input(LEDS_SDA);
	gpio_direction_input(LEDS_SCL);
	udelay(50);
	while(i++<10)
	{
		if(gpio_get_value(LEDS_SDA)==1)
			break;
		udelay(5);
	}
}

/*
static u8 leds_iicrd(void)
{
	u8 i,j=0;
	gpio_direction_input(LEDS_SDA);
	gpio_direction_output(LEDS_SCL,0);
	for(i=0;i<8;i++)
	{
		gpio_direction_output(LEDS_SCL,1);
		j=(j<<1)|gpio_get_value(LEDS_SDA);
		gpio_direction_output(LEDS_SCL,0);
	}
	gpio_direction_output(LEDS_SDA,1);
	gpio_direction_output(LEDS_SCL,1);
	gpio_direction_output(LEDS_SCL,0);
	return j;
}

static u8 leds_iic_readbyte(u8 i2c_adr,u8 adr)
{
	u8 num1;
	leds_iicstart((i2c_adr<<1)|1);
	leds_iicwr(0x61);
	leds_iicwr(adr);
	num1=leds_iicrd();
	leds_iicstop();
	return num1;
}
*/

void leds_iic_writebyte(u8 i2c_adr,u8 adr,u8 dat)
{
	int ret,i=0;
L_iicwrbyte_again:
	i++;
	leds_iicstart();
	ret = leds_iicwr(i2c_adr<<1);
	if((ret)&&(i<5))
	{
		leds_iicstop();
		goto L_iicwrbyte_again;
	}
	ret = leds_iicwr(adr);
	if((ret)&&(i<5))
	{
		leds_iicstop();
		goto L_iicwrbyte_again;
	}
	ret = leds_iicwr(dat);
	leds_iicstop();
	if((ret)&&(i<5))
		goto L_iicwrbyte_again;
}
#endif 


void ktd2026_init(void)
{
	//MQA led
	unsigned char i2c_addr = 0x30;
	unsigned char TempChar;
	
	TempChar = (3<<5)|(3<<3)|(7<<0);
	leds_iic_writebyte(i2c_addr,0x00,TempChar);
	mdelay(20);
	TempChar = (0<<5)|(3<<3);
	leds_iic_writebyte(i2c_addr,0x00,TempChar);	

	leds_iic_writebyte(i2c_addr,0x06,0x00);	//r
	leds_iic_writebyte(i2c_addr,0x07,0x00);	//g
	leds_iic_writebyte(i2c_addr,0x08,0x00);	//b
	leds_iic_writebyte(i2c_addr,0x04,0x00);

	//Khz Led
	i2c_addr = 0x32;
	
	TempChar = (3<<5)|(3<<3)|(7<<0);
	leds_iic_writebyte(i2c_addr,0x00,TempChar);
	mdelay(20);
	TempChar = (0<<5)|(3<<3);
	leds_iic_writebyte(i2c_addr,0x00,TempChar);	
	
	leds_iic_writebyte(i2c_addr,0x06,0x00);	//r
	leds_iic_writebyte(i2c_addr,0x07,0x00);	//g
	leds_iic_writebyte(i2c_addr,0x08,0x00);	//b
	leds_iic_writebyte(i2c_addr,0x04,0x00);
	
}

void ktd2026_mqa_gpio_control(unsigned char LedType)
{
	unsigned char TempChar;
	unsigned char i2c_addr = 0x30;
	switch(LedType)
	{
		case LED_WHITE:
			leds_iic_writebyte(i2c_addr,0x06,0x38);	//r
			leds_iic_writebyte(i2c_addr,0x07,0x40);	//g
			leds_iic_writebyte(i2c_addr,0x08,0x20);	//b
			//RGB			
			TempChar = (1<<0)|(1<<2)|(1<<4);	
			break;
		
		case LED_BLACK:
			TempChar = 0;	
			break;
		
		case LED_RED:
			leds_iic_writebyte(i2c_addr,0x06,0x20);	//R
			TempChar = (1<<0);	
			break;
		
		case LED_GREEN:
			leds_iic_writebyte(i2c_addr,0x07,0x60);	//G
			TempChar = (1<<2);	
			break;
		
		case LED_BLUE:
			leds_iic_writebyte(i2c_addr,0x08,0x60);	//B
			TempChar = (1<<4);	
			break;
		
		case LED_YELLOW:
			leds_iic_writebyte(i2c_addr,0x06,0x28);	//R
			leds_iic_writebyte(i2c_addr,0x07,0x38);	//G 	
			TempChar = (1<<2)|(1<<0);	
			break;		
		
		case LED_PURPLE:
			leds_iic_writebyte(i2c_addr,0x06,0x28);	//R 		
			leds_iic_writebyte(i2c_addr,0x08,0x38);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		
		case LED_CYAN:
			leds_iic_writebyte(i2c_addr,0x07,0x38);	//G
			leds_iic_writebyte(i2c_addr,0x08,0x40);	//B 	
			TempChar = (1<<2)|(1<<4);	
			break;
		case LED_MQARED:
			leds_iic_writebyte(i2c_addr,0x06,0x1a);	//R 		
			leds_iic_writebyte(i2c_addr,0x08,0x25);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		default:
			TempChar = 0;	
			break;
	}
	leds_iic_writebyte(i2c_addr,0x04,TempChar);
	led_mqa_value = LedType;
}


void ktd2026_khz_gpio_control(unsigned char LedType)
{
	unsigned char TempChar;
	unsigned char i2c_addr = 0x32;
	switch(LedType)
	{
		case LED_WHITE:
			leds_iic_writebyte(i2c_addr,0x06,0x38);	//r
			leds_iic_writebyte(i2c_addr,0x07,0x40);	//g
			leds_iic_writebyte(i2c_addr,0x08,0x20);	//b
			//RGB			
			TempChar = (1<<0)|(1<<2)|(1<<4);	
			break;
		
		case LED_BLACK:
			TempChar = 0;	
			break;
		
		case LED_RED:
			leds_iic_writebyte(i2c_addr,0x06,0x20);	//R
			TempChar = (1<<0);	
			break;
		
		case LED_GREEN:
			leds_iic_writebyte(i2c_addr,0x07,0x60);	//G
			TempChar = (1<<2);	
			break;
		
		case LED_BLUE:
			leds_iic_writebyte(i2c_addr,0x08,0x60);	//B
			TempChar = (1<<4);	
			break;
		
		case LED_YELLOW:
			leds_iic_writebyte(i2c_addr,0x06,0x28);	//R
			leds_iic_writebyte(i2c_addr,0x07,0x38);	//G 	
			TempChar = (1<<2)|(1<<0);	
			break;		
		
		case LED_PURPLE:
			leds_iic_writebyte(i2c_addr,0x06,0x28);	//R 		
			leds_iic_writebyte(i2c_addr,0x08,0x38);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		
		case LED_CYAN:
			leds_iic_writebyte(i2c_addr,0x07,0x38);	//G
			leds_iic_writebyte(i2c_addr,0x08,0x40);	//B 	
			TempChar = (1<<2)|(1<<4);	
			break;
		case LED_MQARED:
			leds_iic_writebyte(i2c_addr,0x06,0x1a);	//R 		
			leds_iic_writebyte(i2c_addr,0x08,0x25);	//B 
			TempChar = (1<<4)|(1<<0);
			break;
		default:
			TempChar = 0;	
			break;
	}
	leds_iic_writebyte(i2c_addr,0x04,TempChar);
	led_khz_value = LedType;
}


static ssize_t led_mqa_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	return sprintf(ubuf,"led_value=%d[0:RED;1:GREEN;2:BLUE;3:YELLOW;4:PURPLE;5:CYAN;6:WHITE;7:BLACK]\n",led_mqa_value);
}
static ssize_t led_mqa_store(struct class *c, struct class_attribute *attr,
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
	if(value >= LED_RED && value <=  LED_MQARED && value != led_mqa_value)
	{
		ktd2026_mqa_gpio_control(value);
	}
	return count;
}


static ssize_t led_khz_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	return sprintf(ubuf,"led_value=%d[0:RED;1:GREEN;2:BLUE;3:YELLOW;4:PURPLE;5:CYAN;6:WHITE;7:BLACK]\n",led_khz_value);
}
static ssize_t led_khz_store(struct class *c, struct class_attribute *attr,
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
	if(value >= LED_RED && value <=  LED_MQARED && value != led_khz_value)
	{
		ktd2026_khz_gpio_control(value);
	}
	return count;
}


static DEVICE_ATTR(led_mqa, 0644, led_mqa_show, led_mqa_store);
static DEVICE_ATTR(led_khz, 0644, led_khz_show, led_khz_store);


static int ktd2026_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	
	printk("========fun[%s]===\n",__FUNCTION__);
	ret = gpio_request(LEDS_SDA,"ktd2026_gpioled_sda");
	gpio_direction_output(LEDS_SDA,1);

	gpio_request(LEDS_SCL,"ktd2026_gpioled_scl");
	gpio_direction_output(LEDS_SCL,1);

	ktd2026_init();
	ret = device_create_file(dev, &dev_attr_led_mqa);
	if(ret != 0)
	{
		dev_err(dev,"Failed to create led_mqa sysfs files: %d\n",ret);
	}
	ret = device_create_file(dev, &dev_attr_led_khz);
	if(ret != 0)
	{
		dev_err(dev,"Failed to create led_khz sysfs files: %d\n",ret);
	}
	return ret;
}

static int ktd2026_led_remove(struct platform_device *pdev)
{
	gpio_free(LEDS_SDA);
	gpio_free(LEDS_SCL);
	return 0;
}



struct platform_driver ktd2026_device_driver = {
	.probe		= ktd2026_led_probe,
	.remove		= ktd2026_led_remove,
	.driver		= {
		.name	= "led_gpioi2c_ktd2026",
	}
};

static int __init ktd2026_gpio_led_init(void)
{
	return platform_driver_register(&ktd2026_device_driver);
}

static void __exit ktd2026_gpio_led_exit(void)
{
	platform_driver_unregister(&ktd2026_device_driver);
}

module_init(ktd2026_gpio_led_init);
module_exit(ktd2026_gpio_led_exit);

MODULE_DESCRIPTION("gpio led ktd2026 driver");
MODULE_LICENSE("GPL v2");
