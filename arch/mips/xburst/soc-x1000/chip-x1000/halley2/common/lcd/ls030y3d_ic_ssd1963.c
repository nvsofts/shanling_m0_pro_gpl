/*
 * Copyright (c) 2014 Engenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * JZ-M200 orion board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>
#include <mach/jzfb.h>
#include "../board_base.h"

/*ifdef is 18bit,6-6-6 ,ifndef default 5-6-6*/
//#define CONFIG_SLCD_TRULY_18BIT

#ifdef	CONFIG_SLCD_TRULY_18BIT
static int slcd_inited = 1;
#else
static int slcd_inited = 0;
#endif

struct sharp_tft480x800_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct sharp_tft480x800_power lcd_power = {
	NULL,
    NULL,
    0
};

int sharp_tft480x800_power_init(struct lcd_device *ld)
{
    int ret ;
    printk("======sharp_tft480x800_power_init==============\n");
    if(GPIO_LCD_RST > 0){
        ret = gpio_request(GPIO_LCD_RST, "lcd rst");
        if (ret) {
            printk(KERN_ERR "can's request lcd rst\n");
            return ret;
        }
    }
    if(GPIO_LCD_CS > 0){
        ret = gpio_request(GPIO_LCD_CS, "lcd cs");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
    }
    if(GPIO_LCD_RD > 0){
        ret = gpio_request(GPIO_LCD_RD, "lcd rd");
        if (ret) {
            printk(KERN_ERR "can's request lcd rd\n");
            return ret;
        }
    }
    printk("set lcd_power.inited  =======1 \n");
    lcd_power.inited = 1;
    return 0;
}

int sharp_tft480x800_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(10);

	return 0;
}

int sharp_tft480x800_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && sharp_tft480x800_power_init(ld))
		return -EFAULT;

	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);

		sharp_tft480x800_power_reset(ld);

		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);

	} else {
		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
		slcd_inited = 0;
	}
	return 0;
}

struct lcd_platform_data sharp_tft480x800_pdata = {
	.reset    = sharp_tft480x800_power_reset,
	.power_on = sharp_tft480x800_power_on,
};

/* LCD Panel Device */
struct platform_device sharp_tft480x800_device = {
	.name		= "sharp_tft480x800_slcd",
	.dev		= {
		.platform_data	= &sharp_tft480x800_pdata,
	},
};
		
#define SMART_CONFIG_SPI_CMD	3
#define SMART_CONFIG_SPI_DATA	4

static struct smart_lcd_data_table sharp_tft480x800_data_table[] = {
    /* LCD init code */
#if 1
/**rgb lcd param 24bit
	*dclk=536x808x60=25.985280M w=480 h=800  
	*hspw=16 hbp=24 hfp=16
	*vspw=2  vsp=2  vfp=4
	*/

	//M = 400M = 0x08M*(0x31+1) ,N = 0x03, pll=M/N=100M
	{SMART_CONFIG_CMD, 0xE2},
	{SMART_CONFIG_DATA, 0x31}, 
	{SMART_CONFIG_DATA, 0x03}, 
	//{SMART_CONFIG_DATA, 0x1f}, 
	//{SMART_CONFIG_DATA, 0x03},  // pll = 64M
	{SMART_CONFIG_UDELAY, 1000},
	//system output
	{SMART_CONFIG_CMD, 0xE0},
	{SMART_CONFIG_DATA, 0x01}, 
	{SMART_CONFIG_UDELAY, 1000},
	{SMART_CONFIG_CMD, 0xE0},
	{SMART_CONFIG_DATA, 0x03}, 
	{SMART_CONFIG_UDELAY, 1000},
	//reset
	{SMART_CONFIG_CMD, 0x01},
	//Configure the dot clock frequency
	//25.985280M
	//1048576
	//LCDC_FPR 0x0428f5        0x02147a
/*For example,
22MHz = 100MHz * (LCDC_FPR+1) / 220
LCDC_FPR = 230686=0x3851D
WRITE COMMAND “0xE6”
WRITE DATA “0x03”
WRITE DATA “0x85”
WRITE DATA “0x1D*/
	{SMART_CONFIG_CMD, 0xE6},
#if 1

	{SMART_CONFIG_DATA, 0x04}, //0x0428f5
	{SMART_CONFIG_DATA, 0x85},  //7f
	{SMART_CONFIG_DATA, 0xff}, //f5
#else
	{SMART_CONFIG_DATA, 0x06}, //0x067f0e
	{SMART_CONFIG_DATA, 0x7f},
	{SMART_CONFIG_DATA, 0x0d}, 
#endif

//Configure the LCD pane
	{SMART_CONFIG_CMD, 0xB0},
	{SMART_CONFIG_DATA, 0x20},//24bit panel tft mode
	{SMART_CONFIG_DATA, 0x00},//24bit panel tft mode
	{SMART_CONFIG_DATA, 0x01}, //horizontal:480=0x01df+1
	{SMART_CONFIG_DATA, 0xdf}, //horizontal:480=0x01df+1
	{SMART_CONFIG_DATA, 0x03}, //vertical:800=0x031f+1
	{SMART_CONFIG_DATA, 0x1f}, //vertical:800=0x031f+1
	{SMART_CONFIG_DATA, 0x00}, //rgb order
	//Set the horizontal period
	{SMART_CONFIG_CMD, 0xB4},
	{SMART_CONFIG_DATA, 0x02}, //536-1=0x0217
	{SMART_CONFIG_DATA, 0x17}, //536-1=0x0217
	{SMART_CONFIG_DATA, 0x00}, //hspw+hbp=16+24=0x28
	{SMART_CONFIG_DATA, 0x28}, //hspw+hbp=16+24=0x28
	{SMART_CONFIG_DATA, 0x0f}, //HPW HPW-1= 16-1 = 0x0f Horizontal Display Period Start Position = 0x0000
	{SMART_CONFIG_DATA, 0x00}, //HPW HPW-1= 16-1 = 0x0f Horizontal Display Period Start Position = 0x0000
	/*
	LPSPP: Horizontal Sync Pulse Subpixel Start Position
	(for serialTFT interface). Dummy value for TFT interface.
	*/
	{SMART_CONFIG_DATA, 0x00}, 
	{SMART_CONFIG_DATA, 0x00}, 
	//Set the vertical period
	{SMART_CONFIG_CMD, 0xB6},
	{SMART_CONFIG_DATA, 0x03}, //808-1=0x0327
	{SMART_CONFIG_DATA, 0x27}, //808-1=0x0327
	{SMART_CONFIG_DATA, 0x00}, //vspw+vbp=2+2=0x04
	{SMART_CONFIG_DATA, 0x04}, //vspw+vbp=2+2=0x04
	{SMART_CONFIG_DATA, 0x01}, //VPW VPW-1= 2-1 = 0x01  Horizontal Display Period Start Position = 0x0000
	{SMART_CONFIG_DATA, 0x00}, //VPW VPW-1= 2-1 = 0x01  Horizontal Display Period Start Position = 0x0000
	{SMART_CONFIG_DATA, 0x00}, 
	//Set the back light control PWM clock frequency(ignorant this)

	/***********************************************************
	*	9. init lcd spi command
	*
	**/
	//9.1 set gpio function
	{SMART_CONFIG_CMD,	0xB8}, //config gpio[3:0] as output
	{SMART_CONFIG_DATA, 0x0f}, 
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD,	0xBA}, //set GPIO[3:0] to high first.
	{SMART_CONFIG_DATA, 0x0f},
	//lcd reset low   bit0 gpio0=0
	{SMART_CONFIG_CMD,	0xBA},
	{SMART_CONFIG_DATA, 0x0e},
	{SMART_CONFIG_UDELAY, 1000},//delay 1ms
	//lcd reset height
	{SMART_CONFIG_CMD,	0xBA},
	{SMART_CONFIG_DATA, 0x0f},
	{SMART_CONFIG_UDELAY, 1000*5},//delay 5ms

	//9.2 lcd param
	{SMART_CONFIG_SPI_CMD, 0x29},	//display on
	{SMART_CONFIG_SPI_CMD, 0x11},	//sleep out
	{SMART_CONFIG_UDELAY, 1000*5},//delay 5ms
	{SMART_CONFIG_SPI_CMD, 0x36},	//addr mode
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0x3a},	//pixel format
	{SMART_CONFIG_SPI_DATA,0x70},	//
	{SMART_CONFIG_SPI_CMD, 0xb0},	//command access enable
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0xb8},	//back enable 00 or 01
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0xb9},	//pwm
	{SMART_CONFIG_SPI_DATA,0x01},	//
	{SMART_CONFIG_SPI_DATA,0xff},	//
	{SMART_CONFIG_SPI_CMD, 0xb0},	//command access enable
	{SMART_CONFIG_SPI_DATA,0x03},	//

	

	//10. Turn on the display
	{SMART_CONFIG_CMD, 0x29},// display on
	//{SMART_CONFIG_CMD, 0x11},// display on
	
	//11. Config the frame buffer
	{SMART_CONFIG_CMD, 0x2A},
	{SMART_CONFIG_DATA, 0x00}, //SC: 0 = 0x0000
	{SMART_CONFIG_DATA, 0x00}, //SC: 0 = 0x0000
	{SMART_CONFIG_DATA, 0x01}, // EC: 480 -1 = 0x01DF
	{SMART_CONFIG_DATA, 0xdf}, // EC: 480 -1 = 0x01DF
	{SMART_CONFIG_CMD, 0x2B},
	{SMART_CONFIG_DATA, 0x00}, //SC: 0 = 0x0000
	{SMART_CONFIG_DATA, 0x00}, //SC: 0 = 0x0000
	{SMART_CONFIG_DATA, 0x03}, // EC: 800 -1 = 0x031f
	{SMART_CONFIG_DATA, 0x1f}, // EC: 800 -1 = 0x031f

	//12. Setup the addressing mode to rotate mod
	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_DATA, 0x00},
	//13. Setup the MCU interface data write
	{SMART_CONFIG_CMD, 0xf0},
	{SMART_CONFIG_DATA, 0x03}, //0x000: 8bit,0x02:16bit(package) 0x03:16-bit(565)  0x05:24 bit
	//14. Start to write the data to frame buffer with command “write_memory_start”
	{SMART_CONFIG_CMD, 0x2c2c},


#else
//9.1 set gpio function
	{SMART_CONFIG_CMD,	0xB8}, //config gpio[3:0] as output
	{SMART_CONFIG_DATA, 0x0f}, 
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD,	0xBA}, //set GPIO[3:0] to high first.
	{SMART_CONFIG_DATA, 0x0f},
	//lcd reset low   bit0 gpio0=0
	{SMART_CONFIG_CMD,	0xBA},
	{SMART_CONFIG_DATA, 0x0e},
	{SMART_CONFIG_UDELAY, 1000},//delay 1ms
	//lcd reset height
	{SMART_CONFIG_CMD,	0xBA},
	{SMART_CONFIG_DATA, 0x0f},
	{SMART_CONFIG_UDELAY, 1000*5},//delay 5ms

	//9.2 lcd param
	{SMART_CONFIG_UDELAY, 1000*120},//delay 120ms
	{SMART_CONFIG_SPI_CMD, 0x36},	//addr mode
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0x3a},	//pixel format
	{SMART_CONFIG_SPI_DATA,0x70},	//
	{SMART_CONFIG_SPI_CMD, 0xb0},	//command access enable
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0xb8},	//back enable 00 or 01
	{SMART_CONFIG_SPI_DATA,0x00},	//
	{SMART_CONFIG_SPI_CMD, 0xb9},	//pwm
	{SMART_CONFIG_SPI_DATA,0x01},	//
	{SMART_CONFIG_SPI_DATA,0xff},	//
	{SMART_CONFIG_SPI_CMD, 0xb0},	//command access enable
	{SMART_CONFIG_SPI_DATA,0x03},	//
	{SMART_CONFIG_UDELAY, 1000*5},//delay 5ms
	{SMART_CONFIG_SPI_CMD, 0x29},	//display on
	{SMART_CONFIG_SPI_CMD, 0x11},	//sleep out

	
#endif
};

unsigned long truly_cmd_buf[]= {
	0x2C2C2C2C,
};

struct fb_videomode jzfb0_videomode = {
	.name = "480x800",
	.refresh = 60,
	#if 1
	.xres = 480,
	.yres = 800,
	#else
	.xres = 800,
	.yres = 480,
	#endif
	.init_pixclock=KHZ2PICOS(10000),
	.pixclock = KHZ2PICOS(46080),    // ԼΪ 1/(240*240*60)     1/3456000
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};


struct jzfb_platform_data jzfb_pdata = {
	.num_modes = 1,
	.modes = &jzfb0_videomode,
	.lcd_type = LCD_TYPE_SLCD,
	.bpp    = 24,
	.width = 31,
	.height = 31,
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	.smart_config.newcfg_fmt_conv =  1,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.write_gram_cmd = truly_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(truly_cmd_buf),
	.smart_config.bus_width = 16,
	.smart_config.length_data_table =  ARRAY_SIZE(sharp_tft480x800_data_table),
	.smart_config.data_table = sharp_tft480x800_data_table,
	.dither_enable = 0,
};
/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM

void fix_new_lcd(void)
{
	unsigned int value=0;
	unsigned int pull_value=0;

	pull_value = *((volatile unsigned int*)(0xb0010300+0x70)) & (1 << 3);
	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x78)) = 1 << 3;

	value = (*((volatile unsigned int *)(0xb0010300+0))) & (1 << 3);
//modify by zxj
#if 0
	if(!value) {
		jzfb_pdata.smart_config.length_data_table = ARRAY_SIZE(new_sharp_tft480x800_data_table);
		jzfb_pdata.smart_config.data_table = new_sharp_tft480x800_data_table;
	}
#endif

	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x74)) = 1 << 3;

}

static int backlight_init(struct device *dev)
{
	int ret;
#if 0
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}
#endif
	printk("------------------ %s --------------------\n", __func__);
	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);
	return 0;
}

static int backlight_notify(struct device *dev, int brightness)
{
	if (brightness)
		gpio_direction_output(GPIO_BL_PWR_EN, 1);
	else
		gpio_direction_output(GPIO_BL_PWR_EN, 0);

	return brightness;
}

static void backlight_exit(struct device *dev)
{
	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 120,
	.pwm_period_ns	= 30000,
	.init		= backlight_init,
	.exit		= backlight_exit,
	.notify		= backlight_notify,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif
