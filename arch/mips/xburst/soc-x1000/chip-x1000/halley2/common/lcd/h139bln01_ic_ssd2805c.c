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

struct rm67162_ic_ssd2805c_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct rm67162_ic_ssd2805c_power lcd_power = {
	NULL,
	NULL,
	0
};

int rm67162_ic_ssd2805c_power_init(struct lcd_device *ld)
{
    int ret ;
    printk("======rm67162_ic_ssd2805c_power_init============== start \n");
    if(GPIO_LCD_CLOCK > 0){
        ret = gpio_request(GPIO_LCD_CLOCK, "lcd clock");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
		jz_gpio_set_func(GPIO_LCD_CLOCK, GPIO_CLOCK_FUNC);
		mdelay(10);
    }

    if(GPIO_LCD_CS > 0){
        ret = gpio_request(GPIO_LCD_CS, "lcd cs");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
    }

    if(GPIO_LCD_RST > 0){
        ret = gpio_request(GPIO_LCD_RST, "lcd rst");
        if (ret) {
            printk(KERN_ERR "can's request lcd rst\n");
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

    lcd_power.inited = 1;
	
    printk("======rm67162_ic_ssd2805c_power_init============== end \n");
    return 0;
}

int rm67162_ic_ssd2805c_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	#if 1
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(200);
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(800);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(800);
	printk("lcd device reset ok add by zxj\n");
	#endif
	return 0;
}

int rm67162_ic_ssd2805c_power_on(struct lcd_device *ld, int enable)
{

	printk("arch... h139bln01_ic_ssd2805c.c func:%s\n", __func__);
	if (!lcd_power.inited && rm67162_ic_ssd2805c_power_init(ld))
		return -EFAULT;
	//add by zxj
	gpio_request(GPIO_LCD_PWR_EN, "lcd pwren");
	udelay(10);
	printk("power_on 1.1\n");
	gpio_direction_output(GPIO_LCD_PWR_EN, 1);	//lcd power enable
	printk("power_on 1\n");
	
	gpio_request(GPIO_TP_PWR_EN, "tp pwren");
	udelay(10);
	gpio_direction_output(GPIO_TP_PWR_EN, 1);	//tp power enable
	printk("power_on 2\n");
	
	udelay(10);
	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		printk("power_on 2.1\n");
		gpio_direction_output(GPIO_LCD_RD, 1);
		printk("power_on 2.2\n");
		rm67162_ic_ssd2805c_power_reset(ld);
		mdelay(5);
		printk("power_on 2.3\n");
		gpio_direction_output(GPIO_LCD_CS, 0);
		printk("power_on 2.4\n");

	} else {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);
		gpio_direction_output(GPIO_LCD_RST, 0);
	}
	printk("power_on 3\n");
	return 0;
}

struct lcd_platform_data rm67162_ic_ssd2805c_pdata = {
	.reset    = rm67162_ic_ssd2805c_power_reset,
	.power_on = rm67162_ic_ssd2805c_power_on,
};

struct platform_device rm67162_ic_ssd2805c_device = {
	.name		= "rm67162_ic_ssd2805c",
	.dev		= {
		.platform_data	= &rm67162_ic_ssd2805c_pdata,
	},
};

static struct smart_lcd_data_table rm67162_ic_ssd2805c_data_table[] = {
	
#if 0
	{SMART_CONFIG_UDELAY, 1000*1000},	  /* sleep out 5 ms  */
	{SMART_CONFIG_CMD, 0X01},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X02},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X04},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X08},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X10},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X20},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X40},//cmd 0X00BE
	{SMART_CONFIG_UDELAY, 1000*1000},
	{SMART_CONFIG_CMD, 0X80},//cmd 0X00BE
#endif		

#if 1
	//{SMART_CONFIG_UDELAY, 1000},
	{SMART_CONFIG_CMD, 0X00},//cmd 0X00BE
	{SMART_CONFIG_CMD, 0XBE},	
	{SMART_CONFIG_DATA, 0xC3},//data 0XC329
	{SMART_CONFIG_DATA, 0x29}, 

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0001
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0004 
    {SMART_CONFIG_DATA, 0x04}, 

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0002
	{SMART_CONFIG_CMD, 0X02},	
	{SMART_CONFIG_DATA, 0x01},//data 0X0100
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0003
	{SMART_CONFIG_CMD, 0X03},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0030
    {SMART_CONFIG_DATA, 0x30},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0005
	{SMART_CONFIG_CMD, 0X05},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0006
	{SMART_CONFIG_CMD, 0X06},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0007
	{SMART_CONFIG_CMD, 0X07},	
	{SMART_CONFIG_DATA, 0x01},//data 0X0103
    {SMART_CONFIG_DATA, 0x03},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0008
	{SMART_CONFIG_CMD, 0X08},	
	{SMART_CONFIG_DATA, 0x03},//data 0X0303
    {SMART_CONFIG_DATA, 0x03},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X000D
	{SMART_CONFIG_CMD, 0X0D},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

//////////////////////////////////////////////
	{SMART_CONFIG_CMD, 0X00},//cmd 0X0010
	{SMART_CONFIG_CMD, 0X10},	
	{SMART_CONFIG_DATA, 0x00},//data 0X00C1
    {SMART_CONFIG_DATA, 0xc1},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0011
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0xB1},//data 0XB108
    {SMART_CONFIG_DATA, 0x08},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0012
	{SMART_CONFIG_CMD, 0X12},	
	{SMART_CONFIG_DATA, 0xB1},//data 0XB108
    {SMART_CONFIG_DATA, 0x08},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0013
	{SMART_CONFIG_CMD, 0X13},	
	{SMART_CONFIG_DATA, 0x00},//data 0X000F
    {SMART_CONFIG_DATA, 0x0F},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0014
	{SMART_CONFIG_CMD, 0X14},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0014
    {SMART_CONFIG_DATA, 0x14},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0015
	{SMART_CONFIG_CMD, 0X15},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0004
    {SMART_CONFIG_DATA, 0x04},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0016
	{SMART_CONFIG_CMD, 0X16},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0022
	{SMART_CONFIG_CMD, 0X22},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0023
	{SMART_CONFIG_CMD, 0X23},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0030
	{SMART_CONFIG_CMD, 0X30},	
	{SMART_CONFIG_DATA, 0x7C},//data 0X7C3F
    {SMART_CONFIG_DATA, 0x3F},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0032
	{SMART_CONFIG_CMD, 0X32},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0070
	{SMART_CONFIG_CMD, 0X70},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
    {SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X0091
	{SMART_CONFIG_CMD, 0X91},	
	{SMART_CONFIG_DATA, 0x01},//data 0X0100
    {SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X00E0
	{SMART_CONFIG_CMD, 0XE0},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
    {SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X00},//cmd 0X00E1
	{SMART_CONFIG_CMD, 0XE1},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0061
    {SMART_CONFIG_DATA, 0x61},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0100
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x10},//data 0X1030
    {SMART_CONFIG_DATA, 0x30},
    
	{SMART_CONFIG_CMD, 0X01},//cmd 0X0101
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0xc6},//data 0XC633
	{SMART_CONFIG_DATA, 0x33},
		
	{SMART_CONFIG_CMD, 0X01},//cmd 0X0102
	{SMART_CONFIG_CMD, 0X02},	
	{SMART_CONFIG_DATA, 0x50},//data 0X501F
    {SMART_CONFIG_DATA, 0x1f},

	
	{SMART_CONFIG_CMD, 0X01},//cmd 0X0103
	{SMART_CONFIG_CMD, 0X03},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0025
	{SMART_CONFIG_DATA, 0x25},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0108
	{SMART_CONFIG_CMD, 0X08},	
	{SMART_CONFIG_DATA, 0x03},//data 0X0360
	{SMART_CONFIG_DATA, 0x60},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0111
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0135
	{SMART_CONFIG_CMD, 0X35},	
	{SMART_CONFIG_DATA, 0x76},//data 0X7666
	{SMART_CONFIG_DATA, 0x66},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0139
	{SMART_CONFIG_CMD, 0X39},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0026
	{SMART_CONFIG_DATA, 0x26},

	{SMART_CONFIG_CMD, 0X04},//cmd 0X0400
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x00},//data 0X00C7
	{SMART_CONFIG_DATA, 0xc7},

	{SMART_CONFIG_CMD, 0X04},//cmd 0X0401
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X06},//cmd 0X0606
	{SMART_CONFIG_CMD, 0X06},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},
/*************gamma**************/
	{SMART_CONFIG_CMD, 0X03},//cmd 0X0300
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x0d},//data 0X0D0E
	{SMART_CONFIG_DATA, 0x0e},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0301
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0003
	{SMART_CONFIG_DATA, 0x03},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0302
	{SMART_CONFIG_CMD, 0X02},	
	{SMART_CONFIG_DATA, 0x08},//data 0X0808
	{SMART_CONFIG_DATA, 0x08},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0303
	{SMART_CONFIG_CMD, 0X03},	
	{SMART_CONFIG_DATA, 0x02},//data 0X0201
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0304
	{SMART_CONFIG_CMD, 0X04},	
	{SMART_CONFIG_DATA, 0x03},//data 0X0301
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0305
	{SMART_CONFIG_CMD, 0X05},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0004
	{SMART_CONFIG_DATA, 0x04},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0306
	{SMART_CONFIG_CMD, 0X06},	
	{SMART_CONFIG_DATA, 0x1b},//data 0X1B21
	{SMART_CONFIG_DATA, 0x21},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0307
	{SMART_CONFIG_CMD, 0X07},	
	{SMART_CONFIG_DATA, 0x0f},//data 0X0F0E
	{SMART_CONFIG_DATA, 0x0e},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0308
	{SMART_CONFIG_CMD, 0X08},	
	{SMART_CONFIG_DATA, 0x01},//data 0X0104
	{SMART_CONFIG_DATA, 0x04},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X0309
	{SMART_CONFIG_CMD, 0X09},	
	{SMART_CONFIG_DATA, 0x08},//data 0X0808
	{SMART_CONFIG_DATA, 0x08},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X030A
	{SMART_CONFIG_CMD, 0X0a},	
	{SMART_CONFIG_DATA, 0x02},//data 0X0201
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X030B
	{SMART_CONFIG_CMD, 0X0b},	
	{SMART_CONFIG_DATA, 0x03},//data 0X0301
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X030C
	{SMART_CONFIG_CMD, 0X0c},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0003
	{SMART_CONFIG_DATA, 0x03},

	{SMART_CONFIG_CMD, 0X03},//cmd 0X030D
	{SMART_CONFIG_CMD, 0X0d},	
	{SMART_CONFIG_DATA, 0x31},//data 0X3134
	{SMART_CONFIG_DATA, 0x34},

	////////////////////////////////////////////
	{SMART_CONFIG_CMD, 0X05},//cmd 0X0581
	{SMART_CONFIG_CMD, 0X81},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0006
	{SMART_CONFIG_DATA, 0x06},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0583
	{SMART_CONFIG_CMD, 0X83},	
	{SMART_CONFIG_DATA, 0x00},//data 0X003F
	{SMART_CONFIG_DATA, 0x3f},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0585
	{SMART_CONFIG_CMD, 0X85},	
	{SMART_CONFIG_DATA, 0x00},//data 0X003A
	{SMART_CONFIG_DATA, 0x3a},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0589
	{SMART_CONFIG_CMD, 0X89},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0090
	{SMART_CONFIG_DATA, 0x90},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0582
	{SMART_CONFIG_CMD, 0X82},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0584
	{SMART_CONFIG_CMD, 0X84},	
	{SMART_CONFIG_DATA, 0x00},//data 0X003F
	{SMART_CONFIG_DATA, 0x3f},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0586
	{SMART_CONFIG_CMD, 0X86},	
	{SMART_CONFIG_DATA, 0x00},//data 0X003F
	{SMART_CONFIG_DATA, 0x3f},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X058A
	{SMART_CONFIG_CMD, 0X8a},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0010
	{SMART_CONFIG_DATA, 0x10},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0587
	{SMART_CONFIG_CMD, 0X87},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X05},//cmd 0X0588
	{SMART_CONFIG_CMD, 0X88},	
	{SMART_CONFIG_DATA, 0x00},//data 0X001F
	{SMART_CONFIG_DATA, 0x1f},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0590
	{SMART_CONFIG_CMD, 0X90},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0591
	{SMART_CONFIG_CMD, 0X91},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0592
	{SMART_CONFIG_CMD, 0X92},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0593
	{SMART_CONFIG_CMD, 0X93},	
	{SMART_CONFIG_DATA, 0x00},//data 0X001D
	{SMART_CONFIG_DATA, 0x1d},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0594
	{SMART_CONFIG_CMD, 0X94},	
	{SMART_CONFIG_DATA, 0x00},//data 0X002A
	{SMART_CONFIG_DATA, 0x2a},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0595
	{SMART_CONFIG_CMD, 0X95},	
	{SMART_CONFIG_DATA, 0x00},//data 0X004D
	{SMART_CONFIG_DATA, 0x4d},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0596
	{SMART_CONFIG_CMD, 0X96},	
	{SMART_CONFIG_DATA, 0x00},//data 0X007A
	{SMART_CONFIG_DATA, 0x7a},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0597
	{SMART_CONFIG_CMD, 0X97},	
	{SMART_CONFIG_DATA, 0x00},//data 0X00B1
	{SMART_CONFIG_DATA, 0xb1},
	
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0598
	{SMART_CONFIG_CMD, 0X98},	
	{SMART_CONFIG_DATA, 0x00},//data 0X00F2
	{SMART_CONFIG_DATA, 0xf2},
	
    /////////////////////////

	{SMART_CONFIG_CMD, 0X05},//cmd 0X05A0
	{SMART_CONFIG_CMD, 0Xa0},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},
		
    {SMART_CONFIG_CMD, 0X05},//cmd 0X05A1
	{SMART_CONFIG_CMD, 0Xa1},	
	{SMART_CONFIG_DATA, 0x00},//data 0X00FF
	{SMART_CONFIG_DATA, 0xff},
		
    {SMART_CONFIG_CMD, 0X05},//cmd 0X05A2
	{SMART_CONFIG_CMD, 0Xa2},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},
		
    {SMART_CONFIG_CMD, 0X05},//cmd 0X05A3
	{SMART_CONFIG_CMD, 0Xa3},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0018
	{SMART_CONFIG_DATA, 0x18},
		
    {SMART_CONFIG_CMD, 0X05},//cmd 0X0580
	{SMART_CONFIG_CMD, 0X80},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0003
	{SMART_CONFIG_DATA, 0x03},
		//??20140124
		
    {SMART_CONFIG_CMD, 0X00},//cmd 0X0000
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

#if 1
	// xoff yoff x y
	{SMART_CONFIG_CMD, 0X02},//cmd 0X0210
	{SMART_CONFIG_CMD, 0X10},	
	{SMART_CONFIG_DATA, 0x00},//data 0X001E
	{SMART_CONFIG_DATA, 0x1e},

	{SMART_CONFIG_CMD, 0X02},//cmd 0X0211
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0x01},//data 0X0185
	{SMART_CONFIG_DATA, 0xdf},

	{SMART_CONFIG_CMD, 0X02},//cmd 0X0212
	{SMART_CONFIG_CMD, 0X12},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X02},//cmd 0X0213
	{SMART_CONFIG_CMD, 0X13},	
	{SMART_CONFIG_DATA, 0x01},//data 0X018F
	{SMART_CONFIG_DATA, 0x67},

	{SMART_CONFIG_CMD, 0X02},//cmd 0X0200
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x00},//data 0X001E
	{SMART_CONFIG_DATA, 0x1e},

	{SMART_CONFIG_CMD, 0X02},//cmd 0X0201
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0X01},//cmd 0X0111
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0001
	{SMART_CONFIG_DATA, 0x01},

	{SMART_CONFIG_CMD, 0X02},//cmd 0x0202
	{SMART_CONFIG_CMD, 0X02},
	#endif
/***************startx 0 starty 0  xend 399 yend 399****************/
	//xtart 0
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0210
	{SMART_CONFIG_CMD, 0X10},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	#if 0
	//use 360x480
		
	//xend 360
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0211
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0x01},//data 360
	{SMART_CONFIG_DATA, 0x68},

	//ystart 0
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0212
	{SMART_CONFIG_CMD, 0X12},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	//yend 480
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0213
	{SMART_CONFIG_CMD, 0X13},	
	{SMART_CONFIG_DATA, 0x01},//data 480
	{SMART_CONFIG_DATA, 0xe0},
	#else
	//use 480x360  479x359
	//xend 480
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0211
	{SMART_CONFIG_CMD, 0X11},	
	{SMART_CONFIG_DATA, 0x01},//data 480 1df   truly 389 0x185 
	{SMART_CONFIG_DATA, 0x85},

	//ystart 0
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0212
	{SMART_CONFIG_CMD, 0X12},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	//yend 360
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0213
	{SMART_CONFIG_CMD, 0X13},	
	{SMART_CONFIG_DATA, 0x01},//data 360 0x167 truly 399 0x18f
	{SMART_CONFIG_DATA, 0x8f},  

	#endif
	

	//xstart
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0200
	{SMART_CONFIG_CMD, 0X00},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},

	//ystart
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0201
	{SMART_CONFIG_CMD, 0X01},	
	{SMART_CONFIG_DATA, 0x00},//data 0X0000
	{SMART_CONFIG_DATA, 0x00},
		
	{SMART_CONFIG_CMD, 0X02},//cmd 0x0202
	{SMART_CONFIG_CMD, 0X02},			
	#endif


#if 0
	//width height startX startY
	WriteComm(0x0210);   WriteData(Xstart);//0
	WriteComm(0x0211);   WriteData(Xend);//399
	WriteComm(0x0212);   WriteData(Ystart);//0
	WriteComm(0x0213);   WriteData(Yend);//399

	WriteComm(0x0200);   WriteData(Xstart);//0
	WriteComm(0x0201);   WriteData(Ystart);//0

	WriteComm(0x0202);
#endif
};


unsigned long rm67162_ic_ssd2805c_cmd_buf[]= {
//	0x2c2c2c2c,
	0x02020202,
};

struct fb_videomode jzfb0_videomode = {
	.name = "480x360",
	.refresh = 60,
#if 0
	.xres = 360,
	.yres = 480,
#else
	.xres = 390,
	.yres = 400,
#endif
#if 1
	//.init_pixclock = KHZ2PICOS(6000),
	.init_pixclock = KHZ2PICOS(50000), 
	.pixclock = KHZ2PICOS(50000), 	//50000
	//.pixclock = KHZ2PICOS(6000), 	//
	//.pixclock = KHZ2PICOS(3200), 
	#else
	.init_pixclock = KHZ2PICOS(10000),
	.pixclock = KHZ2PICOS(9600), //400*400*60=9600*1000=9600K
#endif
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
	.bpp    = 16,
	#if 0
	.width = 31,
	.height = 31,
	#else
	.width=0,
	.height=0,
	#endif
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	.smart_config.newcfg_fmt_conv =  1,
	.smart_config.write_gram_cmd = rm67162_ic_ssd2805c_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(rm67162_ic_ssd2805c_cmd_buf),
	.smart_config.bus_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(rm67162_ic_ssd2805c_data_table),
	.smart_config.data_table = rm67162_ic_ssd2805c_data_table,
	.dither_enable = 0,
};


void fix_new_lcd(void)
{
	unsigned int value=0;
	unsigned int pull_value=0;

	pull_value = *((volatile unsigned int*)(0xb0010300+0x70)) & (1 << 3);
	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x78)) = 1 << 3;

	value = (*((volatile unsigned int *)(0xb0010300+0))) & (1 << 3);
	if(!value) {
		//jzfb_pdata.smart_config.length_data_table = ARRAY_SIZE(new_truly_tft240240_data_table);
		//jzfb_pdata.smart_config.data_table = new_truly_tft240240_data_table;
		;
	}

	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x74)) = 1 << 3;

}

