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
	#if 0
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(50);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(120);
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
	//设置ssd2805 pll clk sys_clk等设置后可以用示波器抓sys_clk这个时钟是否变为自己设置的
	//0xbb[7:6] : SYS_CLK = TX_CLK

	//1.ssd2805 param init
	//disable pll
	{SMART_CONFIG_CMD, 0xb9},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_UDELAY, 2000}, //delay_ms(2)
	//set pll 24.576M * 0x0e = 344M
	{SMART_CONFIG_CMD, 0xba},
	{SMART_CONFIG_DATA, 0x0e},
	{SMART_CONFIG_DATA, 0x00},
	//enable pll
	{SMART_CONFIG_CMD, 0xb9},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_UDELAY, 2000}, //delay_ms(2)
	
	//set sys_clk lp    lp=pll/(8 * 0x09+1) = 344/8/10 = 4   sys_clk=24.576/2=12.2
	{SMART_CONFIG_CMD, 0xbb},
	{SMART_CONFIG_DATA, 0x49},
	{SMART_CONFIG_DATA, 0x00},
	//set enable output sys_clk for test 
	#if 1
	{SMART_CONFIG_CMD, 0xd6},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
	#endif
	
	//set vc 虚拟通道
	{SMART_CONFIG_CMD, 0xb8},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},

	//2 mipi lcd init
	// use dcs package for mipi
	{SMART_CONFIG_CMD, 0xb7},
	{SMART_CONFIG_DATA, 0x43},
	{SMART_CONFIG_DATA, 0x02},

	//set package size
	{SMART_CONFIG_CMD, 0xbc},	
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},


	//width 400 height 400
	{SMART_CONFIG_CMD, 0x2a},	//0x2a
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x90}, 
	{SMART_CONFIG_CMD, 0x2b},	//0x2b
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x90},

	//set package size
	{SMART_CONFIG_CMD, 0xbc},	
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x36},	//0x36
	{SMART_CONFIG_DATA, 0x00},	
	{SMART_CONFIG_CMD, 0x3a},
	{SMART_CONFIG_DATA, 0x77},	//0x3a


#if 0
	7 W 0x15 FE FE00 07
	8 W 0x15 07 07A0 4F
	9 W 0x15 FE FE00 0A
	10 W 0x15 1C 1CD0 1B
	11 W 0x15 FE FE00 00
	12 W 0x15 35 3500 00
	13 Sleep out W 0x05 11 1100 00
	14 Turn on peripheral packet 0x32 Video Turn On
	15 Delay 300 ms
	16 Display on W 0x05 29 2900 00
#endif
	//set package size
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x1},
	{SMART_CONFIG_DATA, 0x0},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x0},
	{SMART_CONFIG_DATA, 0x0},

	//lcd power on sequence
	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x07},
	{SMART_CONFIG_CMD, 0x07},
	{SMART_CONFIG_DATA, 0x4f},
	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x0a},
	{SMART_CONFIG_CMD, 0x1c},
	{SMART_CONFIG_DATA, 0x1b},
	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x35},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_UDELAY, (1000*300)}, //delay_ms(2)
	{SMART_CONFIG_CMD, 0x29},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_UDELAY, (1000)}, //delay_ms(2)
	// set backlight
	{SMART_CONFIG_CMD, 0x51},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x39},
	{SMART_CONFIG_DATA, 0x00},



#if 0

	//package size 400*400*3
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x53},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x07},
	{SMART_CONFIG_DATA, 0x00},

	//800
	{SMART_CONFIG_CMD, 0xbe},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x2c},

#endif

	#if 0
	//generic  设置mpip包格式，传送给mipi屏参数
	{SMART_CONFIG_CMD, 0xb7},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_DATA, 0x02},
	//设置透传mipi包大小3byte
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x3},
	{SMART_CONFIG_DATA, 0x0},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x0},
	{SMART_CONFIG_DATA, 0x0},
	//透传数据来自mipi屏
	//0x15 FE00 07  数据类型0x15 地址0xfe00 值0x07
	{SMART_CONFIG_CMD, 0xBF},
	{SMART_CONFIG_DATA, 0x15}, //15	
	{SMART_CONFIG_CMD, 0xBF},  
	{SMART_CONFIG_DATA, 0x00},	//00
	{SMART_CONFIG_CMD, 0xBF},
	{SMART_CONFIG_DATA, 0xfe}, //fe	
	{SMART_CONFIG_CMD, 0xBF},
	{SMART_CONFIG_DATA, 0x07}, //07

	 //目前问题 bf后面只能跟一个字节数据值吗？规格书上描述8位宽度的是这样
	 //怎么发送一个指定mipi数据类型参数的如上面 0x15类型
#endif


// test 8080 16bit
#if 0
	{SMART_CONFIG_CMD, 0xb9},
	{SMART_CONFIG_DATA, 0x0},

	{SMART_CONFIG_CMD, 0xde},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xb7},
	{SMART_CONFIG_DATA, 0x0142},

	{SMART_CONFIG_CMD, 0xba},
	{SMART_CONFIG_DATA, 0x000a},

	{SMART_CONFIG_CMD, 0xbb},
	{SMART_CONFIG_DATA, 0x0040},

	{SMART_CONFIG_CMD, 0xb9},
	{SMART_CONFIG_DATA, 0x0001},

	{SMART_CONFIG_CMD, 0xb8},
	{SMART_CONFIG_DATA, 0x0},

	{SMART_CONFIG_CMD, 0xbb},
	{SMART_CONFIG_DATA, 0x0004},

	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x1},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x0},


	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x06},
	{SMART_CONFIG_DATA, 0x62},
	{SMART_CONFIG_CMD, 0x0a},
	{SMART_CONFIG_DATA, 0xe8},
	{SMART_CONFIG_CMD, 0x0e},
	{SMART_CONFIG_DATA, 0x80},
	{SMART_CONFIG_CMD, 0x0f},
	{SMART_CONFIG_DATA, 0x80},
	{SMART_CONFIG_CMD, 0x10},
	{SMART_CONFIG_DATA, 0x71},
	{SMART_CONFIG_CMD, 0x13},
	{SMART_CONFIG_DATA, 0x81},
	{SMART_CONFIG_CMD, 0x14},
	{SMART_CONFIG_DATA, 0x81},
	{SMART_CONFIG_CMD, 0x15},
	{SMART_CONFIG_DATA, 0x82},
	{SMART_CONFIG_CMD, 0x16},
	{SMART_CONFIG_DATA, 0x82},
	{SMART_CONFIG_CMD, 0x18},
	{SMART_CONFIG_DATA, 0x88},
	{SMART_CONFIG_CMD, 0x19},
	{SMART_CONFIG_DATA, 0x55},
	{SMART_CONFIG_CMD, 0x1a},
	{SMART_CONFIG_DATA, 0x10},
	{SMART_CONFIG_CMD, 0x1c},
	{SMART_CONFIG_DATA, 0x99},
	{SMART_CONFIG_CMD, 0x1d},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x1e},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x1f},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x20},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x25},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x26},
	{SMART_CONFIG_DATA, 0x8d},
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0x8d},
	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x37},
	{SMART_CONFIG_DATA, 0x10},
	{SMART_CONFIG_CMD, 0x3a},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x3b},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x3d},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_CMD, 0x3f},
	{SMART_CONFIG_DATA, 0x3a},
	{SMART_CONFIG_CMD, 0x40},
	{SMART_CONFIG_DATA, 0x30},
	{SMART_CONFIG_CMD, 0x41},
	{SMART_CONFIG_DATA, 0x30},
	{SMART_CONFIG_CMD, 0x42},
	{SMART_CONFIG_DATA, 0x33},
	{SMART_CONFIG_CMD, 0x43},
	{SMART_CONFIG_DATA, 0x22},
	{SMART_CONFIG_CMD, 0x44},
	{SMART_CONFIG_DATA, 0x11},
	{SMART_CONFIG_CMD, 0x45},
	{SMART_CONFIG_DATA, 0x66},
	{SMART_CONFIG_CMD, 0x46},
	{SMART_CONFIG_DATA, 0x55},
	{SMART_CONFIG_CMD, 0x47},
	{SMART_CONFIG_DATA, 0x44},
	{SMART_CONFIG_CMD, 0x4c},
	{SMART_CONFIG_DATA, 0x33},
	{SMART_CONFIG_CMD, 0x4d},
	{SMART_CONFIG_DATA, 0x22},
	{SMART_CONFIG_CMD, 0x4e},
	{SMART_CONFIG_DATA, 0x11},
	{SMART_CONFIG_CMD, 0x4f},
	{SMART_CONFIG_DATA, 0x66},
	{SMART_CONFIG_CMD, 0x50},
	{SMART_CONFIG_DATA, 0x55},
	{SMART_CONFIG_CMD, 0x51},
	{SMART_CONFIG_DATA, 0x44},
	{SMART_CONFIG_CMD, 0x57},
	{SMART_CONFIG_DATA, 0xb3},
	{SMART_CONFIG_CMD, 0x6b},
	{SMART_CONFIG_DATA, 0x19},
	{SMART_CONFIG_CMD, 0x70},
	{SMART_CONFIG_DATA, 0x55},
	{SMART_CONFIG_CMD, 0x74},
	{SMART_CONFIG_DATA, 0x0c},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x9b},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_CMD, 0x9c},
	{SMART_CONFIG_DATA, 0x67},
	{SMART_CONFIG_CMD, 0x9d},
	{SMART_CONFIG_DATA, 0x20},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x03},
	{SMART_CONFIG_CMD, 0x9b},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_CMD, 0x9c},
	{SMART_CONFIG_DATA, 0x67},
	{SMART_CONFIG_CMD, 0x9d},
	{SMART_CONFIG_DATA, 0x20},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x5d},
	{SMART_CONFIG_DATA, 0x10},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x00},
	{SMART_CONFIG_DATA, 0x8d},
	{SMART_CONFIG_CMD, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x02},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x03},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x04},
	{SMART_CONFIG_DATA, 0x10},
	{SMART_CONFIG_CMD, 0x05},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x06},
	{SMART_CONFIG_DATA, 0xa7},
	{SMART_CONFIG_CMD, 0x07},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_CMD, 0x08},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x09},
	{SMART_CONFIG_DATA, 0xc2},
	{SMART_CONFIG_CMD, 0x0a},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x0b},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x0c},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x0d},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_CMD, 0x0e},
	{SMART_CONFIG_DATA, 0x06},
	{SMART_CONFIG_CMD, 0x0f},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x10},
	{SMART_CONFIG_DATA, 0xa7},
	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x12},
	{SMART_CONFIG_DATA, 0xc2},
	{SMART_CONFIG_CMD, 0x13},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x14},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x15},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x16},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_CMD, 0x17},
	{SMART_CONFIG_DATA, 0x07},
	{SMART_CONFIG_CMD, 0x18},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x19},
	{SMART_CONFIG_DATA, 0xa7},
	{SMART_CONFIG_CMD, 0x1a},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x1b},
	{SMART_CONFIG_DATA, 0x82},
	{SMART_CONFIG_CMD, 0x1c},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x1d},
	{SMART_CONFIG_DATA, 0xff},
	{SMART_CONFIG_CMD, 0x1e},
	{SMART_CONFIG_DATA, 0x05},
	{SMART_CONFIG_CMD, 0x1f},
	{SMART_CONFIG_DATA, 0x60},
	{SMART_CONFIG_CMD, 0x20},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x21},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_CMD, 0x22},
	{SMART_CONFIG_DATA, 0x7c},
	{SMART_CONFIG_CMD, 0x23},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x24},
	{SMART_CONFIG_DATA, 0xc2},
	{SMART_CONFIG_CMD, 0x25},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x26},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x27},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x28},
	{SMART_CONFIG_DATA, 0x70},
	{SMART_CONFIG_CMD, 0x29},
	{SMART_CONFIG_DATA, 0x05},
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0x74},
	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0x8d},
	{SMART_CONFIG_CMD, 0x2d},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x2f},
	{SMART_CONFIG_DATA, 0xc2},
	{SMART_CONFIG_CMD, 0x30},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x31},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x32},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_CMD, 0x33},
	{SMART_CONFIG_DATA, 0x70},
	{SMART_CONFIG_CMD, 0x34},
	{SMART_CONFIG_DATA, 0x07},
	{SMART_CONFIG_CMD, 0x35},
	{SMART_CONFIG_DATA, 0x74},
	{SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_DATA, 0x8d},
	{SMART_CONFIG_CMD, 0x37},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x5e},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_CMD, 0x5f},
	{SMART_CONFIG_DATA, 0x31},
	{SMART_CONFIG_CMD, 0x60},
	{SMART_CONFIG_DATA, 0x54},
	{SMART_CONFIG_CMD, 0x61},
	{SMART_CONFIG_DATA, 0x76},
	{SMART_CONFIG_CMD, 0x62},
	{SMART_CONFIG_DATA, 0x98},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x05},
	{SMART_CONFIG_CMD, 0x05},
	{SMART_CONFIG_DATA, 0x17},
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x91},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xfe},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x35},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0x0600},
	{SMART_CONFIG_DATA, 0x8b01},
	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0x0000},
	{SMART_CONFIG_DATA, 0x8501},

	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x11},
	{SMART_CONFIG_UDELAY, 200000},
	{SMART_CONFIG_CMD, 0x29},

	{SMART_CONFIG_UDELAY, 10000},
	{SMART_CONFIG_CMD, 0xb7},
	{SMART_CONFIG_DATA, 0x0143},
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0xff6c},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x06},
	{SMART_CONFIG_CMD, 0xbe},
	{SMART_CONFIG_DATA, 0x030c},
	{SMART_CONFIG_CMD, 0x2c},

#endif

};


unsigned long rm67162_ic_ssd2805c_cmd_buf[]= {
	0x2c2c2c2c,
};

struct fb_videomode jzfb0_videomode = {
	.name = "400x400",
	.refresh = 60,
	.xres = 400,
	.yres = 400,
#if 1
	.init_pixclock = KHZ2PICOS(10000),
	.pixclock = KHZ2PICOS(50000), 
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
	.bpp    = 24,
	.width = 31,
	.height = 31,
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
