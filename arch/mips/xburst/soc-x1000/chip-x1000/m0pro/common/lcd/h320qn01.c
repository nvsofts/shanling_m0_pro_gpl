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
    printk("======h320qn01_power_init==============\n");
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
	if(GPIO_LCD_PWR_EN > 0){
		ret = gpio_request(GPIO_LCD_PWR_EN, "lcd_pwr_en");
		if(ret)
			printk(KERN_ERR "can's request lcd_pwr_en\n");
	}
	if(GPIO_LCD_PWR_EN_1 > 0){
		ret = gpio_request(GPIO_LCD_PWR_EN_1, "lcd_pwr_en_1");
		if(ret)
			printk(KERN_ERR "can's request lcd_pwr_en_1\n");
	}
    lcd_power.inited = 1;
    return 0;
}

int rm67162_ic_ssd2805c_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST,1);
	mdelay(5);
	gpio_direction_output(GPIO_LCD_RST,0);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST,1);
	return 0;
}

int rm67162_ic_ssd2805c_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && rm67162_ic_ssd2805c_power_init(ld))
		return -EFAULT;
	
    gpio_direction_output(GPIO_LCD_PWR_EN_1, 1);
   	gpio_direction_output(GPIO_LCD_PWR_EN, 1);
	
	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);
		rm67162_ic_ssd2805c_power_reset(ld);
		gpio_direction_output(GPIO_LCD_CS, 0);
	} else {
		#if 0
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
		#endif
	}
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
	
	//Step 1: Set PLL
	{SMART_CONFIG_CMD, 0xba},
	{SMART_CONFIG_DATA, 0x09},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xb9},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_UDELAY, 2000}, //delay_ms(2)

		//Step 2: Now it is safe to set PMP at max. speed
	{SMART_CONFIG_CMD, 0xbb},
	{SMART_CONFIG_DATA, 0x48},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xd6},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},

	//Step 4: Set MIPI packet format
	{SMART_CONFIG_CMD, 0xb7},
	{SMART_CONFIG_DATA, 0x43},
	{SMART_CONFIG_DATA, 0x02},

    //Step 5: set Virtual Channel (VC) to use
	{SMART_CONFIG_CMD, 0xb8},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},

    ////////Disp_AUO_Init();//////////////
	/*0x35 DCS, 2 parameters*/
	{SMART_CONFIG_CMD, 0xB7},   //DCS 
	{SMART_CONFIG_DATA, 0x42},
	{SMART_CONFIG_DATA, 0x02},
	
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	
	{SMART_CONFIG_CMD, 0x36},//????
	{SMART_CONFIG_DATA, 0x08},//RGB
	
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0x3A},
	{SMART_CONFIG_DATA, 0x77}, //Set 0x3a to 0x07 (24-bit color)

	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x35},


	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},

	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x11},
	//
	{SMART_CONFIG_CMD, 0xbc},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0xbd},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_CMD, 0x29},

	{SMART_CONFIG_CMD, 0xB7},
	{SMART_CONFIG_DATA, 0x43},
	{SMART_CONFIG_DATA, 0x02},
	//Step 7: Now configuration parameters sent to AUO/LG panel for 16-bits/pixel

	{SMART_CONFIG_CMD, 0xBC},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
//
	{SMART_CONFIG_CMD, 0xBD},
	{SMART_CONFIG_DATA, 0x0B},
	{SMART_CONFIG_DATA, 0x00},
};


unsigned long rm67162_ic_ssd2805c_cmd_buf[]= {
	0x2c2c2c2c,
};

struct fb_videomode jzfb0_videomode = {
	.name = "240x240",
	.refresh = 60,
	.xres = 320,
	.yres = 480,
	.init_pixclock = KHZ2PICOS(10000),
	.pixclock = KHZ2PICOS(50000),
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
	.smart_config.write_gram_cmd = rm67162_ic_ssd2805c_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(rm67162_ic_ssd2805c_cmd_buf),
	.smart_config.bus_width = 8,
    .smart_config.data_table_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(rm67162_ic_ssd2805c_data_table),
	.smart_config.data_table = rm67162_ic_ssd2805c_data_table,
	.dither_enable = 0,
};
/**************************************************************************************************/
/*
void fix_new_lcd(void)
{
	unsigned int value=0;
	unsigned int pull_value=0;

	pull_value = *((volatile unsigned int*)(0xb0010300+0x70)) & (1 << 3);
	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x78)) = 1 << 3;

	value = (*((volatile unsigned int *)(0xb0010300+0))) & (1 << 3);
	if(!value) {
		jzfb_pdata.smart_config.length_data_table = ARRAY_SIZE(new_truly_tft240240_data_table);
		jzfb_pdata.smart_config.data_table = new_truly_tft240240_data_table;
	}

	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x74)) = 1 << 3;

}
*/
#ifdef CONFIG_BACKLIGHT_PWM
static int backlight_init(struct device *dev)
{
	//int ret;
#if 0
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}
#endif

#if 0
	printk("------------------ %s --------------------\n", __func__);
	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);
#endif	
	return 0;
}

static int backlight_notify(struct device *dev, int brightness)
{

#if 0
	if (brightness)
		gpio_direction_output(GPIO_BL_PWR_EN, 1);
	else
		gpio_direction_output(GPIO_BL_PWR_EN, 0);
#endif

	return brightness;
}

static void backlight_exit(struct device *dev)
{
	//gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 100,
	.pwm_period_ns	= 10000,
    .active_level   = 1,
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
