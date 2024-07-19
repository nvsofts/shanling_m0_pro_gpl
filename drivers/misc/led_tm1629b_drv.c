

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <soc/gpio.h>

#define LED_VCLK   GPIO_PA(23)
#define LED_VDATA  GPIO_PA(22)
#define LED_STB1   GPIO_PA(24)
#define LED_STB2   GPIO_PA(25)


#define LED_DATA_WRITE_INC_ADDR			0x40
#define LED_ADDRESS_SETING				0xc0
#define LED_MAX_GRID					8

#define LED_DELAY   udelay(1)

enum {
	DIMMER_ACTION = 0x80,
	DIMMER_LEVEL_0 = 0,
	DIMMER_LEVEL_1,
	DIMMER_LEVEL_2,
	DIMMER_LEVEL_MAX
};


static const unsigned char bLedDim_tbl[DIMMER_LEVEL_MAX] =
{
	0x8a,	0x89,	0x80
};

static void Led_send_byte(unsigned char KeyData)
{
	unsigned char i;
	for(i=0;i<8;i++) 
	{
		gpio_set_value(LED_VCLK,0);
		LED_DELAY;
		if (KeyData & 0x01) {
			gpio_set_value(LED_VDATA,1);
		} else {
			gpio_set_value(LED_VDATA,0);
		}
		
		gpio_set_value(LED_VCLK,1);
		KeyData >>= 1;
		LED_DELAY;
	}
	gpio_set_value(LED_VDATA,1);
	LED_DELAY;
}

/*========================================================
 Function:  Send 16311(2)'s command to 16311(2).

 Parameter: Command: the 16311(2)'s command which want to write to 16311(2).

 Return:    void.
========================================================*/
static void Led_send_cmd(unsigned char Command,int chip)
{
	if(chip == 1)
	{
		gpio_set_value(LED_STB1,0);
	}
	else if(chip == 2)
	{
		gpio_set_value(LED_STB2,0);
	}
	LED_DELAY;
	Led_send_byte(Command);
	if(chip == 1)
	{
		gpio_set_value(LED_STB1,1);
	}
	else if(chip == 2)
	{
		gpio_set_value(LED_STB2,1);
	}
	LED_DELAY;
}

void Led_dim_ctrl(uint8_t dim1,uint8_t dim2)
{
	Led_send_cmd(bLedDim_tbl[dim1],1);
	Led_send_cmd(bLedDim_tbl[dim2],2);
}


void led_init(void)
{
	unsigned char i;
	//****stb1****
	Led_send_cmd(LED_DATA_WRITE_INC_ADDR,1);
	gpio_set_value(LED_STB1,0);
	LED_DELAY;
	Led_send_byte(LED_ADDRESS_SETING);
	
	for (i = 0; i < LED_MAX_GRID * 2; i++) {
		Led_send_byte(0x00);
	}
	gpio_set_value(LED_STB1,1);
	LED_DELAY;

	//*****stb2****
	Led_send_cmd(LED_DATA_WRITE_INC_ADDR,2);
	gpio_set_value(LED_STB2,0);
	LED_DELAY;
	Led_send_byte(LED_ADDRESS_SETING);
	
	for (i = 0; i < LED_MAX_GRID * 2; i++) {
		Led_send_byte(0x00);
	}
	gpio_set_value(LED_STB2,1);
	LED_DELAY;
	
}

//*****ÌØÊâ****
/*
*0x01 b 
*0x02 c
*0x04 d
*0x08 e
*0x10 f
*0x20g
*0x40 dp
*0x80 a
*/
unsigned char num_table1[10]={0x9F,0x03,0xAD,0xA7,0x33,0xB6,0xBE,0x83,0xBF,0xB7};

//*****Õý³£*****
/*
*0x01    a
*0x02    b
*0x04    c
*0x08    d
*0x10    e 
*0x20   f
*0x40    g
*0x80    dp
*/
unsigned char num_table2[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
int value = 0;


struct led_tm1629b_info
{
	unsigned char vol1; //0~9 
	unsigned char vol0; //0~9
	unsigned char num1; //0~9
	unsigned char num0; //0~9
	unsigned char min_num1; //0~9 
	unsigned char min_num0; //0~9
	unsigned char sec_num1; //0~9
	unsigned char sec_num0; //0~9
	unsigned char time_dot; //0~1

	unsigned char led_source; // 1 cd 2 usb 3 bt 4 aux
	unsigned char led_power;  //
};


void Led_refresh(struct led_tm1629b_info* info)
{
	Led_send_cmd(LED_DATA_WRITE_INC_ADDR,1);
	gpio_set_value(LED_STB1,0);
	LED_DELAY;
	Led_send_byte(LED_ADDRESS_SETING);
	//*******1********
	if(info->vol1 >= 0 && info->vol1 <= 9)
	{
		Led_send_byte(num_table1[info->vol1]);
		Led_send_byte(num_table1[info->vol1]);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******2********
	if(info->vol0 >= 0 && info->vol0 <= 9)
	{
		Led_send_byte(num_table1[info->vol0]);
		Led_send_byte(num_table1[info->vol0]);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******3********
	if(info->num1 >= 0 && info->num1 <= 9)
	{
		Led_send_byte(num_table2[info->num1]);
		Led_send_byte(num_table2[info->num1]);
	}
	else if(info->num1 == 100) //U
	{
		Led_send_byte(0x3E);
		Led_send_byte(0x3E);
	}
	else if(info->num1 == 101) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******4********
	if(info->num0 >= 0 && info->num0 <= 9)
	{
		Led_send_byte(num_table2[info->num0]);
		Led_send_byte(num_table2[info->num0]);
	}
	else if(info->num0 == 100) //P
	{
		Led_send_byte(0x73);
		Led_send_byte(0x73);
	}
	else if(info->num0 == 101) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	gpio_set_value(LED_STB1,1);
	LED_DELAY;

	Led_send_cmd(LED_DATA_WRITE_INC_ADDR,2);
	gpio_set_value(LED_STB2,0);
	LED_DELAY;
	Led_send_byte(LED_ADDRESS_SETING);

	//*******1********
	unsigned char timedot = 0x00;
	if(info->time_dot == 1)
	{
		timedot = 0x80;
	}
	if(info->min_num1 >= 0 && info->min_num1 <= 9)
	{
		Led_send_byte(num_table2[info->min_num1]|timedot); //0x80 dot1
		Led_send_byte(num_table2[info->min_num1]|timedot); //0x80 dot1
	}
	else if(info->min_num1 == 100) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else if(info->min_num1 == 101) //E
	{
		Led_send_byte(0x79);
		Led_send_byte(0x79);
	}
	else if(info->min_num1 == 110) //L
	{
		Led_send_byte(0x38);
		Led_send_byte(0x38);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******2********
	timedot = 0x00;
	if(info->time_dot == 1 || info->time_dot == 2)
	{
		timedot = 0x80;
	}
	if(info->min_num0 >= 0 && info->min_num0 <= 9)
	{
		Led_send_byte(num_table2[info->min_num0]|timedot);//0x80 dot2
		Led_send_byte(num_table2[info->min_num0]|timedot);//0x80 dot2
	}
	else if(info->min_num0 == 100) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else if(info->min_num0 == 101) //r
	{
		Led_send_byte(0x31);
		Led_send_byte(0x31);
	}
	else if(info->min_num0 == 110) //A
	{
		Led_send_byte(0x77);
		Led_send_byte(0x77);
	}
	else if(info->min_num0 == 111) //D
	{
		
		Led_send_byte(0x3F);
		Led_send_byte(0x3F);
	}
	else if(info->min_num0 == 112) //S
	{
		Led_send_byte(0x6D);
		Led_send_byte(0x6D);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******3********
	if(info->sec_num1 >= 0 && info->sec_num1 <= 9)
	{
		Led_send_byte(num_table2[info->sec_num1]);
		Led_send_byte(num_table2[info->sec_num1]);
	}
	else if(info->sec_num1 == 100) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else if(info->sec_num1 == 101) //r
	{
		Led_send_byte(0x31);
		Led_send_byte(0x31);
	}
	else if(info->sec_num1 == 110) //A
	{
		Led_send_byte(0x77);
		Led_send_byte(0x77);
	}
	else if(info->sec_num1 == 111) //B
	{
		Led_send_byte(0x7F);
		Led_send_byte(0x7F);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//*******4********
	if(info->sec_num0 >= 0 && info->sec_num0 <= 9)
	{
		Led_send_byte(num_table2[info->sec_num0]);
		Led_send_byte(num_table2[info->sec_num0]);
	}
	else if(info->sec_num0 == 100) //-
	{
		Led_send_byte(0x40);
		Led_send_byte(0x40);
	}
	else if(info->sec_num0 == 110) //C
	{
		Led_send_byte(0x39);
		Led_send_byte(0x39);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
#if 0
	//********µÆ****************
	//*******0x01 cd 0x02 udisk********
	if(info->led_source == 4)
	{
		Led_send_byte(0x01);
		Led_send_byte(0x01);
	}
	else if(info->led_source == 1)
	{
		Led_send_byte(0x02);
		Led_send_byte(0x02);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//******0x04 bt 0x08 aux********
	if(info->led_source == 2)
	{
		Led_send_byte(0x04);
		Led_send_byte(0x04);
	}
	else if(info->led_source == 3)
	{
		Led_send_byte(0x08);
		Led_send_byte(0x08);
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
	//******0x20 powr****************
	if(info->led_power == 1)
	{
		Led_send_byte(0x20);
		Led_send_byte(0x20);	
	}
	else
	{
		Led_send_byte(0x00);
		Led_send_byte(0x00);
	}
#endif
	gpio_set_value(LED_STB2,1);
	LED_DELAY;
}



static long led_tm1629b_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct led_tm1629b_info led_info;
	int ret;
	long val = 0;
	memset(&led_info,0,sizeof(struct led_tm1629b_info));
	switch(cmd)
	{
		case 0x01:
		ret = copy_from_user((struct led_tm1629b_info*)&led_info, (long *)arg, sizeof(struct led_tm1629b_info));
		if(ret == 0)
		{
			//printk("==info.num[%d][%d][%d][%d]==time[%d][%d]:[%d][%d]==ledsource[%d]===ledpower[%d]==\n",led_info.vol1,led_info.vol0,led_info.num1,led_info.num0,led_info.min_num1,led_info.min_num0,led_info.sec_num1,led_info.sec_num0,led_info.led_source,led_info.led_power);
			Led_refresh(&led_info);
			Led_dim_ctrl(DIMMER_LEVEL_0,DIMMER_LEVEL_1);
		}
		break;

	}
	return 0;
}

static struct file_operations led_tm1629b_opt_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =	led_tm1629b_ioctl,
};

static char driver_name[] = "led_tm1629b_dev";

static struct miscdevice led_tm1629b_misc = {	
	.minor	=		MISC_DYNAMIC_MINOR,
	.name	=		(char *)driver_name,
	.fops	=		&led_tm1629b_opt_fops, 
};



static int __init led2281_display_init(void)
{
	int ret = 0;
	printk("=======fun[%s]===\n",__FUNCTION__);
	
	gpio_request(LED_VCLK, "led2281 vclk");
	gpio_direction_output(LED_VCLK, 1);

	gpio_request(LED_VDATA, "led2281 vdata");
	gpio_direction_output(LED_VDATA, 1);


	gpio_request(LED_STB1, "led2281 stb1");
	gpio_direction_output(LED_STB1, 1);

	gpio_request(LED_STB2, "led2281 stb2");
	gpio_direction_output(LED_STB2, 1);

	led_init();
	Led_dim_ctrl(DIMMER_LEVEL_2,DIMMER_LEVEL_2);
	//**********show power led****
	struct led_tm1629b_info info;
	info.vol1 = 0xFF;
	info.vol0 = 0xFF;
	info.num1 = 0xFF;
	info.num0 = 0xFF;

	info.min_num1 = 0xFF;
	info.min_num0 = 0xFF;

	info.sec_num1 = 0xFF;
	info.sec_num0 = 0xFF;
	info.led_source = 0;
	info.led_power = 1;
	Led_refresh(&info);
	Led_dim_ctrl(DIMMER_LEVEL_0,DIMMER_LEVEL_1);
	return misc_register(&led_tm1629b_misc);
}

static void __exit led2281_display_exit(void)
{
	misc_deregister(&led_tm1629b_misc);
}

module_init(led2281_display_init);
module_exit(led2281_display_exit);


MODULE_DESCRIPTION("ASoC led display 2281 driver");
MODULE_LICENSE("GPL v2");






