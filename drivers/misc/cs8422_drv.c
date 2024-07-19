

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
#include <linux/soundcard.h>
#include <soc/gpio.h>


#define I2C_ADDR 0x11
#define CS8422_DEBUG
//***ioctl command****
#define CS8422_IOCTL_INIT 1
#define CS8422_IOCTL_SWITCH 2
#define CS8422_IOCTL_SAMPRATE 3

#ifdef CS8422_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

// Input sample rate is 32KHz, FS_XTI = 0.333, The register value is 0x0155~0x0156.
#define SAMPLE_32K_MIN                        0x0145
#define SAMPLE_32K_MAX                        0x0166
// Input sample rate is 44.1KHz, FS_XTI = 0.459, The register value is 0x01d6~0x01d7.
#define SAMPLE_441K_MIN                        0x01c6
#define SAMPLE_441K_MAX                        0x01e7
// Input sample rate is 48KHz, FS_XTI = 0.5, The register value is 0x0200.
#define SAMPLE_48K_MIN                        0x01f0
#define SAMPLE_48K_MAX                        0x0211
// Input sample rate is 64KHz, FS_XTI = 0.666, The register value is 0x02AA~0x02AB.
#define SAMPLE_64K_MIN                        0x029A
#define SAMPLE_64K_MAX                        0x02BB
// Input sample rate is 88.2KHz, FS_XTI = 0.919, The register value is 0x03ac~0x03ad.
#define SAMPLE_882K_MIN                        0x0390
#define SAMPLE_882K_MAX                        0x03bd
// Input sample rate is 96KHz, FS_XTI = 1, The register value is 0x0400.
#define SAMPLE_96K_MIN                        0x03f0
#define SAMPLE_96K_MAX                        0x0421
// Input sample rate is 64KHz, FS_XTI = 0.666, The register value is 0x0555~0x0556.
#define SAMPLE_128K_MIN                        0x0545
#define SAMPLE_128K_MAX                        0x0566
// Input sample rate is 176.4KHz, FS_XTI = 1.837, The register value is 0x0759~0x075a.
#define SAMPLE_176K_MIN                        0x0749
#define SAMPLE_176K_MAX                        0x078a
// Input sample rate is 192KHz, FS_XTI = 2, The register value is 0x0800.
#define SAMPLE_192K_MIN                        0x07f0
#define SAMPLE_192K_MAX                        0x0831




static char driver_name[] = "shanling_spdif";
static struct i2c_client *i2c_dev = NULL;
static unsigned char cs8422_pwr_flag = 0;
static unsigned char cs8422_input_mode = 0;
static int reset_gpio    = GPIO_PA(18);
static int power_gpio    = GPIO_PA(17);


static unsigned int cs8422_i2c_read(unsigned char reg)
{
	int ret = -1;
	unsigned char data[4]={0,0,0,0};
	int len = 1;
	if(i2c_dev == NULL)		return 0;
	ret = i2c_master_send(i2c_dev, &reg,len);
	if (ret < 1) {
		akdbgprt("%s 0x%02x err\n", __func__, reg);
		return len;
	}

	ret = i2c_master_recv(i2c_dev, data, len);
	if (ret < len)
		akdbgprt("%s 0x%02x err\n", __func__, reg);
		
	return data[0];
}

static int cs8422_i2c_write(unsigned char reg,
	unsigned int value)
{
	int ret = -1;
	int len = 1;
	unsigned char buf[4] = {0};
	if(i2c_dev == NULL)	return 0;
	buf[0] = reg;
	buf[1] = value&0xFF;
	ret = i2c_master_send(i2c_dev, buf, len+1);
	if (ret < len+1)
	{
        akdbgprt("%s 0x%02x err %d!\n", __func__, reg, ret);
	}
	return ret < len+1 ? ret : 0;
}




int cs8422_update_samprate(void)
{
	int samprate = 0;
	if(cs8422_pwr_flag == 1)
	{
		unsigned char highbyte = cs8422_i2c_read(0x17);
		unsigned char lowbyte = cs8422_i2c_read(0x18);
		int value = ((highbyte&0xFF)<<8)|(lowbyte&0xFF);
		if(value >= SAMPLE_32K_MIN && value <= SAMPLE_32K_MAX)
		{
			samprate = 32000;
		}
		else if(value >= SAMPLE_441K_MIN && value <= SAMPLE_441K_MAX)
		{
			samprate = 44100;
		}
		else if(value >= SAMPLE_48K_MIN && value <= SAMPLE_48K_MAX)
		{
			samprate = 48000;
		}
		else if(value >= SAMPLE_64K_MIN && value <= SAMPLE_64K_MAX)
		{
			samprate = 64000;
		}
		else if(value >= SAMPLE_882K_MIN && value <= SAMPLE_882K_MAX)
		{
			samprate = 88200;
		}
		else if(value >= SAMPLE_96K_MIN && value <= SAMPLE_96K_MAX)
		{
			samprate = 96000;
		}
		else if(value >= SAMPLE_128K_MIN && value <= SAMPLE_128K_MAX)
		{
			samprate = 128000;
		}
		else if(value >= SAMPLE_176K_MIN && value <= SAMPLE_176K_MAX)
		{
			samprate = 176400;
		}
		else if(value >= SAMPLE_192K_MIN && value <= SAMPLE_192K_MAX)
		{
			samprate = 192000;
		}
	}
	return samprate;
}

void cs8422_switch_input(int mode)
{
	if(cs8422_pwr_flag == 1)
	{
		akdbgprt("====[%s]==mode[%d]===\n",__FUNCTION__,mode);
		if(mode == 1) //RX0 
		{
			cs8422_i2c_write(0x03,0x80);
			cs8422_input_mode = 1;
		}
		else if(mode == 2) //RX1
		{
			cs8422_i2c_write(0x03,0xA0);
			cs8422_input_mode = 2;
		}
	}
}

void cs8422_power(int mode)
{
	akdbgprt("====[%s]==mode[%d]=cs8422_pwr_flag[%d]==\n",__FUNCTION__,mode,cs8422_pwr_flag);
	if(mode == 1) //power on
	{
		if(cs8422_pwr_flag == 0)
		{
			if(power_gpio > 0)		gpio_direction_output(power_gpio,1);
			if(reset_gpio > 0)
			{
				gpio_direction_output(reset_gpio,1);
				msleep(20);
	  			gpio_direction_output(reset_gpio,0);
				msleep(20);
			}
			cs8422_i2c_write(0x02,0x40);
			cs8422_i2c_write(0x04,0x24);
			cs8422_i2c_write(0x08,0x20);
			cs8422_i2c_write(0x09,0x48);
			cs8422_i2c_write(0x0a,0x02);
			cs8422_i2c_write(0x0c,0x84);
			cs8422_i2c_write(0x0d,0x84);
			cs8422_i2c_write(0x0e,0x7f);
			if(cs8422_input_mode == 1) //RX0 
			{
				cs8422_i2c_write(0x03,0x80);
			}
			else if(cs8422_input_mode == 2) //RX1
			{
				cs8422_i2c_write(0x03,0xA0);
			}
			cs8422_pwr_flag = 1;
		}
	}
	else //powroff 
	{
		if(cs8422_pwr_flag == 1)
		{
			if(reset_gpio > 0) gpio_direction_output(reset_gpio, 1);
			if(power_gpio > 0) gpio_direction_output(power_gpio,0);
			cs8422_pwr_flag = 0;
		}
	}
}




static long cs8422_converter_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	long val = 0;
	switch(cmd)
	{
		case CS8422_IOCTL_INIT:
			ret = get_user(val, (long *) arg);
			if(val == 0 || val == 1) //dac 
			{
				cs8422_power(val);
			}
			break;
		case CS8422_IOCTL_SWITCH: //switch coxial or optical			ret = get_user(val, (long *) arg);
			if(val == 1 || val == 2)
			{
				cs8422_switch_input(val);
			}
			break;
		case CS8422_IOCTL_SAMPRATE: //read samprate			val = cs8422_update_samprate();
			put_user(val,(long *) arg);
			break;
	}
	return 0;
}

static struct file_operations cs8422_converter_opt_fops = {
	    .owner  	    =	THIS_MODULE,
        .unlocked_ioctl =   cs8422_converter_ioctl,
};


static struct miscdevice cs8422_converter_misc = {	
	.minor	=		MISC_DYNAMIC_MINOR,
	.name   =       (char *)driver_name,
	.fops   =       &cs8422_converter_opt_fops,	
};




static int cs8422_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	int ret = 0;
    akdbgprt("\t[%s] (%d)\n",__FUNCTION__,__LINE__);
	i2c_dev = i2c;
	if(power_gpio > 0)
	{
	    ret = gpio_request(power_gpio, "cs8422 pwr en");
		if(ret == 0)	gpio_direction_output(power_gpio, 0);
	}
	if(reset_gpio > 0)
	{
		ret = gpio_request(reset_gpio, "cs8422 reset");
        if(ret == 0) gpio_direction_output(reset_gpio, 0);
	}
	cs8422_power(1);
	printk("----cs8422_i2c.----\n");
    return ret;
}

static int cs8422_i2c_remove(struct i2c_client *client)
{	
	akdbgprt("\t[%s](%d)\n",__FUNCTION__,__LINE__);
	return 0;
}

static const struct i2c_device_id cs8422_i2c_id[] = {
    { "cs8422", 0 },
};


static struct i2c_driver cs8422_i2c_driver = {
    .driver = {
        .name = "cs8422",
		.owner = THIS_MODULE,
    },
    .probe = cs8422_i2c_probe,
    .remove = cs8422_i2c_remove,
    .id_table = cs8422_i2c_id,
};


static int __init cs8422_converter_init(void)
{
	int ret = 0;

    ret = i2c_add_driver(&cs8422_i2c_driver);
    if (ret)
	{
        printk(KERN_ERR "[%s]: failed to register i2c driver\n",__FUNCTION__);
        return ret;
    }
	ret |= misc_register(&cs8422_converter_misc);
	
	return ret;
}

static void __exit cs8422_converter_exit(void)
{
	misc_deregister(&cs8422_converter_misc);
    i2c_del_driver(&cs8422_i2c_driver);
}

module_init(cs8422_converter_init);
module_exit(cs8422_converter_exit);


MODULE_DESCRIPTION("ASoC CS8422 codec driver");
MODULE_LICENSE("GPL v2");






