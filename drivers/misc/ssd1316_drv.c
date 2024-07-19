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

#include "ssd1316_drv.h"

typedef struct
{
	struct i2c_client	*client;
	unsigned char DataBuffer[128+1];
}ssd1316_data;

static ssd1316_data *pSSD1316 = 0;


static void OLED_WriteCommand(unsigned char command)
{
	int ret;

	if(pSSD1316 == 0)
	{
		return;
	}

	pSSD1316->DataBuffer[0] = 0x00;
	pSSD1316->DataBuffer[1] = command;
	ret = i2c_master_send(pSSD1316->client,pSSD1316->DataBuffer,2);
	if (ret < 1)
	{
		printk("oled command error.\n");
		return;
	}
}

static void OLED_WriteCommands(unsigned char *pCmd,unsigned char CmdLen)
{
	int ret;

	if(pSSD1316 == 0)
	{
		return;
	}
	pSSD1316->DataBuffer[0] = 0x00;
	memcpy(pSSD1316->DataBuffer+1,pCmd,CmdLen);
	ret = i2c_master_send(pSSD1316->client,pSSD1316->DataBuffer,CmdLen+1);
	if (ret < 1)
	{
		printk("oled command error.\n");
		return;
	}
}

static void OLED_WriteData(unsigned char data)
{
	int ret;

	if(pSSD1316 == 0)
	{
		return;
	}
	pSSD1316->DataBuffer[0] = 0x40;
	pSSD1316->DataBuffer[1] = data;
	ret = i2c_master_send(pSSD1316->client,pSSD1316->DataBuffer,2);
	if (ret < 1)
	{
		printk("oled data error.\n");
		return;
	}

}

static void OLED_WriteDatas(unsigned char *pData,unsigned char DataLen)
{
	int ret;

	if(pSSD1316 == 0)
	{
		return;
	}
	pSSD1316->DataBuffer[0] = 0x40;
	memcpy(pSSD1316->DataBuffer+1,pData,DataLen);
	ret = i2c_master_send(pSSD1316->client,pSSD1316->DataBuffer,DataLen+1);
	if (ret < 1)
	{
		printk("oled data error.\n");
		return;
	}
}


static void OLED_SetOPWindow(unsigned char x,unsigned char page)
{
    unsigned char CmdBuf[3];
    
    CmdBuf[0] = 0xb0|(1-page);
    CmdBuf[1] = x&0x0f;
    CmdBuf[2] = (x>>4)|0x10;    
    OLED_WriteCommands(CmdBuf,3);
    
}

static void OLED_PageCls(unsigned char x,unsigned char page,unsigned char len)//0-7
{
	unsigned char i;
    
	OLED_SetOPWindow(x,page);
	for(i=0;i<len;i++)
	{
		OLED_WriteData(0x00);
	}	
}

static const unsigned char InitTable_Normal[] = {
	
	0xAE,//关闭显示 
	0xAD,
	0x11,
	
	0x81,//对比度设置
	0xFF,//1~255;默认0X7F (亮度设置，越大越亮)	
	
	0xA0,//段重定义设置，bit0:0，0->0;1，0->127;  
	0xC0,//设置COM扫描方向;bit3:0，普通模式;1，重定义模式 COM[N-1]->COM0;N:驱动路数			
	0xA8,//设置驱动路数  
	0x0F,//默认0X3F(1/64)  	
	0xD3,//设置显示偏移  
	0x1F,//默认为0  

	0xD5,//设置时钟分频因子，震荡频率  
	0xF4,
	
	0xD9,//设置预充电周期
	0x22,//[3:0]，PHASE 1;[7:4]，PHASE 2;
	
	0xDA,//设置COM硬件引脚配置
	0x12,//[5:4]配置

	0xDB,//设置VCOMH 电压倍率
	0x30,//[6:4] 000，0.65*vcc;001，0.77*vcc;011，0.83*vcc;	
	0xA4,//全局显示开启;bit0:1，开启;0，关闭;(白屏/黑屏)
	0xA6,//设置显示方式;bit0:1，反相显示;0		
	0x8D,  //set vcomh
	0x14,  // Set_Charge_Pump 0x14:9v; 0x15 7.5v
	0x20,//设置内存地址模式
	0x02,//[1:0]，00，列地址模式;01，行地址模式;10，页地址模式;默认10;
	//0xAF//开启显示
};

static void OLED_Init(void)
{
    OLED_WriteCommands((unsigned char*)InitTable_Normal,sizeof(InitTable_Normal));
}

static void OLED_On(void)
{
    OLED_WriteCommand(0xAF);
}

static void OLED_Off(void)
{
    OLED_WriteCommand(0xAE);
}

static void OLED_ClearAll(void)
{
	unsigned char i;

	for(i=0;i<2;i++)
	{
		OLED_PageCls(0,i,128);
	}
}

static void OLED_SetBacklight(unsigned char value)
{
    unsigned char tmpbuf[2];
    
    tmpbuf[0] = 0x81;
    tmpbuf[1] = value;

    OLED_WriteCommands(tmpbuf,2);
}

static void OLED_SetRotate(unsigned char bFlag)
{
    unsigned char tmpbuf[2];
    
    if(bFlag)
    {
        tmpbuf[0] = 0xA1;
        tmpbuf[1] = 0xC8;
    }
    else
    {
        tmpbuf[0] = 0xA0;
        tmpbuf[1] = 0xC0;
    }
    OLED_WriteCommands(tmpbuf,2);
}


static void OLED_PageDatum(OLED_STRUCT* pData)
{
	unsigned char i;
	unsigned char* pRes;
    
	pRes = pData->pRes;
	for(i=0;i<pData->Page;i++)
	{
		OLED_SetOPWindow(pData->X,(unsigned char)(pData->Y+i));
		OLED_WriteDatas(pRes,pData->Len);
		pRes += pData->Len;
	}
}


static const unsigned char Logo[] = 
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x07,0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x1E,0x13,0x11,0x18,0x08,0x00,0x00,0x1F,0x1F,0x04,
	0x04,0x07,0x03,0x00,0x00,0x00,0x04,0x04,0x04,0x07,0x03,0x00,0x00,0x07,0x07,0x04,0x04,0x07,0x03,0x00,0x00,0x10,0x10,0x1F,0x1F,0x00,0x00,0x00,0x00,0x04,0x04,0x37,
	0x37,0x00,0x00,0x00,0x00,0x07,0x07,0x04,0x04,0x07,0x03,0x00,0x00,0x03,0x07,0x04,0x04,0x07,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x01,
	0x01,0x1F,0x1F,0x00,0x00,0x10,0x10,0x11,0x17,0x1E,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x07,0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x30,0x10,0x90,0xF0,0x60,0x00,0x00,0xF0,0xF0,0x00,
	0x00,0xF0,0xF0,0x00,0x00,0x60,0xF0,0x90,0x90,0xF0,0xF0,0x00,0x00,0xF0,0xF0,0x00,0x00,0xF0,0xF0,0x00,0x00,0x10,0x10,0xF0,0xF0,0x10,0x10,0x00,0x00,0x10,0x10,0xF0,
	0xF0,0x10,0x10,0x00,0x00,0xF0,0xF0,0x00,0x00,0xF0,0xF0,0x00,0x00,0xE2,0xF2,0x12,0x12,0xFE,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF0,0x00,
	0x00,0xF0,0xF0,0x00,0x00,0x00,0x70,0xF0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"H:\H7 logo.BMP",0*/

};


static void OLED_Logo(void)
{
	OLED_STRUCT LogoInfo;

	LogoInfo.X = 0;
	LogoInfo.Y = 0;
	LogoInfo.Len = 128;
	LogoInfo.Page = 2;
	LogoInfo.pRes = (unsigned char*)Logo;

	OLED_PageDatum(&LogoInfo);
}


static int ssd1316_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ssd1316_data *pData;
	struct oled1316_platform_data *pdata;
	int ret = 0;
	int err = 0;

	pdata = (struct oled1316_platform_data *)client->dev.platform_data;

	if (!(pData = kzalloc(sizeof(ssd1316_data), GFP_KERNEL)))
		return -ENOMEM;

	if(pdata->gpio_enable >= 0)
	{
		err = gpio_request(pdata->gpio_enable, "oled enable");
		if (err < 0)
		{
			dev_err(&client->dev, "%s:failed to set oled enable.\n",__func__);
			return -2;
		}
		gpio_direction_output(pdata->gpio_enable, 1);
	}
	if(pdata->gpio_reset >= 0)
	{
		err = gpio_request(pdata->gpio_reset, "oled reset");
		if (err < 0)
		{
			dev_err(&client->dev, "%s:failed to set oled reset.\n",__func__);
			return -3;
		}
		gpio_direction_output(pdata->gpio_reset, 0);
		mdelay(20);
		gpio_direction_output(pdata->gpio_reset, 1);
		mdelay(20);
	}
	printk("----%s:set oled gpio ok.===[%d]==[%d]===\n----",__func__,pdata->gpio_enable,pdata->gpio_reset);
	i2c_set_clientdata(client, pData);
	pData->client = client;
	pSSD1316 = pData;
	OLED_Init();
	OLED_ClearAll();
	
	OLED_Logo();
	OLED_On();
	return 0;
}

static int ssd1316_remove(struct i2c_client *client)
{
	struct ssd1316_data *pData = i2c_get_clientdata(client);
	kfree(pData);
	return 0;
}


static const struct i2c_device_id ssd1316_id[] = {
	{ "ssd1316", 0 },
};

static struct i2c_driver ssd1316_driver = {
	.driver = {
	.name = "ssd1316",
	.owner = THIS_MODULE,
	},
	.probe      = ssd1316_probe,
	.remove     = ssd1316_remove,
	.id_table	= ssd1316_id,
};


static long oled_ssd1316_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	OLED_STRUCT DrawData;
	int ret;
	long val = 0;

	switch(cmd)
	{
		case 0:
			ret = get_user(val, (long *) arg);
			if(ret == 0)
			{
				if(val == 0)
				{
					OLED_Off();
				}
				else if(val == 1)
				{
					OLED_On();
				}
			}
			break;
		
		case 1:
			ret = get_user(val, (long *) arg);
			if(ret == 0)
			{
				if(val <= 0xff)
				{
					OLED_SetBacklight((unsigned char)val);
				}
			}
			break;
		
		case 2:
			ret = get_user(val, (long *) arg);
			if(ret == 0)
			{
				if(val == 0)
				{
					OLED_SetRotate(0);
				}
				else if(val == 1)
				{
					OLED_SetRotate(1);
				}
			}
			break;

		case 3:
			ret = copy_from_user((char*)&DrawData, (long *) arg, sizeof(OLED_STRUCT));
			if(ret == 0)
			{
				//printk("==oled==%d,%d,%d,%d\n",DrawData.X,DrawData.Y,DrawData.Len,DrawData.Page);
				OLED_PageDatum(&DrawData);
			}
		break;

	case 4:
		OLED_Logo();
		OLED_On();
			break;

	}

	return 0;
}

static struct file_operations oled_ssd1316_opt_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =	oled_ssd1316_ioctl,
};

static char driver_name[] = "oled_ssd1316";

static struct miscdevice oled_ssd1316_misc = {	
	.minor	=		MISC_DYNAMIC_MINOR,
	.name	=		(char *)driver_name,
	.fops	=		&oled_ssd1316_opt_fops, 
};



static int __init ssd1316_init(void)
{
	int ret;

	ret = i2c_add_driver(&ssd1316_driver);
	ret |= misc_register(&oled_ssd1316_misc);
	return ret;
}

static void __exit ssd1316_exit(void)
{
	i2c_del_driver(&ssd1316_driver);
}


fs_initcall(ssd1316_init);
module_exit(ssd1316_exit);
MODULE_LICENSE("GPL");

