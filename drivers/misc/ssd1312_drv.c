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

#include "ssd1312_drv.h"
#include <linux/mtd/mtd.h>

char rotate_flag = 0;

//==============================================================
static const int gpio_oled_cs = GPIO_PC(1);
static const int gpio_oled_dc = GPIO_PC(0);
static const int gpio_oled_rst = GPIO_PC(3);
static const int gpio_oled_clk = GPIO_PC(16);
static const int gpio_oled_dat = GPIO_PC(17);


#define	OLED_CS_LOW()	gpio_set_value(gpio_oled_cs, 0)
#define	OLED_CS_HIGH()	gpio_set_value(gpio_oled_cs, 1)

#define OLED_DC_LOW()	gpio_set_value(gpio_oled_dc, 0)
#define OLED_DC_HIGH()	gpio_set_value(gpio_oled_dc, 1)

#define OLED_RST_LOW()	gpio_set_value(gpio_oled_rst, 0)
#define OLED_RST_HIGH()	gpio_set_value(gpio_oled_rst, 1)

#define OLED_CLK_LOW()	gpio_set_value(gpio_oled_clk, 0)
#define OLED_CLK_HIGH()	gpio_set_value(gpio_oled_clk, 1)

#define OLED_DAT_LOW()	gpio_set_value(gpio_oled_dat, 0)
#define OLED_DAT_HIGH()	gpio_set_value(gpio_oled_dat, 1)

//==============================================================
static unsigned long ScreenBuffer[128*3];
static unsigned long *pFrameA = ScreenBuffer;
static unsigned long *pFrameB = ScreenBuffer+128;
static unsigned long *pFrameC = ScreenBuffer+256;

static oled_bitmap DisplayFrameData = 
{
	0,		//X
	0,		//Y
	128,	//Len
	4,		//Page
	(unsigned char*)&ScreenBuffer
};

//==============================================================
static void OLED_UpdateData(oled_bitmap* pData);


static void OLED_Update(void)
{
	OLED_UpdateData(&DisplayFrameData);
}

static void OLED_ClearAll(void)
{
	memset(pFrameA,0,512);
	OLED_Update();
}

static void OLED_RollRect(unsigned char x_start,unsigned char x_end,unsigned char y_start,unsigned char y_end,unsigned char RollMode)
{
	unsigned char i,j;
	unsigned long DataMaskValue;
	unsigned long LastDataValue;
	unsigned long FontMask;	
	unsigned long BackMask;		
	
	memcpy(pFrameC,pFrameA,512);
	memset(pFrameA,0,512);
	
	if(y_end > 31)
	{
		y_end = 31;
	}
	if(x_end > 127)
	{
		y_end = 127;
	}	
	FontMask = 0;
	for(i=0; i<32; i++)
	{
		if(i >= y_start && i <= y_end)
		{
			FontMask |= (unsigned long)1<<i;
		}
	}
	BackMask = ~FontMask;

	if(RollMode >= ROLL_UP && RollMode <= ROLL_DOWN)
	{
		for(j=y_start; j<=y_end; j++)
		{
			for(i=0; i<128; i++)
			{
				if(i >= x_start && i <= x_end)
				{
					DataMaskValue = pFrameC[i]&FontMask;
					LastDataValue = pFrameB[i]&FontMask;
					if(RollMode == ROLL_UP)
					{
						DataMaskValue <<= (y_end-j);
						if(j < 31)
						{
							LastDataValue >>= j+1;
						}
						else
						{
							LastDataValue = 0;
						}
					}
					else
					{
						DataMaskValue >>= (y_end-j);
						if(j < 31)
						{
							LastDataValue <<= j+1;
						}
						else
						{
							LastDataValue = 0;
						}
					}
					DataMaskValue |= LastDataValue;
					DataMaskValue &= FontMask;
					DataMaskValue |= pFrameB[i]&BackMask;
					pFrameA[i] = DataMaskValue;				
				}
				else
				{
					pFrameA[i] = pFrameB[i];
				}
			}
			OLED_Update();
			mdelay(6);
		}
	}
}


static void OLED_LowRollUp(void)
{
	unsigned char i,j;
	unsigned long DataMaskValue;
	unsigned long LastDataValue;
	unsigned long TempMask;
	
	memcpy(pFrameC,pFrameA,512);
	
	TempMask = 0;
	for(j=0; j<16; j++)
	{
		TempMask |= (1<<(31-j));
		for(i=0; i<128; i++)
		{
			LastDataValue = pFrameB[i]>>(j+1);
			DataMaskValue = pFrameC[i]<<(15-j);
			DataMaskValue &= TempMask;

			pFrameA[i] = (DataMaskValue|LastDataValue)&0xffff0000;
			pFrameA[i] |= pFrameB[i]&0xffff;
		}
		
		OLED_Update();
		mdelay(6);
	}
}
//==============================================================
static void OLED_WriteByte(unsigned char value)
{
	unsigned char i;
	
	for(i=0;i<8;i++)
	{
		OLED_CLK_LOW();
		if(value&0x80)
		{
			OLED_DAT_HIGH();
		}
		else
		{
			OLED_DAT_LOW();
		}
		OLED_CLK_HIGH();
		value <<= 1;
	}
}

static void OLED_SendBytes(unsigned char* pData,unsigned short Size)
{
	unsigned short i;

	for(i=0;i<Size;i++)
	{
		OLED_WriteByte(pData[i]);
	}
}


static void OLED_WriteCommand(unsigned char Cmd)
{
	OLED_DC_LOW();
	OLED_SendBytes(&Cmd,1);
}

static void OLED_WriteCommands(unsigned char *pCmd,unsigned char CmdLen)
{
	OLED_DC_LOW();
	OLED_SendBytes(pCmd,CmdLen);
}


static void OLED_WriteDatas(unsigned char *pData,unsigned short StrLen)
{
	OLED_DC_HIGH();
	OLED_SendBytes(pData,StrLen);
}


static void OLED_SetArea(unsigned char StartColumn,unsigned char EndColumn,unsigned char StartPage,unsigned char EndPage)
{
	unsigned char CmdBuf[6];

	CmdBuf[0] = 0x21;
	CmdBuf[1] = StartColumn;
	CmdBuf[2] = EndColumn;
	
	CmdBuf[3] = 0x22;
	CmdBuf[4] = StartPage;
	CmdBuf[5] = EndPage;	
	OLED_WriteCommands(CmdBuf,6);
}


static const unsigned char InitTable[] = {
	
	0xAE,	//display off
	0xD5,	//set osc division
	0x80,	//[3:0]
	0xA8,	//multiplex ratio 
	0x1F,	//duty = 1/28
	0xAD,	//External or Internal IREF Selection
	0x40,
	0xD3,	//set display offset
	0x10,	//16
	0xA0,	//set segment remap
	0xC8,	//Com scan direction
	0xA6,	//normal / reverse
	0x40,	//set display start line
	0xA4,	//bit0:1 on, 0 off
	0x81,	//contract control
	0x14,	//1~255
	0xD9,	//set pre-charge period
	0x22,	//
	0xDA,	//set COM pins
	0x10,
	0x20,	//Set Memory Addressing Mode
	0x01,	//Vertical Addressing Mode
	0xDB,	//set vcomh
	0x30,	//[6:4] 000£¬0.65*vcc;001£¬0.77*vcc;011£¬0.83*vcc;
	0x8D,	//set charge pump enable
	0x72,	// 0x12:7.5V; 0x52:8V;  0x72:9V;  0x92:10V
};

static void OLED_Reset(void)
{
	OLED_RST_LOW();
	mdelay(30);
	OLED_RST_HIGH();
	mdelay(20);
}



static void OLED_On(void)
{
	OLED_CS_LOW();
	OLED_WriteCommand(0x8D);	//Set DC/DC
	OLED_WriteCommand(0x14);	//DC/DC On
	OLED_WriteCommand(0xAF);
	OLED_CS_HIGH();
}

static void OLED_Off(void)
{
	OLED_CS_LOW();
	OLED_WriteCommand(0xAE);
	OLED_WriteCommand(0x8D);	//Set DC/DC
	OLED_WriteCommand(0x10);	//DC/DC Off
	OLED_CS_HIGH();
}


static void OLED_Init(void)
{
	gpio_request(gpio_oled_cs,"oled_cs");
	gpio_direction_output(gpio_oled_cs, 1);
	
	gpio_request(gpio_oled_dc,"oled_dc");
	gpio_direction_output(gpio_oled_dc, 1);

	gpio_request(gpio_oled_rst,"oled_rst");
	gpio_direction_output(gpio_oled_rst, 1);

	gpio_request(gpio_oled_clk,"oled_clk");
	gpio_direction_output(gpio_oled_clk, 0);

	gpio_request(gpio_oled_dat,"oled_dat");
	gpio_direction_output(gpio_oled_dat, 0);

	OLED_Reset();
	OLED_CS_LOW();
	OLED_WriteCommands((unsigned char*)InitTable,sizeof(InitTable));
	OLED_CS_HIGH();
	OLED_ClearAll();
	//OLED_SetRotate(1);
	OLED_On();
}

static void OLED_SetBacklight(unsigned char value)
{
	OLED_CS_LOW();
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(value);
	OLED_CS_HIGH();
}

static void OLED_SetRotate(unsigned char bFlag)
{
	unsigned char tmpbuf[6];

	if(!bFlag)
	{
		tmpbuf[0] = 0x60;
		tmpbuf[1] = 0xD3;
		tmpbuf[2] = 0x10;
		tmpbuf[3] = 0xA0;
		tmpbuf[4] = 0xC8;
		tmpbuf[5] = 0x40;			
	}
	else
	{
		tmpbuf[0] = 0x60;		
		tmpbuf[1] = 0xD3;
		tmpbuf[2] = 0x30;
		tmpbuf[3] = 0xA1;
		tmpbuf[4] = 0xC0;
		tmpbuf[5] = 0x40;	
	}
	OLED_CS_LOW();	
	OLED_WriteCommands(tmpbuf,6);
	OLED_CS_HIGH();
}


static void OLED_UpdateData(oled_bitmap* pData)
{
	OLED_CS_LOW();
	OLED_SetArea(pData->X,pData->X+pData->Len-1,pData->Y,pData->Y+pData->Page-1);
	OLED_WriteDatas(pData->pRes,pData->Len*pData->Page);
	OLED_CS_HIGH();	
}


static const unsigned char BmpLogo[] = 
{
	0x38,0x18,0x7C,0x30,0xE6,0x20,0xC2,0x20,0x82,0x21,0x82,0x31,0x06,0x1F,0x0E,0x0E,0x00,0x00,0x00,0x00,0x02,0x20,0xFE,0x3F,0xFE,0x3F,0x82,0x20,0x80,0x00,0x80,0x00,
	0x80,0x00,0x80,0x00,0x82,0x20,0xFE,0x3F,0xFE,0x3F,0x02,0x20,0x00,0x00,0x00,0x20,0x00,0x30,0x00,0x3E,0x80,0x23,0x70,0x02,0x0E,0x02,0x0E,0x02,0xFC,0x22,0xE0,0x27,
	0x00,0x3F,0x00,0x30,0x00,0x20,0x00,0x00,0x02,0x20,0xFE,0x3F,0x0E,0x20,0x3C,0x00,0x70,0x00,0xE0,0x01,0x80,0x03,0x00,0x0F,0x02,0x1C,0xFE,0x3F,0x02,0x00,0x00,0x00,
	0x00,0x00,0x02,0x20,0xFE,0x3F,0xFE,0x3F,0x02,0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x30,0x00,0x3C,0x00,0x00,0x00,0x00,0x02,0x20,0xFE,0x3F,0xFE,0x3F,0x02,0x20,
	0x00,0x00,0x00,0x00,0x02,0x20,0xFE,0x3F,0x0E,0x20,0x3C,0x00,0x70,0x00,0xE0,0x01,0x80,0x03,0x00,0x0F,0x02,0x1C,0xFE,0x3F,0x02,0x00,0x00,0x00,0x00,0x00,0xE0,0x07,
	0xF8,0x1F,0x0C,0x18,0x06,0x30,0x02,0x20,0x02,0x20,0x02,0x20,0x02,0x21,0x06,0x1F,0x0C,0x3F,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x02,0x20,0xFE,0x3F,0xFE,0x3F,0x82,0x20,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x82,0x20,0xFE,0x3F,0xFE,0x3F,0x02,0x20,0x00,0x00,0x00,0x00,
	0x00,0x18,0xFE,0x30,0x46,0x20,0x46,0x20,0x46,0x20,0xC6,0x30,0x86,0x1F,0x07,0x0F,/*"E:\ShanLing\H5\Logo.bmp",0*/
};


const unsigned char BmpIcon[] = 
{
	0x00,0x00,0x01,0x00,0x00,0x80,0x03,0x00,0x00,0xC0,0x06,0x00,0x00,0xE0,0x0E,0x00,0x00,0x80,0x02,0x00,0x00,0x80,0x02,0x00,0x00,0x80,0x02,0x00,0x00,0x82,0x82,0x00,
	0x00,0x83,0x82,0x01,0x80,0xFF,0xFE,0x03,0xC0,0x00,0x00,0x06,0x80,0xFF,0xFE,0x03,0x00,0x83,0x82,0x01,0x00,0x82,0x82,0x00,0x00,0x80,0x02,0x00,0x00,0x80,0x02,0x00,
	0x00,0x80,0x02,0x00,0x00,0xE0,0x0E,0x00,0x00,0xC0,0x06,0x00,0x00,0x80,0x03,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xF0,0x01,
	0x00,0x0F,0xE0,0x01,0x00,0x1B,0xB0,0x01,0x00,0x37,0xD8,0x01,0x00,0x6D,0x6C,0x01,0x00,0xD8,0x36,0x00,0x00,0xB0,0x1B,0x00,0x00,0x60,0x0D,0x00,0x00,0xC0,0x06,0x00,
	0x00,0x60,0x0D,0x00,0x00,0xB0,0x1B,0x00,0x00,0xD8,0x36,0x00,0x00,0x6D,0x6C,0x01,0x00,0x37,0xD8,0x01,0x00,0x1B,0xB0,0x01,0x00,0x0F,0xE0,0x01,0x00,0x1F,0xF0,0x01,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"E:\ShanLing\H5\Ani.bmp",0*/
};



static void OLED_Fill_LogoData(unsigned char x,unsigned char y,const unsigned char *pRes,unsigned char Width)
{
	unsigned char i;
	unsigned long *pBuffer;
	unsigned long FontData;

	memset(pFrameC,0,512);
	pBuffer = pFrameC+x;
	
	for(i=0; i<Width; i++)
	{
		FontData = ((unsigned short*)pRes)[i];
		FontData <<= y;
		if((x+i) < 128)
		{
			pBuffer[i] |= FontData;
		}
	}
}

static void OLED_Draw_Icon(unsigned char x)
{
	unsigned char i;
	unsigned long *pBuffer;
	unsigned long* pBmpRes;
	unsigned long* pLogo;	
	unsigned char Temp;
	
	memset(pFrameA,0,512);
	pBuffer = ((unsigned long*)pFrameA)+x;
	pBmpRes = (unsigned long*)BmpIcon;
	
	Temp = x/8;
	if(Temp%2 == 0)
	{
		pBmpRes += 21;
	}
	for(i=0; i<21; i++)
	{
		if((x+i) < 128)
		{
			pBuffer[i] = pBmpRes[i];
		}
	}
	pLogo = pFrameC;
	
	for(i=0; i<x; i++)
	{
		if(i < 128)
		{
			pFrameA[i] = pLogo[128-x+i];
		}
	}
}


static void OLED_Logo(void)
{
	unsigned char i;
	unsigned char offset;

	if(rotate_flag == 0)
	{
		offset = -2;
	}
	else
	{
		offset = 2;
	}
	OLED_Fill_LogoData(4+offset,8,BmpLogo,120);
	for(i=0; i<=128; i++)
	{
		OLED_Draw_Icon(i);
		OLED_Update();
		mdelay(5);
	}
}


static long oled_ssd1312_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	oled_bitmap DrawData;
	oled_roll_rect rect;
	int ret;
	long val = 0;

	switch(cmd)
	{
	case 0x00:
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

	case 0x01:
		ret = get_user(val, (long *) arg);
		if(ret == 0)
		{
			if(val <= 0xff)
			{
				OLED_SetBacklight((unsigned char)val);
			}
			//printk("====oled====%d\n",val);
		}
		break;

	case 0x02:
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

	case 0x03:
		ret = copy_from_user((char*)&DrawData, (long *)arg, sizeof(oled_bitmap));
		if(ret == 0)
		{
			//printk("==oled==%d,%d,%d,%d\n",DrawData.X,DrawData.Y,DrawData.Len,DrawData.Page);
			OLED_UpdateData(&DrawData);
		}
		break;

	case 0x04:
		OLED_Logo();
		break;

	case 0x10:
		/*
		val = (long)pFrameA;
		if(copy_to_user((long *)arg,&val,4) == 0)
		{
			printk("====oled====pFrameA:0x%x\n",pFrameA);
		}
		*/
		copy_from_user((unsigned char*)pFrameA, (unsigned char *)arg, 512);
		break;

	case 0x11:
		/*
		val = (long)pFrameB;
		if(copy_to_user((long *)arg,&val,4) == 0)
		{
			printk("====oled====pFrameB:0x%x\n",pFrameB);
		}
		*/
		copy_from_user((unsigned char*)pFrameB, (unsigned char *)arg, 512);
		break;

	case 0x12:
		OLED_Update();
		//printk("====oled====Update\n");
		break;

	case 0x13:
		ret = copy_from_user((char*)&rect, (unsigned char *)arg, sizeof(oled_roll_rect));
		if(ret == 0)
		{
			//printk("====oled====%d,%d,%d,%d\n",rect.x_start,rect.x_end,rect.y_start,rect.y_end);
			OLED_RollRect(rect.x_start,rect.x_end,rect.y_start,rect.y_end,rect.mode);

		}
		break;

	case 0x14:
		OLED_LowRollUp();
		break;

	}
	return 0;
}

static struct file_operations oled_ssd1312_opt_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =	oled_ssd1312_ioctl,
};

static char driver_name[] = "oled_ssd1312";

static struct miscdevice oled_ssd1312_misc = {	
	.minor	=		MISC_DYNAMIC_MINOR,
	.name	=		(char *)driver_name,
	.fops	=		&oled_ssd1312_opt_fops, 
};


int ssd1312_mtdread_rotate()
{
	int i;
	int ret = 0;
	int ret_size = 0;
	u_char read_buf[16];
	struct mtd_info* mtdinfo = NULL;
	mtdinfo =  get_mtd_device_nm("bt_mac");
	if(mtdinfo == NULL || IS_ERR(mtdinfo))
	{
		return 0;
	}
	memset(read_buf,0,sizeof(read_buf));
	ret = mtd_read(mtdinfo,0,16,&ret_size,read_buf);
	if(ret != 0)
	{
		put_mtd_device(mtdinfo);
		return 0;
	}
	for(i=0;i<ret_size;i++)
	{
		if(read_buf[i] != 0xFF)
		{
			rotate_flag = 1;
			break;
		}
	}
	put_mtd_device(mtdinfo);
	return 1;
}


static int __init ssd1312_init(void)
{
	ssd1312_mtdread_rotate();
	OLED_Init();
	//*********rotate********
	if(rotate_flag == 1)
	{
		OLED_SetRotate(1);
	}
	else 
	{
		OLED_SetRotate(0);
	}
	//************************
	OLED_Logo();
	return misc_register(&oled_ssd1312_misc);
}

static void __exit ssd1312_exit(void)
{
	misc_deregister(&oled_ssd1312_misc);
}


late_initcall_sync(ssd1312_init);
module_exit(ssd1312_exit);
MODULE_LICENSE("GPL");

