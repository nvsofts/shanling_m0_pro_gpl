/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft6x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/ft6x06_ts.h>
#include <soc/gpio.h>
#include <jz_notifier.h>


/* After report early 10 points, skip report point(1/3). */
// #define DEBUG_SKIP_REPORT_POINT
// #define DEBUG_SKIP_POINT_DIVIDE_RATIO (3) /* only report 1/3 */


//#define FTS_CTL_IIC
//#define SYSFS_DEBUG
//#define FTS_APK_DEBUG
#define DEBUG_LCD_VCC_ALWAYS_ON
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "ft6x06_ex_fun.h"
#endif

#ifdef CONFIG_PRODUCT_X1000_M5S
#define FT6X06_FW_UP 
#include "m5sft6x06_fw.h"
#elif defined(CONFIG_PRODUCT_X1000_M2X)
#define FT6X06_FW_UP 
#include "m2xft6x06_fw.h"
#elif defined(CONFIG_PRODUCT_X1000_Q1)
#define FT6X06_FW_UP 
#include "q1ft6x06_fw.h"
#else
#define FT6X06_FW_UP 
#include "ft6x06_fw.h"
#endif

#ifdef FT6X06_FW_UP
#define	BIN_CHECKSUM	0xFF00//test
#define I2C_TRANSFER_WSIZE 256
#define	MAX_FLASH_SIZE	0xc000
#define	PJ_ID_OFFSET	0xcb


static unsigned char *fw_bin = NULL;
static int fw_size = 0;
static int fw_version=0;


enum BL6XXX_flash_cmd {
	
	ERASE_SECTOR_MAIN_CMD	= 0X06,
	ERASE_ALL_MAIN_CMD	= 0X09,	
	RW_REGISTER_CMD		= 0X0a,
	READ_MAIN_CMD		= 0X0D,
	WRITE_MAIN_CMD		= 0X0F,
	WRITE_RAM_CMD		= 0X11,
	READ_RAM_CMD		= 0X12,
};

enum fw_reg {

	WORK_MODE_REG		= 0X00,
	CHECKSUM_REG		= 0x3f,
	CHECKSUM_CAL_REG	= 0x8a,
	AC_REG			= 0X8b,
	RESOLUTION_REG		= 0X98,
	LPM_REG			= 0Xa5,
	PROXIMITY_REG		= 0Xb0,
	PROXIMITY_FLAG_REG	= 0XB1,
	PJ_ID_REG		= 0Xb5,
};

enum checksum {
	CHECKSUM_READY	= 0x01,
	CHECKSUM_CAL	= 0xaa,
};

#endif

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u8 au8_touch_wight[CFG_MAX_TOUCH_POINTS];	/*touch Wight */
	u16 pressure;
	u8 touch_point;
};

struct i2c_client *gloalts_client = NULL;



struct ft6x06_ts_data {
	unsigned int irq;
	unsigned int irq_pin;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int va_x_max;
	unsigned int va_y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft6x06_platform_data *pdata;
	struct work_struct  work;
	struct workqueue_struct *workqueue;
	struct regulator *vcc_reg;

#ifdef DEBUG_SKIP_REPORT_POINT
	unsigned int report_count;
#endif
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

unsigned char key_home_status = 0;
unsigned char key_back_status = 0;
unsigned char key_menu_status = 0;


//*************添加版本显示**************
static ssize_t
tp_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf,"%hu",fw_version);
}

static ssize_t
tp_version_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t n)
{
        return n;
}


static DEVICE_ATTR(tp_version, 0644, tp_version_show, tp_version_store);
//***************************************
/*
*ft6x06_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*/
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/*release the point*/
static void ft6x06_ts_release(struct ft6x06_ts_data *data)
{
#ifdef DEBUG_SKIP_REPORT_POINT
	data->report_count = 0;
#endif
#ifdef CONFIG_FT6X06_MULTITOUCH
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	if(1 == key_menu_status){
		input_event(data->input_dev,EV_KEY,KEY_MENU,0);
		key_menu_status = 0;
	}
	if(1 == key_home_status){
		input_event(data->input_dev,EV_KEY,KEY_HOMEPAGE,0);
		key_home_status = 0;
	}
	if(1 == key_back_status){
		input_event(data->input_dev,EV_KEY,KEY_BACK,0);
		key_back_status = 0;
	}

	input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
#else
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_sync(data->input_dev);
#endif
}

static int ft6x06_read_Touchdata(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft6x06_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);

	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[2] & 0x0F;
	if (event->touch_point == 0) {
		ft6x06_ts_release(data);
		return 1;
	}

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		event->au8_touch_wight[i] =
			(buf[FT_TOUCH_WEIGHT_POS + FT_TOUCH_STEP * i]);
	}

	event->pressure = FT_PRESS;

	return 0;
}

static void ft6x06_report_value(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;

	int i = 0;
	for (i = 0; i < event->touch_point; i++) {
		/* LCD view area */
		if (event->au16_x[i] < data->va_x_max
		    && event->au16_y[i] < data->va_y_max) {
#ifdef DEBUG_SKIP_REPORT_POINT
			int report_count;
			report_count = data->report_count;
			data->report_count++;
			/* After report early 10 points, skip report point(1/3). */
			if ( (report_count < 10) || ( report_count % DEBUG_SKIP_POINT_DIVIDE_RATIO == 0) ) {
#endif	/* DEBUG_SKIP_REPORT_POINT */

#ifdef CONFIG_FT6X06_MULTITOUCH
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					event->au16_y[i]);
			input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR,
					event->au8_touch_wight[i]);
			input_report_abs(data->input_dev,ABS_MT_WIDTH_MAJOR,
					event->au8_touch_wight[i]);
			input_mt_sync(data->input_dev);

#else
			if(0 == i){
				s16 convert_x = 0;
				s16 convert_y = 0;
				convert_x = event->au16_x[0];
				convert_y = event->au16_y[0];
				if (event->touch_point == 1) {
					input_report_abs(data->input_dev, ABS_X, (u16)convert_x);
					input_report_abs(data->input_dev, ABS_Y, (u16)convert_y);
					input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
				}
				input_report_key(data->input_dev, BTN_TOUCH, 1);
			}
#endif
#ifdef DEBUG_SKIP_REPORT_POINT
			}
#endif /* DEBUG_SKIP_REPORT_POINT */
		}
		/*Virtual key*/
		else{
#ifdef CONFIG_FT6X06_MULTITOUCH

			if ((event->au8_touch_event[i] == FTS_POINT_DOWN)
					|| (event->au8_touch_event[i] == FTS_POINT_CONTACT)) {
				if(event->au16_y[i] >= data->va_y_max
						&& event->au16_y[i] <= data->y_max) {
					/*menu key*/
					if (event->au16_x[i] >= 0 && event->au16_x[i] < 100){
						if(0 == key_menu_status){
							input_event(data->input_dev,EV_KEY,KEY_MENU,1);
							key_menu_status = 1;
						}
					}
					/*home key*/
					if (event->au16_x[i] >= 100 && event->au16_x[i] < 200){
						if(0 == key_home_status){
							input_event(data->input_dev,EV_KEY,KEY_HOMEPAGE,1);
							key_home_status = 1;
						}
					}
					/*back key*/
					if (event->au16_x[i] >= 200 && event->au16_x[i] < 300){
						if(0 == key_back_status){
							input_event(data->input_dev,EV_KEY,KEY_BACK,1);
							key_back_status = 1;
						}
					}
				}
			}else{
				if(1 == key_menu_status){
					input_event(data->input_dev,EV_KEY,KEY_MENU,0);
					key_menu_status = 0;
				}
				if(1 == key_home_status){
					input_event(data->input_dev,EV_KEY,KEY_HOMEPAGE,0);
					key_home_status = 0;
				}
				if(1 == key_back_status){
					input_event(data->input_dev,EV_KEY,KEY_BACK,0);
					key_back_status = 0;
				}
			}
#else
#endif
		}
	}

	//printk("$ly-test----%s: x1:%d y1:%d |<*_*>| x2:%d y2:%d \n", __func__,event->au16_x[0], event->au16_y[0],event->au16_x[1], event->au16_y[1]);
	
	input_sync(data->input_dev);
}

static void ft6x06_work_handler(struct work_struct *work)
{
	struct ft6x06_ts_data *ft6x06_ts = container_of(work, struct ft6x06_ts_data, work);
	int ret = 0;
	if(gpio_get_value(ft6x06_ts->pdata->reset) == 1)
	{
		ret = ft6x06_read_Touchdata(ft6x06_ts);
		if (ret == 0)
			ft6x06_report_value(ft6x06_ts);
	}
	enable_irq(ft6x06_ts->irq);
}

/*The ft6x06 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *ft6x06_ts = dev_id;
//	jz_notifier_call(NOTEFY_PROI_NORMAL, JZ_CLK_CHANGING, NULL);
	disable_irq_nosync(ft6x06_ts->irq);

#if 0
	if (ft6x060_ts->is_suspend)
		return IRQ_HANDLED;
#endif

	if (!work_pending(&ft6x06_ts->work)) {
		queue_work(ft6x06_ts->workqueue, &ft6x06_ts->work);
	} else {
		enable_irq(ft6x06_ts->irq);
	}

	return IRQ_HANDLED;
}

static void ft6x06_ts_reset(struct ft6x06_ts_data *ts)
{
	//printk("[FTS] ft6x06_ts_reset()\n");
	if(ts->pdata->reset >= 0) {
		gpio_set_value(ts->pdata->reset, 1);
		msleep(5);
		gpio_set_value(ts->pdata->reset, 0);
		msleep(10);
		gpio_set_value(ts->pdata->reset, 1);
		msleep(15);
	}

}

#ifdef FT6X06_FW_UP
static int bl_erase_flash(void)
{
	unsigned char cmd[2];
	int ret = -1;
	
	cmd[0] = ERASE_ALL_MAIN_CMD; 
	cmd[1] = ~cmd[0];

	if(gloalts_client){
		gloalts_client->addr = 0x2c;
		ret = ft6x06_i2c_Write(gloalts_client,cmd,2);
		gloalts_client->addr = 0x38;
		printk("bl6xx0 bl_erase_flash ret=%d\n",ret);
	}
	return ret;
}

static int bl_write_flash(unsigned char cmd,unsigned short flash_start_addr, unsigned char *buf, int len)
{
	unsigned char cmd_buf[6+I2C_TRANSFER_WSIZE];
	unsigned short flash_end_addr;
	int ret = 0;	
	
	if(!len){
		printk("bl6xx0 ___write flash len is 0x00,return___\n");
		return -1;	
	}

	flash_end_addr = flash_start_addr + len - 1;

	if(flash_end_addr >= MAX_FLASH_SIZE){
		printk("bl6xx0 ___write flash end addr is overflow,return___\n");
		return -1;	
	}

	cmd_buf[0] = cmd;
	cmd_buf[1] = ~cmd;
	cmd_buf[2] = flash_start_addr >> 0x08;
	cmd_buf[3] = flash_start_addr & 0xff;
	cmd_buf[4] = flash_end_addr >> 0x08;
	cmd_buf[5] = flash_end_addr & 0xff;

	memcpy(&cmd_buf[6],buf,len);
	
	if(gloalts_client){
	  	gloalts_client->addr = 0x2c;
		ret = ft6x06_i2c_Write(gloalts_client,cmd_buf,len+6);
		//printk("bl6xx0 __%s:ret = %d\n",__func__,ret);
		gloalts_client->addr = 0x38;
	}

	if(ret < 0){
		printk("bl6xx0 ___%s:i2c transfer error___\n",__func__);
		return -1;
	}

	return 0;
}


static int bl_download_fw(void)
{
	unsigned int i;
	unsigned short size,len;
	unsigned short addr;
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(gloalts_client);
	bl_erase_flash();
	mdelay(50);
	gpio_direction_output(ft6x06_ts->pdata->irq,1);
	mdelay(10);
	gpio_direction_output(ft6x06_ts->pdata->irq,0);
	mdelay(10);
	
	for(i=0;i<fw_size;)
	{
			size = fw_size - i;
			//size = FW_SIZE - i;
			if(size > I2C_TRANSFER_WSIZE){
				len = I2C_TRANSFER_WSIZE;
			}else{
				len = size;
			}
			addr = i;
//			printk("bl6xx0 bl_download_fw len=%d fw_bin[%d]=0x%x\n",len,i,fw_bin[i]);
			if(bl_write_flash(WRITE_MAIN_CMD,addr, (unsigned char *)&fw_bin[i],len)){
				return -1;
			}
	
			mdelay(10);
			i += len;
			
	}	
	return 0;
}


static int bl_get_fw_checksum(unsigned short *fw_checksum)
{
	unsigned char buf[3];
	unsigned char checksum_ready = 0;
	int retry = 5;
	int ret = 0x00;
	unsigned char addr = 0x00;	

	buf[0] = CHECKSUM_CAL_REG;
	buf[1] = CHECKSUM_CAL;

	ret = ft6x06_i2c_Write(gloalts_client,&buf[0],2);
	if(ret < 0){
		printk("___%s:write checksum cmd error_0__\n",__func__);
		return -1;
	}

	msleep(250);

	addr = CHECKSUM_REG;	
	ret = ft6x06_i2c_Read(gloalts_client,&addr, 1, buf, 3);
	if(ret < 0){
		printk("___%s:read checksum error_1__\n",__func__);
		return -1;
	}

	checksum_ready = buf[0];

	while((retry--) && (checksum_ready != CHECKSUM_READY))
	{
		msleep(50);
		ret = ft6x06_i2c_Read(gloalts_client,&addr, 1, buf, 3);
		if(ret < 0){
			printk("___%s:retry= %d read checksum error_2__\n",__func__,retry);
			return -1;
		}

		checksum_ready = buf[0];
	}
	
	if(checksum_ready != CHECKSUM_READY){
		printk("___%s:read checksum fail_3__\n",__func__);
		return -1;
	}
	*fw_checksum = (buf[1]<<8)+buf[2];

	return 0;
}


int bl_update_flash(u8 *pfwbin,int fwsize)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(gloalts_client);
	unsigned short fw_checksum = !BIN_CHECKSUM;
	int retry;
	int ret = 0;
	fw_bin = pfwbin;
	fw_size = fwsize;
	gpio_set_value(ft6x06_ts->pdata->reset,1);
	
	retry =3;
	while((retry--) && (fw_checksum != BIN_CHECKSUM))
	{
		gpio_direction_output(ft6x06_ts->pdata->irq,0);
		mdelay(10);
		ret = bl_download_fw();
		mdelay(20);
		gpio_direction_output(ft6x06_ts->pdata->irq,1);
		mdelay(10);

		if(ret<0){
			continue;
		}else{
			break;
		}
		bl_get_fw_checksum(&fw_checksum);					
	}
	return 0;
}


int bl_update_fw()
{
	struct mutex i2c_lock;	
	int ret = 0;
	unsigned char fw_id = 0x00;
	unsigned char uc_reg_addr = 0xb5;
	mutex_init(&i2c_lock);
	ret = ft6x06_i2c_Read(gloalts_client, &uc_reg_addr, 1, &fw_id, 1);
	printk("======ftret[%d]====fw_id[%d]===fwbin[PJ_ID_OFFSET][%d]===\n",ret,fw_id,fwbin[PJ_ID_OFFSET]);
	if( fw_id < fwbin[PJ_ID_OFFSET])
	{
		local_irq_disable();
		mutex_lock(&i2c_lock);
		bl_update_flash((u8 *)fwbin, sizeof(fwbin));
		mutex_unlock(&i2c_lock);
		local_irq_enable();
		msleep(50);
		ft6x06_i2c_Read(gloalts_client, &uc_reg_addr, 1, &fw_id, 1);
	}
	fw_version = fw_id;
	return 0;
}	

#endif

static void ft6x06_close(struct input_dev *dev)
{
	struct ft6x06_ts_data *ts = input_get_drvdata(dev);

	dev_dbg(&ts->client->dev, "[FTS]ft6x06 suspend\n");
	disable_irq(ts->pdata->irq);
}

static int ft6x06_open(struct input_dev *dev)
{
	struct ft6x06_ts_data *ts = input_get_drvdata(dev);

	dev_dbg(&ts->client->dev, "[FTS]ft6x06 resume.\n");
	ft6x06_ts_reset(ts);
	enable_irq(ts->pdata->irq);
	return 0;
}

static int ft6x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft6x06_platform_data *pdata =
	    (struct ft6x06_platform_data *)client->dev.platform_data;
	struct ft6x06_ts_data *ft6x06_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft6x06_ts = kzalloc(sizeof(struct ft6x06_ts_data), GFP_KERNEL);

	if (!ft6x06_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ft6x06_ts);
	gloalts_client = client;
	
	ft6x06_ts->irq_pin = pdata->irq;
	ft6x06_ts->irq = gpio_to_irq(pdata->irq);
	client->irq = ft6x06_ts->irq;
	ft6x06_ts->client = client;
	ft6x06_ts->pdata = pdata;
	ft6x06_ts->x_max = pdata->x_max - 1;
	ft6x06_ts->y_max = pdata->y_max - 1;
	ft6x06_ts->va_x_max = pdata->va_x_max - 1;
	ft6x06_ts->va_y_max = pdata->va_y_max - 1;

	if(pdata->reset >= 0) {
		err = gpio_request(pdata->reset, "ft6x06 reset");
			if (err < 0) {
				dev_err(&client->dev, "%s:failed to set gpio reset.\n",
					__func__);
				goto exit_request_fail;
			}
			gpio_direction_output(pdata->reset, 1);
	}

	if(pdata->power >= 0) {
		err = gpio_request(pdata->power, "ft6x06 power");
			if (err < 0) {
				dev_err(&client->dev, "%s:failed to set gpio power.\n",
					__func__);
				goto exit_request_fail;
			}
			gpio_direction_output(pdata->power, pdata->power_level_en);
	}

	err = gpio_request(pdata->irq,"ft6x06 irq");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio irq.\n",
			__func__);
		goto exit_request_fail;
	}
	gpio_direction_input(pdata->irq);
#ifndef DEBUG_LCD_VCC_ALWAYS_ON
	ft6x06_ts->vcc_reg = regulator_get(NULL, "vlcd");
	if (IS_ERR(ft6x06_ts->vcc_reg)) {
		dev_err(&client->dev, "failed to get VCC regulator.");
		err = PTR_ERR(ft6x06_ts->vcc_reg);
		goto exit_request_reset;
	}
	regulator_enable(ft6x06_ts->vcc_reg);
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft6x06_ts->input_dev = input_dev;

#ifdef CONFIG_FT6X06_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft6x06_ts->va_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft6x06_ts->va_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, ft6x06_ts->va_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ft6x06_ts->va_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);

	set_bit(KEY_HOMEPAGE, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name = FT6X06_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft6x06_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	input_dev->open = ft6x06_open;
	input_dev->close = ft6x06_close;
	input_set_drvdata(input_dev, ft6x06_ts);
	/*make sure CTP already finish startup process */
	ft6x06_ts_reset(ft6x06_ts);
	msleep(150);

#ifdef FT6X06_FW_UP
	bl_update_fw();	
	gpio_direction_input(pdata->irq);
#endif
	if (0) {

		/*get some register information */
		uc_reg_addr = FT6x06_REG_FW_VER;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

		uc_reg_addr = FT6x06_REG_POINT_RATE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	/* Try to slow down point rate, Failed. */
		char write_buf[2];
		write_buf[0] = FT6x06_REG_POINT_RATE;
		write_buf[1] = 1;
		ft6x06_i2c_Write(client, &write_buf[0], 2);

		uc_reg_addr = FT6x06_REG_POINT_RATE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_err(&client->dev, "[FTS] report rate is %dHz.\n",
			uc_reg_value * 10);


		uc_reg_addr = 0x89;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_err(&client->dev, "[FTS] report rate 89is %dHz.\n",
			uc_reg_value * 10);

		write_buf[0] = 0x89;
		write_buf[1] = 1;
		ft6x06_i2c_Write(client, &write_buf[0], 2);

		uc_reg_addr = 0x89;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_err(&client->dev, "[FTS] report rate 89is %dHz.\n",
			uc_reg_value * 10);

		uc_reg_addr = FT6x06_REG_THGROUP;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
			uc_reg_value * 4);

		uc_reg_addr = FT6x06_REG_D_MODE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] DEVICE_MODE = 0x%x\n", uc_reg_value);

		uc_reg_addr = FT6x06_REG_G_MODE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] G_MODE = 0x%x\n", uc_reg_value);

		uc_reg_addr = FT6x06_REG_P_MODE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] POWER_MODE = 0x%x\n", uc_reg_value);

		uc_reg_addr = FT6x06_REG_STATE;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] STATE = 0x%x\n", uc_reg_value);

		uc_reg_addr = FT6x06_REG_CTRL;
		ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		dev_dbg(&client->dev, "[FTS] CTRL = 0x%x\n", uc_reg_value);
	}

#ifdef SYSFS_DEBUG
	ft6x06_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

	INIT_WORK(&ft6x06_ts->work, ft6x06_work_handler);
	ft6x06_ts->workqueue = create_singlethread_workqueue("ft6x06_tsc");

	err = request_irq(ft6x06_ts->irq, ft6x06_ts_interrupt,
			pdata->irqflags, client->dev.driver->name,
			ft6x06_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft6x06_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(ft6x06_ts->irq);
	jzgpio_ctrl_pull(pdata->irq / 32, 0, BIT(pdata->irq % 32));
	enable_irq(ft6x06_ts->irq);
	//***********添加TP版本**************
	 err = device_create_file(&client->dev, &dev_attr_tp_version);
	 if (err != 0) {
	 dev_err(&client->dev,  "Failed to create xxx sysfs files: %d\n", err);
	}
	 //********************************
	return 0;

exit_irq_request_failed:

exit_input_register_device_failed:
	input_free_device(input_dev);

#ifndef DEBUG_LCD_VCC_ALWAYS_ON
exit_request_reset:
#endif
	if(ft6x06_ts->pdata->reset >= 0)
		gpio_free(ft6x06_ts->pdata->reset);
exit_request_fail:
exit_input_dev_alloc_failed:
#ifndef DEBUG_LCD_VCC_ALWAYS_ON
	if (!IS_ERR(ft6x06_ts->vcc_reg)) {
		regulator_disable(ft6x06_ts->vcc_reg);
		regulator_put(ft6x06_ts->vcc_reg);
	}
#endif
	i2c_set_clientdata(client, NULL);
	kfree(ft6x06_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	input_unregister_device(ft6x06_ts->input_dev);
	if(ft6x06_ts->pdata->reset >= 0)
		gpio_free(ft6x06_ts->pdata->reset);
	if(ft6x06_ts->pdata->power >= 0)
		gpio_free(ft6x06_ts->pdata->power);

	device_remove_file(&client->dev, &dev_attr_tp_version);
#ifdef SYSFS_DEBUG
	ft6x06_release_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
	free_irq(client->irq, ft6x06_ts);
#ifndef DEBUG_LCD_VCC_ALWAYS_ON
	if (!IS_ERR(ft6x06_ts->vcc_reg)) {
		regulator_disable(ft6x06_ts->vcc_reg);
		regulator_put(ft6x06_ts->vcc_reg);
	}
#endif
	kfree(ft6x06_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

int ft6x06_ts_switch(int mode)
{
	struct ft6x06_ts_data *ft6x06_ts;
	unsigned char us_reg_addr;
	unsigned char uc_reg_value;
	int ret=0;
	ft6x06_ts = i2c_get_clientdata(gloalts_client);
	if(mode == 1) //poweron
	{
		if(ft6x06_ts->pdata->reset >= 0)
		{
			gpio_set_value(ft6x06_ts->pdata->reset,1);
			msleep(10);
		}
		if(ft6x06_ts->pdata->power >= 0)
			gpio_set_value(ft6x06_ts->pdata->power, ft6x06_ts->pdata->power_level_en);
		msleep(10);
		us_reg_addr=0x01;
		int retry=4;
		while(retry--)
		{
			ret = ft6x06_i2c_Read(gloalts_client,&us_reg_addr,1,&uc_reg_value,1);
			if(ret >= 0)	break;
			msleep(10);
		}	
		if(ret < 0)
		{
			//poweroff
			if(ft6x06_ts->pdata->power >= 0)
				gpio_set_value(ft6x06_ts->pdata->power, !ft6x06_ts->pdata->power_level_en);
			msleep(50);
			if(ft6x06_ts->pdata->reset >= 0)
				gpio_set_value(ft6x06_ts->pdata->reset,0);	
			msleep(100);
			//poweron
			if(ft6x06_ts->pdata->reset >= 0)
			{
				gpio_set_value(ft6x06_ts->pdata->reset,1);
				msleep(50);
			}
			if(ft6x06_ts->pdata->power >= 0)
				gpio_set_value(ft6x06_ts->pdata->power, ft6x06_ts->pdata->power_level_en);
			msleep(50);
			us_reg_addr=0x01;
			retry=4;
			while(retry--)
			{
				ret = ft6x06_i2c_Read(gloalts_client,&us_reg_addr,1,&uc_reg_value,1);
				if(ret >= 0)	break;
				msleep(50);
			}	
		}		
	}
	else  //poweroff
	{
		if(ft6x06_ts->pdata->power >= 0)
			gpio_set_value(ft6x06_ts->pdata->power, !ft6x06_ts->pdata->power_level_en);
		msleep(10);
		if(ft6x06_ts->pdata->reset >= 0)
			gpio_set_value(ft6x06_ts->pdata->reset,0);	
	}
	return 0;
}


static int ft6x06_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	
	//ft6x06_ts_switch(0);
	return 0;
}

static int ft6x06_ts_resume(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	//ft6x06_ts_switch(1);
	return 0;
}


static const struct i2c_device_id ft6x06_ts_id[] = {
	{FT6X06_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);

static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.suspend = ft6x06_ts_suspend,
	.resume = ft6x06_ts_resume,
	.id_table = ft6x06_ts_id,
	.driver = {
		.name = FT6X06_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ft6x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft6x06_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft6x06 driver failed "
				"(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
				ft6x06_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}

#if defined(CONFIG_PRODUCT_X1000_M5S) || defined(CONFIG_PRODUCT_X1000_M2X) || defined(CONFIG_PRODUCT_X1000_Q1)
device_initcall_sync(ft6x06_ts_init);
#else
module_init(ft6x06_ts_init);
#endif
module_exit(ft6x06_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL");
