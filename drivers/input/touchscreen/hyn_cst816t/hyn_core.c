	  
/*
 *
 * hyntrion TouchScreen driver.
 *
 * Copyright (c) 2012-2019, hyntrion Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: hyn_core.c
*
* Author: hyntrion Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for hyntrion ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include "hyn_core.h"
#include "hyn_common.h"
#include "i2cdev.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define HYN_DRIVER_NAME                     "hyn_ts"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */
#if HYN_POWER_SOURCE_CUST_EN
#define HYN_VTG_MIN_UV                      2800000
#define HYN_VTG_MAX_UV                      3300000
#define HYN_I2C_VTG_MIN_UV                  1800000
#define HYN_I2C_VTG_MAX_UV                  1800000
#endif

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct hyn_ts_data *hyn_data;
extern struct i2c_client *hi_client;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int hyn_ts_suspend(struct device *dev);
static int hyn_ts_resume(struct device *dev);

static int hyn_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
    int ret=0;
    mutex_lock(&hyn_data->bus_lock);
    ret = hi_i2c_read(*cmd,cmdlen,data,datalen);
    mutex_unlock(&hyn_data->bus_lock);
    return ret;
}

static int hyn_read_reg(u8 addr, u8 *value)
{
    return hyn_read(&addr, 1, value, 1);
}

//#ifdef CONFIG_HAS_EARLYSUSPEND
static int hyn_write(u8 *writebuf, u32 writelen)
{
	int ret;
    mutex_lock(&hyn_data->bus_lock);
    ret = hi_i2c_write(writebuf[0],1,writebuf[1],1);
    mutex_unlock(&hyn_data->bus_lock);
    return ret;
}

static int hyn_write_reg(u8 addr, u8 value)
{
    u8 buf[2] = { 0 };

    buf[0] = addr;
    buf[1] = value;
    return hyn_write(buf, sizeof(buf));
}
//#endif

static int hyn_bus_exit(struct hyn_ts_data *ts_data)
{
    HYN_FUNC_ENTER();
    HYN_FUNC_EXIT();
    return 0;
}

static int hyn_reset_proc(int hdelayms)
{
	gpio_direction_output(hyn_data->pdata->reset_gpio,0);
	mdelay(20);
	gpio_direction_output(hyn_data->pdata->reset_gpio,1);
	mdelay(hdelayms);
    	return 0;
}

/*****************************************************************************
*  Name: hyn_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
static int hyn_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;
    u8 chip_id = hyn_data->ic_info.ids.chip_id;

    do {
        ret = hyn_read_reg(HYN_REG_CHIP_ID, &reg_value);
	
	
        if ((ret < 0) || (reg_value != chip_id))// 
	    {
            HYN_DEBUG("TP Not Ready, ReadData = 0x%x", reg_value);
        } else if (reg_value == chip_id) {
            HYN_INFO("TP Ready, Device ID = 0x%x", reg_value);
            return 0;
        }
        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return ret;
}

/*****************************************************************************
*  Name: hyn_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void hyn_tp_state_recovery(struct hyn_ts_data *ts_data)
{
    HYN_FUNC_ENTER();
    /* wait tp stable */
    hyn_wait_tp_to_valid();
    HYN_FUNC_EXIT();
}

static void hyn_irq_disable(void)
{
    unsigned long irqflags;

    HYN_FUNC_ENTER();
    spin_lock_irqsave(&hyn_data->irq_lock, irqflags);

    if (!hyn_data->irq_disabled) {
        disable_irq_nosync(hyn_data->irq);
        hyn_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&hyn_data->irq_lock, irqflags);
    HYN_FUNC_EXIT();
}

static void hyn_irq_enable(void)
{
    unsigned long irqflags = 0;

    HYN_FUNC_ENTER();
    spin_lock_irqsave(&hyn_data->irq_lock, irqflags);

    if (hyn_data->irq_disabled) {
        enable_irq(hyn_data->irq);
        hyn_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&hyn_data->irq_lock, irqflags);
    HYN_FUNC_EXIT();
}

/*****************************************************************************
* Name: hyn_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not hyntrion's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int hyn_get_ic_information(struct hyn_ts_data *ts_data)
{
    int ret = 0;
    int cnt = 0;
    u8 chip_id[2] = { 0 };

    ts_data->ic_info.is_incell = HYN_CHIP_IDC;
    ts_data->ic_info.hid_supported = HYN_HID_SUPPORTTED;

    do {
        ret = hyn_read_reg(HYN_REG_CHIP_ID, &chip_id[0]);
        ret |= hyn_read_reg(HYN_REG_FW_VER, &chip_id[1]);
        if (ret < 0) {
            HYN_DEBUG("i2c read invalid, read:0x%02x%02x",
                      chip_id[0], chip_id[1]);
        } else {
			ts_data->ic_info.ids.chip_id = chip_id[0];
            ts_data->ic_info.ids.fw_ver = chip_id[1];
			break;
        }

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
        HYN_INFO("read chip_ID fail !!!");
		ts_data->ic_info.ids.chip_id = 0xFF;
        ts_data->ic_info.ids.fw_ver = 0;    //to set low FWver waite behand autoupdata
    }
    
    printk("chip_ID = 0x%02x, FW_ver = 0x%02x",chip_id[0], chip_id[1]);
    return 0;
}

#if HYN_FW_UPDATA
#include "capacitive_hynitron_cst8xx_update.h"
#include "capacitive_hynitron_cst816t_update.h"
//extern unsigned char app_bin[];
static unsigned char *p_cst836u_upgrade_firmware;

#define REG_LEN_1B    1
#define REG_LEN_2B    2

static int hctp_write_bytes(unsigned short reg,unsigned char *buf,unsigned short len,unsigned char reg_len){
    int ret;
	ret = update_i2c_write(reg,reg_len,buf,len);

    return ret;
}

static int hctp_read_bytes(unsigned short reg,unsigned char* buf,unsigned short len,unsigned char reg_len){
    int ret;
    ret = update_i2c_read(reg,reg_len,buf,len);

    return ret;
}

static int cst78xx_enter_bootmode(void)
{
     char retryCnt = 10;

	hyn_reset_proc(10);
     while(retryCnt--){
         u8 cmd[3];
         cmd[0] = 0xAB;
         if (-1 == hctp_write_bytes(0xA001,cmd,1,REG_LEN_2B)){  // enter program mode
             mdelay(2); // 4ms
             continue;                   
         }
         if (-1 == hctp_read_bytes(0xA003,cmd,1,REG_LEN_2B)) { // read flag
             mdelay(2); // 4ms
             continue;                           
         }else{
             if (cmd[0] != 0xC1){
                 msleep(2); // 4ms
                 continue;
             }else{
                 return 0;
             }
         }
     }
     return -1;
}

static u32 cst78xx_read_checksum(u16 startAddr,u16 len){
    union{
        u32 sum;
        u8 buf[4];
    }checksum;
    char cmd[3];
    char readback[4] = {0};

    if (cst78xx_enter_bootmode() == -1){
	HYN_ERROR("enter boot mode faile!!!");
       return -1;
    }
    
    cmd[0] = 0;
    if (-1 == hctp_write_bytes(0xA003,cmd,1,REG_LEN_2B)){
        return -1;
    }
    msleep(500);
    
    if (-1 == hctp_read_bytes(0xA000,readback,1,REG_LEN_2B)){
        return -1;   
    }
    if (readback[0] != 1){
        return -1;
    }
    if (-1 == hctp_read_bytes(0xA008,checksum.buf,4,REG_LEN_2B)){
        return -1;
    }
	
    return checksum.sum;
}

static int cst78xx_update(u16 startAddr,u16 len,u8* src){
    u16 sum_len;
    u8 cmd[10];
	int ret;

	ret = 0;
	
    if (cst78xx_enter_bootmode() == -1){
	    HYN_ERROR("enter boot mode faile!");
       return -1;
    }
    sum_len = 0;
	
 #define PER_LEN	512
    do{
        if (sum_len >= len){
            return -1;
        }
        
        // send address
        if(startAddr>0x3C00L){
            HYN_ERROR("write addr erro!!");
            return -2;
        }
        cmd[1] = startAddr>>8;
        cmd[0] = startAddr&0xFF;
        hctp_write_bytes(0xA014,cmd,2,REG_LEN_2B);
 
#if HYN_MTK_IIC_TRANSFER_LIMIT
	{       
		u8 temp_buf[8];
		u16 j,iic_addr;
		iic_addr=0;
		for(j=0; j<128; j++){
			
	    	temp_buf[0] = *((u8*)src+iic_addr+0);
	    	temp_buf[1] = *((u8*)src+iic_addr+1);
			temp_buf[2] = *((u8*)src+iic_addr+2);
			temp_buf[3] = *((u8*)src+iic_addr+3);

	    	hctp_write_bytes((0xA018+iic_addr),(u8* )temp_buf,4,REG_LEN_2B);
			iic_addr+=4;
			if(iic_addr==512) break;
		}

	}
#else
		hctp_write_bytes(0xA018,src,PER_LEN,REG_LEN_2B);
#endif 	
        cmd[0] = 0xEE;
        hctp_write_bytes(0xA004,cmd,1,REG_LEN_2B);
		
		mdelay(100);
		
        {
            u8 retrycnt = 50;
            while(retrycnt--){
                cmd[0] = 0;
                hctp_read_bytes(0xA005,cmd,1,REG_LEN_2B);
                if (cmd[0] == 0x55){
                    // success 
                    break;
                }
                msleep(10);
            }

			if(cmd[0]!=0x55)
			{
				ret = -1;
			}
        }
        startAddr += PER_LEN;
        src       += PER_LEN;
        sum_len   += PER_LEN;
    }while(len);
    
    // exit program mode
    cmd[0] = 0x00;
    hctp_write_bytes(0xA003,cmd,1,REG_LEN_2B);
	
	return ret;
}

int hyn_ctpm_fw_upgrade_with_i_file(struct hyn_ts_data *ts_data)
{
    unsigned short startAddr;
    unsigned short length;
    unsigned short checksum; 
	unsigned short chipchecksum;

	if(ts_data->ic_info.ids.chip_id == 0xAA)
	{
		p_cst836u_upgrade_firmware=(unsigned char *)app_bin_new;
	}
	else
	{
		p_cst836u_upgrade_firmware=(unsigned char *)app_bin;
	}
	startAddr = *(p_cst836u_upgrade_firmware+1);
	length =*(p_cst836u_upgrade_firmware+3);
	checksum = *(p_cst836u_upgrade_firmware+5);
	startAddr <<= 8; 
	startAddr |= *(p_cst836u_upgrade_firmware+0);
	length <<= 8; 
	length |= *(p_cst836u_upgrade_firmware+2);
	checksum <<= 8; 
	checksum |= *(p_cst836u_upgrade_firmware+4);

	chipchecksum = cst78xx_read_checksum(startAddr, length);
	printk("===chipid[%X]===chipchecksum[0x%04X]=====checksum[0x%04X]===\n",ts_data->ic_info.ids.chip_id,chipchecksum,checksum);
	if(chipchecksum != checksum)
	{
		cst78xx_update(startAddr, length, (p_cst836u_upgrade_firmware+6));

		chipchecksum = cst78xx_read_checksum(startAddr, length);

	    printk("\r\nCTP cst78xx update %s, checksum-0x%04x \n",((chipchecksum==checksum) ? "success" : "fail"),chipchecksum);
	}
    return 0;
}

#if 0
unsigned char hyn_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
	
	p_cst836u_upgrade_firmware=(unsigned char *)app_bin;
	
    ui_sz = sizeof(app_bin);
    if (ui_sz > 2)
    {
        return *(p_cst836u_upgrade_firmware+0x3BFC+6);
    }
    else
        return 0; 
}
#endif
#endif

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void hyn_show_touch_buffer(u8 *data, int datalen)
{
    int i = 0;
    int count = 0;
    char *tmpbuf = NULL;

    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        HYN_ERROR("tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    HYN_DEBUG("point buffer:%s", tmpbuf);

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

static void hyn_release_all_finger(void)
{
    struct input_dev *input_dev = hyn_data->input_dev;
#if HYN_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = hyn_data->pdata->max_touch_number;
#endif

    HYN_FUNC_ENTER();
    mutex_lock(&hyn_data->report_mutex);
#if HYN_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

    hyn_data->touchs = 0;
    hyn_data->key_state = 0;
    mutex_unlock(&hyn_data->report_mutex);
    HYN_FUNC_EXIT();
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int hyn_input_report_key(struct hyn_ts_data *data, int index)
{
    int i = 0;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int *x_dim = &data->pdata->key_x_coords[0];
    int *y_dim = &data->pdata->key_y_coords[0];

    if (!data->pdata->have_key) {
        return -EINVAL;
    }
    for (i = 0; i < data->pdata->key_number; i++) {
        if ((x >= x_dim[i] - HYN_KEY_DIM) && (x <= x_dim[i] + HYN_KEY_DIM) &&
            (y >= y_dim[i] - HYN_KEY_DIM) && (y <= y_dim[i] + HYN_KEY_DIM)) {
            if (EVENT_DOWN(data->events[index].flag)
                && !(data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 1);
                data->key_state |= (1 << i);
                HYN_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            } else if (EVENT_UP(data->events[index].flag)
                       && (data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 0);
                data->key_state &= ~(1 << i);
                HYN_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -EINVAL;
}

#if HYN_MT_PROTOCOL_B_EN
static int hyn_input_report_b(struct hyn_ts_data *data)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    struct ts_event *events = data->events;
    HYN_FUNC_ENTER();
    for (i = 0; i < data->touch_point; i++) {
        if (hyn_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        input_mt_slot(data->input_dev, events[i].id);

        if (EVENT_DOWN(events[i].flag)) {
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if HYN_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (HYN_TOUCH_DOWN == events[i].flag))) {
                HYN_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
        } else {
            uppoint++;
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(events[i].id);
            if (data->log_level >= 1) {
                HYN_DEBUG("[B]P%d UP!", events[i].id);
            }
        }
    }
    printk("liqw--- tp_x =%d , tp_y = %d \n",events[i].x,events[i].y);
	
    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                if (data->log_level >= 1) {
                    HYN_DEBUG("[B]P%d UP!", i);
                }
                va_reported = true;
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    data->touchs = touchs;

    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if (data->log_level >= 1) {
                HYN_DEBUG("[B]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    HYN_FUNC_EXIT();

    return 0;
}

#else
static int hyn_input_report_a(struct hyn_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (hyn_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
			#if 1
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
			if(i == 0)
			{
		       if (events[i].area <= 0) {
                	events[i].area = 0x09;
            	}
							
				input_report_abs(data->input_dev, ABS_X,239-events[i].x);
				input_report_abs(data->input_dev, ABS_Y,239-events[i].y);
				input_report_abs(data->input_dev, ABS_PRESSURE, events[i].p);
				//printk("======x[%d]====y[%d]====\n",events[i].x,events[i].y);
			}
			#else
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if HYN_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (HYN_TOUCH_DOWN == events[i].flag))) {
                HYN_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
            touchs++;
			#endif
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
		//	printk("=====[A]Points All Up!===\n");
            input_report_key(data->input_dev, BTN_TOUCH, 0);	
			input_report_abs(data->input_dev, ABS_PRESSURE, 0);
			input_sync(data->input_dev);
	
            input_mt_sync(data->input_dev);
        } else {
        //	printk("=====[A]Points All down!===\n");
            input_report_key(data->input_dev, BTN_TOUCH, 1);
			input_sync(data->input_dev);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif

static int hyn_read_touchdata(struct hyn_ts_data *data)
{
    int ret = 0;
    u8 *buf = data->point_buf;

    memset(buf, 0xFF, data->pnt_buf_size);

    buf[0] = 0x00;
    ret = hyn_read(buf, 1, buf, data->pnt_buf_size);
    if (ret < 0) {
        HYN_ERROR("read touchdata failed, ret:%d", ret);
        return ret;
    }

    if (data->log_level >= 3) {
        hyn_show_touch_buffer(buf, data->pnt_buf_size);
    }
    //if(buf[2]>0)
    //printk("vision_d:X:%d Y:%d \n",(((buf[3]<<8) &0x0f) | buf[4]),(((buf[5]<<8)&0x0f)| buf[6]));

    return 0;
}

static int hyn_read_parse_touchdata(struct hyn_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;

    ret = hyn_read_touchdata(data);
    if (ret) {
        return ret;
    }

    data->point_num = buf[2] & 0x0F;
    data->touch_point = 0;

    if (data->point_num > max_touch_num) {
        HYN_INFO("invalid point_num(%d)", data->point_num);
        return -EIO;
    }

    for (i = 0; i < max_touch_num; i++) {
        base = HYN_ONE_TCH_LEN * i;
        pointid = (buf[5 + base]) >> 4;
        if (pointid >= HYN_MAX_ID)
            break;
        //else if (pointid >= max_touch_num) {
        //    HYN_ERROR("ID(%d) beyond max_touch_number", pointid);
        //    return -EINVAL;
        //}
        //data->touch_point++;
        events[i].x = ((buf[3 + base] & 0x0F) << 8) +
                      (buf[4 + base] & 0xFF);
        events[i].y = ((buf[5 + base] & 0x0F) << 8) +
                      (buf[6 + base] & 0xFF);
        events[i].flag = buf[3 + base] >> 6;
        events[i].id = buf[5 + base] >> 4;
        events[i].area = buf[8 + base] >> 4;
        events[i].p =  buf[7 + base];

	    data->touch_point++;
    }

    if (data->touch_point == 0) {
        HYN_INFO("no touch point information");
        return -EIO;
    }

    return 0;
}

static void hyn_irq_read_report(void)
{
    int ret = 0;
    struct hyn_ts_data *ts_data = hyn_data;

    ret = hyn_read_parse_touchdata(ts_data);
    if (ret == 0) {
        //mutex_lock(&ts_data->report_mutex);
#if HYN_MT_PROTOCOL_B_EN
        hyn_input_report_b(ts_data);
#else
        hyn_input_report_a(ts_data);
#endif
        //mutex_unlock(&ts_data->report_mutex);
    }
}

static irqreturn_t hyn_irq_handler(int irq, void *data)
{
    hyn_irq_read_report();
    return IRQ_HANDLED;
}

static int hyn_irq_registration(struct hyn_ts_data *ts_data)
{
    int ret = 0;
    struct hyn_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    pdata->irq_gpio_flags = IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT /*| IRQ_TYPE_EDGE_RISING*/;
    HYN_INFO("== irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
    ret = request_threaded_irq(ts_data->irq, NULL, hyn_irq_handler,
                               pdata->irq_gpio_flags,
                               HYN_DRIVER_NAME, ts_data);
    return ret;
}

static int hyn_input_init(struct hyn_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct hyn_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    HYN_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        HYN_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = HYN_DRIVER_NAME;

    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = ts_data->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        HYN_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if HYN_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if HYN_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	//******add*********************
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFF, 0 , 0);
	//******************************


    ret = input_register_device(input_dev);
    if (ret) {
        HYN_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

    ts_data->input_dev = input_dev;

    HYN_FUNC_EXIT();
    return 0;
}

static int hyn_report_buffer_init(struct hyn_ts_data *ts_data)
{
    int point_num = 0;
    int events_num = 0;

    point_num = ts_data->pdata->max_touch_number;
    ts_data->pnt_buf_size = point_num * HYN_ONE_TCH_LEN + 3;
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        HYN_ERROR("failed to alloc memory for point buf");
        return -ENOMEM;
    }

    events_num = point_num * sizeof(struct ts_event);
    ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
    if (!ts_data->events) {
        HYN_ERROR("failed to alloc memory for point events");
        kfree_safe(ts_data->point_buf);
        return -ENOMEM;
    }

    return 0;
}

#if HYN_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if HYN_PINCTRL_EN
static int hyn_pinctrl_init(struct hyn_ts_data *ts)
{
    int ret = 0;

    ts->pinctrl = devm_pinctrl_get(ts->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        HYN_ERROR("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        HYN_ERROR("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }

    ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
    if (IS_ERR_OR_NULL(ts->pins_suspend)) {
        HYN_ERROR("Pin state[suspend] not found");
        ret = PTR_ERR(ts->pins_suspend);
        goto err_pinctrl_lookup;
    }

    ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
    if (IS_ERR_OR_NULL(ts->pins_release)) {
        HYN_ERROR("Pin state[release] not found");
        ret = PTR_ERR(ts->pins_release);
    }

    return 0;
err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_release = NULL;
    ts->pins_suspend = NULL;
    ts->pins_active = NULL;
    return ret;
}

static int hyn_pinctrl_select_normal(struct hyn_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_active) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
        if (ret < 0) {
            HYN_ERROR("Set normal pin state error:%d", ret);
        }
    }

    return ret;
}

static int hyn_pinctrl_select_suspend(struct hyn_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_suspend) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
        if (ret < 0) {
            HYN_ERROR("Set suspend pin state error:%d", ret);
        }
    }

    return ret;
}

static int hyn_pinctrl_select_release(struct hyn_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl) {
        if (IS_ERR_OR_NULL(ts->pins_release)) {
            devm_pinctrl_put(ts->pinctrl);
            ts->pinctrl = NULL;
        } else {
            ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
            if (ret < 0)
                HYN_ERROR("Set gesture pin state error:%d", ret);
        }
    }

    return ret;
}
#endif /* HYN_PINCTRL_EN */

static int hyn_power_source_ctrl(struct hyn_ts_data *ts_data, int enable)
{
    int ret = 0;
    return ret;
}

/*****************************************************************************
* Name: hyn_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after hyn_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in hyn_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int hyn_power_source_init(struct hyn_ts_data *ts_data)
{
    int ret = 0;

    HYN_FUNC_ENTER();
    ts_data->vdd = regulator_get(ts_data->dev, "vdd");
    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        ret = PTR_ERR(ts_data->vdd);
        HYN_ERROR("get vdd regulator failed,ret=%d", ret);
        return ret;
    }

    if (regulator_count_voltages(ts_data->vdd) > 0) {
        ret = regulator_set_voltage(ts_data->vdd, HYN_VTG_MIN_UV,
                                    HYN_VTG_MAX_UV);
        if (ret) {
            HYN_ERROR("vdd regulator set_vtg failed ret=%d", ret);
            regulator_put(ts_data->vdd);
            return ret;
        }
    }

    ts_data->vcc_i2c = regulator_get(ts_data->dev, "vcc_i2c");
    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
            ret = regulator_set_voltage(ts_data->vcc_i2c,
                                        HYN_I2C_VTG_MIN_UV,
                                        HYN_I2C_VTG_MAX_UV);
            if (ret) {
                HYN_ERROR("vcc_i2c regulator set_vtg failed,ret=%d", ret);
                regulator_put(ts_data->vcc_i2c);
            }
        }
    }

#if HYN_PINCTRL_EN
    hyn_pinctrl_init(ts_data);
    hyn_pinctrl_select_normal(ts_data);
#endif

    ts_data->power_disabled = true;
    ret = hyn_power_source_ctrl(ts_data, ENABLE);
    if (ret) {
        HYN_ERROR("fail to enable power(regulator)");
    }

    HYN_FUNC_EXIT();
    return ret;
}

static int hyn_power_source_exit(struct hyn_ts_data *ts_data)
{
#if HYN_PINCTRL_EN
    hyn_pinctrl_select_release(ts_data);
#endif

    hyn_power_source_ctrl(ts_data, DISABLE);

    if (!IS_ERR_OR_NULL(ts_data->vdd)) {
        if (regulator_count_voltages(ts_data->vdd) > 0)
            regulator_set_voltage(ts_data->vdd, 0, HYN_VTG_MAX_UV);
        regulator_put(ts_data->vdd);
    }

    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
            regulator_set_voltage(ts_data->vcc_i2c, 0, HYN_I2C_VTG_MAX_UV);
        regulator_put(ts_data->vcc_i2c);
    }

    return 0;
}

static int hyn_power_source_suspend(struct hyn_ts_data *ts_data)
{
    int ret = 0;

#if HYN_PINCTRL_EN
    hyn_pinctrl_select_suspend(ts_data);
#endif

    ret = hyn_power_source_ctrl(ts_data, DISABLE);
    if (ret < 0) {
        HYN_ERROR("power off fail, ret=%d", ret);
    }

    return ret;
}

static int hyn_power_source_resume(struct hyn_ts_data *ts_data)
{
    int ret = 0;

#if HYN_PINCTRL_EN
    hyn_pinctrl_select_normal(ts_data);
#endif

    ret = hyn_power_source_ctrl(ts_data, ENABLE);
    if (ret < 0) {
        HYN_ERROR("power on fail, ret=%d", ret);
    }

    return ret;
}
#endif /* HYN_POWER_SOURCE_CUST_EN */

static int hyn_gpio_configure(struct hyn_ts_data *data)
{
    int ret = 0;
    return ret;
}

static int hyn_parse_dt(struct hyn_ts_platform_data *pdata)
{
    HYN_FUNC_ENTER();

    pdata->x_min = HYN_X_MIN_DISPLAY_DEFAULT;
    pdata->y_min = HYN_Y_MIN_DISPLAY_DEFAULT;
    pdata->x_max = HYN_X_MAX_DISPLAY_DEFAULT;
    pdata->y_max = HYN_Y_MAX_DISPLAY_DEFAULT;
    pdata->max_touch_number = 2;
    
    /* reset, irq gpio info */
    pdata->reset_gpio = HYN_RST_PORT;
    if (pdata->reset_gpio < 0)
        HYN_ERROR("Unable to get reset_gpio");
    else
	gpio_request(pdata->reset_gpio,"tp_reset");

    pdata->irq_gpio = HYN_INT_PORT;
    if (pdata->irq_gpio < 0)
        HYN_ERROR("Unable to get irq_gpio");
    else
	gpio_request(pdata->irq_gpio,"tp_irq");

    HYN_FUNC_EXIT();
    return 0;
}

#if defined(CONFIG_FB)
static void hyn_resume_work(struct work_struct *work)
{
    struct hyn_ts_data *ts_data = container_of(work, struct hyn_ts_data,
                                  resume_work);

    hyn_ts_resume(ts_data->dev);
}

static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;

    if (!( event == FB_EVENT_BLANK)) {
        HYN_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    HYN_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EVENT_BLANK == event) {
            queue_work(hyn_data->ts_workqueue, &hyn_data->resume_work);
        }
        break;
    case FB_BLANK_POWERDOWN:
         if (FB_EVENT_BLANK == event) {
            HYN_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        HYN_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void hyn_ts_early_suspend(struct early_suspend *handler)
{
    struct hyn_ts_data *ts_data = container_of(handler, struct hyn_ts_data,
                                  early_suspend);

    hyn_ts_suspend(ts_data->dev);
}

static void hyn_ts_late_resume(struct early_suspend *handler)
{
    struct hyn_ts_data *ts_data = container_of(handler, struct hyn_ts_data,
                                  early_suspend);

    hyn_ts_resume(ts_data->dev);
}
#endif



int hyn_ts_switch(int mode)
{
	if(mode == 0)
	{
		hyn_ts_suspend(hyn_data->dev);
	}
	else
	{
		hyn_ts_resume(hyn_data->dev);
	}
	return 0;
}



static int hyn_ts_probe_entry(struct hyn_ts_data *ts_data)
{
#if HYN_FW_UPDATA
	int dev_addr;
#endif
    	int ret = 0;
    	u8 value;
    	int pdata_size = sizeof(struct hyn_ts_platform_data);
	struct i2c_client *client = NULL;
	
	client = hyn_data->client;

    	HYN_FUNC_ENTER();
    
    	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    
        ret = hyn_parse_dt(ts_data->pdata);
        if (ret)
            HYN_ERROR("device-tree parse fail");
    
	printk("hyn_ts_probe_entry 1\n");
    	ts_data->ts_workqueue = create_singlethread_workqueue("hyn_wq");
    	if (!ts_data->ts_workqueue) {
        	HYN_ERROR("create fts workqueue fail");
    	}
	printk("hyn_ts_probe_entry 2\n");
    	spin_lock_init(&ts_data->irq_lock);
    	mutex_init(&ts_data->report_mutex);
    	mutex_init(&ts_data->bus_lock);
	printk("hyn_ts_probe_entry 3\n");
    	/* Init communication interface */
	printk("hyn_ts_probe_entry 4\n");
    	ret = hyn_input_init(ts_data);
    	if (ret) {
        	HYN_ERROR("input initialize fail");
        	goto err_input_init;
    	}
	printk("hyn_ts_probe_entry 5\n");
    	ret = hyn_report_buffer_init(ts_data);
    	if (ret) {
        	HYN_ERROR("report buffer init fail");
        	goto err_report_buffer;
    	}
	printk("hyn_ts_probe_entry 6\n");
    	ret = hyn_gpio_configure(ts_data);
    	if (ret) {
        	HYN_ERROR("configure the gpios fail");
        	goto err_gpio_config;
    	}
	printk("hyn_ts_probe_entry 7\n");
#if HYN_POWER_SOURCE_CUST_EN
    	ret = hyn_power_source_init(ts_data);
    	if (ret) {
        	HYN_ERROR("fail to get power(regulator)");
        	goto err_power_init;
    	}
#endif

	printk("hyn_ts_probe_entry 8\n");

    	hyn_reset_proc(200);  //rest plus delay 200ms waite touch ic ready

    	ret = hyn_get_ic_information(ts_data);
    	if (ret) {
        	HYN_ERROR("not hyn IC, unregister driver");
        	goto err_irq_req;
    	}
#if HYN_FW_UPDATA	
	//if(hyn_ctpm_get_upg_ver() > ts_data->ic_info.ids.fw_ver) //if bin ver > fw_ver updata
	{
      printk("hyn_ts_probe_entry 9-2  fw update start\n");

	  dev_addr	= client->addr;
	  
	  client->addr = 0x6A;  //read point use 0x15 slave addr  updata use 0x6A slave addr

	  hyn_ctpm_fw_upgrade_with_i_file(ts_data);
	  
	  client->addr = dev_addr;

	  hyn_reset_proc(200);
      
	 printk("hyn_ts_probe_entry 9-2  fw update end\n");
	}
#endif

	printk("hyn_ts_probe_entry 10\n");
    	ret = hyn_irq_registration(ts_data);
    	if (ret) {
        	HYN_ERROR("request irq failed");
        	goto err_irq_req;
    	}

#if defined(CONFIG_FB)
    	if (ts_data->ts_workqueue) {
        	INIT_WORK(&ts_data->resume_work, hyn_resume_work);
    	}
    	ts_data->fb_notif.notifier_call = fb_notifier_callback;
    	ret = fb_register_client(&ts_data->fb_notif);
    	if (ret) {
        	HYN_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    	ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    	ts_data->early_suspend.suspend = hyn_ts_early_suspend;
    	ts_data->early_suspend.resume = hyn_ts_late_resume;
    	register_early_suspend(&ts_data->early_suspend);
#endif
	printk("hyn_ts_probe_entry 11\n");
    	HYN_FUNC_EXIT();
    	return 0;

err_irq_req:
#if HYN_POWER_SOURCE_CUST_EN
err_power_init:
    	hyn_power_source_exit(ts_data);
#endif
err_gpio_config:
    	kfree_safe(ts_data->point_buf);
    	kfree_safe(ts_data->events);
err_report_buffer:
    	input_unregister_device(ts_data->input_dev);
err_input_init:
    	if (ts_data->ts_workqueue)
        	destroy_workqueue(ts_data->ts_workqueue);

    	kfree_safe(ts_data->bus_buf);
    	kfree_safe(ts_data->pdata);

    	HYN_FUNC_EXIT();
    	return ret;
}

static int hyn_ts_remove_entry(struct hyn_ts_data *ts_data)
{
    HYN_FUNC_ENTER();

    hyn_bus_exit(ts_data);

    free_irq(ts_data->irq, ts_data);
    input_unregister_device(ts_data->input_dev);
    input_free_device(ts_data->input_dev);
    i2cdev_exit();

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        HYN_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

#if HYN_POWER_SOURCE_CUST_EN
    hyn_power_source_exit(ts_data);
#endif

    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);

    kfree_safe(ts_data->pdata);
    kfree_safe(ts_data);

    HYN_FUNC_EXIT();

    return 0;
}
static int hyn_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct hyn_ts_data *ts_data = hyn_data;

    HYN_FUNC_ENTER();
    if (ts_data->suspended) {
        HYN_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        HYN_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

    hyn_irq_disable();

    /* TP enter sleep mode */
    ret = hyn_write_reg(HYN_REG_POWER_MODE, HYN_REG_POWER_MODE_SLEEP_VALUE);
    if (ret < 0)
        HYN_ERROR("set TP to sleep mode fail, ret=%d", ret);

    if (!ts_data->ic_info.is_incell) {
#if HYN_POWER_SOURCE_CUST_EN
        ret = hyn_power_source_suspend(ts_data);
        if (ret < 0) {
            HYN_ERROR("power enter suspend fail");
        }
#endif
    }

    ts_data->suspended = true;
    HYN_FUNC_EXIT();
    return 0;
}

static int hyn_ts_resume(struct device *dev)
{
    struct hyn_ts_data *ts_data = hyn_data;

    HYN_FUNC_ENTER();
    if (!ts_data->suspended) {
        HYN_DEBUG("Already in awake state");
        return 0;
    }

    hyn_release_all_finger();

#if HYN_POWER_SOURCE_CUST_EN
        hyn_power_source_resume(ts_data);
#endif
        hyn_reset_proc(200);

    hyn_tp_state_recovery(ts_data);

    hyn_irq_enable();

    ts_data->suspended = false;
    HYN_FUNC_EXIT();
    return 0;
}


/*****************************************************************************
* TP Driver
*****************************************************************************/

static int hyn_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct hyn_ts_data *ts_data = NULL;

   
	//init_cam_iic();
    /* malloc memory for global struct variable */
    ts_data = (struct hyn_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        HYN_ERROR("allocate memory for hyn_data fail");
        return -ENOMEM;
    }

    hyn_data = ts_data;
    ts_data->client = hi_client;
    ts_data->dev = &hi_client->dev;
    ts_data->log_level = 4;	//vision 1
    ts_data->fw_is_running = 0;
   
    ret = hyn_ts_probe_entry(ts_data);
    if (ret) {
        HYN_ERROR("Touch Screen(I2C BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }

    HYN_INFO("Touch Screen(I2C BUS) driver prboe successfully");
    return 0;
}

static int hyn_ts_remove(struct i2c_client *client)
{
    return hyn_ts_remove_entry(hyn_data);
}

static int __init hyn_ts_init(void)
{
    int ret = 0;

    HYN_FUNC_ENTER();	////printk
    printk("%s load touch driver\n",__func__);
    ret = i2cdev_init();
    if (ret)
    {
        dev_err(NULL, " i2cdev_init fail!\n");
        goto error_end;
    }
    ret = hyn_ts_probe(0, 0);
    if ( ret != 0 ) {
        HYN_ERROR("Focaltech touch screen driver init failed!");
    }
    HYN_FUNC_EXIT();
    return ret;
error_end:
    return -1;
}

static void __exit hyn_ts_exit(void)
{
	hyn_ts_remove(0);
}
module_init(hyn_ts_init);
module_exit(hyn_ts_exit);

MODULE_AUTHOR("hyn Driver Team");
MODULE_DESCRIPTION("hyn Touchscreen Driver");
MODULE_LICENSE("GPL v2");
