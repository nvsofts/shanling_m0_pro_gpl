/*
 * AXS touchscreen driver.
 *
 * Copyright (c) 2020-2021 AiXieSheng Technology. All rights reserved.
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

#include "axs_core.h"
struct axs_ts_data *g_axs_data=NULL;
u8 g_axs_fw_ver = 0;
u8 g_axs_reset_flag = 0; // 0:normal;  1:abnormal, need reset
#ifdef HAVE_TOUCH_KEY
const struct key_data key_array[]=
{
    KEY_ARRAY
};
#define KEY_NUM   (sizeof(key_array)/sizeof(key_array[0]))
#endif

u8 write_cmd[22] = {0xb5,0xab,0x5a,0xa5,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xff,0x00};  // max write len: 22-12=10
u8 read_cmd[11] = {0xb5,0xab,0xa5,0x5a,0x00,0x00,0x00,0x01,0x00,0x00,0x00};

u8 read_sfr_cmd[11] = {0x5a,0xa5,0xab,0xb5,0x00,0x00,0x00,0x01,0x00,0x80,0x00};

int axs_write_read_reg(u8 *reg,u16 reg_len,u8 *rd_data,u16 rd_len)
{
    int ret = 0;
    if (reg_len>10)
    {
        reg_len = 10;
    }
    write_cmd[4] = (u8)((reg_len+1)>>8);
    write_cmd[5] = (u8)((reg_len+1)&0xff);
    memcpy(&write_cmd[11],reg,reg_len);
    write_cmd[11+reg_len] = 0;
    ret = axs_write(write_cmd,12+reg_len);
    if (ret < 0)
    {
        AXS_ERROR("axs_write fail");
        return ret;
    }
    if (rd_len > 0)
    {
        read_cmd[6] = (u8)(rd_len>>8);
        read_cmd[7] = (u8)(rd_len&0xff);
        ret = axs_write_read(read_cmd,sizeof(read_cmd), rd_data, rd_len);
        if (ret < 0)
        {
            AXS_ERROR("axs_write_read fail");
            return ret;
        }
    }
    return 0;
}


int axs_read_reg(u8 *rd_data,u16 rd_len)
{
    int ret = 0;
    if (rd_len > 0)
    {
        read_cmd[6] = (u8)(rd_len>>8);
        read_cmd[7] = (u8)(rd_len&0xff);
        ret = axs_write_read(read_cmd,sizeof(read_cmd), rd_data, rd_len);
        if (ret < 0)
        {
            AXS_ERROR("axs_write_read fail");
            return ret;
        }
    }
    return 0;
}

int axs_read_sfr_reg(u8 sfr_reg,u8 *rd_data,u16 rd_len)
{
    int ret = 0;
    if (rd_len > 0)
    {
        read_sfr_cmd[6] = (u8)(rd_len>>8);
        read_sfr_cmd[7] = (u8)(rd_len&0xff);
        read_sfr_cmd[10] = sfr_reg;
        ret = axs_write_read(read_sfr_cmd,sizeof(read_sfr_cmd), rd_data, rd_len);
        if (ret < 0)
        {
            AXS_ERROR("axs_read_esd_reg fail");
            return ret;
        }
    }
    else
    {
        return -1;
    }
    return 0;
}

int axs_read_fw_version(u8 *ver)
{
    u8 cmd_type[1] = {AXS_REG_VERSION_READ};
    u8 sfr_value;
    u8 sfr_value2;

    u8 common_value[2]= {0,0};
    u8 common_value2[2]= {0,0};
    //read firmware version from sfr reg
    if (axs_read_sfr_reg(0x89,&sfr_value,1))
    {
        return -1;
    }
    if (axs_read_sfr_reg(0x89,&sfr_value2,1))
    {
        return -1;
    }
    if (sfr_value != sfr_value2)
    {
        return  -1;
    }
    if (sfr_value2!=0)
    {
        *ver = sfr_value2;
        return 0;
    }
    //read firmware version from common reg; compatible with AXS15230 firmware old version;
    if (axs_write_read_reg(cmd_type, 1, common_value, 2))
    {
        return  -1;
    }
    if (axs_write_read_reg(cmd_type, 1, common_value2, 2))
    {
        return  -1;
    }
    if (common_value[1] != common_value2[1])
    {
        return  -1;
    }
    *ver = common_value2[1];
    return 0;
}

bool axs_fwupg_get_ver_in_tp(u8 *ver)
{
    int ret = 0;
    u8 val[1] = { 0 };

    ret = axs_read_fw_version(&val[0]);
    if (ret < 0)
    {
        AXS_DEBUG("tp fw invaild");
        return false;
    }
    *ver = val[0];
    return true;
}
void axs_reset_level(u8 level)
{
    if (g_axs_data->pdata)
    {

        gpio_direction_output(g_axs_data->pdata->reset_gpio, level);
        //tpd_gpio_output(g_axs_data->pdata->reset_gpio, level);
    }
}
void axs_reset_proc(int hdelayms)
{
    struct axs_ts_platform_data *pdata = g_axs_data->pdata;
    AXS_DEBUG("axs_reset_proc");
    if (pdata)
    {
        gpio_direction_output(pdata->reset_gpio, 1);
        msleep(1);
        gpio_set_value(pdata->reset_gpio, 0);
        msleep(10);
        gpio_set_value(pdata->reset_gpio, 1);
        if (hdelayms)
            msleep(hdelayms);
    }
    else
    {
        AXS_DEBUG("axs_reset_proc fail!");
    }
}
static int axs_input_init(struct axs_ts_data *ts_data)
{
    int ret = 0;
#ifdef HAVE_TOUCH_KEY
    int i = 0;
#endif
    struct input_dev *input_dev;
    input_dev = input_allocate_device();
    if (!input_dev)
    {
        AXS_ERROR("Failed to allocate memory for input device");
        return  -ENOMEM;
    }
    g_axs_data->input_dev = input_dev;

    input_dev->dev.parent = ts_data->dev;
    input_dev->name = AXS_DRIVER_NAME;      //dev_name(&client->dev)
    if (ts_data->bus_type == BUS_TYPE_I2C)
        input_dev->id.bustype = BUS_I2C;
    else
        input_dev->id.bustype = BUS_SPI;
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
#if AXS_MT_PROTOCOL_B_EN
    set_bit(BTN_TOOL_FINGER, input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    input_mt_init_slots(input_dev, AXS_MAX_TOUCH_NUMBER, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, (AXS_MAX_TOUCH_NUMBER - 1), 0, 0);
#endif

    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if AXS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

#ifdef CYADD_POINT_ONE
	//******cyadd one point*********************
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFF, 0 , 0);
	//******************************
#endif



#ifdef HAVE_TOUCH_KEY
    for (i = 0; i < KEY_NUM; i++)
    {
        input_set_capability(input_dev, EV_KEY, key_array[i].key);
    }
#endif

    ret = input_register_device(input_dev);
    if (ret)
    {
        AXS_ERROR("Input device registration failed");
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

    ts_data->input_dev = input_dev;
    return 0;
}

#if 1
static int axs_read_touchdata(struct axs_ts_data *data)
{
    int ret = 0;
    u8 *buf = data->point_buf;
    memset(buf, 0xFF, data->pnt_buf_size);
    //ret = axs_write_read_reg(cmd_type, 1, buf+1, (u16)(data->pnt_buf_size - 1));
    ret = axs_read_reg(buf+1, (u16)(data->pnt_buf_size - 1));
    if (ret < 0)
    {
        AXS_ERROR("axs_read_reg failed, ret:%d", ret);
        return ret;
    }
    return 0;
}
#else
static int axs_read_touchdata(struct axs_ts_data *data)
{
    int ret = 0;
    u8 *buf = data->point_buf;

    memset(buf, 0xFF, data->pnt_buf_size);
    buf[0] = 0x0;

#if AXS_BUS_SPI
    ret = axs_read(buf + 1, data->pnt_buf_size - 1); // spi don't need to write 0
#else
    ret = axs_write_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
#endif
    if (ret < 0)
    {
        AXS_ERROR("read touchdata failed, ret:%d", ret);
        return ret;
    }


    return 0;
}
#endif

/*
error:return < 0
point:return 0
gesture:return > 0
*/
static int axs_read_parse_touchdata(struct axs_ts_data *data)
{
    int ret = 0;
    int i = 0;
    int point_num ;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = NULL;
    u8 *buf  = NULL;
    if (!data)
    {
        AXS_DEBUG("!data ret");
        return -1;
    }
    buf = data->point_buf;
    events = data->events;
    ret = axs_read_touchdata(data);
    if (ret< 0)
    {
        AXS_DEBUG("axs_read_touchdata failed, ret:%d", ret);
        return ret;
    }
#if AXS_GESTURE_EN
    if (data->gesture_enable)
    {
        if (0 != buf[AXS_TOUCH_GESTURE_POS])
        {
            AXS_DEBUG("succuss to get gesture data in irq handler");
            return buf[AXS_TOUCH_GESTURE_POS];
        }
    }
#endif

    /*printk("axs_read_parse_touchdata:");
    for(i = 0; i < 33; i++){
        printk( "0x%02x,",buf[i]);
    }
    printk( "\n");*/

    point_num = buf[AXS_TOUCH_POINT_NUM] & 0x0F;
    data->touch_point = 0;
    if (point_num > AXS_MAX_TOUCH_NUMBER)
    {
        AXS_ERROR("invalid point_num(%d)", point_num);
        point_num = AXS_MAX_TOUCH_NUMBER;
        //return -1;
    }

   // AXS_DEBUG("point_num=%d\n",point_num);
    for (i = 0; i < point_num; i++)   //AXS_MAX_TOUCH_NUMBER
    {
        base = AXS_ONE_TCH_LEN * i;
        pointid = (buf[AXS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= AXS_MAX_ID)
        {
            AXS_ERROR("pointid(%d) beyond AXS_MAX_ID", pointid);
            break;
        }
        else if (pointid >= AXS_MAX_TOUCH_NUMBER)
        {
            AXS_ERROR("pointid(%d) beyond max_touch_number", pointid);
            return -1;
        }

        data->touch_point++;
        events[i].x = ((buf[AXS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[AXS_TOUCH_X_L_POS + base]);
        events[i].y = ((buf[AXS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[AXS_TOUCH_Y_L_POS + base]);
        events[i].flag = buf[AXS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[AXS_TOUCH_ID_POS + base] >> 4;
        events[i].area = buf[AXS_TOUCH_AREA_POS + base] >> 4;
        events[i].weight =  buf[AXS_TOUCH_WEIGHT_POS + base];

        if (EVENT_DOWN(events[i].flag) && (point_num == 0))
        {
            AXS_ERROR("abnormal touch data from fw");
            return -1;
        }
    }

    if (data->touch_point == 0)
    {
        AXS_ERROR("no touch point information");
        return -1;
    }
    return 0;
}

void axs_release_all_finger(void)
{
    struct input_dev *input_dev = g_axs_data->input_dev;
#if AXS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = AXS_MAX_TOUCH_NUMBER;
#endif

    AXS_DEBUG("release all app point");
    mutex_lock(&g_axs_data->report_mutex);
#if AXS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++)
    {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
    input_sync(input_dev);
#endif
    g_axs_data->touchs = 0;
    g_axs_data->touch_point = 0;
    g_axs_data->key_state = 0;
    mutex_unlock(&g_axs_data->report_mutex);
}

#ifdef HAVE_TOUCH_KEY
static int axs_input_report_key(struct axs_ts_data *data, int index)
{
    int i = 0;
    int x = data->events[index].x;
    int y = data->events[index].y;

    for (i = 0; i < KEY_NUM; i++)
    {
        if ((x > key_array[i].x_min) && (x < key_array[i].x_max) &&
            (y > key_array[i].y_min) && (y < key_array[i].y_max))
        {
            if (EVENT_DOWN(data->events[index].flag)
                && !(data->key_state & (1 << i)))
            {
                input_report_key(data->input_dev, key_array[i].key, 1);
                data->key_state |= (1 << i);
                AXS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            }
            else if (EVENT_UP(data->events[index].flag)
                     && (data->key_state & (1 << i)))
            {
                input_report_key(data->input_dev, key_array[i].key, 0);
                data->key_state &= ~(1 << i);
                AXS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -1;
}
#endif


#if AXS_PEN_EVENT_CHECK_EN
int axs_pen_event_check_suspend(void)
{
    AXS_FUNC_ENTER();
    if (g_axs_data->pen_event_check_enable)
    {
        cancel_delayed_work(&g_axs_data->pen_event_check_work);
    }
    AXS_FUNC_EXIT();
    return 0;
}
int axs_pen_event_check_resume(void)
{
    AXS_FUNC_ENTER();
    if (g_axs_data->pen_event_check_enable)
    {
        queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->pen_event_check_work,
                           msecs_to_jiffies(500));
    }
    AXS_FUNC_EXIT();
    return 0;
}

void axs_pen_event_check_func(struct work_struct *work)
{
    if (g_axs_data->tp_report_interval_500ms <= 10)
    {
        g_axs_data->tp_report_interval_500ms += 1;
    }
    AXS_DEBUG("interval_500ms:%d,touchs:%d", g_axs_data->tp_report_interval_500ms,g_axs_data->touchs);

    if ((g_axs_data->tp_report_interval_500ms >= 2) && (g_axs_data->touchs != 0)) // max wait time:1s
    {
        int i = 0;
        for (i = 0; i < AXS_MAX_TOUCH_NUMBER; i++)
        {
            if (BIT(i) & (g_axs_data->touchs))
            {
                AXS_ERROR("missed P%d UP!", i);
                input_mt_slot(g_axs_data->input_dev, i);
                input_mt_report_slot_state(g_axs_data->input_dev, MT_TOOL_FINGER, false);
                g_axs_data->touchs &= ~BIT(i);
            }
        }
        input_report_key(g_axs_data->input_dev, BTN_TOUCH, 0);
        input_sync(g_axs_data->input_dev);
    }

    queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->pen_event_check_work,
                       msecs_to_jiffies(500));

}
int axs_pen_event_check_init(struct axs_ts_data *ts_data)
{
    AXS_FUNC_ENTER();

    if (ts_data->ts_workqueue)
    {
        INIT_DELAYED_WORK(&ts_data->pen_event_check_work, axs_pen_event_check_func);
    }
    else
    {
        AXS_ERROR("ts_workqueue is NULL");
        return -EINVAL;
    }

    g_axs_data->pen_event_check_enable = 1;

    queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->pen_event_check_work,
                       msecs_to_jiffies(500));
    AXS_FUNC_EXIT();
    return 0;
}

#endif


#if AXS_MT_PROTOCOL_B_EN
static int axs_input_report_b(struct axs_ts_data *data)
{
    int i = 0;
    int down_points = 0;
    int up_points = 0;

    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = AXS_MAX_TOUCH_NUMBER;

    struct ts_event *events = data->events;
    //AXS_DEBUG("[B] report touch begin, point:%2x",data->touch_point);
    for (i = 0; i < data->touch_point; i++)
    {
#ifdef HAVE_TOUCH_KEY
        if (axs_input_report_key(data, i) == 0)
        {
            continue;
        }
#endif
        //AXS_DEBUG("P%d [touchs:%x]  !", events[i].id,data->touchs);
        va_reported = true;
#ifdef CYADD_POINT_ONE
		if (EVENT_DOWN(events[i].flag))
        {
        	if (events[i].weight <= 0)
            {
                events[i].weight = 0x3f;
            }
			if (events[i].area <= 0)
            {
                events[i].area = 0x09;
            }
			input_report_abs(data->input_dev, ABS_X,events[i].x);
			input_report_abs(data->input_dev, ABS_Y,events[i].y);
			input_report_abs(data->input_dev, ABS_PRESSURE, events[i].weight);
			//printk("======x[%d]====y[%d]====\n",events[i].x,events[i].y);
		    touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);
			down_points++;
		}
		else
		{
			up_points++;
            data->touchs &= ~BIT(events[i].id);
		}
#else 
        input_mt_slot(data->input_dev, events[i].id);
        if (EVENT_DOWN(events[i].flag))
        {
        
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if AXS_REPORT_PRESSURE_EN
            if (events[i].weight <= 0)
            {
                events[i].weight = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].weight);
#endif
            if (events[i].area <= 0)
            {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);

            /*     AXS_DEBUG("[B]P%d(%d, %d)[weight:%d,area:%d] DOWN!",
                           events[i].id,
                           events[i].x, events[i].y,
                           events[i].weight, events[i].area);
               AXS_DEBUG("P%d [touchs:%x]  !", events[i].id,data->touchs);
            */
            down_points++;
        }
        else
        {
            up_points++;
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(events[i].id);
            //AXS_DEBUG("[B]P%d UP!", events[i].id);
        }
 #endif
    }

    if (unlikely(data->touchs ^ touchs))
    {
        for (i = 0; i < max_touch_num; i++)
        {
            if (BIT(i) & (data->touchs ^ touchs))
            {
                //AXS_ERROR("P%d UP, Reup missed point !", i);
                va_reported = true;
#ifndef CYADD_POINT_ONE
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#endif
            }
        }
    }
    data->touchs = touchs;

    if (va_reported)
    {
        /* touchs==0, there's no point but key */
        if (0 == down_points)
        {
            //AXS_DEBUG("[B] BTN_TOUCH UP!");
            input_report_key(data->input_dev, BTN_TOUCH, 0);
#ifdef CYADD_POINT_ONE
			input_report_abs(data->input_dev, ABS_PRESSURE, 0);
#endif
			
        }
        else
        {
            //AXS_DEBUG("[B] BTN_TOUCH DOWN!");
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }
    //AXS_DEBUG("[B] report touch end");

    input_sync(data->input_dev);
    return 0;
}

#else
static int axs_input_report_a(struct axs_ts_data *data)
{
    int i = 0;
    int down_points = 0;
    int up_points = 0;
    bool va_reported = false;
    struct ts_event *events = data->events;
    //AXS_DEBUG(KERN_EMERG "axs_input_report_a touch_point:%d\n",data->touch_point);
    for (i = 0; i < data->touch_point; i++)
    {
#ifdef HAVE_TOUCH_KEY
        if (axs_input_report_key(data, i) == 0)
        {
            continue;
        }
#endif
        va_reported = true;
        //AXS_DEBUG(KERN_EMERG "axs_input_report_a events[i].flag:%d\n",events[i].flag);
        if (EVENT_DOWN(events[i].flag))
        {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if AXS_REPORT_PRESSURE_EN
            if (events[i].weight <= 0)
            {
                events[i].weight = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].weight);
#endif
            if (events[i].area <= 0)
            {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            AXS_DEBUG("[A]P%d(%d, %d)[weight:%d,area:%d] DOWN!",
                      events[i].id,
                      events[i].x, events[i].y,
                      events[i].weight, events[i].area);
            down_points++;
        }
        else
        {
            up_points++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !down_points)
    {
        va_reported = true;
    }
    data->touchs = down_points;

    if (va_reported)
    {
        if (0 == down_points)
        {
            AXS_DEBUG("[A]Points All Up!");
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        }
        else
        {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif
/*
int axs_wait_tp_to_valid(void)
{
    msleep(2);
    return 0;
}*/
#if AXS_INTERRUPT_THREAD
static void axs_ts_report_thread_handler(void)
{
    int ret = 0;
    struct axs_ts_data *ts_data = g_axs_data;
    //AXS_DEBUG("TP interrupt ");
#if AXS_ESD_CHECK_EN
    g_axs_data->tp_report_interval = 1;
#endif
#if AXS_PEN_EVENT_CHECK_EN
    g_axs_data->tp_report_interval_500ms = 0;
#endif
    ret = axs_read_parse_touchdata(ts_data);

#if AXS_GESTURE_EN
    if (ret > 0)
    {
        struct input_dev *input_dev = ts_data->input_dev;
        axs_gesture_report(input_dev, ret);
    }
    else
#endif
    if (ret == 0)
    {
        mutex_lock(&ts_data->report_mutex);
#if AXS_MT_PROTOCOL_B_EN
        axs_input_report_b(ts_data);
#else
        axs_input_report_a(ts_data);
#endif
        mutex_unlock(&ts_data->report_mutex);
    }

}
#else
static void axs_ts_worker(struct work_struct *work)
{
    int ret = 0;
    struct axs_ts_data *ts_data = g_axs_data;

#if AXS_ESD_CHECK_EN
    g_axs_data->tp_report_interval = 1;
#endif
#if AXS_PEN_EVENT_CHECK_EN
    g_axs_data->tp_report_interval_500ms = 0;
#endif
    ret = axs_read_parse_touchdata(ts_data);
#if AXS_GESTURE_EN
    if (ret > 0)
    {
        struct input_dev *input_dev = ts_data->input_dev;
        axs_gesture_report(input_dev, ret);
    }
    else
#endif
    if (ret == 0)
    {
        mutex_lock(&ts_data->report_mutex);
#if AXS_MT_PROTOCOL_B_EN
        axs_input_report_b(ts_data);
#else
        axs_input_report_a(ts_data);
#endif
        mutex_unlock(&ts_data->report_mutex);
    }

    enable_irq(g_axs_data->irq);
}
#endif


static irqreturn_t axs_ts_interrupt(int irq, void *dev_id)
{
#if AXS_INTERRUPT_THREAD
    //AXS_DEBUG("axs_ts_interrupt");
    /*avoid writing i2c in upgrade period*/
    if (g_axs_data->fw_loading)
    {
        return IRQ_HANDLED;
    }
    axs_ts_report_thread_handler();
    return IRQ_HANDLED;
#else
    struct axs_ts_data *ts_data = (struct axs_ts_data *)dev_id;
    /*avoid writing i2c in upgrade period*/
    if (g_axs_data->fw_loading)
    {
        return IRQ_HANDLED;
    }
    disable_irq_nosync(g_axs_data->irq);
    if (!work_pending(&ts_data->pen_event_work))
    {
        queue_work(ts_data->ts_workqueue, &ts_data->pen_event_work);
    }
    return IRQ_HANDLED;
#endif
}

void axs_irq_disable(void)
{
    unsigned long irqflags;

    spin_lock_irqsave(&g_axs_data->irq_lock, irqflags);

    if (!g_axs_data->irq_disabled)
    {
        disable_irq_nosync(g_axs_data->irq);
        g_axs_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&g_axs_data->irq_lock, irqflags);
}

void axs_irq_enable(void)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&g_axs_data->irq_lock, irqflags);

    if (g_axs_data->irq_disabled)
    {
        enable_irq(g_axs_data->irq);
        g_axs_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&g_axs_data->irq_lock, irqflags);
}

int axs_ts_suspend(struct device *dev)
{
    struct axs_ts_data *ts_data = g_axs_data;
    AXS_DEBUG("axs_ts_suspend begin");
    if (ts_data->suspended)
    {
        AXS_DEBUG("Already in suspend state");
        return 0;
    }
#if AXS_ESD_CHECK_EN
    axs_esd_check_suspend();
#endif
#if AXS_PEN_EVENT_CHECK_EN
    axs_pen_event_check_suspend();
#endif

    if (ts_data->fw_loading)
    {
        AXS_DEBUG("tp in upgrade state, can't suspend");
        return 0;
    }
#if AXS_GESTURE_EN
    if (!ts_data->gesture_enable)
#endif
    {
        axs_irq_disable();
        //axs_reset_level(0);
    }
    axs_release_all_finger();
    ts_data->suspended = true;

    AXS_DEBUG("axs_ts_suspend end");
    return 0;
}

#if AXS_POWERUP_CHECK_EN
void axs_powerup_check(void)
{
    int ret = 0;
    u8 val[2] = { 0 };
    ret = axs_read_fw_version(&val[0]);
    if (ret < 0)
    {
        AXS_DEBUG("axs_read_fw_version fail");
        //axs_reset_proc(20);
    }
}
#endif
int axs_ts_resume(struct device *dev)
{
    struct axs_ts_data *ts_data = g_axs_data;
    AXS_DEBUG("axs_ts_resume begin");
    if (!ts_data->suspended)
    {
        AXS_DEBUG("Already in awake state");
        return 0;
    }
    axs_release_all_finger();
#if AXS_DOWNLOAD_APP_EN
    if (!axs_download_init())
    {
        AXS_ERROR("axs_download_init fail!\n");
    }
#else
    //axs_reset_proc(20);
#if AXS_POWERUP_CHECK_EN
    axs_powerup_check();
#endif

#endif
#if AXS_ESD_CHECK_EN
    axs_esd_check_resume();
#endif
#if AXS_PEN_EVENT_CHECK_EN
    axs_pen_event_check_resume();
#endif

    axs_irq_enable();
    ts_data->suspended = false;

    AXS_DEBUG("axs_ts_resume end");
    return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void axs_ts_early_suspend(struct early_suspend *handler)
{
    axs_ts_suspend(g_axs_data->dev);
}

static void axs_ts_late_resume(struct early_suspend *handler)
{

    axs_ts_resume(g_axs_data->dev);
}
#endif


static void axs_ts_hw_init(struct axs_ts_data *ts_data)
{
#if AXS_POWER_SOURCE_CUST_EN
    struct regulator *reg_vdd =  NULL;
    struct i2c_client *client = ts_data->client;
#endif
    struct axs_ts_platform_data *pdata = ts_data->pdata;
    AXS_DEBUG("%s [irq=%d];[rst=%d]\n",__func__,pdata->irq_gpio,pdata->reset_gpio);
    gpio_request(pdata->irq_gpio, "ts_irq_pin");
    gpio_request(pdata->reset_gpio, "ts_rst_pin");
    //gpio_direction_output(pdata->reset_gpio, 1);
    gpio_direction_input(pdata->irq_gpio);

    //ts_data->irq = gpio_to_irq(pdata->irq_gpio);

#if AXS_POWER_SOURCE_CUST_EN
    reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
    regulator_set_voltage(reg_vdd, 2800000, 2800000);
    regulator_enable(reg_vdd);
#endif
    msleep(120);
    //axs_reset_proc(20);
}

#ifdef CONFIG_OF
static  bool axs_platform_data_init(struct axs_ts_data *ts_data,struct device *dev)
{
    struct axs_ts_platform_data *pdata = ts_data->pdata;
#if 1 //cyadd
	 pdata->reset_gpio = GPIO_PA(15);
	 pdata->irq_gpio = GPIO_PA(13);
	 AXS_DEBUG("  %-12s: %d", "rst gpio", pdata->reset_gpio);
	 AXS_DEBUG("  %-12s: %d", "int gpio", pdata->irq_gpio);
	 ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    if (ts_data->irq < 0)
    {
        AXS_ERROR("Parse irq failed %d", ts_data->irq);
        return false;
    }
	AXS_DEBUG("  %-12s: %d", "irq num", ts_data->irq);
#else
    struct device_node *np = dev->of_node;
#if AXS_POWER_SOURCE_CUST_EN
    int ret;
#endif
    if (!np)
    {
        AXS_ERROR("axs_platform_data_init !np");
        return false;
    }
    pdata->reset_gpio = of_get_named_gpio(np, "touch,reset-gpio", 0);

    //pdata->reset_gpio = of_get_gpio(np, 0);
    if (!gpio_is_valid(pdata->reset_gpio))
    {
        AXS_ERROR("Parse RST GPIO from dt failed %d", pdata->reset_gpio);
        pdata->reset_gpio = -1;
        return false;
    }
    AXS_DEBUG("  %-12s: %d", "rst gpio", pdata->reset_gpio);
    pdata->irq_gpio = of_get_named_gpio(np, "touch,irq-gpio", 0);
    //pdata->irq_gpio = of_get_gpio(np, 1);
    if (!gpio_is_valid(pdata->irq_gpio))
    {
        AXS_ERROR("Parse INT GPIO from dt failed %d", pdata->irq_gpio);
        pdata->irq_gpio = -1;
        return false;
    }
    AXS_DEBUG("  %-12s: %d", "int gpio", pdata->irq_gpio);
    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    if (ts_data->irq < 0)
    {
        AXS_ERROR("Parse irq failed %d", ts_data->irq);
        return false;
    }
    AXS_DEBUG("  %-12s: %d", "irq num", ts_data->irq);
#if AXS_POWER_SOURCE_CUST_EN
    ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
    if (ret)
    {
        AXS_ERROR("fail to get vdd_name\n");
        return false;
    }
#endif
#endif
    return true;
}
#endif


static int axs_report_buffer_init(struct axs_ts_data *ts_data)
{
    int point_num = AXS_MAX_TOUCH_NUMBER;
    ts_data->pnt_buf_size = point_num * AXS_ONE_TCH_LEN + 3;
    return 0;
}

static void axs_ts_probe_entry(struct axs_ts_data *ts_data)
{
    int ret = 0;

#if AXS_GESTURE_EN
    if (!axs_gesture_init(ts_data))
    {
        AXS_ERROR("init gesture fail");
    }
#else
    ts_data->gesture_enable = 0;
#endif

#if AXS_AUTO_UPGRADE_EN
    if (!axs_fwupg_init())
    {
        AXS_ERROR("init fw upgrade fail!\n");
    }
#elif AXS_DOWNLOAD_APP_EN
    if (!axs_download_init())
    {
        AXS_ERROR("axs_download_init fail!\n");
    }
#endif

#if AXS_DEBUG_PROCFS_EN
    ret = axs_create_proc_file(ts_data);
    if (ret)
    {
        AXS_ERROR("axs_create_proc_file fail");
    }
#endif

#if AXS_DEBUG_SYSFS_EN
    if (!axs_debug_create_sysfs(ts_data))
    {
        AXS_ERROR("axs_debug_create_sysfs fail!\n");
    }
#endif
#if AXS_ESD_CHECK_EN
    ret = axs_esd_check_init(ts_data);
    if (ret)
    {
        AXS_ERROR("init esd check fail");
    }
#endif
#if AXS_PEN_EVENT_CHECK_EN
    ret = axs_pen_event_check_init(ts_data);
    if (ret)
    {
        AXS_ERROR("init pen event check fail");
    }
#endif

}

static int axs_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct axs_ts_data *ts_data = NULL;
    int err = 0;
    int pdata_size;

    AXS_DEBUG("axs_ts_probe...");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        AXS_ERROR("I2C not supported");
        return -ENODEV;
    }

    /*
    if (client->addr != AXS_I2C_SLAVE_ADDR) {
        AXS_DEBUG("[TPD]Change i2c addr 0x%02x to %x",
                 client->addr, AXS_I2C_SLAVE_ADDR);
        client->addr = AXS_I2C_SLAVE_ADDR;
        AXS_DEBUG("[TPD]i2c addr=0x%x\n", client->addr);
    }*/

    if (client->addr != AXS_I2C_SLAVE_ADDR)
    {
        AXS_DEBUG("[TPD]i2c addr=0x%x\n", client->addr);
        return -ESRCH;
    }

    ts_data = (struct axs_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data)
    {
        AXS_ERROR("allocate memory for g_axs_data fail");
        return -ENOMEM;
    }

    g_axs_data = ts_data;

    ts_data->client = client;
    ts_data->dev = &client->dev;
    ts_data->bus_type = BUS_TYPE_I2C;
    //i2c_set_clientdata(client, ts_data);

    // axs_ts_probe_entry begin
    pdata_size = sizeof(struct axs_ts_platform_data);
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata)
    {
        AXS_ERROR("allocate memory for platform_data fail");
        goto exit_release_g_axs_data;
    }

#ifdef CONFIG_OF
    if (!axs_platform_data_init(ts_data,&client->dev))
    {
        AXS_ERROR("axs_platform_data_init fail");
        goto exit_release_pdata;
    }
#endif

    axs_ts_hw_init(ts_data);

#if !AXS_INTERRUPT_THREAD
    INIT_WORK(&ts_data->pen_event_work, axs_ts_worker);
#endif

    ts_data->ts_workqueue = create_singlethread_workqueue("axs_wq");
    if (!ts_data->ts_workqueue)
    {
        AXS_DEBUG("create ts_workqueue fail\n");
        err = -ESRCH;
        goto exit_release_gpio;
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->bus_mutex);

    err = axs_bus_init(ts_data);
    if (err)
    {
        AXS_ERROR("bus initialize fail");
        goto exit_release_bus;
    }
    //maybe read error before download/upgrade;do not move this code;
    axs_fwupg_get_ver_in_tp(&g_axs_fw_ver);
    AXS_DEBUG("firmware version=0x%2x\n",g_axs_fw_ver);

    if (axs_input_init(ts_data))
    {
        err = -ENOMEM;
        AXS_DEBUG("failed to allocate input device\n");
        goto exit_release_bus;
    }

    err = axs_report_buffer_init(ts_data);
    if (err)
    {
        AXS_ERROR("report buffer init fail");
        goto exit_release_input;
    }

    axs_ts_probe_entry(ts_data);


#if defined(CONFIG_HAS_EARLYSUSPEND)
    AXS_DEBUG("register_early_suspend");
    g_axs_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
    g_axs_data->early_suspend.suspend = axs_ts_early_suspend;
    g_axs_data->early_suspend.resume    = axs_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif
    AXS_DEBUG("%s IRQ number is %d", client->name, g_axs_data->irq);
#if AXS_INTERRUPT_THREAD
    err = request_threaded_irq(g_axs_data->irq, NULL, axs_ts_interrupt, IRQF_TRIGGER_FALLING|IRQF_ONESHOT, AXS_DRIVER_NAME, g_axs_data);
    if (err)
    {
        AXS_ERROR("axs_probe: request irq failed\n");
        goto exit_release_input;
    }
#else
    err = request_irq(g_axs_data->irq, axs_ts_interrupt, IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND, AXS_DRIVER_NAME, g_axs_data);
    if (err)
    {
        AXS_ERROR("axs_probe: request irq failed\n");
        goto exit_release_input;
    }
#endif





    AXS_DEBUG("probe successfully");
    return 0;

exit_release_input:
    input_free_device(ts_data->input_dev);
exit_release_bus:

    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
//exit_release_workqueue:
#if !AXS_INTERRUPT_THREAD
    cancel_work_sync(&ts_data->pen_event_work);
#endif
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
exit_release_gpio:
    gpio_free(ts_data->pdata->reset_gpio);
    gpio_free(ts_data->pdata->irq_gpio);
exit_release_pdata:
    kfree_safe(ts_data->pdata);
exit_release_g_axs_data:
    kfree_safe(g_axs_data);
    return err;
}


static const struct of_device_id axs_of_match[] =   // custom 1
{
    {.compatible = "axs,axs_touch",},
    { }
};

MODULE_DEVICE_TABLE(of, axs_of_match);

static int axs_ts_remove(struct i2c_client *client)
{
    AXS_DEBUG("axs_ts_remove");
#if AXS_DEBUG_SYSFS_EN
    axs_debug_remove_sysfs(g_axs_data);
#endif

#if AXS_DEBUG_PROCFS_EN
    axs_release_proc_file(g_axs_data);
#endif

    axs_bus_exit(g_axs_data);

#if defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&g_axs_data->early_suspend);
#endif

//  device_init_wakeup(&client->dev, 0);

    free_irq(g_axs_data->irq, g_axs_data);
#if !AXS_INTERRUPT_THREAD
    cancel_work_sync(&g_axs_data->pen_event_work);
#endif
    destroy_workqueue(g_axs_data->ts_workqueue);
    input_unregister_device(g_axs_data->input_dev);

    kfree_safe(g_axs_data->pdata);
    kfree_safe(g_axs_data);
    return 0;
}

static const struct dev_pm_ops axs_dev_pm_ops =
{
    .suspend = axs_ts_suspend,
    .resume  = axs_ts_resume,
};
#if AXS_BUS_IIC
static const struct i2c_device_id axs_ts_id[] =
{
    {AXS_DRIVER_NAME, 0},{ }
};
MODULE_DEVICE_TABLE(i2c, axs_ts_id);
#endif
static struct i2c_driver axs_ts_driver =
{
    .probe      = axs_ts_probe,
    .remove     = axs_ts_remove,
    .id_table   = axs_ts_id,
    .driver = {
        .name   = AXS_DRIVER_NAME,
        .owner  = THIS_MODULE,
        .pm = &axs_dev_pm_ops,
        .of_match_table = axs_of_match,
    },
};

static int __init axs_ts_init(void)
{
    AXS_DEBUG("axs_ts_init Driver version: %s", AXS_DRIVER_VERSION);

    return  i2c_add_driver(&axs_ts_driver);
}

static void __exit axs_ts_exit(void)
{
    i2c_del_driver(&axs_ts_driver);
}

module_init(axs_ts_init);
module_exit(axs_ts_exit);

MODULE_AUTHOR("AiXieSheng Technology.");
MODULE_DESCRIPTION("AXS TouchScreen Driver");
MODULE_LICENSE("GPL");





