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

#if AXS_ESD_CHECK_EN
#define ESD_CHECK_DELAY_TIME              1000 /*unit:ms*/
static u8 g_i2c_error_times = 0;
static u8 g_esd_status_0_times = 0;

//u8 g_esd_work_init_flag = 0;
static void esd_check_func(struct work_struct *work)
{
    int ret = 0;
    u8 esd_value[1];
    mutex_lock(&g_axs_data->report_mutex);
    if (g_axs_data->tp_report_interval)
    {
        g_axs_data->tp_report_interval += 1;
        if (g_axs_data->tp_report_interval>=5)
        {
            g_axs_data->tp_report_interval = 0;
        }
    }

    if (g_axs_data->fw_loading || g_axs_data->tp_report_interval ||(!g_axs_data->esd_check_enable))
    {
        mutex_unlock(&g_axs_data->report_mutex);
        AXS_DEBUG("skip esd_check_func,fw_loading:%d,interva:%d,esd_check_enable:%d",g_axs_data->fw_loading,g_axs_data->tp_report_interval,g_axs_data->esd_check_enable);
    }
    else
    {
        ret = axs_read_sfr_reg(0x8A,esd_value,1);
        AXS_DEBUG("read esd_value:%x,ret:%d",esd_value[0],ret);
        mutex_unlock(&g_axs_data->report_mutex);
        if (ret >= 0)
        {
            if (esd_value[0] == 0xA5) /*reset when read 0xa5 times>= 1 */
            {
                g_axs_reset_flag = 1;
                g_esd_status_0_times = 0;
            }
            else if (esd_value[0] == 0x00)
            {
                g_esd_status_0_times++;
                if (g_esd_status_0_times >=2) /*reset when read 0x00 times>= 2 */
                {
                    g_axs_reset_flag = 1;
                }
            }

            g_i2c_error_times = 0;
        }
        else //i2c/spi error
        {
            g_i2c_error_times++;
            AXS_DEBUG("g_i2c_error_times:%d",g_i2c_error_times);
            if (g_i2c_error_times>=3) /*reset when i2c/spi error times>= 3*/
            {
                g_axs_reset_flag = 1;
            }
        }

        if (g_axs_reset_flag)
        {
            axs_release_all_finger();
            g_i2c_error_times = 0;
            g_esd_status_0_times = 0;
        }
    }
    queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->esd_check_work,
                       msecs_to_jiffies(ESD_CHECK_DELAY_TIME));
    AXS_FUNC_EXIT();
}

int axs_esd_check_init(struct axs_ts_data *ts_data)
{
    AXS_FUNC_ENTER();

    if (ts_data->ts_workqueue)
    {
        INIT_DELAYED_WORK(&ts_data->esd_check_work, esd_check_func);
    }
    else
    {
        AXS_ERROR("ts_workqueue is NULL");
        return -EINVAL;
    }

    g_axs_data->esd_check_enable = 1;

    queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->esd_check_work,
                       msecs_to_jiffies(ESD_CHECK_DELAY_TIME));

    AXS_FUNC_EXIT();
    return 0;
}

int axs_esd_check_suspend(void)
{
    AXS_FUNC_ENTER();
    if (g_axs_data->esd_check_enable) // g_esd_work_init_flag
    {
        cancel_delayed_work(&g_axs_data->esd_check_work);
    }
    AXS_FUNC_EXIT();
    return 0;
}

int axs_esd_check_resume( void )
{
    AXS_FUNC_ENTER();
    if (g_axs_data->esd_check_enable) // g_esd_work_init_flag
    {
        queue_delayed_work(g_axs_data->ts_workqueue, &g_axs_data->esd_check_work,
                           msecs_to_jiffies(ESD_CHECK_DELAY_TIME));
    }
    AXS_FUNC_EXIT();
    return 0;
}
#endif

