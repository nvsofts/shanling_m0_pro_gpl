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

#ifndef __LINUX_AXS_CORE_H__
#define __LINUX_AXS_CORE_H__
#include "axs_platform.h"
#include "axs_config.h"

#define AXS_DRIVER_VERSION                  "V1.9.0"
#define AXS_DRIVER_NAME "axs_ts"

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

#define vfree_safe(pbuf) do {\
    if (pbuf) {\
        vfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

/*MAX SUPPORT POINTS*/
#define AXS_MAX_TOUCH_NUMBER    1
#define SCREEN_MAX_X    180
#define SCREEN_MAX_Y    640

#if defined(HAVE_TOUCH_KEY)
#define AXS_MAX_KEYS                        4

#define KEY_ARRAY   \
    {KEY_MENU, 30,50 ,970, 1000},  \
    {KEY_HOMEPAGE, 110, 130, 970, 1000}, \
    {KEY_BACK, 190, 210, 970, 1000}
#endif

#define AXS_ONE_TCH_LEN         6

//POS 0:read reg;  1:gesture id;  2:point num;  3:event+X_H;  4:X_L;  5:ID+Y_H;  6:Y_L;  7:WEIGHT;  8:AREA;
#define AXS_MAX_ID                          0x05
#define AXS_TOUCH_GESTURE_POS               1
#define AXS_TOUCH_POINT_NUM                 2
#define AXS_TOUCH_EVENT_POS                 3
#define AXS_TOUCH_X_H_POS                   3
#define AXS_TOUCH_X_L_POS                   4
#define AXS_TOUCH_ID_POS                    5
#define AXS_TOUCH_Y_H_POS                   5
#define AXS_TOUCH_Y_L_POS                   6
#define AXS_TOUCH_WEIGHT_POS                7
#define AXS_TOUCH_AREA_POS                  8

#define AXS_TOUCH_DOWN                      0
#define AXS_TOUCH_UP                        1
#define AXS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((AXS_TOUCH_DOWN == flag) || (AXS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (AXS_TOUCH_UP == flag)
//#define EVENT_NO_DOWN(data)                 (!data->point_num)

#define AXS_REG_ESD_READ                0X0B
#define AXS_REG_VERSION_READ            0X0C        /*����??firware��?��?*/
#define AXS_REG_GESTURE_READ            0X0D        /*����??��?��?��1?��*/
#define AXS_REG_GESTURE_WRITE           0X0E        /*����??��?��?��1?��*/

#define AXS_FREG_RAWDATA_READ           0X02
#define AXS_FREG_TXRX_NUM_READ          0X03
#define AXS_FREG_DIFF_READ              0X82
#define AXS_FREG_DEBUG_LEN_READ         0X83
#define AXS_FREG_DEBUG_STR_READ         0X84
#define AXS_FREG_TRACE_MODE_WRITE       0X85

struct key_data
{
    u16 key;
    u16 x_min;
    u16 x_max;
    u16 y_min;
    u16 y_max;
};

struct axs_ts_platform_data
{
    int irq_gpio;
    int reset_gpio;
    //int power_en_gpio_number;
    const char *vdd_name;
    int TP_MAX_X;
    int TP_MAX_Y;
};

struct ts_event
{
    int x;      /*x coordinate */
    int y;      /*y coordinate */
    int weight;      /* pressure */
    int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;     /*touch ID */
    int area;
};

struct axs_ts_data
{
    struct i2c_client   *client;
    struct spi_device *spi;
    struct device *dev;
    struct input_dev    *input_dev;
    struct axs_ts_platform_data *pdata;
    struct workqueue_struct *ts_workqueue;
    struct work_struct fwupg_work;
    struct work_struct  pen_event_work;
    struct delayed_work esd_check_work;
    struct delayed_work pen_event_check_work;

    spinlock_t irq_lock;
    struct mutex report_mutex;
    struct mutex bus_mutex;
    int irq;
    bool suspended;
    bool irq_disabled;
    u8 gesture_enable;      /* gesture enable or disable, default: disable */
    u8 esd_check_enable;
    u8 pen_event_check_enable;

    struct ts_event events[AXS_MAX_TOUCH_NUMBER];
    u8 *bus_tx_buf;
    u8 *bus_rx_buf;
    u8 *debug_tx_buf;
    u8 *debug_rx_buf;
    int bus_type;
    u8 point_buf[AXS_MAX_TOUCH_NUMBER*AXS_ONE_TCH_LEN+4];
    int pnt_buf_size;
    int touchs;
    int key_state;
    int touch_point;
//    int point_num ;
#if defined(CONFIG_FB)
    struct work_struct resume_work;
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_suspend;
#endif
    bool fw_loading;
    u8 tp_report_interval;
    u8 tp_report_interval_500ms;

    struct proc_dir_entry *proc_dir;
    u8 proc_opmode;
};

extern u8 g_axs_fw_ver;
extern u8 g_axs_reset_flag;
#if AXS_DEBUG_LOG_EN
extern u8 g_log_en;

#define AXS_DEBUG(fmt, args...) if(g_log_en){ \
   printk("[AXS]%s:"fmt"\n", __func__, ##args); \
}

#define AXS_FUNC_ENTER() if(g_log_en){ \
  printk("[AXS]%s: Enter\n", __func__); \
}

#define AXS_FUNC_EXIT() if(g_log_en){ \
    printk("[AXS]%s: Exit(%d)\n", __func__, __LINE__); \
}

#define AXS_ERROR(fmt, args...) if(g_log_en){ \
    printk(KERN_ERR "[AXS ERROR]%s:"fmt"\n", __func__, ##args); \
}

#else /* #if AXS_DEBUG_LOG_EN*/
#define AXS_DEBUG(fmt, args...)
#define AXS_FUNC_ENTER()
#define AXS_FUNC_EXIT()
#define AXS_ERROR(fmt, args...)
#endif


enum _AXS_BUS_TYPE
{
    BUS_TYPE_NONE,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_SPI_V2,
};

extern struct axs_ts_data *g_axs_data;

int axs_write_read(u8 *cmd, u16 cmdlen, u8 *data, u16 datalen);
int axs_write(u8 *writebuf, u16 writelen);
int axs_read(u8 *rdbuf, u16 rdlen);
int axs_read_sfr_reg(u8 sfr_reg,u8 *rd_data,u16 rd_len);
#if AXS_BUS_SPI
int axs_write_read_onecs(u8 *cmd, u16 cmdlen, u8 *data, u16 datalen);
#endif

int axs_bus_init(struct axs_ts_data *ts_data);
int axs_bus_exit(struct axs_ts_data *ts_data);

#if AXS_GESTURE_EN
bool axs_gesture_init(struct axs_ts_data *ts_data);
void axs_gesture_report(struct input_dev *input_dev, int gesture_id);
int axs_read_gesture_enable(u8 *enable);
int axs_write_gesture_enable(u8 enable);
#endif

int axs_read_fw_version(u8 *ver);

#if AXS_AUTO_UPGRADE_EN
bool axs_fwupg_init(void);
#endif

#if AXS_DEBUG_SYSFS_EN
bool axs_debug_create_sysfs(struct axs_ts_data *ts_data);
int axs_debug_remove_sysfs(struct axs_ts_data *ts_data);
int axs_read_file(char *file_name, u8 **file_buf);
void axs_debug_download_app(char *fw_name);
void axs_debug_fwupg(char *fw_name);
#endif

#if AXS_DEBUG_PROCFS_EN
int axs_create_proc_file(struct axs_ts_data *);
void axs_release_proc_file(struct axs_ts_data *);
#endif

#if AXS_DOWNLOAD_APP_EN
bool axs_download_init(void);
#endif

void axs_reset_proc(int hdelayms);
//int axs_wait_tp_to_valid(void);
void axs_release_all_finger(void);

void axs_irq_disable(void);
void axs_irq_enable(void);

void axs_reset_level(u8 level);
#if AXS_ESD_CHECK_EN
int axs_esd_check_init(struct axs_ts_data *ts_data);
int axs_esd_check_suspend(void);
int axs_esd_check_resume(void);
#endif
int axs_write_read_reg(u8 *reg,u16 reg_len,u8 *rd_data,u16 rd_len);
bool axs_fwupg_get_ver_in_tp(u8 *ver);
#endif /* __LINUX_AXS_CORE_H__ */

