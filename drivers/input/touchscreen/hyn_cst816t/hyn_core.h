/*
 *
 * hyntrion TouchScreen driver.
 *
 * Copyright (c) 2012-2019, hyntrion Ltd. All rights reserved.
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
* File Name: hyntrion_core.h

* Author: hyntrion Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_HYN_CORE_H__
#define __LINUX_HYN_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>


/******************* Enables *********************/
/*********** 1 to enable, 0 to disable ***********/

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define HYN_DEBUG_EN                            0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define HYN_MT_PROTOCOL_B_EN                    0

/*
 * Report Pressure in multitouch
 * 1:enable(default),0:disable
*/
#define HYN_REPORT_PRESSURE_EN                  0

/*
 * Gesture function enable
 * default: disable
 */
#define HYN_GESTURE_EN                          0

/*
 * FW updata  function enable
 * default: disable
 */
#define HYN_FW_UPDATA                           1
#define HYN_MTK_IIC_TRANSFER_LIMIT              1


/*
 * Nodes for tools
 */
#define HYN_SYSFS_NODE_EN                       0
#define HYN_APK_NODE_EN                         0

/*
 * Pinctrl enable
 * default: disable
 */
#define HYN_PINCTRL_EN                          0
/*
 * Customer power enable
 * enable it when customer need control TP power
 * default: disable
 */
#define HYN_POWER_SOURCE_CUST_EN                0


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define HYN_MAX_POINTS_SUPPORT              2 
#define HYN_MAX_KEYS                        4
#define HYN_KEY_DIM                         10
#define HYN_ONE_TCH_LEN                     6

#define HYN_MAX_ID                          HYN_MAX_POINTS_SUPPORT


#define HYN_COORDS_ARR_SIZE                 4
#define HYN_X_MIN_DISPLAY_DEFAULT           0
#define HYN_Y_MIN_DISPLAY_DEFAULT           0
#define HYN_X_MAX_DISPLAY_DEFAULT           240
#define HYN_Y_MAX_DISPLAY_DEFAULT           240

#define HYN_TOUCH_DOWN                      0
#define HYN_TOUCH_UP                        1
#define HYN_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((HYN_TOUCH_DOWN == flag) || (HYN_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (HYN_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 (!data->point_num)

#define HYN_MAX_COMPATIBLE_TYPE             4
#define HYN_MAX_COMMMAND_LENGTH             16

#define I2C_RETRY_NUMBER                    3


/****************************************************/


struct hyn_chip_t {
    u64 type;
    u8 chip_id;
    u8 fw_ver;
    u8 rom_idh;
    u8 rom_idl;
    u8 pb_idh;
    u8 pb_idl;
    u8 bl_idh;
    u8 bl_idl;
};

struct ts_ic_info {
    bool is_incell;
    bool hid_supported;
    struct hyn_chip_t ids;
};


/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct cstxxxx_proc {
    struct proc_dir_entry *proc_entry;
    u8 opmode;
    u8 cmd_len;
    u8 cmd[HYN_MAX_COMMMAND_LENGTH];
};

struct hyn_ts_platform_data {
    u32 irq_gpio;
    u32 irq_gpio_flags;
    u32 reset_gpio;
    u32 reset_gpio_flags;
    bool have_key;
    u32 key_number;
    u32 keys[HYN_MAX_KEYS];
    u32 key_y_coords[HYN_MAX_KEYS];
    u32 key_x_coords[HYN_MAX_KEYS];
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 max_touch_number;
};

struct ts_event {
    int x;      /*x coordinate */
    int y;      /*y coordinate */
    int p;      /* pressure */
    int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;     /*touch ID */
    int area;
};

struct hyn_ts_data {
    struct i2c_client *client;
    struct spi_device *spi;
    struct device *dev;
    struct input_dev *input_dev;
    struct hyn_ts_platform_data *pdata;
    struct ts_ic_info ic_info;
    struct workqueue_struct *ts_workqueue;
    struct work_struct fwupg_work;
    struct delayed_work esdcheck_work;
    struct delayed_work prc_work;
    struct work_struct resume_work;
    struct cstxxxx_proc proc;
    spinlock_t irq_lock;
    struct mutex report_mutex;
    struct mutex bus_lock;
    int irq;
    int log_level;
    int fw_is_running;      /* confirm fw is running when using spi:default 0 */
    bool suspended;
    bool fw_loading;
    bool irq_disabled;
    bool power_disabled;
    bool glove_mode;
    bool cover_mode;
    bool charger_mode;
    /* multi-touch */
    struct ts_event *events;
    u8 *bus_buf;
    u8 *point_buf;
    int pnt_buf_size;
    int touchs;
    int key_state;
    int touch_point;
    int point_num;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
#if HYN_PINCTRL_EN
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_active;
    struct pinctrl_state *pins_suspend;
    struct pinctrl_state *pins_release;
#endif
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/



#endif /* __LINUX_FOCALTECH_CORE_H__ */
