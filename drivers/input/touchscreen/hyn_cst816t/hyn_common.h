/*
 *
 * hyntrion fts TouchScreen driver.
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
* File Name: hyn_common.h
*
* Author: hyntrion Driver Team
*
* Created: 2016-08-16
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_HYN_COMMON_H__
#define __LINUX_HYN_COMMON_H__

//#include "hyn_core.h"

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define HYN_DRIVER_VERSION                  "hyntrion V3.0 20190815"

#define HYN_CHIP_IDC            81
#define HYN_HID_SUPPORTTED      0

#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
/*register address*/
#define HYN_REG_CHIP_ID                     0xAA
#define HYN_REG_CHIP_ID2                    0xAB
#define HYN_REG_POWER_MODE                  0xA5
#define HYN_REG_POWER_MODE_SLEEP_VALUE      0x03
#define HYN_REG_FW_VER                      0xA9
#define HYN_REG_VENDOR_ID                   0xA8

#define HYN_REG_FACE_DEC_MODE_EN            0xB0
#define HYN_REG_FACTORY_MODE_DETACH_FLAG    0x04
#define HYN_REG_FACE_DEC_MODE_STATUS        0x01
#define HYN_REG_GESTURE_EN                  0xD0
#define HYN_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_LIC_VER                     0xE4

#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

//gpio config
#define HYN_RST_PORT    GPIO_PA(19)// @ysc S5PV210_GPJ3(6)
#define HYN_INT_PORT    GPIO_PA(16)// @ysc S5PV210_GPH1(3)

/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * point report check
 * default: disable
 */

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/
#if HYN_DEBUG_EN
#define HYN_DEBUG(fmt, args...) do { \
    printk("[HYN_TS]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define HYN_FUNC_ENTER() do { \
    printk("[HYN_TS]%s: Enter\n", __func__); \
} while (0)

#define HYN_FUNC_EXIT() do { \
    printk("[HYN_TS]%s: Exit(%d)\n", __func__, __LINE__); \
} while (0)
#else /* #if HYN_DEBUG_EN*/
#define HYN_DEBUG(fmt, args...)
#define HYN_FUNC_ENTER()
#define HYN_FUNC_EXIT()
#endif

#define HYN_INFO(fmt, args...) do { \
    printk(KERN_INFO "[HYN_TS/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define HYN_ERROR(fmt, args...) do { \
    printk(KERN_ERR "[HYN_TS/E]%s:"fmt"\n", __func__, ##args); \
} while (0)
#endif /* __LINUX_FOCALTECH_COMMON_H__ */
