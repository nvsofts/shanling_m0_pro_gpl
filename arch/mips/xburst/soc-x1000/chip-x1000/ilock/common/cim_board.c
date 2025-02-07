/*
 * [board]-cim.c - This file defines camera host driver (cim) on the board.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: xiaoyangfu <xyfu@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <media/soc_camera.h>
#include <mach/platform.h>
#include <mach/jz_camera.h>
#include <linux/regulator/machine.h>
#include <gpio.h>
#include "board_base.h"

#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V13
struct jz_camera_pdata mensa_camera_pdata = {
    .mclk_10khz = 2400,
    .flags = 0,
    .cam_sensor_pdata[FRONT_CAMERA_INDEX] = {
        .gpio_rst = CAMERA_SENSOR_RESET,
        .gpio_power = CAMERA_FRONT_SENSOR_PWDN,
        .gpio_en = CAMERA_VDD_EN,
    },
};

static int camera_sensor_reset(struct device *dev) {
#ifdef CAMERA_SENSOR_RESET
    if (gpio_is_valid(CAMERA_VDD_EN)) {
        gpio_direction_output(CAMERA_VDD_EN, 1);
    }
#endif
    if (gpio_is_valid(CAMERA_SENSOR_RESET)) {
        gpio_direction_output(CAMERA_SENSOR_RESET, 0);
        mdelay(250);
        gpio_direction_output(CAMERA_SENSOR_RESET, 1);
        mdelay(250);
    }

    return 0;
}
static int front_camera_sensor_power(struct device *dev, int on) {
    /* enable or disable the camera */
    if (gpio_is_valid(CAMERA_FRONT_SENSOR_PWDN)) {
        gpio_direction_output(CAMERA_FRONT_SENSOR_PWDN, on ? 0 : 1);
        mdelay(150);
    }

    return 0;
}

static struct soc_camera_link iclink_front = {
    .bus_id         = 0,        /* Must match with the camera ID */
    .board_info     = &jz_v4l2_camera_devs[FRONT_CAMERA_INDEX],
    .i2c_adapter_id = 0,
    .power          = front_camera_sensor_power,
    .reset          = camera_sensor_reset,
};

struct platform_device mensa_front_camera_sensor = {
    .name    = "soc-camera-pdrv",
    .id      = -1,
    .dev     = {
        .platform_data = &iclink_front,
    },
};
#endif

static int __init mensa_board_cim_init(void) {
    /* camera host */

#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V13
    jz_device_register(&jz_cim_device, &mensa_camera_pdata);
#endif
    /* camera sensor */
#ifdef CONFIG_SOC_CAMERA
    platform_device_register(&mensa_front_camera_sensor);
#endif

    return 0;
}

//arch_initcall(mensa_board_cim_init);
core_initcall(mensa_board_cim_init);
