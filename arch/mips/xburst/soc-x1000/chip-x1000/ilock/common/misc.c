#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/74hc595.h>
#include <linux/zb_cc2530.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/power/pwm_battery.h>
//#include <linux/android_pmem.h>
#include <mach/platform.h>
#include <mach/jzsnd.h>
#include <mach/jzmmc.h>
#include <mach/jzssi.h>
#include <mach/jz_efuse.h>
#include <mach/jz_uart.h>
#include <gpio.h>
#include <linux/jz_dwc.h>
#include <linux/sgm42609_motor.h>
#include <linux/interrupt.h>
//#include <sound/jz-aic.h>
#include "board_base.h"

#ifdef CONFIG_JZ_MAC
#ifndef CONFIG_MDIO_GPIO
#ifdef CONFIG_JZGPIO_PHY_RESET
static struct jz_gpio_phy_reset gpio_phy_reset = {
	.gpio = GMAC_PHY_PORT_GPIO,
	.active_level = GMAC_PHY_ACTIVE_HIGH,
	.crtl_port = GMAC_CRLT_PORT,
	.crtl_pins = GMAC_CRLT_PORT_PINS,
	.set_func = GMAC_CRTL_PORT_SET_FUNC,
	.delaytime_msec = GMAC_PHY_DELAYTIME,
};
#endif
struct platform_device jz_mii_bus = {
	.name = "jz_mii_bus",
#ifdef CONFIG_JZGPIO_PHY_RESET
	.dev.platform_data = &gpio_phy_reset,
#endif
};
#else /* CONFIG_MDIO_GPIO */
static struct mdio_gpio_platform_data mdio_gpio_data = {
	.mdc = MDIO_MDIO_MDC_GPIO,
	.mdio = MDIO_MDIO_GPIO,
	.phy_mask = 0,
	.irqs = { 0 },
};

struct platform_device jz_mii_bus = {
	.name = "mdio-gpio",
	.dev.platform_data = &mdio_gpio_data,
};
#endif /* CONFIG_MDIO_GPIO */
struct platform_device jz_mac_device = {
	.name = "jz_mac",
	.dev.platform_data = &jz_mii_bus,
};
#endif /* CONFIG_JZ_MAC */


#ifdef CONFIG_JZ_EFUSE_V13
struct jz_efuse_platform_data jz_efuse_pdata = {
	    /* supply 2.5V to VDDQ */
	    .gpio_vddq_en_n = GPIO_EFUSE_VDDQ,
};
#endif


#ifdef CONFIG_LEDS_GPIO
struct gpio_led jz_leds[] = {
#if 0
	[0]={
		.name = "wl_led_r",
		.gpio = WL_LED_R,
		.active_low = 0,
	},
	[1]={
		.name = "wl_led_g",
		.gpio = WL_LED_G,
		.active_low = 0,
	},

	[2]={
		.name = "wl_led_b",
		.gpio = WL_LED_B,
		.active_low = 0,
	},
#endif
};

struct gpio_led_platform_data  jz_led_pdata = {
	.num_leds = ARRAY_SIZE(jz_leds),
	.leds = jz_leds,
};

struct platform_device jz_led_rgb = {
	.name       = "leds-gpio",
	.id     = -1,
	.dev        = {
		.platform_data  = &jz_led_pdata,
	}
};
#endif

#ifdef CONFIG_LEDS_PWM
static struct led_pwm leds_pwm[] = {
    {
        .name = "led_rgb0",
        .default_trigger = NULL,
        .pwm_id     = 0,
        .active_low = false,
        .brightness = 0,
        .max_brightness = 4095,
        .pwm_period_ns  = 30000,
    },
    {
        .name = "led_rgb1",
        .default_trigger = NULL,
        .pwm_id     = 2,
        .active_low = false,
        .brightness = 0,
        .max_brightness = 4095,
        .pwm_period_ns  = 30000,
    },
    {
        .name = "led_rgb2",
        .default_trigger = NULL,
        .pwm_id     = 1,
        .active_low = false,
        .brightness = 4095,
        .max_brightness = 4095,
        .pwm_period_ns  = 30000,
    },
    {
        .name = "led_mire",
        .default_trigger = NULL,
        .pwm_id     = 4,
        .active_low = false,
        .brightness = 0,
        .max_brightness = 4095,
        .pwm_period_ns  = 30000,
    },
};

static struct led_pwm_platform_data led_pwm_info = {
    .num_leds = ARRAY_SIZE(leds_pwm),
    .leds = leds_pwm,
};

struct platform_device jz_leds_pwm = {
    .name = "leds_pwm",
    .id = -1,
    .dev = {
        .platform_data = &led_pwm_info,
    },
};
#endif

#ifdef CONFIG_SGM42609_MOTOR
struct sgm42609_motor_platform_data sgm42609_motor_platform_data0 = {
	.in1_gpio = SGM42609_MOTOR_IN1,
	.in2_gpio = SGM42609_MOTOR_IN2,
	.fault_gpio = SGM42609_MOTOR_FAULT,
	.power_gpio = SGM42609_MOTOR_POWER,
	.power_active_level = SGM42609_MOTOR_POWER_LEVEL,
};

struct platform_device sgm42609_motor_device = {
    .name = "sgm42609_motor",
    .id = 0,
    .dev = {
        .platform_data = &sgm42609_motor_platform_data0,
    },
};
#endif

#ifdef CONFIG_74HC595
static struct sn74hc595_platform_data jz_74hc595_pdata = {
    .en_level = 0,
    .out_bits = 16,
    .init_val = 0xffff,
    .data_pin = GPIO_74HC595_DATA,
    .rclk_pin = GPIO_74HC595_RCLK,
    .sclk_pin = GPIO_74HC595_SCLK,
    .sclr_pin = GPIO_74HC595_SCLR,
    .oe_pin   = GPIO_74HC595_OE,
};

struct platform_device jz_74hc595_dev = {
    .name = "sn74hc595",
    .id = 0,
    .dev = {
        .platform_data = &jz_74hc595_pdata,
    },
};
#endif

#ifdef CONFIG_ZIGBEE_CC2530

static void zb_restore_pin_status(void)
{
    jz_gpio_set_func(GPIO_CC2530_UART_RXD, GPIO_FUNC_1);
    jz_gpio_set_func(GPIO_CC2530_UART_TXD, GPIO_FUNC_1);
}

static void zb_set_pin_status(void)
{
    jz_gpio_set_func(GPIO_CC2530_UART_RXD, GPIO_OUTPUT0);
    jz_gpio_set_func(GPIO_CC2530_UART_TXD, GPIO_OUTPUT0);
}
static struct zb_cc2530_platform_data jz_cc2530_pdata = {
    .en_level   = 1,
    .rst_level  = 0,
    .wake_level = 1,
    .en_pin     = GPIO_CC2530_EN,
    .int_pin    = GPIO_CC2530_INT,
    .wake_pin   = GPIO_CC2530_WAKE,
    .rst_pin    = GPIO_CC2530_RST,
    .restore_pin_status = zb_restore_pin_status,
    .set_pin_status     = zb_set_pin_status,
};

struct platform_device jz_cc2530_dev = {
    .name = "zb_cc2530",
    .dev = {
        .platform_data = &jz_cc2530_pdata,
    },
};
#endif

#ifdef CONFIG_SERIAL_JZ47XX_UART0
struct jz_uart_platform_data jz_uart0_platform_data = {
        .wakeup_pin = NULL,
};
#endif

#ifdef CONFIG_SERIAL_JZ47XX_UART1
struct uart_wakeup_pin uart1_wakeup_pin = {
        .num = -1,
        .trigger_edge = IRQF_TRIGGER_FALLING,
        .def_func = GPIO_FUNC_1,
};

struct jz_uart_platform_data jz_uart1_platform_data = {
        .wakeup_pin = NULL,
};
#endif

#ifdef CONFIG_SERIAL_JZ47XX_UART2
struct jz_uart_platform_data jz_uart2_platform_data = {
        .wakeup_pin = NULL,
};
#endif

#ifdef CONFIG_BATTERY_PWM
struct pwm_battery_platform_data pwm_battery_platform_data = {
    .gpio_usb = GPIO_USB_DETE,
    .gpio_usb_active_low = GPIO_ACTIVE_HIGH,

    .gpio_charger = GPIO_LI_ION_CHARGE,
    .gpio_charger_active_low = GPIO_ACTIVE_LOW,
    .charger_debounce = 0,

    .pwm_id = CONFIG_BATTERY_PWM_INDEX,
    .pwm_active_low = GPIO_ACTIVE_HIGH,

    .gpio_op = GPIO_BATTERY_STATUS,
    .gpio_op_active_low = GPIO_ACTIVE_HIGH,

    .pwm_ref_voltage = 3260,

    .battery_ref_scale = 3,

    .battery_info = {
            .battery_max_cpt = 4000,
            .sleep_current = 20,
    },
};

struct platform_device pwm_battery_device = {
    .name = "pwm-battery",
    .dev = {
        .platform_data = &pwm_battery_platform_data,
    },
};
#endif

#if defined(GPIO_USB_ID) && defined(GPIO_USB_ID_LEVEL)
struct jzdwc_pin dwc2_id_pin = {
	.num = GPIO_USB_ID,
	.enable_level = GPIO_USB_ID_LEVEL,
};
#endif


#if defined(GPIO_USB_DETE) && defined(GPIO_USB_DETE_LEVEL) && (!defined(CONFIG_BATTERY_PWM))
struct jzdwc_pin dwc2_dete_pin = {
	.num = GPIO_USB_DETE,
	.enable_level = GPIO_USB_DETE_LEVEL,
};
#endif


#if defined(GPIO_USB_DRVVBUS) && defined(GPIO_USB_DRVVBUS_LEVEL) && !defined(USB_DWC2_DRVVBUS_FUNCTION_PIN)
struct jzdwc_pin dwc2_drvvbus_pin = {
	.num = GPIO_USB_DRVVBUS,
	.enable_level = GPIO_USB_DRVVBUS_LEVEL,
};
#endif

#if defined(CONFIG_SND_ASOC_INGENIC)

#if defined(CONFIG_SND_ASOC_JZ_EXTCODEC_AKM4951)
struct snd_codec_data snd_alsa_platform_data = {
    .gpio_spk_en = {.gpio = GPIO_AKM4951_SPEAKER_EN, .active_level = GPIO_AKM4951_SPEAKER_EN_LEVEL},
    .gpio_amp_pwr = {.gpio = GPIO_AKM4951_AMP_POWER_EN, .active_level = GPIO_AKM4951_AMP_POWER_EN_LEVEL},
    .gpio_linein_detect = {.gpio = GPIO_AKM4951_LINEIN_DETECT, .active_level = GPIO_AKM4951_LINEIN_INSERT_LEVEL},
    .gpio_spk_mute = {.gpio = GPIO_SPEAKER_MUTE, .active_level = GPIO_SPEAKER_MUTE_EN_LEVEL},
    .gpio_hp_detect = {.gpio = GPIO_AKM4951_HP_DETECT, .active_level = GPIO_AKM4951_HP_INSERT_LEVEL},
    .gpio_hp_mute = {.gpio = GPIO_AKM4951_HP_MUTE, .active_level = GPIO_AKM4951_HP_MUTE_EN_LEVEL},
};

struct platform_device snd_alsa_device = {
    .name = "ingenic-ilock",
    .dev = {
        .platform_data = &snd_alsa_platform_data,
    },
};
#else
static struct snd_codec_data snd_alsa_platform_data = {
	.gpio_spk_en = {.gpio = GPIO_SPEAKER_EN, .active_level = GPIO_SPEAKER_EN_LEVEL},
};

struct platform_device snd_alsa_device = {
	.name = "ingenic-alsa",
	.dev = {
		.platform_data = &snd_alsa_platform_data,
	},
};
#endif

#endif
