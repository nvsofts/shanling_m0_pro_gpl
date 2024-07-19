#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/interrupt.h>
#include "board_base.h"
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/jzsnd.h>
#ifdef CONFIG_EEPROM_AT24
#include <asm-generic/sizes.h>
#include <linux/i2c/at24.h>
#include <mach/platform.h>
#endif

#define SZ_16K	0x00004000

//extern struct platform_device jz_i2c2_device;

/**
add by zxj
**/
#define CONFIG_ES9118_CODEC 1

#ifdef CONFIG_ES9118_CODEC
#define ES9118_RESET	GPIO_PD(5)
#define ES9118_LDO_0	GPIO_PD(4)
#define ES9118_LDO_1	GPIO_PD(4)

struct es9118_data {
    int reset_gpio;
    int i2c_scl_gpio;
    int i2c_sda_gpio;
    int i2c_addr;
    int clk_441k;
    int clk_48k;
    int ldo_0;
    int ldo_1;
};
#endif


#ifdef CONFIG_ES9118_CODEC
struct es9118_data es9118_platform_data = {
	.reset_gpio = ES9118_RESET,
	.ldo_0 = ES9118_LDO_0,		//硬件未接随便使用
	.ldo_1 = ES9118_LDO_1,
	.i2c_addr = 0x90,
};
#endif



/**
end
**/

#ifdef CONFIG_EEPROM_AT24
static struct at24_platform_data at24c16 = {
	.byte_len = SZ_16K / 8,
	.page_size = 16,

};
#endif
#ifdef CONFIG_WM8594_CODEC_V12
static struct snd_codec_data wm8594_codec_pdata = {
	.codec_sys_clk = 1200000,

};
#endif

#ifdef CONFIG_TOUCHSCREEN_FT6X06
#include <linux/input/ft6x06_ts.h>
struct ft6x06_platform_data ft6x06_tsc_pdata = {
	.x_max          = 400,
	.y_max          = 400,
	.va_x_max	= 400,
	.va_y_max	= 400,
	.irqflags = IRQF_TRIGGER_FALLING|IRQF_DISABLED,
	.irq = GPIO_TP_INT,
	.reset = GPIO_TP_RESET,
};
#endif

#if (defined(CONFIG_SOFT_I2C0_GPIO_V12_JZ) || defined(CONFIG_I2C0_V12_JZ))
struct i2c_board_info jz_i2c0_devs[] __initdata = {
/*
 * sensor i2c info for jz sensor drivers
 */
#ifdef CONFIG_CIM_SENSOR_GC2155
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc2155", 0x3c),
		.platform_data = &cim_sensor_pdata,
	},
#endif
#ifdef CONFIG_CIM_SENSOR_GC0308
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc0308", 0x21),
		.platform_data = &cim_sensor_pdata,
	},
#endif
#ifdef CONFIG_CIM_SENSOR_OV7725
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("ov7725", 0x21),
		.platform_data = &cim_sensor_pdata,
	},
#endif
#ifdef CONFIG_SENSORS_BMA2X2
	{
		I2C_BOARD_INFO("bma2x2", 0x18),
		.irq = GPIO_GSENSOR_INTR,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_FT6X06
	{
		I2C_BOARD_INFO("ft6x06_ts", 0x38),
		.platform_data = &ft6x06_tsc_pdata,
	},
#endif
};
int jz_i2c0_devs_size = ARRAY_SIZE(jz_i2c0_devs);

#ifdef CONFIG_SOC_CAMERA
/*
 * sensor i2c info for v4l2 camera drivers
 */
struct i2c_board_info jz_v4l2_camera_devs[] __initdata = {
#ifdef CONFIG_SOC_CAMERA_OV5640
	[FRONT_CAMERA_INDEX] = {
        I2C_BOARD_INFO("ov5640-front", 0x3c),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_OV7740
	[FRONT_CAMERA_INDEX] = {
        I2C_BOARD_INFO("ov7740", 0x21),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_OV7725
	[FRONT_CAMERA_INDEX] = {
        I2C_BOARD_INFO("ov7725", 0x21),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_GC0308
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc0308", 0x21),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_GC2155
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc2155", 0x3c),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_OV9282
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("ov9282", 0x60), //0xc0 or 0xe0
	},
#endif
#ifdef CONFIG_SOC_CAMERA_SP1409
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("sp1409", 0x34), //0xc0 or 0xe0
	},
#endif

#ifdef CONFIG_SOC_CAMERA_OV2640
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("ov2640", 0x30),
	},
#endif
};
int jz_v4l2_devs_size = ARRAY_SIZE(jz_v4l2_camera_devs);
#endif
#endif /* CONFIG_SOC_CAMERA */

#if (defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ))
struct i2c_board_info jz_i2c2_devs[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{
		I2C_BOARD_INFO("at24",0x57),
		.platform_data  = &at24c16,
	},
#endif
#ifdef CONFIG_WM8594_CODEC_V12
	{
		I2C_BOARD_INFO("wm8594", 0x1a),
		.platform_data  = &wm8594_codec_pdata,
	},
#endif
};
#endif

#if (defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ))
struct i2c_board_info jz_i2c1_devs[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{
		I2C_BOARD_INFO("at24",0x57),
		.platform_data  = &at24c16,
	},
#endif
#ifdef CONFIG_WM8594_CODEC_V12
	{
		I2C_BOARD_INFO("wm8594", 0x1a),
		.platform_data  = &wm8594_codec_pdata,
	},
#endif

#if 0
#ifdef CONFIG_ES9118_CODEC
        {
                I2C_BOARD_INFO("es9118", 0x48),
                .platform_data  = &es9118_platform_data,
        },
#endif
#else
	{
		I2C_BOARD_INFO("akm4951", 0x48),	//seven bit 0x48 eight bit 0x90
		.platform_data	= &es9118_platform_data,
	},


#endif

};
#endif


#if     defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ)
int jz_i2c2_devs_size = ARRAY_SIZE(jz_i2c2_devs);
#endif

#if     defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ)
int jz_i2c1_devs_size = ARRAY_SIZE(jz_i2c1_devs);
#endif


#ifdef CONFIG_EEPROM_AT24

struct i2c_client *at24_client;

static int  at24_dev_init(void)
{
	struct i2c_adapter *i2c_adap;


#if     defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ)
	i2c_adap = i2c_get_adapter(2);
	at24_client = i2c_new_device(i2c_adap, jz_i2c2_devs);
#endif
#if     defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ)
	i2c_adap = i2c_get_adapter(1);
	at24_client = i2c_new_device(i2c_adap, jz_i2c1_devs);
#endif
	i2c_put_adapter(i2c_adap);

	return 0;
}


static void  at24_dev_exit(void)
{
	 i2c_unregister_device(at24_client);
}



module_init(at24_dev_init);

module_exit(at24_dev_exit);


MODULE_LICENSE("GPL");
#endif
#ifdef CONFIG_I2C_GPIO
#define DEF_GPIO_I2C(NO)                        \
    static struct i2c_gpio_platform_data i2c##NO##_gpio_data = {    \
        .sda_pin    = GPIO_I2C##NO##_SDA,           \
        .scl_pin    = GPIO_I2C##NO##_SCK,           \
     };                              \
    struct platform_device i2c##NO##_gpio_device = {        \
        .name   = "i2c-gpio",                   \
        .id = NO,                       \
        .dev    = { .platform_data = &i2c##NO##_gpio_data,},    \
    };
#ifdef CONFIG_SOFT_I2C1_GPIO_V12_JZ
DEF_GPIO_I2C(1);
#endif
#ifdef CONFIG_SOFT_I2C0_GPIO_V12_JZ
DEF_GPIO_I2C(0);
#endif
#endif /*CONFIG_I2C_GPIO*/
