#ifndef _AXP210X_H_
#define _AXP210X_H_

#include "linux/types.h"


#define AXP210X_MANUFACTURER "X-POWERS"

/* AXP210X params */
#define AXP2101_VERSION		(0x18)
#define AXP2101_INVALID_REGADDR (0x22)
#define AXP2101_DEBUG	0

//cyadd
#ifdef __ASSEMBLY__
#define _AC(X,Y)        X
#define _AT(T,X)        X
#else
#define __AC(X,Y)       (X##Y)
#define _AC(X,Y)        __AC(X,Y)
#define _AT(T,X)        ((T)(X))
#endif

#define _UL(x)          (_AC(x, UL))
#define _ULL(x)         (_AC(x, ULL)
#define UL(x)           (_UL(x))
#define ULL(x)          (_ULL(x))

#define GENMASK_INPUT_CHECK(h, l) 0

#define __GENMASK(h, l) \
        (((~UL(0)) - (UL(1) << (l)) + 1) & \
         (~UL(0) >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK(h, l) \
        (GENMASK_INPUT_CHECK(h, l) + __GENMASK(h, l))

#define __GENMASK_ULL(h, l) \
        (((~ULL(0)) - (ULL(1) << (l)) + 1) & \
         (~ULL(0) >> (BITS_PER_LONG_LONG - 1 - (h))))
#define GENMASK_ULL(h, l) \
        (GENMASK_INPUT_CHECK(h, l) + __GENMASK_ULL(h, l))
//*****************************



typedef union {
	uint8_t byte;
	struct {
		unsigned sleep: 1;
		unsigned r1: 3;
		unsigned por: 1;
		unsigned reset: 1;
		unsigned r2: 2;
	};
} reg_mode_t;

typedef union {
	uint8_t byte;
	struct {
		unsigned bromupen: 1;
		unsigned r0: 3;
		unsigned update_mark: 1;
		unsigned enwdt: 1;
		unsigned r1: 2;
	};
} reg_config_t;

typedef union {
	uint8_t byte;
	struct {
		unsigned lowsoc: 1;
		unsigned newsoc: 1;
		unsigned ot: 1;
		unsigned wdt: 1;
		unsigned r : 4;
	};
} reg_irq_t;

typedef union {
	uint8_t byte;
	struct {
		unsigned lowsoc: 1;
		unsigned newsoc: 1;
		unsigned ot: 1;
		unsigned wdt: 1;
		unsigned r : 4;
	};
} reg_irqmask_t;

struct axp210x_reg_t {
	uint8_t        brom;
	reg_mode_t     mode;
	reg_config_t   config;
	uint16_t       vbat;
	uint8_t        temp;
	uint8_t        soc;
	uint16_t       t2e;
	uint16_t       t2f;
	uint8_t        lowsocth;
	reg_irq_t      irq;
	reg_irqmask_t  irqmask;
};

enum axp210x_regaddr_index {
	AXP210X_REG_ID = 0,
	AXP210X_REG_BROM,
	AXP210X_REG_MODE,
	AXP210X_REG_CONFIG,
	AXP210X_REG_VBAT,
	AXP210X_REG_COMSTAT0,
	AXP210X_REG_TM,
	AXP210X_REG_SOC,
	AXP210X_REG_T2E,
	AXP210X_REG_T2F,
	AXP210X_REG_LOWSOC,
	AXP210X_REG_IRQ,
	AXP210X_REG_IRQMASK,
	AXP210X_REG_MAX,
	AXP210X_REG_IIN_LIM,
	AXP210X_REG_ICC_CFG,
	AXP210X_COMM_STAT1,
	AXP210X_CHGLED_CFG,
	AXP210X_COMM_STAT0,
	AXP210X_CHIP_ID,
	AXP210X_MODULE_EN,
};


enum axp210x_chip {
	AXP2101 = 0,
};

struct axp210x_model_data {
	uint8_t *model;
	size_t model_size;
};

enum axp210x_irq {
	AXP210x_IRQ_WDT = 0x08,
	AXP210X_IRQ_OT = 0x04,
	AXP210X_IRQ_NEWSOC = 0x02,
	AXP210X_IRQ_LOWSOC = 0x01,
	AXP210X_IRQ_ALL = 0x0F,
};

/*
 * @bat_init:
 * por = 0
 * wait for ocv init = 1
 * normal run  = 2
 * get better gauge init soc
 */
struct axp210x_state {
	int bat_stat;
	int bat_full;
	int bat_read;
	int bat_init;
	int charger_disable;
	int charger_reg;
};

struct axp_config_info {
	u32 pmu_used;
	u32 pmu_id;
	u32 pmu_battery_rdc;
	u32 pmu_battery_cap;
	u32 pmu_batdeten;
	u32 pmu_chg_ic_temp;
	u32 pmu_runtime_chgcur;
	u32 pmu_suspend_chgcur;
	u32 pmu_shutdown_chgcur;
	u32 pmu_init_chgvol;
	u32 pmu_init_chgend_rate;
	u32 pmu_init_chg_enabled;
	u32 pmu_init_bc_en;
	u32 pmu_init_adc_freq;
	u32 pmu_init_adcts_freq;
	u32 pmu_init_chg_pretime;
	u32 pmu_init_chg_csttime;
	u32 pmu_batt_cap_correct;
	u32 pmu_chg_end_on_en;
	u32 ocv_coulumb_100;

	u32 pmu_bat_para1;
	u32 pmu_bat_para2;
	u32 pmu_bat_para3;
	u32 pmu_bat_para4;
	u32 pmu_bat_para5;
	u32 pmu_bat_para6;
	u32 pmu_bat_para7;
	u32 pmu_bat_para8;
	u32 pmu_bat_para9;
	u32 pmu_bat_para10;
	u32 pmu_bat_para11;
	u32 pmu_bat_para12;
	u32 pmu_bat_para13;
	u32 pmu_bat_para14;
	u32 pmu_bat_para15;
	u32 pmu_bat_para16;
	u32 pmu_bat_para17;
	u32 pmu_bat_para18;
	u32 pmu_bat_para19;
	u32 pmu_bat_para20;
	u32 pmu_bat_para21;
	u32 pmu_bat_para22;
	u32 pmu_bat_para23;
	u32 pmu_bat_para24;
	u32 pmu_bat_para25;
	u32 pmu_bat_para26;
	u32 pmu_bat_para27;
	u32 pmu_bat_para28;
	u32 pmu_bat_para29;
	u32 pmu_bat_para30;
	u32 pmu_bat_para31;
	u32 pmu_bat_para32;

	u32 pmu_ac_vol;
	u32 pmu_ac_cur;
	u32 pmu_usbpc_vol;
	u32 pmu_usbpc_cur;
	u32 pmu_usbad_vol;
	u32 pmu_usbad_cur;
	u32 pmu_pwroff_vol;
	u32 pmu_pwron_vol;
	u32 pmu_powkey_off_time;
	u32 pmu_powkey_off_en;
	u32 pmu_powkey_off_delay_time;
	u32 pmu_powkey_off_func;
	u32 pmu_powkey_long_time;
	u32 pmu_powkey_on_time;
	u32 pmu_powkey_wakeup_irq;
	u32 pmu_pwrok_time;
	u32 pmu_pwrnoe_time;
	u32 pmu_reset_shutdown_en;
	u32 pmu_battery_warning_level1;
	u32 pmu_battery_warning_level2;
	u32 pmu_restvol_adjust_time;
	u32 pmu_ocv_cou_adjust_time;
	u32 pmu_chgled_func;
	u32 pmu_chgled_type;
	u32 pmu_vbusen_func;
	u32 pmu_reset;
	u32 pmu_irq_wakeup;
	u32 pmu_hot_shutdown;
	u32 pmu_inshort;
	u32 power_start;
	u32 pmu_as_slave;
	u32 pmu_bat_unused;
	u32 pmu_ocv_en;
	u32 pmu_cou_en;
	u32 pmu_update_min_time;

	u32 pmu_bat_temp_enable;
	u32 pmu_bat_charge_ltf;
	u32 pmu_bat_charge_htf;
	u32 pmu_bat_shutdown_ltf;
	u32 pmu_bat_shutdown_htf;
	u32 pmu_bat_temp_para1;
	u32 pmu_bat_temp_para2;
	u32 pmu_bat_temp_para3;
	u32 pmu_bat_temp_para4;
	u32 pmu_bat_temp_para5;
	u32 pmu_bat_temp_para6;
	u32 pmu_bat_temp_para7;
	u32 pmu_bat_temp_para8;
	u32 pmu_bat_temp_para9;
	u32 pmu_bat_temp_para10;
	u32 pmu_bat_temp_para11;
	u32 pmu_bat_temp_para12;
	u32 pmu_bat_temp_para13;
	u32 pmu_bat_temp_para14;
	u32 pmu_bat_temp_para15;
	u32 pmu_bat_temp_para16;

	u32 wakeup_usb_in;
	u32 wakeup_usb_out;
	u32 wakeup_bat_in;
	u32 wakeup_bat_out;
	u32 wakeup_bat_charging;
	u32 wakeup_bat_charge_over;
	u32 wakeup_low_warning1;
	u32 wakeup_low_warning2;
	u32 wakeup_bat_untemp_work;
	u32 wakeup_bat_ovtemp_work;
	u32 wakeup_untemp_chg;
	u32 wakeup_ovtemp_chg;
};

struct axp210x_device_info {
	char                      *name;
	struct device             *dev;
	uint8_t                    batnum;
	enum axp210x_chip          chip;
	struct axp_config_info     dts_info;
	uint8_t                   *regaddrs;
	struct axp210x_reg_t       regcache;
	struct axp210x_model_data  data;
	struct task_struct        *poll_read;
	int                        version;
	struct gpio_desc          *gpiod;
	struct regmap             *regmap;
	int                        virq;
	struct power_supply       bat;
	struct power_supply       usb;
	struct power_supply       ac;
	struct delayed_work        bat_chk;
	struct axp210x_state       stat;

	int (*read)(uint8_t regaddr, uint8_t *regdata, uint8_t bytenum);
	int (*write)(uint8_t regaddr, uint8_t *regdata, uint8_t bytenum);
};

#define BATRDC          100
#define INTCHGCUR       300000      /* set initial charging current limite */
#define SUSCHGCUR       1000000     /* set suspend charging current limite */
#define RESCHGCUR       INTCHGCUR   /* set resume charging current limite */
#define CLSCHGCUR       SUSCHGCUR   /* set shutdown charging current limite */
#define INTCHGVOL       4200000     /* set initial charing target voltage */
#define INTCHGENDRATE   10          /* set initial charing end current rate */
#define INTCHGENABLED   1           /* set initial charing enabled */
#define INTADCFREQ      25          /* set initial adc frequency */
#define INTADCFREQC     100         /* set initial coulomb adc coufrequency */
#define INTCHGPRETIME   50          /* set initial pre-charging time */
#define INTCHGCSTTIME   480         /* set initial pre-charging time */
#define BATMAXVOL       4200000     /* set battery max design volatge */
#define BATMINVOL       3500000     /* set battery min design volatge */
#define UPDATEMINTIME   30          /* set bat percent update min time */

#define OCVREG0         0x00        /* 2.99V */
#define OCVREG1         0x00        /* 3.13V */
#define OCVREG2         0x00        /* 3.27V */
#define OCVREG3         0x00        /* 3.34V */
#define OCVREG4         0x00        /* 3.41V */
#define OCVREG5         0x00        /* 3.48V */
#define OCVREG6         0x00        /* 3.52V */
#define OCVREG7         0x00        /* 3.55V */
#define OCVREG8         0x04        /* 3.57V */
#define OCVREG9         0x05        /* 3.59V */
#define OCVREGA         0x06        /* 3.61V */
#define OCVREGB         0x07        /* 3.63V */
#define OCVREGC         0x0a        /* 3.64V */
#define OCVREGD         0x0d        /* 3.66V */
#define OCVREGE         0x1a        /* 3.70V */
#define OCVREGF         0x24        /* 3.73V */
#define OCVREG10        0x29        /* 3.77V */
#define OCVREG11        0x2e        /* 3.78V */
#define OCVREG12        0x32        /* 3.80V */
#define OCVREG13        0x35        /* 3.84V */
#define OCVREG14        0x39        /* 3.85V */
#define OCVREG15        0x3d        /* 3.87V */
#define OCVREG16        0x43        /* 3.91V */
#define OCVREG17        0x49        /* 3.94V */
#define OCVREG18        0x4f        /* 3.98V */
#define OCVREG19        0x54        /* 4.01V */
#define OCVREG1A        0x58        /* 4.05V */
#define OCVREG1B        0x5c        /* 4.08V */
#define OCVREG1C        0x5e        /* 4.10V */
#define OCVREG1D        0x60        /* 4.12V */
#define OCVREG1E        0x62        /* 4.14V */
#define OCVREG1F        0x64        /* 4.15V */

#define AXP_OF_PROP_READ(name, def_value)\
do {\
	if (of_property_read_u32(node, #name, &axp_config->name))\
		axp_config->name = def_value;\
} while (0)

struct axp_interrupts {
	char *name;
	irq_handler_t isr;
	int irq;
};

/*****************Debug************************/
#if (AXP2101_DEBUG)
extern u32 debug_level;

/* Message always need to be present even in release version. */
#define XPOWER_DBG_ALWY		1

/* Error message to report an error, it can hardly works. */
#define XPOWER_DBG_ERROR	2

/* Warning message to inform us of something unnormal or
* something very important, but it still work. */
#define XPOWER_DBG_WARN		3

/* Important message we need to know in unstable version. */
#define XPOWER_DBG_INFO		4

/* Normal message just for debug in developing stage. */
#define XPOWER_DBG_DEBUG	5

#define XPOWER_DBG_DEFAULT	XPOWER_DBG_WARN

#define axp210x_alway(...)	\
	printk(KERN_ERR "[AXP210X]" __VA_ARGS__);

#define axp210x_err(...)	\
	do {	\
		if (debug_level >= XPOWER_DBG_ERROR)	\
			printk(KERN_ERR "[AXP210X_ERR]" __VA_ARGS__);	\
	} while (0)

#define axp210x_warn(...)	\
	do {	\
		if (debug_level >= XPOWER_DBG_WARN)	\
			printk(KERN_ERR "[AXP210X_WARN]" __VA_ARGS__);	\
	} while (0)

#define axp210x_info(...)	\
	do {	\
		if (debug_level >= XPOWER_DBG_INFO)	\
			printk(KERN_ERR "[AXP210X_INFO]" __VA_ARGS__);	\
	} while (0)

#define axp210x_debug(...)	\
	do {	\
		if (debug_level >= XPOWER_DBG_DEBUG)	\
			printk(KERN_ERR "[AXP210X_DBG]" __VA_ARGS__);	\
	} while (0)
#else

#define axp210x_alway(...)
#define axp210x_err(...)
#define axp210x_warn(...)
#define axp210x_info(...)
#define axp210x_debug(...)

#endif

#define axp2101_RSB_RTSADDR     (0x2d)

#define axp2101_COMM_STAT0      (0x00)
#define axp2101_COMM_STAT1      (0x01)
#define axp2101_CHIP_ID         (0x03)
#define axp2101_DATA_BUFFER0    (0x04)
#define axp2101_DATA_BUFFER1    (0x05)
#define axp2101_DATA_BUFFER2    (0x06)
#define axp2101_DATA_BUFFER3    (0x07)
#define axp2101_COMM_FAULT      (0x08)
#define axp2101_COMM_CFG        (0X10)
#define axp2101_BATFET_CTRL     (0X12)
#define axp2101_DIE_TEMP_CFG    (0X13)
#define axp2101_VSYS_MIN        (0x14)
#define axp2101_VINDPM_CFG      (0x15)
#define axp2101_IIN_LIM         (0x16)
#define axp2101_RESET_CFG       (0x17)
#define axp2101_MODULE_EN       (0x18)
#define axp2101_WATCHDOG_CFG    (0x19)
#define axp2101_GAUGE_THLD      (0x1A)
#define axp2101_GPIO12_CTRL     (0x1B)
#define axp2101_GPIO34_CTRL     (0x1C)
#define axp2101_BUS_MODE_SEL    (0x1D)
#define axp2101_PWRON_STAT      (0x20)
#define axp2101_PWROFF_STAT     (0x21)
#define axp2101_PWROFF_EN       (0x22)
#define axp2101_DCDC_PWROFF_EN  (0x23)
#define axp2101_VOFF_THLD       (0x24)
#define axp2101_PWR_TIME_CTRL   (0x25)
#define axp2101_SLEEP_CFG       (0x26)
#define axp2101_PONLEVEL        (0x27)
#define axp2101_FAST_PWRON_CFG0 (0x28)
#define axp2101_FAST_PWRON_CFG1 (0x29)
#define axp2101_FAST_PWRON_CFG2 (0x2A)
#define axp2101_FAST_PWRON_CFG3 (0x2B)
#define axp2101_ADC_CH_EN0      (0x30)
#define axp2101_ADC_CH_EN1      (0x31)
#define axp2101_ADC_CH_EN2      (0x32)
#define axp2101_ADC_CH_EN3      (0x33)
#define axp2101_VBAT_H          (0x34)
#define axp2101_VBAT_L          (0x35)
#define axp2101_TS_H            (0x36)
#define axp2101_TS_L            (0x37)
#define axp2101_VBUS_H          (0x38)
#define axp2101_VBUS_L          (0x39)
#define axp2101_VSYS_H          (0x3A)
#define axp2101_VSYS_L          (0x3B)
#define axp2101_TDIE_H          (0x3C)
#define axp2101_TDIE_L          (0x3D)
#define axp2101_GPADC_H         (0x3E)
#define axp2101_GPADC_L         (0x3F)
#define axp2101_INTEN1          (0x40)
#define axp2101_INTEN2          (0x41)
#define axp2101_INTEN3          (0x42)
#define axp2101_INTSTS1         (0x48)
#define axp2101_INTSTS2         (0x49)
#define axp2101_INTSTS3         (0x4A)
#define axp2101_TS_CFG          (0x50)

#define axp2101_TS_HYSHL2H      (0x52)
#define axp2101_TS_HYSH21       (0x53)
#define axp2101_VLTF_CHG        (0x54)
#define axp2101_VHTF_CHG        (0x55)
#define axp2101_VLTF_WORK       (0x56)
#define axp2101_VHTF_WORK       (0x57)
#define axp2101_JEITA_CFG       (0x58)
#define axp2101_JEITA_CV_CFG    (0x59)
#define axp2101_JEITA_COOL      (0x5A)
#define axp2101_JEITA_WARM      (0x5B)
#define axp2101_TS_CFG_DATA_H   (0x5C)
#define axp2101_TS_CFG_DATA_L   (0x5D)
#define axp2101_CHG_CFG         (0x60)
#define axp2101_IPRECHG_CFG     (0x61)
#define axp2101_ICC_CFG         (0x62)
#define axp2101_ITERM_CFG       (0x63)
#define axp2101_CHG_V_CFG       (0x64)
#define axp2101_TREGU_THLD      (0x65)
#define axp2101_CHG_FREQ        (0x66)
#define axp2101_CHG_TMR_CFG     (0x67)
#define axp2101_BAT_DET         (0x68)
#define axp2101_CHGLED_CFG      (0x69)
#define axp2101_BTN_CHG_CFG     (0x6A)
#define axp2101_SW_TEST_CFG     (0x7B)
#define axp2101_DCDC_CFG0       (0x80)
#define axp2101_DCDC_CFG1       (0x81)
#define axp2101_DCDC1_CFG       (0x82)
#define axp2101_DCDC2_CFG       (0x83)
#define axp2101_DCDC3_CFG       (0x84)
#define axp2101_DCDC4_CFG       (0x85)
#define axp2101_DCDC5_CFG       (0x86)
#define axp2101_DCDC_OC_CFG     (0x87)
#define axp2101_LDO_EN_CFG0     (0x90)
#define axp2101_LDO_EN_CFG1     (0x91)
#define axp2101_ALDO1_CFG       (0x92)
#define axp2101_ALDO2_CFG       (0x93)
#define axp2101_ALDO3_CFG       (0x94)
#define axp2101_ALDO4_CFG       (0x95)
#define axp2101_BLDO1_CFG       (0x96)
#define axp2101_BLDO2_CFG       (0x97)
#define axp2101_CPUSLD_CFG      (0x98)
#define axp2101_DLDO1_CFG       (0x99)
#define axp2101_DLDO2_CFG       (0x9A)
#define axp2101_IP_VER          (0xA0)
#define axp2101_BROM            (0xA1)
#define axp2101_CONFIG          (0xA2)
#define axp2101_TEMPERATURE     (0xA3)
#define axp2101_SOC             (0xA4)
#define axp2101_TIME2EMPTY_H    (0xA6)
#define axp2101_TIME2EMPTY_L    (0xA7)
#define axp2101_TIME2FULL_H     (0xA8)
#define axp2101_TIME2FULL_L     (0xA9)
#define axp2101_FW_VERSION      (0xAB)
#define axp2101_INT0_FLAG       (0xAC)
#define axp2101_COUTER_PERIOD   (0xAD)
#define axp2101_BG_TRIM         (0xAE)
#define axp2101_OSC_TRIM        (0xAF)
#define axp2101_FG_ADDR         (0xB0)
#define axp2101_FG_DATA_H       (0xB2)
#define axp2101_FG_DATA_L       (0xB3)
#define axp2101_RAM_MBIST       (0xB4)
#define axp2101_ROM_TEST        (0xB5)
#define axp2101_ROM_TEST_RT0    (0xB6)
#define axp2101_ROM_TEST_RT1    (0xB7)
#define axp2101_ROM_TEST_RT2    (0xB8)
#define axp2101_ROM_TEST_RT3    (0xB9)
#define axp2101_WD_CLR_DIS      (0xBA)

#define axp2101_BUFFERC         (0xff)

/* bit definitions for AXP events ,irq event */
#define IRQ_NUM(offset, b)      (offset * 8 + b)

#define axp2101_IRQ_SOCWL2      IRQ_NUM(0, 7)
#define axp2101_IRQ_SOCWL1      IRQ_NUM(0, 6)
#define axp2101_IRQ_GWDT        IRQ_NUM(0, 5)
#define axp2101_IRQ_NEWSOC      IRQ_NUM(0, 4)
#define axp2101_IRQ_BCOT        IRQ_NUM(0, 3)
#define axp2101_IRQ_BCUT        IRQ_NUM(0, 2)
#define axp2101_IRQ_BWOT        IRQ_NUM(0, 1)
#define axp2101_IRQ_BWUT        IRQ_NUM(0, 0)
#define axp2101_IRQ_VINSET      IRQ_NUM(1, 7)
#define axp2101_IRQ_VREMOV      IRQ_NUM(1, 6)
#define axp2101_IRQ_BINSERT     IRQ_NUM(1, 5)
#define axp2101_IRQ_BREMOV      IRQ_NUM(1, 4)
#define axp2101_IRQ_PONS        IRQ_NUM(1, 3)
#define axp2101_IRQ_PONL        IRQ_NUM(1, 2)
#define axp2101_IRQ_PONN        IRQ_NUM(1, 1)
#define axp2101_IRQ_PONP        IRQ_NUM(1, 0)
#define axp2101_IRQ_WDEXP       IRQ_NUM(2, 7)
#define axp2101_IRQ_LDOOC       IRQ_NUM(2, 6)
#define axp2101_IRQ_BOCP        IRQ_NUM(2, 5)
#define axp2101_IRQ_CHGDN       IRQ_NUM(2, 4)
#define axp2101_IRQ_CHGST       IRQ_NUM(2, 3)
#define axp2101_IRQ_DOTL1       IRQ_NUM(2, 2)
#define axp2101_IRQ_CHGTE       IRQ_NUM(2, 1)
#define axp2101_IRQ_BOVP        IRQ_NUM(2, 0)

#endif
