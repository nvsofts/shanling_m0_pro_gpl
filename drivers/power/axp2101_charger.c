#define pr_fmt(x) KBUILD_MODNAME ": " x "\n"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/of.h>
//#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/string.h>
#include <asm/irq.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
//#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <linux/err.h>
//#include "../drivers/gpio/gpiolib.h"
#include <linux/mfd/axp2101.h>
#include "axp2101_charger.h"
int axp_usb_connect = 1;
/* #define DONOT_Correction */
/* #define POLL_READ */
#define SOC_RISE_INTERVAL (30)
#define POLL_INTERVAL     (1 * HZ)

#define AXP210X_MASK_WDT    (0x1 << 3)
#define AXP210X_MASK_OT     (0x1 << 2)
#define AXP210X_MASK_NEWSOC (0x1 << 1)
#define AXP210X_MASK_LOWSOC (0x1 << 0)

#define AXP210X_MODE_RSTGAUGE (0x1 << 3)
#define AXP210X_MODE_RSTMCU   (0x1 << 2)
#define AXP210X_MODE_POR      (0x1 << 4)
#define AXP210X_MODE_SLEEP    (0x1 << 0)

#define AXP210X_CFG_ENWDT       (0x1 << 5)
#define AXP210X_CFG_UPDATE_MARK (0x1 << 4)
#define AXP210X_CFG_BROMUP      (0x1 << 0)

#define AXP210X_MASK_VBUS_STATE (BIT(5))

#define AXP210X_VBAT_MAX        (8000)
#define AXP210X_VBAT_MIN        (2000)
#define AXP210X_SOC_MAX         (100)
#define AXP210X_SOC_MIN         (0)

#define AXP210X_CHARGING_TRI  (0)
#define AXP210X_CHARGING_PRE  (1)
#define AXP210X_CHARGING_CC   (2)
#define AXP210X_CHARGING_CV   (3)
#define AXP210X_CHARGING_DONE (4)
#define AXP210X_CHARGING_NCHG (5)
#define AXP210X_MAX_PARAM     (512)

int axp210x_init_chip(struct axp210x_device_info *di);
/* when charge plugged , charger_plugged = 1,  remove is 0 */
int charger_plugged = 1;
EXPORT_SYMBOL_GPL(charger_plugged);

/*
struct class *my_class;
struct cdev cdev;
dev_t devno;
*/

#if AXP2101_DEBUG
u32 debug_level = XPOWER_DBG_DEFAULT;
#endif

struct axp210x_device_info *axp210x_info;

static enum power_supply_property axp2101_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static enum power_supply_property axp2101_usb_ac_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};
/* not test yelt
static enum power_supply_property axp2602_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	...
};
*/
static unsigned char axp2101_model[] = {

	0x01, 0xF5, 0x00, 0x00, 0xFB, 0x00, 0x00, 0xFB, 0x00, 0x1E, 0x32, 0x01,
	0x14, 0x04, 0xD8, 0x04, 0x74, 0xFD, 0x58, 0x0B, 0xB3, 0x10, 0x3F, 0xFB,
	0xC8, 0x00, 0xBE, 0x03, 0x4E, 0x06, 0x3F, 0x06, 0x02, 0x0A, 0xD3, 0x0F,
	0x74, 0x0F, 0x31, 0x09, 0xE5, 0x0E, 0xB9, 0x0E, 0xC0, 0x04, 0xBE, 0x04,
	0xBB, 0x09, 0xB4, 0x0E, 0xA0, 0x0E, 0x92, 0x09, 0x79, 0x0E, 0x4C, 0x0E,
	0x27, 0x03, 0xFC, 0x03, 0xD5, 0x08, 0xBC, 0x0D, 0x9C, 0x0D, 0x55, 0x06,
	0xB8, 0x2E, 0x24, 0x2E, 0x2E, 0x24, 0x2E, 0x24, 0xC5, 0x98, 0x7E, 0x66,
	0x4E, 0x44, 0x38, 0x1A, 0x12, 0x0A, 0xF6, 0x00, 0x00, 0xF6, 0x00, 0xF6,
	0x00, 0xFB, 0x00, 0x00, 0xFB, 0x00, 0x00, 0xFB, 0x00, 0x00, 0xF6, 0x00,
	0x00, 0xF6, 0x00, 0xF6, 0x00, 0xFB, 0x00, 0x00, 0xFB, 0x00, 0x00, 0xFB,
	0x00, 0x00, 0xF6, 0x00, 0x00, 0xF6, 0x00, 0xF6,
};

static struct axp210x_model_data axp2101_model_data = {
	.model = axp2101_model,
	.model_size = ARRAY_SIZE(axp2101_model),
};

static uint8_t axp2101_regaddrs[] = {
	[AXP210X_COMM_STAT1]      = 0x01,
	[AXP210X_REG_BROM]        = 0xA1,
	[AXP210X_REG_MODE]        = 0x17,
	[AXP210X_REG_COMSTAT0]    = 0x00,
	[AXP210X_REG_CONFIG]      = 0xA2,
	[AXP210X_REG_VBAT]        = 0x34,
	[AXP210X_REG_TM]          = 0x3C,
	[AXP210X_REG_SOC]         = 0xA4,
	[AXP210X_REG_T2E]         = 0xA6,
	[AXP210X_REG_T2F]         = 0xA8,
	[AXP210X_REG_LOWSOC]      = 0x1a,
	[AXP210X_REG_IIN_LIM]     = 0x16,
	[AXP210X_REG_ICC_CFG]     = 0x62,
	[AXP210X_CHGLED_CFG]      = 0x69,
	[AXP210X_COMM_STAT0]      = 0x00,
	[AXP210X_CHIP_ID]         = 0x03,
	[AXP210X_MODULE_EN]       = 0x18,
	/* [AXP210X_REG_IRQ] = 0x20, */
	/* [AXP210X_REG_IRQMASK] = 0x21, */
};

static int axp210x_read_vbat(union power_supply_propval *val)
{
	uint8_t data[2];
	uint16_t vtemp[3], tempv;
	int ret = 0;
	uint8_t i;

	for (i = 0; i < 3; i++) {
		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_REG_VBAT], data, 2);
		if (ret < 0)
			return ret;

		vtemp[i] = (((data[0] & GENMASK(5, 0)) << 0x08) | (data[1]));
	}
	if (vtemp[0] > vtemp[1]) {
		tempv = vtemp[0];
		vtemp[0] = vtemp[1];
		vtemp[1] = tempv;
	}
	if (vtemp[1] > vtemp[2]) {
		tempv = vtemp[1];
		vtemp[1] = vtemp[2];
		vtemp[2] = tempv;
	}
	if (vtemp[0] > vtemp[1]) {
		tempv = vtemp[0];
		vtemp[0] = vtemp[1];
		vtemp[1] = tempv;
	}
	/*incase vtemp[1] exceed AXP210X_VBAT_MAX */
	if ((vtemp[1] > AXP210X_VBAT_MAX) || (vtemp[1] < AXP210X_VBAT_MIN)) {
		val->intval = axp210x_info->regcache.vbat;
		return 0;
	}
	axp210x_info->regcache.vbat = vtemp[1];
	val->intval = vtemp[1];
	return 0;
}

/* read temperature */
static int axp210x_read_temp(union power_supply_propval *val)
{
	uint8_t data[2];
	int ret = 0;
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_TM], data,
				 2);
	if (ret < 0)
		return ret;
	axp210x_info->regcache.temp = (data[0] << 8) + data[1];
	val->intval = axp210x_info->regcache.temp;
	return 0;
}

static int axp210x_param_in_ram(void)
{
	uint8_t data[2];

	axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (data[0] & AXP210X_CFG_UPDATE_MARK)
		return true;
	else
		return false;
}

static int axp210x_read_soc(union power_supply_propval *val)
{
	uint8_t data[2];
#ifdef DONOT_Correction
	static long int lasttime;
	struct timeval sysday;
#endif

	int ret = 0;

	if (!axp210x_param_in_ram()) {
		pr_warn("something reset the gauge\n");
		axp210x_init_chip(axp210x_info);
		/* 500ms can read the soc */
		msleep(500);
	}

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_SOC], data,
				 1);
	if (ret < 0)
		return ret;
	if (data[0] > AXP210X_SOC_MAX)
		data[0] = AXP210X_SOC_MAX;
	else if (data[0] < AXP210X_SOC_MIN)
		data[0] = AXP210X_SOC_MIN;
#ifdef DONOT_Correction
	if (charger_plugged) {
		do_gettimeofday(&sysday);
		printk("systime = %ld, lastime = %ld \r\n", sysday.tv_sec,
		       lasttime);
		if (lasttime == 0) {
			lasttime = sysday.tv_sec;
		}
		if (data[0] > axp210x_info->regcache.soc) {
			if (axp210x_info->regcache.soc < 92) {
				if (sysday.tv_sec - lasttime >
				    (SOC_RISE_INTERVAL)) {
					axp210x_info->regcache.soc++;
					val->intval =
						axp210x_info->regcache.soc;
					lasttime = sysday.tv_sec;
					//					printk("no
					//corrention socreal = %d, socnow = %d
					//\r\n",data[0],
					//axp210x_info->regcache.soc);
				}
			} else if (axp210x_info->regcache.soc >= 92) {
				if (sysday.tv_sec - lasttime >
				    (SOC_RISE_INTERVAL *
				     (axp210x_info->regcache.soc - 92))) {
					axp210x_info->regcache.soc++;
					val->intval =
						axp210x_info->regcache.soc;
					lasttime = sysday.tv_sec;
					//					printk("no
					//corrention socreg = %d, socnow = %d
					//\r\n",data[0],
					//axp210x_info->regcache.soc);
				}
			}
		} else if (data[0] < axp210x_info->regcache.soc) {
			axp210x_info->regcache.soc--;
			val->intval = axp210x_info->regcache.soc;
		}

	} else {
		if (data[0] < axp210x_info->regcache.soc) {
			axp210x_info->regcache.soc--;
			val->intval = axp210x_info->regcache.soc;
			lasttime = 0;
		}
	}
#else
	axp210x_info->regcache.soc = data[0];
	val->intval = data[0];
#endif
	return 0;
}

static int axp210x_read_time2empty(union power_supply_propval *val)
{
	uint8_t data[2];
	uint16_t ttemp[3], tempt;
	int ret = 0;
	uint8_t i;

	for (i = 0; i < 3; i++) {
		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_REG_T2E], data, 2);
		if (ret < 0)
			return ret;
		ttemp[i] = ((data[0] << 0x08) | (data[1]));
	}
	if (ttemp[0] > ttemp[1]) {
		tempt = ttemp[0];
		ttemp[0] = ttemp[1];
		ttemp[1] = tempt;
	}
	if (ttemp[1] > ttemp[2]) {
		tempt = ttemp[1];
		ttemp[1] = ttemp[2];
		ttemp[2] = tempt;
	}
	if (ttemp[0] > ttemp[1]) {
		tempt = ttemp[0];
		ttemp[0] = ttemp[1];
		ttemp[1] = tempt;
	}
	axp210x_info->regcache.t2e = ttemp[1];
	val->intval = ttemp[1];
	return 0;
}

static int axp210x_read_vbus_state(union power_supply_propval *val)
{
	int ret = 0;
	uint8_t data;

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_COMSTAT0],
				 &data, 1);
	if (ret < 0)
		return ret;
	/* vbus is good when vbus state set */
	val->intval = !!(data & AXP210X_MASK_VBUS_STATE);

	return ret;
}

static int axp210x_read_time2full(union power_supply_propval *val)
{
	uint8_t data[2];
	uint16_t ttemp[3], tempt;
	int ret = 0;
	uint8_t i;

	for (i = 0; i < 3; i++) {
		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_REG_T2F], data, 2);
		if (ret < 0)
			return ret;
		ttemp[i] = ((data[0] << 0x08) | (data[1]));
	}
	if (ttemp[0] > ttemp[1]) {
		tempt = ttemp[0];
		ttemp[0] = ttemp[1];
		ttemp[1] = tempt;
	}
	if (ttemp[1] > ttemp[2]) {
		tempt = ttemp[1];
		ttemp[1] = ttemp[2];
		ttemp[2] = tempt;
	}
	if (ttemp[0] > ttemp[1]) {
		tempt = ttemp[0];
		ttemp[0] = ttemp[1];
		ttemp[1] = tempt;
	}
	axp210x_info->regcache.t2f = ttemp[1];
	val->intval = ttemp[1];
	return 0;
}

static int axp210x_read_lowsocth(union power_supply_propval *val)
{
	uint8_t data[2];
	int ret = 0;
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_LOWSOC],
				 data, 1);
	if (ret < 0)
		return ret;
	axp210x_info->regcache.lowsocth = data[0] >> 4;
	val->intval = data[0];
	return 0;
}

static int axp210x_set_lowsocth(uint8_t val)
{
	int ret = 0;
	uint8_t data[2];

	data[0] = val;

	if (data[0] > 20)
		return -EINVAL;
	axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_LOWSOC], data, 1);
	if (ret < 0)
		return ret;

	data[0] &= ~GENMASK(7, 4);
	data[0] |= val << 4;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_LOWSOC],
				  data, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int axp210x_reset_gauge(void)
{
	int ret = 0;
	uint8_t data[2];

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_MODE], data,
				 1);
	if (ret < 0)
		return ret;

	data[0] |= AXP210X_MODE_RSTMCU;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_MODE],
				  data, 1);
	if (ret < 0)
		return ret;

	data[0] &= ~AXP210X_MODE_RSTMCU;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_MODE],
				  data, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int axp210x_reset_mcu(void)
{
	int ret = 0;
	uint8_t data[2];

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_MODE], data,
				 1);
	if (ret < 0)
		return ret;
	data[0] |= AXP210X_MODE_RSTMCU;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_MODE],
				  data, 1);
	if (ret < 0)
		return ret;
	data[0] &= ~AXP210X_MODE_RSTMCU;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_MODE],
				  data, 1);
	if (ret < 0)
		return ret;

	return 0;
}

int axp210x_model_update(void)
{
	int ret = 0;
	uint8_t data[2];
	uint8_t para[axp210x_info->data.model_size];
	uint8_t i;

	/*
	 * when write parameters, first close charger
	 */
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_MODULE_EN], data,
				 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] &= ~BIT(1);
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_MODULE_EN],
				  data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	msleep(1000);

	/* reset_mcu */
	ret = axp210x_reset_mcu();
	if (ret < 0)
		goto UPDATE_ERR;

	/* reset and open brom */
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				 data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] &= ~AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				  data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] |= AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				  data, 1);
	if (ret < 0)
		goto UPDATE_ERR;

	/* down load battery parameters */
	for (i = 0; i < axp210x_info->data.model_size; i++) {
		ret = axp210x_info->write(
			axp210x_info->regaddrs[AXP210X_REG_BROM],
			&axp210x_info->data.model[i], 1);
	}
	if (ret < 0)
		goto UPDATE_ERR;

	/* reset and open brom */
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				 data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] &= ~AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				  data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] |= AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				  data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	/* check battery parameters is ok ? */
	for (i = 0; i < axp210x_info->data.model_size; i++) {
		axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_BROM],
				   &para[i], 1);
		if (para[i] != axp210x_info->data.model[i]) {
			axp210x_warn(
				"model [%d] para reading %02x != write %02x\n",
				i, para[i], axp210x_info->data.model[i]);
			ret = -EINVAL;
			//	goto UPDATE_ERR;
		}
	}
	if (ret < 0)
		goto UPDATE_ERR;

	/* close brom and set battery update flag */
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				 data, 1);
	if (ret < 0)
		goto UPDATE_ERR;
	data[0] &= ~AXP210X_CFG_BROMUP;
	data[0] |= AXP210X_CFG_UPDATE_MARK;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				  data, 1);
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				 data, 1);
	if (ret < 0)
		goto UPDATE_ERR;

	/* reset_mcu */
	ret = axp210x_reset_mcu();
	if (ret < 0)
		goto UPDATE_ERR;

	return 0;

UPDATE_ERR:
	axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	data[0] &= ~AXP210X_CFG_BROMUP;
	axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	axp210x_reset_mcu();
	return ret;
}


static bool axp210x_model_update_check(void)
{
	int ret = 0;
	uint8_t data[2];
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;
	if ((data[0] & AXP210X_CFG_UPDATE_MARK) == 0)
		goto CHECK_ERR;

#if 0
	/* if need check every bytes of battery parameters , due to battery parameters changed */
	/* reset and open brom */
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;
	data[0] &= ~AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;
	data[0] |= AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;
	/* check battery parameters is ok ? */
	for (i = 0; i < axp210x_info->data.model_size; i++) {
		axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_BROM], &para[i], 1);
		//  if (ret < 0)
		//	break;
		if (para[i] != axp210x_info->data.model[i]) {
			axp210x_warn("model [%d] para reading %02x != write %02x\n", i, para[i], axp210x_info->data.model[i]);
			ret = -EINVAL;
			//	break;
		}
	}
	if (ret < 0) {
		ret = axp210x_reset_mcu();
		if (ret < 0)
			goto CHECK_ERR;
		ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
		if (ret < 0)
			goto CHECK_ERR;
		data[0] &= ~AXP210X_CFG_BROMUP;
		ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
		if (ret < 0)
			goto CHECK_ERR;
		data[0] |= AXP210X_CFG_BROMUP;
		ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
		if (ret < 0)
			goto CHECK_ERR;
		/* check battery parameters is ok ? */
		for (i = 0; i < axp210x_info->data.model_size; i++) {
			axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_BROM], &para[i], 1);
			//if (ret < 0)
			//	goto CHECK_ERR;
			if (para[i] != axp210x_info->data.model[i]) {
				axp210x_warn("model [%d] para reading %02x != write %02x\n", i, para[i], axp210x_info->data.model[i]);
				ret = -EINVAL;
				//	goto CHECK_ERR;
			}
		}
		if (ret < 0)
			goto CHECK_ERR;
	}
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;

	data[0] &= ~AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	if (ret < 0)
		goto CHECK_ERR;

#endif

	return true;

CHECK_ERR:

	axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	data[0] &= ~AXP210X_CFG_BROMUP;
	ret = axp210x_info->write(axp210x_info->regaddrs[AXP210X_REG_CONFIG], data, 1);
	axp210x_reset_mcu();
	return false;
}

static int axp210x_reg_update(void)
{
	int ret = 0;
	uint8_t data[2];

	data[0] = 0x10;
	ret = axp210x_info->write(0x50, data, 1);
	if (ret < 0)
		return ret;

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_CONFIG],
				 data, 1); // 0x03,
	if (ret < 0)
		return ret;
	axp210x_info->regcache.config.byte = data[0];

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_VBAT], data,
				 2);
	if (ret < 0)
		return ret;
	axp210x_info->regcache.vbat =
		((data[0] & GENMASK(5, 0)) << 8) + data[1];

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_TM], data,
				 2);
	axp210x_info->regcache.temp = (data[0] << 8) + data[1];
	if (ret < 0)
		return ret;

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_SOC], data, 1);
	axp210x_info->regcache.soc = data[0];
	if (ret < 0)
		return ret;
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_T2E], data, 2);
	if (ret < 0)
		return ret;
	axp210x_info->regcache.t2e = (data[0] << 8) + data[1];
	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_T2F], data, 2);
	if (ret < 0)
		return ret;
	axp210x_info->regcache.t2f = (data[0] << 8) + data[1];

	ret = axp210x_info->read(axp210x_info->regaddrs[AXP210X_REG_LOWSOC], data, 1);
	axp210x_info->regcache.lowsocth = data[0] >> 4;
	if (ret < 0)
		return ret;

	/* set led not bright power on first */
	regmap_update_bits(axp210x_info->regmap,
			   axp2101_regaddrs[AXP210X_CHGLED_CFG], BIT(0), 0);
	return 0;
}

static int axp210x_usb_ac_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		ret = axp210x_read_vbus_state(val);
		break;
	default:
		break;
	}

	return ret;
}

static int axp210x_get_bat_status(struct power_supply *psy,
				  union power_supply_propval *val)
{
	uint8_t data[2];
	int ret;

	struct axp210x_device_info *di = axp210x_info;

	/* some bug cause can't get battery out */
	if (!di->stat.bat_stat) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	ret = axp210x_info->read(di->regaddrs[AXP210X_COMM_STAT1],
				 data, 1);
	if (ret) {
		printk("error read AXP210X_COM_STAT1\n");
		return ret;
	}
	/* chg_stat = bit[2:0] */
	switch (data[0] & 0x07) {
	case AXP210X_CHARGING_NCHG:
		if (((data[0] & 0x60) >> 5) != 0x1) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case AXP210X_CHARGING_TRI:


		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		/* BIT[6:5] bat_curr_dir */
		/* if (((data[0] >> 5) & 0x03) == 2) { */
			/* val->intval = POWER_SUPPLY_STATUS_DISCHARGING; */
		/* } */
		break;
	case AXP210X_CHARGING_PRE:
	case AXP210X_CHARGING_CC:
	case AXP210X_CHARGING_CV:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case AXP210X_CHARGING_DONE:
		if (di->stat.bat_full)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int axp210x_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	struct axp210x_device_info *di = axp210x_info;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL: // customer modify
		if (axp210x_info->regcache.soc == 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (axp210x_info->regcache.soc > 80)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else if (axp210x_info->regcache.soc > axp210x_info->regcache.lowsocth)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (axp210x_info->regcache.soc < axp210x_info->regcache.lowsocth)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (axp210x_info->regcache.soc <= 1)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = axp210x_get_bat_status(psy, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/*
		 * ret = axp210x_read_vbat(val);
		 * val->intval = axp210x_info->regcache.vbat > 0 ? 1 : 0;
		 */

		val->intval = di->stat.bat_stat;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = axp210x_read_vbat(val);
		val->intval = val->intval * 1000; 											//unit uV;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = di->dts_info.pmu_battery_cap;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (di->stat.bat_stat&&
		    di->stat.bat_read)
			ret = axp210x_read_soc(val); // unit %;
		else
			val->intval = -1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = axp210x_read_lowsocth(val);							//unit %;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = axp210x_read_temp(val);								//unit degree celsius
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:							//unit degree celsius
		val->intval = 85;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = axp210x_read_time2empty(val);
		val->intval = val->intval * 60;												//unit second
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = axp210x_read_time2full(val);
		val->intval = val->intval * 60;												//unit second
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = AXP210X_MANUFACTURER;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int axp210x_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	int ret = 0;

	if (psp != POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN)
		ret = -EINVAL;
	else
		ret = axp210x_set_lowsocth((uint8_t) val->intval);

	pm_runtime_put_sync(axp210x_info->dev);
	return ret;
}

static int axp210x_writeable(struct power_supply *psy,
			     enum power_supply_property psp)
{
	int ret = 0;

	if (psp != POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN)
		ret = -EINVAL;
	else
		ret = 0;

	return ret;
}

static int axp210x_register_battery(struct axp210x_device_info *di)
{
#if 1
	int ret = 0;
	struct power_supply *batt = &di->bat;
	struct power_supply *usb = &di->usb;
	struct power_supply *ac = &di->ac;
	
	batt->name = "battery";
	batt->use_for_apm = 1;
	batt->type = POWER_SUPPLY_TYPE_BATTERY;
	batt->get_property = axp210x_get_property;
	batt->set_property = axp210x_set_property;
	batt->properties  = axp2101_props;
	batt->num_properties = ARRAY_SIZE(axp2101_props);
	batt->property_is_writeable = axp210x_writeable;
	ret = power_supply_register(di->dev, batt);
	if (ret)
	{
		return -ENOMEM;
	}

	usb->name = "usb";
	usb->type = POWER_SUPPLY_TYPE_USB;
	usb->get_property = axp210x_usb_ac_get_property;
	usb->set_property = NULL;
	usb->properties = axp2101_usb_ac_props;
	usb->num_properties = ARRAY_SIZE(axp2101_usb_ac_props);
	usb->property_is_writeable = NULL;
	ret = power_supply_register(di->dev, usb);
	if (ret)
	{
		ret = -ENOMEM;
		goto err1;
	}

	ac->name = "ac";
	ac->type = POWER_SUPPLY_TYPE_MAINS;
	ac->get_property = axp210x_usb_ac_get_property;
	usb->set_property = NULL;
	ac->properties = axp2101_usb_ac_props;
	ac->num_properties = ARRAY_SIZE(axp2101_usb_ac_props);
	usb->property_is_writeable = NULL;
	ret = power_supply_register(di->dev, ac);
	if (ret)
	{
		ret = -ENOMEM;
		goto err2;
	}
	return ret;
err2:
	power_supply_unregister(&di->usb);
err1:
	power_supply_unregister(&di->bat);

	return ret;

#else
	int ret = 0;

	struct power_supply_desc *psy_desc;
	struct power_supply_desc *usb_desc, *ac_desc;
	struct power_supply_config psy_cfg = {
		.of_node = di->dev->of_node,
		.drv_data = di,
	};

	psy_desc = devm_kzalloc(di->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_desc->name = "battery";
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = axp2101_props;
	psy_desc->num_properties = ARRAY_SIZE(axp2101_props);
	psy_desc->get_property = axp210x_get_property;
	psy_desc->set_property = axp210x_set_property;
	psy_desc->property_is_writeable = axp210x_writeable;

	di->bat = power_supply_register(di->dev, psy_desc, &psy_cfg);
	if (IS_ERR(di->bat)) {
		axp210x_err("failed to register battery\n");
		ret = PTR_ERR(di->bat);
		return ret;
	}

	usb_desc = devm_kzalloc(di->dev, sizeof(*usb_desc), GFP_KERNEL);
	if (!usb_desc) {
		ret = -ENOMEM;
		goto err1;
	}

	usb_desc->name = "usb";
	usb_desc->type = POWER_SUPPLY_TYPE_USB;
	usb_desc->properties = axp2101_usb_ac_props;
	usb_desc->num_properties = ARRAY_SIZE(axp2101_usb_ac_props);
	usb_desc->get_property = axp210x_usb_ac_get_property;
	usb_desc->set_property = NULL;
	usb_desc->property_is_writeable = NULL;

	di->usb = power_supply_register(di->dev, usb_desc, &psy_cfg);
	if (IS_ERR(di->usb)) {
		axp210x_err("failed to register usb\n");
		ret = PTR_ERR(di->bat);
		goto err1;
	}

	ac_desc = devm_kzalloc(di->dev, sizeof(*ac_desc), GFP_KERNEL);
	if (!ac_desc) {
		ret = -ENOMEM;
		goto err2;
	}

	ac_desc->name = "ac";
	ac_desc->type = POWER_SUPPLY_TYPE_MAINS;
	ac_desc->properties = axp2101_usb_ac_props;
	ac_desc->num_properties = ARRAY_SIZE(axp2101_usb_ac_props);
	ac_desc->get_property = axp210x_usb_ac_get_property;
	ac_desc->set_property = NULL;
	;
	ac_desc->property_is_writeable = NULL;

	di->ac = power_supply_register(di->dev, ac_desc, &psy_cfg);
	if (IS_ERR(di->ac)) {
		axp210x_err("failed to register battery\n");
		ret = PTR_ERR(di->bat);
		goto err2;
	}

	return ret;

err2:
	power_supply_unregister(di->usb);
err1:
	power_supply_unregister(di->bat);

	return ret;
#endif
}

void axp210x_teardown_battery(struct axp210x_device_info *di)
{
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->usb);
}

int axp210x_init_chip(struct axp210x_device_info *di)
{
	int ret = 0;

	if (di == NULL) {
		axp210x_err("axp210x_info is invalid!\n");
		return -ENODEV;
	}

	ret = axp210x_reg_update();
	if (ret < 0) {
		axp210x_err("axp210x reg update, i2c communication err!\n");
		return ret;
	}

	if (!axp210x_model_update_check()) {
		di->stat.bat_init = 0;
		ret = axp210x_model_update();
		if (ret < 0) {
			axp210x_err("axp210x model update fail!\n");
			return ret;
		}
	} else {
		di->stat.bat_read = true;
	}

	axp210x_debug("axp210x model update ok\n");

	/* after 500ms can read soc */
	ret = axp210x_reg_update();
	if (ret < 0) {
		axp210x_err("axp210x reg update, i2c communication err!\n");
		return ret;
	}

	return ret;
}

#if 0 //cyadd 
static irqreturn_t axp210x_irq_handler_usb_in(int irq, void *data)
{
	unsigned int value = 0;
	struct axp210x_device_info *di = data;

	power_supply_changed(di->bat);
	regmap_read(di->regmap, axp2101_COMM_STAT0, &value);

	if (value & BIT(5)) {
		axp_usb_connect = 1;
	} else {
		axp_usb_connect = 0;
	}

	return IRQ_HANDLED;
}

static irqreturn_t axp210x_irq_handler_usb_out(int irq, void *data)
{
	u32 value = 0;
	struct axp210x_device_info *di = data;

	power_supply_changed(di->bat);
	regmap_read(di->regmap, axp2101_COMM_STAT0, &value);

	if (value & BIT(5)) {
		axp_usb_connect = 1;
	} else {
		axp_usb_connect = 0;
	}

	return IRQ_HANDLED;
}

static irqreturn_t axp210x_irq_handler_thread(int irq, void *data)
{
	int ret = 0;
	struct irq_desc *id = irq_to_desc(irq);
	struct axp210x_device_info *di = data;

	union power_supply_propval val;
	pr_debug("%s: enter interrupt %d\n", __func__, irq);

	power_supply_changed(di->bat);
	switch (id->irq_data.hwirq) {
	case axp2101_IRQ_CHGDN:
		pr_debug("interrupt:charger done");
		break;
	case axp2101_IRQ_CHGST:
		pr_debug("interrutp:charger start");
		break;
	case axp2101_IRQ_BINSERT:
		pr_debug("interrupt:battery insert");
		break;
	case axp2101_IRQ_BREMOV:
		pr_debug("interrupt:battery remove");
		break;
	default:
		pr_debug("interrupt:others");
		break;
	}

	ret = axp210x_read_soc(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: soc update fail!\n", __FUNCTION__);
	ret = axp210x_read_vbat(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: vbat update fail!\n", __FUNCTION__);

	ret = axp210x_read_temp(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: temprature update fail!\n", __FUNCTION__);

	ret = axp210x_read_time2empty(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: time2empty update fail!\n", __FUNCTION__);

	ret = axp210x_read_time2full(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: time2full update fail!\n", __FUNCTION__);

	/*
		if ((axp210x_info->regcache.irq.wdt) &&
	   (axp210x_info->regcache.irqmask.wdt == 0)){
			axp210x_init_chip(axp210x_info);
			printk("%s: wdt irq alert!\n",__func__);
			// inform sys
		}
		if ((axp210x_info->regcache.irq.ot) &&
	   (axp210x_info->regcache.irqmask.ot == 0)){
			//inform sys
			printk("%s: ot irq alert!\n",__func__);
		}
		if ((axp210x_info->regcache.irq.newsoc) &&
	   (axp210x_info->regcache.irqmask.newsoc == 0)){
			//inform sys
			printk("%s: newsoc irq alert!\n",__func__);
		}
		if ((axp210x_info->regcache.irq.lowsoc) &&
	   (axp210x_info->regcache.irqmask.lowsoc == 0)){
			//inform sys
			printk("%s: lowsoc irq alert!\n",__func__);
		}
		enable_irq(irq);
		*/
	return IRQ_HANDLED;
}
#endif

static int axp210x_read(uint8_t regaddr, uint8_t *regdata, uint8_t bytenum)
{
	int ret, i;
	struct regmap *regmap = axp210x_info->regmap;

	if (!regmap) {
		return -ENODEV;
	}

	for (i = 0; i < bytenum; i++) {
		unsigned int v;

		ret = regmap_read(regmap, regaddr + i, &v);
		if (ret < 0) {
			pr_debug("regmap_read error");
			return ret;
		}

		*(regdata + i) = v;
	}

	return 0;
}


static int axp210x_write(uint8_t regaddr, uint8_t *regdata, uint8_t bytenum)
{
	int ret, i;
	struct regmap *regmap = axp210x_info->regmap;

	if (!regmap) {
		return -ENODEV;
	}

	for (i = 0; i < bytenum; i++) {
		unsigned int v;

		v = *(regdata + i);
		ret = regmap_write(regmap, regaddr + i, v);
		if (ret < 0)
			break;
	}

	return 0;
}

/*

static int axp210x_open(struct inode *ip, struct file * filp)
{
	return 0;
}

static int axp210x_release(struct inode *ip, struct file * filp)
{
	return 0;
}

static ssize_t axp210x_read(struct file * filp, char __user *buf, size_t size,
loff_t * ppos)
{

	union power_supply_propval val;
	int ret = 0;
//	ret = axp210x_read_vbat(&val);
	printk(KERN_INFO "ret %d, vbat = %d\n", ret, val.intval);
//	ret = axp210x_read_soc(&val);
	printk(KERN_INFO "ret %d, soc = %d\n", ret, val.intval);
//	ret = axp210x_read_time2empty(&val);
	printk(KERN_INFO "ret %d, t2e = %d\n", ret, val.intval);
//	ret = axp210x_read_time2full(&val);
	printk(KERN_INFO "ret %d, t2f = %d\n", ret, val.intval);
//	ret = axp210x_read_irq(&val);
	printk(KERN_INFO "ret %d, irq = %d\n", ret, val.intval);
	return ret;
}



struct file_operations axp210x_file_ops = {
	.owner = THIS_MODULE,
	.open = axp210x_open,
	.release = axp210x_release,
	.read = axp210x_read,
};
*/

#if ((defined DONOT_Correction) || (defined POLL_READ))
static void timer_handler(unsigned long arg)
{
	int ret = 0;
	union power_supply_propval val;
	printk("%s: timer_handler work!\n", __FUNCTION__);

	ret = axp210x_read_soc(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: soc update fail!\n", __FUNCTION__);
	ret = axp210x_read_vbat(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: vbat update fail!\n", __FUNCTION__);

	ret = axp210x_read_temp(&val);
	if (ret != 0)
		printk(KERN_ALERT "%s: temprature update fail!\n", __FUNCTION__);

	//	ret = axp210x_read_time2empty(&val);
	//	if (ret != 0)
	//		printk(KERN_ALERT "%s: time2empty update fail!\n",
	//__FUNCTION__);

	//	ret = axp210x_read_time2full(&val);
	//	if (ret != 0)
	//		printk(KERN_ALERT "%s: time2full update fail!\n",
	//__FUNCTION__);

	printk("%s: soc[%d] vbat[%d] \n", __func__, axp210x_info->regcache.soc,
	       axp210x_info->regcache.vbat);
}

static int thread_dosomthing(void *data)
{
	set_freezable();

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(POLL_INTERVAL);
		try_to_freeze();
		timer_handler(0);
	}

	return 0;
}
#endif


#if (AXP2101_DEBUG)
static ssize_t register_read(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int address = 0;
	uint8_t value = 0;
	int ret = 0;
	char *endptr = NULL;

	address = simple_strtoul(buf, &endptr, 16);
	printk(KERN_ERR "address=%x\n", address);

	ret = axp210x_info->read(address, &value, 1);
	if (ret < 0)
		return 0;

	axp210x_alway("value=%x\n", value);
	return count;
}

static ssize_t register_write(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int address = 0;
	uint8_t value = 0;
	int ret = 0;
	char *endptr = NULL;
	const char *start = &buf[0];

	address = simple_strtoul(buf, &endptr, 16);
	start = endptr + 1;
	if (start < buf + count)
		value = simple_strtoul(start, &endptr, 16);
	axp210x_alway("address=%x,value=%x\n", address, value);

	ret = axp210x_info->write(address, (uint8_t *)&value, 1);
	if (ret < 0)
		return 0;

	return count;
}

static struct class_attribute axp210x_user_define_property[] = {
	__ATTR(reg_read, S_IWUSR, NULL, register_read),
	__ATTR(reg_write, S_IWUSR, NULL, register_write),
	__ATTR_NULL,
};

static struct class axp210x_user_define = {
	.name = "axp210x_user_define",
	.class_attrs = axp210x_user_define_property,
};
#endif

#if 0 //cyadd 
enum axp2101_virq_index {
	AXP2101_VIRQ_USB_IN,
	AXP2101_VIRQ_USB_OUT,
	AXP2101_VIRQ_BAT_IN,
	AXP2101_VIRQ_BAT_OUT,
	AXP2101_VIRQ_CHARGING,
	AXP2101_VIRQ_CHARGE_OVER,
	AXP2101_VIRQ_LOW_WARNING1,
	AXP2101_VIRQ_LOW_WARNING2,
	AXP2101_VIRQ_BAT_UNTEMP_WORK,
	AXP2101_VIRQ_BAT_OVTEMP_WORK,
	AXP2101_VIRQ_BAT_UNTEMP_CHG,
	AXP2101_VIRQ_BAT_OVTEMP_CHG,

	AXP2101_VIRQ_MAX_VIRQ,
};

static struct axp_interrupts axp_charger_irq[] = {
	[AXP2101_VIRQ_USB_IN] = { "usb in", axp210x_irq_handler_usb_in },
	[AXP2101_VIRQ_USB_OUT] = { "usb out", axp210x_irq_handler_usb_out },
	[AXP2101_VIRQ_BAT_IN] = { "bat in", axp210x_irq_handler_thread },
	[AXP2101_VIRQ_BAT_OUT] = { "bat out", axp210x_irq_handler_thread },
	[AXP2101_VIRQ_CHARGING] = { "charging", axp210x_irq_handler_thread },
	[AXP2101_VIRQ_CHARGE_OVER] = { "charge over",
				       axp210x_irq_handler_thread },
	[AXP2101_VIRQ_LOW_WARNING1] = { "low warning1",
					axp210x_irq_handler_thread },
	[AXP2101_VIRQ_LOW_WARNING2] = { "low warning2",
					axp210x_irq_handler_thread },
	[AXP2101_VIRQ_BAT_UNTEMP_WORK] = { "bat untemp work",
					   axp210x_irq_handler_thread },
	[AXP2101_VIRQ_BAT_OVTEMP_WORK] = { "bat ovtemp work",
					   axp210x_irq_handler_thread },
	[AXP2101_VIRQ_BAT_UNTEMP_CHG] = { "bat untemp chg",
					  axp210x_irq_handler_thread },
	[AXP2101_VIRQ_BAT_OVTEMP_CHG] = { "bat ovtemp chg",
					  axp210x_irq_handler_thread },
};
#endif	

static void axp_set_charger_info(struct axp210x_device_info *di)
{
	axp210x_info = di;
}

static void axp2101_charger_sysconfig(struct axp210x_device_info *di)
{
	struct regmap *regmap = di->regmap;
	struct axp_config_info *dinfo = &di->dts_info;
	uint8_t value;

	if (dinfo->pmu_chg_ic_temp)
		regmap_update_bits(regmap, axp2101_TS_CFG, 0x0a, 0x0a);
	else
		regmap_update_bits(regmap, axp2101_TS_CFG, 0x0a, 0x00);

	/* set charger voltage limit */
	/* 4600 is change to 2101b */
	if (dinfo->pmu_init_chgvol < 4100) {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x01);
	} else if (dinfo->pmu_init_chgvol < 4200) {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x02);
	} else if (dinfo->pmu_init_chgvol < 4350) {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x03);
	} else if (dinfo->pmu_init_chgvol < 4400) {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x04);
	} else if (dinfo->pmu_init_chgvol < 4600) {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x05);
	} else {
		regmap_update_bits(regmap, axp2101_CHG_V_CFG, 0x07, 0x00);
	}
	/* set button battery charge termination voltage to 2.9v */
	regmap_update_bits(regmap, axp2101_BTN_CHG_CFG, 0x07, 0x03);
	/* set chglend func 0x69 */
	regmap_update_bits(regmap, axp2101_CHGLED_CFG, 0x06,
			   dinfo->pmu_chgled_type << 1);
	/* set gauge_thld */
	value = clamp_val(dinfo->pmu_battery_warning_level1 - 5, 0, 15) << 4;
	value |= clamp_val(dinfo->pmu_battery_warning_level2, 0, 15);
	regmap_write(regmap, axp2101_GAUGE_THLD, value);
}

static uint32_t iin_lim_tbl[] = {100, 500, 900, 1000, 1500, 2000};

int axp2101_charger_dt_parse(struct device_node *node,
			 struct axp_config_info *axp_config)
{
	if (!of_device_is_available(node)) {
		pr_err("%s: failed\n", __func__);
		return -1;
	}

	AXP_OF_PROP_READ(pmu_battery_rdc,              BATRDC);
	AXP_OF_PROP_READ(pmu_battery_cap,                4000);
	AXP_OF_PROP_READ(pmu_batdeten,                      1);
	AXP_OF_PROP_READ(pmu_chg_ic_temp,                   0);
	AXP_OF_PROP_READ(pmu_runtime_chgcur, INTCHGCUR / 1000);
	AXP_OF_PROP_READ(pmu_suspend_chgcur,             1200);
	AXP_OF_PROP_READ(pmu_shutdown_chgcur,            1200);
	AXP_OF_PROP_READ(pmu_init_chgvol,    INTCHGVOL / 1000);
	AXP_OF_PROP_READ(pmu_init_chgend_rate,  INTCHGENDRATE);
	AXP_OF_PROP_READ(pmu_init_chg_enabled,              1);
	AXP_OF_PROP_READ(pmu_init_bc_en,                    0);
	AXP_OF_PROP_READ(pmu_init_adc_freq,        INTADCFREQ);
	AXP_OF_PROP_READ(pmu_init_adcts_freq,     INTADCFREQC);
	AXP_OF_PROP_READ(pmu_init_chg_pretime,  INTCHGPRETIME);
	AXP_OF_PROP_READ(pmu_init_chg_csttime,  INTCHGCSTTIME);
	AXP_OF_PROP_READ(pmu_batt_cap_correct,              1);
	AXP_OF_PROP_READ(pmu_chg_end_on_en,                 0);
	AXP_OF_PROP_READ(ocv_coulumb_100,                   0);
	AXP_OF_PROP_READ(pmu_bat_para1,               OCVREG0);
	AXP_OF_PROP_READ(pmu_bat_para2,               OCVREG1);
	AXP_OF_PROP_READ(pmu_bat_para3,               OCVREG2);
	AXP_OF_PROP_READ(pmu_bat_para4,               OCVREG3);
	AXP_OF_PROP_READ(pmu_bat_para5,               OCVREG4);
	AXP_OF_PROP_READ(pmu_bat_para6,               OCVREG5);
	AXP_OF_PROP_READ(pmu_bat_para7,               OCVREG6);
	AXP_OF_PROP_READ(pmu_bat_para8,               OCVREG7);
	AXP_OF_PROP_READ(pmu_bat_para9,               OCVREG8);
	AXP_OF_PROP_READ(pmu_bat_para10,              OCVREG9);
	AXP_OF_PROP_READ(pmu_bat_para11,              OCVREGA);
	AXP_OF_PROP_READ(pmu_bat_para12,              OCVREGB);
	AXP_OF_PROP_READ(pmu_bat_para13,              OCVREGC);
	AXP_OF_PROP_READ(pmu_bat_para14,              OCVREGD);
	AXP_OF_PROP_READ(pmu_bat_para15,              OCVREGE);
	AXP_OF_PROP_READ(pmu_bat_para16,              OCVREGF);
	AXP_OF_PROP_READ(pmu_bat_para17,             OCVREG10);
	AXP_OF_PROP_READ(pmu_bat_para18,             OCVREG11);
	AXP_OF_PROP_READ(pmu_bat_para19,             OCVREG12);
	AXP_OF_PROP_READ(pmu_bat_para20,             OCVREG13);
	AXP_OF_PROP_READ(pmu_bat_para21,             OCVREG14);
	AXP_OF_PROP_READ(pmu_bat_para22,             OCVREG15);
	AXP_OF_PROP_READ(pmu_bat_para23,             OCVREG16);
	AXP_OF_PROP_READ(pmu_bat_para24,             OCVREG17);
	AXP_OF_PROP_READ(pmu_bat_para25,             OCVREG18);
	AXP_OF_PROP_READ(pmu_bat_para26,             OCVREG19);
	AXP_OF_PROP_READ(pmu_bat_para27,             OCVREG1A);
	AXP_OF_PROP_READ(pmu_bat_para28,             OCVREG1B);
	AXP_OF_PROP_READ(pmu_bat_para29,             OCVREG1C);
	AXP_OF_PROP_READ(pmu_bat_para30,             OCVREG1D);
	AXP_OF_PROP_READ(pmu_bat_para31,             OCVREG1E);
	AXP_OF_PROP_READ(pmu_bat_para32,             OCVREG1F);
	AXP_OF_PROP_READ(pmu_ac_vol,                     4400);
	AXP_OF_PROP_READ(pmu_usbpc_vol,                  4400);
	AXP_OF_PROP_READ(pmu_ac_cur,                        0);
	AXP_OF_PROP_READ(pmu_usbpc_cur,                     0);
	AXP_OF_PROP_READ(pmu_pwroff_vol,                 3300);
	AXP_OF_PROP_READ(pmu_pwron_vol,                  2900);
	AXP_OF_PROP_READ(pmu_battery_warning_level1,       15);
	AXP_OF_PROP_READ(pmu_battery_warning_level2,        0);
	AXP_OF_PROP_READ(pmu_restvol_adjust_time,          30);
	AXP_OF_PROP_READ(pmu_ocv_cou_adjust_time,          60);
	AXP_OF_PROP_READ(pmu_chgled_func,                   0);
	AXP_OF_PROP_READ(pmu_chgled_type,                   0);
	AXP_OF_PROP_READ(pmu_bat_temp_enable,               0);
	AXP_OF_PROP_READ(pmu_bat_charge_ltf,             0xA5);
	AXP_OF_PROP_READ(pmu_bat_charge_htf,             0x1F);
	AXP_OF_PROP_READ(pmu_bat_shutdown_ltf,           0xFC);
	AXP_OF_PROP_READ(pmu_bat_shutdown_htf,           0x16);
	AXP_OF_PROP_READ(pmu_bat_temp_para1,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para2,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para3,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para4,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para5,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para6,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para7,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para8,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para9,                0);
	AXP_OF_PROP_READ(pmu_bat_temp_para10,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para11,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para12,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para13,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para14,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para15,               0);
	AXP_OF_PROP_READ(pmu_bat_temp_para16,               0);
	AXP_OF_PROP_READ(pmu_bat_unused,                    0);
	AXP_OF_PROP_READ(power_start,                       0);
	AXP_OF_PROP_READ(pmu_ocv_en,                        1);
	AXP_OF_PROP_READ(pmu_cou_en,                        1);
	AXP_OF_PROP_READ(pmu_update_min_time,   UPDATEMINTIME);

	axp_config->wakeup_usb_in =
		of_property_read_bool(node, "wakeup_usb_in");
	axp_config->wakeup_usb_out =
		of_property_read_bool(node, "wakeup_usb_out");
	axp_config->wakeup_bat_in =
		of_property_read_bool(node, "wakeup_bat_in");
	axp_config->wakeup_bat_out =
		of_property_read_bool(node, "wakeup_bat_out");
	axp_config->wakeup_bat_charging =
		of_property_read_bool(node, "wakeup_bat_charging");
	axp_config->wakeup_bat_charge_over =
		of_property_read_bool(node, "wakeup_bat_charge_over");
	axp_config->wakeup_low_warning1 =
		of_property_read_bool(node, "wakeup_low_warning1");
	axp_config->wakeup_low_warning2 =
		of_property_read_bool(node, "wakeup_low_warning2");
	axp_config->wakeup_bat_untemp_work =
		of_property_read_bool(node, "wakeup_bat_untemp_work");
	axp_config->wakeup_bat_ovtemp_work =
		of_property_read_bool(node, "wakeup_bat_ovtemp_work");
	axp_config->wakeup_untemp_chg =
		of_property_read_bool(node, "wakeup_bat_untemp_chg");
	axp_config->wakeup_ovtemp_chg =
		of_property_read_bool(node, "wakeup_bat_ovtemp_chg");
	/* axp_config_obj = axp_config; */
	return 0;
}

static void axp2101_icchg_set(struct axp210x_device_info *di, int mA)
{
	if (mA <= 200)
		mA = mA / 25;
	else
		mA = (mA - 200) / 100 + 8;

	/* bit 4:0 is the ctrl bit */
	regmap_update_bits(di->regmap, axp2101_ICC_CFG, 0x1f, mA);
}



static void axp2101_parse_device_tree(struct axp210x_device_info *di)
{

#if 1
	int32_t prop = 0,i;

	di->dts_info.pmu_chg_ic_temp = 0;
	di->dts_info.pmu_runtime_chgcur = 360; //ʵ��320
	di->dts_info.pmu_suspend_chgcur = 360; 
	di->dts_info.pmu_shutdown_chgcur = 360;
	di->dts_info.pmu_init_chgvol = 4400;
	di->dts_info.pmu_usbpc_cur = 500;
	di->dts_info.pmu_chgled_type = 0;

	//charge cur limit2
	axp2101_icchg_set(di, di->dts_info.pmu_runtime_chgcur);

	/* charge voltage */
	if (di->dts_info.pmu_init_chgvol < 4100) {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x01);
	} else if (di->dts_info.pmu_init_chgvol < 4200) {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x02);
	} else if (di->dts_info.pmu_init_chgvol < 4350) {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x03);
	} else if (di->dts_info.pmu_init_chgvol < 4400) {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x04);
	} else if (di->dts_info.pmu_init_chgvol < 4600) {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x05);
	} else {
		regmap_update_bits(di->regmap, axp2101_CHG_V_CFG, 0x07, 0x00);
	}
	
	// charge cur limit1
	prop = di->dts_info.pmu_usbpc_cur;
	for (i = 0; i < ARRAY_SIZE(iin_lim_tbl); i++) {
		if (prop < iin_lim_tbl[i])
			break;
	}
	i = i ? i - 1 : i;
	if (prop > iin_lim_tbl[ARRAY_SIZE(iin_lim_tbl) - 1])
		i = GENMASK(2, 0);
	regmap_update_bits(di->regmap,
			   axp2101_regaddrs[AXP210X_REG_IIN_LIM],
			   GENMASK(2, 0), i);

	if (di->dts_info.pmu_chg_ic_temp)
		regmap_update_bits(di->regmap, axp2101_TS_CFG, 0x0a, 0x0a);
	else
		regmap_update_bits(di->regmap, axp2101_TS_CFG, 0x0a, 0x00);	


	regmap_update_bits(di->regmap, axp2101_CHGLED_CFG, 0x06,
			   di->dts_info.pmu_chgled_type << 1);

	//***term current limit****
	regmap_update_bits(di->regmap, axp2101_ITERM_CFG, 0x0f, 0x02);
#else 
	int ret;
	uint32_t prop = 0, i;
	struct axp_config_info *cfg;

	/* set input current limit */
	if (!di->dev->of_node) {
		pr_info("can not find device tree\n");
		return;
	}

	cfg = &di->dts_info;
	ret = axp2101_charger_dt_parse(di->dev->of_node, cfg);
	if (ret) {
		pr_info("can not parse device tree err\n");
		return;
	}

	/* old sysconfig parse */
	axp2101_charger_sysconfig(di);

	/* prechg default change to 100mA */
	if (!of_property_read_u32(di->dev->of_node, "pmu_pre_chg", &prop))
		regmap_update_bits(di->regmap, axp2101_IPRECHG_CFG, 0x0f,
				   prop / 25);
	else
		regmap_update_bits(di->regmap, axp2101_IPRECHG_CFG, 0x0f, 0x04);

	/* Termination default current limit to 50mA */
	if (!of_property_read_u32(di->dev->of_node, "pmu_iterm_limit", &prop)) {
		if (prop) {
			regmap_update_bits(di->regmap, axp2101_ITERM_CFG, 0x0f,
					   prop / 25);
			regmap_update_bits(di->regmap, axp2101_ITERM_CFG,
					   BIT(4), BIT(4));
		} else {
			regmap_write(di->regmap, axp2101_ITERM_CFG, 0x00);
		}
	} else {
		regmap_update_bits(di->regmap, axp2101_ITERM_CFG, 0x0f, 0x02);
	}

	if (!of_property_read_u32(di->dev->of_node, "pmu_chled_enable",
				  &prop) &&
	    !prop) {
		regmap_update_bits(di->regmap, axp2101_CHGLED_CFG, 0x01, 0x00);
	}

	prop = di->dts_info.pmu_usbpc_cur;
	if (!of_property_read_u32(di->dev->of_node, "iin_limit", &prop) ||
	    true) {
		for (i = 0; i < ARRAY_SIZE(iin_lim_tbl); i++) {
			if (prop < iin_lim_tbl[i])
				break;
		}

		i = i ? i - 1 : i;

		if (prop > iin_lim_tbl[ARRAY_SIZE(iin_lim_tbl) - 1])
			i = GENMASK(2, 0);

		regmap_update_bits(di->regmap,
				   axp2101_regaddrs[AXP210X_REG_IIN_LIM],
				   GENMASK(2, 0), i);
	}

	prop = di->dts_info.pmu_runtime_chgcur;
	if (!of_property_read_u32(di->dev->of_node, "icc_cfg", &prop) || true) {
		prop = clamp_val(prop, 0, 2000);
		/* step is 25mA, and then 100mA step */
		if (prop <= 200)
			prop /= 25;
		else
			prop = 8 + (prop - 200) / 100;

		regmap_update_bits(di->regmap,
				   axp2101_regaddrs[AXP210X_REG_ICC_CFG],
				   GENMASK(4, 0), prop);
	}

	if (!of_property_read_u32(di->dev->of_node, "pmu_bat_det", &prop)) {
		regmap_write(di->regmap, axp2101_BAT_DET, (unsigned int)prop);
	}

	if (of_property_read_bool(di->dev->of_node, "pmu_btn_chg_en")) {
		regmap_update_bits(di->regmap, axp2101_MODULE_EN, BIT(2),
				   BIT(2));
	} else {
		regmap_update_bits(di->regmap, axp2101_MODULE_EN, BIT(2), 0);
	}

	if (!of_property_read_u32(di->dev->of_node, "pmu_btn_chg_cfg", &prop)) {
		prop = (prop - 2600) / 100;
		regmap_write(di->regmap, axp2101_BTN_CHG_CFG,
			     (unsigned int)(prop & 0x07));
	}
	#endif
}

static void battery_set_full(int *rs)
{
	static ktime_t l_time;

	if (ktime_ms_delta(ktime_get(), l_time) > MSEC_PER_SEC) {
		(*rs) = true;
		//WRITE_ONCE(*rs, true);
	}

	l_time = ktime_get();
}

static void battery_chk_online_v1(struct work_struct *work)
{
	int ret;
	uint8_t data[2] = {0};
	static ktime_t s_chg = { .tv64 = 0 };
	struct axp210x_device_info *di =
		container_of(work, typeof(*di), bat_chk.work);

	if (di->stat.bat_init == 0) {

		di->stat.bat_init = 1;

		schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));

		return;

	} else if (di->stat.bat_init == 1) {

		di->stat.bat_init = 2;

		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_REG_COMSTAT0], data, 1);
		if (data[0] & BIT(5)) {
			/* reset_mcu */
			ret = axp210x_reset_mcu();
			if (ret < 0)
				goto err_read;

			msleep(10);
		}
		di->stat.bat_read = true;
		//WRITE_ONCE(di->stat.bat_read, true);

		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_MODULE_EN], data, 1);
		if (ret < 0)
			goto err_read;

		data[0] |= BIT(1);
		ret = axp210x_info->write(
			axp210x_info->regaddrs[AXP210X_MODULE_EN], data, 1);
		if (ret < 0)
			goto err_read;

		schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));

		return;
	}

	/*
	 * check_full of batt, because bug of axp2101b, full flag can generate
	 * in batt plugged out
	 */
	ret = di->read(di->regaddrs[AXP210X_COMM_STAT1], data, 1);
	if (ret < 0) {
		pr_info("read AXP210X_REG_VBAT error\n");
		goto err_read;
	}
	/* chg_stat is full with bit[2:0] is b100 */
	if ((data[0] & 0x07) == 0x04) {
		battery_set_full(&di->stat.bat_full);
	} else {
		di->stat.bat_full = false;
		//WRITE_ONCE(di->stat.bat_full, false);
	}

	ret = di->read(di->regaddrs[AXP210X_COMM_STAT0], data, 1);
	if (ret < 0) {
		pr_info("read AXP210X_COMM_STAT0 error\n");
		goto err_read;
	}

	if ((!(data[0] & BIT(3))) && (di->stat.bat_stat != 0)) {
		pr_debug("bat_stat change to none\n");
		di->stat.bat_stat = 0;
		di->stat.bat_read = false;
		//WRITE_ONCE(di->stat.bat_read, false);
		axp210x_reset_gauge();
		s_chg = ktime_get();
		regmap_update_bits(di->regmap,
				   axp2101_regaddrs[AXP210X_CHGLED_CFG], BIT(0),
				   0);
		power_supply_changed(&di->bat);
	}
	if ((data[0] & BIT(3)) && (di->stat.bat_stat != 1)) {
		pr_debug("bat_stat change to exist\n");
		di->stat.bat_stat = 1;
		axp210x_reset_gauge();
		s_chg = ktime_get();
		regmap_update_bits(di->regmap,
				   axp2101_regaddrs[AXP210X_CHGLED_CFG], BIT(0),
				   1);
		power_supply_changed(&di->bat);
	}

	if (di->stat.bat_stat &&
	    ktime_ms_delta(ktime_get(), s_chg) > MSEC_PER_SEC) {
	    di->stat.bat_read = true;
		//WRITE_ONCE(di->stat.bat_read, true);
	}

err_read:
	schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));
}

static void battery_chk_online(struct work_struct *work)
{
	int ret;
	static int cnt;
	static int rst[3] = { 1, 1, 1 };
	uint8_t data[2] = {0};
	static ktime_t s_chg = { .tv64 = 0 };
	struct axp210x_device_info *di =
		container_of(work, typeof(*di), bat_chk.work);

	if (di->stat.bat_init == 0) {

		di->stat.bat_init = 1;

		schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));

		return;

	} else if (di->stat.bat_init == 1) {

		di->stat.bat_init = 2;

		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_REG_COMSTAT0], data, 1);
		if (data[0] & BIT(5)) {
			/* reset_mcu */
			ret = axp210x_reset_mcu();
			if (ret < 0)
				goto err_read;

			msleep(10);
		}
		di->stat.bat_read = true;
		//WRITE_ONCE(di->stat.bat_read, true);

		ret = axp210x_info->read(
			axp210x_info->regaddrs[AXP210X_MODULE_EN], data, 1);
		if (ret < 0)
			goto err_read;

		data[0] |= BIT(1);
		ret = axp210x_info->write(
			axp210x_info->regaddrs[AXP210X_MODULE_EN], data, 1);
		if (ret < 0)
			goto err_read;

		schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));

		return;
	}

	/*
	 * check_full of batt, because bug of axp2101b, full flag can generate
	 * in batt plugged out
	 */
	ret = di->read(di->regaddrs[AXP210X_COMM_STAT1], data, 1);
	if (ret < 0) {
		pr_info("read AXP210X_REG_VBAT error\n");
		goto err_read;
	}
	/* chg_stat is full with bit[2:0] is b100 */
	if ((data[0] & 0x07) == 0x04) {
		battery_set_full(&di->stat.bat_full);
	} else {
		di->stat.bat_full = false;
		//WRITE_ONCE(di->stat.bat_full, false);
	}

	ret = di->read(di->regaddrs[AXP210X_REG_VBAT], data, 2);
	if (ret < 0) {
		pr_info("read AXP210X_REG_VBAT error\n");
		goto err_read;
	}

	rst[cnt] = (((data[0] & GENMASK(5, 0)) << 0x08) | (data[1]));
	if (rst[cnt] < AXP210X_VBAT_MIN)
		rst[cnt] = 0;
	cnt = ++cnt == 3 ? 0 : cnt;

	/* there's one zero to indicate battery disconnect */
	if (!rst[0] || !rst[1] || !rst[2]) {
		if (di->stat.bat_stat != 0) {
			pr_debug("bat_stat change to none\n");
			di->stat.bat_stat = 0;
			di->stat.bat_read = false;
			//WRITE_ONCE(di->stat.bat_read, false);
			axp210x_reset_gauge();
			s_chg = ktime_get();
			regmap_update_bits(di->regmap,
				      axp2101_regaddrs[AXP210X_CHGLED_CFG],
				      BIT(0), 0);
			power_supply_changed(&di->bat);
		}
	} else {
		if (di->stat.bat_stat != 1) {
			pr_debug("bat_stat change to exist\n");
			di->stat.bat_stat = 1;
			axp210x_reset_gauge();
			s_chg = ktime_get();
			regmap_update_bits(di->regmap,
				      axp2101_regaddrs[AXP210X_CHGLED_CFG],
				      BIT(0), 1);
			power_supply_changed(&di->bat);
		}
	}

	if (di->stat.bat_stat &&
	    ktime_ms_delta(ktime_get(), s_chg) > MSEC_PER_SEC) {
	    di->stat.bat_read = true;
		//WRITE_ONCE(di->stat.bat_read, true);
	}

err_read:
	schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));
}

/**
 * axp2101_get_param - get battery config from dts
 *
 * is not get battery config parameter from dts,
 * then it use the default config.
 */
static int axp2101_get_param(struct axp210x_device_info *di)
{
	#if 0
	struct device_node *n_para, *r_para;
	const char *pparam;
	int cnt;
	void *pres;

	pres = devm_kzalloc(di->dev, AXP210X_MAX_PARAM, GFP_KERNEL);
	if (!pres)
		return -ENOMEM;

	n_para = of_parse_phandle(di->dev->of_node, "param", 0);
	if (!n_para)
		goto e_n_para;

	if (of_property_read_string(n_para, "select", &pparam))
		goto e_para;

	r_para = of_get_child_by_name(n_para, pparam);
	if (!r_para)
		goto e_para;

	cnt = of_property_read_variable_u8_array(r_para, "parameter", pres, 1,
						 AXP210X_MAX_PARAM);
	if (cnt <= 0)
		goto e_n_parameter;

	di->data.model = pres;
	di->data.model_size = cnt;
	
	of_node_put(r_para);
	of_node_put(n_para);

	return 0;

e_n_parameter:
	of_node_put(r_para);
e_para:
	of_node_put(n_para);
e_n_para:
	devm_kfree(di->dev, pres);

	return -ENODATA;
	#endif
	return 0;
}

static ssize_t charger_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct axp210x_device_info *di = axp210x_info;

	return sprintf(buf, "%d\n", !di->stat.charger_disable);
}

static ssize_t charger_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct axp210x_device_info *di = axp210x_info;
	long val;
	ssize_t rc;

	rc = kstrtol(buf, 0, &val);
	if (rc)
		return rc;

	if (val) {
		/* enable */
		di->stat.charger_disable = 0;
		axp2101_icchg_set(di, di->dts_info.pmu_runtime_chgcur);
	} else {
		/* disable */
		di->stat.charger_disable = 1;
		axp2101_icchg_set(di, 0);
	}

	return count;
}

DEVICE_ATTR_RW(charger);

static int axp210x_chip_id(struct axp210x_device_info *di)
{
	int ret;
	u8 data;

	ret = di->read(di->regaddrs[AXP210X_CHIP_ID], &data, 1);
	if (ret < 0) {
		pr_info("read AXP210X_CHIP_ID error\n");
		return -ENODEV;
	}

	data &= 0xcf;
	if (data == 0x47 || data == 0x4a) {
		pr_info("read chipid = %x\n", data);
		if (data == 0x4a)
			di->version = 1;
		else
			di->version = 0;

		return 0;
	}

	return -ENODEV;


}

static int axp2101_charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	//int i = 0, irq;
	struct axp210x_device_info *di;

	struct axp20x_dev *axp_dev = dev_get_drvdata(pdev->dev.parent);

	if (!axp_dev->irq) {
		pr_err("can not register axp2101-charger without irq\n");
		return -EINVAL;
	}

	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (di == NULL) {
		axp210x_err("axp210x_device_info alloc failed\n");
		ret = -ENOMEM;
		goto err;
	}

	di->name = "axp210x_chip";
	di->dev = &pdev->dev;
	di->chip = AXP2101;
	di->read = axp210x_read;
	di->write = axp210x_write;
	di->regaddrs = axp2101_regaddrs;
	di->data = axp2101_model_data;
	di->regmap = axp_dev->regmap;
	di->stat.bat_stat = true;

	di->stat.bat_init = 2;

	axp_set_charger_info(di);
	/* for device tree parse */
	axp2101_parse_device_tree(di);
	/* get param frome device tree */
	axp2101_get_param(di);

	ret = axp210x_chip_id(di);
	if (ret < 0)
		goto err;

	ret = axp210x_init_chip(axp210x_info);
	if (ret < 0) {
		axp210x_err("axp210x init chip fail!\n");
		ret = -ENODEV;
		goto err;
	}

	ret = axp210x_register_battery(axp210x_info);
	if (ret < 0) {
		axp210x_err("axp210x register battery dev fail!\n");
		goto err;
	}

	device_create_file(axp210x_info->bat.dev, &dev_attr_charger);

#if ((defined DONOT_Correction) || (defined POLL_READ))
	di->poll_read = kthread_run(thread_dosomthing, di, "axp2101");

#else
	#if 0
	for (i = 0; i < ARRAY_SIZE(axp_charger_irq); i++) {
		irq = platform_get_irq_byname(pdev, axp_charger_irq[i].name);
		if (irq < 0)
			continue;

		irq = regmap_irq_get_virq(axp_dev->regmap_irqc, irq);
		if (irq < 0) {
			dev_err(&pdev->dev, "can not get irq\n");
			return irq;
		}
		/* we use this variable to suspend irq */
		axp_charger_irq[i].irq = irq;
		ret = devm_request_any_context_irq(&pdev->dev, irq,
						   axp_charger_irq[i].isr, 0,
						   axp_charger_irq[i].name, di);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request %s IRQ %d: %d\n",
				axp_charger_irq[i].name, irq, ret);
			return ret;
		} else {
			ret = 0;
		}

		dev_dbg(&pdev->dev, "Requested %s IRQ %d: %d\n",
			axp_charger_irq[i].name, irq, ret);
	}
	#endif
#endif
	platform_set_drvdata(pdev, di);

	if (di->version == 1)
		INIT_DELAYED_WORK(&di->bat_chk, battery_chk_online_v1);
	else
		INIT_DELAYED_WORK(&di->bat_chk, battery_chk_online);

	schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));
#if (AXP2101_DEBUG)
	ret = class_register(&axp210x_user_define);
	if (ret < 0) {
		axp210x_err("axp210x register class fail!\n");
	}
#endif

	return ret;

#if ((defined DONOT_Correction) || (defined POLL_READ))
	kthread_stop(di->poll_read);
#endif

err:
	axp210x_err("%s,probe fail, ret = %d\n", __func__, ret);

	return ret;
}

static int axp2101_charger_remove(struct platform_device *pdev)
{
	axp210x_alway("==============AXP2101 unegister==============\n");
#if ((defined DONOT_Correction) || (defined POLL_READ))
	kthread_stop(axp210x_info->poll_read);
#endif
	axp210x_teardown_battery(axp210x_info);
	axp210x_debug("axp210x teardown battery dev\n");

#if (AXP2101_DEBUG)
	class_unregister(&axp210x_user_define);
#endif

	axp210x_info = NULL;
	return 0;
}

static inline void axp2101_irq_set(unsigned int irq, bool enable)
{
	if (enable)
		enable_irq(irq);
	else
		disable_irq(irq);
}

static void axp2101_virq_dts_set(struct axp210x_device_info *di, bool enable)
{
	#if 0
	struct axp_config_info *dts_info = &di->dts_info;

	if (!dts_info->wakeup_usb_in)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_USB_IN].irq,
				enable);
	if (!dts_info->wakeup_usb_out)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_USB_OUT].irq,
				enable);
	if (!dts_info->wakeup_bat_in)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_BAT_IN].irq,
				enable);
	if (!dts_info->wakeup_bat_out)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_BAT_OUT].irq,
				enable);
	if (!dts_info->wakeup_bat_charging)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_CHARGING].irq,
				enable);
	if (!dts_info->wakeup_bat_charge_over)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_CHARGE_OVER].irq,
				enable);
	if (!dts_info->wakeup_low_warning1)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_LOW_WARNING1].irq,
				enable);
	if (!dts_info->wakeup_low_warning2)
		axp2101_irq_set(axp_charger_irq[AXP2101_VIRQ_LOW_WARNING2].irq,
				enable);
	if (!dts_info->wakeup_bat_untemp_work)
		axp2101_irq_set(
			axp_charger_irq[AXP2101_VIRQ_BAT_UNTEMP_WORK].irq,
			enable);
	if (!dts_info->wakeup_bat_ovtemp_work)
		axp2101_irq_set(
			axp_charger_irq[AXP2101_VIRQ_BAT_OVTEMP_WORK].irq,
			enable);
	if (!dts_info->wakeup_untemp_chg)
		axp2101_irq_set(
			axp_charger_irq[AXP2101_VIRQ_BAT_UNTEMP_CHG].irq,
			enable);
	if (!dts_info->wakeup_ovtemp_chg)
		axp2101_irq_set(
			axp_charger_irq[AXP2101_VIRQ_BAT_OVTEMP_CHG].irq,
			enable);
	#endif
}

static void axp2101_shutdown(struct platform_device *p)
{
	struct axp210x_device_info *di = platform_get_drvdata(p);

	/*
	 * for reduce shutdown current
	 */
	regmap_write(di->regmap, AXP2101_VBAT_H, 0);

	cancel_delayed_work_sync(&di->bat_chk);

	axp2101_icchg_set(di, di->dts_info.pmu_shutdown_chgcur);
}

static int axp2101_suspend(struct platform_device *p, pm_message_t state)
{
	struct axp210x_device_info *di = platform_get_drvdata(p);

	if (!di->stat.charger_disable)
		axp2101_icchg_set(di, di->dts_info.pmu_suspend_chgcur);

	cancel_delayed_work_sync(&di->bat_chk);

	axp2101_virq_dts_set(di, false);
	return 0;
}

static int axp2101_resume(struct platform_device *p)
{
	struct axp210x_device_info *di = platform_get_drvdata(p);

	if (!di->stat.charger_disable)
		axp2101_icchg_set(di, di->dts_info.pmu_runtime_chgcur);
	else
		axp2101_icchg_set(di, 0);

	schedule_delayed_work(&di->bat_chk, msecs_to_jiffies(500));

	axp2101_virq_dts_set(di, true);
	return 0;
}

static const struct platform_device_id axp2101_charger_dt_ids[] = {
	{ .name = "axp2101-power-supply", },
	{},
};
MODULE_DEVICE_TABLE(of, axp2101_charger_dt_ids);

static struct platform_driver axp210x_charger_driver = {
	.driver = {
		.name = "axp2101-power-supply",
	},
	.probe = axp2101_charger_probe,
	.remove = axp2101_charger_remove,
	.id_table = axp2101_charger_dt_ids,
	.shutdown = axp2101_shutdown,
	.suspend = axp2101_suspend,
	.resume = axp2101_resume,
};

module_platform_driver(axp210x_charger_driver);

MODULE_AUTHOR("wangxiaoliang <wangxiaoliang@x-powers.com>");
MODULE_DESCRIPTION("axp210x i2c driver");
MODULE_LICENSE("GPL");
