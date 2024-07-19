
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/usb/phy.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/interrupt.h>


#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/version.h>


#if defined(CONFIG_PRODUCT_X1000_ECMINI)
static int bq25890_int_gpio = GPIO_PD(4);        //charge ic int
#else
static int bq25890_int_gpio = GPIO_PA(2);        //charge ic int
#endif
//********************x2000 copy cyadd****************


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


struct reg_field {
    unsigned int reg;
    unsigned int lsb;
    unsigned int msb;
    unsigned int id_size;
    unsigned int id_offset;
};

struct regmap_field {
    struct regmap *regmap;
    unsigned int mask;
    /* lsb */
    unsigned int shift;
    unsigned int reg;

    unsigned int id_size;
    unsigned int id_offset;
};


#define REG_FIELD(_reg, _lsb, _msb) {       \
    .reg = _reg,    \
    .lsb = _lsb,    \
    .msb = _msb,    \
    }

#define read_poll_timeout(op, val, cond, sleep_us, timeout_us, \
                sleep_before_read, args...) \
({ \
    u64 __timeout_us = (timeout_us); \
    unsigned long __sleep_us = (sleep_us); \
    ktime_t __timeout = ktime_add_us(ktime_get(), __timeout_us); \
    might_sleep_if((__sleep_us) != 0); \
    if (sleep_before_read && __sleep_us) \
        usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
    for (;;) { \
        (val) = op(args); \
        if (cond) \
            break; \
        if (__timeout_us && \
            ktime_compare(ktime_get(), __timeout) > 0) { \
            (val) = op(args); \
            break; \
        } \
        if (__sleep_us) \
            usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
    } \
    (cond) ? 0 : -ETIMEDOUT; \
})


#define regmap_field_read_poll_timeout(field, val, cond, sleep_us, timeout_us) \
({ \
    int __ret, __tmp; \
    __tmp = read_poll_timeout(regmap_field_read, __ret, __ret || (cond), \
            sleep_us, timeout_us, false, (field), &(val)); \
    __ret ?: __tmp; \
})



static void regmap_field_init(struct regmap_field *rm_field,
    struct regmap *regmap, struct reg_field reg_field)
{
    rm_field->regmap = regmap;
    rm_field->reg = reg_field.reg;
    rm_field->shift = reg_field.lsb;
    rm_field->mask = GENMASK(reg_field.msb, reg_field.lsb);
    rm_field->id_size = reg_field.id_size;
    rm_field->id_offset = reg_field.id_offset;
}
struct regmap_field *devm_regmap_field_alloc(struct device *dev,
        struct regmap *regmap, struct reg_field reg_field)
{
    struct regmap_field *rm_field = devm_kzalloc(dev,
                    sizeof(*rm_field), GFP_KERNEL);
    if (!rm_field)
        return ERR_PTR(-ENOMEM);

    regmap_field_init(rm_field, regmap, reg_field);

    return rm_field;

}

int regmap_field_read(struct regmap_field *field, unsigned int *val)
{
    int ret;
    unsigned int reg_val;
    ret = regmap_read(field->regmap, field->reg, &reg_val);
    if (ret != 0)
        return ret;

    reg_val &= field->mask;
    reg_val >>= field->shift;
    *val = reg_val;

    return ret;
}


int regmap_field_update_bits_base(struct regmap_field *field,
                  unsigned int mask, unsigned int val)
{
    mask = (mask << field->shift) & field->mask;

    return regmap_update_bits(field->regmap, field->reg, mask, val << field->shift);
}

static inline int regmap_field_write(struct regmap_field *field,
                     unsigned int val)
{
    return regmap_field_update_bits_base(field, ~0, val);
}


//**********************************************




#define BQ25890_MANUFACTURER		"Texas Instruments"
#define BQ25890_IRQ_PIN			"bq25890_irq"

#define BQ25890_ID			3
#define BQ25895_ID			7
#define BQ25896_ID			0

enum bq25890_chip_version {
	BQ25890,
	BQ25892,
	BQ25895,
	BQ25896,
};

static const char *const bq25890_chip_name[] = {
	"BQ25890",
	"BQ25892",
	"BQ25895",
	"BQ25896",
};

enum bq25890_fields {
	F_EN_HIZ, F_EN_ILIM, F_IILIM,				     /* Reg00 */
	F_BHOT, F_BCOLD, F_VINDPM_OFS,				     /* Reg01 */
	F_CONV_START, F_CONV_RATE, F_BOOSTF, F_ICO_EN,
	F_HVDCP_EN, F_MAXC_EN, F_FORCE_DPM, F_AUTO_DPDM_EN,	     /* Reg02 */
	F_BAT_LOAD_EN, F_WD_RST, F_OTG_CFG, F_CHG_CFG, F_SYSVMIN,
	F_MIN_VBAT_SEL,						     /* Reg03 */
	F_PUMPX_EN, F_ICHG,					     /* Reg04 */
	F_IPRECHG, F_ITERM,					     /* Reg05 */
	F_VREG, F_BATLOWV, F_VRECHG,				     /* Reg06 */
	F_TERM_EN, F_STAT_DIS, F_WD, F_TMR_EN, F_CHG_TMR,
	F_JEITA_ISET,						     /* Reg07 */
	F_BATCMP, F_VCLAMP, F_TREG,				     /* Reg08 */
	F_FORCE_ICO, F_TMR2X_EN, F_BATFET_DIS, F_JEITA_VSET,
	F_BATFET_DLY, F_BATFET_RST_EN, F_PUMPX_UP, F_PUMPX_DN,	     /* Reg09 */
	F_BOOSTV, F_PFM_OTG_DIS, F_BOOSTI,			     /* Reg0A */
	F_VBUS_STAT, F_CHG_STAT, F_PG_STAT, F_SDP_STAT, F_0B_RSVD,
	F_VSYS_STAT,						     /* Reg0B */
	F_WD_FAULT, F_BOOST_FAULT, F_CHG_FAULT, F_BAT_FAULT,
	F_NTC_FAULT,						     /* Reg0C */
	F_FORCE_VINDPM, F_VINDPM,				     /* Reg0D */
	F_THERM_STAT, F_BATV,					     /* Reg0E */
	F_SYSV,							     /* Reg0F */
	F_TSPCT,						     /* Reg10 */
	F_VBUS_GD, F_VBUSV,					     /* Reg11 */
	F_ICHGR,						     /* Reg12 */
	F_VDPM_STAT, F_IDPM_STAT, F_IDPM_LIM,			     /* Reg13 */
	F_REG_RST, F_ICO_OPTIMIZED, F_PN, F_TS_PROFILE, F_DEV_REV,   /* Reg14 */

	F_MAX_FIELDS
};

/* initial field values, converted to register values */
struct bq25890_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 boostv;	/* boost regulation voltage	*/
	u8 boosti;	/* boost current limit		*/
	u8 boostf;	/* boost frequency		*/
	u8 ilim_en;	/* enable ILIM pin		*/
	u8 treg;	/* thermal regulation threshold */
	u8 rbatcomp;	/* IBAT sense resistor value    */
	u8 vclamp;	/* IBAT compensation voltage limit */
};

struct bq25890_state {
	u8 online;
	u8 chrg_status;
	u8 chrg_fault;
	u8 vsys_status;
	u8 boost_fault;
	u8 bat_fault;
};

struct bq25890_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply charger;

	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	enum bq25890_chip_version chip_version;
	struct bq25890_init_data init_data;
	struct bq25890_state state;

	struct mutex lock; /* protect state data */
};


struct bq25890_device *gloal_bq25890_1 = NULL;



static const struct regmap_range bq25890_readonly_reg_ranges[] = {
	regmap_reg_range(0x0b, 0x0c),
	regmap_reg_range(0x0e, 0x13),
};

static const struct regmap_access_table bq25890_writeable_regs = {
	.no_ranges = bq25890_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25890_readonly_reg_ranges),
};

static const struct regmap_range bq25890_volatile_reg_ranges[] = {
	regmap_reg_range(0x00, 0x00),
	regmap_reg_range(0x02, 0x02),
	regmap_reg_range(0x09, 0x09),
	regmap_reg_range(0x0b, 0x14),
};

static const struct regmap_access_table bq25890_volatile_regs = {
	.yes_ranges = bq25890_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25890_volatile_reg_ranges),
};

static const struct regmap_config bq25890_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x14,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bq25890_writeable_regs,
	.volatile_table = &bq25890_volatile_regs,
};

static const struct reg_field bq25890_reg_fields[] = {
	/* REG00 */
	[F_EN_HIZ]		= REG_FIELD(0x00, 7, 7),
	[F_EN_ILIM]		= REG_FIELD(0x00, 6, 6),
	[F_IILIM]		= REG_FIELD(0x00, 0, 5),
	/* REG01 */
	[F_BHOT]		= REG_FIELD(0x01, 6, 7),
	[F_BCOLD]		= REG_FIELD(0x01, 5, 5),
	[F_VINDPM_OFS]		= REG_FIELD(0x01, 0, 4),
	/* REG02 */
	[F_CONV_START]		= REG_FIELD(0x02, 7, 7),
	[F_CONV_RATE]		= REG_FIELD(0x02, 6, 6),
	[F_BOOSTF]		= REG_FIELD(0x02, 5, 5),
	[F_ICO_EN]		= REG_FIELD(0x02, 4, 4),
	[F_HVDCP_EN]		= REG_FIELD(0x02, 3, 3),  // reserved on BQ25896
	[F_MAXC_EN]		= REG_FIELD(0x02, 2, 2),  // reserved on BQ25896
	[F_FORCE_DPM]		= REG_FIELD(0x02, 1, 1),
	[F_AUTO_DPDM_EN]	= REG_FIELD(0x02, 0, 0),
	/* REG03 */
	[F_BAT_LOAD_EN]		= REG_FIELD(0x03, 7, 7),
	[F_WD_RST]		= REG_FIELD(0x03, 6, 6),
	[F_OTG_CFG]		= REG_FIELD(0x03, 5, 5),
	[F_CHG_CFG]		= REG_FIELD(0x03, 4, 4),
	[F_SYSVMIN]		= REG_FIELD(0x03, 1, 3),
	[F_MIN_VBAT_SEL]	= REG_FIELD(0x03, 0, 0), // BQ25896 only
	/* REG04 */
	[F_PUMPX_EN]		= REG_FIELD(0x04, 7, 7),
	[F_ICHG]		= REG_FIELD(0x04, 0, 6),
	/* REG05 */
	[F_IPRECHG]		= REG_FIELD(0x05, 4, 7),
	[F_ITERM]		= REG_FIELD(0x05, 0, 3),
	/* REG06 */
	[F_VREG]		= REG_FIELD(0x06, 2, 7),
	[F_BATLOWV]		= REG_FIELD(0x06, 1, 1),
	[F_VRECHG]		= REG_FIELD(0x06, 0, 0),
	/* REG07 */
	[F_TERM_EN]		= REG_FIELD(0x07, 7, 7),
	[F_STAT_DIS]		= REG_FIELD(0x07, 6, 6),
	[F_WD]			= REG_FIELD(0x07, 4, 5),
	[F_TMR_EN]		= REG_FIELD(0x07, 3, 3),
	[F_CHG_TMR]		= REG_FIELD(0x07, 1, 2),
	[F_JEITA_ISET]		= REG_FIELD(0x07, 0, 0), // reserved on BQ25895
	/* REG08 */
	[F_BATCMP]		= REG_FIELD(0x08, 5, 7),
	[F_VCLAMP]		= REG_FIELD(0x08, 2, 4),
	[F_TREG]		= REG_FIELD(0x08, 0, 1),
	/* REG09 */
	[F_FORCE_ICO]		= REG_FIELD(0x09, 7, 7),
	[F_TMR2X_EN]		= REG_FIELD(0x09, 6, 6),
	[F_BATFET_DIS]		= REG_FIELD(0x09, 5, 5),
	[F_JEITA_VSET]		= REG_FIELD(0x09, 4, 4), // reserved on BQ25895
	[F_BATFET_DLY]		= REG_FIELD(0x09, 3, 3),
	[F_BATFET_RST_EN]	= REG_FIELD(0x09, 2, 2),
	[F_PUMPX_UP]		= REG_FIELD(0x09, 1, 1),
	[F_PUMPX_DN]		= REG_FIELD(0x09, 0, 0),
	/* REG0A */
	[F_BOOSTV]		= REG_FIELD(0x0A, 4, 7),
	[F_BOOSTI]		= REG_FIELD(0x0A, 0, 2), // reserved on BQ25895
	[F_PFM_OTG_DIS]		= REG_FIELD(0x0A, 3, 3), // BQ25896 only
	/* REG0B */
	[F_VBUS_STAT]		= REG_FIELD(0x0B, 5, 7),
	[F_CHG_STAT]		= REG_FIELD(0x0B, 3, 4),
	[F_PG_STAT]		= REG_FIELD(0x0B, 2, 2),
	[F_SDP_STAT]		= REG_FIELD(0x0B, 1, 1), // reserved on BQ25896
	[F_VSYS_STAT]		= REG_FIELD(0x0B, 0, 0),
	/* REG0C */
	[F_WD_FAULT]		= REG_FIELD(0x0C, 7, 7),
	[F_BOOST_FAULT]		= REG_FIELD(0x0C, 6, 6),
	[F_CHG_FAULT]		= REG_FIELD(0x0C, 4, 5),
	[F_BAT_FAULT]		= REG_FIELD(0x0C, 3, 3),
	[F_NTC_FAULT]		= REG_FIELD(0x0C, 0, 2),
	/* REG0D */
	[F_FORCE_VINDPM]	= REG_FIELD(0x0D, 7, 7),
	[F_VINDPM]		= REG_FIELD(0x0D, 0, 6),
	/* REG0E */
	[F_THERM_STAT]		= REG_FIELD(0x0E, 7, 7),
	[F_BATV]		= REG_FIELD(0x0E, 0, 6),
	/* REG0F */
	[F_SYSV]		= REG_FIELD(0x0F, 0, 6),
	/* REG10 */
	[F_TSPCT]		= REG_FIELD(0x10, 0, 6),
	/* REG11 */
	[F_VBUS_GD]		= REG_FIELD(0x11, 7, 7),
	[F_VBUSV]		= REG_FIELD(0x11, 0, 6),
	/* REG12 */
	[F_ICHGR]		= REG_FIELD(0x12, 0, 6),
	/* REG13 */
	[F_VDPM_STAT]		= REG_FIELD(0x13, 7, 7),
	[F_IDPM_STAT]		= REG_FIELD(0x13, 6, 6),
	[F_IDPM_LIM]		= REG_FIELD(0x13, 0, 5),
	/* REG14 */
	[F_REG_RST]		= REG_FIELD(0x14, 7, 7),
	[F_ICO_OPTIMIZED]	= REG_FIELD(0x14, 6, 6),
	[F_PN]			= REG_FIELD(0x14, 3, 5),
	[F_TS_PROFILE]		= REG_FIELD(0x14, 2, 2),
	[F_DEV_REV]		= REG_FIELD(0x14, 0, 1)
};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum bq25890_table_ids {
	/* range tables */
	TBL_ICHG,
	TBL_ITERM,
	TBL_IILIM,
	TBL_VREG,
	TBL_BOOSTV,
	TBL_SYSVMIN,
	TBL_VBATCOMP,
	TBL_RBATCOMP,

	/* lookup tables */
	TBL_TREG,
	TBL_BOOSTI,
};

/* Thermal Regulation Threshold lookup table, in degrees Celsius */
static const u32 bq25890_treg_tbl[] = { 60, 80, 100, 120 };

#define BQ25890_TREG_TBL_SIZE		ARRAY_SIZE(bq25890_treg_tbl)

/* Boost mode current limit lookup table, in uA */
static const u32 bq25890_boosti_tbl[] = {
	500000, 700000, 1100000, 1300000, 1600000, 1800000, 2100000, 2400000
};

#define BQ25890_BOOSTI_TBL_SIZE		ARRAY_SIZE(bq25890_boosti_tbl)

struct bq25890_range {
	u32 min;
	u32 max;
	u32 step;
};

struct bq25890_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct bq25890_range  rt;
	struct bq25890_lookup lt;
} bq25890_tables[] = {
	/* range tables */
	/* TODO: BQ25896 has max ICHG 3008 mA */
	[TBL_ICHG] =	{ .rt = {0,	  5056000, 64000} },	 /* uA */
	[TBL_ITERM] =	{ .rt = {64000,   1024000, 64000} },	 /* uA */
	[TBL_IILIM] =   { .rt = {100000,  3250000, 50000} },	 /* uA */
	[TBL_VREG] =	{ .rt = {3840000, 4608000, 16000} },	 /* uV */
	[TBL_BOOSTV] =	{ .rt = {4550000, 5510000, 64000} },	 /* uV */
	[TBL_SYSVMIN] = { .rt = {3000000, 3700000, 100000} },	 /* uV */
	[TBL_VBATCOMP] ={ .rt = {0,        224000, 32000} },	 /* uV */
	[TBL_RBATCOMP] ={ .rt = {0,        140000, 20000} },	 /* uOhm */

	/* lookup tables */
	[TBL_TREG] =	{ .lt = {bq25890_treg_tbl, BQ25890_TREG_TBL_SIZE} },
	[TBL_BOOSTI] =	{ .lt = {bq25890_boosti_tbl, BQ25890_BOOSTI_TBL_SIZE} }
};


int bq25890_1_mutex_lock(void)
{
	if(gloal_bq25890_1 != NULL)
	{
		mutex_lock(&gloal_bq25890_1->lock);
	}
	return 0;
}

int bq25890_1_mutex_ulock(void)
{
	if(gloal_bq25890_1 != NULL)
	{
		mutex_unlock(&gloal_bq25890_1->lock);
	}
	return 0;
}


unsigned int charge_status = 0;


int bq25890_get_chargestatus()
{
	return charge_status;
}

EXPORT_SYMBOL_GPL(bq25890_get_chargestatus);



static ssize_t
bq25890status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf,"%hu",charge_status);
}

static ssize_t
bq25890status_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t n)
{
        return n;
}	


static DEVICE_ATTR(bq25890_charge_status, 0644, bq25890status_show, bq25890status_store);


static int bq25890_field_read(struct bq25890_device *bq,
			      enum bq25890_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(bq->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}


static int bq25890_field_write(struct bq25890_device *bq,
			       enum bq25890_fields field_id, u8 val)
{
	return regmap_field_write(bq->rmap_fields[field_id], val);
}


static u8 bq25890_find_idx(u32 value, enum bq25890_table_ids id)
{
	u8 idx;

	if (id >= TBL_TREG) {
		const u32 *tbl = bq25890_tables[id].lt.tbl;
		u32 tbl_size = bq25890_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct bq25890_range *rtbl = &bq25890_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for (idx = 1;
		     idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value);
		     idx++)
			;
	}

	return idx - 1;
}

static u32 bq25890_find_val(u8 idx, enum bq25890_table_ids id)
{
	const struct bq25890_range *rtbl;

	/* lookup table? */
	if (id >= TBL_TREG)
		return bq25890_tables[id].lt.tbl[idx];

	/* range table */
	rtbl = &bq25890_tables[id].rt;

	return (rtbl->min + idx * rtbl->step);
}

enum bq25890_status {
	STATUS_NOT_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING,
	STATUS_TERMINATION_DONE,
};

enum bq25890_chrg_fault {
	CHRG_FAULT_NORMAL,
	CHRG_FAULT_INPUT,
	CHRG_FAULT_THERMAL_SHUTDOWN,
	CHRG_FAULT_TIMER_EXPIRED,
};

static bool bq25890_is_adc_property(enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return true;

	default:
		return false;
	}
}

static irqreturn_t __bq25890_handle_irq(struct bq25890_device *bq);

static int bq25890_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct bq25890_device *bq = gloal_bq25890_1;
	struct bq25890_state state;
	bool do_adc_conv;
	int ret;

	mutex_lock(&bq->lock);
	/* update state in case we lost an interrupt */
	__bq25890_handle_irq(bq);
	state = bq->state;
	do_adc_conv = !state.online && bq25890_is_adc_property(psp);
	
	if (do_adc_conv)
		bq25890_field_write(bq, F_CONV_START, 1);
	mutex_unlock(&bq->lock);
	if (do_adc_conv)
		regmap_field_read_poll_timeout(bq->rmap_fields[F_CONV_START],
			ret, !ret, 25000, 1000000);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING ||
			 state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!state.online || state.chrg_status == STATUS_NOT_CHARGING || state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else if (state.chrg_status == STATUS_PRE_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else /* unreachable */
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25890_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq25890_chip_name[bq->chip_version];
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!state.chrg_fault && !state.bat_fault && !state.boost_fault)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.chrg_fault == CHRG_FAULT_TIMER_EXPIRED)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else if (state.chrg_fault == CHRG_FAULT_THERMAL_SHUTDOWN)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq25890_find_val(bq->init_data.ichg, TBL_ICHG);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (!state.online) {
			val->intval = 0;
			break;
		}

		ret = bq25890_field_read(bq, F_BATV); /* read measured value */
		if (ret < 0)
			return ret;

		/* converted_val = 2.304V + ADC_val * 20mV (table 10.3.15) */
		val->intval = 2304000 + ret * 20000;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq25890_find_val(bq->init_data.vreg, TBL_VREG);
		break;
#if 0
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		val->intval = bq25890_find_val(bq->init_data.iprechg, TBL_ITERM);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq25890_find_val(bq->init_data.iterm, TBL_ITERM);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25890_field_read(bq, F_IILIM);
		if (ret < 0)
			return ret;

		val->intval = bq25890_find_val(ret, TBL_IILIM);
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq25890_field_read(bq, F_SYSV); /* read measured value */
		if (ret < 0)
			return ret;

		/* converted_val = 2.304V + ADC_val * 20mV (table 10.3.15) */
		val->intval = 2304000 + ret * 20000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq25890_field_read(bq, F_ICHGR); /* read measured value */
		if (ret < 0)
			return ret;

		/* converted_val = ADC_val * 50mA (table 10.3.19) */
		val->intval = ret * -50000;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bq25890_get_chip_state(struct bq25890_device *bq,
				  struct bq25890_state *state)
{
	int i, ret;

	struct {
		enum bq25890_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_PG_STAT,	&state->online},
		{F_VSYS_STAT,	&state->vsys_status},
		{F_BOOST_FAULT, &state->boost_fault},
		{F_BAT_FAULT,	&state->bat_fault},
		{F_CHG_FAULT,	&state->chrg_fault}
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = bq25890_field_read(bq, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	dev_dbg(bq->dev, "S:CHG/PG/VSYS=%d/%d/%d, F:CHG/BOOST/BAT=%d/%d/%d\n",
		state->chrg_status, state->online, state->vsys_status,
		state->chrg_fault, state->boost_fault, state->bat_fault);
	if(state->online == 1)
	{
		if(state->chrg_status == STATUS_PRE_CHARGING || state->chrg_status == STATUS_FAST_CHARGING)
		{
			charge_status = 1;
		}
		else if(state->chrg_status == STATUS_TERMINATION_DONE)
		{
			charge_status = 2;
		}
		else
		{
			charge_status = 0;
		}
	}
	else 
	{
		charge_status = 0;
	}

	return 0;
}

static irqreturn_t __bq25890_handle_irq(struct bq25890_device *bq)
{
	struct bq25890_state new_state;
	int ret;

	ret = bq25890_get_chip_state(bq, &new_state);
	if (ret < 0)
		return IRQ_NONE;

	if (!memcmp(&bq->state, &new_state, sizeof(new_state)))
		return IRQ_NONE;

	if (!new_state.online && bq->state.online) {	    /* power removed */
		/* disable ADC */
		ret = bq25890_field_write(bq, F_CONV_START, 0);
		if (ret < 0)
			goto error;
	} else if (new_state.online && !bq->state.online) { /* power inserted */
		/* enable ADC, to have control of charge current/voltage */
		ret = bq25890_field_write(bq, F_CONV_START, 1);
		if (ret < 0)
			goto error;
	}

	bq->state = new_state;
	power_supply_changed(&bq->charger);

	return IRQ_HANDLED;
error:
	dev_err(bq->dev, "Error communicating with the chip: %pe\n",
		ERR_PTR(ret));
	return IRQ_HANDLED;
}

static irqreturn_t bq25890_irq_handler_thread(int irq, void *private)
{
	struct bq25890_device *bq = private;
	irqreturn_t ret;
	mutex_lock(&bq->lock);
	ret = __bq25890_handle_irq(bq);
	mutex_unlock(&bq->lock);

	return ret;
}


static int bq25890_chip_reset(struct bq25890_device *bq)
{
	int ret;
	int rst_check_counter = 10;

	ret = bq25890_field_write(bq, F_REG_RST, 1);
	if (ret < 0)
		return ret;

	do {
		ret = bq25890_field_read(bq, F_REG_RST);
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}

static int bq25890_hw_init(struct bq25890_device *bq)
{
	int ret;
	int i;

	const struct {
		enum bq25890_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG,	 bq->init_data.ichg},
		{F_VREG,	 bq->init_data.vreg},
		{F_ITERM,	 bq->init_data.iterm},
		{F_IPRECHG,	 bq->init_data.iprechg},
		{F_SYSVMIN,	 bq->init_data.sysvmin},
		{F_BOOSTV,	 bq->init_data.boostv},
		{F_BOOSTI,	 bq->init_data.boosti},
		{F_BOOSTF,	 bq->init_data.boostf},
		{F_EN_ILIM,	 bq->init_data.ilim_en},
		{F_TREG,	 bq->init_data.treg},
		{F_BATCMP,	 bq->init_data.rbatcomp},
		{F_VCLAMP,	 bq->init_data.vclamp},
	};

	ret = bq25890_chip_reset(bq);
	if (ret < 0) {
		dev_dbg(bq->dev, "Reset failed %d\n", ret);
		return ret;
	}

	/* disable watchdog */
	ret = bq25890_field_write(bq, F_WD, 0);
	if (ret < 0) {
		dev_dbg(bq->dev, "Disabling watchdog failed %d\n", ret);
		return ret;
	}

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = bq25890_field_write(bq, init_data[i].id,
					  init_data[i].value);
		if (ret < 0) {
			dev_dbg(bq->dev, "Writing init data failed %d\n", ret);
			return ret;
		}
	}

	/* Configure ADC for continuous conversions when charging */
	ret = bq25890_field_write(bq, F_CONV_RATE, !!bq->state.online);
	if (ret < 0) {
		dev_dbg(bq->dev, "Config ADC failed %d\n", ret);
		return ret;
	}

	ret = bq25890_get_chip_state(bq, &bq->state);
	if (ret < 0) {
		dev_dbg(bq->dev, "Get state failed %d\n", ret);
		return ret;
	}

	return 0;
}

#if 1
static const enum power_supply_property bq25890_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,	
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

#else
static const enum power_supply_property bq25890_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};
#endif



static int bq25890_power_supply_init(struct bq25890_device *bq)
{
	#if 1
	int ret = 0;
	
	struct power_supply *usb_bq25890 = &bq->charger;
	usb_bq25890->name = "bq25890-charger";
	usb_bq25890->type = POWER_SUPPLY_TYPE_USB;
	usb_bq25890->get_property = bq25890_power_supply_get_property;
	usb_bq25890->set_property = NULL;
	usb_bq25890->properties = bq25890_power_supply_props;
	usb_bq25890->num_properties = ARRAY_SIZE(bq25890_power_supply_props);
	ret = power_supply_register(bq->dev,usb_bq25890);
	if(ret)
	{
		printk("=======bq25890_power_supply_init no memory====\n");
		return -ENOMEM;
	}
	return ret;
	#else
	struct power_supply_config psy_cfg = { .drv_data = bq, };

	psy_cfg.supplied_to = bq25890_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25890_charger_supplied_to);

	bq->charger = power_supply_register(bq->dev, &bq25890_power_supply_desc,
					    &psy_cfg);

	return PTR_ERR_OR_ZERO(bq->charger);
	#endif
}


static void bq25890_usb_work(struct work_struct *data)
{
	int ret;
	struct bq25890_device *bq =
			container_of(data, struct bq25890_device, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		/* Enable boost mode */
		ret = bq25890_field_write(bq, F_OTG_CFG, 1);
		if (ret < 0)
			goto error;
		break;

	case USB_EVENT_NONE:
		/* Disable boost mode */
		ret = bq25890_field_write(bq, F_OTG_CFG, 0);
		if (ret < 0)
			goto error;

		power_supply_changed(&bq->charger);
		break;
	}

	return;

error:
	dev_err(bq->dev, "Error switching to boost/charger mode.\n");
}

static int bq25890_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct bq25890_device *bq =
			container_of(nb, struct bq25890_device, usb_nb);

	bq->usb_event = val;
	schedule_work(&bq->usb_work);
	//queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}


static int bq25890_get_chip_version(struct bq25890_device *bq)
{
	int id, rev;

	id = bq25890_field_read(bq, F_PN);
	if (id < 0) {
		dev_err(bq->dev, "Cannot read chip ID.\n");
		return id;
	}

	rev = bq25890_field_read(bq, F_DEV_REV);
	if (rev < 0) {
		dev_err(bq->dev, "Cannot read chip revision.\n");
		return rev;
	}

	switch (id) {
	case BQ25890_ID:
		bq->chip_version = BQ25890;
		break;

	/* BQ25892 and BQ25896 share same ID 0 */
	case BQ25896_ID:
		switch (rev) {
		case 2:
			bq->chip_version = BQ25896;
			break;
		case 1:
			bq->chip_version = BQ25892;
			break;
		default:
			dev_err(bq->dev,
				"Unknown device revision %d, assume BQ25892\n",
				rev);
			bq->chip_version = BQ25892;
		}
		break;

	case BQ25895_ID:
		bq->chip_version = BQ25895;
		break;

	default:
		dev_err(bq->dev, "Unknown chip ID %d\n", id);
		return -ENODEV;
	}

	return 0;
}

static int bq25890_fw_read_u32_props(struct bq25890_device *bq)
{
	//int ret;
	u32 property;
	int i;
	struct bq25890_init_data *init = &bq->init_data;
	struct {
		char *name;
		bool optional;
		enum bq25890_table_ids tbl_id;
		u8 *conv_data; /* holds converted value from given property */
	} props[] = {
		/* required properties */
		{"ti,charge-current", false, TBL_ICHG, &init->ichg},
		{"ti,battery-regulation-voltage", false, TBL_VREG, &init->vreg},
		{"ti,termination-current", false, TBL_ITERM, &init->iterm},
		{"ti,precharge-current", false, TBL_ITERM, &init->iprechg},
		{"ti,minimum-sys-voltage", false, TBL_SYSVMIN, &init->sysvmin},
		{"ti,boost-voltage", false, TBL_BOOSTV, &init->boostv},
		{"ti,boost-max-current", false, TBL_BOOSTI, &init->boosti},

		/* optional properties */
		{"ti,thermal-regulation-threshold", true, TBL_TREG, &init->treg},
		{"ti,ibatcomp-micro-ohms", true, TBL_RBATCOMP, &init->rbatcomp},
		{"ti,ibatcomp-clamp-microvolt", true, TBL_VBATCOMP, &init->vclamp},
	};

	/* initialize data for optional properties */
	init->treg = 3; /* 120 degrees Celsius */
	init->rbatcomp = init->vclamp = 0; /* IBAT compensation disabled */
	#if 1
#ifdef CONFIG_PRODUCT_X1000_ECMINI
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		if(strcmp("ti,charge-current",props[i].name) == 0) //charge current
		{
			property = 	2400000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,battery-regulation-voltage",props[i].name) == 0)
		{
			property = 	4200000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,termination-current",props[i].name) == 0)
		{
			property = 	256000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,precharge-current",props[i].name) == 0)
		{
			property = 	128000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,minimum-sys-voltage",props[i].name) == 0)
		{
			property = 3500000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,boost-voltage",props[i].name) == 0)
		{
			property = 5000000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,boost-max-current",props[i].name) == 0)
		{
			property = 1000000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}//optional
		else if(strcmp("ti,thermal-regulation-threshold",props[i].name) == 0)
		{
			property = 120;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
								
	}
#else
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		if(strcmp("ti,charge-current",props[i].name) == 0) //charge current
		{
			property = 	1800000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,battery-regulation-voltage",props[i].name) == 0)
		{
			property = 	4350000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,termination-current",props[i].name) == 0)
		{
			property = 	128000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,precharge-current",props[i].name) == 0)
		{
			property = 	128000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,minimum-sys-voltage",props[i].name) == 0)
		{
			property = 3500000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,boost-voltage",props[i].name) == 0)
		{
			property = 5000000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
		else if(strcmp("ti,boost-max-current",props[i].name) == 0)
		{
			property = 1000000;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}//optional
		else if(strcmp("ti,thermal-regulation-threshold",props[i].name) == 0)
		{
			property = 120;
			*props[i].conv_data = bq25890_find_idx(property,props[i].tbl_id);
		}
								
	}
#endif
	#else
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = device_property_read_u32(bq->dev, props[i].name,
					       &property);
		if (ret < 0) {
			if (props[i].optional)
				continue;

			dev_err(bq->dev, "Unable to read property %d %s\n", ret,
				props[i].name);

			return ret;
		}

		*props[i].conv_data = bq25890_find_idx(property,
						       props[i].tbl_id);
	}
	#endif

	return 0;
}

static int bq25890_fw_probe(struct bq25890_device *bq)
{
	int ret;
	struct bq25890_init_data *init = &bq->init_data;

	ret = bq25890_fw_read_u32_props(bq);
	if (ret < 0)
		return ret;

#if 1
	init->ilim_en = 1;	
	init->boostf = 0;
#else
	init->ilim_en = device_property_read_bool(bq->dev, "ti,use-ilim-pin");
	init->boostf = device_property_read_bool(bq->dev, "ti,boost-low-freq");
#endif
	return 0;
}



static int bq2589x_charger1_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	printk("========fun[%s]====\n",__FUNCTION__);

	struct device *dev = &client->dev;
	struct bq25890_device *bq;
	int ret;
	int irqn = -1;
	int i;

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;
	gloal_bq25890_1 = bq;

	mutex_init(&bq->lock);
	
	bq->rmap = devm_regmap_init_i2c(client, &bq25890_regmap_config);
	if (IS_ERR(bq->rmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(bq->rmap);
	}	
	

	for (i = 0; i < ARRAY_SIZE(bq25890_reg_fields); i++) {
		const struct reg_field *reg_fields = bq25890_reg_fields;

		bq->rmap_fields[i] = devm_regmap_field_alloc(dev, bq->rmap,
							     reg_fields[i]);
		if (IS_ERR(bq->rmap_fields[i])) {
			dev_err(dev, "cannot allocate regmap field\n");
			return PTR_ERR(bq->rmap_fields[i]);
		}
	}

	i2c_set_clientdata(client, bq);

	ret = bq25890_get_chip_version(bq);
	if (ret) {
		dev_err(dev, "Cannot read chip ID or unknown chip.\n");
		return ret;
	}

	if (!dev->platform_data) {
		ret = bq25890_fw_probe(bq);
		if (ret < 0) {
			dev_err(dev, "Cannot read device properties.\n");
			return ret;
		}
	} else {
		return -ENODEV;
	}

	ret = bq25890_hw_init(bq);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}

	if(bq25890_int_gpio >= 0)
	{
		ret = gpio_request(bq25890_int_gpio, "bq25890_1 irq pin");
		if (ret) {
			dev_err(dev, "%s: %d gpio request failed\n", __func__, bq25890_int_gpio);
			return ret;
		}
		gpio_direction_input(bq25890_int_gpio);
		irqn = gpio_to_irq(bq25890_int_gpio);
	}


	if (irqn < 0) {
		dev_err(dev, "No irq resource found.\n");
		return irqn;
	}


	/* OTG reporting */
	bq->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(bq->usb_phy)) {
		INIT_WORK(&bq->usb_work, bq25890_usb_work);
		bq->usb_nb.notifier_call = bq25890_usb_notifier;
		usb_register_notifier(bq->usb_phy, &bq->usb_nb);
	}

	ret = devm_request_threaded_irq(dev,irqn, NULL,
					bq25890_irq_handler_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					BQ25890_IRQ_PIN, bq);
	if (ret)
		goto irq_fail;

	ret = bq25890_power_supply_init(bq);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		goto irq_fail;
	}

	ret = device_create_file(&client->dev, &dev_attr_bq25890_charge_status);
	if(ret != 0)
	{
		dev_err(&client->dev,  "11Failed to create xxx sysfs files: %d\n",ret);
	}

	
	return 0;

irq_fail:
	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);
	
	return ret;

}

static int bq2589x_charger1_remove(struct i2c_client *client)
{
	struct bq25890_device *bq = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	power_supply_unregister(&bq->charger);

	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);

	/* reset all registers to default values */
	bq25890_chip_reset(bq);
	if(bq25890_int_gpio >= 0 )
	{
		gpio_free(bq25890_int_gpio);
	}
	devm_kfree(dev,bq);
	return 0;
}



static const struct i2c_device_id bq25890_charger1_id[] = {
	{ "bq25890", 0 },
};


static struct i2c_driver bq25890_charger_driver = {
    .driver     = {
        .name   = "bq25890",
		.owner = THIS_MODULE,
    },
   	.probe      = bq2589x_charger1_probe,
    .remove     = bq2589x_charger1_remove,
	.id_table	= bq25890_charger1_id,
};


static int __init bq25890_charge_init(void)
{
	int ret = 0;
	
	ret = i2c_add_driver(&bq25890_charger_driver);
	if (ret) {
		printk(KERN_ERR "bq25890_charge_init: failed to register i2c driver\n");
		return ret;
	}

	return ret;
}

static void __exit bq25890_charge_exit(void)
{
	i2c_del_driver(&bq25890_charger_driver);
}

module_init(bq25890_charge_init);
module_exit(bq25890_charge_exit);


MODULE_DESCRIPTION("charge ic bq25890 driver");
MODULE_LICENSE("GPL v2");






