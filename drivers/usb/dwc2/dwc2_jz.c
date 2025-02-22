#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/jz_dwc.h>
#include <soc/base.h>

#include "core.h"
#include "gadget.h"
#include "dwc2_jz.h"

#define OTG_CLK_NAME		"otg1"
#define VBUS_REG_NAME		"vbus"
#define CGU_USB_CLK_NAME	"cgu_usb"
#define USB_PWR_CLK_NAME	"pwc_usb"
#define DWC2_HOST_ID_TIMER_INTERVAL (HZ / 2)
#define DWC2_HOST_ID_MAX_DOG_COUNT  3
struct dwc2_jz {
    struct platform_device dwc2;
    struct device *dev;
    struct clk *clk;
    struct clk *cgu_clk;
    struct clk *pwr_clk;
    struct mutex irq_lock; /* protect irq op */

    /*device*/
    struct jzdwc_pin *dete_pin; /* Host mode may used this pin to judge extern vbus mode
     * Device mode may used this pin judge B-device insert */
    int dete_irq;
    int pullup_on;
    unsigned long delay_jiffies;
#ifdef CONFIG_PRODUCT_X1000_IDRIVER
    int insert_state;
#endif
    /*host*/
    int id_irq;
    struct jzdwc_pin *id_pin;
    struct wake_lock id_resume_wake_lock;
    struct jzdwc_pin *drvvbus_pin; /*Use drvvbus pin or regulator to set vbus on*/
    struct regulator *vbus;
    atomic_t vbus_on;
    struct work_struct vbus_work;
    void *work_data;
    struct input_dev *input;
};

#ifdef CONFIG_PRODUCT_X1000_IDRIVER
static struct dwc2_jz* global_dwc2_jz;
static int global_dwc2_jz_insert_state = 0;
#endif

#define to_dwc2_jz(dwc) container_of((dwc)->pdev, struct dwc2_jz, dwc2)

#if DWC2_HOST_MODE_ENABLE
int dwc2_jz_input_init(struct dwc2_jz *jz) {
#ifdef CONFIG_USB_DWC2_INPUT_EVDEV
	int ret = 0;
	struct input_dev *input = NULL;
	input = input_allocate_device();
	if (input == NULL)
		return -ENOMEM;

	input->name = "dwc2";
	input->dev.parent = jz->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(KEY_POWER2, input->keybit);

	if ((ret = input_register_device(input)) < 0)
		goto error;

	jz->input = input;
	return 0;
error:
	input_free_device(input);
	return ret;
#else
	jz->input = NULL;
	return 0;
#endif
}

static void dwc2_jz_input_cleanup(struct dwc2_jz *jz)
{
	if (jz->input)
		input_unregister_device(jz->input);
}

static void __dwc2_jz_input_report_key(struct dwc2_jz *jz,
					unsigned int code, int value)
{
	if (jz->input) {
		input_report_key(jz->input, code, value);
		input_sync(jz->input);
	}
}

static void __dwc2_input_report_power2_key(struct dwc2_jz *jz) {
	if (jz->input) {
		__dwc2_jz_input_report_key(jz, KEY_POWER2, 1);
		msleep(50);
		__dwc2_jz_input_report_key(jz, KEY_POWER2, 0);
	}
}
#endif

int dwc2_clk_is_enabled(struct dwc2 *dwc) {
	printk("dwc2_clk_is_enabled 0\n");
	
	struct dwc2_jz *jz = to_dwc2_jz(dwc);
	printk("dwc2_clk_is_enabled 0.1\n");
	if (IS_ERR(jz->clk))
	{
		printk("dwc2_clk_is_enabled 1\n");
		return 1;
	}
	printk("dwc2_clk_is_enabled 2\n");
	return clk_is_enabled(jz->clk);
}

EXPORT_SYMBOL(dwc2_clk_is_enabled);

void dwc2_clk_enable(struct dwc2 *dwc) {
	struct dwc2_jz *jz = to_dwc2_jz(dwc);

	if (!dwc2_clk_is_enabled(dwc) && !IS_ERR(jz->clk)) {
		if (jz->pwr_clk)
			clk_enable(jz->pwr_clk);
		clk_enable(jz->cgu_clk);
		clk_enable(jz->clk);
	}
}

void dwc2_clk_disable(struct dwc2 *dwc) {
	struct dwc2_jz *jz = to_dwc2_jz(dwc);

	if (dwc2_clk_is_enabled(dwc) && !IS_ERR(jz->clk)) {
		clk_disable(jz->clk);
		clk_disable(jz->cgu_clk);
		if (jz->pwr_clk)
			clk_disable(jz->pwr_clk);
	}
}

/*usb insert detect*/
struct jzdwc_pin __attribute__((weak)) dwc2_dete_pin = {
	.num = -1,//GPIO_PA(17),
	.enable_level = 1,
};

/*usb host plug insert detect*/
struct jzdwc_pin __attribute__((weak)) dwc2_id_pin = {
    .num = -1,//GPIO_PC(23),
	.enable_level = 0,
};

/*usb drvvbus pin*/
struct jzdwc_pin __attribute__((weak)) dwc2_drvvbus_pin = {
	.num = -1,//GPIO_PB(25),
	.enable_level = 1,
};

static int __dwc2_get_detect_pin_status(struct dwc2_jz *jz) {
	int insert = 0;
	if (gpio_is_valid(jz->dete_pin->num)) {
		insert = gpio_get_value(jz->dete_pin->num);
		if (jz->dete_pin->enable_level == LOW_ENABLE)
			return !insert;
	}
	return insert;
}

int dwc2_get_detect_pin_status(struct dwc2 *dwc)
{
	struct dwc2_jz *jz = to_dwc2_jz(dwc);
	return __dwc2_get_detect_pin_status(jz);
}
EXPORT_SYMBOL_GPL(dwc2_get_detect_pin_status);

static int __dwc2_get_id_level(struct dwc2_jz* jz) {
	int id_level = 1;
	if (gpio_is_valid(jz->id_pin->num)) {
		id_level = gpio_get_value(jz->id_pin->num);
		if (jz->id_pin->enable_level == HIGH_ENABLE)
			id_level = !id_level;
	}
	return id_level;
}
int dwc2_get_id_level(struct dwc2 *dwc)
{
	struct dwc2_jz *jz = to_dwc2_jz(dwc);
	return __dwc2_get_id_level(jz);
}
EXPORT_SYMBOL_GPL(dwc2_get_id_level);

void dwc2_gpio_irq_mutex_lock(struct dwc2 *dwc)
{
	struct dwc2_jz *jz = to_dwc2_jz(dwc);
	mutex_lock(&jz->irq_lock);
}
EXPORT_SYMBOL_GPL(dwc2_gpio_irq_mutex_lock);

void dwc2_gpio_irq_mutex_unlock(struct dwc2 *dwc)
{
	struct dwc2_jz *jz = to_dwc2_jz(dwc);
	mutex_unlock(&jz->irq_lock);
}
EXPORT_SYMBOL_GPL(dwc2_gpio_irq_mutex_unlock);

#if DWC2_DEVICE_MODE_ENABLE
extern void dwc2_gadget_plug_change(int plugin);
static void usb_plug_change(struct dwc2_jz *jz) {

	int insert = __dwc2_get_detect_pin_status(jz);
	struct dwc2 *dwc = platform_get_drvdata(&jz->dwc2);
	pr_info("DWC USB %s\n", insert ? "connect" : "disconnect");
	dwc2_disable_global_interrupts(dwc);
	synchronize_irq(dwc->irq);
	flush_work(&dwc->otg_id_work);
	dwc2_gadget_plug_change(insert);
	if (!jz_otg_phy_is_suspend())
		dwc2_enable_global_interrupts(dwc);
}

static irqreturn_t usb_detect_interrupt(int irq, void *dev_id)
{
	struct dwc2_jz	*jz = (struct dwc2_jz *)dev_id;

	mutex_lock(&jz->irq_lock);
	msleep(100);
	usb_plug_change(jz);
	mutex_unlock(&jz->irq_lock);
	return IRQ_HANDLED;
}
#ifdef CONFIG_PRODUCT_X1000_IDRIVER
void usb_detect_interrupt_for_usb(int insert)
{
	struct dwc2_jz *jz = global_dwc2_jz;
    if( !jz )
    {
        global_dwc2_jz_insert_state = insert;
        return ;
    }
    if( jz->dete_pin->num != USB_DETECT_BY_PMU )
    {
        global_dwc2_jz_insert_state = insert;
        return ;
    }
    
    struct dwc2 *dwc = platform_get_drvdata(&jz->dwc2);
    if( insert != global_dwc2_jz_insert_state )
    {
        mutex_lock(&jz->irq_lock);
        msleep(100);

        printk("DWC USB %s\n", insert ? "connect" : "disconnect");
        dwc2_disable_global_interrupts(dwc);
        flush_work(&dwc->otg_id_work);
        dwc2_gadget_plug_change(insert);
        if (!jz_otg_phy_is_suspend())
            dwc2_enable_global_interrupts(dwc);    
        mutex_unlock(&jz->irq_lock);
    }
    global_dwc2_jz_insert_state = insert;
	return ;
}EXPORT_SYMBOL_GPL(usb_detect_interrupt_for_usb);
#endif
#endif /* !DWC2_DEVICE_MODE_ENABLE */

#if DWC2_HOST_MODE_ENABLE
static irqreturn_t usb_host_id_interrupt(int irq, void *dev_id) {
	struct dwc2_jz	*jz = (struct dwc2_jz *)dev_id;
	struct dwc2 *dwc = platform_get_drvdata(&jz->dwc2);
	int is_first = 1, dither_count = DWC2_HOST_ID_MAX_DOG_COUNT;

	mutex_lock(&jz->irq_lock);
	wake_lock(&jz->id_resume_wake_lock);
	/* 50ms dither filter */
	msleep(50);
	while (1) {
		if (__dwc2_get_id_level(jz) == 0) { /* host */
			/* Think about vbus with an big capacity, when we disconnect B-device, the vbus
			 * slow down to 0v, Cause the detect pin is still active, B-device not disconnect from
			 * system in time, At thie time, the A-host is connected, we resume controller,
			 * But after a while the device disconnected, suppend controller , shit happend
			 */
			if (__dwc2_get_detect_pin_status(jz))
				dither_count++;
			else {
				printk("host detect\n");
				dwc2_resume_controller(dwc);
				if (is_first) /*slcao's version just report power2_key in the first time,
						it's a bug or not, we just keep it and take care of it*/
					__dwc2_input_report_power2_key(jz);
				jz_otg_sft_id_off();
				break;
			}
		}
		/*keep filter*/
		is_first = 0;
		if (!dither_count)
			break;
		dither_count--;
		schedule_timeout_uninterruptible(DWC2_HOST_ID_TIMER_INTERVAL + 1);
	}
	mutex_unlock(&jz->irq_lock);
	wake_lock_timeout(&jz->id_resume_wake_lock, 3 * HZ);
	return IRQ_HANDLED;
}

static int __dwc2_get_drvvbus_level(struct dwc2_jz *jz)
{
	int drvvbus = 0;
	if (gpio_is_valid(jz->drvvbus_pin->num)) {
		drvvbus = gpio_get_value(jz->drvvbus_pin->num);
		if (jz->drvvbus_pin->enable_level == LOW_ENABLE)
			drvvbus = !drvvbus;
	}
	return drvvbus;
}

static void dwc2_vbus_work(struct work_struct *work)
{
	struct dwc2_jz* jz = container_of(work, struct dwc2_jz, vbus_work);
	struct dwc2 *dwc = (struct dwc2 *)jz->work_data;
	int old_is_on = atomic_read(&jz->vbus_on);
	int is_on = 0, ret = 0;

	if (IS_ERR_OR_NULL(jz->vbus) && !gpio_is_valid(jz->drvvbus_pin->num))
		return;

	is_on = old_is_on ? dwc2_is_host_mode(dwc) : 0;

	dev_info(jz->dev, "set vbus %s(%s) for %s mode\n",
			is_on ? "on" : "off",
			old_is_on ? "on" : "off",
			dwc2_is_host_mode(dwc) ? "host" : "device");

	if (!IS_ERR_OR_NULL(jz->vbus)) {
		if (is_on && !regulator_is_enabled(jz->vbus))
			ret = regulator_enable(jz->vbus);
		else if (!is_on && regulator_is_enabled(jz->vbus))
			ret = regulator_disable(jz->vbus);
		WARN(ret != 0, "dwc2 usb host ,regulator can not be used\n");
	} else {
		if (is_on && !__dwc2_get_drvvbus_level(jz))
			gpio_direction_output(jz->drvvbus_pin->num,
					jz->drvvbus_pin->enable_level == HIGH_ENABLE);
		else if (!is_on && __dwc2_get_drvvbus_level(jz))
			gpio_direction_output(jz->drvvbus_pin->num,
					jz->drvvbus_pin->enable_level == LOW_ENABLE);
	}
	return;
}

void jz_set_vbus(struct dwc2 *dwc, int is_on)
{
	struct dwc2_jz* jz = (struct dwc2_jz *)to_dwc2_jz(dwc);

	/*CHECK it lost some vbus set is ok ??*/
	atomic_set(&jz->vbus_on, !!is_on);
	jz->work_data = (void *)dwc;
	if (!work_pending(&jz->vbus_work)) {
		schedule_work(&jz->vbus_work);
		if (!in_atomic())
			flush_work(&jz->vbus_work);
	}
}

static ssize_t jz_vbus_show(struct device *dev,
		struct device_attribute *attr,
		char *buf) {
	struct dwc2_jz *jz = dev_get_drvdata(dev);
	struct dwc2 *dwc = platform_get_drvdata(&jz->dwc2);
	int vbus_is_on = 0;

	if (!IS_ERR_OR_NULL(jz->vbus)) {
		if (regulator_is_enabled(jz->vbus))
			vbus_is_on = 1;
		else
			vbus_is_on = 0;
	} else if (gpio_is_valid(jz->drvvbus_pin->num)) {
		vbus_is_on = __dwc2_get_drvvbus_level(jz);
	} else {
		vbus_is_on = dwc2_is_host_mode(dwc);
	}

	return sprintf(buf, "%s\n", vbus_is_on ? "on" : "off");
}

static ssize_t jz_vbus_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct dwc2_jz *jz = dev_get_drvdata(dev);
	struct dwc2 *dwc = platform_get_drvdata(&jz->dwc2);
	int is_on = 0;

	if (strncmp(buf, "on", 2) == 0)
		is_on = 1;
	jz_set_vbus(dwc, is_on);

	return count;
}

static DEVICE_ATTR(vbus, S_IWUSR | S_IRUSR,
		jz_vbus_show, jz_vbus_set);

static ssize_t jz_id_show(struct device *dev,
		struct device_attribute *attr,
		char *buf) {
	struct dwc2_jz *jz = dev_get_drvdata(dev);
	int id_level = __dwc2_get_id_level(jz);

	return sprintf(buf, "%s\n", id_level ? "1" : "0");
}

static DEVICE_ATTR(id, S_IRUSR |  S_IRGRP | S_IROTH,
		jz_id_show, NULL);
#else	/* DWC2_HOST_MODE_ENABLE */
void jz_set_vbus(struct dwc2 *dwc, int is_on) {}
#endif	/* !DWC2_HOST_MODE_ENABLE */
//EXPORT_SYMBOL(jz_set_vbus);

int dwc2_suspend_controller(struct dwc2 *dwc)
{
	if (dwc->suspended || (!dwc->keep_phy_on)) {
#ifndef CONFIG_USB_DWC2_SAVING_POWER
		pr_info("Suspend otg by suspend phy\n");
		dwc2_disable_global_interrupts(dwc);
		jz_otg_phy_suspend(1);
#else
		if (dwc->suspended ||
				(dwc2_is_host_mode(dwc)) ||
				(dwc2_is_device_mode(dwc) && !dwc2_has_ep_enabled(dwc))) {
			pr_info("Suspend otg by shutdown dwc cotroller and phy\n");
			dwc->phy_inited = 0;
			dwc2_disable_global_interrupts(dwc);
			jz_otg_phy_suspend(1);
			jz_otg_phy_powerdown();
			dwc2_clk_disable(dwc);
		}
#endif
	}
	return 0;
}
EXPORT_SYMBOL_GPL(dwc2_suspend_controller);

int dwc2_resume_controller(struct dwc2* dwc)
{
	if (!dwc2_clk_is_enabled(dwc)) {
		pr_info("Resume otg by reinit dwc controller and phy\n");
		dwc2_clk_enable(dwc);
		jz_otg_ctr_reset();
		jz_otg_phy_init(dwc2_usb_mode());
		jz_otg_phy_suspend(0);
		dwc2_core_init(dwc);
#if DWC2_DEVICE_MODE_ENABLE
		if (dwc2_is_device_mode(dwc))
			dwc2_device_mode_init(dwc);
#endif
		dwc2_enable_global_interrupts(dwc);
	}
#ifndef CONFIG_USB_DWC2_SAVING_POWER
	else if (jz_otg_phy_is_suspend()) {
		pr_info("Resume otg by resume phy\n");
		jz_otg_phy_suspend(0);
		dwc2_enable_global_interrupts(dwc);
	}
#endif
	return 0;
}
//EXPORT_SYMBOL_GPL(dwc2_resume_controller);

extern unsigned short pc_link;
static ssize_t
pc_link_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf,"%hu",pc_link);
}

static ssize_t
pc_link_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t n)
{
        sscanf(buf, "%hu", &pc_link) ;
        return n;
}

static DEVICE_ATTR(pc_link, 0644, pc_link_show, pc_link_store);


static struct attribute *dwc2_jz_attributes[] = {
#if DWC2_HOST_MODE_ENABLE
	&dev_attr_vbus.attr,
	&dev_attr_id.attr,
#endif	/*DWC2_HOST_MODE_ENABLE*/
	&dev_attr_pc_link.attr,
	NULL
};

static const struct attribute_group dwc2_jz_attr_group = {
	.attrs = dwc2_jz_attributes,
};


static void platform_devicd_dw2_jz_release(struct device * dev)
{

	return ;
}

static u64 dwc2_jz_dma_mask = DMA_BIT_MASK(32);

struct dwc2_platform_data	*dwc2_plat_data;
struct dwc2_jz	*jz;
static int dwc2_jz_probe(struct platform_device *pdev) {
	struct platform_device		*dwc2;
	//struct dwc2_jz		*jz;
	//struct dwc2_platform_data	*dwc2_plat_data;
	int				 ret = -ENOMEM;
	
	printk("func:%s\n", __func__);
	jz = devm_kzalloc(&pdev->dev, sizeof(*jz), GFP_KERNEL);
	if (!jz) {
		dev_err(&pdev->dev, "not enough memory\n");
		return -ENOMEM;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &dwc2_jz_dma_mask;

	platform_set_drvdata(pdev, jz);
	dwc2_plat_data = devm_kzalloc(&pdev->dev,
			sizeof(struct dwc2_platform_data), GFP_KERNEL);
	if (!dwc2_plat_data)
		return -ENOMEM;
	dwc2 = &jz->dwc2;
	dwc2->name = "dwc2";
	dwc2->id = -1;
	device_initialize(&dwc2->dev);
	dma_set_coherent_mask(&dwc2->dev, pdev->dev.coherent_dma_mask);
	dwc2->dev.parent = &pdev->dev;
	dwc2->dev.dma_mask = pdev->dev.dma_mask;
	dwc2->dev.dma_parms = pdev->dev.dma_parms;
	dwc2->dev.platform_data = dwc2_plat_data;
	dwc2->dev.release =	platform_devicd_dw2_jz_release;

	jz->dev	= &pdev->dev;
	mutex_init(&jz->irq_lock);

	jz->clk = devm_clk_get(&pdev->dev, OTG_CLK_NAME);
	if (IS_ERR(jz->clk)) {
		dev_err(&pdev->dev, "clk gate get error\n");
		return PTR_ERR(jz->clk);
	}
	jz->cgu_clk = devm_clk_get(&pdev->dev, CGU_USB_CLK_NAME);
	if (IS_ERR(jz->cgu_clk)) {
		dev_err(&pdev->dev, "cgu clk gate get error\n");
		return PTR_ERR(jz->cgu_clk);
	}
	jz->pwr_clk = devm_clk_get(&pdev->dev, USB_PWR_CLK_NAME);
	if (IS_ERR(jz->pwr_clk))
		jz->pwr_clk = NULL;

	clk_set_rate(jz->cgu_clk, 24000000);
	clk_enable(jz->cgu_clk);
	clk_enable(jz->clk);
	if (jz->pwr_clk)
		clk_enable(jz->pwr_clk);

	jz->dete_pin = &dwc2_dete_pin;
	jz->drvvbus_pin = &dwc2_drvvbus_pin;
	jz->id_pin = &dwc2_id_pin;

	/*pull enable*/
	if (jz->dete_pin->num >= 0)
		ret = devm_gpio_request_one(&pdev->dev, jz->dete_pin->num,
				GPIOF_DIR_IN, "usb-insert-detect");
	jz->dete_irq= -1;
#if DWC2_DEVICE_MODE_ENABLE
	if (ret == 0) {
		jz->dete_irq = gpio_to_irq(jz->dete_pin->num);
	}
#ifdef CONFIG_PRODUCT_X1000_IDRIVER
	else if( jz->dete_pin->num == USB_DETECT_BY_PMU ){
        //?
    }
#endif 
	else {
#ifdef CONFIG_USB_OTG_BOND_TO_CHARGER
		dwc2_plat_data->keep_phy_on = 0;
#else
		dwc2_plat_data->keep_phy_on = 1;
#endif
	}
#endif	/* DWC2_DEVICE_MODE_ENABLE */

	jz->id_irq = -1;
#if DWC2_HOST_MODE_ENABLE
	jz->vbus = regulator_get(NULL, VBUS_REG_NAME);
	if (IS_ERR_OR_NULL(jz->vbus)) {
		if(jz->drvvbus_pin->num > 0) {
			ret = devm_gpio_request_one(&pdev->dev, jz->drvvbus_pin->num,
					GPIOF_DIR_OUT, "drvvbus_pin");

			if (ret < 0) jz->drvvbus_pin->num = -1;
		}
		dev_warn(&pdev->dev, "regulator %s get error\n", VBUS_REG_NAME);
	} else {
		jz->drvvbus_pin->num = -1;
	}
	INIT_WORK(&jz->vbus_work, dwc2_vbus_work);
	atomic_set(&jz->vbus_on, 0);

	if(jz->id_pin->num > 0) {
		ret = devm_gpio_request_one(&pdev->dev, jz->id_pin->num,
				GPIOF_DIR_IN, "otg-id-detect");
		if (ret == 0) {
			jz->id_irq = gpio_to_irq(jz->id_pin->num);
			wake_lock_init(&jz->id_resume_wake_lock, WAKE_LOCK_SUSPEND, "otgid-resume");
		} else {
			dwc2_plat_data->keep_phy_on = 1;
		}
	} else {
		dwc2_plat_data->keep_phy_on = 1;
	}

	dwc2_jz_input_init(jz);
#endif	/* DWC2_HOST_MODE_ENABLE */

	jz_otg_ctr_reset();
	jz_otg_phy_init(dwc2_usb_mode());
	if (!dwc2_plat_data->keep_phy_on)
		jz_otg_sft_id(1);
	else {
		jz_otg_sft_id_off();
	}

	/*
	 * Close VBUS detect in DWC-OTG PHY.	WHY???
	 */
	//*(unsigned int*)0xb3500000 |= 0xc;

	ret = platform_device_add_resources(dwc2, pdev->resource,
					pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "couldn't add resources to dwc2 device\n");
		goto fail_register_dwc2_dev;
	}

	ret = platform_device_add(dwc2);
	//ret = platform_device_register(dwc2);
	if (ret) {
		dev_err(&pdev->dev, "failed to register dwc2 device\n");
		goto fail_register_dwc2_dev;
	}

#if (defined DWC2_DEVICE_MODE_ENABLE) && (!defined(CONFIG_USB_OTG_BOND_TO_CHARGER))
	if (dwc2_plat_data->keep_phy_on) {
		dwc2_gadget_plug_change(1);
	} 
#ifdef CONFIG_PRODUCT_X1000_IDRIVER
    else if( jz->dete_pin->num == USB_DETECT_BY_PMU ) {
        //
        if( global_dwc2_jz_insert_state )
        {
			mutex_lock(&jz->irq_lock);
			dwc2_gadget_plug_change(1);
			mutex_unlock(&jz->irq_lock);            
        }
    }
#endif
    else {
		ret = devm_request_threaded_irq(&pdev->dev,
				gpio_to_irq(jz->dete_pin->num),
				NULL,
				usb_detect_interrupt,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				"usb-detect", jz);
		if (ret) {
			dev_err(&pdev->dev, "request usb-detect fail\n");
			goto fail_reuqest_irq;
		}
		
		
		if (__dwc2_get_detect_pin_status(jz)) {
			mutex_lock(&jz->irq_lock);
			dwc2_gadget_plug_change(1);
			mutex_unlock(&jz->irq_lock);
		}
	}
#endif	/* DWC2_DEVICE_MODE_ENABLE */

#if DWC2_HOST_MODE_ENABLE
	if (dwc2_plat_data->keep_phy_on) {
		struct dwc2* dwc = platform_get_drvdata(dwc2);
		printk("if (dwc2_plat_data->keep_phy_on) 111\n");
		dwc2_resume_controller(dwc);
	} else {
		unsigned long flags = 0;
		if (jz->id_pin->enable_level == HIGH_ENABLE)
			flags = IRQF_TRIGGER_RISING;
		else
			flags = IRQF_TRIGGER_FALLING;
		ret = devm_request_threaded_irq(&pdev->dev,
				gpio_to_irq(jz->id_pin->num),
				NULL,
				usb_host_id_interrupt,
				flags | IRQF_ONESHOT,
				"usb-host-id", jz);
		if (ret) {
			dev_err(&pdev->dev, "request host id interrupt fail!\n");
			goto fail_reuqest_irq;
		} else if (!__dwc2_get_id_level(jz)) {
			struct dwc2* dwc = platform_get_drvdata(dwc2);
			printk("if (dwc2_plat_data->keep_phy_on) 222\n");
			dwc2_resume_controller(dwc);
			jz_otg_sft_id_off();
		}
	}
#endif	/* DWC2_HOST_MODE_ENABLE */

	ret = sysfs_create_group(&pdev->dev.kobj, &dwc2_jz_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs group\n");
	}
#ifdef CONFIG_PRODUCT_X1000_IDRIVER
    global_dwc2_jz = jz;
#endif
	
	
	return 0;

fail_reuqest_irq:
	printk("platform_device_unregister(&jz->dwc2) \n");
	platform_device_unregister(&jz->dwc2);
fail_register_dwc2_dev:
	platform_set_drvdata(pdev, NULL);
	if (!dwc2_plat_data->keep_phy_on)
		wake_lock_destroy(&jz->id_resume_wake_lock);
	if (!IS_ERR_OR_NULL(jz->vbus))
		regulator_put(jz->vbus);
	clk_disable(jz->clk);
	clk_disable(jz->cgu_clk);
	if (jz->pwr_clk)
		clk_disable(jz->pwr_clk);
	return ret;
}

static int dwc2_jz_remove(struct platform_device *pdev) {
//	struct dwc2_jz	*jz = platform_get_drvdata(pdev);



	if (jz->dete_irq >= 0)
		gpio_free(jz->dete_pin->num);
#if DWC2_HOST_MODE_ENABLE
	dwc2_jz_input_cleanup(jz);
#endif
	
	if (jz->id_irq >= 0)
		gpio_free(jz->id_pin->num);
	if (!IS_ERR_OR_NULL(jz->vbus))
		regulator_put(jz->vbus);
#if DWC2_HOST_MODE_ENABLE
	if (!dwc2_plat_data->keep_phy_on)
		wake_lock_destroy(&jz->id_resume_wake_lock);
#endif
	printk("func:%s 1\n", __func__);	
	jz_otg_sft_id_off();

	if(jz->clk)	clk_disable(jz->clk);
	if(jz->cgu_clk)	clk_disable(jz->cgu_clk);
	if(jz->pwr_clk)	clk_disable(jz->pwr_clk);
	jz_otg_phy_powerdown();

	//end		
	devm_kfree(&pdev->dev, dwc2_plat_data);
	platform_device_unregister(&jz->dwc2);			
	//devm_kfree(&pdev->dev, jz);	
	//platform_set_drvdata(pdev, NULL);
printk("func:%s 2\n", __func__);
#ifndef CONFIG_USB_JZ_DWC2_MODULE
	platform_device_unregister(&jz->dwc2);
#endif
sysfs_remove_group(&pdev->dev.kobj, &dwc2_jz_attr_group);
	return 0;
}

static int dwc2_jz_suspend(struct platform_device *pdev, pm_message_t state) {
	struct dwc2_jz	*jz = platform_get_drvdata(pdev);
#if DWC2_DEVICE_MODE_ENABLE
	if (jz->dete_irq >= 0)
		enable_irq_wake(jz->dete_irq);
#endif

#if DWC2_HOST_MODE_ENABLE
	if (jz->id_irq >= 0)
		enable_irq_wake(jz->id_irq);
#endif

	return 0;
}

static int dwc2_jz_resume(struct platform_device *pdev) {
	struct dwc2_jz	*jz = platform_get_drvdata(pdev);

#if DWC2_DEVICE_MODE_ENABLE
	if (jz->dete_irq >= 0)
		disable_irq_wake(jz->dete_irq);
#endif

#if DWC2_HOST_MODE_ENABLE
	if (jz->id_irq >= 0)
		disable_irq_wake(jz->id_irq);
#endif
	return 0;
}

struct platform_driver dwc2_jz_driver = {
	.probe		= dwc2_jz_probe,
	.remove		= dwc2_jz_remove,
	.suspend	= dwc2_jz_suspend,
	.resume		= dwc2_jz_resume,
	.driver		= {
		.name	= "jz-dwc2",
		.owner =  THIS_MODULE,
	},
};


#if 0
static int __init dwc2_jz_init(void)
{
	return platform_driver_register(&dwc2_jz_driver);
}

static void __exit dwc2_jz_exit(void)
{
	platform_driver_unregister(&dwc2_jz_driver);
}

/* make us init after usbcore and i2c (transceivers, regulators, etc)
 * and before usb gadget and host-side drivers start to register
 */
fs_initcall(dwc2_jz_init);
module_exit(dwc2_jz_exit);
MODULE_ALIAS("platform:jz-dwc2");
MODULE_AUTHOR("Lutts Cao <slcao@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB2.0 JZ Glue Layer");
#endif