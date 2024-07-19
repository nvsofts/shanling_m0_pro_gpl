/*
 * audio.c -- Audio gadget driver
 *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/utsname.h>

#include "u_audio.h"

#ifdef CONFIG_PRODUCT_X1000_M5S
#define DRIVER_DESC		"shanling M5s"
#elif defined(CONFIG_PRODUCT_X1000_M2X)
#define DRIVER_DESC		"shanling M2x"
#elif defined(CONFIG_PRODUCT_X1000_Q1)
#define DRIVER_DESC		"shanling Q1"
#elif defined(CONFIG_PRODUCT_X1000_ECMINI)
#define DRIVER_DESC		"Shanling EC Mini"
#else
#define DRIVER_DESC		"shanling M0"
#endif
#define DRIVER_VERSION		"Dec 18, 2008"

/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "u_audio.c"
#include "f_audio.c"

#include <linux/kfifo.h>
#include <linux/proc_fs.h>


/*-------------------------------------------------------------------------*/

/* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

/* Thanks to Linux Foundation for donating this product ID. */

#define AUDIO_VENDOR_NUM		0x20B1	/* Linux Foundation */
#define AUDIO_PRODUCT_NUM		0x301F	/* Linux-USB Audio Gadget */
/*-------------------------------------------------------------------------*/
//************cyadd*************************
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string device_str[] = {
	[STRING_MANUFACTURER_IDX].s = "ingenic",
	[STRING_PRODUCT_IDX].s = "audio",
	[STRING_SERIAL_IDX].s = "your_serial_num",
	{  }			/* end of list */
};

static struct usb_gadget_strings device_stringtab = {
	.language	= 0x0409,	/* en-us */
	.strings	= device_str,
};

static struct usb_gadget_strings *device_strings[] = {
	&device_stringtab,
	NULL,
};
//******************************************


static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16(0x200),
	.bDeviceClass =		USB_CLASS_MISC,
	.bDeviceSubClass =	0x02,
	.bDeviceProtocol =	0x01,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id defaults change according to what configs
	 * we support.  (As does bNumConfigurations.)  These values can
	 * also be overridden by module parameters.
	 */
	.idVendor =		__constant_cpu_to_le16(AUDIO_VENDOR_NUM),
	.idProduct =		__constant_cpu_to_le16(AUDIO_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

/*-------------------------------------------------------------------------*/

static int __init audio_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}


	return audio_bind_config(c);
}

static struct usb_configuration audio_config_driver = {
	.label			= DRIVER_DESC,
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/
static char manufacturer[64];

static int __init audio_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	int			status;

	gcnum = usb_gadget_controller_number(cdev->gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0300 | gcnum);
	else {
		ERROR(cdev, "controller '%s' not recognized; trying %s\n",
			cdev->gadget->name,
			audio_config_driver.label);
		device_desc.bcdDevice =
			__constant_cpu_to_le16(0x0300 | 0x0099);
	}

	/* device descriptor strings: manufacturer, product */
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		cdev->gadget->name);
	//************cyadd*************************
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	device_str[STRING_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	device_str[STRING_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;
	/*
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	device_str[STRING_SERIAL_IDX].id = status;
	device_desc.iSerialNumber = status;
	*/
	//***********
	status = usb_add_config(cdev, &audio_config_driver, audio_do_config);
	if (status < 0)
		goto fail;

	INFO(cdev, "%s, version: %s\n", DRIVER_DESC, DRIVER_VERSION);
	return 0;

fail:
	return status;
}

static int __exit audio_unbind(struct usb_composite_dev *cdev)
{
	gaudio_cleanup();
	return 0;
}

static struct usb_composite_driver audio_driver = {
	.name		= "g_audio",
	.dev		= &device_desc,
	.strings	= device_strings,
	.unbind		= __exit_p(audio_unbind),
};


#define USBDAC_SIZE 64*1024
struct kfifo fifo; 
int usbdac_output = 0; //0 native  1 bt
static DEFINE_MUTEX(read_lock);


static ssize_t fifo_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;
	if (mutex_lock_interruptible(&read_lock))
		return -ERESTARTSYS;
	ret = kfifo_to_user(&fifo, buf, count, &copied);;
	mutex_unlock(&read_lock);
	return ret ? ret : copied;
}

long fifo_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	long	val = 0;
	switch(cmd)
	{
		case 1:
	 		ret = get_user(val, (long *) arg);
	 		if(val == 0 || val == 1)
	 		{	
	 			usbdac_output = val;
	 		}
	 		printk("=======fifo_ioctl[%d]===\n",usbdac_output);
		break;
	}
	return 0;
}

int fifo_write(void* buf,size_t count)
{
	if(kfifo_avail(&fifo) >= count) //如果fifo还有空间
	{
		kfifo_in(&fifo,buf,count);
	}
	return 1;
}

static const struct file_operations fifo_fops = {
	.owner = THIS_MODULE,
	.read  = fifo_read,
	.unlocked_ioctl =  fifo_ioctl,
};


static int __init init(void)
{  
	int res = kfifo_alloc(&fifo, USBDAC_SIZE, GFP_KERNEL);
	if(res)
	{
		printk("====error kfifo_alloc=====\n");
		return -ENOMEM;
	}
	if(proc_create("usbdac-fifo", 0, NULL, &fifo_fops) == NULL)
	{
		kfifo_free(&fifo);
		return -ENOMEM;
	}
	//*****************************

	return usb_composite_probe(&audio_driver, audio_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&audio_driver);
	kfifo_free(&fifo);
	remove_proc_entry("usbdac-fifo",NULL);
	
}
module_exit(cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Bryan Wu <cooloney@kernel.org>");
MODULE_LICENSE("GPL");

