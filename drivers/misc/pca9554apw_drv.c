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


static struct i2c_client *pca9554_i2c_dev = NULL;

#define CMD_PORT_IN		 	0x00
#define CMD_PORT_OUT	 		0x01
#define CMD_INVERSION	 		0x02
#define CMD_CONFIG	 			0x03


int pca9554_i2c_write(unsigned char reg,unsigned char value)
{
	int ret = -1;
	int len = 1;
	unsigned char buf[4] = {0};
	if(pca9554_i2c_dev == NULL)	
	{
		printk("=====pca9554_i2c_dev is null====\n");
		return 0;
	}
	buf[0] = reg;
	buf[1] = value;
	//printk("==pca9554_i2c_write==address[%X]===reg[%X]==value[%X]===\n",pca9554_i2c_dev->addr,reg,value);
	ret = i2c_master_send(pca9554_i2c_dev, buf, len+1);
	if (ret < len+1)
	{
		printk("%s 0x%02x err %d!\n", __func__, reg, ret);
	}
	return ret < len+1 ? ret : 0;
}

static unsigned char pca9554_conf_output()
{	
	//all output
	unsigned char value = 0x00;	
	pca9554_i2c_write(CMD_CONFIG,value);
	return 0;
	
}


unsigned char pca9554_set_outputport(unsigned char value)
{
	//printk("==1=pca9554_set_port==value[%X]====\n",value);
	pca9554_i2c_write(CMD_PORT_OUT,value);
	return 0;
}


static ssize_t ledpca9554_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	return sprintf(ubuf,"===\n");
}
static ssize_t ledpca9554_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	int value;
	int ret;
	ret = kstrtoint(ubuf,10,&value);
	if(ret < 0)
	{
		printk("===kstrtoint error==\n");
		return ret;
	}
	if(value >= 0 && value <= 7)
	{
		pca9554_set_outputport(~(1<<value));
	}
	else
	{
		pca9554_set_outputport(0x00);
	}
	return count;
}


static DEVICE_ATTR(ledpca9554, 0644, ledpca9554_show, ledpca9554_store);


static int pca9554_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	int ret = 0;
	printk("=========fun[%s]====\n",__FUNCTION__);
	pca9554_i2c_dev = i2c;
	pca9554_conf_output();
	pca9554_set_outputport(0xBF); //power led
	ret = device_create_file(&i2c->dev, &dev_attr_ledpca9554);
	if(ret != 0)
	{
		dev_err(&i2c->dev,  "Failed to create xxx sysfs files: %d\n",ret);
	}
	return ret;
}

static int  pca9554_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id pca9554_i2c_id[] = {
    { "pca9554", 0 },
};


static struct i2c_driver pca9554_i2c_driver = {
    .driver = {
        .name = "pca9554",
    },
    .probe = pca9554_i2c_probe,
    .remove = pca9554_i2c_remove,
    .id_table = pca9554_i2c_id,
};



static int __init pca9554_drv_init(void)
{
	int ret =0;
	
	ret = i2c_add_driver(&pca9554_i2c_driver);
    if (ret)
	{
        printk(KERN_ERR "pca9554: failed to register i2c driver\n");
        return ret;
    }
	return ret;
}

static void __exit pca9554_drv_exit(void)
{
    i2c_del_driver(&pca9554_i2c_driver);
}

module_init(pca9554_drv_init);
module_exit(pca9554_drv_exit);

MODULE_DESCRIPTION("pca9554 driver");
MODULE_LICENSE("GPL v2");

