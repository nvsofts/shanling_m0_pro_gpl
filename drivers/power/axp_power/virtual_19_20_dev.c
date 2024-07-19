#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
//#include <mach/irqs.h>
#include <linux/power_supply.h>
#include <linux/module.h>

#include "axp-cfg.h"
#include "axp-mfd.h"

static struct platform_device virt[]={
	{
			.name = "reg-19-20-ldo1",
			.id = -1,
			.dev		= {
				.platform_data = "ldo1",
			}
 	},{
			.name = "reg-19-20-ldo2",
			.id = -1,
			.dev		= {
				.platform_data = "ldo2",
			}
 	},{
			.name = "reg-19-20-ldo3",
			.id = -1,
			.dev		= {
				.platform_data = "ldo3",
			}
 	},
#if defined (CONFIG_KP_AXP20)

	{
			.name = "reg-19-20-ldo4",
			.id = -1,
			.dev		= {
				.platform_data = "ldo4",
			}
	},
#endif

#if defined (CONFIG_KP_AXP19)

	{
			.name = "reg-19-20-dcdc1",
			.id = -1,
			.dev		= {
				.platform_data = "dcdc1",
			}
 	},
#endif



	{
			.name = "reg-19-20-dcdc2",
			.id = -1,
			.dev		= {
				.platform_data = "dcdc2",
			}
 	},

	{
			.name = "reg-19-20-dcdc3",
			.id = -1,
			.dev		= {
				.platform_data = "dcdc3",
			}
 	},

	{
				.name = "reg-19-20-ldoio0",
				.id = -1,
				.dev		= {
					.platform_data = "ldoio0",
				}
		},

	
	
};



 static int __init virtual_init(void)
{
	int j,ret;
	for (j = 0; j < ARRAY_SIZE(virt); j++){
 		ret =  platform_device_register(&virt[j]);
  		if (ret)
				goto creat_devices_failed;
	}
	return ret;

creat_devices_failed:
	while (j--)
		platform_device_register(&virt[j]);
	return ret;

}

module_init(virtual_init);

static void __exit virtual_exit(void)
{
	int j;
	for (j = ARRAY_SIZE(virt) - 1; j >= 0; j--){
		platform_device_unregister(&virt[j]);
	}
}
module_exit(virtual_exit);

MODULE_DESCRIPTION("X-POWER axp regulator test");
MODULE_AUTHOR("Chen Guanrong X-powers");
MODULE_LICENSE("GPL");
