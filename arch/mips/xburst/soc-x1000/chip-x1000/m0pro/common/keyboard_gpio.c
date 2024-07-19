#include <linux/platform_device.h>

#include <linux/gpio_keys.h>
#include <linux/input.h>
#include "board_base.h"

struct gpio_keys_button __attribute__((weak)) board_buttons[] = {
#ifdef GPIO_ENDCALL_KEY
	{
		.gpio           = GPIO_ENDCALL_KEY,
		.code           = KEY_POWER,
		.desc           = "end call key",
		.active_low     = ACTIVE_LOW_ENDCALL,
		.wakeup         = 1,
		.debounce_interval = 10,
	},
#endif
#ifdef GPIO_LOCK_KEY
	{
		.gpio           = GPIO_LOCK_KEY,
		.code           = KEY_SLEEP,
		.desc           = "lock key",
		.active_low     = ACTIVE_LOW_LOCK,
		.wakeup         = 1,
		.debounce_interval = 10,
	},
#endif
};

static struct gpio_keys_platform_data board_button_data = {
	.buttons	= board_buttons,
	.nbuttons	= ARRAY_SIZE(board_buttons),
	.rep		= 1,
};

struct platform_device jz_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
                .platform_data	= &board_button_data,
	}
};
