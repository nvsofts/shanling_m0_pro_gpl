/*
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <gpio.h>

// default gpio state is input pull;
 __initdata int gpio_nc_table[][2] = {
	{GSS_TABLET_END,GSS_TABLET_END	},
};


__initdata int gpio_ss_table[][2] = {
    /* GPIO Group - A */
    {32*0+0,    GSS_OUTPUT_LOW  },    /* SLCD_D0 */
    {32*0+1,    GSS_OUTPUT_LOW  },    /* SLCD_D1 */
    {32*0+2,    GSS_OUTPUT_LOW  },    /* SLCD_D2 */
    {32*0+3,    GSS_OUTPUT_LOW  },    /* SLCD_D3 */
    {32*0+4,    GSS_OUTPUT_LOW  },    /* SLCD_D4 */
    {32*0+5,    GSS_OUTPUT_LOW  },    /* SLCD_D5 */
    {32*0+6,    GSS_OUTPUT_LOW  },    /* SLCD_D6 */
    {32*0+7,    GSS_OUTPUT_LOW  },    /* SLCD_D7 */
    {32*0+8,    GSS_OUTPUT_LOW  },    /* null */
    {32*0+9,    GSS_OUTPUT_LOW  },    /* null */
    {32*0+10,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+11,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+12,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+13,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+14,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+15,   GSS_OUTPUT_LOW  },    /* null */
    {32*0+16,   GSS_OUTPUT_LOW  },    /* tp_int */
    {32*0+17,   GSS_OUTPUT_LOW  },    /* usb_dete */
    {32*0+18,   GSS_INPUT_PULL  },    /* null */
    {32*0+19,   GSS_OUTPUT_LOW  },    /* tp_rst */
    {32*0+20,   GSS_OUTPUT_LOW  },    /* MSC0_D3 */
    {32*0+21,   GSS_OUTPUT_LOW  },    /* MSC0_D2 */
    {32*0+22,   GSS_OUTPUT_LOW  },    /* MSC0_D1 */
    {32*0+23,   GSS_OUTPUT_LOW  },    /* MSC0_D0 */
    {32*0+24,   GSS_OUTPUT_LOW  },    /* MSC0_CLK */
    {32*0+25,   GSS_OUTPUT_LOW  },    /* MSC0_CMD */
    {32*0+26,   GSS_INPUT_PULL  },    /* SFC_CLK */
    {32*0+27,   GSS_INPUT_PULL  },    /* SFC_CE */
    {32*0+28,   GSS_INPUT_PULL  },    /* SFC_DR */
    {32*0+29,   GSS_INPUT_PULL  },    /* SFC_DT */
    {32*0+30,   GSS_INPUT_PULL  },    /* SFC_WP */
    {32*0+31,   GSS_INPUT_PULL  },    /* SFC_HOLD */

    /* GPIO Group - B */
    {32*1+0,    GSS_INPUT_PULL  },    /* I2S_MCLK */
    {32*1+1,    GSS_OUTPUT_LOW  },    /* I2S_BCLK */
    {32*1+2,    GSS_OUTPUT_LOW  },    /* I2S_LRCK */
    {32*1+3,    GSS_INPUT_PULL  },    /* I2S_DI */
    {32*1+4,    GSS_OUTPUT_LOW  },    /* I2S_DO */
    {32*1+5,    GSS_INPUT_PULL  },    /* null */
    {32*1+6,    GSS_OUTPUT_LOW  },    /* lcden */
    {32*1+7,    GSS_OUTPUT_LOW  },    /* lcden1 */
    {32*1+8,    GSS_OUTPUT_LOW  },    /* tp_pwr*/
    {32*1+9,    GSS_INPUT_NOPULL  },    /* sd_cd_n */
    {32*1+10,   GSS_INPUT_PULL  },    /* null */
    {32*1+11,   GSS_INPUT_PULL  },    /* null */
    {32*1+12,   GSS_INPUT_PULL  },    /* null */
    {32*1+13,   GSS_OUTPUT_LOW  },    /* po_pwr */
    {32*1+14,   GSS_OUTPUT_LOW  },    /* lcd_int */
    {32*1+15,   GSS_OUTPUT_LOW  },    /* lcd_rst */
    {32*1+16,   GSS_OUTPUT_LOW  },    /* SLCD_RD */
    {32*1+17,   GSS_OUTPUT_LOW  },    /* SLCD_WR */
    {32*1+18,   GSS_OUTPUT_LOW  },    /* SLCD_CS */
    {32*1+19,   GSS_OUTPUT_LOW  },    /* SLCD_TE */
    {32*1+20,   GSS_OUTPUT_LOW  },    /* SLCD_RS */
    {32*1+21,   GSS_INPUT_PULL  },    /* null */
    {32*1+22,   GSS_INPUT_PULL  },    /* null */
    {32*1+23,   GSS_OUTPUT_LOW  },    /* tp_da */
    {32*1+24,   GSS_OUTPUT_LOW  },    /* tp_scl */
    {32*1+25,   GSS_OUTPUT_LOW  },    /* DRVVBUS */
    {32*1+26,   GSS_OUTPUT_LOW  },    /* CLK32K */
    {32*1+27,   GSS_INPUT_PULL  },    /* EXCLK */
    {32*1+28,   GSS_INPUT_NOPULL},    /* BOOT_SEL0 */
    {32*1+29,   GSS_INPUT_NOPULL},    /* BOOT_SEL1 */
    {32*1+30,   GSS_INPUT_NOPULL},    /* BOOT_SEL2 */
    {32*1+31,   GSS_INPUT_NOPULL},    /* WKUP */

    /* GPIO Group - C */
    {32*2+0,    GSS_INPUT_PULL  },     /* null */
    {32*2+1,    GSS_INPUT_PULL},     /* null */
    {32*2+2,    GSS_INPUT_PULL},     /* null */
    {32*2+3,    GSS_INPUT_PULL},     /* null */
    {32*2+4,    GSS_INPUT_PULL},     /* null */
    {32*2+5,    GSS_INPUT_PULL},     /* null */
    {32*2+6,    GSS_INPUT_PULL  },     /* PCM_CLK */
    {32*2+7,    GSS_INPUT_PULL  },     /* PCM_DO */
    {32*2+8,    GSS_INPUT_PULL  },     /* PCM_DI */
    {32*2+9,    GSS_INPUT_PULL  },     /* PCM_SYN */
    {32*2+10,   GSS_OUTPUT_LOW  },     /* UART0_RXD */
    {32*2+11,   GSS_OUTPUT_LOW  },     /* UART0_TXD */
    {32*2+12,   GSS_OUTPUT_LOW  },     /* UART0_CTS_N */
    {32*2+13,   GSS_OUTPUT_LOW  },     /* UART0_RTS_N */
    {32*2+14,   GSS_INPUT_PULL  },     /* null */
    {32*2+15,   GSS_INPUT_PULL  },    /* null */
    {32*2+16,   GSS_INPUT_PULL  },     /* null */
    {32*2+17,   GSS_INPUT_PULL  },     /* null*/
    {32*2+18,   GSS_OUTPUT_LOW  },     /* HOST WUKE BT */
    {32*2+19,   GSS_INPUT_PULL  },     /* null */
    {32*2+20,   GSS_OUTPUT_LOW  },     /* null */
    {32*2+21,   GSS_INPUT_PULL  },     /* pwe_irq */
    {32*2+22,   GSS_INPUT_PULL  },     /* null */
    {32*2+23,   GSS_INPUT_PULL  },     /* OTG_ID */
    {32*2+24,   GSS_INPUT_PULL  },     /* null */
    {32*2+25,   GSS_OUTPUT_LOW  },     /* PWM0 */
    {32*2+26,   GSS_OUTPUT_LOW  },     /* scl */
    {32*2+27,   GSS_OUTPUT_LOW  },     /* sda */
    {32*2+28,   GSS_IGNORE  },     /* null */
    {32*2+29,   GSS_IGNORE  },     /* null */
    {32*2+30,   GSS_IGNORE  },     /* null */
    {32*2+31,   GSS_IGNORE  },     /* null*/

    /*GPIO Group -D */
    {32*3+0,    GSS_INPUT_PULL  },     /* PWE_SCL*/
    {32*3+1,    GSS_INPUT_PULL  },     /* PWE_SDA*/
    {32*3+2,    GSS_INPUT_PULL  },     /* VOL+ */
    {32*3+3,    GSS_INPUT_PULL  },     /* VOL- */
    {32*3+4,    GSS_INPUT_PULL  },     /* null */
    {32*3+5,    GSS_OUTPUT_LOW  },     /* dac_rst */

    /* GPIO Group Set End */
    {GSS_TABLET_END,GSS_TABLET_END	}
};