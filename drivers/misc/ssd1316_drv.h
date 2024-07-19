#ifndef _SSD1316_DRV_H
#define _SSD1316_DRV_H



typedef struct
{
    unsigned char X;
    unsigned char Y;
    unsigned char Len;
    unsigned char Page;
    unsigned char *pRes;
}OLED_STRUCT;

struct oled1316_platform_data {
	int gpio_enable;
	int gpio_reset;
};


#endif	//_SSD1316_DRV_H