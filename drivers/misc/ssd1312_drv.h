#ifndef _SSD1312_DRV_H
#define _SSD1312_DRV_H


enum
{
	ROLL_UP = 0x00,
	ROLL_DOWN,
	ROLL_LEFT,
	ROLL_RIGHT,
};

typedef struct
{
	unsigned char X;
	unsigned char Y;
	unsigned char Len;
	unsigned char Page;
	unsigned char *pRes;
}oled_bitmap;


typedef struct
{
	unsigned char x_start;
	unsigned char x_end;
	unsigned char y_start;
	unsigned char y_end;
	unsigned char mode;
}oled_roll_rect;


#endif	//_SSD1312_DRV_H