#ifndef _KTD2026_DRV_H
#define _KTD2026_DRV_H



#define	LED_RED			0
#define	LED_GREEN		1
#define	LED_BLUE		2
#define	LED_YELLOW	3
#define	LED_PURPLE	4
#define	LED_CYAN		5
#define	LED_WHITE		6
#define	LED_BLACK		7
#define LED_MQARED      8


//typedef enum
//{
//	LED_RED = 0x00,
//	LED_GREEN,	
//	LED_BLUE,
//	LED_YELLOW,
//	LED_PURPLE,
//	LED_CYAN,
//	LED_WHITE,
//	LED_BLACK
//}led_type;

void KTD2026_Init(void);
void KTD2026_Control(unsigned char LedType);

#endif	//_KTD2026_DRV_H

