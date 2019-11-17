#ifndef __LED_H__
#define	__LED_H__

#include "stm32f10x.h"
#include "common.h"

#define ON  0
#define OFF 1

//带参宏，可以像内联函数一样使用
#define LED(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_13);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_13)

void LED_Blink(void);

#endif 
