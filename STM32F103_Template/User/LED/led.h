#ifndef __LED_H__
#define	__LED_H__

#include "stm32f10x.h"
#include "common.h"

#define ON  0
#define OFF 1

//���κ꣬��������������һ��ʹ��
#define LED(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_13);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_13)

void LED_Blink(void);

#endif 