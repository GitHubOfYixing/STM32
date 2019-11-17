#ifndef __PWM_H__
#define __PWM_H__

#include "led.h"

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
void TIM3_PWM_Init(u16 arr,u16 psc);

#endif

