#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "common.h"

// TRIG = PB4;
// ECHO = PB3;
#define TRIG_L GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define TRIG_H GPIO_SetBits(GPIOB, GPIO_Pin_5)  
// ******一定是读取输入数据******
#define ECHO_Flag GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)

void Ultrasonic_GPIO_Init(void);
void Init_Ultrasonic(void);
double Origin_Measure_Distance(void);

#endif


