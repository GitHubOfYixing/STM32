#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "common.h"

void Motor_Control(u8 M_Flag, u16 PWM1, u16 PWM2);
void Motor_Forward(u16 PWM1, u16 PWM2);
void Motor_Back(u16 PWM1, u16 PWM2);
void Motor_TurnLeft(u16 PWM1, u16 PWM2);
void Motor_TurnRight(u16 PWM1, u16 PWM2);
void Motor_Stop(void);

#endif

