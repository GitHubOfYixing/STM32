#include "motor.h"

//电机控制程序
void Motor_Control(u8 M_Flag, u16 PWM1, u16 PWM2)
{
	if(M_Flag == 1)
	{
		TIM_SetCompare1(TIM4,PWM1);	// 更新M1_L占空比(PB6)			
		TIM_SetCompare2(TIM4,PWM2); // 更新M1_R占空比(PB7)
	}
	if(M_Flag == 2)
	{
		TIM_SetCompare3(TIM4,PWM1);	// 更新M2_L占空比(PB8)			
		TIM_SetCompare4(TIM4,PWM2); // 更新M2_R占空比(PB9)
	}
}

//前进
void Motor_Forward(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, PWM1, 0); // 电机1控制		
		Motor_Control(2, 0, PWM2); // 电机2控制	
	
	  //printf("Forward: PWM1 = %d\r\n", PWM1);
}
//后退
void Motor_Back(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, 0, PWM1); // 电机1控制		
		Motor_Control(2, PWM2, 0); // 电机2控制	

	  //printf("Back: PWM2 = %d\r\n", PWM2);
}
//左转
void Motor_TurnLeft(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, 0, PWM1); // 电机1控制		
		Motor_Control(2, 0, PWM2); // 电机2控制	
	
	  //printf("TurnLeft: PWM1 = %d, PWM2 = %d\r\n", PWM1, PWM2);
}
//右转
void Motor_TurnRight(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, PWM1, 0); // 电机1控制		
		Motor_Control(2, PWM2, 0); // 电机2控制

	  //printf("TurnRight: PWM1 = %d, PWM2 = %d\r\n", PWM1, PWM2);	
}
//停止
void Motor_Stop(void)
{
		Motor_Control(1, 0, 0); // 电机1控制		
		Motor_Control(2, 0, 0); // 电机2控制	
}


