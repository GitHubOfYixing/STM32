#include "motor.h"

//������Ƴ���
void Motor_Control(u8 M_Flag, u16 PWM1, u16 PWM2)
{
	if(M_Flag == 1)
	{
		TIM_SetCompare1(TIM4,PWM1);	// ����M1_Lռ�ձ�(PB6)			
		TIM_SetCompare2(TIM4,PWM2); // ����M1_Rռ�ձ�(PB7)
	}
	if(M_Flag == 2)
	{
		TIM_SetCompare3(TIM4,PWM1);	// ����M2_Lռ�ձ�(PB8)			
		TIM_SetCompare4(TIM4,PWM2); // ����M2_Rռ�ձ�(PB9)
	}
}

//ǰ��
void Motor_Forward(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, PWM1, 0); // ���1����		
		Motor_Control(2, 0, PWM2); // ���2����	
	
	  //printf("Forward: PWM1 = %d\r\n", PWM1);
}
//����
void Motor_Back(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, 0, PWM1); // ���1����		
		Motor_Control(2, PWM2, 0); // ���2����	

	  //printf("Back: PWM2 = %d\r\n", PWM2);
}
//��ת
void Motor_TurnLeft(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, 0, PWM1); // ���1����		
		Motor_Control(2, 0, PWM2); // ���2����	
	
	  //printf("TurnLeft: PWM1 = %d, PWM2 = %d\r\n", PWM1, PWM2);
}
//��ת
void Motor_TurnRight(u16 PWM1, u16 PWM2)
{
		Motor_Control(1, PWM1, 0); // ���1����		
		Motor_Control(2, PWM2, 0); // ���2����

	  //printf("TurnRight: PWM1 = %d, PWM2 = %d\r\n", PWM1, PWM2);	
}
//ֹͣ
void Motor_Stop(void)
{
		Motor_Control(1, 0, 0); // ���1����		
		Motor_Control(2, 0, 0); // ���2����	
}


