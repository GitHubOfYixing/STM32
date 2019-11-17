#include "pwm.h"

u8 LED_Flag = 0;

void TIM3_Int_Init(u16 arr,u16 psc) 
{ 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	NVIC_InitTypeDef NVIC_InitStructure; 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = arr;    
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);         
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);   
	TIM_Cmd(TIM3, ENABLE); 
}

void TIM3_IRQHandler(void) 
{ 
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{ 
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  ); 
		LED_Flag = ~LED_Flag;
		LED1(LED_Flag);		
	} 
}

void TIM3_PWM_Init(u16 arr,u16 psc) 
{   
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器3时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); //使能GPIO与AFIO复用功能时钟  
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // 重映射TIM3_CH2 -> PB5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // TIM3_CH2   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; // 设置自动重装载周期值          
	TIM_TimeBaseStructure.TIM_Prescaler = psc; // 设置预分频值      
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; // 设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // 选择PWM模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); // 初始化外设TIM_OC2
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); // 使能TIM_OC2预装载寄存器
	TIM_Cmd(TIM3, ENABLE); // 使能TIM3 
}
