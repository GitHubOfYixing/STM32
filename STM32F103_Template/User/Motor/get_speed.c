#include "get_speed.h"

// 霍尔编码器测速模块
//		如果两个信号相位差为90度，则这两个信号称为正交。
//		由于两个信号相差90度，因此可以根据两个信号哪个先哪个后来判断方向。
//		根据每个信号脉冲数量的多少及整个编码轮的周长就可以算出当前行走的距离、如果再加上定时器的话还可以计算出速度。
// 单位cm/s
// 转速计算方法:
//		用捕获值（一秒内输出的脉冲数）/编码器线数（转速一圈输出脉冲数）/电机减数比（内部电机转动圈数与电机输出轴转动圈数比，即减速齿轮比）
// 在STM32芯片中，都有这么一个定时器，叫通用定时器“General-purpose timers”，定时器里面存在这么一个模式，叫编码器接口模式“Encoder interface mode”。
// STM32提供的编码器接口模式主要针对的就是“正交编码器”，它可以利用定时器的“计数”功能，得出编码器计了多少个脉冲；同时，它可以根据编码器AB的相位得出编码器是正转，还是反转。

// 1.计算脉冲个数
//		有点类似TIM的捕获功能，捕获A相、B相的脉冲信号；只是编码器模式是捕获A(TI1)、B(TI2)相的边沿信号，相当于一个周期内，计4个脉冲信号的值。
// 2.计数器的增减（方向）
//		STM32的计数器会根据方向（+ 或者 -）来进行计数，TI1和TI2相位相差90，4个阶段的边沿，对应TI1和TI2不同电平信号，从这个不同的信号，硬件自身可以判断出其方向。在编码器模式下，有个寄存器（TIMx_CR1）中有一个方向位（DIR），会随着编码器旋转方向的改变而改变，我们可以通过读取该位来判断编码器的正转，还是反转。
// 3.TIM时基
//		STM32编码器接口模式，其实是通过利用AB相位TIM时基提供时钟信号，使其计数。
	
// 霍尔编码器端口初始化
void Hall_Sensor_Init(void)
{	
	// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1
	GPIO_InitTypeDef GPIO_InitStructure;	
	// IO时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	
	
	// IO配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入  
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1 
	// TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// 定时器3计数模式配置
	// TIM_Configuration(TIM3, 1000-1, 72-1); // 配置周期f=72000000/72/1000=1000Hz,T=1ms
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);// 定时器中断清空	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;// 设定计数器重装值（最大值为65536-1）
	TIM_TimeBaseStructure.TIM_Prescaler = 0;// TIM3时钟预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// TIM向上计数 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// 定时器3编码模式配置
	// TIM_EncoderMode参数是模式，是单相计数（只能反映速度）还是两相计数（速度和方向）
	// TIM_IC1Polarity和TIM_IC2Polarity参数就是通道1、2的捕捉极性	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,  TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	// 使用编码器模式3，上升下降都计数
  // TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);

	TIM_ICInitTypeDef TIM_ICInitStructure;	
  TIM_ICStructInit(&TIM_ICInitStructure);// 将结构体中的内容缺省输入
  TIM_ICInitStructure.TIM_ICFilter = 6;// 选择输入比较滤波器(670ns)
  TIM_ICInit(TIM3, &TIM_ICInitStructure);// 将TIM_ICInitStructure中的指定参数初始化TIM3

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);// 清除TIM3的更新标志位
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);// 运行更新中断
  // TIM_SetCounter(TIM3,0);
  TIM3->CNT = 0;
  TIM_Cmd(TIM3, ENABLE);// 启动TIM3定时器
}

// 获取速度
s16 Get_Speed(void)
{
	static u16 lastCount = 0;
//	double speed;

	// 单位时间读取编码器计数
	// 获取定时器3编码器数据
	// 速度=捕获值（一秒内输出的脉冲数）/编码器线数（转一圈输出的脉冲数）/电机减数比（减速齿轮比）
	//	speed = dAngle/2/10;
        
	u16  curCount = TIM3->CNT;//获取编码值
	s32 dAngle = curCount - lastCount;
	
	if(dAngle >= 10000) // 10000也就是1ms内不会超过10000个脉冲
	{
			dAngle -= 0xFFFF;
	}
	else if(dAngle < -10000)
	{
			dAngle += 0xFFFF;
	}
	lastCount = curCount;
	
	return (s16)(dAngle/2/10);
	
//	return speed;
}

