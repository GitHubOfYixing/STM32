#include "ultrasonic.h"
#include "led.h"

//超声波接口初始化
void Ultrasonic_GPIO_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIOB端口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//GPIOB引脚配置超声波														   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//TRIG = PB5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//ECHO = PB4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);	
}
// 初始化超声波
void Init_Ultrasonic(void)
{
	TRIG_H;
	Delay_us(20); // 大于10us
	TRIG_L; // 产生一个20us的脉冲触发信号
}

// 超声波测距
double Origin_Measure_Distance(void)
{
	u16 cnt1=0,cnt2=0;
	u16 T=0;
	double S=0;
	
	Init_Ultrasonic(); // 触发超声波
	// 回波信号的脉冲宽度与距离成正比
	// S = 回波信号高电平时间*声速（340m/s）/2；建议测量周期为60ms以上，防止发射信号对回响信号的影响
	while(ECHO_Flag == 0); //等待ECHO回波引脚变为高电平
	// （1）获取当前定时器计数值cnt1，溢出标志位清零TIM4_Count = 0;
	TIM4->CNT = 0; TIM4_Count = 0;
	cnt1 = TIM4->CNT; 
	// （2）判断ECHO高电平是否结束，是否溢出，TIM4_Count++;
	while(ECHO_Flag == 1); 
	// （3）获取当前定时器计数值cnt2
	cnt2 = TIM4->CNT;
	// （1）计算高电平时间；T = cnt2 - cnt1 + TIM4_Count*溢出周期（us）;
	T = cnt2 - cnt1 + TIM4_Count*1000;
	//printf("cnt1 = %d, cnt2 = %d, TIM4_Count = %d, T = %d\r\n",cnt1,cnt2,TIM4_Count,T);

	// 判断模块是否超出最大距离4m
	// 计算距离；S = (T/1000)*340/2.0 - 1（cm，温度补偿1cm）；
	S = (double)(T*340/10000)/2;
	// printf("S = %.1lf\r\n",S);
	
	return S;
}



