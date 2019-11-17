#include "ultrasonic.h"
#include "led.h"

//�������ӿڳ�ʼ��
void Ultrasonic_GPIO_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIOB�˿�ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//GPIOB�������ó�����														   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//TRIG = PB5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//ECHO = PB4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);	
}
// ��ʼ��������
void Init_Ultrasonic(void)
{
	TRIG_H;
	Delay_us(20); // ����10us
	TRIG_L; // ����һ��20us�����崥���ź�
}

// ���������
double Origin_Measure_Distance(void)
{
	u16 cnt1=0,cnt2=0;
	u16 T=0;
	double S=0;
	
	Init_Ultrasonic(); // ����������
	// �ز��źŵ�����������������
	// S = �ز��źŸߵ�ƽʱ��*���٣�340m/s��/2�������������Ϊ60ms���ϣ���ֹ�����źŶԻ����źŵ�Ӱ��
	while(ECHO_Flag == 0); //�ȴ�ECHO�ز����ű�Ϊ�ߵ�ƽ
	// ��1����ȡ��ǰ��ʱ������ֵcnt1�������־λ����TIM4_Count = 0;
	TIM4->CNT = 0; TIM4_Count = 0;
	cnt1 = TIM4->CNT; 
	// ��2���ж�ECHO�ߵ�ƽ�Ƿ�������Ƿ������TIM4_Count++;
	while(ECHO_Flag == 1); 
	// ��3����ȡ��ǰ��ʱ������ֵcnt2
	cnt2 = TIM4->CNT;
	// ��1������ߵ�ƽʱ�䣻T = cnt2 - cnt1 + TIM4_Count*������ڣ�us��;
	T = cnt2 - cnt1 + TIM4_Count*1000;
	//printf("cnt1 = %d, cnt2 = %d, TIM4_Count = %d, T = %d\r\n",cnt1,cnt2,TIM4_Count,T);

	// �ж�ģ���Ƿ񳬳�������4m
	// ������룻S = (T/1000)*340/2.0 - 1��cm���¶Ȳ���1cm����
	S = (double)(T*340/10000)/2;
	// printf("S = %.1lf\r\n",S);
	
	return S;
}



