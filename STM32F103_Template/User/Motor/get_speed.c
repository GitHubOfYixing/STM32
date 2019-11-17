#include "get_speed.h"

// ��������������ģ��
//		��������ź���λ��Ϊ90�ȣ����������źų�Ϊ������
//		���������ź����90�ȣ���˿��Ը��������ź��ĸ����ĸ������жϷ���
//		����ÿ���ź����������Ķ��ټ����������ֵ��ܳ��Ϳ��������ǰ���ߵľ��롢����ټ��϶�ʱ���Ļ������Լ�����ٶȡ�
// ��λcm/s
// ת�ټ��㷽��:
//		�ò���ֵ��һ�����������������/������������ת��һȦ�����������/��������ȣ��ڲ����ת��Ȧ�����������ת��Ȧ���ȣ������ٳ��ֱȣ�
// ��STM32оƬ�У�������ôһ����ʱ������ͨ�ö�ʱ����General-purpose timers������ʱ�����������ôһ��ģʽ���б������ӿ�ģʽ��Encoder interface mode����
// STM32�ṩ�ı������ӿ�ģʽ��Ҫ��Եľ��ǡ������������������������ö�ʱ���ġ����������ܣ��ó����������˶��ٸ����壻ͬʱ�������Ը��ݱ�����AB����λ�ó�����������ת�����Ƿ�ת��

// 1.�����������
//		�е�����TIM�Ĳ����ܣ�����A�ࡢB��������źţ�ֻ�Ǳ�����ģʽ�ǲ���A(TI1)��B(TI2)��ı����źţ��൱��һ�������ڣ���4�������źŵ�ֵ��
// 2.������������������
//		STM32�ļ���������ݷ���+ ���� -�������м�����TI1��TI2��λ���90��4���׶εı��أ���ӦTI1��TI2��ͬ��ƽ�źţ��������ͬ���źţ�Ӳ����������жϳ��䷽���ڱ�����ģʽ�£��и��Ĵ�����TIMx_CR1������һ������λ��DIR���������ű�������ת����ĸı���ı䣬���ǿ���ͨ����ȡ��λ���жϱ���������ת�����Ƿ�ת��
// 3.TIMʱ��
//		STM32�������ӿ�ģʽ����ʵ��ͨ������AB��λTIMʱ���ṩʱ���źţ�ʹ�������
	
// �����������˿ڳ�ʼ��
void Hall_Sensor_Init(void)
{	
	// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1
	GPIO_InitTypeDef GPIO_InitStructure;	
	// IOʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	
	
	// IO����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������  
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1 
	// TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// ��ʱ��3����ģʽ����
	// TIM_Configuration(TIM3, 1000-1, 72-1); // ��������f=72000000/72/1000=1000Hz,T=1ms
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);// ��ʱ���ж����	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;// �趨��������װֵ�����ֵΪ65536-1��
	TIM_TimeBaseStructure.TIM_Prescaler = 0;// TIM3ʱ��Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// TIM���ϼ��� 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// ��ʱ��3����ģʽ����
	// TIM_EncoderMode������ģʽ���ǵ��������ֻ�ܷ�ӳ�ٶȣ���������������ٶȺͷ���
	// TIM_IC1Polarity��TIM_IC2Polarity��������ͨ��1��2�Ĳ�׽����	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,  TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	// ʹ�ñ�����ģʽ3�������½�������
  // TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);

	TIM_ICInitTypeDef TIM_ICInitStructure;	
  TIM_ICStructInit(&TIM_ICInitStructure);// ���ṹ���е�����ȱʡ����
  TIM_ICInitStructure.TIM_ICFilter = 6;// ѡ������Ƚ��˲���(670ns)
  TIM_ICInit(TIM3, &TIM_ICInitStructure);// ��TIM_ICInitStructure�е�ָ��������ʼ��TIM3

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);// ���TIM3�ĸ��±�־λ
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);// ���и����ж�
  // TIM_SetCounter(TIM3,0);
  TIM3->CNT = 0;
  TIM_Cmd(TIM3, ENABLE);// ����TIM3��ʱ��
}

// ��ȡ�ٶ�
s16 Get_Speed(void)
{
	static u16 lastCount = 0;
//	double speed;

	// ��λʱ���ȡ����������
	// ��ȡ��ʱ��3����������
	// �ٶ�=����ֵ��һ�����������������/������������תһȦ�������������/��������ȣ����ٳ��ֱȣ�
	//	speed = dAngle/2/10;
        
	u16  curCount = TIM3->CNT;//��ȡ����ֵ
	s32 dAngle = curCount - lastCount;
	
	if(dAngle >= 10000) // 10000Ҳ����1ms�ڲ��ᳬ��10000������
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

