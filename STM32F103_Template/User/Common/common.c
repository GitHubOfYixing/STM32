/*Include---------------------------*/
#include "common.h"

uint16_t TIM4_Count = 0;
uint16_t ADC_Data[ADC_Data_Size];

/* SPI���� */
volatile uint8_t  gSPI_RxBuf[10];               //SPI����BUF
volatile uint8_t  gSPI_FlagOver = FLAG_N_VALID; //SPI�����־
volatile uint16_t gSPI_Cnt = 0; 
/*******************************************************************************
 * ��������BKP��д
 * ����  ����������
 * ����  ����
 * ���  ����
*******************************************************************************/
void BKP_ReadWrite(void)
{
  PWR_BackupAccessCmd(ENABLE); //����д�������
  BKP_ClearFlag(); //�����־
  if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
  {
    RCC_ClearFlag();
    //��һ���ϵ�
    if(BKP_ReadBackupRegister(BKP_DR1) != 0xA55A)
    {
      printf("Is't first\r\n");
			//��һ��д������0xA55A
      BKP_WriteBackupRegister(BKP_DR1, 0xA55A);  
    }
    //�ǵ�һ���ϵ�
    else
      printf("Not first\r\n");
  }
  else
    printf("RCC Err\r\n");
}

/*******************************************************************************
 * ��������SPI����
 * ����  ��
 * ����  ����
 * ���  ����
*******************************************************************************/
void SPI_Configuration(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  /* SPI ��ʼ������ */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI���ͽ��� 8 λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         //ʱ�ӿ���Ϊ��
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       //���ݲ����ڵڶ���ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //������� NSS �ź�
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //������Ԥ��Ƶֵ
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //���ݴ���� MSB λ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;                           //���������� CRCֵ����Ķ���ʽ
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);                   //ʹ�ܽ����ж�

  SPI_Cmd(SPI1, ENABLE);                                             //ʹ��SPI1
}

/*******************************************************************************
 * ��������SPI��д
 * ����  ����SPIд���ֽ�����
 * ����  ��TxData���͵��ֽ�����
 * ���  ����
*******************************************************************************/
uint8_t SPI_WriteReadByte(uint8_t TxData)
{
  while((SPI1->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI1->DR = TxData;

  while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  return SPI1->DR;
}

//SPI�ж�
void SPI1_IRQHandler(void)
{
  if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
  {
    gSPI_RxBuf[gSPI_Cnt++] = SPI1->DR;           //��������
    if(10 <= gSPI_Cnt)
    {
      gSPI_Cnt = 0;
      gSPI_FlagOver = FLAG_VALID;
    }
  }
}
/*******************************************************************************
 * ��������GPIO����
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
*******************************************************************************/
void GPIO_Configuration(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef GPIO_Mode_x, GPIOSpeed_TypeDef GPIO_Speed_x)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO��������													   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_x;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_x; 
  GPIO_Init(GPIOx, &GPIO_InitStructure);		  
  GPIO_SetBits(GPIOx, GPIO_Pin_x);
}

/**************************************************************************************
 * ��  �� : LED��˸
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void LED_Blink(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);  //��PC13�����õ�,����ָʾ��
	Delay_ms(500);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);    //��PC13�����ø�,Ϩ��ָʾ��
	Delay_ms(500);
}

/**************************************************************************************
 * ��  �� : ADC����
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void ADC_Configuration(ADC_TypeDef * ADCx)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC�����ڶ���ģʽ 
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE; //��ͨ��ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //���ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //��DISABLE,���βɼ�ģʽ��ENABLE,ѭ���ɼ�ģʽ��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //ת�����������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ת���Ҷ���
	//ADC_InitStructure.ADC_NbrOfChannel = 1; //ת��ͨ����1
	ADC_InitStructure.ADC_NbrOfChannel = 3; //ת��ͨ����3
	ADC_Init(ADCx, &ADC_InitStructure);
  /******************************************************************
     ����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
     ʹ��ADC1,ģ��ͨ��1���������к�Ϊ1������ʱ��Ϊ55.5����
  *******************************************************************/	
	ADC_RegularChannelConfig(ADCx, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5); //���ù�����(ADCͨ��1,1��ͨ��������ʱ��239.5)
  ADC_RegularChannelConfig(ADCx, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5);
	
	ADC_DMACmd(ADCx,ENABLE);//��AD��DMA����һ��	
	
	ADC_Cmd(ADCx,ENABLE); //ADCʹ��	
	
	ADC_ResetCalibration(ADCx); //ADC��λ
	while(ADC_GetResetCalibrationStatus(ADCx)); //�ȴ��Ƿ�λ���
	
	ADC_StartCalibration(ADCx); //ADC1У׼
	while(ADC_GetCalibrationStatus(ADCx)); //�ȴ��Ƿ�У׼���	
}

/**************************************************************************************
 * ��  �� : DMA����
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void DMA_Configuration(ADC_TypeDef * ADCx)
{
	DMA_InitTypeDef DMA_InitStructure;	
	DMA_DeInit(DMA1_Channel1); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADCx->DR;//�����ַADC1->DR 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Data;//���ݴ���׵�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//������Ϊ���ݴ������Դ 
	DMA_InitStructure.DMA_BufferSize = ADC_Data_Size;//��������С����ͬ�����С 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ�Ĵ�������	 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�������ݿ��Ϊ16λ 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�ڴ����ݿ��Ϊ16λ	 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//������ѭ������ģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMAͨ��1ӵ�и����ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMAͨ��1û������Ϊ�ڴ浽�ڴ洫��,���赽�ڴ�
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1,ENABLE);//DMAʹ��
}

/**************************************************************************************
 * ��  �� : ADC�ɼ�
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void Get_ADC(ADC_TypeDef * ADCx, uint32_t DRValue)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);      //����ת��,�ȴ�ת�����
	while(!DMA_GetFlagStatus(DMA1_IT_TC1));      //�ȴ��������
	DMA_ClearFlag(DMA1_IT_GL1);                  //�����־λ
	//ADC��������ɼ�������ƽ��ֵ
  uint32_t ResultVolt = 0;
  ResultVolt = (DRValue*3300) / (4096-1);   
	printf("%.2lfV\r\n", (double)ResultVolt/1000);
	
	/*
  uint32_t ResultVolt = 0;
  uint8_t i;
  for(i=0; i<8; i++)
  {
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);      //����ת��,�ȴ�ת�����
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));
    ResultVolt += (uint32_t)ADC_GetConversionValue(ADCx);
  }
  ResultVolt = ResultVolt >> 3;                  //��ƽ��(��8)
  ResultVolt = (ResultVolt*3300) >> 12;          //����õ�1000���ĵ�ѹֵ
	printf("PA1��ѹֵ = %.2lfV\r\n", (double)ResultVolt/1000);
	
  //return ResultVolt;                             //����: ʵ�ʵ�ѹΪ1.25V, ���ص�����Ϊ1250
	*/
}

/**************************************************************************************
 * ��  �� : DAC��ʼ��
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void DAC_Configuration(void)
{
  DAC_InitTypeDef  DAC_InitStructure;

  /* DA1���� */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;              //�������DAת��
  //DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;               //TIM2����DAת��
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;    //����������
  //DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;//���ǲ���
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
  //DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;      //ʹ���������
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  DAC_Cmd(DAC_Channel_1, ENABLE);                                    //ʹ��DAͨ��1
  
	//DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);             //����ͨ��ֵ
}

/**************************************************************************************
 * ��  �� : DAC�����ѹ
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void DAC_OutVoltage(float Voltage)
{
  uint16_t data;

  data = (uint16_t)((Voltage/3.3) * 4096);       //������ֵ
  DAC_SetChannel1Data(DAC_Align_12b_R, data);    //12λ�� �Ҷ���
  DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE); //����ת��
}

/**************************************************************************************
 * ��  �� : ���ö�ʱ��
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void TIM_Configuration(TIM_TypeDef* TIMx, u16 arr, u16 psc)
{
	//TIMx����
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ClearITPendingBit(TIMx,TIM_IT_Update);//��ʱ���ж����
	//��1/��Ƶ�����ʱ�ӣ�*����ֵ=�趨ʱ��;��1/(72MHZ/7200)��*10000=1s
	TIM_TimeBaseStructure.TIM_Period = arr;//1000-1, ����������ֵ������1000�Σ�����һ���жϣ�
	TIM_TimeBaseStructure.TIM_Prescaler = psc;//72-1, Ĭ�ϻ��1��Ԥ��Ƶ��72M/72=1M������һ��ʱ��t=1/1M=1us��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//0, ������ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);//��ʱ����ʽִ��
  TIM_Cmd(TIMx,ENABLE);//��ʱ������ʹ��
}

// TIM3�жϴ�����
void TIM3_IRQHandler(void) 
{ 
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{ 
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 
	} 
}

// TIM4�жϴ�����
void TIM4_IRQHandler(void) 
{ 
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{ 
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 
		TIM4_Count++;
		if(TIM4_Count == 1000)//1000ms��һ���ٶ�
		{
			TIM4_Count = 0;
			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13) == Bit_RESET)//����״̬
				GPIO_SetBits(GPIOC,GPIO_Pin_13); //LEDϨ��
			else
				GPIO_ResetBits(GPIOC,GPIO_Pin_13); //LED����	
		}
	} 
}

/**************************************************************************************
 * ��  �� : ����PWM���
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
// TIM1 CH1-PA8; CH2-PA9; CH3-PA10; CH4_PA11
// TIM2 CH1-PA0; CH2-PA1; CH3-PA2;  CH4_PA3
// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1
// TIM4 CH1-PB6; CH2-PB7; CH3-PB8;  CH4_PB9
void PWM_Configuration(TIM_TypeDef* TIMx, u8 Channal)
{
	//PWM����
	TIM_OCInitTypeDef TIM_OCInitStructure;//PWM����	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//���ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//�������Ϊ��
	switch(Channal)
	{
		case 1: TIM_OC1Init(TIMx, &TIM_OCInitStructure);//TIMͨ��1��ʼ��
						TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Enable);//Ԥװ��ʹ�ܣ��ظ�ִ��
		break;
		case 2: TIM_OC2Init(TIMx, &TIM_OCInitStructure);//TIMͨ��2��ʼ��
						TIM_OC2PreloadConfig(TIMx,TIM_OCPreload_Enable);//Ԥװ��ʹ�ܣ��ظ�ִ��
		break;
		case 3:	TIM_OC3Init(TIMx, &TIM_OCInitStructure);//TIMͨ��3��ʼ��
						TIM_OC3PreloadConfig(TIMx,TIM_OCPreload_Enable);//Ԥװ��ʹ�ܣ��ظ�ִ��
		break;	
		case 4:	TIM_OC4Init(TIMx, &TIM_OCInitStructure);//TIMͨ��4��ʼ��
						TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable);//Ԥװ��ʹ�ܣ��ظ�ִ��
		break;
		default: TIM_OC4Init(TIMx, &TIM_OCInitStructure);//TIMͨ��4��ʼ��
						 TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable);//Ԥװ��ʹ�ܣ��ظ�ִ��	
		break;
	}	
  TIM_Cmd(TIMx,ENABLE);//��ʱ������ʹ��
}

/**************************************************************************************
 * ��  �� : ��������
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
// USART1 TXD=PA9; RXD=PA10
// USART2 TXD=PA2; RXD=PA3
// USART3 TXD=PB10; RXD=PB11
void USART_Configuration(USART_TypeDef* USARTx, u16 BaudRate)
{
	//USART����
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = BaudRate; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;	//��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ�������ƣ�DTR��RTS��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//Tx��Rxʹ��
	// ��ʼ��USART
	USART_Init(USARTx, &USART_InitStructure);
	USART_ITConfig(USARTx,USART_IT_RXNE,ENABLE); //�򿪽����ж�
	USART_ITConfig(USARTx,USART_IT_IDLE,ENABLE); //�򿪴���IDLE�ж�
	USART_Cmd(USARTx,ENABLE); //�򿪴���ʹ��
	USART_ClearFlag(USARTx,USART_FLAG_TC); //��շ�����ɱ�־λ
}

// USART1�жϺ���
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)//�жϵ�ǰ�Ƿ�����ж�
	{
		USART_SendData(USART1,USART_ReceiveData(USART1)); //�����յ������ݷ��ص���λ��
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //�ȴ��������
	}
}

// USART2�жϺ���
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)//�жϵ�ǰ�Ƿ�����ж�
	{
		USART_SendData(USART2,USART_ReceiveData(USART2)); //�����յ������ݷ��͸�USART1
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET); //�ȴ��������
	}
}

// USART3�жϺ���
void USART3_IRQHandler(void)
{
  u8 k = 0;
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)//�жϵ�ǰ�Ƿ�����ж�
	{
		k = USART_ReceiveData(USART3); //��ȡ��������
		USART_SendData(USART3,k); //�����յ������ݷ���
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //�ȴ��������
	}
}

/*******************************************************************************
 * ��  �� : ��ʽ�������
 * ��  �� : ��
 * ����ֵ : ��
*******************************************************************************/
int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(u8)ch); //��Ҫ���͵����ݷ��͵�������
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //�ȴ��������
	
	return ch;
}

/**************************************************************************************
 * ��  �� : �����ж���Ӧ���ȼ�
 * ��  �� : ��
 * ����ֵ : ��
 **************************************************************************************/
void NVIC_Configuration(uint32_t NVIC_PriorityGroup_x, u8 x_IRQn, u8 IRQ_PP, u8 IRQ_SP)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_x);//�ֵ���x��
	NVIC_InitStructure.NVIC_IRQChannel = x_IRQn;//�ж�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_PP;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = IRQ_SP;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * ��  �� : us��ʱ����
 * ��  �� : ��ʱʱ��
 * ����ֵ : ��
*******************************************************************************/
void Delay_us(u32 us)
{
	u32 temp;							   
	SysTick->LOAD = 9*us;//���ֵ0xFFFFFF=16 777 215��SysTick��װ��ֵ�Ĵ�����ϵͳʱ��72M��8��Ƶ��Ϊ9M��1us����9��
	SysTick->VAL = 0x00;//SysTick��ǰֵ�Ĵ�������ռ�����
	SysTick->CTRL = 0x01;//SysTick���ƺ�״̬�Ĵ���
	/*
	��0λ��ENABLE��Systickʹ��λ��0���رգ�1��������
	��1λ��TICKINT��Systick�ж�ʹ��λ��0���رգ�1��������
	��2λ��CLKSOURCE��Systickʱ��Դѡ��0��HCLK/8��1��HCLK��
	��16λ��COUNTFLAG��Systick�����Ƚϱ�־��0��Systick������0��1����λ����ȡ��
	*/
	do
	{
	   temp = SysTick->CTRL;//��ȡ��ǰ������ֵ
	}while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
	SysTick->CTRL = 0x00;//�رռ�����
	SysTick->VAL = 0x00;//��ռ�����
}

/*******************************************************************************
 * ��  �� : ms��ʱ����
 * ��  �� : ��ʱʱ��
 * ����ֵ : ��
*******************************************************************************/
void Delay_ms(u16 ms)
{
	u32 temp;
	SysTick->LOAD = 9000*ms;//ϵͳʱ��72M��8��Ƶ��Ϊ9M��1us����9��
	SysTick->VAL = 0x00;//��ռ�����
	SysTick->CTRL = 0x01;//SysTick���ƺ�״̬�Ĵ���

	do
	{
	   temp = SysTick->CTRL;//��ȡ��ǰ������ֵ
	}while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
	SysTick->CTRL = 0x00;//�رռ�����
	SysTick->VAL = 0x00;//��ռ�����
}

/*******************************************************************************
 * ��  �� : �ײ��ʼ��
 * ��  �� : ��
 * ����ֵ : ��
*******************************************************************************/
void BSP_Initializes(void)
{
	//RCC_HSE_Configuration(); //�Զ���ϵͳʱ�ӳ�ʼ��
  SystemInit();//����ϵͳʱ��72MHZ 	
	//GPIOA�˿�ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//�˿ڹܽŸ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/******************************************************************************************/
	//GPIOC�˿�ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//GPIOC��������LED
	GPIO_Configuration(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
	//GPIOB�˿�ʹ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//�жϿ���������(PB12~PB15)
	//GPIO_Configuration(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_Mode_IPU, GPIO_Speed_50MHz);
	/******************************************************************************************/
	
  //TIM1�жϿ���ʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA�˿�����
	//GPIO_Configuration(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM1��ʼ��
	//TIM_Configuration(TIM1, 1000-1, 72-1);
	//TIM1�ж����ȼ�����
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM1_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM2�жϿ���ʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA�˿�����
	//GPIO_Configuration(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM2��ʼ��
	//TIM_Configuration(TIM2, 1000-1, 72-1);
	//TIM2�ж����ȼ�����
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM2_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM3�жϿ���ʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA�˿�����
	//GPIO_Configuration(GPIOA, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOB�˿�����
	//GPIO_Configuration(GPIOB, GPIO_Pin_0 | GPIO_Pin_1, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM3��ʼ��
	//TIM_Configuration(TIM3, 1000-1, 72-1);
	//TIM3�ж����ȼ�����
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM3_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM4�жϿ���ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOB�˿�����
	GPIO_Configuration(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM4��ʼ��
	TIM_Configuration(TIM4, 1000-1, 72-1); //PWM��������f=72000000/72/1000=1000Hz,T=1ms
	//TIM4�ж����ȼ�����
	NVIC_Configuration(NVIC_PriorityGroup_3, TIM4_IRQn, 0, 1); 
	/******************************************************************************************/
	
	//PWM_Configuration(TIM4, 1); //����TIM4��ͨ��1��ΪPWM1_L���
	//PWM_Configuration(TIM4, 2); //����TIM4��ͨ��2��ΪPWM1_R���
	//PWM_Configuration(TIM4, 3); //����TIM4��ͨ��3��ΪPWM2_L���
	//PWM_Configuration(TIM4, 4); //����TIM4��ͨ��4��ΪPWM2_R���
	/******************************************************************************************/
	
	//USART1����ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	//GPIOA�˿�����(GPIO_Pin_TXD)
	GPIO_Configuration(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOA�˿�����(GPIO_Pin_RXD)
	GPIO_Configuration(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	//USART1��ʼ��
	USART_Configuration(USART1, 9600);
	//USART1�ж����ȼ�����
	NVIC_Configuration(NVIC_PriorityGroup_1, USART1_IRQn, 0, 1);
	/******************************************************************************************/
	
	//USART2����ʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//GPIOA�˿�����(GPIO_Pin_TXD)
	//GPIO_Configuration(GPIOA, GPIO_Pin_2, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOA�˿�����(GPIO_Pin_RXD)
	//GPIO_Configuration(GPIOA, GPIO_Pin_3, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);	
	//USART2��ʼ��
	//USART_Configuration(USART2, 9600);
	//USART2�ж����ȼ�����
	//NVIC_Configuration(NVIC_PriorityGroup_3, USART2_IRQn, 0, 4);
	/******************************************************************************************/
	
	//USART3����ʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
	//GPIOB�˿�����(GPIO_Pin_TXD)
	//GPIO_Configuration(GPIOB, GPIO_Pin_10, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOB�˿�����(GPIO_Pin_RXD)
	//GPIO_Configuration(GPIOB, GPIO_Pin_11, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);	
	//USART3��ʼ��
	//USART_Configuration(USART3, 9600);
	//USART3�ж����ȼ�����
	//NVIC_Configuration(NVIC_PriorityGroup_3, USART3_IRQn, 0, 4); 
	/******************************************************************************************/	

	//ADC1ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	//ADC����Ƶ��С��14MHZ
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //(72MHZ)/6=12MHZ 
	//GPIOA�˿�����
	GPIO_Configuration(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AIN, GPIO_Speed_50MHz);
	//ADC1��ʼ��
	ADC_Configuration(ADC1);
	//DMAʹ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA��ʼ��
	DMA_Configuration(ADC1);
	/******************************************************************************************/	
	
	//��Դ����PWR,BKPʹ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP, ENABLE);
	/******************************************************************************************/	
	
	//DACʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	//GPIOA�˿�����
	GPIO_Configuration(GPIOA, GPIO_Pin_4, GPIO_Mode_AIN, GPIO_Speed_50MHz);
	//DAC��ʼ��
	DAC_Configuration();
	/******************************************************************************************/	

	//SPIʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//GPIO�˿�����(SCK��������)
	GPIO_Configuration(GPIOA, GPIO_Pin_5, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIO�˿�����(MOSI���ù���)
	GPIO_Configuration(GPIOA, GPIO_Pin_7, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIO�˿�����(MISO���ù���)
	GPIO_Configuration(GPIOA, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	//GPIO�˿�����(CS�������)
	GPIO_Configuration(GPIOA, GPIO_Pin_4, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);	
	//SPI��ʼ��
	SPI_Configuration();
	//SPI1�ж����ȼ�����
	NVIC_Configuration(NVIC_PriorityGroup_2, SPI1_IRQn, 1, 1); 
	/******************************************************************************************/	

}
