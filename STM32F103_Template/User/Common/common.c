/*Include---------------------------*/
#include "common.h"

uint16_t TIM4_Count = 0;
uint16_t ADC_Data[ADC_Data_Size];

/* SPI变量 */
volatile uint8_t  gSPI_RxBuf[10];               //SPI接收BUF
volatile uint8_t  gSPI_FlagOver = FLAG_N_VALID; //SPI溢出标志
volatile uint16_t gSPI_Cnt = 0; 
/*******************************************************************************
 * 函数名：BKP读写
 * 描述  ：备份数据
 * 输入  ：无
 * 输出  ：无
*******************************************************************************/
void BKP_ReadWrite(void)
{
  PWR_BackupAccessCmd(ENABLE); //允许写入后备区域
  BKP_ClearFlag(); //清除标志
  if(RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
  {
    RCC_ClearFlag();
    //第一次上电
    if(BKP_ReadBackupRegister(BKP_DR1) != 0xA55A)
    {
      printf("Is't first\r\n");
			//第一次写入数据0xA55A
      BKP_WriteBackupRegister(BKP_DR1, 0xA55A);  
    }
    //非第一次上电
    else
      printf("Not first\r\n");
  }
  else
    printf("RCC Err\r\n");
}

/*******************************************************************************
 * 函数名：SPI配置
 * 描述  ：
 * 输入  ：无
 * 输出  ：无
*******************************************************************************/
void SPI_Configuration(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  /* SPI 初始化定义 */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI发送接收 8 位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         //时钟空闲为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       //数据捕获于第二个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //软件控制 NSS 信号
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //波特率预分频值
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //数据传输从 MSB 位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;                           //定义了用于 CRC值计算的多项式
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);                   //使能接收中断

  SPI_Cmd(SPI1, ENABLE);                                             //使能SPI1
}

/*******************************************************************************
 * 函数名：SPI读写
 * 描述  ：对SPI写读字节数据
 * 输入  ：TxData发送的字节数据
 * 输出  ：无
*******************************************************************************/
uint8_t SPI_WriteReadByte(uint8_t TxData)
{
  while((SPI1->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI1->DR = TxData;

  while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  return SPI1->DR;
}

//SPI中断
void SPI1_IRQHandler(void)
{
  if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != RESET)
  {
    gSPI_RxBuf[gSPI_Cnt++] = SPI1->DR;           //保存数据
    if(10 <= gSPI_Cnt)
    {
      gSPI_Cnt = 0;
      gSPI_FlagOver = FLAG_VALID;
    }
  }
}
/*******************************************************************************
 * 函数名：GPIO配置
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
*******************************************************************************/
void GPIO_Configuration(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef GPIO_Mode_x, GPIOSpeed_TypeDef GPIO_Speed_x)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO引脚配置													   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_x;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_x; 
  GPIO_Init(GPIOx, &GPIO_InitStructure);		  
  GPIO_SetBits(GPIOx, GPIO_Pin_x);
}

/**************************************************************************************
 * 描  述 : LED闪烁
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void LED_Blink(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);  //将PC13引脚置低,点亮指示灯
	Delay_ms(500);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);    //将PC13引脚置高,熄灭指示灯
	Delay_ms(500);
}

/**************************************************************************************
 * 描  述 : ADC配置
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void ADC_Configuration(ADC_TypeDef * ADCx)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC工作在独立模式 
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE; //单通道模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //浏览模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //（DISABLE,单次采集模式；ENABLE,循环采集模式）
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //转换由软件启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //转换右对齐
	//ADC_InitStructure.ADC_NbrOfChannel = 1; //转换通道数1
	ADC_InitStructure.ADC_NbrOfChannel = 3; //转换通道数3
	ADC_Init(ADCx, &ADC_InitStructure);
  /******************************************************************
     设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
     使用ADC1,模拟通道1，采样序列号为1，采样时间为55.5周期
  *******************************************************************/	
	ADC_RegularChannelConfig(ADCx, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5); //采用规则组(ADC通道1,1个通道，采样时间239.5)
  ADC_RegularChannelConfig(ADCx, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5);
	
	ADC_DMACmd(ADCx,ENABLE);//将AD和DMA绑定在一起	
	
	ADC_Cmd(ADCx,ENABLE); //ADC使能	
	
	ADC_ResetCalibration(ADCx); //ADC复位
	while(ADC_GetResetCalibrationStatus(ADCx)); //等待是否复位完成
	
	ADC_StartCalibration(ADCx); //ADC1校准
	while(ADC_GetCalibrationStatus(ADCx)); //等待是否校准完成	
}

/**************************************************************************************
 * 描  述 : DMA配置
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void DMA_Configuration(ADC_TypeDef * ADCx)
{
	DMA_InitTypeDef DMA_InitStructure;	
	DMA_DeInit(DMA1_Channel1); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADCx->DR;//外设地址ADC1->DR 
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Data;//数据存放首地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据传输的来源 
	DMA_InitStructure.DMA_BufferSize = ADC_Data_Size;//缓冲区大小，等同数组大小 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增	 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据宽度为16位 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据宽度为16位	 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓存模式 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA通道1拥有高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道1没有设置为内存到内存传输,外设到内存
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1,ENABLE);//DMA使能
}

/**************************************************************************************
 * 描  述 : ADC采集
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void Get_ADC(ADC_TypeDef * ADCx, uint32_t DRValue)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);      //启动转换,等待转换完成
	while(!DMA_GetFlagStatus(DMA1_IT_TC1));      //等待传输完成
	DMA_ClearFlag(DMA1_IT_GL1);                  //清除标志位
	//ADC多次连续采集，求其平均值
  uint32_t ResultVolt = 0;
  ResultVolt = (DRValue*3300) / (4096-1);   
	printf("%.2lfV\r\n", (double)ResultVolt/1000);
	
	/*
  uint32_t ResultVolt = 0;
  uint8_t i;
  for(i=0; i<8; i++)
  {
    ADC_SoftwareStartConvCmd(ADCx, ENABLE);      //启动转换,等待转换完成
    while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC));
    ResultVolt += (uint32_t)ADC_GetConversionValue(ADCx);
  }
  ResultVolt = ResultVolt >> 3;                  //求平均(除8)
  ResultVolt = (ResultVolt*3300) >> 12;          //计算得到1000倍的电压值
	printf("PA1电压值 = %.2lfV\r\n", (double)ResultVolt/1000);
	
  //return ResultVolt;                             //比如: 实际电压为1.25V, 返回的数据为1250
	*/
}

/**************************************************************************************
 * 描  述 : DAC初始化
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void DAC_Configuration(void)
{
  DAC_InitTypeDef  DAC_InitStructure;

  /* DA1配置 */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;              //软件触发DA转换
  //DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;               //TIM2触发DA转换
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;    //不产生波形
  //DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;//三角波形
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
  //DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;      //使能输出缓存
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  DAC_Cmd(DAC_Channel_1, ENABLE);                                    //使能DA通道1
  
	//DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);             //设置通道值
}

/**************************************************************************************
 * 描  述 : DAC输出电压
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void DAC_OutVoltage(float Voltage)
{
  uint16_t data;

  data = (uint16_t)((Voltage/3.3) * 4096);       //换算数值
  DAC_SetChannel1Data(DAC_Align_12b_R, data);    //12位数 右对齐
  DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE); //启动转换
}

/**************************************************************************************
 * 描  述 : 配置定时器
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void TIM_Configuration(TIM_TypeDef* TIMx, u16 arr, u16 psc)
{
	//TIMx配置
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ClearITPendingBit(TIMx,TIM_IT_Update);//定时器中断清空
	//（1/分频后计数时钟）*计数值=设定时间;（1/(72MHZ/7200)）*10000=1s
	TIM_TimeBaseStructure.TIM_Period = arr;//1000-1, 赋计数周期值（计数1000次，产生一次中断）
	TIM_TimeBaseStructure.TIM_Prescaler = psc;//72-1, 默认会加1，预分频（72M/72=1M，计数一次时间t=1/1M=1us）
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//0, 不进行时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);//定时器方式执行
  TIM_Cmd(TIMx,ENABLE);//定时器外设使能
}

// TIM3中断处理函数
void TIM3_IRQHandler(void) 
{ 
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{ 
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 
	} 
}

// TIM4中断处理函数
void TIM4_IRQHandler(void) 
{ 
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{ 
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 
		TIM4_Count++;
		if(TIM4_Count == 1000)//1000ms测一次速度
		{
			TIM4_Count = 0;
			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13) == Bit_RESET)//点亮状态
				GPIO_SetBits(GPIOC,GPIO_Pin_13); //LED熄灭
			else
				GPIO_ResetBits(GPIOC,GPIO_Pin_13); //LED点亮	
		}
	} 
}

/**************************************************************************************
 * 描  述 : 配置PWM输出
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
// TIM1 CH1-PA8; CH2-PA9; CH3-PA10; CH4_PA11
// TIM2 CH1-PA0; CH2-PA1; CH3-PA2;  CH4_PA3
// TIM3 CH1-PA6; CH2-PA7; CH3-PB0;  CH4_PB1
// TIM4 CH1-PB6; CH2-PB7; CH3-PB8;  CH4_PB9
void PWM_Configuration(TIM_TypeDef* TIMx, u8 Channal)
{
	//PWM配置
	TIM_OCInitTypeDef TIM_OCInitStructure;//PWM定义	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//输出极性为低
	switch(Channal)
	{
		case 1: TIM_OC1Init(TIMx, &TIM_OCInitStructure);//TIM通道1初始化
						TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Enable);//预装载使能，重复执行
		break;
		case 2: TIM_OC2Init(TIMx, &TIM_OCInitStructure);//TIM通道2初始化
						TIM_OC2PreloadConfig(TIMx,TIM_OCPreload_Enable);//预装载使能，重复执行
		break;
		case 3:	TIM_OC3Init(TIMx, &TIM_OCInitStructure);//TIM通道3初始化
						TIM_OC3PreloadConfig(TIMx,TIM_OCPreload_Enable);//预装载使能，重复执行
		break;	
		case 4:	TIM_OC4Init(TIMx, &TIM_OCInitStructure);//TIM通道4初始化
						TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable);//预装载使能，重复执行
		break;
		default: TIM_OC4Init(TIMx, &TIM_OCInitStructure);//TIM通道4初始化
						 TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable);//预装载使能，重复执行	
		break;
	}	
  TIM_Cmd(TIMx,ENABLE);//定时器外设使能
}

/**************************************************************************************
 * 描  述 : 串口配置
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
// USART1 TXD=PA9; RXD=PA10
// USART2 TXD=PA2; RXD=PA3
// USART3 TXD=PB10; RXD=PB11
void USART_Configuration(USART_TypeDef* USARTx, u16 BaudRate)
{
	//USART配置
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = BaudRate; //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;	//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制（DTR与RTS）
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//Tx和Rx使能
	// 初始化USART
	USART_Init(USARTx, &USART_InitStructure);
	USART_ITConfig(USARTx,USART_IT_RXNE,ENABLE); //打开接收中断
	USART_ITConfig(USARTx,USART_IT_IDLE,ENABLE); //打开串口IDLE中断
	USART_Cmd(USARTx,ENABLE); //打开串口使能
	USART_ClearFlag(USARTx,USART_FLAG_TC); //清空发送完成标志位
}

// USART1中断函数
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)//判断当前是否产生中断
	{
		USART_SendData(USART1,USART_ReceiveData(USART1)); //将接收到的数据返回到上位机
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //等待发送完成
	}
}

// USART2中断函数
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)//判断当前是否产生中断
	{
		USART_SendData(USART2,USART_ReceiveData(USART2)); //将接收到的数据发送给USART1
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET); //等待发送完成
	}
}

// USART3中断函数
void USART3_IRQHandler(void)
{
  u8 k = 0;
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)//判断当前是否产生中断
	{
		k = USART_ReceiveData(USART3); //读取串口数据
		USART_SendData(USART3,k); //将接收到的数据返回
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET); //等待发送完成
	}
}

/*******************************************************************************
 * 描  述 : 格式输出函数
 * 入  参 : 无
 * 返回值 : 无
*******************************************************************************/
int fputc(int ch,FILE *f)
{
	USART_SendData(USART1,(u8)ch); //将要发送的数据发送到串口上
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); //等待发送完成
	
	return ch;
}

/**************************************************************************************
 * 描  述 : 配置中断响应优先级
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************************/
void NVIC_Configuration(uint32_t NVIC_PriorityGroup_x, u8 x_IRQn, u8 IRQ_PP, u8 IRQ_SP)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_x);//分到第x组
	NVIC_InitStructure.NVIC_IRQChannel = x_IRQn;//中断类型
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_PP;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = IRQ_SP;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * 描  述 : us延时函数
 * 入  参 : 延时时间
 * 返回值 : 无
*******************************************************************************/
void Delay_us(u32 us)
{
	u32 temp;							   
	SysTick->LOAD = 9*us;//最大值0xFFFFFF=16 777 215；SysTick重装载值寄存器；系统时钟72M，8分频后为9M，1us计数9次
	SysTick->VAL = 0x00;//SysTick当前值寄存器；清空计数器
	SysTick->CTRL = 0x01;//SysTick控制和状态寄存器
	/*
	第0位：ENABLE，Systick使能位（0：关闭；1：开启）
	第1位：TICKINT，Systick中断使能位（0：关闭；1：开启）
	第2位：CLKSOURCE，Systick时钟源选择（0：HCLK/8；1：HCLK）
	第16位：COUNTFLAG，Systick计数比较标志（0：Systick计数到0；1：该位被读取）
	*/
	do
	{
	   temp = SysTick->CTRL;//读取当前倒计数值
	}while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
	SysTick->CTRL = 0x00;//关闭计数器
	SysTick->VAL = 0x00;//清空计数器
}

/*******************************************************************************
 * 描  述 : ms延时函数
 * 入  参 : 延时时间
 * 返回值 : 无
*******************************************************************************/
void Delay_ms(u16 ms)
{
	u32 temp;
	SysTick->LOAD = 9000*ms;//系统时钟72M，8分频后为9M，1us计数9次
	SysTick->VAL = 0x00;//清空计数器
	SysTick->CTRL = 0x01;//SysTick控制和状态寄存器

	do
	{
	   temp = SysTick->CTRL;//读取当前倒计数值
	}while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
	SysTick->CTRL = 0x00;//关闭计数器
	SysTick->VAL = 0x00;//清空计数器
}

/*******************************************************************************
 * 描  述 : 底层初始化
 * 入  参 : 无
 * 返回值 : 无
*******************************************************************************/
void BSP_Initializes(void)
{
	//RCC_HSE_Configuration(); //自定义系统时钟初始化
  SystemInit();//设置系统时钟72MHZ 	
	//GPIOA端口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//端口管脚复用
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/******************************************************************************************/
	//GPIOC端口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//GPIOC引脚配置LED
	GPIO_Configuration(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
	//GPIOB端口使能
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//中断口引脚配置(PB12~PB15)
	//GPIO_Configuration(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_Mode_IPU, GPIO_Speed_50MHz);
	/******************************************************************************************/
	
  //TIM1中断控制使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA端口配置
	//GPIO_Configuration(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM1初始化
	//TIM_Configuration(TIM1, 1000-1, 72-1);
	//TIM1中断优先级配置
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM1_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM2中断控制使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA端口配置
	//GPIO_Configuration(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM2初始化
	//TIM_Configuration(TIM2, 1000-1, 72-1);
	//TIM2中断优先级配置
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM2_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM3中断控制使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOA端口配置
	//GPIO_Configuration(GPIOA, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOB端口配置
	//GPIO_Configuration(GPIOB, GPIO_Pin_0 | GPIO_Pin_1, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM3初始化
	//TIM_Configuration(TIM3, 1000-1, 72-1);
	//TIM3中断优先级配置
	//NVIC_Configuration(NVIC_PriorityGroup_3, TIM3_IRQn, 0, 1); 
	/******************************************************************************************/
	
  //TIM4中断控制使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//GPIOB端口配置
	GPIO_Configuration(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//TIM4初始化
	TIM_Configuration(TIM4, 1000-1, 72-1); //PWM配置周期f=72000000/72/1000=1000Hz,T=1ms
	//TIM4中断优先级配置
	NVIC_Configuration(NVIC_PriorityGroup_3, TIM4_IRQn, 0, 1); 
	/******************************************************************************************/
	
	//PWM_Configuration(TIM4, 1); //配置TIM4的通道1作为PWM1_L输出
	//PWM_Configuration(TIM4, 2); //配置TIM4的通道2作为PWM1_R输出
	//PWM_Configuration(TIM4, 3); //配置TIM4的通道3作为PWM2_L输出
	//PWM_Configuration(TIM4, 4); //配置TIM4的通道4作为PWM2_R输出
	/******************************************************************************************/
	
	//USART1串口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	//GPIOA端口配置(GPIO_Pin_TXD)
	GPIO_Configuration(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOA端口配置(GPIO_Pin_RXD)
	GPIO_Configuration(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	//USART1初始化
	USART_Configuration(USART1, 9600);
	//USART1中断优先级配置
	NVIC_Configuration(NVIC_PriorityGroup_1, USART1_IRQn, 0, 1);
	/******************************************************************************************/
	
	//USART2串口使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//GPIOA端口配置(GPIO_Pin_TXD)
	//GPIO_Configuration(GPIOA, GPIO_Pin_2, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOA端口配置(GPIO_Pin_RXD)
	//GPIO_Configuration(GPIOA, GPIO_Pin_3, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);	
	//USART2初始化
	//USART_Configuration(USART2, 9600);
	//USART2中断优先级配置
	//NVIC_Configuration(NVIC_PriorityGroup_3, USART2_IRQn, 0, 4);
	/******************************************************************************************/
	
	//USART3串口使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
	//GPIOB端口配置(GPIO_Pin_TXD)
	//GPIO_Configuration(GPIOB, GPIO_Pin_10, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIOB端口配置(GPIO_Pin_RXD)
	//GPIO_Configuration(GPIOB, GPIO_Pin_11, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);	
	//USART3初始化
	//USART_Configuration(USART3, 9600);
	//USART3中断优先级配置
	//NVIC_Configuration(NVIC_PriorityGroup_3, USART3_IRQn, 0, 4); 
	/******************************************************************************************/	

	//ADC1使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	//ADC采样频率小于14MHZ
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //(72MHZ)/6=12MHZ 
	//GPIOA端口配置
	GPIO_Configuration(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AIN, GPIO_Speed_50MHz);
	//ADC1初始化
	ADC_Configuration(ADC1);
	//DMA使能
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//DMA初始化
	DMA_Configuration(ADC1);
	/******************************************************************************************/	
	
	//电源管理PWR,BKP使能
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP, ENABLE);
	/******************************************************************************************/	
	
	//DAC时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	//GPIOA端口配置
	GPIO_Configuration(GPIOA, GPIO_Pin_4, GPIO_Mode_AIN, GPIO_Speed_50MHz);
	//DAC初始化
	DAC_Configuration();
	/******************************************************************************************/	

	//SPI时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//GPIO端口配置(SCK浮动输入)
	GPIO_Configuration(GPIOA, GPIO_Pin_5, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIO端口配置(MOSI复用功能)
	GPIO_Configuration(GPIOA, GPIO_Pin_7, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
	//GPIO端口配置(MISO复用功能)
	GPIO_Configuration(GPIOA, GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
	//GPIO端口配置(CS推挽输出)
	GPIO_Configuration(GPIOA, GPIO_Pin_4, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);	
	//SPI初始化
	SPI_Configuration();
	//SPI1中断优先级配置
	NVIC_Configuration(NVIC_PriorityGroup_2, SPI1_IRQn, 1, 1); 
	/******************************************************************************************/	

}
