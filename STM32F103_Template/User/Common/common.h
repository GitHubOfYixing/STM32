#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32f10x.h" //包含所有的头文件
#include "stdio.h"
#include "string.h"
#include "math.h"

#define PI 3.1415926

#define ADC_Data_Size 3

extern uint16_t ADC_Data[ADC_Data_Size];

/* 标志位 */
#define FLAG_N_VALID              0xFF  //标志无效
#define FLAG_VALID                'V'  
/* SPI变量 */
extern volatile uint8_t  gSPI_RxBuf[10]; //SPI接收BUF
extern volatile uint8_t  gSPI_FlagOver;  //SPI溢出标志
extern volatile uint16_t gSPI_Cnt;       //SPI计数


void BKP_ReadWrite(void);
void SPI_Configuration(void);
uint8_t SPI_WriteReadByte(uint8_t TxData);
void GPIO_Configuration(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef GPIO_Mode_x, GPIOSpeed_TypeDef GPIO_Speed_x);
void LED_Blink(void);
void ADC_Configuration(ADC_TypeDef * ADCx);
void DMA_Configuration(ADC_TypeDef * ADCx);
void DAC_Configuration(void);
void DAC_OutVoltage(float Voltage);
void Get_ADC(ADC_TypeDef * ADCx, uint32_t DRValue);
void TIM_Configuration(TIM_TypeDef* TIMx, u16 arr,u16 psc);
void PWM_Configuration(TIM_TypeDef* TIMx, u8 Channal);
void USART_Configuration(USART_TypeDef* USARTx, u16 BaudRate);
void EXTI_Configuration(void);
void NVIC_Configuration(uint32_t NVIC_PriorityGroup_x, u8 x_IRQn, u8 IRQ_PP, u8 IRQ_SP);
void Delay_us(u32 us);
void Delay_ms(u16 ms);
void BSP_Initializes(void);

#endif
