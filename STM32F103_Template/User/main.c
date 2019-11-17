#include "common.h"
#include "hardware_I2C.h"
#include "analog_I2C.h"

/*
 * ��������main
 * ����  ��������
 */
int main(void)
{
	//�ײ��ʼ��
	BSP_Initializes();
	
	//��д����������
	//BKP_ReadWrite(); 
	/******************************************************************************************/
	
	u32 data = 123456789;
	//FLASH����
	FLASH_Unlock();
	//�����־λ
	FLASH_ClearFlag(FLASH_FLAG_BSY| FLASH_FLAG_EOP| FLASH_FLAG_PGERR| FLASH_FLAG_WRPRTERR| FLASH_FLAG_BSY);
	//����ҳ����ʼ��ַ
	#define FLASH_ADDR 0x0801FC00 //��������Ʒ��ַ
	FLASH_ErasePage(FLASH_ADDR);
	//����ʼ��ַ��ʼ��д��һ���֣�4byte������
	FLASH_ProgramWord(FLASH_ADDR,data);
	//����FLASH
	FLASH_Lock();
	data = 0;
	data = (*(volatile int*)(FLASH_ADDR));//32λ��int��16λ��short��8λ��char
  printf("data = %d\r\n",(int)data);
	/******************************************************************************************/

	//����I2CӦ����ò�ʹ����ѭ����⣬�����������
	//Hardware_I2C_Configuration(); //Ӳ��I2C��ʼ��
	//Hardware_I2C_MPU6050_Init(); //MPU6050��ʼ��	
	//Analog_I2C_Configuration();
	//Analog_I2C_MPU6050_Init();
	/******************************************************************************************/	
	printf("Hello World...\r\n");
	Delay_ms(1000);
	
	/* SPI���� */
	volatile uint8_t  gSPI_RxBuf[10];                          //SPI����BUF
	volatile uint8_t  gSPI_FlagOver = FLAG_N_VALID;            //SPI�����־
	volatile uint16_t gSPI_Cnt = 0; 
	
	//uint8_t  read_buf[6];
	//SFLASH_WriteNByte((uint8_t*)"ABCDEF", 0, 6);   //�ӵ�ַ0������д��6�ֽ�����(ABCDEF)
  //TIMDelay_Nms(500);
  //SFLASH_ReadNByte(read_buf, 0, 6);              //�ӵ�ַ0����������6�ֽ�����
	
	while(1)
	{
		//LED_Blink();// D1��(PC13)
		
		//printf("BKP Test..\r\n");
    //Delay_ms(500);
		
		printf("DAC�����ѹ: 1.5V\r\n"); 
		DAC_OutVoltage(1.5); 
		Delay_ms(500);
		
		printf("PA1��ѹֵ: ");
		Get_ADC(ADC1, ADC_Data[0]);
		Delay_ms(500);
		
		if(FLAG_VALID == gSPI_FlagOver) //�ȴ�������Ч
    {
      gSPI_FlagOver = FLAG_N_VALID;     
      LED_Blink(); //LED����(����仯)
			// USART1����N�ֽ��ַ�
			for(int i=0;i<10;i++)
				printf("%d",gSPI_RxBuf[i]); //���ڷ���SPI�յ�������
			printf("\r\n");
    }
		
		/*
		for(uint8_t cnt=0; cnt<10; cnt++)
    {
      SPI_WriteReadByte('0' + cnt); //��������10�ֽ�����
      Delay_ms(100);
    }
		*/
		
		/*
		printf("PA2��ѹֵ: ");
		Get_ADC(ADC1, ADC_Data[1]);
		Delay_ms(500);
		printf("PA3��ѹֵ: ");
		Get_ADC(ADC1, ADC_Data[2]);
		Delay_ms(500);
		*/
	}
}

