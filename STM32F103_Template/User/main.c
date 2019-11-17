#include "common.h"
#include "hardware_I2C.h"
#include "analog_I2C.h"

/*
 * 函数名：main
 * 描述  ：主函数
 */
int main(void)
{
	//底层初始化
	BSP_Initializes();
	
	//读写后备区域数据
	//BKP_ReadWrite(); 
	/******************************************************************************************/
	
	u32 data = 123456789;
	//FLASH解锁
	FLASH_Unlock();
	//清除标志位
	FLASH_ClearFlag(FLASH_FLAG_BSY| FLASH_FLAG_EOP| FLASH_FLAG_PGERR| FLASH_FLAG_WRPRTERR| FLASH_FLAG_BSY);
	//擦除页的起始地址
	#define FLASH_ADDR 0x0801FC00 //中容量产品地址
	FLASH_ErasePage(FLASH_ADDR);
	//从起始地址开始，写入一个字（4byte）数据
	FLASH_ProgramWord(FLASH_ADDR,data);
	//锁定FLASH
	FLASH_Lock();
	data = 0;
	data = (*(volatile int*)(FLASH_ADDR));//32位用int，16位用short，8位用char
  printf("data = %d\r\n",(int)data);
	/******************************************************************************************/

	//关于I2C应答，最好不使用死循环检测，容易造成死机
	//Hardware_I2C_Configuration(); //硬件I2C初始化
	//Hardware_I2C_MPU6050_Init(); //MPU6050初始化	
	//Analog_I2C_Configuration();
	//Analog_I2C_MPU6050_Init();
	/******************************************************************************************/	
	printf("Hello World...\r\n");
	Delay_ms(1000);
	
	/* SPI变量 */
	volatile uint8_t  gSPI_RxBuf[10];                          //SPI接收BUF
	volatile uint8_t  gSPI_FlagOver = FLAG_N_VALID;            //SPI溢出标志
	volatile uint16_t gSPI_Cnt = 0; 
	
	//uint8_t  read_buf[6];
	//SFLASH_WriteNByte((uint8_t*)"ABCDEF", 0, 6);   //从地址0，连续写入6字节数据(ABCDEF)
  //TIMDelay_Nms(500);
  //SFLASH_ReadNByte(read_buf, 0, 6);              //从地址0，连续读出6字节数据
	
	while(1)
	{
		//LED_Blink();// D1灯(PC13)
		
		//printf("BKP Test..\r\n");
    //Delay_ms(500);
		
		printf("DAC输出电压: 1.5V\r\n"); 
		DAC_OutVoltage(1.5); 
		Delay_ms(500);
		
		printf("PA1电压值: ");
		Get_ADC(ADC1, ADC_Data[0]);
		Delay_ms(500);
		
		if(FLAG_VALID == gSPI_FlagOver) //等待接收有效
    {
      gSPI_FlagOver = FLAG_N_VALID;     
      LED_Blink(); //LED开关(亮灭变化)
			// USART1发送N字节字符
			for(int i=0;i<10;i++)
				printf("%d",gSPI_RxBuf[i]); //串口发送SPI收到的数据
			printf("\r\n");
    }
		
		/*
		for(uint8_t cnt=0; cnt<10; cnt++)
    {
      SPI_WriteReadByte('0' + cnt); //主机发送10字节数据
      Delay_ms(100);
    }
		*/
		
		/*
		printf("PA2电压值: ");
		Get_ADC(ADC1, ADC_Data[1]);
		Delay_ms(500);
		printf("PA3电压值: ");
		Get_ADC(ADC1, ADC_Data[2]);
		Delay_ms(500);
		*/
	}
}

