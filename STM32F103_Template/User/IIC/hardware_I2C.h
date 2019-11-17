#ifndef __HARDWARE_I2C_H__
#define __HARDWARE_I2C_H__

#include "common.h"

#define I2C_PORT GPIOB
#define I2C_SCL GPIO_Pin_6
#define I2C_SDA GPIO_Pin_7

#define HostAddress 0xc0  //��������������ַ
#define BusSpeed 200000 //�����ٶȣ�<400000��

void Hardware_I2C_Configuration(void);
void I2C_Send_Buffer(u8 SlaveAddr, u8 WriteAddr, u8* pBuffer, u16 NumByteToWrite);
void I2C_Send_Byte(u8 SlaveAddr, u8 WriteAddr, u8 pBuffer);
void I2C_Read_Buffer(u8 SlaveAddr, u8 ReadAddr, u8* pBuffer, u16 NumByteToRead);
u8   I2C_Read_Byte(u8 SlaveAddr, u8 ReadAddr);

#endif
