#ifndef __ANALOG_I2C_H__
#define __ANALOG_I2C_H__

#include "common.h"

//sbit SCL=P2^6;//IIC时钟引脚定义
//ssbit SDA=P2^7;//IIC数据引脚定义
#define SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SDA_Flag GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)

void  Analog_I2C_Configuration(void);
void  IIC_Init(void);//初始化IIC
void  IIC_Start(void);//IIC起始信号
void  IIC_Stop(void);//IIC停止信号
void  IIC_SendACK(u8 ack);//IIC发送应答信号
u8    IIC_RecACK(void);//IIC接收应答信号
void  IIC_WriteByte(u8 dat);//向IIC总线发送一个字节数据
u8    IIC_ReadByte(void);//从IIC总线接收一个字节数据
u8    Single_WriteIIC(u8 REG_Add,u8 REG_dat);//向IIC设备写入一个字节数据（DA转换）
u8	  Single_ReadIIC(u8 REG_Add);//从IIC设备读取一个字节数据（AD转换）
int   GetData(u8 REG_Address);//合成数据

#endif

