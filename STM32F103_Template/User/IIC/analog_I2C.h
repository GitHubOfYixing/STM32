#ifndef __ANALOG_I2C_H__
#define __ANALOG_I2C_H__

#include "common.h"

//sbit SCL=P2^6;//IICʱ�����Ŷ���
//ssbit SDA=P2^7;//IIC�������Ŷ���
#define SCL_L GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SCL_H GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define SDA_H GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SDA_Flag GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)

void  Analog_I2C_Configuration(void);
void  IIC_Init(void);//��ʼ��IIC
void  IIC_Start(void);//IIC��ʼ�ź�
void  IIC_Stop(void);//IICֹͣ�ź�
void  IIC_SendACK(u8 ack);//IIC����Ӧ���ź�
u8    IIC_RecACK(void);//IIC����Ӧ���ź�
void  IIC_WriteByte(u8 dat);//��IIC���߷���һ���ֽ�����
u8    IIC_ReadByte(void);//��IIC���߽���һ���ֽ�����
u8    Single_WriteIIC(u8 REG_Add,u8 REG_dat);//��IIC�豸д��һ���ֽ����ݣ�DAת����
u8	  Single_ReadIIC(u8 REG_Add);//��IIC�豸��ȡһ���ֽ����ݣ�ADת����
int   GetData(u8 REG_Address);//�ϳ�����

#endif

