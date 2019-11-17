#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "common.h"

#define	WHO_AM_I        0x75 //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)  
#define	SlaveAddress	  0xD0 //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

extern double mpu6050_dat[7];
extern double angleAx;
extern double gyroGy;

//****************************************
//����MPU6050�ڲ���ַ
//**************************�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	SMPLRT_DIV		0x19 
//**************************��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	CONFIG			  0x1A 
//**************************�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	GYRO_CONFIG		0x1B
//**************************���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz) 
#define	ACCEL_CONFIG	0x1C 
//**************************��Դ��������ֵ��0x00(��������)
#define	PWR_MGMT_1		0x6B

//x����ٶ�
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
//y����ٶ�
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
//z����ٶ�
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
//�¶�
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
//x����ٶ�
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
//y����ٶ�	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
//z����ٶ�
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

//Ӳ��I2C
int   Hardware_I2C_GetData(u8 REG_Add);//���ٶ�����ٶȵ�ַ
void  Hardware_I2C_MPU6050_Init(void);//��ʼ��MPU6050
double*  Hardware_I2C_Get_MPU6050_Data(void);//��ȡMPU6050����
//ģ��I2C
int   Analog_I2C_GetData(u8 REG_Add);//���ٶ�����ٶȵ�ַ
void  Analog_I2C_MPU6050_Init(void);//��ʼ��MPU6050
double*  Analog_I2C_Get_MPU6050_Data(void);//��ȡMPU6050����

#endif

