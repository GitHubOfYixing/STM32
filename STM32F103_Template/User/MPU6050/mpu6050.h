#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "common.h"

#define	WHO_AM_I        0x75 //IIC地址寄存器(默认数值0x68，只读)  
#define	SlaveAddress	  0xD0 //IIC写入时的地址字节数据，+1为读取

extern double mpu6050_dat[7];
extern double angleAx;
extern double gyroGy;

//****************************************
//定义MPU6050内部地址
//**************************陀螺仪采样率，典型值：0x07(125Hz)
#define	SMPLRT_DIV		0x19 
//**************************低通滤波频率，典型值：0x06(5Hz)
#define	CONFIG			  0x1A 
//**************************陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	GYRO_CONFIG		0x1B
//**************************加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) 
#define	ACCEL_CONFIG	0x1C 
//**************************电源管理，典型值：0x00(正常启用)
#define	PWR_MGMT_1		0x6B

//x轴加速度
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
//y轴加速度
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
//z轴加速度
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
//温度
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
//x轴角速度
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
//y轴角速度	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
//z轴角速度
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

//硬件I2C
int   Hardware_I2C_GetData(u8 REG_Add);//加速度与角速度地址
void  Hardware_I2C_MPU6050_Init(void);//初始化MPU6050
double*  Hardware_I2C_Get_MPU6050_Data(void);//获取MPU6050数据
//模拟I2C
int   Analog_I2C_GetData(u8 REG_Add);//加速度与角速度地址
void  Analog_I2C_MPU6050_Init(void);//初始化MPU6050
double*  Analog_I2C_Get_MPU6050_Data(void);//获取MPU6050数据

#endif

