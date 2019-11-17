#include "mpu6050.h"
#include "analog_I2C.h"
#include "hardware_I2C.h"

double mpu6050_dat[7];
double angleAx;
double gyroGy;

//Hardware_I2C合成数据
int Hardware_I2C_GetData(u8 REG_Add)//加速度与角速度地址
{
		u8 H,L;
	  H = I2C_Read_Byte(SlaveAddress, REG_Add);
		L = I2C_Read_Byte(SlaveAddress, REG_Add+1);
	
		return (H<<8)+L;   //合成数据
}
//Hardware_I2C初始化MPU6050
void Hardware_I2C_MPU6050_Init(void)
{
	I2C_Send_Byte(SlaveAddress, PWR_MGMT_1, 0x80);    //复位MPU6050
	Delay_ms(100);
	I2C_Send_Byte(SlaveAddress, PWR_MGMT_1, 0x00);    //解除休眠状态，唤醒MPU6050
	I2C_Send_Byte(SlaveAddress, SMPLRT_DIV, 0x07);    //陀螺仪采样率
	I2C_Send_Byte(SlaveAddress, CONFIG, 0x06);        //低通滤波频率
	I2C_Send_Byte(SlaveAddress, GYRO_CONFIG, 0x18);   //250deg/s陀螺仪自检及测量范围
	I2C_Send_Byte(SlaveAddress, ACCEL_CONFIG, 0x01);  //2g加速计自检、测量范围及高通滤波频率
}
//Hardware_I2C获取MPU6050数据
double* Hardware_I2C_Get_MPU6050_Data(void)
{
		int AX,AY,AZ,GX,GY,GZ,T;
	  int k;
	
		AX = Hardware_I2C_GetData(ACCEL_XOUT_H);//显示X轴加速度（2g）
		mpu6050_dat[0] = (double)AX*2/32768;
		AY = Hardware_I2C_GetData(ACCEL_YOUT_H);//显示Y轴加速度
		mpu6050_dat[1] = (double)AY*2/32768;
		AZ = Hardware_I2C_GetData(ACCEL_ZOUT_H);//显示Z轴加速度
		mpu6050_dat[2] = (double)AZ*2/32768;
		GX = Hardware_I2C_GetData(GYRO_XOUT_H);//显示X轴角速度（250deg/s）
		mpu6050_dat[3] = (double)GX*250/32768;
		GY = Hardware_I2C_GetData(GYRO_YOUT_H);//显示Y轴角速度
		mpu6050_dat[4] = (double)GY*250/32768;
		GZ = Hardware_I2C_GetData(GYRO_ZOUT_H);//显示Z轴角速度
		mpu6050_dat[5] = (double)GZ*250/32768;
		T = Hardware_I2C_GetData(TEMP_OUT_H);//显示温度
		mpu6050_dat[6] = (double)T/340 + 36.53;
	
	  for(k=0; k<7; k++)
	  {
			printf("%.1lf  ",mpu6050_dat[k]);
			if(k==6) printf("\r\n");
		}
	
	  return mpu6050_dat;
}

//Analog_I2C合成数据
int Analog_I2C_GetData(u8 REG_Add)//加速度与角速度地址
{
		char H,L;
		H=Single_ReadIIC(REG_Add);
		L=Single_ReadIIC(REG_Add+1);
	
		return (H<<8)+L;   //合成数据
}
//Analog_I2C初始化MPU6050
void Analog_I2C_MPU6050_Init(void)
{
	  Single_WriteIIC(PWR_MGMT_1, 0x80);    //复位MPU6050
	  Delay_ms(100);
	  Single_WriteIIC(PWR_MGMT_1, 0x00);    //解除休眠状态，唤醒MPU6050
	  Single_WriteIIC(SMPLRT_DIV, 0x07);    //陀螺仪采样率
	  Single_WriteIIC(CONFIG, 0x06);        //低通滤波频率
	  Single_WriteIIC(GYRO_CONFIG, 0x00);   //250deg/s陀螺仪自检及测量范围
	  Single_WriteIIC(ACCEL_CONFIG, 0x00);  //2g加速计自检、测量范围及高通滤波频率
}
//Analog_I2C获取MPU6050数据
double* Analog_I2C_Get_MPU6050_Data(void)
{
		int AX,AY,AZ,GX,GY,GZ,T;
	  double ax,ay,az;
	
	  AX = Analog_I2C_GetData(ACCEL_XOUT_H); if(AX&0x8000) AX = AX-65535;
	  AY = Analog_I2C_GetData(ACCEL_YOUT_H); if(AY&0x8000) AY = AY-65535;	
	  AZ = Analog_I2C_GetData(ACCEL_ZOUT_H); if(AZ&0x8000) AZ = AZ-65535;
	  //静态加速度
		ax = (double)2*AX/32768;//显示X轴加速度	
	  ay = (double)2*AY/32768;//显示Y轴加速度
	  az = (double)2*AZ/32768;//显示Z轴加速度	
	  //动态加速度，a为小车的运动加速度
//		ax = ax + a*cos(beta_x);//beta_x为小车与yz平面夹角	
//	  ay = ay + a*cos(beta_y);//beta_y为小车与xz平面夹角
//	  az = ay + a*cos(beta_z);//beta_z为小车与xy平面夹角	
//	  a = sqrt(ax*ax+ay*ay+az*az);
		    
//		mpu6050_dat[0] = (asin(ax/a)*180/PI);//显示X轴加速度
//		mpu6050_dat[1] = (asin(ay/a)*180/PI);//显示Y轴加速度
//		mpu6050_dat[2] = (asin(az/a)*180/PI);//显示Z轴加速度
/*********************************************************/
//		mpu6050_dat[0] = (ax/fabs(ax))*(acos(ax/a)*180/PI);//显示X轴加速度
//		mpu6050_dat[1] = (ay/fabs(ay))*(acos(ay/a)*180/PI);//显示Y轴加速度
//		mpu6050_dat[2] = (az/fabs(az))*(acos(az/a)*180/PI);//显示Z轴加速度
/*********************************************************/
		mpu6050_dat[0] = ax;//显示X轴加速度
		mpu6050_dat[1] = ay;//显示Y轴加速度
		mpu6050_dat[2] = az;//显示Z轴加速度
	  
		angleAx = atan2(ax,az)*180/PI;
		
	  /****/
		GX = Analog_I2C_GetData(GYRO_XOUT_H); if(GX&0x8000) GX = GX-65535;
		GY = Analog_I2C_GetData(GYRO_YOUT_H); if(GY&0x8000) GY = GY-65535;
		GZ = Analog_I2C_GetData(GYRO_ZOUT_H); if(GZ&0x8000) GZ = GZ-65535; 
		mpu6050_dat[3] = (double)250*GX/32768; //显示X轴角速度 		
		mpu6050_dat[4] = (double)250*GY/32768; //显示Y轴角速度
		mpu6050_dat[5] = (double)250*GZ/32768; //显示Z轴角速度	
		//对角速度进行积分得到角度（不受小车运动影响）
		
		gyroGy = mpu6050_dat[4];
		
		T = Analog_I2C_GetData(TEMP_OUT_H);//显示温度
		mpu6050_dat[6] = (double)(T-65535)/340 + 36.65;
					
	  return mpu6050_dat;
}
//DMP数据输出


