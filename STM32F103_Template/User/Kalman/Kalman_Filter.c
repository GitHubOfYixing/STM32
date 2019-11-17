#include "Kalman_Filter.h"
#include "mpu6050.h"
  
//卡尔曼滤波参数与函数  
float dt = 0.1;//注意：dt的取值为kalman滤波器采样时间  
float angle, angle_dot;//角度和角速度  
float Q_angle=0.001, Q_gyro=0.003; //角度数据置信度,角速度数据置信度  
float R_angle=0.5 ,C_0 = 1;  
float q_bias, angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;  
float Pdot[4] = {0,0,0,0}; 
float P[2][2] = {{1, 0}, {0, 1}};  

//卡尔曼滤波  
float Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy   
{  
		angle += (gyro_m - q_bias) * dt; //先验估计 
	
		Pdot[0] = Q_angle - P[0][1] - P[1][0]; // Pk-先验估计误差协方差的微分 
		Pdot[1] = -P[1][1];  
		Pdot[2] = -P[1][1];  
    Pdot[3] = Q_gyro;  

    P[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分   
		P[0][1] += Pdot[1] * dt; // =先验估计误差协方差   
		P[1][0] += Pdot[2] * dt;  
		P[1][1] += Pdot[3] * dt; 

		angle_err = angle_m - angle;  	
		//zk-先验估计 
	
		PCt_0 = C_0 * P[0][0];  
		PCt_1 = C_0 * P[1][0];  

		E = R_angle + C_0 * PCt_0;  

		K_0 = PCt_0 / E;  
		K_1 = PCt_1 / E;  

		t_0 = PCt_0;  
		t_1 = C_0 * P[0][1];  

		//后验估计误差协方差
		P[0][0] -= K_0 * t_0;  
		P[0][1] -= K_0 * t_1;  
		P[1][0] -= K_1 * t_0;  
		P[1][1] -= K_1 * t_1;  

		angle += K_0 * angle_err; //最优角度  
		//后验估计  
		q_bias += K_1 * angle_err;  
    angle_dot = gyro_m - q_bias;//最优角速度  
		
		//输出值(后验估计)的微分=角速度 
    return angle;
}  




