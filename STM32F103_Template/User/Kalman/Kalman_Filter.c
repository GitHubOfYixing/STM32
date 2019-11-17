#include "Kalman_Filter.h"
#include "mpu6050.h"
  
//�������˲������뺯��  
float dt = 0.1;//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��  
float angle, angle_dot;//�ǶȺͽ��ٶ�  
float Q_angle=0.001, Q_gyro=0.003; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�  
float R_angle=0.5 ,C_0 = 1;  
float q_bias, angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;  
float Pdot[4] = {0,0,0,0}; 
float P[2][2] = {{1, 0}, {0, 1}};  

//�������˲�  
float Kalman_Filter(float angle_m, float gyro_m)//angleAx �� gyroGy   
{  
		angle += (gyro_m - q_bias) * dt; //������� 
	
		Pdot[0] = Q_angle - P[0][1] - P[1][0]; // Pk-����������Э�����΢�� 
		Pdot[1] = -P[1][1];  
		Pdot[2] = -P[1][1];  
    Pdot[3] = Q_gyro;  

    P[0][0] += Pdot[0] * dt; // Pk-����������Э����΢�ֵĻ���   
		P[0][1] += Pdot[1] * dt; // =����������Э����   
		P[1][0] += Pdot[2] * dt;  
		P[1][1] += Pdot[3] * dt; 

		angle_err = angle_m - angle;  	
		//zk-������� 
	
		PCt_0 = C_0 * P[0][0];  
		PCt_1 = C_0 * P[1][0];  

		E = R_angle + C_0 * PCt_0;  

		K_0 = PCt_0 / E;  
		K_1 = PCt_1 / E;  

		t_0 = PCt_0;  
		t_1 = C_0 * P[0][1];  

		//����������Э����
		P[0][0] -= K_0 * t_0;  
		P[0][1] -= K_0 * t_1;  
		P[1][0] -= K_1 * t_0;  
		P[1][1] -= K_1 * t_1;  

		angle += K_0 * angle_err; //���ŽǶ�  
		//�������  
		q_bias += K_1 * angle_err;  
    angle_dot = gyro_m - q_bias;//���Ž��ٶ�  
		
		//���ֵ(�������)��΢��=���ٶ� 
    return angle;
}  




