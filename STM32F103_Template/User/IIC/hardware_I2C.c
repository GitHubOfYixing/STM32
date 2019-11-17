#include "hardware_I2C.h"

//IIC�ӿڳ�ʼ��
void I2C_GPIO_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIOB�˿�ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//IIC�˿�ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	//GPIOB��������MPU6050
	GPIO_InitStructure.GPIO_Pin = I2C_SCL | I2C_SDA;//I2C_SCL(PB6) | I2C_SDA(PB7)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//���ÿ�©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
//IIC��ʼ��
void Hardware_I2C_Configuration(void) 
{
	I2C_InitTypeDef I2C_InitStructure;
	I2C_GPIO_Init();
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//����δI2Cģʽ
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = HostAddress; //������ַ���ӻ�������ʹ��
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//����Ӧ��
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//7λ��ַģʽ
	I2C_InitStructure.I2C_ClockSpeed = BusSpeed;//�����ٶ�����
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}
//I2C����һ���ַ���
void I2C_Send_Buffer(u8 SlaveAddr, u8 WriteAddr, u8* pBuffer, u16 NumByteToWrite)
{
	I2C_GenerateSTART(I2C1,ENABLE);//������ʼλ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//���EV5
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//����������ַ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//���EV6
	I2C_SendData(I2C1, WriteAddr);//�ڲ����ܵ�ַ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//��λ�Ĵ����ǿ�
	while(NumByteToWrite--)//ѭ����������
	{
		I2C_SendData(I2C1, *pBuffer);//��������
		pBuffer++;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//���EV8
	}
	I2C_GenerateSTOP(I2C1, ENABLE);//����ֹͣ�ź�
}
//I2C����һ���ֽ�
void I2C_Send_Byte(u8 SlaveAddr, u8 WriteAddr, u8 pBuffer)
{
	I2C_GenerateSTART(I2C1, ENABLE);//���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//�ȴ����
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//����������ַ��״̬
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//�ȴ����
	I2C_SendData(I2C1, WriteAddr);//���������ڲ��Ĵ�����ַ
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//�ȴ����
	I2C_SendData(I2C1, pBuffer);//����Ҫд�������
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){};//�ȴ����
  I2C_GenerateSTOP(I2C1, ENABLE);//����ֹͣ�ź�	
}
//I2C��ȡһ���ַ���
void I2C_Read_Buffer(u8 SlaveAddr, u8 ReadAddr, u8* pBuffer, u16 NumByteToRead)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);//���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//���EV5
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//����������ַ��״̬
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//�ȴ����	
	I2C_Cmd(I2C1, ENABLE);
	I2C_SendData(I2C1, ReadAddr);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);//���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));	
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);//����������ַ��״̬
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//�ȴ����
	while(NumByteToRead)
	{
		if(NumByteToRead == 1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);//�����һ������ʱ�ر�Ӧ��λ
			I2C_GenerateSTOP(I2C1, ENABLE);//����ֹͣ�ź�
		}
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))//��ȡ����
		{
			*pBuffer = I2C_ReceiveData(I2C1);
			pBuffer++;
			NumByteToRead--;
		}
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}
//I2C��ȡһ���ֽ�
u8 I2C_Read_Byte(u8 SlaveAddr, u8 ReadAddr)
{
	u8 a;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);//���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//����������ַ��״̬
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//�ȴ����
	I2C_Cmd(I2C1, ENABLE);
	I2C_SendData(I2C1, ReadAddr);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);//���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);//����������ַ��״̬
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//�ȴ����
	I2C_AcknowledgeConfig(I2C1, DISABLE);//�����һ������ʱ�ر�Ӧ��λ
  I2C_GenerateSTOP(I2C1, ENABLE);//����ֹͣ�ź�
  a = I2C_ReceiveData(I2C1);

  return a;	
}




