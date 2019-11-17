#include "hardware_I2C.h"

//IIC接口初始化
void I2C_GPIO_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIOB端口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//IIC端口使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	//GPIOB引脚配置MPU6050
	GPIO_InitStructure.GPIO_Pin = I2C_SCL | I2C_SDA;//I2C_SCL(PB6) | I2C_SDA(PB7)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//复用开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
//IIC初始化
void Hardware_I2C_Configuration(void) 
{
	I2C_InitTypeDef I2C_InitStructure;
	I2C_GPIO_Init();
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//设置未I2C模式
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = HostAddress; //主机地址，从机不允许使用
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//允许应答
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//7位地址模式
	I2C_InitStructure.I2C_ClockSpeed = BusSpeed;//总线速度设置
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}
//I2C发送一个字符串
void I2C_Send_Buffer(u8 SlaveAddr, u8 WriteAddr, u8* pBuffer, u16 NumByteToWrite)
{
	I2C_GenerateSTART(I2C1,ENABLE);//产生起始位
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//发送器件地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//清除EV6
	I2C_SendData(I2C1, WriteAddr);//内部功能地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//移位寄存器非空
	while(NumByteToWrite--)//循环发送数据
	{
		I2C_SendData(I2C1, *pBuffer);//发送数据
		pBuffer++;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//清除EV8
	}
	I2C_GenerateSTOP(I2C1, ENABLE);//产生停止信号
}
//I2C发送一个字节
void I2C_Send_Byte(u8 SlaveAddr, u8 WriteAddr, u8 pBuffer)
{
	I2C_GenerateSTART(I2C1, ENABLE);//发送开始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//等待完成
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//发送器件地址及状态
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待完成
	I2C_SendData(I2C1, WriteAddr);//发送器件内部寄存器地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待完成
	I2C_SendData(I2C1, pBuffer);//发送要写入的内容
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){};//等待完成
  I2C_GenerateSTOP(I2C1, ENABLE);//产生停止信号	
}
//I2C读取一个字符串
void I2C_Read_Buffer(u8 SlaveAddr, u8 ReadAddr, u8* pBuffer, u16 NumByteToRead)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);//发送开始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//清除EV5
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//发送器件地址及状态
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待完成	
	I2C_Cmd(I2C1, ENABLE);
	I2C_SendData(I2C1, ReadAddr);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);//发送开始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));	
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);//发送器件地址及状态
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//等待完成
	while(NumByteToRead)
	{
		if(NumByteToRead == 1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);//最后有一个数据时关闭应答位
			I2C_GenerateSTOP(I2C1, ENABLE);//产生停止信号
		}
		if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))//读取数据
		{
			*pBuffer = I2C_ReceiveData(I2C1);
			pBuffer++;
			NumByteToRead--;
		}
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}
//I2C读取一个字节
u8 I2C_Read_Byte(u8 SlaveAddr, u8 ReadAddr)
{
	u8 a;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);//发送开始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);//发送器件地址及状态
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待完成
	I2C_Cmd(I2C1, ENABLE);
	I2C_SendData(I2C1, ReadAddr);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);//发送开始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);//发送器件地址及状态
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//等待完成
	I2C_AcknowledgeConfig(I2C1, DISABLE);//最后有一个数据时关闭应答位
  I2C_GenerateSTOP(I2C1, ENABLE);//产生停止信号
  a = I2C_ReceiveData(I2C1);

  return a;	
}




