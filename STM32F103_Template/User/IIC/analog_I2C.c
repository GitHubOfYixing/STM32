#include "analog_I2C.h"
#include "mpu6050.h"

void Analog_I2C_Configuration(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIOB端口使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//GPIOB引脚配置MPU6050
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//PB10(USART2_TXD/I2C2_SDA),PB11(USART2_RXD/I2C2_SCL)
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//初始化IIC
void IIC_Init(void)
{
  SDA_H;
  SCL_H;
}
//IIC起始信号(SCL为高电平期间，SDA由高电平往低电平跳变)
void IIC_Start(void)
{
    SCL_H; 
		Delay_us(5);	
	
    SDA_H;        
    Delay_us(5);  //延时大于4.7us
    SDA_L;         //产生下降沿
    Delay_us(5);
	
    SCL_L;         //拉低时钟线，准备发送或接收数据
		Delay_us(5);	
}
//IIC停止信号(SCL为高电平期间，SDA由低电平往高电平跳变)
void IIC_Stop(void)
{
    SCL_H;
    Delay_us(5);	
	
	  SDA_L;          
    Delay_us(5);   //延时大于4.7us
    SDA_H;          //产生上升沿
    Delay_us(5); 
}
//IIC发送应答信号,入口参数:ack (0:ACK 1:NAK)
void IIC_SendACK(u8 ack)
{
    SCL_L;        
    Delay_us(5);
	
	  if(ack) SDA_H;
		else SDA_L;    //写应答信号
	  Delay_us(5);
	
    SCL_H;         //SCL高->低
    Delay_us(5); 
    SCL_L;        
    Delay_us(5);
}
//IIC接收应答信号（在每次发送完1字节数据后判断应答信号）
//应答信号为低电平时，规定为有效应答位（ACK，简称应答位），表示接收器已经成功地接收了该字节
//应答信号为高电平时，规定为非应答位（NACK），一般表示接收器接收该字节没有成功
u8 IIC_RecACK(void)
{
    u8 ack;
	
    SCL_H;
    Delay_us(5);   //延时大于4us
	
    ack = SDA_Flag; //读取应答信号；如果SDA_Flag读取到0，则应答；否者未应答
		
    SCL_L;        
    Delay_us(5); 

    //printf("ACK = %d\r\n",ack);	

    return ack;
}
//向IIC总线写入一个字节数据
//IIC总线进行数据传送时，时钟信号为高电平期间，数据线上的数据必须保持稳定；
//只有在时钟线上的信号为低电平期间，数据线上的高电平或低电平状态才允许变化。
void IIC_WriteByte(u8 dat)
{
		u8 mask;

		for(mask=0x80;mask!=0;mask >>= 1) //开始写入1个字节数据
		{
			SCL_L;         //只有SCL为低电平期间，SDA才能变化
			Delay_us(5);
			
			if(mask&dat)   //先发送最高位，最高位为1，则SDA = 1
				SDA_H;
			else
				SDA_L;
			Delay_us(5);
			
			SCL_H;         //一位数据写入完成，SCL = 1，数据不能变化
			Delay_us(5);
		}                  

		//写完数据，释放数据线，进行应答信号的判断
		SCL_L;
		Delay_us(5);	
}
//从IIC总线读取一个字节数据
u8 IIC_ReadByte(void)
{
    u8 i,dat = 0;
	
		//SCL_H;                // 先SCL低，再释放SDA
		//Delay_us(dt); 
	
    SDA_H;                //使能内部上拉,准备读取数据,
    for(i=0;i<8;i++)      //8位计数器
    {
        SCL_H;            
        Delay_us(5);
			
			  dat <<= 1;
        dat |= SDA_Flag;  //先读取最高位 
			
        SCL_L;           
			  Delay_us(5);    
    }
		
    return dat;
}
//向IIC设备写入一个字节数据（DA转换）
u8 Single_WriteIIC(u8 REG_Add,u8 REG_dat)
{
    //IIC_Init();//IIC初始化
	
    //通知接收设备：EEPROM在读写方向，启动写操作（Device address 1001（固定位）） 
    //xxx（变化，设备地址000，最多控制8个外部器件）x（最低位为1-读取数据，最低位为0-写入数据）
	  IIC_Start();//起始信号
	
    IIC_WriteByte(SlaveAddress+0);//发送从设备地址+写信号
    if(IIC_RecACK()) return 0;//如果应答不成功，则结束读取
    IIC_WriteByte(REG_Add);//内部寄存器地址
    if(IIC_RecACK()) return 0;
    IIC_WriteByte(REG_dat);//内部寄存器数据
    if(IIC_RecACK()) return 0;

    IIC_Stop();//发送停止信号
	
		return 0;
}
//从IIC设备读取一个字节数据
u8 Single_ReadIIC(u8 REG_Add)
{
		u8 REG_dat;
		//IIC_Init();//IIC初始化
	
		IIC_Start();//起始信号
	
		IIC_WriteByte(SlaveAddress+0);//发送从设备地址+写信号;通知接收设备：主机将要对从机进行操作
		if(IIC_RecACK()) return 0;//写应答判断
		//发送写命令的对应地址
	  IIC_WriteByte(REG_Add);//发送存储单元地址，从0开始;(0x00,0x40,0x80,0xc0)确定将要操作的数据的存储地址	
		if(IIC_RecACK()) return 0;
		
	  IIC_Start();//起始信号
	
		IIC_WriteByte(SlaveAddress+1);//发送从设备地址+读信号;最低位为1，进行读操作
		if(IIC_RecACK()) return 0;//读应答判断

		REG_dat=IIC_ReadByte();//读取一个字节数据	
	  //IIC_SendACK(1);//发送读取完成应答信号
	
		IIC_Stop();//发送停止信号
		
		return REG_dat;
}


