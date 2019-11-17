#include "analog_I2C.h"
#include "mpu6050.h"

void Analog_I2C_Configuration(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIOB�˿�ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//GPIOB��������MPU6050
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//PB10(USART2_TXD/I2C2_SDA),PB11(USART2_RXD/I2C2_SCL)
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//��ʼ��IIC
void IIC_Init(void)
{
  SDA_H;
  SCL_H;
}
//IIC��ʼ�ź�(SCLΪ�ߵ�ƽ�ڼ䣬SDA�ɸߵ�ƽ���͵�ƽ����)
void IIC_Start(void)
{
    SCL_H; 
		Delay_us(5);	
	
    SDA_H;        
    Delay_us(5);  //��ʱ����4.7us
    SDA_L;         //�����½���
    Delay_us(5);
	
    SCL_L;         //����ʱ���ߣ�׼�����ͻ��������
		Delay_us(5);	
}
//IICֹͣ�ź�(SCLΪ�ߵ�ƽ�ڼ䣬SDA�ɵ͵�ƽ���ߵ�ƽ����)
void IIC_Stop(void)
{
    SCL_H;
    Delay_us(5);	
	
	  SDA_L;          
    Delay_us(5);   //��ʱ����4.7us
    SDA_H;          //����������
    Delay_us(5); 
}
//IIC����Ӧ���ź�,��ڲ���:ack (0:ACK 1:NAK)
void IIC_SendACK(u8 ack)
{
    SCL_L;        
    Delay_us(5);
	
	  if(ack) SDA_H;
		else SDA_L;    //дӦ���ź�
	  Delay_us(5);
	
    SCL_H;         //SCL��->��
    Delay_us(5); 
    SCL_L;        
    Delay_us(5);
}
//IIC����Ӧ���źţ���ÿ�η�����1�ֽ����ݺ��ж�Ӧ���źţ�
//Ӧ���ź�Ϊ�͵�ƽʱ���涨Ϊ��ЧӦ��λ��ACK�����Ӧ��λ������ʾ�������Ѿ��ɹ��ؽ����˸��ֽ�
//Ӧ���ź�Ϊ�ߵ�ƽʱ���涨Ϊ��Ӧ��λ��NACK����һ���ʾ���������ո��ֽ�û�гɹ�
u8 IIC_RecACK(void)
{
    u8 ack;
	
    SCL_H;
    Delay_us(5);   //��ʱ����4us
	
    ack = SDA_Flag; //��ȡӦ���źţ����SDA_Flag��ȡ��0����Ӧ�𣻷���δӦ��
		
    SCL_L;        
    Delay_us(5); 

    //printf("ACK = %d\r\n",ack);	

    return ack;
}
//��IIC����д��һ���ֽ�����
//IIC���߽������ݴ���ʱ��ʱ���ź�Ϊ�ߵ�ƽ�ڼ䣬�������ϵ����ݱ��뱣���ȶ���
//ֻ����ʱ�����ϵ��ź�Ϊ�͵�ƽ�ڼ䣬�������ϵĸߵ�ƽ��͵�ƽ״̬������仯��
void IIC_WriteByte(u8 dat)
{
		u8 mask;

		for(mask=0x80;mask!=0;mask >>= 1) //��ʼд��1���ֽ�����
		{
			SCL_L;         //ֻ��SCLΪ�͵�ƽ�ڼ䣬SDA���ܱ仯
			Delay_us(5);
			
			if(mask&dat)   //�ȷ������λ�����λΪ1����SDA = 1
				SDA_H;
			else
				SDA_L;
			Delay_us(5);
			
			SCL_H;         //һλ����д����ɣ�SCL = 1�����ݲ��ܱ仯
			Delay_us(5);
		}                  

		//д�����ݣ��ͷ������ߣ�����Ӧ���źŵ��ж�
		SCL_L;
		Delay_us(5);	
}
//��IIC���߶�ȡһ���ֽ�����
u8 IIC_ReadByte(void)
{
    u8 i,dat = 0;
	
		//SCL_H;                // ��SCL�ͣ����ͷ�SDA
		//Delay_us(dt); 
	
    SDA_H;                //ʹ���ڲ�����,׼����ȡ����,
    for(i=0;i<8;i++)      //8λ������
    {
        SCL_H;            
        Delay_us(5);
			
			  dat <<= 1;
        dat |= SDA_Flag;  //�ȶ�ȡ���λ 
			
        SCL_L;           
			  Delay_us(5);    
    }
		
    return dat;
}
//��IIC�豸д��һ���ֽ����ݣ�DAת����
u8 Single_WriteIIC(u8 REG_Add,u8 REG_dat)
{
    //IIC_Init();//IIC��ʼ��
	
    //֪ͨ�����豸��EEPROM�ڶ�д��������д������Device address 1001���̶�λ���� 
    //xxx���仯���豸��ַ000��������8���ⲿ������x�����λΪ1-��ȡ���ݣ����λΪ0-д�����ݣ�
	  IIC_Start();//��ʼ�ź�
	
    IIC_WriteByte(SlaveAddress+0);//���ʹ��豸��ַ+д�ź�
    if(IIC_RecACK()) return 0;//���Ӧ�𲻳ɹ����������ȡ
    IIC_WriteByte(REG_Add);//�ڲ��Ĵ�����ַ
    if(IIC_RecACK()) return 0;
    IIC_WriteByte(REG_dat);//�ڲ��Ĵ�������
    if(IIC_RecACK()) return 0;

    IIC_Stop();//����ֹͣ�ź�
	
		return 0;
}
//��IIC�豸��ȡһ���ֽ�����
u8 Single_ReadIIC(u8 REG_Add)
{
		u8 REG_dat;
		//IIC_Init();//IIC��ʼ��
	
		IIC_Start();//��ʼ�ź�
	
		IIC_WriteByte(SlaveAddress+0);//���ʹ��豸��ַ+д�ź�;֪ͨ�����豸��������Ҫ�Դӻ����в���
		if(IIC_RecACK()) return 0;//дӦ���ж�
		//����д����Ķ�Ӧ��ַ
	  IIC_WriteByte(REG_Add);//���ʹ洢��Ԫ��ַ����0��ʼ;(0x00,0x40,0x80,0xc0)ȷ����Ҫ���������ݵĴ洢��ַ	
		if(IIC_RecACK()) return 0;
		
	  IIC_Start();//��ʼ�ź�
	
		IIC_WriteByte(SlaveAddress+1);//���ʹ��豸��ַ+���ź�;���λΪ1�����ж�����
		if(IIC_RecACK()) return 0;//��Ӧ���ж�

		REG_dat=IIC_ReadByte();//��ȡһ���ֽ�����	
	  //IIC_SendACK(1);//���Ͷ�ȡ���Ӧ���ź�
	
		IIC_Stop();//����ֹͣ�ź�
		
		return REG_dat;
}


