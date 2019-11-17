#include "can.h"

u8  CAN1_Configuration(void) //CAN��ʼ��������0��ʾ���óɹ�������������ʾʧ�ܣ�
{
    GPIO_InitTypeDef        GPIO_InitStructure; 
    CAN_InitTypeDef         CAN_InitStructure;
    CAN_FilterInitTypeDef   CAN_FilterInitStructure;

		#if CAN_INT_ENABLE
				NVIC_InitTypeDef        NVIC_InitStructure;	 
		#endif
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PORTAʱ��                                                                
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  //ʹ��CAN1ʱ�� 
	
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��IO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
	  GPIO_Init(GPIOA, &GPIO_InitStructure);          //��ʼ��IO
	
    //CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM=DISABLE;           //��ʱ�䴥��ͨ��ģʽ  
    CAN_InitStructure.CAN_ABOM=DISABLE;           //����Զ����߹���   
    CAN_InitStructure.CAN_AWUM=DISABLE;           //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART=ENABLE;            //��ֹ�����Զ����� 
    CAN_InitStructure.CAN_RFLM=DISABLE;           //���Ĳ�����,�µĸ��Ǿɵ�  
    CAN_InitStructure.CAN_TXFP=DISABLE;           //���ȼ��ɱ��ı�ʶ������ 
    CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;  //ģʽ���ã�CAN_Mode_Normal ��ͨģʽ��CAN_Mode_LoopBack �ػ�ģʽ; 
		
    //���ò�����
    CAN_InitStructure.CAN_SJW=tsjw;      //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq    CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=tbs1;      //Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~ CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=tbs2;      //Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~ CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=brp; //��Ƶϵ��(Fdiv)Ϊbrp+1  
    CAN_Init(CAN1, &CAN_InitStructure);  //��ʼ��CAN1
		
	  //���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber=0; //������0��0-13��
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //����λģʽ
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ��32-16��
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //32λID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; //32λMASK������ֵΪ0��ȫ�������ģ��κ����ݶ�����ͨ��������
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000; 
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0; //������0������FIFO0����
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
    CAN_FilterInit(&CAN_FilterInitStructure); //�˲�����ʼ��

		#if CAN_INT_ENABLE 	//����������CAN�жϷ�ʽ���յ�����
				CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE); //FIFO0��Ϣ�Һ��ж�����.            
				NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // �����ȼ�Ϊ1
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // �����ȼ�Ϊ0
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
		#endif
		
    return 0;		
}
 
//CAN����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)   
//msg:����ָ��,���Ϊ8���ֽڣ�len:���ݳ���(���Ϊ8) 
//����ֵ:0,�ɹ�; ����,ʧ��;
u8 CAN_Send_Msg(u8* msg, u8 len)
{   
    u8 mbox;
    u16 i=0;
    CanTxMsg TxMessage;
	
    TxMessage.StdId=0x12;           // ��׼��ʶ�� 
    TxMessage.ExtId=0x00;           // ������չ��ʶ��
    TxMessage.IDE=CAN_Id_Standard;  // ��׼֡
    TxMessage.RTR=CAN_RTR_Data;     // ����֡
    TxMessage.DLC=len;              // Ҫ���͵����ݳ���
    for(i=0;i<len;i++)              
		{
			TxMessage.Data[i]=msg[i];     //д������
		}			              
    mbox = CAN_Transmit(CAN1,&TxMessage); // ���ر��ĵķ�������  
    i=0; 
    while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++; //�ȴ����ͽ���
    if(i>=0XFFF) return 1; //��ʱ�˳�����������ʧ��
	
	  return 0;  // ���ͳɹ�  
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;     
//����ֵ:0,�����ݱ��յ�������,���յ����ݳ���;
u8 CAN_Receive_Msg(u8 *buf)
{                  
    u32 i;
    CanRxMsg RxMessage;
	
    if(CAN_MessagePending(CAN1,CAN_FIFO0)) 
		{
			CAN_Receive(CAN1,CAN_FIFO0,&RxMessage); //�����ݣ����ȡ���� 
		} 
		else
		{
			return 0; //û�н��յ�����,ֱ���˳� 
		}
    
    for(i=0;i<8;i++) buf[i]=RxMessage.Data[i]; //��8�����ݷ����������
	
    return RxMessage.DLC;  //������������ 
}

//CAN���жϽ��ճ����жϴ������	
//������can.h�ļ���CAN_INT_ENABLEΪ1����ʹ���ж�
//���ݴ��������жϺ�������ɣ��ⲿ����Ҫ�ڴ���ǰ��CAN�жϣ���ֹ���ݸ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	  vu8 CAN_ReceiveBuff[8]; //CAN�����жϽ��ܵ����ݼĴ���
    vu8 i = 0;
    vu8 u8_RxLen = 0;
  	CAN_ReceiveBuff[0] = 0;	//��ռĴ���
    RxMessage.StdId = 0x00;
    RxMessage.ExtId = 0x00;
    RxMessage.IDE = 0;
    RxMessage.RTR = 0;
    RxMessage.DLC = 0;
    RxMessage.FMI = 0;
	
    for(i=0;i<8;i++) RxMessage.Data[i]=0x00; 
    CAN_Receive(CAN1,CAN_FIFO0,&RxMessage); //����FIFO0����
	
    u8_RxLen = RxMessage.DLC; //������������
    if(RxMessage.StdId==0x12) //�ж�ID�Ƿ�һ�� 
		{ 
		    CAN_ReceiveBuff[0] = RxMessage.DLC; //���յ����������ŵ�����0��λ��
        for( i=0;i<u8_RxLen; i++) //���յ������ݴ���CAN�Ĵ���
			  { 
            CAN_ReceiveBuff[i] = RxMessage.Data[i]; //��8λ���ݴ���CAN���ռĴ���
        }
    }
}

//CAN���߲��Գ���
void Test_CAN(void)
{
		u8 buff[8];
		u8 x;
	
		if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
    {
			buff[0]='A';
		  CAN_Send_Msg(buff,1); //��RS232���ڷ����ַ�����OLED����ʾ		
		} 
		if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
    {
			buff[0]='B';
		  CAN_Send_Msg(buff,1); //��RS232���ڷ����ַ�����OLED����ʾ		
		} 
		if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2))
    {
			buff[0]='C';
			CAN_Send_Msg(buff,1); //��RS232���ڷ����ַ�����OLED����ʾ
		}
		if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3))
		{
			buff[0]='D';
			CAN_Send_Msg(buff,1); //��RS232���ڷ����ַ�����OLED����ʾ
		}
		
		//CAN��Ѱ��ʽ�Ľ��մ���
		x = CAN_Receive_Msg(buff); //����Ƿ��յ�����
		if(x) //�жϽ������ݵ���������Ϊ0��ʾ���յ�����
		{ 
			 //OLED_DISPLAY_8x16(6,11*8,buff[0]); //��OLED����ʾ
		}
}

/*********************************************************************************************
ѡ��IO�ӿڹ�����ʽ��
GPIO_Mode_AIN ģ������
GPIO_Mode_IN_FLOATING ��������
GPIO_Mode_IPD ��������
GPIO_Mode_IPU ��������
GPIO_Mode_Out_PP �������
GPIO_Mode_Out_OD ��©���
GPIO_Mode_AF_PP �����������
GPIO_Mode_AF_OD ���ÿ�©���
*********************************************************************************************/



