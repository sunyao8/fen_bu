
#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//EERPROM AT24CXX���� PB12ΪWP;PB13ΪSCL;PB14ΪSDA	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//��ʼ��IIC
void IIC_Init(void)
{
/*
 	RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 							 
	GPIOB->CRH&=0XF000FFFF;//PB12/1113/14 �������
	GPIOB->CRH|=0X03330000;	   
	GPIOB->ODR&=~(1<<12);     //PB12����͵�ƽ
	GPIOB->ODR|=1<<13;		  //PB13,14 �����
	GPIOB->ODR|=1<<14;
	*/
	  GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  GPIO_SetBits(GPIOB,GPIO_Pin_13);
    GPIO_SetBits(GPIOB,GPIO_Pin_14);

}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	//SDA_OUT();     //sda�����
			  GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	IIC_SDA_1;	  	  
	IIC_SCL_1;
	delay_us(4);
 	IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	//SDA_OUT();//sda�����
				  GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_1; 
	IIC_SDA_1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	//SDA_IN();      //SDA����Ϊ����
		  GPIO_InitTypeDef      GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	IIC_SDA_1;delay_us(1);	   
	IIC_SCL_1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{				  GPIO_InitTypeDef      GPIO_InitStructure;

	IIC_SCL_0;
	//SDA_OUT();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	IIC_SDA_0;
	delay_us(2);
	IIC_SCL_1;
	delay_us(2);
	IIC_SCL_0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{			  GPIO_InitTypeDef      GPIO_InitStructure;

	IIC_SCL_0;
//	SDA_OUT();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	IIC_SDA_1;
	delay_us(2);
	IIC_SCL_1;
	delay_us(2);
	IIC_SCL_0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	//SDA_OUT();
				  GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

    IIC_SCL_0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        
			if(((txd&0x80)>>7)==0)IIC_SDA_0;
						if(((txd&0x80)>>7)==1)IIC_SDA_1;

        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_1;
		delay_us(2); 
		IIC_SCL_0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	//SDA_IN();//SDA����Ϊ����
	  GPIO_InitTypeDef      GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

    for(i=0;i<8;i++ )
	{
        IIC_SCL_0; 
        delay_us(2);
		IIC_SCL_1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























