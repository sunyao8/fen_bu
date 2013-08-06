#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//EERPROM AT24CXX ���� PB12ΪWP;PB13ΪSCL;PB14ΪSDA	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

   	   		   
//IO��������
//#define SDA_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=8<<24;}
//#define SDA_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}

//IO��������	 
//#define IIC_SCL    PBout(13) //SCL
#define IIC_SCL_1     GPIO_SetBits(GPIOB, GPIO_Pin_13)//SCL 
#define IIC_SDA_1    GPIO_SetBits(GPIOB, GPIO_Pin_14)//SDA	 

//#define IIC_SDA    PBout(14) //SDA	
#define IIC_SCL_0     GPIO_ResetBits(GPIOB, GPIO_Pin_13)//SCL 
#define IIC_SDA_0    GPIO_ResetBits(GPIOB, GPIO_Pin_14)//SDA	 

#define READ_SDA   PBin(14)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif

















