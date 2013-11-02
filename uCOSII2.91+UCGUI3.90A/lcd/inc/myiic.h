#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//EERPROM AT24CXX 代码 PB12为WP;PB13为SCL;PB14为SDA	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

   	   		   
//IO方向设置
//#define SDA_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=8<<24;}
//#define SDA_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}

//IO操作函数	 
//#define IIC_SCL    PBout(13) //SCL
#define IIC_SCL_1     GPIO_SetBits(GPIOB, GPIO_Pin_13)//SCL 
#define IIC_SDA_1    GPIO_SetBits(GPIOB, GPIO_Pin_14)//SDA	 

//#define IIC_SDA    PBout(14) //SDA	
#define IIC_SCL_0     GPIO_ResetBits(GPIOB, GPIO_Pin_13)//SCL 
#define IIC_SDA_0    GPIO_ResetBits(GPIOB, GPIO_Pin_14)//SDA	 

#define READ_SDA  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif

















