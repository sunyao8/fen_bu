#ifndef __HT1621_H
#define __HT1621_H	 
#include "sys.h"

#define BIAS 0x29
#define SYSEN 0x01
#define LCDOFF 0x02
#define LCDON 0x03

//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//LCD����HT1621���� PB3Ϊ595RCLK;PB4Ϊ1621WR;PB5Ϊ1621CS;PB6Ϊ1621DATA��595SCLK	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//HT1621�˿ڶ���
//#define CS PBout(5) 	// ht1621Ƭѡ
//#define WR PBout(7)	// ht1621дʱ��,74hc595���ݶ�	
//#define DATA PBout(6)	//ht1621���ݣ�74hc595��������ʱ�Ӷ�
//#define RCLK_595 PBout(9)	//74hc595�������������ʱ�Ӷ�
#define CS_1 GPIO_SetBits(GPIOB, GPIO_Pin_5)	// ht1621Ƭѡ
#define WR_1 GPIO_SetBits(GPIOB, GPIO_Pin_7)	
	// ht1621дʱ��,74hc595���ݶ�	
#define DATA_1 GPIO_SetBits(GPIOB, GPIO_Pin_6)	
	//ht1621���ݣ�74hc595��������ʱ�Ӷ�
#define RCLK_595_1 GPIO_SetBits(GPIOB, GPIO_Pin_9)	//74hc595�������������ʱ�Ӷ�

#define CS_0 GPIO_ResetBits(GPIOB, GPIO_Pin_5)	// ht1621Ƭѡ
#define WR_0 GPIO_ResetBits(GPIOB, GPIO_Pin_7)	
	// ht1621дʱ��,74hc595���ݶ�	
#define DATA_0 GPIO_ResetBits(GPIOB, GPIO_Pin_6)
	//ht1621���ݣ�74hc595��������ʱ�Ӷ�
#define RCLK_595_0 GPIO_ResetBits(GPIOB, GPIO_Pin_9)	//74hc595�������������ʱ�Ӷ�


#define GREEN_RED 0X73
#define RED_GREEN 0X6D
#define RED_RED 0X75
#define GREEN_GREEN 0X6B
#define YELLOW_YELLOW 0X61
#define OFF_OFF 0X7F
#define background_light_on 0X80

void HT1621_Init(void);//��ʼ��1621
void SendBit_1621(u8 data,u8 cnt);
void HT595_Send_Byte(u8);

void SendDataBit_1621(u8 data,u8 cnt);
void SendCmd(u8 command);
void Write_1621(u8 addr,u8 data);
void WriteAll_1621(u8 addr,u8 *p,u8 cnt);
void Clera_lcd(void);
void Graf_con_u(u8 cos,u16 volt);
void Graf_cuirrent(u32 current);	
void Graf_qkvar(u16 qkvar);
void Graf_temp(u8 temp);
void Graf_id(u8 hostguest,u8 id);
void Graf_ver(u8 ver);
void Graf_setid(u8 idnum);	 				    
#endif

















