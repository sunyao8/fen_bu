#ifndef __HT1621_H
#define __HT1621_H	 
#include "sys.h"

#define BIAS 0x29
#define SYSEN 0x01
#define LCDOFF 0x02
#define LCDON 0x03

//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//LCD驱动HT1621代码 PB3为595RCLK;PB4为1621WR;PB5为1621CS;PB6为1621DATA和595SCLK	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//HT1621端口定义
//#define CS PBout(5) 	// ht1621片选
//#define WR PBout(7)	// ht1621写时钟,74hc595数据端	
//#define DATA PBout(6)	//ht1621数据，74hc595数据输入时钟端
//#define RCLK_595 PBout(9)	//74hc595输出锁存器锁存时钟端
#define CS_1 GPIO_SetBits(GPIOB, GPIO_Pin_5)	// ht1621片选
#define WR_1 GPIO_SetBits(GPIOB, GPIO_Pin_7)	
	// ht1621写时钟,74hc595数据端	
#define DATA_1 GPIO_SetBits(GPIOB, GPIO_Pin_6)	
	//ht1621数据，74hc595数据输入时钟端
#define RCLK_595_1 GPIO_SetBits(GPIOB, GPIO_Pin_9)	//74hc595输出锁存器锁存时钟端

#define CS_0 GPIO_ResetBits(GPIOB, GPIO_Pin_5)	// ht1621片选
#define WR_0 GPIO_ResetBits(GPIOB, GPIO_Pin_7)	
	// ht1621写时钟,74hc595数据端	
#define DATA_0 GPIO_ResetBits(GPIOB, GPIO_Pin_6)
	//ht1621数据，74hc595数据输入时钟端
#define RCLK_595_0 GPIO_ResetBits(GPIOB, GPIO_Pin_9)	//74hc595输出锁存器锁存时钟端

#define RED_RED_GREEN 0X35
#define RED_GREEN_RED 0X4D
#define GREEN_RED_RED 0X53
#define GREEN_RED_GREEN 0X33
#define RED_GREEN_GREEN 0X2D
#define GREEN_GREEN_RED 0X4B
#define GREEN_GREEN_GREEN 0X2B
#define RED_RED_RED 0X55
#define YELLOW_YELLOW_YELLOW 0X01
#define OFF_OFF 0X7F
#define background_light_on 0X80

void HT1621_Init(void);//初始化1621
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

















