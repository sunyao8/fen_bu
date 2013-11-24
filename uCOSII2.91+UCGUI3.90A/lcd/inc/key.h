#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//KEY 代码 PA11为显示板设置按键；PA12为手动投切开关	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 

//#define KEY0 PAin(11)   	//PA11为显示板设置按键
#define KEY0 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)
#define KEY1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)


void KEY_Init(void);//IO初始化
void key_idset(void);					    
#endif

