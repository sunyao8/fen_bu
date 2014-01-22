/*********************************************************************************************************
*
* File                : includes.h
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/


#ifndef  __INCLUDES_H__
#define  __INCLUDES_H__

/* Includes ------------------------------------------------------------------*/
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include  <math.h>

#include  <uCOS-II\Source\ucos_ii.h>
#include  <uCOS-II\Ports\ARM-Cortex-M3\RealView\os_cpu.h>
#include  <uC-CPU\ARM-Cortex-M3\RealView\cpu.h>
#include  <lib_def.h>
#include  <lib_mem.h>
#include  <lib_str.h>
#include "delay.h"
#include "ht1621.h"
#include "key.h"
#include "24cxx.h" 
#include "myiic.h"
#include "arm_math.h"
#include "App_cfg.h"
#if (OS_VIEW_MODULE == DEF_ENABLED)
#include    <uCOS-VIEW\Ports\ARM-Cortex-M3\Realview\os_viewc.h>
#include    <uCOS-VIEW\Source\os_view.h>
#endif

#include    <GUI.h>
#include    <stm32f4xx.h>
#include    "TouchPanel.h"
#include    "LCDConf.h"
#include    "GLCD.h"

 typedef struct  
{ 
  u8 myid;      //本电容箱ID号
  u8 size[3];      //容量单位千法
  u8 work_status[3];
}statusbox;
void LIGHT_backligt_on(u8 status_1,u8 status_2,u8 status_3);
void LIGHT_backligt_off(u8 status_1,u8 status_2,u8 status_3);

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


