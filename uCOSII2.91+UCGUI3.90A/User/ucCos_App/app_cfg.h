/*********************************************************************************************************
*
* File                : app_cfg.c
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

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__
					  

/*
*********************************************************************************************************
*                                       MODULE ENABLE / DISABLE
*********************************************************************************************************
*/
#define  OS_VIEW_MODULE                  DEF_ENABLED	     	/* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */
#define  FUNCTION_MODULE                  DF_THREE/* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */
//#define  FUNCTION_MODULE                  COMMON/* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */


/*
*********************************************************************************************************
*                                              TASKS NAMES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/ 
#define  OS_VIEW_TASK_PRIO               (                 2)
#define  APP_TASK_BACKLIGHT_PRIO         (                 3) 
#define  APP_TASK_TouchPanel_PRIO		 (				   4)
#define  APP_TASK_UCGUI_PRIO             (                 5)
#define  APP_TASK_BLINK_PRIO          	 (OS_LOWEST_PRIO - 4)


//#define  APP_TASK_START_PRIO                               10
#define  APP_TASK_OSVIEW_TERMINAL_PRIO	 (OS_LOWEST_PRIO - 6)
#define  OS_TASK_TMR_PRIO                (OS_LOWEST_PRIO - 2)




/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/
#define  APP_TASK_UCGUI_STK_SIZE                         512u 
#define  APP_TASK_BACKLIGHT_STK_SIZE					 128u
#define  APP_TASK_TouchPanel_STK_SIZE					 256u
#define  APP_TASK_BLINK_STK_SIZE                         128u


//#define  APP_TASK_START_STK_SIZE                          4096u
#define  APP_TASK_OSVIEW_TERMINAL_STK_SIZE   			 256u
#define  OS_VIEW_TASK_STK_SIZE                           256u




/*
*********************************************************************************************************
*                                                  LIB
*********************************************************************************************************
*/

#define  uC_CFG_OPTIMIZE_ASM_EN                 DEF_ENABLED
#define  LIB_STR_CFG_FP_EN                      DEF_DISABLED





#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

