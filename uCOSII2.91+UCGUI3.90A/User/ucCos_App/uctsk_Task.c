/*********************************************************************************************************
*
* File                : uctsk_Task.c
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

/* Includes ------------------------------------------------------------------*/
#include <includes.h>            



/* Private variables ---------------------------------------------------------*/
static  OS_STK         App_Task_UCGUI_Stk        [APP_TASK_UCGUI_STK_SIZE];
static  OS_STK         App_Task_TouchPanel_Stk   [APP_TASK_TouchPanel_STK_SIZE];
static  OS_STK         App_TaskBlinkStk          [APP_TASK_BLINK_STK_SIZE];

/* Private function prototypes -----------------------------------------------*/
       void MainTask         (void);
static void uctsk_UCGUI      (void);
static void uctsk_TouchPanel (void);
static void uctsk_Blink      (void);
extern void _ExecCalibration (void);
 
void  App_UCGUI_TaskCreate (void)
{
    CPU_INT08U  os_err;

	os_err = os_err; 

	os_err = OSTaskCreate((void (*)(void *)) uctsk_UCGUI,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_Task_UCGUI_Stk[APP_TASK_UCGUI_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_UCGUI_PRIO  );							

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_UCGUI_PRIO, "Task UCGUI", &os_err);
	#endif
   
    os_err = OSTaskCreate((void (*)(void *)) uctsk_TouchPanel,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_Task_TouchPanel_Stk[APP_TASK_TouchPanel_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_TouchPanel_PRIO  );

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_TouchPanel_PRIO, "TASK TouchPanel Messages", &os_err);
	#endif


	os_err = OSTaskCreate((void (*)(void *)) uctsk_Blink,				
                          (void          * ) 0,							
                          (OS_STK        * )&App_TaskBlinkStk[APP_TASK_BLINK_STK_SIZE - 1],		
                          (INT8U           ) APP_TASK_BLINK_PRIO  );							

	#if OS_TASK_NAME_EN > 0
    	OSTaskNameSet(APP_TASK_BLINK_PRIO, "Task LED Blink", &os_err);
	#endif


}							 

static void uctsk_UCGUI (void) 
{   
   GUI_Init();
   _ExecCalibration();	  /* Calibrate touch panel */
   	for(;;)
   	{  
	  MainTask(); 
    }
}

static void uctsk_Blink (void) 
{                 
    for(;;)
   	{  
     	
        /*====LED-ON=======*/
	    GPIO_SetBits(GPIOF , GPIO_Pin_6);
	    GPIO_SetBits(GPIOF , GPIO_Pin_7);
	    GPIO_SetBits(GPIOF , GPIO_Pin_8);
	    GPIO_SetBits(GPIOF , GPIO_Pin_9);
		OSTimeDlyHMSM(0, 0, 0, 50);	 /* 50 MS  */
      	
	    /*====LED-OFF=======*/ 
	    GPIO_ResetBits(GPIOF , GPIO_Pin_6);
	    GPIO_ResetBits(GPIOF , GPIO_Pin_7);
	    GPIO_ResetBits(GPIOF , GPIO_Pin_8);
	    GPIO_ResetBits(GPIOF , GPIO_Pin_9); 
		OSTimeDlyHMSM(0, 0, 0, 50);	 /* 50 MS  */
    }
}

static void uctsk_TouchPanel (void) 
{  
    TP_Init();			             /* Initialize touch panel */ 
    
   	for(;;)
   	{  
	   GUI_TOUCH_Exec();
	   OSTimeDlyHMSM(0, 0, 0, 20);	 /* 20 MS  */
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
