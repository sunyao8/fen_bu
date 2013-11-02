/*********************************************************************************************************
*
* File                : main.c
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
#define ON_time 13400
#define OFF_time 15000		   //18500
#define  k 0.8	//0.8
#define PI2  6.28318530717959

#define  APP_TASK_START_STK_SIZE                          1024u
static  OS_STK         App_TaskStartStk[APP_TASK_START_STK_SIZE];
#define  APP_TASK_START_PRIO                               10


#define  APP_TASK_LCD_STK_SIZE                          1024u
static  OS_STK         App_TaskLCDStk[APP_TASK_LCD_STK_SIZE];
#define  APP_TASK_LCD_PRIO                               4

#define  APP_TASK_SLAVE3_STK_SIZE                          10240u
static  OS_STK         App_TaskSLAVE3Stk[APP_TASK_SLAVE3_STK_SIZE];
#define  APP_TASK_SLAVE3_PRIO                               1

#define  APP_TASK_COMPUTER_STK_SIZE                       1024u    
static  OS_STK         App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE];
#define  APP_TASK_COMPUTER_PRIO                               3

#define  APP_TASK_Master_STK_SIZE                         10240u
static  OS_STK         App_TaskMasterStk[APP_TASK_Master_STK_SIZE];
#define  APP_TASK_Master_PRIO                               2


/***************************************************/
 typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid;      //本电容箱ID号
  u8 size[2];      //容量单位千法
  u8 work_status[2];    //工作状态 1 为投入工作；0 为没有工作
  u8 work_time[2];     //工作时间   
}status_comm_node;

 typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid;      //本电容箱ID号
  u8 size[3];      //容量单位千法
  u8 work_status[3];    //工作状态 1 为投入工作；0 为没有工作
}status_dis_node;

/***************************************************/
/* Private function prototypes -----------------------------------------------*/
#if (OS_VIEW_MODULE == DEF_ENABLED)
extern void  App_OSViewTaskCreate   (void);
#endif

static  void  App_TaskCreate		(void);
static  void  App_TaskStart			(void		*p_arg);  
extern  void  App_UCGUI_TaskCreate  (void);
static  void  App_TaskLCD		(void		*p_arg); ;
static  void  App_Taskslave_three		(void		*p_arg);  
static  void  App_Taskcomputer	 (void		*p_arg );
static  void  App_TaskMaster(void		*p_arg );


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument(s) : none.
*
* Return(s)   : none.
*********************************************************************************************************
*/
#define ADC1_DR_Address    ((u32)0x4001204C)
#define ADC2_DR_Address    ((u32)0x4001214C)
#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)

u16 ADC_Converted_VValue=0;
u16 ADC_Converted_CValue=0;
u16 ADC_Converted_base=0;
extern u16 dianya_zhi;
extern u8 hguestnum,gonglvshishu;
extern u32 idle_time,scan_time,dianliuzhi;
extern u16 wugongkvar;
extern s8 L_C_flag;
extern u8 id_num,tempshuzhi;
#if (FUNCTION_MODULE == DF_THREE)
extern u16 dianya_zhi_A,dianya_zhi_B,dianya_zhi_C,wugongkvar_A,wugongkvar_B,wugongkvar_C;
extern u32	dianliuzhi_A,dianliuzhi_B,dianliuzhi_C;
extern u8 gonglvshishu_A,gonglvshishu_B,gonglvshishu_C;
#endif

void ADC3_CH10_DMA_Config_VA(void);
void ADC2_CH8_DMA_Config_VEE(void);
void ADC1_CH1_DMA_Config_CA(void);
void ADC3_CH11_DMA_Config_VB(void);
void ADC1_CH4_DMA_Config_CB(void);
void ADC3_CH12_DMA_Config_VC(void);
void ADC1_CH7_DMA_Config_CC(void);
void ADC2_CH13_DMA_Config_A1(void);
void ADC2_CH14_DMA_Config_B1(void);
void ADC2_CH15_DMA_Config_C1(void);

void ADC1_CH1_DMA_Config_VC_phase(void);

void Init_ADC(void);

static  void  GPIO_Configuration    (void);
void allphase(float32_t *V,float32_t *I);
u8 computer_gonglu(status_dis_node *dis_list,status_comm_node *comm_list,u8 *,u8 *);


/*****************************485_start*********************************************************/


#define LEN_control 14
#define EN_USART2_RX 	1			//0,不接收;1,接收.
#define RS485_TX_EN_1		GPIO_SetBits(GPIOB, GPIO_Pin_15)	// 485模式控制.0,接收;1,发送.本工程用PB15
#define RS485_TX_EN_0		GPIO_ResetBits(GPIOB, GPIO_Pin_15)	// 485模式控制.0,接收;1,发送.本工程用PB15
 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485邮箱信号量
 OS_EVENT *computer_sem,*swicth_ABC;			 //
 OS_EVENT *swicth_A;			 //

static u8 rs485buf[LEN_control];//发送控制信息


//接收到的数据长度
u8 RS485_RX_CNT=0;  



 typedef struct  
{ 
  u8 start;
  u8 myid;      //本电容箱ID号
  u8 source;
  u8 destination; //目的电容箱
  u8 send;      //是否是发送命令1为是，0为不是
  u8 relay;    //第几组电容器
  u8 message;     //开关信息
  u8 master;      //主机令牌
  u8 end;   
}box;
box mybox;
 typedef struct  
{ 
  u8 myid;      //本电容箱ID号
  u8 size[3];      //容量单位千法
}statusbox;

static statusbox status_box;
void RS485_Init(u32 bound);
void initmybox(void);//初始化自身信息
void set_now_mystatus(u8 ,u8 ,u8 ,u8 );

void USART2_IRQHandler(void);
u16 comp_16(u16 a,u16 b);
int rs485_trans_order(u8 *tx_r485);//解析由主机发送过来的信号，并发送给下位机
 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void computer_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机计算出来数据解析成RS485信息，发送给目的从机
 u8 rs485_trans_computer(u8 *tx_r485);//解析由主机发送过来的信号，并发送给下位机
void NVIC_Configuration(void);

/***********************************485_end****************************************************/



/********************************switch_A_B_C**************************************************/
//#define ON_time 13400//60
//#define OFF_time 15600//60

#define ON_time 13400                 //100
#define OFF_time 15000		   //1//100

u8 key_A=0,key_B=0,key_C=0;
u8  subswitchABC_onoff	 (u8 relay,u8 message ,u8 flag);

/***********************************end*******************************************************/


/************************************TIME******************************************************/
u16  dog_clock=10;
u8 cont=0;//用于更改主机号的记次数器
 void TIM4_Int_Init(u16 arr,u16 psc);
void delay_time(u32 time);
 void heartbeat(u8 t);



/************************************TIME_end******************************************************/



/************************************MAster data structure*******************/

//status_comm_node comm_list[2];
//status_dis_node dis_list[2];
 void rs485_trans_status_comm(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void status_trans_rs485_dis(statusbox *mystatus);//从机程序
 u8 inquiry_slave_status_dis(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list);   
void scanf_slave_machine(status_dis_node *dis_list,status_comm_node *comm_list,u8 *,u8 *);
 void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list);
void init_Queue(status_dis_node *dis_list,status_comm_node *comm_list,u8 * ,u8 *);
void change_Queue(u8 list_flag,u8 Level, status_dis_node *dis_list,status_comm_node *comm_list,u8 *,u8 * );


/*************************************MAster data structure_end***************/





u8 L_C_flag_A=1;//感性容性标准变量

#define TEST_LENGTH_SAMPLES 512*2 
 




INT32S main (void)
{
CPU_INT08U  os_err;
	

//CPU_IntDis();                   
/***************  Init hardware ***************/


    OS_CPU_SysTickInit();/* Initialize the SysTick.                              */
	delay_init();
	delay_us(500000);
NVIC_Configuration();
GPIO_Configuration();
initmybox();//初始化自身信息
{while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
{while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
{while(subswitchABC_onoff(3,0,1)==0)break;}		  //投

set_now_mystatus(mybox.myid,6,6,6);
os_err = os_err; 


   {
		OSInit();                        


	os_err = OSTaskCreateExt((void (*)(void *)) App_TaskStart,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_START_PRIO,
                             (INT16U          ) APP_TASK_START_PRIO,
                             (OS_STK        * )&App_TaskStartStk[0],
                             (INT32U          ) APP_TASK_START_STK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
                             

#if OS_TASK_NAME_EN > 0
    OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *)"Start Task", &os_err);
#endif

	OSStart();                                               
	return (0);
     }
/************************************************/
  


}


/*
*********************************************************************************************************
*                                          App_TaskStart()
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  
static  void  App_TaskStart (void *p_arg)
{   
	(void)p_arg;
	


		OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
            App_TaskCreate();                                        /* Create application tasks.                            */
	OSTaskSuspend(APP_TASK_START_PRIO);	//挂起起始任务.




}

/*
*********************************************************************************************************
*                                            App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_TaskCreate (void)
{
	
CPU_INT08U  os_err;

RS485_MBOX=OSMboxCreate((void*)0);
RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
computer_sem=OSSemCreate(0);
swicth_A=OSMboxCreate((void*)0);
	os_err = OSTaskCreateExt((void (*)(void *)) App_Taskslave_three,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskSLAVE3Stk[APP_TASK_SLAVE3_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_SLAVE3_PRIO,
                             (INT16U          ) APP_TASK_SLAVE3_PRIO,
                             (OS_STK        * )&App_TaskSLAVE3Stk[0],
                             (INT32U          ) APP_TASK_SLAVE3_STK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
                             

#if OS_TASK_NAME_EN > 0
    OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *)"Start Task", &os_err);
#endif
	 	OSTaskCreate(App_TaskLCD,(void *)0,(OS_STK*)&App_TaskLCDStk[APP_TASK_LCD_STK_SIZE-1],APP_TASK_LCD_PRIO);	 				   
	 	OSTaskCreate(App_Taskcomputer,(void *)0,(OS_STK*)&App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE-1],APP_TASK_COMPUTER_PRIO);	 				   
	 	OSTaskCreate(App_TaskMaster,(void *)0,(OS_STK*)&App_TaskMasterStk[APP_TASK_Master_STK_SIZE-1],APP_TASK_Master_PRIO);	 				   

     }

/*
*********************************************************************************************************
*                                          App_TaskMaster	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  
static  void  App_TaskMaster(void		*p_arg )
{  
u8 scan_init=1;
// static status_dis_node     dis_list[10];
 //static status_comm_node comm_list[10];

	for(;;)
		{

 if(mybox.master==0)
		 	{
			OSTaskSuspend(APP_TASK_Master_PRIO);//挂起从机任务
		        }
/*
 if(start_scan==1)
 	{ scanf_slave_machine();  start_scan=0;}
 */

if(scan_init==1)
{
OSSemPost(computer_sem);
 scan_init=1;
}
   mybox.myid=AT24CXX_ReadOneByte(0x0010);

delay_ms(1500);


					// delay_ms(100);

	        }
   	
}





/**********************************************************************************/

static  void  App_Taskslave_three(void *p_arg)
{  

         u8 err;
	 u8 *msg;


	 
   // OSStatInit();                                            /* Determine CPU capacity.                              */
	for(;;)
   	{	 
        if(mybox.master==1)
		 	{
			OSTaskSuspend(APP_TASK_SLAVE3_PRIO);//挂起从机任务
		        }	 		
   msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
   rs485_trans_computer(msg);
   	 dog_clock=20;
   mybox.myid=AT24CXX_ReadOneByte(0x0010);

   // key_idset();//按键与显示功能

}
}

/*
*********************************************************************************************************
*                                          App_TaskLCD	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  
static  void  App_TaskLCD	 (void		*p_arg )
{  



	for(;;)
		{key_idset();
                     delay_ms(100);//100

	        }
   	
}

/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  
static  void  App_Taskcomputer	 (void		*p_arg )

{  
u8 err;
 static status_dis_node     dis_list[33];
static status_comm_node comm_list[33];
static  u8 slave_dis[1];
static  u8 slave_comm[20];

for(;;)
   	{
   	OSSemPend(computer_sem,0,&err);
#if (FUNCTION_MODULE == DF_THREE)


 scanf_slave_machine(dis_list,comm_list,slave_dis,slave_comm);
 init_Queue(dis_list,comm_list,slave_dis,slave_comm);
 computer_gonglu(dis_list,comm_list,slave_dis,slave_comm);


//inquiry_slave_status_dis(3,dis_list,comm_list);   
#endif

    }	
   	
}

/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/	  





/*
*********************************************************************************************************
*                                          App_Taskcomputer	 (void		*p_arg )
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument(s) : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/


u8  subswitchABC_onoff	 (u8 relay,u8 message ,u8 flag)

{  
u16 i;
float32_t a=0,b=0,max=0;
if(flag==1)
{
if(relay==1)
   	{
   	
ADC3_CH10_DMA_Config_VA();
ADC2_CH13_DMA_Config_A1();
if(message==0)
{
/*
	GPIO_SetBits(GPIOD,GPIO_Pin_8);
		GPIO_ResetBits(GPIOD,GPIO_Pin_9);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 		key_A=1;
*/				

 for(i=0;i<512*2;i++)	 //512*2
	 	{
	 		 	
a=(float32_t)((ADC_Converted_VValue));///  1550
//  a = (a/4096)*3.3;	
if(max<a)max=a;
delay_us(36);//36->512

        }
// GPIO_ResetBits(GPIOD, GPIO_Pin_12);

 for(i=0;i<512*2;i++)
	 	{
	 		 	
b=(float32_t)((ADC_Converted_VValue));///  1550
 // b = (b/4096)*3.3;	
    if(b>max*995/1000)
          {
	    delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_8);
		GPIO_ResetBits(GPIOD,GPIO_Pin_9);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 		key_A=1;
			  max=0;
			  a=0;
			  b=0;
			  return 0;
			
            }
delay_us(36);//36->512

        }
 max=0;
 return 3;
}

if(message==1)
{
/*
GPIO_ResetBits(GPIOD,GPIO_Pin_8); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_9);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 		 		key_A=0;
*/

 for(i=0;i<512*2;i++)
	{ 	b=(float32_t)(((float32_t)(k*ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=10))
		{	
		   					   
		      
				 delay_us(ON_time);
			GPIO_ResetBits(GPIOD,GPIO_Pin_8); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_9);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 		key_A=0;
				return 1;
		
		   
		   }
   		  }
 return 3;

}
}	

if(relay==2)
{
   	
ADC3_CH11_DMA_Config_VB();
ADC2_CH14_DMA_Config_B1();
if(message==0)
{
/*
	GPIO_SetBits(GPIOD,GPIO_Pin_8);
		GPIO_ResetBits(GPIOD,GPIO_Pin_9);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 		key_A=1;
*/				

 for(i=0;i<512*2;i++)
	 	{
	 		 	
a=(float32_t)((ADC_Converted_VValue));///  1550
if(max<a)max=a;
delay_us(36);//36->512

        }
// GPIO_ResetBits(GPIOD, GPIO_Pin_12);

 for(i=0;i<512*2;i++)
	 	{
	 		 	
b=(float32_t)((ADC_Converted_VValue));///  1550
    if(b>max*995/1000)
          {
	     delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_10);
		GPIO_ResetBits(GPIOD,GPIO_Pin_11);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_10);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_11);
		 		key_A=1;
			  max=0;
			  a=0;
			  b=0;
			  return 0;
			
            }
delay_us(36);//36->512

        }
 max=0;
 return 3;
}

if(message==1)
{
/*
GPIO_ResetBits(GPIOD,GPIO_Pin_8); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_9);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 		 		key_A=0;
*/

 for(i=0;i<512*2;i++)
	{ 	b=(float32_t)(((float32_t)(k*ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=10))
		{	
		   					   
		      
				 delay_us(ON_time);
			GPIO_ResetBits(GPIOD,GPIO_Pin_10); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_11);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_10);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_11);
		 		key_A=0;
				return 1;
		
		   
		   }
   		  }
 return 3;

}
}

if(relay==3)
{
   	
ADC3_CH12_DMA_Config_VC();
ADC2_CH15_DMA_Config_C1();
if(message==0)
{
/*
	GPIO_SetBits(GPIOD,GPIO_Pin_8);
		GPIO_ResetBits(GPIOD,GPIO_Pin_9);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 		key_A=1;
*/				

 for(i=0;i<512*2;i++)
	 	{
	 		 	
a=(float32_t)((ADC_Converted_VValue));///  1550
if(max<a)max=a;
delay_us(36);//36->512

        }
// GPIO_ResetBits(GPIOD, GPIO_Pin_12);

 for(i=0;i<512*2;i++)
	 	{
	 		 	
b=(float32_t)((ADC_Converted_VValue));///  1550
    if(b>max*995/1000)
          {
	     delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_12);
		GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_13);
		 		key_A=1;
			  max=0;
			  a=0;
			  b=0;
			  return 0;
			
            }
delay_us(36);//36->512

        }
  max=0;
 return 3;
}

if(message==1)
{
/*
GPIO_ResetBits(GPIOD,GPIO_Pin_8); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_9);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 		 		key_A=0;
*/

 for(i=0;i<512*2;i++)
	{ b=(float32_t)(((float32_t)(k*ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=10))
		{	
		   					   
		      
				 delay_us(ON_time);
			GPIO_ResetBits(GPIOD,GPIO_Pin_12); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_13);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_13);
		 		key_A=0;
				return 1;
		
		   
		   }
   		  }
 return 3;

}
}


}
return 4;
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure GPIO Pin
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* Configure PF6 PF7 PF8 PF9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

/*********************屏幕和按键*****************************************/
	HT1621_Init();
AT24CXX_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  

/***********************采样和DMA**************************************/	
#if (FUNCTION_MODULE == DF_THREE)
ADC2_CH8_DMA_Config_VEE();
Init_ADC();
#endif
/********************485****************************************/	
RS485_Init(9600);
/************************************************************/



/*************************TIME*******************************/
	TIM4_Int_Init(9999*2,7199);//10Khz的计数频率，计数10K次为1000ms 
/************************************************************/


}


void allphase(float32_t *V,float32_t *I)
{
int i=0;
int NPT=TEST_LENGTH_SAMPLES;
for(i=0;i<=NPT/2-1;i++)
{
V[i]=(i+1)*V[i];
I[i]=(i+1)*I[i];
}
for(i=NPT/2;i<NPT-1;i++)
{
V[i]=(NPT-(i+1))*V[i];
I[i]=(NPT-(i+1))*I[i];

}

for(i=0;i<NPT/2-1;i++)
{
V[i+NPT/2]=V[i]+V[i+NPT/2];
I[i+NPT/2]=I[i]+I[i+NPT/2];

}

for(i=0;i<=NPT/2-1;i++)
{
V[i]=V[NPT/2-1+i];
I[i]=I[NPT/2-1+i];

}
}
void ADC1_CH1_DMA_Config_VC_phase(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}

void ADC3_CH10_DMA_Config_VA(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}



void ADC1_CH1_DMA_Config_CA(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}

void ADC2_CH8_DMA_Config_VEE(void)

{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration *************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC2_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_base;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream2, ENABLE);

  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC2, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC2, ENABLE);
ADC_SoftwareStartConv(ADC2);

}

/*******************************B_phase***************************************/

void ADC3_CH11_DMA_Config_VB(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}

void ADC1_CH4_DMA_Config_CB(void)
{
 ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}
/*******************************B_phase_end***********************************/




/********************************C_phase**************************************/
void ADC3_CH12_DMA_Config_VC(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

}

void ADC1_CH7_DMA_Config_CC(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC1);

}
/******************************C_phase_end*********************************/



/********************************A1***************************************/
void ADC2_CH13_DMA_Config_A1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************A1_end***********************************/

/********************************B1***************************************/
void ADC2_CH14_DMA_Config_B1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************B1_end***********************************/

/********************************C1***************************************/
void ADC2_CH15_DMA_Config_C1(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC2);

}

/********************************C1_end***********************************/





void Init_ADC(void)
{
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration *************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_VValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream1, ENABLE);

DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_CValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC2_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Converted_base;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream2, ENABLE);
  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

}

}



/********************************C_phase_end*********************************/
void RS485_Init(u32 bound)

{  
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟


	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //本工程配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //本工程使用

 GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);



  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

/*
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽
    	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //浮空输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

*/	
/******************************************************/
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
 
	
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

    USART_Init(USART2, &USART_InitStructure); ; //初始化串口

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
  //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //使能串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
   
    USART_Cmd(USART2, ENABLE);                    //使能串口 

 //USART_ClearFlag(USART2, USART_FLAG_TC);

 RS485_TX_EN_0;			//默认为接收模式

  
 
}

		void USART2_IRQHandler(void)
{
      CPU_SR          cpu_sr;

	u8 RS485_RX_BUF[64];
	   CPU_CRITICAL_ENTER();                                       /* Tell uC/OS-II that we are starting an ISR            */
    OSIntNesting++;
    CPU_CRITICAL_EXIT();	
 	//GPIO_SetBits(GPIOD, GPIO_Pin_12);	 
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//读取接收到的数据
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='$'){RS485_RX_BUF[0]='$'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='?')
		{
				RS485_RX_CNT=0;
				
		 OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 

			if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

				
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
		} 
	
	}  	
	OSIntExit();  											 

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN_1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN_0;				//设置为接收模式	

}

int rs485_trans_order(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机
{
#if (FUNCTION_MODULE ==  DF_THREE)	
  dianya_zhi_A=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi_A=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar_A=comp_16(tx_r485[10],tx_r485[11]);
  gonglvshishu_A=tx_r485[12];

    dianya_zhi_B=comp_16(tx_r485[13],tx_r485[14]);
  dianliuzhi_B=comp_16(tx_r485[15],tx_r485[16]);
  wugongkvar_B=comp_16(tx_r485[17],tx_r485[18]);
  gonglvshishu_B=tx_r485[19];

    dianya_zhi_C=comp_16(tx_r485[20],tx_r485[21]);
  dianliuzhi_C=comp_16(tx_r485[22],tx_r485[23]);
  wugongkvar_C=comp_16(tx_r485[24],tx_r485[25]);
  gonglvshishu_C=tx_r485[26];
  
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//判断是否是发给本机的信息或是广播信息
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
   #endif

#if (FUNCTION_MODULE ==  COMMON)	
  dianya_zhi=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar=comp_16(tx_r485[10],tx_r485[11]);
  //tempshuzhi=tx_r485[12];
  gonglvshishu=tx_r485[12];
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//判断是否是发给本机的信息或是广播信息
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
   #endif

}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//主机程序，主机命令解析成RS485信息，发送给目的从机
{  
  if(ctr==CONTROL)
    {
      rs485buf[0]='&';//协议头
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=0;
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;
	rs485buf[9]='*';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节

  }
	
if(ctr==CPT_LL )

		{
      rs485buf[0]='&';//协议头	
	rs485buf[1]=(dianya_zhi & (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi & (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;	
	rs485buf[9]='*';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节
	  // 	if(destination==source){mybox.send=send;slave_control(relay, message);}//如果信息发给的自己

    	}
//#endif
}
 u8 rs485_trans_computer(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机

 {
 	         

#if (FUNCTION_MODULE ==  DF_THREE)
if(tx_r485[8]==CPT_A)
{
  dianya_zhi_A=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_A=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_A=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_A=tx_r485[7];
return 1;
}

if(tx_r485[8]==CPT_B)
{
    dianya_zhi_B=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_B=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_B=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_B=tx_r485[7];
return 1;

}
 
if(tx_r485[8]==CPT_C)
{
    dianya_zhi_C=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_C=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_C=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_C=tx_r485[7];
return 1;

}
  #endif



if(tx_r485[5]==CONTROL)

{
   if(mybox.myid==tx_r485[1])//判断是否是发给本机的信息或是广播信息
   	{

	if( tx_r485[2]==1) 
	{
	 if( tx_r485[4]==0)
	 	{while(subswitchABC_onoff(tx_r485[3],0,1)==0)break;}
if(tx_r485[4]==1)
	 	{while(subswitchABC_onoff(tx_r485[3],1,1)==1)break;}
	return 1;

	}

	if( tx_r485[2]==2) 
		{
         status_trans_rs485_dis(&status_box);//从机程序
         return 1;

	    }

   	}
   
}

return 0;

}
 void computer_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//主机程序，主机计算出来数据解析成RS485信息，发送给目的从机

{  

#if (FUNCTION_MODULE == DF_THREE)	
    {
    if(ctr==CPT_A)
    	{
      rs485buf[0]='$';//协议头
	rs485buf[1]=(dianya_zhi_A& (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi_A& (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi_A& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi_A& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar_A& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar_A& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu_A;
	rs485buf[8]=ctr;
	rs485buf[9]='?';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节
    	}
	/************************************/
    if(ctr==CPT_B)
    	{
      rs485buf[0]='$';//协议头
	rs485buf[1]=(dianya_zhi_B& (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi_B& (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi_B& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi_B& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar_B& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar_B& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu_B;
	rs485buf[8]=ctr;
	rs485buf[9]='?';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节
    	}

/***************************************************/
    if(ctr==CPT_C)
    	{
       rs485buf[0]='$';//协议头
       rs485buf[1]=(dianya_zhi_C& (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi_C& (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi_C& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi_C& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar_C& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar_C& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu_C;
	rs485buf[8]=ctr;
	rs485buf[9]='?';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节
    	}
/*********************************************/

    	}
#endif
/*
  if(ctr==CPT_LL)
    	{
       rs485buf[0]='&';//协议头
       rs485buf[1]=(dianya_zhi& (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi& (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu;
	rs485buf[8]=ctr;
	rs485buf[9]='*';//协议尾
	RS485_Send_Data(rs485buf,10);//发送5个字节
    	}
*/
  if(ctr==CONTROL)
  	{
      rs485buf[0]='$';//协议头
	rs485buf[1]=destination;
	rs485buf[2]=send;
	rs485buf[3]=relay;
	rs485buf[4]=message;
	rs485buf[5]=ctr;
	rs485buf[6]='?';//协议尾
	RS485_Send_Data(rs485buf,7);//发送5个字节

  	}
}

 void heartbeat(u8 t)
{	/*u8 i;
for(i=0;i<=t;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_ms(1);
		}	
*/
}

 
void delay_time(u32 time)
{ heartbeat(time);
}    //本系统的延时函数，time*1ms




u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}

void initmybox()//初始化自身信息
{  	 
  
  mybox.master=0;
 mybox.start='&';
mybox.myid=AT24CXX_ReadOneByte(0x0010);
id_num=AT24CXX_ReadOneByte(0x0010);
//mybox.myid=1;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	

/*
status_box.myid=1;
status_box.size[0]=1;
status_box.size[1]=1;
status_box.size[2]=1;
status_box.work_status[0]=0;
status_box.work_status[1]=0;
status_box.work_status[2]=0;
status_box.work_time[0]=0;
status_box.work_time[1]=0;
status_box.work_time[2]=0;
*/
}

void set_now_mystatus(u8 myid,u8 size_1,u8 size_2,u8 size_3)
 {
status_box.myid=myid;
status_box.size[0]=size_1;
status_box.size[1]=size_2;
status_box.size[2]=size_3;
//status_box.work_status[0]=work_status_1;
//status_box.work_status[1]=work_status_2;
//status_box.work_status[2]=work_status_3;

 }

void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list)
{
if(dis_comm==0)
{
if(relay==1)
        {
       dis_list[count].myid=id;
   	   dis_list[count].size[0]=size;
  // 	   dis_list[count].work_status[0]=work_status;
       }
if(relay==2)
        {
       dis_list[count].myid=id;
   	   dis_list[count].size[1]=size;
   //	   dis_list[count].work_status[1]=work_status;
       }
if(relay==3)
        {
       dis_list[count].myid=id;
   	   dis_list[count].size[2]=size;
 //  	   dis_list[count].work_status[2]=work_status;
       }
}
if(dis_comm==1)
{
  if(relay==1)
  	{
	   comm_list[count].myid=id;
   	   comm_list[count].size[0]=size;
   	   comm_list[count].work_status[0]=work_status;
      // comm_list[count].work_time[0]=work_time;
  	}
  if(relay==2)
  	{
	   comm_list[count].myid=id;
   	   comm_list[count].size[1]=size;
   	   comm_list[count].work_status[1]=work_status;
      // comm_list[count].work_time[1]=work_time;
  	}  
}

}
/**********************/
u8 inquiry_slave_status_comm(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list)   
  {  u8 *msg;
        u8 err;
	

   order_trans_rs485(mybox.myid,id,3,0,0,CONTROL);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/30,&err);
   if(err==OS_ERR_TIMEOUT){ return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{  rs485_trans_status_comm(count,msg,dis_list,comm_list);return 1;}

} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号

/*******************************/
 void rs485_trans_status_comm(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{
 	 set_statuslist(count,tx_r485[2],tx_r485[3],tx_r485[5],0,1,1,dis_list,comm_list);//主机状态信息写入状态表
	   set_statuslist(count,tx_r485[2],tx_r485[4],tx_r485[6],0,1,2,dis_list,comm_list);//主机状态信息写入状态表
      
   }
/**********************************/
 u8 inquiry_slave_status_dis(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list)   
  {  u8 *msg;
        u8 err;
/*		
			if(id==mybox.myid)
		{
set_statuslist(id,status_box.size[0],status_box.work_status[0],status_box.work_time[0],0,1,dis_list,comm_list);
set_statuslist(id,status_box.size[1],status_box.work_status[1],status_box.work_time[1],0,2,dis_list,comm_list);
set_statuslist(id,status_box.size[2],status_box.work_status[2],status_box.work_time[2],0,3,dis_list,comm_list);

return 1;
		}
	*/		
{
 computer_trans_rs485(mybox.myid,id,2,0,0,CONTROL);
  // order_trans_rs485(mybox.myid,id,2,0,0);

   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/20,&err);
   if(err==OS_ERR_TIMEOUT)
   	{
          return 0;
   }//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{ 
	rs485_trans_status_dis(count,msg,dis_list,comm_list);//主机状态信息写入状态表

	return 1;
	}

}
} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号
/**********************/
 void status_trans_rs485_dis(statusbox *mystatus)//从机程序
{  	
       rs485buf[0]='&';
	rs485buf[1]='#';
	rs485buf[2]=mystatus->myid;
	rs485buf[3]=mystatus->size[0];
	rs485buf[4]=mystatus->size[1];
	rs485buf[5]=mystatus->size[2];

	//rs485buf[6]=mystatus->work_status[0];
	//rs485buf[7]=mystatus->work_status[1];
	//rs485buf[8]=mystatus->work_status[2];


	rs485buf[6]='*';
	RS485_Send_Data(rs485buf,7);//发送10个字节
}
/**************/
 void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{
 	 set_statuslist(count,tx_r485[2],tx_r485[3],tx_r485[6],0,0,1,dis_list,comm_list);//主机状态信息写入状态表
	 set_statuslist(count,tx_r485[2],tx_r485[4],tx_r485[7],0,0,2,dis_list,comm_list);//主机状态信息写入状态表
      	  set_statuslist(count,tx_r485[2],tx_r485[5],tx_r485[8],0,0,3,dis_list,comm_list);//主机状态信息写入状态表

   } 
 	


/*********************************/
u8 computer_gonglu(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{
int i=0,s=1;
arm_status status; 
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32  S_CFFT;
float32_t maxValue=0.0,maxValue_C=0.0; 
 float32_t testInput_V[TEST_LENGTH_SAMPLES]; 
 float32_t testInput_C[TEST_LENGTH_SAMPLES]; 

float32_t testOutput[TEST_LENGTH_SAMPLES*2/2]; 
float32_t reslut[TEST_LENGTH_SAMPLES/2]; 

/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint32_t fftSize = 512; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1; 
 
/* Reference index at which max energy of bin ocuurs */ 
uint32_t  testIndex = 0,a,b,c; 
 double angle[3]; 
float32_t sine=0;
u16 phase;
u16 wugongkvar_95,wugongkvar_95A,wugongkvar_95B,wugongkvar_95C;
/*********************A_phase*********************************/
//for(s=1;s<=9;s++)
{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();

{
 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
		angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
//		arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_A=maxValue/100;
dianya_zhi_A=dianya_zhi_A/2.6125;
	
/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

dianliuzhi_A=6*maxValue_C/100;
 dianliuzhi_A=1.0554*dianliuzhi_A;
gonglvshishu_A=arm_cos_f32(angle[0]-angle[1])*100;//功率因素

//dianya_zhi_A=0;
//	dianya_zhi_A=comm_list[slave_comm[5]].myid;

//gonglvshishu_A=0;
//	gonglvshishu_A=comm_list[slave_comm[5]].size[0];

arm_sqrt_f32(1-(arm_cos_f32(angle[0]-angle[1]))*(arm_cos_f32(angle[0]-angle[1])),&sine);
        a=dianya_zhi_A*dianliuzhi_A*sine/10;
	wugongkvar_A=dianya_zhi_A*dianliuzhi_A*sine/1000;
      wugongkvar_95A=dianya_zhi_A*dianliuzhi_A*0.3122/1000;
				//	L_C_flag_A=1;

}


				angle[2]=((angle[1]-angle[0])*360)/PI2-90;
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_A=1;
								if(angle[2]>=90&&angle[2]<=180)L_C_flag_A=0;

				if(angle[2]>180&&angle[2]<270)L_C_flag_A=0;

								//	dianya_zhi_A=angle[2];
								//	gonglvshishu_A=1;

				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>=-360.0&&angle[2]<-270.0))L_C_flag_A=1;
										if((angle[2]>=-270.0&&angle[2]<-180.0))L_C_flag_A=0;
					if((angle[2]>=-450.0&&angle[2]<-360.0))L_C_flag_A=1;
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_A=1;
					if((angle[2]>=-180.0&&angle[2]<=-90.0))L_C_flag_A=0;

				//	dianya_zhi_A=-angle[2];
				//	gonglvshishu_A=2;
			     }
computer_trans_rs485(0,33,0,0,0,CPT_A);

/*********************B_phase*********************************/

{
ADC3_CH11_DMA_Config_VB();
ADC1_CH4_DMA_Config_CB();
 maxValue=0.0;
 maxValue_C=0.0; 

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
	 angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位

	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
		//arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_B=maxValue/100;
dianya_zhi_B=dianya_zhi_B/2.6125;

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

dianliuzhi_B=6*maxValue_C/100;
 dianliuzhi_B=1.0554*dianliuzhi_B;
gonglvshishu_B=arm_cos_f32(angle[0]-angle[1])*100;//功率因素
arm_sqrt_f32(1-(arm_cos_f32(angle[0]-angle[1]))*(arm_cos_f32(angle[0]-angle[1])),&sine);
         b=dianya_zhi_B*dianliuzhi_B*sine/10;
	wugongkvar_B=dianya_zhi_B*dianliuzhi_B*sine/1000;
      wugongkvar_95B=dianya_zhi_B*dianliuzhi_B*0.3122/1000;

//dianya_zhi_B=0;
//	dianya_zhi_B=comm_list[slave_comm[3]].myid;

//gonglvshishu_B=0;
//	gonglvshishu_B=comm_list[slave_comm[3]].size[0];


}
computer_trans_rs485(0,33,0,0,0,CPT_B);

/*********************C_phase*********************************/

{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
 maxValue=0.0;
 maxValue_C=0.0; 

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
	 angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电压初始相位
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 

	/*******通过原始数据计算电压值***********/
//		arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
dianya_zhi_C=maxValue/100;
dianya_zhi_C=dianya_zhi_C/2.6125;


/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

dianliuzhi_C=6*maxValue_C/100;
 dianliuzhi_C=1.0554*dianliuzhi_C;
gonglvshishu_C=arm_cos_f32(angle[0]-angle[1])*100;//功率因素
arm_sqrt_f32(1-(arm_cos_f32(angle[0]-angle[1]))*(arm_cos_f32(angle[0]-angle[1])),&sine);
           c=dianya_zhi_C*dianliuzhi_C*sine/10;
	wugongkvar_C=dianya_zhi_C*dianliuzhi_C*sine/1000;
      wugongkvar_95C=dianya_zhi_C*dianliuzhi_C*0.3122/1000;

//dianya_zhi_C=0;
//	dianya_zhi_C=comm_list[slave_comm[0]].myid;

//gonglvshishu_C=0;
//	gonglvshishu_C=comm_list[slave_comm[0]].size[0];

}

/*********************判断相序*******************************/
{

ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_VC_phase();

{
 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }

 
allphase(testInput_V,testInput_C);

 
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
		angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//A相初始相位

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//C相初始相位


}
if((angle[0]-angle[1])>0)
{
phase=((angle[0]-angle[1])*360)/PI2;
if(phase>=118&&phase<=122)dianya_zhi_A=phase;//正序

}
else 
	{
	phase=((angle[1]-angle[0])*360)/PI2;
if(phase>=238&&phase<=242)dianya_zhi_A=phase;//正序


     }
}
/************************判断相序end**************************/






/****************************************************/
computer_trans_rs485(0,33,0,0,0,CPT_C);

/***************************************************/
//inquiry_slave_status_dis(3,dis_list,comm_list);   

/*********************ALL***********************************/
dianya_zhi=1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3;
dianliuzhi=(dianliuzhi_A+dianliuzhi_B+dianliuzhi_C)/3;
gonglvshishu=(gonglvshishu_A+gonglvshishu_B+gonglvshishu_C)/3;
wugongkvar=(a+b+c)/100;
  wugongkvar_95=wugongkvar_95A+wugongkvar_95B+wugongkvar_95C;

   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);

//tempshuzhi=comm_list[1].size[0];
//delay_ms(1000);
}
//computer_trans_rs485(mybox.myid,slave_dis[1],1,3,1,CONTROL);

//computer_trans_rs485(mybox.myid,slave_dis[1],1,3,0,CONTROL);

//tempshuzhi=dis_list[2].size[0];
// tempshuzhi=slave_comm[0];
 //inquiry_slave_status_dis(2,dis_list,comm_list);   
//tempshuzhi=dis_list[2].size[0];
//tempshuzhi= inquiry_slave_status_dis(2,dis_list,comm_list);   

//tempshuzhi=slave_dis[1];
//wugongkvar_A=dis_list[2].size[0];

//inquiry_slave_status_dis(4,dis_list,comm_list);   
//wugongkvar_B=dis_list[3].size[0];

//inquiry_slave_status_comm(5,dis_list,comm_list);   
//wugongkvar_C=comm_list[5].size[0];

/****************************************************/
//computer_trans_rs485(0,0,0,0,0,CPT_LL);

/***************************************************/
//order_trans_rs485(1,3,1,1,0,CONTROL);
//delay_ms(2000);
//order_trans_rs485(1,3,1,1,1,CONTROL);
//delay_ms(2000);


if(gonglvshishu<93&&L_C_flag_A==1)
 {
if(slave_comm[0]>0)
      {
      if(wugongkvar>=20)
      	{
for(i=slave_comm[3];i<=slave_comm[9];i++)
if(comm_list[i].work_status[0]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
change_Queue(1,20,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0 ;
}


{
for(i=slave_comm[6];i<=slave_comm[12];i++)
if(comm_list[i].work_status[1]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],1,comm_list[i].work_time[1],1,2,dis_list,comm_list);
change_Queue(2,20,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0;
}

}
      	}


	  if(wugongkvar>=10)
{
for(i=slave_comm[2];i<=slave_comm[8]-1;i++)
if(comm_list[i].work_status[0]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
change_Queue(1,10,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0;

}

{
for(i=slave_comm[5];i<=slave_comm[11]-1;i++)
if(comm_list[i].work_status[1]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],1,comm_list[i].work_time[1],1,2,dis_list,comm_list);
change_Queue(2,10,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0;
}

}


	  }

	  if(wugongkvar>=5)

{
for(i=slave_comm[1];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status[0]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
change_Queue(1,5,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[4];i<=slave_comm[10]-1;i++)
if(comm_list[i].work_status[1]==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,1,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],1,comm_list[i].work_time[1],1,2,dis_list,comm_list);
change_Queue(2,5,dis_list,comm_list,slave_dis,slave_comm);
//delay_ms(1000);
return 0;
}

}

      	}




      
}
 }

if(gonglvshishu>=94&&L_C_flag_A==1)
   
{
if(slave_comm[0]>0)
      {
{
for(i=slave_comm[1];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[4];i<=slave_comm[10]-1;i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}

}




{
for(i=slave_comm[2];i<=slave_comm[8]-1;i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[5];i<=slave_comm[11]-1;i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}
}

{
for(i=slave_comm[3];i<=slave_comm[9];i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[6];i<=slave_comm[12];i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}
}


       }
 }

if(1)

  {
if(gonglvshishu_A<93&&L_C_flag_A==1)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]<wugongkvar_A&&dis_list[i].work_status[0]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,1,1,CONTROL);
dis_list[i].work_status[0]=1;
//delay_ms(1000);
return 0;
}

}


}
if(gonglvshishu_B<93&&L_C_flag_A==1)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]<wugongkvar_B&&dis_list[i].work_status[1]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,2,1,CONTROL);
dis_list[i].work_status[1]=1;

//delay_ms(1000);
return 0;
}

}


}

if(gonglvshishu_C<93&&L_C_flag_A==1)

{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]<wugongkvar_C&&dis_list[i].work_status[2]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,3,1,CONTROL);
dis_list[i].work_status[2]=1;

//delay_ms(1000);
return 0;
}

}


}
	
  }

if(1)

{
if(gonglvshishu_A>=94&&L_C_flag_A==1)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
//delay_ms(1000);
return 0;
}

}


}
if(gonglvshishu_B>=94&&L_C_flag_A==1)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
//delay_ms(1000);
return 0;
}

}


}

if(gonglvshishu_C>=94&&L_C_flag_A==1)

{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
//delay_ms(1000);
return 0;
}

}


}
	
  }


if(L_C_flag_A==0)
{
if(slave_comm[0]>0)
      {
{
for(i=slave_comm[3];i<=slave_comm[9];i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[6];i<=slave_comm[12];i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}
}



{
for(i=slave_comm[2];i<=slave_comm[8]-1;i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[5];i<=slave_comm[11]-1;i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}
}

{
for(i=slave_comm[1];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status[0]==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,1,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],0,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//delay_ms(1000);
return 0;
}

{
for(i=slave_comm[4];i<=slave_comm[10]-1;i++)
if(comm_list[i].work_status[1]==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,2,0,CONTROL);
set_statuslist(i,comm_list[i].myid,comm_list[i].size[1],0,comm_list[i].work_time[1],1,2,dis_list,comm_list);
//delay_ms(1000);
return 0;
}
}

}


       }
if(1)
{
if(slave_dis[0]>0)
{
if(L_C_flag_A==0)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
//delay_ms(1000);
break;
}

}


}
if(L_C_flag_A==0)
{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
//delay_ms(1000);
break;
}

}


}

if(L_C_flag_A==0)

{
if(slave_dis[0]>0)
{
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid,1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
//delay_ms(1000);
break;
}

}


}
	
  }

}

}
return 0;

}


void scanf_slave_machine(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{
u8 i,j=0,g,flag_comm=0;
//u8 c;
u8 count=1;

for(i=1;i<=2;i++)
	{  
//	for(c=1;c<=2;c++)
		{
	j=inquiry_slave_status_dis(count,i,dis_list,comm_list); 
	if(j==1){count++;break;}
		}
       }
      slave_dis[0]=count-1;


count=1;
j=0;
{
for(i=3;i<=32;i++)
	{  

for(g=1;g<=slave_comm[0];g++)
{
if(i==comm_list[g].myid){flag_comm=1;break;}
else flag_comm=0;
}
if(flag_comm==0)
		{
//	for(c=1;c<=2;c++)
		{
	j=inquiry_slave_status_comm(slave_comm[0]+1,i,dis_list,comm_list);
	        if(j==1){slave_comm[0]++;break;}
		}
			}

if(flag_comm==1)
		{

{
	j=inquiry_slave_status_comm(g,i,dis_list,comm_list);
/*
			if(j==0)
		{
for(i=g;i<slave_comm[0];i++)
  {
          comm_list[i].size[0]=comm_list[i+1].size[0];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[0]=comm_list[i+1].work_time[0];
	   comm_list[i].work_status[0]=comm_list[i+1].work_status[0];

          comm_list[i].size[1]=comm_list[i+1].size[1];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[1]=comm_list[i+1].work_time[1];
	   comm_list[i].work_status[1]=comm_list[i+1].work_status[1];


  }
slave_comm[0]--;
break;	
}
*/		

		}
	
	flag_comm=0;
		}


    }
}

}

/**************************************************************/

/******************************************************************/




void init_Queue(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{

u8 i,j;

u8 t=0;
u8 g=0;
u8 w=0;
u8 s=0;

{
for(i=2;i<=slave_comm[0];i++)
{
  
          t=comm_list[i].size[0];
	   g=comm_list[i].myid;
	   w=comm_list[i].work_time[0];
	   s=	comm_list[i].work_status[0];
	   for(j=i-1;j>=1&&t<comm_list[j].size[0];j--)
	   	{
	   	comm_list[j+1].myid=comm_list[j].myid;
               comm_list[j+1].size[0]=comm_list[j].size[0];
		 comm_list[j+1].work_time[0]=comm_list[j].work_time[0];
	       }
	   comm_list[j+1].myid=g;
	   comm_list[j+1].size[0]=t;
       comm_list[j+1].work_time[0]=w;
            comm_list[j+1].work_status[0]=s;

}
for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[0]==5)
{
slave_comm[1]=i;
break;
}
if(i>slave_comm[0]){slave_comm[1]=0;slave_comm[7]=0;}

if(slave_comm[1]!=0)
{
for(i=slave_comm[1];i<=slave_comm[0];i++)
if(comm_list[i].size[0]!=5)
{
slave_comm[7]=i;
break;
}
if(i>slave_comm[0]){slave_comm[7]=slave_comm[0]+1;}

}


for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[0]==10)
{
slave_comm[2]=i;
break;
}
if(i>slave_comm[0]){slave_comm[2]=0;slave_comm[8]=0;}

if(slave_comm[2]!=0)
{
for(i=slave_comm[2];i<=slave_comm[0];i++)
if(comm_list[i].size[0]!=10)
{
slave_comm[8]=i;
break;
}
if(i>slave_comm[0]){slave_comm[8]=slave_comm[0]+1;}

}



for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[0]==20)
{
slave_comm[3]=i;
break;
}
slave_comm[9]=slave_comm[0];
if(i>slave_comm[0]){slave_comm[3]=0;slave_comm[9]=0;}





}




{
for(i=2;i<=slave_comm[0];i++)
{
  
          t=comm_list[i].size[1];
	   g=comm_list[i].myid;
	   w=comm_list[i].work_time[1];
	   s=	comm_list[i].work_status[1];
	   for(j=i-1;j>=1&&t<comm_list[j].size[1];j--)
	   	{
	   	comm_list[j+1].myid=comm_list[j].myid;
               comm_list[j+1].size[1]=comm_list[j].size[1];
		 comm_list[j+1].work_time[1]=comm_list[j].work_time[1];
	       }
	   comm_list[j+1].myid=g;
	   comm_list[j+1].size[1]=t;
       comm_list[j+1].work_time[1]=w;
            comm_list[j+1].work_status[1]=s;

}


for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[1]==5)
{
slave_comm[4]=i;
break;
}
if(i>slave_comm[0]){slave_comm[4]=0;slave_comm[10]=0;}

if(slave_comm[4]!=0)
{
for(i=slave_comm[4];i<=slave_comm[0];i++)
if(comm_list[i].size[1]!=5)
{
slave_comm[10]=i;
break;
}
if(i>slave_comm[0]){slave_comm[10]=slave_comm[0]+1;}

}



for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[1]==10)
{
slave_comm[5]=i;
break;
}
if(i>slave_comm[0]){slave_comm[5]=0;slave_comm[11]=0;}

if(slave_comm[5]!=0)
{
for(i=slave_comm[5];i<=slave_comm[0];i++)
if(comm_list[i].size[1]!=10)
{
slave_comm[11]=i;
break;
}
if(i>slave_comm[0]){slave_comm[11]=slave_comm[0]+1;}

}




for(i=1;i<=slave_comm[0];i++)
if(comm_list[i].size[1]==20)
{
slave_comm[6]=i;
break;
}

slave_comm[12]=slave_comm[0];
if(i>slave_comm[0]){slave_comm[6]=0;slave_comm[12]=0;}

}

}	

void change_Queue(u8 list_flag,u8 Level, status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm)
{
u8 i;
u8 t=0, g=0,w=0, s=0;

if(list_flag==1)
{
if(Level==5)
{
          t=comm_list[slave_comm[1]].size[0];
	   g=comm_list[slave_comm[1]].myid;
	   w=comm_list[slave_comm[1]].work_time[0];
	   s=	comm_list[slave_comm[1]].work_status[0];

for(i=slave_comm[1];i<slave_comm[7]-1;i++)
  {
          comm_list[i].size[0]=comm_list[i+1].size[0];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[0]=comm_list[i+1].work_time[0];
	   comm_list[i].work_status[0]=comm_list[i+1].work_status[0];

  }
        comm_list[slave_comm[7]-1].size[0]=t;
	  comm_list[slave_comm[7]-1].myid=g;
	  comm_list[slave_comm[7]-1].work_time[0]=w;
	  comm_list[slave_comm[7]-1].work_status[0]=s;

	
}

if(Level==10)
{
          t=comm_list[slave_comm[2]].size[0];
	   g=comm_list[slave_comm[2]].myid;
	   w=comm_list[slave_comm[2]].work_time[0];
	   s=	comm_list[slave_comm[2]].work_status[0];

for(i=slave_comm[2];i<slave_comm[8]-1;i++)
  {
          comm_list[i].size[0]=comm_list[i+1].size[0];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[0]=comm_list[i+1].work_time[0];
	   comm_list[i].work_status[0]=comm_list[i+1].work_status[0];

  }
        comm_list[slave_comm[8]-1].size[0]=t;
	  comm_list[slave_comm[8]-1].myid=g;
	  comm_list[slave_comm[8]-1].work_time[0]=w;
	  comm_list[slave_comm[8]-1].work_status[0]=s;

	
}

if(Level==20)

{
          t=comm_list[slave_comm[3]].size[0];
	   g=comm_list[slave_comm[3]].myid;
	   w=comm_list[slave_comm[3]].work_time[0];
	   s=	comm_list[slave_comm[3]].work_status[0];

for(i=slave_comm[3];i<slave_comm[9];i++)
  {
          comm_list[i].size[0]=comm_list[i+1].size[0];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[0]=comm_list[i+1].work_time[0];
	   comm_list[i].work_status[0]=comm_list[i+1].work_status[0];

  }
        comm_list[slave_comm[9]].size[0]=t;
	  comm_list[slave_comm[9]].myid=g;
	  comm_list[slave_comm[9]].work_time[0]=w;
	  comm_list[slave_comm[9]].work_status[0]=s;

	
}
}
if(list_flag==2)
{
if(Level==5)
{
          t=comm_list[slave_comm[4]].size[1];
	   g=comm_list[slave_comm[4]].myid;
	   w=comm_list[slave_comm[4]].work_time[1];
	   s=	comm_list[slave_comm[4]].work_status[1];

for(i=slave_comm[4];i<slave_comm[10]-1;i++)
  {
          comm_list[i].size[1]=comm_list[i+1].size[1];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[1]=comm_list[i+1].work_time[1];
	   comm_list[i].work_status[1]=comm_list[i+1].work_status[1];

  }
        comm_list[slave_comm[10]-1].size[1]=t;
	  comm_list[slave_comm[10]-1].myid=g;
	  comm_list[slave_comm[10]-1].work_time[1]=w;
	  comm_list[slave_comm[10]-1].work_status[1]=s;

	
}

if(Level==10)
{
          t=comm_list[slave_comm[5]].size[1];
	   g=comm_list[slave_comm[5]].myid;
	   w=comm_list[slave_comm[5]].work_time[1];
	   s=	comm_list[slave_comm[5]].work_status[1];

for(i=slave_comm[5];i<slave_comm[11]-1;i++)
  {
          comm_list[i].size[1]=comm_list[i+1].size[1];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[1]=comm_list[i+1].work_time[1];
	   comm_list[i].work_status[1]=comm_list[i+1].work_status[1];

  }
        comm_list[slave_comm[11]-1].size[1]=t;
	  comm_list[slave_comm[11]-1].myid=g;
	  comm_list[slave_comm[11]-1].work_time[1]=w;
	  comm_list[slave_comm[11]-1].work_status[1]=s;

	
}

if(Level==20)

{
          t=comm_list[slave_comm[6]].size[1];
	   g=comm_list[slave_comm[6]].myid;
	   w=comm_list[slave_comm[6]].work_time[1];
	   s=	comm_list[slave_comm[6]].work_status[1];

for(i=slave_comm[6];i<slave_comm[12];i++)
  {
          comm_list[i].size[1]=comm_list[i+1].size[1];
	   comm_list[i].myid=comm_list[i+1].myid;
	   comm_list[i].work_time[1]=comm_list[i+1].work_time[1];
	   comm_list[i].work_status[1]=comm_list[i+1].work_status[1];

  }
        comm_list[slave_comm[12]].size[1]=t;
	  comm_list[slave_comm[12]].myid=g;
	  comm_list[slave_comm[12]].work_time[1]=w;
	  comm_list[slave_comm[12]].work_status[1]=s;

	
}
}
}



/***********************************************************************
TIME_4

**********************************************************************/
 void TIM4_Int_Init(u16 arr,u16 psc)

{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}
void turn_master_id(u8 id)//改变当前整个系统中主机的ID号
{
   u8 flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	  	   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);
	//delay_time(2);
         mybox.master=1;
		 hguestnum=111;
	    OSTaskResume(APP_TASK_Master_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }

 void TIM4_IRQHandler(void)   //TIM4中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
// GPIO_SetBits(GPIOD, GPIO_Pin_12);
  //GPIO_ResetBits(GPIOD, GPIO_Pin_12);	 
			turn_master_id(mybox.myid);
			  cont++;
			}
			if(dog_clock>0){dog_clock--;cont=1;}
		 }
		}
	   	OSIntExit();  

 	}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
