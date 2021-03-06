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
#define ON_time 13000        //13400
#define OFF_time 15000		   //18500
#define PI2  6.28318530717959
#define TIME_TQ 2
#define temperture_w 55

#define  APP_TASK_START_STK_SIZE                         64u
static  OS_STK         App_TaskStartStk[APP_TASK_START_STK_SIZE];
#define  APP_TASK_START_PRIO                               10


#define  APP_TASK_LCD_STK_SIZE                          256u
static  OS_STK         App_TaskLCDStk[APP_TASK_LCD_STK_SIZE];
#define  APP_TASK_LCD_PRIO                               5

#define  APP_TASK_SLAVE3_STK_SIZE                          64u
static  OS_STK         App_TaskSLAVE3Stk[APP_TASK_SLAVE3_STK_SIZE];
#define  APP_TASK_SLAVE3_PRIO                               2

#define  APP_TASK_COMPUTER_STK_SIZE                       10240u    
static  OS_STK         App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE];
#define  APP_TASK_COMPUTER_PRIO                               4

#define  APP_TASK_Master_STK_SIZE                         64u
static  OS_STK         App_TaskMasterStk[APP_TASK_Master_STK_SIZE];
#define  APP_TASK_Master_PRIO                               3

#define SETID_TASK_PRIO       			1 
//设置任务堆栈大小
#define SETID_STK_SIZE  		    		64
//任务堆栈
OS_STK SETID_TASK_STK[SETID_STK_SIZE];

#define urgent_TASK_PRIO       			0 
//设置任务堆栈大小
#define urgent_STK_SIZE  		    		64
//任务堆栈
OS_STK urgent_TASK_STK[urgent_STK_SIZE];


//任务函数

/***************************************************/
 typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid;      //本电容箱ID号
  u8 size;      //容量单位千法
  u8 work_status;    //工作状态 1 为投入工作；0 为没有工作
  u8 work_time;     //工作时间
  u8 group;// 第几组表示
}status_comm_node;

 typedef struct  
{ 
  u8 dis_comm;//dis=0 comm=1
  u8 myid[3];      //本电容箱ID号
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
static  void SETID_task(void *pdata);
static  void urgent_task(void *pdata);

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
void ADC3_CH1_DMA_Config_CA_phase(void);
u16 ADC_Temperature(void);

void Init_ADC(void);

static  void  GPIO_Configuration    (void);
void allphase(float32_t *V,float32_t *I);
u8 computer_gonglu(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm);

void temperature(void);   //电容器温度检测

/*****************************485_start*********************************************************/


#define LEN_control 14
#define EN_USART2_RX 	1			//0,不接收;1,接收.
#define RS485_TX_EN_1		GPIO_SetBits(GPIOB, GPIO_Pin_15)	// 485模式控制.0,接收;1,发送.本工程用PB15
#define RS485_TX_EN_0		GPIO_ResetBits(GPIOB, GPIO_Pin_15)	// 485模式控制.0,接收;1,发送.本工程用PB15
 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX,* RS485_RT,*RS485_STUTAS_MBOX_dis;			//	rs485邮箱信号量
 OS_EVENT *computer_sem,*urgent_sem;			 //

static u8 rs485buf[LEN_control];//发送控制信息


//接收到的数据长度
vu32 RS485_RX_CNT=0;  



 typedef struct  
{ u8 start;
  u8 myid;      //本电容箱ID号
  u8 source;
  u8 destination; //目的电容箱
  u8 send;      //是否是发送命令1为是，0为不是
  u8 relay;    //第几组电容器
  u8 message;     //开关信息
  u8 master;      //主机令牌
u8 end;
}box;
static box mybox;


 statusbox status_box;
u8 auto_on=1;
u8 ON_OFF=2;
void RS485_Init(u32 bound);
void initmybox(void);//初始化自身信息
void set_now_mystatus(u8 ,u8 ,u8 ,u8,u8,u8,u8 );

void USART2_IRQHandler(void);
 void EXTI15_10_IRQHandler(void);
 void TIM3_IRQHandler(void) ; //TIM3中断

u16 comp_16(u16 a,u16 b);
int rs485_trans_order(u8 *tx_r485);//解析由主机发送过来的信号，并发送给下位机
 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void computer_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr);//主机程序，主机计算出来数据解析成RS485信息，发送给目的从机
 u8 rs485_trans_computer(u8 *tx_r485);//解析由主机发送过来的信号，并发送给下位机
void NVIC_Configuration(void);
void EXTI_Configuration(void);//初始化函数

/***********************************485_end****************************************************/



/********************************switch_A_B_C**************************************************/
//#define ON_time 13400//60
//#define OFF_time 15600//60

//#define ON_time 13400                 //100
//#define OFF_time 15000		   //1//100

u16 var=0;

u8  subswitchABC_onoff	 (u8 relay,u8 message ,u8 flag);
void LIGHT_backligt_on(u8 status_1,u8 status_2,u8 status_3);
void LIGHT_backligt_off(u8 status_1,u8 status_2,u8 status_3);

/***********************************end*******************************************************/


/************************************TIME******************************************************/
u16  dog_clock=10;
u8 cont=0;//用于更改主机号的记次数器
 void TIM3_Int_Init(u16 arr,u16 psc);
  void TIM4_Int_Init(u16 arr,u16 psc);

void delay_time(u32 time);
 void heartbeat(u8 t);



/************************************TIME_end******************************************************/

void init_Queue_dis(status_dis_node *dis_list,u8 *slave_dis);
void change_Queue_dis(u8 abc,u8 Level, status_dis_node *dis_list,u8 *slave_dis);


/************************************MAster data structure*******************/

//status_comm_node comm_list[2];
//status_dis_node dis_list[2];
  void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list);//主机程序，主机命令解析成RS485信息，发送给目的从机
 void status_trans_rs485_dis(statusbox *mystatus);//从机程序
 u8 inquiry_slave_status_dis(u8 count,u8 id,status_dis_node *dis_list,status_comm_node *comm_list);   
 void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list_1,status_comm_node *comm_list_2,u8 group);
 
 
 void	warn(void);


/*************************************MAster data structure_end***************/

/************************************MAster data structure*******************/
void scanf_slave_machine(status_dis_node *dis_list,status_comm_node *comm_list,u8 *slave_dis,u8 *slave_comm);
u8 inquiry_slave_status_comm(u8 id,u8 *slave_comm,status_comm_node *comm_list); 
void init_Queue(u8 id,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 *slave_comm,status_comm_node *comm_list);
void del_comm_listnode(u8 id,u8 group,u8 *slave_comm,status_comm_node *comm_list);
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list);
void change_Queue(u8 *slave_comm,status_comm_node *comm_list,u8 size);
void init_listindex(u8 *slave_comm);




/********************************************************************************/



u8 L_C_flag_A=1;//感性容性标准变量
u8 L_C_flag_B=1;//感性容性标准变量
u8 L_C_flag_C=1;//感性容性标准变量

#define TEST_LENGTH_SAMPLES 512*2 
 
u8 phase_flag=0;

u8 phase_flag_c=0;

u16 T=10;
u8 RT_FLAG=2;//3//3  为 自动判断变比
u16 scan_init=0;//20  为自动判断变比
u8 MASTER=0;
extern u8 light_time;
 u8 temperature_warn=0;
u8 rework_time[3]={1,1,1};//再投延时控制变量
extern vu8 BT_num;
extern vu8	warn_volt_onlimt;//过压值 +400
#define BT  12//*50
#define SIZE 6 //总容量除以3 
#define AF_TOP 99
#define AF_DEAD 96

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
 EXTI_Configuration();//初始化函数

initmybox();//初始化自身信息

set_now_mystatus(mybox.myid,SIZE,SIZE,SIZE,0,0,0);
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
RS485_STUTAS_MBOX_dis=OSMboxCreate((void*)0);
computer_sem=OSSemCreate(0);
RS485_RT=OSMboxCreate((void*)0);
urgent_sem=OSSemCreate(0);
                             

#if OS_TASK_NAME_EN > 0
    OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *)"Start Task", &os_err);
#endif
	 	OSTaskCreate(App_TaskLCD,(void *)0,(OS_STK*)&App_TaskLCDStk[APP_TASK_LCD_STK_SIZE-1],APP_TASK_LCD_PRIO);	 				   
	 	OSTaskCreate(App_Taskcomputer,(void *)0,(OS_STK*)&App_TaskComputerStk[APP_TASK_COMPUTER_STK_SIZE-1],APP_TASK_COMPUTER_PRIO);	 				   
	 	OSTaskCreate(App_TaskMaster,(void *)0,(OS_STK*)&App_TaskMasterStk[APP_TASK_Master_STK_SIZE-1],APP_TASK_Master_PRIO);	 				   
		 OSTaskCreate(App_Taskslave_three,(void *)0,(OS_STK*)&App_TaskSLAVE3Stk[APP_TASK_SLAVE3_STK_SIZE-1],APP_TASK_SLAVE3_PRIO);		 
		 OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);		 
//OSTaskCreate(urgent_task,(void *)0,(OS_STK*)&urgent_TASK_STK[urgent_STK_SIZE-1],urgent_TASK_PRIO);

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
 static u8 frist=1;

static  void  App_TaskMaster(void		*p_arg )
{  
// static status_dis_node     dis_list[10];
 //static status_comm_node comm_list[10];
	for(;;)
		{
		
		if(frist==1)
			{
subswitchABC_onoff(1,0,1);		  //投
subswitchABC_onoff(2,0,1);		  //投
subswitchABC_onoff(3,0,1);//投
frist=0;
		}
 if(MASTER==0)
		 	{
			OSTaskSuspend(APP_TASK_Master_PRIO);//挂起从机任务
		        }
 


 if(MASTER==1)
 	{
 	
hguestnum=111;

OSSemPost(computer_sem);

if(scan_init!=0) {scan_init--;order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);}
if(scan_init==1)
{
RT_FLAG=2;
scan_init=0;
}

   mybox.myid=AT24CXX_ReadOneByte(0xa000);
status_box.myid= mybox.myid;
	
 	}

delay_ms(1500);
//delay_ms(100);
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
   	{
if((ON_OFF==1)&&(KEY1==1&&auto_on==0))
	{   
	light_time=30;
		  auto_on=1;
                 if(	status_box.work_status[0]==1)
		   {subswitchABC_onoff(1,0,1);}		  //投
                 if(	status_box.work_status[1]==1)
		   {subswitchABC_onoff(2,0,1);}		  //投
                 if(	status_box.work_status[2]==1)
		   {subswitchABC_onoff(3,0,1);}		  //投

 }
     if((ON_OFF==0)&&(KEY1==0&&auto_on==1))
 	{
	light_time=30;
		auto_on=0;
                 if((status_box.work_status[0]==0)&&(rework_time[0]==0))
		   {subswitchABC_onoff(1,1,1);}		  //投
                 if((status_box.work_status[1]==0)&&(rework_time[1]==0))
		   {subswitchABC_onoff(2,1,1);}		  //投
                 if((status_box.work_status[2]==0)&&(rework_time[2]==0))
		   {subswitchABC_onoff(3,1,1);}		  //投

	 }

		  }
        if(MASTER==1)
		 	{
			OSTaskSuspend(APP_TASK_SLAVE3_PRIO);//挂起从机任务
		        }	 	
	
   msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
   rs485_trans_computer(msg);
   	 dog_clock=20;
   mybox.myid=AT24CXX_ReadOneByte(0xa000);
status_box.myid= mybox.myid;
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
		{
	warn();
		key_idset();
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

static  u8 slave_dis[20];
static  u8 slave_comm[20];
static status_comm_node comm_list[78];
/*
 static status_dis_node     dis_list[20];

static  u8 slave_dis[10];
static  u8 slave_comm[10];
static status_comm_node comm_list[70];
*/
static u8 first_init=1;

for(;;)
   	{
   	   	OSSemPend(computer_sem,0,&err);
if(MASTER==1)
		{
#if (FUNCTION_MODULE == DF_THREE)
if(first_init==1)
{
init_listindex(slave_comm);
first_init=0;
}
 scanf_slave_machine(dis_list,comm_list,slave_dis,slave_comm);
  init_Queue_dis(dis_list,slave_dis);
 computer_gonglu(dis_list,comm_list,slave_dis,slave_comm);


#endif

    }
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



void SETID_task(void *pdata)

{
        OS_CPU_SR cpu_sr=0;  	    	
          while(1)
          	{
		  id_num=AT24CXX_ReadOneByte(0xa000);
	///	  id_num=1;//测试开发板使用
		if(id_num<1||id_num>33)
			{        	OS_ENTER_CRITICAL();    		
                                   MASTER=2;
                      		OSTaskSuspend( APP_TASK_SLAVE3_PRIO  );//挂起主机任状态.
                      		OSTaskSuspend( APP_TASK_COMPUTER_PRIO);
                                   OSTaskSuspend(APP_TASK_Master_PRIO);
						OS_EXIT_CRITICAL();
						 LIGHT_backligt_on(2,2,2);
			delay_ms(100);
								   
		       }
               else if(id_num<=32&&id_num>=1)
               	{ 
               				   OS_ENTER_CRITICAL();
						 LIGHT_backligt_on(0,0,0);
                                  MASTER=0;
				      mybox.myid=id_num;
			//	HT595_Send_Byte((GREEN_GREEN)|background_light_on);
		 OSTaskResume(APP_TASK_SLAVE3_PRIO );//启动主机任务状态
		 OSTaskResume(APP_TASK_COMPUTER_PRIO );//启动显示任务状态
		 OSTaskResume(APP_TASK_Master_PRIO );//启动从机任务状态
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

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
    if(b>max*970/1000)
          {
	    delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_8);
		GPIO_ResetBits(GPIOD,GPIO_Pin_9);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_8);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
				status_box.work_status[0]=3;
				rework_time[0]=1;
				if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
				if(light_time==1)LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
				
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
while(1)
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
	{ 	b=(float32_t)(((float32_t)(ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=3))
		{	
		   					   
		      
				 delay_us(ON_time);
				// delay_us(12900);
			GPIO_ResetBits(GPIOD,GPIO_Pin_8); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_9);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_9);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_8);
				status_box.work_status[0]=1;
		if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
		if(light_time==1) LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);		
				return 1;
		
		   
		   }
   		  }
 //return 3;

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
    if(b>max*970/1000)
          {
	     delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_10);
		GPIO_ResetBits(GPIOD,GPIO_Pin_11);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_10);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_11);
				status_box.work_status[1]=3;
				rework_time[1]=1;
			if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);		
			 if(light_time==1)LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);	
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
while(1)
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
	{ 	b=(float32_t)(((float32_t)(ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=3))
		{	
		   					   
		      
				 delay_us(ON_time);
				 //delay_us(12900);
			GPIO_ResetBits(GPIOD,GPIO_Pin_10); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_11);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_10);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_11);
				status_box.work_status[1]=1;
	if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);				
	 if(light_time==1)LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);			
				return 1;
		
		   
		   }
   		  }
 //return 3;

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
    if(b>max*970/1000)
          {
	     delay_us(OFF_time);
		GPIO_SetBits(GPIOD,GPIO_Pin_12);
		GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			   delay_ms(100);
         GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_13);
				status_box.work_status[2]=3;
				rework_time[2]=1;				
if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);				
if(light_time==1) LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
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
while(1)
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
	{ b=(float32_t)(((float32_t)(ADC_Converted_VValue)-(float32_t)(ADC_Converted_base)));///  1550
		delay_us(36);//36->512			        
		   if((b>0)&&(b<=3))
		{	
		   					   
		      
				 delay_us(ON_time);
				 //delay_us(12900);
			GPIO_ResetBits(GPIOD,GPIO_Pin_12); //PD2->1
			GPIO_SetBits(GPIOD,GPIO_Pin_13);  //PC11->0
			 delay_ms(100);//脉冲延时
		 GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		 GPIO_ResetBits(GPIOD,GPIO_Pin_13);
		 				status_box.work_status[2]=1;
if(light_time>1)LIGHT_backligt_on(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
if(light_time==1) LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
				return 1;
		
		   
		   }
   		  }
// return 3;

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

IWDG_Init(4,625); 


/*************************TIME*******************************/
	TIM3_Int_Init(9999*2,7199);//10Khz的计数频率，计数10K次为1000ms 
		TIM4_Int_Init(4999,7199);//10Khz的计数频率，计数10K次为1000ms 

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

u16 ADC_Temperature(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles);
ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果

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

void ADC3_CH1_DMA_Config_CA_phase(void)
{
  ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
ADC_SoftwareStartConv(ADC3);

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

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_7|GPIO_Pin_5;
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

	vu8 RS485_RX_BUF[512];
	   CPU_CRITICAL_ENTER();                                       /* Tell uC/OS-II that we are starting an ISR            */
    OSIntNesting++;
    CPU_CRITICAL_EXIT();	
 	//GPIO_SetBits(GPIOD, GPIO_Pin_12);	 
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//读取接收到的数据
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='$'){RS485_RX_BUF[0]='$'; RS485_RX_CNT=1;}//作为从机时，接收主机的计算数据协议
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='?')
		{
				
	if(((RS485_RX_CNT==12)&&(RS485_RX_BUF[8]==CPT_A))||((RS485_RX_CNT==12)&&(RS485_RX_BUF[8]==CPT_B))||((RS485_RX_CNT==12)&&(RS485_RX_BUF[8]==CPT_C)))

		{
		if(MASTER==1)
			{				
			if(mybox.myid>RS485_RX_BUF[10])
						{
					MASTER=0;
					  hguestnum=222;
					dog_clock=20;	 
				       OSTaskSuspend(APP_TASK_Master_PRIO );//挂起主机任状态.	
					OSTaskResume(APP_TASK_SLAVE3_PRIO );//启动从机任务状态	 
					}
		}
			if(MASTER==0) {OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);}
		}

						RS485_RX_CNT=0;
		} 

		if(RS485_RX_BUF[RS485_RX_CNT-1]=='-'){RS485_RX_BUF[0]='-'; RS485_RX_CNT=1;}//作为从机时，接收主机的控制协议
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='=')
		{
				
if(((RS485_RX_CNT==7)&&(RS485_RX_BUF[5]==CONTROL)))
			{OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);}
						RS485_RX_CNT=0;

		}

 if(MASTER==1) //当为主机时，对共补状态信息进行处理
 	{
			if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

		 	
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
				if(RS485_RX_BUF[1]=='+'){OSMboxPost(RS485_RT,(void*)&RS485_RX_BUF);}
		 	
		} 
 	}
 // if((MASTER==0)&&(RS485_RX_BUF[RS485_RX_CNT-1]=='*')) RS485_RX_CNT=0;//直接清除，避免内存溢出和信息相互搅
	/********************************************************************************************/
 if(MASTER==1) //当为主机时，对分补状态信息进行处理
		{
			if(RS485_RX_BUF[RS485_RX_CNT-1]=='%'){RS485_RX_BUF[0]='%'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]==')')
	{
 	
				if((RS485_RX_BUF[1]=='(')&&(RS485_RX_CNT==10))
					{

/*		if(mybox.myid>RS485_RX_BUF[2])
				{
					MASTER=0;
					  hguestnum=222;
					dog_clock=20;
                      		OSTaskSuspend(APP_TASK_Master_PRIO );//挂起主机任状态.					
					OSTaskResume(APP_TASK_SLAVE3_PRIO );//启动从机任务状态	 
					}	
			
				else
				*/
					{OSMboxPost(RS485_STUTAS_MBOX_dis,(void*)&RS485_RX_BUF);}

				}
	RS485_RX_CNT=0;
		}	

		}
 if((MASTER==0)&&(RS485_RX_BUF[RS485_RX_CNT-1]==')')) RS485_RX_CNT=0;//直接清除，避免内存溢出和信息相互搅
/********************************************************************************************/
if(RS485_RX_CNT>=500)RS485_RX_CNT=0;
//if(RS485_RX_CNT>=512)RS485_RX_CNT=0;
	}  	
	OSIntExit();  											 

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	    GPIO_InitTypeDef GPIO_InitStructure;

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //本工程配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //推挽输出
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	
	RS485_TX_EN_1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
		delay_us(100);
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
	rs485buf[9]=L_C_flag_B;		
	rs485buf[10]='*';//协议尾
	RS485_Send_Data(rs485buf,11);//发送5个字节
	  // 	if(destination==source){mybox.send=send;slave_control(relay, message);}//如果信息发给的自己

    	}
//#endif
}
 u8 rs485_trans_computer(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机

 {
 	         
s16 sine_A=0;
 s16 sine_B=0;
 s16 sine_C=0;


#if (FUNCTION_MODULE ==  DF_THREE)
if(tx_r485[8]==CPT_A)
{
  dianya_zhi_A=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_A=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_A=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_A=tx_r485[7];
  if(tx_r485[9]==5)
  	{
  	  phase_flag=1;
  L_C_flag_A=0;
  	}
  if(tx_r485[9]==6)
  	{
  	  phase_flag=1;
  L_C_flag_A=1;
  	}
  if((tx_r485[9]==1)||(tx_r485[9]==0))
  	{
  phase_flag=0;
  L_C_flag_A=tx_r485[9];
  	}
return 1;
}

if(tx_r485[8]==CPT_B)
{
    dianya_zhi_B=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_B=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_B=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_B=tx_r485[7];
    L_C_flag_B=tx_r485[9];
return 1;

}
 
if(tx_r485[8]==CPT_C)
{
    dianya_zhi_C=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi_C=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar_C=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu_C=tx_r485[7];
    L_C_flag_C=tx_r485[9];
return 1;

}
  #endif


if(auto_on==1)
  	{
if(tx_r485[5]==CONTROL)

{
   if(mybox.myid==tx_r485[1])//判断是否是发给本机的信息或是广播信息
   	{
if((tempshuzhi<temperture_w)&&(temperature_warn==0))//自身温度是否正常 
{
	if( tx_r485[2]==1) 
	{
	 if( tx_r485[4]==0)
	 	{
	 	                if(status_box.work_status[tx_r485[3]-1]==1)
	 	{while(subswitchABC_onoff(tx_r485[3],0,1)==0)break;}
	 	}
if(tx_r485[4]==1)
{	 	               
            if((status_box.work_status[tx_r485[3]-1]==0)&&(rework_time[tx_r485[3]-1]==0))
	 	{while(subswitchABC_onoff(tx_r485[3],1,1)==1)break;}
}
if(tx_r485[4]==23)
{
                if(status_box.work_status[0]==1)
		   {while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
                 if(	status_box.work_status[1]==1)
		   {while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
                 if(	status_box.work_status[2]==1)
		   {while(subswitchABC_onoff(3,0,1)==0)break;}		  //投
}
if(tx_r485[4]==32)
{
                 if((status_box.work_status[0]==0)&&(rework_time[0]==0))
		   {while(subswitchABC_onoff(1,1,1)==1)break;}		  //投
                 if((status_box.work_status[1]==0)&&(rework_time[1]==0))
		   {while(subswitchABC_onoff(2,1,1)==1)break;}		  //投
                 if((status_box.work_status[2]==0)&&(rework_time[2]==0))
		   {while(subswitchABC_onoff(3,1,1)==1)break;}	
}
	return 1;

	}
}

	if( tx_r485[2]==2) 
		{
         status_trans_rs485_dis(&status_box);//从机程序
         return 1;

	    }

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
	if(phase_flag==1)rs485buf[9]=L_C_flag_A+5;//+5说明是反相序，把反相序这个信息传递给从机
	else rs485buf[9]=L_C_flag_A;
	rs485buf[10]=source;	
	rs485buf[11]='?';//协议尾
	RS485_Send_Data(rs485buf,12);//发送5个字节
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
	rs485buf[9]=L_C_flag_B;//协议尾
		rs485buf[10]=source;	
	rs485buf[11]='?';//协议尾
	RS485_Send_Data(rs485buf,12);//发送5个字节
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
	rs485buf[9]=L_C_flag_C;//协议尾
	rs485buf[10]=source;		
	rs485buf[11]='?';//协议尾
	RS485_Send_Data(rs485buf,12);//发送5个字节
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
      rs485buf[0]='-';//协议头
	rs485buf[1]=destination;
	rs485buf[2]=send;
	rs485buf[3]=relay;
	rs485buf[4]=message;
	rs485buf[5]=ctr;
	rs485buf[6]='=';//协议尾
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
  mybox.myid=AT24CXX_ReadOneByte(0xa000);
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;

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

BT_num=AT24CXX_ReadOneByte(0x0100);
warn_volt_onlimt=AT24CXX_ReadOneByte(0x1000);
if(BT_num==255){BT_num=1;AT24CXX_WriteOneByte(0x0100,BT_num);}
if(warn_volt_onlimt==255){warn_volt_onlimt=45;AT24CXX_WriteOneByte(0x1000,warn_volt_onlimt);}	
}

void set_now_mystatus(u8 myid,u8 size_1,u8 size_2,u8 size_3,u8 work_status_1 ,u8 work_status_2,u8 work_status_3)
 {
status_box.myid=myid;
status_box.size[0]=size_1;
status_box.size[1]=size_2;
status_box.size[2]=size_3;
status_box.work_status[0]=work_status_1;
status_box.work_status[1]=work_status_2;
status_box.work_status[2]=work_status_3;

 }

void set_statuslist(u8 count,u8 id,u8 size,u8 work_status,u8 work_time,u8 dis_comm,u8 relay,status_dis_node *dis_list,status_comm_node *comm_list_1,status_comm_node *comm_list_2,u8 group)
{
if(dis_comm==0)
{
if(relay==1)
        {
       dis_list[count].myid[0]=id;
   	   dis_list[count].size[0]=size;
   	   dis_list[count].work_status[0]=work_status;
       }
if(relay==2)
        {
	dis_list[count].myid[1]=id;
   	   dis_list[count].size[1]=size;
   	   dis_list[count].work_status[1]=work_status;
       }
if(relay==3)
        {
       dis_list[count].myid[2]=id;
   	   dis_list[count].size[2]=size;
   	   dis_list[count].work_status[2]=work_status;
       }
}
if(dis_comm==1)
{
  if(relay==1)
  	{
	   comm_list_1[count].myid=id;
   	   comm_list_1[count].size=size;
   	   comm_list_1[count].work_status=work_status;
	   comm_list_1[count].group=group;

      // comm_list[count].work_time[0]=work_time;
  	}
  if(relay==2)
  	{
	   comm_list_2[count].myid=id;
   	   comm_list_2[count].size=size;
   	   comm_list_2[count].work_status=work_status;
	   comm_list_2[count].group=group;

      // comm_list[count].work_time[1]=work_time;
  	}  
}

}
/**********************/

/****************************************************************************************************/
void init_Queue(u8 id,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i=0;
{
  if(size_1==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_1;
   	   comm_list[slave_comm[3]].work_status=work_status_1;
	   comm_list[slave_comm[3]].group=1;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_1==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_1;
   	   comm_list[slave_comm[5]].work_status=work_status_1;
	   comm_list[slave_comm[5]].group=1;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_1;
   	   comm_list[slave_comm[7]].work_status=work_status_1;
	   comm_list[slave_comm[7]].group=1;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_1;
   	   comm_list[slave_comm[9]].work_status=work_status_1;
	   comm_list[slave_comm[9]].group=1;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}



{
  if(size_2==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_2;
   	   comm_list[slave_comm[3]].work_status=work_status_2;
	   comm_list[slave_comm[3]].group=2;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_2==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_2;
   	   comm_list[slave_comm[5]].work_status=work_status_2;
	   comm_list[slave_comm[5]].group=2;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_2;
   	   comm_list[slave_comm[7]].work_status=work_status_2;
	   comm_list[slave_comm[7]].group=2;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_2;
   	   comm_list[slave_comm[9]].work_status=work_status_2;
	   comm_list[slave_comm[9]].group=2;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}

}

void init_listindex(u8 *slave_comm)
{
slave_comm[2]=0;//队列2v 标示初始化
slave_comm[3]=0;

slave_comm[4]=1;//队列5v 标示初始化
slave_comm[5]=1;

slave_comm[6]=2;//队列10v 标示初始化
slave_comm[7]=2;

slave_comm[8]=3;//队列20v 标示初始化
slave_comm[9]=3;

}

void change_Queue(u8 *slave_comm,status_comm_node *comm_list,u8 size)
{

u8 i=0;
u8 m,s,w,g;
{
  if(size==2)
  	{
  	m=comm_list[slave_comm[2]].myid;
	s=comm_list[slave_comm[2]].size;
	w=comm_list[slave_comm[2]].work_status;
	g=comm_list[slave_comm[2]].group;

	for(i=slave_comm[2];i<slave_comm[3]-1;i++)// 2 的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[3]-1].myid=m;
   	   comm_list[slave_comm[3]-1].size=s;
   	   comm_list[slave_comm[3]-1].work_status=w;
	   comm_list[slave_comm[3]-1].group=g;

  	}
  
    if(size==5)
    	{
  	m=comm_list[slave_comm[4]].myid;
	s=comm_list[slave_comm[4]].size;
	w=comm_list[slave_comm[4]].work_status;
	g=comm_list[slave_comm[4]].group;

	for(i=slave_comm[4];i<slave_comm[5]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[5]-1].myid=m;
   	   comm_list[slave_comm[5]-1].size=s;
   	   comm_list[slave_comm[5]-1].work_status=w;
	   comm_list[slave_comm[5]-1].group=g;

  	}
	if(size==10)
  	{
  	m=comm_list[slave_comm[6]].myid;
	s=comm_list[slave_comm[6]].size;
	w=comm_list[slave_comm[6]].work_status;
	g=comm_list[slave_comm[6]].group;

	for(i=slave_comm[6];i<slave_comm[7]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[7]-1].myid=m;
   	   comm_list[slave_comm[7]-1].size=s;
   	   comm_list[slave_comm[7]-1].work_status=w;
	   comm_list[slave_comm[7]-1].group=g;

  	} 
	if(size==20)
  	{
  	m=comm_list[slave_comm[8]].myid;
	s=comm_list[slave_comm[8]].size;
	w=comm_list[slave_comm[8]].work_status;
	g=comm_list[slave_comm[8]].group;

	for(i=slave_comm[8];i<slave_comm[9]-1;i++)//20的队列移动
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[9]-1].myid=m;
   	   comm_list[slave_comm[9]-1].size=s;
   	   comm_list[slave_comm[9]-1].work_status=w;
	   comm_list[slave_comm[9]-1].group=g;

  	}
}
}
/***********************************************
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list)
节点状态更新函数


*********************************************/
void flash_comm_list(u8 id,u8 size ,u8 work_status ,u8 group,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i;
if(size==2)
{
	for(i=slave_comm[2];i<slave_comm[3];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==5)
{
	for(i=slave_comm[4];i<slave_comm[5];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==10)
{
	for(i=slave_comm[6];i<slave_comm[7];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}

if(size==20)
{
	for(i=slave_comm[8];i<slave_comm[9];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{
	   comm_list[i].myid=id;
   	   comm_list[i].size=size;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   break;
		}
	}
}
}

void del_comm_listnode(u8 id,u8 group,u8 *slave_comm,status_comm_node *comm_list)
{
u8 i=0;
u8 j=0;
  	
{
  	  		for(i=slave_comm[2];i<=slave_comm[9]-1;i++)//2
	if(id==comm_list[i].myid&&group==comm_list[i].group)//锁定节点进行更新
		{ 

	
		for(j=i;j<slave_comm[9]-1;j++)
		      {
		         comm_list[j].myid=comm_list[j+1].myid;
   	   comm_list[j].size=comm_list[j+1].size;
   	   comm_list[j].work_status=comm_list[j+1].work_status;
	   comm_list[j].group=comm_list[j+1].group;
		      }
	       if(i<slave_comm[3])
		   	{
                         {
			slave_comm[3]--;
	             comm_list[slave_comm[3]].myid=0;
   	               comm_list[slave_comm[3]].size=0;
   	                comm_list[slave_comm[3]].work_status=0;
	              comm_list[slave_comm[3]].group=0;

		          }	
		   
                      {
			slave_comm[4]--;					  	
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[5]&&i>slave_comm[3])
			{
                         	   
                      {
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[7]&&i>slave_comm[5])
			{
                         		  
                      {
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[9]&&i>slave_comm[7])
			{
							  
	             {
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}

break;
	}



		 
  	 
  	
}

}

/**********************/
u8 inquiry_slave_status_comm(u8 id,u8 *slave_comm,status_comm_node *comm_list)   
  {  u8 *msg;
        u8 err;
	

   order_trans_rs485(mybox.myid,id,3,0,0,CONTROL);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/10,&err);
   if(err==OS_ERR_TIMEOUT)
   	{ return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{ 
if(msg[2]==id)
		{
 init_Queue(id,msg[3],msg[4],msg[5],msg[6],slave_comm,comm_list);
		
	return 1;
		}
else return 0;
	}

} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号


/**********************************/



  //查询从机状态并保存到从机状态表中，参数id是要查询的从机号

 
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

   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX_dis,OS_TICKS_PER_SEC/10,&err);
   if(err==OS_ERR_TIMEOUT)
   	{
          return 0;
   }//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	{ 
	if(msg[2]==id)//检查传过来的从机的状态信息是否真是该从机的。如果不是就不录入
		{
	rs485_trans_status_dis(count,msg,dis_list,comm_list);//主机状态信息写入状态表
	return 1;
		}
	else return 0;
	}

}
} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号
/**********************/
 void status_trans_rs485_dis(statusbox *mystatus)//从机程序
{  	
       rs485buf[0]='%';
	rs485buf[1]='(';
	rs485buf[2]=mystatus->myid;
	rs485buf[3]=mystatus->size[0];
	rs485buf[4]=mystatus->size[1];
	rs485buf[5]=mystatus->size[2];

	rs485buf[6]=mystatus->work_status[0];
	rs485buf[7]=mystatus->work_status[1];
	rs485buf[8]=mystatus->work_status[2];


	rs485buf[9]=')';
	RS485_Send_Data(rs485buf,10);//发送10个字节
}
/**************/
 void rs485_trans_status_dis(u8 count,u8 *tx_r485,status_dis_node *dis_list,status_comm_node *comm_list)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{
 	 set_statuslist(count,tx_r485[2],tx_r485[3],tx_r485[6],0,0,1,dis_list,comm_list,0,0);//主机状态信息写入状态表
	 set_statuslist(count,tx_r485[2],tx_r485[4],tx_r485[7],0,0,2,dis_list,comm_list,0,0);//主机状态信息写入状态表
      	  set_statuslist(count,tx_r485[2],tx_r485[5],tx_r485[8],0,0,3,dis_list,comm_list,0,0);//主机状态信息写入状态表

   } 
 	


/*********************************/
extern u8 flag_ABC;

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
 u8 TR[]={1,2,3,4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};//2//TR[ ]*50为实际变比

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
s32 gl[2];
u16 wugongkvar_95,wugongkvar_95A,wugongkvar_95B,wugongkvar_95C;
static u8 init=1;
static u8 warning_flag=0;
u8 bt,warn_vlot;
bt=AT24CXX_ReadOneByte(0x0100);
warn_vlot=AT24CXX_ReadOneByte(0x1000);
T=TR[bt];


/*********************电压判断相序*******************************/
if(init==1)
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

	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//C相初始相位


}
if((angle[0]-angle[1])>0)
{
phase=((angle[0]-angle[1])*360)/PI2;
if(phase>=118&&phase<=122)phase_flag=0;//正序
else phase_flag=1;
}
else 
	{
	phase=((angle[1]-angle[0])*360)/PI2;
if(phase>=238&&phase<=242)phase_flag=0;//正序
else phase_flag=1;


     }


/*********************电流相序******************************

 ADC3_CH1_DMA_Config_CA_phase();
ADC1_CH7_DMA_Config_CC();


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
	 
	arm_rfft_f32(&S, testInput_V,testOutput); 

             testIndex=1;
		angle[0]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//A相初始相位

	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//C相初始相位


}
if((angle[0]-angle[1])>0)
{
phase=((angle[0]-angle[1])*360)/PI2;
if(phase>=118&&phase<=122)phase_flag_c=0;//正序
else phase_flag_c=1;
}
else 
	{
	phase=((angle[1]-angle[0])*360)/PI2;
if(phase>=238&&phase<=242)phase_flag_c=0;//正序
else phase_flag_c=1;


     }

*********************电流相序end*****************************/

/********************初始第一屏根据相序显示ABC**************************/
{
	if(phase_flag==0)
		flag_ABC=1;
	if(phase_flag==1)
		flag_ABC=3;
		Clera_lcd();
		if(flag_ABC==1)
	Graf_con_u(gonglvshishu_A,dianya_zhi_A,L_C_flag_A);
				if(flag_ABC==3)
	Graf_con_u(gonglvshishu_C,dianya_zhi_C,L_C_flag_C);

		
}
/***************************************************/
	init=0;
}
/************************判断相序end**************************/

/*********************A_phase*********************************/
//for(s=1;s<=9;s++)
{
	if(phase_flag==0)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==1)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}

//if(phase_flag==0&&phase_flag_c==0)ADC1_CH1_DMA_Config_CA();
//if(phase_flag==1&&phase_flag_c==1)ADC1_CH1_DMA_Config_CA();
//if(phase_flag==0&&phase_flag_c==1)ADC1_CH7_DMA_Config_CC();
//if(phase_flag==1&&phase_flag_c==0)ADC1_CH7_DMA_Config_CC();

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
dianya_zhi_A=dianya_zhi_A/2.57;
	
/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_A=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_A=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_A=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_A=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_A=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_A=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_A=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_A=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
 dianliuzhi_A=0.98*maxValue_C;
dianliuzhi_A=T*dianliuzhi_A;
 if(dianliuzhi_A<=2500*T){dianliuzhi_A=0;gonglvshishu_A=100;L_C_flag_A=1;}
else{ 
	dianliuzhi_A=dianliuzhi_A/1000;
	gonglvshishu_A=arm_cos_f32(angle[2])*100;//功率因素
}


arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
        a=dianya_zhi_A*dianliuzhi_A*sine/10;
	wugongkvar_A=dianya_zhi_A*dianliuzhi_A*sine/1000;
      wugongkvar_95A=dianya_zhi_A*dianliuzhi_A*0.3122/1000;
				//	L_C_flag_A=1;

}


computer_trans_rs485(mybox.myid,33,0,0,0,CPT_A);

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
dianya_zhi_B=dianya_zhi_B/2.57;

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);


/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_B=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_B=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_B=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_B=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_B=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_B=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_B=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_B=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
dianliuzhi_B=0.98*maxValue_C;
 dianliuzhi_B=T*dianliuzhi_B;
 if(dianliuzhi_B<=2500*T){dianliuzhi_B=0;gonglvshishu_B=100;L_C_flag_B=1;}
else {
        dianliuzhi_B=dianliuzhi_B/1000;
	gonglvshishu_B=arm_cos_f32(angle[2])*100;//功率因素
}
arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
         b=dianya_zhi_B*dianliuzhi_B*sine/10;
	wugongkvar_B=dianya_zhi_B*dianliuzhi_B*sine/1000;
      wugongkvar_95B=dianya_zhi_B*dianliuzhi_B*0.3122/1000;
			


}
computer_trans_rs485(mybox.myid,33,0,0,0,CPT_B);

/*********************C_phase*********************************/

{
	if(phase_flag==1)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==0)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}

//if(phase_flag==0&&phase_flag_c==0)ADC1_CH7_DMA_Config_CC();
//if(phase_flag==1&&phase_flag_c==1)ADC1_CH7_DMA_Config_CC();
//if(phase_flag==0&&phase_flag_c==1)ADC1_CH1_DMA_Config_CA();
//if(phase_flag==1&&phase_flag_c==0)ADC1_CH1_DMA_Config_CA();
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
dianya_zhi_C=dianya_zhi_C/2.57;


/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

	//	arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_C=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_C=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_C=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_C=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_C=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_C=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_C=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_C=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
dianliuzhi_C=0.98*maxValue_C;
 dianliuzhi_C=T*dianliuzhi_C;
 if(dianliuzhi_C<=2500*T){dianliuzhi_C=0;gonglvshishu_C=100;L_C_flag_C=1;}
else
	{
	dianliuzhi_C=dianliuzhi_C/1000;
	gonglvshishu_C=arm_cos_f32(angle[2])*100;//功率因素
}
arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
           c=dianya_zhi_C*dianliuzhi_C*sine/10;
	wugongkvar_C=dianya_zhi_C*dianliuzhi_C*sine/1000;
      wugongkvar_95C=dianya_zhi_C*dianliuzhi_C*0.3122/1000;
			

}



/****************************************************/
computer_trans_rs485(mybox.myid,33,0,0,0,CPT_C);

/***************************************************/
//inquiry_slave_status_dis(3,dis_list,comm_list);   

/*********************ALL***********************************/
dianya_zhi=1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3;
dianliuzhi=(dianliuzhi_A+dianliuzhi_B+dianliuzhi_C)/3;
gonglvshishu=(gonglvshishu_A+gonglvshishu_B+gonglvshishu_C)/3;
wugongkvar=wugongkvar_A+wugongkvar_B+wugongkvar_C;
  wugongkvar_95=wugongkvar_95A+wugongkvar_95B+wugongkvar_95C;

   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);


}

/*********************变比判断*******************************/
//if(1)
if(RT_FLAG==0)
{
        u8 err;
if(L_C_flag_B==1)gl[0]=wugongkvar;
else gl[0]=-wugongkvar;

{
      	{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==0)

{

order_trans_rs485(mybox.myid,comm_list[i].myid,4,comm_list[i].group,1,CONTROL);
 OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)RT_FLAG=0;
else 
		{
//set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//change_Queue(1,20,dis_list,comm_list,slave_dis,slave_comm);
RT_FLAG=1;
var=var+(200*dianya_zhi*dianya_zhi)/450/450;
//var=166+var;
		}
//return 0 ;
}


      	}


{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==0)
	{

order_trans_rs485(mybox.myid,comm_list[i].myid,4,comm_list[i].group,1,CONTROL);
OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
  if(err==OS_ERR_TIMEOUT)RT_FLAG=0;
else 
		{
//set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//change_Queue(1,10,dis_list,comm_list,slave_dis,slave_comm);
RT_FLAG=1;
var=var+(100*dianya_zhi*dianya_zhi)/450/450;
//var=var+100;
		}
//return 0 ;
}





	  }


{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==0)
	{

order_trans_rs485(mybox.myid,comm_list[i].myid,4,comm_list[i].group,1,CONTROL);
  OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)RT_FLAG=0;
	else 
		{
//set_statuslist(i,comm_list[i].myid,comm_list[i].size[0],1,comm_list[i].work_time[0],1,1,dis_list,comm_list);
//change_Queue(1,5,dis_list,comm_list,slave_dis,slave_comm);
RT_FLAG=1;
var=var+(50*dianya_zhi*dianya_zhi)/450/450;
//var=var+50;
		}
//return 0 ;
}



      	}




      
}

}
 
if(RT_FLAG==1)
{
u16 min;
delay_us(2500000);//36->512

//u16 min=2;
/******************************/
{
	if(phase_flag==0)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==1)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}


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
/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_A=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_A=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_A=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_A=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_A=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_A=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_A=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_A=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/

 dianliuzhi_A=0.98*maxValue_C;
dianliuzhi_A=T*dianliuzhi_A/1000;
gonglvshishu_A=arm_cos_f32(angle[2])*100;//功率因素

//dianya_zhi_A=0;
//	dianya_zhi_A=comm_list[slave_comm[5]].myid;

//gonglvshishu_A=0;
//	gonglvshishu_A=comm_list[slave_comm[5]].size[0];

arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
        a=dianya_zhi_A*dianliuzhi_A*sine/10;
	wugongkvar_A=dianya_zhi_A*dianliuzhi_A*sine/1000;
      wugongkvar_95A=dianya_zhi_A*dianliuzhi_A*0.3122/1000;
				//	L_C_flag_A=1;

}




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
/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_B=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_B=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_B=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_B=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_B=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_B=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_B=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_B=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
 dianliuzhi_B=0.98*maxValue_C;
dianliuzhi_B=T*dianliuzhi_B/1000;
gonglvshishu_B=arm_cos_f32(angle[2])*100;//功率因素
arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
         b=dianya_zhi_B*dianliuzhi_B*sine/10;
	wugongkvar_B=dianya_zhi_B*dianliuzhi_B*sine/1000;
      wugongkvar_95B=dianya_zhi_B*dianliuzhi_B*0.3122/1000;

}


/*********************C_phase*********************************/

{
	if(phase_flag==1)
		{
ADC3_CH10_DMA_Config_VA();
ADC1_CH1_DMA_Config_CA();
		}
		if(phase_flag==0)
			{
ADC3_CH12_DMA_Config_VC();
ADC1_CH7_DMA_Config_CC();
		}

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
/****************************************************************/
				angle[2]=((angle[1]-angle[0])*360)/PI2;

				  if(angle[2]>0){while(1){if(angle[2]>360){angle[2]=angle[2]-360;} else break;}}
				else if(angle[2]<0){while(1){if(angle[2]<-360){angle[2]=angle[2]+360;} else break;}}

					  
				if(angle[2]>0.0)
                               {
				if(angle[2]<90)L_C_flag_C=1;
				if(angle[2]>90&&angle[2]<180)L_C_flag_C=0;				
				if(angle[2]>180&&angle[2]<270)L_C_flag_C=1;
				if(angle[2]>270&&angle[2]<360)L_C_flag_C=0;


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))L_C_flag_C=0;
					if((angle[2]>-180.0&&angle[2]<-90.0))L_C_flag_C=1;
					if((angle[2]>-270.0&&angle[2]<-180.0))L_C_flag_C=0;
					if((angle[2]>-360.0&&angle[2]<-270.0))L_C_flag_C=1;

			     }

			if(angle[2]>0.0)
                               {
				if(angle[2]>90&&angle[2]<180)angle[2]=-(angle[2]-180);				
				if(angle[2]>180&&angle[2]<270)angle[2]=angle[2]-180;
				if(angle[2]>270&&angle[2]<360)angle[2]=-(angle[2]-360);


				}

				else if(angle[2]<=0.0)
				{
					if((angle[2]>-90.0&&angle[2]<=0.0))angle[2]=-angle[2];
					if((angle[2]>-180.0&&angle[2]<-90.0))angle[2]=(angle[2]+180);
					if((angle[2]>-270.0&&angle[2]<-180.0))angle[2]=-(angle[2]+180);
					if((angle[2]>-360.0&&angle[2]<-270.0))angle[2]=(angle[2]+360);

			     }


        
angle[2]=((angle[2])*PI2)/360;


/***************************************************************/
 dianliuzhi_C=0.98*maxValue_C;
dianliuzhi_C=T*dianliuzhi_C/1000;
gonglvshishu_C=arm_cos_f32(angle[2])*100;//功率因素
arm_sqrt_f32(1-(arm_cos_f32(angle[2]))*(arm_cos_f32(angle[2])),&sine);
           c=dianya_zhi_C*dianliuzhi_C*sine/10;
	wugongkvar_C=dianya_zhi_C*dianliuzhi_C*sine/1000;
      wugongkvar_95C=dianya_zhi_C*dianliuzhi_C*0.3122/1000;



}



/*********************ALL***********************************/
dianya_zhi=1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3;
dianliuzhi=(dianliuzhi_A+dianliuzhi_B+dianliuzhi_C)/3;
gonglvshishu=(gonglvshishu_A+gonglvshishu_B+gonglvshishu_C)/3;
wugongkvar=(a+b+c)/100;
  wugongkvar_95=wugongkvar_95A+wugongkvar_95B+wugongkvar_95C;


}
/*******************************/
if(L_C_flag_B==1)gl[1]=wugongkvar;
else gl[1]=-wugongkvar;
min=abs(abs(gl[0]-gl[1])*TR[0]-var);

for(i=1;i<16;i++)
{
if(abs(abs(gl[0]-gl[1])*TR[i]-var)<=min){min=abs(abs(gl[0]-gl[1])*TR[i]-var);T=TR[i];}

}
RT_FLAG=2;
//tempshuzhi=T;
order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);
delay_us(1000000);
order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);
delay_us(1000000);
return 0;
}

//tempshuzhi=T;
/**************************end*************************/
if(RT_FLAG==2)

{

{
if((ON_OFF==1)&&(KEY1==1&&auto_on==0))
	{   
	light_time=30;
		  auto_on=1;
                 if(	status_box.work_status[0]==1)
		   {subswitchABC_onoff(1,0,1);}		  //投
                 if(	status_box.work_status[1]==1)
		   {subswitchABC_onoff(2,0,1);}		  //投
                 if(	status_box.work_status[2]==1)
		   {subswitchABC_onoff(3,0,1);}		  //投

 }
     if((ON_OFF==0)&&(KEY1==0&&auto_on==1))
 	{
	light_time=30;
		auto_on=0;
                 if((status_box.work_status[0]==0)&&(rework_time[0]==0))
		   {subswitchABC_onoff(1,1,1);}		  //投
                 if((status_box.work_status[1]==0)&&(rework_time[1]==0))
		   {subswitchABC_onoff(2,1,1);}		  //投
                 if((status_box.work_status[2]==0)&&(rework_time[2]==0))
		   {subswitchABC_onoff(3,1,1);}		  //投

	 }

		  }









/**************************************过压保护**/
{
if((dianya_zhi>1.732*(warn_vlot+200)||dianya_zhi<310))
{

LIGHT_backligt_off(2,2,2);

}

if((dianya_zhi>1.732*(warn_vlot+200)||dianya_zhi<310)&&warning_flag==0)
{
  if(	status_box.work_status[0]==1)
		   {while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
                 if(	status_box.work_status[1]==1)
		   {while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
                 if(	status_box.work_status[2]==1)
		   {while(subswitchABC_onoff(3,0,1)==0)break;}		  //投
		   
order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);
delay_ms(5000);
order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);
delay_ms(5000);
warning_flag=1;
}
if(warning_flag==1&&dianya_zhi<=1.732*(warn_vlot+200-7)&&dianya_zhi>=313)
	{warning_flag=0;
status_box.work_status[0]=3;
status_box.work_status[1]=3;
status_box.work_status[2]=3;
rework_time[0]=1;
rework_time[1]=1;
rework_time[2]=1;
LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);

}
}
/**************************************过压保护END**/

if(dianya_zhi<=1.732*(warn_vlot+200)&&dianya_zhi>=310&&warning_flag==0&&temperature_warn==0)
{
if(gonglvshishu<AF_DEAD&&L_C_flag_B==1)
 {
      {
      if(wugongkvar>=20)
      	{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
 change_Queue(slave_comm,comm_list,20);			
delay_ms(TIME_TQ);
return 0 ;

		}
}


      	}


	  if(wugongkvar>=10)
{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
 change_Queue(slave_comm,comm_list,10);			
delay_ms(TIME_TQ);
return 0 ;

		}
}

	  }


	  if(wugongkvar>=5)

{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
 change_Queue(slave_comm,comm_list,5);			
delay_ms(TIME_TQ);
return 0 ;

		}
}

	  }


	  if(wugongkvar>=2)

{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
 change_Queue(slave_comm,comm_list,2);			
delay_ms(TIME_TQ);
return 0 ;

		}
}

	  }


      
}
 }

if(gonglvshishu>AF_TOP&&L_C_flag_B==1)
   
{
      {

{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}
	  
{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}



}




{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}

{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}


       }
 }

if(1)

  {
if(gonglvshishu_A<AF_DEAD&&L_C_flag_A==1)
{
if(slave_dis[0]>0)
      if(wugongkvar_A>=6)	
{
for(i=slave_dis[3];i<=slave_dis[9]-1;i++)
if(dis_list[i].work_status[0]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,1,CONTROL);
dis_list[i].work_status[0]=1;
change_Queue_dis(0,6,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}


if(slave_dis[0]>0)
      if(wugongkvar_A>=3)	
{
for(i=slave_dis[2];i<=slave_dis[8]-1;i++)
if(dis_list[i].work_status[0]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,1,CONTROL);
dis_list[i].work_status[0]=1;
change_Queue_dis(0,3,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}

if(slave_dis[0]>0)
      if(wugongkvar_A>=1)	
{
for(i=slave_dis[1];i<=slave_dis[7]-1;i++)
if(dis_list[i].work_status[0]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,1,CONTROL);
dis_list[i].work_status[0]=1;
change_Queue_dis(0,1,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}	
   if(wugongkvar_A>=status_box.size[0])	
{
if((status_box.work_status[0]==0)&&(rework_time[0]==0))
{
{while(subswitchABC_onoff(1,1,1)==1)break;}		  //投
status_box.work_status[0]=1;
delay_ms(TIME_TQ);

return 0;
}

}	  
}
if(gonglvshishu_B<AF_DEAD&&L_C_flag_B==1)
{
if(slave_dis[0]>0)
      if(wugongkvar_B>=6)	
{
for(i=slave_dis[6];i<=slave_dis[12]-1;i++)
if(dis_list[i].work_status[1]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,1,CONTROL);
dis_list[i].work_status[1]=1;
change_Queue_dis(1,6,dis_list,slave_dis);
delay_ms(TIME_TQ);

//delay_ms(1000);
return 0;
}

}

if(slave_dis[0]>0)
      if(wugongkvar_B>=3)	
{
for(i=slave_dis[5];i<=slave_dis[11]-1;i++)
if(dis_list[i].work_status[1]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,1,CONTROL);
dis_list[i].work_status[1]=1;
change_Queue_dis(1,3,dis_list,slave_dis);
delay_ms(TIME_TQ);

//delay_ms(1000);
return 0;
}

}


if(slave_dis[0]>0)
      if(wugongkvar_B>=1)	
{
for(i=slave_dis[4];i<=slave_dis[10]-1;i++)
if(dis_list[i].work_status[1]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,1,CONTROL);
dis_list[i].work_status[1]=1;
change_Queue_dis(1,1,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}

						
   if(wugongkvar_B>=status_box.size[1])	
{
if((status_box.work_status[1]==0)&&(rework_time[1]==0))
{
{while(subswitchABC_onoff(2,1,1)==1)break;}		  //投
status_box.work_status[1]=1;

delay_ms(TIME_TQ);
return 0;
}

}
}

if(gonglvshishu_C<AF_DEAD&&L_C_flag_C==1)

{
if(slave_dis[0]>0)
      if(wugongkvar_C>=6)	
{
for(i=slave_dis[17];i<=slave_dis[18]-1;i++)
if(dis_list[i].work_status[2]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,1,CONTROL);
dis_list[i].work_status[2]=1;
change_Queue_dis(2,6,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}
if(slave_dis[0]>0)
      if(wugongkvar_C>=3)	
{
for(i=slave_dis[15];i<=slave_dis[16]-1;i++)
if(dis_list[i].work_status[2]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,1,CONTROL);
dis_list[i].work_status[2]=1;
change_Queue_dis(2,3,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}

if(slave_dis[0]>0)
      if(wugongkvar_C>=1)	
{
for(i=slave_dis[13];i<=slave_dis[14]-1;i++)
if(dis_list[i].work_status[2]==0)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,1,CONTROL);
dis_list[i].work_status[2]=1;
change_Queue_dis(2,1,dis_list,slave_dis);

//delay_ms(1000);
delay_ms(TIME_TQ);
return 0;
}

}


   if(wugongkvar_C>=status_box.size[2])	
{
if((status_box.work_status[2]==0)&&(rework_time[2]==0))
{
{while(subswitchABC_onoff(3,1,1)==1)break;}		  //投
status_box.work_status[2]=1;

delay_ms(TIME_TQ);
return 0;
}

}	  
}
	
  }

if(1)

{


if(gonglvshishu_A>AF_TOP&&L_C_flag_A==1)
{
if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[0]==1)
{
{while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
status_box.work_status[0]=0;

delay_ms(TIME_TQ);
return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[1];i<=slave_dis[7]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
delay_ms(TIME_TQ);
return 0;
}

}	  

if(slave_dis[0]>0)
{
for(i=slave_dis[2];i<=slave_dis[8]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
delay_ms(TIME_TQ);
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[3];i<=slave_dis[9]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
delay_ms(TIME_TQ);
return 0;
}

}

}
if(gonglvshishu_B>AF_TOP&&L_C_flag_B==1)
{
if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[1]==1)
{
{while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
status_box.work_status[1]=0;

delay_ms(TIME_TQ);
return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[4];i<=slave_dis[10]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
delay_ms(TIME_TQ);
return 0;
}

}


if(slave_dis[0]>0)
{
for(i=slave_dis[5];i<=slave_dis[11]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
delay_ms(TIME_TQ);
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[6];i<=slave_dis[12]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
delay_ms(TIME_TQ);
return 0;
}

}



}

if(gonglvshishu_C>AF_TOP&&L_C_flag_C==1)

{

if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[2]==1)
{
{while(subswitchABC_onoff(3,0,1)==0)break;}		  //投
status_box.work_status[2]=0;

delay_ms(TIME_TQ);
return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[13];i<=slave_dis[14]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
delay_ms(TIME_TQ);
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[15];i<=slave_dis[16]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
delay_ms(TIME_TQ);
return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[17];i<=slave_dis[18]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
delay_ms(TIME_TQ);
return 0;
}

}

}
	
  }


if(L_C_flag_B==0)
{
      {

{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}
	  
{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}



}




{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}

{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}


}


       }
}

{
if(1)
{
/************************A*****************************/
if(L_C_flag_A==0)
{
if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[0]==1)
{
{while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
status_box.work_status[0]=0;

return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[3];i<=slave_dis[9]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
return 0;
}

}


if(slave_dis[0]>0)
{
for(i=slave_dis[2];i<=slave_dis[8]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[1];i<=slave_dis[7]-1;i++)
if(dis_list[i].work_status[0]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[0],1,1,0,CONTROL);
dis_list[i].work_status[0]=0;
return 0;
}

}	  
}

/************************B*****************************/
if(L_C_flag_B==0)
{
if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[1]==1)
{
{while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
status_box.work_status[1]=0;

return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[6];i<=slave_dis[12]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[5];i<=slave_dis[11]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
return 0;
}

}


if(slave_dis[0]>0)
{
for(i=slave_dis[4];i<=slave_dis[10]-1;i++)
if(dis_list[i].work_status[1]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[1],1,2,0,CONTROL);
dis_list[i].work_status[1]=0;
return 0;
}

}


}

/************************C*****************************/
if(L_C_flag_C==0)

{
if(KEY1==1)//必须是在自动条件下
{
if(status_box.work_status[2]==1)
{
{while(subswitchABC_onoff(3,0,1)==0)break;}		  //投
status_box.work_status[2]=0;

return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[17];i<=slave_dis[18]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
return 0;
}

}
if(slave_dis[0]>0)
{
for(i=slave_dis[15];i<=slave_dis[16]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
return 0;
}

}

if(slave_dis[0]>0)
{
for(i=slave_dis[13];i<=slave_dis[14]-1;i++)
if(dis_list[i].work_status[2]==1)
{
computer_trans_rs485(mybox.myid,dis_list[i].myid[2],1,3,0,CONTROL);
dis_list[i].work_status[2]=0;
return 0;
}

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
u8 i,j=0,g,flag_comm=0,flag_dis=0,s;
u8 g_1,g_2,g_3,f_1,f_2,f_3;
//u8 c;
u8 *msg;
  u8 err;
  static u8 dis_err[7];
    static u8 comm_err[32];
for(i=1;i<=7;i++)
{  

for(g=1;g<=slave_dis[0];g++)
{
if(i==dis_list[g].myid[0]||i==dis_list[g].myid[1]||i==dis_list[g].myid[2]){flag_dis=1;break;}
else flag_dis=0;
}
if(flag_dis==0)
		{
//	for(c=1;c<=2;c++)
		{
	j=inquiry_slave_status_dis(slave_dis[0]+1,i,dis_list,comm_list); 
	        if(j==1){ slave_dis[0]++;break;}
		}
			}
if(flag_dis==1)
{
 computer_trans_rs485(mybox.myid,i,2,0,0,CONTROL);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX_dis,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)
	 	{
	 	dis_err[i-1]++;
	 	if(dis_err[i-1]==3)//三次确认，如果三次都没有收到数据就认为是从机死了
	 	{
		dis_err[i-1]=0;
for(g_1=1;g_1<=slave_dis[0];g_1++)
{
if(i==dis_list[g_1].myid[0])
	{
	dis_list[g_1].work_status[0]=2;dis_list[g_1].myid[0]=0;
         break;
       }

}
for(g_2=1;g_2<=slave_dis[0];g_2++)
{
if(i==dis_list[g_2].myid[1])
	{
	dis_list[g_2].work_status[1]=2;dis_list[g_2].myid[1]=0;
         break;
       }

}


for(g_3=1;g_3<=slave_dis[0];g_3++)
{
if(i==dis_list[g_3].myid[2])
	{
	dis_list[g_3].work_status[2]=2;dis_list[g_3].myid[2]=0;
         break;
       }

}
for(f_1=g_1;f_1<slave_dis[0];f_1++)
{
dis_list[f_1].myid[0]=dis_list[f_1+1].myid[0];
dis_list[f_1].work_status[0]=dis_list[f_1+1].work_status[0];
dis_list[f_1].size[0]=dis_list[f_1+1].size[0];

}

for(f_2=g_2;f_2<slave_dis[0];f_2++)
{
dis_list[f_2].myid[1]=dis_list[f_2+1].myid[1];
dis_list[f_2].work_status[1]=dis_list[f_2+1].work_status[1];
dis_list[f_2].size[1]=dis_list[f_2+1].size[1];

}

for(f_3=g_3;f_3<slave_dis[0];f_3++)
{
dis_list[f_3].myid[2]=dis_list[f_3+1].myid[2];
dis_list[f_3].work_status[2]=dis_list[f_3+1].work_status[2];
dis_list[f_3].size[2]=dis_list[f_3+1].size[2];

}
slave_dis[0]--;

}
	     }
else if(msg[2]==i)//检查传过来的从机的状态信息是否真是该从机的。如果不是就不更新
{
dis_err[i-1]=0;
		for(s=1;s<=slave_dis[0];s++)
			{
                   if(i==dis_list[s].myid[0])dis_list[s].work_status[0]=msg[6];
                   if(i==dis_list[s].myid[1])dis_list[s].work_status[1]=msg[7];
                   if(i==dis_list[s].myid[2])dis_list[s].work_status[2]=msg[8];
			}
}

}

	flag_dis=0;
       j=0;
    }

//delay_ms(1000);

j=0;
{
for(i=slave_dis[0]+1;i<=32;i++)
	{  

for(g=0;g<=slave_comm[9]-1;g++)
{
if(i==comm_list[g].myid){flag_comm=1;break;}

else flag_comm=0;
}
if(flag_comm==0)
		{		
 inquiry_slave_status_comm(i,slave_comm,comm_list);   
			
			}
//以下是为了更新从机的投切状态
if(flag_comm==1)

{
{order_trans_rs485(mybox.myid,i,3,0,0,CONTROL); 
  msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)
	 	{
	  	comm_err[i-1]++; 
if(comm_err[i-1]==6)
	  {
 	  	comm_err[i-1]=0; 
 
/*******************从已知队列中删除该节点***********************************/


 del_comm_listnode(i,1,slave_comm,comm_list);
 del_comm_listnode(i,2,slave_comm,comm_list);
//slave_comm[0]=slave_comm[0]-2;
 /******************从已知队列中删除该节点end***********************************/

}


	 }
else  if(msg[2]==i)
	{
		  	comm_err[i-1]=0; 
 
	if(flag_comm==1)
		{
		 flash_comm_list(i,msg[3] ,msg[5] ,1,slave_comm,comm_list);
		 flash_comm_list(i,msg[4] ,msg[6] ,2,slave_comm,comm_list);
		}
       }

}
	


}
	flag_comm=0;
       j=0;
    } 
//delay_ms(1000);

}

}

/**************************************************************/

/******************************************************************/




 	


void init_Queue_dis(status_dis_node *dis_list,u8 *slave_dis)

{

u8 i,j;

u8 t=0;
u8 g=0;
u8 s=0;
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[0];
	   g=dis_list[i].myid[0];
	   s=	dis_list[i].work_status[0];
	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[0])
	   		{
	   	dis_list[j+1].myid[0]=dis_list[j].myid[0];
               dis_list[j+1].size[0]=dis_list[j].size[0];
		 	dis_list[j+1].work_status[0]=dis_list[j].work_status[0];

	   		}
		else break;
		}
	   dis_list[j+1].myid[0]=g;
	   dis_list[j+1].size[0]=t;
            dis_list[j+1].work_status[0]=s;
}
for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==1)
{
slave_dis[1]=i;
break;
}
if(i>slave_dis[0]){slave_dis[1]=0;slave_dis[7]=0;}

if(slave_dis[1]!=0)
{
for(i=slave_dis[1];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=1)
{
slave_dis[7]=i;
break;
}
if(i>slave_dis[0]){slave_dis[7]=slave_dis[0]+1;}

}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==3)
{
slave_dis[2]=i;
break;
}
if(i>slave_dis[0]){slave_dis[2]=0;slave_dis[8]=0;}

if(slave_dis[2]!=0)
{
for(i=slave_dis[2];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=3)
{
slave_dis[8]=i;
break;
}
if(i>slave_dis[0]){slave_dis[8]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[0]==6)
{
slave_dis[3]=i;
break;
}
if(i>slave_dis[0]){slave_dis[3]=0;slave_dis[9]=0;}
if(slave_dis[3]!=0)
{
for(i=slave_dis[3];i<=slave_dis[0];i++)
if(dis_list[i].size[0]!=6)
{
slave_dis[9]=i;
break;
}
if(i>slave_dis[0]){slave_dis[9]=slave_dis[0]+1;}

}




}


/***************************************************/
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[1];
	   g=dis_list[i].myid[1];//设置myid两个
	   s=	dis_list[i].work_status[1];

	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[1])
	   		{
		dis_list[j+1].myid[1]=dis_list[j].myid[1];
               dis_list[j+1].size[1]=dis_list[j].size[1];
		 dis_list[j+1].work_status[1]=dis_list[j].work_status[1];

	   		}
		else break;
	       }
	   dis_list[j+1].myid[1]=g;
	   dis_list[j+1].size[1]=t;
            dis_list[j+1].work_status[1]=s;


}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==1)
{
slave_dis[4]=i;
break;
}
if(i>slave_dis[0]){slave_dis[4]=0;slave_dis[10]=0;}

if(slave_dis[4]!=0)
{
for(i=slave_dis[4];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=1)
{
slave_dis[10]=i;
break;
}
if(i>slave_dis[0]){slave_dis[10]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==3)
{
slave_dis[5]=i;
break;
}
if(i>slave_dis[0]){slave_dis[5]=0;slave_dis[11]=0;}

if(slave_dis[5]!=0)
{
for(i=slave_dis[5];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=3)
{
slave_dis[11]=i;
break;
}
if(i>slave_dis[0]){slave_dis[11]=slave_dis[0]+1;}

}




for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[1]==6)
{
slave_dis[6]=i;
break;
}
//slave_comm[12]=slave_comm[0]+1;
if(i>slave_dis[0]){slave_dis[6]=0;slave_dis[12]=0;}
if(slave_dis[6]!=0)
{
for(i=slave_dis[6];i<=slave_dis[0];i++)
if(dis_list[i].size[1]!=6)
{
slave_dis[12]=i;
break;
}
if(i>slave_dis[0]){slave_dis[12]=slave_dis[0]+1;}

}
}
/***********************************/
{
for(i=2;i<=slave_dis[0];i++)
{
  
          t=dis_list[i].size[2];
	   g=dis_list[i].myid[2];//设置myid两个
	   s=	dis_list[i].work_status[2];

	   for(j=i-1;j>=1;j--)
	   	{
	   	if(t<dis_list[j].size[2])
	   		{
		dis_list[j+1].myid[2]=dis_list[j].myid[2];
               dis_list[j+1].size[2]=dis_list[j].size[2];
		 dis_list[j+1].work_status[2]=dis_list[j].work_status[2];

	   		}
		else break;
	       }
	   dis_list[j+1].myid[2]=g;
	   dis_list[j+1].size[2]=t;
            dis_list[j+1].work_status[2]=s;


}


for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==1)
{
slave_dis[13]=i;
break;
}
if(i>slave_dis[0]){slave_dis[13]=0;slave_dis[14]=0;}

if(slave_dis[13]!=0)
{
for(i=slave_dis[13];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=1)
{
slave_dis[14]=i;
break;
}
if(i>slave_dis[0]){slave_dis[14]=slave_dis[0]+1;}

}



for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==3)
{
slave_dis[15]=i;
break;
}
if(i>slave_dis[0]){slave_dis[15]=0;slave_dis[16]=0;}

if(slave_dis[15]!=0)
{
for(i=slave_dis[15];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=3)
{
slave_dis[16]=i;
break;
}
if(i>slave_dis[0]){slave_dis[16]=slave_dis[0]+1;}

}




for(i=1;i<=slave_dis[0];i++)
if(dis_list[i].size[2]==6)
{
slave_dis[17]=i;
break;
}
//slave_comm[12]=slave_comm[0]+1;
if(i>slave_dis[0]){slave_dis[17]=0;slave_dis[18]=0;}
if(slave_dis[17]!=0)
{
for(i=slave_dis[17];i<=slave_dis[0];i++)
if(dis_list[i].size[2]!=6)
{
slave_dis[18]=i;
break;
}
if(i>slave_dis[0]){slave_dis[18]=slave_dis[0]+1;}

}
}




/********************************/
}

 
/*********************************************************************/
void change_Queue_dis(u8 abc,u8 Level, status_dis_node *dis_list,u8 *slave_dis)
{
u8 i;
u8 t=0, g=0,s=0;

{
if(Level==1)
	
{
if(abc==0)
{
if(slave_dis[1]!=0)

{
          t=dis_list[slave_dis[1]].size[abc];
	   g=dis_list[slave_dis[1]].myid[abc];
	   s=	dis_list[slave_dis[1]].work_status[abc];
for(i=slave_dis[1];i<slave_dis[7]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[7]-1].size[abc]=t;
	  dis_list[slave_dis[7]-1].myid[abc]=g;
	  dis_list[slave_dis[7]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[4]!=0)

{
          t=dis_list[slave_dis[4]].size[abc];
	   g=dis_list[slave_dis[4]].myid[abc];
	   s=	dis_list[slave_dis[4]].work_status[abc];
for(i=slave_dis[4];i<slave_dis[10]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[10]-1].size[abc]=t;
	  dis_list[slave_dis[10]-1].myid[abc]=g;
	  dis_list[slave_dis[10]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[13]!=0)

{
          t=dis_list[slave_dis[13]].size[abc];
	   g=dis_list[slave_dis[13]].myid[abc];
	   s=	dis_list[slave_dis[13]].work_status[abc];
for(i=slave_dis[13];i<slave_dis[14]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[14]-1].size[abc]=t;
	  dis_list[slave_dis[14]-1].myid[abc]=g;
	  dis_list[slave_dis[14]-1].work_status[abc]=s;
}
}
}
/***********************************************************************************/
/***********************************************************************************/
if(Level==3)

{
if(abc==0)
{
if(slave_dis[2]!=0)

{
          t=dis_list[slave_dis[2]].size[abc];
	   g=dis_list[slave_dis[2]].myid[abc];
	   s=	dis_list[slave_dis[2]].work_status[abc];
for(i=slave_dis[2];i<slave_dis[8]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[8]-1].size[abc]=t;
	  dis_list[slave_dis[8]-1].myid[abc]=g;
	  dis_list[slave_dis[8]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[5]!=0)

{
          t=dis_list[slave_dis[5]].size[abc];
	   g=dis_list[slave_dis[5]].myid[abc];
	   s=	dis_list[slave_dis[5]].work_status[abc];
for(i=slave_dis[5];i<slave_dis[11]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[11]-1].size[abc]=t;
	  dis_list[slave_dis[11]-1].myid[abc]=g;
	  dis_list[slave_dis[11]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[15]!=0)

{
          t=dis_list[slave_dis[15]].size[abc];
	   g=dis_list[slave_dis[15]].myid[abc];
	   s=	dis_list[slave_dis[15]].work_status[abc];
for(i=slave_dis[15];i<slave_dis[16]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[16]-1].size[abc]=t;
	  dis_list[slave_dis[16]-1].myid[abc]=g;
	  dis_list[slave_dis[16]-1].work_status[abc]=s;
}
}
}
/***********************************************************************************/
/***********************************************************************************/
if(Level==6)
{
if(abc==0)
{
if(slave_dis[3]!=0)

{
          t=dis_list[slave_dis[3]].size[abc];
	   g=dis_list[slave_dis[3]].myid[abc];
	   s=	dis_list[slave_dis[3]].work_status[abc];
for(i=slave_dis[3];i<slave_dis[9]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[9]-1].size[abc]=t;
	  dis_list[slave_dis[9]-1].myid[abc]=g;
	  dis_list[slave_dis[9]-1].work_status[abc]=s;
}
}
/***********************************************************************************/
if(abc==1)

{
if(slave_dis[6]!=0)

{
          t=dis_list[slave_dis[6]].size[abc];
	   g=dis_list[slave_dis[6]].myid[abc];
	   s=	dis_list[slave_dis[6]].work_status[abc];
for(i=slave_dis[6];i<slave_dis[12]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[12]-1].size[abc]=t;
	  dis_list[slave_dis[12]-1].myid[abc]=g;
	  dis_list[slave_dis[12]-1].work_status[abc]=s;
}
}

/***********************************************************************************/
if(abc==2)
{
if(slave_dis[17]!=0)

{
          t=dis_list[slave_dis[17]].size[abc];
	   g=dis_list[slave_dis[17]].myid[abc];
	   s=	dis_list[slave_dis[17]].work_status[abc];
for(i=slave_dis[17];i<slave_dis[18]-1;i++)
  {
          dis_list[i].size[abc]=dis_list[i+1].size[abc];
	   dis_list[i].myid[abc]=dis_list[i+1].myid[abc];
	   dis_list[i].work_status[abc]=dis_list[i+1].work_status[abc];

  }
        dis_list[slave_dis[18]-1].size[abc]=t;
	  dis_list[slave_dis[18]-1].myid[abc]=g;
	  dis_list[slave_dis[18]-1].work_status[abc]=s;
}
}
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
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}

 void TIM4_IRQHandler(void)   //TIM3中断
{	  static u16 count_rework[3];
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志
                     IWDG_Feed();

	 if(rework_time[0]==1)
 	{
 	count_rework[0]++;
	if(count_rework[0]==500)//240
		{
count_rework[0]=0;
rework_time[0]=0;
status_box.work_status[0]=0;

	  }
	}
  if(rework_time[1]==1)
 	{
 	count_rework[1]++;
	if(count_rework[1]==500)//240
		{
count_rework[1]=0;
rework_time[1]=0;
status_box.work_status[1]=0;
	  }
	}
  if(rework_time[2]==1)
 	{
 	count_rework[2]++;
	if(count_rework[2]==500)//240
		{
count_rework[2]=0;
rework_time[2]=0;
status_box.work_status[2]=0;
	  }
	}
		}
	   	OSIntExit();  

 	}

/**************************************************************************/
void TIM3_Int_Init(u16 arr,u16 psc)

{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}
void turn_master_id(u8 id)//改变当前整个系统中主机的ID号
{
   u8 flag=0;
          OS_CPU_SR cpu_sr=0;  	    	
 
	{ 
	  flag=cont;
      if(id==(flag)){
	  	   order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);
	//delay_time(2);
		OS_ENTER_CRITICAL();    		
MASTER=1;
OSTaskResume(APP_TASK_Master_PRIO);
	OS_EXIT_CRITICAL();
	
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }

 void TIM3_IRQHandler(void)   //TIM3中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志
	if(MASTER==0)	
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
	if(light_time>1)light_time--;
 if(light_time==1)LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
	
		}
	   	OSIntExit();  

 	}


/******************************************************/
void EXTI_Configuration(void)//初始化函数
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//打开时钟
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	 		
	//使能外部中断复用时钟
	
	//映射GPIOE的Pin0至EXTILine0
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);




EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);         	//嵌套分组为组0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//中断通道为通道10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//开中断
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line12);

}


void EXTI15_10_IRQHandler(void)
{
	OSIntEnter();   

  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	
	{
	delay_us(1000);
if(KEY1==1&&auto_on==0)ON_OFF=1;
 if(KEY1==0&&auto_on==1)ON_OFF=0;

	}
      EXTI_ClearITPendingBit(EXTI_Line12);

	   	OSIntExit();  

}

/*************************************************/
void LIGHT_backligt_on(u8 status_1,u8 status_2,u8 status_3)
{
if(((status_1==0||status_1==3)&&status_2==1&&(status_3==0||status_3==3)))HT595_Send_Byte((GREEN_RED_GREEN)|background_light_on);
if((status_1==1&&(status_2==0||status_2==3)&&(status_3==0||status_3==3)))HT595_Send_Byte((RED_GREEN_GREEN)|background_light_on);
if(((status_1==0||status_1==3)&&(status_2==0||status_2==3)&&(status_3==0||status_3==3)))HT595_Send_Byte((GREEN_GREEN_GREEN)|background_light_on);
if((status_1==1&&status_2==1&&(status_3==0||status_3==3)))HT595_Send_Byte((RED_RED_GREEN)|background_light_on);

if(((status_1==0||status_1==3)&&status_2==1&&status_3==1))HT595_Send_Byte((GREEN_RED_RED)|background_light_on);
if((status_1==1&&(status_2==0||status_2==3)&&status_3==1))HT595_Send_Byte((RED_GREEN_RED)|background_light_on);
if(((status_1==0||status_1==3)&&(status_2==0||status_2==3)&&status_3==1))HT595_Send_Byte((GREEN_GREEN_RED)|background_light_on);
if(status_1==1&&status_2==1&&status_3==1)HT595_Send_Byte((RED_RED_RED)|background_light_on);

if(status_1==2&&status_2==2&&status_3==2)HT595_Send_Byte((YELLOW_YELLOW_YELLOW)|background_light_on);

}


/*************************************************/
void LIGHT_backligt_off(u8 status_1,u8 status_2,u8 status_3)
{
if(((status_1==0||status_1==3)&&status_2==1&&(status_3==0||status_3==3)))HT595_Send_Byte((GREEN_RED_GREEN));
if((status_1==1&&(status_2==0||status_2==3)&&(status_3==0||status_3==3)))HT595_Send_Byte((RED_GREEN_GREEN));
if(((status_1==0||status_1==3)&&(status_2==0||status_2==3)&&(status_3==0||status_3==3)))HT595_Send_Byte((GREEN_GREEN_GREEN));
if((status_1==1&&status_2==1&&(status_3==0||status_3==3)))HT595_Send_Byte((RED_RED_GREEN));

if(((status_1==0||status_1==3)&&status_2==1&&status_3==1))HT595_Send_Byte((GREEN_RED_RED));
if((status_1==1&&(status_2==0||status_2==3)&&status_3==1))HT595_Send_Byte((RED_GREEN_RED));
if(((status_1==0||status_1==3)&&(status_2==0||status_2==3)&&status_3==1))HT595_Send_Byte((GREEN_GREEN_RED));
if(status_1==1&&status_2==1&&status_3==1)HT595_Send_Byte((RED_RED_RED)|background_light_on);

if(status_1==2&&status_2==2&&status_3==2)HT595_Send_Byte((YELLOW_YELLOW_YELLOW));

}


/*************************************************/


void temperature()   //电容器温度检测
{
tempshuzhi=(u8)(258-((ADC_Temperature()*255)/4096));

}




/*************************************************/
void warn()
{
static u8 warning_flag=0;
		   temperature();   //电容器温度检测

/************从机功能 温度报警************************/
{
if(tempshuzhi>=temperture_w&&temperature_warn==0&&auto_on==1)
{
 if(	status_box.work_status[0]==1)
		   {while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
                 if(	status_box.work_status[1]==1)
		   {while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
                 if(	status_box.work_status[2]==1)
		   {while(subswitchABC_onoff(3,0,1)==0)break;}

	status_box.work_status[0]=2;	
	status_box.work_status[1]=2;			 
	status_box.work_status[2]=2;	
	LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);

	 temperature_warn=1;
}

if(tempshuzhi<=temperture_w-5&&temperature_warn==1)
{temperature_warn=0;
status_box.work_status[0]=3;
status_box.work_status[1]=3;
status_box.work_status[2]=3;
rework_time[0]=1;
rework_time[1]=1;
rework_time[2]=1;
LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);

}
}

/************从机功能 温度报警END************************/

/**************************************过压保护*
{

if(((1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3)>420||(1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3)<330)&&warning_flag==0)
{
 if(MASTER==0)
{
 if(	status_box.work_status[0]==1)
		   {while(subswitchABC_onoff(1,0,1)==0)break;}		  //投
                 if(	status_box.work_status[1]==1)
		   {while(subswitchABC_onoff(2,0,1)==0)break;}		  //投
                 if(	status_box.work_status[2]==1)
		   {while(subswitchABC_onoff(3,0,1)==0)break;}
status_box.work_status[0]=2;
status_box.work_status[1]=2;
status_box.work_status[2]=2;	
LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);
}
warning_flag=1;
}
if(warning_flag==1&&(1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3)<=417&&(1.732*(dianya_zhi_A+dianya_zhi_B+dianya_zhi_C)/3)>=333)
	{

status_box.work_status[0]=0;
status_box.work_status[1]=0;
status_box.work_status[2]=0;
	
LIGHT_backligt_off(status_box.work_status[0],status_box.work_status[1],status_box.work_status[2]);

	warning_flag=0;
     }
}
*************************************过压保护END**/


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



