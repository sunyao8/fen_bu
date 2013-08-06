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
static  OS_STK         App_TaskStartStk[APP_TASK_START_STK_SIZE];



/* Private function prototypes -----------------------------------------------*/
#if (OS_VIEW_MODULE == DEF_ENABLED)
extern void  App_OSViewTaskCreate   (void);
#endif

static  void  App_TaskCreate		(void);
static  void  App_TaskStart			(void		*p_arg);  
extern  void  App_UCGUI_TaskCreate  (void);
static  void  GPIO_Configuration    (void);


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
void ADC1_CH10_DMA_Config(void);
void ADC2_CH4_DMA_Config(void);
void ADC3_CH13_DMA_Config(void);
void allphase(float32_t *V,float32_t *I);

#define TEST_LENGTH_SAMPLES 512*2 
 
/* ------------------------------------------------------------------- 
* External Input and Output buffer Declarations for FFT Bin Example 
* ------------------------------------------------------------------- */ 
 float32_t testInput_V[TEST_LENGTH_SAMPLES]; 
 float32_t testInput_C[TEST_LENGTH_SAMPLES]; 

 float32_t testInput_V_source[TEST_LENGTH_SAMPLES/2]; 
 float32_t testInput_C_source[TEST_LENGTH_SAMPLES/2]; 

float32_t testOutput[TEST_LENGTH_SAMPLES*2/2]; 
float32_t reslut[TEST_LENGTH_SAMPLES/2]; 

/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint32_t fftSize = 512; 
uint32_t ifftFlag = 0; 
uint32_t doBitReverse = 1; 
 
/* Reference index at which max energy of bin ocuurs */ 
uint32_t  testIndex = 0; 
 double angle[2]; 

/* ---------------------------------------------------------------------- 
* Max magnitude FFT Bin test 
* ------------------------------------------------------------------- */ 





INT32S main (void)
{
/*
	CPU_INT08U  os_err;
	os_err = os_err; 

	CPU_IntDis();                   

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
	*/
/************************************************/
{  
int i=0;
arm_status status; 
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32  S_CFFT;
float32_t maxValue=0.0,maxValue_C=0.0; 
///	(void)p_arg;
	
	/***************  Init hardware ***************/

	GPIO_Configuration();

    OS_CPU_SysTickInit();                                    /* Initialize the SysTick.                              */

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                            /* Determine CPU capacity.                              */
#endif

  //  App_TaskCreate();                                        /* Create application tasks.                            */
delay_init();

HT1621_Init();
//AT24CXX_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  
ADC1_CH10_DMA_Config();
ADC2_CH4_DMA_Config();
ADC3_CH13_DMA_Config();

    ADC_SoftwareStartConv(ADC1);
  ADC_SoftwareStartConv(ADC2);
    ADC_SoftwareStartConv(ADC3);


	for(;;)
   	{
   	

 for(i=0;i<TEST_LENGTH_SAMPLES;i++)
	 	{
	 	
	 	
testInput_C[i]=(float32_t)((ADC_Converted_CValue-ADC_Converted_base)*3.3/4096);///  1550

testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);///  1550

delay_us(36);//36->512

        }
 for(i=0;i<TEST_LENGTH_SAMPLES/2;i++)
 {
testInput_C_source[i]=testInput_C[i];
testInput_V_source[i]=testInput_V[i];
 }//保存原始数据 用于计算电流电压值

 
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
		arm_rfft_f32(&S, testInput_V_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	/* Calculates maxValue and returns corresponding BIN value */ 

	arm_max_f32(reslut, fftSize/2, &maxValue, &testIndex);
	dianya_zhi=maxValue*2;

/******************************************************************/
	arm_rfft_f32(&S, testInput_C,testOutput); 
         
	angle[1]=atan2(testOutput[2*testIndex],testOutput[2*testIndex+1]);//电流初始相位

	/*******通过原始数据计算电压值***********/

		arm_rfft_f32(&S, testInput_C_source,testOutput); 

	arm_cmplx_mag_f32(testOutput, reslut,  
	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
	arm_max_f32(reslut, fftSize/2, &maxValue_C, &testIndex);

 dianliuzhi=maxValue_C/2;

gonglvshishu=arm_cos_f32(angle[0]-angle[1])*100;//功率因素

 key_idset();//按键与显示功能
//delay_ms(100);

    }	
}
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
int i=0;
arm_status status; 
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32  S_CFFT;
 float32_t maxValue=0.0; 
	(void)p_arg;
	
	/***************  Init hardware ***************/

	GPIO_Configuration();

    OS_CPU_SysTickInit();                                    /* Initialize the SysTick.                              */

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                            /* Determine CPU capacity.                              */
#endif

  //  App_TaskCreate();                                        /* Create application tasks.                            */
delay_init();

HT1621_Init();
//AT24CXX_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  
ADC1_CH10_DMA_Config();
ADC3_CH13_DMA_Config();

    ADC_SoftwareStartConv(ADC1);
    ADC_SoftwareStartConv(ADC3);


	for(;;)
   	{
   	
  // 	dianliuzhi=ADC_Converted_VValue;
//wugongkvar=ADC_Converted_VValue;
//gongluyinshu();
 for(i=0;i<512;i++)
	 	{
testInput_V[i]=(float32_t)((ADC_Converted_VValue-ADC_Converted_base)*3.3/4096);
        }
	/* Initialize the CFFT/CIFFT module */  
	status = arm_rfft_init_f32(&S,&S_CFFT, fftSize,  
	  								ifftFlag, doBitReverse); 
	 
	/* Process the data through the CFFT/CIFFT module */ 
	arm_rfft_f32(&S, testInput_V,testOutput); 
	 
	 
	/* Process the data through the Complex Magnitude Module for  
	calculating the magnitude at each bin */ 
///	arm_cmplx_mag_f32(testInput_f32_10khz, testOutput,  
///	  				fftSize);  
	 
	/* Calculates maxValue and returns corresponding BIN value */ 
///	arm_max_f32(testOutput, fftSize, &maxValue, &testIndex); 
wugongkvar=maxValue;


 
 key_idset();//按键与显示功能
//delay_ms(100);

    }	
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
#if (OS_VIEW_MODULE == DEF_ENABLED)
	App_OSViewTaskCreate();
#endif	
	
	App_UCGUI_TaskCreate();	

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
  	/* GPIOF Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	/* Configure PF6 PF7 PF8 PF9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* Configure PF6 PF7 PF8 PF9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

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


void ADC1_CH10_DMA_Config(void)

{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration *************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
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
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

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
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC1, ENABLE);

}


void ADC3_CH13_DMA_Config(void)

{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

//  DMA_DeInit(DMA2_Stream0);
  /* DMA2 Stream0 channe0 configuration *************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
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
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream1, ENABLE);

  /* Configure ADC1 Channel10 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

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

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

}

void ADC2_CH4_DMA_Config(void)

{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);
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
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
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
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC1 regular channe6 configuration *************************************/
  ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC2, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC2, ENABLE);

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
