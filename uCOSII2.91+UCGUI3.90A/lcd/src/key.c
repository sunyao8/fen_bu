//#include <stm32f10x_lib.h>
#include "key.h"
#include "delay.h"
#include "ht1621.h"
//#include "led.h"
//#include "exti.h"
#include "24cxx.h" 	
static u8 m=1;
static u8 grafnum=1;
u8 zhongduan_flag=1;
u8 id_num=0;
u8 grafnum,tempshuzhi,vernum=101,hguestnum=222,gonglvshishu=0;
u16 dianya_zhi=0,wugongkvar=0;
u32	dianliuzhi=0;
//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//KEY ���� PA11Ϊ��ʾ�����ð�����PA12Ϊ�ֶ�Ͷ�п���	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//������ʼ������
void KEY_Init(void)
{/*
	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	GPIOA->CRH&=0XFFF00FFF;	//PA11 PA12���ó�����	  
	GPIOA->CRH|=0X00088000; 
	GPIOA->ODR|=1<11;		// ����
	GPIOA->ODR|=1<12;		  //����
	*/
		 GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
 // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

} 

void key_idset(void)
{
	
	u8 h=0;
	if((KEY0==0)&&m)
	{
           	  // delay_us(10000);
	   m=0;
	 while(KEY0==0)
	   	{
	   	   delay_us(3000);//3000
		   h++;
		   if(h>=180)break;
	   
	   	}
			   if(h>=180)//180
			   {		
					zhongduan_flag=0;
						
			   		Clera_lcd();
		
			   		Graf_setid(id_num);
			   //Graf_con_u(gonglvshishu,dianya_zhi);

			   }
			   else
				   {  
				     if(zhongduan_flag==1)
				      	{
					  		grafnum++;
					  		if(grafnum>6)grafnum=1;
					  
					    	  switch(grafnum)
								{				 
									case 1:	//��ʾ���������͵�ѹֵ
										Clera_lcd();
										Graf_con_u(gonglvshishu,dianya_zhi);
										break;
									case 2:	//��ʾ����
										Clera_lcd();
										Graf_cuirrent(dianliuzhi);
										break;
									case 3:	//��ʾ�޹�����	 
										Clera_lcd();
										Graf_qkvar(wugongkvar);
										break;
									case 4:	//��ʾ�¶� 
										Clera_lcd();
										Graf_temp(tempshuzhi);
										break;
					
									case 5:	//��ʾID 
										Clera_lcd();
										Graf_id(hguestnum,id_num);
										break;
					
									case 6:	//��ʾVER 
										Clera_lcd();
										Graf_ver(vernum);
										break;
					
								}
					 	}
						if(zhongduan_flag==0)
				      	{
					  		id_num++;
					  		if(id_num>32)id_num=0;
							Clera_lcd();
	   						Graf_setid(id_num);
			//			AT24CXX_WriteOneByte(0x0010,id_num);
						}
				   }
	
	}
	else if(KEY0==1)
		{
                  	  // delay_us(10000);
			m=1;
			 while(KEY0==1)
			 {
		   	   delay_us(2500);//2500
			   h++;
			   if(h>=200)break;
	   
	   		 } 
			   if(h>=200)//200
				 {
						  zhongduan_flag=1;
					  	  switch(grafnum)
							{				 
								case 1:	//��ʾ���������͵�ѹֵ
									Clera_lcd();
									Graf_con_u(gonglvshishu,dianya_zhi);
									break;
								case 2:	//��ʾ����
									Clera_lcd();
									Graf_cuirrent(dianliuzhi);
									break;
								case 3:	//��ʾ�޹�����	 
									Clera_lcd();
									Graf_qkvar(wugongkvar);
									break;
								case 4:	//��ʾ�¶� 
									Clera_lcd();
									Graf_temp(tempshuzhi);
									break;
				
								case 5:	//��ʾID 
									Clera_lcd();
									Graf_id(hguestnum,id_num);
									break;
				
								case 6:	//��ʾVER 
									Clera_lcd();
									Graf_ver(vernum);
									break;
				
							}	
				 }

		}
}




















