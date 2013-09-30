//#include <stm32f10x_lib.h>
#include "ht1621.h"
#include "delay.h"
#include "key.h"
//#include "led.h"
	   
u8 num456Seg[]={0x0d,0x07,0x00,0x06,0x0e,0x03,0x0a,0x07,0x03,0x06,0x0b,0x05,0x0f,0x05,0x00,0x07,0x0f,0x07,0x0b,0x07 };
u8 num1237Seg[]={0x07,0x0d,0x06,0x00,0x03,0x0e,0x07,0x0a,0x06,0x03,0x05,0x0b,0x05,0x0f,0x07,0x00,0x07,0x0f,0x07,0x0b};
extern u8 L_C_flag_A;//感性容性标准变量

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

//初始化PB4 PB5和PB6为输出口.并使能这PE时钟		    
//HT1621 IO初始化
void HT1621_Init(void)
{		
	/*RCC->APB2ENR|=1<<3;    //使能PORTB时钟
											  

      	GPIOB->CRH&=0XFFFFFF0F;
	GPIOB->CRH|=0X00000030;//PB9

	GPIOB->CRL&=0X000FFFFF;
	GPIOB->CRL|=0X33300000;//PB 4 5 6推挽输出

	GPIOB->ODR|=1<<9;      //PB9输出高 
	GPIOB->ODR|=1<<5;      //PB5输出高 
	GPIOB->ODR|=1<<6;      //PB6输出高
	GPIOB->ODR|=1<<7;      //PB7输出高 
	*/
  GPIO_InitTypeDef      GPIO_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits( GPIOB, GPIO_Pin_9);
   GPIO_SetBits( GPIOB, GPIO_Pin_5);
  GPIO_SetBits( GPIOB, GPIO_Pin_6);
  GPIO_SetBits( GPIOB, GPIO_Pin_7);
 
	SendCmd(LCDOFF);
	SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
}

void SendBit_1621(u8 data,u8 cnt)	 //data高cnt位写入HT1621,高位在前
{
  u8 i;
  for(i=0;i<cnt;i++)
  {
   if((data&0x80)==0)DATA_0;
   else DATA_1;
   WR_0;
   delay_us(1);
   WR_1;
      delay_us(1);
   data<<=1;
  }
}
void SendDataBit_1621(u8 data,u8 cnt)	 //data低cnt位写入HT1621,低位在前
{
  u8 i;
  for(i=0;i<cnt;i++)
  {
   if((data&0x01)==0)DATA_0;
   else DATA_1;
   WR_0;
   delay_us(1);
   WR_1;
      delay_us(1);
   data>>=1;
  }
}

void SendCmd(u8 command)
{
   CS_0;
   SendBit_1621(0x80,3);
   SendBit_1621(command,9);
   CS_1;
   
}

void Write_1621(u8 addr,u8 data)
{
   CS_0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   SendDataBit_1621(data,4);
   CS_1;
}

void WriteAll_1621(u8 addr,u8 *p,u8 cnt)
{
   u8 i;
   CS_0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   for(i=0;i<cnt;i++,p++)
   {
   	   SendDataBit_1621(*p,4);
   }
   CS_1;
}

void Clera_lcd(void)
{
	 u8 t;
	 for(t=0;t<22;t++)
	 {
	   u8 i;
	   for(i=0;i<4;i++)
	   {		  
	   	Write_1621(t,0x00<<i);
	   }
	 }
}

void Graf_con_u(u8 cos,u16 volt)
{
	u8 temp,coszhengshu,cosshifen,cosbaifen;
	u16 voltbaiwei,voltshiwei,voltgewei;

	Write_1621(11,0x01);   //显示U(v)
	Write_1621(13,0x01);   //显示cos
	Write_1621(14,0x08);   //显示dot.
	Write_1621(17,0x08);   //显示auto
	
	temp=cos;
if( L_C_flag_A==1)
{
	coszhengshu=temp/100;
	WriteAll_1621(15,num1237Seg+2*coszhengshu,2);	//显示cos整数部分
}
if( L_C_flag_A==0)

{
	Write_1621(16,0x02);	//显示cos整数部分
}
	cosshifen=(temp%100)/10;
	WriteAll_1621(19,num1237Seg+2*cosshifen,2);	 	//显示cos十分位部分

	cosbaifen=(temp%10)%10;
	WriteAll_1621(17,num1237Seg+2*cosbaifen,2);	   //显示cos百分位部分

	voltbaiwei=volt/100;
	WriteAll_1621(2,num456Seg+2*voltbaiwei,2);	  //显示volt百位部分

	voltshiwei=(volt%100)/10;
	WriteAll_1621(5,num456Seg+2*voltshiwei,2);	  //显示volt十位部分

	voltgewei=volt%10;
	WriteAll_1621(8,num1237Seg+2*voltgewei,2);	  //显示volt个位部分

}

void Graf_cuirrent(u32 current)
{
   u32 temp;
   u16 currshiwei,currgewei,currshifenwei,currbaifenwei,currqianfenwei;

	Write_1621(12,0x01);	//显示C-I(A)
//	Write_1621(13,0x08);	//显示dot4 .
	Write_1621(17,0x08);	//显示auto

   temp=current;

   currshiwei=temp/10000;
   WriteAll_1621(15,num1237Seg+2*currshiwei,2);	//显示C-I(A)整数部分十位

   currgewei=(temp%10000)/1000;
   WriteAll_1621(19,num1237Seg+2*currgewei,2);	//显示C-I(A)整数部分个位

   currshifenwei=(temp%1000)/100;
   WriteAll_1621(17,num1237Seg+2*currshifenwei,2);	//显示C-I(A)小数部分十分位

   currbaifenwei=(temp%100)/10;
   WriteAll_1621(0,num456Seg+2*currbaifenwei,2);	//显示C-I(A)小数部分百分位

   currqianfenwei=temp%10;
   WriteAll_1621(2,num456Seg+2*currqianfenwei,2);	//显示C-I(A)小数部分千分位

}

void Graf_qkvar(u16 qkvar)
{
   u32 temp;
   u16 qkvarshiwei,qkvargewei,qkvarshifenwei,qkvarbaifenwei,qkvarqianfenwei;

	Write_1621(10,0x01);	//显示Q(Kvar)
//	Write_1621(11,0x08);	//显示dot8 .
	Write_1621(17,0x08);	//显示auto

   temp=qkvar;

   qkvarshiwei=temp/10000;
   WriteAll_1621(17,num1237Seg+2*qkvarshiwei,2);	//显示Q(Kvar)整数部分十位

   qkvargewei=(temp%10000)/1000;
   WriteAll_1621(0,num456Seg+2*qkvargewei,2);	//显示Q(Kvar)整数部分个位

   qkvarshifenwei=(temp%1000)/100;
   WriteAll_1621(2,num456Seg+2*qkvarshifenwei,2);	//显示Q(Kvar)小数部分十分位

   qkvarbaifenwei=(temp%100)/10;
   WriteAll_1621(5,num456Seg+2*qkvarbaifenwei,2);	//显示Q(Kvar)小数部分百分位

   qkvarqianfenwei=temp%10;
   WriteAll_1621(8,num1237Seg+2*qkvarqianfenwei,2);	//显示Q(Kvar)小数部分千分位

}

void Graf_temp(u8 temp)
{
	u8 tempbaiwei,tempshiwei,tempgewei;

	Write_1621(4,0x01);	//显示TEMP
	Write_1621(17,0x08);	//显示auto

	tempbaiwei=temp/100;
	WriteAll_1621(2,num456Seg+2*tempbaiwei,2);	//显示TEMP百数位

	tempshiwei=(temp%100)/10;
	WriteAll_1621(5,num456Seg+2*tempshiwei,2);	//显示TEMP十数位

	tempgewei=temp%10;
	WriteAll_1621(8,num1237Seg+2*tempgewei,2);	//显示TEMP个数位

}

void Graf_id(u8 hostguest,u8 id)
{
	u8 h_gbaiwei,h_gshiwei,h_ggewei,idbaiwei,idshiwei,idgewei;

	Write_1621(7,0x04);	//显示ID
	Write_1621(17,0x08);	//显示auto

	h_gbaiwei=hostguest/100;
	WriteAll_1621(15,num1237Seg+2*h_gbaiwei,2);	//显示机号百位数

	h_gshiwei=(hostguest%100)/10;
	WriteAll_1621(19,num1237Seg+2*h_gshiwei,2);	//显示机号十位数

	h_ggewei=hostguest%10;
	WriteAll_1621(17,num1237Seg+2*h_ggewei,2);	//显示机号个位数

	idbaiwei=id/100;
	WriteAll_1621(2,num456Seg+2*idbaiwei,2);	//显示ID百数位

	idshiwei=(id%100)/10;
	WriteAll_1621(5,num456Seg+2*idshiwei,2);	//显示ID十数位

	idgewei=id%10;
	WriteAll_1621(8,num1237Seg+2*idgewei,2);	//显示ID个数位

}

void Graf_ver(u8 ver)
{
   u8 verbaiwei,vershiwei,vergewei;
   
   Write_1621(4,0x04);	//显示dot10.
   Write_1621(10,0x02);	//显示ver
   Write_1621(17,0x08);	//显示auto

   verbaiwei=ver/100;
   WriteAll_1621(2,num456Seg+2*verbaiwei,2);	//显示VER百数位

   vershiwei=(ver%100)/10;
   WriteAll_1621(5,num456Seg+2*vershiwei,2);	//显示VER十数位

   vergewei=ver%10;
   WriteAll_1621(8,num1237Seg+2*vergewei,2);	//显示VER个数位
}

void Graf_setid(u8 idnum)
{   
   u8 idnumbaiwei,idnumshiwei,idnumgewei;
  
   Write_1621(1,0x08);	//显示set
   Write_1621(7,0x04);	//显示ID
   
   idnumbaiwei=idnum/100;
   WriteAll_1621(2,num456Seg+2*idnumbaiwei,2);	//显示idnum百数位

   idnumshiwei=(idnum%100)/10;
   WriteAll_1621(5,num456Seg+2*idnumshiwei,2);	//显示idnum十数位

   idnumgewei=idnum%10;
   WriteAll_1621(8,num1237Seg+2*idnumgewei,2);	//显示idnum个数位

}

void HT595_Send_Byte(u8 state)
{                        
    u8 t; 
		RCLK_595_0;		    
    for(t=0;t<8;t++)
    {    
		DATA_0;          
        if((state&0x80)==0)
		WR_0;
		else WR_1;
		delay_us(3);
		DATA_1;
        state<<=1; 	  

    }

	delay_us(10);
	RCLK_595_1;
}
