/*********************************************************************************************************
*
* File                : LCDConf.h
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


#ifndef LCDCONF_H
#define LCDCONF_H

/* Private define ------------------------------------------------------------*/
#define HY32D              (8989)

#define LCD_XSIZE          (320)
#define LCD_YSIZE          (240)
#define LCD_CONTROLLER     (HY32D)
#define LCD_BITSPERPIXEL   (16)
#define LCD_FIXEDPALETTE   (565)
#define LCD_SWAP_RB        (1)
//#define LCD_SWAP_XY        (1)
#define LCD_INIT_CONTROLLER()  LCD_Initializtion()


#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

