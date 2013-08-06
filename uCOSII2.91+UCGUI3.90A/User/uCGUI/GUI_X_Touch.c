/*
*********************************************************************************************************
*                                                uC/GUI
*                        Universal graphic software for embedded applications
*
*                       (c) Copyright 2002, Micrium Inc., Weston, FL
*                       (c) Copyright 2002, SEGGER Microcontroller Systeme GmbH
*
*              �C/GUI is protected by international copyright laws. Knowledge of the
*              source code may not be used to write a similar product. This file may
*              only be used in accordance with a license and should not be redistributed
*              in any way. We appreciate your understanding and fairness.
*
----------------------------------------------------------------------
File        : GUI_TOUCH_X.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/

/* Includes ------------------------------------------------------------------*/
#include <includes.h>


void GUI_TOUCH_X_ActivateX(void) {
}

void GUI_TOUCH_X_ActivateY(void) {
}

int  GUI_TOUCH_X_MeasureX(void) {
   return 0;
}

int  GUI_TOUCH_X_MeasureY(void) {
   return 0;
}


extern  int xPhys, yPhys;
int CalibrationComplete = 0;

void GUI_TOUCH_Measure(void) {
    Coordinate * Ptr;

	xPhys = -1; yPhys = -1;

    Ptr = Read_Ads7846();

	if( Ptr != (void*)0 )
	{
	  xPhys = Ptr->x;
	  yPhys = Ptr->y;  
	}
}


