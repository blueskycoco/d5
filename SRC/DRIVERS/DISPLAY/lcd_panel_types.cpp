//*********************************************************************
//* Software that is described herein is for illustrative purposes only  
//* which provides customers with programming information regarding the  
//* products. This software is supplied "AS IS" without any warranties.  
//* NXP Semiconductors assumes no responsibility or liability for the 
//* use of the software, conveys no license or title under any patent, 
//* copyright, or mask work right to the product. NXP Semiconductors 
//* reserves the right to make changes in the software without 
//* notification. NXP Semiconductors also make no representation or 
//* warranty that such application will be suitable for the specified 
//* use without further testing or modification. 
//*
//* Copyright NXP Semiconductors
//*********************************************************************
//
// lcd_panel_types.c
//
// List of supported panel configurations
//

#include "precomp.h"
#include "lcd_panel_types.h"

//------------------------------------------------------------------------------

LCD_PARAM_T LCDPanelList[LCD_MAX_PANELS] = {
	{L"DH WXCAT43",
     2,       /* Horizontal back porch */
	 2,       /* Horizontal front porch */
	 40,       /* HSYNC pulse width */
	 480,      /* Pixels per line */
	 2,      /* Vertical back porch */
	 2,      /* Vertical front porch */
	 10,        /* VSYNC pulse width */
	 272,      /* Lines per panel */
     0,        /* Do not invert output enable */
     0,        /* Invert panel clock */
     1,        /* Invert HSYNC */
     1,        /* Do not invert VSYNC */
     1,        /* AC bias frequency (not used) */
     16,       /* Bits per pixel */
     25000000,  /* Optimal clock rate (Hz) */
     TFT,  	   /* LCD panel type */
	 0,        /* Single panel display */
	{0x0000F800, 0x000007E0, 0x0000001F}}
};
//------------------------------------------------------------------------------
