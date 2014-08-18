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
// lpc32xx_touch.h
//
// Touchscreen driver
//

#pragma once

#include "lpc32xx_tsc.h"

//------------------------------------------------------------------------------
// Sample rate range and default - the sample rate is defined in the registry
// and must be withing the low and high range
#define TOUCHPANEL_SAMPLE_RATE_LOW  50   // Sample rate "0"
#define TOUCHPANEL_SAMPLE_RATE_HIGH 100  // Sample rate "1"
#define DEFAULT_SAMPLE_RATE 0 // Use low sample rate by default

//------------------------------------------------------------------------------
// Maximum calibration error
#define DEFAULT_TOUCHPANEL_MAX_CALERROR     10

//------------------------------------------------------------------------------
// Touchscreen point data structure
typedef struct
{
	INT  xpoint;     // X data point
	INT  ypoint;     // Y data point
	BOOL pendown;    // Pendown flag
} TOUCHP_T;

//------------------------------------------------------------------------------
// Hardware specific functions
BOOL lpc32xx_update_samplerate(void);
BOOL lpc32xx_ts_init(void);
void lpc32xx_ts_deinit(void);
BOOL lpc32xx_get_point(TOUCHP_T* pPoint);
