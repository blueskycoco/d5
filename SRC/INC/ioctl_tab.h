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
// ioctl_tab.h
//
// Configuration file for the OAL IOCTL component.
//

#include "oal_ioctl_tab.h"

// IOCTL CODE,                          Flags   Handler Function
//------------------------------------------------------------------------------
{ IOCTL_LPC32XX_GETARMCLK,                  0,  OALGetClock                 },
{ IOCTL_LPC32XX_GETHCLK,                    0,  OALGetClock                 },
{ IOCTL_LPC32XX_GETPCLK,                    0,  OALGetClock                 },
{ IOCTL_LPC32XX_ENSYSCLK,                   0,  OALIoCtlEnableSysClk        },
{ IOCTL_LPC32XX_DMACHGET,                   0,  OALDMAGetCh                 },
{ IOCTL_LPC32XX_DMACHFREE,                  0,  OALDMAFreeCh                },

// Required Termination
{ 0,                                        0,  NULL                        }
