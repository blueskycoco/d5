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
// bsp_serial.c
//
// Debug serial port functions
//

#pragma once

#include <nkintr.h>

BOOL OEMDebugInit();
VOID OEMWriteDebugByte(UINT8 ch);
int OEMReadDebugByte();
void DebugOutputString(UCHAR *str);
VOID OEMDebugDeinit();
