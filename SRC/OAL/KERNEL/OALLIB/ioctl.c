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
// ioctl.h
//
// This file implements the OEM's IO Control (IOCTL) functions and
// declares global variables used by the IOCTL component.
//

#include <bsp.h>
#include "drv_ioctl_funcs.h"

UINT32 g_oalIoCtlClockSpeed = 208;
UINT32 g_oalIoCtlInstructionSet = 0;
LPCWSTR g_oalIoCtlPlatformOEM  = L"ucDragon";
LPCWSTR g_oalIoCtlPlatformType = L"LPC3250";
LPCWSTR g_oalIoCtlProcessorCore   = L"ARM926EJ-S";
LPCWSTR g_oalIoCtlProcessorName   = L"LPC3250";
LPCWSTR g_oalIoCtlProcessorVendor = L"NXP";

//------------------------------------------------------------------------------
//
// g_oalIoCtlTable[]    
//
// IOCTL handler table. This table includes the IOCTL code/handler pairs  
// defined in the IOCTL configuration file. This global array is exported 
// via oal_ioctl.h and is used by the OAL IOCTL component.
//
const OAL_IOCTL_HANDLER g_oalIoCtlTable[] = {
#include "ioctl_tab.h"
};

//------------------------------------------------------------------------------
//
// OALIoCtlHalGetWakeSource    
//
// Unused IOCTL
//
BOOL OALIoCtlHalGetWakeSource(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
    return FALSE;
}

//------------------------------------------------------------------------------
//
// OALIoCtlHalEnableWake    
//
// Unused IOCTL
//
BOOL OALIoCtlHalEnableWake(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
    return FALSE;
}

//------------------------------------------------------------------------------
//
// OALIoCtlHalDisableWake    
//
// Unused IOCTL
//
BOOL OALIoCtlHalDisableWake(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
    return FALSE;
}

//------------------------------------------------------------------------------
//
// OALIoCtlHalReboot    
//
// Unused IOCTL
//
BOOL OALIoCtlHalReboot(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
    return FALSE;
}
