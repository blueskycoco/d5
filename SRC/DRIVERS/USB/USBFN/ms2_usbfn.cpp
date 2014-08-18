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
// ms2_usbfn.c
//
// USB function stub driver
//

#include <windows.h>
#include <ceddk.h>
#include <ddkreg.h>
#include <nkintr.h>
#include <usbfntypes.h>
#include <usbfn.h>
#include <pm.h>

extern DBGPARAM dpCurSettings = {
    L"UsbFn", {
        L"Error",       L"Warning",     L"Init",        L"Transfer",
        L"Pipe",        L"Send",        L"Receive",     L"USB Events",
        L"Interrupts",  L"Power",       L"",            L"",
        L"Function",    L"Comments",    L"",            L"PDD"
    },
    0x11
};

//------------------------------------------------------------------------------
//
// UfnPdd_PowerDown
//
// USB function powerdown
//
void WINAPI UfnPdd_PowerDown(PVOID pvPddContext)
{
	DEBUGMSG(1, (_T("UfnPdd_PowerDown\r\n")));
}

//------------------------------------------------------------------------------
//
// UfnPdd_PowerUp
//
// USB function powerup
//
void WINAPI UfnPdd_PowerUp(PVOID pvPddContext)
{
	DEBUGMSG(1, (_T("UfnPdd_PowerUp\r\n")));
}

//------------------------------------------------------------------------------
//
// UfnPdd_IOControl
//
// USB IOCTL handler
//
DWORD WINAPI UfnPdd_IOControl(
    PVOID           pvPddContext,
    IOCTL_SOURCE    source,
    DWORD           dwCode,
    PBYTE           pbInBuf,
    DWORD           cbInBuf,
    PBYTE           pbOutBuf,
    DWORD           cbOutBuf,
    PDWORD          pcbActualOutBuf
    )
{
	DEBUGMSG(1, (_T("UfnPdd_IOControl\r\n")));
    DWORD dwError = ERROR_INVALID_PARAMETER;
//    if (pvPddContext) {
//        dwError =((BulUsbDevice *)pvPddContext)->IOControl(source, dwCode, pbInBuf, cbInBuf, pbOutBuf,  cbOutBuf, pcbActualOutBuf);
//    };
    return dwError;
}

//------------------------------------------------------------------------------
//
// UfnPdd_Init
//
// USB function init
//
extern "C" DWORD WINAPI UfnPdd_Init(
    LPCTSTR                     pszActiveKey,
    PVOID                       pvMddContext,
    PUFN_MDD_INTERFACE_INFO     pMddInterfaceInfo,
    PUFN_PDD_INTERFACE_INFO     pPddInterfaceInfo
    )
{
	DEBUGMSG(1, (_T("UfnPdd_Init\r\n")));

	return ERROR_SUCCESS; // Doesn't really matter what I return yet
}

//------------------------------------------------------------------------------
//
// UfnPdd_Deinit
//
// USB function de-init
//
DWORD WINAPI UfnPdd_Deinit(PVOID pvPddContext)
{
	DEBUGMSG(1, (_T("UfnPdd_Deinit\r\n")));

	return ERROR_INVALID_PARAMETER;;
}

//------------------------------------------------------------------------------
//
// UfnPdd_DllEntry
//
// DLL entry point
//
extern "C"
BOOL UfnPdd_DllEntry(
    HANDLE hDllHandle,
    DWORD  dwReason, 
    LPVOID lpReserved
    )
{
	DEBUGMSG(1, (_T("UfnPdd_DllEntry\r\n")));
    return TRUE; // Nothing to do.
}
