//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this sample source code is subject to the terms of the Microsoft
// license agreement under which you licensed this sample source code. If
// you did not accept the terms of the license agreement, you are not
// authorized to use this sample source code. For the terms of the license,
// please see the license agreement between you and Microsoft or, if applicable,
// see the LICENSE.RTF on your install media or the root of your tools installation.
// THE SAMPLE SOURCE CODE IS PROVIDED "AS IS", WITH NO WARRANTIES.
//

#include "SDHC.h"
#include <SDCardDDK.h>
#include <nkintr.h>

// initialize debug zones
SD_DEBUG_INSTANTIATE_ZONES(
    TEXT("SDHC"), // module name
    SDHC_INIT_ZONE | SDHC_ERROR_ZONE | SDHC_RESPONSE_ZONE |
    SDHC_INTERRUPT_ZONE | SDHC_SEND_ZONE | SDHC_RESPONSE_ZONE |
    SDHC_RECEIVE_ZONE | SDHC_CLOCK_ZONE | SDHC_TRANSMIT_ZONE |
    SDHC_SDBUS_INTERACT_ZONE | SDHC_BUSY_STATE_ZONE,
    TEXT("Error"),
    TEXT("Init"),
    TEXT("Interrupts"),
    TEXT("Send Handler "), 
    TEXT("Responses"), 
    TEXT("Receive Data"),                   
    TEXT("Clock Control"), 
    TEXT("Transmit Data"), 
    TEXT("SDBus Interaction"), 
    TEXT("Card Busy State"),
    TEXT(""));

//------------------------------------------------------------------------------
//
// DllEntry
//
// DLL entry
//
STDAPI_(BOOL) DllEntry(HINSTANCE hInstance,
                       ULONG Reason,
                       LPVOID pReserved)
{
    if ( Reason == DLL_PROCESS_ATTACH ) {
        SD_DEBUG_ZONE_REGISTER(hInstance, NULL);
        DisableThreadLibraryCalls((HMODULE) hInstance);

        if( !SDInitializeCardLib() )
        {
            return FALSE;
        }
        else if( !SD_API_SUCCESS( SDHCDInitializeHCLib() ) )
        {
            SDDeinitializeCardLib();
            return FALSE;
        }
    }
    else if ( Reason == DLL_PROCESS_DETACH ) 
    {
        SDHCDDeinitializeHCLib();
        SDDeinitializeCardLib();
    }

	return TRUE;
}

//------------------------------------------------------------------------------
//
// SHC_Init
//
// Initialization entry point for the driver instance
//
extern "C" DWORD SHC_Init(DWORD dwContext)
{
	class sdCardController *pController;
    DWORD   dwRet = 0;

    DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC +Init\n")));
	RETAILMSG(1, (TEXT("SDHC +Init\n")));//jj

    pController = new sdCardController;
    if (!pController) {
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SHC_Init: class allocation error\n")));
        goto EXIT;
    }

    if (pController->sdInit((LPCTSTR) dwContext) == FALSE) {
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SHC_Init: Error initializing SD card\n")));
        goto EXIT;
    }

    // Return the controller instance
    dwRet = (DWORD) pController;

EXIT:
	if (dwRet == 0)
	{
		if (pController != NULL)
		{
	        delete pController;
		}
    }

	DEBUGMSG(SDCARD_ZONE_INIT, (TEXT("SDHC -Init\n")));

	return dwRet;
}

//------------------------------------------------------------------------------
//
// SHC_Deinit
//
// De-initialization entry point for the driver instance
//
extern "C" BOOL SHC_Deinit(DWORD hDeviceContext)
{    
    DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC +Deinit\n")));
	RETAILMSG(SDHC_INIT_ZONE, (TEXT("SDHC +Deinit\n")));//jj

    class sdCardController *pController = (class sdCardController*) hDeviceContext;    
    pController->sdCardFreeHostContext();
    delete pController;

	DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC -Deinit\n")));
    
    return TRUE;
}

//------------------------------------------------------------------------------
//
// SHC_Open
//
// Device instance open
//
extern "C" DWORD SHC_Open(DWORD hDeviceContext,
						  DWORD,
						  DWORD)
{
	RETAILMSG(1, (TEXT("SDHC open\n")));//jj
	return (DWORD) hDeviceContext;
}


//------------------------------------------------------------------------------
//
// SHC_Close
//
// Bus power down
//
extern "C" BOOL SHC_Close(DWORD hOpenContext)
{
	RETAILMSG(1, (TEXT("SDHC close\n")));//jj
    return TRUE;
}

//------------------------------------------------------------------------------
//
// SHC_PowerDown
//
// Bus power down
//
extern "C" void SHC_PowerDown(DWORD hDeviceContext)
{
    class sdCardController *pController = (class sdCardController*) hDeviceContext;
    ASSERT(pController != NULL);
	RETAILMSG(1, (TEXT("SDHC powerdown\n")));//jj
	pController->sdPowerUp(FALSE);
}

//------------------------------------------------------------------------------
//
//  SHC_PowerUp
//
//  Bus power up
//
extern "C" void SHC_PowerUp(DWORD hDeviceContext)
{
    class sdCardController *pController = (class sdCardController*) hDeviceContext;
    ASSERT(pController != NULL);
	RETAILMSG(1, (TEXT("SDHC powerup\n")));//jj
	pController->sdPowerUp(TRUE);
}

//------------------------------------------------------------------------------
//
// SHC_IOControl
//
// User defined IOCTL handler
//
extern "C" BOOL SHC_IOControl(DWORD hOpenContext,
							  DWORD dwCode,
							  PBYTE pBufIn,
							  DWORD dwLenIn,
							  PBYTE pBufOut,
							  DWORD dwLenOut,
							  PDWORD pdwActualOut)
{
    return FALSE;
}
