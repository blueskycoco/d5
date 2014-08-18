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
// ms2_backlight.c
//
// Backlight PDD driver source code
//

#include <windows.h>
#include <ceddk.h>
#include "lpc32xx_gpio.h"

#ifdef  RETAIL_DEBUG
#ifndef DEBUG
#define DEBUG 1
extern DBGPARAM dpCurSettings;
#undef DEBUGMSG
#define DEBUGMSG RETAILMSG
#endif
#endif

#ifdef DEBUG
#define ZONE_BACKLIGHT      DEBUGZONE(0)
#define ZONE_FUNCTION       DEBUGZONE(1)
#define ZONE_ERROR          DEBUGZONE(15)
#else
#define ZONE_BACKLIGHT      1
#define ZONE_FUNCTION       1
#define ZONE_ERROR          1
#endif

static 	GPIO_REGS_T *pGPIOREGs;

//-----------------------------------------------------------------------------
//
// BacklightInit
// 
// Sets up backlight hardware
//
extern "C" DWORD BacklightInit(LPCTSTR pContext, LPCVOID lpvBusContext,
							   CEDEVICE_POWER_STATE *pDeviceState)
{
	DWORD rc = FALSE;
	PHYSICAL_ADDRESS pa = {GPIO_BASE, 0};

    DEBUGMSG(ZONE_BACKLIGHT, (TEXT("BKL: Init\r\n")));

	// Enable backlight
	pGPIOREGs = (GPIO_REGS_T *) MmMapIoSpace(pa, sizeof (GPIO_REGS_T), FALSE);
	if (pGPIOREGs != NULL)
	{
		// Turn on backlight
		pGPIOREGs->pio_outp_clr = _BIT(4);
		rc = TRUE;
	}
	*pDeviceState = D4;

    return rc;
}

//-----------------------------------------------------------------------------
//
// BacklightDeInit
// 
// De-inits backlight hardware
//
extern "C" void BacklightDeInit(DWORD dwContext)
{
    DEBUGMSG(ZONE_BACKLIGHT, (TEXT("BKL: De-Init\r\n")));
    if (pGPIOREGs != NULL) {
       MmUnmapIoSpace((PVOID)pGPIOREGs, sizeof(GPIO_REGS_T));
    }
}

//-----------------------------------------------------------------------------
//
// BackLightSetState
// 
// Sets backlight state
//
extern "C" BOOL BackLightSetState(DWORD dwContext, CEDEVICE_POWER_STATE state)
{
    // sets the backlight state (turns the backlight on and off)
    DEBUGMSG(ZONE_FUNCTION, (L"+BackLightSetState(0x%08x)\r\n", (DWORD) state));

    switch (state)
    {
        case D0:
			pGPIOREGs->pio_outp_clr = _BIT(4);
            DEBUGMSG(ZONE_FUNCTION, (L"+BackLightSetState - ON\r\n"));
            break;
        case D1:
        case D2:
        case D3:
        case D4:
			pGPIOREGs->pio_outp_set = _BIT(4);
            DEBUGMSG(ZONE_FUNCTION, (L"+BackLightSetState - OFF\r\n"));
            break;
        default:
            RETAILMSG(ZONE_ERROR, (L"+BackLightSetState - Unsupported power state!\r\n"));
            return FALSE;
    }
    return TRUE;
}

//-----------------------------------------------------------------------------
//
// BacklightGetSupportedStates
// 
// Returns supported backlight power states
//
extern "C" UCHAR BacklightGetSupportedStates()
{
    return DX_MASK(D0) | DX_MASK(D4); //support D0, D4 (ON, OFF)
}

//-----------------------------------------------------------------------------
//
// BacklightIOControl
// 
// Backlight IO control block
//
extern "C" DWORD BacklightIOControl(DWORD dwOpenContext, DWORD dwIoControlCode,
									LPBYTE lpInBuf, DWORD nInBufSize, LPBYTE lpOutBuf,
									DWORD nOutBufSize, LPDWORD lpBytesReturned)
{
    // For IOCTls that MDD doesn't know. ie non-pm IOCTLs
    return ERROR_NOT_SUPPORTED;
}

//-----------------------------------------------------------------------------
//
// BacklightRegChanged
// 
// Backlight update from registry settings
//
extern "C" void BacklightRegChanged()
{
    // Called when the MDD gets a backlight registry changed event
    // eg: read brightness settings from registry and update backlight accordingly
    return;
}

//-----------------------------------------------------------------------------
//
// BacklightPwrSrcChanged
// 
// Backlight update from power source change
//
extern "C" void BacklightPwrSrcChanged(BOOL fOnAC)
{
    // Called when the MDD gets a power source changed (AC->DC or vice-versa) event
    // fOnAC will indicate whether the power source is now AC. TRUE->AC FALSE->Battery
    // eg: update brightness of backlight according to user settings
    return;
}
