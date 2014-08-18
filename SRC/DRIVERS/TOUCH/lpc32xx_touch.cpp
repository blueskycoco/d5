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
// FILENAME: touch_core.cpp
//
// DESCRIPTION:
//   Implementation of the WinCE touch screen PDD.
//
// Functions:
//
//  TouchDriverCalibrationPointGet
//  DdsiTouchPanelGetDeviceCaps
//  DdsiTouchPanelSetMode
//  DdsiTouchPanelEnable
//  DdsiTouchPanelDisable
//  DdsiTouchPanelAttach
//  DdsiTouchPanelDetach
//  DdsiTouchPanelGetPoint
//  DdsiTouchPanelPowerHandler
//
////////////////////////////////////////////////////////////////////////////////


//------------------------------------------------------------------------------
// Public
//
#include <windows.h>
#include <types.h>
#include <nkintr.h>
#include <creg.hxx>
#include <tchddsi.h>
#include <ceddk.h>

//------------------------------------------------------------------------------
// Platform
//
#include "lpc32xx_touch.h"
#include "bsp.h"

//------------------------------------------------------------------------------
// Debug Zones (only for retail)
// DGPARAM is already defined in tcc_mdd
// only use this to get debug messages in retail mode
//#define DEBUG

#ifndef SHIP_BUILD
#ifdef DEBUG
#define ZONE_INFO           DEBUGZONE(4)

#else
#undef  ZONE_ERROR
#undef  ZONE_WARN
#undef  ZONE_FUNCTION
#undef  ZONE_INIT
#undef  ZONE_INFO

#define ZONE_ERROR          DEBUGZONE(0)
#define ZONE_WARN           DEBUGZONE(1)
#define ZONE_FUNCTION       DEBUGZONE(2)
#define ZONE_INIT           DEBUGZONE(3)
#define ZONE_INFO           DEBUGZONE(4)

DBGPARAM dpCurSettings = 
    {
    L"Touch Screen", 
        {
        L"Errors",      
        L"Warnings", 
        L"Function",    
        L"Init",
        L"Info",        
        L"Undefined",   
        L"Undefined",   
        L"Undefined",
        L"Undefined",   
        L"Undefined",   
        L"Undefined",   
        L"Undefined",
        L"Undefined",       
        L"Undefined",   
        L"Undefined",   
        L"Undefined"      
        },
        0x0013  // Errors, Warnings, Info
    };

#undef  DEBUGMSG
#define DEBUGMSG RETAILMSG

#endif
#endif

//------------------------------------------------------------------------------
// local data structures
//
typedef struct
{
    LONG                  nProcessAttached;
    int                   nSampleRate;        // Sample rate
	LPC3250_ADCTSC_REGS_T *pTSCREGs;          // Pointer to touch registers
	BOOL predownflag;
} TOUCH_INSTANCE;

//------------------------------------------------------------------------------
//  Device registry parameters
static WCHAR const* s_szRegistryPath = L"\\HARDWARE\\DEVICEMAP\\TOUCH";

//------------------------------------------------------------------------------
// global variables
//
static TOUCH_INSTANCE s_TouchPad =
{
    0xFFFFFFFF,    
    DEFAULT_SAMPLE_RATE,
    NULL,
	FALSE
};

// Referenced in MDD. TS controller only asserts SYSINTR_TOUCH.
//
DWORD gIntrTouchChanged = SYSINTR_NOP;   // Not used here.
DWORD gIntrTouch        = SYSINTR_NOP;

static int srates[2] = {TOUCHPANEL_SAMPLE_RATE_LOW, TOUCHPANEL_SAMPLE_RATE_HIGH};

// We control the MDD thread wait timeout to force a penup state when no more
// touchscreen events are present
extern "C" DWORD gdwTouchIstTimeout;

// The MDD requires a minimum of MIN_CAL_COUNT consecutive samples before
// it will return a calibration coordinate to GWE.
extern "C" int MIN_CAL_COUNT = 10;

//------------------------------------------------------------------------------
//
//  TouchDriverCalibrationPointGet
//
//  Get calibration data for touch apoint
//
extern "C" BOOL TouchDriverCalibrationPointGet(TPDC_CALIBRATION_POINT *pTCP)
{
    BOOL rc = FALSE;

    INT32 cDisplayWidth  = pTCP->cDisplayWidth;
    INT32 cDisplayHeight = pTCP->cDisplayHeight;

    int CalibrationRadiusX = cDisplayWidth / 20;
    int CalibrationRadiusY = cDisplayHeight / 20;

    DEBUGMSG(ZONE_FUNCTION, (TEXT("TouchDriverCalibrationPointGet+\r\n")));    
    
    // Check which of the 5 calibration point is requested.
    switch ( pTCP->PointNumber )
        {
        case 0:
            pTCP->CalibrationX = cDisplayWidth / 2;
            pTCP->CalibrationY = cDisplayHeight / 2;
            rc = TRUE;
            break;

        case 1:
            pTCP->CalibrationX = CalibrationRadiusX * 2;
            pTCP->CalibrationY = CalibrationRadiusY * 2;
            rc = TRUE;
            break;

        case 2:
            pTCP->CalibrationX = CalibrationRadiusX * 2;
            pTCP->CalibrationY = cDisplayHeight - ( CalibrationRadiusY * 2 );
            rc = TRUE;
            break;

        case 3:
            pTCP->CalibrationX = cDisplayWidth - ( CalibrationRadiusX * 2 );
            pTCP->CalibrationY = cDisplayHeight - ( CalibrationRadiusY * 2 );
            rc = TRUE;
            break;

        case 4:
            pTCP->CalibrationX = cDisplayWidth - ( CalibrationRadiusX * 2 );
            pTCP->CalibrationY = CalibrationRadiusY * 2;
            rc = TRUE;
            break;

        default:
            pTCP->CalibrationX = cDisplayWidth / 2;
            pTCP->CalibrationY = cDisplayHeight / 2;
            SetLastError( ERROR_INVALID_PARAMETER );
            break;
    }

    DEBUGMSG(ZONE_INFO, (TEXT("cDisplayWidth        : %4d\r\n"), cDisplayWidth     ));
    DEBUGMSG(ZONE_INFO, (TEXT("cDisplayHeight       : %4d\r\n"), cDisplayHeight    ));
    DEBUGMSG(ZONE_INFO, (TEXT("CalibrationRadiusX   : %4d\r\n"), CalibrationRadiusX));
    DEBUGMSG(ZONE_INFO, (TEXT("CalibrationRadiusY   : %4d\r\n"), CalibrationRadiusY));
    DEBUGMSG(ZONE_INFO, (TEXT("pTCP -> PointNumber  : %4d\r\n"), pTCP->PointNumber));
    DEBUGMSG(ZONE_INFO, (TEXT("pTCP -> CalibrationX : %4d\r\n"), pTCP->CalibrationX));
    DEBUGMSG(ZONE_INFO, (TEXT("pTCP -> CalibrationY : %4d\r\n"), pTCP->CalibrationY));

    DEBUGMSG(ZONE_FUNCTION, (TEXT("TouchDriverCalibrationPointGet-\r\n")));    
    return ( rc );
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelGetDeviceCaps
//
//  Get touch panel capabilities
//
extern "C" BOOL DdsiTouchPanelGetDeviceCaps(INT iIndex, LPVOID lpOutput)
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelGetDeviceCaps+\r\n")));
 
    BOOL rc = FALSE;
    TPDC_SAMPLE_RATE *pTSR = (TPDC_SAMPLE_RATE*) lpOutput;
    TPDC_CALIBRATION_POINT_COUNT *pTCPC = (TPDC_CALIBRATION_POINT_COUNT*) lpOutput;

	if (pTSR == NULL)
    {
        DEBUGMSG(ZONE_ERROR, (TEXT("TouchPanelGetDeviceCaps: Invalid parameter.\r\n")));
        SetLastError( ERROR_INVALID_PARAMETER );
    }
    else
    {
        // Check which of the device capabilities are requested.
        switch (iIndex)
        {
            // Return the sample rate.
            case TPDC_SAMPLE_RATE_ID:

                pTSR->SamplesPerSecondLow      = TOUCHPANEL_SAMPLE_RATE_LOW;
                pTSR->SamplesPerSecondHigh     = TOUCHPANEL_SAMPLE_RATE_HIGH;
				pTSR->CurrentSampleRateSetting = s_TouchPad.nSampleRate;
                rc = TRUE;
                break;

            // Return the number of calibration points used to calibrate the touch screen.
            case TPDC_CALIBRATION_POINT_COUNT_ID:
                pTCPC->flags              = 0;
                pTCPC->cCalibrationPoints = 5;
                rc = TRUE;
                break;

            // Return the x and y coordinates of the requested calibration point.
            // The index of the calibration point is set in lpOutput->PointNumber.
            case TPDC_CALIBRATION_POINT_ID:
                rc = TouchDriverCalibrationPointGet((TPDC_CALIBRATION_POINT*)lpOutput);
                break;

            default:
                RETAILMSG(ZONE_ERROR, 
                    (TEXT("TouchPanelGetDeviceCaps: Invalid parameter.\r\n")));
                SetLastError(ERROR_INVALID_PARAMETER);
                break;
        }
    }

	DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelGetDeviceCaps-\r\n")));

    return rc;
}

//------------------------------------------------------------------------------
//  DdsiTouchPanelSetMode
//
//    Sets the low or high sample rate
//
BOOL DdsiTouchPanelSetMode(INT iIndex, LPVOID lpInput)
{
    BOOL rc = FALSE;

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelSetMode+\r\n")));
    
    switch (iIndex)
    {
        case TPSM_SAMPLERATE_LOW_ID:
			s_TouchPad.nSampleRate = 0;
			lpc32xx_update_samplerate();
            SetLastError(ERROR_SUCCESS);
            rc = TRUE;
			break;

		case TPSM_SAMPLERATE_HIGH_ID:
			s_TouchPad.nSampleRate = 1;
			lpc32xx_update_samplerate();
            SetLastError(ERROR_SUCCESS);
            rc = TRUE;
            break;

        default:
            RETAILMSG(ZONE_ERROR, (TEXT("DdsiTouchPanelSetMode: Invalid parameter.\r\n")));
            SetLastError(ERROR_INVALID_PARAMETER);
            break;
    }

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelSetMode-\r\n")));

    return rc;
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelEnable
//
//  Enable the touchscreen
//
BOOL DdsiTouchPanelEnable()
{
	DWORD irqt = OAL_INTR_IRQ_TS;

	DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelEnable+\r\n")));
    
    HKEY hkTouch = NULL;
    DWORD dwStatus, dwType, dwSize, samplerate = 0xFF, tmp32;
    BOOL rc = FALSE;

	// Open the registry key and read our configuration
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, s_szRegistryPath, 0, 0, &hkTouch);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("DdsiTouchPanelEnable: Error opening registry!\r\n")));
    }
	else
	{
		// Get sample rate (in Hz)
        dwSize = sizeof(samplerate);
	    dwType = REG_DWORD;
        dwStatus = RegQueryValueEx(hkTouch, _T("Samplerate"), NULL, &dwType, 
            (LPBYTE) &samplerate, &dwSize);
	    if (dwStatus != ERROR_SUCCESS) {
	        DEBUGMSG(ZONE_WARN, (_T("DdsiTouchPanelEnable: No sample rate type "
				_T("found, using default!\r\n"))));
			samplerate = DEFAULT_SAMPLE_RATE;
	    }

		// Get Maximum calibration error
        dwSize = sizeof(tmp32);
	    dwType = REG_DWORD;
        dwStatus = RegQueryValueEx(hkTouch, _T("MaxCalError"), NULL, &dwType, 
            (LPBYTE) &tmp32, &dwSize);
	    if (dwStatus != ERROR_SUCCESS) {
	        DEBUGMSG(ZONE_WARN, (_T("DdsiTouchPanelEnable: No maximum calibration "
				_T("value found, using default!\r\n"))));
			MIN_CAL_COUNT = DEFAULT_TOUCHPANEL_MAX_CALERROR;
	    }
		else
		{
			MIN_CAL_COUNT = (int) tmp32;
		}
	}
	
	// Close the registry key
    if (hkTouch != NULL) {
        RegCloseKey(hkTouch);
    }

	// Check high or low sample rate setting
	if (samplerate > 1)
	{
		DEBUGMSG(ZONE_WARN, (_T("DdsiTouchPanelEnable: Sample rate invalid, using default\r\n")));
		samplerate = DEFAULT_SAMPLE_RATE;
	}

	DEBUGMSG(ZONE_INFO, (_T("DdsiTouchPanelEnable: Using sample rate %d Hz\r\n"),
		srates[samplerate]));
	s_TouchPad.nSampleRate = samplerate;

    if (!lpc32xx_ts_init())
    {
        goto cleanup;
    }

	// Get pen sysintr value
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irqt, sizeof(irqt),
		&gIntrTouch, sizeof(gIntrTouch), NULL))
    {
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: TOUCH: Failed to request the touch sysintr.\r\n")));

        gIntrTouch = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	s_TouchPad.predownflag = FALSE;

	rc = TRUE;
    
cleanup:
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelEnable- (rc = %d)\r\n"), rc));
    return rc;
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelDisable
//
//  Disable the touchscreen
//
VOID DdsiTouchPanelDisable()
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelDisable+\r\n")));

    // Close pen event and kill thread.
    lpc32xx_ts_deinit();

    // Release interrupts
    if (gIntrTouch != SYSINTR_UNDEFINED)
    {
        KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &gIntrTouch, 
            sizeof(gIntrTouch), NULL, 0, NULL);
    }

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelDisable-\r\n")));
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelAttach
//
//
LONG DdsiTouchPanelAttach()
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelAttach+\r\n")));

    // Increment the number of process attach calls.
    s_TouchPad.nProcessAttached++;

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelAttach-\r\n")));
        
    // Return the number.
    return s_TouchPad.nProcessAttached;
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelDetach
//
//
LONG DdsiTouchPanelDetach()
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelDetach+\r\n")));
    
    // Decrement the number of process attach calls.
    s_TouchPad.nProcessAttached--;

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelDetach\r\n")));
    
    // Return the number.
    return s_TouchPad.nProcessAttached;
}

//------------------------------------------------------------------------------
//
//  DdsiTouchPanelGetPoint
//
//  The driver handles the touchscreen interrupt via the touch MDD. When the
//  touchscreen interrupt occurs, the touch MDD will call this function to
//  determine the touch state. 
//
void DdsiTouchPanelGetPoint(TOUCH_PANEL_SAMPLE_FLAGS *pTipStateFlags,
    INT *pUncalX, INT *pUncalY)
{
	TOUCHP_T TPoint;
    // MDD needs us to hold on to last valid sample and previous pen state.
    static INT usFilteredX       = 0;
    static INT usFilteredY       = 0;
    static DWORD dwTimeNextEvent = -1;
        
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelGetPoint+\r\n")));

    // Check if pen data are available If so, get the data.
    if (lpc32xx_get_point(&TPoint))
    {
		if (TPoint.pendown == FALSE)
		{
		    // If the previous pen down state was set and this is a pen up state,
			// then tell the MDD the pen was released and set the MDD thread time
			// back to INFINITE
			if (s_TouchPad.predownflag == TRUE)
			{
		        *pUncalX = usFilteredX;
			    *pUncalY = usFilteredY;
				s_TouchPad.predownflag = FALSE;
				*pTipStateFlags = TouchSampleValidFlag;
			}

			gdwTouchIstTimeout = INFINITE;
		}
		else
		{
			// Pen is currently down
			if (s_TouchPad.predownflag == FALSE)
			{
				// Pen has just been put down
				s_TouchPad.predownflag = TRUE;
			}

			// Save last samples
			*pUncalX = usFilteredX = TPoint.xpoint;
			*pUncalY = usFilteredY = TPoint.ypoint;
			*pTipStateFlags = (TouchSampleValidFlag | TouchSampleDownFlag);

			// Pen down timeout is 23mS, this is slightly longer than the longest
			// time between touchscreen interrupts
	        gdwTouchIstTimeout = 23;
 		}
	}
	else 
	{
		// By default, any sample returned will be ignored.
	    *pTipStateFlags = TouchSampleIgnore;
	}

    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelGetPoint-\r\n")));
}

//------------------------------------------------------------------------------
//  DdsiTouchPanelPowerHandler
//
//  Power down between samples (nothing to do here)
//
void DdsiTouchPanelPowerHandler(BOOL bOff)
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("DdsiTouchPanelPowerHandler\r\n")));
}

//------------------------------------------------------------------------------
//
//  lpc32xx_update_samplerate
//
//  Sets the sample rate of the touchscreen by adjusting the same update time
//  Sample rate is based on s_TouchPad.nSampleRate.
//
BOOL lpc32xx_update_samplerate(void)
{
	BOOL rc = FALSE;

	// Compute update delay (the time between samples when pendown is detected)
	// These are based off the 32KHz TC clock and are ok for hardcoded values
	if (s_TouchPad.nSampleRate == 0)
	{
		s_TouchPad.pTSCREGs->tsc_utr = TSC_UTR_UPDATE_TIME(100); // About 52Hz
	}
	else
	{
		s_TouchPad.pTSCREGs->tsc_utr = TSC_UTR_UPDATE_TIME(25); // About 101Hz
	}

	return rc;
}

//------------------------------------------------------------------------------
//
//  lpc32xx_ts_init
//
//  Initializes the touchscreen controller for auto operation
//
BOOL lpc32xx_ts_init(void)
{
	UINT32 tscval;
    PHYSICAL_ADDRESS pa;
	CLKENID_T clkinfo;
    DWORD bytesret;

	// Map register space
    pa.QuadPart = ADC_BASE;
	s_TouchPad.pTSCREGs = (LPC3250_ADCTSC_REGS_T *)
		MmMapIoSpace(pa, sizeof (LPC3250_ADCTSC_REGS_T), FALSE);
	if (s_TouchPad.pTSCREGs == NULL)
	{
		// Not mapped
		RETAILMSG(ZONE_ERROR, (TEXT("lpc32xx_ts_init: Register map failure\r\n")));
		return FALSE;
	}

	/* Enable clock to ADC block - 32KHz clock */
	clkinfo.clkid = CLKPWR_ADC_CLK;
    clkinfo.enable = TRUE;
    if (KernelIoControl(IOCTL_LPC32XX_ENSYSCLK, &clkinfo, sizeof (clkinfo),
		NULL, 0, &bytesret) == FALSE)
	{
		RETAILMSG(ZONE_ERROR, (TEXT("lpc32xx_ts_init: Error enabling touch clock\r\n")));
	}

	/* Ensure ADC is not powered */
    s_TouchPad.pTSCREGs->adc_con &= ~TSC_ADCCON_POWER_UP;

	/* Set the TSC FIFO depth to 4 samples of 10-bit size */
	s_TouchPad.pTSCREGs->adc_con = (TSC_ADCCON_IRQ_TO_FIFO_4 |
		TSC_ADCCON_X_SAMPLE_SIZE(10) | TSC_ADCCON_Y_SAMPLE_SIZE(10));

    /* set TSC SEL to default state */
    s_TouchPad.pTSCREGs->adc_sel = 0x284;

    // min/max x/y registers
	s_TouchPad.pTSCREGs->tsc_min_x = 0;  
	s_TouchPad.pTSCREGs->tsc_max_x = 0x3ff;  
	s_TouchPad.pTSCREGs->tsc_max_y = 0;  
	s_TouchPad.pTSCREGs->tsc_min_y = 0x3ff;  

    // Aux registers
	s_TouchPad.pTSCREGs->tsc_aux_utr = 0;
	s_TouchPad.pTSCREGs->tsc_aux_min = 0;   
	s_TouchPad.pTSCREGs->tsc_aux_max = 0;   

 	// Rise Time
	s_TouchPad.pTSCREGs->tsc_dtr = TSC_RTR_RISE_TIME(0x2);

	// Update Time
	s_TouchPad.pTSCREGs->tsc_utr = TSC_UTR_UPDATE_TIME(446);

	// Delay Time
	s_TouchPad.pTSCREGs->tsc_dtr = TSC_DTR_DELAY_TIME(0x2);

	// Touch time
	s_TouchPad.pTSCREGs->tsc_ttr = TSC_TTR_TOUCH_TIME(0x10);

	// Drain X Plate time
	s_TouchPad.pTSCREGs->tsc_dxp = TSC_DXP_DRAINX_TIME(0x4);

	// Update the sample rate 
	lpc32xx_update_samplerate();

	// Empty the touchscreen FIFO
	while ((s_TouchPad.pTSCREGs->tsc_stat & TSC_STAT_FIFO_EMPTY) == 0)
	{
		tscval = s_TouchPad.pTSCREGs->tsc_fifo;
	}

	// TSC Auto mode enable, this also sets AUTO bit
	s_TouchPad.pTSCREGs->adc_con |= TSC_ADCCON_AUTO_EN;		     

	return TRUE;
}

//------------------------------------------------------------------------------
//
//  lpc32xx_ts_deinit
//
//  Disables touchscreen
//
void lpc32xx_ts_deinit(void)
{
	CLKENID_T clkinfo;
    DWORD bytesret;

	if (s_TouchPad.pTSCREGs != NULL)
	{
		s_TouchPad.pTSCREGs->adc_con &= ~TSC_ADCCON_AUTO_EN;
		MmUnmapIoSpace(s_TouchPad.pTSCREGs, sizeof(LPC3250_ADCTSC_REGS_T));
	}

	// Disable touchscreen clock
	clkinfo.clkid = CLKPWR_ADC_CLK;
    clkinfo.enable = FALSE;
	KernelIoControl(IOCTL_LPC32XX_ENSYSCLK, &clkinfo, sizeof (clkinfo),
		NULL, 0, &bytesret);
}

//------------------------------------------------------------------------------
//
//  lpc32xx_fifo_empty
//
//  Empty touchscreen FIFO
//
static void lpc32xx_fifo_empty(void)
{
	UINT32 tsval;

	// Empty FIFO
	while ((s_TouchPad.pTSCREGs->tsc_stat & TSC_STAT_FIFO_EMPTY) == 0)
	{
		tsval = s_TouchPad.pTSCREGs->tsc_fifo;
	}
}

//------------------------------------------------------------------------------
//
//  lpc32xx_get_point
//
//  Returns the touchscreen state
//
BOOL lpc32xx_get_point(TOUCHP_T* pPoint)
{
	UINT32 tsamples, tsval, rval [25], xval [25], yval [25];
	BOOL pvalid = FALSE;

	if (s_TouchPad.pTSCREGs != NULL)
	{
		// Has touchscreen FIFO overflowed?
		if ((s_TouchPad.pTSCREGs->tsc_stat & TSC_STAT_FIFO_OVRRN) != 0)
		{
			// FIFO overrun, toss sample
			lpc32xx_fifo_empty();
		}
		else if ((s_TouchPad.pTSCREGs->tsc_stat & TSC_STAT_FIFO_EMPTY) == 0)
		{
			// Get sample data
			tsamples = 0;
			while ((tsamples < 25) &&
				((s_TouchPad.pTSCREGs->tsc_stat & TSC_STAT_FIFO_EMPTY) == 0))
			{
				tsval = s_TouchPad.pTSCREGs->tsc_fifo;
				xval [tsamples] = TSC_FIFO_NORMALIZE_X_VAL(tsval);
				yval [tsamples] = TSC_FIFO_NORMALIZE_Y_VAL(tsval);
				rval [tsamples] = tsval;
				tsamples++;
			}

			// Data is only valid if pen is still down
			pPoint->pendown = FALSE;
			if ((rval [3] & TSC_FIFO_TS_P_LEVEL) == 0)
				{
				// Average samples 1 and 2 only, toss sample 0 and 3
				if (tsamples >= 4)
				{
					pPoint->xpoint = 0x3FF - (INT) ((xval [1] + xval [2] + xval [0] + xval [3]) / 4);
					pPoint->ypoint = 0x3FF - (INT) ((yval [1] + yval [2]) / 2);
					pPoint->pendown = TRUE;
				}
			}
			else
			{
				lpc32xx_fifo_empty();
			}

			pvalid = TRUE;
		}
		else
		{
			// Pen is released
			pPoint->pendown = FALSE;
			pvalid = TRUE;
			lpc32xx_fifo_empty();
		}
	}

	InterruptDone(gIntrTouch);

	return pvalid;
}
