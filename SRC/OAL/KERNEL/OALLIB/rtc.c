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
// rtc.c
//
// The RTC is implemented with 2 peripherals. The RTC peripheral block stores
// a 1-second count of persistent system time (across power cycles). The RTC
// alarm function is also implemented with the RTC with a 1-second granularity.
// The high resolution system time functions are implemented with a standard
// timer which is setup from the RTC when WinCE is cold or warm booted. The
// standard timer is not connected to an interrupt, but is configured to be
// queried by clock functions. The IOCTL_HAL_INIT_RTC IOCTL isn't needed with
// this method, but the OEMSetupRTC() function needs to be called as part of
// OEMInit().
//
#include <windows.h>
#include <nkintr.h>
#include <ceddk.h>
#include <oal.h>

#include "bsp.h"
#include "lpc32xx_rtc.h"
#include "lpc32xx_timer.h"
#include "clkpwr_support.h"

// Pointer to timer block (used for RTC control)
static TIMER_CNTR_REGS_T *pTMR;
static RTC_REGS_T *pRTC;

// Time base used for high reolution conversion
static UINT32 tickspersec;

// Estimated base year used for clock/calaneder offset, trying to set
// a time less than this date will fail.
#define BASEDATE 2000

// FILETIME base offset used for local time conversion (in mS) for 1970
static ULARGE_INTEGER btime;

//------------------------------------------------------------------------------
//
// OEMSetupRTC
//
// This function is called from the OEMInit to initialize the WinCE RTC and
// system time functions. It must be called prior to any system time or RTC
// based functions being called.
//
BOOL OEMSetupRTC(VOID)
{
	SYSTEMTIME systime;
	FILETIME ftime;
	BOOL rc = FALSE;

    OALMSG(OAL_FUNC, (L"+OEMSetupRTC\r\n"));
	RETAILMSG(1, (TEXT("OEM setup RTC time\r\n")));
	// Enable clock for this block
	clkpwr_clk_en_dis(CLKPWR_TIMER1_CLK, TRUE);

	// Get pointer to RTC and timer register blocks
	pTMR = (TIMER_CNTR_REGS_T *) OALPAtoVA((UINT32) TIMER_CNTR1, FALSE);
	pRTC = (RTC_REGS_T *) OALPAtoVA((UINT32) RTC, FALSE);

	// Setup timer
	pTMR->tcr = 0;
	pTMR->ir = TIMER_CNTR_MTCH_BIT(0);
	pTMR->mr [0] = 0;
	pTMR->mcr = 0;
	pTMR->pc = 0;

	// Get base clock for the timer
	tickspersec = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);

	// A 1 second tick rate with a high reolution count
	pTMR->pr = tickspersec;

	// Examine the RTC ONSW load value. If it is incorrect, reset the RTC
	// to it's base time and disable any alarms
	if (pRTC->key != RTC_KEY_ONSW_LOADVAL)
	{
		pRTC->ctrl = RTC_CNTR_DIS;
		pRTC->ucount = 0;
		pRTC->dcount = 0xFFFFFFFF;
		pRTC->intstat = (RTC_MATCH0_INT_STS | RTC_MATCH1_INT_STS |
			RTC_ONSW_INT_STS);
		pRTC->match0 = 0xFFFFFFFF;
		pRTC->ctrl = RTC_MATCH0_EN;
		pRTC->key = RTC_KEY_ONSW_LOADVAL;
	}

	// Transfer current RTC count to timer count
	pTMR->tc = pRTC->ucount;

	// Enable timer
	pTMR->tcr = TIMER_CNTR_TCR_EN;

	// Get base time for clock
	systime.wYear = 2000;
	systime.wMonth = 1;
	systime.wDayOfWeek = 1;
	systime.wDay = 1;
	systime.wHour = 1;
	systime.wMinute = 0;
	systime.wSecond = 0;
	systime.wMilliseconds = 0;
	rc = NKSystemTimeToFileTime(&systime, &ftime);
	btime.LowPart = ftime.dwLowDateTime;
	btime.HighPart = ftime.dwHighDateTime;

	OALMSG(OAL_FUNC, (L"-OEMSetupRTC (rx = %d))\r\n", rc));
	return rc;
}

//------------------------------------------------------------------------------
//
// OEMGetRealTime
//
// Retrieve the system time
//
BOOL OEMGetRealTime(LPSYSTEMTIME lpst)
{
    BOOL rc = FALSE;
    ULARGE_INTEGER time;
    FILETIME fileTime;
	UINT32 psc1, psc2, tc;

    OALMSG(OAL_FUNC, (L"-OEMGetRealTime\r\n"));

	// If the prescaler overflowed during the timer count read, read it again
	psc1 = 1;
	psc2 = 0;
	while (psc2 < psc1)
	{
		// Handle timer overflow
		psc1 = pTMR->pc;
		tc = pTMR->tc;
		psc2 = pTMR->pc;
	}

	// The tc and psc1 contain the time offset from FTOFFS
	// Computet Millisecond offset
	time.LowPart = (DWORD) psc1;
	time.HighPart = 0;
	time.QuadPart = (time.QuadPart * 1000) / (UINT64) tickspersec;

	// Add in second offset
	time.QuadPart = time.QuadPart + (((UINT64) tc) * 1000);

	// Adjust for 100nS base
	time.QuadPart = time.QuadPart * 10000;

	// Adjust for local date offset
	time.QuadPart = time.QuadPart + btime.QuadPart;

	// Convert to system time
	fileTime.dwLowDateTime = time.LowPart;
	fileTime.dwHighDateTime = time.HighPart;
	rc = NKFileTimeToSystemTime(&fileTime, lpst);
    if(rc == TRUE)
    {
        OALMSG(OAL_FUNC, (
            L"-OEMGetRealTime(rc = %d %d/%d/%d %d:%d:%d.%03d)\r\n", rc, 
            lpst->wYear, lpst->wMonth, lpst->wDay, lpst->wHour, lpst->wMinute,
            lpst->wSecond, lpst->wMilliseconds));
    }

	return rc;
}

//------------------------------------------------------------------------------
//
// OEMSetRealTime
//
// Set the system time (Real time clock)
//
BOOL OEMSetRealTime(LPSYSTEMTIME lpst)
{
    BOOL rc = FALSE;
    ULARGE_INTEGER time, lsecs, lnsecs;
    FILETIME fileTime;
	UINT32 secs, nsecs;

    if(lpst != NULL)
	{
	    OALMSG(OAL_FUNC, (
		    L"+OEMSetRealTime(%d/%d/%d %d:%d:%d.%03d)\r\n", 
			lpst->wYear, lpst->wMonth, lpst->wDay, lpst->wHour, lpst->wMinute,
	        lpst->wSecond, lpst->wMilliseconds));
    
		if (lpst->wYear >= BASEDATE)
		{
			rc = NKSystemTimeToFileTime(lpst, &fileTime);
		}
	    if (rc != FALSE)
	    {
			// Get number of nS since Jan 1, 1601
	        time.LowPart = fileTime.dwLowDateTime;
		    time.HighPart = fileTime.dwHighDateTime;

			// Bias milliseconds to get in 32-bit range
			time.QuadPart = time.QuadPart - btime.QuadPart;

			// Break time in nS and Second parts
			lsecs.QuadPart = time.QuadPart / 10000000;
			lnsecs.QuadPart = time.QuadPart - (lsecs.QuadPart * 10000000);

			// Computed offsets
			secs = (UINT32) lsecs.QuadPart;
			time.QuadPart = (lnsecs.QuadPart * (UINT64) tickspersec) / 10000000;
			nsecs = (UINT32) time.QuadPart;

			// Get computed offset times for timer
			pTMR->tcr = 0;
			pTMR->tc = (UINT32) secs;
			pTMR->pc = nsecs;
			pTMR->tcr = TIMER_CNTR_TCR_EN;

			// Set RTC to new time
			pRTC->ctrl = RTC_CNTR_DIS;
			pRTC->ucount = (UINT32) secs;
			pRTC->dcount = (0xFFFFFFFF - (UINT32) secs);
			pRTC->ctrl = RTC_MATCH0_EN;
	    }
	}

    OALMSG(OAL_RTC&&OAL_FUNC, (L"-OEMSetRealTime(rc = %d)\r\n", rc));
    return rc;
}

//------------------------------------------------------------------------------
//
// OEMSetAlarmTime
//
// Set the alarm time
//
BOOL OEMSetAlarmTime(LPSYSTEMTIME lpst)
{
    BOOL rc = FALSE;
    ULARGE_INTEGER time;
    FILETIME fileTime;

	// Convert to seconds
    if(lpst != NULL)
	{
	    OALMSG(OAL_FUNC, (
		    L"+OEMSetAlarmTime(%d/%d/%d %d:%d:%d.%03d)\r\n", 
			lpst->wYear, lpst->wMonth, lpst->wDay, lpst->wHour, lpst->wMinute,
	        lpst->wSecond, lpst->wMilliseconds));
    
		rc = NKSystemTimeToFileTime(lpst, &fileTime);
	    if (rc != FALSE)
	    {
			// Get number of nS since Jan 1, 1601
	        time.LowPart = fileTime.dwLowDateTime;
		    time.HighPart = fileTime.dwHighDateTime;

			// Bias milliseconds to get in 32-bit range
			time.QuadPart = time.QuadPart - btime.QuadPart;

			// Get seconds
			time.QuadPart = time.QuadPart / 10000000;

			// Set RTC alarm time
			pRTC->match0 = (UINT32) time.QuadPart;
		}
	}

    OALMSG(OAL_FUNC, (L"-OEMSetAlarmTime(rc = %d)\r\n", rc));
	return rc;
}

//------------------------------------------------------------------------------
//
// OALIoCtlHalInitRTC
//
// This function is called by WinCE OS to initialize the time after boot.
// Input buffer contains SYSTEMTIME structure with default time value.
// If hardware has persistent real time clock it will ignore this value.
//
BOOL OALIoCtlHalInitRTC(
    UINT32 code, VOID *pInpBuffer, UINT32 inpSize, VOID *pOutBuffer, 
    UINT32 outSize, UINT32 *pOutSize
) {
    BOOL rc = FALSE;
    SYSTEMTIME *pTime = (SYSTEMTIME *) pInpBuffer;
	pRTC = (RTC_REGS_T *) OALPAtoVA((UINT32) RTC, FALSE);

    OALMSG(OAL_IOCTL&&OAL_FUNC, (L"+OALIoCtlHalInitRTC(...)\r\n"));

    if (pOutSize) {
        *pOutSize = 0;
    }

	// Validate inputs
    if (pInpBuffer == NULL || inpSize < sizeof(SYSTEMTIME)) {
        NKSetLastError(ERROR_INVALID_PARAMETER);
        OALMSG(OAL_ERROR, (
            L"ERROR: OALIoCtlHalInitRTC: Invalid parameter\r\n"
        ));
        goto cleanUp;
    }

	if (pRTC->key != RTC_KEY_ONSW_LOADVAL)
	{
	    rc = OEMSetupRTC();
	    rc &= OEMSetRealTime(pTime);
	}
    
cleanUp:
    OALMSG(OAL_IOCTL&&OAL_FUNC, (L"-OALIoCtlHalInitRTC(rc = %d)\r\n", rc));
    return rc;
}
