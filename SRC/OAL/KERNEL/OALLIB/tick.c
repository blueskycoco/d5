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
//  tick.c
//
// System timer support functions
//
#include <windows.h>
#include <nkintr.h>
#include <ceddk.h>
#include <oal.h>

#include "bsp.h"
#include "lpc32xx_timer.h"
#include "clkpwr_support.h"
#include "lpc32xx_gpio.h"

#define ENABLE_LED

// Pointer to timer block (used for system tick)
static TIMER_CNTR_REGS_T *pTMR;
static UINT32 actualMSecPerSysTick;

#ifdef ENABLE_LED
static GPIO_REGS_T *pGPIO;
static UINT32 nms;
static BOOL toggle = TRUE;
#endif

//------------------------------------------------------------------------------
//
// OALTimerInit
//
// This function is typically called from the OEMInit to initialize Windows CE
// system timer. The tickMSec parameter determine timer period in milliseconds.
// On most platform timer period will be 1 ms, but it can be usefull to use
// higher value for some specific devices.
//
BOOL OALTimerInit(UINT32 msecPerSysTick, UINT32 countsPerMSec, UINT32 countsMargin) {
    BOOL rc = FALSE;
    UINT32 matchrate, baseclk, sysIntr, irq;
	RETAILMSG(1, (TEXT("OEM timer init\r\n")));
    OALMSG(OAL_FUNC, (
        L"+OALTimerInit(%d, %d, %d)\r\n", msecPerSysTick, countsPerMSec,
        countsMargin));

	// Enable clock for this block
	clkpwr_clk_en_dis(CLKPWR_TIMER3_CLK, TRUE);

	// Get pointer to timer register block
	pTMR = (TIMER_CNTR_REGS_T *) OALPAtoVA((UINT32) TIMER_CNTR3, FALSE);

#ifdef ENABLE_LED
	pGPIO = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);
	pGPIO->pio_dir_set = OUTP_STATE_GPO(1);
	nms = 0;
#endif

	// Setup timer
	pTMR->tcr = 0;
	pTMR->ir = TIMER_CNTR_MTCH_BIT(0);
	pTMR->mr [0] = 1;
	pTMR->mcr = (TIMER_CNTR_MCR_MTCH(0) | TIMER_CNTR_MCR_RESET(0));
	pTMR->pc = 0;
	pTMR->tc = 0;

	// Get base clock for the timer
	baseclk = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);

	// Compute prescale divider to get the desired tick rate
	actualMSecPerSysTick = msecPerSysTick;
	matchrate = actualMSecPerSysTick * (baseclk / 1000);
	pTMR->pr = matchrate / 2;

	// Enable timer
	pTMR->tcr = TIMER_CNTR_TCR_EN;

    // Get SYSINTR value for tick interrupt
    irq = OAL_INTR_IRQ_TICK; 
    sysIntr = OALIntrRequestSysIntr(1, &irq, OAL_INTR_FORCE_STATIC);
	if (sysIntr == SYSINTR_UNDEFINED)
	{
        OALMSG(OAL_ERROR, (
            L"ERROR: OALTimerInit: Cannot map SYSINTR value\r\n"));
        goto cleanUp;
	}

    // Enable System Tick interrupt
    if (!OEMInterruptEnable(sysIntr, NULL, 0)) {
        OALMSG(OAL_ERROR, (
            L"ERROR: OALTimerInit: Interrupt enable for system timer failed\r\n"));
        goto cleanUp;
    }

    rc = TRUE;
    
cleanUp:
    OALMSG(OAL_FUNC, (L"-OALTimerInit..(rc = %d)\r\n", rc));
	return rc;
}

//------------------------------------------------------------------------------
//
// OALTimerIntrHandler
//
// This function implement timer interrupt handler. It is called from common
// ARM interrupt handler.
//
UINT32 OALTimerIntrHandler()
{
    UINT32 sysIntr = SYSINTR_NOP;

	// Clear timer interrupt
	pTMR->ir = TIMER_CNTR_MTCH_BIT(0);

	// Increment millisecond count
    CurMSec += actualMSecPerSysTick;

#ifdef ENABLE_LED
    // Toggle LED
	if (CurMSec > nms) {
		nms = CurMSec + 1000; // 1 sec on, 1 sec off

		if (toggle == TRUE)
		{
			toggle = FALSE;
			pGPIO->pio_outp_set = OUTP_STATE_GPO(1);
		}
		else
		{
			pGPIO->pio_outp_clr = OUTP_STATE_GPO(1);
			toggle = TRUE;
		}
	}
#endif

    // Reschedule?
    if ((int)(CurMSec - dwReschedTime) >= 0)
	{
		sysIntr = SYSINTR_RESCHED;
	}

	return sysIntr;
}

//------------------------------------------------------------------------------
//
// OEMGetTickCount
//
// This returns the number of milliseconds that have elapsed since Windows 
// CE was started. If the system timer period is 1ms the function simply 
// returns the value of CurMSec. If the system timer period is greater then
// 1 ms, the HiRes offset is added to the value of CurMSec.
//
UINT32 OEMGetTickCount()
{
    return CurMSec;
}
