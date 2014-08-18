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
//------------------------------------------------------------------------------
//
//  File:  profiler.c
//
//  Profiler support functions (currently stubbed)
//

#ifdef IMGPROFILFER
#if IMGPROFILFER==1

#include <windows.h>
#include <nkintr.h>
#include <oal.h>
#include <intr.h>
#include "clkpwr_support.h"
#include "lpc32xx_timer.h"

UINT32 OALProfileIntrHandler(UINT32 ra);

// Function pointer to profiling timer ISR routine.
PFN_PROFILER_ISR g_pProfilerISR = OALProfileIntrHandler;

// Pointer to timer block
static TIMER_CNTR_REGS_T *pTMR = NULL;

//------------------------------------------------------------------------------
//
// OEMProfileTimerEnable
//
// This function is called by kernel to start kernel profiling timer
//
VOID OEMProfileTimerEnable(DWORD interval)
{
    BOOL enabled;
	UNS_32 rate;
	UINT32 clk, irq = OAL_INTR_IRQ_PROFILER;
    
    OALMSG(TRUE, (L"+OEMProfileTimerEnable(%d)\r\n", interval));
	RETAILMSG(1, (TEXT("OEM profile timer enable\r\n")));
	// Setup tier pointer address
	pTMR = OALPAtoVA(TIMER2_BASE, FALSE);

	// Enable clock for this block
	clkpwr_clk_en_dis(CLKPWR_TIMER2_CLK, TRUE);

	// Setup timer for profiling
	// Setup timer
	pTMR->tcr = 0;
	pTMR->ir = TIMER_CNTR_MTCH_BIT(0);
	pTMR->mr [0] = 1;
	pTMR->mcr = (TIMER_CNTR_MCR_MTCH(0) | TIMER_CNTR_MCR_RESET(0));
	pTMR->pc = 0;
	pTMR->tc = 0;

	// Get current timer clock rate
	clk = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);

	// Compute timer rate based on the interval(uS) rate
	rate = 1000000 / interval;
	rate = (clk / rate) - 1;
	pTMR->pr = rate / 2;

	// Enable profiling interrupt
	g_pProfilerISR = OALProfileIntrHandler;
	pTMR->tcr = TIMER_CNTR_TCR_EN;
	OALIntrEnableIrqs(1, &irq);
}

//------------------------------------------------------------------------------
//
// OEMProfileTimerDisable
//
// This function is called by kernel to stop kernel profiling timer
//
VOID OEMProfileTimerDisable() 
{
	UINT32 irq = OAL_INTR_IRQ_PROFILER;

	OALMSG(TRUE, (L"-OEMProfileTimerDisable\r\n"));
	RETAILMSG(1, (TEXT("OEM profile timer disable\r\n")));
	if (pTMR != NULL)
	{
		pTMR->tcr = 0;
	}
    
    // Disable the profile timer interrupt
    OALIntrDisableIrqs(1, &irq);
}

//------------------------------------------------------------------------------
//
// OALProfileIntrHandler
//
// This is timer interrupt handler which replace default handler in time when
// kernel profiling is active. It calls original interrupt handler in
// appropriate times
//
UINT32 OALProfileIntrHandler(UINT32 ra)
{
    ProfilerHit(ra);

	// Clear timer interrupt
	xx
	
	return SYSINTR_NOP;
}

#endif
#endif
