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
// sdhwctrl.c
//
// SD slot hardware control driver
//

#include "sdhwctrl.h"
#include "sdhc.h"

//------------------------------------------------------------------------------
//
// sdHwCardWriteProtected
//
// Determines if the inserted SD card is write protected
//
BOOL sdHwCtrl::sdHwCardWriteProtected(GPIO_REGS_T *pGPIORegs)
{
	BOOL wp = FALSE;

	if ((pGPIORegs->pio_inp_state & INP_STATE_GPIO_00) != 0)
	{
		wp = TRUE;
	}

	return wp;
}

//------------------------------------------------------------------------------
//
// sdHwCardInserted
//
// Determines if an SD card is inserted
//
BOOL sdHwCtrl::sdHwCardInserted(GPIO_REGS_T *pGPIORegs)
{
	BOOL Inserted = FALSE;

	if ((pGPIORegs->pio_inp_state & INP_STATE_GPIO_01) == 0)
	{
		Inserted = TRUE;
	}

	return Inserted;
}

//------------------------------------------------------------------------------
//
//  Function:  sdHwSlotPowerControl
//
//  Enables and disable SD card power
//
void sdHwCtrl::sdHwSlotPowerControl(GPIO_REGS_T *pGPIORegs,
								    BOOL fPowerOn)
{
	if (fPowerOn == 0) {
		pGPIORegs->pio_outp_clr = OUTP_STATE_GPO(11);
	}
	else
	{
		pGPIORegs->pio_outp_set = OUTP_STATE_GPO(11);
	}
}

//------------------------------------------------------------------------------
//
// sdHwUpdateDetectState
//
// Modifies the SD card detect interrupt state control to high or low
// edge interrupts, should be called when an SD card insertion state
// changes, returns TRUE if the state changed
//
BOOL sdHwCtrl::sdHwUpdateDetectState(GPIO_REGS_T *pGPIORegs,
		                             INTC_REGS_T *pINTCRegs)
{
	BOOL inserted, changed = FALSE;

	// Get current state
	inserted = sdHwCardInserted(pGPIORegs);

	if (inserted == FALSE)
	{
		// Swap GPIO IRQ state to active high detect
		pINTCRegs->apr &= ~GPIO_IRQ_MASK;

		changed = TRUE;
	}

	// If card was perviously inserted and it is now not inserted, then
	// signal an removal event
	if (inserted != FALSE)
	{
		// Swap GPIO IRQ state to active high detect
		pINTCRegs->apr |= GPIO_IRQ_MASK;

		changed = TRUE;
	}

	return changed;
}
