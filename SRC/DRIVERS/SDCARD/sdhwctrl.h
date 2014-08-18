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
// sdhwctrl.h
//
// SD slot hardware control driver
//

#pragma once

#include <windows.h>
#include "lpc32xx_gpio.h"
#include "lpc32xx_intc.h"

// SD card slot hardware control and query class
class sdHwCtrl
{
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	sdHwCtrl(void) {};

	// Destructor
	~sdHwCtrl() {};

	//********************************************************************
	// Control and query functions
	//********************************************************************

	// Determines if the inserted SD card is write protected
	BOOL sdHwCardWriteProtected(GPIO_REGS_T *pGPIORegs);

	// Determines if an SD card is inserted
	BOOL sdHwCardInserted(GPIO_REGS_T *pGPIORegs);

	// Enables and disable SD card power
	void sdHwSlotPowerControl(GPIO_REGS_T *pGPIORegs,
		                      BOOL fPowerOn);

	// Modifies the SD card detect interrupt state control to high or low
	// edge interrupts, should be called when an SD card insertion state
	// changes, returns TRUE if the state changed
	BOOL sdHwUpdateDetectState(GPIO_REGS_T *pGPIORegs,
		                       INTC_REGS_T *pINTCRegs);

private:
};
