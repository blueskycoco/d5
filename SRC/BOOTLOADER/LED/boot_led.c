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
//*********************************************************************
//
// bsp_boot_led.c
//
// LED setup and control functions used with the bootloader
//

#include <windows.h>
#include <oal.h>
#include "boot_led.h"
#include "lpc32xx_gpio.h"

//------------------------------------------------------------------------------
//
// SetupLEDSGPIO
//
// Sets up LED(s) drive hardware
//
void SetupLEDSGPIO()
{
	;
}

//------------------------------------------------------------------------------
//
// SetLEDS
//
// Sets the passed LED states to ON.
//
void SetLEDS(UCHAR LedValues)
{
	GPIO_REGS_T *pGPIORegs;

	(void) LedValues;
	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);
	pGPIORegs->pio_outp_set = OUTP_STATE_GPO(1);
}

//------------------------------------------------------------------------------
//
// ClearLEDS
//
// Sets the passed LED states to OFF.
//
void ClearLEDS(UCHAR LedValues)
{
	GPIO_REGS_T *pGPIORegs;

	(void) LedValues;
	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);
	pGPIORegs->pio_outp_clr = OUTP_STATE_GPO(1);
}

//------------------------------------------------------------------------------
//
// SetLEDValue
//
// Sets the passed LED states to PN or OFF
//
void SetLEDValue(UCHAR LedValue)
{
	if ((LedValue & 0x1) == 0)
	{
		ClearLEDS(LedValue);
	}
	else
	{
		SetLEDS(LedValue);
	}
}
