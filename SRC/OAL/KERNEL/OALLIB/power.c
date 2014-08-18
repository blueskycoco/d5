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
// power.c
//
// Power BSP callback functions implementation. This function are
// called as last function before OALCPUPowerOff. The KITL was already
// disabled by OALKitlPowerOff
//

#include <bsp.h>
#include "lpc32xx_clkpwr.h"

static UINT32 savedRegs[512];

//------------------------------------------------------------------------------
//
// I2CPeriphDown
//
// Disables clocking and power state
//
static void I2CPowerDown(UINT32 **pSavedRegsPtr)
{
	UINT32 *pSavedRegs = *pSavedRegsPtr;
	CLKPWR_REGS_T *pCLKPWRRegs = (CLKPWR_REGS_T *) OALPAtoVA(CLK_PM_BASE,
		FALSE);
	RETAILMSG(1, (TEXT("IIC Power down\r\n")));
	// Save current I2C state
	*pSavedRegs = pCLKPWRRegs->clkpwr_i2c_clk_ctrl;
	pSavedRegs++;

	// Disable I2C clocks
	pCLKPWRRegs->clkpwr_i2c_clk_ctrl &= ~(CLKPWR_I2CCLK_I2C1CLK_EN |
		CLKPWR_I2CCLK_I2C2CLK_EN);

	*pSavedRegsPtr = pSavedRegs;
}

//------------------------------------------------------------------------------
//
// I2CPowerUp
//
// Enables peripheral clocking and power state
//
static void I2CPowerUp(UINT32 **pSavedRegsPtr)
{
	UINT32 *pSavedRegs = *pSavedRegsPtr;
	CLKPWR_REGS_T *pCLKPWRRegs = (CLKPWR_REGS_T *) OALPAtoVA(CLK_PM_BASE,
		FALSE);
	RETAILMSG(1, (TEXT("IIC Power UP\r\n")));
	// Restore original I2C state
	pSavedRegs--;
	pCLKPWRRegs->clkpwr_i2c_clk_ctrl = *pSavedRegs;

	*pSavedRegsPtr = pSavedRegs;
}

//------------------------------------------------------------------------------
//
// sysSave
//
// Saves system states and powers down some peripherals
//
void sysSave(UINT32 **pSavedRegsPtr)
{
	// Power down I2C block
	I2CPowerDown(pSavedRegsPtr);
}

//------------------------------------------------------------------------------
//
// sysRestore
//
// Restores system states and restores power to some peripherals
//
void sysRestore(UINT32 **pSavedRegsPtr)
{
	// Power down I2C block
	I2CPowerUp(pSavedRegsPtr);
}

//------------------------------------------------------------------------------
//
// OEMPowerOff
//
// Safe system power down and restore
//
void OEMPowerOff (void)
{
	UINT32 *pSavedRegPtr = savedRegs;

	// Save important system information
	sysSave(&pSavedRegPtr);
	RETAILMSG(1, (TEXT("OEM power OFF\r\n")));
	OALMSGS(1, (L"*** POWER OFF ***\r\n"));

	// Save important system information
	sysRestore(&pSavedRegPtr);

	OALMSGS(1, (L"*** SYSTEM RESTORED ***\r\n"));
}    
