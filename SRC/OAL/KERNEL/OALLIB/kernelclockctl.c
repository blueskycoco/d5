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
// kernelclockctl.h
//
// The file provides the kernel ioctl functions that query system clock
// speed or enable and disable individual peripheral clocks.
//

#include <bsp.h>
#include "clkpwr_support.h"

//------------------------------------------------------------------------------
//
// OALGetClock
//
// IOCTL handler for (IOCTL_LPC32XX_GETARMCLK, IOCTL_LPC32XX_GETHCLK, and
// IOCTL_LPC32XX_GETPCLK)
// Return clock speed in Hz
BOOL OALGetClock(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
    BOOL rc = FALSE;

    OALMSG(OAL_FUNC, (L"+OALGetClock(...)\r\n"));
//    RETAILMSG(1, (TEXT("OAL Get clock\r\n")));
    // Set required/returned size if pointer isn't NULL
    if ((outSize >= sizeof (UINT32)) && (pOutBuffer != NULL))
	{
		rc = TRUE;
		switch (code) {
			case IOCTL_LPC32XX_GETARMCLK:
				* ((UINT32 *) pOutBuffer) = clkpwr_get_base_clock_rate(CLKPWR_ARM_CLK);
				break;

			case IOCTL_LPC32XX_GETHCLK:
				* ((UINT32 *) pOutBuffer) = clkpwr_get_base_clock_rate(CLKPWR_HCLK);
				break;

			case IOCTL_LPC32XX_GETPCLK:
				* ((UINT32 *) pOutBuffer) = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);
				break;

			default:
				rc = FALSE;
		}
	}
    
    OALMSG(OAL_FUNC, (L"-OALGetClock(rc = %d)\r\n", rc));
//	RETAILMSG(1, (L"-OALGetClock(rc = %d)\r\n", rc));
    return rc;
}

//------------------------------------------------------------------------------
//
// OALIoCtlEnableSysClk
//
// IOCTL handler for IOCTL_LPC32XX_ENSYSCLK
// Enables or disables a system clock
BOOL OALIoCtlEnableSysClk(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
	CLKENID_T *pClk = (CLKENID_T *) pInpBuffer;
    BOOL rc = FALSE;

    OALMSG(OAL_FUNC, (L"+OALIoCtlEnableSysClk(...)\r\n"));
    
    // Set required/returned size if pointer isn't NULL
    if ((inpSize >= sizeof (CLKENID_T)) && (pClk != NULL))
	{
		// Verify clock range
		if (pClk->clkid < CLKPWR_LAST_CLK)
		{
			clkpwr_clk_en_dis(pClk->clkid, (pClk->enable == TRUE));
			rc = TRUE;
		}
	}
    
    OALMSG(OAL_FUNC, (L"-OALIoCtlEnableSysClk(rc = %d)\r\n", rc));
    return rc;
}
