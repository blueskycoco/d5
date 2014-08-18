///********************************************************************
/// Software that is described herein is for illustrative purposes only  
/// which provides customers with programming information regarding the  
/// products. This software is supplied "AS IS" without any warranties.  
/// NXP Semiconductors assumes no responsibility or liability for the 
/// use of the software, conveys no license or title under any patent, 
/// copyright, or mask work right to the product. NXP Semiconductors 
/// reserves the right to make changes in the software without 
/// notification. NXP Semiconductors also make no representation or 
/// warranty that such application will be suitable for the specified 
/// use without further testing or modification. 
///********************************************************************
//
// boot_utilities.c
//
// Various utility functions used with EBOOT
//

#include <windows.h>
#include <oal.h>
#include "boot_utilities.h"
#include "bsp_serial.h"

//------------------------------------------------------------------------------
//
// OALWaitMS
//
// Wait the specified number of milliseconds
//
void OALWaitMS(ULONG MSToWait)
{
     ULONG   StartWaitTime = OALGetTickCount();

     while(OALGetTickCount() - StartWaitTime < MSToWait);
}

//------------------------------------------------------------------------------
//
// decStrToVal
//
// Convert a decimal string to a value
//
UINT32 decStrToVal(WCHAR *pStr)
{
    UINT32 part;
    LPCWSTR psz;

	psz = pStr;
	part = 0;
    while (*psz != L'\0') {
		if (*psz >= L'0' && *psz <= L'9') {
		    part = part * 10 + (*psz - L'0');
		}
	    psz++;
	}

	return part;
}
