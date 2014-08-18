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
// bsp_kitlname.h
//
// KITL name generation support
//

#include <bsp.h>

//------------------------------------------------------------------------------
//
// OALKitlCreateName
//
// Create KITL name deom ethernet MAC address
//
VOID OALKitlCreateName(CHAR *pPrefix, UINT16 mac[3], CHAR *pBuffer)
{
    UINT32 code, count, d;
    INT32  j;

    OALMSG(OAL_KITL&&OAL_FUNC, (
        L"+OALKitlCreateName('%hs', 0x%04x)\r\n", pPrefix, mac));

    // Copy prefix    
    count = 0;
    while ((count < (OAL_KITL_ID_SIZE - 1)) && (pPrefix[count] != '\0'))
	{
        pBuffer[count] = pPrefix[count];
        count++;
    }

    // Create unique part of name from MAC
    code  = (((UINT32) mac [1]) << 16) | ((UINT32) mac [0]);

    // Convert it to hexa number
	j = 0;
	while (count < (OAL_KITL_ID_SIZE - 1))
	{
		d = (code >> j) & 0xF;
		if (d <= 9)
		{
			pBuffer[count] = '0' + d;
		}
		else
		{
			pBuffer[count] = 'a' + (d - 10);
		}
		j = j + 4;
		count++;
	}
    pBuffer[count] = '\0';

    OALMSG(OAL_KITL&&OAL_FUNC, (
        L"-OALKitlCreateName(pBuffer = '%hs') %d\r\n", pBuffer));
}

//------------------------------------------------------------------------------
