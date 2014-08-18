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
// bsp_args.c
//
// Handles argument calls between EBOOT and the OAL.
//  

#include <bsp.h>

extern PHY_HW_T phyhwdesc;

//------------------------------------------------------------------------------
//
// OALArgsQuery
//
// This function is called from other OAL modules to return boot arguments.
// Boot arguments are typically placed in fixed memory location and they are
// filled by boot loader. In case that boot arguments can't be located
// the function should return NULL. The OAL module then must use
// default values.
//
VOID* OALArgsQuery(UINT32 type)
{
    VOID *pData = NULL;
    BSP_ARGS *pArgs;
    static UCHAR deviceId[16] = "";

    // Get pointer to expected boot args location
    pArgs = OALPAtoCA(IMAGE_SHARE_ARGS_PA);

    // Check if there is expected signature
    if (
        pArgs->header.signature != OAL_ARGS_SIGNATURE ||
        pArgs->header.oalVersion != OAL_ARGS_VERSION ||
        pArgs->header.bspVersion != BSP_ARGS_VERSION
    ) goto cleanUp;

    // Depending on required args    
    switch (type) {
    case OAL_ARGS_QUERY_UPDATEMODE:
        pData = &pArgs->updateMode;
        break;

	case OAL_ARGS_QUERY_KITL:
        pData = &pArgs->kitl;
        break;

    case OAL_ARGS_QUERY_DEVID:
		// Create KITL device name
		OALKitlCreateName(BSP_DEVICE_PREFIX, pArgs->kitl.mac, deviceId);
        pData = deviceId;
        break;
    }

cleanUp:
    OALMSG(OAL_KITL&&OAL_FUNC, (L"-OALArgsQuery(pData = 0x%x)\r\n", pData));
    return pData;
}

//------------------------------------------------------------------------------
