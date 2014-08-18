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
// boot_args.h
//
// Structure of persistent boot arguments used with EBOOT
//

#pragma once

//------------------------------------------------------------------------------

#include "oal_args.h"
#include "oal_kitl.h"

//------------------------------------------------------------------------------

#define BOOT_CFG_SIGNATURE      'BCFG'
#define BOOT_CFG_VERSION        1

typedef struct {
    UINT32 signature;                   // Structure signature
    UINT32 version;                     // Structure version

    DEVICE_LOCATION bootDevLoc;         // Boot device
    DEVICE_LOCATION kitlDevLoc;
    
    UINT32 kitlFlags;                   // Debug/KITL mode
    UINT32 ipAddress;
    UINT32 ipMask;
    UINT32 ipRoute;
	UINT32 boot_to;
    CHAR   binName[MAX_PATH];           // for SD/MMC boot
	UINT32 baudRate;
	UINT32 verikey;
} BOOT_CFG;
