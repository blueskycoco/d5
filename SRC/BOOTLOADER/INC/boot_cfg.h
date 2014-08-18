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
// boot_cfg.h
//
// List of boot devices used with EBOOT
//

#pragma once

//------------------------------------------------------------------------------
OAL_KITL_DEVICE g_bootDevices[] = {
    { 
        L"LPC32xx RMII ethernet", Internal, ETHERNET_BASE, 
        0, OAL_KITL_TYPE_ETH, &g_kitllpc32xx
    },
    { 
        L"SD/MMC Card", Internal, SD_BASE, 
        0, OAL_KITL_TYPE_FLASH, NULL
    },
    { 
        L"NAND FLASH", Internal, MLC_BASE, 
        0, OAL_KITL_TYPE_FLASH, NULL
    },
    {
        NULL, 0, 0, 0, 0, NULL
    }
};    
