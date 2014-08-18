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
// bl_sdmmc.c
//
//  This file implements bootloader functions related to SD/MMC download
//

#include <eboot.h>
#include <sdfat.h>

//------------------------------------------------------------------------------
//
// g_ethState
//
// This structure contains local state variables.
//
FILEINFO g_fnFileInfo;

//------------------------------------------------------------------------------
//
// BLSDMMCDownload
//
// This function initialize SD/MMC controller and file system
//
UINT32 BLSDMMCDownload(BOOT_CFG *pConfig, OAL_KITL_DEVICE *pBootDevices)
{
	if(!FATInitDisk()){
        return BL_ERROR;
    }
    if(!FATOpenFile(&g_fnFileInfo, pConfig->binName)){
        OALMSG(OAL_ERROR, (L"ERROR: cann't open image file'%S'\r\n", pConfig->binName));
        return BL_ERROR;
    }

	return BL_DOWNLOAD;
}

//------------------------------------------------------------------------------
//
// BLSDMMCReadData
//
// Read data from SD/MMC card through FAT file system
//
BOOL BLSDMMCReadData(DWORD cbData, LPBYTE pbData)
{
    if(FATReadFile(&g_fnFileInfo, pbData, cbData) != cbData){
        OALMSG(OAL_ERROR, (L"ERROR: read image data failure\r\n"));
        return FALSE;
    }
    else{
        return TRUE;
    }
}
