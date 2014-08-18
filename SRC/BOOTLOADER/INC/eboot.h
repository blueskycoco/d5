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
// eboot.h
//
// EBOOT support functions and structures
//

#pragma once

#include <windows.h>
#include <ceddk.h>
#include <blcommon.h>
#include <nkintr.h>
#include <halether.h>
#include <oal.h>
#include <oal_blmenu.h>
#include <bsp_cfg.h>
#include <boot_args.h>
#include <args.h>
#include <image_cfg.h>

//------------------------------------------------------------------------------

#define EBOOT_VERSION_MAJOR         1
#define EBOOT_VERSION_MINOR         0

//------------------------------------------------------------------------------

typedef struct {
    UINT32 start;
    UINT32 length;
    UINT32 base;
} REGION_INFO_EX;

//------------------------------------------------------------------------------

#define DOWNLOAD_TYPE_RAM           1
#define DOWNLOAD_TYPE_FLASH         2
#define DOWNLOAD_TYPE_EBOOT         3

//------------------------------------------------------------------------------

typedef struct {
    OAL_KITL_TYPE bootDeviceType;
    UINT32 type;
    UINT32 numRegions;
    REGION_INFO_EX region[BL_MAX_BIN_REGIONS];
} EBOOT_CONTEXT;

//------------------------------------------------------------------------------

extern BOOT_CFG g_bootCfg;
extern EBOOT_CONTEXT g_eboot;
extern OAL_KITL_DEVICE g_bootDevices[];
extern OAL_KITL_DEVICE g_kitlDevices[];

//------------------------------------------------------------------------------

BOOL OEMReportError(DWORD dwReason, DWORD dwReserved);
VOID OEMMultiBinNotify(MultiBINInfo *pInfo);


//------------------------------------------------------------------------------

VOID   BLMenu();    
BOOL   BLReadBootCfg(BOOT_CFG *pBootCfg);
BOOL   BLWriteBootCfg(BOOT_CFG *pBootCfg);
UINT32 BLEthDownload(BOOT_CFG *pBootCfg, OAL_KITL_DEVICE *pBootDevices);
UINT32 BLSDMMCDownload(BOOT_CFG *pBootCfg, OAL_KITL_DEVICE *pBootDevices);
BOOL   BLSDMMCReadData(DWORD cbData, LPBYTE pbData);
VOID   BLEthConfig(BSP_ARGS *pArgs);
VOID*  BLVAtoPA(UINT32 address);
UINT8  lpc3250_sspread(INT32 index);
BOOL   FLASHWriteBinImage(UINT8 *buff, DWORD numbytes);
UINT32 BLFLASHDownload(BOOT_CFG *pBootCfg, OAL_KITL_DEVICE *pBootDevices);
BOOL   BLFLASHReadData(DWORD cbData, LPBYTE pbData);

//------------------------------------------------------------------------------

BOOL BLInitBootCfg(VOID);

//------------------------------------------------------------------------------

// Start of FLASH download area (block) for EBOOT
#define EBOOT_FIRST_BLOCK 25
#define EBOOT_MAX_BLOCKS 32 // Up to 512K of data area
