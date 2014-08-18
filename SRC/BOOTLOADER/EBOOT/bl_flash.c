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
// bl_flash.c
//
// FLASH save and load functions
//

#include <windows.h>
#include <blcommon.h>
#include <eboot.h>
#include <boot_flash.h>

static INT_32 block, page;
static UNS_8 secbuff [528];
static UINT32 buffidx;
static BOOL scached;
extern NAND_GEOM_T savedgeom;

// Defines the first block where the WinCE image to boot is stored
#define WINCEIMAGEFIRSTBLOCK 100

//------------------------------------------------------------------------------
//
// OEMStartEraseFlash
//
// Start a FLASH erase cycle
//
BOOL OEMStartEraseFlash (DWORD dwStartAddr, DWORD dwLength) 
{
	return TRUE;
}

//------------------------------------------------------------------------------
//
// OEMContinueEraseFlash
//
// Continue a FLASH erase cycle
//
VOID OEMContinueEraseFlash()
{
	;
}

//------------------------------------------------------------------------------
//
// OEMFinishEraseFlash
//
// Finish a FLASH erase cycle
//
BOOL OEMFinishEraseFlash()
{
    return TRUE;
}

//------------------------------------------------------------------------------
//
// OEMWriteFlash
//
// Write data to FLASH, not used, use S1L instead
//
BOOL OEMWriteFlash(DWORD dwStartAddr, DWORD dwLength)
{
	return TRUE;
}

//------------------------------------------------------------------------------
//
// BLFLASHDownload
//
// This function initializes the NAND FLASH for operation
//
UINT32 BLFLASHDownload(BOOT_CFG *pConfig, OAL_KITL_DEVICE *pBootDevices)
{
	block = WINCEIMAGEFIRSTBLOCK;
	page = 0;
	scached = FALSE;

	// Nothing to do
	return BL_DOWNLOAD;
}

//------------------------------------------------------------------------------
//
// BLFLASHReadData
//
// Read data from FLASH device
//
BOOL BLFLASHReadData(DWORD cbData, LPBYTE pbData)
{
	DWORD tocopy;

	while (cbData > 0)
	{
		// Cache a new sector?
		if (scached == FALSE)
		{
			// Read sector
			nand_read_page(block, page, secbuff, &secbuff[512]);

			// Is block bad?
			if (((page == 0) && (secbuff[NAND_BADBLOCK_OFFS] != NAND_GOOD_BLOCK_MARKER))||
				((page == 1) && (secbuff[NAND_BADBLOCK_OFFS] != NAND_GOOD_BLOCK_MARKER)))
			{
				// Block is bad, skip it
				OALMSG(OAL_INFO, (
					L"Skipping bad block %d:\r\n", block));
				block++;
			}
			else
			{
				// Sector is cached
				scached = TRUE;
				buffidx = 0;
				page++;
				if (page >= savedgeom.pages_per_block)
				{
					block++;
					page = 0;
				} 
			}
		}

		if (scached == TRUE)
		{
			// Copy data
			tocopy = cbData;
			if (tocopy > (512 - buffidx))
			{
				tocopy = 512 - buffidx;
				scached = FALSE;
			}

			// Copy data
			memcpy(pbData, &secbuff[buffidx], tocopy);
			buffidx += tocopy;
			cbData -= tocopy;
			pbData += tocopy;
		}
	}

	return TRUE;
}
