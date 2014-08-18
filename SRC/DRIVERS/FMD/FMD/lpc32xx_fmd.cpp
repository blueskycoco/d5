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
// lpc32xx_fmd.c
//
// NAND FLASH functions
//

#include <fmd.h>
#include <ceddk.h>
#include <ddkreg.h>
#include <Nkintr.h>
#include <Winbase.h>
#include "lbecc.h"
#include "lpc32xx_fmd.h"
#include "lpc32xx_clkpwr.h"
#include "lpc32xx_slcnand.h"
#include "lpc32xx_gpio.h"
#include "bsp.h"

// NAND info structure
typedef struct
{
	SLCNAND_REGS_T *pSLCRegs;      // Pointer to SLC registers
	CLKPWR_REGS_T *pCLKPWRRegs;    // Pointer to CLKPWR registers
	GPIO_REGS_T *pGPIORegs;        // Pointer to GPIO registers
	DWORD dwSysIntr;               // SLC NAND sysIntr value
	HANDLE dwEvent;                // SLC NAND event
#ifdef FMDACCESSLOCKS
	HANDLE Lockmutex;              // Access mutex
#endif
	// Queried device geometry
	DWORD dwNumBlocks;
	DWORD dwBytesPerBlock;
	WORD dwSectorsPerBlock;
	WORD dwDataBytesPerSector;
	DWORD addressCycles;           // NAND access address cycles
} NANDDRVDAT_T;
static NANDDRVDAT_T nandDrvDat;

//------------------------------------------------------------------------------
//
// nandMutexLock
//
// Get mutex lock
//
static void nandMutexLock(void)
{
#ifdef FMDACCESSLOCKS
    WaitForSingleObject(nandDrvDat.Lockmutex, INFINITE);
#endif
}

//------------------------------------------------------------------------------
//
// nandMutexUnlock
//
// Return mutex lock
//
static void nandMutexUnlock(void)
{
#ifdef FMDACCESSLOCKS
    ReleaseMutex(nandDrvDat.Lockmutex);
#endif
}

//------------------------------------------------------------------------------
//
// nandWpEnable
//
// Enable or disable NAND write protect
//
void nandWpEnable(BOOL enable) 
{
	if (enable == TRUE)
	{
		nandDrvDat.pGPIORegs->pio_outp_set = OUTP_STATE_GPO(19);
	}
	else
	{
		nandDrvDat.pGPIORegs->pio_outp_clr = OUTP_STATE_GPO(19);
	}
}

//------------------------------------------------------------------------------
//
// nandCSLock
//
// Lock or unlock NAND chip select signal
//
static void nandCSLock(BOOL lock)
{
	if (lock != FALSE)
	{
		nandDrvDat.pSLCRegs->slc_cfg |= SLCCFG_CE_LOW;
	}
	else
	{
		nandDrvDat.pSLCRegs->slc_cfg &= ~SLCCFG_CE_LOW;
	}
}

//------------------------------------------------------------------------------
//
// nandCheckReady
//
// Check NAND ready
//
BOOL nandCheckReady(void)
{
	BOOL rdy = FALSE;

	if ((nandDrvDat.pSLCRegs->slc_stat & SLCSTAT_NAND_READY) != 0)
	{
		rdy = TRUE;
	}

	return rdy;
}

//------------------------------------------------------------------------------
//
// nandWriteAddress
//
// Write FLASH address
//
void nandWriteAddress (unsigned char *addr,
					   int bytes)
{
    int idx = 0;
    
    for (idx = 0; idx < bytes; idx++)
	{
		nandDrvDat.pSLCRegs->slc_addr = (UNS_32) addr[idx];
    }
}

//------------------------------------------------------------------------------
//
// nandWriteCommand
//
// Write FLASH command
//
void nandWriteCommand (unsigned char cmd)
{
    nandDrvDat.pSLCRegs->slc_cmd = (UNS_32) cmd;
}

//------------------------------------------------------------------------------
//
// nandWriteData
//
// Write to FLASH data
//
void nandWriteData (void *data,
					int bytes)
{
    int idx;
    unsigned char *datab = (unsigned char *) data;

	for (idx = 0; idx < bytes; idx++)
	{
        nandDrvDat.pSLCRegs->slc_data = (UINT32) datab[idx];
    }
}

//------------------------------------------------------------------------------
//
// nandReadData
//
// Read from FLASH data
//
void nandReadData (void *addr,
				   int bytes)
{
    int idx;
    unsigned char *datab = (unsigned char *) addr;

	for (idx = 0; idx < bytes; idx++)
	{
        datab[idx] = (UNS_8) nandDrvDat.pSLCRegs->slc_data;
    }
}

//------------------------------------------------------------------------------
//
// nandWaitReady
//
// Wait for data ready from device
//
BOOL nandWaitReady(DWORD dataTimeout)
{
	if (dataTimeout > 0)
	{
		// Clear and enable NAND interrupts
		nandDrvDat.pSLCRegs->slc_icr = (SLCSTAT_INT_TC |
			SLCSTAT_INT_RDY_EN);
		InterruptDone(nandDrvDat.dwSysIntr);

		// Wait for interrupt
		WaitForSingleObject(nandDrvDat.dwEvent, dataTimeout);
	}

	// Return RDY status
	return nandCheckReady();
}

//------------------------------------------------------------------------------
//
// nandGetGeom
//
// Get device geometry
//
BOOL nandGetGeom (void)
{
	unsigned char temp[2];
	int to = 50;
	BOOL goodid = FALSE;

    // Send read ID command and wait for response
    nandCSLock(TRUE);
    nandWriteCommand(LPCNAND_CMD_READ_ID);
    temp[0] = 0;
    nandWriteAddress(temp, 1);
    while ((nandCheckReady() == FALSE) && (to > 0))
	{
		Sleep(1);
		to--;
	}

	// RDY failed?
	if (nandWaitReady(0) == FALSE)
	{
		RETAILMSG(1, (TEXT("FMD: Failed RDY response on NAND\r\n")));
		nandCSLock(FALSE);
		return FALSE;
	}

    // Read response
    nandReadData(temp, 2);

	// Verify manufacturer
	if( (temp[0] == LPCNAND_VENDOR_STMICRO)||(temp[0] == LPCNAND_VENDOR_SAMSUNG))
	{
		nandDrvDat.dwSectorsPerBlock = 32;
		nandDrvDat.dwDataBytesPerSector = 512;

		// Determine geometry based on device ID
		goodid = TRUE;
		switch (temp[1])
		{
		case 0x73:
			// 128MBit device
			nandDrvDat.dwNumBlocks = 1024;
			nandDrvDat.addressCycles = 3;
			break;

		case 0x35:
		case 0x75:
			// 256MBit device
			nandDrvDat.dwNumBlocks = 2048;
			nandDrvDat.addressCycles = 3;
			break;

		case 0x36:
		case 0x76:
			// 512MBit device
			nandDrvDat.dwNumBlocks = 4096;
			nandDrvDat.addressCycles = 4;
			break;

		case 0x39:
		case 0x79:
			// 1024MBit device
			nandDrvDat.dwNumBlocks = 8192;
			nandDrvDat.addressCycles = 4;
			break;

		default:
			goodid = FALSE;
			break;
		}

#if BYPASSBLOCKS>0
		nandDrvDat.dwNumBlocks = nandDrvDat.dwNumBlocks - BYPASSBLOCKS;
#endif

		nandDrvDat.dwBytesPerBlock = (nandDrvDat.dwSectorsPerBlock *
			nandDrvDat.dwDataBytesPerSector);
	}

    nandCSLock(FALSE);

	return goodid;
}

//------------------------------------------------------------------------------
//
// nandGetPageIndex
//
// Get an address from a sector number
//
void nandGetPageIndex(ULONG SectorAddr,
					  ULONG offset,
					  unsigned char *addrbytes) {
    ULONG block, page, nandaddr;

	// Limit offset
    if (offset >= 256)
	{
        offset = 0;
    }

    // Determine block and page offsets from passed sector address
    block = SectorAddr / nandDrvDat.dwSectorsPerBlock;
    page = SectorAddr - (block * nandDrvDat.dwSectorsPerBlock);

	// Block  Page  Index
	// 31..13 12..8 7..0
	nandaddr = offset + ((page & 0x1F) << 8);
	nandaddr = nandaddr | ((block & 0xFFF) << 13);

    // Save block and page address
	addrbytes[0] = (UNS_8) ((nandaddr >> 0) & 0xFF);
	addrbytes[1] = (UNS_8) ((nandaddr >> 8) & 0xFF);
	addrbytes[2] = (UNS_8) ((nandaddr >> 16) & 0xFF);
	if (nandDrvDat.addressCycles == 4) 
	{
		addrbytes[3] = (UNS_8) ((nandaddr >> 24) & 0xFF);
	}
}

//------------------------------------------------------------------------------
//
// FMD_Init
//
// Initialize FLASH interface and data
//
extern "C"
PVOID FMD_Init(LPCTSTR lpActiveReg,
			   PPCI_REG_INFO pRegIn,
               PPCI_REG_INFO pRegOut)
{
    PHYSICAL_ADDRESS pa;
	DWORD irq;
	UINT32 clk, bytesret;
	PVOID ret = NULL;
    BOOL validconfig = FALSE;
    
    /* Unused parameters */
    (void) pRegIn;
    (void) lpActiveReg;

	// Defaults
	nandDrvDat.pCLKPWRRegs = NULL;
	nandDrvDat.pSLCRegs = NULL;
	nandDrvDat.pGPIORegs = NULL;
	nandDrvDat.dwSysIntr = SYSINTR_UNDEFINED;
	nandDrvDat.dwEvent = NULL;
#ifdef FMDACCESSLOCKS
	nandDrvDat.Lockmutex = NULL;
#endif
	// MAP CLKPWR and SLC registers
	pa.QuadPart = CLK_PM_BASE;
	nandDrvDat.pCLKPWRRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);
	pa.QuadPart = SLC_BASE;
	nandDrvDat.pSLCRegs = (SLCNAND_REGS_T *) MmMapIoSpace(pa,
		sizeof (SLCNAND_REGS_T), FALSE);
	pa.QuadPart = GPIO_BASE;
	nandDrvDat.pGPIORegs = (GPIO_REGS_T *) MmMapIoSpace(pa,
		sizeof (GPIO_REGS_T), FALSE);
	if ((nandDrvDat.pCLKPWRRegs == NULL) || (nandDrvDat.pSLCRegs == NULL) ||
		(nandDrvDat.pGPIORegs == NULL))
	{
        RETAILMSG(1, 
			(TEXT("FMD: Failed to map registers\r\n")));
		goto cleanup;
	}

	// Setup SLC mode and enable SLC clock
	nandDrvDat.pCLKPWRRegs->clkpwr_nand_clk_ctrl = (CLKPWR_NANDCLK_SEL_SLC |
		CLKPWR_NANDCLK_SLCCLK_EN);

	// Reset SLC controller and setup for 8-bit mode, disable and clear interrupts
	nandDrvDat.pSLCRegs->slc_ctrl = SLCCTRL_SW_RESET;
	Sleep(1);
	nandDrvDat.pSLCRegs->slc_cfg = 0;
	nandDrvDat.pSLCRegs->slc_ien = SLCSTAT_INT_RDY_EN;
	nandDrvDat.pSLCRegs->slc_icr = (SLCSTAT_INT_TC |
		SLCSTAT_INT_RDY_EN);

	// Get current system clock speed for the SLC block
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
		sizeof (clk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock, use default
        RETAILMSG(1, 
            (TEXT("FMD: Error getting SLC base clock rate.\r\n")));
		clk = 104000000;
	}

	// Setup SLC timing based on current clock
    nandDrvDat.pSLCRegs->slc_tac = (
		SLCTAC_WDR(PHY_NAND_WDR) |
		SLCTAC_WWIDTH(clk / PHY_NAND_WWIDTH) |
		SLCTAC_WHOLD(clk / PHY_NAND_WHOLD) |
		SLCTAC_WSETUP(clk / PHY_NAND_WSETUP) |
		SLCTAC_RDR(PHY_NAND_RDR) |
		SLCTAC_RWIDTH(clk / PHY_NAND_RWIDTH) |
		SLCTAC_RHOLD(clk / PHY_NAND_RHOLD) |
		SLCTAC_RSETUP(clk / PHY_NAND_RSETUP));

	// Get device geometry
	if (nandGetGeom() == FALSE)
	{
		// Unsupported type
        RETAILMSG(1, (TEXT("FMD: Unsupported device type.\r\n")));
		goto cleanup;
	}

	// Map sysIntr value to the NAND interrupt
	irq = OAL_INTR_IRQ_NAND;
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irq,
		sizeof(irq), &nandDrvDat.dwSysIntr, sizeof(nandDrvDat.dwSysIntr), NULL))
    {
		RETAILMSG(1,
			(TEXT("FMD: Error obtaining SYSINTR value!\r\n")));
        nandDrvDat.dwSysIntr = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	// Create NAND interrupt event
	nandDrvDat.dwEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (nandDrvDat.dwEvent == NULL)
	{
		RETAILMSG(1, 
			(TEXT("FMD: Failed to create NAND handler event.\r\n")));
		nandDrvDat.dwEvent = NULL;
        goto cleanup;
	}

	// Bind interrupt to events
	if (InterruptInitialize(nandDrvDat.dwSysIntr, nandDrvDat.dwEvent, NULL,
		0) == FALSE)
	{
		// Cannot initialize interrupt
		RETAILMSG(1, 
			(TEXT("FMD: Cannot initialize NAND interrupt\r\n")));
        goto cleanup;
	}

#ifdef FMDACCESSLOCKS
	// Create lock
	nandDrvDat.Lockmutex = CreateMutex(NULL, FALSE, NULL);
    if (nandDrvDat.Lockmutex == NULL) {
		RETAILMSG(1, 
			(TEXT("FMD: Cannot create mutex lock\r\n")));
        goto cleanup;
    }
#endif

	// Initialize ECC
    eccInitTables();

	// Reset device
	nandWriteCommand(LPCNAND_CMD_RESET);
	nandWaitReady(10);

    ret = &nandDrvDat;
    
cleanup:
	if (ret == NULL)
	{
	    FMD_Deinit(&nandDrvDat);
	}

	return ret;
}

//------------------------------------------------------------------------------
//
// FMD_Deinit
//
// De-initialize NAND interface
//
extern "C"
BOOL FMD_Deinit(PVOID hFMD)
{
	// Close event handle
	if (nandDrvDat.dwEvent != NULL)
	{
		CloseHandle(nandDrvDat.dwEvent);
		nandDrvDat.dwEvent = NULL;
	}

	// Return sysIntr value
	if (nandDrvDat.dwSysIntr != SYSINTR_UNDEFINED)
	{
		InterruptDisable(nandDrvDat.dwSysIntr);
		KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &nandDrvDat.dwSysIntr,
            sizeof(nandDrvDat.dwSysIntr), NULL, 0, NULL);
		nandDrvDat.dwSysIntr = SYSINTR_UNDEFINED;
	}

	// Diable SLC clock and unmap registers
	if (nandDrvDat.pCLKPWRRegs != NULL)
	{
		nandDrvDat.pCLKPWRRegs->clkpwr_nand_clk_ctrl &= ~CLKPWR_NANDCLK_SLCCLK_EN;
        MmUnmapIoSpace((PVOID) nandDrvDat.pCLKPWRRegs, sizeof (CLKPWR_REGS_T));
		nandDrvDat.pCLKPWRRegs = NULL;
	}

	if (nandDrvDat.pSLCRegs != NULL)
	{
        MmUnmapIoSpace((PVOID) nandDrvDat.pSLCRegs, sizeof (SLCNAND_REGS_T));
		nandDrvDat.pSLCRegs = NULL;
	}

	// Enable write protect and unmap GPIO registers
	if (nandDrvDat.pGPIORegs != NULL)
	{
		nandWpEnable(TRUE);

        MmUnmapIoSpace((PVOID) nandDrvDat.pGPIORegs, sizeof (GPIO_REGS_T));
		nandDrvDat.pGPIORegs = NULL;
	}

#ifdef FMDACCESSLOCKS
	// Close lock mutex
	if (nandDrvDat.Lockmutex != NULL)
	{
        CloseHandle(nandDrvDat.Lockmutex);
        nandDrvDat.Lockmutex = NULL;
	}
#endif

    return TRUE;
}

//------------------------------------------------------------------------------
//
// FMD_EraseBlock
//
// Erase a FLASH block
//
extern "C"
BOOL FMD_EraseBlock(BLOCK_ID blockID)
{
    unsigned char addrbytes[3];
    BYTE Status;
    
#if BYPASSBLOCKS>0
	blockID += BYPASSBLOCKS;
#endif

	// Lock access and chip select
	nandMutexLock();
    nandCSLock(TRUE);
    
    // Issue erase command with block address and wait
    nandWriteCommand(LPCNAND_CMD_ERASE1);

	// Generate block address
	addrbytes[0] = (unsigned char) ((blockID << 5) & 0x00E0);
	addrbytes[1] = (unsigned char) ((blockID >> 3) & 0x00FF);
	if (nandDrvDat.addressCycles == 4)
	{
		addrbytes[2] = (unsigned char) ((blockID >> 11) & 0x0003);
	}

	nandWriteAddress(addrbytes, (nandDrvDat.addressCycles - 1));
    nandWriteCommand(LPCNAND_CMD_ERASE2);
    nandWaitReady(MAXERASETIMEINMS);
    
    // Get NAND operation status
    nandWriteCommand(LPCNAND_CMD_STATUS);
    nandReadData(&Status, 1);
    
    // Unlock access and chip select
    nandCSLock(FALSE);
	nandMutexUnlock();
    
    if ((Status & 0x1) == 0)
	{
        // Passed
        return TRUE;
    }

	// Failed
    RETAILMSG(1,
		(TEXT("FMD: Failed block erase (status = 0x%x)\r\n"), Status));

	return FALSE;
}

//------------------------------------------------------------------------------
//
// FMD_GetBlockStatus
//
// Return the status of the specified block
//
extern "C"
DWORD FMD_GetBlockStatus(BLOCK_ID blockID)
{
    SectorInfo SI;
    DWORD dwResult = 0;
	SECTOR_ADDR Sector;

	// Compute sector address
	Sector = blockID * nandDrvDat.dwSectorsPerBlock;

    /* Get SectorInfo data from the sector */ 
    if (FMD_ReadSector(Sector, NULL, &SI, 1) == FALSE) {
        RETAILMSG(1, (TEXT("FMD: Cannot read block\r\n")));
        return BLOCK_STATUS_UNKNOWN;
    }

    // Add in statuses
	if (SI.bBadBlock != 0xFF)
	{
		dwResult |= BLOCK_STATUS_BAD;
	}
    if ((SI.bOEMReserved & OEM_BLOCK_READONLY) == 0)
	{
        dwResult |= BLOCK_STATUS_READONLY;
    }
    if ((SI.bOEMReserved & OEM_BLOCK_RESERVED) == 0)
	{
        dwResult |= BLOCK_STATUS_RESERVED;
    }
    
    return dwResult;
}

//------------------------------------------------------------------------------
//
// FMD_GetInfo
//
// Return FLASH geometry
//
extern "C"
BOOL FMD_GetInfo(PFlashInfo pFlashInfo)
{
	if (pFlashInfo == NULL) {
        return FALSE;
    }

    pFlashInfo->flashType           = NAND;
    pFlashInfo->wDataBytesPerSector = nandDrvDat.dwDataBytesPerSector;
    pFlashInfo->dwNumBlocks         = nandDrvDat.dwNumBlocks;
    pFlashInfo->wSectorsPerBlock    = nandDrvDat.dwSectorsPerBlock;
    pFlashInfo->dwBytesPerBlock     = nandDrvDat.dwBytesPerBlock;

	return TRUE;
}

//------------------------------------------------------------------------------
//
// FMD_OEMIoControl
//
// FLASH IO control block
//
extern "C"
BOOL  FMD_OEMIoControl(DWORD dwIoControlCode,
					   PBYTE pInBuf,
					   DWORD nInBufSize, 
                       PBYTE pOutBuf,
					   DWORD nOutBufSize,
                       PDWORD pBytesReturned)
{
    return FALSE;
}

//------------------------------------------------------------------------------
//
// FMD_PowerUp
//
// Power up
//
extern "C"
VOID FMD_PowerUp(VOID)
{
}

//------------------------------------------------------------------------------
//
// FMD_PowerDown
//
// Power down
//
extern "C"
VOID FMD_PowerDown(VOID)
{
}

//------------------------------------------------------------------------------
//
// FMD_SetBlockStatus
//
// Set the block status
//
extern "C"
BOOL FMD_SetBlockStatus(BLOCK_ID blockID,
						DWORD dwStatus)
{
    SectorInfo SI;
	SECTOR_ADDR Sector;

	// Compute sector address
	Sector = blockID * nandDrvDat.dwSectorsPerBlock;

    /* Get SectorInfo data from the sector */ 
    if (FMD_ReadSector(Sector, NULL, &SI, 1) == FALSE)
	{
        RETAILMSG(1, (TEXT("FMD: Cannot read block\r\n")));
        return FALSE;
    }

	// Should block be marked as bad?
	if ((dwStatus & BLOCK_STATUS_BAD) != 0)
	{
		SI.bBadBlock = 0;
	}

	// Other statuses
    if ((dwStatus & BLOCK_STATUS_READONLY) != 0)
	{
        SI.bOEMReserved &= ~OEM_BLOCK_READONLY;
    }
        
    if ((dwStatus & BLOCK_STATUS_RESERVED) != 0)
	{
        SI.bOEMReserved &= ~OEM_BLOCK_RESERVED;
    }

	// Write back sector info
    if (FMD_WriteSector (Sector, NULL, &SI, 1) == FALSE)
	{
        RETAILMSG(1, (TEXT("FMD: Write sector failure\r\n")));
        return FALSE;
    }    

	return TRUE;
}

//------------------------------------------------------------------------------
//
// FMD_ReadSector
//
// Read a number of sectors
//
extern "C"
BOOL FMD_ReadSector(SECTOR_ADDR startSectorAddr,
					LPBYTE pSectorBuff,
                    PSectorInfo pSectorInfoBuff,
					DWORD dwNumSectors)
{
    unsigned char addrbytes[5];
	unsigned short eccin[2], ecccomp[2];
    ULONG SectorAddr;
	SectorInfo tSec;

#if BYPASSBLOCKS>0
	startSectorAddr += (BYPASSBLOCKS * nandDrvDat.dwSectorsPerBlock);
#endif
	SectorAddr = startSectorAddr;

	// Lock
	nandMutexLock();

    // Read all sectors
    while (dwNumSectors > 0)
    {
        // Wait until device is ready
        if (nandWaitReady(MAXEREADTIMEINMS) == FALSE)
		{
			// Device not ready
			RETAILMSG(1, (TEXT("FMD: Timeout reading sector\r\n")));
			return FALSE;
		}

		// Lock chip select
	    nandCSLock(TRUE);

		// Is this a sector read with or without sector info?
        if (pSectorBuff != NULL) {
            nandGetPageIndex(SectorAddr, 0, addrbytes);
			nandWriteCommand(LPCNAND_CMD_PAGE_READA);
            nandWriteAddress(addrbytes, nandDrvDat.addressCycles);
            nandWaitReady(MAXEREADTIMEINMS);

            // Copy buffer from NAND
            nandReadData(pSectorBuff, 512);
            eccGenerate512(ecccomp, pSectorBuff);
            
            if (pSectorInfoBuff != NULL)
			{
                // Seek to sector info structure and read it
                nandReadData(pSectorInfoBuff, sizeof(SectorInfo));
                pSectorInfoBuff++;
            }
			else
			{
				// Just read garbage to place NAND pointer in right location
				// for the ECC fetch
                nandReadData(&tSec, sizeof(tSec));
			}

            // Get the saved ECC data from the FLASH
            nandReadData(&eccin, sizeof(eccin));
            
            // Check and correct data
            if (eccCheckAndCorrect(eccin, ecccomp,
                pSectorBuff) == ECC_NOTCORRECTABLE)
			{
                // Uncorrectable error, report it to FAL
                nandCSLock(FALSE);
                RETAILMSG(1, (TEXT("FMD: ECC failure\r\n")));
				nandMutexUnlock();
                return FALSE;
            }
                
            pSectorBuff = pSectorBuff + 512;
        }
        else if (pSectorInfoBuff != NULL)
        {
            nandGetPageIndex(SectorAddr, 0, addrbytes);
            /* Read sectorinfo structure from FLASH */
            nandWriteCommand(LPCNAND_CMD_PAGE_READC);
            nandWriteAddress(addrbytes, nandDrvDat.addressCycles);
            nandWaitReady(MAXEREADTIMEINMS);
            nandReadData(pSectorInfoBuff, sizeof(SectorInfo));
            pSectorInfoBuff++;
		}

        ++SectorAddr;
        dwNumSectors--;

	    /* Unlock access and chip select */
		nandCSLock(FALSE);
		nandMutexUnlock();
	}

    return TRUE;
}

//------------------------------------------------------------------------------
//
// FMD_WriteSector
//
// Write a number of sectors
//
extern "C"
BOOL FMD_WriteSector(SECTOR_ADDR startSectorAddr,
					 LPBYTE pSectorBuff,
                     PSectorInfo pSectorInfoBuff,
					 DWORD dwNumSectors)
{
	unsigned char addrbytes[5];
    BYTE Status;
    unsigned short ecccomp[2];
    ULONG SectorAddr;

#if BYPASSBLOCKS>0
	startSectorAddr += (BYPASSBLOCKS * nandDrvDat.dwSectorsPerBlock);
#endif
	SectorAddr = startSectorAddr;

	if ((pSectorBuff == NULL) && (pSectorInfoBuff == NULL)) {
        return FALSE;
    }

	/* Lock access and chip select */
	nandMutexLock();
    nandCSLock(TRUE);
    
    while (dwNumSectors > 0) {
        if (pSectorBuff != NULL) {
            /* Setup write */
            nandGetPageIndex(SectorAddr, 0, addrbytes);
            nandWriteCommand(LPCNAND_CMD_PAGE_READA);
            nandWriteCommand(LPCNAND_CMD_PAGE_WRITE1);
            nandWriteAddress(addrbytes, nandDrvDat.addressCycles);
            
            /* Copy buffer to NAND */
            nandWriteData(pSectorBuff, 512);
                
            /* Copy buffer to NAND and generate ECC */
            eccGenerate512(ecccomp, pSectorBuff);
            pSectorBuff = pSectorBuff + 512;
            
            /* Write sectorinfo structure */
            if (pSectorInfoBuff != NULL) {
                /* Seek to sectorinfo offset */
                nandWriteData(pSectorInfoBuff, sizeof(SectorInfo));
                pSectorInfoBuff++;
            }
			else
			{
				// For the ECC to be written in the correct location,
				// the page needs to be written now and a new write
				// session needs to be opened at the correct FLASH
				// page data index
	            nandWriteCommand(LPCNAND_CMD_PAGE_WRITE2);
		        nandWaitReady(MAXEWRITETIMEINMS);

				nandGetPageIndex(SectorAddr, 8, addrbytes);
		        nandWriteCommand(LPCNAND_CMD_PAGE_READC);
			    nandWriteCommand(LPCNAND_CMD_PAGE_WRITE1);
	            nandWriteAddress(addrbytes, nandDrvDat.addressCycles);
			}
            
            /* Write ECC data */
            nandWriteData(&ecccomp, sizeof(ecccomp));
            nandWriteCommand(LPCNAND_CMD_PAGE_WRITE2);
            nandWaitReady(MAXEWRITETIMEINMS);
        }
        else if (pSectorInfoBuff != NULL) {
            /* Update jsut sectorinfo data */
            nandGetPageIndex(SectorAddr, 0, addrbytes);
            nandWriteCommand(LPCNAND_CMD_PAGE_READC);
            nandWriteCommand(LPCNAND_CMD_PAGE_WRITE1);
            nandWriteAddress(addrbytes, nandDrvDat.addressCycles);
            nandWriteData(pSectorInfoBuff, sizeof(SectorInfo));
            pSectorInfoBuff++;
            nandWriteCommand(LPCNAND_CMD_PAGE_WRITE2);
            nandWaitReady(MAXEWRITETIMEINMS);
        }
        
        nandWriteCommand(LPCNAND_CMD_STATUS);
        nandReadData(&Status, 1);
        if ((Status & 0x1) != 0) {
            nandCSLock(FALSE);
			RETAILMSG(1,
				(TEXT("FMD: Write status failure, status = 0x%02x\r\n"), Status));
			nandMutexUnlock();
            return FALSE;
        }
        
        ++SectorAddr;
        dwNumSectors--;
    }
    
    /* Unlock access and chip select */
    nandCSLock(FALSE);
	nandMutexUnlock();

    return TRUE;
}
