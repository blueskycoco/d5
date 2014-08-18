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
// dmadrv.c
//
// DMA driver
//

#include <windows.h>
#include <nkintr.h>
#include <ceddk.h>
#include "bsp.h"
#include "dmadrv.h"

// Number of DMA list entries
#define DMALISTENTRIES 16

// DMA list control structure
typedef struct
{
	DMAC_REGS_T *pDMARegs;   // Pointer to DMA registers
	DMAINFO_T dmaInfo;       // Saved DMA configuration info
	UINT32 chMaskBit;        // Preshifted channel bit
	DWORD savedControl;      // Control word used in LLIs
	DWORD savedConfig;       // Saved channel configuration value
	DWORD xferSize;          // Transfer size (in bytes)
	DMAC_CHAN_T *pDMAChan;

	// DMA linked list control data
	DMAC_LL_T *pDMAListVirt; // Virtual pointer to DMA list memory area
	DMAC_LL_T *pDMAListPhy;  // Physical pointer to DMA list memory area
	int GetListIdx;          // Next index for clearing a list
	int PutListIdx;          // Next index for placing a list
	int ListEntries;         // Number of entries currently in list
} DMADRVDATA_T;

//-----------------------------------------------------------------------------
//
// dmaStartDMA
//
// Start a DMA transfer
//
static void dmaStartDMA(DMADRVDATA_T *pDrvData,
						DWORD src,      // Physical address
				        DWORD dst,      // Physical address
						DWORD lli,      // Physical address
				        DWORD ctrl)     // Control word
{
	pDrvData->pDMAChan->src_addr = (UNS_32) src;
	pDrvData->pDMAChan->dest_addr = (UNS_32) dst;
	pDrvData->pDMAChan->lli = (UNS_32) lli;
	pDrvData->pDMAChan->control = ctrl;

	// Start transfer
	pDrvData->pDMAChan->config_ch |= DMAC_CTRL_ENABLE;
}

//-----------------------------------------------------------------------------
//
// dmaToPhy
//
// Converts a virtual DMA list address to a physical address
//
static DMAC_LL_T *dmaToPhy(DMADRVDATA_T *pDrvDat,
			               DMAC_LL_T *va)
{	
	DWORD offset = 0;
	if ((DWORD) va != 0)
	{
		offset = (DWORD) va - (DWORD) pDrvDat->pDMAListVirt;
		offset = offset + (DWORD) pDrvDat->pDMAListPhy;
	}
	return (DMAC_LL_T *) offset;
}

//-----------------------------------------------------------------------------
//
// dmaToVirt
//
// Converts a physical DMA list address to a virtual address
//
static DMAC_LL_T *dmaToVirt(DMADRVDATA_T *pDrvDat,
			                DMAC_LL_T *pa)
{	
	DWORD offset = 0;

	if ((DWORD) pa != 0)
	{
		offset = (DWORD) pa - (DWORD) pDrvDat->pDMAListPhy;
		offset = offset + (DWORD) pDrvDat->pDMAListVirt;
	}

	return (DMAC_LL_T *) offset;
}

//------------------------------------------------------------------------------
//
// dmaAllocList
//
// Allocate a space of DMA memory for a DMA linked list
//
DWORD dmaAllocate(DMASETUP_T *pDmaSetup,
				  DWORD buffSizeBytes)
{
	DMADRVDATA_T *pDrvData;
    PHYSICAL_ADDRESS pa;
    DMA_ADAPTER_OBJECT Adapter;
	DWORD bytesret;

	// Allocate space for the DMA control structure
	pDrvData = (DMADRVDATA_T *) malloc(sizeof(DMADRVDATA_T));
	if (pDrvData == NULL)
	{
		return 0;
	}

	// Setup some defaults
	pDrvData->pDMARegs = NULL;
	pDrvData->dmaInfo.irq = 0xFFFFFFFF;
	pDrvData->dmaInfo.dmaCh = pDmaSetup->dmaCh;
	pDrvData->dmaInfo.perID = pDmaSetup->perID;
	pDrvData->dmaInfo.pBuffPhy = 0;
	pDrvData->dmaInfo.pBuffVirt = (DWORD) NULL;
	pDrvData->dmaInfo.buffSize = buffSizeBytes;
	pDrvData->chMaskBit = _BIT(pDrvData->dmaInfo.dmaCh);
	pDrvData->savedControl = 0;
	pDrvData->savedConfig = 0;
	pDrvData->pDMAListVirt = NULL;
	pDrvData->pDMAListPhy = NULL;
	pDrvData->xferSize = 1;

	// Map space for DMA registers
	pa.HighPart = 0;
	pa.LowPart = DMA_BASE;
	pDrvData->pDMARegs = MmMapIoSpace(pa, sizeof (DMAC_REGS_T), FALSE);
	if (pDrvData->pDMARegs == NULL)
	{
        goto dmaErr;
	}

	// Enable the channel and clocking in the kernel
	if (KernelIoControl(IOCTL_LPC32XX_DMACHGET, &pDrvData->dmaInfo.dmaCh,
		sizeof(pDrvData->dmaInfo.dmaCh), &pDrvData->dmaInfo.irq,
		sizeof (pDrvData->dmaInfo.irq), &bytesret) == FALSE)
	{
		// Error setting up DMA channel
		pDrvData->dmaInfo.irq = 0xFFFFFFFF;
		goto dmaErr;
	}

	// Allocate space for the DMA buffers if buffers are requested
	if (buffSizeBytes != 0)
	{
	    memset(&Adapter, 0, sizeof(DMA_ADAPTER_OBJECT));
		Adapter.InterfaceType = Internal;
	    Adapter.ObjectSize = sizeof(DMA_ADAPTER_OBJECT);
		pDrvData->dmaInfo.pBuffVirt = (DWORD) HalAllocateCommonBuffer(&Adapter,
			(pDrvData->dmaInfo.buffSize), &pa, FALSE);
	    if (pDrvData->dmaInfo.pBuffVirt == (DWORD) NULL)
		{
	        goto dmaErr;
		}
		pDrvData->dmaInfo.pBuffPhy = pa.LowPart;
	}

	// Allocate space for the DMA list
    memset(&Adapter, 0, sizeof(DMA_ADAPTER_OBJECT));
    Adapter.InterfaceType = Internal;
    Adapter.ObjectSize = sizeof(DMA_ADAPTER_OBJECT);
    pDrvData->pDMAListVirt = (DMAC_LL_T *) HalAllocateCommonBuffer(&Adapter,
		(sizeof(DMAC_LL_T) * DMALISTENTRIES), &pa, FALSE);
    if (pDrvData->pDMAListVirt == NULL)
    {
        goto dmaErr;
    }
	pDrvData->pDMAListPhy = (DMAC_LL_T *) pa.LowPart;
    RETAILMSG(1, (TEXT("DMA list addr = (V)0x%x (P)0x%x\r\n"), pDrvData->pDMAListVirt, pDrvData->pDMAListPhy));

	// Setup sync and request
	pDrvData->pDMARegs->sync &= ~_BIT(pDrvData->dmaInfo.perID);
	// Save premade pointer to DMA channel registers
	pDrvData->pDMAChan = (DMAC_CHAN_T *) &pDrvData->pDMARegs->dma_chan[pDrvData->dmaInfo.dmaCh];
	// Make sure DMA is disabled
	dmaDisable((DWORD) pDrvData);;
	// Clear initial channel values
	pDrvData->pDMAChan->src_addr = 0;
	pDrvData->pDMAChan->dest_addr = 0;
	pDrvData->pDMAChan->lli = 0;
	// Setup channel values
	dmaChanConfig((DWORD) pDrvData, pDmaSetup);

	// Reset DMA list
	dmaListReset((DWORD) pDrvData);
	RETAILMSG(1, 
				(TEXT("DMA pDrvData = 0x%xr\n"),(DWORD) pDrvData));
	return ((DWORD)pDrvData) ;

dmaErr:
	dmaDeAllocate((DWORD) pDrvData);

	return 0;
}

//-----------------------------------------------------------------------------
//
// dmaDeAllocate
//
// Disables and deallocates a DMA channel used for peripheral operation
//
DWORD dmaDeAllocate(DWORD dmaData)
{
	DWORD bytesret;
    PHYSICAL_ADDRESS pa;
    DMA_ADAPTER_OBJECT Adapter;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	if (pDrvData->pDMARegs != NULL)
	{
		// Disable DMA channel
		dmaDisable(dmaData);

		// Unmap DMA registers
		MmUnmapIoSpace((PVOID) pDrvData->pDMARegs,
			sizeof (DMAC_REGS_T));
	}

	// Remove DMA buffers
	if (pDrvData->dmaInfo.pBuffVirt == (DWORD) NULL)
	{
	    memset(&Adapter, 0, sizeof(DMA_ADAPTER_OBJECT));
	    Adapter.InterfaceType = Internal;
	    Adapter.ObjectSize = sizeof(DMA_ADAPTER_OBJECT);
		pa.HighPart = 0;
		pa.LowPart = pDrvData->dmaInfo.pBuffPhy;
		HalFreeCommonBuffer(&Adapter,
			(ULONG) (pDrvData->dmaInfo.buffSize), pa,
			(PVOID) pDrvData->dmaInfo.pBuffVirt, FALSE);
	}

	// Remove DMA list area
	if (pDrvData->pDMAListVirt != NULL)
	{
	    memset(&Adapter, 0, sizeof(DMA_ADAPTER_OBJECT));
	    Adapter.InterfaceType = Internal;
	    Adapter.ObjectSize = sizeof(DMA_ADAPTER_OBJECT);
		pa.HighPart = 0;
		pa.LowPart = (ULONG) pDrvData->pDMAListPhy;
		HalFreeCommonBuffer(&Adapter,
			(ULONG) (sizeof(DMAC_LL_T) * DMALISTENTRIES), pa,
			(PVOID) pDrvData->pDMAListVirt, FALSE);
	}

	// Free DMA channel in the kernel (also disables clocks)
	KernelIoControl(IOCTL_LPC32XX_DMACHFREE, &pDrvData->dmaInfo.dmaCh,
		sizeof(pDrvData->dmaInfo.dmaCh), NULL, 0, &bytesret);

	free(pDrvData);

	return TRUE;
}

//-----------------------------------------------------------------------------
//
// dmaGetInfo
//
// Returns DMA channel information
//
DMAINFO_T *dmaGetInfo(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	return &pDrvData->dmaInfo;
}

//-----------------------------------------------------------------------------
//
// dmaEnable
//
// Starts a DMA channel's data transfer
//
void dmaEnable(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	pDrvData->pDMAChan->config_ch |= DMAC_CHAN_ENABLE;
}

//-----------------------------------------------------------------------------
//
// dmaDisable
//
// Stops a DMA channel's data transfer
//
void dmaDisable(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	pDrvData->pDMAChan->config_ch &= ~DMAC_CHAN_ENABLE;
}

//-----------------------------------------------------------------------------
//
// dmaIsOn
//
// Returns DMA on status
//
DWORD dmaIsOn(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	if ((pDrvData->pDMAChan->config_ch & DMAC_CHAN_ENABLE) != 0)
	{
		return 1;
	}

	return 0;
}

//-----------------------------------------------------------------------------
//
// dmaGetRegPtr
//
// Returns address to DMA registers
//
DMAC_REGS_T *dmaGetRegPtr(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	return pDrvData->pDMARegs;
}

//-----------------------------------------------------------------------------
//
// dmaTranEntry
//
// Starts a DMA transfer with the passed attributes (not LLI)
//
DWORD dmaTranEntry(DWORD dmaData,
				   DWORD src,      // Physical address
				   DWORD dst,      // Physical address
				   DWORD bytes)    // Number of bytes to transfer
{
	DWORD ctrl;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;
	DWORD trans;

	trans = bytes / pDrvData->xferSize;
	if (trans >= 4096)
	{
		return 0;
	}

	if (dmaIsOn(dmaData) != 0)
	{
		// Channel is currently on
		return 0;
	}

	// Update config word
	pDrvData->pDMAChan->config_ch = pDrvData->savedConfig;

	// Setup control word
	ctrl = (pDrvData->savedControl | DMAC_CHAN_TRANSFER_SIZE(trans) |
		DMAC_CHAN_INT_TC_EN);

	// Start transfer
	dmaStartDMA((DMADRVDATA_T *) dmaData, src, dst, 0, ctrl);

	return bytes;
}

//-----------------------------------------------------------------------------
//
// dmaListReset
//
// Disable and reset the current DMA list
//
void dmaListReset(DWORD dmaData)
{
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	dmaDisable(dmaData);
	pDrvData->GetListIdx = 0;
	pDrvData->PutListIdx = 0;
	pDrvData->ListEntries = 0;
	pDrvData->pDMAChan->lli = 0;
}

//-----------------------------------------------------------------------------
//
// dmaChanConfig
//
// Change the current channel's attributes, adjusts channel configuation
// without creating new buffers
//
DWORD dmaChanConfig(DWORD dmaData,
					DMASETUP_T *pDmaSetup)
{
	UINT32 tmp;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	// Setup control word for the channel
	// Source on AHB0, destination on AHB1
	tmp = (DMAC_CHAN_SRC_AHB1 | pDmaSetup->SrcWidth | pDmaSetup->DestWidth |
//		DMAC_CHAN_DEST_AHB1 | // TBD
//	tmp = (0 | pDmaSetup->SrcWidth | pDmaSetup->DestWidth |
//		DMAC_CHAN_DEST_AHB1 | // TBD
		pDmaSetup->SrcBurstSize | pDmaSetup->DestBurstSize);
	if (pDmaSetup->DestInc != 0)
	{
		tmp |= DMAC_CHAN_DEST_AUTOINC;
	}
	if (pDmaSetup->SrcInc != 0)
	{
		tmp |= DMAC_CHAN_SRC_AUTOINC;
	}
	pDrvData->savedControl = tmp;
	pDrvData->pDMAChan->control = 0;

	// Compute transfer size
	if (pDmaSetup->SrcWidth == DMAC_CHAN_SRC_WIDTH_8)
	{
		pDrvData->xferSize = 1;
	}
	else if (pDmaSetup->SrcWidth == DMAC_CHAN_SRC_WIDTH_16)
	{
		pDrvData->xferSize = 2;
	}
	else
	{
		pDrvData->xferSize = 4;
	}

	// Setup channel configuration word for the channel
	// DMA requests enabled, channel not enabled
	tmp = (pDmaSetup->perFlowSource | pDmaSetup->destPeripheral |
		pDmaSetup->srcPeripheral | DMAC_CHAN_ITC);
	pDrvData->savedConfig = tmp;
	pDrvData->pDMAChan->config_ch = tmp;

	return 1;
}

//-----------------------------------------------------------------------------
//
// dmaDumpRegs
//
// Dumps current DMA registers
//
void dmaDumpRegs(DWORD dmaData)
{
	int idx;
	UINT32 *pReg;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;

	idx = 0;
	pReg = (UINT32 *) pDrvData->pDMARegs;
	for (idx = 0; idx < 14; idx++)
	{
		RETAILMSG(1,
		    (TEXT("dmaDumpRegs: Reg 0x%x = 0x%x\r\n"), pReg, *pReg));
		pReg++;
	}

	RETAILMSG(1,
	    (TEXT("dmaDumpRegs: Dumping DMA channel(%d) registers\r\n"), pDrvData->dmaInfo.dmaCh));
	idx = 0;
	pReg = (UINT32 *) pDrvData->pDMAChan;
	for (idx = 0; idx < 5; idx++)
	{
		RETAILMSG(1,
		    (TEXT("dmaDumpRegs: Reg 0x%x = 0x%x\r\n"), pReg, *pReg));
		pReg++;
	}
}

//-----------------------------------------------------------------------------
//
// dmaCleanUp
//
// Pops a single list entry and returns the number of bytes sent in
// that list
//
DWORD dmaCleanUp(DWORD dmaData)
{
	int toproc;
	DWORD trans = 0;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;
	DMAC_LL_T *PCurHWList, *pExpListVirt;

	// Only process if entries need processing
	if (pDrvData->ListEntries > 0)
	{
		// Current DMA list entry to check before resubmitting to pool
		pExpListVirt = pDrvData->pDMAListVirt + pDrvData->GetListIdx;

		// Current HW pointer and convert to virtual address
		PCurHWList = (DMAC_LL_T *) (pDrvData->pDMAChan->lli &
			~DMAC_CHAN_LLI_SEL_AHB1);
		PCurHWList = dmaToVirt(pDrvData, PCurHWList);

		// If DMA is stopped, then always pop the list
		if (dmaIsOn(dmaData) == 0)
		{
			// Get size of the completed transfer
			trans = pExpListVirt->next_ctrl;
			trans = trans & DMAC_CHAN_TRANSFER_SIZE(0xFFF);

			// One entry cleaned, can be used by dmaListEntry()
			pDrvData->ListEntries--;
		}
		else if (PCurHWList != NULL)
		{
			// DMA is not idle and there is 2 or more entries on the
			// list. Determine if an entry needs to be popped.

			// Determine the number of entries the DMA has to process
			if (PCurHWList > pExpListVirt)
			{
				toproc = (DWORD) PCurHWList - (DWORD) pExpListVirt;
			}
			else
			{
				toproc = (DWORD) pExpListVirt - (DWORD) PCurHWList;
			}
			toproc = toproc / sizeof (DMAC_LL_T);

			// If there are less entries for DMA processing than what
			// was submitted, then an entry can be popped off the list
			if (toproc != pDrvData->ListEntries)
			{
				// Get size of the completed transfer
				trans = pExpListVirt->next_ctrl;
				trans = trans & DMAC_CHAN_TRANSFER_SIZE(0xFFF);

				// One entry cleaned, can be used by dmaListEntry()
				pDrvData->ListEntries--;
			}
		}
		else
		{
			// Current HW list is NULL
			if (pDrvData->ListEntries > 1)
			{
				// Get size of the completed transfer
				trans = pExpListVirt->next_ctrl;
				trans = trans & DMAC_CHAN_TRANSFER_SIZE(0xFFF);

				// One entry cleaned, can be used by dmaListEntry()
				pDrvData->ListEntries--;
			}
		}
	}

	// Switch to next check entry
	if (trans > 0)
	{
		pDrvData->GetListIdx++;
		if (pDrvData->GetListIdx >= DMALISTENTRIES)
		{
			pDrvData->GetListIdx = 0;
		}

		// Readjust for bytes
		trans = trans * pDrvData->xferSize;
	}

	return trans;
}

//-----------------------------------------------------------------------------
//
// dmaListEntry
//
// Adds a new entry to the DMA linked list and starts DMA
//
DWORD dmaListEntry(DWORD dmaData,
				   DWORD src,      // Physical address
				   DWORD dst,      // Physical address
				   DWORD bytes)    // Number of bytes to transfer
{
	int prevIdx;
	DWORD trans, added = 0;
	DMADRVDATA_T *pDrvData = (DMADRVDATA_T *) dmaData;
	DMAC_LL_T *pPrevListV, *pCurListV, *pCurListP, *PCurHWList;

	trans = bytes / pDrvData->xferSize;
	if ((trans == 0) || (trans >= 4096))
	{
		return 0;
	}

	// Only process if there is enough space
	if (pDrvData->ListEntries < DMALISTENTRIES)
	{
		// Get pointer to next DMA list entry to put
		pCurListV = pDrvData->pDMAListVirt + pDrvData->PutListIdx;

		// Get pointer to previous DMA list entry
		prevIdx = pDrvData->PutListIdx - 1;
		if (prevIdx < 0)
		{
			prevIdx = DMALISTENTRIES - 1;
		}
		pPrevListV = pDrvData->pDMAListVirt + prevIdx;

		// Get physical address for list entry to put
		pCurListP = dmaToPhy(pDrvData, pCurListV);

		// Increment buffer and put counts
		pDrvData->ListEntries++;
		pDrvData->PutListIdx++;
		if (pDrvData->PutListIdx >= DMALISTENTRIES)
		{
			pDrvData->PutListIdx = 0;
		}

		// Add to next entry in the list
		pCurListV->dma_src = src;
		pCurListV->dma_dest = dst;
		pCurListV->next_lli = 0;
		trans = bytes / pDrvData->xferSize;
		pCurListV->next_ctrl = (pDrvData->savedControl |
			DMAC_CHAN_TRANSFER_SIZE(trans) | DMAC_CHAN_INT_TC_EN);

		// Update previous link pointer to point to current link
		pPrevListV->next_lli = ((UNS_32) pCurListP |
			DMAC_CHAN_LLI_SEL_AHB1);

		// If next link is NULL, then update the current DMA HW link to
		// the next link
		PCurHWList = (DMAC_LL_T *) pDrvData->pDMAChan->lli;
		if (PCurHWList == 0)
		{
			pDrvData->pDMAChan->lli = ((UNS_32) pCurListP |
				DMAC_CHAN_LLI_SEL_AHB1);
		}

		// If DMA is idle, then the DMA list was empty and the previous
		// link update occured after DMA was disabled, so restart or
		// start DMA
		if (dmaIsOn(dmaData) == 0)
		{
			// Start transfer
			dmaStartDMA(pDrvData, pCurListV->dma_src, pCurListV->dma_dest,
				0, pCurListV->next_ctrl);
		}

		added = 1;
	}

	return added;
}
