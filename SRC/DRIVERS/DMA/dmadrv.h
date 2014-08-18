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
// dmadrv.h
//
// DMA driver
//

#pragma once

#include <windows.h>
#include <nkintr.h>
#include "lpc32xx_dmac.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure used with the dmaAllocate() function for initial setup
typedef struct
{
	int dmaCh;            // DMA channel to used
	DWORD perID;          // Peripheral ID for this channel (such as DMA_PER_I2S0_DMA0)
	DWORD SrcInc;         // Enable for source data increment
	DWORD DestInc;        // Enable for destination data increment
	DWORD SrcWidth;       // DMAC_CHAN_SRC_WIDTH_8, DMAC_CHAN_SRC_WIDTH_16, DMAC_CHAN_SRC_WIDTH_32
	DWORD DestWidth;      // DMAC_CHAN_DEST_WIDTH_8, DMAC_CHAN_DEST_WIDTH_16, DMAC_CHAN_DEST_WIDTH_32
	DWORD SrcBurstSize;   // DMAC_CHAN_DEST_BURST_xxx, onl applicable for SD card DMA
	DWORD DestBurstSize;  // DMAC_CHAN_DEST_BURST_xxx, onl applicable for SD card DMA
	DWORD perFlowSource;  // Flag used to select flow control, DMAC_CHAN_FLOW_xxxx
	DWORD destPeripheral; // Peripheral used for DMA destination, DMAC_DEST_PERIP(xxx)
	DWORD srcPeripheral;  // Peripheral used for DMA source, DMAC_SRC_PERIP(xxx)
} DMASETUP_T;

// DMA channel information structure
typedef struct
{
	int dmaCh;               // Mapped DMA channel
	DWORD irq;               // Mapped WinCE IRQ number
	DWORD perID;             // Mapped peripheral ID
	DWORD pBuffPhy;          // Physical address of DMA buffer
	DWORD pBuffVirt;         // Virtual address of DMA buffer
	DWORD buffSize;          // Size (contiguous) of DMA buffer in bytes
} DMAINFO_T;

// Allocates and enables a DMA channel for peripheral operation
DWORD dmaAllocate(DMASETUP_T *pDmaSetup,
				  DWORD buffSizeBytes);

// Disables and deallocates a DMA channel used for peripheral operation
DWORD dmaDeAllocate(DWORD dmaData);

// Returns DMA channel information
DMAINFO_T *dmaGetInfo(DWORD dmaData);

// Starts a DMA channel's data transfer
void dmaEnable(DWORD dmaData);

// Stops a DMA channel's data transfer
void dmaDisable(DWORD dmaData);

// Returns DMA on status
DWORD dmaIsOn(DWORD dmaData);

// Returns address to DMA registers
DMAC_REGS_T *dmaGetRegPtr(DWORD dmaData);

// Starts a DMA transfer with the passed attributes (not LLI)
DWORD dmaTranEntry(DWORD dmaData,
				   DWORD src,      // Physical address
				   DWORD dst,      // Physical address
				   DWORD bytes);   // Number of bytes to transfer

// Adds a new entry to the DMA linked list and starts DMA
DWORD dmaListEntry(DWORD dmaData,
				   DWORD src,      // Physical address
				   DWORD dst,      // Physical address
				   DWORD bytes);   // Number of bytes to transfer

// Pops a single list entry and returns the number of bytes sent in
// that list
DWORD dmaCleanUp(DWORD dmaData);

// Disable and reset the current DMA list
void dmaListReset(DWORD dmaData);

// Change the current channel's attributes, adjusts channel configuation
// without creating new buffers
DWORD dmaChanConfig(DWORD dmaData,
					DMASETUP_T *pDmaSetup);

// Diagnostic: Dumps current DMA registers
void dmaDumpRegs(DWORD dmaData);

#ifdef __cplusplus
}
#endif
