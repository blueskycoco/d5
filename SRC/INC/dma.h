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
// dma.h
//
// DMA defines and functions
//

#pragma once

#include <nkintr.h>

// Maximum number of DMA channel used
#define DMAC_CHANNELS_MAX     4

// DMA channel allocation for various devices, use with the
// dma_channel_enable() function.
#define DMAC_SDCARD_CH        0
#define DMAC_AUDIO_TX_CH      1
#define DMAC_AUDIO_RX_CH      2
#define DMAC_NAND_CH          3

// Initialize DMA controller
void dma_init(void);

// Enable a DMA channel
void dma_channel_enable(INT32 channel);

// Disable a DMA channel
void dma_channel_disable(INT32 channel);

// Get OALINTR value for DMA channel that is requesting interrupt
BOOL dma_get_int_ch(DWORD *irq);
