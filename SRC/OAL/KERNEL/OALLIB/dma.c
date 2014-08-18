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
// dma.c
//
// Core DMA handler
//

#include <bsp.h>
#include "lpc32xx_dmac.h"
#include "dma.h"

//------------------------------------------------------------------------------
//
// pDMARegs
//
// Pointer to DMA registers
//
static DMAC_REGS_T *pDMARegs;

//------------------------------------------------------------------------------
//
// dmach_to_intr
//
// Array that maps DMA channel value to it's INTR value
//
const DWORD dmach_to_intr[DMAC_CHANNELS_MAX] = {
	OAL_INTR_DMACH_0, OAL_INTR_DMACH_1, OAL_INTR_DMACH_2, OAL_INTR_DMACH_3};

//------------------------------------------------------------------------------
	//
// dmach_enab_cnt
//
// Number of enabled channels, used for master DMA clock control
//
static int dmach_enab_cnt;

//------------------------------------------------------------------------------
//
// dma_init
//
// Initialize DMA controller
//
void dma_init(void)
{
	INT32 idx;
	RETAILMSG(1, (TEXT("DMA Init\r\n")));
	// Save base address of DMA controller registers
	pDMARegs = (DMAC_REGS_T *) OALPAtoVA((UINT32) DMA_BASE, FALSE);

	// Enable clock to DMA controller
	clkpwr_clk_en_dis(CLKPWR_DMA_CLK, 1);

	// Make sure DMA controller and all channels are disabled.
	// Controller is in little-endian mode. Disable sync signals
	pDMARegs->config = 0;
	pDMARegs->sync = 0;

	// Clear interrupt and error statuses
	pDMARegs->int_tc_clear = 0xFF;
	pDMARegs->raw_tc_stat = 0xFF;

	// All DMA channels are initially disabled
	for (idx = 0; idx < DMAC_CHANNELS_MAX; idx++)
	{
		pDMARegs->dma_chan [idx].control = 0;
		pDMARegs->dma_chan [idx].config_ch = 0;
	}

	// Disable clock to DMA controller
	clkpwr_clk_en_dis(CLKPWR_DMA_CLK, 0);

	// No channels are enabled
	dmach_enab_cnt = 0;
}

//------------------------------------------------------------------------------
//
// dma_channel_enable
//
// Enable a DMA channel
//
void dma_channel_enable(INT32 channel) 
{	
	RETAILMSG(1, (TEXT("DMA channel enable!\r\n")));
	clkpwr_clk_en_dis(CLKPWR_DMA_CLK, 1);

	if (dmach_enab_cnt == 0)
	{
		// Enable DMA controller
		pDMARegs->config = DMAC_CTRL_ENABLE;
	}

	// Increment used channel count
	dmach_enab_cnt++;

	// Clear channel
	pDMARegs->dma_chan [channel].control = 0;
	pDMARegs->dma_chan [channel].config_ch = 0;
}

//------------------------------------------------------------------------------
//
// dma_channel_disable
//
// Disable a DMA channel
//
void dma_channel_disable(INT32 channel)
{
	RETAILMSG(1, (TEXT("DMA channel disable!\r\n")));
	pDMARegs->dma_chan [channel].control = 0;
	pDMARegs->dma_chan [channel].config_ch = 0;

	// Decrement used channel count and disable DMA clock if no
	// channels are used
	dmach_enab_cnt++;
	if (dmach_enab_cnt == 0)
	{
		// Disable clock to DMA controller
		pDMARegs->config = 0;
		clkpwr_clk_en_dis(CLKPWR_DMA_CLK, 0);
	}
}

//------------------------------------------------------------------------------
//
// dma_get_int_ch
//
// Get OALINTR value for DMA channel that is requesting interrupt
//
BOOL dma_get_int_ch(DWORD *irq)
{
	DWORD dmapirq;
	INT32 chan = 0;
	BOOL chint = TRUE;
	dmapirq = pDMARegs->int_stat;
	while ((chan < 8) && (chint == TRUE))
	{
		// Find interrupting channel
		if (((1 << chan) & dmapirq) != 0)
		{
			chint = FALSE;
		}
		else
		{
			chan++;
		}
	}

	// Return mapped OALINTR value if channel is good
	if (chan < DMAC_CHANNELS_MAX)
	{
		// DMA channel needs service
		chint = TRUE;
		*irq = dmach_to_intr[chan];
	}
	else if (chan < 8)
	{
		// Disable the offending channel
		dma_channel_disable(chan);
	}

	return chint;
}

//------------------------------------------------------------------------------
//
// OALDMAGetCh
//
// Enable the selected DMA channel and return the mapped OALINTR value for
// the channel.
//
BOOL OALDMAGetCh(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
	DWORD ch;
	BOOL rc = FALSE;
//	RETAILMSG(1, (TEXT("Enable the select DMA channel\r\n")));
    OALMSG(OAL_FUNC, (L"+OALDMAGetCh(...)\r\n"));

    if ((pInpBuffer != NULL) && (pOutBuffer != NULL) &&
		(inpSize == sizeof(DWORD)) &&
		(outSize == sizeof(DWORD)))
	{
        if (code == IOCTL_LPC32XX_DMACHGET)
		{
            // Enable DMA channel and return mapped OALINTR value
			ch = * (DWORD *) pInpBuffer;
			if (ch < DMAC_CHANNELS_MAX)
			{
				dma_channel_enable(ch);
				* (DWORD *) pOutBuffer = dmach_to_intr [ch];
				rc = TRUE;
			}
		}
    }

    OALMSG(OAL_FUNC, (L"-OALDMAGetCh(...), rc=%d\r\n", rc));
    return rc;
}

//------------------------------------------------------------------------------
//
// OALDMAFreeCh
//
// Free the selected DMA channel
//
BOOL OALDMAFreeCh(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize)
{
	DWORD ch;
	BOOL rc = FALSE;

    OALMSG(OAL_FUNC, (L"+OALDMAFreeCh(...)\r\n"));
	RETAILMSG(1, (TEXT("free the select dma channel\r\n")));
    if ((pInpBuffer != NULL) && (inpSize == sizeof(DWORD)))
	{
        if (code == IOCTL_LPC32XX_DMACHFREE)
		{
            // Disable DMA channel
			ch = * (DWORD *) pInpBuffer;
			if (ch < DMAC_CHANNELS_MAX)
			{
				dma_channel_disable(ch);
				rc = TRUE;
			}
		}
    }

    OALMSG(OAL_FUNC, (L"-OALDMAFreeCh(...), rc=%d\r\n", rc));
    return rc;
}
