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
// ssp_driver.h
//
// SSP driver
//

#include <windows.h>
#include "bsp_ssp.h"

//------------------------------------------------------------------------------
//
// bspssp
//
// Class constructor
//
bspssp::bspssp(SSP_REGS_T *pSSPRegs,
			   CLKPWR_REGS_T *pCLKPWRRegs,
			   int device)
{
	SSP_XFER_SETUP_T sspcfg;
	volatile UINT32 tmp;

	// Save pointers and device
	m_pSSPRegSet = pSSPRegs;
	m_pCLKPWRRegSet = pCLKPWRRegs;
	m_device = device;

	// Enable clock for this device
	if (m_device == 0)
	{
		m_pCLKPWRRegSet->clkpwr_ssp_blk_ctrl |= CLKPWR_SSPCTRL_SSPCLK0_EN;
	}
	else
	{
		m_pCLKPWRRegSet->clkpwr_ssp_blk_ctrl |= CLKPWR_SSPCTRL_SSPCLK1_EN;
	}

	// Setup for 8 bit transfers, 100K clock, locked CS
    sspcfg.databits = 8;
    sspcfg.mode = SSP_CR0_FRF_MOT;
    sspcfg.highclkSpiFrames = FALSE;
    sspcfg.usesecondClkSpi = FALSE;
    sspcfg.sspClk = 5000000;
    sspcfg.masterMode = TRUE;
	bspsspSetup(104000000, &sspcfg); // Slow clock initially

	// Flush the receive FIFO
	while ((m_pSSPRegSet->sr & SSP_SR_RNE) != 0) 
	{
		tmp = m_pSSPRegSet->data;
	}

	// Clear and enable receive interrupts
	m_pSSPRegSet->icr = 0xF;
	m_pSSPRegSet->imsc = (SSP_IIR_RXRIS | SSP_IIR_RORRIS | SSP_IIR_RTRIS);

	// Enable SSP0 device
	m_pSSPRegSet->cr1 |= SSP_CR1_ENABLE;
}

//------------------------------------------------------------------------------
//
// ~bspssp
//
// Class destructor
//
bspssp::~bspssp()
{
	// Disable SSP0 device
	m_pSSPRegSet->cr1 &= ~SSP_CR1_ENABLE;

	// Disable clock for this device
	if (m_device == 0)
	{
		m_pCLKPWRRegSet->clkpwr_ssp_blk_ctrl &= ~CLKPWR_SSPCTRL_SSPCLK0_EN;
	}
	else
	{
		m_pCLKPWRRegSet->clkpwr_ssp_blk_ctrl &= ~CLKPWR_SSPCTRL_SSPCLK1_EN;
	}
}

//------------------------------------------------------------------------------
//
// bspsspSetup
//
// Setup the SSP peripheral
//
void bspssp::bspsspSetup(UINT32 baseClock,
				         SSP_XFER_SETUP_T *pSSPCfg)
{
	UINT32 tmp0, tmp1, prescale, cr0_div, cmp_clk;

	// Setup CR0 word first
	tmp0 = SSP_CR0_DSS(pSSPCfg->databits);
	m_dsize = 1;
	if (pSSPCfg->databits > 8)
	{
		m_dsize = 2;
	}

	// Mode
	tmp0 |= pSSPCfg->mode;

	// SPI clock control
	if (pSSPCfg->highclkSpiFrames == TRUE) 
	{
		tmp0 |= SSP_CR0_CPOL;
	}
	if (pSSPCfg->usesecondClkSpi == TRUE) 
	{
		tmp0 |= SSP_CR0_CPHA;
	}

	// Master/slave mode control
	tmp1 = 0;
	if (pSSPCfg->masterMode == FALSE) 
	{
		tmp1 = SSP_CR1_MS;
	}

	// Find closest divider to get at or under the target frequency.
	// Use smallest prescaler possible and rely on the divider to get
	// the closest target frequency
	cr0_div = 0;
	cmp_clk = 0xFFFFFFFF;
	prescale = 2;
	while (cmp_clk > baseClock) {
		cmp_clk = pSSPCfg->sspClk / ((cr0_div + 1) * prescale);
		if (cmp_clk > baseClock) 
		{
			cr0_div++;
			if (cr0_div > 0xFF) 
			{
				cr0_div = 0;
				prescale += 2;
			}
		}
	}
	tmp0 |= SSP_CR0_SCR(cr0_div - 1);

	// Setup peripheral via registers
	m_pSSPRegSet->cr0 = tmp0;
	tmp0 = m_pSSPRegSet->cr1 & SSP_CR1_ENABLE;
	m_pSSPRegSet->cr1 = (tmp0 | tmp1);
    m_pSSPRegSet->cpsr = prescale;
}

//------------------------------------------------------------------------------
//
// bspsspInt
//
// SSP interrupt handler
//
void bspssp::bspsspInt(SSP_XFER_T *pSSPXfer)
{
	UINT8 *p8;
	UINT16 *p16;
	UINT32 tmp1;

	// Setup for receive first
	p8 = (UINT8 *) pSSPXfer->recvBuff;
	p16 = (UINT16 *) pSSPXfer->recvBuff;
	pSSPXfer->recvBuffBytesFilled = 0;
	pSSPXfer->recvDataOflow = FALSE;

	// Empty receive FIFO first
	if (pSSPXfer->recvBuff != NULL)
	{
	    while ((pSSPXfer->recvBuffSize > 0) && ((m_pSSPRegSet->sr & SSP_SR_RNE) != 0))
        {
        	tmp1 = m_pSSPRegSet->data;
        	if (m_dsize == 1) 
        	{
        		*p8 = (UNS_8) tmp1;
        		p8++;
        	}
        	else 
        	{
        		*p16 = (UNS_16) tmp1;
        		p16++;
        	}

            /* Increment data count and decrement buffer size count */
            pSSPXfer->recvBuffBytesFilled += m_dsize;
			pSSPXfer->recvBuffSize -= m_dsize;
        }
	}

	// Return receive error status
	if ((m_pSSPRegSet->mis & SSP_IIR_RORRIS) != 0)
	{
		pSSPXfer->recvDataOflow = TRUE;
		m_pSSPRegSet->icr = 0xF;
	}

	// Setup for send
	p8 = (UINT8 *) pSSPXfer->sendBuff;
	p16 = (UINT16 *) pSSPXfer->sendBuff;
	pSSPXfer->actSendBuffBytes = 0;

	// Stuff FIFO until full
    while ((pSSPXfer->sendBuffBytes > 0) && ((m_pSSPRegSet->sr & SSP_SR_TNF) != 0))
    {
      	if (m_dsize == 1) 
       	{
       		m_pSSPRegSet->data = (UNS_32) *p8;
       		p8++;
       	}
       	else 
       	{
       		m_pSSPRegSet->data = (UNS_32) *p16;
       		p16++;
       	}

        // Increment data count and decrement buffer size count
        pSSPXfer->actSendBuffBytes += m_dsize;
        pSSPXfer->sendBuffBytes -= m_dsize;
    }

    if (pSSPXfer->actSendBuffBytes > 0)
    {
		// Enable transmit interrupt
     	m_pSSPRegSet->imsc |= SSP_IIR_TXRIS;
    }
	else
	{
		// Disable transmit interrupt
     	m_pSSPRegSet->imsc &= ~SSP_IIR_TXRIS;
	}
}
