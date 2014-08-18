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

#pragma once

#include "lpc_types.h"
#include "lpc32xx_ssp.h"
#include "lpc32xx_clkpwr.h"
#include "drv_ioctl_funcs.h"

// Structure used for data control between the interrupt and controlling driver
typedef struct
{
	void *recvBuff;           // Pointer to receive buffer (must be 16-bit aligned if data size > 8 bits)
	int recvBuffSize;         // Size of receive buffer
	int recvBuffBytesFilled;  // Return value of actual number of bytes placed into receive buffer
	BOOL recvDataOflow;       // Receive data buffer overflowed flag
	void *sendBuff;           // Pointer to send buffer (must be 16-bit aligned if data size > 8 bits)
	int sendBuffBytes;        // Number of bytes to send currently in the send buffer
	int actSendBuffBytes;     // Return value of actual number of sent in send buffer
} SSP_XFER_T;

class bspssp
{
public:
	bspssp(SSP_REGS_T *pSSPRegs,
		   CLKPWR_REGS_T *pCLKPWRRegs,
		   int device);
	~bspssp();

	//  Setup the SSP peripheral
	void bspsspSetup(UINT32 baseClock,
		             SSP_XFER_SETUP_T *pSSPCfg);

	// SSP interrupt handler
	void bspsspInt(SSP_XFER_T *pSSPXfer);

private:
	// Saved pointers to registers
	SSP_REGS_T *m_pSSPRegSet;
	CLKPWR_REGS_T *m_pCLKPWRRegSet;

	// Device number for SSP peripheral (0 or 1)
	int m_device;

	// Data size in bits, used for buffer and FIFO object size control
	int m_dsize;
};
