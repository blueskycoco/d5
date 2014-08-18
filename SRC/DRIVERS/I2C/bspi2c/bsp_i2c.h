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
// bsp_i2c.h
//
// General I2C library
//

#pragma once

#include "lpc_types.h"
#include "lpc32xx_i2c.h"
#include "drv_ioctl_funcs.h"

// There has to be a limit on a data transfer size
#define MAXI2CBUFFSIZE 64

class bspI2C
{
public:
	bspI2C(I2C_REGS_T *pI2CRegs);
	~bspI2C();

	// I2C interrupt handler
	I2C_ST_T bspi2cInt(void);

	//  Sets or adjusts the I2C clock rate to the desired
	// clock rate and either symmetrical or asymmetrical mode.
	UINT32 bspi2cSetClock(UINT32 baseclk,
						  UINT32 clock,
						  BOOL asymClk);

	//  Reset the I2C device.
	void bspi2cReset(void);

	//  Returns the status of the last I2C transfer.
	I2C_ST_T bspi2cGetStatus(void);

	//  Get data from the last I2C read transfer, this function
	// will return up to maxBytes of data until no more data is
	// available. The function will return 0 when all data has
	// been read.
	int bspI2CGetData(UINT8 *buff,
					  int maxBytes);

	//  Start an I2C master transfer. All I2C transfers (write
	// and read) start with this function.
	BOOL bspi2cStartXfer(UINT16 *flags_buff,
					     int nBytes,
						 int recBytes);

private:
	// Pointer to I2C registers
	I2C_REGS_T *pI2CRegSet;

	// Transmit side data
	volatile int toSendBytes;           // Number of bytes to send
	volatile int bytesSendCopyIndex;    // Index of next send data in sendBuffer
	UINT16 sendBuffer [MAXI2CBUFFSIZE]; // Send data buffer

	// Receive side data
	volatile int toRecvBytes;           // Number of bytes to receive
	volatile int bytesReceived;         // Number of bytes received
	volatile int bytesRecvPutIndex;     // Index into recvBuffer to put RX data
	volatile int bytesRecvGetIndex;     // Index into recvBuffer to get RX data
	volatile int bytesRecvNullCnt;      // Number of 0xFF values to send for RX
	UINT8 recvBuffer [MAXI2CBUFFSIZE];  // Receive data buffer

	// Last transfer status
	I2C_ST_T I2Cstatus;
};
