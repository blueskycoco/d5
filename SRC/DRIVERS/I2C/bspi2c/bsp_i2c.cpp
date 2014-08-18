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
// bsp_i2c.c
//
// General I2C library
//

#include <windows.h>
#include "bsp_i2c.h"

//------------------------------------------------------------------------------
//
// bspi2c
//
// Class constructor
//
bspI2C::bspI2C(I2C_REGS_T *pI2CRegs)
{
	pI2CRegSet = pI2CRegs;

	// Reset I2C
	bspi2cReset();

	// Set defaults
	I2Cstatus = I2CST_ERROR;
	toSendBytes = 0;
	bytesSendCopyIndex = 0;
	toRecvBytes = 0;
	bytesRecvNullCnt = 0;
	bytesReceived = 0;
	bytesRecvPutIndex = 0;
	bytesRecvGetIndex = 0;

	// Setup default clocking to slowest speed
	bspi2cSetClock(104000000, 100000, TRUE);
}

//------------------------------------------------------------------------------
//
// ~bspi2c
//
// Class destructor
//
bspI2C::~bspI2C()
{
	// Reset I2C
	bspi2cReset();
}

//------------------------------------------------------------------------------
//
// bspi2cInt
//
// I2C interrupt handler
//
I2C_ST_T bspI2C::bspi2cInt(void)
{
	UINT32 sts;
	int bsend = 0;

	// Copy data from the RX FIFO into the buffer
	while ((pI2CRegSet->i2c_stat & (1 << 9)) == 0)
	{
		recvBuffer[bytesReceived] = (UNS_8) pI2CRegSet->i2c_txrx;
		bytesReceived++;
	}

	// Place data into the FIFO with selected flags. Either place all the data
	// in the FIFO or fill it until it is full
	while ((bsend < toSendBytes) && ((pI2CRegSet->i2c_stat & (1 << 10)) == 0))
	{
		pI2CRegSet->i2c_txrx = (UINT32) sendBuffer[bytesSendCopyIndex];
		bytesSendCopyIndex++;
		bsend++;
	}
	toSendBytes -= bsend;

	// Still need to send out 0xFF's for RX data?
	while ((bytesRecvNullCnt > 0) &&
		((pI2CRegSet->i2c_stat & (1 << 10)) == 0))
	{
		if (bytesRecvNullCnt == 1)
		{
			pI2CRegSet->i2c_txrx = (UINT32) (0xFF | I2C_FLAG_STOP);
		}
		else
		{
			pI2CRegSet->i2c_txrx = (UINT32) 0xFF;
		}
		bytesRecvNullCnt--;
	}

	// Disable TX data interrupt active if no more data needs to be sent
	if ((toSendBytes + bytesRecvNullCnt) == 0)
	{
		pI2CRegSet->i2c_ctrl &= ~(1 << 7);
	}

	// Update status
	sts = pI2CRegSet->i2c_stat;
	if ((sts & (1 << 5)) != 0)
	{
		// Still in active transfer, stop
		I2Cstatus = I2CST_ACTIVE;
	}
	else if ((sts & (1 << 2)) != 0)
	{
		// Last operation had a NAK
		I2Cstatus = I2CST_NAK;
		pI2CRegSet->i2c_ctrl = 0;
	}
	else if ((sts & (1 << 0)) != 0)
	{
		// Last operation had a NAK
		I2Cstatus = I2CST_COMPLETE;

		// Copy last data from FIFO if it exisst
		while ((pI2CRegSet->i2c_stat & (1 << 9)) == 0)
		{
			recvBuffer[bytesReceived] = (UINT8) pI2CRegSet->i2c_txrx;
			bytesReceived++;
		}

		pI2CRegSet->i2c_ctrl = 0;
	}
	else
	{
		// All other operations are failures
		I2Cstatus = I2CST_ERROR;
		pI2CRegSet->i2c_ctrl = 0;
	}

	return I2Cstatus;
}

//------------------------------------------------------------------------------
//
// bspi2cSetClock
//
// Sets or adjusts the I2C clock rate to the desired clock rate and either
// symmetrical or asymmetrical mode.
//
UINT32 bspI2C::bspi2cSetClock(UINT32 baseclk,
							  UINT32 clock,
					          BOOL asymClk)
{
	UINT32 chi, clo, div, newclk = 0;

	// Find best divider
	div = baseclk / clock;

	// Use asymetrical clocK?
	if (asymClk == TRUE)
	{
		// Weight low clock at about 64% duty cycle
		clo = (div * 640) / 1000;
		chi = div - clo;
	}
	else
	{
		// Clock is symmetrical
		chi = div >> 1;
		clo = div - chi;
	}

	// Verify clock settings
	if (chi < 1)
	{
		chi = 1;
	}
	if (clo < 1)
	{
		clo = 1;
	}

	// Update clock settings
	pI2CRegSet->i2c_clk_hi = chi;
	pI2CRegSet->i2c_clk_lo = clo;

	// Return actual clock rate
	newclk = baseclk / (chi + clo);

	return newclk;
}

//------------------------------------------------------------------------------
//
// bspi2cReset
//
// Reset the I2C device.
//
void bspI2C::bspi2cReset(void)
{
	UNS_32 to;

	// Clear all latched interrupts
	pI2CRegSet->i2c_stat = 0x3;

	/* Reset I2C */
	pI2CRegSet->i2c_ctrl = (1 << 8);
	to = 0xFFFF;
	while (((pI2CRegSet->i2c_ctrl & (1 << 8)) != 0) && (to > 0))
	{
		to--;
	}
}

//------------------------------------------------------------------------------
//
// bspi2cGetStatus
//
// Returns the status of the last I2C transfer.
//
I2C_ST_T bspI2C::bspi2cGetStatus(void)
{
	return I2Cstatus;
}

//------------------------------------------------------------------------------
//
// bspI2CGetData
//
// Get data from the last I2C read transfer, this function will return
// up to maxBytes of data until no more data is available. The function
// will return 0 when all data has been read.
//
int bspI2C::bspI2CGetData(UINT8 *buff,
				          int maxBytes)
{
	int bcopied = 0;

	while ((maxBytes > 0) && (bytesRecvGetIndex < bytesReceived))
	{
		*buff = recvBuffer[bytesRecvGetIndex];
		maxBytes--;
		bytesRecvGetIndex++;
		bcopied++;
		buff++;
	}

	return bcopied;
}

//------------------------------------------------------------------------------
//
// bspi2cStartXfer
//
// Start an I2C master transfer. All I2C transfers (write and read) start
// with this function.
//
BOOL bspI2C::bspi2cStartXfer(UINT16 *flags_buff,   // Buffer with start/stop flags
				             int nBytes,           // Number of send bytes in flags_buff
					         int recBytes)         // Number of bytes to receive
{
	UINT32 tmp;
	int idx, bsend = 0;
	BOOL started = FALSE;

	I2Cstatus = I2CST_ERROR;
	if ((nBytes <= MAXI2CBUFFSIZE) && (recBytes <= MAXI2CBUFFSIZE))
	{
		/* Reset I2C transfer */
		bspi2cReset();

		// Set up send side of I2C transfer
		toSendBytes = nBytes;
		bytesSendCopyIndex = 0;

		// Set up receive side of I2C transfer
		toRecvBytes = recBytes;
		bytesRecvNullCnt = recBytes;
		bytesReceived = 0;
		bytesRecvPutIndex = 0;
		bytesRecvGetIndex = 0;

		// Setup basic interrupts
		tmp =
			(0 << 9) | // 7-bit address
			(1 << 3) | // DRMI interrupt
			(1 << 2) | // NAK interrupt
			(1 << 1) | // Bus arbitration failure
			(1 << 0);  // Transfer completed (stop condition issued)

		// Is receive data expected?
		if (toRecvBytes > 0)
		{
			tmp |= (1 << 6); // Enable RX data available interrupt
		}

		pI2CRegSet->i2c_ctrl = tmp;

		// Place data into the FIFO with selected flags. Either place all
		// the data in the FIFO or fill it until it is full
		while ((bsend < toSendBytes) &&
			((pI2CRegSet->i2c_stat & (1 << 10)) == 0))
		{
			pI2CRegSet->i2c_txrx = (UINT32) *flags_buff;
			flags_buff++;
			bsend++;
		}
		toSendBytes -= bsend;

		// If more data needs to be sent, save it here
		if (toSendBytes > 0)
		{
			// Save rest of data to send later
			for (idx = 0; idx < toSendBytes; idx++)
			{
				sendBuffer[idx] = *flags_buff;
				flags_buff++;
			}
		}

		// To receive data, the transmitter needs to send all 1's so the
		// I2C clock keeps running. If needed, this will put that data into
		// the TX FIFO so the RX data can be clocked in
		while ((bytesRecvNullCnt > 0) &&
			((pI2CRegSet->i2c_stat & (1 << 10)) == 0))
		{
			if (bytesRecvNullCnt == 1)
			{
				pI2CRegSet->i2c_txrx = (UINT32) (0xFF | I2C_FLAG_STOP);
			}
			else
			{
				pI2CRegSet->i2c_txrx = (UINT32) 0xFF;
			}
			bytesRecvNullCnt--;
		}

		// If more TX data needs to be handled later, then enable the TX
		// interrupt
		if ((toSendBytes + bytesRecvNullCnt) > 0)
		{
			pI2CRegSet->i2c_ctrl = (tmp | (1 << 7)); // TX FIFO not full interrupt
		}

		started = TRUE;
		I2Cstatus = I2CST_ACTIVE;
	}

	return started;
}
