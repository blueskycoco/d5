//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this sample source code is subject to the terms of the Microsoft
// license agreement under which you licensed this sample source code. If
// you did not accept the terms of the license agreement, you are not
// authorized to use this sample source code. For the terms of the license,
// please see the license agreement between you and Microsoft or, if applicable,
// see the LICENSE.RTF on your install media or the root of your tools installation.
// THE SAMPLE SOURCE CODE IS PROVIDED "AS IS", WITH NO WARRANTIES.
//

#include "lpc32xx_otg_i2c.h"
#include "bsp.h"

// Static driver data
int LPC32XX_OTGI2C::m_instances = 0;
DWORD LPC32XX_OTGI2C::m_goodinit = 0;
OTGI2C_REGS_T *LPC32XX_OTGI2C::m_pI2CRegs = NULL;
DWORD LPC32XX_OTGI2C::m_I2CsysIntr = SYSINTR_UNDEFINED;
HANDLE LPC32XX_OTGI2C::m_i2cEvent = NULL;
HANDLE LPC32XX_OTGI2C::m_i2cLock = NULL;
class bspI2C *LPC32XX_OTGI2C::m_pI2CClass = NULL;

//********************************************************************
// Initialization and de-init functions
//********************************************************************

// Constructor
LPC32XX_OTGI2C::LPC32XX_OTGI2C(void)
{

	m_instances++;
	if (m_instances == 1)
	{	
		m_goodinit = 0;
	}
}

// Destructor
LPC32XX_OTGI2C::~LPC32XX_OTGI2C(void)
{
	m_instances--;
	if (m_instances == 0)
	{
		Deinit();
		m_goodinit = 0;
	}
}

// Initializes interface
DWORD LPC32XX_OTGI2C::Init(void)
{
	PHYSICAL_ADDRESS pa;
	DWORD irqt, sts = 0;
	UINT32 bytesret, clk;

	if (m_goodinit == 0)
	{
		// Setup defaults
		irqt = OAL_INTR_IRQ_OTG_I2C;
		m_pI2CRegs = NULL;
		m_I2CsysIntr = SYSINTR_UNDEFINED;
		m_i2cEvent = NULL;
		m_i2cLock = NULL;

		// Allocate registers for I2C
		pa.QuadPart = OTG_I2C_BASE;
		m_pI2CRegs = (OTGI2C_REGS_T *) MmMapIoSpace(pa,
			sizeof (OTGI2C_REGS_T), FALSE);
		if (m_pI2CRegs == NULL)
		{
		    RETAILMSG(ZONE_ERROR, 
			    (TEXT("ERROR: OTGI2C: Failed to map registers\r\n")));
			goto cleanup;
		}

		// Allocate I2C and initialize I2C interface
		m_pI2CClass = new bspI2C((I2C_REGS_T *) m_pI2CRegs);
		if (m_pI2CClass == NULL)
		{
		    RETAILMSG(ZONE_ERROR, 
			    (TEXT("ERROR: OTGI2C: Failed to allocate I2C driver class\r\n")));
			goto cleanup;
		}

		// Get sysintr value
		if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irqt, sizeof(irqt),
			&m_I2CsysIntr, sizeof(m_I2CsysIntr), NULL))
	    {
		    RETAILMSG(ZONE_ERROR, 
			    (TEXT("ERROR: OTGI2C: Failed to request the I2C sysintr.\r\n")));
	        m_I2CsysIntr = SYSINTR_UNDEFINED;
		    goto cleanup;
	    }

		// Create event handle
		m_i2cEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
		if (m_i2cEvent  == NULL)
		{
			RETAILMSG(ZONE_ERROR, 
				(TEXT("ERROR: OTGI2C: Failed to create handler event.\r\n")));
			goto cleanup;
		}

		// Initialize the I2C interrupt
		if (InterruptInitialize(m_I2CsysIntr, m_i2cEvent,
			NULL, 0) == FALSE) {
			// Cannot initialize interrupt
		    RETAILMSG(ZONE_ERROR, 
				(TEXT("ERROR: OTGI2C: Cannot initialize I2C interrupt\r\n")));
	        goto cleanup;
		}
		RETAILMSG(1, (TEXT("interrupt done\r\n")));
		InterruptDone(m_I2CsysIntr);

		// Create access mutex for I2C registers
		m_i2cLock = CreateMutex(NULL, FALSE, NULL);
		if (m_i2cLock == NULL) {
		    RETAILMSG(ZONE_ERROR,
				(TEXT("ERROR: OTGI2C: Error creating control mutex\r\n")));
			goto cleanup;
		}

		// Get base clock for I2C
		if (KernelIoControl(IOCTL_LPC32XX_GETPCLK, NULL, 0, &clk,
			sizeof (clk), (LPDWORD) &bytesret) == FALSE)
		{
			// Cannot get clock
	        RETAILMSG(ZONE_ERROR, 
		        (TEXT("ERROR: OTGI2C: Error getting I2C base clock rate.\r\n")));
			goto cleanup;
		}

		m_pI2CClass->bspi2cSetClock(clk, 100000, FALSE);
		m_goodinit = 1;
	}
	RETAILMSG(1, (TEXT("otg i2c init is good\r\n")));

cleanup:

	if (m_goodinit == 0)
	{
		Deinit();
	}
	return m_goodinit;
}

// Re-initializes interface
//void LPC32XX_OTGI2C::Reinit(void)
//{ // tbd
	// Nothing to do...yet
//}

// De-initialize driver
void LPC32XX_OTGI2C::Deinit(void)
{
	RETAILMSG(1, (TEXT("LPC32XX_OTGI2C::Deinit\r\n")));
	if (m_i2cEvent != NULL)
	{
		CloseHandle(m_i2cEvent);
		m_i2cEvent = NULL;
	}
	if (m_I2CsysIntr != SYSINTR_UNDEFINED)
	{
		KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &m_I2CsysIntr,
			sizeof(m_I2CsysIntr), NULL, 0, NULL);
		m_I2CsysIntr = SYSINTR_UNDEFINED;
	}
	if (m_pI2CClass != NULL)
	{
		delete m_pI2CClass;
		m_pI2CClass = NULL;
	}
	if (m_pI2CRegs != NULL)
	{
		MmUnmapIoSpace(m_pI2CRegs, sizeof(OTGI2C_REGS_T));
		m_pI2CRegs = NULL;
	}
}

//********************************************************************
// I2C data read and write functions
//********************************************************************

// Perform a read of the OTG device via I2C
DWORD LPC32XX_OTGI2C::ReadReg(UINT8 reg,
							  UINT8 *buff,
							  int bytes)
{
	DWORD good;
	I2C_OUT_XFER_T xferout;
	I2C_IN_XFER_T xferin;

	// Setup transfer
	xferout.flags_buff [0] = (OTG_DEV_ADDR | I2C_WRITE | I2C_START_FLAG);
	xferout.flags_buff [1] = reg;

	// Start read operation with repeated start
	xferout.flags_buff [2] = (OTG_DEV_ADDR | I2C_READ | I2C_START_FLAG);

	// Read bytes, send 3 bytes
	xferout.tosend = 3;
	xferout.torecv = bytes;

	good = I2CTransaction(&xferout, &xferin);
	if (good != 0)
	{
		if (xferin.ist != I2CST_COMPLETE)
		{
			good = 0;
		}
		else
		{
			memcpy(buff, xferin.buff, bytes);
		}
	}

	if (bytes == 1)
	{
		DEBUGMSG(1, (L"LPC32XX_OTGI2C::ReadReg 0x%x = 0x%x\r\n", reg, *buff));
	}
	else
	{
		UINT16 tmp = *buff + (*(buff + 1) << 8);
		DEBUGMSG(1, (L"LPC32XX_OTGI2C::ReadReg 0x%x = 0x%x\r\n", reg, tmp));
	}

	return good;
}

// Perform a write on the OTG device via I2C
DWORD LPC32XX_OTGI2C::WriteReg(UINT8 reg,
							   UINT8 *buff,
							   int bytes)
{
	DWORD good;
	I2C_OUT_XFER_T xferout;
	I2C_IN_XFER_T xferin;
	int nidx;

DEBUGMSG(1, (L"LPC32XX_OTGI2C::WriteReg 0x%x = 0x%x (%d)\r\n", reg, *buff, bytes));

	// Setup transfer
	xferout.flags_buff [0] = (OTG_DEV_ADDR | I2C_WRITE | I2C_START_FLAG);
	xferout.flags_buff [1] = reg;

	// Send bytes, read 0 bytes
	xferout.tosend = 2 + bytes;
	xferout.torecv = 0;

	// Start read operation with repeated start
	nidx = 2;
	while (bytes > 1)
	{
		xferout.flags_buff [nidx] = buff[nidx - 2];
		nidx++;
		bytes--;
	}
	xferout.flags_buff [nidx] = (buff[nidx - 2] | I2C_STOP_FLAG);

	good = I2CTransaction(&xferout, &xferin);
	if (good != 0)
	{
		if (xferin.ist != I2CST_COMPLETE)
		{
			good = 0;
		}
	}

	return good;
}

//********************************************************************
// I2C data read and write function
//********************************************************************

// Perform a transaction on the OTG device via I2C
DWORD LPC32XX_OTGI2C::I2CTransaction(I2C_OUT_XFER_T *pI2COut,
									 I2C_IN_XFER_T *pI2CIn)
{
	I2C_ST_T ist;
	DWORD rsts, xg = 0,count = 0;
	BOOL xcomp;
	INT32 sendBytes = pI2COut->tosend;

	I2C_Lock();

	// Clear sysIntr
	ResetEvent(m_i2cEvent);

	pI2CIn->recvBytes = 0;

	// Timeout failure?
	if (WaitForSingleObject(m_i2cEvent, 0) != WAIT_FAILED)
	{
		// Start transfer
		xcomp = FALSE;
		if (m_pI2CClass->bspi2cStartXfer(pI2COut->flags_buff,
			pI2COut->tosend, pI2COut->torecv) == FALSE)
		{
			xcomp = TRUE;
		}

		// Loop until transfer complete
		while (xcomp == FALSE)
		{
			// Wait for interrupt
			rsts = WaitForSingleObject(m_i2cEvent, 250);
			if ((rsts == WAIT_TIMEOUT) || (rsts == WAIT_FAILED))
			{
				xcomp = TRUE;
			}
			else
			{
				// Handle interrupt
//				RETAILMSG(1, (TEXT("I2C interrupt handle\r\n")));//don't delete it!!
				ist = m_pI2CClass->bspi2cInt();
				// Exit if no longet active
				if (ist != I2CST_ACTIVE)
				{
					xcomp = TRUE;
				}
			}

			InterruptDone(m_I2CsysIntr);
		}

		// Check transfer status and copy RX bytes if needed
		pI2CIn->ist = m_pI2CClass->bspi2cGetStatus();
		if (pI2CIn->ist == I2CST_COMPLETE)
		{
			xg = TRUE;
			// Copy bytes
			pI2CIn->recvBytes =
				m_pI2CClass->bspI2CGetData(pI2CIn->buff, 32);
		}
	}

	I2C_Unlock();

	return xg;
}

//********************************************************************
// OTG I2C driver internal functions
//********************************************************************

//  Lock I2C access
void LPC32XX_OTGI2C::I2C_Lock(void)
{
	WaitForSingleObject(m_i2cLock, 250); //YJ
}

//  Unlock I2C accesss
void LPC32XX_OTGI2C::I2C_Unlock(void)
{
	ReleaseMutex(m_i2cLock);
}
