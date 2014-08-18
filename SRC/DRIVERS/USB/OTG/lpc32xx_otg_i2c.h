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

#pragma once

#include <windows.h>
#include "lpc32xx_otg.h"
#include "phy3250_board.h"
#include "drv_ioctl_funcs.h"
#include "bsp_i2c.h"

// I2C device filename that contains the interface to I2S
#define OTGDEVNAME (L"IIC2:")

// I2C address for  device
#define OTG_DEV_ADDR (PHY_I2C_OTG<<1)

class LPC32XX_OTGI2C {
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	LPC32XX_OTGI2C(void);

	// Destructor
	~LPC32XX_OTGI2C();

	// Initializes interface
	DWORD Init(void);

	// Re-initializes interface
//	void Reinit(void);

	// De-initialize driver
	void Deinit(void);

	//********************************************************************
	// I2C data read and write functions
	//********************************************************************

	// Perform a read of the OTG device via I2C
	DWORD ReadReg(UINT8 reg,
		          UINT8 *buff,
				  int bytes);

	// Perform a write on the OTG device via I2C
	DWORD WriteReg(UINT8 reg,
		           UINT8 *buff,
				   int bytes);

private:
	// Number of instances
	static int m_instances;

	// Good initialization flag
	static DWORD m_goodinit;

	// I2C control values
	static OTGI2C_REGS_T *m_pI2CRegs;
	static DWORD m_I2CsysIntr;           // SysIntr value for device
	static HANDLE m_i2cEvent;            // Interrupt event handler
	static HANDLE m_i2cLock;             // Mutex for I2C access lock
	static class bspI2C *m_pI2CClass;    // Allocated I2C control class

	//********************************************************************
	// I2C data read and write function
	//********************************************************************

	// Perform a transaction on the OTG device via I2C
	DWORD I2CTransaction(I2C_OUT_XFER_T *pI2COut,
		                 I2C_IN_XFER_T *pI2CIn);

	//********************************************************************
	// OTG I2C driver internal functions
	//********************************************************************
	//  Lock I2C access
	void I2C_Lock(void);

	//  Unlock I2C accesss
	void I2C_Unlock(void);
};
