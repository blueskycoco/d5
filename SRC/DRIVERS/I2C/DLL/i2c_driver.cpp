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
// i2c_driver.cpp
//
// General I2C DLL
//

#include <windows.h>
#include <nkintr.h>
#include <CEDDK.h>
#include "bsp.h"
#include "bsp_i2c.h"
#include "drv_ioctl_funcs.h"
#include "lpc32xx_i2c.h"
#include "lpc32xx_clkpwr.h"

//------------------------------------------------------------------------------
// I2C allocation for the Phytec LPC3250 board
// I2C1 (3.0v)
//   I2S codec
//   LCD (not connected)
//   RTC
// I2C2 (1.8v)
//   Not used
//------------------------------------------------------------------------------

// Device context instance
typedef struct
{
	I2C_REGS_T *pI2CRegs;
	CLKPWR_REGS_T *pClkPwrRegs; // Poiner to CLKPWR regs
	DWORD I2CsysIntr;           // SysIntr value for device
	HANDLE i2cEvent;            // Interrupt event handler
	HANDLE i2cLock;             // Mutex for I2C access lock
	int    i2cNum;              // 0 for I2C1, 1 for I2C2
	class bspI2C *pI2CClass;    // Allocated I2C control class
} I2C_DRVCTL_T;
static I2C_DRVCTL_T i2cDrv[3] = {
	{NULL, NULL, SYSINTR_UNDEFINED, NULL, NULL, 0, NULL},  // I2C1
	{NULL, NULL, SYSINTR_UNDEFINED, NULL, NULL, 2, NULL}}; // I2C2

// Number of open instances
static int i2cInstances[3] = {0, 0, 0};

//------------------------------------------------------------------------------
//
// I2C_Lock
//
// Lock access to I2C registers
//
static void I2C_Lock(HANDLE lckHndl) {
	WaitForSingleObject(lckHndl, 150);
}

//------------------------------------------------------------------------------
//
// I2C_Unlock
//
// Unlock access to I2C registers
//
static void I2C_Unlock(HANDLE lckHndl) {
	ReleaseMutex(lckHndl);
}

//------------------------------------------------------------------------------
//
// I2CPeriphDown
//
// Disables clocking and power state
//
static void I2CPeriphDown(DWORD hDeviceContext)
{
	volatile UINT32 tmp;
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hDeviceContext;

	// Disable I2C clock
	tmp = pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl;
	if (pDev->i2cNum == 0)
	{
		pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl = tmp &
			~CLKPWR_I2CCLK_I2C1CLK_EN;
	}
	else
	{
		pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl = tmp &
			~CLKPWR_I2CCLK_I2C2CLK_EN;
	}
}

//------------------------------------------------------------------------------
//
// I2CPeriphUp
//
// Enables peripheral clocking and power state
//
extern "C" void I2CPeriphUp(DWORD hDeviceContext)
{
	volatile UINT32 tmp;
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hDeviceContext;

	// Enable I2C clock and pullup mode
	tmp = pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl;
	if (pDev->i2cNum == 0)
	{
		pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl =
			tmp | CLKPWR_I2CCLK_I2C1CLK_EN | CLKPWR_I2CCLK_I2C2HI_DRIVE;
	}
	else
	{
		tmp = tmp & ~CLKPWR_I2CCLK_I2C2HI_DRIVE;
		pDev->pClkPwrRegs->clkpwr_i2c_clk_ctrl =
			tmp | CLKPWR_I2CCLK_I2C2CLK_EN;
	}
}

//------------------------------------------------------------------------------
//
// I2C_Transfer
//
// I2C transaction handler, handles both RX and TX for a specific channel
//
static BOOL I2C_Transfer(I2C_DRVCTL_T *pI2cDrv,
						 I2C_OUT_XFER_T *i2cout,
						 I2C_IN_XFER_T *i2cin)
{
	I2C_ST_T ist;
	DWORD rsts;
	BOOL xcomp, xg = FALSE;
	INT32 sendBytes = i2cout->tosend;

	I2C_Lock(pI2cDrv->i2cLock);

	// Clear sysIntr
	ResetEvent(pI2cDrv->i2cEvent);

	i2cin->recvBytes = 0;

	// Timeout failure?
	if (WaitForSingleObject(pI2cDrv->i2cEvent, 0) != WAIT_FAILED)
	{
		// Start transfer
		xcomp = FALSE;
		if (pI2cDrv->pI2CClass->bspi2cStartXfer(i2cout->flags_buff,
			i2cout->tosend, i2cout->torecv) == FALSE)
		{
			xcomp = TRUE;
		}

		// Loop until transfer complete
		while (xcomp == FALSE)
		{
			// Wait for interrupt
			rsts = WaitForSingleObject(pI2cDrv->i2cEvent, 250);
			if (rsts == WAIT_TIMEOUT)
			{
				xcomp = TRUE;
				while (1);
			}
			else if (rsts == WAIT_FAILED)
			{
				xcomp = TRUE;
				while (1);
			}
			else
			{
				// Handle interrupt
				ist = pI2cDrv->pI2CClass->bspi2cInt();

				// Exit if no longet active
				if (ist != I2CST_ACTIVE)
				{
					xcomp = TRUE;
				}
			}

			InterruptDone(pI2cDrv->I2CsysIntr);
		}

		// Check transfer status and copy RX bytes if needed
		i2cin->ist = pI2cDrv->pI2CClass->bspi2cGetStatus();
		if (i2cin->ist == I2CST_COMPLETE)
		{
			xg = TRUE;

			// Copy bytes
			i2cin->recvBytes =
				pI2cDrv->pI2CClass->bspI2CGetData(i2cin->buff, 32);
		}
	}

	I2C_Unlock(pI2cDrv->i2cLock);

	return xg;
}

//------------------------------------------------------------------------------
//
// IIC_PowerDown
//
// Stub function for powerdown
//
extern "C" void IIC_PowerDown(DWORD hDeviceContext)
{
	// Do nothing, system power management for I2C is handled in the kernel
	// power down or OTG functions, as other system drivers may require the
	// I2C DLL to correctly work for their power down functions.
}

//------------------------------------------------------------------------------
//
// IIC_PowerUp
//
// Stub function for powerup
//
extern "C" void IIC_PowerUp(DWORD hDeviceContext)
{
	// Do nothing, handled in kernel power up or OTG functions
}

//------------------------------------------------------------------------------
//
// IIC_Deinit
//
// I2C de-init function
//
extern "C" BOOL IIC_Deinit(DWORD hDeviceContext)
{
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hDeviceContext;

	if (pDev->pClkPwrRegs != NULL)
	{
		// Disable I2C clock
		I2CPeriphDown(hDeviceContext);

		MmUnmapIoSpace((PVOID) pDev->pClkPwrRegs,
			sizeof (CLKPWR_REGS_T));
		pDev->pClkPwrRegs = NULL;
	}
	if (pDev->pI2CRegs != NULL)
	{
		if (pDev->pI2CClass != NULL)
		{
			delete pDev->pI2CClass;
			pDev->pI2CClass = NULL;
		}

		MmUnmapIoSpace((PVOID) pDev->pI2CRegs,
			sizeof (I2C_REGS_T));
		pDev->pI2CRegs = NULL;
	}

	// Release sysIntr value
	if (pDev->I2CsysIntr != SYSINTR_UNDEFINED)
	{
        KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &pDev->I2CsysIntr, 
            sizeof(pDev->I2CsysIntr), NULL, 0, NULL);
		pDev->I2CsysIntr = SYSINTR_UNDEFINED;
	}

	// Delete event
	if (pDev->i2cEvent != NULL)
	{
		CloseHandle(pDev->i2cEvent);
		pDev->i2cEvent = NULL;
	}

	// Delete access mutex
	if (pDev->i2cLock != NULL)
	{
		CloseHandle(pDev->i2cLock);
		pDev->i2cLock = NULL;
	}

    return TRUE;
}

//------------------------------------------------------------------------------
//
// IIC_Init
//
// I2C Init function
//
extern "C" DWORD IIC_Init(LPCTSTR pContext,
						  LPCVOID lpvBusContext)
{
	PHYSICAL_ADDRESS pa;
	UINT32 bytesret, clk;
	int index;
	I2C_DRVCTL_T *pDev;
    HKEY hkI2C = NULL;
	DWORD dwStatus, dwType, dwSize, indx, irqt, sts = 0;
	WCHAR regkeyname[256];

    (void) lpvBusContext;

	// Open the registry key and read the key value
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, pContext, 0, 0, &hkI2C);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("IIC_Init: Error opening device registry!\r\n")));
		goto cleanup;
    }

	// Get key value
    dwSize = sizeof(regkeyname);
    dwType = REG_SZ;
    dwStatus = RegQueryValueEx(hkI2C, TEXT("Key"), NULL, &dwType, 
        (LPBYTE) regkeyname, &dwSize);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("IIC_Init: Error getting device key name\r\n")));
	    RegCloseKey(hkI2C);
		goto cleanup;
    }

	// Open the registry key and read the index value
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, regkeyname, 0, 0, &hkI2C);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("IIC_Init: Error opening device registry!\r\n")));
		goto cleanup;
    }

	// Get I2C index value
    dwSize = sizeof(indx);
    dwType = REG_DWORD;
    dwStatus = RegQueryValueEx(hkI2C, TEXT("Index"), NULL, &dwType, 
        (LPBYTE) &indx, &dwSize);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("IIC_Init: Error getting Index\r\n")));
    }
	index = (int) indx;

    RegCloseKey(hkI2C);
    DEBUGMSG(ZONE_INIT, (_T("IIC_Init: Using index %d\r\n"), index));

	// Get IRQ mapped to this I2C channel
	if (index == 0)
	{
		irqt = OAL_INTR_IRQ_I2C_1;
	}
	else
	{
		irqt = OAL_INTR_IRQ_I2C_2;
	}
	pDev = &i2cDrv [index];

	// Setup defaults
	pDev->pI2CRegs = NULL;
	pDev->I2CsysIntr = SYSINTR_UNDEFINED;
	pDev->pClkPwrRegs = NULL;
	pDev->i2cEvent = NULL;
	pDev->i2cLock = NULL;
	pDev->i2cNum = index;

	// Allocate registers for I2C and clock and power
	pa.HighPart = 0;
	pa.LowPart = CLK_PM_BASE;
	pDev->pClkPwrRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);

	pa.HighPart = 0;
	if (index == 0)
	{
		pa.LowPart = I2C1_BASE;
	}
	else
	{
		pa.LowPart = I2C2_BASE;
	}
	pDev->pI2CRegs = (I2C_REGS_T *) MmMapIoSpace(pa,
		sizeof (I2C_REGS_T), FALSE);

	if ((pDev->pClkPwrRegs == NULL) ||
		(pDev->pI2CRegs == NULL))
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: I2C: Failed to map registers\r\n")));
		goto cleanup;
	}

	// Enable I2C clocking and power for init
	I2CPeriphUp((DWORD) pDev);

	// Allocate I2C and initialize I2C interface
	pDev->pI2CClass = new bspI2C(pDev->pI2CRegs);
	if (pDev->pI2CClass == NULL)
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: I2C: Failed to allocate I2C driver class\r\n")));
		goto cleanup;
	}

	// Get sysintr value
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irqt, sizeof(irqt),
		&pDev->I2CsysIntr, sizeof(pDev->I2CsysIntr),
		NULL))
    {
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: I2C: Failed to request the I2C sysintr.\r\n")));

        pDev->I2CsysIntr = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	// Create event handle
	pDev->i2cEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (pDev->i2cEvent == NULL)
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: I2C: Failed to create handler event.\r\n")));
		goto cleanup;
	}

	// Initialize the I2C interrupt
	if (InterruptInitialize(pDev->I2CsysIntr, pDev->i2cEvent,
		NULL, 0) == FALSE) {
		// Cannot initialize interrupt
        RETAILMSG(ZONE_ERROR, 
			(TEXT("ERROR: I2C: Cannot initialize I2C interrupt\r\n")));
        goto cleanup;
	}
	InterruptDone(pDev->I2CsysIntr);

	// Create access mutex for I2C registers
	pDev->i2cLock = CreateMutex(NULL, FALSE, NULL);
	if (pDev->i2cLock == NULL) {
        RETAILMSG(ZONE_ERROR,
			(TEXT("ERROR: I2C: Error creating control mutex\r\n")));
		goto cleanup;
	}

	// Get base clock for I2C
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
		sizeof (clk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock
        RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: I2C: Error getting I2C base clock rate.\r\n")));
		goto cleanup;
	}

	pDev->pI2CClass->bspi2cSetClock(clk, 400000, FALSE);
	sts = (DWORD) pDev;

	// Disable I2C clock until it is needed
	I2CPeriphDown((DWORD) pDev);

cleanup:
	return sts;
}

//------------------------------------------------------------------------------
//
// IIC_IOControl
//
// I2C driver general IOCTL handler function
//
extern "C" BOOL IIC_IOControl(DWORD hOpenContext, DWORD dwCode, PBYTE pBufIn,
                   DWORD dwLenIn, PBYTE pBufOut, DWORD dwLenOut,
                   PDWORD pdwActualOut) {
	UINT32 bytesret, clk;
	I2C_XFER_SETUP_T *pi2cSetup;
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hOpenContext;

	switch (dwCode)
	{
	case IOCTL_APP_I2CREQ:
        // Attempt transfer
		if ((pBufIn != NULL) && (pBufOut != NULL) &&
			(dwLenIn == sizeof(I2C_OUT_XFER_T)) &&
			(dwLenOut == sizeof(I2C_IN_XFER_T)))
		{
			*pdwActualOut = sizeof(I2C_IN_XFER_T);
            return I2C_Transfer(pDev, (I2C_OUT_XFER_T *) pBufIn,
				(I2C_IN_XFER_T *) pBufOut);
		}
		break;

	case IOCTL_APP_I2CSETUP:
        // Attempt setup
		if ((pBufIn != NULL) && (dwLenIn == sizeof(I2C_XFER_SETUP_T)))
		{
			// Get base clock for I2C
			if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
				sizeof (clk), (LPDWORD) &bytesret) != FALSE)
			{
				pi2cSetup = (I2C_XFER_SETUP_T *) pBufIn;
				I2C_Lock(pDev->i2cLock);
				pDev->pI2CClass->bspi2cSetClock(clk, pi2cSetup->clock,
					pi2cSetup->assym);
				I2C_Unlock(pDev->i2cLock);
				return TRUE;
			}
		}
		break;

	default:
		break;
	}

    return FALSE;
}

//------------------------------------------------------------------------------
//
// IIC_Close
//
// I2C driver close function
//
extern "C" BOOL IIC_Close(DWORD hOpenContext) {
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hOpenContext;

	i2cInstances[pDev->i2cNum]--;
	if (i2cInstances[pDev->i2cNum] == 0)
	{
		// Disable clocks, no more I2C channels open
		I2CPeriphDown(hOpenContext);
	}

	return TRUE;
}

//------------------------------------------------------------------------------
//
// IIC_Open
//
// I2C driver open function
//
extern "C" DWORD IIC_Open(DWORD hDeviceContext, DWORD AccessCode,
               DWORD ShareMode) {
	I2C_DRVCTL_T *pDev = (I2C_DRVCTL_T *) hDeviceContext;

    (void) AccessCode;
    (void) ShareMode;

	i2cInstances[pDev->i2cNum]++;
	if (i2cInstances[pDev->i2cNum] == 1)
	{
		// Enable clocks
		I2CPeriphUp(hDeviceContext);
	}

	// Return the device context, this will be used as the open
	// context to identify which I2C channel is used for operations
    return hDeviceContext;
}

//------------------------------------------------------------------------------
//
// IIC_Read
//
// I2C driver read stub function
//
extern "C" DWORD IIC_Read(DWORD hOpenContext, LPVOID pBuffer, DWORD Count) {
    (void) hOpenContext;
    (void) pBuffer;
    (void) Count;

    /* Read not allowed */
    return 0;
}

//------------------------------------------------------------------------------
//
// IIC_Write
//
// I2C driver write stub function
//
extern "C" DWORD IIC_Write(DWORD hOpenContext, LPCVOID pBuffer, DWORD Count) {
    (void) hOpenContext;
    (void) pBuffer;
    (void) Count;

    /* Write not allowed */
    return 0;
}

//------------------------------------------------------------------------------
//
// IIC_Seek
//
// I2C driver seek stub function
//
extern "C" DWORD IIC_Seek(DWORD hOpenContext, long Amount, WORD Type) {
    (void) hOpenContext;
    (void) Amount;
    (void) Type;

    /* Seek not allowed */
    return 0;
}


//------------------------------------------------------------------------------
//
// IIC_DLLEntry
//
// DLL entry point
//
extern "C" BOOL __stdcall IIC_DLLEntry (
                             HANDLE  hinstDLL,      // [in] : Instance pointer
                             DWORD   Op,                // [in] : Reason routine is called
                             LPVOID  lpvReserved        // [in] : system parameter
                             )
{
    switch(Op) {
    case DLL_PROCESS_ATTACH :
        DisableThreadLibraryCalls((HMODULE) hinstDLL);
        break;
        
    case DLL_PROCESS_DETACH :
        break;
        
    case DLL_THREAD_DETACH :
    case DLL_THREAD_ATTACH :
        break;
        
    default :
        break;
    }
    return TRUE;
}
