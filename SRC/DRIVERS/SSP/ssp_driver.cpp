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
#include <nkintr.h>
#include <CEDDK.h>
#include "bsp.h"
#include "bsp_ssp.h"
#include "drv_ioctl_funcs.h"
#include "lpc32xx_ssp.h"
#include "lpc32xx_clkpwr.h"
#include "lpc32xx_gpio.h"

// Device context instance
typedef struct
{
	SSP_REGS_T *pSSPRegs;
	CLKPWR_REGS_T *pClkPwrRegs; // Pointer to CLKPWR regs
	GPIO_REGS_T *pGPIORegs;     // Pointer to GPIO regs
	DWORD SSPsysIntr;           // SysIntr value for device
	HANDLE sspEvent;            // Interrupt event handler
	HANDLE sspLock;             // Mutex for SSP access lock
	int    sspNum;              // 0 for SSP0, 1 for SSP1
	class bspssp *pSSPClass;    // Allocated SSP control class
} SSP_DRVCTL_T;
static SSP_DRVCTL_T sspDrv[2] = {
	{NULL, NULL, NULL, SYSINTR_UNDEFINED, NULL, NULL, 0, NULL},
	{NULL, NULL, NULL, SYSINTR_UNDEFINED, NULL, NULL, 1, NULL}};

// Number of open instances
static int sspInstances[2] = {0, 0};

//------------------------------------------------------------------------------
//
// dump_regs
//
// Dump current SSP registers, debug function
//
void dump_regs(SSP_REGS_T *pRegs)
{
	UINT32 *p32 = (UINT32 *) pRegs;

	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;
	RETAILMSG(1, (_T("SSP Reg 0x%x = 0x%08x!\r\n"), p32, *p32));
	p32++;

}

//------------------------------------------------------------------------------
//
// SSP_Lock
//
// Lock access to SSP registers
//
static void SSP_Lock(HANDLE lckHndl) {
	WaitForSingleObject(lckHndl, 150);
}

//------------------------------------------------------------------------------
//
// SSP_Unlock
//
// Unlock access to SSP registers
//
static void SSP_Unlock(HANDLE lckHndl) {
	ReleaseMutex(lckHndl);
}

//------------------------------------------------------------------------------
//
// SSPPeriphDown
//
// Disables clocking and power state
//
static void SSPPeriphDown(DWORD hDeviceContext)
{
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hDeviceContext;

	// Disable clock for this device
	if (pDev->sspNum == 0)
	{
		pDev->pClkPwrRegs->clkpwr_ssp_blk_ctrl &= ~CLKPWR_SSPCTRL_SSPCLK0_EN;
	}
	else
	{
		pDev->pClkPwrRegs->clkpwr_ssp_blk_ctrl &= ~CLKPWR_SSPCTRL_SSPCLK1_EN;
	}
}

//------------------------------------------------------------------------------
//
// SSPPeriphUp
//
// Enables peripheral clocking and power state
//
extern "C" void SSPPeriphUp(DWORD hDeviceContext)
{
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hDeviceContext;

	// Enable clock for this device
	if (pDev->sspNum == 0)
	{
		pDev->pClkPwrRegs->clkpwr_ssp_blk_ctrl |= CLKPWR_SSPCTRL_SSPCLK0_EN;
	}
	else
	{
		pDev->pClkPwrRegs->clkpwr_ssp_blk_ctrl |= CLKPWR_SSPCTRL_SSPCLK1_EN;
	}
}

//------------------------------------------------------------------------------
//
// SSP_CS_Setup
//
// Sets the control function for the SSP chip select
//
void SSP_CS_Setup(SSP_DRVCTL_T *pDev,
				  BOOL lockedCS)
{
	if (pDev->sspNum == 0)
	{
		if (lockedCS == TRUE)
		{
			// GPIO controls SSP0 CS
			pDev->pGPIORegs->pio_mux_clr = PIO1_GPIO05_SSEL0;
			pDev->pGPIORegs->pio_dir_set = OUTP_STATE_GPIO(5);
		}
		else
		{
			// SSP0 controls SSP0 CS
			pDev->pGPIORegs->pio_mux_set = PIO1_GPIO05_SSEL0;
			pDev->pGPIORegs->pio_dir_clr = OUTP_STATE_GPIO(5);
		}
	}
	else
	{
		// Not supported on Phytec board
		// TBD Customer will need to implement for their own board
	}
}

//------------------------------------------------------------------------------
//
// SSP_Transfer
//
// SSP transaction handler, handles both RX and TX for a specific channel
//
static BOOL SSP_Transfer(SSP_DRVCTL_T *pDev,
						 SSP_SEND_XFER_T *sspout,
						 SSP_RECV_XFER_T *sspin)
{
	SSP_XFER_T sspxfer;
	BOOL waitLoop;
	DWORD rsts;
	DWORD inBuff, outBuff;
	int toRecv, toSend;

	// Drive the chip select low (if GPIO is used for CS)
	if (pDev->sspNum == 0)
	{
		pDev->pGPIORegs->pio_outp_clr = OUTP_STATE_GPIO(5);
	}
	else
	{
		// Not supported on Phytec board
		// TBD Customer will need to implement for their own board
	}

	// Save buffer pointers and transfer sizes
	inBuff = (DWORD) sspin->recvBuff;
	outBuff = (DWORD) sspout->sendBuff;
	toRecv = sspin->recvBuffSize;
	toSend = sspout->sendBuffBytes;

	// Wait for an interrupt until all data has been transferred
	waitLoop = TRUE;
	while (waitLoop == TRUE)
	{
		// Setup the transfer
		sspxfer.recvBuff = (void *) inBuff;
		sspxfer.recvBuffSize = toRecv;
		sspxfer.sendBuff = (void *) outBuff;
		sspxfer.sendBuffBytes = toSend;

		// Start the transfer
		pDev->pSSPClass->bspsspInt(&sspxfer);

		// Update counts from actual values
		inBuff += (DWORD) sspxfer.recvBuffBytesFilled;
		toRecv -= sspxfer.recvBuffBytesFilled;
		outBuff += (DWORD) sspxfer.actSendBuffBytes;
		toSend -= sspxfer.actSendBuffBytes;

		// Transfer complete
		if ((toSend == 0) && (toRecv == 0) || (sspxfer.recvDataOflow == TRUE))
		{
			sspin->recvOflow = sspxfer.recvDataOflow;
			waitLoop = FALSE;
		}
		else
		{
			// Wait for another interrupt to contimue handling
			InterruptDone(pDev->SSPsysIntr);
			rsts = WaitForSingleObject(pDev->sspEvent, 1000);
			if ((rsts == WAIT_TIMEOUT) || (rsts == WAIT_FAILED))
			{
				waitLoop = FALSE;
				sspin->recvOflow = TRUE;
			}
		}
	}

	// Drive the chip select high (if GPIO is used for CS)
	if (pDev->sspNum == 0)
	{
		pDev->pGPIORegs->pio_outp_set = OUTP_STATE_GPIO(5);
	}
	else
	{
		// Not supported on Phytec board
		// TBD Customer will need to implement for their own board
	}

	return TRUE;
}

//------------------------------------------------------------------------------
//
// SSP_PowerDown
//
// Powerdown function
//
extern "C" void SSP_PowerDown(DWORD hDeviceContext)
{
	SSPPeriphDown(hDeviceContext);
}

//------------------------------------------------------------------------------
//
// SSP_PowerUp
//
// Powerup function
//
extern "C" void SSP_PowerUp(DWORD hDeviceContext)
{
	SSPPeriphUp(hDeviceContext);
}

//------------------------------------------------------------------------------
//
// SSP_Deinit
//
// SSP de-init function
//
extern "C" BOOL SSP_Deinit(DWORD hDeviceContext) {
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hDeviceContext;

	// Delete SSP BSP class
	if (pDev->pSSPClass != NULL)
	{
		delete pDev->pSSPClass;
		pDev->pSSPClass = NULL;
	}

	if (pDev->pClkPwrRegs != NULL)
	{
		// Disable SSP clock
		SSPPeriphDown(hDeviceContext);

		MmUnmapIoSpace((PVOID) pDev->pClkPwrRegs,
			sizeof (CLKPWR_REGS_T));
		pDev->pClkPwrRegs = NULL;
	}
	if (pDev->pSSPRegs != NULL)
	{
		MmUnmapIoSpace((PVOID) pDev->pSSPRegs,
			sizeof (SSP_REGS_T));
		pDev->pSSPRegs = NULL;
	}
	if (pDev->pGPIORegs != NULL)
	{
		MmUnmapIoSpace((PVOID) pDev->pGPIORegs,
			sizeof (GPIO_REGS_T));
		pDev->pGPIORegs = NULL;
	}

	// Release sysIntr value
	if (pDev->SSPsysIntr != SYSINTR_UNDEFINED)
	{
        KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &pDev->SSPsysIntr, 
            sizeof(pDev->SSPsysIntr), NULL, 0, NULL);
		pDev->SSPsysIntr = SYSINTR_UNDEFINED;
	}

	// Delete event
	if (pDev->sspEvent != NULL)
	{
		CloseHandle(pDev->sspEvent);
		pDev->sspEvent = NULL;
	}

	// Delete access mutex
	if (pDev->sspLock != NULL)
	{
		CloseHandle(pDev->sspLock);
		pDev->sspLock = NULL;
	}

    return TRUE;
}

//------------------------------------------------------------------------------
//
// SSP_Init
//
// SSP Init function
//
extern "C" DWORD SSP_Init(LPCTSTR pContext, LPCVOID lpvBusContext) {
	PHYSICAL_ADDRESS pa;
	UINT32 bytesret, clk;
	int index;
	SSP_DRVCTL_T *pDev;
    HKEY hkey = NULL;
	DWORD dwStatus, dwType, dwSize, indx, irqt, sts = 0;
	WCHAR regkeyname[256];
	SSP_XFER_SETUP_T sspcfg;

    (void) lpvBusContext;

	DEBUGMSG(ZONE_INIT,
		(_T("SSP_IOCTL ID IOCTL_APP_SSPREQ = %x\r\n"), IOCTL_APP_SSPREQ));
	DEBUGMSG(ZONE_INIT,
		(_T("SSP_IOCTL ID IOCTL_APP_SSPSETUP = %x\r\n"), IOCTL_APP_SSPSETUP));

	// Open the registry key and read the key value
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, pContext, 0, 0, &hkey);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("SSP_Init: Error opening device registry!\r\n")));
		goto cleanup;
    }

	// Get key value
    dwSize = sizeof(regkeyname);
    dwType = REG_SZ;
    dwStatus = RegQueryValueEx(hkey, TEXT("Key"), NULL, &dwType, 
        (LPBYTE) regkeyname, &dwSize);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("SSP_Init: Error getting device key name\r\n")));
	    RegCloseKey(hkey);
		goto cleanup;
    }

	// Open the registry key and read the index value
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, regkeyname, 0, 0, &hkey);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("SSP_Init: Error opening device registry!\r\n")));
		goto cleanup;
    }

	// Get SSP index value
    dwSize = sizeof(indx);
    dwType = REG_DWORD;
    dwStatus = RegQueryValueEx(hkey, TEXT("Index"), NULL, &dwType, 
        (LPBYTE) &indx, &dwSize);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(ZONE_ERROR, (_T("SSP_Init: Error getting Index\r\n")));
    }
	index = (int) indx;

    RegCloseKey(hkey);
    DEBUGMSG(ZONE_INIT, (_T("SSP_Init: Using index %d\r\n"), index));

	// Get IRQ mapped to this SSP channel
	if (index == 0)
	{
		irqt = OAL_INTR_IRQ_SSP0;
	}
	else
	{
		irqt = OAL_INTR_IRQ_SSP1;
	}
	pDev = &sspDrv [index];

	// Setup defaults
	pDev->pSSPRegs = NULL;
	pDev->pClkPwrRegs = NULL;
	pDev->pGPIORegs = NULL;
	pDev->SSPsysIntr = SYSINTR_UNDEFINED;
	pDev->sspEvent = NULL;
	pDev->sspLock = NULL;
	pDev->sspNum = index;

	// Allocate registers for SSP, clock and power, and GPIO
	pa.QuadPart = CLK_PM_BASE;
	pDev->pClkPwrRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);

	// SSP 0 or 1
	if (index == 0)
	{
		pa.QuadPart = SSP0_BASE;
	}
	else
	{
		pa.QuadPart = SSP1_BASE;
	}
	pDev->pSSPRegs = (SSP_REGS_T *) MmMapIoSpace(pa,
		sizeof (SSP_REGS_T), FALSE);

	// GPIO
	pa.QuadPart = GPIO_BASE;
	pDev->pGPIORegs = (GPIO_REGS_T *) MmMapIoSpace(pa,
		sizeof (GPIO_REGS_T), FALSE);

	if ((pDev->pClkPwrRegs == NULL) || (pDev->pGPIORegs == NULL) ||
		(pDev->pSSPRegs == NULL))
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("SSP_Init: Failed to map registers\r\n")));
		goto cleanup;
	}

	// Setup muxing for this peripheral
	if (index == 0)
	{
		// Setup GPIO as an output for SSP0 CS
		pDev->pGPIORegs->pio_mux_clr = PIO1_GPIO05_SSEL0;
		pDev->pGPIORegs->pio_dir_set = OUTP_STATE_GPIO(5);
		pDev->pGPIORegs->pio_outp_set = OUTP_STATE_GPIO(5);

		// The MISO, MOSI, and SCK signals are controlled by SSP0
		pDev->pGPIORegs->pio_mux2_set = (PIO2_SPI1DATAIO_SSP0_MOSI |
			PIO2_SPI1DATAIN_SSP0_MISO | PIO2_SPI1CLK_SCK0);
	}
	else
	{
		// Not supported on Phytec board
		// TBD Customer will need to implement for their own board
	}

	// Enable SSP clocking and power for init
	SSPPeriphUp((DWORD) pDev);

	// Get base clock for SSP
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
		sizeof (clk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock
        RETAILMSG(ZONE_ERROR, 
            (TEXT("SSP_Init: Error getting SSP base clock rate.\r\n")));
		goto cleanup;
	}

	// Allocate SSP and initialize SSP interface
	pDev->pSSPClass = new bspssp(pDev->pSSPRegs, pDev->pClkPwrRegs, pDev->sspNum);
	if (pDev->pSSPClass == NULL)
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("SSP_Init: Failed to allocate SSP driver class\r\n")));
		goto cleanup;
	}

	// Get sysintr value
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irqt, sizeof(irqt),
		&pDev->SSPsysIntr, sizeof(pDev->SSPsysIntr), NULL))
    {
        RETAILMSG(ZONE_ERROR, 
            (TEXT("SSP_Init: Failed to request the SSP sysintr.\r\n")));

        pDev->SSPsysIntr = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	// Create event handle
	pDev->sspEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (pDev->sspEvent == NULL)
	{
        RETAILMSG(ZONE_ERROR, 
            (TEXT("SSP_Init: Failed to create handler event.\r\n")));
		goto cleanup;
	}

	// Initialize the SSP interrupt
	if (InterruptInitialize(pDev->SSPsysIntr, pDev->sspEvent,
		NULL, 0) == FALSE) {
		// Cannot initialize interrupt
        RETAILMSG(ZONE_ERROR, 
			(TEXT("SSP_Init: Cannot initialize SSP interrupt\r\n")));
        goto cleanup;
	}
	InterruptDone(pDev->SSPsysIntr);

	// Create access mutex for SSP registers
	pDev->sspLock = CreateMutex(NULL, FALSE, NULL);
	if (pDev->sspLock == NULL) {
        RETAILMSG(ZONE_ERROR,
			(TEXT("SSP_Init: Error creating control mutex\r\n")));
		goto cleanup;
	}

	// Setup for 8 bit transfers, 100K clock, locked CS
    sspcfg.databits = 8;
    sspcfg.mode = SSP_CR0_FRF_MOT;
    sspcfg.highclkSpiFrames = FALSE;
    sspcfg.usesecondClkSpi = FALSE;
    sspcfg.sspClk = 100000;
    sspcfg.masterMode = TRUE;
	pDev->pSSPClass->bspsspSetup(clk, &sspcfg);
	sts = (DWORD) pDev;

	// Disable SSP clock until it is needed
	SSPPeriphDown((DWORD) pDev);

cleanup:
	return sts;
}

//------------------------------------------------------------------------------
//
// SSP_IOControl
//
// SSP driver general IOCTL handler function
//
extern "C" BOOL SSP_IOControl(DWORD hOpenContext, DWORD dwCode, PBYTE pBufIn,
                   DWORD dwLenIn, PBYTE pBufOut, DWORD dwLenOut,
                   PDWORD pdwActualOut) {
	UINT32 bytesret, clk;
	SSP_XFER_SETUP_T *psspSetup;
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hOpenContext;

	switch (dwCode)
	{
	case IOCTL_APP_SSPREQ:
        // Attempt transfer
		if ((pBufIn != NULL) && (pBufOut != NULL) &&
			(dwLenIn == sizeof(SSP_SEND_XFER_T)) &&
			(dwLenOut == sizeof(SSP_RECV_XFER_T)))
		{
			*pdwActualOut = sizeof(SSP_SEND_XFER_T);
            return SSP_Transfer(pDev, (SSP_SEND_XFER_T *) pBufIn,
				(SSP_RECV_XFER_T *) pBufOut);
		}
		break;

	case IOCTL_APP_SSPSETUP:
        // Attempt setup
		if ((pBufIn != NULL) && (dwLenIn == sizeof(SSP_XFER_SETUP_T)))
		{
			// Get base clock for SSP
			if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
				sizeof (clk), (LPDWORD) &bytesret) != FALSE)
			{
				psspSetup = (SSP_XFER_SETUP_T *) pBufIn;
				SSP_Lock(pDev->sspLock);
				pDev->pSSPClass->bspsspSetup(clk, psspSetup);

				// Also setup GPIO if needed
				SSP_CS_Setup(pDev, psspSetup->csLocked);

				SSP_Unlock(pDev->sspLock);
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
// SSP_Close
//
// SSP driver close function
//
extern "C" BOOL SSP_Close(DWORD hOpenContext) {
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hOpenContext;

	sspInstances[pDev->sspNum]--;
	if (sspInstances[pDev->sspNum] == 0)
	{
		// Disable clocks, no more SSP channels open
		SSPPeriphDown(hOpenContext);
	}

	return TRUE;
}

//------------------------------------------------------------------------------
//
// SSP_Open
//
// SSP driver open function
//
extern "C" DWORD SSP_Open(DWORD hDeviceContext, DWORD AccessCode,
               DWORD ShareMode) {
	SSP_DRVCTL_T *pDev = (SSP_DRVCTL_T *) hDeviceContext;

    (void) AccessCode;
    (void) ShareMode;

	sspInstances[pDev->sspNum]++;
	if (sspInstances[pDev->sspNum] == 1)
	{
		// Enable clocks
		SSPPeriphUp(hDeviceContext);
	}

	// Return the device context, this will be used as the open
	// context to identify which SSP channel is used for operations
    return hDeviceContext;
}

//------------------------------------------------------------------------------
//
// SSP_Read
//
// SSP driver read stub function
//
extern "C" DWORD SSP_Read(DWORD hOpenContext, LPVOID pBuffer, DWORD Count) {
    (void) hOpenContext;
    (void) pBuffer;
    (void) Count;

    /* Read not allowed */
    return 0;
}

//------------------------------------------------------------------------------
//
// SSP_Write
//
// SSP driver write stub function
//
extern "C" DWORD SSP_Write(DWORD hOpenContext, LPCVOID pBuffer, DWORD Count) {
    (void) hOpenContext;
    (void) pBuffer;
    (void) Count;

    /* Write not allowed */
    return 0;
}

//------------------------------------------------------------------------------
//
// SSP_Seek
//
// SSP driver seek stub function
//
extern "C" DWORD SSP_Seek(DWORD hOpenContext, long Amount, WORD Type) {
    (void) hOpenContext;
    (void) Amount;
    (void) Type;

    /* Seek not allowed */
    return 0;
}

//------------------------------------------------------------------------------
//
// DllMain
//
// SSP DLL entry point
//
extern "C" BOOL __stdcall DllMain (
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
