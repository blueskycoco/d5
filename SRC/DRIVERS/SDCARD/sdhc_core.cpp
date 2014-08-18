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
// sdhc_core.cpp
//
// SDHC controller driver implementation
//

#include <SDCardDDK.h>
#include "SDHC.h"
#include <nkintr.h>
#include "bsp.h"
#include "intr.h"

#define IndicateBusRequestComplete(pRequest, status) \
    SDHCDIndicateBusRequestComplete(m_pHCContext, \
    (pRequest), (status))

// Instance counter
int sdCardController::instance = 0;

//------------------------------------------------------------------------------
//
// StartDetectThread
//
// Start the detect thread with the passed class
//
static void StartDetectThread(class sdCardController *pSdCardClass)
{
	pSdCardClass->sdCDThread();
}

//------------------------------------------------------------------------------
//
// StartSD0Thread
//
// Start the command thread with the passed class
//
static void StartSD0Thread(class sdCardController *pSdCardClass)
{
	pSdCardClass->sdSD0Thread();
}

//------------------------------------------------------------------------------
//
// SDHCInitialize
//
// SDHC callback function
//
SD_API_STATUS SDHCInitialize(PSDCARD_HC_CONTEXT pHCContext)
{
    class sdCardController *pController = GET_PCONTROLLER_FROM_HCD(pHCContext);
    return pController->SDHCInitializeImpl();
}

//------------------------------------------------------------------------------
//
// SDHCDeinitialize
//
// SDHC callback function
//
SD_API_STATUS SDHCDeinitialize(PSDCARD_HC_CONTEXT pHCContext)
{
    class sdCardController *pController = GET_PCONTROLLER_FROM_HCD(pHCContext);
    return pController->SDHCDeinitializeImpl();
}

//------------------------------------------------------------------------------
//
// SDHCCancelIoHandler
//
// SDHC callback function
//
BOOLEAN SDHCCancelIoHandler(PSDCARD_HC_CONTEXT pHCContext,
							DWORD Slot,
							PSD_BUS_REQUEST pRequest)
{
    class sdCardController *pController = GET_PCONTROLLER_FROM_HCD(pHCContext);
    return pController->SDHCCancelIoHandlerImpl((UCHAR)Slot, pRequest);
}

//------------------------------------------------------------------------------
//
// SDHCSlotOptionHandler
//
// SDHC callback function
//
SD_API_STATUS SDHCSlotOptionHandler(PSDCARD_HC_CONTEXT pHCContext,
						            DWORD SlotNumber,
									SD_SLOT_OPTION_CODE Option,
									PVOID pData,
									ULONG OptionSize)
{
    class sdCardController *pController = GET_PCONTROLLER_FROM_HCD(pHCContext);
    return pController->SDHCSlotOptionHandlerImpl((UCHAR)SlotNumber,
		Option, pData, OptionSize );
}

//------------------------------------------------------------------------------
//
// SDHCBusRequestHandler
//
// SDHC callback function
//
SD_API_STATUS SDHCBusRequestHandler(PSDCARD_HC_CONTEXT pHCContext,
							        DWORD Slot,
									PSD_BUS_REQUEST pRequest)
{
    class sdCardController *pController = GET_PCONTROLLER_FROM_HCD(pHCContext);
    return pController->SDHCBusRequestHandlerImpl(pRequest);
}

//********************************************************************
// Class constructor and destructor functions
//********************************************************************

//------------------------------------------------------------------------------
//
// sdCardController
//
// Constructor
//
sdCardController::sdCardController(void)
{
	// Multilpe instances of this class aren't really allowed
	instance++;
	if (instance == 1)
	{
		// Set initial class values here
		sdCardSetInitialValues();
	}
}

//------------------------------------------------------------------------------
//
// ~sdCardController
//
// Destructor
//
sdCardController::~sdCardController()
{
	instance--;
}

//------------------------------------------------------------------------------
//
// ~sdCardSetInitialValues
//
// Setup initial driver values - this function is called when the
// driver is first instantiated or when the driver is destroyed, it
// will setup initial values for all class objects
//
void sdCardController::sdCardSetInitialValues(void)
{
	// Access to registers
	m_pGPIORegs = NULL;
	m_pSDCardRegs = NULL;
	m_pSDCardRegs = NULL;
	m_pClkPwrRegs = NULL;

	// SD host context and handle data
    m_hParentBus = NULL;
    m_pHCContext = NULL;
	m_registeredWithBusDriver = FALSE;
	m_PowerState = D4;

	// Card detect IRQ, sysIntr, events, threads, etc.
	m_dwSDCDIrq = MAPPED_INTROAL_VAL;
	m_dwSysintrCD = SYSINTR_UNDEFINED;
	m_hCardInsertInterruptEvent = INVALID_HANDLE_VALUE;
	m_DetectThreadKill = FALSE;
	m_detectThreadDone = FALSE;
	m_cardInserted = FALSE;

	// Comnmand IRQ, sysIntr, event, thread, etc.
	m_dwIrq0 = OAL_INTR_IRQ_SD0;
	m_dwSysIntr0 = SYSINTR_UNDEFINED;
	m_eventSD0 = INVALID_HANDLE_VALUE;
	m_SD0ThreadKill = FALSE;
	m_SD0ThreadDone = FALSE;

	// DMA driver handle
	m_dmaCtl = 0;

	m_hardwareInitialized = FALSE;
}

//********************************************************************
// SDHC initialization and de-init functions. Sets up hardware and
// functions that are used whether a card is installed or not.
//********************************************************************

//------------------------------------------------------------------------------
//
// sdCardGetRegistrySettings
//
// Read the registry settings
//
BOOL sdCardController::sdCardGetRegistrySettings(LPCTSTR pszActiveKey)
{
	CReg regDevice;
    HKEY hKeyDevice = NULL;
    BOOL fRet = FALSE;

	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("sdCardController: INFO: SDHC Active RegPath: %s\r\n"),
		pszActiveKey));

    hKeyDevice = OpenDeviceKey(pszActiveKey);
    if ((hKeyDevice == NULL) || (!regDevice.Open(hKeyDevice, NULL)))
	{
        DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("sdCardController: ERROR: Failed to open device key\r\n")));
        goto error;
    }

	// Thread priorities
	m_dwCardDetectIstThreadPriority =
		regDevice.ValueDW(SHC_DETECT_THREAD_PRIORITY_KEY, SHC_DETECT_THREAD_PRIORITY_DEF);
	m_dwSD0IstThreadPriority =
		regDevice.ValueDW(SHC_SD0_THREAD_PRIORITY_KEY, SHC_SD0_THREAD_PRIORITY_DEF);

	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("REGISTRY: m_dwCardDetectIstThreadPriority: %x\r\n"),
		m_dwCardDetectIstThreadPriority));
	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("REGISTRY: m_dwSD0IstThreadPriority: %x\r\n"),
		m_dwSD0IstThreadPriority));

	// Data timeout clocks
	m_dataTimeout = regDevice.ValueDW(SHC_DATA_TIMEOUT_KEY, SHC_DATA_TIMEOUT_DEF);
	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("REGISTRY: m_dataTimeout: %x\r\n"),
		m_dataTimeout));

    // Maximum clock frequency
    m_dwMaxClockRate = regDevice.ValueDW(SHC_FREQUENCY_MAX_KEY, SHC_FREQUENCY_MAX_DEF);
	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("REGISTRY: m_dwMaxClockRate: %d\r\n"),
		m_dwMaxClockRate));

	fRet = TRUE;

error:
    if (hKeyDevice)
	{
		RegCloseKey(hKeyDevice);
	}
	
	return fRet;
}

//------------------------------------------------------------------------------
//
// sdInit
//
// Initialize class and context
//
DWORD sdCardController::sdInit(LPCTSTR pszActiveKey)
{
    SD_API_STATUS status;
    DWORD dwRet = 0;

    DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC +Init\r\n")));    
	// Setup initial driver values
	sdCardSetInitialValues();

	// Get registry settings
	if (sdCardGetRegistrySettings(pszActiveKey) == FALSE)
	{
        DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHC: Failed to get registry settings\r\n")));
        goto initfail;
	}

	// Open the parent bus driver handle
    m_hParentBus = CreateBusAccessHandle(pszActiveKey);
    if (m_hParentBus == NULL)
	{
        DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHC: Failed to obtain parent bus handle\r\n")));
        goto initfail;
    }

	// Allocate the context for 1 slot
    status = SDHCDAllocateContext(1, &m_pHCContext);
    if (!SD_API_SUCCESS(status)) {
        DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHC: Failed to allocate context : 0x%08X\r\n"),
            status));
        goto initfail;
    }

    // Set our extension
    m_pHCContext->pHCSpecificContext = this;

	//
    // Set the SDHC driver function callbacks
    // Set the host controller name
    SDHCDSetHCName(m_pHCContext, TEXT("SDHC"));

    // set init handler
    SDHCDSetControllerInitHandler(m_pHCContext,
		(PSD_INITIALIZE_CONTROLLER) &SDHCInitialize);

    // set deinit handler    
    SDHCDSetControllerDeinitHandler(m_pHCContext,
		(PSD_DEINITIALIZE_CONTROLLER) &SDHCDeinitialize);

    // set the Send packet handler
    SDHCDSetBusRequestHandler(m_pHCContext,
		(PSD_BUS_REQUEST_HANDLER) &SDHCBusRequestHandler);

    // set the cancel I/O handler
    SDHCDSetCancelIOHandler(m_pHCContext,
		(PSD_CANCEL_REQUEST_HANDLER) &SDHCCancelIoHandler);

    // set the slot option handler
    SDHCDSetSlotOptionHandler(m_pHCContext,
		(PSD_GET_SET_SLOT_OPTION) &SDHCSlotOptionHandler);

    // Register the host controller with the bus driver
    status = SDHCDRegisterHostController(m_pHCContext);
    if (!SD_API_SUCCESS(status)) {
        DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHC Failed to register host controller: %0x08X\r\n"),status));
        goto cleanup;
    }

	m_registeredWithBusDriver = TRUE;

	// Return the controller context
    dwRet = (DWORD) this;

initfail:
cleanup:
	if (dwRet == 0)
	{
		if (m_pHCContext)
		{
			sdCardFreeHostContext();
		}

		// Restore initial driver status
		sdCardSetInitialValues();
	}

	return dwRet;	
}

//------------------------------------------------------------------------------
//
// sdCardFreeHostContext
//
// Free host context
//
void sdCardController::sdCardFreeHostContext(void)
{
    // Deregister the host controller with the bust driver if needed
	if (m_registeredWithBusDriver)
	{
        SDHCDDeregisterHostController(m_pHCContext);
	}

	// De-init hardware
	if (m_hardwareInitialized != FALSE)
	{
	    sdHwShutdown();
	}

	// Return system resources
	SDHCDeinitializeImpl();

	// Free up bus handle
	if (m_hParentBus != NULL)
    {
        m_PowerState = D4;
        UpdateDevicePowerState();
        CloseBusAccessHandle(m_hParentBus);
        m_hParentBus = NULL;
    }

    // Cleanup the host context
	if (m_pHCContext != NULL)
	{
	    SDHCDDeleteContext(m_pHCContext);
		m_pHCContext = NULL;
	}
}

//********************************************************************
// Hardware setup functions - These functions are called whenever a
// card is inserted or removed to initialize or de-init specific
// system resources
//********************************************************************

//------------------------------------------------------------------------------
//
// sdHwSetup
//
// Hardware setup - sets up the hardware when a card is initially
// inserted into the device
//
BOOL sdCardController::sdHwSetup(void)
{
	DWORD threadID;
	// Hardware not yet initialized
	m_hardwareInitialized = FALSE;
	
	// Enable slot power and clocking
	sdHwSlotPowerControl(m_pGPIORegs, TRUE);
	sdCardEnableClocks(TRUE);
	Sleep(100);

	// Set the initial SD card setup
	sdCardClearController();
	m_pSDCardRegs->sd_power = (SD_OPENDRAIN_EN | SD_POWER_OFF_MODE); // Power off state, open drain mode
	m_pSDCardRegs->sd_clock = (SD_CLKDIV_MASK | SD_SDCLK_PWRSAVE); // Slowest/no clock, 1 bit mode

	// Take controller out of power OFF state, leave in open drain mode, enable
	// low power clock mode
	m_pSDCardRegs->sd_power = (SD_OPENDRAIN_EN | SD_POWER_ON_MODE);
	m_pSDCardRegs->sd_clock |= (SD_SDCLK_PWRSAVE | SD_SDCLK_EN);

	// Enable and setup DMA for the SD card controller, this is
	// initially setup for the transmit direction, but may change
	// during use
	m_dmaStp.dmaCh = DMAC_SDCARD_CH;
	m_dmaStp.perID = DMA_PERID_SDCARD;
	m_dmaStp.SrcInc = 0; // Changes during driver use
	m_dmaStp.DestInc = 1; // Changes during driver use
	m_dmaStp.SrcWidth = DMAC_CHAN_SRC_WIDTH_32;
	m_dmaStp.DestWidth = DMAC_CHAN_DEST_WIDTH_32;
	m_dmaStp.SrcBurstSize = DMAC_CHAN_SRC_BURST_8;
	m_dmaStp.DestBurstSize = DMAC_CHAN_DEST_BURST_8;
	m_dmaStp.perFlowSource = DMAC_CHAN_FLOW_P_P2M; // Changes during driver use
	m_dmaStp.destPeripheral = DMAC_DEST_PERIP(DMA_PERID_SDCARD);
	m_dmaStp.srcPeripheral = DMAC_SRC_PERIP(DMA_PERID_SDCARD);
	m_dmaCtl = dmaAllocate(&m_dmaStp, (64 * 1024)); // 64K of buffers
	if (m_dmaCtl == 0)
	{
		// Couldn't setup DMA
		DEBUGMSG(SDHC_ERROR_ZONE, 
			(TEXT("sdCardController: ERROR: Critical error setting up DMA\r\n")));
		m_dmaCtl = 0;
        goto exitInit;
	}

	// Get sysintr value for DMA channel interrupt
	m_dmainfo = dmaGetInfo(m_dmaCtl);

	// Save DMA buffer data
	m_dmaBuffPhy = m_dmainfo->pBuffPhy;
	m_dmaBuffVirt = m_dmainfo->pBuffVirt;
	m_dmaBuffSizeBytes = m_dmainfo->buffSize;

	DEBUGMSG(SDHC_INIT_ZONE, 
		(TEXT("sdCardController: INIT: DMA buffer PHY 0x%x VIRT 0x%x Size %d\r\n"),
		m_dmaBuffPhy, m_dmaBuffVirt, m_dmaBuffSizeBytes));

	// Reset command and data events
	ResetEvent(m_eventSD0);

	// Start command thread
    m_SD0ThreadKill = FALSE;
	m_SD0ThreadDone = FALSE;
	m_SD0InterruptThread =  CreateThread(NULL, 0,
		(LPTHREAD_START_ROUTINE) StartSD0Thread, this, 0, &threadID);
	if (m_SD0InterruptThread == NULL)
	{
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHCInitializeImpl: Error creating command thread!\r\n")));
        goto exitInit;
    }

	m_hardwareInitialized = TRUE;

exitInit:
	return m_hardwareInitialized;
}

//------------------------------------------------------------------------------
//
// sdHwShutdown
//
// Shuts down the hardware when a card is removed
//
void sdCardController::sdHwShutdown(void)
{
	int to;
	// Clear controller state
	sdCardEnableClocks(TRUE);
	sdCardClearController();
	m_pSDCardRegs->sd_power = (SD_OPENDRAIN_EN | SD_POWER_OFF_MODE); // Power off state, open drain mode
	m_pSDCardRegs->sd_clock = (SD_CLKDIV_MASK | SD_SDCLK_PWRSAVE); // Slowest/no clock, 1 bit mode

	// Stop command thread
	m_SD0ThreadKill = TRUE;
	to = 20;
	while ((to > 0) && (m_SD0ThreadDone == FALSE))
	{
		SetEvent(m_eventSD0);
		Sleep(50);
	}

	// Return DMA resources from DMA driver
	if (m_dmaCtl != 0)
	{
		dmaDisable(m_dmaCtl);
		dmaDeAllocate(m_dmaCtl);
		m_dmaCtl = 0;
	}

	// Disable SD card controller clocks
	sdCardEnableClocks(FALSE);

	m_hardwareInitialized = FALSE;
}

//********************************************************************
// System threads
//********************************************************************

//------------------------------------------------------------------------------
//
// sdCDThread
//
// SD card detection and signal thread
//
DWORD sdCardController::sdCDThread(void)
{
    DWORD waitStatus;
	PSD_BUS_REQUEST pRequest;
	// Set thread priority from configured priority
    CeSetThreadPriority(GetCurrentThread(),
		m_dwCardDetectIstThreadPriority);

	// Card initially not installed
	m_cardInserted = FALSE;

	// Thread should run at least once
	SetEvent(m_hCardInsertInterruptEvent);

	// Loop until requested to stop
    while (m_DetectThreadKill == FALSE)
    {
        waitStatus = WaitForSingleObject(m_hCardInsertInterruptEvent,
			INFINITE);
        if (waitStatus != WAIT_OBJECT_0) {
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Detect thread error\r\n")));
            m_detectThreadDone = TRUE;
			return 0;
        }
		else if (m_DetectThreadKill == FALSE)
		{
			// Allows detect signal to debounce
		    Sleep(750);

		    DEBUGMSG(SDHC_SDBUS_INTERACT_ZONE,
				(TEXT("SD card insertion(1)/removal(0) = %d/%d\r\n"),
				sdHwCardInserted(m_pGPIORegs), m_cardInserted));

			// Has the state changed?
			if (m_cardInserted != sdHwCardInserted(m_pGPIORegs))
			{
				// Get new card insertion state
				m_cardInserted = sdHwCardInserted(m_pGPIORegs);

				// Update card dection logic to new state
				sdHwUpdateDetectState(m_pGPIORegs, m_pINTCRegs);

				SDHCDAcquireHCLock(m_pHCContext);

				// Insert or removal event?
				if (m_cardInserted != FALSE)
				{
					// Initialize hardware and tell host that device is inserted
					m_savedCardAddr = 0xFFFFFFFF;
					if (sdHwSetup() == TRUE)
					{
						SDHCDIndicateSlotStateChange(m_pHCContext, (UCHAR) 0,
							DeviceInserted);
					}
					else
					{
					    DEBUGMSG(SDHC_ERROR_ZONE, (TEXT("Error initializing hardware on insertion\r\n")));
						sdHwShutdown();
						sdHwSlotPowerControl(m_pGPIORegs, FALSE);
						sdCardEnableClocks(FALSE);
						m_cardInserted = FALSE;
					}
				}
				else
				{
					// Tell host that device is removed
					SDHCDIndicateSlotStateChange(m_pHCContext, (UCHAR) 0,
						DeviceEjected);

			        // Get the current request  
			        pRequest = SDHCDGetAndLockCurrentRequest(m_pHCContext, (UCHAR) 0);
			        if (pRequest != NULL)
					{
						SDHCDIndicateBusRequestComplete(m_pHCContext,
							pRequest, SD_API_STATUS_DEVICE_REMOVED);
						DEBUGMSG(SDHC_ERROR_ZONE, 
			                (TEXT("Card Removal Detected - Canceling current request: 0x%08X, command: %d\r\n"), 
			                pRequest, pRequest->CommandCode));
			        }

					// Shutdown hardware
					sdHwShutdown();

					// Disable slot power and clocking
					sdCardEnableClocks(FALSE);
					sdHwSlotPowerControl(m_pGPIORegs, FALSE);
				}

				SDHCDReleaseHCLock(m_pHCContext);
			}
		}

	    InterruptDone(m_dwSysintrCD);
	}

	// Thread has exited flag
	m_detectThreadDone = TRUE;

	DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC: Detect thread shutdown\r\n")));

	return 0;
}

//------------------------------------------------------------------------------
//
// sdCheckBusyStatus
//
// Checks busy status after write commands
//
SD_API_STATUS sdCardController::sdCheckBusyStatus(void)
{
#if 0
	UINT32 reg;
	SD_API_STATUS status = SD_API_STATUS_UNSUCCESSFUL;
	int tries = 400;

	while (tries > 0)
	{
		// Submit new SDMMC command to check status, command interrupt
		// may call this function again
		sdCmdSMSetup(13, m_savedCardAddr, ResponseR1);
		InterruptDone(m_dwSysIntr0);
		m_pSDCardRegs->sd_cmd |= SD_CPST_EN;

		WaitForSingleObject(m_eventSD0, INFINITE);

		reg = m_pSDCardRegs->sd_status;

		// Has the transfer occurred error-free?
		if ((reg & (SD_CMD_CRC_FAIL | SD_CMD_TIMEOUT)) != 0)
		{
			status = SD_API_STATUS_RESPONSE_TIMEOUT;
			tries = 0;
		}
		else
		{
			// Response received, verify it
			reg = m_pSDCardRegs->sd_resp [0];
			if ((reg & 0x0100) != 0)
			{
				// Program complete
				tries = 0;
				status = SD_API_STATUS_SUCCESS;
			}
			else
			{
				Sleep(SW_DELAY);
			}
		}

		tries--;
	}

	return status;
#else
	return SD_API_STATUS_SUCCESS;
#endif
}

//------------------------------------------------------------------------------
//
// sdSD0Thread
//
// SD command and data handler thread
//
DWORD sdCardController::sdSD0Thread(void)
{
    DWORD waitStatus;
	SD_API_STATUS status;
	UINT32 tmp;
	PSD_BUS_REQUEST pRequest = NULL;

	// Set thread priority from configured priority
    CeSetThreadPriority(GetCurrentThread(),
		m_SD0IstThreadPriority);
	while (m_SD0ThreadKill == FALSE)
	{
        waitStatus = WaitForSingleObject(m_eventSD0, INFINITE);
        if (waitStatus != WAIT_OBJECT_0) {
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Command thread error\r\n")));
            m_SD0ThreadDone = TRUE;
			return 0;
        }
		else if (m_SD0ThreadKill == FALSE)
		{
			// Get current bus request
			pRequest = SDHCDGetAndLockCurrentRequest(m_pHCContext, 0);
			if (pRequest == NULL)
			{
				// This should never happen
				status = SD_API_STATUS_INVALID_DEVICE_REQUEST;
			}
			else
			{
				// Get current status and clear status
				tmp = m_pSDCardRegs->sd_status;

				// Command handler?
				if ((tmp & m_SD0CmdMask) != 0)
				{
					sdCardHandleCommand(tmp, pRequest);
				}

				// Data handler?
				tmp = m_pSDCardRegs->sd_status;
				if ((tmp & m_SD1DataMask) != 0)
				{
					sdCardHandleData(tmp, pRequest);
				}

				// If either of the requests is still pending, don't complete
				// the bus request yet
				if ((m_cmdStatus != SD_API_STATUS_PENDING) &&
					(m_dataStatus != SD_API_STATUS_PENDING))
				{
					status = SD_API_STATUS_SUCCESS;
					if (m_cmdStatus != SD_API_STATUS_SUCCESS)
					{
						status = m_cmdStatus;
					}
					else if (m_dataStatus != SD_API_STATUS_SUCCESS)
					{
						status = m_dataStatus;
					}

					// Need to wait for ready status
					if ((m_BusyCheckFlag == TRUE) && (status == SD_API_STATUS_SUCCESS))
					{
#ifdef USE_HW_CHECK
						// Monitor busy status on DAT0 line
						tmp = 0;
						while ((m_pGPIORegs->pio_inp_state & (1 << 2)) == 0)
						{
							Sleep(0);
							tmp++;
						}
#else
						// Handle busy status check
						status = sdCheckBusyStatus();
#endif
						m_BusyCheckFlag = FALSE;
					}

					Sleep(50); // Hack for now
					CompleteBusDataRequest(status, pRequest);
				}
				else
				{
					SDHCDUnlockRequest(m_pHCContext, pRequest);
				}
			}
		}

		InterruptDone(m_dwSysIntr0);
	}

	// Thread has exited flag
	m_SD0ThreadDone = TRUE;

	DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SDHC: Command thread shutdown\r\n")));

	return 0;
}

void sdCardController::sdCardHandleData(UINT32 reg, PSD_BUS_REQUEST pRequest)
{
	// Stop state machine and DMA
	m_pSDCardRegs->sd_dctrl &= ~SD_DATATRANSFER_EN;
	m_pSDCardRegs->sd_mask0 &= ~m_SD1DataMask;
	m_pSDCardRegs->sd_clear = m_SD1DataMask;
	dmaDisable(m_dmaCtl);

	// Check error statuses
	if ((reg & (SD_FIFO_RXDATA_OFLOW | SD_STARTBIT_ERR |
		SD_FIFO_TXDATA_UFLOW | SD_DATA_CRC_FAIL |
		SD_DATA_TIMEOUT)) != 0)
	{
		if ((reg & SD_DATA_CRC_FAIL) != 0)
		{
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Data CRC error: SD status=0x%x\r\n"), reg));
		}
		if ((reg & SD_DATA_TIMEOUT) != 0)
		{
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Data timeout error: SD status=0x%x\r\n"), reg));
		}
		if ((reg & SD_FIFO_RXDATA_OFLOW) != 0)
		{
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Data overflow error: SD status=0x%x\r\n"), reg));
		}
		if ((reg & SD_STARTBIT_ERR) != 0)
		{
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Data start error: SD status=0x%x\r\n"), reg));
		}
		if ((reg & SD_FIFO_TXDATA_UFLOW) != 0)
		{
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Data underflow error: SD status=0x%x\r\n"), reg));
		}

		m_dataStatus = SD_API_STATUS_UNSUCCESSFUL;
	}
	else
	{
		// Copy DMA data to buffer if this was a read
		if (m_TransferClass == SD_READ)
		{
			if (m_read_iterations > 1)
			{
				SDPerformSafeCopy((void *) m_readbuff, (void *) m_dmaBuffVirt,
					m_BlockSize);
				m_readbuff += m_BlockSize;
				m_bytesleft -= m_BlockSize;

				// Issue command 17 again
				m_CommandArgument += m_BlockSize;
				sdCmdSMSetup(17, m_CommandArgument, m_ResponseType);
				sdDataSMSetup(pRequest->DataAccessClocks);

				m_cmdStatus = SD_API_STATUS_PENDING;
				m_dataStatus = SD_API_STATUS_PENDING;
				m_pSDCardRegs->sd_dctrl |= SD_DATATRANSFER_EN;
				m_pSDCardRegs->sd_cmd |= SD_CPST_EN;

				m_read_iterations--;
			}
			else
			{
				SDPerformSafeCopy((void *) m_readbuff, (void *) m_dmaBuffVirt,
					m_bytesleft);
				m_dataStatus = SD_API_STATUS_SUCCESS;
			}
		}
		else
		{
			// Write operation is complete
			m_dataStatus = SD_API_STATUS_SUCCESS;
		}
	}
}

void sdCardController::sdCardHandleCommand(UINT32 reg, PSD_BUS_REQUEST pRequest)
{
	// Clear the SD command interrupt
	m_pSDCardRegs->sd_cmd &= ~SD_CPST_EN;
	m_pSDCardRegs->sd_mask0 &= ~m_SD0CmdMask;
	m_pSDCardRegs->sd_clear = m_SD0CmdMask;
	// Has the transfer occurred error-free?
	if ((reg & (SD_CMD_CRC_FAIL | SD_CMD_TIMEOUT)) != 0)
	{
		// Command timeout
		if ((reg & SD_CMD_TIMEOUT) != 0)
		{
			m_cmdStatus = SD_API_STATUS_RESPONSE_TIMEOUT;
			m_dataStatus = SD_API_STATUS_UNSUCCESSFUL;
			DEBUGMSG(SDHC_ERROR_ZONE,
				(TEXT("SDHC: Command timeout error: SD status = 0x%x\r\n"), reg));
		}
		else
		{
			// Ignore the CRC error on ACMD41 and CMD1
			if ((m_CommandCode == 1) || (m_CommandCode == 41))
			{
				// CRC error on OCR requests, the response register will not
				// have the right status, so read it here instead
				m_cmdStatus = SD_API_STATUS_SUCCESS;
				sdRespCopy(pRequest->CommandResponse.ResponseBuffer);
			}
			else
			{
				m_cmdStatus = SD_API_STATUS_CRC_ERROR;
				m_dataStatus = SD_API_STATUS_UNSUCCESSFUL;
				DEBUGMSG(SDHC_ERROR_ZONE,
					(TEXT("SDHC: Command  CRC error: SD status = 0x%x\r\n"), reg));
			}
		}
	}
	else
	{
		// Command completed successfully
		m_cmdStatus = SD_API_STATUS_SUCCESS;

		if (m_TransferClass == SD_WRITE)
		{
			// Start data state machine
			m_pSDCardRegs->sd_dctrl |= SD_DATATRANSFER_EN;
		}

		// Capture card address
		if (m_savedCardAddr == 0xFFFFFFFF)
		{
			if (m_CommandCode == 9)
			{
				m_savedCardAddr = m_CommandArgument;
				DEBUGMSG(SDHC_INIT_ZONE,
					(TEXT("SDHC: Saved card address 0x%x\r\n"), m_savedCardAddr));
			}
		}
	}

	// Copy response
	if (m_cmdStatus == SD_API_STATUS_SUCCESS)
	{
		sdRespCopy(pRequest->CommandResponse.ResponseBuffer);
	}
	else
	{
		memset(pRequest->CommandResponse.ResponseBuffer, 0,
			sizeof(UINT32));
	}
}

//********************************************************************
// SD card controller support functions
//********************************************************************

//------------------------------------------------------------------------------
//
// sdCardClearController
//
// Clear/reset SD card controller to default idle state, may be used
// to clear a function that failed or timed out
//
void sdCardController::sdCardClearController(void)
{
	volatile UINT32 tmp;

	if (m_hardwareInitialized == TRUE)
	{
		m_pSDCardRegs->sd_cmd = 0; // Command state machine disabled
		m_pSDCardRegs->sd_dctrl = SD_BLKSIZE_512BYTES; // Data state machine disabled, 512 byte block
		m_pSDCardRegs->sd_mask0 = 0; // SD0 interrupts masked
		m_pSDCardRegs->sd_mask1 = 0; // SD1 interrupts masked
		m_pSDCardRegs->sd_clear = 0xFF7FF; // All status cleared
		m_pSDCardRegs->sd_dtimer = 0xFFFFFF; // Default timeout period in clocks

		// Empty the FIFO
		while ((m_pSDCardRegs->sd_status & SD_FIFO_RXDATA_AVAIL) != 0)
		{
			tmp = m_pSDCardRegs->sd_fifo [0];
		}

		// Clear DMA
		dmaDisable(m_dmaCtl);
	}
}

//------------------------------------------------------------------------------
//
// sdCardEnableClocks
//
// Stops or starts the SD clocks (in the CLKPWR controller)
//
void sdCardController::sdCardEnableClocks(BOOL enable)
{

	if (enable == FALSE)
	{
		// Disable SD card controller clocking
		m_pClkPwrRegs->clkpwr_ms_ctrl &= ~CLKPWR_MSCARD_SDCARD_EN;
	}
	else
	{
		// Enable SD card controller clocking
		m_pClkPwrRegs->clkpwr_ms_ctrl |= CLKPWR_MSCARD_SDCARD_EN;
	}
}

//------------------------------------------------------------------------------
//
// sdCardSetClockRate
//
//  Set clock rate based on HC capability
//
void sdCardController::sdCardSetClockRate(PDWORD pdwRate)
{
	UINT32 tmp, sddiv, sdclk, bytesret;

	// Get base clock rate for SD card controller block
	if (KernelIoControl(IOCTL_LPC32XX_GETARMCLK, NULL, 0, &sdclk,
		sizeof (sdclk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock, use default
        DEBUGMSG(SDHC_ERROR_ZONE, 
            (TEXT("SDHC: Error getting SD card base clock rate.\r\n")));
		sdclk = 208000000;
	}

	// Limit clock rate to registry setting
	if (*pdwRate > m_dwMaxClockRate)
	{
		*pdwRate = m_dwMaxClockRate;
	}

	// Slightly bump up clock rate by 5% to allow for better performance in
	// some cases when on the edge of a divider
	*pdwRate += (*pdwRate / 20);

	// If the clock rate is 1M or lower, set a larger divider in the CLKPWR
	// controller
	if (*pdwRate < 1000000)
	{
		tmp = m_pClkPwrRegs->clkpwr_ms_ctrl & ~(CLKPWR_MSCARD_SDCARD_DIV(0xF));
		m_pClkPwrRegs->clkpwr_ms_ctrl = tmp | CLKPWR_MSCARD_SDCARD_DIV(8);
		sdclk = sdclk / 8;
	}
	else
	{
		tmp = m_pClkPwrRegs->clkpwr_ms_ctrl & ~(CLKPWR_MSCARD_SDCARD_DIV(0xF));
		m_pClkPwrRegs->clkpwr_ms_ctrl = tmp | CLKPWR_MSCARD_SDCARD_DIV(1);
	}

	/* Find best divider to generate target clock rate */
    sddiv = 0;
	tmp = m_pSDCardRegs->sd_clock & ~(SD_CLKDIV_MASK |
		SD_SDCLK_BYPASS);
    while ((sdclk / (2 * (sddiv + 1))) >= (UINT32) *pdwRate)
    {
    	sddiv++;
    }
    if (sddiv > SD_CLKDIV_MASK)
    {
    	/* Limit to maximum supported divider */
    	sddiv = SD_CLKDIV_MASK;
    }
    else if (sddiv == 0)
    {
    	/* May have to use the clock bypass instead */
    	if ((UINT32) *pdwRate >= sdclk)
    	{
    		tmp |= SD_SDCLK_BYPASS;
    	}

		sddiv = 1;
    }

	m_pSDCardRegs->sd_clock = (tmp | sddiv);

	// Adjust bus mode based on clock speed
	if (*pdwRate > 400000)
	{
		// Switch to push-pull mode
		m_pSDCardRegs->sd_power &= ~SD_OPENDRAIN_EN;
	}
	else
	{
		m_pSDCardRegs->sd_power |= SD_OPENDRAIN_EN;
	}

    DEBUGMSG(SDHC_CLOCK_ZONE,
		(TEXT("SDHC - Clock set to %dHz (actual %d) (%x/%x/%x)\r\n"), *pdwRate,
		(sdclk / (2 * (sddiv + 1))), sdclk, sddiv, m_pSDCardRegs->sd_clock));
}

//------------------------------------------------------------------------------
//
// sdCardSetInterface
//
// Set up the controller according to the interface parameters
//
void sdCardController::sdCardSetInterface(PSD_CARD_INTERFACE pInterface)
{
    DEBUGCHK(pInterface);

    if (pInterface->InterfaceMode == SD_INTERFACE_SD_MMC_1BIT) 
    {
        DEBUGMSG(SDHC_INIT_ZONE, 
            (TEXT("SDHC - Setting for 1 bit mode\r\n")));
		m_pSDCardRegs->sd_clock &= ~SD_WIDEBUSMODE_EN;
    } 
    else if (SD_INTERFACE_SD_4BIT == pInterface->InterfaceMode) 
    {
        DEBUGMSG(SDHC_INIT_ZONE, 
            (TEXT("SHCSDSlotOptionHandler - Setting for 4 bit mode\r\n")));
		m_pSDCardRegs->sd_clock |= SD_WIDEBUSMODE_EN;
    } 
    else 
    {
        DEBUGCHK(FALSE);
    }

    sdCardSetClockRate(&pInterface->ClockRate);
}

//------------------------------------------------------------------------------
//
// sdPowerUp
//
// Power up or down the SD card controller
//
void sdCardController::sdPowerUp(BOOL powerUp)
{
	DEBUGMSG(SDHC_INIT_ZONE,
		(TEXT("sdCardController::sdPowerUp\r\n")));
	sdCardEnableClocks(powerUp);
}

//------------------------------------------------------------------------------
//
// UpdateDevicePowerState
//
// Update device power state
//
VOID sdCardController::UpdateDevicePowerState(void) 
{

	if (m_PowerState == D4)
	{
		// Restore slot power
		sdCardEnableClocks(TRUE);
		sdHwSlotPowerControl(m_pGPIORegs, TRUE);
	}
	else
	{
		; // TBD
		sdHwSlotPowerControl(m_pGPIORegs, FALSE);
		sdCardEnableClocks(FALSE);
	}
    if( m_hParentBus ) {
        SetDevicePowerState( m_hParentBus, m_PowerState, NULL );
    }
}

//------------------------------------------------------------------------------
//
// ProcessRequest
//
// Process a SD request
//
SD_API_STATUS sdCardController::ProcessRequest(PSD_BUS_REQUEST pRequest)
{
	// Save transfer parameters
	m_CommandCode = pRequest->CommandCode;
	m_CommandArgument = pRequest->CommandArgument;
	m_ResponseType = pRequest->CommandResponse.ResponseType;
	m_TransferClass = pRequest->TransferClass;

	// If the controller is busy, then exit with error status
	if ((m_pSDCardRegs->sd_status & (SD_RX_INPROGRESS | SD_TX_INPROGRESS |
		SD_CMD_INPROGRESS)) != 0)
	{
        DEBUGMSG(SDHC_BUSY_STATE_ZONE, (TEXT("Controller is busy before command sent!\r\n")));
		sdCardClearController();
		return SD_API_STATUS_UNSUCCESSFUL;
	}

	// Clear any latched SD card controller status and clear state machines
	sdCardClearController();

//    DEBUGMSG(1, (TEXT("ProcessRequest (Cmd=%d, Arg=0x%08x, RespType=%d, Data=0x%x <%dx%d>) starts\r\n"), 
//        m_CommandCode, m_CommandArgument, m_ResponseType, (m_TransferClass==SD_COMMAND)?FALSE:TRUE, pRequest->NumBlocks, pRequest->BlockSize ) );

	m_read_iterations = 0;
	m_bytesleft = pRequest->BlockSize * pRequest->NumBlocks;
#ifdef USE_SINGLE_SECTOR_BURSTS
	if (m_CommandCode == 18)
	{
		m_CommandCode = 17;
		m_read_iterations = pRequest->NumBlocks;
		pRequest->NumBlocks = 1;
	}
#endif
	m_readbuff = (DWORD) pRequest->pBlockBuffer;

	// Setup command state machine and enable CMD state machine interrupts
	sdCmdSMSetup(m_CommandCode, m_CommandArgument, m_ResponseType);

	m_dataStatus = SD_API_STATUS_SUCCESS;
	if ((m_TransferClass == SD_READ) || (m_TransferClass == SD_WRITE))
    {
		// Setup data state machine
		m_NumBlocks = pRequest->NumBlocks;
		m_BlockSize = pRequest->BlockSize;
		m_pBlockBuffer = pRequest->pBlockBuffer;
		m_xferSize = m_BlockSize * m_NumBlocks;
		sdDataSMSetup(pRequest->DataAccessClocks);

		// Enable command and data state machine and interrupt
		if (m_TransferClass == SD_READ)
		{
			// Read commands start the data state machine now, while writes are
			// delayed until the command response is ready
			m_pSDCardRegs->sd_dctrl |= SD_DATATRANSFER_EN;
		}

		m_dataStatus = SD_API_STATUS_PENDING;
	}

	// Enable command state
	m_cmdStatus = SD_API_STATUS_PENDING;
	m_pSDCardRegs->sd_cmd |= SD_CPST_EN;

	return SD_API_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
//
// sdCmdSMSetup
//
// Setup command state machine
//
void sdCardController::sdCmdSMSetup(UINT32 commandCode,
		                            UINT32 commandArgument,
					                UINT16 responseType)
{
	UINT32 tmp;

	// 
	m_BusyCheckFlag = FALSE;

	// Write arg register first
	m_pSDCardRegs->sd_arg = commandArgument;

	// Save command word
	tmp = commandCode;

	// Setup response
    switch(responseType)
    {
	case NoResponse:
		// No response expected
		m_SD0CmdMask = SD_CMD_SENT;
		break;

	case ResponseR2:
		// Long response required
		tmp |= (SD_LONGRESP_EN | SD_RESPONSE);
		m_SD0CmdMask = (SD_CMD_TIMEOUT | SD_CMD_CRC_FAIL | SD_CMD_RESP_RECEIVED);
        break;

    case ResponseR1b:
		// Will need to verify program status after write
//		tmp |= SD_INTERRUPT_EN;
		if (commandCode != 7)
		{
			m_BusyCheckFlag = TRUE;
		}
	default:
		// Short response required
		tmp |= SD_RESPONSE;
		m_SD0CmdMask = (SD_CMD_TIMEOUT | SD_CMD_CRC_FAIL | SD_CMD_RESP_RECEIVED);

		// Write commands require a busy status check on DAT0, which the
		// hardware doesn't support
		if (commandCode == 24)
		{
			m_BusyCheckFlag = TRUE;
		}
		break;
    }

	// Update command register
	m_pSDCardRegs->sd_cmd = tmp;

	// Clear latched command statuses and enable command state
	// machine interrupts
    m_pSDCardRegs->sd_mask0 = m_SD0CmdMask;
}

//------------------------------------------------------------------------------
//
// sdGetBlockSize
//
// Return the blocksize field for the data control state register
//
UINT32 sdCardController::sdGetBlockSize(ULONG blockSize)
{
	UINT32 ret;

	switch (blockSize)
	{
	case 1:
		ret = SD_BLKSIZE_1BYTE;
		break;
	case 2:
		ret = SD_BLKSIZE_2BYTES;
		break;
	case 4:
		ret = SD_BLKSIZE_4BYTES;
		break;
	case 8:
		ret = SD_BLKSIZE_8BYTES;
		break;
	case 16:
		ret = SD_BLKSIZE_16BYTES;
		break;
	case 32:
		ret = SD_BLKSIZE_32BYTES;
		break;
	case 64:
		ret = SD_BLKSIZE_64BYTES;
		break;
	case 128:
		ret = SD_BLKSIZE_128BYTES;
		break;
	case 256:
		ret = SD_BLKSIZE_256BYTES;
		break;
	case 512:
	default:
		ret = SD_BLKSIZE_512BYTES;
		break;
	case 1024:
		ret = SD_BLKSIZE_1024BYTES;
		break;
	case 2048:
		ret = SD_BLKSIZE_2048BYTES;
		break;
	}

	return ret;
}

//------------------------------------------------------------------------------
//
// sdDataSMSetup
//
// Setup data state machine
//
void sdCardController::sdDataSMSetup(DWORD timeout)
{
	UINT32 tmp;

	// Set the number of blocks to transfer
	m_pSDCardRegs->sd_dlen = m_xferSize;

	// Setup data interrupt mask
#if 0
	m_SD1DataMask = (SD_DATA_CRC_FAIL | SD_DATA_TIMEOUT | SD_DATA_END |
		SD_STARTBIT_ERR); // | SD_DATABLK_END);
#else
	m_SD1DataMask = (SD_DATA_CRC_FAIL | SD_DATA_TIMEOUT | 0 |
		SD_STARTBIT_ERR | SD_DATABLK_END);
#endif

	// Set blocksize and enable DMA
	tmp = (SD_DMA_EN | sdGetBlockSize(m_BlockSize));

	// Set direction based on transfer mode
	if (m_TransferClass == SD_READ)
	{
		// Enable read interrupts and set direction for read
		m_SD1DataMask |= SD_FIFO_RXDATA_OFLOW;
		tmp |= SD_DIR_FROMCARD;

		// Setup DMA direction
		m_dmaStp.SrcInc = 0;
		m_dmaStp.DestInc = 1;
		m_dmaStp.perFlowSource = DMAC_CHAN_FLOW_P_P2M;
	}
	else
	{
		// Enable write interrupts
		m_SD1DataMask |= SD_FIFO_TXDATA_UFLOW;

		// Move data to DMA area
		SDPerformSafeCopy((void *) m_dmaBuffVirt, m_pBlockBuffer,
			m_xferSize);

		// Setup DMA direction
		m_dmaStp.SrcInc = 1;
		m_dmaStp.DestInc = 0;
		m_dmaStp.perFlowSource = DMAC_CHAN_FLOW_P_M2P;
	}
	dmaChanConfig(m_dmaCtl, &m_dmaStp);

	// Setup data state machine and enable interrupts
	m_pSDCardRegs->sd_dctrl = tmp;
    m_pSDCardRegs->sd_mask0 |= m_SD1DataMask;

	// Set data timeout
	m_pSDCardRegs->sd_dtimer = (m_dataTimeout + timeout);

	// Read or write?
	if (m_TransferClass == SD_READ)
	{
		// Submit buffer to hardware
		dmaTranEntry(m_dmaCtl, (DWORD) &SDCARD->sd_fifo[0],
			(DWORD) m_dmaBuffPhy, 0);
	}
	else
	{
		// Submit buffer to hardware
		dmaTranEntry(m_dmaCtl, (DWORD) m_dmaBuffPhy,
			(DWORD) &SDCARD->sd_fifo[0], 0);
	}
}

//------------------------------------------------------------------------------
//
// sdRespCopy
//
// Copy the response from the hardware
//
void sdCardController::sdRespCopy(PUCHAR buff)
{
	UINT32 tmp;

	if ((m_pSDCardRegs->sd_status & SD_CMD_RESP_RECEIVED) != 0)
	{
		// No response
		return;
	}

	tmp = m_pSDCardRegs->sd_resp [0];

	switch(m_ResponseType)
    {
	case NoResponse:
		break;

    case ResponseR1b:
    case ResponseR1:
		*(buff + 0) = (BYTE) (START_BIT | TRANSMISSION_BIT | m_pSDCardRegs->sd_respcmd);
		*(buff + 1) = (BYTE) (tmp >> 0);
		*(buff + 2) = (BYTE) (tmp >> 8);
		*(buff + 3) = (BYTE) (tmp >> 16);
		*(buff + 4) = (BYTE) (tmp >> 24);
		*(buff + 5) = (END_RESERVED | END_BIT);
		break;

    case ResponseR2:
		*(buff + 12) = (BYTE) (tmp >> 0);
		*(buff + 13) = (BYTE) (tmp >> 8);
		*(buff + 14) = (BYTE) (tmp >> 16);
		*(buff + 15) = (BYTE) (tmp >> 24);
		tmp = m_pSDCardRegs->sd_resp [1];
		*(buff + 8) = (BYTE) (tmp >> 0);
		*(buff + 9) = (BYTE) (tmp >> 8);
		*(buff + 10) = (BYTE) (tmp >> 16);
		*(buff + 11) = (BYTE) (tmp >> 24);
		tmp = m_pSDCardRegs->sd_resp [2];
		*(buff + 4) = (BYTE) (tmp >> 0);
		*(buff + 5) = (BYTE) (tmp >> 8);
		*(buff + 6) = (BYTE) (tmp >> 16);
		*(buff + 7) = (BYTE) (tmp >> 24);
		tmp = m_pSDCardRegs->sd_resp [3];
		*(buff + 0) = (BYTE) (tmp >> 0);
		*(buff + 1) = (BYTE) (tmp >> 8);
		*(buff + 2) = (BYTE) (tmp >> 16);
		*(buff + 3) = (BYTE) (tmp >> 24);
		break;

    case ResponseR3:
    case ResponseR4:
		*(buff + 0) = (BYTE) (START_BIT | TRANSMISSION_BIT | START_RESERVED);
		*(buff + 1) = (BYTE) (tmp >> 0);
		*(buff + 2) = (BYTE) (tmp >> 8);
		*(buff + 3) = (BYTE) (tmp >> 16);
		*(buff + 4) = (BYTE) (tmp >> 24);
		*(buff + 5) = (END_RESERVED | END_BIT);
		break;

    case ResponseR5:                
    case ResponseR6:
		*(buff + 0) = (BYTE) (START_BIT | TRANSMISSION_BIT | m_pSDCardRegs->sd_respcmd);
		*(buff + 1) = (BYTE) (tmp >> 0);
		*(buff + 2) = (BYTE) (tmp >> 8);
		*(buff + 3) = (BYTE) (tmp >> 16);
		*(buff + 4) = (BYTE) (tmp >> 24);
		*(buff + 5) = END_BIT;
		break;
	
	default:
		break;
	}
}

//------------------------------------------------------------------------------
//
// CompleteBusDataRequest
//
// Complete a data bus request
//
void sdCardController::CompleteBusDataRequest(SD_API_STATUS status,
											  PSD_BUS_REQUEST pRequest)
{
	// Complete request
	IndicateBusRequestComplete(pRequest, status);
}

//********************************************************************
// SD card driver callbacks
//********************************************************************

//------------------------------------------------------------------------------
//
// SDHCInitializeImpl
//
// Initialization callback
//
SD_API_STATUS sdCardController::SDHCInitializeImpl(void)
{
	DWORD threadID;
	PHYSICAL_ADDRESS pa;
	SD_API_STATUS status = SD_API_STATUS_INSUFFICIENT_RESOURCES;

    DEBUGMSG(SDHC_INIT_ZONE, (TEXT("sdCardController::SDHCInitializeImpl\r\n")));

	// All threads are initially non-existant
	m_detectThreadDone = TRUE;
	m_SD0ThreadDone = TRUE;

	// Map device registers
	pa.QuadPart = SD_BASE;
	m_pSDCardRegs = (SDCARD_REGS_T *) MmMapIoSpace(pa,
		sizeof (SDCARD_REGS_T), FALSE);
	pa.QuadPart = CLK_PM_BASE;
	m_pClkPwrRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);
	pa.QuadPart = SIC2_BASE;
	m_pINTCRegs = (INTC_REGS_T *) MmMapIoSpace(pa,
		sizeof (INTC_REGS_T), FALSE);
	pa.QuadPart = GPIO_BASE;
	m_pGPIORegs = (GPIO_REGS_T *) MmMapIoSpace(pa,
		sizeof (GPIO_REGS_T), FALSE);
	if ((m_pSDCardRegs == NULL) || (m_pClkPwrRegs == NULL) ||
		(m_pINTCRegs == NULL) || (m_pGPIORegs == NULL))
	{
        DEBUGMSG(SDHC_ERROR_ZONE, 
			(TEXT("sdCardController: ERROR: Failed to map registers\r\n")));
		goto cleanup;
	}

	// Setup SD card interface, this is setup, but will not be restored to
	// the original state if de-intialized later. Clocks are not enabled yet.
	m_pClkPwrRegs->clkpwr_ms_ctrl = (CLKPWR_MSCARD_MSDIO_PU_EN |
		CLKPWR_MSCARD_SDCARD_DIV(1));

	// Enable clock for SD block
	sdCardEnableClocks(TRUE);

	// Initial setup of SD card controller
	sdCardClearController();
	m_pSDCardRegs->sd_power = (SD_OPENDRAIN_EN | SD_POWER_OFF_MODE);
	m_pSDCardRegs->sd_clock = (SD_CLKDIV_MASK | SD_SDCLK_PWRSAVE);

	// Disable clock for SD block until a card is inserted
	sdCardEnableClocks(FALSE);

	// Setup GPIO1 as an interrupt pin with initial active low level interrupt response
	m_pINTCRegs->atr &= ~GPIO_IRQ_MASK; // Level sensitive
	m_pINTCRegs->apr &= ~GPIO_IRQ_MASK; // low sensitive

	// Map sysIntr value to the card detect interrupt
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &m_dwSDCDIrq,
		sizeof(m_dwSDCDIrq), &m_dwSysintrCD, sizeof(m_dwSysintrCD), NULL))
    {
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDCardEventInit: Error obtaining SD detect SYSINTR value!\r\n")));
        m_dwSysintrCD = SYSINTR_UNDEFINED;
        goto cleanup;
    }

    // Create event for the interrupt
    m_hCardInsertInterruptEvent = CreateEvent(NULL, FALSE, FALSE,NULL);
    if (m_hCardInsertInterruptEvent == NULL) {
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDCardEventInit: Error creating SD detect event!\r\n")));
        goto cleanup;
    }

    // initialize the card insertion interrupt event
    if (!InterruptInitialize (m_dwSysintrCD, m_hCardInsertInterruptEvent,
		NULL, 0)) {
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDCardEventInit: Error initializing SD detect interrupt!\r\n")));
        goto cleanup;
    }

	// Start card detection thread
    m_DetectThreadKill = FALSE;
	m_detectThreadDone = FALSE;
	m_hCardInsertInterruptThread =  CreateThread(NULL, 0,
		(LPTHREAD_START_ROUTINE) StartDetectThread, this, 0, &threadID);
	if (m_hCardInsertInterruptThread == NULL)
	{
		DEBUGMSG(SDHC_ERROR_ZONE,
			(TEXT("SDHCInitializeImpl: Error creating detect thread!\r\n")));
        goto cleanup;
    }

	// Enable SD card detect interrupt
	InterruptDone(m_dwSysintrCD);

	// Create sysIntrs for SD card IRQs
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &m_dwIrq0, sizeof(m_dwIrq0),
		&m_dwSysIntr0, sizeof(m_dwSysIntr0), NULL))
    {
        DEBUGMSG(SDHC_ERROR_ZONE, 
			(TEXT("sdCardController: ERROR: Failed to request the SD0 sysintr.\r\n")));

        m_dwSysIntr0 = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	// Create SD card events
	m_eventSD0 = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (m_eventSD0 == NULL)
	{
		DEBUGMSG(SDHC_ERROR_ZONE, 
			(TEXT("sdCardController: ERROR: Failed to create SD0 handler event.\r\n")));
		m_eventSD0 = INVALID_HANDLE_VALUE;
        goto cleanup;
	}

	// Bind interrupt to events
	if (InterruptInitialize(m_dwSysIntr0, m_eventSD0, NULL, 0) == FALSE)
	{
		// Cannot initialize interrupt
		DEBUGMSG(SDHC_ERROR_ZONE, 
			(TEXT("sdCardController: ERROR: Cannot initialize SD0 interrupt\r\n")));
        goto cleanup;
	}

	status = SD_API_STATUS_SUCCESS;

cleanup:
    if (!SD_API_SUCCESS(status))
	{
        SDHCDeinitializeImpl();
    }

    return status;
}

//------------------------------------------------------------------------------
//
// SDHCDeinitializeImpl
//
// De-initialization callback
//
SD_API_STATUS sdCardController::SDHCDeinitializeImpl(void)
{
	int to;

	// Stop SD controller state machines
	if (m_pSDCardRegs != NULL)
	{
		m_pSDCardRegs->sd_cmd = 0;
		m_pSDCardRegs->sd_dctrl = 0;
	}

	// Shutdown hardware if enabled
	if (m_hardwareInitialized != FALSE)
	{
		sdHwShutdown();
	}

	// Disable power to the card
	if (m_pGPIORegs != NULL)
	{
		sdHwSlotPowerControl(m_pGPIORegs, FALSE);
	}

	// Return SD command and data interrupt resources
	if (m_eventSD0 != NULL)
	{
		CloseHandle(m_eventSD0);
		m_eventSD0 = NULL;
	}
	if (m_dwSysIntr0 != SYSINTR_UNDEFINED)
	{
		InterruptDisable(m_dwSysIntr0);
		KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &m_dwSysIntr0,
            sizeof(m_dwSysIntr0), NULL, 0, NULL);
		m_dwSysIntr0 = SYSINTR_UNDEFINED;
	}

	// Stop the card detect thread
	m_DetectThreadKill = TRUE;
	to = 20;
	while ((to > 0) && (m_detectThreadDone == FALSE))
	{
		SetEvent(m_hCardInsertInterruptEvent);
		Sleep(50);
	}

	// Close event handle
	if (m_hCardInsertInterruptEvent != NULL)
	{
		CloseHandle(m_hCardInsertInterruptEvent);
		m_hCardInsertInterruptEvent = NULL;
	}

	// Disable interrupt
	if (m_dwSDCDIrq != SYSINTR_UNDEFINED)
	{
		InterruptDisable(m_dwSDCDIrq);
		KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &m_dwSDCDIrq,
            sizeof(m_dwSDCDIrq), NULL, 0, NULL);
		m_dwSDCDIrq = SYSINTR_UNDEFINED;
	}

	// Unmap register space
	if (m_pGPIORegs != NULL)
	{
        MmUnmapIoSpace((PVOID) m_pGPIORegs, sizeof (GPIO_REGS_T));
		m_pGPIORegs = NULL;
	}
	if (m_pINTCRegs != NULL)
	{
        MmUnmapIoSpace((PVOID) m_pINTCRegs, sizeof (INTC_REGS_T));
		m_pINTCRegs = NULL;
	}
	if (m_pClkPwrRegs != NULL)
	{
        MmUnmapIoSpace((PVOID) m_pClkPwrRegs, sizeof (CLKPWR_REGS_T));
		m_pClkPwrRegs = NULL;
	}
	if (m_pSDCardRegs != NULL)
	{
        MmUnmapIoSpace((PVOID) m_pSDCardRegs, sizeof (SDCARD_REGS_T));
		m_pSDCardRegs = NULL;
	}

	return SD_API_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
//
// SDHCCancelIoHandlerImpl
//
// Cancel current request
//
BOOLEAN sdCardController::SDHCCancelIoHandlerImpl(UCHAR Slot,
												  PSD_BUS_REQUEST pRequest)
{
	// Does nothing
	DEBUGMSG(SDHC_INIT_ZONE,  (TEXT("SDHCCancelIoHandlerImpl\r\n")));
	return TRUE;
}

//------------------------------------------------------------------------------
//
// SDHCBusRequestHandlerImpl
//
// Handle a SD bus request
//
SD_API_STATUS sdCardController::SDHCBusRequestHandlerImpl(PSD_BUS_REQUEST pRequest)
{
    SD_API_STATUS   status;
    BOOL            fWriteTransferMode = FALSE;

    // acquire the device lock to protect from device removal
    SDHCDAcquireHCLock(m_pHCContext);

	// Trap command 12
#ifdef USE_SINGLE_SECTOR_BURSTS
	if (pRequest->CommandCode == 12)
	{
		status = SD_API_STATUS_SUCCESS;
		IndicateBusRequestComplete(pRequest, status);
		goto EXIT;
	}
#endif
	
	status = ProcessRequest(pRequest);
    if(!SD_API_SUCCESS(status))
    {
        DEBUGMSG(SDHC_ERROR_ZONE, (TEXT("SDHCBusRequestHandlerImpl() - Error sending command:0x%02x\r\n"),
			pRequest->CommandCode));
        goto EXIT;      
    }

	// we will handle the response on another thread
    status = SD_API_STATUS_PENDING;

 EXIT:
    SDHCDReleaseHCLock(m_pHCContext);
    return status;
}

//------------------------------------------------------------------------------
//
// SDHCSlotOptionHandlerImpl
//
// Sd card slot function handler
//
SD_API_STATUS sdCardController::SDHCSlotOptionHandlerImpl(UCHAR SlotNumber, 
                                                          SD_SLOT_OPTION_CODE Option, 
                                                          PVOID pData,
                                                          ULONG OptionSize)
{
    SD_API_STATUS status = SD_API_STATUS_SUCCESS;

    SDHCDAcquireHCLock(m_pHCContext);

    switch (Option)
	{
	case SDHCDSetSlotPower:
		{
			//// this platform has 3.2V tied directly to the slot
			DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SHCSDSlotOptionHandler : SDHCDSetSlotPower\r\n")));
	    }
		break;

	case SDHCDSetSlotInterface:
		{
			PSD_CARD_INTERFACE pInterface = (PSD_CARD_INTERFACE) pData;
	        sdCardSetInterface(pInterface);
	    }
		break;

	case SDHCDGetSlotInfo:
		if( OptionSize != sizeof(SDCARD_HC_SLOT_INFO) || pData == NULL )
	    {
		    status = SD_API_STATUS_INVALID_PARAMETER;
			DEBUGMSG(SDHC_ERROR_ZONE, 
				(TEXT("SHCSDSlotOptionHandler : SDHCDGetSlotInfo\r\n")));
	    }
		else
	    {
			PSDCARD_HC_SLOT_INFO pSlotInfo = (PSDCARD_HC_SLOT_INFO)pData;
			SDHCDSetSlotCapabilities( pSlotInfo, 
	            SD_SLOT_SD_1BIT_CAPABLE |
		        SD_SLOT_SD_4BIT_CAPABLE);

			SDHCDSetVoltageWindowMask(pSlotInfo, (SD_VDD_WINDOW_3_0_TO_3_1 | SD_VDD_WINDOW_3_1_TO_3_2 |
				SD_VDD_WINDOW_3_2_TO_3_3)); 

	        // Set optimal voltage
		    SDHCDSetDesiredSlotVoltage(pSlotInfo, SD_VDD_WINDOW_3_1_TO_3_2);     

	        // Set maximum supported clock rate
		    SDHCDSetMaxClockRate(pSlotInfo, m_dwMaxClockRate);

			// set power up delay
	        SDHCDSetPowerUpDelay(pSlotInfo, 100); 
		}
	    break;

	case SDHCDSetSlotPowerState:
		  DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SHCSDSlotOptionHandler - SetSlotPowerState : on slot %d\r\n"),SlotNumber));
	    if( pData == NULL || OptionSize != sizeof(CEDEVICE_POWER_STATE) )
		  {
			  status = SD_API_STATUS_INVALID_PARAMETER;
	    }
		  else
	   {
		      PCEDEVICE_POWER_STATE pcps = (PCEDEVICE_POWER_STATE) pData;
			  m_PowerState = *pcps;
			  UpdateDevicePowerState();
	    }
		  break;

	case SDHCDGetSlotPowerState:
		DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SHCSDSlotOptionHandler - GetSlotPowerState : on slot %d\r\n"),SlotNumber));
    if( pData == NULL || OptionSize != sizeof(CEDEVICE_POWER_STATE) )
    {
        status = SD_API_STATUS_INVALID_PARAMETER;
    }
    else
    {
        PCEDEVICE_POWER_STATE pcps = (PCEDEVICE_POWER_STATE) pData;
        *pcps = m_PowerState;
    }
    break;

	case SDHCDGetWriteProtectStatus:
    {
        PSD_CARD_INTERFACE pInterface = (PSD_CARD_INTERFACE) pData;
        pInterface->WriteProtected = sdHwCardWriteProtected(m_pGPIORegs);
	    DEBUGMSG(SDHC_INIT_ZONE, (TEXT("SHCSDSlotOptionHandler - SDHCDGetWriteProtectStatus : on slot %d (protect = %d)\r\n"),
			SlotNumber, pInterface->WriteProtected));
    }
	break;

	case SDHCDQueryBlockCapability:
    {
        PSD_HOST_BLOCK_CAPABILITY pBlockCaps = 
            (PSD_HOST_BLOCK_CAPABILITY)pData;

        DEBUGMSG(SDHC_INIT_ZONE, 
			(TEXT("SHCSDSlotOptionHandler: Read Block Length: %d , Requested read Blocks: %d\r\n"), 
            pBlockCaps->ReadBlockSize, pBlockCaps->ReadBlocks));
        DEBUGMSG(SDHC_INIT_ZONE, 
            (TEXT("SHCSDSlotOptionHandler: Write Block Length: %d , Requested write Blocks: %d\r\n"), 
            pBlockCaps->WriteBlockSize, pBlockCaps->WriteBlocks));

		// Up to a 2K blocksize
		if (pBlockCaps->ReadBlockSize < 1)
		{
			pBlockCaps->ReadBlockSize = 1;
		}
		if (pBlockCaps->ReadBlockSize > 1024)
		{
			pBlockCaps->ReadBlockSize = 1024;
		}
		if (pBlockCaps->WriteBlockSize < 1)
		{
			pBlockCaps->WriteBlockSize = 1;
		}
		if (pBlockCaps->WriteBlockSize > 1024)
		{
			pBlockCaps->WriteBlockSize = 1024;
		}

		pBlockCaps->ReadBlocks = 64;
		pBlockCaps->WriteBlocks = 64;
	}
    break;

	default:
		status = SD_API_STATUS_INVALID_PARAMETER;
		break;
	}

    SDHCDReleaseHCLock(m_pHCContext);
    return status;
}
