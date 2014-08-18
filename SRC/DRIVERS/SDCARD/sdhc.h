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
// sdhc.h
//
// SD card controller driver
//

#pragma once

#include <windows.h>
#include <ceddk.h>
#include <devload.h>
#include <pm.h>
#include <SDCardDDK.h>
#include <SDHCD.h>
#include <creg.hxx>
#include "sdhwctrl.h"
#include "lpc32xx_sdcard.h"
#include "lpc32xx_clkpwr.h"
#include "dmadrv.h"

// Debug zones
#define SDHC_INIT_ZONE                  0x1
#define SDHC_ERROR_ZONE                 0x2
#define SDHC_INTERRUPT_ZONE             0x4
#define SDHC_SEND_ZONE                  0x8
#define SDHC_RESPONSE_ZONE              0x10
#define SDHC_RECEIVE_ZONE               0x20
#define SDHC_CLOCK_ZONE                 0x40
#define SDHC_TRANSMIT_ZONE              0x80
#define SDHC_SDBUS_INTERACT_ZONE        0x100
#define SDHC_BUSY_STATE_ZONE            0x200

// Registry lookup information
#define SHC_DETECT_THREAD_PRIORITY_KEY           TEXT("DetectIstPriority")
#define SHC_DETECT_THREAD_PRIORITY_DEF           0x66
#define SHC_DATA_TIMEOUT_KEY                     TEXT("DataTimeOutClocks")
#define SHC_DATA_TIMEOUT_DEF                     0x3FFFF
#define SHC_FREQUENCY_MAX_KEY                    TEXT("MaxClockFrequency")
#define SHC_FREQUENCY_MAX_DEF                    25000000
#define SHC_SD0_THREAD_PRIORITY_KEY              TEXT("CmdIstPriority")
#define SHC_SD0_THREAD_PRIORITY_DEF              0x40
#define SHC_SD1_THREAD_PRIORITY_KEY              TEXT("DataIstPriority")
#define SHC_SD1_THREAD_PRIORITY_DEF              0x41
#define SHC_DMA_THREAD_PRIORITY_KEY              TEXT("DMAIstPriority")
#define SHC_DMA_THREAD_PRIORITY_DEF              0x42

#define START_BIT                           0x00
#define TRANSMISSION_BIT                    0x00
#define START_RESERVED                      0x3F
#define END_RESERVED                        0xFE
#define END_BIT                             0x01

// Enable the following define to enable a hardware check for the
// SD busy status on commands that require a R1b response type. If the
// define is used, signal GPI_02 must be connected to the SD card
// MS_DIO0 signal. If the define isn't used, a hard coded delay of
// SW_DELAY will occur after any command requiring an R1B response.
//#define USE_HW_CHECK
#define SW_DELAY 1

// Enable the following define to use single sector read functions.
// Single sector write functions, if used, should be enabled by setting
// the appropriate registry setting for the SDHC driver. This will
// convert multiple sector read operations for a loop of single sector
// operations
#define USE_SINGLE_SECTOR_BURSTS

// GPIO for card detect - if this is changed, verify that the associated
// activation level/polarity/mux/input registers are changed also in the
// code (if the GPIO is not in the same register set)
#define GPIO_IRQ_MASK _BIT(1)
// Mapped OALINTR value for card detect GPIO interrupt
#define MAPPED_INTROAL_VAL OAL_INTR_IRQ_GPIO_01

#define GET_PCONTROLLER_FROM_HCD(pHCDContext) \
    GetExtensionFromHCDContext(class sdCardController *, pHCDContext)

// SDHC hardware specific context
class sdCardController : public sdHwCtrl
{
public:
	//********************************************************************
	// Class constructor and destructor functions
	//********************************************************************

	// Constructor
	sdCardController(void);

	// Destructor
	~sdCardController();

	// Setup initial driver values - this function is called when the
	// driver is first instantiated or when the driver is destroyed, it
	// will setup initial values for all class objects
	void sdCardSetInitialValues(void);

	//********************************************************************
	// SDHC initialization and de-init functions. Sets up hardware and
	// functions that are used whether a card is installed or not.
	//********************************************************************

	// Read the registry settings used for the SDHC controller
    BOOL sdCardGetRegistrySettings(LPCTSTR pszActiveKey);

	// Initialize class and context
    DWORD sdInit(LPCTSTR pszActiveKey);

	// Free host context
    void sdCardFreeHostContext(void);

	//********************************************************************
	// Hardware setup functions - These functions are called whenever a
	// card is inserted or removed to initialize or de-init specific
	// system resources
	//********************************************************************

	// Hardware setup - sets up the hardware when a card is initially
	// inserted into the device
	BOOL sdHwSetup(void);

	// Shuts down the hardware when a card is removed
	void sdHwShutdown(void);

	//********************************************************************
	// System threads
	//********************************************************************

	// SD card detection and signal thread
	DWORD sdCDThread(void);

	// SD command thread - handles commands
	DWORD sdSD0Thread(void);

	// SD data thread - handles data transfer completion
//	DWORD sdSD1Thread(void);

	// SD DMA thread - handles data transfer
//	DWORD sdDMAThread(void);

	//********************************************************************
	// SD card controller support functions
	//********************************************************************

	// Clear/reset SD card controller to default idle state, may be used
	// to clear a function that failed or timed out
	void sdCardClearController(void);

	// Stops or starts the SD clocks (in the CLKPWR controller)
	void sdCardEnableClocks(BOOL enable);

	//  Set clock rate based on HC capability
	void sdCardSetClockRate(PDWORD pdwRate);

	// Set up the controller according to the interface parameters
	void sdCardSetInterface(PSD_CARD_INTERFACE pInterface);

	// Power up or down the SD card controller
    void sdPowerUp(BOOL powerUp);

	// Setup command state machine
	void sdCmdSMSetup(UINT32 commandCode,
		              UINT32 commandArgument,
					  UINT16 responseType);

	// Return the blocksize field for the data control state register
	UINT32 sdGetBlockSize(ULONG blockSize);

	// Setup data state machine
	void sdDataSMSetup(DWORD timeout);

	// Copies the response to the passed buffer
	void sdRespCopy(PUCHAR buff);

	// Sets up DMA lists
	void sdSetupDMALists(void);

	// Complete a data bus request
	void CompleteBusDataRequest(SD_API_STATUS status,
											  PSD_BUS_REQUEST pRequest);

	//********************************************************************
	// SD card controller support functions
	//********************************************************************

	// Processes a bus request and sets up the hardware to handle it
	SD_API_STATUS ProcessRequest(PSD_BUS_REQUEST pRequest);

	// Updates device and controller power state
	VOID UpdateDevicePowerState(void);

	//********************************************************************
	// SD card driver callbacks
	//********************************************************************
    SD_API_STATUS SDHCInitializeImpl();
    SD_API_STATUS SDHCDeinitializeImpl(void);
    BOOLEAN SDHCCancelIoHandlerImpl(UCHAR Slot,
		                            PSD_BUS_REQUEST pRequest);
    SD_API_STATUS SDHCBusRequestHandlerImpl(PSD_BUS_REQUEST pRequest);
    SD_API_STATUS SDHCSlotOptionHandlerImpl(UCHAR SlotNumber, 
                                            SD_SLOT_OPTION_CODE Option, 
                                            PVOID pData,
                                            ULONG OptionSize);
	void dumpRegs(SDCARD_REGS_T *pRegs);

	void sdCardHandleCommand(UINT32 reg, PSD_BUS_REQUEST pRequest);
	void sdCardHandleData(UINT32 reg, PSD_BUS_REQUEST pRequest);
	SD_API_STATUS sdCheckBusyStatus(void);

private:
	UINT32 m_CommandCode;
	UINT32 m_CommandArgument;
	UINT16 m_ResponseType;
	UINT16 m_TransferClass;
	ULONG m_NumBlocks;
	ULONG m_BlockSize;
	PUCHAR m_pBlockBuffer;
	int m_xferSize;
	UINT32 m_savedCardAddr;

	// Values used for program complete status check after any write function
	volatile BOOL m_BusyCheckFlag;

	// Instance counter
	static int instance;

	// Access to registers
	GPIO_REGS_T *m_pGPIORegs;                 // Pointer to GPIO registers
	INTC_REGS_T *m_pINTCRegs;                 // Pointer to interrupt controller registers
	SDCARD_REGS_T *m_pSDCardRegs;             // Pointer to SD card controller registers
	CLKPWR_REGS_T *m_pClkPwrRegs;             // Pointer to clock and power registers

	// SD host context and handle data
    HANDLE m_hParentBus;                      // Bus parent handle
    PSDCARD_HC_CONTEXT m_pHCContext;          // Host controller context
	BOOL m_registeredWithBusDriver;           // Bus driver registered flag

	// Card detect IRQ, sysIntr, events, threads, etc.
	DWORD m_dwSDCDIrq;                        // IRQ mapped to SD card detection
	DWORD m_dwSysintrCD;                      // Allocated sysIntr value for SD card detection interrupt
	HANDLE m_hCardInsertInterruptEvent;       // Card detect interrupt event handle
	volatile BOOL m_DetectThreadKill;         // Detect thread kill flag
	volatile BOOL m_detectThreadDone;         // Card detection thread done flag
	HANDLE m_hCardInsertInterruptThread;      // Card detect thread handle
	DWORD m_dwCardDetectIstThreadPriority;    // Card detection thread priority from registry
	BOOL m_cardInserted;

	// Comnmand IRQ, sysIntr, event, thread, etc.
	DWORD m_dwIrq0;                           // SD0 (command) mapped IRQ values
	DWORD m_dwSysIntr0;                       // Allocated sysIntr value for SD card command interrupt
	HANDLE m_eventSD0;                        // SD card command interrupt event handle
	volatile BOOL m_SD0ThreadKill;            // Command thread kill flag
	volatile BOOL m_SD0ThreadDone;            // Command thread done flag
	HANDLE m_SD0InterruptThread;              // Command thread handle
	DWORD m_SD0IstThreadPriority;             // Command thread priority from registry

	UINT32 m_SD0CmdMask;
	UINT32 m_SD1DataMask;

	// DMA driver handle and saved DMA info pointer, sysIntr value, IRQ
	// value, DMA buffer information, and DMA event
	DMASETUP_T m_dmaStp;                      // DMA setup parameters
	DMAINFO_T *m_dmainfo;                     // Saved pointer to DMA info
	DWORD m_dmaCtl;                           // Allocated DMA driver handle
	DWORD m_dmaBuffPhy;                       // Pointer to DMA physical buffer
	DWORD m_dmaBuffVirt;                      // Pointer to DMA virtual buffer
	DWORD m_dmaBuffSizeBytes;                 // Size of allocated DMA buffer area

	// Fetched data from the registry
	DWORD m_dataTimeout;                      // Data timeout clocks default
	DWORD m_dwSD0IstThreadPriority;           // SD0 (command) thread priority

	BOOL m_hardwareInitialized;               // Hardware initialized flag

	CEDEVICE_POWER_STATE m_PowerState;        // current extern power state
    DWORD                m_dwMaxClockRate;    // host controller's clock base

	SD_API_STATUS m_cmdStatus, m_dataStatus;
	DWORD m_read_iterations;
	DWORD m_readbuff, m_bytesleft;
};
