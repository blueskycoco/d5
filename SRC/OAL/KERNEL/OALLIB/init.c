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
// init.c
//
// Kernel initialization functions
//

#include <bsp.h>
#include <profiler.h>
#include "bsp_serial.h"
#include "phy3250_board.h"
#include "clkpwr_support.h"
#include "peripheralmapping.h"


extern BOOL OEMSetupRTC(VOID);

//------------------------------------------------------------------------------
//
// phyhwdesc
//
// This structure contains the board configuration data including the MAC
// address and SDRAM configuration.
//
PHY_HW_T phyhwdesc;

//------------------------------------------------------------------------------
//
// Phy3250EnumExtensionDRAM
//
// Returns extended data sections
//
static DWORD Phy3250EnumExtensionDRAM(PMEMORY_SECTION pMemSections, DWORD cMemSections)
{
	DWORD secs = 0;
	RETAILMSG(1, (TEXT("enum extension DRAM begin\r\n")));
    OALMSG(OAL_FUNC, (L"+OEMEnumExtensionDRAM\r\n"));
	if ((phyhwdesc.dramcfg & PHYHW_DRAM_SIZE_MASK) == PHYHW_DRAM_SIZE_128M)
	{
		// Add another 64MBytes of RAM
		pMemSections[0].dwFlags = 0;
		pMemSections[0].dwStart = IMAGE_WINCE_EXT_DRAM;
		pMemSections[0].dwLen = IMAGE_WINCE_EXT_DRAM_SIZE;
		secs = 1;

		OALMSG(OAL_FUNC, (L"+OEMEnumExtensionDRAM - ADDING 64M RAM\r\n"));
	}

	return secs;
}

//------------------------------------------------------------------------------
//
// OEMInitDebugSerial
//
// Initializes serial port
//
void OEMInitDebugSerial (void)
{
	(void) OEMDebugInit();
}

//------------------------------------------------------------------------------
//
// OEMInit
//
// This is Windows CE OAL initialization function. It is called from kernel
// after basic initialization is made.
//
void OEMInit()
{
    BSP_ARGS *pArgs;
	BOOL cleanboot = FALSE;

    pOEMIsProcessorFeaturePresent = OALIsProcessorFeaturePresent;
	RETAILMSG(1, (TEXT("oeminit begin\r\n")));
    // Give the kernel access to the profiling functions
#ifdef IMGPROFILFER
#if IMGPROFILFER==1
    g_pOemGlobal->pfnProfileTimerEnable = OEMProfileTimerEnable;
    g_pOemGlobal->pfnProfileTimerDisable = OEMProfileTimerDisable;
#endif
#endif

    // Pointer to function that extends memory
    g_pOemGlobal->pfnEnumExtensionDRAM = Phy3250EnumExtensionDRAM;

	// Set current clock rate
	g_oalIoCtlClockSpeed = clkpwr_get_base_clock_rate(CLKPWR_ARM_CLK);
	g_oalIoCtlClockSpeed = g_oalIoCtlClockSpeed / (1000000);
 //-------------------------------------------------------------------------
	//InitPeripheralMapping(FALSE);
    //----------------------------------------------------------------------
    // Initialize OAL log zones
    //----------------------------------------------------------------------
    OALLogSetZones(
               //(1<<OAL_VERBOSE)  |
               //(1<<OAL_TIMER)    |
               //(1<<OAL_INFO)     |
               (1<<OAL_ERROR)    |
               //(1<<OAL_WARN)     |
               //(1<<OAL_IOCTL)    | 
               //(1<<OAL_ETHER)    |
               //(1<<OAL_KITL)     |
               //(1<<OAL_FUNC)     |
               //(1<<OAL_INTR)     |
               //(1<<OAL_RTC)      |
               0);

    //----------------------------------------------------------------------
    // Initialize cache globals
    //----------------------------------------------------------------------
	RETAILMSG(1, 
				(TEXT("OutputStreamContext: OALCacheGlobalsInit begin\r\n")));
    OALCacheGlobalsInit();

    // Get pointer to expected boot args location
    pArgs = OALPAtoCA(IMAGE_SHARE_ARGS_PA);
    if ((pArgs->header.signature == OAL_ARGS_SIGNATURE) ||
        (pArgs->header.oalVersion == OAL_ARGS_VERSION) ||
        (pArgs->header.bspVersion == BSP_ARGS_VERSION))
	{
        OALMSG(OAL_INFO, (
            L"INFO: OEMInit: Got board parameters\r\n"));

		// Copy global hardware info structure
		phyhwdesc = pArgs->phyhwdesc;
        cleanboot = pArgs->bCleanBootFlag;
	}
	else
	{
        OALMSG(OAL_ERROR, (
            L"ERROR: OEMInit: Error getting board parameters\r\n"));

		// Use some defaults
		phyhwdesc.dramcfg = (PHYHW_DRAM_TYPE_LPSDRAM |
			PHYHW_DRAM_SIZE_64M);
		phyhwdesc.syscfg = PHYHW_SDIO_POP;
		phyhwdesc.fieldvval = PHY_HW_VER_VAL;
	}
		RETAILMSG(1, 
				(TEXT("OutputStreamContext: DMA Init begin\r\n")));
	// Initialize DMA allocation driver - this should be done prior to
	// setting up interrupts to prevent virtual interrupt issues in the
	// interrupt handler
	dma_init();

	// Initialize interrupts
    if (!OALIntrInit()) {
        OALMSG(OAL_ERROR, (
            L"ERROR: OEMInit: failed to initialize interrupts\r\n"));
        goto cleanUp;
    }

		RETAILMSG(1, 
				(TEXT("OutputStreamContext: OALIntr init is OK!\r\n")));

	// Initialize system clock
    if (!OALTimerInit(1, 0, 0)) {
        OALMSG(OAL_ERROR, (
            L"ERROR: OEMInit: Failed to initialize system clock\r\n"
        ));
        goto cleanUp;
    }
	RETAILMSG(1, 
				(TEXT("OutputStreamContext: OALIntr timer is OK!\r\n")));
	
	// Setup the system time (RTC) and RTC alarm function
	if (OEMSetupRTC() == FALSE)
	{
        OALMSG(OAL_ERROR, (
            L"ERROR: OEMInit: Error setting system time\r\n"));
	}
	RETAILMSG(1, 
				(TEXT("OutputStreamContext: setup the system time is OK!\r\n")));
	
	// Get clock speed of ARM core
	g_oalIoCtlClockSpeed = clkpwr_get_base_clock_rate(CLKPWR_ARM_CLK);

	// Initialize KITL
		RETAILMSG(1, 
				(TEXT("OutputStreamContext: Init KITL begin\r\n")));
	KITLIoctl (IOCTL_KITL_STARTUP, NULL, 0, NULL, 0, NULL);

    // Force clean boot if requested from bootloader
//	if (cleanboot == TRUE) // TBD
	{
		// Clean boot was requested
	    NKForceCleanBoot();
		RETAILMSG(1, 
				(TEXT("OutputStreamContext: force clean boot!\r\n")));
	}

cleanUp:
	return;
}
