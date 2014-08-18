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
//------------------------------------------------------------------------------
//
// main.c
//
// Main bootloader functions
//
#include <windows.h>
#include <blcommon.h>
#include <oal.h>

//BSP Includes
#include <image_cfg.h>
#include <eboot.h>
#include <kitl_cfg.h>
#include <boot_cfg.h>
#include "boot_utilities.h"
#include "bsp_serial.h"
#include "boot_led.h"
#include "boot_flash.h"
#include "lpc32xx_rtc.h"
#include "lpc32xx_mstimer.h"
#include "lpc32xx_timer.h"
#include "phy3250_board.h"
#include "clkpwr_support.h"

//From BLCOMMON.C
void BootloaderMain (void);

// Define to use timer 0 for the MS timer
#define USETMR0

//------------------------------------------------------------------------------
//
// g_bootCfg
//
// This global variable is used to save boot configuration. It is readed from
// flash memory or initialized to default values if flash memory doesn't
// contain valid structure. It can be modified by user in bootloader
// configuration menu invoked by BLMenu.
//
BOOT_CFG g_bootCfg;

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
// g_eboot
//
// This global variable is used to save information about downloaded regions.
//
EBOOT_CONTEXT g_eboot;

//------------------------------------------------------------------------------
//
// cleanboot
//
// Flag used to indicate if a clean boot should occur.
//
BOOL cleanboot;

//------------------------------------------------------------------------------
//
// NAND FLASH flags
//
// Flag used to indicate if image should be placed in FLASH.
//
BOOL flashburnflag;
BOOL flashgood;
extern NAND_GEOM_T savedgeom;

//------------------------------------------------------------------------------
//
// Time_Setup
//
// Sets up the RTC and Millisecond timer for a 1 second count.
//
void Time_Setup (void)
{
#ifndef USETMR0
	UNS_32 rtcsecs;
	RTC_REGS_T *pRTCRegs;
	MSTIMER_REGS_T *pMSTRegs;

	pRTCRegs = (RTC_REGS_T *) OALPAtoVA((UINT32) RTC, FALSE);
	pMSTRegs = (MSTIMER_REGS_T *) OALPAtoVA((UINT32) MSTIMER, FALSE);

	// Enable MSTimer and set it to match every second
	pMSTRegs->mstim_match0 = RTCCLKRATE;
	pMSTRegs->mstim_mctrl = MSTIM_MCTRL_RST_COUNT0;
	pMSTRegs->mstim_ctrl = MSTIM_CTRL_RESET_COUNT;

	// Wait for RTC to 'tick' before setting MS Timer
	pRTCRegs->ctrl &= ~RTC_SW_RESET;
	rtcsecs = pRTCRegs->ucount;
	while (rtcsecs == pRTCRegs->ucount);
	pMSTRegs->mstim_ctrl = MSTIM_CTRL_COUNT_ENAB;
#else
	TIMER_CNTR_REGS_T *pTMR0Regs;
	UINT32 baseclk;

	pTMR0Regs = (TIMER_CNTR_REGS_T *) OALPAtoVA((UINT32) TIMER_CNTR0, FALSE);

	// Enable clock for timer 0
	clkpwr_clk_en_dis(CLKPWR_TIMER0_CLK, 1);

	// Enable counter to update count every 1mS with no match
	pTMR0Regs->tcr = 0;
	pTMR0Regs->ir = TIMER_CNTR_MTCH_BIT(0);
	pTMR0Regs->mcr = 0;
	pTMR0Regs->pc = 0;
	pTMR0Regs->tc = 0;

	// Get base clock for the timer
	baseclk = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);

	// Compute prescale divider to get the desired tick rate
	baseclk = baseclk / 1000;
	pTMR0Regs->pr = baseclk;

	// Enable timer
	pTMR0Regs->tcr = TIMER_CNTR_TCR_EN;
#endif
}

//------------------------------------------------------------------------------
//
// SpinForever
//
// Spins forever and flashes the LEDs.
//
void SpinForever()
{
   KITLOutputDebugString("Spin Forever\r\n");
   while(TRUE)
   {
        SetLEDValue(0xff);
        OALWaitMS(500);
        SetLEDValue(0x00);
        OALWaitMS(500);
   }
}

//------------------------------------------------------------------------------
//
// getbldata
//
// Gets the persistent bootloader configuration data
//
BOOL getbldata(VOID)
{
	BOOL usecfg;

    // Read saved configration and check for validity
   usecfg = BLReadBootCfg(&g_bootCfg);
    if ((usecfg == FALSE) ||
		(g_bootCfg.signature != BOOT_CFG_SIGNATURE) ||
        (g_bootCfg.version != BOOT_CFG_VERSION))
    {
        memset(&g_bootCfg, 0, sizeof(g_bootCfg));
        g_bootCfg.signature = BOOT_CFG_SIGNATURE;
        g_bootCfg.version = BOOT_CFG_VERSION;
        g_bootCfg.bootDevLoc.IfcType = Internal;
        g_bootCfg.bootDevLoc.BusNumber = 0;
        g_bootCfg.bootDevLoc.LogicalLoc = SD_BASE;
        g_bootCfg.kitlDevLoc.IfcType = Internal;
        g_bootCfg.kitlDevLoc.BusNumber = 0;
        g_bootCfg.kitlDevLoc.LogicalLoc = ETHERNET_BASE;
        g_bootCfg.kitlFlags |= OAL_KITL_FLAGS_DHCP|OAL_KITL_FLAGS_ENABLED;
        g_bootCfg.kitlFlags |= OAL_KITL_FLAGS_VMINI|OAL_KITL_FLAGS_EXTNAME;
        g_bootCfg.ipAddress = 0;
        g_bootCfg.ipMask = 0;
        g_bootCfg.ipRoute = 0;
        g_bootCfg.boot_to = 5;
		g_bootCfg.baudRate = BSP_UART_RATE;
        strcpy(g_bootCfg.binName, "nk.bin");
    }

	return usecfg;
}

//------------------------------------------------------------------------------
//
// lpc32xxMain
//
// This function is called from StartUp.s.
//
void lpc32xxMain(void)
{
   // Initialize LED port
   SetupLEDSGPIO();
   SetLEDValue(0x00);

	// Setup RTC and MSTimer for seconds counter
	Time_Setup();

	// Go to BLCOMMON
	BootloaderMain ();

   // Should never get here
   SpinForever();
}

//------------------------------------------------------------------------------
//
// OALGetTickCount
//
// Returns the number of milliSecond ticks since boot
//
UINT32 OALGetTickCount()
{
#ifndef USETMR0
	UINT64 tmp;
	UINT32 trtc, tmst;
	RTC_REGS_T *pRTCRegs;
	MSTIMER_REGS_T *pMSTRegs;

	pRTCRegs = (RTC_REGS_T *) OALPAtoVA((UINT32) RTC, FALSE);
	pMSTRegs = (MSTIMER_REGS_T *) OALPAtoVA((UINT32) MSTIMER, FALSE);

	trtc = pRTCRegs->ucount;
	tmst = pMSTRegs->mstim_counter;
	tmp = (UINT64) (tmst + (trtc * RTCCLKRATE));

	// Convert to milliseconds
	tmp = (tmp * 1000) / RTCCLKRATE;

	return (UINT32) tmp;
#else
	TIMER_CNTR_REGS_T *pTMR0Regs;

	pTMR0Regs = (TIMER_CNTR_REGS_T *) OALPAtoVA((UINT32) TIMER_CNTR0, FALSE);

	return (UINT32) pTMR0Regs->tc;
#endif
}

//------------------------------------------------------------------------------
//
// OEMKitlGetSecs
//
// This function returns relative time in seconds.
//
DWORD OEMKitlGetSecs()
{
    return (DWORD) (OALGetTickCount() / 1000);
}

//------------------------------------------------------------------------------
//
// OALStall
//
// Stall for a period of microSeconds.
//
VOID OALStall(UINT32 uSecs) {
	// MilliSecond granularity
	uSecs = uSecs / 1000;
	uSecs = uSecs + 1 + OALGetTickCount();
	while (uSecs > OALGetTickCount());
}

//------------------------------------------------------------------------------
//
// OEMMultiBinNotify
//
// This function check download type
//
VOID OEMMultiBinNotify( MultiBINInfo *pInfo )
{
    BOOL rc = FALSE;
	UINT32 base = IMAGE_EBOOT_RAM_PA;
    UINT32 start, length;
    UINT32 ix;

    OALMSGS(OAL_FUNC, (
        L"+OEMMultiBinNotify(0x%08x -> %d)\r\n", pInfo, pInfo->dwNumRegions
        ));
    OALMSG(OAL_INFO, (
        L"Download file information:\r\n"
        ));
    OALMSG(OAL_INFO, (
        L"-----------------------------------------------------------\r\n"
        ));

    // Copy information to EBOOT structure and set also save address
    g_eboot.numRegions = pInfo->dwNumRegions;
    for (ix = 0; ix < pInfo->dwNumRegions; ix++)
        {
        g_eboot.region[ix].start = pInfo->Region[ix].dwRegionStart;
        g_eboot.region[ix].length = pInfo->Region[ix].dwRegionLength;
        g_eboot.region[ix].base = base;
        base += g_eboot.region[ix].length;
        OALMSG(OAL_INFO, (
            L"[%d]: Address=0x%08x  Length=0x%08x  Save=0x%08x\r\n",
            ix, g_eboot.region[ix].start, g_eboot.region[ix].length,
            g_eboot.region[ix].base
            ));
        }
    OALMSG(OAL_INFO, (
        L"-----------------------------------------------------------\r\n"
        ));

    // Determine type of image downloaded
    if (g_eboot.numRegions > 1) 
    {
        OALMSG(OAL_ERROR, (L"ERROR: MultiXIP image is not supported\r\n"));
        goto cleanUp;
    }

    base = g_eboot.region[0].base;
    start = g_eboot.region[0].start;
    length = g_eboot.region[0].length; 
    if (start == IMAGE_EBOOT_CODE_PA)
    {
        g_eboot.type = DOWNLOAD_TYPE_EBOOT;
        memset((VOID*)base, 0x00, length);
    }
    else if (flashburnflag == TRUE)
    {
        g_eboot.type = DOWNLOAD_TYPE_FLASH;
    }
    else 
    {
        g_eboot.type = DOWNLOAD_TYPE_RAM;
    }

	rc = TRUE;
    
cleanUp:
	if (!rc) 
    {
        SpinForever();
    }
    OALMSGS(OAL_FUNC, (L"-OEMMultiBinNotify\r\n"));
}

//------------------------------------------------------------------------------
//
// OEMReportError
//
// This function is called for error report
//
BOOL OEMReportError(DWORD dwReason, DWORD dwReserved)
{
	SpinForever();
	return FALSE;
}

//------------------------------------------------------------------------------
//
// OEMPlatformInit
//
// This function provide platform initialization functions. 
//
BOOL OEMPlatformInit(void) 
{ 
    // Initialize callback pointer
    g_pOEMMultiBINNotify = OEMMultiBinNotify;
    g_pOEMReportError = OEMReportError;

    return TRUE;
}

//------------------------------------------------------------------------------
//
// OEMShowProgress
//
// This function is called during the download process to visualise
// download progress.
//
void OEMShowProgress (DWORD dwPacketNum) 
{
    SetLEDValue((UCHAR)(dwPacketNum >> 8));
}

//------------------------------------------------------------------------------
//
// OEMPreDownload
//
// This function is called before downloading an image. 
//
DWORD OEMPreDownload(void) 
{ 
	BOOL gdcfg;
	UINT8 *p8;
	int idx;
    ULONG rc = BL_ERROR;

	__security_init_cookie();

	// Get data for serial port speed
	BLInitBootCfg();
	gdcfg = getbldata();

	OEMDebugInit();
	sport_update_rate(g_bootCfg.baudRate);

    OALMSG(OAL_INFO, (
        L"Microsoft Windows CE EBOOT %d.%d for NXP LPC32XX "
        L"Built %S at %S\r\n",
        EBOOT_VERSION_MAJOR, EBOOT_VERSION_MINOR, __DATE__, __TIME__
    ));

    // Read saved configration and check for validity
   if (gdcfg == FALSE)
   {
		BLWriteBootCfg(&g_bootCfg);
        OALLog(L"WARN: Boot config wasn't found, using defaults\r\n");
   }
   else
   {
		OALLog(L"INFO: Boot configuration found\r\n");
   }

	// Get board configuration
	p8 = (UINT8 *) &phyhwdesc;
	for (idx = 0; idx < sizeof(phyhwdesc); idx++) 
	{
		*p8 = lpc3250_sspread(PHY3250_SEEPROM_CFGOFS + idx);
		p8++;
	}
	if (phyhwdesc.fieldvval != PHY_HW_VER_VAL) 
	{
		OALLog(L"INFO: Board configuration not found, using defaults\r\n");
		OALLog(L"INFO: 0x%x\r\n", phyhwdesc.dramcfg);
		OALLog(L"INFO: 0x%x\r\n", phyhwdesc.syscfg);
		OALLog(L"INFO: 0x%x\r\n", phyhwdesc.fieldvval);

		/* Set some defaults */
		phyhwdesc.dramcfg = (PHYHW_DRAM_TYPE_LPSDRAM |
			PHYHW_DRAM_SIZE_64M);
		phyhwdesc.syscfg = PHYHW_SDIO_POP;
		phyhwdesc.fieldvval = PHY_HW_VER_VAL;

		/* Default MAC address in index order of 0:1:2:3:4:5 */
		phyhwdesc.mac [0] = 0x00;
		phyhwdesc.mac [1] = 0x01;
		phyhwdesc.mac [2] = 0x90;
		phyhwdesc.mac [3] = 0x00;
		phyhwdesc.mac [4] = 0xC0;
		phyhwdesc.mac [5] = 0x81;
		phyhwdesc.mac [6] = 0x00;
		phyhwdesc.mac [7] = 0x00;
	}

	cleanboot = FALSE;
	flashburnflag = FALSE;

	// Initialize FLASH
	flashgood = nand_init(&savedgeom);

    // Call configuration menu
    BLMenu();

    // Image download depends on protocol
    g_eboot.bootDeviceType = OALKitlDeviceType( &g_bootCfg.bootDevLoc, g_bootDevices );

    switch (g_eboot.bootDeviceType)
    {
    case OAL_KITL_TYPE_ETH:
       rc = BLEthDownload(&g_bootCfg, g_bootDevices);
       break;
    case OAL_KITL_TYPE_FLASH:
		if (g_bootCfg.bootDevLoc.LogicalLoc == SD_BASE)
		{
	        rc = BLSDMMCDownload(&g_bootCfg, g_bootDevices);
		}
		else
		{
	        rc = BLFLASHDownload(&g_bootCfg, g_bootDevices);
		}
       break;
    default:
       OALLog(L"WARN: Unknown boot device type.\r\n");
       SpinForever();
    }

	return rc;
}

//------------------------------------------------------------------------------
//
// OEMLaunch
//
// This function is the last one called by the boot framework and it is
// responsible for to launching the image.
//
VOID OEMLaunch(ULONG start, ULONG size, ULONG launch, const ROMHDR *pRomHeader)
{
    BSP_ARGS *pArgs = OALPAtoCA(IMAGE_SHARE_ARGS_PA);

    OALMSG(OAL_FUNC, (L"+OEMLaunch(0x%08x, 0x%08x, 0x%08x, 0x%08x - %d/%d)\r\n", start, size,launch, pRomHeader, g_eboot.bootDeviceType, g_eboot.type));

    // Initialize shared ARGS structure
    memset(pArgs, 0, IMAGE_SHARE_ARGS_SIZE);
    pArgs->header.signature = OAL_ARGS_SIGNATURE;
    pArgs->header.oalVersion = OAL_ARGS_VERSION;
    pArgs->header.bspVersion = BSP_ARGS_VERSION;
    pArgs->kitl.flags = g_bootCfg.kitlFlags;
    pArgs->kitl.devLoc = g_bootCfg.kitlDevLoc;
    pArgs->kitl.ipAddress = g_bootCfg.ipAddress;
    pArgs->kitl.ipMask = g_bootCfg.ipMask;
    pArgs->kitl.ipRoute = g_bootCfg.ipRoute;
	pArgs->phyhwdesc = phyhwdesc;
	pArgs->bCleanBootFlag = cleanboot;

	// Save MAC address
	pArgs->kitl.mac [0] = (phyhwdesc.mac[0] | (phyhwdesc.mac[1] << 8));
	pArgs->kitl.mac [1] = (phyhwdesc.mac[2] | (phyhwdesc.mac[3] << 8));
	pArgs->kitl.mac [2] = (phyhwdesc.mac[4] | (phyhwdesc.mac[5] << 8));

    // Depending on protocol there can be some action required
    switch (g_eboot.bootDeviceType)
    {
        case OAL_KITL_TYPE_ETH:
            BLEthConfig(pArgs);
            switch (g_eboot.type)
            {
                case DOWNLOAD_TYPE_EBOOT:
                    OALMSG(OAL_INFO, (L"Bootloader update not supported.  Please reset system.\r\n"));
                    SetLEDValue(0xaa);
                    while(TRUE);

				default:
                    launch = (UINT32)OEMMapMemAddr(start, launch);
                    break;
            }
            break;

		case OAL_KITL_TYPE_FLASH:
            switch (g_eboot.type)
            {
				default:
                    launch = (UINT32)OEMMapMemAddr(start, launch);
                    break;
            }
            break;
			launch = (UINT32)OEMMapMemAddr(start, launch);
            break;
    }

    // Check if we get launch address
    if (launch == (UINT32)INVALID_HANDLE_VALUE)
    {
        OALMSG(OAL_ERROR, (L"ERROR: OEMLaunch: Unknown image launch address\r\n"));
        SpinForever();
    }        

    // Print message, flush caches and jump to image
    OALLog(L"INFO: Lauching Windows CE .NET by jumping to 0x%08x...\r\n\r\n\r\n", launch);
    SetLEDValue(0xaa);

    OEMDebugDeinit();

    JumpTo(launch);
}

//------------------------------------------------------------------------------
//
// OEMReadData
//
// This function is called to read data from the transport during
// the download process.
//
BOOL OEMReadData(ULONG size, UCHAR *pData)
{
	BOOL rc = FALSE;

    switch (g_eboot.bootDeviceType)
    {
    case OAL_KITL_TYPE_ETH:
        rc = EbootEtherReadData(size, pData);
		break;
    case OAL_KITL_TYPE_FLASH:
		if (g_bootCfg.bootDevLoc.LogicalLoc == SD_BASE)
		{
	        rc = BLSDMMCReadData(size, pData);
		}
		else
		{
	        rc = BLFLASHReadData(size, pData);
		}
       break;
    default:
        break;
    }

	return rc;
}

//------------------------------------------------------------------------------
//
// OEMMapMemAddr
//
// This function maps image relative address to memory address. It is used
// by boot loader to determine if an image is to be FLASH in NAND or executed.
//
UINT8* OEMMapMemAddr(    DWORD image, DWORD address )
{
    UINT8 *pAddress = NULL;

    OALMSG(OAL_FUNC, (L"+OEMMapMemAddr(0x%08x, 0x%08x)\r\n", image, address));

    switch (g_eboot.type)
        {
        case DOWNLOAD_TYPE_EBOOT:
			// Download image to RAM to program into FLASH
            pAddress = BLVAtoPA(IMAGE_EBOOT_RAM_PA);
            break;

        case DOWNLOAD_TYPE_FLASH:
		case DOWNLOAD_TYPE_RAM:
            pAddress = BLVAtoPA(address);
            break;
        default:
            OALMSG(OAL_ERROR, (L"ERROR: OEMMapMemAddr: Invalid download type %d\r\n", g_eboot.type));
            break;
        }

    OALMSGS(OAL_FUNC, (L"-OEMMapMemAddr(pAddress = 0x%08x)\r\n", pAddress));
    return pAddress;
}

//------------------------------------------------------------------------------
//
// OEMIsFlashAddr
//
// This function determines whether the address provided lies in a platform's
// flash or RAM address range.
//
BOOL OEMIsFlashAddr(ULONG address)
{
    BOOL rc;

    OALMSG(OAL_FUNC, (L"+OEMIsFlashAddr(0x%08x)\r\n", address));

    // Depending on download type
    switch (g_eboot.type)
        {
        case DOWNLOAD_TYPE_EBOOT:
            rc = TRUE;
            break;
		case DOWNLOAD_TYPE_FLASH:
            rc = flashburnflag && flashgood;
            break;
		default:
            rc = FALSE;
            break;
        }

    OALMSG(OAL_FUNC, (L"-OEMIsFlashAddr(rc = %d)\r\n", rc));
    return rc;
}
