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
// kitl.h
//
// KITL support over ethernet
//

#include <bsp.h>
#include <kitl_cfg.h>
#include <devload.h>
#include "phy3250_board.h"
#include "lpc32xx_rtc.h"
#include "lpc32xx_timer.h"

//------------------------------------------------------------------------------
//
// phyhwdesc
//
// This structure contains the board configuration data including the MAC
// address and SDRAM configuration.
//
PHY_HW_T phyhwdesc;

//------------------------------------------------------------------------------
// Local time used for KITL with WinCE. This value is passively picked up
// from RTC and timer pair previously set in the OEMInit() function before
// KITL is setup
static UINT64 basetime;

//------------------------------------------------------------------------------
//
// OALGetTickCount
//
// This function is called by some KITL libraries to obtain relative time
// since device boot. It is mostly used to implement timeout in network
// protocol.
//
UINT32 OALGetTickCount()
{
	TIMER_CNTR_REGS_T *pTMR;
	RTC_REGS_T *pRTC;
	UINT32 psc1, psc2, tc;
	UINT64 tickspersec, lms;

	// Get pointer to RTC and timer register blocks
	pTMR = (TIMER_CNTR_REGS_T *) OALPAtoVA((UINT32) TIMER_CNTR1, FALSE);
	pRTC = (RTC_REGS_T *) OALPAtoVA((UINT32) RTC, FALSE);

	// Get base clock for the timer
	tickspersec = (UINT64) pTMR->pr;

	// If the prescaler overflowed during the timer count read, read it again
	psc1 = 1;
	psc2 = 0;
	while (psc2 < psc1)
	{
		// Handle timer overflow
		psc1 = pTMR->pc;
		tc = pTMR->tc;
		psc2 = pTMR->pc;
	}

	// Convert to milliSeconds
	lms = (((UINT64) psc1) * 1000) / tickspersec;
	lms = lms + ((UINT64) tc) * 1000;
	lms = lms - basetime;

	return (UINT32) lms;
}

//------------------------------------------------------------------------------
//
// OEMKitlStartup
//
// KITL startup function
//
BOOL OEMKitlStartup(void)
{
    BOOL rc = FALSE;
    OAL_KITL_ARGS *pKITLArgs, KITLArgs;
    CHAR *pszDeviceId, devid[OAL_KITL_ID_SIZE];

    KITL_RETAILMSG(ZONE_KITL_OAL, ("+OEMKitlStartup\r\n"));

    // Get KITL arguments
    pKITLArgs   = (OAL_KITL_ARGS*) OALArgsQuery(OAL_ARGS_QUERY_KITL);
    pszDeviceId = (CHAR*) OALArgsQuery(OAL_ARGS_QUERY_DEVID);

    // If no KITL arguments were found (typically provided by the bootloader), then disable KITL
    if (pKITLArgs == NULL)
    {
        KITL_RETAILMSG(ZONE_ERROR, ("ERROR: Kitl boot arguments not found. Disabling KITL.\r\n"));
        memset(&KITLArgs, 0, sizeof(OAL_KITL_ARGS));
        pKITLArgs = &KITLArgs;
    }

    if (pszDeviceId == NULL)
    {
        KITL_RETAILMSG(ZONE_WARNING, ("WARN: Unable to locate device ID, using default\r\n"));
        OALKitlCreateName(BSP_DEVICE_PREFIX, pKITLArgs->mac, devid);
		pszDeviceId = devid;
    }

	// Save MAC address in this DLL's hardware copy
	phyhwdesc.mac[0] = (UINT8) ((pKITLArgs->mac [0] & 0x00FF) >> 0);
	phyhwdesc.mac[1] = (UINT8) ((pKITLArgs->mac [0] & 0xFF00) >> 8);
	phyhwdesc.mac[2] = (UINT8) ((pKITLArgs->mac [1] & 0x00FF) >> 0);
	phyhwdesc.mac[3] = (UINT8) ((pKITLArgs->mac [1] & 0xFF00) >> 8);
	phyhwdesc.mac[4] = (UINT8) ((pKITLArgs->mac [2] & 0x00FF) >> 0);
	phyhwdesc.mac[5] = (UINT8) ((pKITLArgs->mac [2] & 0xFF00) >> 8);

	// Get base time for KITL count
	basetime = OALGetTickCount();

    // Finally call KITL library.
    rc = OALKitlInit(pszDeviceId, pKITLArgs, g_kitlDevices);

    KITL_RETAILMSG(ZONE_KITL_OAL, ("-OEMKitlStartup(rc = %d)\r\n", rc));
    return(rc);
}

//------------------------------------------------------------------------------
//
// OEMKitlIoctl
//
// This function is called by some KITL libraries to process platform specific
// KITL IoCtl calls.
//
BOOL OEMKitlIoctl (DWORD code, VOID * pInBuffer, DWORD inSize, VOID * pOutBuffer, DWORD outSize, DWORD * pOutSize)
{
    KITL_RETAILMSG(ZONE_KITL_OAL, ("+OEMKitlIoctl\r\n"));
    return OALIoCtlVBridge (code, pInBuffer, inSize, pOutBuffer, outSize, pOutSize);
}

//------------------------------------------------------------------------------
//
// OEMKitlGetSecs
//
// Return seconds as needed by KITL library
//
DWORD OEMKitlGetSecs()
{
    return OALGetTickCount()/1000;
}

//------------------------------------------------------------------------------
//
// OALPAtoVA
//
// This function implements address translation function by using kernel
// mapping call. It is intended to be used in the KITL module only.
//
VOID* OALPAtoVA(UINT32 pa,
				BOOL cached)
{
    UINT32 offset;
    UINT8 *pAddress;

    offset = pa & (PAGE_SIZE - 1);
    pa &= ~(PAGE_SIZE - 1);
    pAddress = NKPhysToVirt(pa >> 8, cached);
    pAddress += offset;

	return pAddress;
}

//------------------------------------------------------------------------------
//
// OALKitlFindDevice
//
// Finds the KITL device and returns the structure pointer to it
//
OAL_KITL_DEVICE* OALKitlFindDevice(DEVICE_LOCATION *pDevLoc,
								   OAL_KITL_DEVICE *pDevice)
{
    KITL_RETAILMSG(ZONE_KITL_OAL, (
        "+OALKitlFindDevice(%d/%d/0x%x, 0x%x)\r\n",
        pDevLoc->IfcType, pDevLoc->BusNumber, pDevLoc->LogicalLoc, pDevice
    ));

	// Look for driver in list
    for(; pDevice->name != NULL; ++pDevice) {
        if(pDevLoc->IfcType==pDevice->ifcType && pDevLoc->LogicalLoc==pDevice->id) {
            pDevLoc->PhysicalLoc = (VOID*)pDevLoc->LogicalLoc; // don't convert!
            break;
        }
    }

    // Return NULL if driver wasn't found
    if (!pDevice->name) pDevice = NULL;

    KITL_RETAILMSG(ZONE_KITL_OAL, (
        "-OALKitlFindDevice(pDevice = 0x%08x(%s), PhysicalLoc = 0x%08x)\r\n",
        pDevice, (pDevice != NULL) ? pDevice->name : L"", pDevLoc->PhysicalLoc
    ));

	return pDevice;
}

//------------------------------------------------------------------------------
//
// OEMGetEthBuffers
//
// Returns address and size of ethernet DMA buffers
//
void OEMGetEthBuffers(UINT32 *dmabase, UINT32 *dmasize)
{
	*dmabase = IMAGE_WINCE_ETHDMA_PA;
	*dmasize = IMAGE_WINCE_ETHDMA_SIZE;
}
