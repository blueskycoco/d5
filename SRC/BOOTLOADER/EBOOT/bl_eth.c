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
//  File:  eth.c
//
//  This file implements bootloader functions related to ethernet download
//

#include <eboot.h>

//------------------------------------------------------------------------------
//
// g_ethState
//
// This structure contains local state variables.
//
static struct {
    EDBG_ADDR edbgAddress;
    OAL_KITL_ETH_DRIVER *pDriver;
    UINT8 buffer[OAL_KITL_BUFFER_SIZE];
} g_ethState;

//------------------------------------------------------------------------------
//
// OEMGetEthBuffers
//
// Returns address and size of ethernet DMA buffers.
//
void OEMGetEthBuffers(UINT32 *dmabase, UINT32 *dmasize)
{
	*dmabase = IMAGE_EBOOT_ETHDMA_PA;
	*dmasize = IMAGE_EBOOT_ETHDMA_SIZE;
}

//------------------------------------------------------------------------------
//
// BLEthDownload
//
// This function initialize ethernet adapter and call download function from
// bootloader common library.
//
UINT32 BLEthDownload(BOOT_CFG *pConfig, OAL_KITL_DEVICE *pBootDevices)
{
	UINT32 rc = BL_ERROR;
    OAL_KITL_DEVICE *pDevice;
    UINT32 subnetMask, *pLeaseTime, leaseTime = 0;
    UINT16 mac[3];
    BOOL jumpToImage = FALSE;
    UINT8 deviceId[OAL_KITL_ID_SIZE];

    // Find boot/KITL device driver
    pDevice = OALKitlFindDevice(&pConfig->bootDevLoc, pBootDevices);        
    if (pDevice == NULL || pDevice->type != OAL_KITL_TYPE_ETH) {
        OALMSG(OAL_ERROR, (
            L"ERROR: Boot device doesn't exist or it is unsupported\r\n"
        ));
        goto cleanUp;
    }

    // Get device driver
    g_ethState.pDriver = (OAL_KITL_ETH_DRIVER*)pDevice->pDriver;

    // Call InitDmaBuffer if there is any
    if (g_ethState.pDriver->pfnInitDmaBuffer != NULL) 
	{
        if (!g_ethState.pDriver->pfnInitDmaBuffer((UINT32)g_ethState.buffer, sizeof(g_ethState.buffer))) 
		{
            OALMSG(OAL_ERROR, (L"ERROR: Boot device driver InitDmaBuffer call failed\r\n"));
            goto cleanUp;
        }
    }      

    // Call Init
    if (!g_ethState.pDriver->pfnInit(pConfig->bootDevLoc.PhysicalLoc, pConfig->bootDevLoc.LogicalLoc, mac)) 
	{
        OALMSG(OAL_ERROR, (L"ERROR: Boot device driver Init call failed\r\n"));
        goto cleanUp;
    }
	
    OALMSG(OAL_INFO, (L"INFO: Boot device uses MAC %s\r\n", OALKitlMACtoString(mac)));

    // Generate device name
    OALKitlCreateName(BSP_DEVICE_PREFIX, mac, deviceId);
    OALMSG(OAL_INFO, (L"INFO: *** Device Name %S ***\r\n", deviceId));

    // Initialize ethernet
    memset(&g_ethState.edbgAddress, 0, sizeof(g_ethState.edbgAddress));

    // Set lease time pointer to activate DHCP or update global structure
    if ((pConfig->kitlFlags & OAL_KITL_FLAGS_DHCP) != 0) 
	{
       pLeaseTime = &leaseTime;
    } 
	else 
	{
       pLeaseTime = NULL;
       g_ethState.edbgAddress.dwIP = pConfig->ipAddress;
    }

    memcpy(g_ethState.edbgAddress.wMAC, mac, sizeof(g_ethState.edbgAddress.wMAC));
    subnetMask = 0xFFFFFF;
       
    // Initialize TFTP transport
    // Note that first parameter must be pointer to global variable...
    if (!EbootInitEtherTransport(
       &g_ethState.edbgAddress, &subnetMask, &jumpToImage, pLeaseTime,
       EBOOT_VERSION_MAJOR, EBOOT_VERSION_MINOR, BSP_DEVICE_PREFIX, 
       deviceId, EDBG_CPU_TYPE_ARM|0x0F, 0)) 
    {
       OALMSG(OAL_ERROR, (L"ERROR: EbootInitEtherTransport call failed\r\n"));
       goto cleanUp;
    }

    // If DHCP was used update lease time and address
    if (pLeaseTime != NULL) 
    {
		pConfig->ipAddress = g_ethState.edbgAddress.dwIP;
    }

    // Depending on what we get from TFTP
    rc = jumpToImage ? BL_JUMP : BL_DOWNLOAD;

cleanUp:
    return rc;
}

//------------------------------------------------------------------------------
//
// BLEthConfig
//
// This function receives connect message from host and it updates arguments
// based on it. Most part of connect message isn't used by KITL anymore, but
// we want set passive KITL mode when no-KITL is select on PB.
//
VOID BLEthConfig(BSP_ARGS *pArgs)
{
    EDBG_OS_CONFIG_DATA *pConfig;

    // Get host connect info
    pConfig = EbootWaitForHostConnect(&g_ethState.edbgAddress, NULL);

    pArgs->kitl.flags &= ~OAL_KITL_FLAGS_PASSIVE;
    if ((pConfig->KitlTransport & KTS_PASSIVE_MODE) != 0) {
        pArgs->kitl.flags |= OAL_KITL_FLAGS_PASSIVE;
    }
}

//------------------------------------------------------------------------------
//
// OEMEthGetFrame
//
// Check to see if a frame has been received, and if so copy to buffer.
// An optimization  which may be performed in the Ethernet driver is to 
// filter out all received broadcast packets except for ARPs.
//
BOOL OEMEthGetFrame(BYTE *pData, UINT16 *pLength)
{
    return g_ethState.pDriver->pfnGetFrame(pData, pLength) > 0;
}

//------------------------------------------------------------------------------
//
// OEMEthSendFrame
//
// Send Ethernet frame 
//
BOOL OEMEthSendFrame(BYTE *pData, DWORD length)
{
    return g_ethState.pDriver->pfnSendFrame(pData, length) == 0;
}
