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
//  File:  kitl_cfg.h
//

#pragma once

//------------------------------------------------------------------------------
// Prototypes for LPC32XX RMI ethernet
BOOL   LPC32XX_Eth_Init(UINT8 *pAddress, UINT32 offset, UINT16 mac[3]);
//BOOL   LPC32xx_Eth_InitDMABuffer(UINT32 address, UINT32 size);
BOOL   LPC32XX_Eth_Deinit(VOID);
UINT16 LPC32XX_Eth_SendFrame(UINT8 *pbData, UINT32 length);
UINT16 LPC32XX_Eth_GetFrame(UINT8 *pbData, UINT16 *pLength);
VOID   LPC32XX_Eth_EnableInts();
VOID   LPC32XX_Eth_DisableInts();
VOID   LPC32XX_Eth_PowerOff();
VOID   LPC32XX_Eth_PowerOn();
VOID   LPC32XX_Eth_CurrentPacketFilter(UINT32 filter);
BOOL   LPC32XX_Eth_MulticastList(UINT8 *pAddresses, UINT32 count);

//------------------------------------------------------------------------------

OAL_KITL_ETH_DRIVER g_kitllpc32xx = {
	LPC32XX_Eth_Init, NULL, LPC32XX_Eth_Deinit,
	LPC32XX_Eth_SendFrame, LPC32XX_Eth_GetFrame, LPC32XX_Eth_EnableInts,
	LPC32XX_Eth_DisableInts, LPC32XX_Eth_PowerOff, LPC32XX_Eth_PowerOn,
	LPC32XX_Eth_CurrentPacketFilter, LPC32XX_Eth_MulticastList};

OAL_KITL_DEVICE g_kitlDevices[] = {
	{
        L"LPC32xx RMII ethernet", Internal, ETHERNET_BASE, 
        0, OAL_KITL_TYPE_ETH, &g_kitllpc32xx
    },
	{
        NULL, 0, 0, 0, 0, NULL
    }
};    

//------------------------------------------------------------------------------
