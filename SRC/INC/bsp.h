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
//  File:  bsp.h
//

#pragma once

//------------------------------------------------------------------------------

#include <windows.h>
#include <ceddk.h>
#include <nkintr.h>
#include <oal.h>

#include "intr.h"
#include "args.h"
#include "bsp_cfg.h"
#include "image_cfg.h"

#include "winioctl.h"
#include "clkpwr_support.h"
#include "drv_ioctl_funcs.h"
#include "dma.h"

//------------------------------------------------------------------------------
//  _IOCTL_LPC32XX_BASE
//
//  Defines the LP32XX specific IOCTL base.
//
#define _IOCTL_LPC32XX_BASE     2048
    
#define IOCTL_LPC32XX_GETARMCLK CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+0), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_LPC32XX_GETHCLK   CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+1), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_LPC32XX_GETPCLK   CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+2), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_LPC32XX_ENSYSCLK  CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+3), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_LPC32XX_DMACHGET  CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+4), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_LPC32XX_DMACHFREE CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_LPC32XX_BASE+5), METHOD_BUFFERED, FILE_ANY_ACCESS)

//------------------------------------------------------------------------------
//  IOCTLs and handler for getting the ARM, HCLK, and PCLK speeds (in Hz)
//
//  Use:
//  DWORD clk, bytesret;
//  KernelIoControl(IOCTL_LPC32XX_GETARMCLK, NULL, 0, &clk,
//      sizeof (clk), &bytesret); // For ARM clock
//  KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
//      sizeof (clk), &bytesret); // For HCLK
//  KernelIoControl(IOCTL_LPC32XX_GETPCLK, NULL, 0, &clk,
//      sizeof (clk), &bytesret); // For Peripheral clock
// 
BOOL OALGetClock(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize);

//------------------------------------------------------------------------------
//  IOCTL for disabling and enabling specific system clocks
//
//  Use: (example enabling the SSP0 clock)
//  DWORD bytesret;
//  CLKPWR_CLK_T clk;
//  clk.clkid = CLKPWR_SSP0_CLK;
//  clk.enable = TRUE;
//  KernelIoControl(IOCTL_LPC32XX_ENSYSCLK, &clk, sizeof (clk), NULL, 0,
//      &bytesret);
// 
typedef struct
{
	CLKPWR_CLK_T clkid;   // Clock ID, see clkpwr_support.h
	BOOL         enable;  // TRUE to enable, FALSE to disable
} CLKENID_T;
BOOL OALIoCtlEnableSysClk(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize);

//------------------------------------------------------------------------------
//  IOCTL for enabling a DMA channel
//
//  Use:
//  DWORD chan = DMAC_AUDIO_RX_CH, bytesret, irq;
//  KernelIoControl(IOCTL_LPC32XX_DMACHGET, &chan, sizeof (chan), &irq,
//      sizeof (irq), &bytesret);
// 
BOOL OALDMAGetCh(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize);

//------------------------------------------------------------------------------
//  IOCTL for freeing an enabled DMA channel
//
//  Use:
//  DWORD chan = DMAC_AUDIO_RX_CH, bytesret;
//  KernelIoControl(IOCTL_LPC32XX_DMACHFREE, &chan, sizeof (chan), NULL,
//      0, &bytesret);
// 
BOOL OALDMAFreeCh(
    UINT32 code, VOID* pInpBuffer, UINT32 inpSize, VOID* pOutBuffer,
    UINT32 outSize, UINT32 *pOutSize);

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  dma_init
//
//  Initializes kernel DMA channel allocation
void dma_init(void);
