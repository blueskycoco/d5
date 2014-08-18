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
//  File:  bsp_cfg.h
//

#pragma once

//------------------------------------------------------------------------------
//
//  Define:  BSP_DEVICE_PREFIX
//
//  This define is used as device name prefix when KITL creates device name.
//
#define BSP_DEVICE_PREFIX       "lpc32-"

//------------------------------------------------------------------------------
//
//  Define:  BSP_UART_xxx
//
//  This constant is used to initialize serial debugger output UART.
//  On ArubaBoard serial debugger uses 115200 Bd, 8 bits, 1 stop bit, no parity.
//
#define BSP_UART_RATE           115200

//------------------------------------------------------------------------------
//
//  Define:  EBOOT_CFG_SSEPROM_SINDEX
//
//  Offset (bytes) in the serial EEPROM where the boot configuration is stored.
//
#define EBOOT_CFG_SSEPROM_SINDEX 0x1000

//------------------------------------------------------------------------------
//
//  Define:  RTC clock rate in Hz
//
#define RTCCLKRATE 32768

//------------------------------------------------------------------------------
