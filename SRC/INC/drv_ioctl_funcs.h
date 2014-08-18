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
// drv_ioctl_funcs.h
//
// This file defines IOCTLs shared between an application and a driver, but
// not a driver and the kernel. These IOCTLs are used in applications for
// specific drivers to control functions and data transfer.
//

#pragma once

#include <windows.h>
#include "winioctl.h"

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
//  _IOCTL_LPC32XX_BASE
//
//  Defines the LP32XX specific application IOCTL base.
//
#define _IOCTL_APP_BASE         (2048+256)

#define IOCTL_APP_I2CREQ     CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_APP_BASE+0), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_APP_I2CSETUP   CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_APP_BASE+1), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_APP_SSPREQ     CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_APP_BASE+2), METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_APP_SSPSETUP   CTL_CODE(FILE_DEVICE_HAL, (_IOCTL_APP_BASE+3), METHOD_BUFFERED, FILE_ANY_ACCESS)

//------------------------------------------------------------------------------
//  Various I2C transfer structures and defines
//
// Structure for output of I2C data
typedef struct
{
	UINT16 flags_buff[16];
	INT32 tosend;
	INT32 torecv;    // Expected bytes to receive
} I2C_OUT_XFER_T;
// I2C statuses
typedef enum {
	I2CST_ACTIVE,     /* I2C is currently active */
	I2CST_COMPLETE,   /* Last I2C transfer has completed and is idle */
	I2CST_NAK,        /* Last I2C address was NAK and is idle */
	I2CST_ERROR       /* Last I2C transfer requested has an error */
} I2C_ST_T;
// Structure for input of I2C data
typedef struct
{
	UINT8 buff[32];
	INT32 recvBytes;
	I2C_ST_T ist;
} I2C_IN_XFER_T;

// Flag values used with the flags_buff[] array and need to be OR'ed
// with each data byte being sent to control start and stop bit placement.
#define I2C_START_FLAG 0x100
#define I2C_STOP_FLAG  0x200

// I2C read/write flag bits used with to define data direction
#define I2C_READ  0x1
#define I2C_WRITE 0x0

//------------------------------------------------------------------------------
//  IOCTL_APP_I2CREQ
//
//  Used with the I2C driver to perform the following sequence:
//      Start
//       Write to address 0x18
//       Write 0x20
//       Repeat start
//         Read from address 0x18
//         Device sends data
//      Stop (from device)
//  Use:
//  I2C_XFER_T i2cout;
//  I2C_IN_XFER_T i2cin;
//  DWORD clk, bytesret;
//  i2cout.flags_buff [0] = ((0x18<<1) | I2C_WRITE | I2C_START_FLAG)
//  i2cout.flags_buff [1] = 0x20;
//  i2cout.flags_buff [2] = ((0x18<<1) | I2C_READ | I2C_START_FLAG)
//  i2cout.tosend = 3;
//  i2cout.torecv = 1; // 1 byte of data expected
//  KernelIoControl(IOCTL_LPC32XX_GETHCLK, &i2cout, sizeof(i2cout), &i2cout,
//      sizeof (i2cout), &bytesret);
//

//------------------------------------------------------------------------------
//  IOCTL_APP_I2CREQ
//
//  Used with the I2C driver to perform the following sequence:
//      Start
//       Write to address 0x18
//       Write 0x20
//       Write 0xFF
//      Stop (from controller)
//  Use:
//  I2C_XFER_T i2cout;
//  I2C_IN_XFER_T i2cin;
//  DWORD clk, bytesret;
//  i2cout.flags_buff [0] = ((0x18<<1) | I2C_WRITE | I2C_START_FLAG)
//  i2cout.flags_buff [1] = 0x20;
//  i2cout.flags_buff [2] = (0xFF | I2C_STOP_FLAG)
//  i2cout.tosend = 3;
//  i2cout.torecv = 0; // Not receive data expected
//  KernelIoControl(IOCTL_LPC32XX_GETHCLK, &i2cout, sizeof(i2cout), &i2cout,
//      sizeof (i2cout), &bytesret);

//------------------------------------------------------------------------------
//  IOCTL_APP_I2CSETUP
//
//  Sets up an I2C channel for a specific clock rate and duty cycle.
//  Use:
//  I2C_XFER_SETUP_T i2csetup;
//  DWORD bytesret;
//  i2csetup.clock = 100000; /* 100KHz clock */
//  i2csetup.assym = FALSE;  /* Symmetrical clock (50% duty) */
//  KernelIoControl(IOCTL_APP_I2CSETUP, &i2csetup, sizeof(i2csetup), NULL,
//      0, &bytesret);
// 
// Structure for setup of I2C transfer
typedef struct
{
	UINT32 clock;
	BOOL assym;
} I2C_XFER_SETUP_T;

//------------------------------------------------------------------------------
//  Various SSP transfer structures and defines
//
// Structure for setting up the SSP interfaces
typedef struct 
{
	UINT32 databits; // Number of data bits, must be between 4 and 16
	UINT32 mode; // Transfer mode (SSP_CR0_FRF_MOT (Motorola SPI), SSP_CR0_FRF_TI (TI synchronous serial), or SSP_CR0_FRF_NS (National Microwire))
	BOOL highclkSpiFrames; // Flag used to set clock polarity high between frames when in SPI mode
	BOOL usesecondClkSpi; // Flag used to set clock out phase, use TRUE to capture serial data on the second clock transition of the frame, or FALSE to use the first clock transition of the frame
	UINT32 sspClk; // Serial clock rate
	BOOL masterMode; // Master/slave mode, use TRUE for master mode
	BOOL csLocked; // Chip select stays low for entire transfer
} SSP_XFER_SETUP_T;

// Structure for an SSP data receive
typedef struct
{
	void *recvBuff; // 8- or 16-bit aligned receive data buffer (must be 16-bit aligned if data > 8 bits)
	int recvBuffSize; // Size of the receive buffer in bytes
	int recvBytesReceived; // Will be filled in by the driver with the actual bytes received
	BOOL recvOflow; // Receive data overflow error flag
} SSP_RECV_XFER_T;

// Structure for an SSP data se nd
typedef struct
{
	void *sendBuff; // 8- or 16-bit aligned send data buffer (must be 16-bit aligned if data > 8 bits)
	int sendBuffBytes; // Number pf bytes to send in the send buffer
} SSP_SEND_XFER_T;

//------------------------------------------------------------------------------
//  IOCTL_APP_SSPSETUP
//
//  Sets up an SSP channel for a specific configuration.
//  Use:
//  DWORD bytesret;
//  SSP_XFER_SETUP_T sspsetup;
//  sspsetup.databits = 8; // 8 bits per transfer, byte aligned buffers
//  sspsetup.mode = SSP_CR0_FRF_MOT; // Motorol SPI mode
//  sspsetup.highclkSpiFrames = FALSE; // Low between frames
//  sspsetup.usesecondClkSpi = FALSE; // Normal SPI clocking
//  sspsetup.sspClk = 1000000; // 1MHz clock
//  sspsetup.masterMode = TRUE; // Master mode (should always be TRUE)
//  sspsetup.csLocked = TRUE; // CS will stay low for entire transfer
//  KernelIoControl(IOCTL_APP_SSPSETUP, &sspsetup, sizeof(sspsetup), NULL,
//      0, &bytesret);

//------------------------------------------------------------------------------
//  IOCTL_APP_SSPREQ
//
//  Used with the SSP driver to perform the following sequence:
//      Start
//       Send 0x91, Send 0x85, Send 0xFF, Send 0xFF
//       Receive 0xFF, 0xFF, 0xTT, 0xTT (0xTT is returned data)
//  Use:
//  SSP_RECV_XFER_T sspin;
//  SSP_SEND_XFER_T sspout;
//  UINT8 sendBuff[8], recvBuff[8];
//  DWORD bytesret;
//  sendBuff[0] = 0x91;
//  sendBuff[1] = 0x85;
//  sspout.sendBuff = sendBuff;
//  sspout.sendBuffBytes = 2;
//  sspin.recvBuff = recvBuff;
//  sspin.recvBuffSize = sizeof(recvBuff);
//  KernelIoControl(IOCTL_APP_SSPREQ, &sspout, sizeof(sspout), &sspin,
//      sizeof(sspin), &bytesret);

#ifdef __cplusplus
}
#endif
