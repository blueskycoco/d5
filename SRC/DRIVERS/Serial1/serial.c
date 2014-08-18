//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
/*++
  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
  ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  PARTICULAR PURPOSE.

  Module Name:  

  ser16550.c

Abstract:  

This file implements the standard device specific functions for a 16550
based serial device.

Functions:

SL_Init()
SL_PostInit()
SL_Deinit()
SL_Open()
SL_Close()
SL_ClearDTR()
SL_SetDTR()
SL_ClearRTS()
SL_SetRTS()
SL_ClearBreak()
SL_SetBreak()
SL_SetBaudRate()
SL_SetByteSize()
SL_SetParity()
SL_SetStopBits()
SL_GetRxBufferSize()
SL_GetRxStart()
SL_GetInterruptType()
SL_RxIntr()
SL_PutBytes()
SL_TxIntr()
SL_LineIntr()
SL_OtherIntr()
SL_GetStatus()
SL_Reset()
SL_GetModemStatus()
SL_PurgeComm()
SL_XmitComChar()
SL_PowerOff()
SL_PowerOn()
SL_SetDCB()
SL_SetCommTimeouts()
SL_Ioctl()
ReadLSR()
ReadMSR()
DumpSerialRegisters()
LookUpValue()
DivisorOfRate()



Notes:
The RegCritSec is there to protect against non-atomic access of
register pairs.  On a 16550, the main such collision comes from 
the fact that THR and IER are overloaded as the DivLatch registers
and their mode is controlled via the LCR.  So we need the 
critical section protection around all access of these 3 registers.
But we also need to watch out for read/modify/write where we are
setting/ckearing a bit.  In general, I just go ahead and acquire
the CS around all register accesses.
--*/

#include <windows.h>
#include <types.h>
#include <nkintr.h>
#include <ceddk.h>
#include <memory.h>
#include <devload.h>
#include <ddkreg.h>
#define USE_NEW_SERIAL_MODEL
#include <serhw.h>
#include <serdbg.h>
#include <excpt.h>
#include "lpc32xx_uart.h"
#include "lpc32xx_clkpwr.h"
#include "clkpwr_support.h"
#include "isr.h"
#include "hw.h"
#include "serial.h"


#ifndef _PREFAST_
#pragma warning(disable: 4068) // Disable pragma warnings
#endif

// for 8BIT data bus
#define CPU_REG_STRIDE		1

#define INL(pInfo, reg) (READ_PORT_ULONG((ULONG *)((pInfo)->reg)))
#define OUTL(pInfo, reg, value) (WRITE_PORT_ULONG((ULONG *)((pInfo)->reg), (unsigned long)(value)))


//
// This is a reverse lookup table which can be used to determine
// the FIFO trigger level from the 2 bit value stored in the FCR
//
#define HIGH_WATER_SIZE     4
static const
PAIRS    HighWaterPairs[HIGH_WATER_SIZE] = {
	{SERIAL_1_BYTE_HIGH_WATER, 0},
	{SERIAL_4_BYTE_HIGH_WATER, 4},
	{SERIAL_8_BYTE_HIGH_WATER, 8},
	{SERIAL_14_BYTE_HIGH_WATER, 14}
};
static const
LOOKUP_TBL  HighWaterTable = {HIGH_WATER_SIZE, (PAIRS *) HighWaterPairs};

#define WATERMAKER_ENTRY 2

// Interrupt Enable bits
#define IER_NORMAL_INTS (SERIAL_IER_RDA | SERIAL_IER_RLS | SERIAL_IER_MS)


#define EXCEPTION_ACCESS_VIOLATION STATUS_ACCESS_VIOLATION 


//
// Reading the LSR clears most of its bits.  So, we provide this wrapper,
// which reads the register, records any interesting values, and
// stores the current LSR contents in the shadow register.
//
	__inline 
VOID ProcessLSR (PSER16550_INFO  pHWHead)
{
	ULONG LineEvents = 0;
	if ( pHWHead->Iir & (LPC32XX_HSU_RX_OE_INT | LPC32XX_HSU_FE_INT)) {
		// Note: Its not wise to do debug msgs in here since they will
		// pretty much guarantee that the FIFO gets overrun.
		if ( pHWHead->Iir & LPC32XX_HSU_RX_OE_INT ) {
			// DEBUGMSG (ZONE_WARN, (TEXT("Overrun\r\n")));
			pHWHead->DroppedBytes++;
			pHWHead->CommErrors |= CE_OVERRUN;
			OUTL(pHWHead, pIir, pHWHead->Iir&LPC32XX_HSU_RX_OE_INT);
		}

		if ( pHWHead->Iir & LPC32XX_HSU_FE_INT ) {
			// DEBUGMSG (ZONE_WARN, (TEXT("frame\r\n")));
			pHWHead->CommErrors |= CE_FRAME;
			OUTL(pHWHead, pIir, pHWHead->Iir&LPC32XX_HSU_FE_INT);
		}

		LineEvents |= EV_ERR;
	}

	if ( pHWHead->Iir & LPC32XX_HSU_BRK_INT )
	{
		LineEvents |= EV_BREAK;
		OUTL(pHWHead, pIir, pHWHead->Iir&LPC32XX_HSU_BRK_INT);
	}

	// Let WaitCommEvent know about this error
	if ( LineEvents )
		pHWHead->EventCallback( pHWHead->pMddHead, LineEvents );

}
__inline
	VOID
ReadLSR(
		PSER16550_INFO  pHWHead
	   )
{

	try {
		pHWHead->Iir = INL(pHWHead, pIir);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		pHWHead->Iir = 0xff;
	}
	ProcessLSR (pHWHead);
}

//#ifdef DEBUG
//
// This routine is used only for debugging, and performs a formatted
// ascii dump of the various UART registers.
//
	VOID
DumpSerialRegisters(
		PVOID  pHead
		)
{
	UINT32 byte;
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;


#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		ReadLSR( pHWHead );
		byte = pHWHead->Iir;

		NKDbgPrintfW(TEXT("16550 Iir: \t%2.2X\t"), byte);
		if ( byte & LPC32XX_HSU_TX_INT )
			NKDbgPrintfW(TEXT("Tx int "));
		if ( byte & LPC32XX_HSU_RX_TRIG_INT )
			NKDbgPrintfW(TEXT("Rx trig int "));
		if ( byte & LPC32XX_HSU_RX_TIMEOUT_INT )
			NKDbgPrintfW(TEXT("Rx timeout int "));
		if ( byte & LPC32XX_HSU_FE_INT )
			NKDbgPrintfW(TEXT("FramingErr "));
		if ( byte & LPC32XX_HSU_BRK_INT )
			NKDbgPrintfW(TEXT("BreakIntpt "));
		if ( byte & LPC32XX_HSU_RX_OE_INT )
			NKDbgPrintfW(TEXT("Rx overrun int "));
		if ( byte & LPC32XX_HSU_TX_INT_SET )
			NKDbgPrintfW(TEXT("Tx int set "));
		NKDbgPrintfW(TEXT("\r\n"));

		byte = INL(pHWHead, pRData);
		NKDbgPrintfW(TEXT("16550 rbr/thr:\t%2.2X\r\n"), byte);

		byte = INL(pHWHead, pLevel);
		NKDbgPrintfW(TEXT("16550 Tx /Rx Level: \t%2.2X\t"), byte);
		
		NKDbgPrintfW(TEXT("\r\n"));

		byte = INL(pHWHead, pCr);
		NKDbgPrintfW(TEXT("16550 Cr: \t%2.2X\t"), byte);

		NKDbgPrintfW(TEXT("\r\n"));

		byte = INL(pHWHead, pRcr);
		NKDbgPrintfW(TEXT("16550 Rcr: \t%2.2X\t"), byte);
		NKDbgPrintfW(TEXT("\r\n"));

	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Nothing much to clean up here.
	}
#pragma prefast(pop)
}
//#endif // DEBUG
unsigned int __serial_get_clock_div(unsigned long uartclk,
	unsigned long rate) {
	UINT32 div, goodrate, hsu_rate, l_hsu_rate, comprate;
	UINT32 rate_diff;

	/* Find the closest divider to get the desired clock rate */
	div = uartclk / rate;
	goodrate = hsu_rate = (div / 14) - 1;
	if (hsu_rate != 0)
		hsu_rate--;

	/* Tweak divider */
	l_hsu_rate = hsu_rate + 3;
	rate_diff = 0xFFFFFFFF;

	while (hsu_rate < l_hsu_rate) {
		comprate = uartclk / ((hsu_rate + 1) * 14);
		if (abs(comprate - rate) < rate_diff) {
			goodrate = hsu_rate;
			rate_diff = abs(comprate - rate);
		}

		hsu_rate++;
	}
	if (hsu_rate > 0xFF)
		hsu_rate = 0xFF;

	return goodrate;
}

VOID set_div(PSER16550_INFO    pHWHead,UINT32 baud)
{

	UINT32 basepclk, savedclkrate, diff, clkrate;

	// Get base clock for UART
	basepclk = (INT32)(clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK) >> 4);

	OUTL(pHWHead,pRcr,__serial_get_clock_div(basepclk,baud));
	return ;
}

// Routine to clear any pending interrupts.  Called from Init and PostInit
// to make sure we start out in a known state.
	VOID
ClearPendingInts(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;


	EnterCriticalSection(&(pHWHead->RegCritSec));

	try {
		OUTL(pHWHead,pIir,(LPC32XX_HSU_TX_INT | LPC32XX_HSU_FE_INT |LPC32XX_HSU_BRK_INT | LPC32XX_HSU_RX_OE_INT));		
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		DEBUGMSG (ZONE_ERROR,(TEXT("-SL_PostInit, 0x%X - ERROR\r\n"), pHWHead));
		// Just fall through & release CritSec
	}

	LeaveCriticalSection(&(pHWHead->RegCritSec));
}
#define NUM_OF_SHARED_PAGE 1
BOOL InstallSoftwareISR(PSER16550_INFO pHWHead,PVOID pIoAddr,WORD uMulti)
{
	pHWHead->pIsrInfoVirt=NULL;
	pHWHead->pReceiveBuffer=NULL;
	pHWHead->pXmitBuffer=NULL;
	pHWHead->hIsrHandler=NULL;
	if (pHWHead->dwIrq!=(DWORD)-1 && pHWHead->RegIsrDll[0] !=0 && pHWHead->RegIsrHandler[0]!=0) {
		// We create Share Structure First.
		DWORD dwBlockSize=NUM_OF_SHARED_PAGE*PAGE_SIZE;
		PVOID pIsrAddress=NULL;
		KLibAllocShareMem(NUM_OF_SHARED_PAGE,TRUE,(LPVOID *)&(pHWHead->pIsrInfoVirt),&pIsrAddress);
		DEBUGMSG (1,(TEXT("SL_InstallSoftwareISR, VirtualAddr=0x%X Kernel Addr=0x%X\r\n"),pHWHead->pIsrInfoVirt ,pIsrAddress));

		if (pHWHead->pIsrInfoVirt) {
			DWORD dwRcvBlockSize;
			DWORD dwXmitBlockSize;
			pHWHead->pIsrInfoVirt->pBlockPhysAddr=pIsrAddress;
			pHWHead->pIsrInfoVirt->dwBlockSize=dwBlockSize;
			pHWHead->pIsrInfoVirt->dwReceiveHWWaterMaker=HighWaterPairs[2].AssociatedValue;
			pHWHead->pIsrInfoVirt->InBufferOffset=sizeof(ISR16550_INFO);
			dwRcvBlockSize=dwBlockSize - sizeof(ISR16550_INFO);
			dwRcvBlockSize = dwRcvBlockSize * 2 / 3; // Harf for xmitting harf for Receiving. receive buffer is double size of send
			dwRcvBlockSize = (dwRcvBlockSize/sizeof(DWORD))*sizeof(DWORD); // Make DWORD alignment.
			pHWHead->pIsrInfoVirt->OutBufferOffset=sizeof(ISR16550_INFO)+dwRcvBlockSize;
			// Initial Receive 
			pHWHead->pReceiveBuffer=(PRcvDataBuffer)((PBYTE)pHWHead->pIsrInfoVirt+pHWHead->pIsrInfoVirt->InBufferOffset);
			pHWHead->pReceiveBuffer->dwBufferSize=(dwRcvBlockSize-sizeof(RcvDataBuffer))/sizeof(IIR_EVENT);
			pHWHead->pReceiveBuffer->dwWaterMark=pHWHead->pReceiveBuffer->dwBufferSize/2;
			pHWHead->pReceiveBuffer->dwFIFO_In = pHWHead->pReceiveBuffer->dwFIFO_Out=0;
			// Inital Xmit Buffer.
			pHWHead->pXmitBuffer=(PXmitDataBuffer)((PBYTE)pHWHead->pIsrInfoVirt+pHWHead->pIsrInfoVirt->OutBufferOffset);
			ASSERT(pHWHead->pIsrInfoVirt->OutBufferOffset+sizeof(XmitDataBuffer)<dwBlockSize);
			dwXmitBlockSize =dwBlockSize- pHWHead->pIsrInfoVirt->OutBufferOffset;
			pHWHead->pXmitBuffer->dwBufferSize=dwXmitBlockSize-sizeof(XmitDataBuffer);
			pHWHead->pXmitBuffer->dwWaterMark=pHWHead->pXmitBuffer->dwBufferSize/2;
			pHWHead->pXmitBuffer->dwFIFO_In= pHWHead->pXmitBuffer->dwFIFO_Out=0;
			//Set Hardware Info.
			pHWHead->pIsrInfoVirt->SysIntr=pHWHead->dwSysIntr;
			pHWHead->pIsrInfoVirt->pIoAddress=pIoAddr;
			pHWHead->pIsrInfoVirt->lIoSpace = 0; // This one support IO address only.
			pHWHead->pIsrInfoVirt->uMultiplier=uMulti;
			pHWHead->bMoreXmitData=FALSE;
			pHWHead->pIsrInfoVirt->bIntrBypass=FALSE;
			// INstall The ISR.
			DEBUGMSG (1,(TEXT("SL_InstallSoftwareISR, SysIntr=0x%X,Irq=0x%X,ioAddr==0x%X \r\n"),
						pHWHead->pIsrInfoVirt->SysIntr,pHWHead->dwIrq,pHWHead->pIsrInfoVirt->pIoAddress));

			pHWHead->hIsrHandler = LoadIntChainHandler(pHWHead->RegIsrDll, pHWHead->RegIsrHandler, (BYTE)pHWHead->dwIrq);
			if (pHWHead->hIsrHandler == NULL) {
				DEBUGMSG(ZONE_ERROR, (TEXT("SL_InstallSoftwareISR: LoadIntChainHandler(%s, %s, %d) failed\r\n"),
							pHWHead->RegIsrDll, pHWHead->RegIsrHandler,pHWHead->dwIrq));
				FreePhysMem((LPVOID)pHWHead->pIsrInfoVirt);
				pHWHead->pIsrInfoVirt=NULL;
				return FALSE;;
			}
			if (!KernelLibIoControl(pHWHead->hIsrHandler, IOCTL_ISR16550_INFO, pIsrAddress, dwBlockSize, NULL, 0, NULL)) {
				DEBUGMSG(ZONE_ERROR,(TEXT("SL_InstallSoftwareISR: KernelLibIoControl call failed.\r\n")));
				KernelLibIoControl(pHWHead->hIsrHandler, IOCTL_ISR16550_UNLOAD, (LPVOID)&pHWHead->pIsrInfoVirt, sizeof(ISR16550_INFO), NULL, 0, NULL);
				FreePhysMem((LPVOID)pHWHead->pIsrInfoVirt);
				pHWHead->pIsrInfoVirt=NULL;
				return FALSE;
			}
			return TRUE;
		}
		else {
			DEBUGMSG (ZONE_ERROR,(TEXT("SL_InstallSoftwareISR, Cano not alloc Phys Buffer size=0x%X - ERROR\r\n"),dwBlockSize ));
			return FALSE;
		}
	}
	else {
		DEBUGMSG (1,(TEXT("SL_InstallSoftwareISR, No Installable ISR \r\n")));
		return TRUE;
	}
}
BOOL UninstallSoftwareISR(PSER16550_INFO pHWHead)
{
	if (pHWHead->hIsrHandler){
		FreeIntChainHandler(pHWHead->hIsrHandler);
	};
	if (pHWHead->pIsrInfoVirt) {
		FreePhysMem((LPVOID)pHWHead->pIsrInfoVirt);
		pHWHead->pIsrInfoVirt=NULL;
	}
	return TRUE;   
}

//
/////////////////// Start of exported entrypoints ////////////////
//
//
// @doc OEM 
// @func PVOID | SL_Open | Configures 16550 for default behaviour.
//
	BOOL
SL_Open(
		PVOID   pHead // @parm PVOID returned by HWinit.
	   )
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_OPEN,
			(TEXT("+SL_Open 0x%X\r\n"), pHead));

	// If the device is already open, all we do is increment count
	if ( pHWHead->OpenCount++ ) {
		DEBUGMSG (ZONE_OPEN,
				(TEXT("-SL_Open 0x%X (%d opens)\r\n"),
				 pHead, pHWHead->OpenCount));
		return FALSE;
	}
	if (pHWHead->pIsrInfoVirt) {
		pHWHead->pIsrInfoVirt->dwReceiveHWWaterMaker=HighWaterPairs[WATERMAKER_ENTRY].AssociatedValue;
		if (pHWHead->pReceiveBuffer)
			pHWHead->pReceiveBuffer->dwFIFO_Out=pHWHead->pReceiveBuffer->dwFIFO_In ;
		while (pHWHead->pXmitBuffer && pHWHead->pXmitBuffer->dwFIFO_In!= pHWHead->pXmitBuffer->dwFIFO_Out)
			pHWHead->pXmitBuffer->dwFIFO_In= pHWHead->pXmitBuffer->dwFIFO_Out;
	}
	pHWHead->FCR = 0;
	pHWHead->IER = 0;
	pHWHead->IIR = 0;
	pHWHead->LSR = 0;
	pHWHead->MSR = 0;
	pHWHead->DroppedBytes = 0;
	pHWHead->CTSFlowOff = FALSE;  // Not flowed off yet
	pHWHead->DSRFlowOff = FALSE;  // Not flowed off yet
	pHWHead->CommErrors   = 0;
	pHWHead->ModemStatus  = 0;

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		OUTL(pHWHead, pIER, (UCHAR)IER_NORMAL_INTS);
		OUTL(pHWHead, pMCR, SERIAL_MCR_IRQ_ENABLE);

		// Set default framing bits.
		OUTL(pHWHead, pLCR, SERIAL_8_DATA | SERIAL_1_STOP | SERIAL_NONE_PARITY);

		DEBUGMSG (ZONE_OPEN,
				(TEXT("SL_Open Setting DCB parameters\r\n")));

		// Get defaults from the DCB structure
		SL_SetBaudRate( pHead, pHWHead->dcb.BaudRate );
		SL_SetByteSize( pHead, pHWHead->dcb.ByteSize );
		SL_SetStopBits( pHead, pHWHead->dcb.StopBits );
		SL_SetParity(   pHead, pHWHead->dcb.Parity );

		//
		// A 16450 (which is pretty much a FIFO-less 16550) can be supported by
		// not initializing the FIFO.
		//
		if (pHWHead->ChipID == CHIP_ID_16550) {
			// Set up to use 16550 fifo for 8 byte interrupt granularity.
			// Shadow the FCR bitmask since reading this location is the IIR.
			pHWHead->FCR = SERIAL_FCR_ENABLE | (BYTE)HighWaterPairs[WATERMAKER_ENTRY].Key;

			OUTL(pHWHead, pIIR_FCR,
					(pHWHead->FCR | SERIAL_FCR_RCVR_RESET | SERIAL_FCR_TXMT_RESET) );
		}

		// For CE 3.0, we are still supporting
		// the old style MDDs, and they don't call our PostInit, which
		// needs to happen sometime prior to this.  So for now, we go ahead
		// ahead and clear out interrupts one last time.  In 4.0, we can
		// kill the old serial MDD and assume that everyone uses the new
		// MDD and calls post init.  
		SL_PostInit(pHWHead);

		ReadMSR(pHWHead);
		ReadLSR(pHWHead);

#ifdef DEBUG
		if ( ZONE_INIT )
			DumpSerialRegisters(pHWHead);
#endif
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just get out of here.
	}
#pragma prefast(pop)

	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_OPEN,
			(TEXT("-SL_Open 0x%X, IIR 0x%X\r\n"), pHead, pHWHead->IIR));
	return TRUE;
}

//
// @doc OEM 
// @func PVOID | SL_Close | Does nothing except keep track of the
// open count so that other routines know what to do.
//
	ULONG
SL_Close(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_CLOSE,
			(TEXT("+SL_Close 0x%X\r\n"), pHead));

	if ( pHWHead->OpenCount )
		pHWHead->OpenCount--;

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		// Disable all interrupts and clear MCR.

		OUTL(pHWHead, pIER, (UCHAR)0); 
		OUTL(pHWHead, pMCR, (UCHAR)0);

		pHWHead->IIR   = INL(pHWHead, pIIR_FCR);        
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just get out of here.
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_CLOSE,
			(TEXT("-SL_Close 0x%X\r\n"), pHead));
	return 0;
}

/*++
 *******************************************************************************
Routine:

Ser_GetRegistryData

Description:

Take the registry path provided to COM_Init and use it to find this 
requested comm port's DeviceArrayIndex, the IOPort Base Address, and the
Interrupt number.

Arguments:

LPCTSTR regKeyPath  the registry path passed in to COM_Init.

Return Value:

-1 if there is an error.

 *******************************************************************************
 --*/
	BOOL
Ser_GetRegistryData(PSER_INFO pHWHead, LPCTSTR regKeyPath)
{
#define GCI_BUFFER_SIZE 256   
	LONG    regError;
	HKEY    hKey;
	DWORD   dwDataSize = GCI_BUFFER_SIZE;
	PHYSICAL_ADDRESS phBase; 
	ULONG irq;

	DEBUGMSG(ZONE_INIT, (TEXT("Try to open %s\r\n"), regKeyPath));

	// We've been handed the name of a key in the registry that was generated
	// on the fly by device.exe.  We're going to open that key and pull from it
	// a value that is the name of this serial port's real key.  That key
	// will have the DeviceArrayIndex that we're trying to find.  
	hKey = OpenDeviceKey(regKeyPath);
	if ( hKey == NULL ) {
		DEBUGMSG(ZONE_INIT | ZONE_ERROR,
				(TEXT("Failed to open device key\r\n")));
		return ( FALSE );        
	}

	// Okay, we're finally ready to try and load our registry data - DeviceArrayIndex.
	dwDataSize = PC_REG_DEVINDEX_VAL_LEN;
	regError = RegQueryValueEx(
			hKey, 
			PC_REG_DEVINDEX_VAL_NAME, 
			NULL, 
			NULL,
			(LPBYTE)(&pHWHead->dwDevIndex), 
			&dwDataSize);
	RegCloseKey (hKey);

	if ( regError != ERROR_SUCCESS ) {
		DEBUGMSG(ZONE_INIT | ZONE_ERROR,
				(TEXT("Failed to get serial registry values, Error 0x%X\r\n"),
				 regError));
		return ( FALSE );
	}

	switch(pHWHead->dwDevIndex)
	{
		case 1:
			{        			
				phBase.QuadPart = HS_UART1_BASE;
				irq = IRQ_UART_IIR1;
				pHWHead->dwSysIntr = OAL_INTR_IRQ_UART1;
				RETAILMSG(1,(TEXT("Ser_GetRegistryData UART1\r\n")));
			}
			break;
		default:
			RETAILMSG(1,(TEXT("HS driver , Index is invalid\r\n")));
			return FALSE;
	}

	// Map physical memory
	pHWHead->pBaseAddress = (PUCHAR)MmMapIoSpace(phBase, 24, FALSE);

	if (pHWHead->pBaseAddress == NULL) {
		DEBUGMSG(ZONE_ERROR, (L" SL_Init - Failed map physical memory 0x%x\n", phBase.LowPart));
		return FALSE;
	}

	// Save IRQ

	pHWHead->ser16550.dwIrq = irq;
	pHWHead->ser16550.dwSysIntr = pHWHead->dwSysIntr;

	if (SYSINTR_NOP==pHWHead->dwSysIntr) {
		DEBUGMSG(ZONE_ERROR, (L" SL_Init - Failed map IRQ %d\n",irq));
		return FALSE;
	}

	DEBUGMSG (1|ZONE_INIT,
			(TEXT("SerInit - Devindex %d, SysIntr %d, BaseAddress %X \r\n"),
			 pHWHead->dwDevIndex, pHWHead->dwSysIntr, pHWHead->pBaseAddress));

	return ( TRUE ); 
}

//
// @doc OEM 
// @func PVOID | SL_Init2 | Initializes 16550 device head.  
//
	VOID
SL_Init2(
		PVOID   pHead, // @parm points to device head
		PUCHAR  pRegBase, // Pointer to 16550 register base
		UINT8   RegStride, // Stride amongst the 16550 registers
		EVENT_FUNC EventCallback, // This callback exists in MDD
		PVOID   pMddHead
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_CLOSE,(TEXT("+SL_INIT, 0x%X\r\n"), pHWHead));

	pHWHead->ChipID = CHIP_ID_16550;

	// Set up pointers to 16550 registers
	pHWHead->pRData    = pRegBase + (RegStride * RECEIVE_BUFFER_REGISTER);
	pHWHead->pTData     = pRegBase + (RegStride * TRANSMIT_HOLDING_REGISTER);
	pHWHead->pLevel = pRegBase + (RegStride * LEVEL_REGISTER);
	pHWHead->pIir     = pRegBase + (RegStride * INTERRUPT_IDENT_REGISTER);
	pHWHead->pCr     = pRegBase + (RegStride * CONTROL_REGISTER);
	pHWHead->pRcr     = pRegBase + (RegStride * RATE_CONTROL_REGISTER);

	// Store info for callback function
	pHWHead->EventCallback = EventCallback;
	pHWHead->pMddHead = pMddHead;


	pHWHead->FlushDone      = CreateEvent(0, FALSE, FALSE, NULL);

	pHWHead->OpenCount = 0;
	DumpSerialRegisters(pHWHead);

	// Don't allow any interrupts till PostInit.
	OUTL(pHWHead, pCr, LPC32XX_HSU_TX_TL8B | LPC32XX_HSU_RX_TL32B |LPC32XX_HSU_OFFSET(20) | LPC32XX_HSU_TMO_INACT_4B);

	InitializeCriticalSection(&(pHWHead->TransmitCritSec));
	InitializeCriticalSection(&(pHWHead->RegCritSec));
	pHWHead->PowerDown = FALSE;
	pHWHead->bSuspendResume = FALSE;
	InstallSoftwareISR(pHWHead,pHWHead->pVirtualStaticAddr,RegStride);
	// Clear any interrupts which may be pending.  Normally only
	// happens if we were warm reset.
	ClearPendingInts( pHWHead );
	DumpSerialRegisters(pHWHead);

	DEBUGMSG (ZONE_CLOSE,(TEXT("-SL_INIT, 0x%X\r\n"), pHWHead));
}

/*
   @doc OEM 
   @func PVOID | SL_Init | Initializes device identified by argument.
 *  This routine sets information controlled by the user
 *  such as Line control and baud rate. It can also initialize events and
 *  interrupts, thereby indirectly managing initializing hardware buffers.
 *  Exported only to driver, called only once per process.
 *
 @rdesc The return value is a PVOID to be passed back into the HW
 dependent layer when HW functions are called.
 */
	PVOID
SL_Init(
		ULONG   Identifier, // @parm Device identifier.
		PVOID   pMddHead,   // @parm First argument to mdd callbacks.
		PHWOBJ  pHWObj      // @parm Pointer to our own HW OBJ for this device
	   )
{
	PSER_INFO   pHWHead;

	// Allocate for our main data structure and one of it's fields.
	pHWHead = (PSER_INFO)LocalAlloc( LMEM_ZEROINIT|LMEM_FIXED ,
			sizeof(SER_INFO) );
	RETAILMSG(1,(TEXT("in 16c2550 SL_Init\r\n")));
	if ( !pHWHead )
		return( NULL );

	if ( ! Ser_GetRegistryData(pHWHead, (LPCTSTR)Identifier) ) {
		DEBUGMSG (ZONE_INIT|ZONE_ERROR,
				(TEXT("SerInit - Unable to read registry data.  Failing Init !!! \r\n")));
		pHWHead->pBaseAddress = NULL;   // clear this field so de-init won't call VirtualFree
		goto ALLOCFAILED;
	}

	pHWHead->pMddHead     = pMddHead;
	pHWHead->pHWObj = pHWObj;
	pHWHead->cOpenCount   = 0;

	// Legacy - We have 2 identical fields because registry used to contain SysIntr
	pHWHead->pHWObj->dwIntID = pHWHead->dwSysIntr;
	DEBUGMSG (1|ZONE_INIT,
			(TEXT("SerInit - SYSINTR %d\r\n"),  pHWHead->pHWObj->dwIntID));

	// Set up our Comm Properties data  
	pHWHead->CommProp.wPacketLength		= 0xffff;
	pHWHead->CommProp.wPacketVersion	= 0xffff;
	pHWHead->CommProp.dwServiceMask		= SP_SERIALCOMM;
	pHWHead->CommProp.dwReserved1		= 0;
	pHWHead->CommProp.dwMaxTxQueue		= 16;
	pHWHead->CommProp.dwMaxRxQueue		= 16;
	pHWHead->CommProp.dwMaxBaud			= BAUD_921600;
	pHWHead->CommProp.dwProvSubType		= PST_RS232;

	pHWHead->CommProp.dwProvCapabilities = PCF_INTTIMEOUTS |PCF_SPECIALCHARS | PCF_TOTALTIMEOUTS;

	pHWHead->CommProp.dwSettableBaud	=BAUD_075 | BAUD_110 | BAUD_150 | BAUD_300 | BAUD_600 | BAUD_1200 | 
		BAUD_1800 | BAUD_2400 | BAUD_4800 | BAUD_7200 | BAUD_9600 | BAUD_14400 |
		BAUD_19200 | BAUD_38400 | BAUD_57600 | BAUD_115200 | BAUD_230400 | BAUD_460800 | BAUD_921600;

	pHWHead->CommProp.dwSettableParams	=SP_BAUD;

	pHWHead->CommProp.wSettableData		=DATABITS_8;

	pHWHead->CommProp.wSettableStopParity=STOPBITS_10 |PARITY_NONE;

	pHWHead->fIRMode  = FALSE;   // Select wired by default

	// Init 16550 info, RegStride=2 for AU1200 16bit databus
	DEBUGMSG (ZONE_INIT, (TEXT("SerInit - Init 16550 data\r\n")));
	SL_Init2( pHWHead, pHWHead->pBaseAddress, CPU_REG_STRIDE, EvaluateEventFlag, pMddHead);

	DEBUGMSG (ZONE_INIT,
			(TEXT("SerInit - Disabling UART Power\r\n")));

	return (pHWHead);

ALLOCFAILED:

	LocalFree(pHWHead);
	return (NULL);
}

//
// @doc OEM
// @func void | SL_PostInit | This routine takes care of final initialization.
//
// @rdesc None.
//
	BOOL
SL_PostInit(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_INIT,(TEXT("+SL_PostInit, 0x%X\r\n"), pHWHead));

	// Since we are just a library which might get used for 
	// builtin ports which init at boot, or by PCMCIA ports
	// which init at Open, we can't do anything too fancy.
	// Lets just make sure we cancel any pending interrupts so
	// that if we are being used with an edge triggered PIC, he
	// will see an edge after the MDD hooks the interrupt.
	ClearPendingInts( pHWHead );

	DEBUGMSG (ZONE_INIT,(TEXT("-SL_PostInit, 0x%X\r\n"), pHWHead));
	return(TRUE);
}

//
// @doc OEM 
// @func ULONG | SL_Deinit | De-initializes 16550 device head.  
//
	BOOL
SL_Deinit(
		PVOID   pHead // @parm points to device head
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_CLOSE,(TEXT("+SL_DEINIT, 0x%X\r\n"), pHWHead));
	UninstallSoftwareISR( pHWHead);
	DeleteCriticalSection(&(pHWHead->TransmitCritSec));
	DeleteCriticalSection(&(pHWHead->RegCritSec));

	// Free the flushdone event
	if ( pHWHead->FlushDone )
		CloseHandle( pHWHead->FlushDone );

	DEBUGMSG (ZONE_CLOSE,(TEXT("-SL_DEINIT, 0x%X\r\n"), pHWHead));

	return TRUE;
}

//
// @doc OEM
// @func void | SL_ClearDtr | This routine clears DTR.
//
// @rdesc None.
//
	VOID
SL_ClearDTR(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_ClearDTR, 0x%X\r\n"), pHead));
	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pMCR);
		OUTL((PSER16550_INFO)pHead, pMCR, byte & ~SERIAL_MCR_DTR);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_ClearDTR, 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func VOID | SL_SetDTR | This routine sets DTR.
// 
// @rdesc None.
//
	VOID
SL_SetDTR(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{    
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_SetDTR, 0x%X\r\n"), pHead));
	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pMCR);
		OUTL((PSER16550_INFO)pHead, pMCR, byte | SERIAL_MCR_DTR);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_SetDTR, 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func VOID | SL_ClearRTS | This routine clears RTS.
// 
// @rdesc None.
// 
	VOID
SL_ClearRTS(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_ClearRTS, 0x%X\r\n"), pHead));
	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pMCR);
		OUTL((PSER16550_INFO)pHead, pMCR, byte & ~SERIAL_MCR_RTS);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_ClearRTS, 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func VOID | SL_SetRTS | This routine sets RTS.
// 
// @rdesc None.
//
	VOID
SL_SetRTS(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_SetRTS, 0x%X\r\n"), pHead));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pMCR);
		OUTL((PSER16550_INFO)pHead, pMCR, byte | SERIAL_MCR_RTS);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_SetRTS, 0x%X\r\n"), pHead));
}

/*
   @doc OEM
   @func BOOL | SerEnableIR | This routine enables ir.
 *  Not exported to users, only to driver.
 *
 @rdesc Returns TRUE if successful, FALSEotherwise.
 */
static
	BOOL
SL_EnableIR(
		PVOID   pHead, // @parm PVOID returned by Serinit.
		ULONG   BaudRate  // @parm PVOID returned by HWinit.
		)
{
	return (TRUE);
}

/*
   @doc OEM
   @func BOOL | SerDisableIR | This routine disable the ir.
 *  Not exported to users, only to driver.
 *
 @rdesc Returns TRUE if successful, FALSEotherwise.
 */
static
	BOOL
SL_DisableIR(
		PVOID   pHead /*@parm PVOID returned by Serinit. */
		)
{
	return (TRUE);
}

//
// @doc OEM
// @func VOID | SL_ClearBreak | This routine clears break.
// 
// @rdesc None.
// 
	VOID
SL_ClearBreak(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_ClearBreak, 0x%X\r\n"), pHead));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pLCR);
		OUTL((PSER16550_INFO)pHead, pLCR, byte & ~SERIAL_LCR_BREAK);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_ClearBreak, 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func VOID | SL_SetBreak | This routine sets break.
// 
// @rdesc None.
//
	VOID
SL_SetBreak(
		PVOID   pHead // @parm PVOID returned by HWinit.
		)
{
	PSER16550_INFO   pHWHead   = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_SetBreak, 0x%X\r\n"), pHead));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		unsigned char byte;

		byte = INL((PSER16550_INFO)pHead, pLCR);
		OUTL((PSER16550_INFO)pHead, pLCR, byte | SERIAL_LCR_BREAK);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_SetBreak, 0x%X\r\n"), pHead));
}

//
// SetBaudRate
//
// Internal function.  The only real reason for splitting this out
// is so that we can call it from PowerOn and still allow SL_SetBaud
// to do debug messages, acquire critical sections, etc.
//
	BOOL
SetBaudRate(
		PVOID   pHead,
		ULONG   BaudRate
		)
{
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;
	InterruptMask(pHWHead->dwSysIntr,TRUE);
	set_div(pHWHead,BaudRate);
	InterruptMask(pHWHead->dwSysIntr,FALSE);
	return TRUE;
}

//
// @doc OEM
// @func BOOL | SL_SetBaudRate |
//  This routine sets the baud rate of the device.
//
// @rdesc None.
//
	BOOL
SL_SetBaudRate(
		PVOID   pHead,    // @parm     PVOID returned by HWInit
		ULONG   BaudRate    // @parm     ULONG representing decimal baud rate.
		)
{
	BOOL fRet;
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetbaudRate 0x%X, x%X\r\n"), pHead, BaudRate));

	try {
		// Enter critical section before calling function, since
		// we can't make sys calls inside SetBaudRate
		EnterCriticalSection(&(pHWHead->RegCritSec));
		fRet = SetBaudRate(pHead, BaudRate);
		LeaveCriticalSection(&(pHWHead->RegCritSec));
	}except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		return( FALSE );
	}

	if ( fRet ) {
		pHWHead->dcb.BaudRate = BaudRate;

		DEBUGMSG (ZONE_FUNCTION,
				(TEXT("-SL_SetbaudRate 0x%X (%d Baud)\r\n"),
				 pHead, BaudRate));
		return( TRUE );
	} else {
		DEBUGMSG (ZONE_FUNCTION | ZONE_ERROR,
				(TEXT("-SL_SetbaudRate - Error setting %d, failing to %d\r\n"),
				 BaudRate, pHWHead->dcb.BaudRate) );
		return( FALSE );
	}
}

//
// @doc OEM
// @func BOOL | SL_SetByteSize |
//  This routine sets the WordSize of the device.
//
// @rdesc None.
//
	BOOL
SL_SetByteSize(
		PVOID   pHead,        // @parm     PVOID returned by HWInit
		ULONG   ByteSize    // @parm     ULONG ByteSize field from DCB.
		)
{
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;
	UINT8 lcr;
	BOOL bRet;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetByteSize 0x%X, x%X\r\n"), pHead, ByteSize));

	bRet = TRUE;

	EnterCriticalSection(&(pHWHead->RegCritSec));
	try {
		lcr = INL(pHWHead, pLCR);
		lcr &= ~SERIAL_DATA_MASK;
		switch ( ByteSize ) {
			case 5:
				lcr |= SERIAL_5_DATA;
				break;
			case 6:
				lcr |= SERIAL_6_DATA;
				break;
			case 7:
				lcr |= SERIAL_7_DATA;
				break;
			case 8:
				lcr |= SERIAL_8_DATA;
				break;
			default:
				bRet = FALSE;
				break;
		}
		if (bRet) {
			OUTL(pHWHead, pLCR, lcr);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		bRet = FALSE;
	}
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_SetByteSize 0x%X\r\n"), pHead));

	return(bRet);
}
//
// @doc OEM
// @func BOOL | SL_SetParity |
//  This routine sets the parity of the device.
//
// @rdesc None.
//
	BOOL
SL_SetParity(
		PVOID   pHead,    // @parm     PVOID returned by HWInit
		ULONG   Parity    // @parm     ULONG parity field from DCB.
		)
{
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;
	UINT8 lcr;
	BOOL bRet;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetParity 0x%X, x%X\r\n"), pHead, Parity));

	bRet = TRUE;

	EnterCriticalSection(&(pHWHead->RegCritSec));
	try {
		lcr = INL(pHWHead, pLCR);
		lcr &= ~SERIAL_PARITY_MASK;
		switch ( Parity ) {
			case ODDPARITY:
				lcr |= SERIAL_ODD_PARITY;
				break;

			case EVENPARITY:
				lcr |= SERIAL_EVEN_PARITY;
				break;

			case MARKPARITY:
				lcr |= SERIAL_MARK_PARITY;
				break;

			case SPACEPARITY:
				lcr |= SERIAL_SPACE_PARITY;
				break;

			case NOPARITY:
				lcr |= SERIAL_NONE_PARITY;
				break;
			default:
				bRet = FALSE;
				break;
		}
		if (bRet) {
			OUTL(pHWHead, pLCR, lcr);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		bRet = FALSE;
	}
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_SetParity 0x%X\r\n"), pHead));

	return(bRet);
}
//
// @doc OEM
// @func VOID | SL_SetStopBits |
//  This routine sets the Stop Bits for the device.
//
// @rdesc None.
//
	BOOL
SL_SetStopBits(
		PVOID   pHead,      // @parm     PVOID returned by HWInit
		ULONG   StopBits  // @parm     ULONG StopBits field from DCB.
		)
{
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;
	UINT8 lcr;
	BOOL bRet;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetStopBits 0x%X, x%X\r\n"), pHead, StopBits));

	bRet = TRUE;

	EnterCriticalSection(&(pHWHead->RegCritSec));
	lcr = INL(pHWHead, pLCR);
	lcr &= ~SERIAL_STOP_MASK;

	try {
		// Note that 1.5 stop bits only works if the word size
		// is 5 bits.  Any other xmit word size will cause the
		// 1.5 stop bit setting to generate 2 stop bits.
		switch ( StopBits ) {
			case ONESTOPBIT :
				lcr |= SERIAL_1_STOP ;
				break;
			case ONE5STOPBITS :
				lcr |= SERIAL_1_5_STOP ;
				break;
			case TWOSTOPBITS :
				lcr |= SERIAL_2_STOP ;
				break;
			default:
				bRet = FALSE;
				break;
		}

		if (bRet) {
			OUTL(pHWHead, pLCR, lcr);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		bRet = FALSE;
	}

	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_SetStopBits 0x%X\r\n"), pHead));

	return(bRet);
}

//
// @doc OEM
// @func ULONG | SL_GetRxBufferSize | This function returns
// the size of the hardware buffer passed to the interrupt
// initialize function.  It would be used only for devices
// which share a buffer between the MDD/PDD and an ISR.
//
// 
// @rdesc This routine always returns 0 for 16550 UARTS.
// 
	ULONG
SL_GetRxBufferSize(
		PVOID pHead
		)
{
	return(0);
}

//
// @doc OEM
// @func PVOID | SC_GetRxStart | This routine returns the start of the hardware
// receive buffer.  See SL_GetRxBufferSize.
// 
// @rdesc The return value is a pointer to the start of the device receive buffer.
// 
	PVOID
SL_GetRxStart(
		PVOID   pHead // @parm PVOID returned by SC_init.
		)
{
	return(NULL);
}

//
// @doc OEM
// @func ULONG | SL_GetGetInterruptType | This function is called
//   by the MDD whenever an interrupt occurs.  The return code
//   is then checked by the MDD to determine which of the four
//   interrupt handling routines are to be called.
// 
// @rdesc This routine returns a bitmask indicating which interrupts
//   are currently pending.
// 
	INTERRUPT_TYPE
SL_GetInterruptType(
		PVOID pHead      // Pointer to hardware head
		)
{
	PSER16550_INFO    pHWHead = (PSER16550_INFO)pHead;
	INTERRUPT_TYPE interrupts=INTR_NONE;
	BOOL bContinue;

	DEBUGMSG (0,
			(TEXT("+SL_GetInterruptType 0x%X\r\n"), pHead));
	do {
		bContinue = FALSE;
		if (pHWHead->bSuspendResume) {
			pHWHead->bSuspendResume = FALSE;
			pHWHead->EventCallback( pHWHead->pMddHead, EV_POWER );
		}
		if (pHWHead->pIsrInfoVirt!=NULL) { 
			if ( pHWHead->pReceiveBuffer && GetRcvDataSize(pHWHead->pReceiveBuffer)>0) {
				pHWHead->IIR=pHWHead->pReceiveBuffer->bBuffer[pHWHead->pReceiveBuffer->dwFIFO_Out].bIntFlag;
			}
			else
				pHWHead->IIR=SERIAL_IIR_INT_INVALID;
		}
		else
			try {
				pHWHead->IIR = INL(pHWHead, pIIR_FCR);
			}
		except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
				EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
			pHWHead->IIR = SERIAL_IIR_INT_INVALID; // simulate no interrupt
		}
		DEBUGMSG (ZONE_THREAD,
				(TEXT("+SL_GetInterruptType IIR=%x\r\n"),pHWHead->IIR ));
		if ( pHWHead->IIR & SERIAL_IIR_INT_INVALID ) {
			// No interrupts pending, vector is useless
			interrupts = INTR_NONE;
		} else {

			// The interrupt value is valid
			switch ( pHWHead->IIR & SERIAL_IIR_INT_MASK ) {
				case SERIAL_IIR_RLS:
					if (pHWHead->pIsrInfoVirt) {
						pHWHead->LSR=pHWHead->pReceiveBuffer->bBuffer[pHWHead->pReceiveBuffer->dwFIFO_Out].bIntData;
						pHWHead->pReceiveBuffer->dwFIFO_Out=IncRcvIndex(pHWHead->pReceiveBuffer,pHWHead->pReceiveBuffer->dwFIFO_Out);
						ProcessLSR (pHWHead);
						bContinue=TRUE;
					}
					else
						interrupts |= INTR_LINE;
					break;
				case SERIAL_IIR_CTI:
				case SERIAL_IIR_CTI_2:
				case SERIAL_IIR_RDA:
					interrupts = INTR_RX;
					break;

				case SERIAL_IIR_THRE :
					interrupts = INTR_TX;
					if (pHWHead->pIsrInfoVirt) {
						pHWHead->pReceiveBuffer->dwFIFO_Out=IncRcvIndex(pHWHead->pReceiveBuffer,pHWHead->pReceiveBuffer->dwFIFO_Out);
					}
					break;
				case SERIAL_IIR_MS :
					if (pHWHead->pIsrInfoVirt) {
						pHWHead->MSR=pHWHead->pReceiveBuffer->bBuffer[pHWHead->pReceiveBuffer->dwFIFO_Out].bIntData;
						pHWHead->pReceiveBuffer->dwFIFO_Out=IncRcvIndex(pHWHead->pReceiveBuffer,pHWHead->pReceiveBuffer->dwFIFO_Out);
						ProcessMSR (pHWHead);
						bContinue=TRUE;
					}
					else
						interrupts = INTR_MODEM;
					break;

				default:
					if (pHWHead->pIsrInfoVirt) {
						pHWHead->pReceiveBuffer->dwFIFO_Out=IncRcvIndex(pHWHead->pReceiveBuffer,pHWHead->pReceiveBuffer->dwFIFO_Out);
					}
					interrupts = INTR_NONE;
					break;
			}
		}
		if (pHWHead->pIsrInfoVirt) {
			if (pHWHead->pReceiveBuffer && GetRcvDataSize(pHWHead->pReceiveBuffer)>0) { //Data available.
				interrupts |= INTR_RX;
			};
		}
		if (pHWHead->AddTXIntr) {
			interrupts |= INTR_TX;
			pHWHead->AddTXIntr = FALSE;
		}
	}while (bContinue && interrupts == INTR_NONE);
	DEBUGMSG (ZONE_THREAD,
			(TEXT("-SL_GetInterruptType 0x%X, 0x%X\r\n"),
			 pHead, interrupts));

	return(interrupts);
}


// @doc OEM
// @func ULONG | SL_RxIntr | This routine gets several characters from the hardware
//   receive buffer and puts them in a buffer provided via the second argument.
//   It returns the number of bytes lost to overrun.
// 
// @rdesc The return value indicates the number of overruns detected.
//   The actual number of dropped characters may be higher.
//
	ULONG
SL_RxIntr(
		PVOID pHead,                // @parm Pointer to hardware head
		PUCHAR pRxBuffer,           // @parm Pointer to receive buffer
		ULONG *pBufflen             // @parm In = max bytes to read, out = bytes read
		)
{
	PSER16550_INFO   pHWHead    = (PSER16550_INFO)pHead;
	ULONG        RetVal    = 0;
	ULONG        TargetRoom    = *pBufflen;
	BOOL        fRXFlag = FALSE;
	BOOL        fReplaceparityErrors = FALSE;
	BOOL        fNull;
	UCHAR       cEvtChar, cRXChar;


	*pBufflen = 0;

	// LAM - I have local copies of some DCB elements since I don't
	// want to keep dereferencing inside my read loop and there are too
	// many of them to trust the compiler.
	cEvtChar = pHWHead->dcb.EvtChar;
	fNull = pHWHead->dcb.fNull;
	if ( pHWHead->dcb.fErrorChar && pHWHead->dcb.fParity )
		fReplaceparityErrors = TRUE;

#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		while ( TargetRoom ) {
			// See if there is another byte to be read
			if (pHWHead->pIsrInfoVirt && pHWHead->pReceiveBuffer) { // Soft FIFO
				if (GetRcvDataSize(pHWHead->pReceiveBuffer)>0 ) {
					UCHAR uIntFlag = pHWHead->pReceiveBuffer->bBuffer[pHWHead->pReceiveBuffer->dwFIFO_Out].bIntFlag & SERIAL_IIR_INT_MASK;
					if ( uIntFlag==SERIAL_IIR_CTI || uIntFlag==SERIAL_IIR_CTI_2 || uIntFlag==SERIAL_IIR_RDA ) {
						cRXChar = pHWHead->pReceiveBuffer->bBuffer[pHWHead->pReceiveBuffer->dwFIFO_Out].bIntData;
						pHWHead->pReceiveBuffer->dwFIFO_Out=IncRcvIndex(pHWHead->pReceiveBuffer,pHWHead->pReceiveBuffer->dwFIFO_Out);
					}
					else
						break;
				}
				else
					break;
			}
			else {
				ReadLSR( pHWHead );
				if ( pHWHead->LSR & SERIAL_LSR_DR ) {
					// Read the byte
					cRXChar = INL(pHWHead, pData);
					//NKDbgPrintfW(TEXT("%c"), cRXChar);
				}
				else
					break;
			}
			// But we may want to discard it
			if ( pHWHead->dcb.fDsrSensitivity &&
					(! (pHWHead->MSR & SERIAL_MSR_DSR)) ) {
				// Do nothing - byte gets discarded
				DEBUGMSG (ZONE_FLOW,
						(TEXT("Dropping byte because DSR is low\r\n")));
			} else if (!cRXChar && fNull) {
				// Do nothing - byte gets discarded
				DEBUGMSG (ZONE_FLOW| ZONE_WARN,
						(TEXT("Dropping NULL byte due to fNull\r\n")));
			} else {
				// Do character replacement if parity error detected.
				if ( fReplaceparityErrors && (pHWHead->LSR & SERIAL_LSR_PE) ) {
					cRXChar = pHWHead->dcb.ErrorChar;
				} else {
					// See if we need to generate an EV_RXFLAG for the received char.
					if ( cRXChar == cEvtChar )
						fRXFlag = TRUE;
				}

				// Finally, we can get byte, update status and save.
				*pRxBuffer++ = cRXChar;
				(*pBufflen)++;

				--TargetRoom;
			}
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// just exit
	}
#pragma prefast(pop)

	// if we saw one (or more) EVT chars, then generate an event
	if ( fRXFlag )
		pHWHead->EventCallback( pHWHead->pMddHead, EV_RXFLAG );

	if ( pHWHead->DroppedBytes )
		DEBUGMSG (ZONE_WARN, (TEXT("Rx drop %d.\r\n"),
					pHWHead->DroppedBytes));

	DEBUGMSG (ZONE_READ, (TEXT("-GetBytes - rx'ed %d, dropped %d.\r\n"),
				*pBufflen,
				pHWHead->DroppedBytes));

	RetVal = pHWHead->DroppedBytes;
	pHWHead->DroppedBytes = 0;
	return(RetVal);
}

//
// @doc OEM
// @func ULONG | SL_TXIntrEx | This routine is called from the new MDD
//   whenever INTR_TX is returned by SL_GetInterruptType
// 
// @rdesc None
//
	VOID
SL_TxIntrEx(
		PVOID pHead,                // Hardware Head
		PUCHAR pTxBuffer,          // @parm Pointer to transmit buffer
		ULONG *pBufflen            // @parm In = max bytes to transmit, out = bytes transmitted
		)
{
	PSER16550_INFO   pHWHead    = (PSER16550_INFO)pHead;
	ULONG NumberOfBytes = *pBufflen;

	DEBUGMSG (ZONE_THREAD, (TEXT("Transmit Event\r\n")));

	DEBUGMSG (ZONE_WRITE,
			(TEXT("+SL_TxIntrEx 0x%X, Len %d\r\n"), pHead, *pBufflen));


	// We may be done sending.  If so, just disable the TX interrupts
	// and return to the MDD.  
	if( ! *pBufflen ) {
		DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx: Disable INTR_TX.\r\n")));
		if (pHWHead->pIsrInfoVirt && pHWHead->pXmitBuffer) {
			pHWHead->bMoreXmitData=FALSE;
			while (pHWHead->pXmitBuffer->dwFIFO_In!= pHWHead->pXmitBuffer->dwFIFO_Out) {
				RETAILMSG(1,(TEXT("!!!SL_TxIntrEx: We found lost Xmit FIFO Data \r\n")));
				pHWHead->pXmitBuffer->dwFIFO_In= pHWHead->pXmitBuffer->dwFIFO_Out;
				OUTL(pHWHead, pIIR_FCR, pHWHead->FCR | SERIAL_FCR_TXMT_RESET);
			}
		}
		OUTL(pHWHead, pIER, IER_NORMAL_INTS);
		return;
	}


	*pBufflen = 0;  // In case we don't send anything below.

	// Disable xmit intr.  Most 16550s will keep hammering
	// us with xmit interrupts if we don't turn them off
	// Whoever gets the FlushDone will then need to turn
	// TX Ints back on if needed.
	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		// Need to signal FlushDone for XmitComChar
		PulseEvent(pHWHead->FlushDone);

		pHWHead->CommErrors &= ~CE_TXFULL;

		// If CTS flow control is desired, check cts. If clear, don't send,
		// but loop.  When CTS comes back on, the OtherInt routine will
		// detect this and re-enable TX interrupts (causing Flushdone).
		// For finest granularity, we would check this in the loop below,
		// but for speed, I check it here (up to 8 xmit characters before
		// we actually flow off.
		if ( pHWHead->dcb.fOutxCtsFlow ) {
			// ReadMSR( pHWHead );
			// We don't need to explicitly read the MSR, since we always enable
			// IER_MS, which ensures that we will get an interrupt and read
			// the MSR whenever CTS, DSR, TERI, or DCD change.

			if (! (pHWHead->MSR & SERIAL_MSR_CTS) ) {
				unsigned char byte;
				DEBUGMSG (ZONE_WRITE|ZONE_FLOW,
						(TEXT("SL_TxIntrEx, flowed off via CTS\n") ) );
				pHWHead->CTSFlowOff = TRUE;  // Record flowed off state
				if (pHWHead->pIsrInfoVirt==NULL || pHWHead->pXmitBuffer==NULL || GetXmitDataSize(pHWHead->pXmitBuffer)==0)  {// no data inbuffer. 
					byte = INL(pHWHead, pIER);
					OUTL(pHWHead, pIER, byte & ~SERIAL_IER_THR); // disable TX interrupts while flowed off
				}
				// We could return a positive value here, which would
				// cause the MDD to periodically check the flow control
				// status.  However, we don't need to since we know that
				// the DCTS interrupt will cause the MDD to call us, and we
				// will subsequently fake a TX interrupt to the MDD, causing
				// him to call back into PutBytes.

				LeaveCriticalSection(&(pHWHead->RegCritSec));
				return;
			}
		}

		// Same thing applies for DSR
		if ( pHWHead->dcb.fOutxDsrFlow ) {
			// ReadMSR( pHWHead );
			// We don't need to explicitly read the MSR, since we always enable
			// IER_MS, which ensures that we will get an interrupt and read
			// the MSR whenever CTS, DSR, TERI, or DCD change.

			if (! (pHWHead->MSR & SERIAL_MSR_DSR) ) {
				DEBUGMSG (ZONE_WRITE|ZONE_FLOW,
						(TEXT("SL_TxIntrEx, flowed off via DSR\n") ) );
				pHWHead->DSRFlowOff = TRUE;  // Record flowed off state
				if (pHWHead->pIsrInfoVirt==NULL || pHWHead->pXmitBuffer==NULL || GetXmitDataSize(pHWHead->pXmitBuffer)==0) { // no data inbuffer. 
					OUTL(pHWHead, pIER, IER_NORMAL_INTS); // disable TX interrupts while flowed off
				}
				// See the comment above above positive return codes.

				LeaveCriticalSection(&(pHWHead->RegCritSec));
				return;
			}
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Do nothing.  The worst case is that this was a fluke,
		// and a TX Intr will come right back at us and we will
		// resume transmission.
	}
#pragma prefast(pop)

	LeaveCriticalSection(&(pHWHead->RegCritSec));


	//  OK, now lets actually transmit some data.
	DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx wait for CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));
	EnterCriticalSection(&(pHWHead->TransmitCritSec));
	DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx got CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		if (pHWHead->pIsrInfoVirt==NULL || pHWHead->pXmitBuffer==NULL || GetXmitDataSize(pHWHead->pXmitBuffer)==0) {// no data inbuffer. 
			ReadLSR( pHWHead );
			if ( pHWHead->LSR & SERIAL_LSR_THRE ) {
				UCHAR       byteCount;
				if ( pHWHead->IIR & SERIAL_IIR_FIFOS_ENABLED )
					byteCount = SERIAL_FIFO_DEPTH;
				else
					byteCount = 1;

				DEBUGMSG (ZONE_WRITE | ZONE_THREAD,
						(TEXT("SL_TxIntrEx - Write max of %d bytes\r\n"),
						 byteCount));
				if (pHWHead->pIsrInfoVirt!=NULL && pHWHead->pXmitBuffer!=NULL) {
					UCHAR bFirstByte=*pTxBuffer;
					NumberOfBytes--;
					++pTxBuffer;
					(*pBufflen)++;
					// Fill up the software ISR buffer.
					OUTL(pHWHead, pIER, IER_NORMAL_INTS );
					while (GetXmitAvailableBuffer(pHWHead->pXmitBuffer)>min(pHWHead->pXmitBuffer->dwWaterMark,2) &&
							NumberOfBytes) {
						pHWHead->pXmitBuffer->bBuffer[pHWHead->pXmitBuffer->dwFIFO_In]=*pTxBuffer;
						pHWHead->pXmitBuffer->dwFIFO_In=IncXmitIndex(pHWHead->pXmitBuffer,pHWHead->pXmitBuffer->dwFIFO_In);
						++pTxBuffer;
						(*pBufflen)++;
						NumberOfBytes--;
					}
					DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx - Write %d bytes to FIFO\r\n"), (*pBufflen)));
					OUTL(pHWHead, pData, bFirstByte);
					OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR);
					pHWHead->bMoreXmitData=(NumberOfBytes==0?FALSE:TRUE);
				}
				else
					for ( *pBufflen=0; NumberOfBytes && byteCount; NumberOfBytes--, byteCount-- ) {
						DEBUGLED( ZONE_WRITE, (1, 0x10200000 | *pTxBuffer) );
						OUTL(pHWHead, pData, *pTxBuffer);
						InterruptDone(pHWHead->dwSysIntr);
						++pTxBuffer;
						(*pBufflen)++;
					}
			}
		}
		// Enable xmit intr. We need to do this no matter what, 
		// since the MDD relies on one final interrupt before
		// returning to the application. 
		DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx: Enable INTR_TX.\r\n")));
		OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Hmm, not sure what would cause this.  Lets just tell
		// the MDD to go away until we get another TX
		// interrupt.
	}
#pragma prefast(pop)

	LeaveCriticalSection(&(pHWHead->RegCritSec));

	LeaveCriticalSection(&(pHWHead->TransmitCritSec));
	DEBUGMSG (ZONE_WRITE, (TEXT("SL_TxIntrEx released CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));

	DEBUGMSG (ZONE_WRITE, (TEXT("-SL_TxIntrEx - sent %d.\r\n"),
				*pBufflen));
	return;

}

//
// @doc OEM
// @func ULONG | SL_LineIntr | This routine is called from the MDD
//   whenever INTR_LINE is returned by SL_GetInterruptType.
// 
// @rdesc None
//
	VOID
SL_LineIntr(
		PVOID pHead                // Hardware Head
		)
{
	PSER16550_INFO   pHWHead    = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_READ,
			(TEXT("+SL_LineIntr 0x%X\r\n"), pHead));
	ReadLSR( pHWHead );
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		OUTL(pHWHead, pIIR_FCR, pHWHead->FCR| SERIAL_FCR_RCVR_RESET ); // We have to reset Receive FIFO because it has error.
		while (INL(pHWHead, pLSR) & SERIAL_LSR_DR ) {
			INL(pHWHead, pData);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		;
	};
#pragma prefast(pop)
	DEBUGMSG (ZONE_READ,
			(TEXT("-SL_LineIntr 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func ULONG | SL_OtherIntr | This routine is called from the MDD
//   whenever INTR_MODEM is returned by SL_GetInterruptType.
// 
// @rdesc None
//
	VOID
SL_OtherIntr(
		PVOID pHead                // Hardware Head
		)
{
	PSER16550_INFO   pHWHead    = (PSER16550_INFO)pHead;

	DEBUGMSG (0,
			(TEXT("+SL_OtherIntr 0x%X\r\n"), pHead));

	ReadMSR( pHWHead );

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		// If we are currently flowed off via CTS or DSR, then
		// we better signal the TX thread when one of them changes
		// so that TX can resume sending.
		if ( pHWHead->DSRFlowOff && (pHWHead->MSR & SERIAL_MSR_DSR) ) {
			DEBUGMSG (ZONE_WRITE|ZONE_FLOW,
					(TEXT("PutBytes, flowed on via DSR\n") ) );
			pHWHead->DSRFlowOff = FALSE;
			// DSR is set, so go ahead and resume sending
			OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR); // Enable xmit intr.
			// Then simulate a TX intr to get things moving
			pHWHead->AddTXIntr = TRUE;
		}
		if ( pHWHead->CTSFlowOff && (pHWHead->MSR & SERIAL_MSR_CTS) ) {
			DEBUGMSG (ZONE_WRITE|ZONE_FLOW,
					(TEXT("PutBytes, flowed on via CTS\n") ) );
			pHWHead->CTSFlowOff = FALSE;
			// CTS is set, so go ahead and resume sending
			OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR); // Enable xmit intr.
			// Then simulate a TX intr to get things moving
			pHWHead->AddTXIntr = TRUE;
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)

	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (0,
			(TEXT("-SL_OtherIntr 0x%X\r\n"), pHead));
}


//
// @doc OEM
// @func ULONG | SL_OtherIntr | This routine is called from the MDD
//   whenever INTR_MODEM is returned by SL_GetInterruptType.
// 
// @rdesc None
//
	VOID
SL_ModemIntr(
		PVOID pHead                // Hardware Head
		)
{
	SL_OtherIntr(pHead);
}

//  
// @doc OEM
// @func    ULONG | SL_GetStatus | This structure is called by the MDD
//   to retrieve the contents of a COMSTAT structure.
//
// @rdesc    The return is a ULONG, representing success (0) or failure (-1).
//
	ULONG
SL_GetStatus(
		PVOID    pHead,    // @parm PVOID returned by HWInit.
		LPCOMSTAT    lpStat    // Pointer to LPCOMMSTAT to hold status.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;
	ULONG      RetVal  = pHWHead->CommErrors;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_GetStatus 0x%X\r\n"), pHead));

	pHWHead->CommErrors = 0; // Clear old errors each time

	if ( lpStat ) {
		try {
			if (pHWHead->CTSFlowOff)
				pHWHead->Status.fCtsHold = 1;
			else
				pHWHead->Status.fCtsHold = 0;

			if (pHWHead->DSRFlowOff)
				pHWHead->Status.fDsrHold = 1;
			else
				pHWHead->Status.fDsrHold = 0;

			// NOTE - I think what they really want to know here is
			// the amount of data in the MDD buffer, not the amount
			// in the UART itself.  Just set to 0 for now since the
			// MDD doesn't take care of this.
			pHWHead->Status.cbInQue  = 0;
			pHWHead->Status.cbOutQue = 0;

			memcpy(lpStat, &(pHWHead->Status), sizeof(COMSTAT));
		}
		except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
				EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
			RetVal = (ULONG)-1;
		}        
	} else
		RetVal = (ULONG)-1;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_GetStatus 0x%X\r\n"), pHead));
	return(RetVal);
}

//
// @doc OEM
// @func    ULONG | SL_Reset | Perform any operations associated
//   with a device reset
//
// @rdesc    None.
//
	VOID
SL_Reset(
		PVOID   pHead    // @parm PVOID returned by HWInit.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_Reset 0x%X\r\n"), pHead));

	memset(&pHWHead->Status, 0, sizeof(COMSTAT));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
		OUTL(pHWHead, pIER, IER_NORMAL_INTS);
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Do nothing
	}
#pragma prefast(pop)
	if (pHWHead->pIsrInfoVirt) {
		if (pHWHead->pReceiveBuffer)
			pHWHead->pReceiveBuffer->dwFIFO_Out=pHWHead->pReceiveBuffer->dwFIFO_In ;
		while (pHWHead->pXmitBuffer && pHWHead->pXmitBuffer->dwFIFO_In!= pHWHead->pXmitBuffer->dwFIFO_Out)
			pHWHead->pXmitBuffer->dwFIFO_In= pHWHead->pXmitBuffer->dwFIFO_Out;
	}
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_Reset 0x%X\r\n"), pHead));
}

//
// @doc OEM
// @func    VOID | SL_GetModemStatus | Retrieves modem status.
//
// @rdesc    None.
//
	VOID
SL_GetModemStatus(
		PVOID   pHead,        // @parm PVOID returned by HWInit.
		PULONG  pModemStatus    // @parm PULONG passed in by user.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;
	UINT8 ubModemStatus;


	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_GetModemStatus 0x%X\r\n"), pHead));

	ReadMSR( pHWHead );
	ubModemStatus = pHWHead->MSR;

	if ( ubModemStatus & SERIAL_MSR_CTS )
		*pModemStatus |= MS_CTS_ON;

	if ( ubModemStatus & SERIAL_MSR_DSR )
		*pModemStatus |= MS_DSR_ON;

	if ( ubModemStatus & SERIAL_MSR_RI )
		*pModemStatus |= MS_RING_ON;

	if ( ubModemStatus & SERIAL_MSR_DCD )
		*pModemStatus |= MS_RLSD_ON;

	DEBUGMSG (ZONE_FUNCTION | ZONE_EVENTS,
			(TEXT("-SL_GetModemStatus 0x%X (stat x%X) \r\n"), pHead, *pModemStatus));
	return;
}


//------------------------------------------------------------------------------

	VOID
SL_GetCommProperties(PVOID pContext, COMMPROP *pCommProp)
{

	DEBUGMSG(ZONE_FUNCTION, (L"+ser16550::SL_GetCommProper 0x%x\n", pContext));

	memset(pCommProp, 0, sizeof(COMMPROP));
	pCommProp->wPacketLength = 0xffff;
	pCommProp->wPacketVersion = 0xffff;
	pCommProp->dwServiceMask = SP_SERIALCOMM;
	pCommProp->dwMaxTxQueue = 16;
	pCommProp->dwMaxRxQueue = 16;
	pCommProp->dwMaxBaud = BAUD_921600;
	pCommProp->dwProvSubType = PST_RS232;

	pCommProp->dwProvCapabilities = PCF_INTTIMEOUTS |  PCF_SPECIALCHARS | PCF_TOTALTIMEOUTS;

	pCommProp->dwSettableParams = SP_BAUD;

	pCommProp->dwSettableBaud = BAUD_075 | BAUD_110 | BAUD_150 | BAUD_300 | BAUD_600 | BAUD_1200 | 
		BAUD_1800 | BAUD_2400 | BAUD_4800 | BAUD_7200 | BAUD_9600 | BAUD_14400 |
		BAUD_19200 | BAUD_38400 | BAUD_57600 | BAUD_115200 | BAUD_230400 | BAUD_460800 | BAUD_921600;
	pCommProp->wSettableData = DATABITS_8;

	pCommProp->wSettableStopParity = STOPBITS_10 | PARITY_NONE;

	DEBUGMSG(ZONE_FUNCTION, (L"-ser16550::SL_GetCommProper\n"));
}

//
// @doc OEM
// @func    VOID | SL_PurgeComm | Purge RX and/or TX
// 
// @rdesc    None.
//

	VOID
SL_PurgeComm(
		PVOID   pHead,        // @parm PVOID returned by HWInit.
		DWORD   fdwAction        // @parm Action to take. 
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_PurgeComm 0x%X\r\n"), pHead));

	EnterCriticalSection(&(pHWHead->RegCritSec));
#pragma prefast(disable: 322, "Recover gracefully from hardware failure")
	try {
#ifdef TODO
		// REVIEW THIS - I don't see how this could have terminated a pending read,
		// nor how RX interrupts would ever get turned back on.  I suspect that
		// RXABORT and TXABORT would both be better implemented in the MDD.
		if ( fdwAction & PURGE_RXABORT )
			OUTL(pHWHead, pIER, IER_NORMAL_INTS & ~SERIAL_IER_RDA);
#endif    
		if ( fdwAction & PURGE_TXCLEAR ) {
			// Write the TX reset bit.  It is self clearing
			OUTL(pHWHead, pIIR_FCR, pHWHead->FCR | SERIAL_FCR_TXMT_RESET);
		}

		if ( fdwAction & PURGE_RXCLEAR ) {
			// Write the RX reset bit.  It is self clearing
			OUTL(pHWHead, pIIR_FCR, pHWHead->FCR | SERIAL_FCR_RCVR_RESET);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Just exit
	}
#pragma prefast(pop)
	LeaveCriticalSection(&(pHWHead->RegCritSec));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_PurgeComm 0x%X\r\n"), pHead));
	return;
}

//
// @doc OEM
// @func    BOOL | SL_XmitComChar | Transmit a char immediately
// 
// @rdesc    TRUE if succesful
//
	BOOL
SL_XmitComChar(
		PVOID   pHead,    // @parm PVOID returned by HWInit.
		UCHAR   ComChar   // @parm Character to transmit. 
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_XmitComChar 0x%X\r\n"), pHead));

	// Get critical section, then transmit when buffer empties
	DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar wait for CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));
	EnterCriticalSection(&(pHWHead->TransmitCritSec));
	DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar got CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));
	try {
		while ( TRUE ) {  // We know THR will eventually empty
			EnterCriticalSection(&(pHWHead->RegCritSec));
			// Write the character if we can
			if (pHWHead->pIsrInfoVirt==NULL || pHWHead->pXmitBuffer==NULL || GetXmitDataSize(pHWHead->pXmitBuffer)==0) {// no data inbuffer. 
				ReadLSR( pHWHead );
				if ( pHWHead->LSR & SERIAL_LSR_THRE ) {
					// FIFO is empty, send this character
					OUTL(pHWHead, pData, ComChar);
					// Make sure we release the register critical section
					OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR);
					LeaveCriticalSection(&(pHWHead->RegCritSec));

					DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar wrote x%X\r\n"),
								ComChar));
					break;
				}
			}
			else
				if  (GetXmitAvailableBuffer(pHWHead->pXmitBuffer)>2) { // Not full
					pHWHead->pXmitBuffer->bBuffer[pHWHead->pXmitBuffer->dwFIFO_In]=ComChar;
					pHWHead->pXmitBuffer->dwFIFO_In=IncXmitIndex(pHWHead->pXmitBuffer,pHWHead->pXmitBuffer->dwFIFO_In);
					LeaveCriticalSection(&(pHWHead->RegCritSec));
					DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar wrote x%X to Soft FIFO\r\n"),
								ComChar));
					OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR);
					LeaveCriticalSection(&(pHWHead->RegCritSec));
					break;
				}
			// If we couldn't write the data yet, then wait for a
			// TXINTR to come in and try it again.

			// Enable xmit intr.
			OUTL(pHWHead, pIER, IER_NORMAL_INTS | SERIAL_IER_THR);
			LeaveCriticalSection(&(pHWHead->RegCritSec));

			// Wait until the txintr has signalled.
			DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar WaitIntr x%X\r\n"),
						pHWHead->FlushDone));
			WaitForSingleObject(pHWHead->FlushDone, (ULONG)1000);
		}
	}
	except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ?
			EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
		// Make sure we release the register critical section
		LeaveCriticalSection(&(pHWHead->RegCritSec));
	}

	LeaveCriticalSection(&(pHWHead->TransmitCritSec));
	DEBUGMSG (ZONE_WRITE, (TEXT("XmitComChar released CritSec %x.\r\n"),
				&(pHWHead->TransmitCritSec)));

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_XmitComChar 0x%X\r\n"), pHead));

	return(TRUE);
}

//
// @doc OEM
// @func    BOOL | SL_PowerOff | Perform powerdown sequence.
// 
// @rdesc    TRUE if succesful
//
	BOOL
SL_PowerOff(
		PVOID   pHead        // @parm    PVOID returned by HWInit.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	// Current FCR is already saved in a shadow

	// Current IER is not normally shadowed, save it
	pHWHead->IER = INL(pHWHead, pIER);

	// Current LCR is not normally shadowed, save it
	pHWHead->LCR = INL(pHWHead, pLCR);

	// Current MCR is not normally shadowed, save it
	pHWHead->MCR = INL(pHWHead, pMCR);

	// Current Scratch is not normally shadowed, save it
	pHWHead->Scratch = INL(pHWHead, pScratch);

	pHWHead->PowerDown = TRUE;
	RETAILMSG(1, (TEXT("16550 SL_PowerOff\n")));

	return TRUE;
}

//
// @doc OEM
// @func    BOOL | SL_PowerOn | Perform poweron sequence.
// 
// @rdesc    TRUE if succesful
//
	BOOL
SL_PowerOn(
		PVOID   pHead        // @parm    PVOID returned by HWInit.
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;

	if (pHWHead->PowerDown) {
		// Restore any registers that we need

		// In power handler context, so don't try to do a critical section
		OUTL(pHWHead, pIIR_FCR, pHWHead->FCR);
		OUTL(pHWHead, pIER, pHWHead->IER);
		OUTL(pHWHead, pLCR, pHWHead->LCR);
		OUTL(pHWHead, pMCR, pHWHead->MCR);
		OUTL(pHWHead, pScratch, pHWHead->Scratch);

		pHWHead->PowerDown = FALSE;

		// And we didn't save the Divisor Reg, so set baud rate
		// But don't call SL_SetBaud, since it does DebugMsg.
		// Call our internal function instead.  Can't acquire
		// the RegCritSec, but shouldn't really need to since
		// we are in power context.
		SetBaudRate( pHWHead, pHWHead->dcb.BaudRate );
		pHWHead->bSuspendResume = TRUE;
		SetInterruptEvent(pHWHead->dwSysIntr);
		RETAILMSG(1, (TEXT("16550 SL_PowerOn\n")));
	}

	return TRUE;
}

//
// @doc OEM
// @func    BOOL | SL_SetDCB | Sets new values for DCB.  This
// routine gets a DCB from the MDD.  It must then compare
// this to the current DCB, and if any fields have changed take
// appropriate action.
// 
// @rdesc    BOOL
//
	BOOL
SL_SetDCB(
		PVOID   pHead,        // @parm    PVOID returned by HWInit.
		LPDCB   lpDCB       // @parm    Pointer to DCB structure
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;
	BOOL bRet;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetDCB 0x%X\r\n"), pHead));

	bRet = TRUE;

	// If the device is open, scan for changes and do whatever
	// is needed for the changed fields.  if the device isn't
	// open yet, just save the DCB for later use by the open.
	if ( pHWHead->OpenCount ) {
		// Note, fparity just says whether we should check
		// receive parity.  And the 16550 won't let us NOT
		// check parity if we generate it.  So this field
		// has no effect on the hardware.

		if ( lpDCB->BaudRate != pHWHead->dcb.BaudRate ) {
			bRet = SL_SetBaudRate( pHWHead, lpDCB->BaudRate );
		}

		if ( bRet && (lpDCB->ByteSize != pHWHead->dcb.ByteSize )) {
			bRet = SL_SetByteSize( pHWHead, lpDCB->ByteSize );
		}

		if ( bRet && (lpDCB->Parity != pHWHead->dcb.Parity )) {
			bRet = SL_SetParity( pHWHead, lpDCB->Parity );
		}

		if ( bRet && (lpDCB->StopBits != pHWHead->dcb.StopBits )) {
			bRet = SL_SetStopBits( pHWHead, lpDCB->StopBits );
		}

		// Don't worry about fOutxCtsFlow.  It is a flag which
		// will be examined every time we load the TX buffer.
		// No special action required here.
	}

	if (bRet) {
		// Now that we have done the right thing, store this DCB
		pHWHead->dcb = *lpDCB;
	}

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_SetDCB 0x%X\r\n"), pHead));

	return(bRet);
}

//
// @doc OEM
// @func    BOOL | SL_SetCommTimeouts | Sets new values for the
// CommTimeouts structure. routine gets a DCB from the MDD.  It
// must then compare this to the current DCB, and if any fields
// have changed take appropriate action.
// 
// @rdesc    ULONG
//
	ULONG
SL_SetCommTimeouts(
		PVOID   pHead,        // @parm    PVOID returned by HWInit.
		LPCOMMTIMEOUTS   lpCommTimeouts // @parm Pointer to CommTimeout structure
		)
{
	PSER16550_INFO pHWHead = (PSER16550_INFO)pHead;
	ULONG retval = 0;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("+SL_SetCommTimeout 0x%X\r\n"), pHead));

	// OK, first check for any changes and act upon them
	if ( lpCommTimeouts->WriteTotalTimeoutMultiplier !=
			pHWHead->CommTimeouts.WriteTotalTimeoutMultiplier ) {
	}

	// Now that we have done the right thing, store this DCB
	pHWHead->CommTimeouts = *lpCommTimeouts;

	DEBUGMSG (ZONE_FUNCTION,
			(TEXT("-SL_SetCommTimeout 0x%X\r\n"), pHead));

	return(retval);
}



//
//  @doc OEM
//  @func    BOOL | SL_Ioctl | Device IO control routine.  
//  @parm DWORD | dwOpenData | value returned from COM_Open call
//    @parm DWORD | dwCode | io control code to be performed
//    @parm PBYTE | pBufIn | input data to the device
//    @parm DWORD | dwLenIn | number of bytes being passed in
//    @parm PBYTE | pBufOut | output data from the device
//    @parm DWORD | dwLenOut |maximum number of bytes to receive from device
//    @parm PDWORD | pdwActualOut | actual number of bytes received from device
//
//    @rdesc        Returns TRUE for success, FALSE for failure
//
//  @remark  The MDD will pass any unrecognized IOCTLs through to this function.
//
	BOOL
SL_Ioctl(PVOID pHead, DWORD dwCode,PBYTE pBufIn,DWORD dwLenIn,
		PBYTE pBufOut,DWORD dwLenOut,PDWORD pdwActualOut)
{
	BOOL RetVal = TRUE;
	DEBUGMSG (ZONE_FUNCTION, (TEXT("+SL_Ioctl 0x%X\r\n"), pHead));
	switch (dwCode) {

		// Currently, no defined IOCTLs
		default:
			RetVal = FALSE;
			DEBUGMSG (ZONE_FUNCTION, (TEXT(" Unsupported ioctl 0x%X\r\n"), dwCode));
			break;            
	}
	DEBUGMSG (ZONE_FUNCTION, (TEXT("-SL_Ioctl 0x%X\r\n"), pHead));
	return(RetVal);
}

static HW_VTBL IoVTbl = {
	SL_Init,
	SL_PostInit,
	SL_Deinit,
	SL_Open,
	SL_Close,
	SL_GetInterruptType,
	SL_RxIntr,
	SL_TxIntrEx,
	SL_ModemIntr,
	SL_LineIntr,
	SL_GetRxBufferSize,
	SL_PowerOff,
	SL_PowerOn,
	SL_ClearDTR,
	SL_SetDTR,
	SL_ClearRTS,
	SL_SetRTS,
	SL_EnableIR,
	SL_DisableIR,
	SL_ClearBreak,
	SL_SetBreak,
	SL_XmitComChar,
	SL_GetStatus,
	SL_Reset,
	SL_GetModemStatus,
	SL_GetCommProperties,
	SL_PurgeComm,
	SL_SetDCB,
	SL_SetCommTimeouts,
	SL_Ioctl
};
// GetSerialObj : The purpose of this function is to allow multiple PDDs to be
// linked with a single MDD creating a multiport driver.  In such a driver, the
// MDD must be able to determine the correct vtbl and associated parameters for
// each PDD.  Immediately prior to calling HWInit, the MDD calls GetSerialObject
// to get the correct function pointers and parameters.
//
	PHWOBJ
GetSerialObject(
		DWORD DeviceArrayIndex
		)
{
	PHWOBJ pSerObj;

	// Unlike many other serial samples, we do not have a statically allocated
	// array of HWObjs.  Instead, we allocate a new HWObj for each instance
	// of the driver.  The MDD will always call GetSerialObj/HWInit/HWDeinit in
	// that order, so we can do the alloc here and do any subsequent free in
	// HWDeInit.
	// Allocate space for the HWOBJ.
	pSerObj=(PHWOBJ)LocalAlloc( LPTR ,sizeof(HWOBJ) );
	if ( !pSerObj )
		return (NULL);

	// Fill in the HWObj structure that we just allocated.

	pSerObj->BindFlags = THREAD_AT_INIT;    // Have MDD create thread when device is first opened.
	pSerObj->dwIntID = 0;                	// SysIntr is filled in at init time
	pSerObj->pFuncTbl = (HW_VTBL *) &IoVTbl; // Return pointer to appropriate functions
	// Now return this structure to the MDD.
	return (pSerObj);
}

