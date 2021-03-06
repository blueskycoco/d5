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
//
/*++
  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
  ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  PARTICULAR PURPOSE.

  Module Name:  

Abstract:

Serial PDD for SamSang 3250 UART Common Code.

Notes: 
--*/
#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <bsp.h>
#include <ddkreg.h>
#include <serhw.h>
//#include <Serdbg.h>
#include "pdd_ser.h"
#define MAX_RETRY 0x1000
static PAIRS s_HighWaterPairs[] = {
	{0, 1 },
	{1, 8 },
	{2, 16 },
	{3, 32 }
};

#define FIFO_READ_LIMIT 128
/*
#define DEBUG
#define ZONE_THREAD    2
#define ZONE_INIT    1
*/
//#ifdef DEBUG
void CReg3250Uart::DumpRegister()
{
	NKDbgPrintfW(TEXT("DumpRegister (CTL=%x, RCTL=%x, LEVEL=%x, IIR = %x, DATA =%x)\r\n"),
			Read_CTL(),Read_RCTL(),Read_LEVEL(),Read_IIR(),Read_DATA());

}
//#endif

CReg3250Uart::CReg3250Uart(PULONG pRegAddr):m_pReg(pRegAddr)
{
	RETAILMSG(0,(TEXT("CReg3250Uart %x\r\n"),pRegAddr));
	m_fIsBackedUp = FALSE;
}
BOOL   CReg3250Uart::Init() 
{
	RETAILMSG(0,(TEXT("CReg3250Uart::Init\r\n")));
	if (m_pReg) { // Set Value to default.
		Write_IIR(0);
		Write_CTL(0);
		Write_RCTL(0);
		return TRUE;
	}
	else
		return FALSE;
}

void CReg3250Uart::Backup()
{
	RETAILMSG(0,(TEXT("Backup\r\n")));
	m_fIsBackedUp = TRUE;
	m_IirBackup = Read_IIR();
	m_CtlBackup = Read_CTL();
	m_RctlBackup = Read_RCTL();
}
void CReg3250Uart::Restore()
{
	RETAILMSG(0,(TEXT("Restore\r\n")));
	if (m_fIsBackedUp) {
		Write_CTL(m_CtlBackup );
		Write_IIR( m_IirBackup );
		Write_RCTL( m_RctlBackup );
		m_fIsBackedUp = FALSE;
	}
}
BOOL CReg3250Uart::Write_BaudRate(ULONG BaudRate)
{
	UINT32 basepclk;
	UINT32 div, goodrate, hsu_rate, l_hsu_rate, comprate;
	UINT32 rate_diff;
	DWORD bytesret;
	RETAILMSG(0, (TEXT("Write_BaudRate -> %d\r\n"), BaudRate));
	if (KernelIoControl(IOCTL_LPC32XX_GETPCLK, NULL, 0, &basepclk,
				sizeof (basepclk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock
		RETAILMSG(0,(TEXT("ERROR: Write_BaudRate getting uart1 pclk.\r\n")));
		return FALSE;
	}
	/* Find the closest divider to get the desired clock rate */
	div = basepclk / BaudRate;
	goodrate = hsu_rate = (div / 14) - 1;
	if (hsu_rate != 0)
		hsu_rate--;

	/* Tweak divider */
	l_hsu_rate = hsu_rate + 3;
	rate_diff = 0xFFFFFFFF;

	while (hsu_rate < l_hsu_rate) {
		comprate = basepclk / ((hsu_rate + 1) * 14);
		if ((UINT32)abs(comprate - BaudRate) < rate_diff) {
			goodrate = hsu_rate;
			rate_diff = (UINT32)abs(comprate - BaudRate);
		}

		hsu_rate++;
	}
	if (hsu_rate > 0xFF)
		hsu_rate = 0xFF;
	RETAILMSG(0, (TEXT("Write_BaudRate ->rate %d\r\n"), goodrate));
	Write_RCTL(goodrate);
	return TRUE;
}

CPdd3250Uart::CPdd3250Uart (LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj ):CSerialPDD(lpActivePath,pMdd, pHwObj),m_ActiveReg(HKEY_LOCAL_MACHINE,lpActivePath),CMiniThread (0, TRUE)
{
	RETAILMSG(0,(TEXT("CPdd3250Uart\r\n")));
	m_pReg3250Uart = NULL;
	m_dwSysIntr = MAXDWORD;
	m_hISTEvent = NULL;
	m_dwDevIndex = 0;
	m_pRegVirtualAddr = NULL;
	m_XmitFlushDone =  CreateEvent(0, FALSE, FALSE, NULL);
	m_XmitFifoEnable = FALSE;
	m_dwWaterMark = 8 ;
}
CPdd3250Uart::~CPdd3250Uart()
{
	RETAILMSG(0,(TEXT("~CPdd3250Uart\r\n")));
	InitModem(FALSE);
	if (m_hISTEvent) {
		m_bTerminated=TRUE;
		ThreadStart();
		SetEvent(m_hISTEvent);
		ThreadTerminated(1000);
		InterruptDisable( m_dwSysIntr );         
		CloseHandle(m_hISTEvent);
	};
	if (m_pReg3250Uart)
		delete m_pReg3250Uart;
	if (m_XmitFlushDone)
		CloseHandle(m_XmitFlushDone);
	if (m_pRegVirtualAddr != NULL) {
		MmUnmapIoSpace((PVOID)m_pRegVirtualAddr,0UL);
	}

}
BOOL CPdd3250Uart::Init()
{
	RETAILMSG(0,(TEXT("Init\r\n")));
	if ( CSerialPDD::Init() && IsKeyOpened() && m_XmitFlushDone!=NULL) { 
		// IST Setup .
		DDKISRINFO ddi;
		if (GetIsrInfo(&ddi)!=ERROR_SUCCESS) {
			return FALSE;
		}
		m_dwSysIntr = ddi.dwSysintr;
		if (m_dwSysIntr !=  MAXDWORD && m_dwSysIntr!=0 ) 
			m_hISTEvent= CreateEvent(0,FALSE,FALSE,NULL);

		if (m_hISTEvent!=NULL)
			InterruptInitialize(m_dwSysIntr,m_hISTEvent,0,0);
		else
			return FALSE;

		// Get Device Index.
		if (!GetRegValue(PC_REG_DEVINDEX_VAL_NAME, (PBYTE)&m_dwDevIndex, PC_REG_DEVINDEX_VAL_LEN)) {
			m_dwDevIndex = 0;
		}
		if (!GetRegValue(PC_REG_SERIALWATERMARK_VAL_NAME,(PBYTE)&m_dwWaterMark,sizeof(DWORD))) {
			m_dwWaterMark = 8;
		}
		if (!GetRegValue(PC_REG_3250UART_INTBIT_VAL_NAME,(PBYTE)&m_dwIntShift,sizeof(DWORD))) {
		//	RETAILMSG(0,(TEXT("Registery does not have %s set. Drivers fail!!!\r\n"),PC_REG_3250UART_INTBIT_VAL_NAME));
			m_dwIntShift =0;
			//return FALSE;
		}
		//if (!GetRegValue(PC_REG_3250UART_IST_TIMEOUTS_VAL_NAME,(PBYTE)&m_dwISTTimeout, PC_REG_3250UART_IST_TIMEOUTS_VAL_LEN)) {
			m_dwISTTimeout = INFINITE;
		//}
		if (!MapHardware() || !CreateHardwareAccess()) {
			return FALSE;
		}

		return TRUE;        
	}
	return FALSE;
}
BOOL CPdd3250Uart::MapHardware() 
{
	RETAILMSG(0,(TEXT("MapHardware\r\n")));
	if (m_pRegVirtualAddr !=NULL)
		return TRUE;

	// Get IO Window From Registry
	DDKWINDOWINFO dwi;
	if ( GetWindowInfo( &dwi)!=ERROR_SUCCESS || 
			dwi.dwNumMemWindows < 1 || 
			dwi.memWindows[0].dwBase == 0 || 
			dwi.memWindows[0].dwLen < 0x10) //0x2c)
				return FALSE;
	DWORD dwInterfaceType;
	if (m_ActiveReg.IsKeyOpened() && 
			m_ActiveReg.GetRegValue( DEVLOAD_INTERFACETYPE_VALNAME, (PBYTE)&dwInterfaceType,sizeof(DWORD))) {
		dwi.dwInterfaceType = dwInterfaceType;
	}

	// Translate to System Address.
	PHYSICAL_ADDRESS    ioPhysicalBase = { dwi.memWindows[0].dwBase, 0};
	ULONG               inIoSpace = 0;
	if (TranslateBusAddr(m_hParent,(INTERFACE_TYPE)dwi.dwInterfaceType,dwi.dwBusNumber, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
		// Map it if it is Memeory Mapped IO.
		m_pRegVirtualAddr = MmMapIoSpace(ioPhysicalBase, dwi.memWindows[0].dwLen,FALSE);
	}
	return (m_pRegVirtualAddr!=NULL);
}
BOOL CPdd3250Uart::CreateHardwareAccess()
{
	RETAILMSG(0,(TEXT("CreateHardwareAccess\r\n")));
	if (m_pReg3250Uart)
		return TRUE;
	if (m_pRegVirtualAddr!=NULL) {
		m_pReg3250Uart = new CReg3250Uart((PULONG)m_pRegVirtualAddr);
		if (m_pReg3250Uart && !m_pReg3250Uart->Init()) { // FALSE.
			delete m_pReg3250Uart ;
			m_pReg3250Uart = NULL;
		}

	}
	return (m_pReg3250Uart!=NULL);
}

void CPdd3250Uart::__serial_uart_flush()
{
	UINT32 tmp;
	int cnt = 0;

	while ((m_pReg3250Uart->Read_LEVEL() > 0) &&
			(cnt++ < FIFO_READ_LIMIT))
		tmp = m_pReg3250Uart->Read_DATA() ;
}
void CPdd3250Uart::PostInit()
{
	RETAILMSG(0,(TEXT("PostInit\r\n")));
	DWORD dwCount=0;
	m_HardwareLock.Lock();
	__serial_uart_flush();
	DisableInterrupt(LPC32XX_HSU_RX_INT_EN | LPC32XX_HSU_TX_INT_EN | LPC32XX_HSU_ERR_INT_EN);
	// Mask all interrupt.
	while ((GetInterruptStatus() & (LPC32XX_HSU_TX_INT | LPC32XX_HSU_FE_INT|LPC32XX_HSU_BRK_INT|LPC32XX_HSU_RX_OE_INT))!=0 && 
			dwCount <MAX_RETRY) { // Interrupt.
		InitReceive(TRUE);
		InitLine(TRUE);
		ClearInterrupt(LPC32XX_HSU_TX_INT | LPC32XX_HSU_FE_INT|LPC32XX_HSU_BRK_INT|LPC32XX_HSU_RX_OE_INT);
		dwCount++;
	}
	ASSERT((GetInterruptStatus() & (LPC32XX_HSU_TX_INT | LPC32XX_HSU_FE_INT|LPC32XX_HSU_BRK_INT|LPC32XX_HSU_RX_OE_INT))==0);	
	m_pReg3250Uart->Write_CTL(LPC32XX_HSU_TX_TL8B | LPC32XX_HSU_RX_TL32B |LPC32XX_HSU_OFFSET(20) | LPC32XX_HSU_TMO_INACT_4B); // Set to Default;    
	// IST Start to Run.
	m_HardwareLock.Unlock();
	CSerialPDD::PostInit();
	CeSetPriority(m_dwPriority256);
//#ifdef DEBUG
//	if ( ZONE_INIT )
//		m_pReg3250Uart->DumpRegister();
//#endif
	ThreadStart();  // Start IST.
}
DWORD CPdd3250Uart::ThreadRun()
{
	RETAILMSG(0,(TEXT("ThreadRun\r\n")));
	while ( m_hISTEvent!=NULL && !IsTerminated() ) {
		if ( WaitForSingleObject( m_hISTEvent, m_dwISTTimeout) == WAIT_OBJECT_0) {
			m_HardwareLock.Lock();    
			while ( !IsTerminated() ) {
				DWORD dwInt=GetInterruptStatus();
				DWORD dwData = ( dwInt & (LPC32XX_HSU_BRK_INT | LPC32XX_HSU_FE_INT | LPC32XX_HSU_RX_OE_INT|LPC32XX_HSU_RX_TIMEOUT_INT | LPC32XX_HSU_RX_TRIG_INT|LPC32XX_HSU_TX_INT) );
				//DWORD dwMask = ( GetIntrruptMask() & (S3250UART_INT_RXD | S3250UART_INT_TXD | S3250UART_INT_ERR));
				RETAILMSG(0,
						(TEXT(" CPdd3250Uart::ThreadRun INT=%x , ==>%x\r\n"),dwInt,dwData));
				//dwMask &= dwData;
				if (dwData) {
					DEBUGMSG(ZONE_THREAD,
							(TEXT(" CPdd3250Uart::ThreadRun Active INT=%x\r\n"),dwData));
					DWORD interrupts=0;//INTR_MODEM; // Always check Modem when we have change. It may work at polling mode.
					if ((dwData & (LPC32XX_HSU_RX_TIMEOUT_INT | LPC32XX_HSU_RX_TRIG_INT))!=0)
						interrupts |= INTR_RX;
					if ((dwData & LPC32XX_HSU_TX_INT)!=0)
						interrupts |= INTR_TX;
					if ((dwData & (LPC32XX_HSU_BRK_INT | LPC32XX_HSU_FE_INT | LPC32XX_HSU_RX_OE_INT))!=0) 
						interrupts |= INTR_LINE | INTR_RX;
					NotifyPDDInterrupt( (INTERRUPT_TYPE)interrupts );
					if((dwData & LPC32XX_HSU_TX_INT)!=0)
						ClearInterrupt(LPC32XX_HSU_TX_INT);
				}
				else 
					break;
			}
			m_HardwareLock.Unlock();   
			InterruptDone(m_dwSysIntr);
		}
		else { // Polling Modem.
			NotifyPDDInterrupt(INTR_MODEM);
			DEBUGMSG(ZONE_THREAD,(TEXT(" CPdd3250Uart::ThreadRun timeout INT=%x,MASK=%d\r\n"), m_pINTregs->SUBSRCPND, m_pINTregs->INTSUBMSK) );
#ifdef DEBUG
			if ( ZONE_THREAD )
				m_pReg3250Uart->DumpRegister();
#endif
		}
	}
	return 1;
}
BOOL CPdd3250Uart::InitialEnableInterrupt(BOOL bEnable )
{
	RETAILMSG(0,(TEXT("InitialEnableInterrupt %d\r\n"),bEnable));
	m_HardwareLock.Lock();
	if (bEnable) 
		EnableInterrupt(LPC32XX_HSU_RX_INT_EN | LPC32XX_HSU_ERR_INT_EN);
	else
		DisableInterrupt(LPC32XX_HSU_RX_INT_EN | LPC32XX_HSU_ERR_INT_EN);
	m_HardwareLock.Unlock();
	return TRUE;
}
void CPdd3250Uart::wait_for_xmit_empty()
{
	unsigned int timeout = 10000;

	do {
		if (LPC32XX_HSU_TX_LEV(m_pReg3250Uart->Read_LEVEL()) == 0)
			break;
		if (--timeout == 0)
			break;
		Sleep(1);
	} while (1);
}

void CPdd3250Uart::wait_for_xmit_ready()
{
	unsigned int timeout = 10000;

	while (1) {
		if (LPC32XX_HSU_TX_LEV(m_pReg3250Uart->Read_LEVEL()) < 32)
			break;
		if (--timeout == 0)
			break;
		Sleep(1);
	}
}
BOOL  CPdd3250Uart::InitXmit(BOOL bInit)
{
	RETAILMSG(0,(TEXT("InitXmit %d\r\n"),bInit));
	if (bInit) { 
		m_HardwareLock.Lock();    
		EnableInterrupt(LPC32XX_HSU_TX_INT_EN);
		m_HardwareLock.Unlock();
	}
	else { // Make Sure data has been trasmit out.
		// We have to make sure the xmit is complete because MDD will shut donw the device after this return
		wait_for_xmit_empty();
		DisableInterrupt(LPC32XX_HSU_TX_INT_EN);
	}
	return TRUE;
}
DWORD   CPdd3250Uart::GetWriteableSize()
{
	DWORD dwWriteSize = 0;
	DWORD dwUfState = LPC32XX_HSU_TX_LEV(m_pReg3250Uart->Read_LEVEL());

	dwWriteSize=64-dwUfState;
	return dwWriteSize;
}
void    CPdd3250Uart::XmitInterruptHandler(PUCHAR pTxBuffer, ULONG *pBuffLen)
{
	PREFAST_DEBUGCHK(pBuffLen!=NULL);
	RETAILMSG(0,(TEXT("XmitInterruptHandler\r\n")));
	m_HardwareLock.Lock();    
	if (*pBuffLen == 0) { 
		EnableXmitInterrupt(FALSE);
	}
	else {
		DEBUGCHK(pTxBuffer);
		PulseEvent(m_XmitFlushDone);
		DWORD dwDataAvaiable = *pBuffLen;
		*pBuffLen = 0;

		Rx_Pause(TRUE);
		if ((m_DCB.fOutxCtsFlow && IsCTSOff()) ||(m_DCB.fOutxDsrFlow && IsDSROff())) { // We are in flow off
			RETAILMSG(0, (TEXT("CPdd3250Uart::XmitInterruptHandler! Flow Off, Data Discard.\r\n")));
			EnableXmitInterrupt(FALSE);
		}
		else  {
			DWORD dwWriteSize = GetWriteableSize();
			DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd3250Uart::XmitInterruptHandler! WriteableSize=%x to FIFO,dwDataAvaiable=%x\r\n"),
						dwWriteSize,dwDataAvaiable));
			for (DWORD dwByteWrite=0; dwByteWrite<dwWriteSize && dwDataAvaiable!=0;dwByteWrite++) {
				m_pReg3250Uart->Write_DATA(*pTxBuffer);
				RETAILMSG(0,(TEXT("1==>%c\r\n"),*pTxBuffer));
				pTxBuffer ++;
				dwDataAvaiable--;
			}
			DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd3250Uart::XmitInterruptHandler! Write %d byte to FIFO\r\n"),dwByteWrite));
			*pBuffLen = dwByteWrite;
			EnableXmitInterrupt(TRUE);        
		}
		//ClearInterrupt(S3250UART_INT_TXD);

		//if (m_pReg3250Uart->Read_ULCON() & (0x1<<6))
		//while( (m_pReg3250Uart->Read_UFSTAT() >> 0x8 ) & 0x3f );
		//wait_for_xmit_empty();
		Rx_Pause(FALSE);
	}
	m_HardwareLock.Unlock();
}

void    CPdd3250Uart::XmitComChar(UCHAR ComChar)
{
	// This function has to poll until the Data can be sent out.
	BOOL bDone = FALSE;
	RETAILMSG(0,(TEXT("XmitComChar %c\r\n"),ComChar));
	do {
		m_HardwareLock.Lock(); 
		if ( GetWriteableSize()!=0 ) {  // If not full 
			m_pReg3250Uart->Write_DATA(ComChar);
			RETAILMSG(0,(TEXT("2==>%c\r\n"),ComChar));
			bDone = TRUE;
		}
		else {
			EnableXmitInterrupt(TRUE);
		}
		m_HardwareLock.Unlock();
		if (!bDone)
			WaitForSingleObject(m_XmitFlushDone, (ULONG)1000); 
	}
	while (!bDone);
}
BOOL    CPdd3250Uart::EnableXmitInterrupt(BOOL fEnable)
{
	RETAILMSG(0,(TEXT("EnableXmitInterrupt %d\r\n"),fEnable));
	m_HardwareLock.Lock();
	if (fEnable)
		EnableInterrupt(LPC32XX_HSU_TX_INT_EN);
	else
		DisableInterrupt(LPC32XX_HSU_TX_INT_EN);
	m_HardwareLock.Unlock();
	return TRUE;

}
BOOL  CPdd3250Uart::CancelXmit()
{
	RETAILMSG(0,(TEXT("CancelXmit\r\n")));
	return InitXmit(TRUE);     
}

BYTE  CPdd3250Uart::GetWaterMarkBit()
{
	BYTE bReturnKey = (BYTE)s_HighWaterPairs[0].Key;
	for (DWORD dwIndex = dim(s_HighWaterPairs)-1 ; dwIndex != 0; dwIndex --) {
		if (m_dwWaterMark >= s_HighWaterPairs[dwIndex].AssociatedValue) {
			bReturnKey = (BYTE)s_HighWaterPairs[dwIndex].Key;
			break;
		}
	}
	return bReturnKey;
}
DWORD   CPdd3250Uart::GetWaterMark()
{
	BYTE bReturnValue = (BYTE)s_HighWaterPairs[0].AssociatedValue;
	for (DWORD dwIndex = dim(s_HighWaterPairs)-1 ; dwIndex != 0; dwIndex --) {
		if (m_dwWaterMark >= s_HighWaterPairs[dwIndex].AssociatedValue) {
			bReturnValue = (BYTE)s_HighWaterPairs[dwIndex].AssociatedValue;
			break;
		}
	}
	return bReturnValue;
}

// Receive
BOOL    CPdd3250Uart::InitReceive(BOOL bInit)
{
	RETAILMSG(0,(TEXT("InitReceive\r\n")));
	m_HardwareLock.Lock();    
	if (bInit) {         
		EnableInterrupt(LPC32XX_HSU_RX_INT_EN | LPC32XX_HSU_ERR_INT_EN);
	}
	else {
		DisableInterrupt(LPC32XX_HSU_RX_INT_EN | LPC32XX_HSU_ERR_INT_EN);
	}
	m_HardwareLock.Unlock();
	return TRUE;
}
ULONG   CPdd3250Uart::ReceiveInterruptHandler(PUCHAR pRxBuffer,ULONG *pBufflen)
{
	DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("+CPdd3250Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x\r\n"),
				pRxBuffer,pBufflen!=NULL?*pBufflen:0));
	RETAILMSG(0,(TEXT("+CPdd3250Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x\r\n"),
				pRxBuffer,pBufflen!=NULL?*pBufflen:0));
	DWORD dwBytesDropped = 0;
	unsigned int tmp;
	if (pRxBuffer && pBufflen ) {
		DWORD dwBytesStored = 0 ;
		DWORD dwRoomLeft = *pBufflen;
		m_bReceivedCanceled = FALSE;
		m_HardwareLock.Lock();
		tmp = m_pReg3250Uart->Read_DATA();
		RETAILMSG(0,(TEXT("read data %x\r\n"),tmp));
		while (dwRoomLeft && !m_bReceivedCanceled && !(tmp & LPC32XX_HSU_RX_EMPTY)) {
			
			/*ULONG ulUFSTATE = m_pReg3250Uart->Read_UFSTAT();
			  DWORD dwNumRxInFifo = (ulUFSTATE & (0x3f<<0));
			  if ((ulUFSTATE & (1<<6))!=0) // Overflow. Use FIFO depth (16);
			  dwNumRxInFifo = SER3250_FIFO_DEPTH_RX;
			  DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("CPdd3250Uart::ReceiveInterruptHandler ulUFSTATE=%x,UTRSTAT=%x, dwNumRxInFifo=%X\r\n"),
			  ulUFSTATE, m_pReg3250Uart->Read_UTRSTAT(), dwNumRxInFifo));
			  if (dwNumRxInFifo) {
			  ASSERT((m_pReg3250Uart->Read_UTRSTAT () & (1<<0))!=0);
			  while (dwNumRxInFifo && dwRoomLeft) {
			  UCHAR uLineStatus = GetLineStatus();
			  UCHAR uData = m_pReg3250Uart->Read_URXH();
			  if (DataReplaced(&uData,(uLineStatus & UERSTATE_PARITY_ERROR)!=0)) {
			 *pRxBuffer++ = uData;
			 dwRoomLeft--;
			 dwBytesStored++;                    
			 }
			 dwNumRxInFifo --;
			 }
			 }
			 else
			 break;*/


			/* Read data from FIFO and push into terminal */

			//		while (!(tmp & LPC32XX_HSU_RX_EMPTY)) {
			//flag = TTY_NORMAL;
			//port->icount.rx++;

			if (tmp & LPC32XX_HSU_ERROR_DATA) {
				/* Framing error */
				//__raw_writel(LPC32XX_HSU_FE_INT,
				//LPC32XX_HSUART_IIR(port->membase));
				m_pReg3250Uart->Write_IIR(LPC32XX_HSU_FE_INT/*|LPC32XX_HSU_RX_TRIG_INT*/);
				//port->icount.frame++;
				//flag = TTY_FRAME;
				//tty_insert_flip_char(port->state->port.tty, 0,
				//TTY_FRAME);
				//tty_schedule_flip(port->state->port.tty);
			}

			//tty_insert_flip_char(port->state->port.tty, (tmp & 0xFF),flag);
			*pRxBuffer++ = tmp & 0xFF;
			dwRoomLeft--;
			dwBytesStored++;       

			tmp = m_pReg3250Uart->Read_DATA();
			RETAILMSG(0,(TEXT("read data %x\r\n"),tmp));
			//	}
		}
		if (m_bReceivedCanceled)
			dwBytesStored = 0;

		m_HardwareLock.Unlock();
		*pBufflen = dwBytesStored;
	}
	else {
		ASSERT(FALSE);
	}
	DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("-CPdd3250Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x,dwBytesDropped=%x\r\n"),
				pRxBuffer,pBufflen!=NULL?*pBufflen:0,dwBytesDropped));
	RETAILMSG(0,(TEXT("-CPdd3250Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x,dwBytesDropped=%x\r\n"),
				pRxBuffer,pBufflen!=NULL?*pBufflen:0,dwBytesDropped));
	return dwBytesDropped;
}
ULONG   CPdd3250Uart::CancelReceive()
{
	RETAILMSG(0,(TEXT("CancelReceive\r\n")));
	m_bReceivedCanceled = TRUE;
	m_HardwareLock.Lock();   
	InitReceive(TRUE);
	m_HardwareLock.Unlock();
	return 0;
}
BOOL    CPdd3250Uart::InitModem(BOOL bInit)
{
	RETAILMSG(0,(TEXT("InitModem %d\r\n"),bInit));
	return TRUE;
}

ULONG   CPdd3250Uart::GetModemStatus()
{
	ULONG ulReturn =0 ;
	RETAILMSG(0,(TEXT("GetModemStatus\r\n")));
	return ulReturn;
}
void    CPdd3250Uart::SetRTS(BOOL bSet)
{
	RETAILMSG(0,(TEXT("SetRTS %d\r\n"),bSet));
}
BOOL CPdd3250Uart::InitLine(BOOL bInit)
{
	RETAILMSG(0,(TEXT("InitLine %d\r\n"),bInit));
	m_HardwareLock.Lock();
	if  (bInit) {
		EnableInterrupt(LPC32XX_HSU_ERR_INT_EN);
	}
	else {
		DisableInterrupt(LPC32XX_HSU_ERR_INT_EN);
	}
	m_HardwareLock.Unlock();
	return TRUE;
}
BYTE CPdd3250Uart::GetLineStatus()
{
	
	m_HardwareLock.Lock();
	ULONG ulData = m_pReg3250Uart->Read_IIR();
	m_HardwareLock.Unlock();  
	ULONG ulError = 0;
	if (ulData & LPC32XX_HSU_RX_OE_INT ) {
		ulError |=  CE_OVERRUN;
		ClearInterrupt(LPC32XX_HSU_RX_OE_INT);
	}
	if (ulData & LPC32XX_HSU_FE_INT) {
		ulError |=  CE_FRAME;
		ClearInterrupt(LPC32XX_HSU_FE_INT);
	}
	if (ulError)
		SetReceiveError(ulError);

	if (ulData & LPC32XX_HSU_BRK_INT) {
		EventCallback(EV_BREAK);
		ClearInterrupt(LPC32XX_HSU_BRK_INT);
	}
	
	RETAILMSG(0,(TEXT("GetLineStatus %d\r\n"),ulData));
	return (UINT8)ulData;

}
void    CPdd3250Uart::SetBreak(BOOL bSet)
{
	RETAILMSG(0,(TEXT("SetBreak %d\r\n"),bSet));
	m_HardwareLock.Lock();   
	if (bSet)
		m_pReg3250Uart->Write_CTL(m_pReg3250Uart->Read_CTL()|LPC32XX_HSU_BREAK);
	else
		m_pReg3250Uart->Write_CTL(m_pReg3250Uart->Read_CTL()&(~LPC32XX_HSU_BREAK));
	m_HardwareLock.Unlock();      
}
BOOL    CPdd3250Uart::SetBaudRate(ULONG BaudRate,BOOL /*bIrModule*/)
{
	RETAILMSG(0,(TEXT("SetBaudRate\r\n")));
	m_HardwareLock.Lock();
	BOOL bReturn = m_pReg3250Uart->Write_BaudRate(BaudRate);
	m_HardwareLock.Unlock();      
	return TRUE;
}
BOOL    CPdd3250Uart::SetByteSize(ULONG ByteSize)
{
	BOOL bRet = TRUE;
	RETAILMSG(0,(TEXT("SetByteSize %d\r\n"),ByteSize));
	return bRet;
}
BOOL    CPdd3250Uart::SetParity(ULONG Parity)
{
	BOOL bRet = TRUE;
	RETAILMSG(0,(TEXT("SetParity %d\r\n"),Parity));
	return bRet;
}
BOOL    CPdd3250Uart::SetStopBits(ULONG StopBits)
{
	BOOL bRet = TRUE;
	RETAILMSG(0,(TEXT("SetStopBits %d\r\n"),StopBits));
	return bRet;
}
void    CPdd3250Uart::SetOutputMode(BOOL UseIR, BOOL Use9Pin)
{
	RETAILMSG(0,(TEXT("SetOutputMode\r\n")));
}

