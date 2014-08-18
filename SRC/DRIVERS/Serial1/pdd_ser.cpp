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

    Serial PDD for SamSang 2450 UART Common Code.

Notes: 
--*/
#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <bsp.h>
#include <ddkreg.h>
#include <serhw.h>
#include <Serdbg.h>
#include <pdds3c2450_ser.h>
#include <s3c2450_base_regs.h>

#define    EPLL_CLK    0

/*
#define DEBUG
#define ZONE_THREAD    2
#define ZONE_INIT    1
*/

CReg2450Uart::CReg2450Uart(PULONG pRegAddr)
:   m_pReg(pRegAddr)
{
    m_fIsBackedUp = FALSE;
//    PROCESSOR_INFO procInfo;
//    DWORD dwBytesReturned;
//    if (!KernelIoControl(IOCTL_PROCESSOR_INFORMATION, NULL, 0, &procInfo, sizeof(PROCESSOR_INFO), &dwBytesReturned))
//    {
        m_s3c2450_pclk = S3C2450_PCLK;//DEFAULT_S3C2450_PCLK; 
        //RETAILMSG(TRUE, (TEXT("WARNING: CReg2450Uart::CReg2450Uart failed to obtain processor frequency - using default value (%d).\r\n"), m_s3c2450_pclk)); 
//    }
//    else
//    {
//        m_s3c2450_pclk = procInfo.dwClockSpeed;
//        RETAILMSG(TRUE, (TEXT("INFO: CReg2450Uart::CReg2450Uart using processor frequency reported by the OAL (%d).\r\n"), m_s3c2450_pclk)); 
//    }

}
BOOL   CReg2450Uart::Init() 
{

    if (m_pReg) { // Set Value to default.
        Write_ULCON(0);
        Write_UCON(0);
        Write_UFCON(0);
        Write_UMCON(0);
        return TRUE;
    }
    else
        return FALSE;
}

void CReg2450Uart::Backup()
{
    m_fIsBackedUp = TRUE;
    m_ULCONBackup = Read_ULCON();
    m_UCONBackup = Read_UCON();
    m_UFCONBackup = Read_UFCON();
    m_UMCOMBackup = Read_UMCON();
    m_UBRDIVBackup = Read_UBRDIV();    
        m_UDIVSLOTBackup = Read_UDIVSLOT();
}
void CReg2450Uart::Restore()
{
    if (m_fIsBackedUp) {
        Write_ULCON(m_ULCONBackup );
        Write_UCON( m_UCONBackup );
        Write_UFCON( m_UFCONBackup );
        Write_UMCON( m_UMCOMBackup );
        Write_UBRDIV( m_UBRDIVBackup);
                Write_UDIVSLOT( m_UDIVSLOTBackup );
        m_fIsBackedUp = FALSE;
    }
}
CReg2450Uart::Write_BaudRate(ULONG BaudRate)
{
        DOUBLE Div_val;
        UINT UDIVSLOTn = 0;
        UINT UBRDIV = 0;
    DEBUGMSG(ZONE_INIT, (TEXT("SetBaudRate -> %d\r\n"), BaudRate));

    if ( (Read_UCON() & CS_MASK) == CS_PCLK ) {
        Div_val = (m_s3c2450_pclk/16.0/BaudRate);
        UBRDIV = (int)Div_val - 1;
        Write_UBRDIV( UBRDIV );
        UDIVSLOTn = (int)( (Div_val - (int)Div_val) * 16);
        Write_UDIVSLOT( UDIVSLOT_TABLE[UDIVSLOTn] );
        RETAILMSG( true , (TEXT("CLK:%d, BaudRate:%d, UBRDIV:%d, UDIVSLOTn:%d\r\n"), m_s3c2450_pclk, BaudRate, UBRDIV, UDIVSLOTn));
        return TRUE;
    }
   else if( (Read_UCON() & CS_MASK) == (3 << 10) )
   {
        Div_val = (S3C2450_SCLK/16.0/BaudRate);
        UBRDIV = (int)Div_val - 1;
        Write_UBRDIV( UBRDIV );
        UDIVSLOTn = (int)( (Div_val - (int)Div_val) * 16);
        Write_UDIVSLOT( UDIVSLOT_TABLE[UDIVSLOTn] );
        RETAILMSG( TRUE , (TEXT("CLK:%d, BaudRate:%d, UBRDIV:%d, UDIVSLOTn:%d\r\n"), S3C2450_SCLK, BaudRate, UBRDIV, UDIVSLOTn));
        return TRUE;
   }
    else {
        // TODO: Support external UART clock.
        //OUTREG(pHWHead,UBRDIV,( (int)(S2450UCLK/16.0/BaudRate) -1 ));
        RETAILMSG(TRUE, (TEXT("ERROR: The s3c2450a serial driver doesn't support an external UART clock.\r\n")));
        ASSERT(FALSE);
        return(FALSE);
    }
}
#ifdef DEBUG
void CReg2450Uart::DumpRegister()
{
    NKDbgPrintfW(TEXT("DumpRegister (ULCON=%x, UCON=%x, UFCON=%x, UMCOM = %x, UBDIV =%x)\r\n"),
        Read_ULCON(),Read_UCON(),Read_UFCON(),Read_UMCON(),Read_UBRDIV());
    
}
#endif

CPdd2450Uart::CPdd2450Uart (LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj )
:   CSerialPDD(lpActivePath,pMdd, pHwObj)
,   m_ActiveReg(HKEY_LOCAL_MACHINE,lpActivePath)
,   CMiniThread (0, TRUE)   
{
    m_pReg2450Uart = NULL;
    m_pINTregs = NULL;
    m_dwIntShift = 0;
    m_dwSysIntr = MAXDWORD;
    m_hISTEvent = NULL;
    m_dwDevIndex = 0;
    m_pRegVirtualAddr = NULL;
    m_XmitFlushDone =  CreateEvent(0, FALSE, FALSE, NULL);
    m_XmitFifoEnable = FALSE;
    m_dwWaterMark = 8 ;
}
CPdd2450Uart::~CPdd2450Uart()
{
    InitModem(FALSE);
    if (m_hISTEvent) {
        m_bTerminated=TRUE;
        ThreadStart();
        SetEvent(m_hISTEvent);
        ThreadTerminated(1000);
        InterruptDisable( m_dwSysIntr );         
        CloseHandle(m_hISTEvent);
    };
    if (m_pReg2450Uart)
        delete m_pReg2450Uart;
    if (m_XmitFlushDone)
        CloseHandle(m_XmitFlushDone);
    if (m_pRegVirtualAddr != NULL) {
        MmUnmapIoSpace((PVOID)m_pRegVirtualAddr,0UL);
    }
    if (m_pINTregs!=NULL) {
        MmUnmapIoSpace((PVOID)m_pINTregs,0UL);
    }
        
}
BOOL CPdd2450Uart::Init()
{
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
        if (!GetRegValue(PC_REG_2450UART_INTBIT_VAL_NAME,(PBYTE)&m_dwIntShift,sizeof(DWORD))) {
            RETAILMSG(1,(TEXT("Registery does not have %s set. Drivers fail!!!\r\n"),PC_REG_2450UART_INTBIT_VAL_NAME));
            m_dwIntShift =0;
            return FALSE;
        }
        if (!GetRegValue(PC_REG_2450UART_IST_TIMEOUTS_VAL_NAME,(PBYTE)&m_dwISTTimeout, PC_REG_2450UART_IST_TIMEOUTS_VAL_LEN)) {
            m_dwISTTimeout = INFINITE;
        }
        if (!MapHardware() || !CreateHardwareAccess()) {
            return FALSE;
        }
        
        return TRUE;        
    }
    return FALSE;
}
BOOL CPdd2450Uart::MapHardware() 
{
    if (m_pRegVirtualAddr !=NULL)
        return TRUE;

    // Get IO Window From Registry
    DDKWINDOWINFO dwi;
    if ( GetWindowInfo( &dwi)!=ERROR_SUCCESS || 
            dwi.dwNumMemWindows < 1 || 
            dwi.memWindows[0].dwBase == 0 || 
            dwi.memWindows[0].dwLen < 0x30) //0x2c)
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
    ioPhysicalBase.LowPart = S3C2450_BASE_REG_PA_INTR ;
    ioPhysicalBase.HighPart = 0;
    inIoSpace = 0; 
    if (TranslateBusAddr(m_hParent,(INTERFACE_TYPE)dwi.dwInterfaceType,dwi.dwBusNumber, ioPhysicalBase,&inIoSpace,&ioPhysicalBase)) {
        m_pINTregs = (S3C2450_INTR_REG *) MmMapIoSpace(ioPhysicalBase,sizeof(S3C2450_INTR_REG),FALSE);
    }
    return (m_pRegVirtualAddr!=NULL && m_pINTregs!=NULL);
}
BOOL CPdd2450Uart::CreateHardwareAccess()
{
    if (m_pReg2450Uart)
        return TRUE;
    if (m_pRegVirtualAddr!=NULL) {
        m_pReg2450Uart = new CReg2450Uart((PULONG)m_pRegVirtualAddr);
        if (m_pReg2450Uart && !m_pReg2450Uart->Init()) { // FALSE.
            delete m_pReg2450Uart ;
            m_pReg2450Uart = NULL;
        }
            
    }
    return (m_pReg2450Uart!=NULL);
}
#define MAX_RETRY 0x1000
void CPdd2450Uart::PostInit()
{
    DWORD dwCount=0;
    m_HardwareLock.Lock();
    m_pReg2450Uart->Write_UCON(0); // Set to Default;
    DisableInterrupt(S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR);
    // Mask all interrupt.
    while ((GetInterruptStatus() & (S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR))!=0 && 
            dwCount <MAX_RETRY) { // Interrupt.
        InitReceive(TRUE);
        InitLine(TRUE);
        ClearInterrupt(S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR);
        dwCount++;
    }
    ASSERT((GetInterruptStatus() & (S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR))==0);
    // IST Start to Run.
    m_HardwareLock.Unlock();
    CSerialPDD::PostInit();
    CeSetPriority(m_dwPriority256);
#ifdef DEBUG
    if ( ZONE_INIT )
        m_pReg2450Uart->DumpRegister();
#endif
    ThreadStart();  // Start IST.
}
DWORD CPdd2450Uart::ThreadRun()
{
    while ( m_hISTEvent!=NULL && !IsTerminated() ) {
        if ( WaitForSingleObject( m_hISTEvent, m_dwISTTimeout) == WAIT_OBJECT_0) {
            m_HardwareLock.Lock();    
            while ( !IsTerminated() ) {
                DWORD dwData = ( GetInterruptStatus() & (S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR) );
                DWORD dwMask = ( GetIntrruptMask() & (S2450UART_INT_RXD | S2450UART_INT_TXD | S2450UART_INT_ERR));
                 DEBUGMSG(ZONE_THREAD,
                      (TEXT(" CPdd2450Uart::ThreadRun INT=%x, MASK =%x\r\n"),dwData,dwMask));
                dwMask &= dwData;
                if (dwMask) {
                    DEBUGMSG(ZONE_THREAD,
                      (TEXT(" CPdd2450Uart::ThreadRun Active INT=%x\r\n"),dwMask));
                    DWORD interrupts=INTR_MODEM; // Always check Modem when we have change. It may work at polling mode.
                    if ((dwMask & S2450UART_INT_RXD)!=0)
                        interrupts |= INTR_RX;
                    if ((dwMask & S2450UART_INT_TXD)!=0)
                        interrupts |= INTR_TX;
                    if ((dwMask & S2450UART_INT_ERR)!=0) 
                        interrupts |= INTR_LINE | INTR_RX;
                    NotifyPDDInterrupt( (INTERRUPT_TYPE)interrupts );
                    ClearInterrupt(dwData);
                }
                else 
                    break;
            }
            m_HardwareLock.Unlock();   
            InterruptDone(m_dwSysIntr);
        }
        else { // Polling Modem.
            NotifyPDDInterrupt(INTR_MODEM);
            DEBUGMSG(ZONE_THREAD,(TEXT(" CPdd2450Uart::ThreadRun timeout INT=%x,MASK=%d\r\n"), m_pINTregs->SUBSRCPND, m_pINTregs->INTSUBMSK) );
#ifdef DEBUG
            if ( ZONE_THREAD )
                m_pReg2450Uart->DumpRegister();
#endif
        }
    }
    return 1;
}
BOOL CPdd2450Uart::InitialEnableInterrupt(BOOL bEnable )
{
    m_HardwareLock.Lock();
    if (bEnable) 
        EnableInterrupt(S2450UART_INT_RXD | S2450UART_INT_ERR );
    else
        DisableInterrupt(S2450UART_INT_RXD | S2450UART_INT_ERR );
    m_HardwareLock.Unlock();
    return TRUE;
}

BOOL  CPdd2450Uart::InitXmit(BOOL bInit)
{
    if (bInit) { 
        m_HardwareLock.Lock();    
        DWORD dwBit = m_pReg2450Uart->Read_UCON();
#if EPLL_CLK
        // Set UART CLK.
        dwBit &= ~(3 << 10);
        dwBit |= (3 << 10);
#endif
        // Set TxINterrupt To Level.
        dwBit |= (1<<9);
        // Set Interrupt Tx Mode.
        dwBit &= ~(3<<2);
        dwBit |= (1<<2);
        m_pReg2450Uart->Write_UCON(dwBit );

        dwBit = m_pReg2450Uart->Read_UFCON();
        // Reset Xmit Fifo.
        dwBit |= (1<<2);
        dwBit &= ~(1<<0);
        m_pReg2450Uart->Write_UFCON( dwBit);
        // Set Trigger level to 16. 
        dwBit &= ~(3<<6);//empty
        dwBit |= (1<<6);//16
        m_pReg2450Uart->Write_UFCON(dwBit); 
        // Enable Xmit FIFO.
        dwBit &= ~(1<<2);
        dwBit |= (1<<0);
        m_pReg2450Uart->Write_UFCON(dwBit); // Xmit Fifo Reset Done..
        m_HardwareLock.Unlock();
    }
    else { // Make Sure data has been trasmit out.
        // We have to make sure the xmit is complete because MDD will shut donw the device after this return
        DWORD dwTicks = 0;
        DWORD dwUTRState;
        while (dwTicks < 1000 && 
                (((dwUTRState = m_pReg2450Uart->Read_UTRSTAT())>>1) & 3)!=3  ) { // Transmitter empty is not true
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE, (TEXT("CPdd2450Uart::InitXmit! Wait for UTRSTAT=%x clear.\r\n"), dwUTRState));
            Sleep(5);
            dwTicks +=5;
        }
    }
    return TRUE;
}
DWORD   CPdd2450Uart::GetWriteableSize()
{
    DWORD dwWriteSize = 0;
    DWORD dwUfState = m_pReg2450Uart->Read_UFSTAT() ;
    if ((dwUfState& (1<<14))==0) { // It is not full.
        dwUfState = ((dwUfState>>8) & 0x3f); // It is fifo count.
        if (dwUfState < SER2450_FIFO_DEPTH_TX-1)
            dwWriteSize = SER2450_FIFO_DEPTH_TX-1 - dwUfState;
    }
    return dwWriteSize;
}
void    CPdd2450Uart::XmitInterruptHandler(PUCHAR pTxBuffer, ULONG *pBuffLen)
{
    PREFAST_DEBUGCHK(pBuffLen!=NULL);
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
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE, (TEXT("CPdd2450Uart::XmitInterruptHandler! Flow Off, Data Discard.\r\n")));
            EnableXmitInterrupt(FALSE);
        }
        else  {
            DWORD dwWriteSize = GetWriteableSize();
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd2450Uart::XmitInterruptHandler! WriteableSize=%x to FIFO,dwDataAvaiable=%x\r\n"),
                    dwWriteSize,dwDataAvaiable));
            for (DWORD dwByteWrite=0; dwByteWrite<dwWriteSize && dwDataAvaiable!=0;dwByteWrite++) {
                m_pReg2450Uart->Write_UTXH(*pTxBuffer);
                pTxBuffer ++;
                dwDataAvaiable--;
            }
            DEBUGMSG(ZONE_THREAD|ZONE_WRITE,(TEXT("CPdd2450Uart::XmitInterruptHandler! Write %d byte to FIFO\r\n"),dwByteWrite));
            *pBuffLen = dwByteWrite;
            EnableXmitInterrupt(TRUE);        
        }
        ClearInterrupt(S2450UART_INT_TXD);

        if (m_pReg2450Uart->Read_ULCON() & (0x1<<6))
            while( (m_pReg2450Uart->Read_UFSTAT() >> 0x8 ) & 0x3f );
        Rx_Pause(FALSE);
    }
    m_HardwareLock.Unlock();
}

void    CPdd2450Uart::XmitComChar(UCHAR ComChar)
{
    // This function has to poll until the Data can be sent out.
    BOOL bDone = FALSE;
    do {
        m_HardwareLock.Lock(); 
        if ( GetWriteableSize()!=0 ) {  // If not full 
            m_pReg2450Uart->Write_UTXH(ComChar);
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
BOOL    CPdd2450Uart::EnableXmitInterrupt(BOOL fEnable)
{
    m_HardwareLock.Lock();
    if (fEnable)
        EnableInterrupt(S2450UART_INT_TXD);
    else
        DisableInterrupt(S2450UART_INT_TXD);
    m_HardwareLock.Unlock();
    return TRUE;
        
}
BOOL  CPdd2450Uart::CancelXmit()
{
    return InitXmit(TRUE);     
}
static PAIRS s_HighWaterPairs[] = {
    {0, 1 },
    {1, 8 },
    {2, 16 },
    {3, 32 }
};

BYTE  CPdd2450Uart::GetWaterMarkBit()
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
DWORD   CPdd2450Uart::GetWaterMark()
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
BOOL    CPdd2450Uart::InitReceive(BOOL bInit)
{
    m_HardwareLock.Lock();    
    if (bInit) {         
        BYTE uWarterMarkBit = GetWaterMarkBit();
        if (uWarterMarkBit> 3)
            uWarterMarkBit = 3;
        // Setup Receive FIFO.
        // Reset Receive Fifo.
        DWORD dwBit = m_pReg2450Uart->Read_UFCON();
        dwBit |= (1<<1);
        dwBit &= ~(1<<0);
        m_pReg2450Uart->Write_UFCON( dwBit);
        // Set Trigger level to WaterMark.
        dwBit &= ~(3<<4);
        dwBit |= (uWarterMarkBit<<4);
        m_pReg2450Uart->Write_UFCON(dwBit); 
        // Enable Receive FIFO.
        dwBit &= ~(1<<1);
        dwBit |= (1<<0);
        m_pReg2450Uart->Write_UFCON(dwBit); // Xmit Fifo Reset Done..
        m_pReg2450Uart->Read_UERSTAT(); // Clean Line Interrupt.
        dwBit = m_pReg2450Uart->Read_UCON();
#if EPLL_CLK
        // Set UART CLK.
        dwBit &= ~(3 << 10);
        dwBit |= (3 << 10);
#endif
        dwBit &= ~(3<<0);
        dwBit |= (1<<0)|(1<<7)|(1<<8); // Enable Rx Timeout and Level Interrupt Trigger.
        m_pReg2450Uart->Write_UCON(dwBit);
        EnableInterrupt(S2450UART_INT_RXD | S2450UART_INT_ERR );
    }
    else {
        DisableInterrupt(S2450UART_INT_RXD | S2450UART_INT_ERR );
    }
    m_HardwareLock.Unlock();
    return TRUE;
}
ULONG   CPdd2450Uart::ReceiveInterruptHandler(PUCHAR pRxBuffer,ULONG *pBufflen)
{
    DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("+CPdd2450Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x\r\n"),
        pRxBuffer,pBufflen!=NULL?*pBufflen:0));
    DWORD dwBytesDropped = 0;
    if (pRxBuffer && pBufflen ) {
        DWORD dwBytesStored = 0 ;
        DWORD dwRoomLeft = *pBufflen;
        m_bReceivedCanceled = FALSE;
        m_HardwareLock.Lock();
        
        while (dwRoomLeft && !m_bReceivedCanceled) {
            ULONG ulUFSTATE = m_pReg2450Uart->Read_UFSTAT();
            DWORD dwNumRxInFifo = (ulUFSTATE & (0x3f<<0));
            if ((ulUFSTATE & (1<<6))!=0) // Overflow. Use FIFO depth (16);
                dwNumRxInFifo = SER2450_FIFO_DEPTH_RX;
            DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("CPdd2450Uart::ReceiveInterruptHandler ulUFSTATE=%x,UTRSTAT=%x, dwNumRxInFifo=%X\r\n"),
                ulUFSTATE, m_pReg2450Uart->Read_UTRSTAT(), dwNumRxInFifo));
            if (dwNumRxInFifo) {
                ASSERT((m_pReg2450Uart->Read_UTRSTAT () & (1<<0))!=0);
                while (dwNumRxInFifo && dwRoomLeft) {
                    UCHAR uLineStatus = GetLineStatus();
                    UCHAR uData = m_pReg2450Uart->Read_URXH();
                    if (DataReplaced(&uData,(uLineStatus & UERSTATE_PARITY_ERROR)!=0)) {
                        *pRxBuffer++ = uData;
                        dwRoomLeft--;
                        dwBytesStored++;                    
                    }
                    dwNumRxInFifo --;
                }
            }
            else
                break;
        }
        if (m_bReceivedCanceled)
            dwBytesStored = 0;
        
        m_HardwareLock.Unlock();
        *pBufflen = dwBytesStored;
    }
    else {
        ASSERT(FALSE);
    }
    DEBUGMSG(ZONE_THREAD|ZONE_READ,(TEXT("-CPdd2450Uart::ReceiveInterruptHandler pRxBuffer=%x,*pBufflen=%x,dwBytesDropped=%x\r\n"),
        pRxBuffer,pBufflen!=NULL?*pBufflen:0,dwBytesDropped));
    return dwBytesDropped;
}
ULONG   CPdd2450Uart::CancelReceive()
{
    m_bReceivedCanceled = TRUE;
    m_HardwareLock.Lock();   
    InitReceive(TRUE);
    m_HardwareLock.Unlock();
    return 0;
}
BOOL    CPdd2450Uart::InitModem(BOOL bInit)
{
    m_HardwareLock.Lock();   
    m_pReg2450Uart->Write_UMCON((1<<0)); // Disable AFC and Set RTS as default.
    m_HardwareLock.Unlock();
    return TRUE;
}

ULONG   CPdd2450Uart::GetModemStatus()
{
    m_HardwareLock.Lock();    
    ULONG ulReturn =0 ;
    ULONG Events = 0;
    UINT8 ubModemStatus = (UINT8) m_pReg2450Uart->Read_UMSTAT();
    m_HardwareLock.Unlock();

    // Event Notification.
    if (ubModemStatus & (1<<2))
        Events |= EV_CTS;
    if (Events!=0)
        EventCallback(Events);

    // Report Modem Status;
    if ( ubModemStatus & (1<<0) )
        ulReturn |= MS_CTS_ON;
    return ulReturn;
}
void    CPdd2450Uart::SetRTS(BOOL bSet)
{
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_UMCON();
    if (bSet) {
        ulData |= (1<<0);
    }
    else
        ulData &= ~(1<<0);
    m_pReg2450Uart->Write_UMCON(ulData);
    m_HardwareLock.Unlock();

}
BOOL CPdd2450Uart::InitLine(BOOL bInit)
{
    m_HardwareLock.Lock();
    if  (bInit) {
        // Set 8Bit,1Stop,NoParity,Normal Mode.
        //m_pReg2450Uart->Write_ULCON( (0x3<<0) | (0<<1) | (0<<3) | (0<<6) );
        EnableInterrupt( S2450UART_INT_ERR );
    }
    else {
        DisableInterrupt(S2450UART_INT_ERR );
    }
    m_HardwareLock.Unlock();
    return TRUE;
}
BYTE CPdd2450Uart::GetLineStatus()
{
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_UERSTAT();
    m_HardwareLock.Unlock();  
    ULONG ulError = 0;
    if (ulData & (1<<0) ) {
        ulError |=  CE_OVERRUN;
    }
    if (ulData & (1<<1)) {
        ulError |= CE_RXPARITY;
    }
    if (ulData & (1<<2)) {
        ulError |=  CE_FRAME;
    }
    if (ulError)
        SetReceiveError(ulError);
    
    if (ulData & (1<<3)) {
         EventCallback(EV_BREAK);
    }
    return (UINT8)ulData;
        
}
void    CPdd2450Uart::SetBreak(BOOL bSet)
{
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_UCON();
    if (bSet)
        ulData |= (1<<4);
    else
        ulData &= ~(1<<4);
    m_pReg2450Uart->Write_UCON(ulData);
    m_HardwareLock.Unlock();      
}
BOOL    CPdd2450Uart::SetBaudRate(ULONG BaudRate,BOOL /*bIrModule*/)
{
    m_HardwareLock.Lock();
    BOOL bReturn = m_pReg2450Uart->Write_BaudRate(BaudRate);
    m_HardwareLock.Unlock();      
    return TRUE;
}
BOOL    CPdd2450Uart::SetByteSize(ULONG ByteSize)
{
    BOOL bRet = TRUE;
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_ULCON() & (~0x3);
    switch ( ByteSize ) {
    case 5: 
        break;
    case 6:
        ulData|= (1<<0);
        break;
    case 7:
        ulData |= (2<<0);
        break;
    case 8:
        ulData |= (3<<0);
        break;
    default:
        bRet = FALSE;
        break;
    }
    if (bRet) {
        m_pReg2450Uart->Write_ULCON(ulData);
    }
    m_HardwareLock.Unlock();
    return bRet;
}
BOOL    CPdd2450Uart::SetParity(ULONG Parity)
{
    BOOL bRet = TRUE;
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_ULCON() & (~(0x7<<3));
    switch ( Parity ) {
    case ODDPARITY:
        ulData |= (4<<3);
        break;
    case EVENPARITY:
        ulData |= (5<<3);
        break;
    case MARKPARITY:
        ulData |= (6<<3);
        break;
    case SPACEPARITY:
        ulData |= (7<<3);
        break;
    case NOPARITY:
        break;
    default:
        bRet = FALSE;
        break;
    }
    if (bRet) {
        m_pReg2450Uart->Write_ULCON(ulData);
    }
    m_HardwareLock.Unlock();
    return bRet;
}
BOOL    CPdd2450Uart::SetStopBits(ULONG StopBits)
{
    BOOL bRet = TRUE;
    m_HardwareLock.Lock();
    ULONG ulData = m_pReg2450Uart->Read_ULCON() & (~(0x1<<2));

    switch ( StopBits ) {
    case ONESTOPBIT :
        break;
    case TWOSTOPBITS :
        ulData |= (0x1<<2);
        break;
    default:
        bRet = FALSE;
        break;
    }
    if (bRet) {
        m_pReg2450Uart->Write_ULCON(ulData);
    }
    m_HardwareLock.Unlock();
    return bRet;
}
void    CPdd2450Uart::SetOutputMode(BOOL UseIR, BOOL Use9Pin)
{
    m_HardwareLock.Lock();
    CSerialPDD::SetOutputMode(UseIR, Use9Pin);
    ULONG ulData = m_pReg2450Uart->Read_ULCON() & (~(0x1<<6));
    ulData |= (UseIR?(0x1<<6):0);
    m_pReg2450Uart->Write_ULCON(ulData);
    m_HardwareLock.Unlock();
}

