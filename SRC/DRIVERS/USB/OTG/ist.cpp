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
#include <windows.h>
#include <types.h>
#include <nkintr.h>
#include "intr.h"
#include "ist.h"

CIST::CIST(LPCTSTR pszActiveKey, DWORD dwISTTimeout, DWORD dwTerminationTimeout)
:   CMiniThread(0,TRUE), m_Reg(pszActiveKey), m_ISTTimeout(dwISTTimeout),
    m_TerminationTimeout(dwTerminationTimeout)
{
	LPCWSTR lpFilename = L"giisr.dll";
	LPCWSTR lpszFunctionName = L"ISRHandler";
    m_fAllocatedSysIntr = FALSE;
    m_hIsrHandler = NULL;
    m_hISTEvent = NULL;
    m_fIntInitialized = FALSE;
    m_dwIrq = OAL_INTR_IRQ_OTG_ATC;
    m_dwSysIntr = SYSINTR_UNDEFINED;

	BOOL RetVal = KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &m_dwIrq, sizeof(m_dwIrq),
		&m_dwSysIntr, sizeof(m_dwSysIntr), NULL);
    if ((!RetVal) || (m_dwSysIntr == SYSINTR_UNDEFINED))
	{
		m_dwSysIntr = SYSINTR_UNDEFINED;
    }
    else
	{
		m_fAllocatedSysIntr = TRUE ;
		m_hIsrHandler = LoadIntChainHandler(lpFilename, lpszFunctionName, (BYTE) m_dwIrq);
	}
}
BOOL CIST::Init()
{
    return (m_dwSysIntr != SYSINTR_NOP);
}
BOOL  CIST::IntChainHandlerIoControl(DWORD dwIoContro, PVOID pInPtr, DWORD dwInSize, PVOID pOutPtr, DWORD dwOutSize,PDWORD pdwActualWrite)
{
    BOOL fReturn = (m_hIsrHandler?
        (::IntChainHandlerIoControl(m_hIsrHandler,dwIoContro,pInPtr,dwInSize,pOutPtr,dwOutSize,pdwActualWrite)) :
        TRUE);
    ASSERT(fReturn);
    return fReturn;
}

BOOL CIST::IntializeInterrupt(  LPVOID pvData,  DWORD cbData ) 
{
    if (m_dwSysIntr != SYSINTR_NOP && !m_hISTEvent && !m_fIntInitialized) {
        m_hISTEvent= CreateEvent(0,FALSE,FALSE,NULL);
        if (m_hISTEvent) {
            m_fIntInitialized = InterruptInitialize(m_dwSysIntr,m_hISTEvent,pvData, cbData );
        }
    }
    if (m_fIntInitialized) {
        ThreadStart();
    }
    return m_fIntInitialized;
}
CIST::~CIST()
{
    m_bTerminated=TRUE;
    ThreadStart();
    if (m_hISTEvent) {
        SetEvent(m_hISTEvent);
        BOOL fTerminated = ThreadTerminated(m_TerminationTimeout);
        ASSERT(fTerminated) ;
    };
    if (m_fIntInitialized) {
        InterruptDisable(m_dwSysIntr);
    }
    if (m_hISTEvent)
        CloseHandle(m_hISTEvent);
    
    if (m_hIsrHandler) {
        FreeIntChainHandler(m_hIsrHandler);
    }
    if (m_fAllocatedSysIntr && m_dwSysIntr != SYSINTR_NOP) {
        BOOL fRetVal = KernelIoControl( IOCTL_HAL_RELEASE_SYSINTR, &m_dwSysIntr, sizeof( m_dwSysIntr ), NULL, 0, NULL );
    }
}
DWORD   CIST::ThreadRun()
{
    if (!m_bTerminated && m_dwSysIntr!= SYSINTR_NOP && m_hISTEvent!=NULL) {
        BOOL fRet= TRUE;
        while (!m_bTerminated && fRet ) {
            switch (WaitForSingleObject(m_hISTEvent,m_ISTTimeout)){
              case WAIT_OBJECT_0:
                fRet = ISTProcess();
                ASSERT(fRet);
                InterruptDone(m_dwSysIntr);
                break;
              case WAIT_TIMEOUT:
                fRet = ISTTimeout();
                ASSERT(fRet);
                break;
              default:
                ASSERT(FALSE);
                fRet = FALSE;
                break;
            }
        }
    }
    return 0;
}

