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

Abstract:

    Serial PDD for nxp lpc3250 Development Board.

Notes: 
--*/
#include <windows.h>
#include <types.h>
#include <ceddk.h>

#include <ddkreg.h>
#include <serhw.h>
#include <Serdbg.h>
#include "pdd_ser.h"
#include "intr.h"
class CPdd3250Serial1 : public CPdd3250Uart {
public:
    CPdd3250Serial1(LPTSTR lpActivePath, PVOID pMdd, PHWOBJ pHwObj)
        : CPdd3250Uart(lpActivePath, pMdd, pHwObj)
        {
    }
    ~CPdd3250Serial1() {
    }
    virtual BOOL Init() {
            DDKISRINFO ddi;
		ULONG irq=OAL_INTR_IRQ_UART1;		
		if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irq, sizeof(irq),
			  &(ddi.dwSysintr), sizeof(ddi.dwSysintr),
			  NULL))
		{
		    RETAILMSG(1, (TEXT("ERROR: UART1: Failed to request the UART1 sysintr.\r\n")));				
		    ddi.dwSysintr = SYSINTR_UNDEFINED;
		    return FALSE;
		}else
			RETAILMSG(1, (TEXT("OK: UART1: request the UART1 sysintr. %x\r\n"),ddi.dwSysintr));	
            RegSetValueEx(DEVLOAD_SYSINTR_VALNAME,REG_DWORD,(PBYTE)&ddi.dwSysintr, sizeof(UINT32));
            
            return CPdd3250Uart::Init();
    };
    virtual void    SetDefaultConfiguration() {
        CPdd3250Uart::SetDefaultConfiguration();
    }
};

CSerialPDD * CreateSerialObject(LPTSTR lpActivePath, PVOID pMdd,PHWOBJ pHwObj, DWORD DeviceArrayIndex)
{
    CSerialPDD * pSerialPDD = NULL;
    RETAILMSG( TRUE, (TEXT("DEBUG: CreateSerialObject %d\r\n"), DeviceArrayIndex)); 
    switch (DeviceArrayIndex) {
      case 1:        ///< UART1
        pSerialPDD = new CPdd3250Serial1(lpActivePath,pMdd, pHwObj);
        break;
    }
    if (pSerialPDD && !pSerialPDD->Init()) {
        delete pSerialPDD;
        pSerialPDD = NULL;
    }    
    return pSerialPDD;
}
void DeleteSerialObject(CSerialPDD * pSerialPDD)
{
    if (pSerialPDD)
        delete pSerialPDD;
}


