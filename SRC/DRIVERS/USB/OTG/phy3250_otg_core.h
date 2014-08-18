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

#pragma once

#include <usbotg.hpp>
#include <usbotgxr.hpp>
#include "lpc32xx_otg_i2c.h"
#include "lpc32xx_otg.h"
#include "lpc32xx_clkpwr.h"
#include "ist.h"

class PHY3250_OTG :public USBOTG, public CIST
{
public:
    PHY3250_OTG(LPCTSTR lpActivePath);
    ~PHY3250_OTG(void);
	BOOL Init(void);

	BOOL PostInit() ;

    // Overwrite 
    virtual BOOL IOControl(DWORD dwCode, PBYTE pBufIn, DWORD dwLenIn, PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut);
    virtual BOOL PowerUp() ;
    virtual BOOL PowerDown();
    
    // OTG PDD Function.
    BOOL    SessionRequest(BOOL fPulseLocConn, BOOL fPulseChrgVBus);
    BOOL    DischargVBus()  { return TRUE; };
    BOOL    NewStateAction(USBOTG_STATES usbOtgState , USBOTG_OUTPUT usbOtgOutput) ;
    BOOL    IsSE0();
    BOOL    UpdateInput() ;
    BOOL    StateChangeNotification (USBOTG_TRANSCEIVER_STATUS_CHANGE , USBOTG_TRANSCEIVER_STATUS);
    USBOTG_MODE UsbOtgConfigure(USBOTG_MODE usbOtgMode ) { return usbOtgMode; };

    BOOL ReadXCVR(UCHAR reg, USHORT *pData) {
        m_SyncAccess.Lock();
        BOOL fRet = m_LPCI2C->ReadReg(reg, (UINT8 *) pData, sizeof(*pData));
        m_SyncAccess.Unlock();
        return fRet;
    }
    BOOL ReadXCVR(UCHAR reg, BYTE *pData) {
        m_SyncAccess.Lock();
        BOOL fRet = m_LPCI2C->ReadReg(reg, pData, sizeof(*pData));
        m_SyncAccess.Unlock();
        return fRet;
    }
    BOOL WriteXCVR(UCHAR reg, BYTE Data) {
        m_SyncAccess.Lock();
        BOOL fRet = m_LPCI2C->WriteReg(reg, &Data, sizeof(Data));
        m_SyncAccess.Unlock();
        return fRet;
    }

protected:
    HANDLE  m_hParent;
//    HANDLE  m_hOTGFeatureEvent;
    USBOTGTransceiverISP1301 *m_pP1301Tranceiver; 
    LPTSTR  m_ActiveKeyPath;
	class LPC32XX_OTGI2C *m_LPCI2C; // CBulverdeI2COTG m_BulverdeI2COtg;

//    PVOID       m_pUSBDStaticAddr; // Used by Installable ISR
//    volatile PBULVERDE_USBD_REG m_pUSBDReg;
//    P_XLLP_GPIO_T v_pGPIORegs ;

	OTG_REGS_T *m_pOTGRegs;
	CLKPWR_REGS_T *m_pCLKPWRRegs;

	USBOTG_TRANSCEIVER_CTL SetupTransCtr(USBOTG_OUTPUT usbOtgOutput);
private:
    virtual BOOL    ISTProcess() ;
    virtual BOOL    ISTTimeout() ;

	void host_clock_init(void);
	void host_pll_init(void);
};
