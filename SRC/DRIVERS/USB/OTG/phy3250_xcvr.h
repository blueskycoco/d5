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
#include "phy3250_otg_core.h"

class PHY3250_XCVR: public USBOTGTransceiverISP1301 {
public:
    PHY3250_XCVR(LPCTSTR lpActivePath, class PHY3250_OTG *pOtgCl)
    :   USBOTGTransceiverISP1301(lpActivePath), m_pOtgCl(pOtgCl)
    {
    }
	~PHY3250_XCVR(void)
	{
	}

	BOOL Init(void)
	{
		return ((m_pOtgCl != NULL )&& 
			(USBOTGTransceiverISP1301::Init()));
	}

	virtual BOOL StateChangeNotification(USBOTG_TRANSCEIVER_STATUS_CHANGE usbStatusChange,
		                                 USBOTG_TRANSCEIVER_STATUS usbTransceiverStatus)
	{
        PREFAST_ASSERT(m_pOtgCl);
        return m_pOtgCl->StateChangeNotification(usbStatusChange, usbTransceiverStatus);
    }

private:
    virtual BOOL ReadISP1301W(UCHAR reg,
		                      USHORT *pData)
	{
        PREFAST_ASSERT(m_pOtgCl != NULL);
        return m_pOtgCl->ReadXCVR(reg, pData);
    }
    virtual BOOL ReadISP1301(UCHAR reg,
		                     BYTE *pData)
	{
        PREFAST_ASSERT(m_pOtgCl != NULL);
        return m_pOtgCl->ReadXCVR(reg,pData);
    }
    virtual BOOL WriteISP1301(UCHAR reg,
		                      BYTE Data)
	{
        PREFAST_ASSERT(m_pOtgCl != NULL);
        return m_pOtgCl->WriteXCVR(reg, Data);
    }

	class PHY3250_OTG *m_pOtgCl;
};
