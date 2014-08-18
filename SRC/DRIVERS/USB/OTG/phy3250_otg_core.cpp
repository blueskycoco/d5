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
#include "ist.h"
#include "phy3250_xcvr.h"
#include "phy3250_otg_core.h"

#define VENDOR_ID                   0x00 // Microsoft = 0x0045E
#define PRODUCT_ID                  0x02
#define VERSION_ID                  0x14

void PHY3250_OTG::host_pll_init(void)
{	
	// Enable host need clock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBHSTND_EN;

	// Enable oscillator to PLL clock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN1;

	// Setup and enable PLL for 48MHz
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~0x1FE;
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (191 << 1); // M=192
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(3 << 9); // N=1
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (1 << 11); // P=2 (div by 4)
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 13); // Feed CCO back to PLL
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 14); // Output of PLL is postdiv
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 15); // Feed CCO sent to postdiv

	// Enable PLL and wait for lock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (1 << 16); // Enable PLL
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & 0x1) == 0); // Wait for PLL lock

	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBHSTND_EN;
}

void PHY3250_OTG::host_clock_init(void)
{	
	// Clock into PLL is 13MHz / (1 + 12) = 1MHz
	m_pCLKPWRRegs->clkpwr_usbclk_pdiv = 0x0C; // 1 Mhz clock into PLL

	// USB clock is always on
	m_pCLKPWRRegs->clkpwr_autoclock |= CLKPWR_AUTOCLK_USB_EN;

	// Restore USB control to it's default state
	m_pCLKPWRRegs->clkpwr_usb_ctrl = CLKPWR_USBCTRL_BUS_KEEPER;
	while (m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_PLL_PWRUP);

	// Enable host need clock and slave clock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (CLKPWR_USBCTRL_HCLK_EN |
		CLKPWR_USBCTRL_USBHSTND_EN);

	// Enable all OTG related clocks
	m_pOTGRegs->otg_clk_ctrl = (OTG_CLK_AHB_EN | OTG_CLK_OTG_EN |
		OTG_CLK_I2C_EN | OTG_CLK_DEV_EN | OTG_CLK_HOST_EN);
	DEBUGMSG(1, (L"CLOCK VERI 1\r\n"));
	while (m_pOTGRegs->otg_clk_ctrl != (OTG_CLK_AHB_EN |
		OTG_CLK_OTG_EN | OTG_CLK_I2C_EN | OTG_CLK_DEV_EN |
		OTG_CLK_HOST_EN));
	DEBUGMSG(1, (L"CLOCK VERI 2\r\n"));

	// Setup the USB PLL
	host_pll_init();

	// Enable USBCLKEN2 and PLL and USB clock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN2;
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_CLK_EN2) !=
		CLKPWR_USBCTRL_CLK_EN2);
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBDVND_EN;
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_USBDVND_EN) !=
		CLKPWR_USBCTRL_USBDVND_EN);

	// This is needed even if done before
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_HCLK_EN;

	// ISP1301 reg MODE_CONTROL_1 = 0x04
	// ISP1301 reg CTRL = 0x20
	// m_pOTGRegs->otg_sts_ctrl = OTG_HOST_EN;
}

PHY3250_OTG::PHY3250_OTG(LPCTSTR lpActivePath)
    : USBOTG(lpActivePath), CIST(lpActivePath, INFINITE)
{
	DWORD dwLength;

	DEBUGMSG(1, (L"PHY3250_OTG::PHY3250_OTG\r\n"));
    RETAILMSG(1, (TEXT("PHY3250_OTG::PHY3250_OTG\r\n")));
	m_hParent = CreateBusAccessHandle(lpActivePath);
//    m_hOTGFeatureEvent = NULL;
    m_pP1301Tranceiver = NULL;
    m_ActiveKeyPath = NULL;
	m_LPCI2C = NULL;
	m_pOTGRegs = NULL;
	m_pCLKPWRRegs = NULL;

    if (lpActivePath)
	{
        dwLength = _tcslen(lpActivePath) + 1;
        m_ActiveKeyPath = new TCHAR [dwLength];
        if (m_ActiveKeyPath)
		{
            StringCchCopy(m_ActiveKeyPath, dwLength, lpActivePath);
		}
    }
}

PHY3250_OTG::~PHY3250_OTG(void) 
{
    BOOL fTerminated = FALSE;

	DEBUGMSG(1, (L"PHY3250_OTG::~PHY3250_OTG\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::~PHY3250_OTG\r\n")));
	if (m_pP1301Tranceiver)
	{
        m_pP1301Tranceiver->Terminate();
	}
    fTerminated = ThreadTerminated(m_TerminationTimeout);
    ASSERT(fTerminated);
    if (m_pP1301Tranceiver) 
	{
        delete m_pP1301Tranceiver;
	}
    if (m_hParent)
	{
        ::SetDevicePowerState(m_hParent, D4, NULL);
        CloseBusAccessHandle(m_hParent);
    }
	if (m_pOTGRegs)
	{
		MmUnmapIoSpace(m_pOTGRegs, sizeof(OTG_REGS_T));
	}
	if (m_pCLKPWRRegs)
	{
		MmUnmapIoSpace(m_pCLKPWRRegs, sizeof(CLKPWR_REGS_T));
	}
	if (m_LPCI2C)
	{
		delete m_LPCI2C;
	}
    if (m_ActiveKeyPath)
	{
        delete[] m_ActiveKeyPath;
	}
//    if (m_hOTGFeatureEvent)
//	{
//        CloseHandle(m_hOTGFeatureEvent);
//	}
}
//#define VENDOR_ID                   0x00
//#define PRODUCT_ID                  0x02
//#define VERSION_ID                  0x14
BOOL PHY3250_OTG::Init(void)
{
    GIISR_INFO Info;
	PHYSICAL_ADDRESS pa;
//	int to;
    BOOL fRet = FALSE;

	DEBUGMSG(1, (L"PHY3250_OTG::Init\r\n"));

	m_SyncAccess.Lock();

	if (m_ActiveKeyPath == NULL)
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() m_ActiveKeyPath is NULL failure\r\n"));
		goto cleanup;
	}
//	m_hOTGFeatureEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
//	if (m_hOTGFeatureEvent == NULL)
//	{
//		DEBUGMSG(1, (L"PHY3250_OTG::Init() event allocation failure\r\n"));
//		goto cleanup;
//	}

	// Setup mapped register space
	pa.QuadPart = USB_OTG_BASE;
	m_pOTGRegs = (OTG_REGS_T *) MmMapIoSpace(pa,
		sizeof (OTG_REGS_T), FALSE);
	pa.QuadPart = CLK_PM_BASE;
	m_pCLKPWRRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);
	if ((m_pOTGRegs == NULL) || (m_pCLKPWRRegs == NULL))
	{
	    RETAILMSG(1, 
		    (TEXT("PHY3250_OTG::Init() failed to map registers\r\n")));
		goto cleanup;
	}
	    RETAILMSG(1, 
		    (TEXT("0x%x 0x%x\r\n"), m_pOTGRegs, m_pCLKPWRRegs));

	// Setup system and clocking
	host_clock_init();

	//Create I2C class

	m_LPCI2C = new LPC32XX_OTGI2C;
	if (m_LPCI2C == NULL)
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failed to create I2C class()\r\n"));
		goto cleanup;
	}
	if (!m_LPCI2C->Init())
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failed I2C init\r\n"));
		goto cleanup;
	}

	if (!USBOTG::Init())
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failed USBOTG::Init()\r\n"));
		goto cleanup;
	}
	if (!CIST::Init())
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failed thread init\r\n"));
		goto cleanup;
	}
	if (!IsIISRLoaded())
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failure:ISR is not loaded\r\n"));
		goto cleanup;
	}
	if (m_pP1301Tranceiver != NULL)
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failure:ISR is not loaded\r\n"));
		goto cleanup;
	}

    if (IsKeyOpened()) {
        // Get OTG transceiver info
        WORD vendorId,productId,versionId,RVID,RPID,RVEID;
	//	USHORT vendorId,productId,versionId,RVID,RPID,RVEID;//modify by jj
		RVID=ReadXCVR( VENDOR_ID, &vendorId);
		RPID=ReadXCVR( PRODUCT_ID, &productId);
		RVEID=ReadXCVR( VERSION_ID, &versionId);
  //      if (
  //          !ReadXCVR( VENDOR_ID, &vendorId) ||
  //          !ReadXCVR( PRODUCT_ID, &productId) ||
  //          !ReadXCVR( VERSION_ID, &versionId)
  //      ) 
		if(
			!RVID||
			!RPID||
			!RVEID
		   )
		{
            return FALSE;
        }
        DWORD dwTransMode = TRANSCEIVER_4_PIN ;
        if (!GetRegValue(TEXT("USBTrxMode"), (PBYTE)&dwTransMode,sizeof(dwTransMode))) {
            dwTransMode = TRANSCEIVER_4_PIN;
        }
        // Print vendor, product & version
        DEBUGMSG(1, (L"USBOTGTransceiverISP1301::Init:USB OTG Transceiver Vendor %04x  Product %04x  Version %04x\r\n",
            vendorId, productId, versionId )); 
        DEBUGMSG(1, (L"USBOTGTransceiverISP1301::Init:USB OTG Transceiver mode %04x\r\n", dwTransMode));
	}

#if 0
	// Enable USB clocks based on 13MHz crystal
	m_pCLKPWRRegs->clkpwr_usbclk_pdiv = 0x0C; // 1 Mhz clock into PLL
	m_pCLKPWRRegs->clkpwr_autoclock |= CLKPWR_AUTOCLK_USB_EN;
	m_pCLKPWRRegs->clkpwr_usb_ctrl = CLKPWR_USBCTRL_BUS_KEEPER;
	while (m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_PLL_PWRUP);
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_HCLK_EN;

	/* USB PLL configuration */
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBHSTND_EN;
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN1;
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~0x1FE;
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (191 << 1); // M=192
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(3 << 9); // N=1
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (1 << 11); // P=2 (div by 4)
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 13); // Feed CCO back to PLL
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 14); // Output of PLL is postdiv
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~(1 << 15); // Feed CCO sent to postdiv
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (1 << 16); // Enable PLL
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & 0x1) == 0); // Wait for PLL lock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBHSTND_EN;

	/* Enable USB CLKEN2 and device clock */
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_CLK_EN2;
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_CLK_EN2) !=
		CLKPWR_USBCTRL_CLK_EN2);
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_USBDVND_EN;
	while ((m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_USBDVND_EN) !=
		CLKPWR_USBCTRL_USBDVND_EN);

	/* Required even if done before! */
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_HCLK_EN;

#if 0

	// Setup USB for a 48MHz clock
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (0x0000097D | // 48x 1MHz
		CLKPWR_USBCTRL_USBDVND_EN | CLKPWR_USBCTRL_CLK_EN2 |
		CLKPWR_USBCTRL_CLK_EN1);
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= CLKPWR_USBCTRL_PLL_PWRUP;

	// Wait for USB PLL to lock
	to = 10;
	while (((m_pCLKPWRRegs->clkpwr_usb_ctrl & CLKPWR_USBCTRL_PLL_STS) == 0) &&
		(to > 0))
	{
		Sleep(10);
		to--;
	}
	if (to == 0)
	{
		// PLL didn't lock
		DEBUGMSG(1, (L"PHY3250_OTG::Init() failure:USB PLL not locked\r\n"));
		goto cleanup;
	}

	/* Add pullup */
	m_pCLKPWRRegs->clkpwr_usb_ctrl &= ~CLKPWR_USBCTRL_PD_ADD;
	m_pCLKPWRRegs->clkpwr_usb_ctrl |= (0x2 << 19);

#endif
	// Enable clock for OTG
	m_pOTGRegs->otg_clk_ctrl = (OTG_CLK_AHB_EN | OTG_CLK_OTG_EN |
		OTG_CLK_I2C_EN | OTG_CLK_DEV_EN | OTG_CLK_HOST_EN);

	// Enable OTG interrupts
	//        m_pUSBDReg->udc_otgicr = 0 ;
//        m_pUSBDReg->udc_otgisr = m_pUSBDReg->udc_otgisr;

	m_pOTGRegs->otg_sts_ctrl = OTG_HOST_EN;
	// | OTG_PLLUP_REMD | OTG_AB_HNP_TRK | OTG_BA_HNP_TRK); // TBD
	m_pOTGRegs->otg_int_clr = (OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL |
		OTG_INT_REM_PLLUP);
	m_pOTGRegs->otg_int_enab = 0;
#endif

	// Setup IISR
    Info.SysIntr = CIST::GetSysIntr();
    Info.CheckPort = FALSE;
    Info.PortIsIO = FALSE;
    Info.UseMaskReg = FALSE;
    Info.PortAddr = (DWORD) &m_pOTGRegs->otg_int_sts;
    Info.Mask = (OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL | OTG_INT_REM_PLLUP);
    Info.PortSize = sizeof(DWORD);
    if (!IntChainHandlerIoControl(IOCTL_GIISR_INFO, &Info, sizeof(Info), NULL,
		0, NULL))
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() Int chain handler error\r\n"));
		goto cleanup;
    }

	DEBUGMSG(1, (L"PHY3250_OTG::Init() Setting up m_pP1301Tranceiver\r\n"));
	m_pP1301Tranceiver = new PHY3250_XCVR(m_ActiveKeyPath, this);
	
   if (!(m_pP1301Tranceiver && m_pP1301Tranceiver->Init()))
	{
		DEBUGMSG(1, (L"PHY3250_OTG::Init() 1301 transceiver init error\r\n"));
		RETAILMSG(1, (L"PHY3250_OTG::Init() 1301 transceiver init error\r\n"));
		goto cleanup;
    }

	// Enable PSW for VBUS control
	{
	RETAILMSG(1, (TEXT("Enable PSW for VBUS control\r\n")));
		BYTE ctrl;
		if (ReadXCVR(0x12, &ctrl) == FALSE)
		{
			DEBUGMSG(1, (L"PHY3250_OTG::Init() 1301 transceiver VBUS control error\r\n"));
			RETAILMSG(1, (L"PHY3250_OTG::Init() 1301 transceiver VBUS control error\r\n"));
		}
		ctrl |= 0x40;
		WriteXCVR(0x12, ctrl);
	}


	m_SyncAccess.Unlock();
	fRet = TRUE;

cleanup:
	return fRet;
}

BOOL PHY3250_OTG::PostInit()
{
    BOOL fReturn = FALSE;

	DEBUGMSG(1, (L"PHY3250_OTG::PostInit\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::PostInit\r\n")));
	m_SyncAccess.Lock();
    if (USBOTG::PostInit()) {
#if 0
		// Enable OTG Hardware
		m_pOTGRegs->otg_int_clr = (OTG_INT_HNP_SUCC |
			OTG_INT_HNP_FAIL | OTG_INT_REM_PLLUP);
		m_pOTGRegs->otg_int_enab = (OTG_INT_HNP_SUCC |
			OTG_INT_HNP_FAIL | OTG_INT_REM_PLLUP);
//        UDCCR udccr;
//        udccr.ulValue = m_pUSBDReg->udc_cr;
//        udccr.bit.UDE=udccr.bit.OEN = 1;
//        m_pUSBDReg->udc_cr = udccr.ulValue ;
//        m_pUSBDReg->udc_otgicr = MAXDWORD ;
//        UP2OCR up2ocr;
//        up2ocr.ul = 0;
//        up2ocr.bit.HXS=0;
//        up2ocr.bit.SEOS= 4;
//        m_pUSBDReg->up2ocr = up2ocr.ul;
//        m_pUSBDReg->up3ocr = 0;
#endif        
#if 1
	// ISP1301 reg MODE_CONTROL_1 = 0x04
	// ISP1301 reg CTRL = 0x20
	m_pOTGRegs->otg_sts_ctrl = OTG_HOST_EN;
#endif
		ASSERT(m_pP1301Tranceiver!=NULL);
        CIST::CeSetPriority(m_iThreadPriority);
        CIST::IntializeInterrupt();
        fReturn = TRUE;
        
        // Let following run to handle the outstanding interrupt.
        m_pP1301Tranceiver->ISTThreadRun();
        ISTProcess();
        InterruptDone(GetSysIntr());

//		m_pOTGRegs->otg_int_enab = 0; //(OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL |
//			OTG_INT_REM_PLLUP);
//        DEBUGMSG(ZONE_OTG_FUNCTION, (L"CBulverdeOTG::PostInit: m_pUSBDReg->udc_otgicr = 0x%x\r\n",m_pUSBDReg->udc_otgicr));
//        m_pUSBDReg->udc_otgicr = MAXDWORD ;
    }

	m_SyncAccess.Unlock();

	return fReturn;
}

//
// device is turn off by Power Down, we need reintialize everything
BOOL PHY3250_OTG::PowerUp()
{
DEBUGMSG(1, (L"PHY3250_OTG::PowerUp\r\n"));
RETAILMSG(1, (TEXT("PHY3250_OTG::PowerUp\r\n")));
//    m_LPCI2C->Reinit();
    
//    UDCCR udccr;
//    udccr.ulValue = m_pUSBDReg->udc_cr;
//    udccr.bit.UDE=udccr.bit.OEN = 1;
//    m_pUSBDReg->udc_cr = udccr.ulValue ;
//    m_pUSBDReg->udc_otgicr = MAXDWORD ;
    
//    UP2OCR up2ocr;
//    up2ocr.ul = 0;
//    up2ocr.bit.HXS=0;
//    up2ocr.bit.SEOS= 4;
//    m_UsbOtgState = USBOTG_b_idle;
    
//    m_pUSBDReg->up2ocr = up2ocr.ul;
//    m_pUSBDReg->up3ocr = 0;
    
    ISTTimeout();
    return TRUE;
}
BOOL PHY3250_OTG::PowerDown()
{
DEBUGMSG(1, (L"PHY3250_OTG::PowerDown\r\n"));
RETAILMSG(1, (TEXT("PHY3250_OTG::PowerDown\r\n")));
    USBOTG_TRANSCEIVER_CTL usbOtgTransCtl;
    usbOtgTransCtl.ul = 0;
    usbOtgTransCtl.bit.DM_PullDown = 1;
    usbOtgTransCtl.bit.DP_PullDown =1;
    m_pP1301Tranceiver->SetTransceiver(usbOtgTransCtl);
    return TRUE;
}

BOOL PHY3250_OTG::IOControl(DWORD dwCode, PBYTE pBufIn, DWORD dwLenIn, PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut)
{
    BOOL bReturn = FALSE;
DEBUGMSG(1, (L"PHY3250_OTG::IOControl\r\n"));
#if 0
	switch (dwCode) {
      case IOCTL_BUS_USBOTG_BULVERDE_GET_EVENT:
        if (pBufOut && dwLenOut >= sizeof(HANDLE) ) {
            *(HANDLE *)pBufOut = m_hOTGFeatureEvent;
            bReturn = TRUE;
            if (pdwActualOut)
                *pdwActualOut = sizeof(HANDLE);
        }
        break;
      default:
        bReturn = USBOTG::IOControl(dwCode,pBufIn,dwLenIn,pBufOut,dwLenOut,pdwActualOut);
        break;
    }
    ASSERT(bReturn);
#else
    bReturn = USBOTG::IOControl(dwCode,pBufIn,dwLenIn,pBufOut,dwLenOut,pdwActualOut);
#if 0
if (pBufOut != NULL)
{
	DEBUGMSG(1, (L"PHY3250_OTG::IOControl (%d)%d 0x%x\r\n", ((dwCode - IOCTL_BUS_USBOTG_GETOTG_ENABLE_BIT) / 4), bReturn, *(PBYTE)pBufOut));
}
else
{
	DEBUGMSG(1, (L"PHY3250_OTG::IOControl (%d)%d %x\r\n", ((dwCode - IOCTL_BUS_USBOTG_GETOTG_ENABLE_BIT) / 4),
		bReturn, dwCode));
}

#endif
#endif

	return bReturn;
}

USBOTG_TRANSCEIVER_CTL PHY3250_OTG::SetupTransCtr(USBOTG_OUTPUT usbOtgOutput)
{
    USBOTG_TRANSCEIVER_CTL usbOtgTransCtl;
    usbOtgTransCtl.ul = 0;
#if 1
	usbOtgTransCtl.bit.DM_PullDown = 1;
    usbOtgTransCtl.bit.DM_PullUp= 0;
    if (usbOtgOutput.bit.loc_con) {
        usbOtgTransCtl.bit.DP_PullUp=1;
        usbOtgTransCtl.bit.DP_PullDown =0;
    }
    else {
        usbOtgTransCtl.bit.DP_PullUp=0;
        usbOtgTransCtl.bit.DP_PullDown =1;
        usbOtgTransCtl.bit.vbusDischange = 0;
    }
    usbOtgTransCtl.bit.ID_PullDown =0;
    usbOtgTransCtl.bit.vbusDrv = usbOtgOutput.bit.drv_vbus;
    usbOtgTransCtl.bit.vbusCharge = usbOtgOutput.bit.chrg_vbus ;
    //usbOtgTransCtl.bit.vbusDischange = 0;
    if (usbOtgTransCtl.bit.vbusDrv ==0 && usbOtgTransCtl.bit.vbusCharge == 0 )
        usbOtgTransCtl.bit.vbusDischange = 1;

#else
	usbOtgTransCtl.bit.DM_PullDown = 0;
    usbOtgTransCtl.bit.DM_PullUp= 0;
    if (usbOtgOutput.bit.loc_con) {
        usbOtgTransCtl.bit.DP_PullUp=0;
        usbOtgTransCtl.bit.DP_PullDown =0;
    }
    else {
        usbOtgTransCtl.bit.DP_PullUp=0;
        usbOtgTransCtl.bit.DP_PullDown =0;
        usbOtgTransCtl.bit.vbusDischange = 0;
    }
    usbOtgTransCtl.bit.ID_PullDown =0;
    usbOtgTransCtl.bit.vbusDrv = usbOtgOutput.bit.drv_vbus;
    usbOtgTransCtl.bit.vbusCharge = usbOtgOutput.bit.chrg_vbus ;
    //usbOtgTransCtl.bit.vbusDischange = 0;
    if (usbOtgTransCtl.bit.vbusDrv ==0 && usbOtgTransCtl.bit.vbusCharge == 0 )
        usbOtgTransCtl.bit.vbusDischange = 1;
#endif

DEBUGMSG(1, (L"PHY3250_OTG::SetupTransCtr 0x%x --> 0x%x\r\n", usbOtgOutput, usbOtgTransCtl));
    return usbOtgTransCtl ;
}   
    // OTG PDD Function.
BOOL PHY3250_OTG::SessionRequest(BOOL fPulseLocConn, BOOL fPulseChrgVBus)
{
DEBUGMSG(1, (L"PHY3250_OTG::SessionRequest\r\n"));

	RETAILMSG(1, (TEXT("phy3250_otg:session request\r\n")));

    m_SyncAccess.Lock();
    USBOTG_TRANSCEIVER_CTL usbOtgTransCtl = SetupTransCtr(m_UsbOtgOutputValues);
    Sleep(2);
    if (fPulseLocConn) {
        usbOtgTransCtl.bit.DP_PullUp=1;
        usbOtgTransCtl.bit.DP_PullDown =0;
        m_pP1301Tranceiver->SetTransceiver(usbOtgTransCtl);
        Sleep(10);        
        usbOtgTransCtl.bit.DP_PullUp=0;
        usbOtgTransCtl.bit.DP_PullDown =1;
        m_pP1301Tranceiver->SetTransceiver(usbOtgTransCtl);
        
    }
    usbOtgTransCtl = SetupTransCtr(m_UsbOtgOutputValues);
    if (fPulseChrgVBus) {
        usbOtgTransCtl.bit.vbusCharge =1;
        usbOtgTransCtl.bit.vbusDischange = 0;
        m_pP1301Tranceiver->SetTransceiver(usbOtgTransCtl);
        Sleep(80);
        usbOtgTransCtl.bit.vbusCharge =0;
        usbOtgTransCtl.bit.vbusDischange = 1;
        m_pP1301Tranceiver->SetTransceiver(usbOtgTransCtl);
    }

    m_SyncAccess.Unlock();
    return TRUE;
}
BOOL PHY3250_OTG::NewStateAction(USBOTG_STATES usbOtgState , USBOTG_OUTPUT usbOtgOutput) 
{
DEBUGMSG(1, (L"PHY3250_OTG::NewStateAction 0x%x 0x%x\r\n", usbOtgState, usbOtgOutput));
	RETAILMSG(1, (TEXT("phy3250_otg newstateaction\r\n")));

    m_SyncAccess.Lock();
    USBOTG_OUTPUT lusbOtgOutput = usbOtgOutput;

    if (lusbOtgOutput.bit.loc_sof) {// Switch to Host.
        m_pP1301Tranceiver->SetTransceiver(SetupTransCtr(lusbOtgOutput));
//        UP2OCR up2ocr;
//        up2ocr.ul = 0;
//        up2ocr.bit.SEOS = 5;
//        up2ocr.bit.DMPDE = up2ocr.bit.DPPDE = 1;
//        m_pUSBDReg->up2ocr = up2ocr.ul;
        //m_pUSBDReg->up3ocr = 0;
		UINT32 tmp = m_pCLKPWRRegs->clkpwr_usb_ctrl;
		tmp &= ~CLKPWR_USBCTRL_PD_ADD;
		tmp |= CLKPWR_USBCTRL_PD_ADD;
		m_pCLKPWRRegs->clkpwr_usb_ctrl = tmp;

		m_pOTGRegs->otg_sts_ctrl |= OTG_HOST_EN;
        SetEvent(GetOtgAConnectEvent());
        DEBUGMSG(1, (L"PHY3250_OTG::NewStateAction:USB Host m_pOTGRegs->otg_sts_ctrl =0x%x\r\n", m_pOTGRegs->otg_sts_ctrl));
        // We should reset bus if possible 
    } else {// Otherwise stay in Function.
        if (!m_UsbOtgInput.bit.b_usbfn_active && lusbOtgOutput.bit.loc_con) // Device Have not ready yet. 
            lusbOtgOutput.bit.loc_con = 0 ;
        m_pP1301Tranceiver->SetTransceiver(SetupTransCtr(lusbOtgOutput));
//        UP2OCR up2ocr;
//        up2ocr.ul = 0;
//        up2ocr.bit.HXS=0;
//        up2ocr.bit.SEOS = 4;
//        m_pUSBDReg->up2ocr = up2ocr.ul;
//        m_pUSBDReg->up3ocr = 0;
		UINT32 tmp = m_pCLKPWRRegs->clkpwr_usb_ctrl;
		tmp &= ~CLKPWR_USBCTRL_PD_ADD;
		tmp |= CLKPWR_USBCTRL_PD_ADD;
		m_pCLKPWRRegs->clkpwr_usb_ctrl = tmp;

		m_pOTGRegs->otg_sts_ctrl &= ~OTG_HOST_EN;
        DEBUGMSG(1, (L"PHY3250_OTG::NewStateAction:USB Function m_pOTGRegs->otg_sts_ctrl =0x%x\r\n",m_pOTGRegs->otg_sts_ctrl));
    }

    if (lusbOtgOutput.bit.loc_sof || lusbOtgOutput.bit.loc_con) 
        m_pP1301Tranceiver->EnableDataLineInterrupt(FALSE);
    else
        m_pP1301Tranceiver->EnableDataLineInterrupt(TRUE);

#if 0
		{
		int idx;
		UNS_8 data;
		UNS_8 regs [] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
			0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x12, 0x13, 0x14, 0x15,
			0xFF};

		idx = 0;
		while (regs[idx] != 0xFF)
		{
			if (ReadXCVR(regs[idx], &data) == FALSE)
			{
				DEBUGMSG(1, (L"PHY3250_OTG::REG 0x%x read error\r\n", regs[idx]));
			}
			else
			{
				DEBUGMSG(1, (L"PHY3250_OTG::REG 0x%x = 0x%x\r\n", regs[idx], data));
			}

			idx++;
		}

	}
#endif
		m_SyncAccess.Unlock();

	return TRUE;
}
BOOL PHY3250_OTG::IsSE0()
{
    USBOTG_TRANSCEIVER_STATUS usbTransceiverStatus;
DEBUGMSG(1, (L"PHY3250_OTG::IsSE0\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::IsSE0\r\n")));//jj

    if (m_pP1301Tranceiver && m_pP1301Tranceiver->GetTransceiver(&usbTransceiverStatus)) {
        return ((usbTransceiverStatus.bit.DP_HI==0 && usbTransceiverStatus.bit.DM_HI==0)?TRUE:FALSE);
    }
    else
        return FALSE;
}
BOOL PHY3250_OTG::UpdateInput() 
{
    USBOTG_TRANSCEIVER_STATUS usbTransceiverStatus;
DEBUGMSG(1, (L"PHY3250_OTG::UpdateInput\r\n"));
    if (m_pP1301Tranceiver && m_pP1301Tranceiver->GetTransceiver(&usbTransceiverStatus)) {
DEBUGMSG(1, (L"PHY3250_OTG::UpdateInput usbTransceiverStatus = 0x%x 0x%x\r\n", usbTransceiverStatus, m_UsbOtgOutputValues));
        if (m_UsbOtgOutputValues.bit.loc_con || m_UsbOtgOutputValues.bit.loc_sof)
            m_UsbOtgInput.bit.a_conn = m_UsbOtgInput.bit.b_conn =  1;
        else
            m_UsbOtgInput.bit.a_conn = m_UsbOtgInput.bit.b_conn = 
                ((usbTransceiverStatus.bit.DP_HI==0 && usbTransceiverStatus.bit.DM_HI==0)?0:1) ;
        m_UsbOtgInput.bit.a_sess_vld = usbTransceiverStatus.bit.aSessValid;
        
        if (m_UsbOtgOutputValues.bit.drv_vbus)
            //we can't set this because it make hw and sw un-sync
            m_UsbOtgInput.bit.a_vbus_vld = 1;
        else
            m_UsbOtgInput.bit.a_vbus_vld = usbTransceiverStatus.bit.aBusValid; 
        //
        m_UsbOtgInput.bit.b_sess_end = usbTransceiverStatus.bit.bSessEnd ;
        m_UsbOtgInput.bit.b_sess_vld = usbTransceiverStatus.bit.bSessValid;
        m_UsbOtgInput.bit.id = (usbTransceiverStatus.bit.ID!=0?1:0);
        m_UsbOtgInput.bit.a_srp_det = m_UsbOtgInput.bit.a_sess_vld ;
        DEBUGMSG(1, (L"PHY3250_OTG::UpdateInput:m_UsbOtgInput.ul=%x\r\n",m_UsbOtgInput.ul));
        // Update Output if there is any.
/*
        USBOTG_OUTPUT usbOtgOutput = m_UsbOtgOutputValues;
        if (!m_UsbOtgInput.bit.b_usbfn_active && usbOtgOutput.bit.loc_con) // Device Have not ready yet. 
            usbOtgOutput.bit.loc_con = 0 ;
        m_pP1301Tranceiver->SetTransceiver(SetupTransCtr(usbOtgOutput));
*/
    }
    DEBUGMSG(ZONE_OTG_FUNCTION, (L"-CBulverdeOTG::UpdateInput:\r\n"));
    return TRUE;
}
BOOL PHY3250_OTG::StateChangeNotification (USBOTG_TRANSCEIVER_STATUS_CHANGE usbStatusChange, USBOTG_TRANSCEIVER_STATUS usbTransceiverStatus)
{
DEBUGMSG(1, (L"PHY3250_OTG::StateChangeNotification\r\n"));
    m_SyncAccess.Lock();
    UpdateInput();
    m_SyncAccess.Unlock();
    EventNotification();
    return TRUE;
}

BOOL PHY3250_OTG::ISTProcess()
{
//	UINT32 otgisr;

	DEBUGMSG(1, (L"PHY3250_OTG::ISTProcess\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::Istprocess\r\n")));//jj


//	PREFAST_ASSERT(m_pUSBDReg != NULL);
    m_SyncAccess.Lock();

#if 0
	UDCOTG udcOtgIsr;
    udcOtgIsr.ul = m_pUSBDReg->udc_otgisr; // 
    DEBUGMSG(1, (L"CBulverdeOTG::ISTProcess() udcOtgIsr=0x%x\r\n",udcOtgIsr.ul));
    if (udcOtgIsr.bit.XR || udcOtgIsr.bit.XF)
	{
        m_pP1301Tranceiver->ISTThreadRun();
    }
    if (udcOtgIsr.bit.SF)
	{
        SetEvent(m_hOTGFeatureEvent);
    }

#else
//	otgisr = m_pOTGRegs->otg_int_sts;
//    DEBUGMSG(1, (L"PHY3250_OTG::ISTProcess() otgisr=0x%x\r\n", otgisr));
//    if (otgisr & (OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL))
//	{
        m_pP1301Tranceiver->ISTThreadRun();
//		m_pOTGRegs->otg_int_clr = (OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL);
//    }
//    if (otgisr & OTG_INT_REM_PLLUP)
//	{
//        SetEvent(m_hOTGFeatureEvent); // TBD
//		m_pOTGRegs->otg_int_clr = OTG_INT_REM_PLLUP;
//    }

#endif
//    m_pUSBDReg->udc_otgisr = udcOtgIsr.ul; // Clear the bit.
//	m_pOTGRegs->otg_int_clr = (OTG_INT_HNP_SUCC | OTG_INT_HNP_FAIL |
//		OTG_INT_REM_PLLUP);

	m_SyncAccess.Unlock();

	return TRUE;
    
}

BOOL PHY3250_OTG::ISTTimeout()
{
    USBOTG_TRANSCEIVER_STATUS usbOtgTransStatus;
    USBOTG_TRANSCEIVER_STATUS_CHANGE usbOtgStatusChange;

	DEBUGMSG(1, (L"PHY3250_OTG::ISTTimeout\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::Isttimeout\r\n")));//jj

	usbOtgStatusChange.ul = usbOtgTransStatus.ul = 0;
    m_pP1301Tranceiver->GetISP1301Change(&usbOtgStatusChange);    
    m_pP1301Tranceiver->GetTransceiver(&usbOtgTransStatus);
    
//    DEBUGMSG(ZONE_OTG_THREAD, (L"CBulverdeOTG::ISTTimeout() udcOtgIsr=0x%x,udcOtgicr=0x%x,m_pUSBDReg+0x1c = 0x%x\r\n",
//        m_pUSBDReg->udc_otgisr,m_pUSBDReg->udc_otgicr, *(PDWORD)((PBYTE)m_pUSBDReg+0x1c)));
    DEBUGMSG(1, (L"PHY3250_OTG::ISTTimeout() usbOtgStatusChange =0x%x, usbOtgTransStatus= 0x%x\r\n",
        usbOtgStatusChange.ul, usbOtgTransStatus.ul));
    return TRUE;
}

// Class Factory.
USBOTG *CreateUSBOTGObject(LPTSTR lpActivePath)
{
DEBUGMSG(1, (L"CreateUSBOTGObject\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::createusb otg\r\n")));//jj

    return new PHY3250_OTG(lpActivePath);
}
void DeleteUSBOTGObject(USBOTG *pUsbOtg)
{
DEBUGMSG(1, (L"DeleteUSBOTGObject\r\n"));
	RETAILMSG(1, (TEXT("PHY3250_OTG::delete usb otg\r\n")));//jj

    if (pUsbOtg)
	{
        delete pUsbOtg;
	}
}
