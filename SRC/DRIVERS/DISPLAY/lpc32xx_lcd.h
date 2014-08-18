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
// lpc32xx_lcd.c
//
// LCD driver class header
//

#pragma once

#include "lpc32xx_clcdc.h"
#include "lpc32xx_clkpwr.h"

//------------------------------------------------------------------------------

class LPC32XXLCD : public GPE
{
public:
    LPC32XXLCD();
    virtual ~LPC32XXLCD();

    virtual int InVBlank();
    virtual int NumModes();

    virtual SCODE SetPalette(const PALETTEENTRY*, WORD, WORD);

    virtual SCODE AllocSurface(GPESurf**, int, int, EGPEFormat, int);
    virtual SCODE Line(GPELineParms*, EGPEPhase);
    virtual SCODE BltPrepare(GPEBltParms*);
    virtual SCODE BltComplete(GPEBltParms*);

    virtual SCODE GetModeInfo(GPEMode*, int);
    virtual SCODE SetMode(int, HPALETTE*);
    virtual SCODE SetPointerShape(GPESurf*, GPESurf*, int, int, int, int);
    virtual SCODE MovePointer(int, int);

    virtual VOID PowerHandler(BOOL bOff);

	virtual void WaitForNotBusy();
	virtual int IsBusy();
	virtual void GetPhysicalVideoMemory(
		unsigned long * pPhysicalMemoryBase,
	    unsigned long * pVideoMemorySize
	    );
	virtual void GetVirtualVideoMemory(
	    unsigned long * pVirtualMemoryBase,
	    unsigned long * pVideoMemorySize
	    );

private:
    // Actual mode info (width, height, bpp, frequency, and format)
    GPEMode m_gpeMode;
    DWORD m_colorDepth;
    DWORD m_cbScanLineLength;
	DWORD m_FrameBufferSize;
	DWORD m_VirtualFrameBuffer;

    // Driver emulate cursor
    BOOL   m_cursorDisabled;
    BOOL   m_cursorVisible;
    BOOL   m_cursorForcedOff;
    RECTL  m_cursorRect;
    POINTL m_cursorSize;
    POINTL m_cursorHotspot;
    UCHAR* m_cursorStore;
    UCHAR* m_cursorXor;
    UCHAR* m_cursorAnd;

    // Helper functions
    VOID CursorOn();
    VOID CursorOff();
    SCODE WrappedEmulatedLine(GPELineParms*);

	// Hardware init function
	void lpc32xx_hw_init(void);
	void lcd_update_clock(UINT32 desired_clock);

	CLCDC_REGS_T *pLCDRegs;
	CLKPWR_REGS_T *pCLKPWRRegs;
};
