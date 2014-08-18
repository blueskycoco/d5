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
// lpc32xx_lcd.cpp
//
// This file implements basic flat display driver. Informations about
// supported display modes and switching between them are offloaded to kernel
// via KernelIoControl calls.
//

#define DDI 1
#include "precomp.h"
#include <windows.h>
#include <types.h>
#include <winddi.h>
#include <gpe.h>
#include <emul.h>
#include <ceddk.h>
#include <dispperf.h>
#include <pkfuncs.h>
#include <ddhal.h>
#include "lpc32xx_lcd.h"
#include "lcd_panel_types.h"
#include "lpc32xx_gpio.h"
#include "bsp.h"

#define dim(x) (sizeof(x) / sizeof(x[0]))

//------------------------------------------------------------------------------
// Debug Zones (only for retail)
//#ifndef SHIP_BUILD
INSTANTIATE_GPE_ZONES(0x03, "MGDI Driver", "error", "warn")
#undef  ZONE_ERROR
#undef  ZONE_WARN
#undef  ZONE_FUNCTION
#undef  ZONE_INIT
#undef  ZONE_INFO

#define ZONE_ERROR          DEBUGZONE(0)
#define ZONE_WARN           DEBUGZONE(1)
#define ZONE_FUNCTION       DEBUGZONE(2)
#define ZONE_INIT           DEBUGZONE(3)
#define ZONE_INFO           DEBUGZONE(4)
//#endif

//------------------------------------------------------------------------------
// Static variables  
static GPE *g_pGPE = (GPE *) NULL;
static TCHAR gszBaseInstance[256] = _T("Drivers\\Display\\LPC32XX\\CONFIG");
static DWORD PanelType = 0;

//------------------------------------------------------------------------------
// External functions
BOOL APIENTRY GPEEnableDriver(
   ULONG version, ULONG cj, DRVENABLEDATA *pData, ENGCALLBACKS *pCallbacks
);

//------------------------------------------------------------------------------
//
//  Function:  DisplayInit
//
//  The GWES will invoke this routine once prior to making any other calls
//  into the driver. This routine needs to save its instance path information
//  and return TRUE.  If it returns FALSE, GWES will abort the display
//  initialization.
//
BOOL DisplayInit(LPCTSTR pszInstance, DWORD monitors)
{
	// Save copy of instance base
    if(pszInstance != NULL) {
        _tcsncpy(gszBaseInstance, pszInstance, dim(gszBaseInstance));
    }

    g_pGPE = new LPC32XXLCD;

    return TRUE;
}

//------------------------------------------------------------------------------
//
//  Function:  DrvEnableDriver
//
//  This function must be exported from display driver. As long as we use
//  GPE class implementation we don't need do anything else than call
//  GPEEnableDriver library function.
//
BOOL DrvEnableDriver(
    ULONG version, ULONG cj, DRVENABLEDATA *pData, ENGCALLBACKS *pCallbacks
) {
    BOOL rc;

    rc = GPEEnableDriver(version, cj, pData, pCallbacks);

    return rc;
}

//------------------------------------------------------------------------------
//
//  Function:  DrvGetMasks
//
ULONG *APIENTRY DrvGetMasks(DHPDEV dhpdev)
{
    return LCDPanelList[PanelType].m_BitMasks;
}

//------------------------------------------------------------------------------
//
//  Function:  RegisterDDHALAPI
//
VOID RegisterDDHALAPI()
{
    DEBUGMSG(ZONE_FUNCTION, (L"+RegisterDDHALAPI\r\n"));
}

//------------------------------------------------------------------------------
//
//  Function:  GetGPE
//
//  This function is called from GPE class library to get pointer to class
//  deriver from GPE.
//
GPE* GetGPE()
{
    return g_pGPE;
}    

//------------------------------------------------------------------------------
//
//  Function:  lcd_update_clock
//
//  Sets the LCD clock.
//
void LPC32XXLCD::lcd_update_clock(UINT32 desired_clock)
{
    UINT32 pixel_div, tmp;
    DWORD clk, bytesret;

	// Get base clock for LCD (HCLK)
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
		sizeof (clk), &bytesret) == FALSE)
	{
		// Cannot get clock
        RETAILMSG(1, (_T("lcd_update_clock: lcd_update_clock: Error getting LCD base clock\r\n")));
        RETAILMSG(1, (_T("lcd_update_clock: Using 104MHz for the base clock\r\n")));
        clk = 104000000;
	}

	// Find closest clock divider to get clock rate
	pixel_div = 1;
	while (((clk / pixel_div) > desired_clock) && (pixel_div < 0x3F)) {
		pixel_div++;
	}

   DEBUGMSG(ZONE_INFO, (_T("LCD: lcd_update_clock: Base clock %d, divider %d, LCD clock %d\r\n"),
	   clk, pixel_div, clk/pixel_div));

	tmp = pLCDRegs->lcdpol;

	if (pixel_div <= 1)
    {
        /* Pixel clock divider is 1, skip divider logic */
        tmp |= CLCDC_LCDTIMING2_BCD;
    }
    else
    {
        // Add in new divider
        tmp &= ~(CLCDC_LCDTIMING2_BCD | CLCDC_LCDTIMING2_PCD(0x1F) | 0xF8000000);
		tmp |= (pixel_div & 0x1F);
		tmp |= (((pixel_div >> 5) & 0x1F) << 27);
    }

    pLCDRegs->lcdpol = tmp;
}

//------------------------------------------------------------------------------
//
//  Constructor
//
void LPC32XXLCD::lpc32xx_hw_init(void)
{
    HKEY hkDisplay = NULL;
    DWORD dwStatus, dwType, dwSize, tmp, bsize, UseIRAM, phya;
    PHYSICAL_ADDRESS pa;
	GPIO_REGS_T *pGPIOREGs;

	// Open the registry key and read our configuration
    dwStatus = RegOpenKeyEx(HKEY_LOCAL_MACHINE, gszBaseInstance, 0, 0, &hkDisplay);
    if (dwStatus != ERROR_SUCCESS) {
        RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Error opening registry!\r\n")));
        return;
    }
	else
	{
		// Get panel type from the register, the panel type matches the panels
		// index specified in the panels.cpp list.
		PanelType = 0xffff;
        dwSize = sizeof(DWORD);
	    dwType = REG_DWORD;
        dwStatus = RegQueryValueEx(hkDisplay, _T("LCDPanelType"), NULL, &dwType, 
            (LPBYTE) &PanelType, &dwSize);

	    // Sanity checks
		if (dwStatus != ERROR_SUCCESS) {
	        DEBUGMSG(ZONE_WARN,
				(_T("LCD: lpc32xx_hw_init: No panel type found, using 0!\r\n")));
			PanelType = 0;
		}
		if (PanelType > (LCD_MAX_PANELS - 1)) {
		    RETAILMSG(1,
				(_T("LCD: lpc32xx_hw_init: Invalid panel type, use 0!\r\n")));
			PanelType = 0;
	    }

		// Get IRAM use flag
		// Get panel type from the register, the panel type matches the panels
		// index specified in the panels.cpp list.
		UseIRAM = 1;
        dwSize = sizeof(DWORD);
	    dwType = REG_DWORD;
        dwStatus = RegQueryValueEx(hkDisplay, _T("UseIRAM"), NULL, &dwType, 
            (LPBYTE) &UseIRAM, &dwSize);

		if (dwStatus != ERROR_SUCCESS) {
	        DEBUGMSG(ZONE_WARN,
				(_T("LCD: lpc32xx_hw_init: Using IRAM for LCD buffer\r\n")));
			UseIRAM = 1;
		}
	}
    
    // Close the registry key
    if (hkDisplay != NULL) {
        RegCloseKey(hkDisplay);
    }

	// Only TFT panels on 16bpp and 24bpp are supported
	if (LCDPanelList[PanelType].lcd_panel_type != TFT) {
        RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Not a TFT panel!\r\n")));
        return;
    }

	// Get pointer to registers used in this driver
    pa.QuadPart = LCD_BASE;
	pLCDRegs = (CLCDC_REGS_T *) MmMapIoSpace(pa, sizeof (CLCDC_REGS_T), FALSE);
    pa.QuadPart = CLK_PM_BASE;
	pCLKPWRRegs = (CLKPWR_REGS_T *)
		MmMapIoSpace(pa, sizeof (CLKPWR_REGS_T), FALSE);
	
	if ((pLCDRegs == NULL) || (pCLKPWRRegs == NULL))
	{
		RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Critcal error: cannot map registers!\r\n")));
		if (pLCDRegs != NULL)
		{
			MmUnmapIoSpace(pLCDRegs, sizeof(CLCDC_REGS_T));
		}
		if (pCLKPWRRegs != NULL)
		{
			MmUnmapIoSpace(pCLKPWRRegs, sizeof(CLKPWR_REGS_T));
		}

		return;
	}

	if (LCDPanelList[PanelType].bits_per_pixel == 16) {
		tmp = CLKPWR_LCDCTRL_LCDTYPE_TFT16;
		bsize = 2;
	}
	else if (LCDPanelList[PanelType].bits_per_pixel == 24) {
		tmp = CLKPWR_LCDCTRL_LCDTYPE_TFT24;
		bsize = 4;
	}
	else {
        RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Unsupported color depth!\r\n")));
        return;
	}

	// Enough memory for frame buffer?
	if ((LCDPanelList[PanelType].pixels_per_line *
		LCDPanelList[PanelType].lines_per_panel * bsize) >
		IMAGE_WINCE_LCD_BUFFERS_SIZE) {
        RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Frame buffer not big enough!\r\n")));
        return;
	}

	// Enable LCD clocks and setup prescaler to 1
	pCLKPWRRegs->clkpwr_lcdclk_ctrl = (tmp | CLKPWR_LCDCTRL_CLK_EN);

	// Configure LCD based on selected panel type
    pLCDRegs->lcdctrl &= ~CLCDC_LCDCTRL_ENABLE;

    // Generate the timing 0 word
    tmp = (CLCDC_LCDTIMING0_PPL(LCDPanelList[PanelType].pixels_per_line) |
        CLCDC_LCDTIMING0_HSW(LCDPanelList[PanelType].h_sync_pulse_width) |
        CLCDC_LCDTIMING0_HFP(LCDPanelList[PanelType].h_front_porch) |
        CLCDC_LCDTIMING0_HBP(LCDPanelList[PanelType].h_back_porch));
    pLCDRegs->lcdtimh = tmp;

    // Generate the timing 1 word
    tmp = (CLCDC_LCDTIMING1_LPP(LCDPanelList[PanelType].lines_per_panel) |
        CLCDC_LCDTIMING1_VSW(LCDPanelList[PanelType].v_sync_pulse_width) |
        CLCDC_LCDTIMING1_VFP(LCDPanelList[PanelType].v_front_porch) |
        CLCDC_LCDTIMING1_VBP(LCDPanelList[PanelType].v_back_porch));
    pLCDRegs->lcdtimv = tmp;

    // Generate the timing 2 word
    tmp = CLCDC_LCDTIMING2_ACB(LCDPanelList[PanelType].ac_bias_frequency);
    if (LCDPanelList[PanelType].invert_output_enable != 0)
    {
        tmp |= CLCDC_LCDTIMING2_IOE;
    }
    if (LCDPanelList[PanelType].invert_panel_clock != 0)
    {
        tmp |= CLCDC_LCDTIMING2_IPC;
    }
    if (LCDPanelList[PanelType].invert_hsync != 0)
    {
        tmp |= CLCDC_LCDTIMING2_IHS;
    }
    if (LCDPanelList[PanelType].invert_vsync != 0)
    {
        tmp |= CLCDC_LCDTIMING2_IVS;
    }

    // Clocks per line and pixels per line are the same
    tmp = tmp | CLCDC_LCDTIMING2_CPL(
		LCDPanelList[PanelType].pixels_per_line - 1);
    pLCDRegs->lcdpol = tmp;

    // Update timing 2 word with correct clock data
    lcd_update_clock(LCDPanelList[PanelType].optimal_clock);

    // Skip timing 3 word - just set to 0x0
    pLCDRegs->lcdle = 0x00000000;

    // Default with all interrupts off
    pLCDRegs->lcdintrenable = 0x00000000; 

#if 0 // SDRAM only version
    // Default configuration is 16 bits per pixel with blue and
    // green not swapped
	if (LCDPanelList[PanelType].bits_per_pixel == 16) {
		tmp = CLCDC_LCDCTRL_BPP16_565;
	}
	else
	{
		tmp = CLCDC_LCDCTRL_BPP24;
	}
    pLCDRegs->lcdctrl = (tmp | CLCDC_LCDCTRL_RGB | CLCDC_LCDCTRL_TFT);

	// Setup frame buffer
	pLCDRegs->lcdupbase = IMAGE_WINCE_LCD_BUFFERS_PA;

	// Enable LCD
	pLCDRegs->lcdctrl |= (CLCDC_LCDCTRL_ENABLE | CLCDC_LCDCTRL_PWR);

	// Enable LCD and backlight power
    pa.QuadPart = GPIO_BASE;
	pGPIOREGs = (GPIO_REGS_T *) MmMapIoSpace(pa, sizeof (GPIO_REGS_T), FALSE);
	pGPIOREGs->pio_outp_set = _BIT(0);
	pGPIOREGs->pio_outp_clr = _BIT(4);
    MmUnmapIoSpace((PVOID) pGPIOREGs, 0);

    // Setup up display mode related constants
    m_nScreenWidth = LCDPanelList[PanelType].pixels_per_line;
    m_nScreenHeight = LCDPanelList[PanelType].lines_per_panel;
    m_colorDepth = LCDPanelList[PanelType].bits_per_pixel;
    m_cbScanLineLength = LCDPanelList[PanelType].pixels_per_line * bsize;
    m_FrameBufferSize = m_nScreenHeight * m_cbScanLineLength;

	// Allocate virtual memory for frame buffer
    m_VirtualFrameBuffer = (DWORD) VirtualAlloc(NULL, m_FrameBufferSize,
		MEM_RESERVE, (PAGE_READWRITE | PAGE_NOCACHE));
    if (!m_VirtualFrameBuffer)
    {
        RETAILMSG(1, (TEXT("m_VirtualFrameBuffer is not allocated\n\r")));
        return;
    }
    if (!VirtualCopy((PVOID)m_VirtualFrameBuffer, (PVOID) (IMAGE_WINCE_LCD_BUFFERS_PA >> 8),
		m_FrameBufferSize, (PAGE_READWRITE | PAGE_NOCACHE | PAGE_PHYSICAL)))
    {
        RETAILMSG(1, (TEXT("m_VirtualFrameBuffer is not mapped\n\r")));
        VirtualFree((PVOID)m_VirtualFrameBuffer, 0, MEM_RELEASE);
        return;
    }

    RETAILMSG(1, (TEXT("m_VirtualFrameBuffer is mapped at %x(PHY : %x)\n\r"),
		m_VirtualFrameBuffer, IMAGE_WINCE_LCD_BUFFERS_PA));

#else // SDRAM and IRAM version
    // Default configuration is 16 bits per pixel with blue and
    // green not swapped
	if (LCDPanelList[PanelType].bits_per_pixel == 16) {
		tmp = CLCDC_LCDCTRL_BPP16_565;
	}
	else
	{
		tmp = CLCDC_LCDCTRL_BPP24;
	}
    pLCDRegs->lcdctrl = (tmp | CLCDC_LCDCTRL_RGB | CLCDC_LCDCTRL_TFT);

    // Setup up display mode related constants
    m_nScreenWidth = LCDPanelList[PanelType].pixels_per_line;
    m_nScreenHeight = LCDPanelList[PanelType].lines_per_panel;
    m_colorDepth = LCDPanelList[PanelType].bits_per_pixel;
    m_cbScanLineLength = LCDPanelList[PanelType].pixels_per_line * bsize;
    m_FrameBufferSize = m_nScreenHeight * m_cbScanLineLength;

	// Allocate virtual memory for frame buffer
    m_VirtualFrameBuffer = (DWORD) VirtualAlloc(NULL, m_FrameBufferSize,
		MEM_RESERVE, (PAGE_READWRITE | PAGE_NOCACHE));
    if (!m_VirtualFrameBuffer)
    {
        RETAILMSG(1, (TEXT("m_VirtualFrameBuffer is not allocated\n\r")));
        return;
    }

	// Setup frame buffer
	phya = IMAGE_WINCE_LCD_BUFFERS_PA;
	if ((UseIRAM == 1) && (m_FrameBufferSize <= (256 * 1024)))
	{
		phya = DEVICE_IRAM_PA;
        RETAILMSG(1, (TEXT("Using IRAM for LCD buffer!!\n\r")));
	}

	if (!VirtualCopy((PVOID)m_VirtualFrameBuffer, (PVOID) (phya >> 8),
		m_FrameBufferSize, (PAGE_READWRITE | PAGE_NOCACHE | PAGE_PHYSICAL)))
   {
	    RETAILMSG(1, (TEXT("m_VirtualFrameBuffer is not mapped\n\r")));
        VirtualFree((PVOID)m_VirtualFrameBuffer, 0, MEM_RELEASE);
	    return;
    }

	// Enable LCD
	pLCDRegs->lcdupbase = phya;
	pLCDRegs->lcdctrl |= (CLCDC_LCDCTRL_ENABLE | CLCDC_LCDCTRL_PWR);

	// Enable LCD and backlight power
    pa.QuadPart = GPIO_BASE;
	pGPIOREGs = (GPIO_REGS_T *) MmMapIoSpace(pa, sizeof (GPIO_REGS_T), FALSE);
	pGPIOREGs->pio_outp_set = _BIT(0);
	pGPIOREGs->pio_outp_clr = _BIT(4);
    MmUnmapIoSpace((PVOID) pGPIOREGs, 0);

    DEBUGMSG(1, (TEXT("m_VirtualFrameBuffer is mapped at %x(PHY : %x)\n\r"),
		m_VirtualFrameBuffer, phya));
#endif
}

//------------------------------------------------------------------------------
//
//  Constructor
//
LPC32XXLCD::LPC32XXLCD()
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("LPC32XXLCD::LPC32XXLCD\r\n")));

	// Initialize hardware
	lpc32xx_hw_init();

    // setup ModeInfo structure
    m_gpeMode.modeId = 0;
    m_gpeMode.width = m_nScreenWidth;
    m_gpeMode.height = m_nScreenHeight;
	if (LCDPanelList[PanelType].bits_per_pixel == 16) {
		m_gpeMode.format = gpe16Bpp;
	}
	else
	{
		m_gpeMode.format = gpe24Bpp;
	}
    m_gpeMode.Bpp = m_colorDepth;
    m_gpeMode.frequency = 60;
    m_pMode = &m_gpeMode;

	// Allocate display surface
    m_pPrimarySurface = new GPESurf(m_nScreenWidth, m_nScreenHeight,
		(void*)(m_VirtualFrameBuffer), m_cbScanLineLength, m_gpeMode.format); 
    if (m_pPrimarySurface)
    {
        memset ((void*)m_pPrimarySurface->Buffer(), 0x0, m_FrameBufferSize);
    }

	m_cursorVisible = FALSE;
    m_cursorDisabled = TRUE;
    m_cursorForcedOff = FALSE;
    memset(&m_cursorRect, 0x0, sizeof(m_cursorRect));
    m_cursorStore = NULL;
    m_cursorXor = NULL;
    m_cursorAnd = NULL;

    DEBUGMSG(ZONE_FUNCTION, (TEXT("--LPC32XXLCD::LPC32XXLCD\r\n")));
}

//------------------------------------------------------------------------------
//
//  Dectructor
//
LPC32XXLCD::~LPC32XXLCD()
{
    delete [] m_cursorStore; m_cursorStore = NULL;
    delete [] m_cursorXor; m_cursorXor = NULL;
    delete [] m_cursorAnd; m_cursorAnd = NULL;

//	lpc32xx_hw_deinit();
}

//------------------------------------------------------------------------------
//
//  Method:  NumModes
//
//  This method returns number of modes supported by display.
//
int LPC32XXLCD::NumModes()
{
    DEBUGMSG(ZONE_FUNCTION, (L"+LPC32XXLCD::NumModes()\r\n"));
    return 1;
}

//------------------------------------------------------------------------------
//
//  Method:  GetModeInfo
//
//  This method returns mode information for mode number of modes supported
//  by display.
//
SCODE LPC32XXLCD::GetModeInfo(GPEMode* pMode, int modeNumber)
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("++LPC32XXLCD::GetModeInfo\r\n")));
    
    if (modeNumber != 0)
    {
        return E_INVALIDARG;
    }
    
    *pMode = m_gpeMode;
    
    DEBUGMSG(ZONE_FUNCTION, (TEXT("--LPC32XXLCD::GetModeInfo\r\n")));
    
    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  SetMode
//
//  This method should set display hardware to given mode.
//
SCODE LPC32XXLCD::SetMode(int modeNumber, HPALETTE *pPalette)
{
    DEBUGMSG(ZONE_FUNCTION, (TEXT("++LPC32XXLCD::SetMode\r\n")));

    if (modeNumber != 0)
    {
        DEBUGMSG(ZONE_WARN, (TEXT("LPC32XXLCD::SetMode Want mode %d, only have mode 0\r\n"),
			modeNumber));
        return  E_INVALIDARG;
    }

    if (pPalette)
    {
		RETAILMSG(1, (TEXT("set display hardware to give mode\n\r")));

        *pPalette = EngCreatePalette (PAL_BITFIELDS, 0, NULL, 
            LCDPanelList[PanelType].m_BitMasks[0], LCDPanelList[PanelType].m_BitMasks[1],
			LCDPanelList[PanelType].m_BitMasks[2]);
    }

    DEBUGMSG(ZONE_FUNCTION, (TEXT("--LPC32XXLCD::SetMode\r\n")));
    
    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  AllocSurface
//
//  This method executes when the driver must allocate storage for surface
//  pixels. In our case there isn't video memory which can be used for this
//  purpose. In case that video memory is required we should fail call
//  otherwise normal memory chunk will be allocated.
//
SCODE LPC32XXLCD::AllocSurface(
    GPESurf **ppSurf, int width, int height, EGPEFormat format, int flags
) {
    SCODE sc = S_OK;

    // There isn't extra video memory, so fail if it is required
    if ((flags & GPE_REQUIRE_VIDEO_MEMORY) != 0) {
        DEBUGMSG(ZONE_ERROR, (L"LPC32XXLCD::AllocSurface: "
            L"Flat display driver can't allocate extra video memory\r\n"
        ));
        *ppSurf = NULL;
        sc = E_OUTOFMEMORY;
        goto cleanUp;
    }

    // Allocate surface and check result
    *ppSurf = new GPESurf(width, height, format);
    if (*ppSurf == NULL || (*ppSurf)->Buffer() == NULL) {
        DEBUGMSG(ZONE_ERROR, (L"LPC32XXLCD::AllocSurface: "
            L"Failed allocate surface (width: %d, height: %d, format %d\r\n",
            width, height, format
        ));
        delete *ppSurf; *ppSurf = NULL;
        sc = E_OUTOFMEMORY;
        goto cleanUp;
    }
    
cleanUp:
    return sc;
}

//------------------------------------------------------------------------------
//
//  Method: WrapperEmulatedLine
//
//  This function is wrapped around emulated line implementation. It must
//  be implemented only if software pointer is used. It switch off/on cursor
//  if line cross it.
//
SCODE LPC32XXLCD::WrappedEmulatedLine (GPELineParms *lineParameters)
{
    SCODE sc;
    RECT bounds;
    int N_plus_1;

    // If cursor is on check for line overlap
    if (m_cursorVisible && !m_cursorDisabled) {

        // Calculate the bounding-rect to determine overlap with cursor
        if (lineParameters->dN) {
            // The line has a diagonal component
            N_plus_1 = 2 + (
                (lineParameters->cPels * lineParameters->dN)/lineParameters->dM
            );
        } else {
            N_plus_1 = 1;
        }

        switch (lineParameters->iDir) {
        case 0:
            bounds.left = lineParameters->xStart;
            bounds.top = lineParameters->yStart;
            bounds.right = lineParameters->xStart + lineParameters->cPels + 1;
            bounds.bottom = bounds.top + N_plus_1;
            break;
        case 1:
            bounds.left = lineParameters->xStart;
            bounds.top = lineParameters->yStart;
            bounds.bottom = lineParameters->yStart + lineParameters->cPels + 1;
            bounds.right = bounds.left + N_plus_1;
            break;
        case 2:
            bounds.right = lineParameters->xStart + 1;
            bounds.top = lineParameters->yStart;
            bounds.bottom = lineParameters->yStart + lineParameters->cPels + 1;
            bounds.left = bounds.right - N_plus_1;
            break;
        case 3:
            bounds.right = lineParameters->xStart + 1;
            bounds.top = lineParameters->yStart;
            bounds.left = lineParameters->xStart - lineParameters->cPels;
            bounds.bottom = bounds.top + N_plus_1;
            break;
        case 4:
            bounds.right = lineParameters->xStart + 1;
            bounds.bottom = lineParameters->yStart + 1;
            bounds.left = lineParameters->xStart - lineParameters->cPels;
            bounds.top = bounds.bottom - N_plus_1;
            break;
        case 5:
            bounds.right = lineParameters->xStart + 1;
            bounds.bottom = lineParameters->yStart + 1;
            bounds.top = lineParameters->yStart - lineParameters->cPels;
            bounds.left = bounds.right - N_plus_1;
            break;
        case 6:
            bounds.left = lineParameters->xStart;
            bounds.bottom = lineParameters->yStart + 1;
            bounds.top = lineParameters->yStart - lineParameters->cPels;
            bounds.right = bounds.left + N_plus_1;
            break;
        case 7:
            bounds.left = lineParameters->xStart;
            bounds.bottom = lineParameters->yStart + 1;
            bounds.right = lineParameters->xStart + lineParameters->cPels + 1;
            bounds.top = bounds.bottom - N_plus_1;
            break;
        default:
            DEBUGMSG(ZONE_WARN, (L"LPC32XXLCD::WrappedEmulatedLine: "
                L"Invalid direction: %d\r\n", lineParameters->iDir));
            sc = E_INVALIDARG;
            goto cleanUp;
        }

        // If line overlap cursor, turn if off
        RECTL cursorRect = m_cursorRect;
        RotateRectl (&cursorRect);

        if (cursorRect.top < bounds.bottom && 
            cursorRect.bottom > bounds.top &&
            cursorRect.left < bounds.right && 
            cursorRect.right > bounds.left
        ) { 
            CursorOff();
            m_cursorForcedOff = TRUE;
        }            
    }

    // Do emulated line
    sc = EmulatedLine(lineParameters);

    // If cursor was forced off turn it back on
    if (m_cursorForcedOff) {
        m_cursorForcedOff = FALSE;
        CursorOn();
    }

cleanUp:
    return sc;
}

//------------------------------------------------------------------------------
//
//  Method: Line
//
//  This method executes before and after a sequence of line segments,
//  which are drawn as a path. It examines the line parameters to determine
//  whether the operation can be accelerated. It also places a pointer to
//  a function to execute once per line segment into the pLine member
//  of the GPELineParms structure.
//
SCODE LPC32XXLCD::Line(GPELineParms *pLineParms, EGPEPhase phase)
{
    if (phase == gpeSingle || phase == gpePrepare) {
        if ((pLineParms->pDst != m_pPrimarySurface)) {
            pLineParms->pLine = &GPE::EmulatedLine;
        } else {
            pLineParms->pLine = 
				(SCODE (GPE::*)(struct GPELineParms *))&LPC32XXLCD::WrappedEmulatedLine;
        }            
    }
    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  BltPrepare
//
//  This method identifies the appropriate functions needed to perform
//  individual blits. This function executes before a sequence of clipped blit
//  operations.
//
SCODE LPC32XXLCD::BltPrepare(GPEBltParms *pBltParms)
{
    RECTL rect;
    LONG swapTmp;

	pBltParms->pBlt = &LPC32XXLCD::EmulatedBlt;

    // Check if destination overlap with cursor
    if (
        pBltParms->pDst == m_pPrimarySurface &&
        m_cursorVisible && !m_cursorDisabled
    ) { 
        if (pBltParms->prclDst != NULL) {
            rect = *pBltParms->prclDst;     // if so, use it

            // There is no guarantee of a well
            // ordered rect in blitParamters
            // due to flipping and mirroring.
            if (rect.top > rect.bottom) {
                swapTmp = rect.top;
                rect.top = rect.bottom;
                rect.bottom = swapTmp;
            }
            if (rect.left > rect.right) {
                swapTmp    = rect.left;
                rect.left  = rect.right;
                rect.right = swapTmp;
            }
        } else {
            rect = m_cursorRect;
        }

        // Turn off cursor if it overlap
        if (
            m_cursorRect.top <= rect.bottom &&
            m_cursorRect.bottom >= rect.top &&
            m_cursorRect.left <= rect.right &&
            m_cursorRect.right >= rect.left
        ) {
            CursorOff();
            m_cursorForcedOff = TRUE;
        }
    }

    // Check if source overlap with cursor
    if (
        pBltParms->pSrc == m_pPrimarySurface &&
        m_cursorVisible && !m_cursorDisabled
    ) {
        if (pBltParms->prclSrc != NULL) {
            rect = *pBltParms->prclSrc;
        } else {
            rect = m_cursorRect;
        }
        if (
            m_cursorRect.top < rect.bottom && m_cursorRect.bottom > rect.top &&
            m_cursorRect.left < rect.right && m_cursorRect.right > rect.left
        ) {
            CursorOff();
            m_cursorForcedOff = TRUE;
        }
    }

    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  BltComplete
//
//  This method executes to complete a blit sequence. 
//
SCODE LPC32XXLCD::BltComplete(GPEBltParms *pBtlParms)
{
    // If cursor was forced off turn it back on
    if (m_cursorForcedOff) {
        m_cursorForcedOff = FALSE;
        CursorOn();
    }
    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  CursorOn
//
VOID LPC32XXLCD::CursorOn()
{
    UCHAR *pFrame;
    UCHAR *pFrameLine, *pStoreLine, *pXorLine, *pAndLine, data;
    int bytesPerPixel, bytesPerLine;
    int xf, yf, xc, yc, i;

    DEBUGMSG(ZONE_FUNCTION, (L"+LPC32XXLCD::CursonOn\r\n"));

    // If cursor should not be visible or already is then exit
    if (m_cursorForcedOff || m_cursorDisabled || m_cursorVisible) goto cleanUp;

    if (m_cursorStore == NULL) {
        DEBUGMSG(ZONE_WARN, (L"LPC32XXLCD::CursorOn: "
            L"No cursor store available\r\n"
        ));
        goto cleanUp;
    }

    // We support only 1,2,3 and 4 bytes per pixel
    bytesPerPixel = (m_gpeMode.Bpp + 7) >> 3;
    if (bytesPerPixel <= 0 || bytesPerPixel > 4) goto cleanUp;
    
    // Get some base metrics
    pFrame = (UCHAR*)m_pPrimarySurface->Buffer();
    bytesPerLine = m_pPrimarySurface->Stride();

    for (yf = m_cursorRect.top, yc = 0; yf < m_cursorRect.bottom; yf++, yc++) {
        // Check if we are done
        if (yf < 0) continue;
        if (yf >= m_gpeMode.height) break;
    
        pFrameLine = &pFrame[yf * bytesPerLine];
        pStoreLine = &m_cursorStore[yc * m_cursorSize.x * bytesPerPixel];
        pAndLine = &m_cursorAnd[yc * m_cursorSize.x * bytesPerPixel];
        pXorLine = &m_cursorXor[yc * m_cursorSize.x * bytesPerPixel];
    
        for (
            xf = m_cursorRect.left, xc = 0; xf < m_cursorRect.right; xf++, xc++
        ) {
            // Check if we are done
            if (xf < 0) continue;
            if (xf >= m_gpeMode.width) break;

            // Depending on bytes per pixel
            switch (bytesPerPixel) {
            case 1:
                pStoreLine[xc] = pFrameLine[xf];
                pFrameLine[xf] &= pAndLine[xc];
                pFrameLine[xf] ^= pXorLine[xc];
                break;
            case 2:
                ((USHORT*)pStoreLine)[xc]  = ((USHORT*)pFrameLine)[xf];
                ((USHORT*)pFrameLine)[xf] &= ((USHORT*)pAndLine)[xc];
                ((USHORT*)pFrameLine)[xf] ^= ((USHORT*)pXorLine)[xc];
                break;
            case 3:
                for (i = 0; i < bytesPerPixel; i++) {
                    data = pFrameLine[xf * bytesPerPixel + i];
                    pStoreLine[xc * bytesPerPixel + i] = data;
                    data &= pAndLine[xc * bytesPerPixel + i];
                    data ^= pXorLine[xc * bytesPerPixel + i];
                    pFrameLine[xf * bytesPerPixel + i] = data;
                }                    
                break;
            case 4:
                ((ULONG*)pStoreLine)[xc]  = ((ULONG*)pFrameLine)[xf];
                ((ULONG*)pFrameLine)[xf] &= ((ULONG*)pAndLine)[xc];
                ((ULONG*)pFrameLine)[xf] ^= ((ULONG*)pXorLine)[xc];
                break;
            }                    
        }
    }
    
    // Cursor is visible now
    m_cursorVisible = TRUE;

cleanUp: 
    DEBUGMSG(ZONE_FUNCTION, (L"-LPC32XXLCD::CursonOn\r\n"));
    return;
}

//------------------------------------------------------------------------------

VOID LPC32XXLCD::CursorOff()
{
    UCHAR *pFrame, *pFrameLine, *pStoreLine, data;
    int bytesPerPixel, bytesPerLine;
    int xf, yf, xc, yc, i;

    DEBUGMSG(ZONE_FUNCTION, (L"+LPC32XXLCD::CursonOff\r\n"));

    if (m_cursorForcedOff || m_cursorDisabled || !m_cursorVisible) goto cleanUp;

    if (m_cursorStore == NULL) {
        DEBUGMSG(ZONE_WARN, (L"LPC32XXLCD::CursorOff: "
            L"No cursor store available\r\n"
        ));
        goto cleanUp;
    }

    // We support only 1,2,3 and 4 bytes per pixel
    bytesPerPixel = (m_gpeMode.Bpp + 7) >> 3;
    if (bytesPerPixel <= 0 || bytesPerPixel > 4) goto cleanUp;

    // Get some base metrics
    pFrame = (UCHAR*)m_pPrimarySurface->Buffer();
    bytesPerLine = m_pPrimarySurface->Stride();

    for (yf = m_cursorRect.top, yc = 0; yf < m_cursorRect.bottom; yf++, yc++) {
        // Check if we are done
        if (yf < 0) continue;
        if (yf >= m_gpeMode.height) break;

        pFrameLine = &pFrame[yf * bytesPerLine];
        pStoreLine = &m_cursorStore[yc * m_cursorSize.x * bytesPerPixel];
    
        for (
            xf = m_cursorRect.left, xc = 0; xf < m_cursorRect.right; xf++, xc++
        ) {
            // Check if we are done
            if (xf < 0) continue;
            if (xf >= m_gpeMode.width) break;
    
            // Depending on bytes per pixel
            switch (bytesPerPixel) {
            case 1:
                pFrameLine[xf] = pStoreLine[xc];
                break;
            case 2:
                ((USHORT*)pFrameLine)[xf] = ((USHORT*)pStoreLine)[xc];
                break;
            case 3:
                for (i = 0; i < bytesPerPixel; i++) {
                    data = pStoreLine[xc * bytesPerPixel + i];
                    pFrameLine[xf * bytesPerPixel + i] = data;
                }                    
                break;
            case 4:
                ((ULONG*)pFrameLine)[xf] = ((ULONG*)pStoreLine)[xc];
                break;
            }                    
        }
    }

    // Cursor isn't visible now
    m_cursorVisible = FALSE;

cleanUp: 
    DEBUGMSG(ZONE_FUNCTION, (L"-LPC32XXLCD::CursonOff\r\n"));
    return;
}

//------------------------------------------------------------------------------
//
//  Method:  SetPointerShape
//
//  This method sets the shape of the pointer, the hot spot of the pointer,
//  and the colors to use for the cursor if the cursor is multicolored.
//
SCODE LPC32XXLCD::SetPointerShape(
    GPESurf* pMask, GPESurf* pColorSurf, int xHotspot, int yHotspot, 
    int xSize, int ySize 
) {
    SCODE sc = S_OK;
    UCHAR *pAndPtr, *pXorPtr, *pAndLine, *pXorLine;
    UCHAR andPtr, xorPtr, mask;
    ULONG size;
    int bytesPerPixel;
    int row, col, i;


    DEBUGMSG(ZONE_INFO, (
        L"+LPC32XXLCD::SetPointerShape(0x%08x, 0x%08x, %d, %d, %d, %d)\r\n",
        pMask, pColorSurf, xHotspot, yHotspot, xSize, ySize
    ));

    // Turn current cursor off
    CursorOff();

    // Release memory associated with old cursor
    delete [] m_cursorStore; m_cursorStore = NULL;
    delete [] m_cursorXor; m_cursorXor = NULL;
    delete [] m_cursorAnd; m_cursorAnd = NULL;

    // Is there a new mask?
    if (pMask == NULL) {

        // No, so tag as disabled
        m_cursorDisabled = TRUE;

    } else {

        // Yes, so tag as not disabled
        m_cursorDisabled = FALSE;

        // Check if cursor size is correct
        if (xSize > m_nScreenWidth || ySize > m_nScreenHeight) {
            DEBUGMSG(ZONE_ERROR, (L"LPC32XXLCD::SetPointerShape: "
                L"Invalid cursor size %d, %d\r\n", xSize, ySize
            ));
            sc = E_FAIL;
            goto cleanUp;
        }
        
        // How many bytes we need per pixel on screen
        bytesPerPixel = (m_gpeMode.Bpp + 7) >> 3;

        // Cursor mask & store size
        size = xSize * ySize * bytesPerPixel;
        
        // Allocate memory based on new cursor size
        m_cursorStore = new UCHAR[size];
        m_cursorXor   = new UCHAR[size];
        m_cursorAnd   = new UCHAR[size];

        if (
            m_cursorStore == NULL || m_cursorXor == NULL || m_cursorAnd == NULL
        ) {
            DEBUGMSG(ZONE_ERROR, (L"LPC32XXLCD::SetPointerShape: "
                L"Memory allocation for cursor buffers failed\r\n"
            ));
            sc = E_OUTOFMEMORY;
            goto cleanUp;
        }

        // Store size and hotspot for new cursor
        m_cursorSize.x = xSize;
        m_cursorSize.y = ySize;
        m_cursorHotspot.x = xHotspot;
        m_cursorHotspot.y = yHotspot;

        // Pointers to AND and XOR masks
        pAndPtr = (UCHAR*)pMask->Buffer();
        pXorPtr = (UCHAR*)pMask->Buffer() + (ySize * pMask->Stride());

        // store OR and AND mask for new cursor
        for (row = 0; row < ySize; row++) {
            pAndLine = &m_cursorAnd[row * xSize * bytesPerPixel];
            pXorLine = &m_cursorXor[row * xSize * bytesPerPixel];
            for (col = 0; col < xSize; col++) {
                andPtr = pAndPtr[row * pMask->Stride() + (col >> 3)];
                xorPtr = pXorPtr[row * pMask->Stride() + (col >> 3)];
                mask = 0x80 >> (col & 0x7);
                for (i = 0; i < bytesPerPixel; i++) {
                    pAndLine[col * bytesPerPixel + i] = andPtr&mask?0xFF:0x00;
                    pXorLine[col * bytesPerPixel + i] = xorPtr&mask?0xFF:0x00;
                }                    
            }
        }
        
    }

cleanUp:
    DEBUGMSG(ZONE_FUNCTION, (L"-LPC32XXLCD::SetPointerShape(sc = 0x%08x)\r\n", sc));
    return sc;
}

//------------------------------------------------------------------------------
//
//  Method:  MovePointer
//
//  This method executes from applications either to move the hot spot
//  of the cursor to a specific screen location or to hide the cursor (x == -1).
//
SCODE LPC32XXLCD::MovePointer(int x, int y)
{
    DEBUGMSG(ZONE_FUNCTION, (L"+LPC32XXLCD::MovePointer(%d, %d)\r\n", x, y));

    CursorOff();

    if (x != -1 || y != -1) {
        // Compute new cursor rect
        m_cursorRect.left = x - m_cursorHotspot.x;
        m_cursorRect.right = m_cursorRect.left + m_cursorSize.x;
        m_cursorRect.top = y - m_cursorHotspot.y;
        m_cursorRect.bottom = m_cursorRect.top + m_cursorSize.y;
        CursorOn();
    }

    DEBUGMSG(ZONE_FUNCTION, (L"-LPC32XXLCD::MovePointer(sc = 0x%08x)\r\n", S_OK));
    return  S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  InVBlank
//
int LPC32XXLCD::InVBlank()
{
    return 1;
}

//------------------------------------------------------------------------------
//
//  Method:  SetPalette
//
SCODE LPC32XXLCD::SetPalette(
    const PALETTEENTRY *pSource, WORD firstEntry, WORD numEntries
) {
    DEBUGMSG(ZONE_FUNCTION, (
        L"+LPC32XXLCD::SetPalette(0x%08x, %d, %d)\r\n", pSource, firstEntry,
        numEntries));

    return S_OK;
}

//------------------------------------------------------------------------------
//
//  Method:  PowerHandler
//
VOID LPC32XXLCD::PowerHandler(BOOL off)
{
    ULONG commands[2];
    
    // Ask HAL to set power
    commands[0] = DDHAL_COMMAND_POWER;
    commands[1] = off;
    if (!KernelIoControl(
        IOCTL_HAL_DDI, commands, sizeof(commands), NULL, 0, NULL
    )) {
        DEBUGMSG(ZONE_WARN, (L"LPC32XXLCD::PowerHandler: "
            L"IOCTL_HAL!DDI_POWER failed for off flag %d\r\n", off
        ));
    }
}

//------------------------------------------------------------------------------

void LPC32XXLCD::WaitForNotBusy()
{
    return;
}

int LPC32XXLCD::IsBusy()
{
    return 0;    // Never busy as there is no acceleration
}

void LPC32XXLCD::GetPhysicalVideoMemory(
    unsigned long * pPhysicalMemoryBase,
    unsigned long * pVideoMemorySize
    )
{
    *pPhysicalMemoryBase = (ULONG)m_VirtualFrameBuffer;
    *pVideoMemorySize    = m_FrameBufferSize;
}

void LPC32XXLCD::GetVirtualVideoMemory(
    unsigned long * pVirtualMemoryBase,
    unsigned long * pVideoMemorySize
    )
{
    *pVirtualMemoryBase = (unsigned)(m_pPrimarySurface->Buffer());
    *pVideoMemorySize   = m_FrameBufferSize;
}
