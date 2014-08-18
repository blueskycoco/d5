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
// kb_phy3250_map.h
//
// Matrix to key mapping functions
//

#include <windows.h>
#include <keybddr.h>
#include <laymgr.h>
#include "devicelayout.h"
#include "lpc32xx_kb.h"

// Scan codes from the PDD are converted to key functions as
// defined in the following list
#define ScanCodeTableFirst  0
#define ScanCodeTableLast   63
static UINT8 ScanCodeToVKeyTable[ScanCodeTableLast + 1] = {
	// Col 0
	'1',
	'2',
	'3',
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 2
	'4',
	'5',
	'6',
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 3
	'7',
	'8',
	'9',
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 4
	'*',
	'0',
	'#',
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 5
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 6
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,

	// Col 7
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF,
	0xFF
};

static ScanCodeToVKeyData scvkEngUS = 
{
    0,
    ScanCodeTableFirst,
    ScanCodeTableLast,
    ScanCodeToVKeyTable
};

static ScanCodeToVKeyData *FFEngUSTables[] = {&scvkEngUS};

//------------------------------------------------------------------------------
//
// FFEngUsRemapVKey
//
// Remap function - handles remapping of keys from raw to key code
//
static UINT WINAPI FFEngUsRemapVKey(
    const KEYBD_EVENT *pKbdEvents,
    UINT               cKbdEvents,
    KEYBD_EVENT       *pRmpKbdEvents,
    UINT               cMaxRmpKbdEvents
    )
{
    UINT ui, uiCurrent = 0;

    if (pRmpKbdEvents == NULL) {
        DEBUGCHK(cMaxRmpKbdEvents == 0);
        return cKbdEvents;
    }
    
    DEBUGCHK(pKbdEvents != NULL);

    if (cMaxRmpKbdEvents < cKbdEvents) {
		/* Limit processed events to size of output buffer */
		cKbdEvents = cMaxRmpKbdEvents;
    }

	// Remap each key
    for (ui = 0; ui < cKbdEvents; ++ui) {
		if (pKbdEvents[ui].uiVk <= ScanCodeTableLast) {
			// Just copy codes
			pRmpKbdEvents[uiCurrent].uiSc = pKbdEvents[ui].uiSc;
			pRmpKbdEvents[uiCurrent].uiVk = pKbdEvents[ui].uiVk;
			pRmpKbdEvents[uiCurrent].KeyStateFlags = pKbdEvents[ui].KeyStateFlags;
			uiCurrent++;
		}
	}

	return uiCurrent;
}

//------------------------------------------------------------------------------
// dlATEngUs
//
// Current keyboard layout
//
static DEVICE_LAYOUT dlATEngUs =
{
    sizeof(DEVICE_LAYOUT),
    MATRIX_PDD,
    FFEngUSTables,
    dim(FFEngUSTables),
    FFEngUsRemapVKey,
};

//------------------------------------------------------------------------------
// Matrix
//
// Matrix keypad entry code
//
extern "C" BOOL Matrix(PDEVICE_LAYOUT pDeviceLayout)
{
    PREFAST_ASSERT(pDeviceLayout != NULL);
    
	DEBUGMSG(1, (TEXT("Matrix\r\n")));

    if (pDeviceLayout->dwSize != sizeof(DEVICE_LAYOUT)) {
        DEBUGMSG(1, (_T("Matrix: data structure size mismatch\r\n")));
        return FALSE;
    }

    *pDeviceLayout = dlATEngUs;

	return TRUE;
}
