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
// kb_pddlist.cpp
//
// Keyboard PDD function entry list
//

#include <windows.h>
#include <keybdpdd.h>

// Add NOP driver for USB HID and RDP support
BOOL WINAPI PS2_NOP_Entry(UINT uiPddId,
						  PFN_KEYBD_EVENT pfnKeybdEvent,
						  PKEYBD_PDD *ppKeybdPdd);

// Built-in LH7A404 matrix keypad driver
BOOL WINAPI KB_LPC32XX_Entry(UINT uiPddId,
							 PFN_KEYBD_EVENT pfnKeybdEvent,
							 PKEYBD_PDD *ppKeybdPdd);

// PDD list
PFN_KEYBD_PDD_ENTRY g_rgpfnPddEntries[] = {
    PS2_NOP_Entry,
    KB_LPC32XX_Entry,
    NULL
};
