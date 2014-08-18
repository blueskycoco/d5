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
//------------------------------------------------------------------------------
//
// menu.c
//

#include <eboot.h>
#include "boot_utilities.h"

extern BOOT_CFG g_bootCfg;
extern EBOOT_CONTEXT g_eboot;
extern OAL_KITL_DEVICE g_bootDevices[];
extern OAL_KITL_DEVICE g_kitlDevices[];

//------------------------------------------------------------------------------
//
//  Define:  dimof
//
#ifdef dimof
#undef dimof
#endif
#define dimof(x)                (sizeof(x)/sizeof(x[0]))

//------------------------------------------------------------------------------

static VOID ChangeBootTo(OAL_BLMENU_ITEM *pMenu);
static VOID ShowSettings(OAL_BLMENU_ITEM *pMenu);
static VOID ShowNetworkSettings(OAL_BLMENU_ITEM *pMenu);
static VOID SelectBootDevice(OAL_BLMENU_ITEM *pMenu);
static VOID SetKitlMode(OAL_BLMENU_ITEM *pMenu);
static VOID SaveSettings(OAL_BLMENU_ITEM *pMenu);
static VOID ChangeBaud(OAL_BLMENU_ITEM *pMenu);
static VOID frcclnboot(OAL_BLMENU_ITEM *pMenu);
extern BOOL cleanboot;

//------------------------------------------------------------------------------

static OAL_BLMENU_ITEM g_menuNetwork[] = {
    {
        L'1', L"Show Current Settings", ShowNetworkSettings,
        NULL, NULL, NULL
    }, {
        L'2', L"Enable/disable KITL", OALBLMenuEnable,
        L"KITL", &g_bootCfg.kitlFlags, (VOID*)OAL_KITL_FLAGS_ENABLED
    }, {
        L'3', L"KITL interrupt/poll mode", SetKitlMode,
        NULL, NULL, NULL
    }, {
        L'4', L"Enable/disable DHCP", OALBLMenuEnable,
        L"DHCP", &g_bootCfg.kitlFlags, (VOID*)OAL_KITL_FLAGS_DHCP
    }, {
        L'5', L"Set IP address", OALBLMenuSetIpAddress,
        L"Device", &g_bootCfg.ipAddress, NULL
    }, {
        L'6', L"Set IP mask", OALBLMenuSetIpMask,
        L"Device", &g_bootCfg.ipMask, NULL
    }, {
        L'7', L"Set default router", OALBLMenuSetIpAddress,
        L"Default router", &g_bootCfg.ipRoute, NULL
    }, {
        L'8', L"Enable/disable VMINI", OALBLMenuEnable,
        L"VMINI", &g_bootCfg.kitlFlags, (VOID*)OAL_KITL_FLAGS_VMINI
    }, {
        L'0', L"Exit and Continue", NULL,
        NULL, NULL, NULL
    }, {
        0, NULL, NULL,
        NULL, NULL, NULL
    }
};

static OAL_BLMENU_ITEM g_menuMain[] = {
    {
        L'1', L"Change boot timeout", ChangeBootTo,
        NULL, NULL, NULL
    }, {
        L'2', L"Set baud rate", ChangeBaud,
        NULL, NULL, NULL
    }, {
        L'3', L"Show Current Settings", ShowSettings,
        NULL, NULL, NULL
    }, {
        L'4', L"Select Boot Device", SelectBootDevice,
        L"Boot", &g_bootCfg.bootDevLoc, g_bootDevices
    }, {
        L'5', L"Select Debug Device", OALBLMenuSelectDevice,
        L"Debug", &g_bootCfg.kitlDevLoc, g_kitlDevices
    }, {
        L'6', L"Network Settings", OALBLMenuShow,
        L"Network Settings", &g_menuNetwork, NULL
    }, {
        L'7', L"Force clean boot", frcclnboot,
        NULL, NULL, NULL
    }, {
		L'9', L"Save Settings", SaveSettings,
        NULL, NULL, NULL
    }, {
        L'0', L"Exit and Continue", NULL,
        NULL, NULL, NULL
    }, {
        0, NULL, NULL,
        NULL, NULL, NULL
    }
};

static OAL_BLMENU_ITEM g_menuRoot = {
    0, NULL, OALBLMenuShow,
    L"NXP LPC32XX Main Menu", g_menuMain, NULL
};

//------------------------------------------------------------------------------

VOID BLMenu()
{
    UINT32 ud, delay;
    WCHAR key = 0;

    // First let user break to menu
    OALLog(L"Hit space to enter LPC32XX bootloader menu.\n\r");
	delay = OALGetTickCount() + (g_bootCfg.boot_to * 1000);
	ud = OALGetTickCount() - 1;
    while (delay > OALGetTickCount())
	{
		if (ud <= OALGetTickCount()) {
			OALLog(L".");
			ud = OALGetTickCount() + 1000;
		}

		key = OALBLMenuReadKey(FALSE);
		if (key == L' ') break;
	}
	OALLog(L"\r\n");
    
    if (key == L' ') {
        OALBLMenuShow(&g_menuRoot);
    }
}

//------------------------------------------------------------------------------
VOID ChangeBaud(OAL_BLMENU_ITEM *pMenu)
{
    WCHAR key;
	BOOL bu = FALSE;

	while (bu == FALSE)
	{
		OALLog(L" 1: 115.2K\r\n");
		OALLog(L" 2: 57600\r\n");
		OALLog(L" 3: 38400\r\n");
		OALLog(L" 4: 19200\r\n");
		OALLog(L" 5: 9600\r\n");
		OALLog(L"Select a new baud rate: ");
	    key = OALBLMenuReadKey(TRUE);
		OALLog(L"\r\n");

		switch (key)
		{
			case '1':
				g_bootCfg.baudRate = 115200;
				bu = TRUE;
				break;

			case '2':
				g_bootCfg.baudRate = 57600;
				bu = TRUE;
				break;

			case '3':
				g_bootCfg.baudRate = 38400;
				bu = TRUE;
				break;

			case '4':
				g_bootCfg.baudRate = 19200;
				bu = TRUE;
				break;

			case '5':
				g_bootCfg.baudRate = 9600;
				bu = TRUE;
				break;

			default:
				break;
		}
	}

	sport_update_rate(g_bootCfg.baudRate);
}

//------------------------------------------------------------------------------
static VOID frcclnboot(OAL_BLMENU_ITEM *pMenu)
{
	OALLog(L"Next WinCE boot cycle will use a clean boot\r\n");
	cleanboot = TRUE;
}

//------------------------------------------------------------------------------

VOID ChangeBootTo(OAL_BLMENU_ITEM *pMenu)
{
    WCHAR buf[MAX_PATH];

	OALLog(L"Enter new boot timeout (Seconds): ");
    if(OALBLMenuReadLine(buf, dimof(buf))){
		g_bootCfg.boot_to = decStrToVal(buf);
	}
}

//------------------------------------------------------------------------------

VOID ShowSettings(OAL_BLMENU_ITEM *pMenu)
{
    OALLog(L"\r\n Main:\r\n");

	if(g_bootCfg.bootDevLoc.LogicalLoc == SD_BASE){
        OALLog(
            L"  Boot device:   %s(%S)\r\n",
            OALKitlDeviceName(&g_bootCfg.bootDevLoc, g_bootDevices), g_bootCfg.binName
            );
    }
    else{
        OALLog(
            L"  Boot device:   %s\r\n",
            OALKitlDeviceName(&g_bootCfg.bootDevLoc, g_bootDevices)
            );
    }
    OALLog(
        L"  Debug device:  %s\r\n",
        OALKitlDeviceName(&g_bootCfg.kitlDevLoc, g_kitlDevices)
    );

    OALLog(L"  Boot timeout:  %d seconds\r\n", g_bootCfg.boot_to);

	ShowNetworkSettings(pMenu);
}

//------------------------------------------------------------------------------

VOID ShowNetworkSettings(OAL_BLMENU_ITEM *pMenu)
{
    OALLog(L"\r\n Network:\r\n");
    OALLog(
        L"  KITL state:    %s\r\n",
        (g_bootCfg.kitlFlags & OAL_KITL_FLAGS_ENABLED) ? L"enabled" : L"disabled"
    );
    OALLog(
        L"  KITL mode:     %s\r\n",
        (g_bootCfg.kitlFlags & OAL_KITL_FLAGS_POLL) ? L"poll" : L"interrupt"
    );
    OALLog(
        L"  DHCP:          %s\r\n",
        (g_bootCfg.kitlFlags & OAL_KITL_FLAGS_DHCP) ? L"enabled" : L"disabled"
    );
    OALLog(L"  IP address:    %s\r\n", OALKitlIPtoString(g_bootCfg.ipAddress));
    OALLog(L"  IP mask:       %s\r\n", OALKitlIPtoString(g_bootCfg.ipMask));
    OALLog(L"  IP router:     %s\r\n", OALKitlIPtoString(g_bootCfg.ipRoute));
    OALLog(
        L"  VMINI:         %s\r\n",
        (g_bootCfg.kitlFlags & OAL_KITL_FLAGS_VMINI) ? L"enabled" : L"disabled"
    );
}

//------------------------------------------------------------------------------

VOID SelectBootDevice(OAL_BLMENU_ITEM *pMenu){
    int i;
    WCHAR buf[MAX_PATH];

    OALBLMenuSelectDevice(pMenu);

	if(g_bootCfg.bootDevLoc.LogicalLoc == SD_BASE){
        // let user select image name
        OALLog(L"Enter image name(actual '%S'): ", g_bootCfg.binName);
        if(OALBLMenuReadLine(buf, dimof(buf))){
            for(i = 0; buf[i] != 0; ++i){
                g_bootCfg.binName[i] = (CHAR)buf[i];
            }
        }
    }
}

//------------------------------------------------------------------------------

VOID SetKitlMode(OAL_BLMENU_ITEM *pMenu)
{
    WCHAR key;

    if ((g_bootCfg.kitlFlags & OAL_KITL_FLAGS_POLL) != 0) {
        OALLog(L" Set KITL to interrupt mode [y/-]: ");
    } else {
        OALLog(L" Set KITL to poll mode [y/-]: ");
    }    

    // Get key
    key = OALBLMenuReadKey(TRUE);
    OALLog(L"%c\r\n", key);

    if (key == L'y' || key == L'Y') {
        if ((g_bootCfg.kitlFlags & OAL_KITL_FLAGS_POLL) != 0) {
            g_bootCfg.kitlFlags &= ~OAL_KITL_FLAGS_POLL;
            OALLog(L" KITL set to interrupt mode\r\n");
        } else {
            g_bootCfg.kitlFlags |= OAL_KITL_FLAGS_POLL;
            OALLog(L" KITL set to poll mode\r\n");
        }            
    }
}

//------------------------------------------------------------------------------

VOID SaveSettings(OAL_BLMENU_ITEM *pMenu)
{
    WCHAR key;

    OALLog(L" Do you want save current settings [-/y]? ");

    // Get key
    key = OALBLMenuReadKey(TRUE);
    OALLog(L"%c\r\n", key);

    // Depending on result
    if (key != L'y' && key != L'Y') goto cleanUp;

    if (BLWriteBootCfg(&g_bootCfg)) 
	{
        OALLog(L" Current settings has been saved\r\n");
    }
	else 
	{        
        OALLog(L"ERROR: Settings save failed!\r\n");
    }

cleanUp:
    return;
}

//------------------------------------------------------------------------------


