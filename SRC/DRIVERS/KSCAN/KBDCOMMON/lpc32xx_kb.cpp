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
// lpc32xx_kb.cpp
//
// PDD for the LPC32xx matrix keyboard driver
//

#include "lpc32xx_kb.h"
#include <ceddk.h>
#include <nkintr.h>
#include <keybddbg.h>
#include <keybddr.h>
#include <keybdpdd.h>
#include <laymgr.h>
#include <keybdist.h>
#include "lpc32xx_kscan.h"
#include "lpc32xx_clkpwr.h"
#include "intr.h"

// Structure used for keyboard controller driver data
typedef struct {
	KSCAN_REGS_T *pKSCANRegs;        // Pointer to KSCAN registers
	CLKPWR_REGS_T *pCLKPWRRegs;      // Pointer to CLKPWR registers
	UINT8 keyStates [MATRIXSIZE];    // Last KSCAN states
	DWORD sysIntr;                   // Keyboard SysIntr value
	HANDLE hEvent;                   // Keyboard interrupt event flag
	HANDLE hThread;                  // Keyboard thread handle
	UINT v_uiPddId;
	PFN_KEYBD_EVENT v_pfnKeybdEvent;
} KB_DATA_T;
static KB_DATA_T kbdata; 

//------------------------------------------------------------------------------
//
// kscan_enable
//
// Enables or disables KSCAN block
//
void kscan_enable(BOOL enable)
{
	if (enable != FALSE)
	{
		// Enable KSCAN clock and clear IRQ
		kbdata.pCLKPWRRegs->clkpwr_key_clk_ctrl |= CLKPWR_KEYCLKCTRL_CLK_EN;
		kbdata.pKSCANRegs->ks_irq = 1;
	}
	else
	{
		// Clear IRQ and disable KSCAN clock
		kbdata.pKSCANRegs->ks_irq = 1;
		kbdata.pCLKPWRRegs->clkpwr_key_clk_ctrl &= ~CLKPWR_KEYCLKCTRL_CLK_EN;
	}
}

//------------------------------------------------------------------------------
//
// kscan_int_handler
//
// KSCAN interrupt handler, updates KSCAN matrix of a key change
//
void kscan_int_handler(UINT8 newStates[MATRIXSIZE])
{
	int idx = 0;

	// Read new key scan states
	for (idx = 0; idx < MATRIXSIZE; idx++)
	{
		newStates[idx] = (UINT8) kbdata.pKSCANRegs->ks_data [idx];
	}

	// Clear IRQ state
	kbdata.pKSCANRegs->ks_irq = 1;
}

//------------------------------------------------------------------------------
//
// kscanGetEvents
//
// Generates a list of raw KSCAN changes codes and states
// Raw key scan code is in the format 0bcccrrr, ccc=column, rrr=row
//
int kscanGetEvents(UINT8 rawScanCodeList[16],
				   BOOL keyUpList[16])
{
	UINT8 bitm, newEvents[MATRIXSIZE];
	int idx, innl;
	int changes = 0;

	// Get latest events
	kscan_int_handler(newEvents);

	for (idx = 0; ((idx < MATRIXSIZE) && (changes < 16)); idx++)
	{
		if (newEvents[idx] != kbdata.keyStates[idx])
		{
			// Key state has changed, determine changed key
			for (innl = 0; ((innl < MATRIXSIZE) && (changes < 16)); innl++)
			{
				bitm = newEvents[idx] & _BIT(innl);
				if (bitm != (kbdata.keyStates[idx] & _BIT(innl)))
				{
					// Save new key state
					rawScanCodeList[changes] = (UINT8) (idx << 3) |
						(UINT8) (innl);
					if (bitm == 0)
					{
						// Key was released
						keyUpList[changes] = TRUE;
					}
					else
					{
						// Key was pressed
						keyUpList[changes] = FALSE;
					}

					changes++;
				}
			}
		}
	}

	return changes;
}

//------------------------------------------------------------------------------
//
// KB_LPC32XX_GetEventEx2
//
// Process keypad events
//
static UINT KB_LPC32XX_GetEventEx2(UINT uiPddId,
								   UINT32 rguiScanCode[16],
								   BOOL rgfKeyUp[16])
{
	UINT8 bitm, newEvents[MATRIXSIZE];
	int idx, innl;
	int changes = 0;

	// Get latest events
	kscan_int_handler(newEvents);

	for (idx = 0; ((idx < MATRIXSIZE) && (changes < 16)); idx++)
	{
		if (newEvents[idx] != kbdata.keyStates[idx])
		{
			// Key state has changed, determine changed key
			for (innl = 0; ((innl < MATRIXSIZE) && (changes < 16)); innl++)
			{
				bitm = newEvents[idx] & _BIT(innl);
				if (bitm != (kbdata.keyStates[idx] & _BIT(innl)))
				{
					// Save new key state
					rguiScanCode[changes] = (UINT8) (idx << 3) |
						(UINT8) (innl);
					if (bitm == 0)
					{
						// Key was released
						rgfKeyUp[changes] = TRUE;
					}
					else
					{
						// Key was pressed
						rgfKeyUp[changes] = FALSE;
					}

					changes++;
				}
			}

			// Save new state
			kbdata.keyStates[idx] = newEvents[idx];
		}
	}

	return changes;
}

//------------------------------------------------------------------------------
//
// KB_LPC32XX_Thread
//
// KSCAN handler thread
//
void KB_LPC32XX_Thread(void *pData)
{
    KEYBD_IST keyPadIst;

	// Enable KSCAN interrupt
	InterruptDone(kbdata.sysIntr);

	// Setup keypad thread
    keyPadIst.hevInterrupt = kbdata.hEvent;
    keyPadIst.dwSysIntr_Keybd = kbdata.sysIntr;
    keyPadIst.uiPddId = kbdata.v_uiPddId;
    keyPadIst.pfnGetKeybdEvent = KB_LPC32XX_GetEventEx2;
    keyPadIst.pfnKeybdEvent = kbdata.v_pfnKeybdEvent;

	// No return from this function
	KeybdIstLoop(&keyPadIst);
}

//------------------------------------------------------------------------------
//
// KB_LPC32XX_Setup
//
// Sets up the KSCAN block
//
BOOL KB_LPC32XX_Setup(void)
{
	PHYSICAL_ADDRESS pa;
	DWORD irq, threadID;
	BOOL sts = FALSE;

	// Setup initial values
	kbdata.pCLKPWRRegs = NULL;
	kbdata.pKSCANRegs = NULL;
	kbdata.sysIntr = SYSINTR_UNDEFINED;
	kbdata.hEvent = NULL;

	// Map pointers to KSCAN and CLKPWR blocks
	pa.QuadPart = CLK_PM_BASE;
	kbdata.pCLKPWRRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
		sizeof (CLKPWR_REGS_T), FALSE);
	pa.QuadPart = KSCAN_BASE;
	kbdata.pKSCANRegs = (KSCAN_REGS_T *) MmMapIoSpace(pa,
		sizeof (KSCAN_REGS_T), FALSE);
	if ((kbdata.pCLKPWRRegs == NULL) || (kbdata.pKSCANRegs == NULL))
	{
		RETAILMSG(1, (TEXT("KSCAN: Error mapping registers\r\n")));
		goto cleanup;
	}

	// Enable clocking for the KSCAN block
	kscan_enable(TRUE);

	// Clear KSCAN IRQ and get initial matrix
	kscan_int_handler(kbdata.keyStates);

	// Get sysIntr value mapped to the IRQ
	irq = OAL_INTR_IRQ_KEY;
    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &irq, sizeof(irq),
		&kbdata.sysIntr, sizeof(kbdata.sysIntr), NULL))
    {
        RETAILMSG(1, 
            (TEXT("KSCAN: Failed to request the sysintr.\r\n")));

        kbdata.sysIntr = SYSINTR_UNDEFINED;
        goto cleanup;
    }

	// Create event handle
	kbdata.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (kbdata.hEvent == NULL)
	{
        RETAILMSG(1, 
            (TEXT("KSCAN: Failed to create handler event.\r\n")));
		goto cleanup;
	}

	// Initialize the SSP interrupt
	if (InterruptInitialize(kbdata.sysIntr, kbdata.hEvent,
		NULL, 0) == FALSE) {
		// Cannot initialize interrupt
        RETAILMSG(1, 
			(TEXT("KSCAN: Cannot initialize interrupt\r\n")));
        goto cleanup;
	}

	// Start the KSCAN thread
	kbdata.hThread =  CreateThread(NULL, 0,
		(LPTHREAD_START_ROUTINE) KB_LPC32XX_Thread, NULL, 0, &threadID);
	if (kbdata.hThread == NULL)
	{
		RETAILMSG(1,
			(TEXT("KSCAN: Error creating handler thread!\r\n")));
        goto cleanup;
    }

	// Setup KSCAN timing and debounce values
	kbdata.pKSCANRegs->ks_fast_tst = KSCAN_FTST_USE32K_CLK;
	kbdata.pKSCANRegs->ks_matrix_dim = KSCAN_MSEL_SELECT(MATRIXSIZE);
	kbdata.pKSCANRegs->ks_deb = MATRIXDEBCYCLES;
	kbdata.pKSCANRegs->ks_scan_ctl = MATRIXDELCYCLES;
 
	sts = TRUE;

cleanup:
	if (sts == FALSE)
	{
		if (kbdata.hEvent != NULL)
		{
			CloseHandle(kbdata.hEvent);
		}
		if (kbdata.sysIntr != SYSINTR_UNDEFINED)
		{
			KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &kbdata.sysIntr,
		        sizeof(kbdata.sysIntr), NULL, 0, NULL);
		}
		if (kbdata.pKSCANRegs != NULL)
		{
	        MmUnmapIoSpace((PVOID) kbdata.pKSCANRegs, sizeof (KSCAN_REGS_T));
		}
		if (kbdata.pCLKPWRRegs != NULL)
		{
	        MmUnmapIoSpace((PVOID) kbdata.pCLKPWRRegs, sizeof (CLKPWR_REGS_T));
		}
	}

	return sts;
}

//------------------------------------------------------------------------------
//
// KB_LPC32XX_PowerHandler
//
// Keyboard PDD power handler function
//
static void KB_LPC32XX_PowerHandler(UINT uiPddId,
									BOOL fTurnOff)
{
	if (uiPddId == kbdata.v_uiPddId)
	{
		if (fTurnOff == FALSE)
		{
			kscan_enable(TRUE);
		}
		else
		{
			kscan_enable(FALSE);
		}
	}
}

//------------------------------------------------------------------------------
//
// KBLPC32xxPdd
//
// PDD structure used for MDD
//
static KEYBD_PDD KBLPC32xxPdd =
{
  MATRIX_PDD,
  MATRIXTYPE,
  KB_LPC32XX_PowerHandler,
  NULL
};

//------------------------------------------------------------------------------
//
// KB_PHY3250_Entry
//
// Keyboard PDD entry point
//
BOOL WINAPI KB_LPC32XX_Entry(UINT uiPddId,
							 PFN_KEYBD_EVENT pfnKeybdEvent,
							 PKEYBD_PDD *ppKeybdPdd)
{
	// Return additional PDD data
	*ppKeybdPdd = &KBLPC32xxPdd;

	// Save PDD ID and event callback
	kbdata.v_uiPddId = uiPddId;
	kbdata.v_pfnKeybdEvent = pfnKeybdEvent;

	// Initialize the KSCAN hardware and return
	return KB_LPC32XX_Setup();
}
