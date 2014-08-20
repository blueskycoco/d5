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
// bsp_serial.c
//
// Serial port support
//

#include <windows.h>

#include <bsp_cfg.h>
#include <oal.h>
#include "bsp_serial.h"
#include "boot_utilities.h"
#include <boot_args.h>

#include "lpc32xx_uart.h"
#include "lpc32xx_clkpwr.h"
#include "clkpwr_support.h"

// Structure for computing high/low dividers for clock rate
typedef struct
{
	UNS_32 divx;
	UNS_32 divy; // For x/y
} UART_CLKDIV_T;

extern BOOT_CFG g_bootCfg;

//------------------------------------------------------------------------------
//
// sport_update_rate
//
// Update serial port baud rate
//
VOID sport_update_rate(UINT32 baud)
{
	CLKPWR_REGS_T *pClkpwr;
	UINT32 basepclk, savedclkrate, diff, clkrate;
	UINT32 idxx, idyy;
	UART_CLKDIV_T div;

	pClkpwr = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);

	// Get base clock for UART
	basepclk = (INT32)
		(clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK) >> 4);

	// Find the best divider
	div.divx = div.divy = 0;
	savedclkrate = 0;
	diff = 0xFFFFFFFF;
	for (idxx = 1; idxx < 0xFF; idxx++) {
		for (idyy = idxx; idyy < 0xFF; idyy++) {
			clkrate = (basepclk * idxx) / idyy;
			if ((UINT32) val_diff_abs(clkrate, baud) < diff) {
				diff = val_diff_abs(clkrate, baud);
				savedclkrate = clkrate;
				div.divx = idxx;
				div.divy = idyy;
			}
		}
	}
	pClkpwr->clkpwr_uart5_clk_ctrl = CLKPWR_UART_X_DIV(div.divx) |
		CLKPWR_UART_Y_DIV(div.divy);
}

//------------------------------------------------------------------------------
//
// uart_setup_trans_mode
//
// Setup debug serial port parameters
//
VOID uart_setup_trans_mode(VOID) {
	UART_REGS_T *pUARTRegs;

	pUARTRegs = (UART_REGS_T *) OALPAtoVA((UINT32) UART5, FALSE);

	// 1 stop bit, no parity, 8 data bits, 
	pUARTRegs->lcr = UART_LCR_WLEN_8BITS;

	// 38400 bps default
	sport_update_rate(BSP_UART_RATE);
}

//------------------------------------------------------------------------------
//
// OEMDebugInit
//
// Initialize debug serial port
//
BOOL OEMDebugInit()
{
	volatile UINT32 tmp;
	UART_REGS_T *pUARTRegs;
	UART_CNTL_REGS_T *pUARTCntlRegs;

	pUARTRegs = (UART_REGS_T *) OALPAtoVA((UINT32) UART5, FALSE);
	pUARTCntlRegs = (UART_CNTL_REGS_T *) OALPAtoVA((UINT32) UARTCNTL,
		FALSE);

	// Enable UART system clock and automode */
	clkpwr_clk_en_dis(CLKPWR_UART5_CLK, 1);

	tmp = pUARTCntlRegs->clkmode & UART_CLKMODE_MASK(5);
	pUARTCntlRegs->clkmode = (tmp |
		UART_CLKMODE_LOAD(UART_CLKMODE_AUTO, (5)));
	// UART baud rate generator isn't used, so just set it to divider
	//  by 1
	pUARTRegs->lcr |= UART_LCR_DIVLATCH_EN;
	pUARTRegs->dll_fifo = 1;
	pUARTRegs->dlm_ie = 0;
	pUARTRegs->lcr &= ~UART_LCR_DIVLATCH_EN;

	// Setup UART and clock
	uart_setup_trans_mode();

	// Clear FIFOs, set FIFO level, and pending interrupts
	pUARTRegs->iir_fcr = (UART_FCR_RXFIFO_TL16 | UART_FCR_TXFIFO_TL0 |
		UART_FCR_FIFO_CTRL | UART_FCR_FIFO_EN | UART_FCR_TXFIFO_FLUSH |
		UART_FCR_RXFIFO_FLUSH);
	tmp = pUARTRegs->iir_fcr;
	tmp = pUARTRegs->lsr;

	// Interrupts disabled
	pUARTRegs->dlm_ie = 0;

    return TRUE;
}

//------------------------------------------------------------------------------
//
// OEMDebugDeinit
//
// Close debug serial port
//
VOID OEMDebugDeinit()
{
	// Disable clock to the UART
	clkpwr_clk_en_dis(CLKPWR_UART5_CLK, 0);
}

//------------------------------------------------------------------------------
//
// OEMWriteDebugByte
//
// Write a byte to the serial port
//
VOID OEMWriteDebugByte(UINT8 ch)
{
	UART_REGS_T *pUARTRegs;

	pUARTRegs = (UART_REGS_T *) OALPAtoVA((UINT32) UART5, FALSE);

	// Wait for TX FIFO to get free space
	while ((pUARTRegs->lsr & UART_LSR_THRE) == 0);

	// Write byte to UART FIFO
	pUARTRegs->dll_fifo = (UNS_32) ch;
}

//------------------------------------------------------------------------------
//
// OEMReadDebugByte
//
// Read a byte from the serial port
//
int OEMReadDebugByte(void)
{
	UART_REGS_T *pUARTRegs;
    int ch = OEM_DEBUG_READ_NODATA;

	pUARTRegs = (UART_REGS_T *) OALPAtoVA((UINT32) UART5, FALSE);

	// Does RX FIFO have data?
	if ((pUARTRegs->lsr & UART_LSR_RDR) != 0)
	{
		ch = (int) pUARTRegs->dll_fifo;
	}

	return ch;
}

//------------------------------------------------------------------------------
//
// DebugOutputString
//
// Output a null terminated string to the serial port
//
void DebugOutputString(UCHAR *str)
{
    UCHAR *p;

    for(p=str;p && *p;p++)
        OEMWriteDebugByte(*p);
}
