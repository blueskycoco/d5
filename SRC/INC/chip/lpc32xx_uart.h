/***********************************************************************
* $Id:: lpc32xx_uart.h 627 2008-04-18 19:39:54Z wellsk                $
*
* Project: LPC32xx standard UART definitions
*
* Description:
*     This file contains the structure definitions and manifest
*     constants for the LPC32xx chip family component:
*         standard UART definitions
*
***********************************************************************
* Software that is described herein is for illustrative purposes only  
* which provides customers with programming information regarding the  
* products. This software is supplied "AS IS" without any warranties.  
* NXP Semiconductors assumes no responsibility or liability for the 
* use of the software, conveys no license or title under any patent, 
* copyright, or mask work right to the product. NXP Semiconductors 
* reserves the right to make changes in the software without 
* notification. NXP Semiconductors also make no representation or 
* warranty that such application will be suitable for the specified 
* use without further testing or modification. 
**********************************************************************/

#ifndef LPC32XX_STANDARD_UART_H
#define LPC32XX_STANDARD_UART_H

#include "lpc_types.h"
#include "lpc32xx_chip.h"

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************
* Standard UART register structures
**********************************************************************/

/* Standard UART register structures */ 
typedef struct {
    volatile UNS_32 dll_fifo;     /* UART data FIFO/holding/dll reg */
    volatile UNS_32 dlm_ie;       /* UART high divisor/int enable reg */
	volatile UNS_32 iir_fcr;      /* UART int pending/FIFO contrl reg */
	volatile UNS_32 lcr;          /* UART line control reg */
	volatile UNS_32 reserved1;
	volatile UNS_32 lsr;          /* UART line status reg */
	volatile UNS_32 reserved2;
	volatile UNS_32 rxlev;        /* UART RX FIFO level reg */
} UART_REGS_T;

/* UART control structure */
typedef struct {
	volatile UNS_32 ctrl;         /* General UART control register */
	volatile UNS_32 clkmode;      /* UART clock control register */
	volatile UNS_32 loop;         /* UART loopmode enable/disable */
} UART_CNTL_REGS_T;

/**********************************************************************
* dll and dlm register definitions
**********************************************************************/
/* Macro for loading most and least significant halfs of divisors */
#define UART_LOAD_DM(div)          ((div) & 0xFF)

/**********************************************************************
* ie register definitions
**********************************************************************/
/* Bit for enabling the RX line status interrupt(s) */
#define UART_IE_RXLINE_STS         _BIT(2)
/* Bit for enabling the transmit holding register empty interrupt */
#define UART_IE_THRE               _BIT(1)
/* Bit for enabling the receive data available (RDA) interrupt */
#define UART_IE_RDA                _BIT(0)

/**********************************************************************
* iir register definitions
**********************************************************************/
/* Bit for masking interrupt pending status */
#define UART_IIR_INT_PENDING       _BIT(0)
/* Mask for getting interrupt source */
#define UART_IIR_INT_MASK          0xE
/* Interrupt sources */
#define UART_IIR_INTSRC_THRE       0x2
#define UART_IIR_INTSRC_RDA        0x4
#define UART_IIR_INTSRC_RXLINE     0x6
#define UART_IIR_INTSRC_CTI        0xC /* Character timeout */
/* Interrupt bits mask word */
#define UART_IIR_INTSRC_MASK       0xE

/**********************************************************************
* fcr register definitions
**********************************************************************/
/* Receive FIFO trigger level selections */
#define UART_FCR_RXFIFO_TL16       0x0
#define UART_FCR_RXFIFO_TL32       _BIT(6)
#define UART_FCR_RXFIFO_TL48       _BIT(7)
#define UART_FCR_RXFIFO_TL60       (_BIT(7) | _BIT(6))
/* Transmit FIFO trigger level selections */
#define UART_FCR_TXFIFO_TL0        0x0
#define UART_FCR_TXFIFO_TL4        _BIT(4)
#define UART_FCR_TXFIFO_TL8        _BIT(5)
#define UART_FCR_TXFIFO_TL16       (_BIT(5) | _BIT(4))
/* Enable FIFO bit - must be set with UART_FCR_FIFO_EN */
#define UART_FCR_FIFO_CTRL         _BIT(3)
/* Clear TX FIFO bit */
#define UART_FCR_TXFIFO_FLUSH      _BIT(2)
/* Clear RX FIFO bit */
#define UART_FCR_RXFIFO_FLUSH      _BIT(1)
/* Enable FIFO bit - must be set with UART_FCR_FIFO_CTRL */
#define UART_FCR_FIFO_EN           _BIT(0)

/**********************************************************************
* lcr register definitions
**********************************************************************/
/* Bit for enabling divisor latch and IER register */
#define UART_LCR_DIVLATCH_EN       _BIT(7)
/* Bit for enabling break transmission (forces TX low) */
#define UART_LCR_BREAK_EN          _BIT(6)
/* Parity selection */
#define UART_LCR_PARITY_ODD        0x0
#define UART_LCR_PARITY_EVEN       _BIT(4)
#define UART_LCR_PARITY_FORCE1     _BIT(5)
#define UART_LCR_PARITY_FORCE0     (_BIT(5) | _BIT(4))
/* Parity selection mask */
#define UART_LCR_PARITY_MASK       (_BIT(5) | _BIT(4))
/* Parity enable bit */
#define UART_LCR_PARITY_ENABLE     _BIT(3)
/* Stop bit selection */
#define UART_LCR_STOP1BIT          0x0
#define UART_LCR_STOP2BITS         _BIT(2)
/* Word length selections */
#define UART_LCR_WLEN_5BITS        0x0
#define UART_LCR_WLEN_6BITS        _BIT(0)
#define UART_LCR_WLEN_7BITS        _BIT(1)
#define UART_LCR_WLEN_8BITS        (_BIT(1) | _BIT(0))
/* Word length mask */
#define UART_LCR_WLEN_MASK         (_BIT(1) | _BIT(0))

/**********************************************************************
* lsr register definitions
**********************************************************************/
/* Bit for masking FIFO RX error status */
#define UART_LSR_FIFORX_ERR        _BIT(7)
/* Bit for masking transmitter empty status */
#define UART_LSR_TEMT              _BIT(6)
/* Bit for masking transmit FIFO trip point status */
#define UART_LSR_THRE              _BIT(5)
/* Bit for masking break interrupt status */
#define UART_LSR_BI                _BIT(4)
/* Bit for masking framing error status */
#define UART_LSR_FR                _BIT(3)
/* Bit for masking parity error status */
#define UART_LSR_PE                _BIT(2)
/* Bit for masking RX FIFO overrun error status */
#define UART_LSR_OE                _BIT(1)
/* Bit for masking RX FIFO empty status */
#define UART_LSR_RDR               _BIT(0)

/**********************************************************************
* rxlev register definitions
**********************************************************************/
/* Macro for masking off the receive FIFO level */
#define UART_RXLEV(n)              ((n) & 0x7F)

/**********************************************************************
* ctrl register definitions
**********************************************************************/
/* UART3 modem control pin enable bit */
#define UART_U3_MD_CTRL_EN         _BIT(11)
/* IRRX6 inversion enable bit */
#define UART_IRRX6_INV_EN          _BIT(10)
/* IRRX6 RX mask while TX enabled enable bit */
#define UART_HDPX_EN               _BIT(9)
/* UART6 IRA modulator bypass bit */
#define UART_UART6_IRDAMOD_BYPASS  _BIT(5)
/* IRTX6 inversion enable bit */
#define RT_IRTX6_INV_EN            _BIT(4)
/* IRRX6 inversion enable bit */
#define RT_IRTX6_INV_MIR_EN        _BIT(3)
/* IR RX length, 3/16th pulse length with a 115Kbps clock */
#define RT_RX_IRPULSE_3_16_115K    _BIT(2)
/* IR TX length, 3/16th pulse length with a 115Kbps clock */
#define RT_TX_IRPULSE_3_16_115K    _BIT(1)
/* UART5 mirror route to the USB D+ and D- pins bit */
#define UART_U5_ROUTE_TO_USB       _BIT(0)

/**********************************************************************
* clkmode register definitions
**********************************************************************/
/* Macro return the UART clock enabled field, shifted */
#define UART_ENABLED_CLOCKS(n)     (((n) >> 16) & 0x7F)
/* Macro returning a selected enabled UART clock bit */
#define UART_ENABLED_CLOCK(n, u)   (((n) >> (16 + (u)) & 0x1)
/* Bit that indicates if any UARTS are being clocked */
#define UART_ENABLED_CLKS_ANY      _BIT(14)
/* Defnies for setting a IARTs clock mode */
#define UART_CLKMODE_OFF           0x0    /* Clocks are off */
#define UART_CLKMODE_ON            0x1    /* Clocks are on */
#define UART_CLKMODE_AUTO          0x2    /* Clocks are automatic */
/* Clock mode mask for a UART, for UARTs 6 to 3 only */
#define UART_CLKMODE_MASK(u)      (0x3 << ((((u) - 3) * 2) + 4))
/* Macro for loading a UARTs clock mode, for UARTs 6 to 3 only */
#define UART_CLKMODE_LOAD(m, u)   ((m) << ((((u) - 3) * 2) + 4))

/**********************************************************************
* loop register definitions
**********************************************************************/
/* Marco for getting a specific UART loopback nmode enable bit, for
   all UARTs 1 to 7 */
#define UART_LPBACK_ENABLED(u)     (0x1 << ((u) - 1))

/* Macros pointing to UART registers */
#define UART3 ((UART_REGS_T *)(UART3_BASE))
#define UART4 ((UART_REGS_T *)(UART4_BASE))
#define UART5 ((UART_REGS_T *)(UART5_BASE))
#define UART6 ((UART_REGS_T *)(UART6_BASE))

/* Macro pointing to UAT control */
#define UARTCNTL ((UART_CNTL_REGS_T *) (UART_CTRL_BASE))

#ifdef __cplusplus
}
#endif

#endif /* LPC32XX_STANDARD_UART_H */ 
