//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
/*++
THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
PARTICULAR PURPOSE.

Module Name:  

hw16550.h

Abstract:  

Type definitions and data for the serial port driver

Notes: 


--*/

#ifdef __cplusplus
extern "C" {
#endif
/*
 * High speed UART register offsets
 */
#define LPC32XX_HSUART_FIFO(x)			(x + 0x00)
#define LPC32XX_HSUART_LEVEL(x)			(x + 0x04)
#define LPC32XX_HSUART_IIR(x)			(x + 0x08)
#define LPC32XX_HSUART_CTRL(x)			(x + 0x0C)
#define LPC32XX_HSUART_RATE(x)			(x + 0x10)

#define LPC32XX_HSU_BREAK_DATA			(1 << 10)
#define LPC32XX_HSU_ERROR_DATA			(1 << 9)
#define LPC32XX_HSU_RX_EMPTY			(1 << 8)

#define LPC32XX_HSU_TX_LEV(n)			(((n) >> 8) & 0xFF)
#define LPC32XX_HSU_RX_LEV(n)			((n) & 0xFF)

#define LPC32XX_HSU_TX_INT_SET			(1 << 6)
#define LPC32XX_HSU_RX_OE_INT			(1 << 5)
#define LPC32XX_HSU_BRK_INT			(1 << 4)
#define LPC32XX_HSU_FE_INT			(1 << 3)
#define LPC32XX_HSU_RX_TIMEOUT_INT		(1 << 2)
#define LPC32XX_HSU_RX_TRIG_INT			(1 << 1)
#define LPC32XX_HSU_TX_INT			(1 << 0)

#define LPC32XX_HSU_HRTS_INV			(1 << 21)
#define LPC32XX_HSU_HRTS_TRIG_8B		(0x0 << 19)
#define LPC32XX_HSU_HRTS_TRIG_16B		(0x1 << 19)
#define LPC32XX_HSU_HRTS_TRIG_32B		(0x2 << 19)
#define LPC32XX_HSU_HRTS_TRIG_48B		(0x3 << 19)
#define LPC32XX_HSU_HRTS_EN			(1 << 18)
#define LPC32XX_HSU_TMO_DISABLED		(0x0 << 16)
#define LPC32XX_HSU_TMO_INACT_4B		(0x1 << 16)
#define LPC32XX_HSU_TMO_INACT_8B		(0x2 << 16)
#define LPC32XX_HSU_TMO_INACT_16B		(0x3 << 16)
#define LPC32XX_HSU_HCTS_INV			(1 << 15)
#define LPC32XX_HSU_HCTS_EN			(1 << 14)
#define LPC32XX_HSU_OFFSET(n)			((n) << 9)
#define LPC32XX_HSU_BREAK			(1 << 8)
#define LPC32XX_HSU_ERR_INT_EN			(1 << 7)
#define LPC32XX_HSU_RX_INT_EN			(1 << 6)
#define LPC32XX_HSU_TX_INT_EN			(1 << 5)
#define LPC32XX_HSU_RX_TL1B			(0x0 << 2)
#define LPC32XX_HSU_RX_TL4B			(0x1 << 2)
#define LPC32XX_HSU_RX_TL8B			(0x2 << 2)
#define LPC32XX_HSU_RX_TL16B			(0x3 << 2)
#define LPC32XX_HSU_RX_TL32B			(0x4 << 2)
#define LPC32XX_HSU_RX_TL48B			(0x5 << 2)
#define LPC32XX_HSU_TX_TLEMPTY			(0x0 << 0)
#define LPC32XX_HSU_TX_TL0B			(0x0 << 0)
#define LPC32XX_HSU_TX_TL4B			(0x1 << 0)
#define LPC32XX_HSU_TX_TL8B			(0x2 << 0)
#define LPC32XX_HSU_TX_TL16B			(0x3 << 0)

//
// Offsets from the base register address of the
// various registers for the 8250/16440 family of UARTS.
//
#define RECEIVE_BUFFER_REGISTER    		0x00
#define TRANSMIT_HOLDING_REGISTER  	0x00
#define LEVEL_REGISTER  				0x04
#define INTERRUPT_IDENT_REGISTER   		0x08
#define CONTROL_REGISTER	   		0x0c
#define RATE_CONTROL_REGISTER	   	0x10


//
// All 16550's have the same FIFO depth
//
#define SERIAL_FIFO_DEPTH 16

//
// Bitmask definitions for accessing the 8250 device registers.
//

//
// These bits define the number of data bits trasmitted in
// the Serial Data Unit (SDU - Start,data, parity, and stop bits)
//
#define SERIAL_DATA_LENGTH_5 0x00
#define SERIAL_DATA_LENGTH_6 0x01
#define SERIAL_DATA_LENGTH_7 0x02
#define SERIAL_DATA_LENGTH_8 0x03


//
// These masks define the interrupts that can be enabled or disabled.
//
//
// This interrupt is used to notify that there is new incomming
// data available.  The SERIAL_RDA interrupt is enabled by this bit.
//
#define SERIAL_IER_RDA   0x01

//
// This interrupt is used to notify that there is space available
// in the transmitter for another character.  The SERIAL_THR
// interrupt is enabled by this bit.
//
#define SERIAL_IER_THR   0x02

//
// This interrupt is used to notify that some sort of error occured
// with the incomming data.  The SERIAL_RLS interrupt is enabled by
// this bit.
#define SERIAL_IER_RLS   0x04

//
// This interrupt is used to notify that some sort of change has
// taken place in the modem control line.  The SERIAL_MS interrupt is
// enabled by this bit.
//
#define SERIAL_IER_MS    0x08

// If bit 0 of IIR != 0, then the interrupt vector is invalid
#define SERIAL_IIR_INT_INVALID   0x01

// The bits 1..3 of the IIR indicate the source of the interruptr
#define SERIAL_IIR_INT_MASK      0x0f

//
// These defines represent the interrupt values of the interrupt identification
// register.  The values are listed below in decreasing order
// of priority.  Note that a higher ordinal value does NOT
// imply a higher priority interrupt.
//
#define SERIAL_IIR_RLS      0x06  // Receiver Line Status
#define SERIAL_IIR_RDA      0x04  // Receive Data Available
#define SERIAL_IIR_CTI      0x0C  // Character Timeout Indicator
#define SERIAL_IIR_CTI_2    0x0E  // Yet Another Character Timeout Indicator
#define SERIAL_IIR_THRE     0x02  // Transmit Holding Register Empty
#define SERIAL_IIR_MS       0x00  // Check Modem Status Register

//
// This bit mask get the value of the high two bits of the
// interrupt id register.  If this is a 16550 class chip
// these bits will be a one if the fifo's are enbled, otherwise
// they will always be zero.
//
#define SERIAL_IIR_FIFOS_ENABLED 0xc0

//
// If the low bit is logic one in the interrupt identification register
// this implies that *NO* interrupts are pending on the device.
//
#define SERIAL_IIR_NO_INTERRUPT_PENDING 0x01

//
// These masks define access to the fifo control register.
//

//
// Enabling this bit in the fifo control register will turn
// on the fifos.  If the fifos are enabled then the high two
// bits of the interrupt id register will be set to one.  Note
// that this only occurs on a 16550 class chip.  If the high
// two bits in the interrupt id register are not one then
// we know we have a lower model chip.
//
//
#define SERIAL_FCR_ENABLE     ((UCHAR)0x01)
#define SERIAL_FCR_RCVR_RESET ((UCHAR)0x02)
#define SERIAL_FCR_TXMT_RESET ((UCHAR)0x04)

//
// This set of values define the high water marks (when the
// interrupts trip) for the receive fifo.
//
#define SERIAL_1_BYTE_HIGH_WATER   ((UCHAR)0x00)
#define SERIAL_4_BYTE_HIGH_WATER   ((UCHAR)0x40)
#define SERIAL_8_BYTE_HIGH_WATER   ((UCHAR)0x80)
#define SERIAL_14_BYTE_HIGH_WATER  ((UCHAR)0xc0)

//
// These masks define access to the line control register.
//

//
// This defines the bit used to control the definition of the "first"
// two registers for the 8250.  These registers are the input/output
// register and the interrupt enable register.  When the DLAB bit is
// enabled these registers become the least significant and most
// significant bytes of the divisor value.
//
#define SERIAL_LCR_DLAB     0x80

//
// This defines the bit used to control whether the device is sending
// a break.  When this bit is set the device is sending a space (logic 0).
//
// Most protocols will assume that this is a hangup.
//
#define SERIAL_LCR_BREAK    0x40

//
// These defines are used to set the line control register.
//
#define SERIAL_5_DATA       ((UCHAR)0x00)
#define SERIAL_6_DATA       ((UCHAR)0x01)
#define SERIAL_7_DATA       ((UCHAR)0x02)
#define SERIAL_8_DATA       ((UCHAR)0x03)
#define SERIAL_DATA_MASK    ((UCHAR)0x03)

#define SERIAL_1_STOP       ((UCHAR)0x00)
#define SERIAL_1_5_STOP     ((UCHAR)0x04) // Only valid for 5 data bits
#define SERIAL_2_STOP       ((UCHAR)0x04) // Not valid for 5 data bits
#define SERIAL_STOP_MASK    ((UCHAR)0x04)

#define SERIAL_NONE_PARITY  ((UCHAR)0x00)
#define SERIAL_ODD_PARITY   ((UCHAR)0x08)
#define SERIAL_EVEN_PARITY  ((UCHAR)0x18)
#define SERIAL_MARK_PARITY  ((UCHAR)0x28)
#define SERIAL_SPACE_PARITY ((UCHAR)0x38)
#define SERIAL_PARITY_MASK  ((UCHAR)0x38)

//
// These masks define access the modem control register.
//

//
// This bit controls the data terminal ready (DTR) line.  When
// this bit is set the line goes to logic 0 (which is then inverted
// by normal hardware).  This is normally used to indicate that
// the device is available to be used.  Some hardware
// protocols (like the kernel debugger) use this for handshaking
// purposes.
//
#define SERIAL_MCR_DTR      0x01

//
// This bit controls the ready to send (RTS) line.  When this bit
// is set the line goes to logic 0 (which is then inverted by the normal
// hardware).  This is used for hardware handshaking.  It indicates that
// the hardware is ready to send data and it is waiting for the
// receiving end to set clear to send (CTS).
//
#define SERIAL_MCR_RTS      0x02

//
// This bit is used for general purpose output.
//
#define SERIAL_MCR_OUT1     0x04

//
// OUT2 en/disables serial interrupts 
//
#define SERIAL_MCR_OUT2         0x08
#define SERIAL_MCR_IRQ_ENABLE   SERIAL_MCR_OUT2

//
// This bit controls the loopback testing mode of the device.  Basically
// the outputs are connected to the inputs (and vice versa).
//
#define SERIAL_MCR_LOOP     0x10


//
// These masks define access to the line status register.  The line
// status register contains information about the status of data
// transfer.  The first five bits deal with receive data and the
// last two bits deal with transmission.  An interrupt is generated
// whenever bits 1 through 4 in this register are set.
//

//
// This bit is the data ready indicator.  It is set to indicate that
// a complete character has been received.  This bit is cleared whenever
// the receive buffer register has been read.
//
#define SERIAL_LSR_DR       0x01

//
// This is the overrun indicator.  It is set to indicate that the receive
// buffer register was not read befor a new character was transferred
// into the buffer.  This bit is cleared when this register is read.
//
#define SERIAL_LSR_OE       0x02

//
// This is the parity error indicator.  It is set whenever the hardware
// detects that the incoming serial data unit does not have the correct
// parity as defined by the parity select in the line control register.
// This bit is cleared by reading this register.
//
#define SERIAL_LSR_PE       0x04

//
// This is the framing error indicator.  It is set whenever the hardware
// detects that the incoming serial data unit does not have a valid
// stop bit.  This bit is cleared by reading this register.
//
#define SERIAL_LSR_FE       0x08

//
// This is the break interrupt indicator.  It is set whenever the data
// line is held to logic 0 for more than the amount of time it takes
// to send one serial data unit.  This bit is cleared whenever the
// this register is read.
//
#define SERIAL_LSR_BI       0x10

//
// This is the transmit holding register empty indicator.  It is set
// to indicate that the hardware is ready to accept another character
// for transmission.  This bit is cleared whenever a character is
// written to the transmit holding register.
//
#define SERIAL_LSR_THRE     0x20

//
// This bit is the transmitter empty indicator.  It is set whenever the
// transmit holding buffer is empty and the transmit shift register
// (a non-software accessable register that is used to actually put
// the data out on the wire) is empty.  Basically this means that all
// data has been sent.  It is cleared whenever the transmit holding or
// the shift registers contain data.
//
#define SERIAL_LSR_TEMT     0x40

//
// This bit indicates that there is at least one error in the fifo.
// The bit will not be turned off until there are no more errors
// in the fifo.
//
#define SERIAL_LSR_FIFOERR  0x80


//
// These masks are used to access the modem status register.
// Whenever one of the first four bits in the modem status
// register changes state a modem status interrupt is generated.
//

//
// This bit is the delta clear to send.  It is used to indicate
// that the clear to send bit (in this register) has *changed*
// since this register was last read by the CPU.
//
#define SERIAL_MSR_DCTS     0x01

//
// This bit is the delta data set ready.  It is used to indicate
// that the data set ready bit (in this register) has *changed*
// since this register was last read by the CPU.
//
#define SERIAL_MSR_DDSR     0x02

//
// This is the trailing edge ring indicator.  It is used to indicate
// that the ring indicator input has changed from a low to high state.
//
#define SERIAL_MSR_TERI     0x04

//
// This bit is the delta data carrier detect.  It is used to indicate
// that the data carrier bit (in this register) has *changed*
// since this register was last read by the CPU.
//
#define SERIAL_MSR_DDCD     0x08

//
// This bit contains the (complemented) state of the clear to send
// (CTS) line.
//
#define SERIAL_MSR_CTS      0x10

//
// This bit contains the (complemented) state of the data set ready
// (DSR) line.
//
#define SERIAL_MSR_DSR      0x20

//
// This bit contains the (complemented) state of the ring indicator
// (RI) line.
//
#define SERIAL_MSR_RI       0x40

//
// This bit contains the (complemented) state of the data carrier detect
// (DCD) line.
//
#define SERIAL_MSR_DCD      0x80

//
// This should be more than enough space to hold then
// numeric suffix of the device name.
//
#define DEVICE_NAME_DELTA 20


//
// Default xon/xoff characters.
//
#define SERIAL_DEF_XON 0x11
#define SERIAL_DEF_XOFF 0x13

//
// Reasons that recption may be held up.
//
#define SERIAL_RX_DTR	    ((UCHAR)0x01)
#define SERIAL_RX_XOFF	    ((UCHAR)0x02)
#define SERIAL_RX_RTS	    ((UCHAR)0x04)
#define SERIAL_RX_DSR	    ((UCHAR)0x08)

//
// Reasons that transmission may be held up.
//
#define SERIAL_TX_CTS	    ((UCHAR)0x01)
#define SERIAL_TX_DSR	    ((UCHAR)0x02)
#define SERIAL_TX_DCD	    ((UCHAR)0x04)
#define SERIAL_TX_XOFF	    ((UCHAR)0x08)
#define SERIAL_TX_BREAK     ((UCHAR)0x10)

//
// These values are used by the routines that can be used
// to complete a read (other than interval timeout) to indicate
// to the interval timeout that it should complete.
//
#define SERIAL_COMPLETE_READ_CANCEL ((CHAR)-1)
#define SERIAL_COMPLETE_READ_TOTAL ((CHAR)-2)
#define SERIAL_COMPLETE_READ_COMPLETE ((CHAR)-3)


#ifdef __cplusplus
}
#endif


