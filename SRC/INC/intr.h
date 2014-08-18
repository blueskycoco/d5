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
// intr.h
//
// OAL interrupt IDs
//

#pragma once

// There are more hardware interrupts than the WinCE configuration can support.
// In WinCE 6.0, the maximum number of interrupts supported is
// OAL_INTR_IRQ_MAXIMUM. The LCP32xx hardware supports 96 interrupts. These
// interrupt macros are used by drivers and software to map to WinCE events
// and do not represent the LPC32xx interrupt IDs in lpc3xxx_intc.h or interrupt
// priority.
#define OAL_INTR_IRQ_UART5       0 // Must start at 0 and increment by 1
#define OAL_INTR_IRQ_UART3       1
#define OAL_INTR_IRQ_NAND        2
#define OAL_INTR_IRQ_SD0         3
#define OAL_INTR_IRQ_SD1         4
#define OAL_INTR_IRQ_MSTIMER     5
#define OAL_INTR_IRQ_TIMER0      6
#define OAL_INTR_IRQ_SSP0        7
#define OAL_INTR_IRQ_SSP1        8
#define OAL_INTR_IRQ_I2S1        9
#define OAL_INTR_IRQ_ETHERNET    10
#define OAL_INTR_IRQ_TS_P        11
#define OAL_INTR_IRQ_TS          12
#define OAL_INTR_IRQ_I2C_1       13
#define OAL_INTR_IRQ_KEY         14
#define OAL_INTR_IRQ_USB_HOST    15
#define OAL_INTR_IRQ_USB_DEV_DMA 16
#define OAL_INTR_IRQ_USB_DEV_LP  17
#define OAL_INTR_IRQ_USB_DEV_HP  18
#define OAL_INTR_IRQ_I2C         19
#define OAL_INTR_IRQ_GPIO_00     20
#define OAL_INTR_IRQ_GPIO_01     21
#define OAL_INTR_IRQ_GPIO_02     22
#define OAL_INTR_IRQ_GPIO_03     23
#define OAL_INTR_IRQ_GPIO_04     24
#define OAL_INTR_IRQ_GPIO_05     25
#define OAL_INTR_IRQ_P0_P1_IRQ   26
#define OAL_INTR_IRQ_OTG_TIMER   27
#define OAL_INTR_IRQ_OTG_ATC     28
#define OAL_INTR_IRQ_OTG_I2C     29
#define OAL_INTR_IRQ_GPI_07      30
#define OAL_INTR_IRQ_GPI_00      31
#define OAL_INTR_IRQ_GPI_01      32
#define OAL_INTR_IRQ_GPI_02      33
#define OAL_INTR_IRQ_GPI_03      34
#define OAL_INTR_IRQ_GPI_04      35
#define OAL_INTR_IRQ_GPI_05      36
#define OAL_INTR_IRQ_GPI_06      37
#define OAL_INTR_IRQ_I2C_2       38
#define OAL_INTR_IRQ_LAST_GRP    38

// Required system interrupts
#define OAL_INTR_IRQ_RTC         39 // Mapped to RTC (alarm) AND timer 1
#define OAL_INTR_IRQ_TICK        40 // Mapped to timer 3
#define OAL_INTR_IRQ_PROFILER    41 // Mapped to timer 2 (if used)

// The DMA controller has 1 interrupt, but up to 8 virtual interrupts
// The main DMA interrupt is trapped in the interrupt handler and the
// handler will use the correct virtual interrupt based on which DMA
// channel is requesting service. Although 8 channels are supported,
// not all 8 channels have to be used in a design.
#define OAL_INTR_VIRT_FIRST      42
#define OAL_INTR_DMACH_0         OAL_INTR_VIRT_FIRST
#define OAL_INTR_DMACH_1         43
#define OAL_INTR_DMACH_2         44
#define OAL_INTR_DMACH_3         45
#define OAL_INTR_DMACH_4         46
#define OAL_INTR_DMACH_5         47
#define OAL_INTR_DMACH_6         48
#define OAL_INTR_IRQ_UART1       49
#define OAL_INTR_VIRT_LAST       49

#define OAL_INTR_IRQ_LAST        49

// Profiler ISR function pointer type
typedef UINT32 (*PFN_PROFILER_ISR)(UINT32 ra);
