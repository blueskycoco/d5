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
// clkpwr_support.h
//
// System clock gate and query functions
//

#pragma once

#include "lpc32xx_clkpwr.h"

// Enumeration for clock enable/disable selection and determining base
// clock rates and sources
typedef enum {
	CLKPWR_FIRST_CLK = 0,
	CLKPWR_USB_HCLK = CLKPWR_FIRST_CLK,
	CLKPWR_LCD_CLK,
	CLKPWR_SSP1_CLK,
	CLKPWR_SSP0_CLK,
	CLKPWR_I2S1_CLK,
	CLKPWR_I2S0_CLK,
	CLKPWR_MSCARD_CLK,
	CLKPWR_MAC_DMA_CLK,
	CLKPWR_MAC_MMIO_CLK,
	CLKPWR_MAC_HRC_CLK,
	CLKPWR_I2C2_CLK,
	CLKPWR_I2C1_CLK,
	CLKPWR_KEYSCAN_CLK,
	CLKPWR_ADC_CLK,
	CLKPWR_PWM2_CLK,
	CLKPWR_PWM1_CLK,
	CLKPWR_HSTIMER_CLK,
	CLKPWR_WDOG_CLK,
	CLKPWR_TIMER3_CLK,
	CLKPWR_TIMER2_CLK,
	CLKPWR_TIMER1_CLK,
	CLKPWR_TIMER0_CLK,
	CLKPWR_PWM4_CLK,
	CLKPWR_PWM3_CLK,
	CLKPWR_SPI2_CLK,
	CLKPWR_SPI1_CLK,
	CLKPWR_NAND_SLC_CLK,
	CLKPWR_NAND_MLC_CLK,
	CLKPWR_UART6_CLK,
	CLKPWR_UART5_CLK,
	CLKPWR_UART4_CLK,
	CLKPWR_UART3_CLK,
	CLKPWR_DMA_CLK,
	CLKPWR_SDRAMDDR_CLK,
	CLKPWR_ETHERNET_CLK,
	CLKPWR_LAST_CLK
	
} CLKPWR_CLK_T;

// Main system clocks
typedef enum 
{
	CLKPWR_MAINOSC_CLK, /* Main oscillator clock */
	CLKPWR_RTC_CLK,     /* RTC clock */
	CLKPWR_ARM_CLK,     /* ARM clock, either HCLK(PLL), SYSCLK, or
	                       PERIPH_CLK */
	CLKPWR_HCLK,        /* HCLK (HCLKPLL divided, SYSCLK, or
	                       PERIPH_CLK) */
	CLKPWR_PERIPH_CLK,  /* Peripheral clock (HCLKPLL divided or
	                       SYSCLK) */
	CLKPWR_BASE_INVALID
} CLKPWR_BASE_CLOCK_T;

// Structure used for setting up the HCLK PLL
typedef struct {
	INT_32 analog_on;      /* (0) = analog off, (!0) = on */
	INT_32 cco_bypass_b15; /* (0) = CCO clock sent to post divider,
						      (!0) = PLL input clock sent to post div */
	INT_32 direct_output_b14; /* (0) = PLL out from post divider, (!0) =
	                          PLL out bypasses post divider */
	INT_32 fdbk_div_ctrl_b13; /* (0) = use CCO clock, (!0) = use FCLKOUT */
	INT_32 pll_p;          /* Must be 1, 2, 4, or 8 */
	INT_32 pll_n;          /* Must be 1, 2, 3, or 4 */
	UNS_32 pll_m;          /* Feedback multiplier 1-256 */
} CLKPWR_HCLK_PLL_SETUP_T;

// Get the clock frequency for a system base clock (such as HCLK PLL,
// main oscillator, PERIPH_CLK, etc.)
UNS_32 clkpwr_get_base_clock_rate(CLKPWR_BASE_CLOCK_T baseclk);

// Enable or disable a clock to a specific clocked peripheral
void clkpwr_clk_en_dis(CLKPWR_CLK_T clk,
					   INT_32 enable);

// Return absolute difference between 2 values
INT32 val_diff_abs(INT32 v1, INT32 v2);
