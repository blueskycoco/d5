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
// bsp_clkpwr_support.c
//
// LED setup and control functions used with the bootloader
//

#include <windows.h>
#include <oal.h>
#include "clkpwr_support.h"

/* Post divider values for PLLs based on selected register value */
static UNS_32 pll_postdivs[4] = {1, 2, 4, 8};

/* CLK divider values for HCLK based on selected register value */
static UNS_32 hclkdivs[4] = {1, 2, 4, 4};

//------------------------------------------------------------------------------
//
// val_diff_abs
//
// Returns the difference between 2 values.
//
INT32 val_diff_abs(INT32 v1, INT32 v2)
{
    if (v1 > v2)
	{
	    return v1 - v2;
	}

    return v2 - v1;
}

//------------------------------------------------------------------------------
//
// clkpwr_mask_and_set
//
// Set or mask off a specific field or bit
//
void clkpwr_mask_and_set(volatile UNS_32 *pReg,
						 UNS_32 mask,
						 UNS_32 set) {
	UNS_32 tmp;

	tmp = *pReg & ~mask;
	if (set != 0) {
		tmp |= mask;
	}
	*pReg = tmp;
}

//------------------------------------------------------------------------------
//
// clkpwr_check_pll_setup
//
// Determines if a PLL setup is valid
//
UNS_32 clkpwr_check_pll_setup(UNS_32 ifreq,
						      CLKPWR_HCLK_PLL_SETUP_T *pllsetup)
{
	UNS_64 i64freq, p, m, n, fcco, fref, cfreq;
	INT_32 mode;

	/* PLL requirements */
	/* ifreq must be >= 1MHz and <= 20MHz */
	/* FCCO must be >= 156MHz and <= 320MHz */
	/* FREF must be >= 1MHz and <= 27MHz. */
	/* Assume the passed input data is not valid */

	/* Work with 64-bit values to prevent overflow */
	i64freq = (UNS_64) ifreq;
	m = (UNS_64) pllsetup->pll_m;
	n = (UNS_64) pllsetup->pll_n;
	p = (UNS_64) pllsetup->pll_p;

	/* Get components of the PLL register */
	mode = (pllsetup->cco_bypass_b15 << 2) |
		(pllsetup->direct_output_b14 << 1) |
		pllsetup->fdbk_div_ctrl_b13;
	switch (mode)
	{
		case 0x0: /* Non-integer mode */
			cfreq = (m * i64freq) / (2 * p * n);
			fcco = (m * i64freq) / n;
			fref = i64freq / n;
			break;

		case 0x1: /* integer mode */
			cfreq = (m * i64freq) / n;
			fcco = (m * i64freq) / (n * 2 * p);
			fref = i64freq / n;
			break;

		case 0x2:
		case 0x3: /* Direct mode */
			cfreq = (m * i64freq) / n;
			fcco = cfreq;
			fref = i64freq / n;
			break;

		case 0x4:
		case 0x5: /* Bypass mode */
			cfreq = i64freq / (2 * p);
			fcco = 156000000;
			fref = 1000000;
			break;

		case 0x6:
		case 0x7: /* Direct bypass mode */
			cfreq = i64freq;
			fcco = 156000000;
			fref = 1000000;
			break;
	}

	if ((fcco < 156000000) || (fcco > 320000000))
	{
		/* not a valid range */
		cfreq = 0;
	}

	if ((fref < 1000000) || (fref > 27000000))
	{
		/* not a valid range */
		cfreq = 0;
	}

	return (INT_32) cfreq;
}

//------------------------------------------------------------------------------
//
// clkpwr_pll_rate_from_val
//
// Compute a PLL's frequency from a PLL register value
//
UNS_32 clkpwr_pll_rate_from_val(UNS_32 osc_rate,
                                UNS_32 val)
{
	CLKPWR_HCLK_PLL_SETUP_T pllcfg;

	/* Get components of the PLL register */
	pllcfg.cco_bypass_b15 = 0;
	pllcfg.direct_output_b14 = 0;
	pllcfg.fdbk_div_ctrl_b13 = 0;
	if ((val & CLKPWR_HCLKPLL_CCO_BYPASS) != 0)
	{
		pllcfg.cco_bypass_b15 = 1;
	}
	if ((val & CLKPWR_HCLKPLL_POSTDIV_BYPASS) != 0)
	{
		pllcfg.direct_output_b14 = 1;
	}
	if ((val & CLKPWR_HCLKPLL_FDBK_SEL_FCLK) != 0)
	{
		pllcfg.fdbk_div_ctrl_b13 = 1;
	}
	pllcfg.pll_m = 1 + ((val >> 1) & 0xFF);
	pllcfg.pll_n = 1 + ((val >> 9) & 0x3);
	pllcfg.pll_p = pll_postdivs[((val >> 11) & 0x3)];

    return clkpwr_check_pll_setup(osc_rate, &pllcfg);
}

//------------------------------------------------------------------------------
//
// clkpwr_pll_rate
//
// Compute a PLL's frequency
//
UNS_32 clkpwr_pll_rate(UNS_32 osc_rate,
                       UNS_32 pPllreg)
{
   return clkpwr_pll_rate_from_val(osc_rate, pPllreg);
}

//------------------------------------------------------------------------------
//
// clkpwr_get_base_clock_rate
//
// Get the clock frequency for a base clock
//
UNS_32 clkpwr_get_base_clock_rate(CLKPWR_BASE_CLOCK_T baseclk) 
{
	UNS_32 sys_clk, ddr_clock, ddr_hclk_div, hclkpll_clk, periph_clk;
	UNS_32 tmp, hclk1_clk, arm1_clk, hclk_clk, arm_clk, clkrate;
	CLKPWR_REGS_T *pClkPwr;
	PHYSICAL_ADDRESS phBase; 
	phBase.QuadPart = CLK_PM_BASE;

	pClkPwr = (CLKPWR_REGS_T *)MmMapIoSpace(phBase, sizeof(CLKPWR_REGS_T), FALSE);// OALPAtoVA((UINT32) CLKPWR, FALSE);

    /* Is PLL397 oscillator being used? */
    if ((pClkPwr->clkpwr_sysclk_ctrl & CLKPWR_SYSCTRL_USEPLL397) != 0)
    {
        /* PLL397 is used */
    	sys_clk = CLOCK_OSC_FREQ * 397;
    }
    else 
    {
    	sys_clk = MAIN_OSC_FREQ;
    }

	/* Compute HCLK DDR divider */
    ddr_hclk_div = 0;
    if ((pClkPwr->clkpwr_sdramclk_ctrl & CLKPWR_SDRCLK_USE_DDR) != 0) 
    {
    	/* DDR is being used */
		if ((pClkPwr->clkpwr_hclk_div & CLKPWR_HCLKDIV_DDRCLK_NORM) != 0)
		{
			ddr_hclk_div = 1;
		}
		else if ((pClkPwr->clkpwr_hclk_div &
			CLKPWR_HCLKDIV_DDRCLK_HALF) != 0)
		{
			ddr_hclk_div = 2;
		}
    }
    else 
    {
    	/* SDRAM is being used */
    	tmp = pClkPwr->clkpwr_hclk_div & CLKPWR_HCLKDIV_DIV_2POW(0x3);
    	ddr_hclk_div = hclkdivs[tmp] - 1;
    }

	/* Is the device in run mode? */
	if ((pClkPwr->clkpwr_pwr_ctrl & CLKPWR_SELECT_RUN_MODE) != 0)
	{
		/* In run mode */

		/* Compute HCLK PLL rate */
		hclkpll_clk = clkpwr_pll_rate(sys_clk,
    		pClkPwr->clkpwr_hclkpll_ctrl);

		/* Base DDR rate */
		ddr_clock = hclkpll_clk;

		/* Base peripheral clock rate */
		tmp = 1 + ((pClkPwr->clkpwr_hclk_div >> 2) & 0x1F);
		periph_clk = hclkpll_clk / tmp;

		/* Base HCLK rate (when not using peripheral clock */
		hclk1_clk = hclkpll_clk /
			hclkdivs[CLKPWR_HCLKDIV_DIV_2POW(pClkPwr->clkpwr_hclk_div)];

		/* Base ARM clock (when not using peripheral clock */
		arm1_clk = hclkpll_clk;
	}
	else
	{
		/* In direct-run mode */

		/* Base DDR rate */
		ddr_clock = sys_clk;

		/* Base peripheral clock rate */
		periph_clk = sys_clk;

		/* Base HCLK rate (when not using peripheral clock */
		hclk1_clk = sys_clk;

		/* Base ARM clock (when not using peripheral clock */
		arm1_clk = sys_clk;
	}

	/* Compute SDRAM/DDR clock */
	ddr_clock = ddr_clock / (ddr_hclk_div + 1);

	/* Compute HCLK and ARM clock rates */
	if ((pClkPwr->clkpwr_pwr_ctrl & CLKPWR_CTRL_FORCE_PCLK) != 0)
	{
		/* HCLK and ARM clock run from peripheral clock */
		hclk_clk = periph_clk;
		arm_clk = periph_clk;
	}
	else
	{
		/* Normal clock is used for HCLK and ARM clock */
		hclk_clk = hclk1_clk;
		arm_clk = arm1_clk;
	}

    /* Determine rates */
    switch (baseclk) 
    {
    	case CLKPWR_RTC_CLK:
			/* RTC oscillator rate */
    	    clkrate = CLOCK_OSC_FREQ;
    	    break;

    	case CLKPWR_ARM_CLK:
    	    clkrate = arm_clk;
    	    break;

    	case CLKPWR_HCLK:
    	    clkrate = hclk_clk;
    	    break;

    	case CLKPWR_PERIPH_CLK:
    	    clkrate = periph_clk;
    	    break;

        default:
			clkrate = 0;
            break;
    }

    return clkrate;
}

//------------------------------------------------------------------------------
//
// clkpwr_clk_en_dis
//
// Enable or disable a clock
//
void clkpwr_clk_en_dis(CLKPWR_CLK_T clk,
					   INT_32 enable) {
	CLKPWR_REGS_T *pClkPwr;
	PHYSICAL_ADDRESS phBase; 
	phBase.QuadPart = CLK_PM_BASE;
	
	pClkPwr = (CLKPWR_REGS_T *)MmMapIoSpace(phBase, sizeof(CLKPWR_REGS_T), FALSE);// OALPAtoVA((UINT32) CLKPWR, FALSE);

	//pClkPwr = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);

   switch (clk) {
		case CLKPWR_LCD_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_lcdclk_ctrl,
				CLKPWR_LCDCTRL_CLK_EN, enable);
			break;

		case CLKPWR_SSP1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_ssp_blk_ctrl,
				CLKPWR_SSPCTRL_SSPCLK1_EN, enable);
			break;

		case CLKPWR_SSP0_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_ssp_blk_ctrl,
				CLKPWR_SSPCTRL_SSPCLK0_EN, enable);
			break;

		case CLKPWR_I2S1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_i2s_clk_ctrl,
				CLKPWR_I2SCTRL_I2SCLK1_EN, enable);
			break;

		case CLKPWR_I2S0_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_i2s_clk_ctrl,
				CLKPWR_I2SCTRL_I2SCLK0_EN, enable);
			break;

		case CLKPWR_MSCARD_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_ms_ctrl,
				CLKPWR_MSCARD_SDCARD_EN, enable);
			if ((pClkPwr->clkpwr_ms_ctrl &
			    CLKPWR_MSCARD_SDCARD_DIV(0xF)) == 0)
			{
				/* Set fastest clock */
				pClkPwr->clkpwr_ms_ctrl |= CLKPWR_MSCARD_SDCARD_DIV(1);
			}
			break;

		case CLKPWR_MAC_DMA_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_macclk_ctrl,
				CLKPWR_MACCTRL_DMACLK_EN, enable);
			break;

		case CLKPWR_MAC_MMIO_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_macclk_ctrl,
				CLKPWR_MACCTRL_MMIOCLK_EN, enable);
			break;

		case CLKPWR_MAC_HRC_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_macclk_ctrl,
				CLKPWR_MACCTRL_HRCCLK_EN, enable);
			break;

		case CLKPWR_ETHERNET_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_macclk_ctrl,
				CLKPWR_MACCTRL_DMACLK_EN|CLKPWR_MAC_MMIO_CLK|CLKPWR_MACCTRL_HRCCLK_EN|(3<<3), enable);
			break;

		case CLKPWR_I2C2_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_i2c_clk_ctrl,
				CLKPWR_I2CCLK_I2C2CLK_EN, enable);
			break;

		case CLKPWR_I2C1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_i2c_clk_ctrl,
				CLKPWR_I2CCLK_I2C2CLK_EN, enable);
			break;

		case CLKPWR_KEYSCAN_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_key_clk_ctrl,
				CLKPWR_KEYCLKCTRL_CLK_EN, enable);
			break;

		case CLKPWR_ADC_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_adc_clk_ctrl,
				CLKPWR_ADC32CLKCTRL_CLK_EN, enable);
			break;

		case CLKPWR_PWM2_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_pwm_clk_ctrl,
				CLKPWR_PWMCLK_PWM2CLK_EN, enable);
			break;

		case CLKPWR_PWM1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_pwm_clk_ctrl,
				CLKPWR_PWMCLK_PWM1CLK_EN, enable);
			break;

		case CLKPWR_HSTIMER_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timer_clk_ctrl,
				CLKPWR_PWMCLK_HSTIMER_EN, enable);
			break;

		case CLKPWR_WDOG_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timer_clk_ctrl,
				CLKPWR_PWMCLK_WDOG_EN, enable);
			break;

		case CLKPWR_TIMER3_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_TMRPWMCLK_TIMER3_EN, enable);
			break;

		case CLKPWR_TIMER2_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_TMRPWMCLK_TIMER2_EN, enable);
			break;

		case CLKPWR_TIMER1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_TMRPWMCLK_TIMER1_EN, enable);
			break;

		case CLKPWR_TIMER0_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_TMRPWMCLK_TIMER0_EN, enable);
			break;

		case CLKPWR_PWM4_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_TMRPWMCLK_PWM4_EN, enable);
			break;

		case CLKPWR_PWM3_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_timers_pwms_clk_ctrl_1,
				CLKPWR_PWM3_CLK, enable);
			break;

		case CLKPWR_SPI2_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_spi_clk_ctrl,
				CLKPWR_SPICLK_SPI2CLK_EN, enable);
			break;

		case CLKPWR_SPI1_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_spi_clk_ctrl,
				CLKPWR_SPICLK_SPI1CLK_EN, enable);
			break;

		case CLKPWR_NAND_SLC_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_nand_clk_ctrl,
				CLKPWR_NANDCLK_SLCCLK_EN, enable);
			break;

		case CLKPWR_NAND_MLC_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_nand_clk_ctrl,
				CLKPWR_NANDCLK_MLCCLK_EN, enable);
			break;
			
		case CLKPWR_UART6_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_uart_clk_ctrl,
				CLKPWR_UARTCLKCTRL_UART6_EN, enable);
			break;
			
		case CLKPWR_UART5_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_uart_clk_ctrl,
				CLKPWR_UARTCLKCTRL_UART5_EN, enable);
			break;

		case CLKPWR_UART4_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_uart_clk_ctrl,
				CLKPWR_UARTCLKCTRL_UART4_EN, enable);
			break;

		case CLKPWR_UART3_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_uart_clk_ctrl,
				CLKPWR_UARTCLKCTRL_UART3_EN, enable);
			break;

		case CLKPWR_DMA_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_dmaclk_ctrl,
				CLKPWR_DMACLKCTRL_CLK_EN, enable);
			break;

		case CLKPWR_SDRAMDDR_CLK:
			clkpwr_mask_and_set(&pClkPwr->clkpwr_sdramclk_ctrl,
				CLKPWR_SDRCLK_CLK_DIS, !enable);
			break;

		default:
			break;
	}
}
