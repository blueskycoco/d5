
#include "wavemain.h"
#include "bsp.h"

// Static driver data
int I2SControl::m_I2Sinstances = 0;
DWORD I2SControl::m_I2Sgoodinit = FALSE;
CLKPWR_REGS_T *I2SControl::m_I2SpClkPwrRegs = NULL;
I2S_REGS_T *I2SControl::m_I2SpI2SRegs = NULL;

//********************************************************************
// Initialization and de-init functions
//********************************************************************

// Constructor
I2SControl::I2SControl(void)
{
	PHYSICAL_ADDRESS pa;
	GPIO_REGS_T *pI2SpGPIORegs;

	// Assume output channel until changed
	m_I2SchanDir = I2S_OUT_CH;

	m_I2Sinstances++;
	if (m_I2Sinstances == 1)
	{	
		m_I2Sgoodinit = 1;

		// Map space for registers
		pa.HighPart = 0;
		pa.LowPart = CLK_PM_BASE;
		m_I2SpClkPwrRegs = (CLKPWR_REGS_T *) MmMapIoSpace(pa,
			sizeof (CLKPWR_REGS_T), FALSE);
		pa.HighPart = 0;
		pa.LowPart = I2S1_BASE;
		m_I2SpI2SRegs = (I2S_REGS_T *) MmMapIoSpace(pa,
			sizeof (I2S_REGS_T), FALSE);
		if ((m_I2SpClkPwrRegs == NULL) || (m_I2SpI2SRegs == NULL))
		{
			RETAILMSG(1,
			    (TEXT("I2SContext: Error mapping register space\r\n")));
			m_I2Sgoodinit = 0;
		}

		// Mux setup for I2S1
		pa.HighPart = 0;
		pa.LowPart = GPIO_BASE;
		pI2SpGPIORegs = (GPIO_REGS_T *) MmMapIoSpace(pa,
			sizeof (GPIO_REGS_T), FALSE);
		if ((pI2SpGPIORegs == NULL))
		{
			RETAILMSG(1,
			    (TEXT("I2SContext: Error mapping GPIO register space\r\n")));
			m_I2Sgoodinit = 0;
		}
		else
		{
			// Setup the I2S1 pin muxing and interface
			pI2SpGPIORegs->pio_mux2_clr = 0x1C; // I2S1TX_SDA, I2S1TX_CLK, I2S1TX_WS
			pI2SpGPIORegs->p0_mux_set   = 0x03; // I2S1RX_CLK, I2S1RX_WS
			pI2SpGPIORegs->p0_outp_set  = 0x01; // I2S1RX_SDA

			MmUnmapIoSpace((PVOID) pI2SpGPIORegs,
				sizeof (GPIO_REGS_T));
		}

		// Enable I2S clock, DMA, and lock TX/RX clocks
		if (m_I2SpClkPwrRegs != NULL)
		{
			m_I2SpClkPwrRegs->clkpwr_i2s_clk_ctrl |= (CLKPWR_I2SCTRL_I2SCLK1_EN |
				CLKPWR_I2SCTRL_I2S1_USE_DMA | CLKPWR_I2SCTRL_I2S1_TX_FOR_RX);
		}

		// Setup default I2S
		if (m_I2SpI2SRegs != NULL)
		{
			// Reset the I2S channels and FIFOs
			m_I2SpI2SRegs->i2s_dao |= I2S_RESET;
			m_I2SpI2SRegs->i2s_dai |= I2S_RESET;
			m_I2SpI2SRegs->i2s_dao &= ~I2S_RESET;
			m_I2SpI2SRegs->i2s_dai &= ~I2S_RESET;

			// Enable data output at 16 bits, stereo, 16 clock half period 
			m_I2SpI2SRegs->i2s_dao = (I2S_WW16 | I2S_WS_HP(I2S_WW16_HP));

			// Data input format at 16 bits, stereo, 16 clock half period 
			m_I2SpI2SRegs->i2s_dai = (I2S_WW16 | I2S_WS_HP(I2S_WW16_HP));

			// Enable DMA0 for RX channel with FIFO level of 4 full
			m_I2SpI2SRegs->i2s_dma0 = I2S_DMA0_RX_EN | I2S_DMA0_RX_DEPTH(4);

			// Enable DMA1 for RX channel with FIFO level of 4 full
			m_I2SpI2SRegs->i2s_dma1 = I2S_DMA1_TX_EN | I2S_DMA1_TX_DEPTH(4);

			// Enable clocks for I2S input and output at 11.025KHz
			I2SSetSampleRate(11025, 16);
		}
	}

	// Set default values
	m_I2SStereoFlag = 1;
	m_I2SSampleRate = 0;
	m_I2SSampleSize = 16;
	m_I2SPauseFlag = 0;

	// Setup default I2S clock rate for 11KHz at 16 bits/sample
	I2SSetSampleRate(11025, 16);
}

// Destructor
I2SControl::~I2SControl(void)
{
	m_I2Sinstances--;

	if (m_I2Sinstances == 0)
	{
		// Disable I2S clock, DMA, and lock TX/RX clocks
		if (m_I2SpClkPwrRegs != NULL)
		{
			m_I2SpClkPwrRegs->clkpwr_i2s_clk_ctrl &= ~(CLKPWR_I2SCTRL_I2SCLK1_EN |
				CLKPWR_I2SCTRL_I2S1_USE_DMA | CLKPWR_I2SCTRL_I2S1_TX_FOR_RX);

			// Unmap clock and power registers
			MmUnmapIoSpace((PVOID) m_I2SpClkPwrRegs,
				sizeof (CLKPWR_REGS_T));
			m_I2SpClkPwrRegs = NULL;
		}

		// Unmap I2S registers
		if (m_I2SpI2SRegs != NULL)
		{
			MmUnmapIoSpace((PVOID) m_I2SpI2SRegs,
				sizeof (I2S_REGS_T));
			m_I2SpI2SRegs = NULL;
		}

		m_I2Sgoodinit = 0;
	}
}

// Returns initialized 'good' status after construction
DWORD I2SControl::I2SInitGood(void)
{
	return m_I2Sgoodinit;
}

// Set direction for this instances
void I2SControl::I2SSetDirection(I2S_CH_T ch)
{
	m_I2SchanDir = ch;
}

//********************************************************************
// General audio setup functions
//********************************************************************

// Set source for stereo or mono
DWORD I2SControl::I2SSetStereo(DWORD stereo)
{
	UNS_32 regval;
	DWORD valid = 0;

	if (m_I2SchanDir == I2S_IN_CH)
	{
		regval = m_I2SpI2SRegs->i2s_dai;
		if (stereo != FALSE)
		{
			regval = regval & ~I2S_MONO;
		}
		else
		{
			regval = regval | I2S_MONO;
		}
		m_I2SpI2SRegs->i2s_dai = regval;
		m_I2SStereoFlag = stereo;
		valid = 1;
	}

	if (m_I2SchanDir == I2S_OUT_CH)
	{
		regval = m_I2SpI2SRegs->i2s_dao;
		if (stereo != FALSE)
		{
			regval = regval & ~I2S_MONO;
		}
		else
		{
			regval = regval | I2S_MONO;
		}
		m_I2SpI2SRegs->i2s_dao = regval;
		m_I2SStereoFlag = stereo;
		valid = 1;
	}

	return valid;
}

// Get stereo flag
DWORD I2SControl::I2SGetStereo(void)
{
	return m_I2SStereoFlag;
}

// Flush the current FIFO, used prior to starting an audio stream to
// prevent audio pops
DWORD I2SControl::I2SFlushFIFO(void)
{
	DWORD valid = 0;

	if (m_I2SchanDir == I2S_IN_CH)
	{
		m_I2SpI2SRegs->i2s_dai |= I2S_RESET;
		m_I2SpI2SRegs->i2s_dai &= ~I2S_RESET;
		valid = 1;
	}

	if (m_I2SchanDir == I2S_OUT_CH)
	{
		m_I2SpI2SRegs->i2s_dao |= I2S_RESET;
		m_I2SpI2SRegs->i2s_dao &= ~I2S_RESET;
		valid = 1;
	}

	return valid;
}

// Sets up data sample rate - both output and input are rate locked
// so setting one sample rate will affect both input and output,
// returns actual sample rate
DWORD I2SControl::I2SSetSampleRate(DWORD wsrate,
								   DWORD bits)
{
	UINT32 divx = 0, divy = 0, regval;
	DWORD setGood = 0;

	// Skip if no changes are needed
	if ((m_I2SSampleRate == wsrate) && (m_I2SSampleSize == bits))
	{
		return 1;
	}

	// Setup I2S sample rate for new clock rate
	setGood = I2SClkGetDivs(wsrate, bits, &divx, &divy);
	if (setGood != 0)
	{
		// Setup I2S dividers and sample size
		if (m_I2SchanDir == I2S_OUT_CH)
		{
			// Set new WS rate
			m_I2SpI2SRegs->i2s_tx_rate = ((divx << 8) | divy);

			// Set new data width
			regval = m_I2SpI2SRegs->i2s_dao;
			regval &= ~I2S_WS_HP(WSMASK_HP);
			if (bits == 8)
			{
				regval |= I2S_WS_HP(I2S_WW8_HP);
			}
			else
			{
				regval |= I2S_WS_HP(I2S_WW16_HP);
			}
			m_I2SpI2SRegs->i2s_dao = regval;
		}
		else
		{
			// Set new WS rate
			m_I2SpI2SRegs->i2s_rx_rate = ((divx << 8) | divy);

			// Set new data width
			regval = m_I2SpI2SRegs->i2s_dai;
			regval &= ~I2S_WS_HP(WSMASK_HP);
			if (bits == 8)
			{
				regval |= I2S_WS_HP(I2S_WW8_HP);
			}
			else
			{
				regval |= I2S_WS_HP(I2S_WW16_HP);
			}
			m_I2SpI2SRegs->i2s_dai = regval;
		}

		// Save rate and sample size
		m_I2SSampleRate = wsrate;
		m_I2SSampleSize = bits;

		// Compute actual frequency for return
		setGood = setGood / (m_I2SSampleSize << 2);
	}

	// If the rate is 0, a possible frequency couldn't be found
	if (setGood == 0)
	{
        RETAILMSG(1, 
            (TEXT("I2SControl: Cannot set sample rate to %d Hz.\r\n"),
			wsrate));
	}

	return setGood;
}

// Get current sample rate - both output and input are rate locked
DWORD I2SControl::I2SGetSampleRate(void)
{
	// No differentiation among input and output for sample rate
	return m_I2SSampleRate;
}

// Get the current sample size in bits
DWORD I2SControl::I2SGetSampleSize(void)
{
	return m_I2SSampleSize;
}

// Pause or unpause stream
DWORD I2SControl::I2SPause(DWORD pause)
{
	DWORD valid = 0;

	if (m_I2SchanDir == I2S_IN_CH)
	{
		if (pause == 0)
		{
			m_I2SpI2SRegs->i2s_dai &= ~I2S_STOP;
		}
		else
		{
			// Pause input channel by setting STOP bit in I2S block
			m_I2SpI2SRegs->i2s_dai |= I2S_STOP;
		}

		m_I2SPauseFlag = pause;
		valid = 1;
	}

	if (m_I2SchanDir == I2S_OUT_CH)
	{
		if (pause == 0)
		{
			m_I2SpI2SRegs->i2s_dao &= ~I2S_STOP;
		}
		else
		{
			// Pause input channel by setting STOP bit in I2S block
			m_I2SpI2SRegs->i2s_dao |= I2S_STOP;
		}

		m_I2SPauseFlag = pause;
		valid = 1;
	}

	return valid;
}

// Returns paused status of stream
DWORD I2SControl::I2SIsPaused(void)
{
	return m_I2SPauseFlag;
}

// Mute (via I2S peripheral)
DWORD I2SControl::I2SMute(DWORD mute)
{
	DWORD valid = 0;

	if (m_I2SchanDir == I2S_IN_CH)
	{
		if (mute == 0)
		{
			m_I2SpI2SRegs->i2s_dai &= ~I2S_MUTE;
		}
		else
		{
			// Mute input channel by setting MUTE bit in I2S block
			m_I2SpI2SRegs->i2s_dai |= I2S_MUTE;
		}

		m_I2SMuteFlag = mute;
		valid = 1;
	}

	if (m_I2SchanDir == I2S_OUT_CH)
	{
		if (mute == 0)
		{
			m_I2SpI2SRegs->i2s_dao &= ~I2S_MUTE;
		}
		else
		{
			// Mute input channel by setting MUTE bit in I2S block
			m_I2SpI2SRegs->i2s_dao |= I2S_MUTE;
		}

		m_I2SMuteFlag = mute;
		valid = 1;
	}

	return valid;
}

// Get mute status
DWORD I2SControl::I2SIsMuted(void)
{
	return m_I2SMuteFlag;
}

// Diagnostic function for dumping current I2S registers
void I2SControl::I2SDumpRegs(void)
{
	int idx;
	UINT32 *regptr = (UINT32 *) m_I2SpI2SRegs;

	RETAILMSG(1,
	    (TEXT("I2SContext: Dumping I2S registers\r\n")));

	for (idx = 0; idx < 10; idx++)
	{
		RETAILMSG(1,
		    (TEXT("I2SContext: Reg 0x%x = 0x%x\r\n"), regptr, *regptr));
		regptr++;
	}
}

//********************************************************************
// Miscellaneous functions
//********************************************************************

// Returns absolute value 
int I2SControl::I2SABS(int v1,
		               int v2)
{
    if (v1 > v2)
	{
	    return v1 - v2;
	}

    return v2 - v1;
}

// Determines bit rate clock to generate WS rate, returns actual bit
// rate for the generated dividers
DWORD I2SControl::I2SClkGetDivs(UINT32 rate,
								UINT32 samplebits,
								UINT32 *divx,
								UINT32 *divy)
{
	UINT32 clk;
	DWORD bytesret;
	UINT64 baseclk, savedbitcklrate, diff, clkrate;
	int idxx, idyy;

	// Get current HCLK rate for basis of clock divider computation
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &clk,
		sizeof (clk), (LPDWORD) &bytesret) == FALSE)
	{
		// Cannot get clock
        RETAILMSG(1, 
            (TEXT("I2SControl: Error getting I2C base clock rate.\r\n")));
		return 0;
	}
	baseclk = (UINT64) clk;

	// Adjust rate for number of channels and bits/sample
	rate = rate * samplebits * 2; // Always 2 channels
	rate = rate << 1; // Offset for divider in clock output

	// Find the best divider
	*divx = *divy = 0;
	savedbitcklrate = 0;
	diff = 0xFFFFFFFF;
	for (idxx = 1; idxx < 0xFF; idxx++) {
		for (idyy = idxx; idyy < 0xFF; idyy++) {
			clkrate = (baseclk * (UNS_64) idxx) / (UNS_64) idyy;
			if (I2SABS((UNS_32) clkrate, rate) < diff) {
				diff = I2SABS((UNS_32) clkrate, rate);
				savedbitcklrate = clkrate;
				*divx = (UNS_32) idxx;
				*divy = (UNS_32) idyy;
			}
		}
	}

	return (DWORD) savedbitcklrate;
}
