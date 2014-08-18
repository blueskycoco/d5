#pragma once

#include <windows.h>
#include "wavedir.h"
#include "lpc32xx_clkpwr.h"
#include "lpc32xx_i2s.h"
#include "lpc32xx_gpio.h"

class I2SControl
{
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	I2SControl(void);

	// Destructor
	~I2SControl();

	// Returns initialized 'good' status after construction
	DWORD I2SInitGood(void);

	// Set direction for this instances
	void I2SSetDirection(I2S_CH_T ch);

	//********************************************************************
	// General audio setup functions
	//********************************************************************

	// Set source for stereo or mono
	DWORD I2SSetStereo(DWORD stereo);

	// Get stereo flag
	DWORD I2SGetStereo(void);

	// Flush the current FIFO, used prior to starting an audio stream to
	// prevent audio pops
	DWORD I2SFlushFIFO(void);

	// Pause or unpause stream
	DWORD I2SPause(DWORD pause);

	// Returns paused status of stream
	DWORD I2SIsPaused(void);

	// Mute (via I2S peripheral)
	DWORD I2SMute(DWORD mute);

	// Get mute status
	DWORD I2SIsMuted(void);

	// Diagnostic function for dumping current I2S registers
	void I2SDumpRegs(void);

protected:
	// Sets up data sample rate - both output and input are rate locked
	// so setting one sample rate will affect both input and output,
	// returns actual sample rate
	DWORD I2SSetSampleRate(DWORD wsrate,
		                   DWORD bits);

	// Get current sample rate - both output and input are rate locked
	DWORD I2SGetSampleRate(void);

	// Get the current sample size in bits
	DWORD I2SGetSampleSize(void);

private:
	// Number of instances
	static int m_I2Sinstances;

	// Safe initialization flag
	static DWORD m_I2Sgoodinit;

	// Allocated pointer to register regions
	static CLKPWR_REGS_T *m_I2SpClkPwrRegs;
	static I2S_REGS_T *m_I2SpI2SRegs;

	// Various saved values
	DWORD m_I2SStereoFlag;
	DWORD m_I2SSampleRate;
	DWORD m_I2SSampleSize;
	DWORD m_I2SPauseFlag;
	DWORD m_I2SMuteFlag;

	// Direction for this instance
	I2S_CH_T m_I2SchanDir;

	//********************************************************************
	// Miscellaneous functions
	//********************************************************************

	// Returns absolute value 
	int I2SABS(int v1,
		       int v2);

	// Determines bit rate clock to generate WS rate, returns actual bit
	// rate for the generated dividers
	DWORD I2SClkGetDivs(UINT32 rate,
		                UINT32 samplebits,
						UINT32 *divx,
						UINT32 *divy);
};
