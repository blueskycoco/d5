
#pragma once

#include "wavemain.h"

class HardwareControl : public CODECControl, public I2SControl
{
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	HardwareControl(I2S_CH_T ch);

	// Destructor
	~HardwareControl();

	// Returns initialized 'good' status after construction
	DWORD hwInitGood(void);

	//********************************************************************
	// Hardware query functions
	//********************************************************************

	// Get device capabilities
	DWORD hwWaveDevGetCaps(PVOID pCaps,
					       UINT wSize);

	// Get direction of this channel
	I2S_CH_T hwGetChanDir(void);

	//********************************************************************
	// General audio setup functions
	//********************************************************************

	// Adjust the current sample rate
	DWORD hwAdjustSampleRate(DWORD newRate,
		                     DWORD SampleSize);

	// Get current programmed sample rate
	DWORD hwGetSampleRate(void);

	// Get actual sample rate (from hardware)
	DWORD hwGetActualSampleRate(void);

	// Get the current sample size (in bits)
	DWORD hwGetSampleSize(void);

	//********************************************************************
	// Power control and query functions
	//********************************************************************

	void GetPowerCapabilities(void * pBuf);
	CEDEVICE_POWER_STATE GetPowerState(void);
	BOOL QueryPowerState(CEDEVICE_POWER_STATE dwState);
	DWORD SetPowerState(CEDEVICE_POWER_STATE dwState);

private:
	// Number of instances
	static int m_hwinstances;

	// Safe initialization flag
	static DWORD m_hwgoodinit;

	// Various saved values
	DWORD m_hwActualSampleRate;

	// Direction for this instance
	I2S_CH_T m_hwchanDir;

	// Power capabilities
	static const POWER_CAPABILITIES g_PowerCaps;
    CEDEVICE_POWER_STATE m_dwPowerState;
};
