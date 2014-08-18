
#include "HardwareControl.h"
#include "bsp.h"

// Static driver data
int HardwareControl::m_hwinstances  = 0;
DWORD HardwareControl::m_hwgoodinit = 0;

const POWER_CAPABILITIES HardwareControl::g_PowerCaps = 
{
    // DeviceDx:    Supported power states
    DX_MASK(D0) | DX_MASK(D4),

    0,              // WakeFromDx:
    0,              // InrushDx:    No inrush of power

    {               // Power: Maximum milliwatts in each state
        0x00000001, //        D0 = 0
        0x00000001, //        D1 = 0
        0x00000001, //        D2 = 0
        0x00000001, //        D3 = 0
        0x00000001  //        D4 = 0 (off)
    },

    {               // Latency
        0x00000000, //        D0 = 0
        0x00000000, //        D1 = 0
        0x00000000, //        D2 = 0
        0x00000000, //        D3 = 0
        0x00000000  //        D4 = 0
    },
    0,                    // Flags: None
};

//********************************************************************
// Initialization and de-init functions
//********************************************************************

// Constructor
HardwareControl::HardwareControl(I2S_CH_T ch)
{
	// Save this instances channel direction
	m_hwchanDir = ch;
	m_hwinstances++;
	if (m_hwinstances == 1)
	{	
		m_hwgoodinit = 1;
		// Verify new instances of CODEC and I2S classes
		if (I2SInitGood() == 0)
		{
			RETAILMSG(1,
			    (TEXT("HardwareControl: Error initializing I2SContext class\r\n")));
			m_hwgoodinit = 0;
		}

		// Check codec
		if (codecIsInitGood() == 0)
		{
			RETAILMSG(1,
			    (TEXT("HardwareControl: Error initializing CODECContext class\r\n")));
			m_hwgoodinit = 0;
		}
		else
		{
			// Setup CODEC registers
			m_hwgoodinit = codecSetup();
		}
	}

	// Set channel direction for other instances
	
	codecSetDirection(m_hwchanDir);
	
	I2SSetDirection(m_hwchanDir);
	
	
	// Flush I2S FIFO
	I2SFlushFIFO();

	// Channel not paused or muted
	codecMute(0);
	I2SPause(0);

	// Set default for stereo, 11.025KHz, 16-bit audio
	RETAILMSG(1, (TEXT("I2S set Stereo\r\n")));
	I2SSetStereo(1);
	hwAdjustSampleRate(11025, 16);

	if (m_hwchanDir == I2S_OUT_CH)
	{
		// Enable headphone amp
		codecHeadPhoneEnable(1);

		// Enable bbass and treble boost
		codecEnableBoost(1);
		codecSetBoostLevel(0xF, 0x3);
	}

	// Initial power state
    m_dwPowerState = D0;
}

// Destructor
HardwareControl::~HardwareControl(void)
{
	m_hwinstances--;
	if (m_hwinstances == 0)
	{
		m_hwgoodinit = 0;
	}
}

// Returns initialized 'good' status after construction
DWORD HardwareControl::hwInitGood(void)
{
	return m_hwgoodinit;
}

//********************************************************************
// Hardware query functions
//********************************************************************

// Get device capabilities
DWORD HardwareControl::hwWaveDevGetCaps(PVOID pCaps,
							            UINT wSize)
{
    PWAVEOUTCAPS pwoc = (PWAVEOUTCAPS) pCaps;
    PWAVEINCAPS pwic = (PWAVEINCAPS) pCaps;

	if (pCaps != NULL)
	{
		if (m_hwchanDir == I2S_IN_CH)
		{
			return MMSYSERR_ERROR; // TBD fix me with input
			// Support all formats for generic wave output
		    pwic->wMid = MM_MICROSOFT;  
			pwic->wPid = 23;
	        pwic->vDriverVersion = 0x0001;
		    wsprintf (pwic->szPname, TEXT("UDA1380 I2S Stereo audio in"));
			pwic->dwFormats = (
				WAVE_FORMAT_1M08 | WAVE_FORMAT_1M16 | WAVE_FORMAT_1S08 |
	            WAVE_FORMAT_1S16 | WAVE_FORMAT_2M08 | WAVE_FORMAT_2M16 |
		        WAVE_FORMAT_2S08 | WAVE_FORMAT_2S16 | WAVE_FORMAT_4M08 |
			    WAVE_FORMAT_4M16 | WAVE_FORMAT_4S08 | WAVE_FORMAT_4S16);
	        pwic->wChannels = 2;
		}
		else
		{
			// Support all formats for generic wave output
		    pwoc->wMid = MM_MICROSOFT;  
			pwoc->wPid = 24;
	        pwoc->vDriverVersion = 0x0001;
		    wsprintf (pwoc->szPname, TEXT("UDA1380 I2S Stereo audio out"));
			pwoc->dwFormats = (
				WAVE_FORMAT_1M08 | WAVE_FORMAT_1M16 | WAVE_FORMAT_1S08 |
	            WAVE_FORMAT_1S16 | WAVE_FORMAT_2M08 | WAVE_FORMAT_2M16 |
		        WAVE_FORMAT_2S08 | WAVE_FORMAT_2S16 | WAVE_FORMAT_4M08 |
			    WAVE_FORMAT_4M16 | WAVE_FORMAT_4S08 | WAVE_FORMAT_4S16);
	        pwoc->wChannels = 2;
		    pwoc->dwSupport = WAVECAPS_LRVOLUME;
		}
	}

    return MMSYSERR_NOERROR;
}

// Get direction of this channel
I2S_CH_T HardwareControl::hwGetChanDir(void)
{
	return m_hwchanDir;
}

//********************************************************************
// General audio setup functions
//********************************************************************

// Adjust the current sample rate
DWORD HardwareControl::hwAdjustSampleRate(DWORD newRate,
										  DWORD SampleSize)
{
	DWORD pst, flag = 0;

	// Get paused status
	pst = I2SIsPaused();

	// Pause channel (allows WS to run with no data) - WS needs to run
	// when changing the PLL clock to prevent lockup issues with the
	// CODEC PLL
	I2SPause(0);

	// Adjust rate for WS
	newRate = newRate << 1;

	m_hwActualSampleRate = I2SSetSampleRate(newRate, SampleSize);
	m_hwActualSampleRate = newRate;
	if (m_hwActualSampleRate != 0)
	{
		flag = codecSetPLLRate(m_hwActualSampleRate);
	}

	if ((m_hwActualSampleRate == 0) || (flag == 0))
	{
		RETAILMSG(1,
		    (TEXT("HardwareControl: Error setting new sample rate\r\n")));
	}

	// Restore original paused status
	I2SPause(pst);

	return m_hwActualSampleRate;
}

// Get current programmed sample rate
DWORD HardwareControl::hwGetSampleRate(void)
{
	return I2SGetSampleRate();
}

// Get actual sample rate (from hardware)
DWORD HardwareControl::hwGetActualSampleRate(void)
{
	return m_hwActualSampleRate;
}

// Get the current sample size (in bits)
DWORD HardwareControl::hwGetSampleSize(void)
{
	return I2SGetSampleSize();
}

//********************************************************************
// Power control and query functions
//********************************************************************

void HardwareControl::GetPowerCapabilities(void *pBuf)
{
    memcpy(pBuf, &g_PowerCaps, sizeof(POWER_CAPABILITIES));
}

CEDEVICE_POWER_STATE HardwareControl::GetPowerState(void)
{
    return m_dwPowerState;
}

BOOL HardwareControl::QueryPowerState(CEDEVICE_POWER_STATE dwState)
{
    return VALID_DX(dwState);
}

DWORD HardwareControl::SetPowerState(CEDEVICE_POWER_STATE dwState)
{
	// Not implemented yet
    switch (dwState)
    {
    case D0:
        m_dwPowerState = D0;
        break;

    case D4:
        m_dwPowerState = D4;
        break;

    default:
        return MMSYSERR_INVALPARAM;
    }

    return MMSYSERR_NOERROR;
}
