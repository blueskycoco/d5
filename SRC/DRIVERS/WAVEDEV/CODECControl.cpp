
#include "wavemain.h"
#include "uda1380.h"
#include "bsp.h"

// IMPORTANT NOTE
// When programming the CODEC, the WS clock should be running to generate
// the base clock fo the CODEC WSPLL. Failure to run the WS clock will
// cause some CODEC operations to fail!

// Static driver data
int CODECControl::m_codecinstances              = 0;
DWORD CODECControl::m_codecgoodinit             = 0;
HANDLE CODECControl::m_codecdrvI2CCtl           = INVALID_HANDLE_VALUE;
UINT32 CODECControl::m_codecsavedVol            = 0xFFFFFFFF;
DWORD CODECControl::m_codecboostEnable          = 0;
UINT32 CODECControl::m_codecSavedBoostLevel     = 0;
DWORD CODECControl::m_codecmuteEnabled          = 0;
DWORD CODECControl::m_codecHeadPhoneEnabledFlag = 1;
DWORD CODECControl::m_codecLineInUsedFlag       = 1;
UINT32 CODECControl::m_codecGainADCPGA          = 0;
UINT16 CODECControl::m_codecGainADCVGA          = 0;
UINT32 CODECControl::m_codecVolDecimator        = 0;
DWORD CODECControl::m_codecDigitalMixerEnabled  = 1;

//********************************************************************
// Initial setup data for the I2S device (via I2C)
//********************************************************************
typedef struct
{
	UNS_8 reg;
	UNS_16 val;
} I2S_SETUP_T;
static I2S_SETUP_T i2sinitdata[] =
{
	// L3 reset
	{UDA1380_REG_L3,          0x0000},
	// CODEC ADC and DAC clock from WSPLL, all clocks enabled
	{UDA1380_REG_EVALCLK,     0x0F30},
	// I2S bus data I/O formats, use digital mixer for output
	// BCKO is slave
	{UDA1380_REG_I2S,         0x0000},
	// Enable all power for now
	{UDA1380_REG_PWRCTRL,     0xA5D0},
	// Full mixer analog input gain
	{UDA1380_REG_ANAMIX,      0x0000},
	// Enable headphone short circuit protection
	{UDA1380_REG_HEADAMP,     0x0202},
	// Full master volume
	{UDA1380_REG_MSTRVOL,     0x0000},
	// Enable full mixer volume on both channels
	{UDA1380_REG_MIXVOL,      0x0000},
	// Bass and treble boost set to flat
	{UDA1380_REG_MODEBBT,     0x0000},
	// Disable mute and de-emphasis
	{UDA1380_REG_MSTRMUTE,    0x0000},
	// Mixer off, other settings off
	{UDA1380_REG_MIXSDO,      0x0000},
	// ADC decimator volume to max
	{UDA1380_REG_DECVOL,      0x0000},
	// No PGA mute, full gain
	{UDA1380_REG_PGA,         0x0F0F},
	// Select line in and MIC, max MIC gain
	{UDA1380_REG_ADC,         0x0F02},
	// AGC
	{UDA1380_REG_AGC,         0x0000},
	// End of list
	{0xFF,                    0xFFFF}
};

//********************************************************************
// Initialization and de-init functions
//********************************************************************

// Constructor
CODECControl::CODECControl(void)
{
	// Assume output channel until changed
	m_codecchanDir = I2S_OUT_CH;

	m_codecinstances++;
	if (m_codecinstances == 1)
	{	
		m_codecgoodinit = 1;

		// Open I2C driver to communicate with I2S
		m_codecdrvI2CCtl = CreateFile(I2SI2SDEVNAME, (GENERIC_READ | GENERIC_WRITE),
			(FILE_SHARE_WRITE | FILE_SHARE_READ), NULL, OPEN_EXISTING,
			FILE_ATTRIBUTE_SYSTEM, 0);
		if (m_codecdrvI2CCtl == INVALID_HANDLE_VALUE)
		{
			RETAILMSG(1,
			    (TEXT("CODECContext: Error initializing I2C CODEC interface\r\n")));
			m_codecgoodinit = 0;
		}

		// Set static default levels for all instances
		m_codecsavedVol             = 0xFFFFFFFF;
		m_codecboostEnable          = 1;
		m_codecSavedBoostLevel      = 0;
		m_codecmuteEnabled          = 1;
		m_codecHeadPhoneEnabledFlag = 1;
		m_codecLineInUsedFlag       = 1;
		m_codecGainADCPGA           = 0;
		m_codecGainADCVGA           = 0;
		m_codecVolDecimator         = 0;
		m_codecDigitalMixerEnabled  = 1;
	}
}

// Destructor
CODECControl::~CODECControl(void)
{
	m_codecinstances--;
	if (m_codecinstances == 0)
	{
		// Delete I2C driver instance
		if (m_codecdrvI2CCtl != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_codecdrvI2CCtl);
			m_codecdrvI2CCtl = INVALID_HANDLE_VALUE;
		}

		m_codecgoodinit = 0;
	}
}

// Returns initialized 'good' status after construction
DWORD CODECControl::codecIsInitGood(void)
{
	return m_codecgoodinit;
}

// Setup CODEC default registers, this should be done once the I2S
// clock is running
DWORD CODECControl::codecSetup(void)
{
	int idx;
	DWORD goodSetup = 1;

	// Initialize CODEC via I2C interface
	idx = 0;

	while (i2sinitdata[idx].reg != 0xFF)
	{	
		if (codecWriteReg(i2sinitdata[idx].val, i2sinitdata[idx].reg) == 0)
		{
			goodSetup = 0;
		}

		idx++;
	}

	return goodSetup;
}

// Set direction for this instances
void CODECControl::codecSetDirection(I2S_CH_T ch)
{
	m_codecchanDir = ch;
}

//********************************************************************
// CODEC raw register read/write functions
//********************************************************************

// Write a value to a CODEC register
DWORD CODECControl::codecWriteReg(UINT16 val,
							      UINT8 reg)
{
	DWORD good;
	I2C_OUT_XFER_T xferout;
	I2C_IN_XFER_T xferin;
	// Setup transfer
	xferout.flags_buff [0] = (I2S_DEV_ADDR | I2C_WRITE | I2C_START_FLAG);
	xferout.flags_buff [1] = reg;

	// Start read operation with repeated start
	xferout.flags_buff [2] = ((val >> 8) & 0xFF);
	xferout.flags_buff [3] = (((val >> 0) & 0xFF) | I2C_STOP_FLAG);

	// Send 4 bytes, read 0 bytes
	xferout.tosend = 4;
	xferout.torecv = 0;

	good = codecI2CTransaction(&xferout, &xferin);
	if (good != 0)
	{
		if (xferin.ist != I2CST_COMPLETE)
		{
			good = 0;
		}
	}
	return good;
}

// Read a value from a register
DWORD CODECControl::codecReadReg(UINT16 *val,
							     UINT8 reg)
{
	DWORD good;
	I2C_OUT_XFER_T xferout;
	I2C_IN_XFER_T xferin;

	// Setup transfer
	xferout.flags_buff [0] = (I2S_DEV_ADDR | I2C_WRITE | I2C_START_FLAG);
	xferout.flags_buff [1] = reg;

	// Start read operation with repeated start
	xferout.flags_buff [2] = (I2S_DEV_ADDR | I2C_READ | I2C_START_FLAG);

	// Read 2 bytes, send 3 bytes
	xferout.tosend = 3;
	xferout.torecv = 2;

	good = codecI2CTransaction(&xferout, &xferin);
	if (good != 0)
	{
		if (xferin.ist != I2CST_COMPLETE)
		{
			good = 0;
		}
		else
		{
			*val = ((UINT16) xferin.buff[0] << 8) |
				((UINT16) xferin.buff[1]);
		}
	}

	return good;
}

//********************************************************************
// CODEC clock and power control functions
//********************************************************************

// Adjust the CODEC PLL to the pass sample rate
DWORD CODECControl::codecSetPLLRate(DWORD wsrate)
{
	UINT16 rmask, regval;
	DWORD setGood = 0;

	// Based on the WS rate, the PLL also needs to be adjusted
	if (wsrate < 12500)
	{
		rmask = EVALCLK_WSPLL_SEL6_12K;
	}
	else if (wsrate < 25000)
	{
		rmask = EVALCLK_WSPLL_SEL12_25K;
	}
	else if (wsrate < 50000)
	{
		rmask = EVALCLK_WSPLL_SEL25_50K;
	}
	else
	{
		rmask = EVALCLK_WSPLL_SEL50_100K;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_EVALCLK) != 0)
	{
		regval = regval & ~0x3;
		regval = regval | rmask;
		if (codecWriteReg(regval, UDA1380_REG_EVALCLK) != 0)
		{
			setGood = 1;
		}
	}

	return setGood;
}

// Power down/up the output or input side of the CODEC. This will enable
// power in the CODEC for output or input functionality. This should only
// be used when bringing up or down the CODEC
DWORD CODECControl::codecPowerUp(DWORD powerup)
{
	// TBD fix me
	// Not support yet, CODEC is always powered on
	return 1;
}

// Returns power up status for input or output channels
DWORD CODECControl::codecIsPoweredUp(void)
{
	// TBD fix me
	// Not support yet, CODEC is always powered on
	return 1;
}

// Enable clocking for the output or input side of the CODEC. This can
// be used to gate clocking when no audio is output or input to save
// power.
DWORD CODECControl::codecClockUp(DWORD clockup)
{
	// TBD fix me
	// Not support yet, CODEC is always clocked
	return 1;
}

// Returns clocking status for input or output channels
DWORD CODECControl::codecIsClockUp(void)
{
	// TBD fix me
	// Not support yet, CODEC is always clocked
	return 1;
}

//********************************************************************
// Sound control functions
//********************************************************************

// Return or set volume, accepts and returns a value beween 0 and 0xFFFF,
// 0 will mute the volume, High 16 bits=right, low 16 bits=left
DWORD CODECControl::codecSetMasterVolume(UINT32 newvol)
{
	UINT16 trunVolL, trunVolR, vol;
	DWORD status = 0;

	// Skip if no changes are needed
	if (m_codecsavedVol == newvol)
	{
		return 1;
	}

	trunVolL = 0xFFFF - (UINT16) ((newvol >> 0) & 0xFFFF);
	trunVolR = 0xFFFF - (UINT16) ((newvol >> 16) & 0xFFFF);

	// The I2S CODEC uses 8 bits for volume level for each channel (left
	// and right), so use just the upper 8 bits for the left and right
	// channels
	vol = ((trunVolL >> 8) & 0x00FF) | ((trunVolR >> 0) & 0xFF00);

	// Write to the volume register
	if (codecWriteReg(vol, UDA1380_REG_MSTRVOL) != 0)
	{
		status = 1;
		m_codecsavedVol = newvol;
	}

	return status;
}

// Return current master volume, High 16 bits=right, low 16 bits=left
UINT32 CODECControl::codecGetMasterVolume(void)
{
	return m_codecsavedVol;
}

// Bass and treble boost enable/disable
DWORD CODECControl::codecEnableBoost(DWORD enable)
{
	UINT16 regval;
	DWORD good = 0;

	// Skip if no changes are needed
	if (m_codecboostEnable == enable)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_MODEBBT) != 0)
	{
		if (enable != 0)
		{
			regval |= MODEBBT_BOOST_MASK;
		}
		else
		{
			regval &= ~MODEBBT_BOOST_MASK;
		}
		if (codecWriteReg(regval, UDA1380_REG_MODEBBT) != 0)
		{
			good = 1;
			m_codecboostEnable = enable;
		}
	}

	return good;
}

// Return boost enabled status flag
DWORD CODECControl::codecIsBoosted(void)
{
	return m_codecboostEnable;
}

// Set curent boost levels, basslvl must be between 0(no boost) and
// 0xF(max boost) and treblvl must be between 0(no boost) and
// 0x3(max boost)
DWORD CODECControl::codecSetBoostLevel(UINT16 basslvl,
		                               UINT16 treblvl)
{
	UINT32 testBoostLvl;
	UINT16 regval;
	DWORD good = 0;

	// Limit values
	basslvl &= ~0xF;
	treblvl &= ~0x3;

	// Skip if no changes are needed
	testBoostLvl = (((UINT32) basslvl) << 16) | ((UINT32) treblvl);;
	if (m_codecSavedBoostLevel == testBoostLvl)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_MODEBBT) != 0)
	{
		regval &= ~0x3F3F;
		regval |= (basslvl | (basslvl << 8) | (treblvl << 4) | (treblvl << 12));
		if (codecWriteReg(regval, UDA1380_REG_MODEBBT) != 0)
		{
			good = 1;
			m_codecSavedBoostLevel = testBoostLvl;
		}
	}

	return good;
}

// Returns current boost levels, High 16 bits=bass, low 16 bits=treble
UINT32 CODECControl::codecGetBoostLevel(void)
{
	return m_codecSavedBoostLevel;
}

// Enable and disable CODEC mute
DWORD CODECControl::codecMute(DWORD mute)
{
	UINT16 regval;
	DWORD good = 0;

	// Skip if no changes are needed
	if (m_codecmuteEnabled == mute)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_MSTRMUTE) != 0)
	{
		if (mute != 0)
		{
			// Mute output
			regval |= 0x4000;
		}
		else
		{
			// Unmute output
			regval &= ~0x4000;
		}
		if (codecWriteReg(regval, UDA1380_REG_MSTRMUTE) != 0)
		{
			good = 1;
			m_codecmuteEnabled = mute;
		}
	}

	return good;
}

// Returns current CODEC mute enabled status flag
DWORD CODECControl::codecIsMuted(void)
{
	return m_codecmuteEnabled;
}

//********************************************************************
// Miscellaneous control functions
//********************************************************************

// Select line input or MIC for audio input, if MIC is used, the LNA amp
// is selectable with the VGA settings
DWORD CODECControl::codecSelectLineIn(DWORD use_line_in)
{
	UINT16 regval;
	DWORD good = 0;

	// Skip if no changes are needed
	if (m_codecLineInUsedFlag == use_line_in)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_ADC) != 0)
	{
		if (use_line_in == 0)
		{
			// Disable line in, enable MIC input and DC filter
			regval &= ~0x0008;
			regval |= 0x0005;
		}
		else
		{
			// Disable MIC and filter, enable line in
			regval &= ~0x0007;
			regval |= 0x0008;
		}
		if (codecWriteReg(regval, UDA1380_REG_ADC) != 0)
		{
			good = 1;
			m_codecLineInUsedFlag = use_line_in;
		}
	}

	return good;
}

// Return line in selection (TRUE) or MIC selection (FALSE);
DWORD CODECControl::codecLineInUsed(void)
{
	return m_codecLineInUsedFlag;
}

// Function for disabling and enabling headphone output driver
DWORD CODECControl::codecHeadPhoneEnable(DWORD enable)
{
	UINT16 regval;
	DWORD good = 0;

	// Skip if no changes are needed
	if (m_codecHeadPhoneEnabledFlag == enable)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_PWRCTRL) != 0)
	{
		if (enable != 0)
		{
			regval = regval | PWR_PON_HP_EN;
		}
		else
		{
			regval = regval & ~PWR_PON_HP_EN;
		}
		if (codecWriteReg(regval, UDA1380_REG_PWRCTRL) != 0)
		{
			good = 1;
			m_codecHeadPhoneEnabledFlag = enable;
		}
	}

	return good;
}

// Return headphone driver enabled status
DWORD CODECControl::codecHeadPhoneEnabled(void)
{
	return m_codecHeadPhoneEnabledFlag;
}

//********************************************************************
// Mixer and gain control functions
//********************************************************************

// Select output from digital mixer or decimator. The Digital mixer
// is used to route input signals directly to the output in the CODEC
DWORD CODECControl::codecDigitalMixerEnable(DWORD enable)
{
	UINT16 regval;
	DWORD good = 0;

	// Skip if no changes are needed
	if (m_codecDigitalMixerEnabled == enable)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_I2S) != 0)
	{
		if (enable != 0)
		{
			regval = regval | 0x0040;
		}
		else
		{
			regval = regval & ~0x0040;;
		}
		if (codecWriteReg(regval, UDA1380_REG_I2S) != 0)
		{
			good = 1;
			m_codecDigitalMixerEnabled = enable;
		}
	}

	return good;
}

// Returns digital mixer enabled status flag
DWORD CODECControl::codecIsDigitalMixerEnabled(void)
{
	return m_codecDigitalMixerEnabled;
}

// Select PGA (programmable gain) for ADC (must be between 0 (0db gain)
// and 0x8 (24db gain))
DWORD CODECControl::codecSetADCPGA(UINT16 lgain,
			                       UINT16 rgain)
{
	UINT16 regval;
	UINT testGain;
	DWORD good = 0;

	// Limit values
	lgain &= ~0xF;
	rgain &= ~0xF;

	// Skip if no changes are needed
	testGain = (((UINT32) rgain) << 16) | ((UINT32) lgain);
	if (m_codecGainADCPGA == testGain)
	{
		return 1;
	}

	regval = (rgain << 8) | lgain;
	if (codecWriteReg(regval, UDA1380_REG_PGA) != 0)
	{
		good = 1;
		m_codecGainADCPGA = testGain;
	}

	return good;
}

// Get current PGA gain, High 16 bits=right, low 16 bits=left
UINT32 CODECControl::codecGetADCPGA(void)
{
	return m_codecGainADCPGA;
}

// Select VGA (variable gain) for ADC (must be between 0 (0db gain)
// and 0xF (30db gain) in 2db gain steps). Used with the MIC only.
DWORD CODECControl::codecSetADCVGA(UINT16 gain)
{
	UINT16 regval;
	DWORD good = 0;

	// Limit value
	gain &= ~0xF;

	// Skip if no changes are needed
	if (m_codecGainADCVGA == gain)
	{
		return 1;
	}

	// Read current value first
	if (codecReadReg(&regval, UDA1380_REG_ADC) != 9)
	{
		regval &= ~0x0F00;
		regval |= (gain << 8);
		if (codecWriteReg(regval, UDA1380_REG_ADC) != 9)
		{
			good = 1;
			m_codecGainADCVGA = gain;
		}
	}

	return good;
}

// Get current ADC VGA gain
UINT16 CODECControl::codecGetADCVGA(void)
{
	return m_codecGainADCVGA;
}

// Set decimator volume, must be a value between 0 (24db gain) and
// 0x81 (-63.5db gain), >0x81 is -inf
DWORD CODECControl::codecSetDecimatorVol(UINT16 lgain,
				                         UINT16 rgain)
{
	UINT16 regval;
	UINT32 testDec;
	DWORD good = 0;

	// Limit values
	lgain &= ~0xFF;
	rgain &= ~0xFF;

	// Skip if no changes are needed
	testDec = (((UINT32) rgain) << 16) | ((UINT32) lgain);
	if (m_codecVolDecimator == testDec)
	{
		return 1;
	}

	regval = (lgain << 8) | rgain;
	if (codecWriteReg(regval, UDA1380_REG_DECVOL) != 0)
	{
		good = 1;
		m_codecVolDecimator = testDec;
	}

	return good;
}

// Get current decimator volume, High 16 bits=right, low 16 bits=left
UINT32 CODECControl::codecGetDecimatorVol(void)
{
	return m_codecVolDecimator;
}

// Diagnostic function for dumping current CODEC registers
void CODECControl::codecDumpRegs(void)
{
	static UNS_8 regs[] = {UDA1380_REG_EVALCLK, UDA1380_REG_I2S,
		UDA1380_REG_PWRCTRL, UDA1380_REG_ANAMIX, UDA1380_REG_HEADAMP,
		UDA1380_REG_MSTRVOL, UDA1380_REG_MIXVOL, UDA1380_REG_MODEBBT,
		UDA1380_REG_MSTRMUTE, UDA1380_REG_MIXSDO, UDA1380_REG_DECVOL,
		UDA1380_REG_PGA, UDA1380_REG_ADC, UDA1380_REG_AGC, 0xFF};
	int idx;
	UINT16 val16;

	idx = 0;
	while (regs[idx] != 0xFF)
	{
		codecReadReg(&val16, regs[idx]);
		RETAILMSG(1,
		    (TEXT("CODECControl: Reg 0x%x = 0x%x\r\n"), regs[idx], val16));
		idx++;
	}
}

//********************************************************************
// I2C interface related functions
//********************************************************************

// Perform a transaction on the I2S device via I2C
DWORD CODECControl::codecI2CTransaction(I2C_OUT_XFER_T *pI2COut,
		                                I2C_IN_XFER_T *pI2CIn)
{
	DWORD bytesRead;
	DWORD good = 0;
	// RETAILMSG(1, (TEXT("codec i2c transaction!!\r\n")));
	// Continue if driver was previously initialized
	if (m_codecdrvI2CCtl != INVALID_HANDLE_VALUE)
	{
		good = DeviceIoControl(m_codecdrvI2CCtl, IOCTL_APP_I2CREQ, pI2COut,
			sizeof(I2C_OUT_XFER_T), pI2CIn, sizeof(I2C_IN_XFER_T),
			&bytesRead, NULL);
			
	}
	return good;
}

