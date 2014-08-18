
#pragma once

#include <windows.h>
#include "wavedir.h"
#include "phy3250_board.h"
#include "drv_ioctl_funcs.h"

// I2C device filename that contains the interface to I2S
#define I2SI2SDEVNAME (L"IIC0:")

// I2C address for I2S device
#define I2S_DEV_ADDR (PHY_I2C_I2S<<1)

class CODECControl
{
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	CODECControl(void);

	// Destructor
	~CODECControl();

	// Returns initialized 'good' status after construction
	DWORD codecIsInitGood(void);

	// Setup CODEC default registers, this should be done once the I2S
	// clock is running
	DWORD codecSetup(void);

	// Set direction for this instances
	void codecSetDirection(I2S_CH_T ch);

	//********************************************************************
	// CODEC raw register read/write functions
	//********************************************************************

	// Write a value to a CODEC register
	DWORD codecWriteReg(UINT16 val,
		                UINT8 reg);

	// Read a value from a register
	DWORD codecReadReg(UINT16 *val,
		               UINT8 reg);

	//********************************************************************
	// CODEC clock and power control functions
	//********************************************************************

	// Power down/up the output or input side of the CODEC. This will enable
	// power in the CODEC for output or input functionality. This should only
	// be used when bringing up or down the CODEC
	DWORD codecPowerUp(DWORD powerup);

	// Returns power up status for input or output channels
	DWORD codecIsPoweredUp(void);

	// Enable clocking for the output or input side of the CODEC. This can
	// be used to gate clocking when no audio is output or input to save
	// power.
	DWORD codecClockUp(DWORD clockup);

	// Returns clocking status for input or output channels
	DWORD codecIsClockUp(void);

	//********************************************************************
	// Sound control functions
	//********************************************************************

	// Return or set volume, accepts and returns a value beween 0 and 0xFFFF,
	// 0 will mute the volume, High 16 bits=right, low 16 bits=left
	DWORD codecSetMasterVolume(UINT32 newvol);

	// Return current master volume, High 16 bits=right, low 16 bits=left
	UINT32 codecGetMasterVolume(void);

	// Bass and treble boost enable/disable
	DWORD codecEnableBoost(DWORD enable);

	// Return boost enabled status flag
	DWORD codecIsBoosted(void);

	// Set curent boost levels, basslvl must be between 0(no boost) and
	// 0xF(max boost) and treblvl must be between 0(no boost) and
	// 0x3(max boost)
	DWORD codecSetBoostLevel(UINT16 basslvl,
		                     UINT16 treblvl);

	// Returns current boost levels, High 16 bits=bass, low 16 bits=treble
	UINT32 codecGetBoostLevel(void);

	// Enable and disableCODEC  mute
	DWORD codecMute(DWORD mute);

	// Returns current CODEC mute enabled status flag
	DWORD codecIsMuted(void);

	//********************************************************************
	// Miscellaneous control functions
	//********************************************************************

	// Select line input or MIC for audio input, if MIC is used, the LNA
	// amp is selectable with the VGA settings
	DWORD codecSelectLineIn(DWORD use_line_in);

	// Return line in selection (TRUE) or MIC selection (FALSE);
	DWORD codecLineInUsed(void);

	// Function for disabling and enabling headphone output driver
	DWORD codecHeadPhoneEnable(DWORD enable);

	// Return headphone driver enabled status
	DWORD codecHeadPhoneEnabled(void);

	//********************************************************************
	// Mixer and gain control functions
	//********************************************************************

	// Select output from digital mixer or decimator. The Digital mixer
	// is used to route input signals directly to the output in the CODEC
	DWORD codecDigitalMixerEnable(DWORD enable);

	// Returns digital mixer enabled status flag
	DWORD codecIsDigitalMixerEnabled(void);

	// Select PGA (programmable gain) for ADC (must be between 0 (0db gain)
	// and 0x8 (24db gain))
	DWORD codecSetADCPGA(UINT16 lgain,
				         UINT16 rgain);

	// Get current PGA gain, High 16 bits=right, low 16 bits=left
	UINT32 codecGetADCPGA(void);

	// Select VGA (variable gain) for ADC (must be between 0 (0db gain)
	// and 0xF (30db gain) in 2db gain steps). Used with the MIC only.
	DWORD codecSetADCVGA(UINT16 gain);

	// Get current ADC VGA gain
	UINT16 codecGetADCVGA(void);

	// Set decimator volume, must be a value between 0 (24db gain) and
	// 0x81 (-63.5db gain), >0x81 is -inf
	DWORD codecSetDecimatorVol(UINT16 lgain,
				               UINT16 rgain);

	// Get current decimator volume, High 16 bits=right, low 16 bits=left
	UINT32 codecGetDecimatorVol(void);

	// Diagnostic function for dumping current CODEC registers
	void codecDumpRegs(void);

protected:
	// Adjust the CODEC PLL to the pass sample rate
	DWORD codecSetPLLRate(DWORD wsrate);

private:
	// Number of instances
	static int m_codecinstances;

	// Safe initialization flag
	static DWORD m_codecgoodinit;

	// Drive handle for I2C
	static HANDLE m_codecdrvI2CCtl;

	// Various saved values
	static UINT32 m_codecsavedVol;             // Saved volume levels
	static DWORD m_codecboostEnable;           // Boost enabled flag
	static UINT32 m_codecSavedBoostLevel;      // Saved boost levels
	static DWORD m_codecmuteEnabled;           // Output mute flag
	static DWORD m_codecHeadPhoneEnabledFlag;  // Headphone driver enabled flag
	static DWORD m_codecLineInUsedFlag;        // Line in (TRUE or MIC (FALSE) flag
	static UINT32 m_codecGainADCPGA;           // Saved programmable gain (line in)
	static UINT16 m_codecGainADCVGA;           // Save variable gain (MIC)
	static UINT32 m_codecVolDecimator;         // Saved decimator volume
	static DWORD m_codecDigitalMixerEnabled;   // Digital mixer enabled flag

	// This instances' direction
	I2S_CH_T m_codecchanDir;

	//********************************************************************
	// I2C interface related functions
	//********************************************************************

	// Perform a transaction on the I2S device via I2C
	DWORD codecI2CTransaction(I2C_OUT_XFER_T *pI2COut,
		                      I2C_IN_XFER_T *pI2CIn);
};
