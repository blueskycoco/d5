///********************************************************************
/// Software that is described herein is for illustrative purposes only  
/// which provides customers with programming information regarding the  
/// products. This software is supplied "AS IS" without any warranties.  
/// NXP Semiconductors assumes no responsibility or liability for the 
/// use of the software, conveys no license or title under any patent, 
/// copyright, or mask work right to the product. NXP Semiconductors 
/// reserves the right to make changes in the software without 
/// notification. NXP Semiconductors also make no representation or 
/// warranty that such application will be suitable for the specified 
/// use without further testing or modification. 
///********************************************************************
//
// sdpdd.c
//
// This file implements SD PDD driver used with EBOOT
//

#include <sdpdd.h>
#include "boot_utilities.h"
#include <oal.h>
#include "lpc32xx_sdcard.h"
#include "clkpwr_support.h"
#include "lpc32xx_gpio.h"

#define OCR_SUPPORTED_VOLTAGES_MMC 0x00ff8000
#define OCR_SUPPORTED_VOLTAGES_SD 0x00ff8000

static UNS_32 secbuff [512 / 4];
static BOOL datarecv;

//------------------------------------------------------------------------------
//
// sd_set_clk
//
// Set SD card clock
//
static void sd_set_clk(UNS_32 clk)
{
	UNS_32 sd_div, sd_clk, tmp = 0;
	volatile UNS_32 tmpv;
	SDCARD_REGS_T *pSDCARDRegs;

	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);

	// Get current SD clock rate
	sd_clk = clkpwr_get_base_clock_rate(CLKPWR_ARM_CLK);

	// Find best divider to generate target clock rate
    sd_div = 0;
	tmp = 0;
    while ((sd_clk / (2 * (sd_div + 1))) >= clk)
    {
    	sd_div++;
    }
    if (sd_div > SD_CLKDIV_MASK)
    {
    	// Limit to maximum supported divider
    	sd_div = SD_CLKDIV_MASK;
    }
    else if (sd_div == 0)
    {
    	// May have to use the clock bypass instead
    	if (clk >= sd_clk)
    	{
    		tmp = SD_SDCLK_BYPASS;
    	}
    }

	tmpv = pSDCARDRegs->sd_clock;
	tmpv &= (SD_SDCLK_BYPASS | SD_CLKDIV_MASK);
	tmpv |= (tmp | sd_div);
}

//------------------------------------------------------------------------------
//
// sd_set_ctrl_params
//
// Set SD card controller general parameters
//
static void sd_set_ctrl_params(UNS_32 clk)
{
	SDCARD_REGS_T *pSDCARDRegs;

	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);

	pSDCARDRegs->sd_clock = (SD_SDCLK_EN | SD_CLKDIV_MASK);

	sd_set_clk(clk);
}

//------------------------------------------------------------------------------
//
// PDD_SDInitializeHardware
//
// Initializes SD/MMC hardware
//
BOOL PDD_SDInitializeHardware() {
	CLKPWR_REGS_T *pClkpwr;
	SDCARD_REGS_T *pSDCARDRegs;
	GPIO_REGS_T *pGPIORegs;

	pClkpwr = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);
	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);
	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO_BASE, FALSE);

	// Setup pullups
	pClkpwr->clkpwr_ms_ctrl = (CLKPWR_MSCARD_MSDIO_PU_EN |
		CLKPWR_MSCARD_SDCARD_DIV(1));

	// Enable power to the SD card
	pGPIORegs->pio_outp_set = OUTP_STATE_GPO(11);
	OALStall(5000);

	// Enable open drain mode and power on mode
    pSDCARDRegs->sd_power = SD_POWER_ON_MODE;
	
	// Enable SD clock at IP clock divided by 2
    clkpwr_clk_en_dis(CLKPWR_MSCARD_CLK, 1);
    pSDCARDRegs->sd_clock = SD_SDCLK_EN;

	// Setup SDCARD interface and controller. Enable SD
	// controller and pullups, setup clock rate to 390KHz,
	// disable wide bus, power off mode
	sd_set_ctrl_params(390000);

	// Disable command and data state machines
	pSDCARDRegs->sd_cmd = 0;
	pSDCARDRegs->sd_dctrl = 0;

    // Disable all interrupts
    pSDCARDRegs->sd_mask0 = 0;
    pSDCARDRegs->sd_mask1 = 0;

    // Clear any latched statuses
    pSDCARDRegs->sd_clear = 0x7FF;

	// Set data transfer defaults for the interface
	pSDCARDRegs->sd_dctrl = SD_BLKSIZE_512BYTES;
	pSDCARDRegs->sd_dtimer = 0xFFFFFFFF;

	datarecv = FALSE;

	return TRUE;
}

//------------------------------------------------------------------------------
//
// PDD_SDSendCommand
//
// Sends a command to the SD/MMC card
//
BOOL PDD_SDSendCommand(UINT8 cmd, UINT32 arg, RESP_TYPE resp, CMD_TYPE type, BOOL read) {
	UNS_32 cmdsm, datasm, errmask0, mask0, errmask1, mask1;
	INT32 idx;
	SDCARD_REGS_T *pSDCARDRegs;

	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);

	// Disable command and data state machines
	pSDCARDRegs->sd_cmd &= ~SD_CPST_EN;
	pSDCARDRegs->sd_dctrl &= ~SD_DATATRANSFER_EN;

	// Clear any pending statuses
	pSDCARDRegs->sd_clear = 0x7FF;

	// Setup SDMMC argument
	pSDCARDRegs->sd_arg = arg;

	// Setup command state machine mask
	if (resp != RESP_TYPE_NONE) {
#if 0
		// SD cards return a CMD CRC error on first APP cmd, so
		// this is a workaround for the issue. This only occurs on the
		// first APP command on an SD card either in push/pull or open
		// drain modes
		errmask0 = (SD_CMD_TIMEOUT | SD_CMD_CRC_FAIL);
		mask0 = (errmask0 | SD_CMD_RESP_RECEIVED);
#else
		errmask0 = (SD_CMD_TIMEOUT);
		mask0 = (errmask0 | SD_CMD_RESP_RECEIVED | SD_CMD_CRC_FAIL);
#endif
	}
	else
	{
		errmask0 = 0;
		mask0 = SD_CMD_SENT;
	}
    pSDCARDRegs->sd_mask0 = mask0;

	// Setup command state machine
	cmdsm = cmd | SD_CPST_EN;
	switch (resp)
	{
		case RESP_TYPE_NONE:
			// No response expected
			break;

		case RESP_TYPE_R2:
			cmdsm |= (SD_RESPONSE | SD_LONGRESP_EN);
			break;

		default:
			cmdsm |= SD_RESPONSE;
			break;
	}

	// Setup data state machine if this is a data transfer
	if (read)
	{
		// 512 bytes to transfer
		pSDCARDRegs->sd_dlen = 512;

		// Setup data state machine control register
		datasm = (SD_BLKSIZE_512BYTES | SD_DIR_FROMCARD |
			SD_DATATRANSFER_EN);

		// Data masks
		errmask1 = (SD_DATA_CRC_FAIL | SD_DATA_TIMEOUT |
			SD_FIFO_RXDATA_OFLOW);
		mask1 = (errmask1 | SD_DATABLK_END);
	}
	else
	{
		datasm = 0;
		errmask1 = 0;
		mask1 = 0;
	}
	datarecv = FALSE;

	// Start data and command state machines
	pSDCARDRegs->sd_cmd = cmdsm;
	pSDCARDRegs->sd_dctrl = datasm;

	// Wait for command state machine to finish by either getting
	// a response, command timeout, or command sent status based
	while ((pSDCARDRegs->sd_status & mask0) == 0);

	// Stop command state machine
	pSDCARDRegs->sd_cmd &= ~SD_CPST_EN;

	// Clear command mask
	pSDCARDRegs->sd_mask0 = 0;

	// Check command status for errors
	if (((pSDCARDRegs->sd_status & errmask0) != 0) && (cmd != 1))
	{
		// Command response timeout or CRC error
		OALMSG(1, (TEXT(
			"PDD_SDSendCommand: Error condition on command(cmd=%d, stat=0x%08x)\r\n"),
			cmd, pSDCARDRegs->sd_status));

		// Stop data state machine
		pSDCARDRegs->sd_dctrl &= ~SD_DATATRANSFER_EN;

		return FALSE;
	}

	if (read)
	{
		// Wait for data state machine
		idx = 0;
		while ((pSDCARDRegs->sd_status & mask1) == 0)
		{
			// Is data in the receive FIFO?
			if ((pSDCARDRegs->sd_status & SD_FIFO_RXDATA_HFULL) != 0)
			{
				memcpy((void *) &secbuff [idx], (void *) &pSDCARDRegs->sd_fifo[0], 32);
				idx += 8;
			}
		}

		// Drain rest of the FIFO
		while ((pSDCARDRegs->sd_status & SD_FIFO_RXDATA_AVAIL) != 0)
		{
			secbuff [idx] = pSDCARDRegs->sd_fifo [0];
			idx++;
		}

		// Stop data state machine
		pSDCARDRegs->sd_dctrl &= ~SD_DATATRANSFER_EN;

		// Clear command mask
		pSDCARDRegs->sd_mask1 = 0;

		// Has data transfer completed without errors?
		if ((pSDCARDRegs->sd_status & errmask1) != 0)
		{
			OALMSG(1, (TEXT(
				"PDD_SDSendCommand: Error condition on data(cmd=%d, stat=0x%08x)\r\n"),
				cmd, pSDCARDRegs->sd_status));
			return FALSE;
		}
		else
		{
			datarecv = TRUE;
		}
	}

	return TRUE;	
}

//------------------------------------------------------------------------------
//
// PDD_SDReceiveData
//
// Returns the SD/MMC read buffer
//
BOOL PDD_SDReceiveData(UINT8 *pBuffer) {
	INT32 idx;
	UINT8 *pbuff8 = (UINT8 *) secbuff;

	if (datarecv == TRUE)
	{
		for (idx = 0; idx < 512; idx++)
		{
			pBuffer [idx] = pbuff8 [idx];
		}
	}

	return datarecv;
}

//------------------------------------------------------------------------------
//
// PDD_SDGetResponse
//
// Returns the SD/MMC response
//
UINT32 PDD_SDGetResponse(int whichResp) {
	SDCARD_REGS_T *pSDCARDRegs;

	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);

	switch( whichResp ) {
		case RCA_REGISTER:	//16-bit response
			return (UINT32) (pSDCARDRegs->sd_resp [0] >> 16);

		case CARD_STATUS_REGISTER:
		case OCR_REGISTER:  //32-bit responses
			return (UINT32) pSDCARDRegs->sd_resp [0];

		case SCR_REGISTER:  //64-bit response
		case CID_REGISTER:  //128-bit responses
		case CSD_REGISTER:
		default:
			return 0;      //Not implemented
	}

	return 0;
}

//------------------------------------------------------------------------------
//
// PDD_SDSetPDDCapabilities
//
// Sets the SD/MMC capabilities
//
VOID PDD_SDSetPDDCapabilities(PDD_IOCTL whichAbility, UINT32 Ability) {
	volatile UNS_32 tmp;
	SDCARD_REGS_T *pSDCARDRegs;

	pSDCARDRegs = (SDCARD_REGS_T *) OALPAtoVA((UINT32) SDCARD, FALSE);

	switch( whichAbility ) {
		case SET_OPEN_DRAIN:
			tmp = pSDCARDRegs->sd_power;
			if (Ability == 0) {
				tmp &= ~SD_OPENDRAIN_EN;
			}
			else {
				tmp |= SD_OPENDRAIN_EN;
			}
			pSDCARDRegs->sd_power = tmp;
			break;

		case SET_4BIT_MODE:
			tmp = pSDCARDRegs->sd_clock;
			if( Ability == 0) {
				tmp &= ~SD_WIDEBUSMODE_EN;
			} else {
				tmp |= SD_WIDEBUSMODE_EN;
			}
			pSDCARDRegs->sd_clock = tmp;
			break;

		case SET_CLOCK_RATE:
			sd_set_clk(Ability);
			break;
	}
}

//------------------------------------------------------------------------------
//
// PDD_SDGetPDDCapabilities
//
// Gets the SD/MMC capabilities
//
UINT32 PDD_SDGetPDDCapabilities(PDD_IOCTL whichAbility) {
	switch( whichAbility ) {
		case GET_SUPPORTED_OCR_MMC:
			return OCR_SUPPORTED_VOLTAGES_MMC;

		case GET_SUPPORTED_OCR_SD:
			return OCR_SUPPORTED_VOLTAGES_SD;

		default:
			return 0;
	}

	return 0;
}

//------------------------------------------------------------------------------
//
// PDD_SDStallExecute
//
// MilliSecond stall function
//
VOID PDD_SDStallExecute(UINT32 waitMs) {
	OALStall(waitMs*1000);
}
