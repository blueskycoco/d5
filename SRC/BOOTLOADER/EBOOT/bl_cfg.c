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
// bl_cfg.c
//
// This file implements functions used to load/save EBOOT configuration
// info. The EBOOT configuration is located in the serial EEPROM.
//

#include <windows.h>
#include <eboot.h>
#include <bsp_cfg.h>
#include <oal.h>
#include "lpc32xx_ssp.h"
#include "lpc32xx_clkpwr.h"
#include "lpc32xx_gpio.h"

/* Serial EEPROM commands (SPI via SSP) */
#define SEEPROM_WREN          0x06
#define SEEPROM_WRDI          0x04
#define SEEPROM_RDSR          0x05
#define SEEPROM_WRSR          0x01
#define SEEPROM_READ          0x03
#define SEEPROM_WRITE         0x02


//------------------------------------------------------------------------------
//
// ssp_read
//
// Read data from the SSP FIFO
//
INT32 ssp_read(void *buffer,
                INT32 max_fifo)
{
    INT32 count = 0;
	SSP_REGS_T *pSSPRegs;
    UINT8 *data8 = (UINT8 *) buffer;

	pSSPRegs = (SSP_REGS_T *) OALPAtoVA((UINT32) SSP0, FALSE);
	
	while ((max_fifo > 0) && ((pSSPRegs->sr & SSP_SR_RNE) != 0))
    {
   		*data8 = (UINT8) pSSPRegs->data;
   		data8++;

		/* Increment data count and decrement buffer size count */
        count++;
        max_fifo--;
    }

    return count;
}

//------------------------------------------------------------------------------
//
// ssp_write
//
// Write data to the SSP FIFO
//
INT32 ssp_write(void *buffer,
                 INT32 n_fifo)
{
    INT32 count = 0;
    UINT8 *data8 = (UINT8 *) buffer;
	SSP_REGS_T *pSSPRegs;

	pSSPRegs = (SSP_REGS_T *) OALPAtoVA((UINT32) SSP0, FALSE);

    /* Loop until transmit ring buffer is full or until n_bytes
       expires */
    while ((n_fifo > 0) && ((pSSPRegs->sr & SSP_SR_TNF) != 0))
    {
   		pSSPRegs->data = (UINT32) *data8;
   		data8++;

		/* Increment data count and decrement buffer size count */
        count++;
        n_fifo--;
    }

	return count;
}

//------------------------------------------------------------------------------
//
// lpc3250_sspxfer
//
// Transfer data between the SSP and the EEPROM
//
BOOL lpc3250_sspxfer(UINT8 *out,
                      UINT8 *in,
                      INT32 bytes) 
{
	GPIO_REGS_T *pGPIORegs;

	INT32 rbytes = 0;

	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);

    /* Asset chip select */
    pGPIORegs->pio_outp_clr = OUTP_STATE_GPIO(5);
    ssp_write(out, bytes);
    while (rbytes < bytes) 
    {
        rbytes += ssp_read(&in [rbytes], 1);
    }
    pGPIORegs->pio_outp_set = OUTP_STATE_GPIO(5);

	return TRUE;
}

//------------------------------------------------------------------------------
//
// lpc3250_sspread
//
// Read data from the EEPROM on the SSP
//
 UINT8 lpc3250_sspread(INT32 index) 
{
	UINT8 datai [8], datao [8];
	UINT8 byte = 0;

	/* Read byte */
	datao [0] = SEEPROM_READ;
	datao [1] = (UINT8)((index >> 8) & 0xFF);
	datao [2] = (UINT8)((index >> 0) & 0xFF);
	datao [3] = 0xFF;
	lpc3250_sspxfer(datao, datai, 4);
	byte = datai [3];

	return byte;
}

//------------------------------------------------------------------------------
//
// lpc3250_sspwrite
//
// Write data to the EEPROM on the SSP
//
 void lpc3250_sspwrite(UINT8 byte,
                       INT32 index) 
{
	UINT8 prog, datai [8], datao [8];

	/* Write enable */
	datao [0] = SEEPROM_WREN;
	datao [1] = 0xFF;
	lpc3250_sspxfer(datao, datai, 2);

	/* Write byte */
	datao [0] = SEEPROM_WRITE;
	datao [1] = (UINT8)((index >> 8) & 0xFF);
	datao [2] = (UINT8)((index >> 0) & 0xFF);
	datao [3] = byte;
	lpc3250_sspxfer(datao, datai, 4);

	/* Wait for device to finish programming */
	prog = 0xFF;
	while ((prog & 0x2) != 0) 
	{
		/* Read status */
		datao [0] = SEEPROM_RDSR;
		datao [1] = 0xFF;
		lpc3250_sspxfer(datao, datai, 2);
		prog = datai [1];
	}
}

//------------------------------------------------------------------------------
//
// BLInitBootCfg
//
// Initialize the SSP for access to the EEPROM
//
BOOL BLInitBootCfg(VOID)
{
	SSP_REGS_T *pSSPRegs;
	CLKPWR_REGS_T *pCLKPWRRegs;
	GPIO_REGS_T *pGPIORegs;

	pSSPRegs = (SSP_REGS_T *) OALPAtoVA((UINT32) SSP0, FALSE);
	pCLKPWRRegs = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);
	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);

	/* Enable SSP clock */
    pCLKPWRRegs->clkpwr_ssp_blk_ctrl = CLKPWR_SSPCTRL_SSPCLK0_EN;

    /* Setup SSP0 configuration for 8 data bits, SPI mode, master mode,
       clock divide by 1 */
    pSSPRegs->cr0 = SSP_CR0_SCR (0) | SSP_CR0_DSS(8) | SSP_CR0_FRF_MOT;
    pSSPRegs->cr1 = 0;

    /* Setup SSP0 clock to divide by 32, which will allow about <5MHz max
       when HCLK is 150MHz */
    pSSPRegs->cpsr = 32;

    /* Clear latched interrupts and disable interrupts */
    pSSPRegs->icr = (SSP_IIR_RORRIS | SSP_IIR_RTRIS);
    pSSPRegs->imsc = 0;

	/* Setup GPIO5 as an output driven high */
	pGPIORegs->pio_dir_set = OUTP_STATE_GPIO(5);
	pGPIORegs->pio_outp_set = OUTP_STATE_GPIO(5);

	/* Enable controller */
    pSSPRegs->cr1 = SSP_CR1_ENABLE;

    return TRUE;
}

//------------------------------------------------------------------------------
//
// BLDeinitBootCfg
//
// De-initialize the SSP
//
BOOL BLDeinitBootCfg(void)
{
	CLKPWR_REGS_T *pCLKPWRRegs;

	pCLKPWRRegs = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);

	/* Disable SSP clock */
    pCLKPWRRegs->clkpwr_ssp_blk_ctrl = 0;

	return TRUE;
}

//------------------------------------------------------------------------------
//
// BLWriteBootCfg
//
// Write the boot configuration to the EEPROM
//
BOOL BLWriteBootCfg(BOOT_CFG *pBootCfg)
{
	int idx;
	UINT8 *psspcfg = (UINT8 *) pBootCfg;
	UINT32 genck = 0, *dat = (UINT32 *) pBootCfg;
	int sz = sizeof(BOOT_CFG) / sizeof(UINT32);

	/* Generate verification key */
	sz -= 1;
	while (sz > 0) 
	{
		genck = genck + *dat;
		dat++;
		sz--;
	}
	pBootCfg->verikey = ~genck;

	/* Write configuration structure */
	for (idx = 0; idx < sizeof(BOOT_CFG); idx++) 
	{
		lpc3250_sspwrite(*psspcfg, (EBOOT_CFG_SSEPROM_SINDEX + idx));
		psspcfg++;
	}

	return TRUE;
}

//------------------------------------------------------------------------------
//
// BLReadBootCfg
//
// Read the boot configuration from the EEPROM
//
BOOL BLReadBootCfg(BOOT_CFG *pBootCfg)
{
	UINT8 data8, *psspcfg = (UINT8 *) &g_bootCfg;
	BOOL goodinfo = FALSE;
	UINT32 genck = 0, *dat = (UINT32 *) pBootCfg;
	int idx, sz = sizeof(BOOT_CFG) / sizeof(UINT32);

	/* Read configuration structure */
	for (idx = 0; idx < sizeof(BOOT_CFG); idx++) 
	{
		data8 = lpc3250_sspread(EBOOT_CFG_SSEPROM_SINDEX + idx);
		psspcfg[idx] = data8;
	}

	/* Verify checksum */
	sz -= 1;
	while (sz > 0) 
	{
		genck = genck + *dat;
		dat++;
		sz--;
	}

	if ((genck ^ pBootCfg->verikey) == 0xFFFFFFFF) 
	{
		goodinfo = TRUE;
	}

	return goodinfo;
}
