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
// bsp_boot_led.c
//
// LED setup and control functions used with the bootloader
//

#include <windows.h>
#include <oal.h>
#include <blcommon.h>
#include "clkpwr_support.h"
#include "lpc32xx_gpio.h"
#include "lpc32xx_slcnand.h"
#include "boot_flash.h"
#include "phy3250_board.h"

// Driver data structure
NAND_GEOM_T savedgeom;

//------------------------------------------------------------------------------
//
// slc_cmd
//
// Issue SLC/NAND command
//
void slc_cmd(UNS_8 cmd)
{
	SLCNAND_REGS_T *pSLCRegs;
	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);
	pSLCRegs->slc_cmd = (UNS_32) cmd;
}

//------------------------------------------------------------------------------
//
// slc_addr
//
// Send address byte to FLASH
//
void slc_addr(UNS_8 addr)
{
	SLCNAND_REGS_T *pSLCRegs;
	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);
	pSLCRegs->slc_addr = (UNS_32) addr;
}

//------------------------------------------------------------------------------
//
// slc_wait_ready
//
// Wait for NAND ready
//
void slc_wait_ready(void)
{
	SLCNAND_REGS_T *pSLCRegs;

	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);
	while ((pSLCRegs->slc_stat & SLCSTAT_NAND_READY) == 0);
}

//------------------------------------------------------------------------------
//
// slc_get_status
//
// Get NAND status
//
UNS_8 slc_get_status(void)
{
	SLCNAND_REGS_T *pSLCRegs;
	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);
	slc_cmd(LPCNAND_CMD_STATUS);
	slc_wait_ready();

	return (UNS_8) pSLCRegs->slc_data;
}

//------------------------------------------------------------------------------
//
// slc_write_addr
//
// Write a NAND address
//
void slc_write_addr(INT_32 block,
                    INT_32 page) 
{
    UNS_32 nandaddr;

	// Block  Page  Index
	// 31..13 12..8 7..0

	nandaddr = (page & 0x1F) << 8;
	nandaddr = nandaddr | ((block & 0xFFF) << 13);

    // Write block and page address
	slc_addr((UNS_8) (nandaddr >> 0)  & 0xFF);
	slc_addr((UNS_8) (nandaddr >> 8)  & 0xFF);
	slc_addr((UNS_8) (nandaddr >> 16) & 0xFF);
	if (savedgeom.addrcycles == 4) 
	{
		slc_addr((UNS_8) (nandaddr >> 24) & 0xFF);
	}
}

//------------------------------------------------------------------------------
//
// slc_write_page
//
// Write a NAND page
//
INT_32 nand_write_page(INT_32 block,
                       INT_32 page,
                       UNS_8 *buff,
                       UNS_8 *extrabuff)
{
    INT_32 towrite, idx;
    UNS_8 status;
	volatile UNS_32 tmp;
	SLCNAND_REGS_T *pSLCRegs;

	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);

	// Force nCE for the entire cycle
    pSLCRegs->slc_cfg |= SLCCFG_CE_LOW;
        
    // Issue page write1 command
    slc_cmd(LPCNAND_CMD_PAGE_READA);
    slc_cmd(LPCNAND_CMD_PAGE_WRITE1);

	// Write address
	slc_write_addr(block, page);

    // Write 512 bytes of data
    towrite = 512;
    while (towrite > 0)
   	{
		tmp = *buff;
		buff++;
   		pSLCRegs->slc_data = tmp;
   		towrite = towrite - 1;
    }
    // Write 6 bytes
    if (extrabuff != NULL) 
	{
		for (idx = 0; idx < 6; idx++)
		{
			pSLCRegs->slc_data = (UNS_32) *extrabuff;
			extrabuff++;
	    }
    }

    // Issue page write2 command
    slc_cmd(LPCNAND_CMD_PAGE_WRITE2);

    // Deassert nCE
    pSLCRegs->slc_cfg &= ~SLCCFG_CE_LOW;

	// Wait for device ready
	slc_wait_ready();

    // Read status
    status = slc_get_status();
    if ((status & 0x1) != 0)
    {
    	// Program was not good
    	return 0;
    }

    return 512;
}

//------------------------------------------------------------------------------
//
// slc_read_page
//
// Read a NAND page
//
INT_32 nand_read_page(INT_32 block,
                      INT_32 page,
                      UNS_8 *buff,
                      UNS_8 *extrabuff)
{
    INT_32 toread, idx;
	SLCNAND_REGS_T *pSLCRegs;

	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);

    // Force nCE for the entire cycle
    pSLCRegs->slc_cfg |= SLCCFG_CE_LOW;
        
    // Issue page read1 command
    slc_cmd(LPCNAND_CMD_PAGE_READA);

	// Write address
	slc_write_addr(block, page);

	// Wait for ready
	slc_wait_ready();

    // Read data
   	toread = 512;
    while (toread > 0)
   	{
    	*buff = (UNS_8) pSLCRegs->slc_data;
   		buff++;
   		toread = toread - 1;
    }

	// Read 6 bytes
    if (extrabuff != NULL) 
	{
		for (idx = 0; idx < 6; idx++)
		{
   			*extrabuff = (UNS_8) pSLCRegs->slc_data;
			extrabuff++;
	    }
	}

	// Deassert nCE
    pSLCRegs->slc_cfg &= ~SLCCFG_CE_LOW;

	// Wait for device ready
	slc_wait_ready();

    return 512;
}

//------------------------------------------------------------------------------
//
// slc_nand_detect
//
// Detect NAND device
//
static INT_32 slc_nand_detect(NAND_GEOM_T *geom)
{
	SLCNAND_REGS_T *pSLCRegs;
	UNS_8 id[4];
	volatile UNS_32 tmp;
	INT_32 init = 0;

	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);

	// Reset NAND device and wait for ready
	slc_cmd(LPCNAND_CMD_RESET);
	slc_wait_ready();

	// Read the device ID
	slc_cmd(LPCNAND_CMD_READ_ID);
	slc_addr(0);
	id [0] = pSLCRegs->slc_data;
	id [1] = pSLCRegs->slc_data;
	id [2] = pSLCRegs->slc_data;
	id [3] = pSLCRegs->slc_data;

	OALMSG(OAL_INFO, (L"ID %x,%x\r\n",id[0],id[1] ));
	// Verify ID
	if ((id [0] == LPCNAND_VENDOR_STMICRO)||(id [0] ==LPCNAND_VENDOR_SAMSUNG))
	{
		switch (id [1]) 
		{
			case 0x73:
				// NAND128-A
			    init = 1;
			    savedgeom.num_blocks = 1024;
			    savedgeom.addrcycles = 3;
		    	break;

			case 0x35:
			case 0x75:
				// NAND256-A
			    init = 1;
			    savedgeom.num_blocks = 2048;
			    savedgeom.addrcycles = 3;
		    	break;

			case 0x36:
			case 0x76:
				// NAND512-A
			    init = 1;
			    savedgeom.num_blocks = 4096;
			    savedgeom.addrcycles = 4;
		    	break;

			case 0x39:
			case 0x79:
				// NAND01G-A
			    init = 1;
			    savedgeom.num_blocks = 8192;
			    savedgeom.addrcycles = 4;
		    	break;
		    	
	    	default:
		    	break; 
		}

	    savedgeom.pages_per_block = 32;
	   	savedgeom.bytes_per_page  = 512;
	   	savedgeom.extra_per_page  = 16;
	    *geom = savedgeom;
	}

	return init;
}

//------------------------------------------------------------------------------
//
// nand_clock_setup
//
// Optimize NAND clocks
//
void nand_clock_setup(void) 
{
	UNS_32 clk;
	SLCNAND_REGS_T *pSLCRegs;

	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);

	// Get NAND controller clock
	clk = clkpwr_get_base_clock_rate(CLKPWR_HCLK);

    pSLCRegs->slc_tac = (
		SLCTAC_WDR(PHY_NAND_WDR) |
		SLCTAC_WWIDTH(clk / PHY_NAND_WWIDTH) |
		SLCTAC_WHOLD(clk / PHY_NAND_WHOLD) |
		SLCTAC_WSETUP(clk / PHY_NAND_WSETUP) |
		SLCTAC_RDR(PHY_NAND_RDR) |
		SLCTAC_RWIDTH(clk / PHY_NAND_RWIDTH) |
		SLCTAC_RHOLD(clk / PHY_NAND_RHOLD) |
		SLCTAC_RSETUP(clk / PHY_NAND_RSETUP));
}

//------------------------------------------------------------------------------
//
// nand_init
//
// Initialize NAND interface
//
INT_32 nand_init(NAND_GEOM_T *geom)
{
	CLKPWR_REGS_T *pClkpwr;
	SLCNAND_REGS_T *pSLCRegs;
	int idx;

	pClkpwr = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);
	pSLCRegs = (SLCNAND_REGS_T *) OALPAtoVA((UINT32) SLCNAND, FALSE);

	pClkpwr->clkpwr_nand_clk_ctrl = (CLKPWR_NANDCLK_SEL_SLC |
		CLKPWR_NANDCLK_SLCCLK_EN);

	// Reset SLC controller and setup for 8-bit mode, disable and clear interrupts
	for (idx = 0; idx < 0x1FFFF; idx++);
	pSLCRegs->slc_cfg = 0;
	pSLCRegs->slc_icr = 0;

	// Setup NAND timing
	nand_clock_setup();

	// Detect device
	return slc_nand_detect(geom);
}

//------------------------------------------------------------------------------
//
// nand_erase_block
//
// Erase a data block
//
INT_32 nand_erase_block(INT_32 block) 
{
    UNS_8 status;
    INT_32 erased = 0;

    // Issue block erase1 command
    slc_cmd(LPCNAND_CMD_ERASE1);

    // Write block and page address
	slc_addr((UNS_8) ((block << 5) & 0x00E0));
	slc_addr((UNS_8) ((block >> 3) & 0x00FF));
	if (savedgeom.addrcycles == 4) 
	{
		slc_addr((UNS_8) ((block >> 11) & 0x0003));
	}

    // Issue page erase2 command
    slc_cmd(LPCNAND_CMD_ERASE2);

	// Wait for ready
	slc_wait_ready();

    // Read status
    status = slc_get_status();
    if ((status & 0x1) == 0)
    {
    	// Erase was good
    	erased = 1;
    }

    return erased;
}

//------------------------------------------------------------------------------
//
// nand_read_sectors
//
// Read a series of data sectors
//
INT_32 nand_read_sectors(INT_32 sector_start,
                         INT_32 num_sectors,
					     UNS_8 *readbuff,
					     BOOL_32 skipbad) 
{
	UNS_8 extrabuff [512 + 16];
	BOOL_32 checkblk = skipbad;
	INT_32 block, page, bread = 0;

    // Translate to page/block address
    sector_to_nand(sector_start, &block, &page);

	// Read all sectors
	while (num_sectors > 0) 
	{
		// Is a block read needed?
		while (checkblk == TRUE) 
		{
			nand_read_page(block, 0, extrabuff, &extrabuff[512]);
			if (extrabuff [NAND_BADBLOCK_OFFS] !=
				NAND_GOOD_BLOCK_MARKER) 
			{
				// Block is bad, skip to next block
				block++;
				page = 0;
				checkblk = TRUE;
			}
			else 
			{
				checkblk = FALSE;
			}
		}

		// Read sector
		nand_read_page(block, page, readbuff, &extrabuff[512]);
		readbuff += savedgeom.bytes_per_page;
		bread += savedgeom.bytes_per_page;
		page++;
		if (page >= savedgeom.pages_per_block) 
		{
			page = 0;
			block++;
			checkblk = skipbad;
		}

		num_sectors--;
	}

	return bread;
}

//------------------------------------------------------------------------------
//
// nand_write_sectors
//
// Write a series of data sectors
//
INT_32 nand_write_sectors(INT_32 sector_start,
                          INT_32 num_sectors,
					      UNS_8 *writebuff,
					      BOOL_32 skipbad) 
{
	UNS_8 extrabuff [512 + 16];
	BOOL_32 checkblk = skipbad;
	INT_32 block, page, bwrite = 0;

    // Translate to page/block address
    sector_to_nand(sector_start, &block, &page);

	// Write all sectors
	while (num_sectors > 0) 
	{
		// Is a block read needed?
		while (checkblk == TRUE) 
		{
			nand_write_page(block, 0, extrabuff, NULL);
			if (extrabuff [NAND_BADBLOCK_OFFS] !=
				NAND_GOOD_BLOCK_MARKER) 
			{
				// Block is bad, skip to next block
				block++;
				page = 0;
				checkblk = TRUE;
			}
			else 
			{
				checkblk = FALSE;
			}
		}

		// Write sector
		nand_write_page(block, page, writebuff, NULL);
		writebuff += savedgeom.bytes_per_page;
		bwrite += savedgeom.bytes_per_page;
		page++;
		if (page >= savedgeom.pages_per_block) 
		{
			page = 0;
			block++;
			checkblk = skipbad;
		}

		num_sectors--;
	}

	return bwrite;
}

//------------------------------------------------------------------------------
//
// nand_to_sector
//
// Convert a block/page number to a sector number
//
INT_32 nand_to_sector(INT_32 block,
                      INT_32 page) 
{
    return (page + (block * savedgeom.pages_per_block));
}

//------------------------------------------------------------------------------
//
// sector_to_nand
//
// Convert a sector to a block/page number
//
void sector_to_nand(INT_32 sector,
                    INT_32 *block,
                    INT_32 *page)
{
    *block = sector / savedgeom.pages_per_block;
    *page = sector - (*block * savedgeom.pages_per_block);
}

//------------------------------------------------------------------------------
//
// phy3250_nand_wp
//
// Enable or disable NAND write protect
//
void phy3250_nand_wp(BOOL_32 enable) 
{
	GPIO_REGS_T *pGPIORegs;

	pGPIORegs = (GPIO_REGS_T *) OALPAtoVA((UINT32) GPIO, FALSE);

	if (enable == TRUE) {
		pGPIORegs->pio_outp_set = OUTP_STATE_GPO(19);
	}
	else
	{
		pGPIORegs->pio_outp_clr = OUTP_STATE_GPO(19);
	}
}
