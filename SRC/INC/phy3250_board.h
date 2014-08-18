/***********************************************************************
 * $Id:: phy3250_board.h 498 2008-04-08 16:29:08Z wellsk               $
 *
 * Project: Phytec 3250 board definitions
 *
 * Description:
 *     This file contains board specific information such as the
 *     chip select wait states, and other board specific information.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only  
 * which provides customers with programming information regarding the  
 * products. This software is supplied "AS IS" without any warranties.  
 * NXP Semiconductors assumes no responsibility or liability for the 
 * use of the software, conveys no license or title under any patent, 
 * copyright, or mask work right to the product. NXP Semiconductors 
 * reserves the right to make changes in the software without 
 * notification. NXP Semiconductors also make no representation or 
 * warranty that such application will be suitable for the specified 
 * use without further testing or modification. 
 **********************************************************************/

#ifndef PHY3250_BOARD_H
#define PHY3250_BOARD_H

#include "lpc_types.h"
#include "lpc32xx_chip.h"
#include "lpc32xx_emc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Structure used to define the hardware for the Phytec board */
typedef struct {
	UNS_32 dramcfg;    /* DRAM config word */
	UNS_32 syscfg;     /* Configuration word */
	/* MAC address, use lower 6 bytes only, index 0 is first byte */
	UNS_8  mac[8];     /* Only the first 6 are used */
	UNS_32 rsvd [5];   /* Reserved, must be 0 */
	UNS_32 fieldvval;  /* Must be PHY_HW_VER_VAL */
} PHY_HW_T;
extern PHY_HW_T phyhwdesc;

/* Phytec hardware verification value for configuation field */
#define PHY_HW_VER_VAL 0x000A3250

/***********************************************************************
 * DRAM config word (dramcfg) bits
 * Bits        Value             Description
 * 1..0        00                Low power SDRAM
 * 1..0        01                SDRAM
 * 1..0        1x                Reserved
 * 4..2        000               SDRAM 16M,  16 bits x 2 devices, 0xa5
 * 4..2        001               SDRAM 32M,  16 bits x 2 devices, 0xa9
 * 4..2        010               SDRAM 64M,  16 bits x 2 devices, 0xad
 * 4..2        011               SDRAM 128M, 16 bits x 2 devices, 0xb1
 * 31..5       0                 Reserved
 **********************************************************************/
/* Mask for get the DRAM type */
#define PHYHW_DRAM_TYPE_MASK       0x3
/* DRAM types (2 bits) */
#define PHYHW_DRAM_TYPE_LPSDRAM    0x0
#define PHYHW_DRAM_TYPE_SDRAM      0x1
#define PHYHW_DRAM_TYPE_LPDDR      0x2
#define PHYHW_DRAM_TYPE_DDR        0x3
/* DRAM configuration mask */
#define PHYHW_DRAM_SIZE_MASK       _SBF(2, 0x3)
/* DRAM configurations (3 bits) */
#define PHYHW_DRAM_SIZE_16M        _SBF(2, 0x0)
#define PHYHW_DRAM_SIZE_32M        _SBF(2, 0x1)
#define PHYHW_DRAM_SIZE_64M        _SBF(2, 0x2)
#define PHYHW_DRAM_SIZE_128M       _SBF(2, 0x3)

/***********************************************************************
 * Configuation word (syscfg) bits
 * Bits        Value             Description
 * 0           0                 SDIO not populated
 * 0           1                 SDIO populated
 * 31..1       0                 Reserved
 **********************************************************************/
/* SDIO populated bit */
#define PHYHW_SDIO_POP             _BIT(0)

/***********************************************************************
 * SDRAM configuration
 **********************************************************************/

/* Structure used to store programming and timing information for each
   SDRAM device configuration */
typedef struct 
{
	UNS_32 dyncfgword;    /* Programmed value for emcdynamicconfig0 */
	UNS_32 modeword;      /* Mode word for devices */
	UNS_32 emodeword;     /* Extended mode word for devices */
	UNS_32 cas;           /* CAS half clocks */
	UNS_32 ras;           /* RAS clocks */
	UNS_32 tdynref;
	UNS_32 trp;
	UNS_32 tras;
	UNS_32 tsrex;
	UNS_32 twr;
	UNS_32 trc;
	UNS_32 trfc;
	UNS_32 txsr;
	UNS_32 trrd;
	UNS_32 tmrd;
	UNS_32 tcdlr;
} PHY_DRAM_CFG_T;
extern PHY_DRAM_CFG_T dram_cfg [4];

/* SDRAM address map configuration values
   MT48H4M16LFB4-8 IT (x2 chips -> 16MB)
   MT48H8M16LFB4-8 IT (x2 chips -> 32MB)
   MT48H16M16LFBF-8 IT (x2 chips -> 64MB)
   MT48H32M16LFBF-8 IT (x2 chips -> 128MB) */
#define SDRAM_ADDRESS_MAP_16MB	0xA5
#define SDRAM_ADDRESS_MAP_32MB	0xA9
#define SDRAM_ADDRESS_MAP_64MB  0xAD
#define SDRAM_ADDRESS_MAP_128MB 0xB1

/* A read starts with the bank and row on the SDRAM address lines.
   This is when the SDRAM reads the address lines to configure the
   mode and extended mode registers. Since the address lines are
   multiplex for BANK, ROW, COLUMN, the address is sent in two parts
   (BANK + ROW followed by BANK + COLUMN). The 32-bit address
   supplied to the SDRAM controller when performing a read or write
   is composed like:

   [31............0]
   [bank_bits,row_bits,col_bits]
  
   The actual bit placement of the bank, row, and column depends on
   the memory density. The mapping is as follows for the
   phyCORE-LPC3250:
  
    16MB: [23]bank_1, [22]bank_0, [21:10]row_bits, [9:2]col_bits
    32MB: [24]bank_0, [23]bank_1, [22:11]row_bits, [10:2]col_bits
    64MB: [25]bank_1, [24]bank_0, [23:11]row_bits, [10:2]col_bits
   128MB: [26]bank_0, [25]bank_1, [24:12]row_bits, [11:2]col_bits
  
   The ROW and COLUMN are transmitted on LPC3250 address pins 12:0
   while the bank bits are transmitted on address pins 14 and 13
   (bank_bit_1 and bank_bit_0).
  
   Writing to the mode and extended mode registers involves setting
   the bank bits and row bits (see SDRAM datasheet for details).
   Below are macros to translate a value into the correct position
   in the 32-bit address for ROW and BANK. */
#define SDRAM_MAP_ROW_16MB(x)           (x << 10)
#define SDRAM_MAP_ROW_32MB(x)           (x << 11)
#define SDRAM_MAP_ROW_64MB(x)           (x << 11)
#define SDRAM_MAP_ROW_128MB(x)          (x << 12)

#define SDRAM_MAP_BANK_16MB(BA1,BA0)    ((BA1 << 23) | (BA0 << 22))
#define SDRAM_MAP_BANK_32MB(BA1,BA0)    ((BA1 << 23) | (BA0 << 24))
#define SDRAM_MAP_BANK_64MB(BA1,BA0)    ((BA1 << 25) | (BA0 << 24))
#define SDRAM_MAP_BANK_128MB(BA1,BA0)   ((BA1 << 25) | (BA0 << 26))

/* A reference to the mapping of the address pins to mode and
   extended mode register bit definitions is provided (see Micron
   SDRAM datasheet for details):
   
   --MODE REGISTER--
   BA1,BA0 	= mode or extended mode select -- [0,0] for mode register
   A11,A10 	= reserved
   A9		= write burst mode
   A8,A7	= operating mode
   A6...A4	= CAS latency
   A3		= burst type
   A2...A0	= burst length
  
   --EXTENDED MODE REGISTER --
   BA1,BA0	= mode or extended mode select -- [1,0] for extended
              mode register
   A11...A7	= must be set to 0
   A6,A5	= driver strength
   A4,A3	= maximum case temperature (setting these bits has no
              effect for Micron SDRAM due to on-die temp sensor
              used instead)
   A2...A0	= self refresh coverage (i.e., four banks, two banks,
              one bank, etc..)
   
   Compose the addresses required to set the mode & extended mode
   registers for various SDRAM densities. For the mode registers we
   only set the cas to 3 and leave the other settings as default in
   mode & extended mode regs. Notice the EMODE reg definitions set
   the bank mapping to [1,0] to select the extended mode register. */

#define DRAM_MODE_VAL_16MB  (SDRAM_MAP_BANK_16MB(0,0) | \
	SDRAM_MAP_ROW_16MB(0x3 << 4)) 	/* cas = 3 */
#define DRAM_MODE_VAL_32MB  (SDRAM_MAP_BANK_32MB(0,0) | \
	SDRAM_MAP_ROW_32MB(0x3 << 4)) 	/* cas = 3 */
#define DRAM_MODE_VAL_64MB  (SDRAM_MAP_BANK_64MB(0,0) | \
	SDRAM_MAP_ROW_64MB(0x3 << 4)) 	/* cas = 3 */
#define DRAM_MODE_VAL_128MB (SDRAM_MAP_BANK_128MB(0,0) | \
	SDRAM_MAP_ROW_128MB(0x3 << 4))	/* cas = 3 */
#define DRAM_EMODE_VAL_16MB  (SDRAM_MAP_BANK_16MB(1,0))
#define DRAM_EMODE_VAL_32MB  (SDRAM_MAP_BANK_32MB(1,0))
#define DRAM_EMODE_VAL_64MB  (SDRAM_MAP_BANK_64MB(1,0))
#define DRAM_EMODE_VAL_128MB (SDRAM_MAP_BANK_128MB(1,0))

/***********************************************************************
 * NAND, NOR FLASH, and SDIO timing information
 **********************************************************************/
/* S29AL008D NOR device */
#define PHY_NOR_MEM_CFG       (EMC_STC_MEMWIDTH_32 | EMC_STC_BLS_EN_BIT)
#define PHY_NOR_CS_TO_WE      0xFFFFFFFF /* Not used */
#define PHY_NOR_CS_TO_OE      0xFFFFFFFF /* Not used */
#define PHY_NOR_CS_TO_OUT     18181818
#define PHY_NOR_OE_BURST      0xFFFFFFFF /* Not used */
#define PHY_NOR_WE_HOLD_TIME  28571428
#define PHY_NOR_OE_TO_HIZ     62500000

/* SDIO101 device */
#define PHY_SDIO_MEM_CFG      (EMC_STC_MEMWIDTH_16 | EMC_STC_BLS_EN_BIT)
#define PHY_SDIO_CS_TO_WE     0xFFFFFFFF /* Not used */
#define PHY_SDIO_CS_TO_OE     0xFFFFFFFF /* Not used */
#define PHY_SDIO_CS_TO_OUT    40000000
#define PHY_SDIO_OE_BURST     0xFFFFFFFF /* Not used */
#define PHY_SDIO_WE_HOLD_TIME 50000000
#define PHY_SDIO_OE_TO_HIZ    50000000

// NAND256R3A2CZA6 device timing values 2
#define PHY_NAND_WDR         3
#define PHY_NAND_WWIDTH      28571428  // 35nS
#define PHY_NAND_WHOLD       100000000 // 10nS
#define PHY_NAND_WSETUP      66666666  // 15nS
#define PHY_NAND_RDR         3
#define PHY_NAND_RWIDTH      28571428  // 35nS
#define PHY_NAND_RHOLD       100000000 // 10nS
#define PHY_NAND_RSETUP      66666666  // 15nS

// Vendor IDs
#define LPCNAND_VENDOR_STMICRO      0x20
#define LPCNAND_VENDOR_SAMSUNG     0xEC
#define LPCNAND_VENDOR_MICRON       0x2C

// Common NAND commands
#define LPCNAND_CMD_PAGE_READA    0x00
#define LPCNAND_CMD_PAGE_READB    0x01
#define LPCNAND_CMD_PAGE_READC    0x50
#define LPCNAND_CMD_PAGE_READ1    0x00
#define LPCNAND_CMD_PAGE_READ2    0x30
#define LPCNAND_CMD_RANDOM_READ1  0x05
#define LPCNAND_CMD_RANDOM_READ2  0xE0
#define LPCNAND_CMD_STATUS        0x70
#define LPCNAND_CMD_READ_ID       0x90
#define LPCNAND_CMD_PAGE_WRITE1   0x80
#define LPCNAND_CMD_PAGE_WRITE2   0x10
#define LPCNAND_CMD_RANDOM_WRITE1 0x85
#define LPCNAND_CMD_ERASE1        0x60
#define LPCNAND_CMD_ERASE2        0xD0
#define LPCNAND_CMD_RESET         0xFF

/***********************************************************************
 * Functions
 **********************************************************************/

/* Serial EEPROM commands (SPI via SSP) */
#define SEEPROM_WREN          0x06
#define SEEPROM_WRDI          0x04
#define SEEPROM_RDSR          0x05
#define SEEPROM_WRSR          0x01
#define SEEPROM_READ          0x03
#define SEEPROM_WRITE         0x02

/* Size of serial EEPROM */
#define PHY3250_SEEPROM_SIZE  0x8000

/* Offset into serial EEPROM where board configuration information is
   saved */
#define PHY3250_SEEPROM_CFGOFS (PHY3250_SEEPROM_SIZE - 0x100)

/***********************************************************************
 * I2C device address
 **********************************************************************/
#define PHY_I2C_I2S   0x18
#define PHY_I2C_RTC   0xA2
#define PHY_I2C_OTG   0x2D

#ifdef __cplusplus
}
#endif

#endif /* PHY3250_BOARD_H */
