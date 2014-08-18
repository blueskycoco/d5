//*********************************************************************
//* Software that is described herein is for illustrative purposes only  
//* which provides customers with programming information regarding the  
//* products. This software is supplied "AS IS" without any warranties.  
//* NXP Semiconductors assumes no responsibility or liability for the 
//* use of the software, conveys no license or title under any patent, 
//* copyright, or mask work right to the product. NXP Semiconductors 
//* reserves the right to make changes in the software without 
//* notification. NXP Semiconductors also make no representation or 
//* warranty that such application will be suitable for the specified 
//* use without further testing or modification. 
//*
//* Copyright NXP Semiconductors
//*********************************************************************
//
// image_cfg.h
//
// This file contains image layout definition. They should be consistent
// with *.bib files and addrtab_cfg.inc mapping table. Generally saying
// *.bib files constants should be determined from constants in this file
// and mapping file (*.bib file values are virtual cached addresses for
// Windows CE OS, but physical address for IPL/EBOOT if they don't use
// MMU).
//

#pragma once

#include "lpc32xx_chip.h"

//------------------------------------------------------------------------------
//
//  Define: DEVICE_RAM_xxx
//
//  Phytec LPC32XX has 64MB SDRAM located at physical address 0x80000000.
//
#define DEVICE_RAM_PA                   EMC_DYCS0_BASE
#define DEVICE_RAM_SIZE                 0x04000000

#define DEVICE_CS0_PA                   EMC_CS0_BASE
#define DEVICE_CS1_PA                   EMC_CS1_BASE
#define DEVICE_CS2_PA                   EMC_CS2_BASE
#define DEVICE_CS3_PA                   EMC_CS3_BASE

#define DEVICE_IRAM_PA                  0x08000000


//------------------------------------------------------------------------------
//
//  Define: IMAGE_SHARE_ARGS_xxx
//
//  Following constants define location and maximal size of arguments shared
//  between loader and kernel. For actual structure see args.h file. WHen using
//  IRAM for the LCD, this may conflict in the ARGs structure is accessed after
//  the LCD is initialized
//
#define IMAGE_SHARE_ARGS_PA         0x00000800
#define IMAGE_SHARE_ARGS_SIZE       0x00000800

//------------------------------------------------------------------------------
//
//  Define: IMAGE_WINCE_xxx
//
//  Following constants define Windows CE OS image layout.
//
// Start of extended DRAM
#define IMAGE_WINCE_EXT_DRAM            0x84000000
#define IMAGE_WINCE_EXT_DRAM_SIZE       0x04000000

// Start of DMA buffer area
#define IMAGE_WINCE_DMA_BUFFERS_PA      0x83F00000//0x83D00000
#define IMAGE_WINCE_DMA_BUFFERS_SIZE    0x00100000

// DMA ethernet area
#define IMAGE_WINCE_ETHDMA_PA           0x83F00000//IMAGE_WINCE_DMA_BUFFERS_PA
#define IMAGE_WINCE_ETHDMA_SIZE         0x00010000

// Start of LCD data area
#define IMAGE_WINCE_LCD_BUFFERS_PA      0x83D00000//0x83E00000
#define IMAGE_WINCE_LCD_BUFFERS_SIZE    0x00200000

//------------------------------------------------------------------------------
//
//  Define: IMAGE_EBOOT_xxx
//
//  Following constants define EBOOT image layout. These locations must match
//  eboot.bib locations. These locations are in IRAM. IRAM is cached at address
//  0x00000000 and uncached at address 0x08000000. The DMA buffers are setup
//  in uncached memory, while all other EBOOT data is in cached memory.
//
#define IMAGE_EBOOT_CODE_PA             0x83FC0000
#define IMAGE_EBOOT_CODE_SIZE           0x00040000

#define IMAGE_EBOOT_VECTORS_PA          0x00000000
#define IMAGE_EBOOT_VECTORS_SIZE        0x00000800

#define IMAGE_EBOOT_DATA_PA             0x00001000
#define IMAGE_EBOOT_DATA_SIZE           0x00023000

#define IMAGE_EBOOT_ETHDMA_PA           0x08024000
#define IMAGE_EBOOT_ETHDMA_SIZE         0x00010000

#define IMAGE_EBOOT_SVCSTK_PA           0x00034000
#define IMAGE_EBOOT_SVCSTK_SIZE         0x00007000

#define IMAGE_EBOOT_IRQSTK_PA           0x0003B000
#define IMAGE_EBOOT_IRQSTK_SIZE         0x00001000

#define IMAGE_EBOOT_MMUTABLE_PA         0x0003C000
#define IMAGE_EBOOT_MMUTABLE_SIZE       0x00004000

// Where images are downloaded in EBOOT
#define IMAGE_EBOOT_RAM_PA              0x80000000
#define IMAGE_EBOOT_RAM_SIZE            0x04000000

//------------------------------------------------------------------------------
