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
// lpc32xx_fmd.h
//
// NAND FLASH functions
//

#pragma once

// Uncomment this pragma to enable FLASH locks (not needed)
//#define FMDACCESSLOCKS

#define BYPASSBLOCKS 0 // 0=ignored
#define MAXERASETIMEINMS 4 // 3~4mS
#define MAXEREADTIMEINMS 2 // 1~2mS
#define MAXEWRITETIMEINMS 2 // 1~2mS

#define CMD_RANDOMDATAOUTPUT        0x05        //  Reset column read index
#define CMD_RANDOMDATAOUTPUT2       0xE0        //  Reset column read index confirm
#define CMD_RANDOMDATAINPUT         0x85        //  Reset column write index

// Extra data area offsets
#define BADBLOCKMARKER              517         // Bad lock marker from factor at offset 2048
#define SECTORINFOOFFSET            512         // Start of OEM sectorinfo structure
#define ECCAREA1                    520         // Start of ECC area 1 for bytes 0 to 511

/***********************************************************************
 * NAND part number : ST Micro NAND128-A/NAND256-A/NAND512-A/NAND01G-A
 * Size             : 128Mbits/256Mbits/512Mbits/1Gbits
 * Ids              : 0x73/(0x35,0x75)/(0x36,0x76)/(0x39,0x79)
 * Number of blocks : 1024/2048/4096/8192
 * Pages per block  : 32
 * Bytes per page   : 528 (512 + 16 spare)
 * Bad block marker : (Offset 512+6)
 * Bus size         : 8 bits
 * Other info       : None
 **********************************************************************/
