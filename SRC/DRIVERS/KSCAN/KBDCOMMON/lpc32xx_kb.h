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
// lpc32xx_kb.h
//
// PDD for the LPC32xx matrix keyboard driver
//

#pragma once

#include <windows.h>
#include <types.h>

// Matrix keyboard mask
#define MATRIX_PDD       0x10

// Matrix size used for the driver
#define MATRIXSIZE 4

// Matrix type
#define MATRIXTYPE _T("Matrix_409")

// Matrix scan time can be adjusted by changing the MATRIXDEBCYCLES and
// MATRIXDELCYCLES values below. A scan time of the entire matrix (in Hz)
// can be approximated by the following formula:
//  Scan frequency = 32768 / ((MATRIXSIZE * MATRIXDEBCYCLES) + (32 * MATRIXDELCYCLES))

// Number of debounce cycles per matrix scan
#define MATRIXDEBCYCLES 3 // 3 clocks per MATRIXSIZE

// Number of '32 clock' periods between each matrix scan state
#define MATRIXDELCYCLES 34 // Makes about a 30Hz scan rate