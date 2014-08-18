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
// lbecc.h
//
// ECC correction functions
//

#pragma once

#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif
    
/* ECC error check types */
typedef enum {
    ECC_NOERR,           /* ECC and data both correct */
    ECC_CORRECTED,       /* Data has been corrected */
    ECC_AREA_ERR,        /* Data is correct, but ECC is invalid */
    ECC_NOTCORRECTABLE   /* Data is not correctable */
} ECC_ERROR_T;
    
/* Generate parity lookup tables for software mode */
void eccInitTables(void);
    
/* Generate an software ECC for a range of data */
unsigned char eccGenerate512(unsigned short *eccbuf, unsigned char *datbuf);
    
/* Check and correct ECC and data */
ECC_ERROR_T eccCheckAndCorrect(unsigned short *eccgood, unsigned short *eccerr,
    unsigned char *buf);
    
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
