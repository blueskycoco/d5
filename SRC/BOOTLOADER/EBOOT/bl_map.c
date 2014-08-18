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
// bl_map.c
//
// Bootloader virtual/physical address mapping functions
//

#include <eboot.h>

//------------------------------------------------------------------------------
//
// OALPAtoVA
//
// Returns a virtual address for the passed physical address
//
VOID* OALPAtoVA(UINT32 pa, BOOL cached)
{
	return (VOID *) pa;
}

//------------------------------------------------------------------------------
//
// OALVAtoPA
//
// Returns a physical address for the passed virtual address
//
UINT32 OALVAtoPA(VOID *va)
{
	return (UINT32) va;
}

//------------------------------------------------------------------------------
//
// BLVAtoPA
//
// Returns a load address for the passed bootloader load address. This function
// is normally used with EBOOT downloads for FLASH programming, but EBOOT FLASH
// programming is not supported by this bootloader. Use S1L instead.
//
VOID* BLVAtoPA(UINT32 address)
{
	// Just return the passed address
	return (VOID *) address;
}
