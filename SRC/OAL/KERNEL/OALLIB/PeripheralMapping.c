//-----------------------------------------------------------------------------
//! \addtogroup	EBOOT
//! @{
//!
//  
//-----------------------------------------------------------------------------

// Standard includes
#include <windows.h>
#include <oal.h>
#include "Lpc32xx_chip.h"
#include "peripheralMapping.h"


P_ETHERNET_REGS_T			g_pEthernet;	//	Ethernet Controller mapping


//------------------------------------------------------------------------------
//!	\fn			void InitPeripheralMapping(BOOL bForce)
//
//!  \brief		This function is called to initialize register mapping.
//!
//! \param		bForce : if true, re-initializes
//
//! \return		None
//------------------------------------------------------------------------------
void InitPeripheralMapping(BOOL bForce)
{

	static BOOL bInitialized = FALSE;
	if ((bInitialized == FALSE) || (bForce == TRUE))
	{
		g_pEthernet				= (P_ETHERNET_REGS_T)			OALPAtoVA(ETHERNET_BASE,	FALSE);

		bInitialized = TRUE;
	}
}


//------------------------------------------------------------------------------
//! End of $URL: http://centaure/svn/adeneo_corp-bsp_lcp3180/TAGS/OF4580-V1.0.0/PLATFORM/PhyCore_LPC3250/SRC/NXP/LPC3250/MISC/MAP/PeripheralMapping.c $
//------------------------------------------------------------------------------

//
//! @}
//