// Standard includes

#include "lpc32xx_rtc.h"
#include "lpc32xx_otg.h"
#include "lpc32xx_uart.h"
#include "lpc32xx_slcnand.h"
#include "lpc32xx_mlcnand.h"
#include "lpc32xx_clkpwr.h"
#include "lpc32xx_intc.h"
#include "lpc32xx_mstimer.h"
#include "lpc32xx_gpio.h"
#include "lpc32xx_dmac.h"
#include "lpc32xx_mac.h"
#include "lpc32xx_sdcard.h"

#define TIME_BOMB_XOR	0x5987adc7

extern P_ETHERNET_REGS_T			g_pEthernet;	//	Ethernet Controller mapped registers

extern void InitPeripheralMapping(BOOL bForce);


//------------------------------------------------------------------------------
//! End of $URL: http://centaure/svn/adeneo_corp-bsp_lcp3180/TAGS/OF4580-V1.0.0/PLATFORM/PhyCore_LPC3250/SRC/INC/PeripheralMapping.h $
//------------------------------------------------------------------------------

//
//! @}
//
