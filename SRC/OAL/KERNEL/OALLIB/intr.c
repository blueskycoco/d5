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
// intr.h
//
// This file implements LPC32xx interrupt functions.
//
#include <windows.h>
#include <ceddk.h>
#include <nkintr.h>
#include <nkexport.h>
#include "oal_log.h"
#include "oal_memory.h"
#include "oal_io.h"
#include "oal_timer.h"
#include "oal_intr.h"
#include <oal_ilt.h>

#include "bsp.h"
#include "dma.h"
#include "lpc32xx_intc.h"
#include "lpc32xx_dmac.h"

#define VIRTINTR 0xFFFFFFFF

// IRQ support list for SYSINTR mapping, not all interrupts are
// supported, as there is a limit of about 56 user interrupts in
// WinCE 6.0. The order of this list should match the OAL_INTR_IRQ_xxx
// order as specified in bsp.h to work correctly. This list does not
// control priority.
typedef struct {
	UINT32             oal_intr_irq_num;
	INTERRUPT_SOURCE_T lpc32xx_irq_num;
} OAL_TO_HW_IRQ_T;
static const OAL_TO_HW_IRQ_T lp32xx_ierqsupp[OAL_INTR_IRQ_LAST + 1] = {
	{OAL_INTR_IRQ_UART5,       IRQ_UART_IIR5},
	{OAL_INTR_IRQ_UART3,       IRQ_UART_IIR3},
    {OAL_INTR_IRQ_NAND,        IRQ_FLASH},
	{OAL_INTR_IRQ_SD0,         IRQ_SD0},
    {OAL_INTR_IRQ_SD1,         IRQ_SD1},
    {OAL_INTR_IRQ_MSTIMER,     IRQ_MSTIMER},
	{OAL_INTR_IRQ_TIMER0,      IRQ_TIMER0},
	{OAL_INTR_IRQ_SSP0,        IRQ_SSP0},
	{OAL_INTR_IRQ_SSP1,        IRQ_SSP1},
	{OAL_INTR_IRQ_I2S1,        IRQ_I2S1},
    {OAL_INTR_IRQ_ETHERNET,    IRQ_ETHERNET},
    {OAL_INTR_IRQ_TS_P,        IRQ_TS_P},
    {OAL_INTR_IRQ_TS,          IRQ_TS_IRQ},
    {OAL_INTR_IRQ_I2C_1,       IRQ_I2C_1},
    {OAL_INTR_IRQ_KEY,         IRQ_KEY},
    {OAL_INTR_IRQ_USB_HOST,    IRQ_USB_HOST},
    {OAL_INTR_IRQ_USB_DEV_DMA, IRQ_USB_DEV_DMA},
    {OAL_INTR_IRQ_USB_DEV_LP,  IRQ_USB_DEV_LP},
	{OAL_INTR_IRQ_USB_DEV_HP,  IRQ_USB_DEV_HP},
    {OAL_INTR_IRQ_I2C,         IRQ_USB_I2C},
    {OAL_INTR_IRQ_GPIO_00,     IRQ_GPIO_00},
    {OAL_INTR_IRQ_GPIO_01,     IRQ_GPIO_01},
    {OAL_INTR_IRQ_GPIO_02,     IRQ_GPIO_02},
    {OAL_INTR_IRQ_GPIO_03,     IRQ_GPIO_03},
    {OAL_INTR_IRQ_GPIO_04,     IRQ_GPIO_04},
    {OAL_INTR_IRQ_GPIO_05,     IRQ_GPIO_05},
    {OAL_INTR_IRQ_P0_P1_IRQ,   IRQ_P0_P1_IRQ},
	{OAL_INTR_IRQ_OTG_TIMER,   IRQ_USB_OTG_TIMER},
    {OAL_INTR_IRQ_OTG_ATC,     IRQ_USB_OTG_ATX},
    {OAL_INTR_IRQ_OTG_I2C,     IRQ_USB_I2C},
    {OAL_INTR_IRQ_GPI_07,      IRQ_GPI_07},
    {OAL_INTR_IRQ_GPI_00,      IRQ_GPI_00},
    {OAL_INTR_IRQ_GPI_01,      IRQ_GPI_01},
    {OAL_INTR_IRQ_GPI_02,      IRQ_GPI_02},
    {OAL_INTR_IRQ_GPI_03,      IRQ_GPI_03},
    {OAL_INTR_IRQ_GPI_04,      IRQ_GPI_04},
    {OAL_INTR_IRQ_GPI_05,      IRQ_GPI_05},
    {OAL_INTR_IRQ_GPI_06,      IRQ_GPI_06},
    {OAL_INTR_IRQ_I2C_2,       IRQ_I2C_2},

	// Required system interrupts
	{OAL_INTR_IRQ_RTC,         IRQ_RTC},
    {OAL_INTR_IRQ_TICK,        IRQ_TIMER3},
    {OAL_INTR_IRQ_PROFILER,    IRQ_TIMER2},

	// Virtual interrupts are not mapped
    {OAL_INTR_DMACH_0,         VIRTINTR},
    {OAL_INTR_DMACH_1,         VIRTINTR},
    {OAL_INTR_DMACH_2,         VIRTINTR},
    {OAL_INTR_DMACH_3,         VIRTINTR},
    {OAL_INTR_DMACH_4,         VIRTINTR},
    {OAL_INTR_DMACH_5,         VIRTINTR},
    {OAL_INTR_DMACH_6,         VIRTINTR}
};

// Map list for LPC32xx IRQ to BSP IRQ. This is used to convert a LPC32xx
// interrupt ID (0..95) to a WinCE IRQ ID of type OAL_INTR_IRQ_xxxx.
static UINT32 lp32xxToBSPIRQ[IRQ_END_OF_INTERRUPTS];

// Pointers to interrupt controller registers
static INTC_REGS_T *pMIC, *pSIC1, *pSIC2;

// Pointer to DMA registers (for virtual DMA mapping)
static DMAC_REGS_T *pDMARegs;

#ifdef IMGPROFILFER
#if IMGPROFILFER==1
//  Function pointer to profiling timer ISR routine
extern PFN_PROFILER_ISR g_pProfilerISR;
#endif
#endif

//------------------------------------------------------------------------------
//
// int_get_controller
//
// Returns the controller ID and mask based on the interrupt source
//
static BOOL int_get_controller(INTERRUPT_SOURCE_T source, 
							   INTC_REGS_T **pIntc_base,
							   UINT32 *pBit_pos)
{
	BOOL ret_value = TRUE;
	/* Determine the interrupt controller */
	if (source < IRQ_SIC1_BASE)
	{
		*pIntc_base = pMIC;
		*pBit_pos = (UINT32) source;
	}
	else if ((source >= IRQ_SIC1_BASE) && (source < IRQ_SIC2_BASE))
	{
		*pIntc_base = pSIC1;
		*pBit_pos = ((UINT32) source - IRQ_SIC1_BASE);
	}
	else if (source <= IRQ_END_OF_INTERRUPTS)
	{
		*pIntc_base = pSIC2;
		*pBit_pos = ((UINT32) source - IRQ_SIC2_BASE);
	}
	else 
	{
		*pIntc_base = 0;
		*pBit_pos = 0;
		ret_value = FALSE;
	}

	return ret_value;
}

//------------------------------------------------------------------------------
//
// OALIntrInit
//
// This function initializes the interrupt hardware and sets the default IRQ
// to SYSINTR mapping for some static peripherals.
//
BOOL OALIntrInit()
{
	int idx;

	// Save pointers to MIC, SIC1, and SIC2
	pMIC = (INTC_REGS_T *) OALPAtoVA((UINT32) MIC, FALSE);
	pSIC1 = (INTC_REGS_T *) OALPAtoVA((UINT32) SIC1, FALSE);
	pSIC2 = (INTC_REGS_T *) OALPAtoVA((UINT32) SIC2, FALSE);
	pDMARegs = (DMAC_REGS_T *) OALPAtoVA((UINT32) DMA_BASE, FALSE);
	RETAILMSG(1, (TEXT("OAL  interrupt init\r\n")));
    // Initialize interrupt mapping
    OALIntrMapInit();

	// Clear backmapping for LPC32xx to BSP IRQ list
	for (idx = 0; idx < IRQ_END_OF_INTERRUPTS; idx++)
	{
		lp32xxToBSPIRQ [idx] = OAL_INTR_IRQ_UNDEFINED;
	}

	// Setup backmapping for non-virtual interrupts
	for (idx = 0; idx < OAL_INTR_VIRT_FIRST; idx++)
	{
		lp32xxToBSPIRQ [lp32xx_ierqsupp [idx].lpc32xx_irq_num] =
			lp32xx_ierqsupp [idx].oal_intr_irq_num;
	}

    // Setup static interrupt mappings
    OALIntrStaticTranslate(SYSINTR_RESCHED, OAL_INTR_IRQ_TICK);
	lp32xxToBSPIRQ[IRQ_TIMER3] = OAL_INTR_IRQ_TICK;
    OALIntrStaticTranslate(SYSINTR_RTC_ALARM, OAL_INTR_IRQ_RTC);
	lp32xxToBSPIRQ[IRQ_RTC] = OAL_INTR_IRQ_RTC;
#ifdef IMGPROFILFER
#if IMGPROFILFER==1
    OALIntrStaticTranslate(SYSINTR_PROFILE, OAL_INTR_IRQ_PROFILER);
	lp32xxToBSPIRQ[IRQ_TIMER2] = OAL_INTR_IRQ_PROFILER;
#endif
#endif

	// Static map of USB OHCI interrupt makes using the WinCE OHCI
	// HCD driver easier
    OALIntrStaticTranslate(SYSINTR_FIRMWARE, OAL_INTR_IRQ_USB_HOST);
	lp32xxToBSPIRQ[IRQ_USB_HOST] = OAL_INTR_IRQ_USB_HOST;

	// Initialize main interrupt controller
	pMIC->er = 0x00000000;         // disable all interrupt sources
	pMIC->rsr = 0xFFFFFFFF;        // clear all edge triggered interrupts
	pMIC->apr = MIC_APR_DEFAULT;   // set polarity for all internal irqs
	pMIC->atr = MIC_ATR_DEFAULT;   // set act. types for all internal irqs
	pMIC->itr = (_BIT(IRQ_SUB1FIQ) | _BIT(IRQ_SUB2FIQ));

	// Initialize sub interrupt controller 1
	pSIC1->er = 0x00000000;        // disable all interrupt sources
	pSIC1->rsr = 0xFFFFFFFF;       // clear all edge triggered interrupts
	pSIC1->apr = SIC1_APR_DEFAULT; // set polarity for internal irqs
	pSIC1->atr = SIC1_ATR_DEFAULT; // set act-types for internal irqs
	pSIC1->itr = 0x00000000;       // set all interrupts as irqs

	// Initialize sub interrupt controller 2
	pSIC2->er = 0x00000000;        // disable all interrupt sources
	pSIC2->rsr = 0xFFFFFFFF;       // clear all edge triggered interrupts
	pSIC2->apr = SIC2_APR_DEFAULT; // set polarity for internal irqs
	pSIC2->atr = SIC2_ATR_DEFAULT; // set act. types for internal irqs
	pSIC2->itr = 0x00000000;       // set all interrupts as irqs

    // Enable sub-IRQ handlers and DMA on main interrupt controller
	pMIC->er = (_BIT(IRQ_SUB1IRQ) | _BIT(IRQ_SUB2IRQ) | _BIT(IRQ_DMA));

	return TRUE;
}

//------------------------------------------------------------------------------
//
// findfirstbit
//
// Returns the first high bit in a word
//
#if 0
static UINT32 findfirstbit(UINT32 val)
{
	UINT32 msk = 0;

	while ((val & _BIT(msk)) == 0)
	{
		msk++;
	}

	return msk;
}
#else
// Use assembly version, it's much faster
extern UINT32 findfirstbit(UINT32 val);
#endif
//------------------------------------------------------------------------------
//
// IntrEnableVirtual
//
// Used for enabling and disabling virtual interrupts (DMA).
//
static BOOL IntrEnableVirtual(OAL_TO_HW_IRQ_T *pIrqHw,
							  BOOL enable)
{
	UINT32 dmaChan;
	BOOL rc = FALSE;

	// For now, these only apply to DMA
	if ((pIrqHw->oal_intr_irq_num >= OAL_INTR_DMACH_0) &&
		(pIrqHw->oal_intr_irq_num <= OAL_INTR_DMACH_6))
	{
		// Get DMA channel
		dmaChan = (UINT32) pIrqHw->oal_intr_irq_num - OAL_INTR_DMACH_0;

		// Only works when DMA channel has been enabled
		if (enable == FALSE)
		{
			pDMARegs->dma_chan[dmaChan].config_ch &= ~(DMAC_CHAN_ITC | DMAC_CHAN_IE);
		}
		else
		{
			pDMARegs->dma_chan[dmaChan].config_ch |= (DMAC_CHAN_ITC | DMAC_CHAN_IE);
		}

		rc = TRUE;
	}

	return rc;
}

//------------------------------------------------------------------------------
//
// OALIntrEnableIrqs
//
// The IRQ passed in is of a value of type OAL_INTR_IRQ_xxx from bsp.h. This
// needs to be converted to an LPC32XX type (INTERRUPT_SOURCE_T) to be disabled
// in hardware.
//
BOOL OALIntrEnableIrqs(UINT32 count, const UINT32 *pIrqs)
{
    BOOL rc = TRUE;
	UINT32 i, bit_pos;
	INTERRUPT_SOURCE_T lirq;
	INTC_REGS_T *pIntc;

    OALMSG((OAL_INTR&&OAL_FUNC), (L"+OALIntrEnableIrqs\r\n"));
	// Enable each IRQ
    for (i = 0; i < count; i++)
	{
		// Out of range?
		if (pIrqs[i] <= OAL_INTR_IRQ_LAST)
		{
			lirq = lp32xx_ierqsupp[pIrqs[i]].lpc32xx_irq_num;
			if (lirq == VIRTINTR)
			{
				// Interrupt is virtual, call virtual interrupt handler
				rc = IntrEnableVirtual((OAL_TO_HW_IRQ_T *) &pIrqs[i], TRUE);
			}
		    else if (int_get_controller(lirq, &pIntc, &bit_pos) == TRUE)
			{
			    // Get the interrupt controller for the given interrupt source
				pIntc->er |= _BIT(bit_pos);
			}
		}
		else
		{
			rc = FALSE;
		}
    }

    OALMSG((OAL_INTR&&OAL_FUNC), (L"-OALIntrEnableIrqs(rc = %d)\r\n", rc));

	return rc;    
}

//------------------------------------------------------------------------------
//
// OALIntrDisableIrqs
//
// Disable one or more LPC32XX IRQs.
//
VOID OALIntrDisableIrqs(UINT32 count, const UINT32 *pIrqs)
{
	INTERRUPT_SOURCE_T lirq;
	INTC_REGS_T *pIntc;
	UINT32 i, bit_pos = 0;

    OALMSG((OAL_INTR&&OAL_FUNC), (L"+OALIntrDisableIrqs(%d, 0x%08x)\r\n",
		count, pIrqs));
    
	// Disable each IRQ
    for (i = 0; i < count; i++) {
		// Out of range?
		if (pIrqs[i] <= OAL_INTR_IRQ_LAST)
		{
			lirq = lp32xx_ierqsupp[pIrqs[i]].lpc32xx_irq_num;
			if (lirq == VIRTINTR)
			{
				// Interrupt is virtual, call virtual interrupt handler
				IntrEnableVirtual((OAL_TO_HW_IRQ_T *) &pIrqs[i], FALSE);
			}
		    else if (int_get_controller(lirq, &pIntc, &bit_pos) == TRUE)
			{
			    // Get the interrupt controller for the give interrupt source
				pIntc->er &= ~_BIT(bit_pos);
			}
		}
	}

    OALMSG((OAL_INTR&&OAL_FUNC), (L"-OALIntrDisableIrqs\r\n"));
}

//------------------------------------------------------------------------------
//
// IntrDoneVirtual
//
// Used for clearing and re-enabling virtual interrupts (DMA).
//
static BOOL IntrDoneVirtual(OAL_TO_HW_IRQ_T *pIrqHw)
{
	UINT32 dmaChan, msk;
	BOOL rc = FALSE;

	// For now, these only apply to DMA
	if ((pIrqHw->oal_intr_irq_num >= OAL_INTR_DMACH_0) &&
		(pIrqHw->oal_intr_irq_num <= OAL_INTR_DMACH_6))
	{
		// Get DMA channel
		dmaChan = (UINT32) pIrqHw->oal_intr_irq_num - OAL_INTR_DMACH_0;

		// Only works when DMA channel has been enabled
		msk = _BIT(dmaChan);

		// Clear latched TC and ERR interrupt flags
		pDMARegs->int_tc_clear = msk;
		pDMARegs->int_err_clear = msk;

		// Re-enable channel
		pDMARegs->dma_chan[dmaChan].config_ch |= (DMAC_CHAN_ITC | DMAC_CHAN_IE);

		rc = TRUE;
	}

	return rc;
}

//------------------------------------------------------------------------------
//
// OALIntrDoneIrqs
//
// Re-enable obe or more LPC32xx IRQs.
//
VOID OALIntrDoneIrqs(UINT32 count, const UINT32 *pIrqs)
{
	INTERRUPT_SOURCE_T lirq;
	INTC_REGS_T *pIntc;
	UINT32 i,  bit_pos = 0;

    OALMSG((OAL_INTR&&OAL_FUNC), (
        L"+OALIntrDoneIrqs(%d, 0x%08x)\r\n", count, pIrqs));

    for (i = 0; i < count; i++) {
		// Out of range?
		if (pIrqs[i] <= OAL_INTR_IRQ_LAST)
		{
			lirq = lp32xx_ierqsupp[pIrqs[i]].lpc32xx_irq_num;
			if (lirq == VIRTINTR)
			{
				// Interrupt is virtual, call virtual interrupt handler
				IntrDoneVirtual((OAL_TO_HW_IRQ_T *) &pIrqs[i]);
			}
			else
			{
			    // Get the interrupt controller for the give interrupt source
				if (int_get_controller(lirq, &pIntc, &bit_pos) == TRUE)
				{
					// Clear pending interrupt
					pIntc->rsr = _BIT(bit_pos);

					// Re-enable interrupt
					pIntc->er |= _BIT(bit_pos);
				}
			}
		}
	}

    OALMSG((OAL_INTR&&OAL_FUNC), (L"-OALIntrDoneIrqs\r\n"));
}

//------------------------------------------------------------------------------
//
// OALIntrRequestIrqs
//
// This function returns IRQs for CPU/SoC devices based on their
// physical address.
//
BOOL OALIntrRequestIrqs(DEVICE_LOCATION *pDevLoc, UINT32 *pCount, UINT32 *pIrqs)
{
    BOOL rc = FALSE;

    OALMSG((OAL_INTR&&OAL_FUNC), (L"+OALIntrRequestIrqs\r\n"));

    // This shouldn't happen
    if (*pCount < 1) goto cleanUp;

	switch (pDevLoc->IfcType) {
    case Internal:
        switch ((ULONG)pDevLoc->PhysicalLoc) {
        case ETHERNET_BASE:
            *pCount = 1;
            pIrqs[0] = OAL_INTR_IRQ_ETHERNET;
            rc = TRUE;
            break;
        }
        break;
    }

    OALMSG((OAL_INTR&&OAL_FUNC&&OAL_VERBOSE), (
        L"+OALIntrRequestIrqs(0x%x->%d/%d/0x%x/%d, 0x%x, 0x%x)\r\n",
        pDevLoc, pDevLoc->IfcType, pDevLoc->BusNumber, pDevLoc->LogicalLoc,
        pDevLoc->Pin, pCount, pIrqs[0]
    ));

cleanUp:
    OALMSG((OAL_INTR&&OAL_FUNC), (L"-OALIntrRequestIrqs(rc = %d)\r\n", rc));
    return rc;
}

//------------------------------------------------------------------------------
//
// OEMInterruptHandler
//
// Handles the LPC32XX interrupts and generates the correct sysIntr values.
//
ULONG OEMInterruptHandler(ULONG ra)
{
	DWORD dirq;
	INTERRUPT_SOURCE_T intsrc;
	INTC_REGS_T *pIntc;
    UINT32 sts, sysIntr = SYSINTR_NOP, bit_pos = 0;

	// Read MIC status masking off IRQ bits
	sts = pMIC->sr & ~(_BIT(IRQ_SUB1IRQ) | _BIT(IRQ_SUB2IRQ) |
		_BIT(IRQ_SUB1FIQ) | _BIT(IRQ_SUB2FIQ));

	// Interrupt on MIC?
	if (sts != 0)
	{
		// Find first active interrupt
		bit_pos = (INTERRUPT_SOURCE_T) findfirstbit(sts);
		intsrc = bit_pos;
		pIntc = pMIC;
	}
	else if (pSIC1->sr != 0)
	{
		// Find first active interrupt
		bit_pos = (INTERRUPT_SOURCE_T) findfirstbit(pSIC1->sr);
		intsrc = bit_pos + IRQ_SIC1_BASE;
		pIntc = pSIC1;
	}
	else if (pSIC2->sr != 0)
	{
		// Find first active interrupt
		bit_pos = (INTERRUPT_SOURCE_T) findfirstbit(pSIC2->sr);
		intsrc = bit_pos + IRQ_SIC2_BASE;
		pIntc = pSIC2;
	}
	else
	{
		// This should never happen
		return SYSINTR_NOP;
	}

	// Get BSP IRQ mapped to this hardware IRQ
	sts = lp32xxToBSPIRQ [intsrc];
	if ((sts == OAL_INTR_IRQ_UNDEFINED) && (intsrc != IRQ_DMA))
	{
		// Unmapped interrupt, disable interrupt
		pIntc->er &= ~_BIT(bit_pos);

		return SYSINTR_NOP;
	}

	// Is this the system timer interrupt?
    if (sts == OAL_INTR_IRQ_TICK)
    {
        sysIntr = OALTimerIntrHandler();
    }
#ifdef IMGPROFILFER
#if IMGPROFILFER==1
    else if (sts == OAL_INTR_IRQ_PROFILER)
    {
		// Disable profiler interrupt
		pIntc->er &= ~_BIT(bit_pos);

        // Call profiler interrupt
        if (g_pProfilerISR != NULL)
        {
            sysIntr = g_pProfilerISR(ra);
        }
    }
#endif
#endif
	else if (intsrc == IRQ_DMA)
    {
		// DMA interrupt requires remapping to a virtual channel
		// INTR value
		dma_get_int_ch(&dirq);
		IntrEnableVirtual((OAL_TO_HW_IRQ_T *) &lp32xx_ierqsupp[dirq], FALSE);
        sysIntr = OALIntrTranslateIrq(dirq);
	}
	else
	{
		// Mapped interrupt, return SYSINTR value for the interrupt
		sysIntr = OALIntrTranslateIrq(sts);

		// Disable interrupt
		pIntc->er &= ~_BIT(bit_pos);
	}

    return sysIntr;
}

//------------------------------------------------------------------------------
//
// OEMInterruptHandlerFIQ
//
// Stub function for ARM FIQ handler
//
void OEMInterruptHandlerFIQ()
{
}
