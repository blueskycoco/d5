;------------------------------------------------------------------------------
; Software that is described herein is for illustrative purposes only  
; which provides customers with programming information regarding the  
; products. This software is supplied "AS IS" without any warranties.  
; NXP Semiconductors assumes no responsibility or liability for the 
; use of the software, conveys no license or title under any patent, 
; copyright, or mask work right to the product. NXP Semiconductors 
; reserves the right to make changes in the software without 
; notification. NXP Semiconductors also make no representation or 
; warranty that such application will be suitable for the specified 
; use without further testing or modification. 
; Copyright NXP Semiconductors
;------------------------------------------------------------------------------
;
; bsp_clkpwr_support_halt.s
;
; CPU halt function for power reduction
;

        INCLUDE kxarm.h

        TEXTAREA

;-------------------------------------------------------------------------------
;
;  Function:  clkpwr_halt_cpu
;
;  CPU halt function. Halts CPU until an interrupt occurs.
;
        LEAF_ENTRY clkpwr_halt_cpu

10      mcr     p15, 0, r0, c7, c0, 4
        bx      lr
        
        ENTRY_END clkpwr_halt_cpu
        
;------------------------------------------------------------------------------

        END
