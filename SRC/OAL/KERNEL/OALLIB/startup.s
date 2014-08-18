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
; startup.s
;
; Kernel startup routine
;

        INCLUDE kxarm.h
        INCLUDE bsp_cfg.inc
       
        IMPORT  KernelStart

        TEXTAREA
        
;-------------------------------------------------------------------------------
;
; StartUp
;
; This function is entry point to Windows CE OS. It should be called
; in state with deactivated MMU.
;
; Main system initialization is performed in stage 1 loader and/or eboot.
;
        LEAF_ENTRY StartUp

        ;---------------------------------------------------------------
        ; Initialize cache
        ;---------------------------------------------------------------

        ; Clean & invalidate D cache
10      mrc     p15, 0, r15, c7, c14, 3
        bne     %b10
        
        ; Invalidate I cache
        mov     r0, #0
        mcr     p15, 0, r0, c7, c5, 0
       
        ; Enable I and D caches and write buffer
        mrc     p15, 0, r0, c1, c0, 0
        orr     r0, r0, #(1 :SHL: 12)           ; I-cache
        orr     r0, r0, #(1 :SHL: 2)            ; D-cache
        orr     r0, r0, #(1 :SHL: 3)            ; write buffer
        mcr     p15, 0, r0, c1, c0, 0

        ;---------------------------------------------------------------
        ; Jump to WinCE KernelStart
        ;---------------------------------------------------------------
        ; Compute the OEMAddressTable's physical address and 
        ; load it into r0. KernelStart expects r0 to contain
        ; the physical address of this table. The MMU isn't 
        ; turned on until well into KernelStart.  

        add     r0, pc, #g_oalAddressTable - (. + 8)
        bl      KernelStart
        b       .

        ; Include memory configuration file with g_oalAddressTable
        INCLUDE addrtab_cfg.inc

        ENTRY_END 

        END

;------------------------------------------------------------------------------

