;
; Copyright (c) Microsoft Corporation.  All rights reserved.
;
;
; Use of this sample source code is subject to the terms of the Microsoft
; license agreement under which you licensed this sample source code. If
; you did not accept the terms of the license agreement, you are not
; authorized to use this sample source code. For the terms of the license,
; please see the license agreement between you and Microsoft or, if applicable,
; see the LICENSE.RTF on your install media or the root of your tools installation.
; THE SAMPLE SOURCE CODE IS PROVIDED "AS IS", WITH NO WARRANTIES.
;
; Portions Copyright (c) Texas Instruments.  All rights reserved. 
;
;------------------------------------------------------------------------------
;
;   File:  startup.s
;
;   Boot startup routine for NXP LPC32XX.
;

        OPT   2             ; Disable listing 

        INCLUDE kxarm.h     ; This defines the WinCE/ARM Calling Sequence Specification

        OPT   1             ; Reenable listing
        OPT   128           ; Disable listing of macro expansions

		IMPORT lpc32xxMain
        INCLUDE image_cfg.inc

        ;---------------------------------------------------------------
        ;   Function:  StartUp
        ;---------------------------------------------------------------
        ; This function is entry point to Windows CE EBOOT. The MMU
        ; may be enabled or disabled. Caches are assumed to be flushed
        ; (if enabled) prior to entry to this function. This function
        ; should be executed in SDRAM at this point. IRAM should be
        ; mapped to 0x00000000.

        STARTUPTEXT

        LEAF_ENTRY StartUp 

        ; Put the processor is in system mode with interrupts disabled
        mov     r0, #0x013:OR:0x080:OR:0x040
        msr     cpsr_cxsf, r0

        ; The MMU and caches are left enabled to improve performance
        ; during eboot operation

        ; Enter IRQ mode and setup the IRQ stack pointer
        mov     r0, #0x012:OR:0x080:OR:0x040
        msr     cpsr_cxsf, r0
        ldr     r0, =IMAGE_EBOOT_IRQSTK_PA
        ldr     r1, =IMAGE_EBOOT_IRQSTK_SIZE
        add     sp, r0, r1

        ; Enter SVC mode and setup the SVC stack pointer
        mov     r0, #0x013:OR:0x080:OR:0x040
        msr     cpsr_cxsf, r0
        ldr     r0, =IMAGE_EBOOT_SVCSTK_PA
        ldr     r1, =IMAGE_EBOOT_SVCSTK_SIZE
        add     sp, r0, r1

        ldr     r1, =lpc32xxMain
	    bx      r1

;------------------------------------------------------------------------------

        ; Include memory configuration file with g_oalAddressTable
        INCLUDE addrtab_cfg.inc

        ENTRY_END 

        END
