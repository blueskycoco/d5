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
;------------------------------------------------------------------------------
;
;  File:  jumpto.s

        INCLUDE kxarm.h

        TEXTAREA

;-------------------------------------------------------------------------------
;
;  Function:  JumpTo
;
        LEAF_ENTRY JumpTo

flushonly
        ; Flash data cache
        MRC     p15, 0, r15, c7, c10, 3
        BNE     flushonly

        ; Ensure the MMU and caches are disabled
        mrc     p15, 0, r1, c1, c0, 0
        ldr     r2,=~0x00001005
        and     r1, r1, r2
        mcr     p15, 0, r1, c1, c0, 0

        ; Invalidate both caches
        mcr     p15, 0, r1, c7, c7, 0

        ; Re-enable instruction cache
        mrc     p15, 0, r1, c1, c0, 0
        ldr     r2,=0x1000
        orr     r1, r1, r2
        mcr     p15, 0, r1, c1, c0, 0

        mov     pc, r0
        b       .

;------------------------------------------------------------------------------

        END
