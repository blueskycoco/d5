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
; findfirstbit.s
;
; Returns first active bit in a passed value
;

        OPT     2       ; disable listing
        INCLUDE kxarm.h
        OPT     1       ; reenable listing
		
        TEXTAREA
	
        LEAF_ENTRY findfirstbit
        clz   r0, r0
        rsb   r0, r0, #31
        bx    lr
        ENTRY_END

        END

; EOF oemabort.s
