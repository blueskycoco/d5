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
; image_cfg.inc
;
; This file is ARM assembler version of image_cfg.h file 
;

;-------------------------------------------------------------------------------
;
;  Define: IMAGE_EBOOT_xxx
;
;  Following constants define EBOOT image layout.
;
IMAGE_EBOOT_CODE_PA			EQU     0x83FC0000
IMAGE_EBOOT_CODE_SIZE	    EQU     0x00040000
	
IMAGE_EBOOT_DATA_PA	        EQU     0x00001000
IMAGE_EBOOT_DATA_SIZE	    EQU     0x00033000

IMAGE_EBOOT_IRQSTK_PA	    EQU     0x0003B000
IMAGE_EBOOT_IRQSTK_SIZE	    EQU     0x00001000

IMAGE_EBOOT_SVCSTK_PA       EQU     0x00034000
IMAGE_EBOOT_SVCSTK_SIZE     EQU     0x00007000
;------------------------------------------------------------------------------

                            END

