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
; addrtab_cfg.inc
;
; This file is used to define g_oalAddressTable. This table is passed to
; KernelStart to estabilish physical to virtual memory mapping. This table
; is used also in memory OAL module to map between physical and virtual
; memory addresses via OALPAtoVA/OALVAtoPA functions.
;
; The config.bib file defines image memory layout and it contains virtual
; cached memory addresses which must be synchronized with g_oalAddressTable.
; With each table change make sure that those constant are still valid.
;
;------------------------------------------------------------------------------
;  Export Definition

        EXPORT  g_oalAddressTable[DATA]

;------------------------------------------------------------------------------
;  Table format: cached address, physical address, size

g_oalAddressTable
        DCD     0x80000000, 0x80000000,  64     ; SDRAM
        DCD     0x84000000, 0x84000000,  64     ; SDRAM on 128MB boards
        DCD     0x90000000, 0xE0000000,  8      ; NOR FLASH
        DCD     0x91000000, 0xE1000000,  1      ; SDIO
        DCD     0x92000000, 0x00000000,  1      ; IRAM (256K) mirrored
        DCD     0x93000000, 0x08000000,  1      ; IRAM (256K)
        DCD     0x94000000, 0x20000000,  1      ; AHB peripherals 1
        DCD     0x95000000, 0x31000000,  1      ; AHB peripherals 2
        DCD     0x96000000, 0x40000000,  1      ; FAB peripherals
        DCD     0x00000000, 0x00000000,  0      ; end of table

;------------------------------------------------------------------------------

        END
