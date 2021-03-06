@REM
@REM WinCE BSP misc. file installation script
@REM
@REM Execute this batch file once after BSP installation
@REM Delete the file after execution.
@REM
@echo off

@REM Bootloader SD/MMC support
copy ..\VOIP_PXA270\SRC\BOOTLOADER\SDBOOTLIB\sdmdd.c .\SRC\BOOTLOADER\SDMMC
copy ..\VOIP_PXA270\SRC\BOOTLOADER\SDBOOTLIB\sdfat.c .\SRC\BOOTLOADER\SDMMC
copy ..\VOIP_PXA270\SRC\BOOTLOADER\SDBOOTLIB\sdmdd.h .\SRC\BOOTLOADER\INC
copy ..\VOIP_PXA270\SRC\BOOTLOADER\SDBOOTLIB\sdpdd.h .\SRC\BOOTLOADER\INC
copy ..\VOIP_PXA270\SRC\BOOTLOADER\INC\sdfat.h .\SRC\BOOTLOADER\INC

@REM Backlight MDD support
copy ..\MAINSTONEIII\SRC\DRIVERS\BACKLIGHT\MDD\bkldrvapi.cpp .\SRC\DRIVERS\BACKLIGHT\MDD
copy ..\MAINSTONEIII\SRC\DRIVERS\BACKLIGHT\MDD\bkldrvmain.cpp .\SRC\DRIVERS\BACKLIGHT\MDD
copy ..\MAINSTONEIII\SRC\DRIVERS\BACKLIGHT\MDD\bkli.h .\SRC\DRIVERS\BACKLIGHT\MDD
copy ..\MAINSTONEIII\SRC\DRIVERS\BACKLIGHT\MDD\bklpdd.h .\SRC\DRIVERS\BACKLIGHT\MDD
