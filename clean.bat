@REM
@REM Copyright (c) Microsoft Corporation.  All rights reserved.
@REM
@REM
@REM Use of this source code is subject to the terms of the Microsoft end-user
@REM license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
@REM If you did not accept the terms of the EULA, you are not authorized to use
@REM this source code. For a copy of the EULA, please see the LICENSE.RTF on your
@REM install media.
@REM

@echo off

Echo Cleaning platform build logs
del /s build.*
Echo Cleaning cesysgen\files
rd /s /q .\CESYSGEN\files
del /f/q *.bif >nul 2>&1
Echo Cleaning target
rd /s /q .\target
Echo Cleaning lib
rd /s /q .\lib

Echo Cleaning out platform object files
for /F %%f in ('dir /s /ad /b .\obj') do del /f/s/q %%f >nul 2>&1 && rd /s/q %%f >nul 2>&1
