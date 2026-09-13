@echo off
setlocal
cd /d "%~dp0"
title Robust Hole Metrology - Build Standalone Setup
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_setup.ps1"
set "ERR=%ERRORLEVEL%"
echo.
if not "%ERR%"=="0" (
  echo BUILD/PACKAGE FAILED. Nothing was uploaded.
) else (
  echo BUILD/PACKAGE COMPLETE.
)
echo.
pause
exit /b %ERR%
