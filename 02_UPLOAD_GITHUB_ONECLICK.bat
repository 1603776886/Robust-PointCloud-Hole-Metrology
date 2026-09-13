@echo off
setlocal
cd /d "%~dp0"
title Robust Hole Metrology - GitHub Source + Release Upload
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\publish_release.ps1"
set "ERR=%ERRORLEVEL%"
echo.
if not "%ERR%"=="0" (
  echo UPLOAD STOPPED SAFELY.
) else (
  echo GITHUB UPLOAD COMPLETE.
)
echo.
pause
exit /b %ERR%
