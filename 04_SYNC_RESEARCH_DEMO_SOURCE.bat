@echo off
setlocal
cd /d "%~dp0"
echo ============================================================
echo  Sync Research Demo source/docs to GitHub (no Release/gh)
echo ============================================================
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\sync_research_demo_source.ps1"
if errorlevel 1 (
  echo.
  echo SOURCE SYNC STOPPED SAFELY.
  pause
  exit /b 1
)
echo.
echo SOURCE SYNC COMPLETE.
pause
