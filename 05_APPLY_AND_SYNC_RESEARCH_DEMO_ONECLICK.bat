@echo off
setlocal
cd /d "%~dp0"
echo ============================================================
echo  Apply Research Demo wording + sync to GitHub main
echo ============================================================
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\apply_research_demo.ps1"
if errorlevel 1 (
  echo.
  echo UPDATE FAILED. Nothing was pushed.
  pause
  exit /b 1
)
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\sync_research_demo_source.ps1"
if errorlevel 1 (
  echo.
  echo TEXT UPDATE SUCCEEDED, BUT GIT SYNC STOPPED SAFELY.
  echo You can rerun 04_SYNC_RESEARCH_DEMO_SOURCE.bat later.
  pause
  exit /b 1
)
echo.
echo ============================================================
echo  DONE - Research Demo wording is synchronized to GitHub
echo ============================================================
echo.
echo If you also want the installer metadata refreshed, run:
echo   01_BUILD_SETUP_ONECLICK.bat
echo.
pause
