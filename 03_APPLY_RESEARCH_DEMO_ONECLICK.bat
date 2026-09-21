@echo off
setlocal
cd /d "%~dp0"
echo ============================================================
echo  Apply Research Demo / Development Preview positioning
echo ============================================================
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\apply_research_demo.ps1"
if errorlevel 1 (
  echo.
  echo UPDATE FAILED. Nothing was automatically pushed.
  pause
  exit /b 1
)
echo.
echo Public wording updated successfully.
echo.
echo Next:
echo   - Run 04_SYNC_RESEARCH_DEMO_SOURCE.bat to sync these text changes.
echo   - Or use 05_APPLY_AND_SYNC_RESEARCH_DEMO_ONECLICK.bat next time.
echo   - Run 01_BUILD_SETUP_ONECLICK.bat only when you want a refreshed installer.
echo.
pause
