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
echo IMPORTANT:
echo   1. Run 01_BUILD_SETUP_ONECLICK.bat once to rebuild the installer
echo      with the Research Demo display name.
echo   2. Then use 02_UPLOAD_GITHUB_ONECLICK.bat, or manually commit/push
echo      these public-text changes if you only want to update the repository.
echo.
pause
