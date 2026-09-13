@echo off
setlocal
cd /d "%~dp0"
echo This product repository now uses the no-stitch release uploader.
echo It pushes the source and uploads the standalone Setup.exe Release asset.
echo.
call "%~dp002_UPLOAD_GITHUB_ONECLICK.bat"
exit /b %ERRORLEVEL%
