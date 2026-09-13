@echo off
setlocal
cd /d "%~dp0"
title Robust Hole Metrology - Build and Upload
call "%~dp001_BUILD_SETUP_ONECLICK.bat"
if errorlevel 1 exit /b %ERRORLEVEL%
call "%~dp002_UPLOAD_GITHUB_ONECLICK.bat"
exit /b %ERRORLEVEL%
