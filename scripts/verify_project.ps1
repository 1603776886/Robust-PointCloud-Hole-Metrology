$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'preflight.ps1')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "`nFor zero-dependency execution, use the Setup.exe attached to GitHub Releases." -ForegroundColor Cyan
Write-Host 'Source rebuild verification requires the documented Visual Studio + Qt + PCL publisher/developer environment.' -ForegroundColor Gray
exit 0
