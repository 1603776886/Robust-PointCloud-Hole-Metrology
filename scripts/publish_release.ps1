$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$version = (Get-Content -LiteralPath (Join-Path $repo 'product\VERSION.txt') -Raw).Trim()
$tag = "v$version"
$setup = Join-Path $repo "out\release\RobustHoleMetrology-Dev-$version-Setup-x64.exe"

function Section([string]$s) { Write-Host "`n=== $s ===" -ForegroundColor Cyan }
function Pass([string]$s) { Write-Host "[PASS] $s" -ForegroundColor Green }
function Warn([string]$s) { Write-Host "[WARN] $s" -ForegroundColor Yellow }


# Run a native command that is being used as a status/query operation.
# Windows PowerShell 5.1 can promote native STDERR into a terminating
# NativeCommandError when $ErrorActionPreference='Stop'.  Query commands such
# as "git remote get-url", "git rev-parse", "git diff --quiet" and
# "gh release view" legitimately use non-zero exit codes to mean "not found"
# or "different".  Capture those exit codes explicitly instead of letting
# PowerShell abort the publisher.
function Invoke-NativeQuery {
    param(
        [Parameter(Mandatory=$true)][string]$FilePath,
        [Parameter(Mandatory=$true)][string[]]$Arguments
    )
    $oldPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'SilentlyContinue'
        $output = @(& $FilePath @Arguments 2>$null)
        $code = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $oldPreference
    }
    [pscustomobject]@{
        ExitCode = [int]$code
        Output   = @($output)
    }
}

Section '1. Safety check'
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Preflight failed. Nothing was uploaded.' }
if (-not (Test-Path $setup)) { throw "Setup.exe for version $version was not found. Run 01_BUILD_SETUP_ONECLICK.bat first." }
Pass 'Preflight and release asset check passed.'

Section '2. Git source sync (no force push)'
$git = Get-Command git.exe -ErrorAction SilentlyContinue
if (-not $git) { throw 'Git for Windows is not installed or not in PATH.' }
Set-Location $repo
if (-not (Test-Path (Join-Path $repo '.git'))) {
    & git init
    if ($LASTEXITCODE -ne 0) { throw 'git init failed.' }
}
& git branch -M main

$remoteListQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('remote')
if ($remoteListQuery.ExitCode -ne 0) { throw 'Could not inspect local Git remotes.' }
$remoteNames = @($remoteListQuery.Output | ForEach-Object { ([string]$_).Trim() } | Where-Object { $_ })
$origin = $null
if ($remoteNames -contains 'origin') {
    $originQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('remote','get-url','origin')
    if ($originQuery.ExitCode -eq 0 -and $originQuery.Output.Count -gt 0) {
        $origin = ([string]$originQuery.Output[0]).Trim()
    }
}
if (-not $origin) {
    Write-Host 'Create an EMPTY GitHub repository first (no GitHub-generated README/license/.gitignore).'
    $origin = (Read-Host 'Paste GitHub repository HTTPS URL').Trim()
    if (-not $origin) { throw 'Repository URL is required.' }
    & $git.Source remote add origin $origin
    if ($LASTEXITCODE -ne 0) { throw 'Could not add origin.' }
}
Write-Host "origin: $origin"

& git add -A
if ($LASTEXITCODE -ne 0) { throw 'git add failed.' }

# Verify the exact staged/upload scope after git add. Ignored local build products are
# intentionally absent; accidentally tracked build products are blocked here.
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Post-stage preflight failed. Nothing was uploaded.' }

$headQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('rev-parse','--verify','HEAD')
$headExists = ($headQuery.ExitCode -eq 0)
if (-not $headExists) {
    & $git.Source commit -m 'Initial public no-stitch product release'
    if ($LASTEXITCODE -ne 0) { throw 'Initial commit failed. Check git user.name/user.email configuration.' }
} else {
    $diffQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('diff','--cached','--quiet')
    if ($diffQuery.ExitCode -eq 1) {
        & $git.Source commit -m "Prepare Robust Hole Metrology $version"
        if ($LASTEXITCODE -ne 0) { throw 'Commit failed.' }
    } elseif ($diffQuery.ExitCode -eq 0) {
        Write-Host 'No source changes to commit.' -ForegroundColor Gray
    } else {
        throw "git diff --cached --quiet failed with exit code $($diffQuery.ExitCode)."
    }
}

$remoteMainQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('ls-remote','--heads','origin','refs/heads/main')
if ($remoteMainQuery.ExitCode -ne 0) {
    throw 'Could not query the GitHub origin. Check the repository URL, network access, and GitHub authentication.'
}
$remoteMain = @($remoteMainQuery.Output | Where-Object { ([string]$_).Trim() })
if ($remoteMain.Count -eq 0) {
    & $git.Source push -u origin main
    if ($LASTEXITCODE -ne 0) { throw 'Initial push failed.' }
} else {
    & $git.Source fetch origin main
    if ($LASTEXITCODE -ne 0) { throw 'git fetch failed.' }
    $localAheadQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('merge-base','--is-ancestor','origin/main','HEAD')
    if ($localAheadQuery.ExitCode -eq 0) {
        & $git.Source push -u origin main
        if ($LASTEXITCODE -ne 0) { throw 'Push failed.' }
    } else {
        $remoteAheadQuery = Invoke-NativeQuery -FilePath $git.Source -Arguments @('merge-base','--is-ancestor','HEAD','origin/main')
        if ($remoteAheadQuery.ExitCode -eq 0) {
            & $git.Source merge --ff-only origin/main
            if ($LASTEXITCODE -ne 0) { throw 'Fast-forward from origin/main failed.' }
            & $git.Source push -u origin main
            if ($LASTEXITCODE -ne 0) { throw 'Push failed.' }
        } else {
            throw 'Local main and origin/main have diverged or are unrelated. Stopped safely; no force push was attempted.'
        }
    }
}
Pass 'Source main branch synchronized safely.'

Section '3. GitHub CLI authentication'
function Find-Gh {
    $c = Get-Command gh.exe -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    $known = @(
        (Join-Path $env:ProgramFiles 'GitHub CLI\gh.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'GitHub CLI\gh.exe')
    ) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
    if ($known) { return [string]$known }
    return $null
}
$gh = Find-Gh
if (-not $gh) {
    $winget = Get-Command winget.exe -ErrorAction SilentlyContinue
    if ($winget) {
        Warn 'GitHub CLI not found. Attempting installation through winget...'
        & $winget.Source install --id GitHub.cli -e --silent --accept-source-agreements --accept-package-agreements
        $gh = Find-Gh
    }
}
if (-not $gh) { throw 'GitHub CLI (gh) is required to upload the Release asset.' }
$authQuery = Invoke-NativeQuery -FilePath $gh -Arguments @('auth','status')
if ($authQuery.ExitCode -ne 0) {
    Write-Host 'GitHub sign-in is required. Your browser may open.' -ForegroundColor Yellow
    & $gh auth login --web --git-protocol https
    if ($LASTEXITCODE -ne 0) { throw 'GitHub CLI authentication failed.' }
}
Pass 'GitHub CLI authenticated.'

# Convert origin URL to OWNER/REPO.
$repoSlug = $origin.Trim()
$repoSlug = $repoSlug -replace '^https://github\.com/',''
$repoSlug = $repoSlug -replace '^git@github\.com:',''
$repoSlug = $repoSlug -replace '\.git$',''
if ($repoSlug -notmatch '^[^/]+/[^/]+$') { throw "Could not derive OWNER/REPO from origin: $origin" }

Section "4. Create GitHub Release $tag"
$releaseQuery = Invoke-NativeQuery -FilePath $gh -Arguments @('release','view',$tag,'-R',$repoSlug)
if ($releaseQuery.ExitCode -eq 0) {
    throw "GitHub Release $tag already exists. To avoid replacing a reviewed binary, bump product\VERSION.txt and rebuild."
}
$notes = 'Windows x64 standalone installer for the no-stitch Robust Hole Metrology production GUI. Download the single Setup.exe asset and install; no separate Qt/PCL/VTK development environment is required.'
& $gh release create $tag $setup -R $repoSlug --target main --title "Robust Hole Metrology Development Preview $version" --notes $notes --latest
if ($LASTEXITCODE -ne 0) { throw 'GitHub Release creation/upload failed.' }
Pass 'Release created. Only the standalone Setup.exe was uploaded as the release asset.'

Write-Host "`nSUCCESS" -ForegroundColor Green
Write-Host "Source: $origin"
Write-Host "Release tag: $tag"
Write-Host "Asset: $([IO.Path]::GetFileName($setup))"
