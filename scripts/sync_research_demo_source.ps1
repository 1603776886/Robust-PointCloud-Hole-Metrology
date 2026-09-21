$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Set-Location $repo

function Pass([string]$s) { Write-Host "[PASS] $s" -ForegroundColor Green }
function Warn([string]$s) { Write-Host "[WARN] $s" -ForegroundColor Yellow }

$git = Get-Command git.exe -ErrorAction SilentlyContinue
if (-not $git) { throw 'Git for Windows is not installed or not in PATH.' }
if (-not (Test-Path (Join-Path $repo '.git'))) { throw 'This folder is not the existing local Git repository. Run this script inside the repository you already uploaded.' }

Write-Host "Repository: $repo" -ForegroundColor Cyan
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo 'scripts\preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Preflight failed. Nothing was pushed.' }

$remotes = @(& $git.Source remote 2>$null)
if ($LASTEXITCODE -ne 0) { throw 'Could not inspect Git remotes.' }
if ($remotes -notcontains 'origin') {
    $url = (Read-Host 'Paste the existing GitHub repository HTTPS URL').Trim()
    if (-not $url) { throw 'Repository URL is required.' }
    & $git.Source remote add origin $url
    if ($LASTEXITCODE -ne 0) { throw 'Could not add origin.' }
}

& $git.Source branch -M main
& $git.Source add -A
if ($LASTEXITCODE -ne 0) { throw 'git add failed.' }

& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo 'scripts\preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Post-stage preflight failed. Nothing was pushed.' }

& $git.Source diff --cached --quiet
$diffCode = $LASTEXITCODE
if ($diffCode -eq 1) {
    & $git.Source commit -m 'Mark public repository as Research Demo / Development Preview'
    if ($LASTEXITCODE -ne 0) { throw 'Commit failed. Check Git user.name/user.email.' }
} elseif ($diffCode -eq 0) {
    Write-Host 'No new local source changes to commit.' -ForegroundColor Gray
} else {
    throw "git diff failed with exit code $diffCode."
}

# Fetch first so web-uploaded example files are preserved. Rebase local commit on
# top of origin/main when needed; never force-push.
& $git.Source fetch origin main
if ($LASTEXITCODE -ne 0) { throw 'Could not fetch origin/main. Check network/GitHub authentication.' }

& $git.Source merge-base --is-ancestor origin/main HEAD 2>$null
$originIsAncestor = ($LASTEXITCODE -eq 0)
if (-not $originIsAncestor) {
    Warn 'origin/main contains changes not in the local branch (for example files uploaded in the browser). Rebasing safely...'
    & $git.Source rebase origin/main
    if ($LASTEXITCODE -ne 0) {
        throw 'Rebase stopped due to a conflict. No force push was attempted. Resolve the conflict or run "git rebase --abort".'
    }
}

& $git.Source push -u origin main
if ($LASTEXITCODE -ne 0) { throw 'Git push failed. No force push was attempted.' }
Pass 'Research Demo source/documentation synchronized to GitHub main.'
