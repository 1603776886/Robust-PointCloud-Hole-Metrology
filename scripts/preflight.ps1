$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$root = Split-Path -Parent $PSScriptRoot
$failures = New-Object System.Collections.Generic.List[string]
$gitCmd = Get-Command git.exe -ErrorAction SilentlyContinue
Write-Host "Repository preflight: $root"

function Get-RelativeRepoPath([string]$fullPath) {
    $rootWithSlash = $root.TrimEnd('\','/') + [IO.Path]::DirectorySeparatorChar
    if ($fullPath.StartsWith($rootWithSlash, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $fullPath.Substring($rootWithSlash.Length).Replace('\','/')
    }
    return $fullPath.Replace('\','/')
}

function Test-GeneratedBuildPath([string]$relativePath) {
    $p = $relativePath.Replace('\','/')
    if ($p -match '(^|/)(\.git|out|\.vs|x64|x86|Debug|Release|build|bin|obj)(/|$)') { return $true }
    if ($p -match '(^|/)RobustHo\.[^/]+(/|$)') { return $true }
    if ($p -match '(?i)\.(obj|iobj|ipdb|pch|pdb|ilk|idb|tlog|lastbuildstate|exp)$') { return $true }
    if ($p -match '(?i)(^|/)Local\.Build\.props$') { return $true }
    return $false
}

function Get-UploadCandidateFiles {
    # If this directory is already a Git repository, inspect exactly what Git can
    # upload: tracked/staged files plus untracked files not excluded by .gitignore.
    # This deliberately excludes local build outputs such as x64/Release, out/, PDB,
    # IOBJ and Qt/MSBuild temporary projects.
    if ((Test-Path (Join-Path $root '.git')) -and $gitCmd) {
        $paths = @(& $gitCmd.Source -C $root ls-files --cached --others --exclude-standard 2>$null)
        if ($LASTEXITCODE -eq 0) {
            $files = New-Object System.Collections.Generic.List[System.IO.FileInfo]
            foreach ($rel in $paths) {
                if ([string]::IsNullOrWhiteSpace([string]$rel)) { continue }
                $full = Join-Path $root ([string]$rel)
                if (Test-Path -LiteralPath $full -PathType Leaf) {
                    $files.Add((Get-Item -LiteralPath $full))
                }
            }
            return @($files)
        }
    }

    # Before git init, use a conservative filesystem fallback that excludes known
    # generated/build directories. This keeps BUILD_SETUP repeatable after a prior build.
    return @(Get-ChildItem -Path $root -Recurse -File -Force | Where-Object {
        $rel = Get-RelativeRepoPath $_.FullName
        -not (Test-GeneratedBuildPath $rel)
    })
}

$repoFiles = @(Get-UploadCandidateFiles)

# If Git already tracks/stages a generated build artifact, fail explicitly. This is
# different from a normal ignored local artifact: tracked artifacts WOULD be uploaded.
if ((Test-Path (Join-Path $root '.git')) -and $gitCmd) {
    $tracked = @(& $gitCmd.Source -C $root ls-files --cached 2>$null)
    if ($LASTEXITCODE -eq 0) {
        foreach ($rel in $tracked) {
            if (Test-GeneratedBuildPath ([string]$rel)) {
                $failures.Add("Generated/build artifact is tracked by Git and must not be uploaded: $rel")
            }
        }
    }
}

$repoFiles | Where-Object { $_.Name -match '\.vcxproj\.user$|\.suo$|\.user$' } |
    ForEach-Object { $failures.Add("User-specific IDE file: $($_.FullName)") }

$pathSensitiveExt = @('.cpp','.h','.vcxproj','.filters','.props','.sln','.ui','.qrc')
$repoFiles | Where-Object {
    -not (Test-GeneratedBuildPath (Get-RelativeRepoPath $_.FullName)) -and
    $pathSensitiveExt -contains $_.Extension -and
    $_.Name -ne 'Local.Build.props.example'
} | ForEach-Object {
    $content = Get-Content -LiteralPath $_.FullName -Raw -ErrorAction SilentlyContinue
    if ($null -ne $content -and $content -match '(?i)[A-Z]:\\') {
        $failures.Add("Machine-specific absolute path in application/build file: $($_.FullName)")
    }
}

$noStitchRoot = (Join-Path $root 'apps\no_stitch')
$repoFiles | Where-Object {
    $_.FullName.StartsWith($noStitchRoot, [System.StringComparison]::OrdinalIgnoreCase) -and
    ($_.Extension -eq '.cpp' -or $_.Extension -eq '.h') -and
    -not (Test-GeneratedBuildPath (Get-RelativeRepoPath $_.FullName))
} | ForEach-Object {
    $content = Get-Content -LiteralPath $_.FullName -Raw
    if ($content -match 'AutoTest_' -or $content -match '--batch' -or $content -match '--self-check') {
        $failures.Add("Validation-only token inside public GUI: $($_.FullName)")
    }
    if ($content -match 'DianYunPinJie_Stitch') {
        $failures.Add("Stitching code/token inside public no-stitch GUI: $($_.FullName)")
    }
}

# Public repository must not upload any stitching source. Check upload candidates,
# not arbitrary local ignored folders.
$repoFiles | ForEach-Object {
    $rel = Get-RelativeRepoPath $_.FullName
    if ($rel -match '(^|/)apps/with_stitch(/|$)' -or $_.Name -match '^DianYunPinJie_Stitch\.') {
        $failures.Add("Stitching file is in Git upload scope: $rel")
    }
}

$repoFiles | Where-Object {
    -not (Test-GeneratedBuildPath (Get-RelativeRepoPath $_.FullName)) -and $_.Length -gt 90MB
} | ForEach-Object { $failures.Add("Source repository file larger than 90 MB: $($_.FullName)") }

@('LICENSE','README.md','01_BUILD_SETUP_ONECLICK.bat','02_UPLOAD_GITHUB_ONECLICK.bat','BUILD_AND_UPLOAD_ONECLICK.bat','scripts\build_setup.ps1','scripts\publish_release.ps1','apps\no_stitch\RobustHoleMetrology_NoStitch.sln') | ForEach-Object {
    if (-not (Test-Path (Join-Path $root $_))) { $failures.Add("Missing required public/product file: $_") }
}

if ($failures.Count -gt 0) {
    Write-Host 'Preflight FAILED' -ForegroundColor Red
    $failures | Select-Object -Unique | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}

Write-Host "Upload-scope files checked: $($repoFiles.Count)" -ForegroundColor Gray
Write-Host 'Preflight PASSED' -ForegroundColor Green
exit 0
