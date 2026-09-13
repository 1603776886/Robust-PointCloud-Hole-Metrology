$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$projectDir = Join-Path $repo 'apps\no_stitch'
$solution = Join-Path $projectDir 'RobustHoleMetrology_NoStitch.sln'
$version = (Get-Content -LiteralPath (Join-Path $repo 'product\VERSION.txt') -Raw).Trim()
$outRoot = Join-Path $repo 'out'
$appDir = Join-Path $outRoot 'package\app'
$redistDir = Join-Path $outRoot 'package\redist'
$releaseDir = Join-Path $outRoot 'release'
$iss = Join-Path $repo 'product\installer\RobustHoleMetrology.iss'
$qtInstallName = '6.5.3_msvc2019_64'

function Section([string]$s) { Write-Host "`n=== $s ===" -ForegroundColor Cyan }
function Pass([string]$s) { Write-Host "[PASS] $s" -ForegroundColor Green }
function Info([string]$s) { Write-Host "[INFO] $s" -ForegroundColor Gray }
function Warn([string]$s) { Write-Host "[WARN] $s" -ForegroundColor Yellow }
function Fail([string]$s) { Write-Host "[FAIL] $s" -ForegroundColor Red }

function Find-MSBuild {
    $vswhere = @(
        (Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'),
        (Join-Path $env:ProgramFiles 'Microsoft Visual Studio\Installer\vswhere.exe')
    ) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
    if ($vswhere) {
        $m = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
        if ($m -and (Test-Path $m)) { return $m }
    }
    $cmd = Get-Command msbuild.exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

function Find-Dumpbin {
    $cmd = Get-Command dumpbin.exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $vswhere = @(
        (Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'),
        (Join-Path $env:ProgramFiles 'Microsoft Visual Studio\Installer\vswhere.exe')
    ) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
    if ($vswhere) {
        $install = & $vswhere -latest -products * -property installationPath | Select-Object -First 1
        if ($install) {
            $base = Join-Path $install 'VC\Tools\MSVC'
            if (Test-Path $base) {
                $c = Get-ChildItem -Path $base -Recurse -Filter dumpbin.exe -File -ErrorAction SilentlyContinue |
                    Where-Object { $_.FullName -match 'Hostx64\\x64\\dumpbin\.exe$' } |
                    Sort-Object FullName -Descending | Select-Object -First 1
                if ($c) { return $c.FullName }
            }
        }
    }
    return $null
}

function Test-PclRoot([string]$p) {
    return [bool]($p -and (Test-Path (Join-Path $p 'include\pcl-1.15')) -and (Test-Path (Join-Path $p 'lib')))
}

function Find-PclRoot {
    $candidates = New-Object System.Collections.Generic.List[string]
    if ($env:PCL_ROOT) { $candidates.Add($env:PCL_ROOT) }
    $localProps = Join-Path $projectDir 'Local.Build.props'
    if (Test-Path $localProps) {
        $raw = Get-Content -LiteralPath $localProps -Raw
        $m = [regex]::Match($raw, '<PCL_ROOT>(.*?)</PCL_ROOT>', [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
        if ($m.Success) { $candidates.Add($m.Groups[1].Value.Trim()) }
    }
    @('C:\Program Files\PCL 1.15.1','C:\Program Files\PCL 1.15.0','C:\PCL 1.15.1') | ForEach-Object { $candidates.Add($_) }
    foreach ($p in $candidates) { if (Test-PclRoot $p) { return (Resolve-Path $p).Path } }
    $entered = Read-Host 'PCL 1.15.x was not auto-detected. Enter PCL root'
    if (Test-PclRoot $entered) { return (Resolve-Path $entered).Path }
    return $null
}

function Find-QtRoot {
    $roots = New-Object System.Collections.Generic.List[string]
    if ($env:QT_ROOT) { $roots.Add($env:QT_ROOT) }
    if ($env:QTDIR) { $roots.Add($env:QTDIR) }
    $cmd = Get-Command windeployqt.exe -ErrorAction SilentlyContinue
    if ($cmd) { $roots.Add((Split-Path -Parent (Split-Path -Parent $cmd.Source))) }
    @('C:\Qt\6.5.3\msvc2019_64','C:\Qt\6.5.3\msvc2022_64') | ForEach-Object { $roots.Add($_) }
    foreach ($p in $roots) {
        if ($p -and (Test-Path (Join-Path $p 'bin\windeployqt.exe'))) { return (Resolve-Path $p).Path }
    }
    $entered = Read-Host 'Qt was not auto-detected. Enter Qt kit root (folder containing bin\windeployqt.exe)'
    if ($entered -and (Test-Path (Join-Path $entered 'bin\windeployqt.exe'))) { return (Resolve-Path $entered).Path }
    return $null
}

function Find-InnoSetup {
    # IMPORTANT: this function must emit only the final ISCC.exe path (or $null).
    # Do not let installer/search command output leak into the success output stream,
    # because callers assign the function result directly to $iscc.
    $fixedCandidates = @(
        (Join-Path ${env:ProgramFiles(x86)} 'Inno Setup 6\ISCC.exe'),
        (Join-Path $env:ProgramFiles 'Inno Setup 6\ISCC.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'Inno Setup 7\ISCC.exe'),
        (Join-Path $env:ProgramFiles 'Inno Setup 7\ISCC.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 7\ISCC.exe')
    )
    foreach ($candidate in $fixedCandidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return [string](Resolve-Path -LiteralPath $candidate).Path
        }
    }

    # Inno Setup writes InstallLocation to the normal Windows uninstall registry keys.
    # Check both machine/user and 32/64-bit views so winget installs are discovered
    # regardless of scope.
    $registryKeys = @(
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1',
        'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1',
        'HKCU:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 6_is1',
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 7_is1',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 7_is1',
        'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 7_is1',
        'HKCU:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup 7_is1'
    )
    foreach ($key in $registryKeys) {
        $item = Get-ItemProperty -LiteralPath $key -ErrorAction SilentlyContinue
        if ($item -and $item.InstallLocation) {
            $candidate = Join-Path ([string]$item.InstallLocation) 'ISCC.exe'
            if (Test-Path -LiteralPath $candidate) {
                return [string](Resolve-Path -LiteralPath $candidate).Path
            }
        }
    }

    $cmd = Get-Command ISCC.exe -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source -and (Test-Path -LiteralPath $cmd.Source)) {
        return [string](Resolve-Path -LiteralPath $cmd.Source).Path
    }
    return $null
}

function Ensure-InnoSetup {
    $iscc = Find-InnoSetup
    if ($iscc) { return [string]$iscc }

    $winget = Get-Command winget.exe -ErrorAction SilentlyContinue
    if (-not $winget) { return $null }

    Warn 'Inno Setup was not found. Attempting installation through winget...'

    # Capture winget's text so it cannot become this function's return value.
    # This fixes the case where "Found Inno Setup... Successfully installed"
    # was mistakenly assigned to $iscc and then executed as a command.
    $wingetOutput = @(& $winget.Source install --id JRSoftware.InnoSetup -e --silent --accept-source-agreements --accept-package-agreements 2>&1)
    $wingetExit = $LASTEXITCODE
    foreach ($line in $wingetOutput) { Write-Host ([string]$line) }

    if ($wingetExit -ne 0) {
        Warn "winget returned exit code $wingetExit while installing Inno Setup. Re-checking in case it is already installed."
    }

    Start-Sleep -Seconds 2
    $iscc = Find-InnoSetup
    if ($iscc) { return [string]$iscc }

    # Last-resort shallow search of standard roots. Keep command output captured so
    # only an actual ISCC.exe full path can escape from this function.
    $searchRoots = @($env:ProgramFiles, ${env:ProgramFiles(x86)}, $env:LOCALAPPDATA) |
        Where-Object { $_ -and (Test-Path -LiteralPath $_) }
    foreach ($root in $searchRoots) {
        $found = @(Get-ChildItem -LiteralPath $root -Filter ISCC.exe -File -Recurse -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match '(?i)Inno Setup' } |
            Select-Object -First 1)
        if ($found.Length -gt 0) {
            return [string]$found[0].FullName
        }
    }

    return $null
}

function Get-Dependencies([string]$file, [string]$dumpbin) {
    $lines = & $dumpbin /DEPENDENTS $file 2>$null
    $result = New-Object System.Collections.Generic.List[string]
    foreach ($line in $lines) {
        $m = [regex]::Match($line, '^\s*([A-Za-z0-9_.+\-]+\.dll)\s*$')
        if ($m.Success) { $result.Add($m.Groups[1].Value) }
    }
    return $result | Select-Object -Unique
}

function Build-DllIndex([string]$pclRoot, [string]$qtRoot) {
    $index = @{}
    $roots = @($pclRoot, (Join-Path $qtRoot 'bin'))
    foreach ($r in $roots) {
        if (-not (Test-Path $r)) { continue }
        Get-ChildItem -Path $r -Recurse -File -Filter *.dll -ErrorAction SilentlyContinue | ForEach-Object {
            $key = $_.Name.ToLowerInvariant()
            if (-not $index.ContainsKey($key)) { $index[$key] = $_.FullName }
        }
    }
    return $index
}

function Copy-DependencyClosure([string]$entryExe, [string]$dest, [string]$dumpbin, [hashtable]$dllIndex) {
    $queue = New-Object System.Collections.Generic.Queue[string]
    $seen = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    $queue.Enqueue($entryExe)
    while ($queue.Count -gt 0) {
        $current = $queue.Dequeue()
        foreach ($dep in (Get-Dependencies $current $dumpbin)) {
            if (-not $seen.Add($dep)) { continue }
            $already = Join-Path $dest $dep
            if (Test-Path $already) { $queue.Enqueue($already); continue }
            $key = $dep.ToLowerInvariant()
            if ($dllIndex.ContainsKey($key)) {
                Copy-Item -LiteralPath $dllIndex[$key] -Destination $already -Force
                $queue.Enqueue($already)
                Info "runtime: $dep"
            }
        }
    }
}

function Copy-LicenseFiles([string]$sourceRoot, [string]$destRoot, [string]$label) {
    if (-not (Test-Path $sourceRoot)) { return }
    $files = Get-ChildItem -Path $sourceRoot -Recurse -File -ErrorAction SilentlyContinue | Where-Object {
        $_.Name -match '^(LICENSE|LICENCE|COPYING|NOTICE|COPYRIGHT)(\.|$)' -or $_.Name -match '(?i)license.*\.(txt|md)$'
    }
    foreach ($f in $files) {
        $relative = $f.FullName.Substring($sourceRoot.Length).TrimStart('\')
        $target = Join-Path (Join-Path $destRoot $label) $relative
        New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force | Out-Null
        Copy-Item -LiteralPath $f.FullName -Destination $target -Force
    }
}

Section '1. Repository safety check'
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Repository preflight failed.' }
Pass 'Preflight passed.'

Section '2. Locate publisher toolchain'
$msbuild = Find-MSBuild
if (-not $msbuild) { throw 'MSBuild not found. Install Visual Studio 2022 Desktop development with C++.' }
$pclRoot = Find-PclRoot
if (-not $pclRoot) { throw 'Compatible PCL 1.15.x installation not found.' }
$qtRoot = Find-QtRoot
if (-not $qtRoot) { throw 'Qt kit root not found.' }
$windeployqt = Join-Path $qtRoot 'bin\windeployqt.exe'
$dumpbin = Find-Dumpbin
if (-not $dumpbin) { throw 'dumpbin.exe not found in the Visual Studio toolchain.' }
Pass "MSBuild: $msbuild"
Pass "PCL: $pclRoot"
Pass "Qt: $qtRoot"
Pass "dumpbin: $dumpbin"

Section '3. Rebuild validated no-stitch GUI (Release x64)'
& $msbuild $solution /m /t:Rebuild /p:Configuration=Release /p:Platform=x64 "/p:PCL_ROOT=$pclRoot" "/p:QtInstallName=$qtInstallName" /nologo /verbosity:minimal
if ($LASTEXITCODE -ne 0) { throw 'Release x64 build failed.' }
$builtExe = Get-ChildItem -Path $projectDir -Recurse -File -Filter 'RobustHoleMetrology_NoStitch.exe' -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $builtExe) { throw 'Build reported success but RobustHoleMetrology_NoStitch.exe was not found.' }
Pass "Built: $($builtExe.FullName)"

Section '4. Assemble standalone application directory'
if (Test-Path (Join-Path $outRoot 'package')) { Remove-Item -Recurse -Force (Join-Path $outRoot 'package') }
New-Item -ItemType Directory -Path $appDir -Force | Out-Null
New-Item -ItemType Directory -Path $redistDir -Force | Out-Null
New-Item -ItemType Directory -Path $releaseDir -Force | Out-Null
$appExe = Join-Path $appDir 'RobustHoleMetrology.exe'
Copy-Item -LiteralPath $builtExe.FullName -Destination $appExe -Force
Copy-Item -LiteralPath (Join-Path $repo 'LICENSE') -Destination (Join-Path $appDir 'LICENSE.txt') -Force
Copy-Item -LiteralPath (Join-Path $repo 'THIRD_PARTY_NOTICES.md') -Destination (Join-Path $appDir 'THIRD_PARTY_NOTICES.txt') -Force

& $windeployqt --release --no-translations --no-compiler-runtime --dir $appDir $appExe
if ($LASTEXITCODE -ne 0) { throw 'windeployqt failed.' }
if (-not (Test-Path (Join-Path $appDir 'platforms\qwindows.dll'))) { throw 'Qt platform plugin qwindows.dll was not deployed.' }
Pass 'Qt runtime/plugins deployed.'

$dllIndex = Build-DllIndex $pclRoot $qtRoot
Copy-DependencyClosure $appExe $appDir $dumpbin $dllIndex

# Also resolve dependencies of every deployed Qt plugin/DLL after the first pass.
Get-ChildItem -Path $appDir -Recurse -File -Include *.dll,*.exe | ForEach-Object {
    Copy-DependencyClosure $_.FullName $appDir $dumpbin $dllIndex
}

# Preserve OpenNI2 driver tree if available. This is harmless for file-only workflows and prevents runtime driver lookup failures.
$openniTree = Join-Path $pclRoot '3rdParty\OpenNI2\Redist\OpenNI2'
if (Test-Path $openniTree) {
    Copy-Item -Path $openniTree -Destination $appDir -Recurse -Force
    Pass 'OpenNI2 driver runtime tree included.'
}

# Copy license/notice material found in exact local runtime installations.
$licenseDir = Join-Path $appDir 'licenses'
New-Item -ItemType Directory -Path $licenseDir -Force | Out-Null
Copy-LicenseFiles $qtRoot $licenseDir 'Qt'
Copy-LicenseFiles $pclRoot $licenseDir 'PCL_bundle'

# Basic package sanity checks.
$expected = @('RobustHoleMetrology.exe','Qt6Core.dll','Qt6Gui.dll','Qt6Widgets.dll')
foreach ($e in $expected) { if (-not (Test-Path (Join-Path $appDir $e))) { throw "Missing packaged runtime: $e" } }
$pclDll = Get-ChildItem -Path $appDir -File -Filter 'pcl_common*.dll' | Select-Object -First 1
$vtkDll = Get-ChildItem -Path $appDir -File -Filter 'vtkCommonCore-*.dll' | Select-Object -First 1
if (-not $pclDll) { throw 'PCL runtime DLL closure was not resolved (pcl_common*.dll missing).' }
if (-not $vtkDll) { throw 'VTK runtime DLL closure was not resolved (vtkCommonCore-*.dll missing).' }
Pass 'PCL/VTK runtime dependency closure assembled.'

Section '5. Bundle official Microsoft Visual C++ Runtime'
$vcRedist = Join-Path $redistDir 'vc_redist.x64.exe'
Invoke-WebRequest -Uri 'https://aka.ms/vc14/vc_redist.x64.exe' -OutFile $vcRedist -UseBasicParsing
if (-not (Test-Path $vcRedist) -or (Get-Item $vcRedist).Length -lt 1MB) { throw 'VC++ Redistributable download failed or is unexpectedly small.' }
Pass 'Official vc_redist.x64.exe downloaded.'

Section '6. Build single Setup.exe'
$isccResults = @(Ensure-InnoSetup)
$iscc = $isccResults | Where-Object {
    $_ -is [string] -and $_ -match '(?i)ISCC\.exe$' -and (Test-Path -LiteralPath $_)
} | Select-Object -First 1
if (-not $iscc) {
    throw 'Inno Setup compiler not found and could not be installed automatically. Install Inno Setup, then rerun this script.'
}
$iscc = [string](Resolve-Path -LiteralPath $iscc).Path
Pass "Inno Setup: $iscc"

# Remove an old installer with the same version before compiling.
$setupName = "RobustHoleMetrology-Dev-$version-Setup-x64.exe"
$setupPath = Join-Path $releaseDir $setupName
if (Test-Path $setupPath) { Remove-Item -Force $setupPath }

& $iscc "/DMyAppVersion=$version" "/DSourceDir=$appDir" "/DOutputDir=$releaseDir" "/DVCRedist=$vcRedist" "/DRepoRoot=$repo" $iss
if ($LASTEXITCODE -ne 0) { throw 'Inno Setup compilation failed.' }
if (-not (Test-Path $setupPath)) {
    $setupFound = Get-ChildItem -Path $releaseDir -File -Filter "RobustHoleMetrology-Dev-$version-Setup-x64.exe" | Select-Object -First 1
    if ($setupFound) { $setupPath = $setupFound.FullName } else { throw 'Installer compiler finished but Setup.exe was not found.' }
}

$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $setupPath).Hash
Set-Content -LiteralPath ($setupPath + '.sha256.txt') -Value "$hash  $([IO.Path]::GetFileName($setupPath))" -Encoding ascii

Section 'DONE'
Pass 'Standalone Windows installer created successfully.'
Write-Host "`nSETUP EXE:" -ForegroundColor Green
Write-Host $setupPath -ForegroundColor White
Write-Host "`nThis is the file to test on a clean Windows machine and then upload to GitHub Releases." -ForegroundColor Yellow
Write-Host 'End users do not need Visual Studio, Qt, PCL or VTK installed separately.' -ForegroundColor Yellow
