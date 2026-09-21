$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Write-Host "Research Demo updater: $repo" -ForegroundColor Cyan

function Write-Utf8NoBom([string]$Path, [string]$Text) {
    $enc = New-Object System.Text.UTF8Encoding($false)
    [IO.File]::WriteAllText($Path, $Text.Replace("`r`n","`n").Replace("`r","`n"), $enc)
}

$readme = @'
# Robust Point-Cloud Hole Metrology

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

This repository is deliberately scoped as a **research demonstration**. It does **not** represent the complete engineering system, complete project deliverables, complete experimental datasets, industrial deployment package, or an official software release of any funding programme or sponsoring organization.

The public tree contains only the selected **single-cloud / no-stitch** GUI implementation. The multi-cloud stitching GUI, CLI/batch validation harness, project-specific datasets, internal interfaces, and other non-public engineering components are not included.

## For users: download and run

Normal users do **not** need Visual Studio, Qt, PCL, VTK, Boost, Eigen, OpenNI2, or any other development dependency.

Open the repository's **Releases** page and download the Windows installer:

```text
RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

Run the installer and launch **Robust Hole Metrology - Research Demo** from the Start menu or optional desktop shortcut. The installer contains the application runtime dependencies and installs the official Microsoft Visual C++ runtime when required.

## Public demo scope

```text
apps/no_stitch/                 selected no-stitch research-demo GUI source
examples/                       selected reproducible examples that are safe to publish
product/installer/              Windows research-demo installer definition
scripts/build_setup.ps1         build + self-contained deployment + Setup.exe
scripts/publish_release.ps1     safe Git push + GitHub Release upload
01_BUILD_SETUP_ONECLICK.bat     publisher one-click build/package
02_UPLOAD_GITHUB_ONECLICK.bat   publisher one-click source/release upload
03_APPLY_RESEARCH_DEMO_AND_SYNC.bat
                                apply this public demo positioning + safe Git sync
```

The application source under `apps/no_stitch/` remains separate from packaging and publication scripts. Public-repository packaging does not alter the recognition/metrology implementation.

For the exact public/non-public boundary, see [`docs/RESEARCH_DEMO_SCOPE.md`](docs/RESEARCH_DEMO_SCOPE.md).

## Selected reproducible examples

If redistributable sample point clouds are provided under `examples/`, they are intended only to demonstrate the public workflow. They are not intended to represent the complete experimental dataset used in associated research.

Only data that you have the right to publish should be placed in this repository.

## Publisher workflow

The publisher/developer machine uses the reference toolchain:

- Windows 10/11 x64
- Visual Studio 2022 / MSVC v143
- Qt 6.5.3 MSVC 64-bit
- PCL 1.15.1 Windows bundle (including the compatible VTK/Boost/Eigen/FLANN/OpenNI2/Qhull/lz4 runtime set)

Then:

1. Double-click `01_BUILD_SETUP_ONECLICK.bat` to rebuild Release x64, collect runtime DLLs, deploy Qt plugins, bundle the official VC++ Redistributable, and create one installer EXE.
2. Test the generated installer on a clean Windows machine/VM.
3. Double-click `02_UPLOAD_GITHUB_ONECLICK.bat` to safely commit/push the source and create the GitHub Release containing the **single Setup.exe asset**.

See [`docs/PRODUCT_RELEASE.md`](docs/PRODUCT_RELEASE.md) for details.

## Build source manually

For source reviewers/developers, copy `apps/no_stitch/Local.Build.props.example` to `Local.Build.props`, set the local PCL root if needed, and build `Release | x64` in Visual Studio. Machine-local `Local.Build.props` is ignored by Git.

## Third-party software

This project dynamically uses Qt/PCL/VTK and related libraries. Their binaries and license notices in a generated installer remain under their respective upstream licenses. The repository's own `LICENSE` covers only project-authored material. See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Academic / project context

This public repository is a demonstration-oriented subset for academic communication, review, portfolio presentation, and non-commercial technical evaluation. It should not be interpreted as the complete deliverable of a research programme or as an official software release by a funding agency, university, laboratory, industrial partner, or other organization.

Where an associated paper or research output requires funding acknowledgement, use the acknowledgement wording and project number required by the corresponding project documentation and publication venue.

## License

The project-authored source is distributed under the repository's source-available evaluation license for academic review, recruitment/portfolio evaluation, and personal non-commercial technical evaluation. See [`LICENSE`](LICENSE).

'@
Write-Utf8NoBom (Join-Path $repo 'README.md') $readme

$scope = @'
# Research Demo / Development Preview Scope

**Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.**

**The public repository contains only the demonstration-oriented software components and selected reproducible examples.**

## What this repository is

This repository is a public **Research Demo / Development Preview** intended for:

- academic communication and peer review;
- recruitment, interview, and portfolio evaluation;
- non-commercial technical evaluation;
- reproducible demonstration with selected publishable examples.

## What this repository is not

The public repository should **not** be interpreted as:

- the complete engineering system;
- the complete deliverable of an associated research project;
- the complete source tree or complete algorithm-development history;
- the complete experimental or industrial dataset;
- a production-certified industrial deployment;
- an official software release by any funding programme, university, laboratory, company, or project consortium.

## Public technical boundary

The public repository intentionally contains the selected **single-cloud / no-stitch** GUI implementation and publication/deployment tooling. Multi-cloud stitching, CLI/batch validation infrastructure, project-specific datasets, internal interfaces, and other non-public components are outside this public demo scope.

Selected example point clouds may be added only when redistribution is permitted. Such examples demonstrate use of the public software and do not constitute the complete dataset behind associated research.

## Funding acknowledgement

Funding acknowledgements belong to the associated paper/research output and should follow the exact wording and project number required by the relevant project documentation and publication venue. This public demo does not claim to be an official release on behalf of a funding body or project consortium.

'@
Write-Utf8NoBom (Join-Path $repo 'docs\RESEARCH_DEMO_SCOPE.md') $scope

$review = @'
# Reviewer guide

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

This public tree intentionally publishes only the selected single-cloud/no-stitch GUI implementation. There is no stitching module and no CLI/batch harness in the public demo.

Fast source-review entry points:

- `apps/no_stitch/main.cpp`
- `apps/no_stitch/ZhuChuangKou_Window.*`
- `apps/no_stitch/HoleShibie_Recognition.*`
- `apps/no_stitch/HoleFaXian_Normal.h`
- `apps/no_stitch/HoleJihe_Geometry.*`
- `apps/no_stitch/HoleFenxi_Analysis.*`
- `apps/no_stitch/HoleWeizi_Pose.h`

For execution without a development environment, use the Setup.exe attached to the GitHub Release rather than rebuilding the source.

See `RESEARCH_DEMO_SCOPE.md` for the public/non-public boundary.

'@
Write-Utf8NoBom (Join-Path $repo 'docs\REVIEWER_GUIDE.md') $review

$build = @'
# Windows source build

This repository is a **Research Demo / Development Preview** and contains only the selected no-stitch GUI implementation.

Reference environment: Windows x64, Visual Studio 2022/MSVC v143, Qt 6.5.3 MSVC x64, PCL 1.15.1 Windows bundle.

1. Copy `apps/no_stitch/Local.Build.props.example` to `apps/no_stitch/Local.Build.props`.
2. Edit `PCL_ROOT` if your local PCL path differs.
3. Open `apps/no_stitch/RobustHoleMetrology_NoStitch.sln`.
4. Select `Release | x64` and rebuild.

`Local.Build.props` is intentionally ignored by Git.

For the scope of the public demonstration, see `RESEARCH_DEMO_SCOPE.md`.

'@
Write-Utf8NoBom (Join-Path $repo 'docs\BUILD_WINDOWS.md') $build

$productRelease = @'
# Research Demo release workflow (no-stitch only)

## Goal

The public release is a **Research Demo / Development Preview** containing only the selected **no-stitch** GUI implementation. End users receive one Windows x64 installer EXE and do not install the development toolchain.

The release does not represent the complete engineering system, complete project deliverables, complete experimental datasets, or an official software release of any funding programme or sponsoring organization.

## One-click build

Double-click:

```text
01_BUILD_SETUP_ONECLICK.bat
```

The script:

1. runs repository safety checks;
2. locates Visual Studio/MSBuild, Qt 6.5.3 and PCL 1.15.x;
3. rebuilds `apps/no_stitch/RobustHoleMetrology_NoStitch.sln` as `Release | x64`;
4. copies the executable to a clean deployment directory as `RobustHoleMetrology.exe`;
5. runs Qt's `windeployqt` to collect the Qt DLLs and plugins;
6. resolves and copies the transitive PCL/VTK/Boost/FLANN/OpenNI2/Qhull/lz4 DLL dependencies needed by the program;
7. preserves the OpenNI2 driver runtime tree when present;
8. downloads Microsoft's official x64 Visual C++ Redistributable installer for inclusion in Setup;
9. copies available third-party license/notice files into `licenses/`;
10. compiles a single Windows installer with Inno Setup.

Output:

```text
out/release/RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

If Inno Setup is not installed, the script attempts to install it through Windows Package Manager (`winget`).

## Required publisher environment

Only the publisher/build machine needs the development environment. The final user's machine does not.

The script can auto-detect common installs. If detection fails, it prompts for the PCL root and Qt root. Recommended reference values are PCL 1.15.1 and Qt 6.5.3 MSVC x64.

## Clean-machine test — required before public release

Before uploading a release, test the generated Setup.exe on a clean Windows 10/11 x64 VM or PC that does not have Visual Studio, Qt or PCL installed. Confirm:

- installation completes;
- the application starts from the Start menu;
- the VTK 3D view appears normally;
- PCD/PLY files load;
- the normal point-cloud workflow functions;
- hole detection/measurement runs;
- overlays and export work;
- uninstall completes.

A successful build proves packaging/build integrity only; it does not replace metrology validation or imply production certification.

## GitHub source + release upload

After the clean-machine test, double-click:

```text
02_UPLOAD_GITHUB_ONECLICK.bat
```

The script safely commits/pushes the repository without force-pushing and creates the GitHub Release for the version in `product/VERSION.txt`. Only the single Setup.exe is uploaded as the release asset.

The GitHub Release title and notes identify the binary as a **Research Demo / Development Preview**.

For the very first publish, create an **empty** GitHub repository yourself. Do not pre-create README, `.gitignore`, or a license on GitHub because they already exist locally.

## Releasing a new version

Change only:

```text
product/VERSION.txt
```

For example, change `0.1.0-dev` to `0.1.1-dev`, rebuild, test, and upload. The release uploader refuses to overwrite an existing version tag/release; this prevents accidental replacement of a previously reviewed binary.

'@
Write-Utf8NoBom (Join-Path $repo 'docs\PRODUCT_RELEASE.md') $productRelease

$licensePath = Join-Path $repo 'LICENSE'
$license = Get-Content -LiteralPath $licensePath -Raw
if ($license -notmatch 'Research Demo / Development Preview scope notice') {
    $marker = "Copyright (c) 2026. All rights reserved.`n"
    $notice = "`nResearch Demo / Development Preview scope notice (informational)`nThis public repository is a demonstration-oriented subset. It does not represent the complete engineering system, complete project deliverables, complete datasets, or an official software release of any funding programme or sponsoring organization.`n"
    $license = $license.Replace("`r`n","`n").Replace($marker, $marker + $notice)
    Write-Utf8NoBom $licensePath $license
}

$issPath = Join-Path $repo 'product\installer\RobustHoleMetrology.iss'
$iss = Get-Content -LiteralPath $issPath -Raw
$iss = $iss.Replace('#define MyAppName "Robust Hole Metrology"','#define MyAppName "Robust Hole Metrology - Research Demo"')
$iss = $iss.Replace('AppPublisher=Robust Hole Metrology Project','AppPublisher=Robust Hole Metrology Research Demo')
$iss = $iss.Replace('DefaultDirName={autopf}\Robust Hole Metrology','DefaultDirName={autopf}\Robust Hole Metrology Research Demo')
$iss = $iss.Replace('DefaultGroupName=Robust Hole Metrology','DefaultGroupName=Robust Hole Metrology Research Demo')
$iss = $iss.Replace('Name: "{autoprograms}\Robust Hole Metrology"','Name: "{autoprograms}\Robust Hole Metrology - Research Demo"')
$iss = $iss.Replace('Name: "{autodesktop}\Robust Hole Metrology"','Name: "{autodesktop}\Robust Hole Metrology - Research Demo"')
$iss = $iss.Replace('Description: "Launch Robust Hole Metrology"','Description: "Launch Robust Hole Metrology - Research Demo"')
Write-Utf8NoBom $issPath $iss

$publishPath = Join-Path $repo 'scripts\publish_release.ps1'
$publish = Get-Content -LiteralPath $publishPath -Raw
$publish = $publish.Replace("& `$git.Source commit -m 'Initial public no-stitch product release'", "& `$git.Source commit -m 'Initial public Research Demo / Development Preview'")
$publish = $publish.Replace('& $git.Source commit -m "Prepare Robust Hole Metrology $version"', '& $git.Source commit -m "Prepare Research Demo / Development Preview $version"')
$publish = $publish.Replace("`$notes = 'Windows x64 standalone installer for the no-stitch Robust Hole Metrology production GUI. Download the single Setup.exe asset and install; no separate Qt/PCL/VTK development environment is required.'", "`$notes = 'Research Demo / Development Preview. Demonstration implementation of robust geometric metrology for incomplete 3D point clouds. The public repository contains only the demonstration-oriented software components and selected reproducible examples. Windows x64 standalone installer for the selected no-stitch GUI. This is not the complete engineering system, complete project deliverable, complete dataset, or an official software release of any funding programme. Download the single Setup.exe asset and install; no separate Qt/PCL/VTK development environment is required.'")
$publish = $publish.Replace('--title "Robust Hole Metrology Development Preview $version"','--title "Robust Hole Metrology - Research Demo / Development Preview $version"')
Write-Utf8NoBom $publishPath $publish

$changelogPath = Join-Path $repo 'CHANGELOG.md'
$changelog = Get-Content -LiteralPath $changelogPath -Raw
if ($changelog -notmatch 'Research Demo public-positioning update') {
    $entry = @'

## Research Demo public-positioning update

- Public repository is explicitly labeled **Research Demo / Development Preview**.
- README, reviewer/build/release documentation, installer display name, and GitHub Release metadata now use the same demonstration-oriented positioning.
- Public scope explicitly states that the repository is not the complete engineering system, complete project deliverable, complete dataset, or an official software release of a funding programme.
- No `apps/no_stitch` algorithm/GUI production source was changed by this positioning update.
'@
    $changelog = $changelog.Replace('# Changelog', '# Changelog' + $entry)
    Write-Utf8NoBom $changelogPath $changelog
}

Write-Host "Running repository preflight..." -ForegroundColor Cyan
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repo 'scripts\preflight.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Preflight failed after Research Demo update.' }

Write-Host ''
Write-Host '[PASS] Research Demo / Development Preview positioning applied.' -ForegroundColor Green
Write-Host 'No apps/no_stitch algorithm source was modified.' -ForegroundColor Green
Write-Host 'Because the installer display name changed, run 01_BUILD_SETUP_ONECLICK.bat once before publishing a new Setup.exe.' -ForegroundColor Yellow
