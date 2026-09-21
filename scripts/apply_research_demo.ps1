$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Write-Host "Research Demo public-positioning updater: $repo" -ForegroundColor Cyan

function Write-Utf8NoBom([string]$Path, [string]$Text) {
    $enc = New-Object System.Text.UTF8Encoding($false)
    $normalized = $Text.Replace("`r`n","`n").Replace("`r","`n")
    [IO.File]::WriteAllText($Path, $normalized, $enc)
}
function Ensure-Directory([string]$Path) {
    if (-not (Test-Path $Path)) { New-Item -ItemType Directory -Force -Path $Path | Out-Null }
}

Ensure-Directory (Join-Path $repo 'docs')
Ensure-Directory (Join-Path $repo 'examples')

$readme = @'
# Robust Point-Cloud Hole Metrology

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

A Windows x64 research demo for robust hole-feature recognition, geometric reconstruction, and dimensional metrology from incomplete 3D point clouds.

This public repository is intentionally limited to a **demonstration-oriented subset**. It does **not** represent the complete engineering system, complete research-project deliverables, complete experimental datasets, industrial deployment package, or an official software release of any funding programme, institution, laboratory, company, or project consortium.

The public tree contains only the selected **single-cloud / no-stitch Research Demo GUI** together with the files needed to build, package, review, and demonstrate it. Multi-cloud stitching, CLI/batch validation infrastructure, project-specific datasets, internal interfaces, and other non-public research or engineering components are outside this public scope.

## What this Research Demo shows

The public demo includes the following workflow components:

- 3D point-cloud import and visualization;
- point-cloud preprocessing used by the public GUI workflow;
- automatic hole-feature recognition;
- geometric reconstruction for incomplete and severely partial hole boundaries;
- hole-axis estimation;
- upper/lower opening geometry measurement;
- hole-depth measurement;
- 3D visualization of measurement results; and
- measurement-result export.

This repository publishes only the **no-stitch Research Demo GUI**. It does not publish the multi-cloud stitching GUI, CLI/batch stability-validation tools, complete experimental datasets, or other non-public development components.

## For users: download and run

Normal users do **not** need Visual Studio, Qt, PCL, VTK, Boost, Eigen, OpenNI2, or any other development environment.

Open the repository's **Releases** page and download the Windows x64 installer:

```text
RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

Run the installer, then launch:

```text
Robust Hole Metrology - Research Demo
```

The installer contains the runtime dependencies required by the packaged application and installs the Microsoft Visual C++ runtime when required.

## Selected reproducible examples

If redistributable example point clouds are provided under `examples/`, they are included only to demonstrate the public Research Demo workflow.

Recommended public example layout:

```text
examples/
├─ normal/
│  └─ sample_normal.pcd
├─ partial/
│  └─ sample_partial.pcd
└─ severe/
   └─ sample_severe.pcd
```

Only data that you have the right to redistribute should be placed in this repository. Public examples do **not** represent the complete experimental dataset used in associated research or project work.

## Public repository scope

```text
apps/no_stitch/                 Research Demo GUI source
examples/                       selected public demonstration samples
product/installer/              Windows installer definition
scripts/build_setup.ps1         build + standalone deployment + Setup.exe
scripts/publish_release.ps1     source sync + GitHub Release helper
01_BUILD_SETUP_ONECLICK.bat     one-click Windows package build
02_UPLOAD_GITHUB_ONECLICK.bat   one-click GitHub publishing helper
03_APPLY_RESEARCH_DEMO_ONECLICK.bat
                                apply the public Research Demo positioning
04_SYNC_RESEARCH_DEMO_SOURCE.bat
                                safe source/docs Git sync only
05_APPLY_AND_SYNC_RESEARCH_DEMO_ONECLICK.bat
                                apply wording + sync to GitHub main
```

The application source under `apps/no_stitch/` is kept separate from packaging and publication tooling. Updating the public-repository positioning does not change the hole-recognition or geometric-metrology implementation.

For the exact public/non-public boundary, see [`docs/RESEARCH_DEMO_SCOPE.md`](docs/RESEARCH_DEMO_SCOPE.md).

## Build from source

Reference development environment:

- Windows 10/11 x64;
- Visual Studio 2022 / MSVC v143;
- Qt 6.5.3 MSVC 64-bit;
- PCL 1.15.1 Windows bundle; and
- the compatible VTK / Boost / Eigen / FLANN / OpenNI2 / Qhull / lz4 runtime set used by that environment.

For source review or local development:

1. Copy `apps/no_stitch/Local.Build.props.example` to `apps/no_stitch/Local.Build.props`.
2. Set the local PCL root if your installation differs from the example.
3. Open `apps/no_stitch/RobustHoleMetrology_NoStitch.sln`.
4. Build `Release | x64` in Visual Studio.

`Local.Build.props` is machine-local and is ignored by Git.

## Publisher workflow

For a standalone Windows installer:

1. Run `01_BUILD_SETUP_ONECLICK.bat`.
2. Test the generated installer on a clean Windows 10/11 x64 machine or VM.
3. Run `02_UPLOAD_GITHUB_ONECLICK.bat` to synchronize the public source and create/update the GitHub Release, or upload the generated Setup.exe manually through the GitHub Releases web page.

See [`docs/PRODUCT_RELEASE.md`](docs/PRODUCT_RELEASE.md) for packaging details.

## Third-party software

This project dynamically uses Qt, PCL, VTK, and related third-party libraries. Their binaries and notices remain governed by their respective upstream licenses. The repository's own `LICENSE` applies only to project-authored material.

See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Research / project boundary

This repository is provided for academic communication, peer review, technical demonstration, recruitment/portfolio evaluation, and non-commercial technical evaluation.

It should **not** be interpreted as:

- the complete deliverable of an associated research programme;
- the complete source tree or complete algorithm-development history;
- the complete experimental or industrial dataset;
- a production-certified industrial deployment; or
- an official software release by any funding agency, university, laboratory, company, project consortium, or collaborating organization.

Where an associated paper or research output requires a funding acknowledgement, use the exact acknowledgement wording and project number required by the corresponding project documentation and publication venue.

## License

Project-authored material is distributed under the repository's **Source-Available Evaluation License** for academic review, recruitment/portfolio evaluation, and personal non-commercial technical evaluation.

See [`LICENSE`](LICENSE).

---

### Research Scope Notice

**Research Demo / Development Preview**  
Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
The public repository contains only the demonstration-oriented software components and selected reproducible examples.
'@
Write-Utf8NoBom (Join-Path $repo 'README.md') $readme

$licenseText = @'
Source-Available Evaluation License
Version 1.0 — 2026

Copyright (c) 2026. All rights reserved.

RESEARCH DEMO / DEVELOPMENT PREVIEW — SCOPE NOTICE

This public repository is a demonstration-oriented subset intended for academic communication, peer review, recruitment/portfolio evaluation, and non-commercial technical evaluation.

It does not represent the complete engineering system, complete research-project deliverables, complete experimental datasets, an industrial deployment package, or an official software release of any funding programme, institution, laboratory, company, project consortium, or collaborating organization.

The public repository contains only the demonstration-oriented software components and selected reproducible examples that the repository publisher has chosen to make public.

1. Permitted use
Permission is granted, free of charge, to view, clone, compile, and run project-authored material in this repository solely for:
(a) non-commercial academic peer review;
(b) recruitment, interview, and portfolio evaluation; and
(c) personal, non-commercial technical evaluation.

2. Restrictions
Except as required by applicable law or expressly permitted in writing by the copyright holder, you may not:
(a) sell, sublicense, commercially exploit, or provide this software as a service;
(b) redistribute the project-authored source code or compiled binaries outside the ordinary GitHub functionality needed to access or fork the public repository;
(c) publish modified or derivative versions;
(d) remove copyright, attribution, or license notices; or
(e) use any patent, trademark, trade name, logo, or other branding associated with the project.

3. Third-party software
Third-party software, libraries, binaries, notices, and other materials included or referenced by this repository remain governed by their respective upstream licenses. This license does not relicense third-party material.

4. No patent license
No patent rights are granted under this license.

5. No warranty
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY ARISING FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR ITS USE.

6. Other permissions
For commercial use, redistribution, publication of modified versions, or any use beyond the permissions above, obtain separate written permission from the copyright holder.

This is a source-available evaluation license, not an OSI-approved open-source license.
'@
Write-Utf8NoBom (Join-Path $repo 'LICENSE') $licenseText

$scope = @'
# Research Demo / Development Preview Scope

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

## Purpose of this public repository

This repository is a deliberately limited public demonstration intended for:

- academic communication and peer review;
- technical demonstration;
- recruitment, interview, and portfolio evaluation;
- non-commercial technical evaluation; and
- reproducible demonstration with selected publishable examples.

## Public technical boundary

The public repository contains the selected **single-cloud / no-stitch Research Demo GUI** and the files required to review, build, package, and demonstrate that public software.

The public repository does not include the complete research or engineering system. In particular, the following may remain outside the public repository:

- multi-cloud stitching software;
- CLI/batch validation infrastructure;
- complete stability-test outputs;
- complete experimental or project-specific datasets;
- internal interfaces and deployment integration;
- unpublished research components; and
- other project-specific engineering material.

Selected point-cloud examples may be published only when redistribution is permitted. Such examples demonstrate use of the public software and do not constitute the complete dataset behind associated research.

## What this repository is not

The repository should not be interpreted as:

- the complete engineering system;
- the complete deliverable of an associated research programme;
- the complete source tree or algorithm-development history;
- the complete experimental or industrial dataset;
- a production-certified industrial deployment; or
- an official software release by a funding programme, university, laboratory, company, project consortium, or collaborating organization.

## Funding acknowledgement

Funding acknowledgements belong to the associated paper or research output and should follow the exact wording and project number required by the relevant project documentation and publication venue.

The existence of this public Research Demo does not by itself state or imply that it is an official release on behalf of a funding body, project consortium, institution, or collaborating organization.
'@
Write-Utf8NoBom (Join-Path $repo 'docs\RESEARCH_DEMO_SCOPE.md') $scope

$reviewer = @'
# Reviewer Guide — Research Demo / Development Preview

> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

This public repository intentionally contains only the selected **single-cloud / no-stitch Research Demo GUI**. It is not the complete engineering system, complete project deliverable, complete dataset, or an official software release of any funding programme or organization.

## Fast source-review entry points

- `apps/no_stitch/main.cpp`
- `apps/no_stitch/ZhuChuangKou_Window.*`
- `apps/no_stitch/HoleShibie_Recognition.*`
- `apps/no_stitch/HoleFaXian_Normal.h`
- `apps/no_stitch/HoleJihe_Geometry.*`
- `apps/no_stitch/HoleFenxi_Analysis.*`
- `apps/no_stitch/HoleWeizi_Pose.h`

## Fast execution path

For execution without a development environment, use the standalone Setup.exe attached to the GitHub Release. If public example point clouds are present under `examples/`, they may be used to exercise the demonstration workflow.

## Public/non-public boundary

See [`RESEARCH_DEMO_SCOPE.md`](RESEARCH_DEMO_SCOPE.md).
'@
Write-Utf8NoBom (Join-Path $repo 'docs\REVIEWER_GUIDE.md') $reviewer

$buildDoc = @'
# Windows Source Build — Research Demo / Development Preview

This repository contains only the selected **no-stitch Research Demo GUI**.

Reference development environment:

- Windows 10/11 x64
- Visual Studio 2022 / MSVC v143
- Qt 6.5.3 MSVC x64
- PCL 1.15.1 Windows bundle

## Build

1. Copy `apps/no_stitch/Local.Build.props.example` to `apps/no_stitch/Local.Build.props`.
2. Edit `PCL_ROOT` if your local PCL path differs.
3. Open `apps/no_stitch/RobustHoleMetrology_NoStitch.sln`.
4. Select `Release | x64` and rebuild.

`Local.Build.props` is intentionally ignored by Git.

A successful build verifies source/build integration only. It does not by itself establish metrology accuracy, production certification, or completeness of the associated research project.

For the public demonstration boundary, see [`RESEARCH_DEMO_SCOPE.md`](RESEARCH_DEMO_SCOPE.md).
'@
Write-Utf8NoBom (Join-Path $repo 'docs\BUILD_WINDOWS.md') $buildDoc

$releaseDoc = @'
# Research Demo Windows Release Workflow

## Scope

The public binary is a **Research Demo / Development Preview** containing only the selected **no-stitch** GUI implementation.

It does not represent the complete engineering system, complete research-project deliverables, complete experimental datasets, an industrial deployment package, or an official software release of any funding programme or organization.

## One-click build

Double-click:

```text
01_BUILD_SETUP_ONECLICK.bat
```

The build/package script:

1. runs repository safety checks;
2. locates the required Visual Studio/MSBuild, Qt, and PCL development environment;
3. rebuilds the no-stitch GUI as `Release | x64`;
4. creates a clean deployment directory;
5. deploys Qt runtime DLLs/plugins;
6. collects the required PCL/VTK and related runtime dependencies;
7. includes the OpenNI2 runtime tree when required;
8. bundles the Microsoft Visual C++ Redistributable installer;
9. copies available third-party license/notice material; and
10. builds a single Windows installer with Inno Setup.

Expected output:

```text
out/release/RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

Only the publisher/developer machine needs the full development toolchain. End users should install the generated Setup.exe.

## Clean-machine test before release

Before public distribution, test the generated installer on a clean Windows 10/11 x64 VM or PC without Visual Studio, Qt, or PCL installed. Confirm at minimum:

- installation completes;
- the application starts;
- the VTK 3D view initializes normally;
- supported point-cloud files load;
- the public recognition/measurement workflow runs;
- overlays and export behave as expected; and
- uninstall completes.

A successful packaging test verifies deployability only; it does not replace quantitative metrology validation or imply production certification.

## GitHub source + Release upload

After the clean-machine test, use:

```text
02_UPLOAD_GITHUB_ONECLICK.bat
```

or upload the Setup.exe manually through the GitHub Releases web page.

The GitHub Release title and notes identify the package as a **Research Demo / Development Preview**. Only the standalone Setup.exe should be required by an end user.

For public/non-public scope, see [`RESEARCH_DEMO_SCOPE.md`](RESEARCH_DEMO_SCOPE.md).
'@
Write-Utf8NoBom (Join-Path $repo 'docs\PRODUCT_RELEASE.md') $releaseDoc

$examplesReadme = @'
# Public demonstration examples

This directory is reserved for selected point-cloud examples that are safe and permitted to redistribute with the public **Research Demo / Development Preview**.

The public examples are intended only to demonstrate the workflow of the repository. They are not the complete experimental dataset, project-specific dataset, or industrial validation database associated with the underlying research.

Recommended layout:

```text
examples/
├─ normal/
│  └─ sample_normal.pcd
├─ partial/
│  └─ sample_partial.pcd
└─ severe/
   └─ sample_severe.pcd
```

Only publish data for which redistribution is permitted. Keep individual files comfortably below GitHub's regular file-size limits; use a separate approved data-distribution mechanism if a legitimate public dataset is too large for ordinary Git storage.
'@
Write-Utf8NoBom (Join-Path $repo 'examples\README.md') $examplesReadme

# Ensure the Windows installer presents itself as a Research Demo.
$iss = Join-Path $repo 'product\installer\RobustHoleMetrology.iss'
if (Test-Path $iss) {
    $t = [IO.File]::ReadAllText($iss)
    $t = [regex]::Replace($t, '(?m)^#define MyAppName ".*"$', '#define MyAppName "Robust Hole Metrology - Research Demo"')
    $t = [regex]::Replace($t, '(?m)^AppPublisher=.*$', 'AppPublisher=Robust Hole Metrology Research Demo')
    $t = [regex]::Replace($t, '(?m)^DefaultDirName=.*$', 'DefaultDirName={autopf}\Robust Hole Metrology Research Demo')
    $t = [regex]::Replace($t, '(?m)^DefaultGroupName=.*$', 'DefaultGroupName=Robust Hole Metrology Research Demo')
    Write-Utf8NoBom $iss $t
}

# Rewrite GitHub Release title/notes while preserving the proven upload logic.
$publishPath = Join-Path $repo 'scripts\publish_release.ps1'
if (Test-Path $publishPath) {
    $p = [IO.File]::ReadAllText($publishPath)
    $start = $p.IndexOf('$notes =')
    $end = $p.IndexOf('$oldPreference =', [Math]::Max(0,$start))
    if ($start -ge 0 -and $end -gt $start) {
        $releaseNotes = @(
            "`$notes = @'",
            '**Research Demo / Development Preview**',
            '',
            'Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.',
            '',
            'The public repository contains only the demonstration-oriented software components and selected reproducible examples.',
            '',
            'This Windows x64 release packages the selected no-stitch Research Demo GUI as a standalone installer. End users do not need a separate Visual Studio, Qt, PCL, or VTK development environment.',
            '',
            'This release does not represent the complete engineering system, complete research-project deliverables, complete experimental datasets, industrial deployment package, or an official software release of any funding programme, institution, laboratory, company, or project consortium.',
            "'@"
        ) -join "`n"
        $p = $p.Substring(0,$start) + $releaseNotes + "`n" + $p.Substring($end)
    }
    $p = [regex]::Replace($p, '--title\s+"Robust Hole Metrology[^\"]*\$version"', '--title "Robust Hole Metrology - Research Demo $version"')
    Write-Utf8NoBom $publishPath $p
}

# Clean public-facing wording in logs/changelog only; do not rename algorithm symbols.
$changelog = Join-Path $repo 'CHANGELOG.md'
if (Test-Path $changelog) {
    $c = [IO.File]::ReadAllText($changelog)
    $c = $c.Replace('algorithm/GUI production source','algorithm/GUI application source')
    $c = $c.Replace('validated no-stitch production GUI only','selected no-stitch Research Demo GUI only')
    $c = $c.Replace('Production `.cpp/.h/.ui/.qrc` source','Application `.cpp/.h/.ui/.qrc` source')
    $c = $c.Replace('No production GUI source or measurement logic changed.','No application GUI source or measurement logic changed.')
    $c = $c.Replace('No production GUI/algorithm source was changed.','No application GUI/algorithm source was changed.')
    Write-Utf8NoBom $changelog $c
}

$preflight = Join-Path $repo 'scripts\preflight.ps1'
if (Test-Path $preflight) {
    $q = [IO.File]::ReadAllText($preflight)
    $q = $q.Replace('Machine-specific absolute path in production/build file:', 'Machine-specific absolute path in application/build file:')
    $q = $q.Replace('Validation-only token inside production GUI:', 'Validation-only token inside public GUI:')
    Write-Utf8NoBom $preflight $q
}

Write-Host ''
Write-Host '[PASS] Public repository positioning updated.' -ForegroundColor Green
Write-Host '[PASS] README, LICENSE, reviewer/build/release docs and GitHub Release wording updated.' -ForegroundColor Green
Write-Host '[PASS] No apps/no_stitch algorithm source was edited by this updater.' -ForegroundColor Green
Write-Host ''
Write-Host 'Next:' -ForegroundColor Cyan
Write-Host '  - Run 04_SYNC_RESEARCH_DEMO_SOURCE.bat to push the public wording changes.'
Write-Host '  - Or run 05_APPLY_AND_SYNC_RESEARCH_DEMO_ONECLICK.bat next time for both steps.'
Write-Host '  - Rebuild Setup.exe only if you want refreshed installer metadata/binary.'
