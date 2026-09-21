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