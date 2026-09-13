# Product release workflow (no-stitch only)

## Goal

The public release contains one product only: the validated **no-stitch** GUI. End users receive one Windows x64 installer EXE and do not install the development toolchain.

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

A successful build only proves packaging/build integrity; it does not replace metrology validation.

## GitHub source + release upload

After the clean-machine test, double-click:

```text
02_UPLOAD_GITHUB_ONECLICK.bat
```

The script safely commits/pushes the repository without force-pushing and creates the GitHub Release for the version in `product/VERSION.txt`. Only the single Setup.exe is uploaded as the release asset.

GitHub CLI (`gh`) is used for the Release operation. If missing, the script attempts to install it through `winget`; if not authenticated, the normal browser authentication flow is opened.

For the very first publish, create an **empty** GitHub repository yourself. Do not pre-create README, `.gitignore`, or a license on GitHub because they already exist locally.

## Releasing a new version

Change only:

```text
product/VERSION.txt
```

For example, change `0.1.0-dev` to `0.1.1-dev`, rebuild, test, and upload. The release uploader refuses to overwrite an existing version tag/release; this prevents accidental replacement of a previously reviewed binary.
