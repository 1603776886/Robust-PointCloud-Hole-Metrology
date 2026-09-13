# Robust Point-Cloud Hole Metrology

Windows x64 desktop software for geometric inspection and measurement of holes from partial 3D point clouds.

This public repository intentionally contains **only the validated no-stitch production GUI**. The multi-cloud stitching GUI and CLI/batch validation harness are not included in this repository.

## For users: download and run

Normal users do **not** need Visual Studio, Qt, PCL, VTK, Boost, Eigen, OpenNI2, or any other development dependency.

Open the repository's **Releases** page, download the single Windows installer:

```text
RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

Run the installer and launch **Robust Hole Metrology** from the Start menu or optional desktop shortcut. The installer contains the application runtime dependencies and installs the official Microsoft Visual C++ runtime when required.

## Repository scope

```text
apps/no_stitch/                 validated production GUI source only
product/installer/              single-EXE Windows installer definition
scripts/build_setup.ps1         build + self-contained deployment + Setup.exe
scripts/publish_release.ps1     safe Git push + GitHub Release upload
01_BUILD_SETUP_ONECLICK.bat     publisher one-click build/package
02_UPLOAD_GITHUB_ONECLICK.bat   publisher one-click source/release upload
BUILD_AND_UPLOAD_ONECLICK.bat   build then upload in one pass
```

The production source under `apps/no_stitch/` remains separate from packaging code. Packaging does not alter the recognition/metrology implementation.

## Publisher workflow

The publisher/developer machine uses the existing reference toolchain:

- Windows 10/11 x64
- Visual Studio 2022 / MSVC v143
- Qt 6.5.3 MSVC 64-bit
- PCL 1.15.1 Windows bundle (including the compatible VTK/Boost/Eigen/FLANN/OpenNI2/Qhull/lz4 runtime set)

Then:

1. Double-click `01_BUILD_SETUP_ONECLICK.bat` to rebuild Release x64, collect runtime DLLs, deploy Qt plugins, bundle the official VC++ Redistributable, and create one installer EXE.
2. Test the generated installer on a clean Windows machine/VM.
3. Double-click `02_UPLOAD_GITHUB_ONECLICK.bat` to safely commit/push the source and create the GitHub Release containing the **single Setup.exe asset**.

Or double-click `BUILD_AND_UPLOAD_ONECLICK.bat` to run steps 1 and 3 consecutively.

See [`docs/PRODUCT_RELEASE.md`](docs/PRODUCT_RELEASE.md) for details.

## Build source manually

For source reviewers/developers, copy `apps/no_stitch/Local.Build.props.example` to `Local.Build.props`, set the local PCL root if needed, and build `Release | x64` in Visual Studio. Machine-local `Local.Build.props` is ignored by Git.

## Third-party software

This project dynamically uses Qt/PCL/VTK and related libraries. Their binaries and license notices in a generated installer remain under their respective upstream licenses. The repository's own `LICENSE` covers only project-authored material. See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## License

The project-authored source is distributed under the repository's source-available evaluation license for academic review, recruitment/portfolio evaluation, and personal non-commercial technical evaluation. See [`LICENSE`](LICENSE).
