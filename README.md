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
