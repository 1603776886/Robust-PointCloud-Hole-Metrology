# Robust Point-Cloud Hole Metrology

> **Research Demo / Development Preview**  
> Demonstration implementation of robust geometric metrology for incomplete 3D point clouds.  
> The public repository contains only the demonstration-oriented software components and selected reproducible examples.

A Windows x64 research demo for hole-feature recognition, geometric reconstruction, and dimensional metrology from incomplete 3D point clouds.

This repository is intentionally limited to the selected **single-cloud / no-stitch Research Demo GUI**. It does **not** represent the complete engineering system, complete research-project deliverables, complete experimental datasets, an industrial deployment package, or an official software release of any funding programme, institution, laboratory, company, or project consortium.

## What the demo includes

- 3D point-cloud import and visualization
- point-cloud preprocessing used by the public GUI workflow
- automatic hole-feature recognition
- geometric reconstruction for incomplete and severely partial hole boundaries
- hole-axis estimation
- upper/lower opening geometry measurement
- hole-depth measurement
- 3D visualization and result export

Multi-cloud stitching, CLI/batch validation infrastructure, project-specific datasets, internal interfaces, and other non-public research or engineering components are outside this public repository.

## Download and run

For normal users, download the Windows x64 installer from the repository's **Releases** page:

```text
RobustHoleMetrology-Dev-<version>-Setup-x64.exe
```

The packaged application is intended to run without requiring users to install the development toolchain such as Visual Studio, Qt, PCL, or VTK separately.

## Public repository layout

```text
apps/no_stitch/        Research Demo GUI source
examples/              selected public demonstration point clouds
                       and example notes, when available
docs/BUILD_WINDOWS.md  source-build instructions
LICENSE                project-authored source license
THIRD_PARTY_NOTICES.md third-party software notices
```

The GitHub source tree intentionally excludes local packaging, publishing, release-upload, CI housekeeping, and private development utilities. Those tools are not required to review or use the public Research Demo.

## Example point clouds


Suggested structure:

```text
1.1/1.2/1.3/1.4.pcd
```

## Build from source

Reference environment:

- Windows 10/11 x64
- Visual Studio 2022 / MSVC v143
- Qt 6.5.3 MSVC x64
- PCL 1.15.1 Windows bundle


## Third-party software

The application dynamically uses Qt, PCL, VTK, and related third-party libraries. Their binaries and notices remain governed by their respective upstream licenses. The repository's own `LICENSE` applies only to project-authored material.

See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Research / project boundary

This public repository is provided for academic communication, peer review, technical demonstration, recruitment/portfolio evaluation, and non-commercial technical evaluation.

It should not be interpreted as the complete deliverable of an associated research programme, the complete algorithm-development history, the complete experimental/industrial dataset, a production-certified industrial deployment, or an official software release by any funding agency or participating organization.

## License

Project-authored material is distributed under the repository's **Source-Available Evaluation License**. See [`LICENSE`](LICENSE).
