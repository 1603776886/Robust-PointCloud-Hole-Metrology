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
鈹溾攢 normal/
鈹? 鈹斺攢 sample_normal.pcd
鈹溾攢 partial/
鈹? 鈹斺攢 sample_partial.pcd
鈹斺攢 severe/
   鈹斺攢 sample_severe.pcd
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