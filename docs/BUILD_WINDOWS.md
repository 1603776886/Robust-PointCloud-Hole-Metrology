# Windows Source Build 鈥?Research Demo / Development Preview

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