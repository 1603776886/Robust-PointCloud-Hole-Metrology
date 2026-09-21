# Windows source build

This repository is a **Research Demo / Development Preview** and contains only the selected no-stitch GUI implementation.

Reference environment: Windows x64, Visual Studio 2022/MSVC v143, Qt 6.5.3 MSVC x64, PCL 1.15.1 Windows bundle.

1. Copy `apps/no_stitch/Local.Build.props.example` to `apps/no_stitch/Local.Build.props`.
2. Edit `PCL_ROOT` if your local PCL path differs.
3. Open `apps/no_stitch/RobustHoleMetrology_NoStitch.sln`.
4. Select `Release | x64` and rebuild.

`Local.Build.props` is intentionally ignored by Git.

For the scope of the public demonstration, see `RESEARCH_DEMO_SCOPE.md`.
