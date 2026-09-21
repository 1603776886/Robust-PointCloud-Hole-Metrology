# Changelog
## Research Demo public-positioning update

- Public repository is explicitly labeled **Research Demo / Development Preview**.
- README, reviewer/build/release documentation, installer display name, and GitHub Release metadata now use the same demonstration-oriented positioning.
- Public scope explicitly states that the repository is not the complete engineering system, complete project deliverable, complete dataset, or an official software release of a funding programme.
- No `apps/no_stitch` algorithm/GUI application source was changed by this positioning update.

## 0.1.0-dev 鈥?product packaging baseline

- Public repository reduced to the selected no-stitch Research Demo GUI only.
- Added one-click Release x64 build and standalone Windows Setup.exe packaging.
- Added automatic Qt deployment and PCL/VTK runtime dependency closure collection.
- Added official Microsoft Visual C++ Redistributable bundling.
- Added safe source sync and single-asset GitHub Release upload.
- Application `.cpp/.h/.ui/.qrc` source is unchanged from the selected no-stitch public baseline.

## FIX4 packaging script correction

- Fixed Inno Setup auto-detection after `winget` installation: installer console output can no longer be mistaken for the `ISCC.exe` path.
- Added fixed-path, registry, PATH, and last-resort `ISCC.exe` discovery plus caller-side path validation.
- No application GUI source or measurement logic changed.
