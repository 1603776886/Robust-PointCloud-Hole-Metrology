# Changelog

## 0.1.0-dev — product packaging baseline

- Public repository reduced to the validated no-stitch production GUI only.
- Added one-click Release x64 build and standalone Windows Setup.exe packaging.
- Added automatic Qt deployment and PCL/VTK runtime dependency closure collection.
- Added official Microsoft Visual C++ Redistributable bundling.
- Added safe source sync and single-asset GitHub Release upload.
- Production `.cpp/.h/.ui/.qrc` source is unchanged from the selected no-stitch public baseline.

## FIX4 packaging script correction

- Fixed Inno Setup auto-detection after `winget` installation: installer console output can no longer be mistaken for the `ISCC.exe` path.
- Added fixed-path, registry, PATH, and last-resort `ISCC.exe` discovery plus caller-side path validation.
- No production GUI source or measurement logic changed.
