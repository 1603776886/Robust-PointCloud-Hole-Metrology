# Third-party runtime notices

Robust Hole Metrology is built with third-party software. Those components remain under their own upstream licenses; this repository's `LICENSE` does not relicense them.

The reference Windows build uses:

- Qt 6.5.3: Core, GUI, Widgets, OpenGL/OpenGLWidgets
- Point Cloud Library (PCL) 1.15.1
- VTK 9.4 from the reference PCL Windows bundle
- Boost 1.87 from the reference PCL Windows bundle
- Eigen3, FLANN, OpenNI2, Qhull and lz4 from the reference PCL Windows bundle
- Microsoft Visual C++ Redistributable (official Microsoft installer)

`01_BUILD_SETUP_ONECLICK.bat` collects the runtime binaries from the publisher's installed Qt/PCL environment. It also copies license/notice files it can find into the packaged application's `licenses` directory. Review that generated license directory before public redistribution, particularly if the local dependency versions differ from the reference versions above.

Qt is deployed dynamically with `windeployqt`; the product source is not statically linked into Qt. The exact obligations depend on the Qt modules/version and the license under which the publisher obtained Qt. The publisher is responsible for confirming the applicable upstream license terms before distribution.
