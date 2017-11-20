# How to compile and run the CGAL Polyhedron demo with NURBS related plugins

## Prerequisites
  Use C/C++ compiler handling c++14 features.

## Install Qt
  ### Install Qt
    Install Qt (version > 5.7 adviced) from RPM (best), or from web binary, or from source (then you must be careful with what you are doing).
    Make sure that : Qt5Concurrent, Qt5Core, Qt5Gui, Qt5Network, Qt5Qml, Qt5Quick, Qt5Svg, Qt5Test, Qt5Widgets, Qt5Xml modules are available (development packages).

## Recover and compile dtk (Qt dependence)
  ### Recover dtk
    Clone git@github.com:d-tk/dtk.git.
  ### Configure and compile dtk
    Use root CMakeLists.txt as CMake directive to configure (make sure all paths to Qt modules are correct), then build.

## Recover and compile dtk-continuous-geometry (Qt and dtk dependence)
  ### Recover dtk-continuous-geometry
    Clone git@github.com:d-tk/dtk-continuous-geometry.git.
    Checkout branch : "develop".
  ### Compile dtk-continuous-geometry
    Use root CMakeLists.txt as CMake directive to configure, then build.

    More information is available in the root README.md.

## Recover and compile dtk-discrete-geometry (Qt and dtk dependence)
  ### Recover dtk-discrete-geometry
    Clone git@github.com:d-tk/dtk-discrete-geometry.git.
    Checkout branch : "develop".
  ### Compile dtk-discrete-geometry
    Use root CMakeLists.txt as CMake directive to configure, then build.

    More information is available in the root README.md.

## Recover CGAL (header only)
  ### Recover CGAL
    Clone git@github.com:CGAL/cgal.git.
    Add remote git@github.com:clebreto/cgal.git.
    Checkout nurbs-viewer-mesher.

## Recover Blaze (header only)
  ### Recover Blaze
    Clone git@bitbucket.org:blaze-lib/blaze.git.

## Recover and compile openNURBS
   ### Recover openNURBS
     Clone git@gitlab.inria.fr:tkloczko/opennurbs.git.
   ### Compile openNURBS
     Use root CMakeLists.txt as CMake directive to configure, then build.

## Recover and compile SISL
  ### Recover SISL
    git@github.com:SINTEF-Geometry/SISL.git
  ### Compile SISL
    Modify the root CMakeLists.txt to compile a shared lib : replace "ADD_LIBRARY(sisl ${sisl_SRCS})" with "ADD_LIBRARY(sisl SHARED ${sisl_SRCS})", to ease the compilation of plugins.
    Use root CMakeLists.txt as CMake directive to configure, then build.

## Install lapack and blas
  Install lapack and blas by RPM (development packages).

## Recover and compile dtk-nurbs-probing (Qt, dtk and dtk-continuous-geometry dependence)
  ### Recover dtk-nurbs-probing
    Clone git@gitlab.inria.fr:dream/dtk-nurbs-probing.git.
  ### Compile dtk-nurbs-probing
    Use root CMakeLists.txt as CMake directive to configure.

    Activate DTK_NURBS_PROBING_BUILD_MREP, let BLA_VENDOR empty to choose automatically the blas and lapack librairies, specify "Blaze_INDCLUDE_DIR".

## Recover and compile dtk-plugins-continuous-geometry (Qt, dtk and dtk-continuous-geometry dependence)
  ### Recover dtk-plugins-continuous-geometry
    Clone git@github.com:d-tk/dtk-plugins-continuous-geometry.git.
    Checkout branch : "develop".
  ### Compile dtk-plugins-continuous-geometry
    Use root CMakeLists.txt as CMake directive to configure : specify path to CGAL, Blaze, openNURBS, SISL, dtk-discrete-geometry, lapack and blas, then build.

## Compile CGAL demo
  ### Compile CGAL demo
  Configure CGAL : activate "WITH_demos", specify path to "dtk_DIR", "dtkContinuousGeometry_DIR", "dtkNurbsProbing_DIR", "Blaze_DIR".
  Compile the Polyhedron demo, "cad_meshing_plugin" and "cAD_initialization_plugin" and "cAD_refine_plugin"

# Complements
This is a cumbersome installation procedure. We aim at simplifying it as much as possible in the future, thanks to conda.
