# Runtime Package Documentation

[Back to README](../README.md)

This directory is reserved for runtime packages. These are shared-library modules loaded by EvoEngine while an app is running. They are separate from Plugins, which live under `EvoEngine_Plugins` and are linked at build time.

| Package | Documentation |
| --- | --- |
| _None yet_ | Runtime packages will be documented here as they are added. |

Runtime package support is controlled by `EVOENGINE_ENABLE_RUNTIME_PACKAGES`. Individual package targets use options such as `EVOENGINE_ENABLE_<PackageName>_PACKAGE`.

Packages are copied to the app runtime `Packages` folder after build and installed to `bin/Packages`. On Windows, EvoEngine loads packages from a shadow copy so the original package DLL can usually be rebuilt while the app is still open.
