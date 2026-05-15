# Runtime Package Documentation

[Back to README](../README.md)

This directory is reserved for runtime packages. These are shared-library modules loaded by EvoEngine while an app is running. They are separate from Services, which live under `EvoEngine_Services` and are linked at build time.

| Package | Documentation |
| --- | --- |
| BillboardClouds | [BillboardClouds/README.md](BillboardClouds/README.md) |
| EcoSysLab | [EcoSysLab/README.md](EcoSysLab/README.md) |
| DigitalAgriculture | [DigitalAgriculture/README.md](DigitalAgriculture/README.md) |
| DatasetGeneration | [DatasetGeneration/README.md](DatasetGeneration/README.md) |
| Universe | [Universe/README.md](Universe/README.md) |
| Gpr | [Gpr/README.md](Gpr/README.md) |
| LogGrading | [LogGrading/README.md](LogGrading/README.md) |
| LogScanning | [LogScanning/README.md](LogScanning/README.md) |
| MeshRepair | [MeshRepair/README.md](MeshRepair/README.md) |
| TextureBaking | [TextureBaking/README.md](TextureBaking/README.md) |

Runtime package support is controlled by `EVOENGINE_ENABLE_RUNTIME_PACKAGES`. Individual package targets use options such as `EVOENGINE_ENABLE_<PackageName>_PACKAGE`, and current package targets are built by default on supported platforms.

Applications also do not load runtime packages during startup by default. Enable `ApplicationInitializationSettings::enable_runtime_packages` or use the runtime package manager UI/API when a workflow needs additional packages.

Package dependencies are declared in `PackageInfo.cmake` with `EVOENGINE_PACKAGE_DEPENDS`. Each package emits a sidecar `.evepackage` manifest so the runtime can scan dependency metadata before loading package libraries.

Packages are copied to the app runtime `Packages` folder after build and installed to `bin/Packages`. On Windows, EvoEngine loads packages from a shadow copy so the original package DLL can usually be rebuilt while the app is still open.
