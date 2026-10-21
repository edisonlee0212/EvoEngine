# Runtime Package Documentation

[Back to README](../README.md)

This directory contains runtime packages and their separate editor companions. These are shared-library modules loaded by EvoEngine while an app is running. They are separate from Services, which live under `EvoEngine_Services` and are linked at build time.

| Package | Documentation |
| --- | --- |
| BillboardClouds | [BillboardClouds/README.md](BillboardClouds/README.md) |
| EcoSysLab | [EcoSysLab/README.md](EcoSysLab/README.md) |
| DigitalAgriculture | [DigitalAgriculture/README.md](DigitalAgriculture/README.md) |
| DatasetGeneration | [DatasetGeneration/README.md](DatasetGeneration/README.md) |
| LSystem | [LSystem/README.md](LSystem/README.md) |
| Universe | [Universe/README.md](Universe/README.md) |
| MeshRepair | [MeshRepair/README.md](MeshRepair/README.md) |
| TextureBaking | [TextureBaking/README.md](TextureBaking/README.md) |

Runtime package support is controlled by `EVOENGINE_ENABLE_RUNTIME_PACKAGES`. Individual package targets use options such as `EVOENGINE_ENABLE_<PackageName>_PACKAGE`, and current package targets are built by default on supported platforms.

The SDK initialization default leaves package loading disabled. The generic editor enables packages named by the
project or demo profile, and exported players use the exported package list. Custom hosts enable
`ApplicationInitializationSettings::enable_runtime_packages` and choose startup packages or use the manager API.

Package dependencies are declared in `PackageInfo.cmake` with `EVOENGINE_PACKAGE_DEPENDS`. Runtime libraries contain simulation/types; `Editor/` companions provide inspection and authoring UI when the editor is present. Each package emits a sidecar `.evepackage` manifest so the runtime can scan dependency metadata before loading package libraries.

Packages are copied to the app runtime `Packages` folder after build and installed to `bin/Packages`. On Windows, EvoEngine loads packages from a shadow copy so the original package DLL can usually be rebuilt while the app is still open.
