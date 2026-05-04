# Plugin Documentation

[Back to README](../README.md)

This directory documents EvoEngine Plugins separately from the SDK overview. Each Plugin page describes the Plugin's purpose, current build status, important source files, SDK extension points, and notes for future work.

| Plugin | Documentation |
| --- | --- |
| EcoSysLab | [EcoSysLab/README.md](EcoSysLab/README.md) |
| DigitalAgriculture | [DigitalAgriculture/README.md](DigitalAgriculture/README.md) |
| DatasetGeneration | [DatasetGeneration/README.md](DatasetGeneration/README.md) |
| Universe | [Universe/README.md](Universe/README.md) |
| BillboardClouds | [BillboardClouds/README.md](BillboardClouds/README.md) |
| TextureBaking | [TextureBaking/README.md](TextureBaking/README.md) |
| MeshRepair | [MeshRepair/README.md](MeshRepair/README.md) |
| Gpr | [Gpr/README.md](Gpr/README.md) |
| LogGrading | [LogGrading/README.md](LogGrading/README.md) |
| LogScanning | [LogScanning/README.md](LogScanning/README.md) |
| CudaModule | [CudaModule/README.md](CudaModule/README.md) |
| PhysXPhysics | [PhysXPhysics/README.md](PhysXPhysics/README.md) |

Plugins are registered from `EvoEngine_Plugins/CMakeLists.txt`. Default registration is not the same as directory presence: some Plugin directories are present but disabled or gated by platform/optional SDKs.
